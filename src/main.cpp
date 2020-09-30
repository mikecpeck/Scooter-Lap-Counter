#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
#include <Fonts/xlmonoalt32pt7b.h>
//https://eloquentarduino.github.io/2020/01/motion-detection-with-esp32-cam-only-arduino-version/

#define CAMERA_MODEL_AI_THINKER

#include "esp_camera.h"
#include "camera_pins.h"

#define FRAME_SIZE FRAMESIZE_QVGA
#define WIDTH 320
#define HEIGHT 240
#define BLOCK_SIZE 10
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)
#define BLOCK_DIFF_THRESHOLD 0.3
#define IMAGE_DIFF_THRESHOLD 0.35
#define LED_BUILTIN 4
#define DEBUG 0
#define scWIDTH 128   // OLED display width, in pixels
#define scHEIGHT 64   // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)

uint16_t prev_frame[H][W] = {0};
uint16_t current_frame[H][W] = {0};

bool setup_camera(framesize_t);
bool capture_still();
bool motion_detect();
void update_frame();
void print_frame(uint16_t frame[H][W]);

const char *ssid = "Stump";
const char *password = "";

int laps = 0;

Adafruit_SH1106 display(14, 15);

void setup()
{
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);

  // put your setup code here, to run once:
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);

  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.setHostname("scooter");
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println(setup_camera(FRAME_SIZE) ? "OK" : "ERR INIT");

  display.clearDisplay();
  display.display();
  display.setRotation(0);
  //display.setTextSize(3);
  display.setTextWrap(false);
  display.setFont(&xlmonoalt32pt7b);
  //display.setTextSize(fontsize);
  display.setTextColor(WHITE);
  display.setCursor(0, 40);
  display.println("lets go");

  display.display();

  pinMode(LED_BUILTIN, OUTPUT); //Specify that LED pin is output
}

void loop()
{
  // put your main code here, to run repeatedly:

  if (!capture_still())
  {
    Serial.println("Failed capture");
    delay(3000);

    return;
  }

  if (motion_detect())
  {
    Serial.println("Motion detected");

    laps++;
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    display.clearDisplay();
    display.display();
    display.setRotation(0);
    //display.setTextSize(3);
    display.setTextWrap(false);
    display.setFont(&xlmonoalt32pt7b);
    //display.setTextSize(fontsize);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    display.println(laps);

    display.display();
    delay(2000);
  }

  update_frame();
  Serial.println("=================");

  ArduinoOTA.handle();
}

bool setup_camera(framesize_t frameSize)
{
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = frameSize;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  bool ok = esp_camera_init(&config) == ESP_OK;

  sensor_t *sensor = esp_camera_sensor_get();
  sensor->set_framesize(sensor, frameSize);

  return ok;
}

/**
 * Capture image and do down-sampling
 */
bool capture_still()
{
  camera_fb_t *frame_buffer = esp_camera_fb_get();

  if (!frame_buffer)
    return false;

  // set all 0s in current frame
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      current_frame[y][x] = 0;

  // down-sample image in blocks
  for (uint32_t i = 0; i < WIDTH * HEIGHT; i++)
  {
    const uint16_t x = i % WIDTH;
    const uint16_t y = floor(i / WIDTH);
    const uint8_t block_x = floor(x / BLOCK_SIZE);
    const uint8_t block_y = floor(y / BLOCK_SIZE);
    const uint8_t pixel = frame_buffer->buf[i];
    const uint16_t current = current_frame[block_y][block_x];

    // average pixels in block (accumulate)
    current_frame[block_y][block_x] += pixel;
  }

  // average pixels in block (rescale)
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;

#if DEBUG
  Serial.println("Current frame:");
  print_frame(current_frame);
  Serial.println("---------------");
#endif

  return true;
}

/**
 * Compute the number of different blocks
 * If there are enough, then motion happened
 */
bool motion_detect()
{
  uint16_t changes = 0;
  const uint16_t blocks = (WIDTH * HEIGHT) / (BLOCK_SIZE * BLOCK_SIZE);

  for (int y = 0; y < H; y++)
  {
    for (int x = 0; x < W; x++)
    {
      float current = current_frame[y][x];
      float prev = prev_frame[y][x];
      float delta = abs(current - prev) / prev;

      if (delta >= BLOCK_DIFF_THRESHOLD)
      {
#if DEBUG
        Serial.print("diff\t");
        Serial.print(y);
        Serial.print('\t');
        Serial.println(x);
#endif

        changes += 1;
      }
    }
  }

  Serial.print("Changed ");
  Serial.print(changes);
  Serial.print(" out of ");
  Serial.println(blocks);

  return (1.0 * changes / blocks) > IMAGE_DIFF_THRESHOLD;
}

/**
 * Copy current frame to previous
 */
void update_frame()
{
  for (int y = 0; y < H; y++)
  {
    for (int x = 0; x < W; x++)
    {
      prev_frame[y][x] = current_frame[y][x];
    }
  }
}

/**
 * For serial debugging
 * @param frame
 */
void print_frame(uint16_t frame[H][W])
{
  for (int y = 0; y < H; y++)
  {
    for (int x = 0; x < W; x++)
    {
      Serial.print(frame[y][x]);
      Serial.print('\t');
    }

    Serial.println();
  }
}