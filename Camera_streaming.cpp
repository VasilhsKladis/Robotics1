#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// === WiFi Config ===
const char* ssid = "AP_wifi";
const char* password = "Gus2003!";

// === UDP Config ===
const char* udp_host = "192.168.137.1";  // <- PC IP address
const int udp_port = 12345;
WiFiUDP udp;

// === Camera Pin Config (AI Thinker) ===
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QQVGA;  // 160x120
    config.jpeg_quality = 10;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    ESP.restart();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  startCamera();

  udp.begin(udp_port);  // Not strictly needed for sending
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(100);
    return;
  }

  // Frame too big? Split into chunks (optional)
  int maxPacketSize = 1400;  // UDP MTU safe size
  int totalBytes = fb->len;
  int bytesSent = 0;

  while (bytesSent < totalBytes) {
    int chunkSize = min(maxPacketSize, totalBytes - bytesSent);
    udp.beginPacket(udp_host, udp_port);
    udp.write(fb->buf + bytesSent, chunkSize);
    udp.endPacket();
    bytesSent += chunkSize;
    delay(2);  // Helps reduce packet loss
  }

  Serial.printf("Sent frame of %d bytes in %d packets\n", fb->len, (fb->len + maxPacketSize - 1) / maxPacketSize);
  esp_camera_fb_return(fb);

  delay(50);  // ~20 fps
}
