#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// WiFi config
const char* ssid = "AP_wifi";
const char* password = "Gus2003!";

// ESP-NOW config 
uint8_t gyroMac[] = {0xF4,0x65,0x0B,0x54,0x58,0xC4};

// Dynamic channel - will be discovered from WiFi AP
uint8_t DISCOVERED_WIFI_CHANNEL = 0;

// IMU data structure (matches sender)
typedef struct {
  float gx, gy, gz;  // Gyroscope (rad/s)
  float ax, ay, az;  // Accelerometer (g)
} SensorData;

// Motor control structure
typedef struct {
  int32_t speed;
  int32_t angle;
} ControlCommand;

// FreeRTOS objects
SemaphoreHandle_t dataMutex;
QueueHandle_t udpCommandQueue;
QueueHandle_t slamDataQueue;
TaskHandle_t udpTaskHandle = NULL;
TaskHandle_t slamSenderTaskHandle = NULL;
TaskHandle_t monitorTaskHandle = NULL;

// Watchdog and connection management
volatile bool wifiHealthy = false;
volatile bool espnowHealthy = false;
volatile unsigned long lastImuReceived = 0;
volatile unsigned long lastWifiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 5000;
const unsigned long IMU_TIMEOUT = 3000;
const unsigned long PC_TIMEOUT = 15000; // Increased timeout

// Shared data with atomic access
struct SharedData {
  SensorData imu = {0};
  ControlCommand motor = {0, 90};
  IPAddress pcAddress;
  uint16_t pcPort = 4210;
  bool pcConnected = false;
  unsigned long lastPcContact = 0;
} sharedData;

// Global UDP objects to prevent recreation
WiFiUDP udpReceiver;
WiFiUDP udpSender;

// SLAM Packet Structure
#pragma pack(push, 1)
typedef struct {
  uint32_t timestamp;
  float imu[6];        // [0-2]: gyro x,y,z, [3-5]: accel x,y,z
  int16_t speed;
  int16_t angle;
} SlamPacket;
#pragma pack(pop)

// Function prototypes
uint8_t discoverWiFiChannel();
void initializeWiFi();
void initializeESPNOW();
void udpTask(void *pvParameters);
void slamSenderTask(void *pvParameters);
void monitorTask(void *pvParameters);
void sendSlamPacket(IPAddress address, uint16_t port);
void healthCheck();
void printMac(const uint8_t* mac);

void printMac(const uint8_t* mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X", 
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// CRITICAL: Discover which channel the WiFi AP is using
uint8_t discoverWiFiChannel() {
  Serial.println("Scanning for WiFi AP to discover channel...");
  
  // Start WiFi scan
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  int n = WiFi.scanNetworks();
  Serial.printf("Found %d networks\n", n);
  
  uint8_t targetChannel = 0;
  int32_t bestRSSI = -100;
  
  for (int i = 0; i < n; i++) {
    String foundSSID = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    uint8_t channel = WiFi.channel(i);
    
    Serial.printf("Network %d: %s (Ch: %d, RSSI: %d)\n", 
                 i, foundSSID.c_str(), channel, rssi);
    
    // Look for our target AP
    if (foundSSID.equals(ssid)) {
      if (rssi > bestRSSI) {  // In case there are multiple APs with same name
        targetChannel = channel;
        bestRSSI = rssi;
      }
    }
  }
  
  WiFi.scanDelete();
  
  if (targetChannel == 0) {
    Serial.printf("ERROR: Could not find WiFi AP '%s'\n", ssid);
    Serial.println("Available networks:");
    WiFi.scanNetworks();
    for (int i = 0; i < WiFi.scanComplete(); i++) {
      Serial.printf("  - %s (Ch: %d)\n", WiFi.SSID(i).c_str(), WiFi.channel(i));
    }
    WiFi.scanDelete();
    return 1; // Default fallback
  }
  
  Serial.printf("SUCCESS: Found '%s' on channel %d (RSSI: %d)\n", 
               ssid, targetChannel, bestRSSI);
  
  return targetChannel;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== Starting Dynamic Channel Base Station ===");
  
  // Initialize FreeRTOS objects with error checking
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    Serial.println("FATAL: Failed to create mutex");
    ESP.restart();
  }
  
  udpCommandQueue = xQueueCreate(5, sizeof(ControlCommand));
  slamDataQueue = xQueueCreate(10, sizeof(SlamPacket));
  
  if (udpCommandQueue == NULL || slamDataQueue == NULL) {
    Serial.println("FATAL: Failed to create queues");
    ESP.restart();
  }
  
  // STEP 1: Discover the WiFi AP's channel
  DISCOVERED_WIFI_CHANNEL = discoverWiFiChannel();
  Serial.printf("=== Using discovered channel: %d ===\n", DISCOVERED_WIFI_CHANNEL);
  
  // STEP 2: Connect to WiFi on the discovered channel
  initializeWiFi();
  
  // STEP 3: Initialize ESP-NOW on the same channel
  initializeESPNOW();
  
  // Create tasks with error checking
  BaseType_t result1 = xTaskCreatePinnedToCore(
    udpTask, "UDP Task", 4096, NULL, 3, &udpTaskHandle, 1
  );
  
  BaseType_t result2 = xTaskCreatePinnedToCore(
    slamSenderTask, "SLAM Sender", 4096, NULL, 2, &slamSenderTaskHandle, 1
  );
  
  BaseType_t result3 = xTaskCreatePinnedToCore(
    monitorTask, "Monitor Task", 3072, NULL, 1, &monitorTaskHandle, 1
  );
  
  if (result1 != pdPASS || result2 != pdPASS || result3 != pdPASS) {
    Serial.println("FATAL: Failed to create tasks");
    ESP.restart();
  }
  
  Serial.println("=== System Initialized Successfully ===");
}

void loop() {
  healthCheck();
  
  // Monitor channel stability
  static unsigned long lastChannelCheck = 0;
  if (millis() - lastChannelCheck > 10000) { // Check every 10 seconds
    uint8_t currentChannel;
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&currentChannel, &secondChan);
    
    if (currentChannel != DISCOVERED_WIFI_CHANNEL) {
      Serial.printf("Channel drift detected! Current: %d, Expected: %d\n", 
                   currentChannel, DISCOVERED_WIFI_CHANNEL);
      // Try to recover by reconnecting
      WiFi.disconnect();
      delay(1000);
      initializeWiFi();
      initializeESPNOW();
    }
    lastChannelCheck = millis();
  }
  
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void healthCheck() {
  unsigned long now = millis();
  
  // Check WiFi health
  if (now - lastWifiCheck > WIFI_CHECK_INTERVAL) {
    wifiHealthy = (WiFi.status() == WL_CONNECTED);
    if (!wifiHealthy) {
      Serial.println("WARNING: WiFi disconnected, attempting reconnection");
      initializeWiFi();  // This will rediscover channel if needed
    }
    lastWifiCheck = now;
  }
  
  // Check ESP-NOW health (IMU data freshness)
  espnowHealthy = (now - lastImuReceived < IMU_TIMEOUT);
  
  // System restart if both connections are dead for too long
  static unsigned long bothDeadStart = 0;
  if (!wifiHealthy && !espnowHealthy) {
    if (bothDeadStart == 0) {
      bothDeadStart = now;
    } else if (now - bothDeadStart > 30000) { // 30 seconds
      Serial.println("FATAL: Both connections dead, restarting");
      ESP.restart();
    }
  } else {
    bothDeadStart = 0;
  }
}

void initializeWiFi() {
  Serial.printf("Connecting to WiFi on channel %d...\n", DISCOVERED_WIFI_CHANNEL);
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(500);
  
  Serial.print("Base MAC: ");
  uint8_t baseMac[6];
  WiFi.macAddress(baseMac);
  printMac(baseMac);
  Serial.println();
  
  // Set hostname
  WiFi.setHostname("ESP32-BaseStation");
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.printf("Connecting to %s", ssid);
  
  // Connection attempt
  unsigned long wifiTimeout = millis() + 30000; // 30 seconds
  int attempts = 0;
  
  while (WiFi.status() != WL_CONNECTED && millis() < wifiTimeout) {
    delay(500);
    Serial.print(".");
    attempts++;
    
    if (attempts % 10 == 0) {
      Serial.printf("\nAttempt %d, Status: %d", attempts, WiFi.status());
    }
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nERROR: WiFi connection failed!");
    // Try to rediscover the channel in case AP moved
    Serial.println("Rediscovering WiFi channel...");
    DISCOVERED_WIFI_CHANNEL = discoverWiFiChannel();
    wifiHealthy = false;
    return;
  }
  
  // Verify we're on the expected channel
  uint8_t currentChannel;
  wifi_second_chan_t secondChan;
  esp_wifi_get_channel(&currentChannel, &secondChan);
  
  if (currentChannel != DISCOVERED_WIFI_CHANNEL) {
    Serial.printf("WARNING: Connected on channel %d, expected %d\n", 
                 currentChannel, DISCOVERED_WIFI_CHANNEL);
    DISCOVERED_WIFI_CHANNEL = currentChannel;  // Update our expectation
  }
  
  // Configure WiFi for stability
  WiFi.setSleep(false);
  wifi_config_t conf;
  esp_wifi_get_config(WIFI_IF_STA, &conf);
  conf.sta.listen_interval = 1;
  esp_wifi_set_config(WIFI_IF_STA, &conf);
  
  wifiHealthy = true;
  Serial.printf("\nWiFi Connected! IP: %s | RSSI: %d dBm | Channel: %d\n", 
               WiFi.localIP().toString().c_str(), WiFi.RSSI(), currentChannel);
}

void initializeESPNOW() {
  Serial.printf("Initializing ESP-NOW on channel %d...\n", DISCOVERED_WIFI_CHANNEL);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: ESP-NOW init failed");
    ESP.restart();
  }

  // Ensure ESP-NOW uses the same channel as WiFi
  uint8_t currentChannel;
  wifi_second_chan_t secondChan;
  esp_wifi_get_channel(&currentChannel, &secondChan);
  
  Serial.printf("Current WiFi channel: %d, Target: %d\n", 
               currentChannel, DISCOVERED_WIFI_CHANNEL);
  
  if (currentChannel != DISCOVERED_WIFI_CHANNEL) {
    Serial.println("WARNING: Channel mismatch between WiFi and ESP-NOW target");
  }

  // Optimize ESP-NOW settings
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(78);
  
  // Register callbacks
  esp_now_register_send_cb([](const uint8_t* mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
      Serial.println("ESP-NOW send failed");
    }
  });
  
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    lastImuReceived = millis();
    
    if (len == sizeof(SensorData)) {
      SensorData newData;
      memcpy(&newData, data, sizeof(SensorData));
      
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sharedData.imu = newData;
        xSemaphoreGive(dataMutex);
      }
    }
  });

  // Add peer - channel will be automatically matched to current WiFi channel
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));
  peerInfo.channel = currentChannel;  // Use current WiFi channel
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  memcpy(peerInfo.peer_addr, gyroMac, 6);
  esp_err_t result = esp_now_add_peer(&peerInfo);
  
  Serial.print("Gyro peer ");
  printMac(gyroMac);
  Serial.printf(" on channel %d: %s\n", currentChannel, 
               result == ESP_OK ? "Added" : "Failed");
  
  Serial.println("=== ESP-NOW DYNAMIC CHANNEL MODE ===");
  Serial.printf("Channel discovered: %d\n", DISCOVERED_WIFI_CHANNEL);
  Serial.printf("ESP-NOW using channel: %d\n", currentChannel);
  Serial.println("=====================================");
}

void udpTask(void *pvParameters) {
  Serial.println("UDP Task: Starting...");
  
  // Wait for WiFi to be ready
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("UDP Task: Waiting for WiFi...");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  
  // Initialize UDP receiver once
  bool bound = false;
  for (int i = 0; i < 10; i++) {
    if (udpReceiver.begin(4210)) {
      bound = true;
      Serial.println("UDP Task: Bound to port 4210");
      break;
    }
    Serial.printf("UDP bind attempt %d failed\n", i + 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  if (!bound) {
    Serial.println("ERROR: Could not bind UDP port - continuing without UDP");
    vTaskDelete(NULL);
    return;
  }

  while (1) {
    // Check WiFi before processing
    if (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    
    int packetSize = udpReceiver.parsePacket();
    if (packetSize > 0) {
      Serial.printf("UDP: Received packet size %d from %s:%d\n", 
                   packetSize, udpReceiver.remoteIP().toString().c_str(), udpReceiver.remotePort());
      
      // Update PC connection info atomically
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        sharedData.pcAddress = udpReceiver.remoteIP();
        sharedData.pcPort = udpReceiver.remotePort();
        sharedData.pcConnected = true;
        sharedData.lastPcContact = millis();
        xSemaphoreGive(dataMutex);
      }
      
      if (packetSize == sizeof(ControlCommand)) {
        ControlCommand msg;
        int len = udpReceiver.read((char*)&msg, sizeof(ControlCommand));
        
        if (len == sizeof(ControlCommand)) {
          Serial.printf("UDP: Motor command speed=%d, angle=%d\n", msg.speed, msg.angle);
          
          // Update motor command
          if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            sharedData.motor = msg;
            xSemaphoreGive(dataMutex);
          }
          
          // Forward to gyro device
          esp_err_t result = esp_now_send(gyroMac, (uint8_t*)&msg, sizeof(msg));
          if (result != ESP_OK) {
            Serial.printf("ESP-NOW send failed: %d\n", result);
          } else {
            Serial.println("ESP-NOW: Command forwarded to gyro");
          }
        }
      } else {
        // Clear the buffer for unexpected packet sizes
        char buffer[256];
        udpReceiver.read(buffer, min(packetSize, 255));
        Serial.printf("UDP: Unexpected packet size %d, cleared buffer\n", packetSize);
      }
    }
    
    // Check PC connection timeout
    unsigned long now = millis();
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (sharedData.pcConnected && (now - sharedData.lastPcContact > PC_TIMEOUT)) {
        sharedData.pcConnected = false;
        Serial.println("UDP: PC connection timeout");
      }
      xSemaphoreGive(dataMutex);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void slamSenderTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  Serial.println("SLAM Sender Task: Started");
  
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Skip if WiFi is down
    if (WiFi.status() != WL_CONNECTED) {
      continue;
    }

    bool shouldSend = false;
    IPAddress pcAddress;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      shouldSend = sharedData.pcConnected;
      pcAddress = sharedData.pcAddress;
      xSemaphoreGive(dataMutex);
    }
    
    if (shouldSend && pcAddress != IPAddress(0, 0, 0, 0)) {
      sendSlamPacket(pcAddress, 4211);
    }
  }
}

void sendSlamPacket(IPAddress address, uint16_t port) {
  SlamPacket packet;
  packet.timestamp = millis();
  
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    packet.imu[0] = sharedData.imu.gx;
    packet.imu[1] = sharedData.imu.gy;
    packet.imu[2] = sharedData.imu.gz;
    packet.imu[3] = sharedData.imu.ax;
    packet.imu[4] = sharedData.imu.ay;
    packet.imu[5] = sharedData.imu.az;
    packet.speed = sharedData.motor.speed;
    packet.angle = sharedData.motor.angle;
    xSemaphoreGive(dataMutex);
  } else {
    memset(packet.imu, 0, sizeof(packet.imu));
    packet.speed = 0;
    packet.angle = 90;
  }

  if (udpSender.beginPacket(address, port)) {
    udpSender.write((const uint8_t*)&packet, sizeof(packet));
    udpSender.endPacket();
  }
}

void monitorTask(void *pvParameters) {
  Serial.println("Monitor Task: Started");
  
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // Channel monitoring
    uint8_t currentChannel;
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&currentChannel, &secondChan);
    
    int heap = ESP.getFreeHeap();
    int minHeap = ESP.getMinFreeHeap();
    bool wifiConnected = (WiFi.status() == WL_CONNECTED);
    int rssi = wifiConnected ? WiFi.RSSI() : -100;
    
    SensorData imu;
    ControlCommand motor;
    bool pcConnected;
    unsigned long lastImu = lastImuReceived;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      imu = sharedData.imu;
      motor = sharedData.motor;
      pcConnected = sharedData.pcConnected;
      xSemaphoreGive(dataMutex);
    }
    
    Serial.printf("\n=== System Status ===\n");
    Serial.printf("Channel: %d (Discovered: %d) %s\n", currentChannel, DISCOVERED_WIFI_CHANNEL,
                  currentChannel == DISCOVERED_WIFI_CHANNEL ? "OK" : "DRIFT!");
    Serial.printf("Heap: %d bytes (min: %d)\n", heap, minHeap);
    Serial.printf("WiFi: %s (RSSI: %d dBm)\n", 
                  wifiConnected ? "OK" : "FAIL", rssi);
    Serial.printf("ESP-NOW: %s (last IMU: %lu ms ago)\n",
                  espnowHealthy ? "OK" : "FAIL", millis() - lastImu);
    Serial.printf("PC: %s\n", pcConnected ? "Connected" : "Disconnected");
    Serial.printf("Motor: speed=%d, angle=%d\n", motor.speed, motor.angle);
    Serial.printf("IMU: G(%.2f,%.2f,%.2f) A(%.2f,%.2f,%.2f)\n",
                  imu.gx, imu.gy, imu.gz, imu.ax, imu.ay, imu.az);
    
    if (heap < 50000) {
      Serial.println("WARNING: Low memory!");
    }
  }
}
