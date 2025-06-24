#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <MPU9250_asukiaaa.h>
#include <Wire.h>
#include "esp_wifi.h"
#include <esp_wifi_types.h>

// Motor control pins
#define IN1 25
#define IN2 26
#define ENA 23
#define SERVO_PIN 13

// PWM settings
#define MOTOR_PWM_CHANNEL 0
#define SERVO_PWM_CHANNEL 2
#define MOTOR_PWM_FREQ 1000
#define SERVO_PWM_FREQ 50
#define PWM_RESOLUTION 16

// Base Station MAC
uint8_t baseMac[] = {0x78,0x42,0x1C,0x6C,0x89,0x84}; // Replace with your actual base MAC

// Channel discovery settings
#define MAX_CHANNEL_SCAN 13
#define DISCOVERY_TIMEOUT 30000  // 30 seconds
#define PING_INTERVAL 1000       // 1 second between pings
#define PING_RETRIES 3           // Retries per channel

// Data structures
typedef struct {
  int32_t speed;
  int32_t angle;
} ControlCommand;

typedef struct {
  float gx, gy, gz;  // Gyroscope data (rad/s)
  float ax, ay, az;  // Accelerometer data (g)
} SensorData;

typedef struct {
  uint8_t type;      // 0 = ping, 1 = pong, 2 = sensor_data, 3 = control_command
  uint32_t timestamp;
  uint8_t data[32];  // Flexible payload
} DiscoveryPacket;

// Shared state
volatile ControlCommand currentCommand = {0, 90}; // Default values
MPU9250_asukiaaa imu;
float gyroBias[3] = {0};
float accelBias[3] = {0};

// Timing control
unsigned long lastSendTime = 0;
const int SEND_INTERVAL = 100; // 10Hz gyro update
unsigned long lastStatusTime = 0;
const int STATUS_INTERVAL = 5000; // 5s status reports

// Connection management
volatile bool baseStationConnected = false;
volatile unsigned long lastBaseContact = 0;
volatile bool imuHealthy = false;
volatile uint8_t activeChannel = 0;
volatile bool channelDiscovered = false;

// === Hardware Control Functions === 
void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  int duty = (pulseWidth * (1 << PWM_RESOLUTION)) / 20000;
  ledcWrite(SERVO_PWM_CHANNEL, duty);
}

void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  int scaledSpeed = map(abs(speed), 0, 255, 0, (1 << PWM_RESOLUTION) - 1);
  
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  
  ledcWrite(MOTOR_PWM_CHANNEL, scaledSpeed);
}

// === IMU Functions ===
bool initializeIMU() {
  Serial.println("Initializing IMU...");
  
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  
  imu.setWire(&Wire);
  
  // Initialize with retries
  imu.beginAccel();
  imu.beginGyro(); 
  Serial.println("IMU initialized successfully");
  return true;
}

void calibrateIMU(int samples) {
  Serial.println("Calibrating IMU... Keep sensor still!");
  float gyroSum[3] = {0};
  float accelSum[3] = {0};
  int validSamples = 0;
  
  for (int i = 0; i < samples; i++) {
    if (imu.accelUpdate() == 0 && imu.gyroUpdate() == 0) {
      gyroSum[0] += imu.gyroX();
      gyroSum[1] += imu.gyroY();
      gyroSum[2] += imu.gyroZ();
      
      accelSum[0] += imu.accelX();
      accelSum[1] += imu.accelY();
      accelSum[2] += imu.accelZ() - 1.0; // Remove gravity (Z axis)
      
      validSamples++;
    }
    delay(10);
    
    if (i % 100 == 0) {
      Serial.printf("Calibration progress: %d/%d\n", i, samples);
    }
  }

  if (validSamples < samples / 2) {
    Serial.println("WARNING: Poor calibration quality");
  }

  for (int i = 0; i < 3; i++) {
    gyroBias[i] = gyroSum[i] / validSamples;
    accelBias[i] = accelSum[i] / validSamples;
  }

  Serial.printf("Gyro Bias: X=%.4f, Y=%.4f, Z=%.4f rad/s\n", gyroBias[0], gyroBias[1], gyroBias[2]);
  Serial.printf("Accel Bias: X=%.4f, Y=%.4f, Z=%.4f g\n", accelBias[0], accelBias[1], accelBias[2]);
  Serial.printf("Calibration complete with %d valid samples\n", validSamples);
}

bool readIMU(SensorData* data) {
  if (imu.accelUpdate() == 0 && imu.gyroUpdate() == 0) {
    data->gx = imu.gyroX() - gyroBias[0];
    data->gy = imu.gyroY() - gyroBias[1];
    data->gz = imu.gyroZ() - gyroBias[2];
    data->ax = imu.accelX() - accelBias[0];
    data->ay = imu.accelY() - accelBias[1];
    data->az = imu.accelZ() - accelBias[2];
    return true;
  }
  return false;
}

// === Channel Discovery Functions ===
bool testChannelConnection(uint8_t channel) {
  Serial.printf("Testing channel %d...\n", channel);
  
  // Set channel
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  delay(100);
  
  // Verify channel was set
  uint8_t currentChannel;
  wifi_second_chan_t secondChan;
  esp_wifi_get_channel(&currentChannel, &secondChan);
  
  if (currentChannel != channel) {
    Serial.printf("Failed to set channel %d (got %d)\n", channel, currentChannel);
    return false;
  }
  
  // Update peer info for new channel
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(esp_now_peer_info_t));
  memcpy(peerInfo.peer_addr, baseMac, 6);
  peerInfo.channel = channel;
  peerInfo.encrypt = false;
  
  // Remove old peer if exists
  esp_now_del_peer(baseMac);
  delay(50);
  
  // Add peer for this channel
  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK) {
    Serial.printf("Failed to add peer on channel %d: %d\n", channel, result);
    return false;
  }
  
  // Send ping packets
  for (int retry = 0; retry < PING_RETRIES; retry++) {
    DiscoveryPacket ping;
    ping.type = 0; // ping
    ping.timestamp = millis();
    memset(ping.data, 0, sizeof(ping.data));
    
    // Reset connection flag
    baseStationConnected = false;
    
    esp_err_t sendResult = esp_now_send(baseMac, (uint8_t*)&ping, sizeof(ping));
    if (sendResult != ESP_OK) {
      Serial.printf("Ping send failed on channel %d: %d\n", channel, sendResult);
      continue;
    }
    
    // Wait for response
    unsigned long startWait = millis();
    while (millis() - startWait < PING_INTERVAL && !baseStationConnected) {
      delay(10);
    }
    
    if (baseStationConnected) {
      Serial.printf("SUCCESS: Found base station on channel %d!\n", channel);
      return true;
    }
    
    Serial.printf("No response on channel %d (attempt %d/%d)\n", channel, retry + 1, PING_RETRIES);
  }
  
  return false;
}

bool discoverBaseChannel() {
  Serial.println("\n=== Starting Base Station Discovery ===");
  Serial.printf("Scanning channels 1-%d for base station MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                MAX_CHANNEL_SCAN, baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  
  unsigned long discoveryStart = millis();
  
  // Try each channel
  for (uint8_t channel = 1; channel <= MAX_CHANNEL_SCAN; channel++) {
    if (millis() - discoveryStart > DISCOVERY_TIMEOUT) {
      Serial.println("Discovery timeout reached");
      break;
    }
    
    if (testChannelConnection(channel)) {
      activeChannel = channel;
      channelDiscovered = true;
      Serial.printf("=== DISCOVERY COMPLETE: Channel %d ===\n", channel);
      return true;
    }
    
    // Brief pause between channels
    delay(200);
  }
  
  Serial.println("=== DISCOVERY FAILED: No base station found ===");
  return false;
}

// === ESP-NOW Handlers ===
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    baseStationConnected = true;
    lastBaseContact = millis();
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  lastBaseContact = millis();
  baseStationConnected = true;
  
  // Handle discovery packets
  if (len == sizeof(DiscoveryPacket)) {
    DiscoveryPacket* packet = (DiscoveryPacket*)data;
    
    if (packet->type == 1) { // pong response
      Serial.printf("Received pong from base station on channel %d\n", activeChannel);
      return;
    }
  }
  
  // Handle control commands
  if (len == sizeof(ControlCommand)) {
    ControlCommand newCommand;
    memcpy(&newCommand, data, len);
    
    // Update current command immediately
    currentCommand.speed = newCommand.speed;
    currentCommand.angle = newCommand.angle;
    
    // Apply commands immediately
    setMotorSpeed(currentCommand.speed);
    setServoAngle(currentCommand.angle);
    
    Serial.printf("Command Applied: Speed=%d, Angle=%d\n", 
                 newCommand.speed, newCommand.angle);
  }
}

void initializeESPNOW() {
  Serial.println("Initializing ESP-NOW...");
  
  // Set WiFi mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  Serial.printf("Device MAC: %s\n", WiFi.macAddress().c_str());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("FATAL: ESP-NOW Init Failed");
    ESP.restart();
  }

  // Optimize settings
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(78);

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP-NOW initialized, ready for channel discovery");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== Starting Auto-Discovery Gyro/Motor Controller ===");

  // 1. Initialize Motor Hardware
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  
  // Set safe initial state
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, LOW);
  
  ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, MOTOR_PWM_CHANNEL);
  ledcWrite(MOTOR_PWM_CHANNEL, 0);
  
  ledcSetup(SERVO_PWM_CHANNEL, SERVO_PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN, SERVO_PWM_CHANNEL);
  setServoAngle(90); // Center position

  Serial.println("Motor hardware initialized");

  // 2. Initialize IMU
  if (!initializeIMU()) {
    Serial.println("FATAL: IMU initialization failed");
    ESP.restart();
  }
  
  calibrateIMU(500);
  imuHealthy = true;
  Serial.println("IMU calibrated and ready");

  // 3. Initialize ESP-NOW
  initializeESPNOW();

  // 4. Discover base station channel
  if (!discoverBaseChannel()) {
    Serial.println("FATAL: Could not find base station");
    Serial.println("Please verify:");
    Serial.println("1. Base station is powered and running");
    Serial.println("2. Base station MAC address is correct");
    Serial.println("3. Base station is within range");
    Serial.println("Restarting in 10 seconds...");
    delay(10000);
    ESP.restart();
  }

  Serial.printf("=== System Ready on Channel %d ===\n", activeChannel);
  lastSendTime = millis();
  lastStatusTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if we need to rediscover (connection lost for too long)
  if (channelDiscovered && baseStationConnected && 
      (currentMillis - lastBaseContact > 10000)) { // 10 second timeout
    Serial.println("Connection lost - attempting rediscovery...");
    baseStationConnected = false;
    channelDiscovered = false;
    
    if (!discoverBaseChannel()) {
      Serial.println("Rediscovery failed - restarting...");
      delay(5000);
      ESP.restart();
    }
  }

  // Only proceed with normal operation if channel is discovered
  if (!channelDiscovered) {
    delay(1000);
    return;
  }

  // 1. IMU Reading and Transmission (10Hz)
  if (currentMillis - lastSendTime >= SEND_INTERVAL) {
    SensorData sensorData;
    bool imuRead = readIMU(&sensorData);
    
    if (imuRead) {
      imuHealthy = true;
      
      esp_err_t result = esp_now_send(baseMac, (uint8_t*)&sensorData, sizeof(sensorData));
      
      if (result != ESP_OK) {
        Serial.printf("Send failed: %d\n", result);
      }
    } else {
      imuHealthy = false;
      Serial.println("IMU read failed");
    }
    
    lastSendTime = currentMillis;
  }

  // 2. Status Reporting
  if (currentMillis - lastStatusTime >= STATUS_INTERVAL) {
    Serial.printf("\n=== Status Report ===\n");
    Serial.printf("Channel: %d\n", activeChannel);
    Serial.printf("Connection: %s (last contact: %lu ms ago)\n",
                 baseStationConnected ? "OK" : "LOST", 
                 currentMillis - lastBaseContact);
    Serial.printf("IMU: %s\n", imuHealthy ? "OK" : "FAIL");
    Serial.printf("Current Commands: Speed=%d, Angle=%d\n",
                 currentCommand.speed, currentCommand.angle);
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    
    lastStatusTime = currentMillis;
  }
  
  // 3. System stability delay
  delay(80);
}