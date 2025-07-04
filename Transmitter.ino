#include <Wire.h>
#include <MPU6050.h>
#include <esp_now.h>
#include <WiFi.h>

MPU6050 mpu;

#define CHANNEL 1  // Communication channel

// Define the structure for the data to be sent
typedef struct struct_message {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} struct_message;

struct_message myData;

// Receiver MAC Address (replace with your receiver's MAC address)
uint8_t receiverMac[6] = {0x14, 0x33, 0x5C, 0x0A, 0x88, 0xC8};

void setup() {
  Serial.begin(115200);
  
  Wire.begin(21, 22); // SDA, SCL
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // WiFi setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Disconnect from any network
  delay(100);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    while (1);
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Clear peer info
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = CHANNEL;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1);
  } else {
    Serial.println("Peer added successfully");
  }

  Serial.println("Setup complete");
}

void loop() {
  mpu.getAcceleration(&myData.ax, &myData.ay, &myData.az);
  mpu.getRotation(&myData.gx, &myData.gy, &myData.gz);

  // Send data
  esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.print("Error sending data: ");
    Serial.println(result);
  }

  Serial.print("Accelerometer: ");
  Serial.print("AX: "); Serial.print(myData.ax); 
  Serial.print(" AY: "); Serial.print(myData.ay); 
  Serial.print(" AZ: "); Serial.println(myData.az);

  delay(100);
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}