#include <esp_now.h>
#include <WiFi.h>

// Data structure must match the transmitter
typedef struct struct_message {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} struct_message;

struct_message incomingData;

// Back L298N (rear motors)
#define B_IN1 16
#define B_IN2 17
#define B_IN3 18
#define B_IN4 19
#define B_ENA 22
#define B_ENB 23

// Front L298N (front motors)
#define F_IN1 33
#define F_IN2 25
#define F_IN3 26
#define F_IN4 27
#define F_ENA 32
#define F_ENB 14

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    while (true);
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Set all pins as output
  int motorPins[] = {
    B_IN1, B_IN2, B_IN3, B_IN4, B_ENA, B_ENB,
    F_IN1, F_IN2, F_IN3, F_IN4, F_ENA, F_ENB
  };
  for (int i = 0; i < 12; i++) {
    pinMode(motorPins[i], OUTPUT);
  }

  stopMotors();
}

void loop() {
  // No need to write anything here
}

void stopMotors() {
  // All motors stop
  digitalWrite(B_IN1, LOW); digitalWrite(B_IN2, LOW);
  digitalWrite(B_IN3, LOW); digitalWrite(B_IN4, LOW);
  digitalWrite(F_IN1, LOW); digitalWrite(F_IN2, LOW);
  digitalWrite(F_IN3, LOW); digitalWrite(F_IN4, LOW);
}

// All forward
void moveForward() {
  analogWrite(B_ENA, 255); analogWrite(B_ENB, 255);
  analogWrite(F_ENA, 255); analogWrite(F_ENB, 255);

  digitalWrite(B_IN1, HIGH); digitalWrite(B_IN2, LOW);
  digitalWrite(B_IN3, HIGH); digitalWrite(B_IN4, LOW);

  digitalWrite(F_IN1, HIGH); digitalWrite(F_IN2, LOW);
  digitalWrite(F_IN3, HIGH); digitalWrite(F_IN4, LOW);
}

// All backward
void moveBackward() {
  analogWrite(B_ENA, 255); analogWrite(B_ENB, 255);
  analogWrite(F_ENA, 255); analogWrite(F_ENB, 255);

  digitalWrite(B_IN1, LOW); digitalWrite(B_IN2, HIGH);
  digitalWrite(B_IN3, LOW); digitalWrite(B_IN4, HIGH);

  digitalWrite(F_IN1, LOW); digitalWrite(F_IN2, HIGH);
  digitalWrite(F_IN3, LOW); digitalWrite(F_IN4, HIGH);
}

// Turn Left: Right wheels forward, left wheels stop/backward
void turnLeft() {
  analogWrite(B_ENA, 255); analogWrite(B_ENB, 255);
  analogWrite(F_ENA, 255); analogWrite(F_ENB, 255);

  // Right motors forward
  digitalWrite(B_IN3, HIGH); digitalWrite(B_IN4, LOW);
  digitalWrite(F_IN3, HIGH); digitalWrite(F_IN4, LOW);

  // Left motors backward
  digitalWrite(B_IN1, LOW); digitalWrite(B_IN2, HIGH);
  digitalWrite(F_IN1, LOW); digitalWrite(F_IN2, HIGH);
}

// Turn Right: Left wheels forward, right wheels stop/backward
void turnRight() {
  analogWrite(B_ENA, 255); analogWrite(B_ENB, 255);
  analogWrite(F_ENA, 255); analogWrite(F_ENB, 255);

  // Left motors forward
  digitalWrite(B_IN1, HIGH); digitalWrite(B_IN2, LOW);
  digitalWrite(F_IN1, HIGH); digitalWrite(F_IN2, LOW);

  // Right motors backward
  digitalWrite(B_IN3, LOW); digitalWrite(B_IN4, HIGH);
  digitalWrite(F_IN3, LOW); digitalWrite(F_IN4, HIGH);
}

// Data receive handler
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));

  Serial.print("AX: "); Serial.print(incomingData.ax);
  Serial.print(" AY: "); Serial.print(incomingData.ay);
  Serial.print(" AZ: "); Serial.println(incomingData.az);

  // Threshold logic
  if (incomingData.ax < -5000) {
    moveForward();
    Serial.println("→ Forward");
  } else if (incomingData.ax > 5000) {
    moveBackward();
    Serial.println("← Backward");
  } else if (incomingData.ay < -5000) {
    turnRight();
    Serial.println("↷ Right");
  } else if (incomingData.ay > 5000) {
    turnLeft();
    Serial.println("↶ Left");
  } else {
    stopMotors();
    Serial.println("■ Stop");
  }
}
