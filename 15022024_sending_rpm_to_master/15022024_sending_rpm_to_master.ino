#include "BluetoothSerial.h"

volatile int pulseCount = 0;
unsigned long lastTime;
unsigned long currentTime;
unsigned long timeInterval = 1000; // Time interval in milliseconds

const int encoderPinA = 26; // Example GPIO pin for encoder channel A
const int encoderPinB = 14; // Example GPIO pin for encoder channel B

BluetoothSerial SerialBT;

const char *myName = "ESP32-BT-Encoder";
const char *slaveName = "RM17"; // Name of the slave Bluetooth device

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), countPulse, RISING);
  lastTime = millis();

  Serial.begin(115200);
  SerialBT.begin(myName, true); // Start Bluetooth in master mode

  // Wait for connection
  while (!SerialBT.connected()) {
    delay(100);
  }

  // Connect to the slave device
  Serial.println("Connecting to slave Bluetooth device...");
  if (SerialBT.connect(slaveName)) {
    Serial.println("Connected to slave Bluetooth device successfully!");
  } else {
    Serial.println("Failed to connect to slave Bluetooth device.");
  }
}

void loop() {
  currentTime = millis();
  if ((currentTime - lastTime) >= timeInterval) {
    float rpm = (pulseCount / 400.0) * 60.0 / (timeInterval / 1000.0); // Assuming 20 counts per revolution
    SerialBT.print("Speed (RPM): ");
    SerialBT.println(rpm);
    pulseCount = 0;
    lastTime = currentTime;
  }
}

void countPulse() {
  pulseCount++;
}
