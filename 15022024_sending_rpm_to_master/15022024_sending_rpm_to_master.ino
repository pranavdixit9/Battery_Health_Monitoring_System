// This example code is in the Public Domain (or CC0 licensed, at your option.)
// By Victor Tchistiak - 2019
//
// This example demonstrates master mode Bluetooth connection to a slave BT device using PIN (password)
// defined either by String "slaveName" by default "OBDII" or by MAC address
//
// This example creates a bridge between Serial and Classical Bluetooth (SPP)
// This is an extension of the SerialToSerialBT example by Evandro Copercini - 2018
//
// DO NOT try to connect to phone or laptop - they are master
// devices, same as the ESP using this code - it will NOT work!
//
// You can try to flash a second ESP32 with the example SerialToSerialBT - it should
// automatically pair with ESP32 running this code

#include "BluetoothSerial.h"

#define USE_NAME // Comment this to use MAC address instead of a slaveName
const char *pin = "0000"; // Change this to reflect the pin expected by the real slave BT device

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#ifdef USE_NAME
  String slaveName = "RM17"; // Change this to reflect the real name of your slave BT device
#else
  String MACadd = "00:18:91:D8:03:FA"; // This only for printing
  uint8_t address[6]  = {0x00, 0x18, 0x91, 0xD8, 0x03, 0xFA}; // Change this to reflect real MAC address of your slave BT device
#endif

String myName = "ESP32-BT-Master";
volatile int pulseCount = 0;
unsigned long lastTime;
unsigned long currentTime;
unsigned long timeInterval = 500; // Time interval in milliseconds

const int encoderPinA = 26; // Example GPIO pin for encoder channel A
const int encoderPinB = 14; // Example GPIO pin for encoder channel B

void setup() {
 
  bool connected;
  Serial.begin(115200);

  SerialBT.begin(myName, true);
  Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());

  #ifndef USE_NAME
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif

  // connect(address) is fast (up to 10 secs max), connect(slaveName) is slow (up to 30 secs max) as it needs
  // to resolve slaveName to address first, but it allows to connect to different devices with the same name.
  // Set CoreDebugLevel to Info to view devices Bluetooth address and device names
  #ifdef USE_NAME
    connected = SerialBT.connect(slaveName);
    Serial.printf("Connecting to slave BT device named \"%s\"\n", slaveName.c_str());
  #else
    connected = SerialBT.connect(address);
    Serial.print("Connecting to slave BT device with MAC "); Serial.println(MACadd);
  #endif

  if(connected) {
    Serial.println("Connected Successfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");
    }
  }
  // Disconnect() may take up to 10 secs max
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Successfully!");
  }
  // This would reconnect to the slaveName(will use address, if resolved) or address used with connect(slaveName/address).
  SerialBT.connect();
  if(connected) {
    Serial.println("Reconnected Successfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to reconnect. Make sure remote device is available and in range, then restart app.");
    }
  }
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), countPulse, RISING);
  lastTime = millis();
  
}

void loop() {
  currentTime = millis();
  if ((currentTime - lastTime) >= timeInterval) {
    float rpm = (pulseCount / 400.0) * 60.0 / (timeInterval / 1000.0); // Assuming 400 counts per revolution
    Serial.print("Speed (RPM): ");
    Serial.println(rpm);
    SerialBT.write(rpm);
    // Send data over Bluetooth
    String rpmString = String(rpm, 2); // Format RPM to 2 decimal places
//    pCharacteristic->setValue(rpmString.c_str());
//    pCharacteristic->notify();

    pulseCount = 0;
    lastTime = currentTime;
  }
//  if (Serial.available()) {
//    SerialBT.write(Serial.read());
//  }
//  if (SerialBT.available()) {
//    Serial.write(SerialBT.read());
//  }
  delay(20);
}

void countPulse() {
  pulseCount++;
}
