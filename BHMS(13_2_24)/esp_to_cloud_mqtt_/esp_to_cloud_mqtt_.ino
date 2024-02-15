#include <WiFi.h>
#include <PubSubClient.h>
#include<HardwareSerial.h>
HardwareSerial SerialPort(2);

// WiFi settings
const char* ssid = "Meow9pro";
const char* password = "Realmeow";

// ThingsBoard MQTT settings
const char* tbServer = "demo.thingsboard.io";
const int tbPort = 1883;
const char* tbToken = "CwhObfCS6EyMRQMWwFZc";

// MQTT topics
const char* tbTopic = "v1/devices/me/telemetry";

// Initialize WiFi and MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// Function to connect to WiFi and MQTT
void connectToWiFiAndMQTT() {
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to ThingsBoard MQTT
  client.setServer(tbServer, tbPort); 
}


void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200,SERIAL_8N1,16,17);
  // Connect to WiFi and MQTT
  connectToWiFiAndMQTT();
}

void loop() {

   if (!client.connected()) {
        reConnect();
    }
  int receivedData[4];
  
    if (SerialPort.available() >= 17) { // Ensure enough data is available
    String tempData = SerialPort.readStringUntil('#');
    tempData.trim(); // Remove leading/trailing whitespace

    // Parse data using strtok_r (more robust and reentrant)
    char buffer[tempData.length() + 1];
    tempData.toCharArray(buffer, sizeof(buffer));
    char* next_token = NULL;
    char* token = strtok_r(buffer, ",", &next_token); // Get first token
    int i = 0;

    while (token != NULL && i < 4) {
      receivedData[i] = atoi(token); // Convert token to integer
      token = strtok_r(NULL, ",", &next_token); // Get next token
      i++;
    }

    Serial.println(receivedData[0]);
    Serial.println(receivedData[1]);
    Serial.println(receivedData[2]);
    Serial.println(receivedData[3]);
  

  sendToTB("data1", String(receivedData[0]));
  sendToTB("data2", String(receivedData[1]));
  sendToTB("data3", String(receivedData[2]));
  sendToTB("data4", String(receivedData[3]));
  
  }
  // Wait for a few seconds before sending the next reading
  delay(1000);
}

void sendToTB(const char *parameter, String value) {
    
    // Construct the JSON payload
    String payload = "{\"" + String(parameter) + "\":" + value + "}";
    
    // Publish the payload to ThingsBoard
  if (client.publish(tbTopic, payload.c_str())) {
    Serial.println(String(parameter) + " sent to ThingsBoard");
  } else {
    Serial.println("Failed to send data");
  }
}

void reConnect() {
  while (!client.connected()) {
    Serial.print("Attempting to Connect MQTT...\n");
    
    if (client.connect("ESP32Client", tbToken, NULL)) {
      Serial.println("Connected to ThingsBoard");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}
