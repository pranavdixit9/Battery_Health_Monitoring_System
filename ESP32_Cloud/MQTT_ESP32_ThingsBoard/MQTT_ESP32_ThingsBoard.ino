+#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_wifi.h>

// Spoofed MAC address for WiFi
uint8_t newMACAddress[] = {0xf4, 0x96, 0x34, 0x9d, 0xe5, 0xd1};

// WiFi credentials
const char *ssid = "acts";
const char *password = "";

// ThingsBoard MQTT configuration
const char *mqttServer = "demo.thingsboard.io";  // ThingsBoard MQTT broker address
const char *token = "kehq7u9qdm9141hfqr79";

// Initialize WiFi and MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
    // Begin serial communication for debugging
    Serial.begin(115200);

    // Connect to WiFi and set up MQTT server
    connectToWiFi();
    client.setServer(mqttServer, 1883);
}

void loop() {
    // Check if not connected to ThingsBoard MQTT broker
    if (!client.connected()) {
        // Reconnect to ThingsBoard MQTT broker if disconnected
        reconnect();
    }
  int receivedValues[2];
  if (Serial.available() >= 6) { // Assuming each integer is sent as 2 bytes (16 bits)
    for (int i = 0; i < 2; i++) {
      receivedValues[i] = Serial.read() << 8 | Serial.read();
    }
    // Generate random temperature and humidity
//    float temperature = random(25, 50);
//    float humidity = random(10, 70);

    // Send telemetry data to ThingsBoard
    sendTelemetryToThingsBoard("temperature", String(receivedValues[1]));
    sendTelemetryToThingsBoard("Humidity", String(receivedValues[2]));

    // Delay between data transmissions
    delay(5000);  // Adjust delay between data transmissions as needed
}

void connectToWiFi() {
    // Print status message
    Serial.println("Attempting to connect...");

    // Set WiFi mode to STA (station/client)
    WiFi.mode(WIFI_STA);

    // Spoof WiFi MAC address
    esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);

    // Connect to WiFi network
    WiFi.begin(ssid, password);

    // Wait until connected to WiFi
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print(".");
    }

    // Print connection success message
    Serial.println("\nConnected to WiFi");
}

void reconnect() {
    // Attempt to reconnect to ThingsBoard MQTT broker
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");

        // Attempt to connect to ThingsBoard MQTT broker
        if (client.connect("ESP32Client", token, NULL)) {
            Serial.println("connected");
        } else {
            // Print connection failure message with retry information
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");

            // Delay before retrying
            delay(5000);
        }
    }
}

void sendTelemetryToThingsBoard(const char *parameter, String value) {
    // Define the MQTT topic for sending telemetry data
    String topic = "v1/devices/me/telemetry";

    // Construct the JSON payload
    String payload = "{\"" + String(parameter) + "\":" + value + "}";

    // Publish telemetry data to ThingsBoard
    client.publish(topic.c_str(), payload.c_str());
    
    // Print success message
    Serial.println("Data sent to ThingsBoard");
}
