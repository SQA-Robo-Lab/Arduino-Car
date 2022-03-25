#include "MQTTConnector.h"
#include "WiFiConnector.h"

#define ESP_CONNECTION_PIN_TX 2
#define ESP_CONNECTION_PIN_RX 3

char ssid[] = "Davids Galaxy A52s";
char pass[] = "1ccwDnSs!";

char mqttServerAddress[] = "192.168.209.56";
int mqttServerPort = 1883;
char mqttClientId[] = "Arduino Tester";

void setup(){
    Serial.begin(9600);
    Serial.println("Starting setup...");

    initializeWiFiModule(ESP_CONNECTION_PIN_TX, ESP_CONNECTION_PIN_RX);
    connectToWiFi(ssid, pass);
    Serial.println("Connected to WiFi!");

    WiFiEspClient wiFiClient;
    initializeMQTTclient(wiFiClient, mqttServerAddress, mqttServerPort);
    connectToMQTT(mqttClientId);
    Serial.println("Connected to MQTT");

    Serial.println("...setup done!");
}

void loop(){
    connectToWiFi(ssid, pass);
    connectToMQTT(mqttClientId);
    mqttLoop();
}