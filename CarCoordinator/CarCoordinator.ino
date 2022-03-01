#include <Wire.h>
#include <SoftwareSerial.h>
#include <WiFiEsp.h>
#include <PubSubClient.h>

#define _SS_MAX_RX_BUFF 256

const byte ownI2CAddress = 9;
const byte driverI2CAddress = 8;

char ssid[] = "Section_Control";
char pass[] = "************";
int status = WL_IDLE_STATUS;
SoftwareSerial EspSerial(2,3);

WiFiEspClient wifiClient;
PubSubClient mqttClient;

char mqttTopic[] = "car-commands";

boolean mqttNewMessageFlag = false;
char mqttMessageBuffer[128];

void consumeMessage(char* topic, byte* payload, unsigned int length){
    mqttNewMessageFlag = true;
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    memset(mqttMessageBuffer, 0, sizeof(mqttMessageBuffer));
    for (int i = 0; i < length; i++) {
        mqttMessageBuffer[i] = (char) payload[i];
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void setup(){
    Serial.begin(9600);

    Wire.begin(ownI2CAddress);

    EspSerial.begin(9600);
    WiFi.init(&EspSerial);

    while(status != WL_CONNECTED){
        status = WiFi.begin(ssid, pass);
    }
    
    delay(1000);
    mqttClient.setClient(wifiClient);
    mqttClient.setServer("192.168.0.100", 1883);
    mqttClient.setCallback(consumeMessage);

    mqttConnect();

    mqttClient.publish("hello-world", "Hello! The new car has registered!");

}

void loop(){
    Serial.println("loop");
    if(!mqttClient.connected()){
        mqttConnect();
    }
    mqttClient.loop();
    if (mqttNewMessageFlag){
        Wire.beginTransmission(driverI2CAddress);
        Wire.write(mqttMessageBuffer);
        Wire.endTransmission();
        mqttNewMessageFlag = false;
    }
}

void mqttConnect(){
    while(!mqttClient.connected()){ 
        if (mqttClient.connect("arduino")) {
            mqttClient.subscribe(mqttTopic);
            Serial.println("MQTT connected");
        } else {
            Serial.println("MQTT not connected");
            delay(1000);
        }
    }
}