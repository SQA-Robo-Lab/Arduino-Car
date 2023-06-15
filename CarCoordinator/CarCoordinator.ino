//#define ON_ESP
//#define SECOND_CAR

#include <Wire.h>

#ifdef ON_ESP
    #include <ESP8266WiFi.h>
#else 
    #include <SoftwareSerial.h>
    #include <WiFiEsp.h>
#endif
#include <PubSubClient.h>

#define _SS_MAX_RX_BUFF 256

const byte ownI2CAddress = 9;
const byte driverI2CAddress = 8;

char ssid[] = "SpyWiFi";
char pass[] = "Raspberry";

#ifdef ON_ESP
    WiFiClient wifiClient;
#else
    int status = WL_IDLE_STATUS;
    SoftwareSerial EspSerial(2,3);
    WiFiEspClient wifiClient;
#endif

PubSubClient mqttClient;

#ifdef SECOND_CAR
    char mqttTopicPub[] = "car2-commands";
    char mqttTopicSub[] = "car1-commands";
#else
    char mqttTopicPub[] = "car1-commands";
    char mqttTopicSub[] = "car2-commands";
#endif

boolean mqttNewMessageFlag = false;
char mqttMessageBuffer[128];

bool i2cNewMessageFlag = false;
char i2cMessageBuffer[32]; //32 bytes is max of arduino i2c wire library

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
    #ifdef ON_ESP
        Serial.begin(115200);
    #else
        Serial.begin(115200);
    #endif

    #ifdef ON_ESP
        Wire.begin(2, 0, ownI2CAddress);
    #else 
        Wire.begin(ownI2CAddress);
    #endif

    Serial.println("Sending test i2c message");
    Wire.beginTransmission(driverI2CAddress);
    Wire.write("Hallo Welt");
    Wire.endTransmission();
    Wire.onReceive(receiveEvent);

    #ifdef ON_ESP
        Serial.println("Connecting to Wifi");
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, pass);
        while (WiFi.status() != WL_CONNECTED) {
            Serial.print('.');
            delay(500);
        }
        Serial.print("Connected with IP: ");
        Serial.println(WiFi.localIP());

    #else
        EspSerial.begin(9600);
        WiFi.init(&EspSerial);
        while(status != WL_CONNECTED){
            status = WiFi.begin(ssid, pass);
        }
        delay(1000);
    #endif
    mqttClient.setClient(wifiClient);

    mqttClient.setServer("devonport.informatik.uni-stuttgart.de", 1883);
    mqttClient.setCallback(consumeMessage);

    Serial.println("MQTT setup complete. Connecting to server");

    mqttConnect();

    Serial.println("Connected to MQTT-Server");

    #ifdef SECOND_CAR
        mqttClient.publish("hello-world", "Second car has registered!");
    #else
        mqttClient.publish("hello-world", "Car has registered!");
    #endif

}

void loop(){
    //Serial.println("loop");
    if(!mqttClient.connected()){
        Serial.println("MQTT-Connection lost. Trying to reestablish");
        mqttConnect();
    }
    mqttClient.loop();
    if (mqttNewMessageFlag){
        Wire.beginTransmission(driverI2CAddress);
        Wire.write(mqttMessageBuffer);
        Wire.endTransmission();
        mqttNewMessageFlag = false;
        Serial.println("Sent message to i2c");
    }
    if (i2cNewMessageFlag) {
        mqttClient.publish(mqttTopicPub, i2cMessageBuffer);
        i2cNewMessageFlag = false;
        Serial.println("Sent message to mqtt");
    }
    //delay(100);
}

void mqttConnect(){
    while(!mqttClient.connected()){ 
        #ifdef SECOND_CAR
        if (mqttClient.connect("arduino2")) {
        #else
        if (mqttClient.connect("arduino1")) {
        #endif
            mqttClient.subscribe(mqttTopicSub);
            Serial.println("MQTT connected");
        } else {
            Serial.println("MQTT not connected");
            delay(1000);
        }
    }
}

void receiveEvent(size_t howMany) {
  uint8_t bufferIndex = 0;
  while (Wire.available() > 0) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    i2cMessageBuffer[bufferIndex] = c;         // print the character
    bufferIndex++;
  }
  Serial.print("I2C message: ");
  Serial.println(i2cMessageBuffer);
  i2cNewMessageFlag = true;
}