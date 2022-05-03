#include "MqttCustomLib.hpp"
#include "I2cCustomLib.hpp"
#include <WiFiEsp.h>

struct MqttConfig *mqttConfig;
MqttSubscriber *sub;
int i;

void setup(){
    i=0;
    Serial.begin(9600);


    Serial.println("Setup");
    struct MqttConfig mConf = {
        "192.168.0.100",
        1883,
        "Arduino_Test"
    };
    mqttConfig = &mConf;
    
    struct WiFiConfig wifiConfig = {
        "Section_Control",
        "Latcc@tsc4c!",
        WL_IDLE_STATUS
    };

    mqttCommunication_setup(&wifiConfig, mqttConfig);
    delay(10000);
    sendMqttMessage("car/", "commands", (byte*) "Setup", 6);
    
    sub = (MqttSubscriber*) malloc(sizeof(MqttSubscriber));
    initAndRegisterMqttSubscriber(sub, "car/", "commands", 1, 20, true);

    // i2cCommunication_setup(9);

    Serial.println("Setup done!");
}

void loop(){
    mqttCommunication_loop(mqttConfig);
    // globalMqttClient.loop();
    if (MessageBuffer_doesMessageExists(sub->buffer)){
        Serial.println("New message!");
        char* message;
        MessageBuffer_dequeue(sub->buffer, message);
        Serial.println(message);
    }
    if (i % 300 == 0){
        Serial.println("Hello");
        sendMqttMessage("helloWorld/", "hello", (byte*) "hello", 6);
    }
    i++;
}