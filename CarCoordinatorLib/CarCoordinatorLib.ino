#define DEBUG 1

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
    char* test_message = "hello";
    sendMqttMessage("helloWorld/", "hello", (byte*) test_message, strlen(test_message));
    char* sub_topic = "car-commands/";
    createAndRegisterMqttSubscriber(sub, sub_topic, sub_topic, 1, 20, true);

    i2cCommunication_setup(9);
}

void loop(){
    mqttCommunication_loop(mqttConfig);
    Serial.println(MessageBuffer_doesMessageExists(sub->buffer));
    Serial.println(sub->buffer->count);
    delay(1000);
    // if (MessageBuffer_doesMessageExists(sub->buffer)){
    //     Serial.println("New message!");
    //     char* message;
    //     MessageBuffer_dequeue(sub->buffer, message);
    // }
    // if (i % 10000 == 0){
    //     Serial.println("Hello");
    //     char* test_message = "hello";
    //     sendMqttMessage("helloWorld/", "hello", &test_message);
    // }
    // i++;
}