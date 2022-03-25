#ifndef MQTT_CUSTOM_LIB_H
#define MQTT_CUSTOM_LIB_H

#include <SoftwareSerial.h>
#include <WiFiEsp.h>
#include <PubSubClient.h>

#include "ContainerTypes.h"

#define _SS_MAX_RX_BUFF 256

typedef struct MqttSubscriber {
    uint16_T messageId;
    MessageBuffer* buffer;
} MqttSubscriber;


//create MQTTHandle
typedef struct MQTTHandle {
    uint8_T numOfSubs;
    MqttSubscriber subscribers[];
} MQTTHandle;

#endif /* MQTT_CUSTOM_LIB_H */