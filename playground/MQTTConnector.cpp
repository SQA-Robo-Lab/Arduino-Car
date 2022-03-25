#include <PubSubClient.h>
#include <WiFiEsp.h>

PubSubClient mqttClient;

/**
 * @brief Initialize the MQTT client.
 * 
 * @param wiFiClient the WiFi client to be used for the MQTT connection
 * @param mqttServerAddress the IP address of the MQTT server as a string
 * @param mqttServerPort the TCP port of the MQTT server as an int
 */
void initializeMQTTclient(WiFiEspClient& wiFiClient, char mqttServerAddress[], int mqttServerPort){
    mqttClient.setClient(wiFiClient);
    mqttClient.setServer(mqttServerAddress, mqttServerPort);
}

/**
 * @brief Connect to the MQTT server iff the client is not connected.
 * 
 * @param clientId an identifier of the client at the MQTT server.
 */
void connectToMQTT(char clientId[]){
    while(!mqttClient.connected()){
        mqttClient.connect(clientId);
    }
}

/**
 * @brief Connect to the MQTT server iff the client is not connected including a subscription.
 * 
 * @param clientId an identifier of the client at the MQTT server.
 * @param subscriptionTopic the name of the topic to subscribe to.
 * @param callback a callback of the shape void (*callback)(char*, uint8_t*, unsigned int)
 */
void connectToMQTT(char clientId[], char subscriptionTopic[], void (*callback)(char*, uint8_t*, unsigned int)){
    while(!mqttClient.connected()){
        mqttClient.connect(clientId);
        mqttClient.setCallback(callback);
        mqttClient.subscribe(subscriptionTopic);
    }
}

void mqttLoop(){
    mqttClient.loop();
}