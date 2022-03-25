#include <WiFiEsp.h>

void initializeMQTTclient(WiFiEspClient& wiFiClient, char mqttServerAddress[], int mqttServerPort);

void connectToMQTT(char clientId[]);

void connectToMQTT(char clientId[], char subscriptionTopic[], void (*callback)(char*, uint8_t*, unsigned int));

void mqttLoop();