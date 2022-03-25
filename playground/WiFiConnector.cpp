#include <WiFiEsp.h>
#include <SoftwareSerial.h>

int wiFiConnectionStatus = WL_IDLE_STATUS;

/**
 * @brief Initializes the ESP8266-01S WiFi module. Call once in setup.
 * 
 * Connects to the WiFi module via SoftwareSerial. MUST be called before any other method of this library is effective.
 * 
 * @param digitalPinTX the Arduino digitalIO pin used for TX by SoftwareSerial.
 * @param digitalPinRX the Arduino digitalIO pin used for TX by SoftwareSerial.
 */
void initializeWiFiModule(int digitalPinTX, int digitalPinRX){
    SoftwareSerial EspSerial(2,3);
    EspSerial.begin(9600);
    WiFi.init(&EspSerial);
}

/**
 * @brief Connects to WiFi using the ESP8266-01S iff there is no WiFi connection, yet.
 * 
 * Suggested use: Call in loop to issue reconnection in case of connection loss.
 * 
 * @param ssid the SSID of the WiFi network to connect to.
 * @param password the password of the WiFi network to connect to.
 */
void connectToWiFi(char ssid[], char password[]){
    while(wiFiConnectionStatus != WL_CONNECTED){
        wiFiConnectionStatus = WiFi.begin(ssid, password);
    }
}