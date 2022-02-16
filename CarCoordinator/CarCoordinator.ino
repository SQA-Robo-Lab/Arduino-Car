#include <Wire.h>
#include <SoftwareSerial.h>
#include <WiFiEsp.h>

const byte ownI2CAddress = 9;
const byte driverI2CAddress = 8;

char ssid[] = "Davids Galaxy A52s";
char pass[] = "********";
int status = WL_IDLE_STATUS;
SoftwareSerial EspSerial(2,3);

void setup(){
    Serial.begin(9600);

    Wire.begin(ownI2CAddress);

    EspSerial.begin(9600);
    WiFi.init(&EspSerial);

    while(status != WL_CONNECTED){
        status = WiFi.begin(ssid, pass);
    }
}

void loop(){
    Wire.beginTransmission(driverI2CAddress);
    Wire.write("START");
    Wire.endTransmission();
    Serial.println("START");
    delay(random(2000, 7000));

    Wire.beginTransmission(driverI2CAddress);
    Wire.write("STOP");
    Wire.endTransmission();
    Serial.println("STOP");
    delay(random(2000, 7000));
}