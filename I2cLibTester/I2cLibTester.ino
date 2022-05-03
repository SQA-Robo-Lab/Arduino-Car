const byte ownI2CAddress = 9;
const byte driverI2CAddress = 8;

#include "I2cCustomLib.hpp"

void setup(){
    Serial.begin(9600);

    i2cCommunication_setup(ownI2CAddress);

    Serial.println("Setup done");
}

void loop(){
    sendI2cMessage(driverI2CAddress, "start", (byte*) "unimportant", 12);
    Serial.println("start");
    delay(random(2000, 7000));

    sendI2cMessage(driverI2CAddress, "stop", (byte*) "unimportant", 12);
    Serial.println("stop");
    delay(random(2000, 7000));
} 