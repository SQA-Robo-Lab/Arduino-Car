/**
 * @file LineFollowerLib.ino
 * @author david
 * @brief Demonstration code for a line follower car. 
 */

#include "LineFollower.h"
#include "DistanceSensor.h"
#include "I2cCustomLib.hpp"

int baseSpeed = 60;

const byte ownI2CAddress = 8;
const byte communicatorI2CAddress = 9;

I2cReceiver* startReceiver;
I2cReceiver* stopReceiver;

boolean drivingAllowed;

void setup(){
    Serial.begin(9600);

    initLineFollower();

    initializeDistanceSensors();

    i2cCommunication_setup(ownI2CAddress);

    drivingAllowed = false;

    startReceiver = (I2cReceiver*) malloc(sizeof(I2cReceiver));
    initAndRegisterI2cReceiver(startReceiver, "start", 1, 12, true);

    stopReceiver = (I2cReceiver*) malloc(sizeof(I2cReceiver));
    initAndRegisterI2cReceiver(stopReceiver, "stop", 1, 12, true);

    Serial.println("Setup complete!");
}

void loop(){
    if(MessageBuffer_doesMessageExists(startReceiver->buffer)){
        Serial.println("Start Message!");
        drivingAllowed = true;
        consumeAndDiscardMessageFromBuffer(startReceiver->buffer);
    }
    if(MessageBuffer_doesMessageExists(stopReceiver->buffer)){
        Serial.println("Stop Message!");
        drivingAllowed = false;
        consumeAndDiscardMessageFromBuffer(stopReceiver->buffer);
    }

    int d = getFrontDistance();
    if (d < 10){
        stop();
    } else if (!drivingAllowed){
        stop();
    } else {
        followLine(baseSpeed);
    }
}

void consumeAndDiscardMessageFromBuffer(MessageBuffer* buffer){
    byte* message = (byte*) malloc(buffer->elementSize);
    MessageBuffer_dequeue(buffer, message);
    free (message);
}