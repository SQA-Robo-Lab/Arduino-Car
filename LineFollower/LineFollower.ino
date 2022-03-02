/**
 * @file LineFollower.ino
 * @author david
 * @brief Demonstration code for a line follower car. 
 */
#include <Wire.h>

#include "./MotorDriver.c"
#include "./LineDetector.c"
#include "./DistanceSensor.c"

LinePosition position;
int baseSpeed = 60;

const byte ownI2CAddress = 8;
const byte coordinatorI2CAddress = 9;

char i2cBuffer[32];
boolean i2cNewMessageFlag = false;

boolean drivingAllowed = true;

void setup(){
    Wire.begin(ownI2CAddress);
    Wire.onReceive(receiveI2CMessage);
    Serial.begin(9600);
    initMotorDriver();

    initLineDetector();
    position = detectPosition(ON_LINE);

    initializeDistanceSensors();

    Serial.println("Setup complete!");
}

void loop(){
    if (i2cNewMessageFlag) {
        // Serial.println(i2cBuffer);
        i2cNewMessageFlag = false;
        if (strncmp(i2cBuffer, "STOP", 4) == 0){
            drivingAllowed = false;
        } else {
            drivingAllowed = true;
        }
        // Serial.println(drivingAllowed);
    }
    
    int d = getFrontDistance();
    // Serial.println(d);
    if (d < 10){
        stop();
    } else if (!drivingAllowed){
        stop();
    } else {
        followLine(baseSpeed);
    }
    // delay(100);
    // followLine(baseSpeed);
}

void followLine(int speed){
    position = detectPosition(position);
    if (position == ON_LINE){
        // Serial.println("On line!");
        driveForward(speed);
    } else if (position == LEFT_OF_LINE){
        // Serial.println("LEFT of line!");
        turnRightForward(speed*1.5);
    } else if (position == RIGHT_OF_LINE){
        // Serial.println("RIGHT of line");
        turnLeftForward(speed*1.5);
    } else {
        Serial.println("This is strange...");
        stop();
    }
}

void receiveI2CMessage(int numberOfBytesReceived){
    // Serial.println("Message received!");
    memset(i2cBuffer, 0, sizeof(i2cBuffer));
    Wire.readBytes(i2cBuffer, numberOfBytesReceived);
    i2cNewMessageFlag = true;
}