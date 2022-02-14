/**
 * @file LineFollower.ino
 * @author david
 * @brief Demonstration code for a line follower car. 
 */

#include "MotorDriver.c"
#include "LineDetector.c"

LinePosition position;
int base_speed = 60;

void setup(){
    Serial.begin(9600);
    initMotorDriver();

    initLineDetector();
    position = detectPosition(ON_LINE);

    Serial.println("Setup complete!");
}

void loop(){
    followLine(base_speed);
    // delay(100);
}

void followLine(int speed){
    position = detectPosition(position);
    if (position == ON_LINE){
        Serial.println("On line!");
        driveForward(speed);
    } else if (position == LEFT_OF_LINE){
        Serial.println("LEFT of line!");
        turnRightForward(speed*1.5);
    } else if (position == RIGHT_OF_LINE){
        Serial.println("RIGHT of line");
        turnLeftForward(speed*1.5);
    } else {
        Serial.println("This is strange...");
        stop();
    }
}