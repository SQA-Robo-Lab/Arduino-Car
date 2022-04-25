/**
 * @file LineFollowerLib.ino
 * @author david
 * @brief Demonstration code for a line follower car. 
 */

#include "LineFollower.h"
#include "DistanceSensor.h"

int baseSpeed = 60;

void setup(){
    Serial.begin(9600);

    initLineFollower();

    initializeDistanceSensors();

    Serial.println("Setup complete!");
}

void loop(){    
    int d = getFrontDistance();
    if (d < 10){
        stop();
    } else {
        followLine(baseSpeed);
    }
}