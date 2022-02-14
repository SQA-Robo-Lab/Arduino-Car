/**
 * @file LineFollower.c
 * @author david
 * @brief This libary connects the sensors and actuators required for line following.
 */

#include "MotorDriver.c"
#include "LineDetector.c"

LinePosition lineFollowerPosition;

/**
 * @brief Initialize all sensors and actuators required for the LineFollower.
 * 
 * MUST be called before followLine(int speed) can be used effectively.
 */
void initializeLineFollower(){
    initMotorDriver();

    initLineDetector();
    lineFollowerPosition = detectPosition(ON_LINE);
}

/**
 * @brief Call this function repeatedly in order to make the car follow a line.
 * 
 * On every call, it determines the car's relative position to the line and adjusts its movement.
 * 
 * @param speed specify the car's speed between 60 (slowest moving speed) and 255 (full speed).
 */
void followLine(int speed){
    lineFollowerPosition = detectPosition(lineFollowerPosition);
    if (lineFollowerPosition == ON_LINE){
        driveForward(speed);
    } else if (lineFollowerPosition == LEFT_OF_LINE){
        turnRightForward(speed*1.5);
    } else if (lineFollowerPosition == RIGHT_OF_LINE){
        turnLeftForward(speed*1.5);
    }
}