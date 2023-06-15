/**
 * @file MotorDriver.h
 * @author david
 * @brief A library providing functions to control an arduino car using the motor controller L298N.
 */

#include "Arduino.h"

//#define ARDUINO_UNO_CAR
//#define STEERED_CAR

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

enum MotorDirection {
    MOTOR_STOP = 0b00,
    MOTOR_BACKWARD = 0b01,
    MOTOR_FORWARD = 0b10,
    MOTOR_STOP_HIGH = 0b11
};

/**
 * @brief Initializes the motor driver by setting the pin modes for the arduino pins.
 *
 * This method has to be called once for all other operations to work correctly!
 */
void initMotorDriver();

void testTimer();

/**
 * @brief Sets the speed of the front right motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void frontLeftSpeed(int speed);

/**
 * @brief Sets the speed of the rear right motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void rearLeftSpeed(int speed);

/**
 * @brief Sets the speed of the front left motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void frontRightSpeed(int speed);

/**
 * @brief Sets the speed of the rear left motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void rearRightSpeed(int speed);

void frontLeftDirection(uint8_t direction);

void frontRightDirection(uint8_t direction);

void rearLeftDirection(uint8_t direction);

void rearRightDirection(uint8_t direction);

void setSteeringAngle(int8_t angle);

/**
 * @brief Sets the dc motor direction of the right motors to "reverse".
 *
 * If the motor direction is currently set to "reverse", this method changes the motor direction.
 * If the motor direction was already set to "forward", the method will not affect any change.
 */
void rightForward();

/**
 * @brief Sets the dc motor direction of the right motors to "reverse".
 *
 * If the motor direction is currently set to "forward", this method changes the motor direction.
 * If the motor direction was already set to "reverse", the method will not affect any change.
 */
void rightReverse();

/**
 * @brief Sets the speed of the right motors to the desired value.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void rightSpeed(int speed);

/**
 * @brief Sets the dc motor direction of the left motors to "forward".
 *
 * If the motor direction is currently set to "reverse", this method changes the motor direction.
 * If the motor direction was already set to "forward", the method will not affect any change.
 */
void leftForward();

/**
 * @brief Sets the dc motor direction of the left motors to "reverse".
 *
 * If the motor direction is currently set to "forward", this method changes the motor direction.
 * If the motor direction was already set to "reverse", the method will not affect any change.
 */
void leftReverse();

void frontLeftStop();

void frontRightStop();

void rearLeftStop();

void rearRightStop();

/**
 * @brief Stops the dc motor on the right side to be shorted thus stopping them immediately
 */
void rightStop();

/**
 * @brief Stops the dc motor on the left side to be shorted thus stopping them immediately
 */
void leftStop();

/**
 * @brief Sets the speed of the left motors to the desired value.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void leftSpeed(int speed);

/**
 * @brief Sets the speed of all motors to the desired value.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void setSpeed(int speed);

/**
 * @brief Makes the car stop by deactivating all motors.
 *
 */
void stop();

/**
 * @brief Sets the movement direction of all motors to "forward".
 *
 * If the motor direction is currently set to "reverse", this method changes the motor direction.
 * If the motor direction was already set to "forward", the method will not affect any change.
 */
void forward();

/**
 * @brief Sets the direction of all motors to the desired direction
 */
void setDirection(MotorDirection direction);

/**
 * @brief Sets the movement direction of all motors to "forward" AND sets the motor speed.
 *
 * If the motor direction is currently set to "reverse", this method changes the motor direction.
 * If the motor direction was already set to "forward", the method will not affect any change.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void driveForward(int speed);

/**
 * @brief Sets the dc motor direction of all motors to "reverse".
 *
 * If the motor direction is currently set to "forward", this method changes the motor direction.
 * If the motor direction was already set to "reverse", the method will not affect any change.
 */
void reverse();

/**
 * @brief Sets the movement direction of all motors to "reverse" AND sets the motor speed.
 *
 * If the motor direction is currently set to "forward", this method changes the motor direction.
 * If the motor direction was already set to "reverse", the method will not affect any change.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void driveReverse(int speed);

/**
 * @brief Turns the car left if driving forward.
 *
 * More specifically: the right motors are set to "forward" and the left motors to "reverse".
 */
void turnLeft();

/**
 * @brief Turns the car left with the desired speed.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void turnLeftForward(int speed);

/**
 * @brief Turns the car right if driving forward.
 *
 * More specifically: the right motors are set to "reverse" and the left motors to "forward".
 */
void turnRight();

/**
 * @brief Turns the car right with the desired speed.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void turnRightForward(int speed);

void steerLeftForward(int speed, double fraction);

void steerRightForward(int speed, double fraction);

#endif