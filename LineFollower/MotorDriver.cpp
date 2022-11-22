/**
 * @file MotorDriver.c
 * @author david
 * @brief A library providing functions to control an arduino car using the motor controller L298N.
 */

#include "MotorDriver.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "HardwareSerial.h"

// The pins of an arduino (mega in this example) that are connected to the L298N motor controller.

// Front Motor Controller
// PMW pins
#define FRONT_LEFT_ENGINE_SPEED_PIN 2
#define FRONT_RIGHT_ENGINE_SPEED_PIN 7
// Digital IO pins
#define FRONT_LEFT_ENGINE_FORWARD_PIN 4
#define FRONT_LEFT_ENGINE_REVERSE_PIN 3
#define FRONT_RIGHT_ENGINE_FORWARD_PIN 5
#define FRONT_RIGHT_ENGINE_REVERSE_PIN 6

// Rear Motor Controller
// PMW pins
#define REAR_LEFT_ENGINE_SPEED_PIN 8
#define REAR_RIGHT_ENGINE_SPEED_PIN 13
// Digital IO pins
#define REAR_LEFT_ENGINE_FORWARD_PIN 10
#define REAR_LEFT_ENGINE_REVERSE_PIN 9
#define REAR_RIGHT_ENGINE_FORWARD_PIN 11
#define REAR_RIGHT_ENGINE_REVERSE_PIN 12

#define BOOST_THRESHOLD 60


uint8_t frontLeftCurrSpeed = 0;
uint8_t frontRightCurrSpeed = 0;
uint8_t rearLeftCurrSpeed = 0;
uint8_t rearRightCurrSpeed = 0;
/* enum MotorName {
    FRONT_LEFT,
    FRONT_RIGHT,
    REAR_LEFT,
    REAR_RIGHT,
};

enum MotorPinName {
    SPEED,
    FORWARD,
    REVERSE,
};

uint8_t getMotorPinNumber(MotorName motor, MotorPinName pin)
{
    switch (motor)
    {
    case FRONT_LEFT:
        switch (pin)
        {
        case SPEED:
            return 2;
        case FORWARD:
            return 4;
        case REVERSE:
            return 3;
        }
        break;
    case FRONT_RIGHT:
        switch (pin)
        {
        case SPEED:
            return 7;
        case FORWARD:
            return 5;
        case REVERSE:
            return 6;
        }
        break;
    case REAR_LEFT:
        switch (pin)
        {
        case SPEED:
            return 8;
        case FORWARD:
            return 10;
        case REVERSE:
            return 11;
        }
        break;
    case REAR_RIGHT:
        switch (pin)
        {
        case SPEED:
            return 13;
        case FORWARD:
            return 11;
        case REVERSE:
            return 12;
        }
        break;
    }
    return 0;
} */

/**
 * @brief Initializes the motor driver by setting the pin modes for the arduino pins.
 *
 * This method has to be called once for all other operations to work correctly!
 */
void initMotorDriver()
{
    /* for (int motor = FRONT_LEFT; motor <= REAR_RIGHT; motor++) {
        for (int pinIdx = SPEED; pinIdx <= )
    } */
    pinMode(FRONT_LEFT_ENGINE_SPEED_PIN, OUTPUT);
    pinMode(FRONT_RIGHT_ENGINE_SPEED_PIN, OUTPUT);
    pinMode(FRONT_LEFT_ENGINE_FORWARD_PIN, OUTPUT);
    pinMode(FRONT_LEFT_ENGINE_REVERSE_PIN, OUTPUT);
    pinMode(FRONT_RIGHT_ENGINE_FORWARD_PIN, OUTPUT);
    pinMode(FRONT_RIGHT_ENGINE_REVERSE_PIN, OUTPUT);
    pinMode(REAR_LEFT_ENGINE_SPEED_PIN, OUTPUT);
    pinMode(REAR_RIGHT_ENGINE_SPEED_PIN, OUTPUT);
    pinMode(REAR_LEFT_ENGINE_FORWARD_PIN, OUTPUT);
    pinMode(REAR_LEFT_ENGINE_REVERSE_PIN, OUTPUT);
    pinMode(REAR_RIGHT_ENGINE_FORWARD_PIN, OUTPUT);
    pinMode(REAR_RIGHT_ENGINE_REVERSE_PIN, OUTPUT);
    pinMode(52, OUTPUT);

    Serial.println("Soll:");
    Serial.println(TCCR5A & ~(1 << WGM51) & ~(1 << WGM50));
    Serial.println((TCCR5B & (~(1 << WGM52)) & (~(1 << CS51))) | (1 << WGM53) | (1 << CS50) | (1 << CS52));
    Serial.println(TIMSK5 | (1 << TOIE5));

    cli();
    // Set timer 5 to 9-bit PWM mode
    TCCR5A = TCCR5A & ~(1 << WGM51) & ~(1 << WGM50);
    // Ensure waveform generation mode bits are 0 to set mode
    // And set clock select to /1024 prescaler
    TCCR5B = (TCCR5B & (~(1 << WGM52)) & (~(1 << CS51))) | (1 << WGM53) | (1 << CS50) | (1 << CS52);
    // Set top value of counter
    ICR5H = 0b00011111;
    ICR5L = 0b11111111;
    // Enable overflow interrupt
    TIMSK5 = TIMSK5 | (1 << TOIE5);
    sei();
}

bool isOn = false;

ISR(TIMER5_OVF_vect) {
    isOn = !isOn;
    digitalWrite(52, isOn);
    analogWrite(FRONT_LEFT_ENGINE_SPEED_PIN, frontLeftCurrSpeed);
    analogWrite(REAR_LEFT_ENGINE_SPEED_PIN, rearLeftCurrSpeed);
    analogWrite(FRONT_RIGHT_ENGINE_SPEED_PIN, frontRightCurrSpeed);
    analogWrite(REAR_RIGHT_ENGINE_SPEED_PIN, rearRightCurrSpeed);
}

void resetTimer5() {
    TCNT5H = 0;
    TCNT5L = 1;
}

/**
 * @brief Sets the speed of the front right motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void frontLeftSpeed(int speed)
{
    frontLeftCurrSpeed = speed;
    /*if (frontLeftCurrSpeed > 0 && frontLeftCurrSpeed < BOOST_THRESHOLD) {
        Serial.print("Boosting for ");
        Serial.println(frontLeftCurrSpeed);
        analogWrite(FRONT_LEFT_ENGINE_SPEED_PIN, 255);
        resetTimer5();
    } else {
        Serial.print("Setting directly to ");
        Serial.println(frontLeftCurrSpeed);
        analogWrite(FRONT_LEFT_ENGINE_SPEED_PIN, frontLeftCurrSpeed);
    }*/
    analogWrite(FRONT_LEFT_ENGINE_SPEED_PIN, frontLeftCurrSpeed);
}

/**
 * @brief Sets the speed of the rear right motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void rearLeftSpeed(int speed)
{
    rearLeftCurrSpeed = speed;
    analogWrite(REAR_LEFT_ENGINE_SPEED_PIN, speed);
}

/**
 * @brief Sets the speed of the front left motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void frontRightSpeed(int speed)
{
    frontRightCurrSpeed = speed;
    analogWrite(FRONT_RIGHT_ENGINE_SPEED_PIN, speed);
}

/**
 * @brief Sets the speed of the rear left motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void rearRightSpeed(int speed)
{
    rearRightCurrSpeed = speed;
    analogWrite(REAR_RIGHT_ENGINE_SPEED_PIN, speed);
}

/**
 * @brief Sets the dc motor direction of the right motors to "reverse".
 *
 * If the motor direction is currently set to "reverse", this method changes the motor direction.
 * If the motor direction was already set to "forward", the method will not affect any change.
 */
void rightForward()
{
    digitalWrite(FRONT_RIGHT_ENGINE_FORWARD_PIN, HIGH);
    digitalWrite(FRONT_RIGHT_ENGINE_REVERSE_PIN, LOW);
    digitalWrite(REAR_RIGHT_ENGINE_FORWARD_PIN, HIGH);
    digitalWrite(REAR_RIGHT_ENGINE_REVERSE_PIN, LOW);
}

/**
 * @brief Sets the dc motor direction of the right motors to "reverse".
 *
 * If the motor direction is currently set to "forward", this method changes the motor direction.
 * If the motor direction was already set to "reverse", the method will not affect any change.
 */
void rightReverse()
{
    digitalWrite(FRONT_RIGHT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(FRONT_RIGHT_ENGINE_REVERSE_PIN, HIGH);
    digitalWrite(REAR_RIGHT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(REAR_RIGHT_ENGINE_REVERSE_PIN, HIGH);
}

/**
 * @brief Sets the speed of the right motors to the desired value.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void rightSpeed(int speed)
{
    analogWrite(FRONT_RIGHT_ENGINE_SPEED_PIN, speed);
    analogWrite(REAR_RIGHT_ENGINE_SPEED_PIN, speed);
}

/**
 * @brief Sets the dc motor direction of the left motors to "forward".
 *
 * If the motor direction is currently set to "reverse", this method changes the motor direction.
 * If the motor direction was already set to "forward", the method will not affect any change.
 */
void leftForward()
{
    digitalWrite(FRONT_LEFT_ENGINE_FORWARD_PIN, HIGH);
    digitalWrite(FRONT_LEFT_ENGINE_REVERSE_PIN, LOW);
    digitalWrite(REAR_LEFT_ENGINE_FORWARD_PIN, HIGH);
    digitalWrite(REAR_LEFT_ENGINE_REVERSE_PIN, LOW);
}

/**
 * @brief Sets the dc motor direction of the left motors to "reverse".
 *
 * If the motor direction is currently set to "forward", this method changes the motor direction.
 * If the motor direction was already set to "reverse", the method will not affect any change.
 */
void leftReverse()
{
    digitalWrite(FRONT_LEFT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(FRONT_LEFT_ENGINE_REVERSE_PIN, HIGH);
    digitalWrite(REAR_LEFT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(REAR_LEFT_ENGINE_REVERSE_PIN, HIGH);
}

/**
 * @brief Stops the dc motor on the right side to be shorted thus stopping them immediately
 */
void rightStop()
{
    digitalWrite(FRONT_RIGHT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(FRONT_RIGHT_ENGINE_REVERSE_PIN, LOW);
    digitalWrite(REAR_RIGHT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(REAR_RIGHT_ENGINE_REVERSE_PIN, LOW);
}

/**
 * @brief Stops the dc motor on the left side to be shorted thus stopping them immediately
 */
void leftStop()
{
    digitalWrite(FRONT_LEFT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(FRONT_LEFT_ENGINE_REVERSE_PIN, LOW);
    digitalWrite(REAR_LEFT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(REAR_LEFT_ENGINE_REVERSE_PIN, LOW);
}

/**
 * @brief Sets the speed of the left motors to the desired value.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void leftSpeed(int speed)
{
    frontLeftSpeed(speed);
    rearLeftSpeed(speed);
}

/**
 * @brief Sets the speed of all motors to the desired value.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void setSpeed(int speed)
{
    leftSpeed(speed);
    rightSpeed(speed);
}

/**
 * @brief Makes the car stop by deactivating all motors.
 *
 */
void stop()
{
    leftStop();
    rightStop();
    setSpeed(255);
}

/**
 * @brief Sets the movement direction of all motors to "forward".
 *
 * If the motor direction is currently set to "reverse", this method changes the motor direction.
 * If the motor direction was already set to "forward", the method will not affect any change.
 */
void forward()
{
    leftForward();
    rightForward();
}

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
void driveForward(int speed)
{
    setSpeed(speed);
    forward();
}

/**
 * @brief Sets the dc motor direction of all motors to "reverse".
 *
 * If the motor direction is currently set to "forward", this method changes the motor direction.
 * If the motor direction was already set to "reverse", the method will not affect any change.
 */
void reverse()
{
    leftReverse();
    rightReverse();
}

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
void driveReverse(int speed)
{
    setSpeed(speed);
    reverse();
}

/**
 * @brief Turns the car left if driving forward.
 *
 * More specifically: the right motors are set to "forward" and the left motors to "reverse".
 */
void turnLeft()
{
    rightForward();
    leftReverse();
}

/**
 * @brief Turns the car left with the desired speed.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void turnLeftForward(int speed)
{
    leftSpeed(speed*0.75);
    rightSpeed(speed*1.25);
    turnLeft();
}

/**
 * @brief Turns the car right if driving forward.
 *
 * More specifically: the right motors are set to "reverse" and the left motors to "forward".
 */
void turnRight()
{
    leftForward();
    rightReverse();
}

/**
 * @brief Turns the car right with the desired speed.
 *
 * @param speed specify the motor speed between 60 (slowest moving speed) and 255 (full speed), or 0 to deactivate the motors.
 *
 * The operation controls the motors my sending PMW signals to the L298N motor controller.
 * A value between 0-59 is not recommended, because the motors to not get enough power to move the car.
 */
void turnRightForward(int speed)
{
    rightSpeed(speed*0.75);
    leftSpeed(speed*1.25);
    turnRight();
}

void steerLeftForward(int speed, double fraction)
{
    forward();
    leftSpeed(speed / fraction);
    rightSpeed(speed);
}

void steerRightForward(int speed, double fraction)
{
    forward();
    leftSpeed(speed);
    rightSpeed(speed / fraction);
}