/**
 * @file MotorDriver.c
 * @author david
 * @brief A library providing functions to control an arduino car using the motor controller L298N.
 */

#include "MotorDriver.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "HardwareSerial.h"

#ifdef STEERED_CAR
    #include "Servo.h"
#endif


#ifdef ARDUINO_UNO_CAR

    #define FRONT_LEFT_ENGINE_SPEED_PIN 6
    #define FRONT_RIGHT_ENGINE_SPEED_PIN 11
    // Digital IO pins
    #define FRONT_LEFT_ENGINE_FORWARD_PIN 8
    #define FRONT_LEFT_ENGINE_REVERSE_PIN 9
    #define FRONT_RIGHT_ENGINE_FORWARD_PIN 12
    #define FRONT_RIGHT_ENGINE_REVERSE_PIN 13

    // Rear Motor Controller
    // PMW pins
    #define REAR_LEFT_ENGINE_SPEED_PIN 3
    #define REAR_RIGHT_ENGINE_SPEED_PIN 3
    // Digital IO pins
    #define REAR_LEFT_ENGINE_FORWARD_PIN 3
    #define REAR_LEFT_ENGINE_REVERSE_PIN 3
    #define REAR_RIGHT_ENGINE_FORWARD_PIN 3
    #define REAR_RIGHT_ENGINE_REVERSE_PIN 3

#else
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

#endif

#ifdef STEERED_CAR
    #define STEER_SERVO_PIN 2
    #define MAX_STEERING_ANGLE 45
    #define MAX_STEER_SERVO_VALUE 111
    #define MIN_STEER_SERVO_VALUE 21

    Servo steeringServo;
#endif

#define BOOST_THRESHOLD 60


uint8_t frontLeftCurrSpeed = 0;
uint8_t frontRightCurrSpeed = 0;
uint8_t rearLeftCurrSpeed = 0;
uint8_t rearRightCurrSpeed = 0;
uint8_t frontLeftCurrDirection = 0; //0b00, 0b11: Off; 0b10: forward; 0b01: backward
uint8_t frontRightCurrDirection = 0;
uint8_t rearLeftCurrDirection = 0;
uint8_t rearRightCurrDirection = 0;

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
    Serial.println(FRONT_LEFT_ENGINE_SPEED_PIN);

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

    #ifdef STEERED_CAR
        steeringServo.attach(STEER_SERVO_PIN);
        setSteeringAngle(0);
    #endif

    #ifdef AVR_MEGA
        //Set timer 5
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

    #endif
}

#ifdef AVR_MEGA
    bool isOn = false;

    ISR(TIMER5_OVF_vect) {
        isOn = !isOn;
        digitalWrite(52, isOn);
    }

    void resetTimer5() {
        TCNT5H = 0;
        TCNT5L = 1;
    }
#endif

/**
 * @brief Sets the speed of the front right motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void frontLeftSpeed(int speed)
{
    frontLeftCurrSpeed = speed;
    if (frontLeftCurrSpeed == 0) {
        frontLeftStop();
    } else {
        frontLeftDirection(frontLeftCurrDirection);
        analogWrite(FRONT_LEFT_ENGINE_SPEED_PIN, frontLeftCurrSpeed);
    }
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
}

/**
 * @brief Sets the speed of the rear right motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void rearLeftSpeed(int speed)
{
    rearLeftCurrSpeed = speed;
    if (rearLeftCurrSpeed == 0) {
        rearLeftStop();
    } else {
        rearLeftDirection(rearLeftCurrDirection);
        analogWrite(REAR_LEFT_ENGINE_SPEED_PIN, speed);
    }
}

/**
 * @brief Sets the speed of the front left motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void frontRightSpeed(int speed)
{
    frontRightCurrSpeed = speed;
    if (frontRightCurrSpeed == 0) {
        frontRightStop();
    } else {
        frontRightDirection(frontRightCurrDirection);
        analogWrite(FRONT_RIGHT_ENGINE_SPEED_PIN, speed);
    }
}

/**
 * @brief Sets the speed of the rear left motor to the desired value
 *
 * @param speed specify the motor speed between 0 (slowest speed) and 255 (full speed), or 0 to deactivate the motors.
 */
void rearRightSpeed(int speed)
{
    rearRightCurrSpeed = speed;
    if (rearRightCurrSpeed == 0) {
        rearRightStop();
    } else {
        rearRightDirection(rearRightCurrDirection);
        analogWrite(REAR_RIGHT_ENGINE_SPEED_PIN, speed);
    }
}

void frontLeftDirection(uint8_t direction) {
    frontLeftCurrDirection = direction;
    analogWrite(FRONT_LEFT_ENGINE_SPEED_PIN, frontLeftCurrSpeed);
    digitalWrite(FRONT_LEFT_ENGINE_FORWARD_PIN, (direction >> 1) & 0b1);
    digitalWrite(FRONT_LEFT_ENGINE_REVERSE_PIN, (direction) & 0b1);
}

void frontRightDirection(uint8_t direction) {
    frontRightCurrDirection = direction;
    analogWrite(FRONT_RIGHT_ENGINE_SPEED_PIN, frontRightCurrSpeed);
    digitalWrite(FRONT_RIGHT_ENGINE_FORWARD_PIN, (direction >> 1) & 0b1);
    digitalWrite(FRONT_RIGHT_ENGINE_REVERSE_PIN, (direction) & 0b1);
}

void rearLeftDirection(uint8_t direction) {
    rearLeftCurrDirection = direction;
    analogWrite(REAR_LEFT_ENGINE_SPEED_PIN, rearLeftCurrSpeed);
    digitalWrite(REAR_LEFT_ENGINE_FORWARD_PIN, (direction >> 1) & 0b1);
    digitalWrite(REAR_LEFT_ENGINE_REVERSE_PIN, (direction) & 0b1);
}

void rearRightDirection(uint8_t direction) {
    rearRightCurrDirection = direction;
    analogWrite(REAR_RIGHT_ENGINE_SPEED_PIN, rearRightCurrSpeed);
    digitalWrite(REAR_RIGHT_ENGINE_FORWARD_PIN, (direction >> 1) & 0b1);
    digitalWrite(REAR_RIGHT_ENGINE_REVERSE_PIN, (direction) & 0b1);
}

/**
 * @brief Sets the dc motor direction of the right motors to "reverse".
 *
 * If the motor direction is currently set to "reverse", this method changes the motor direction.
 * If the motor direction was already set to "forward", the method will not affect any change.
 */
void rightForward()
{
    frontRightDirection(MOTOR_FORWARD);
    rearRightDirection(MOTOR_FORWARD);
}

/**
 * @brief Sets the dc motor direction of the right motors to "reverse".
 *
 * If the motor direction is currently set to "forward", this method changes the motor direction.
 * If the motor direction was already set to "reverse", the method will not affect any change.
 */
void rightReverse()
{
    frontRightDirection(MOTOR_BACKWARD);
    rearRightDirection(MOTOR_BACKWARD);
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
    frontRightSpeed(speed);
    rearRightSpeed(speed);
}

/**
 * @brief Sets the dc motor direction of the left motors to "forward".
 *
 * If the motor direction is currently set to "reverse", this method changes the motor direction.
 * If the motor direction was already set to "forward", the method will not affect any change.
 */
void leftForward()
{
    frontLeftDirection(MOTOR_FORWARD);
    rearLeftDirection(MOTOR_FORWARD);
}

/**
 * @brief Sets the dc motor direction of the left motors to "reverse".
 *
 * If the motor direction is currently set to "forward", this method changes the motor direction.
 * If the motor direction was already set to "reverse", the method will not affect any change.
 */
void leftReverse()
{
    frontLeftDirection(MOTOR_BACKWARD);
    rearLeftDirection(MOTOR_BACKWARD);
}

void frontLeftStop() {
    analogWrite(FRONT_LEFT_ENGINE_SPEED_PIN, 255);
    digitalWrite(FRONT_LEFT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(FRONT_LEFT_ENGINE_REVERSE_PIN, LOW);
}

void frontRightStop() {
    analogWrite(FRONT_RIGHT_ENGINE_SPEED_PIN, 255);
    digitalWrite(FRONT_RIGHT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(FRONT_RIGHT_ENGINE_REVERSE_PIN, LOW);
}

void rearLeftStop() {
    analogWrite(REAR_LEFT_ENGINE_SPEED_PIN, 255);
    digitalWrite(REAR_LEFT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(REAR_LEFT_ENGINE_REVERSE_PIN, LOW);
}

void rearRightStop() {
    analogWrite(REAR_RIGHT_ENGINE_SPEED_PIN, 255);
    digitalWrite(REAR_RIGHT_ENGINE_FORWARD_PIN, LOW);
    digitalWrite(REAR_RIGHT_ENGINE_REVERSE_PIN, LOW);
}

void setSteeringAngle(int8_t angle) {
#ifdef STEERED_CAR
    long value = min(MAX_STEER_SERVO_VALUE, max(MIN_STEER_SERVO_VALUE, map(angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE, MIN_STEER_SERVO_VALUE, MAX_STEER_SERVO_VALUE)));
    /*Serial.print("Steering: Angle: ");
    Serial.print(angle);
    Serial.print("°; Value: ");
    Serial.println(value);*/
    steeringServo.write(value);
#endif
}

int8_t fracionToAngle(double fraction) {
    if (fraction >= 0) {
        return round(90.0-atan(2/fraction)/PI*180);
    } else {
        return round(-(90+atan(2/fraction)/PI*180));
    }
}

/**
 * @brief Stops the dc motor on the right side to be shorted thus stopping them immediately
 */
void rightStop()
{
    frontRightStop();
    rearRightStop();
}

/**
 * @brief Stops the dc motor on the left side to be shorted thus stopping them immediately
 */
void leftStop()
{
    frontLeftStop();
    rearLeftStop();
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
}

/**
 * @brief Sets the direction of all motors to the desired direction
 */
void setDirection(MotorDirection direction) {
    frontLeftDirection(direction);
    frontRightDirection(direction);
    rearLeftDirection(direction);
    rearRightDirection(direction);
}

/**
 * @brief Sets the movement direction of all motors to "forward".
 *
 * If the motor direction is currently set to "reverse", this method changes the motor direction.
 * If the motor direction was already set to "forward", the method will not affect any change.
 */
void forward()
{
    #ifdef STEERED_CAR
        setSteeringAngle(0);
    #endif
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
    #ifdef STEERED_CAR
        setSteeringAngle(-45);
        leftForward();
        rightForward();
    #else
        rightForward();
        leftReverse();
    #endif
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
    #ifdef STEERED_CAR
        setSteeringAngle(-30);
        setSpeed(speed);
    #else
        leftSpeed(speed*0.75);
        rightSpeed(speed*1.25);
        turnLeft();
    #endif
}

/**
 * @brief Turns the car right if driving forward.
 *
 * More specifically: the right motors are set to "reverse" and the left motors to "forward".
 */
void turnRight()
{
    #ifdef STEERED_CAR
        setSteeringAngle(45);
        leftForward();
        rightForward();
    #else
        leftForward();
        rightReverse();
    #endif
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
    #ifdef STEERED_CAR
        setSteeringAngle(30);
        setSpeed(speed);
    #else
        rightSpeed(speed*0.75);
        leftSpeed(speed*1.25);
        turnRight();
    #endif
}

void steerLeftForward(int speed, double fraction)
{
    forward();

    #ifdef STEERED_CAR
        setSteeringAngle(-fracionToAngle(fraction));
        setSpeed(speed);
    #else
        leftSpeed(speed / fraction);
        rightSpeed(speed);
    #endif
}

void steerRightForward(int speed, double fraction)
{
    forward();

    #ifdef STEERED_CAR
        setSteeringAngle(fracionToAngle(fraction));
        setSpeed(speed);
    #else
        leftSpeed(speed);
        rightSpeed(speed / fraction);
    #endif
}
