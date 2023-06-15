/**
 * @file LineDetector.c
 * @author david
 * @brief A library providing operations to use three KY-033 line detection sensors to detect the position of an arduino car with respect to a line on the ground.
 */
#include "Arduino.h"

//define your pins connected to OUT/SIGNAL (ditial IO pins)
#ifdef ARDUINO_UNO_CAR
#define IR_SIGNAL_PIN_LEFT A0
#define IR_SIGNAL_PIN_CENTER A1
#define IR_SIGNAL_PIN_RIGHT A2
#define ANALOG_THREATHOLD 85
#else
#define IR_SIGNAL_PIN_LEFT 30
#define IR_SIGNAL_PIN_CENTER 32
#define IR_SIGNAL_PIN_RIGHT 34
#endif


typedef enum {
    LEFT_OF_LINE,
    ON_LINE,
    RIGHT_OF_LINE,
}LinePosition;

/**
 * @brief Initializes the line detector by setting the pin modes for the arduino pins.
 * 
 * This method has to be called once for all other operations to work correctly!
 */
void initLineDetector(){
    pinMode(IR_SIGNAL_PIN_LEFT, INPUT);
    pinMode(IR_SIGNAL_PIN_CENTER, INPUT);
    pinMode(IR_SIGNAL_PIN_RIGHT, INPUT);
}

/**
 * @brief read the values of the line detection sensors.
 * 
 * @param signals an int buffer of size 3 to write the signals to as return values.
 * 
 * signals[0] will hold the value of the LEFT sensor
 * signals[1] will hold the value of the CENTER sensor
 * signals[2] will hold the value of the RIGHT sensor
 */
void readSensorSignals(int signals[]){
    #ifdef ARDUINO_UNO_CAR
        signals[0] = analogRead(IR_SIGNAL_PIN_LEFT) < ANALOG_THREATHOLD ? 1 : 0;
        signals[1] = analogRead(IR_SIGNAL_PIN_CENTER) < ANALOG_THREATHOLD ? 1 : 0;
        signals[2] = analogRead(IR_SIGNAL_PIN_RIGHT) < ANALOG_THREATHOLD ? 1 : 0;
    #else
        signals[0] = digitalRead(IR_SIGNAL_PIN_LEFT);
        signals[1] = digitalRead(IR_SIGNAL_PIN_CENTER);
        signals[2] = digitalRead(IR_SIGNAL_PIN_RIGHT);
    #endif
}

/**
 * @brief Detecs the position of an arduino car w.r.t. a line on the ground.
 * 
 * @param previousPosition hand over the last detected position so it can be taken into account for calculating the new position.
 * @return LinePosition the position of the car relative to the line.
 */
LinePosition detectPosition(LinePosition previousPosition){
    int signals[3];
    readSensorSignals(signals);
    if (signals[0] == 1 && signals[2] == 0){
        return RIGHT_OF_LINE;
    } else if (signals[0] == 0 && signals[1] == 1 && signals[2] == 0){
        return ON_LINE;
    } else if (signals[0] == 0 && signals[2] == 1){
        return LEFT_OF_LINE;
    } else { // if the sensor reading is unclear (e.g. because left and center detect the line at the same time), do not change the position
        return previousPosition;
    }
}