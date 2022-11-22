/**
 * @file LineFollower.ino
 * @author david
 * @brief Demonstration code for a line follower car.
 */
#include <Wire.h>

#include "./MotorDriver.h"
#include "./LineDetector.c"
#include "./DistanceSensor.c"

//#define DO_MOTOR_DEBUG
#define DO_OVERTAKE_REQUEST
#define DO_COORDINATOR_SERIAL


LinePosition position;
int baseSpeed = 60;

const byte ownI2CAddress = 8;
const byte coordinatorI2CAddress = 9;

char i2cBuffer[32];
boolean i2cNewMessageFlag = false;

#ifdef DO_OVERTAKE_REQUEST
char* overtakeRequest = "REQUEST_OVERTAKING";
char* approveOvertaking = "APPROVE_OVERTAKING";
char* cancelIllegal = "CANCEL_ILLEGAL";
char* finishedOvertaking = "FINISHED_OVERTAKING";
uint8_t overtakingState = 0; //0: normal, 1: requested overtaking, 2: requested overtaking approved, 3: being overtaken requested+appoved, 4: being overtaken active
#endif

boolean drivingAllowed = true;

void setup()
{
    Wire.begin(ownI2CAddress);
    Wire.onReceive(receiveI2CMessage);
    digitalWrite(SDA, 0);
    digitalWrite(SCL, 0);

    Serial.begin(115200);
    initMotorDriver();

    initLineDetector();
    position = detectPosition(ON_LINE);

    initializeDistanceSensors();

    Serial.println("Setup complete!");

    #ifdef DO_MOTOR_DEBUG
        motorDebugSetup();
    #endif

    Serial.println("Counter 5 mode: ");
    Serial.println(TCCR5A);
    Serial.println(TCCR5B);
    Serial.println(TCCR5C);
}

unsigned long lastCall = 0;

void loop()
{
    if (i2cNewMessageFlag)
    {
        Serial.println(i2cBuffer);
        i2cNewMessageFlag = false;
        if (strncmp(i2cBuffer, "STOP", 4) == 0)
        {
            drivingAllowed = false;
        } else if (strncmp(i2cBuffer, "START", 4) == 0) {
            drivingAllowed = true;
        }
        #ifdef DO_OVERTAKE_REQUEST
        else if (strncmp(i2cBuffer, overtakeRequest, 4) == 0) {
            if (overtakingState == 0 || overtakingState == 3 || overtakingState == 4) {
                sendI2CMessage(approveOvertaking);
                overtakingState = 3;
            } else {
                sendI2CMessage(cancelIllegal);
                overtakingState = 0;
            }
        } else if (strncmp(i2cBuffer, approveOvertaking, 4) == 0) {
            if (overtakingState == 1) {
                overtakingState = 2;
            } else {
                sendI2CMessage(cancelIllegal);
                overtakingState = 0;
            }
        } else if (strncmp(i2cBuffer, cancelIllegal, 4) == 0) {
            overtakingState = 0;
        } else if (strncmp(i2cBuffer, finishedOvertaking, 4) == 0) {
            if (overtakingState == 3) {
                overtakingState = 0;
            } else {
                sendI2CMessage(cancelIllegal);
            }
        }
        Serial.print("Overtaking state:");
        Serial.println(overtakingState);
        Serial.println((int)i2cBuffer[0]);
        #endif
        // Serial.println(drivingAllowed);
    }

    #ifdef DO_MOTOR_DEBUG
        motorDebugLoop();
    #else
        uint32_t d = getFrontDistance();
        /*Serial.print(",Front:");
        Serial.print(d*100);*/
        #ifdef DO_OVERTAKE_REQUEST
            if (overtakingState == 2) {
                //handle overtaking
            } else {
        #endif
        if (d < 10)
        {
            stop();
        }
        else if (!drivingAllowed)
        {
            stop();
        }
        #ifdef DO_OVERTAKE_REQUEST
            else if (overtakingState == 3) {
                Serial.println("Approved overtake request. Stopping");
                stop();
            }
            else if (d < 20) {
                if (overtakingState == 0) {
                    sendI2CMessage(overtakeRequest);
                    overtakingState = 1;
                    Serial.println("Less than 20cm. Sent overtake request. Slowing until approved.");
                    followLine(baseSpeed/2);
                } else if (overtakingState == 2) {
                    Serial.println("Overtaking was approved, stopping as not implemented.");
                    stop();
                } else if (overtakingState == 1) {
                    followLine(baseSpeed/2);
                }
            }
        #endif
        else
        {
            followLine(baseSpeed);
        }
        #ifdef DO_OVERTAKE_REQUEST
            }
        #endif
    #endif
    unsigned long now = micros();
    /*Serial.print(",Cycle_time:");
    Serial.println(now - lastCall);*/
    lastCall = now;
    // delay(100);
    //  followLine(baseSpeed);
}

void followLine(int speed)
{
    /*int signals[3];
    readSensorSignals(signals);
    Serial.print(",Left:");
    Serial.print(signals[0]*50000);
    Serial.print(",Middle:");
    Serial.print(signals[1]*50000);
    Serial.print(",Right:");
    Serial.print(signals[2]*50000);*/
    position = detectPosition(position);
    if (position == ON_LINE)
    {
        //Serial.println("On line!");
        driveForward(speed);
    }
    else if (position == LEFT_OF_LINE)
    {
        //Serial.println("LEFT of line!");
        turnRightForward(speed * 1.5);
    }
    else if (position == RIGHT_OF_LINE)
    {
        //Serial.println("RIGHT of line");
        turnLeftForward(speed * 1.5);
    }
    else
    {
        //Serial.println("This is strange...");
        stop();
    }
}

void receiveI2CMessage(int numberOfBytesReceived)
{
    // Serial.println("Message received!");
    memset(i2cBuffer, 0, sizeof(i2cBuffer));
    Wire.readBytes(i2cBuffer, numberOfBytesReceived);
    i2cNewMessageFlag = true;
}

void sendI2CMessage(char* data) {
    Wire.beginTransmission((int) coordinatorI2CAddress);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t speed, speedBuffer, motorNum;

void motorDebugSetup()
{
    leftForward();
    rightForward();
}

void motorDebugLoop()
{
    while (Serial.available())
    {
        int read = Serial.read();
        if (read == '\n')
        {
            speed = speedBuffer;
            speedBuffer = 0;
            if (speed > 0) {
                forward();
            }
            switch (motorNum)
            {
            case 0:
                frontLeftSpeed(speed);
                break;
            case 1:
                frontRightSpeed(speed);
                break;
            case 2:
                rearLeftSpeed(speed);
                break;
            case 3:
                rearRightSpeed(speed);
                break;
            default:
                setSpeed(speed);
            }
            Serial.print("Set speed of motor ");
            Serial.print(motorNum);
            Serial.print(" to ");
            Serial.println(speed);
            motorNum = 255;
        }
        else if (read >= '0' && read <= '9')
        {
            speedBuffer = (speedBuffer * 10) + (read - '0');
        }
        else if (read == 'x')
        {
            Serial.println("Boosting");
            if (speed > 0) {
                forward();
            }
            switch (motorNum)
            {
            case 0:
                frontLeftSpeed(80);
                break;
            case 1:
                frontRightSpeed(80);
                break;
            case 2:
                rearLeftSpeed(80);
                break;
            case 3:
                rearRightSpeed(80);
                break;
            default:
                setSpeed(80);
            }
            delay(25);
        }
        else if (read >= 'a' && read <= 'd')
        {
            motorNum = read - 'a';
        }
    }
}