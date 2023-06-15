/**
 * @file LineFollower.ino
 * @author david
 * @brief Demonstration code for a line follower car.
 */
#include <Wire.h>

// #define ARDUINO_UNO_CAR
// #define STEERED_CAR

#include "./MotorDriver.h"
#include "./LineDetector.c"
#include "./DistanceSensor.c"

//#define DO_MOTOR_DEBUG
// #define DO_OVERTAKE_REQUEST
// #define DO_COORDINATOR_SERIAL

float sensorThresholdLow[3] = {56, 79, 47};
float sensorThresholdHigh[3] = {138, 172, 113};
float angleSpeed = 10;

LinePosition position;
int baseSpeed = 60;
int8_t baseAngle = 15;
unsigned long lastSideChange = 0;
LinePosition lastPosition = ON_LINE;
uint8_t currSpeed = baseSpeed;
int8_t currAngle = 0;
#define SPEED_INCREASE_THRESHOLD 100

const byte ownI2CAddress = 8;
const byte coordinatorI2CAddress = 9;
uint8_t setDistance = 10;

char i2cBuffer[32];
boolean i2cNewMessageFlag = false;
uint8_t i2cBufferLen = 0;

#ifdef DO_OVERTAKE_REQUEST
char *overtakeRequest = "REQUEST_OVERTAKING";
char *approveOvertaking = "APPROVE_OVERTAKING";
char *cancelIllegal = "CANCEL_ILLEGAL";
char *finishedOvertaking = "FINISHED_OVERTAKING";
uint8_t overtakingState = 0; // 0: normal, 1: requested overtaking, 2: requested overtaking approved, 3: being overtaken requested+appoved
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
}

unsigned long lastCall = 0;

void loop()
{

    /*Serial.print(analogRead(A0));
    Serial.print(", ");
    Serial.print(analogRead(A1));
    Serial.print(", ");
    Serial.println(analogRead(A2));*/

    if (i2cNewMessageFlag)
    {
        Serial.println(i2cBuffer);
        i2cNewMessageFlag = false;
        if (strncmp(i2cBuffer, "STOP", 4) == 0)
        {
            drivingAllowed = false;
        }
        else if (strncmp(i2cBuffer, "START", 4) == 0)
        {
            drivingAllowed = true;
        }
        else if (strncmp(i2cBuffer, "DISTANCE", 8) == 0)
        {
            uint8_t i = 8;
            setDistance = 0;
            while (i < i2cBufferLen)
            {
                char c = i2cBuffer[i];
                if (c >= '0' && c <= '9')
                {
                    setDistance = setDistance * 10 + (c - '0');
                }
                i++;
            }
            Serial.print("Setting distance to ");
            Serial.println(setDistance);
        }
#ifdef DO_OVERTAKE_REQUEST
        else if (strncmp(i2cBuffer, overtakeRequest, 4) == 0)
        {
            if (overtakingState == 0 || overtakingState == 3 || overtakingState == 4)
            {
                sendI2CMessage(approveOvertaking);
                overtakingState = 3;
            }
            else
            {
                sendI2CMessage(cancelIllegal);
                overtakingState = 0;
            }
        }
        else if (strncmp(i2cBuffer, approveOvertaking, 4) == 0)
        {
            if (overtakingState == 1)
            {
                overtakingState = 2;
            }
            else
            {
                sendI2CMessage(cancelIllegal);
                overtakingState = 0;
            }
        }
        else if (strncmp(i2cBuffer, cancelIllegal, 4) == 0)
        {
            overtakingState = 0;
        }
        else if (strncmp(i2cBuffer, finishedOvertaking, 4) == 0)
        {
            if (overtakingState == 3)
            {
                overtakingState = 0;
            }
            else
            {
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
Serial.println(d*100);*/
#ifdef DO_OVERTAKE_REQUEST
    if (overtakingState == 2)
    {
        // handle overtaking
    }
    else if (overtakingState == 3)
    {
        Serial.println("Approved overtake request. Stopping");
        stop();
    }
    else
    {
        if (d < 20)
        {
            if (overtakingState == 0)
            {
                sendI2CMessage(overtakeRequest);
                overtakingState = 1;
                Serial.println("Less than 20cm. Sent overtake request. Slowing until approved.");
                followLine(baseSpeed / 2);
            }
            else if (overtakingState == 2)
            {
                Serial.println("Overtaking was approved, stopping as not implemented.");
                stop();
            }
            else if (overtakingState == 1)
            {
                if (d < setDistance)
                {
                    stop();
                }
                else
                {
                    followLine(baseSpeed / 2);
                }
            }
        }
        else
        {
#endif
            if (d < setDistance)
            {
                stop();
            }
            else if (!drivingAllowed)
            {
                stop();
            }
            else
            {
                followLine(baseSpeed);
                // analogFollowLine();
            }
#ifdef DO_OVERTAKE_REQUEST
        }
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

void analogFollowLine()
{
    float signals[3];
    signals[0] = analogRead(A0);
    signals[1] = analogRead(A1);
    signals[2] = analogRead(A2);
    /*Serial.print(signals[0]);
    Serial.print(", ");
    Serial.print(signals[1]);
    Serial.print(", ");
    Serial.print(signals[2]);
    Serial.print(", ");*/
    float pointX = 0;
    float pointY = 0;
    float sum = 0;
    uint8_t maxIndex = 255;
    float maxVal = -1;
    for (uint8_t i = 0; i < 3; i++)
    {
        signals[i] = 1 - max(0, min(1, (signals[i] - sensorThresholdLow[i]) / (sensorThresholdHigh[i] - sensorThresholdLow[i])));
        pointX += cos(i * PI * 2.0 / 3.0) * signals[i];
        pointY += sin(i * PI * 2.0 / 3.0) * signals[i];
        if (signals[i] > maxVal)
        {
            maxIndex = i;
            maxVal = signals[i];
        }
    }
    sum = sqrt(pointX * pointX + pointY * pointY);
    Serial.print("s:");
    Serial.print(sum);
    Serial.print(",l:");
    Serial.print(signals[0]);
    Serial.print(",m:");
    Serial.print(signals[1]);
    Serial.print(",r:");
    Serial.println(signals[2]);
    // Serial.print(", ");

    unsigned long now = millis();
    if (now - lastSideChange > SPEED_INCREASE_THRESHOLD)
    {
        if (sum > 0.3)
        {
            switch (maxIndex)
            {
            case 0: // RIGHT_OF_LINE
                currAngle = max(-90, min(-baseAngle, currAngle - sum * angleSpeed));
                break;
            case 1: // ON_LINE
                if (sum > 0.8)
                {
                    currAngle = 0;
                }
                else
                {
                    if (currAngle > 0)
                    {
                        currAngle = max(0, currAngle - sum * angleSpeed);
                    }
                    else
                    {
                        currAngle = min(0, currAngle + sum * angleSpeed);
                    }
                }
                break;
            case 2: // LEFT_OF_LINE
                currAngle = min(90, max(baseAngle, currAngle + sum * angleSpeed));
                break;
            }
        }
        setSteeringAngle(currAngle);
        lastSideChange = now;
    }
    // Serial.println(currAngle);
    setSpeed(baseSpeed);
    leftForward();
    rightForward();
}

void followLine(int speed)
{
    int signals[3];
    readSensorSignals(signals);
    Serial.print(",Left:");
    Serial.print(signals[0]);
    Serial.print(",Middle:");
    Serial.print(signals[1]);
    Serial.print(",Right:");
    Serial.println(signals[2]);

    unsigned long now = millis();
    position = detectPosition(position);

    if (position != lastPosition)
    {
        lastSideChange = now;
        currSpeed = baseSpeed;
        currAngle = baseAngle;
        lastPosition = position;
    }

    if (position == ON_LINE)
    {
         Serial.println("On line!");
        driveForward(currSpeed);
    }
    else if (position == LEFT_OF_LINE)
    {
         Serial.println("LEFT of line!");
        if (now - lastSideChange > SPEED_INCREASE_THRESHOLD)
        {
            // currSpeed = max(60, currSpeed*0.95);
            currAngle = min(90, currAngle + 5);
            if (currAngle > 45)
            {
                // currSpeed = 60;
            }
            lastSideChange = now;
        }
        /*if (signals[0] == 0 && signals[1] == 0 && signals[2] == 0)
        {
            setSteeringAngle(90);
            setSpeed(55);
        }
        else
        {*/
        turnRightForward(currSpeed * 1.5);
        setSpeed(currSpeed);
        //}
    }
    else if (position == RIGHT_OF_LINE)
    {
         Serial.println("RIGHT of line");
        if (now - lastSideChange > SPEED_INCREASE_THRESHOLD)
        {
            // currSpeed = max(60, currSpeed*0.95);
            currAngle = min(90, currAngle + 5);
            if (currAngle > 45)
            {
                // currSpeed = 60;
            }
            lastSideChange = now;
        }
        /*if (signals[0] == 0 && signals[1] == 0 && signals[2] == 0)
        {
            setSteeringAngle(-90);
            setSpeed(55);
        }
        else
        {*/
        turnLeftForward(currSpeed * 1.5);
        setSpeed(currSpeed);
        //}
    }
    else
    {
        // Serial.println("This is strange...");
        stop();
    }
}

void receiveI2CMessage(int numberOfBytesReceived)
{
    // Serial.println("Message received!");
    memset(i2cBuffer, 0, sizeof(i2cBuffer));
    Wire.readBytes(i2cBuffer, numberOfBytesReceived);
    i2cNewMessageFlag = true;
    i2cBufferLen = numberOfBytesReceived;
}

void sendI2CMessage(char *data)
{
    Serial.print("Sending states: ");
    Wire.beginTransmission((int)coordinatorI2CAddress);
    Serial.print(Wire.write(data));
    Serial.print(" ");
    Serial.println(Wire.endTransmission());
}

uint8_t speed, speedBuffer, motorNum = 255, directionBuffer, i2cSendBufferIndex = 255;
char i2cSendBuffer[32];

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
            if (directionBuffer != 0)
            {
                switch (motorNum)
                {
                case 0:
                    frontLeftDirection(directionBuffer);
                    break;
                case 1:
                    frontRightDirection(directionBuffer);
                    break;
                case 2:
                    rearLeftDirection(directionBuffer);
                    break;
                case 3:
                    rearRightDirection(directionBuffer);
                    break;
                case 254:
                    break;
                default:
                    setDirection(directionBuffer);
                }
                Serial.print("Set direction of motor ");
                Serial.print(motorNum);
                Serial.print(" to ");
                Serial.println(directionBuffer);
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
            case 254:
                if (directionBuffer == 1)
                {
                    setSteeringAngle(-speed);
                }
                else
                {
                    setSteeringAngle(speed);
                }
                break;
            default:
                setSpeed(speed);
            }
            Serial.print("Set speed of motor ");
            Serial.print(motorNum);
            Serial.print(" to ");
            Serial.println(speed);
            motorNum = 255;
            directionBuffer = 0;
            if (i2cSendBufferIndex <= 32)
            {
                i2cSendBuffer[i2cSendBufferIndex++] = 0;
                sendI2CMessage(i2cSendBuffer);
                Serial.print("Sent to I2C: ");
                Serial.println(i2cSendBuffer);
                i2cSendBufferIndex = 255;
            }
        }
        else if (i2cSendBufferIndex < 32)
        {
            i2cSendBuffer[i2cSendBufferIndex++] = read;
        }
        else if (read >= '0' && read <= '9')
        {
            speedBuffer = (speedBuffer * 10) + (read - '0');
        }
        else if (read == 'x')
        {
            Serial.println("Boosting");
            if (speed > 0)
            {
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
        else if (read == '-')
        {
            directionBuffer = MOTOR_BACKWARD;
        }
        else if (read == '+')
        {
            directionBuffer = MOTOR_FORWARD;
        }
        else if (read == '>')
        {
            i2cSendBufferIndex = 0;
        }
        else if (read == '/')
        {
            motorNum = 254;
        }
    }
}