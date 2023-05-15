#ifndef WIFI_DEBUG
#define WIFI_DEBUG

#include <Wire.h>
#include <Arduino.h>

void receiveI2CMessage(int numberOfBytesReceived)
{
    Serial.print("Message received: ");
    char i2cBuffer[numberOfBytesReceived];
    Wire.readBytes(i2cBuffer, numberOfBytesReceived);
    Serial.println(i2cBuffer);
}

class WifiDebug : public Print
{
public:
    inline static WifiDebug *instance;

    bool isOpen = false;

    WifiDebug()
    {
        WifiDebug::instance = this;
        Wire.begin(8);
        Wire.onReceive(receiveI2CMessage);
        digitalWrite(SDA, 0);
        digitalWrite(SCL, 0);
    }

    ~WifiDebug() {}

    size_t write(const uint8_t *buffer, size_t size)
    {
        if (size > 0)
        {
            if (!this->isOpen)
            {
                Wire.beginTransmission((int)9);
                this->isOpen = true;
            }
            Wire.write(buffer, size);
            if (buffer[size - 1] == '\n')
            {
                Wire.endTransmission();
                this->isOpen = false;
            }
        }
    }

    size_t write(uint8_t c)
    {
        if (!this->isOpen)
        {
            Wire.beginTransmission((int)9);
            this->isOpen = true;
        }
        Wire.write(c);
        if (c == '\n')
        {
            Wire.endTransmission();
            this->isOpen = false;
        }
    }
};

#endif