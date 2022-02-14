# The Arduino Car

This Arduino Car is used for the Master's thesis of David (@Code-Schwabe). It has the capabilities to
* drive autonomously following a line on the ground
* measure the distance in N directions (tbd)
* communicate with another Arduino Car using WiFi

## Hardware

The hardware in use is the following:
* 1 Arduino Mega 2560 Rev3
* 1 Arduino Nano Rev3
* 1 chassis with 4 simple DC motors
* 1 motor controller L298N
* 1 WiFi shield ESP8266-01S
* 3 infrared sensors KY-033
* N ultrasonic distance sensors HC-SR04 (tdb)
* ...many cables, glue and a breadboard

It can be powered by any 12V power source - for mobility reasons, a 3 cell LiPo battery is used in our current set-up.

The hardware is wired as depicted here:
![](wirings/arduino%20car.png)

| :warning: The wiring isn't complete yet! |
| --- |

## Software

The software in this repository has two purposes:
* Demonstrate the capabilities of the Arduino Car.
* Provide device libararies for the car's capabilities (continuous components) that can later be included in the code generation process.

### LineFollower

The LineFollower libraries implement the interaction with the sensors (infrared) and actuators (dc motors via motor controller) in order to make the car drive autonomously while following a black line on the ground.  

### CarCoordinator (TBD)

The CarCoordinator libraries encapsulate the behavior of detecting other cars in close distance and communicating with them.