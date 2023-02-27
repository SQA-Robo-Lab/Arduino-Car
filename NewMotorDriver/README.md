# New Motor Driver (Hardware abstraction libarary)
Library to abstract away the interaction with the hardware of the car and unify utilization between different car types.
## Class diagram
### Simplified (Functional block diagram)

### Full class diagrm
```mermaid
classDiagram
    DriveController <|-- FixedWheelDriveController
    DriveController <|-- TurnSteeringDriveController
    DriveController --> "1 state" DriveControllerState
    class DriveController{
        #int16_t speed
        #int8_t angle

        +drive(int8_t angle, int16_t speed)
        +pause()
        +resume()

        +setSpeed(int16_t speed)
        +setAngle(int8_t angle)
        +int16_t getSpeed()
        +int8_t getAngle()

        +loop()*

        #setSpeedInternal(int16_t speed)*
        #setAngleInternal(int8_t angle)*
    }
    class FixedWheelDriveController{
        +loop()

        #setSpeedInternal(int16_t speed)
        #setAngleInternal(int8_t angle)
    }
    class TurnSteeringDriveController{
        +loop()

        #setSpeedInternal(int16_t speed)
        #setAngleInternal(int8_t angle)
    }
    class DriveControllerState {
        <<enumeration>>
        DRIVE_CONTROLLER_STATE_STOPPED
        DRIVE_CONTROLLER_STATE_PAUSED
        DRIVE_CONTROLLER_STATE_DRIVING
    }
```