#pragma once
#include <AccelStepper.h>

class MotorController {
public:
    MotorController(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, 
                    uint8_t undockLimitPin, uint8_t dockLimitPin);
    
    void init();
    void update(); // call in loop

    void startUndocking();
    void startDocking();
    void stop();

    bool isUndocked();
    bool isDocked();
    bool isMoving();

private:
    AccelStepper stepper;
    uint8_t enablePin;
    uint8_t undockLimitPin;
    uint8_t dockLimitPin;
    bool moving;
    bool movingToUndock;
    bool movingToDock;
};
