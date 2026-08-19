#pragma once
#include <AccelStepper.h>

class MotorController {
public:
    MotorController(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin,
                    uint8_t undockLimitPin, uint8_t dockLimitPin, bool invertDirection,
                    float maxSpeed, float acceleration);
    
    void init();
    void update(); // call in loop

    void startUndocking();
    void startDocking();
    void jog(bool towardUndock); // bench test: spin continuously, ignoring limit switches
    void stop();

    bool isUndocked();
    bool isDocked();
    bool isMoving();

private:
    AccelStepper stepper;
    uint8_t enablePin;
    uint8_t undockLimitPin;
    uint8_t dockLimitPin;
    float maxSpeed;
    float acceleration;
    bool moving;
    bool movingToUndock;
    bool movingToDock;
    bool jogging;

    // Debounced limit switch state (see LIMIT_DEBOUNCE_MS)
    bool undockDebounced;
    unsigned long undockPendingSince;
    bool dockDebounced;
    unsigned long dockPendingSince;
    void updateDebounce();
};
