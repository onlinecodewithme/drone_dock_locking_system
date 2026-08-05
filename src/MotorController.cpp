#include "MotorController.h"
#include "Config.h"

MotorController::MotorController(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, 
                                 uint8_t undockLimitPin, uint8_t dockLimitPin)
    : stepper(AccelStepper::DRIVER, stepPin, dirPin),
      enablePin(enablePin),
      undockLimitPin(undockLimitPin),
      dockLimitPin(dockLimitPin),
      moving(false), movingToUndock(false), movingToDock(false) {}

void MotorController::init() {
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH); // Disabled by default
    
    pinMode(undockLimitPin, INPUT_PULLUP);
    pinMode(dockLimitPin, INPUT_PULLUP);

    stepper.setMaxSpeed(MOTOR_MAX_SPEED);
    stepper.setAcceleration(MOTOR_ACCELERATION);
}

void MotorController::startUndocking() {
    if (isUndocked()) return; // Already there
    
    digitalWrite(enablePin, LOW); // Enable driver
    moving = true;
    movingToUndock = true;
    movingToDock = false;
    
    // Reset position and speed so direction change is immediate
    stepper.setCurrentPosition(0);
    stepper.moveTo(MOTOR_CONTINUOUS_STEPS);
}

void MotorController::startDocking() {
    if (isDocked()) return; // Already there

    digitalWrite(enablePin, LOW);
    moving = true;
    movingToDock = true;
    movingToUndock = false;

    // Reset position and speed so direction change is immediate
    stepper.setCurrentPosition(0);
    stepper.moveTo(-MOTOR_CONTINUOUS_STEPS);
}

void MotorController::stop() {
    stepper.stop();
    moving = false;
    movingToUndock = false;
    movingToDock = false;
    digitalWrite(enablePin, HIGH); // Disable driver
}

void MotorController::update() {
    if (!moving) return;

    stepper.run();

    if (movingToUndock && isUndocked()) {
        stop();
    } else if (movingToDock && isDocked()) {
        stop();
    }
}

bool MotorController::isUndocked() {
    return digitalRead(undockLimitPin) == LOW; // Active LOW
}

bool MotorController::isDocked() {
    return digitalRead(dockLimitPin) == LOW;
}

bool MotorController::isMoving() {
    return moving;
}
