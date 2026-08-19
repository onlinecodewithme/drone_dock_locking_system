#include "MotorController.h"
#include "Config.h"

MotorController::MotorController(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin,
                                 uint8_t undockLimitPin, uint8_t dockLimitPin, bool invertDirection,
                                 float maxSpeed, float acceleration)
    : stepper(AccelStepper::DRIVER, stepPin, dirPin),
      enablePin(enablePin),
      undockLimitPin(undockLimitPin),
      dockLimitPin(dockLimitPin),
      maxSpeed(maxSpeed),
      acceleration(acceleration),
      moving(false), movingToUndock(false), movingToDock(false), jogging(false),
      undockDebounced(false), undockPendingSince(0), dockDebounced(false), dockPendingSince(0) {
    stepper.setPinsInverted(invertDirection, false, false);
}

void MotorController::init() {
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH); // Disabled by default

    pinMode(undockLimitPin, INPUT_PULLUP);
    pinMode(dockLimitPin, INPUT_PULLUP);

    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(acceleration);

    // Seed debounced state from the real pin reading so init doesn't report a false transition
    undockDebounced = digitalRead(undockLimitPin) == LOW;
    dockDebounced = digitalRead(dockLimitPin) == LOW;
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

void MotorController::jog(bool towardUndock) {
    digitalWrite(enablePin, LOW); // Enable driver
    moving = true;
    jogging = true;
    movingToUndock = false;
    movingToDock = false;

    stepper.setCurrentPosition(0);
    stepper.moveTo(towardUndock ? MOTOR_CONTINUOUS_STEPS : -MOTOR_CONTINUOUS_STEPS);
}

void MotorController::stop() {
    stepper.stop();
    moving = false;
    movingToUndock = false;
    movingToDock = false;
    jogging = false;
    digitalWrite(enablePin, HIGH); // Disable driver
}

void MotorController::update() {
    updateDebounce(); // must run every tick, even when idle, so isUndocked()/isDocked() stay current

    if (!moving) return;

    stepper.run();

    if (jogging) return; // bench test: run until timed stop(), ignore limit switches

    if (movingToUndock && isUndocked()) {
        stop();
    } else if (movingToDock && isDocked()) {
        stop();
    }
}

void MotorController::updateDebounce() {
    unsigned long now = millis();

    bool undockRaw = digitalRead(undockLimitPin) == LOW; // Active LOW
    if (undockRaw != undockDebounced) {
        if (undockPendingSince == 0) {
            undockPendingSince = now;
        } else if (now - undockPendingSince >= LIMIT_DEBOUNCE_MS) {
            undockDebounced = undockRaw;
            undockPendingSince = 0;
        }
    } else {
        undockPendingSince = 0;
    }

    bool dockRaw = digitalRead(dockLimitPin) == LOW;
    if (dockRaw != dockDebounced) {
        if (dockPendingSince == 0) {
            dockPendingSince = now;
        } else if (now - dockPendingSince >= LIMIT_DEBOUNCE_MS) {
            dockDebounced = dockRaw;
            dockPendingSince = 0;
        }
    } else {
        dockPendingSince = 0;
    }
}

bool MotorController::isUndocked() {
    return undockDebounced;
}

bool MotorController::isDocked() {
    return dockDebounced;
}

bool MotorController::isMoving() {
    return moving;
}
