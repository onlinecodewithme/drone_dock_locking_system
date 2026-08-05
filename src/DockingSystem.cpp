#include "DockingSystem.h"
#include "Config.h"

DockingSystem::DockingSystem() 
    : motor1(M1_STEP_PIN, M1_DIR_PIN, M1_ENABLE_PIN, M1_UNDOCK_LIMIT_PIN, M1_DOCK_LIMIT_PIN),
      motor2(M2_STEP_PIN, M2_DIR_PIN, M2_ENABLE_PIN, M2_UNDOCK_LIMIT_PIN, M2_DOCK_LIMIT_PIN),
      currentState(SystemState::IDLE), lastReportedState(SystemState::IDLE),
      stateStartTime(0), undockingJustCompleted(false), dockingJustCompleted(false) {}

void DockingSystem::init() {
    motor1.init();
    motor2.init();
    
    // Servo configuration for 270 degrees (common range: 500-2500us)
    servo.setPeriodHertz(50);
    servo.attach(SERVO_PIN, 500, 2500); 
}

void DockingSystem::commandUndock() {
    if (currentState != SystemState::IDLE) {
        Serial.println("System busy, cannot undock.");
        return;
    }
    Serial.println("Starting UNDOCK sequence...");
    currentState = SystemState::UNDOCKING_M1;
    motor1.startUndocking();
}

void DockingSystem::commandDock() {
    if (currentState != SystemState::IDLE) {
        Serial.println("System busy, cannot dock.");
        return;
    }
    Serial.println("Starting DOCK sequence...");
    currentState = SystemState::DOCKING_SERVO;
    servo.write(SERVO_DOCK_ANGLE);
    stateStartTime = millis();
}

SystemState DockingSystem::getState() const {
    return currentState;
}

const char* DockingSystem::getStateString() const {
    // If a completion just happened, report it before returning to IDLE
    if (undockingJustCompleted) return "UNDOCKING_COMPLETE";
    if (dockingJustCompleted)  return "DOCKING_COMPLETE";

    switch (currentState) {
        case SystemState::IDLE:            return "IDLE";
        case SystemState::UNDOCKING_M1:    return "UNDOCKING_M1";
        case SystemState::UNDOCKING_M2:    return "UNDOCKING_M2";
        case SystemState::UNDOCKING_SERVO: return "UNDOCKING_SERVO";
        case SystemState::DOCKING_SERVO:   return "DOCKING_SERVO";
        case SystemState::DOCKING_M2:      return "DOCKING_M2";
        case SystemState::DOCKING_M1:      return "DOCKING_M1";
        case SystemState::ERROR:           return "ERROR";
        default:                           return "UNKNOWN";
    }
}

bool DockingSystem::stateChanged() {
    // Also trigger on transient completion flags
    if (undockingJustCompleted || dockingJustCompleted) {
        return true;
    }
    if (currentState != lastReportedState) {
        lastReportedState = currentState;
        return true;
    }
    return false;
}

void DockingSystem::update() {
    motor1.update();
    motor2.update();

    // Clear transient completion flags from the previous cycle
    undockingJustCompleted = false;
    dockingJustCompleted = false;

    switch (currentState) {
        case SystemState::IDLE:
        case SystemState::ERROR:
            // Do nothing
            break;

        case SystemState::UNDOCKING_M1:
            if (!motor1.isMoving()) {
                if (motor1.isUndocked()) {
                    Serial.println("M1 Undocked. Starting M2...");
                    currentState = SystemState::UNDOCKING_M2;
                    motor2.startUndocking();
                } else {
                    Serial.println("ERROR: M1 stopped but not undocked.");
                    currentState = SystemState::ERROR;
                }
            }
            break;

        case SystemState::UNDOCKING_M2:
            if (!motor2.isMoving()) {
                if (motor2.isUndocked()) {
                    Serial.println("M2 Undocked. Moving Servo to 270...");
                    currentState = SystemState::UNDOCKING_SERVO;
                    servo.write(SERVO_UNDOCK_ANGLE);
                    stateStartTime = millis();
                } else {
                    Serial.println("ERROR: M2 stopped but not undocked.");
                    currentState = SystemState::ERROR;
                }
            }
            break;

        case SystemState::UNDOCKING_SERVO:
            // Time-based: wait for servo to physically reach target
            if (millis() - stateStartTime >= SERVO_SETTLE_MS) {
                Serial.println("Servo reached 270. UNDOCKING_COMPLETE");
                undockingJustCompleted = true;
                currentState = SystemState::IDLE;
            }
            break;

        case SystemState::DOCKING_SERVO:
            // Time-based: wait for servo to physically reach target
            if (millis() - stateStartTime >= SERVO_SETTLE_MS) {
                Serial.println("Servo reached 0. Starting M2 reverse...");
                currentState = SystemState::DOCKING_M2;
                motor2.startDocking();
            }
            break;

        case SystemState::DOCKING_M2:
            if (!motor2.isMoving()) {
                if (motor2.isDocked()) {
                    Serial.println("M2 Docked. Starting M1 reverse...");
                    currentState = SystemState::DOCKING_M1;
                    motor1.startDocking();
                } else {
                    Serial.println("ERROR: M2 stopped but not docked.");
                    currentState = SystemState::ERROR;
                }
            }
            break;

        case SystemState::DOCKING_M1:
            if (!motor1.isMoving()) {
                if (motor1.isDocked()) {
                    Serial.println("DOCKING_COMPLETE");
                    dockingJustCompleted = true;
                    currentState = SystemState::IDLE;
                } else {
                    Serial.println("ERROR: M1 stopped but not docked.");
                    currentState = SystemState::ERROR;
                }
            }
            break;
    }
}
