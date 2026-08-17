#include "DockingSystem.h"
#include "Config.h"

DockingSystem::DockingSystem()
    : motor1(M1_STEP_PIN, M1_DIR_PIN, M1_ENABLE_PIN, M1_UNDOCK_LIMIT_PIN, M1_DOCK_LIMIT_PIN, true),
      motor2(M2_STEP_PIN, M2_DIR_PIN, M2_ENABLE_PIN, M2_UNDOCK_LIMIT_PIN, M2_DOCK_LIMIT_PIN, false),
      motorProp(M3_STEP_PIN, M3_DIR_PIN, M3_ENABLE_PIN, M3_OPEN_LIMIT_PIN, M3_CLOSE_LIMIT_PIN),
      currentState(SystemState::IDLE), lastReportedState(SystemState::IDLE),
      stateStartTime(0), undockingJustCompleted(false), dockingJustCompleted(false),
      testMotor(nullptr), testStartTime(0) {}

void DockingSystem::init() {
    motor1.init();
    motor2.init();
    motorProp.init();

    // Propeller closer must always rest closed — enforce it at boot in case
    // power was lost mid-cycle. motorProp.isDocked() == close limit triggered.
    if (!motorProp.isDocked()) {
        Serial.println("[PROP] Not at close limit on boot — closing...");
        motorProp.startDocking();
    }
}

void DockingSystem::commandUndock() {
    if (currentState != SystemState::IDLE) {
        Serial.println("System busy, cannot undock.");
        return;
    }
    Serial.println("Starting UNDOCK sequence: M1 -> M2");
    currentState = SystemState::UNDOCKING_M1;
    motor1.startUndocking();
}

void DockingSystem::commandDock() {
    if (currentState != SystemState::IDLE) {
        Serial.println("System busy, cannot dock.");
        return;
    }
    Serial.println("Starting DOCK sequence: M2 -> M1 -> PROP_OPEN -> PROP_CLOSE");
    currentState = SystemState::DOCKING_M2;
    motor2.startDocking();
}

void DockingSystem::commandJog(int motorNum) {
    if (currentState != SystemState::IDLE) {
        Serial.println("System busy, cannot jog.");
        return;
    }

    switch (motorNum) {
        case 1: testMotor = &motor1; break;
        case 2: testMotor = &motor2; break;
        case 3: testMotor = &motorProp; break;
        default:
            Serial.println("Unknown motor. Valid: JOG1, JOG2, JOG3");
            return;
    }

    Serial.print("[JOG] Spinning motor ");
    Serial.print(motorNum);
    Serial.print(" for ");
    Serial.print(JOG_DURATION_MS);
    Serial.println("ms...");

    testStartTime = millis();
    currentState = SystemState::TESTING;
    testMotor->jog(true); // spins toward "undock"/"open" direction, ignoring limit switches
}

void DockingSystem::commandReset() {
    Serial.println("[RESET] Stopping all actuators...");

    // Stop all motors immediately
    motor1.stop();
    motor2.stop();
    motorProp.stop();
    testMotor = nullptr;

    // Clear completion flags
    undockingJustCompleted = false;
    dockingJustCompleted = false;

    // Return to IDLE
    currentState = SystemState::IDLE;
    lastReportedState = SystemState::IDLE;

    Serial.println("[RESET] System recovered. State: IDLE");
}

void DockingSystem::commandStatus() {
    Serial.print("[STATUS] State=");
    Serial.print(getStateString());
    Serial.print(" M1_undock=");
    Serial.print(motor1.isUndocked() ? "HIT" : "open");
    Serial.print(" M1_dock=");
    Serial.print(motor1.isDocked() ? "HIT" : "open");
    Serial.print(" M2_undock=");
    Serial.print(motor2.isUndocked() ? "HIT" : "open");
    Serial.print(" M2_dock=");
    Serial.print(motor2.isDocked() ? "HIT" : "open");
    Serial.print(" Prop_open=");
    Serial.print(motorProp.isUndocked() ? "HIT" : "open");
    Serial.print(" Prop_close=");
    Serial.println(motorProp.isDocked() ? "HIT" : "open");
}

SystemState DockingSystem::getState() const {
    return currentState;
}

const char* DockingSystem::getStateString() const {
    // If a completion just happened, report it before returning to IDLE
    if (undockingJustCompleted) return "UNDOCKING_COMPLETE";
    if (dockingJustCompleted)  return "DOCKING_COMPLETE";

    switch (currentState) {
        case SystemState::IDLE:               return "IDLE";
        case SystemState::UNDOCKING_M1:       return "UNDOCKING_M1";
        case SystemState::UNDOCKING_M2:       return "UNDOCKING_M2";
        case SystemState::DOCKING_M2:         return "DOCKING_M2";
        case SystemState::DOCKING_M1:         return "DOCKING_M1";
        case SystemState::DOCKING_PROP_OPEN:  return "DOCKING_PROP_OPEN";
        case SystemState::DOCKING_PROP_CLOSE: return "DOCKING_PROP_CLOSE";
        case SystemState::TESTING:            return "TESTING";
        case SystemState::ERROR:              return "ERROR";
        default:                              return "UNKNOWN";
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
    motorProp.update();

    // Clear transient completion flags from the previous cycle
    undockingJustCompleted = false;
    dockingJustCompleted = false;

    switch (currentState) {
        case SystemState::IDLE:
        case SystemState::ERROR:
            break;

        // ── UNDOCK: M1 → M2 ──────────────────────────────────────────────
        // Propeller closer is not touched during undock — it stays closed.
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
                    Serial.println("UNDOCKING_COMPLETE");
                    undockingJustCompleted = true;
                    currentState = SystemState::IDLE;
                } else {
                    Serial.println("ERROR: M2 stopped but not undocked.");
                    currentState = SystemState::ERROR;
                }
            }
            break;

        // ── DOCK: M2 → M1 → PROP_OPEN → PROP_CLOSE ───────────────────────
        case SystemState::DOCKING_M2:
            if (!motor2.isMoving()) {
                if (motor2.isDocked()) {
                    Serial.println("M2 Docked. Starting M1...");
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
                    Serial.println("M1 Docked. Opening propeller closer...");
                    currentState = SystemState::DOCKING_PROP_OPEN;
                    motorProp.startUndocking(); // rotates toward the open limit
                } else {
                    Serial.println("ERROR: M1 stopped but not docked.");
                    currentState = SystemState::ERROR;
                }
            }
            break;

        case SystemState::DOCKING_PROP_OPEN:
            if (!motorProp.isMoving()) {
                if (motorProp.isUndocked()) { // open limit hit
                    Serial.println("Propeller closer open. Closing again...");
                    currentState = SystemState::DOCKING_PROP_CLOSE;
                    motorProp.startDocking(); // rotates back toward the close limit
                } else {
                    Serial.println("ERROR: Propeller closer stopped but not open.");
                    currentState = SystemState::ERROR;
                }
            }
            break;

        case SystemState::DOCKING_PROP_CLOSE:
            if (!motorProp.isMoving()) {
                if (motorProp.isDocked()) { // close limit hit
                    Serial.println("DOCKING_COMPLETE");
                    dockingJustCompleted = true;
                    currentState = SystemState::IDLE;
                } else {
                    Serial.println("ERROR: Propeller closer stopped but not closed.");
                    currentState = SystemState::ERROR;
                }
            }
            break;

        // ── TESTING: bench jog for a fixed JOG_DURATION_MS, ignoring limits ──
        case SystemState::TESTING:
            if (millis() - testStartTime >= JOG_DURATION_MS) {
                testMotor->stop();
                testMotor = nullptr;
                Serial.println("[JOG] Done.");
                currentState = SystemState::IDLE;
            }
            break;
    }
}
