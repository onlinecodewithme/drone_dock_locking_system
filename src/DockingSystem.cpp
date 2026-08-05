#include "DockingSystem.h"
#include "Config.h"

DockingSystem::DockingSystem() 
    : motor1(M1_STEP_PIN, M1_DIR_PIN, M1_ENABLE_PIN, M1_UNDOCK_LIMIT_PIN, M1_DOCK_LIMIT_PIN),
      motor2(M2_STEP_PIN, M2_DIR_PIN, M2_ENABLE_PIN, M2_UNDOCK_LIMIT_PIN, M2_DOCK_LIMIT_PIN),
      currentState(SystemState::IDLE), lastReportedState(SystemState::IDLE),
      stateStartTime(0), undockingJustCompleted(false), dockingJustCompleted(false),
      servoCurrentAngle(0), servoTargetAngle(0), servoLastStepTime(0), servoMoving(false) {}

void DockingSystem::init() {
    motor1.init();
    motor2.init();
    
    // Load last servo angle from NVS (survives power cycles)
    int savedAngle = loadServoAngle();
    servoCurrentAngle = savedAngle;
    servoTargetAngle  = savedAngle;
    
    // Servo configuration for 270 degrees (common range: 500-2500us)
    servo.setPeriodHertz(50);
    servo.attach(SERVO_PIN, 500, 2500);
    servo.write(savedAngle);  // restore to last known position — no movement
    
    Serial.print("[SERVO] Restored to ");
    Serial.print(savedAngle);
    Serial.println("° from NVS");
}

void DockingSystem::commandUndock() {
    if (currentState != SystemState::IDLE) {
        Serial.println("System busy, cannot undock.");
        return;
    }
    Serial.println("Starting UNDOCK sequence: M1 -> M2 -> SERVO");
    currentState = SystemState::UNDOCKING_M1;
    motor1.startUndocking();
}

void DockingSystem::commandDock() {
    if (currentState != SystemState::IDLE) {
        Serial.println("System busy, cannot dock.");
        return;
    }
    Serial.println("Starting DOCK sequence: M2 -> M1 -> SERVO");
    currentState = SystemState::DOCKING_M2;
    motor2.startDocking();
}

// --- Internal servo helpers ---

void DockingSystem::startServo(int targetAngle) {
    servoTargetAngle   = targetAngle;
    servoLastStepTime  = millis();
    servoMoving        = true;
    Serial.print("[SERVO] Sweeping from ");
    Serial.print(servoCurrentAngle);
    Serial.print("° to ");
    Serial.print(targetAngle);
    Serial.println("°...");
}

bool DockingSystem::updateServo() {
    if (!servoMoving) return true; // already at target

    unsigned long now = millis();
    if (now - servoLastStepTime < SERVO_STEP_INTERVAL_MS) return false;
    servoLastStepTime = now;

    // Degrees to move this step
    float degreesPerStep = SERVO_SPEED_DEG_PER_SEC * (SERVO_STEP_INTERVAL_MS / 1000.0f);

    if (servoCurrentAngle < servoTargetAngle) {
        servoCurrentAngle = min((float)servoTargetAngle, servoCurrentAngle + degreesPerStep);
    } else if (servoCurrentAngle > servoTargetAngle) {
        servoCurrentAngle = max((float)servoTargetAngle, servoCurrentAngle - degreesPerStep);
    }

    servo.write((int)servoCurrentAngle);

    if ((int)servoCurrentAngle == servoTargetAngle) {
        servoMoving = false;
        saveServoAngle(servoTargetAngle);  // persist to NVS
        Serial.print("[SERVO] Reached ");
        Serial.print(servoTargetAngle);
        Serial.println("° (saved to NVS)");
        return true; // done
    }
    return false; // still moving
}

// --- NVS persistence ---

void DockingSystem::saveServoAngle(int angle) {
    prefs.begin("dock", false);  // read-write
    prefs.putInt("servoAngle", angle);
    prefs.end();
}

int DockingSystem::loadServoAngle() {
    prefs.begin("dock", true);  // read-only
    int angle = prefs.getInt("servoAngle", SERVO_DOCK_ANGLE);  // default to 0° if first boot
    prefs.end();
    return angle;
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
        case SystemState::DOCKING_M2:      return "DOCKING_M2";
        case SystemState::DOCKING_M1:      return "DOCKING_M1";
        case SystemState::DOCKING_SERVO:   return "DOCKING_SERVO";
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
    updateServo();  // always tick the servo sweep

    // Clear transient completion flags from the previous cycle
    undockingJustCompleted = false;
    dockingJustCompleted = false;

    switch (currentState) {
        case SystemState::IDLE:
        case SystemState::ERROR:
            break;

        // ── UNDOCK: M1 → M2 → SERVO ──────────────────────────────────────
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
                    Serial.println("M2 Undocked. Sweeping Servo to 270...");
                    currentState = SystemState::UNDOCKING_SERVO;
                    startServo(SERVO_UNDOCK_ANGLE);
                } else {
                    Serial.println("ERROR: M2 stopped but not undocked.");
                    currentState = SystemState::ERROR;
                }
            }
            break;

        case SystemState::UNDOCKING_SERVO:
            if (updateServo()) {
                Serial.println("UNDOCKING_COMPLETE");
                undockingJustCompleted = true;
                currentState = SystemState::IDLE;
            }
            break;

        // ── DOCK: M2 → M1 → SERVO ────────────────────────────────────────
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
                    Serial.println("M1 Docked. Sweeping Servo to 0...");
                    currentState = SystemState::DOCKING_SERVO;
                    startServo(SERVO_DOCK_ANGLE);
                } else {
                    Serial.println("ERROR: M1 stopped but not docked.");
                    currentState = SystemState::ERROR;
                }
            }
            break;

        case SystemState::DOCKING_SERVO:
            if (updateServo()) {
                Serial.println("DOCKING_COMPLETE");
                dockingJustCompleted = true;
                currentState = SystemState::IDLE;
            }
            break;
    }
}
