#include "DockingSystem.h"
#include "Config.h"

DockingSystem::DockingSystem()
    : motor1(M1_STEP_PIN, M1_DIR_PIN, M1_ENABLE_PIN, M1_UNDOCK_LIMIT_PIN, M1_DOCK_LIMIT_PIN, true),
      motor2(M2_STEP_PIN, M2_DIR_PIN, M2_ENABLE_PIN, M2_UNDOCK_LIMIT_PIN, M2_DOCK_LIMIT_PIN, false),
      motorProp(M3_STEP_PIN, M3_DIR_PIN, M3_ENABLE_PIN, M3_OPEN_LIMIT_PIN, M3_CLOSE_LIMIT_PIN),
      currentState(SystemState::UNKNOWN), lastReportedState(SystemState::UNKNOWN),
      stageStartMs(0), stageRetries(0), awaitingRetry(false), retryReadyAtMs(0),
      testMotor(nullptr), testStartTime(0), lastRestCheckMs(0) {}

void DockingSystem::init() {
    motor1.init();
    motor2.init();
    motorProp.init();

    // Propeller closer must always rest closed — enforce it at boot in case
    // power was lost mid-cycle. Blocks here, but reuses the same
    // timeout+bounded-retry logic as every other stage (runStage()) rather
    // than a one-off wait — nothing else is running yet at this point in
    // setup(), so blocking is fine.
    bool propOk = true;
    if (!motorProp.isDocked()) {
        Serial.println("[PROP] Not at close limit on boot — closing...");
        motorProp.startDocking();
        beginStage();
        bool closed = false;
        while (!closed && currentState != SystemState::ERROR) {
            motorProp.update();
            closed = runStage(motorProp, motorProp.isDocked(), "boot propeller close",
                               [this]() { motorProp.startDocking(); });
            delay(2);
        }
        propOk = closed;
    }

    if (propOk) {
        // Only re-derive from switches if the propeller genuinely settled —
        // if runStage() gave up above, currentState is already ERROR and
        // that's a real fault worth preserving, not papering over.
        currentState = inferRestingState();
    }
    lastReportedState = currentState; // nothing is subscribed yet — nothing to notify
    Serial.print("[BOOT] Resting state verified as: ");
    Serial.println(getStateString());
}

void DockingSystem::commandUndock() {
    if (isBusy()) {
        Serial.println("System busy, cannot undock.");
        return;
    }
    Serial.println("Starting UNDOCK sequence: M1 -> M2");
    // Start the motor BEFORE publishing the new state. onWrite() runs on the
    // BLE stack's own FreeRTOS task, concurrently with update() on the main
    // loop task — if currentState went to UNDOCKING_M1 first, update() could
    // observe it before motor1.moving actually flips true.
    motor1.startUndocking();
    enterStage(SystemState::UNDOCKING_M1);
}

void DockingSystem::commandDock() {
    if (isBusy()) {
        Serial.println("System busy, cannot dock.");
        return;
    }
    Serial.println("Starting DOCK sequence: M2 -> M1 -> PROP_OPEN -> PROP_CLOSE");
    motor2.startDocking();
    enterStage(SystemState::DOCKING_M2);
}

void DockingSystem::commandJog(int motorNum) {
    if (isBusy()) {
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

    motor1.stop();
    motor2.stop();
    motorProp.stop();
    testMotor = nullptr;
    awaitingRetry = false;

    // Re-derive the real resting state from the limit switches — never
    // assume. Deliberately does NOT touch lastReportedState — stateChanged()
    // compares it against currentState to decide whether to notify BLE
    // clients; setting both here would make the comparison see no change,
    // leaving the status characteristic frozen on its pre-RESET value.
    currentState = inferRestingState();

    Serial.print("[RESET] System recovered. State: ");
    Serial.println(getStateString());
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
    switch (currentState) {
        case SystemState::UNKNOWN:            return "UNKNOWN";
        case SystemState::DOCKED:             return "DOCKED";
        case SystemState::UNDOCKED:           return "UNDOCKED";
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
    if (currentState != lastReportedState) {
        lastReportedState = currentState;
        return true;
    }
    return false;
}

bool DockingSystem::isBusy() const {
    switch (currentState) {
        case SystemState::UNDOCKING_M1:
        case SystemState::UNDOCKING_M2:
        case SystemState::DOCKING_M2:
        case SystemState::DOCKING_M1:
        case SystemState::DOCKING_PROP_OPEN:
        case SystemState::DOCKING_PROP_CLOSE:
        case SystemState::TESTING:
            return true;
        default:
            return false;
    }
}

SystemState DockingSystem::inferRestingState() {
    bool docked = motor1.isDocked() && motor2.isDocked() && motorProp.isDocked();
    bool undocked = motor1.isUndocked() && motor2.isUndocked();

    if (docked && !undocked) return SystemState::DOCKED;
    if (undocked && !docked) return SystemState::UNDOCKED;
    return SystemState::UNKNOWN;
}

void DockingSystem::enterStage(SystemState state) {
    currentState = state;
    beginStage();
}

void DockingSystem::beginStage() {
    stageStartMs = millis();
    stageRetries = 0;
    awaitingRetry = false;
}

bool DockingSystem::runStage(MotorController& motor, bool atTarget, const char* stageName,
                              const std::function<void()>& restart) {
    if (atTarget) {
        return true;
    }

    if (awaitingRetry) {
        if (millis() >= retryReadyAtMs) {
            awaitingRetry = false;
            stageStartMs = millis();
            Serial.print("Retrying stage: ");
            Serial.println(stageName);
            restart();
        }
        return false;
    }

    if (millis() - stageStartMs > STAGE_TIMEOUT_MS) {
        motor.stop();
        stageRetries++;
        if (stageRetries > MAX_STAGE_RETRIES) {
            Serial.print("ERROR: ");
            Serial.print(stageName);
            Serial.print(" timed out after ");
            Serial.print(stageRetries);
            Serial.println(" attempt(s) — giving up");
            currentState = SystemState::ERROR;
        } else {
            Serial.print("WARNING: ");
            Serial.print(stageName);
            Serial.print(" timed out — will retry (attempt ");
            Serial.print(stageRetries);
            Serial.print("/");
            Serial.print(MAX_STAGE_RETRIES);
            Serial.println(")");
            awaitingRetry = true;
            retryReadyAtMs = millis() + STAGE_RETRY_DELAY_MS;
        }
    }

    return false;
}

void DockingSystem::update() {
    motor1.update();
    motor2.update();
    motorProp.update();

    switch (currentState) {
        case SystemState::UNKNOWN:
        case SystemState::DOCKED:
        case SystemState::UNDOCKED:
        case SystemState::ERROR:
            // Passively re-verify against the switches — catches a reading
            // drifting after the state was last computed, and lets ERROR
            // clear itself if the physical evidence now genuinely supports
            // a definitive DOCKED/UNDOCKED. Never retries a motor here —
            // only acts on what the switches show right now.
            if (millis() - lastRestCheckMs >= REST_RECHECK_INTERVAL_MS) {
                lastRestCheckMs = millis();
                SystemState fresh = inferRestingState();
                bool errorClearable = (currentState == SystemState::ERROR &&
                                       fresh != SystemState::UNKNOWN);
                if (fresh != currentState &&
                    (currentState != SystemState::ERROR || errorClearable)) {
                    Serial.print("Resting state re-verified: ");
                    Serial.print(getStateString());
                    currentState = fresh;
                    Serial.print(" -> ");
                    Serial.println(getStateString());
                }
            }
            break;

        // ── UNDOCK: M1 → M2 ──────────────────────────────────────────────
        // Propeller closer is not touched during undock — it stays closed.
        case SystemState::UNDOCKING_M1:
            if (runStage(motor1, motor1.isUndocked(), "M1 undock",
                         [this]() { motor1.startUndocking(); })) {
                Serial.println("M1 Undocked. Starting M2...");
                motor2.startUndocking();
                enterStage(SystemState::UNDOCKING_M2);
            }
            break;

        case SystemState::UNDOCKING_M2:
            if (runStage(motor2, motor2.isUndocked(), "M2 undock",
                         [this]() { motor2.startUndocking(); })) {
                Serial.println("UNDOCKING_COMPLETE -- resting UNDOCKED");
                currentState = SystemState::UNDOCKED;
            }
            break;

        // ── DOCK: M2 → M1 → PROP_OPEN → PROP_CLOSE ───────────────────────
        case SystemState::DOCKING_M2:
            if (runStage(motor2, motor2.isDocked(), "M2 dock",
                         [this]() { motor2.startDocking(); })) {
                Serial.println("M2 Docked. Starting M1...");
                motor1.startDocking();
                enterStage(SystemState::DOCKING_M1);
            }
            break;

        case SystemState::DOCKING_M1:
            if (runStage(motor1, motor1.isDocked(), "M1 dock",
                         [this]() { motor1.startDocking(); })) {
                Serial.println("M1 Docked. Opening propeller closer...");
                motorProp.startUndocking(); // rotates toward the open limit
                enterStage(SystemState::DOCKING_PROP_OPEN);
            }
            break;

        case SystemState::DOCKING_PROP_OPEN:
            if (runStage(motorProp, motorProp.isUndocked(), "propeller open",
                         [this]() { motorProp.startUndocking(); })) {
                Serial.println("Propeller closer open. Closing again...");
                motorProp.startDocking(); // rotates back toward the close limit
                enterStage(SystemState::DOCKING_PROP_CLOSE);
            }
            break;

        case SystemState::DOCKING_PROP_CLOSE:
            if (runStage(motorProp, motorProp.isDocked(), "propeller close",
                         [this]() { motorProp.startDocking(); })) {
                Serial.println("DOCKING_COMPLETE -- resting DOCKED");
                currentState = SystemState::DOCKED;
            }
            break;

        // ── TESTING: bench jog for a fixed JOG_DURATION_MS, ignoring limits ──
        case SystemState::TESTING:
            if (millis() - testStartTime >= JOG_DURATION_MS) {
                testMotor->stop();
                testMotor = nullptr;
                Serial.println("[JOG] Done.");
                // Jog deliberately ignores limits and can leave things in an
                // arbitrary position — re-verify from the switches rather
                // than assuming a resting state.
                currentState = inferRestingState();
            }
            break;
    }
}
