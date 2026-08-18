#pragma once
#include <functional>
#include "MotorController.h"

enum class SystemState {
    UNKNOWN,              // Position not yet verified against limit switches
    DOCKED,                // Stable resting state — drone secured
    UNDOCKED,               // Stable resting state — drone released, safe to fly
    UNDOCKING_M1,
    UNDOCKING_M2,
    DOCKING_M2,
    DOCKING_M1,
    DOCKING_PROP_OPEN,
    DOCKING_PROP_CLOSE,
    TESTING,
    ERROR
};

class DockingSystem {
public:
    DockingSystem();
    void init();
    void update();

    void commandUndock();
    void commandDock();
    void commandReset();    // stop everything now; re-verify position from limit switches
    void commandStatus();   // print full system status
    void commandJog(int motorNum); // bench test: spin motor 1/2/3 for JOG_DURATION_MS

    SystemState getState() const;
    const char* getStateString() const;
    bool stateChanged();  // returns true once per state transition

private:
    MotorController motor1;
    MotorController motor2;
    MotorController motorProp; // propeller closer — rests closed; opens/closes during dock stage only

    SystemState currentState;
    SystemState lastReportedState;

    // Per-stage watchdog: when the current motor stage started, and how many
    // times it's been retried after timing out. Reset on every stage entry.
    unsigned long stageStartMs;
    uint8_t stageRetries;

    // Non-blocking retry pause — set after a stage times out, cleared once
    // the retry actually restarts the motor. While set, the stage is
    // "waiting to retry", not actively moving.
    bool awaitingRetry;
    unsigned long retryReadyAtMs;

    MotorController* testMotor;
    unsigned long testStartTime;

    // Last time the resting state was passively re-verified against the
    // limit switches — see REST_RECHECK_INTERVAL_MS.
    unsigned long lastRestCheckMs;

    bool isBusy() const;
    // Re-derives DOCKED/UNDOCKED/UNKNOWN directly from the limit switches —
    // never assumed. Used at boot and on RESET. Not const: MotorController's
    // isUndocked()/isDocked() aren't const-qualified.
    SystemState inferRestingState();

    // Shared per-stage timeout/retry handling. `atTarget` is the stage's own
    // limit-switch check; `stageName` is for logging; `restart` re-issues the
    // same motor command (e.g. motor1.startUndocking) to retry. Returns true
    // if the stage succeeded this tick (caller advances to the next state).
    bool runStage(MotorController& motor, bool atTarget, const char* stageName,
                  const std::function<void()>& restart);

    void enterStage(SystemState state);
    void beginStage(); // resets stageStartMs/stageRetries/awaitingRetry for the state just entered
};
