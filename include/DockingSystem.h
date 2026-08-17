#pragma once
#include "MotorController.h"

enum class SystemState {
    IDLE,
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
    void commandReset();    // recover from ERROR or cancel operation
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
    unsigned long stateStartTime;

    // Transient completion flags — set for one cycle so BLE can notify
    bool undockingJustCompleted;
    bool dockingJustCompleted;

    MotorController* testMotor;
    unsigned long testStartTime;
};
