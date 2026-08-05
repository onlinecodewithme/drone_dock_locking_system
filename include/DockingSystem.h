#pragma once
#include "MotorController.h"
#include <ESP32Servo.h>

enum class SystemState {
    IDLE,
    UNDOCKING_M1,
    UNDOCKING_M2,
    UNDOCKING_SERVO,
    DOCKING_M2,
    DOCKING_M1,
    DOCKING_SERVO,
    ERROR
};

class DockingSystem {
public:
    DockingSystem();
    void init();
    void update();

    void commandUndock();
    void commandDock();

    SystemState getState() const;
    const char* getStateString() const;
    bool stateChanged();  // returns true once per state transition

private:
    MotorController motor1;
    MotorController motor2;
    Servo servo;

    SystemState currentState;
    SystemState lastReportedState;
    unsigned long stateStartTime;

    // Transient completion flags — set for one cycle so BLE can notify
    bool undockingJustCompleted;
    bool dockingJustCompleted;
};
