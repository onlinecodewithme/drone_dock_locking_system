#pragma once
#include <Arduino.h>
#include "DockingSystem.h"

class SerialCommand {
public:
    SerialCommand(DockingSystem& dockingSystem);
    void update();

private:
    DockingSystem& system;
};
