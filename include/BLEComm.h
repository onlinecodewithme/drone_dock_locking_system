#pragma once

#include <Arduino.h>
#include "DockingSystem.h"

class BLEComm {
public:
    BLEComm(DockingSystem& dockingSystem);
    void init();
    void update();  // call in loop — sends BLE notifications on state change

private:
    DockingSystem& system;
};
