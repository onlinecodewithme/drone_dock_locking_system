#include <Arduino.h>
#include "DockingSystem.h"
#include "SerialCommand.h"
#include "BLEComm.h"

DockingSystem dockingSystem;
SerialCommand serialCommand(dockingSystem);
BLEComm bleComm(dockingSystem);

void setup() {
    Serial.begin(115200);
    delay(3000); // 3-second delay to let power stabilize
    Serial.println("\n=== DOCKING SYSTEM STARTING ===");
    
    Serial.println("Initializing motors...");
    dockingSystem.init();
    Serial.println("Motors OK.");
    
    Serial.println("Initializing BLE...");
    bleComm.init();
    Serial.println("BLE OK.");

    Serial.println("System Ready! Waiting for DOCK or UNDOCK (Serial or BLE).");
    Serial.flush(); // Ensure all output is sent before entering loop
}

void loop() {
    serialCommand.update();
    dockingSystem.update();
    bleComm.update();
}
