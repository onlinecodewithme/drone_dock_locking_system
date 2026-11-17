#include <Arduino.h>
#include "DockingSystem.h"
#include "SerialCommand.h"

// USB serial is the only command/status interface. BLE was removed: the
// Raspberry Pi talks to this board over its USB-UART (CP2102) link.
//
// Machine-readable protocol (one line each, '\n' terminated):
//   Pi -> ESP32:  DOCK | UNDOCK | RESET | STATUS | JOG1 | JOG2 | JOG3
//   ESP32 -> Pi:  STATE:<state>                 on every state change, and once at boot
//                 [STATUS] State=<state> ...     in reply to STATUS
// Everything else printed is human-readable log output.

DockingSystem dockingSystem;
SerialCommand serialCommand(dockingSystem);

static void reportState() {
    Serial.print("STATE:");
    Serial.println(dockingSystem.getStateString());
}

void setup() {
    Serial.begin(115200);
    delay(3000); // 3-second delay to let power stabilize
    Serial.println("\n=== DOCKING SYSTEM STARTING ===");

    Serial.println("Initializing motors...");
    dockingSystem.init();
    Serial.println("Motors OK.");

    Serial.println("System Ready! Waiting for DOCK or UNDOCK (Serial).");
    reportState();  // announce the verified resting state once at boot
    Serial.flush(); // Ensure all output is sent before entering loop
}

void loop() {
    serialCommand.update();
    dockingSystem.update();

    if (dockingSystem.stateChanged()) {
        reportState();
    }
}
