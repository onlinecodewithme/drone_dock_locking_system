#include "SerialCommand.h"

SerialCommand::SerialCommand(DockingSystem& dockingSystem) : system(dockingSystem) {}

void SerialCommand::update() {
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();

        if (cmd == "UNDOCK") {
            system.commandUndock();
        } else if (cmd == "DOCK") {
            system.commandDock();
        } else if (cmd == "RESET") {
            system.commandReset();
        } else if (cmd == "STATUS") {
            system.commandStatus();
        } else if (cmd.length() > 0) {
            Serial.println("Unknown command. Valid: DOCK, UNDOCK, RESET, STATUS");
        }
    }
}
