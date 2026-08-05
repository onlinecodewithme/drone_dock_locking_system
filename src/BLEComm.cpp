#include "BLEComm.h"
#include "Config.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Forward declarations for callback classes
static DockingSystem* pSystemRef = nullptr;
static BLECharacteristic* pStatusCharacteristic = nullptr;
static BLEServer* pServer = nullptr;
static bool deviceConnected = false;
static bool oldDeviceConnected = false;

// --- BLE Server Callbacks (connection tracking) ---
class DockServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* server) override {
        deviceConnected = true;
        Serial.println("[BLE] Client connected.");
    }

    void onDisconnect(BLEServer* server) override {
        deviceConnected = false;
        Serial.println("[BLE] Client disconnected.");
    }
};

// --- Command Characteristic Callbacks (receive DOCK/UNDOCK) ---
class CommandCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        String value = pCharacteristic->getValue().c_str();
        value.trim();
        value.toUpperCase();

        Serial.print("[BLE] Command received: ");
        Serial.println(value);

        if (value == "UNDOCK") {
            pSystemRef->commandUndock();
        } else if (value == "DOCK") {
            pSystemRef->commandDock();
        } else {
            Serial.println("[BLE] Unknown command. Use DOCK or UNDOCK.");
        }
    }
};

// --- BLEComm Implementation ---

BLEComm::BLEComm(DockingSystem& dockingSystem) : system(dockingSystem) {}

void BLEComm::init() {
    pSystemRef = &system;

    // Initialize BLE
    BLEDevice::init(BLE_DEVICE_NAME);
    
    // Print BLE MAC address for Pi connection
    Serial.print("[BLE] MAC Address: ");
    Serial.println(BLEDevice::getAddress().toString().c_str());
    Serial.print("[BLE] Device name: ");
    Serial.println(BLE_DEVICE_NAME);

    // Create BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new DockServerCallbacks());

    // Create BLE Service
    BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

    // Create Command Characteristic (Write)
    BLECharacteristic* pCmdCharacteristic = pService->createCharacteristic(
        BLE_CMD_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCmdCharacteristic->setCallbacks(new CommandCallbacks());

    // Create Status Characteristic (Read + Notify)
    pStatusCharacteristic = pService->createCharacteristic(
        BLE_STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    // Add the Client Characteristic Configuration Descriptor (required for notifications)
    pStatusCharacteristic->addDescriptor(new BLE2902());

    // Set initial status
    const char* initialStatus = system.getStateString();
    pStatusCharacteristic->setValue(initialStatus);

    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // helps with iPhone connection issues
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("[BLE] Advertising started. Waiting for connections...");
}

void BLEComm::update() {
    // If state changed, update the BLE characteristic and notify
    if (system.stateChanged()) {
        const char* statusStr = system.getStateString();
        pStatusCharacteristic->setValue(statusStr);

        if (deviceConnected) {
            pStatusCharacteristic->notify();
        }

        Serial.print("[BLE] Status updated: ");
        Serial.println(statusStr);
    }

    // Handle reconnection — restart advertising when a client disconnects
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);  // brief pause to let BLE stack settle
        pServer->startAdvertising();
        Serial.println("[BLE] Restarted advertising.");
        oldDeviceConnected = false;
    }

    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = true;
    }
}
