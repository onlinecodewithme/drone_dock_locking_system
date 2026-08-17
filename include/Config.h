#pragma once

#include <Arduino.h>

// --- Pins Setup ---

// Motor 1 (e.g., Left side)
constexpr uint8_t M1_STEP_PIN = 25;
constexpr uint8_t M1_DIR_PIN = 26;
constexpr uint8_t M1_ENABLE_PIN = 27;

// Motor 2 (e.g., Right side)
constexpr uint8_t M2_STEP_PIN = 14;
constexpr uint8_t M2_DIR_PIN = 23; // Moved from 12 — GPIO12 is a flash-voltage strapping pin;
                                    // if pulled HIGH at boot (e.g. by driver board wiring) it
                                    // makes the ESP32 mis-configure flash voltage and fail to boot.
constexpr uint8_t M2_ENABLE_PIN = 13;

// Limit Switches (Active LOW assumed)
constexpr uint8_t M1_UNDOCK_LIMIT_PIN = 32;
constexpr uint8_t M1_DOCK_LIMIT_PIN = 33;
constexpr uint8_t M2_UNDOCK_LIMIT_PIN = 18; // Changed from 34 (input-only, no pull-up)
constexpr uint8_t M2_DOCK_LIMIT_PIN = 19;   // Changed from 35 (input-only, no pull-up)

// Motor 3 (Propeller Closer)
constexpr uint8_t M3_STEP_PIN = 4;
constexpr uint8_t M3_DIR_PIN = 5;
constexpr uint8_t M3_ENABLE_PIN = 15;

// Propeller Closer Limit Switches (Active LOW assumed)
constexpr uint8_t M3_OPEN_LIMIT_PIN = 21;
constexpr uint8_t M3_CLOSE_LIMIT_PIN = 22;

// --- Constants ---
constexpr float MOTOR_MAX_SPEED = 75.0f; // steps per second
constexpr float MOTOR_ACCELERATION = 500.0f;

// For continuous rotation during docking/undocking, we can just set a very large target position
constexpr long MOTOR_CONTINUOUS_STEPS = 1000000;

// Bench-test jog duration (JOG1/JOG2/JOG3 serial commands)
constexpr unsigned long JOG_DURATION_MS = 5000;

// Limit switch debounce — a reading must hold steady this long before it's trusted.
// Filters brief noise spikes on limit switch lines (e.g. induced by motor start transients).
constexpr unsigned long LIMIT_DEBOUNCE_MS = 30;

// --- BLE Configuration ---
#define BLE_DEVICE_NAME        "DockController"
#define BLE_SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CMD_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_STATUS_CHAR_UUID   "8c1c10ea-4536-4a5b-9c37-2f7a3e5c1d2b"
