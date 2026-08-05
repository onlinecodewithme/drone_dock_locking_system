#pragma once

#include <Arduino.h>

// --- Pins Setup ---

// Motor 1 (e.g., Left side)
constexpr uint8_t M1_STEP_PIN = 25;
constexpr uint8_t M1_DIR_PIN = 26;
constexpr uint8_t M1_ENABLE_PIN = 27;

// Motor 2 (e.g., Right side)
constexpr uint8_t M2_STEP_PIN = 14;
constexpr uint8_t M2_DIR_PIN = 12;
constexpr uint8_t M2_ENABLE_PIN = 13;

// Limit Switches (Active LOW assumed)
constexpr uint8_t M1_UNDOCK_LIMIT_PIN = 32;
constexpr uint8_t M1_DOCK_LIMIT_PIN = 33;
constexpr uint8_t M2_UNDOCK_LIMIT_PIN = 18; // Changed from 34 (input-only, no pull-up)
constexpr uint8_t M2_DOCK_LIMIT_PIN = 19;   // Changed from 35 (input-only, no pull-up)

// Servo
constexpr uint8_t SERVO_PIN = 2; // Suitable for PWM

// --- Constants ---
constexpr float MOTOR_MAX_SPEED = 1000.0f; // steps per second
constexpr float MOTOR_ACCELERATION = 500.0f;

// For continuous rotation during docking/undocking, we can just set a very large target position
constexpr long MOTOR_CONTINUOUS_STEPS = 1000000;

// Servo Angles
constexpr int SERVO_DOCK_ANGLE = 0;
constexpr int SERVO_UNDOCK_ANGLE = 270;

// Servo speed control — degrees per second
// Lower = slower. 270° travel at 45°/s takes 6 seconds.
// At full speed (~300°/s) it takes ~1s. Adjust to taste.
constexpr float SERVO_SPEED_DEG_PER_SEC = 45.0f;  // degrees per second

// Step interval in ms derived from speed (do not edit)
constexpr unsigned long SERVO_STEP_INTERVAL_MS = 20; // update every 20ms

// --- BLE Configuration ---
#define BLE_DEVICE_NAME        "DockController"
#define BLE_SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CMD_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_STATUS_CHAR_UUID   "8c1c10ea-4536-4a5b-9c37-2f7a3e5c1d2b"
