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

// Max time a single motor stage (e.g. "M1 undock", "propeller close") is allowed
// to run before it's considered stuck. Previously the only "timeout" was a motor
// exhausting MOTOR_CONTINUOUS_STEPS, which at MOTOR_MAX_SPEED works out to ~3.7
// hours — not a real watchdog. This is a real elapsed-time bound per stage.
// The propeller-closer stage in particular has been observed taking up to ~90s
// on real hardware — keep this comfortably above that, not the individual
// motor's typical time.
constexpr unsigned long STAGE_TIMEOUT_MS = 90000;

// A stage that times out is retried this many times (stop, brief pause, restart
// the same stage) before the system gives up and reports ERROR. Handles
// transient issues (a momentary jam, a switch bounce that delayed confirmation)
// without masking a genuine, persistent fault — retries exhausted still ends in
// a real ERROR requiring RESET or physical inspection.
constexpr uint8_t MAX_STAGE_RETRIES = 2;

// Pause between a timed-out stage and its retry attempt. Handled without
// blocking delay() — see DockingSystem::update()'s retryReadyAtMs.
constexpr unsigned long STAGE_RETRY_DELAY_MS = 1000;

// How often to re-verify the resting state (UNKNOWN/DOCKED/UNDOCKED/ERROR)
// against the limit switches while not actively running a stage. A reported
// state is a snapshot from whenever it was last computed — this catches a
// switch reading drifting afterward (e.g. a marginal connection) and
// self-corrects the reported status instead of silently going stale. Also
// lets ERROR clear itself automatically if the physical evidence genuinely
// resolves back to a definitive DOCKED/UNDOCKED — never a blind motor retry,
// only acting on what the switches actually show right now.
constexpr unsigned long REST_RECHECK_INTERVAL_MS = 2000;

// --- BLE Configuration ---
#define BLE_DEVICE_NAME        "ESP_UGV"
#define BLE_SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CMD_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_STATUS_CHAR_UUID   "8c1c10ea-4536-4a5b-9c37-2f7a3e5c1d2b"
