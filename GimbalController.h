#pragma once

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

// High-level operating states of the gimbal controller
enum class GimbalState
{
  STARTUP,                 // Initial boot state
  WAITING_FOR_CALIBRATION, // Waiting for user to set zero positions
  READY,                   // Normal operation, accepts movement commands
  RUNNING_TEST,            // Automated motion test sequence
  ERROR_STATE              // Fault state (unexpected condition)
};

class GimbalController
{
public:
  // Constructor: assigns motor IDs, direction pin, and serial interface
  GimbalController(uint8_t panId, uint8_t tiltId, int dirPin,
                   HardwareSerial &dxlSerial);

  void init();   // Initializes serial, Dynamixel bus, and prints help
  void update(); // Main loop: handles serial + state machine

private:
  // ===== Hardware configuration =====
  uint8_t m_panId;  // Dynamixel ID for pan motor
  uint8_t m_tiltId; // Dynamixel ID for tilt motor

  int m_dirPin;             // Direction control pin for half-duplex bus
  HardwareSerial &m_serial; // UART used for Dynamixel communication
  Dynamixel2Arduino *m_dxl; // Dynamixel driver instance

  // ===== Calibration =====
  int32_t m_zeroPan;  // Stored zero position (raw ticks) for pan
  int32_t m_zeroTilt; // Stored zero position (raw ticks) for tilt

  bool m_panCal;  // True if pan has been calibrated
  bool m_tiltCal; // True if tilt has been calibrated

  // ===== Torque state =====
  bool m_panTorque;  // True if pan motor torque is enabled
  bool m_tiltTorque; // True if tilt motor torque is enabled

  // ===== Serial input =====
  String m_buffer; // Accumulates incoming serial command characters

  // ===== State machine =====
  GimbalState m_state;          // Current operating state
  unsigned long m_stateStartMs; // Timestamp when current state began

  // ===== Test sequence =====
  int m_testStep;                  // Current step index in test sequence
  unsigned long m_testStepStartMs; // Timestamp of current test step

  // ===== Core logic =====
  void handleSerial();              // Reads serial input and builds commands
  void processCommand(String line); // Parses and executes a command

  void setState(GimbalState newState); // Transition to new state
  const char *stateName() const;       // Return current statename
  bool isFullyCalibrated() const;      // True if both axes calibrated

  void updateStateMachine(); // Runs state-dependent behavior
  void updateTestSequence(); // Runs non-blocking test steps

  // ===== Motor control =====
  void enableTorque(uint8_t id);  // Enable torque on motor
  void disableTorque(uint8_t id); // Disable torque on motor

  void calibrate(uint8_t id);           // Capture current position as zero
  void move(uint8_t id, float deg);     // Move one motor relative to zero
  void moveBoth(float pan, float tilt); // Move both motors

  void statusAll(); // Print status of both motors
  void startTest(); // Begin automated motion test

  // ===== Utilities =====
  int32_t degToTicks(float deg); // Convert degrees to Dynamixel ticks
};
