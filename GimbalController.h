#pragma once

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

enum class GimbalState
{
  STARTUP,
  WAITING_FOR_CALIBRATION,
  READY,
  RUNNING_TEST,
  ERROR_STATE
};

class GimbalController
{
public:
  GimbalController(uint8_t panId, uint8_t tiltId, int dirPin,
                   HardwareSerial &dxlSerial);

  void init();
  void update();

private:
  uint8_t m_panId;
  uint8_t m_tiltId;

  int m_dirPin;
  HardwareSerial &m_serial;
  Dynamixel2Arduino *m_dxl;

  int32_t m_zeroPan;
  int32_t m_zeroTilt;

  bool m_panCal;
  bool m_tiltCal;

  bool m_panTorque;
  bool m_tiltTorque;

  String m_buffer;

  GimbalState m_state;
  unsigned long m_stateStartMs;

  int m_testStep;
  unsigned long m_testStepStartMs;

  void handleSerial();
  void processCommand(String line);

  void setState(GimbalState newState);
  const char *stateName() const;
  bool isFullyCalibrated() const;
  void updateStateMachine();
  void updateTestSequence();

  void enableTorque(uint8_t id);
  void disableTorque(uint8_t id);
  void calibrate(uint8_t id);
  void move(uint8_t id, float deg);
  void moveBoth(float pan, float tilt);
  void statusAll();
  void startTest();

  int32_t degToTicks(float deg);
};
