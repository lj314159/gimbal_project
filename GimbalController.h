#pragma once

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

class GimbalController {
public:
    GimbalController(uint8_t panId,
                     uint8_t tiltId,
                     int dirPin,
                     HardwareSerial& dxlSerial);

    void init();
    void update();

private:
    uint8_t m_panId;
    uint8_t m_tiltId;

    int m_dirPin;
    HardwareSerial& m_serial;
    Dynamixel2Arduino* m_dxl;

    int32_t m_zeroPan;
    int32_t m_zeroTilt;

    bool m_panCal;
    bool m_tiltCal;

    bool m_panTorque;
    bool m_tiltTorque;

    String m_buffer;

    void handleSerial();
    void processCommand(String line);

    void enableTorque(uint8_t id);
    void disableTorque(uint8_t id);
    void calibrate(uint8_t id);
    void move(uint8_t id, float deg);
    void moveBoth(float pan, float tilt);
    void statusAll();
    void runTest();

    int32_t degToTicks(float deg);
};