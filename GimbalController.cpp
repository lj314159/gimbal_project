#include "GimbalController.h"

#define BAUD 57600
#define USB_BAUD 115200

static constexpr float TICKS_PER_DEG = 4096.0f / 360.0f;

// Keep these conservative until direction/range are confirmed.
static constexpr float MAX_SAFE_PAN_DEG  = 180.0f;
static constexpr float MAX_SAFE_TILT_DEG = 80.0f;

GimbalController::GimbalController(uint8_t panId,
                                   uint8_t tiltId,
                                   int dirPin,
                                   HardwareSerial& dxlSerial)
    : m_panId(panId),
      m_tiltId(tiltId),
      m_dirPin(dirPin),
      m_serial(dxlSerial),
      m_dxl(nullptr),
      m_zeroPan(0),
      m_zeroTilt(0),
      m_panCal(false),
      m_tiltCal(false),
      m_panTorque(false),
      m_tiltTorque(false),
      m_buffer("") {}

void GimbalController::init() {
    Serial.begin(USB_BAUD);
    while (!Serial) {}

    Serial.println("FIRMWARE VERSION: SAFE_RELATIVE_V2");

    m_serial.begin(BAUD);
    m_dxl = new Dynamixel2Arduino(m_serial, m_dirPin);

    m_dxl->begin(BAUD);
    m_dxl->setPortProtocolVersion(2.0);

    Serial.println("READY");
    Serial.println("Commands:");
    Serial.println("  ping");
    Serial.println("  e <id>       enable torque");
    Serial.println("  d <id>       disable torque");
    Serial.println("  c <id>       capture current position as zero");
    Serial.println("  p <id> <deg> move relative to zero");
    Serial.println("  pb <p> <t>   move both relative to zero");
    Serial.println("  z            move both to zero");
    Serial.println("  s <id>       status one");
    Serial.println("  sa           status all");
    Serial.println("  test         safe motion test");
}

void GimbalController::update() {
    handleSerial();
}

void GimbalController::handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n') {
            processCommand(m_buffer);
            m_buffer = "";
        } else if (c != '\r') {
            m_buffer += c;
        }
    }
}

static float clampFloat(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void GimbalController::processCommand(String line) {
    line.trim();

    if (line.length() == 0) {
        return;
    }

    if (line == "ping") {
        Serial.println("PONG");
        return;
    }

    if (line == "test") {
        runTest();
        return;
    }

    if (line == "sa") {
        statusAll();
        return;
    }

    int firstSpace = line.indexOf(' ');
    String cmd = (firstSpace == -1) ? line : line.substring(0, firstSpace);

    if (cmd == "e" || cmd == "d" || cmd == "c" || cmd == "s") {
        if (firstSpace == -1) {
            Serial.print("ERR missing argument: ");
            Serial.println(line);
            return;
        }

        int id = line.substring(firstSpace + 1).toInt();

        if (cmd == "e") {
            enableTorque((uint8_t)id);
            return;
        }
        if (cmd == "d") {
            disableTorque((uint8_t)id);
            return;
        }
        if (cmd == "c") {
            calibrate((uint8_t)id);
            return;
        }
        if (cmd == "s") {
            int32_t pos = m_dxl->getPresentPosition((uint8_t)id);

            Serial.print("STATUS ID ");
            Serial.print(id);
            Serial.print(": raw=");
            Serial.print(pos);

            if ((uint8_t)id == m_panId) {
                Serial.print(" zero=");
                Serial.print(m_zeroPan);
                Serial.print(" calibrated=");
                Serial.print(m_panCal ? "YES" : "NO");
                Serial.print(" torque=");
                Serial.println(m_panTorque ? "ON" : "OFF");
            } else if ((uint8_t)id == m_tiltId) {
                Serial.print(" zero=");
                Serial.print(m_zeroTilt);
                Serial.print(" calibrated=");
                Serial.print(m_tiltCal ? "YES" : "NO");
                Serial.print(" torque=");
                Serial.println(m_tiltTorque ? "ON" : "OFF");
            } else {
                Serial.println(" ERR invalid ID");
            }
            return;
        }
    }

    if (cmd == "p") {
        if (firstSpace == -1) {
            Serial.print("ERR missing arguments: ");
            Serial.println(line);
            return;
        }

        int secondSpace = line.indexOf(' ', firstSpace + 1);
        if (secondSpace == -1) {
            Serial.print("ERR missing degree value: ");
            Serial.println(line);
            return;
        }

        int id = line.substring(firstSpace + 1, secondSpace).toInt();
        float deg = line.substring(secondSpace + 1).toFloat();

        move((uint8_t)id, deg);
        return;
    }

    if (cmd == "pb") {
        if (firstSpace == -1) {
            Serial.print("ERR missing arguments: ");
            Serial.println(line);
            return;
        }

        int secondSpace = line.indexOf(' ', firstSpace + 1);
        if (secondSpace == -1) {
            Serial.print("ERR missing tilt degree value: ");
            Serial.println(line);
            return;
        }

        float pan = line.substring(firstSpace + 1, secondSpace).toFloat();
        float tilt = line.substring(secondSpace + 1).toFloat();

        moveBoth(pan, tilt);
        return;
    }

    if (line == "z") {
        moveBoth(0.0f, 0.0f);
        return;
    }

    Serial.print("ERR unknown command: ");
    Serial.println(line);
}

void GimbalController::enableTorque(uint8_t id) {
    m_dxl->torqueOn(id);

    if (id == m_panId) {
        m_panTorque = true;
    } else if (id == m_tiltId) {
        m_tiltTorque = true;
    }

    Serial.print("OK torque enabled on ID ");
    Serial.println(id);
}

void GimbalController::disableTorque(uint8_t id) {
    m_dxl->torqueOff(id);

    if (id == m_panId) {
        m_panTorque = false;
    } else if (id == m_tiltId) {
        m_tiltTorque = false;
    }

    Serial.print("OK torque disabled on ID ");
    Serial.println(id);
}

void GimbalController::calibrate(uint8_t id) {
    int32_t pos = m_dxl->getPresentPosition(id);

    if (id == m_panId) {
        m_zeroPan = pos;
        m_panCal = true;
    } else if (id == m_tiltId) {
        m_zeroTilt = pos;
        m_tiltCal = true;
    } else {
        Serial.print("ERR invalid ID ");
        Serial.println(id);
        return;
    }

    Serial.print("ZERO SET FOR ID ");
    Serial.print(id);
    Serial.print(": raw=");
    Serial.println(pos);
}

void GimbalController::move(uint8_t id, float deg) {
    bool isPan = (id == m_panId);
    bool isTilt = (id == m_tiltId);

    if (!isPan && !isTilt) {
        Serial.print("ERR invalid ID ");
        Serial.println(id);
        return;
    }

    bool calibrated = isPan ? m_panCal : m_tiltCal;
    bool torqueOn = isPan ? m_panTorque : m_tiltTorque;
    int32_t zero = isPan ? m_zeroPan : m_zeroTilt;

    if (!calibrated) {
        Serial.print("ERR motor ");
        Serial.print(id);
        Serial.println(" not calibrated; run c <id> first");
        return;
    }

    if (!torqueOn) {
        Serial.print("ERR motor ");
        Serial.print(id);
        Serial.println(" torque is OFF; run e <id> first");
        return;
    }

    float limit = isPan ? MAX_SAFE_PAN_DEG : MAX_SAFE_TILT_DEG;
    float clampedDeg = clampFloat(deg, -limit, limit);

    if (clampedDeg != deg) {
        Serial.print("WARN clamped ID ");
        Serial.print(id);
        Serial.print(" from ");
        Serial.print(deg, 2);
        Serial.print(" deg to ");
        Serial.print(clampedDeg, 2);
        Serial.println(" deg");
    }

    int32_t goal = zero + degToTicks(clampedDeg);

    m_dxl->setGoalPosition(id, (float)goal);

    Serial.print("MOVE ID ");
    Serial.print(id);
    Serial.print(": rel_deg=");
    Serial.print(clampedDeg, 2);
    Serial.print(" zero_raw=");
    Serial.print(zero);
    Serial.print(" goal_raw=");
    Serial.println(goal);
}

void GimbalController::moveBoth(float pan, float tilt) {
    move(m_panId, pan);
    move(m_tiltId, tilt);
}

void GimbalController::statusAll() {
    int32_t panPos = m_dxl->getPresentPosition(m_panId);
    int32_t tiltPos = m_dxl->getPresentPosition(m_tiltId);

    Serial.print("PAN: raw=");
    Serial.print(panPos);
    Serial.print(" zero=");
    Serial.print(m_zeroPan);
    Serial.print(" calibrated=");
    Serial.print(m_panCal ? "YES" : "NO");
    Serial.print(" torque=");
    Serial.println(m_panTorque ? "ON" : "OFF");

    Serial.print("TILT: raw=");
    Serial.print(tiltPos);
    Serial.print(" zero=");
    Serial.print(m_zeroTilt);
    Serial.print(" calibrated=");
    Serial.print(m_tiltCal ? "YES" : "NO");
    Serial.print(" torque=");
    Serial.println(m_tiltTorque ? "ON" : "OFF");
}

void GimbalController::runTest() {
    Serial.println("SAFE TEST START");

    // Capture the CURRENT position as zero so test motion is truly relative.
    calibrate(m_panId);
    calibrate(m_tiltId);

    enableTorque(m_panId);
    enableTorque(m_tiltId);

    delay(500);

    // Very small moves only.
    move(m_panId,20.0f);
    delay(700);
    move(m_panId, 0.0f);
    delay(700);

    move(m_tiltId, 45.0f);
    delay(700);
    move(m_tiltId, 0.0f);
    delay(700);

    Serial.println("SAFE TEST DONE");
}

int32_t GimbalController::degToTicks(float deg) {
    return (int32_t)(deg * TICKS_PER_DEG);
}