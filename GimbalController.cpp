#include "GimbalController.h"

#define BAUD 57600
#define USB_BAUD 115200

static constexpr float TICKS_PER_DEG = 4096.0f / 360.0f;

// Keep these conservative until direction/range are confirmed.
static constexpr float MAX_SAFE_PAN_DEG = 270.0f;
static constexpr float MAX_SAFE_TILT_DEG = 85.0f;

static float clampFloat(float x, float lo, float hi)
{
  if(x < lo)
    return lo;
  if(x > hi)
    return hi;
  return x;
}

GimbalController::GimbalController(uint8_t panId, uint8_t tiltId, int dirPin,
                                   HardwareSerial &dxlSerial)
  : m_panId(panId), m_tiltId(tiltId), m_dirPin(dirPin), m_serial(dxlSerial),
    m_dxl(nullptr), m_zeroPan(0), m_zeroTilt(0), m_panCal(false),
    m_tiltCal(false), m_panTorque(false), m_tiltTorque(false), m_buffer(""),
    m_state(GimbalState::STARTUP), m_stateStartMs(0), m_testStep(0),
    m_testStepStartMs(0)
{
}

void GimbalController::init()
{
  Serial.begin(USB_BAUD);
  while(!Serial)
  {
  }

  Serial.println("FIRMWARE VERSION: SAFE_RELATIVE_V3_STATE_MACHINE");

  m_serial.begin(BAUD);
  m_dxl = new Dynamixel2Arduino(m_serial, m_dirPin);

  m_dxl->begin(BAUD);
  m_dxl->setPortProtocolVersion(2.0);

  Serial.println(
    "READY\n"
    "Commands:\n"
    "  ping\n"
    "  e <id>       enable torque\n"
    "  d <id>       disable torque\n"
    "  c <id>       capture current position as zero\n"
    "  p <id> <deg> move relative to zero\n"
    "  pb <p> <t>   move both relative to zero\n"
    "  z            move both to zero\n"
    "  s <id>       status one\n"
    "  sa           status all\n"
    "  test         motion test");

  setState(GimbalState::WAITING_FOR_CALIBRATION);
}

void GimbalController::update()
{
  handleSerial();
  updateStateMachine();
}

void GimbalController::setState(GimbalState newState)
{
  if(m_state == newState)
  {
    return;
  }

  m_state = newState;
  // millis: returns arduino milliseconds
  m_stateStartMs = millis();

  Serial.print("STATE -> ");
  Serial.println(stateName());
}

const char *GimbalController::stateName() const
{
  switch(m_state)
  {
  case GimbalState::STARTUP:
    return "STARTUP";
  case GimbalState::WAITING_FOR_CALIBRATION:
    return "WAITING_FOR_CALIBRATION";
  case GimbalState::READY:
    return "READY";
  case GimbalState::RUNNING_TEST:
    return "RUNNING_TEST";
  case GimbalState::ERROR_STATE:
    return "ERROR_STATE";
  default:
    return "UNKNOWN";
  }
}

bool GimbalController::isFullyCalibrated() const
{
  return m_panCal && m_tiltCal;
}

void GimbalController::updateStateMachine()
{
  switch(m_state)
  {
  case GimbalState::STARTUP:
    setState(GimbalState::WAITING_FOR_CALIBRATION);
    break;

  case GimbalState::WAITING_FOR_CALIBRATION:
    if(isFullyCalibrated())
    {
      setState(GimbalState::READY);
    }
    break;

  case GimbalState::READY:
    if(!isFullyCalibrated())
    {
      setState(GimbalState::WAITING_FOR_CALIBRATION);
    }
    break;

  case GimbalState::RUNNING_TEST:
    updateTestSequence();
    break;

  case GimbalState::ERROR_STATE:
    break;
  }
}

void GimbalController::handleSerial()
{
  while(Serial.available())
  {
    char c = Serial.read();

    if(c == '\n')
    {
      processCommand(m_buffer);
      m_buffer = "";
    }
    else if(c != '\r')
    {
      m_buffer += c;
    }
  }
}

void GimbalController::processCommand(String line)
{
  line.trim();

  if(line.length() == 0)
  {
    return;
  }

  if(line == "ping")
  {
    Serial.println("PONG");
    return;
  }

  if(line == "test")
  {
    if(m_state != GimbalState::READY)
    {
      Serial.print("ERR test only allowed in READY state; current state=");
      Serial.println(stateName());
      return;
    }

    startTest();
    return;
  }

  if(line == "sa")
  {
    statusAll();
    return;
  }

  int firstSpace = line.indexOf(' ');
  String cmd = (firstSpace == -1) ? line : line.substring(0, firstSpace);

  if(cmd == "e" || cmd == "d" || cmd == "c" || cmd == "s")
  {
    if(firstSpace == -1)
    {
      Serial.print("ERR missing argument: ");
      Serial.println(line);
      return;
    }

    int id = line.substring(firstSpace + 1).toInt();

    if(cmd == "e")
    {
      enableTorque((uint8_t)id);
      return;
    }
    if(cmd == "d")
    {
      disableTorque((uint8_t)id);
      return;
    }
    if(cmd == "c")
    {
      calibrate((uint8_t)id);
      return;
    }
    if(cmd == "s")
    {
      int32_t pos = m_dxl->getPresentPosition((uint8_t)id);

      Serial.print("STATUS ID ");
      Serial.print(id);
      Serial.print(": raw=");
      Serial.print(pos);

      if((uint8_t)id == m_panId)
      {
        Serial.print(" zero=");
        Serial.print(m_zeroPan);
        Serial.print(" calibrated=");
        Serial.print(m_panCal ? "YES" : "NO");
        Serial.print(" torque=");
        Serial.print(m_panTorque ? "ON" : "OFF");
        Serial.print(" state=");
        Serial.println(stateName());
      }
      else if((uint8_t)id == m_tiltId)
      {
        Serial.print(" zero=");
        Serial.print(m_zeroTilt);
        Serial.print(" calibrated=");
        Serial.print(m_tiltCal ? "YES" : "NO");
        Serial.print(" torque=");
        Serial.print(m_tiltTorque ? "ON" : "OFF");
        Serial.print(" state=");
        Serial.println(stateName());
      }
      else
      {
        Serial.println(" ERR invalid ID");
      }
      return;
    }
  }

  if(cmd == "p")
  {
    if(firstSpace == -1)
    {
      Serial.print("ERR missing arguments: ");
      Serial.println(line);
      return;
    }

    int secondSpace = line.indexOf(' ', firstSpace + 1);
    if(secondSpace == -1)
    {
      Serial.print("ERR missing degree value: ");
      Serial.println(line);
      return;
    }

    int id = line.substring(firstSpace + 1, secondSpace).toInt();
    float deg = line.substring(secondSpace + 1).toFloat();

    if(m_state != GimbalState::READY)
    {
      Serial.print("ERR move only allowed in READY state; current state=");
      Serial.println(stateName());
      return;
    }

    move((uint8_t)id, deg);
    return;
  }

  if(cmd == "pb")
  {
    if(firstSpace == -1)
    {
      Serial.print("ERR missing arguments: ");
      Serial.println(line);
      return;
    }

    int secondSpace = line.indexOf(' ', firstSpace + 1);
    if(secondSpace == -1)
    {
      Serial.print("ERR missing tilt degree value: ");
      Serial.println(line);
      return;
    }

    float pan = line.substring(firstSpace + 1, secondSpace).toFloat();
    float tilt = line.substring(secondSpace + 1).toFloat();

    if(m_state != GimbalState::READY)
    {
      Serial.print("ERR move only allowed in READY state; current state=");
      Serial.println(stateName());
      return;
    }

    moveBoth(pan, tilt);
    return;
  }

  if(line == "z")
  {
    if(m_state != GimbalState::READY)
    {
      Serial.print("ERR zero move only allowed in READY state; current state=");
      Serial.println(stateName());
      return;
    }

    moveBoth(0.0f, 0.0f);
    return;
  }

  Serial.print("ERR unknown command: ");
  Serial.println(line);
}

void GimbalController::enableTorque(uint8_t id)
{
  m_dxl->torqueOn(id);

  if(id == m_panId)
  {
    m_panTorque = true;
  }
  else if(id == m_tiltId)
  {
    m_tiltTorque = true;
  }
  else
  {
    Serial.print("ERR invalid ID ");
    Serial.println(id);
    return;
  }

  Serial.print("OK torque enabled on ID ");
  Serial.println(id);
}

void GimbalController::disableTorque(uint8_t id)
{
  m_dxl->torqueOff(id);

  if(id == m_panId)
  {
    m_panTorque = false;
  }
  else if(id == m_tiltId)
  {
    m_tiltTorque = false;
  }
  else
  {
    Serial.print("ERR invalid ID ");
    Serial.println(id);
    return;
  }

  Serial.print("OK torque disabled on ID ");
  Serial.println(id);
}

void GimbalController::calibrate(uint8_t id)
{
  int32_t pos = m_dxl->getPresentPosition(id);

  if(id == m_panId)
  {
    m_zeroPan = pos;
    m_panCal = true;
  }
  else if(id == m_tiltId)
  {
    m_zeroTilt = pos;
    m_tiltCal = true;
  }
  else
  {
    Serial.print("ERR invalid ID ");
    Serial.println(id);
    return;
  }

  Serial.print("ZERO SET FOR ID ");
  Serial.print(id);
  Serial.print(": raw=");
  Serial.println(pos);

  if(isFullyCalibrated() && m_state == GimbalState::WAITING_FOR_CALIBRATION)
  {
    setState(GimbalState::READY);
  }
}

void GimbalController::move(uint8_t id, float deg)
{
  bool isPan = (id == m_panId);
  bool isTilt = (id == m_tiltId);

  if(!isPan && !isTilt)
  {
    Serial.print("ERR invalid ID ");
    Serial.println(id);
    return;
  }

  bool calibrated = isPan ? m_panCal : m_tiltCal;
  bool torqueOn = isPan ? m_panTorque : m_tiltTorque;
  int32_t zero = isPan ? m_zeroPan : m_zeroTilt;

  if(!calibrated)
  {
    Serial.print("ERR motor ");
    Serial.print(id);
    Serial.println(" not calibrated; run c <id> first");
    return;
  }

  if(!torqueOn)
  {
    Serial.print("ERR motor ");
    Serial.print(id);
    Serial.println(" torque is OFF; run e <id> first");
    return;
  }

  float limit = isPan ? MAX_SAFE_PAN_DEG : MAX_SAFE_TILT_DEG;
  float clampedDeg = clampFloat(deg, -limit, limit);

  if(clampedDeg != deg)
  {
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

void GimbalController::moveBoth(float pan, float tilt)
{
  move(m_panId, pan);
  move(m_tiltId, tilt);
}

void GimbalController::statusAll()
{
  int32_t panPos = m_dxl->getPresentPosition(m_panId);
  int32_t tiltPos = m_dxl->getPresentPosition(m_tiltId);

  Serial.print("STATE=");
  Serial.println(stateName());

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

void GimbalController::startTest()
{
  Serial.println("SAFE TEST START");

  m_testStep = 0;
  m_testStepStartMs = millis();
  setState(GimbalState::RUNNING_TEST);
}

void GimbalController::updateTestSequence()
{
  unsigned long now = millis();

  switch(m_testStep)
  {
  case 0:
    enableTorque(m_panId);
    enableTorque(m_tiltId);
    calibrate(m_panId);
    calibrate(m_tiltId);
    m_testStep = 1;
    m_testStepStartMs = now;
    break;

  case 1:
    if(now - m_testStepStartMs >= 500)
    {
      move(m_panId, 20.0f);
      m_testStep = 2;
      m_testStepStartMs = now;
    }
    break;

  case 2:
    if(now - m_testStepStartMs >= 700)
    {
      move(m_panId, 0.0f);
      m_testStep = 3;
      m_testStepStartMs = now;
    }
    break;

  case 3:
    if(now - m_testStepStartMs >= 700)
    {
      move(m_tiltId, 45.0f);
      m_testStep = 4;
      m_testStepStartMs = now;
    }
    break;

  case 4:
    if(now - m_testStepStartMs >= 700)
    {
      move(m_tiltId, 0.0f);
      m_testStep = 5;
      m_testStepStartMs = now;
    }
    break;

  case 5:
    if(now - m_testStepStartMs >= 700)
    {
      Serial.println("SAFE TEST DONE");
      setState(GimbalState::READY);
    }
    break;

  default:
    Serial.println("ERR invalid test state");
    setState(GimbalState::ERROR_STATE);
    break;
  }
}

int32_t GimbalController::degToTicks(float deg)
{
  return (int32_t)(deg * TICKS_PER_DEG);
}
