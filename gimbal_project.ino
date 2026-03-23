#include "GimbalController.h"

static constexpr uint8_t PAN_ID  = 1;
static constexpr uint8_t TILT_ID = 2;
static constexpr int DXL_DIR_PIN = -1;

GimbalController gimbal(PAN_ID, TILT_ID, DXL_DIR_PIN, Serial1);

void setup() {
    gimbal.init();
}

void loop() {
    gimbal.update();
}