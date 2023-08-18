#include "twiControls.h"
#include <Wire.h>

TwoWire twiControlWire(PB3, PB10);

uint8_t ledStateWithParity(const uint8_t);
bool twiControlsCheckButtonState(const uint8_t, const twiControls);

void twiControlsInit(void) {
  twiControlWire.begin();
}

volatile uint8_t activeBrewButton = 0;

volatile bool currentButtonState[4] = { false };
volatile bool lastButtonState[4] = { false };

void twiControlsRead(SensorState& currentState) {
  twiControlWire.requestFrom(0x00, 1, 0x01, 1, true);
  uint8_t buttonState = twiControlWire.read();

  activeBrewButton = 0;
  activeBrewButton |= twiControlsCheckButtonState(buttonState, ONECUP) << 0;
  activeBrewButton |= twiControlsCheckButtonState(buttonState, TWOCUP) << 1;
#ifndef TWI_CONTROLS_EX
  activeBrewButton |= twiControlsCheckButtonState(buttonState, MANUAL) << 2;
  currentState.hotWaterSwitchState = twiControlsCheckButtonState(buttonState, WATER);
#endif
  currentState.brewSwitchState = activeBrewButton > 0;

#ifdef TWI_CONTROLS_EX
  twiControlWire.requestFrom(0x00, 1, 0x02, 1, true);
  uint8_t buttonStateEx = twiControlWire.read() << 4;

  currentState.steamSwitchState = twiControlsCheckButtonState(buttonStateEx, STEAM);
  currentState.powerSwitchState = twiControlsCheckButtonState(buttonStateEx, POWER);
#endif

  uint8_t ledState = 0;
  uint8_t ledStateEx = 0;

  if (currentState.hotWaterSwitchState) {
    currentState.brewSwitchState = false;
    currentState.steamSwitchState = false;
    ledState = WATER;
  }
  if (currentState.steamSwitchState) {
    currentState.brewSwitchState = false;
    currentState.hotWaterSwitchState = false;
    ledState = STEAM;
  }
  if (currentState.brewSwitchState) {
    currentState.steamSwitchState = false;
    currentState.hotWaterSwitchState = false;
    ledState = activeBrewButton;
  }
  if (currentState.waterLvl < 10) {
    ledState |= ELEMENT;
  }

#ifdef TWI_CONTROLS_EX
  if (currentState.powerSwitchState) {
    ledStateEx |= POWER;
  }
#endif

  twiControlWire.beginTransmission(0x00);
  twiControlWire.write((uint8_t)0x00u);
  twiControlWire.write(ledStateWithParity(ledState));
#ifdef TWI_CONTROLS_EX
  twiControlWire.write(ledStateWithParity(ledStateEx >> 4));
#endif
  twiControlWire.endTransmission();
}

void twiControlsRead(SensorState& currentState, Measurements& weightMeasurements) {
  twiControlsRead(currentState);

#ifdef TWI_CONTROLS_EX
  if (!FORCE_PREDICTIVE_SCALES) {
    twiControlWire.requestFrom(0x00, 2, 0x03cau, 2, true);
    uint8_t data[2];
    if (sizeof(data) == twiControlWire.readBytes(data, sizeof(data))) {
      int16_t weight = (data[0] << 8) | data[1];
      weightMeasurements.add(Measurement{
         .value = weight / 10.f,
         .millis = millis(),
        });
      currentState.weight = weight / 10.f;
    }
    if (currentState.tarePending) {
      currentState.tarePending = false;
      weightMeasurements.clear();

      twiControlWire.beginTransmission(0x00);
      twiControlWire.write((uint8_t)0x03u);
      twiControlWire.write((uint8_t)0x0fu);
      twiControlWire.endTransmission();
    }
  }
#endif
}

uint8_t ledStateWithParity(const uint8_t ledState) {
  uint8_t result = ledState & 0x3F;
  if ((ledState & 0x01) ^ ((ledState >> 1) & 0x01) ^ ((ledState >> 2) & 0x01)) {
    result |= 0x40;
  }
  if (((ledState >> 3) & 0x01) ^ ((ledState >> 4) & 0x01) ^ ((ledState >> 5) & 0x01)) {
    result |= 0x80;
  }
  return result;
}

int maskToIdx(const twiControls mask) {
  if (mask == twiControls::ONECUP) { return 0; }
  if (mask == twiControls::TWOCUP) { return 1; }
  #ifndef TWI_CONTROLS_EX
  if (mask == twiControls::MANUAL) { return 2; }
  if (mask == twiControls::WATER) { return 3; }
  #else
  if (mask == twiControls::STEAM) { return 2; }
  if (mask == twiControls::POWER) { return 3; }
  #endif
  return -1;
}

bool twiControlsCheckButtonState(const uint8_t state, const twiControls mask) {
  int idx = maskToIdx(mask);
  if (state & mask) {
    lastButtonState[idx] = true;
  }
  else {
    if (lastButtonState[idx]) {
      if (currentButtonState[idx]) {
        currentButtonState[idx] = false;
      }
      else {
        currentButtonState[idx] = true;
      }
      lastButtonState[idx] = false;
    }
  }
  return currentButtonState[idx];
}
