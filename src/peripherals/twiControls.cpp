#include "twiControls.h"
#include <Wire.h>

TwoWire twiControlWire(PB3, PB10);

uint8_t ledStateWithParity(const uint8_t);
void twiControlsCheckButtonState(const uint8_t, const twiControls, volatile uint8_t&, volatile uint8_t&);

void twiControlsInit(void) {
  twiControlWire.begin();
}

volatile uint8_t twiControlsButtonState = 0;
volatile uint8_t twiControlsLastButtonState = 0;
volatile uint8_t twiControlsButtonStateEx = 0;
volatile uint8_t twiControlsLastButtonStateEx = 0;

volatile uint8_t activeBrewButton = 0;

void twiControlsRead(SensorState& currentState) {

  twiControlWire.requestFrom(0x00, 1, 0x01, 1, true);

  uint8_t buttonState = twiControlWire.read();

  twiControlsCheckButtonState(buttonState, ONECUP, twiControlsButtonState, twiControlsLastButtonState);
  twiControlsCheckButtonState(buttonState, TWOCUP, twiControlsButtonState, twiControlsLastButtonState);
  twiControlsCheckButtonState(buttonState, MANUAL, twiControlsButtonState, twiControlsLastButtonState);
  twiControlsCheckButtonState(buttonState, WATER, twiControlsButtonState, twiControlsLastButtonState);

#ifdef TWI_CONTROLS_EX
  twiControlWire.requestFrom(0x00, 1, 0x02, 1, true);

  uint8_t buttonStateEx = twiControlWire.read();

  buttonStateEx = buttonStateEx << 4;

  twiControlsCheckButtonState(buttonStateEx, STEAM, twiControlsButtonStateEx, twiControlsLastButtonStateEx);
  twiControlsCheckButtonState(buttonStateEx, POWER, twiControlsButtonStateEx, twiControlsLastButtonStateEx);

  currentState.steamSwitchState = twiControlsButtonStateEx & STEAM;
  currentState.powerSwitchState = twiControlsButtonStateEx & POWER;
#endif

  activeBrewButton = twiControlsButtonState & 0x07u;

  currentState.brewSwitchState = twiControlsButtonState & 0x07u;
  currentState.hotWaterSwitchState = twiControlsButtonState & WATER;

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

void twiControlsCheckButtonState(const uint8_t state, const twiControls mask, volatile uint8_t& currentState, volatile uint8_t& lastState) {
  if (state & mask) {
    lastState |= mask;
  }
  else {
    if (lastState & mask) {
      if (currentState & mask) {
        currentState &= ~mask;
      }
      else {
        currentState |= mask;
      }
      lastState &= ~mask;
    }
  }
}
