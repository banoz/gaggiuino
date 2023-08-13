#pragma once

#include <Wire.h>
#include "sensors_state.h"
#include "measurements.h"

typedef enum {
  ONECUP = (1u << 0),
  TWOCUP = (1u << 1),
  MANUAL = (1u << 2),
  WATER = (1u << 3),
  STEAM = (1u << 4),
  ELEMENT = (1u << 5),
#ifdef TWI_CONTROLS_EX
  POWER = (1u << 6),
#endif
} twiControls;

void twiControlsInit(void);
void twiControlsRead(SensorState& currentState);
void twiControlsRead(SensorState& currentState, Measurements& weightMeasurements);
