/* 09:32 15/03/2023 - change triggering comment */
#include "pump.h"
#include "pindef.h"
#include <PSM.h>
#include "utils.h"

PSM pump(zcPin, dimmerPin, PUMP_RANGE, ZC_MODE, 1, 4);

float flowPerClickAtZeroBar = 0.27f;
int maxPumpClicksPerSecond = 50;
float fpc_multiplier = 1.2f;

const std::array<float, 8> pressureInefficiencyCoefficient {
  649.905892665f,
    -43.052540008f,
    -25.060116107f,
    9.645286367f,
    -1.437319612f,
    0.101542238f,
    -0.003283776f,
    0.000035942f,
};

// Initialising some pump specific specs, mainly:
// - max pump clicks(dependant on region power grid spec)
// - pump clicks at 0 pressure in the system
void pumpInit(const int powerLineFrequency, const float pumpFlowAtZero) {
  maxPumpClicksPerSecond = powerLineFrequency;
  flowPerClickAtZeroBar = pumpFlowAtZero;
  fpc_multiplier = 60.f / (float)maxPumpClicksPerSecond / 3000.f;
}

// Function that returns the percentage of clicks the pump makes in it's current phase
inline float getPumpPct(const float targetPressure, const float flowRestriction, const SensorState& currentState) {
  if (targetPressure == 0.f) {
    return 0.f;
  }

  float diff = targetPressure - currentState.smoothedPressure;
  float maxPumpPct = flowRestriction <= 0.f ? 1.f : getClicksPerSecondForFlow(flowRestriction, currentState.smoothedPressure) / (float)maxPumpClicksPerSecond;
  float pumpPctToMaintainFlow = getClicksPerSecondForFlow(currentState.smoothedPumpFlow, currentState.smoothedPressure) / (float)maxPumpClicksPerSecond;

  if (diff > 2.f) {
    return fminf(maxPumpPct, 0.25f + 0.2f * diff);
  }

  if (diff > 0.f) {
    return fminf(maxPumpPct, pumpPctToMaintainFlow * 0.95f + 0.1f + 0.2f * diff);
  }

  if (currentState.isPressureFalling) {
    return fminf(maxPumpPct, pumpPctToMaintainFlow * 0.2f);
  }

  return 0;
}

// Sets the pump output based on a couple input params:
// - live system pressure
// - expected target
// - flow
// - pressure direction
void setPumpPressure(const float targetPressure, const float flowRestriction, const SensorState& currentState) {
  float pumpPct = getPumpPct(targetPressure, flowRestriction, currentState);
  setPumpToRawValue((uint8_t)(pumpPct * PUMP_RANGE));
}

void setPumpOff(void) {
  pump.set(0);
}

void setPumpFullOn(void) {
  pump.set(PUMP_RANGE);
}

void setPumpToRawValue(const uint8_t val) {
  pump.set(val);
}

long getAndResetClickCounter(void) {
  long counter = pump.getCounter();
  pump.resetCounter();
  return counter;
}

int getCPS(void) {
  unsigned int cps = pump.cps();
  if (cps > 80u) {
    pump.setDivider(2);
  }
  return cps;
}

void pumpPhaseShift(void) {
  pump.shiftDividerCounter();
}

// The function is split to compensate for the rapid decline in fpc at low pressures
// float fpc = (flowPerClickAtZeroBar - pressureInefficiencyConstant0) + (pressureInefficiencyConstant1 + (pressureInefficiencyConstant2 + (pressureInefficiencyConstant3 + (pressureInefficiencyConstant4 + (pressureInefficiencyConstant5 + pressureInefficiencyConstant6 * pressure) * pressure) * pressure) * pressure) * pressure) * pressure;
// Polinomyal func that should in theory calc fpc faster than the above.
float getPumpFlowPerClick(const float pressure) {
  // float fpc = (flowPerClickAtZeroBar - pressureInefficiencyCoefficient[0]) + (pressureInefficiencyCoefficient[1] + (pressureInefficiencyCoefficient[2] + (pressureInefficiencyCoefficient[3] + (pressureInefficiencyCoefficient[4] + (pressureInefficiencyCoefficient[5] + pressureInefficiencyCoefficient[6] * pressure) * pressure) * pressure) * pressure) * pressure) * pressure;
  float fpc = 0.f;
  for (int i = 7; i > 0; i--) {
    fpc = (fpc + pressureInefficiencyCoefficient[i]) * pressure;
  }
  fpc += pressureInefficiencyCoefficient[0]; // in cm3/min

  return fmaxf(fpc, 0.f) * fpc_multiplier;
}

// Follows the schematic from https://www.cemegroup.com/solenoid-pump/e5-60 modified to per-click
float getPumpFlow(const float cps, const float pressure) {
  return cps * getPumpFlowPerClick(pressure);
}

// Currently there is no compensation for pressure measured at the puck, resulting in incorrect estimates
float getClicksPerSecondForFlow(const float flow, const float pressure) {
  if (flow == 0.f) return 0;
  float flowPerClick = getPumpFlowPerClick(pressure);
  float cps = flow / flowPerClick;
  return fminf(cps, maxPumpClicksPerSecond);
}

// Calculates pump percentage for the requested flow and updates the pump raw value
void setPumpFlow(const float targetFlow, const float pressureRestriction, const SensorState& currentState) {
  // If a pressure restriction exists then the we go into pressure profile with a flowRestriction
  // which is equivalent but will achieve smoother pressure management
  if (pressureRestriction > 0.f && currentState.smoothedPressure > pressureRestriction * 0.5f) {
    setPumpPressure(pressureRestriction, targetFlow, currentState);
  }
  else {
    float pumpPct = getClicksPerSecondForFlow(targetFlow, currentState.smoothedPressure) / (float)maxPumpClicksPerSecond;
    setPumpToRawValue(pumpPct * PUMP_RANGE);
  }
}
