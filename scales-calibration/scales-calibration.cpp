#include <Arduino.h>
#include <HX711_2.h>
#include <EasyNextionLibrary.h>
#include <ADS1X15.h>
#include <PSM.h>
#include <SimpleKalmanFilter.h>
#include <math.h>
#include "sensors_state.h"

#define thermoDO      PB4
#define thermoDI      PA7 // not used
#define thermoCS      PA6
#define thermoCLK     PA5

#define zcPin         PA0
#define dimmerPin     PA1
#define relayPin      PA15  // PB0
#define brewPin       PC14
#define steamPin      PC15
#define valvePin      PC13
#define LOADCELL_1_DOUT_PIN  PB8
#define LOADCELL_2_DOUT_PIN  PB9
#define LOADCELL_1_SCK_PIN  PB0
#define LOADCELL_2_SCK_PIN  PB1
#define UART_LCD Serial2
#define USART_DEBUG Serial  // USB-CDC (Takes PA8,PA9,PA10,PA11)

#define PUMP_RANGE 100

HX711_2 loadcell(TIM3);

PSM pump(zcPin, dimmerPin, 100, FALLING, 1, 6);

ADS1015 ADS(0x48);

#if defined SINGLE_BOARD
#include <Adafruit_MAX31855.h>
SPIClass thermoSPI(thermoDI, thermoDO, thermoCLK);
Adafruit_MAX31855 thermocouple(thermoCS, &thermoSPI);
#else
#include <max6675.h>
SPIClass thermoSPI(thermoDI, thermoDO, thermoCLK);
MAX6675 thermocouple(thermoCS, &thermoSPI);
#endif

SimpleKalmanFilter smoothPressure(0.6f, 0.6f, 0.1f);
SimpleKalmanFilter smoothPumpFlow(0.1f, 0.1f, 0.01f);
SimpleKalmanFilter smoothWeightFlow(0.1f, 0.1f, 0.01f);
void pumpInit(int, float);
float getPumpFlowPerClick(float);
void setPumpFlow(float, float);

#if defined SINGLE_HX711_BOARD
unsigned char scale_clk = OUTPUT;
#else
unsigned char scale_clk = OUTPUT_OPEN_DRAIN;
#endif

volatile float lf = 4000.f; //-7050 worked for my 440lb max scale setup
volatile float rf = 4000.f; //-7050 worked for my 440lb max scale setup

volatile int cps, adc;
volatile float lw, rw, p, f, pz = 24.01f, temp;
volatile float smoothedPumpFlow, smoothedPressure, previousSmoothedPressure, pressureChangeSpeed;
volatile int pumpFlowValue = 100, timerValue = 10, threeWayValveValue = 0;
volatile unsigned long pumpUntil = 0;
volatile bool pumpFlow;

//Nextion object init
EasyNex myNex(UART_LCD);

void adsInit(void) {
  ADS.begin();
  ADS.setGain(0);      // 6.144 volt
  ADS.setDataRate(4);  // fast
  ADS.setMode(0);      // continuous mode
  ADS.readADC(0);      // first read to trigger
}

void setup() {
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  pinMode(valvePin, OUTPUT);
  digitalWrite(valvePin, LOW);

  pinMode(brewPin, INPUT_PULLUP);
  pinMode(steamPin, INPUT_PULLUP);

  USART_DEBUG.begin(115200);

  myNex.begin(115200);
  myNex.writeStr("rest");
  while (myNex.readNumber("initCheck") != 100) {
    delay(600);
  }

  USART_DEBUG.println("Init LC");
  loadcell.begin(LOADCELL_1_DOUT_PIN, LOADCELL_2_DOUT_PIN, LOADCELL_1_SCK_PIN, 128U, scale_clk);
  loadcell.set_scale();
  loadcell.power_up();
  loadcell.tare();
  USART_DEBUG.println("LC Initialized");

  USART_DEBUG.println("Init TC");
  thermocouple.begin();
  USART_DEBUG.println("TC Initialized");

  USART_DEBUG.println("Init ADC");
  ADS.begin();
  ADS.setGain(0);      // 6.144 volt
  ADS.setDataRate(4);  // fast
  ADS.setMode(0);      // continuous mode
  ADS.readADC(0);      // first read to trigger
  USART_DEBUG.println("ADC Initialized");

  USART_DEBUG.print("Measuring CPS: ");
  cps = pump.cps();
  USART_DEBUG.println(cps);

  if (cps > 80u) {
    pump.setDivider(2);
    pump.initTimer(cps > 110u ? 60u : 50u, TIM9);
  }
  else {
    pump.initTimer(cps > 55u ? 60u : 50u, TIM9);
  }

  USART_DEBUG.println("Initialized!");
}

void loop() {
  myNex.NextionListen();

  static unsigned long timer = millis();
  static unsigned long adcTimer = millis();
  static unsigned long flowTimer = millis();
  static unsigned long displayTimer = millis();
  static unsigned long debugOutputTimer = millis();
  static long rawValues[2];
  static float values[2];

  if (millis() > timer) {
    temp = thermocouple.readCelsius();

    loadcell.read(rawValues);

    float prevValues[2];
    prevValues[0] = values[0];
    prevValues[1] = values[1];

    loadcell.set_scale(lf, rf);
    loadcell.get_units(values, 8);

    f = (values[0] - prevValues[0] + values[1] - prevValues[1]) / ((millis() - timer + 200ul) / 1000.f);

    timer = millis() + 200ul;
  }

  if (millis() > adcTimer) {
    adc = ADS.getValue();
    p = (adc - 166) / 111.11f;

    previousSmoothedPressure = smoothedPressure;
    smoothedPressure = smoothPressure.updateEstimate(p);
    pressureChangeSpeed = (smoothedPressure - previousSmoothedPressure) / ((millis() - adcTimer) / 1000.f);

    adcTimer = millis() + 10ul;
  }

  if (millis() > displayTimer) {

    myNex.writeNum("cps", cps);
    myNex.writeNum("adc", adc);
    myNex.writeNum("tc", (int)roundf(temp * 10.f));
    myNex.writeNum("lc1", rawValues[0]);
    myNex.writeNum("lc2", rawValues[1]);

    myNex.writeNum("lf", (int)roundf(lf * 10.f));
    myNex.writeNum("rf", (int)roundf(rf * 10.f));
    myNex.writeNum("lw", (int)roundf(values[0] * 10.f));
    myNex.writeNum("rw", (int)roundf(values[1] * 10.f));

    myNex.writeNum("pr", (int)roundf(p * 10.f));
    myNex.writeNum("fl", (int)roundf(f * 10.f));
    myNex.writeNum("pz", (int)roundf(pz * 100.f));
    myNex.writeNum("pf", pumpFlowValue);
    myNex.writeNum("pt", timerValue);

    myNex.writeNum("s", millis() / 1000);
    myNex.writeNum("cd", pumpUntil > 0 ? (pumpUntil - millis()) / 1000 : 0);

    myNex.writeNum("br", digitalRead(brewPin));
    myNex.writeNum("st", digitalRead(steamPin));

    displayTimer = millis() + 200ul;
  }

  if (millis() > debugOutputTimer) {
    USART_DEBUG.printf("LC: %8d %8d\n", rawValues[0], rawValues[1]);
    USART_DEBUG.printf("ADC: %8d T: %4.1f\n", adc, temp);

    debugOutputTimer = millis() + 1000ul;
  }

  if (pumpUntil > millis())
  {
    if (pumpFlow)
    {
      pumpInit(cps, pz / 100.f);

      if (millis() > flowTimer)
      {
        long counter = pump.getCounter();
        pump.resetCounter();

        float pumpClicksPerSecond = counter / ((millis() - flowTimer) / 1000.f);

        float pumpFlow = pumpClicksPerSecond * getPumpFlowPerClick(smoothedPressure);

        smoothedPumpFlow = smoothPumpFlow.updateEstimate(pumpFlow);

        flowTimer = millis() + 50ul;
      }

      setPumpFlow(pumpFlowValue / 10.f, 10.f);
    }
    else
    {
      pump.set(pumpFlowValue);
    }
  }
  else
  {
    pump.set(0);
    if (pumpUntil > 0)
    {
      pumpUntil = 0;
      myNex.writeNum("btStart.val", 0); // disengage the start button
    }
  }

  digitalWrite(valvePin, threeWayValveValue > 0 ? HIGH : LOW);
}

////////////
/// PUMP
////////////

float flowPerClickAtZeroBar = 0.27f;
int maxPumpClicksPerSecond = 50;
float fpc_multiplier = 1.2f;
//https://www.desmos.com/calculator/axyl70gjae  - blue curve
constexpr std::array<float, 7> pressureInefficiencyCoefficient{ {
  0.045f,
  0.015f,
  0.0033f,
  0.000685f,
  0.000045f,
  0.009f,
  -0.0018f
} };

// Initialising some pump specific specs, mainly:
// - max pump clicks(dependant on region power grid spec)
// - pump clicks at 0 pressure in the system
void pumpInit(const int powerLineFrequency, const float pumpFlowAtZero) {
  // pump.freq = powerLineFrequency;
  maxPumpClicksPerSecond = powerLineFrequency;
  flowPerClickAtZeroBar = pumpFlowAtZero;
  fpc_multiplier = 60.f / (float)maxPumpClicksPerSecond;
}

// Models the flow per click, follows a compromise between the schematic and recorded findings
// plotted: https://www.desmos.com/calculator/eqynzclagu
float getPumpFlowPerClick(const float pressure) {
  float fpc = 0.f;
  fpc = (pressureInefficiencyCoefficient[5] / pressure + pressureInefficiencyCoefficient[6]) * (-pressure * pressure) + (flowPerClickAtZeroBar - pressureInefficiencyCoefficient[0]) - (pressureInefficiencyCoefficient[1] + (pressureInefficiencyCoefficient[2] - (pressureInefficiencyCoefficient[3] - pressureInefficiencyCoefficient[4] * pressure) * pressure) * pressure) * pressure;
  return fpc * fpc_multiplier;
}

// Currently there is no compensation for pressure measured at the puck, resulting in incorrect estimates
float getClicksPerSecondForFlow(const float flow, const float pressure) {
  if (flow == 0.f) return 0;
  float flowPerClick = getPumpFlowPerClick(pressure);
  float cps = flow / flowPerClick;
  return fminf(cps, (float)maxPumpClicksPerSecond);
}

// Function that returns the percentage of clicks the pump makes in it's current phase
inline float getPumpPct(const float targetPressure, const float flowRestriction) {
  if (targetPressure == 0.f) {
    return 0.f;
  }

  float diff = targetPressure - smoothedPressure;
  float maxPumpPct = flowRestriction <= 0.f ? 1.f : getClicksPerSecondForFlow(flowRestriction, smoothedPressure) / (float)maxPumpClicksPerSecond;
  float pumpPctToMaintainFlow = getClicksPerSecondForFlow(smoothedPumpFlow, smoothedPressure) / (float)maxPumpClicksPerSecond;

  if (diff > 2.f) {
    return fminf(maxPumpPct, 0.25f + 0.2f * diff);
  }

  if (diff > 0.f) {
    return fminf(maxPumpPct, pumpPctToMaintainFlow * 0.95f + 0.1f + 0.2f * diff);
  }

  if (pressureChangeSpeed < 0) {
    return fminf(maxPumpPct, pumpPctToMaintainFlow * 0.2f);
  }

  return 0;
}

// Sets the pump output based on a couple input params:
// - live system pressure
// - expected target
// - flow
// - pressure direction
void setPumpPressure(const float targetPressure, const float flowRestriction) {
  float pumpPct = getPumpPct(targetPressure, flowRestriction);
  pump.set((uint8_t)(pumpPct * PUMP_RANGE));
}

// Calculates pump percentage for the requested flow and updates the pump raw value
void setPumpFlow(const float targetFlow, const float pressureRestriction) {
  // If a pressure restriction exists then the we go into pressure profile with a flowRestriction
  // which is equivalent but will achieve smoother pressure management
  if (pressureRestriction > 0.f && smoothedPressure > pressureRestriction * 0.5f) {
    setPumpPressure(pressureRestriction, targetFlow);
  }
  else {
    float pumpPct = getClicksPerSecondForFlow(targetFlow, smoothedPressure) / (float)maxPumpClicksPerSecond;
    pump.set(pumpPct * PUMP_RANGE);
  }
}

////////////
/// TRIGGERS
////////////

void trigger0() { // home
  pumpFlowValue = 100;
  timerValue = 10;
  threeWayValveValue = 0;
  pumpUntil = 0;
  //loadcell.set_offset(0L, 0L);
}

void trigger1() { // tare
  loadcell.tare();
}

void trigger2() { // lf
  uint8_t b0 = lowByte(myNex.readByte());
  uint8_t b1 = lowByte(myNex.readByte());
  uint8_t b2 = lowByte(myNex.readByte());
  uint8_t b3 = lowByte(myNex.readByte());
  int b4 = (uint32_t)((b3 << 24) | (b2 << 16) | (b1 << 8) | b0);
  lf = b4 / 10.f;
}

void trigger3() { // rf
  uint8_t b0 = lowByte(myNex.readByte());
  uint8_t b1 = lowByte(myNex.readByte());
  uint8_t b2 = lowByte(myNex.readByte());
  uint8_t b3 = lowByte(myNex.readByte());
  int b4 = (uint32_t)((b3 << 24) | (b2 << 16) | (b1 << 8) | b0);
  rf = b4 / 10.f;
}

void trigger4() { // reset
  lf = 4000.f;
  rf = 4000.f;
}

void trigger5() { // pump/flow slider
  pumpFlow = myNex.readByte() > 0;
  pumpFlowValue = myNex.readByte();
}

void trigger6() { // timer slider
  timerValue = myNex.readByte();
}

void trigger7() { // PZ
  uint8_t b0 = (uint8_t)myNex.readByte();
  uint8_t b1 = (uint8_t)myNex.readByte();
  int b2 = (uint16_t)((b1 << 8) | b0);
  pz = b2 / 100.f;
}

void trigger8() { // 3WV
  threeWayValveValue = myNex.readByte();
}

void trigger9() { // Pump
  pumpUntil = millis() + myNex.readByte() * 1000;
}
