#if defined(DEBUG_ENABLED) && defined(ARDUINO_ARCH_STM32)
  #include "dbg.h"
#endif
#if defined(ARDUINO_ARCH_AVR)
  #include <EEPROM.h>
#elif defined(ARDUINO_ARCH_STM32)
  #include "ADS1X15.h"
  #include <FlashStorage_STM32.h>
#endif
#include <EasyNextionLibrary.h>
#if defined(MAX31855_ENABLED)
  #include <Adafruit_MAX31855.h>
#else
  #include <max6675.h>
#endif
#if defined(SINGLE_HX711_CLOCK)
  #include <HX711_2.h>
#else
  #include <HX711.h>
#endif
#include <PSM.h>
#include "PressureProfile.h"

#if defined(ARDUINO_ARCH_AVR)
  // ATMega32P pins definitions
  #define zcPin 3
  #define thermoDO 4
  #define thermoCS 5
  #define thermoCLK 6
  #define steamPin A7
  #define relayPin 11  // PB3
  #define dimmerPin 10 // PB2
  #define valvePin 9  // PB1
  #define brewPin A6
  #define pressurePin A0
  #define HX711_dout_1 A2 //mcu > HX711 no 1 dout pin
  #define HX711_dout_2 A3 //mcu > HX711 no 2 dout pin
  #define HX711_sck_1 A1 //mcu > HX711 no 1 sck pin
  #define HX711_sck_2 -1 //mcu > HX711 no 2 sck pin
  #define USART_LCD Serial

  #if defined(ADCINTERRUPT_ENABLED)
    void initADC();
    void adcISR();
  #endif
#elif defined(ARDUINO_ARCH_STM32_V1)// if arch is stm32
  // STM32F4 pins definitions
  #define zcPin PA0
  #define thermoDO PB14
  #define thermoCS PB12
  #define thermoCLK PB13
  #define brewPin PB1
  #define relayPin PA6
  #define dimmerPin PB8
  #define valvePin PC15
  #define pressurePin ADS115_A0 //set here just for reference
  #define steamPin PB0
  #define HX711_sck_1 PA1 //mcu > HX711 no 1 sck pin
  #define HX711_sck_2 -1 // no connection
  #define HX711_dout_1 PA2 //mcu > HX711 no 1 dout pin
  #define HX711_dout_2 PA3 //mcu > HX711 no 2 dout pin
  #define USART_LCD Serial1
  //#define // USART_CH1 Serial
#elif defined(ARDUINO_ARCH_STM32)// if arch is stm32
  // STM32F4 pins definitions
  #define zcPin PA0
  #define thermoDO PB4
  #define thermoCS PA6
  #define thermoCLK PA5
  #define brewPin PC14
  #define relayPin PA15
  #define dimmerPin PA1
  #define valvePin PC13
  #define pressurePin ADS115_A0 //set here just for reference
  #define steamPin PC15
  #define relay1Pin PB13
  #define relay2Pin PB12
  #define HX711_sck_1 PB0 //mcu > HX711 no 1 sck pin
  #define HX711_sck_2 -1 // no connection
  #define HX711_dout_1 PB8 //mcu > HX711 no 1 dout pin
  #define HX711_dout_2 PB9 //mcu > HX711 no 2 dout pin
  #define USART_LCD Serial2 // PA2 & PA3
  #define USART_DEBUG Serial  // USB-CDC
  //#define // USART_CH1 Serial
#endif

// Define some const values
#define GET_KTYPE_READ_EVERY 250 // thermocouple data read interval not recommended to be changed to lower than 250 (ms)
#define GET_PRESSURE_READ_EVERY 100
#define GET_SCALES_READ_EVERY 200
#define REFRESH_SCREEN_EVERY 250 // Screen refresh interval (ms)
#define REFRESH_FLOW_EVERY 1000
#define DESCALE_PHASE1_EVERY 60000 // short pump pulses during descale
#define DESCALE_PHASE2_EVERY 120000 // long pause for scale softening
#define DESCALE_PHASE3_EVERY 4000 // short pause for pulse effficience activation
#define DELTA_RANGE 0.25f // % to apply as delta
#define MAX_SETPOINT_VALUE 110 //Defines the max value of the setpoint
#define EEPROM_RESET 1 //change this value if want to reset to defaults
#define PUMP_RANGE 127
#if defined(ARDUINO_ARCH_AVR)
  #define ZC_MODE FALLING
#else
  #define ZC_MODE RISING
#endif

#if defined(ARDUINO_ARCH_STM32_V1)// if arch is stm32
//If additional USART ports want ti eb used thy should be enable first
//HardwareSerial USART_CH(PA10, PA9);
  TwoWire Wire2(PB3, PB10);

  ADS1115 ADS(0x48, &Wire2);
#elif defined(ARDUINO_ARCH_STM32)
  ADS1115 ADS(0x48);
#endif
//Init the thermocouples with the appropriate pins defined above with the prefix "thermo"
#if defined(ADAFRUIT_MAX31855_H)
  Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);
#else
  MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
#endif
// EasyNextion object init
EasyNex myNex(USART_LCD);
//Banoz PSM - for more cool shit visit https://github.com/banoz  and don't forget to star
PSM pump(zcPin, dimmerPin, PUMP_RANGE, ZC_MODE, 1, 4);
//#######################__HX711_stuff__##################################
#if defined(SINGLE_HX711_CLOCK)
HX711_2 LoadCells;
#else
HX711 LoadCell_1; //HX711 1
HX711 LoadCell_2; //HX711 2
#endif


// Some vars are better global
//Timers
unsigned long pressureTimer = 0;
unsigned long thermoTimer = 0;
unsigned long scalesTimer = 0;
unsigned long flowTimer = 0;
unsigned long pageRefreshTimer = 0;
unsigned long modeRefreshTimer = 0;
unsigned long brewingTimer = 0;

//volatile vars
volatile float kProbeReadValue; //temp val
volatile float livePressure;
volatile float liveWeight;
volatile bool isPressureFalling;
volatile bool weightTargetHit;
//scales vars
/* If building for STM32 define the scales factors here */
float scalesF1 = 2407.5f;
float scalesF2 = 2382.0f;
float currentWeight;
float previousWeight;
float flowVal;

bool scalesPresent;
bool tareDone;

// brew detection vars
bool brewActive;
unsigned long brewUntil;
unsigned long steamUntil;
unsigned long flushUntil;

//PP&PI variables
//default phases. Updated in updatePressureProfilePhases.
Phase phaseArray[] = {
  Phase{1, 2, 6000},
  Phase{2, 2, 6000},
  Phase{0, 0, 2000},
  Phase{9, 9, 500},
  Phase{9, 6, 40000}
};
Phases phases {5,  phaseArray};
int preInfusionFinishedPhaseIdx = 3;

bool preinfusionFinished;

bool  preinfusionState;
bool  pressureProfileState;
bool  warmupEnabled;
//bool  flushEnabled;
//bool  descaleEnabled;
bool brewDeltaActive;
bool homeScreenScalesEnabled;
int  HPWR;
int  setPoint;
int  offsetTemp;
int  MainCycleDivider;
int  BrewCycleDivider;
int  preinfuseTime;
int preinfuseBar;
int preinfuseSoak;
int ppStartBar;
int ppFinishBar;
int ppHold;
int ppLength;
int selectedOperationalMode;

float weightTarget; // tenths of grams

// EEPROM  stuff
#define  EEP_SIG                 0
#define  EEP_SETPOINT            10
#define  EEP_OFFSET              20
#define  EEP_HPWR                40
#define  EEP_M_DIVIDER           60
#define  EEP_B_DIVIDER           80
#define  EEP_P_START             100
#define  EEP_P_FINISH            120
#define  EEP_P_HOLD              110
#define  EEP_P_LENGTH            130
#define  EEP_PREINFUSION         140
#define  EEP_P_PROFILE           160
#define  EEP_PREINFUSION_SEC     180
#define  EEP_PREINFUSION_BAR     190
#define  EEP_PREINFUSION_SOAK    170
#define  EEP_WARMUP              200
#define  EEP_HOME_ON_SHOT_FINISH 205
#define  EEP_GRAPH_BREW          210
#define  EEP_SCALES_F1           215
#define  EEP_SCALES_F2           220
#define  EEP_BREW_DELTA          225
#define  EEP_SHOT_TARGET_WEIGHT  230

void setup() {
  #ifdef USART_DEBUG
    USART_DEBUG.begin(115200); //debug channel
  #endif
  USART_LCD.begin(115200); // LCD comms channel

  // Various pins operation mode handling
  pinInit();

  // init the exteranl ADC
  ads1115Init();

  // Debug init if enabled
  dbgInit();

  // Turn off boiler in case init is unsecessful
  setBoiler(LOW);  // relayPin LOW

  #if defined(ARDUINO_ARCH_STM32)
    digitalWrite(relay1Pin, LOW);
    digitalWrite(relay2Pin, LOW);
  #endif

  //Pump
  pump.set(0);

  digitalWrite(valvePin, LOW);

  unsigned int cps = pump.cps();

  if (cps > 80u) {
    pump.setDivider(2);
  }

#if defined(ARDUINO_ARCH_AVR) && defined(ADCINTERRUPT_ENABLED)
  initADC();
  delay(100);
#endif

  while (brewState()) {
    lcdShowPopup("Brew Switch ON!");
    delay(0);
  }

  while (steamState()) {
    lcdShowPopup("Steam Switch ON!");
    delay(0);
  }

  // USART_CH1.println("Init step 4");
  // Will wait hereuntil full serial is established, this is done so the LCD fully initializes before passing the EEPROM values
  while (myNex.readNumber("safetyTempCheck") != 100 )
  {
    delay(100);
  }

  // USART_CH1.println("Init step 5");
  // Initialising the vsaved values or writing defaults if first start
  eepromInit();

  #if defined(ADAFRUIT_MAX31855_H)
    thermocouple.begin();
  #endif

  // Scales handling
  scalesInit();
  myNex.lastCurrentPageId = -1;

  myNex.writeNum("cps", cps);

  #ifdef NO_PHYSICAL_BUTTONS
  myNex.writeNum("physicalButtons", 0);
  #endif

  // USART_CH1.println("Init step 6");
}

//##############################################################################################################################
//############################################________________MAIN______________################################################
//##############################################################################################################################


//Main loop where all the logic is continuously run
void loop() {
  sensorsRead();
  myNex.NextionListen();
  pageValuesRefresh();
  brewDetect();
  modeSelect();
  if (myNex.currentPageId == 1 || myNex.currentPageId == 2 || myNex.currentPageId == 8 || homeScreenScalesEnabled ) {
    /* Only setting the weight activity value if it's been previously unset */
    calculateWeightAndFlow();
  }
  lcdRefresh();
}

//##############################################################################################################################
//#############################################___________SENSORS_READ________##################################################
//##############################################################################################################################

uint32_t getNextMillis(uint32_t offset)
{
  if (millis() > pump.getLastMillis() + 200)
  {
    #if defined(ADCINTERRUPT_ENABLED)
      adcISR();
    #endif
    return millis() + offset;
  }

  return pump.getLastMillis() + offset;
}

void sensorsRead() { // Reading the thermocouple temperature
  // static long thermoTimer;
  // Reading the temperature every 350ms between the loops
  if (millis() > thermoTimer) {
    kProbeReadValue = thermocouple.readCelsius();  // Making sure we're getting a value
    /*
    This *while* is here to prevent situations where the system failed to get a temp reading and temp reads as 0 or -7(cause of the offset)
    If we would use a non blocking function then the system would keep the SSR in HIGH mode which would most definitely cause boiler overheating
    */
    while (kProbeReadValue <= 0.0f || kProbeReadValue == NAN || kProbeReadValue > 165.0f) {
      /* In the event of the temp failing to read while the SSR is HIGH
      we force set it to LOW while trying to get a temp reading - IMPORTANT safety feature */
      setBoiler(LOW);
      if (millis() > thermoTimer) {
        kProbeReadValue = thermocouple.readCelsius();  // Making sure we're getting a value
        thermoTimer = millis() + GET_KTYPE_READ_EVERY;
      }
    }
    thermoTimer = getNextMillis(GET_KTYPE_READ_EVERY);
  }

  // Read pressure and store in a global var for further controls

  float lastPressure = livePressure;
  if (millis() > pressureTimer) {
    livePressure = getPressure();
    isPressureFalling = lastPressure > livePressure + 0.01f;
    pressureTimer = getNextMillis(GET_PRESSURE_READ_EVERY);
  }

  // Weight output
  if (scalesPresent) {
    if (millis() > scalesTimer) {
      #if defined(SINGLE_HX711_CLOCK)
        if (LoadCells.is_ready()) {
          float values[2];
          LoadCells.get_units(values);
          currentWeight = values[0] + values[1];
        }
      #else
        currentWeight = LoadCell_1.get_units() + LoadCell_2.get_units();
      #endif

      scalesTimer = getNextMillis(GET_SCALES_READ_EVERY);
    }
  }
}

void calculateWeightAndFlow() {
  if (brewActive) {
    if (scalesPresent) {
      if (millis() > flowTimer) {
        flowVal = currentWeight - previousWeight;
        previousWeight = currentWeight;
        flowTimer = millis() + REFRESH_FLOW_EVERY;
      }
    }
  } else {
    flowVal = 0.f;
    previousWeight = 0.f;
  }
}

// Stops the pump if setting active and dose/weight conditions met
bool stopOnWeight() {
  if (selectedOperationalMode <= 4) {
    if (scalesPresent && weightTarget > 1) {
      if (weightTargetHit || (currentWeight + .5f) >= weightTarget) {
        weightTargetHit = true;
        return true;
      }
    }
  }
  return false;
}

//##############################################################################################################################
//############################################______PRESSURE_____TRANSDUCER_____################################################
//##############################################################################################################################
#if defined(ARDUINO_ARCH_AVR) && defined(ADCINTERRUPT_ENABLED)
  volatile uint8_t adcData[4];
  volatile char adcInput[4];
  volatile int adcIndex = 0;
  bool adcInitialized = false;

  volatile uint8_t adc1Data[4];
  volatile int adc1Index = 0;

  void adcISR() {
    if (adcInitialized) {
      adcData[adcIndex] = ADCH;

      if (adcIndex == 1) {
        adc1Data[adc1Index] = adcData[1];

        adc1Index++;

        if (adc1Index > 3) {
          adc1Index = 0;
        }
      }

      adcIndex++;

      if (adcIndex > 3) {
        adcIndex = 0;
      }

      ADMUX = (1 << REFS0) | (1 << ADLAR) | (adcInput[adcIndex] & 0b00001111);
    }
  }

  void initADC() {
    ADMUX =  (1 << REFS0) | (1 << ADLAR) | 0b00001111;
    ADCSRB = (1 << ACME);
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADPS1);

    adcInput[0] = 0x08; // internal temperature sensor
    adcInput[1] = pressurePin - 14U;
    adcInput[2] = brewPin - 14U;
    adcInput[3] = steamPin - 14U;

    adcData[0] = 0;
    adcData[1] = 0;
    adcData[2] = 0xFFU;
    adcData[3] = 0xFFU;

    adc1Data[0] = 25U;
    adc1Data[1] = 25U;
    adc1Data[2] = 25U;
    adc1Data[3] = 25U;

    adcInitialized = true;
  }

  void onPSMInterrupt() {
    adcISR();
  }
#endif

float getPressure() {  //returns sensor pressure data
    // voltageZero = 0.5V --> 25.6 (8 bit) or 102.4 (10 bit) or 2666.7 (ADS 15 bit)
    // voltageMax = 4.5V --> 230.4 (8 bit) or 921.6 (10 bit) or 24000 (ADS 15 bit)
    // range 921.6 - 102.4 = 204.8 or 819.2 or 21333.3
    // pressure gauge range 0-1.2MPa - 0-12 bar
    // 1 bar = 17.1 or 68.27 or 1777.8

    #if defined(ARDUINO_ARCH_AVR)
      #if defined(ADCINTERRUPT_ENABLED)
        uint16_t avgPressure = 0U;
        avgPressure += (uint8_t)adc1Data[0];
        avgPressure += (uint8_t)adc1Data[1];
        avgPressure += (uint8_t)adc1Data[2];
        avgPressure += (uint8_t)adc1Data[3];
        avgPressure = avgPressure >> 2;
        if (avgPressure > 25U) {
          return adcInitialized ? (avgPressure - 25U) / 17.1f : 0.0f;
        } else {
          return 0.F;
        }
      #else
        return (analogRead(pressurePin) - 102) / 68.27f;
      #endif
    #elif defined(ARDUINO_ARCH_STM32)
      return (ADS.getValue() - 2666) / 1777.8f;
    #endif
}

void setPressure(float targetValue) {
  unsigned int pumpValue = 0;
  if (targetValue > 0.5f) {
    float diff = targetValue - livePressure;
    if (diff > 6.F) {
      pumpValue = PUMP_RANGE;
    } else if (diff > 0.F) {
      pumpValue = PUMP_RANGE / (1.f + exp(1.5f - diff / 1.4f));
    } else if (isPressureFalling && diff > -0.7F) {
      pumpValue = PUMP_RANGE / (1.f + exp(2.f - diff / 0.2f));
    }
  }
  pump.set(pumpValue);
}

//##############################################################################################################################
//############################################______PAGE_CHANGE_VALUES_REFRESH_____#############################################
//##############################################################################################################################

void pageValuesRefresh() {  // Refreshing our values on page changes
  if (myNex.currentPageId != myNex.lastCurrentPageId) {
    myNex.lastCurrentPageId = myNex.currentPageId;
    modeRefreshTimer = millis() + 500;
  }

  if (millis() > modeRefreshTimer) {
    selectedOperationalMode = myNex.readNumber("modeSelect");
    updatePressureProfilePhases();
    modeRefreshTimer = millis() + 1500;
  }
}

//#############################################################################################
//############################____OPERATIONAL_MODE_CONTROL____#################################
//#############################################################################################
void modeSelect() {
  // USART_CH1.println("MODE SELECT ENTER");
  switch (selectedOperationalMode) {
    case 0:
    case 1:
    case 2:
    case 4:
      if (!steamState()) newPressureProfile();
      else steamCtrl();
      break;
    case 3:
      // USART_CH1.println("MODE SELECT 3");
      manualPressureProfile();
      break;
    case 5:
      // USART_CH1.println("MODE SELECT 5");
      if (!steamState() ) justDoCoffee();
      else steamCtrl();
      break;
    case 6:
      // USART_CH1.println("MODE SELECT 6");
      deScale();
      break;
    case 7:
      // USART_CH1.println("MODE SELECT 7");
      break;
    case 8:
      // USART_CH1.println("MODE SELECT 8");
      break;
    case 9:
      // USART_CH1.println("MODE SELECT 9");
      if (!steamState() ) justDoCoffee();
      else steamCtrl();
      break;
    default:
      break;
  }
  // USART_CH1.println("MODE SELECT EXIT");
}

//#############################################################################################
//#########################____NO_OPTIONS_ENABLED_POWER_CONTROL____############################
//#############################################################################################

//delta stuff
inline static float TEMP_DELTA(float d) { return (d*DELTA_RANGE); }

void justDoCoffee() {
  // USART_CH1.println("DO_COFFEE ENTER");
  int HPWR_LOW = HPWR/MainCycleDivider;
  static double heaterWave;
  static bool heaterState;
  float BREW_TEMP_DELTA;
  // Calculating the boiler heating power range based on the below input values
  int HPWR_OUT = mapRange(kProbeReadValue, setPoint - 10, setPoint, HPWR, HPWR_LOW, 0);
  HPWR_OUT = constrain(HPWR_OUT, HPWR_LOW, HPWR);  // limits range of sensor values to HPWR_LOW and HPWR
  BREW_TEMP_DELTA = mapRange(kProbeReadValue, setPoint, setPoint+TEMP_DELTA(setPoint), TEMP_DELTA(setPoint), 0, 0);
  BREW_TEMP_DELTA = constrain(BREW_TEMP_DELTA, 0,  TEMP_DELTA(setPoint));

  // USART_CH1.println("DO_COFFEE TEMP CTRL BEGIN");
  if (brewActive) {
  // Applying the HPWR_OUT variable as part of the relay switching logic
    if (kProbeReadValue > setPoint-1.5f && kProbeReadValue < setPoint+0.25f && !preinfusionFinished ) {
      if (millis() - heaterWave > HPWR_OUT*BrewCycleDivider && !heaterState ) {
        setBoiler(LOW);
        heaterState=true;
        heaterWave=millis();
      } else if (millis() - heaterWave > HPWR_LOW*MainCycleDivider && heaterState ) {
        setBoiler(HIGH);
        heaterState=false;
        heaterWave=millis();
      }
    } else if (kProbeReadValue > setPoint-1.5f && kProbeReadValue < setPoint+(brewDeltaActive ? BREW_TEMP_DELTA : 0.f) && preinfusionFinished ) {
      if (millis() - heaterWave > HPWR*BrewCycleDivider && !heaterState ) {
        setBoiler(HIGH);
        heaterState=true;
        heaterWave=millis();
      } else if (millis() - heaterWave > HPWR && heaterState ) {
        setBoiler(LOW);
        heaterState=false;
        heaterWave=millis();
      }
    } else if (brewDeltaActive && kProbeReadValue >= (setPoint+BREW_TEMP_DELTA) && kProbeReadValue <= (setPoint+BREW_TEMP_DELTA+2.5f)  && preinfusionFinished ) {
      if (millis() - heaterWave > HPWR*MainCycleDivider && !heaterState ) {
        setBoiler(HIGH);
        heaterState=true;
        heaterWave=millis();
      } else if (millis() - heaterWave > HPWR && heaterState ) {
        setBoiler(LOW);
        heaterState=false;
        heaterWave=millis();
      }
    } else if(kProbeReadValue <= setPoint-1.5f) {
      setBoiler(HIGH);
    } else {
      setBoiler(LOW);
    }
  } else { //if brewState == 0
    if (kProbeReadValue < ((float)setPoint - 10.f)) {
      setBoiler(HIGH);
    } else if (kProbeReadValue < ((float)setPoint - 5.f)) {
      if (millis() - heaterWave > HPWR_OUT && !heaterState) {
        setBoiler(HIGH);
        heaterState=true;
        heaterWave=millis();
      } else if (millis() - heaterWave > HPWR_OUT / BrewCycleDivider && heaterState ) {
        setBoiler(LOW);
        heaterState=false;
        heaterWave=millis();
      }
    } else if (kProbeReadValue < ((float)setPoint - .25f)) {
      if (millis() - heaterWave > HPWR_OUT * BrewCycleDivider && !heaterState) {
        setBoiler(HIGH);
        heaterState=!heaterState;
        heaterWave=millis();
      } else if (millis() - heaterWave > HPWR_OUT / BrewCycleDivider && heaterState ) {
        setBoiler(LOW);
        heaterState=!heaterState;
        heaterWave=millis();
      }
    } else {
      setBoiler(LOW);
    }
  }
}

//#############################################################################################
//################################____STEAM_POWER_CONTROL____##################################
//#############################################################################################

void steamCtrl() {

  if (!brewActive) {
    if (livePressure <= 6.f) { // steam temp control, needs to be aggressive to keep steam pressure acceptable
      if ((kProbeReadValue > setPoint-10.f) && (kProbeReadValue <= 155.f)) setBoiler(HIGH);
      else setBoiler(LOW);
    } else setBoiler(LOW);
  } else if (brewActive) { //added to cater for hot water from steam wand functionality
    if ((kProbeReadValue > setPoint-10.f) && (kProbeReadValue <= 105.f)) {
      setBoiler(HIGH);
    } else {
      setBoiler(LOW);
    }
  } else setBoiler(LOW);
}

//#############################################################################################
//################################____LCD_REFRESH_CONTROL___###################################
//#############################################################################################

void lcdRefresh() {
  if (millis() > pageRefreshTimer) {
    /*LCD pressure output, as a measure to beautify the graphs locking the live pressure read for the LCD alone*/
    // if (brewActive) myNex.writeNum("pressure.val", (livePressure > 0.f) ? livePressure*10.f : 0.f);
    myNex.writeNum("cp", livePressure > 0.f ? int(livePressure * 10) : 0);
    /*LCD temp output*/
    myNex.writeNum("ct", int(kProbeReadValue) - offsetTemp);
    /*LCD weight output*/
    myNex.writeNum("cw", currentWeight > 0.f ? int(currentWeight * 10) : 0);
    /*LCD flow output*/
    myNex.writeNum("cf", flowVal > 0.f ? int(flowVal * 10) : 0);

    myNex.writeNum("ms", millis());

    dbgOutput();

    pageRefreshTimer = millis() + REFRESH_SCREEN_EVERY;
  }
}

//#############################################################################################
//###################################____SAVE_BUTTON____#######################################
//#############################################################################################
// Save the desired temp values to EEPROM
void trigger1() {
  #if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_STM32)
    int valueToSave;
    int allValuesUpdated;

    switch (myNex.currentPageId){
      case 1:
        break;
      case 2:
        break;
      case 3:
        // Saving ppStart,ppFin,ppHold and ppLength
        ppStartBar = myNex.readNumber("ppStart");
        EEPROM.put(EEP_P_START, ppStartBar);
        allValuesUpdated++;

        ppFinishBar = myNex.readNumber("ppFin");
        EEPROM.put(EEP_P_FINISH, ppFinishBar);
        allValuesUpdated++;

        ppHold = myNex.readNumber("ppHold");
        EEPROM.put(EEP_P_HOLD, ppHold);
        allValuesUpdated++;

        ppLength = myNex.readNumber("ppLength");
        EEPROM.put(EEP_P_LENGTH, ppLength);
        allValuesUpdated++;

        // Saving PI and PP
        preinfusionState = myNex.readNumber("piState");
        EEPROM.put(EEP_PREINFUSION, preinfusionState);
        allValuesUpdated++;

        pressureProfileState = myNex.readNumber("ppState");
        EEPROM.put(EEP_P_PROFILE, pressureProfileState);
        allValuesUpdated++;

        //Saved piSec
        preinfuseTime = myNex.readNumber("piSec");
        EEPROM.put(EEP_PREINFUSION_SEC, preinfuseTime);
        allValuesUpdated++;

        //Saved piBar
        preinfuseBar = myNex.readNumber("piBar");
        EEPROM.put(EEP_PREINFUSION_BAR, preinfuseBar);
        allValuesUpdated++;

        //Saved piSoak
        preinfuseSoak = myNex.readNumber("piSoak");
        EEPROM.put(EEP_PREINFUSION_SOAK, preinfuseSoak);
        allValuesUpdated++;

        if (allValuesUpdated == 9) {
          allValuesUpdated=0;
          myNex.writeStr("popupMSG.t0.txt","UPDATE SUCCESSFUL!");
        } else myNex.writeStr("popupMSG.t0.txt","ERROR!");
        myNex.writeStr("page popupMSG");
        break;
      case 0x0b:
        //Saving brewSettings
        valueToSave = myNex.readNumber("homeOnBrewFinish");
        if ( valueToSave >= 0 ) {
          EEPROM.put(EEP_HOME_ON_SHOT_FINISH, valueToSave);
          allValuesUpdated++;
        }
        valueToSave = myNex.readNumber("graphEnabled");
        if ( valueToSave >= 0 ) {
          EEPROM.put(EEP_GRAPH_BREW, valueToSave);
          allValuesUpdated++;
        }
        brewDeltaActive = myNex.readNumber("deltaState");
        EEPROM.put(EEP_BREW_DELTA, brewDeltaActive);
        allValuesUpdated++;

        if (allValuesUpdated == 3) {
          allValuesUpdated=0;
          myNex.writeStr("popupMSG.t0.txt","UPDATE SUCCESSFUL!");
        } else myNex.writeStr("popupMSG.t0.txt","ERROR!");
        myNex.writeStr("page popupMSG");
        break;
      case 0x0c:
        //Saving shotSettings
        weightTarget = myNex.readNumber("weightTarget") / 10.f;
        EEPROM.put(EEP_SHOT_TARGET_WEIGHT, weightTarget);
        allValuesUpdated++;

        if (allValuesUpdated == 1) {
          allValuesUpdated=0;
          myNex.writeStr("popupMSG.t0.txt","UPDATE SUCCESSFUL!");
        } else myNex.writeStr("popupMSG.t0.txt","ERROR!");
        myNex.writeStr("page popupMSG");
        break;
      case 5:
        break;
      case 6:
        // Reading the LCD side set values
        setPoint = myNex.readNumber("setPoint");
        EEPROM.put(EEP_SETPOINT, setPoint);
        allValuesUpdated++;

        // Saving offset
        offsetTemp = myNex.readNumber("offSet");
        EEPROM.put(EEP_OFFSET, offsetTemp);
        allValuesUpdated++;

        // Saving HPWR
        HPWR = myNex.readNumber("hpwr");
        EEPROM.put(EEP_HPWR, HPWR);
        allValuesUpdated++;

        // Saving mDiv
        MainCycleDivider = myNex.readNumber("mDiv");
        EEPROM.put(EEP_M_DIVIDER, MainCycleDivider);
        allValuesUpdated++;

        //Saving bDiv
        BrewCycleDivider = myNex.readNumber("bDiv");
        EEPROM.put(EEP_B_DIVIDER, BrewCycleDivider);
        allValuesUpdated++;

        if (allValuesUpdated == 5) {
          allValuesUpdated=0;
          myNex.writeStr("popupMSG.t0.txt","UPDATE SUCCESSFUL!");
        } else myNex.writeStr("popupMSG.t0.txt","ERROR!");
        myNex.writeStr("page popupMSG");
        break;
      case 7:
        // Saving warmup state
        warmupEnabled = myNex.readNumber("warmupState");
        EEPROM.put(EEP_WARMUP, warmupEnabled);
        allValuesUpdated++;

        scalesF1 = myNex.readNumber("scalesF1") / 10.f;
        EEPROM.put(EEP_SCALES_F1, scalesF1);
        allValuesUpdated++;

        scalesF1 = myNex.readNumber("scalesF2") / 10.f;
        EEPROM.put(EEP_SCALES_F2, scalesF1);
        allValuesUpdated++;

        if (allValuesUpdated == 3) {
          allValuesUpdated=0;
          myNex.writeStr("popupMSG.t0.txt","UPDATE SUCCESSFUL!");
        } else myNex.writeStr("popupMSG.t0.txt","ERROR!");
        myNex.writeStr("page popupMSG");
        break;
      default:
        break;
    }
  #endif
}

//#############################################################################################
//###################################_____SCALES_TARE____######################################
//#############################################################################################

void trigger2() {
  scalesTare();
}

void trigger3() {
  homeScreenScalesEnabled = myNex.readNumber("scalesEnabled");
  if (homeScreenScalesEnabled) {
    scalesTare();
  }
}

void trigger11() { // scales calibration
  int command = myNex.readByte();
  if (scalesPresent) {
    long values[2];
  #if defined(SINGLE_HX711_CLOCK)
    if (LoadCells.is_ready()) {
      LoadCells.read_average(values, 5U);
    }
  #else
    values[0] = LoadCell_1.get_value(5U);
    values[1] = LoadCell_2.get_value(5U);
  #endif
    uint32_t currentVal;
    switch (command) {
      case 0x01:
        currentVal = myNex.readNumber("weight1.val");
        myNex.writeNum("lc1.val", values[0] * 100 / (currentVal == 0 ? 1 : currentVal));
        break;
      case 0x02:
        currentVal = myNex.readNumber("lc1.val");
        myNex.writeNum("weight1.val", values[0] * 100 / (currentVal == 0 ? 1 : currentVal));
        break;
      case 0x03:
        currentVal = myNex.readNumber("weight2.val");
        myNex.writeNum("lc2.val", values[1] * 100 / (currentVal == 0 ? 1 : currentVal));
        break;
      case 0x04:
        currentVal = myNex.readNumber("lc2.val");
        myNex.writeNum("weight2.val", values[1] * 100 / (currentVal == 0 ? 1 : currentVal));
        break;
      default:
        break;
    }
  }
}

void trigger12() {
  int duration = myNex.readByte();
  if (duration == 0)
  {
    flushUntil = 0;
  }
  else
  {
    flushUntil = millis() + duration * 1000;
  }
}

void trigger13() {
  int duration = myNex.readByte();
  if (duration == 0)
  {
    brewUntil = 0;
  }
  else
  {
    brewUntil = millis() + duration * 1000;
  }
}

void trigger14() {
  int duration = myNex.readByte();
  if (duration == 0)
  {
    steamUntil = 0;
  }
  else
  {
    steamUntil = millis() + duration * 1000;
  }
}

//#############################################################################################
//###############################_____HELPER_FUCTIONS____######################################
//#############################################################################################

//Function to get the state of the brew switch button
bool brewState() {
  if (selectedOperationalMode <= 4 && brewUntil > millis()) {
    return true;
  }
  if (selectedOperationalMode == 5 && flushUntil > millis()) {
    return true;
  }
  #if defined(ARDUINO_ARCH_AVR) && defined(ADCINTERRUPT_ENABLED)
    return adcInitialized ? (uint8_t)adcData[2] < 128U : false;
  #elif defined(ARDUINO_ARCH_AVR)
    return analogRead(brewPin) < 128;
  #else
    return digitalRead(brewPin) == LOW; // pin will be low when switch is ON.
  #endif
}

//Function to get the state of the steam switch button
bool steamState() {
  if ((selectedOperationalMode <= 4 || selectedOperationalMode == 9) && steamUntil > millis()) {
    return true;
  }
  #if defined(ARDUINO_ARCH_AVR) && defined(ADCINTERRUPT_ENABLED)
    return adcInitialized ? (uint8_t)adcData[3] < 128U : false;
  #elif defined(ARDUINO_ARCH_AVR)
    return analogRead(steamPin) < 128;
  #else
    return digitalRead(steamPin) == LOW; // pin will be low when switch is ON.
  #endif
}

void brewTimer(uint32_t c) { // small function for easier timer start/stop
  myNex.writeNum("timerState", c);
}

// Actuating the heater element
void setBoiler(int val) {
  // USART_CH1.println("SET_BOILER BEGIN");
  #if defined(ARDUINO_ARCH_AVR)
  // USART_CH1.println("SET_BOILER AVR BLOCK BEGIN");
    if (val == HIGH) {
      PORTB |= _BV(PB3);  // boilerPin -> HIGH
    } else {
      PORTB &= ~_BV(PB3);  // boilerPin -> LOW
    }
  // USART_CH1.println("SET_BOILER AVR BLOCK END");
  #elif defined(ARDUINO_ARCH_STM32)// if arch is stm32
  // USART_CH1.println("SET_BOILER STM32 BLOCK BEGIN");
    if (val == HIGH) {
      digitalWrite(relayPin, HIGH);  // boilerPin -> HIGH
    } else {
      digitalWrite(relayPin, LOW);   // boilerPin -> LOW
    }
  // USART_CH1.println("SET_BOILER STM32 BLOCK END");
  #endif
  // USART_CH1.println("SET_BOILER END");
}

//#############################################################################################
//###############################____DESCALE__CONTROL____######################################
//#############################################################################################

void deScale() {
  static bool blink = true;
  static long timer = 0;
  static int currentCycleRead = 0;
  static int lastCycleRead = 0;
  static bool descaleFinished = false;
  if (brewActive && !descaleFinished) {
    if (currentCycleRead < lastCycleRead) { // descale in cycles for 5 times then wait according to the below condition
      if (blink == true) { // Logic that switches between modes depending on the $blink value
        pump.set(10);
        if (millis() - timer > DESCALE_PHASE1_EVERY) { // 60 sec
          blink = false;
          currentCycleRead += 2;
          timer = millis();
        }
      } else {
        pump.set(0);
        if (millis() - timer > DESCALE_PHASE2_EVERY) { // nothing for 120 sec
          blink = true;
          currentCycleRead += 4;
          timer = millis();
        }
      }
    } else {
      pump.set(30);
      if ((millis() - timer) > DESCALE_PHASE3_EVERY) { //set dimmer power to max descale value for 4 sec
        solenoidBeat();
        blink = true;
        currentCycleRead += 4; // need an overflow on 3rd cycle (34 -> 68 -> 102)
        lastCycleRead += 33;
        timer = millis();
      }
    }

    if (currentCycleRead >= 100) {
      descaleFinished = true;
    }

    if (millis() > pageRefreshTimer) {
      if (currentCycleRead < 100) {
        myNex.writeNum("j0.val", currentCycleRead);
      } else {
        myNex.writeNum("j0.val", 100);
      }
    }
  } else if (brewActive && descaleFinished) {
    pump.set(0);
    if ((millis() - timer) > 1000) {
      brewTimer(0);
      myNex.writeStr("t14.txt", "FINISHED!");
      timer = millis();
    }
  } else {
    pump.set(0);
    blink = true;
    currentCycleRead = 0;
    lastCycleRead = 30;
    descaleFinished = false;
    timer = millis();
  }
  //keeping it at temp
  justDoCoffee();
}

void solenoidBeat() {
  pump.set(PUMP_RANGE);
  digitalWrite(valvePin, LOW);
  delay(200);
  digitalWrite(valvePin, HIGH);
  delay(200);
  digitalWrite(valvePin, LOW);
  delay(200);
  digitalWrite(valvePin, HIGH);
  delay(200);
  digitalWrite(valvePin, LOW);
  delay(200);
  digitalWrite(valvePin, HIGH);
  pump.set(0);
}

//#############################################################################################
//###############################____PRESSURE_CONTROL____######################################
//#############################################################################################
void updatePressureProfilePhases() {
  switch (selectedOperationalMode)
  {
  case 0: // no PI and no PP -> Pressure fixed at 9bar
    phases.count = 1;
    setPhase(0, 9, 9, 1000);
    preInfusionFinishedPhaseIdx = 0;
    break;
  case 1: // Just PI no PP -> after PI pressure fixed at 9bar
    phases.count = 4;
    setPreInfusionPhases(0, preinfuseBar, preinfuseTime, preinfuseSoak);
    setPhase(3, 9, 9, 1000);
    preInfusionFinishedPhaseIdx = 3;
    break;
  case 2: // No PI, yes PP
    phases.count = 2;
    setPresureProfilePhases(0, ppStartBar, ppFinishBar, ppHold, ppLength);
    preInfusionFinishedPhaseIdx = 0;
    break;
  case 4: // Both PI + PP
    phases.count = 5;
    setPreInfusionPhases(0, preinfuseBar, preinfuseTime, preinfuseSoak);
    setPresureProfilePhases(3, ppStartBar, ppFinishBar, ppHold, ppLength);
    preInfusionFinishedPhaseIdx = 3;
    break;
  default:
    break;
  }
}

void setPreInfusionPhases(int startingIdx, int piBar, int piTime, int piSoakTime) {
    setPhase(startingIdx + 0, piBar/2, piBar, piTime * 1000 / 2);
    setPhase(startingIdx + 1, piBar, piBar, piTime * 1000 / 2);
    setPhase(startingIdx + 2, 0, 0, piSoakTime * 1000);
}

void setPresureProfilePhases(int startingIdx,int fromBar, int toBar, int holdTime, int dropTime) {
    setPhase(startingIdx + 0, fromBar, fromBar, holdTime * 1000);
    setPhase(startingIdx + 1, fromBar, toBar, dropTime * 1000);
}

void setPhase(int phaseIdx, int fromBar, int toBar, int timeMs) {
    phases.phases[phaseIdx].startPressure = fromBar;
    phases.phases[phaseIdx].endPressure = toBar;
    phases.phases[phaseIdx].durationMs = timeMs;
}

void newPressureProfile() {
  float newBarValue;

  if (brewActive) { //runs this only when brew button activated and pressure profile selected
    unsigned long timeInPP = millis() - brewingTimer;
    CurrentPhase currentPhase = phases.getCurrentPhase(timeInPP);
    newBarValue = phases.phases[currentPhase.phaseIndex].getPressure(currentPhase.timeInPhase);
    preinfusionFinished = currentPhase.phaseIndex >= preInfusionFinishedPhaseIdx;
  } else {
    newBarValue = 0.0f;
    preinfusionFinished = false;
  }
  setPressure(newBarValue);
  // Keep that water at temp
  justDoCoffee();
}

void manualPressureProfile() {
  if (selectedOperationalMode == 3) {
    int power_reading = myNex.readNumber("h0.val");
    setPressure(mapIntRange(power_reading, 0, 10, 0, ppStartBar, 1));
  }
  justDoCoffee();
}

//#############################################################################################
//###############################____INIT_AND_ADMIN_CTRL____###################################
//#############################################################################################

void brewDetect() {
  bool bState = brewState();
  if (bState) {
    if (!brewActive) {
      if (selectedOperationalMode <= 4) {
        scalesTare();
        currentWeight = 0.f;
        previousWeight = 0.f;
      }
    }
  }
  bool stoppedOnWeight = stopOnWeight();
  if (brewActive && scalesPresent && stoppedOnWeight) {
    myNex.writeNum("finalWeight", currentWeight > 0.f ? int(currentWeight * 10) : 0);
  }
  if (bState && !stoppedOnWeight) {
    digitalWrite(valvePin, HIGH);
    /* Applying the below block only when brew detected */
    if (selectedOperationalMode <= 4) {
      if (!brewActive) {
        preinfusionFinished = false;
      }
    } else if (selectedOperationalMode == 5) {
      pump.set(PUMP_RANGE); // setting the pump output target to 9 bars for non PP or PI profiles
    } else if (selectedOperationalMode == 9) {
      setPressure(9);
    }
    if (!brewActive) {
      brewTimer(1); // starting the timer
    }
    brewActive = true;
  } else {
    pump.set(0);
    digitalWrite(valvePin, LOW);
    brewTimer(bState ? 2 : 0); // stopping timer
    brewActive = false;
    if (!bState) {
      weightTargetHit = false;
    }
    /* UPDATE VARIOUS INTRASHOT TIMERS and VARS */
    brewingTimer = millis();
    /* Only resetting the brew activity value if it's been previously set */
    preinfusionFinished = false;
  }
}

void scalesInit() {

  #if defined(SINGLE_HX711_CLOCK)
    LoadCells.begin(HX711_dout_1, HX711_dout_2, HX711_sck_1);
    LoadCells.set_scale(scalesF1, scalesF2);
    LoadCells.power_up();

    delay(500);

    if (LoadCells.wait_ready_timeout()) {
      LoadCells.tare(5);
      scalesPresent = true;
    }
  #else
    LoadCell_1.begin(HX711_dout_1, HX711_sck_1);
    LoadCell_2.begin(HX711_dout_2, HX711_sck_2);
    LoadCell_1.set_scale(scalesF1); // calibrated val1
    LoadCell_2.set_scale(scalesF2); // calibrated val2

    delay(500);

    if (LoadCell_1.is_ready() && LoadCell_2.is_ready()) {
      scalesPresent = true;
      LoadCell_1.tare();
      LoadCell_2.tare();
    }
  #endif
}

void scalesTare() {
  if (scalesPresent) {
    #if defined(SINGLE_HX711_CLOCK)
      if (LoadCells.wait_ready_timeout()) {
        LoadCells.tare(5);
      }
    #else
      if (LoadCell_1.wait_ready_timeout(300) && LoadCell_2.wait_ready_timeout(300)) {
        LoadCell_1.tare(2);
        LoadCell_2.tare(2);
      }
    #endif
  }
  tareDone = true;
}

void eepromInit() {
  int sig;

  #if defined(ARDUINO_ARCH_AVR)
  sig = EEPROM.read(EEP_SIG);
  setPoint = EEPROM.read(EEP_SETPOINT);
  preinfuseSoak = EEPROM.read(EEP_PREINFUSION_SOAK);
  #elif defined(ARDUINO_ARCH_STM32)
  EEPROM.get(EEP_SIG, sig);
  EEPROM.get(EEP_SETPOINT, setPoint);
  EEPROM.get(EEP_PREINFUSION_SOAK, preinfuseSoak);
  #endif

  //If it's the first boot we'll need to set some defaults
  if (sig != EEPROM_RESET || setPoint <= 0 || setPoint > 160 || preinfuseSoak > 500) {
    // USART_CH.print("SECU_CHECK FAILED! Applying defaults!");
    EEPROM.put(EEP_SIG, EEPROM_RESET);
    //The values can be modified to accomodate whatever system it tagets
    //So on first boot it writes and reads the desired system values
    setDefaultValues();

    valuesWriteToEEPROM();

    EEPROM.put(EEP_HOME_ON_SHOT_FINISH, 1);
    EEPROM.put(EEP_GRAPH_BREW, 1);

    myNex.writeNum("homeOnBrewFinish", 1);
    myNex.writeNum("graphEnabled", 1);
  } else {
    // Applying our saved EEPROM saved values
    valuesLoadFromEEPROM();

    myNex.writeNum("piState", preinfusionState); // preinfusion state value which should be 0 or 1
    myNex.writeNum("piSec", preinfuseTime);
    myNex.writeNum("piBar", preinfuseBar);
    myNex.writeNum("piSoak", preinfuseSoak); // pre-infusion soak value
    myNex.writeNum("ppState", pressureProfileState); // pressure profile state value which should be 0 or 1
    myNex.writeNum("ppStart", ppStartBar);
    myNex.writeNum("ppFin", ppFinishBar);
    myNex.writeNum("ppHold", ppHold); // pp start pressure hold
    myNex.writeNum("ppLength", ppLength); // pp shot length
    myNex.writeNum("setPoint", setPoint);  // setPoint value from the lcd
    myNex.writeNum("offSet", offsetTemp);  // offset value from the lcd
    myNex.writeNum("hpwr", HPWR);  // brew time delay used to apply heating in waves
    myNex.writeNum("mDiv", MainCycleDivider);  // delay divider
    myNex.writeNum("bDiv", BrewCycleDivider);  // delay divider
    myNex.writeNum("warmupState", warmupEnabled);
    myNex.writeNum("scalesF1", scalesF1 * 10);
    myNex.writeNum("scalesF2", scalesF2 * 10);
    myNex.writeNum("deltaState", brewDeltaActive);
    myNex.writeNum("weightTarget", weightTarget * 10);
  }
}

void setDefaultValues() {
  preinfusionState        = true;
  preinfuseTime           = 10;
  preinfuseBar            = 2;
  preinfuseSoak           = 10; // pre-infusion soak value
  pressureProfileState    = true; // reding the pressure profile state value which should be 0 or 1
  ppStartBar              = 9;
  ppFinishBar             = 6;
  ppHold                  = 5; // pp start pressure hold
  ppLength                = 15; // pp shot length
  setPoint                = 100;  // reading the setPoint value from the lcd
  offsetTemp              = 7;  // reading the offset value from the lcd
  HPWR                    = 550;  // reading the brew time delay used to apply heating in waves
  MainCycleDivider        = 5;  // reading the delay divider
  BrewCycleDivider        = 2;  // reading the delay divider
  warmupEnabled           = false;
  scalesF1                = 4000.f;
  scalesF2                = 4000.f;
  brewDeltaActive         = true;
  weightTarget            = 0.f;
}

void valuesLoadFromEEPROM() {
  int init_val;

  // Loading the saved values fro EEPROM and sending them to the LCD

  EEPROM.get(EEP_SETPOINT, setPoint);// reading setpoint value from eeprom
  EEPROM.get(EEP_OFFSET, offsetTemp); // reading offset value from eeprom
  EEPROM.get(EEP_HPWR, HPWR);//reading HPWR value from eeprom
  EEPROM.get(EEP_M_DIVIDER, MainCycleDivider);//reading main cycle div from eeprom
  EEPROM.get(EEP_B_DIVIDER, BrewCycleDivider);//reading brew cycle div from eeprom

  EEPROM.get(EEP_PREINFUSION, preinfusionState);//reading preinfusion checkbox value from eeprom
  EEPROM.get(EEP_PREINFUSION_SEC, preinfuseTime);//reading preinfusion time value from eeprom
  EEPROM.get(EEP_PREINFUSION_BAR, preinfuseBar);//reading preinfusion pressure value from eeprom
  EEPROM.get(EEP_PREINFUSION_SOAK, preinfuseSoak);//reading preinfusion soak times value from eeprom

  EEPROM.get(EEP_P_PROFILE, pressureProfileState);//reading pressure profile checkbox value from eeprom
  EEPROM.get(EEP_P_START, ppStartBar);//reading pressure profile start value from eeprom
  EEPROM.get(EEP_P_FINISH, ppFinishBar);// reading pressure profile finish value from eeprom
  EEPROM.get(EEP_P_HOLD, ppHold);// reading pressure profile hold value from eeprom
  EEPROM.get(EEP_P_LENGTH, ppLength);// reading pressure profile length value from eeprom

  // Brew page settings
  EEPROM.get(EEP_HOME_ON_SHOT_FINISH, init_val);//reading bre time value from eeprom
  if (  init_val == 0 || init_val == 1 ) {
    myNex.writeNum("homeOnBrewFinish", init_val);
  }

  EEPROM.get(EEP_GRAPH_BREW, init_val);//reading preinfusion pressure value from eeprom
  if (  init_val == 0 || init_val == 1) {
    myNex.writeNum("graphEnabled", init_val);
  }

  EEPROM.get(EEP_BREW_DELTA, brewDeltaActive);//reading preinfusion pressure value from eeprom

  // Warmup checkbox value
  EEPROM.get(EEP_WARMUP, warmupEnabled);//reading preinfusion pressure value from eeprom

  // Scales values
  EEPROM.get(EEP_SCALES_F1, scalesF1);//reading scale factors value from eeprom
  EEPROM.get(EEP_SCALES_F2, scalesF2);//reading scale factors value from eeprom

  EEPROM.get(EEP_SHOT_TARGET_WEIGHT, weightTarget);
}

void valuesWriteToEEPROM() {
    EEPROM.put(EEP_SETPOINT, setPoint);
    EEPROM.put(EEP_OFFSET, offsetTemp);
    EEPROM.put(EEP_HPWR, HPWR);
    EEPROM.put(EEP_M_DIVIDER, MainCycleDivider);
    EEPROM.put(EEP_B_DIVIDER, BrewCycleDivider);
    EEPROM.put(EEP_PREINFUSION, preinfusionState);
    EEPROM.put(EEP_PREINFUSION_SEC, preinfuseTime);
    EEPROM.put(EEP_PREINFUSION_BAR, preinfuseBar);
    EEPROM.put(EEP_PREINFUSION_SOAK, preinfuseSoak);
    EEPROM.put(EEP_P_PROFILE, pressureProfileState);
    EEPROM.put(EEP_P_START, ppStartBar);
    EEPROM.put(EEP_P_FINISH, ppFinishBar);
    EEPROM.put(EEP_P_HOLD, ppHold);
    EEPROM.put(EEP_P_LENGTH, ppLength);
    EEPROM.put(EEP_WARMUP, warmupEnabled);
    EEPROM.put(EEP_SCALES_F1, scalesF1);
    EEPROM.put(EEP_SCALES_F2, scalesF2);
    EEPROM.put(EEP_BREW_DELTA, brewDeltaActive);
    EEPROM.put(EEP_SHOT_TARGET_WEIGHT, weightTarget);
}

void lcdShowPopup(const char *msg) {
  static unsigned int lcdPopupTimer;
  if (lcdPopupTimer < millis()) {
    myNex.writeStr("popupMSG.t0.txt", msg);
    myNex.writeStr("page popupMSG");
    lcdPopupTimer = millis() + 1000u;
  }
}

void ads1115Init() {
#if defined(ARDUINO_ARCH_STM32)
  ADS.begin();
  ADS.setWireClock(400000);
  ADS.setGain(0);      // 6.144 volt
  ADS.setDataRate(5);  // 250 SPS
  ADS.setMode(0);      // continuous mode
  ADS.readADC(0);      // first read to trigger
#endif
}

void pinInit() {
  pinMode(relayPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
  #if !defined(ADCINTERRUPT_ENABLED)
    pinMode(brewPin, INPUT_PULLUP);
    pinMode(steamPin, INPUT_PULLUP);
  #endif
  #if defined(ARDUINO_ARCH_STM32)
    pinMode(relay1Pin, OUTPUT);
    pinMode(relay2Pin, OUTPUT);
  #endif

  //pinMode(HX711_dout_1, INPUT_PULLUP);
  //pinMode(HX711_dout_2, INPUT_PULLUP);
}

void dbgInit() {
  #if defined(STM32F4xx) && defined(DEBUG_ENABLED)
  analogReadResolution(12);
  #endif
}
void dbgOutput() {
  #if defined(STM32F4xx) && defined(DEBUG_ENABLED)
  int VRef = readVref();
  myNex.writeNum("debug1",readTempSensor(VRef));
  myNex.writeNum("debug2",ADS.getError());
  #endif
}
