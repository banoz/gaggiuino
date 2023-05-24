/* 09:32 15/03/2023 - change triggering comment */
#ifndef PINDEF_H
#define PINDEF_H

#ifdef PCBV1

#define thermoDO      PB14
#define thermoDI      PB15 // not used
#define thermoCS      PB12
#define thermoCLK     PB13

#define zcPin         PA0
#define brewPin       PB1
#define relayPin      PA6
#define dimmerPin     PB8
#define steamPin      PB0
#define valvePin      PC15
#define waterPin      PA12

#define steamValveRelayPin PC14
#define steamBoilerRelayPin PC13

#define HX711_sck_1   PA1
#define HX711_dout_1  PA2
#define HX711_dout_2  PA3

#define USART_LCD     Serial1 // PA9(TX) & PA10(RX)
#define USART_ESP     Serial2 // PA11(TX) & PA12(RX)
#define USART_DEBUG   Serial  // USB-CDC (Takes PA8,PA9,PA10,PA11)

#define hw_SCL        PB10
#define hw_SDA        PB3

#else

// STM32F4 pins definitions
#define thermoDO      PB4
#define thermoDI      PA7 // not used
#define thermoCS      PA6
#define thermoCLK     PA5

#define zcPin         PA0
#define brewPin       PC14
#define relayPin      PA15
#define dimmerPin     PA1
#define steamPin      PC15
#define valvePin      PC13
#define waterPin      PA12

#ifdef PCBV2
// PCB V2
#define steamValveRelayPin PB12
#define steamBoilerRelayPin PB13
#endif

#define HX711_sck_1   PB0
#define HX711_sck_2   PB1
#define HX711_dout_1  PB8
#define HX711_dout_2  PB9

#define USART_LCD     Serial2 // PA2(TX) & PA3(RX)
#define USART_ESP     Serial1 // PA9(TX) & PA10(RX)
#define USART_DEBUG   Serial  // USB-CDC (Takes PA8,PA9,PA10,PA11)

#define hw_SCL        PB6
#define hw_SDA        PB7

#endif



#endif
