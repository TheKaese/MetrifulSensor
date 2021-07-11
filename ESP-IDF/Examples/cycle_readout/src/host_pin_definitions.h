/* 
   host_pin_definitions.h

   This file defines which host pins are used to interface to the 
   Metriful MS430 board. The relevant file section is selected 
   automatically when the board is chosen in the Arduino IDE.

   More detail is provided in the readme and User Guide.

   This file provides settings for the following host systems:
   * Arduino Uno
   * Arduino Nano 33 IoT
   * Arduino Nano
   * Arduino MKR WiFi 1010
   * ESP8266 (tested on NodeMCU and Wemos D1 Mini - other boards may require changes)
   * ESP32 (tested on DOIT ESP32 DEVKIT V1 - other boards may require changes)

   The Metriful MS430 is compatible with many more development boards
   than those listed. You can use this file as a guide to define the
   necessary settings for other host systems.

   Copyright 2020 Metriful Ltd. 
   Licensed under the MIT License - for further details see LICENSE.txt

   For code examples, datasheet and user guide, visit 
   https://github.com/metriful/sensor
*/

#ifndef ARDUINO_PIN_DEFINITIONS_H
#define ARDUINO_PIN_DEFINITIONS_H

// The examples have been tested on DOIT ESP32 DEVKIT V1 development board.
// Other ESP32 boards may require changes.

#define ISR_ATTRIBUTE IRAM_ATTR

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define READY_PIN GPIO_NUM_23 // Pin D23 connects to RDY
#define L_INT_PIN GPIO_NUM_18 // Pin D18 connects to LIT
#define S_INT_PIN GPIO_NUM_19 // Pin D19 connects to SIT
/* Also make the following connections:
  ESP32 pin D21 to MS430 pin SDA
  ESP32 pin D22 to MS430 pin SCL
  ESP32 pin GND to MS430 pin GND
  ESP32 pin 3V3 to MS430 pins VPU and VDD
  MS430 pin VIN is unused

  If a PPD42 particle sensor is used, also connect the following:
  ESP32 pin Vin to PPD42 pin 3
  ESP32 pin GND to PPD42 pin 1
  PPD42 pin 4 to MS430 pin PRT

  If an SDS011 particle sensor is used, connect the following:
  ESP32 pin Vin to SDS011 pin "5V"
  ESP32 pin GND to SDS011 pin "GND"
  SDS011 pin "25um" to MS430 pin PRT
  */

#endif
