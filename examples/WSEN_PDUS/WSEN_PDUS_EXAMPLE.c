/**
 ***************************************************************************************************
 * This file is part of Sensors SDK:
 * https://www.we-online.com/sensors, https://github.com/WurthElektronik/Sensors-SDK_STM32
 *
 * THE SOFTWARE INCLUDING THE SOURCE CODE IS PROVIDED “AS IS”. YOU ACKNOWLEDGE THAT WÜRTH ELEKTRONIK
 * EISOS MAKES NO REPRESENTATIONS AND WARRANTIES OF ANY KIND RELATED TO, BUT NOT LIMITED
 * TO THE NON-INFRINGEMENT OF THIRD PARTIES’ INTELLECTUAL PROPERTY RIGHTS OR THE
 * MERCHANTABILITY OR FITNESS FOR YOUR INTENDED PURPOSE OR USAGE. WÜRTH ELEKTRONIK EISOS DOES NOT
 * WARRANT OR REPRESENT THAT ANY LICENSE, EITHER EXPRESS OR IMPLIED, IS GRANTED UNDER ANY PATENT
 * RIGHT, COPYRIGHT, MASK WORK RIGHT, OR OTHER INTELLECTUAL PROPERTY RIGHT RELATING TO ANY
 * COMBINATION, MACHINE, OR PROCESS IN WHICH THE PRODUCT IS USED. INFORMATION PUBLISHED BY
 * WÜRTH ELEKTRONIK EISOS REGARDING THIRD-PARTY PRODUCTS OR SERVICES DOES NOT CONSTITUTE A LICENSE
 * FROM WÜRTH ELEKTRONIK EISOS TO USE SUCH PRODUCTS OR SERVICES OR A WARRANTY OR ENDORSEMENT
 * THEREOF
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE (license_terms_wsen_sdk.pdf)
 * LOCATED IN THE ROOT DIRECTORY OF THIS DRIVER PACKAGE.
 *
 * COPYRIGHT (c) 2021 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 **/

#include "WSEN_PDUS_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include "../SensorsSDK/WSEN_PDUS_25131308XXX01/WSEN_PDUS_25131308XXX01.h"

/* Sensor initialization function */
bool PDUS_init(void);

static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);

#ifdef WE_USE_FLOAT
static void debugPrintPressure_float(float pressureKPa);
static void debugPrintTemperature_float(float temperature);
#endif

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_pdusExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the example program for the PDUS sensor.");
  debugPrintln("Notes:");
  debugPrintln("* You need to use the correct sensor sub-type when converting pressure data (the default is PDUS_pdus3).");
  debugPrintln("* External 1k pull-up resistors should be used for SDA/SCL.");
  debugPrint("* The PDUS sensor uses 5V Vcc and logic levels. ");
  debugPrintln("Level conversion to 3.3V is required to talk with a STM32 (or any other 3.3V MCU).");

  /* init PDUS */
  if (false == PDUS_init())
  {
    debugPrintln("**** PDUS_Init() error. STOP ****");
    HAL_Delay(5);
    while(1);
  }

  /* LED on */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  HAL_Delay(5);
}

/**
 * @brief Example main loop code.
 * Call this function in main loop (infinite loop).
 */
void WE_pdusExampleLoop()
{
#ifdef WE_USE_FLOAT
  float presskPaP;
  float tempDegCP;
  if (WE_SUCCESS == PDUS_getPressureAndTemperature_float(PDUS_pdus3, &presskPaP, &tempDegCP))
  {
    debugPrintPressure_float(presskPaP);
    debugPrintTemperature_float(tempDegCP);
  }
  else
  {
    debugPrintln("**** PDUS_getPressureAndTemperature_float(): Failed ****");
  }
#else
  uint16_t pressureRaw;
  uint16_t temperatureRaw;
  if (WE_SUCCESS == PDUS_getRawPressureAndTemperature(&pressureRaw, &temperatureRaw))
  {
    char pressureStr[6];
    char temperatureStr[6];
    sprintf(pressureStr, "%u", pressureRaw);
    sprintf(temperatureStr, "%u", temperatureRaw);
    debugPrint("PDUS pressure (raw) = ");
    debugPrintln(pressureStr);
    debugPrint("PDUS temperature (raw) = ");
    debugPrintln(temperatureStr);
  }
  else
  {
    debugPrintln("**** PDUS_getPressureAndTemperature_float(): Failed ****");
  }
#endif // WE_USE_FLOAT

  /* Max. sample interval: See data sheet 25131308xxx01 - "Response time" = 2.2ms */
  HAL_Delay(1000);
}

/**
 * @brief Initializes the sensor for this example application.
 */
bool PDUS_init(void)
{
  /* Initialize sensor interface (i2c with PDUS address, burst mode activated) */
  WE_sensorInterface_t interface;
  PDUS_getInterface(&interface);
  interface.interfaceType = WE_i2c;
  interface.options.i2c.burstMode = 1;
  interface.handle = &hi2c1;
  PDUS_initInterface(&interface);

  /* Wait for boot */
  HAL_Delay(50);
  while (WE_SUCCESS != PDUS_isInterfaceReady())
  {
  }
  debugPrintln("**** PDUS_isInterfaceReady(): OK ****");

  HAL_Delay(5);

  return true;
}

static void debugPrint(char _out[])
{
  HAL_UART_Transmit(&huart2, (uint8_t *) _out, strlen(_out), 10);
}

static void debugPrintln(char _out[])
{
  HAL_UART_Transmit(&huart2, (uint8_t *) _out, strlen(_out), 10);
  char newline[2] = "\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *) newline, 2, 10);
}

#ifdef WE_USE_FLOAT

/**
 * @brief Prints the pressure to the debug interface.
 * @param pressureKPa  Pressure [kPa]
 */
static void debugPrintPressure_float(float pressureKPa)
{
  float pressureAbs = fabs(pressureKPa);
  uint16_t full = (uint16_t) pressureAbs;
  uint16_t decimals = (uint16_t) (((uint32_t) (pressureKPa * 10000)) % 10000); /* 4 decimal places */

  char bufferFull[4]; /* max 3 pre-decimal point positions */
  char bufferDecimals[5]; /* 4 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%04u", decimals);

  debugPrint("PDUS pressure (float) = ");
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" kPa");
}

/**
 * @brief Prints the temperature to the debug interface.
 * @param tempDegC Temperature [°C]
 */
static void debugPrintTemperature_float(float tempDegC)
{
  float tempAbs = fabs(tempDegC);
  uint16_t full = (uint16_t) tempAbs;
  uint16_t decimals = ((uint16_t) (tempAbs * 100)) % 100; /* 2 decimal places */

  char bufferFull[4]; /* 3 pre-decimal point positions (from 0 to 70 degrees Celsius) */
  char bufferDecimals[3]; /* 2 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%02u", decimals);

  debugPrint("PDUS temperature (float) = ");
  if (tempDegC < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" degrees Celsius");
}

#endif
