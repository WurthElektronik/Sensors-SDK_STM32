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

#include "WSEN_TIDS_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gpio.h"
#include "i2c.h"
#include "usart.h"

#include "../SensorsSDK/WSEN_TIDS_2521020222501/WSEN_TIDS_2521020222501.h"


/* Comment/uncomment the following lines to disable/enable the examples for
 * each data type (see main function). */
/* Note: The float example won't compile unless WE_USE_FLOAT is defined. */
#define TIDS_EXAMPLE_ENABLE_FLOAT
#define TIDS_EXAMPLE_ENABLE_INT


/* Sensor initialization function */
bool TIDS_init(void);

/* Example modes for the TIDS sensor */
void TIDS_singleConversionMode(void);
void TIDS_continuousMode(void);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);

#ifdef TIDS_EXAMPLE_ENABLE_FLOAT
static void debugPrintTemperatureFloat(float temperature);
#endif

#ifdef TIDS_EXAMPLE_ENABLE_INT
static void debugPrintTemperatureInt(int16_t temperature);
#endif

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_tidsExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the example program for the TIDS sensor.");

  /* init TIDS */
  if (false == TIDS_init())
  {
    debugPrintln("**** TIDS_Init() error. STOP ****");
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
void WE_tidsExampleLoop()
{
  /* Comment/uncomment the following lines to switch between example modes. */
  TIDS_singleConversionMode();
//  TIDS_continuousMode();
}

/**
 * @brief Initializes the sensor for this example application.
 */
bool TIDS_init(void)
{
  /* Initialize sensor interface (i2c with TIDS address, burst mode deactivated) */
  WE_sensorInterface_t interface;
  TIDS_getInterface(&interface);
  interface.interfaceType = WE_i2c;
  interface.handle = &hi2c1;
  TIDS_initInterface(&interface);

  /* Wait for boot */
  HAL_Delay(50);
  while (WE_SUCCESS != TIDS_isInterfaceReady())
  {
  }
  debugPrintln("**** TIDS_isInterfaceReady(): OK ****");

  HAL_Delay(5);

  /* First communication test */
  uint8_t deviceIdValue = 0;
  if (WE_SUCCESS == TIDS_getDeviceID(&deviceIdValue))
  {
    if (deviceIdValue == TIDS_DEVICE_ID_VALUE) /* who am i ? - i am WSEN-TIDS! */
    {
      debugPrintln("**** TIDS_DEVICE_ID_VALUE: OK ****");
    }
    else
    {
      debugPrintln("**** TIDS_DEVICE_ID_VALUE: NOT OK ****");
      return false;
    }
  }
  else
  {
    debugPrintln("**** TIDS_getDeviceID(): NOT OK ****");
    return false;
  }

  /* Perform software reset */
  TIDS_softReset(TIDS_enable);
  HAL_Delay(5);
  TIDS_softReset(TIDS_disable);

  /* Enable auto address increment */
  TIDS_enableAutoIncrement(TIDS_enable);

  return true;
}

/**
* @brief Setup the sensor in single conversion mode.
* Read and print the measured values at a rate of 1 Hz.
*/
void TIDS_singleConversionMode()
{
  TIDS_state_t status = TIDS_disable;

  debugPrintln("Starting single conversion mode...");

  while (1)
  {
    /* Start a conversion (one shot) */
    TIDS_enableOneShot(TIDS_enable);

    /* Wait until the busy bit is set to 0 */
    do
    {
      TIDS_isBusy(&status);
    }
    while (status != TIDS_enable);

#ifdef TIDS_EXAMPLE_ENABLE_FLOAT
    float temperatureFloat = 0;
    if (TIDS_getTemperature(&temperatureFloat) == WE_SUCCESS)
    {
      debugPrintTemperatureFloat(temperatureFloat);
    }
    else
    {
      debugPrintln("**** TIDS_getTemperature(): NOT OK ****");
    }
#endif

#ifdef TIDS_EXAMPLE_ENABLE_INT
    int16_t temperatureInt;
    if (TIDS_getRawTemperature(&temperatureInt) == WE_SUCCESS)
    {
      debugPrintTemperatureInt(temperatureInt);
    }
    else
    {
      debugPrintln("**** TIDS_getRawTemperature(): NOT OK ****");
    }
#endif

    /* Perform software reset - must be done when using ONE_SHOT bit
     * (i.e. single conversion mode) */
    TIDS_softReset(TIDS_enable);
    HAL_Delay(5);
    TIDS_softReset(TIDS_disable);
	
    /* Wait 1s */
	HAL_Delay(1000);
	
  }
}

/**
* @brief Setup the sensor in continuous mode.
* Read and print the measured values at a rate of 1 Hz.
*/
void TIDS_continuousMode(void)
{
  /* Set ODR to 25Hz */
  TIDS_setOutputDataRate(TIDS_outputDataRate25Hz);

  /* Enable block data update */
  TIDS_enableBlockDataUpdate(TIDS_enable);

  /* Enable continuous mode */
  TIDS_enableContinuousMode(TIDS_enable);

  debugPrintln("Starting continuous mode...");

  while (1)
  {
#ifdef TIDS_EXAMPLE_ENABLE_FLOAT
    float temperatureFloat = 0;
    if (TIDS_getTemperature(&temperatureFloat) == WE_SUCCESS)
    {
      debugPrintTemperatureFloat(temperatureFloat);
    }
    else
    {
      debugPrintln("**** TIDS_getTemperature(): NOT OK ****");
    }
#endif

#ifdef TIDS_EXAMPLE_ENABLE_INT
    int16_t temperatureInt = 0;
    if (TIDS_getRawTemperature(&temperatureInt) == WE_SUCCESS)
    {
      debugPrintTemperatureInt(temperatureInt);
    }
    else
    {
      debugPrintln("**** TIDS_getRawTemperature(): NOT OK ****");
    }
#endif

    /* Wait 1s */
    HAL_Delay(1000);
  }
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


#ifdef TIDS_EXAMPLE_ENABLE_FLOAT

static void debugPrintTemperatureFloat(float tempDegC)
{
  float tempAbs = fabs(tempDegC);
  uint16_t full = (uint16_t) tempAbs;
  uint16_t decimals = ((uint16_t) (tempAbs * 100)) % 100; /* 2 decimal places */

  char bufferFull[4]; /* 3 pre-decimal point positions (from -40 to 125 degrees Celsius) */
  char bufferDecimals[3]; /* 2 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%02u", decimals);

  debugPrint("TIDS temperature (float) = ");
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

#ifdef TIDS_EXAMPLE_ENABLE_INT

static void debugPrintTemperatureInt(int16_t temperature)
{
  uint16_t full = ((uint16_t) abs(temperature)) / 100;
  uint16_t decimals = (uint16_t) (abs(temperature) % 100); /* 2 decimal places */

  char bufferFull[4]; /* 3 pre-decimal point positions (from -40 to 125 degrees Celsius) */
  char bufferDecimals[3]; /* 2 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%02u", decimals);

  debugPrint("TIDS temperature (int) = ");
  if (temperature < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" degrees Celsius");
}

#endif // TIDS_EXAMPLE_ENABLE_INT
