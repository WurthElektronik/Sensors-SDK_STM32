/*
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
 * COPYRIGHT (c) 2022 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 */

/**
 * @file
 * @brief WSEN_HIDS example.
 *
 * Demonstrates basic usage of the HIDS humidity sensor connected via I2C.
 */

#include "WSEN_HIDS_2525020210001_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gpio.h"
#include "i2c.h"
#include "usart.h"

#include <platform.h>

#include "../SensorsSDK/WSEN_HIDS_2525020210001/WSEN_HIDS_2525020210001.h"


/* Comment/uncomment the following lines to disable/enable the examples for
 * each data type (see WE_hidsExampleLoop() function). */
/* Note: The float example won't compile unless WE_USE_FLOAT is defined. */
#define HIDS_EXAMPLE_ENABLE_FLOAT
#define HIDS_EXAMPLE_ENABLE_INT16
#define HIDS_EXAMPLE_ENABLE_INT8

/* Sensor interface configuration */
static WE_sensorInterface_t hids;

/* Sensor initialization function */
static bool HIDS_init(void);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_hidsExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the example program for the HIDS sensor connected via I2C.");

  /* init HIDS */
  if (false == HIDS_init())
  {
    debugPrintln("**** HIDS_Init() error. STOP ****");
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
void WE_hidsExampleLoop()
{
  bool waitForMeasurement = true;
  HIDS_state_t humStatus = HIDS_disable;
  HIDS_state_t tempStatus = HIDS_disable;
  HIDS_state_t oneShotStatus = HIDS_enable;

  if (WE_FAIL == HIDS_enableOneShot(&hids, HIDS_enable)) /* trigger a single measurement - oneshot it is! */
  {
    debugPrintln("**** HIDS_enOneShot(enable): NOT OK ****");
  }

  HAL_Delay(1);

  while (waitForMeasurement == true)
  {
    /* Get status info for humidity, temperature and oneshot */
    if (WE_FAIL == HIDS_isHumidityDataAvailable(&hids, &humStatus))
    {
      debugPrintln("**** HIDS_getHumStatus(): NOT OK ****");
    }

    if (WE_FAIL == HIDS_isTemperatureDataAvailable(&hids, &tempStatus))
    {
      debugPrintln("**** HIDS_getTempStatus(): NOT OK ****");
    }

    if (WE_FAIL == HIDS_isOneShotEnabled(&hids, &oneShotStatus))
    {
      debugPrintln("**** HIDS_getOneShotState(): NOT OK ****");
    }

    /* Wait until both status bits are '1' and the oneshot bit was reset to '0' => indicating measurement was done */
    if ((humStatus == HIDS_enable)
        && (tempStatus == HIDS_enable)
          && (oneShotStatus == HIDS_disable))
    {
      waitForMeasurement = false;
    }
    else
    {
      HAL_Delay(1);
    }
  }

#ifdef HIDS_EXAMPLE_ENABLE_FLOAT

  /* Example using float values */
  float humidity = 0.0f;
  float temperature = 0.0f;

  if (HIDS_getHumidity_float(&hids, &humidity) == WE_SUCCESS)
  {
    float humidityAbs = humidity; /* humidity is preclamped in the 0 ... 100 % range by the HIDS_getHumidity_float() function already. no fabs() needed here! */
    uint16_t full = (uint16_t) humidityAbs;
    uint16_t decimals = ((uint16_t) (humidityAbs * 100)) % 100; /* 2 decimal places */

    char bufferFull[4]; /* 3 pre-decimal point positions (max. 100% ) */
    char bufferDecimals[3]; /* 2 decimal places */
    sprintf(bufferFull, "%u", full);
    sprintf(bufferDecimals, "%02u", decimals);

    debugPrint("HIDS RH (float) = ");
    debugPrint(bufferFull);
    debugPrint(".");
    debugPrint(bufferDecimals);
    debugPrintln("%");
  }

  if (HIDS_getTemperature_float(&hids, &temperature) == WE_SUCCESS)
  {
    float temperatureAbs = fabs(temperature);
    uint16_t full = (uint16_t) temperatureAbs;
    uint16_t decimals = ((uint16_t) (temperatureAbs * 100)) % 100; /* 2 decimal places */

    char bufferFull[4]; /* 3 pre-decimal point positions (from -40 to 100 degrees Celsius) */
    char bufferDecimals[3]; /* 2 decimal places */
    sprintf(bufferFull, "%u", full);
    sprintf(bufferDecimals, "%02u", decimals);

    debugPrint("HIDS temperature (float) = ");
    if (temperature < 0)
    {
      debugPrint("-");
    }
    debugPrint(bufferFull);
    debugPrint(".");
    debugPrint(bufferDecimals);
    debugPrintln(" degrees Celsius");
  }

#endif /* HIDS_EXAMPLE_ENABLE_FLOAT */


#ifdef HIDS_EXAMPLE_ENABLE_INT16

  /* Example using 16 bit integer values */
  uint16_t humidity_uint16 = 0;
  int16_t temperature_int16 = 0;

  if (HIDS_getHumidity_uint16(&hids, &humidity_uint16) == WE_SUCCESS)
  {
    uint16_t full = humidity_uint16 / 100;
    uint16_t decimals = humidity_uint16  % 100; /* 2 decimal places */

    char bufferFull[4]; /* 3 pre-decimal point positions (max. 100%) */
    char bufferDecimals[3]; /* 2 decimal places */
    sprintf(bufferFull, "%u", full);
    sprintf(bufferDecimals, "%02u", decimals);

    debugPrint("HIDS RH (uint16) = ");
    debugPrint(bufferFull);
    debugPrint(".");
    debugPrint(bufferDecimals);
    debugPrintln("%");
  }

  if (HIDS_getTemperature_int16(&hids, &temperature_int16) == WE_SUCCESS)
  {
    uint16_t full = ((uint16_t) abs(temperature_int16)) / 100;
    uint16_t decimals = (uint16_t) (abs(temperature_int16) % 100); /* 2 decimal places */

    char bufferFull[4]; /* 3 pre-decimal point positions (from -40 to 100 degrees Celsius) */
    char bufferDecimals[3]; /* 2 decimal places */
    sprintf(bufferFull, "%u", full);
    sprintf(bufferDecimals, "%02u", decimals);

    debugPrint("HIDS temperature (int16) = ");
    if (temperature_int16 < 0)
    {
      debugPrint("-");
    }
    debugPrint(bufferFull);
    debugPrint(".");
    debugPrint(bufferDecimals);
    debugPrintln(" degrees Celsius");
  }

#endif /* HIDS_EXAMPLE_ENABLE_INT16 */


#ifdef HIDS_EXAMPLE_ENABLE_INT8

  /* Example using 8 bit integer values */
  int8_t humidity_int8 = 0;
  int8_t temperature_int8 = 0;

  if (HIDS_getHumidity_int8(&hids, &humidity_int8) == WE_SUCCESS)
  {
    char buffer[4]; /* 3 pre-decimal point positions (from 0% to max 100% RH) */
    sprintf(buffer, "%d", humidity_int8);

    debugPrint("HIDS RH (int8) = ");
    debugPrint(buffer);
    debugPrintln("%");
  }

  if (HIDS_getTemperature_int8(&hids, &temperature_int8) == WE_SUCCESS)
  {
    char buffer[4]; /* 3 pre-decimal point positions (from -40 to +85 degrees Celsius) */
    sprintf(buffer, "%d", temperature_int8);

    debugPrint("HIDS temperature (int8) = ");
    debugPrint(buffer);
    debugPrintln(" degrees Celsius");
  }

#endif /* HIDS_EXAMPLE_ENABLE_INT8 */

  /* Wait 1s */
  HAL_Delay(1000);
}

/**
 * @brief Initializes the sensor for this example application.
 */
static bool HIDS_init(void)
{
  /* Initialize sensor interface (use i2c with HIDS address, burst mode activated) */
  HIDS_getDefaultInterface(&hids);
  hids.interfaceType = WE_i2c;
  hids.handle = &hi2c1;

  /* Wait for boot */
  HAL_Delay(50);
  while (WE_SUCCESS != WE_isSensorInterfaceReady(&hids))
  {
  }
  debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

  HAL_Delay(5);

  /* First communication test */
  uint8_t deviceIdValue = 0;
  if (WE_SUCCESS == HIDS_getDeviceID(&hids, &deviceIdValue))
  {
    if (deviceIdValue == HIDS_DEVICE_ID_VALUE) /* who am i ? - i am WSEN-HIDS! */
    {
      debugPrintln("**** HIDS_DEVICE_ID_VALUE: OK ****");
    }
    else
    {
      debugPrintln("**** HIDS_DEVICE_ID_VALUE: NOT OK ****");
      return false;
    }
  }
  else
  {
    debugPrintln("**** HIDS_getDeviceID(): NOT OK ****");
    return false;
  }

  /* Enables block data update, prevents that an update happens before both value registers were read */
  if (WE_SUCCESS != HIDS_enableBlockDataUpdate(&hids, HIDS_enable))
  {
    debugPrintln("**** HIDS_setBdu(true): NOT OK ****");
    return false;
  }

  /* Make sure the device is in ODR=oneshot mode '00' */
  if (WE_SUCCESS !=  HIDS_setOutputDataRate(&hids, HIDS_oneShot))
  {
    debugPrintln("**** HIDS_setOdr(oneShot): NOT OK ****");
    return false;
  }

  /* Make sure the device is in active mode '1' */
  if (WE_SUCCESS !=  HIDS_setPowerMode(&hids, HIDS_activeMode))
  {
    debugPrintln("**** HIDS_setPowerMode(active): NOT OK ****");
    return false;
  }

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

