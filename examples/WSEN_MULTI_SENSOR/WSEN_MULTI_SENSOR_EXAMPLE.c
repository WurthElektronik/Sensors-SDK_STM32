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
 * @brief WSEN multi sensor example.
 *
 * Example for using multiple sensors connected via I2C simultaneously (HIDS, ITDS and TIDS)
 */

#include "WSEN_MULTI_SENSOR_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include <platform.h>

#include "../SensorsSDK/WSEN_HIDS_2523020210001/WSEN_HIDS_2523020210001.h"
#include "../SensorsSDK/WSEN_ITDS_2533020201601/WSEN_ITDS_2533020201601.h"
#include "../SensorsSDK/WSEN_TIDS_2521020222501/WSEN_TIDS_2521020222501.h"

/* Comment/uncomment the following defines to exclude/include the examples for the corresponding sensors. */
#define MULTI_SENSOR_EXAMPLE_HIDS
#define MULTI_SENSOR_EXAMPLE_ITDS
#define MULTI_SENSOR_EXAMPLE_TIDS

/* Sensor interface configuration */
static WE_sensorInterface_t hids;
static WE_sensorInterface_t itds;
static WE_sensorInterface_t tids;

/* Sensor initialization functions */
static bool initSensors(void);
static bool HIDS_init(void);
static bool ITDS_init(void);
static bool TIDS_init(void);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);

#ifdef MULTI_SENSOR_EXAMPLE_ITDS
static void debugPrintAcceleration(char axis[], float acc);
#endif // MULTI_SENSOR_EXAMPLE_ITDS

#ifdef MULTI_SENSOR_EXAMPLE_TIDS
static void debugPrintTemperatureInt(int16_t temperature);
#endif // MULTI_SENSOR_EXAMPLE_TIDS

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_multiSensorExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the example program for using multiple sensors in parallel.");

  /* Initialize sensors (depending on the defines at the top of this file). */
  initSensors();

  /* LED on */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  HAL_Delay(5);
}

/**
 * @brief Example main loop code.
 * Call this function in main loop (infinite loop).
 */
void WE_multiSensorExampleLoop()
{
#ifdef MULTI_SENSOR_EXAMPLE_HIDS
  if (WE_FAIL == HIDS_enableOneShot(&hids, HIDS_enable)) /* trigger a single measurement - oneshot it is! */
  {
    debugPrintln("**** HIDS_enOneShot(enable): NOT OK ****");
  }

  HAL_Delay(1);

  bool waitForMeasurement = true;
  while (waitForMeasurement == true)
  {
    HIDS_state_t humidityAvailable = HIDS_disable;
    if (WE_FAIL == HIDS_isHumidityDataAvailable(&hids, &humidityAvailable))
    {
      debugPrintln("**** HIDS_getHumStatus(): NOT OK ****");
    }

    HIDS_state_t oneShotStatus = HIDS_enable;
    if (WE_FAIL == HIDS_isOneShotEnabled(&hids, &oneShotStatus))
    {
      debugPrintln("**** HIDS_getOneShotState(): NOT OK ****");
    }

    /* Wait until the humidity status bit is '1' and the oneshot bit was reset to '0' => indicating measurement was done */
    if ((humidityAvailable == HIDS_enable)
          && (oneShotStatus == HIDS_disable))
    {
      waitForMeasurement = false;
    }
    else
    {
      HAL_Delay(1);
    }
  }

  uint16_t humidity_uint16 = 0;
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
#endif // MULTI_SENSOR_EXAMPLE_HIDS


#ifdef MULTI_SENSOR_EXAMPLE_ITDS
  ITDS_state_t dataReady = ITDS_disable;
  float xAcceleration = 0.0f;
  float yAcceleration = 0.0f;
  float zAcceleration = 0.0f;

  do
  {
    /* Wait until the value is ready to read */
    ITDS_isAccelerationDataReady(&itds, &dataReady);
  } while (dataReady == ITDS_disable);

  if (ITDS_getAccelerations_float(&itds, 1, &xAcceleration, &yAcceleration, &zAcceleration) == WE_SUCCESS)
  {
    debugPrintAcceleration("X", xAcceleration);
    debugPrintAcceleration("Y", yAcceleration);
    debugPrintAcceleration("Z", zAcceleration);
  }
  else
  {
    debugPrintln("**** ITDS_getAccelerations_float(): NOT OK ****");
  }
#endif // MULTI_SENSOR_EXAMPLE_ITDS



#ifdef MULTI_SENSOR_EXAMPLE_TIDS
  /* Start a conversion (one shot) */
  TIDS_enableOneShot(&tids, TIDS_enable);

  /* Wait until the busy bit is set to 0 */
  TIDS_state_t tidsBusy = TIDS_disable;
  do
  {
    TIDS_isBusy(&tids, &tidsBusy);
  }
  while (tidsBusy == TIDS_enable);

  int16_t temperatureInt;
  if (TIDS_getRawTemperature(&tids, &temperatureInt) == WE_SUCCESS)
  {
    debugPrintTemperatureInt(temperatureInt);
  }
  else
  {
    debugPrintln("**** TIDS_getRawTemperature(): NOT OK ****");
  }

  /* Perform software reset - must be done when using ONE_SHOT bit
   * (i.e. single conversion mode) */
  TIDS_softReset(&tids, TIDS_enable);
  HAL_Delay(5);
  TIDS_softReset(&tids, TIDS_disable);
#endif


  /* Wait 1s */
  HAL_Delay(1000);
}

static bool initSensors(void)
{
#ifdef MULTI_SENSOR_EXAMPLE_HIDS
  /* init HIDS */
  debugPrintln("**** Initializing HIDS sensor...");
  if (false == HIDS_init())
  {
    debugPrintln("**** HIDS_Init() error. STOP ****");
    HAL_Delay(5);
    while(1);
  }
#endif // MULTI_SENSOR_EXAMPLE_HIDS

#ifdef MULTI_SENSOR_EXAMPLE_ITDS
  /* init ITDS */
  debugPrintln("**** Initializing ITDS sensor...");
  if (false == ITDS_init())
  {
    debugPrintln("**** ITDS_Init() error. STOP ****");
    HAL_Delay(5);
    while(1);
  }
#endif // MULTI_SENSOR_EXAMPLE_ITDS

#ifdef MULTI_SENSOR_EXAMPLE_TIDS
  /* init TIDS */
  debugPrintln("**** Initializing TIDS sensor...");
  if (false == TIDS_init())
  {
    debugPrintln("**** TIDS_Init() error. STOP ****");
    HAL_Delay(5);
    while(1);
  }
#endif // MULTI_SENSOR_EXAMPLE_TIDS

  return true;
}

/**
 * @brief Initializes the HIDS sensor for this example application.
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

  /* Enables block data update (prevents an update from happening before both value registers were read) */
  if (WE_SUCCESS != HIDS_enableBlockDataUpdate(&hids, HIDS_enable))
  {
    debugPrintln("**** HIDS_setBdu(true): NOT OK ****");
    return false;
  }

  if (WE_SUCCESS !=  HIDS_setOutputDataRate(&hids, HIDS_oneShot)) /* Make sure the device is in ODR=oneshot mode '00'*/
  {
    debugPrintln("**** HIDS_setOdr(oneShot): NOT OK ****");
    return false;
  }

  if (WE_SUCCESS !=  HIDS_setPowerMode(&hids, HIDS_activeMode)) /* Make sure the device is in active mode*/
  {
    debugPrintln("**** HIDS_setPowerMode(active): NOT OK ****");
    return false;
  }

  return true;
}

/**
 * @brief Initializes the ITDS sensor for this example application.
 */
static bool ITDS_init(void)
{
  /* Initialize sensor interface (i2c with ITDS address, burst mode activated) */
  ITDS_getDefaultInterface(&itds);
  itds.interfaceType = WE_i2c;
  itds.options.i2c.burstMode = 1;
  itds.handle = &hi2c1;

  /* Wait for boot */
  HAL_Delay(50);
  while (WE_SUCCESS != WE_isSensorInterfaceReady(&itds))
  {
  }
  debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

  HAL_Delay(5);

  /* First communication test */
  uint8_t deviceIdValue = 0;
  if (WE_SUCCESS == ITDS_getDeviceID(&itds, &deviceIdValue))
  {
    if (deviceIdValue == ITDS_DEVICE_ID_VALUE) /* who am i ? - i am WSEN-ITDS! */
    {
      debugPrintln("**** ITDS_DEVICE_ID_VALUE: OK ****");
    }
    else
    {
      debugPrintln("**** ITDS_DEVICE_ID_VALUE: NOT OK ****");
      return false;
    }
  }
  else
  {
    debugPrintln("**** ITDS_getDeviceID(): NOT OK ****");
    return false;
  }

  /* Perform soft reset of the sensor */
  ITDS_softReset(&itds, ITDS_enable);
  ITDS_state_t swReset;
  do
  {
    ITDS_getSoftResetState(&itds, &swReset);
  } while (swReset);
  debugPrintln("**** ITDS reset complete ****");

  /* Perform reboot (retrieve trimming parameters from nonvolatile memory) */
  ITDS_reboot(&itds, ITDS_enable);
  ITDS_state_t boot;
  do
  {
    ITDS_isRebooting(&itds, &boot);
  } while (boot);
  debugPrintln("**** ITDS reboot complete ****");

  /* Enable high performance mode */
  ITDS_setOperatingMode(&itds, ITDS_highPerformance);
  /* Sampling rate of 200 Hz */
  ITDS_setOutputDataRate(&itds, ITDS_odr6);
  /* Enable block data update */
  ITDS_enableBlockDataUpdate(&itds, ITDS_enable);
  /* Enable address auto increment */
  ITDS_enableAutoIncrement(&itds, ITDS_enable);
  /* Filter bandwidth = ODR/2 */
  ITDS_setFilteringCutoff(&itds, ITDS_outputDataRate_2);
  /* Full scale +-16g */
  ITDS_setFullScale(&itds, ITDS_sixteenG);

  return true;
}

/**
 * @brief Initializes the TIDS sensor for this example application.
 */
static bool TIDS_init(void)
{
  /* Initialize sensor interface (i2c with TIDS address, burst mode deactivated) */
  TIDS_getDefaultInterface(&tids);
  tids.interfaceType = WE_i2c;
  tids.handle = &hi2c1;

  /* Wait for boot */
  HAL_Delay(50);
  while (WE_SUCCESS != WE_isSensorInterfaceReady(&tids))
  {
  }
  debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

  HAL_Delay(5);

  /* First communication test */
  uint8_t deviceIdValue = 0;
  if (WE_SUCCESS == TIDS_getDeviceID(&tids, &deviceIdValue))
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
  TIDS_softReset(&tids, TIDS_enable);
  HAL_Delay(5);
  TIDS_softReset(&tids, TIDS_disable);

  /* Enable auto address increment */
  TIDS_enableAutoIncrement(&tids, TIDS_enable);

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

#ifdef MULTI_SENSOR_EXAMPLE_ITDS
static void debugPrintAcceleration(char axis[], float acc)
{
  acc /= 1000.0f;
  float accAbs = fabs(acc);
  uint16_t full = (uint16_t) accAbs;
  uint16_t decimals = (uint16_t) (((uint32_t) (accAbs * 10000)) % 10000); /* 4 decimal places */

  char bufferFull[4]; /* max 3 pre-decimal point positions */
  char bufferDecimals[5]; /* 4 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%04u", decimals);

  debugPrint("ITDS acceleration (float) ");
  debugPrint(axis);
  debugPrint(" = ");
  if (acc < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" g");
}
#endif // MULTI_SENSOR_EXAMPLE_ITDS

#ifdef MULTI_SENSOR_EXAMPLE_TIDS
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
#endif // MULTI_SENSOR_EXAMPLE_TIDS
