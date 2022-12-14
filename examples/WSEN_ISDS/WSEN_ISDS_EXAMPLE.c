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
 * @brief WSEN_ISDS example.
 *
 * Demonstrates basic usage of the ISDS 6-axis accelerometer/gyroscope connected via I2C.
 */

#include "WSEN_ISDS_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include <platform.h>

#include "../SensorsSDK/WSEN_ISDS_2536030320001/WSEN_ISDS_2536030320001.h"

/* Comment/uncomment the following lines to disable/enable the examples for
 * each data type (see WE_isdsExampleLoop() function). */
/* Note: The float example won't compile unless WE_USE_FLOAT is defined. */
#define ISDS_EXAMPLE_ENABLE_FLOAT
#define ISDS_EXAMPLE_ENABLE_INT

/* Sensor interface configuration */
static WE_sensorInterface_t isds;

/* Sensor initialization function */
static bool ISDS_init(void);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);

#ifdef ISDS_EXAMPLE_ENABLE_FLOAT
static void debugPrintAcceleration_float(char axis[], float acc);
static void debugPrintAngularRate_float(char axis[], float rate);
static void debugPrintTemperature_float(float tempDegC);
#endif

#ifdef ISDS_EXAMPLE_ENABLE_INT
static void debugPrintAcceleration_int(char axis[], int32_t accMg);
static void debugPrintAngularRate_int(char axis[], int32_t rateMdps);
static void debugPrintTemperature_int(int16_t temperature);
#endif

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_isdsExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the example program showing basic usage of the ISDS sensor connected via I2C.");

  /* init ISDS */
  if (false == ISDS_init())
  {
    debugPrintln("**** ISDS_Init() error. STOP ****");
    HAL_Delay(5);
    while(1);
  }
}

/**
 * @brief Example main loop code.
 * Call this function in main loop (infinite loop).
 */
void WE_isdsExampleLoop()
{
  ISDS_state_t dataReady;

  /* Optional: Enable accelerometer reference mode (store the current X,Y,Z accelerometer
   * samples and subtract these from all subsequent output values) */
//  ISDS_enableHighPassFilterRefMode(&isds, ISDS_enable);
//  ISDS_enableAccHighPassSlopeFilter(&isds, ISDS_enable);
//  ISDS_setAccFilterConfig(&isds, ISDS_accFilterConfig_LowPassPath_ODRDiv9);
//  /* Discard at least one sample after enabling reference mode */
//  HAL_Delay(20);

  while (1)
  {
#ifdef ISDS_EXAMPLE_ENABLE_FLOAT
    {
      float xAcc = 0;
      float yAcc = 0;
      float zAcc = 0;
      float xRate = 0;
      float yRate = 0;
      float zRate = 0;
      float temperature = 0;

      /* Wait until the acceleration values are ready to read */
      do
      {
        ISDS_isAccelerationDataReady(&isds, &dataReady);
      } while (dataReady == ISDS_disable);

      /* Read acceleration values (alternatively use ISDS_getAccelerations_float() to get values for all three axes in one go) */
      if (ISDS_getAccelerationX_float(&isds, &xAcc) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAccelerationX_float(): NOT OK ****");
        xAcc = 0;
      }
      if (ISDS_getAccelerationY_float(&isds, &yAcc) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAccelerationY_float(): NOT OK ****");
        yAcc = 0;
      }
      if (ISDS_getAccelerationZ_float(&isds, &zAcc) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAccelerationZ_float(): NOT OK ****");
        zAcc = 0;
      }


      /* Wait until the gyroscope values are ready to read */
      do
      {
        ISDS_isGyroscopeDataReady(&isds, &dataReady);
      } while (dataReady == ISDS_disable);


      /* Read gyroscope values (alternatively use ISDS_getAngularRates()_float to get values for all three axes in one go) */
      if (ISDS_getAngularRateX_float(&isds, &xRate) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAngularRateX_float(): NOT OK ****");
        xRate = 0;
      }
      if (ISDS_getAngularRateY_float(&isds, &yRate) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAngularRateY_float(): NOT OK ****");
        yRate = 0;
      }
      if (ISDS_getAngularRateZ_float(&isds, &zRate) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAngularRateZ_float(): NOT OK ****");
        zRate = 0;
      }

      /* Wait until the temperature value is ready to read */
      do
      {
        ISDS_isTemperatureDataReady(&isds, &dataReady);
      } while (dataReady == ISDS_disable);
      if (ISDS_getTemperature_float(&isds, &temperature) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getTemperature_float(): NOT OK ****");
        temperature = 0;
      }

      /* Print values to debug UART */
      debugPrintAcceleration_float("X", xAcc);
      debugPrintAcceleration_float("Y", yAcc);
      debugPrintAcceleration_float("Z", zAcc);
      debugPrintAngularRate_float("X", xRate);
      debugPrintAngularRate_float("Y", yRate);
      debugPrintAngularRate_float("Z", zRate);
      debugPrintTemperature_float(temperature);
    }
#endif /* ISDS_EXAMPLE_ENABLE_FLOAT */


#ifdef ISDS_EXAMPLE_ENABLE_INT
    {
      int16_t xAcc = 0;
      int16_t yAcc = 0;
      int16_t zAcc = 0;
      int32_t xRate = 0;
      int32_t yRate = 0;
      int32_t zRate = 0;
      int16_t temperature = 0;

      /* Wait until the acceleration values are ready to read */
      do
      {
        ISDS_isAccelerationDataReady(&isds, &dataReady);
      } while (dataReady == ISDS_disable);

      /* Read acceleration values (alternatively use ISDS_getAccelerations_int() to get values for all three axes in one go) */
      if (ISDS_getAccelerationX_int(&isds, &xAcc) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAccelerationX_int(): NOT OK ****");
        xAcc = 0;
      }
      if (ISDS_getAccelerationY_int(&isds, &yAcc) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAccelerationY_int(): NOT OK ****");
        yAcc = 0;
      }
      if (ISDS_getAccelerationZ_int(&isds, &zAcc) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAccelerationZ_int(): NOT OK ****");
        zAcc = 0;
      }

      /* Wait until the gyroscope values are ready to read */
      do
      {
        ISDS_isGyroscopeDataReady(&isds, &dataReady);
      } while (dataReady == ISDS_disable);


      /* Read gyroscope values (alternatively use ISDS_getAngularRates_int() to get values for all three axes in one go) */
      if (ISDS_getAngularRateX_int(&isds, &xRate) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAngularRateX_int(): NOT OK ****");
        xRate = 0;
      }
      if (ISDS_getAngularRateY_int(&isds, &yRate) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAngularRateY_int(): NOT OK ****");
        yRate = 0;
      }
      if (ISDS_getAngularRateZ_int(&isds, &zRate) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getAngularRateZ_int(): NOT OK ****");
        zRate = 0;
      }

      /* Wait until the temperature value is ready to read */
      do
      {
        ISDS_isTemperatureDataReady(&isds, &dataReady);
      } while (dataReady == ISDS_disable);
      if (ISDS_getTemperature_int(&isds, &temperature) != WE_SUCCESS)
      {
        debugPrintln("**** ISDS_getTemperature_int(): NOT OK ****");
        temperature = 0;
      }

      /* Print values to debug UART */
      debugPrintAcceleration_int("X", xAcc);
      debugPrintAcceleration_int("Y", yAcc);
      debugPrintAcceleration_int("Z", zAcc);
      debugPrintAngularRate_int("X", xRate);
      debugPrintAngularRate_int("Y", yRate);
      debugPrintAngularRate_int("Z", zRate);
      debugPrintTemperature_int(temperature);
    }
#endif /* ISDS_EXAMPLE_ENABLE_INT */

    HAL_Delay(1000);
  }
}

/**
 * @brief Initializes the sensor for this example application.
 */
static bool ISDS_init(void)
{
  /* Initialize sensor interface (i2c with ISDS address, burst mode activated) */
  ISDS_getDefaultInterface(&isds);
  isds.interfaceType = WE_i2c;
  isds.options.i2c.burstMode = 1;
  isds.options.i2c.address = ISDS_ADDRESS_I2C_1;
  #warning "Please use correct i2c address here"
  isds.handle = &hi2c1;

  /* Wait for boot */
  HAL_Delay(50);
  while (WE_SUCCESS != WE_isSensorInterfaceReady(&isds))
  {
  }
  debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

  HAL_Delay(5);

  /* First communication test */
  uint8_t deviceIdValue = 0;
  if (WE_SUCCESS == ISDS_getDeviceID(&isds, &deviceIdValue))
  {
    if (deviceIdValue == ISDS_DEVICE_ID_VALUE) /* who am i ? - i am WSEN-ISDS! */
    {
      debugPrintln("**** ISDS_DEVICE_ID_VALUE: OK ****");
    }
    else
    {
      debugPrintln("**** ISDS_DEVICE_ID_VALUE: NOT OK ****");
      return false;
    }
  }
  else
  {
    debugPrintln("**** ISDS_getDeviceID(): NOT OK ****");
    return false;
  }

  /* Perform soft reset of the sensor */
  ISDS_softReset(&isds, ISDS_enable);
  ISDS_state_t swReset;
  do
  {
    ISDS_getSoftResetState(&isds, &swReset);
  } while (swReset);
  debugPrintln("**** ISDS reset complete ****");

  /* Perform reboot (retrieve trimming parameters from nonvolatile memory) */
  ISDS_reboot(&isds, ISDS_enable);
  HAL_Delay(15);
  debugPrintln("**** ISDS reboot complete ****");

  /* Enable block data update */
  ISDS_enableBlockDataUpdate(&isds, ISDS_enable);

  /* Sampling rate (104 Hz) */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr104Hz);
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr104Hz);

  /* Accelerometer 2g range */
  ISDS_setAccFullScale(&isds, ISDS_accFullScaleTwoG);

  /* Gyroscope 2000 dps range */
  ISDS_setGyroFullScale(&isds, ISDS_gyroFullScale2000dps);


  /* Optional: Disable accelerometer and/or gyroscope high performance mode.
   * If high performance mode is disabled, the ODR is used to switch between
   * power modes as follows:
   * 1.6 Hz - 52 Hz     Low power mode
   * 104 Hz - 208 Hz    Normal power mode
   * 416 Hz - 6.66 kHz  High performance mode */
//  ISDS_disableAccHighPerformanceMode(&isds, ISDS_enable);
//  ISDS_disableGyroHighPerformanceMode(&isds, ISDS_enable);


  /* Note that, depending on the configuration of the filtering chain of the
   * accelerometer and the gyroscope, it might be necessary to discard the
   * first samples after changing ODRs or when changing the power mode.
   * Consult the user manual for details on the number of samples to be discarded. */

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


#ifdef ISDS_EXAMPLE_ENABLE_FLOAT

/**
 * @brief Prints the acceleration for the supplied axis to the debug interface.
 * @param axis Axis name
 * @param acc  Acceleration [mg]
 */
static void debugPrintAcceleration_float(char axis[], float acc)
{
  acc /= 1000.0f;
  float accAbs = fabs(acc);
  uint16_t full = (uint16_t) accAbs;
  uint16_t decimals = (uint16_t) (((uint32_t) (accAbs * 10000)) % 10000); /* 4 decimal places */

  char bufferFull[4]; /* max 3 pre-decimal point positions */
  char bufferDecimals[5]; /* 4 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%04u", decimals);

  debugPrint("ISDS acceleration (float) ");
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

/**
 * @brief Prints the angular rate for the supplied axis to the debug interface.
 * @param axis Axis name
 * @param rate Angular rate [mdps]
 */
static void debugPrintAngularRate_float(char axis[], float rate)
{
  rate /= 1000.0f;
  float rateAbs = fabs(rate);
  uint16_t full = (uint16_t) rateAbs;
  uint16_t decimals = (uint16_t) (((uint32_t) (rateAbs * 10000)) % 10000); /* 4 decimal places */

  char bufferFull[4]; /* max 3 pre-decimal point positions */
  char bufferDecimals[5]; /* 4 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%04u", decimals);

  debugPrint("ISDS angular rate (float) ");
  debugPrint(axis);
  debugPrint(" = ");
  if (rate < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" dps");
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

  char bufferFull[4]; /* 3 pre-decimal point positions (from -40 to 125 degrees Celsius) */
  char bufferDecimals[3]; /* 2 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%02u", decimals);

  debugPrint("ISDS temperature (float) = ");
  if (tempDegC < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" degrees Celsius");
}

#endif /* ISDS_EXAMPLE_ENABLE_FLOAT */


#ifdef ISDS_EXAMPLE_ENABLE_INT

/**
 * @brief Prints the acceleration for the supplied axis to the debug interface.
 * @param axis Axis name
 * @param acc  Acceleration [mg]
 */
static void debugPrintAcceleration_int(char axis[], int32_t accMg)
{
  uint16_t full = ((uint16_t) labs(accMg)) / 1000;
  uint16_t decimals = (uint16_t) (labs(accMg) % 1000); /* 3 decimal places */

  char bufferFull[4]; /* max 3 pre-decimal point positions */
  char bufferDecimals[4]; /* 3 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%03u", decimals);

  debugPrint("ISDS acceleration (int) ");
  debugPrint(axis);
  debugPrint(" = ");
  if (accMg < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" g");
}

/**
 * @brief Prints the angular rate for the supplied axis to the debug interface.
 * @param axis Axis name
 * @param acc  Angular rate [mdps]
 */
static void debugPrintAngularRate_int(char axis[], int32_t rateMdps)
{
  uint16_t full = ((uint16_t) labs(rateMdps)) / 1000;
  uint16_t decimals = (uint16_t) (labs(rateMdps) % 1000); /* 3 decimal places */

  char bufferFull[4]; /* max 3 pre-decimal point positions */
  char bufferDecimals[4]; /* 3 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%03u", decimals);

  debugPrint("ISDS angular rate (int) ");
  debugPrint(axis);
  debugPrint(" = ");
  if (rateMdps < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" dps");
}

/**
 * @brief Prints the temperature to the debug interface.
 * @param temperature Temperature [°C x 100]
 */
static void debugPrintTemperature_int(int16_t temperature)
{
  uint16_t full = ((uint16_t) abs(temperature)) / 100;
  uint16_t decimals = (uint16_t) (abs(temperature) % 100); /* 2 decimal places */

  char bufferFull[4]; /* 3 pre-decimal point positions (from -40 to 125 degrees Celsius) */
  char bufferDecimals[3]; /* 2 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%02u", decimals);

  debugPrint("ISDS temperature (int) = ");
  if (temperature < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" degrees Celsius");
}

#endif /* ISDS_EXAMPLE_ENABLE_INT */
