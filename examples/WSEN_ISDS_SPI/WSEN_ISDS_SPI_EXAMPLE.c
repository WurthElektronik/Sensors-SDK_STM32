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
 * @brief WSEN_ISDS SPI example.
 *
 * Demonstrates basic usage of the ISDS accelerometer/gyroscope connected via SPI.
 */

#include "WSEN_ISDS_SPI_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "../SensorsSDK/WSEN_ISDS_2536030320001/WSEN_ISDS_2536030320001.h"

/* Sensor interface configuration */
static WE_sensorInterface_t isds;

/* Sensor initialization function */
static bool ISDS_init(void);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);
static void debugPrintAcceleration_int(char axis[], int32_t accMg);
static void debugPrintAngularRate_int(char axis[], int32_t rateMdps);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_isdsSpiExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the example program showing basic usage of the ISDS sensor connected via SPI.");
  debugPrintln("Note that for this example to work, the following pin configuration is required:");
  debugPrintln("* PA0 is used as chip select output (connected to CS input of sensor)");

  /* init ISDS */
  if (false == ISDS_init())
  {
    debugPrintln("**** ISDS_Init() error. STOP ****");
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
void WE_isdsSpiExampleLoop()
{
  /* This example puts the sensor in high performance mode and polls data every second.
   * See the WSEN_ISDS i2c example for basic usage of the sensor and the various WSEN_ISDS_*
   * examples for specific usage scenarios. */

  ISDS_state_t dataReady;
  int16_t xAcc = 0;
  int16_t yAcc = 0;
  int16_t zAcc = 0;
  int32_t xRate = 0;
  int32_t yRate = 0;
  int32_t zRate = 0;

  /* Wait until the acceleration values are ready to read */
  do
  {
    ISDS_isAccelerationDataReady(&isds, &dataReady);
  } while (dataReady == ISDS_disable);

  /* Read acceleration values */
  if (ISDS_getAccelerations_int(&isds, &xAcc, &yAcc, &zAcc) != WE_SUCCESS)
  {
    debugPrintln("**** ISDS_getAccelerations_int(): NOT OK ****");
    xAcc = 0;
    yAcc = 0;
    zAcc = 0;
  }

  /* Wait until the gyroscope values are ready to read */
  do
  {
    ISDS_isGyroscopeDataReady(&isds, &dataReady);
  } while (dataReady == ISDS_disable);


  /* Read gyroscope values */
  if (ISDS_getAngularRates_int(&isds, &xRate, &yRate, &zRate) != WE_SUCCESS)
  {
    debugPrintln("**** ISDS_getAngularRates_int(): NOT OK ****");
    xRate = 0;
    yRate = 0;
    zRate = 0;
  }

  /* Print values to debug UART */
  debugPrintAcceleration_int("X", xAcc);
  debugPrintAcceleration_int("Y", yAcc);
  debugPrintAcceleration_int("Z", zAcc);
  debugPrintAngularRate_int("X", xRate);
  debugPrintAngularRate_int("Y", yRate);
  debugPrintAngularRate_int("Z", zRate);

  /* Wait 1s */
  HAL_Delay(1000);
}

/**
 * @brief Initializes the sensor for this example application.
 */
static bool ISDS_init(void)
{
  /* Initialize sensor interface (SPI, burst mode activated) */
  ISDS_getDefaultInterface(&isds);
  isds.interfaceType = WE_spi;
  isds.options.spi.chipSelectPort = SPI1_CS0_GPIO_Port;
  isds.options.spi.chipSelectPin = SPI1_CS0_Pin;
  isds.options.spi.burstMode = 1;
  isds.handle = &hspi1;

  /* Wait for boot */
  HAL_Delay(50);

  /* SPI chip select output must be initially high (because it is active low) */
  HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET);
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
