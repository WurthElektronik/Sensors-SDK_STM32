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
 * @brief WSEN_ITDS SPI example.
 *
 * Demonstrates basic usage of the ITDS accelerometer connected via SPI.
 */

#include "WSEN_ITDS_SPI_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "../SensorsSDK/WSEN_ITDS_2533020201601/WSEN_ITDS_2533020201601.h"

/* Sensor interface configuration */
static WE_sensorInterface_t itds;

/* Sensor initialization function */
static bool ITDS_init(void);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);
static void debugPrintAcceleration_int(char axis[], int32_t accMg);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_itdsSpiExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the example program showing basic usage of the ITDS sensor connected via SPI.");
  debugPrintln("Note that for this example to work, the following pin configuration is required:");
  debugPrintln("* PA0 is used as chip select output (connected to CS input of sensor)");

  /* init ITDS */
  if (false == ITDS_init())
  {
    debugPrintln("**** ITDS_Init() error. STOP ****");
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
void WE_itdsSpiExampleLoop()
{
  /* This example puts the sensor in high performance mode and polls data every second.
   * See the WSEN_ITDS i2c example for basic usage of the sensor and the various WSEN_ITDS_*
   * examples for specific usage scenarios. */

  /* Wait until the value is ready to read */
  ITDS_state_t dataReady = ITDS_disable;
  do
  {
    ITDS_isAccelerationDataReady(&itds, &dataReady);
  } while (dataReady == ITDS_disable);

  /* Retrieve and print acceleration data.
   * Here, the acceleration values of all axes in [mg] are read in one go.
   * Note that as an alternative, there are also functions to get the values for single
   * axes or to get the raw, unconverted values. */

  int16_t xAcc, yAcc, zAcc;
  if (ITDS_getAccelerations_int(&itds, 1, &xAcc, &yAcc, &zAcc) == WE_SUCCESS)
  {
    debugPrintAcceleration_int("X", xAcc);
    debugPrintAcceleration_int("Y", yAcc);
    debugPrintAcceleration_int("Z", zAcc);
  }
  else
  {
    debugPrintln("**** ITDS_getAccelerations_int(): NOT OK ****");
  }

  /* Wait 1s */
  HAL_Delay(1000);
}

/**
 * @brief Initializes the sensor for this example application.
 */
static bool ITDS_init(void)
{
  /* Initialize sensor interface (SPI, burst mode activated) */
  ITDS_getDefaultInterface(&itds);
  itds.interfaceType = WE_spi;
  itds.options.spi.chipSelectPort = SPI1_CS0_GPIO_Port;
  itds.options.spi.chipSelectPin = SPI1_CS0_Pin;
  itds.options.spi.burstMode = 1;
  itds.handle = &hspi1;

  /* Wait for boot */
  HAL_Delay(50);

  /* SPI chip select output must be initially high (because it is active low) */
  HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET);
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

  debugPrint("ITDS acceleration (int) ");
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
