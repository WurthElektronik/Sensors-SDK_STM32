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
 * @brief WSEN_HIDS SPI example.
 *
 * Demonstrates basic usage of the HIDS humidity sensor connected via SPI.
 */

#include "WSEN_HIDS_SPI_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "../SensorsSDK/WSEN_HIDS_2523020210001/WSEN_HIDS_2523020210001.h"

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
void WE_hidsSpiExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the example program for the HIDS sensor connected via SPI.");
  debugPrintln("Note that for this example to work, the following pin configuration is required:");
  debugPrintln("* PA0 is used as chip select output (connected to CS input of sensor)");
  debugPrintln("* SPI MISO is directly connected to SDA pin of sensor");
  debugPrintln("* SPI MOSI is connected to SDA pin of sensor through a 1K resistor");

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
void WE_hidsSpiExampleLoop()
{
  if (WE_FAIL == HIDS_enableOneShot(&hids, HIDS_enable)) /* trigger a single measurement - oneshot it is! */
  {
    debugPrintln("**** HIDS_enableOneShot(enable): NOT OK ****");
  }

  HAL_Delay(1);

  bool waitForMeasurement = true;
  while (waitForMeasurement == true)
  {
    HIDS_state_t humStatus = HIDS_disable;
    if (WE_FAIL == HIDS_isHumidityDataAvailable(&hids, &humStatus))
    {
      debugPrintln("**** HIDS_isHumidityDataAvailable(): NOT OK ****");
    }

    HIDS_state_t oneShotStatus = HIDS_enable;
    if (WE_FAIL == HIDS_isOneShotEnabled(&hids, &oneShotStatus))
    {
      debugPrintln("**** HIDS_isOneShotEnabled(): NOT OK ****");
    }

    /* Wait until the humidity status bit is '1' and the oneshot bit was reset to '0' => indicating measurement was done */
    if ((humStatus == HIDS_enable)
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

  /* Wait 1s */
  HAL_Delay(1000);
}

/**
 * @brief Initializes the sensor for this example application.
 */
static bool HIDS_init(void)
{
  /* Initialize sensor interface (SPI, burst mode deactivated) */
  HIDS_getDefaultInterface(&hids);
  hids.interfaceType = WE_spi;
  hids.options.spi.chipSelectPort = SPI1_CS0_GPIO_Port;
  hids.options.spi.chipSelectPin = SPI1_CS0_Pin;
  hids.options.spi.burstMode = 0;
  hids.handle = &hspi1;

  /* Wait for boot */
  HAL_Delay(50);

  /* SPI chip select output must be initially high (because it is active low) */
  HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET);
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
