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
 * @brief WSEN-PADS example.
 *
 * Demonstrates basic usage of the PADS absolute pressure sensor connected via I2C.
 */

#include "WSEN_PADS_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include <platform.h>

#include "../SensorsSDK/WSEN_PADS_2511020213301/WSEN_PADS_2511020213301.h"

/* Comment/uncomment the following lines to disable/enable the examples for
 * each data type (see WE_padsExampleLoop() function). */
/* Note: The float example won't compile unless WE_USE_FLOAT is defined. */
#define PADS_EXAMPLE_ENABLE_FLOAT
#define PADS_EXAMPLE_ENABLE_INT

/* Sensor interface configuration */
static WE_sensorInterface_t pads;

/* Sensor initialization function */
static bool PADS_init(void);

/* Example modes for the PADS sensor */
void PADS_singleConversionModeExample(void);
void PADS_continuousModeExample(void);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);

#ifdef PADS_EXAMPLE_ENABLE_FLOAT
static void debugPrintPressure_float(float pressureKPa);
static void debugPrintTemperature_float(float temperature);
#endif

#ifdef PADS_EXAMPLE_ENABLE_INT
static void debugPrintPressure_int(int32_t pressure);
static void debugPrintTemperature_int(int16_t temperature);
#endif

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_padsExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the example program for the PADS sensor.");

  /* init PADS */
  if (false == PADS_init())
  {
    debugPrintln("**** PADS_Init() error. STOP ****");
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
void WE_padsExampleLoop()
{
  /* Comment/uncomment the following lines to switch between example modes. */
  PADS_singleConversionModeExample();
//  PADS_continuousModeExample();
}

/**
 * @brief Initializes the sensor for this example application.
 */
static bool PADS_init(void)
{
  /* Initialize sensor interface (i2c with PADS address, burst mode deactivated) */
  PADS_getDefaultInterface(&pads);
  pads.interfaceType = WE_i2c;
  pads.handle = &hi2c1;

  /* Wait for boot */
  HAL_Delay(50);
  while (WE_SUCCESS != WE_isSensorInterfaceReady(&pads))
  {
  }
  debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

  HAL_Delay(5);

  /* First communication test */
  uint8_t deviceIdValue = 0;
  if (WE_SUCCESS == PADS_getDeviceID(&pads, &deviceIdValue))
  {
    if (deviceIdValue == PADS_DEVICE_ID_VALUE) /* who am i ? - i am WSEN-PADS! */
    {
      debugPrintln("**** PADS_DEVICE_ID_VALUE: OK ****");
    }
    else
    {
      debugPrintln("**** PADS_DEVICE_ID_VALUE: NOT OK ****");
      return false;
    }
  }
  else
  {
    debugPrintln("**** PADS_getDeviceID(): NOT OK ****");
    return false;
  }

  /* Perform soft reset of the sensor */
  PADS_softReset(&pads, PADS_enable);
  PADS_state_t swReset;
  do
  {
    PADS_getSoftResetState(&pads, &swReset);
  } while (swReset);
  debugPrintln("**** PADS reset complete ****");

  return true;
}

/**
 * @brief Setup the sensor in single conversion mode.
 * Read and print the measured values at a rate of 1 Hz.
 * @param  no parameter.
 * @retval none
 */
void PADS_singleConversionModeExample()
{
  debugPrintln("Starting single conversion mode...");

  /* Automatic increment register address */
  PADS_enableAutoIncrement(&pads, PADS_enable);

  /* Enable block data update */
  PADS_enableBlockDataUpdate(&pads, PADS_enable);

  while (1)
  {
    /* Start a conversion (one shot) */
    PADS_enableOneShot(&pads, PADS_enable);

    /* Wait until the value is ready to read */
    PADS_state_t presStatus;
    do
    {
      PADS_isPressureDataAvailable(&pads, &presStatus);
    } while (presStatus != PADS_enable);

    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);


#ifdef PADS_EXAMPLE_ENABLE_FLOAT
    float pressureFloat = 0;
    if (WE_SUCCESS == PADS_getPressure_float(&pads, &pressureFloat))
    {
      debugPrintPressure_float(pressureFloat);
    }
    else
    {
      debugPrintln("**** PADS_getPressure_float(): NOT OK ****");
    }

    float temperatureFloat = 0;
    if (WE_SUCCESS == PADS_getTemperature_float(&pads, &temperatureFloat))
    {
      debugPrintTemperature_float(temperatureFloat);
    }
    else
    {
      debugPrintln("**** PADS_getTemperature_float(): NOT OK ****");
    }
#endif /* PADS_EXAMPLE_ENABLE_FLOAT */

#ifdef PADS_EXAMPLE_ENABLE_INT
    int32_t pressureInt = 0;
    if (WE_SUCCESS == PADS_getPressure_int(&pads, &pressureInt))
    {
      debugPrintPressure_int(pressureInt);
    }
    else
    {
      debugPrintln("**** PADS_getPressure_int(): NOT OK ****");
    }

    int16_t tempInt = 0;
    if (WE_SUCCESS == PADS_getTemperature_int(&pads, &tempInt))
    {
      debugPrintTemperature_int(tempInt);
    }
    else
    {
      debugPrintln("**** PADS_getTemperature_int(): NOT OK ****");
    }
#endif /* PADS_EXAMPLE_ENABLE_INT */

    /* Wait 1s */
    HAL_Delay(1000);
  }
}

/**
 * @brief Setup the sensor in continuous mode.
 * Read values at rate of 50 Hz and print at a rate of 1 Hz.
 * @param  no parameter.
 * @retval none
 */
void PADS_continuousModeExample(void)
{
  debugPrintln("Starting continuous mode...");

  /* Enable low-noise configuration */
  PADS_setPowerMode(&pads, PADS_lowNoise);

  /* Automatic increment register address */
  PADS_enableAutoIncrement(&pads, PADS_enable);

  /* Enable additional low pass filter */
  PADS_enableLowPassFilter(&pads, PADS_enable);

  /* Set filter bandwidth of ODR/20 */
  PADS_setLowPassFilterConfig(&pads, PADS_lpFilterBW2);

  /* Enable block data update */
  PADS_enableBlockDataUpdate(&pads, PADS_enable);

  /* Enable continuous operation with an update rate of 50 Hz */
  PADS_setOutputDataRate(&pads, PADS_outputDataRate50Hz);


  uint16_t counter = 0;
  while (1)
  {
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

    /* Print at 1 Hz */
    counter++;
    bool printNow = counter >= 50;
    if (printNow)
    {
      counter = 0;
    }

#ifdef PADS_EXAMPLE_ENABLE_FLOAT
    float pressureFloat = 0;
    if (WE_SUCCESS == PADS_getPressure_float(&pads, &pressureFloat))
    {
      if (printNow)
      {
        debugPrintPressure_float(pressureFloat);
      }
    }
    else
    {
      debugPrintln("**** PADS_getPressure_float(): NOT OK ****");
    }

    float temperatureFloat = 0;
    if (WE_SUCCESS == PADS_getTemperature_float(&pads, &temperatureFloat))
    {
      if (printNow)
      {
        debugPrintTemperature_float(temperatureFloat);
      }
    }
    else
    {
      debugPrintln("**** PADS_getTemperature_float(): NOT OK ****");
    }
#endif /* PADS_EXAMPLE_ENABLE_FLOAT */

#ifdef PADS_EXAMPLE_ENABLE_INT
    int32_t pressureInt = 0;
    if (WE_SUCCESS == PADS_getPressure_int(&pads, &pressureInt))
    {
      if (printNow)
      {
        debugPrintPressure_int(pressureInt);
      }
    }
    else
    {
      debugPrintln("**** PADS_getPressure_int(): NOT OK ****");
    }

    int16_t tempInt = 0;
    if (WE_SUCCESS == PADS_getTemperature_int(&pads, &tempInt))
    {
      if (printNow)
      {
        debugPrintTemperature_int(tempInt);
      }
    }
    else
    {
      debugPrintln("**** PADS_getTemperature_int(): NOT OK ****");
    }
#endif /* PADS_EXAMPLE_ENABLE_INT */

    /* Wait 1/ODR */
    HAL_Delay(1000 / 50);
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

#ifdef PADS_EXAMPLE_ENABLE_FLOAT

/**
 * @brief Prints the pressure to the debug interface.
 * @param pressureKPa Pressure [kPa]
 */
static void debugPrintPressure_float(float pressureKPa)
{
  float pressureAbs = fabs(pressureKPa);
  uint16_t full = (uint16_t) pressureAbs;
  uint16_t decimals = (uint16_t) (((uint32_t) (pressureAbs * 10000)) % 10000); /* 4 decimal places */

  char bufferFull[4]; /* max 3 pre-decimal point positions (26 to 126 kPa) */
  char bufferDecimals[5]; /* 4 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%04u", decimals);

  debugPrint("PADS pressure (float) = ");
  if (pressureKPa < 0)
  {
    debugPrint("-");
  }
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

  char bufferFull[4]; /* 3 pre-decimal point positions (from -40 to 85 degrees Celsius) */
  char bufferDecimals[3]; /* 2 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%02u", decimals);

  debugPrint("PADS temperature (float) = ");
  if (tempDegC < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" degrees Celsius");
}

#endif /* PADS_EXAMPLE_ENABLE_FLOAT */

#ifdef PADS_EXAMPLE_ENABLE_INT

/**
 * @brief Prints the pressure to the debug interface.
 * @param pressurePa Pressure [Pa]
 */
static void debugPrintPressure_int(int32_t pressurePa)
{
  uint16_t full = (uint16_t) (abs(pressurePa) / 1000);
  uint16_t decimals = (uint16_t) (abs(pressurePa) % 1000); /* 3 decimal places */

  char bufferFull[4]; /* max 3 pre-decimal point positions */
  char bufferDecimals[4]; /* 3 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%03u", decimals);

  debugPrint("PADS pressure (int) = ");
  if (pressurePa < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" kPa");
}

/**
 * @brief Prints the temperature to the debug interface.
 * @param temperature Temperature [°C x 100]
 */
static void debugPrintTemperature_int(int16_t temperature)
{
  uint16_t full = ((uint16_t) abs(temperature)) / 100;
  uint16_t decimals = (uint16_t) (abs(temperature) % 100); /* 2 decimal places */

  char bufferFull[4]; /* 3 pre-decimal point positions (from -40 to 85 degrees Celsius) */
  char bufferDecimals[3]; /* 2 decimal places */
  sprintf(bufferFull, "%u", full);
  sprintf(bufferDecimals, "%02u", decimals);

  debugPrint("PADS temperature (int) = ");
  if (temperature < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" degrees Celsius");
}

#endif /* PADS_EXAMPLE_ENABLE_INT */
