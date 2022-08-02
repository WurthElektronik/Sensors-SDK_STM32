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
 * @brief WSEN_ITDS example.
 *
 * Demonstrates basic usage of the ITDS accelerometer connected via I2C.
 */

#include "WSEN_ITDS_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gpio.h"
#include "i2c.h"
#include "usart.h"

#include <platform.h>

#include "../SensorsSDK/WSEN_ITDS_2533020201601/WSEN_ITDS_2533020201601.h"


/* Comment/uncomment the following lines to disable/enable the examples for
 * each data type (see WE_itdsExampleLoop() function). */
/* Note: The float example won't compile unless WE_USE_FLOAT is defined. */
#define ITDS_EXAMPLE_ENABLE_FLOAT
#define ITDS_EXAMPLE_ENABLE_INT


/* Enum used to switch between different example modes (see variable itdsExampleMode) */
typedef enum
{
  highPerformanceExample,
  normalExample,
  lowPowerExample,
  temperatureExample
} ITDS_example_mode;

/* Use the following variable to switch among the available example modes. */
static ITDS_example_mode itdsExampleMode = highPerformanceExample;

/* Sensor interface configuration */
static WE_sensorInterface_t itds;

/* Sensor initialization function */
static bool ITDS_init(void);

/* Example modes for the ITDS sensor */
void ITDS_startHighPerformanceMode();
void ITDS_startNormalMode();
void ITDS_startLowPowerMode();
void ITDS_startTemperatureMode();

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);

#ifdef ITDS_EXAMPLE_ENABLE_FLOAT
static void debugPrintAcceleration_float(char axis[], float acc);
static void debugPrintTemperature_float(float temperature);
#endif

#ifdef ITDS_EXAMPLE_ENABLE_INT
static void debugPrintAcceleration_int(char axis[], int32_t accMg);
static void debugPrintTemperature_int(int16_t temperature);
#endif

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_itdsExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the example program showing basic usage of the ITDS sensor connected via I2C.");

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
void WE_itdsExampleLoop()
{
  /* Run example depending on the value of itdsExampleMode */
  switch (itdsExampleMode)
  {
  case highPerformanceExample:
    ITDS_startHighPerformanceMode();
    break;

  case normalExample:
    ITDS_startNormalMode();
    break;

  case lowPowerExample:
    ITDS_startLowPowerMode();
    break;

  case temperatureExample:
    ITDS_startTemperatureMode();
    break;
  }
}

/**
 * @brief Initializes the sensor for this example application.
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

  return true;
}

/**
 * @brief  High performance mode
 * @param  No parameter.
 * @retval None
 */
void ITDS_startHighPerformanceMode()
{
  debugPrintln("Starting high performance mode...");

  ITDS_state_t dataReady = ITDS_disable;

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

  while (1)
  {
    /* Wait until the value is ready to read */
    do
    {
      ITDS_isAccelerationDataReady(&itds, &dataReady);
    } while (dataReady == ITDS_disable);

    /* Below, you'll find examples for reading acceleration values of all axes in [mg],
     * either as float or as integer. Note that as an alternative, there are also
     * functions to get the values for single axes or to get the raw, unconverted values. */

#ifdef ITDS_EXAMPLE_ENABLE_FLOAT
    {
      float xAcc, yAcc, zAcc;
      if (ITDS_getAccelerations_float(&itds, 1, &xAcc, &yAcc, &zAcc) == WE_SUCCESS)
      {
        debugPrintAcceleration_float("X", xAcc);
        debugPrintAcceleration_float("Y", yAcc);
        debugPrintAcceleration_float("Z", zAcc);

        float accSum = sqrtf((xAcc * xAcc) +
                (yAcc * yAcc) +
                (zAcc * zAcc));
        debugPrintAcceleration_float("sum", accSum);
      }
      else
      {
        debugPrintln("**** ITDS_getAccelerations_float(): NOT OK ****");
      }
    }

#endif

#ifdef ITDS_EXAMPLE_ENABLE_INT
    {
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
    }

#endif // ITDS_EXAMPLE_ENABLE_INT

    /* Wait 1s */
    HAL_Delay(1000);
  }
}

/**
 * @brief  Normal mode
 * @param  No parameter.
 * @retval None
 */
void ITDS_startNormalMode()
{
  debugPrintln("Starting normal mode...");

  ITDS_state_t dataReady = ITDS_disable;

  /* Enable normal mode*/
  ITDS_setOperatingMode(&itds, ITDS_normalOrLowPower);
  ITDS_setPowerMode(&itds, ITDS_normalMode);
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

  while (1)
  {
    /* Wait until the value is ready to read */
    do
    {
      ITDS_isAccelerationDataReady(&itds, &dataReady);
    } while (dataReady == ITDS_disable);

    /* Below, you'll find examples for reading acceleration values of all axes in [mg],
     * either as float or as integer. Note that as an alternative, there are also
     * functions to get the values for single axes or to get the raw, unconverted values. */

#ifdef ITDS_EXAMPLE_ENABLE_FLOAT
    {
      float xAcc, yAcc, zAcc;
      if (ITDS_getAccelerations_float(&itds, 1, &xAcc, &yAcc, &zAcc) == WE_SUCCESS)
      {
        debugPrintAcceleration_float("X", xAcc);
        debugPrintAcceleration_float("Y", yAcc);
        debugPrintAcceleration_float("Z", zAcc);

        float accSum = sqrtf((xAcc * xAcc) +
                (yAcc * yAcc) +
                (zAcc * zAcc));
        debugPrintAcceleration_float("sum", accSum);
      }
      else
      {
        debugPrintln("**** ITDS_getAccelerations_float(): NOT OK ****");
      }
    }

#endif

#ifdef ITDS_EXAMPLE_ENABLE_INT
    {
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
    }

#endif // ITDS_EXAMPLE_ENABLE_INT

    /* Wait 1s */
    HAL_Delay(1000);
  }
}

/**
 * @brief  Low power mode
 * @param  No parameter.
 * @retval None
 */
void ITDS_startLowPowerMode()
{
  debugPrintln("Starting low power mode...");

  ITDS_state_t dataReady = ITDS_disable;

  /* Enable low power mode */
  ITDS_setOperatingMode(&itds, ITDS_normalOrLowPower);
  ITDS_setPowerMode(&itds, ITDS_lowPower);
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

  while (1)
  {
    /* Wait until the value is ready to read */
    do
    {
      ITDS_isAccelerationDataReady(&itds, &dataReady);
    } while (dataReady == ITDS_disable);

    /* Below, you'll find examples for reading acceleration values of all axes in [mg],
     * either as float or as integer. Note that as an alternative, there are also
     * functions to get the values for single axes or to get the raw, unconverted values. */

#ifdef ITDS_EXAMPLE_ENABLE_FLOAT
    {
      float xAcc, yAcc, zAcc;
      if (ITDS_getAccelerations_float(&itds, 1, &xAcc, &yAcc, &zAcc) == WE_SUCCESS)
      {
        debugPrintAcceleration_float("X", xAcc);
        debugPrintAcceleration_float("Y", yAcc);
        debugPrintAcceleration_float("Z", zAcc);

        float accSum = sqrtf((xAcc * xAcc) +
                (yAcc * yAcc) +
                (zAcc * zAcc));
        debugPrintAcceleration_float("sum", accSum);
      }
      else
      {
        debugPrintln("**** ITDS_getAccelerations_float(): NOT OK ****");
      }
    }

#endif

#ifdef ITDS_EXAMPLE_ENABLE_INT
    {
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
    }

#endif // ITDS_EXAMPLE_ENABLE_INT

    /* Wait 1s */
    HAL_Delay(1000);
  }
}

void ITDS_startTemperatureMode()
{
  debugPrintln("Starting temperature mode...");

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

  while (1)
  {

#ifdef ITDS_EXAMPLE_ENABLE_FLOAT

    float tempDegC = 0.0f;
    if (ITDS_getTemperature12bit(&itds, &tempDegC) == WE_SUCCESS)
    {
      debugPrintTemperature_float(tempDegC);
    }
    else
    {
      debugPrintln("**** ITDS_getTemperature12bit(): NOT OK ****");
    }

#endif // ITDS_EXAMPLE_ENABLE_FLOAT

#ifdef ITDS_EXAMPLE_ENABLE_INT

    int16_t temperature = 0.0f;
    if (ITDS_getRawTemperature12bit(&itds, &temperature) == WE_SUCCESS)
    {
      // Convert temperature to hundredths of degrees Celsius
      temperature = (int16_t) ((int32_t) temperature * 100) / 16 + 2500;
      debugPrintTemperature_int(temperature);
    }
    else
    {
      debugPrintln("**** ITDS_getRawTemperature12bit(): NOT OK ****");
    }

#endif // ITDS_EXAMPLE_ENABLE_FLOAT

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

#ifdef ITDS_EXAMPLE_ENABLE_FLOAT

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

  debugPrint("ITDS temperature (float) = ");
  if (tempDegC < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" degrees Celsius");
}

#endif /* ITDS_EXAMPLE_ENABLE_FLOAT */

#ifdef ITDS_EXAMPLE_ENABLE_INT

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

  debugPrint("ITDS temperature (int) = ");
  if (temperature < 0)
  {
    debugPrint("-");
  }
  debugPrint(bufferFull);
  debugPrint(".");
  debugPrint(bufferDecimals);
  debugPrintln(" degrees Celsius");
}

#endif /* ITDS_EXAMPLE_ENABLE_INT */
