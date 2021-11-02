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

#include "WSEN_ITDS_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gpio.h"
#include "i2c.h"
#include "usart.h"

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
ITDS_example_mode itdsExampleMode = highPerformanceExample;

/* Sensor initialization function */
bool ITDS_init(void);

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
bool ITDS_init(void)
{
  /* Initialize sensor interface (i2c with ITDS address, burst mode activated) */
  WE_sensorInterface_t interface;
  ITDS_getInterface(&interface);
  interface.interfaceType = WE_i2c;
  interface.options.i2c.burstMode = 1;
  interface.handle = &hi2c1;
  ITDS_initInterface(&interface);

  /* Wait for boot */
  HAL_Delay(50);
  while (WE_SUCCESS != ITDS_isInterfaceReady())
  {
  }
  debugPrintln("**** ITDS_isInterfaceReady(): OK ****");

  HAL_Delay(5);

  /* First communication test */
  uint8_t deviceIdValue = 0;
  if (WE_SUCCESS == ITDS_getDeviceID(&deviceIdValue))
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
  ITDS_softReset(ITDS_enable);
  ITDS_state_t swReset;
  do
  {
    ITDS_getSoftResetState(&swReset);
  } while (swReset);
  debugPrintln("**** ITDS reset complete ****");

  /* Perform reboot (retrieve trimming parameters from nonvolatile memory) */
  ITDS_reboot(ITDS_enable);
  ITDS_state_t boot;
  do
  {
    ITDS_isRebooting(&boot);
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
  int16_t xRawAcc = 0;
  int16_t yRawAcc = 0;
  int16_t zRawAcc = 0;

  /* Enable high performance mode */
  ITDS_setOperatingMode(ITDS_highPerformance);
  /* Sampling rate of 200 Hz */
  ITDS_setOutputDataRate(ITDS_odr6);
  /* Enable block data update */
  ITDS_enableBlockDataUpdate(ITDS_enable);
  /* Enable address auto increment */
  ITDS_enableAutoIncrement(ITDS_enable);
  /* Filter bandwidth = ODR/2 */
  ITDS_setFilteringCutoff(ITDS_outputDataRate_2);
  /* Full scale +-16g */
  ITDS_setFullScale(ITDS_sixteenG);

  while (1)
  {
    /* Wait until the value is ready to read */
    do
    {
      ITDS_isAccelerationDataReady(&dataReady);
    } while (dataReady == ITDS_disable);

    /* Read raw acceleration values */
    if (ITDS_getRawAccelerationX(&xRawAcc) != WE_SUCCESS)
    {
      debugPrintln("**** ITDS_getRawAccelerationX(): NOT OK ****");
      xRawAcc = 0;
    }
    if (ITDS_getRawAccelerationY(&yRawAcc) != WE_SUCCESS)
    {
      debugPrintln("**** ITDS_getRawAccelerationY(): NOT OK ****");
      yRawAcc = 0;
    }
    if (ITDS_getRawAccelerationZ(&zRawAcc) != WE_SUCCESS)
    {
      debugPrintln("**** ITDS_getRawAccelerationZ(): NOT OK ****");
      zRawAcc = 0;
    }

    /* Shift by 2 as 14bit resolution is used in high performance mode */
    xRawAcc = xRawAcc >> 2;
    yRawAcc = yRawAcc >> 2;
    zRawAcc = zRawAcc >> 2;

#ifdef ITDS_EXAMPLE_ENABLE_FLOAT
    {
      float xAcceleration = 0.0f;
      float yAcceleration = 0.0f;
      float zAcceleration = 0.0f;

      xAcceleration = (float) xRawAcc;
      xAcceleration = xAcceleration / 1000;  /* mg to g */
      xAcceleration = xAcceleration * 1.952; /* Multiply with sensitivity 1.952 in high performance mode, 14bit, and full scale +-16g */
      debugPrintAcceleration_float("X", xAcceleration);

      yAcceleration = (float) yRawAcc;
      yAcceleration = yAcceleration / 1000;
      yAcceleration = yAcceleration * 1.952;
      debugPrintAcceleration_float("Y", yAcceleration);

      zAcceleration = (float) zRawAcc;
      zAcceleration = zAcceleration / 1000;
      zAcceleration = zAcceleration * 1.952;
      debugPrintAcceleration_float("Z", zAcceleration);

      float accSum = sqrtf((xAcceleration * xAcceleration) +
              (yAcceleration * yAcceleration) +
              (zAcceleration * zAcceleration));
      debugPrintAcceleration_float("sum", accSum);
    }

#endif

#ifdef ITDS_EXAMPLE_ENABLE_INT
    {
      int32_t xAcceleration = 0;
      int32_t yAcceleration = 0;
      int32_t zAcceleration = 0;

      xAcceleration = (int32_t) xRawAcc;
      xAcceleration = (xAcceleration * 1952) / 1000; /* Multiply with sensitivity 1.952 in high performance mode, 14bit, and full scale +-16g */
      debugPrintAcceleration_int("X", xAcceleration);

      yAcceleration = (int32_t) yRawAcc;
      yAcceleration = (yAcceleration * 1952) / 1000;
      debugPrintAcceleration_int("Y", yAcceleration);

      zAcceleration = (int32_t) zRawAcc;
      zAcceleration = (zAcceleration * 1952) / 1000;
      debugPrintAcceleration_int("Z", zAcceleration);
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
  int16_t xRawAcc = 0;
  int16_t yRawAcc = 0;
  int16_t zRawAcc = 0;

  /* Enable normal mode*/
  ITDS_setOperatingMode(ITDS_normalOrLowPower);
  ITDS_setPowerMode(ITDS_normalMode);
  /* Sampling rate of 200 Hz */
  ITDS_setOutputDataRate(ITDS_odr6);
  /* Enable block data update */
  ITDS_enableBlockDataUpdate(ITDS_enable);
  /* Enable address auto increment */
  ITDS_enableAutoIncrement(ITDS_enable);
  /* Filter bandwidth = ODR/2 */
  ITDS_setFilteringCutoff(ITDS_outputDataRate_2);
  /* Full scale +-16g */
  ITDS_setFullScale(ITDS_sixteenG);

  while (1)
  {
    /* Wait until the value is ready to read */
    do
    {
      ITDS_isAccelerationDataReady(&dataReady);
    } while (dataReady == ITDS_disable);

    /* Read raw acceleration values */
    if (ITDS_getRawAccelerationX(&xRawAcc) != WE_SUCCESS)
    {
      debugPrintln("**** ITDS_getRawAccelerationX(): NOT OK ****");
      xRawAcc = 0;
    }
    if (ITDS_getRawAccelerationY(&yRawAcc) != WE_SUCCESS)
    {
      debugPrintln("**** ITDS_getRawAccelerationY(): NOT OK ****");
      yRawAcc = 0;
    }
    if (ITDS_getRawAccelerationZ(&zRawAcc) != WE_SUCCESS)
    {
      debugPrintln("**** ITDS_getRawAccelerationZ(): NOT OK ****");
      zRawAcc = 0;
    }

    /* Shift by 2 as 14bit resolution is used in normal mode */
    xRawAcc = xRawAcc >> 2;
    yRawAcc = yRawAcc >> 2;
    zRawAcc = zRawAcc >> 2;


#ifdef ITDS_EXAMPLE_ENABLE_FLOAT
    {
      float xAcceleration = 0.0f;
      float yAcceleration = 0.0f;
      float zAcceleration = 0.0f;

      xAcceleration = (float) xRawAcc;
      xAcceleration = xAcceleration / 1000;  /* mg to g */
      xAcceleration = xAcceleration * 1.952; /* Multiply with sensitivity 1.952 in normal mode, 14bit, and full scale +-16g */
      debugPrintAcceleration_float("X", xAcceleration);

      yAcceleration = (float) yRawAcc;
      yAcceleration = yAcceleration / 1000;
      yAcceleration = yAcceleration * 1.952;
      debugPrintAcceleration_float("Y", yAcceleration);

      zAcceleration = (float) zRawAcc;
      zAcceleration = zAcceleration / 1000;
      zAcceleration = zAcceleration * 1.952;
      debugPrintAcceleration_float("Z", zAcceleration);

      float accSum = sqrtf((xAcceleration * xAcceleration) +
              (yAcceleration * yAcceleration) +
              (zAcceleration * zAcceleration));
      debugPrintAcceleration_float("sum", accSum);
    }

#endif

#ifdef ITDS_EXAMPLE_ENABLE_INT
    {
      int32_t xAcceleration = 0;
      int32_t yAcceleration = 0;
      int32_t zAcceleration = 0;

      xAcceleration = (int32_t) xRawAcc;
      xAcceleration = (xAcceleration * 1952) / 1000; /* Multiply with sensitivity 1.952 in normal mode, 14bit, and full scale +-16g */
      debugPrintAcceleration_int("X", xAcceleration);

      yAcceleration = (int32_t) yRawAcc;
      yAcceleration = (yAcceleration * 1952) / 1000;
      debugPrintAcceleration_int("Y", yAcceleration);

      zAcceleration = (int32_t) zRawAcc;
      zAcceleration = (zAcceleration * 1952) / 1000;
      debugPrintAcceleration_int("Z", zAcceleration);
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
  int16_t xRawAcc = 0;
  int16_t yRawAcc = 0;
  int16_t zRawAcc = 0;

  /* Enable low power mode */
  ITDS_setOperatingMode(ITDS_normalOrLowPower);
  ITDS_setPowerMode(ITDS_lowPower);
  /* Sampling rate of 200 Hz */
  ITDS_setOutputDataRate(ITDS_odr6);
  /* Enable block data update */
  ITDS_enableBlockDataUpdate(ITDS_enable);
  /* Enable address auto increment */
  ITDS_enableAutoIncrement(ITDS_enable);
  /* Filter bandwidth = ODR/2 */
  ITDS_setFilteringCutoff(ITDS_outputDataRate_2);
  /* Full scale +-16g */
  ITDS_setFullScale(ITDS_sixteenG);

  while (1)
  {
    /* Wait until the value is ready to read */
    do
    {
      ITDS_isAccelerationDataReady(&dataReady);
    } while (dataReady == ITDS_disable);

    /* Read raw acceleration values */
    if (ITDS_getRawAccelerationX(&xRawAcc) != WE_SUCCESS)
    {
      debugPrintln("**** ITDS_getRawAccelerationX(): NOT OK ****");
      xRawAcc = 0;
    }
    if (ITDS_getRawAccelerationY(&yRawAcc) != WE_SUCCESS)
    {
      debugPrintln("**** ITDS_getRawAccelerationY(): NOT OK ****");
      yRawAcc = 0;
    }
    if (ITDS_getRawAccelerationZ(&zRawAcc) != WE_SUCCESS)
    {
      debugPrintln("**** ITDS_getRawAccelerationZ(): NOT OK ****");
      zRawAcc = 0;
    }

    /* Shift by 4 as 12bit resolution is used in low power mode */
    xRawAcc = xRawAcc >> 4;
    yRawAcc = yRawAcc >> 4;
    zRawAcc = zRawAcc >> 4;


#ifdef ITDS_EXAMPLE_ENABLE_FLOAT
    {
      float xAcceleration = 0.0f;
      float yAcceleration = 0.0f;
      float zAcceleration = 0.0f;

      xAcceleration = (float) xRawAcc;
      xAcceleration = xAcceleration / 1000;  /* mg to g */
      xAcceleration = xAcceleration * 7.808; /* Multiply with sensitivity 7.808 in low power mode, 12 bit, and full scale +-16g */
      debugPrintAcceleration_float("X", xAcceleration);

      yAcceleration = (float) yRawAcc;
      yAcceleration = yAcceleration / 1000;
      yAcceleration = yAcceleration * 7.808;
      debugPrintAcceleration_float("Y", yAcceleration);

      zAcceleration = (float) zRawAcc;
      zAcceleration = zAcceleration / 1000;
      zAcceleration = zAcceleration * 7.808;
      debugPrintAcceleration_float("Z", zAcceleration);

      float accSum = sqrtf((xAcceleration * xAcceleration) +
              (yAcceleration * yAcceleration) +
              (zAcceleration * zAcceleration));
      debugPrintAcceleration_float("sum", accSum);
    }

#endif

#ifdef ITDS_EXAMPLE_ENABLE_INT
    {
      int32_t xAcceleration = 0;
      int32_t yAcceleration = 0;
      int32_t zAcceleration = 0;

      xAcceleration = (int32_t) xRawAcc;
      xAcceleration = (xAcceleration * 7808) / 1000; /* Multiply with sensitivity 7.808 in low power mode, 12 bit, and full scale +-16g */
      debugPrintAcceleration_int("X", xAcceleration);

      yAcceleration = (int32_t) yRawAcc;
      yAcceleration = (yAcceleration * 7808) / 1000;
      debugPrintAcceleration_int("Y", yAcceleration);

      zAcceleration = (int32_t) zRawAcc;
      zAcceleration = (zAcceleration * 7808) / 1000;
      debugPrintAcceleration_int("Z", zAcceleration);
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
  ITDS_setOperatingMode(ITDS_highPerformance);
  /* Sampling rate of 200 Hz */
  ITDS_setOutputDataRate(ITDS_odr6);
  /* Enable block data update */
  ITDS_enableBlockDataUpdate(ITDS_enable);
  /* Enable address auto increment */
  ITDS_enableAutoIncrement(ITDS_enable);
  /* Filter bandwidth = ODR/2 */
  ITDS_setFilteringCutoff(ITDS_outputDataRate_2);
  /* Full scale +-16g */
  ITDS_setFullScale(ITDS_sixteenG);

  while (1)
  {

#ifdef ITDS_EXAMPLE_ENABLE_FLOAT

    float tempDegC = 0.0f;
    if (ITDS_getTemperature12bit(&tempDegC) == WE_SUCCESS)
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
    if (ITDS_getRawTemperature12bit(&temperature) == WE_SUCCESS)
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
 * @param acc  Acceleration [g]
 */
static void debugPrintAcceleration_float(char axis[], float acc)
{
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
