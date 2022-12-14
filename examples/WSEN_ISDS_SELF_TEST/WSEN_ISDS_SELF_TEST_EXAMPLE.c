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
 * @brief WSEN_ISDS self test example.
 *
 * Example for the ISDS accelerometer/gyroscope showing how to execute the sensor's self test procedure.
 */

#include "WSEN_ISDS_SELF_TEST_EXAMPLE.h"

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

/* Number of samples to use for self-test. The recommended number is 5. */
#define ISDS_SELF_TEST_SAMPLE_COUNT 5

/* Self test limits */
#define ISDS_SELF_TEST_MIN_ACC_MG     90.0f
#define ISDS_SELF_TEST_MAX_ACC_MG     1700.0f
#define ISDS_SELF_TEST_MIN_GYRO_MDPS  150000.0f
#define ISDS_SELF_TEST_MAX_GYRO_MDPS  700000.0f

/* Sensor interface configuration */
static WE_sensorInterface_t isds;

/* Sensor initialization function */
static bool ISDS_init(void);

static bool ISDS_selfTestAcc(void);
static bool ISDS_selfTestGyro(void);
static bool ISDS_discardOldDataAcc(void);
static bool ISDS_discardOldDataGyro(void);
static bool ISDS_checkSelfTestValueAcc(float value, float valueSelfTest);
static bool ISDS_checkSelfTestValueGyro(float value, float valueSelfTest);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_isdsSelfTestExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the \"self-test\" example program for the ISDS sensor.");

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
void WE_isdsSelfTestExampleLoop()
{
  bool ok = ISDS_selfTestAcc();
  if (ok)
  {
    debugPrintln("Accelerometer self-test completed successfully.");
  }
  else
  {
    debugPrintln("Accelerometer self-test completed with errors.");
  }
  debugPrintln("");

  HAL_Delay(1000);

  ok = ISDS_selfTestGyro();
  if (ok)
  {
    debugPrintln("Gyroscope self-test completed successfully.");
  }
  else
  {
    debugPrintln("Gyroscope self-test completed with errors.");
  }
  debugPrintln("");

  HAL_Delay(1000);
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

  return true;
}

/**
 * @brief Runs the ISDS accelerometer self-test.
 * @retval True if the test was successful, false if not.
 */
static bool ISDS_selfTestAcc(void)
{
  /* Last read raw acceleration values */
  int16_t rawAccX;
  int16_t rawAccY;
  int16_t rawAccZ;

  /* Average of accelerations captured with self-test mode disabled [mg] */
  float avgX = 0;
  float avgY = 0;
  float avgZ = 0;

  /* Average of accelerations captured with self-test mode enabled [mg] */
  float avgXSelfTest = 0;
  float avgYSelfTest = 0;
  float avgZSelfTest = 0;

  /* Perform soft reset of the sensor */
  ISDS_softReset(&isds, ISDS_enable);
  ISDS_state_t swReset;
  do
  {
    ISDS_getSoftResetState(&isds, &swReset);
  } while (swReset);

  /* Enable block data update */
  ISDS_enableBlockDataUpdate(&isds, ISDS_enable);

  /* Full scale +-4g */
  ISDS_setAccFullScale(&isds, ISDS_accFullScaleFourG);

  /* Sampling rate of 52 Hz */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr52Hz);

  /* Wait 100 ms for stable sensor output */
  HAL_Delay(100);

  /* Clear old samples (if any) */
  ISDS_discardOldDataAcc();

  /* Collect ISDS_SELF_TEST_SAMPLE_COUNT samples (with sensor not in self-test mode) */
  for (uint8_t i = 0; i < ISDS_SELF_TEST_SAMPLE_COUNT; )
  {
    ISDS_state_t dataReady;
    ISDS_isAccelerationDataReady(&isds, &dataReady);

    if (dataReady == ISDS_enable)
    {
      /* Read accelerometer data */
      if (WE_FAIL == ISDS_getRawAccelerations(&isds, &rawAccX, &rawAccY, &rawAccZ))
      {
        return false;
      }

      /* Convert to mg and add to sum */
      avgX += ISDS_convertAccelerationFs4g_float(rawAccX);
      avgY += ISDS_convertAccelerationFs4g_float(rawAccY);
      avgZ += ISDS_convertAccelerationFs4g_float(rawAccZ);

      i++;
    }
  }

  /* Compute average */
  avgX /= ISDS_SELF_TEST_SAMPLE_COUNT;
  avgY /= ISDS_SELF_TEST_SAMPLE_COUNT;
  avgZ /= ISDS_SELF_TEST_SAMPLE_COUNT;


  /* Enable self-test mode (positive) */
  ISDS_setAccSelfTestMode(&isds, ISDS_accSelfTestModePositive);

  /* Wait 100 ms for stable sensor output */
  HAL_Delay(100);

  /* Clear old samples (if any) */
  ISDS_discardOldDataAcc();

  /* Collect ISDS_SELF_TEST_SAMPLE_COUNT samples (with sensor in self-test mode) */
  for (uint8_t i = 0; i < ISDS_SELF_TEST_SAMPLE_COUNT; )
  {
    ISDS_state_t dataReady;
    ISDS_isAccelerationDataReady(&isds, &dataReady);

    if (dataReady == ISDS_enable)
    {
      /* Read accelerometer data */
      if (WE_FAIL == ISDS_getRawAccelerations(&isds, &rawAccX, &rawAccY, &rawAccZ))
      {
        return false;
      }

      /* Convert to mg and add to sum */
      avgXSelfTest += ISDS_convertAccelerationFs4g_float(rawAccX);
      avgYSelfTest += ISDS_convertAccelerationFs4g_float(rawAccY);
      avgZSelfTest += ISDS_convertAccelerationFs4g_float(rawAccZ);

      i++;
    }
  }

  /* Compute average */
  avgXSelfTest /= ISDS_SELF_TEST_SAMPLE_COUNT;
  avgYSelfTest /= ISDS_SELF_TEST_SAMPLE_COUNT;
  avgZSelfTest /= ISDS_SELF_TEST_SAMPLE_COUNT;


  /* Check value range for all three axes */
  bool xPass = ISDS_checkSelfTestValueAcc(avgX, avgXSelfTest);
  bool yPass = ISDS_checkSelfTestValueAcc(avgY, avgYSelfTest);
  bool zPass = ISDS_checkSelfTestValueAcc(avgZ, avgZSelfTest);


  /* Print results */
  debugPrintln("**********************************************");
  debugPrintln("**** ISDS accelerometer self-test results ****");
  debugPrint("****             X axis: ");
  debugPrint(xPass ? "PASS" : "FAIL");
  debugPrintln("             ****");
  debugPrint("****             Y axis: ");
  debugPrint(yPass ? "PASS" : "FAIL");
  debugPrintln("             ****");
  debugPrint("****             Z axis: ");
  debugPrint(zPass ? "PASS" : "FAIL");
  debugPrintln("             ****");
  debugPrintln("**********************************************");

  /* Disable self-test mode */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdrOff);
  ISDS_setAccSelfTestMode(&isds, ISDS_accSelfTestModeOff);

  return (xPass && yPass && zPass);
}

/**
 * @brief Runs the ISDS gyroscope self-test.
 * @retval True if the test was successful, false if not.
 */
static bool ISDS_selfTestGyro(void)
{
  /* Last read raw angular rates */
  int16_t rawRateX;
  int16_t rawRateY;
  int16_t rawRateZ;

  /* Average of angular rates captured with self-test mode disabled [mdps] */
  float avgX = 0;
  float avgY = 0;
  float avgZ = 0;

  /* Average of angular rates captured with self-test mode enabled [mdps] */
  float avgXSelfTest = 0;
  float avgYSelfTest = 0;
  float avgZSelfTest = 0;

  /* Perform soft reset of the sensor */
  ISDS_softReset(&isds, ISDS_enable);
  ISDS_state_t swReset;
  do
  {
    ISDS_getSoftResetState(&isds, &swReset);
  } while (swReset);

  /* Enable block data update */
  ISDS_enableBlockDataUpdate(&isds, ISDS_enable);

  /* Full scale 2000 dps */
  ISDS_setGyroFullScale(&isds, ISDS_gyroFullScale2000dps);

  /* Sampling rate of 208 Hz */
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr208Hz);

  /* Wait 150 ms for stable sensor output */
  HAL_Delay(150);

  /* Clear old samples (if any) */
  ISDS_discardOldDataGyro();

  /* Collect ISDS_SELF_TEST_SAMPLE_COUNT samples (with sensor not in self-test mode) */
  for (uint8_t i = 0; i < ISDS_SELF_TEST_SAMPLE_COUNT; )
  {
    ISDS_state_t dataReady;
    ISDS_isGyroscopeDataReady(&isds, &dataReady);

    if (dataReady == ISDS_enable)
    {
      /* Read gyroscope data */
      if (WE_FAIL == ISDS_getRawAngularRates(&isds, &rawRateX, &rawRateY, &rawRateZ))
      {
        return false;
      }

      /* Convert to mdps and add to sum */
      avgX += ISDS_convertAngularRateFs2000dps_float(rawRateX);
      avgY += ISDS_convertAngularRateFs2000dps_float(rawRateY);
      avgZ += ISDS_convertAngularRateFs2000dps_float(rawRateZ);

      i++;
    }
  }

  /* Compute average */
  avgX /= ISDS_SELF_TEST_SAMPLE_COUNT;
  avgY /= ISDS_SELF_TEST_SAMPLE_COUNT;
  avgZ /= ISDS_SELF_TEST_SAMPLE_COUNT;


  /* Enable self-test mode (positive) */
  ISDS_setGyroSelfTestMode(&isds, ISDS_gyroSelfTestModePositive);

  /* Wait 50 ms for stable sensor output */
  HAL_Delay(50);

  /* Clear old samples (if any) */
  ISDS_discardOldDataGyro();

  /* Collect ISDS_SELF_TEST_SAMPLE_COUNT samples (with sensor in self-test mode) */
  for (uint8_t i = 0; i < ISDS_SELF_TEST_SAMPLE_COUNT; )
  {
    ISDS_state_t dataReady;
    ISDS_isGyroscopeDataReady(&isds, &dataReady);

    if (dataReady == ISDS_enable)
    {
      /* Read gyroscope data */
      if (WE_FAIL == ISDS_getRawAngularRates(&isds, &rawRateX, &rawRateY, &rawRateZ))
      {
        return false;
      }

      /* Convert to mdps and add to sum */
      avgXSelfTest += ISDS_convertAngularRateFs2000dps_float(rawRateX);
      avgYSelfTest += ISDS_convertAngularRateFs2000dps_float(rawRateY);
      avgZSelfTest += ISDS_convertAngularRateFs2000dps_float(rawRateZ);

      i++;
    }
  }

  /* Compute average */
  avgXSelfTest /= ISDS_SELF_TEST_SAMPLE_COUNT;
  avgYSelfTest /= ISDS_SELF_TEST_SAMPLE_COUNT;
  avgZSelfTest /= ISDS_SELF_TEST_SAMPLE_COUNT;


  /* Check value range for all three axes */
  bool xPass = ISDS_checkSelfTestValueGyro(avgX, avgXSelfTest);
  bool yPass = ISDS_checkSelfTestValueGyro(avgY, avgYSelfTest);
  bool zPass = ISDS_checkSelfTestValueGyro(avgZ, avgZSelfTest);


  /* Print results */
  debugPrintln("**********************************************");
  debugPrintln("****   ISDS gyroscope self-test results   ****");
  debugPrint("****             X axis: ");
  debugPrint(xPass ? "PASS" : "FAIL");
  debugPrintln("             ****");
  debugPrint("****             Y axis: ");
  debugPrint(yPass ? "PASS" : "FAIL");
  debugPrintln("             ****");
  debugPrint("****             Z axis: ");
  debugPrint(zPass ? "PASS" : "FAIL");
  debugPrintln("             ****");
  debugPrintln("**********************************************");

  /* Disable self-test mode */
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdrOff);
  ISDS_setGyroSelfTestMode(&isds, ISDS_gyroSelfTestModeOff);

  return (xPass && yPass && zPass);
}

/**
 * @brief Discards the current accelerometer sample (if any).
 */
static bool ISDS_discardOldDataAcc(void)
{
  ISDS_state_t dataReady;
  ISDS_isAccelerationDataReady(&isds, &dataReady);

  if (dataReady == ISDS_enable)
  {
    int16_t xRawAcc, yRawAcc, zRawAcc;
    if (WE_FAIL == ISDS_getRawAccelerations(&isds, &xRawAcc, &yRawAcc, &zRawAcc))
    {
      return false;
    }
  }
  return true;
}

/**
 * @brief Discards the current gyroscope sample (if any).
 */
static bool ISDS_discardOldDataGyro(void)
{
  ISDS_state_t dataReady;
  ISDS_isGyroscopeDataReady(&isds, &dataReady);

  if (dataReady == ISDS_enable)
  {
    int16_t xRawRate, yRawRate, zRawRate;
    if (WE_FAIL == ISDS_getRawAngularRates(&isds, &xRawRate, &yRawRate, &zRawRate))
    {
      return false;
    }
  }
  return true;
}

/**
 * @brief Checks the validity of the supplied acceleration values.
 * @param value Acceleration value captured with self-test mode disabled.
 * @param valueSelfTest Acceleration value captured with self-test mode enabled.
 * @retval True if the difference of the supplied values is in the valid range.
 */
static bool ISDS_checkSelfTestValueAcc(float value, float valueSelfTest)
{
  float diff = fabs(valueSelfTest - value);
  return (diff >= ISDS_SELF_TEST_MIN_ACC_MG && diff <= ISDS_SELF_TEST_MAX_ACC_MG);
}

/**
 * @brief Checks the validity of the supplied angular rate values.
 * @param value Angular rate value captured with self-test mode disabled.
 * @param valueSelfTest Angular rate value captured with self-test mode enabled.
 * @retval True if the difference of the supplied values is in the valid range.
 */
static bool ISDS_checkSelfTestValueGyro(float value, float valueSelfTest)
{
  float diff = fabs(valueSelfTest - value);
  return (diff >= ISDS_SELF_TEST_MIN_GYRO_MDPS && diff <= ISDS_SELF_TEST_MAX_GYRO_MDPS);
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
