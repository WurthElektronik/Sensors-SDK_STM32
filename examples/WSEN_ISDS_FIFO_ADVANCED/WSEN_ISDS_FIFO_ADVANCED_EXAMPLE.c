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
 * @brief WSEN_ISDS FIFO advanced example.
 *
 * Example for the ISDS accelerometer/gyroscope demonstrating advanced usage of the FIFO
 * buffer (temperature, high data only, patterns, DEN triggers).
 */

#include "WSEN_ISDS_FIFO_ADVANCED_EXAMPLE.h"

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


/* FIFO fill threshold - an event is generated when the FIFO buffer is
 * filled up to this amount (6 values per sample, 3 accelerations and 3
 * angular rates). */
#define FIFO_THRESH (12 * 6)

typedef enum
{
  ISDS_fifoAdvancedExampleTemperature,
  ISDS_fifoAdvancedExampleOnlyHighData,
  ISDS_fifoAdvancedExamplePattern,
  ISDS_fifoAdvancedExampleEdgeSensitiveTrigger,
  ISDS_fifoAdvancedExampleLevelSensitiveTrigger,
  ISDS_fifoAdvancedExampleLevelSensitiveLatched,
  ISDS_fifoAdvancedExampleLevelSensitiveFifo
} ISDS_fifoAdvancedExample_t;

/* Use the following variable to switch among the available example modes */
static ISDS_fifoAdvancedExample_t isdsFifoAdvancedExampleMode = ISDS_fifoAdvancedExampleTemperature;

/* Sensor interface configuration */
static WE_sensorInterface_t isds;

/* Is set to true when an interrupt has been triggered */
static bool interrupt0Triggered = false;

/* Sensor initialization function */
static bool ISDS_init(void);

/* Functions containing main loops for the available example modes */
static void ISDS_startTemperatureExample();
static void ISDS_startOnlyHighDataExample();
static void ISDS_startPatternExample();
static void ISDS_startEdgeSensitiveTriggerExample();
static void ISDS_startLevelSensitiveTriggerExample();
static void ISDS_startLevelSensitiveLatchedExample();
static void ISDS_startLevelSensitiveFifoExample();

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);
static void debugPrintAcceleration_int(char axis[], int32_t accMg);
static void debugPrintAngularRate_int(char axis[], int32_t rateMdps);
static void debugPrintTemperature_int(int16_t temperature);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_isdsFifoAdvancedExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the \"advanced FIFO\" example program for the ISDS sensor.");
  debugPrintln("Note that for this example to work, the following pin/interrupt configuration is required:");
  debugPrintln("* INT_0 to PA0, rising edge interrupt only");
  debugPrintln("* PA1 to INT_1, output used as data enable (DEN) signal for DEN examples");

  /* init ISDS */
  if (false == ISDS_init())
  {
    debugPrintln("**** ISDS_Init() error. STOP ****");
    HAL_Delay(5);
    while (1);
  }
}

/**
 * @brief Example main loop code.
 * Call this function in main loop (infinite loop).
 */
void WE_isdsFifoAdvancedExampleLoop()
{
  switch (isdsFifoAdvancedExampleMode)
  {

  case ISDS_fifoAdvancedExampleTemperature:
    ISDS_startTemperatureExample();
    break;

  case ISDS_fifoAdvancedExampleOnlyHighData:
    ISDS_startOnlyHighDataExample();
    break;

  case ISDS_fifoAdvancedExamplePattern:
    ISDS_startPatternExample();
    break;

  case ISDS_fifoAdvancedExampleEdgeSensitiveTrigger:
    ISDS_startEdgeSensitiveTriggerExample();
    break;

  case ISDS_fifoAdvancedExampleLevelSensitiveTrigger:
    ISDS_startLevelSensitiveTriggerExample();
    break;

  case ISDS_fifoAdvancedExampleLevelSensitiveLatched:
    ISDS_startLevelSensitiveLatchedExample();
    break;

  case ISDS_fifoAdvancedExampleLevelSensitiveFifo:
    ISDS_startLevelSensitiveFifoExample();
    break;
  }
}

/**
 * @brief Initializes the sensor for this example application.
 */
bool ISDS_init(void)
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

  /* Set FIFO initially off */
  ISDS_setFifoMode(&isds, ISDS_bypassMode);

  /* Enable block data update */
  ISDS_enableBlockDataUpdate(&isds, ISDS_enable);

  /* Enable address auto increment */
  ISDS_enableAutoIncrement(&isds, ISDS_enable);

  /* Full scale +-2 g and +-2000 dps */
  ISDS_setAccFullScale(&isds, ISDS_accFullScaleTwoG);
  ISDS_setGyroFullScale(&isds, ISDS_gyroFullScale2000dps);

  /* Set FIFO fill threshold (note that the threshold is not used in all examples) */
  ISDS_setFifoThreshold(&isds, FIFO_THRESH);

  /* Interrupts are active high */
  ISDS_setInterruptActiveLevel(&isds, ISDS_activeHigh);

  /* Interrupts are push-pull */
  ISDS_setInterruptPinType(&isds, ISDS_pushPull);

  /* Latched mode disabled (interrupt signal is automatically reset) */
  ISDS_enableLatchedInterrupt(&isds, ISDS_disable);

  /* Enable interrupts */
  ISDS_enableInterrupts(&isds, ISDS_enable);

  /* Note that, depending on the configuration of the filtering chain of the
   * accelerometer and the gyroscope, it might be necessary to discard the
   * first samples after changing ODRs or when changing the power mode.
   * Consult the user manual for details on the number of samples to be discarded. */

  return true;
}


/**
 * @brief Example for collecting temperature data in FIFO.
 *
 * In this example, the ISDS temperature sensor readings are collected at the same rate
 * as the other sensor readings (angular rates and accelerations). When the FIFO is
 * full, the average sensor readings are printed.
 */
static void ISDS_startTemperatureExample()
{
  debugPrintln("Starting FIFO temperature example...");

  /* Sampling rate of 208 Hz */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr208Hz);
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr208Hz);

  /* Set FIFO to operate at the same data rate as accelerometer and gyroscope */
  ISDS_setFifoOutputDataRate(&isds, ISDS_fifoOdr208Hz);

  /* Enable FIFO full interrupt */
  ISDS_enableFifoFullINT0(&isds, ISDS_enable);

  /* Collect temperature in FIFO */
  ISDS_enableFifoTemperature(&isds, ISDS_enable);

  /* No decimation of accelerations, angular rates and temperatures
   * (i.e. use the FIFO sample rate set above) */
  ISDS_setFifoAccDecimation(&isds, ISDS_fifoDecimationDisabled);
  ISDS_setFifoGyroDecimation(&isds, ISDS_fifoDecimationDisabled);
  ISDS_setFifoDataset4Decimation(&isds, ISDS_fifoDecimationDisabled);

  /* Enable FIFO mode */
  ISDS_setFifoMode(&isds, ISDS_fifoEnabled);

  uint16_t fifoDataRaw[2048];
  int32_t xRateAvg = 0;
  int32_t yRateAvg = 0;
  int32_t zRateAvg = 0;
  int32_t xAccAvg = 0;
  int32_t yAccAvg = 0;
  int32_t zAccAvg = 0;
  int32_t temperatureAvg = 0;

  while (1)
  {
    if (interrupt0Triggered == true)
    {
      /* FIFO buffer is filled up to threshold */
      interrupt0Triggered = false;

      /* Get FIFO status */
      ISDS_fifoStatus2_t status;
      uint16_t fillLevel, fifoPattern;
      ISDS_getFifoStatus(&isds, &status, &fillLevel, &fifoPattern);

      /* Note that we are always reading a multiple of the number of variables being
       * captured and the collection rate is identical for all data sets -
       * therefore fifoPattern should always be zero and the order of
       * the retrieved values should always be
       * xRate,yRate,zRate,xAcc,yAcc,zAcc,UNUSED,temperature,UNUSED,...
       * Depending on the timing and the way the FIFO is read, fifoPattern might
       * be non-zero - in that case, we must use fifoPattern to determine the type
       * of samples read from the FIFO. In this example, we skip the first samples
       * if necessary, so that the first used sample is always xRate. */
      uint16_t patternOffset = (fifoPattern == 0) ? 0 : (9 - fifoPattern);

      /* Retrieve samples from sensor */
      if (WE_SUCCESS == ISDS_getFifoData(&isds, fillLevel, fifoDataRaw))
      {
        xRateAvg = 0;
        yRateAvg = 0;
        zRateAvg = 0;
        xAccAvg = 0;
        yAccAvg = 0;
        zAccAvg = 0;
        temperatureAvg = 0;

        uint16_t numSamples = (fillLevel - patternOffset) / 9;

        for (uint16_t i = 0; i < numSamples; i++)
        {
          uint16_t offset = i * 9 + patternOffset;
          xRateAvg += *(int16_t*) &fifoDataRaw[offset];
          yRateAvg += *(int16_t*) &fifoDataRaw[offset + 1];
          zRateAvg += *(int16_t*) &fifoDataRaw[offset + 2];
          xAccAvg += *(int16_t*) &fifoDataRaw[offset + 3];
          yAccAvg += *(int16_t*) &fifoDataRaw[offset + 4];
          zAccAvg += *(int16_t*) &fifoDataRaw[offset + 5];

          uint8_t *tempData = (uint8_t*) &fifoDataRaw[offset + 7];
          temperatureAvg += (tempData[1] << 8) | tempData[0];
        }

        xRateAvg /= numSamples;
        yRateAvg /= numSamples;
        zRateAvg /= numSamples;
        xAccAvg /= numSamples;
        yAccAvg /= numSamples;
        zAccAvg /= numSamples;
        temperatureAvg /= numSamples;

        xRateAvg = ISDS_convertAngularRateFs2000dps_int(xRateAvg);
        yRateAvg = ISDS_convertAngularRateFs2000dps_int(yRateAvg);
        zRateAvg = ISDS_convertAngularRateFs2000dps_int(zRateAvg);
        xAccAvg = ISDS_convertAccelerationFs2g_int(xAccAvg);
        yAccAvg = ISDS_convertAccelerationFs2g_int(yAccAvg);
        zAccAvg = ISDS_convertAccelerationFs2g_int(zAccAvg);
        temperatureAvg = ISDS_convertTemperature_int(temperatureAvg);

        char msg[64];
        sprintf(msg, "Average of %u samples:", numSamples);
        debugPrintln(msg);
        debugPrintAcceleration_int("X", xAccAvg);
        debugPrintAcceleration_int("Y", yAccAvg);
        debugPrintAcceleration_int("Z", zAccAvg);
        debugPrintAngularRate_int("X", xRateAvg);
        debugPrintAngularRate_int("Y", yRateAvg);
        debugPrintAngularRate_int("Z", zRateAvg);
        debugPrintTemperature_int(temperatureAvg);
      }
      else
      {
        debugPrintln("**** ISDS_getFifoData() failed");
      }

      /* Restart data collection by first setting bypass mode and then re-enabling FIFO mode */
      ISDS_setFifoMode(&isds, ISDS_bypassMode);
      ISDS_setFifoMode(&isds, ISDS_fifoEnabled);
    }
  }
}

/**
 * @brief Example for collecting only MSBs of gyroscope and accelerometer output.
 *
 * This allows us to store twice as many samples in the FIFO buffer (compared
 * to normal FIFO operation).
 */
static void ISDS_startOnlyHighDataExample()
{
  debugPrintln("Starting FIFO only high data example...");

  /* Enable FIFO full interrupt */
  ISDS_enableFifoFullINT0(&isds, ISDS_enable);

  /* Enable only high data mode */
  ISDS_enableFifoOnlyHighData(&isds, ISDS_enable);

  /* Set higher data rates, as we can store more samples in only high data mode */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr1k66Hz);
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr1k66Hz);
  ISDS_setFifoOutputDataRate(&isds, ISDS_fifoOdr1k66Hz);

  /* No decimation of accelerations and angular rates in FIFO (use the ODR set above) */
  ISDS_setFifoGyroDecimation(&isds, ISDS_fifoDecimationDisabled);
  /* Decimation parameter for accelerations must be set to zero in only high data mode! */
  ISDS_setFifoAccDecimation(&isds, ISDS_fifoDecimationNoFifo);

  /* Enable FIFO mode */
  ISDS_setFifoMode(&isds, ISDS_fifoEnabled);

  uint16_t fifoDataRaw[2048];
  int32_t xRateAvg = 0;
  int32_t yRateAvg = 0;
  int32_t zRateAvg = 0;
  int32_t xAccAvg = 0;
  int32_t yAccAvg = 0;
  int32_t zAccAvg = 0;

  while (1)
  {
    if (interrupt0Triggered == true)
    {
      /* FIFO buffer is filled up to threshold */
      interrupt0Triggered = false;

      /* Get FIFO status */
      ISDS_fifoStatus2_t status;
      uint16_t fillLevel, fifoPattern;
      ISDS_getFifoStatus(&isds, &status, &fillLevel, &fifoPattern);

      /* Note that we are always reading a multiple of the number of variables being
       * captured and the collection rate is identical for all data sets -
       * therefore fifoPattern should always be zero and the order of
       * the retrieved values should always be
       * xRateAndAcc,yRateAndAcc,zRateAndAcc,...
       * Depending on the timing and the way the FIFO is read, fifoPattern might
       * be non-zero - in that case, we must use fifoPattern to determine the type
       * of samples read from the FIFO. In this example, we skip the first samples
       * if necessary, so that the first used sample is always xRateAndAcc. */
      uint16_t patternOffset = (fifoPattern == 0) ? 0 : (3 - fifoPattern);

      /* Retrieve samples from sensor */
      if (WE_SUCCESS == ISDS_getFifoData(&isds, fillLevel, fifoDataRaw))
      {
        xRateAvg = 0;
        yRateAvg = 0;
        zRateAvg = 0;
        xAccAvg = 0;
        yAccAvg = 0;
        zAccAvg = 0;

        uint16_t numSamples = (fillLevel - patternOffset) / 3;

        for (uint16_t i = 0; i < numSamples; i++)
        {
          uint16_t offset = i * 3 + patternOffset;
          int8_t *fifoDataBytes = (int8_t *) &fifoDataRaw[offset];
          xRateAvg += ((int16_t) fifoDataBytes[1]) << 8;
          yRateAvg += ((int16_t) fifoDataBytes[3]) << 8;
          zRateAvg += ((int16_t) fifoDataBytes[5]) << 8;
          xAccAvg += ((int16_t) fifoDataBytes[0]) << 8;
          yAccAvg += ((int16_t) fifoDataBytes[2]) << 8;
          zAccAvg += ((int16_t) fifoDataBytes[4]) << 8;
        }

        xRateAvg /= numSamples;
        yRateAvg /= numSamples;
        zRateAvg /= numSamples;
        xAccAvg /= numSamples;
        yAccAvg /= numSamples;
        zAccAvg /= numSamples;

        xRateAvg = ISDS_convertAngularRateFs2000dps_int(xRateAvg);
        yRateAvg = ISDS_convertAngularRateFs2000dps_int(yRateAvg);
        zRateAvg = ISDS_convertAngularRateFs2000dps_int(zRateAvg);
        xAccAvg = ISDS_convertAccelerationFs2g_int(xAccAvg);
        yAccAvg = ISDS_convertAccelerationFs2g_int(yAccAvg);
        zAccAvg = ISDS_convertAccelerationFs2g_int(zAccAvg);

        char msg[64];
        sprintf(msg, "Average of %u samples (high data only):", numSamples);
        debugPrintln(msg);
        debugPrintAcceleration_int("X", xAccAvg);
        debugPrintAcceleration_int("Y", yAccAvg);
        debugPrintAcceleration_int("Z", zAccAvg);
        debugPrintAngularRate_int("X", xRateAvg);
        debugPrintAngularRate_int("Y", yRateAvg);
        debugPrintAngularRate_int("Z", zRateAvg);
      }
      else
      {
        debugPrintln("**** ISDS_getFifoData() failed");
      }

      /* Restart data collection by first setting bypass mode and then re-enabling FIFO mode */
      ISDS_setFifoMode(&isds, ISDS_bypassMode);
      ISDS_setFifoMode(&isds, ISDS_fifoEnabled);
    }
  }
}


/**
 * @brief Example demonstrating usage of the FIFO pattern.
 *
 * In this example, angular rates, accelerations and temperature values are captured
 * at different rates - accelerations at 416 Hz, angular rates at 208 Hz and
 * temperature readings at 104 Hz.
 *
 * The buffer contents is read at regular intervals. The read values are then interpreted
 * according to the FIFO pattern, which defines the order of the values in the FIFO buffer.
 *
 * For a given configuration, the order of the values is always the same, so it is not
 * necessary to query the FIFO pattern variable for each sample - it is sufficient to
 * query the variable once before reading a data set from the buffer in order to know
 * what values will be read next.
 *
 * In this example, the pattern is as follows (repeats every 21 values):
 * -------------------------------------------
 * Time   FIFO pattern    Next element in FIFO
 * -------------------------------------------
 * t0     0               xRate
 * t0     1               yRate
 * t0     2               zRate
 * t0     3               xAcc
 * t0     4               yAcc
 * t0     5               zAcc
 * t0     6               UNUSED
 * t0     7               temperature
 * t0     8               UNUSED
 * t1     9               xAcc
 * t1     10              yAcc
 * t1     11              zAcc
 * t2     12              xRate
 * t2     13              yRate
 * t2     14              zRate
 * t2     15              xAcc
 * t2     16              yAcc
 * t2     17              zAcc
 * t3     18              xAcc
 * t3     19              yAcc
 * t3     20              zAcc
 * -------------------------------------------
 * t4     0               xRate
 * t4     1               yRate
 * ...
 * -------------------------------------------
 */
static void ISDS_startPatternExample()
{
  debugPrintln("Starting FIFO pattern example...");

  /* Enable FIFO overrun interrupt */
  ISDS_enableFifoOverrunINT0(&isds, ISDS_enable);

  /* Set data rates */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr416Hz);
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr208Hz);
  ISDS_setFifoOutputDataRate(&isds, ISDS_fifoOdr416Hz);

  /* Collect temperature in FIFO */
  ISDS_enableFifoTemperature(&isds, ISDS_enable);

  /* Set decimation of accelerations, angular rates and temperatures.
   * The FIFO data rate set above is ODR_FIFO = 416 Hz, so the individual
   * capturing rates resulting from the decimation factors are
   * - ODR_ACC = 1 * ODR_FIFO = 416 Hz
   * - ODR_GYRO = 1/2 * ODR_FIFO = 208 Hz
   * - ODR_TEMP = 1/4 * ODR_FIFO = 104 Hz */
  ISDS_setFifoAccDecimation(&isds, ISDS_fifoDecimationDisabled);
  ISDS_setFifoGyroDecimation(&isds, ISDS_fifoDecimationFactor2);
  ISDS_setFifoDataset4Decimation(&isds, ISDS_fifoDecimationFactor4);

  /* Enable FIFO continuous mode */
  ISDS_setFifoMode(&isds, ISDS_continuousMode);

  uint16_t fifoDataRaw[2048];
  int32_t xRateAvg = 0;
  int32_t yRateAvg = 0;
  int32_t zRateAvg = 0;
  int32_t xAccAvg = 0;
  int32_t yAccAvg = 0;
  int32_t zAccAvg = 0;
  int32_t temperatureAvg = 0;

  /* Variables counting the number of rate/acceleration/temperature samples
   * contained in the FIFO data read in one iteration.
   * Note that for the sake of simplicity, only the number of X axis samples are
   * counted. */
  uint16_t countRate = 0;
  uint16_t countAcc = 0;
  uint16_t countTemp = 0;

  /* Read FIFO data every 500 ms */
  uint32_t readInterval = 500;
  uint32_t nextReadTime = HAL_GetTick() + readInterval;

  while (1)
  {
    uint32_t currentTime = HAL_GetTick();

    if (interrupt0Triggered == true)
    {
      /* FIFO buffer overrun - this shouldn't happen */
      interrupt0Triggered = false;

      debugPrintln("FIFO buffer overrun!");
    }

    /* Read data and print average values at regular intervals */
    if (currentTime >= nextReadTime)
    {
      nextReadTime = currentTime + readInterval;

      /* Get FIFO status */
      ISDS_fifoStatus2_t status;
      uint16_t fillLevel, fifoPattern;
      ISDS_getFifoStatus(&isds, &status, &fillLevel, &fifoPattern);

      if (fillLevel == 0)
      {
        /* FIFO is currently empty */
        continue;
      }

      xRateAvg = 0;
      yRateAvg = 0;
      zRateAvg = 0;
      xAccAvg = 0;
      yAccAvg = 0;
      zAccAvg = 0;
      temperatureAvg = 0;
      countRate = 0;
      countAcc = 0;
      countTemp = 0;


      /* Retrieve samples from sensor */
      if (WE_SUCCESS == ISDS_getFifoData(&isds, fillLevel, fifoDataRaw))
      {
        /* Iterate over the retrieved values, interpreting the values based on the FIFO pattern
         * (Note that when running this example, the pattern value will in most cases be zero) */
        for (uint16_t i = 0; i < fillLevel; i++)
        {
          /* In this example, the FIFO pattern repeats every 21 values */
          switch ((fifoPattern + i) % 21)
          {
          case 0:
          case 12:
            xRateAvg += *(int16_t*) &fifoDataRaw[i];
            countRate++;
            break;

          case 1:
          case 13:
            yRateAvg += *(int16_t*) &fifoDataRaw[i];
            break;

          case 2:
          case 14:
            zRateAvg += *(int16_t*) &fifoDataRaw[i];
            break;

          case 3:
          case 9:
          case 15:
          case 18:
            xAccAvg += *(int16_t*) &fifoDataRaw[i];
            countAcc++;
            break;

          case 4:
          case 10:
          case 16:
          case 19:
            yAccAvg += *(int16_t*) &fifoDataRaw[i];
            break;

          case 5:
          case 11:
          case 17:
          case 20:
            zAccAvg += *(int16_t*) &fifoDataRaw[i];
            break;

          case 6:
            /* Unused (temperature only uses byte 2 of 3) */
            break;

          case 7:
          {
            uint8_t *tempData = (uint8_t*) &fifoDataRaw[i];
            temperatureAvg += (tempData[1] << 8) | tempData[0];
            countTemp++;
            break;
          }

          case 8:
            /* Unused (temperature only uses byte 2 of 3) */
            break;
          }
        }
      }
      else
      {
        debugPrintln("**** ISDS_getFifoData() failed");
      }


      /* Compute average values */
      xRateAvg /= countRate;
      yRateAvg /= countRate;
      zRateAvg /= countRate;
      xAccAvg /= countAcc;
      yAccAvg /= countAcc;
      zAccAvg /= countAcc;
      temperatureAvg /= countTemp;

      xRateAvg = ISDS_convertAngularRateFs2000dps_int(xRateAvg);
      yRateAvg = ISDS_convertAngularRateFs2000dps_int(yRateAvg);
      zRateAvg = ISDS_convertAngularRateFs2000dps_int(zRateAvg);
      xAccAvg = ISDS_convertAccelerationFs2g_int(xAccAvg);
      yAccAvg = ISDS_convertAccelerationFs2g_int(yAccAvg);
      zAccAvg = ISDS_convertAccelerationFs2g_int(zAccAvg);
      temperatureAvg = ISDS_convertTemperature_int(temperatureAvg);

      /* Print counter and average values */
      char msg[128];
      sprintf(msg,
              "FIFO Pattern offset = %u. Average of %u acceleration, %u angular rate and %u temperature samples:",
              fifoPattern, countAcc, countRate, countTemp);
      debugPrintln(msg);
      debugPrintAcceleration_int("X", xAccAvg);
      debugPrintAcceleration_int("Y", yAccAvg);
      debugPrintAcceleration_int("Z", zAccAvg);
      debugPrintAngularRate_int("X", xRateAvg);
      debugPrintAngularRate_int("Y", yRateAvg);
      debugPrintAngularRate_int("Z", zRateAvg);
      debugPrintTemperature_int(temperatureAvg);


      /* Restart data collection by first setting bypass mode and then re-enabling FIFO mode */
      ISDS_setFifoMode(&isds, ISDS_bypassMode);
      ISDS_setFifoMode(&isds, ISDS_fifoEnabled);
    }
  }
}

/**
 * @brief Edge sensitive trigger mode example
 *
 * In this example, a data enable (DEN) trigger pulse is generated every 20 ms causing the
 * next sample to be captured. The captured samples are then counted.
 */
static void ISDS_startEdgeSensitiveTriggerExample()
{
  debugPrintln("Starting edge sensitive trigger mode...");

  /* Sampling rate of 208 Hz */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr208Hz);
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr208Hz);

  /* Set FIFO to operate at the same data rate as accelerometer and gyroscope */
  ISDS_setFifoOutputDataRate(&isds, ISDS_fifoOdr208Hz);

  /* Interrupt for FIFO threshold reached event on INT0 */
  ISDS_enableFifoThresholdINT0(&isds, ISDS_enable);

  /* Accelerometer low pass (LPF2) must be disabled for edge sensitive trigger mode */
  ISDS_enableAccLowPass(&isds, ISDS_disable);

  /* No decimation of accelerations and angular rates
   * (i.e. use the FIFO sample rate set above) */
  ISDS_setFifoAccDecimation(&isds, ISDS_fifoDecimationDisabled);
  ISDS_setFifoGyroDecimation(&isds, ISDS_fifoDecimationDisabled);

  /* Use data enable for both gyroscope and accelerometer */
  ISDS_extendDataEnableToAcc(&isds, ISDS_enable);

  /* Data enable signal should be active low */
  ISDS_setDataEnableActiveHigh(&isds, ISDS_disable);

  /* Level sensitive trigger mode */
  ISDS_setDataEnableTriggerMode(&isds, ISDS_dataEnableTriggerModeEdgeSensitiveTrigger);

  /* Enable FIFO continuous mode */
  ISDS_setFifoMode(&isds, ISDS_continuousMode);

  uint16_t fifoDataRaw[2048];
  uint32_t countRateSamples = 0;
  uint32_t countAccSamples = 0;
  uint32_t lastCountRateSamples = 0;
  uint32_t nextPrintTime = HAL_GetTick() + 1000;

  /* Interval at which the data enable signal is triggered (corresponds to 50 Hz) */
  uint32_t dataEnableSignalIntervalMs = 20;

  /* Time of next data enable trigger signal */
  uint32_t nextDataEnableTriggerTime = HAL_GetTick() + dataEnableSignalIntervalMs;

  /* Set data enable (DEN) signal initially high (i.e. inactive) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  while (1)
  {
    if (interrupt0Triggered == true)
    {
      /* FIFO buffer is filled up to threshold */
      interrupt0Triggered = false;

      /* Get FIFO status */
      ISDS_fifoStatus2_t status;
      uint16_t fillLevel, fifoPattern;
      ISDS_getFifoStatus(&isds, &status, &fillLevel, &fifoPattern);

      /* Count samples */
      for (uint16_t i = 0; i < fillLevel; i++)
      {
        if (WE_SUCCESS == ISDS_getFifoData(&isds, 1, fifoDataRaw))
        {
          /* Pattern [0,1,2] corresponds to angular rate, [3,4,5] corresponds to acceleration  */
          if (fifoPattern < 3)
          {
            countRateSamples++;
          }
          else
          {
            countAccSamples++;
          }
        }

        /* Get type of next sample */
        ISDS_getFifoPattern(&isds, &fifoPattern);
      }

      debugPrint(".");
    }

    uint32_t currentTime = HAL_GetTick();

    /* Trigger data enable signal at defined intervals */
    if (currentTime >= nextDataEnableTriggerTime)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
      nextDataEnableTriggerTime = currentTime + dataEnableSignalIntervalMs;
    }

    /* Print counter values every second */
    if (currentTime >= nextPrintTime)
    {
      nextPrintTime = currentTime + 1000;

      /* 3 values (axes) per sample */
      uint32_t samplesPerSecond = (countRateSamples - lastCountRateSamples) / 3;

      char msg[128];
      sprintf(msg, "Sample rate = %lu Hz, total counter angular rate = %lu, acceleration = %lu", samplesPerSecond, countRateSamples, countAccSamples);
      debugPrintln(msg);

      lastCountRateSamples = countRateSamples;
    }
  }
}

/**
 * @brief Level sensitive trigger mode example
 *
 * In this example, the data enable (DEN) signal is toggled at regular intervals.
 * The sensor is configured to mark the Z-axis angular rates with LSB=1 when
 * the data enable signal is active and LSB=0 when the signal is inactive.
 *
 * For each sample read from the FIFO, it is then determined whether it was
 * captured with DEN=active or DEN=inactive.
 */
static void ISDS_startLevelSensitiveTriggerExample()
{
  debugPrintln("Starting level sensitive trigger mode...");

  /* Sampling rate of 208 Hz */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr208Hz);
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr208Hz);

  /* Set FIFO to operate at the same data rate as accelerometer and gyroscope */
  ISDS_setFifoOutputDataRate(&isds, ISDS_fifoOdr208Hz);

  /* Interrupt for FIFO threshold reached event on INT0 */
  ISDS_enableFifoThresholdINT0(&isds, ISDS_enable);

  /* No decimation of accelerations and angular rates
   * (i.e. use the FIFO sample rate set above) */
  ISDS_setFifoAccDecimation(&isds, ISDS_fifoDecimationDisabled);
  ISDS_setFifoGyroDecimation(&isds, ISDS_fifoDecimationDisabled);

//  ISDS_extendDataEnableToAcc(&isds, ISDS_enable);

  /* Data enable signal should be active low */
  ISDS_setDataEnableActiveHigh(&isds, ISDS_disable);

  /* Level sensitive trigger mode */
  ISDS_setDataEnableTriggerMode(&isds, ISDS_dataEnableTriggerModeLevelSensitiveTrigger);

  /* Stamp LSB of angular rate Z values */
  ISDS_setDataEnableStampingSensor(&isds, ISDS_dataEnableStampingSensorGyro);
  ISDS_storeDataEnableValueInXAxisLSB(&isds, ISDS_disable);
  ISDS_storeDataEnableValueInYAxisLSB(&isds, ISDS_disable);
  ISDS_storeDataEnableValueInZAxisLSB(&isds, ISDS_enable);

  /* Enable DEN data ready interrupt */
//  ISDS_enableDataEnableDataReadyINT0(&isds, ISDS_enable);

  /* Enable FIFO continuous mode */
  ISDS_setFifoMode(&isds, ISDS_continuousMode);

  uint16_t fifoDataRaw[2048];
  uint32_t countDataEnable = 0;
  uint32_t countNotDataEnable = 0;
  uint32_t lastCountDataEnable = 0;
  uint32_t lastCountNotDataEnable = 0;
  uint32_t nextPrintTime = HAL_GetTick() + 1000;

  uint32_t dataEnableDuration = 50;
  uint32_t dataNotEnableDuration = 100;
  uint32_t nextDataEnableSwitchTime = HAL_GetTick() + dataEnableDuration;

  /* Set data enable (DEN) signal initially low (i.e. active) */
  bool dataEnable = true;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  while (1)
  {
    if (interrupt0Triggered == true)
    {
      /* FIFO buffer is filled up to threshold */
      interrupt0Triggered = false;

      /* Get FIFO status */
      ISDS_fifoStatus2_t status;
      uint16_t fillLevel, fifoPattern;
      ISDS_getFifoStatus(&isds, &status, &fillLevel, &fifoPattern);

      /* Note that we are always reading a multiple of the number of variables being
       * captured and the collection rate is identical for all data sets -
       * therefore fifoPattern should always be zero and the order of
       * the retrieved values should always be xRate,yRate,zRate,xAcc,yAcc,zAcc,...
       * Depending on the timing and the way the FIFO is read, fifoPattern might
       * be non-zero - in that case, we must use fifoPattern to determine the type
       * of samples read from the FIFO. In this example, we skip the first samples
       * if necessary, so that the first used sample is always xRate. */
      uint16_t patternOffset = (fifoPattern == 0) ? 0 : (6 - fifoPattern);

      /* Retrieve FIFO_THRESH samples from sensor */
      if (WE_SUCCESS == ISDS_getFifoData(&isds, FIFO_THRESH, fifoDataRaw))
      {
        uint16_t numSamples = (FIFO_THRESH - patternOffset) / 6;

        /* Count samples collected with DEN active - i.e. those with LSB of angular rate
         * Z (which is the third value of each sample) set to 1 */
        for (uint16_t i = 0; i < numSamples; i++)
        {
          uint16_t offset = i * 6 + patternOffset;
          if (0 != (fifoDataRaw[offset + 2] & 0x0001))
          {
            countDataEnable++;
          }
          else
          {
            countNotDataEnable++;
          }
        }
      }
      else
      {
        debugPrintln("**** ISDS_getFifoData() failed");
      }

      debugPrint(".");
    }

    uint32_t currentTime = HAL_GetTick();

    /* Toggle data enable signal at defined intervals */
    if (currentTime >= nextDataEnableSwitchTime)
    {
      dataEnable = !dataEnable;
      if (dataEnable)
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        nextDataEnableSwitchTime += dataEnableDuration;
      }
      else
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        nextDataEnableSwitchTime += dataNotEnableDuration;
      }
    }

    /* Print counter values every second. */
    if (currentTime >= nextPrintTime)
    {
      nextPrintTime = currentTime + 1000;

      uint32_t dataEnableSamplesPerSecond = (countDataEnable - lastCountDataEnable);
      uint32_t notDataEnableSamplesPerSecond = (countNotDataEnable - lastCountNotDataEnable);

      char msg[128];
      sprintf(msg, "Samples collected per second DEN = %lu, ~DEN = %lu", dataEnableSamplesPerSecond, notDataEnableSamplesPerSecond);
      debugPrintln("");
      debugPrintln(msg);

      lastCountDataEnable = countDataEnable;
      lastCountNotDataEnable = countNotDataEnable;
    }
  }
}

/**
 * @brief Level sensitive latched mode example
 *
 * In this example, a data enable (DEN) trigger pulse is generated every 20 ms causing the
 * next Z-axis angular rate sample to be marked with LSB=1 while all other samples are
 * marked with LSB=0.
 *
 * The captured samples are then counted.
 */
static void ISDS_startLevelSensitiveLatchedExample()
{
  debugPrintln("Starting level sensitive latched mode...");

  /* Sampling rate of 208 Hz */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr208Hz);
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr208Hz);

  /* Set FIFO to operate at the same data rate as accelerometer and gyroscope */
  ISDS_setFifoOutputDataRate(&isds, ISDS_fifoOdr208Hz);

  /* Interrupt for FIFO threshold reached event on INT0 */
  ISDS_enableFifoThresholdINT0(&isds, ISDS_enable);

  /* No decimation of accelerations and angular rates
   * (i.e. use the FIFO sample rate set above) */
  ISDS_setFifoAccDecimation(&isds, ISDS_fifoDecimationDisabled);
  ISDS_setFifoGyroDecimation(&isds, ISDS_fifoDecimationDisabled);

  /* Use data enable for both gyroscope and accelerometer */
  ISDS_extendDataEnableToAcc(&isds, ISDS_enable);

  /* Data enable signal should be active low */
  ISDS_setDataEnableActiveHigh(&isds, ISDS_disable);

  /* Level sensitive latched mode */
  ISDS_setDataEnableTriggerMode(&isds, ISDS_dataEnableTriggerModeLevelSensitiveLatched);

  /* Stamp LSB of angular rate Z values */
  ISDS_setDataEnableStampingSensor(&isds, ISDS_dataEnableStampingSensorGyro);
  ISDS_storeDataEnableValueInXAxisLSB(&isds, ISDS_disable);
  ISDS_storeDataEnableValueInYAxisLSB(&isds, ISDS_disable);
  ISDS_storeDataEnableValueInZAxisLSB(&isds, ISDS_enable);

  /* Enable FIFO continuous mode */
  ISDS_setFifoMode(&isds, ISDS_continuousMode);

  uint16_t fifoDataRaw[2048];
  uint32_t countDataEnableSamples = 0;
  uint32_t countNotDataEnableSamples = 0;
  uint32_t lastCountDataEnableSamples = 0;
  uint32_t lastCountNotDataEnableSamples = 0;
  uint32_t nextPrintTime = HAL_GetTick() + 1000;

  /* Interval at which the data enable signal is triggered (corresponds to 50 Hz) */
  uint32_t dataEnableSignalIntervalMs = 20;

  /* Time of next data enable trigger signal */
  uint32_t nextDataEnableTriggerTime = HAL_GetTick() + dataEnableSignalIntervalMs;

  /* Set data enable (DEN) signal initially high (i.e. inactive) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  while (1)
  {
    if (interrupt0Triggered == true)
    {
      /* FIFO buffer is filled up to threshold */
      interrupt0Triggered = false;

      /* Get FIFO status */
      ISDS_fifoStatus2_t status;
      uint16_t fillLevel, fifoPattern;
      ISDS_getFifoStatus(&isds, &status, &fillLevel, &fifoPattern);

      /* Note that we are always reading a multiple of the number of variables being
       * captured and the collection rate is identical for all data sets -
       * therefore fifoPattern should always be zero and the order of
       * the retrieved values should always be xRate,yRate,zRate,xAcc,yAcc,zAcc,...
       * Depending on the timing and the way the FIFO is read, fifoPattern might
       * be non-zero - in that case, we must use fifoPattern to determine the type
       * of samples read from the FIFO. In this example, we skip the first samples
       * if necessary, so that the first used sample is always xRate. */
      uint16_t patternOffset = (fifoPattern == 0) ? 0 : (6 - fifoPattern);

      /* Retrieve FIFO_THRESH samples from sensor */
      if (WE_SUCCESS == ISDS_getFifoData(&isds, FIFO_THRESH, fifoDataRaw))
      {
        uint16_t numSamples = (FIFO_THRESH - patternOffset) / 6;

        /* Count samples collected after data enable trigger signals */
        for (uint16_t i = 0; i < numSamples; i++)
        {
          uint16_t offset = i * 6 + patternOffset;
          /* LSB of angular rate Z (the third value in each sample) is 1 for
           * first sample after data enable trigger */
          if (0 != (fifoDataRaw[offset + 2] & 0x001))
          {
            countDataEnableSamples++;
          }
          else
          {
            countNotDataEnableSamples++;
          }
        }
        debugPrint(".");
      }
      else
      {
        debugPrintln("**** ISDS_getFifoData() failed");
      }
    }

    uint32_t currentTime = HAL_GetTick();

    /* Trigger data enable signal at defined intervals */
    if (currentTime >= nextDataEnableTriggerTime)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
      nextDataEnableTriggerTime = currentTime + dataEnableSignalIntervalMs;
    }

    /* Print sample rate every second.
     * Data enable sample rate should be around 50 Hz (one trigger signal every 20 ms),
     * sample rate of remaining samples should be around 158 Hz (= ODR of 208 Hz - 50 Hz) */
    if (currentTime >= nextPrintTime)
    {
      nextPrintTime = currentTime + 1000;

      uint32_t dataEnableSamplesPerSecond = (countDataEnableSamples - lastCountDataEnableSamples);
      uint32_t notDataEnableSamplesPerSecond = (countNotDataEnableSamples - lastCountNotDataEnableSamples);

      char msg[128];
      sprintf(msg, "Samples collected per second DEN = %lu, ~DEN = %lu", dataEnableSamplesPerSecond, notDataEnableSamplesPerSecond);
      debugPrintln(msg);

      lastCountDataEnableSamples = countDataEnableSamples;
      lastCountNotDataEnableSamples = countNotDataEnableSamples;
    }
  }
}

/**
 * @brief Level sensitive FIFO mode example
 *
 * In this example, the data enable (DEN) signal is toggled at regular intervals.
 * Samples are added to the FIFO only if the data enable signal is active.
 * The Z-axis angular rate is marked with LSB=1 when the sample is from an
 * even DEN block and with LSB=0 when it is from an odd block.
 *
 * The captured samples are then counted.
 */
static void ISDS_startLevelSensitiveFifoExample()
{
  debugPrintln("Starting level sensitive FIFO mode...");

  /* Sampling rate of 208 Hz */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr208Hz);
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr208Hz);

  /* Set FIFO to operate at the same data rate as accelerometer and gyroscope */
  ISDS_setFifoOutputDataRate(&isds, ISDS_fifoOdr208Hz);

  /* Interrupt for FIFO threshold reached event on INT0 */
  ISDS_enableFifoThresholdINT0(&isds, ISDS_enable);

  /* No decimation of accelerations and angular rates
   * (i.e. use the FIFO sample rate set above) */
  ISDS_setFifoAccDecimation(&isds, ISDS_fifoDecimationDisabled);
  ISDS_setFifoGyroDecimation(&isds, ISDS_fifoDecimationDisabled);

  /* Data enable signal should be active low */
  ISDS_setDataEnableActiveHigh(&isds, ISDS_disable);

  /* Level sensitive trigger mode */
  ISDS_setDataEnableTriggerMode(&isds, ISDS_dataEnableTriggerModeLevelSensitiveFifo);

  /* Stamp LSB of angular rate Z values */
  ISDS_setDataEnableStampingSensor(&isds, ISDS_dataEnableStampingSensorGyro);
  ISDS_storeDataEnableValueInXAxisLSB(&isds, ISDS_disable);
  ISDS_storeDataEnableValueInYAxisLSB(&isds, ISDS_disable);
  ISDS_storeDataEnableValueInZAxisLSB(&isds, ISDS_enable);

  /* Enable FIFO continuous mode */
  ISDS_setFifoMode(&isds, ISDS_continuousMode);

  uint16_t fifoDataRaw[2048];
  uint32_t countDataEnableOdd = 0;
  uint32_t countDataEnableEven = 0;
  uint32_t lastCountDataEnableOdd = 0;
  uint32_t lastCountDataEnableEven = 0;
  uint32_t printIntervalMs = 5000;
  uint32_t nextPrintTime = HAL_GetTick() + printIntervalMs;

  uint32_t dataEnableDuration = 20;
  uint32_t dataNotEnableDuration = 80;
  uint32_t nextDataEnableSwitchTime = HAL_GetTick() + dataEnableDuration;

  /* Set data enable (DEN) signal initially low (i.e. active) */
  bool dataEnable = true;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  while (1)
  {
    if (interrupt0Triggered == true)
    {
      /* FIFO buffer is filled up to threshold */
      interrupt0Triggered = false;

      /* Get FIFO status */
      ISDS_fifoStatus2_t status;
      uint16_t fillLevel, fifoPattern;
      ISDS_getFifoStatus(&isds, &status, &fillLevel, &fifoPattern);

      /* Note that we are always reading a multiple of the number of variables being
       * captured and the collection rate is identical for all data sets -
       * therefore fifoPattern should always be zero and the order of
       * the retrieved values should always be xRate,yRate,zRate,xAcc,yAcc,zAcc,...
       * Depending on the timing and the way the FIFO is read, fifoPattern might
       * be non-zero - in that case, we must use fifoPattern to determine the type
       * of samples read from the FIFO. In this example, we skip the first samples
       * if necessary, so that the first used sample is always xRate. */
      uint16_t patternOffset = (fifoPattern == 0) ? 0 : (6 - fifoPattern);

      /* Retrieve FIFO_THRESH samples from sensor */
      if (WE_SUCCESS == ISDS_getFifoData(&isds, FIFO_THRESH, fifoDataRaw))
      {
        uint16_t numSamples = (FIFO_THRESH - patternOffset) / 6;

        /* Count samples collected per DEN active window - i.e. those with LSB of angular
         * rate Z (third value of each sample) set to 1 are from even blocks, those with
         * LSB=0 are from odd blocks */
        for (uint16_t i = 0; i < numSamples; i++)
        {
          uint16_t offset = i * 6 + patternOffset;
          if (0 != (fifoDataRaw[offset + 2] & 0x0001))
          {
            countDataEnableEven++;
          }
          else
          {
            countDataEnableOdd++;
          }
        }
      }
      else
      {
        debugPrintln("**** ISDS_getFifoData() failed");
      }

      debugPrint(".");
    }

    uint32_t currentTime = HAL_GetTick();

    /* Toggle data enable signal at defined intervals */
    if (currentTime >= nextDataEnableSwitchTime)
    {
      dataEnable = !dataEnable;
      if (dataEnable)
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        nextDataEnableSwitchTime += dataEnableDuration;
      }
      else
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        nextDataEnableSwitchTime += dataNotEnableDuration;
      }
    }

    /* Print sample rate every second.
     * Sample rate of odd and even blocks should be both around 20 Hz (i.e. DEN signal
     * is active about 20% of the time, split evenly among odd and even blocks - so 10%
     * of ODR = 208 Hz) */
    if (currentTime >= nextPrintTime)
    {
      nextPrintTime = currentTime + printIntervalMs;

      uint32_t oddSamplesPerSecond = ((countDataEnableOdd - lastCountDataEnableOdd) * 1000) / printIntervalMs;
      uint32_t evenSamplesPerSecond = ((countDataEnableEven - lastCountDataEnableEven) * 1000) / printIntervalMs;

      char msg[128];
      sprintf(msg, "Samples collected per second odd = %lu, even = %lu", oddSamplesPerSecond, evenSamplesPerSecond);
      debugPrintln(msg);

      lastCountDataEnableOdd = countDataEnableOdd;
      lastCountDataEnableEven = countDataEnableEven;
    }
  }
}

#if defined(STM32L432xx)
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_Pin) == GPIO_PIN_SET)
  {
    HAL_GPIO_EXTI_Rising_Callback(GPIO_Pin);
  }
  else
  {
    HAL_GPIO_EXTI_Falling_Callback(GPIO_Pin);
  }
}
#endif

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  /* Interrupt source depends on example mode. */

  if (GPIO_Pin == GPIO_PIN_0)
  {
    /* Trigger event handling in main function. */
    interrupt0Triggered = true;

    /* Toggle LED */
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  UNUSED(GPIO_Pin);
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
