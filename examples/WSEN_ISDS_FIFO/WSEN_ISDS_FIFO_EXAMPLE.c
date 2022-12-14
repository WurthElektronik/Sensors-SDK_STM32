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
 * @brief WSEN_ISDS FIFO example.
 *
 * Example for the ISDS accelerometer/gyroscope demonstrating usage of the FIFO buffer (FIFO mode,
 * continuous-to-FIFO mode, bypass-to-continuous mode and continuous mode)
 */

#include "WSEN_ISDS_FIFO_EXAMPLE.h"

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
#define FIFO_THRESH (24 * 6)

/* Sensor interface configuration */
static WE_sensorInterface_t isds;

/* Use the following variable to switch among the available example modes. */
static ISDS_fifoMode_t isdsFifoExampleMode = ISDS_fifoEnabled;

/* Is set to true when an interrupt has been triggered */
static bool interrupt0Triggered = false;
static bool interrupt1Triggered = false;

/* Sensor initialization function */
static bool ISDS_init(void);

/* Functions containing main loops for the available example modes */
static void ISDS_startFifoMode();
static void ISDS_startContinuousToFifoMode();
static void ISDS_startBypassToContinuousMode();
static void ISDS_startContinuousMode();

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);
static void debugPrintAcceleration_int(char axis[], int32_t accMg);
static void debugPrintAngularRate_int(char axis[], int32_t rateMdps);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_isdsFifoExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the \"FIFO\" example program for the ISDS sensor.");
  debugPrintln("Note that for this example to work, the following pin/interrupt configuration is required:");
  debugPrintln("* INT_0 to PA0, rising edge interrupt only");
  debugPrintln("* INT_1 to PA1, rising edge interrupt only");

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
void WE_isdsFifoExampleLoop()
{
  switch (isdsFifoExampleMode)
  {
  case ISDS_bypassMode:
    /* FIFO is disabled. Stop. */
    debugPrintln("**** FIFO example mode set to bypass. STOP ****");
    HAL_Delay(5);
    while (1);
    break;

  case ISDS_fifoEnabled:
    ISDS_startFifoMode();
    break;

  case ISDS_continuousMode:
    ISDS_startContinuousMode();
    break;

  case ISDS_continuousToFifo:
    ISDS_startContinuousToFifoMode();
    break;

  case ISDS_bypassToContinuous:
    ISDS_startBypassToContinuousMode();
    break;
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

  /* Set FIFO initially off */
  ISDS_setFifoMode(&isds, ISDS_bypassMode);

  /* Enable block data update */
  ISDS_enableBlockDataUpdate(&isds, ISDS_enable);

  /* Enable address auto increment */
  ISDS_enableAutoIncrement(&isds, ISDS_enable);

  /* Sampling rate of 208 Hz */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr208Hz);
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr208Hz);

  /* Set FIFO to operate at the same data rate as accelerometer and gyroscope */
  ISDS_setFifoOutputDataRate(&isds, ISDS_fifoOdr208Hz);

  /* Full scale +-2 g and +-2000 dps */
  ISDS_setAccFullScale(&isds, ISDS_accFullScaleTwoG);
  ISDS_setGyroFullScale(&isds, ISDS_gyroFullScale2000dps);

  /* Set FIFO fill threshold */
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
 * @brief FIFO mode example: Samples are collected until the FIFO buffer is full.
 * After the corresponding interrupt has been triggered, the collected data is
 * read and data collection is manually restarted by re-enabling FIFO mode.
 */
static void ISDS_startFifoMode()
{
  debugPrintln("Starting FIFO mode...");

  /* Interrupts for FIFO buffer full and overrun events on INT1 */
  ISDS_enableFifoOverrunINT1(&isds, ISDS_enable);
  ISDS_enableFifoFullINT1(&isds, ISDS_enable);

  /* No decimation of accelerations and angular rates
   * (i.e. use the FIFO sample rate set in init function) */
  ISDS_setFifoAccDecimation(&isds, ISDS_fifoDecimationDisabled);
  ISDS_setFifoGyroDecimation(&isds, ISDS_fifoDecimationDisabled);

  /* Stop when fill threshold is reached */
//  ISDS_enableFifoStopOnThreshold(&isds, ISDS_enable);

  /* Enable FIFO */
  ISDS_setFifoMode(&isds, ISDS_fifoEnabled);

  uint16_t fifoDataRaw[2048];
  int32_t xRateAvg = 0;
  int32_t yRateAvg = 0;
  int32_t zRateAvg = 0;
  int32_t xAccAvg = 0;
  int32_t yAccAvg = 0;
  int32_t zAccAvg = 0;
  uint32_t nextPrintTime = 0;
  while (1)
  {
    if (interrupt0Triggered == true)
    {
      interrupt0Triggered = false;
    }

    if (interrupt1Triggered == true)
    {
      interrupt1Triggered = false;

      /* Get FIFO status */
      ISDS_fifoStatus2_t status;
      uint16_t fillLevel, fifoPattern;
      ISDS_getFifoStatus(&isds, &status, &fillLevel, &fifoPattern);

      /* Note that we are always reading the complete buffer starting
       * from the beginning (or the buffer up to the threshold, if enabled) -
       * therefore fifoPattern should always be zero and the order of
       * the retrieved values should always be xRate,yRate,zRate,xAcc,yAcc,zAcc,...
       * Depending on the timing and the way the FIFO is read, fifoPattern might
       * be non-zero - in that case, we must use fifoPattern to determine the type
       * of samples read from the FIFO. In this example, we skip the first samples
       * if necessary, so that the first used sample is always xRate. */
      uint16_t patternOffset = (fifoPattern == 0) ? 0 : (6 - fifoPattern);

      char msg[256];
      sprintf(msg, "FIFO interrupt. "
          "Fill level=%d, pattern=%d, empty=%d, full=%d, overrun=%d, thresh=%d",
          fillLevel,
          fifoPattern,
          status.fifoEmptyState,
          status.fifoFullSmartState,
          status.fifoOverrunState,
          status.fifoThresholdState);
      debugPrintln(msg);

      /* Read contents of complete FIFO buffer */
      if (WE_SUCCESS == ISDS_getFifoData(&isds, fillLevel, fifoDataRaw))
      {
        xRateAvg = 0;
        yRateAvg = 0;
        zRateAvg = 0;
        xAccAvg = 0;
        yAccAvg = 0;
        zAccAvg = 0;

        uint16_t numSamples = (fillLevel - patternOffset) / 6;

        for (uint16_t i = 0; i < numSamples; i++)
        {
          uint16_t offset = i * 6 + patternOffset;
          xRateAvg += *(int16_t*) &fifoDataRaw[offset];
          yRateAvg += *(int16_t*) &fifoDataRaw[offset + 1];
          zRateAvg += *(int16_t*) &fifoDataRaw[offset + 2];
          xAccAvg += *(int16_t*) &fifoDataRaw[offset + 3];
          yAccAvg += *(int16_t*) &fifoDataRaw[offset + 4];
          zAccAvg += *(int16_t*) &fifoDataRaw[offset + 5];
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
      }
      else
      {
        debugPrintln("**** ISDS_getFifoData() failed");
      }

      /* Restart data collection by first setting bypass mode and then re-enabling FIFO mode */
      ISDS_setFifoMode(&isds, ISDS_bypassMode);
      ISDS_setFifoMode(&isds, ISDS_fifoEnabled);
    }

    /* Print last accelerations and angular rates (averaged) every second. */
    uint32_t currentTime = HAL_GetTick();
    if (currentTime >= nextPrintTime)
    {
      nextPrintTime = currentTime + 1000;

      debugPrintln("");
      debugPrintAcceleration_int("X", xAccAvg);
      debugPrintAcceleration_int("Y", yAccAvg);
      debugPrintAcceleration_int("Z", zAccAvg);
      debugPrintAngularRate_int("X", xRateAvg);
      debugPrintAngularRate_int("Y", yRateAvg);
      debugPrintAngularRate_int("Z", zRateAvg);
    }
  }
}

/**
 * @brief Continuous FIFO mode example: Samples are collected continuously.
 * When the FIFO buffer is filled up to FIFO_THRESH, the collected data is read
 * thus making space for new samples.
 */
static void ISDS_startContinuousMode()
{
  debugPrintln("Starting continuous mode...");

  /* Interrupt when FIFO buffer is filled up to threshold on INT0 */
  ISDS_enableFifoThresholdINT0(&isds, ISDS_enable);

  /* Interrupts for FIFO buffer full and overrun events on INT1 */
  ISDS_enableFifoOverrunINT1(&isds, ISDS_enable);
  ISDS_enableFifoFullINT1(&isds, ISDS_enable);

  /* No decimation of accelerations and angular rates
   * (i.e. use the FIFO sample rate set in init function) */
  ISDS_setFifoAccDecimation(&isds, ISDS_fifoDecimationDisabled);
  ISDS_setFifoGyroDecimation(&isds, ISDS_fifoDecimationDisabled);

  /* Enable continuous mode */
  ISDS_setFifoMode(&isds, ISDS_continuousMode);

  uint16_t fifoDataRaw[FIFO_THRESH];
  int32_t xRateAvg = 0;
  int32_t yRateAvg = 0;
  int32_t zRateAvg = 0;
  int32_t xAccAvg = 0;
  int32_t yAccAvg = 0;
  int32_t zAccAvg = 0;
  uint32_t nextPrintTime = 0;
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
        xRateAvg = 0;
        yRateAvg = 0;
        zRateAvg = 0;
        xAccAvg = 0;
        yAccAvg = 0;
        zAccAvg = 0;

        uint16_t numSamples = (FIFO_THRESH - patternOffset) / 6;

        for (uint16_t i = 0; i < numSamples; i++)
        {
          uint16_t offset = i * 6 + patternOffset;
          xRateAvg += *(int16_t*) &fifoDataRaw[offset];
          yRateAvg += *(int16_t*) &fifoDataRaw[offset + 1];
          zRateAvg += *(int16_t*) &fifoDataRaw[offset + 2];
          xAccAvg += *(int16_t*) &fifoDataRaw[offset + 3];
          yAccAvg += *(int16_t*) &fifoDataRaw[offset + 4];
          zAccAvg += *(int16_t*) &fifoDataRaw[offset + 5];
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

        debugPrint(".");
      }
      else
      {
        debugPrintln("**** ISDS_getFifoData() failed");
      }

      debugPrint(".");
    }

    if (interrupt1Triggered == true)
    {
      /* FIFO buffer full or overrun - this shouldn't happen, if samples are read fast enough */

      interrupt1Triggered = false;

      ISDS_fifoStatus2_t status;
      uint16_t fillLevel, fifoPattern;
      ISDS_getFifoStatus(&isds, &status, &fillLevel, &fifoPattern);

      char buffer[5];
      sprintf(buffer, "%d", fillLevel);

      if (status.fifoOverrunState)
      {
        debugPrintln("FIFO buffer overrun");
      }
      else if (status.fifoFullSmartState)
      {
        debugPrint("FIFO buffer is full (contains ");
        debugPrint(buffer);
        debugPrintln(" samples)");
      }

      /* Restart data collection by first setting bypass mode and then re-enabling FIFO mode */
      ISDS_setFifoMode(&isds, ISDS_bypassMode);
      ISDS_setFifoMode(&isds, ISDS_continuousMode);
    }

    /* Print last accelerations and angular rates (averaged) every second. */
    uint32_t currentTime = HAL_GetTick();
    if (currentTime >= nextPrintTime)
    {
      nextPrintTime = currentTime + 1000;

      debugPrintln("");
      debugPrintAcceleration_int("X", xAccAvg);
      debugPrintAcceleration_int("Y", yAccAvg);
      debugPrintAcceleration_int("Z", zAccAvg);
      debugPrintAngularRate_int("X", xRateAvg);
      debugPrintAngularRate_int("Y", yRateAvg);
      debugPrintAngularRate_int("Z", zRateAvg);
    }
  }
}

/**
 * @brief Continuous-to-FIFO mode example: Samples are collected continuously until a 6D orientation
 * change interrupt has been triggered, after which the sensor will automatically stop collecting
 * data (as soon as the buffer is full).
 * The program then waits until the FIFO buffer is full, reads the collected data and re-enables
 * continuous-to-FIFO mode.
 */
static void ISDS_startContinuousToFifoMode()
{
  debugPrintln("Starting continuous-to-FIFO mode...");

  /* Route the FIFO threshold, fill and overrun interrupts to INT0 */
  ISDS_enableFifoThresholdINT0(&isds, ISDS_enable);
  ISDS_enableFifoFullINT0(&isds, ISDS_enable);
  ISDS_enableFifoOverrunINT0(&isds, ISDS_enable);

  /* Configure sensor for 6D orientation change event detection - see example
   * WSEN_ISDS_ORIENTATION for details on the used parameters. */
  ISDS_set6DThreshold(&isds, ISDS_sixDThresholdSixtyDeg);
  ISDS_enable6dLowPass(&isds, ISDS_enable);
  /* Interrupt for 6D orientation change on INT1 */
  ISDS_enable6dINT1(&isds, ISDS_enable);

  /* No decimation of accelerations and angular rates
   * (i.e. use the FIFO sample rate set in init function) */
  ISDS_setFifoAccDecimation(&isds, ISDS_fifoDecimationDisabled);
  ISDS_setFifoGyroDecimation(&isds, ISDS_fifoDecimationDisabled);

  /* Enable continuous-to-FIFO mode */
  ISDS_setFifoMode(&isds, ISDS_continuousToFifo);

  uint16_t fifoDataRaw[2048];
  int32_t xRateAvg = 0;
  int32_t yRateAvg = 0;
  int32_t zRateAvg = 0;
  int32_t xAccAvg = 0;
  int32_t yAccAvg = 0;
  int32_t zAccAvg = 0;
  bool orientationChanged = false;
  while (1)
  {
    if (interrupt1Triggered == true)
    {
      /* In this example, interrupt 1 is triggered when the device's 6D orientation has changed */

      interrupt1Triggered = false;

      debugPrintln("");
      debugPrintln("Orientation changed");

      orientationChanged = true;
    }

    if (interrupt0Triggered == true)
    {
      /* In this example, interrupt 0 is triggered if the FIFO buffer is either
       * filled up to threshold, is full or in case of an overrun */

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

      if (status.fifoOverrunState)
      {
        debugPrintln("FIFO overrun");
      }

      if (orientationChanged == true)
      {
        /* 6D orientation change event has been triggered - wait for FIFO buffer to be full */
        if (status.fifoFullSmartState)
        {
          interrupt0Triggered = false;
          orientationChanged = false;

          /* Read contents of complete FIFO buffer, compute and print average values */
          if (WE_SUCCESS == ISDS_getFifoData(&isds, fillLevel, fifoDataRaw))
          {
            xRateAvg = 0;
            yRateAvg = 0;
            zRateAvg = 0;
            xAccAvg = 0;
            yAccAvg = 0;
            zAccAvg = 0;

            uint16_t numSamples = (fillLevel - patternOffset) / 6;

            for (uint16_t i = 0; i < numSamples; i++)
            {
              uint16_t offset = i * 6 + patternOffset;
              xRateAvg += *(int16_t*) &fifoDataRaw[offset];
              yRateAvg += *(int16_t*) &fifoDataRaw[offset + 1];
              zRateAvg += *(int16_t*) &fifoDataRaw[offset + 2];
              xAccAvg += *(int16_t*) &fifoDataRaw[offset + 3];
              yAccAvg += *(int16_t*) &fifoDataRaw[offset + 4];
              zAccAvg += *(int16_t*) &fifoDataRaw[offset + 5];
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

          /* Restart data collection by first setting bypass mode and then re-enabling continuous to FIFO mode */
          ISDS_setFifoMode(&isds, ISDS_bypassMode);
          ISDS_setFifoMode(&isds, ISDS_continuousToFifo);
        }
      }
      else if (status.fifoThresholdState)
      {
        /* No orientation change event happened - this is probably an interrupt signaling
         * that the FIFO buffer has at least FIFO_THRESH elements. */

        interrupt0Triggered = false;

        debugPrint(".");

        /* Must read acceleration values so that there is no overrun while in continuous mode */
        ISDS_getFifoData(&isds, FIFO_THRESH, fifoDataRaw);
      }
    }
  }
}

/**
 * @brief Bypass-to-continuous mode example: Samples are not collected unless a wake-up interrupt
 * has been triggered. After such an interrupt, the sensor automatically switches to continuous mode.
 * The program then processes samples for 5 s and then re-enables bypass-to-continuous mode.
 */
static void ISDS_startBypassToContinuousMode()
{
  debugPrintln("Starting bypass-to-continuous mode...");

  /* Interrupt when FIFO buffer is filled up to threshold on INT0 */
  ISDS_enableFifoThresholdINT0(&isds, ISDS_enable);

  /* Set wake-up interrupt parameters - see example
   * WSEN_ISDS_WAKE_UP for details on the used parameters. */
  ISDS_setWakeUpDuration(&isds, 1);
  ISDS_setWakeUpThreshold(&isds, 1);
  /* Use slope filter for wake-up event detection */
  ISDS_setActivityFilter(&isds, ISDS_activityFilterSlope);
  /* Depending on the parameters set above (threshold, duration), it might be
   * necessary to wait until the first sample has been collected, before activating
   * the wake-up interrupt. This prevents an erroneous wake-up event on startup.
   * Alternatively, the first wake-up interrupt could be ignored. */
  HAL_Delay(5);
  /* Interrupt for wake-up event on INT1 */
  ISDS_enableWakeUpINT1(&isds, ISDS_enable);

  /* No decimation of accelerations and angular rates
   * (i.e. use the FIFO sample rate set in init function) */
  ISDS_setFifoAccDecimation(&isds, ISDS_fifoDecimationDisabled);
  ISDS_setFifoGyroDecimation(&isds, ISDS_fifoDecimationDisabled);

  /* Enable bypass-to-continuous mode */
  ISDS_setFifoMode(&isds, ISDS_bypassToContinuous);

  uint16_t fifoDataRaw[FIFO_THRESH];
  int32_t xRateAvg = 0;
  int32_t yRateAvg = 0;
  int32_t zRateAvg = 0;
  int32_t xAccAvg = 0;
  int32_t yAccAvg = 0;
  int32_t zAccAvg = 0;
  uint32_t timeToSleep = 0;
  while (1)
  {
    if (interrupt1Triggered == true)
    {
      /* In this example, interrupt 1 is triggered when the device has woken up */

      interrupt1Triggered = false;

      debugPrintln("");
      debugPrintln("Waking up...");

      /* Collect data for 5 s, then return to bypass-to-continuous mode */
      timeToSleep = HAL_GetTick() + 5000;
    }

    if (interrupt0Triggered == true)
    {
      /* In this example, interrupt 0 is triggered if the FIFO buffer is
       * filled up to the configured threshold */

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
        xRateAvg = 0;
        yRateAvg = 0;
        zRateAvg = 0;
        xAccAvg = 0;
        yAccAvg = 0;
        zAccAvg = 0;

        uint16_t numSamples = (FIFO_THRESH - patternOffset) / 6;

        for (uint16_t i = 0; i < numSamples; i++)
        {
          uint16_t offset = i * 6 + patternOffset;
          xRateAvg += *(int16_t*) &fifoDataRaw[offset];
          yRateAvg += *(int16_t*) &fifoDataRaw[offset + 1];
          zRateAvg += *(int16_t*) &fifoDataRaw[offset + 2];
          xAccAvg += *(int16_t*) &fifoDataRaw[offset + 3];
          yAccAvg += *(int16_t*) &fifoDataRaw[offset + 4];
          zAccAvg += *(int16_t*) &fifoDataRaw[offset + 5];
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

        debugPrint(".");
      }
      else
      {
        debugPrintln("**** ISDS_getFifoData() failed");
      }

      if (HAL_GetTick() >= timeToSleep)
      {
        /* Print last accelerations and angular rates (averaged) */
        debugPrintln("");
        debugPrintAcceleration_int("X", xAccAvg);
        debugPrintAcceleration_int("Y", yAccAvg);
        debugPrintAcceleration_int("Z", zAccAvg);
        debugPrintAngularRate_int("X", xRateAvg);
        debugPrintAngularRate_int("Y", yRateAvg);
        debugPrintAngularRate_int("Z", zRateAvg);
        debugPrintln("Going to sleep...");

        /* Return to bypass-to-continuous mode */
        ISDS_setFifoMode(&isds, ISDS_bypassMode);
        ISDS_setFifoMode(&isds, ISDS_bypassToContinuous);
      }
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
  else if (GPIO_Pin == GPIO_PIN_1)
  {
    /* Trigger event handling in main function. */
    interrupt1Triggered = true;
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

