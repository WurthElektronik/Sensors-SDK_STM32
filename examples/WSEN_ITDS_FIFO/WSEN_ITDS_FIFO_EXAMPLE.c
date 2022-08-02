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
 * @brief WSEN_ITDS FIFO example.
 *
 * Example for the ITDS accelerometer demonstrating usage of the FIFO buffer (FIFO mode,
 * continuous-to-FIFO mode, bypass-to-continuous mode and continuous mode)
 */

#include "WSEN_ITDS_FIFO_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include <platform.h>

#include "../SensorsSDK/WSEN_ITDS_2533020201601/WSEN_ITDS_2533020201601.h"


/* FIFO fill threshold - an event is generated when the FIFO buffer is filled up to this amount. */
#define FIFO_THRESH 12

/* Use the following variable to switch among the available example modes. */
static ITDS_FifoMode_t itdsFifoExampleMode = ITDS_fifoEnabled;

/* Sensor interface configuration */
static WE_sensorInterface_t itds;

/* Is set to true when an interrupt has been triggered */
static bool interrupt0Triggered = false;
static bool interrupt1Triggered = false;

/* Sensor initialization function */
static bool ITDS_init(void);

/* Functions containing main loops for the available example modes. */
void ITDS_startFifoMode();
void ITDS_startContinuousToFifoMode();
void ITDS_startBypassToContinuousMode();
void ITDS_startContinuousMode();

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);
static void debugPrintAcceleration_int(char axis[], int32_t accMg);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_itdsFifoExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the \"FIFO\" example program for the ITDS sensor.");
  debugPrintln("Note that for this example to work, the following pin/interrupt configuration is required:");
  debugPrintln("* INT_0 to PA0, rising edge interrupt only");
  debugPrintln("* INT_1 to PA1, rising edge interrupt only");

  /* init ITDS */
  if (false == ITDS_init())
  {
    debugPrintln("**** ITDS_Init() error. STOP ****");
    HAL_Delay(5);
    while(1);
  }
}

/**
 * @brief Example main loop code.
 * Call this function in main loop (infinite loop).
 */
void WE_itdsFifoExampleLoop()
{
  switch (itdsFifoExampleMode)
  {
  case ITDS_bypassMode:
    /* FIFO is disabled. Stop. */
    debugPrintln("**** FIFO example mode set to bypass. STOP ****");
    HAL_Delay(5);
    while(1);
    break;

  case ITDS_fifoEnabled:
    ITDS_startFifoMode();
    break;

  case ITDS_continuousMode:
    ITDS_startContinuousMode();
    break;

  case ITDS_continuousToFifo:
    ITDS_startContinuousToFifoMode();
    break;

  case ITDS_bypassToContinuous:
    ITDS_startBypassToContinuousMode();
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

  /* Set FIFO mode depending on private variable defined at the beginning of this file. */
  ITDS_setFifoMode(&itds, itdsFifoExampleMode);

  /* Set FIFO fill threshold */
  ITDS_setFifoThreshold(&itds, FIFO_THRESH);

  /* Interrupts are active high */
  ITDS_setInterruptActiveLevel(&itds, ITDS_activeHigh);

  /* Interrupts are push-pull */
  ITDS_setInterruptPinType(&itds, ITDS_pushPull);

  /* Latched mode disabled (interrupt signal is automatically reset) */
  ITDS_enableLatchedInterrupt(&itds, ITDS_disable);

  /* Enable interrupts */
  ITDS_enableInterrupts(&itds, ITDS_enable);

  return true;
}

/**
 * @brief FIFO mode example: Samples are collected until the FIFO buffer is full.
 * After the corresponding interrupt has been triggered, the collected data is
 * read and data collection is manually restarted by re-enabling FIFO mode.
 */
void ITDS_startFifoMode()
{
  debugPrintln("Starting FIFO mode...");

  /* Interrupts for FIFO buffer full and overrun events on INT1 */
  ITDS_enableFifoOverrunIntINT1(&itds, ITDS_enable);
  ITDS_enableFifoFullINT1(&itds, ITDS_enable);

  int16_t xRawAcc[32];
  int16_t yRawAcc[32];
  int16_t zRawAcc[32];
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

      /* Read contents of complete FIFO buffer (32 samples) */
      if (WE_SUCCESS == ITDS_getRawAccelerations(&itds, 32, xRawAcc, yRawAcc, zRawAcc))
      {
        xAccAvg = 0;
        yAccAvg = 0;
        zAccAvg = 0;

        for (uint8_t i = 0; i < 32; i++)
        {
          xAccAvg += (int32_t) xRawAcc[i];
          yAccAvg += (int32_t) yRawAcc[i];
          zAccAvg += (int32_t) zRawAcc[i];
        }

        xAccAvg /= 32;
        yAccAvg /= 32;
        zAccAvg /= 32;

        xAccAvg = ITDS_convertAccelerationFs16g_int(xAccAvg);
        yAccAvg = ITDS_convertAccelerationFs16g_int(yAccAvg);
        zAccAvg = ITDS_convertAccelerationFs16g_int(zAccAvg);

        /* Restart data collection by first setting bypass mode and then re-enabling FIFO mode */
        ITDS_setFifoMode(&itds, ITDS_bypassMode);
        ITDS_setFifoMode(&itds, ITDS_fifoEnabled);

        debugPrint(".");
      }
      else
      {
        debugPrintln("**** ITDS_getRawAccelerations() failed");
      }
    }


    /* Print last acceleration values (averaged) every second. */
    uint32_t currentTime = HAL_GetTick();
    if (currentTime >= nextPrintTime)
    {
      nextPrintTime = currentTime + 1000;

      debugPrintln("");
      debugPrintAcceleration_int("X", xAccAvg);
      debugPrintAcceleration_int("Y", yAccAvg);
      debugPrintAcceleration_int("Z", zAccAvg);
    }
  }
}

/**
 * @brief Continuous FIFO mode example: Samples are collected continuously.
 * When the FIFO buffer is filled up to FIFO_THRESH, the collected data is read
 * thus making space for new samples.
 */
void ITDS_startContinuousMode()
{
  debugPrintln("Starting continuous mode...");

  /* Interrupt when FIFO buffer is filled up to threshold on INT0 */
  ITDS_enableFifoThresholdINT0(&itds, ITDS_enable);

  /* Interrupts for FIFO buffer full and overrun events on INT1 */
  ITDS_enableFifoOverrunIntINT1(&itds, ITDS_enable);
  ITDS_enableFifoFullINT1(&itds, ITDS_enable);

  int16_t xRawAcc[FIFO_THRESH];
  int16_t yRawAcc[FIFO_THRESH];
  int16_t zRawAcc[FIFO_THRESH];
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

      /* Retrieve FIFO_THRESH samples from sensor */
      if (WE_SUCCESS == ITDS_getRawAccelerations(&itds, FIFO_THRESH, xRawAcc, yRawAcc, zRawAcc))
      {
        xAccAvg = 0;
        yAccAvg = 0;
        zAccAvg = 0;

        for (uint8_t i = 0; i < FIFO_THRESH; i++)
        {
          xAccAvg += (int32_t) xRawAcc[i];
          yAccAvg += (int32_t) yRawAcc[i];
          zAccAvg += (int32_t) zRawAcc[i];
        }

        xAccAvg /= FIFO_THRESH;
        yAccAvg /= FIFO_THRESH;
        zAccAvg /= FIFO_THRESH;

        xAccAvg = ITDS_convertAccelerationFs16g_int(xAccAvg);
        yAccAvg = ITDS_convertAccelerationFs16g_int(yAccAvg);
        zAccAvg = ITDS_convertAccelerationFs16g_int(zAccAvg);
      }
      else
      {
        debugPrintln("**** ITDS_getRawAccelerations() failed");
      }

      debugPrint(".");
    }

    if (interrupt1Triggered == true)
    {
      /* FIFO buffer full or overrun - this shouldn't happen, if samples are read fast enough */

      interrupt1Triggered = false;

      ITDS_fifoSamples_t fifoSamplesStatus;
      ITDS_getFifoSamplesRegister(&itds, &fifoSamplesStatus);

      char buffer[4];
      sprintf(buffer, "%d", fifoSamplesStatus.fifoFillLevel);

      if (fifoSamplesStatus.fifoOverrunState)
      {
        debugPrint("FIFO buffer overrun ");
      }
      else
      {
        debugPrint("FIFO buffer is full ");
      }

      debugPrint("(contains ");
      debugPrint(buffer);
      debugPrintln(" samples)");

      /* Restart data collection by first setting bypass mode and then re-enabling continuous FIFO mode */
      ITDS_setFifoMode(&itds, ITDS_bypassMode);
      ITDS_setFifoMode(&itds, ITDS_continuousMode);
    }

    /* Print last acceleration values (averaged) every second. */
    uint32_t currentTime = HAL_GetTick();
    if (currentTime >= nextPrintTime)
    {
      nextPrintTime = currentTime + 1000;

      debugPrintln("");
      debugPrintAcceleration_int("X", xAccAvg);
      debugPrintAcceleration_int("Y", yAccAvg);
      debugPrintAcceleration_int("Z", zAccAvg);
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
void ITDS_startContinuousToFifoMode()
{
  debugPrintln("Starting continuous-to-FIFO mode...");

  /* Route both the threshold interrupt and the overrun interrupt to INT1 */
  ITDS_enableFifoThresholdINT1(&itds, ITDS_enable);
  ITDS_enableFifoOverrunIntINT1(&itds, ITDS_enable);

  /* Latched mode enabled (interrupt signal is kept high) */
  /* This is required for continuous to FIFO mode! */
  ITDS_enableLatchedInterrupt(&itds, ITDS_enable);

  /* Configure sensor for 6D orientation change event detection - see example
   * WSEN_ITDS_ORIENTATION for details on the used parameters. */
  ITDS_enableLowNoise(&itds, ITDS_enable);
  ITDS_enableLowPassOn6D(&itds, ITDS_disable);
  ITDS_set6DThreshold(&itds, ITDS_sixtyDeg);
  /* Interrupt for 6D orientation change on INT0 */
  ITDS_enable6DOnINT0(&itds, ITDS_enable);

  int16_t xRawAcc[32];
  int16_t yRawAcc[32];
  int16_t zRawAcc[32];
  int32_t xAccAvg = 0;
  int32_t yAccAvg = 0;
  int32_t zAccAvg = 0;
  bool orientationChanged = false;
  while (1)
  {
    if (interrupt0Triggered == true)
    {
      /* In this example, interrupt 0 is triggered when the device's 6D orientation has changed */

      interrupt0Triggered = false;

      debugPrintln("");
      debugPrintln("Orientation changed");

      orientationChanged = true;
    }

    if (interrupt1Triggered == true)
    {
      /* In this example, interrupt 1 is triggered if the FIFO buffer is either
       * filled up to threshold or in case of an overrun */

      ITDS_fifoSamples_t fifoSamplesStatus;
      ITDS_getFifoSamplesRegister(&itds, &fifoSamplesStatus);

      if (fifoSamplesStatus.fifoOverrunState)
      {
        debugPrintln("FIFO overrun");
      }

      if (orientationChanged == true)
      {
        // 6D orientation change event has been triggered - wait for FIFO buffer to be full
        if (fifoSamplesStatus.fifoFillLevel >= 32)
        {
          interrupt1Triggered = false;
          orientationChanged = false;

          /* Get info on current interrupt events. This causes the 6D orientation change interrupt signal
           * to be revoked (latched mode). */
          ITDS_allInterruptEvents_t interruptEvents;
          ITDS_getAllInterruptEvents(&itds, &interruptEvents);

          /* Read contents of complete FIFO buffer, compute and print average accelerations. */
          if (WE_SUCCESS == ITDS_getRawAccelerations(&itds, 32, xRawAcc, yRawAcc, zRawAcc))
          {
            xAccAvg = 0;
            yAccAvg = 0;
            zAccAvg = 0;

            for (uint8_t i = 0; i < 32; i++)
            {
              xAccAvg += (int32_t) xRawAcc[i];
              yAccAvg += (int32_t) yRawAcc[i];
              zAccAvg += (int32_t) zRawAcc[i];
            }

            xAccAvg /= 32;
            yAccAvg /= 32;
            zAccAvg /= 32;

            xAccAvg = ITDS_convertAccelerationFs16g_int(xAccAvg);
            yAccAvg = ITDS_convertAccelerationFs16g_int(yAccAvg);
            zAccAvg = ITDS_convertAccelerationFs16g_int(zAccAvg);

            debugPrintAcceleration_int("X", xAccAvg);
            debugPrintAcceleration_int("Y", yAccAvg);
            debugPrintAcceleration_int("Z", zAccAvg);
          }
          else
          {
            debugPrintln("**** ITDS_getRawAccelerations() failed");
          }

          /* Restart data collection by first setting bypass mode and then re-enabling continuous to FIFO mode */
          ITDS_setFifoMode(&itds, ITDS_bypassMode);
          ITDS_setFifoMode(&itds, ITDS_continuousToFifo);
        }
      }
      else if (fifoSamplesStatus.fifoFillLevel >= FIFO_THRESH)
      {
        /* No orientation change event happened - this is probably an interrupt signaling
         * that the FIFO buffer has at least FIFO_THRESH elements. */

        interrupt1Triggered = false;

        debugPrint(".");

        /* Must read acceleration values so that there is no overrun while in continuous mode */
        ITDS_getRawAccelerations(&itds, FIFO_THRESH, xRawAcc, yRawAcc, zRawAcc);
      }
    }
  }
}

/**
 * @brief Bypass-to-continuous mode example: Samples are not collected unless a wake-up interrupt
 * has been triggered. After such an interrupt, the sensor automatically switches to continuous mode.
 * The program then processes samples for 500 ms and then re-enables bypass-to-continuous mode.
 */
void ITDS_startBypassToContinuousMode()
{
  debugPrintln("Starting bypass-to-continuous mode...");

  /* Interrupt when FIFO buffer is filled up to threshold on INT1 */
  ITDS_enableFifoThresholdINT1(&itds, ITDS_enable);

  /* Set wake-up interrupt parameters - see example
   * WSEN_ITDS_WAKE_UP for details on the used parameters. */
  ITDS_setWakeUpDuration(&itds, 1);
  ITDS_setWakeUpThreshold(&itds, 1);
  ITDS_enableLowNoise(&itds, ITDS_enable);
  /* Interrupt for wake-up event on INT0 */
  ITDS_enableWakeUpOnINT0(&itds, ITDS_enable);

  int16_t xRawAcc[32];
  int16_t yRawAcc[32];
  int16_t zRawAcc[32];
  int32_t xAccAvg = 0;
  int32_t yAccAvg = 0;
  int32_t zAccAvg = 0;
  uint32_t timeToSleep = 0;
  while (1)
  {
    if (interrupt0Triggered == true)
    {
      /* In this example, interrupt 0 is triggered when the device has woken up */

      interrupt0Triggered = false;

      debugPrintln("");
      debugPrintln("Waking up...");

      /* Collect data for 500 ms, then return to bypass-to-continuous mode */
      timeToSleep = HAL_GetTick() + 500;
    }

    if (interrupt1Triggered == true)
    {
      /* In this example, interrupt 1 is triggered if the FIFO buffer is
       * filled up to the configured threshold */

      interrupt1Triggered = false;

      /* Retrieve FIFO_THRESH samples from sensor */
      if (WE_SUCCESS == ITDS_getRawAccelerations(&itds, FIFO_THRESH, xRawAcc, yRawAcc, zRawAcc))
      {
        xAccAvg = 0;
        yAccAvg = 0;
        zAccAvg = 0;

        for (uint8_t i = 0; i < FIFO_THRESH; i++)
        {
          xAccAvg += (int32_t) xRawAcc[i];
          yAccAvg += (int32_t) yRawAcc[i];
          zAccAvg += (int32_t) zRawAcc[i];
        }

        xAccAvg /= FIFO_THRESH;
        yAccAvg /= FIFO_THRESH;
        zAccAvg /= FIFO_THRESH;

        xAccAvg = ITDS_convertAccelerationFs16g_int(xAccAvg);
        yAccAvg = ITDS_convertAccelerationFs16g_int(yAccAvg);
        zAccAvg = ITDS_convertAccelerationFs16g_int(zAccAvg);

        debugPrintAcceleration_int("X", xAccAvg);
        debugPrintAcceleration_int("Y", yAccAvg);
        debugPrintAcceleration_int("Z", zAccAvg);
      }
      else
      {
        debugPrintln("**** ITDS_getRawAccelerations() failed");
      }

      if (HAL_GetTick() >= timeToSleep)
      {
        /* Return to bypass-to-continuous mode */
        ITDS_setFifoMode(&itds, ITDS_bypassMode);
        ITDS_setFifoMode(&itds, ITDS_bypassToContinuous);
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

