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
 * @brief WSEN-PADS advanced usage example.
 *
 * Advanced usage of the PADS absolute pressure sensor connected via I2C (data-ready interrupt,
 * AUTOZERO mode, usage of the FIFO buffer).
 */

#include "WSEN_PADS_ADVANCED_EXAMPLE.h"

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

/* Enum used to switch between different example modes (see variable padsExampleMode) */
typedef enum
{
  PADS_exampleDataReadyInterrupt,   /* Example demonstrating usage of data-ready interrupt (new sample available) */
  PADS_exampleAutoZero,             /* Example showing how to use the AUTOZERO functionality (setting a reference pressure, output of pressure difference) */
  PADS_exampleFifoMode,             /* Example demonstrating usage of FIFO buffer (full/overrun interrupt) */
  PADS_exampleContinuousMode,       /* Example demonstrating usage of FIFO buffer in continuous mode (threshold interrupt) */
  PADS_exampleBypassToFifoMode      /* Example showing how to start FIFO mode when a pressure high/low interrupt has been triggered */
} PADS_advanced_example_mode;

/* Number of samples to be discarded after startup (to ensure that the value used as reference pressure is valid) */
#define PADS_EXAMPLE_DISCARD_SAMPLES_COUNT 2

/* Threshold (relative to reference pressure) used for examples using high/low pressure event (Pa) */
#define PADS_EXAMPLE_PRESSURE_THRESH 20

/* Use the following variable to switch among the available example modes. */
static PADS_advanced_example_mode padsExampleMode = PADS_exampleDataReadyInterrupt;

/* Sensor interface configuration */
static WE_sensorInterface_t pads;

/* Is set to true when an interrupt has been triggered */
static bool interruptTriggered = false;

/* Sensor initialization function */
static bool PADS_init(void);

/* Functions containing main loops for the available example modes. */
void PADS_startDataReadyInterruptExample();
void PADS_startAutoZeroExample();
void PADS_startFifoExample();
void PADS_startContinuousExample();
void PADS_startBypassToFifoExample();

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);
static void debugPrintPressure_int(int32_t pressure);
static void debugPrintTemperature_int(int16_t temperature);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_padsAdvancedExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the advanced example program for the PADS sensor.");
  debugPrintln("Note that for this example to work, the following pin/interrupt configuration is required:");
  debugPrintln("* INT_0 to PA0, rising edge interrupt only");

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
void WE_padsAdvancedExampleLoop()
{
  switch (padsExampleMode)
  {
  case PADS_exampleDataReadyInterrupt:
    PADS_startDataReadyInterruptExample();
    break;

  case PADS_exampleAutoZero:
    PADS_startAutoZeroExample();
    break;

  case PADS_exampleFifoMode:
    PADS_startFifoExample();
    break;

  case PADS_exampleContinuousMode:
    PADS_startContinuousExample();
    break;

  case PADS_exampleBypassToFifoMode:
    PADS_startBypassToFifoExample();
    break;
  }
}

/**
 * @brief Initializes the sensor for this example application.
 */
static bool PADS_init(void)
{
  /* Initialize sensor interface (i2c with PADS address, burst mode activated) */
  PADS_getDefaultInterface(&pads);
  pads.interfaceType = WE_i2c;
  pads.options.i2c.address = PADS_ADDRESS_I2C_0;
  pads.options.i2c.burstMode = 1;
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

  /* Interrupts are active high */
  PADS_setInterruptActiveLevel(&pads, PADS_activeHigh);

  /* Interrupts are push-pull */
  PADS_setInterruptPinType(&pads, PADS_pushPull);

  return true;
}

/**
 * @brief Data-ready interrupt example: An interrupt is generated as soon as a
 * new sample is available.
 * The latest sample is printed at 1 Hz.
 */
void PADS_startDataReadyInterruptExample()
{
  debugPrintln("Starting data-ready interrupt example");

  /* Disable FIFO */
  PADS_setFifoMode(&pads, PADS_bypassMode);

  /* Enable data-ready interrupt */
  PADS_enableDataReadyInterrupt(&pads, PADS_enable);

  /* Activate data-ready interrupt in event control register */
  PADS_setInterruptEventControl(&pads, PADS_dataReady);

  /* Enable continuous operation with an update rate of 50 Hz */
  PADS_setOutputDataRate(&pads, PADS_outputDataRate50Hz);

  uint32_t printIntervalMs = 1000;
  uint32_t printTime = HAL_GetTick() + printIntervalMs;

  int32_t pressure = 0;

  while (1)
  {

    if (interruptTriggered == true)
    {
      interruptTriggered = false;
      PADS_getPressure_int(&pads, &pressure);
    }

    uint32_t currentTime = HAL_GetTick();
    if (currentTime > printTime)
    {
      debugPrintPressure_int(pressure);
      printTime = currentTime + printIntervalMs;
    }
  }
}

/**
 * @brief AUTOZERO example: AUTOZERO mode is enabled on startup, which stores the
 * current pressure as reference. Uses the data-ready interrupt to trigger
 * reading of new samples.
 * In AUTOZERO mode, the sensor returns the difference between the measured pressure
 * and the reference pressure in its output registers.
 */
void PADS_startAutoZeroExample()
{
  debugPrintln("Starting auto-zero example");

  /* Disable FIFO */
  PADS_setFifoMode(&pads, PADS_bypassMode);

  /* Enable data-ready interrupt */
  PADS_enableDataReadyInterrupt(&pads, PADS_enable);

  /* Activate data-ready interrupt in event control register */
  PADS_setInterruptEventControl(&pads, PADS_dataReady);

  /* Discard first samples */
  for (uint8_t i = 0; i < PADS_EXAMPLE_DISCARD_SAMPLES_COUNT; i++)
  {
    /* Start a conversion (one shot) */
    PADS_enableOneShot(&pads, PADS_enable);
    PADS_state_t presStatus;
    do
    {
      PADS_isPressureDataAvailable(&pads, &presStatus);
    } while (presStatus != PADS_enable);
  }

  /* Set either AUTOZERO (standard output register contain differential signal) or
   * AUTOREFP (standard output registers contain the usual output) mode */
  PADS_enableAutoZeroMode(&pads, PADS_enable);
//  PADS_enableAutoRefp(&pads, PADS_enable);

  /* Wait for AUTOZERO to be revoked (after first conversion) */
  PADS_state_t autoZero = PADS_enable;
  while (autoZero == PADS_enable)
  {
    PADS_isEnablingAutoZeroMode(&pads, &autoZero);
  }

  /* Retrieve and print the reference pressure that has been set when enabling AUTOZERO/AUTOREFP mode */
  uint32_t refPressure;
  PADS_getReferencePressure(&pads, &refPressure);
  debugPrint("Reference pressure: ");
  debugPrintPressure_int(refPressure);

  /* Enable continuous operation with an update rate of 50 Hz */
  PADS_setOutputDataRate(&pads, PADS_outputDataRate50Hz);

  uint32_t printIntervalMs = 1000;
  uint32_t printTime = HAL_GetTick() + printIntervalMs;

  int32_t pressure = 0;
  int32_t rawPressure = 0;

  while (1)
  {
    if (interruptTriggered == true)
    {
      interruptTriggered = false;

      PADS_getRawPressure(&pads, &rawPressure);
      PADS_getDifferentialPressure_int(&pads, &pressure);
    }

    uint32_t currentTime = HAL_GetTick();
    if (currentTime > printTime)
    {
      /* Print both raw pressure and pressure in kPa */
      char buffer[32];
      sprintf(buffer, "%ld", rawPressure);
      debugPrint("Raw pressure: ");
      debugPrintln(buffer);

      debugPrintPressure_int(pressure);
      printTime = currentTime + printIntervalMs;
    }
  }
}

/**
 * @brief FIFO example: Samples are collected until the FIFO buffer is full.
 * The complete FIFO buffer is then read in one go, thus emptying the buffer,
 * and collection of new samples is started by re-enabling FIFO mode.
 */
void PADS_startFifoExample()
{
  debugPrintln("Starting FIFO example");

  /* Enable FIFO mode */
  PADS_setFifoMode(&pads, PADS_fifoEnabled);

  /* Enable interrupts for FIFO buffer full and overrun events on INT1 */
  PADS_enableFifoOverrunInterrupt(&pads, PADS_enable);
  PADS_enableFifoFullInterrupt(&pads, PADS_enable);

  /* Activate FIFO full and overrun interrupts in event control register */
  PADS_setInterruptEventControl(&pads, PADS_dataReady);

  /* Enable continuous operation with an update rate of 200 Hz */
  PADS_setOutputDataRate(&pads, PADS_outputDataRate200Hz);

  uint32_t printIntervalMs = 1000;
  uint32_t printTime = HAL_GetTick() + printIntervalMs;

  int32_t pressureBuffer[PADS_FIFO_BUFFER_SIZE] = {0};
  int16_t temperatureBuffer[PADS_FIFO_BUFFER_SIZE] = {0};

  while (1)
  {
    if (interruptTriggered == true)
    {
      interruptTriggered = false;

      /* Get pressure and temperature measurements in one go. */
      PADS_getFifoValues_int(&pads, PADS_FIFO_BUFFER_SIZE, pressureBuffer, temperatureBuffer);

      /* Alternatively, one of the following functions can be used to get pressure or temperature. */
//      PADS_getFifoPressure_int(&pads, PADS_FIFO_BUFFER_SIZE, pressureBuffer);
//      PADS_getFifoTemperature_int(&pads, PADS_FIFO_BUFFER_SIZE, temperatureBuffer);

      /* Must set to bypass mode and then re-enable FIFO mode to start capturing of new data. */
      PADS_setFifoMode(&pads, PADS_bypassMode);
      PADS_setFifoMode(&pads, PADS_fifoEnabled);
    }

    uint32_t currentTime = HAL_GetTick();
    if (currentTime > printTime)
    {
      /* Compute average of captured pressure/temperature values and print the results */

      uint32_t pressure = 0;
      for (uint8_t i = 0; i < PADS_FIFO_BUFFER_SIZE; i++)
      {
        pressure += pressureBuffer[i];
      }
      pressure /= PADS_FIFO_BUFFER_SIZE;
      debugPrintPressure_int(pressure);

      int32_t temperature = 0;
      for (uint8_t i = 0; i < PADS_FIFO_BUFFER_SIZE; i++)
      {
        temperature += temperatureBuffer[i];
      }
      temperature /= PADS_FIFO_BUFFER_SIZE;
      debugPrintTemperature_int(temperature);

      printTime = currentTime + printIntervalMs;
    }
  }
}

/**
 * @brief Continuous FIFO mode example: Samples are collected continuously.
 * When the FIFO buffer is filled up to FIFO_THRESH, the collected data is read
 * thus making space for new samples.
 */
void PADS_startContinuousExample()
{
  debugPrintln("Starting continuous mode example");

  /* Enable FIFO continuous mode */
  PADS_setFifoMode(&pads, PADS_continuousMode);

  /* Set FIFO fill threshold */
  const int fifoThresh = PADS_FIFO_BUFFER_SIZE / 4;
  PADS_setFifoThreshold(&pads, fifoThresh);

  /* Interrupt for FIFO buffer fill threshold reached on INT1 */
  PADS_enableFifoThresholdInterrupt(&pads, PADS_enable);

  /* Activate FIFO threshold interrupt in event control register */
  PADS_setInterruptEventControl(&pads, PADS_dataReady);

  /* Enable continuous operation with an update rate of 200 Hz */
  PADS_setOutputDataRate(&pads, PADS_outputDataRate200Hz);

  uint32_t printIntervalMs = 1000;
  uint32_t printTime = HAL_GetTick() + printIntervalMs;

  int32_t pressureBuffer[PADS_FIFO_BUFFER_SIZE] = {0};
  int16_t temperatureBuffer[PADS_FIFO_BUFFER_SIZE] = {0};

  while (1)
  {
    if (interruptTriggered == true)
    {
      interruptTriggered = false;

      /* Get pressure and temperature measurements in one go. */
      /* Note: Samples must be read faster than the ODR. */
      PADS_getFifoValues_int(&pads, fifoThresh, pressureBuffer, temperatureBuffer);

      /* Alternatively, one of the following functions can be used to get pressure or temperature. */
//      PADS_getFifoPressure_int(&pads, fifoThresh, pressureBuffer);
//      PADS_getFifoTemperature_int(&pads, fifoThresh, temperatureBuffer);
    }

    uint32_t currentTime = HAL_GetTick();
    if (currentTime > printTime)
    {
      /* Compute average of captured pressure/temperature values and print the results */

      uint32_t pressure = 0;
      for (uint8_t i = 0; i < fifoThresh; i++)
      {
        pressure += pressureBuffer[i];
      }
      pressure /= fifoThresh;
      debugPrintPressure_int(pressure);

      int32_t temperature = 0;
      for (uint8_t i = 0; i < fifoThresh; i++)
      {
        temperature += temperatureBuffer[i];
      }
      temperature /= fifoThresh;
      debugPrintTemperature_int(temperature);

      printTime = currentTime + printIntervalMs;
    }
  }
}

/**
 * @brief Bypass-to-FIFO mode example: Samples are not collected unless a pressure
 * high/low event has been triggered. After such an interrupt, the sensor
 * automatically switches to FIFO mode and collects samples until the user-specified
 * FIFO fill threshold is reached, which triggers a FIFO threshold interrupt.
 * After that, bypass-to-FIFO mode is re-enabled.
 */
void PADS_startBypassToFifoExample()
{
  debugPrintln("Starting bypass-to-FIFO mode example");

  /* Discard first samples */
  for (uint8_t i = 0; i < PADS_EXAMPLE_DISCARD_SAMPLES_COUNT; i++)
  {
    /* Start a conversion (one shot) */
    PADS_enableOneShot(&pads, PADS_enable);
    PADS_state_t presStatus;
    do
    {
      PADS_isPressureDataAvailable(&pads, &presStatus);
    } while (presStatus != PADS_enable);
  }

  /* Set FIFO fill threshold */
  const int fifoThresh = PADS_FIFO_BUFFER_SIZE / 4;
  PADS_setFifoThreshold(&pads, fifoThresh);

  /* Stop filling FIFO as soon as the configured threshold is reached */
  PADS_enableStopOnThreshold(&pads, PADS_enable);

  /* Set either AUTOZERO (standard output register contain differential signal) or
   * AUTOREFP (standard output registers contain the usual output) mode */
  PADS_enableAutoZeroMode(&pads, PADS_enable);
//  PADS_enableAutoRefp(&pads, PADS_enable);

  /* Wait for AUTOZERO to be revoked (after first conversion) */
  PADS_state_t autoZero = PADS_enable;
  while (autoZero == PADS_enable)
  {
    PADS_isEnablingAutoZeroMode(&pads, &autoZero);
  }

  /* Retrieve and print the reference pressure that has been set when enabling AUTOZERO/AUTOREFP mode */
  uint32_t refPressure;
  PADS_getReferencePressure(&pads, &refPressure);
  debugPrint("Reference pressure: ");
  debugPrintPressure_int(refPressure);

  /* Set pressure threshold (relative to reference pressure, used for high/low pressure events) */
  PADS_setPressureThreshold(&pads, PADS_EXAMPLE_PRESSURE_THRESH);

  /* Retrieve and print the configured pressure threshold */
  uint32_t pressureThresh;
  PADS_getPressureThreshold(&pads, &pressureThresh);
  debugPrint("Pressure threshold: ");
  debugPrintPressure_int(pressureThresh);

  /* Enable bypass-to-FIFO mode */
  PADS_setFifoMode(&pads, PADS_bypassToFifo);

  /* Enable high and low pressure interrupts */
  PADS_enableDiffPressureInterrupt(&pads, PADS_enable);
  PADS_enableLowPressureInterrupt(&pads, PADS_enable);
  PADS_enableHighPressureInterrupt(&pads, PADS_enable);

  /* Activate high/low pressure interrupts in event control register */
  PADS_setInterruptEventControl(&pads, PADS_pressureHighOrLow);

  /* Enable continuous operation with an update rate of 200 Hz */
  PADS_setOutputDataRate(&pads, PADS_outputDataRate200Hz);

  int32_t pressureBuffer[PADS_FIFO_BUFFER_SIZE] = {0};

  /* Last FIFO thresh exceeded state */
  PADS_state_t lastThreshExceededState = PADS_disable;

  /* Last FIFO buffer fill level */
//  uint8_t lastFillLevel = 0;

  while (1)
  {
    if (interruptTriggered == true)
    {
      interruptTriggered = false;

      PADS_intSource_t intSource;
      PADS_getInterruptSource(&pads, &intSource);

      debugPrint("Interrupt triggered, interrupt status = ");
      debugPrintln(intSource.intStatus ? "1" : "0");
      if (intSource.diffPresLowEvent)
      {
        debugPrintln("Low pressure event");
      }
      if (intSource.diffPresHighEvent)
      {
        debugPrintln("High pressure event");
      }
    }

    PADS_state_t threshExceeded;
    PADS_isFifoThresholdReached(&pads, &threshExceeded);
    if (lastThreshExceededState != threshExceeded)
    {
      if (threshExceeded == PADS_enable)
      {
        debugPrintln("FIFO thresh exceeded event");

        PADS_getFifoPressure_int(&pads, fifoThresh, pressureBuffer);

        /* Compute average of captured pressure values */

        uint32_t pressure = 0;
        for (uint8_t i = 0; i < fifoThresh; i++)
        {
          pressure += pressureBuffer[i];
        }
        pressure /= fifoThresh;
        debugPrintPressure_int(pressure);


        /* Must set to bypass mode and then re-enable bypass-to-FIFO mode to start capturing of new data. */
        PADS_setFifoMode(&pads, PADS_bypassMode);
        PADS_setFifoMode(&pads, PADS_bypassToFifo);
      }
      lastThreshExceededState = threshExceeded;
    }

    /* Uncomment the following lines to print the current FIFO fill level */
//    uint8_t fillLevel;
//    PADS_getFifoFillLevel(&pads, &fillLevel);
//    if (fillLevel != lastFillLevel)
//    {
//      char buffer[50];
//      sprintf(buffer, "FIFO fill level = %d", fillLevel);
//      debugPrintln(buffer);
//      lastFillLevel = fillLevel;
//    }
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
    interruptTriggered = true;

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
