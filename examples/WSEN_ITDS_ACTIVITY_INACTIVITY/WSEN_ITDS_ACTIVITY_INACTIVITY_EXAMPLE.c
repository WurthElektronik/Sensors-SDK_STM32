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

#include "WSEN_ITDS_ACTIVITY_INACTIVITY_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include "../SensorsSDK/WSEN_ITDS_2533020201601/WSEN_ITDS_2533020201601.h"


/* Enum used to switch between different example modes (see variable itdsExampleMode) */
typedef enum
{
  sleepChangeInterruptMode, /* Example using trigger signal on transition from inactive to active and vice versa */
  sleepStatusInterruptMode, /* Example using rising and falling edge of sleep status signal */
  motionDetectionMode       /* Similar to sleepStatusInterruptMode but no ODR / power mode changes while stationary */
} ITDS_activity_inactivity_example_mode;


/* Use the following variable to switch among the available example modes. */
ITDS_activity_inactivity_example_mode itdsExampleMode = sleepChangeInterruptMode;

/* Boolean variables used to notify main loop that interrupts have been triggered */
bool activityDetected = false;
bool leftSleepMode = false;
bool enteredSleepMode = false;
bool sleepChangeEvent = false;

/* Sensor initialization function */
bool ITDS_init(void);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_itdsActivityInactivityExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrint("This is the \"activity/inactivity-detection\" (");
  switch (itdsExampleMode)
  {
  case sleepChangeInterruptMode:
    debugPrint("using sleep change interrupt");
    break;

  case sleepStatusInterruptMode:
    debugPrint("using sleep status interrupt");
    break;

  case motionDetectionMode:
    debugPrint("motion detection");
    break;
  }
  debugPrintln(") example program for the ITDS sensor.");
  debugPrintln("Note that for this example to work, the following pin/interrupt configuration is required:");
  debugPrintln("* INT_0 to PA0, rising edge only");
  debugPrintln("* INT_1 to PA1, rising and falling edge");

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
void WE_itdsActivityInactivityExampleLoop()
{
  if (activityDetected == true)
  {
    /* Wake-up condition met */
    activityDetected = false;
    debugPrintln("Activity detected.");
  }

  if (enteredSleepMode == true)
  {
    /* Sleep mode signal has been triggered */
    enteredSleepMode = false;
    debugPrintln("Sleep status rising edge: Going to sleep.");
  }

  if (leftSleepMode == true)
  {
    /* Sleep mode signal has been revoked */
    leftSleepMode = false;
    debugPrintln("Sleep status falling edge: Waking up.");
  }

  if (sleepChangeEvent == true)
  {
    /* Interrupt due to sleep state transition trigger signal - read sleep state
     * register to find out if in active or inactive state */

    sleepChangeEvent = false;

    ITDS_state_t sleepState;
    if (ITDS_getSleepState(&sleepState) == WE_SUCCESS)
    {
      if (sleepState == ITDS_enable)
      {
        debugPrintln("Sleep status change event: Going to sleep.");

        /* LED off */
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
      }
      else
      {
        debugPrintln("Sleep status change event: Waking up.");

        /* LED on */
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
      }
    }
  }

  HAL_Delay(1);
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


  /* Turn on accelerometer (high performance, 200Hz) */
  ITDS_setOperatingMode(ITDS_highPerformance);
  ITDS_setOutputDataRate(ITDS_odr6);

  /* 2g range */
  ITDS_setFullScale(ITDS_twoG);

  /* Enable inactivity detection */
  ITDS_enableInactivityDetection(ITDS_enable);

  /* Set minimum duration of wake-up event (1 bit = 1 / ODR) */
  /* Corresponds to 2 / 200 = 10 ms */
  ITDS_setWakeUpDuration(2);

  /* Set minimum inactivity time (1 bit = 1 * 512 / ODR) */
  /* Corresponds to 1 * 512 / 200 = 2.56 s */
  ITDS_setSleepDuration(1);

  /* Set wake-up acceleration threshold (1 bit = full_scale / 64 i.e. 2 * 2 g / 64 = 0.0625 g) */
  ITDS_setWakeUpThreshold(2);

  /* If required, the following options can be used to specify a custom
   * offset for triggering the wake-up event (wake-up is triggered if the
   * difference between measured data and user offset exceeds the threshold). */

  /* Set weight of 15.6 mg per LSB */
//  ITDS_setOffsetWeight(ITDS_enable);
  /* No offset for X and Y, Z offset of 1g (64 * 15.6 mg) */
//  ITDS_setOffsetValueOnXAxis(0);
//  ITDS_setOffsetValueOnYAxis(0);
//  ITDS_setOffsetValueOnZAxis(64);
  /* Apply user offset to all data or to data used for wake-up only */
//  ITDS_enApplyOffset(ITDS_enable);
//  ITDS_enApplyWakeUpOffset(ITDS_enable);

  /* Interrupts are active high */
  ITDS_setInterruptActiveLevel(ITDS_activeHigh);

  /* Interrupts are push-pull */
  ITDS_setInterruptPinType(ITDS_pushPull);

  /* Latched mode disabled (interrupt signals are automatically reset) */
  ITDS_enableLatchedInterrupt(ITDS_disable);

  /* Enable interrupts */
  ITDS_enableInterrupts(ITDS_enable);

  /* Enable activity interrupt on INT_0 */
  ITDS_enableWakeUpOnINT0(ITDS_enable);

  switch (itdsExampleMode)
  {
  case sleepChangeInterruptMode:
    /* Enable sleep state transition interrupt on INT_1 (trigger signal when waking up or going to sleep) */
    ITDS_enableSleepStatusChangeINT1(ITDS_enable);
    break;

  case sleepStatusInterruptMode:
    /* Setting both of the following interrupts causes interrupt INT_1 to be
     * high as long as the sensor is in sleep mode */
    ITDS_enableSleepStatusINT1(ITDS_enable);
    ITDS_enableSleepStatusChangeINT1(ITDS_enable);
    break;

  case motionDetectionMode:
    /* Setting both of the following interrupts causes interrupt INT_1 to be
     * high as long as the sensor is in sleep mode */
    ITDS_enableSleepStatusINT1(ITDS_enable);
    ITDS_enableSleepStatusChangeINT1(ITDS_enable);

    /* Enable stationary/motion detection mode */
    /* In this mode, no ODR and power mode changes occur when changing between inactive and active mode. */
    ITDS_enableStationaryDetection(ITDS_enable);
    break;
  }

  return true;
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
  if (GPIO_Pin == GPIO_PIN_0)
  {
    /* Wake-up event has been triggered. */
    /* Trigger event handling in main function. */
    activityDetected = true;
  }
  else if (GPIO_Pin == GPIO_PIN_1)
  {
    /* Sleep mode event has been triggered. Interpretation depends on configuration. */

    switch (itdsExampleMode)
    {
    case sleepChangeInterruptMode:
      /* Sleep state transition. Trigger event handling in main (where the status register
       * is read to find out if in sleep mode or not). */
      sleepChangeEvent = true;
      break;

    case sleepStatusInterruptMode:
    case motionDetectionMode:
      /* Inactivity interrupt - turn LED off */
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

      /* Trigger event handling in main function. */
      enteredSleepMode = true;
      break;
    }
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_1)
  {
    /* Sleep mode event has been triggered. Interpretation depends on configuration. */

    switch (itdsExampleMode)
    {
    case sleepChangeInterruptMode:
      break;

    case sleepStatusInterruptMode:
    case motionDetectionMode:
      /* Wake-up interrupt - turn LED on */
      HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

      /* Trigger event handling in main function. */
      leftSleepMode = true;
      break;
    }
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
