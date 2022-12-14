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
 * @brief WSEN_ISDS wake-up example.
 *
 * Example for the ISDS accelerometer/gyroscope demonstrating usage of the wake-up interrupt.
 */

#include "WSEN_ISDS_WAKE_UP_EXAMPLE.h"

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

/* Sensor interface configuration */
static WE_sensorInterface_t isds;

/* Is set to true when a wake-up interrupt occurs */
static bool wakeUp = false;

/* Sensor initialization function */
static bool ISDS_init(void);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_isdsWakeUpExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the \"wake-up\" example program for the ISDS sensor.");
  debugPrintln("Note that for this example to work, the following pin/interrupt configuration is required:");
  debugPrintln("* INT_0 to PA0, rising edge only");

  /* init ISDS */
  if (false == ISDS_init())
  {
    debugPrintln("**** ISDS_Init() error. STOP ****");
    HAL_Delay(5);
    while(1);
  }
}

/**
 * @brief Example main loop code.
 * Call this function in main loop (infinite loop).
 */
void WE_isdsWakeUpExampleLoop()
{
  /* Wake-up event can be detected via the interrupt pin or by reading the status register. */
  /* Here, the status register is checked after a wake-up interrupt has occurred. */
  /* See HAL_GPIO_EXTI_Rising_Callback() for the corresponding ISR callback. */
  if (wakeUp)
  {
    wakeUp = false;

    ISDS_wakeUpEvent_t wakeUpEvent;
    if (ISDS_getWakeUpEventRegister(&isds, &wakeUpEvent) == WE_SUCCESS)
    {
      /* Check if wake-up event has occurred (at least one axis exceeded the threshold) */
      if (wakeUpEvent.wakeUpState != 0)
      {
        /* Check acceleration of which axes exceeded the threshold */
        debugPrint("Wake-up event in direction ");
        if (wakeUpEvent.wakeUpX != 0)
        {
          debugPrint("x");
        }
        if (wakeUpEvent.wakeUpY != 0)
        {
          debugPrint("y");
        }
        if (wakeUpEvent.wakeUpZ != 0)
        {
          debugPrint("z");
        }
        debugPrintln("");
      }
    }
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

  /* Turn on accelerometer (416 Hz) */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr416Hz);

  /* 2g range */
  ISDS_setAccFullScale(&isds, ISDS_accFullScaleTwoG);

  /* Set min. duration of wake-up event (1 bit = 1 * 1 / ODR) */
  ISDS_setWakeUpDuration(&isds, 1);

  /* Set wake-up acceleration threshold (1 bit = 1 * full_scale / 64 i.e. 4 * 2 g / 64 = 0.125 g) */
  ISDS_setWakeUpThreshold(&isds, 4);

  /* Use slope filter for wake-up event detection */
  ISDS_setActivityFilter(&isds, ISDS_activityFilterSlope);

  /* Interrupts are active high */
  ISDS_setInterruptActiveLevel(&isds, ISDS_activeHigh);

  /* Interrupts are push-pull */
  ISDS_setInterruptPinType(&isds, ISDS_pushPull);

  /* Latched mode disabled (interrupt signal is automatically reset) */
  ISDS_enableLatchedInterrupt(&isds, ISDS_disable);

  /* Enable interrupts */
  ISDS_enableInterrupts(&isds, ISDS_enable);

  /* Depending on the parameters set above (threshold, duration), it might be
   * necessary to wait until the first sample has been collected, before activating
   * the wake-up interrupt. This prevents an erroneous wake-up event on startup.
   * Alternatively, the first wake-up interrupt could be ignored. */
//  HAL_Delay(5);

  /* Enable wake-up interrupt on INT_0 */
  ISDS_enableWakeUpINT0(&isds, ISDS_enable);

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
    /* Wake-up interrupt - toggle LED */
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

    /* Trigger event handling in main function. */
    wakeUp = true;
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
