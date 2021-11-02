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

#include "WSEN_ITDS_TAP_EXAMPLE.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include "../SensorsSDK/WSEN_ITDS_2533020201601/WSEN_ITDS_2533020201601.h"

/* Is set to true when a tap interrupt occurs */
bool tapOccurred = false;

/* Sensor initialization function */
bool ITDS_init(void);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_itdsTapExampleInit()
{
  char bufferMajor[4];
  char bufferMinor[4];
  sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
  sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
  debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
  debugPrint(bufferMajor);
  debugPrint(".");
  debugPrintln(bufferMinor);
  debugPrintln("This is the \"single/double tap\" example program for the ITDS sensor.");
  debugPrintln("Note that for this example to work, the following pin/interrupt configuration is required:");
  debugPrintln("* INT_0 to PA0, rising edge only");

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
void WE_itdsTapExampleLoop()
{
  /* Tap events can be detected via the interrupt pin or by reading the status register. */
  /* Here, the status register is checked after a tap interrupt has occurred. */
  /* See HAL_GPIO_EXTI_Rising_Callback() for the corresponding ISR callback. */
  if (tapOccurred == true)
  {
    tapOccurred = false;

    ITDS_tapEvent_t tapEvent;
    if (ITDS_getTapEventRegister(&tapEvent) == WE_SUCCESS)
    {
        /* Check if tap event has occurred */
        if (tapEvent.singleState != 0 || tapEvent.doubleState != 0)
        {
          if (tapEvent.singleState != 0)
          {
            debugPrint("Single");
          }
          else if (tapEvent.doubleState != 0)
          {
            debugPrint("Double");
          }
          debugPrint(" tap event in direction ");
          if (tapEvent.tapSign == 1)
          {
            debugPrint("-");
          }
          if (tapEvent.tapXAxis != 0)
          {
            debugPrint("x");
          }
          if (tapEvent.tapYAxis != 0)
          {
            debugPrint("y");
          }
          if (tapEvent.tapZAxis != 0)
          {
            debugPrint("z");
          }
          debugPrintln("");
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

  /* Turn on accelerometer (high performance, 400 Hz) */
  /* (400 Hz is the minimum recommended ODR for tap recognition) */
  ITDS_setOperatingMode(ITDS_highPerformance);
  ITDS_setOutputDataRate(ITDS_odr7);

  /* Low noise mode */
  ITDS_enableLowNoise(ITDS_enable);

  /* 2g range */
  ITDS_setFullScale(ITDS_twoG);

  /* Set tap thresholds for x, y and z (5 bits, 1 bit = 1 * full_scale / 32) */
  /* Corresponds to 9 * 2 / 32 = 0.5625 g */
  ITDS_setTapThresholdX(9);
  ITDS_setTapThresholdY(9);
  ITDS_setTapThresholdZ(9);

  /* Enable tap recognition for x, y and z */
  ITDS_enableTapX(ITDS_enable);
  ITDS_enableTapY(ITDS_enable);
  ITDS_enableTapZ(ITDS_enable);

  /* Enable both single and double tap */
  /* Change this to ITDS_disable to enable single-tap only */
  ITDS_enableDoubleTapEvent(ITDS_enable);

  /* Set priority z-y-x */
  ITDS_setTapAxisPriority(ITDS_Z_Y_X);

  /* Set quiet time (1 bit = 1 * 4 / ODR) */
  /* Corresponds to 1 * 4 / 400 = 10 ms */
  ITDS_setTapQuietTime(1);

  /* Set shock time (1 bit = 1 * 8 / ODR) */
  /* Corresponds to 2 * 8 / 400 = 40 ms */
  ITDS_setTapShockTime(2);

  /* Set latency time (1 bit = 1 * 32 / ODR) */
  /* Corresponds to 5 * 32 / 400 = 400 ms */
  ITDS_setTapLatencyTime(5);

  /* Interrupts are active high */
  ITDS_setInterruptActiveLevel(ITDS_activeHigh);

  /* Interrupts are push-pull */
  ITDS_setInterruptPinType(ITDS_pushPull);

  /* Latched mode disabled (interrupt signal is automatically reset) */
  ITDS_enableLatchedInterrupt(ITDS_disable);

  /* Enable interrupts */
  ITDS_enableInterrupts(ITDS_enable);

  /* Enable single-tap interrupt on INT_0 */
  ITDS_enableSingleTapINT0(ITDS_enable);

  /* Enable double-tap interrupt on INT_0 */
  ITDS_enableDoubleTapINT0(ITDS_enable);

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
    /* Tap interrupt (may be single or double) - toggle LED */
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

    /* Trigger event handling in main function. */
    tapOccurred = true;
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
