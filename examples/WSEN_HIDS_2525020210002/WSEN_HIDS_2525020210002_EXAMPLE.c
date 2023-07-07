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
 * @brief WSEN_hids_Evaluation example.
 *
 * Demonstrates basic usage of the HIDS humidity sensor connected via I2C.
 */
#include "WSEN_HIDS_2525020210002_EXAMPLE.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include <platform.h>
#include "../SensorsSDK/WSEN_HIDS_2525020210002/WSEN_HIDS_2525020210002.h"

/* Functions containing main loops for the available example */
void WE_hidsEvaluationCSVRaw();

/* Sensor interface configuration */
static WE_sensorInterface_t hids;

/* Sensor initialization function */
static bool HIDS_init(void);

/* Debug output functions */
static void debugPrint(char _out[]);
static void debugPrintln(char _out[]);
uint8_t readCurrentChannel = 0xFF;

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_hidsExampleInit()
{
	//MULTIPLEXER_Init(&hids);
	if (false == HIDS_init())
	{
	debugPrintln("**** HIDS_init() error. STOP ****");
	HAL_Delay(5);
	while(1);
	}
	HAL_Delay(5);
}

/**
 * @brief Example main loop code.
 * Call this function in main loop (infinite loop).
 */
void WE_hidsExampleLoop()
{
	WE_hidsEvaluationCSVRaw();
}

/**
 * @brief Prints the humidity and temperature raw values in CSV format
 */
void WE_hidsEvaluationCSVRaw()
{
	int32_t temperatureRaw = 0;
	int32_t humidityRaw = 0;
	hids_measureCmd_t measureCmd = HIDS_MEASURE_HPM;
	if(WE_SUCCESS == HIDS_Sensor_Measure_Raw(&hids,measureCmd,&temperatureRaw, &humidityRaw))
	{
		char bufferHumidity[11];
		sprintf(bufferHumidity, "%li", humidityRaw);
		debugPrint(bufferHumidity);
		debugPrint(",");
		char bufferTemperature[11];
		sprintf(bufferTemperature, "%li", temperatureRaw);
		debugPrint(bufferTemperature);
		debugPrintln("");
	}
}

/**
 * @brief Initializes the hids sensor for this example application.
 */
static bool HIDS_init(void)
{
	/* Initialize sensor interface (use i2c with HIDS address, burst mode activated) */
	HIDS_Get_Default_Interface(&hids);
	hids.interfaceType = WE_i2c_fifo;
	hids.handle = &hi2c1;
	  /* Wait for boot */
	  HAL_Delay(50);
	  if(WE_SUCCESS != HIDS_Sensor_Init(&hids))
	   {
		  debugPrintln("**** HIDS_MULTIPLEXER_Init error. STOP ****");
		  HAL_Delay(5);
		  while(1);
	   }

	debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");
	return true;
}

/* Debug output functions */
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

