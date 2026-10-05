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
 * @brief WSEN_HIDS_2525020210002 example.
 *
 * Demonstrates basic usage of the HIDS2 humidity sensor connected via I2C.
 */
#include "WSEN_HIDS_2525020210002_EXAMPLE.h"
#include "../SensorsSDK/WSEN_HIDS_2525020210002/WSEN_HIDS_2525020210002.h"
#include "gpio.h"
#include "i2c.h"
#include <platform.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Functions containing main loops for the available example */
static void WE_hidsEvaluationCSVRaw();

/* Sensor initialization function */
static bool WE_hidsInit(void);

/* Sensor interface configuration */
static WE_sensorInterface_t hids;

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_hidsExampleInit()
{
    if (false == WE_hidsInit())
    {
        debugPrintln("**** WE_hidsInit() error. STOP ****");
        HAL_Delay(5);
        while (1)
            ;
    }
    HAL_Delay(5);
}

/**
 * @brief Example main loop code.
 * Call this function in main loop (infinite loop).
 */
void WE_hidsExampleLoop() { WE_hidsEvaluationCSVRaw(); }

/**
 * @brief Prints the humidity and temperature raw values in CSV format
 */
void WE_hidsEvaluationCSVRaw()
{
    int32_t temperatureRaw = 0;
    int32_t humidityRaw = 0;
    hids_measureCmd_t measureCmd = HIDS_MEASURE_HPM;
    if (WE_SUCCESS == HIDS_Sensor_Measure_Raw(&hids, measureCmd, &temperatureRaw, &humidityRaw))
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
static bool WE_hidsInit(void)
{
    /* Initialize sensor interface (use i2c with HIDS address, burst mode activated) */
    HIDS_Get_Default_Interface(&hids);
    hids.interfaceType = WE_i2c;
    hids.handle = &hi2c1;

    /* Wait for boot */
    HAL_Delay(50);
    while (WE_SUCCESS != WE_isSensorInterfaceReady(&hids))
    {
    }
    debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

    if (WE_SUCCESS != HIDS_Sensor_Init(&hids))
    {
        debugPrintln("**** HIDS_Sensor_Init error. STOP ****");
        HAL_Delay(5);
        while (1)
            ;
    }

    return true;
}
