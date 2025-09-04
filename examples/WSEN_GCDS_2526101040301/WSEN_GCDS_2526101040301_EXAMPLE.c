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
 * COPYRIGHT (c) 2025 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 */

/**
 * @file
 * @brief WSEN_GCDS_2526101040301 example.
 *
 * Demonstrates basic usage of the GCDS co2 sensor connected via I2C.
 */
#include "WSEN_GCDS_2526101040301_EXAMPLE.h"
#include "../SensorsSDK/WSEN_GCDS_2526101040301/WSEN_GCDS_2526101040301.h"
#include "gpio.h"
#include "i2c.h"
#include <math.h>
#include <platform.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Functions containing main loops for the available example */
void WE_gcdsEvaluationCSVRaw();

/* Sensor interface configuration */
static WE_sensorInterface_t gcds;

/* Sensor initialization function */
static bool GCDS_init(void);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_gcdsExampleInit()
{
    if (false == GCDS_init())
    {
        debugPrintln("**** GCDS_init() error. STOP ****");
        WE_Delay(5);
        while (1)
            ;
    }
}

/**
 * @brief Example main loop code.
 * Call this function in main loop (infinite loop).
 */
void WE_gcdsExampleLoop() { WE_gcdsEvaluationCSVRaw(); }

/**
 * @brief Prints the humidity and temperature raw values in CSV format
 */
void WE_gcdsEvaluationCSVRaw()
{
    bool dataReadyStatus = false;

    if (WE_SUCCESS != GCDS_Get_Data_Ready_Status(&gcds, &dataReadyStatus))
    {
        debugPrintln("**** GCDS_Get_Data_Ready_Status(): NOT OK ****");
        return;
    }

    if (dataReadyStatus != true)
    {
        return;
    }

    int32_t temperatureRaw = 0;
    uint32_t humidityRaw = 0;
    uint16_t co2value = 0;

    if (WE_SUCCESS == GCDS_Measure_Data(&gcds, &co2value, &temperatureRaw, &humidityRaw))
    {
        char bufferCO2[11];
        sprintf(bufferCO2, "%u", co2value);
        debugPrint(bufferCO2);
        debugPrint(",");
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
 * @brief Initializes the gcds sensor for this example application.
 */
static bool GCDS_init(void)
{
    /* Initialize sensor interface (use i2c with GCDS address, burst mode activated) */
    GCDS_Get_Default_Interface(&gcds);
    gcds.interfaceType = WE_i2c;
    gcds.handle = &hi2c1;

    /* Wait for boot */
    WE_Delay(50);

    if (WE_SUCCESS != GCDS_Init(&gcds))
    {
        debugPrintln("**** GCDS_Init error. STOP ****");
        WE_Delay(5);
        while (1)
            ;
    }

    debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

    if (WE_SUCCESS != GCDS_Start_Periodic_Measurement(&gcds))
    {
        debugPrintln("**** GCDS_Start_Periodic_Measurement(): NOT OK ****");
        WE_Delay(5);
        while (1)
            ;
    }

    debugPrintln("**** GCDS_Start_Periodic_Measurement(): OK ****");

    return true;
}
