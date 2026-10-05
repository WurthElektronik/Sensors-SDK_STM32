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
 * COPYRIGHT (c) 2026 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 */

/**
 * @file
 * @brief WSEN_PDDS I2C example.
 *
 * Demonstrates basic usage of the PDDS differential pressure sensor connected via I2C.
 */
#include "WSEN_PDDS_25131310XXX01_Example.h"
#include "../SensorsSDK/WSEN_PDDS_25131310XXX01/WSEN_PDDS_25131310XXX01.h"
#include "gpio.h"
#include "i2c.h"
#include <platform.h>
#include <stdbool.h>
#include <stdio.h>

/* Sensor interface configuration */
static WE_sensorInterface_t pdds;

/* Sensor initialization function */
static bool WE_pddsInit(void);

/* PDDS sensor type */
static PDDS_SensorType_t pddsSensorType;

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_pddsExampleInit(void)
{
    char bufferMajor[4];
    char bufferMinor[4];
    sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
    sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
    debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
    debugPrint(bufferMajor);
    debugPrint(".");
    debugPrintln(bufferMinor);

    /* init PDDS */
    if (false == WE_pddsInit())
    {
        debugPrintln("**** PDDS initialization failed. STOP ****");
        WE_Delay(100);
        while (1)
            ;
    }

    /* LED on */
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
    WE_Delay(5);
}

/**
 * @brief Example main loop code.
 * Call this function in main loop (infinite loop).
 */
void WE_pddsExampleLoop(void)
{
    float pressure_kPa = 0.0f;
    float temperature_C = 0.0f;

    /* Please select the right PDDS_SensorType_t in the WE_pddsInit() to convert raw values to pressure values */
    if (WE_SUCCESS == PDDS_getPressureAndTemperature_float(&pdds, pddsSensorType, &pressure_kPa, &temperature_C))
    {
        char bufferFull[64] = {0};
        snprintf(bufferFull, sizeof(bufferFull), "%.4fkPa,%.4fC", (double)pressure_kPa, (double)temperature_C);
        debugPrintln(bufferFull);
    }
    else
    {
        debugPrintln("**** PDDS pressure and temperature read failed ****");
    }

    /* Delay of 1 second between successive measurements. */
    WE_Delay(1000);
}

/**
 * @brief Initializes the sensor for this example application.
 */
static bool WE_pddsInit(void)
{
    /* Initialize sensor interface */
    PDDS_GetDefaultInterface(&pdds);
    pdds.interfaceType = WE_i2c;
    pdds.options.i2c.burstMode = 1; /* Burst mode enabled */
    pdds.handle = &hi2c1;

    /* Wait for boot */
    WE_Delay(50);
    if (WE_SUCCESS != WE_isSensorInterfaceReady(&pdds))
    {
        debugPrintln("**** PDDS sensor not responding on I2C. STOP ****");
        return false;
    }

    /* PDDS Sensor Types and Their Specifications:
       - PDDS_pdds0: Order code 2513131035201, range = ±35 kPa
       - PDDS_pdds1: Order code 2513131070301, range =  0 to 100 kPa
       - PDDS_pdds2: Order code 2513131070701, range =  0 to 700 kPa
    */
    pddsSensorType = PDDS_pdds0;
    debugPrintln("**** By default sensorType PDDS_pdds0 with range = ±35 kPa selected ****");
    debugPrintln("**** Please set the appropriate sensorType in the WE_pddsInit function ****");
    debugPrintln("**** PDDS I2C initialization successful ****");
    return true;
}
