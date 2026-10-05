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
 * @brief WSEN_PDMS SPI example.
 *
 * Basic usage of the PDMS differential pressure sensor connected via SPI.
 */
#include "../SensorsSDK/WSEN_PDMS_25131308XXX05/WSEN_PDMS_25131308XXX05.h"
#include "gpio.h"
#include "spi.h"
#include <examples/WSEN_PDMS_25131308XXX05_SPI/WSEN_PDMS_25131308XXX05_SPI_EXAMPLE.h>
#include <math.h>
#include <platform.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Sensor interface configuration */
static WE_sensorInterface_t pdms;

/* PDMS sensor type */
static PDMS_SensorType_t pdmsSensorType;

/* SPI crc select variable */
static PDMS_Spi_CrcSelect_t spiCrcSelect;

/* Sensor initialization function */
static bool WE_pdmsInit(void);

/* Functions to print float values */
static void debugPrintPressure_float(float pressureKPa);
static void debugPrintTemperature_float(float temperature);

/**
 * @brief Example initialization.
 * Call this function after HAL initialization.
 */
void WE_pdmsSpiExampleInit()
{
    char bufferMajor[4];
    char bufferMinor[4];
    sprintf(bufferMajor, "%d", WE_SENSOR_SDK_MAJOR_VERSION);
    sprintf(bufferMinor, "%d", WE_SENSOR_SDK_MINOR_VERSION);
    debugPrint("Wuerth Elektronik eiSos Sensors SDK version ");
    debugPrint(bufferMajor);
    debugPrint(".");
    debugPrintln(bufferMinor);
    debugPrintln("Pin CS/SA0 at power on connected to VDD via pull-up resistors activates SPI communication..");
    debugPrintln("This example gives SPI measurement with CRC activated.");
    debugPrintln("Set spi protocol to 0 in WE_pdmsInit() function for measurement without CRC.");
    debugPrintln("Select the right pdms sensor type in WE_pdmsInit() function. PDMS_pdus3 is selected as default.");

    /* init PDMS */
    if (false == WE_pdmsInit())
    {
        debugPrintln("**** PDMS_Init() error. STOP ****");
        WE_Delay(5);
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
void WE_pdmsSpiExampleLoop()
{
    float presskPa;
    float tempDegC;
    uint16_t syncStatusValue;

    /* Please select PDMS_SensorType_t here accordingly to convert raw values to pressure values */
    if (WE_SUCCESS == PDMS_getPressureAndTemperature_float(&pdms, pdmsSensorType, &presskPa, &tempDegC, &syncStatusValue))
    {
        debugPrintPressure_float(presskPa);
        debugPrintTemperature_float(tempDegC);
        /* Enable the below code to print the synchronized status value */
        /*
		char syncStatusValueStr[8];
		sprintf(syncStatusValueStr, "0x%04X", syncStatusValue);
		debugPrint("Status Value = ");
		debugPrintln(syncStatusValueStr);
		*/
    }
    else
    {
        debugPrintln("**** PDMS_getPressureAndTemperature_float(): Failed ****");
    }

    /* Delay of 1 second between successive measurements. */
    WE_Delay(1000);
}

/**
 * @brief Initializes the sensor for this example application.
 */
static bool WE_pdmsInit(void)
{
    /* Initialize sensor interface */
    PDMS_getDefaultInterface(&pdms);
    pdms.interfaceType = WE_spi;
    pdms.options.spi.chipSelectPort = SPI1_CS0_GPIO_Port;
    pdms.options.spi.chipSelectPin = SPI1_CS0_Pin;
    pdms.options.spi.duplexMode = 1;
    pdms.handle = &hspi1;
    /* PDMS SPI CRC selection:
	   - PDMS_SPI_withCRC: PDMS SPI measurement with CRC
	   - PDMS_SPI_withoutCRC: PDMS SPI measurement without CRC
	*/
    spiCrcSelect = PDMS_SPI_withCRC;
    pdms.options.spi.sensorSpecificSettings = &spiCrcSelect;

    /* PDMS Sensor Types and their Specifications:
	   - PDMS_pdms0: Order code 2513130810105, range = -1 to +1 kPa
	   - PDMS_pdms1: Order code 2513130810205, range = -10 to +10 kPa
	   - PDMS_pdms2: Order code 2513130835205, range = -35 to +35 kPa
	   - PDMS_pdms3: Order code 2513130810305, range =  0 to 100 kPa
	   - PDMS_pdms4: Order code 2513130810405, range = -100 to 1000 kPa
	*/
    pdmsSensorType = PDMS_pdms3;

    /* Wait for boot */
    WE_Delay(50);
    while (WE_SUCCESS != WE_isSensorInterfaceReady(&pdms))
    {
    }
    debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

    return true;
}

/**
 * @brief Prints the pressure to the debug interface.
 * @param pressureKPa  Pressure [kPa]
 */
static void debugPrintPressure_float(float pressureKPa)
{
    float pressureAbs = fabs(pressureKPa);
    uint16_t full = (uint16_t)pressureAbs;
    uint16_t decimals = (uint16_t)(((uint32_t)(pressureAbs * 10000)) % 10000); /* 4 decimal places */

    char bufferFull[6];     /* Max 5 digits + null terminator */
    char bufferDecimals[5]; /* 4 decimal places + null terminator */
    sprintf(bufferFull, "%u", full);
    sprintf(bufferDecimals, "%04u", decimals);

    debugPrint("PDMS pressure (float) = ");
    if (pressureKPa < 0)
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
 * @param tempDegC Temperature [°C]
 */
static void debugPrintTemperature_float(float tempDegC)
{
    float tempAbs = fabs(tempDegC);
    uint16_t full = (uint16_t)tempAbs;
    uint16_t decimals = ((uint16_t)(tempAbs * 100)) % 100; /* 2 decimal places */

    char bufferFull[6];     /* Max 5 digits + null terminator */
    char bufferDecimals[3]; /* 4 decimal places + null terminator */
    sprintf(bufferFull, "%u", full);
    sprintf(bufferDecimals, "%02u", decimals);

    debugPrint("PDMS temperature (float) = ");
    if (tempDegC < 0)
    {
        debugPrint("-");
    }
    debugPrint(bufferFull);
    debugPrint(".");
    debugPrint(bufferDecimals);
    debugPrintln(" degrees Celsius");
}
