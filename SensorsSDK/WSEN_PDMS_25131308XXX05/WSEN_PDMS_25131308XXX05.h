/*
 ***************************************************************************************************
 * This file is part of Sensors SDK:
 * https://www.we-online.com/sensors, https://github.com/WurthElektronik/Sensors-SDK
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
 * @brief Header file for the WSEN-PDMS-XXXXXXXXXXX05 sensor driver.
 */

#ifndef _WSEN_PDMS_25131308XXX05_H
#define _WSEN_PDMS_25131308XXX05_H

/* Includes */
#include "../WeSensorsSDK.h"

/* Macros and Constants */
#define PDMS_I2C_ADDRESS (uint8_t)0x6C          /**< PDMS I2C address without CRC */
#define PDMS_I2C_ADDRESS_CRC (uint8_t)0x6D      /**< PDMS I2C address with CRC */
#define PDMS_I2C_READ_MEASUREMENT (uint8_t)0x2E /**< PDMS I2C read measurement command*/
#define PDMS_SPI_READ_MEASUREMENT (uint8_t)0x2F /**< PDMS SPI read measurement command*/
#define CRC4_I2C_POLYNOMIAL (uint8_t)0x03       /**< CRC-4 polynomial used for I2C communication */
#define CRC4_I2C_INIT (uint8_t)0x0F             /**< Initial value for CRC-4 calculation */
#define CRC8_I2C_POLYNOMIAL (uint8_t)0xD5       /**< CRC-8 polynomial used for I2C communication */
#define CRC8_I2C_INIT (uint8_t)0xFF             /**< Initial value for CRC-8 calculation */

#define CRC4_SPI_POLYNOMIAL (uint8_t)0x03 /**< CRC-4 polynomial used for SPI communication */
#define CRC4_SPI_INIT (uint8_t)0x0F       /**< Initial value for CRC-4 calculation */
#define CRC8_SPI_POLYNOMIAL (uint8_t)0x07 /**< CRC-8 polynomial used for SPI communication */
#define CRC8_SPI_INIT (uint8_t)0xF3       /**< Initial value for CRC-8 calculation */

#define P_MIN_TYP_VAL_PDMS (uint16_t)3277  /**< Typical RAW value at minimum pressure, calibrated */
#define P_MAX_TYP_VAL_PDMS (uint16_t)29491 /**< Typical RAW value at maximum pressure, calibrated */
#define T_MIN_TYP_VAL_PDMS (uint16_t)8192  /**< Typical RAW value at minimum temperature in degrees Celsius = 0°C, calibrated */

/* PDMS Sensor types and their Specifications */
typedef enum
{
    PDMS_pdms0 = 0,       /**< Order code 2513130810105, range = -1 to + 1 kPa */
    PDMS_pdms1 = 1,       /**< Order code 2513130810205 range = -10 to + 10 kPa */
    PDMS_pdms2 = 2,       /**< Order code 2513130835205, range = -35 to + 35 kPa */
    PDMS_pdms3 = 3,       /**< Order code 2513130810305, range =  0 to 100 kPa */
    PDMS_pdms4 = 4,       /**< Order code 2513130810405  -100 to 1000 kPa  */
    PDMS_invalid = 0xFFFF /**< Invalid sensor type */
} PDMS_SensorType_t;

/* PDMS SPI read selection with CRC and without */
typedef enum
{
    PDMS_SPI_withoutCRC = 0, /**< SPI read without CRC */
    PDMS_SPI_withCRC = 1     /**< SPI read with CRC */
} PDMS_Spi_CrcSelect_t;

#ifdef __cplusplus
extern "C"
{
#endif
    /* Function definitions */
    int8_t PDMS_getDefaultInterface(WE_sensorInterface_t* sensorInterface);
    int8_t PDMS_I2C_GetRawPressureAndTemperature(WE_sensorInterface_t* sensorInterface, uint16_t* rawPressure, uint16_t* rawTemperature, uint16_t* syncStatusValue);
    int8_t PDMS_I2C_GetRawPressureAndTemperature_WithCRC(WE_sensorInterface_t* sensorInterface, uint16_t* rawPressure, uint16_t* rawTemperature, uint16_t* syncStatusValue);
    int8_t PDMS_SPI_GetRawPressureAndTemperature(WE_sensorInterface_t* sensorInterface, uint16_t* rawPressure, uint16_t* rawTemperature, uint16_t* syncStatusValue);
    int8_t PDMS_SPI_getRawPressureAndTemperature_WithCRC(WE_sensorInterface_t* sensorInterface, uint16_t* rawPressure, uint16_t* rawTemperature, uint16_t* syncStatusValue);
    int8_t PDMS_getPressureAndTemperature_float(WE_sensorInterface_t* sensorInterface, PDMS_SensorType_t type, float* pressureKPa, float* temperatureDegC, uint16_t* syncStatusValue);
#ifdef __cplusplus
}
#endif

#endif /* _WSEN_PDMS_25131308XXX05_H */
