/**
 ***************************************************************************************************
 * This file is part of Sensors SDK:
 * https://www.we-online.com/sensors, https://github.com/WurthElektronik/Sensors-SDK
 *
 * THE SOFTWARE INCLUDING THE SOURCE CODE IS PROVIDED "AS IS". YOU ACKNOWLEDGE THAT WÜRTH ELEKTRONIK
 * EISOS MAKES NO REPRESENTATIONS AND WARRANTIES OF ANY KIND RELATED TO, BUT NOT LIMITED
 * TO THE NON-INFRINGEMENT OF THIRD PARTIES' INTELLECTUAL PROPERTY RIGHTS OR THE
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
 **/

/**
 * @file
 * @brief Driver header for the WSEN-PDDS-25131310XXX01 sensor.
 */

#ifndef _WSEN_PDDS_25131310XXX01_H
#define _WSEN_PDDS_25131310XXX01_H

/* Includes */
#include "../WeSensorsSDK.h"

/* I2C Addresses */
#define PDDS_I2C_ADDRESS_0 0x6C /**< I2C address when SA0 is connected to ground */
#define PDDS_I2C_ADDRESS_1 0x6D /**< Default I2C address. SA0 is connected to VDD */
#define PDDS_I2C_DEFAULT PDDS_I2C_ADDRESS_1

/* Register Addresses */
#define PDDS_REG_IF_CTRL (0x00)    /**< Interface control (R/W) */
#define PDDS_REG_DEVICE_ID (0x01)  /**< Part ID, read-only */
#define PDDS_REG_STATUS (0x02)     /**< Status register, read-only */
#define PDDS_REG_DATA_P_MSB (0x06) /**< Pressure output [23:16] */
#define PDDS_REG_DATA_P_CSB (0x07) /**< Pressure output [15:8] */
#define PDDS_REG_DATA_P_LSB (0x08) /**< Pressure output [7:0] */
#define PDDS_REG_DATA_T_MSB (0x09) /**< Temperature output [15:8] */
#define PDDS_REG_DATA_T_LSB (0x0A) /**< Temperature output [7:0] */
#define PDDS_REG_CMD (0x30)        /**< Command register (R/W) */
#define PDDS_REG_P_CONFIG (0xA6)   /**< P_config  register (R/W) – OSR pressure channel */
#define PDDS_REG_T_CONFIG (0xA7)   /**< T_config1 register (R/W) – OSR temperature channel */

/* Constants */
#define PDDS_PRESSURE_SCALE_700KPA (8000.0f)         /**< 0 - 700 kPa  – high-pressure absolute */
#define PDDS_PRESSURE_SCALE_100KPA (64000.0f)        /**< 0 - 100 kPa  – barometric / standard  */
#define PDDS_PRESSURE_SCALE_35KPA (128000.0f)        /**< ±35 kPa       – differential / gauge   */
#define PDDS_COMBINED_CONV_TIMEOUT_DEFAULT_MS (150U) /**< T_conv_T + T_conv_P + margin at OSR 32768X */

/**
 * @brief STATUS register (0x02, Read only)
 */
typedef struct
{
    uint8_t drdy : 1;      /**< Bit  [0] data ready flag                                       */
    uint8_t reserved : 3 ; /**< Bits [3:1] reserved                                            */
    uint8_t errorCode : 4; /**< Bits [7:4] error code - non-zero indicates an electrical error */
} pdds_status_reg_t;

/**
 * @brief CMD register (0x30, R/W)
 */
typedef struct
{
    uint8_t measurement_ctrl : 3; /**< Bits [2:0] measurement mode    */
    uint8_t sco : 1;              /**< Bit  [3] start of conversion   */
    uint8_t reserved : 4;         /**< Bits [7:4] reserved            */
} pdds_cmd_reg_t;

/**
 * @brief P_config register (0xA6, R/W)
 */
typedef struct
{
    uint8_t osr_p : 3;    /**< Bits [2:0] over-sampling ratio, pressure channel */
    uint8_t reserved : 5; /**< Bits [7:3] reserved                              */
} pdds_p_config_reg_t;

/**
 * @brief T_config register (0xA7, R/W)
 */
typedef struct
{
    uint8_t osr_t : 3;    /**< Bits [2:0] over-sampling ratio, temperature channel */
    uint8_t reserved : 5; /**< Bits [7:3] reserved                                 */
} pdds_t_config_reg_t;

/**
 * @brief Measurement mode for CMD register bits [2:0]
 */
typedef enum
{
    PDDS_MEAS_COMBINED = 0x02,    /**< 3'b010: single shot mode temperature + pressure */
} pdds_measMode_t;

/**
 * @brief WSEN-PDDS sensor variant — selects the correct pressure scale factor.
 */
typedef enum
{
    PDDS_pdds0 = 0,       /**< Order code 2513131035201, range = ±35 kPa       */
    PDDS_pdds1 = 1,       /**< Order code 2513131070301, range = 0 to 100 kPa  */
    PDDS_pdds2 = 2,       /**< Order code 2513131070701, range = 0 to 700 kPa  */
    PDDS_invalid = 0xFFFF /**< Invalid sensor type                              */
} PDDS_SensorType_t;

/**
 * @brief OSR setting (shared by pressure and temperature channels)
 */
typedef enum
{
    PDDS_OSR_1024X = 0x00U,  /**< 1024X  oversampling                    */
    PDDS_OSR_2048X = 0x01U,  /**< 2048X  oversampling                    */
    PDDS_OSR_4096X = 0x02U,  /**< 4096X  oversampling                    */
    PDDS_OSR_8192X = 0x03U,  /**< 8192X  oversampling                    */
    PDDS_OSR_256X = 0x04U,   /**< 256X   oversampling                    */
    PDDS_OSR_512X = 0x05U,   /**< 512X   oversampling                    */
    PDDS_OSR_16384X = 0x06U, /**< 16384X oversampling                    */
    PDDS_OSR_32768X = 0x07U  /**< 32768X oversampling (power-on default) */
} pdds_osr_t;

#ifdef __cplusplus
extern "C"
{
#endif

/* Function definitions */
int8_t PDDS_GetDefaultInterface(WE_sensorInterface_t* sensorInterface);
int8_t PDDS_ActivateSPI(WE_sensorInterface_t* sensorInterface);
int8_t PDDS_I2C_SetMeasurementMode(WE_sensorInterface_t* sensorInterface, pdds_measMode_t mode);
int8_t PDDS_I2C_GetSingleShotRawPressureAndTemperature(WE_sensorInterface_t* sensorInterface, int32_t* rawPressure, int16_t* rawTemperature);
int8_t PDDS_I2C_SetOSR(WE_sensorInterface_t* sensorInterface, pdds_osr_t osrP, pdds_osr_t osrT);
int8_t PDDS_I2C_Wait_DataReady(WE_sensorInterface_t* sensorInterface, uint32_t timeout_ms);
int8_t PDDS_SPI_SetMeasurementMode(WE_sensorInterface_t* sensorInterface, pdds_measMode_t mode);
int8_t PDDS_SPI_GetSingleShotRawPressureAndTemperature(WE_sensorInterface_t* sensorInterface, int32_t* rawPressure, int16_t* rawTemperature);
int8_t PDDS_SPI_SetOSR(WE_sensorInterface_t* sensorInterface, pdds_osr_t osrP, pdds_osr_t osrT);
int8_t PDDS_SPI_Wait_DataReady(WE_sensorInterface_t* sensorInterface, uint32_t timeout_ms);

#ifdef WE_USE_FLOAT
int8_t PDDS_getPressureAndTemperature_float(WE_sensorInterface_t* sensorInterface, PDDS_SensorType_t sensorType, float* pressure_kPa, float* temperature_C);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _WSEN_PDDS_25131310XXX01_H */
