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
 * @brief Header file for the WSEN-GCDS_252610104030 sensor driver.
 */

#ifndef _WSEN_GCDS_2526101040301_H
#define _WSEN_GCDS_2526101040301_H

/*         Includes         */
#include "../WeSensorsSDK.h"
#include <stdbool.h>

/*         Macros and Constants */
#define CRC8_INIT 0xFF
#define CRC8_POLYNOMIAL 0x31
#define CRC8_LEN 1
#define GCDS_WORD_SIZE 2
#define SERIAL_NO_LENGTH 6
#define FRC_CORRECTION_CONSTANT 0x8000
#define GCDS_ADDRESS 0x62 /**< GCDS I2C address */

typedef enum
{
    GCDS_START_PERIODIC_MEASUREMENT = 0x21B1,     /**< Start Periodic Measurement */
    GCDS_READ_MEASUREMENT = 0xEC05,               /**< Read Measurement */
    GCDS_STOP_PERIODIC_MEASUREMENT = 0x3F86,      /**< Stop Periodic Measurement */
    GCDS_SET_TEMPERATURE_OFFSET = 0x241D,         /**< Set Temperature Offset */
    GCDS_GET_TEMPERATURE_OFFSET = 0x2318,         /**< Get Temperature Offset */
    GCDS_SET_SENSOR_ALTITUDE = 0x2427,            /**< Set Sensor Altitude */
    GCDS_GET_SENSOR_ALTITUDE = 0x2322,            /**< Get Sensor Altitude */
    GCDS_GET_SET_AMBIENT_PRESSURE = 0xE000,       /**< Set Ambient Pressure */
    GCDS_PERFORM_MANUAL_RECALIBRATION = 0x362F,   /**< Perform Manual Re-Calibration */
    GCDS_SET_SELF_CALIB_ENABLED = 0x2416,         /**< Set Self-Calibration Enabled */
    GCDS_GET_SELF_CALIB_ENABLED = 0x2313,         /**< Get Self-Calibration Enabled */
    GCDS_SET_SELF_CALIB_TARGET = 0x243A,          /**< Set Self-Calibration Target */
    GCDS_GET_SELF_CALIB_TARGET = 0x233F,          /**< Get Self-Calibration Target */
    GCDS_START_LOW_POWER_MEASUREMENT = 0x21AC,    /**< Start Low Power Periodic Measurement */
    GCDS_GET_DATA_READY_STATUS = 0xE4B8,          /**< Get Data Ready Status */
    GCDS_PERSIST_SETTINGS = 0x3615,               /**< Persist Settings */
    GCDS_GET_SERIAL_NUMBER = 0x3682,              /**< Get Serial Number */
    GCDS_PERFORM_SELF_TEST = 0x3639,              /**< Perform Self-Test */
    GCDS_PERFORM_FACTORY_RESET = 0x3632,          /**< Perform Factory Reset */
    GCDS_REINIT = 0x3646,                         /**< Reinitialize */
    GCDS_MEASURE_SINGLE_SHOT = 0x219D,            /**< Measure Single Shot */
    GCDS_MEASURE_SINGLE_SHOT_RHT = 0x2196,        /**< Measure Single Shot RHT*/
    GCDS_POWER_DOWN = 0x36E0,                     /**< Power Down */
    GCDS_WAKE_UP = 0x36F6,                        /**< Wake Up */
    GCDS_SET_SELF_CALIB_INITIAL_PERIOD = 0x2445,  /**< Set Self-Calibration Initial Period */
    GCDS_GET_SELF_CALIB_INITIAL_PERIOD = 0x2340,  /**< Get Self-Calibration Initial Period */
    GCDS_SET_SELF_CALIB_STANDARD_PERIOD = 0x244E, /**< Set Self-Calibration Standard Period */
    GCDS_GET_SELF_CALIB_STANDARD_PERIOD = 0x234B  /**< Get Self-Calibration Standard Period */
} gcds_measureCmd_t;

#ifdef __cplusplus
extern "C"
{
#endif
    int8_t GCDS_Start_Periodic_Measurement(WE_sensorInterface_t* sensorInterface);
    int8_t GCDS_Measure_Data(WE_sensorInterface_t* sensorInterface, uint16_t* co2, int32_t* temperature, uint32_t* humidity);
    int8_t GCDS_Measure_Data_Float(WE_sensorInterface_t* sensorInterface, uint16_t* co2, float* temperature, float* humidity);
    int8_t GCDS_Stop_Periodic_Measurement(WE_sensorInterface_t* sensorInterface);
    int8_t GCDS_Set_Temperature_Offset(WE_sensorInterface_t* sensorInterface, const uint32_t* temperatureOffset);
    int8_t GCDS_Get_Temperature_Offset(WE_sensorInterface_t* sensorInterface, uint32_t* temperatureOffset);
    int8_t GCDS_Set_Sensor_Altitude(WE_sensorInterface_t* sensorInterface, const uint16_t* sensorAltitude);
    int8_t GCDS_Get_Sensor_Altitude(WE_sensorInterface_t* sensorInterface, uint16_t* sensorAltitude);
    int8_t GCDS_Set_Ambient_Pressure(WE_sensorInterface_t* sensorInterface, const uint32_t* ambientPressure);
    int8_t GCDS_Get_Ambient_Pressure(WE_sensorInterface_t* sensorInterface, uint32_t* ambientPressure);
    int8_t GCDS_Perform_Manual_Recalibration(WE_sensorInterface_t* sensorInterface, const uint16_t* co2Target, uint16_t* frcCorrection);
    int8_t GCDS_Set_Self_Calib_Enabled(WE_sensorInterface_t* sensorInterface, bool* ASCEnabled);
    int8_t GCDS_Get_Self_Calib_Enabled(WE_sensorInterface_t* sensorInterface, bool* ASCEnabled);
    int8_t GCDS_Set_Self_Calib_Target(WE_sensorInterface_t* sensorInterface, const uint16_t* co2Target);
    int8_t GCDS_Get_Self_Calib_Target(WE_sensorInterface_t* sensorInterface, uint16_t* co2Target);
    int8_t GCDS_Start_Low_Power_Measurement(WE_sensorInterface_t* sensorInterface);
    int8_t GCDS_Get_Data_Ready_Status(WE_sensorInterface_t* sensorInterface, bool* dataReady);
    int8_t GCDS_Persist_Settings(WE_sensorInterface_t* sensorInterface);
    int8_t GCDS_Read_SlNo(WE_sensorInterface_t* sensorInterface, uint8_t* serialNo);
    int8_t GCDS_Perform_Self_Test(WE_sensorInterface_t* sensorInterface, uint16_t* sensorStatus);
    int8_t GCDS_Perform_Factory_Reset(WE_sensorInterface_t* sensorInterface);
    int8_t GCDS_Reinit(WE_sensorInterface_t* sensorInterface);
    int8_t GCDS_Measure_Single_Shot(WE_sensorInterface_t* sensorInterface);
    int8_t GCDS_Measure_Single_Shot_RHT(WE_sensorInterface_t* sensorInterface);
    int8_t GCDS_Power_Down(WE_sensorInterface_t* sensorInterface);
    int8_t GCDS_Wake_Up(WE_sensorInterface_t* sensorInterface);
    int8_t GCDS_Set_Self_Calib_Init_Period(WE_sensorInterface_t* sensorInterface, const uint16_t* ascInitialPeriod);
    int8_t GCDS_Get_Self_Calib_Init_Period(WE_sensorInterface_t* sensorInterface, uint16_t* ascInitialPeriod);
    int8_t GCDS_Set_Self_Calib_Std_Period(WE_sensorInterface_t* sensorInterface, const uint16_t* ascStdPeriod);
    int8_t GCDS_Get_Self_Calib_Std_Period(WE_sensorInterface_t* sensorInterface, uint16_t* ascStdPeriod);
    int8_t GCDS_Init(WE_sensorInterface_t* sensorInterface);
    int8_t GCDS_Get_Default_Interface(WE_sensorInterface_t* sensorInterface);
#ifdef __cplusplus
}
#endif

#endif /* _WSEN_GCDS_2526101040301_H  */
