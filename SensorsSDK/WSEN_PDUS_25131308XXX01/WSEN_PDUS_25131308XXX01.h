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
 * COPYRIGHT (c) 2022 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 */

/**
 * @file
 * @brief Header file for the WSEN-PDUS sensor driver.
 *
 * #### INFORMATIVE ####
 * The PDUS sensor has no registers to request.
 * It will automatically send up to 4 bytes as reply to any read request to it's PDUS_ADDRESS_I2C.
 * This sensor does not support write requests.
 * This sensor only has a I2C communication interface alongside the analog interface.
 */

#ifndef _WSEN_PDUS_H
#define _WSEN_PDUS_H

/*         Includes         */

#include <stdint.h>

#include "../WeSensorsSDK.h"

/*         Available PDUS I2C slave addresses         */

#define PDUS_ADDRESS_I2C    (uint8_t) 0x78

/*         Misc. defines         */

#define P_MIN_VAL_PDUS      (uint16_t) 3277     /**< Minimum raw value for pressure */
#define T_MIN_VAL_PDUS      (uint16_t) 8192     /**< Minimum raw value for temperature in degrees Celsius */

/*         Functional type definitions         */

/**
 * @brief PDUS sensor model
 */
typedef enum
{
  PDUS_pdus0,           /**< order code 2513130810001, range = -0.1 to +0.1 kPa */
  PDUS_pdus1,           /**< order code 2513130810101, range = -1 to +1 kPa */
  PDUS_pdus2,           /**< order code 2513130810201, range = -10 to +10 kPa */
  PDUS_pdus3,           /**< order code 2513130810301, range =  0 to 100 kPa */
  PDUS_pdus4,           /**< order code 2513130810401, range = -100 to +1000 kPa */
} PDUS_SensorType_t;

#ifdef __cplusplus
extern "C"
{
#endif

/*         Function definitions         */

int8_t PDUS_getDefaultInterface(WE_sensorInterface_t* sensorInterface);

int8_t PDUS_getRawPressure(WE_sensorInterface_t* sensorInterface, uint16_t *pressure);
int8_t PDUS_getRawPressureAndTemperature(WE_sensorInterface_t* sensorInterface, uint16_t *pressure, uint16_t *temperature);

#ifdef WE_USE_FLOAT
int8_t PDUS_getPressure_float(WE_sensorInterface_t* sensorInterface, PDUS_SensorType_t type, float *presskPa);
int8_t PDUS_getPressureAndTemperature_float(WE_sensorInterface_t* sensorInterface, PDUS_SensorType_t type, float *presskPa, float *tempDegC);

int8_t PDUS_convertPressureToFloat(PDUS_SensorType_t type, uint16_t rawPressure, float *presskPa);
#else
  #warning "WSEN_PDUS sensor driver: Float support is turned off by default. Define WE_USE_FLOAT to enable float support."
#endif // WE_USE_FLOAT

#ifdef __cplusplus
}
#endif

#endif /* _WSEN_PDUS_H */
