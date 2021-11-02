/**
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
 * COPYRIGHT (c) 2021 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 **/

#include "WSEN_PDUS_25131308XXX01.h"

#include <stdio.h>

#include "platform.h"

/**
 * @brief Sensor interface configuration.
 * Can be set using PDUS_initInterface().
 */
static WE_sensorInterface_t pdusSensorInterface = {
    .sensorType = WE_PDUS,
    .interfaceType = WE_i2c,
    .options = {.i2c = {.address = PDUS_ADDRESS_I2C, .burstMode = 0, .slaveTransmitterMode = 1, .reserved = 0},
                .spi = {.chipSelectPort = 0, .chipSelectPin = 0, .burstMode = 0, .reserved = 0},
                .readTimeout = 1000,
                .writeTimeout = 1000},
    .handle = 0};

/**
 * @brief Read data from sensor.
 *
 * Note that this sensor doesn't have any registers to request - it
 * will simply send up to 4 bytes of data in response to any read
 * request.
 *
 * @param numBytesToRead Number of bytes to be read
 * @param data Target buffer
 * @return Error Code
 */
static inline int8_t PDUS_ReadReg(uint16_t numBytesToRead,
                                  uint8_t *data)
{
  /*
   * Caution: This sensor uses 5V Vcc and logic levels.
   * Level conversion to 3.3V is required to talk with a STM32 or any
   * other 3.3V MCU.
   * This sensor only supports I2C read operation and returns either
   * 2 or 4 bytes when the sensor address is written to the I2C bus.
   * Sending a register address is not required.
   * The first 2 bytes returned are the raw pressure value and the next 2
   * bytes are the raw temperature values.
   *
   * See chapter "reading digital output data" of the PDUS user manual for
   * the protocol.
   *
   * 1st I2C master sends sensor's I2C address (PDUS_ADDRESS_I2C) with read
   * bit set and waits for ACK by sensor.
   * 2nd I2C master read either 2 or 4 bytes back from the sensor; the slave
   * (sensor) will send up to 4 bytes.
   * Master has to ACK each byte and provide clock.
   */

  return WE_ReadReg(&pdusSensorInterface, 0, numBytesToRead, data);
}

/**
 * @brief Initialize the interface of the sensor.
 *
 * Note that the sensor type can't be changed.
 *
 * @param sensorInterface Sensor interface configuration
 * @return Error code
 */
int8_t PDUS_initInterface(WE_sensorInterface_t* sensorInterface)
{
  pdusSensorInterface = *sensorInterface;
  pdusSensorInterface.sensorType = WE_PDUS;
  return WE_SUCCESS;
}

/**
 * @brief Returns the sensor interface configuration.
 * @param sensorInterface Sensor interface configuration (output parameter)
 * @return Error code
 */
int8_t PDUS_getInterface(WE_sensorInterface_t* sensorInterface)
{
  *sensorInterface = pdusSensorInterface;
  return WE_SUCCESS;
}

/**
 * @brief Checks if the sensor interface is ready.
 * @return WE_SUCCESS if interface is ready, WE_FAIL if not.
 */
int8_t PDUS_isInterfaceReady()
{
  return WE_isSensorInterfaceReady(&pdusSensorInterface);
}

/**
* @brief Read the raw pressure
* @param data Pointer to raw pressure value (unconverted), 15 bits
* @retval Error code
*/
int8_t PDUS_getRawPressure(uint16_t *pressure)
{
  uint8_t tmp[2] = {0};

  if (WE_FAIL == PDUS_ReadReg(2, tmp))
  {
    return WE_FAIL;
  }

  *pressure = (uint16_t) ((tmp[0] & 0x7F) << 8);
  *pressure |= (uint16_t) tmp[1];

  return WE_SUCCESS;
}

/**
* @brief Read the raw pressure and temperature values
* @param pressure Pointer to raw pressure value (unconverted), 15 bits
* @param temperature Pointer to raw temperature value (unconverted), 15 bits
* @retval Error code
*/
int8_t PDUS_getRawPressureAndTemperature(uint16_t *pressure, uint16_t *temperature)
{
  uint8_t tmp[4] = {0};

  if (WE_FAIL == PDUS_ReadReg(4, tmp))
  {
    return WE_FAIL;
  }

  *pressure = (uint16_t) ((tmp[0] & 0x7F) << 8);
  *pressure |= (uint16_t) tmp[1];

  *temperature = (uint16_t) ((tmp[2] & 0x7F) << 8);
  *temperature |= (uint16_t) tmp[3];

  return WE_SUCCESS;
}

#ifdef WE_USE_FLOAT

/**
* @brief Read the pressure and temperature values
* @param type PDUS sensor type (i.e. pressure ranges) for internal conversion of pressure
* @param presskPa Pointer to pressure value
* @retval Error code
*/
int8_t PDUS_getPressure_float(PDUS_SensorType_t type, float *presskPa)
{
  uint16_t rawPres = 0;

  if (WE_FAIL == PDUS_getRawPressure(&rawPres))
  {
    return WE_FAIL;
  }

  if (rawPres < P_MIN_VAL_PDUS)
  {
    rawPres = P_MIN_VAL_PDUS;
  }

  /* Perform conversion (depending on sensor sub-type) */
  return PDUS_convertPressureToFloat(type, rawPres, presskPa);
}

/**
* @brief Read the pressure and temperature values
* @param type PDUS sensor type (i.e. pressure ranges) for internal conversion of pressure
* @param presskPa Pointer to pressure value
* @param tempDegC Pointer to temperature value
* @retval Error code
*/
int8_t PDUS_getPressureAndTemperature_float(PDUS_SensorType_t type, float *presskPa, float *tempDegC)
{
  uint16_t rawPres = 0;
  uint16_t rawTemp = 0;

  if (WE_FAIL == PDUS_getRawPressureAndTemperature(&rawPres, &rawTemp))
  {
    return WE_FAIL;
  }

  if (rawPres < P_MIN_VAL_PDUS)
  {
    rawPres = P_MIN_VAL_PDUS;
  }

  if (rawTemp < T_MIN_VAL_PDUS)
  {
    rawTemp = T_MIN_VAL_PDUS;
  }
  
  /* Apply temperature offset to raw temperature and convert to °C (0-70°C) */
  *tempDegC = (((float) (rawTemp - T_MIN_VAL_PDUS) * 4.272) / 1000);
  
  /* Perform conversion regarding sensor sub-type */
  return PDUS_convertPressureToFloat(type, rawPres, presskPa);
}

/**
 * @brief Converts a raw pressure value to kPa, depending on the PDUS sensor type.
 * @param type PDUS sensor type (i.e. pressure ranges)
 * @param rawPressure Raw pressure value as returned by the sensor
 * @param presskPa Pointer to pressure value
 * @retval Error code
 */
uint8_t PDUS_convertPressureToFloat(PDUS_SensorType_t type, uint16_t rawPressure, float *presskPa)
{
  float temp = (float) (rawPressure - P_MIN_VAL_PDUS);
  switch (type)
  {
  case PDUS_pdus0:
    *presskPa = ((temp * 7.63f) / 1000000) - 0.1f;
    break;

  case PDUS_pdus1:
    *presskPa = ((temp * 7.63f) / 100000) - 1.0f;
    break;

  case PDUS_pdus2:
    *presskPa = ((temp * 7.63f) / 10000) - 10.0f;
    break;

  case PDUS_pdus3:
    *presskPa = ((temp * 3.815f) / 1000);
    break;

  case PDUS_pdus4:
    *presskPa = ((temp * 4.196f) / 100) - 100.0f;
    break;

  default:
    return WE_FAIL;
  }
  return WE_SUCCESS;
}

#endif // WE_USE_FLOAT

/**         EOF         */
