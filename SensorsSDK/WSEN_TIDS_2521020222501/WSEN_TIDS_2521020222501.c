/**
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
 * COPYRIGHT (c) 2021 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 **/

#include "WSEN_TIDS_2521020222501.h"

#include <stdio.h>

#include "platform.h"

/**
 * @brief Sensor interface configuration.
 * Can be set using TIDS_initInterface().
 */
static WE_sensorInterface_t tidsSensorInterface = {
    .sensorType = WE_TIDS,
    .interfaceType = WE_i2c,
    .options = {.i2c = {.address = TIDS_ADDRESS_I2C_1, .burstMode = 0, .slaveTransmitterMode = 0, .reserved = 0},
                .spi = {.chipSelectPort = 0, .chipSelectPin = 0, .burstMode = 0, .reserved = 0},
                .readTimeout = 1000,
                .writeTimeout = 1000},
    .handle = 0};

/**
 * @brief Read data from sensor.
 *
 * @param regAdr Address of register to read from
 * @param numBytesToRead Number of bytes to be read
 * @param data Target buffer
 * @return Error Code
 */
static inline int8_t TIDS_ReadReg(uint8_t regAdr,
                                  uint16_t numBytesToRead,
                                  uint8_t *data)
{
  return WE_ReadReg(&tidsSensorInterface, regAdr, numBytesToRead, data);
}

/**
 * @brief Write data to sensor.
 *
 * @param regAdr Address of register to write to
 * @param numBytesToWrite Number of bytes to be written
 * @param data Source buffer
 * @return Error Code
 */
static inline int8_t TIDS_WriteReg(uint8_t regAdr,
                                   uint16_t numBytesToWrite,
                                   uint8_t *data)
{
  return WE_WriteReg(&tidsSensorInterface, regAdr, numBytesToWrite, data);
}

/**
 * @brief Initialize the interface of the sensor.
 *
 * Note that the sensor type can't be changed.
 *
 * @param sensorInterface Sensor interface configuration
 * @return Error code
 */
int8_t TIDS_initInterface(WE_sensorInterface_t* sensorInterface)
{
  tidsSensorInterface = *sensorInterface;
  tidsSensorInterface.sensorType = WE_TIDS;
  return WE_SUCCESS;
}

/**
 * @brief Returns the sensor interface configuration.
 * @param sensorInterface Sensor interface configuration (output parameter)
 * @return Error code
 */
int8_t TIDS_getInterface(WE_sensorInterface_t* sensorInterface)
{
  *sensorInterface = tidsSensorInterface;
  return WE_SUCCESS;
}

/**
 * @brief Checks if the sensor interface is ready.
 * @return WE_SUCCESS if interface is ready, WE_FAIL if not.
 */
int8_t TIDS_isInterfaceReady()
{
  return WE_isSensorInterfaceReady(&tidsSensorInterface);
}

/**
* @brief Read the device ID
*
* Expected value is TIDS_DEVICE_ID_VALUE.
*
* @param deviceID The returned device ID.
* @retval Error code
*/
int8_t TIDS_getDeviceID(uint8_t *deviceID)
{
  return TIDS_ReadReg(TIDS_DEVICE_ID_REG, 1, deviceID);
}

/**
* @brief Set software reset [enabled, disabled]
* @param swReset Software reset state
* @retval Error code
*/
int8_t TIDS_softReset(TIDS_state_t swReset)
{
  TIDS_softReset_t swRstReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_SOFT_RESET_REG, 1, (uint8_t *) &swRstReg))
  {
    return WE_FAIL;
  }

  swRstReg.reset = swReset;

  return TIDS_WriteReg(TIDS_SOFT_RESET_REG, 1, (uint8_t *) &swRstReg);
}

/**
* @brief Read the software reset state [enabled, disabled]
* @param swReset The returned software reset state.
* @retval Error code
*/
int8_t TIDS_getSoftResetState(TIDS_state_t *swReset)
{
  TIDS_softReset_t swRstReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_SOFT_RESET_REG, 1, (uint8_t *) &swRstReg))
  {
    return WE_FAIL;
  }
  
  *swReset = (TIDS_state_t) swRstReg.reset;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable continuous (free run) mode
* @param mode Continuous mode state
* @retval Error code
*/
int8_t TIDS_enableContinuousMode(TIDS_state_t mode)
{
  TIDS_ctrl_t ctrlReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg))
  {
    return WE_FAIL;
  }

  ctrlReg.freeRunBit = mode;

  return TIDS_WriteReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg);
}

/**
* @brief Check if continuous (free run) mode is enabled
* @param mode The returned continuous mode enable state
* @retval Error code
*/
int8_t TIDS_isContinuousModeEnabled(TIDS_state_t *mode)
{
  TIDS_ctrl_t ctrlReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg))
  {
    return WE_FAIL;
  }
  
  *mode = (TIDS_state_t) ctrlReg.freeRunBit;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable block data update mode
* @param bdu Block data update state
* @retval Error code
*/
int8_t TIDS_enableBlockDataUpdate(TIDS_state_t bdu)
{
  TIDS_ctrl_t ctrlReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg))
  {
    return WE_FAIL;
  }

  ctrlReg.blockDataUpdate = bdu;

  return TIDS_WriteReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg);
}

/**
* @brief Read the block data update state
* @param bdu The returned block data update state
* @retval Error code
*/
int8_t TIDS_isBlockDataUpdateEnabled(TIDS_state_t *bdu)
{
  TIDS_ctrl_t ctrlReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg))
  {
    return WE_FAIL;
  }
  
  *bdu = (TIDS_state_t) ctrlReg.blockDataUpdate;

  return WE_SUCCESS;
}

/**
* @brief Set the output data rate of the sensor
* @param odr Output data rate
* @return Error code
*/
int8_t TIDS_setOutputDataRate(TIDS_outputDataRate_t odr)
{
  TIDS_ctrl_t ctrlReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg))
  {
    return WE_FAIL;
  }

  ctrlReg.outputDataRate = odr;

  return TIDS_WriteReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg);
}

/**
* @brief Read the output data rate of the sensor
* @param odr The returned output data rate
* @return Error code
*/
int8_t TIDS_getOutputDataRate(TIDS_outputDataRate_t* odr)
{
  TIDS_ctrl_t ctrlReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg))
  {
    return WE_FAIL;
  }

  *odr = (TIDS_outputDataRate_t) ctrlReg.outputDataRate;

  return WE_SUCCESS;
}

/**
* @brief Trigger capturing of a new value in one-shot mode.
* @param oneShot One shot bit state
* @return Error code
*/
int8_t TIDS_enableOneShot(TIDS_state_t oneShot)
{
  TIDS_ctrl_t ctrlReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg))
  {
    return WE_FAIL;
  }

  ctrlReg.oneShotBit = oneShot;

  return TIDS_WriteReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg);
}

/**
* @brief Read the one shot bit state
* @param oneShot The returned one shot bit state
* @retval Error code
*/
int8_t TIDS_isOneShotEnabled(TIDS_state_t *oneShot)
{
  TIDS_ctrl_t ctrlReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg))
  {
    return WE_FAIL;
  }
  
  *oneShot = (TIDS_state_t) ctrlReg.oneShotBit;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable auto increment mode
* @param autoIncr Auto increment mode state
* @retval Error code
*/
int8_t TIDS_enableAutoIncrement(TIDS_state_t autoIncr)
{
  TIDS_ctrl_t ctrlReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg))
  {
    return WE_FAIL;
  }

  ctrlReg.autoAddIncr = autoIncr;

  return TIDS_WriteReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg);
}

/**
* @brief Read the auto increment mode state
* @param autoIncr The returned auto increment mode state
* @retval Error code
*/
int8_t TIDS_isAutoIncrementEnabled(TIDS_state_t *autoIncr)
{
  TIDS_ctrl_t ctrlReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_CTRL_REG, 1, (uint8_t *) &ctrlReg))
  {
    return WE_FAIL;
  }
  
  *autoIncr = (TIDS_state_t) ctrlReg.autoAddIncr;

  return WE_SUCCESS;
}

/**
* @brief Set upper temperature limit
* @param hLimit Upper limit
* @retval Error code
*/
int8_t TIDS_setTempHighLimit(uint8_t hLimit)
{
  return TIDS_WriteReg(TIDS_LIMIT_T_H_REG, 1, &hLimit);
}

/**
* @brief Get upper temperature limit
* @param hLimit The returned temperature high limit
* @retval Error code
*/
int8_t TIDS_getTempHighLimit(uint8_t *hLimit)
{
  return TIDS_ReadReg(TIDS_LIMIT_T_H_REG, 1, hLimit);
}

/**
* @brief Set lower temperature limit
* @param lLimit Low limit
* @retval Error code
*/
int8_t TIDS_setTempLowLimit(uint8_t lLimit)
{
  return TIDS_WriteReg(TIDS_LIMIT_T_L_REG, 1, &lLimit);
}

/**
* @brief Get lower temperature limit
* @param lLimit The returned temperature low limit
* @retval Error code
*/
int8_t TIDS_getTempLowLimit(uint8_t *lLimit)
{
  return TIDS_ReadReg(TIDS_LIMIT_T_L_REG, 1, lLimit);
}

/**
* @brief Check if the sensor is busy
* @param busy The returned busy state
* @retval Error code
*/
int8_t TIDS_isBusy(TIDS_state_t *busy)
{
  TIDS_status_t statusReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_STATUS_REG, 1, (uint8_t *) &statusReg))
  {
    return WE_FAIL;
  }
  
  *busy = (TIDS_state_t) statusReg.busy;

  return WE_SUCCESS;
}

/**
* @brief Check if upper limit has been exceeded
* @param state The returned limit exceeded state
* @retval Error code
*/
int8_t TIDS_isUpperLimitExceeded(TIDS_state_t *state)
{
  TIDS_status_t statusReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_STATUS_REG, 1, (uint8_t *) &statusReg))
  {
    return WE_FAIL;
  }
  
  *state = (TIDS_state_t) statusReg.upperLimitExceeded;

  return WE_SUCCESS;
}

/**
* @brief Check if lower limit has been exceeded
* @param state The returned limit exceeded state
* @retval Error code
*/
int8_t TIDS_isLowerLimitExceeded(TIDS_state_t *state)
{
  TIDS_status_t statusReg;

  if (WE_FAIL == TIDS_ReadReg(TIDS_STATUS_REG, 1, (uint8_t *) &statusReg))
  {
    return WE_FAIL;
  }
  
  *state = (TIDS_state_t) statusReg.lowerLimitExceeded;

  return WE_SUCCESS;
}

/**
* @brief Read the raw measured temperature value
* @param rawTemp The returned temperature measurement
* @retval Error code
*/
int8_t TIDS_getRawTemperature(int16_t *rawTemp)
{
  uint8_t tmp[2] = {0};

  if (WE_FAIL == TIDS_ReadReg(TIDS_DATA_T_L_REG, 1, &tmp[0]))
  {
    return WE_FAIL;
  }
  
  if (WE_FAIL == TIDS_ReadReg(TIDS_DATA_T_H_REG, 1, &tmp[1]))
  {
    return WE_FAIL;
  }

  *rawTemp = (int16_t)(tmp[1] << 8);
  *rawTemp |= (int16_t)tmp[0];
  return WE_SUCCESS;
}

#ifdef WE_USE_FLOAT

/**
* @brief Read the measured temperature value in °C
* @param tempdegC The returned temperature measurement
* @retval Error code
*/
int8_t TIDS_getTemperature(float *tempdegC)
{
  int16_t rawTemp = 0;
  if (WE_FAIL == TIDS_getRawTemperature(&rawTemp))
  {
    return WE_FAIL;
  }

  *tempdegC = (float) rawTemp;
  *tempdegC = *tempdegC / 100;
  return WE_SUCCESS;
}

#endif /* WE_USE_FLOAT */

/**         EOF         */
