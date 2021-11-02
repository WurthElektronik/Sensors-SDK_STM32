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

#include "WSEN_PADS_2511020213301.h"

#include <stdio.h>

#include "platform.h"

/**
 * @brief Sensor interface configuration.
 * Can be set using PADS_initInterface().
 */
static WE_sensorInterface_t padsSensorInterface = {
    .sensorType = WE_PADS,
    .interfaceType = WE_i2c,
    .options = {.i2c = {.address = PADS_ADDRESS_I2C_1, .burstMode = 0, .slaveTransmitterMode = 0, .reserved = 0},
                .spi = {.chipSelectPort = 0, .chipSelectPin = 0, .burstMode = 0, .reserved = 0},
                .readTimeout = 1000,
                .writeTimeout = 1000},
    .handle = 0};

/* FIFO buffer stores pressure (3 bytes) and temperature (2 bytes) values. */
uint8_t fifoBuffer[PADS_FIFO_BUFFER_SIZE * 5] = {0};

/**
 * @brief Read data from sensor.
 *
 * @param regAdr Address of register to read from
 * @param numBytesToRead Number of bytes to be read
 * @param data Target buffer
 * @return Error Code
 */
static inline int8_t PADS_ReadReg(uint8_t regAdr,
                                  uint16_t numBytesToRead,
                                  uint8_t *data)
{
  return WE_ReadReg(&padsSensorInterface, regAdr, numBytesToRead, data);
}

/**
 * @brief Write data to sensor.
 *
 * @param regAdr Address of register to write to
 * @param numBytesToWrite Number of bytes to be written
 * @param data Source buffer
 * @return Error Code
 */
static inline int8_t PADS_WriteReg(uint8_t regAdr,
                                   uint16_t numBytesToWrite,
                                   uint8_t *data)
{
  return WE_WriteReg(&padsSensorInterface, regAdr, numBytesToWrite, data);
}

/**
 * @brief Initialize the interface of the sensor.
 *
 * Note that the sensor type can't be changed.
 *
 * @param sensorInterface Sensor interface configuration
 * @return Error code
 */
int8_t PADS_initInterface(WE_sensorInterface_t* sensorInterface)
{
  padsSensorInterface = *sensorInterface;
  padsSensorInterface.sensorType = WE_PADS;
  return WE_SUCCESS;
}

/**
 * @brief Returns the sensor interface configuration.
 * @param sensorInterface Sensor interface configuration (output parameter)
 * @return Error code
 */
int8_t PADS_getInterface(WE_sensorInterface_t* sensorInterface)
{
  *sensorInterface = padsSensorInterface;
  return WE_SUCCESS;
}

/**
 * @brief Checks if the sensor interface is ready.
 * @return WE_SUCCESS if interface is ready, WE_FAIL if not.
 */
int8_t PADS_isInterfaceReady()
{
  return WE_isSensorInterfaceReady(&padsSensorInterface);
}

/**
* @brief Read the device ID
*
* Expected value is PADS_DEVICE_ID_VALUE.
*
* @param deviceID The returned device ID.
* @retval Error code
*/
int8_t PADS_getDeviceID(uint8_t *deviceID)
{
  return PADS_ReadReg(PADS_DEVICE_ID_REG, 1, deviceID);
}

/**
* @brief Enable the AUTOREFP function
*
* Note that when enabling AUTOREFP using this function, the AUTOREFP bit
* will stay high only until the first conversion is complete. The function will remain
* turned on even if the bit is zero.
*
* The function can be turned off with PADS_resetAutoRefp().
*
* @param autoRefp Turns AUTOREFP function on
* @retval Error code
*/
int8_t PADS_enableAutoRefp(PADS_state_t autoRefp)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  interruptConfigurationReg.autoRefp = autoRefp;

  return PADS_WriteReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg);
}

/**
* @brief Check if the AUTOREFP function is currently being enabled.
*
* Note that when enabling AUTOREFP using PADS_enableAutoRefp(), the AUTOREFP bit
* will stay high only until the first conversion is complete. The function will remain
* turned on even if the bit is zero.
*
* The function can be turned off with PADS_resetAutoRefp().
*
* @param autoRefp The returned state
* @retval Error code
*/
int8_t PADS_isEnablingAutoRefp(PADS_state_t *autoRefp)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }
  *autoRefp = (PADS_state_t) interruptConfigurationReg.autoRefp;
  return WE_SUCCESS;
}

/**
* @brief Turn off the AUTOREFP function
* @param reset Reset state
* @retval Error code
*/
int8_t PADS_resetAutoRefp(PADS_state_t reset)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  interruptConfigurationReg.resetAutoRefp = reset;

  return PADS_WriteReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg);
}

/**
* @brief Enable the AUTOZERO function
*
* Note that when enabling AUTOZERO using this function, the AUTOZERO bit
* will stay high only until the first conversion is complete. The function will remain
* turned on even if the bit is zero.
*
* The function can be turned off with PADS_resetAutoZeroMode().
*
* @param autoZero Turns AUTOZERO function on
* @retval Error code
*/
int8_t PADS_enableAutoZeroMode(PADS_state_t autoZero)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  interruptConfigurationReg.autoZero = autoZero;

  return PADS_WriteReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg);
}

/**
* @brief Check if the AUTOZERO function is currently being enabled.
*
* Note that when enabling AUTOZERO using PADS_enableAutoZeroMode(), the AUTOZERO bit
* will stay high only until the first conversion is complete. The function will remain
* turned on even if the bit is zero.
*
* The function can be turned off with PADS_resetAutoZeroMode().
*
* @param autoZero The returned state
* @retval Error code
*/
int8_t PADS_isEnablingAutoZeroMode(PADS_state_t *autoZero)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  *autoZero = (PADS_state_t) interruptConfigurationReg.autoZero;

  return WE_SUCCESS;
}

/**
* @brief Turn off the AUTOZERO function
* @param reset Reset state
* @retval Error code
*/
int8_t PADS_resetAutoZeroMode(PADS_state_t reset)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  interruptConfigurationReg.resetAutoZero = reset;

  return PADS_WriteReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg);
}

/**
* @brief Enable/disable the differential pressure interrupt [enabled,disabled]
* @param diffEn Differential pressure interrupt enable state
* @retval Error code
*/
int8_t PADS_enableDiffPressureInterrupt(PADS_state_t diffEn)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  interruptConfigurationReg.diffInt = diffEn;

  return PADS_WriteReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg);
}

/**
* @brief Check if the differential pressure interrupt is enabled
* @param diffIntState The returned differential interrupt enable state
* @retval Error code
*/
int8_t PADS_isDiffPressureInterruptEnabled(PADS_state_t *diffIntState)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  *diffIntState = (PADS_state_t) interruptConfigurationReg.diffInt;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable latched interrupt [enabled, disabled]
* @param state Latched interrupt enable state
* @retval Error code
*/
int8_t PADS_enableLatchedInterrupt(PADS_state_t state)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  interruptConfigurationReg.latchedInt = state;

  return PADS_WriteReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg);
}

/**
* @brief Check if latched interrupts are enabled
* @param latchInt The returned latched interrupts enable state
* @retval Error code
*/
int8_t PADS_isLatchedInterruptEnabled(PADS_state_t *latchInt)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  *latchInt = (PADS_state_t) interruptConfigurationReg.latchedInt;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable the low pressure interrupt [enabled, disabled]
* @param state Low pressure interrupt enable state
* @retval Error code
*/
int8_t PADS_enableLowPressureInterrupt(PADS_state_t state)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  interruptConfigurationReg.lowPresInt = state;

  return PADS_WriteReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg);
}

/**
* @brief Check if the low pressure interrupt is enabled
* @param lpint The returned low pressure interrupt enable state
* @retval Error code
*/
int8_t PADS_isLowPressureInterruptEnabled(PADS_state_t *lpint)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  *lpint = (PADS_state_t) interruptConfigurationReg.lowPresInt;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable the high pressure interrupt [enabled, disabled]
* @param state High pressure interrupt enable state
* @retval Error code
*/
int8_t PADS_enableHighPressureInterrupt(PADS_state_t state)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  interruptConfigurationReg.highPresInt = state;

  return PADS_WriteReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg);
}

/**
* @brief Check if the high pressure interrupt is enabled
* @param hpint The returned high pressure interrupt enable state
* @retval Error code
*/
int8_t PADS_isHighPressureInterruptEnabled(PADS_state_t *hpint)
{
  PADS_interruptConfiguration_t interruptConfigurationReg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_CFG_REG, 1, (uint8_t *) &interruptConfigurationReg))
  {
    return WE_FAIL;
  }

  *hpint = (PADS_state_t) interruptConfigurationReg.highPresInt;

  return WE_SUCCESS;
}

/**
 * @brief Read interrupt source register
 * @param intSource The returned interrupt source register state
 * @retval Error code
 */
int8_t PADS_getInterruptSource(PADS_intSource_t *intSource)
{
  return PADS_ReadReg(PADS_INT_SOURCE_REG, 1, (uint8_t *) intSource);
}

/**
* @brief Read the state of the interrupts
* @param intState Returned state of the interrupts
* @retval Error code
*/
int8_t PADS_getInterruptStatus(PADS_state_t *intState)
{
  PADS_intSource_t int_source;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_SOURCE_REG, 1, (uint8_t *) &int_source))
  {
    return WE_FAIL;
  }

  *intState = (PADS_state_t) int_source.intStatus;

  return WE_SUCCESS;
}

/**
* @brief Read the state of the differential low pressure interrupt [not active, active]
* @param lpState The returned state of the differential low pressure interrupt
* @retval Error code
*/
int8_t PADS_getLowPressureInterruptStatus(PADS_state_t *lpState)
{
  PADS_intSource_t int_source;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_SOURCE_REG, 1, (uint8_t *) &int_source))
  {
    return WE_FAIL;
  }

  *lpState = (PADS_state_t) int_source.diffPresLowEvent;

  return WE_SUCCESS;
}

/**
* @brief Read the state of the differential high pressure interrupt [not active, active]
* @param hpState The returned state of the differential high pressure interrupt
* @retval No error
*/
int8_t PADS_getHighPressureInterruptStatus(PADS_state_t *hpState)
{
  PADS_intSource_t int_source;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_SOURCE_REG, 1, (uint8_t *) &int_source))
  {
    return WE_FAIL;
  }

  *hpState = int_source.diffPresHighEvent;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable the FIFO full interrupt
* @param fullState FIFO full interrupt enable state
* @retval Error code
*/
int8_t PADS_enableFifoFullInterrupt(PADS_state_t fullState)
{
  PADS_ctrl3_t ctrl3;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3))
  {
    return WE_FAIL;
  }

  ctrl3.fifoFullInt = fullState;

  return PADS_WriteReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3);
}

/**
* @brief Enable/disable the FIFO threshold interrupt
* @param threshState FIFO threshold interrupt enable state
* @retval Error code
*/
int8_t PADS_enableFifoThresholdInterrupt(PADS_state_t threshState)
{
  PADS_ctrl3_t ctrl3;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3))
  {
    return WE_FAIL;
  }

  ctrl3.fifoThresholdInt = threshState;

  return PADS_WriteReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3);
}

/**
* @brief Enable/disable the FIFO overrun interrupt
* @param ovrState FIFO overrun interrupt enable state
* @retval Error code
*/
int8_t PADS_enableFifoOverrunInterrupt(PADS_state_t ovrState)
{
  PADS_ctrl3_t ctrl3;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3))
  {
    return WE_FAIL;
  }

  ctrl3.fifoOverrunInt = ovrState;

  return PADS_WriteReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3);
}

/**
* @brief Check if FIFO is full [enabled, disabled]
* @param fifoFull The returned FIFO full state
* @retval Error code
*/
int8_t PADS_isFifoFull(PADS_state_t *fifoFull)
{
  PADS_fifoStatus2_t fifo_status2;

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_STATUS2_REG, 1, (uint8_t *) &fifo_status2))
  {
    return WE_FAIL;
  }

  *fifoFull = (PADS_state_t) fifo_status2.fifoFull;

  return WE_SUCCESS;
}

/**
* @brief Check if FIFO fill level has exceeded the user defined threshold [enabled, disabled]
* @param fifoWtm The returned FIFO threshold reached state
* @retval Error code
*/
int8_t PADS_isFifoThresholdReached(PADS_state_t *fifoWtm)
{
  PADS_fifoStatus2_t fifo_status2;

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_STATUS2_REG, 1, (uint8_t *) &fifo_status2))
  {
    return WE_FAIL;
  }

  *fifoWtm = (PADS_state_t) fifo_status2.fifoWtm;

  return WE_SUCCESS;
}

/**
* @brief Read the FIFO overrun state [enabled, disabled]
* @param fifoOvr The returned FIFO overrun state
* @retval Error code
*/
int8_t PADS_getFifoOverrunState(PADS_state_t *fifoOvr)
{
  PADS_fifoStatus2_t fifo_status2;

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_STATUS2_REG, 1, (uint8_t *) &fifo_status2))
  {
    return WE_FAIL;
  }

  *fifoOvr = (PADS_state_t) fifo_status2.fifoOverrun;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable the data ready signal interrupt
* @param drdy Data ready interrupt enable state
* @retval Error code
*/
int8_t PADS_enableDataReadyInterrupt(PADS_state_t drdy)
{
  PADS_ctrl3_t ctrl3;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3))
  {
    return WE_FAIL;
  }

  ctrl3.dataReadyInt = drdy;

  return PADS_WriteReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3);
}

/**
* @brief Check if the data ready signal interrupt is enabled [enabled,disabled]
* @param drdy The returned data ready interrupt enable state
* @retval Error code
*/
int8_t PADS_isDataReadyInterruptEnabled(PADS_state_t *drdy)
{
  PADS_ctrl3_t ctrl3;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3))
  {
    return WE_FAIL;
  }

  *drdy = (PADS_state_t) ctrl3.dataReadyInt;

  return WE_SUCCESS;
}

/**
* @brief Configure interrupt events (interrupt event control)
* @param ctr Interrupt event configuration
* @retval Error code
*/
int8_t PADS_setInterruptEventControl(PADS_interruptEventControl_t ctr)
{
  PADS_ctrl3_t ctrl3;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3))
  {
    return WE_FAIL;
  }

  ctrl3.intEventCtrl = ctr;

  return PADS_WriteReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3);
}

/**
* @brief Read the interrupt event configuration (interrupt event control)
* @param intEvent The returned interrupt event configuration
* @retval Error code
*/
int8_t PADS_getInterruptEventControl(PADS_interruptEventControl_t *intEvent)
{
  PADS_ctrl3_t ctrl3;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_3_REG, 1, (uint8_t *) &ctrl3))
  {
    return WE_FAIL;
  }

  *intEvent = (PADS_interruptEventControl_t) ctrl3.intEventCtrl;

  return WE_SUCCESS;
}

/**
 * @brief Set the pressure threshold (relative to reference pressure,
 * both in positive and negative direction).
 *
 * @param thresholdPa Threshold in Pa. Resolution is 6.25 Pa.
 * @retval Error code
 */
int8_t PADS_setPressureThreshold(uint32_t thresholdPa)
{
  uint32_t thresholdBits = (thresholdPa * 16) / 100;
  if (WE_FAIL == PADS_setPressureThresholdLSB((uint8_t) (thresholdBits & 0xFF)))
  {
    return WE_FAIL;
  }
  return PADS_setPressureThresholdMSB((uint8_t) ((thresholdBits >> 8) & 0xFF));
}

/**
 * @brief Read the pressure threshold (relative to reference pressure,
 * both in positive and negative direction).
 *
 * @param thresholdPa The returned threshold in Pa
 * @retval Error code
 */
int8_t PADS_getPressureThreshold(uint32_t *thresholdPa)
{
  uint8_t thrLSB, thrMSB;
  if (WE_FAIL == PADS_getPressureThresholdLSB(&thrLSB))
  {
    return WE_FAIL;
  }
  if (WE_FAIL == PADS_getPressureThresholdMSB(&thrMSB))
  {
    return WE_FAIL;
  }
  *thresholdPa = (thrLSB & 0xFF) | ((thrMSB & 0xFF) << 8);
  *thresholdPa = (*thresholdPa * 100) / 16;
  return WE_SUCCESS;
}

/**
* @brief Set the LSB pressure threshold value
*
* @see PADS_setPressureThreshold()
*
* @param thr Pressure threshold LSB
* @retval Error code
*/
int8_t PADS_setPressureThresholdLSB(uint8_t thr)
{
  return PADS_WriteReg(PADS_THR_P_L_REG, 1, &thr);
}

/**
* @brief Set the MSB pressure threshold value
*
* @see PADS_setPressureThreshold()
*
* @param thr Pressure threshold MSB
* @retval Error code
*/
int8_t PADS_setPressureThresholdMSB(uint8_t thr)
{
  return PADS_WriteReg(PADS_THR_P_H_REG, 1, &thr);
}

/**
* @brief Read the LSB pressure threshold value
*
* @see PADS_getPressureThreshold()
*
* @param thrLSB The returned pressure threshold LSB value
* @retval Error code
*/
int8_t PADS_getPressureThresholdLSB(uint8_t *thrLSB)
{
  return PADS_ReadReg(PADS_THR_P_L_REG, 1, thrLSB);
}

/**
* @brief Read the MSB pressure threshold value
*
* @see PADS_getPressureThreshold()
*
* @param thrMSB The returned pressure threshold MSB value
* @retval Error code
*/
int8_t PADS_getPressureThresholdMSB(uint8_t *thrMSB)
{
  return PADS_ReadReg(PADS_THR_P_H_REG, 1, thrMSB);
}

/**
* @brief Disable the I2C interface
* @param i2cDisable I2C interface disable state (0: I2C enabled, 1: I2C disabled)
* @retval Error code
*/
int8_t PADS_disableI2CInterface(PADS_state_t i2cDisable)
{
  PADS_interfaceCtrl_t interfaceCtrl;

  if (WE_FAIL == PADS_ReadReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl))
  {
    return WE_FAIL;
  }

  interfaceCtrl.disableI2C = i2cDisable;

  return PADS_WriteReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl);
}

/**
* @brief Read the I2C interface disable state [enabled, disabled]
* @param i2cDisabled The returned I2C interface disable state (0: I2C enabled, 1: I2C disabled)
* @retval Error code
*/
int8_t PADS_isI2CInterfaceDisabled(PADS_state_t *i2cDisabled)
{
  PADS_interfaceCtrl_t interfaceCtrl;

  if (WE_FAIL == PADS_ReadReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl))
  {
    return WE_FAIL;
  }

  *i2cDisabled = (PADS_state_t) interfaceCtrl.disableI2C;

  return WE_SUCCESS;
}


/**
* @brief Disable/enable the internal pull-down on interrupt pin
* @param pullDownState Disable pull-down state (0: PD connected; 1: PD disconnected)
* @retval Error code
*/
int8_t PADS_disablePullDownIntPin(PADS_state_t pullDownState)
{
  PADS_interfaceCtrl_t interfaceCtrl;

  if (WE_FAIL == PADS_ReadReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl))
  {
    return WE_FAIL;
  }

  interfaceCtrl.disPullDownOnIntPin = pullDownState;

  return PADS_WriteReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl);
}

/**
* @brief Read the state of the pull down on the interrupt pin
* @param pinState The returned pull-down state (0: PD connected; 1: PD disconnected)
* @retval Error code
*/
int8_t PADS_isPullDownIntDisabled(PADS_state_t *pinState)
{
  PADS_interfaceCtrl_t interfaceCtrl;

  if (WE_FAIL == PADS_ReadReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl))
  {
    return WE_FAIL;
  }

  *pinState = (PADS_state_t) interfaceCtrl.disPullDownOnIntPin;

  return WE_SUCCESS;
}

/**
* @brief Set internal pull-up on the SAO pin
* @param saoStatus SAO pull-up state
* @retval Error code
*/
int8_t PADS_setSAOPullUp(PADS_state_t saoStatus)
{
  PADS_interfaceCtrl_t interfaceCtrl;
  if (WE_FAIL == PADS_ReadReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl))
  {
    return WE_FAIL;
  }

  interfaceCtrl.pullUpOnSAOPin = saoStatus;

  return PADS_WriteReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl);
}

/**
* @brief Read the state of the pull-up on the SAO pin
* @param saoPinState The returned SAO pull-up state
* @retval Error code
*/
int8_t PADS_isSAOPullUp(PADS_state_t *saoPinState)
{
  PADS_interfaceCtrl_t interfaceCtrl;

  if (WE_FAIL == PADS_ReadReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl))
  {
    return WE_FAIL;
  }

  *saoPinState = (PADS_state_t) interfaceCtrl.pullUpOnSAOPin;

  return WE_SUCCESS;
}

/**
* @brief Set internal pull-up on the SDA pin
* @param sdaStatus SDA pull-up state
* @retval Error code
*/
int8_t PADS_setSDAPullUp(PADS_state_t sdaStatus)
{
  PADS_interfaceCtrl_t interfaceCtrl;

  if (WE_FAIL == PADS_ReadReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl))
  {
    return WE_FAIL;
  }

  interfaceCtrl.pullUpOnSDAPin = sdaStatus;

  return PADS_WriteReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl);
}

/**
* @brief Read the state of the pull-up on the SDA pin
* @param sdaPinState The returned SDA pull-up state
* @retval Error code
*/
int8_t PADS_isSDAPullUp(PADS_state_t *sdaPinState)
{
  PADS_interfaceCtrl_t interfaceCtrl;

  if (WE_FAIL == PADS_ReadReg(PADS_INTERFACE_CTRL_REG, 1, (uint8_t *) &interfaceCtrl))
  {
    return WE_FAIL;
  }

  *sdaPinState = (PADS_state_t) interfaceCtrl.pullUpOnSDAPin;

  return WE_SUCCESS;
}

/**
* @brief Set the output data rate of the sensor
* @param odr output data rate
* @retval Error code
*/
int8_t PADS_setOutputDataRate(PADS_outputDataRate_t odr)
{
  PADS_ctrl1_t ctrl1;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1))
  {
    return WE_FAIL;
  }

  ctrl1.outputDataRate = odr;

  return PADS_WriteReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

/**
* @brief Read the output data rate of the sensor
* @param odr The returned output data rate
* @retval Error code
*/
int8_t PADS_getOutputDataRate(PADS_outputDataRate_t* odr)
{
  PADS_ctrl1_t ctrl1;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1))
  {
    return WE_FAIL;
  }

  *odr = (PADS_outputDataRate_t) ctrl1.outputDataRate;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable the low pass filter
* @param filterEnabled Filter state (0: disabled, 1: enabled)
* @retval Error code
*/
int8_t PADS_enableLowPassFilter(PADS_state_t filterEnabled)
{
  PADS_ctrl1_t ctrl1;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1))
  {
    return WE_FAIL;
  }

  ctrl1.enLowPassFilter = filterEnabled;

  return PADS_WriteReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

/**
* @brief Check if the low pass filter is enabled
* @param filterEnabled The returned low pass filter enable state
* @retval Error code
*/
int8_t PADS_isLowPassFilterEnabled(PADS_state_t *filterEnabled)
{
  PADS_ctrl1_t ctrl1;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1))
  {
    return WE_FAIL;
  }

  *filterEnabled = (PADS_state_t) ctrl1.enLowPassFilter;

  return WE_SUCCESS;
}

/**
* @brief Set the low pass filter configuration
* @param conf Low pass filter configuration
* @retval Error code
*/
int8_t PADS_setLowPassFilterConfig(PADS_filterConf_t conf)
{
  PADS_ctrl1_t ctrl1;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1))
  {
    return WE_FAIL;
  }

  ctrl1.lowPassFilterConfig = conf;

  return PADS_WriteReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

/**
* @brief Read the low pass filter configuration
* @param conf The returned low pass filter configuration
* @retval Error code
*/
int8_t PADS_getLowPassFilterConfig(PADS_filterConf_t *conf)
{
  PADS_ctrl1_t ctrl1;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1))
  {
    return WE_FAIL;
  }

  *conf = (PADS_filterConf_t) ctrl1.lowPassFilterConfig;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable block data update
* @param bdu Block data update state
* @retval Error code
*/
int8_t PADS_enableBlockDataUpdate(PADS_state_t bdu)
{
  PADS_ctrl1_t ctrl1;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1))
  {
    return WE_FAIL;
  }

  ctrl1.blockDataUpdate = bdu;

  return PADS_WriteReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1);
}

/**
* @brief Check if block data update is enabled
* @param bdu The returned block data update enable state
* @retval Error code
*/
int8_t PADS_isBlockDataUpdateEnabled(PADS_state_t *bdu)
{
  PADS_ctrl1_t ctrl1;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_1_REG, 1, (uint8_t *) &ctrl1))
  {
    return WE_FAIL;
  }

  *bdu = (PADS_state_t) ctrl1.blockDataUpdate;

  return WE_SUCCESS;
}

/**
* @brief (Re)boot the device [enabled, disabled]
* @param reboot Reboot state
* @retval Error code
*/
int8_t PADS_reboot(PADS_state_t reboot)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }

  ctrl2.boot = reboot;

  return PADS_WriteReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

/**
* @brief Read the reboot state
* @param rebooting The returned reboot state.
* @retval Error code
*/
int8_t PADS_isRebooting(PADS_state_t *reboot)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }
  *reboot = (PADS_state_t) ctrl2.boot;

  return WE_SUCCESS;
}

/**
* @brief Read the boot state
* @param boot The returned Boot state
* @retval Error code
*/
int8_t PADS_getBootStatus(PADS_state_t *boot)
{
  PADS_intSource_t int_source_reg;

  if (WE_FAIL == PADS_ReadReg(PADS_INT_SOURCE_REG, 1, (uint8_t *) &int_source_reg))
  {
    return WE_FAIL;
  }

  *boot = (PADS_state_t) int_source_reg.bootOn;

  return WE_SUCCESS;
}

/**
* @brief Set the interrupt active level [active high/active low]
* @param level Interrupt active level
* @retval Error code
*/
int8_t PADS_setInterruptActiveLevel(PADS_interruptActiveLevel_t level)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }

  ctrl2.intActiveLevel = level;

  return PADS_WriteReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

/**
* @brief Read the interrupt active level
* @param level The returned interrupt active level
* @retval Error code
*/
int8_t PADS_getInterruptActiveLevel(PADS_interruptActiveLevel_t *level)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }

  *level = (PADS_interruptActiveLevel_t) ctrl2.intActiveLevel;

  return WE_SUCCESS;
}

/**
* @brief Set the interrupt pin type [push-pull/open-drain]
* @param pinType Interrupt pin type
* @retval Error code
*/
int8_t PADS_setInterruptPinType(PADS_interruptPinConfig_t pinType)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }

  ctrl2.openDrainOnINTPin = pinType;

  return PADS_WriteReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

/**
* @brief Read the interrupt pin type [push-pull/open-drain]
* @param pinType The returned interrupt pin type.
* @retval Error code
*/
int8_t PADS_getInterruptPinType(PADS_interruptPinConfig_t *pinType)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }

  *pinType = (PADS_interruptPinConfig_t) ctrl2.openDrainOnINTPin;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable the auto address increment feature
* @param autoInc Auto address increment feature enable state
* @retval Error code
*/
int8_t PADS_enableAutoIncrement(PADS_state_t autoInc)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }

  ctrl2.autoAddIncr = autoInc;

  return PADS_WriteReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

/**
* @brief Check if the auto address increment feature is enabled
* @param inc The returned auto address increment feature enable state
* @retval Error code
*/
int8_t PADS_isAutoIncrementEnabled(PADS_state_t *inc)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }

  *inc = (PADS_state_t)ctrl2.autoAddIncr;

  return WE_SUCCESS;
}

/**
* @brief Set software reset [enabled, disabled]
* @param swReset Software reset state
* @retval Error code
*/
int8_t PADS_softReset(PADS_state_t mode)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }

  ctrl2.softwareReset = mode;

  return PADS_WriteReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

/**
* @brief Read the software reset state [enabled, disabled]
* @param swReset The returned software reset state.
* @retval Error code
*/
int8_t PADS_getSoftResetState(PADS_state_t *mode)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }
  *mode = (PADS_state_t) ctrl2.softwareReset;

  return WE_SUCCESS;
}

/**
* @brief Set the power mode of the sensor [low noise, low current]
* @param mode Power mode
* @retval Error code
*/
int8_t PADS_setPowerMode(PADS_powerMode_t mode)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }

  ctrl2.lowNoiseMode = mode;

  return PADS_WriteReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

/**
* @brief Read the power mode [low noise, low current]
* @param mode The returned power mode
* @retval Error code
*/
int8_t PADS_getPowerMode(PADS_powerMode_t *mode)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }
  *mode = (PADS_powerMode_t) ctrl2.lowNoiseMode;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable the one shot mode
* @param oneShot One shot bit state
* @retval Error code
*/
int8_t PADS_enableOneShot(PADS_state_t oneShot)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }

  ctrl2.oneShotBit = oneShot;

  return PADS_WriteReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2);
}

/**
* @brief Check if one shot mode is enabled
* @param oneShot The returned one shot bit state
* @retval Error code
*/
int8_t PADS_isOneShotEnabled(PADS_state_t *oneShot)
{
  PADS_ctrl2_t ctrl2;

  if (WE_FAIL == PADS_ReadReg(PADS_CTRL_2_REG, 1, (uint8_t *) &ctrl2))
  {
    return WE_FAIL;
  }

  *oneShot = (PADS_state_t)ctrl2.oneShotBit;

  return WE_SUCCESS;
}

/**
* @brief Set LSB part of the pressure offset value
* @param offset LSB part of the pressure offset value
* @retval Error code
*/
int8_t PADS_setPressureOffsetLSB(uint8_t offset)
{
  return PADS_WriteReg(PADS_OPC_P_L_REG, 1, &offset);
}

/**
* @brief Read the LSB part of the pressure offset value
* @param offset The returned LSB part of the pressure offset value
* @retval Error code
*/
int8_t PADS_getPressureOffsetLSB(uint8_t *offset)
{
  return PADS_ReadReg(PADS_OPC_P_L_REG, 1, offset);
}

/**
* @brief Set MSB part of the pressure offset value
* @param offset MSB part of the pressure offset value
* @retval Error code
*/
int8_t PADS_setPressureOffsetMSB(uint8_t offset)
{
  return PADS_WriteReg(PADS_OPC_P_H_REG, 1, &offset);
}

/**
* @brief Read the MSB part of the pressure offset value
* @param offset The returned MSB part of the pressure offset value
* @retval Error code
*/
int8_t PADS_getPressureOffsetMSB(uint8_t *offset)
{
  return PADS_ReadReg(PADS_OPC_P_H_REG, 1, offset);
}

/**
* @brief Set the FIFO mode
* @param fifoMode FIFO mode to be set
* @retval Error code
*/
int8_t PADS_setFifoMode(PADS_fifoMode_t fifoMode)
{
  PADS_fifoCtrl_t fifoCtrlReg;

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_CTRL_REG, 1, (uint8_t *) &fifoCtrlReg))
  {
    return WE_FAIL;
  }

  fifoCtrlReg.fifoMode = fifoMode;

  return PADS_WriteReg(PADS_FIFO_CTRL_REG, 1, (uint8_t *) &fifoCtrlReg);
}

/**
* @brief Read the FIFO mode
* @param fifoMode The returned FIFO mode
* @retval Error code
*/
int8_t PADS_getFifoMode(PADS_fifoMode_t *fifoMode)
{
  PADS_fifoCtrl_t fifoCtrlReg;

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_CTRL_REG, 1, (uint8_t *) &fifoCtrlReg))
  {
    return WE_FAIL;
  }

  *fifoMode = (PADS_fifoMode_t) fifoCtrlReg.fifoMode;

  return WE_SUCCESS;
}


/**
* @brief Set stop on user-defined FIFO threshold level
* @param state Stop on FIFO threshold state
* @retval Error code
*/
int8_t PADS_enableStopOnThreshold(PADS_state_t state)
{
  PADS_fifoCtrl_t fifoCtrlReg;

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_CTRL_REG, 1, (uint8_t *) &fifoCtrlReg))
  {
    return WE_FAIL;
  }

  fifoCtrlReg.stopOnThreshold = state;

  return PADS_WriteReg(PADS_FIFO_CTRL_REG, 1, (uint8_t *) &fifoCtrlReg);
}

/**
* @brief Check if stopping on user-defined threshold level is enabled
* @param state Stop on FIFO threshold enable state
* @retval Error code
*/
int8_t PADS_isStopOnThresholdEnabled(PADS_state_t *state)
{
  PADS_fifoCtrl_t fifoCtrlReg;

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_CTRL_REG, 1, (uint8_t *) &fifoCtrlReg))
  {
    return WE_FAIL;
  }

  *state = (PADS_state_t) fifoCtrlReg.stopOnThreshold;

  return WE_SUCCESS;
}

/**
* @brief Set the FIFO threshold level
* @param fifoThr FIFO threshold level (value between 0 and 127)
* @retval Error code
*/
int8_t PADS_setFifoThreshold(uint8_t fifoThr)
{
  PADS_fifoThreshold_t fifoThresholdReg;

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_WTM_REG, 1, (uint8_t *) &fifoThresholdReg))
  {
    return WE_FAIL;
  }

  fifoThresholdReg.fifoThreshold = fifoThr;

  return PADS_WriteReg(PADS_FIFO_WTM_REG, 1, (uint8_t *) &fifoThresholdReg);
}

/**
* @brief Read the FIFO threshold level
* @param fifoThr The returned FIFO threshold level
* @retval Error code
*/
int8_t PADS_getFifoThreshold(uint8_t *fifoThr)
{
  PADS_fifoThreshold_t fifoThresholdReg;

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_WTM_REG, 1, (uint8_t *) &fifoThresholdReg))
  {
    return WE_FAIL;
  }

  *fifoThr = (uint8_t) fifoThresholdReg.fifoThreshold;

  return WE_SUCCESS;
}

/**
* @brief Read the current FIFO fill level
* @param fifoLevel The returned FIFO fill level
* @retval Error code
*/
int8_t PADS_getFifoFillLevel(uint8_t *fifoLevel)
{
  return PADS_ReadReg(PADS_FIFO_STATUS1_REG, 1, fifoLevel);
}

/**
 * @brief Read the reference pressure
 *
 * Note: The reference pressure is set automatically when enabling AUTOZERO or AUTOREFP.
 *
 * @param referencePressurePa The returned reference pressure in Pa
 * @retval Error code
 */
int8_t PADS_getReferencePressure(uint32_t *referencePressurePa)
{
  if (WE_FAIL == PADS_getRawReferencePressure(referencePressurePa))
  {
    return WE_FAIL;
  }
  *referencePressurePa = (*referencePressurePa * 100) / 4096;
  return WE_SUCCESS;
}

/**
 * @brief Read the raw reference pressure
 *
 * Note: The reference pressure is set automatically when enabling AUTOZERO or AUTOREFP.
 *
 * @param referencePressure The returned raw reference pressure.
 * @retval Error code
 */
int8_t PADS_getRawReferencePressure(uint32_t *referencePressure)
{
  uint8_t low, high;
  if (WE_FAIL == PADS_getReferencePressureLSB(&low))
  {
    return WE_FAIL;
  }
  if (WE_FAIL == PADS_getReferencePressureMSB(&high))
  {
    return WE_FAIL;
  }
  *referencePressure = (((uint32_t) high) << 16) | (((uint32_t) low) << 8);
  return WE_SUCCESS;
}

/**
* @brief Read the LSB of the reference pressure
* @param lowReferenceValue The returned reference pressure LSB
* @retval Error code
*/
int8_t PADS_getReferencePressureLSB(uint8_t *lowReferenceValue)
{
  return PADS_ReadReg(PADS_REF_P_L_REG, 1, lowReferenceValue);
}

/**
* @brief Read the MSB of the reference pressure
* @param highReferenceValue The returned reference pressure MSB
* @retval Error code
*/
int8_t PADS_getReferencePressureMSB(uint8_t *highReferenceValue)
{
  return PADS_ReadReg(PADS_REF_P_H_REG, 1, highReferenceValue);
}

/**
* @brief Check if the temperature data register has been overwritten
* @param state The returned temperature data overwritten state
* @retval Error code
*/
int8_t PADS_getTemperatureOverrunStatus(PADS_state_t *state)
{
  PADS_status_t statusReg;

  if (WE_FAIL == PADS_ReadReg(PADS_STATUS_REG, 1, (uint8_t *) &statusReg))
  {
    return WE_FAIL;
  }

  *state = (PADS_state_t) statusReg.tempDataOverrun;

  return WE_SUCCESS;
}

/**
* @brief Check if the pressure data register has been overwritten
* @param state The returned pressure data overwritten state
* @retval Error code
*/
int8_t PADS_getPressureOverrunStatus(PADS_state_t *state)
{
  PADS_status_t statusReg;

  if (WE_FAIL == PADS_ReadReg(PADS_STATUS_REG, 1, (uint8_t *) &statusReg))
  {
    return WE_FAIL;
  }

  *state = (PADS_state_t) statusReg.presDataOverrun;

  return WE_SUCCESS;
}

/**
* @brief Check if new pressure data is available
* @param state The returned pressure data availability state
* @retval Error code
*/
int8_t PADS_isPressureDataAvailable(PADS_state_t *state)
{
  PADS_status_t statusReg;

  if (WE_FAIL == PADS_ReadReg(PADS_STATUS_REG, 1, (uint8_t *) &statusReg))
  {
    return WE_FAIL;
  }
  *state = (PADS_state_t) statusReg.presDataAvailable;

  return WE_SUCCESS;
}

/**
* @brief Check if new temperature data is available
* @param state The returned temperature data availability state
* @retval Error code
*/
int8_t PADS_isTemperatureDataAvailable(PADS_state_t *state)
{
  PADS_status_t statusReg;

  if (WE_FAIL == PADS_ReadReg(PADS_STATUS_REG, 1, (uint8_t *) &statusReg))
  {
    return WE_FAIL;
  }

  *state = (PADS_state_t) statusReg.tempDataAvailable;

  return WE_SUCCESS;
}

/**
* @brief Read the raw measured pressure value
* @param rawPres The returned raw pressure
* @retval Error code
*/
int8_t PADS_getRawPressure(int32_t *rawPres)
{
  uint8_t tmp[3] = {0};

  if (WE_FAIL == PADS_ReadReg(PADS_DATA_P_XL_REG, 3, tmp))
  {
    return WE_FAIL;
  }

  *rawPres = (int32_t) (tmp[2] << 24);
  *rawPres |= (int32_t) (tmp[1] << 16);
  *rawPres |= (int32_t) (tmp[0] << 8);
  *rawPres /= 256;

  return WE_SUCCESS;
}

/**
* @brief Read the raw measured temperature value
* @param rawTemp The returned raw temperature
* @retval Error code
*/
int8_t PADS_getRawTemperature(int16_t *rawTemp)
{
  uint8_t tmp[2] = {0};

  if (WE_FAIL == PADS_ReadReg(PADS_DATA_T_L_REG, 2, tmp))
  {
    return WE_FAIL;
  }

  *rawTemp = (int16_t) (tmp[1] << 8);
  *rawTemp |= (int16_t) tmp[0];

  return WE_SUCCESS;
}

/**
* @brief Read one or more raw pressure values from FIFO
* @param numSamples Number of samples to read
* @param rawPres The returned FIFO pressure measurement(s)
* @retval Error code
*/
int8_t PADS_getFifoRawPressure(uint8_t numSamples, int32_t *rawPres)
{
  if (numSamples > PADS_FIFO_BUFFER_SIZE)
  {
    return WE_FAIL;
  }

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_DATA_P_XL_REG, 5 * numSamples, fifoBuffer))
  {
    return WE_FAIL;
  }

  uint8_t *bufferPtr = fifoBuffer;
  for (uint8_t i = 0; i < numSamples; i++, bufferPtr += 5, rawPres++)
  {
    *rawPres = (int32_t) (bufferPtr[2] << 24);
    *rawPres |= (int32_t) (bufferPtr[1] << 16);
    *rawPres |= (int32_t) (bufferPtr[0] << 8);
    *rawPres /= 256;
  }

  return WE_SUCCESS;
}

/**
* @brief Reads one or more raw temperature values from FIFO
* @param numSamples Number of samples to read
* @param rawTemp The returned FIFO temperature measurement(s)
* @retval Error code
*/
int8_t PADS_getFifoRawTemperature(uint8_t numSamples, int16_t *rawTemp)
{
  if (numSamples > PADS_FIFO_BUFFER_SIZE)
  {
    return WE_FAIL;
  }

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_DATA_P_XL_REG, 5 * numSamples, fifoBuffer))
  {
    return WE_FAIL;
  }

  uint8_t *bufferPtr = fifoBuffer;
  for (uint8_t i = 0; i < numSamples; i++, bufferPtr += 5, rawTemp++)
  {
    *rawTemp = (int16_t) (bufferPtr[4] << 8);
    *rawTemp |= (int16_t) bufferPtr[3];
  }

  return WE_SUCCESS;
}

/**
* @brief Reads one or more raw pressure and temperature values from FIFO
* @param numSamples Number of samples to read
* @param rawPres The returned FIFO pressure measurement(s)
* @param rawTemp The returned FIFO temperature measurement(s)
* @retval Error code
*/
int8_t PADS_getFifoRawValues(uint8_t numSamples, int32_t *rawPres, int16_t *rawTemp)
{
  if (numSamples > PADS_FIFO_BUFFER_SIZE)
  {
    return WE_FAIL;
  }

  if (WE_FAIL == PADS_ReadReg(PADS_FIFO_DATA_P_XL_REG, 5 * numSamples, fifoBuffer))
  {
    return WE_FAIL;
  }

  uint8_t *bufferPtr = fifoBuffer;
  for (uint8_t i = 0; i < numSamples; i++, bufferPtr += 5, rawPres++, rawTemp++)
  {
    *rawPres = (int32_t) (bufferPtr[2] << 24);
    *rawPres |= (int32_t) (bufferPtr[1] << 16);
    *rawPres |= (int32_t) (bufferPtr[0] << 8);
    *rawPres /= 256;

    *rawTemp = (int16_t) (bufferPtr[4] << 8);
    *rawTemp |= (int16_t) bufferPtr[3];
  }

  return WE_SUCCESS;
}

/**
* @brief Read the measured pressure value in Pa
*
* Note that, depending on the mode of operation, the sensor's output register
* might contain differential pressure values (e.g. if AUTOZERO is enabled).
* In that case, the function PADS_getDifferentialPressure_int() should be used.
*
* @param pressPa The returned pressure measurement
* @retval Error code
*/
int8_t PADS_getPressure_int(int32_t *pressPa)
{
  int32_t rawPressure = 0;
  if (PADS_getRawPressure(&rawPressure) == WE_SUCCESS)
  {
    *pressPa = (rawPressure * 100) / 4096;
  }
  else
  {
    return WE_FAIL;
  }
  return WE_SUCCESS;
}

/**
* @brief Read the measured differential pressure value in Pa
*
* Use this function if the sensor is configured to write differential pressure
* values to the output register (e.g. if AUTOZERO is enabled).
*
* @param pressPa The returned differential pressure measurement
* @retval Error code
*/
int8_t PADS_getDifferentialPressure_int(int32_t *pressPa)
{
  int32_t rawPressure = 0;
  if (PADS_getRawPressure(&rawPressure) == WE_SUCCESS)
  {
    *pressPa = (rawPressure * 100 * 256) / 4096;
  }
  else
  {
    return WE_FAIL;
  }
  return WE_SUCCESS;
}

/**
* @brief Read the measured temperature value in 0.01 °C
* @param temperature The returned temperature measurement
* @retval Error code
*/
int8_t PADS_getTemperature_int(int16_t *temperature)
{
  return PADS_getRawTemperature(temperature);
}

/**
 * @brief Read one or more pressure values from FIFO.
 * @param numSamples Number of samples to read
 * @param pressPa The returned FIFO pressure measurement(s) in Pa
 * @retval Error code
 */
int8_t PADS_getFifoPressure_int(uint8_t numSamples, int32_t *pressPa)
{
  if (WE_FAIL == PADS_getFifoRawPressure(numSamples, pressPa))
  {
    return WE_FAIL;
  }

  for (uint8_t i = 0; i < numSamples; i++)
  {
    pressPa[i] = (pressPa[i] * 100) / 4096;
  }

  return WE_SUCCESS;
}

/**
 * @brief Read one or more temperature values from FIFO.
 * @param numSamples Number of samples to read
 * @param temperature The returned FIFO temperature measurement(s) in 0.01 °C
 * @retval Error code
 */
int8_t PADS_getFifoTemperature_int(uint8_t numSamples, int16_t *temperature)
{
  return PADS_getFifoRawTemperature(numSamples, temperature);
}

/**
* @brief Reads one or more pressure and temperature values from FIFO
* @param numSamples Number of samples to read
* @param pressPa The returned FIFO pressure measurement(s) in Pa
* @param temperature The returned FIFO temperature measurement(s) in 0.01 °C
* @retval Error code
*/
int8_t PADS_getFifoValues_int(uint8_t numSamples, int32_t *pressPa, int16_t *temperature)
{
  if (WE_FAIL == PADS_getFifoRawValues(numSamples, pressPa, temperature))
  {
    return WE_FAIL;
  }

  for (uint8_t i = 0; i < numSamples; i++)
  {
    pressPa[i] = (pressPa[i] * 100) / 4096;
  }

  return WE_SUCCESS;
}

#ifdef WE_USE_FLOAT

/**
* @brief Read the measured pressure value in kPa
* @param presskPa The returned pressure measurement
* @retval Error code
*/
int8_t PADS_getPressure_float(float *presskPa)
{
  int32_t rawPressure = 0;
  if (PADS_getRawPressure(&rawPressure) == WE_SUCCESS)
  {
    *presskPa = (float) rawPressure;
    *presskPa = *presskPa / 40960;
  }
  else
  {
    return WE_FAIL;
  }
  return WE_SUCCESS;
}

/**
* @brief Read the measured temperature value in °C
* @param tempDegC The returned temperature measurement
* @retval Error code
*/
int8_t PADS_getTemperature_float(float *tempDegC)
{
  int16_t rawTemp = 0;
  if (PADS_getRawTemperature(&rawTemp) == WE_SUCCESS)
  {
    *tempDegC = (float) rawTemp;
    *tempDegC = *tempDegC / 100;
  }
  else
  {
    return WE_FAIL;
  }

  return WE_SUCCESS;
}

/**
* @brief Read the pressure value from FIFO in kPa
* @param presskPa The returned FIFO pressure measurement
* @retval Error code
*/
int8_t PADS_getFifoPressure_float(float *presskPa)
{
  int32_t rawPressure = 0;
  if (PADS_getFifoRawPressure(1, &rawPressure) == WE_SUCCESS)
  {
    *presskPa = (float) rawPressure;
    *presskPa = *presskPa / 40960;
  }
  else
  {
    return WE_FAIL;
  }
  return WE_SUCCESS;
}

/**
* @brief Read the temperature value from FIFO in °C
* @param tempDegC The returned FIFO temperature measurement
* @retval Error code
*/
int8_t PADS_getFifoTemperature_float(float *tempDegC)
{
  int16_t rawTemp = 0;
  if (PADS_getFifoRawTemperature(1, &rawTemp) == WE_SUCCESS)
  {
    *tempDegC = (float) rawTemp;
    *tempDegC = *tempDegC / 100;
  }
  else
  {
    return WE_FAIL;
  }

  return WE_SUCCESS;
}

#endif /* WE_USE_FLOAT */

/**         EOF         */
