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
 * COPYRIGHT (c) 2022 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 */

/**
 * @file
 * @brief Driver file for the WSEN-HIDS sensor.
 */

#include "WSEN_HIDS_2523020210001.h"

#include <stdio.h>

#include "platform.h"

/**
 * @brief Sensor interface configuration.
 * Can be set using HIDS_initInterface().
 */
static WE_sensorInterface_t hidsSensorInterface = {
    .sensorType = WE_HIDS,
    .interfaceType = WE_i2c,
    .options = {.i2c = {.address = HIDS_ADDRESS_I2C_0, .burstMode = 1, .slaveTransmitterMode = 0, .useRegAddrMsbForMultiBytesRead = 1, .reserved = 0},
                .spi = {.chipSelectPort = 0, .chipSelectPin = 0, .burstMode = 0, .reserved = 0},
                .readTimeout = 1000,
                .writeTimeout = 1000},
    .handle = 0};

/**
 * @brief HIDS calibration data.
 */
typedef struct
{
  /* Is set to true when the sensor's calibration data has been read */
  uint8_t calibrationPresent;

  /* Humidity linear interpolation point H0 */
  uint8_t H0_rh;      /* H0 RH calibration data*/
  int16_t H0_T0_out;  /* H0_T0 calibration data */

  /* Humidity linear interpolation point H1 */
  uint8_t H1_rh;      /* H1 RH calibration data */
  int16_t H1_T0_out;  /* H1_T0 calibration data */

  /* Temperature linear interpolation point T0 */
  uint16_t T0_degC;
  int16_t T0_out;

  /* Temperature linear interpolation point T1 */
  uint16_t T1_degC;
  int16_t T1_out;
} HIDS_calibrationData_t;

/**
 * @brief Stores HIDS calibration data. Can be read from sensor using HIDS_readCalibrationData().
 */
static HIDS_calibrationData_t hidsCalibrationData = {0};

/* Get the calibration parameters for humidity and store the data in hidsCalibrationData */
static int8_t HIDS_get_H0_T0_out();
static int8_t HIDS_get_H1_T0_out();
static int8_t HIDS_get_H0_rh();
static int8_t HIDS_get_H1_rh();

/* Get the calibration parameters for temperature and store the data in hidsCalibrationData */
static int8_t HIDS_get_T1_OUT();
static int8_t HIDS_get_T0_OUT();
static int8_t HIDS_get_T0_degC();
static int8_t HIDS_get_T1_degC();


/**
 * @brief Read data from sensor.
 *
 * @param[in] regAdr Address of register to read from
 * @param[in] numBytesToRead Number of bytes to be read
 * @param[out] data Target buffer
 * @return Error Code
 */
static inline int8_t HIDS_ReadReg(uint8_t regAdr,
                                  uint16_t numBytesToRead,
                                  uint8_t *data)
{
  return WE_ReadReg(&hidsSensorInterface, regAdr, numBytesToRead, data);
}

/**
 * @brief Write data to sensor.
 *
 * @param[in] regAdr Address of register to write to
 * @param[in] numBytesToWrite Number of bytes to be written
 * @param[in] data Source buffer
 * @return Error Code
 */
static inline int8_t HIDS_WriteReg(uint8_t regAdr,
                                   uint16_t numBytesToWrite,
                                   uint8_t *data)
{
  return WE_WriteReg(&hidsSensorInterface, regAdr, numBytesToWrite, data);
}

/**
 * @brief Initialize the interface of the sensor.
 *
 * Note that the sensor type can't be changed.
 *
 * @param[in] sensorInterface Sensor interface configuration
 * @return Error code
 */
int8_t HIDS_initInterface(WE_sensorInterface_t* sensorInterface)
{
  hidsSensorInterface = *sensorInterface;
  hidsSensorInterface.sensorType = WE_HIDS;
  return WE_SUCCESS;
}

/**
 * @brief Returns the sensor interface configuration.
 * @param[out] sensorInterface Sensor interface configuration (output parameter)
 * @return Error code
 */
int8_t HIDS_getInterface(WE_sensorInterface_t* sensorInterface)
{
  *sensorInterface = hidsSensorInterface;
  return WE_SUCCESS;
}

/**
 * @brief Checks if the sensor interface is ready.
 * @return WE_SUCCESS if interface is ready, WE_FAIL if not.
 */
int8_t HIDS_isInterfaceReady()
{
  return WE_isSensorInterfaceReady(&hidsSensorInterface);
}

/**
* @brief Read the device ID
*
* The expected value is HIDS_DEVICE_ID_VALUE.
*
* @param[out] deviceID The returned device ID.
* @return Error code
*/
int8_t HIDS_getDeviceID(uint8_t *deviceID)
{
  return HIDS_ReadReg(HIDS_DEVICE_ID_REG, 1, deviceID);
}

/**
* @brief Set the humidity average configuration
* @param[in] avgHum Humidity average parameter
* @return Error code
*/
uint8_t HIDS_setHumidityAverageConfig(HIDS_humidityAverageConfig_t avgHum)
{
  HIDS_averageConfig_t averageReg;

  if (WE_FAIL == HIDS_ReadReg(HIDS_AVERAGE_REG, 1, (uint8_t *) &averageReg))
  {
    return WE_FAIL;
  }

  averageReg.avgHum = avgHum;

  return HIDS_WriteReg(HIDS_AVERAGE_REG, 1, (uint8_t *) &averageReg);
}

/**
* @brief Read the humidity average configuration
* @param[out] avgHum The returned humidity average configuration
* @return Error code
*/
uint8_t HIDS_getHumidityAverageConfig(HIDS_humidityAverageConfig_t *avgHum)
{
  HIDS_averageConfig_t averageReg;

  if (WE_FAIL == HIDS_ReadReg(HIDS_AVERAGE_REG, 1, (uint8_t *) &averageReg))
  {
    return WE_FAIL;
  }

  *avgHum = (HIDS_humidityAverageConfig_t) averageReg.avgHum;

  return WE_SUCCESS;
}

/**
* @brief Set the temperature average configuration
* @param[in] avgTemp Temperature average parameter
* @return Error code
*/
uint8_t HIDS_setTemperatureAverageConfig(HIDS_temperatureAverageConfig_t avgTemp)
{
  HIDS_averageConfig_t averageReg;

  if (WE_FAIL == HIDS_ReadReg(HIDS_AVERAGE_REG, 1, (uint8_t *) &averageReg))
  {
    return WE_FAIL;
  }

  averageReg.avgTemp = avgTemp;

  return HIDS_WriteReg(HIDS_AVERAGE_REG, 1, (uint8_t *) &averageReg);
}

/**
* @brief Read the temperature average configuration
* @param[out] avgTemp The returned temperature average configuration
* @return Error code
*/
uint8_t HIDS_getTemperatureAverageConfig(HIDS_temperatureAverageConfig_t *avgTemp)
{
  HIDS_averageConfig_t averageReg;

  if (WE_FAIL == HIDS_ReadReg(HIDS_AVERAGE_REG, 1, (uint8_t *) &averageReg))
  {
    return WE_FAIL;
  }

  *avgTemp = (HIDS_temperatureAverageConfig_t) averageReg.avgTemp;

  return WE_SUCCESS;
}

/**
* @brief Set the output data rate of the sensor
* @param[in] odr Output data rate
* @return Error code
*/
int8_t HIDS_setOutputDataRate(HIDS_outputDataRate_t odr)
{
  HIDS_ctrl1_t ctrlReg1;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_1, 1, (uint8_t *) &ctrlReg1))
  {
    return WE_FAIL;
  }

  ctrlReg1.odr = odr;

  return HIDS_WriteReg(HIDS_CTRL_REG_1, 1, (uint8_t *) &ctrlReg1);
}

/**
* @brief Read the output data rate of the sensor
* @param[out] odr The returned output data rate
* @return Error code
*/
int8_t HIDS_getOutputDataRate(HIDS_outputDataRate_t *odr)
{
  HIDS_ctrl1_t ctrlReg1;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_1, 1, (uint8_t *) &ctrlReg1))
  {
    return WE_FAIL;
  }

  *odr = (HIDS_outputDataRate_t) ctrlReg1.odr;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable block data update mode
* @param[in] bdu Block data update state
* @retval Error code
*/
int8_t HIDS_enableBlockDataUpdate(HIDS_state_t bdu)
{
  HIDS_ctrl1_t ctrlReg1;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_1, 1, (uint8_t *) &ctrlReg1))
  {
    return WE_FAIL;
  }

  ctrlReg1.bdu = bdu;

  return HIDS_WriteReg(HIDS_CTRL_REG_1, 1, (uint8_t *) &ctrlReg1);
}

/**
* @brief Read the block data update state
* @param[out] bdu The returned block data update state
* @return Error code
*/
int8_t HIDS_isBlockDataUpdateEnabled(HIDS_state_t *bdu)
{
  HIDS_ctrl1_t ctrlReg1;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_1, 1, (uint8_t *) &ctrlReg1))
  {
    return WE_FAIL;
  }

  *bdu = (HIDS_state_t) ctrlReg1.bdu;

  return WE_SUCCESS;
}

/**
* @brief Set the power control mode
* @param[in] pd Power control mode
* @return Error code
*/
int8_t HIDS_setPowerMode(HIDS_powerMode_t pd)
{
  HIDS_ctrl1_t ctrlReg1;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_1, 1, (uint8_t *) &ctrlReg1))
  {
    return WE_FAIL;
  }

  ctrlReg1.powerControlMode = pd;

  return HIDS_WriteReg(HIDS_CTRL_REG_1, 1, (uint8_t *) &ctrlReg1);
}

/**
* @brief Read the power control mode
* @param[out] pd The returned power control mode
* @return Error code
*/
int8_t HIDS_getPowerMode(HIDS_powerMode_t *pd)
{
  HIDS_ctrl1_t ctrlReg1;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_1, 1, (uint8_t *) &ctrlReg1))
  {
    return WE_FAIL;
  }

  *pd = (HIDS_powerMode_t) ctrlReg1.powerControlMode;

  return WE_SUCCESS;
}

/**
* @brief Trigger capturing of a new value in one-shot mode.
*
* Note: Depends on ctrl_reg_1.ODR = '00' (one-shot mode)
*
* @param[in] oneShot One shot bit state
* @return Error code
*/
int8_t HIDS_enableOneShot(HIDS_state_t oneShot)
{
  HIDS_ctrl2_t ctrlReg2;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_2, 1, (uint8_t *) &ctrlReg2))
  {
    return WE_FAIL;
  }

  ctrlReg2.oneShotBit = oneShot;

  return HIDS_WriteReg(HIDS_CTRL_REG_2, 1, (uint8_t *) &ctrlReg2);
}

/**
* @brief Read the one shot bit state
* @param[out] oneShot The returned one shot bit state
* @return Error code
*/
int8_t HIDS_isOneShotEnabled(HIDS_state_t *oneShot)
{
  HIDS_ctrl2_t ctrlReg2;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_2, 1, (uint8_t *) &ctrlReg2))
  {
    return WE_FAIL;
  }

  *oneShot = (HIDS_state_t) ctrlReg2.oneShotBit;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable the heater
* @param[in] heater Heater state
* @return Error code
*/
int8_t HIDS_enableHeater(HIDS_state_t heater)
{
  HIDS_ctrl2_t ctrlReg2;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_2, 1, (uint8_t *) &ctrlReg2))
  {
    return WE_FAIL;
  }

  ctrlReg2.heater = heater;

  return HIDS_WriteReg(HIDS_CTRL_REG_2, 1, (uint8_t *) &ctrlReg2);
}

/**
* @brief Read the heater state
* @param[out] heater The returned heater state
* @return Error code
*/
int8_t HIDS_isHeaterEnabled(HIDS_state_t *heater)
{
  HIDS_ctrl2_t ctrlReg2;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_2, 1, (uint8_t *) &ctrlReg2))
  {
    return WE_FAIL;
  }

  *heater = (HIDS_state_t) ctrlReg2.heater;

  return WE_SUCCESS;
}

/**
* @brief Enable the memory reboot
* @param[in] reboot Reboot state
* @return Error code
*/
int8_t HIDS_reboot(HIDS_state_t reboot)
{
  HIDS_ctrl2_t ctrlReg2;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_2, 1, (uint8_t *) &ctrlReg2))
  {
    return WE_FAIL;
  }

  ctrlReg2.rebootMemory = reboot;

  return HIDS_WriteReg(HIDS_CTRL_REG_2, 1, (uint8_t *) &ctrlReg2);
}

/**
* @brief Read the reboot state
* @param[out] reboot The returned reboot state
* @return Error code
*/
int8_t HIDS_isRebooting(HIDS_state_t *rebooting)
{
  HIDS_ctrl2_t ctrlReg2;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_2, 1, (uint8_t *) &ctrlReg2))
  {
    return WE_FAIL;
  }

  *rebooting = (HIDS_state_t) ctrlReg2.rebootMemory;

  return WE_SUCCESS;
}

/**
* @brief Enable/disable the data ready interrupt
* @param[in] drdy Data ready interrupt enabled/disabled
* @return Error code
*/
int8_t HIDS_enableDataReadyInterrupt(HIDS_state_t drdy)
{
  HIDS_ctrl3_t ctrlReg3;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_3, 1, (uint8_t *) &ctrlReg3))
  {
    return WE_FAIL;
  }

  ctrlReg3.enDataReady = drdy;

  return HIDS_WriteReg(HIDS_CTRL_REG_3, 1, (uint8_t *) &ctrlReg3);
}

/**
* @brief Read the data ready interrupt enable state
* @param[out] drdy The returned data ready enable state
* @return Error code
*/
int8_t HIDS_isDataReadyInterruptEnabled(HIDS_state_t *drdy)
{
  HIDS_ctrl3_t ctrlReg3;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_3, 1, (uint8_t *) &ctrlReg3))
  {
    return WE_FAIL;
  }

  *drdy = (HIDS_state_t) ctrlReg3.enDataReady;

  return WE_SUCCESS;
}

/**
* @brief Set the (data ready) interrupt pin type
* @param[in] pinConfig Interrupt pin type (push-pull / open drain)
* @return Error code
*/
int8_t HIDS_setInterruptPinType(HIDS_interruptPinConfig_t pinType)
{
  HIDS_ctrl3_t ctrlReg3;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_3, 1, (uint8_t *) &ctrlReg3))
  {
    return WE_FAIL;
  }

  ctrlReg3.interruptPinConfig = pinType;

  return HIDS_WriteReg(HIDS_CTRL_REG_3, 1, (uint8_t *) &ctrlReg3);
}

/**
* @brief Read the (data ready) interrupt pin type
* @param[out] pinConfig The returned interrupt pin type (push-pull / open drain)
* @return Error code
*/
int8_t HIDS_getInterruptPinType(HIDS_interruptPinConfig_t *pinType)
{
  HIDS_ctrl3_t ctrlReg3;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_3, 1, (uint8_t *) &ctrlReg3))
  {
    return WE_FAIL;
  }

  *pinType = (HIDS_interruptPinConfig_t) ctrlReg3.interruptPinConfig;

  return WE_SUCCESS;
}

/**
* @brief Set the (data ready) output interrupt pin level
* @param[in] level Level of output interrupt pin
* @return Error code
*/
int8_t HIDS_setInterruptActiveLevel(HIDS_interruptActiveLevel_t level)
{
  HIDS_ctrl3_t ctrlReg3;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_3, 1, (uint8_t *) &ctrlReg3))
  {
    return WE_FAIL;
  }

  ctrlReg3.drdyOutputLevel = level;

  return HIDS_WriteReg(HIDS_CTRL_REG_3, 1, (uint8_t *) &ctrlReg3);
}

/**
* @brief Read the (data ready) output interrupt pin level
* @param[out] level The returned output interrupt pin level
* @return Error code
*/
int8_t HIDS_getInterruptActiveLevel(HIDS_interruptActiveLevel_t *level)
{
  HIDS_ctrl3_t ctrlReg3;

  if (WE_FAIL == HIDS_ReadReg(HIDS_CTRL_REG_3, 1, (uint8_t *) &ctrlReg3))
  {
    return WE_FAIL;
  }

  *level = (HIDS_interruptActiveLevel_t) ctrlReg3.drdyOutputLevel;

  return WE_SUCCESS;
}

/**
* @brief Check if a new humidity data sample is available
* @param[out] state Is set to true if a new sample is available
* @return Error code
*/
int8_t HIDS_isHumidityDataAvailable(HIDS_state_t *state)
{
  HIDS_status_t status_reg;

  if (WE_FAIL == HIDS_ReadReg(HIDS_STATUS_REG, 1, (uint8_t *) &status_reg))
  {
    return WE_FAIL;
  }

  *state = (HIDS_state_t) status_reg.humDataAvailable;

  return WE_SUCCESS;
}

/**
* @brief Check if a new temperature data sample is available
* @param[out] state Is set to true if a new sample is available
* @return Error code
*/
int8_t HIDS_isTemperatureDataAvailable(HIDS_state_t *state)
{
  HIDS_status_t statusReg;

  if (WE_FAIL == HIDS_ReadReg(HIDS_STATUS_REG, 1, (uint8_t *) &statusReg))
  {
    return WE_FAIL;
  }

  *state = (HIDS_state_t) statusReg.tempDataAvailable;

  return WE_SUCCESS;
}

/**
* @brief Read a raw humidity value
* @param[out] rawHumidity The returned raw humidity
* @return Error code
*/
int8_t HIDS_getRawHumidity(int16_t *rawHumidity)
{
  uint8_t buffer[2];

  if (WE_FAIL == HIDS_ReadReg(HIDS_H_OUT_L_REG, 2, buffer))
  {
    *rawHumidity = 0;
    return WE_FAIL;
  }

  *rawHumidity = (int16_t) (buffer[1] << 8);
  *rawHumidity |= (int16_t) buffer[0];

  return WE_SUCCESS;
}

/**
* @brief Read a raw temperature value
* @param[out] rawTemp The returned raw temperature
* @return Error code
*/
int8_t HIDS_getRawTemperature(int16_t *rawTemp)
{
  uint8_t buffer[2];

  if (WE_FAIL == HIDS_ReadReg(HIDS_T_OUT_L_REG, 2, buffer))
  {
    *rawTemp = 0;
    return WE_FAIL;
  }

  *rawTemp = (int16_t) (buffer[1] << 8);
  *rawTemp |= (int16_t) buffer[0];

  return WE_SUCCESS;
}

/**
* @brief Read raw temperature and humidity values
* @param[out] rawHumidity The returned raw humidity
* @param[out] rawTemp The returned raw temperature
* @return Error code
*/
int8_t HIDS_getRawValues(int16_t *rawHumidity, int16_t *rawTemp)
{
  uint8_t buffer[4];

  if (WE_FAIL == HIDS_ReadReg(HIDS_H_OUT_L_REG, 4, buffer))
  {
    *rawHumidity = 0;
    *rawTemp = 0;
    return WE_FAIL;
  }

  *rawHumidity = (int16_t) (buffer[1] << 8);
  *rawHumidity |= (int16_t) buffer[0];

  *rawTemp = (int16_t) (buffer[3] << 8);
  *rawTemp |= (int16_t) buffer[2];

  return WE_SUCCESS;
}

#ifdef WE_USE_FLOAT

/**
* @brief Read humidity
*
* Note: Architecture must support float
*
* @param[out] humidity The returned humidity in %
* @return Error code
*/
int8_t HIDS_getHumidity_float(float *humidity)
{
  int16_t rawHumidity;
  if (WE_FAIL == HIDS_getRawHumidity(&rawHumidity))
  {
    *humidity = 0;
    return WE_FAIL;
  }
  return HIDS_convertHumidity_float(rawHumidity, humidity);
}

/**
* @brief Read the temperature
*
* Note: Architecture must support float
*
* @param[out] tempDegC The returned temperature in °C
* @return Error code
*/
int8_t HIDS_getTemperature_float(float *tempDegC)
{
  int16_t tempRaw;
  if (WE_FAIL == HIDS_getRawTemperature(&tempRaw))
  {
    *tempDegC = 0;
    return WE_FAIL;
  }
  return HIDS_convertTemperature_float(tempRaw, tempDegC);
}

/**
* @brief Convert raw humidity to humidity in %
*
* Note: Architecture must support float
*
* @param[in] rawHumidity The raw humidity to be converted
* @param[out] humidity The returned humidity in %
* @return Error code
*/
int8_t HIDS_convertHumidity_float(int16_t rawHumidity, float *humidity)
{
  if (hidsCalibrationData.calibrationPresent == 0)
  {
    if (WE_FAIL == HIDS_readCalibrationData())
    {
      *humidity = 0;
      return WE_FAIL;
    }
  }

  *humidity = ((((float) hidsCalibrationData.H1_rh - (float) hidsCalibrationData.H0_rh) * ((float) rawHumidity - (float) hidsCalibrationData.H0_T0_out))) /
      ((float) hidsCalibrationData.H1_T0_out - (float) hidsCalibrationData.H0_T0_out) + (float) hidsCalibrationData.H0_rh;

  if (*humidity > 100)
  {
    *humidity = 100;
  }
  else if (*humidity < 0)
  {
    *humidity = 0;
  }

  return WE_SUCCESS;
}

/**
* @brief Convert raw temperature to temperature in °C
*
* Note: Architecture must support float
*
* @param[in] rawTemp The raw temperature to be converted
* @param[out] tempDegC The returned temperature in °C
* @return Error code
*/
int8_t HIDS_convertTemperature_float(int16_t rawTemp, float *tempDegC)
{
  if (hidsCalibrationData.calibrationPresent == 0)
  {
    if (WE_FAIL == HIDS_readCalibrationData())
    {
      *tempDegC = 0;
      return WE_FAIL;
    }
  }

  // Decode temperature
  // Calculate temperature in degrees Celsius
  // Provide signed Celsius measurement unit

  *tempDegC = (float) (((int16_t) rawTemp - (int16_t) hidsCalibrationData.T0_out) * (float) ((int16_t) hidsCalibrationData.T1_degC - (int16_t) hidsCalibrationData.T0_degC)) /
      (float) ((int16_t) hidsCalibrationData.T1_out - (int16_t) hidsCalibrationData.T0_out) + (float) ((int16_t) hidsCalibrationData.T0_degC);

  return WE_SUCCESS;
}

#endif /* WE_USE_FLOAT */



/**
* @brief Read the humidity
* @param[out] humidity The returned humidity in 0...100 % RH
* @return Error code
*/
int8_t HIDS_getHumidity_int8(int8_t *humidity)
{
  int16_t rawHumidity;
  if (WE_FAIL == HIDS_getRawHumidity(&rawHumidity))
  {
    *humidity = 0;
    return WE_FAIL;
  }
  return HIDS_convertHumidity_int8(rawHumidity, humidity);
}

/**
* @brief Read the temperature
* @param[out] tempDegC The returned temperature in -40...+85 °C
* @return Error code
*/
int8_t HIDS_getTemperature_int8(int8_t *tempDegC)
{
  int16_t tempRaw;
  if (WE_FAIL == HIDS_getRawTemperature(&tempRaw))
  {
    *tempDegC = 0;
    return WE_FAIL;
  }
  return HIDS_convertTemperature_int8(tempRaw, tempDegC);
}

/**
* @brief Convert raw humidity to 0...100 % RH
* @param[in] rawHumidity The raw humidity to be converted
* @param[out] humidity The returned humidity in 0...100 % RH
* @return Error code
*/
int8_t HIDS_convertHumidity_int8(int16_t rawHumidity, int8_t *humidity)
{
  int32_t relHum;

  if (hidsCalibrationData.calibrationPresent == 0)
  {
    if (WE_FAIL == HIDS_readCalibrationData())
    {
      return WE_FAIL;
    }
  }

  relHum = (((int32_t) hidsCalibrationData.H1_rh - (int32_t) hidsCalibrationData.H0_rh) * ((int32_t) rawHumidity - (int32_t) hidsCalibrationData.H0_T0_out)) /
      ((int32_t) hidsCalibrationData.H1_T0_out - (int32_t) hidsCalibrationData.H0_T0_out) + (int32_t) hidsCalibrationData.H0_rh;

  if (relHum > 100)
  {
    relHum = 100;
  }
  else if (relHum < 0)
  {
    relHum = 0;
  }

  *humidity = (int8_t) relHum; // provide signed % measurement unit

  return WE_SUCCESS;
}

/**
* @brief Convert raw temperature to °C
* @param[in] rawTemp The raw temperature to be converted
* @param[out] tempDegC The returned temperature in -40...+85 °C
* @return Error code
*/
int8_t HIDS_convertTemperature_int8(int16_t rawTemp, int8_t *tempDegC)
{
  int32_t tTemp;

  if (hidsCalibrationData.calibrationPresent == 0)
  {
    if (WE_FAIL == HIDS_readCalibrationData())
    {
      *tempDegC = 0;
      return WE_FAIL;
    }
  }

  // Calculate temperature in full degrees
  tTemp = (((int32_t) rawTemp - (int32_t) hidsCalibrationData.T0_out) * ((int32_t) hidsCalibrationData.T1_degC - (int32_t) hidsCalibrationData.T0_degC)) /
      ((int32_t) hidsCalibrationData.T1_out - (int32_t) hidsCalibrationData.T0_out) + (int32_t) hidsCalibrationData.T0_degC;

  if (tTemp > 85)
  {
    tTemp = 85;
  }
  else if (tTemp < -40)
  {
    tTemp = -40;
  }

  *tempDegC = (int8_t) tTemp;

  return WE_SUCCESS;
}

/**
 * @brief Read the humidity
 * @param[out] humidity The returned humidity in 0.01%
 * @return Error code
 */
int8_t HIDS_getHumidity_uint16(uint16_t *humidity)
{
  int16_t rawHumidity;
  if (WE_FAIL == HIDS_getRawHumidity(&rawHumidity))
  {
    *humidity = 0;
    return WE_FAIL;
  }
  return HIDS_convertHumidity_uint16(rawHumidity, humidity);
}

/**
 * @brief Read the temperature
 * @param[out] temperature The returned temperature in 0.01°C
 * @return Error code
 */
int8_t HIDS_getTemperature_int16(int16_t *temperature)
{
  int16_t tempRaw;
  if (WE_FAIL == HIDS_getRawTemperature(&tempRaw))
  {
    *temperature = 0;
    return WE_FAIL;
  }
  return HIDS_convertTemperature_int16(tempRaw, temperature);
}

/**
 * @brief Convert raw humidity to 0.01%
 * @param[in] rawHumidity The raw humidity to be converted
 * @param[out] humidity The returned humidity in 0.01%
 * @return Error code
 */
int8_t HIDS_convertHumidity_uint16(int16_t rawHumidity, uint16_t *humidity)
{
  int32_t relHum;
  
  if (hidsCalibrationData.calibrationPresent == 0)
  {
    if (WE_FAIL == HIDS_readCalibrationData())
    {
      *humidity = 0;
      return WE_FAIL;
    }
  }

  // Decode Humidity
  // Calculate humidity in decimal i.e. 15.0 = 1500.

  relHum = ((((int32_t) hidsCalibrationData.H1_rh - (int32_t) hidsCalibrationData.H0_rh) * ((int32_t) rawHumidity - (int32_t) hidsCalibrationData.H0_T0_out)) * 100) /
      ((int32_t) hidsCalibrationData.H1_T0_out - (int32_t) hidsCalibrationData.H0_T0_out) + (((int32_t) hidsCalibrationData.H0_rh) * 100);

  if (relHum > 100 * 100)
  {
    *humidity = 100 * 100;
  }
  else if (relHum < 0)
  {
    *humidity = 0;
  }
  else
  {
    *humidity = (uint16_t) relHum; // provide unsigned % measurement unit
  }

  return WE_SUCCESS;
}

/**
 * @brief Convert raw temperature to 0.01°C
 * @param[in] rawTemp The raw temperature to be converted
 * @param[out] temperature The returned temperature in 0.01°C
 * @return Error code
 */
int8_t HIDS_convertTemperature_int16(int16_t rawTemp, int16_t *temperature)
{
  int32_t tTemp;

  if (hidsCalibrationData.calibrationPresent == 0)
  {
    if (WE_FAIL == HIDS_readCalibrationData())
    {
      *temperature = 0;
      return WE_FAIL;
    }
  }

  // Decode temperature
  // Calculate temperature in decimal of degree
  // centigrade i.e. 15.0 = 1500.

  tTemp = ((((int32_t) rawTemp - (int32_t) hidsCalibrationData.T0_out) * ((int32_t) hidsCalibrationData.T1_degC - (int32_t) hidsCalibrationData.T0_degC)) * 100) /
      ((int32_t) hidsCalibrationData.T1_out - (int32_t) hidsCalibrationData.T0_out) + (((int32_t) hidsCalibrationData.T0_degC) * 100);

  // provide signed celsius*100 measurement unit

  *temperature = (int16_t) tTemp;

  return WE_SUCCESS;
}


/* ********************************************************* */

/**
* @brief Get the sensor's calibration data for re-use
* @return Error code
*/
int8_t HIDS_readCalibrationData(void)
{
  /* Temperature calibration data for T0 and T1 points */
  if (WE_FAIL == HIDS_get_T0_degC())
  {
    return WE_FAIL;
  }

  if (WE_FAIL == HIDS_get_T1_degC())
  {
    return WE_FAIL;
  }

  if (WE_FAIL == HIDS_get_T0_OUT())
  {
    return WE_FAIL;
  }

  if (WE_FAIL == HIDS_get_T1_OUT())
  {
    return WE_FAIL;
  }

  /* Relative humidity calibration data for H0 and H1 points */
  if (WE_FAIL == HIDS_get_H0_rh())
  {
    return WE_FAIL;
  }

  if (WE_FAIL == HIDS_get_H1_rh())
  {
    return WE_FAIL;
  }

  if (WE_FAIL == HIDS_get_H0_T0_out())
  {
    return WE_FAIL;
  }

  if (WE_FAIL == HIDS_get_H1_T0_out())
  {
    return WE_FAIL;
  }

  hidsCalibrationData.calibrationPresent = 1;

  return WE_SUCCESS;
}

/**
* @brief Read H0_T0_out (calibration data) and store the value in hidsCalibrationData.
* @return Error code
*/
static int8_t HIDS_get_H0_T0_out()
{
  uint8_t buffer;
  int16_t temp;

  if (WE_FAIL == HIDS_ReadReg(HIDS_H0_T0_OUT_H, 1, &buffer))
  {
    return WE_FAIL;
  }

  temp = (((int16_t) buffer) << 8);

  if (WE_FAIL == HIDS_ReadReg(HIDS_H0_T0_OUT_L, 1, &buffer))
  {
    return WE_FAIL;
  }

  hidsCalibrationData.H0_T0_out = temp | buffer;

  return WE_SUCCESS;
}

/**
* @brief  Read H1_T0_out (calibration data) and store the value in hidsCalibrationData.
* @return Error code
*/
static int8_t HIDS_get_H1_T0_out()
{
  uint8_t buffer;
  int16_t temp;

  if (WE_FAIL == HIDS_ReadReg(HIDS_H1_T0_OUT_H, 1, &buffer))
  {
    return WE_FAIL;
  }

  temp = (((int16_t) buffer) << 8);

  if (WE_FAIL == HIDS_ReadReg(HIDS_H1_T0_OUT_L, 1, &buffer))
  {
    return WE_FAIL;
  }

  hidsCalibrationData.H1_T0_out = temp | buffer;

  return WE_SUCCESS;
}

/**
* @brief  Read H0_rh (calibration data) and store the value in hidsCalibrationData.
* @return Error code
*/
static int8_t HIDS_get_H0_rh()
{
  uint8_t buffer;

  if (WE_FAIL == HIDS_ReadReg(HIDS_H0_RH_X2, 1, &buffer))
  {
    return WE_FAIL;
  }

  hidsCalibrationData.H0_rh = buffer >> 1;

  return WE_SUCCESS;
}

/**
* @brief Read H1_rh (calibration data) and store the value in hidsCalibrationData.
* @return Error code
*/
static int8_t HIDS_get_H1_rh()
{
  uint8_t buffer;

  if (WE_FAIL == HIDS_ReadReg(HIDS_H1_RH_X2, 1, &buffer))
  {
    return WE_FAIL;
  }

  hidsCalibrationData.H1_rh = buffer >> 1;

  return WE_SUCCESS;
}

/**
* @brief Read T0_OUT (calibration data) and store the value in hidsCalibrationData.
* @return Error code
*/
static int8_t HIDS_get_T0_OUT()
{
  uint8_t buffer;
  int16_t temp;

  if (WE_FAIL == HIDS_ReadReg(HIDS_T0_OUT_H, 1, &buffer))
  {
    return WE_FAIL;
  }

  temp = (((int16_t) buffer) << 8);

  if (WE_FAIL == HIDS_ReadReg(HIDS_T0_OUT_L, 1, &buffer))
  {
    return WE_FAIL;
  }

  hidsCalibrationData.T0_out = temp | buffer;

  return WE_SUCCESS;
}

/**
* @brief Read T1_OUT (calibration data) and store the value in hidsCalibrationData.
* @return Error code
*/
static int8_t HIDS_get_T1_OUT()
{
  uint8_t buffer;
  int16_t temp;

  if (WE_FAIL == HIDS_ReadReg(HIDS_T1_OUT_H, 1, &buffer))
  {
    return WE_FAIL;
  }

  temp = (((int16_t) buffer) << 8);

  if (WE_FAIL == HIDS_ReadReg(HIDS_T1_OUT_L, 1, &buffer))
  {
    return WE_FAIL;
  }

  hidsCalibrationData.T1_out = temp | buffer;

  return WE_SUCCESS;
}


/**
* @brief Read T0_degC (calibration data) and store the value in hidsCalibrationData.
* @return Error code
*/
static int8_t HIDS_get_T0_degC()
{
  uint16_t T0_degC_x8_u16;
  uint8_t lsb, msb;

  /* Temperature calibration data for T0 and T1 - 2 MSBs for T0 and T1, where [0+1] = T0 MSBs and [2+3] = T1 MSBs */
  if (WE_FAIL == HIDS_ReadReg(HIDS_T0_T1_DEGC_H2, 1, &msb))
  {
    return WE_FAIL;
  }

  /* Get LSBs for T0 */
  if (WE_FAIL == HIDS_ReadReg(HIDS_T0_DEGC_X8, 1, &lsb))
  {
    return WE_FAIL;
  }

  /* Calc T0 using 8 LSBs + 2 MSBs */
  T0_degC_x8_u16 = (((uint16_t) (msb & 0x03)) << 8) | ((uint16_t) lsb);

  // Divide by 8 (=3LSBs)
  hidsCalibrationData.T0_degC = T0_degC_x8_u16 >> 3;


  return WE_SUCCESS;
}

/**
* @brief Read T1_degC (calibration data) and store the value in hidsCalibrationData.
* @return Error code
*/
static int8_t HIDS_get_T1_degC()
{
  uint16_t T1_degC_x8_u16;
  uint8_t lsb, msb;

  /* Temperature calibration data for T0 and T1 - 2 MSBs for T0 and T1, where [0+1] = T0 MSBs and [2+3] = T1 MSBs */
  if (WE_FAIL == HIDS_ReadReg(HIDS_T0_T1_DEGC_H2, 1, &msb))
  {
    return WE_FAIL;
  }

  /* Get LSBs for T1 */

  if (WE_FAIL == HIDS_ReadReg(HIDS_T1_DEGC_X8, 1, &lsb))
  {
    return WE_FAIL;
  }

  /* Calc T1 using 8 LSBs + 2 MSBs */
  T1_degC_x8_u16 = (((uint16_t) (msb & 0x0C)) << 6) | ((uint16_t) lsb);

  // Divide by 8 (=3LSBs)
  hidsCalibrationData.T1_degC = T1_degC_x8_u16 >> 3;

  return WE_SUCCESS;
}

/*         EOF         */
