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
 * @brief Contains platform-specific functions.
 */

#include "platform.h"

#ifdef STM32L432xx
#include "stm32l4xx_hal.h"
#endif

#ifdef STM32G031xx
#include "stm32g0xx_hal.h"
#endif

#ifdef HAL_SPI_MODULE_ENABLED
#include "spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */


#ifdef HAL_I2C_MODULE_ENABLED
static HAL_StatusTypeDef I2Cx_ReadBytes(I2C_HandleTypeDef *handle,
                                        uint8_t addr,
                                        uint16_t reg,
                                        uint16_t numBytesToRead,
                                        uint8_t slaveTransmitterMode,
                                        uint16_t timeout,
                                        uint8_t *value);
static HAL_StatusTypeDef I2Cx_WriteBytes(I2C_HandleTypeDef *handle,
                                         uint8_t addr,
                                         uint16_t reg,
                                         uint16_t numBytesToWrite,
                                         uint16_t timeout,
                                         uint8_t *value);
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
static HAL_StatusTypeDef SPIx_ReadBytes(SPI_HandleTypeDef *handle,
                                        GPIO_TypeDef* chipSelectPort,
                                        uint16_t chipSelectPin,
                                        uint8_t reg,
                                        uint16_t numBytesToRead,
                                        uint16_t timeout,
                                        uint8_t *value);
static HAL_StatusTypeDef SPIx_WriteBytes(SPI_HandleTypeDef *handle,
                                         GPIO_TypeDef* chipSelectPort,
                                         uint16_t chipSelectPin,
                                         uint8_t reg,
                                         uint16_t numBytesToWrite,
                                         uint16_t timeout,
                                         uint8_t *value);
#endif /* HAL_SPI_MODULE_ENABLED */

/**
 * @brief Read data starting from the addressed register
 * @param[in] interface Sensor interface
 * @param[in] regAdr The register address to read from
 * @param[in] numBytesToRead Number of bytes to read
 * @param[out] data The read data will be stored here
 * @retval Error code
 */
inline int8_t WE_ReadReg(WE_sensorInterface_t *interface,
                         uint8_t regAdr,
                         uint16_t numBytesToRead,
                         uint8_t *data)
{
  HAL_StatusTypeDef status = HAL_OK;

  switch (interface->interfaceType)
  {
  case WE_i2c_fifo:
#ifdef HAL_I2C_MODULE_ENABLED
	if (interface->options.i2c.burstMode != 0 || numBytesToRead == 1)
	{
	  if (numBytesToRead > 1 && interface->options.i2c.useRegAddrMsbForMultiBytesRead)
	  {
		/* Register address most significant bit is used to enable multi bytes read */
		regAdr |= 1 << 7;
	  }
	  status = I2Cx_ReadBytes((I2C_HandleTypeDef*) interface->handle,
							  interface->options.i2c.address << 1, /* stm32 needs shifted value */
							  (uint16_t) regAdr,
							  numBytesToRead,
							  interface->options.i2c.slaveTransmitterMode,
							  interface->options.readTimeout,
							  data);
	}
#else
    status = HAL_ERROR;
#endif /* HAL_I2C_MODULE_ENABLED */
    break;

  case WE_i2c:
#ifdef HAL_I2C_MODULE_ENABLED
    if (interface->options.i2c.burstMode != 0 || numBytesToRead == 1)
    {
      if (numBytesToRead > 1 && interface->options.i2c.useRegAddrMsbForMultiBytesRead)
      {
        /* Register address most significant bit is used to enable multi bytes read */
        regAdr |= 1 << 7;
      }
      status = I2Cx_ReadBytes((I2C_HandleTypeDef*) interface->handle,
                              interface->options.i2c.address << 1, /* stm32 needs shifted value */
                              (uint16_t) regAdr,
                              numBytesToRead,
                              interface->options.i2c.slaveTransmitterMode,
                              interface->options.readTimeout,
                              data);
    }
    else
    {
      for (uint16_t i = 0; (i < numBytesToRead) && (status == HAL_OK); i++)
      {
        status = I2Cx_ReadBytes((I2C_HandleTypeDef*) interface->handle,
                                interface->options.i2c.address << 1, /* stm32 needs shifted value */
                                regAdr + i,
                                1,
                                interface->options.i2c.slaveTransmitterMode,
                                interface->options.readTimeout,
                                data + i);
      }
    }
#else
    status = HAL_ERROR;
#endif /* HAL_I2C_MODULE_ENABLED */
    break;

  case WE_spi:
#ifdef HAL_SPI_MODULE_ENABLED
    if (interface->options.spi.burstMode != 0 || numBytesToRead == 1)
    {
      status = SPIx_ReadBytes((SPI_HandleTypeDef*) interface->handle,
                              (GPIO_TypeDef*) interface->options.spi.chipSelectPort,
                              interface->options.spi.chipSelectPin,
                              regAdr,
                              numBytesToRead,
                              interface->options.readTimeout,
                              data);
    }
    else
    {
      for (uint16_t i = 0; (i < numBytesToRead) && (status == HAL_OK); i++)
      {
        status = SPIx_ReadBytes((SPI_HandleTypeDef*) interface->handle,
                                (GPIO_TypeDef*) interface->options.spi.chipSelectPort,
                                interface->options.spi.chipSelectPin,
                                regAdr + i,
                                1,
                                interface->options.readTimeout,
                                data + i);
      }
    }
#else
    status = HAL_ERROR;
#endif /* HAL_SPI_MODULE_ENABLED */
    break;


  default:
	  status = HAL_ERROR;
	  break;
  }

  return status == HAL_OK ? WE_SUCCESS : WE_FAIL;
}


/**
 * @brief Write data starting from the addressed register
 * @param[in] interface Sensor interface
 * @param[in] regAdr Address of register to be written
 * @param[in] numBytesToWrite Number of bytes to write
 * @param[in] data Data to be written
 * @retval Error code
 */
inline int8_t WE_WriteReg(WE_sensorInterface_t *interface,
                          uint8_t regAdr,
                          uint16_t numBytesToWrite,
                          uint8_t *data)
{
  HAL_StatusTypeDef status = HAL_OK;

  switch (interface->interfaceType)
  {


  case WE_i2c_fifo:
#ifdef HAL_I2C_MODULE_ENABLED
	  status = HAL_I2C_Master_Transmit(interface->handle,
			                          interface->options.i2c.address << 1,
									  data,
									  numBytesToWrite,
									  interface->options.writeTimeout);
#else
    status = HAL_ERROR;
#endif /* HAL_I2C_MODULE_ENABLED */
	  break;

  case WE_i2c:
#ifdef HAL_I2C_MODULE_ENABLED
    if (interface->options.i2c.burstMode != 0 || numBytesToWrite == 1)
    {
      status = I2Cx_WriteBytes((I2C_HandleTypeDef*) interface->handle,
                               interface->options.i2c.address << 1, /* stm32 needs shifted value */
                               regAdr,
                               numBytesToWrite,
                               interface->options.writeTimeout,
                               data);
    }
    else
    {
      for (uint16_t i = 0; (i < numBytesToWrite) && (status == HAL_OK); i++)
      {
        status = I2Cx_WriteBytes((I2C_HandleTypeDef*) interface->handle,
                                 interface->options.i2c.address << 1, /* stm32 needs shifted value */
                                 regAdr + i,
                                 1,
                                 interface->options.writeTimeout,
                                 data + i);
      }
    }
#else
    status = HAL_ERROR;
#endif /* HAL_I2C_MODULE_ENABLED */
    break;

  case WE_spi:
#ifdef HAL_SPI_MODULE_ENABLED
    if (interface->options.spi.burstMode != 0 || numBytesToWrite == 1)
    {
      status = SPIx_WriteBytes((SPI_HandleTypeDef*) interface->handle,
                               (GPIO_TypeDef*) interface->options.spi.chipSelectPort,
                               interface->options.spi.chipSelectPin,
                               regAdr,
                               numBytesToWrite,
                               interface->options.writeTimeout,
                               data);
    }
    else
    {
      for (uint16_t i = 0; (i < numBytesToWrite) && (status == HAL_OK); i++)
      {
        status = SPIx_WriteBytes((SPI_HandleTypeDef*) interface->handle,
                                 (GPIO_TypeDef*) interface->options.spi.chipSelectPort,
                                 interface->options.spi.chipSelectPin,
                                 regAdr + i,
                                 1,
                                 interface->options.writeTimeout,
                                 data + i);
      }
    }
#else
    status = HAL_ERROR;
#endif /* HAL_SPI_MODULE_ENABLED */
    break;

  default:
	  status = HAL_ERROR;
	  break;
  }

  return status == HAL_OK ? WE_SUCCESS : WE_FAIL;
}

/**
 * @brief Checks if the sensor interface is ready.
 * @param[in] interface Sensor interface
 * @return WE_SUCCESS if interface is ready, WE_FAIL if not.
 */
int8_t WE_isSensorInterfaceReady(WE_sensorInterface_t* interface)
{
  switch (interface->interfaceType)
  {
  case WE_i2c:
#ifdef HAL_I2C_MODULE_ENABLED
    return (HAL_OK == HAL_I2C_IsDeviceReady((I2C_HandleTypeDef*) interface->handle,
                                            interface->options.i2c.address << 1,
                                            64,
                                            5000)) ? WE_SUCCESS : WE_FAIL;
#else
    return WE_FAIL;
#endif

  case WE_spi:
#ifdef HAL_SPI_MODULE_ENABLED
    return WE_SUCCESS;
#else
    return WE_FAIL;
#endif

  default:
    return WE_FAIL;
  }
}


#ifdef HAL_I2C_MODULE_ENABLED

/**
 * @brief Reads bytes from I2C
 * @param[in] handle I2C handle
 * @param[in] addr I2C address
 * @param[in] reg Register address
 * @param[in] numBytesToRead Number of bytes to read
 * @param[in] slaveTransmitterMode Enables slave-transmitter mode (read-only, polling mode IO operation), slaveTransmitterMode = 1 is only required for WSEN-PDUS operation, other sensors use 0.
 * @param[in] timeout Timeout for read operation
 * @param[out] value Pointer to data buffer
 * @retval HAL status
 */
static HAL_StatusTypeDef I2Cx_ReadBytes(I2C_HandleTypeDef *handle,
                                        uint8_t addr,
                                        uint16_t reg,
                                        uint16_t numBytesToRead,
                                        uint8_t slaveTransmitterMode,
                                        uint16_t timeout,
                                        uint8_t *value)
{
  if (slaveTransmitterMode == 0)
  {
    return HAL_I2C_Mem_Read(handle,
                            addr,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            numBytesToRead,
                            timeout);
  }
  else
  {
    return HAL_I2C_Master_Receive(handle,
                                  addr,
                                  value,
                                  numBytesToRead,
                                  timeout);
  }
}


/**
 * @brief Writes bytes to I2C.
 * @param[in] handle I2C handle
 * @param[in] addr I2C address
 * @param[in] reg The target register address to write
 * @param[in] numBytesToWrite Number of bytes to write
 * @param[in] timeout Timeout for write operation
 * @param[in] value The target register value to be written
 * @retval HAL status
 */
static HAL_StatusTypeDef I2Cx_WriteBytes(I2C_HandleTypeDef *handle,
                                         uint8_t addr,
                                         uint16_t reg,
                                         uint16_t numBytesToWrite,
                                         uint16_t timeout,
                                         uint8_t *value)
{
  return HAL_I2C_Mem_Write(handle,
                           addr,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           value,
                           numBytesToWrite,
                           timeout);
}

#endif /* HAL_I2C_MODULE_ENABLED */

/**
 * @brief Provides delay
 * @param[in] Delay in milliseconds
 */
void WE_Delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}


#ifdef HAL_SPI_MODULE_ENABLED
/**
 * @brief Reads bytes from SPI.
 * @param[in] handle SPI handle
 * @param[in] chipSelectPort Port of pin used for chip select
 * @param[in] chipSelectPin Pin used for chip select
 * @param[in] reg Register address
 * @param[in] numBytesToRead Number of bytes to read
 * @param[in] timeout Timeout for read operation
 * @param[out] value Pointer to data buffer
 * @retval HAL status
 */
static HAL_StatusTypeDef SPIx_ReadBytes(SPI_HandleTypeDef *handle,
                                        GPIO_TypeDef* chipSelectPort,
                                        uint16_t chipSelectPin,
                                        uint8_t reg,
                                        uint16_t numBytesToRead,
                                        uint16_t timeout,
                                        uint8_t *value)
{
  HAL_StatusTypeDef status = HAL_OK;
  HAL_GPIO_WritePin(chipSelectPort, chipSelectPin, GPIO_PIN_RESET);
  /* Bit 7 has to be 0 for write and 1 for read operations */
  uint8_t header = reg | (1 << 7);
  /* For some sensors (e.g. HIDS) Bit 6 is used for auto-increment (MS) in multiple reads/writes (1: enabled, 0: disabled) */
//  if (numBytesToRead > 1)
//  {
//    header |= 1 << 6;
//  }
  status = HAL_SPI_Transmit(handle, &header, 1, timeout);
  if (status != HAL_OK)
  {
    return status;
  }
  status = HAL_SPI_Receive(handle, value, numBytesToRead, timeout);
  HAL_GPIO_WritePin(chipSelectPort, chipSelectPin, GPIO_PIN_SET);
  return status;
}



/**
 * @brief Writes bytes to SPI.
 * @param[in] handle SPI handle
 * @param[in] chipSelectPort Port of pin used for chip select
 * @param[in] chipSelectPin Pin used for chip select
 * @param[in] reg The target register address to write
 * @param[in] numBytesToWrite Number of bytes to write
 * @param[in] timeout Timeout for write operation
 * @param[in] value The target register value to be written
 * @retval HAL status
 */
static HAL_StatusTypeDef SPIx_WriteBytes(SPI_HandleTypeDef *handle,
                                         GPIO_TypeDef* chipSelectPort,
                                         uint16_t chipSelectPin,
                                         uint8_t reg,
                                         uint16_t numBytesToWrite,
                                         uint16_t timeout,
                                         uint8_t *value)
{
  HAL_StatusTypeDef status = HAL_OK;
  HAL_GPIO_WritePin(chipSelectPort, chipSelectPin, GPIO_PIN_RESET);
  /* Bit 7 has to be 0 for write and 1 for read operations */
  uint8_t header = reg & ~(1 << 7);
  /* For some sensors (e.g. HIDS) Bit 6 is used for auto-increment (MS) in multiple reads/writes (1: enabled, 0: disabled) */
//  if (numBytesToWrite > 1)
//  {
//    header |= 1 << 6;
//  }
  status = HAL_SPI_Transmit(handle, &header, 1, timeout);
  if (status != HAL_OK)
  {
    return status;
  }
  status = HAL_SPI_Transmit(handle, value, numBytesToWrite, timeout);
  HAL_GPIO_WritePin(chipSelectPort, chipSelectPin, GPIO_PIN_SET);
  return status;
}

#endif /* HAL_SPI_MODULE_ENABLED */
