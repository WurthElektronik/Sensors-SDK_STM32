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
 * COPYRIGHT (c) 2025 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 **/

/**
 * @file
 * @brief Driver file for the WSEN-PDMS-25131308XXX05 sensor.
 */

#include "WSEN_PDMS_25131308XXX05.h"
#include <platform.h>
#include <stdio.h>
#include <string.h>

/* SPI CRC select default config variable */
static PDMS_Spi_CrcSelect_t defaultCrcSelect = PDMS_SPI_withoutCRC;

/**
 * @brief Default sensor interface configuration.
 */
static WE_sensorInterface_t PDMSDefaultSensorInterface = {.sensorType = WE_PDMS,
                                                          .interfaceType = WE_i2c,
                                                          .options = {.i2c = {.address = PDMS_I2C_ADDRESS_CRC, .burstMode = 1, .protocol = WE_i2cProtocol_Raw, .useRegAddrMsbForMultiBytesRead = 1, .reserved = 0}, .spi = {.chipSelectPort = 0, .chipSelectPin = 0, .burstMode = 1, .duplexMode = 1, .reserved = 0, .sensorSpecificSettings = &defaultCrcSelect}, .readTimeout = 1000, .writeTimeout = 1000},
                                                          .handle = 0};

/**
 * @brief Read data from sensor.
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[in] numBytesToRead Number of bytes to be read
 * @param[out] data Read buffer
 * @return Error Code
 */
static inline int8_t PDMS_ReadReg(WE_sensorInterface_t* sensorInterface, uint16_t numBytesToRead, uint8_t* data)
{
    /* 0xFF is used here as a place holder and it will not be used in the WE_ReadReg with WE_i2cProtocol_Raw and useRegAddrMsbForMultiBytesRead = 1; */
    return WE_ReadReg(sensorInterface, 0xFF, numBytesToRead, data);
}

/**
 * @brief Write data to sensor.
 * @param[in] sensorInterface Pointer to sensor interface.
 * @param[in] numBytesToRead Number of bytes to be read.
 * @param[in] data Write buffer.
 * @return Error Code.
 */
static inline int8_t PDMS_WriteReg(WE_sensorInterface_t* sensorInterface, uint8_t regAdr, uint16_t numBytesToWrite, uint8_t* data)
{
    /* 0xFF is used here as a place holder and it will not be used in the WE_WriteReg with WE_i2cProtocol_Raw and useRegAddrMsbForMultiBytesRead = 1; */
    return WE_WriteReg(sensorInterface, 0xFF, numBytesToWrite, data);
}

/**
 * @brief Duplex communication with SPI.
 * @param[in] sensorInterface Pointer to sensor interface.
 * @param[in] numBytes Number of bytes to be transmitted/received.
 * @param[in] txData Transmit buffer.
 * @param[out] rxData Receive buffer.
 * @return Error Code.
 */
static inline int8_t PDMS_SPITransceive(WE_sensorInterface_t* sensorInterface, uint16_t numBytes, uint8_t* txData, uint8_t* rxData) { return WE_SPITransceive(sensorInterface, numBytes, txData, rxData); }

/**
 * @brief Returns the default sensor interface configuration.
 * @param[out] sensorInterface Sensor interface configuration (output parameter)
 * @return Error code
 */
int8_t PDMS_getDefaultInterface(WE_sensorInterface_t* sensorInterface)
{
    *sensorInterface = PDMSDefaultSensorInterface;
    return WE_SUCCESS;
}

/**
  Calculates CRC4 given a polynom, initialization value and data
  @param polynom The Polynom to use for CRC-calculation
  @param init The initial value
  @param data The data to calculate the CRC4 for
  @param length The length of the data in bytes
  @return Calculated CRC4
*/
static uint8_t calc_crc4(uint8_t polynom, uint8_t init, uint8_t* data, uint16_t length)
{
    uint8_t crc;
    uint16_t byteIndex;
    int8_t bitIndex;

    /* Initialize CRC with the initial value */
    crc = init;
    for (byteIndex = 0; byteIndex < length; byteIndex++)
    {
        for (bitIndex = 7; bitIndex >= 0; bitIndex--)
        {
            /* Process only the 4 least significant bits of the last byte */
            if ((byteIndex >= length - 1) && (bitIndex < 4))
            {
                break;
            }

            /* Compare the most significant bit of crc with the current bit of data */
            if (((crc >> 3) & 0x01) != ((data[byteIndex] >> bitIndex) & 0x01))
            {
                /* Shift crc left by 1 bit and XOR with polynomial */
                crc = (crc << 1) ^ polynom;
            }
            else
            {
                /* Shift crc to left */
                crc = crc << 1;
            }
            /* Ensure crc remains a 4-bit value */
            crc = crc & 0x0F;
        }
    }

    /* Return the final 4-bit CRC value */
    return crc & 0x0F;
}

/**
  Calculates CRC8 given a polynom, initialization value and data
  @param polynom The Polynom to use for CRC-calculation
  @param init The initial value
  @param data The data to calculate the CRC8 for
  @param length The length of the data in bytes
  @return Calculated CRC8
*/
static uint8_t calc_crc8(uint8_t polynom, uint8_t init, uint8_t* data, uint16_t length)
{
    uint8_t crc;
    uint16_t byteIndex;
    int8_t bitIndex;

    /* Initialize CRC with the initial value */
    crc = init;
    for (byteIndex = 0; byteIndex < length; byteIndex++)
    {
        for (bitIndex = 7; bitIndex >= 0; bitIndex--)
        {
            /* Compare the most significant bit of crc (bit 7) with the current bit of data[byteIndex] */
            if (((crc >> 7) & 0x01) != ((data[byteIndex] >> bitIndex) & 0x01))
            {
                /* Shift crc left by 1 bit and XOR with polynomial */
                crc = (crc << 1) ^ polynom;
            }
            else
            {
                /* Shift crc left */
                crc = crc << 1;
            }
        }
    }

    /* Return the final 8-bit CRC value  */
    return crc & 0xFF;
}

/**
 * @brief Read the pressure and temperature values
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[in] type PDMS sensor type (i.e., pressure measurement range) for internal conversion of pressure
 * @param[out] pressureKPa Pointer to pressure value in kPa
 * @param[out] temperatureDegC Pointer to temperature value in degrees Celsius
 * @param[out] syncStatusValue Pointer to store the synchronized status value
 * @retval Error code
 */
int8_t PDMS_getPressureAndTemperature_float(WE_sensorInterface_t* sensorInterface, PDMS_SensorType_t type, float* pressureKPa, float* temperatureDegC, uint16_t* syncStatusValue)
{
    uint16_t rawPressure = 0;
    uint16_t rawTemperature = 0;
    uint16_t statusValue = 0;

    /* Check if sensor interface is initialized */
    if (sensorInterface == NULL)
    {
        return WE_FAIL;
    }

    switch (sensorInterface->interfaceType)
    {
        case WE_i2c:
        {
            switch (sensorInterface->options.i2c.address)
            {
                case PDMS_I2C_ADDRESS_CRC:
                    if (WE_FAIL == PDMS_I2C_GetRawPressureAndTemperature_WithCRC(sensorInterface, &rawPressure, &rawTemperature, &statusValue))
                    {
                        return WE_FAIL;
                    }
                    break;

                case PDMS_I2C_ADDRESS:
                    if (WE_FAIL == PDMS_I2C_GetRawPressureAndTemperature(sensorInterface, &rawPressure, &rawTemperature, &statusValue))
                    {
                        return WE_FAIL;
                    }
                    break;

                default:
                    /* Invalid I2C address for PDMS type pressure sensors */
                    return WE_FAIL;
            }
        }
        break;

        case WE_spi:
        {

            /* Check if sensor-specific SPI settings are configured (not NULL) */
            if (sensorInterface->options.spi.sensorSpecificSettings == NULL)
            {
                return WE_FAIL;
            }
            switch (*(PDMS_Spi_CrcSelect_t*)sensorInterface->options.spi.sensorSpecificSettings)
            {
                case PDMS_SPI_withoutCRC:
                    if (WE_FAIL == PDMS_SPI_GetRawPressureAndTemperature(sensorInterface, &rawPressure, &rawTemperature, &statusValue))
                    {
                        return WE_FAIL;
                    }
                    break;

                case PDMS_SPI_withCRC:
                    if (WE_FAIL == PDMS_SPI_getRawPressureAndTemperature_WithCRC(sensorInterface, &rawPressure, &rawTemperature, &statusValue))
                    {
                        return WE_FAIL;
                    }
                    break;

                default:
                    /* Invalid SPI CRC select */
                    return WE_FAIL;
            }
        }
    }
    *syncStatusValue = statusValue;

    /* Apply temperature offset to raw temperature and convert to °C. Please refer to the manual for more details */
    *temperatureDegC = (((float)(rawTemperature - T_MIN_TYP_VAL_PDMS) * 4.272f) / 1000);

    /* Apply pressure offset to raw pressure and convert to KPa. Please refer to the manual for more details */
    /* Conversion logic is based on the sensor type */
    float adjustedRawPressure = (float)((float)rawPressure - (float)P_MIN_TYP_VAL_PDMS);
    switch (type)
    {
        case PDMS_pdms0:
            *pressureKPa = ((adjustedRawPressure * 7.63f) / 100000) - 1.0f;
            break;

        case PDMS_pdms1:
            *pressureKPa = ((adjustedRawPressure * 7.63f) / 10000) - 10.0f;
            break;

        case PDMS_pdms2:
            *pressureKPa = ((adjustedRawPressure * 2.67f) / 1000) - 35.0f;
            break;

        case PDMS_pdms3:
            *pressureKPa = ((adjustedRawPressure * 3.81f) / 1000);
            break;

        case PDMS_pdms4:
            *pressureKPa = ((adjustedRawPressure * 4.19f) / 100) - 100.0f;
            break;

        default:
            /* Invalid PDMS sensor type */
            return WE_FAIL;
    }

    return WE_SUCCESS;
}

/**
 * @brief Read the raw pressure and temperature values via I2C interface
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[out] rawPressure Pointer to store the raw pressure value
 * @param[out] rawTemperature Pointer to store the raw temperature value
 * @param[out] syncStatusValue Pointer to store the synchronized status value
 * @retval Error code
 */
int8_t PDMS_I2C_GetRawPressureAndTemperature(WE_sensorInterface_t* sensorInterface, uint16_t* rawPressure, uint16_t* rawTemperature, uint16_t* syncStatusValue)
{
    uint8_t writeBuffer8 = PDMS_I2C_READ_MEASUREMENT;
    int8_t status = WE_FAIL;
    uint8_t readbuffer8[6];

    /* Invalid I2C check address for measurement without CRC */
    if (sensorInterface->options.i2c.address != PDMS_I2C_ADDRESS)
    {
        return WE_FAIL;
    }

    status = PDMS_WriteReg(sensorInterface, 0xFF, 1, &writeBuffer8);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = PDMS_ReadReg(sensorInterface, 6, &readbuffer8[0]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    *rawTemperature = ((uint16_t)readbuffer8[1] << 8) | (uint16_t)readbuffer8[0];
    *rawPressure = ((uint16_t)readbuffer8[3] << 8) | (uint16_t)readbuffer8[2];
    *syncStatusValue = ((uint16_t)readbuffer8[5] << 8) | (uint16_t)readbuffer8[4];

    return WE_SUCCESS;
}

/**
 * @brief Read the raw pressure and temperature values with CRC check via I2C interface
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[out] rawPressure Pointer to store the raw pressure value
 * @param[out] rawTemperature Pointer to store the raw temperature value
 * @param[out] syncStatusValue Pointer to store the synchronized status value
 * @retval Error code
 */
int8_t PDMS_I2C_GetRawPressureAndTemperature_WithCRC(WE_sensorInterface_t* sensorInterface, uint16_t* rawPressure, uint16_t* rawTemperature, uint16_t* syncStatusValue)
{
    uint8_t writeBuffer8[2] = {0};
    int8_t status = WE_FAIL;
    uint8_t readbuffer8[7];
    uint8_t crc4;
    uint8_t crc8;
    uint8_t combinedData[10];

    /* Invalid I2C address check for measurement with CRC */
    if (sensorInterface->options.i2c.address != PDMS_I2C_ADDRESS_CRC)
    {
        return WE_FAIL;
    }

    /* Set the first BYTE to the I2C read measurement command */
    writeBuffer8[0] = PDMS_I2C_READ_MEASUREMENT;
    /* Set the upper 4 bits of the second byte to (number of data bytes to read (6) - 1) which is 5 */
    writeBuffer8[1] = (6 - 1) << 4;

    /* Compute and place CRC4 value into the least significant 4 bits of the second byte */
    crc4 = calc_crc4(CRC4_I2C_POLYNOMIAL, CRC4_I2C_INIT, &writeBuffer8[0], 2);
    writeBuffer8[1] |= (crc4 & 0x0F);

    status = PDMS_WriteReg(sensorInterface, 0xff, 2, &writeBuffer8[0]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = PDMS_ReadReg(sensorInterface, 7, &readbuffer8[0]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Combine all data into a single buffer for CRC calculation. Please refer to the manual for more information on CRC calculation */
    combinedData[0] = PDMS_I2C_ADDRESS_CRC << 1;
    combinedData[1] = writeBuffer8[0];
    combinedData[2] = writeBuffer8[1];
    combinedData[3] = (PDMS_I2C_ADDRESS_CRC << 1) | 1;
    memcpy(&combinedData[4], &readbuffer8[0], 6);

    crc8 = calc_crc8(CRC8_I2C_POLYNOMIAL, CRC8_I2C_INIT, &combinedData[0], 10);
    if (crc8 != readbuffer8[6])
    {
        return WE_FAIL;
    }

    *rawTemperature = ((uint16_t)readbuffer8[1] << 8) | (uint16_t)readbuffer8[0];
    *rawPressure = ((uint16_t)readbuffer8[3] << 8) | (uint16_t)readbuffer8[2];
    *syncStatusValue = ((uint16_t)readbuffer8[5] << 8) | (uint16_t)readbuffer8[4];

    return WE_SUCCESS;
}

/**
 * @brief Read the raw pressure and temperature values via SPI interface
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[out] rawPressure Pointer to store the raw pressure value
 * @param[out] rawTemperature Pointer to store the raw temperature value
 * @param[out] syncStatusValue Pointer to store the synchronized status value
 * @retval Error code
 */
int8_t PDMS_SPI_GetRawPressureAndTemperature(WE_sensorInterface_t* sensorInterface, uint16_t* rawPressure, uint16_t* rawTemperature, uint16_t* syncStatusValue)
{
    int8_t status = WE_FAIL;
    uint8_t readbuffer8[8] = {0};
    uint8_t writebuffer8[8] = {0};

    /* Invalid SPI protocol for non CRC measurement */
    if (*(PDMS_Spi_CrcSelect_t*)sensorInterface->options.spi.sensorSpecificSettings != PDMS_SPI_withoutCRC)
    {
        return WE_FAIL;
    }
    /* Set the first BYTE to the SPI read measurement command */
    writebuffer8[0] = PDMS_SPI_READ_MEASUREMENT;
    /* Clear the first bit (bit 7) of the second byte to disable CRC*/
    writebuffer8[1] &= ~(1 << 7);
    /* Set next 3 bits (bits 4–6) of the second byte to (number of data words to read (3) - 1), which is 2 */
    writebuffer8[1] |= ((3 - 1) << 4);

    status = PDMS_SPITransceive(sensorInterface, 8, &writebuffer8[0], &readbuffer8[0]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* readbuffer8[0] and readbuffer8[1] are don't care */
    *rawTemperature = ((uint16_t)readbuffer8[2] << 8) | (uint16_t)readbuffer8[3];
    *rawPressure = ((uint16_t)readbuffer8[4] << 8) | (uint16_t)readbuffer8[5];
    *syncStatusValue = ((uint16_t)readbuffer8[6] << 8) | (uint16_t)readbuffer8[7];

    return WE_SUCCESS;
}

/**
 * @brief Read the raw pressure and temperature values with CRC via SPI interface
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[out] rawPressure Pointer to store the raw pressure value
 * @param[out] rawTemperature Pointer to store the raw temperature value
 * @param[out] syncStatusValue Pointer to store the synchronized status value
 * @retval Error code
 */
int8_t PDMS_SPI_getRawPressureAndTemperature_WithCRC(WE_sensorInterface_t* sensorInterface, uint16_t* rawPressure, uint16_t* rawTemperature, uint16_t* syncStatusValue)
{

    int8_t status = WE_FAIL;
    uint8_t readbuffer8[9] = {0};
    uint8_t writebuffer8[9] = {0};
    uint8_t crc4;
    uint8_t crc8;

    if (*(PDMS_Spi_CrcSelect_t*)sensorInterface->options.spi.sensorSpecificSettings != PDMS_SPI_withCRC)
    {
        return WE_FAIL;
    }

    /* Set the first BYTE to the I2C read measurement command */
    writebuffer8[0] = PDMS_SPI_READ_MEASUREMENT;
    /* Set the first bit of the second byte to enable CRC */
    writebuffer8[1] |= (1 << 7);
    /* Set the next 3 bits (bits 4–6) of the second byte to (number of data words to read (3) - 1) which is 2 */
    writebuffer8[1] |= ((3 - 1) << 4);

    /* Compute and place CRC4 value into the least significant 4 bits of the second byte */
    crc4 = calc_crc4(CRC4_SPI_POLYNOMIAL, CRC4_SPI_INIT, &writebuffer8[0], 2);
    writebuffer8[1] |= (crc4 & 0x0F);

    status = PDMS_SPITransceive(sensorInterface, 9, &writebuffer8[0], &readbuffer8[0]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* readbuffer8[0] and readbuffer8[1] are don't care */
    *rawTemperature = ((uint16_t)readbuffer8[2] << 8) | (uint16_t)readbuffer8[3];
    *rawPressure = ((uint16_t)readbuffer8[4] << 8) | (uint16_t)readbuffer8[5];
    *syncStatusValue = ((uint16_t)readbuffer8[6] << 8) | (uint16_t)readbuffer8[7];

    /* Calculate the 8-bit CRC (CRC8) over the first 8 bytes of readbuffer8. Please refer to the manual for more information on CRC calculation */
    crc8 = calc_crc8(CRC8_SPI_POLYNOMIAL, CRC8_SPI_INIT, &readbuffer8[0], 8);
    if (crc8 != readbuffer8[8])
    {
        return WE_FAIL;
    }

    return WE_SUCCESS;
}
