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
 * @brief Driver file for the WSEN-GCDS-2525020210002 sensor.
 */
#include "WSEN_GCDS_2526101040301.h"
#include <platform.h>
#include <stdio.h>

/**
 * @brief Default sensor interface configuration.
 */
static const WE_sensorInterface_t gcdsDefaultSensorInterface = {
    .sensorType = WE_GCDS, .interfaceType = WE_i2c, .options = {.i2c = {.address = GCDS_ADDRESS, .burstMode = 1, .protocol = WE_i2cProtocol_Raw, .useRegAddrMsbForMultiBytesRead = 1, .reserved = 0}, .spi = {.chipSelectPort = 0, .chipSelectPin = 0, .burstMode = 0, .duplexMode = 0, .reserved = 0, .sensorSpecificSettings = NULL}, .readTimeout = 1000, .writeTimeout = 1000}, .handle = 0};

/**
 * @brief Converts a 16-bit data buffer to an 8-bit array.
 * @param[in] writeBuffer16 Pointer to the 16-bit data buffer.
 * @param[out] writeBuffer8 Pointer to the 8-bit array to store the extracted bytes.
 */
static void convertDataToUInt8Array(const uint16_t* writeBuffer16, uint8_t* writeBuffer8)
{
    /* Extract the bytes from writeBuffer16 and assign them to writeBuffer8 */
    /* LSB goes to writeBuffer8[1] */
    *(writeBuffer8 + 1) = (uint8_t)(*writeBuffer16 & 0xFF);
    /* MSB goes to writeBuffer8[0] */
    *writeBuffer8 = (uint8_t)((*writeBuffer16 >> 8) & 0xFF);
}

/**
 * @brief Read data from sensor.
 * @param[in] sensorInterface Pointer to sensor interface.
 * @param[in] numBytesToRead Number of bytes to be read.
 * @param[out] data Target buffer.
 * @return Error Code.
 */
static inline int8_t GCDS_ReadData(WE_sensorInterface_t* sensorInterface, uint8_t* data, uint16_t numBytesToRead)
{
    /* 0xFF is used here as a place holder and it will not be used in the WE_ReadReg with WE_i2cProtocol_Raw and useRegAddrMsbForMultiBytesRead = 1; */
    return WE_ReadReg(sensorInterface, 0xFF, numBytesToRead, data);
}

/**
 * @brief Write data to sensor.
 * @param[in] sensorInterface Pointer to sensor interface.
 * @param[in] numBytesToRead Number of bytes to be read.
 * @param[out] data Target buffer.
 * @return Error Code.
 */
static inline int8_t GCDS_WriteData(WE_sensorInterface_t* sensorInterface, uint8_t* data, uint16_t numBytesToWrite)
{
    /* 0xFF is used here as a place holder and it will not be used in the WE_WriteReg with WE_i2cProtocol_Raw and useRegAddrMsbForMultiBytesRead = 1; */
    return WE_WriteReg(sensorInterface, 0xFF, numBytesToWrite, data);
}

/**
 * @brief generate CRC for the data bytes.
 * @param[in] data input.
 * @param[in] count of data bytes.
 * @retval Error code.
 */
static uint8_t GCDS_GenerateCRC(const uint8_t* data, uint16_t count)
{
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte)
    {
        uint8_t crc_bit;
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

/**
 * @brief CRC check.
 * @param[in] data input.
 * @param[in] count of data bytes.
 * @retval Error code.
 */
static int8_t GCDS_CheckCRC(const uint8_t* data, uint16_t count, uint8_t checksum)
{
    if (GCDS_GenerateCRC(data, count) != checksum)
    {
        return WE_FAIL;
    }

    return WE_SUCCESS;
}

/**
 * @brief Returns the default sensor interface configuration.
 * @param[out] sensorInterface Sensor interface configuration (output parameter).
 * @return Error code.
 */
int8_t GCDS_Get_Default_Interface(WE_sensorInterface_t* sensorInterface)
{
    *sensorInterface = gcdsDefaultSensorInterface;

    return WE_SUCCESS;
}

/**
 * @brief Performs a measurement and retrieves CO2, temperature, and humidity data.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] co2 CO2 concentration (ppm).
 * @param[out] temperature Temperature in milli °C.
 * @param[out] humidity Humidity in milli %RH.
 * Temperature and Humidity values are multiplied by 1000 to have milli values.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Measure_Data(WE_sensorInterface_t* sensorInterface, uint16_t* co2, int32_t* temperature, uint32_t* humidity)
{

    int8_t status = WE_FAIL;
    uint16_t temperatureRaw = 0;
    uint16_t humidityRaw = 0;
    uint16_t co2ValueInt = 0;
    uint8_t readBuffer8[9] = {0};
    uint16_t writeBuffer16[1] = {GCDS_READ_MEASUREMENT};
    uint8_t writeBuffer8[2] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 9);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }
    status = GCDS_CheckCRC(&readBuffer8[3], GCDS_WORD_SIZE, readBuffer8[5]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }
    status = GCDS_CheckCRC(&readBuffer8[6], GCDS_WORD_SIZE, readBuffer8[8]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    co2ValueInt = ((uint16_t)readBuffer8[0] << 8) | ((uint16_t)readBuffer8[1]);
    temperatureRaw = ((uint16_t)readBuffer8[3] << 8) | ((uint16_t)readBuffer8[4]);
    humidityRaw = ((uint16_t)readBuffer8[6] << 8) | ((uint16_t)readBuffer8[7]);

    /* Please refer to the user manual for signal conversion logic. The result is multiplied with 1000 to create milli values */
    *temperature = (uint16_t)(((temperatureRaw * 175.0f / 65535.0f) - 45.0f) * 1000);
    *humidity = (uint32_t)((humidityRaw * 100.0f / 65535.0f) * 1000);
    *co2 = co2ValueInt;

    return status;
}

/**
 * @brief Performs a measurement and retrieves CO2, temperature, and humidity data.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] co2 CO2 concentration (ppm).
 * @param[out] temperature Temperature in °C.
 * @param[out] humidity Humidity in %RH.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Measure_Data_Float(WE_sensorInterface_t* sensorInterface, uint16_t* co2, float* temperature, float* humidity)
{
    int8_t status = WE_FAIL;
    uint16_t temperatureRaw = 0;
    uint16_t humidityRaw = 0;
    uint16_t co2ValueInt = 0;
    uint8_t readBuffer8[9] = {0};
    uint16_t writeBuffer16[1] = {GCDS_READ_MEASUREMENT};
    uint8_t writeBuffer8[2] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 9);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }
    status = GCDS_CheckCRC(&readBuffer8[3], GCDS_WORD_SIZE, readBuffer8[5]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }
    status = GCDS_CheckCRC(&readBuffer8[6], GCDS_WORD_SIZE, readBuffer8[8]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    co2ValueInt = ((uint16_t)readBuffer8[0] << 8) | ((uint16_t)readBuffer8[1]);
    temperatureRaw = ((uint16_t)readBuffer8[3] << 8) | ((uint16_t)readBuffer8[4]);
    humidityRaw = ((uint16_t)readBuffer8[6] << 8) | ((uint16_t)readBuffer8[7]);

    /* Please refer to the user manual for signal conversion logic. */
    *temperature = (temperatureRaw * 175.0f / 65535.0f) - 45.0f;
    *humidity = (humidityRaw * 100.0f / 65535.0f);
    *co2 = co2ValueInt;

    return status;
}

/**
 * @brief Reads the serial number of the sensor.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] serialNo Pointer to the buffer to store the serial number.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Read_SlNo(WE_sensorInterface_t* sensorInterface, uint8_t* serialNo)
{

    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_GET_SERIAL_NUMBER};
    uint8_t writeBuffer8[2] = {0};
    uint8_t readBuffer8[9] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_Stop_Periodic_Measurement(sensorInterface);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 9);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }
    status = GCDS_CheckCRC(&readBuffer8[3], GCDS_WORD_SIZE, readBuffer8[5]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }
    status = GCDS_CheckCRC(&readBuffer8[6], GCDS_WORD_SIZE, readBuffer8[8]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    *serialNo = readBuffer8[0];
    *(serialNo + 1) = readBuffer8[1];
    *(serialNo + 2) = readBuffer8[3];
    *(serialNo + 3) = readBuffer8[4];
    *(serialNo + 4) = readBuffer8[6];
    *(serialNo + 5) = readBuffer8[7];

    return status;
}

/**
 * @brief Initializes the sensor by reading its serial number.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @retval WE_SUCCESS if the serial number is valid, WE_FAIL if the serial number is invalid.
 */
int8_t GCDS_Init(WE_sensorInterface_t* sensorInterface)
{
    uint8_t serialNo[6] = {0};
    int8_t status = WE_FAIL;

    status = GCDS_Read_SlNo(sensorInterface, serialNo);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    return status;
}

/**
 * @brief Powers down the sensor.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Power_Down(WE_sensorInterface_t* sensorInterface)
{

    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_POWER_DOWN};
    uint8_t writeBuffer8[2] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    return status;
}

/**
 * @brief Wakes up the sensor.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Wake_Up(WE_sensorInterface_t* sensorInterface)
{

    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_WAKE_UP};
    uint8_t writeBuffer8[2] = {0};
    uint8_t serialNo[SERIAL_NO_LENGTH] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    /* GCDS does not acknowledge the wake_up command. */
    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);

    /* Mandatory 30 ms delay for the sensor to process the command. */
    WE_Delay(30);

    status = GCDS_Read_SlNo(sensorInterface, serialNo);
    if (status < 0)
    {
        return WE_FAIL;
    }

    return status;
}

/**
 * @brief Performs manual re-calibration of the sensor.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[in] co2Target Target CO2 value for re-calibration.
 * @param[out] frcCorrection Calibration correction value.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Perform_Manual_Recalibration(WE_sensorInterface_t* sensorInterface, const uint16_t* co2Target, uint16_t* frcCorrection)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_PERFORM_MANUAL_RECALIBRATION};
    uint8_t writeBuffer8[5] = {0};
    uint8_t readBuffer8[3] = {0};
    uint16_t frcCorrectionRaw = 0;

    convertDataToUInt8Array(writeBuffer16, &writeBuffer8[0]);
    convertDataToUInt8Array(co2Target, &writeBuffer8[2]);
    writeBuffer8[4] = GCDS_GenerateCRC(&writeBuffer8[2], GCDS_WORD_SIZE);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 5);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 400 ms delay for the sensor to process the command. */
    WE_Delay(400);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 3);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {

        return WE_FAIL;
    }

    /* Failed FRC returns 0xFFFF. */
    frcCorrectionRaw = ((uint16_t)readBuffer8[0] << 8) | ((uint16_t)readBuffer8[1]);
    if (frcCorrectionRaw == 0xFFFF)
    {
        return WE_FAIL;
    }

    *frcCorrection = frcCorrectionRaw - FRC_CORRECTION_CONSTANT;

    return status;
}

/**
 * @brief Retrieves the self-calibration CO2 target.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] co2Target CO2 target value for self-calibration.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Get_Self_Calib_Target(WE_sensorInterface_t* sensorInterface, uint16_t* co2Target)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_GET_SELF_CALIB_TARGET};
    uint8_t writeBuffer8[2] = {0};
    uint8_t readBuffer8[3] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 3);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    *co2Target = ((uint16_t)readBuffer8[0] << 8) | ((uint16_t)readBuffer8[1]);

    return status;
}

/**
 * @brief Sets the self-calibration CO2 target.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[in] co2Target CO2 target value for self-calibration.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Set_Self_Calib_Target(WE_sensorInterface_t* sensorInterface, const uint16_t* co2Target)
{

    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_SET_SELF_CALIB_TARGET};
    uint8_t writeBuffer8[5] = {0};

    convertDataToUInt8Array(writeBuffer16, &writeBuffer8[0]);
    convertDataToUInt8Array(co2Target, &writeBuffer8[2]);
    writeBuffer8[4] = GCDS_GenerateCRC(&writeBuffer8[2], GCDS_WORD_SIZE);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 5);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    return status;
}

/**
 * @brief Retrieves the self-calibration (ASC) enabled status.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] ASCEnabled ASC enabled status (0 or 1).
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Get_Self_Calib_Enabled(WE_sensorInterface_t* sensorInterface, bool* ASCEnabled)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_GET_SELF_CALIB_ENABLED};
    uint8_t writeBuffer8[2] = {0};
    uint8_t readBuffer8[3] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 3);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    *ASCEnabled = ((uint16_t)readBuffer8[0] << 8) | ((uint16_t)readBuffer8[1]);

    return status;
}

/**
 * @brief Sets the self-calibration (ASC) enabled status.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[in] ASCEnabled ASC enabled status (0 or 1).
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Set_Self_Calib_Enabled(WE_sensorInterface_t* sensorInterface, bool* ASCEnabled)
{

    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_SET_SELF_CALIB_ENABLED};
    uint8_t writeBuffer8[5] = {0};
    uint16_t ASCEnabled16 = (uint16_t)(*ASCEnabled);

    convertDataToUInt8Array(writeBuffer16, &writeBuffer8[0]);
    convertDataToUInt8Array(&ASCEnabled16, &writeBuffer8[2]);
    writeBuffer8[4] = GCDS_GenerateCRC(&writeBuffer8[2], GCDS_WORD_SIZE);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 5);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    return status;
}

/**
 * @brief Gets the self-calibration (ASC) initial period.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] ascInitialPeriod Retrieved ASC initial period value.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Get_Self_Calib_Init_Period(WE_sensorInterface_t* sensorInterface, uint16_t* ascInitialPeriod)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_GET_SELF_CALIB_INITIAL_PERIOD};
    uint8_t writeBuffer8[2] = {0};
    uint8_t readBuffer8[3] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 3);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    *ascInitialPeriod = ((uint16_t)readBuffer8[0] << 8) | ((uint16_t)readBuffer8[1]);

    return status;
}

/**
 * @brief Sets the self-calibration (ASC) initial period.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[in] ascInitialPeriod Initial ASC period value to set.
 * ASC period value should be intiger multiples of 4 hours.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Set_Self_Calib_Init_Period(WE_sensorInterface_t* sensorInterface, const uint16_t* ascInitialPeriod)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_SET_SELF_CALIB_INITIAL_PERIOD};
    uint8_t writeBuffer8[5] = {0};

    convertDataToUInt8Array(writeBuffer16, &writeBuffer8[0]);
    convertDataToUInt8Array(ascInitialPeriod, &writeBuffer8[2]);
    writeBuffer8[4] = GCDS_GenerateCRC(&writeBuffer8[2], GCDS_WORD_SIZE);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 5);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    return status;
}

/**
 * @brief Gets the self-calibration standard period.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] ascStdPeriod Pointer to store the retrieved ASC standard period.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Get_Self_Calib_Std_Period(WE_sensorInterface_t* sensorInterface, uint16_t* ascStdPeriod)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_GET_SELF_CALIB_STANDARD_PERIOD};
    uint8_t writeBuffer8[2] = {0};
    uint8_t readBuffer8[3] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 3);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    *ascStdPeriod = ((uint16_t)readBuffer8[0] << 8) | ((uint16_t)readBuffer8[1]);

    return status;
}

/**
 * @brief Sets the self-calibration standard period.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[in] ascStdPeriod ASC standard period value to be set.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Set_Self_Calib_Std_Period(WE_sensorInterface_t* sensorInterface, const uint16_t* ascStdPeriod)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_SET_SELF_CALIB_STANDARD_PERIOD};
    uint8_t writeBuffer8[5] = {0};

    convertDataToUInt8Array(writeBuffer16, &writeBuffer8[0]);
    convertDataToUInt8Array(ascStdPeriod, &writeBuffer8[2]);
    writeBuffer8[4] = GCDS_GenerateCRC(&writeBuffer8[2], GCDS_WORD_SIZE);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 5);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    return status;
}

/**
 * @brief Retrieves the temperature offset from the sensor.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] temperatureOffset Pointer to store the temperature offset (in milli-degrees Celsius).
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Get_Temperature_Offset(WE_sensorInterface_t* sensorInterface, uint32_t* temperatureOffset)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_GET_TEMPERATURE_OFFSET};
    uint8_t writeBuffer8[2] = {0};
    uint8_t readBuffer8[3] = {0};
    uint16_t temperatureRaw = 0;

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 3);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    temperatureRaw = ((uint16_t)readBuffer8[0] << 8) | ((uint16_t)readBuffer8[1]);
    *temperatureOffset = (uint32_t)(((temperatureRaw * 175.0) / 65535.0) * 1000);

    return status;
}

/**
 * @brief Sets the temperature offset for the sensor.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[in] temperatureOffset Pointer to the temperature offset (in milli-degrees Celsius).
 * Recommended temperature offset values are between 0 and 20000 milli-degrees Celsius.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Set_Temperature_Offset(WE_sensorInterface_t* sensorInterface, const uint32_t* temperatureOffset)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_SET_TEMPERATURE_OFFSET};
    uint8_t writeBuffer8[5] = {0};
    uint16_t setTemperatureOffset_raw = 0;

    setTemperatureOffset_raw = (uint16_t)(((*temperatureOffset) / 175000.0) * 65536.0);
    convertDataToUInt8Array(writeBuffer16, &writeBuffer8[0]);
    convertDataToUInt8Array(&setTemperatureOffset_raw, &writeBuffer8[2]);
    writeBuffer8[4] = GCDS_GenerateCRC(&writeBuffer8[2], GCDS_WORD_SIZE);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 5);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    return status;
}

/**
 * @brief Retrieves the previously set sensor altitude.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] sensorAltitude Pointer to store the retrieved altitude value (in meters).
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Get_Sensor_Altitude(WE_sensorInterface_t* sensorInterface, uint16_t* sensorAltitude)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_GET_SENSOR_ALTITUDE};
    uint8_t writeBuffer8[2] = {0};
    uint8_t readBuffer8[3] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 3);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    *sensorAltitude = ((uint16_t)readBuffer8[0] << 8) | ((uint16_t)readBuffer8[1]);

    return status;
}

/**
 * @brief Sets the sensor altitude.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[in] sensorAltitude Pointer to the altitude value (in meters).
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Set_Sensor_Altitude(WE_sensorInterface_t* sensorInterface, const uint16_t* sensorAltitude)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_SET_SENSOR_ALTITUDE};
    uint8_t writeBuffer8[5] = {0};

    convertDataToUInt8Array(writeBuffer16, &writeBuffer8[0]);
    convertDataToUInt8Array(sensorAltitude, &writeBuffer8[2]);
    writeBuffer8[4] = GCDS_GenerateCRC(&writeBuffer8[2], GCDS_WORD_SIZE);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 5);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    return status;
}

/**
 * @brief Reads the previously set ambient pressure value.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] ambientPressure Pointer to store the retrieved ambient pressure in Pa.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Get_Ambient_Pressure(WE_sensorInterface_t* sensorInterface, uint32_t* ambientPressure)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_GET_SET_AMBIENT_PRESSURE};
    uint8_t writeBuffer8[2] = {0};
    uint8_t readBuffer8[3] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 3);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    uint16_t rawPressure = (uint16_t)((readBuffer8[0] << 8) | readBuffer8[1]);
    /* Convert to ambient pressure from raw value by multiplying by 100 */
    *ambientPressure = rawPressure * 100;

    return status;
}

/**
 * @brief Sets the ambient pressure for continuous pressure compensation.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[in] ambientPressure Pointer to the ambient pressure value in Pa (70,000–120,000).
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Set_Ambient_Pressure(WE_sensorInterface_t* sensorInterface, const uint32_t* ambientPressure)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_GET_SET_AMBIENT_PRESSURE};
    uint8_t writeBuffer8[5] = {0};

    /* Convert ambient pressure to raw value by dividing by 100 */
    uint16_t rawPressure = (uint16_t)(*ambientPressure / 100);

    convertDataToUInt8Array(writeBuffer16, &writeBuffer8[0]);
    convertDataToUInt8Array(&rawPressure, &writeBuffer8[2]);
    writeBuffer8[4] = GCDS_GenerateCRC(&writeBuffer8[2], GCDS_WORD_SIZE);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 5);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    return status;
}

/**
 * @brief Persists current sensor settings to EEPROM for retention after power cycling.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Persist_Settings(WE_sensorInterface_t* sensorInterface)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_PERSIST_SETTINGS};
    uint8_t writeBuffer8[2] = {0};

    convertDataToUInt8Array(writeBuffer16, &writeBuffer8[0]);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 800 ms delay for the sensor to process the command. */
    WE_Delay(800);

    return status;
}

/**
 * @brief Reinitializes the sensor by reloading user settings from EEPROM.
 * The sensor must be in the idle state before this function is called.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Reinit(WE_sensorInterface_t* sensorInterface)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_REINIT};
    uint8_t writeBuffer8[2] = {0};

    convertDataToUInt8Array(writeBuffer16, &writeBuffer8[0]);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 30 ms delay for the sensor to process the command. */
    WE_Delay(30);

    return status;
}

/**
 * @brief Performs a self-test to check the sensor functionality.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] sensorStatus Status of the self-test: 0 if no malfunction, non-zero if malfunction detected.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Perform_Self_Test(WE_sensorInterface_t* sensorInterface, uint16_t* sensorStatus)
{
    int8_t status = WE_FAIL;
    uint8_t writeBuffer8[2] = {0};
    uint16_t writeBuffer16[1] = {GCDS_PERFORM_SELF_TEST};
    uint8_t readBuffer8[3] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 10000 ms delay for the sensor to process the command. */
    WE_Delay(10000);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 3);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], 2, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    *sensorStatus = (uint16_t)((uint16_t)readBuffer8[0] << 8) | (uint16_t)readBuffer8[1];

    return status;
}

/**
 * @brief Performs a factory reset on the sensor, resetting all configuration settings and erasing FRC and ASC history.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Perform_Factory_Reset(WE_sensorInterface_t* sensorInterface)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_PERFORM_FACTORY_RESET};
    uint8_t writeBuffer8[2] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1200 ms delay for the sensor to process the command. */
    WE_Delay(1200);

    return status;
}

/**
 * @brief Starts periodic measurement on the sensor.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Start_Periodic_Measurement(WE_sensorInterface_t* sensorInterface)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_START_PERIODIC_MEASUREMENT};
    uint8_t writeBuffer8[2] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    return status;
}

/**
 * @brief Stops periodic measurement on the sensor.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Stop_Periodic_Measurement(WE_sensorInterface_t* sensorInterface)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_STOP_PERIODIC_MEASUREMENT};
    uint8_t writeBuffer8[2] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 500 ms delay for the sensor to process the command. */
    WE_Delay(500);

    return status;
}

/**
 * @brief Starts the low power periodic measurement mode.
 * This mode updates the sensor every 30 seconds while consuming less power.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Start_Low_Power_Measurement(WE_sensorInterface_t* sensorInterface)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_START_LOW_POWER_MEASUREMENT};
    uint8_t writeBuffer8[2] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    return status;
}

/**
 * @brief Checks if the data is ready for read-out.
 * This function polls the sensor to check if data from the measurement is ready.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @param[out] dataReady Indicates if the data is ready (true/false).
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Get_Data_Ready_Status(WE_sensorInterface_t* sensorInterface, bool* dataReady)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_GET_DATA_READY_STATUS};
    uint8_t writeBuffer8[2] = {0};
    uint8_t readBuffer8[3] = {0};
    uint16_t dataReadyReadVal = 0;

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 1 ms delay for the sensor to process the command. */
    WE_Delay(1);

    status = GCDS_ReadData(sensorInterface, readBuffer8, 3);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    status = GCDS_CheckCRC(&readBuffer8[0], GCDS_WORD_SIZE, readBuffer8[2]);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    dataReadyReadVal = ((uint16_t)readBuffer8[0] << 8) | ((uint16_t)readBuffer8[1]);
    *dataReady = (dataReadyReadVal & 0x07FF) != 0;

    return WE_SUCCESS;
}

/**
 * @brief Performs a single-shot measurement of CO2, temperature, and humidity.
 * This sends the command to initiate the measurement. The results should be retrieved
 * using the read_measurement command after the specified delay.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Measure_Single_Shot(WE_sensorInterface_t* sensorInterface)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_MEASURE_SINGLE_SHOT};
    uint8_t writeBuffer8[2] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 5000 ms delay for the sensor to process the command. */
    WE_Delay(5000);

    return WE_SUCCESS;
}

/**
 * @brief Performs a single-shot measurement of relative humidity and temperature only.
 * This command reduces power consumption. CO2 output is returned as 0 ppm.
 * The results should be retrieved using the read_measurement command after the specified delay.
 * @param[in] sensorInterface Pointer to the sensor interface.
 * @return WE_SUCCESS if successful, WE_FAIL if failed.
 */
int8_t GCDS_Measure_Single_Shot_RHT(WE_sensorInterface_t* sensorInterface)
{
    int8_t status = WE_FAIL;
    uint16_t writeBuffer16[1] = {GCDS_MEASURE_SINGLE_SHOT_RHT};
    uint8_t writeBuffer8[2] = {0};

    convertDataToUInt8Array(writeBuffer16, writeBuffer8);

    status = GCDS_WriteData(sensorInterface, writeBuffer8, 2);
    if (status != WE_SUCCESS)
    {
        return WE_FAIL;
    }

    /* Mandatory 50 ms delay for the sensor to process the command. */
    WE_Delay(50);

    return WE_SUCCESS;
}
