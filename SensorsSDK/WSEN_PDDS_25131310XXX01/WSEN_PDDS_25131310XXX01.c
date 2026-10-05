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
 * @brief Driver file for the WSEN-PDDS-25131310XXX01 sensor.
 */

#include "WSEN_PDDS_25131310XXX01.h"
#include <platform.h>
#include <stdio.h>

/**
 * @brief Default sensor interface configuration.
 */
static WE_sensorInterface_t PDDSDefaultSensorInterface = {
    .sensorType = WE_PDDS, .interfaceType = WE_i2c, .options = {.i2c = {.address = PDDS_I2C_DEFAULT, .burstMode = 0, .protocol = WE_i2cProtocol_RegisterBased, .useRegAddrMsbForMultiBytesRead = 0, .reserved = 0}, .spi = {.chipSelectPort = 0, .chipSelectPin = 0, .burstMode = 0, .duplexMode = 0, .reserved = 0, .sensorSpecificSettings = NULL}, .readTimeout = 1000, .writeTimeout = 1000}, .handle = 0};
/**
 * @brief Describes whether it's an read or write SPI operation
 */
typedef enum
{
    PDDS_SPI_CMD_WRITE = 0x00,
    PDDS_SPI_CMD_READ = 0x01,
} pdds_spi_cmd_read_write;

/**
 * @brief Read from a sensor register.
 * @param[in]  sensorInterface Pointer to sensor interface
 * @param[in]  regAdr          Register address
 * @param[in]  numBytesToRead  Number of bytes to read
 * @param[out] data            Read buffer
 * @return WE_SUCCESS or WE_FAIL
 */
static inline int8_t PDDS_ReadReg(WE_sensorInterface_t* sensorInterface, uint8_t regAdr, uint16_t numBytesToRead, uint8_t* data) { return WE_ReadReg(sensorInterface, regAdr, numBytesToRead, data); }

/**
 * @brief Write to a sensor register.
 * @param[in]  sensorInterface  Pointer to sensor interface
 * @param[in]  regAdr           Register address
 * @param[in]  numBytesToWrite  Number of bytes to write
 * @param[in]  data             Write buffer
 * @return WE_SUCCESS or WE_FAIL
 */
static inline int8_t PDDS_WriteReg(WE_sensorInterface_t* sensorInterface, uint8_t regAdr, uint16_t numBytesToWrite, uint8_t* data) { return WE_WriteReg(sensorInterface, regAdr, numBytesToWrite, data); }

/**
 * @brief Duplex communication with SPI.
 * @param[in]  sensorInterface Pointer to sensor interface.
 * @param[in]  numBytes        Number of bytes to be transmitted/received.
 * @param[in]  txData          Transmit buffer.
 * @param[out] rxData          Receive buffer.
 * @return WE_SUCCESS or WE_FAIL
 */
static inline int8_t PDDS_SPITransceive(WE_sensorInterface_t* sensorInterface, uint16_t numBytes, uint8_t* txData, uint8_t* rxData) { return WE_SPITransceive(sensorInterface, numBytes, txData, rxData); }

/**
 * @brief Build PDDS 16-bit SPI instruction header into a 2-byte buffer for SPI.
 *
 *        Bit 15    : R/W  (1 = read, 0 = write)
 *        Bits 14:13: W1:W0 — number of data bytes (Table 6.6):
 *                    00 = 1 byte
 *                    01 = 2 bytes
 *                    10 = 3 bytes
 *                    11 = 4 or more bytes
 *        Bits 12:0 : A12..A0 — register address
 *
 * @param[in]  rw       1 = read, 0 = write
 * @param[in]  reg      Register address
 * @param[in]  numBytes Number of data bytes to transfer
 * @param[out] hdr      2-byte output buffer, MSB first
 */
static void PDDS_SPI_BuildInstructionHeader(pdds_spi_cmd_read_write rw, uint8_t reg, uint16_t numBytes, uint8_t hdr[2])
{
    uint16_t w;

    switch (numBytes)
    {
        case 1:
        {
            w = 0x0; /* 00 = 1 byte  */
            break;
        }
        case 2:
        {
            w = 0x1; /* 01 = 2 bytes */
            break;
        }
        case 3:
        {
            w = 0x2; /* 10 = 3 bytes */
            break;
        }
        default:
        {
            w = 0x3; /* 11 = 4+ bytes */
            break;
        }
    }

    uint16_t addr = (uint16_t)reg & 0x1FFF; /* 13 bit address */
    uint16_t header = ((uint16_t)(rw & 0x1u) << 15) | (w << 13) | addr;

    hdr[0] = (uint8_t)(header >> 8);
    hdr[1] = (uint8_t)(header & 0xFF);
}

/**
 * @brief Returns the default sensor interface configuration.
 * @param[out] sensorInterface Sensor interface configuration (output parameter)
 * @return Error code
 */
int8_t PDDS_GetDefaultInterface(WE_sensorInterface_t* sensorInterface)
{
    if (NULL == sensorInterface)
    {
        return WE_FAIL;
    }

    *sensorInterface = PDDSDefaultSensorInterface;
    return WE_SUCCESS;
}

/**
 * @brief Activate SPI by writing the IF_CTRL register to enable SPI data read.
 * @param[in] sensorInterface Pointer to sensor interface
 * @return WE_SUCCESS if write succeeded, WE_FAIL on error
 */
int8_t PDDS_ActivateSPI(WE_sensorInterface_t* sensorInterface)
{
    if (NULL == sensorInterface)
    {
        return WE_FAIL;
    }

    /* 1 data byte → total frame = 2 header + 1 data = 3 bytes */
    uint8_t txBuf[3] = {0};
    uint8_t rxBuf[3] = {0};

    /* Build 16-bit header — R/W = 0 (write), 1 byte, IF_CTRL register 0x00 */
    PDDS_SPI_BuildInstructionHeader(PDDS_SPI_CMD_WRITE, PDDS_REG_IF_CTRL, 1U, &txBuf[0]);

    /* Activate SPI data read command */
    txBuf[2] = 0x81U;
    if (WE_SUCCESS != PDDS_SPITransceive(sensorInterface, sizeof(txBuf), txBuf, rxBuf))
    {
        return WE_FAIL;
    }

    /* Small delay to allow sensor SDO output stage to reconfigure */
    WE_Delay(50U);

    return WE_SUCCESS;
}

/**
 * @brief I2C: Write the CMD register to trigger a combined measurement conversion.
 *        Sets measurement mode and SCO bit in one transaction.
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[in] mode            Measurement mode (pdds_measMode_t)
 * @return WE_SUCCESS or WE_FAIL
 */
int8_t PDDS_I2C_SetMeasurementMode(WE_sensorInterface_t* sensorInterface, pdds_measMode_t mode)
{
    if (NULL == sensorInterface)
    {
        return WE_FAIL;
    }

    pdds_cmd_reg_t cmd = {0};
    cmd.measurement_ctrl = (uint8_t)mode;
    /* Start of conversion; auto-clears once the conversion completes */
    cmd.sco = 1U;

    if (WE_SUCCESS != PDDS_WriteReg(sensorInterface, PDDS_REG_CMD, 1, (uint8_t*)&cmd))
    {
        return WE_FAIL;
    }

    return WE_SUCCESS;
}

/**
 * @brief SPI: Write CMD register to trigger a combined measurement conversion.
 *        Sets measurement mode and SCO bit in one transaction.
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[in] mode            Measurement mode (pdds_measMode_t)
 * @return WE_SUCCESS or WE_FAIL
 */
int8_t PDDS_SPI_SetMeasurementMode(WE_sensorInterface_t* sensorInterface, pdds_measMode_t mode)
{
    if (NULL == sensorInterface)
    {
        return WE_FAIL;
    }

    uint8_t txBuf[3] = {0};
    uint8_t rxBuf[3] = {0};

    /* Build header: R/W=0 (write), PDDS_REG_CMD, 1 byte (W1:W0=00) */
    PDDS_SPI_BuildInstructionHeader(PDDS_SPI_CMD_WRITE, PDDS_REG_CMD, 1U, &txBuf[0]);

    pdds_cmd_reg_t* cmdP = (pdds_cmd_reg_t*)&txBuf[2];
    cmdP->measurement_ctrl = (uint8_t)mode;
    /* Start of conversion; auto-clears once the conversion completes */
    cmdP->sco = 1U;

    if (WE_SUCCESS != PDDS_SPITransceive(sensorInterface, sizeof(txBuf), txBuf, rxBuf))
    {
        return WE_FAIL;
    }

    /* Allow sensor to begin conversion before first DRDY poll */
    WE_Delay(50U);

    return WE_SUCCESS;
}

/**
 * @brief I2C: Poll DRDY bit until data is ready or timeout is reached.
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[in] timeout_ms      Maximum wait time in milliseconds
 * @return WE_SUCCESS when data ready, WE_FAIL on timeout or read error
 */
int8_t PDDS_I2C_Wait_DataReady(WE_sensorInterface_t* sensorInterface, uint32_t timeout_ms)
{
    const uint32_t pollStepMs = 2U;
    uint32_t waited = 0U;

    if (NULL == sensorInterface)
    {
        return WE_FAIL;
    }

    do
    {
        pdds_status_reg_t status = {0};
        if (WE_SUCCESS != PDDS_ReadReg(sensorInterface, PDDS_REG_STATUS, 1, (uint8_t*)&status))
        {
            return WE_FAIL;
        }

        if ((status.drdy != 0U) && (status.errorCode == 0U))
        {
            return WE_SUCCESS;
        }

        WE_Delay(pollStepMs);
        waited += pollStepMs;

    } while (waited < timeout_ms);

    return WE_FAIL; /* timeout */
}

/**
 * @brief SPI: Poll DRDY bit until data is ready or timeout.
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[in] timeout_ms      Maximum wait time in milliseconds
 * @return WE_SUCCESS when data ready, WE_FAIL on timeout or error
 */
int8_t PDDS_SPI_Wait_DataReady(WE_sensorInterface_t* sensorInterface, uint32_t timeout_ms)
{
    if (NULL == sensorInterface)
    {
        return WE_FAIL;
    }

    const uint32_t pollStepMs = 2U;
    uint32_t waited = 0U;

    /* 1 data byte → total frame = 2 header + 1 data = 3 bytes */
    uint8_t txBuf[3] = {0};
    uint8_t rxBuf[3] = {0};

    /* Build 16-bit header once — R/W = 1 (read), 1 byte, same register every poll */
    PDDS_SPI_BuildInstructionHeader(PDDS_SPI_CMD_READ, PDDS_REG_STATUS, 1U, &txBuf[0]);
    /* txBuf[2] = 0x00 dummy TX byte */

    do
    {
        rxBuf[0] = 0;
        rxBuf[1] = 0;
        rxBuf[2] = 0;

        if (WE_SUCCESS != PDDS_SPITransceive(sensorInterface, sizeof(txBuf), txBuf, rxBuf))
        {
            return WE_FAIL;
        }

        /* rxBuf[0], rxBuf[1] = garbage (received during header TX phase) */
        /* rxBuf[2] = actual STATUS register byte                        */
        pdds_status_reg_t* statusP = (pdds_status_reg_t*)&rxBuf[2];
        if ((statusP->drdy != 0U) && (statusP->errorCode == 0U))

        {
            return WE_SUCCESS;
        }

        WE_Delay(pollStepMs);
        waited += pollStepMs;

    } while (waited < timeout_ms);

    return WE_FAIL; /* timeout */
}

/**
 * @brief I2C: Trigger a single-shot conversion and read raw pressure and temperature.
 * @param[in]  sensorInterface Pointer to sensor interface
 * @param[out] rawPressure     Pointer to store signed 24-bit raw pressure
 * @param[out] rawTemperature  Pointer to store signed 16-bit raw temperature
 * @retval Error code
 */
int8_t PDDS_I2C_GetSingleShotRawPressureAndTemperature(WE_sensorInterface_t* sensorInterface, int32_t* rawPressure, int16_t* rawTemperature)
{
    if (NULL == sensorInterface || NULL == rawPressure || NULL == rawTemperature)
    {
        return WE_FAIL;
    }

    /* Trigger combined temperature + pressure single-shot conversion. */
    if (WE_SUCCESS != PDDS_I2C_SetMeasurementMode(sensorInterface, PDDS_MEAS_COMBINED))
    {
        return WE_FAIL;
    }

    /* Wait for conversion: T_conv_T (43 ms) + T_conv_P (43 ms) + 10 ms margin at default OSR 32768X */
    if (WE_SUCCESS != PDDS_I2C_Wait_DataReady(sensorInterface, PDDS_COMBINED_CONV_TIMEOUT_DEFAULT_MS))
    {
        return WE_FAIL;
    }

    /* Burst read: P[23:0] @ 0x06..0x08, T[15:0] @ 0x09..0x0A */
    uint8_t buf[5] = {0};
    if (WE_SUCCESS != PDDS_ReadReg(sensorInterface, PDDS_REG_DATA_P_MSB, sizeof(buf), buf))
    {
        return WE_FAIL;
    }

    /* Sign-extend 24-bit pressure to int32:
     * Place P_MSB sign bit at bit 31 via shifts, cast to int32_t so >> 8
     * becomes arithmetic (fills upper byte with sign bit). */
    *rawPressure = ((int32_t)((((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8)))) >> 8;

    /* Combine T_MSB and T_LSB, cast to int16_t for signed interpretation (LSB = 1/256 °C). */
    *rawTemperature = (int16_t)(((uint16_t)buf[3] << 8) | ((uint16_t)buf[4] << 0));

    return WE_SUCCESS;
}

/**
 * @brief SPI: Trigger a single-shot conversion and read raw pressure and temperature.
 *        Sends a combined measurement command via the 16-bit SPI instruction header,
 *        waits for DRDY, then performs a 5-byte burst read (address auto-decrements).
 * @param[in]  sensorInterface Pointer to sensor interface
 * @param[out] rawPressure     Pointer to store signed 24-bit raw pressure
 * @param[out] rawTemperature  Pointer to store signed 16-bit raw temperature (LSB = 1/256 °C)
 * @retval Error code
 */
int8_t PDDS_SPI_GetSingleShotRawPressureAndTemperature(WE_sensorInterface_t* sensorInterface, int32_t* rawPressure, int16_t* rawTemperature)
{
    if (NULL == sensorInterface || NULL == rawPressure || NULL == rawTemperature)
    {
        return WE_FAIL;
    }

    /* Step 1: Trigger combined single-shot conversion */
    if (WE_SUCCESS != PDDS_SPI_SetMeasurementMode(sensorInterface, PDDS_MEAS_COMBINED))
    {
        return WE_FAIL;
    }

    /* Step 2: Wait for conversion complete */
    if (WE_SUCCESS != PDDS_SPI_Wait_DataReady(sensorInterface, PDDS_COMBINED_CONV_TIMEOUT_DEFAULT_MS))
    {
        return WE_FAIL;
    }

    /* Step 3: Burst read — 5 data bytes starting at PDDS_REG_DATA_T_LSB
     *
     * Frame: 2-byte header + 5 data bytes = 7 bytes total (W1:W0 = 11)
     *
     * rxBuf[0..1] = garbage  (header TX phase)
     * rxBuf[2]    = T_LSB    (0x0A)
     * rxBuf[3]    = T_MSB    (0x09)
     * rxBuf[4]    = P_LSB    (0x08)
     * rxBuf[5]    = P_CSB    (0x07)
     * rxBuf[6]    = P_MSB    (0x06)                */
    uint8_t txBuf[7] = {0};
    uint8_t rxBuf[7] = {0};

    /* Build header: R/W=1 (read), start=PDDS_REG_DATA_T_LSB, 5 bytes (W1:W0=11) */
    PDDS_SPI_BuildInstructionHeader(PDDS_SPI_CMD_READ, PDDS_REG_DATA_T_LSB, 5U, &txBuf[0]);

    if (WE_SUCCESS != PDDS_SPITransceive(sensorInterface, sizeof(txBuf), txBuf, rxBuf))
    {
        return WE_FAIL;
    }

    /* Sign-extend 24-bit pressure to int32:
     * Place P_MSB sign bit at bit 31 via shifts, cast to int32_t so >> 8
     * becomes arithmetic (fills upper byte with sign bit). */
    *rawPressure = ((int32_t)((((uint32_t)rxBuf[6] << 24) | ((uint32_t)rxBuf[5] << 16) | ((uint32_t)rxBuf[4] << 8)))) >> 8;

    /* Combine T_MSB and T_LSB, cast to int16_t for signed interpretation (LSB = 1/256 °C). */
    *rawTemperature = (int16_t)(((uint16_t)rxBuf[3] << 8) | ((uint16_t)rxBuf[2] << 0));

    return WE_SUCCESS;
}

#ifdef WE_USE_FLOAT

/**
 * @brief Read pressure and temperature as physical values.
 * @param[in]  sensorInterface Pointer to sensor interface
 * @param[in]  sensorType      Sensor variant (PDDS_SensorType_t), selects pressure scale factor
 * @param[out] pressure_kPa    Pointer to pressure value in kPa
 * @param[out] temperature_C   Pointer to temperature value in degrees Celsius
 * @retval Error code
 */
int8_t PDDS_getPressureAndTemperature_float(WE_sensorInterface_t* sensorInterface, PDDS_SensorType_t sensorType, float* pressure_kPa, float* temperature_C)
{
    if (NULL == sensorInterface || NULL == pressure_kPa || NULL == temperature_C)
    {
        return WE_FAIL;
    }

    int32_t rawP = 0;
    int16_t rawT = 0;

    switch (sensorInterface->interfaceType)
    {
        case WE_i2c:
        {
            if (WE_SUCCESS != PDDS_I2C_GetSingleShotRawPressureAndTemperature(sensorInterface, &rawP, &rawT))
            {
                return WE_FAIL;
            }
        }
        break;

        case WE_spi:
        {
            if (WE_SUCCESS != PDDS_SPI_GetSingleShotRawPressureAndTemperature(sensorInterface, &rawP, &rawT))
            {
                return WE_FAIL;
            }
        }
        break;

        default:
        {
            return WE_FAIL;
        }
    }

    /* Temperature: LSB = 1/256 °C */
    *temperature_C = (float)rawT / 256.0f;

    /* Pressure: raw to kPa — conversion depends on sensor variant */
    switch (sensorType)
    {
        case PDDS_pdds0:
            *pressure_kPa = (float)rawP / PDDS_PRESSURE_SCALE_35KPA;
            break;

        case PDDS_pdds1:
            *pressure_kPa = (float)rawP / PDDS_PRESSURE_SCALE_100KPA;
            break;

        case PDDS_pdds2:
            *pressure_kPa = (float)rawP / PDDS_PRESSURE_SCALE_700KPA;
            break;

        default:
            /* Invalid PDDS sensor type */
            return WE_FAIL;
    }

    return WE_SUCCESS;
}

#endif /* WE_USE_FLOAT */

/**
 * @brief Set OSR for pressure (0xA6) and temperature (0xA7) channels.
 *        Performs read-modify-write on bits [2:0] only; all other bits are preserved.
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[in] osrP            OSR for pressure channel    (pdds_osr_t)
 * @param[in] osrT            OSR for temperature channel (pdds_osr_t)
 * @retval Error code
 */
int8_t PDDS_I2C_SetOSR(WE_sensorInterface_t* sensorInterface, pdds_osr_t osrP, pdds_osr_t osrT)
{
    if (NULL == sensorInterface)
    {
        return WE_FAIL;
    }

    pdds_p_config_reg_t pCfg = {0};
    pdds_t_config_reg_t tCfg = {0};

    if (WE_SUCCESS != PDDS_ReadReg(sensorInterface, PDDS_REG_P_CONFIG, 1U, (uint8_t*)&pCfg))
    {
        return WE_FAIL;
    }

    pCfg.osr_p = (uint8_t)osrP;

    if (WE_SUCCESS != PDDS_WriteReg(sensorInterface, PDDS_REG_P_CONFIG, 1U, (uint8_t*)&pCfg))
    {
        return WE_FAIL;
    }

    if (WE_SUCCESS != PDDS_ReadReg(sensorInterface, PDDS_REG_T_CONFIG, 1U, (uint8_t*)&tCfg))
    {
        return WE_FAIL;
    }

    tCfg.osr_t = (uint8_t)osrT;

    if (WE_SUCCESS != PDDS_WriteReg(sensorInterface, PDDS_REG_T_CONFIG, 1U, (uint8_t*)&tCfg))
    {
        return WE_FAIL;
    }

    return WE_SUCCESS;
}

/**
 * @brief SPI: Set OSR for pressure (0xA6) and temperature (0xA7) channels.
 *        Performs read-modify-write on bits [2:0] only; all other bits are preserved.
 *        Each register requires a separate read frame and write frame via the
 *        16-bit SPI instruction header.
 *
 * @param[in] sensorInterface Pointer to sensor interface
 * @param[in] osrP            OSR for pressure channel    (pdds_osr_t)
 * @param[in] osrT            OSR for temperature channel (pdds_osr_t)
 * @retval Error code
 */
int8_t PDDS_SPI_SetOSR(WE_sensorInterface_t* sensorInterface, pdds_osr_t osrP, pdds_osr_t osrT)
{
    if (NULL == sensorInterface)
    {
        return WE_FAIL;
    }

    uint8_t txBuf[3] = {0};
    uint8_t rxBuf[3] = {0};

    /* --- P_CONFIG (0xA6) read-modify-write --- */

    /* Read P_CONFIG: R/W=1 (read), 1 byte */
    PDDS_SPI_BuildInstructionHeader(PDDS_SPI_CMD_READ, PDDS_REG_P_CONFIG, 1U, &txBuf[0]);
    txBuf[2] = 0x00U;

    if (WE_SUCCESS != PDDS_SPITransceive(sensorInterface, sizeof(txBuf), txBuf, rxBuf))
    {
        return WE_FAIL;
    }

    /* Write P_CONFIG: R/W=0 (write), 1 byte */
    PDDS_SPI_BuildInstructionHeader(PDDS_SPI_CMD_WRITE, PDDS_REG_P_CONFIG, 1U, &txBuf[0]);
    txBuf[2] = rxBuf[2];
    pdds_p_config_reg_t* pCfgP = (pdds_p_config_reg_t*)&txBuf[2];
    pCfgP->osr_p = (uint8_t)osrP;

    if (WE_SUCCESS != PDDS_SPITransceive(sensorInterface, sizeof(txBuf), txBuf, rxBuf))
    {
        return WE_FAIL;
    }

    /* --- T_CONFIG (0xA7) read-modify-write --- */

    /* Read T_CONFIG: R/W=1 (read), 1 byte */
    PDDS_SPI_BuildInstructionHeader(PDDS_SPI_CMD_READ, PDDS_REG_T_CONFIG, 1U, &txBuf[0]);
    txBuf[2] = 0x00U;

    if (WE_SUCCESS != PDDS_SPITransceive(sensorInterface, sizeof(txBuf), txBuf, rxBuf))
    {
        return WE_FAIL;
    }

    /* Write T_CONFIG: R/W=0 (write), 1 byte */
    PDDS_SPI_BuildInstructionHeader(PDDS_SPI_CMD_WRITE, PDDS_REG_T_CONFIG, 1U, &txBuf[0]);
    txBuf[2] = rxBuf[2];
    pdds_t_config_reg_t* tCfgP = (pdds_t_config_reg_t*)&txBuf[2];
    tCfgP->osr_t = (uint8_t)osrT;

    if (WE_SUCCESS != PDDS_SPITransceive(sensorInterface, sizeof(txBuf), txBuf, rxBuf))
    {
        return WE_FAIL;
    }

    return WE_SUCCESS;
}
