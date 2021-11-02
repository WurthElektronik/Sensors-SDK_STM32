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

#ifndef _WSEN_ITDS_H
#define _WSEN_ITDS_H

/**         Includes         */

#include <stdint.h>

#include "../WeSensorsSDK.h"


/**         ITDS 2533020201601 DEVICE_ID         */

#define ITDS_DEVICE_ID_VALUE              0x44      /* This is the expected answer when requesting the ITDS_DEVICE_ID_REG */


/**         Available ITDS I2C slave addresses         */

#define ITDS_ADDRESS_I2C_0                0x18      /* When SAO of ITDS is connected to ground */
#define ITDS_ADDRESS_I2C_1                0x19      /* When SAO of ITDS is connected to positive supply voltage */


/** Register address definitions **/

#define ITDS_T_OUT_L_REG                  0x0D      /* Temperature output LSB value register */
#define ITDS_T_OUT_H_REG                  0x0E      /* Temperature output MSB value register */
#define ITDS_DEVICE_ID_REG                0x0F      /* Device ID register */
/* Registers 0x10 - 0x1F are reserved. They contain factory calibration values that shall not be changed */
#define ITDS_CTRL_1_REG                   0x20      /* Control register 1 */
#define ITDS_CTRL_2_REG                   0x21      /* Control register 2 */
#define ITDS_CTRL_3_REG                   0x22      /* Control register 3 */
#define ITDS_CTRL_4_REG                   0x23      /* Control register 4 */
#define ITDS_CTRL_5_REG                   0x24      /* Control register 5 */
#define ITDS_CTRL_6_REG                   0x25      /* Control register 6 */
#define ITDS_T_OUT_REG                    0x26      /* Temperature output data in 8 bit resolution register */
#define ITDS_STATUS_REG                   0x27      /* Status register */
#define ITDS_X_OUT_L_REG                  0x28      /* X axis acceleration output LSB value register */
#define ITDS_X_OUT_H_REG                  0x29      /* X axis acceleration output MSB value register */
#define ITDS_Y_OUT_L_REG                  0x2A      /* Y axis acceleration output LSB value register */
#define ITDS_Y_OUT_H_REG                  0x2B      /* Y axis acceleration output MSB value register */
#define ITDS_Z_OUT_L_REG                  0x2C      /* Z axis acceleration output LSB value register */
#define ITDS_Z_OUT_H_REG                  0x2D      /* Z axis acceleration output MSB value register */
#define ITDS_FIFO_CTRL_REG                0x2E      /* FIFO control register */
#define ITDS_FIFO_SAMPLES_REG             0x2F      /* FIFO samples register */
#define ITDS_TAP_X_TH_REG                 0x30      /* Tap recognition threshold in X direction register */
#define ITDS_TAP_Y_TH_REG                 0x31      /* Tap recognition threshold in Y direction register */
#define ITDS_TAP_Z_TH_REG                 0x32      /* Tap recognition threshold in Z direction register */
#define ITDS_INT_DUR_REG                  0x33      /* Interrupt duration register */
#define ITDS_WAKE_UP_TH_REG               0x34      /* Wake-up threshold register */
#define ITDS_WAKE_UP_DUR_REG              0x35      /* Wake-up duration register */
#define ITDS_FREE_FALL_REG                0x36      /* Free-fall register */
#define ITDS_STATUS_DETECT_REG            0x37      /* Status detect register */
#define ITDS_WAKE_UP_EVENT_REG            0x38      /* Wake-up event register */
#define ITDS_TAP_EVENT_REG                0x39      /* Tap event register  */
#define ITDS_6D_EVENT_REG                 0x3A      /* 6D (orientation change) event register */
#define ITDS_ALL_INT_EVENT_REG            0x3B      /* All interrupts event register */
#define ITDS_X_OFS_USR_REG                0x3C      /* Offset value for X axis register */
#define ITDS_Y_OFS_USR_REG                0x3D      /* Offset value for Y axis register */
#define ITDS_Z_OFS_USR_REG                0x3E      /* Offset value for Z axis register */
#define ITDS_CTRL_7_REG                   0x3F      /* Control register 7 */


/** Register type definitions **/

/**
* CTR_1_REG
* Address 0x20
* Type  R/W
* Default value: 0x00
*
* ODR[3:0]  |       Power down / data rate configuration
* --------------------------------------------------------------
*   0000    | Power down
*           |
*           | High performance    Normal mode   Low power mode
*   0001    |    12.5 Hz            12.5 Hz        1.6 Hz
*   0010    |    12.5 Hz            12.5 Hz        12.5 Hz
*   0011    |    25 Hz              25 Hz          25 Hz
*   0100    |    50 Hz              50 Hz          50 Hz
*   0101    |    100 Hz             100 Hz         100 Hz
*   0110    |    200 Hz             200 Hz         200 Hz
*   0111    |    400 Hz             200 Hz         200 Hz
*   1000    |    800 Hz             800 Hz         200 Hz
*   1001    |    1600Hz             1600Hz         200 Hz
*
*
* MODE[1:0] |                      Operating mode and resolution
* ----------------------------------------------------------------------------------------
*   00      |    Normal mode (14-bit resolution) / Low power mode (12-bit resolution)
*   01      |    High performance mode (14-bit resolution)
*   10      |    Single data conversion on demand mode (12/14-bit resolution)
*   11      |    Unused
*/
typedef struct
{
  uint8_t powerMode : 2;          /* LP_MODE[1:0]: Select normal mode or low power mode. Default 00 [00: low power mode; 10: normal mode] */
  uint8_t operatingMode : 2;      /* MODE[1:0]: Select the operating mode and resolution. Default 00 */
  uint8_t outputDataRate : 4;     /* ODR[3:0]: Output data rate selection. Default 0000 */
} ITDS_ctrl1_t;

/**
* CTR_2_REG
* Address 0x21
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t notUsed01 : 1;        /* This bit must be set to 0 for proper operation of the device. */
  uint8_t i2cDisable : 1;       /* I2C_DISABLE: Disable I2C digital Interface. Default: 0 (0: enabled; 1: disabled). */
  uint8_t autoAddIncr : 1;      /* IF_ADD_INC: Register address automatically incremented during a multiple byte access with I2C interface. Default: 1. (0: disable; 1: enable). */
  uint8_t blockDataUpdate : 1;  /* BDU: Block data update. 0 - continuous update; 1 - output registers are not updated until MSB and LSB have been read. */
  uint8_t disCSPullUp : 1;      /* CP_PU_DISC: Disconnect pull up to CS pin. Default: 0 (0: connected; 1: disconnected). */
  uint8_t notUsed02 : 1;        /* This bit must be set to 0 for proper operation of the device. */
  uint8_t softReset : 1;        /* SOFT_RESET: Software reset. 0: normal mode; 1: SW reset; Self-clearing upon completion. */
  uint8_t boot : 1;             /* BOOT: Set this bit to 1 to initiate boot sequence. 0: normal mode; 1: Execute boot sequence. Self-clearing upon completion. */
} ITDS_ctrl2_t;

/**
* CTR_3_REG
* Address 0x22
* Type  R/W
* Default value: 0x00
*
*
*   ST[1:0]    |     Self-test mode
* -------------------------------------------
*   00         |     Normal mode
*   01         |     Positive sign self-test
*   10         |     Negative sign self-test
*   11         |              -
*/
typedef struct
{
  uint8_t startSingleDataConv : 1;    /* SLP_MODE_1: Request single data conversion */
  uint8_t singleConvTrigger : 1;      /* SLP_MODE_SEL: Single data conversion (on-demand) trigger signal. 0: Triggered by external signal on INT_1, 1: Triggered by writing 1 to SLP_MODE_1. */
  uint8_t notUsed01 : 1;              /* This bit must be set to 0 for proper operation of the device */
  uint8_t intActiveLevel : 1;         /* H_LACTIVE: Interrupt active level. Default: 0 (0: active high; 1: active low) */
  uint8_t enLatchedInterrupt : 1;     /* LIR: Enable latched interrupt. Default: 0. (0: disabled; 1: enabled) */
  uint8_t intPinConf : 1;             /* PP_OD: Push-pull/open-drain selection on interrupt pad. Default: 0 (0: push-pull; 1: open-drain) */
  uint8_t selfTestMode : 2;           /* ST[1:0]: Select self test mode. Default: 00. */
} ITDS_ctrl3_t;

/**
* CTR_4_REG
* Address 0x23
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t dataReadyINT0 : 1;      /* INT0_DRDY: Data-ready interrupt signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
  uint8_t fifoThresholdINT0 : 1;  /* INT0_FTH: FIFO threshold interrupt signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled)) */
  uint8_t fifoFullINT0 : 1;       /* INT0_DIFF5: FIFO full interrupt signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
  uint8_t doubleTapINT0 : 1;      /* INT0_TAP: Double-tap recognition signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
  uint8_t freeFallINT0 : 1;       /* INT0_FF: Free-fall recognition signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
  uint8_t wakeUpINT0 : 1;         /* INT0_WU: Wake-up recognition signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
  uint8_t singleTapINT0 : 1;      /* INT0_SINGLE_TAP: Single-tap recognition signal is routed to INT_0 pin: Default: 0 (0: disabled, 1: enabled) */
  uint8_t sixDINT0 : 1;           /* INT0_6D: 6D recognition signal is routed to INT_0 pin. Default: 0 (0: disabled, 1: enabled) */
} ITDS_ctrl4_t;

/**
* CTR_5_REG
* Address 0x24
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t dataReadyINT1 : 1;              /* INT1_DRDY: Data-ready interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
  uint8_t fifoThresholdINT1 : 1;          /* INT1_FTH: FIFO threshold interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled)) */
  uint8_t fifoFullINT1 : 1;               /* INT1_DIFF5: FIFO full interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
  uint8_t fifoOverrunINT1 : 1;            /* INT1_OVR: FIFO overrun interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
  uint8_t tempDataReadyINT1 : 1;          /* INT1_DRDY_T: Temperature data-ready interrupt signal  is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
  uint8_t bootStatusINT1 : 1;             /* INT1_BOOT: Boot status interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
  uint8_t sleepStatusChangeINT1 : 1;      /* INT1_SLEEP_CHG: Sleep change status interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
  uint8_t sleepStateINT1 : 1;             /* INT1_SLEEP_STATE: Sleep state interrupt signal is routed to INT_1 pin. Default: 0 (0: disabled, 1: enabled) */
} ITDS_ctrl5_t;

/**
* CTR_6_REG
* Address 0x25
* Type  R/W
* Default value: 0x00
*
*
*   BW_FILT[1:0]    |          Bandwidth selection
* -------------------------------------------------------------
*     00            |    ODR/2 (except for ODR = 1600 Hz, 400 Hz)
*     01            |    ODR/4 (High pass / Low pass filter)
*     10            |    ODR/10 (High pass / Low pass filter)
*     11            |    ODR/20 (High pass / Low pass filter)
*
*
*   FS[1:0]    |   Full scale selection
* ---------------------------------------
*       00     |          ±2g
*       01     |          ±4g
*       10     |          ±8g
*       11     |          ±16g
*/
typedef struct
{
  uint8_t notUsed01 : 1;        /* This bit must be set to 0 for proper operation of the device */
  uint8_t notUsed02 : 1;        /* This bit must be set to 0 for proper operation of the device */
  uint8_t enLowNoise : 1;       /* LOW_NOISE: low noise configuration (0: disabled; 1: enabled) */
  uint8_t filterPath : 1;       /* FDS: Filtered path configuration. Default value: 0. (0: low pass filter; 1: high pass filter) */
  uint8_t fullScale : 2;        /* FS[1:0]: Full scale selection */
  uint8_t filterBandwidth : 2;  /* BW_FILT[1:0]: Filter bandwidth selection  */
} ITDS_ctrl6_t;

/**
* STATUS_REG
* Address 0x27
* Type  R/W
* Default value: 0x00
*
* Note: The status register is partially duplicated to the STATUS_DETECT_REG register.
*/
typedef struct
{
  uint8_t dataReady : 1;        /* DRDY: Acceleration data-ready status bit (0: not ready, 1: X-, Y- and Z-axis new data available) */
  uint8_t freeFall : 1;         /* FF_IA: Free-fall event detection bit (0: free-fall event not detected; 1: free-fall event detected) */
  uint8_t sixDDetection : 1;    /* 6D_IA: Source of change in position portrait/landscape/face-up/face-down. (0: no event detected, 1: change in position detected) */
  uint8_t singleTap : 1;        /* SINGLE_TAP: Single-tap event status bit (0: Single-tap event not detected, 1: Single-tap event detected) */
  uint8_t doubleTap : 1;        /* DOUBLE_TAP: Double-tap event status bit (0: Double-tap event not detected, 1: Double-tap event detected) */
  uint8_t sleepState : 1;       /* SLEEP_STATE: Sleep event status bit (0: Sleep event not detected, 1: Sleep event detected) */
  uint8_t wakeUp : 1;           /* WU_IA: Wake-up event detection status bit (0: Wake-up event not detected, 1: Wake-up event detected) */
  uint8_t fifoThreshold : 1;    /* FIFO_THS: FIFO threshold status bit (0: FIFO filling is lower than threshold level, 1: FIFO filling is equal to or higher than the threshold level) */
} ITDS_status_t;

/**
* FIFO_CTRL_REG
* Address 0x2E
* Type  R/W
* Default value: 0x00
*
* FMODE[2:0]     |             Mode Description
* --------------------------------------------------------------------
* 000            |     Enable bypass mode and FIFO buffer is turned off (not active)
* 001            |     Enable FIFO mode
* 010            |     Reserved
* 011            |     Enable continuous to FIFO mode
* 100            |     Enable bypass to continuous mode
* 101            |     Reserved
* 110            |     Enable continuous mode
* 111            |     Reserved
*/
typedef struct
{
  uint8_t fifoThresholdLevel : 5;   /* FTH[4:0]: Set the FIFO threshold level */
  uint8_t fifoMode : 3;             /* FMODE[2:0]: Select the FIFO mode */
} ITDS_fifoCtrl_t;

/**
* FIFO_SAMPLES_REG
* Address 0x2F
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t fifoFillLevel : 6;       /* Diff[5:0]: Current fill level of FIFO i.e. the number of unread samples (’000000’ = FIFO empty, ’100000’ = FIFO full, 32 unread samples) */
  uint8_t fifoOverrunState : 1;    /* FIFO_OVR: FIFO overrun status (0: FIFO is not completely filled, 1: FIFO is completely filled and at least one sample has been overwritten) */
  uint8_t fifoThresholdState : 1;  /* FIFO_FTH: FIFO threshold status bit (0: FIFO filling is lower than threshold level, 1: FIFO filling is equal to or higher than the threshold level) */
} ITDS_fifoSamples_t;

/**
* TAP_X_TH_REG
* Address 0x30
* Type  R/W
* Default value: 0x00
*
*   6D_THS[1:0]  |   Threshold definition (degrees)
* -------------------------------------------
*        00      |       6  (80 degrees)
*        01      |       11 (70 degrees)
*        10      |       16 (60 degrees)
*        11      |       21 (50 degrees)
*
*/
typedef struct
{
  uint8_t xAxisTapThreshold : 5;        /* TAP_THSX_[4:0]: Threshold for tap recognition at FS = ±2g in X direction */
  uint8_t sixDThreshold : 2;            /* 6D_THS[1:0]: Threshold definition (degrees) */
  uint8_t fourDDetectionEnabled : 1;    /* 4D_EN: Enable 4D portrait/landscape detection. (0: 4D mode disabled; 1: portrait/landscape detection and face-up/face-down detection enabled) */
} ITDS_tapXThreshold_t;

/**
* TAP_Y_TH_REG
* Address 0x31
* Type  R/W
* Default value: 0x00
*
* TAP_PRIOR[2:0]  |  Max Priority  | Mid Priority  | Min Priority
* ------------------------------------------------------
*     000         |    X           |     Y         |    Z
*     001         |    Y           |     X         |    Z
*     010         |    X           |     Z         |    Y
*     011         |    Z           |     Y         |    X
*     100         |    X           |     Y         |    Z
*     101         |    Y           |     Z         |    X
*     110         |    Z           |     X         |    Y
*     111         |    Z           |     Y         |    X
*/
typedef struct
{
  uint8_t yAxisTapThreshold : 5;  /* TAP_THSY_[4:0]: Threshold for tap recognition at FS = ±2g in Y direction */
  uint8_t tapAxisPriority : 3;    /* TAP_PRIOR[2:0]: Select the axis priority for tap detection */
} ITDS_tapYThreshold_t;

/**
* TAP_Z_TH_REG
* Address 0x32
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t zAxisTapThreshold : 5;  /* TAP_THSZ_[4:0]: Threshold for tap recognition at FS: ±2g in Z direction. */
  uint8_t enTapZ : 1;             /* TAP_Z_EN: Enables tap recognition for Z axis. (0: disabled, 1: enabled) */
  uint8_t enTapY : 1;             /* TAP_Y_EN: Enables tap recognition for Y axis. (0: disabled, 1: enabled) */
  uint8_t enTapX : 1;             /* TAP_X_EN: Enables tap recognition for X axis. (0: disabled, 1: enabled) */
} ITDS_tapZThreshold_t;

/**
* INT_DUR_REG
* Address 0x33
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t shock : 2;    /* SHOCK[1:0]: Defines the maximum duration of over-threshold event when detecting taps */
  uint8_t quiet : 2;    /* QUIET[1:0]: Defines the expected quiet time after a tap detection */
  uint8_t latency : 4;  /* LATENCY[3:0]: Defines the maximum duration time gap for double-tap recognition */
} ITDS_intDuration_t;

/**
* WAKE_UP_TH_REG
* Address 0x34
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t wakeUpThreshold : 6;      /* WK_THS[5:0]: Defines wake-up threshold, 6-bit unsigned 1 LSB = 1/64 of FS. Default value: 000000 */
  uint8_t enInactivityEvent: 1;     /* SLEEP_ON: Enables inactivity (sleep). Default value: 0 (0: sleep disabled, 1: sleep enabled) */
  uint8_t enDoubleTapEvent : 1;     /* SINGLE_DOUBLE_TAP: Enable double-tap event. Default value: 0 (0: enable only single-tap, 1: enable both: single and double-tap) */
} ITDS_wakeUpThreshold_t;

/**
* WAKE_UP_DUR_REG
* Address 0x35
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t sleepDuration : 4;        /* SLEEP_DUR[3:0]: Defines the sleep mode duration. Default value is SLEEP_DUR[3:0] = 0000 (which is 16 * 1/ODR) 1 LSB = 512 * 1/ODR */
  uint8_t enStationary : 1;         /* STATIONARY: Enables stationary detection / motion detection with no automatic ODR change when detecting stationary state. Default value: 0 (0: disabled, 1: enabled) */
  uint8_t wakeUpDuration : 2;       /* WAKE_DUR[1:0]: This parameter defines the wake-up duration. 1 LSB = 1 * 1/ODR */
  uint8_t freeFallDurationMSB : 1;  /* FF_DUR5: This bit defines the free-fall duration. Combined with FF_DUR [4:0] bit in FREE_FALL (0x36) register. 1 LSB = 1 * 1/ODR */
} ITDS_wakeUpDuration_t;

/**
* FREE_FALL_REG
* Address 0x36
* Type  R/W
* Default value: 0x00
*
*   FF_TH[2:0]  |  Decoded threshold
* -----------------------------------------
*     000       |        5
*     001       |        7
*     010       |        8
*     011       |        10
*     100       |        11
*     101       |        13
*     110       |        15
*     111       |        16
*/
typedef struct
{
  uint8_t freeFallThreshold : 3;  /* FF_TH[2:0]: Encoded free-fall threshold value. The decoded value can be multiplied with 31.25mg to get the used threshold. */
  uint8_t freeFallDurationLSB: 5; /* FF_DUR[4:0]: Defines free-fall duration. Is combined with FF_DUR5 bit in WAKE_UP_DUR (0x35) register. 1 LSB = 1 * 1/ODR */
} ITDS_freeFall_t;

/**
* STATUS_DETECT_REG
* Address 0x37
* Type  R/W
* Default value: 0x00
*
* Note: This register is partially duplicated from the STATUS_REG register.
*/
typedef struct
{
  uint8_t dataReady : 1;            /* DRDY: Acceleration data-ready status (0: not ready, 1: X-, Y- and Z-axis new data available) */
  uint8_t freeFall : 1;             /* FF_IA: Free-fall event detection status (0: Free-fall event not detected, 1: Free-fall event detected) */
  uint8_t sixDDetection : 1;        /* 6D_IA: Orientation change detection status (0: No event detected, 1: A change in orientation has been detected) */
  uint8_t singleTap : 1;            /* SINGLE_TAP: Single-tap event status (0: Single-tap event not detected; 1: Single-tap event detected) */
  uint8_t doubleTap : 1;            /* DOUBLE_TAP: Double-tap event status (0: Double-tap event not detected, 1: Double-tap event detected) */
  uint8_t sleepState : 1;           /* SLEEP_STATE_IA: Sleep event status (0: Sleep event not detected, 1: Sleep event detected) */
  uint8_t temperatureDataReady: 1;  /* DRDY_T: Temperature available status (0: Data not available, 1: A new set of data is available) */
  uint8_t fifoOverrunState : 1;     /* OVR: FIFO overrun status (0: FIFO is not completely filled, 1: FIFO is completely filled and at least one sample has been overwritten) */
} ITDS_statusDetect_t;

/**
* WAKE_UP_EVENT_REG
* Address 0x38
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t wakeUpZ : 1;        /* Z_WU: Wake-up event on Z-axis status (0: Wake-up event on Z-axis not detected, 1: Wake-up event on Z-axis detected) */
  uint8_t wakeUpY : 1;        /* Y_WU: Wake-up event on Y-axis status (0: Wake-up event on Y-axis not detected, 1: Wake-up event on Y-axis detected) */
  uint8_t wakeUpX : 1;        /* X_WU: Wake-up event on X-axis status (0: Wake-up event on X-axis not detected, 1: Wake-up event on X-axis detected) */
  uint8_t wakeUpState : 1;    /* WU_IA: Wake-up event detection status (0: Wake-up event not detected, 1: Wake-up event detected) */
  uint8_t sleepState : 1;     /* SLEEP_STATE_IA: Sleep event status (0: Sleep event not detected, 1: Sleep event detected) */
  uint8_t freeFallState : 1;  /* FF_IA: Free-fall event detection status (0: FF event not detected, 1: FF event detected) */
  uint8_t notUsed01 : 1;      /* This bit must be set to 0 for proper operation of the device */
  uint8_t notUsed02 : 1;      /* This bit must be set to 0 for proper operation of the device */
} ITDS_wakeUpEvent_t;

/**
* TAP_EVENT_REG
* Address 0x39
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t tapZAxis : 1;       /* Z_TAP: Tap event detection on Z-axis status (0: Tap event on Z-axis not detected, 1: Tap event on Z-axis detected) */
  uint8_t tapYAxis : 1;       /* Y_TAP: Tap event detection on Y-axis status (0: Tap event on Y-axis not detected, 1: Tap event on Y-axis detected) */
  uint8_t tapXAxis : 1;       /* X_TAP: Tap event detection on X-axis status (0: Tap event on X-axis not detected, 1: Tap event on X-axis detected) */
  uint8_t tapSign : 1;        /* TAP_SIGN: Sign of acceleration detected by tap event (0: Tap in positive direction, 1: Tap in negative direction) */
  uint8_t doubleState : 1;    /* DOUBLE_TAP: Double-tap event status (0: Double-tap event not detected, 1: Double-tap event detected) */
  uint8_t singleState : 1;    /* SINGLE_TAP: Single-tap event status (0: Single-tap event not detected, 1: Single-tap event detected) */
  uint8_t tapEventState : 1;  /* TAP_IA: Tap event status (0: Tap event not detected, 1: Tap event detected) */
  uint8_t notUsed01 : 1;      /* This bit must be set to 0 for proper operation of the device */
} ITDS_tapEvent_t;

/**
* 6D_EVENT_REG
* Address 0x3A
* Type  R/W
* Default value: 0x00
*
* xhOverThreshold, yhOverThreshold, zhOverThreshold: Is set high when the face perpendicular to the
* Z (Y, X) axis is almost flat and the acceleration measured on the Z (Y, X) axis is positive and in
* the absolute value bigger than the threshold.
*
* xlOverThreshold (ylOverThreshold, zlOverThreshold): Is set high when the face perpendicular to the
* Z (Y, X) axis is almost flat and the acceleration measured on the Z (Y, X) axis is negative and in
* the absolute value bigger than the threshold.
*/
typedef struct
{
  uint8_t xlOverThreshold : 1;  /* 1: XL threshold exceeded, 0: XL threshold not exceeded */
  uint8_t xhOverThreshold : 1;  /* 1: XH threshold exceeded, 0: XH threshold not exceeded */
  uint8_t ylOverThreshold : 1;  /* 1: YL threshold exceeded, 0: YL threshold not exceeded */
  uint8_t yhOverThreshold : 1;  /* 1: YH threshold exceeded, 0: YH threshold not exceeded */
  uint8_t zlOverThreshold : 1;  /* 1: ZL threshold exceeded, 0: ZL threshold not exceeded */
  uint8_t zhOverThreshold : 1;  /* 1: ZH threshold exceeded, 0: ZH threshold not exceeded */
  uint8_t sixDChange : 1;       /* 6D_IA: Orientation change detection status (0: No event detected, 1: A change in orientation has been detected) */
  uint8_t notUsed01 : 1;        /* This bit must be set to 0 for proper operation of the device */
} ITDS_6dEvent_t;

/**
* ALL_INT_EVENT_REG
* Address 0x3B
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t freeFallState : 1;      /* FF_IA: Free-fall event detection status (0: free-fall event not detected, 1: free-fall event detected) */
  uint8_t wakeupState : 1;        /* WU_IA: Wake-up event detection status (0: wake-up event not detected, 1: wake-up event detected) */
  uint8_t singleTapState : 1;     /* SINGLE_TAP: Single-tap event status (0: single-tap event not detected, 1: single-tap event detected) */
  uint8_t doubleTapState : 1;     /* DOUBLE_TAP: Double-tap event status (0: double-tap event not detected, 1: double-tap event detected) */
  uint8_t sixDState : 1;          /* 6D_IA: Orientation change detection status (0: No event detected, 1: A change in orientation has been detected) */
  uint8_t sleepChangeState : 1;   /* SLEEP_CHANGE_IA: Sleep change status (0: Sleep change not detected; 1: Sleep change detected) */
  uint8_t notUsed01 : 1;          /* This bit must be set to 0 for proper operation of the device */
  uint8_t notUsed02 : 1;          /* This bit must be set to 0 for proper operation of the device */
} ITDS_allInterruptEvents_t;

/**
* CTRL_7_REG
* Address 0x3F
* Type  R/W
* Default value: 0x00
*/
typedef struct
{
  uint8_t lowPassOn6D : 1;         /* 0: ODR/2 low pass filtered data sent to 6D interrupt function (default), 1: LPF_1 output data sent to 6D interrupt function */
  uint8_t highPassRefMode : 1;     /* HP_REF_MODE: Enables high-pass filter reference mode. Default: 0 (0: high-pass filter reference mode disabled, 1: high-pass filter reference mode enabled) */
  uint8_t userOffset : 1;          /* USR_OFF_W: Defines the selection of weight of the user offset words specified by X_OFS_USR[7:0], Y_OFS_USR[7:0] and Z_OFS_USR[7:0] bits (0:977 µg/LSB, 1: 15.6 mg/LSB) */
  uint8_t applyWakeUpOffset : 1;   /* USR_OFF_ON_WU: Enable application of user offset value to data for wake-up function only */
  uint8_t applyOffset : 1;         /* USR_OFF_ON_OUT: Enable application of user offset value to output data registers. FDS: bit in CTRL_6 (0x25) must be set to ’0’-logic (low-pass path selected) */
  uint8_t enInterrupts : 1;        /* INTERRUPTS_ENABLE: Enable interrupts */
  uint8_t INT1toINT0 : 1;          /* INT1_ON_INT0: Defines signal routing (0: normal mode, 1: all signals available only on INT_1 are routed to INT_0) */
  uint8_t drdyPulse : 1;           /* DRDY_PULSED: Switches between latched and pulsed mode for data ready interrupt (0: latched mode is used, 1: pulsed mode enabled for data-ready) */

} ITDS_ctrl7_t;


/**         Functional type definitions         */

typedef enum
{
  ITDS_disable = 0,
  ITDS_enable = 1
} ITDS_state_t;

typedef enum
{
  ITDS_positive = 0,
  ITDS_negative = 1
} ITDS_tapSign_t;

typedef enum
{
  ITDS_odr0,    /* Power down */
                /* High performance   Normal mode   Low power mode */
  ITDS_odr1,    /* 12.5 Hz              12.5 Hz        1.6 Hz      */
  ITDS_odr2,    /* 12.5 Hz              12.5 Hz        12.5 Hz     */
  ITDS_odr3,    /* 25 Hz                25 Hz          25 Hz       */
  ITDS_odr4,    /* 50 Hz                50 Hz          50 Hz       */
  ITDS_odr5,    /* 100 Hz               100 Hz         100 Hz      */
  ITDS_odr6,    /* 200 Hz               200 Hz         200 Hz      */
  ITDS_odr7,    /* 400 Hz               200 Hz         200 Hz      */
  ITDS_odr8,    /* 800 Hz               800 Hz         200 Hz      */
  ITDS_odr9     /* 1600Hz               1600Hz         200 Hz      */
} ITDS_outputDataRate_t;

typedef enum
{
  ITDS_normalOrLowPower,
  ITDS_highPerformance,
  ITDS_singleConversion
} ITDS_operatingMode_t;

typedef enum
{
  ITDS_lowPower,
  ITDS_normalMode
} ITDS_powerMode_t;

typedef enum
{
  ITDS_off = 0,
  ITDS_positiveAxis = 1,
  ITDS_negativeAxis = 2
} ITDS_selfTestConfig_t;

typedef enum
{
  ITDS_pushPull = 0,
  ITDS_openDrain = 1
} ITDS_interruptPinConfig_t;

typedef enum
{
  ITDS_activeHigh = 0,
  ITDS_activeLow = 1
} ITDS_interruptActiveLevel_t;

typedef enum
{
  ITDS_externalTrigger = 0,     /* Triggered by external signal on INT_1 */
  ITDS_registerTrigger = 1      /* Triggered by writing register (SLP_MODE_1 = 1) */
} ITDS_singleDataConversionTrigger_t;

typedef enum
{
  ITDS_outputDataRate_2 = 0,    /* ODR/2 (except for ODR = 1600 Hz, 400 Hz) */
  ITDS_outputDataRate_4 = 1,    /* ODR/4 (High pass / Low pass filter) */
  ITDS_outputDataRate_10 = 2,   /* ODR/10 (High pass / Low pass filter) */
  ITDS_outputDataRate_20 = 3    /* ODR/20 (High pass / Low pass filter) */
} ITDS_bandwidth_t;

typedef enum
{
  ITDS_twoG = 0,      /* ±2g */
  ITDS_fourG = 1,     /* ±4g */
  ITDS_eightG = 2,    /* ±8g */
  ITDS_sixteenG = 3   /* ±16g */
} ITDS_fullScale_t;

typedef enum
{
  ITDS_lowPass = 0,
  ITDS_highPass = 1
} ITDS_filterType_t;

typedef enum
{
  ITDS_bypassMode = 0,
  ITDS_fifoEnabled = 1,
  ITDS_continuousToFifo = 3,
  ITDS_bypassToContinuous = 4,
  ITDS_continuousMode = 6
} ITDS_FifoMode_t;

typedef enum
{
  ITDS_eightyDeg = 0,   /* 6 (80 degrees) */
  ITDS_seventyDeg = 1,  /* 11 (70 degrees) */
  ITDS_sixtyDeg = 2,    /* 16 (60 degrees) */
  ITDS_fiftyDeg = 3     /* 21 (50 degrees) */
} ITDS_thresholdDegree_t;

typedef enum
{
  ITDS_X_Y_Z = 0,
  ITDS_Y_X_Z = 1,
  ITDS_X_Z_Y = 2,
  ITDS_Z_Y_X = 3,
  ITDS_Y_Z_X = 5,
  ITDS_Z_X_Y = 6
} ITDS_tapAxisPriority_t;

typedef enum
{
  ITDS_five = 0,
  ITDS_seven = 1,
  ITDS_eight = 2,
  ITDS_ten = 3,
  ITDS_eleven = 4,
  ITDS_thirteen = 5,
  ITDS_fifteen = 6,
  ITDS_sixteen = 7,
} ITDS_FreeFallThreshold_t;

typedef enum
{
  ITDS_latched = 0,
  ITDS_pulsed = 1
} ITDS_drdyPulse_t;



#ifdef __cplusplus
extern "C"
{
#endif

  /**         Function definitions         */

  int8_t ITDS_initInterface(WE_sensorInterface_t* sensorInterface);
  int8_t ITDS_getInterface(WE_sensorInterface_t* sensorInterface);

  int8_t ITDS_isInterfaceReady();

  int8_t ITDS_getDeviceID(uint8_t *deviceID) ;

  /* CTRL-REG 1 */
  int8_t ITDS_setOutputDataRate(ITDS_outputDataRate_t odr);
  int8_t ITDS_getOutputDataRate(ITDS_outputDataRate_t *odr) ;
  int8_t ITDS_setOperatingMode(ITDS_operatingMode_t opMode);
  int8_t ITDS_getOperatingMode(ITDS_operatingMode_t *opMode);
  int8_t ITDS_setPowerMode(ITDS_powerMode_t powerMode);
  int8_t ITDS_getPowerMode(ITDS_powerMode_t *powerMode);

  /* CTRL-REG 2 */
  int8_t ITDS_reboot(ITDS_state_t reboot);
  int8_t ITDS_isRebooting(ITDS_state_t *rebooting);
  int8_t ITDS_softReset(ITDS_state_t swReset);
  int8_t ITDS_getSoftResetState(ITDS_state_t *swReset);
  int8_t ITDS_setCSPullUpDisconnected(ITDS_state_t disconnectPU);
  int8_t ITDS_isCSPullUpDisconnected(ITDS_state_t *puDisconnected);
  int8_t ITDS_enableBlockDataUpdate(ITDS_state_t bdu);
  int8_t ITDS_isBlockDataUpdateEnabled(ITDS_state_t *bdu);
  int8_t ITDS_enableAutoIncrement(ITDS_state_t autoIncr);
  int8_t ITDS_isAutoIncrementEnabled(ITDS_state_t *autoIncr);
  int8_t ITDS_disableI2CInterface(ITDS_state_t i2cDisable);
  int8_t ITDS_isI2CInterfaceDisabled(ITDS_state_t *i2cDisabled);

  /* CTRL-REG 3 */
  int8_t ITDS_setSelfTestMode(ITDS_selfTestConfig_t selfTest);
  int8_t ITDS_getSelfTestMode(ITDS_selfTestConfig_t *selfTest);
  int8_t ITDS_setInterruptPinType(ITDS_interruptPinConfig_t pinType);
  int8_t ITDS_getInterruptPinType(ITDS_interruptPinConfig_t *pinType);
  int8_t ITDS_enableLatchedInterrupt(ITDS_state_t lir);
  int8_t ITDS_isLatchedInterruptEnabled(ITDS_state_t *lir);
  int8_t ITDS_setInterruptActiveLevel(ITDS_interruptActiveLevel_t level);
  int8_t ITDS_getInterruptActiveLevel(ITDS_interruptActiveLevel_t *level);
  int8_t ITDS_startSingleDataConversion(ITDS_state_t start);
  int8_t ITDS_isSingleDataConversionStarted(ITDS_state_t *start);
  int8_t ITDS_setSingleDataConversionTrigger(ITDS_singleDataConversionTrigger_t conversionTrigger);
  int8_t ITDS_getSingleDataConversionTrigger(ITDS_singleDataConversionTrigger_t *conversionTrigger) ;

  /* CTRL-REG 4 */
  int8_t ITDS_enable6DOnINT0(ITDS_state_t int06D);
  int8_t ITDS_is6DOnINT0Enabled(ITDS_state_t *int06D);
  int8_t ITDS_enableSingleTapINT0(ITDS_state_t int0SingleTap);
  int8_t ITDS_isSingleTapINT0Enabled(ITDS_state_t *int0SingleTap);
  int8_t ITDS_enableWakeUpOnINT0(ITDS_state_t int0WakeUp);
  int8_t ITDS_isWakeUpOnINT0Enabled(ITDS_state_t *int0WakeUp);
  int8_t ITDS_enableFreeFallINT0(ITDS_state_t int0FreeFall);
  int8_t ITDS_isFreeFallINT0Enabled(ITDS_state_t *int0FreeFall);
  int8_t ITDS_enableDoubleTapINT0(ITDS_state_t int0DoubleTap);
  int8_t ITDS_isDoubleTapINT0Enabled(ITDS_state_t *int0DoubleTap);
  int8_t ITDS_enableFifoFullINT0(ITDS_state_t int0FifoFull);
  int8_t ITDS_isFifoFullINT0Enabled(ITDS_state_t *int0FifoFull);
  int8_t ITDS_enableFifoThresholdINT0(ITDS_state_t int0FifoThreshold);
  int8_t ITDS_isFifoThresholdINT0Enabled(ITDS_state_t *int0FifoThreshold);
  int8_t ITDS_enableDataReadyINT0(ITDS_state_t int0DataReady);
  int8_t ITDS_isDataReadyINT0Enabled(ITDS_state_t *int0DataReady);

  /* CTRL-REG 5 */
  int8_t ITDS_enableSleepStatusINT1(ITDS_state_t int1SleepStatus);
  int8_t ITDS_isSleepStatusINT1Enabled(ITDS_state_t *int1SleepStatus);
  int8_t ITDS_enableSleepStatusChangeINT1(ITDS_state_t int1SleepChange);
  int8_t ITDS_isSleepStatusChangeINT1Enabled(ITDS_state_t *int1SleepChange);
  int8_t ITDS_enableBootStatusINT1(ITDS_state_t int1Boot);
  int8_t ITDS_isBootStatusINT1Enabled(ITDS_state_t *int1Boot);
  int8_t ITDS_enableTempDataReadyINT1(ITDS_state_t int1TempDataReady);
  int8_t ITDS_isTempDataReadyINT1Enabled(ITDS_state_t *int1TempDataReady);
  int8_t ITDS_enableFifoOverrunIntINT1(ITDS_state_t int1FifoOverrun);
  int8_t ITDS_isFifoOverrunIntINT1Enabled(ITDS_state_t *int1FifoOverrun);
  int8_t ITDS_enableFifoFullINT1(ITDS_state_t int1FifoFull);
  int8_t ITDS_isFifoFullINT1Enabled(ITDS_state_t *int1FifoFull);
  int8_t ITDS_enableFifoThresholdINT1(ITDS_state_t int1FifoThresholdInt);
  int8_t ITDS_isFifoThresholdINT1Enabled(ITDS_state_t *int1FifoThresholdInt);
  int8_t ITDS_enableDataReadyINT1(ITDS_state_t int1DataReadyInt);
  int8_t ITDS_isDataReadyINT1Enabled(ITDS_state_t *int1DataReadyInt);

  /* CTRL-REG 6 */
  int8_t ITDS_setFilteringCutoff(ITDS_bandwidth_t filteringCutoff);
  int8_t ITDS_getFilteringCutoff(ITDS_bandwidth_t *filteringCutoff);
  int8_t ITDS_setFullScale(ITDS_fullScale_t fullScale);
  int8_t ITDS_getFullScale(ITDS_fullScale_t *fullScale);
  int8_t ITDS_setFilterPath(ITDS_filterType_t filterType);
  int8_t ITDS_getFilterPath(ITDS_filterType_t *filterType);
  int8_t ITDS_enableLowNoise(ITDS_state_t lowNoise);
  int8_t ITDS_isLowNoiseEnabled(ITDS_state_t *lowNoise);

  /* Status */
  /* Note: The status register is partially duplicated to the STATUS_DETECT register. */
  int8_t ITDS_getStatusRegister(ITDS_status_t *status);
  int8_t ITDS_isAccelerationDataReady(ITDS_state_t *dataReady);
  int8_t ITDS_getSingleTapState(ITDS_state_t *singleTap);
  int8_t ITDS_getDoubleTapState(ITDS_state_t *doubleTap);
  int8_t ITDS_getSleepState(ITDS_state_t *sleepState);

  /* Acceleration output */
  int8_t ITDS_getRawAccelerationX(int16_t *xRawAcc);
  int8_t ITDS_getRawAccelerationY(int16_t *yRawAcc) ;
  int8_t ITDS_getRawAccelerationZ(int16_t *zRawAcc);
  int8_t ITDS_getRawAccelerations(uint8_t numSamples,
                                  int16_t *xRawAcc,
                                  int16_t *yRawAcc,
                                  int16_t *zRawAcc);

  /* Temperature output */
  int8_t ITDS_getTemperature8bit(uint8_t *temp8bit);
  int8_t ITDS_getRawTemperature12bit(int16_t *temp12bit);
#ifdef WE_USE_FLOAT
  int8_t ITDS_getTemperature12bit(float *tempDegC);
#else
  #warning "WSEN_ITDS sensor driver: Float support is turned off by default. Define WE_USE_FLOAT to enable float support."
#endif /* WE_USE_FLOAT */

  /* FIFO CTRL */
  int8_t ITDS_setFifoMode(ITDS_FifoMode_t fifoMode);
  int8_t ITDS_getFifoMode(ITDS_FifoMode_t *fifoMode);
  int8_t ITDS_setFifoThreshold(uint8_t fifoThreshold);
  int8_t ITDS_getFifoThreshold(uint8_t *fifoThreshold);

  /* FIFO_SAMPLES */
  int8_t ITDS_getFifoSamplesRegister(ITDS_fifoSamples_t *fifoSamplesStatus);
  int8_t ITDS_isFifoThresholdReached(ITDS_state_t *fifoThr);
  int8_t ITDS_getFifoOverrunState(ITDS_state_t *fifoOverrun);
  int8_t ITDS_getFifoFillLevel(uint8_t *fifoFill);

  /* TAP_X_TH */
  int8_t ITDS_enable4DDetection(ITDS_state_t detection4D);
  int8_t ITDS_is4DDetectionEnabled(ITDS_state_t *detection4D);
  int8_t ITDS_setTapThresholdX(uint8_t tapThresholdX);
  int8_t ITDS_getTapThresholdX(uint8_t *tapThresholdX);
  int8_t ITDS_set6DThreshold(ITDS_thresholdDegree_t threshold6D);
  int8_t ITDS_get6DThreshold(ITDS_thresholdDegree_t *threshold6D);

  /* TAP_Y_TH */
  int8_t ITDS_setTapThresholdY(uint8_t tapThresholdY);
  int8_t ITDS_getTapThresholdY(uint8_t *tapThresholdY);
  int8_t ITDS_setTapAxisPriority(ITDS_tapAxisPriority_t priority);
  int8_t ITDS_getTapAxisPriority(ITDS_tapAxisPriority_t *priority);

  /* TAP_Z_TH */
  int8_t ITDS_setTapThresholdZ(uint8_t tapThresholdZ);
  int8_t ITDS_getTapThresholdZ(uint8_t *tapThresholdZ);
  int8_t ITDS_enableTapX(ITDS_state_t tapX);
  int8_t ITDS_isTapXEnabled(ITDS_state_t *tapX);
  int8_t ITDS_enableTapY(ITDS_state_t tapY);
  int8_t ITDS_isTapYEnabled(ITDS_state_t *tapY);
  int8_t ITDS_enableTapZ(ITDS_state_t tapZ);
  int8_t ITDS_isTapZEnabled(ITDS_state_t *tapZ);

  /* INT_DUR */
  int8_t ITDS_setTapLatencyTime(uint8_t latencyTime);
  int8_t ITDS_getTapLatencyTime(uint8_t *latencyTime);
  int8_t ITDS_setTapQuietTime(uint8_t quietTime);
  int8_t ITDS_getTapQuietTime(uint8_t *quietTime);
  int8_t ITDS_setTapShockTime(uint8_t shockTime);
  int8_t ITDS_getTapShockTime(uint8_t *shockTime);

  /* WAKE_UP_TH */
  int8_t ITDS_enableDoubleTapEvent(ITDS_state_t doubleTap);
  int8_t ITDS_isDoubleTapEventEnabled(ITDS_state_t *doubleTap);
  int8_t ITDS_enableInactivityDetection(ITDS_state_t inactivity);
  int8_t ITDS_isInactivityDetectionEnabled(ITDS_state_t *inactivity) ;
  int8_t ITDS_setWakeUpThreshold(uint8_t wakeUpThresh);
  int8_t ITDS_getWakeUpThreshold(uint8_t *wakeUpThresh);

  /* WAKE_UP_DUR */
  int8_t ITDS_setFreeFallDurationMSB(ITDS_state_t freeFallDurationMsb);
  int8_t ITDS_getFreeFallDurationMSB(ITDS_state_t *freeFallDurationMsb);
  int8_t ITDS_setWakeUpDuration(uint8_t duration);
  int8_t ITDS_getWakeUpDuration(uint8_t *duration);
  int8_t ITDS_enableStationaryDetection(ITDS_state_t stationary);
  int8_t ITDS_isStationaryDetectionEnabled(ITDS_state_t *stationary);
  int8_t ITDS_setSleepDuration(uint8_t duration);
  int8_t ITDS_getSleepDuration(uint8_t *duration);

  /* FREE_FALL */
  int8_t ITDS_setFreeFallDuration(uint8_t freeFallDuration);
  int8_t ITDS_getFreeFallDuration(uint8_t *freeFallDuration);
  int8_t ITDS_setFreeFallDurationLSB(uint8_t freeFallDurationLsb);
  int8_t ITDS_getFreeFallDurationLSB(uint8_t *freeFallDurationLsb);
  int8_t ITDS_setFreeFallThreshold(ITDS_FreeFallThreshold_t threshold);
  int8_t ITDS_getFreeFallThreshold(ITDS_FreeFallThreshold_t *threshold);

  /* STATUS_DETECT */
  /* Note: Most of the status bits are already covered by the STATUS_REG register. */
  int8_t ITDS_getStatusDetectRegister(ITDS_statusDetect_t *statusDetect);
  int8_t ITDS_isTemperatureDataReady(ITDS_state_t *dataReady);

  /* WAKE_UP_EVENT */
  int8_t ITDS_getWakeUpEventRegister(ITDS_wakeUpEvent_t *status);
  int8_t ITDS_isWakeUpXEvent(ITDS_state_t *wakeUpX);
  int8_t ITDS_isWakeUpYEvent(ITDS_state_t *wakeUpY);
  int8_t ITDS_isWakeUpZEvent(ITDS_state_t *wakeUpZ);
  int8_t ITDS_isWakeUpEvent(ITDS_state_t *wakeUpState);
  int8_t ITDS_isFreeFallEvent(ITDS_state_t *freeFall);

  /* TAP_EVENT */
  int8_t ITDS_getTapEventRegister(ITDS_tapEvent_t *status);
  int8_t ITDS_isTapEvent(ITDS_state_t *tapEventState);
  int8_t ITDS_getTapSign(ITDS_tapSign_t *tapSign);
  int8_t ITDS_isTapEventXAxis(ITDS_state_t *tapXAxis);
  int8_t ITDS_isTapEventYAxis(ITDS_state_t *tapYAxis);
  int8_t ITDS_isTapEventZAxis(ITDS_state_t *tapZAxis);


  /* 6D_EVENT */
  int8_t ITDS_get6dEventRegister(ITDS_6dEvent_t *status);
  int8_t ITDS_has6dOrientationChanged(ITDS_state_t *orientationChanged);
  int8_t ITDS_isXLOverThreshold(ITDS_state_t *xlOverThreshold);
  int8_t ITDS_isXHOverThreshold(ITDS_state_t *xhOverThreshold);
  int8_t ITDS_isYLOverThreshold(ITDS_state_t *ylOverThreshold);
  int8_t ITDS_isYHOverThreshold(ITDS_state_t *yhOverThreshold);
  int8_t ITDS_isZLOverThreshold(ITDS_state_t *zlOverThreshold);
  int8_t ITDS_isZHOverThreshold(ITDS_state_t *zhOverThreshold);

  /* ALL_INT_EVENT */
  int8_t ITDS_getAllInterruptEvents(ITDS_allInterruptEvents_t *events);
  int8_t ITDS_isSleepChangeEvent(ITDS_state_t *sleep);

  /* X_Y_Z_OFS_USR */
  int8_t ITDS_setOffsetValueX(uint8_t offsetvalueXAxis);
  int8_t ITDS_getOffsetValueX(uint8_t *offsetvalueXAxis);
  int8_t ITDS_setOffsetValueY(uint8_t offsetvalueYAxis);
  int8_t ITDS_getOffsetValueY(uint8_t *offsetvalueYAxis);
  int8_t ITDS_setOffsetValueZ(uint8_t offsetvalueZAxis);
  int8_t ITDS_getOffsetValueZ(uint8_t *offsetvalueZAxis);

  /* CTRL_7 */
  int8_t ITDS_setDataReadyPulsed(ITDS_drdyPulse_t drdyPulsed);
  int8_t ITDS_isDataReadyPulsed(ITDS_drdyPulse_t *drdyPulsed);
  int8_t ITDS_setInt1OnInt0(ITDS_state_t int1OnInt0);
  int8_t ITDS_getInt1OnInt0(ITDS_state_t *int1OnInt0) ;
  int8_t ITDS_enableInterrupts(ITDS_state_t interrupts);
  int8_t ITDS_areInterruptsEnabled(ITDS_state_t *interrupts);
  int8_t ITDS_enableApplyOffset(ITDS_state_t applyOffset);
  int8_t ITDS_isApplyOffsetEnabled(ITDS_state_t *applyOffset);
  int8_t ITDS_enableApplyWakeUpOffset(ITDS_state_t applyOffset);
  int8_t ITDS_isApplyWakeUpOffsetEnabled(ITDS_state_t *applyOffset);

  int8_t ITDS_setOffsetWeight(ITDS_state_t offsetWeight);
  int8_t ITDS_getOffsetWeight(ITDS_state_t *offsetWeight);

  int8_t ITDS_enableHighPassRefMode(ITDS_state_t refMode);
  int8_t ITDS_isHighPassRefModeEnabled(ITDS_state_t *refMode);

  int8_t ITDS_enableLowPassOn6D(ITDS_state_t lowPassOn6D);
  int8_t ITDS_isLowPassOn6DEnabled(ITDS_state_t *lowPassOn6D);

#ifdef __cplusplus
}
#endif

#endif /* _WSEN_ITDS_H */

/**         EOF         */
