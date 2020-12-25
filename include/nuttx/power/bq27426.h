/****************************************************************************
 * include/nuttx/power/bq27426.h
 * Lower half driver for bq27426 battery fuel gauge.
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __DRIVERS_POWER_BQ27426_H
#define __DRIVERS_POWER_BQ27426_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Auxiliary Definitions */

#define BQ27426_I2C_TIMEOUT             2000

#define BQ27426_I2C_ADDRESS             0x55 /* Default I2C address of the BQ27426 */

/* General Constants */

#define BQ27426_UNSEAL_KEY              0x8000 /* Secret code to unseal the BQ27426 */
#define BQ27426_DEVICE_ID               0x0426 /* Default device ID */

/* Standard Commands */

/* The fuel gauge uses a series of 2-byte standard commands to enable system
 * reading and writing of battery information. Each command has an associated
 * sequential command-code pair.
 */

#define BQ27426_COMMAND_CONTROL         0x00 /* Control() */
#define BQ27426_COMMAND_TEMP            0x02 /* Temperature() */
#define BQ27426_COMMAND_VOLTAGE         0x04 /* Voltage() */
#define BQ27426_COMMAND_FLAGS           0x06 /* Flags() */
#define BQ27426_COMMAND_NOM_CAPACITY    0x08 /* NominalAvailableCapacity() */
#define BQ27426_COMMAND_AVAIL_CAPACITY  0x0A /* FullAvailableCapacity() */
#define BQ27426_COMMAND_REM_CAPACITY    0x0C /* RemainingCapacity() */
#define BQ27426_COMMAND_FULL_CAPACITY   0x0E /* FullChargeCapacity() */
#define BQ27426_COMMAND_AVG_CURRENT     0x10 /* AverageCurrent() */
#define BQ27426_COMMAND_AVG_POWER       0x18 /* AveragePower() */
#define BQ27426_COMMAND_SOC             0x1C /* StateOfCharge() */
#define BQ27426_COMMAND_INT_TEMP        0x1E /* InternalTemperature() */
#define BQ27426_COMMAND_SOH             0x20 /* StateOfHealth() */
#define BQ27426_COMMAND_REM_CAP_UNFL    0x28 /* RemainingCapacityUnfiltered() */
#define BQ27426_COMMAND_REM_CAP_FIL     0x2A /* RemainingCapacityFiltered() */
#define BQ27426_COMMAND_FULL_CAP_UNFL   0x2C /* FullChargeCapacityUnfiltered() */
#define BQ27426_COMMAND_FULL_CAP_FIL    0x2E /* FullChargeCapacityFiltered() */
#define BQ27426_COMMAND_SOC_UNFL        0x30 /* StateOfChargeUnfiltered() */

/* Control Sub-commands */

/* Issuing a Control() command requires a subsequent 2-byte subcommand. These
 * additional bytes specify the particular control function desired. The
 * Control() command allows the system to control specific features of the
 * fuel gauge during normal operation and additional features when the
 * device is in different access modes.
 */

#define BQ27426_CONTROL_STATUS          0x0000
#define BQ27426_CONTROL_DEVICE_TYPE     0x0001
#define BQ27426_CONTROL_FW_VERSION      0x0002
#define BQ27426_CONTROL_DM_CODE         0x0004
#define BQ27426_CONTROL_PREV_MACWRITE   0x0007
#define BQ27426_CONTROL_CHEM_ID         0x0008
#define BQ27426_CONTROL_BAT_INSERT      0x000C
#define BQ27426_CONTROL_BAT_REMOVE      0x000D
#define BQ27426_CONTROL_SET_CFGUPDATE   0x0013
#define BQ27426_CONTROL_SMOOTH_SYNC     0x0019
#define BQ27426_CONTROL_SHUTDOWN_ENABLE 0x001B
#define BQ27426_CONTROL_SHUTDOWN        0x001C
#define BQ27426_CONTROL_SEALED          0x0020
#define BQ27426_CONTROL_PULSE_SOC_INT   0x0023
#define BQ27426_CONTROL_CHEM_A          0x0030
#define BQ27426_CONTROL_CHEM_B          0x0031
#define BQ27426_CONTROL_CHEM_C          0x0032
#define BQ27426_CONTROL_RESET           0x0041
#define BQ27426_CONTROL_SOFT_RESET      0x0042

/* Control Status Word - Bit Definitions */

/* Bit positions for the 16-bit data of CONTROL_STATUS.
 * CONTROL_STATUS instructs the fuel gauge to return status
 * information to Control() addresses 0x00 and 0x01.
 * The read-only status word contains status  bits that are
 * set or cleared either automatically as conditions warrant or
 * through using specified subcommands.
 */

#define BQ27426_STATUS_SHUTDOWNEN      (1 << 15)
#define BQ27426_STATUS_WDRESET         (1 << 14)
#define BQ27426_STATUS_SS              (1 << 13)
#define BQ27426_STATUS_CALMODE         (1 << 12)
#define BQ27426_STATUS_CCA             (1 << 11)
#define BQ27426_STATUS_BCA             (1 << 10)
#define BQ27426_STATUS_QMAX_UP         (1 << 9)
#define BQ27426_STATUS_RES_UP          (1 << 8)
#define BQ27426_STATUS_INITCOMP        (1 << 7)
#define BQ27426_STATUS_SLEEP           (1 << 4)
#define BQ27426_STATUS_LDMD            (1 << 3)
#define BQ27426_STATUS_RUP_DIS         (1 << 2)
#define BQ27426_STATUS_VOK             (1 << 1)
#define BQ27426_STATUS_CHEMCHANGE      (1 << 0)

/* Flag Command - Bit Definitions */

/* Bit positions for the 16-bit data of Flags()
 * This read-word function returns the contents of the fuel gauging status
 * register, depicting the current operating status.
 */

#define BQ27426_FLAG_OT                (1 << 15)
#define BQ27426_FLAG_UT                (1 << 14)
#define BQ27426_FLAG_FC                (1 << 9)
#define BQ27426_FLAG_CHG               (1 << 8)
#define BQ27426_FLAG_OCVTAKEN          (1 << 7)
#define BQ27426_FLAG_DODCORRECT        (1 << 6)
#define BQ27426_FLAG_ITPOR             (1 << 5)
#define BQ27426_FLAG_CFGUPMODE         (1 << 4)
#define BQ27426_FLAG_BAT_DET           (1 << 3)
#define BQ27426_FLAG_SOC1              (1 << 2)
#define BQ27426_FLAG_SOCF              (1 << 1)
#define BQ27426_FLAG_DSG               (1 << 0)

/* Extended Data Commands */

/* Extended data commands offer additional functionality beyond
 * the standard set of commands.
 * They are used in the same manner; however, unlike standard
 * scommands, extended commands are not limited to 2-byte words.
 */

#define BQ27426_EXTENDED_OPCONFIG      0x3A /* OpConfig()*/
#define BQ27426_EXTENDED_CAPACITY      0x3C /* DesignCapacity()*/
#define BQ27426_EXTENDED_DATACLASS     0x3E /* DataClass()*/
#define BQ27426_EXTENDED_DATABLOCK     0x3F /* DataBlock()*/
#define BQ27426_EXTENDED_BLOCKDATA     0x40 /* BlockData()*/
#define BQ27426_EXTENDED_CHECKSUM      0x60 /* BlockDataCheckSum()*/
#define BQ27426_EXTENDED_CONTROL       0x61 /* BlockDataControl()*/

/* Configuration Class, Subclass ID's */

/*  To access a subclass of the extended data, set the DataClass()
 *  function with one of these values.
 */

/* Configuration Classes */

#define BQ27426_ID_SAFETY               2  /* Safety */
#define BQ27426_ID_CHG_TERMINATION      6  /* Charge Termination */
#define BQ27426_ID_CONFIG_DATA         48  /* Data */
#define BQ27426_ID_DISCHARGE           49  /* Discharge */
#define BQ27426_ID_REGISTERS           64  /* Registers */
#define BQ27426_ID_POWER               68  /* Power */

/* Gas Gauging Classes */

#define BQ27426_ID_IT_CFG              80  /* IT Cfg */
#define BQ27426_ID_CURRENT_THRESH      81  /* Current Thresholds */
#define BQ27426_ID_STATE               82  /* State */

/* Ra Tables Classes */

#define BQ27426_ID_R_A_RAM             89  /* R_a RAM */

/* Calibration Classes */

#define BQ27426_ID_CALIB_DATA          104 /* Data */
#define BQ27426_ID_CC_CAL              105 /* CC Cal */
#define BQ27426_ID_CURRENT             107 /* Current */

/* Security Classes */

#define BQ27426_ID_CODES               112 /* Codes */

/* OpConfig Register - Bit Definitions */

/* Bit positions of the OpConfig Register */

#define BQ27426_OPCONFIG_BIE           (1 << 13)
#define BQ27426_OPCONFIG_GPIOPOL       (1 << 11)
#define BQ27426_OPCONFIG_RESFACTSTEP   (1 << 6)
#define BQ27426_OPCONFIG_SLEEP         (1 << 5)
#define BQ27426_OPCONFIG_RMFCC         (1 << 4)
#define BQ27426_OPCONFIG_FASTCONVEN    (1 << 3)
#define BQ27426_OPCONFIG_BATLOWEN      (1 << 2)
#define BQ27426_OPCONFIG_TEMPS         (1 << 0)

#define  BQ27426_ACCESS_SUB_CLASS_80   0x50
#define  BQ27426_ACCESS_SUB_CLASS_81   0x51
#define  BQ27426_ACCESS_SUB_CLASS_82   0x52
#define  BQ27426_ACCESS_SUB_CLASS_89   0x59
#define  BQ27426_ACCESS_SUB_CLASS_64   0x40

#endif /* __DRIVERS_POWER_BQ27426_H */
