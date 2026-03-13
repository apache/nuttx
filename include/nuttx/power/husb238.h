/****************************************************************************
 * include/nuttx/power/husb238.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_POWER_HUSB238_H
#define __INCLUDE_NUTTX_POWER_HUSB238_H

#if defined(CONFIG_I2C) && defined(CONFIG_USBPD_HUSB238)

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Values for the PD_STATUS0 register ***************************************/

/* The voltage information when an explicit contract is established */

#define HUSB238_PD_SRC_VOLTAGE_UNATTACHED  0b0000  /* Unattached */
#define HUSB238_PD_SRC_VOLTAGE_5V          0b0001  /* PD 5V      */
#define HUSB238_PD_SRC_VOLTAGE_9V          0b0010  /* PD 9V      */
#define HUSB238_PD_SRC_VOLTAGE_12V         0b0011  /* PD 12V     */
#define HUSB238_PD_SRC_VOLTAGE_15V         0b0100  /* PD 15V     */
#define HUSB238_PD_SRC_VOLTAGE_18V         0b0101  /* PD 18V     */
#define HUSB238_PD_SRC_VOLTAGE_20V         0b0110  /* PD 20V     */

/* The current information when an explicit contract is established */

/* (See HUSB238_CURRENT_* macros) */

/* Values for the PD_STATUS1 register ***************************************/

#define HUSB238_CC_DIR_CC1_CONNECTED         0      /* CC1 is connected to CC line or unattached mode */
#define HUSB238_CC_DIR_CC2_CONNECTED         1      /* CC2 is connected to CC line */
#define HUSB238_ATTACH_UNATTACHED            0      /* CC1 is connected to CC line or unattached mode */
#define HUSB238_ATTACH_ATTACHED              1      /* HUSB238 is in modes other than unattached mode */
#define HUSB238_PD_RESPONSE_NONE             0b000  /* No response */
#define HUSB238_PD_RESPONSE_SUCCESS          0b001  /* Success */
#define HUSB238_PD_RESPONSE_INVALID_CMD      0b011  /* Invalid command or argument */
#define HUSB238_PD_RESPONSE_UNSUPPORTED      0b100  /* Command not supported */
#define HUSB238_PD_RESPONSE_FAIL             0b101  /* Transaction fail (no GoodCRC is received after sending) */

/* Voltage information of 5V contract */

#define HUSB238_5V_CONTRACT_VOLTAGE_OTHERS   0      /* Others */
#define HUSB238_5V_CONTRACT_VOLTAGE_5V       1      /* 5V     */

/* Current information of 5V contract */

#define HUSB238_5V_CONTRACT_CURRENT_DEFAULT  0b00   /* Default current */
#define HUSB238_5V_CONTRACT_CURRENT_1_5A     0b01   /* 1.5A            */
#define HUSB238_5V_CONTRACT_CURRENT_2_4A     0b10   /* 2.4A            */
#define HUSB238_5V_CONTRACT_CURRENT_3A       0b11   /* 3A              */

/* Values for the SRC_PDO_* register ****************************************/

#define HUSB238_VOLTAGE_NOT_DETECTED  0       /* Not detected */
#define HUSB238_VOLTAGE_DETECTED      1       /* Detected     */

/* Common current values */

#define HUSB238_CURRENT_0_5A          0b0000  /* 0.50A */
#define HUSB238_CURRENT_0_7A          0b0001  /* 0.70A */
#define HUSB238_CURRENT_1A            0b0010  /* 1.00A */
#define HUSB238_CURRENT_1_25A         0b0011  /* 1.25A */
#define HUSB238_CURRENT_1_5A          0b0100  /* 1.50A */
#define HUSB238_CURRENT_1_75A         0b0101  /* 1.75A */
#define HUSB238_CURRENT_2A            0b0110  /* 2.00A */
#define HUSB238_CURRENT_2_25A         0b0111  /* 2.25A */
#define HUSB238_CURRENT_2_5A          0b1000  /* 2.50A */
#define HUSB238_CURRENT_2_75A         0b1001  /* 2.75A */
#define HUSB238_CURRENT_3A            0b1010  /* 3.00A */
#define HUSB238_CURRENT_3_25A         0b1011  /* 3.25A */
#define HUSB238_CURRENT_3_5A          0b1100  /* 3.50A */
#define HUSB238_CURRENT_4A            0b1101  /* 4.00A */
#define HUSB238_CURRENT_4_5A          0b1110  /* 4.50A */
#define HUSB238_CURRENT_5A            0b1111  /* 5.00A */

/* Values for the SRC_PDO register ******************************************/

#define HUSB238_SRC_PDO_NONE  0b0000  /* Not selected */
#define HUSB238_SRC_PDO_5V    0b0001  /* SRC_PDO_5V   */
#define HUSB238_SRC_PDO_9V    0b0010  /* SRC_PDO_9V   */
#define HUSB238_SRC_PDO_12V   0b0011  /* SRC_PDO_12V  */
#define HUSB238_SRC_PDO_15V   0b1000  /* SRC_PDO_15V  */
#define HUSB238_SRC_PDO_18V   0b1001  /* SRC_PDO_18V  */
#define HUSB238_SRC_PDO_20V   0b1010  /* SRC_PDO_20V  */

/* Values for the GO_CIMMAND register ***************************************/

#define HUSB238_CMD_REQUEST_PDO     0b00001  /* Requests the PDO set by PDO_SELECT */
#define HUSB238_CMD_SEND_GETSRCCAP  0b00100  /* Send out Get_SRC_Cap command       */
#define HUSB238_CMD_SEND_HARDRESET  0b10000  /* Send out hard reset command        */

/* IOCTL Commands ***********************************************************/

/* Cmd: PWRIOC_HUSB238_GET_SELECTED_PDO
 * Arg: uint8_t *pdo_select                    (See HUSB238_SRC_PDO_* macros)
 *
 * Cmd: PWRIOC_HUSB238_SET_SELECTED_PDO
 * Arg: uint8_t *pdo_select                    (See HUSB238_SRC_PDO_* macros)
 *
 * Cmd: PWRIOC_HUSB238_GET_PDSTATUS_0
 * Arg: husb238_pdstatus_0_s *pdstatus_0
 *
 * Cmd: PWRIOC_HUSB238_GET_PDSTATUS_1
 * Arg: husb238_pdstatus_1_s *pdstatus_1
 *
 * Cmd: PWRIOC_HUSB238_GET_SRC_PDOS
 * Arg: husb238_pdos_s *src_pdos
 *
 * Cmd: PWRIOC_HUSB238_SEND_COMMAND
 * Arg: uint8_t *command                       (See HUSB238_CMD_* macros)
 */

#define PWRIOC_HUSB238_GET_PDSTATUS_0    _PWRIOC(PWR_HUSB238_FIRST + 1)
#define PWRIOC_HUSB238_GET_PDSTATUS_1    _PWRIOC(PWR_HUSB238_FIRST + 2)
#define PWRIOC_HUSB238_GET_SRC_PDOS      _PWRIOC(PWR_HUSB238_FIRST + 3)
#define PWRIOC_HUSB238_GET_SELECTED_PDO  _PWRIOC(PWR_HUSB238_FIRST + 4)
#define PWRIOC_HUSB238_SET_SELECTED_PDO  _PWRIOC(PWR_HUSB238_FIRST + 5)
#define PWRIOC_HUSB238_SEND_COMMAND      _PWRIOC(PWR_HUSB238_FIRST + 6)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct husb238_pdstatus_0_s
  {
    uint8_t src_voltage;   /* See HUSB238_PD_SRC_VOLTAGE_* macros */
    uint8_t src_current;   /* See HUSB238_CURRENT_* macros */
  };

struct husb238_pdstatus_1_s
  {
    uint8_t cc_dir;        /* See HUSB238_CC_DIR_CC* macros */
    uint8_t attached;      /* See HUSB238_ATTACH_* macros */
    uint8_t pd_response;   /* See HUSB238_PD_RESPONSE_* macros */
    uint8_t voltage_5v;    /* See HUSB238_5V_CONTRACT_VOLTAGE_* macros */
    uint8_t current_5v;    /* See HUSB238_5V_CONTRACT_CURRENT_* macros */
  };
struct husb238_src_pdos_s
  {
    uint8_t detected_5v;   /* See HUSB238_VOLTAGE_* macros */
    uint8_t current_5v;    /* See HUSB238_CURRENT_* macros */
    uint8_t detected_9v;   /* See HUSB238_VOLTAGE_* macros */
    uint8_t current_9v;    /* See HUSB238_CURRENT_* macros */
    uint8_t detected_12v;  /* See HUSB238_VOLTAGE_* macros */
    uint8_t current_12v;   /* See HUSB238_CURRENT_* macros */
    uint8_t detected_15v;  /* See HUSB238_VOLTAGE_* macros */
    uint8_t current_15v;   /* See HUSB238_CURRENT_* macros */
    uint8_t detected_18v;  /* See HUSB238_VOLTAGE_* macros */
    uint8_t current_18v;   /* See HUSB238_CURRENT_* macros */
    uint8_t detected_20v;  /* See HUSB238_VOLTAGE_* macros */
    uint8_t current_20v;   /* See HUSB238_CURRENT_* macros */
  };

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN  extern "C"
extern "C"
{
#else
#define EXTERN  extern
#endif

/****************************************************************************
 * Name: husb238_register
 *
 * Description:
 *   Register the HUSB238 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/usbpd0"
 *   i2c     - An instance of the I2C interface to use.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int husb238_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_USBPD_HUSB238 */
#endif /* __INCLUDE_NUTTX_POWER_HUSB238_H */
