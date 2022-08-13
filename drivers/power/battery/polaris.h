/****************************************************************************
 * drivers/power/battery/polaris.h
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

#ifndef __POLARIS_H
#define __POLARIS_H

#define NVM_SECTOR_SIZE_BYTES           256 //128
#define NVM_PATCH_START_SECTOR_INDEX    0
#define NVM_CFG_START_SECTOR_INDEX      126 //93
#define I2C_CHUNK_SIZE                  256 //128

#define AFTER_SYS_RESET_SLEEP_MS        50

/* FW register address */
#define FWREG_OP_MODE_ADDR              0x000E
#define FWREG_SYS_CMD_ADDR              0x0020
#define FWREG_NVM_SECTOR_INDEX_ADDR     0x0024
#define FWREG_AUX_DATA_00               0x0180
#define FWREG_NVM_PWD_ADDR              0x0022

/* SYSREG registers */
#define HWREG_HW_VER_ADDR               0x2001C002
#define HWREG_HW_SYS_RST_ADDR           0x2001F200

typedef enum
{
  FW_OP_MODE_SA = 1,
  FW_OP_MODE_RX = 2,
  FW_OP_MODE_TX = 3
}fw_op_mode_t;

#define WRITE_READ_OPERATION            0x01
#define WRITE_OPERATION                 0x02

#define OPCODE_WRITE                    0xFA

#define MIN_WR_BYTE_LENGTH              5
#define MIN_W_BYTE_LENGTH               4

#define CMD_STR_LEN                     1024

#define OK                              ((int)0x00000000)
#define E_BUS_R                         ((int)0x80000001)
#define E_BUS_W                         ((int)0x80000002)
#define E_BUS_WR                        ((int)0x80000003)
#define E_UNEXPECTED_OP_MODE            ((int)0x80000004)
#define E_NVM_WRITE                     ((int)0x80000005)
#define E_INVALID_INPUT                 ((int)0x80000006)
#define E_MEMORY_ALLOC                  ((int)0x80000007)
#define E_UNEXPECTED_HW_REV             ((int)0x80000008)
#define E_TIMEOUT                       ((int)0x80000009)
#define E_NVM_DATA_MISMATCH             ((int)0x8000000A)

#define WLC_CHIPID_LOW_REG              0x0000

#define WLC_DRV_VERSION                 "1.0.0" /* driver version string format */

struct polaris_chip_info
{
  uint16_t chip_id;
  uint8_t chip_revision;
  uint8_t customer_id;
  uint16_t project_id;
  uint16_t nvm_patch_id;
  uint16_t ram_patch_id;
  uint16_t config_id;
  uint16_t pe_id;
  uint8_t cut_id;
  uint16_t config_size;
  uint16_t patch_size;
};

struct rx_int_state_s
{
  bool wlc_rx_int_otp;
  bool wlc_rx_int_ocp;
  bool wlc_rx_int_ovp;
  bool wlc_rx_int_scp;
  bool wlc_rx_int_ss_tx;
  bool wlc_rx_int_output_on;
  bool wlc_rx_int_output_off;
  bool wlc_rx_int_uvp;
  bool wlc_rx_pp_done;
};

enum rx_sleep_state_e
{
  RX_SLEEP_NOT = 0,
  RX_SLEEP_ENTER,
  RX_SLEEP_QUIT,
  RX_SLEEP_MAX,
};

#define WLC_HEALTH_TEMP_MAX  80
#define WLC_HEALTH_TEMP_MIN  10

#define WLC_HEALTH_UNKNOWN   -1
#define WLC_HEALTH_GOOD      0
#define WLC_HEALTH_OVERHEAT  1
#define WLC_HEALTH_OVERCOLD  2

#define RX_DETECT_WORK_TIME               1000000
#define RX_VOUT_RESTART_DETECT_WORK_TIME  2200000
#define RX_VOUT_RESTART_INT_WORK_TIME     4000000

#define DETECT_WORK_NO_EXIST       0
#define DETECT_WORK_EXIST          1
#define BATT_CHARGING_STAT_INIT   -1
#define BATT_CHARGING_STAT_ENTER   1

#define WLC_SUPPORT_QC_3_0        0x1a
#define WLC_GEN_TX                0x11
#define WLC_HANDSHAKE_OK          0x01

#define ST_IIC_RETRY_NUM          3

int get_fw_head_info(struct polaris_chip_info *head_info);
int get_fw_data(uint8_t *cfg_data, uint8_t *patch_data);

#endif /* __POLARIS_H */

