/****************************************************************************
 * arch/risc-v/src/bl602/hardware/bl602_ef.h
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

#ifndef __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_EF_H
#define __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_EF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl602_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define  BL602_EF_CFG_0_OFFSET               0x000000  /* ef_cfg_0 */
#define  BL602_EF_DBG_PWD_LOW_OFFSET         0x000004  /* ef_dbg_pwd_low */
#define  BL602_EF_DBG_PWD_HIGH_OFFSET        0x000008  /* ef_dbg_pwd_high */
#define  BL602_EF_ANA_TRIM_0_OFFSET          0x00000c  /* ef_ana_trim_0 */
#define  BL602_EF_SW_USAGE_0_OFFSET          0x000010  /* ef_sw_usage_0 */
#define  BL602_EF_WIFI_MAC_LOW_OFFSET        0x000014  /* ef_wifi_mac_low */
#define  BL602_EF_WIFI_MAC_HIGH_OFFSET       0x000018  /* ef_wifi_mac_high */
#define  BL602_EF_KEY_SLOT_0_W0_OFFSET       0x00001c  /* ef_key_slot_0_w0 */
#define  BL602_EF_KEY_SLOT_0_W1_OFFSET       0x000020  /* ef_key_slot_0_w1 */
#define  BL602_EF_KEY_SLOT_0_W2_OFFSET       0x000024  /* ef_key_slot_0_w2 */
#define  BL602_EF_KEY_SLOT_0_W3_OFFSET       0x000028  /* ef_key_slot_0_w3 */
#define  BL602_EF_KEY_SLOT_1_W0_OFFSET       0x00002c  /* ef_key_slot_1_w0 */
#define  BL602_EF_KEY_SLOT_1_W1_OFFSET       0x000030  /* ef_key_slot_1_w1 */
#define  BL602_EF_KEY_SLOT_1_W2_OFFSET       0x000034  /* ef_key_slot_1_w2 */
#define  BL602_EF_KEY_SLOT_1_W3_OFFSET       0x000038  /* ef_key_slot_1_w3 */
#define  BL602_EF_KEY_SLOT_2_W0_OFFSET       0x00003c  /* ef_key_slot_2_w0 */
#define  BL602_EF_KEY_SLOT_2_W1_OFFSET       0x000040  /* ef_key_slot_2_w1 */
#define  BL602_EF_KEY_SLOT_2_W2_OFFSET       0x000044  /* ef_key_slot_2_w2 */
#define  BL602_EF_KEY_SLOT_2_W3_OFFSET       0x000048  /* ef_key_slot_2_w3 */
#define  BL602_EF_KEY_SLOT_3_W0_OFFSET       0x00004c  /* ef_key_slot_3_w0 */
#define  BL602_EF_KEY_SLOT_3_W1_OFFSET       0x000050  /* ef_key_slot_3_w1 */
#define  BL602_EF_KEY_SLOT_3_W2_OFFSET       0x000054  /* ef_key_slot_3_w2 */
#define  BL602_EF_KEY_SLOT_3_W3_OFFSET       0x000058  /* ef_key_slot_3_w3 */
#define  BL602_EF_KEY_SLOT_4_W0_OFFSET       0x00005c  /* ef_key_slot_4_w0 */
#define  BL602_EF_KEY_SLOT_4_W1_OFFSET       0x000060  /* ef_key_slot_4_w1 */
#define  BL602_EF_KEY_SLOT_4_W2_OFFSET       0x000064  /* ef_key_slot_4_w2 */
#define  BL602_EF_KEY_SLOT_4_W3_OFFSET       0x000068  /* ef_key_slot_4_w3 */
#define  BL602_EF_KEY_SLOT_5_W0_OFFSET       0x00006c  /* ef_key_slot_5_w0 */
#define  BL602_EF_KEY_SLOT_5_W1_OFFSET       0x000070  /* ef_key_slot_5_w1 */
#define  BL602_EF_KEY_SLOT_5_W2_OFFSET       0x000074  /* ef_key_slot_5_w2 */
#define  BL602_EF_KEY_SLOT_5_W3_OFFSET       0x000078  /* ef_key_slot_5_w3 */
#define  BL602_EF_DATA_0_LOCK_OFFSET         0x00007c  /* ef_data_0_lock */

#define  BL602_EF_REG_KEY_SLOT_6_W0_OFFSET   0x000080  /* reg_key_slot_6_w0 */
#define  BL602_EF_REG_KEY_SLOT_6_W1_OFFSET   0x000084  /* reg_key_slot_6_w1 */
#define  BL602_EF_REG_KEY_SLOT_6_W2_OFFSET   0x000088  /* reg_key_slot_6_w2 */
#define  BL602_EF_REG_KEY_SLOT_6_W3_OFFSET   0x00008c  /* reg_key_slot_6_w3 */
#define  BL602_EF_REG_KEY_SLOT_7_W0_OFFSET   0x000090  /* reg_key_slot_7_w0 */
#define  BL602_EF_REG_KEY_SLOT_7_W1_OFFSET   0x000094  /* reg_key_slot_7_w1 */
#define  BL602_EF_REG_KEY_SLOT_7_W2_OFFSET   0x000098  /* reg_key_slot_7_w2 */
#define  BL602_EF_REG_KEY_SLOT_7_W3_OFFSET   0x00009c  /* reg_key_slot_7_w3 */
#define  BL602_EF_REG_KEY_SLOT_8_W0_OFFSET   0x0000a0  /* reg_key_slot_8_w0 */
#define  BL602_EF_REG_KEY_SLOT_8_W1_OFFSET   0x0000a4  /* reg_key_slot_8_w1 */
#define  BL602_EF_REG_KEY_SLOT_8_W2_OFFSET   0x0000a8  /* reg_key_slot_8_w2 */
#define  BL602_EF_REG_KEY_SLOT_8_W3_OFFSET   0x0000ac  /* reg_key_slot_8_w3 */
#define  BL602_EF_REG_KEY_SLOT_9_W0_OFFSET   0x0000b0  /* reg_key_slot_9_w0 */
#define  BL602_EF_REG_KEY_SLOT_9_W1_OFFSET   0x0000b4  /* reg_key_slot_9_w1 */
#define  BL602_EF_REG_KEY_SLOT_9_W2_OFFSET   0x0000b8  /* reg_key_slot_9_w2 */
#define  BL602_EF_REG_KEY_SLOT_9_W3_OFFSET   0x0000bc  /* reg_key_slot_9_w3 */
#define  BL602_EF_REG_KEY_SLOT_10_W0_OFFSET  0x0000c0  /* reg_key_slot_10_w0 */
#define  BL602_EF_REG_KEY_SLOT_10_W1_OFFSET  0x0000c4  /* reg_key_slot_10_w1 */
#define  BL602_EF_REG_KEY_SLOT_10_W2_OFFSET  0x0000c8  /* reg_key_slot_10_w2 */
#define  BL602_EF_REG_KEY_SLOT_10_W3_OFFSET  0x0000cc  /* reg_key_slot_10_w3 */
#define  BL602_EF_REG_KEY_SLOT_11_W0_OFFSET  0x0000d0  /* reg_key_slot_11_w0 */
#define  BL602_EF_REG_KEY_SLOT_11_W1_OFFSET  0x0000d4  /* reg_key_slot_11_w1 */
#define  BL602_EF_REG_KEY_SLOT_11_W2_OFFSET  0x0000d8  /* reg_key_slot_11_w2 */
#define  BL602_EF_REG_KEY_SLOT_11_W3_OFFSET  0x0000dc  /* reg_key_slot_11_w3 */
#define  BL602_EF_REG_DATA_1_LOCK_OFFSET     0x0000e0  /* reg_data_1_lock */

#define  BL602_EF_IF_CTRL_0_OFFSET           0x000800  /* ef_if_ctrl_0 */
#define  BL602_EF_IF_CYC_0_OFFSET            0x000804  /* ef_if_cyc_0 */
#define  BL602_EF_IF_CYC_1_OFFSET            0x000808  /* ef_if_cyc_1 */
#define  BL602_EF_IF_0_MANUAL_OFFSET         0x00080c  /* ef_if_0_manual */
#define  BL602_EF_IF_0_STATUS_OFFSET         0x000810  /* ef_if_0_status */
#define  BL602_EF_IF_CFG_0_OFFSET            0x000814  /* ef_if_cfg_0 */
#define  BL602_EF_SW_CFG_0_OFFSET            0x000818  /* ef_sw_cfg_0 */
#define  BL602_EF_RESERVED_OFFSET            0x00081c  /* ef_reserved */
#define  BL602_EF_IF_ANA_TRIM_0_OFFSET       0x000820  /* ef_if_ana_trim_0 */
#define  BL602_EF_IF_SW_USAGE_0_OFFSET       0x000824  /* ef_if_sw_usage_0 */
#define  BL602_EF_CRC_CTRL_0_OFFSET          0x000a00  /* ef_crc_ctrl_0 */
#define  BL602_EF_CRC_CTRL_1_OFFSET          0x000a04  /* ef_crc_ctrl_1 */
#define  BL602_EF_CRC_CTRL_2_OFFSET          0x000a08  /* ef_crc_ctrl_2 */
#define  BL602_EF_CRC_CTRL_3_OFFSET          0x000a0c  /* ef_crc_ctrl_3 */
#define  BL602_EF_CRC_CTRL_4_OFFSET          0x000a10  /* ef_crc_ctrl_4 */
#define  BL602_EF_CRC_CTRL_5_OFFSET          0x000a14  /* ef_crc_ctrl_5 */

/* Register definitions *****************************************************/

#define  BL602_EF_CFG_0          ( BL602_EF_BASE +  BL602_EF_CFG_0_OFFSET)
#define  BL602_EF_DBG_PWD_LOW    ( BL602_EF_BASE +  BL602_EF_DBG_PWD_LOW_OFFSET)
#define  BL602_EF_DBG_PWD_HIGH   ( BL602_EF_BASE +  BL602_EF_DBG_PWD_HIGH_OFFSET)
#define  BL602_EF_ANA_TRIM_0     ( BL602_EF_BASE +  BL602_EF_ANA_TRIM_0_OFFSET)
#define  BL602_EF_SW_USAGE_0     ( BL602_EF_BASE +  BL602_EF_SW_USAGE_0_OFFSET)
#define  BL602_EF_WIFI_MAC_LOW   ( BL602_EF_BASE +  BL602_EF_WIFI_MAC_LOW_OFFSET)
#define  BL602_EF_WIFI_MAC_HIGH  ( BL602_EF_BASE +  BL602_EF_WIFI_MAC_HIGH_OFFSET)
#define  BL602_EF_KEY_SLOT_0_W0  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_0_W0_OFFSET)
#define  BL602_EF_KEY_SLOT_0_W1  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_0_W1_OFFSET)
#define  BL602_EF_KEY_SLOT_0_W2  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_0_W2_OFFSET)
#define  BL602_EF_KEY_SLOT_0_W3  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_0_W3_OFFSET)
#define  BL602_EF_KEY_SLOT_1_W0  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_1_W0_OFFSET)
#define  BL602_EF_KEY_SLOT_1_W1  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_1_W1_OFFSET)
#define  BL602_EF_KEY_SLOT_1_W2  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_1_W2_OFFSET)
#define  BL602_EF_KEY_SLOT_1_W3  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_1_W3_OFFSET)
#define  BL602_EF_KEY_SLOT_2_W0  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_2_W0_OFFSET)
#define  BL602_EF_KEY_SLOT_2_W1  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_2_W1_OFFSET)
#define  BL602_EF_KEY_SLOT_2_W2  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_2_W2_OFFSET)
#define  BL602_EF_KEY_SLOT_2_W3  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_2_W3_OFFSET)
#define  BL602_EF_KEY_SLOT_3_W0  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_3_W0_OFFSET)
#define  BL602_EF_KEY_SLOT_3_W1  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_3_W1_OFFSET)
#define  BL602_EF_KEY_SLOT_3_W2  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_3_W2_OFFSET)
#define  BL602_EF_KEY_SLOT_3_W3  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_3_W3_OFFSET)
#define  BL602_EF_KEY_SLOT_4_W0  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_4_W0_OFFSET)
#define  BL602_EF_KEY_SLOT_4_W1  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_4_W1_OFFSET)
#define  BL602_EF_KEY_SLOT_4_W2  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_4_W2_OFFSET)
#define  BL602_EF_KEY_SLOT_4_W3  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_4_W3_OFFSET)
#define  BL602_EF_KEY_SLOT_5_W0  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_5_W0_OFFSET)
#define  BL602_EF_KEY_SLOT_5_W1  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_5_W1_OFFSET)
#define  BL602_EF_KEY_SLOT_5_W2  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_5_W2_OFFSET)
#define  BL602_EF_KEY_SLOT_5_W3  ( BL602_EF_BASE +  BL602_EF_KEY_SLOT_5_W3_OFFSET)
#define  BL602_EF_DATA_0_LOCK    ( BL602_EF_BASE +  BL602_EF_DATA_0_LOCK_OFFSET)

#define  BL602_EF_REG_KEY_SLOT_6_W0   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_6_W0_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_6_W1   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_6_W1_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_6_W2   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_6_W2_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_6_W3   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_6_W3_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_7_W0   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_7_W0_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_7_W1   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_7_W1_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_7_W2   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_7_W2_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_7_W3   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_7_W3_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_8_W0   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_8_W0_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_8_W1   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_8_W1_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_8_W2   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_8_W2_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_8_W3   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_8_W3_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_9_W0   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_9_W0_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_9_W1   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_9_W1_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_9_W2   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_9_W2_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_9_W3   ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_9_W3_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_10_W0  ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_10_W0_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_10_W1  ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_10_W1_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_10_W2  ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_10_W2_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_10_W3  ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_10_W3_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_11_W0  ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_11_W0_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_11_W1  ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_11_W1_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_11_W2  ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_11_W2_OFFSET)
#define  BL602_EF_REG_KEY_SLOT_11_W3  ( BL602_EF_BASE +  BL602_EF_REG_KEY_SLOT_11_W3_OFFSET)
#define  BL602_EF_REG_DATA_1_LOCK     ( BL602_EF_BASE +  BL602_EF_REG_DATA_1_LOCK_OFFSET)

#define  BL602_EF_IF_CTRL_0      ( BL602_EF_BASE +  BL602_EF_IF_CTRL_0_OFFSET)
#define  BL602_EF_IF_CYC_0       ( BL602_EF_BASE +  BL602_EF_IF_CYC_0_OFFSET)
#define  BL602_EF_IF_CYC_1       ( BL602_EF_BASE +  BL602_EF_IF_CYC_1_OFFSET)
#define  BL602_EF_IF_0_MANUAL    ( BL602_EF_BASE +  BL602_EF_IF_0_MANUAL_OFFSET)
#define  BL602_EF_IF_0_STATUS    ( BL602_EF_BASE +  BL602_EF_IF_0_STATUS_OFFSET)
#define  BL602_EF_IF_CFG_0       ( BL602_EF_BASE +  BL602_EF_IF_CFG_0_OFFSET)
#define  BL602_EF_SW_CFG_0       ( BL602_EF_BASE +  BL602_EF_SW_CFG_0_OFFSET)
#define  BL602_EF_RESERVED       ( BL602_EF_BASE +  BL602_EF_RESERVED_OFFSET)
#define  BL602_EF_IF_ANA_TRIM_0  ( BL602_EF_BASE +  BL602_EF_IF_ANA_TRIM_0_OFFSET)
#define  BL602_EF_IF_SW_USAGE_0  ( BL602_EF_BASE +  BL602_EF_IF_SW_USAGE_0_OFFSET)
#define  BL602_EF_CRC_CTRL_0     ( BL602_EF_BASE +  BL602_EF_CRC_CTRL_0_OFFSET)
#define  BL602_EF_CRC_CTRL_1     ( BL602_EF_BASE +  BL602_EF_CRC_CTRL_1_OFFSET)
#define  BL602_EF_CRC_CTRL_2     ( BL602_EF_BASE +  BL602_EF_CRC_CTRL_2_OFFSET)
#define  BL602_EF_CRC_CTRL_3     ( BL602_EF_BASE +  BL602_EF_CRC_CTRL_3_OFFSET)
#define  BL602_EF_CRC_CTRL_4     ( BL602_EF_BASE +  BL602_EF_CRC_CTRL_4_OFFSET)
#define  BL602_EF_CRC_CTRL_5     ( BL602_EF_BASE +  BL602_EF_CRC_CTRL_5_OFFSET)

/* Register bit definitions *************************************************/

#define EF_CFG_0_EF_DBG_MODE_SHIFT           (28)
#define EF_CFG_0_EF_DBG_MODE_MASK            (0x0f << EF_CFG_0_EF_DBG_MODE_SHIFT)
#define EF_CFG_0_EF_DBG_JTAG_0_DIS_SHIFT     (26)
#define EF_CFG_0_EF_DBG_JTAG_0_DIS_MASK      (0x03 << EF_CFG_0_EF_DBG_JTAG_0_DIS_SHIFT)
#define EF_CFG_0_EF_DBG_JTAG_1_DIS_SHIFT     (24)
#define EF_CFG_0_EF_DBG_JTAG_1_DIS_MASK      (0x03 << EF_CFG_0_EF_DBG_JTAG_1_DIS_SHIFT)
#define EF_CFG_0_EF_EFUSE_DBG_DIS            (1 << 23)
#define EF_CFG_0_EF_SE_DBG_DIS               (1 << 22)
#define EF_CFG_0_EF_CPU_RST_DBG_DIS          (1 << 21)
#define EF_CFG_0_EF_CPU1_DIS                 (1 << 20)
#define EF_CFG_0_EF_SF_DIS                   (1 << 19)
#define EF_CFG_0_EF_CAM_DIS                  (1 << 18)
#define EF_CFG_0_EF_0_KEY_ENC_EN             (1 << 17)
#define EF_CFG_0_EF_WIFI_DIS                 (1 << 16)
#define EF_CFG_0_EF_BLE_DIS                  (1 << 15)
#define EF_CFG_0_EF_SDU_DIS                  (1 << 14)
#define EF_CFG_0_EF_SW_USAGE_1_SHIFT         (12)
#define EF_CFG_0_EF_SW_USAGE_1_MASK          (0x03 << EF_CFG_0_EF_SW_USAGE_1_SHIFT)
#define EF_CFG_0_EF_BOOT_SEL_SHIFT           (8)
#define EF_CFG_0_EF_BOOT_SEL_MASK            (0x0f << EF_CFG_0_EF_BOOT_SEL_SHIFT)
#define EF_CFG_0_EF_CPU0_ENC_EN              (1 << 7)
#define EF_CFG_0_EF_CPU1_ENC_EN              (1 << 6)
#define EF_CFG_0_EF_SBOOT_EN_SHIFT           (4)
#define EF_CFG_0_EF_SBOOT_EN_MASK            (0x03 << EF_CFG_0_EF_SBOOT_EN_SHIFT)
#define EF_CFG_0_EF_SBOOT_SIGN_MODE_SHIFT    (2)
#define EF_CFG_0_EF_SBOOT_SIGN_MODE_MASK     (0x03 << EF_CFG_0_EF_SBOOT_SIGN_MODE_SHIFT)
#define EF_CFG_0_EF_SF_AES_MODE_MASK         (0x03)

#define EF_LOCK_RD_LOCK_KEY_SLOT_5    (1 << 31)
#define EF_LOCK_RD_LOCK_KEY_SLOT_4    (1 << 30)
#define EF_LOCK_RD_LOCK_KEY_SLOT_3    (1 << 29)
#define EF_LOCK_RD_LOCK_KEY_SLOT_2    (1 << 28)
#define EF_LOCK_RD_LOCK_KEY_SLOT_1    (1 << 27)
#define EF_LOCK_RD_LOCK_KEY_SLOT_0    (1 << 26)
#define EF_LOCK_RD_LOCK_DBG_PWD       (1 << 25)
#define EF_LOCK_WR_LOCK_KEY_SLOT_5_H  (1 << 24)
#define EF_LOCK_WR_LOCK_KEY_SLOT_4_H  (1 << 23)
#define EF_LOCK_WR_LOCK_KEY_SLOT_3    (1 << 22)
#define EF_LOCK_WR_LOCK_KEY_SLOT_2    (1 << 21)
#define EF_LOCK_WR_LOCK_KEY_SLOT_1    (1 << 20)
#define EF_LOCK_WR_LOCK_KEY_SLOT_0    (1 << 19)
#define EF_LOCK_WR_LOCK_WIFI_MAC      (1 << 18)
#define EF_LOCK_WR_LOCK_SW_USAGE_0    (1 << 17)
#define EF_LOCK_WR_LOCK_DBG_PWD       (1 << 16)
#define EF_LOCK_WR_LOCK_BOOT_MODE     (1 << 15)
#define EF_LOCK_WR_LOCK_KEY_SLOT_5_L  (1 << 14)
#define EF_LOCK_WR_LOCK_KEY_SLOT_4_L  (1 << 13)
#define EF_LOCK_EF_ANA_TRIM_1_MASK    (0x1fff)

#define EF_DATA_1_REG_DATA_1_LOCK_RD_LOCK_KEY_SLOT_9  (1 << 29)
#define EF_DATA_1_REG_DATA_1_LOCK_RD_LOCK_KEY_SLOT_8  (1 << 28)
#define EF_DATA_1_REG_DATA_1_LOCK_RD_LOCK_KEY_SLOT_7  (1 << 27)
#define EF_DATA_1_REG_DATA_1_LOCK_RD_LOCK_KEY_SLOT_6  (1 << 26)
#define EF_DATA_1_REG_DATA_1_LOCK_WR_LOCK_KEY_SLOT_9  (1 << 13)
#define EF_DATA_1_REG_DATA_1_LOCK_WR_LOCK_KEY_SLOT_8  (1 << 12)
#define EF_DATA_1_REG_DATA_1_LOCK_WR_LOCK_KEY_SLOT_7  (1 << 11)
#define EF_DATA_1_REG_DATA_1_LOCK_WR_LOCK_KEY_SLOT_6  (1 << 10)

#define EF_IF_CTRL_0_EF_IF_PROT_CODE_CYC_SHIFT       (24)
#define EF_IF_CTRL_0_EF_IF_PROT_CODE_CYC_MASK        (0xff << EF_IF_CTRL_0_EF_IF_PROT_CODE_CYC_SHIFT)
#define EF_IF_CTRL_0_EF_IF_0_INT_SET                 (1 << 22)
#define EF_IF_CTRL_0_EF_IF_0_INT_CLR                 (1 << 21)
#define EF_IF_CTRL_0_EF_IF_0_INT                     (1 << 20)
#define EF_IF_CTRL_0_EF_IF_CYC_MODIFY_LOCK           (1 << 19)
#define EF_IF_CTRL_0_EF_IF_AUTO_RD_EN                (1 << 18)
#define EF_IF_CTRL_0_EF_CLK_SAHB_DATA_GATE           (1 << 17)
#define EF_IF_CTRL_0_EF_IF_POR_DIG                   (1 << 16)
#define EF_IF_CTRL_0_EF_IF_PROT_CODE_CTRL_SHIFT      (8)
#define EF_IF_CTRL_0_EF_IF_PROT_CODE_CTRL_MASK       (0xff << EF_IF_CTRL_0_EF_IF_PROT_CODE_CTRL_SHIFT)
#define EF_IF_CTRL_0_EF_CLK_SAHB_DATA_SEL            (1 << 7)
#define EF_IF_CTRL_0_EF_IF_0_CYC_MODIFY              (1 << 6)
#define EF_IF_CTRL_0_EF_IF_0_MANUAL_EN               (1 << 5)
#define EF_IF_CTRL_0_EF_IF_0_TRIG                    (1 << 4)
#define EF_IF_CTRL_0_EF_IF_0_RW                      (1 << 3)
#define EF_IF_CTRL_0_EF_IF_0_BUSY                    (1 << 2)
#define EF_IF_CTRL_0_EF_IF_0_AUTOLOAD_DONE           (1 << 1)
#define EF_IF_CTRL_0_EF_IF_0_AUTOLOAD_P1_DONE        (1 << 0)

#define EF_IF_CYC_0_EF_IF_CYC_PD_CS_S_SHIFT          (24)
#define EF_IF_CYC_0_EF_IF_CYC_PD_CS_S_MASK           (0xff << EF_IF_CYC_0_EF_IF_CYC_PD_CS_S_SHIFT)
#define EF_IF_CYC_0_EF_IF_CYC_CS_SHIFT               (18)
#define EF_IF_CYC_0_EF_IF_CYC_CS_MASK                (0x3f << EF_IF_CYC_0_EF_IF_CYC_CS_SHIFT)
#define EF_IF_CYC_0_EF_IF_CYC_RD_ADR_SHIFT           (12)
#define EF_IF_CYC_0_EF_IF_CYC_RD_ADR_MASK            (0x3f << EF_IF_CYC_0_EF_IF_CYC_RD_ADR_SHIFT)
#define EF_IF_CYC_0_EF_IF_CYC_RD_DAT_SHIFT           (6)
#define EF_IF_CYC_0_EF_IF_CYC_RD_DAT_MASK            (0x3f << EF_IF_CYC_0_EF_IF_CYC_RD_DAT_SHIFT)
#define EF_IF_CYC_0_EF_IF_CYC_RD_DMY_MASK            (0x3f)

#define EF_IF_CYC_1_EF_IF_CYC_PD_CS_H_SHIFT          (26)
#define EF_IF_CYC_1_EF_IF_CYC_PD_CS_H_MASK           (0x3f << EF_IF_CYC_1_EF_IF_CYC_PD_CS_H_SHIFT)
#define EF_IF_CYC_1_EF_IF_CYC_PS_CS_SHIFT            (20)
#define EF_IF_CYC_1_EF_IF_CYC_PS_CS_MASK             (0x3f << EF_IF_CYC_1_EF_IF_CYC_PS_CS_SHIFT)
#define EF_IF_CYC_1_EF_IF_CYC_WR_ADR_SHIFT           (14)
#define EF_IF_CYC_1_EF_IF_CYC_WR_ADR_MASK            (0x3f << EF_IF_CYC_1_EF_IF_CYC_WR_ADR_SHIFT)
#define EF_IF_CYC_1_EF_IF_CYC_PP_SHIFT               (6)
#define EF_IF_CYC_1_EF_IF_CYC_PP_MASK                (0xff << EF_IF_CYC_1_EF_IF_CYC_PP_SHIFT)
#define EF_IF_CYC_1_EF_IF_CYC_PI_MASK                (0x3f)

#define EF_IF_0_MANUAL_EF_IF_PROT_CODE_MANUAL_SHIFT  (24)
#define EF_IF_0_MANUAL_EF_IF_PROT_CODE_MANUAL_MASK   (0xff << EF_IF_0_MANUAL_EF_IF_PROT_CODE_MANUAL_SHIFT)
#define EF_IF_0_MANUAL_EF_IF_0_Q_SHIFT               (16)
#define EF_IF_0_MANUAL_EF_IF_0_Q_MASK                (0xff << EF_IF_0_MANUAL_EF_IF_0_Q_SHIFT)
#define EF_IF_0_MANUAL_EF_IF_CSB                     (1 << 15)
#define EF_IF_0_MANUAL_EF_IF_LOAD                    (1 << 14)
#define EF_IF_0_MANUAL_EF_IF_PGENB                   (1 << 13)
#define EF_IF_0_MANUAL_EF_IF_STROBE                  (1 << 12)
#define EF_IF_0_MANUAL_EF_IF_PS                      (1 << 11)
#define EF_IF_0_MANUAL_EF_IF_PD                      (1 << 10)
#define EF_IF_0_MANUAL_EF_IF_A_MASK                  (0x3ff)

#define EF_IF_CFG_0_EF_IF_DBG_MODE_SHIFT             (28)
#define EF_IF_CFG_0_EF_IF_DBG_MODE_MASK              (0x0f << EF_IF_CFG_0_EF_IF_DBG_MODE_SHIFT)
#define EF_IF_CFG_0_EF_IF_DBG_JTAG_0_DIS_SHIFT       (26)
#define EF_IF_CFG_0_EF_IF_DBG_JTAG_0_DIS_MASK        (0x03 << EF_IF_CFG_0_EF_IF_DBG_JTAG_0_DIS_SHIFT)
#define EF_IF_CFG_0_EF_IF_DBG_JTAG_1_DIS_SHIFT       (24)
#define EF_IF_CFG_0_EF_IF_DBG_JTAG_1_DIS_MASK        (0x03 << EF_IF_CFG_0_EF_IF_DBG_JTAG_1_DIS_SHIFT)
#define EF_IF_CFG_0_EF_IF_EFUSE_DBG_DIS              (1 << 23)
#define EF_IF_CFG_0_EF_IF_SE_DBG_DIS                 (1 << 22)
#define EF_IF_CFG_0_EF_IF_CPU_RST_DBG_DIS            (1 << 21)
#define EF_IF_CFG_0_EF_IF_CPU1_DIS                   (1 << 20)
#define EF_IF_CFG_0_EF_IF_SF_DIS                     (1 << 19)
#define EF_IF_CFG_0_EF_IF_CAM_DIS                    (1 << 18)
#define EF_IF_CFG_0_EF_IF_0_KEY_ENC_EN               (1 << 17)
#define EF_IF_CFG_0_EF_IF_WIFI_DIS                   (1 << 16)
#define EF_IF_CFG_0_EF_IF_BLE_DIS                    (1 << 15)
#define EF_IF_CFG_0_EF_IF_SDU_DIS                    (1 << 14)
#define EF_IF_CFG_0_EF_IF_SW_USAGE_1_SHIFT           (12)
#define EF_IF_CFG_0_EF_IF_SW_USAGE_1_MASK            (0x03 << EF_IF_CFG_0_EF_IF_SW_USAGE_1_SHIFT)
#define EF_IF_CFG_0_EF_IF_BOOT_SEL_SHIFT             (8)
#define EF_IF_CFG_0_EF_IF_BOOT_SEL_MASK              (0x0f << EF_IF_CFG_0_EF_IF_BOOT_SEL_SHIFT)
#define EF_IF_CFG_0_EF_IF_CPU0_ENC_EN                (1 << 7)
#define EF_IF_CFG_0_EF_IF_CPU1_ENC_EN                (1 << 6)
#define EF_IF_CFG_0_EF_IF_SBOOT_EN_SHIFT             (4)
#define EF_IF_CFG_0_EF_IF_SBOOT_EN_MASK              (0x03 << EF_IF_CFG_0_EF_IF_SBOOT_EN_SHIFT)
#define EF_IF_CFG_0_EF_IF_SBOOT_SIGN_MODE_SHIFT      (2)
#define EF_IF_CFG_0_EF_IF_SBOOT_SIGN_MODE_MASK       (0x03 << EF_IF_CFG_0_EF_IF_SBOOT_SIGN_MODE_SHIFT)
#define EF_IF_CFG_0_EF_IF_SF_AES_MODE_MASK           (0x03)

#define EF_SW_CFG_0_EF_SW_DBG_MODE_SHIFT             (28)
#define EF_SW_CFG_0_EF_SW_DBG_MODE_MASK              (0x0f << EF_SW_CFG_0_EF_SW_DBG_MODE_SHIFT)
#define EF_SW_CFG_0_EF_SW_DBG_JTAG_0_DIS_SHIFT       (26)
#define EF_SW_CFG_0_EF_SW_DBG_JTAG_0_DIS_MASK        (0x03 << EF_SW_CFG_0_EF_SW_DBG_JTAG_0_DIS_SHIFT)
#define EF_SW_CFG_0_EF_SW_DBG_JTAG_1_DIS_SHIFT       (24)
#define EF_SW_CFG_0_EF_SW_DBG_JTAG_1_DIS_MASK        (0x03 << EF_SW_CFG_0_EF_SW_DBG_JTAG_1_DIS_SHIFT)
#define EF_SW_CFG_0_EF_SW_EFUSE_DBG_DIS              (1 << 23)
#define EF_SW_CFG_0_EF_SW_SE_DBG_DIS                 (1 << 22)
#define EF_SW_CFG_0_EF_SW_CPU_RST_DBG_DIS            (1 << 21)
#define EF_SW_CFG_0_EF_SW_CPU1_DIS                   (1 << 20)
#define EF_SW_CFG_0_EF_SW_SF_DIS                     (1 << 19)
#define EF_SW_CFG_0_EF_SW_CAM_DIS                    (1 << 18)
#define EF_SW_CFG_0_EF_SW_0_KEY_ENC_EN               (1 << 17)
#define EF_SW_CFG_0_EF_SW_WIFI_DIS                   (1 << 16)
#define EF_SW_CFG_0_EF_SW_BLE_DIS                    (1 << 15)
#define EF_SW_CFG_0_EF_SW_SDU_DIS                    (1 << 14)
#define EF_SW_CFG_0_EF_SW_SW_USAGE_1_SHIFT           (12)
#define EF_SW_CFG_0_EF_SW_SW_USAGE_1_MASK            (0x03 << EF_SW_CFG_0_EF_SW_SW_USAGE_1_SHIFT)
#define EF_SW_CFG_0_EF_SW_CPU0_ENC_EN                (1 << 7)
#define EF_SW_CFG_0_EF_SW_CPU1_ENC_EN                (1 << 6)
#define EF_SW_CFG_0_EF_SW_SBOOT_EN_SHIFT             (4)
#define EF_SW_CFG_0_EF_SW_SBOOT_EN_MASK              (0x03 << EF_SW_CFG_0_EF_SW_SBOOT_EN_SHIFT)
#define EF_SW_CFG_0_EF_SW_SBOOT_SIGN_MODE_SHIFT      (2)
#define EF_SW_CFG_0_EF_SW_SBOOT_SIGN_MODE_MASK       (0x03 << EF_SW_CFG_0_EF_SW_SBOOT_SIGN_MODE_SHIFT)
#define EF_SW_CFG_0_EF_SW_SF_AES_MODE_MASK           (0x03)

#define EF_CRC_CTRL_0_EF_CRC_SLP_N_SHIFT             (16)
#define EF_CRC_CTRL_0_EF_CRC_SLP_N_MASK              (0xffff << EF_CRC_CTRL_0_EF_CRC_SLP_N_SHIFT)
#define EF_CRC_CTRL_0_EF_CRC_LOCK                    (1 << 11)
#define EF_CRC_CTRL_0_EF_CRC_INT_SET                 (1 << 10)
#define EF_CRC_CTRL_0_EF_CRC_INT_CLR                 (1 << 9)
#define EF_CRC_CTRL_0_EF_CRC_INT                     (1 << 8)
#define EF_CRC_CTRL_0_EF_CRC_DIN_ENDIAN              (1 << 7)
#define EF_CRC_CTRL_0_EF_CRC_DOUT_ENDIAN             (1 << 6)
#define EF_CRC_CTRL_0_EF_CRC_DOUT_INV_EN             (1 << 5)
#define EF_CRC_CTRL_0_EF_CRC_ERROR                   (1 << 4)
#define EF_CRC_CTRL_0_EF_CRC_MODE                    (1 << 3)
#define EF_CRC_CTRL_0_EF_CRC_EN                      (1 << 2)
#define EF_CRC_CTRL_0_EF_CRC_TRIG                    (1 << 1)
#define EF_CRC_CTRL_0_EF_CRC_BUSY                    (1 << 0)

#endif /* __ARCH_RISCV_SRC_BL602_HARDWARE_BL602_EF_H */
