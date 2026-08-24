/****************************************************************************
 * arch/arm/src/stm32c5/hardware/stm32c5xx_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32C5_HARDWARE_STM32C5XX_FLASH_H
#define __ARCH_ARM_SRC_STM32C5_HARDWARE_STM32C5XX_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FLASH geometry ***********************************************************/

#define STM32_FLASH_NPAGES              64
#define STM32_FLASH_PAGESIZE            8192

#define STM32_FLASH_SIZE \
  (STM32_FLASH_NPAGES * STM32_FLASH_PAGESIZE)

/* Register Offsets *********************************************************/

#define STM32_FLASH_ACR_OFFSET            0x0000
#define STM32_FLASH_KEYR_OFFSET           0x0004
#define STM32_FLASH_OPTKEYR_OFFSET        0x000c
#define STM32_FLASH_OPSR_OFFSET           0x0018
#define STM32_FLASH_OPTCR_OFFSET          0x001c
#define STM32_FLASH_SR_OFFSET             0x0020
#define STM32_FLASH_CR_OFFSET             0x0028
#define STM32_FLASH_CCR_OFFSET            0x0030
#define STM32_FLASH_PRIVCFGR_OFFSET       0x003c
#define STM32_FLASH_HDPEXTR_OFFSET        0x0048
#define STM32_FLASH_OPTSR_CUR_OFFSET      0x0050
#define STM32_FLASH_OPTSR_PRG_OFFSET      0x0054
#define STM32_FLASH_OPTSR2_CUR_OFFSET     0x0070
#define STM32_FLASH_OPTSR2_PRG_OFFSET     0x0074
#define STM32_FLASH_BOOTR_CUR_OFFSET      0x0080
#define STM32_FLASH_BOOTR_PRG_OFFSET      0x0084
#define STM32_FLASH_OTPBLR_CUR_OFFSET     0x0090
#define STM32_FLASH_OTPBLR_PRG_OFFSET     0x0094
#define STM32_FLASH_BL_COM_CFG_CUR_OFFSET 0x0098
#define STM32_FLASH_BL_COM_CFG_PRG_OFFSET 0x009c
#define STM32_FLASH_OEMKEYR1_PRG_OFFSET   0x00a4
#define STM32_FLASH_OEMKEYR2_PRG_OFFSET   0x00ac
#define STM32_FLASH_OEMKEYR3_PRG_OFFSET   0x00b4
#define STM32_FLASH_OEMKEYR4_PRG_OFFSET   0x00bc
#define STM32_FLASH_BSKEYR_PRG_OFFSET     0x00c4
#define STM32_FLASH_WRP1R_CUR_OFFSET      0x00e8
#define STM32_FLASH_WRP1R_PRG_OFFSET      0x00ec
#define STM32_FLASH_HDP1R_CUR_OFFSET      0x00f8
#define STM32_FLASH_HDP1R_PRG_OFFSET      0x00fc
#define STM32_FLASH_ECCCORR_OFFSET        0x0100
#define STM32_FLASH_ECCDETR_OFFSET        0x0104
#define STM32_FLASH_ECCDR_OFFSET          0x0108
#define STM32_FLASH_WRP2R_CUR_OFFSET      0x01e8
#define STM32_FLASH_WRP2R_PRG_OFFSET      0x01ec
#define STM32_FLASH_HDP2R_CUR_OFFSET      0x01f8
#define STM32_FLASH_HDP2R_PRG_OFFSET      0x01fc

/* Register Addresses *******************************************************/

#define STM32_FLASH_ACR            (STM32_FLASHIF_BASE + STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR           (STM32_FLASHIF_BASE + STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_OPTKEYR        (STM32_FLASHIF_BASE + STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_OPSR           (STM32_FLASHIF_BASE + STM32_FLASH_OPSR_OFFSET)
#define STM32_FLASH_OPTCR          (STM32_FLASHIF_BASE + STM32_FLASH_OPTCR_OFFSET)
#define STM32_FLASH_SR             (STM32_FLASHIF_BASE + STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_CR             (STM32_FLASHIF_BASE + STM32_FLASH_CR_OFFSET)
#define STM32_FLASH_CCR            (STM32_FLASHIF_BASE + STM32_FLASH_CCR_OFFSET)
#define STM32_FLASH_PRIVCFGR       (STM32_FLASHIF_BASE + STM32_FLASH_PRIVCFGR_OFFSET)
#define STM32_FLASH_HDPEXTR        (STM32_FLASHIF_BASE + STM32_FLASH_HDPEXTR_OFFSET)
#define STM32_FLASH_OPTSR_CUR      (STM32_FLASHIF_BASE + STM32_FLASH_OPTSR_CUR_OFFSET)
#define STM32_FLASH_OPTSR_PRG      (STM32_FLASHIF_BASE + STM32_FLASH_OPTSR_PRG_OFFSET)
#define STM32_FLASH_OPTSR2_CUR     (STM32_FLASHIF_BASE + STM32_FLASH_OPTSR2_CUR_OFFSET)
#define STM32_FLASH_OPTSR2_PRG     (STM32_FLASHIF_BASE + STM32_FLASH_OPTSR2_PRG_OFFSET)
#define STM32_FLASH_BOOTR_CUR      (STM32_FLASHIF_BASE + STM32_FLASH_BOOTR_CUR_OFFSET)
#define STM32_FLASH_BOOTR_PRG      (STM32_FLASHIF_BASE + STM32_FLASH_BOOTR_PRG_OFFSET)
#define STM32_FLASH_OTPBLR_CUR     (STM32_FLASHIF_BASE + STM32_FLASH_OTPBLR_CUR_OFFSET)
#define STM32_FLASH_OTPBLR_PRG     (STM32_FLASHIF_BASE + STM32_FLASH_OTPBLR_PRG_OFFSET)
#define STM32_FLASH_BL_COM_CFG_CUR (STM32_FLASHIF_BASE + STM32_FLASH_BL_COM_CFG_CUR_OFFSET)
#define STM32_FLASH_BL_COM_CFG_PRG (STM32_FLASHIF_BASE + STM32_FLASH_BL_COM_CFG_PRG_OFFSET)
#define STM32_FLASH_OEMKEYR1_PRG   (STM32_FLASHIF_BASE + STM32_FLASH_OEMKEYR1_PRG_OFFSET)
#define STM32_FLASH_OEMKEYR2_PRG   (STM32_FLASHIF_BASE + STM32_FLASH_OEMKEYR2_PRG_OFFSET)
#define STM32_FLASH_OEMKEYR3_PRG   (STM32_FLASHIF_BASE + STM32_FLASH_OEMKEYR3_PRG_OFFSET)
#define STM32_FLASH_OEMKEYR4_PRG   (STM32_FLASHIF_BASE + STM32_FLASH_OEMKEYR4_PRG_OFFSET)
#define STM32_FLASH_BSKEYR_PRG     (STM32_FLASHIF_BASE + STM32_FLASH_BSKEYR_PRG_OFFSET)
#define STM32_FLASH_WRP1R_CUR      (STM32_FLASHIF_BASE + STM32_FLASH_WRP1R_CUR_OFFSET)
#define STM32_FLASH_WRP1R_PRG      (STM32_FLASHIF_BASE + STM32_FLASH_WRP1R_PRG_OFFSET)
#define STM32_FLASH_HDP1R_CUR      (STM32_FLASHIF_BASE + STM32_FLASH_HDP1R_CUR_OFFSET)
#define STM32_FLASH_HDP1R_PRG      (STM32_FLASHIF_BASE + STM32_FLASH_HDP1R_PRG_OFFSET)
#define STM32_FLASH_ECCCORR        (STM32_FLASHIF_BASE + STM32_FLASH_ECCCORR_OFFSET)
#define STM32_FLASH_ECCDETR        (STM32_FLASHIF_BASE + STM32_FLASH_ECCDETR_OFFSET)
#define STM32_FLASH_ECCDR          (STM32_FLASHIF_BASE + STM32_FLASH_ECCDR_OFFSET)
#define STM32_FLASH_WRP2R_CUR      (STM32_FLASHIF_BASE + STM32_FLASH_WRP2R_CUR_OFFSET)
#define STM32_FLASH_WRP2R_PRG      (STM32_FLASHIF_BASE + STM32_FLASH_WRP2R_PRG_OFFSET)
#define STM32_FLASH_HDP2R_CUR      (STM32_FLASHIF_BASE + STM32_FLASH_HDP2R_CUR_OFFSET)
#define STM32_FLASH_HDP2R_PRG      (STM32_FLASHIF_BASE + STM32_FLASH_HDP2R_PRG_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* FLASH access control register */

#define FLASH_ACR_RESET                       0x00000027
#define FLASH_ACR_LATENCY_SHIFT               (0)
#define FLASH_ACR_LATENCY_MASK                (0xf << FLASH_ACR_LATENCY_SHIFT)
#define FLASH_ACR_LATENCY(n)                  ((n) << FLASH_ACR_LATENCY_SHIFT)
#define FLASH_ACR_LATENCY_0                 FLASH_ACR_LATENCY(0)
#define FLASH_ACR_LATENCY_1                 FLASH_ACR_LATENCY(1)
#define FLASH_ACR_LATENCY_2                 FLASH_ACR_LATENCY(2)
#define FLASH_ACR_LATENCY_3                 FLASH_ACR_LATENCY(3)
#define FLASH_ACR_LATENCY_4                 FLASH_ACR_LATENCY(4)
#define FLASH_ACR_LATENCY_5                 FLASH_ACR_LATENCY(5)
#define FLASH_ACR_LATENCY_6                 FLASH_ACR_LATENCY(6)
#define FLASH_ACR_LATENCY_7                 FLASH_ACR_LATENCY(7)
#define FLASH_ACR_LATENCY_8                 FLASH_ACR_LATENCY(8)
#define FLASH_ACR_LATENCY_9                 FLASH_ACR_LATENCY(9)
#define FLASH_ACR_LATENCY_10                FLASH_ACR_LATENCY(10)
#define FLASH_ACR_LATENCY_11                FLASH_ACR_LATENCY(11)
#define FLASH_ACR_LATENCY_12                FLASH_ACR_LATENCY(12)
#define FLASH_ACR_LATENCY_13                FLASH_ACR_LATENCY(13)
#define FLASH_ACR_LATENCY_14                FLASH_ACR_LATENCY(14)
#define FLASH_ACR_LATENCY_15                FLASH_ACR_LATENCY(15)
#define FLASH_ACR_WRHIGHFREQ_SHIFT            (4)
#define FLASH_ACR_WRHIGHFREQ_MASK             (0x3 << FLASH_ACR_WRHIGHFREQ_SHIFT)
#define FLASH_ACR_WRHIGHFREQ(n)               ((n) << FLASH_ACR_WRHIGHFREQ_SHIFT)
#define FLASH_ACR_WRHIGHFREQ_0              FLASH_ACR_WRHIGHFREQ(0)
#define FLASH_ACR_WRHIGHFREQ_1              FLASH_ACR_WRHIGHFREQ(1)
#define FLASH_ACR_WRHIGHFREQ_2              FLASH_ACR_WRHIGHFREQ(2)
#define FLASH_ACR_PRFTEN                      (1 << 8)
#define FLASH_ACR_EMPTY                       (1 << 16)

/* FLASH key register */

#define FLASH_KEYR_RESET                      0x00000000
#define FLASH_KEYR_KEY_SHIFT                  (0)
#define FLASH_KEYR_KEY_MASK                   (0xffffffff << FLASH_KEYR_KEY_SHIFT)
#define FLASH_KEYR_KEY(n)                     ((n) << FLASH_KEYR_KEY_SHIFT)
#define FLASH_KEYR_KEY1                     0x45670123
#define FLASH_KEYR_KEY2                     0xcdef89ab

/* FLASH option key register */

#define FLASH_OPTKEYR_RESET                   0x00000000
#define FLASH_OPTKEYR_OPTKEY_SHIFT            (0)
#define FLASH_OPTKEYR_OPTKEY_MASK             (0xffffffff << FLASH_OPTKEYR_OPTKEY_SHIFT)
#define FLASH_OPTKEYR_OPTKEY(n)               ((n) << FLASH_OPTKEYR_OPTKEY_SHIFT)
#define FLASH_OPTKEYR_OPTKEY1               0x08192a3b
#define FLASH_OPTKEYR_OPTKEY2               0x4c5d6e7f

/* FLASH operation status register */

#define FLASH_OPSR_RESET                      0x00000000
#define FLASH_OPSR_ADDR_OP_SHIFT              (0)
#define FLASH_OPSR_ADDR_OP_MASK               (0xffff << FLASH_OPSR_ADDR_OP_SHIFT)
#define FLASH_OPSR_ADDR_OP(n)                 ((n) << FLASH_OPSR_ADDR_OP_SHIFT)
#define FLASH_OPSR_DATA_OP                    (1 << 21)
#define FLASH_OPSR_BK_OP                      (1 << 22)
#define FLASH_OPSR_OTP_OP                     (1 << 24)
#define FLASH_OPSR_CODE_OP_SHIFT              (29)
#define FLASH_OPSR_CODE_OP_MASK               (0x7 << FLASH_OPSR_CODE_OP_SHIFT)
#define FLASH_OPSR_CODE_OP(n)                 ((n) << FLASH_OPSR_CODE_OP_SHIFT)
#define FLASH_OPSR_CODE_OP_NONE             FLASH_OPSR_CODE_OP(0)
#define FLASH_OPSR_CODE_OP_WRITE            FLASH_OPSR_CODE_OP(1)
#define FLASH_OPSR_CODE_OP_PAGE_ERASE       FLASH_OPSR_CODE_OP(3)
#define FLASH_OPSR_CODE_OP_BANK_ERASE       FLASH_OPSR_CODE_OP(4)
#define FLASH_OPSR_CODE_OP_MASS_ERASE       FLASH_OPSR_CODE_OP(5)
#define FLASH_OPSR_CODE_OP_OPTION_CHANGE    FLASH_OPSR_CODE_OP(6)

/* FLASH option control register */

#define FLASH_OPTCR_RESET                     0x00000001
#define FLASH_OPTCR_OPTLOCK                   (1 << 0)
#define FLASH_OPTCR_OPTSTRT                   (1 << 1)
#define FLASH_OPTCR_SWAP_BANK                 (1 << 31)

/* FLASH status register */

#define FLASH_SR_RESET                        0x00000000
#define FLASH_SR_BSY                          (1 << 0)
#define FLASH_SR_WBNE                         (1 << 1)
#define FLASH_SR_DBNE                         (1 << 3)
#define FLASH_SR_OEMLOCK                      (1 << 8)
#define FLASH_SR_BSLOCK                       (1 << 9)
#define FLASH_SR_EOP                          (1 << 16)
#define FLASH_SR_WRPERR                       (1 << 17)
#define FLASH_SR_PGSERR                       (1 << 18)
#define FLASH_SR_STRBERR                      (1 << 19)
#define FLASH_SR_INCERR                       (1 << 20)
#define FLASH_SR_OPTCHANGEERR                 (1 << 23)

/* FLASH control register */

#define FLASH_CR_RESET                        0x00000001
#define FLASH_CR_LOCK                         (1 << 0)
#define FLASH_CR_PG                           (1 << 1)
#define FLASH_CR_PER                          (1 << 2)
#define FLASH_CR_BER                          (1 << 3)
#define FLASH_CR_FW                           (1 << 4)
#define FLASH_CR_STRT                         (1 << 5)
#define FLASH_CR_PNB_SHIFT                    (6)
#define FLASH_CR_PNB_MASK                     (0x3f << FLASH_CR_PNB_SHIFT)
#define FLASH_CR_PNB(n)                       ((n) << FLASH_CR_PNB_SHIFT)
#define FLASH_CR_MER                          (1 << 15)
#define FLASH_CR_EOPIE                        (1 << 16)
#define FLASH_CR_WRPERRIE                     (1 << 17)
#define FLASH_CR_PGSERRIE                     (1 << 18)
#define FLASH_CR_STRBERRIE                    (1 << 19)
#define FLASH_CR_INCERRIE                     (1 << 20)
#define FLASH_CR_OPTCHANGEERRIE               (1 << 23)
#define FLASH_CR_EDATASEL                     (1 << 29)
#define FLASH_CR_BKSEL                        (1 << 31)

/* FLASH clear control register */

#define FLASH_CCR_RESET                       0x00000000
#define FLASH_CCR_CLR_EOP                     (1 << 16)
#define FLASH_CCR_CLR_WRPERR                  (1 << 17)
#define FLASH_CCR_CLR_PGSERR                  (1 << 18)
#define FLASH_CCR_CLR_STRBERR                 (1 << 19)
#define FLASH_CCR_CLR_INCERR                  (1 << 20)
#define FLASH_CCR_CLR_OPTCHANGEERR            (1 << 23)

/* FLASH privilege configuration register */

#define FLASH_PRIVCFGR_RESET                  0x00000000
#define FLASH_PRIVCFGR_PRIV                   (1 << 1)

/* FLASH HDP extension register */

#define FLASH_HDPEXTR_RESET                   0x00000000
#define FLASH_HDPEXTR_HDP1_EXT_SHIFT          (0)
#define FLASH_HDPEXTR_HDP1_EXT_MASK           (0x3f << FLASH_HDPEXTR_HDP1_EXT_SHIFT)
#define FLASH_HDPEXTR_HDP1_EXT(n)             ((n) << FLASH_HDPEXTR_HDP1_EXT_SHIFT)
#define FLASH_HDPEXTR_HDP2_EXT_SHIFT          (16)
#define FLASH_HDPEXTR_HDP2_EXT_MASK           (0x3f << FLASH_HDPEXTR_HDP2_EXT_SHIFT)
#define FLASH_HDPEXTR_HDP2_EXT(n)             ((n) << FLASH_HDPEXTR_HDP2_EXT_SHIFT)

/* FLASH option status register */

#define FLASH_OPTSR_CUR_RESET                 0x00000000
#define FLASH_OPTSR_CUR_IWDG_SW               (1 << 3)
#define FLASH_OPTSR_CUR_WWDG_SW               (1 << 4)
#define FLASH_OPTSR_CUR_NRST_STOP             (1 << 6)
#define FLASH_OPTSR_CUR_NRST_STDBY            (1 << 7)
#define FLASH_OPTSR_CUR_RDP_LEVEL_SHIFT       (8)
#define FLASH_OPTSR_CUR_RDP_LEVEL_MASK        (0xff << FLASH_OPTSR_CUR_RDP_LEVEL_SHIFT)
#define FLASH_OPTSR_CUR_RDP_LEVEL(n)          ((n) << FLASH_OPTSR_CUR_RDP_LEVEL_SHIFT)
#define FLASH_OPTSR_CUR_RDP_LEVEL_0         FLASH_OPTSR_CUR_RDP_LEVEL(0xed)
#define FLASH_OPTSR_CUR_RDP_LEVEL_2_WBS     FLASH_OPTSR_CUR_RDP_LEVEL(0xd1)
#define FLASH_OPTSR_CUR_RDP_LEVEL_2         FLASH_OPTSR_CUR_RDP_LEVEL(0x72)
#define FLASH_OPTSR_CUR_IWDG_STOP             (1 << 20)
#define FLASH_OPTSR_CUR_IWDG_STDBY            (1 << 21)
#define FLASH_OPTSR_CUR_BOOT_SEL              (1 << 22)
#define FLASH_OPTSR_CUR_BOOT0                 (1 << 23)
#define FLASH_OPTSR_CUR_EDATA_EN              (1 << 29)
#define FLASH_OPTSR_CUR_SINGLE_BANK           (1 << 30)
#define FLASH_OPTSR_CUR_SWAP_BANK             (1 << 31)

/* FLASH option status register */

#define FLASH_OPTSR_PRG_RESET                 0x00000000
#define FLASH_OPTSR_PRG_IWDG_SW               (1 << 3)
#define FLASH_OPTSR_PRG_WWDG_SW               (1 << 4)
#define FLASH_OPTSR_PRG_NRST_STOP             (1 << 6)
#define FLASH_OPTSR_PRG_NRST_STDBY            (1 << 7)
#define FLASH_OPTSR_PRG_RDP_LEVEL_SHIFT       (8)
#define FLASH_OPTSR_PRG_RDP_LEVEL_MASK        (0xff << FLASH_OPTSR_PRG_RDP_LEVEL_SHIFT)
#define FLASH_OPTSR_PRG_RDP_LEVEL(n)          ((n) << FLASH_OPTSR_PRG_RDP_LEVEL_SHIFT)
#define FLASH_OPTSR_PRG_RDP_LEVEL_0         FLASH_OPTSR_PRG_RDP_LEVEL(0xed)
#define FLASH_OPTSR_PRG_RDP_LEVEL_2_WBS     FLASH_OPTSR_PRG_RDP_LEVEL(0xd1)
#define FLASH_OPTSR_PRG_RDP_LEVEL_2         FLASH_OPTSR_PRG_RDP_LEVEL(0x72)
#define FLASH_OPTSR_PRG_IWDG_STOP             (1 << 20)
#define FLASH_OPTSR_PRG_IWDG_STDBY            (1 << 21)
#define FLASH_OPTSR_PRG_BOOT_SEL              (1 << 22)
#define FLASH_OPTSR_PRG_BOOT0                 (1 << 23)
#define FLASH_OPTSR_PRG_EDATA_EN              (1 << 29)
#define FLASH_OPTSR_PRG_SINGLE_BANK           (1 << 30)
#define FLASH_OPTSR_PRG_SWAP_BANK             (1 << 31)

/* FLASH option status register 2 */

#define FLASH_OPTSR2_CUR_RESET                0x00000000
#define FLASH_OPTSR2_CUR_SRAM1_RST            (1 << 0)
#define FLASH_OPTSR2_CUR_SRAM2_RST            (1 << 1)
#define FLASH_OPTSR2_CUR_SRAM2_ECC            (1 << 4)

/* FLASH option status register 2 */

#define FLASH_OPTSR2_PRG_RESET                0x00000000
#define FLASH_OPTSR2_PRG_SRAM1_RST            (1 << 0)
#define FLASH_OPTSR2_PRG_SRAM2_RST            (1 << 1)
#define FLASH_OPTSR2_PRG_SRAM2_ECC            (1 << 4)

/* FLASH unique boot entry register */

#define FLASH_BOOTR_CUR_RESET                 0x00000000
#define FLASH_BOOTR_CUR_BOOT_LOCK_SHIFT       (0)
#define FLASH_BOOTR_CUR_BOOT_LOCK_MASK        (0xff << FLASH_BOOTR_CUR_BOOT_LOCK_SHIFT)
#define FLASH_BOOTR_CUR_BOOT_LOCK(n)          ((n) << FLASH_BOOTR_CUR_BOOT_LOCK_SHIFT)
#define FLASH_BOOTR_CUR_BOOT_LOCK_MODIFIABLE \
  FLASH_BOOTR_CUR_BOOT_LOCK(0xc3)
#define FLASH_BOOTR_CUR_BOOT_LOCK_FROZEN \
  FLASH_BOOTR_CUR_BOOT_LOCK(0xb4)
#define FLASH_BOOTR_CUR_BOOTADD_SHIFT         (8)
#define FLASH_BOOTR_CUR_BOOTADD_MASK          (0xffffff << FLASH_BOOTR_CUR_BOOTADD_SHIFT)
#define FLASH_BOOTR_CUR_BOOTADD(n)            ((n) << FLASH_BOOTR_CUR_BOOTADD_SHIFT)

/* FLASH unique boot entry address */

#define FLASH_BOOTR_PRG_RESET                 0x00000000
#define FLASH_BOOTR_PRG_BOOT_LOCK_SHIFT       (0)
#define FLASH_BOOTR_PRG_BOOT_LOCK_MASK        (0xff << FLASH_BOOTR_PRG_BOOT_LOCK_SHIFT)
#define FLASH_BOOTR_PRG_BOOT_LOCK(n)          ((n) << FLASH_BOOTR_PRG_BOOT_LOCK_SHIFT)
#define FLASH_BOOTR_PRG_BOOT_LOCK_MODIFIABLE \
  FLASH_BOOTR_PRG_BOOT_LOCK(0xc3)
#define FLASH_BOOTR_PRG_BOOT_LOCK_FROZEN \
  FLASH_BOOTR_PRG_BOOT_LOCK(0xb4)
#define FLASH_BOOTR_PRG_BOOTADD_SHIFT         (8)
#define FLASH_BOOTR_PRG_BOOTADD_MASK          (0xffffff << FLASH_BOOTR_PRG_BOOTADD_SHIFT)
#define FLASH_BOOTR_PRG_BOOTADD(n)            ((n) << FLASH_BOOTR_PRG_BOOTADD_SHIFT)

/* FLASH OTP block lock */

#define FLASH_OTPBLR_CUR_RESET                0x00000000
#define FLASH_OTPBLR_CUR_LOCKBL_SHIFT         (0)
#define FLASH_OTPBLR_CUR_LOCKBL_MASK          (0xffffff << FLASH_OTPBLR_CUR_LOCKBL_SHIFT)
#define FLASH_OTPBLR_CUR_LOCKBL(n)            ((n) << FLASH_OTPBLR_CUR_LOCKBL_SHIFT)

/* FLASH OTP block lock */

#define FLASH_OTPBLR_PRG_RESET                0x00000000
#define FLASH_OTPBLR_PRG_LOCKBL_SHIFT         (0)
#define FLASH_OTPBLR_PRG_LOCKBL_MASK          (0xffffff << FLASH_OTPBLR_PRG_LOCKBL_SHIFT)
#define FLASH_OTPBLR_PRG_LOCKBL(n)            ((n) << FLASH_OTPBLR_PRG_LOCKBL_SHIFT)

/* FLASH bootloader interface selection */

#define FLASH_BL_COM_CFG_CUR_RESET            0x00000000
#define FLASH_BL_COM_CFG_CUR_BL_COM_CFG_SHIFT (0)
#define FLASH_BL_COM_CFG_CUR_BL_COM_CFG_MASK  (0xffffffff << FLASH_BL_COM_CFG_CUR_BL_COM_CFG_SHIFT)
#define FLASH_BL_COM_CFG_CUR_BL_COM_CFG(n)    ((n) << FLASH_BL_COM_CFG_CUR_BL_COM_CFG_SHIFT)

/* FLASH bootloader interface selection */

#define FLASH_BL_COM_CFG_PRG_RESET            0x00000000
#define FLASH_BL_COM_CFG_PRG_BL_COM_CFG_SHIFT (0)
#define FLASH_BL_COM_CFG_PRG_BL_COM_CFG_MASK  (0xffffffff << FLASH_BL_COM_CFG_PRG_BL_COM_CFG_SHIFT)
#define FLASH_BL_COM_CFG_PRG_BL_COM_CFG(n)    ((n) << FLASH_BL_COM_CFG_PRG_BL_COM_CFG_SHIFT)

/* FLASH OEM key register 1 */

#define FLASH_OEMKEYR1_PRG_RESET              0x00000000
#define FLASH_OEMKEYR1_PRG_OEMKEY_SHIFT       (0)
#define FLASH_OEMKEYR1_PRG_OEMKEY_MASK        (0xffffffff << FLASH_OEMKEYR1_PRG_OEMKEY_SHIFT)
#define FLASH_OEMKEYR1_PRG_OEMKEY(n)          ((n) << FLASH_OEMKEYR1_PRG_OEMKEY_SHIFT)

/* FLASH OEM key register 2 */

#define FLASH_OEMKEYR2_PRG_RESET              0x00000000
#define FLASH_OEMKEYR2_PRG_OEMKEY_SHIFT       (0)
#define FLASH_OEMKEYR2_PRG_OEMKEY_MASK        (0xffffffff << FLASH_OEMKEYR2_PRG_OEMKEY_SHIFT)
#define FLASH_OEMKEYR2_PRG_OEMKEY(n)          ((n) << FLASH_OEMKEYR2_PRG_OEMKEY_SHIFT)

/* FLASH OEM key register 3 */

#define FLASH_OEMKEYR3_PRG_RESET              0x00000000
#define FLASH_OEMKEYR3_PRG_OEMKEY_SHIFT       (0)
#define FLASH_OEMKEYR3_PRG_OEMKEY_MASK        (0xffffffff << FLASH_OEMKEYR3_PRG_OEMKEY_SHIFT)
#define FLASH_OEMKEYR3_PRG_OEMKEY(n)          ((n) << FLASH_OEMKEYR3_PRG_OEMKEY_SHIFT)

/* FLASH OEM key register 4 */

#define FLASH_OEMKEYR4_PRG_RESET              0x00000000
#define FLASH_OEMKEYR4_PRG_OEMKEY_SHIFT       (0)
#define FLASH_OEMKEYR4_PRG_OEMKEY_MASK        (0xffffffff << FLASH_OEMKEYR4_PRG_OEMKEY_SHIFT)
#define FLASH_OEMKEYR4_PRG_OEMKEY(n)          ((n) << FLASH_OEMKEYR4_PRG_OEMKEY_SHIFT)

/* FLASH boundary scan key register */

#define FLASH_BSKEYR_PRG_RESET                0x00000000
#define FLASH_BSKEYR_PRG_BSKEY_SHIFT          (0)
#define FLASH_BSKEYR_PRG_BSKEY_MASK           (0xffffffff << FLASH_BSKEYR_PRG_BSKEY_SHIFT)
#define FLASH_BSKEYR_PRG_BSKEY(n)             ((n) << FLASH_BSKEYR_PRG_BSKEY_SHIFT)

/* FLASH write page protection for bank1 */

#define FLASH_WRP1R_CUR_RESET                 0x00000000
#define FLASH_WRP1R_CUR_WRPSG1_SHIFT          (0)
#define FLASH_WRP1R_CUR_WRPSG1_MASK           (0xffffffff << FLASH_WRP1R_CUR_WRPSG1_SHIFT)
#define FLASH_WRP1R_CUR_WRPSG1(n)             ((n) << FLASH_WRP1R_CUR_WRPSG1_SHIFT)

/* FLASH write-page protection for bank1 */

#define FLASH_WRP1R_PRG_RESET                 0x00000000
#define FLASH_WRP1R_PRG_WRPSG1_SHIFT          (0)
#define FLASH_WRP1R_PRG_WRPSG1_MASK           (0xffffffff << FLASH_WRP1R_PRG_WRPSG1_SHIFT)
#define FLASH_WRP1R_PRG_WRPSG1(n)             ((n) << FLASH_WRP1R_PRG_WRPSG1_SHIFT)

/* FLASH HDP bank1 register */

#define FLASH_HDP1R_CUR_RESET                 0x00000000
#define FLASH_HDP1R_CUR_HDP1_STRT_SHIFT       (0)
#define FLASH_HDP1R_CUR_HDP1_STRT_MASK        (0x3f << FLASH_HDP1R_CUR_HDP1_STRT_SHIFT)
#define FLASH_HDP1R_CUR_HDP1_STRT(n)          ((n) << FLASH_HDP1R_CUR_HDP1_STRT_SHIFT)
#define FLASH_HDP1R_CUR_HDP1_END_SHIFT        (16)
#define FLASH_HDP1R_CUR_HDP1_END_MASK         (0x3f << FLASH_HDP1R_CUR_HDP1_END_SHIFT)
#define FLASH_HDP1R_CUR_HDP1_END(n)           ((n) << FLASH_HDP1R_CUR_HDP1_END_SHIFT)

/* FLASH HDP bank1 register */

#define FLASH_HDP1R_PRG_RESET                 0x00000000
#define FLASH_HDP1R_PRG_HDP1_STRT_SHIFT       (0)
#define FLASH_HDP1R_PRG_HDP1_STRT_MASK        (0x3f << FLASH_HDP1R_PRG_HDP1_STRT_SHIFT)
#define FLASH_HDP1R_PRG_HDP1_STRT(n)          ((n) << FLASH_HDP1R_PRG_HDP1_STRT_SHIFT)
#define FLASH_HDP1R_PRG_HDP1_END_SHIFT        (16)
#define FLASH_HDP1R_PRG_HDP1_END_MASK         (0x3f << FLASH_HDP1R_PRG_HDP1_END_SHIFT)
#define FLASH_HDP1R_PRG_HDP1_END(n)           ((n) << FLASH_HDP1R_PRG_HDP1_END_SHIFT)

/* FLASH ECC correction register */

#define FLASH_ECCCORR_RESET                   0x00000000
#define FLASH_ECCCORR_ADDR_ECC_SHIFT          (0)
#define FLASH_ECCCORR_ADDR_ECC_MASK           (0x3fff << FLASH_ECCCORR_ADDR_ECC_SHIFT)
#define FLASH_ECCCORR_ADDR_ECC(n)             ((n) << FLASH_ECCCORR_ADDR_ECC_SHIFT)
#define FLASH_ECCCORR_EDATA_ECC               (1 << 21)
#define FLASH_ECCCORR_BK_ECC                  (1 << 22)
#define FLASH_ECCCORR_SYSF_ECC                (1 << 23)
#define FLASH_ECCCORR_OTP_ECC                 (1 << 24)
#define FLASH_ECCCORR_ECCCIE                  (1 << 25)
#define FLASH_ECCCORR_ECCC                    (1 << 30)

/* FLASH ECC detection register */

#define FLASH_ECCDETR_RESET                   0x00000000
#define FLASH_ECCDETR_ADDR_ECC_SHIFT          (0)
#define FLASH_ECCDETR_ADDR_ECC_MASK           (0x3fff << FLASH_ECCDETR_ADDR_ECC_SHIFT)
#define FLASH_ECCDETR_ADDR_ECC(n)             ((n) << FLASH_ECCDETR_ADDR_ECC_SHIFT)
#define FLASH_ECCDETR_EDATA_ECC               (1 << 21)
#define FLASH_ECCDETR_BK_ECC                  (1 << 22)
#define FLASH_ECCDETR_SYSF_ECC                (1 << 23)
#define FLASH_ECCDETR_OTP_ECC                 (1 << 24)
#define FLASH_ECCDETR_ECCD                    (1 << 31)

/* FLASH ECC data */

#define FLASH_ECCDR_RESET                     0x00000000
#define FLASH_ECCDR_DATA_ECC_SHIFT            (0)
#define FLASH_ECCDR_DATA_ECC_MASK             (0xffff << FLASH_ECCDR_DATA_ECC_SHIFT)
#define FLASH_ECCDR_DATA_ECC(n)               ((n) << FLASH_ECCDR_DATA_ECC_SHIFT)
#define FLASH_ECCDR_DATA_ADDR_ECC_SHIFT       (16)
#define FLASH_ECCDR_DATA_ADDR_ECC_MASK        (0x7 << FLASH_ECCDR_DATA_ADDR_ECC_SHIFT)
#define FLASH_ECCDR_DATA_ADDR_ECC(n)          ((n) << FLASH_ECCDR_DATA_ADDR_ECC_SHIFT)

/* FLASH write page protection for bank2 */

#define FLASH_WRP2R_CUR_RESET                 0x00000000
#define FLASH_WRP2R_CUR_WRPSG2_SHIFT          (0)
#define FLASH_WRP2R_CUR_WRPSG2_MASK           (0xffffffff << FLASH_WRP2R_CUR_WRPSG2_SHIFT)
#define FLASH_WRP2R_CUR_WRPSG2(n)             ((n) << FLASH_WRP2R_CUR_WRPSG2_SHIFT)

/* FLASH write page protection for bank2 */

#define FLASH_WRP2R_PRG_RESET                 0x00000000
#define FLASH_WRP2R_PRG_WRPSG2_SHIFT          (0)
#define FLASH_WRP2R_PRG_WRPSG2_MASK           (0xffffffff << FLASH_WRP2R_PRG_WRPSG2_SHIFT)
#define FLASH_WRP2R_PRG_WRPSG2(n)             ((n) << FLASH_WRP2R_PRG_WRPSG2_SHIFT)

/* FLASH HDP bank2 register */

#define FLASH_HDP2R_CUR_RESET                 0x00000000
#define FLASH_HDP2R_CUR_HDP2_STRT_SHIFT       (0)
#define FLASH_HDP2R_CUR_HDP2_STRT_MASK        (0x3f << FLASH_HDP2R_CUR_HDP2_STRT_SHIFT)
#define FLASH_HDP2R_CUR_HDP2_STRT(n)          ((n) << FLASH_HDP2R_CUR_HDP2_STRT_SHIFT)
#define FLASH_HDP2R_CUR_HDP2_END_SHIFT        (16)
#define FLASH_HDP2R_CUR_HDP2_END_MASK         (0x3f << FLASH_HDP2R_CUR_HDP2_END_SHIFT)
#define FLASH_HDP2R_CUR_HDP2_END(n)           ((n) << FLASH_HDP2R_CUR_HDP2_END_SHIFT)

/* FLASH HDP bank2 register */

#define FLASH_HDP2R_PRG_RESET                 0x00000000
#define FLASH_HDP2R_PRG_HDP2_STRT_SHIFT       (0)
#define FLASH_HDP2R_PRG_HDP2_STRT_MASK        (0x3f << FLASH_HDP2R_PRG_HDP2_STRT_SHIFT)
#define FLASH_HDP2R_PRG_HDP2_STRT(n)          ((n) << FLASH_HDP2R_PRG_HDP2_STRT_SHIFT)
#define FLASH_HDP2R_PRG_HDP2_END_SHIFT        (16)
#define FLASH_HDP2R_PRG_HDP2_END_MASK         (0x3f << FLASH_HDP2R_PRG_HDP2_END_SHIFT)
#define FLASH_HDP2R_PRG_HDP2_END(n)           ((n) << FLASH_HDP2R_PRG_HDP2_END_SHIFT)
#endif /* __ARCH_ARM_SRC_STM32C5_HARDWARE_STM32C5XX_FLASH_H */
