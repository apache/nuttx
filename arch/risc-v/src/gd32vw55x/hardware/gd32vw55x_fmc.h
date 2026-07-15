/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_fmc.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_FMC_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_FMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash geometry ***********************************************************/

/* The GD32VW55x embeds a system-in-package NOR flash mapped at
 * GD32VW55X_FLASH_BASE (0x08000000).  The FMC erases the main flash array
 * with a uniform page size of 4 KiB and programs it 32 bits at a time.
 * The total size is device dependent (2 MiB on the GD32VW551, 4 MiB on the
 * GD32VW553).
 */

#define GD32VW55X_FLASH_PAGE_SIZE    4096
#define GD32VW55X_FLASH_ERASEDVAL    0xff

/* Register offsets *********************************************************/

#define GD32VW55X_FMC_KEY_OFFSET     0x0004  /* Unlock key register */
#define GD32VW55X_FMC_OBKEY_OFFSET   0x0008  /* Option bytes unlock key */
#define GD32VW55X_FMC_STAT_OFFSET    0x000c  /* Status register */
#define GD32VW55X_FMC_CTL_OFFSET     0x0010  /* Control register */
#define GD32VW55X_FMC_ADDR_OFFSET    0x0014  /* Address register */
#define GD32VW55X_FMC_OBSTAT_OFFSET  0x001c  /* Option byte status register */
#define GD32VW55X_FMC_OBR_OFFSET     0x0040  /* Option byte register */
#define GD32VW55X_FMC_OBUSER_OFFSET  0x0044  /* Option byte user register */
#define GD32VW55X_FMC_OBWRP0_OFFSET  0x0048  /* Write protection area 0 */
#define GD32VW55X_FMC_OBWRP1_OFFSET  0x004c  /* Write protection area 1 */
#define GD32VW55X_FMC_NODEC0_OFFSET  0x0070  /* NO-RTDEC region register 0 */
#define GD32VW55X_FMC_NODEC1_OFFSET  0x0074  /* NO-RTDEC region register 1 */
#define GD32VW55X_FMC_NODEC2_OFFSET  0x0078  /* NO-RTDEC region register 2 */
#define GD32VW55X_FMC_NODEC3_OFFSET  0x007c  /* NO-RTDEC region register 3 */
#define GD32VW55X_FMC_OFRG_OFFSET    0x0080  /* Offset region register */
#define GD32VW55X_FMC_OFVR_OFFSET    0x0084  /* Offset value register */
#define GD32VW55X_FMC_PID0_OFFSET    0x0100  /* Product ID register 0 */
#define GD32VW55X_FMC_PID1_OFFSET    0x0104  /* Product ID register 1 */

/* Register addresses *******************************************************/

#define GD32VW55X_FMC_KEY    (GD32VW55X_FMC_BASE + GD32VW55X_FMC_KEY_OFFSET)
#define GD32VW55X_FMC_OBKEY  (GD32VW55X_FMC_BASE + GD32VW55X_FMC_OBKEY_OFFSET)
#define GD32VW55X_FMC_STAT   (GD32VW55X_FMC_BASE + GD32VW55X_FMC_STAT_OFFSET)
#define GD32VW55X_FMC_CTL    (GD32VW55X_FMC_BASE + GD32VW55X_FMC_CTL_OFFSET)
#define GD32VW55X_FMC_ADDR   (GD32VW55X_FMC_BASE + GD32VW55X_FMC_ADDR_OFFSET)
#define GD32VW55X_FMC_OBSTAT (GD32VW55X_FMC_BASE + GD32VW55X_FMC_OBSTAT_OFFSET)
#define GD32VW55X_FMC_OBR    (GD32VW55X_FMC_BASE + GD32VW55X_FMC_OBR_OFFSET)
#define GD32VW55X_FMC_OBUSER (GD32VW55X_FMC_BASE + GD32VW55X_FMC_OBUSER_OFFSET)
#define GD32VW55X_FMC_OBWRP0 (GD32VW55X_FMC_BASE + GD32VW55X_FMC_OBWRP0_OFFSET)
#define GD32VW55X_FMC_OBWRP1 (GD32VW55X_FMC_BASE + GD32VW55X_FMC_OBWRP1_OFFSET)
#define GD32VW55X_FMC_NODEC0 (GD32VW55X_FMC_BASE + GD32VW55X_FMC_NODEC0_OFFSET)
#define GD32VW55X_FMC_NODEC1 (GD32VW55X_FMC_BASE + GD32VW55X_FMC_NODEC1_OFFSET)
#define GD32VW55X_FMC_NODEC2 (GD32VW55X_FMC_BASE + GD32VW55X_FMC_NODEC2_OFFSET)
#define GD32VW55X_FMC_NODEC3 (GD32VW55X_FMC_BASE + GD32VW55X_FMC_NODEC3_OFFSET)
#define GD32VW55X_FMC_OFRG   (GD32VW55X_FMC_BASE + GD32VW55X_FMC_OFRG_OFFSET)
#define GD32VW55X_FMC_OFVR   (GD32VW55X_FMC_BASE + GD32VW55X_FMC_OFVR_OFFSET)
#define GD32VW55X_FMC_PID0   (GD32VW55X_FMC_BASE + GD32VW55X_FMC_PID0_OFFSET)
#define GD32VW55X_FMC_PID1   (GD32VW55X_FMC_BASE + GD32VW55X_FMC_PID1_OFFSET)

/* Register bit definitions *************************************************/

/* Unlock keys (written in sequence to FMC_KEY / FMC_OBKEY) */

#define FMC_UNLOCK_KEY0              0x45670123
#define FMC_UNLOCK_KEY1              0xcdef89ab

/* Status register */

#define FMC_STAT_BUSY                (1 << 0)  /* Flash busy */
#define FMC_STAT_WPERR               (1 << 4)  /* Erase/program prot. error */
#define FMC_STAT_ENDF                (1 << 5)  /* End of operation */

#define FMC_STAT_ALLERRS             (FMC_STAT_WPERR)

/* Control register */

#define FMC_CTL_PG                   (1 << 0)  /* Main flash program command */
#define FMC_CTL_PER                  (1 << 1)  /* Page erase command */
#define FMC_CTL_MER                  (1 << 2)  /* Mass erase command */
#define FMC_CTL_WTPG                 (1 << 3)  /* Wi-Fi trim program command */
#define FMC_CTL_START                (1 << 6)  /* Send erase command to FMC */
#define FMC_CTL_LK                   (1 << 7)  /* FMC_CTL lock */
#define FMC_CTL_OBWEN                (1 << 9)  /* Option bytes write enable */
#define FMC_CTL_ERRIE                (1 << 10) /* Error interrupt enable */
#define FMC_CTL_ENDIE                (1 << 12) /* End of operation int en */
#define FMC_CTL_OBSTART              (1 << 14) /* Option bytes modif. start */
#define FMC_CTL_OBRLD                (1 << 15) /* Option bytes reload */

/* Option byte status register */

#define FMC_OBSTAT_SPC               (1 << 1)  /* Security protection level 1 */
#define FMC_OBSTAT_WP                (1 << 2)  /* Write/erase protection */

/* Option byte register */

#define FMC_OBR_SPC_SHIFT            (0)       /* Bits 0-7: Protection code */
#define FMC_OBR_SPC_MASK             (0xff << FMC_OBR_SPC_SHIFT)
#  define FMC_OBR_SPC_NONE           (0xaa << FMC_OBR_SPC_SHIFT)
#  define FMC_OBR_SPC_LEVEL1         (0xcc << FMC_OBR_SPC_SHIFT)
#define FMC_OBR_NWDG_HW              (1 << 9)  /* Software watchdog */
#define FMC_OBR_NRST_STDBY           (1 << 10) /* No reset entering standby */
#define FMC_OBR_NRST_DPSLP           (1 << 11) /* No reset entering deepsleep */
#define FMC_OBR_SRAM1_RST            (1 << 12) /* Erase SRAM1 on system reset */

/* Option byte write protection area registers */

#define FMC_OBWRP_SPAGE_SHIFT        (0)       /* Bits 0-9: Start page */
#define FMC_OBWRP_SPAGE_MASK         (0x3ff << FMC_OBWRP_SPAGE_SHIFT)
#define FMC_OBWRP_EPAGE_SHIFT        (16)      /* Bits 16-25: End page */
#define FMC_OBWRP_EPAGE_MASK         (0x3ff << FMC_OBWRP_EPAGE_SHIFT)

/* NO-RTDEC region registers */

#define FMC_NODEC_SPAGE_SHIFT        (0)       /* Bits 0-9: Start page */
#define FMC_NODEC_SPAGE_MASK         (0x3ff << FMC_NODEC_SPAGE_SHIFT)
#define FMC_NODEC_EPAGE_SHIFT        (16)      /* Bits 16-25: End page */
#define FMC_NODEC_EPAGE_MASK         (0x3ff << FMC_NODEC_EPAGE_SHIFT)

/* Offset region and offset value registers */

#define FMC_OFRG_SPAGE_SHIFT         (0)       /* Bits 0-12: Start page */
#define FMC_OFRG_SPAGE_MASK          (0x1fff << FMC_OFRG_SPAGE_SHIFT)
#define FMC_OFRG_EPAGE_SHIFT         (16)      /* Bits 16-28: End page */
#define FMC_OFRG_EPAGE_MASK          (0x1fff << FMC_OFRG_EPAGE_SHIFT)

#define FMC_OFVR_VALUE_SHIFT         (0)       /* Bits 0-12: Offset value */
#define FMC_OFVR_VALUE_MASK          (0x1fff << FMC_OFVR_VALUE_SHIFT)

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_FMC_H */
