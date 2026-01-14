/****************************************************************************
 * arch/arm/src/xmc4/hardware/xmc4_flash.h
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

/****************************************************************************
 * May include some logic from sample code provided by Infineon:
 *
 *   Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use
 * with Infineon's microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
 * SOFTWARE. INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************/

/* Reference: XMC4500 Reference Manual V1.5 2014-07 Microcontrollers. */

#ifndef __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_FLASH_H
#define __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/xmc4_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash memory address */

#define XMC4_FLASH_S0_OFFSET       0X000000
#define XMC4_FLASH_S1_OFFSET       0X004000
#define XMC4_FLASH_S2_OFFSET       0X008000
#define XMC4_FLASH_S3_OFFSET       0X00c000
#define XMC4_FLASH_S4_OFFSET       0X010000
#define XMC4_FLASH_S5_OFFSET       0X014000
#define XMC4_FLASH_S6_OFFSET       0X018000
#define XMC4_FLASH_S7_OFFSET       0X01c000
#define XMC4_FLASH_S8_OFFSET       0X020000
#define XMC4_FLASH_S9_OFFSET       0X040000
#define XMC4_FLASH_S10_OFFSET      0X080000
#define XMC4_FLASH_S11_OFFSET      0X0c0000
#define XMC4_FLASH_S12_OFFSET      0X100000
#define XMC4_FLASH_S13_OFFSET      0X140000
#define XMC4_FLASH_S14_OFFSET      0X180000
#define XMC4_FLASH_S15_OFFSET      0X1c0000

#define XMC4_FLASH_S0  (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S0_OFFSET)
#define XMC4_FLASH_S1  (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S1_OFFSET)
#define XMC4_FLASH_S2  (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S2_OFFSET)
#define XMC4_FLASH_S3  (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S3_OFFSET)
#define XMC4_FLASH_S4  (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S4_OFFSET)
#define XMC4_FLASH_S5  (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S5_OFFSET)
#define XMC4_FLASH_S6  (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S6_OFFSET)
#define XMC4_FLASH_S7  (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S7_OFFSET)
#define XMC4_FLASH_S8  (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S8_OFFSET)
#define XMC4_FLASH_S9  (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S9_OFFSET)
#define XMC4_FLASH_S10 (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S10_OFFSET)
#define XMC4_FLASH_S11 (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S11_OFFSET)
#define XMC4_FLASH_S12 (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S12_OFFSET)
#define XMC4_FLASH_S13 (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S13_OFFSET)
#define XMC4_FLASH_S14 (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S14_OFFSET)
#define XMC4_FLASH_S15 (XMC4_UNCACHED_PFLASH_BASE + XMC4_FLASH_S15_OFFSET)

/* Register Offsets *********************************************************/

/* PMU Registers -- See ID register */

/* Prefetch Registers -- See PCON register */

/* FLASH Registers */

#define XMC4_FLASH_ID_OFFSET       0x1008    /* Flash Module Identification Register */
#define XMC4_FLASH_FSR_OFFSET      0x1010    /* Flash Status Register */
#define XMC4_FLASH_FCON_OFFSET     0x1014    /* Flash Configuration Register */
#define XMC4_FLASH_MARP_OFFSET     0x1018    /* Flash Margin Control Register PFLASH */
#define XMC4_FLASH_PROCON0_OFFSET  0x1020    /* Flash Protection Configuration User 0 */
#define XMC4_FLASH_PROCON1_OFFSET  0x1024    /* Flash Protection Configuration User 1 */
#define XMC4_FLASH_PROCON2_OFFSET  0x1028    /* Flash Protection Configuration User 2 */

/* Register Addresses *******************************************************/

/* FLASH Registers */

#define XMC4_FLASH_ID               (XMC4_FLASH0_BASE+XMC4_FLASH_ID_OFFSET)
#define XMC4_FLASH_FSR              (XMC4_FLASH0_BASE+XMC4_FLASH_FSR_OFFSET)
#define XMC4_FLASH_FCON             (XMC4_FLASH0_BASE+XMC4_FLASH_FCON_OFFSET)
#define XMC4_FLASH_MARP             (XMC4_FLASH0_BASE+XMC4_FLASH_MARP_OFFSET)
#define XMC4_FLASH_PROCON0          (XMC4_FLASH0_BASE+XMC4_FLASH_PROCON0_OFFSET)
#define XMC4_FLASH_PROCON1          (XMC4_FLASH0_BASE+XMC4_FLASH_PROCON1_OFFSET)
#define XMC4_FLASH_PROCON2          (XMC4_FLASH0_BASE+XMC4_FLASH_PROCON2_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* FLASH Registers */

/* Flash Module Identification Register */

#define FLASH_ID_MOD_REV_SHIFT      (0)       /* Bits 0-7: Module Revision Number */
#define FLASH_ID_MOD_REV_MASK       (0xff << FLASH_ID_MOD_REV_SHIFT)
#define FLASH_ID_MOD_TYPE_SHIFT     (8)       /* Bits 8-15: Module Type */
#define FLASH_ID_MOD_TYPE_MASK      (0xff << FLASH_ID_MOD_REV_SHIFT)
#define FLASH_ID_MOD_NUMBER_SHIFT   (16)      /* Bits 16-31: Module Number Value */
#define FLASH_ID_MOD_NUMBER_MASK    (0xffff << FLASH_ID_MOD_NUMBER_SHIFT)

/* Flash Status Register */

#define FLASH_FSR_PBUSY             (1 << 0)  /* Bit 0:  Program Flash Busy */
#define FLASH_FSR_FABUSY            (1 << 1)  /* Bit 1:  Flash Array Busy */
#define FLASH_FSR_PROG              (1 << 4)  /* Bit 4:  Programming State */
#define FLASH_FSR_ERASE             (1 << 5)  /* Bit 5:  Erase State */
#define FLASH_FSR_PFPAGE            (1 << 6)  /* Bit 6:  Program Flash in Page Mode */
#define FLASH_FSR_PFOPER            (1 << 8)  /* Bit 8:  Program Flash Operation Error */
#define FLASH_FSR_SQER              (1 << 10) /* Bit 10: Command Sequence Error */
#define FLASH_FSR_PROER             (1 << 11) /* Bit 11: Protection Error */
#define FLASH_FSR_PFSBER            (1 << 12) /* Bit 12: PFLASH Single-Bit Error and Correction */
#define FLASH_FSR_PFDBER            (1 << 14) /* Bit 14: PFLASH Double-Bit Error */
#define FLASH_FSR_PROIN             (1 << 16) /* Bit 16: Protection Installed */
#define FLASH_FSR_RPROIN            (1 << 18) /* Bit 18: Read Protection Installed */
#define FLASH_FSR_RPRODIS           (1 << 19) /* Bit 19: Read Protection Disable State */
#define FLASH_FSR_WPROIN0           (1 << 21) /* Bit 21: Sector Write Protection Installed for User 0 */
#define FLASH_FSR_WPROIN1           (1 << 22) /* Bit 22: Sector Write Protection Installed for User 1 */
#define FLASH_FSR_WPROIN2           (1 << 23) /* Bit 23: Sector Write Protection Installed for User 2 */
#define FLASH_FSR_WPRODIS0          (1 << 25) /* Bit 25: Sector Write Protection Disabled for User 0 */
#define FLASH_FSR_WPRODIS1          (1 << 26) /* Bit 26: Sector Write Protection Disabled for User 1 */
#define FLASH_FSR_SLM               (1 << 28) /* Bit 28: Flash Sleep Mode */
#define FLASH_FSR_VER               (1 << 31) /* Bit 31: Verify Error */

/* Flash Configuration Register */

#define FLASH_FCON_WSPFLASH_SHIFT   (0)       /* Bits 0-3: Wait States for read access to PFLASH */
#define FLASH_FCON_WSPFLASH_MASK    (15 << FLASH_FCON_WSPFLASH_SHIFT)
#  define FLASH_FCON_WSPFLASH(n)    ((uint32_t)((n)-1) << FLASH_FCON_WSPFLASH_SHIFT)
#define FLASH_FCON_WSECPF           (1 << 4)  /* Bit 4:  Wait State for Error Correction of PFLASH */
#define FLASH_FCON_IDLE             (1 << 13) /* Bit 13: Dynamic Flash Idle */
#define FLASH_FCON_ESLDIS           (1 << 14) /* Bit 14: External Sleep Request Disable */
#define FLASH_FCON_SLEEP            (1 << 15) /* Bit 15: Flash SLEEP */
#define FLASH_FCON_RPA              (1 << 16) /* Bit 16: Read Protection Activated */
#define FLASH_FCON_DCF              (1 << 17) /* Bit 17: Disable Code Fetch from Flash Memory */
#define FLASH_FCON_DDF              (1 << 18) /* Bit 18: Disable Any Data Fetch from Flash */
#define FLASH_FCON_VOPERM           (1 << 24) /* Bit 24: Verify and Operation Error Interrupt Mask */
#define FLASH_FCON_SQERM            (1 << 25) /* Bit 25: Command Sequence Error Interrupt Mask */
#define FLASH_FCON_PROERM           (1 << 26) /* Bit 26: Protection Error Interrupt Mask */
#define FLASH_FCON_PFSBERM          (1 << 27) /* Bit 27: PFLASH Single-Bit Error Interrupt Mask */
#define FLASH_FCON_PFDBERM          (1 << 29) /* Bit 29: PFLASH Double-Bit Error Interrupt Mask */
#define FLASH_FCON_EOBM             (1 << 31) /* Bit 31: End of Busy Interrupt Mask */

/* Flash Margin Control Register PFLASH */

#define FLASH_MARP_MARGIN_SHIFT     (0)       /* Bits 0-3: PFLASH Margin Selection */
#define FLASH_MARP_MARGIN_MASK      (15 << FLASH_MARP_MARGIN_SHIFT)
#define FLASH_MARP_TRAPDIS          (1 << 15) /* Bit 15: PFLASH Double-Bit Error Trap Disable */

/* Flash Protection Configuration User 0 */

#define FLASH_PROCON0_S0L           (1 << 0)  /* Bit 0:  Sector 0 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S1L           (1 << 1)  /* Bit 1:  Sector 1 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S2L           (1 << 2)  /* Bit 2:  Sector 2 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S3L           (1 << 3)  /* Bit 3:  Sector 3 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S4L           (1 << 4)  /* Bit 4:  Sector 4 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S5L           (1 << 5)  /* Bit 5:  Sector 5 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S6L           (1 << 6)  /* Bit 6:  Sector 6 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S7L           (1 << 7)  /* Bit 7:  Sector 7 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S8L           (1 << 8)  /* Bit 8:  Sector 8 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S9L           (1 << 9)  /* Bit 9:  Sector 9 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S10_S11L      (1 << 10) /* Bit 10: Sectors 10 and 11 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S12_S13L      (1 << 11) /* Bit 11: Sectors 12 and 13 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_S14_S15L      (1 << 12) /* Bit 12: Sectors 14 and 15 Locked for Write Protection by User 0 */
#define FLASH_PROCON0_RPRO          (1 << 15) /* Bit 15: Read Protection Configuration */

/* Flash Protection Configuration User 1 */

#define FLASH_PROCON1_S0L           (1 << 0)  /* Bit 0:  Sector 0 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S1L           (1 << 1)  /* Bit 1:  Sector 1 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S2L           (1 << 2)  /* Bit 2:  Sector 2 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S3L           (1 << 3)  /* Bit 3:  Sector 3 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S4L           (1 << 4)  /* Bit 4:  Sector 4 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S5L           (1 << 5)  /* Bit 5:  Sector 5 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S6L           (1 << 6)  /* Bit 6:  Sector 6 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S7L           (1 << 7)  /* Bit 7:  Sector 7 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S8L           (1 << 8)  /* Bit 8:  Sector 8 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S9L           (1 << 9)  /* Bit 9:  Sector 9 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S10_S11L      (1 << 10) /* Bit 10: Sectors 10 and 11 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S12_S13L      (1 << 11) /* Bit 11: Sectors 12 and 13 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_S14_S15L      (1 << 12) /* Bit 12: Sectors 14 and 15 Locked for Write Protection by User 1 */
#define FLASH_PROCON1_PSR           (1 << 16) /* Bit 16:  */

/* Flash Protection Configuration User 2 */

#define FLASH_PROCON2_S0ROM         (1 << 0)  /* Bit 0:  Sector 0 Locked Forever by User 2 */
#define FLASH_PROCON2_S1ROM         (1 << 1)  /* Bit 1:  Sector 1 Locked Forever by User 2 */
#define FLASH_PROCON2_S2ROM         (1 << 2)  /* Bit 2:  Sector 2 Locked Forever by User 2 */
#define FLASH_PROCON2_S3ROM         (1 << 3)  /* Bit 3:  Sector 3 Locked Forever by User 2 */
#define FLASH_PROCON2_S4ROM         (1 << 4)  /* Bit 4:  Sector 4 Locked Forever by User 2 */
#define FLASH_PROCON2_S5ROM         (1 << 5)  /* Bit 5:  Sector 5 Locked Forever by User 2 */
#define FLASH_PROCON2_S6ROM         (1 << 6)  /* Bit 6:  Sector 6 Locked Forever by User 2 */
#define FLASH_PROCON2_S7ROM         (1 << 7)  /* Bit 7:  Sector 7 Locked Forever by User 2 */
#define FLASH_PROCON2_S8ROM         (1 << 8)  /* Bit 8:  Sector 8 Locked Forever by User 2 */
#define FLASH_PROCON2_S9ROM         (1 << 9)  /* Bit 9:  Sector 9 Locked Forever by User 2 */
#define FLASH_PROCON2_S10_S11ROM    (1 << 10) /* Bit 10: Sectors 10 and 11 Locked Forever by User 2 */
#define FLASH_PROCON2_S12_S13ROM    (1 << 11) /* Bit 11: Sectors 12 and 13 Locked Forever by User 2 */
#define FLASH_PROCON2_S14_S15ROM    (1 << 12) /* Bit 12: Sectors 14 and 15 Locked Forever by User 2 */

#endif /* __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_FLASH_H */
