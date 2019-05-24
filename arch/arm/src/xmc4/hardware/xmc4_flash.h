/************************************************************************************
 * arch/arm/src/xmc4/hardware/xmc4_flash.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: XMC4500 Reference Manual V1.5 2014-07 Microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * May include some logic from sample code provided by Infineon:
 *
 *   Copyright (C) 2011-2015 Infineon Technologies AG. All rights reserved.
 *
 * Infineon Technologies AG (Infineon) is supplying this software for use with
 * Infineon's microcontrollers.  This file can be freely distributed within
 * development tools that are supporting such microcontrollers.
 *
 * THIS SOFTWARE IS PROVIDED AS IS. NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_FLASH_H
#define __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_FLASH_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "hardware/xmc4_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

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

/* Register Addresses ****************************************************************/

/* FLASH Registers */

#define XMC4_FLASH_ID               (XMC4_FLASH0_BASE+XMC4_FLASH_ID_OFFSET)
#define XMC4_FLASH_FSR              (XMC4_FLASH0_BASE+XMC4_FLASH_FSR_OFFSET)
#define XMC4_FLASH_FCON             (XMC4_FLASH0_BASE+XMC4_FLASH_FCON_OFFSET)
#define XMC4_FLASH_MARP             (XMC4_FLASH0_BASE+XMC4_FLASH_MARP_OFFSET)
#define XMC4_FLASH_PROCON0          (XMC4_FLASH0_BASE+XMC4_FLASH_PROCON0_OFFSET)
#define XMC4_FLASH_PROCON1          (XMC4_FLASH0_BASE+XMC4_FLASH_PROCON1_OFFSET)
#define XMC4_FLASH_PROCON2          (XMC4_FLASH0_BASE+XMC4_FLASH_PROCON2_OFFSET)

/* Register Bit-Field Definitions **************************************************/

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
