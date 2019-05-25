/********************************************************************************************
 * arch/arm/src/imxrt/imxrt_src.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_SRC_H
#define __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_SRC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include "chip/imxrt_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register offsets *************************************************************************/

#define IMXRT_SRC_SCR_OFFSET              0x0000  /* SRC Control Register */
#define IMXRT_SRC_SBMR1_OFFSET            0x0004  /* SRC Boot Mode Register 1 */
#define IMXRT_SRC_SRSR_OFFSET             0x0008  /* SRC Reset Status Register */
#define IMXRT_SRC_SBMR2_OFFSET            0x001c  /* SRC Boot Mode Register 2 */
#define IMXRT_SRC_GPR1_OFFSET             0x0020  /* SRC General Purpose Register 1 */
#define IMXRT_SRC_GPR2_OFFSET             0x0024  /* SRC General Purpose Register 2 */
#define IMXRT_SRC_GPR3_OFFSET             0x0028  /* SRC General Purpose Register 3 */
#define IMXRT_SRC_GPR4_OFFSET             0x002c  /* SRC General Purpose Register 4 */
#define IMXRT_SRC_GPR5_OFFSET             0x0030  /* SRC General Purpose Register 5 */
#define IMXRT_SRC_GPR6_OFFSET             0x0034  /* SRC General Purpose Register 6 */
#define IMXRT_SRC_GPR7_OFFSET             0x0038  /* SRC General Purpose Register 7 */
#define IMXRT_SRC_GPR8_OFFSET             0x003c  /* SRC General Purpose Register 8 */
#define IMXRT_SRC_GPR9_OFFSET             0x0040  /* SRC General Purpose Register 9 */
#define IMXRT_SRC_GPR10_OFFSET            0x0044  /* SRC General Purpose Register 10 */

/* Register addresses ***********************************************************************/

#define IMXRT_SRC_SCR                     (IMXRT_SRC_BASE + IMXRT_SRC_SCR_OFFSET)
#define IMXRT_SRC_SBMR1                   (IMXRT_SRC_BASE + IMXRT_SRC_SBMR1_OFFSET)
#define IMXRT_SRC_SRSR                    (IMXRT_SRC_BASE + IMXRT_SRC_SRSR_OFFSET)
#define IMXRT_SRC_SBMR2                   (IMXRT_SRC_BASE + IMXRT_SRC_SBMR2_OFFSET)
#define IMXRT_SRC_GPR1                    (IMXRT_SRC_BASE + IMXRT_SRC_GPR1_OFFSET)
#define IMXRT_SRC_GPR2                    (IMXRT_SRC_BASE + IMXRT_SRC_GPR2_OFFSET)
#define IMXRT_SRC_GPR3                    (IMXRT_SRC_BASE + IMXRT_SRC_GPR3_OFFSET)
#define IMXRT_SRC_GPR4                    (IMXRT_SRC_BASE + IMXRT_SRC_GPR4_OFFSET)
#define IMXRT_SRC_GPR5                    (IMXRT_SRC_BASE + IMXRT_SRC_GPR5_OFFSET)
#define IMXRT_SRC_GPR6                    (IMXRT_SRC_BASE + IMXRT_SRC_GPR6_OFFSET)
#define IMXRT_SRC_GPR7                    (IMXRT_SRC_BASE + IMXRT_SRC_GPR7_OFFSET)
#define IMXRT_SRC_GPR8                    (IMXRT_SRC_BASE + IMXRT_SRC_GPR8_OFFSET)
#define IMXRT_SRC_GPR9                    (IMXRT_SRC_BASE + IMXRT_SRC_GPR9_OFFSET)
#define IMXRT_SRC_GPR10                   (IMXRT_SRC_BASE + IMXRT_SRC_GPR10_OFFSET)

/* Register bit definitions *****************************************************************/

/* SRC Control Register */

                                                    /* Bits 0-3: Reserved */
#define SRC_SCR_LOCKUP_RST                (1 << 4)  /* Bit 4: Lockup reset enable bit */
                                                    /* Bits 5-6: Reserved */
#define SRC_SCR_MASK_WDOG_RST_SHIFT       (7)       /* Bits 7-10: Mask wdog_rst_b source */
#define SRC_SCR_MASK_WDOG_RST_MASK        (15 << SRC_SCR_MASK_WDOG_RST_SHIFT)
#  define SRC_SCR_MASK_WDOG_RST_MASKED    (5 << SRC_SCR_MASK_WDOG_RST_SHIFT)
#  define SRC_SCR_MASK_WDOG_RST_UNMASKED  (10 << SRC_SCR_MASK_WDOG_RST_SHIFT)
                                                    /* Bits 11-12: Reserved */
#define SRC_SCR_CORE0_RST                 (1 << 13) /* Bit 13: Software reset for core0 only. */
                                                    /* Bits 14-16: Reserved */
#define SRC_SCR_CORE0_DBG_RST             (1 << 17) /* Bit 17: Software reset for core0 debug only */
                                                    /* Bits 18-24: Reserved */
#define SRC_SCR_DBG_RST_MSK_PG            (1 << 25) /* Bit 25: Do not assert debug resets
                                                     * after power gating event of core */
                                                    /* Bits 26-27: Reserved */
#define SRC_SCR_MASK_WDOG3_RST_SHIFT      (28)      /* Bits 38-31: Mask wdog3_rst_b source */
#define SRC_SCR_MASK_WDOG3_RST_MASK       (15 << SRC_SCR_MASK_WDOG3_RST_SHIFT)
#  define SRC_SCR_MASK_WDOG3_RST_MASKED   (5 << SRC_SCR_MASK_WDOG3_RST_SHIFT)
#  define SRC_SCR_MASK_WDOG3_RST_UNMASKED (10 << SRC_SCR_MASK_WDOG3_RST_SHIFT)

/* SRC Boot Mode Register 1 */

#define SRC_SBMR1_BOOT_CFG_SHIFT          (24)      /* Bits 24-31: Refer to fusemap */
#define SRC_SBMR1_BOOT_CFG_MASK           (0xff << SRC_SBMR1_BOOT_CFG_SHIFT)
#define SRC_SBMR1_BOOT_CFG2_SHIFT         (16)      /* Bits 16-23: Refer to fusemap */
#define SRC_SBMR1_BOOT_CFG2_MASK          (0xff << SRC_SBMR1_BOOT_CFG2_SHIFT)
#define SRC_SBMR1_BOOT_CFG3_SHIFT         (8)       /* Bits 8-15:  Refer to fusemap */
#define SRC_SBMR1_BOOT_CFG3_MASK          (0xff << SRC_SBMR1_BOOT_CFG3_SHIFT)
#define SRC_SBMR1_BOOT_CFG4_SHIFT         (0)       /* Bits 0-7:   Refer to fusemap */
#define SRC_SBMR1_BOOT_CFG4_MASK          (0xff << SRC_SBMR1_BOOT_CFG4_SHIFT)

/* SRC Reset Status Register */

#define SRC_SRSR_IPP_RESET_B              (1 << 0)  /* Bit 0:  Indicates whether reset was the
                                                     * result of ipp_reset_b pin (Power-up
                                                     * sequence) */
#define SRC_SRSR_LOCKUP_SYSRESETREQ       (1 << 1)  /* Bit 1:  Indicates a reset has been
                                                     * caused by CPU lockup or software setting
                                                     * of SYSRESETREQ bit */
#define SRC_SRSR_CSU_RESET_B              (1 << 2)  /* Bit 2:  Indicates whether the reset was
                                                     * the result of the csu_reset_b input */
#define SRC_SRSR_IPP_USER_RESET_B         (1 << 3)  /* Bit 3:  Indicates whether the reset was
                                                     * the result of the ipp_user_reset_b qualified
                                                     * reset */
#define SRC_SRSR_WDOG_RST_B               (1 << 4)  /* Bit 4:  IC Watchdog Time-out reset */
#define SRC_SRSR_JTAG_RST_B               (1 << 5)  /* Bit 5:  HIGH - Z JTAG reset */
#define SRC_SRSR_JTAG_SW_RST              (1 << 6)  /* Bit 6:  JTAG software reset */
#define SRC_SRSR_WDOG3_RST_B              (1 << 7)  /* Bit 7:  IC Watchdog3 Time-out reset */
#define SRC_SRSR_TEMPSENSE_RST_B          (1 << 8)  /* Bit 8:  Temper Sensor software reset */
                                                    /* Bits 9-31: Reserved */

/* SRC Boot Mode Register 2 */

#define SRC_SBMR2_SEC_CONFIG_SHIFT        (0)       /* Bits 0-1:  State of the corresponding
                                                     * SECONFIG fuse */
                                                    /* Bit 2:  Reserved */
#define SRC_SBMR2_DIR_BT_DIS              (1 << 3)  /* Bit 3:  State of the DIR_BT_DIS fuse */
#define SRC_SBMR2_BT_FUSE_SEL             (1 << 4)  /* Bit 4:  State of the BT_FUSE_SEL fuse */
                                                    /* Bits 5-23: Reserved */
#define SRC_SBMR2_BMOD_SHIFT              (24)      /* Bits 24-25: Latched state of the BOOT_MODE
                                                     *  and BOOT_MODE0 signals on POR.
                                                    /* Bits 26-31: Reserved */

/* SRC General Purpose Register 1 (32-bit values, some have reserved bits)
 * NOTE:  Ald GPR registers are used by the ROM code and should not be used by application
 * software.
 */

#endif /* __ARCH_ARM_SRC_IMXRT_CHIP_IMXRT_SRC_H */

