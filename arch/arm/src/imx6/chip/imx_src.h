/****************************************************************************************************
 * arch/arm/src/imx6/imx_src.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   "i.MX 6Dual/6Quad ApplicationsProcessor Reference Manual," Document Number
 *   IMX6DQRM, Rev. 3, 07/2015, FreeScale.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_IMX6_CHIP_IMX_SRC_H
#define __ARCH_ARM_SRC_IMX6_CHIP_IMX_SRC_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <chip/imx_memorymap.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* SRC Register Offsets *****************************************************************************/

#define IMX_SRC_SCR_OFFSET         0x0000 /* SRC Control Register */
#define IMX_SRC_SBMR1_OFFSET       0x0004 /* SRC Boot Mode Register 1 */
#define IMX_SRC_SRSR_OFFSET        0x0008 /* SRC Reset Status Register */
#define IMX_SRC_SISR_OFFSET        0x0014 /* SRC Interrupt Status Register */
#define IMX_SRC_SIMR_OFFSET        0x0018 /* SRC Interrupt Mask Register */
#define IMX_SRC_SBMR2_OFFSET       0x001c /* SRC Boot Mode Register 2 */
#define IMX_SRC_GPR1_OFFSET        0x0020 /* SRC General Purpose Register 1 */
#define IMX_SRC_GPR2_OFFSET        0x0024 /* SRC General Purpose Register 2 */
#define IMX_SRC_GPR3_OFFSET        0x0028 /* SRC General Purpose Register 3 */
#define IMX_SRC_GPR4_OFFSET        0x002c /* SRC General Purpose Register 4 */
#define IMX_SRC_GPR5_OFFSET        0x0030 /* SRC General Purpose Register 5 */
#define IMX_SRC_GPR6_OFFSET        0x0034 /* SRC General Purpose Register 6 */
#define IMX_SRC_GPR7_OFFSET        0x0038 /* SRC General Purpose Register 7 */
#define IMX_SRC_GPR8_OFFSET        0x003c /* SRC General Purpose Register 8 */
#define IMX_SRC_GPR9_OFFSET        0x0040 /* SRC General Purpose Register 9 */
#define IMX_SRC_GPR10_OFFSET       0x0044 /* SRC General Purpose Register 10 */

/* SRC Register Addresses ***************************************************************************/

#define IMX_SRC_SCR                (IMX_SRC_VBASE+IMX_SRC_SCR_OFFSET)
#define IMX_SRC_SBMR1              (IMX_SRC_VBASE+IMX_SRC_SBMR1_OFFSET)
#define IMX_SRC_SRSR               (IMX_SRC_VBASE+IMX_SRC_SRSR_OFFSET)
#define IMX_SRC_SISR               (IMX_SRC_VBASE+IMX_SRC_SISR_OFFSET)
#define IMX_SRC_SIMR               (IMX_SRC_VBASE+IMX_SRC_SIMR_OFFSET)
#define IMX_SRC_SBMR2              (IMX_SRC_VBASE+IMX_SRC_SBMR2_OFFSET)
#define IMX_SRC_GPR1               (IMX_SRC_VBASE+IMX_SRC_GPR1_OFFSET)
#define IMX_SRC_GPR2               (IMX_SRC_VBASE+IMX_SRC_GPR2_OFFSET)
#define IMX_SRC_GPR3               (IMX_SRC_VBASE+IMX_SRC_GPR3_OFFSET)
#define IMX_SRC_GPR4               (IMX_SRC_VBASE+IMX_SRC_GPR4_OFFSET)
#define IMX_SRC_GPR5               (IMX_SRC_VBASE+IMX_SRC_GPR5_OFFSET)
#define IMX_SRC_GPR6               (IMX_SRC_VBASE+IMX_SRC_GPR6_OFFSET)
#define IMX_SRC_GPR7               (IMX_SRC_VBASE+IMX_SRC_GPR7_OFFSET)
#define IMX_SRC_GPR8               (IMX_SRC_VBASE+IMX_SRC_GPR8_OFFSET)
#define IMX_SRC_GPR9               (IMX_SRC_VBASE+IMX_SRC_GPR9_OFFSET)
#define IMX_SRC_GPR10              (IMX_SRC_VBASE+IMX_SRC_GPR10_OFFSET)

/* SRC Register Bit Definitions *********************************************************************/

/* SRC Control Register: Reset value 0x00000521 */

#define SRC_SCR_WARM_RESET_ENABLE  (1 << 0)  /* Bit 0:  WARM reset enable bit */
#define SRC_SCR_SW_GPU_RST         (1 << 1)  /* Bit 1:  Software reset for GPU */
#define SRC_SCR_SW_VPU_RST         (1 << 2)  /* Bit 2:  Software reset for VPU */
#define SRC_SCR_SW_IPU1_RST        (1 << 3)  /* Bit 3:  Software reset for IPU1 */
#define SRC_SCR_SW_OPEN_VG_RST     (1 << 4)  /* Bit 4:  Software reset for open_vg */
#define SRC_SCR_WARM_RST_BYPASS_COUNT_SHIFT  (5) /* Bits 5-6: XTALI cycles before bypassing the MMDC ack */
#define SRC_SCR_WARM_RST_BYPASS_COUNT_MASK   (3 << SRC_SCR_WARM_RST_BYPASS_COUNT_SHIFT)
#  define SRC_SCR_WARM_RST_BYPASS_COUNT_NONE (0 << SRC_SCR_WARM_RST_BYPASS_COUNT_SHIFT) /* Counter not used */
#  define SRC_SCR_WARM_RST_BYPASS_COUNT_16   (1 << SRC_SCR_WARM_RST_BYPASS_COUNT_SHIFT) /* 16 XTALI cycles before WARM to COLD reset */
#  define SRC_SCR_WARM_RST_BYPASS_COUNT_32   (2 << SRC_SCR_WARM_RST_BYPASS_COUNT_SHIFT) /* 32 XTALI cycles before WARM to COLD reset */
#  define SRC_SCR_WARM_RST_BYPASS_COUNT_64   (3 << SRC_SCR_WARM_RST_BYPASS_COUNT_SHIFT) /* 64 XTALI cycles before WARM to COLD reset */
#define SRC_SCR_MASK_WDOG_RST_SHIFT          (7) /* Bits 7-10: Mask wdog_rst_b source */
#define SRC_SCR_MASK_WDOG_RST_MASK           (15 << SRC_SCR_MASK_WDOG_RST_SHIFT)
#  define SRC_SCR_MASK_WDOG_RST_MASKED       (15 << SRC_SCR_MASK_WDOG_RST_SHIFT) /* wdog_rst_b is masked */
#  define SRC_SCR_MASK_WDOG_RST_UNMASKED     (15 << SRC_SCR_MASK_WDOG_RST_SHIFT) /* wdog_rst_b is not masked */
#define SRC_SCR_EIM_RST            (1 << 11) /* Bit 11: EIM reset is needed in order to reconfigure the eim chip select */
#define SRC_SCR_SW_IPU2_RST        (1 << 12) /* Bit 12: Software reset for ipu2 */
#define SRC_SCR_CORE0_RST          (1 << 13) /* Bit 13: Software reset for core0 */
#define SRC_SCR_CORE1_RST          (1 << 14) /* Bit 14: Software reset for core1 */
#define SRC_SCR_CORE2_RST          (1 << 15) /* Bit 15: Software reset for core2 */
#define SRC_SCR_CORE3_RST          (1 << 16) /* Bit 16: Software reset for core3 */
#define SRC_SCR_CORE0_DBG_RST      (1 << 17) /* Bit 17: Software reset for core0 debug */
#define SRC_SCR_CORE1_DBG_RST      (1 << 18) /* Bit 18: Software reset for core1 debug */
#define SRC_SCR_CORE2_DBG_RST      (1 << 19) /* Bit 19: Software reset for core2 debug */
#define SRC_SCR_CORE3_DBG_RST      (1 << 20) /* Bit 20: Software reset for core3 debug */
#define SRC_SCR_CORES_DBG_RST      (1 << 21) /* Bit 21: Software reset for debug of arm platform */
#define SRC_SCR_CORE1_ENABLE       (1 << 22) /* Bit 22: core1 enable */
#define SRC_SCR_CORE2_ENABLE       (1 << 23) /* Bit 23: core2 enable */
#define SRC_SCR_CORE3_ENABLE       (1 << 24) /* Bit 24: core3 enable */
#define SRC_SCR_DBG_RST_MSK_PG     (1 << 25) /* Bit 25: No debug resets after core power gating event */
                                             /* Bits 26-31: Reserved */

/* SRC Boot Mode Register 1 */

#define SRC_SBMR1_BOOT_CFG1_SHIFT  (0)       /* Bits 0-7: Refer to fusemap */
#define SRC_SBMR1_BOOT_CFG1_MASK   (0xff << SRC_SBMR1_BOOT_CFG1_SHIFT)
#  define SRC_SBMR1_BOOT_CFG1(n)   ((uint32_t)(n) << SRC_SBMR1_BOOT_CFG1_SHIFT)
#define SRC_SBMR1_BOOT_CFG2_SHIFT  (8)       /* Bits 8-15: Refer to fusemap */
#define SRC_SBMR1_BOOT_CFG2_MASK   (0xff << SRC_SBMR1_BOOT_CFG2_SHIFT)
#  define SRC_SBMR1_BOOT_CFG2(n)   ((uint32_t)(n) << SRC_SBMR1_BOOT_CFG2_SHIFT)
#define SRC_SBMR1_BOOT_CFG3_SHIFT  (16)      /* Bits 16-23: Refer to fusemap */
#define SRC_SBMR1_BOOT_CFG3_MASK   (0xff << SRC_SBMR1_BOOT_CFG3_SHIFT)
#  define SRC_SBMR1_BOOT_CFG3(n)   ((uint32_t)(n) << SRC_SBMR1_BOOT_CFG3_SHIFT)
#define SRC_SBMR1_BOOT_CFG4_SHIFT  (24)      /* Bits 24-31: Refer to fusemap */
#define SRC_SBMR1_BOOT_CFG4_MASK   (0xff << SRC_SBMR1_BOOT_CFG4_SHIFT)
#  define SRC_SBMR1_BOOT_CFG4(n)   ((uint32_t)(n) << SRC_SBMR1_BOOT_CFG4_SHIFT)

/* SRC Reset Status Register */

#define SRC_SRSR_IPP_RESET         (1 << 0)  /* Bit 0:  Reset result of ipp_reset_b pin (Power-up sequence) */
                                             /* Bit 1:  Reserved */
#define SRC_SRSR_CSU_RESET         (1 << 2)  /* Bit 2:  Reset result of the csu_reset_b input */
#define SRC_SRSR_IPP_USER_RESET    (1 << 3)  /* Bit 3:  Reset result of ipp_user_reset_b qualified reset */
#define SRC_SRSR_WDOG_RST          (1 << 4)  /* Bit 4:  IC Watchdog Time-out reset */
#define SRC_SRSR_JTAG_RST          (1 << 5)  /* Bit 5:  HIGH - Z JTAG reset */
#define SRC_SRSR_JTAG_SW_RST       (1 << 6)  /* Bit 6:  JTAG software reset */
                                             /* Bits 7-15: Reserved */
#define SRC_SRSR_WARM_BOOT         (1 << 16) /* Bit 16: WARM boot indication shows that WARM boot was initiated by software */
                                             /* Bits 17-31: Reserved */

/* SRC Interrupt Status Register */

#define SRC_SISR_GPU_PASSED_RESET      (1 << 0)  /* Bit 0:  GPU passed software reset and is ready */
#define SRC_SISR_VPU_PASSED_RESET      (1 << 1)  /* Bit 1:  VPU passed software reset and is ready */
#define SRC_SISR_IPU1_PASSED_RESET     (1 << 2)  /* Bit 2:  ipu passed software reset and is ready */
#define SRC_SISR_OPEN_VG_PASSED_RESET  (1 << 3)  /* Bit 3:  open_vg passed software reset and is ready */
#define SRC_SISR_IPU2_PASSED_RESET     (1 << 4)  /* Bit 4:  ipu2 passed software reset and is ready */
#define SRC_SISR_CORE0_WDOG_RST_REQ    (1 << 5)  /* Bit 5:  WDOG reset request from core0 */
#define SRC_SISR_CORE1_WDOG_RST_REQ    (1 << 6)  /* Bit 6:  WDOG reset request from core1 */
#define SRC_SISR_CORE2_WDOG_RST_REQ    (1 << 7)  /* Bit 7:  WDOG reset request from core2 */
#define SRC_SISR_CORE3_WDOG_RST_REQ    (1 << 8)  /* Bit 8:  WDOG reset request from core3 */
                                                 /* Bits 9-31: Reserved */

/* SRC Interrupt Mask Register */
#define SRC_SIMR_

#define SRC_SIMR_GPU_PASSED_RESET      (1 << 0)  /* Bit 0:  Mask GPU passed software reset interrupt */
#define SRC_SIMR_VPU_PASSED_RESET      (1 << 1)  /* Bit 1:  Mask VPU passed software reset interrupt */
#define SRC_SIMR_IPU1_PASSED_RESET     (1 << 2)  /* Bit 2:  Mask ipu passed software reset interrupt */
#define SRC_SIMR_OPEN_VG_PASSED_RESET  (1 << 3)  /* Bit 3:  Mask open_vg passed software reset interrupt */
#define SRC_SIMR_IPU2_PASSED_RESET     (1 << 4)  /* Bit 4:  Mask ipu2 passed software reset interrupt */
                                                 /* Bits 5-31: Reserved */

/* SRC Boot Mode Register 2 */

#define SRC_SBMR2_SEC_CONFIG_SHIFT (0)       /* Bits 0-1: State of the SECONFIG fuses */
                                             /* Bit 2: Reserved */
#define SRC_SBMR2_DIR_BT_DIS       (1 << 3)  /* Bit 3:  State of the DIR_BT_DIS fuse */
#define SRC_SBMR2_BT_FUSE_SEL      (1 << 4)  /* Bit 4: State of the BT_FUSE_SEL fuse */
                                             /* Bits 5-23: Reserved */
#define SRC_SBMR2_BMOD_SHIFT       (24)      /* Bits 24-25: Latched state of the BOOT_MODE1 and BOOT_MODE0 */
#define SRC_SBMR2_BMOD_MASK        (3 << SRC_SBMR2_BMOD_SHIFT)
                                             /* Bits 26-31: Reserved */

/* SRC General Purpose Register 1:  32-bit PERSISTENT_ENTRY0: core0 entry function for waking-up from low power mode */
/* SRC General Purpose Register 2:  32-bit PERSISTENT_ARG0:  core0 entry function argument */
/* SRC General Purpose Register 3:  32-bit PERSISTENT_ENTRY1: core1 entry function for waking-up from low power mode */
/* SRC General Purpose Register 4:  32-bit PERSISTENT_ARG1:  core1 entry function argument */
/* SRC General Purpose Register 5:  32-bit PERSISTENT_ENTRY2: core2 entry function for waking-up from low power mode */
/* SRC General Purpose Register 6:  32-bit PERSISTENT_ARG2:  core1 entry function argument */
/* SRC General Purpose Register 7:  32-bit PERSISTENT_ENTRY3: core3 entry function for waking-up from low power mode */
/* SRC General Purpose Register 8:  32-bit PERSISTENT_ARG3:  core3 entry function argument */
/* SRC General Purpose Register 9:  Reserved */

/* SRC General Purpose Register 10 */

#define SRC_GPR10_RW1_SHIFT          (0)       /* Bits 0-24: General purpose R/W bits */
#define SRC_GPR10_RW1_MASK           (0x01ffffff << SRC_GPR10_RW1_SHIFT)
#  define SRC_GPR10_RW1(n)           ((uint32_t)(n) << SRC_GPR10_RW1_SHIFT)
#define SRC_GPR10_CORE1_ERROR_STATUS (1 << 25) /* Bit 25: core1 error status bit */
#define SRC_GPR10_CORE2_ERROR_STATUS (1 << 26) /* Bit 26: core2 error status bit */
#define SRC_GPR10_CORE3_ERROR_STATUS (1 << 27) /* Bit 27: core3 error status bit */
#define SRC_GPR10_RW2_SHIFT          (28)      /* Bits 28-31: General purpose R/W bits */
#define SRC_GPR10_RW2_MASK           (15 << SRC_GPR10_RW2_SHIFT)
#  define SRC_GPR10_RW2(n)           ((uint32_t)(n) << SRC_GPR10_RW2_SHIFT)

#endif /* __ARCH_ARM_SRC_IMX6_CHIP_IMX_SRC_H */
