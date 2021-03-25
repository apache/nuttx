/****************************************************************************
 * arch/arm/src/tiva/hardware/cc13x0/cc13x0_aon_wuc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a
 * compatible BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_WUC_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_WUC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AON SYSCTL Register Offsets **********************************************/

#define TIVA_AON_WUC_MCUCLK_OFFSET                  0x0000  /* MCU Clock Management */
#define TIVA_AON_WUC_AUXCLK_OFFSET                  0x0004  /* AUX Clock Management */
#define TIVA_AON_WUC_MCUCFG_OFFSET                  0x0008  /* MCU Configuration */
#define TIVA_AON_WUC_AUXCFG_OFFSET                  0x000c  /* AUX Configuration */
#define TIVA_AON_WUC_AUXCTL_OFFSET                  0x0010  /* AUX Control */
#define TIVA_AON_WUC_PWRSTAT_OFFSET                 0x0014  /* Power Status */
#define TIVA_AON_WUC_SHUTDOWN_OFFSET                0x0018  /* Shutdown Control */
#define TIVA_AON_WUC_CTL0_OFFSET                    0x0020  /* Control 0 */
#define TIVA_AON_WUC_CTL1_OFFSET                    0x0024  /* Control 1 */
#define TIVA_AON_WUC_RECHARGECFG_OFFSET             0x0030  /* Recharge Controller Configuration */
#define TIVA_AON_WUC_RECHARGESTAT_OFFSET            0x0034  /* Recharge Controller Status */
#define TIVA_AON_WUC_OSCCFG_OFFSET                  0x0038  /* Oscillator Configuration */
#define TIVA_AON_WUC_JTAGCFG_OFFSET                 0x0040  /* JTAG Configuration */
#define TIVA_AON_WUC_JTAGUSERCODE_OFFSET            0x0044  /* JTAG USERCODE */

/* AON SYSCTL Register Addresses ********************************************/

#define TIVA_AON_WUC_MCUCLK                         (TIVA_AON_WUC_BASE + TIVA_AON_WUC_MCUCLK_OFFSET)
#define TIVA_AON_WUC_AUXCLK                         (TIVA_AON_WUC_BASE + TIVA_AON_WUC_AUXCLK_OFFSET)
#define TIVA_AON_WUC_MCUCFG                         (TIVA_AON_WUC_BASE + TIVA_AON_WUC_MCUCFG_OFFSET)
#define TIVA_AON_WUC_AUXCFG                         (TIVA_AON_WUC_BASE + TIVA_AON_WUC_AUXCFG_OFFSET)
#define TIVA_AON_WUC_AUXCTL                         (TIVA_AON_WUC_BASE + TIVA_AON_WUC_AUXCTL_OFFSET)
#define TIVA_AON_WUC_PWRSTAT                        (TIVA_AON_WUC_BASE + TIVA_AON_WUC_PWRSTAT_OFFSET)
#define TIVA_AON_WUC_SHUTDOWN                       (TIVA_AON_WUC_BASE + TIVA_AON_WUC_SHUTDOWN_OFFSET)
#define TIVA_AON_WUC_CTL0                           (TIVA_AON_WUC_BASE + TIVA_AON_WUC_CTL0_OFFSET)
#define TIVA_AON_WUC_CTL1                           (TIVA_AON_WUC_BASE + TIVA_AON_WUC_CTL1_OFFSET)
#define TIVA_AON_WUC_RECHARGECFG                    (TIVA_AON_WUC_BASE + TIVA_AON_WUC_RECHARGECFG_OFFSET)
#define TIVA_AON_WUC_RECHARGESTAT                   (TIVA_AON_WUC_BASE + TIVA_AON_WUC_RECHARGESTAT_OFFSET)
#define TIVA_AON_WUC_OSCCFG                         (TIVA_AON_WUC_BASE + TIVA_AON_WUC_OSCCFG_OFFSET)
#define TIVA_AON_WUC_JTAGCFG                        (TIVA_AON_WUC_BASE + TIVA_AON_WUC_JTAGCFG_OFFSET)
#define TIVA_AON_WUC_JTAGUSERCODE                   (TIVA_AON_WUC_BASE + TIVA_AON_WUC_JTAGUSERCODE_OFFSET)

/* AON SYSCTL Register Bitfield Definitions *********************************/

/* AON_WUC_MCUCLK */

#define AON_WUC_MCUCLK_PWR_DWN_SRC_SHIFT            (0)       /* Bits 0-1:  Clock source for MCU domain when request powerdown */
#define AON_WUC_MCUCLK_PWR_DWN_SRC_MASK             (3 << AON_WUC_MCUCLK_PWR_DWN_SRC_SHIFT)
#  define AON_WUC_MCUCLK_PWR_DWN_SRC(n)             ((uint32_t)(n) << AON_WUC_MCUCLK_PWR_DWN_SRC_SHIFT)
#  define AON_WUC_MCUCLK_PWR_DWN_SRC_NONE           (0 << AON_WUC_MCUCLK_PWR_DWN_SRC_SHIFT)
#  define AON_WUC_MCUCLK_PWR_DWN_SRC_SCLK_LF        (1 << AON_WUC_MCUCLK_PWR_DWN_SRC_SHIFT)
#define AON_WUC_MCUCLK_RCOSC_HF_CAL_DONE            (1 << 2)  /* Bit 2:  RCOSC_HF is calibrated to 48 MHz */

/* AON_WUC_AUXCLK */

#define AON_WUC_AUXCLK_SRC_SHIFT                    (0)       /* Bits 0-2:  Clock source for AUX */
#define AON_WUC_AUXCLK_SRC_MASK                     (7 << AON_WUC_AUXCLK_SRC_SHIFT)
#  define AON_WUC_AUXCLK_SRC(n)                     ((uint32_t)(n) << AON_WUC_AUXCLK_SRC_SHIFT)
#  define AON_WUC_AUXCLK_SRC_SCLK_HF                (1 << AON_WUC_AUXCLK_SRC_SHIFT)
#  define AON_WUC_AUXCLK_SRC_SCLK_LF                (4 << AON_WUC_AUXCLK_SRC_SHIFT)
#define AON_WUC_AUXCLK_SCLK_HF_DIV_SHIFT            (8)       /* Bits 8-10:  AUX clock divider for SCLK_HF */
#define AON_WUC_AUXCLK_SCLK_HF_DIV_MASK             (7 << AON_WUC_AUXCLK_SCLK_HF_DIV_SHIFT)
#  define AON_WUC_AUXCLK_SCLK_HF_DIV(n)             ((uint32_t)(n) << AON_WUC_AUXCLK_SCLK_HF_DIV_SHIFT)
#  define AON_WUC_AUXCLK_SCLK_HF_DIV_DIV2           (0 << AON_WUC_AUXCLK_SCLK_HF_DIV_SHIFT)
#  define AON_WUC_AUXCLK_SCLK_HF_DIV_DIV4           (1 << AON_WUC_AUXCLK_SCLK_HF_DIV_SHIFT)
#  define AON_WUC_AUXCLK_SCLK_HF_DIV_DIV8           (2 << AON_WUC_AUXCLK_SCLK_HF_DIV_SHIFT)
#  define AON_WUC_AUXCLK_SCLK_HF_DIV_DIV16          (3 << AON_WUC_AUXCLK_SCLK_HF_DIV_SHIFT)
#  define AON_WUC_AUXCLK_SCLK_HF_DIV_DIV32          (4 << AON_WUC_AUXCLK_SCLK_HF_DIV_SHIFT)
#  define AON_WUC_AUXCLK_SCLK_HF_DIV_DIV64          (5 << AON_WUC_AUXCLK_SCLK_HF_DIV_SHIFT)
#  define AON_WUC_AUXCLK_SCLK_HF_DIV_DIV128         (6 << AON_WUC_AUXCLK_SCLK_HF_DIV_SHIFT)
#  define AON_WUC_AUXCLK_SCLK_HF_DIV_DIV256         (7 << AON_WUC_AUXCLK_SCLK_HF_DIV_SHIFT)
#define AON_WUC_AUXCLK_PWR_DWN_SRC_SHIFT            (11)      /* Bits 11-12: Clock source during AUX power down */
#define AON_WUC_AUXCLK_PWR_DWN_SRC_MASK             (3 << AON_WUC_AUXCLK_PWR_DWN_SRC_SHIFT)
#  define AON_WUC_AUXCLK_PWR_DWN_SRC(n)             ((uint32_t)(n) << AON_WUC_AUXCLK_PWR_DWN_SRC_SHIFT)
#  define AON_WUC_AUXCLK_PWR_DWN_SRC_NONE           (0 << AON_WUC_AUXCLK_PWR_DWN_SRC_SHIFT)
#  define AON_WUC_AUXCLK_PWR_DWN_SRC_SCLK_LF        (1 << AON_WUC_AUXCLK_PWR_DWN_SRC_SHIFT)

/* AON_WUC_MCUCFG */

#define AON_WUC_MCUCFG_SRAM_RET_EN_SHIFT            (0)       /* Bits 0-3:  MCU SRAM banks with retention during MCU power off */
#define AON_WUC_MCUCFG_SRAM_RET_EN_MASK             (15 << AON_WUC_MCUCFG_SRAM_RET_EN_SHIFT)
#  define AON_WUC_MCUCFG_SRAM_RET_EN(n)             ((uint32_t)(n) << AON_WUC_MCUCFG_SRAM_RET_EN_SHIFT)
#  define AON_WUC_MCUCFG_SRAM_RET_EN_RET_NONE       (0 << AON_WUC_MCUCFG_SRAM_RET_EN_SHIFT)
#  define AON_WUC_MCUCFG_SRAM_RET_EN_RET_LEVEL1     (1 << AON_WUC_MCUCFG_SRAM_RET_EN_SHIFT)
#  define AON_WUC_MCUCFG_SRAM_RET_EN_RET_LEVEL2     (3 << AON_WUC_MCUCFG_SRAM_RET_EN_SHIFT)
#  define AON_WUC_MCUCFG_SRAM_RET_EN_RET_LEVEL3     (7 << AON_WUC_MCUCFG_SRAM_RET_EN_SHIFT)
#  define AON_WUC_MCUCFG_SRAM_RET_EN_RET_FULL       (15 << AON_WUC_MCUCFG_SRAM_RET_EN_SHIFT)
#define AON_WUC_MCUCFG_FIXED_WU_EN                  (1 << 16) /* Bit 16 */
#define AON_WUC_MCUCFG_VIRT_OFF                     (1 << 17) /* Bit 17 */

/* AON_WUC_AUXCFG */

#define AON_WUC_AUXCFG_RAM_RET_EN                   (1 << 0)  /* Bit 0:  Enables retention mode for AUX_RAM:BANK0 */

/* AON_WUC_AUXCTL */

#define AON_WUC_AUXCTL_AUX_FORCE_ON                 (1 << 0)  /* Bit 0:  Force the AUX domain into active mode */
#define AON_WUC_AUXCTL_SWEV                         (1 << 1)  /* Bit 1:  Sets software event to AUX domain */
#define AON_WUC_AUXCTL_SCE_RUN_EN                   (1 << 2)  /* Bit 2:  Enable AUX_SCE execution */
#define AON_WUC_AUXCTL_RESET_REQ                    (1 << 31) /* Bit 31: Assert reset to AUX */

/* AON_WUC_PWRSTAT */

#define AON_WUC_SHUTDOWN_EN                         (1 << 0)  /* Bit 0:  Force shutdown request */
#define AON_WUC_PWRSTAT_AUX_RESET_DONE              (1 << 1)  /* Bit 1:  AUX reset done */
#define AON_WUC_PWRSTAT_AUX_BUS_CONNECTED           (1 << 2)  /* Bit 2:  AUX bus is connected */
#define AON_WUC_PWRSTAT_MCU_PD_ON                   (1 << 4)  /* Bit 4:  MCU power sequence finalized; MCU_AONIF reliable */
#define AON_WUC_PWRSTAT_AUX_PD_ON                   (1 << 5)  /* Bit 5:  AUX powered on and connected to bus */
#define AON_WUC_PWRSTAT_JTAG_PD_ON                  (1 << 6)  /* Bit 6:  JTAG is powered on */
#define AON_WUC_PWRSTAT_AUX_PWR_DWN                 (1 << 9)  /* Bit 9:  AUX powerdown state when AUX domain is powered up */

/* AON_WUC_CTL0 */

#define AON_WUC_CTL0_MCU_SRAM_ERASE                 (1 << 2)  /* Bit 2 */
#define AON_WUC_CTL0_AUX_SRAM_ERASE                 (1 << 3)  /* Bit 3 */
#define AON_WUC_CTL0_PWR_DWN_DIS                    (1 << 8)  /* Bit 8:  MCU/AUX power off will enable powerdown */

/* AON_WUC_CTL1 */

#define AON_WUC_CTL1_MCU_WARM_RESET                 (1 << 0)  /* Bit  0:  Last MCU reset was a warm reset */
#define AON_WUC_CTL1_MCU_RESET_SRC                  (1 << 1)  /* Bit 1: Source of lst MCU Voltage Domain warm reset request */
#  define AON_WUC_CTL1_MCU_RESET_SRC_SWRESET        (0)
#  define AON_WUC_CTL1_MCU_RESET_SRC_JTAG           AON_WUC_CTL1_MCU_RESET_SRC

/* AON_WUC_RECHARGECFG */

#define AON_WUC_RECHARGECFG_PER_E_SHIFT             (0)       /* Bits 0-2: 32KHz clocks between activation of rechange
                                                               *           controller (exponent) */
#define AON_WUC_RECHARGECFG_PER_E_MASK              (7 << AON_WUC_RECHARGECFG_PER_E_SHIFT)
#  define AON_WUC_RECHARGECFG_PER_E(n)              ((uint32_t)(n) << AON_WUC_RECHARGECFG_PER_E_SHIFT)
#define AON_WUC_RECHARGECFG_PER_M_SHIFT             (3)       /* Bits 3-7: 32KHz clocks between activation of rechange
                                                               *           controller (mantissa) */
#define AON_WUC_RECHARGECFG_PER_M_MASK              (31 << AON_WUC_RECHARGECFG_PER_M_SHIFT)
#  define AON_WUC_RECHARGECFG_PER_M(n)              ((uint32_t)(n) << AON_WUC_RECHARGECFG_PER_M_SHIFT)
#define AON_WUC_RECHARGECFG_MAX_PER_E_SHIFT         (8)       /* Bits 8-10: Maximum recharge period exponent */
#define AON_WUC_RECHARGECFG_MAX_PER_E_MASK          (7 << AON_WUC_RECHARGECFG_MAX_PER_E_SHIFT)
#  define AON_WUC_RECHARGECFG_MAX_PER_E(n)          ((uint32_t)(n) << AON_WUC_RECHARGECFG_MAX_PER_E_SHIFT)
#define AON_WUC_RECHARGECFG_MAX_PER_M_SHIFT         (11)      /* Bits 11-15: Maximum recharge period (mantissa) */
#define AON_WUC_RECHARGECFG_MAX_PER_M_MASK          (31 << AON_WUC_RECHARGECFG_MAX_PER_M_SHIFT)
#  define AON_WUC_RECHARGECFG_MAX_PER_M(n)          ((uint32_t)(n) << AON_WUC_RECHARGECFG_MAX_PER_M_SHIFT)
#define AON_WUC_RECHARGECFG_C1_SHIFT                (16)      /* Bits 16-19: Gain factor for adaptive recharge algorithm */
#define AON_WUC_RECHARGECFG_C1_MASK                 (15 << AON_WUC_RECHARGECFG_C1_SHIFT)
#  define AON_WUC_RECHARGECFG_C1(n)                 ((uint32_t)(n) << AON_WUC_RECHARGECFG_C1_SHIFT)
#define AON_WUC_RECHARGECFG_C2_SHIFT                (20)      /* Bits 20-23: Gain factor for adaptive recharge algorithm */
#define AON_WUC_RECHARGECFG_C2_MASK                 (15 << AON_WUC_RECHARGECFG_C2_SHIFT)
#  define AON_WUC_RECHARGECFG_C2(n)                 ((uint32_t)(n) << AON_WUC_RECHARGECFG_C2_SHIFT)
#define AON_WUC_RECHARGECFG_ADAPTIVE_EN             (1 << 31) /* Bit 31: Enable adaptive recharge */

/* AON_WUC_RECHARGESTAT */

#define AON_WUC_RECHARGESTAT_MAX_USED_PER_SHIFT     (0)       /* Bits 0-15: Maximum value of recharge period with VDDR > thres */
#define AON_WUC_RECHARGESTAT_MAX_USED_PER_MASK      (0xffff << AON_WUC_RECHARGESTAT_MAX_USED_PER_SHIFT)
#  define AON_WUC_RECHARGESTAT_MAX_USED_PER(n)      ((uint32_t)(n) << AON_WUC_RECHARGESTAT_MAX_USED_PER_SHIFT)
#define AON_WUC_RECHARGESTAT_VDDR_SMPLS_SHIFT       (16)      /* Bits 16-19: Last 4 VDDR samples, bit 0 = newest */
#define AON_WUC_RECHARGESTAT_VDDR_SMPLS_MASK        (15 << AON_WUC_RECHARGESTAT_VDDR_SMPLS_SHIFT)
#  define AON_WUC_RECHARGESTAT_VDDR_SMPLS(n)        ((uint32_t)(n) << AON_WUC_RECHARGESTAT_VDDR_SMPLS_SHIFT)

/* AON_WUC_OSCCFG */

#define AON_WUC_OSCCFG_PER_E_SHIFT                  (0)       /* Bits 0-2: 32KHz clocks between oscillator amplitude
                                                               *           calibrations (exponent) */
#define AON_WUC_OSCCFG_PER_E_MASK                   (7 << AON_WUC_OSCCFG_PER_E_SHIFT)
#  define AON_WUC_OSCCFG_PER_E(n)                   ((uint32_t)(n) << AON_WUC_OSCCFG_PER_E_SHIFT)
#define AON_WUC_OSCCFG_PER_M_SHIFT                  (3)       /* Bits 3-7: 32KHz clocks between oscillator amplitude
                                                               *           calibrations (mantissa) */
#define AON_WUC_OSCCFG_PER_M_MASK                   (31 << AON_WUC_OSCCFG_PER_M_SHIFT)
#  define AON_WUC_OSCCFG_PER_M(n)                   ((uint32_t)(n) << AON_WUC_OSCCFG_PER_M_SHIFT)

/* AON_WUC_JTAGCFG */

#define AON_WUC_JTAGCFG_JTAG_PD_FORCE_ON            (1 << 8)  /* Bit 8:  Force JTAG Power Domain on */

/* AON_WUC_JTAGUSERCODE (32-bit User code) */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X0_CC13X0_AON_WUC_H */
