/********************************************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_aon_pmctl.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible BSD license:
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
 ********************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AON_PMCTL_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AON_PMCTL_H

/********************************************************************************************************************
 * Included Files
 ********************************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/********************************************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************************************/

/* AON PMCTL Register Offsets ***************************************************************************************/

#define TIVA_AON_PMCTL_AUXSCECLK_OFFSET     0x0004  /* AUX SCE Clock Management */
#define TIVA_AON_PMCTL_RAMCFG_OFFSET        0x0008  /* RAM Configuration */
#define TIVA_AON_PMCTL_PWRCTL_OFFSET        0x0010  /* Power Management Control */
#define TIVA_AON_PMCTL_PWRSTAT_OFFSET       0x0014  /* AON Power and Reset Status */
#define TIVA_AON_PMCTL_SHUTDOWN_OFFSET      0x0018  /* Shutdown Control */
#define TIVA_AON_PMCTL_RECHARGECFG_OFFSET   0x001c  /* Recharge Controller Configuration */
#define TIVA_AON_PMCTL_RECHARGESTAT_OFFSET  0x0020  /* Recharge Controller Status */
#define TIVA_AON_PMCTL_OSCCFG_OFFSET        0x0024  /* Oscillator Configuration */
#define TIVA_AON_PMCTL_RESETCTL_OFFSET      0x0028  /* Reset Management */
#define TIVA_AON_PMCTL_SLEEPCTL_OFFSET      0x002c  /* Sleep Control */
#define TIVA_AON_PMCTL_JTAGCFG_OFFSET       0x0034  /* JTAG Configuration */
#define TIVA_AON_PMCTL_JTAGUSERCODE_OFFSET  0x003c  /* JTAG USERCODE */

/* AON PMCTL Register Addresses *************************************************************************************/

#define TIVA_AON_PMCTL_AUXSCECLK            (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_AUXSCECLK_OFFSET)
#define TIVA_AON_PMCTL_RAMCFG               (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_RAMCFG_OFFSET)
#define TIVA_AON_PMCTL_PWRCTL               (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_PWRCTL_OFFSET)
#define TIVA_AON_PMCTL_PWRSTAT              (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_PWRSTAT_OFFSET)
#define TIVA_AON_PMCTL_SHUTDOWN             (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_SHUTDOWN_OFFSET)
#define TIVA_AON_PMCTL_RECHARGECFG          (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_RECHARGECFG_OFFSET)
#define TIVA_AON_PMCTL_RECHARGESTAT         (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_RECHARGESTAT_OFFSET)
#define TIVA_AON_PMCTL_OSCCFG               (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_OSCCFG_OFFSET)
#define TIVA_AON_PMCTL_RESETCTL             (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_RESETCTL_OFFSET)
#define TIVA_AON_PMCTL_SLEEPCTL             (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_SLEEPCTL_OFFSET)
#define TIVA_AON_PMCTL_JTAGCFG              (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_JTAGCFG_OFFSET)
#define TIVA_AON_PMCTL_JTAGUSERCODE         (TIVA_AON_PMCTL_BASE + TIVA_AON_PMCTL_JTAGUSERCODE_OFFSET)

/* AON PMCTL Bitfield Definitions ***********************************************************************************/

/* TIVA_AON_PMCTL_AUXSCECLK */

#define AON_PMCTL_AUXSCECLK_SRC             (1 << 0)  /* Bit 0:  Clock source for AUX dmaon in active mode */
#  define AON_PMCTL_AUXSCECLK_SCLK_HFDIV2   (0)                        /* HF Clock divided by 2 (SCLK_HFDIV2) */
#  define AON_PMCTL_AUXSCECLK_SCLK_MF       AON_PMCTL_AUXSCECLK_SRC    /* MF Clock (SCLK_MF) */
#define AON_PMCTL_AUXSCECLK_PD_SRC          (1 << 8)  /* Bit 8:  Clock source for AUX dmaon in powerdown mode */
#  define AON_PMCTL_AUXSCECLK_PD_NO_CLOCK   (0)                        /* No clock */
#  define AON_PMCTL_AUXSCECLK_PD_SCLK_LF    AON_PMCTL_AUXSCECLK_PD_SRC /* LF clock (SCLK_LF) */

/* TIVA_AON_PMCTL_RAMCFG */

#define AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_SHIFT        (0)       /* Bits 0-3: Select banks for retention during MCU
                                                                 *           bus domain power off */
#define AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_MASK         (15 << AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_SHIFT)
#  define AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_RET_NONE   (0 << AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_SHIFT)  /* Retention is disabled */
#  define AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_RET_LEVEL1 (1 << AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_SHIFT)  /* Retention on for all banks SRAM:BANK0
                                                                                                      * -BANK1 */
#  define AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_RET_LEVEL2 (3 << AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_SHIFT)  /* Retention on for all banks SRAM:BANK0
                                                                                                      * -BANK2 */
#  define AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_RET_LEVEL3 (7 << AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_SHIFT)  /* Retention on for all banks SRAM:BANK0
                                                                                                      * -BANK3 */
#  define AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_RET_FULL   (15 << AON_PMCTL_RAMCFG_BUS_SRAM_RET_EN_SHIFT) /* Retention on for all banks SRAM:BANK0
                                                                                                      * -BANK4 */
#define AON_PMCTL_RAMCFG_AUX_SRAM_RET_EN    (1 << 16) /* Bit 16 */
#define AON_PMCTL_RAMCFG_AUX_SRAM_PWR_OFF   (1 << 17) /* Bit 17 */

/* TIVA_AON_PMCTL_PWRCTL */

#define AON_PMCTL_PWRCTL_DCDC_EN            (1 << 0)  /* Bit 0:  Select to use DCDC or GLC0 during recharge of VDDR */
#  define AON_PMCTL_PWRCTL_DCDC_EN_GLD0     (0)                      /* Use GLDO for recharge of VDDR */
#  define AON_PMCTL_PWRCTL_DCDC_EN_DCDC     AON_PMCTL_PWRCTL_DCDC_EN /* Use DCDC for recharge of VDDR */
#define AON_PMCTL_PWRCTL_EXT_REG_MODE       (1 << 1)  /* Bit 1:  Status of source for VDDR supply */
#  define AON_PMCTL_PWRCTL_EXT_REG_DCDCGLD0 (0)                           /* DCDC or GLDO are generating VDDR */
#  define AON_PMCTL_PWRCTL_EXT_REG_EXTERNAL AON_PMCTL_PWRCTL_EXT_REG_MODE /* External regulator supplies VDDR */
#define AON_PMCTL_PWRCTL_DCDC_ACTIVE        (1 << 2)  /* Bit 2:  Select DCDC regulator for VDDR in active mode */
#  define AON_PMCTL_PWRCTL_DCDC_ACTIVE_GLD0 (0)                          /* Use GLDO for regulation of VDDR in active mode */
#  define AON_PMCTL_PWRCTL_DCDC_ACTIVE_DCDC AON_PMCTL_PWRCTL_DCDC_ACTIVE /* Use DCDC for regulation of VDDR in active mode */

/* TIVA_AON_PMCTL_PWRSTAT */

#define AON_PMCTL_PWRSTAT_AUX_RESET_DONE      (1 << 0)  /* Bit 0:  Indicates Reset Done from AUX */
#define AON_PMCTL_PWRSTAT_AUX_BUS_RESET_DONE  (1 << 1)  /* Bit 1: Indicates Reset Done from AUX Bus */
#define AON_PMCTL_PWRSTAT_JTAG_PD_ON          (1 << 2)  /* Bit 2:  JTAG power state (ON) */

/* TIVA_AON_PMCTL_SHUTDOWN */

#define AON_PMCTL_SHUTDOWN_EN               (1 << 0)  /* Bit 0:  Shutdown control */

/* TIVA_AON_PMCTL_RECHARGECFG */

#define AON_PMCTL_RECHARGECFG_PER_E_SHIFT       (0)       /* Bits 0-2 */
#define AON_PMCTL_RECHARGECFG_PER_E_MASK        (7 << AON_PMCTL_RECHARGECFG_PER_E_SHIFT)
#  define AON_PMCTL_RECHARGECFG_PER_E(n)        ((uint32_t)(n) << AON_PMCTL_RECHARGECFG_PER_E_SHIFT)
#define AON_PMCTL_RECHARGECFG_PER_M_SHIFT       (3)       /* Bits 3-7 */
#define AON_PMCTL_RECHARGECFG_PER_M_MASK        (31 << AON_PMCTL_RECHARGECFG_PER_M_SHIFT)
#  define AON_PMCTL_RECHARGECFG_PER_M(n)        ((uint32_t)(n) << AON_PMCTL_RECHARGECFG_PER_M_SHIFT)
#define AON_PMCTL_RECHARGECFG_MAX_PER_E_SHIFT   (8)       /* Bits 8-10 */
#define AON_PMCTL_RECHARGECFG_MAX_PER_E_MASK    (7 << AON_PMCTL_RECHARGECFG_MAX_PER_E_SHIFT)
#  define AON_PMCTL_RECHARGECFG_MAX_PER_E(n)    ((uint32_t)(n) << AON_PMCTL_RECHARGECFG_MAX_PER_E_SHIFT)
#define AON_PMCTL_RECHARGECFG_MAX_PER_M_SHIFT   (11)      /* Bits 11-15 */
#define AON_PMCTL_RECHARGECFG_MAX_PER_M_MASK    (31 << AON_PMCTL_RECHARGECFG_MAX_PER_M_SHIFT)
#  define AON_PMCTL_RECHARGECFG_MAX_PER_M(n)    ((uint32_t)(n) << AON_PMCTL_RECHARGECFG_MAX_PER_M_SHIFT)
#define AON_PMCTL_RECHARGECFG_C1_SHIFT          (16)      /* Bits 16-19 */
#define AON_PMCTL_RECHARGECFG_C1_MASK           (15 << AON_PMCTL_RECHARGECFG_C1_SHIFT)
#  define AON_PMCTL_RECHARGECFG_C1(n)           ((uint32_t)(n) << AON_PMCTL_RECHARGECFG_C1_SHIFT)
#define AON_PMCTL_RECHARGECFG_C2_SHIFT          (20)      /* Bits 20-23 */
#define AON_PMCTL_RECHARGECFG_C2_MASK           (15 << AON_PMCTL_RECHARGECFG_C2_SHIFT)
#  define AON_PMCTL_RECHARGECFG_C2(n)           ((uint32_t)(n) << AON_PMCTL_RECHARGECFG_C2_SHIFT)
#define AON_PMCTL_RECHARGECFG_MODE_SHIFT        (30)      /* Bits 30-3: Selects recharge algorithm for VDDR when the system is
                                                           *            running on the uLDO */
#define AON_PMCTL_RECHARGECFG_MODE_MASK         (3 << AON_PMCTL_RECHARGECFG_MODE_SHIFT)
#  define AON_PMCTL_RECHARGECFG_MODE_OFF        (0 << AON_PMCTL_RECHARGECFG_MODE_SHIFT) /* Recharge disabled */
#  define AON_PMCTL_RECHARGECFG_MODE_STATIC     (1 << AON_PMCTL_RECHARGECFG_MODE_SHIFT) /* Static timer */
#  define AON_PMCTL_RECHARGECFG_MODE_ADAPTIVE   (2 << AON_PMCTL_RECHARGECFG_MODE_SHIFT) /* Adaptive timer */
#  define AON_PMCTL_RECHARGECFG_MODE_COMPARATOR (3 << AON_PMCTL_RECHARGECFG_MODE_SHIFT) /* External recharge comparator */

/* TIVA_AON_PMCTL_RECHARGESTAT */

#define AON_PMCTL_RECHARGESTAT_MAX_USED_PER_SHIFT (0)  /* Bits 0-15:  Mzx 32KHz periods between recharge cycles and VDDR
                                                        *             is still above BDDR_OK threshold. */
#define AON_PMCTL_RECHARGESTAT_MAX_USED_PER_MASK  (0xffff << AON_PMCTL_RECHARGESTAT_MAX_USED_PER_SHIFT)
#define AON_PMCTL_RECHARGESTAT_VDDR_SMPLS_SHIFT   (16)      /* Bits 16-19:  The last 4 VDDR samples */
#define AON_PMCTL_RECHARGESTAT_VDDR_SMPLS_MASK    (15 << AON_PMCTL_RECHARGESTAT_VDDR_SMPLS_SHIFT)
#  define AON_PMCTL_RECHARGESTAT_VDDR_SMPL0       (1 << AON_PMCTL_RECHARGESTAT_VDDR_SMPLS_SHIFT)
#  define AON_PMCTL_RECHARGESTAT_VDDR_SMPL1       (2 << AON_PMCTL_RECHARGESTAT_VDDR_SMPLS_SHIFT)
#  define AON_PMCTL_RECHARGESTAT_VDDR_SMPL2       (4 << AON_PMCTL_RECHARGESTAT_VDDR_SMPLS_SHIFT)
#  define AON_PMCTL_RECHARGESTAT_VDDR_SMPL3       (8 << AON_PMCTL_RECHARGESTAT_VDDR_SMPLS_SHIFT)

/* TIVA_AON_PMCTL_OSCCFG */

#define AON_PMCTL_OSCCFG_PER_E_SHIFT        (0)       /* Bits 0-2 */
#define AON_PMCTL_OSCCFG_PER_E_MASK         (7 << AON_PMCTL_OSCCFG_PER_E_SHIFT)
#  define AON_PMCTL_OSCCFG_PER_E(n)         ((uint32_t)(n) << AON_PMCTL_OSCCFG_PER_E_SHIFT)
#define AON_PMCTL_OSCCFG_PER_M_SHIFT        (3)       /* Bits 3-7 */
#define AON_PMCTL_OSCCFG_PER_M_MASK         (31 << AON_PMCTL_OSCCFG_PER_M_SHIFT)
#  define AON_PMCTL_OSCCFG_PER_M(n)         ((uint32_t)(n) << AON_PMCTL_OSCCFG_PER_M_SHIFT)

/* TIVA_AON_PMCTL_RESETCTL */

#define AON_PMCTL_RESETCTL_RESET_SRC_SHIFT       (1)       /* Bits 1-3: Shows the root cause of the last system reset */
#define AON_PMCTL_RESETCTL_RESET_SRC_MASK        (7 << AON_PMCTL_RESETCTL_RESET_SRC_SHIFT)
#  define AON_PMCTL_RESETCTL_RESET_SRC_PWR_ON    (0 << AON_PMCTL_RESETCTL_RESET_SRC_SHIFT) /* Power on reset */
#  define AON_PMCTL_RESETCTL_RESET_SRC_PIN_RESET (1 << AON_PMCTL_RESETCTL_RESET_SRC_SHIFT) /* Reset pin */
#  define AON_PMCTL_RESETCTL_RESET_SRC_VDDS_LOSS (2 << AON_PMCTL_RESETCTL_RESET_SRC_SHIFT) /* Brown out detect on VDDS */
#  define AON_PMCTL_RESETCTL_RESET_SRC_VDDR_LOSS (4 << AON_PMCTL_RESETCTL_RESET_SRC_SHIFT) /* Brown out detect on VDDR */
#  define AON_PMCTL_RESETCTL_RESET_SRC_CLK_LOSS  (5 << AON_PMCTL_RESETCTL_RESET_SRC_SHIFT) /* SCLK_LF, SCLK_MF or SCLK_HF clock loss */
#  define AON_PMCTL_RESETCTL_RESET_SRC_SYSRESET  (6 << AON_PMCTL_RESETCTL_RESET_SRC_SHIFT) /* Software reset via SYSRESET or hardware
                                                                                            * power management timeout detection */
#  define AON_PMCTL_RESETCTL_RESET_SRC_WARMRESET (7 << AON_PMCTL_RESETCTL_RESET_SRC_SHIFT) /* Software reset via PRCM warm reset */

#define AON_PMCTL_RESETCTL_MCU_WARM_RESET   (1 << 4)  /* Bit 4 */
#define AON_PMCTL_RESETCTL_CLK_LOSS_EN      (1 << 5)  /* Bit 5:  Controls reset generation in case SCLK_LF, SCLK_MF or
                                                       *         SCLK_HF is lost when clock loss detection is enabled */
#define AON_PMCTL_RESETCTL_VDD_LOSS_EN      (1 << 6)  /* Bit 6:  Controls reset generation in case VDD is lost */
#define AON_PMCTL_RESETCTL_VDDR_LOSS_EN     (1 << 7)  /* Bit 7:  Controls reset generation in case VDDR is lost */
#define AON_PMCTL_RESETCTL_VDDS_LOSS_EN     (1 << 8)  /* Bit 8:  Controls reset generation in case VDDS is lost */
#define AON_PMCTL_RESETCTL_BOOT_DET_0       (1 << 12) /* Bit 12 */
#define AON_PMCTL_RESETCTL_BOOT_DET_1       (1 << 13) /* Bit 13 */
#define AON_PMCTL_RESETCTL_GPIO_WU_FROM_SD  (1 << 14) /* Bit 14: A wakeup from SHUTDOWN on an IO event has occurred */
#define AON_PMCTL_RESETCTL_WU_FROM_SD       (1 << 15) /* Bit 15:  Wakeup from Shutdown */
#define AON_PMCTL_RESETCTL_BOOT_DET_0_SET   (1 << 16) /* Bit 16 */
#define AON_PMCTL_RESETCTL_BOOT_DET_1_SET   (1 << 17) /* Bit 17 */
#define AON_PMCTL_RESETCTL_BOOT_DET_0_CLR   (1 << 24) /* Bit 24 */
#define AON_PMCTL_RESETCTL_BOOT_DET_1_CLR   (1 << 25) /* Bit 25 */
#define AON_PMCTL_RESETCTL_SYSRESET         (1 << 31) /* Bit 31: Cold reset */

/* TIVA_AON_PMCTL_SLEEPCTL */

#define AON_PMCTL_SLEEPCTL_IO_PAD_SLEEP_DIS (1 << 0)  /* Bit 0:  Controls the I/O pad sleep mode */

/* TIVA_AON_PMCTL_JTAGCFG */

#define AON_PMCTL_JTAGCFG_JTAG_PD_FORCE_ON  (1 << 8)  /* Bit 8:  Controls JTAG Power domain power state */

/* TIVA_AON_PMCTL_JTAGUSERCODE (32-bit value) */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_AON_PMCTL_H */
