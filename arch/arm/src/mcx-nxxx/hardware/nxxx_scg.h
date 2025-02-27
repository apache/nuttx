/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/nxxx_scg.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_SCG_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_SCG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/nxxx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TODO: Header file is incomplete / might contain false information */

/* SCG Register Offsets *****************************************************/

#define NXXX_SCG_VERID_OFFSET           0x0000  /* Version ID Register */
#define NXXX_SCG_PARAM_OFFSET           0x0004  /* Parameter Register */
#define NXXX_SCG_TRIM_LOCK_OFFSET       0x0008  /* Trim Lock Register */
#define NXXX_SCG_CSR_OFFSET             0x0010  /* Clock Status Register */
#define NXXX_SCG_RCCR_OFFSET            0x0014  /* Run Clock Control Register */
#define NXXX_SCG_SOSCCSR_OFFSET         0x0100  /* System OSC Control Status Register */
#define NXXX_SCG_SOSCDIV_OFFSET         0x0104  /* System OSC Divide Register */
#define NXXX_SCG_SOSCCFG_OFFSET         0x0108  /* System Oscillator Configuration Register */
#define NXXX_SCG_SIRCCSR_OFFSET         0x0200  /* Slow IRC Control Status Register */
#define NXXX_SCG_SIRCDIV_OFFSET         0x0204  /* Slow IRC Divide Register */
#define NXXX_SCG_SIRCCFG_OFFSET         0x0208  /* Slow IRC Configuration Register */
#define NXXX_SCG_FIRCCSR_OFFSET         0x0300  /* Fast IRC Control Status Register */
#define NXXX_SCG_FIRCDIV_OFFSET         0x0304  /* Fast IRC Divide Register */
#define NXXX_SCG_FIRCCFG_OFFSET         0x0308  /* Fast IRC Configuration Register */
#define NXXX_SCG_APLLCSR_OFFSET         0x0500  /* APLL Control Status Register */
#define NXXX_SCG_APLLCTRL_OFFSET        0x0504  /* APLL Control Register */
#define NXXX_SCG_APLLSTAT_OFFSET        0x0508  /* APLL Status Register */
#define NXXX_SCG_APLLNDIV_OFFSET        0x050C  /* APLL N Divider Register */
#define NXXX_SCG_APLLMDIV_OFFSET        0x0510  /* APLL M Divider Register */
#define NXXX_SCG_APLLPDIV_OFFSET        0x0514  /* APLL P Divider Register */
#define NXXX_SCG_APLLLOCK_CNFG_OFFSET   0x0518  /* APLL LOCK Configuration Register */
#define NXXX_SCG_APLLSSCGSTAT_OFFSET    0x0520  /* APLL SSCG Status Register */
#define NXXX_SCG_APLLSSCG0_OFFSET       0x0524  /* APLL Spread Spectrum Control 0 Register */
#define NXXX_SCG_APLLSSCG1_OFFSET       0x0528  /* APLL Spread Spectrum Control 1 Register */
#define NXXX_SCG_APLL_OVRD_OFFSET       0x05F4  /* APLL Override Register */
#define NXXX_SCG_SPLLCSR_OFFSET         0x0600  /* System PLL Control Status Register */
#define NXXX_SCG_SPLLDIV_OFFSET         0x0604  /* System PLL Divide Register */
#define NXXX_SCG_SPLLCFG_OFFSET         0x0608  /* System PLL Configuration Register */
#define NXXX_SCG_LDOCSR_OFFSET          0x0800  /* LDO Control and Status Register */

/* SCG Register Addresses ***************************************************/

#define NXXX_SCG_VERID              (NXXX_SCG0_BASE + NXXX_SCG_VERID_OFFSET)
#define NXXX_SCG_PARAM              (NXXX_SCG0_BASE + NXXX_SCG_PARAM_OFFSET)
#define NXXX_SCG_TRIM_LOCK          (NXXX_SCG0_BASE + NXXX_SCG_TRIM_LOCK_OFFSET)
#define NXXX_SCG_CSR                (NXXX_SCG0_BASE + NXXX_SCG_CSR_OFFSET)
#define NXXX_SCG_RCCR               (NXXX_SCG0_BASE + NXXX_SCG_RCCR_OFFSET)
#define NXXX_SCG_SOSCCSR            (NXXX_SCG0_BASE + NXXX_SCG_SOSCCSR_OFFSET)
#define NXXX_SCG_SOSCDIV            (NXXX_SCG0_BASE + NXXX_SCG_SOSCDIV_OFFSET)
#define NXXX_SCG_SOSCCFG            (NXXX_SCG0_BASE + NXXX_SCG_SOSCCFG_OFFSET)
#define NXXX_SCG_SIRCCSR            (NXXX_SCG0_BASE + NXXX_SCG_SIRCCSR_OFFSET)
#define NXXX_SCG_SIRCDIV            (NXXX_SCG0_BASE + NXXX_SCG_SIRCDIV_OFFSET)
#define NXXX_SCG_SIRCCFG            (NXXX_SCG0_BASE + NXXX_SCG_SIRCCFG_OFFSET)
#define NXXX_SCG_FIRCCSR            (NXXX_SCG0_BASE + NXXX_SCG_FIRCCSR_OFFSET)
#define NXXX_SCG_FIRCDIV            (NXXX_SCG0_BASE + NXXX_SCG_FIRCDIV_OFFSET)
#define NXXX_SCG_FIRCCFG            (NXXX_SCG0_BASE + NXXX_SCG_FIRCCFG_OFFSET)
#define NXXX_SCG_APLLCSR            (NXXX_SCG0_BASE + NXXX_SCG_APLLCSR_OFFSET)
#define NXXX_SCG_APLLCTRL           (NXXX_SCG0_BASE + NXXX_SCG_APLLCTRL_OFFSET)
#define NXXX_SCG_APLLSTAT           (NXXX_SCG0_BASE + NXXX_SCG_APLLSTAT_OFFSET)
#define NXXX_SCG_APLLNDIV           (NXXX_SCG0_BASE + NXXX_SCG_APLLNDIV_OFFSET)
#define NXXX_SCG_APLLMDIV           (NXXX_SCG0_BASE + NXXX_SCG_APLLMDIV_OFFSET)
#define NXXX_SCG_APLLPDIV           (NXXX_SCG0_BASE + NXXX_SCG_APLLPDIV_OFFSET)
#define NXXX_SCG_APLLLOCK_CNFG      (NXXX_SCG0_BASE + NXXX_SCG_APLLLOCK_CNFG_OFFSET)
#define NXXX_SCG_APLLSSCGSTAT       (NXXX_SCG0_BASE + NXXX_SCG_APLLSSCGSTAT_OFFSET)
#define NXXX_SCG_APLLSSCG0          (NXXX_SCG0_BASE + NXXX_SCG_APLLSSCG0_OFFSET)
#define NXXX_SCG_APLLSSCG1          (NXXX_SCG0_BASE + NXXX_SCG_APLLSSCG1_OFFSET)
#define NXXX_SCG_APLL_OVRD          (NXXX_SCG0_BASE + NXXX_SCG_APLL_OVRD_OFFSET)
#define NXXX_SCG_SPLLCSR            (NXXX_SCG0_BASE + NXXX_SCG_SPLLCSR_OFFSET)
#define NXXX_SCG_SPLLDIV            (NXXX_SCG0_BASE + NXXX_SCG_SPLLDIV_OFFSET)
#define NXXX_SCG_SPLLCFG            (NXXX_SCG0_BASE + NXXX_SCG_SPLLCFG_OFFSET)
#define NXXX_CFG_LDOCSR             (NXXX_SCG0_BASE + NXXX_SCG_LDOCSR_OFFSET)

/* SCG Register Bitfield Definitions ****************************************/

/* Version ID Register (32-bit version number) */

/* Parameter Register */

#define SCG_PARAM_SOSC                 (1 << 1)  /* System OSC (SOSC) clock present */
#define SCG_PARAM_SIRC                 (1 << 2)  /* Slow IRC (SIRC) clock present */
#define SCG_PARAM_FIRC                 (1 << 3)  /* Fast IRC (FIRC) clock present */
#define SCG_PARAM_APLL                 (1 << 5)  /* APLL (APLL) clock present */
#define SCG_PARAM_SPLL                 (1 << 6)  /* System PLL (SPLL) clock present */
#define SCG_PARAM_UPLL                 (1 << 7)  /* UPLL (UPLL) clock present */

/* Trim Lock Register */

#define SCG_TRIM_LOCK_TRIM_UNLOCK      (1 << 0)  /* Bit0: Locks user write access to SCG Trim and PLL LOCK */
#define SCG_TRIM_LOCK IFR_DISABLE      (1 << 1)  /* Bit1: Locks IFR write access to SCG trim registers */
#define SCG_TRIM_LOCK_TRIM_LOCK_KEY    (1 << 16) /* Bits16-31: Write 5A5Ah to unlock */

/* Clock Status Register */

#define SCG_CSR_SCS_SHIFT              (24)      /* Bits 24-27:  System clock source */
#define SCG_CSR_SCS_MASK               (15 << SCG_CSR_SCS_SHIFT)
# define SCG_CSR_SCS_SOSC              (1 << SCG_CSR_SCS_SHIFT)
# define SCG_CSR_SCS_SIRC              (2 << SCG_CSR_SCS_SHIFT)
# define SCG_CSR_SCS_FIRC              (3 << SCG_CSR_SCS_SHIFT)
# define SCG_CSR_SCS_ROSC              (4 << SCG_CSR_SCS_SHIFT)
# define SCG_CSR_SCS_APLL              (5 << SCG_CSR_SCS_SHIFT)
# define SCG_CSR_SCS_SPLL              (6 << SCG_CSR_SCS_SHIFT)
# define SCG_CSR_SCS_UPLL              (7 << SCG_CSR_SCS_SHIFT)

/* Run Clock Control Register */

#define SCG_RCCR_SCS_SHIFT             (24)
#define SCG_RCCR_SCS_MASK              (15 << SCG_RCCR_SCS_SHIFT)
# define SCG_RCCR_SCS_SOSC             (1 << SCG_RCCR_SCS_SHIFT) /* CLK_IN */
# define SCG_RCCR_SCS_SIRC             (2 << SCG_RCCR_SCS_SHIFT) /* FRO_12MHz */
# define SCG_RCCR_SCS_FIRC             (3 << SCG_RCCR_SCS_SHIFT) /* FRO_HF */
# define SCG_RCCR_SCS_ROSC             (4 << SCG_RCCR_SCS_SHIFT) /* XTAL32K */
# define SCG_RCCR_SCS_APLL             (5 << SCG_RCCR_SCS_SHIFT) /* PLL0_CLK */
# define SCG_RCCR_SCS_SPLL             (6 << SCG_RCRR_SCS_SHIFT) /* PLL1_CLK */
# define SCG_RCCR_SCS_UPLL             (7 << SCG_RCCR_SCS_SHIFT) /* USB_PLL_CLK */

/* System OSC Control Status Register */

#define SCG_SOSCCSR_SOSCEN             (1 << 0)  /* Bit 0:  System OSC Enable */
#define SCG_SOSCCSR_SOSCCM             (1 << 16) /* Bit 16: System OSC Clock Monitor */
#define SCG_SOSCCSR_SOSCCMRE           (1 << 17) /* Bit 17: System OSC Clock Monitor Reset Enable */
#define SCG_SOSCCSR_LK                 (1 << 23) /* Bit 23: Lock Register */
#define SCG_SOSCCSR_SOSCVLD            (1 << 24) /* Bit 24: System OSC Valid */
#define SCG_SOSCCSR_SOSCSEL            (1 << 25) /* Bit 25: System OSC Selected */
#define SCG_SOSCCSR_SOSCERR            (1 << 26) /* Bit 26: System OSC Clock Error */

/* System OSC Divide Register */

#define SCG_SOSCDIV_SOSCDIV1_SHIFT     (0)       /* Bits 0-2:  System OSC Clock Divide 1 */
#define SCG_SOSCDIV_SOSCDIV1_MASK      (7 << SCG_SOSCDIV_SOSCDIV1_SHIFT)
#  define SCG_SOSCDIV_SOSCDIV1(n)      ((uint32_t)(n) << SCG_SOSCDIV_SOSCDIV1_SHIFT)
#  define SCG_SOSCDIV_SOSCDIV1_DISABLE (0 << SCG_SOSCDIV_SOSCDIV1_SHIFT)  /* Output disabled */
#  define SCG_SOSCDIV_SOSCDIV1_DIV1    (1 << SCG_SOSCDIV_SOSCDIV1_SHIFT)  /* Divide by 1 */
#  define SCG_SOSCDIV_SOSCDIV1_DIV2    (2 << SCG_SOSCDIV_SOSCDIV1_SHIFT)  /* Divide by 2 */
#  define SCG_SOSCDIV_SOSCDIV1_DIV4    (3 << SCG_SOSCDIV_SOSCDIV1_SHIFT)  /* Divide by 4 */
#  define SCG_SOSCDIV_SOSCDIV1_DIV8    (4 << SCG_SOSCDIV_SOSCDIV1_SHIFT)  /* Divide by 8 */
#  define SCG_SOSCDIV_SOSCDIV1_DIV16   (5 << SCG_SOSCDIV_SOSCDIV1_SHIFT)  /* Divide by 16 */
#  define SCG_SOSCDIV_SOSCDIV1_DIV32   (6 << SCG_SOSCDIV_SOSCDIV1_SHIFT)  /* Divide by 32 */
#  define SCG_SOSCDIV_SOSCDIV1_DIV64   (7 << SCG_SOSCDIV_SOSCDIV1_SHIFT)  /* Divide by 64 */

#define SCG_SOSCDIV_SOSCDIV2_SHIFT     (8)       /* Bits 8-10:  System OSC Clock Divide 2 */
#define SCG_SOSCDIV_SOSCDIV2_MASK      (7 << SCG_SOSCDIV_SOSCDIV2_SHIFT)
#  define SCG_SOSCDIV_SOSCDIV2(n)      ((uint32_t)(n) << SCG_SOSCDIV_SOSCDIV2_SHIFT)
#  define SCG_SOSCDIV_SOSCDIV2_DISABLE (0 << SCG_SOSCDIV_SOSCDIV2_SHIFT)  /* Output disabled */
#  define SCG_SOSCDIV_SOSCDIV2_DIV1    (1 << SCG_SOSCDIV_SOSCDIV2_SHIFT)  /* Divide by 1 */
#  define SCG_SOSCDIV_SOSCDIV2_DIV2    (2 << SCG_SOSCDIV_SOSCDIV2_SHIFT)  /* Divide by 2 */
#  define SCG_SOSCDIV_SOSCDIV2_DIV4    (3 << SCG_SOSCDIV_SOSCDIV2_SHIFT)  /* Divide by 4 */
#  define SCG_SOSCDIV_SOSCDIV2_DIV8    (4 << SCG_SOSCDIV_SOSCDIV2_SHIFT)  /* Divide by 8 */
#  define SCG_SOSCDIV_SOSCDIV2_DIV16   (5 << SCG_SOSCDIV_SOSCDIV2_SHIFT)  /* Divide by 16 */
#  define SCG_SOSCDIV_SOSCDIV2_DIV32   (6 << SCG_SOSCDIV_SOSCDIV2_SHIFT)  /* Divide by 32 */
#  define SCG_SOSCDIV_SOSCDIV2_DIV64   (7 << SCG_SOSCDIV_SOSCDIV2_SHIFT)  /* Divide by 64 */

/* System Oscillator Configuration Register */

#define SCG_SOSCCFG_EREFS              (1 << 2)  /* Bit 2:  External Reference Select */
#define SCG_SOSCCFG_HGO                (1 << 3)  /* Bit 3:  High Gain Oscillator Select */
#define SCG_SOSCCFG_RANGE_SHIFT        (4)       /* Bits 4-5:  System OSC Range Select */
#define SCG_SOSCCFG_RANGE_MASK         (3 << SCG_SOSCCFG_RANGE_SHIFT)
#  define SCG_SOSCCFG_RANGE(n)         ((uint32_t)(n) << SCG_SOSCCFG_RANGE_SHIFT)
#  define SCG_SOSCCFG_RANGE_LOW        (1 << SCG_SOSCCFG_RANGE_SHIFT)  /* Low frequency range */
#  define SCG_SOSCCFG_RANGE_MED        (2 << SCG_SOSCCFG_RANGE_SHIFT)  /* Medium frequency range */
#  define SCG_SOSCCFG_RANGE_HIGH       (3 << SCG_SOSCCFG_RANGE_SHIFT)  /* High frequency range */

/* Slow IRC Control Status Register */

#define SCG_SIRCCSR_SIRCEN             (1 << 0)  /* Bit 0:  Slow IRC Enable */
#define SCG_SIRCCSR_SIRCSTEN           (1 << 1)  /* Bit 1:  Slow IRC Stop Enable */
#define SCG_SIRCCSR_SIRCLPEN           (1 << 2)  /* Bit 2:  Slow IRC Low Power Enable */
#define SCG_SIRCCSR_LK                 (1 << 23) /* Bit 23: Lock Register */
#define SCG_SIRCCSR_SIRCVLD            (1 << 24) /* Bit 24: Slow IRC Valid */
#define SCG_SIRCCSR_SIRCSEL            (1 << 25) /* Bit 25: Slow IRC Selected */

/* Slow IRC Divide Register */

#define SCG_SIRCDIV_SIRCDIV1_SHIFT     (0)       /* Bits 0-2:  Slow IRC Clock Divide 1 */
#define SCG_SIRCDIV_SIRCDIV1_MASK      (7 << SCG_SIRCDIV_SIRCDIV1_SHIFT)
#  define SCG_SIRCDIV_SIRCDIV1(n)      ((uint32_t)(n) << SCG_SIRCDIV_SIRCDIV1_SHIFT)
#  define SCG_SIRCDIV_SIRCDIV1_DISABLE (0 << SCG_SIRCDIV_SIRCDIV1_SHIFT)  /* Output disabled */
#  define SCG_SIRCDIV_SIRCDIV1_DIV1    (1 << SCG_SIRCDIV_SIRCDIV1_SHIFT)  /* Divide by 1 */
#  define SCG_SIRCDIV_SIRCDIV1_DIV2    (2 << SCG_SIRCDIV_SIRCDIV1_SHIFT)  /* Divide by 2 */
#  define SCG_SIRCDIV_SIRCDIV1_DIV4    (3 << SCG_SIRCDIV_SIRCDIV1_SHIFT)  /* Divide by 4 */
#  define SCG_SIRCDIV_SIRCDIV1_DIV8    (4 << SCG_SIRCDIV_SIRCDIV1_SHIFT)  /* Divide by 8 */
#  define SCG_SIRCDIV_SIRCDIV1_DIV16   (5 << SCG_SIRCDIV_SIRCDIV1_SHIFT)  /* Divide by 16 */
#  define SCG_SIRCDIV_SIRCDIV1_DIV32   (6 << SCG_SIRCDIV_SIRCDIV1_SHIFT)  /* Divide by 32 */
#  define SCG_SIRCDIV_SIRCDIV1_DIV64   (7 << SCG_SIRCDIV_SIRCDIV1_SHIFT)  /* Divide by 64 */

#define SCG_SIRCDIV_SIRCDIV2_SHIFT     (8)       /* Bits 8-10:  Slow IRC Clock Divide 2 */
#define SCG_SIRCDIV_SIRCDIV2_MASK      (7 << SCG_SIRCDIV_SIRCDIV2_SHIFT)
#  define SCG_SIRCDIV_SIRCDIV2(n)      ((uint32_t)(n) << SCG_SIRCDIV_SIRCDIV2_SHIFT)
#  define SCG_SIRCDIV_SIRCDIV2_DISABLE (0 << SCG_SIRCDIV_SIRCDIV2_SHIFT)  /* Output disabled */
#  define SCG_SIRCDIV_SIRCDIV2_DIV1    (1 << SCG_SIRCDIV_SIRCDIV2_SHIFT)  /* Divide by 1 */
#  define SCG_SIRCDIV_SIRCDIV2_DIV2    (2 << SCG_SIRCDIV_SIRCDIV2_SHIFT)  /* Divide by 2 */
#  define SCG_SIRCDIV_SIRCDIV2_DIV4    (3 << SCG_SIRCDIV_SIRCDIV2_SHIFT)  /* Divide by 4 */
#  define SCG_SIRCDIV_SIRCDIV2_DIV8    (4 << SCG_SIRCDIV_SIRCDIV2_SHIFT)  /* Divide by 8 */
#  define SCG_SIRCDIV_SIRCDIV2_DIV16   (5 << SCG_SIRCDIV_SIRCDIV2_SHIFT)  /* Divide by 16 */
#  define SCG_SIRCDIV_SIRCDIV2_DIV32   (6 << SCG_SIRCDIV_SIRCDIV2_SHIFT)  /* Divide by 32 */
#  define SCG_SIRCDIV_SIRCDIV2_DIV64   (7 << SCG_SIRCDIV_SIRCDIV2_SHIFT)  /* Divide by 64 */

/* Slow IRC Configuration Register */

#define SCG_SIRCCFG_RANGE              (1 << 0) /* Bit 0:  Frequency Range */
#  define SCG_SIRCCFG_LOWRANGE         (0)      /* Slow IRC low range clock (2 MHz) */
#  define SCG_SIRCCFG_HIGHRANGE        (1 << 0) /* Slow IRC high range clock (8 MHz ) */

/* Fast IRC Control Status Register */

#define SCG_FIRCCSR_FIRCEN             (1 << 0)  /* Bit 0:  Fast IRC Enable */
#define SCG_FIRCCSR_FIRCSTEN           (1 << 1)  /* Bit 1:  Stop enable */
#define SCG_FIRCCSR_SCLK_PERIPH_EN     (1 << 4)  /* Bit 2:  Fast IRC 48 MHz Clock to peripherals Enable */
#define SCG_FIRCCSR_FCLK_PERIPH_EN     (1 << 5)  /* Bit 3:  Fast IRC 144 MHz clock to peripherals Enable */
#define SCG_FIRCCSR_FIRCTREN           (1 << 8)  /* Bit 8:  Fast IRC 144 MHz Trim Enable */
#define SCG_FIRCCSR_FIRCTRUP           (1 << 9)  /* Bit 9:  Fast IRC Trim Update */
#define SCG_FIRCCSR_TRIM_LOCK          (1 << 10) /* Bit 10: Fast IRC auto trim lock */
#define SCG_FIRCCSR_COARSE_TRIM_BYPASS (1 << 11) /* Bit 11: Coarse Auto Trim Bypass */
#define SCG_FIRCCSR_LK                 (1 << 23) /* Bit 23: Lock Register */
#define SCG_FIRCCSR_FIRCVLD            (1 << 24) /* Bit 24: Fast IRC Valid status */
#define SCG_FIRCCSR_FIRCSEL            (1 << 25) /* Bit 25: Fast IRC Selected status */
#define SCG_FIRCCSR_FIRCERR            (1 << 26) /* Bit 26: Fast IRC Clock Error */
#define SCG_FIRCCSR_FIRCERR_IE         (1 << 27) /* Bit 26: Fast IRC Clock Error Interrupt Enable */
#define SCG_FIRCCSR_FIRCACC_IE         (1 << 30) /* Bit 30: Fast IRC Accurate Interrupt Enable */
#define SCG_FIRCCSR_FIRCACC            (1 << 31) /* Bit 30: Fast IRC Frequency Accurate */

/* Fast IRC Divide Register */

#define SCG_FIRCDIV_FIRCDIV1_SHIFT     (0)       /* Bits 0-2:  Fast IRC Clock Divide 1 */
#define SCG_FIRCDIV_FIRCDIV1_MASK      (7 << SCG_FIRCDIV_FIRCDIV1_SHIFT)
#  define SCG_FIRCDIV_FIRCDIV1(n)      ((uint32_t)(n) << SCG_FIRCDIV_FIRCDIV1_SHIFT)
#  define SCG_FIRCDIV_FIRCDIV1_DISABLE (0 << SCG_FIRCDIV_FIRCDIV1_SHIFT)  /* Output disabled */
#  define SCG_FIRCDIV_FIRCDIV1_DIV1    (1 << SCG_FIRCDIV_FIRCDIV1_SHIFT)  /* Divide by 1 */
#  define SCG_FIRCDIV_FIRCDIV1_DIV2    (2 << SCG_FIRCDIV_FIRCDIV1_SHIFT)  /* Divide by 2 */
#  define SCG_FIRCDIV_FIRCDIV1_DIV4    (3 << SCG_FIRCDIV_FIRCDIV1_SHIFT)  /* Divide by 4 */
#  define SCG_FIRCDIV_FIRCDIV1_DIV8    (4 << SCG_FIRCDIV_FIRCDIV1_SHIFT)  /* Divide by 8 */
#  define SCG_FIRCDIV_FIRCDIV1_DIV16   (5 << SCG_FIRCDIV_FIRCDIV1_SHIFT)  /* Divide by 16 */
#  define SCG_FIRCDIV_FIRCDIV1_DIV32   (6 << SCG_FIRCDIV_FIRCDIV1_SHIFT)  /* Divide by 32 */
#  define SCG_FIRCDIV_FIRCDIV1_DIV64   (7 << SCG_FIRCDIV_FIRCDIV1_SHIFT)  /* Divide by 64 */

#define SCG_FIRCDIV_FIRCDIV2_SHIFT     (8)       /* Bits 8-10:  Fast IRC Clock Divide 2 */
#define SCG_FIRCDIV_FIRCDIV2_MASK      (7 << SCG_FIRCDIV_FIRCDIV2_SHIFT)
#  define SCG_FIRCDIV_FIRCDIV2(n)      ((uint32_t)(n) << SCG_FIRCDIV_FIRCDIV2_SHIFT)
#  define SCG_FIRCDIV_FIRCDIV2_DISABLE (0 << SCG_FIRCDIV_FIRCDIV2_SHIFT)  /* Output disabled */
#  define SCG_FIRCDIV_FIRCDIV2_DIV1    (1 << SCG_FIRCDIV_FIRCDIV2_SHIFT)  /* Divide by 1 */
#  define SCG_FIRCDIV_FIRCDIV2_DIV2    (2 << SCG_FIRCDIV_FIRCDIV2_SHIFT)  /* Divide by 2 */
#  define SCG_FIRCDIV_FIRCDIV2_DIV4    (3 << SCG_FIRCDIV_FIRCDIV2_SHIFT)  /* Divide by 4 */
#  define SCG_FIRCDIV_FIRCDIV2_DIV8    (4 << SCG_FIRCDIV_FIRCDIV2_SHIFT)  /* Divide by 8 */
#  define SCG_FIRCDIV_FIRCDIV2_DIV16   (5 << SCG_FIRCDIV_FIRCDIV2_SHIFT)  /* Divide by 16 */
#  define SCG_FIRCDIV_FIRCDIV2_DIV32   (6 << SCG_FIRCDIV_FIRCDIV2_SHIFT)  /* Divide by 32 */
#  define SCG_FIRCDIV_FIRCDIV2_DIV64   (7 << SCG_FIRCDIV_FIRCDIV2_SHIFT)  /* Divide by 64 */

/* Fast IRC Configuration Register */

#define SCG_FIRCCFG_RANGE              (1 << 0) /* Bit 0:  Frequency Range */
#  define SCG_FIRCCFG_48MHZ            (0)      /* Fast IRC is trimmed to 48 MHz */

/* APLL Control Status Register */

#define SCG_APLLCSR_APLLPWREN_SHIFT              (0)
#define SCG_APLLCSR_APLLPWREN_MASK               (0x01 << SCG_APLLCSR_APLLPWREN_SHIFT)
#define SCG_APLLCSR_APLLPWREN(x)                 (((x) << SCG_APLLCSR_APLLPWREN_SHIFT) & SCG_APLLCSR_APLLPWREN_MASK)

#define SCG_APLLCSR_APLLCLKEN_SHIFT              (1)
#define SCG_APLLCSR_APLLCLKEN_MASK               (0x01 << SCG_APLLCSR_APLLCLKEN_SHIFT)
#define SCG_APLLCSR_APLLCLKEN(x)                 (((x) << SCG_APLLCSR_APLLCLKEN_SHIFT) & SCG_APLLCSR_APLLCLKEN_MASK)

#define SCG_APLLCSR_APLLSTEN_SHIFT               (2)
#define SCG_APLLCSR_APLLSTEN_MASK                (0x01 << SCG_APLLCSR_APLLSTEN_SHIFT)
#define SCG_APLLCSR_APLLSTEN(x)                  (((x) << SCG_APLLCSR_APLLSTEN_SHIFT) & SCG_APLLCSR_APLLSTEN_MASK)

#define SCG_APLLCSR_FRM_CLOCKSTABLE_SHIFT        (3)
#define SCG_APLLCSR_FRM_CLOCKSTABLE_MASK         (0x01 << SCG_APLLCSR_FRM_CLOCKSTABLE_SHIFT)
#define SCG_APLLCSR_FRM_CLOCKSTABLE(x)           (((x) << SCG_APLLCSR_FRM_CLOCKSTABLE_SHIFT) & SCG_APLLCSR_FRM_CLOCKSTABLE_MASK)

#define SCG_APLLCSR_APLLCM_SHIFT                 (16)
#define SCG_APLLCSR_APLLCM_MASK                  (0x01 << SCG_APLLCSR_APLLCM_SHIFT)
#define SCG_APLLCSR_APLLCM(x)                    (((x) << SCG_APLLCSR_APLLCM_SHIFT) & SCG_APLLCSR_APLLCM_MASK)

#define SCG_APLLCSR_APLLCMRE_SHIFT               (17)
#define SCG_APLLCSR_APLLCMRE_MASK                (0x01 << SCG_APLLCSR_APLLCMRE_SHIFT)
#define SCG_APLLCSR_APLLCMRE(x)                  (((x) << SCG_APLLCSR_APLLCMRE_SHIFT) & SCG_APLLCSR_APLLCMRE_MASK)

#define SCG_APLLCSR_LK_SHIFT                     (23)
#define SCG_APLLCSR_LK_MASK                      (0x01 << SCG_APLLCSR_LK_SHIFT)
#define SCG_APLLCSR_LK(x)                        (((x) << SCG_APLLCSR_LK_SHIFT) & SCG_APLLCSR_LK_MASK)

#define SCG_APLLCSR_APLL_LOCK_SHIFT              (24)
#define SCG_APLLCSR_APLL_LOCK_MASK               (0x01 << SCG_APLLCSR_APLL_LOCK_SHIFT)
#define SCG_APLLCSR_APLL_LOCK(x)                 (((x) << SCG_APLLCSR_APLL_LOCK_SHIFT) & SCG_APLLCSR_APLL_LOCK_MASK)

#define SCG_APLLCSR_APLLSEL_SHIFT                (25)
#define SCG_APLLCSR_APLLSEL_MASK                 (0x01 << SCG_APLLCSR_APLLSEL_SHIFT)
#define SCG_APLLCSR_APLLSEL(x)                   (((x) << SCG_APLLCSR_APLLSEL_SHIFT) & SCG_APLLCSR_APLLSEL_MASK)

#define SCG_APLLCSR_APLLERR_SHIFT                (26)
#define SCG_APLLCSR_APLLERR_MASK                 (0x01 << SCG_APLLCSR_APLLERR_SHIFT)
#define SCG_APLLCSR_APLLERR(x)                   (((x) << SCG_APLLCSR_APLLERR_SHIFT) & SCG_APLLCSR_APLLERR_MASK)

#define SCG_APLLCSR_APLL_LOCK_IE_SHIFT           (30)
#define SCG_APLLCSR_APLL_LOCK_IE_MASK            (0x01 << SCG_APLLCSR_APLL_LOCK_IE_SHIFT)
#define SCG_APLLCSR_APLL_LOCK_IE(x)              (((x) << SCG_APLLCSR_APLL_LOCK_IE_SHIFT) & SCG_APLLCSR_APLL_LOCK_IE_MASK)

/* APLL Control Register */

#define SCG_APLLCTRL_SELR_SHIFT                  (0)
#define SCG_APLLCTRL_SELR_MASK                   (0x0f << SCG_APLLCTRL_SELR_SHIFT)
#define SCG_APLLCTRL_SELR(x)                     (((x) << SCG_APLLCTRL_SELR_SHIFT) & SCG_APLLCTRL_SELR_MASK)

#define SCG_APLLCTRL_SELI_SHIFT                  (4)
#define SCG_APLLCTRL_SELI_MASK                   (0x3f << SCG_APLLCTRL_SELI_SHIFT)
#define SCG_APLLCTRL_SELI(x)                     (((x) << SCG_APLLCTRL_SELI_SHIFT) & SCG_APLLCTRL_SELI_MASK)

#define SCG_APLLCTRL_SELP_SHIFT                  (10)
#define SCG_APLLCTRL_SELP_MASK                   (0x1f << SCG_APLLCTRL_SELP_SHIFT)
#define SCG_APLLCTRL_SELP(x)                     (((x) << SCG_APLLCTRL_SELP_SHIFT) & SCG_APLLCTRL_SELP_MASK)

#define SCG_APLLCTRL_BYPASSPOSTDIV2_SHIFT        (16)
#define SCG_APLLCTRL_BYPASSPOSTDIV2_MASK         (0x01 << SCG_APLLCTRL_BYPASSPOSTDIV2_SHIFT)
#define SCG_APLLCTRL_BYPASSPOSTDIV2(x)           (((x) << SCG_APLLCTRL_BYPASSPOSTDIV2_SHIFT) & SCG_APLLCTRL_BYPASSPOSTDIV2_MASK)

#define SCG_APLLCTRL_LIMUPOFF_SHIFT              (17)
#define SCG_APLLCTRL_LIMUPOFF_MASK               (0x01 << SCG_APLLCTRL_LIMUPOFF_SHIFT)
#define SCG_APLLCTRL_LIMUPOFF(x)                 (((x) << SCG_APLLCTRL_LIMUPOFF_SHIFT) & SCG_APLLCTRL_LIMUPOFF_MASK)

#define SCG_APLLCTRL_BANDDIRECT_SHIFT            (18)
#define SCG_APLLCTRL_BANDDIRECT_MASK             (0x01 << SCG_APLLCTRL_BANDDIRECT_SHIFT)
#define SCG_APLLCTRL_BANDDIRECT(x)               (((x) << SCG_APLLCTRL_BANDDIRECT_SHIFT) & SCG_APLLCTRL_BANDDIRECT_MASK)

#define SCG_APLLCTRL_BYPASSPREDIV_SHIFT          (19)
#define SCG_APLLCTRL_BYPASSPREDIV_MASK           (0x01 << SCG_APLLCTRL_BYPASSPREDIV_SHIFT)
#define SCG_APLLCTRL_BYPASSPREDIV(x)             (((x) << SCG_APLLCTRL_BYPASSPREDIV_SHIFT) & SCG_APLLCTRL_BYPASSPREDIV_MASK)

#define SCG_APLLCTRL_BYPASSPOSTDIV_SHIFT         (20)
#define SCG_APLLCTRL_BYPASSPOSTDIV_MASK          (0x01 << SCG_APLLCTRL_BYPASSPOSTDIV_SHIFT)
#define SCG_APLLCTRL_BYPASSPOSTDIV(x)            (((x) << SCG_APLLCTRL_BYPASSPOSTDIV_SHIFT) & SCG_APLLCTRL_BYPASSPOSTDIV_MASK)

#define SCG_APLLCTRL_FRM_SHIFT                   (22)
#define SCG_APLLCTRL_FRM_MASK                    (0x01 << SCG_APLLCTRL_FRM_SHIFT)
#define SCG_APLLCTRL_FRM(x)                      (((x) << SCG_APLLCTRL_FRM_SHIFT) & SCG_APLLCTRL_FRM_MASK)

#define SCG_APLLCTRL_SOURCE_SHIFT                (25)
#define SCG_APLLCTRL_SOURCE_MASK                 (0x01 << SCG_APLLCTRL_SOURCE_SHIFT)
#define SCG_APLLCTRL_SOURCE(x)                   (((x) << SCG_APLLCTRL_SOURCE_SHIFT) & SCG_APLLCTRL_SOURCE_MASK)

/* APLL Status Register */

#define SCG_APLLSTAT_NDIVACK_SHIFT               (1)
#define SCG_APLLSTAT_NDIVACK_MASK                (0x01 << SCG_APLLSTAT_NDIVACK_SHIFT)
#define SCG_APLLSTAT_NDIVACK(x)                  (((x) << SCG_APLLSTAT_NDIVACK_SHIFT) & SCG_APLLSTAT_NDIVACK_MASK)

#define SCG_APLLSTAT_MDIVACK_SHIFT               (2)
#define SCG_APLLSTAT_MDIVACK_MASK                (0x01 << SCG_APLLSTAT_MDIVACK_SHIFT)
#define SCG_APLLSTAT_MDIVACK(x)                  (((x) << SCG_APLLSTAT_MDIVACK_SHIFT) & SCG_APLLSTAT_MDIVACK_MASK)

#define SCG_APLLSTAT_PDIVACK_SHIFT               (3)
#define SCG_APLLSTAT_PDIVACK_MASK                (0x01 << SCG_APLLSTAT_PDIVACK_SHIFT)
#define SCG_APLLSTAT_PDIVACK(x)                  (((x) << SCG_APLLSTAT_PDIVACK_SHIFT) & SCG_APLLSTAT_PDIVACK_MASK)

#define SCG_APLLSTAT_FRMDET_SHIFT                (4)
#define SCG_APLLSTAT_FRMDET_MASK                 (0x01 << SCG_APLLSTAT_FRMDET_SHIFT)
#define SCG_APLLSTAT_FRMDET(x)                   (((x) << SCG_APLLSTAT_FRMDET_SHIFT) & SCG_APLLSTAT_FRMDET_MASK)

/* APLL N Divider Register */

#define SCG_APLLNDIV_NDIV_SHIFT                  (0)
#define SCG_APLLNDIV_NDIV_MASK                   (0xff << SCG_APLLNDIV_NDIV_SHIFT)
#define SCG_APLLNDIV_NDIV(x)                     (((x) << SCG_APLLNDIV_NDIV_SHIFT) & SCG_APLLNDIV_NDIV_MASK)

#define SCG_APLLNDIV_NREQ_SHIFT                  (31)
#define SCG_APLLNDIV_NREQ_MASK                   (0x01 << SCG_APLLNDIV_NREQ_SHIFT)
#define SCG_APLLNDIV_NREQ(x)                     (((x) << SCG_APLLNDIV_NREQ_SHIFT) & SCG_APLLNDIV_NREQ_MASK)

/* APLL M Divider Register */

#define SCG_APLLMDIV_MDIV_SHIFT                  (0)
#define SCG_APLLMDIV_MDIV_MASK                   (0xffff << SCG_APLLMDIV_MDIV_SHIFT)
#define SCG_APLLMDIV_MDIV(x)                     (((x) << SCG_APLLMDIV_MDIV_SHIFT) & SCG_APLLMDIV_MDIV_MASK)

#define SCG_APLLMDIV_MREQ_SHIFT                  (31)
#define SCG_APLLMDIV_MREQ_MASK                   (0x01 << SCG_APLLMDIV_MREQ_SHIFT)
#define SCG_APLLMDIV_MREQ(x)                     (((x) << SCG_APLLMDIV_MREQ_SHIFT) & SCG_APLLMDIV_MREQ_MASK)

/* APLL P Divider Register */

#define SCG_APLLPDIV_PDIV_SHIFT                  (0)
#define SCG_APLLPDIV_PDIV_MASK                   (0x1f << SCG_APLLPDIV_PDIV_SHIFT)
#define SCG_APLLPDIV_PDIV(x)                     (((x) << SCG_APLLPDIV_PDIV_SHIFT) & SCG_APLLPDIV_PDIV_MASK)

#define SCG_APLLPDIV_PREQ_SHIFT                  (31)
#define SCG_APLLPDIV_PREQ_MASK                   (0x01 << SCG_APLLPDIV_PREQ_SHIFT)
#define SCG_APLLPDIV_PREQ(x)                     (((x) << SCG_APLLPDIV_PREQ_SHIFT) & SCG_APLLPDIV_PREQ_MASK)

/* APLL LOCK Configuration Register */

#define SCG_APLLLOCK_CNFG_LOCK_TIME_SHIFT        (0)
#define SCG_APLLLOCK_CNFG_LOCK_TIME_MASK         (0x1ffff << SCG_APLLLOCK_CNFG_LOCK_TIME_SHIFT)
#define SCG_APLLLOCK_CNFG_LOCK_TIME(x)           (((x) << SCG_APLLLOCK_CNFG_LOCK_TIME_SHIFT) & SCG_APLLLOCK_CNFG_LOCK_TIME_MASK)

/* APLL SSCG Status Register */

#define SCG_APLLSSCGSTAT_SS_MDIV_ACK_SHIFT       (0)
#define SCG_APLLSSCGSTAT_SS_MDIV_ACK_MASK        (0x01 << SCG_APLLSSCGSTAT_SS_MDIV_ACK_SHIFT)
#define SCG_APLLSSCGSTAT_SS_MDIV_ACK(x)          (((x) << SCG_APLLSSCGSTAT_SS_MDIV_ACK_SHIFT) & SCG_APLLSSCGSTAT_SS_MDIV_ACK_MASK)

/* APLL Spread Spectrum Control 0 Register */

#define SCG_APLLSSCG0_SS_MDIV_LSB_SHIFT          (0)
#define SCG_APLLSSCG0_SS_MDIV_LSB_MASK           (0xffffffff << SCG_APLLSSCG0_SS_MDIV_LSB_SHIFT)
#define SCG_APLLSSCG0_SS_MDIV_LSB(x)             (((x) << SCG_APLLSSCG0_SS_MDIV_LSB_SHIFT) & SCG_APLLSSCG0_SS_MDIV_LSB_MASK)

/* APLL Spread Spectrum Control 1 Register */

#define SCG_APLLSSCG1_SS_MDIV_MSB_SHIFT          (0)
#define SCG_APLLSSCG1_SS_MDIV_MSB_MASK           (0x01 << SCG_APLLSSCG1_SS_MDIV_MSB_SHIFT)
#define SCG_APLLSSCG1_SS_MDIV_MSB(x)             (((x) << SCG_APLLSSCG1_SS_MDIV_MSB_SHIFT) & SCG_APLLSSCG1_SS_MDIV_MSB_MASK)

#define SCG_APLLSSCG1_SS_MDIV_REQ_SHIFT          (1)
#define SCG_APLLSSCG1_SS_MDIV_REQ_MASK           (0x01 << SCG_APLLSSCG1_SS_MDIV_REQ_SHIFT)
#define SCG_APLLSSCG1_SS_MDIV_REQ(x)             (((x) << SCG_APLLSSCG1_SS_MDIV_REQ_SHIFT) & SCG_APLLSSCG1_SS_MDIV_REQ_MASK)

#define SCG_APLLSSCG1_MF_SHIFT                   (2)
#define SCG_APLLSSCG1_MF_MASK                    (0x07 << SCG_APLLSSCG1_MF_SHIFT)
#define SCG_APLLSSCG1_MF(x)                      (((x) << SCG_APLLSSCG1_MF_SHIFT) & SCG_APLLSSCG1_MF_MASK)

#define SCG_APLLSSCG1_MR_SHIFT                   (5)
#define SCG_APLLSSCG1_MR_MASK                    (0x07 << SCG_APLLSSCG1_MR_SHIFT)
#define SCG_APLLSSCG1_MR(x)                      (((x) << SCG_APLLSSCG1_MR_SHIFT) & SCG_APLLSSCG1_MR_MASK)

#define SCG_APLLSSCG1_MC_SHIFT                   (8)
#define SCG_APLLSSCG1_MC_MASK                    (0x03 << SCG_APLLSSCG1_MC_SHIFT)
#define SCG_APLLSSCG1_MC(x)                      (((x) << SCG_APLLSSCG1_MC_SHIFT) & SCG_APLLSSCG1_MC_MASK)

#define SCG_APLLSSCG1_DITHER_SHIFT               (10)
#define SCG_APLLSSCG1_DITHER_MASK                (0x01 << SCG_APLLSSCG1_DITHER_SHIFT)
#define SCG_APLLSSCG1_DITHER(x)                  (((x) << SCG_APLLSSCG1_DITHER_SHIFT) & SCG_APLLSSCG1_DITHER_MASK)

#define SCG_APLLSSCG1_SEL_SS_MDIV_SHIFT          (11)
#define SCG_APLLSSCG1_SEL_SS_MDIV_MASK           (0x01 << SCG_APLLSSCG1_SEL_SS_MDIV_SHIFT)
#define SCG_APLLSSCG1_SEL_SS_MDIV(x)             (((x) << SCG_APLLSSCG1_SEL_SS_MDIV_SHIFT) & SCG_APLLSSCG1_SEL_SS_MDIV_MASK)

#define SCG_APLLSSCG1_SS_PD_SHIFT                (31)
#define SCG_APLLSSCG1_SS_PD_MASK                 (0x01 << SCG_APLLSSCG1_SS_PD_SHIFT)
#define SCG_APLLSSCG1_SS_PD(x)                   (((x) << SCG_APLLSSCG1_SS_PD_SHIFT) & SCG_APLLSSCG1_SS_PD_MASK)

/* APLL Override Register */

#define SCG_APLL_OVRD_APLLPWREN_OVRD_SHIFT       (0)
#define SCG_APLL_OVRD_APLLPWREN_OVRD_MASK        (0x01 << SCG_APLL_OVRD_APLLPWREN_OVRD_SHIFT)
#define SCG_APLL_OVRD_APLLPWREN_OVRD(x)          (((x) << SCG_APLL_OVRD_APLLPWREN_OVRD_SHIFT) & SCG_APLL_OVRD_APLLPWREN_OVRD_MASK)

#define SCG_APLL_OVRD_APLLCLKEN_OVRD_SHIFT       (1)
#define SCG_APLL_OVRD_APLLCLKEN_OVRD_MASK        (0x01 << SCG_APLL_OVRD_APLLCLKEN_OVRD_SHIFT)
#define SCG_APLL_OVRD_APLLCLKEN_OVRD(x)          (((x) << SCG_APLL_OVRD_APLLCLKEN_OVRD_SHIFT) & SCG_APLL_OVRD_APLLCLKEN_OVRD_MASK)

#define SCG_APLL_OVRD_APLL_OVRD_EN_SHIFT         (31)
#define SCG_APLL_OVRD_APLL_OVRD_EN_MASK          (0x01 << SCG_APLL_OVRD_APLL_OVRD_EN_SHIFT)
#define SCG_APLL_OVRD_APLL_OVRD_EN(x)            (((x) << SCG_APLL_OVRD_APLL_OVRD_EN_SHIFT) & SCG_APLL_OVRD_APLL_OVRD_EN_MASK)

/* System PLL Control Status Register */

#define SCG_SPLLCSR_SPLLEN             (1 << 0)  /* Bit 0:  System PLL Enable */
#define SCG_SPLLCSR_SPLLCM             (1 << 16) /* Bit 16: System PLL Clock Monitor */
#define SCG_SPLLCSR_SPLLCMRE           (1 << 17) /* Bit 17: System PLL Clock Monitor Reset Enable */
#define SCG_SPLLCSR_LK                 (1 << 23) /* Bit 23: Lock Register */
#define SCG_SPLLCSR_SPLLVLD            (1 << 24) /* Bit 24: System PLL Valid */
#define SCG_SPLLCSR_SPLLSEL            (1 << 25) /* Bit 25: System PLL Selected */
#define SCG_SPLLCSR_SPLLERR            (1 << 26) /* Bit 65: System PLL Clock Error */

/* System PLL Divide Register */

#define SCG_SPLLDIV_SPLLDIV1_SHIFT     (0)       /* Bits 0-2:  System PLL Clock Divide 1 */
#define SCG_SPLLDIV_SPLLDIV1_MASK      (7 << SCG_SPLLDIV_SPLLDIV1_SHIFT)
#  define SCG_SPLLDIV_SPLLDIV1(n)      ((uint32_t)(n) << SCG_SPLLDIV_SPLLDIV1_SHIFT)
#  define SCG_SPLLDIV_SPLLDIV1_DISABLE (0 << SCG_SPLLDIV_SPLLDIV1_SHIFT)  /* Output disabled */
#  define SCG_SPLLDIV_SPLLDIV1_DIV1    (1 << SCG_SPLLDIV_SPLLDIV1_SHIFT)  /* Divide by 1 */
#  define SCG_SPLLDIV_SPLLDIV1_DIV2    (2 << SCG_SPLLDIV_SPLLDIV1_SHIFT)  /* Divide by 2 */
#  define SCG_SPLLDIV_SPLLDIV1_DIV4    (3 << SCG_SPLLDIV_SPLLDIV1_SHIFT)  /* Divide by 4 */
#  define SCG_SPLLDIV_SPLLDIV1_DIV8    (4 << SCG_SPLLDIV_SPLLDIV1_SHIFT)  /* Divide by 8 */
#  define SCG_SPLLDIV_SPLLDIV1_DIV16   (5 << SCG_SPLLDIV_SPLLDIV1_SHIFT)  /* Divide by 16 */
#  define SCG_SPLLDIV_SPLLDIV1_DIV32   (6 << SCG_SPLLDIV_SPLLDIV1_SHIFT)  /* Divide by 32 */
#  define SCG_SPLLDIV_SPLLDIV1_DIV64   (7 << SCG_SPLLDIV_SPLLDIV1_SHIFT)  /* Divide by 64 */

#define SCG_SPLLDIV_SPLLDIV2_SHIFT     (8)       /* Bits 8-10:  System PLL Clock Divide 2 */
#define SCG_SPLLDIV_SPLLDIV2_MASK      (7 << SCG_SPLLDIV_SPLLDIV2_SHIFT)
#  define SCG_SPLLDIV_SPLLDIV2(n)      ((uint32_t)(n) << SCG_SPLLDIV_SPLLDIV2_SHIFT)
#  define SCG_SPLLDIV_SPLLDIV2_DISABLE (0 << SCG_SPLLDIV_SPLLDIV2_SHIFT)  /* Output disabled */
#  define SCG_SPLLDIV_SPLLDIV2_DIV1    (1 << SCG_SPLLDIV_SPLLDIV2_SHIFT)  /* Divide by 1 */
#  define SCG_SPLLDIV_SPLLDIV2_DIV2    (2 << SCG_SPLLDIV_SPLLDIV2_SHIFT)  /* Divide by 2 */
#  define SCG_SPLLDIV_SPLLDIV2_DIV4    (3 << SCG_SPLLDIV_SPLLDIV2_SHIFT)  /* Divide by 4 */
#  define SCG_SPLLDIV_SPLLDIV2_DIV8    (4 << SCG_SPLLDIV_SPLLDIV2_SHIFT)  /* Divide by 8 */
#  define SCG_SPLLDIV_SPLLDIV2_DIV16   (5 << SCG_SPLLDIV_SPLLDIV2_SHIFT)  /* Divide by 16 */
#  define SCG_SPLLDIV_SPLLDIV2_DIV32   (6 << SCG_SPLLDIV_SPLLDIV2_SHIFT)  /* Divide by 32 */
#  define SCG_SPLLDIV_SPLLDIV2_DIV64   (7 << SCG_SPLLDIV_SPLLDIV2_SHIFT)  /* Divide by 64 */

/* System PLL Configuration Register */

#define SCG_SPLLCFG_PREDIV_SHIFT       (8)     /* Bits 8-10:  PLL Reference Clock Divider */
#define SCG_SPLLCFG_PREDIV_MASK        (7 << SCG_SPLLCFG_PREDIV_SHIFT)
#  define SCG_SPLLCFG_PREDIV(n)        ((uint32_t)((n) - 1) << SCG_SPLLCFG_PREDIV_SHIFT) /* n=1..8 */

#define SCG_SPLLCFG_MULT_SHIFT         (16)     /* Bits 16-20:  System PLL Multiplier */
#define SCG_SPLLCFG_MULT_MASK          (31 << SCG_SPLLCFG_MULT_SHIFT)
#  define SCG_SPLLCFG_MULT(n)          ((uint32_t)((n) - 16) << SCG_SPLLCFG_MULT_SHIFT) /* n=16..47 */

/* LDO Control and Status Register */

#define SCG_LDOCSR_LDOEN               (1 << 0)

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_SCG_H */
