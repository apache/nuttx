/************************************************************************************
 * arch/arm/src/s32k1xx/chip/s32k1xx_scg.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_SCG_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_SCG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* SCG Register Offsets *************************************************************/

#define S32K1XX_SCG_VERID_OFFSET       0x0000  /* Version ID Register */
#define S32K1XX_SCG_PARAM_OFFSET       0x0004  /* Parameter Register */
#define S32K1XX_SCG_CSR_OFFSET         0x0010  /* Clock Status Register */
#define S32K1XX_SCG_RCCR_OFFSET        0x0014  /* Run Clock Control Register */
#define S32K1XX_SCG_VCCR_OFFSET        0x0018  /* VLPR Clock Control Register */
#define S32K1XX_SCG_HCCR_OFFSET        0x001c  /* HSRUN Clock Control Register */
#define S32K1XX_SCG_CLKOUTCNFG_OFFSET  0x0020  /* SCG CLKOUT Configuration Register */
#define S32K1XX_SCG_SOSCCSR_OFFSET     0x0100  /* System OSC Control Status Register */
#define S32K1XX_SCG_SOSCDIV_OFFSET     0x0104  /* System OSC Divide Register */
#define S32K1XX_SCG_SOSCCFG_OFFSET     0x0108  /* System Oscillator Configuration Register */
#define S32K1XX_SCG_SIRCCSR_OFFSET     0x0200  /* Slow IRC Control Status Register */
#define S32K1XX_SCG_SIRCDIV_OFFSET     0x0204  /* Slow IRC Divide Register */
#define S32K1XX_SCG_SIRCCFG_OFFSET     0x0208  /* Slow IRC Configuration Register */
#define S32K1XX_SCG_FIRCCSR_OFFSET     0x0300  /* Fast IRC Control Status Register */
#define S32K1XX_SCG_FIRCDIV_OFFSET     0x0304  /* Fast IRC Divide Register */
#define S32K1XX_SCG_FIRCCFG_OFFSET     0x0308  /* Fast IRC Configuration Register */
#define S32K1XX_SCG_SPLLCSR_OFFSET     0x0600  /* System PLL Control Status Register */
#define S32K1XX_SCG_SPLLDIV_OFFSET     0x0604  /* System PLL Divide Register */
#define S32K1XX_SCG_SPLLCFG_OFFSET     0x0608  /* System PLL Configuration Register */

/* SCG Register Addresses ***********************************************************/

#define S32K1XX_SCG_VERID              (S32K1XX_SCG_BASE + S32K1XX_SCG_VERID_OFFSET)
#define S32K1XX_SCG_PARAM              (S32K1XX_SCG_BASE + S32K1XX_SCG_PARAM_OFFSET)
#define S32K1XX_SCG_CSR                (S32K1XX_SCG_BASE + S32K1XX_SCG_CSR_OFFSET)
#define S32K1XX_SCG_RCCR               (S32K1XX_SCG_BASE + S32K1XX_SCG_RCCR_OFFSET)
#define S32K1XX_SCG_VCCR               (S32K1XX_SCG_BASE + S32K1XX_SCG_VCCR_OFFSET)
#define S32K1XX_SCG_HCCR               (S32K1XX_SCG_BASE + S32K1XX_SCG_HCCR_OFFSET)
#define S32K1XX_SCG_CLKOUTCNFG         (S32K1XX_SCG_BASE + S32K1XX_SCG_CLKOUTCNFG_OFFSET)
#define S32K1XX_SCG_SOSCCSR            (S32K1XX_SCG_BASE + S32K1XX_SCG_SOSCCSR_OFFSET)
#define S32K1XX_SCG_SOSCDIV            (S32K1XX_SCG_BASE + S32K1XX_SCG_SOSCDIV_OFFSET)
#define S32K1XX_SCG_SOSCCFG            (S32K1XX_SCG_BASE + S32K1XX_SCG_SOSCCFG_OFFSET)
#define S32K1XX_SCG_SIRCCSR            (S32K1XX_SCG_BASE + S32K1XX_SCG_SIRCCSR_OFFSET)
#define S32K1XX_SCG_SIRCDIV            (S32K1XX_SCG_BASE + S32K1XX_SCG_SIRCDIV_OFFSET)
#define S32K1XX_SCG_SIRCCFG            (S32K1XX_SCG_BASE + S32K1XX_SCG_SIRCCFG_OFFSET)
#define S32K1XX_SCG_FIRCCSR            (S32K1XX_SCG_BASE + S32K1XX_SCG_FIRCCSR_OFFSET)
#define S32K1XX_SCG_FIRCDIV            (S32K1XX_SCG_BASE + S32K1XX_SCG_FIRCDIV_OFFSET)
#define S32K1XX_SCG_FIRCCFG            (S32K1XX_SCG_BASE + S32K1XX_SCG_FIRCCFG_OFFSET)
#define S32K1XX_SCG_SPLLCSR            (S32K1XX_SCG_BASE + S32K1XX_SCG_SPLLCSR_OFFSET)
#define S32K1XX_SCG_SPLLDIV            (S32K1XX_SCG_BASE + S32K1XX_SCG_SPLLDIV_OFFSET)
#define S32K1XX_SCG_SPLLCFG            (S32K1XX_SCG_BASE + S32K1XX_SCG_SPLLCFG_OFFSET)

/* SCG Register Bitfield Definitions ************************************************/

/* Version ID Register (32-bit version number) */

/* Parameter Register */

#define SCG_PARAM_SOSC                 (1 << 1)  /* System OSC (SOSC) clock present */
#define SCG_PARAM_SIRC                 (1 << 2)  /* Slow IRC (SIRC) clock present */
#define SCG_PARAM_FIRC                 (1 << 3)  /* Fast IRC (FIRC) clock present */
#define SCG_PARAM_SPLL                 (1 << 6)  /* System PLL (SPLL) clock present */
#define SCG_PARAM_DIVSLOW              (1 << 27) /* Bit 27: DIVSLOW clock divider present */
#define SCG_PARAM_DIVBUS               (1 << 28) /* Bit 28: DIVBUS clock divider present */
#define SCG_PARAM_DIVCORE              (1 << 31) /* Bit 31: DIVCORE clock divider present */

/* Clock Status Register */

#define SCG_CSR_DIVSLOW_SHIFT          (0)       /* Bits 0-3:  Slow Clock Divide Ratio */
#define SCG_CSR_DIVSLOW_MASK           (15 << SCG_CSR_DIVSLOW_SHIFT)
#  define SCG_CSR_DIVSLOW(n)           ((uint32_t)((n) - 1) << SCG_CSR_DIVSLOW_SHIFT) /* n=1-8 */
#define SCG_CSR_DIVBUS_SHIFT           (4)       /* Bits 4-7:  Bus Clock Divide Ratio */
#define SCG_CSR_DIVBUS_MASK            (15 << SCG_CSR_DIVBUS_SHIFT)
#  define SCG_CSR_DIVBUS(n)            ((uint32_t)((n) - 1) << SCG_CSR_DIVBUS_SHIFT) /* n=1-16 */
#define SCG_CSR_DIVCORE_SHIFT          (16)      /* Bits 16-19:  Core Clock Divide Ratio */
#define SCG_CSR_DIVCORE_MASK           (15 << SCG_CSR_DIVCORE_SHIFT)
#  define SCG_CSR_DIVCORE(n)           ((uint32_t)((n) - 1) << SCG_CSR_DIVCORE_SHIFT) /* n=1-16 */
#define SCG_CSR_SCS_SHIFT              (24)      /* Bits 24-27:  System clock source */
#define SCG_CSR_SCS_MASK               (15 << SCG_CSR_SCS_SHIFT)
#  define SCG_CSR_SCS_SOSC             (1 << SCG_CSR_SCS_SHIFT)  /* System OSC (SOSC_CLK) */
#  define SCG_CSR_SCS_SIRC             (2 << SCG_CSR_SCS_SHIFT)  /* Slow IRC (SIRC_CLK) */
#  define SCG_CSR_SCS_FIRC             (3 << SCG_CSR_SCS_SHIFT)  /* Fast IRC (FIRC_CLK) */
#  define SCG_CSR_SPLL_FIRC            (6 << SCG_CSR_SCS_SHIFT)  /* System PLL (SPLL_CLK) */

/* Run Clock Control Register */

#define SCG_RCCR_DIVSLOW_SHIFT         (0)       /* Bits 0-3:  Slow Clock Divide Ratio */
#define SCG_RCCR_DIVSLOW_MASK          (15 << SCG_RCCR_DIVSLOW_SHIFT)
#  define SCG_RCCR_DIVSLOW(n)          ((uint32_t)((n) - 1) << SCG_RCCR_DIVSLOW_SHIFT) /* n=1-8 */
#define SCG_RCCR_DIVBUS_SHIFT          (4)       /* Bits 4-7:  Bus Clock Divide Ratio */
#define SCG_RCCR_DIVBUS_MASK           (15 << SCG_RCCR_DIVBUS_SHIFT)
#  define SCG_RCCR_DIVBUS(n)           ((uint32_t)((n) - 1) << SCG_RCCR_DIVBUS_SHIFT) /* n=1-16 */
#define SCG_RCCR_DIVCORE_SHIFT         (16)      /* Bits 16-19:  Core Clock Divide Ratio */
#define SCG_RCCR_DIVCORE_MASK          (15 << SCG_RCCR_DIVCORE_SHIFT)
#  define SCG_RCCR_DIVCORE(n)          ((uint32_t)((n) - 1) << SCG_RCCR_DIVCORE_SHIFT) /* n=1-16 */
#define SCG_RCCR_SCS_SHIFT             (24)      /* Bits 24-27:  System clock source */
#define SCG_RCCR_SCS_MASK              (15 << SCG_RCCR_SCS_SHIFT)
#  define SCG_RCCR_SCS_SOSC            (1 << SCG_RCCR_SCS_SHIFT)  /* System OSC (SOSC_CLK) */
#  define SCG_RCCR_SCS_SIRC            (2 << SCG_RCCR_SCS_SHIFT)  /* Slow IRC (SIRC_CLK) */
#  define SCG_RCCR_SCS_FIRC            (3 << SCG_RCCR_SCS_SHIFT)  /* Fast IRC (FIRC_CLK) */
#  define SCG_RCCR_SPLL_FIRC           (6 << SCG_RCCR_SCS_SHIFT)  /* System PLL (SPLL_CLK) */

/* VLPR Clock Control Register */

#define SCG_VCCR_DIVSLOW_SHIFT         (0)       /* Bits 0-3:  Slow Clock Divide Ratio */
#define SCG_VCCR_DIVSLOW_MASK          (15 << SCG_VCCR_DIVSLOW_SHIFT)
#  define SCG_VCCR_DIVSLOW(n)          ((uint32_t)((n) - 1) << SCG_VCCR_DIVSLOW_SHIFT) /* n=1-8 */
#define SCG_VCCR_DIVBUS_SHIFT          (4)       /* Bits 4-7:  Bus Clock Divide Ratio */
#define SCG_VCCR_DIVBUS_MASK           (15 << SCG_VCCR_DIVBUS_SHIFT)
#  define SCG_VCCR_DIVBUS(n)           ((uint32_t)((n) - 1) << SCG_VCCR_DIVBUS_SHIFT) /* n=1-16 */
#define SCG_VCCR_DIVCORE_SHIFT         (16)      /* Bits 16-19:  Core Clock Divide Ratio */
#define SCG_VCCR_DIVCORE_MASK          (15 << SCG_VCCR_DIVCORE_SHIFT)
#  define SCG_VCCR_DIVCORE(n)          ((uint32_t)((n) - 1) << SCG_VCCR_DIVCORE_SHIFT) /* n=1-16 */
#define SCG_VCCR_SCS_SHIFT             (24)      /* Bits 24-27:  System clock source */
#define SCG_VCCR_SCS_MASK              (15 << SCG_VCCR_SCS_SHIFT)
#  define SCG_VCCR_SCS_SIRC            (2 << SCG_VCCR_SCS_SHIFT)  /* Slow IRC (SIRC_CLK) */

/* HSRUN Clock Control Register */

#define SCG_HCCR_DIVSLOW_SHIFT         (0)       /* Bits 0-3:  Slow Clock Divide Ratio */
#define SCG_HCCR_DIVSLOW_MASK          (15 << SCG_HCCR_DIVSLOW_SHIFT)
#  define SCG_HCCR_DIVSLOW(n)          ((uint32_t)((n) - 1) << SCG_HCCR_DIVSLOW_SHIFT) /* n=1-8 */
#define SCG_HCCR_DIVBUS_SHIFT          (4)       /* Bits 4-7:  Bus Clock Divide Ratio */
#define SCG_HCCR_DIVBUS_MASK           (15 << SCG_HCCR_DIVBUS_SHIFT)
#  define SCG_HCCR_DIVBUS(n)           ((uint32_t)((n) - 1) << SCG_HCCR_DIVBUS_SHIFT) /* n=1-16 */
#define SCG_HCCR_DIVCORE_SHIFT         (16)      /* Bits 16-19:  Core Clock Divide Ratio */
#define SCG_HCCR_DIVCORE_MASK          (15 << SCG_HCCR_DIVCORE_SHIFT)
#  define SCG_HCCR_DIVCORE(n)          ((uint32_t)((n) - 1) << SCG_HCCR_DIVCORE_SHIFT) /* n=1-16 */
#define SCG_HCCR_SCS_SHIFT             (24)      /* Bits 24-27:  System clock source */
#define SCG_HCCR_SCS_MASK              (15 << SCG_HCCR_SCS_SHIFT)
#  define SCG_HCCR_SCS_FIRC            (3 << SCG_HCCR_SCS_SHIFT)  /* Fast IRC (FIRC_CLK) */
#  define SCG_HCCR_SPLL_FIRC           (6 << SCG_HCCR_SCS_SHIFT)  /* System PLL (SPLL_CLK) */

/* SCG CLKOUT Configuration Register */

#define SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT  (24)      /* Bits 24-27:  SCG Clkout Select */
#define SCG_CLKOUTCNFG_CLKOUTSEL_MASK   (15 << SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT)
#  define SCG_CLKOUTCNFG_CLKOUTSEL(src) ((uint32_t)(src) << SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT)
#  define SCG_CLKOUTCNFG_CLKOUTSEL_SOSC (1 << SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT)  /* System OSC (SOSC_CLK) */
#  define SCG_CLKOUTCNFG_CLKOUTSEL_SIRC (2 << SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT)  /* Slow IRC (SIRC_CLK) */
#  define SCG_CLKOUTCNFG_CLKOUTSEL_FIRC (3 << SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT)  /* Fast IRC (FIRC_CLK) */
#  define SCG_CLKOUTCNFG_CLKOUTSEL_SPLL (6 << SCG_CLKOUTCNFG_CLKOUTSEL_SHIFT)  /* System PLL (SPLL_CLK) */

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
#define SCG_FIRCCSR_FIRCREGOFF         (1 << 3)  /* Bit 3:  Fast IRC Regulator Enable */
#define SCG_FIRCCSR_LK                 (1 << 23) /* Bit 23: Lock Register */
#define SCG_FIRCCSR_FIRCVLD            (1 << 24) /* Bit 24: Fast IRC Valid status */
#define SCG_FIRCCSR_FIRCSEL            (1 << 25) /* Bit 25: Fast IRC Selected status */
#define SCG_FIRCCSR_FIRCERR            (1 << 26) /* Bit 26: Fast IRC Clock Error */

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

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_SCG_H */
