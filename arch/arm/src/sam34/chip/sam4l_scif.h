/****************************************************************************************
 * arch/arm/src/sam34/chip/sam4l_scif.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_SCIF_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_SCIF_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* SCIF register offsets ****************************************************************/

#define SAM_SCIF_IER_OFFSET               0x0000 /* Interrupt Enable Register */
#define SAM_SCIF_IDR_OFFSET               0x0004 /* Interrupt Disable Register */
#define SAM_SCIF_IMR_OFFSET               0x0008 /* Interrupt Mask Register */
#define SAM_SCIF_ISR_OFFSET               0x000c /* Interrupt Status Register */
#define SAM_SCIF_ICR_OFFSET               0x0010 /* Interrupt Clear Register */
#define SAM_SCIF_PCLKSR_OFFSET            0x0014 /* Power and Clocks Status Register */
#define SAM_SCIF_UNLOCK_OFFSET            0x0018 /* Unlock Register */
#define SAM_SCIF_CSCR_OFFSET              0x001c /* Chip Specific Configuration Register */
#define SAM_SCIF_OSCCTRL0_OFFSET          0x0020 /* Oscillator Control Register */
#define SAM_SCIF_PLL0_OFFSET              0x0024 /* PLL0 Control Register */
#define SAM_SCIF_DFLL0CONF_OFFSET         0x0028 /* DFLL0 Config Register */
#define SAM_SCIF_DFLL0VAL_OFFSET          0x002c /* DFLL Value Register */
#define SAM_SCIF_DFLL0MUL_OFFSET          0x0030 /* DFLL0 Multiplier Register  */
#define SAM_SCIF_DFLL0STEP_OFFSET         0x0034 /* DFLL0 Step Register  */
#define SAM_SCIF_DFLL0SSG_OFFSET          0x0038 /* DFLL0 Spread Spectrum Generator Control Register */
#define SAM_SCIF_DFLL0RATIO_OFFSET        0x003c /* DFLL0 Ratio Register */
#define SAM_SCIF_DFLL0SYNC_OFFSET         0x0040 /* DFLL0 Synchronization Register */
#define SAM_SCIF_RCCR_OFFSET              0x0044 /* System RC Oscillator Calibration Register */
#define SAM_SCIF_RCFASTCFG_OFFSET         0x0048 /* 4/8/12MHz RC Oscillator Configuration Register */
#define SAM_SCIF_RCFASTSR_OFFSET          0x004c /* 4/8/12MHz RC Oscillator Status Register */
#define SAM_SCIF_RC80MCR_OFFSET           0x0050 /* 80MHz RC Oscillator Register */
#define SAM_SCIF_HRPCR_OFFSET             0x0064 /* High Resolution Prescaler Control Register */
#define SAM_SCIF_FPCR_OFFSET              0x0068 /* Fractional Prescaler Control Register */
#define SAM_SCIF_FPMUL_OFFSET             0x006c /* Fractional Prescaler Multiplier Register */
#define SAM_SCIF_FPDIV_OFFSET             0x0070 /* Fractional Prescaler DIVIDER Register */
#define SAM_SCIF_GCCTRL0_OFFSET           0x0074 /* Generic Clock Control0 */
#define SAM_SCIF_GCCTRL1_OFFSET           0x0078 /* Generic Clock Control1 */
#define SAM_SCIF_GCCTRL2_OFFSET           0x007c /* Generic Clock Control2 */
#define SAM_SCIF_GCCTRL3_OFFSET           0x0080 /* Generic Clock Control3 */
#define SAM_SCIF_GCCTRL4_OFFSET           0x0084 /* Generic Clock Control4 */
#define SAM_SCIF_GCCTRL5_OFFSET           0x0088 /* Generic Clock Control5 */
#define SAM_SCIF_GCCTRL6_OFFSET           0x008c /* Generic Clock Control6 */
#define SAM_SCIF_GCCTRL7_OFFSET           0x0090 /* Generic Clock Control7 */
#define SAM_SCIF_GCCTRL8_OFFSET           0x0094 /* Generic Clock Control8 */
#define SAM_SCIF_GCCTRL9_OFFSET           0x0098 /* Generic Clock Control9 */
#define SAM_SCIF_GCCTRL10_OFFSET          0x009c /* Generic Clock Control10 */
#define SAM_SCIF_GCCTRL11_OFFSET          0x00a0 /* Generic Clock Control11 */
#define SAM_SCIF_RCFASTVERSION_OFFSET     0x03d8 /* 4/8/12MHz RC Oscillator Version Register */
#define SAM_SCIF_GCLKPRESCVERSION_OFFSET  0x03dc /* Generic Clock Prescaler Version Register */
#define SAM_SCIF_PLLIFAVERSION_OFFSET     0x03e0 /* PLL Version Register */
#define SAM_SCIF_OSCIFAVERSION_OFFSET     0x03e4 /* Oscillator0 Version Register */
#define SAM_SCIF_DFLLIFBVERSION_OFFSET    0x03e8 /* DFLL Version Register */
#define SAM_SCIF_RCOSCIFAVERSION_OFFSET   0x03ec /* System RC Oscillator Version Register */
#define SAM_SCIF_RC80MVERSION_OFFSET      0x03f4 /* 80MHz RC Oscillator Version Register */
#define SAM_SCIF_GCLKVERSION_OFFSET       0x03f8 /* Generic Clock Version Register */
#define SAM_SCIF_VERSION_OFFSET           0x03fc /* SCIF Version Register */

/* SCIF register adresses ***************************************************************/

#define SAM_SCIF_IER                      (SAM_SCIF_BASE+SAM_SCIF_IER_OFFSET)
#define SAM_SCIF_IDR                      (SAM_SCIF_BASE+SAM_SCIF_IDR_OFFSET)
#define SAM_SCIF_IMR                      (SAM_SCIF_BASE+SAM_SCIF_IMR_OFFSET)
#define SAM_SCIF_ISR                      (SAM_SCIF_BASE+SAM_SCIF_ISR_OFFSET)
#define SAM_SCIF_ICR                      (SAM_SCIF_BASE+SAM_SCIF_ICR_OFFSET)
#define SAM_SCIF_PCLKSR                   (SAM_SCIF_BASE+SAM_SCIF_PCLKSR_OFFSET)
#define SAM_SCIF_UNLOCK                   (SAM_SCIF_BASE+SAM_SCIF_UNLOCK_OFFSET)
#define SAM_SCIF_CSCR                     (SAM_SCIF_BASE+SAM_SCIF_CSCR_OFFSET)
#define SAM_SCIF_OSCCTRL0                 (SAM_SCIF_BASE+SAM_SCIF_OSCCTRL0_OFFSET)
#define SAM_SCIF_PLL0                     (SAM_SCIF_BASE+SAM_SCIF_PLL0_OFFSET)
#define SAM_SCIF_DFLL0CONF                (SAM_SCIF_BASE+SAM_SCIF_DFLL0CONF_OFFSET)
#define SAM_SCIF_DFLL0VAL                 (SAM_SCIF_BASE+SAM_SCIF_DFLL0VAL_OFFSET)
#define SAM_SCIF_DFLL0MUL                 (SAM_SCIF_BASE+SAM_SCIF_DFLL0MUL_OFFSET)
#define SAM_SCIF_DFLL0STEP                (SAM_SCIF_BASE+SAM_SCIF_DFLL0STEP_OFFSET)
#define SAM_SCIF_DFLL0SSG                 (SAM_SCIF_BASE+SAM_SCIF_DFLL0SSG_OFFSET)
#define SAM_SCIF_DFLL0RATIO               (SAM_SCIF_BASE+SAM_SCIF_DFLL0RATIO_OFFSET)
#define SAM_SCIF_DFLL0SYNC                (SAM_SCIF_BASE+SAM_SCIF_DFLL0SYNC_OFFSET)
#define SAM_SCIF_RCCR                     (SAM_SCIF_BASE+SAM_SCIF_RCCR_OFFSET)
#define SAM_SCIF_RCFASTCFG                (SAM_SCIF_BASE+SAM_SCIF_RCFASTCFG_OFFSET)
#define SAM_SCIF_RCFASTSR                 (SAM_SCIF_BASE+SAM_SCIF_RCFASTSR_OFFSET)
#define SAM_SCIF_RC80MCR                  (SAM_SCIF_BASE+SAM_SCIF_RC80MCR_OFFSET)
#define SAM_SCIF_HRPCR                    (SAM_SCIF_BASE+SAM_SCIF_HRPCR_OFFSET)
#define SAM_SCIF_FPCR                     (SAM_SCIF_BASE+SAM_SCIF_FPCR_OFFSET)
#define SAM_SCIF_FPMUL                    (SAM_SCIF_BASE+SAM_SCIF_FPMUL_OFFSET)
#define SAM_SCIF_FPDIV                    (SAM_SCIF_BASE+SAM_SCIF_FPDIV_OFFSET)
#define SAM_SCIF_GCCTRL0                  (SAM_SCIF_BASE+SAM_SCIF_GCCTRL0_OFFSET)
#define SAM_SCIF_GCCTRL1                  (SAM_SCIF_BASE+SAM_SCIF_GCCTRL1_OFFSET)
#define SAM_SCIF_GCCTRL2                  (SAM_SCIF_BASE+SAM_SCIF_GCCTRL2_OFFSET)
#define SAM_SCIF_GCCTRL3                  (SAM_SCIF_BASE+SAM_SCIF_GCCTRL3_OFFSET)
#define SAM_SCIF_GCCTRL4                  (SAM_SCIF_BASE+SAM_SCIF_GCCTRL4_OFFSET)
#define SAM_SCIF_GCCTRL5                  (SAM_SCIF_BASE+SAM_SCIF_GCCTRL5_OFFSET)
#define SAM_SCIF_GCCTRL6                  (SAM_SCIF_BASE+SAM_SCIF_GCCTRL6_OFFSET)
#define SAM_SCIF_GCCTRL7                  (SAM_SCIF_BASE+SAM_SCIF_GCCTRL7_OFFSET)
#define SAM_SCIF_GCCTRL8                  (SAM_SCIF_BASE+SAM_SCIF_GCCTRL8_OFFSET)
#define SAM_SCIF_GCCTRL9                  (SAM_SCIF_BASE+SAM_SCIF_GCCTRL9_OFFSET)
#define SAM_SCIF_GCCTRL10                 (SAM_SCIF_BASE+SAM_SCIF_GCCTRL10_OFFSET)
#define SAM_SCIF_GCCTRL11                 (SAM_SCIF_BASE+SAM_SCIF_GCCTRL11_OFFSET)
#define SAM_SCIF_RCFASTVERSION            (SAM_SCIF_BASE+SAM_SCIF_RCFASTVERSION_OFFSET)
#define SAM_SCIF_GCLKPRESCVERSION         (SAM_SCIF_BASE+SAM_SCIF_GCLKPRESCVERSION_OFFSET)
#define SAM_SCIF_PLLIFAVERSION            (SAM_SCIF_BASE+SAM_SCIF_PLLIFAVERSION_OFFSET)
#define SAM_SCIF_OSCIFAVERSION            (SAM_SCIF_BASE+SAM_SCIF_OSCIFAVERSION_OFFSET)
#define SAM_SCIF_DFLLIFBVERSION           (SAM_SCIF_BASE+SAM_SCIF_DFLLIFBVERSION_OFFSET)
#define SAM_SCIF_RCOSCIFAVERSION          (SAM_SCIF_BASE+SAM_SCIF_RCOSCIFAVERSION_OFFSET)
#define SAM_SCIF_RC80MVERSION             (SAM_SCIF_BASE+SAM_SCIF_RC80MVERSION_OFFSET)
#define SAM_SCIF_GCLKVERSION              (SAM_SCIF_BASE+SAM_SCIF_GCLKVERSION_OFFSET)
#define SAM_SCIF_VERSION                  (SAM_SCIF_BASE+SAM_SCIF_VERSION_OFFSET)

/* SCIF register bit definitions ********************************************************/

/* Interrupt Enable Register */
/* Interrupt Disable Register */
/* Interrupt Mask Register */
/* Interrupt Status Register */
/* Interrupt Clear Register */
/* Power and Clocks Status Register */

#define SCIF_INT_OSC0RDY                  (1 << 0)  /* Bit 0:  OSC0 Ready */
#define SCIF_INT_DFLL0LOCKC               (1 << 1)  /* Bit 1:  DFLL0 Locked on Coarse Value */
#define SCIF_INT_DFLL0LOCKF               (1 << 2)  /* Bit 2:  DFLL0 Locked on Fine Value */
#define SCIF_INT_DFLL0RDY                 (1 << 3)  /* Bit 3:  DFLL0 Synchronization Ready */
#define SCIF_INT_DFLL0RCS                 (1 << 4)  /* Bit 4:  DFLL0 Reference Clock Stopped */
#define SCIF_INT_PLL0LOCK                 (1 << 6)  /* Bit 6:  PLL0 Locked on Accurate value */
#define SCIF_INT_PLL0LOCKLOST             (1 << 7)  /* Bit 7:  PLL0 lock lost value */
#define SCIF_INT_RCFASTLOCK               (1 << 13) /* Bit 13: RCFAST Locked on Accurate value */
#define SCIF_INT_RCFASTLOCKLOST           (1 << 14) /* Bit 14: RCFAST lock lost value */

/* Unlock Register */

#define SCIF_UNLOCK_ADDR_SHIFT            (0)       /* Bits 0-9: Unlock Address */
#define SCIF_UNLOCK_ADDR_MASK             (0x3ff << SCIF_UNLOCK_ADDR_SHIFT)
#  define SCIF_UNLOCK_ADDR(n)             ((n) << SCIF_UNLOCK_ADDR_SHIFT)
#define SCIF_UNLOCK_KEY_SHIFT             (24)      /* Bits 24-31: Unlock Key */
#define SCIF_UNLOCK_KEY_MASK              (0xff << SCIF_UNLOCK_KEY_SHIFT)
#  define SCIF_UNLOCK_KEY(n)              ((n) << SCIF_UNLOCK_KEY_SHIFT)

/* Chip Specific Configuration Register */

/* Oscillator Control Register */

#define SCIF_OSCCTRL0_MODE                (1 << 0)  /* Bit 0:  Oscillator Mode */
#define SCIF_OSCCTRL0_GAIN_SHIFT          (1)       /* Bits 1-2: Gain */
#define SCIF_OSCCTRL0_GAIN_MASK           (3 << SCIF_OSCCTRL0_GAIN_SHIFT)
#  define SCIF_OSCCTRL0_GAIN(n)           ((n) << SCIF_OSCCTRL0_GAIN_SHIFT)
#define SCIF_OSCCTRL0_AGC                 (1 << 3)  /* Bit 3:  Automatic Gain Control */
#define SCIF_OSCCTRL0_STARTUP_SHIFT       (9)       /* Bits 8-11: Oscillator Start-up Time */
#define SCIF_OSCCTRL0_STARTUP_MASK        (15 << SCIF_OSCCTRL0_STARTUP_SHIFT)
#  define SCIF_OSCCTRL0_STARTUP_0         (0 << SCIF_OSCCTRL0_STARTUP_SHIFT)
#  define SCIF_OSCCTRL0_STARTUP_64        (1 << SCIF_OSCCTRL0_STARTUP_SHIFT)  /* 64 557 us */
#  define SCIF_OSCCTRL0_STARTUP_128       (2 << SCIF_OSCCTRL0_STARTUP_SHIFT)  /* 128 1.1 ms */
#  define SCIF_OSCCTRL0_STARTUP_2K        (3 << SCIF_OSCCTRL0_STARTUP_SHIFT)  /* 2048 18 ms */
#  define SCIF_OSCCTRL0_STARTUP_4K        (4 << SCIF_OSCCTRL0_STARTUP_SHIFT)  /* 4096 36 ms */
#  define SCIF_OSCCTRL0_STARTUP_8K        (5 << SCIF_OSCCTRL0_STARTUP_SHIFT)  /* 8192 71 ms */
#  define SCIF_OSCCTRL0_STARTUP_16K       (6 << SCIF_OSCCTRL0_STARTUP_SHIFT)  /* 16384 143 ms */
#  define SCIF_OSCCTRL0_STARTUP_32K       (7 << SCIF_OSCCTRL0_STARTUP_SHIFT)  /* 32768 285 ms */
#  define SCIF_OSCCTRL0_STARTUP_4         (8 << SCIF_OSCCTRL0_STARTUP_SHIFT)  /* 4 35 us */
#  define SCIF_OSCCTRL0_STARTUP_8         (9 << SCIF_OSCCTRL0_STARTUP_SHIFT)  /* 8 70 us */
#  define SCIF_OSCCTRL0_STARTUP_16        (10 << SCIF_OSCCTRL0_STARTUP_SHIFT) /* 16 139 us */
#  define SCIF_OSCCTRL0_STARTUP_32        (11 << SCIF_OSCCTRL0_STARTUP_SHIFT) /* 32 278 us */
#  define SCIF_OSCCTRL0_STARTUP_256       (12 << SCIF_OSCCTRL0_STARTUP_SHIFT) /* 256 2.2 ms */
#  define SCIF_OSCCTRL0_STARTUP_512       (13 << SCIF_OSCCTRL0_STARTUP_SHIFT) /* 512 4.5 ms */
#  define SCIF_OSCCTRL0_STARTUP_1K        (14 << SCIF_OSCCTRL0_STARTUP_SHIFT) /* 1024 8.9 ms */
#  define SCIF_OSCCTRL0_STARTUP_32K2      (15 << SCIF_OSCCTRL0_STARTUP_SHIFT) /* 2768 285 ms */
#define SCIF_OSCCTRL0_OSCEN               (1 << 16) /* Bit 16: Oscillator Enable */

/* PLL0 Control Register */

#define SCIF_PLL0_PLLEN                   (1 << 0)  /* Bit 0:  PLL Enable */
#define SCIF_PLL0_PLLOSC_SHIFT            (1)       /* Bits 1-2: PLL Oscillator Select */
#define SCIF_PLL0_PLLOSC_MASK             (3 << SCIF_PLL0_PLLOSC_SHIFT)
#  define SCIF_PLL0_PLLOSC_OSC0           (0 << SCIF_PLL0_PLLOSC_SHIFT) /* Output clock from Oscillator0 */
#  define SCIF_PLL0_PLLOSC_GCLK9          (1 << SCIF_PLL0_PLLOSC_SHIFT) /* Generic clock 9 */
#define SCIF_PLL0_PLLOPT_SHIFT            (3)       /* Bits 3-5: PLL Option */
#define SCIF_PLL0_PLLOPT_MASK             (7 << SCIF_PLL0_PLLOPT_SHIFT)
#  define SCIF_PLL0_PLLOPT_FVO            (1 << SCIF_PLL0_PLLOPT_SHIFT) /* Selects the VCO frequency range (fvco) */
#  define SCIF_PLL0_PLLOPT_DIV2           (2 << SCIF_PLL0_PLLOPT_SHIFT) /* Divides the output frequency by 2 */
#  define SCIF_PLL0_PLLOPT_WBM            (4 << SCIF_PLL0_PLLOPT_SHIFT) /* Wide-Bandwidth mode */
#define SCIF_PLL0_PLLDIV_SHIFT            (8)       /* Bits 8-11: PLL Division Factor */
#define SCIF_PLL0_PLLDIV_MASK             (15 << SCIF_PLL0_PLLDIV_SHIFT)
#define SCIF_PLL0_PLLMUL_SHIFT            (16)      /* Bits 16-19: PLL Multiply Factor */
#define SCIF_PLL0_PLLMUL_MASK             (15 << SCIF_PLL0_PLLMUL_SHIFT)
#define SCIF_PLL0_PLLCOUNT_SHIFT          (24)      /* Bits 24-24: PLL Count */
#define SCIF_PLL0_PLLCOUNT_MASK           (63 << SCIF_PLL0_PLLCOUNT_SHIFT)
#  define SCIF_PLL0_PLLCOUNT_MAX          (63 << SCIF_PLL0_PLLCOUNT_SHIFT)

/* PLL0 operates in two frequency ranges as determined by SCIF_PLL0_PLLOPT_FVO:
 *
 * 0: 80MHz  < fvco < 180MHz
 * 1: 160MHz < fvco < 240MHz
 *
 * These ranges and recommend threshold value are defined below:
 */

#define SCIF_PLL0_VCO_RANGE1_MINFREQ      160000000
#define SCIF_PLL0_VCO_RANGE1_MAXFREQ      240000000
#define SCIF_PLL0_VCO_RANGE0_MINFREQ       80000000
#define SCIF_PLL0_VCO_RANGE0_MAXFREQ      180000000

#define SAM_PLL0_VCO_RANGE_THRESHOLD \
  ((SCIF_PLL0_VCO_RANGE1_MINFREQ + SCIF_PLL0_VCO_RANGE0_MAXFREQ) >> 1)

/* DFLL0 Config Register */

#define SCIF_DFLL0CONF_EN                 (1 << 0)  /* Bit 0:  Enable */
#define SCIF_DFLL0CONF_MODE               (1 << 1)  /* Bit 1:  Mode Selection */
#define SCIF_DFLL0CONF_STABLE             (1 << 2)  /* Bit 2:  Stable DFLL Frequency */
#define SCIF_DFLL0CONF_LLAW               (1 << 3)  /* Bit 3:  Lose Lock After Wake */
#define SCIF_DFLL0CONF_CCDIS              (1 << 5)  /* Bit 5:  Chill Cycle Disable */
#define SCIF_DFLL0CONF_QLDIS              (1 << 6)  /* Bit 6:  Quick Lock Disable */
#define SCIF_DFLL0CONF_RANGE_SHIFT        (16)      /* Bits 16-17: Range Value */
#define SCIF_DFLL0CONF_RANGE_MASK         (3 << SCIF_DFLL0CONF_RANGE_SHIFT)
#  define SCIF_DFLL0CONF_RANGE(n)         ((n) << SCIF_DFLL0CONF_RANGE_SHIFT)
#  define SCIF_DFLL0CONF_RANGE0           (0 << SCIF_DFLL0CONF_RANGE_SHIFT) /* 96-150MHz */
#  define SCIF_DFLL0CONF_RANGE1           (1 << SCIF_DFLL0CONF_RANGE_SHIFT) /* 50-110MHz */
#  define SCIF_DFLL0CONF_RANGE2           (2 << SCIF_DFLL0CONF_RANGE_SHIFT) /* 25-55MHz */
#  define SCIF_DFLL0CONF_RANGE3           (3 << SCIF_DFLL0CONF_RANGE_SHIFT) /* 20-30MHz */
#define SCIF_DFLL0CONF_FCD                (1 << 23) /* Bit 23: Fuse Calibration Done */
#define SCIF_DFLL0CONF_CALIB_SHIFT        (24)      /* Bits 24-27: Calibration Value */
#define SCIF_DFLL0CONF_CALIB_MASK         (15 << SCIF_DFLL0CONF_CALIB_SHIFT)

/* Min/max frequencies for each DFLL0 range */

#define SCIF_DFLL0CONF_MAX_RANGE0   (150000000)
#define SCIF_DFLL0CONF_MIN_RANGE0    (96000000)
#define SCIF_DFLL0CONF_MAX_RANGE1   (110000000)
#define SCIF_DFLL0CONF_MIN_RANGE1    (50000000)
#define SCIF_DFLL0CONF_MAX_RANGE2    (55000000)
#define SCIF_DFLL0CONF_MIN_RANGE2    (25000000)
#define SCIF_DFLL0CONF_MAX_RANGE3    (30000000)
#define SCIF_DFLL0CONF_MIN_RANGE3    (20000000)

/* DFLL Value Register */

#define SCIF_DFLL0VAL_FINE_SHIFT          (0)      /* Bits 0-7: Fine Value */
#define SCIF_DFLL0VAL_FINE_MASK           (0xff << SCIF_DFLL0VAL_FINE_SHIFT)
#define SCIF_DFLL0VAL_COARSE_SHIFT        (16)     /* Bits 16-20: Coarse value */
#define SCIF_DFLL0VAL_COARSE_MASK         (31 << SCIF_DFLL0VAL_COARSE_SHIFT)

/* DFLL0 Multiplier Register  */

#define SCIF_DFLL0MUL_MASK                0xffff

/* DFLL0 Step Register  */

#define SCIF_DFLL0STEP_FSTEP_SHIFT        (0)      /* Bits 0-7: Fine Maximum Step */
#define SCIF_DFLL0STEP_FSTEP_MASK         (0xff << SCIF_DFLL0STEP_FSTEP_SHIFT)
#  define SCIF_DFLL0STEP_FSTEP(n)         ((n) << SCIF_DFLL0STEP_FSTEP_SHIFT)
#define SCIF_DFLL0STEP_CSTEP_SHIFT        (16)     /* Bits 16-20: Coarse Maximum Step */
#define SCIF_DFLL0STEP_CSTEP_MASK         (31 << SCIF_DFLL0STEP_CSTEP_SHIFT)
#  define SCIF_DFLL0STEP_CSTEP(n)         ((n) << SCIF_DFLL0STEP_CSTEP_SHIFT)

/* DFLL0 Spread Spectrum Generator Control Register */

#define SCIF_DFLL0SSG_EN                  (1 << 0)  /* Bit 0:  Enable */
#define SCIF_DFLL0SSG_PRBS                (1 << 1)  /* Bit 1:  Pseudo Random Bit Sequence */
#define SCIF_DFLL0SSG_AMPLITUDE_SHIFT     (8)       /* Bits 8-12: SSG Amplitude */
#define SCIF_DFLL0SSG_AMPLITUDE_MASK      (31 << SCIF_DFLL0SSG_AMPLITUDE_SHIFT)
#define SCIF_DFLL0SSG_STEPSIZE_SHIFT      (16)      /* Bits 16-20: SSG Step Size */
#define SCIF_DFLL0SSG_STEPSIZE_MASK       (31 << SCIF_DFLL0SSG_STEPSIZE_SHIFT)

/* DFLL0 Ratio Register */

#define SCIF_DFLL0RATIO_MASK              0xffff

/* DFLL0 Synchronization Register */

#define SCIF_DFLL0SYNC_SYNC               (1 << 0)  /* Bit 0:  Synchronization */

/* System RC Oscillator Calibration Register */

#define SCIF_RCCR_CALIB_SHIFT             (0)       /* Bits 0-9: Calibration Value */
#define SCIF_RCCR_CALIB_MASK              (0x3ff << SCIF_RCCR_CALIB_SHIFT)
#define SCIF_RCCR_FCD                     (1 << 16) /* Bit 16: Flash Calibration Done */

/* 4/8/12MHz RC Oscillator Configuration Register */

#define SCIF_RCFASTCFG_EN                 (1 << 0)  /* Bit 0:  Oscillator Enable */
#define SCIF_RCFASTCFG_TUNEEN             (1 << 1)  /* Bit 1:  Tuner Enable */
#define SCIF_RCFASTCFG_JITMODE            (1 << 2)  /* Bit 2:  Jitter Mode */
#define SCIF_RCFASTCFG_NBPERIODS_SHIFT    (4)       /* Bits 4-6: Number of 32kHz Periods */
#define SCIF_RCFASTCFG_NBPERIODS_MASK     (7 << SCIF_RCFASTCFG_NBPERIODS_SHIFT)
#define SCIF_RCFASTCFG_FCD                (1 << 7)  /* Bit 7:  RCFAST Fuse Calibration Done */
#define SCIF_RCFASTCFG_FRANGE_SHIFT       (8)       /* Bits 8-9: Frequency Range */
#define SCIF_RCFASTCFG_FRANGE_MASK        (3 << SCIF_RCFASTCFG_FRANGE_SHIFT)
#  define SCIF_RCFASTCFG_FRANGE_4MHZ      (0 << SCIF_RCFASTCFG_FRANGE_SHIFT) /* 4MHz range selected */
#  define SCIF_RCFASTCFG_FRANGE_8MHZ      (1 << SCIF_RCFASTCFG_FRANGE_SHIFT) /* 8MHz range selected */
#  define SCIF_RCFASTCFG_FRANGE_12MHZ     (2 << SCIF_RCFASTCFG_FRANGE_SHIFT) /* 12MHz range selected */
#define SCIF_RCFASTCFG_LOCKMARGIN_SHIFT   (12)      /* Bits 12-15: Accepted Count Error for Lock */
#define SCIF_RCFASTCFG_LOCKMARGIN_MASK    (15 << SCIF_RCFASTCFG_LOCKMARGIN_SHIFT)
#define SCIF_RCFASTCFG_CALIB_SHIFT        (16)      /* Bits 16-22: Oscillator Calibration Value */
#define SCIF_RCFASTCFG_CALIB_MASK         (0x7f << SCIF_RCFASTCFG_CALIB_SHIFT)

/* 4/8/12MHz RC Oscillator Status Register */

#define SCIF_RCFASTSR_CURTRIM_SHIFT       (0)       /* Bits 0-6: Current Trim Value */
#define SCIF_RCFASTSR_CURTRIM_MASK        (0x7f << SCIF_RCFASTSR_CURTRIM_SHIFT)
#define SCIF_RCFASTSR_CNTERR_SHIFT        (16)      /* Bits 16-20: Current Count Error */
#define SCIF_RCFASTSR_CNTERR_MASK         (31 << SCIF_RCFASTSR_CNTERR_SHIFT)
#define SCIF_RCFASTSR_SIGN                (1 << 21) /* Bit 21: Sign of Current Count Error */
#define SCIF_RCFASTSR_LOCK                (1 << 24) /* Bit 24: Lock */
#define SCIF_RCFASTSR_LOCKLOST            (1 << 25) /* Bit 25: Lock Lost */
#define SCIF_RCFASTSR_UPDATED             (1 << 31) /* Bit 31: Current Trim Value Updated */

/* 80MHz RC Oscillator Register */

#define SCIF_RC80MCR_EN                   (1 << 0)  /* Bit 0:  Enable */
#define SCIF_RC80MCR_FCD                  (1 << 7)  /* Bit 7:  Flash Calibration Done */
#define SCIF_RC80MCR_CALIB_SHIFT          (16)      /* Bits 16-17: Calibration Value */
#define SCIF_RC80MCR_CALIB_MASK           (3 << SCIF_RC80MCR_CALIB_SHIFT)

/* High Resolution Prescaler Control Register */

#define SCIF_HRPCR_HRPEN                  (1 << 0)  /* Bit 0:  High Resolution Prescaler Enable */
#define SCIF_HRPCR_CKSEL_SHIFT            (1)       /* Bits 1-3: Clock input selection */
#define SCIF_HRPCR_CKSEL_MASK             (7 << SCIF_HRPCR_CKSEL_SHIFT)
#define SCIF_HRPCR_HRCOUNT_SHIFT          (8)       /* Bits 8-31: High Resolution Counter */
#define SCIF_HRPCR_HRCOUNT_MASK           (0xffffff << SCIF_HRPCR_HRCOUNT_SHIFT)

/* Fractional Prescaler Control Register */

#define SCIF_FPCR_FPEN                    (1 << 0)  /* Bit 0:  High Resolution Prescaler Enable */
#define SCIF_FPCR_CKSEL_SHIFT             (1)       /* Bits 1-3: Clock input selection */
#define SCIF_FPCR_CKSEL_MASK              (7 << SCIF_FPCR_CKSEL_SHIFT)

/* Fractional Prescaler Multiplier Register */

#define SCIF_FPMUL_MASK                   0xffff

/* Fractional Prescaler DIVIDER Register */

#define SCIF_FPDIV_MASK                   0xffff

/* Generic Clock Control0-11 */

#define SCIF_GCCTRL_CEN                   (1 << 0)  /* Bit 0:  Clock Enable */
#define SCIF_GCCTRL_DIVEN                 (1 << 1)  /* Bit 1:  Divide Enable */
#define SCIF_GCCTRL_OSCSEL_SHIFT          (8)       /* Bits 8-12: Oscillator Select */
#define SCIF_GCCTRL_OSCSEL_MASK           (31 << SCIF_GCCTRL_OSCSEL_SHIFT)
#  define SCIF_GCCTRL_OSCSEL_RCSYS        (0 << SCIF_GCCTRL_OSCSEL_SHIFT)  /* System RC oscillator */
#  define SCIF_GCCTRL_OSCSEL_OSC32K       (1 << SCIF_GCCTRL_OSCSEL_SHIFT)  /* Output from OSC32K */
#  define SCIF_GCCTRL_OSCSEL_DFLL0        (2 << SCIF_GCCTRL_OSCSEL_SHIFT)  /* Output from DFLL0 */
#  define SCIF_GCCTRL_OSCSEL_OSC0         (3 << SCIF_GCCTRL_OSCSEL_SHIFT)  /* Output from Oscillator0 */
#  define SCIF_GCCTRL_OSCSEL_RC80M        (4 << SCIF_GCCTRL_OSCSEL_SHIFT)  /* Output from 80MHz RCOSC */
#  define SCIF_GCCTRL_OSCSEL_RCFAST       (5 << SCIF_GCCTRL_OSCSEL_SHIFT)  /* Output from 4,8,12MHz RCFAST */
#  define SCIF_GCCTRL_OSCSEL_RC1M         (6 << SCIF_GCCTRL_OSCSEL_SHIFT)  /* Output from 1MHz RC1M */
#  define SCIF_GCCTRL_OSCSEL_CPUCLK       (7 << SCIF_GCCTRL_OSCSEL_SHIFT)  /* The CPU clock */
#  define SCIF_GCCTRL_OSCSEL_HSBCLK       (8 << SCIF_GCCTRL_OSCSEL_SHIFT)  /* High Speed Bus clock */
#  define SCIF_GCCTRL_OSCSEL_PBACLK       (9 << SCIF_GCCTRL_OSCSEL_SHIFT)  /* Peripheral Bus A clock */
#  define SCIF_GCCTRL_OSCSEL_PBBCLK       (10 << SCIF_GCCTRL_OSCSEL_SHIFT) /* Peripheral Bus B clock */
#  define SCIF_GCCTRL_OSCSEL_PBCCLK       (11 << SCIF_GCCTRL_OSCSEL_SHIFT) /* Peripheral Bus C clock */
#  define SCIF_GCCTRL_OSCSEL_PBDCLK       (12 << SCIF_GCCTRL_OSCSEL_SHIFT) /* Peripheral Bus D clock */
#  define SCIF_GCCTRL_OSCSEL_RC32K        (13 << SCIF_GCCTRL_OSCSEL_SHIFT) /* Output from 32kHz RCOSC */
#  define SCIF_GCCTRL_OSCSEL_1K           (15 << SCIF_GCCTRL_OSCSEL_SHIFT) /* 1 kHz output from OSC32K */
#  define SCIF_GCCTRL_OSCSEL_PLL0         (16 << SCIF_GCCTRL_OSCSEL_SHIFT) /* PLL0 */
#  define SCIF_GCCTRL_OSCSEL_HRPCLK       (17 << SCIF_GCCTRL_OSCSEL_SHIFT) /* High resolution prescaler */
#  define SCIF_GCCTRL_OSCSEL_FPCLK        (18 << SCIF_GCCTRL_OSCSEL_SHIFT) /* Fractional prescaler */
#  define SCIF_GCCTRL_OSCSEL_GCLKIN0      (19 << SCIF_GCCTRL_OSCSEL_SHIFT) /* GCLKIN0 */
#  define SCIF_GCCTRL_OSCSEL_GCLKIN1      (20 << SCIF_GCCTRL_OSCSEL_SHIFT) /* GCLKIN1 */
#  define SCIF_GCCTRL_OSCSEL_GCLK11       (21 << SCIF_GCCTRL_OSCSEL_SHIFT) /* GCLK11 */
#define SCIF_GCCTRL_DIV_SHIFT             (16)      /* Bits 16-31: Division Factor */
#define SCIF_GCCTRL_DIV_MASK              (0xffff << SCIF_GCCTRL_DIV_SHIFT)
#  define SCIF_GCCTRL_DIV(n)              ((n) << SCIF_GCCTRL_DIV_SHIFT)

/* 4/8/12MHz RC Oscillator Version Register */
/* Generic Clock Prescaler Version Register */
/* PLL Version Register */
/* Oscillator0 Version Register */
/* DFLL Version Register */
/* System RC Oscillator Version Register */
/* 80MHz RC Oscillator Version Register */
/* Generic Clock Version Register */
/* SCIF Version Register */

#define SCIF_VERSION_SHIFT                (0)        /* Bits 0-11: Version Number */
#define SCIF_VERSION_MASK                 (0xfff << SCIF_VERSION_VERSION_SHIFT)
#define SCIF_VARIANT_SHIFT                (16)       /* Bits 16-19: Variant Number */
#define SCIF_VARIANT_MASK                 (15 << SCIF_VARIANT_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_SCIF_H */
