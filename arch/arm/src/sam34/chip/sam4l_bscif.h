/****************************************************************************************
 * arch/arm/src/sam34/chip/sam4l_bscif.h
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

#ifndef __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_BSCIF_H
#define __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_BSCIF_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* BSCIF register offsets ***************************************************************/

#define SAM_BSCIF_IER_OFFSET              0x0000 /* Interrupt Enable Register */
#define SAM_BSCIF_IDR_OFFSET              0x0004 /* Interrupt Disable Register */
#define SAM_BSCIF_IMR_OFFSET              0x0008 /* Interrupt Mask Register */
#define SAM_BSCIF_ISR_OFFSET              0x000c /* Interrupt Status Register */
#define SAM_BSCIF_ICR_OFFSET              0x0010 /* Interrupt Clear Register */
#define SAM_BSCIF_PCLKSR_OFFSET           0x0014 /* Power and Clocks Status Register */
#define SAM_BSCIF_UNLOCK_OFFSET           0x0018 /* Unlock Register */
#define SAM_BSCIF_CSCR_OFFSET             0x001c /* Chip Specific Configuration Register */
#define SAM_BSCIF_OSCCTRL32_OFFSET        0x0020 /* Oscillator 32 Control Register */
#define SAM_BSCIF_RC32KCR_OFFSET          0x0024 /* 32kHz RC Oscillator Control Register */
#define SAM_BSCIF_RC32KTUNE_OFFSET        0x0028 /* 32kHz RC Oscillator Tuning Register */
#define SAM_BSCIF_BOD33CTRL_OFFSET        0x002c /* BOD33 Control Register */
#define SAM_BSCIF_BOD33LEVEL_OFFSET       0x0030 /* BOD33 Level Register */
#define SAM_BSCIF_BOD33SAMPLING_OFFSET    0x0034 /* BOD33 Sampling Control Register */
#define SAM_BSCIF_BOD18CTRL_OFFSET        0x0038 /* BOD18 Control Register */
#define SAM_BSCIF_BOD18LEVEL_OFFSET       0x003c /* BOD18 Level Register */
#define SAM_BSCIF_BOD18SAMPLING_OFFSET    0x0040 /* BOD18 Sampling Control Register */
#define SAM_BSCIF_VREGCR_OFFSET           0x0044 /* Voltage Regulator Configuration Register */
#define SAM_BSCIF_RC1MCR_OFFSET           0x0058 /* 1MHz RC Clock Configuration Register */
#define SAM_BSCIF_BGCTRL_OFFSET           0x0060 /* Bandgap Control Register */
#define SAM_BSCIF_BGS_OFFSET              0x0064 /* Bandgap Status Register */
#define SAM_BSCIF_BR_OFFSET(n) (0x0078+((n)<<2)  /* 0x0078-0x0084 Backup register n=0..3 */
#define SAM_BSCIF_BR0_OFFSET              0x0078 /* Backup register 0 */
#define SAM_BSCIF_BR1_OFFSET              0x007c /* Backup register 1 */
#define SAM_BSCIF_BR2_OFFSET              0x0080 /* Backup register 2 */
#define SAM_BSCIF_BR3_OFFSET              0x0004 /* Backup register 3 */
#define SAM_BSCIF_BRIFBVERSION_OFFSET     0x03e4 /* Backup Register Interface Version Register */
#define SAM_BSCIF_BGREFIFBVERSION_OFFSET  0x03e8 /* BGREFIF Version Register */
#define SAM_BSCIF_VREGIFGVERSION_OFFSET   0x03ec /* Voltage Regulator Version Register */
#define SAM_BSCIF_BODIFCVERSION_OFFSET    0x03f0 /* BOD Version Register */
#define SAM_BSCIF_RC32KIFBVERSION_OFFSET  0x03f4 /* 32kHz RC Oscillator Version Register */
#define SAM_BSCIF_OSC32IFAVERSION_OFFSET  0x03f8 /* 32 kHz Oscillator Version Register */
#define SAM_BSCIF_VERSION_OFFSET          0x03fc /* BSCIF Version Register */

/* BSCIF register adresses **************************************************************/

#define SAM_BSCIF_IER                     (SAM_BSCIF_BASE+SAM_BSCIF_IER_OFFSET)
#define SAM_BSCIF_IDR                     (SAM_BSCIF_BASE+SAM_BSCIF_IDR_OFFSET)
#define SAM_BSCIF_IMR                     (SAM_BSCIF_BASE+SAM_BSCIF_IMR_OFFSET)
#define SAM_BSCIF_ISR                     (SAM_BSCIF_BASE+SAM_BSCIF_ISR_OFFSET)
#define SAM_BSCIF_ICR                     (SAM_BSCIF_BASE+SAM_BSCIF_ICR_OFFSET)
#define SAM_BSCIF_PCLKSR                  (SAM_BSCIF_BASE+SAM_BSCIF_PCLKSR_OFFSET)
#define SAM_BSCIF_UNLOCK                  (SAM_BSCIF_BASE+SAM_BSCIF_UNLOCK_OFFSET)
#define SAM_BSCIF_CSCR                    (SAM_BSCIF_BASE+SAM_BSCIF_CSCR_OFFSET)
#define SAM_BSCIF_OSCCTRL32               (SAM_BSCIF_BASE+SAM_BSCIF_OSCCTRL32_OFFSET)
#define SAM_BSCIF_RC32KCR                 (SAM_BSCIF_BASE+SAM_BSCIF_RC32KCR_OFFSET)
#define SAM_BSCIF_RC32KTUNE               (SAM_BSCIF_BASE+SAM_BSCIF_RC32KTUNE_OFFSET)
#define SAM_BSCIF_BOD33CTRL               (SAM_BSCIF_BASE+SAM_BSCIF_BOD33CTRL_OFFSET)
#define SAM_BSCIF_BOD33LEVEL              (SAM_BSCIF_BASE+SAM_BSCIF_BOD33LEVEL_OFFSET)
#define SAM_BSCIF_BOD33SAMPLING           (SAM_BSCIF_BASE+SAM_BSCIF_BOD33SAMPLING_OFFSET)
#define SAM_BSCIF_BOD18CTRL               (SAM_BSCIF_BASE+SAM_BSCIF_BOD18CTRL_OFFSET)
#define SAM_BSCIF_BOD18LEVEL              (SAM_BSCIF_BASE+SAM_BSCIF_BOD18LEVEL_OFFSET)
#define SAM_BSCIF_BOD18SAMPLING           (SAM_BSCIF_BASE+SAM_BSCIF_BOD18SAMPLING_OFFSET)
#define SAM_BSCIF_VREGCR                  (SAM_BSCIF_BASE+SAM_BSCIF_VREGCR_OFFSET)
#define SAM_BSCIF_RC1MCR                  (SAM_BSCIF_BASE+SAM_BSCIF_RC1MCR_OFFSET)
#define SAM_BSCIF_BGCTRL                  (SAM_BSCIF_BASE+SAM_BSCIF_BGCTRL_OFFSET)
#define SAM_BSCIF_BGS                     (SAM_BSCIF_BASE+SAM_BSCIF_BGS_OFFSET)
#define SAM_BSCIF_BR(n)                   (SAM_BSCIF_BASE+SAM_BSCIF_BR_OFFSET(n))
#define SAM_BSCIF_BR0                     (SAM_BSCIF_BASE+SAM_BSCIF_BR0_OFFSET)
#define SAM_BSCIF_BR1                     (SAM_BSCIF_BASE+SAM_BSCIF_BR1_OFFSET)
#define SAM_BSCIF_BR2                     (SAM_BSCIF_BASE+SAM_BSCIF_BR2_OFFSET)
#define SAM_BSCIF_BR3                     (SAM_BSCIF_BASE+SAM_BSCIF_BR3_OFFSET)
#define SAM_BSCIF_BRIFBVERSION            (SAM_BSCIF_BASE+SAM_BSCIF_BRIFBVERSION_OFFSET)
#define SAM_BSCIF_BGREFIFBVERSION         (SAM_BSCIF_BASE+SAM_BSCIF_BGREFIFBVERSION_OFFSET)
#define SAM_BSCIF_VREGIFGVERSION          (SAM_BSCIF_BASE+SAM_BSCIF_VREGIFGVERSION_OFFSET)
#define SAM_BSCIF_BODIFCVERSION           (SAM_BSCIF_BASE+SAM_BSCIF_BODIFCVERSION_OFFSET)
#define SAM_BSCIF_RC32KIFBVERSION         (SAM_BSCIF_BASE+SAM_BSCIF_RC32KIFBVERSION_OFFSET)
#define SAM_BSCIF_OSC32IFAVERSION         (SAM_BSCIF_BASE+SAM_BSCIF_OSC32IFAVERSION_OFFSET)
#define SAM_BSCIF_VERSION                 (SAM_BSCIF_BASE+SAM_BSCIF_VERSION_OFFSET)

/* BSCIF register bit definitions *******************************************************/

/* Interrupt Enable Register */
/* Interrupt Disable Register */
/* Interrupt Mask Register */
/* Interrupt Status Register */
/* Interrupt Clear Register */

#define BSCIF_INT_OSC32RDY                (1 << 0)  /* Bit 0 */
#define BSCIF_INT_RC32KRDY                (1 << 1)  /* Bit 1 */
#define BSCIF_INT_RC32KLOCK               (1 << 2)  /* Bit 2 */
#define BSCIF_INT_RC32KREFE               (1 << 3)  /* Bit 3 */
#define BSCIF_INT_RC32KSAT                (1 << 4)  /* Bit 4 */
#define BSCIF_INT_BOD33DET                (1 << 5)  /* Bit 5 */
#define BSCIF_INT_BOD18DET                (1 << 6)  /* Bit 6 */
#define BSCIF_INT_BOD33SYNRDY             (1 << 7)  /* Bit 7 */
#define BSCIF_INT_BOD18SYNRDY             (1 << 8)  /* Bit 8 */
#define BSCIF_INT_SSWRDY                  (1 << 9)  /* Bit 9:  Buck voltage regulator has stopped switching */
#define BSCIF_INT_VREGOK                  (1 << 10) /* Bit 10 */
#define BSCIF_INT_LPBGRDY                 (1 << 12) /* Bit 12 */
#define BSCIF_INT_AE                      (1 << 31) /* Bit 31 */

/* Power and Clocks Status Register */

#define BSCIF_PCLKSR_OSC32RDY             (1 << 0)  /* Bit 0 */
#define BSCIF_PCLKSR_RC32KRDY             (1 << 1)  /* Bit 1 */
#define BSCIF_PCLKSR_RC32KLOCK            (1 << 2)  /* Bit 2 */
#define BSCIF_PCLKSR_RC32KREFE            (1 << 3)  /* Bit 3 */
#define BSCIF_PCLKSR_RC32KSAT             (1 << 4)  /* Bit 4 */
#define BSCIF_PCLKSR_BOD33DET             (1 << 5)  /* Bit 5 */
#define BSCIF_PCLKSR_BOD18DET             (1 << 6)  /* Bit 6 */
#define BSCIF_PCLKSR_BOD33SYNRDY          (1 << 7)  /* Bit 7 */
#define BSCIF_PCLKSR_BOD18SYNRDY          (1 << 8)  /* Bit 8 */
#define BSCIF_PCLKSR_SSWRDY               (1 << 9)  /* Bit 9:  Buck voltage regulator has stopped switching */
#define BSCIF_PCLKSR_VREGOK               (1 << 10) /* Bit 10 */
#define BSCIF_PCLKSR_RC1MRDY              (1 << 11) /* Bit 11 */
#define BSCIF_PCLKSR_LPBGRDY              (1 << 12) /* Bit 12 */

/* Unlock Register */

#define BSCIF_UNLOCK_ADDR_SHIFT           (0)       /* Bits 0-9: Unlock Address */
#define BSCIF_UNLOCK_ADDR_MASK            (0x3ff << BSCIF_UNLOCK_ADDR_SHIFT)
#  define BSCIF_UNLOCK_ADDR(n)            ((n) << BSCIF_UNLOCK_ADDR_SHIFT)
#define BSCIF_UNLOCK_KEY_SHIFT            (24)      /* Bits 24-31: Unlock Key */
#define BSCIF_UNLOCK_KEY_MASK             (0xff << BSCIF_UNLOCK_KEY_SHIFT)
#  define BSCIF_UNLOCK_KEY(n)             ((n) << BSCIF_UNLOCK_KEY_SHIFT)

/* Chip Specific Configuration Register */

/* Oscillator 32 Control Register */

#define BSCIF_OSCCTRL32_OSC32EN           (1 << 0)  /* Bit 0:  32 KHz Oscillator Enable */
#define BSCIF_OSCCTRL32_EN32K             (1 << 2)  /* Bit 2:  32 KHz output Enable */
#define BSCIF_OSCCTRL32_EN1K              (1 << 3)  /* Bit 3:  1 KHz output Enable */
#define BSCIF_OSCCTRL32_MODE_SHIFT        (8)       /* Bits 8-10: Oscillator Mode */
#define BSCIF_OSCCTRL32_MODE_MASK         (7 << BSCIF_OSCCTRL32_MODE_SHIFT)
#  define BSCIF_OSCCTRL32_MODE_EXTCLK     (0 << BSCIF_OSCCTRL32_MODE_SHIFT) /* External clock */
#  define BSCIF_OSCCTRL32_MODE_XTAL       (1 << BSCIF_OSCCTRL32_MODE_SHIFT) /* Crystal mode */
#  define BSCIF_OSCCTRL32_MODE_XTALAC     (3 << BSCIF_OSCCTRL32_MODE_SHIFT) /* Crystal + amplitude controlled mode */
#  define BSCIF_OSCCTRL32_MODE_XTALHC     (4 << BSCIF_OSCCTRL32_MODE_SHIFT) /* Crystal + high current mode */
#  define BSCIF_OSCCTRL32_MODE_XTALHCAC   (5 << BSCIF_OSCCTRL32_MODE_SHIFT) /* Crystal + high current + amplitude controlled mode */
#define BSCIF_OSCCTRL32_SELCURR_SHIFT     (12)      /* Bits 12-15: Current Selection */
#define BSCIF_OSCCTRL32_SELCURR_MASK      (15 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_50      (0 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_75      (1 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_100     (2 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_125     (3 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_150     (4 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_175     (5 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_200     (6 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_225     (7 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_250     (8 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_275     (9 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_300     (10 << BSCIF_OSCCTRL32_SELCURR_SHIFT) /* (recommended value) */
#  define BSCIF_OSCCTRL32_SELCURR_325     (11 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_350     (12 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_375     (13 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_400     (14 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#  define BSCIF_OSCCTRL32_SELCURR_425     (15 << BSCIF_OSCCTRL32_SELCURR_SHIFT)
#define BSCIF_OSCCTRL32_STARTUP_SHIFT     (16)       /* Bits 16-18: Oscillator Start-up Time */
#define BSCIF_OSCCTRL32_STARTUP_MASK      (7 << BSCIF_OSCCTRL32_STARTUP_SHIFT)
#  define BSCIF_OSCCTRL32_STARTUP_0       (7 << BSCIF_OSCCTRL32_STARTUP_SHIFT)
#  define BSCIF_OSCCTRL32_STARTUP_128     (7 << BSCIF_OSCCTRL32_STARTUP_SHIFT) /* 128 1.1 ms */
#  define BSCIF_OSCCTRL32_STARTUP_8K      (7 << BSCIF_OSCCTRL32_STARTUP_SHIFT) /* 8192 72.3 ms */
#  define BSCIF_OSCCTRL32_STARTUP_16K     (7 << BSCIF_OSCCTRL32_STARTUP_SHIFT) /* 16384 143 ms */
#  define BSCIF_OSCCTRL32_STARTUP_64K     (7 << BSCIF_OSCCTRL32_STARTUP_SHIFT) /* 65536 570 ms */
#  define BSCIF_OSCCTRL32_STARTUP_128K    (7 << BSCIF_OSCCTRL32_STARTUP_SHIFT) /* 131072 1.1 s */
#  define BSCIF_OSCCTRL32_STARTUP_256K    (7 << BSCIF_OSCCTRL32_STARTUP_SHIFT) /* 262144 2.3 s */
#  define BSCIF_OSCCTRL32_STARTUP_512K    (7 << BSCIF_OSCCTRL32_STARTUP_SHIFT) /* 524288 4.6 s */
#define BSCIF_OSCCTRL32_RESERVED          (1 << 31) /* Bit 31: Reserved, must always be written as zero */

/* 32kHz RC Oscillator Control Register */

#define BSCIF_RC32KCR_EN                  (1 << 0)  /* Bit 0:  Enable as Generic clock source */
#define BSCIF_RC32KCR_TCEN                (1 << 1)  /* Bit 1:  Temperature Compensation Enable */
#define BSCIF_RC32KCR_EN32K               (1 << 2)  /* Bit 2:  Enable 32 KHz output */
#define BSCIF_RC32KCR_EN1K                (1 << 3)  /* Bit 3:  Enable 1 kHz output */
#define BSCIF_RC32KCR_MODE                (1 << 4)  /* Bit 4:  Mode Selection */
#define BSCIF_RC32KCR_REF                 (1 << 5)  /* Bit 5:  Reference select */
#define BSCIF_RC32KCR_FCD                 (1 << 7)  /* Bit 7:  Flash calibration done */

/* 32kHz RC Oscillator Tuning Register */

#define BSCIF_RC32KTUNE_FINE_SHIFT        (0)      /* Bits 0-5: Fine Value */
#define BSCIF_RC32KTUNE_FINE_MASK         (0x3f << BSCIF_RC32KTUNE_FINE_SHIFT)
#define BSCIF_RC32KTUNE_COARSE_SHIFT      (16)     /* Bits 16-22: Coarse value */
#define BSCIF_RC32KTUNE_COARSE_MASK       (0x7f << BSCIF_RC32KTUNE_COARSE_SHIFT)

/* BOD33 Control Register */
/* BOD18 Control Register */

#define BSCIF_BODCTRL_EN                  (1 << 0)  /* Bit 0:  Enable */
#define BSCIF_BODCTRL_HYST                (1 << 1)  /* Bit 1:  BOD Hysteresis */
#define BSCIF_BODCTRL_ACTION_SHIFT        (8)       /* Bits 8-9: Action */
#  define BSCIF_BODCTRL_ACTION_RESET      (1 << BSCIF_BODCTRL_ACTION_SHIFT) /* The BOD generates a reset */
#  define BSCIF_BODCTRL_ACTION_INTR       (2 << BSCIF_BODCTRL_ACTION_SHIFT) /* The BOD generates an interrupt */
#define BSCIF_BODCTRL_MODE                (1 << 0)  /* Bit 0:  Operation modes */
#define BSCIF_BODCTRL_FCD                 (1 << 0)  /* Bit 0:  BOD Fuse Calibration Done */
#define BSCIF_BODCTRL_SFV                 (1 << 0)  /* Bit 0:  BOD Control Register Store Final Value */

/* BOD33 Level Register */
/* BOD18 Level Register */

#define BSCIF_BODLEVEL_CEN                (1 << 0)  /* Bit 0:  Clock Enable */
#define BSCIF_BODLEVEL_CSSEL              (1 << 1)  /* Bit 1:  Clock Source Select */
#define BSCIF_BODLEVEL_PSEL_SHIFT         (8)       /* Bits 8-11: Prescaler Select */
#define BSCIF_BODLEVEL_PSEL_MASK          (15 << BSCIF_BODLEVEL_PSEL_SHIFT)

/* BOD33 Sampling Control Register */
/* BOD18 Sampling Control Register */

#define BSCIF_BODSAMPLING_VAL_SHIFT       (0)       /* Bits 0-5: BOD Value */
#define BSCIF_BODSAMPLING_VAL_MASK        (0x3f << BSCIF_BODSAMPLING_VAL_SHIFT)
#define BSCIF_BODSAMPLING_RANGE           (1 << 31) /* Bit 31: BOD Threshold Range (available for BOD18 only */

/* Voltage Regulator Configuration Register */

#define BSCIF_VREGCR_DIS                  (1 << 0)  /* Bit 0:  Voltage Regulator disable */
#define BSCIF_VREGCR_SSG                  (1 << 8)  /* Bit 8:  Spread Spectrum Generator Enable */
#define BSCIF_VREGCR_SSW                  (1 << 9)  /* Bit 9:  Stop Switching */
#define BSCIF_VREGCR_SSWEVT               (1 << 10) /* Bit 10: Stop Switching On Event Enable */
#define BSCIF_VREGCR_SFV                  (1 << 31) /* Bit 31: Store Final Value */

/* 1MHz RC Clock Configuration Register */

#define BSCIF_RC1MCR_FCD                  (1 << 0)  /* Bit 0:  Flash Calibration Done */
#define BSCIF_RC1MCR_CLKOEN               (1 << 7)  /* Bit 7:  1MHz RC Osc Clock Output Enable */
#define BSCIF_RC1MCR_CLKCAL_SHIFT         (8)       /* Bits 8-12: 1MHz RC Osc Calibration */
#define BSCIF_RC1MCR_CLKCAL_MASK          (31 << BSCIF_RC1MCR_CLKCAL_SHIFT)

/* Bandgap Control Register */

#define BSCIF_BGCTRL_ADCISEL_SHIFT        (0)       /* Bits 0-1: ADC Input Selection */
#define BSCIF_BGCTRL_ADCISEL_MASK         (3 << BSCIF_BGCTRL_ADCISEL_SHIFT)
#define BSCIF_BGCTRL_TSEN                 (1 << 8)

/* Bandgap Status Register */

#define BSCIF_BGS_BGBUFRDY_SHIFT          (0)       /* Bits 0-7: Bandgap Buffer Ready */
#define BSCIF_BGS_BGBUFRDY_MASK           (0xff << BSCIF_BGS_BGBUFRDY_SHIFT)
#define BSCIF_BGS_BGRDY                   (1 << 16) /* Bit 16: Bandgap Voltage Reference Ready */
#define BSCIF_BGS_LPBGRDY                 (1 << 17) /* Bit 17: Low Power Bandgap Voltage Reference Ready */
#define BSCIF_BGS_VREF_SHIFT              (18)      /* Bits 18-19: Voltage Reference Used by the System */
#define BSCIF_BGS_VREF_MASK               (3 << BSCIF_BGS_VREF_SHIFT)

/* 0x0078-0x0084 Backup register n=0..3 (32-bit data) */

/* Backup Register Interface Version Register */
/* BGREFIF Version Register */
/* Voltage Regulator Version Register */
/* BOD Version Register */
/* 32kHz RC Oscillator Version Register */
/* 32 kHz Oscillator Version Register */
/* BSCIF Version Register */

#define BSCIF_VERSION_SHIFT               (0)        /* Bits 0-11: Version Number */
#define BSCIF_VERSION_MASK                (0xfff << BSCIF_VERSION_VERSION_SHIFT)
#define BSCIF_VARIANT_SHIFT               (16)       /* Bits 16-19: Variant Number */
#define BSCIF_VARIANT_MASK                (15 << BSCIF_VARIANT_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_CHIP_SAM4L_BSCIF_H */
