/********************************************************************************************
 * arch/arm/src/sama5/chip/sam_pmc.h
 * Power Management Controller (PMC) for the SAMA5
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_CHIP_SAM_PMC_H
#define __ARCH_ARM_SRC_SAMA5_CHIP_SAM_PMC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/sam_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* PMC register offsets *********************************************************************/

#define SAM_PMC_SCER_OFFSET            0x0000 /* System Clock Enable Register */
#define SAM_PMC_SCDR_OFFSET            0x0004 /* System Clock Disable Register */
#define SAM_PMC_SCSR_OFFSET            0x0008 /* System Clock Status Register */
                                              /* 0x000c: Reserved */
#define SAM_PMC_PCER0_OFFSET           0x0010 /* Peripheral Clock Enable Register 0 */
#define SAM_PMC_PCDR0_OFFSET           0x0014 /* Peripheral Clock Disable Register 0 */
#define SAM_PMC_PCSR0_OFFSET           0x0018 /* Peripheral Clock Status Register 0 */
#define SAM_PMC_CKGR_UCKR_OFFSET       0x001c /* UTMI Clock Register */
#define SAM_PMC_CKGR_MOR_OFFSET        0x0020 /* Main Oscillator Register */
#define SAM_PMC_CKGR_MCFR_OFFSET       0x0024 /* Main Clock Frequency Register */
#define SAM_PMC_CKGR_PLLAR_OFFSET      0x0028 /* PLLA Register */
                                              /* 0x002c: Reserved */
#define SAM_PMC_MCKR_OFFSET            0x0030 /* Master Clock Register */
                                              /* 0x0034: Reserved */
#define SAM_PMC_USB_OFFSET             0x0038 /* USB Clock Register */
#define SAM_PMC_SMD_OFFSET             0x003c /* Soft Modem Clock Register */
#define SAM_PMC_PCK0_OFFSET            0x0040 /* Programmable Clock 0 Register */
#define SAM_PMC_PCK1_OFFSET            0x0044 /* Programmable Clock 1 Register */
#define SAM_PMC_PCK2_OFFSET            0x0048 /* Programmable Clock 2 Register */
                                              /* 0x004c-0x005c: Reserved */
#define SAM_PMC_IER_OFFSET             0x0060 /* Interrupt Enable Register */
#define SAM_PMC_IDR_OFFSET             0x0064 /* Interrupt Disable Register */
#define SAM_PMC_SR_OFFSET              0x0068 /* Status Register */
#define SAM_PMC_IMR_OFFSET             0x006c /* Interrupt Mask Register */
                                              /* 0x0070-0x0074: Reserved */
#define SAM_PMC_FOCR_OFFSET            0x0078 /* Fault Output Clear Register */
                                              /* 0x007c: Reserved */
#define SAM_PMC_PLLICPR_OFFSET         0x0080 /* PLL Charge Pump Current Register */
                                              /* 0x0084-0x00e0: Reserved */
#define SAM_PMC_WPMR_OFFSET            0x00e4 /* Write Protect Mode Register */
#define SAM_PMC_WPSR_OFFSET            0x00e8 /* Write Protect Status Register */
                                              /* 0x00ec-0x00fc: Reserved */
#define SAM_PMC_PCER1_OFFSET           0x0100 /* Peripheral Clock Enable Register 1 */
#define SAM_PMC_PCDR1_OFFSET           0x0104 /* Peripheral Clock Disable Register 1 */
#define SAM_PMC_PCSR1_OFFSET           0x0108 /* Peripheral Clock Status Register 1 */
#define SAM_PMC_PCR_OFFSET             0x010c /* Peripheral Control Register */

#ifdef ATSAMA5D3
#  define SAM_PMC_OCR_OFFSET           0x0110 /* Oscillator Calibration Register */
#endif

/* PMC register addresses *******************************************************************/

#define SAM_PMC_SCER                   (SAM_PMC_VBASE+SAM_PMC_SCER_OFFSET)
#define SAM_PMC_SCDR                   (SAM_PMC_VBASE+SAM_PMC_SCDR_OFFSET)
#define SAM_PMC_SCSR                   (SAM_PMC_VBASE+SAM_PMC_SCSR_OFFSET)
#define SAM_PMC_PCER0                  (SAM_PMC_VBASE+SAM_PMC_PCER0_OFFSET)
#define SAM_PMC_PCDR0                  (SAM_PMC_VBASE+SAM_PMC_PCDR0_OFFSET)
#define SAM_PMC_PCSR0                  (SAM_PMC_VBASE+SAM_PMC_PCSR0_OFFSET)
#define SAM_PMC_CKGR_UCKR              (SAM_PMC_VBASE+SAM_PMC_CKGR_UCKR_OFFSET)
#define SAM_PMC_CKGR_MOR               (SAM_PMC_VBASE+SAM_PMC_CKGR_MOR_OFFSET)
#define SAM_PMC_CKGR_MCFR              (SAM_PMC_VBASE+SAM_PMC_CKGR_MCFR_OFFSET)
#define SAM_PMC_CKGR_PLLAR             (SAM_PMC_VBASE+SAM_PMC_CKGR_PLLAR_OFFSET)
#define SAM_PMC_MCKR                   (SAM_PMC_VBASE+SAM_PMC_MCKR_OFFSET)
#define SAM_PMC_USB                    (SAM_PMC_VBASE+SAM_PMC_USB_OFFSET)
#define SAM_PMC_SMD                    (SAM_PMC_VBASE+SAM_PMC_SMD_OFFSET)
#define SAM_PMC_PCK0                   (SAM_PMC_VBASE+SAM_PMC_PCK0_OFFSET)
#define SAM_PMC_PCK1                   (SAM_PMC_VBASE+SAM_PMC_PCK1_OFFSET)
#define SAM_PMC_PCK2                   (SAM_PMC_VBASE+SAM_PMC_PCK2_OFFSET)
#define SAM_PMC_IER                    (SAM_PMC_VBASE+SAM_PMC_IER_OFFSET)
#define SAM_PMC_IDR                    (SAM_PMC_VBASE+SAM_PMC_IDR_OFFSET)
#define SAM_PMC_SR                     (SAM_PMC_VBASE+SAM_PMC_SR_OFFSET)
#define SAM_PMC_IMR                    (SAM_PMC_VBASE+SAM_PMC_IMR_OFFSET)
#define SAM_PMC_FOCR                   (SAM_PMC_VBASE+SAM_PMC_FOCR_OFFSET)
#define SAM_PMC_PLLICPR                (SAM_PMC_VBASE+SAM_PMC_PLLICPR_OFFSET)
#define SAM_PMC_WPMR                   (SAM_PMC_VBASE+SAM_PMC_WPMR_OFFSET)
#define SAM_PMC_WPSR                   (SAM_PMC_VBASE+SAM_PMC_WPSR_OFFSET)
#define SAM_PMC_PCER1                  (SAM_PMC_VBASE+SAM_PMC_PCER1_OFFSET)
#define SAM_PMC_PCDR1                  (SAM_PMC_VBASE+SAM_PMC_PCDR1_OFFSET)
#define SAM_PMC_PCSR1                  (SAM_PMC_VBASE+SAM_PMC_PCSR1_OFFSET)
#define SAM_PMC_PCR                    (SAM_PMC_VBASE+SAM_PMC_PCR_OFFSET)

#ifdef ATSAMA5D3
#  define SAM_PMC_OCR                  (SAM_PMC_VBASE+SAM_PMC_OCR_OFFSET)
#endif

/* PMC register bit definitions *************************************************************/

/* PMC System Clock Enable Register, PMC System Clock Disable Register, and PMC System
 * Clock Status Register common bit-field definitions
 */

#ifdef ATSAMA5D3
#  define PMC_PCK                      (1 << 0)  /* Bit 0:  Processor Clock */
#endif

#define PMC_DDRCK                      (1 << 2)  /* Bit 2:  DDR Clock */
#define PMC_LCDCK                      (1 << 3)  /* Bit 3:  LCD2x Clock */
#define PMC_SMDCK                      (1 << 4)  /* Bit 4:  SMD Clock */
#define PMC_UHP                        (1 << 6)  /* Bit 6:  USB Host OHCI Clocks */
#define PMC_UDP                        (1 << 7)  /* Bit 7:  USB Device Clock */

#define PMC_PCKN(n)                    (1 << ((int)(n) + 8))
#define PMC_PCK0                       (1 << 8)  /* Bit 8:  Programmable Clock 0 Output */
#define PMC_PCK1                       (1 << 9)  /* Bit 9:  Programmable Clock 1 Output */
#define PMC_PCK2                       (1 << 10) /* Bit 10: Programmable Clock 2 Output */

/* PMC Peripheral Clock Enable Register, PMC Peripheral Clock Disable Register, and PMC
 * Peripheral Clock Status Register common bit-field definitions.
 */

#define PMC_PIDL(n)                    (1 << (n))
#  define PMC_PID2                     (1 << 2)  /* Bit 2:  Peripheral Clock 2  Enable */
#  define PMC_PID3                     (1 << 3)  /* Bit 3:  Peripheral Clock 3  Enable */
#  define PMC_PID4                     (1 << 4)  /* Bit 4:  Peripheral Clock 4  Enable */
#  define PMC_PID5                     (1 << 5)  /* Bit 5:  Peripheral Clock 5  Enable */
#  define PMC_PID6                     (1 << 6)  /* Bit 6:  Peripheral Clock 6  Enable */
#  define PMC_PID7                     (1 << 7)  /* Bit 7:  Peripheral Clock 7  Enable */
#  define PMC_PID8                     (1 << 8)  /* Bit 8:  Peripheral Clock 8  Enable */
#  define PMC_PID9                     (1 << 9)  /* Bit 9:  Peripheral Clock 9  Enable */
#  define PMC_PID10                    (1 << 10) /* Bit 10: Peripheral Clock 10 Enable */
#  define PMC_PID11                    (1 << 11) /* Bit 11: Peripheral Clock 11 Enable */
#  define PMC_PID12                    (1 << 12) /* Bit 12: Peripheral Clock 12 Enable */
#  define PMC_PID13                    (1 << 13) /* Bit 13: Peripheral Clock 13 Enable */
#  define PMC_PID14                    (1 << 14) /* Bit 14: Peripheral Clock 14 Enable */
#  define PMC_PID15                    (1 << 15) /* Bit 15: Peripheral Clock 15 Enable */
#  define PMC_PID16                    (1 << 16) /* Bit 16: Peripheral Clock 16 Enable */
#  define PMC_PID17                    (1 << 17) /* Bit 17: Peripheral Clock 17 Enable */
#  define PMC_PID18                    (1 << 18) /* Bit 18: Peripheral Clock 18 Enable */
#  define PMC_PID19                    (1 << 19) /* Bit 19: Peripheral Clock 19 Enable */
#  define PMC_PID20                    (1 << 20) /* Bit 20: Peripheral Clock 20 Enable */
#  define PMC_PID21                    (1 << 21) /* Bit 21: Peripheral Clock 21 Enable */
#  define PMC_PID22                    (1 << 22) /* Bit 22: Peripheral Clock 22 Enable */
#  define PMC_PID23                    (1 << 23) /* Bit 23: Peripheral Clock 23 Enable */
#  define PMC_PID24                    (1 << 24) /* Bit 24: Peripheral Clock 24 Enable */
#  define PMC_PID25                    (1 << 25) /* Bit 25: Peripheral Clock 25 Enable */
#  define PMC_PID26                    (1 << 26) /* Bit 26: Peripheral Clock 26 Enable */
#  define PMC_PID27                    (1 << 27) /* Bit 27: Peripheral Clock 27 Enable */
#  define PMC_PID28                    (1 << 28) /* Bit 28: Peripheral Clock 28 Enable */
#  define PMC_PID29                    (1 << 29) /* Bit 29: Peripheral Clock 29 Enable */
#  define PMC_PID30                    (1 << 30) /* Bit 30: Peripheral Clock 30 Enable */
#  define PMC_PID31                    (1 << 31) /* Bit 31: Peripheral Clock 31 Enable */

/* PMC UTMI Clock Configuration Register */

#define PMC_CKGR_UCKR_UPLLEN           (1 << 16) /* Bit 16: UTMI PLL Enable */
#define PMC_CKGR_UCKR_UPLLCOUNT_SHIFT  (20)      /* Bits 20-23: UTMI PLL Start-up Time */
#define PMC_CKGR_UCKR_UPLLCOUNT_MASK   (15 << PMC_CKGR_UCKR_UPLLCOUNT_SHIFT)
#  define PMC_CKGR_UCKR_UPLLCOUNT(n)   ((n) << PMC_CKGR_UCKR_UPLLCOUNT_SHIFT)
#define PMC_CKGR_UCKR_BIASEN           (1 << 24) /* Bit 24: UTMI BIAS Enable */
#define PMC_CKGR_UCKR_BIASCOUNT_SHIFT  (28)      /* Bits 28-31: UTMI BIAS Start-up Time */
#define PMC_CKGR_UCKR_BIASCOUNT_MASK   (15 << PMC_CKGR_UCKR_BIASCOUNT_SHIFT)
#  define PMC_CKGR_UCKR_BIASCOUNT(n)   ((n) << PMC_CKGR_UCKR_BIASCOUNT_SHIFT)

/* PMC Clock Generator Main Oscillator Register */

#define PMC_CKGR_MOR_MOSCXTEN          (1 << 0)  /* Bit 0:  Main Crystal Oscillator Enable */
#define PMC_CKGR_MOR_MOSCXTBY          (1 << 1)  /* Bit 1:  Main Crystal Oscillator Bypass */

#ifdef ATSAMA5D3
#  define PMC_CKGR_MOR_MOSCRCEN        (1 << 3)  /* Bit 3:  Main On-Chip RC Oscillator Enable */
#endif

#define PMC_CKGR_MOR_MOSCXTST_SHIFT    (8)       /* Bits 8-15: Main Crystal Oscillator Start-up Time */
#define PMC_CKGR_MOR_MOSCXTST_MASK     (0xff << PMC_CKGR_MOR_MOSCXTST_SHIFT)
#define PMC_CKGR_MOR_KEY_SHIFT         (16)      /* Bits 16-23: Password */
#define PMC_CKGR_MOR_KEY_MASK          (0xff << PMC_CKGR_MOR_KEY_SHIFT)
#  define PMC_CKGR_MOR_KEY             (0x37 << PMC_CKGR_MOR_KEY_SHIFT)
#define PMC_CKGR_MOR_MOSCSEL           (1 << 24) /* Bit 24: Main Oscillator Selection */
#define PMC_CKGR_MOR_CFDEN             (1 << 25) /* Bit 25: Clock Failure Detector Enable */

#ifdef ATSAMA5D4
#  define PMC_CKGR_MOR_XT32KFME        (1 << 26) /* Bit 26: Slow Crystal Oscillator Frequency Monitoring Enable */
#endif

/* PMC Clock Generator Main Clock Frequency Register */

#define PMC_CKGR_MCFR_MAINF_SHIFT      (0)       /* Bits 0-15: Main Clock Frequency */
#define PMC_CKGR_MCFR_MAINF_MASK       (0xffff << PMC_CKGR_MCFR_MAINF_SHIFT)
#  define PMC_CKGR_MCFR_MAINF(n)       ((uint32_t)(n) << PMC_CKGR_MCFR_MAINF_SHIFT)
#define PMC_CKGR_MCFR_MAINFRDY         (1 << 16) /* Bit 16: Main Clock Ready */
#define PMC_CKGR_MCFR_RCMEAS           (1 << 20) /* Bit 20: RC Oscillator Frequency Measure (write-only) */

/* PMC Clock Generator PLLA Register */

#undef SAMA5_HAVE_PLLAR_DIV
#if defined(ATSAMA5D3)
#  define PMC_CKGR_PLLAR_DIV_SHIFT     (0)       /* Bits 0-7: Divider */
#  define PMC_CKGR_PLLAR_DIV_MASK      (0xff << PMC_CKGR_PLLAR_DIV_SHIFT)
#    define PMC_CKGR_PLLAR_DIV_ZERO    (0 << PMC_CKGR_PLLAR_DIV_SHIFT)   /* Divider output is 0 */
#    define PMC_CKGR_PLLAR_DIV_BYPASS  (1 << PMC_CKGR_PLLAR_DIV_SHIFT)   /* Divider is bypassed (DIV=1) */
#    define PMC_CKGR_PLLAR_DIV(n)      ((n) << PMC_CKGR_PLLAR_DIV_SHIFT) /* Divider output is DIV=n, n=2..255 */
#  define SAMA5_HAVE_PLLAR_DIV         1

/* According the preliminary documentation, there is no DIV field in the SAMA5D4
 * PLLAR register.  However, through trial and error, I find that the PLL output
 * is still disabled if the DIV field is set to zero.
 */

#elif defined(ATSAMA5D4)
#  define PMC_CKGR_PLLAR_DIV_BYPASS    (1)       /* Divider is bypassed (DIV=1) */
#endif

#define PMC_CKGR_PLLAR_COUNT_SHIFT     (8)       /* Bits 8-13: PLLA Counter */
#define PMC_CKGR_PLLAR_COUNT_MASK      (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#  define PMC_CKGR_PLLAR_COUNT(n)      ((uint32_t)(n) << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define PMC_CKGR_PLLAR_OUT_SHIFT       (14)      /* Bits 14-17: PLLA Clock Frequency Range */
#define PMC_CKGR_PLLAR_OUT_MASK        (15 << PMC_CKGR_PLLAR_OUT_SHIFT)
#  define PMC_CKGR_PLLAR_OUT           (0 << PMC_CKGR_PLLAR_OUT_SHIFT) /* To be programmed to 0 */
#define PMC_CKGR_PLLAR_MUL_SHIFT       (18)      /* Bits 18-24: PLLA Multiplier */
#define PMC_CKGR_PLLAR_MUL_MASK        (0x7f << PMC_CKGR_PLLAR_MUL_SHIFT)
#  define PMC_CKGR_PLLAR_MUL(n)        ((uint32_t)(n) << PMC_CKGR_PLLAR_MUL_SHIFT)
#define PMC_CKGR_PLLAR_ONE             (1 << 29) /* Bit 29: Always one */

/* PMC Master Clock Register */

#define PMC_MCKR_CSS_SHIFT             (0)       /* Bits 0-1: Master Clock Source Selection */
#define PMC_MCKR_CSS_MASK              (3 << PMC_MCKR_CSS_SHIFT)
#  define PMC_MCKR_CSS_SLOW            (0 << PMC_MCKR_CSS_SHIFT) /* Slow Clock */
#  define PMC_MCKR_CSS_MAIN            (1 << PMC_MCKR_CSS_SHIFT) /* Main Clock */
#  define PMC_MCKR_CSS_PLLA            (2 << PMC_MCKR_CSS_SHIFT) /* PLLA Clock */
#  define PMC_MCKR_CSS_UPLL            (3 << PMC_MCKR_CSS_SHIFT) /* UPLL Clock */
#define PMC_MCKR_PRES_SHIFT            (4)       /* Bits 4-6: Processor Clock Prescaler */
#define PMC_MCKR_PRES_MASK             (7 << PMC_MCKR_PRES_SHIFT)
#  define PMC_MCKR_PRES_DIV1           (0 << PMC_MCKR_PRES_SHIFT) /* Selected clock */
#  define PMC_MCKR_PRES_DIV2           (1 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 2 */
#  define PMC_MCKR_PRES_DIV4           (2 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 4 */
#  define PMC_MCKR_PRES_DIV8           (3 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 8 */
#  define PMC_MCKR_PRES_DIV16          (4 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 16 */
#  define PMC_MCKR_PRES_DIV32          (5 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 32 */
#  define PMC_MCKR_PRES_DIV64          (6 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 64 */
#define PMC_MCKR_MDIV_SHIFT            (8)        /* Bits 8-9: Master Clock Division */
#define PMC_MCKR_MDIV_MASK             (3 << PMC_MCKR_MDIV_SHIFT)
#  define PMC_MCKR_MDIV_PCKDIV1        (0 << PMC_MCKR_MDIV_SHIFT) /* Prescaler Output Clock divided by 1 */
#  define PMC_MCKR_MDIV_PCKDIV2        (1 << PMC_MCKR_MDIV_SHIFT) /* Prescaler Output Clock divided by 2 */
#  define PMC_MCKR_MDIV_PCKDIV4        (2 << PMC_MCKR_MDIV_SHIFT) /* Prescaler Output Clock divided by 4 */
#  define PMC_MCKR_MDIV_PCKDIV3        (3 << PMC_MCKR_MDIV_SHIFT) /* Prescaler Output Clock divided by 3 */
#define PMC_MCKR_PLLADIV2              (1 << 12) /* Bit 12: PLLA Divider */

#ifdef ATSAMA5D4
#  define PMC_MCKR_H32MXDIV            (1 << 24) /* Bit 24: AHB 32-bit Matrix Divisor */
#endif

/* USB Clock Register PMC_USB */

#define PMC_USB_USBS                   (1 << 0)  /* Bit 0:  USB Input Clock Selection */
#  define PMC_USB_USBS_PLLA            (0)
#  define PMC_USB_USBS_UPLL            PMC_USB_USBS
#define PMC_USB_USBDIV_SHIFT           (8)       /* Bits 8-11: Divider for USB Clock */
#define PMC_USB_USBDIV_MASK            (15 << PMC_USB_USBDIV_SHIFT)
#  define PMC_USB_USBDIV(a)            ((a) << PMC_USB_USBDIV_SHIFT)

/* Soft Modem Clock Register */

#define PMC_SMD_SMDS                   (1 << 0)  /* Bit 0:  SMD Input Clock Selection */
#  define PMC_SMD_SMDS_PLLA            (0)
#  define PMC_SMD_SMDS_UPLL            PMC_SMD_SMDS
#define PMC_SMD_SMDDIV_SHIFT           (8)       /* Bits 8-12: Divider for SMD Clock */
#define PMC_SMD_SMDDIV_MASK            (31 << PMC_SMD_SMDDIV_SHIFT)

/* PMC Programmable Clock Register (0,1,2) */

#define PMC_PCK_CSS_SHIFT              (0)       /* Bits 0-2: Master Clock Source Selection */
#define PMC_PCK_CSS_MASK               (7 << PMC_PCK_CSS_SHIFT)
#  define PMC_PCK_CSS_SLOW             (0 << PMC_PCK_CSS_SHIFT) /* Slow Clock */
#  define PMC_PCK_CSS_MAIN             (1 << PMC_PCK_CSS_SHIFT) /* Main Clock */
#  define PMC_PCK_CSS_PLLA             (2 << PMC_PCK_CSS_SHIFT) /* PLLA Clock */
#  define PMC_PCK_CSS_UPLL             (3 << PMC_PCK_CSS_SHIFT) /* UPLL Clock */
#  define PMC_PCK_CSS_MCK              (4 << PMC_PCK_CSS_SHIFT) /* Master Clock */
#define PMC_PCK_PRES_SHIFT             (4)       /* Bits 4-6: Programmable Clock Prescaler */
#define PMC_PCK_PRES_MASK              (7 << PMC_PCK_PRES_SHIFT)
#  define PMC_PCK_PRES_DIV1            (0 << PMC_PCK_PRES_SHIFT) /* Selected clock */
#  define PMC_PCK_PRES_DIV2            (1 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 2 */
#  define PMC_PCK_PRES_DIV4            (2 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 4 */
#  define PMC_PCK_PRES_DIV8            (3 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 8 */
#  define PMC_PCK_PRES_DIV16           (4 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 16 */
#  define PMC_PCK_PRES_DIV32           (5 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 32 */
#  define PMC_PCK_PRES_DIV64           (6 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 64 */

/* PMC Interrupt Enable Register, PMC Interrupt Disable Register, PMC Status Register,
 * and PMC Interrupt Mask Register common bit-field definitions
 */

#define PMC_INT_MOSCXTS                (1 << 0)  /* Bit 0:  Main Crystal Oscillator Status Interrupt */
#define PMC_INT_LOCKA                  (1 << 1)  /* Bit 1:  PLL A Lock Interrupt */
#define PMC_INT_MCKRDY                 (1 << 3)  /* Bit 3:  Master Clock Ready Interrupt */
#define PMC_INT_LOCKU                  (1 << 6)  /* Bit 6:  UTMI PLL Lock Interrupt */
#define PMC_INT_PCKRDY0                (1 << 8)  /* Bit 8:  Programmable Clock Ready 0 Interrupt */
#define PMC_INT_PCKRDY1                (1 << 9)  /* Bit 9:  Programmable Clock Ready 1 Interrupt */
#define PMC_INT_PCKRDY2                (1 << 10) /* Bit 10: Programmable Clock Ready 2 Interrupt */
#define PMC_INT_MOSCSELS               (1 << 16) /* Bit 16: Main Oscillator Selection Status Interrupt */
#define PMC_INT_MOSCRCS                (1 << 17) /* Bit 17: Main On-Chip RC Status Interrupt */
#define PMC_INT_CFDEV                  (1 << 18) /* Bit 18: Clock Failure Detector Event Interrupt */
#define PMC_SR_CFDS                    (1 << 19) /* Bit 19: Clock Failure Detector Status (SR only) */
#define PMC_SR_FOS                     (1 << 20) /* Bit 20: Clock Failure Detector Fault Output Status (SR only) */

#ifdef ATSAMA5D4
#  define PMC_SR_XT32KERR              (1 << 21) /* Bit 21: Slow Crystal Oscillator Error Interrupt */
#endif

/* PMC Fault Output Clear Register */

#define PMC_FOCLR                      (1 << 0)  /* Bit 0:  Fault Output Clear */

/* PLL Charge Pump Current Register */

#define PMC_PLLICPR_ICP_PLLA_SHIFT     (0)       /* Bits 0-1: Charge Pump Current PLLA */
#define PMC_PLLICPR_ICP_PLLA_MASK      (3 << PMC_PLLICPR_ICP_PLLA_SHIFT)
#  define PMC_PLLICPR_ICP_PLLA(n)      ((uint32_t)(n) << PMC_PLLICPR_ICP_PLLA_SHIFT)
#define PMC_PLLICPR_IPLL_PLLA_SHIFT    (8)       /* Bits 8-10: Engineering Configuration PLLA */
#define PMC_PLLICPR_IPLL_PLLA_MASK     (7 << PMC_PLLICPR_IPLL_PLLA_SHIFT)
#  define PMC_PLLICPR_IPLL_PLLA(n)     ((uint32_t)(n) << PMC_PLLICPR_IPLL_PLLA_SHIFT)
#define PMC_PLLICPR_ICP_PLLU_SHIFT     (16)      /* Bits 16-17: Charge Pump Current PLL UTMI */
#define PMC_PLLICPR_ICP_PLLU_MASK      (3 << PMC_PLLICPR_ICP_PLLU_SHIFT)
#  define PMC_PLLICPR_ICP_PLLU(n)      ((uint32_t)(n) << PMC_PLLICPR_ICP_PLLU_SHIFT)
#define PMC_PLLICPR_IVCO_PLLU_SHIFT    (14)      /* Bits 24-15: Voltage Control Output Current PLL UTMI */
#define PMC_PLLICPR_IVCO_PLLU_MASK     (3 << PMC_PLLICPR_IVCO_PLLU_SHIFT)
#  define PMC_PLLICPR_IVCO_PLLU(n)     ((uint32_t)(n) << PMC_PLLICPR_IVCO_PLLU_SHIFT)

/* PMC Write Protect Mode Register */

#define PMC_WPMR_WPEN                  (1 << 0)  /* Bit 0:  Write Protect Enable */
#define PMC_WPMR_WPKEY_SHIFT           (8)       /* Bits 8-31: Write Protect KEY */
#define PMC_WPMR_WPKEY_MASK            (0x00ffffff << PMC_WPMR_WPKEY_SHIFT)
#  define PMC_WPMR_WPKEY               (0x00504d43 << PMC_WPMR_WPKEY_SHIFT)

/* PMC Write Protect Status Register */

#define PMC_WPSR_WPVS                  (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define PMC_WPSR_WPVSRC_SHIFT          (8)       /* Bits 8-23: Write Protect Violation Source */
#define PMC_WPSR_WPVSRC_MASK           (0xffff << PMC_WPSR_WPVSRC_SHIFT)

/* Peripheral Clock Enable Register 1 */
/* Peripheral Clock Disable Register 1 */
/* Peripheral Clock Status Register 1 */

#define PMC_PIDH(n)                    (1 << ((n) - 32))
#  define PMC_PID32                    (1 << 0)  /* Bit 0:  PID32 */
#  define PMC_PID33                    (1 << 1)  /* Bit 1:  PID33 */
#  define PMC_PID34                    (1 << 2)  /* Bit 2:  PID34 */
#  define PMC_PID35                    (1 << 3)  /* Bit 3:  PID35 */
#  define PMC_PID36                    (1 << 4)  /* Bit 4:  PID36 */
#  define PMC_PID37                    (1 << 5)  /* Bit 5:  PID37 */
#  define PMC_PID38                    (1 << 6)  /* Bit 6:  PID38 */
#  define PMC_PID39                    (1 << 7)  /* Bit 7:  PID39 */
#  define PMC_PID40                    (1 << 8)  /* Bit 8:  PID40 */
#  define PMC_PID41                    (1 << 9)  /* Bit 9:  PID41 */
#  define PMC_PID42                    (1 << 10) /* Bit 10: PID42 */
#  define PMC_PID43                    (1 << 11) /* Bit 11: PID43 */
#  define PMC_PID44                    (1 << 12) /* Bit 12: PID44 */
#  define PMC_PID45                    (1 << 13) /* Bit 13: PID45 */
#  define PMC_PID46                    (1 << 14) /* Bit 14: PID46 */
#  define PMC_PID47                    (1 << 15) /* Bit 15: PID47 */
#  define PMC_PID48                    (1 << 16) /* Bit 16: PID48 */
#  define PMC_PID49                    (1 << 17) /* Bit 17: PID49 */
#  define PMC_PID50                    (1 << 18) /* Bit 18: PID50 */
#  define PMC_PID51                    (1 << 19) /* Bit 19: PID51 */
#  define PMC_PID52                    (1 << 20) /* Bit 20: PID52 */
#  define PMC_PID53                    (1 << 21) /* Bit 21: PID53 */
#  define PMC_PID54                    (1 << 22) /* Bit 22: PID54 */
#  define PMC_PID55                    (1 << 23) /* Bit 23: PID55 */
#  define PMC_PID56                    (1 << 24) /* Bit 24: PID56 */
#  define PMC_PID57                    (1 << 25) /* Bit 25: PID57 */
#  define PMC_PID58                    (1 << 26) /* Bit 26: PID58 */
#  define PMC_PID59                    (1 << 27) /* Bit 27: PID59 */
#  define PMC_PID60                    (1 << 28) /* Bit 28: PID50 */
#  define PMC_PID61                    (1 << 29) /* Bit 29: PID61 */
#  define PMC_PID62                    (1 << 30) /* Bit 30: PID62 */
#  define PMC_PID63                    (1 << 31) /* Bit 31: PID63 */

/* Peripheral Control Register */

#define PMC_PCR_PID_SHIFT              (0)       /* Bits 0-5: Peripheral ID */
#define PMC_PCR_PID_MASK               (63 << PMC_PCR_PID_SHIFT)
#  define PMC_PCR_PID(n)               ((n) << PMC_PCR_PID_SHIFT)
#define PMC_PCR_CMD                    (1 << 12) /* Bit 12: Command */

#ifdef ATSAMA5D3
#  define SAMA5_HAVE_PMC_PCR_DIV       1         /* Supports conditional compilation */
#  define PMC_PCR_DIV_SHIFT            (16)      /* Bits 16-17: Divisor Value */
#  define PMC_PCR_DIV_MASK             (3 << PMC_PCR_DIV_SHIFT)
#    define PMC_PCR_DIV(n)             ((uint32_t)(n) << PMC_PCR_DIV_SHIFT)
#    define PMC_PCR_DIV1               (0 << PMC_PCR_DIV_SHIFT) /* Peripheral clock is MCK */
#    define PMC_PCR_DIV2               (1 << PMC_PCR_DIV_SHIFT) /* Peripheral clock is MCK/2 */
#    define PMC_PCR_DIV4               (2 << PMC_PCR_DIV_SHIFT) /* Peripheral clock is MCK/4 */
#    define PMC_PCR_DIV8               (3 << PMC_PCR_DIV_SHIFT) /* Peripheral clock is MCK/8 */
#endif

#define PMC_PCR_EN                     (1 << 28) /* Bit 28: Enable */

#ifdef ATSAMA5D3
/* Oscillator Calibration Register */

#  define PMC_OCR_CAL_SHIFT            (0)       /* Bits 0-6: 12 MHz RC Oscillator Calibration bits */
#  define PMC_OCR_CAL_MASK             (0x7f << PMC_OCR_CAL_SHIFT)
#    define PMC_OCR_CAL(n)             ((uint32_t)(n) << PMC_OCR_CAL_SHIFT)
#  define PMC_OCR_SEL                  (1 << 7)  /* Bit 7:  Selection of RC Oscillator Calibration bits */
#endif

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMA5_CHIP_SAM_PMC_H */
