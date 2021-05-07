/****************************************************************************
 * arch/arm/src/sam34/hardware/sam_pmc.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_PMC_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_PMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PMC register offsets *****************************************************/

#define SAM_PMC_SCER_OFFSET              0x0000 /* System Clock Enable Register */
#define SAM_PMC_SCDR_OFFSET              0x0004 /* System Clock Disable Register */
#define SAM_PMC_SCSR_OFFSET              0x0008 /* System Clock Status Register */
                                                /* 0x000c: Reserved */
#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_PMC_PCER0_OFFSET           0x0010 /* Peripheral Clock Enable Register 0 */
#  define SAM_PMC_PCDR0_OFFSET           0x0014 /* Peripheral Clock Disable Register 0 */
#  define SAM_PMC_PCSR0_OFFSET           0x0018 /* Peripheral Clock Status Register 0 */
#elif defined(CONFIG_ARCH_CHIP_SAM3U)
#  define SAM_PMC_PCER_OFFSET            0x0010 /* Peripheral Clock Enable Register */
#  define SAM_PMC_PCDR_OFFSET            0x0014 /* Peripheral Clock Disable Register */
#  define SAM_PMC_PCSR_OFFSET            0x0018 /* Peripheral Clock Status Register */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_PMC_CKGR_UCKR_OFFSET       0x001c /* UTMI Clock Register */
#endif
                                                /* 0x001c: Reserved (SAM4S) */
#define SAM_PMC_CKGR_MOR_OFFSET          0x0020 /* Main Oscillator Register */
#define SAM_PMC_CKGR_MCFR_OFFSET         0x0024 /* Main Clock Frequency Register */
#define SAM_PMC_CKGR_PLLAR_OFFSET        0x0028 /* PLLA Register */

#if defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S)
#  define SAM_PMC_CKGR_PLLBR_OFFSET      0x002c /* PLLB Register */
#endif
                                                /* 0x002c: Reserved (SAM3U) */
#define SAM_PMC_MCKR_OFFSET              0x0030 /* Master Clock Register */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
                                                /* 0x0034 Reserved */
#  define SAM_PMC_USB_OFFSET             0x0038 /* USB Clock Register PMC_USB */
                                                /* 0x003c Reserved */
#endif
                                                /* 0x0034-0x003c Reserved (SAM3U) */
#define SAM_PMC_PCK_OFFSET(n)            (0x0040 + ((n) << 2))
#  define SAM_PMC_PCK0_OFFSET            0x0040 /* Programmable Clock 0 Register */
#  define SAM_PMC_PCK1_OFFSET            0x0044 /* Programmable Clock 1 Register */
#  define SAM_PMC_PCK2_OFFSET            0x0048 /* Programmable Clock 2 Register */
                                                /* 0x004c-0x005c: Reserved */
#define SAM_PMC_IER_OFFSET               0x0060 /* Interrupt Enable Register */
#define SAM_PMC_IDR_OFFSET               0x0064 /* Interrupt Disable Register */
#define SAM_PMC_SR_OFFSET                0x0068 /* Status Register */
#define SAM_PMC_IMR_OFFSET               0x006c /* Interrupt Mask Register */
#define SAM_PMC_FSMR_OFFSET              0x0070 /* Fast Startup Mode Register */
#define SAM_PMC_FSPR_OFFSET              0x0074 /* Fast Startup Polarity Register */
#define SAM_PMC_FOCR_OFFSET              0x0078 /* Fault Output Clear Register */
                                                /* 0x007c-0x00e0: Reserved */
#define SAM_PMC_WPMR_OFFSET              0x00e4 /* Write Protect Mode Register */
#define SAM_PMC_WPSR_OFFSET              0x00e8 /* Write Protect Status Register */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
                                                /* 0x00ec-0x00fc Reserved */
#  define SAM_PMC_PCER1_OFFSET           0x0100 /* Peripheral Clock Enable Register 1 */
#  define SAM_PMC_PCDR1_OFFSET           0x0104 /* Peripheral Clock Disable Register 1 */
#  define SAM_PMC_PCSR1_OFFSET           0x0108 /* Peripheral Clock Status Register 1 */
#endif
                                                /* 0x010c Reserved */
#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_PMC_PCR_OFFSET             0x010c /* Peripheral Control Register */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_PMC_OCR_OFFSET             0x0110 /* Oscillator Calibration Register */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_PMC_PMMR_OFFSET            0x0130 /* PLL Maximum Multiplier Value Register */
#endif

/* PMC register addresses ***************************************************/

#define SAM_PMC_SCER                     (SAM_PMC_BASE+SAM_PMC_SCER_OFFSET)
#define SAM_PMC_SCDR                     (SAM_PMC_BASE+SAM_PMC_SCDR_OFFSET)
#define SAM_PMC_SCSR                     (SAM_PMC_BASE+SAM_PMC_SCSR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_PMC_PCER0                  (SAM_PMC_BASE+SAM_PMC_PCER0_OFFSET)
#  define SAM_PMC_PCDR0                  (SAM_PMC_BASE+SAM_PMC_PCDR0_OFFSET)
#  define SAM_PMC_PCSR0                  (SAM_PMC_BASE+SAM_PMC_PCSR0_OFFSET)
#elif defined(CONFIG_ARCH_CHIP_SAM3U)
#  define SAM_PMC_PCER                   (SAM_PMC_BASE+SAM_PMC_PCER_OFFSET)
#  define SAM_PMC_PCDR                   (SAM_PMC_BASE+SAM_PMC_PCDR_OFFSET)
#  define SAM_PMC_PCSR                   (SAM_PMC_BASE+SAM_PMC_PCSR_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_PMC_CKGR_UCKR              (SAM_PMC_BASE+SAM_PMC_CKGR_UCKR_OFFSET)
#endif

#define SAM_PMC_CKGR_MOR                 (SAM_PMC_BASE+SAM_PMC_CKGR_MOR_OFFSET)
#define SAM_PMC_CKGR_MCFR                (SAM_PMC_BASE+SAM_PMC_CKGR_MCFR_OFFSET)
#define SAM_PMC_CKGR_PLLAR               (SAM_PMC_BASE+SAM_PMC_CKGR_PLLAR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S)
#  define SAM_PMC_CKGR_PLLBR             (SAM_PMC_BASE+SAM_PMC_CKGR_PLLBR_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_PMC_USB                    (SAM_PMC_BASE+SAM_PMC_USB_OFFSET)
#endif

#define SAM_PMC_MCKR                     (SAM_PMC_BASE+SAM_PMC_MCKR_OFFSET)
#define SAM_PMC_PCK(n)                   (SAM_PMC_BASE+SAM_PMC_PCK_OFFSET(n))
#define SAM_PMC_PCK0                     (SAM_PMC_BASE+SAM_PMC_PCK0_OFFSET)
#define SAM_PMC_PCK1                     (SAM_PMC_BASE+SAM_PMC_PCK1_OFFSET)
#define SAM_PMC_PCK2                     (SAM_PMC_BASE+SAM_PMC_PCK2_OFFSET)
#define SAM_PMC_IER                      (SAM_PMC_BASE+SAM_PMC_IER_OFFSET)
#define SAM_PMC_IDR                      (SAM_PMC_BASE+SAM_PMC_IDR_OFFSET)
#define SAM_PMC_SR                       (SAM_PMC_BASE+SAM_PMC_SR_OFFSET)
#define SAM_PMC_IMR                      (SAM_PMC_BASE+SAM_PMC_IMR_OFFSET)
#define SAM_PMC_FSMR                     (SAM_PMC_BASE+SAM_PMC_FSMR_OFFSET)
#define SAM_PMC_FSPR                     (SAM_PMC_BASE+SAM_PMC_FSPR_OFFSET)
#define SAM_PMC_FOCR                     (SAM_PMC_BASE+SAM_PMC_FOCR_OFFSET)
#define SAM_PMC_WPMR                     (SAM_PMC_BASE+SAM_PMC_WPMR_OFFSET)
#define SAM_PMC_WPSR                     (SAM_PMC_BASE+SAM_PMC_WPSR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_PMC_PCER1                  (SAM_PMC_BASE+SAM_PMC_PCER1_OFFSET)
#  define SAM_PMC_PCDR1                  (SAM_PMC_BASE+SAM_PMC_PCDR1_OFFSET)
#  define SAM_PMC_PCSR1                  (SAM_PMC_BASE+SAM_PMC_PCSR1_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define SAM_PMC_PCR                    (SAM_PMC_BASE+SAM_PMC_PCR_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_PMC_OCR                    (SAM_PMC_BASE+SAM_PMC_OCR_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_PMC_PMMR                   (SAM_PMC_BASE+SAM_PMC_PMMR_OFFSET)
#endif

/* PMC register bit definitions *********************************************/

/* PMC System Clock Enable Register, PMC System Clock Disable Register,
 * and PMC System Clock Status Register common bit-field definitions
 */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM4S)
#  define PMC_UOTGCLK                    (1 << 5)  /* Bit 5: Enable USB OTG Clock (48 MHz, USB_48M) for UTMI */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define PMC_UDP                        (1 << 7)  /* Bit 7: USB Device Port Clock Enable */
#endif

#define PMC_PCK(n)                       (1 << ((n) + 8)
#define PMC_PCK0                         (1 << 8)  /* Bit 8:  Programmable Clock 0 Output Enable */
#define PMC_PCK1                         (1 << 9)  /* Bit 9:  Programmable Clock 1 Output Enable */
#define PMC_PCK2                         (1 << 10) /* Bit 10: Programmable Clock 2 Output Enable */

#if defined(CONFIG_ARCH_CHIP_SAM4CM)
#  define PMC_CPCK                       (1 << 16)
#  define PMC_CPBMCK                     (1 << 17)
#  define PMC_CPKEY                      (0xa << 20)
#endif

/* PMC Peripheral Clock Enable Register, PMC Peripheral Clock Disable
 * Register, and PMC Peripheral Clock Status Register common bit-field
 * definitions.
 */

#define PMC_PIDL(n)                      (1 << (n))
#define PMC_PID2                         (1 << 2)  /* Bit 2:  Peripheral Clock 2  Enable */
#define PMC_PID3                         (1 << 3)  /* Bit 3:  Peripheral Clock 3  Enable */
#define PMC_PID4                         (1 << 4)  /* Bit 4:  Peripheral Clock 4  Enable */
#define PMC_PID5                         (1 << 5)  /* Bit 5:  Peripheral Clock 5  Enable */
#define PMC_PID6                         (1 << 6)  /* Bit 6:  Peripheral Clock 6  Enable */
#define PMC_PID7                         (1 << 7)  /* Bit 7:  Peripheral Clock 7  Enable */
#define PMC_PID8                         (1 << 8)  /* Bit 8:  Peripheral Clock 8  Enable */
#define PMC_PID9                         (1 << 9)  /* Bit 9:  Peripheral Clock 9  Enable */
#define PMC_PID10                        (1 << 10) /* Bit 10: Peripheral Clock 10 Enable */
#define PMC_PID11                        (1 << 11) /* Bit 11: Peripheral Clock 11 Enable */
#define PMC_PID12                        (1 << 12) /* Bit 12: Peripheral Clock 12 Enable */
#define PMC_PID13                        (1 << 13) /* Bit 13: Peripheral Clock 13 Enable */
#define PMC_PID14                        (1 << 14) /* Bit 14: Peripheral Clock 14 Enable */
#define PMC_PID15                        (1 << 15) /* Bit 15: Peripheral Clock 15 Enable */
#define PMC_PID16                        (1 << 16) /* Bit 16: Peripheral Clock 16 Enable */
#define PMC_PID17                        (1 << 17) /* Bit 17: Peripheral Clock 17 Enable */
#define PMC_PID18                        (1 << 18) /* Bit 18: Peripheral Clock 18 Enable */
#define PMC_PID19                        (1 << 19) /* Bit 19: Peripheral Clock 19 Enable */
#define PMC_PID20                        (1 << 20) /* Bit 20: Peripheral Clock 20 Enable */
#define PMC_PID21                        (1 << 21) /* Bit 21: Peripheral Clock 21 Enable */
#define PMC_PID22                        (1 << 22) /* Bit 22: Peripheral Clock 22 Enable */
#define PMC_PID23                        (1 << 23) /* Bit 23: Peripheral Clock 23 Enable */
#define PMC_PID24                        (1 << 24) /* Bit 24: Peripheral Clock 24 Enable */
#define PMC_PID25                        (1 << 25) /* Bit 25: Peripheral Clock 25 Enable */
#define PMC_PID26                        (1 << 26) /* Bit 26: Peripheral Clock 26 Enable */
#define PMC_PID27                        (1 << 27) /* Bit 27: Peripheral Clock 27 Enable */
#define PMC_PID28                        (1 << 28) /* Bit 28: Peripheral Clock 28 Enable */
#define PMC_PID29                        (1 << 29) /* Bit 29: Peripheral Clock 29 Enable */
#define PMC_PID30                        (1 << 30) /* Bit 30: Peripheral Clock 30 Enable */
#define PMC_PID31                        (1 << 31) /* Bit 31: Peripheral Clock 31 Enable */

/* PMC UTMI Clock Configuration Register */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define PMC_CKGR_UCKR_UPLLEN           (1 << 16) /* Bit 16: UTMI PLL Enable */
#  define PMC_CKGR_UCKR_UPLLCOUNT_SHIFT  (20)      /* Bits 20-23: UTMI PLL Start-up Time */
#  define PMC_CKGR_UCKR_UPLLCOUNT_MASK   (15 << PMC_CKGR_UCKR_UPLLCOUNT_SHIFT)
#endif

/* PMC Clock Generator Main Oscillator Register */

#define PMC_CKGR_MOR_MOSCXTEN            (1 << 0)  /* Bit 0:  Main Crystal Oscillator Enable */
#define PMC_CKGR_MOR_MOSCXTBY            (1 << 1)  /* Bit 1:  Main Crystal Oscillator Bypass */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM4S) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
#  define PMC_CKGR_MOR_WAITMODE          (1 << 2)  /* Bit 2:  Wait Mode Command */
#endif

#define PMC_CKGR_MOR_MOSCRCEN            (1 << 3)  /* Bit 3:  Main On-Chip RC Oscillator Enable */
#define PMC_CKGR_MOR_MOSCRCF_SHIFT       (4)       /* Bits 4-6: Main On-Chip RC Oscillator Frequency Selection */
#define PMC_CKGR_MOR_MOSCRCF_MASK        (7 << PMC_CKGR_MOR_MOSCRCF_SHIFT)
#  define PMC_CKGR_MOR_MOSCRCF_4MHz      (0 << PMC_CKGR_MOR_MOSCRCF_SHIFT) /* Fast RC Osc is 4MHz (default) */
#  define PMC_CKGR_MOR_MOSCRCF_8MHz      (1 << PMC_CKGR_MOR_MOSCRCF_SHIFT) /* Fast RC Osc is 8MHz */
#  define PMC_CKGR_MOR_MOSCRCF_12MHz     (2 << PMC_CKGR_MOR_MOSCRCF_SHIFT) /* Fast RC Osc is 12MHz */

#define PMC_CKGR_MOR_MOSCXTST_SHIFT      (8)       /* Bits 8-15: Main Crystal Oscillator Start-up Time */
#define PMC_CKGR_MOR_MOSCXTST_MASK       (0xff << PMC_CKGR_MOR_MOSCXTST_SHIFT)
#  define PMC_CKGR_MOR_MOSCXTST(n)       ((uint32_t)(n) << PMC_CKGR_MOR_MOSCXTST_SHIFT)
#define PMC_CKGR_MOR_KEY_SHIFT           (16)      /* Bits 16-23: Password */
#define PMC_CKGR_MOR_KEY_MASK            (0xff << PMC_CKGR_MOR_KEY_SHIFT)
#  define PMC_CKGR_MOR_KEY               (0x37 << PMC_CKGR_MOR_KEY_SHIFT)
#define PMC_CKGR_MOR_MOSCSEL             (1 << 24) /* Bit 24: Main Oscillator Selection */
#define PMC_CKGR_MOR_CFDEN               (1 << 25) /* Bit 25: Clock Failure Detector Enable */

/* PMC Clock Generator Main Clock Frequency Register */

#define PMC_CKGR_MCFR_MAINF_SHIFT        (0)       /* Bits 0-15: Main Clock Frequency */
#define PMC_CKGR_MCFR_MAINF_MASK         (0xffff << PMC_CKGR_MCFR_MAINF_SHIFT)
#  define PMC_CKGR_MCFR_MAINF(n)         ((uint32_t)(n) << PMC_CKGR_MCFR_MAINF_SHIFT)
#define PMC_CKGR_MCFR_MAINFRDY           (1 << 16) /* Bit 16: Main Clock Ready */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define PMC_CKGR_MCFR_RCMEAS           (1 << 20) /* Bit 20: RC Oscillator Frequency Measure (write-only) */
#endif

/* PMC Clock Generator PLLA Register */

#define PMC_CKGR_PLLAR_DIV_SHIFT         (0)       /* Bits 0-7: Divider */
#define PMC_CKGR_PLLAR_DIV_MASK          (0xff << PMC_CKGR_PLLAR_DIV_SHIFT)
#  define PMC_CKGR_PLLAR_DIV_ZERO        (0 << PMC_CKGR_PLLAR_DIV_SHIFT)   /* Divider output is 0 */
#  define PMC_CKGR_PLLAR_DIV_BYPASS      (1 << PMC_CKGR_PLLAR_DIV_SHIFT)   /* Divider is bypassed (DIV=1) */
#  define PMC_CKGR_PLLAR_DIV(n)          ((n) << PMC_CKGR_PLLAR_DIV_SHIFT) /* Divider output is DIV=n, n=2..255 */

#define PMC_CKGR_PLLAR_COUNT_SHIFT       (8)       /* Bits 8-13: PLLA Counter */
#define PMC_CKGR_PLLAR_COUNT_MASK        (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)

#if defined(CONFIG_ARCH_CHIP_SAM3U)
#  define PMC_CKGR_PLLAR_STMODE_SHIFT    (14)      /* Bits 14-15: Start Mode */
#  define PMC_CKGR_PLLAR_STMODE_MASK     (3 << PMC_CKGR_PLLAR_STMODE_SHIFT)
#    define PMC_CKGR_PLLAR_STMODE_FAST   (0 << PMC_CKGR_PLLAR_STMODE_SHIFT) /* Fast Startup */
#    define PMC_CKGR_PLLAR_STMODE_NORMAL (2 << PMC_CKGR_PLLAR_STMODE_SHIFT) /* Normal Startup */
#endif

#define PMC_CKGR_PLLAR_MUL_SHIFT         (16)      /* Bits 16-26: PLLA Multiplier */
#define PMC_CKGR_PLLAR_MUL_MASK          (0x7ff << PMC_CKGR_PLLAR_MUL_SHIFT)
#define PMC_CKGR_PLLAR_ONE               (1 << 29) /* Bit 29: Always one */

/* PLLB Register */

#if defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S)
#  define PMC_CKGR_PLLBR_DIV_SHIFT       (0)       /* Bits 0-7: Divider */
#  define PMC_CKGR_PLLBR_DIV_MASK        (0xff << PMC_CKGR_PLLBR_DIV_SHIFT)
#    define PMC_CKGR_PLLBR_DIV_ZERO      (0 << PMC_CKGR_PLLBR_DIV_SHIFT)   /* Divider output is 0 */
#    define PMC_CKGR_PLLBR_DIV_BYPASS    (1 << PMC_CKGR_PLLBR_DIV_SHIFT)   /* Divider is bypassed (DIV=1) */
#    define PMC_CKGR_PLLBR_DIV(n)        ((n) << PMC_CKGR_PLLBR_DIV_SHIFT) /* Divider output is DIV=n, n=2..255 */

#  define PMC_CKGR_PLLBR_COUNT_SHIFT     (8)       /* Bits 8-13: PLLA Counter */
#  define PMC_CKGR_PLLBR_COUNT_MASK      (63 << PMC_CKGR_PLLBR_COUNT_SHIFT)
#  define PMC_CKGR_PLLBR_MUL_SHIFT       (16)      /* Bits 16-26: PLLA Multiplier */
#  define PMC_CKGR_PLLBR_MUL_MASK        (0x7ff << PMC_CKGR_PLLBR_MUL_SHIFT)

#  if defined(CONFIG_ARCH_CHIP_SAM4CM)
#    define PMC_CKGR_PLLBR_SRCB_SHIFT    (29)
#    define PMC_CKGR_PLLBR_SRCB_MASK     (1 << PMC_CKGR_PLLBR_SRCB_SHIFT)
#      define PMC_CKGR_PLLBR_SRCB_MAIN   (0 << PMC_CKGR_PLLBR_SRCB_SHIFT)
#      define PMC_CKGR_PLLBR_SRCB_PLLA   (1 << PMC_CKGR_PLLBR_SRCB_SHIFT)
#  endif
#endif

/* USB Clock Register PMC_USB */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  if defined(CONFIG_ARCH_CHIP_SAM4S)
#    define PMC_USB_USBS                 (1 << 0)  /* Bit 0:  USB Input Clock Selection */
#      define PMC_USB_USBS_PLLA          (0)
#      define PMC_USB_USBS_PLLB          PMC_USB_USBS
#  endif
#  define PMC_USB_USBDIV_SHIFT           (8)       /* Bits 8-11: Divider for USB Clock */
#  define PMC_USB_USBDIV_MASK            (15 << PMC_USB_USBDIV_SHIFT)
#endif

/* PMC Master Clock Register */

#define PMC_MCKR_CSS_SHIFT               (0)       /* Bits 0-1: Master Clock Source Selection */
#define PMC_MCKR_CSS_MASK                (3 << PMC_MCKR_CSS_SHIFT)
#  define PMC_MCKR_CSS_SLOW              (0 << PMC_MCKR_CSS_SHIFT) /* Slow Clock */
#  define PMC_MCKR_CSS_MAIN              (1 << PMC_MCKR_CSS_SHIFT) /* Main Clock */
#  define PMC_MCKR_CSS_PLLA              (2 << PMC_MCKR_CSS_SHIFT) /* PLLA Clock */

#if defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S)
#  define PMC_MCKR_CSS_PLLB              (3 << PMC_MCKR_CSS_SHIFT) /* PLLB Clock */
#elif defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
      defined(CONFIG_ARCH_CHIP_SAM3A)
#  define PMC_MCKR_CSS_UPLL            (3 << PMC_MCKR_CSS_SHIFT) /* UPLL Clock */
#endif

#define PMC_MCKR_PRES_SHIFT              (4)       /* Bits 4-6: Processor Clock Prescaler */
#define PMC_MCKR_PRES_MASK               (7 << PMC_MCKR_PRES_SHIFT)
#  define PMC_MCKR_PRES_DIV1             (0 << PMC_MCKR_PRES_SHIFT) /* Selected clock */
#  define PMC_MCKR_PRES_DIV2             (1 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 2 */
#  define PMC_MCKR_PRES_DIV4             (2 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 4 */
#  define PMC_MCKR_PRES_DIV8             (3 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 8 */
#  define PMC_MCKR_PRES_DIV16            (4 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 16 */
#  define PMC_MCKR_PRES_DIV32            (5 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 32 */
#  define PMC_MCKR_PRES_DIV64            (6 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 64 */
#  define PMC_MCKR_PRES_DIV3             (7 << PMC_MCKR_PRES_SHIFT) /* Selected clock divided by 3 */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define PMC_MCKR_PLLADIV2              (1 << 12) /* Bit 12: PLLA Divider */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S)
#  define PMC_MCKR_PLLBDIV2              (1 << 13) /* Bit 13: PLLB Divider */
#elif defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
      defined(CONFIG_ARCH_CHIP_SAM3U)
#  define PMC_MCKR_UPLLDIV2              (1 << 13) /* Bit 13: UPLL Divider */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4CM)
#  define PMC_MCKR_CPCSS_SHIFT           (16)
#  define PMC_MCKR_CPCSS_MASK            (0x7 << PMC_MCKR_CPCSS_SHIFT)
#    define PMC_MCKR_CPCSS_SLOW          (0 << PMC_MCKR_CPCSS_SHIFT) /* Slow Clock */
#    define PMC_MCKR_CPCSS_MAIN          (1 << PMC_MCKR_CPCSS_SHIFT) /* Main Clock */
#    define PMC_MCKR_CPCSS_PLLA          (2 << PMC_MCKR_CPCSS_SHIFT) /* PLLA Clock */
#    define PMC_MCKR_CPCSS_PLLB          (3 << PMC_MCKR_CPCSS_SHIFT) /* PLLB Clock */
#    define PMC_MCKR_CPCSS_MCK           (4 << PMC_MCKR_CPCSS_SHIFT) /* Master Clock */
#  define PMC_MCKR_CPPRES_SHIFT          (20)
#  define PMC_MCKR_CPPRES_MASK           (0xF << PMC_MCKR_CPPRES_SHIFT)
#    define PMC_MCKR_CPPRES(D)           (((D) - 1) << PMC_MCKR_CPPRES_SHIFT)
#endif

/* PMC Programmable Clock Register (0,1,2) */

#define PMC_PCK_CSS_SHIFT                (0)       /* Bits 0-2: Master Clock Source Selection */
#define PMC_PCK_CSS_MASK                 (7 << PMC_PCK_CSS_SHIFT)
#  define PMC_PCK_CSS_SLOW               (0 << PMC_PCK_CSS_SHIFT) /* Slow Clock */
#  define PMC_PCK_CSS_MAIN               (1 << PMC_PCK_CSS_SHIFT) /* Main Clock */
#  define PMC_PCK_CSS_PLLA               (2 << PMC_PCK_CSS_SHIFT) /* PLLA Clock */

#if defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S)
#  define PMC_PCK_CSS_PLLB               (3 << PMC_PCK_CSS_SHIFT) /* PLLB Clock */
#elif defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
      defined(CONFIG_ARCH_CHIP_SAM3U)
#  define PMC_PCK_CSS_UPLL               (3 << PMC_PCK_CSS_SHIFT) /* UPLL Clock */
#endif

#  define PMC_PCK_CSS_MCK                (4 << PMC_PCK_CSS_SHIFT) /* Master Clock */

#define PMC_PCK_PRES_SHIFT               (4)       /* Bits 4-6: Programmable Clock Prescaler */
#define PMC_PCK_PRES_MASK                (7 << PMC_PCK_PRES_SHIFT)
#  define PMC_PCK_PRES_DIV1              (0 << PMC_PCK_PRES_SHIFT) /* Selected clock */
#  define PMC_PCK_PRES_DIV2              (1 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 2 */
#  define PMC_PCK_PRES_DIV4              (2 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 4 */
#  define PMC_PCK_PRES_DIV8              (3 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 8 */
#  define PMC_PCK_PRES_DIV16             (4 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 16 */
#  define PMC_PCK_PRES_DIV32             (5 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 32 */
#  define PMC_PCK_PRES_DIV64             (6 << PMC_PCK_PRES_SHIFT) /* Selected clock divided by 64 */

/* PMC Interrupt Enable Register, PMC Interrupt Disable Register, PMC Status
 * Register, and PMC Interrupt Mask Register common bit-field definitions
 */

#define PMC_INT_MOSCXTS                  (1 << 0)  /* Bit 0:  Main Crystal Oscillator Status Interrupt */
#define PMC_INT_LOCKA                    (1 << 1)  /* Bit 1:  PLL A Lock Interrupt */

#if defined(CONFIG_ARCH_CHIP_SAM4CM) || defined(CONFIG_ARCH_CHIP_SAM4S)
#  define PMC_INT_LOCKB                  (1 << 2)  /* Bit 2:  PLL B Lock Interrupt */
#endif

#define PMC_INT_MCKRDY                   (1 << 3)  /* Bit 3:  Master Clock Ready Interrupt */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM3U)
#  define PMC_INT_LOCKU                  (1 << 6)  /* Bit 6:  UTMI PLL Lock Interrupt */
#endif

#define PMC_SR_OSCSELS                   (1 << 7)  /* Bit 7:  Slow Clock Oscillator Selection (SR only) */
#define PMC_INT_PCKRDY(n)                (1 << ((n)+8)
#  define PMC_INT_PCKRDY0                (1 << 8)  /* Bit 8:  Programmable Clock Ready 0 Interrupt */
#  define PMC_INT_PCKRDY1                (1 << 9)  /* Bit 9:  Programmable Clock Ready 1 Interrupt */
#  define PMC_INT_PCKRDY2                (1 << 10) /* Bit 10: Programmable Clock Ready 2 Interrupt */
#define PMC_INT_MOSCSELS                 (1 << 16) /* Bit 16: Main Oscillator Selection Status Interrupt */
#define PMC_INT_MOSCRCS                  (1 << 17) /* Bit 17: Main On-Chip RC Status Interrupt */
#define PMC_INT_CFDEV                    (1 << 18) /* Bit 18: Clock Failure Detector Event Interrupt */
#define PMC_SR_CFDS                      (1 << 19) /* Bit 19: Clock Failure Detector Status (SR only) */
#define PMC_SR_FOS                       (1 << 20) /* Bit 20: Clock Failure Detector Fault Output Status (SR only) */

/* PMC Fast Startup Mode Register and PMC Fast Startup Polarity Register
 * common bit-field definitions
 */

#define PMC_FSTI(n)                      (1 << (n))
#  define PMC_FSTI0                      (1 << 0)  /* Bit 0:  Fast Startup Input 0 */
#  define PMC_FSTI1                      (1 << 1)  /* Bit 1:  Fast Startup Input 1 */
#  define PMC_FSTI2                      (1 << 2)  /* Bit 2:  Fast Startup Input 2 */
#  define PMC_FSTI3                      (1 << 3)  /* Bit 3:  Fast Startup Input 3 */
#  define PMC_FSTI4                      (1 << 4)  /* Bit 4:  Fast Startup Input 4 */
#  define PMC_FSTI5                      (1 << 5)  /* Bit 5:  Fast Startup Input 5 */
#  define PMC_FSTI6                      (1 << 6)  /* Bit 6:  Fast Startup Input 6 */
#  define PMC_FSTI7                      (1 << 7)  /* Bit 7:  Fast Startup Input 7 */
#  define PMC_FSTI8                      (1 << 8)  /* Bit 8:  Fast Startup Input 8 */
#  define PMC_FSTI9                      (1 << 9)  /* Bit 9:  Fast Startup Input 9 */
#  define PMC_FSTI10                     (1 << 10) /* Bit 10: Fast Startup Input 10 */
#  define PMC_FSTI11                     (1 << 11) /* Bit 11: Fast Startup Input 11 */
#  define PMC_FSTI12                     (1 << 12) /* Bit 12: Fast Startup Input 12 */
#  define PMC_FSTI13                     (1 << 13) /* Bit 13: Fast Startup Input 13 */
#  define PMC_FSTI14                     (1 << 14) /* Bit 14: Fast Startup Input 14 */
#  define PMC_FSTI15                     (1 << 15) /* Bit 15: Fast Startup Input 15 */
#define PMC_FSMR_RTTAL                   (1 << 16) /* Bit 16: RTT Alarm Enable (MR only) */
#define PMC_FSMR_RTCAL                   (1 << 17) /* Bit 17: RTC Alarm Enable (MR only) */
#define PMC_FSMR_USBAL                   (1 << 18) /* Bit 18: USB Alarm Enable (MR only) */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A) || \
    defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define PMC_FSMR_LPM                   (1 << 20) /* Bit 20: Low Power Mode (MR only) */
#endif

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define PMC_FSMR_FLPM_SHIFT            (21)      /* Bit 21-22: Low Power Mode (MR only) */
#  define PMC_FSMR_FLPM_MASK             (3 << PMC_FSMR_FLPM_SHIFT)
#    define PMC_FSMR_FLPM_STANDBY        (0 << PMC_FSMR_FLPM_SHIFT) /* Flash Standby Mode */
#    define PMC_FSMR_FLPM_PWRDOWN        (1 << PMC_FSMR_FLPM_SHIFT) /* Flash deep power down mode */
#    define PMC_FSMR_FLPM_IDLE           (2 << PMC_FSMR_FLPM_SHIFT) /* Idle mode */
#endif

/* Fast Startup Polarity Register */

#define PMC_FSTP(n)                      (1 << (n)) /* Fast Startup Input Polarity n, n=0..15 */

#  define PMC_FSTP0                      (1 << 0)  /* Bit 0:  Fast Startup Input Polarity 0 */
#  define PMC_FSTP1                      (1 << 1)  /* Bit 1:  Fast Startup Input Polarity 1 */
#  define PMC_FSTP2                      (1 << 2)  /* Bit 2:  Fast Startup Input Polarity 2 */
#  define PMC_FSTP3                      (1 << 3)  /* Bit 3:  Fast Startup Input Polarity 3 */
#  define PMC_FSTP4                      (1 << 4)  /* Bit 4:  Fast Startup Input Polarity 4 */
#  define PMC_FSTP5                      (1 << 5)  /* Bit 5:  Fast Startup Input Polarity 5 */
#  define PMC_FSTP6                      (1 << 6)  /* Bit 6:  Fast Startup Input Polarity 6 */
#  define PMC_FSTP7                      (1 << 7)  /* Bit 7:  Fast Startup Input Polarity 7 */
#  define PMC_FSTP8                      (1 << 8)  /* Bit 8:  Fast Startup Input Polarity 8 */
#  define PMC_FSTP9                      (1 << 9)  /* Bit 9:  Fast Startup Input Polarity 9 */
#  define PMC_FSTP10                     (1 << 10) /* Bit 10: Fast Startup Input Polarity 10 */
#  define PMC_FSTP11                     (1 << 11) /* Bit 11: Fast Startup Input Polarity 11 */
#  define PMC_FSTP12                     (1 << 12) /* Bit 12: Fast Startup Input Polarity 12 */
#  define PMC_FSTP13                     (1 << 13) /* Bit 13: Fast Startup Input Polarity 13 */
#  define PMC_FSTP14                     (1 << 14) /* Bit 14: Fast Startup Input Polarity 14 */
#  define PMC_FSTP15                     (1 << 15) /* Bit 15: Fast Startup Input Polarity 15 */

/* PMC Fault Output Clear Register */

#define PMC_FOCLR                        (1 << 0)  /* Bit 0:  Fault Output Clear */

/* PMC Write Protect Mode Register */

#define PMC_WPMR_WPEN                    (1 << 0)  /* Bit 0:  Write Protect Enable */
#define PMC_WPMR_WPKEY_SHIFT             (8)       /* Bits 8-31: Write Protect KEY */
#define PMC_WPMR_WPKEY_MASK              (0x00ffffff << PMC_WPMR_WPKEY_SHIFT)
#  define PMC_WPMR_WPKEY                 (0x00504d43 << PMC_WPMR_WPKEY_SHIFT)

/* PMC Write Protect Status Register */

#define PMC_WPSR_WPVS                    (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define PMC_WPSR_WPVSRC_SHIFT            (8)       /* Bits 8-23: Write Protect Violation Source */
#define PMC_WPSR_WPVSRC_MASK             (0xffff << PMC_WPSR_WPVSRC_SHIFT)

/* Peripheral Clock Enable Register 1 */

/* Peripheral Clock Disable Register 1 */

/* Peripheral Clock Status Register 1 */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E) || \
    defined(CONFIG_ARCH_CHIP_SAM4CM)
#  define PMC_PIDH(n)                    (1 << ((n) - 32))
#  define PMC_PID32                      (1 << 0)  /* Bit 0:  PID32 */
#  define PMC_PID33                      (1 << 1)  /* Bit 1:  PID33 */
#  define PMC_PID34                      (1 << 2)  /* Bit 2:  PID34 */
#  if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM4E) || defined(CONFIG_ARCH_CHIP_SAM4CM)
#    define PMC_PID35                    (1 << 3)  /* Bit 3:  PID35 */
#    define PMC_PID36                    (1 << 4)  /* Bit 4:  PID36 */
#    define PMC_PID37                    (1 << 5)  /* Bit 5:  PID37 */
#    define PMC_PID38                    (1 << 6)  /* Bit 6:  PID38 */
#    define PMC_PID39                    (1 << 7)  /* Bit 7:  PID39 */
#    define PMC_PID40                    (1 << 8)  /* Bit 8:  PID40 */
#    define PMC_PID41                    (1 << 9)  /* Bit 9:  PID41 */
#    define PMC_PID42                    (1 << 10) /* Bit 10: PID42 */
#    define PMC_PID43                    (1 << 11) /* Bit 11: PID43 */
#    define PMC_PID44                    (1 << 12) /* Bit 12: PID44 */
#  endif
#  if defined(CONFIG_ARCH_CHIP_SAM4E)
#    define PMC_PID45                    (1 << 13) /* Bit 13: PID45 */
#    define PMC_PID46                    (1 << 14) /* Bit 14: PID46 */
#    define PMC_PID47                    (1 << 15) /* Bit 15: PID47 */
#  endif
#endif

/* Peripheral Control Register */

#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  define PMC_PCR_PID_SHIFT              (0)       /* Bits 0-5: Peripheral ID */
#  define PMC_PCR_PID_MASK               (63 < PMC_PCR_PID_SHIFT)
#  define PMC_PCR_CMD                    (1 << 12) /* Bit 12: Command */
#  define PMC_PCR_DIV_SHIFT              (16)      /* Bits 16-17: Divisor Value */
#  define PMC_PCR_DIV_MASK               (3 < PMC_PCR_DIV_SHIFT)
#    define PMC_PCR_DIV1                 (0 < PMC_PCR_DIV_SHIFT) /* Peripheral clock is MCK */
#    define PMC_PCR_DIV2                 (1 < PMC_PCR_DIV_SHIFT) /* Peripheral clock is MCK/2 */
#    define PMC_PCR_DIV4                 (2 < PMC_PCR_DIV_SHIFT) /* Peripheral clock is MCK/4 */

#  define PMC_PCR_EN                     (1 << 28) /* Bit 28: Enable */
#endif

/* Oscillator Calibration Register */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define PMC_OCR_CAL4_SHIFT            (0)       /* Bits 0-6: 4MHzRC Oscillator Calibration */
#  define PMC_OCR_CAL4_MASK             (0x7f << PMC_OCR_CAL4_SHIFT)
#    define PMC_OCR_CAL4(n)             ((uint32_t)(n) << PMC_OCR_CAL4_SHIFT)
#  define PMC_OCR_SEL4                  (1 << 7)  /* Bit 7:  Select 4MHz RC Oscillator Calibration */
#  define PMC_OCR_CAL8_SHIFT            (8)       /* Bits 8-14: 8MHzRC Oscillator Calibration */
#  define PMC_OCR_CAL8_MASK             (0x7f << PMC_OCR_CAL8_SHIFT)
#    define PMC_OCR_CAL8(n)             ((uint32_t)(n) << PMC_OCR_CAL8_SHIFT)
#  define PMC_OCR_SEL8                  (1 << 15) /* Bit 15: Select 8MHz RC Oscillator Calibration */
#  define PMC_OCR_CAL12_SHIFT           (16)      /* Bits 16-22: 12MHzRC Oscillator Calibration */
#  define PMC_OCR_CAL12_MASK            (0x7f << PMC_OCR_CAL12_SHIFT)
#    define PMC_OCR_CAL12(n)            ((uint32_t)(n) << PMC_OCR_CAL12_SHIFT)
#  define PMC_OCR_SEL12                 (1 << 23) /* Bit 23:  Select 12MHz RC Oscillator Calibration */
#endif

/* PLL Maximum Multiplier Value Register */

#if defined(CONFIG_ARCH_CHIP_SAM4E)
#  define PMC_PMMR_MASK                 (0x7ff) /* Bits 0-10: PLLA Maximum Allowed Multiplier */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_PMC_H */
