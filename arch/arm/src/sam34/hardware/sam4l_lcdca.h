/****************************************************************************
 * arch/arm/src/sam34/hardware/sam4l_lcdca.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_LCDCA_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_LCDCA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LCDCA register offsets ***************************************************/

#define SAM_LCDCA_CR_OFFSET        0x0000 /* Control Register */
#define SAM_LCDCA_CFG_OFFSET       0x0004 /* Configuration Register */
#define SAM_LCDCA_TIM_OFFSET       0x0008 /* Timing Register */
#define SAM_LCDCA_SR_OFFSET        0x000c /* Status Register */
#define SAM_LCDCA_SCR_OFFSET       0x0010 /* Status Clear Register */

#define SAM_LCDCA_DRL_OFFSET(n)    (0x0014+((n)<<3))
#define SAM_LCDCA_DRH_OFFSET(n)    (0x0018+((n)<<3))
#define SAM_LCDCA_DRL0_OFFSET      0x0014 /* Data Register Low 0 */
#define SAM_LCDCA_DRH0_OFFSET      0x0018 /* Data Register High 0 */
#define SAM_LCDCA_DRL1_OFFSET      0x001c /* Data Register Low 1 */
#define SAM_LCDCA_DRH1_OFFSET      0x0020 /* Data Register High 1 */
#define SAM_LCDCA_DRL2_OFFSET      0x0024 /* Data Register Low 2 */
#define SAM_LCDCA_DRH2_OFFSET      0x0028 /* Data Register High 2 */
#define SAM_LCDCA_DRL3_OFFSET      0x002c /* Data Register Low 3 */
#define SAM_LCDCA_DRH3_OFFSET      0x0030 /* Data Register High 3 */

#define SAM_LCDCA_IADR_OFFSET      0x0034 /* Indirect Access Data Register */
#define SAM_LCDCA_BCFG_OFFSET      0x0038 /* Blink Configuration Register */
#define SAM_LCDCA_CSRCFG_OFFSET    0x003c /* Circular Shift Register Configuration */
#define SAM_LCDCA_CMCFG_OFFSET     0x0040 /* Character Mapping Configuration Register */
#define SAM_LCDCA_CMDR_OFFSET      0x0044 /* Character Mapping Data Register */
#define SAM_LCDCA_ACMCFG_OFFSET    0x0048 /* Automated Character Mapping Configuration Register */
#define SAM_LCDCA_ACMDR_OFFSET     0x004c /* Automated Character Mapping Data Register */
#define SAM_LCDCA_ABMCFG_OFFSET    0x0050 /* Automated Bit Mapping Configuration Register */
#define SAM_LCDCA_ABMDR_OFFSET     0x0054 /* Automated Bit Mapping Data Register */
#define SAM_LCDCA_IER_OFFSET       0x0058 /* Interrupt Enable Register */
#define SAM_LCDCA_IDR_OFFSET       0x005c /* Interrupt Disable Register */
#define SAM_LCDCA_IMR_OFFSET       0x0060 /* Interrupt Mask Register */
#define SAM_LCDCA_VERSION_OFFSET   0x0064 /* Version Register */

/* LCDCA register addresses *************************************************/

#define SAM_LCDCA_CR               (SAM_LCDCA_BASE+SAM_LCDCA_CR_OFFSET)
#define SAM_LCDCA_CFG              (SAM_LCDCA_BASE+SAM_LCDCA_CFG_OFFSET)
#define SAM_LCDCA_TIM              (SAM_LCDCA_BASE+SAM_LCDCA_TIM_OFFSET)
#define SAM_LCDCA_SR               (SAM_LCDCA_BASE+SAM_LCDCA_SR_OFFSET)
#define SAM_LCDCA_SCR              (SAM_LCDCA_BASE+SAM_LCDCA_SCR_OFFSET)

#define SAM_LCDCA_DRL(n)           (SAM_LCDCA_BASE+SAM_LCDCA_DRL_OFFSET(n))
#define SAM_LCDCA_DRH(n)           (SAM_LCDCA_BASE+SAM_LCDCA_DRH_OFFSET(n))
#define SAM_LCDCA_DRL0             (SAM_LCDCA_BASE+SAM_LCDCA_DRL0_OFFSET)
#define SAM_LCDCA_DRH0             (SAM_LCDCA_BASE+SAM_LCDCA_DRH0_OFFSET)
#define SAM_LCDCA_DRL1             (SAM_LCDCA_BASE+SAM_LCDCA_DRL1_OFFSET)
#define SAM_LCDCA_DRH1             (SAM_LCDCA_BASE+SAM_LCDCA_DRH1_OFFSET)
#define SAM_LCDCA_DRL2             (SAM_LCDCA_BASE+SAM_LCDCA_DRL2_OFFSET)
#define SAM_LCDCA_DRH2             (SAM_LCDCA_BASE+SAM_LCDCA_DRH2_OFFSET)
#define SAM_LCDCA_DRL3             (SAM_LCDCA_BASE+SAM_LCDCA_DRL3_OFFSET)
#define SAM_LCDCA_DRH3             (SAM_LCDCA_BASE+SAM_LCDCA_DRH3_OFFSET)

#define SAM_LCDCA_IADR             (SAM_LCDCA_BASE+SAM_LCDCA_IADR_OFFSET)
#define SAM_LCDCA_BCFG             (SAM_LCDCA_BASE+SAM_LCDCA_BCFG_OFFSET)
#define SAM_LCDCA_CSRCFG           (SAM_LCDCA_BASE+SAM_LCDCA_CSRCFG_OFFSET)
#define SAM_LCDCA_CMCFG            (SAM_LCDCA_BASE+SAM_LCDCA_CMCFG_OFFSET)
#define SAM_LCDCA_CMDR             (SAM_LCDCA_BASE+SAM_LCDCA_CMDR_OFFSET)
#define SAM_LCDCA_ACMCFG           (SAM_LCDCA_BASE+SAM_LCDCA_ACMCFG_OFFSET)
#define SAM_LCDCA_ACMDR            (SAM_LCDCA_BASE+SAM_LCDCA_ACMDR_OFFSET)
#define SAM_LCDCA_ABMCFG           (SAM_LCDCA_BASE+SAM_LCDCA_ABMCFG_OFFSET)
#define SAM_LCDCA_ABMDR            (SAM_LCDCA_BASE+SAM_LCDCA_ABMDR_OFFSET)
#define SAM_LCDCA_IER              (SAM_LCDCA_BASE+SAM_LCDCA_IER_OFFSET)
#define SAM_LCDCA_IDR              (SAM_LCDCA_BASE+SAM_LCDCA_IDR_OFFSET)
#define SAM_LCDCA_IMR              (SAM_LCDCA_BASE+SAM_LCDCA_IMR_OFFSET)
#define SAM_LCDCA_VERSION          (SAM_LCDCA_BASE+SAM_LCDCA_VERSION_OFFSET)

/* LCDCA register bit definitions *******************************************/

/* Control Register */

#define LCDCA_CR_DIS               (1 << 0)  /* Bit 0:  Disable */
#define LCDCA_CR_EN                (1 << 1)  /* Bit 1:  Enable */
#define LCDCA_CR_FC0DIS            (1 << 2)  /* Bit 2:  Frame Counter 0 Disable */
#define LCDCA_CR_FC0EN             (1 << 3)  /* Bit 3:  Frame Counter 0 Enable */
#define LCDCA_CR_FC1DIS            (1 << 4)  /* Bit 4:  Frame Counter 1 Disable */
#define LCDCA_CR_FC1EN             (1 << 5)  /* Bit 5:  Frame Counter 1 Enable */
#define LCDCA_CR_FC2DIS            (1 << 6)  /* Bit 6:  Frame Counter 2 Disable */
#define LCDCA_CR_FC2EN             (1 << 7)  /* Bit 7:  Frame Counter 2 Enable */
#define LCDCA_CR_CDM               (1 << 8)  /* Bit 8:  Clear Display Memory */
#define LCDCA_CR_WDIS              (1 << 9)  /* Bit 9:  Wake up Disable */
#define LCDCA_CR_WEN               (1 << 10) /* Bit 10: Wake up Enable */
#define LCDCA_CR_BSTART            (1 << 11) /* Bit 11: Blinking Start */
#define LCDCA_CR_BSTOP             (1 << 12) /* Bit 12: Blinking Stop */
#define LCDCA_CR_CSTART            (1 << 13) /* Bit 13: Circular Shift Start */
#define LCDCA_CR_CSTOP             (1 << 14) /* Bit 13: Circular Shift Stop */

/* Configuration Register */

#define LCDCA_CFG_XBIAS            (1 << 0)  /* Bit 0:  External Bias Generation */
#define LCDCA_CFG_WMOD             (1 << 1)  /* Bit 1:  Waveform Mode */
#define LCDCA_CFG_BLANK            (1 << 2)  /* Bit 2:  Blank LCD */
#define LCDCA_CFG_LOCK             (1 << 3)  /* Bit 3:  Lock */
#define LCDCA_CFG_DUTY_SHIFT       (8)       /* Bits 8-9: Duty Select */
#define LCDCA_CFG_DUTY_MASK        (3 << LCDCA_CFG_DUTY_SHIFT)
#  define LCDCA_CFG_DUTY_1TO4      (0 << LCDCA_CFG_DUTY_SHIFT) /* 1/4, 1/3, COM[0:3] */
#  define LCDCA_CFG_DUTY_STATIC    (1 << LCDCA_CFG_DUTY_SHIFT) /* Static, Static, COM0 */
#  define LCDCA_CFG_DUTY_1TO2      (2 << LCDCA_CFG_DUTY_SHIFT) /* 1/2, 1/3, COM[0:1] */
#  define LCDCA_CFG_DUTY_1TO3      (3 << LCDCA_CFG_DUTY_SHIFT) /* 1/3, 1/3, COM[0:2] */

#define LCDCA_CFG_FCST_SHIFT       (16)      /* Bits 16-21: Fine Contrast */
#define LCDCA_CFG_FCST_MASK        (63 << LCDCA_CFG_FCST_SHIFT)
#  define LCDCA_CFG_FCST(n)        (((uint32_t)(n) & 63) << LCDCA_CFG_FCST_SHIFT) /* n = -32..31 */

#define LCDCA_CFG_NSU_SHIFT        (24)      /* Bits 24-29: Number of Segment Terminals in Use */
#define LCDCA_CFG_NSU_MASK         (63 << LCDCA_CFG_NSU_SHIFT)
#  define LCDCA_CFG_NSU(n)         ((n) << LCDCA_CFG_NSU_SHIFT) /* n=0-40 */

/* Timing Register */

#define LCDCA_TIM_PRESC            (1 << 0)  /* Bit 0:  LCD Prescaler Select */
#define LCDCA_TIM_CLKDIV_SHIFT     (1)       /* Bits 1-3: LCD Clock Division */
#define LCDCA_TIM_CLKDIV_MASK      (7 << LCDCA_TIM_CLKDIV_SHIFT)
#  define LCDCA_TIM_CLKDIV(n)      (((n)-1) << LCDCA_TIM_CLKDIV_SHIFT) /* n=1..8 */

#define LCDCA_TIM_FC0_SHIFT        (8)       /* Bits 8-12: Frame Counter 0 */
#define LCDCA_TIM_FC0_MASK         (31 << LCDCA_TIM_FC0_SHIFT)
#  define LCDCA_TIM_FC0(n)         ((n) << LCDCA_TIM_FC0_SHIFT) /* n=0-31 */

#define LCDCA_TIM_FC0PB            (1 << 13) /* Bit 13: Frame Counter 0 Prescaler Bypass */
#define LCDCA_TIM_FC1_SHIFT        (16)      /* Bits 16-20: Frame Counter 1 */
#define LCDCA_TIM_FC1_MASK         (31 << LCDCA_TIM_FC1_SHIFT)
#  define LCDCA_TIM_FC1(n)         ((n) << LCDCA_TIM_FC1_SHIFT) /* n=0-31 */

#define LCDCA_TIM_FC2_SHIFT        (24)      /* Bits 24-28: Frame Counter 2 */
#define LCDCA_TIM_FC2_MASK         (31 << LCDCA_TIM_FC2_SHIFT)
#  define LCDCA_TIM_FC2(n)         ((n) << LCDCA_TIM_FC2_SHIFT) /* n=0-31 */

/* Status Register */

#define LCDCA_SR_FC0R              (1 << 0)  /* Bit 0: Frame Counter 0 Rollover */
#define LCDCA_SR_FC0S              (1 << 1)  /* Bit 1: Frame Counter 0 Status */
#define LCDCA_SR_FC1S              (1 << 2)  /* Bit 2: Frame Counter 1 Status */
#define LCDCA_SR_FC2S              (1 << 3)  /* Bit 3: Frame Counter 2 Status */
#define LCDCA_SR_EN                (1 << 4)  /* Bit 4: LCDCA Status */
#define LCDCA_SR_WEN               (1 << 5)  /* Bit 5: Wake up Status */
#define LCDCA_SR_BLKS              (1 << 6)  /* Bit 6: Blink Status */
#define LCDCA_SR_CSRS              (1 << 7)  /* Bit 7: Circular Shift Register Status */
#define LCDCA_SR_CPS               (1 << 8)  /* Bit 8: Charge Pump Status */

/* Status Clear Register */

#define LCDCA_SCR_FC0R             (1 << 0)  /* Bit 0: Frame Counter 0 Rollover */

/* Data Register Low 0-3 (32-bit data, each bit defines a segment value in
 * display memory for segments 0-31).
 */

#define LCDCA_DRL_MASK             0xffffffff

/* Data Register High 0-3 (8 bits data, each bit defines a segment value in
 * display memory for segments 32-39)
 */

#define LCDCA_DRH_MASK             0xff

/* Indirect Access Data Register */

#define LCDCA_IADR_DATA_SHIFT      (0)       /* Bits 0-7: Segments Value */
#define LCDCA_IADR_DATA_MASK       (0xff << LCDCA_IADR_DATA_SHIFT)
#  define LCDCA_IADR_DATA(n)       ((n) << LCDCA_IADR_DATA_SHIFT)
#  define LCDCA_IADR_DATA0         (0x01 << LCDCA_IADR_DATA_SHIFT)
#  define LCDCA_IADR_DATA1         (0x02 << LCDCA_IADR_DATA_SHIFT)
#  define LCDCA_IADR_DATA2         (0x04 << LCDCA_IADR_DATA_SHIFT)
#  define LCDCA_IADR_DATA3         (0x08 << LCDCA_IADR_DATA_SHIFT)
#  define LCDCA_IADR_DATA4         (0x10 << LCDCA_IADR_DATA_SHIFT)
#  define LCDCA_IADR_DATA5         (0x20 << LCDCA_IADR_DATA_SHIFT)
#  define LCDCA_IADR_DATA6         (0x40 << LCDCA_IADR_DATA_SHIFT)
#  define LCDCA_IADR_DATA7         (0x80 << LCDCA_IADR_DATA_SHIFT)
#define LCDCA_IADR_DMASK_SHIFT     (8)       /* Bits 8-15: Data Mask */
#define LCDCA_IADR_DMASK_MASK      (0xff << LCDCA_IADR_DMASK_SHIFT)
#  define LCDCA_IADR_DMASK(n)      ((n) << LCDCA_IADR_DMASK_SHIFT)
#  define LCDCA_IADR_DMASK0        (0x01 << LCDCA_IADR_DMASK_SHIFT)
#  define LCDCA_IADR_DMASK1        (0x02 << LCDCA_IADR_DMASK_SHIFT)
#  define LCDCA_IADR_DMASK2        (0x04 << LCDCA_IADR_DMASK_SHIFT)
#  define LCDCA_IADR_DMASK3        (0x08 << LCDCA_IADR_DMASK_SHIFT)
#  define LCDCA_IADR_DMASK4        (0x10 << LCDCA_IADR_DMASK_SHIFT)
#  define LCDCA_IADR_DMASK5        (0x20 << LCDCA_IADR_DMASK_SHIFT)
#  define LCDCA_IADR_DMASK6        (0x40 << LCDCA_IADR_DMASK_SHIFT)
#  define LCDCA_IADR_DMASK7        (0x80 << LCDCA_IADR_DMASK_SHIFT)
#define LCDCA_IADR_OFF_SHIFT       (16)      /* Bits 16-20: Byte Offset */
#define LCDCA_IADR_OFF_MASK        (31 << LCDCA_IADR_OFF_SHIFT)
#  define LCDCA_IADR_OFF(n)        (31 << LCDCA_IADR_OFF_SHIFT)

/* Blink Configuration Register */

#define LCDCA_BCFG_MODE            (1 << 0)  /* Bit 0:  Blinking Mode */
#define LCDCA_BCFG_FCS_SHIFT       (1)       /* Bits 1-2: Frame Counter Selection */
#define LCDCA_BCFG_FCS_MASK        (3 << LCDCA_BCFG_FCS_SHIFT)
#  define LCDCA_BCFG_FCS(n)        ((n) << LCDCA_BCFG_FCS_SHIFT) /* n=0-2 */
#  define LCDCA_BCFG_FCS0          (0 << LCDCA_BCFG_FCS_SHIFT)
#  define LCDCA_BCFG_FCS1          (1 << LCDCA_BCFG_FCS_SHIFT)
#  define LCDCA_BCFG_FCS2          (2 << LCDCA_BCFG_FCS_SHIFT)

#define LCDCA_BCFG_BSS0_SHIFT      (8)       /* Bits 8-11: Blink Segment Selection 0 */
#define LCDCA_BCFG_BSS0_MASK       (15 << LCDCA_BCFG_BSS0_SHIFT)
#  define LCDCA_BCFG_BSS0(n)       ((n) << LCDCA_BCFG_BSS0_SHIFT) /* n=bitset */
#  define LCDCA_BCFG_BSS00         (0 << LCDCA_BCFG_BSS0_SHIFT)   /* Segment SEG0/COM0 selected */
#  define LCDCA_BCFG_BSS01         (0 << LCDCA_BCFG_BSS0_SHIFT)   /* Segment SEG0/COM1 selected */
#  define LCDCA_BCFG_BSS02         (0 << LCDCA_BCFG_BSS0_SHIFT)   /* Segment SEG0/COM2 selected */
#  define LCDCA_BCFG_BSS03         (0 << LCDCA_BCFG_BSS0_SHIFT)   /* Segment SEG0/COM3 selected */

#define LCDCA_BCFG_BSS1_SHIFT      (12)      /* Bits 12-15: Blink Segment Selection 1 */
#define LCDCA_BCFG_BSS1_MASK       (15 << LCDCA_BCFG_BSS1_SHIFT)
#  define LCDCA_BCFG_BSS1(n)       ((n) << LCDCA_BCFG_BSS1_SHIFT) /* n=bitset */
#  define LCDCA_BCFG_BSS10         (0 << LCDCA_BCFG_BSS1_SHIFT)   /* Segment SEG1/COM0 selected */
#  define LCDCA_BCFG_BSS11         (0 << LCDCA_BCFG_BSS1_SHIFT)   /* Segment SEG1/COM1 selected */
#  define LCDCA_BCFG_BSS12         (0 << LCDCA_BCFG_BSS1_SHIFT)   /* Segment SEG1/COM2 selected */
#  define LCDCA_BCFG_BSS13         (0 << LCDCA_BCFG_BSS1_SHIFT)   /* Segment SEG1/COM3 selected */

/* Circular Shift Register Configuration */

#define LCDCA_CSRCFG_DIR           (1 << 0)  /* Bit 0:  Direction */
#define LCDCA_CSRCFG_FCS_SHIFT     (1)       /* Bits 1-2: Frame Counter Selection */
#define LCDCA_CSRCFG_FCS_MASK      (3 << LCDCA_CSRCFG_FCS_SHIFT)
#  define LCDCA_CSRCFG_FCS(n)      ((n) << LCDCA_CSRCFG_FCS_SHIFT) /* n=0-2 */
#  define LCDCA_CSRCFG_FCS0        (0 << LCDCA_CSRCFG_FCS_SHIFT)
#  define LCDCA_CSRCFG_FCS1        (1 << LCDCA_CSRCFG_FCS_SHIFT)
#  define LCDCA_CSRCFG_FCS2        (2 << LCDCA_CSRCFG_FCS_SHIFT)

#define LCDCA_CSRCFG_SIZE_SHIFT    (3)       /* Bits 3-5: Size */
#define LCDCA_CSRCFG_SIZE_MASK     (7 << LCDCA_CSRCFG_SIZE_SHIFT)
#  define LCDCA_CSRCFG_SIZE(n)     (((n)-1) << LCDCA_CSRCFG_SIZE_SHIFT) /* n=1..8 */

#define LCDCA_CSRCFG_DATA_SHIFT    (8)       /* Bits 8-15: Circular Shift Register Value */
#define LCDCA_CSRCFG_DATA_MASK     (0xff << LCDCA_CSRCFG_DATA_SHIFT)
#  define LCDCA_CSRCFG_DATA(n)     ((n) << LCDCA_CSRCFG_DATA_SHIFT)

/* Character Mapping Configuration Register */

#define LCDCA_CMCFG_DREV           (1 << 0)  /* Bit 0:  Digit Reverse Mode */
#define LCDCA_CMCFG_TDG_SHIFT      (1)       /* Bits 1-2: Type of Digit */
#define LCDCA_CMCFG_TDG_MASK       (3 << LCDCA_CMCFG_TDG_SHIFT)
#  define LCDCA_CMCFG_TDG_7S3C     (0 << LCDCA_CMCFG_TDG_SHIFT) /* 7-segment with 3 common terminals */
#  define LCDCA_CMCFG_TDG_7S4C     (1 << LCDCA_CMCFG_TDG_SHIFT) /* 7-segment with 4 common terminals */
#  define LCDCA_CMCFG_TDG_14S4C    (2 << LCDCA_CMCFG_TDG_SHIFT) /* 14-segment with 4 common terminals */
#  define LCDCA_CMCFG_TDG_14S3C    (3 << LCDCA_CMCFG_TDG_SHIFT) /* 16-segment with 3 common terminals */

#define LCDCA_CMCFG_STSEG_SHIFT    (8)       /* Bits 8-13: Start Segment */
#define LCDCA_CMCFG_STSEG_MASK     (63 << LCDCA_CMCFG_STSEG_SHIFT)
#  define LCDCA_CMCFG_STSEG(n)     ((n) << LCDCA_CMCFG_STSEG_SHIFT)

/* Character Mapping Data Register */

#define LCDCA_CMDR_MASK            0x7f

/* Automated Character Mapping Configuration Register */

#define LCDCA_ACMCFG_EN            (1 << 0)  /* Bit 0:  Enable */
#define LCDCA_ACMCFG_FCS_SHIFT     (1)       /* Bits 1-2: Frame Counter Selection */
#define LCDCA_ACMCFG_FCS_MASK      (3 << LCDCA_ACMCFG_FCS_SHIFT)
#  define LCDCA_ACMCFG_FCS(n)      ((n) << LCDCA_ACMCFG_FCS_SHIFT) /* n=0-2 */
#  define LCDCA_ACMCFG_FCS0        (0 << LCDCA_ACMCFG_FCS_SHIFT)
#  define LCDCA_ACMCFG_FCS1        (1 << LCDCA_ACMCFG_FCS_SHIFT)
#  define LCDCA_ACMCFG_FCS2        (2 << LCDCA_ACMCFG_FCS_SHIFT)

#define LCDCA_ACMCFG_MODE          (1 << 3)  /* Bit 3:  Mode */
#define LCDCA_ACMCFG_DREV          (1 << 4)  /* Bit 4:  Digit Reverse */
#define LCDCA_ACMCFG_TDG_SHIFT     (5)       /* Bits 5-6: Type of Digit */
#define LCDCA_ACMCFG_TDG_MASK      (3 << LCDCA_ACMCFG_TDG_SHIFT)
#  define LCDCA_ACMCFG_TDG_7S3C    (0 << LCDCA_ACMCFG_TDG_SHIFT) /* 7-segment with 3 common terminals */
#  define LCDCA_ACMCFG_TDG_7S4C    (1 << LCDCA_ACMCFG_TDG_SHIFT) /* 7-segment with 4 common terminals */
#  define LCDCA_ACMCFG_TDG_14S4C   (2 << LCDCA_ACMCFG_TDG_SHIFT) /* 14-segment with 4 common terminals */
#  define LCDCA_ACMCFG_TDG_14S3C   (3 << LCDCA_ACMCFG_TDG_SHIFT) /* 16-segment with 3 common terminals */

#define LCDCA_ACMCFG_STSEG_SHIFT   (8)       /* Bits 8-13: Start Segment */
#define LCDCA_ACMCFG_STSEG_MASK    (63 << LCDCA_ACMCFG_STSEG_SHIFT)
#  define LCDCA_ACMCFG_STSEG(n)    ((n) << LCDCA_ACMCFG_STSEG_SHIFT)

#define LCDCA_ACMCFG_STEPS_SHIFT   (16)      /* Bits 16-23: Scrolling Steps */
#define LCDCA_ACMCFG_STEPS_MASK    (0xff << LCDCA_ACMCFG_STEPS_SHIFT)
#  define LCDCA_ACMCFG_STEPS(n)    ((n) << LCDCA_ACMCFG_STEPS_SHIFT) /* n = string length - DIGN + 1 */

#define LCDCA_ACMCFG_DIGN_SHIFT    (24)      /* Bits 24-27: Digit Number */
#define LCDCA_ACMCFG_DIGN_MASK     (15 << LCDCA_ACMCFG_DIGN_SHIFT)
#  define LCDCA_ACMCFG_DIGN(n)     ((n) << LCDCA_ACMCFG_DIGN_SHIFT) /* n=1..15 */

/* Automated Character Mapping Data Register */

#define LCDCA_ACMDR_MASK           0x7f

/* Automated Bit Mapping Configuration Register */

#define LCDCA_ABMCFG_EN            (1 << 0)  /* Bit 0:  Enable */
#define LCDCA_ABMCFG_FCS_SHIFT     (1)       /* Bits 1-2: Frame Counter Selection */
#define LCDCA_ABMCFG_FCS_MASK      (3 << LCDCA_ABMCFG_FCS_SHIFT)
#  define LCDCA_ABMCFG_FCS(n)      ((n) << LCDCA_ABMCFG_FCS_SHIFT) /* n=0-2 */
#  define LCDCA_ABMCFG_FCS0        (0 << LCDCA_ABMCFG_FCS_SHIFT)
#  define LCDCA_ABMCFG_FCS1        (1 << LCDCA_ABMCFG_FCS_SHIFT)
#  define LCDCA_ABMCFG_FCS2        (2 << LCDCA_ABMCFG_FCS_SHIFT)

#define LCDCA_ABMCFG_SIZE_SHIFT    (8)       /* Bits 8-12: Size */
#define LCDCA_ABMCFG_SIZE_MASK     (31 << LCDCA_ABMCFG_SIZE_SHIFT)
#  define LCDCA_ABMCFG_SIZE(n)     (((n)-1) << LCDCA_ABMCFG_SIZE_SHIFT) /* n=1..31 */

/* Automated Bit Mapping Data Register */

#define LCDCA_ABMDR_DATA_SHIFT      (0)       /* Bits 0-7: Segments Value */
#define LCDCA_ABMDR_DATA_MASK       (0xff << LCDCA_ABMDR_DATA_SHIFT)
#  define LCDCA_ABMDR_DATA(n)       ((n) << LCDCA_ABMDR_DATA_SHIFT)
#  define LCDCA_ABMDR_DATA0         (0x01 << LCDCA_ABMDR_DATA_SHIFT)
#  define LCDCA_ABMDR_DATA1         (0x02 << LCDCA_ABMDR_DATA_SHIFT)
#  define LCDCA_ABMDR_DATA2         (0x04 << LCDCA_ABMDR_DATA_SHIFT)
#  define LCDCA_ABMDR_DATA3         (0x08 << LCDCA_ABMDR_DATA_SHIFT)
#  define LCDCA_ABMDR_DATA4         (0x10 << LCDCA_ABMDR_DATA_SHIFT)
#  define LCDCA_ABMDR_DATA5         (0x20 << LCDCA_ABMDR_DATA_SHIFT)
#  define LCDCA_ABMDR_DATA6         (0x40 << LCDCA_ABMDR_DATA_SHIFT)
#  define LCDCA_ABMDR_DATA7         (0x80 << LCDCA_ABMDR_DATA_SHIFT)
#define LCDCA_ABMDR_DMASK_SHIFT     (8)       /* Bits 8-15: Data Mask */
#define LCDCA_ABMDR_DMASK_MASK      (0xff << LCDCA_ABMDR_DMASK_SHIFT)
#  define LCDCA_ABMDR_DMASK(n)      ((n) << LCDCA_ABMDR_DMASK_SHIFT)
#  define LCDCA_ABMDR_DMASK0        (0x01 << LCDCA_ABMDR_DMASK_SHIFT)
#  define LCDCA_ABMDR_DMASK1        (0x02 << LCDCA_ABMDR_DMASK_SHIFT)
#  define LCDCA_ABMDR_DMASK2        (0x04 << LCDCA_ABMDR_DMASK_SHIFT)
#  define LCDCA_ABMDR_DMASK3        (0x08 << LCDCA_ABMDR_DMASK_SHIFT)
#  define LCDCA_ABMDR_DMASK4        (0x10 << LCDCA_ABMDR_DMASK_SHIFT)
#  define LCDCA_ABMDR_DMASK5        (0x20 << LCDCA_ABMDR_DMASK_SHIFT)
#  define LCDCA_ABMDR_DMASK6        (0x40 << LCDCA_ABMDR_DMASK_SHIFT)
#  define LCDCA_ABMDR_DMASK7        (0x80 << LCDCA_ABMDR_DMASK_SHIFT)
#define LCDCA_ABMDR_OFF_SHIFT       (16)      /* Bits 16-20: Byte Offset */
#define LCDCA_ABMDR_OFF_MASK        (31 << LCDCA_ABMDR_OFF_SHIFT)
#  define LCDCA_ABMDR_OFF(n)        (31 << LCDCA_ABMDR_OFF_SHIFT)

/* Interrupt Enable Register */

/* Interrupt Disable Register */

/* Interrupt Mask Register */

#define LCDCA_INT_FC0R              (1 << 0)  /* Bit 0: Frame Counter 0 Rollover */

/* Version Register */

#define LCDCA_VERSION_SHIFT          (0)        /* Bits 0-11: Version Number */
#define LCDCA_VERSION_MASK           (0xfff << LCDCA_VERSION_VERSION_SHIFT)
#define LCDCA_VARIANT_SHIFT          (16)       /* Bits 16-19: Variant Number */
#define LCDCA_VARIANT_MASK           (15 << LCDCA_VARIANT_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_LCDCA_H */
