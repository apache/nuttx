/****************************************************************************************
 * arch/arm/src/samv7/chip/sam_afec.h
 * Analog-Front-End Controller (AFEC) definitions for the SAMV71
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAMV7_CHIP_SAM_AFEC_H
#define __ARCH_ARM_SRC_SAMV7_CHIP_SAM_AFEC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

#include "chip/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/
/* General definitions ******************************************************************/

#define SAM_ADC_NCHANNELS            12     /* 12 ADC Channels */

/* AFEC register offsets ****************************************************************/

#define SAM_AFEC_CR_OFFSET           0x0000 /* Control Register */
#define SAM_AFEC_MR_OFFSET           0x0004 /* Mode Register */
#define SAM_AFEC_EMR_OFFSET          0x0008 /* Extended Mode Register */
#define SAM_AFEC_SEQ1R_OFFSET        0x000c /* Channel Sequence 1 Register */
#define SAM_AFEC_SEQ2R_OFFSET        0x0010 /* Channel Sequence 2 Register */
#define SAM_AFEC_CHER_OFFSET         0x0014 /* Channel Enable Register */
#define SAM_AFEC_CHDR_OFFSET         0x0018 /* Channel Disable Register */
#define SAM_AFEC_CHSR_OFFSET         0x001c /* Channel Status Register */
#define SAM_AFEC_LCDR_OFFSET         0x0020 /* Last Converted Data Register */
#define SAM_AFEC_IER_OFFSET          0x0024 /* Interrupt Enable Register */
#define SAM_AFEC_IDR_OFFSET          0x0028 /* Interrupt Disable Register */
#define SAM_AFEC_IMR_OFFSET          0x002c /* Interrupt Mask Register */
#define SAM_AFEC_ISR_OFFSET          0x0030 /* Interrupt Status Register */
                                            /* 0x0034-0x0040 Reserved */
                                            /* 0x0044-0x0048 Reserved */
#define SAM_AFEC_OVER_OFFSET         0x004c /* Overrun Status Register */
#define SAM_AFEC_CWR_OFFSET          0x0050 /* Compare Window Register */
#define SAM_AFEC_CGR_OFFSET          0x0054 /* Channel Gain Register */
#define SAM_AFEC_CDOR_OFFSET         0x005c /* Channel Calibration DC Offset Register */
#define SAM_AFEC_DIFFR_OFFSET        0x0060 /* Channel Differential Register */
#define SAM_AFEC_CSELR_OFFSET        0x0064 /* Channel Register Selection */
#define SAM_AFEC_CDR_OFFSET          0x0068 /* Channel Data Register */
#define SAM_AFEC_COCR_OFFSET         0x006c /* Channel Offset Compensation Register */
#define SAM_AFEC_TEMPMR_OFFSET       0x0070 /* Temperature Sensor Mode Register */
#define SAM_AFEC_TEMPCWR_OFFSET      0x0074 /* Temperature Compare Window Register */
                                            /* 0x0078-0x0090 Reserved */
#define SAM_AFEC_ACR_OFFSET          0x0094 /* Analog Control Register */
                                            /* 0x0098-0x009c Reserved */
#define SAM_AFEC_SHMR_OFFSET         0x00a0 /* Sample & Hold Mode Register */
                                            /* 0x00a4-0x00ac Reserved */
#define SAM_AFEC_COSR_OFFSET         0x00d0 /* Correction Select Register */
#define SAM_AFEC_CVR_OFFSET          0x00d4 /* Correction Values Register */
#define SAM_AFEC_CECR_OFFSET         0x00d8 /* Channel Error Correction Register */
                                            /* 0x00dc-0x00e0 Reserved */
#define SAM_AFEC_WPMR_OFFSET         0x00e4 /* Write Protect Mode Register */
#define SAM_AFEC_WPSR_OFFSET         0x00e8 /* Write Protect Status Register */
                                            /* 0x00ec-0x00f8 Reserved */
                                            /* 0x0fc Reserved */

/* AFEC register addresses **************************************************************/

#define SAM_AFEC0_CR                 (SAM_AFEC0_BASE+SAM_AFEC_CR_OFFSET)
#define SAM_AFEC0_MR                 (SAM_AFEC0_BASE+SAM_AFEC_MR_OFFSET)
#define SAM_AFEC0_EMR                (SAM_AFEC0_BASE+SAM_AFEC_EMR_OFFSET)
#define SAM_AFEC0_SEQ1R              (SAM_AFEC0_BASE+SAM_AFEC_SEQ1R_OFFSET)
#define SAM_AFEC0_SEQ2R              (SAM_AFEC0_BASE+SAM_AFEC_SEQ2R_OFFSET)
#define SAM_AFEC0_CHER               (SAM_AFEC0_BASE+SAM_AFEC_CHER_OFFSET)
#define SAM_AFEC0_CHDR               (SAM_AFEC0_BASE+SAM_AFEC_CHDR_OFFSET)
#define SAM_AFEC0_CHSR               (SAM_AFEC0_BASE+SAM_AFEC_CHSR_OFFSET)
#define SAM_AFEC0_LCDR               (SAM_AFEC0_BASE+SAM_AFEC_LCDR_OFFSET)
#define SAM_AFEC0_IER                (SAM_AFEC0_BASE+SAM_AFEC_IER_OFFSET)
#define SAM_AFEC0_IDR                (SAM_AFEC0_BASE+SAM_AFEC_IDR_OFFSET)
#define SAM_AFEC0_IMR                (SAM_AFEC0_BASE+SAM_AFEC_IMR_OFFSET)
#define SAM_AFEC0_ISR                (SAM_AFEC0_BASE+SAM_AFEC_ISR_OFFSET)
#define SAM_AFEC0_OVER               (SAM_AFEC0_BASE+SAM_AFEC_OVER_OFFSET)
#define SAM_AFEC0_CWR                (SAM_AFEC0_BASE+SAM_AFEC_CWR_OFFSET)
#define SAM_AFEC0_CGR                (SAM_AFEC0_BASE+SAM_AFEC_CGR_OFFSET)
#define SAM_AFEC0_CDOR               (SAM_AFEC0_BASE+SAM_AFEC_CDOR_OFFSET)
#define SAM_AFEC0_DIFFR              (SAM_AFEC0_BASE+SAM_AFEC_DIFFR_OFFSET)
#define SAM_AFEC0_CSELR              (SAM_AFEC0_BASE+SAM_AFEC_CSELR_OFFSET)
#define SAM_AFEC0_CDR                (SAM_AFEC0_BASE+SAM_AFEC_CDR_OFFSET)
#define SAM_AFEC0_COCR               (SAM_AFEC0_BASE+SAM_AFEC_COCR_OFFSET)
#define SAM_AFEC0_TEMPMR             (SAM_AFEC0_BASE+SAM_AFEC_TEMPMR_OFFSET)
#define SAM_AFEC0_TEMPCWR            (SAM_AFEC0_BASE+SAM_AFEC_TEMPCWR_OFFSET)
#define SAM_AFEC0_ACR                (SAM_AFEC0_BASE+SAM_AFEC_ACR_OFFSET)
#define SAM_AFEC0_SHMR               (SAM_AFEC0_BASE+SAM_AFEC_SHMR_OFFSET)
#define SAM_AFEC0_COSR               (SAM_AFEC0_BASE+SAM_AFEC_COSR_OFFSET)
#define SAM_AFEC0_CVR                (SAM_AFEC0_BASE+SAM_AFEC_CVR_OFFSET)
#define SAM_AFEC0_CECR               (SAM_AFEC0_BASE+SAM_AFEC_CECR_OFFSET)
#define SAM_AFEC0_WPMR               (SAM_AFEC0_BASE+SAM_AFEC_WPMR_OFFSET)
#define SAM_AFEC0_WPSR               (SAM_AFEC0_BASE+SAM_AFEC_WPSR_OFFSET)

#define SAM_AFEC1_CR                 (SAM_AFEC1_BASE+SAM_AFEC_CR_OFFSET)
#define SAM_AFEC1_MR                 (SAM_AFEC1_BASE+SAM_AFEC_MR_OFFSET)
#define SAM_AFEC1_EMR                (SAM_AFEC1_BASE+SAM_AFEC_EMR_OFFSET)
#define SAM_AFEC1_SEQ1R              (SAM_AFEC1_BASE+SAM_AFEC_SEQ1R_OFFSET)
#define SAM_AFEC1_SEQ2R              (SAM_AFEC1_BASE+SAM_AFEC_SEQ2R_OFFSET)
#define SAM_AFEC1_CHER               (SAM_AFEC1_BASE+SAM_AFEC_CHER_OFFSET)
#define SAM_AFEC1_CHDR               (SAM_AFEC1_BASE+SAM_AFEC_CHDR_OFFSET)
#define SAM_AFEC1_CHSR               (SAM_AFEC1_BASE+SAM_AFEC_CHSR_OFFSET)
#define SAM_AFEC1_LCDR               (SAM_AFEC1_BASE+SAM_AFEC_LCDR_OFFSET)
#define SAM_AFEC1_IER                (SAM_AFEC1_BASE+SAM_AFEC_IER_OFFSET)
#define SAM_AFEC1_IDR                (SAM_AFEC1_BASE+SAM_AFEC_IDR_OFFSET)
#define SAM_AFEC1_IMR                (SAM_AFEC1_BASE+SAM_AFEC_IMR_OFFSET)
#define SAM_AFEC1_ISR                (SAM_AFEC1_BASE+SAM_AFEC_ISR_OFFSET)
#define SAM_AFEC1_OVER               (SAM_AFEC1_BASE+SAM_AFEC_OVER_OFFSET)
#define SAM_AFEC1_CWR                (SAM_AFEC1_BASE+SAM_AFEC_CWR_OFFSET)
#define SAM_AFEC1_CGR                (SAM_AFEC1_BASE+SAM_AFEC_CGR_OFFSET)
#define SAM_AFEC1_CDOR               (SAM_AFEC1_BASE+SAM_AFEC_CDOR_OFFSET)
#define SAM_AFEC1_DIFFR              (SAM_AFEC1_BASE+SAM_AFEC_DIFFR_OFFSET)
#define SAM_AFEC1_CSELR              (SAM_AFEC1_BASE+SAM_AFEC_CSELR_OFFSET)
#define SAM_AFEC1_CDR                (SAM_AFEC1_BASE+SAM_AFEC_CDR_OFFSET)
#define SAM_AFEC1_COCR               (SAM_AFEC1_BASE+SAM_AFEC_COCR_OFFSET)
#define SAM_AFEC1_TEMPMR             (SAM_AFEC1_BASE+SAM_AFEC_TEMPMR_OFFSET)
#define SAM_AFEC1_TEMPCWR            (SAM_AFEC1_BASE+SAM_AFEC_TEMPCWR_OFFSET)
#define SAM_AFEC1_ACR                (SAM_AFEC1_BASE+SAM_AFEC_ACR_OFFSET)
#define SAM_AFEC1_SHMR               (SAM_AFEC1_BASE+SAM_AFEC_SHMR_OFFSET)
#define SAM_AFEC1_COSR               (SAM_AFEC1_BASE+SAM_AFEC_COSR_OFFSET)
#define SAM_AFEC1_CVR                (SAM_AFEC1_BASE+SAM_AFEC_CVR_OFFSET)
#define SAM_AFEC1_CECR               (SAM_AFEC1_BASE+SAM_AFEC_CECR_OFFSET)
#define SAM_AFEC1_WPMR               (SAM_AFEC1_BASE+SAM_AFEC_WPMR_OFFSET)
#define SAM_AFEC1_WPSR               (SAM_AFEC1_BASE+SAM_AFEC_WPSR_OFFSET)

/* AFEC register bit definitions *******************************************************/

/* Control Register */

#define AFEC_CR_SWRST                (1 << 0)  /* Bit 0:  Software Reset */
#define AFEC_CR_START                (1 << 1)  /* Bit 1:  Start Conversion */

/* Mode Register */

#define AFEC_MR_TRGEN                (1 << 0)  /* Bit 0: Trigger Enable */
#define AFEC_MR_TRGSEL_SHIFT         (1)       /* Bits 1-3: Trigger Selection */
#define AFEC_MR_TRGSEL_MASK          (7 << AFEC_MR_TRGSEL_SHIFT)
#  define AFEC_MR_TRGSEL_ADTRG       (0 << AFEC_MR_TRGSEL_SHIFT) /* ADTRG */
#  define AFEC_MR_TRGSEL_TIOA0       (1 << AFEC_MR_TRGSEL_SHIFT) /* TIOA0 */
#  define AFEC_MR_TRGSEL_TIOA1       (2 << AFEC_MR_TRGSEL_SHIFT) /* TIOA1 */
#  define AFEC_MR_TRGSEL_TIOA2       (3 << AFEC_MR_TRGSEL_SHIFT) /* TIOA2 */
#  define AFEC_MR_TRGSEL_PWM0        (4 << AFEC_MR_TRGSEL_SHIFT) /* PWM Event Line 0 */
#  define AFEC_MR_TRGSEL_PWM1        (5 << AFEC_MR_TRGSEL_SHIFT) /* PWM Event Line 1 */
#  define AFEC_MR_TRGSEL_ACMP        (6 << AFEC_MR_TRGSEL_SHIFT) /* Analog comparator */
#define AFEC_MR_SLEEP                (1 << 5)  /* Bit 5:  Sleep Mode */
#define AFEC_MR_FWUP                 (1 << 6)  /* Bit 6:  Fast Wake Up */
#define AFEC_MR_FREERUN              (1 << 7)  /* Bit 7:  Free Run Mode */
#define AFEC_MR_PRESCAL_SHIFT        (8)       /* Bits 8-15: Prescaler Rate Selection */
#define AFEC_MR_PRESCAL_MASK         (0xff << AFEC_MR_PRESCAL_SHIFT)
#  define AFEC_MR_PRESCAL(n)         ((uint32_t)(n) << AFEC_MR_PRESCAL_SHIFT)
#define AFEC_MR_STARTUP_SHIFT        (16)      /* Bits 16-19: Start Up Time */
#define AFEC_MR_STARTUP_MASK         (15 << AFEC_MR_STARTUP_SHIFT)
#  define AFEC_MR_STARTUP_0          (0 << AFEC_MR_STARTUP_SHIFT)  /* 0 periods of ADCClock */
#  define AFEC_MR_STARTUP_8          (1 << AFEC_MR_STARTUP_SHIFT)  /* 8 periods of ADCClock */
#  define AFEC_MR_STARTUP_16         (2 << AFEC_MR_STARTUP_SHIFT)  /* 16 periods of ADCClock */
#  define AFEC_MR_STARTUP_24         (3 << AFEC_MR_STARTUP_SHIFT)  /* 24 periods of ADCClock */
#  define AFEC_MR_STARTUP_64         (4 << AFEC_MR_STARTUP_SHIFT)  /* 64 periods of ADCClock */
#  define AFEC_MR_STARTUP_80         (5 << AFEC_MR_STARTUP_SHIFT)  /* 80 periods of ADCClock */
#  define AFEC_MR_STARTUP_96         (6 << AFEC_MR_STARTUP_SHIFT)  /* 96 periods of ADCClock */
#  define AFEC_MR_STARTUP_112        (7 << AFEC_MR_STARTUP_SHIFT)  /* 112 periods of ADCClock */
#  define AFEC_MR_STARTUP_512        (8 << AFEC_MR_STARTUP_SHIFT)  /* 512 periods of ADCClock */
#  define AFEC_MR_STARTUP_576        (9 << AFEC_MR_STARTUP_SHIFT)  /* 576 periods of ADCClock */
#  define AFEC_MR_STARTUP_640        (10 << AFEC_MR_STARTUP_SHIFT) /* 640 periods of ADCClock */
#  define AFEC_MR_STARTUP_704        (11 << AFEC_MR_STARTUP_SHIFT) /* 704 periods of ADCClock */
#  define AFEC_MR_STARTUP_768        (12 << AFEC_MR_STARTUP_SHIFT) /* 768 periods of ADCClock */
#  define AFEC_MR_STARTUP_832        (13 << AFEC_MR_STARTUP_SHIFT) /* 832 periods of ADCClock */
#  define AFEC_MR_STARTUP_896        (14 << AFEC_MR_STARTUP_SHIFT) /* 896 periods of ADCClock */
#  define AFEC_MR_STARTUP_960        (15 << AFEC_MR_STARTUP_SHIFT) /* 960 periods of ADCClock */
#define AFEC_MR_SETTLING_SHIFT       (20)      /* Bits 20-21: Analog Settling Time */
#define AFEC_MR_SETTLING_MASK        (15 << AFEC_MR_SETTLING_SHIFT)
#  define AFEC_MR_SETTLING_3         (0 << AFEC_MR_SETTLING_SHIFT) /* 3 periods of ADCClock */
#  define AFEC_MR_SETTLING_5         (1 << AFEC_MR_SETTLING_SHIFT) /* 5 periods of ADCClock */
#  define AFEC_MR_SETTLING_9         (2 << AFEC_MR_SETTLING_SHIFT) /* 9 periods of ADCClock */
#  define AFEC_MR_SETTLING_17        (3 << AFEC_MR_SETTLING_SHIFT) /* 17 periods of ADCClock */
#define AFEC_MR_ONE                  (1 << 23) /* Bit 23: Must be one */
#define AFEC_MR_TRACKTIM_SHIFT       (24)      /* Bits 24-27: Tracking Time */
#define AFEC_MR_TRACKTIM_MASK        (15 << AFEC_MR_TRACKTIM_SHIFT)
#  define AFEC_MR_TRACKTIM(n)        ((uint32_t)(n) << AFEC_MR_TRACKTIM_SHIFT)
#define AFEC_MR_TRANSFER_SHIFT       (28)      /* Bits 28-29: Transfer Period */
#define AFEC_MR_TRANSFER_MASK        (3 << AFEC_MR_TRANSFER_SHIFT)
#  define AFEC_MR_TRANSFER(n)        ((uint32_t)(n) << AFEC_MR_TRANSFER_SHIFT)
#define AFEC_MR_USEQ                 (1 << 31) /* Bit 31: Use Sequence Enable */

/* Extended Mode Register */

#define AFEC_EMR_CMPMODE_SHIFT       (0)      /* Bit 0-1: Comparison Mode */
#define AFEC_EMR_CMPMODE_MASK        (3 << AFEC_EMR_CMPMODE_SHIFT)
#  define AFEC_EMR_CMPMODE_LOW       (0 << AFEC_EMR_CMPMODE_SHIFT) /* Event when lower than low window threshold */
#  define AFEC_EMR_CMPMODE_HIGH      (1 << AFEC_EMR_CMPMODE_SHIFT) /* Event when higher than high window threshold */
#  define AFEC_EMR_CMPMODE_IN        (2 << AFEC_EMR_CMPMODE_SHIFT) /* Event when in comparison window */
#  define AFEC_EMR_CMPMODE_OUT       (3 << AFEC_EMR_CMPMODE_SHIFT) /* Event when out of comparison window */
#define AFEC_EMR_CMPSEL_SHIFT        (3)       /* Bit 3-7: Comparison Selected Channel */
#define AFEC_EMR_CMPSEL_MASK         (31 << AFEC_EMR_CMPSEL_SHIFT)
#  define AFEC_EMR_CMPSEL(n)         ((uint32_t)(n) << AFEC_EMR_CMPSEL_SHIFT)
#define AFEC_EMR_CMPALL              (1 << 9)  /* Bit 9:  Compare All Channels */
#define AFEC_EMR_CMPFILTER_SHIFT     (12)      /* Bits 12-13: Compare Event Filtering */
#define AFEC_EMR_CMPFILTER_MASK      (3 << AFEC_EMR_CMPFILTER_SHIFT)
#  define AFEC_EMR_CMPFILTER(n)      ((uint32_t)(n) << AFEC_EMR_CMPFILTER_SHIFT)
#define AFEC_EMR_RES_SHIFT           (16)      /* Bits 16-18: Resolution */
#define AFEC_EMR_RES_MASK            (7 << AFEC_EMR_RES_SHIFT)
#  define AFEC_EMR_RES_NOAVG         (0 << AFEC_EMR_RES_SHIFT) /* 12-bit resolution, AFEC sample rate is maximum (no averaging) */
#  define AFEC_EMR_RES_OSR4          (2 << AFEC_EMR_RES_SHIFT) /* 13-bit resolution, AFEC sample rate divided by 4 (averaging) */
#  define AFEC_EMR_RES_OSR16         (3 << AFEC_EMR_RES_SHIFT) /* 14-bit resolution, AFEC sample rate divided by 16 (averaging) */
#  define AFEC_EMR_RES_OSR64         (4 << AFEC_EMR_RES_SHIFT) /* 15-bit resolution, AFEC sample rate divided by 64 (averaging) */
#  define AFEC_EMR_RES_OSR256        (5 << AFEC_EMR_RES_SHIFT) /* 16-bit resolution, AFEC sample rate divided by 256 (averaging) */
#define AFEC_EMR_TAG                 (1 << 24) /* Bit 24: TAG of the AFEC_LDCR register */
#define AFEC_EMR_STM                 (1 << 25) /* Bit 25: Single Trigger Mode */
#define AFEC_EMR_SIGNMODE_SHIFT      (28)      /* Bits 28-29: Sign mode */
#define AFEC_EMR_SIGNMODE_MASK       (3 << AFEC_EMR_SIGNMODE_SHIFT)
#  define AFEC_EMR_SIGNMODE_SEUNSG   (0 << AFEC_EMR_SIGNMODE_SHIFT) /* Single ended channels unsigned */
#  define AFEC_EMR_SIGNMODE_DFSIGN   (0 << AFEC_EMR_SIGNMODE_SHIFT) /* Differential channels signed */
#  define AFEC_EMR_SIGNMODE_SESIGN   (1 << AFEC_EMR_SIGNMODE_SHIFT) /* Singed ended channels signed */
#  define AFEC_EMR_SIGNMODE_DFUNSG   (1 << AFEC_EMR_SIGNMODE_SHIFT) /* Differential channels unsiged */
#  define AFEC_EMR_SIGNMODE_UNSIGNED (2 << AFEC_EMR_SIGNMODE_SHIFT) /* All channels unsigned */
#  define AFEC_EMR_SIGNMODE_SIGNED   (2 << AFEC_EMR_SIGNMODE_SHIFT) /* All channels signed */

/* Channel Sequence 1 Register */

#define AFEC_SEQ1R_USCH_SHIFT(n)     ((n) << 2) /* n=0..7 */
#define AFEC_SEQ1R_USCH_MASK(n)      (15 << AFEC_SEQ1R_USCH_SHIFT(n))
#  define AFEC_SEQ1R_USCH(n,v)       ((uint32_t)(v) << AFEC_SEQ1R_USCH_SHIFT(n))
#define AFEC_SEQ1R_USCH0_SHIFT       (0) /* Bits 0-3: User sequence number 0 */
#define AFEC_SEQ1R_USCH0_MASK        (15 << AFEC_SEQ1R_USCH0_SHIFT)
#  define AFEC_SEQ1R_USCH0(v)        ((uint32_t)(v) << AFEC_SEQ1R_USCH0_SHIFT)
#define AFEC_SEQ1R_USCH1_SHIFT       (4) /* Bits 4-7: User sequence number 1 */
#define AFEC_SEQ1R_USCH1_MASK        (15 << AFEC_SEQ1R_USCH1_SHIFT)
#  define AFEC_SEQ1R_USCH1(v)        ((uint32_t)(v) << AFEC_SEQ1R_USCH1_SHIFT)
#define AFEC_SEQ1R_USCH2_SHIFT       (8) /* Bits 8-11: User sequence number 2 */
#define AFEC_SEQ1R_USCH2_MASK        (15 << AFEC_SEQ1R_USCH2_SHIFT)
#  define AFEC_SEQ1R_USCH2(v)        ((uint32_t)(v) << AFEC_SEQ1R_USCH2_SHIFT)
#define AFEC_SEQ1R_USCH3_SHIFT       (12) /* Bits 12-15: User sequence number 3 */
#define AFEC_SEQ1R_USCH3_MASK        (15 << AFEC_SEQ1R_USCH3_SHIFT)
#  define AFEC_SEQ1R_USCH3(v)        ((uint32_t)(v) << AFEC_SEQ1R_USCH3_SHIFT)
#define AFEC_SEQ1R_USCH4_SHIFT       (16) /* Bits 16-19: User sequence number 4 */
#define AFEC_SEQ1R_USCH4_MASK        (15 << AFEC_SEQ1R_USCH4_SHIFT)
#  define AFEC_SEQ1R_USCH4(v)        ((uint32_t)(v) << AFEC_SEQ1R_USCH4_SHIFT)
#define AFEC_SEQ1R_USCH5_SHIFT       (20) /* Bits 20-23: User sequence number 5 */
#define AFEC_SEQ1R_USCH5_MASK        (15 << AFEC_SEQ1R_USCH5_SHIFT)
#  define AFEC_SEQ1R_USCH5(v)        ((uint32_t)(v) << AFEC_SEQ1R_USCH5_SHIFT)
#define AFEC_SEQ1R_USCH6_SHIFT       (24) /* Bits 24-27: User sequence number 6 */
#define AFEC_SEQ1R_USCH6_MASK        (15 << AFEC_SEQ1R_USCH6_SHIFT)
#  define AFEC_SEQ1R_USCH6(v)        ((uint32_t)(v) << AFEC_SEQ1R_USCH6_SHIFT)
#define AFEC_SEQ1R_USCH7_SHIFT       (28) /* Bits 28-31: User sequence number 7 */
#define AFEC_SEQ1R_USCH7_MASK        (15 << AFEC_SEQ1R_USCH7_SHIFT)
#  define AFEC_SEQ1R_USCH7(v)        ((uint32_t)(v) << AFEC_SEQ1R_USCH7_SHIFT)

/* Channel Sequence 2 Register */

#define AFEC_SEQ2R_USCH_SHIFT(n)     (((n)-8) << 2) /* n=8..15 */
#define AFEC_SEQ2R_USCH_MASK(n)      (15 << AFEC_SEQ2R_USCH_SHIFT(n))
#  define AFEC_SEQ2R_USCH(n,v)       ((uint32_t)(v) << AFEC_SEQ2R_USCH_SHIFT(n))
#define AFEC_SEQ2R_USCH8_SHIFT       (0) /* Bits 0-3: User sequence number 8 */
#define AFEC_SEQ2R_USCH8_MASK        (15 << AFEC_SEQ2R_USCH8_SHIFT)
#  define AFEC_SEQ2R_USCH8(v)        ((uint32_t)(v) << AFEC_SEQ2R_USCH8_SHIFT)
#define AFEC_SEQ2R_USCH9_SHIFT       (4) /* Bits 4-7: User sequence number 9 */
#define AFEC_SEQ2R_USCH9_MASK        (15 << AFEC_SEQ2R_USCH9_SHIFT)
#  define AFEC_SEQ2R_USCH9(v)        ((uint32_t)(v) << AFEC_SEQ2R_USCH9_SHIFT)
#define AFEC_SEQ2R_USCH10_SHIFT      (8) /* Bits 8-11: User sequence number 10 */
#define AFEC_SEQ2R_USCH10_MASK       (15 << AFEC_SEQ2R_USCH10_SHIFT)
#  define AFEC_SEQ2R_USCH10(v)       ((uint32_t)(v) << AFEC_SEQ2R_USCH10_SHIFT)
#define AFEC_SEQ2R_USCH11_SHIFT      (12) /* Bits 12-15: User sequence number 11 */
#define AFEC_SEQ2R_USCH11_MASK       (15 << AFEC_SEQ2R_USCH11_SHIFT)
#  define AFEC_SEQ2R_USCH11(v)       ((uint32_t)(v) << AFEC_SEQ2R_USCH11_SHIFT)
#define AFEC_SEQ2R_USCH12_SHIFT      (16) /* Bits 16-19: User sequence number 12 */
#define AFEC_SEQ2R_USCH12_MASK       (15 << AFEC_SEQ2R_USCH12_SHIFT)
#  define AFEC_SEQ2R_USCH12(v)       ((uint32_t)(v) << AFEC_SEQ2R_USCH12_SHIFT)
#define AFEC_SEQ2R_USCH13_SHIFT      (20) /* Bits 20-23: User sequence number 13 */
#define AFEC_SEQ2R_USCH13_MASK       (15 << AFEC_SEQ2R_USCH13_SHIFT)
#  define AFEC_SEQ2R_USCH13(v)       ((uint32_t)(v) << AFEC_SEQ2R_USCH13_SHIFT)
#define AFEC_SEQ2R_USCH14_SHIFT      (24) /* Bits 24-27: User sequence number 14 */
#define AFEC_SEQ2R_USCH14_MASK       (15 << AFEC_SEQ2R_USCH14_SHIFT)
#  define AFEC_SEQ2R_USCH14(v)       ((uint32_t)(v) << AFEC_SEQ2R_USCH14_SHIFT)
#define AFEC_SEQ2R_USCH15_SHIFT      (28) /* Bits 28-31: User sequence number 15 */
#define AFEC_SEQ2R_USCH15_MASK       (15 << AFEC_SEQ2R_USCH15_SHIFT)
#  define AFEC_SEQ2R_USCH15(v)       ((uint32_t)(v) << AFEC_SEQ2R_USCH15_SHIFT)

/* Channel Enable, Channel Disable, and Channel Status Registers */

#define AFEC_CH(n)                   (1 << (n))
#  define AFEC_CH0                   (1 << 0)  /* Bit 0:  Channel 0 Enable */
#  define AFEC_CH1                   (1 << 1)  /* Bit 1:  Channel 1 Enable */
#  define AFEC_CH2                   (1 << 2)  /* Bit 2:  Channel 2 Enable */
#  define AFEC_CH3                   (1 << 3)  /* Bit 3:  Channel 3 Enable */
#  define AFEC_CH4                   (1 << 4)  /* Bit 4:  Channel 4 Enable */
#  define AFEC_CH5                   (1 << 5)  /* Bit 5:  Channel 5 Enable */
#  define AFEC_CH6                   (1 << 6)  /* Bit 6:  Channel 6 Enable */
#  define AFEC_CH7                   (1 << 7)  /* Bit 7:  Channel 7 Enable */
#  define AFEC_CH8                   (1 << 8)  /* Bit 8:  Channel 8 Enable */
#  define AFEC_CH9                   (1 << 9)  /* Bit 9:  Channel 9 Enable */
#  define AFEC_CH10                  (1 << 10) /* Bit 10: Channel 10 Enable */
#  define AFEC_CH11                  (1 << 11) /* Bit 11: Channel 11 Enable */
#  define AFEC_CHALL                 (0x00000fff)

/* Last Converted Data Register */

#define AFEC_LCDR_LDATA_SHIFT        (0)       /* Bits 0-15: Last Data Converted */
#define AFEC_LCDR_LDATA_MASK         (0xffff << AFEC_LCDR_LDATA_SHIFT)
#define AFEC_LCDR_CHANB_SHIFT        (24)      /* Bits 24-27: Channel number */
#define AFEC_LCDR_CHANB_MASK         (15 << AFEC_LCDR_CHANB_SHIFT)

/* Interrupt Enable, Interrupt Disable, Interrupt Mask, and Interrupt Status Registers */

#define AFEC_INT_EOC(n)              (1 << (n))
#  define AFEC_INT_EOC0              (1 << 0)  /* Bit 0:  End of Conversion 0 */
#  define AFEC_INT_EOC1              (1 << 1)  /* Bit 1:  End of Conversion 1 */
#  define AFEC_INT_EOC2              (1 << 2)  /* Bit 2:  End of Conversion 2 */
#  define AFEC_INT_EOC3              (1 << 3)  /* Bit 3:  End of Conversion 3 */
#  define AFEC_INT_EOC4              (1 << 4)  /* Bit 4:  End of Conversion 4 */
#  define AFEC_INT_EOC5              (1 << 5)  /* Bit 5:  End of Conversion 5 */
#  define AFEC_INT_EOC6              (1 << 6)  /* Bit 6:  End of Conversion 6 */
#  define AFEC_INT_EOC7              (1 << 7)  /* Bit 7:  End of Conversion 7 */
#  define AFEC_INT_EOC8              (1 << 8)  /* Bit 8:  End of Conversion 8 */
#  define AFEC_INT_EOC9              (1 << 9)  /* Bit 9:  End of Conversion 9 */
#  define AFEC_INT_EOC10             (1 << 10) /* Bit 10: End of Conversion 10 */
#  define AFEC_INT_EOC11             (1 << 11) /* Bit 11: End of Conversion 11 */
#  define AFEC_INT_EOCALL            (0x00000fff)

#define AFEC_INT_DRDY                (1 << 24) /* Bit 24: Data Ready Interrupt */
#define AFEC_INT_GOVRE               (1 << 25) /* Bit 25: General Overrun Error */
#define AFEC_INT_COMPE               (1 << 26) /* Bit 26: Comparison Event Interrupt */
#define AFEC_INT_TEMPCHG             (1 << 30) /* Bit 30: Temperature Change Interrupt */

#define AFEC_INT_ALL                 (0x47000fff)

/* Overrun Status Register */

#define AFEC_OVER_OVRE(n)            (1 << (n))
#  define AFEC_OVER_OVRE0            (1 << 0)  /* Bit 0:  Overrun Error 0 */
#  define AFEC_OVER_OVRE1            (1 << 1)  /* Bit 1:  Overrun Error 1 */
#  define AFEC_OVER_OVRE2            (1 << 2)  /* Bit 2:  Overrun Error 2 */
#  define AFEC_OVER_OVRE3            (1 << 3)  /* Bit 3:  Overrun Error 3 */
#  define AFEC_OVER_OVRE4            (1 << 4)  /* Bit 4:  Overrun Error 4 */
#  define AFEC_OVER_OVRE5            (1 << 5)  /* Bit 5:  Overrun Error 5 */
#  define AFEC_OVER_OVRE6            (1 << 6)  /* Bit 6:  Overrun Error 6 */
#  define AFEC_OVER_OVRE7            (1 << 7)  /* Bit 7:  Overrun Error 7 */
#  define AFEC_OVER_OVRE8            (1 << 8)  /* Bit 8:  Overrun Error 8 */
#  define AFEC_OVER_OVRE9            (1 << 9)  /* Bit 9:  Overrun Error 9 */
#  define AFEC_OVER_OVRE10           (1 << 10) /* Bit 10: Overrun Error 10 */
#  define AFEC_OVER_OVRE11           (1 << 11) /* Bit 11: Overrun Error 11 */

/* Compare Window Register */

#define AFEC_CWR_LOWTHRES_SHIFT      (0)       /* Bits 0-11: Low Threshold */
#define AFEC_CWR_LOWTHRES_MASK       (0xfff << AFEC_CWR_LOWTHRES_SHIFT)
#  define AFEC_CWR_LOWTHRES(n)       ((uint32_t)(n) << AFEC_CWR_LOWTHRES_SHIFT)
#define AFEC_CWR_HIGHTHRES_SHIFT     (16)      /* Bits 16-27: High Threshold */
#define AFEC_CWR_HIGHTHRES_MASK      (0xfff << AFEC_CWR_LOWTHRES_SHIFT)
#  define AFEC_CWR_HIGHTHRES(n)K     ((uint32_t)(n) << AFEC_CWR_LOWTHRES_SHIFT)

/* Channel Gain Register */

#define AFEC_CGR_GAIN_SHIFT(n)       ((n) << 1) /* n=0..15 */
#define AFEC_CGR_GAIN_MASK(n)        (3 << AFEC_CGR_GAIN_SHIFT(n))
#  define AFEC_CGR_GAIN(n,v)         ((uint32_t)(v) << AFEC_CGR_GAIN_SHIFT(n))
#define AFEC_CGR_GAIN0_SHIFT         (0)        /* Bits 0-1: Gain for channel 0 */
#define AFEC_CGR_GAIN0_MASK          (3 << AFEC_CGR_GAIN0_SHIFT)
#  define AFEC_CGR_GAIN0(v)          ((uint32_t)(v) << AFEC_CGR_GAIN0_SHIFT)
#define AFEC_CGR_GAIN1_SHIFT         (2)        /* Bits 2-3: Gain for channel 1 */
#define AFEC_CGR_GAIN1_MASK          (3 << AFEC_CGR_GAIN1_SHIFT)
#  define AFEC_CGR_GAIN1(v)          ((uint32_t)(v) << AFEC_CGR_GAIN1_SHIFT)
#define AFEC_CGR_GAIN2_SHIFT         (4)        /* Bits 4-5: Gain for channel 2 */
#define AFEC_CGR_GAIN2_MASK          (3 << AFEC_CGR_GAIN2_SHIFT)
#  define AFEC_CGR_GAIN2(v)          ((uint32_t)(v) << AFEC_CGR_GAIN2_SHIFT)
#define AFEC_CGR_GAIN3_SHIFT         (6)        /* Bits 6-7: Gain for channel 3 */
#define AFEC_CGR_GAIN3_MASK          (3 << AFEC_CGR_GAIN3_SHIFT)
#  define AFEC_CGR_GAIN3(v)          ((uint32_t)(v) << AFEC_CGR_GAIN3_SHIFT)
#define AFEC_CGR_GAIN4_SHIFT         (8)        /* Bits 8-9: Gain for channel 4 */
#define AFEC_CGR_GAIN4_MASK          (3 << AFEC_CGR_GAIN4_SHIFT)
#  define AFEC_CGR_GAIN4(v)          ((uint32_t)(v) << AFEC_CGR_GAIN4_SHIFT)
#define AFEC_CGR_GAIN5_SHIFT         (10)       /* Bits 10-11: Gain for channel 5 */
#define AFEC_CGR_GAIN5_MASK          (3 << AFEC_CGR_GAIN5_SHIFT)
#  define AFEC_CGR_GAIN5(v)          ((uint32_t)(v) << AFEC_CGR_GAIN5_SHIFT)
#define AFEC_CGR_GAIN6_SHIFT         (12)       /* Bits 12-13: Gain for channel 6 */
#define AFEC_CGR_GAIN6_MASK          (3 << AFEC_CGR_GAIN6_SHIFT)
#  define AFEC_CGR_GAIN6(v)          ((uint32_t)(v) << AFEC_CGR_GAIN6_SHIFT)
#define AFEC_CGR_GAIN7_SHIFT         (14)       /* Bits 14-15: Gain for channel 7 */
#define AFEC_CGR_GAIN7_MASK          (3 << AFEC_CGR_GAIN7_SHIFT)
#  define AFEC_CGR_GAIN7(v)          ((uint32_t)(v) << AFEC_CGR_GAIN7_SHIFT)
#define AFEC_CGR_GAIN8_SHIFT         (16)       /* Bits 16-17: Gain for channel 8 */
#define AFEC_CGR_GAIN8_MASK          (3 << AFEC_CGR_GAIN8_SHIFT)
#  define AFEC_CGR_GAIN8(v)          ((uint32_t)(v) << AFEC_CGR_GAIN8_SHIFT)
#define AFEC_CGR_GAIN9_SHIFT         (18)       /* Bits 18-19: Gain for channel 9 */
#define AFEC_CGR_GAIN9_MASK          (3 << AFEC_CGR_GAIN9_SHIFT)
#  define AFEC_CGR_GAIN9(v)          ((uint32_t)(v) << AFEC_CGR_GAIN9_SHIFT)
#define AFEC_CGR_GAIN10_SHIFT        (20)       /* Bits 20-21: Gain for channel 10 */
#define AFEC_CGR_GAIN10_MASK         (3 << AFEC_CGR_GAIN10_SHIFT)
#  define AFEC_CGR_GAIN10(v)         ((uint32_t)(v) << AFEC_CGR_GAIN10_SHIFT)
#define AFEC_CGR_GAIN11_SHIFT        (22)       /* Bits 22-23: Gain for channel 11 */
#define AFEC_CGR_GAIN11_MASK         (3 << AFEC_CGR_GAIN11_SHIFT)
#  define AFEC_CGR_GAIN11(v)         ((uint32_t)(v) << AFEC_CGR_GAIN11_SHIFT)

/* Channel Calibration DC Offset Register (Used in Automatic Calibration Procedure) */

#define AFEC_CDOR_OFF(n)             (1 << (n))
#  define AFEC_CDOR_OFF0             (1 << 0)  /* Bit 0:  Offset for channel 0 */
#  define AFEC_CDOR_OFF1             (1 << 1)  /* Bit 1:  Offset for channel 1 */
#  define AFEC_CDOR_OFF2             (1 << 2)  /* Bit 2:  Offset for channel 2 */
#  define AFEC_CDOR_OFF3             (1 << 3)  /* Bit 3:  Offset for channel 3 */
#  define AFEC_CDOR_OFF4             (1 << 4)  /* Bit 4:  Offset for channel 4 */
#  define AFEC_CDOR_OFF5             (1 << 5)  /* Bit 5:  Offset for channel 5 */
#  define AFEC_CDOR_OFF6             (1 << 6)  /* Bit 6:  Offset for channel 6 */
#  define AFEC_CDOR_OFF7             (1 << 7)  /* Bit 7:  Offset for channel 7 */
#  define AFEC_CDOR_OFF8             (1 << 8)  /* Bit 8:  Offset for channel 8 */
#  define AFEC_CDOR_OFF9             (1 << 9)  /* Bit 9:  Offset for channel 9 */
#  define AFEC_CDOR_OFF10            (1 << 10) /* Bit 10: Offset for channel 10 */
#  define AFEC_CDOR_OFF11            (1 << 11) /* Bit 11: Offset for channel 11 */

/* Channel Differential Register */

#define AFEC_DIFFR_DIFF(n)           (1 << (n))
#  define AFEC_DIFFR_DIFF0           (1 << 0)  /* Bit 0:  Differential inputs for channel 0 */
#  define AFEC_DIFFR_DIFF1           (1 << 1)  /* Bit 1:  Differential inputs for channel 1 */
#  define AFEC_DIFFR_DIFF2           (1 << 2)  /* Bit 2:  Differential inputs for channel 2 */
#  define AFEC_DIFFR_DIFF3           (1 << 3)  /* Bit 3:  Differential inputs for channel 3 */
#  define AFEC_DIFFR_DIFF4           (1 << 4)  /* Bit 4:  Differential inputs for channel 4 */
#  define AFEC_DIFFR_DIFF5           (1 << 5)  /* Bit 5:  Differential inputs for channel 5 */
#  define AFEC_DIFFR_DIFF6           (1 << 6)  /* Bit 6:  Differential inputs for channel 6 */
#  define AFEC_DIFFR_DIFF7           (1 << 7)  /* Bit 7:  Differential inputs for channel 7 */
#  define AFEC_DIFFR_DIFF8           (1 << 8)  /* Bit 8:  Differential inputs for channel 8 */
#  define AFEC_DIFFR_DIFF9           (1 << 9)  /* Bit 9:  Differential inputs for channel 9 */
#  define AFEC_DIFFR_DIFF10          (1 << 10) /* Bit 10: Differential inputs for channel 10 */
#  define AFEC_DIFFR_DIFF11          (1 << 11) /* Bit 11: Differential inputs for channel 11 */

/* Channel Selection Register */

#define AFEC_CSELR_CSEL_SHIFT        (0)       /* Bits 0-3: Channel Selection */
#define AFEC_CSELR_CSEL_MASK         (15 << AFEC_CSELR_CSEL_SHIFT)
#  define AFEC_CSELR_CSEL(n)         ((uint32_t)(n) << AFEC_CSELR_CSEL_SHIFT)

/* Channel Data Register */

#define AFEC_CDR_MASK                (0x0000ffff) /* Bits 0-15: Converted Data */

/* Channel Offset Compensation Register */

#define AFEC_COCR_MASK               (0x00000fff) /* Bits 0-12: Analog Offset */

/* Temperature Sensor Mode Register */

#define AFEC_TEMPMR_RTCT              (1 << 0)  /* Bit 0:  Temperature Sensor RTC Trigger mode */
#define AFEC_TEMPMR_TEMPCMPMOD_SHIFT  (4)       /* Bits 4-5: Temperature Comparison Mode */
#define AFEC_TEMPMR_TEMPCMPMOD_MASK   (3 << AFEC_TEMPMR_TEMPCMPMOD_SHIFT)
#  define AFEC_TEMPMR_TEMPCMPMOD_LOW  (0 << AFEC_TEMPMR_TEMPCMPMOD_SHIFT) /* Event when data is lower than low threshold */
#  define AFEC_TEMPMR_TEMPCMPMOD_HIGH (1 << AFEC_TEMPMR_TEMPCMPMOD_SHIFT) /* Event when data is higher than high threshold */
#  define AFEC_TEMPMR_TEMPCMPMOD_IN   (2 << AFEC_TEMPMR_TEMPCMPMOD_SHIFT) /* Event when data is in the comparison window */
#  define AFEC_TEMPMR_TEMPCMPMOD_OUT  (3 << AFEC_TEMPMR_TEMPCMPMOD_SHIFT) /* Event when data is out of the comparison window */

/* Temperature Compare Window Register */

#define AFEC_TEMPCWR_TLOWTHRES_SHIFT  (0)      /* Bits 0-15: Temperature Low Threshold */
#define AFEC_TEMPCWR_TLOWTHRES_MASK   (0xffff << AFEC_TEMPCWR_TLOWTHRES_SHIFT)
#  define AFEC_TEMPCWR_TLOWTHRES(n)   (0xffff << AFEC_TEMPCWR_TLOWTHRES_SHIFT)
#define AFEC_TEMPCWR_THIGHTHRES_SHIFT (16)     /* Bits 16-31: Temperature High Threshold */
#define AFEC_TEMPCWR_THIGHTHRES_MASK  (0xffff << AFEC_TEMPCWR_THIGHTHRES_SHIFT)
#  define AFEC_TEMPCWR_THIGHTHRES(n)  ((uint32_t)(n) << AFEC_TEMPCWR_THIGHTHRES_SHIFT)

/* Analog Control Register */

#define AFEC_ACR_PGA0EN              (1 << 2)  /* Bit 2: PGA0 Enable */
#define AFEC_ACR_PGA1EN              (1 << 3)  /* Bit 3: PGA1 Enable */
#define AFEC_ACR_IBCTL_SHIFT         (9)       /* Bits 8-9: AFEC Bias Current Control */
#define AFEC_ACR_IBCTL_MASK          (3 << AFEC_ACR_IBCTL_SHIFT)
#  define AFEC_ACR_IBCTL(n)          ((uint32_t)(n) << AFEC_ACR_IBCTL_SHIFT)

/* Sample & Hold Mode Register */

#define AFEC_SHMR_DUAL(n)            (1 << (n))
#  define AFEC_SHMR_DUAL0            (1 << 0)  /* Bit 0:  Dual Sample & Hold for channel 0 */
#  define AFEC_SHMR_DUAL1            (1 << 1)  /* Bit 1:  Dual Sample & Hold for channel 1 */
#  define AFEC_SHMR_DUAL2            (1 << 2)  /* Bit 2:  Dual Sample & Hold for channel 2 */
#  define AFEC_SHMR_DUAL3            (1 << 3)  /* Bit 3:  Dual Sample & Hold for channel 3 */
#  define AFEC_SHMR_DUAL4            (1 << 4)  /* Bit 4:  Dual Sample & Hold for channel 4 */
#  define AFEC_SHMR_DUAL5            (1 << 5)  /* Bit 5:  Dual Sample & Hold for channel 5 */
#  define AFEC_SHMR_DUAL6            (1 << 6)  /* Bit 6:  Dual Sample & Hold for channel 6 */
#  define AFEC_SHMR_DUAL7            (1 << 7)  /* Bit 7:  Dual Sample & Hold for channel 7 */
#  define AFEC_SHMR_DUAL8            (1 << 8)  /* Bit 8:  Dual Sample & Hold for channel 8 */
#  define AFEC_SHMR_DUAL9            (1 << 9)  /* Bit 9:  Dual Sample & Hold for channel 9 */
#  define AFEC_SHMR_DUAL10           (1 << 10) /* Bit 10: Dual Sample & Hold for channel 10 */
#  define AFEC_SHMR_DUAL11           (1 << 11) /* Bit 11: Dual Sample & Hold for channel 11 */

/* Correction Select Register */

#define AFEC_COSR_CSEL               (1 << 0)  /* Bit 0: Sample & Hold unit Correction Select */

/* Correction Values Register */

#define AFEC_CVR_OFFSETCORR_SHIFT    (0)       /* Bits 0-16: Offset Correction */
#define AFEC_CVR_OFFSETCORR_MASK     (0xffff << AFEC_CVR_OFFSETCORR_SHIFT)
#  define AFEC_CVR_OFFSETCORR(n)     ((uint32_t)(n) << AFEC_CVR_OFFSETCORR_SHIFT)
#define AFEC_CVR_GAINCORR_SHIFT      (16)      /* Bits 16-31: Gain Correction */
#define AFEC_CVR_GAINCORR_MASK       (0xffff << AFEC_CVR_GAINCORR_SHIFT)
#  define AFEC_CVR_GAINCORR(n)       ((uint32_t)(n) << AFEC_CVR_GAINCORR_SHIFT)

/* Channel Error Correction Register */

#define AFEC_CECR_ECORR(n)           (1 << (n))
#  define AFEC_CECR_ECORR0           (1 << 0)  /* Bit 0:  Error Correction Enable for channel 0 */
#  define AFEC_CECR_ECORR1           (1 << 1)  /* Bit 1:  Error Correction Enable for channel 1 */
#  define AFEC_CECR_ECORR2           (1 << 2)  /* Bit 2:  Error Correction Enable for channel 2 */
#  define AFEC_CECR_ECORR3           (1 << 3)  /* Bit 3:  Error Correction Enable for channel 3 */
#  define AFEC_CECR_ECORR4           (1 << 4)  /* Bit 4:  Error Correction Enable for channel 4 */
#  define AFEC_CECR_ECORR5           (1 << 5)  /* Bit 5:  Error Correction Enable for channel 5 */
#  define AFEC_CECR_ECORR6           (1 << 6)  /* Bit 6:  Error Correction Enable for channel 6 */
#  define AFEC_CECR_ECORR7           (1 << 7)  /* Bit 7:  Error Correction Enable for channel 7 */
#  define AFEC_CECR_ECORR8           (1 << 8)  /* Bit 8:  Error Correction Enable for channel 8 */
#  define AFEC_CECR_ECORR9           (1 << 9)  /* Bit 9:  Error Correction Enable for channel 9 */
#  define AFEC_CECR_ECORR10          (1 << 10) /* Bit 10: Error Correction Enable for channel 10 */
#  define AFEC_CECR_ECORR11          (1 << 11) /* Bit 11: Error Correction Enable for channel 11 */

/* Write Protect Mode Register */

#define AFEC_WPMR_WPEN               (1 << 0)  /* Bit 0:  Write Protect Enable */
#define AFEC_WPMR_WPKEY_SHIFT        (8)       /* Bits 8-31: Write Protect KEY */
#define AFEC_WPMR_WPKEY_MASK         (0x00ffffff << AFEC_WPMR_WPKEY_SHIFT)
#  define AFEC_WPMR_WPKEY            (0x00414443 << AFEC_WPMR_WPKEY_SHIFT)

/* Write Protect Status Register */

#define AFEC_WPSR_WPVS               (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define AFEC_WPSR_WPVSRC_SHIFT       (8)       /* Bits 8-23: Write Protect Violation Source */
#define AFEC_WPSR_WPVSRC_MASK        (0x0000ffff << AFEC_WPSR_WPVSRC_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_CHIP_SAM_AFEC_H */
