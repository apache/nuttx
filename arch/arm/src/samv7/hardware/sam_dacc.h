/****************************************************************************************
 * arch/arm/src/samv7/hardware/sam_dacc.h
 * Digital-to-Analog Converter Controller (DACC) for the SAMV7
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

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_DACC_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_DACC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* DACC register offsets *****************************************************************/

#define SAM_DACC_CR_OFFSET         0x0000 /* Control Register */
#define SAM_DACC_MR_OFFSET         0x0004 /* Mode Register */
#define SAM_DACC_TRIGR_OFFSET      0x0008 /* Trigger Register */
#define SAM_DACC_CHER_OFFSET       0x0010 /* Channel Enable Register */
#define SAM_DACC_CHDR_OFFSET       0x0014 /* Channel Disable Register */
#define SAM_DACC_CHSR_OFFSET       0x0018 /* Channel Status Register */
#define SAM_DACC_CDR0_OFFSET       0x001c /* Conversion Data Register 0 */
#define SAM_DACC_CDR1_OFFSET       0x0020 /* Conversion Data Register 1 */
#define SAM_DACC_IER_OFFSET        0x0024 /* Interrupt Enable Register */
#define SAM_DACC_IDR_OFFSET        0x0028 /* Interrupt Disable Register */
#define SAM_DACC_IMR_OFFSET        0x002c /* Interrupt Mask Register */
#define SAM_DACC_ISR_OFFSET        0x0030 /* Interrupt Status Register */
#define SAM_DACC_ACR_OFFSET        0x0094 /* Analog Current Register */
#define SAM_DACC_WPMR_OFFSET       0x00e4 /* Write Protect Mode register */
#define SAM_DACC_WPSR_OFFSET       0x00e8 /* Write Protect Status register */

/* DACC register addresses **************************************************************/

#define SAM_DACC_CR                (SAM_DACC_BASE+SAM_DACC_CR_OFFSET)
#define SAM_DACC_MR                (SAM_DACC_BASE+SAM_DACC_MR_OFFSET)
#define SAM_DACC_TRIGR             (SAM_DACC_BASE+SAM_DACC_TRIGR_OFFSET)
#define SAM_DACC_CHER              (SAM_DACC_BASE+SAM_DACC_CHER_OFFSET)
#define SAM_DACC_CHDR              (SAM_DACC_BASE+SAM_DACC_CHDR_OFFSET)
#define SAM_DACC_CHSR              (SAM_DACC_BASE+SAM_DACC_CHSR_OFFSET)
#define SAM_DACC_CDR0              (SAM_DACC_BASE+SAM_DACC_CDR0_OFFSET)
#define SAM_DACC_CDR1              (SAM_DACC_BASE+SAM_DACC_CDR1_OFFSET)
#define SAM_DACC_IER               (SAM_DACC_BASE+SAM_DACC_IER_OFFSET)
#define SAM_DACC_IDR               (SAM_DACC_BASE+SAM_DACC_IDR_OFFSET)
#define SAM_DACC_IMR               (SAM_DACC_BASE+SAM_DACC_IMR_OFFSET)
#define SAM_DACC_ISR               (SAM_DACC_BASE+SAM_DACC_ISR_OFFSET)
#define SAM_DACC_ACR               (SAM_DACC_BASE+SAM_DACC_ACR_OFFSET)
#define SAM_DACC_WPMR              (SAM_DACC_BASE+SAM_DACC_WPMR_OFFSET)
#define SAM_DACC_WPSR              (SAM_DACC_BASE+SAM_DACC_WPSR_OFFSET)

/* DACC register bit definitions ********************************************************/

/* Control Register */

#define DACC_CR_SWRST               (1 << 0)  /* Bit 0:  Software reset */

/* Mode Register */

#define DACC_MR_MAXS0               (1 << 0) /* Max Speed Mode for Channel 0 */
#  define DACC_MR_MAXS0_TRIG_EVENT  (0 << 0) /* External trigger mode or Free-running mode enabled */
#  define DACC_MR_MAXS0_MAXIMUM     (1 << 0) /* Max speed mode enabled */
#define DACC_MR_MAXS1               (1 << 1) /* Max Speed Mode for Channel 1 */
#  define DACC_MR_MAXS1_TRIG_EVENT  (0 << 1) /* External trigger mode or Free-running mode enabled */
#  define DACC_MR_MAXS1_MAXIMUM     (1 << 1) /* Max speed mode enabled */
#define DACC_MR_WORD                (1 << 4) /* Word Transfer Mode */
#  define DACC_MR_WORD_DISABLED     (0 << 4) /* One data to convert is written to the FIFO per access to DACC */
#  define DACC_MR_WORD_ENABLED      (1 << 4) /* Two data to convert are written to the FIFO per access to DACC */
#define DACC_MR_ZERO                (1 << 5) /* Must always be written to 0 */
#define DACC_MR_DIFF                (1 << 23) /* Differential Mode */
#  define DACC_MR_DIFF_DISABLED     (0 << 23) /* DAC0 and DAC1 are single-ended outputs */
#  define DACC_MR_DIFF_ENABLED      (1 << 23) /* DACP and DACN are differential outputs. The differential level is configured by the channel 0 value. */
#define DACC_MR_PRESCALER_SHIFT     (24)
#define DACC_MR_PRESCALER_MASK      (0xfu << DACC_MR_PRESCALER_SHIFT) /* Peripheral Clock to DAC Clock Ratio */
#define DACC_MR_PRESCALER(value)    ((DACC_MR_PRESCALER_MASK & ((value) << DACC_MR_PRESCALER_SHIFT)))
#  define DACC_MR_PRESCALER_2       (0 << DACC_MR_PRESCALER_SHIFT)  /* 2 periods of DAC Clock */
#  define DACC_MR_PRESCALER_3       (1 << DACC_MR_PRESCALER_SHIFT)  /* 3 periods of DAC Clock */
#  define DACC_MR_PRESCALER_4       (2 << DACC_MR_PRESCALER_SHIFT)  /* 4 periods of DAC Clock */
#  define DACC_MR_PRESCALER_5       (3 << DACC_MR_PRESCALER_SHIFT)  /* 5 periods of DAC Clock */
#  define DACC_MR_PRESCALER_6       (4 << DACC_MR_PRESCALER_SHIFT)  /* 6 periods of DAC Clock */
#  define DACC_MR_PRESCALER_7       (5 << DACC_MR_PRESCALER_SHIFT)  /* 7 periods of DAC Clock */
#  define DACC_MR_PRESCALER_8       (6 << DACC_MR_PRESCALER_SHIFT)  /* 8 periods of DAC Clock */
#  define DACC_MR_PRESCALER_9       (7 << DACC_MR_PRESCALER_SHIFT)  /* 9 periods of DAC Clock */
#  define DACC_MR_PRESCALER_10      (8 << DACC_MR_PRESCALER_SHIFT)  /* 10 periods of DAC Clock */
#  define DACC_MR_PRESCALER_11      (9 << DACC_MR_PRESCALER_SHIFT)  /* 11 periods of DAC Clock */
#  define DACC_MR_PRESCALER_12      (10 << DACC_MR_PRESCALER_SHIFT) /* 12 periods of DAC Clock */
#  define DACC_MR_PRESCALER_13      (11 << DACC_MR_PRESCALER_SHIFT) /* 13 periods of DAC Clock */
#  define DACC_MR_PRESCALER_14      (12 << DACC_MR_PRESCALER_SHIFT) /* 14 periods of DAC Clock */
#  define DACC_MR_PRESCALER_15      (13 << DACC_MR_PRESCALER_SHIFT) /* 15 periods of DAC Clock */
#  define DACC_MR_PRESCALER_16      (14 << DACC_MR_PRESCALER_SHIFT) /* 16 periods of DAC Clock */
#  define DACC_MR_PRESCALER_17      (15 << DACC_MR_PRESCALER_SHIFT) /* 17 periods of DAC Clock */

/* Trigger Register */

#define DACC_TRIGR_TRGEN0           (1 << 0) /* Trigger Enable of Channel 0 */
#  define DACC_TRIGR_TRGEN0_DIS     (0 << 0) /* External trigger mode disabled. DACC is in Free-running mode or Max speed mode. */
#  define DACC_TRIGR_TRGEN0_EN      (1 << 0) /* External trigger mode enabled. */
#define DACC_TRIGR_TRGEN1           (1 << 1) /* Trigger Enable of Channel 1 */
#  define DACC_TRIGR_TRGEN1_DIS     (0 << 1) /* External trigger mode disabled. DACC is in Free-running mode or Max speed mode. */
#  define DACC_TRIGR_TRGEN1_EN      (1 << 1) /* External trigger mode enabled. */
#define DACC_TRIGR_TRGSEL0_SHIFT    (4)
#define DACC_TRIGR_TRGSEL0_MASK     (0x7u << DACC_TRIGR_TRGSEL0_SHIFT) /* Trigger Selection of Channel 0 */
#define DACC_TRIGR_TRGSEL0(value)   ((DACC_TRIGR_TRGSEL0_MASK & ((value) << DACC_TRIGR_TRGSEL0_SHIFT)))
#  define DACC_TRIGR_TRGSEL0_DATRG   (0 << 4) /* DATRG output */
#  define DACC_TRIGR_TRGSEL0_TC0     (1 << 4) /* TC0 output */
#  define DACC_TRIGR_TRGSEL0_TC1     (2 << 4) /* TC1 output */
#  define DACC_TRIGR_TRGSEL0_TC2     (3 << 4) /* TC2 output */
#  define DACC_TRIGR_TRGSEL0_PWM0EV0 (4 << 4) /* PWM0 event 0 */
#  define DACC_TRIGR_TRGSEL0_PWM0EV1 (5 << 4) /* PWM0 event 1 */
#  define DACC_TRIGR_TRGSEL0_PWM1EV0 (6 << 4) /* PWM1 event 0 */
#  define DACC_TRIGR_TRGSEL0_PWM1EV1 (7 << 4) /* PWM1 event 1 */
#define DACC_TRIGR_TRGSEL1_SHIFT    (8)
#define DACC_TRIGR_TRGSEL1_MASK     (0x7u << DACC_TRIGR_TRGSEL1_SHIFT) /* Trigger Selection of Channel 1 */
#define DACC_TRIGR_TRGSEL1(value)   ((DACC_TRIGR_TRGSEL1_MASK & ((value) << DACC_TRIGR_TRGSEL1_SHIFT)))
#  define DACC_TRIGR_TRGSEL1_DATRG   (0 << 8) /* DATRG output */
#  define DACC_TRIGR_TRGSEL1_TC0     (1 << 8) /* TC0 output */
#  define DACC_TRIGR_TRGSEL1_TC1     (2 << 8) /* TC1 output */
#  define DACC_TRIGR_TRGSEL1_TC2     (3 << 8) /* TC2 output */
#  define DACC_TRIGR_TRGSEL1_PWM0EV0 (4 << 8) /* PWM0 event 0 */
#  define DACC_TRIGR_TRGSEL1_PWM0EV1 (5 << 8) /* PWM0 event 1 */
#  define DACC_TRIGR_TRGSEL1_PWM1EV0 (6 << 8) /* PWM1 event 0 */
#  define DACC_TRIGR_TRGSEL1_PWM1EV1 (7 << 8) /* PWM1 event 1 */
#define DACC_TRIGR_OSR0_SHIFT       (16)
#define DACC_TRIGR_OSR0_MASK        (0x7u << DACC_TRIGR_OSR0_SHIFT) /* Over Sampling Ratio of Channel 0 */
#define DACC_TRIGR_OSR0(value)      ((DACC_TRIGR_OSR0_MASK & ((value) << DACC_TRIGR_OSR0_SHIFT)))
#  define DACC_TRIGR_OSR0_OSR_1     (0 << 16) /* OSR = 1 */
#  define DACC_TRIGR_OSR0_OSR_2     (1 << 16) /* OSR = 2 */
#  define DACC_TRIGR_OSR0_OSR_4     (2 << 16) /* OSR = 4 */
#  define DACC_TRIGR_OSR0_OSR_8     (3 << 16) /* OSR = 8 */
#  define DACC_TRIGR_OSR0_OSR_16    (4 << 16) /* OSR = 16 */
#  define DACC_TRIGR_OSR0_OSR_32    (5 << 16) /* OSR = 32 */
#define DACC_TRIGR_OSR1_SHIFT       (20)
#define DACC_TRIGR_OSR1_MASK        (0x7u << DACC_TRIGR_OSR1_SHIFT) /* Over Sampling Ratio of Channel 1 */
#define DACC_TRIGR_OSR1(value)      ((DACC_TRIGR_OSR1_MASK & ((value) << DACC_TRIGR_OSR1_SHIFT)))
#  define DACC_TRIGR_OSR1_OSR_1     (0 << 20) /* OSR = 1 */
#  define DACC_TRIGR_OSR1_OSR_2     (1 << 20) /* OSR = 2 */
#  define DACC_TRIGR_OSR1_OSR_4     (2 << 20) /* OSR = 4 */
#  define DACC_TRIGR_OSR1_OSR_8     (3 << 20) /* OSR = 8 */
#  define DACC_TRIGR_OSR1_OSR_16    (4 << 20) /* OSR = 16 */
#  define DACC_TRIGR_OSR1_OSR_32    (5 << 20) /* OSR = 32 */

/* Channel Enable, Channel Disable, and  Channel Status Registers */

#define DACC_CH0                    (1 << 0)  /* Channel 0 */
#define DACC_CH1                    (1 << 1)  /* Channel 1 */
#define DACC_CHSR_DACRDY0           (1 << 8)  /* DAC Ready Flag */
#define DACC_CHSR_DACRDY1           (1 << 9)  /* DAC Ready Flag */

/* Conversion Data Register -- 32-bit data */

#define DACC_CDR_DATA0_SHIFT        (0)
#define DACC_CDR_DATA0_MASK         (0xffffu << DACC_CDR_DATA0_SHIFT) /* Data to Convert for channel 0 */
#define DACC_CDR_DATA0(value)       ((DACC_CDR_DATA0_MASK & ((value) << DACC_CDR_DATA0_SHIFT)))
#define DACC_CDR_DATA1_SHIFT        (16)
#define DACC_CDR_DATA1_MASK         (0xffffu << DACC_CDR_DATA1_SHIFT) /* Data to Convert for channel 1 */
#define DACC_CDR_DATA1(value)       ((DACC_CDR_DATA1_MASK & ((value) << DACC_CDR_DATA1_SHIFT)))

/* Interrupt Enable, Interrupt Disable, Interrupt Mask, and Interrupt Status Register */

#define DACC_INT_TXRDY0             (1 << 0)  /* Transmit Ready Interrupt of channel 0 */
#define DACC_INT_TXRDY1             (1 << 1)  /* Transmit Ready Interrupt of channel 1 */
#define DACC_INT_EOC0               (1 << 4)  /* End of Conversion Interrupt of channel 0 */
#define DACC_INT_EOC1               (1 << 5)  /* End of Conversion Interrupt of channel 1 */
#define DACC_INT_ALL                (0xffffffffu)  /* All interrupts */

/* Analog Current Register */

#define DACC_ACR_IBCTLCH0_SHIFT     (0)
#define DACC_ACR_IBCTLCH0_MASK      (0x3u << DACC_ACR_IBCTLCH0_SHIFT) /* Analog Output Current Control */
#define DACC_ACR_IBCTLCH0(value)    ((DACC_ACR_IBCTLCH0_MASK & ((value) << DACC_ACR_IBCTLCH0_SHIFT)))
#define DACC_ACR_IBCTLCH1_SHIFT     (2)
#define DACC_ACR_IBCTLCH1_MASK      (0x3u << DACC_ACR_IBCTLCH1_SHIFT) /* Analog Output Current Control */
#define DACC_ACR_IBCTLCH1(value)    ((DACC_ACR_IBCTLCH1_MASK & ((value) << DACC_ACR_IBCTLCH1_SHIFT)))

/* Write Protect Mode register */

#define DACC_WPMR_WPEN              (1 << 0)                     /* Write Protection Enable */
#define DACC_WPMR_WPKEY_SHIFT       (8)
#define DACC_WPMR_WPKEY_MASK        (0xffffffu << DACC_WPMR_WPKEY_SHIFT)  /* Write Protect Key */
#define DACC_WPMR_WPKEY(value)      ((DACC_WPMR_WPKEY_MASK & ((value) << DACC_WPMR_WPKEY_SHIFT)))
#  define DACC_WPMR_WPKEY_PASSWD    (0x444143u << 8)                /* Writing any other value in this field aborts the write operation of bit WPEN. Always reads as 0. */

/* Write Protect Status register */

#define DACC_WPSR_WPVS              (1 << 0)                     /* Write Protection Violation Status */
#define DACC_WPSR_WPVSRC_SHIFT      (8)
#define DACC_WPSR_WPVSRC_MASK       (0xffu << DACC_WPSR_WPVSRC_SHIFT) /* Write Protection Violation Source */

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_DACC_H */
