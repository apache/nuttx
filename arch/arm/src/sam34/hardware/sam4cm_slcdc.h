/****************************************************************************
 * arch/arm/src/sam34/hardware/sam4cm_slcdc.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_SLCDC_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_SLCDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SLCDC register offsets ***************************************************/

#define SAM_SLCDC_CR_OFFSET        0x0000 /* Control Register */
#define SAM_SLCDC_MR_OFFSET        0x0004 /* Mode Register */
#define SAM_SLCDC_FRR_OFFSET       0x0008 /* Frame Rate Register */
#define SAM_SLCDC_DR_OFFSET        0x000C /* Display Register */
#define SAM_SLCDC_SR_OFFSET        0x0010 /* Status Register */
#define SAM_SLCDC_IER_OFFSET       0x0020 /* Interrupt Enable Register */
#define SAM_SLCDC_IDR_OFFSET       0x0024 /* Interrupt Disable Register */
#define SAM_SLCDC_IMR_OFFSET       0x0028 /* Interrupt Mask Register */
#define SAM_SLCDC_ISR_OFFSET       0x002C /* Interrupt Status Register */
#define SAM_SLCDC_SMR0_OFFSET      0x0030 /* Segment Map Register 0 */
#define SAM_SLCDC_SMR1_OFFSET      0x0034 /* Segment Map Register 1 */
#  define SAM_SLCDC_SMR_OFFSET(n)  (SAM_SLCDC_SMR0_OFFSET + (n) * 4)
#define SAM_SLCDC_WPMR_OFFSET      0x0034 /*  Write Protect Mode Register */
#define SAM_SLCDC_WPSR_OFFSET      0x0034 /*  Write Protect Status Register */

#define SAM_SLCDC_LMEMR_OFFSET(com) (0x200 + (com)*8 + 0x0)
#define SAM_SLCDC_MMEMR_OFFSET(com) (0x200 + (com)*8 + 0x4)

/* SLCDC register addresses *************************************************/

#define SAM_SLCDC_CR               (SAM_SLCDC_BASE + SAM_SLCDC_CR_OFFSET)
#define SAM_SLCDC_MR               (SAM_SLCDC_BASE + SAM_SLCDC_MR_OFFSET)
#define SAM_SLCDC_FRR              (SAM_SLCDC_BASE + SAM_SLCDC_FRR_OFFSET)
#define SAM_SLCDC_DR               (SAM_SLCDC_BASE + SAM_SLCDC_DR_OFFSET)
#define SAM_SLCDC_SR               (SAM_SLCDC_BASE + SAM_SLCDC_SR_OFFSET)
#define SAM_SLCDC_IER              (SAM_SLCDC_BASE + SAM_SLCDC_IER_OFFSET)
#define SAM_SLCDC_IDR              (SAM_SLCDC_BASE + SAM_SLCDC_IDR_OFFSET)
#define SAM_SLCDC_IMR              (SAM_SLCDC_BASE + SAM_SLCDC_IMR_OFFSET)
#define SAM_SLCDC_ISR              (SAM_SLCDC_BASE + SAM_SLCDC_ISR_OFFSET)
#define SAM_SLCDC_SMR0             (SAM_SLCDC_BASE + SAM_SLCDC_SMR0_OFFSET)
#define SAM_SLCDC_SMR1             (SAM_SLCDC_BASE + SAM_SLCDC_SMR1_OFFSET)
#  define SAM_SLCDC_SMR(n)         (SAM_SLCDC_BASE + SAM_SLCDC_SMR_OFFSET(n))
#define SAM_SLCDC_WPMR             (SAM_SLCDC_BASE + SAM_SLCDC_WPMR_OFFSET)
#define SAM_SLCDC_WPSR             (SAM_SLCDC_BASE + SAM_SLCDC_WPSR_OFFSET)

#define SAM_SLCDC_LMEMR(com)       (SAM_SLCDC_BASE + SAM_SLCDC_LMEMR_OFFSET(com))
#define SAM_SLCDC_MMEMR(com)       (SAM_SLCDC_BASE + SAM_SLCDC_MMEMR_OFFSET(com))

/* SLCDC register bit definitions *******************************************/

/* Control Register */

#define SLCDC_CR_LCDEN             (1 << 0)
#define SLCDC_CR_LCDDIS            (1 << 1)
#define SLCDC_CR_SWRST             (1 << 3)

/* Mode Register */

#define SLCDC_MR_COMSEL(N)         (((N) - 1) << 0)
#define SLCDC_MR_SEGSEL(N)         (((N) - 1) << 8)

#define SLCDC_MR_BUFFTIME_OFFSET   (16)
#define SLCDC_MR_BUFFTIME_MASK     (0xF << SLCDC_MR_BUFFTIME_OFFSET)
#  define SLCDC_MR_BUFFTIME_OFF         (0 << SLCDC_MR_BUFFTIME_OFFSET)    /* Nominal drive time is 0% of SCLK period */
#  define SLCDC_MR_BUFFTIME_X2_SCLK     (1 << SLCDC_MR_BUFFTIME_OFFSET)    /* Nominal drive time is 2 periods of SCLK clock */
#  define SLCDC_MR_BUFFTIME_X4_SCLK     (2 << SLCDC_MR_BUFFTIME_OFFSET)    /* Nominal drive time is 4 periods of SCLK clock */
#  define SLCDC_MR_BUFFTIME_X8_SCLK     (3 << SLCDC_MR_BUFFTIME_OFFSET)    /* Nominal drive time is 8 periods of SCLK clock */
#  define SLCDC_MR_BUFFTIME_X16_SCLK    (4 << SLCDC_MR_BUFFTIME_OFFSET)    /* Nominal drive time is 16 periods of SCLK clock */
#  define SLCDC_MR_BUFFTIME_X32_SCLK    (5 << SLCDC_MR_BUFFTIME_OFFSET)    /* Nominal drive time is 32 periods of SCLK clock */
#  define SLCDC_MR_BUFFTIME_X64_SCLK    (6 << SLCDC_MR_BUFFTIME_OFFSET)    /* Nominal drive time is 64 periods of SCLK clock */
#  define SLCDC_MR_BUFFTIME_X128_SCLK   (7 << SLCDC_MR_BUFFTIME_OFFSET)    /* Nominal drive time is 128 periods of SCLK clock */
#  define SLCDC_MR_BUFFTIME_PERCENT_50  (8 << SLCDC_MR_BUFFTIME_OFFSET)    /* Nominal drive time is 50% of SCLK period */
#  define SLCDC_MR_BUFFTIME_PERCENT_100 (9 << SLCDC_MR_BUFFTIME_OFFSET)    /* Nominal drive time is 100% of SCLK period */

#define SLCDC_MR_BIAS_OFFSET       (20)
#define SLCDC_MR_BIAS_MASK         (0x3 << SLCDC_MR_BIAS_OFFSET)
#  define SLCDC_MR_BIAS_STATIC     (0 << SLCDC_MR_BIAS_OFFSET)             /* Static */
#  define SLCDC_MR_BIAS_1_2        (1 << SLCDC_MR_BIAS_OFFSET)             /* Bias 1/2 */
#  define SLCDC_MR_BIAS_1_3        (2 << SLCDC_MR_BIAS_OFFSET)             /* Bias 1/3 */

#define SLCD_MR_LPMODE_LOW_POWER   (1 << 24)

/* Frame Rate Register */

#define SLCDC_FRR_PRESC_OFFSET     (0)
#define SLCDC_FRR_PRESC_MASK       (0x7 << SLCDC_FRR_PRESC_OFFSET)
#  define SLCDC_FRR_PRESC_SCLK_DIV8     (0x0 << SLCDC_FRR_PRESC_OFFSET) /* Slow clock is divided by 8 */
#  define SLCDC_FRR_PRESC_SCLK_DIV16    (0x1 << SLCDC_FRR_PRESC_OFFSET) /* Slow clock is divided by 16 */
#  define SLCDC_FRR_PRESC_SCLK_DIV32    (0x2 << SLCDC_FRR_PRESC_OFFSET) /* Slow clock is divided by 32 */
#  define SLCDC_FRR_PRESC_SCLK_DIV64    (0x3 << SLCDC_FRR_PRESC_OFFSET) /* Slow clock is divided by 64 */
#  define SLCDC_FRR_PRESC_SCLK_DIV128   (0x4 << SLCDC_FRR_PRESC_OFFSET) /* Slow clock is divided by 128 */
#  define SLCDC_FRR_PRESC_SCLK_DIV256   (0x5 << SLCDC_FRR_PRESC_OFFSET) /* Slow clock is divided by 256 */
#  define SLCDC_FRR_PRESC_SCLK_DIV512   (0x6 << SLCDC_FRR_PRESC_OFFSET) /* Slow clock is divided by 512 */
#  define SLCDC_FRR_PRESC_SCLK_DIV1024  (0x7 << SLCDC_FRR_PRESC_OFFSET) /* Slow clock is divided by 1024 */

#define SLCDC_FRR_DIV_OFFSET       (8)
#define SLCDC_FRR_DIV_MASK         (0x7 << SLCDC_FRR_DIV_OFFSET)
#  define SLCDC_FRR_DIV(N)              (((N) - 1) << SLCDC_FRR_DIV_OFFSET)    /* Clock output from prescaler is divided by N */

/* Display Register */

#define SLCDC_DR_DISPMODE_OFFSET   (0)
#define SLCDC_DR_DISPMODE_MASK     (0x7 << SLCDC_DR_DISPMODE_OFFSET)
#  define SLCDC_DR_DISPMODE_NORMAL           (0x0 << SLCDC_DR_DISPMODE_OFFSET) /* Latched data are displayed */
#  define SLCDC_DR_DISPMODE_FORCE_OFF        (0x1 << SLCDC_DR_DISPMODE_OFFSET) /* All pixels are invisible. (The SLCDC memory is unchanged.) */
#  define SLCDC_DR_DISPMODE_FORCE_ON         (0x2 << SLCDC_DR_DISPMODE_OFFSET) /* All pixels are visible. (The SLCDC memory is unchanged.) */
#  define SLCDC_DR_DISPMODE_BLINKING         (0x3 << SLCDC_DR_DISPMODE_OFFSET) /* All pixels are alternately turned off to the predefined state in SLCDC memory at LCDBLKFREQ frequency */
#  define SLCDC_DR_DISPMODE_INVERTED         (0x4 << SLCDC_DR_DISPMODE_OFFSET) /* All pixels are set in the inverted state as defined in SLCDC memory */
#  define SLCDC_DR_DISPMODE_INVERTED_BLINK   (0x5 << SLCDC_DR_DISPMODE_OFFSET) /* All pixels are alternately turned off to the predefined opposite state in SLCDC memory at LCDBLKFREQ frequency */
#  define SLCDC_DR_DISPMODE_USER_BUFFER_LOAD (0x6 << SLCDC_DR_DISPMODE_OFFSET) /* Blocks the automatic transfer from User Buffer to Display Buffer */
#  define SLCDC_DR_DISPMODE_BUFFERS_SWAP     (0x7 << SLCDC_DR_DISPMODE_OFFSET) /* All pixels are alternatively assigned to the state defined in the User Buffer, then to the state defined in the Display Buffer at LCDBLKFREQ frequency */

#define SLCDC_DR_LCDBLKFREQ(F)     ((F) << 8)                           /* Blinking frequency = Frame Frequency/F */

/* Status Register */

#define SLCDC_SR_ENA               (1 << 0)                             /* The SLCDC is enabled/disabled */

/* Interrupt Enable Register */

#define SLCDC_IER_ENDFRAME         (1 << 0)                             /* End of Frame Interrupt Enable */
#define SLCDC_IER_DIS              (1 << 2)                             /* Enables the "Disable" interrupt */

/* Interrupt Disable Register */

#define SLCDC_IDR_ENDFRAME         (1 << 0)                             /* End of Frame Interrupt Disable */
#define SLCDC_IDR_DIS              (1 << 2)                             /* Disables the "Disable" interrupt */

/* Interrupt Mask Register */

#define SLCDC_IMR_ENDFRAME         (1 << 0)                             /* End of Frame Interrupt mask (0 - disable, 1 - enable) */
#define SLCDC_IMR_DIS              (1 << 2)                             /* The "Disable" interrupt mask (0 - disable, 1 - enable) */

/* Interrupt Status Register */

#define SLCDC_ISR_ENDFRAME         (1 << 0)                             /* End of Frame Interrupt occurred */
#define SLCDC_ISR_DIS              (1 << 2)                             /* The "Disable" interrupt occurred */

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4CM_SLCDC_H */
