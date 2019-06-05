/****************************************************************************************
 * arch/arm/src/sam34/hardware/sam_rtc.h
 * Real-time Clock (RTC) definitions for the SAM3U, SAM4E, and SAM4S
 *
 *   Copyright (C) 2009, 2013-2014 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_RTC_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_RTC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* RTC register offsets *****************************************************************/

#define SAM_RTC_CR_OFFSET            0x00 /* Control Register */
#define SAM_RTC_MR_OFFSET            0x04 /* Mode Register */
#define SAM_RTC_TIMR_OFFSET          0x08 /* Time Register */
#define SAM_RTC_CALR_OFFSET          0x0c /* Calendar Register */
#define SAM_RTC_TIMALR_OFFSET        0x10 /* Time Alarm Register */
#define SAM_RTC_CALALR_OFFSET        0x14 /* Calendar Alarm Register */
#define SAM_RTC_SR_OFFSET            0x18 /* Status Register */
#define SAM_RTC_SCCR_OFFSET          0x1c /* Status Clear Command Register */
#define SAM_RTC_IER_OFFSET           0x20 /* Interrupt Enable Register */
#define SAM_RTC_IDR_OFFSET           0x24 /* Interrupt Disable Register */
#define SAM_RTC_IMR_OFFSET           0x28 /* Interrupt Mask Register */
#define SAM_RTC_VER_OFFSET           0x2c /* Valid Entry Register */

/* RTC register addresses ***************************************************************/

#define SAM_RTC_CR                   (SAM_RTC_BASE+SAM_RTC_CR_OFFSET)
#define SAM_RTC_MR                   (SAM_RTC_BASE+SAM_RTC_MR_OFFSET)
#define SAM_RTC_TIMR                 (SAM_RTC_BASE+SAM_RTC_TIMR_OFFSET)
#define SAM_RTC_CALR                 (SAM_RTC_BASE+SAM_RTC_CALR_OFFSET)
#define SAM_RTC_TIMALR               (SAM_RTC_BASE+SAM_RTC_TIMALR_OFFSET)
#define SAM_RTC_CALALR               (SAM_RTC_BASE+SAM_RTC_CALALR_OFFSET)
#define SAM_RTC_SR                   (SAM_RTC_BASE+SAM_RTC_SR_OFFSET)
#define SAM_RTC_SCCR                 (SAM_RTC_BASE+SAM_RTC_SCCR_OFFSET)
#define SAM_RTC_IER                  (SAM_RTC_BASE+SAM_RTC_IER_OFFSET)
#define SAM_RTC_IDR                  (SAM_RTC_BASE+SAM_RTC_IDR_OFFSET)
#define SAM_RTC_IMR                  (SAM_RTC_BASE+SAM_RTC_IMR_OFFSET)
#define SAM_RTC_VER                  (SAM_RTC_BASE+SAM_RTC_VER_OFFSET)

/* RTC register bit definitions *********************************************************/

/* RTC Control Register */

#define RTC_CR_UPDTIM                (1 << 0)  /* Bit 0:  Update Request Time Register */
#define RTC_CR_UPDCAL                (1 << 1)  /* Bit 1:  Update Request Calendar Register */
#define RTC_CR_TIMEVSEL_SHIFT        (8)       /* Bits 8-9:  Time Event Selection */
#define RTC_CR_TIMEVSEL_MASK         (3 << RTC_CR_TIMEVSEL_SHIFT)
#  define RTC_CR_TIMEVSEL_MIN        (0 << RTC_CR_TIMEVSEL_SHIFT)
#  define RTC_CR_TIMEVSEL_HOUR       (1 << RTC_CR_TIMEVSEL_SHIFT)
#  define RTC_CR_TIMEVSEL_MIDNIGHT   (2 << RTC_CR_TIMEVSEL_SHIFT)
#  define RTC_CR_TIMEVSEL_NOON       (3 << RTC_CR_TIMEVSEL_SHIFT)
#define RTC_CR_CALEVSEL_SHIFT        (16)      /* Bits 16-17:  Calendar Event Selection */
#define RTC_CR_CALEVSEL_MASK         (3 << RTC_CR_CALEVSEL_SHIFT)
#  define RTC_CR_CALEVSEL_WEEK       (0 << RTC_CR_CALEVSEL_SHIFT)
#  define RTC_CR_CALEVSEL_MONTH      (1 << RTC_CR_CALEVSEL_SHIFT)
#  define RTC_CR_CALEVSEL_YEAR       (2 << RTC_CR_CALEVSEL_SHIFT)

/* RTC Mode Register */

#define RTC_MR_HRMOD                 (1 << 0)  /* Bit 0:  12-/24-hour Mode */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define RTC_MR_PERSIAN             (1 << 1)  /* Bit 1:  PERSIAN Calendar */
#  define RTC_MR_NEGPPM              (1 << 4)  /* Bit 4:  Negative PPM Correction */
#  define RTC_MR_CORRECTION_SHIFT    (8)       /* Bits 8-14: Slow Clock Correction */
#  define RTC_MR_CORRECTION_MASK     (0x7f << RTC_MR_CORRECTION_SHIFT)
#    define RTC_MR_CORRECTION(n)     ((uint32_t)(n) << RTC_MR_CORRECTION_SHIFT)
#  define RTC_MR_HIGHPPM             (1 << 15) /* Bit 15: HIGH PPM Correction */
#  define RTC_MR_OUT0_SHIFT          (16)      /* Bits 16-18: RTCOUT0 Output Source Selection */
#  define RTC_MR_OUT0_MASK           (7 << RTC_MR_OUT0_SHIFT)
#    define RTC_MR_OUT0_NOWAVE       (0 << RTC_MR_OUT0_SHIFT) /* No waveform, stuck at 0 */
#    define RTC_MR_OUT0_FREQ1HZ      (1 << RTC_MR_OUT0_SHIFT) /* 1Hz square wave */
#    define RTC_MR_OUT0_FREQ32HZ     (2 << RTC_MR_OUT0_SHIFT) /* 32Hz square wave */
#    define RTC_MR_OUT0_FREQ64HZ     (3 << RTC_MR_OUT0_SHIFT) /* 64Hz square wave */
#    define RTC_MR_OUT0_FREQ512HZ    (4 << RTC_MR_OUT0_SHIFT) /* 512Hz square wave */
#    define RTC_MR_OUT0_ALARM_TOGGLE (5 << RTC_MR_OUT0_SHIFT) /* Output toggles when alarm flag rises */
#    define RTC_MR_OUT0_ALARM_FLAG   (6 << RTC_MR_OUT0_SHIFT) /* Output is a copy of the alarm flag */
#    define RTC_MR_OUT0_PROG_PULSE   (7 << RTC_MR_OUT0_SHIFT) /* Duty cycle programmable pulse */
#  define RTC_MR_OUT1_SHIFT          (20)      /* Bits 20-22: RTCOUT1 Output Source Selection */
#  define RTC_MR_OUT1_MASK           (7 << RTC_MR_OUT1_SHIFT)
#    define RTC_MR_OUT1_NOWAVE       (0 << RTC_MR_OUT1_SHIFT) /* No waveform, stuck at 0 */
#    define RTC_MR_OUT1_FREQ1HZ      (1 << RTC_MR_OUT1_SHIFT) /* 1Hz square wave */
#    define RTC_MR_OUT1_FREQ32HZ     (2 << RTC_MR_OUT1_SHIFT) /* 32Hz square wave */
#    define RTC_MR_OUT1_FREQ64HZ     (3 << RTC_MR_OUT1_SHIFT) /* 64Hz square wave */
#    define RTC_MR_OUT1_FREQ512HZ    (4 << RTC_MR_OUT1_SHIFT) /* 512Hz square wave */
#    define RTC_MR_OUT1_ALARM_TOGGLE (5 << RTC_MR_OUT1_SHIFT) /* Output toggles when alarm flag rises */
#    define RTC_MR_OUT1_ALARM_FLAG   (6 << RTC_MR_OUT1_SHIFT) /* Output is a copy of the alarm flag */
#    define RTC_MR_OUT1_PROG_PULSE   (7 << RTC_MR_OUT1_SHIFT) /* Duty cycle programmable pulse */
#  define RTC_MR_THIGH_SHIFT         (24)      /* Bits 24-26: High Duration of the Output Pulse */
#  define RTC_MR_THIGH_MASK          (7 << RTC_MR_THIGH_SHIFT)
#    define RTC_MR_THIGH_31MS        (0 << RTC_MR_THIGH_SHIFT) /* 31.2 ms */
#    define RTC_MR_THIGH_16MS        (1 << RTC_MR_THIGH_SHIFT) /* 15.6 ms */
#    define RTC_MR_THIGH_4MS         (2 << RTC_MR_THIGH_SHIFT) /* 3.91 ms */
#    define RTC_MR_THIGH_976US       (3 << RTC_MR_THIGH_SHIFT) /* 976 탎 */
#    define RTC_MR_THIGH_488US       (4 << RTC_MR_THIGH_SHIFT) /* 488 탎 */
#    define RTC_MR_THIGH_22US        (5 << RTC_MR_THIGH_SHIFT) /* 122 탎 */
#    define RTC_MR_THIGH_0US         (6 << RTC_MR_THIGH_SHIFT) /* 30.5 탎 */
#    define RTC_MR_THIGH_15US        (7 << RTC_MR_THIGH_SHIFT) /* 15.2 탎 */
#  define RTC_MR_TPERIOD_SHIFT       (28)      /* Bits 28-29: Period of the Output Pulse */
#  define RTC_MR_TPERIOD_MASK        (3 << RTC_MR_TPERIOD_SHIFT)
#    define RTC_MR_TPERIOD_1S        (0 << RTC_MR_TPERIOD_SHIFT) /* 1 second */
#    define RTC_MR_TPERIOD_500MS     (1 << RTC_MR_TPERIOD_SHIFT) /* 500 ms */
#    define RTC_MR_TPERIOD_250MS     (2 << RTC_MR_TPERIOD_SHIFT) /* 250 ms */
#    define RTC_MR_TPERIOD_125MS     (3 << RTC_MR_TPERIOD_SHIFT) /* 125 ms */
#endif

/* RTC Time Register */

#define RTC_TIMR_SEC_SHIFT           (0)       /* Bits 0-6:  Current Second */
#define RTC_TIMR_SEC_MASK            (0x7f << RTC_TIMR_SEC_SHIFT)
#  define RTC_TIMR_SEC(n)            ((uint32_t)(n) << RTC_TIMR_SEC_SHIFT)
#define RTC_TIMR_MIN_SHIFT           (8)       /* Bits 8-14:  Current Minute */
#define RTC_TIMR_MIN_MASK            (0x7f <<  RTC_TIMR_MIN_SHIFT)
#  define RTC_TIMR_MIN(n)            ((uint32_t)(n) <<  RTC_TIMR_MIN_SHIFT)
#define RTC_TIMR_HOUR_SHIFT          (16)      /* Bits 16-21: Current Hour */
#define RTC_TIMR_HOUR_MASK           (0x3f << RTC_TIMR_HOUR_SHIFT)
#  define RTC_TIMR_HOUR(n)           ((uint32_t)(n) << RTC_TIMR_HOUR_SHIFT)
#define RTC_TIMR_AMPM                (1 << 22) /* Bit 22: Ante Meridiem Post Meridiem Indicator */

/* RTC Calendar Register */

#define RTC_CALR_CENT_SHIFT          (0)       /* Bits 0-6:  Current Century */
#define RTC_CALR_CENT_MASK           (0x7f << RTC_CALR_CENT_SHIFT)
#  define RTC_CALR_CENT(n)           ((uint32_t)(n) << RTC_CALR_CENT_SHIFT)
#define RTC_CALR_YEAR_SHIFT          (8)       /* Bits 8-15:  Current Year */
#define RTC_CALR_YEAR_MASK           (0xff << RTC_CALR_YEAR_SHIFT)
#  define RTC_CALR_YEAR(n)           ((uint32_t)(n) << RTC_CALR_YEAR_SHIFT)
#define RTC_CALR_MONTH_SHIFT         (16)      /* Bits 16-20: Current Month */
#define RTC_CALR_MONTH_MASK          (0x1f << RTC_CALR_MONTH_SHIFT)
#  define RTC_CALR_MONTH(n)           ((uint32_t)(n) << RTC_CALR_MONTH_SHIFT)
#define RTC_CALR_DAY_SHIFT           (21)      /* Bits 21-23: Current Day in Current Week */
#define RTC_CALR_DAY_MASK            (7 << RTC_CALR_DAY_SHIFT)
#  define RTC_CALR_DAY(n)            ((uint32_t)(n)7 << RTC_CALR_DAY_SHIFT)
#define RTC_CALR_DATE_SHIFT          (24)      /* Bits 24-29: Current Day in Current Month */
#define RTC_CALR_DATE_MASK           (0x3f << RTC_CALR_DATE_SHIFT)
#  define RTC_CALR_DATE(n)           ((uint32_t)(n) << RTC_CALR_DATE_SHIFT)

/* RTC Time Alarm Register */

#define RTC_TIMALR_SEC_SHIFT         (0)       /* Bits 0-6:  Second Alarm */
#define RTC_TIMALR_SEC_MASK          (0x7f << RTC_TIMALR_SEC_SHIFT)
#  define RTC_TIMALR_SEC(n)          ((uint32_t)(n) << RTC_TIMALR_SEC_SHIFT)
#define RTC_TIMALR_SECEN             (1 << 7)  /* Bit 7:  Second Alarm Enable */
#define RTC_TIMALR_MIN_SHIFT         (8)       /* Bits 8-14:  Minute Alarm */
#define RTC_TIMALR_MIN_MASK          (0x7f << RTC_TIMALR_MIN_SHIFT)
#  define RTC_TIMALR_MIN(n)          ((uint32_t)(n) << RTC_TIMALR_MIN_SHIFT)
#define RTC_TIMALR_MINEN             (1 << 15) /* Bit 15: Minute Alarm Enable */
#define RTC_TIMALR_HOUR_SHIFT        (16)      /* Bits 16-21:  Hour Alarm */
#define RTC_TIMALR_HOUR_MASK         (0x3f << RTC_TIMALR_HOUR_SHIFT)
#  define RTC_TIMALR_HOUR(n)         ((uint32_t)(n) << RTC_TIMALR_HOUR_SHIFT)
#define RTC_TIMALR_AMPM              (1 << 22) /* Bit 22: AM/PM Indicator */
#define RTC_TIMALR_HOUREN            (1 << 23) /* Bit 23: Hour Alarm Enable */

/* RTC Calendar Alarm Register */

#define RTC_CALALR_MONTH_SHIFT       (16)      /* Bits 16-20:  Month Alarm */
#define RTC_CALALR_MONTH_MASK        (0x1f << RTC_CALALR_MONTH_SHIFT)
#  define RTC_CALALR_MONTH(n)        ((uint32_t)(n) << RTC_CALALR_MONTH_SHIFT)
#define RTC_CALALR_MTHEN             (1 << 23) /* Bit 23: Month Alarm Enable */
#define RTC_CALALR_DATE_SHIFT        (24)      /* Bits 24-29:  Date Alarm */
#define RTC_CALALR_DATE_MASK         (0x3f << RTC_CALALR_DATE_SHIFT)
#  define RTC_CALALR_DATE(n)         ((uint32_t)(n) << RTC_CALALR_DATE_SHIFT)
#define RTC_CALALR_DATEEN            (1 << 31) /* Bit 31: Date Alarm Enable */

/* RTC Status Register */

#define RTC_SR_ACKUPD                (1 << 0)  /* Bit 0:  Acknowledge for Update */
#define RTC_SR_ALARM                 (1 << 1)  /* Bit 1:  Alarm Flag */
#define RTC_SR_SEC                   (1 << 2)  /* Bit 2:  Second Event */
#define RTC_SR_TIMEV                 (1 << 3)  /* Bit 3:  Time Event */
#define RTC_SR_CALEV                 (1 << 4)  /* Bit 4:  Calendar Event */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define RTC_SR_TDERR               (1 << 5)  /* Bit 5:  Time and/or Date Free Running Error */
#endif

/* RTC Status Clear Command Register */

#define RTC_SCCR_ACKCLR              (1 << 0)  /* Bit 0:  Acknowledge Clear */
#define RTC_SCCR_ALRCLR              (1 << 1)  /* Bit 1:  Alarm Clear */
#define RTC_SCCR_SECCLR              (1 << 2)  /* Bit 2:  Second Clear */
#define RTC_SCCR_TIMCLR              (1 << 3)  /* Bit 3:  Time Clear */
#define RTC_SCCR_CALCLR              (1 << 4)  /* Bit 4:  Calendar Clear */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define RTC_SR_TDERRCLR            (1 << 5)  /* Bit 5:  Time and/or Date Free Running Error Clear */
#endif

/* RTC Interrupt Enable Register */

#define RTC_IER_ACKEN                (1 << 0)  /* Bit 0:  Acknowledge Update Interrupt Enable */
#define RTC_IER_ALREN                (1 << 1)  /* Bit 1:  Alarm Interrupt Enable */
#define RTC_IER_SECEN                (1 << 2)  /* Bit 2:  Second Event Interrupt Enable */
#define RTC_IER_TIMEN                (1 << 3)  /* Bit 3:  Time Event Interrupt Enable */
#define RTC_IER_CALEN                (1 << 4)  /* Bit 4:  Calendar Event Interrupt Enable */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define RTC_SR_TDERREN             (1 << 5)  /* Bit 5:  Time and/or Date Error Interrupt Enable */
#endif

/* RTC Interrupt Disable Register */

#define RTC_IDR_ACKDIS               (1 << 0)  /* Bit 0:  Acknowledge Update Interrupt Disable */
#define RTC_IDR_ALRDIS               (1 << 1)  /* Bit 1:  Alarm Interrupt Disable */
#define RTC_IDR_SECDIS               (1 << 2)  /* Bit 2:  Second Event Interrupt Disable */
#define RTC_IDR_TIMDIS               (1 << 3)  /* Bit 3:  Time Event Interrupt Disable */
#define RTC_IDR_CALDIS               (1 << 4)  /* Bit 4:  Calendar Event Interrupt Disable */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define RTC_SR_TDERRDIS            (1 << 5)  /* Bit 5:  Time and/or Date Error Interrupt Disable */
#endif

/* RTC Interrupt Mask Register */

#define RTC_IMR_ACK                  (1 << 0)  /* Bit 0:  Acknowledge Update Interrupt Mask */
#define RTC_IMR_ALR                  (1 << 1)  /* Bit 1:  Alarm Interrupt Mask */
#define RTC_IMR_SEC                  (1 << 2)  /* Bit 2:  Second Event Interrupt Mask */
#define RTC_IMR_TIM                  (1 << 3)  /* Bit 3:  Time Event Interrupt Mask */
#define RTC_IMR_CAL                  (1 << 4)  /* Bit 4:  Calendar Event Interrupt Mask */

/* RTC Valid Entry Register */

#define RTC_VER_NVTIM                (1 << 0)  /* Bit 0:  Non-valid Time */
#define RTC_VER_NVCAL                (1 << 1)  /* Bit 1:  Non-valid Calendar */
#define RTC_VER_NVTIMALR             (1 << 2)  /* Bit 2:  Non-valid Time Alarm */
#define RTC_VER_NVCALALR             (1 << 3)  /* Bit 3:  Non-valid Calendar Alarm */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_RTC_H */
