/************************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_rtc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_RTC_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_RTC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define MAX326_RTC_SEC_OFFSET     0x0000  /* Seconds Counter Register */
#define MAX326_RTC_SSEC_OFFSET    0x0004  /* Sub-Seconds Counter Register */
#define MAX326_RTC_RAS_OFFSET     0x0008  /* Alarm Time-of-Day Register */
#define MAX326_RTC_RSSA_OFFSET    0x000c  /* Sub-Second Alarm Register */
#define MAX326_RTC_CTRL_OFFSET    0x0010  /* Control Register */
#define MAX326_RTC_TRIM_OFFSET    0x0014  /* Trim Register */
#define MAX326_RTC_OSCCTRL_OFFSET 0x0018  /* Oscillator Control Register */

/* Register Addresses ***************************************************************/

#define MAX326_RTC_SEC            (MAX326_RTC_BASE + MAX326_RTC_SEC_OFFSET)
#define MAX326_RTC_SSEC           (MAX326_RTC_BASE + MAX326_RTC_SSEC_OFFSET)
#define MAX326_RTC_RAS            (MAX326_RTC_BASE + MAX326_RTC_RAS_OFFSET)
#define MAX326_RTC_RSSA           (MAX326_RTC_BASE + MAX326_RTC_RSSA_OFFSET)
#define MAX326_RTC_CTRL           (MAX326_RTC_BASE + MAX326_RTC_CTRL_OFFSET)
#define MAX326_RTC_TRIM           (MAX326_RTC_BASE + MAX326_RTC_TRIM_OFFSET)
#define MAX326_RTC_OSCCTRL        (MAX326_RTC_BASE + MAX326_RTC_OSCCTRL_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Seconds Counter Register (32-bit seconds counter) */
/* Sub-Seconds Counter Register */

#define RTC_SSEC_MASK             (0xff)    /* Bits 0-7: Sub-second counter */

/* Alarm Time-of-Day Register */

#define RTC_RAS_MASK              (0xfffff) /* Bits 0-19: Time-of-Day Alarm */

/* Sub-Second Alarm Register (32-bit sub-second alarm value) */

/* Control Register */

#define RTC_CTRL_ENABLE           (1 << 0)  /* Bit 0:  Real-Time Clock Enable */
#define RTC_CTRL_ALARM_TODEN      (1 << 1)  /* Bit 1:  Time-of-Day Alarm Interrupt Enable */
#define RTC_CTRL_ALARM_SSEN       (1 << 2)  /* Bit 2:  Sub-Second Alarm Interrupt Enable */
#define RTC_CTRL_BUSY             (1 << 3)  /* Bit 3:  RTC Busy Flag */
#define RTC_CTRL_READY            (1 << 4)  /* Bit 4:  RTC Ready */
#define RTC_CTRL_READY_INTEN      (1 << 5)  /* Bit 5:  RTC Ready Interrupt Enable */
#define RTC_CTRL_ALARM_TODFL      (1 << 6)  /* Bit 6:  Time-of-Day Alarm Interrupt Flag */
#define RTC_CTRL_ALARM_SSFL       (1 << 7)  /* Bit 7:  Sub-second Alarm Interrupt Flag */
#define RTC_CTRL_32KOUTEN         (1 << 8)  /* Bit 8:  Square Wave Output Enable */
#define RTC_CTRL_FREQSEL_SHIFT    (9)       /* Bits 9-10: Frequency Output Select */
#define RTC_CTRL_FREQSEL_MASK     (3 << RTC_CTRL_FREQSEL_SHIFT)
#  define RTC_CTRL_FREQSEL_1HZ    (0 << RTC_CTRL_FREQSEL_SHIFT) /* 1Hz (Compensated) */
#  define RTC_CTRL_FREQSEL_512HZ  (1 << RTC_CTRL_FREQSEL_SHIFT) /* 512Hz (Compensated) */
#  define RTC_CTRL_FREQSEL_4KHZ   (2 << RTC_CTRL_FREQSEL_SHIFT) /* 4kHz */
#define RTC_CTRL_X32KMODE_SHIFT   (11)       /* Bits 11-12: 32kHz Oscillator Mode Select */
#define RTC_CTRL_X32KMODE_MASK    (3 << RTC_CTRL_X32KMODE_SHIFT)
#  define RTC_CTRL_X32KMODE_NI       (0 << RTC_CTRL_X32KMODE_SHIFT) /* Noise immunity mode */
#  define RTC_CTRL_X32KMODE_QUIET    (1 << RTC_CTRL_X32KMODE_SHIFT) /* Quiet mode */
#  define RTC_CTRL_X32KMODE_DSQUIET  (2 << RTC_CTRL_X32KMODE_SHIFT) /* Active: noise immunity mode
                                                                     * DEEPSLEEP: quiet mode */
#  define RTC_CTRL_X32KMODE_STPQUIET (3 << RTC_CTRL_X32KMODE_SHIFT) /* Active: noise immunity mode
                                                                     * STOP: quiet mode */
#define RTC_CTRL_WRITEEN          (1 << 15) /* Bit 15: Write Enable */

/* Trim Register */

#define RTC_TRIM_SHIFT            (0)       /* Bits 0-7:  RTC Trim */
#define RTC_TRIM_MASK             (0xff << RTC_TRIM_SHIFT)
#  define RTC_TRIM(n)             ((uint32_t)(n) << RTC_TRIM_SHIFT)
#define RTC_TRIM_VRTC_TMR_SHIFT   (8)       /* Bits 8-31: VRTC Time Counter */
#define RTC_TRIM_VRTC_TMR_MASK    (0xffffff << RTC_TRIM_VRTC_TMR_SHIFT)
#  define RTC_TRIM_VRTC_TMR(n)    ((uint32_t)(n) << RTC_TRIM_VRTC_TMR_SHIFT)

/* Oscillator Control Register */

#define RTC_OSCCTRL_FILTEREN      (1 << 0)  /* Bit 0:  RTC Oscillator Filter Enable */
#define RTC_OSCCTRL_IBIASSEL      (1 << 1)  /* Bit 1:  RTC Oscillator 4x Bias Current Select */
#define RTC_OSCCTRL_HYSTEN        (1 << 2)  /* Bit 2:  RTC Oscillator Hysteresis Buffer Enable */
#define RTC_OSCCTRL_IBIASEN       (1 << 3)  /* Bit 3:  RTC Oscillator Bias Current Enable */
#define RTC_OSCCTRL_BYPASS        (1 << 4)  /* Bit 4:  RTC Crystal Bypass */
#define RTC_OSCCTRL_32KOUT        (1 << 5)  /* Bit 5:  RTC Square Wave Output */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_RTC_H */
