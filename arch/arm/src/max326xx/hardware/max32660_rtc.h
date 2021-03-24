/****************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_rtc.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_RTC_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define MAX326_RTC_SEC_OFFSET     0x0000  /* Seconds Counter Register */
#define MAX326_RTC_SSEC_OFFSET    0x0004  /* Sub-Seconds Counter Register */
#define MAX326_RTC_RAS_OFFSET     0x0008  /* Alarm Time-of-Day Register */
#define MAX326_RTC_RSSA_OFFSET    0x000c  /* Sub-Second Alarm Register */
#define MAX326_RTC_CTRL_OFFSET    0x0010  /* Control Register */
#define MAX326_RTC_TRIM_OFFSET    0x0014  /* Trim Register */
#define MAX326_RTC_OSCCTRL_OFFSET 0x0018  /* Oscillator Control Register */

/* Register Addresses *******************************************************/

#define MAX326_RTC_SEC            (MAX326_RTC_BASE + MAX326_RTC_SEC_OFFSET)
#define MAX326_RTC_SSEC           (MAX326_RTC_BASE + MAX326_RTC_SSEC_OFFSET)
#define MAX326_RTC_RAS            (MAX326_RTC_BASE + MAX326_RTC_RAS_OFFSET)
#define MAX326_RTC_RSSA           (MAX326_RTC_BASE + MAX326_RTC_RSSA_OFFSET)
#define MAX326_RTC_CTRL           (MAX326_RTC_BASE + MAX326_RTC_CTRL_OFFSET)
#define MAX326_RTC_TRIM           (MAX326_RTC_BASE + MAX326_RTC_TRIM_OFFSET)
#define MAX326_RTC_OSCCTRL        (MAX326_RTC_BASE + MAX326_RTC_OSCCTRL_OFFSET)

/* Register Bit-field Definitions *******************************************/

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
