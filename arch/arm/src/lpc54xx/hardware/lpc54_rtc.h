/****************************************************************************
 * arch/arm/src/lpc54xx/hardware/lpc54_rtc.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_RTC_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LPC54_RTC_NGPREGS          8       /* Number of general purpose registers */

/* Register offsets *********************************************************/

#define LPC54_RTC_CTRL_OFFSET      0x0000  /* RTC control */
#define LPC54_RTC_MATCH_OFFSET     0x0004  /* RTC match */
#define LPC54_RTC_COUNT_OFFSET     0x0008  /* RTC counter */
#define LPC54_RTC_WAKE_OFFSET      0x000c  /* High-resolution/wake-up timer control */

/* General purpose registers */

#define LPC54_RTC_GPREG_OFFSET(n)  (0x0040 + ((n) << 2))

/* Register addresses *******************************************************/

#define LPC54_RTC_CTRL             (LPC54_RTC_BASE + LPC54_RTC_CTRL_OFFSET)
#define LPC54_RTC_MATCH            (LPC54_RTC_BASE + LPC54_RTC_MATCH_OFFSET)
#define LPC54_RTC_COUNT            (LPC54_RTC_BASE + LPC54_RTC_COUNT_OFFSET)
#define LPC54_RTC_WAKE             (LPC54_RTC_BASE + LPC54_RTC_WAKE_OFFSET)

/* General purpose registers */

#define LPC54_RTC_GPREG(n)         (LPC54_RTC_BASE + LPC54_RTC_GPREG_OFFSET(n))

/* Register bit definitions *************************************************/

/* RTC control */

#define RTC_CTRL_SWRESET           (1 << 0)  /* Bit 0:  Software reset control */
#define RTC_CTRL_ALARM1HZ          (1 << 2)  /* Bit 2:  RTC 1 Hz timer alarm flag status */
#define RTC_CTRL_WAKE1KHZ          (1 << 3)  /* Bit 3:  RTC 1 kHz timer wake-up flag status */
#define RTC_CTRL_ALARMDPDEN        (1 << 4)  /* Bit 4:  RTC 1 Hz timer alarm enable for deep power-down */
#define RTC_CTRL_WAKEDPDEN         (1 << 5)  /* Bit 5:  RTC 1 kHz timer wake-up enable for deep power-down */
#define RTC_CTRL_RTC1KHZEN         (1 << 6)  /* Bit 6:  RTC 1 kHz clock enable */
#define RTC_CTRL_RTCEN             (1 << 7)  /* Bit 7:  RTC enable */
#define RTC_CTRL_OSCPD             (1 << 8)  /* Bit 8:  RTC oscillator power-down control */

/* RTC match (32-bit timer match value) */

/* RTC counter (32-bit counter value) */

/* High-resolution/wake-up timer control */

#define RTC_WAKE_MASK              0xffff    /* Bits 0-15: 16 hi-resoution/wake-up timer */

/* General purpose registers (32-bit value) */

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_RTC_H */
