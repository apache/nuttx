/********************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_rtc.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_RTC_H
#define __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_RTC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>
#include "chip/lpc54_memorymap.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define LPC54_RTC_NGPREGS          8       /* Nubmer of general purpose registers */

/* Register offsets *************************************************************************/

#define LPC54_RTC_CTRL_OFFSET      0x0000  /* RTC control */
#define LPC54_RTC_MATCH_OFFSET     0x0004  /* RTC match */
#define LPC54_RTC_COUNT_OFFSET     0x0008  /* RTC counter */
#define LPC54_RTC_WAKE_OFFSET      0x000c  /* High-resolution/wake-up timer control */

/* General purpose registers */

#define LPC54_RTC_GPREG_OFFSET(n)  (0x0040 + ((n) << 2))

/* Register addresses ***********************************************************************/

#define LPC54_RTC_CTRL             (LPC54_RTC_BASE + LPC54_RTC_CTRL_OFFSET)
#define LPC54_RTC_MATCH            (LPC54_RTC_BASE + LPC54_RTC_MATCH_OFFSET)
#define LPC54_RTC_COUNT            (LPC54_RTC_BASE + LPC54_RTC_COUNT_OFFSET)
#define LPC54_RTC_WAKE             (LPC54_RTC_BASE + LPC54_RTC_WAKE_OFFSET)

/* General purpose registers */

#define LPC54_RTC_GPREG(n)         (LPC54_RTC_BASE + LPC54_RTC_GPREG_OFFSET(n))

/* Register bit definitions *****************************************************************/

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

#endif /* __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_RTC_H */
