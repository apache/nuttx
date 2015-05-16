/********************************************************************************************
 * arch/arm/src/samdl/chip/saml_osc32kctrl.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
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

#ifndef __ARCH_ARM_SRC_SAMDL_CHIP_SAML_OS32KCCTRL_H
#define __ARCH_ARM_SRC_SAMDL_CHIP_SAML_OS32KCCTRL_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* OS32KCCTRL register offsets **************************************************************/

#define SAM_OS32KCCTRL_INTENCLR_OFFSET   0x0000  /* Interrupt enable clear */
#define SAM_OS32KCCTRL_INTENSET_OFFSET   0x0004  /* Interrupt enable set */
#define SAM_OS32KCCTRL_INTFLAG_OFFSET    0x0008  /* Interrupt flag status and clear */
#define SAM_OS32KCCTRL_STATUS_OFFSET     0x000c  /* Status */
#define SAM_OS32KCCTRL_RTCCTRL_OFFSET    0x0010  /* RTC clock selection */
#define SAM_OS32KCCTRL_XOSC32K_OFFSET    0x0014  /* 32kHz external crystal oscillator control */
#define SAM_OS32KCCTRL_OSC32K_OFFSET     0x0018  /* 32kHz internal oscillator control */
#define SAM_OS32KCCTRL_OSCULP32K_OFFSET  0x001c  /* 32kHz ultra low power internal oscillator control */


/* OS32KCCTRL register addresses ************************************************************/

#define SAM_OS32KCCTRL_INTENCLR          (SAM_OS32KCCTRL_BASE+SAM_OS32KCCTRL_INTENCLR_OFFSET)
#define SAM_OS32KCCTRL_INTENSET          (SAM_OS32KCCTRL_BASE+SAM_OS32KCCTRL_INTENSET_OFFSET)
#define SAM_OS32KCCTRL_INTFLAG           (SAM_OS32KCCTRL_BASE+SAM_OS32KCCTRL_INTFLAG_OFFSET)
#define SAM_OS32KCCTRL_STATUS            (SAM_OS32KCCTRL_BASE+SAM_OS32KCCTRL_STATUS_OFFSET)
#define SAM_OS32KCCTRL_RTCCTRL           (SAM_OS32KCCTRL_BASE+SAM_OS32KCCTRL_RTCCTRL_OFFSET)
#define SAM_OS32KCCTRL_XOSC32K           (SAM_OS32KCCTRL_BASE+SAM_OS32KCCTRL_XOSC32K_OFFSET)
#define SAM_OS32KCCTRL_OSC32K            (SAM_OS32KCCTRL_BASE+SAM_OS32KCCTRL_OSC32K_OFFSET)
#define SAM_OS32KCCTRL_OSCULP32K         (SAM_OS32KCCTRL_BASE+SAM_OS32KCCTRL_OSCULP32K_OFFSET)

/* OS32KCCTRL register bit definitions ******************************************************/

/* Interrupt enable clear, Interrupt enable set, Interrupt flag status and clear, and
 * status registers.
 */

#define OS32KCCTRL_INT_XOSC32KRDY        (1 << 0)  /* Bit 0:  XOSC32K ready interrupt */
#define OS32KCCTRL_INT_OSC32KRDY         (1 << 1)  /* Bit 1:  OSC32K ready interrupt */

#define OS32KCCTRL_INT_ALL               (0x00000003)

/* RTC clock selection */

#define OS32KCCTRL_RTCCTRL_RTCSEL_SHIFT  (0)       /* Bits 0-2: RTC clock source selection */
#define OS32KCCTRL_RTCCTRL_RTCSEL_MASK   (7 << OS32KCCTRL_RTCCTRL_RTCSEL_SHIFT)
#  define OS32KCCTRL_RTCCTRL_RTCSEL_ULP1K    (0 << OS32KCCTRL_RTCCTRL_RTCSEL_SHIFT) /* 1.024KHz from 32HKz internal ULP oscillator */
#  define OS32KCCTRL_RTCCTRL_RTCSEL_ULP32K   (1 << OS32KCCTRL_RTCCTRL_RTCSEL_SHIFT) /* 32.768KHz from 32KHz internal ULP oscillator */
#  define OS32KCCTRL_RTCCTRL_RTCSEL_OSC1K    (2 << OS32KCCTRL_RTCCTRL_RTCSEL_SHIFT) /* 1.024KHz for 32KHz internal oscillator */
#  define OS32KCCTRL_RTCCTRL_RTCSEL_XOSC1K   (3 << OS32KCCTRL_RTCCTRL_RTCSEL_SHIFT) /* 1.024KHz for 32KHz external oscillator */
#  define OS32KCCTRL_RTCCTRL_RTCSEL_OSC32K   (4 << OS32KCCTRL_RTCCTRL_RTCSEL_SHIFT) /* 32.768KHz from 32KHz external oscillator */
#  define OS32KCCTRL_RTCCTRL_RTCSEL_XOSC312K (5 << OS32KCCTRL_RTCCTRL_RTCSEL_SHIFT) /* 32.768KHz from 32KHz external crystal oscillator */

/* 32kHz external crystal oscillator control register */

#define OS32KCCTRL_XOSC32K_ENABLE        (1 << 1)  /* Bit 1:  Oscillator enable */
#define OS32KCCTRL_XOSC32K_XTALEN        (1 << 2)  /* Bit 2:  Crystal oscillator enable */
#define OS32KCCTRL_XOSC32K_EN32K         (1 << 3)  /* Bit 3:  32kHz Output enable */
#define OS32KCCTRL_XOSC32K_EN1K          (1 << 4)  /* Bit 4:  1kHz Output enable */
#define OS32KCCTRL_XOSC32K_RUNSTDBY      (1 << 6)  /* Bit 6:  Run in standby */
#define OS32KCCTRL_XOSC32K_ONDEMAND      (1 << 7)  /* Bit 7:  On demand control */
#define OS32KCCTRL_XOSC32K_STARTUP_SHIFT     (8)     /* Bits 8-10: Oscillator start-up time */
#define OS32KCCTRL_XOSC32K_STARTUP_MASK      (7 << OS32KCCTRL_XOSC32K_STARTUP_SHIFT)
#  define OS32KCCTRL_XOSC32K_STARTUP(n)      ((n) << OS32KCCTRL_XOSC32K_STARTUP_SHIFT)
#  define OS32KCCTRL_XOSC32K_STARTUP_63MS    (0 << OS32KCCTRL_XOSC32K_STARTUP_SHIFT) /* 62.592 msec */
#  define OS32KCCTRL_XOSC32K_STARTUP_125MS   (1 << OS32KCCTRL_XOSC32K_STARTUP_SHIFT) /* 125.092 msec */
#  define OS32KCCTRL_XOSC32K_STARTUP_500MS   (2 << OS32KCCTRL_XOSC32K_STARTUP_SHIFT) /* 500.092 msec */
#  define OS32KCCTRL_XOSC32K_STARTUP_100MS   (3 << OS32KCCTRL_XOSC32K_STARTUP_SHIFT) /* 100.0092 msec */
#  define OS32KCCTRL_XOSC32K_STARTUP_200MS   (4 << OS32KCCTRL_XOSC32K_STARTUP_SHIFT) /* 200.0092 msec */
#  define OS32KCCTRL_XOSC32K_STARTUP_400MS   (5 << OS32KCCTRL_XOSC32K_STARTUP_SHIFT) /* 400.092 msec */
#  define OS32KCCTRL_XOSC32K_STARTUP_800MS   (6 << OS32KCCTRL_XOSC32K_STARTUP_SHIFT) /* 800.0092 msec */
#define OS32KCCTRL_XOSC32K_WRTLOCK       (1 << 12)  /* Bit 12: Write lock */

/* 32kHz internal oscillator control register */

#define OS32KCCTRL_OSC32K_ENABLE         (1 << 1)  /* Bit 1:  Oscillator enable */
#define OS32KCCTRL_OSC32K_EN32K          (1 << 2)  /* Bit 2:  32kHz Output enable */
#define OS32KCCTRL_OSC32K_EN1K           (1 << 3)  /* Bit 3:  1kHz Output enable */
#define OS32KCCTRL_OSC32K_RUNSTDBY       (1 << 6)  /* Bit 6:  Run in standby */
#define OS32KCCTRL_OSC32K_ONDEMAND       (1 << 7)  /* Bit 7:  On demand control */
#define OS32KCCTRL_OSC32K_STARTUP_SHIFT  (8)       /* Bits 8-10: Oscillator start-up time */
#define OS32KCCTRL_OSC32K_STARTUP_MASK       (7 << OS32KCCTRL_OSC32K_STARTUP_SHIFT)
#  define OS32KCCTRL_OSC32K_STARTUP(n)       ((n) << OS32KCCTRL_OSC32K_STARTUP_SHIFT)
#  define OS32KCCTRL_OSC32K_STARTUP_92US     (0 << OS32KCCTRL_OSC32K_STARTUP_SHIFT) /* 92탎 */
#  define OS32KCCTRL_OSC32K_STARTUP_122US    (1 << OS32KCCTRL_OSC32K_STARTUP_SHIFT) /* 122탎 */
#  define OS32KCCTRL_OSC32K_STARTUP_183US    (2 << OS32KCCTRL_OSC32K_STARTUP_SHIFT) /* 183탎 */
#  define OS32KCCTRL_OSC32K_STARTUP_305US    (3 << OS32KCCTRL_OSC32K_STARTUP_SHIFT) /* 305탎 */
#  define OS32KCCTRL_OSC32K_STARTUP_549US    (4 << OS32KCCTRL_OSC32K_STARTUP_SHIFT) /* 549탎 */
#  define OS32KCCTRL_OSC32K_STARTUP_1MS      (5 << OS32KCCTRL_OSC32K_STARTUP_SHIFT) /* 1038탎 */
#  define OS32KCCTRL_OSC32K_STARTUP_2MS      (6 << OS32KCCTRL_OSC32K_STARTUP_SHIFT) /* 2014탎 */
#  define OS32KCCTRL_OSC32K_STARTUP_4MS      (7 << OS32KCCTRL_OSC32K_STARTUP_SHIFT) /* 3967탎 */
#define OS32KCCTRL_OSC32K_WRTLOCK        (1 << 12)  /* Bit 12: Write lock */
#define OS32KCCTRL_OSC32K_CALIB_SHIFT    (16)       /* Bits 16-22: Oscillator calibration */
#define OS32KCCTRL_OSC32K_CALIB_MASK     (0x7f << OS32KCCTRL_OSC32K_CALIB_SHIFT)
#  define OS32KCCTRL_OSC32K_CALIB(n)     ((n) << OS32KCCTRL_OSC32K_CALIB_SHIFT)

/* 32kHz ultra low power internal oscillator control register */

#define OS32KCCTRL_OSCULP32K_CALIB_SHIFT (8)       /* Bits 0-12: Oscillator Calibration */
#define OS32KCCTRL_OSCULP32K_CALIB_MASK  (31 << OS32KCCTRL_OSCULP32K_CALIB_SHIFT)
#  define OS32KCCTRL_OSCULP32K_CALIB(n)  ((n) << OS32KCCTRL_OSCULP32K_CALIB_SHIFT)
#define OS32KCCTRL_OSCULP32K_WRTLOCK     (1 << 7)  /* Bit 7:  Write Lock */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMDL_CHIP_SAML_OS32KCCTRL_H */
