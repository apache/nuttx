/********************************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_sysctrl.h
 *
 *   Copyright (C) 2014, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *   Datasheet", 42129J–SAM–12/2013
 *   "Atmel SAM D21E / SAM D21G / SAM D21J SMART ARM-Based Microcontroller
 *   Datasheet", Atmel-42181E–SAM-D21_Datasheet–02/2015
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

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_SYSCTRL_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_SYSCTRL_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* SYSCTRL register offsets *****************************************************************/

#define SAM_SYSCTRL_INTENCLR_OFFSET     0x0000  /* Interrupt enable clear */
#define SAM_SYSCTRL_INTENSET_OFFSET     0x0004  /* Interrupt enable set */
#define SAM_SYSCTRL_INTFLAG_OFFSET      0x0008  /* Interrupt flag status and clear */
#define SAM_SYSCTRL_PCLKSR_OFFSET       0x000c  /* Power and clocks status */
#define SAM_SYSCTRL_XOSC_OFFSET         0x0010  /* External multi-purpose crystal oscillator control */
#define SAM_SYSCTRL_XOSC32K_OFFSET      0x0014  /* 32kHz external crystal oscillator control */
#define SAM_SYSCTRL_OSC32K_OFFSET       0x0018  /* 32kHz internal oscillator control */
#define SAM_SYSCTRL_OSCULP32K_OFFSET    0x001c  /* 32kHz ultra low power internal oscillator control */
#define SAM_SYSCTRL_OSC8M_OFFSET        0x0020  /* 8MHz internal oscillator control */
#define SAM_SYSCTRL_DFLLCTRL_OFFSET     0x0024  /* DFLL48M control */
#define SAM_SYSCTRL_DFLLVAL_OFFSET      0x0028  /* DFLL48M value */
#define SAM_SYSCTRL_DFLLMUL_OFFSET      0x002c  /* DFLL48M multiplier */
#define SAM_SYSCTRL_DFLLSYNC_OFFSET     0x0030  /* DFLL48M synchronization */
#define SAM_SYSCTRL_BOD33_OFFSET        0x0034  /* 3.3V brown-out detector control */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_SYSCTRL_VREG_OFFSET       0x003c  /* Voltage regulator system control */
#endif

#define SAM_SYSCTRL_VREF_OFFSET         0x0040  /* Voltage references system control */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_SYSCTRL_DPLLCTRLA_OFFSET  0x0044  /* DPLL Control A */
#  define SAM_SYSCTRL_DPLLRATIO_OFFSET  0x0048  /* DPLL ratio control */
#  define SAM_SYSCTRL_DPLLCTRLB_OFFSET  0x004c  /* DPLL Control B */
#  define SAM_SYSCTRL_DPLLSTATUS_OFFSET 0x0050  /* DPLL status */
#endif

/* SYSCTRL register addresses ***************************************************************/

#define SAM_SYSCTRL_INTENCLR            (SAM_SYSCTRL_BASE+SAM_SYSCTRL_INTENCLR_OFFSET)
#define SAM_SYSCTRL_INTENSET            (SAM_SYSCTRL_BASE+SAM_SYSCTRL_INTENSET_OFFSET)
#define SAM_SYSCTRL_INTFLAG             (SAM_SYSCTRL_BASE+SAM_SYSCTRL_INTFLAG_OFFSET)
#define SAM_SYSCTRL_PCLKSR              (SAM_SYSCTRL_BASE+SAM_SYSCTRL_PCLKSR_OFFSET)
#define SAM_SYSCTRL_XOSC                (SAM_SYSCTRL_BASE+SAM_SYSCTRL_XOSC_OFFSET)
#define SAM_SYSCTRL_XOSC32K             (SAM_SYSCTRL_BASE+SAM_SYSCTRL_XOSC32K_OFFSET)
#define SAM_SYSCTRL_OSC32K              (SAM_SYSCTRL_BASE+SAM_SYSCTRL_OSC32K_OFFSET)
#define SAM_SYSCTRL_OSCULP32K           (SAM_SYSCTRL_BASE+SAM_SYSCTRL_OSCULP32K_OFFSET)
#define SAM_SYSCTRL_OSC8M               (SAM_SYSCTRL_BASE+SAM_SYSCTRL_OSC8M_OFFSET)
#define SAM_SYSCTRL_DFLLCTRL            (SAM_SYSCTRL_BASE+SAM_SYSCTRL_DFLLCTRL_OFFSET)
#define SAM_SYSCTRL_DFLLVAL             (SAM_SYSCTRL_BASE+SAM_SYSCTRL_DFLLVAL_OFFSET)
#define SAM_SYSCTRL_DFLLMUL             (SAM_SYSCTRL_BASE+SAM_SYSCTRL_DFLLMUL_OFFSET)
#define SAM_SYSCTRL_DFLLSYNC            (SAM_SYSCTRL_BASE+SAM_SYSCTRL_DFLLSYNC_OFFSET)
#define SAM_SYSCTRL_BOD33               (SAM_SYSCTRL_BASE+SAM_SYSCTRL_BOD33_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_SYSCTRL_VREG              (SAM_SYSCTRL_BASE+SAM_SYSCTRL_VREG_OFFSET)
#endif

#define SAM_SYSCTRL_VREF                (SAM_SYSCTRL_BASE+SAM_SYSCTRL_VREF_OFFSET)

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SAM_SYSCTRL_DPLLCTRLA         (SAM_SYSCTRL_BASE+SAM_SYSCTRL_DPLLCTRLA_OFFSET)
#  define SAM_SYSCTRL_DPLLRATIO         (SAM_SYSCTRL_BASE+SAM_SYSCTRL_DPLLRATIO_OFFSET)
#  define SAM_SYSCTRL_DPLLCTRLB         (SAM_SYSCTRL_BASE+SAM_SYSCTRL_DPLLCTRLB_OFFSET)
#  define SAM_SYSCTRL_DPLLSTATUS        (SAM_SYSCTRL_BASE+SAM_SYSCTRL_DPLLSTATUS_OFFSET)
#endif

/* SYSCTRL register bit definitions *********************************************************/

/* Interrupt enable clear, Interrupt enable set, Interrupt flag status and clear, and
 * Power and clocks status registers.
 */

#define SYSCTRL_INT_XOSCRDY             (1 << 0)  /* Bit 0:  XOSC ready interrupt */
#define SYSCTRL_INT_XOSC32KRDY          (1 << 1)  /* Bit 1:  XOSC32K ready interrupt */
#define SYSCTRL_INT_OSC32KRDY           (1 << 2)  /* Bit 2:  OSC32K ready interrupt */
#define SYSCTRL_INT_OSC8MRDY            (1 << 3)  /* Bit 3:  OSC8M ready interrupt */
#define SYSCTRL_INT_DFLLRDY             (1 << 4)  /* Bit 4:  DFLL ready interrupt */
#define SYSCTRL_INT_DFLLOOB             (1 << 5)  /* Bit 5:  DFLL out of bounds interrupt */
#define SYSCTRL_INT_DFLLLCKF            (1 << 6)  /* Bit 6:  DFLL lock fine interrupt */
#define SYSCTRL_INT_DFLLLCKC            (1 << 7)  /* Bit 7:  DFLL lock coarse interrupt */
#define SYSCTRL_INT_DFLLRCS             (1 << 8)  /* Bit 8:  DFLL reference clock stopped interrupt */
#define SYSCTRL_INT_BOD33RDY            (1 << 9)  /* Bit 9:  BOD33 ready interrupt */
#define SYSCTRL_INT_BOD33DET            (1 << 10) /* Bit 10: BOD33 detection interrupt */
#define SYSCTRL_INT_B33SRDY             (1 << 11) /* Bit 11: BOD33 synchronization ready interrupt */

#if defined(CONFIG_ARCH_FAMILY_SAMD20)
#  define SYSCTRL_INT_BOD12RDY          (1 << 12) /* Bit 12: BOD12 ready interrupt */
#  define SYSCTRL_INT_BOD12DET          (1 << 13) /* Bit 13: BOD12 detection interrupt */
#  define SYSCTRL_INT_B12SRDY           (1 << 14) /* Bit 14: BOD12 synchronization ready interrupt */

#  define SYSCTRL_INT_ALL               (0x00007fff)
#elif defined(CONFIG_ARCH_FAMILY_SAMD21)
#  define SYSCTRL_INT_DPLLLCKR          (1 << 15) /* Bit 15: DPLL lock rise interrupt */
#  define SYSCTRL_INT_DPLLLCKF          (1 << 16) /* Bit 16: DPLL lock fall interrupt */
#  define SYSCTRL_INT_DPLLLTO           (1 << 17) /* Bit 17: DPLL lock timeout interrupt */

#  define SYSCTRL_INT_ALL               (0x00038fff)
#endif

/* External multi-purpose crystal oscillator control register */

#define SYSCTRL_XOSC_ENABLE             (1 << 1)  /* Bit 1:  Oscillator enable */
#define SYSCTRL_XOSC_XTALEN             (1 << 2)  /* Bit 2:  Crystal oscillator enable */
#define SYSCTRL_XOSC_RUNSTDBY           (1 << 6)  /* Bit 6:  Run in standby */
#define SYSCTRL_XOSC_ONDEMAND           (1 << 7)  /* Bit 7:  On demand control */
#define SYSCTRL_XOSC_GAIN_SHIFT         (8)       /* Bits 8-10: Oscillator gain */
#define SYSCTRL_XOSC_GAIN_MASK          (7 << SYSCTRL_XOSC_GAIN_SHIFT)
#  define SYSCTRL_XOSC_GAIN(n)          ((n) << SYSCTRL_XOSC_GAIN_SHIFT)
#  define SYSCTRL_XOSC_GAIN_2MHZ        (0 << SYSCTRL_XOSC_GAIN_SHIFT) /* 2MHz */
#  define SYSCTRL_XOSC_GAIN_4MHZ        (1 << SYSCTRL_XOSC_GAIN_SHIFT) /* 4MHz */
#  define SYSCTRL_XOSC_GAIN_8MHZ        (2 << SYSCTRL_XOSC_GAIN_SHIFT) /* 8MHz */
#  define SYSCTRL_XOSC_GAIN_16MHZ       (3 << SYSCTRL_XOSC_GAIN_SHIFT) /* 16MHz */
#  define SYSCTRL_XOSC_GAIN_30MHZ       (4 << SYSCTRL_XOSC_GAIN_SHIFT) /* 30MHz */
#define SYSCTRL_XOSC_AMPGC              (1 << 11) /* Bit 11: Automatic amplitude gain control */
#define SYSCTRL_XOSC_STARTUP_SHIFT      (12)      /* Bits 12-15: Start-up time */
#define SYSCTRL_XOSC_STARTUP_MASK       (15 << SYSCTRL_XOSC_STARTUP_SHIFT)
#  define SYSCTRL_XOSC_STARTUP(n)       ((n) << SYSCTRL_XOSC_STARTUP_SHIFT)
#  define SYSCTRL_XOSC_STARTUP_31US     (0 << SYSCTRL_XOSC_STARTUP_SHIFT)  /* 31µs */
#  define SYSCTRL_XOSC_STARTUP_61US     (1 << SYSCTRL_XOSC_STARTUP_SHIFT)  /* 61µs */
#  define SYSCTRL_XOSC_STARTUP_122US    (2 << SYSCTRL_XOSC_STARTUP_SHIFT)  /* 122µs */
#  define SYSCTRL_XOSC_STARTUP_244US    (3 << SYSCTRL_XOSC_STARTUP_SHIFT)  /* 244µs */
#  define SYSCTRL_XOSC_STARTUP_488US    (4 << SYSCTRL_XOSC_STARTUP_SHIFT)  /* 488µs */
#  define SYSCTRL_XOSC_STARTUP_977US    (5 << SYSCTRL_XOSC_STARTUP_SHIFT)  /* 977µs */
#  define SYSCTRL_XOSC_STARTUP_2MS      (6 << SYSCTRL_XOSC_STARTUP_SHIFT)  /* 1953µs */
#  define SYSCTRL_XOSC_STARTUP_4MS      (7 << SYSCTRL_XOSC_STARTUP_SHIFT)  /* 3906µs */
#  define SYSCTRL_XOSC_STARTUP_8MS      (8 << SYSCTRL_XOSC_STARTUP_SHIFT)  /* 7813µs */
#  define SYSCTRL_XOSC_STARTUP_16MS     (9 << SYSCTRL_XOSC_STARTUP_SHIFT)  /* 15625µs */
#  define SYSCTRL_XOSC_STARTUP_31MS     (10 << SYSCTRL_XOSC_STARTUP_SHIFT) /* 31250µs */
#  define SYSCTRL_XOSC_STARTUP_63MS     (11 << SYSCTRL_XOSC_STARTUP_SHIFT) /* 62500µs */
#  define SYSCTRL_XOSC_STARTUP_125MS    (12 << SYSCTRL_XOSC_STARTUP_SHIFT) /* 125000µs */
#  define SYSCTRL_XOSC_STARTUP_250MS    (13 << SYSCTRL_XOSC_STARTUP_SHIFT) /* 250000µs */
#  define SYSCTRL_XOSC_STARTUP_500MS    (14 << SYSCTRL_XOSC_STARTUP_SHIFT) /* 500000µs */
#  define SYSCTRL_XOSC_STARTUP_1S       (15 << SYSCTRL_XOSC_STARTUP_SHIFT) /* 1000000µs */

/* 32kHz external crystal oscillator control register */

#define SYSCTRL_XOSC32K_ENABLE          (1 << 1)  /* Bit 1:  Oscillator enable */
#define SYSCTRL_XOSC32K_XTALEN          (1 << 2)  /* Bit 2:  Crystal oscillator enable */
#define SYSCTRL_XOSC32K_EN32K           (1 << 3)  /* Bit 3:  32kHz Output enable */
#define SYSCTRL_XOSC32K_EN1K            (1 << 4)  /* Bit 4:  1kHz Output enable */
#define SYSCTRL_XOSC32K_AAMPEN          (1 << 5)  /* Bit 5:  Automatic amplitude control enable */
#define SYSCTRL_XOSC32K_RUNSTDBY        (1 << 6)  /* Bit 6:  Run in standby */
#define SYSCTRL_XOSC32K_ONDEMAND        (1 << 7)  /* Bit 7:  On demand control */
#define SYSCTRL_XOSC32K_STARTUP_SHIFT   (8)       /* Bits 8-10: Oscillator start-up time */
#define SYSCTRL_XOSC32K_STARTUP_MASK    (7 << SYSCTRL_XOSC32K_STARTUP_SHIFT)
#  define SYSCTRL_XOSC32K_STARTUP(n)    ((n) << SYSCTRL_XOSC32K_STARTUP_SHIFT)
#  define SYSCTRL_XOSC32K_STARTUP_122US (0 << SYSCTRL_XOSC32K_STARTUP_SHIFT) /* 122µs */
#  define SYSCTRL_XOSC32K_STARTUP_1MS   (1 << SYSCTRL_XOSC32K_STARTUP_SHIFT) /* 1068µs */
#  define SYSCTRL_XOSC32K_STARTUP_63MS  (2 << SYSCTRL_XOSC32K_STARTUP_SHIFT) /* 62592µs */
#  define SYSCTRL_XOSC32K_STARTUP_125MS (3 << SYSCTRL_XOSC32K_STARTUP_SHIFT) /* 125092µs */
#  define SYSCTRL_XOSC32K_STARTUP_500MS (4 << SYSCTRL_XOSC32K_STARTUP_SHIFT) /* 500092µs */
#  define SYSCTRL_XOSC32K_STARTUP_1S    (5 << SYSCTRL_XOSC32K_STARTUP_SHIFT) /* 1000092µs */
#  define SYSCTRL_XOSC32K_STARTUP_2S    (6 << SYSCTRL_XOSC32K_STARTUP_SHIFT) /* 2000092µs */
#  define SYSCTRL_XOSC32K_STARTUP_4S    (7 << SYSCTRL_XOSC32K_STARTUP_SHIFT) /* 4000092µs */
#define SYSCTRL_XOSC32K_WRTLOCK         (1 << 12)  /* Bit 12: Write lock */

/* 32kHz internal oscillator control register */

#define SYSCTRL_OSC32K_ENABLE           (1 << 1)  /* Bit 1:  Oscillator enable */
#define SYSCTRL_OSC32K_EN32K            (1 << 2)  /* Bit 2:  32kHz Output enable */

#ifdef CONFIG_ARCH_FAMILY_SAMD20
#  define SYSCTRL_OSC32K_EN1K           (1 << 3)  /* Bit 3:  1kHz Output enable */
#endif

#define SYSCTRL_OSC32K_RUNSTDBY         (1 << 6)  /* Bit 6:  Run in standby */
#define SYSCTRL_OSC32K_ONDEMAND         (1 << 7)  /* Bit 7:  On demand control */
#define SYSCTRL_OSC32K_STARTUP_SHIFT    (8)       /* Bits 8-10: Oscillator start-up time */
#define SYSCTRL_OSC32K_STARTUP_MASK     (7 << SYSCTRL_OSC32K_STARTUP_SHIFT)
#  define SYSCTRL_OSC32K_STARTUP(n)     ((n) << SYSCTRL_OSC32K_STARTUP_SHIFT)
#  define SYSCTRL_OSC32K_STARTUP_92US   (0 << SYSCTRL_OSC32K_STARTUP_SHIFT) /* 92µs */
#  define SYSCTRL_OSC32K_STARTUP_122US  (1 << SYSCTRL_OSC32K_STARTUP_SHIFT) /* 122µs */
#  define SYSCTRL_OSC32K_STARTUP_183US  (2 << SYSCTRL_OSC32K_STARTUP_SHIFT) /* 183µs */
#  define SYSCTRL_OSC32K_STARTUP_305US  (3 << SYSCTRL_OSC32K_STARTUP_SHIFT) /* 305µs */
#  define SYSCTRL_OSC32K_STARTUP_549US  (4 << SYSCTRL_OSC32K_STARTUP_SHIFT) /* 549µs */
#  define SYSCTRL_OSC32K_STARTUP_1MS    (5 << SYSCTRL_OSC32K_STARTUP_SHIFT) /* 1038µs */
#  define SYSCTRL_OSC32K_STARTUP_2MS    (6 << SYSCTRL_OSC32K_STARTUP_SHIFT) /* 2014µs */
#  define SYSCTRL_OSC32K_STARTUP_4MS    (7 << SYSCTRL_OSC32K_STARTUP_SHIFT) /* 3967µs */
#define SYSCTRL_OSC32K_WRTLOCK          (1 << 12)  /* Bit 12: Write lock */
#define SYSCTRL_OSC32K_CALIB_SHIFT      (16)       /* Bits 16-22: Oscillator calibration */
#define SYSCTRL_OSC32K_CALIB_MASK       (0x7f << SYSCTRL_OSC32K_CALIB_SHIFT)
#  define SYSCTRL_OSC32K_CALIB(n)       ((n) << SYSCTRL_OSC32K_CALIB_SHIFT)

/* 32kHz ultra low power internal oscillator control register */

#define SYSCTRL_OSCULP32K_CALIB_SHIFT   (0)       /* Bits 0-4: Oscillator Calibration */
#define SYSCTRL_OSCULP32K_CALIB_MASK    (0x7f << SYSCTRL_OSCULP32K_CALIB_SHIFT)
#  define SYSCTRL_OSCULP32K_CALIB(n)    ((n) << SYSCTRL_OSCULP32K_CALIB_SHIFT)
#define SYSCTRL_OSCULP32K_WRTLOCK       (1 << 7)  /* Bit 7:  Write Lock */

/* 8MHz internal oscillator control register */

#define SYSCTRL_OSC8M_ENABLE            (1 << 1)  /* Bit 1:  Oscillator enable */
#define SYSCTRL_OSC8M_RUNSTDBY          (1 << 6)  /* Bit 6:  Run in standby */
#define SYSCTRL_OSC8M_ONDEMAND          (1 << 7)  /* Bit 7:  On demand control */
#define SYSCTRL_OSC8M_PRESC_SHIFT       (8)       /* Bits 8-9: Oscillator prescaler */
#define SYSCTRL_OSC8M_PRESC_MASK        (3 << SYSCTRL_OSC8M_PRESC_SHIFT)
#  define SYSCTRL_OSC8M_PRESC(n)        ((n) << SYSCTRL_OSC8M_PRESC_SHIFT)
#  define SYSCTRL_OSC8M_PRESC_DIV1      (0 << SYSCTRL_OSC8M_PRESC_SHIFT) /* 1 */
#  define SYSCTRL_OSC8M_PRESC_DIV2      (1 << SYSCTRL_OSC8M_PRESC_SHIFT) /* 2 */
#  define SYSCTRL_OSC8M_PRESC_DIV3      (2 << SYSCTRL_OSC8M_PRESC_SHIFT) /* 4 */
#  define SYSCTRL_OSC8M_PRESC_DIV8      (3 << SYSCTRL_OSC8M_PRESC_SHIFT) /* 8 */
#define SYSCTRL_OSC8M_CALIB_SHIFT       (16)      /* Bits 16-27: Oscillator calibration */
#define SYSCTRL_OSC8M_CALIB_MASK        (0xfff << SYSCTRL_OSC8M_CALIB_SHIFT)
#  define SYSCTRL_OSC8M_CALIB(n)        ((n) << SYSCTRL_OSC8M_CALIB_SHIFT)
#define SYSCTRL_OSC8M_FRANGE_SHIFT      (30)      /* Bits 30-31: Oscillator frequency range */
#define SYSCTRL_OSC8M_FRANGE_MASK       (3 << SYSCTRL_OSC8M_FRANGE_SHIFT)
#  define SYSCTRL_OSC8M_FRANGE(n)       ((n) << SYSCTRL_OSC8M_FRANGE_SHIFT)
#  define SYSCTRL_OSC8M_FRANGE_LOW      (0 << SYSCTRL_OSC8M_FRANGE_SHIFT) /* 4 to 6MHz */
#  define SYSCTRL_OSC8M_FRANGE_MEDLOW   (1 << SYSCTRL_OSC8M_FRANGE_SHIFT) /* 6 to 8MHz */
#  define SYSCTRL_OSC8M_FRANGE_MEDHI    (2 << SYSCTRL_OSC8M_FRANGE_SHIFT) /* 8 to 11MHz */
#  define SYSCTRL_OSC8M_FRANGE_HI       (3 << SYSCTRL_OSC8M_FRANGE_SHIFT) /* 11 to 15MHz */

/* DFLL48M control register */

#define SYSCTRL_DFLLCTRL_ENABLE         (1 << 1)  /* Bit 1:  DFLL enable */
#define SYSCTRL_DFLLCTRL_MODE           (1 << 2)  /* Bit 2:  Operating mode selection */
#define SYSCTRL_DFLLCTRL_STABLE         (1 << 3)  /* Bit 3:  Stable DFLL frequency */
#define SYSCTRL_DFLLCTRL_LLAW           (1 << 4)  /* Bit 4:  Lose lock after wake */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SYSCTRL_DFLLCTRL_USBCRM       (1 << 5)  /* Bit 5:  USB clock recovery mode */
#  define SYSCTRL_DFLLCTRL_RUNSTDBY     (1 << 6)  /* Bit 6:  Run in standby */
#endif

#define SYSCTRL_DFLLCTRL_ONDEMAND       (1 << 7)  /* Bit 7:  On demand control */
#define SYSCTRL_DFLLCTRL_CCDIS          (1 << 8)  /* Bit 8:  Chill cycle disable */
#define SYSCTRL_DFLLCTRL_QLDIS          (1 << 9)  /* Bit 9:  Quick Lock Disable */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SYSCTRL_DFLLCTRL_BPLCKC       (1 << 10) /* Bit 10: Bypass coards lock */
#  define SYSCTRL_DFLLCTRL_WAITLOCK     (1 << 11) /* Bit 11: Wait lock */
#endif

/* DFLL48M value register */

#define SYSCTRL_DFLLVAL_FINE_SHIFT      (0)       /* Bits 0-9: Fine value */
#define SYSCTRL_DFLLVAL_FINE_MASK       (0x3ff << SYSCTRL_DFLLVAL_FINE_SHIFT)
#  define SYSCTRL_DFLLVAL_FINE(n)       ((n) << SYSCTRL_DFLLVAL_FINE_SHIFT)
#define SYSCTRL_DFLLVAL_COARSE_SHIFT    (10)      /* Bits 10-15: Coarse value */
#define SYSCTRL_DFLLVAL_COARSE_MASK     (0x3f << SYSCTRL_DFLLVAL_COARSE_SHIFT)
#  define SYSCTRL_DFLLVAL_COARSE(n)     ((n) << SYSCTRL_DFLLVAL_COARSE_SHIFT)
#define SYSCTRL_DFLLVAL_DIFF_SHIFT      (16)      /* Bits 16-31: Multiplication ratio difference */
#define SYSCTRL_DFLLVAL_DIFF_MASK       (0xffff << SYSCTRL_DFLLVAL_DIFF_SHIFT)
#  define SYSCTRL_DFLLVAL_DIFF(n)       ((n) << SYSCTRL_DFLLVAL_DIFF_SHIFT)

/* DFLL48M multiplier register */

#define SYSCTRL_DFLLMUL_MUL_SHIFT       (0)       /* Bits 0-15: DFLL multiply factor */
#define SYSCTRL_DFLLMUL_MUL_MASK        (0xffff << SYSCTRL_DFLLMUL_MUL_SHIFT)
#  define SYSCTRL_DFLLMUL_MUL(n)        ((n) << SYSCTRL_DFLLMUL_MUL_SHIFT)
#define SYSCTRL_DFLLMUL_FSTEP_SHIFT     (16)      /* Bits 16-25: Fine maximum step */
#define SYSCTRL_DFLLMUL_FSTEP_MASK      (0x3ff << SYSCTRL_DFLLMUL_FSTEP_SHIFT)
#  define SYSCTRL_DFLLMUL_FSTEP(n)      ((n) << SYSCTRL_DFLLMUL_FSTEP_SHIFT)
#define SYSCTRL_DFLLMUL_CSTEP_SHIFT     (26)      /* Bits 26-31: Coarse maximum step */
#define SYSCTRL_DFLLMUL_CSTEP_MASK      (0x3f << SYSCTRL_DFLLMUL_CSTEP_SHIFT)
#  define SYSCTRL_DFLLMUL_CSTEP(n)      ((n) << SYSCTRL_DFLLMUL_CSTEP_SHIFT)

/* DFLL48M synchronization register */

#define SYSCTRL_DFLLSYNC_READREQ        (1 << 7)  /* Bit 7: Read request */

/* 3.3V brown-out detector control register */

#define SYSCTRL_BOD33_ENABLE            (1 << 1)  /* Bit 1: Enable */
#define SYSCTRL_BOD33_HYST              (1 << 2)  /* Bit 2: Hysteresis */
#define SYSCTRL_BOD33_ACTION_SHIFT      (3)       /* Bits 3-4: BOD33 action */
#define SYSCTRL_BOD33_ACTION_MASK       (3 << SYSCTRL_BOD33_ACTION_SHIFT)
#  define SYSCTRL_BOD33_ACTION(n)       ((n) << SYSCTRL_BOD33_ACTION_SHIFT)
#  define SYSCTRL_BOD33_ACTION_NONE     (0 << SYSCTRL_BOD33_ACTION_SHIFT) /* No action */
#  define SYSCTRL_BOD33_ACTION_RESET    (1 << SYSCTRL_BOD33_ACTION_SHIFT) /* BOD33 generates reset */
#  define SYSCTRL_BOD33_ACTION_INTR     (2 << SYSCTRL_BOD33_ACTION_SHIFT) /* BOD33 generates interrupt */
#define SYSCTRL_BOD33_RUNSTDBY          (1 << 6)  /* Bit 6: Run in standby */
#define SYSCTRL_BOD33_MODE              (1 << 8)  /* Bit 8: Operation mode */
#define SYSCTRL_BOD33_CEN               (1 << 9)  /* Bit 9: Clock enable */
#define SYSCTRL_BOD33_PSEL_SHIFT        (12)      /* Bits 12-15: Prescaler select */
#define SYSCTRL_BOD33_PSEL_MASK         (15 << SYSCTRL_BOD33_PSEL_SHIFT)
#  define SYSCTRL_BOD33_PSEL(n)         ((n) << SYSCTRL_BOD33_PSEL_SHIFT)
#  define SYSCTRL_BOD33_PSEL_DIV2       (0 << SYSCTRL_BOD33_PSEL_SHIFT)  /* Divide clock by 2 */
#  define SYSCTRL_BOD33_PSEL_DIV4       (1 << SYSCTRL_BOD33_PSEL_SHIFT)  /* Divide clock by 4 */
#  define SYSCTRL_BOD33_PSEL_DIV8       (2 << SYSCTRL_BOD33_PSEL_SHIFT)  /* Divide clock by 8 */
#  define SYSCTRL_BOD33_PSEL_DIV16      (3 << SYSCTRL_BOD33_PSEL_SHIFT)  /* Divide clock by 16 */
#  define SYSCTRL_BOD33_PSEL_DIV32      (4 << SYSCTRL_BOD33_PSEL_SHIFT)  /* Divide clock by 32 */
#  define SYSCTRL_BOD33_PSEL_DIV64      (5 << SYSCTRL_BOD33_PSEL_SHIFT)  /* Divide clock by 64 */
#  define SYSCTRL_BOD33_PSEL_DIV128     (6 << SYSCTRL_BOD33_PSEL_SHIFT)  /* Divide clock by 128 */
#  define SYSCTRL_BOD33_PSEL_DIV256     (7 << SYSCTRL_BOD33_PSEL_SHIFT)  /* Divide clock by 256 */
#  define SYSCTRL_BOD33_PSEL_DIV512     (8 << SYSCTRL_BOD33_PSEL_SHIFT)  /* Divide clock by 512 */
#  define SYSCTRL_BOD33_PSEL_DIV1K      (9 << SYSCTRL_BOD33_PSEL_SHIFT)  /* Divide clock by 1024 */
#  define SYSCTRL_BOD33_PSEL_DIV2K      (10 << SYSCTRL_BOD33_PSEL_SHIFT) /* Divide clock by 2048 */
#  define SYSCTRL_BOD33_PSEL_DIV4K      (11 << SYSCTRL_BOD33_PSEL_SHIFT) /* Divide clock by 4096 */
#  define SYSCTRL_BOD33_PSEL_DIV8K      (12 << SYSCTRL_BOD33_PSEL_SHIFT) /* Divide clock by 8192 */
#  define SYSCTRL_BOD33_PSEL_DIV16K     (13 << SYSCTRL_BOD33_PSEL_SHIFT) /* Divide clock by 16384 */
#  define SYSCTRL_BOD33_PSEL_DIV32K     (14 << SYSCTRL_BOD33_PSEL_SHIFT) /* Divide clock by 32768 */
#  define SYSCTRL_BOD33_PSEL_DIV64K     (15 << SYSCTRL_BOD33_PSEL_SHIFT) /* Divide clock by 65536 */
#define SYSCTRL_BOD33_LEVEL_SHIFT       (16)      /* Bits 16-21: BOD33 threshold level */
#define SYSCTRL_BOD33_LEVEL_MASK        (0x3f << SYSCTRL_BOD33_LEVEL_SHIFT)
#  define SYSCTRL_BOD33_LEVEL(n)        ((n) << SYSCTRL_BOD33_LEVEL_SHIFT)

/* Voltage regulator system control register */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SYSCTRL_VREG_RUNSTDBY         (1 << 6)  /* Bit 6:  Run in Standby */
#  define SYSCTRL_VREG_FORCELDO         (1 << 13) /* Bit 13: Force LDO voltage regulator */
#endif

/* Voltage references system control register */

#define SYSCTRL_VREF_TSEN               (1 << 1)  /* Bit 1:  Temperature sensor enable */
#define SYSCTRL_VREF_BGOUTEN            (1 << 2)  /* Bit 2:  Bandgap output enable */
#define SYSCTRL_VREF_CALIB_SHIFT        (16)      /* Bits 16-26: Bandgap voltage generator calibration */
#define SYSCTRL_VREF_CALIB_MASK         (0x7ff << SYSCTRL_VREF_CALIB_SHIFT)
#  define SYSCTRL_VREF_CALIB(n)         ((n) << SYSCTRL_VREF_CALIB_SHIFT)

/* DPLL Control A register */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SYSCTRL_DPLLCTRLA_ENABLE      (1 << 1)  /* Bit 1: DPLL Enable */
#  define SYSCTRL_DPLLCTRLA_RUNSTDBY    (1 << 6)  /* Bit 6: Run in Standby */
#  define SYSCTRL_DPLLCTRLA_ONDEMAND    (1 << 7)  /* Bit 7: On Demand Clock Activation */
#endif

/* DPLL ratio control registers */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SYSCTRL_DPLLRATIO_LDR_SHIFT     (0)     /* Bits 0-11:  Loop Divider Ratio */
#  define SYSCTRL_DPLLRATIO_LDR_MASK      (0xfff << SYSCTRL_DPLLRATIO_LDR_SHIFT)
#    define SYSCTRL_DPLLRATIO_LDR(n)      ((uint32_t)(n) << SYSCTRL_DPLLRATIO_LDR_SHIFT)
#  define SYSCTRL_DPLLRATIO_LDRFRAC_SHIFT (16)    /* Bits 16-19: Loop Divider Ratio Fractional Part */
#  define SYSCTRL_DPLLRATIO_LDRFRAC_MASK  (15 << SYSCTRL_DPLLRATIO_LDRFRAC_SHIFT)
#    define SYSCTRL_DPLLRATIO_LDRFRAC(n)  ((uint32_t)(n) << SYSCTRL_DPLLRATIO_LDRFRAC_SHIFT)
#endif

/* DPLL Control B register */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SYSCTRL_DPLLCTRLB_FILTER_SHIFT      (0)       /* Bits 0-1: Proportional Integral Filter Selection */
#  define SYSCTRL_DPLLCTRLB_FILTER_MASK       (3 << SYSCTRL_DPLLCTRLB_FILTER_SHIFT)
#    define SYSCTRL_DPLLCTRLB_FILTER_DEFAULT  (0 << SYSCTRL_DPLLCTRLB_FILTER_SHIFT) /* Default filter mode */
#    define SYSCTRL_DPLLCTRLB_FILTER_LBFILT   (1 << SYSCTRL_DPLLCTRLB_FILTER_SHIFT) /* Low bandwidth filter */
#    define SYSCTRL_DPLLCTRLB_FILTER_HBFILT   (2 << SYSCTRL_DPLLCTRLB_FILTER_SHIFT) /* High bandwidth filter */
#    define SYSCTRL_DPLLCTRLB_FILTER_HDFILT   (3 << SYSCTRL_DPLLCTRLB_FILTER_SHIFT) /* High damping filter */
#  define SYSCTRL_DPLLCTRLB_LPEN              (1 << 2)  /* Bit 2:  Low-Power Enable */
#  define SYSCTRL_DPLLCTRLB_WUF               (1 << 3)  /* Bit 3:  Wake Up Fast */
#  define SYSCTRL_DPLLCTRLB_REFCLK_SHIFT      (4)       /* Bits 4-5: Reference Clock Selection */
#  define SYSCTRL_DPLLCTRLB_REFCLK_MASK       (3 << SYSCTRL_DPLLCTRLB_REFCLK_SHIFT)
#    define SYSCTRL_DPLLCTRLB_REFCLK_XOSC32   (0 << SYSCTRL_DPLLCTRLB_REFCLK_SHIFT) /* XOSC32 clock reference */
#    define SYSCTRL_DPLLCTRLB_REFCLK_XOSC     (1 << SYSCTRL_DPLLCTRLB_REFCLK_SHIFT) /* XOSC clock reference */
#    define SYSCTRL_DPLLCTRLB_REFCLK_GCLKDPLL (2 << SYSCTRL_DPLLCTRLB_REFCLK_SHIFT) /* GCLK_DPLL clock reference */
#  define SYSCTRL_DPLLCTRLB_LTIME_SHIFT       (8)    /* Bits 8-10: Lock Time */
#  define SYSCTRL_DPLLCTRLB_LTIME_MASK        (7 << SYSCTRL_DPLLCTRLB_LTIME_SHIFT)
#    define SYSCTRL_DPLLCTRLB_LTIME_DEFAULT   (0 << SYSCTRL_DPLLCTRLB_LTIME_SHIFT) /* No time-out */
#    define SYSCTRL_DPLLCTRLB_LTIME_8MS       (4 << SYSCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no lock within 8 ms */
#    define SYSCTRL_DPLLCTRLB_LTIME_9MS       (5 << SYSCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no lock within 9 ms */
#    define SYSCTRL_DPLLCTRLB_LTIME_10MS      (6 << SYSCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no lock within 10 ms */
#    define SYSCTRL_DPLLCTRLB_LTIME_11MS      (7 << SYSCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no lock within 11 ms */
#  define SYSCTRL_DPLLCTRLB_LBYPASS           (1 << 12)  /* Bit 12:  Lock Bypass */
#  define SYSCTRL_DPLLCTRLB_DIV_SHIFT         (16)       /* Bits 16-26:  */
#  define SYSCTRL_DPLLCTRLB_DIV_MASK          (0x7ff << SYSCTRL_DPLLCTRLB_DIV_SHIFT)
#    define SYSCTRL_DPLLCTRLB_DIV(n)          ((uint32_t)(n) << SYSCTRL_DPLLCTRLB_DIV_SHIFT)
#endif

/* DPLL status register */

#ifdef CONFIG_ARCH_FAMILY_SAMD21
#  define SYSCTRL_DPLLSTATUS_LOCK       (1 << 0)  /* Bit 0:  DPLL Lock Status */
#  define SYSCTRL_DPLLSTATUS_CLKRDY     (1 << 1)  /* Bit 1:  Output Clock Ready */
#  define SYSCTRL_DPLLSTATUS_ENABLE     (1 << 2)  /* Bit 2:  DPLL Enable */
#  define SYSCTRL_DPLLSTATUS_DIV        (1 << 3)  /* Bit 3:  Divider Enable */
#endif

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD20 || CONFIG_ARCH_FAMILY_SAMD21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_SYSCTRL_H */
