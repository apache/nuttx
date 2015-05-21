/********************************************************************************************
 * arch/arm/src/samdl/chip/saml_oscctrl.h
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

#ifndef __ARCH_ARM_SRC_SAMDL_CHIP_SAML_OSCCTRL_H
#define __ARCH_ARM_SRC_SAMDL_CHIP_SAML_OSCCTRL_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* OSCCTRL register offsets *****************************************************************/

#define SAM_OSCCTRL_INTENCLR_OFFSET      0x0000  /* Interrupt enable clear */
#define SAM_OSCCTRL_INTENSET_OFFSET      0x0004  /* Interrupt enable set */
#define SAM_OSCCTRL_INTFLAG_OFFSET       0x0008  /* Interrupt flag status and clear */
#define SAM_OSCCTRL_STATUS_OFFSET        0x000c  /* Status */
#define SAM_OSCCTRL_XOSCCTRL_OFFSET      0x0010  /* External multi-purpose crystal oscillator control */
#define SAM_OSCCTRL_OSC16MCTRL_OFFSET    0x0014  /* 16MHz internal oscillator control */
#define SAM_OSCCTRL_DFLLCTRL_OFFSET      0x0018  /* DFLL48M control */
#define SAM_OSCCTRL_DFLLVAL_OFFSET       0x001c  /* DFLL48M value */
#define SAM_OSCCTRL_DFLLMUL_OFFSET       0x0020  /* DFLL48M multiplier */
#define SAM_OSCCTRL_DFLLSYNC_OFFSET      0x0024  /* DFLL48M synchronization */
#define SAM_OSCCTRL_DPLLCTRLA_OFFSET     0x0028  /* DPLL control A */
#define SAM_OSCCTRL_DPLLRATIO_OFFSET     0x002c  /* DPLL ratio control */
#define SAM_OSCCTRL_DPLLCTRLB_OFFSET     0x0030  /* DPLL control B */
#define SAM_OSCCTRL_DPLLPRESC_OFFSET     0x0034  /* DPLL prescaler */
#define SAM_OSCCTRL_DPLLSYNCBUSY_OFFSET  0x0038  /* DPLL synchronization busy */
#define SAM_OSCCTRL_DPLLSTATUS_OFFSET    0x003c  /* DPLL status */

/* OSCCTRL register addresses ***************************************************************/

#define SAM_OSCCTRL_INTENCLR             (SAM_OSCCTRL_BASE+SAM_OSCCTRL_INTENCLR_OFFSET)
#define SAM_OSCCTRL_INTENSET             (SAM_OSCCTRL_BASE+SAM_OSCCTRL_INTENSET_OFFSET)
#define SAM_OSCCTRL_INTFLAG              (SAM_OSCCTRL_BASE+SAM_OSCCTRL_INTFLAG_OFFSET)
#define SAM_OSCCTRL_STATUS               (SAM_OSCCTRL_BASE+SAM_OSCCTRL_STATUS_OFFSET)
#define SAM_OSCCTRL_XOSCCTRL             (SAM_OSCCTRL_BASE+SAM_OSCCTRL_XOSCCTRL_OFFSET)
#define SAM_OSCCTRL_OSC16MCTRL           (SAM_OSCCTRL_BASE+SAM_OSCCTRL_OSC16MCTRL_OFFSET)
#define SAM_OSCCTRL_DFLLCTRL             (SAM_OSCCTRL_BASE+SAM_OSCCTRL_DFLLCTRL_OFFSET)
#define SAM_OSCCTRL_DFLLVAL              (SAM_OSCCTRL_BASE+SAM_OSCCTRL_DFLLVAL_OFFSET)
#define SAM_OSCCTRL_DFLLMUL              (SAM_OSCCTRL_BASE+SAM_OSCCTRL_DFLLMUL_OFFSET)
#define SAM_OSCCTRL_DFLLSYNC             (SAM_OSCCTRL_BASE+SAM_OSCCTRL_DFLLSYNC_OFFSET)
#define SAM_OSCCTRL_DPLLCTRLA            (SAM_OSCCTRL_BASE+SAM_OSCCTRL_DPLLCTRLA_OFFSET)
#define SAM_OSCCTRL_DPLLRATIO            (SAM_OSCCTRL_BASE+SAM_OSCCTRL_DPLLRATIO_OFFSET)
#define SAM_OSCCTRL_DPLLCTRLB            (SAM_OSCCTRL_BASE+SAM_OSCCTRL_DPLLCTRLB_OFFSET)
#define SAM_OSCCTRL_DPLLPRESC            (SAM_OSCCTRL_BASE+SAM_OSCCTRL_DPLLPRESC_OFFSET)
#define SAM_OSCCTRL_DPLLSYNCBUSY         (SAM_OSCCTRL_BASE+SAM_OSCCTRL_DPLLSYNCBUSY_OFFSET)
#define SAM_OSCCTRL_DPLLSTATUS           (SAM_OSCCTRL_BASE+SAM_OSCCTRL_DPLLSTATUS_OFFSET)

/* OSCCTRL register bit definitions *********************************************************/

/* Interrupt enable clear, Interrupt enable set, Interrupt flag status and clear, and
 * Status registers.
 */

#define OSCCTRL_INT_XOSCRDY              (1 << 0)  /* Bit 0:  XOSC ready */
#define OSCCTRL_INT_OSC16MRDY            (1 << 4)  /* Bit 4:  OSC16M ready */
#define OSCCTRL_INT_DFLLRDY              (1 << 8)  /* Bit 8:  DFLL ready */
#define OSCCTRL_INT_DFLLOOB              (1 << 9)  /* Bit 9:  DFLL out of bounds */
#define OSCCTRL_INT_DFLLLCKF             (1 << 10) /* Bit 10: DFLL lock fine */
#define OSCCTRL_INT_DFLLLCKC             (1 << 11) /* Bit 11: DFLL lock coarse */
#define OSCCTRL_INT_DFLLRCS              (1 << 12) /* Bit 12: DFLL reference clock stopped */
#define OSCCTRL_INT_DPLLLCKR             (1 << 16) /* Bit 16: DPLL lock rise */
#define OSCCTRL_INT_DPLLLCKF             (1 << 17) /* Bit 17: DPLL lock fall */
#define OSCCTRL_INT_DPLLLTP              (1 << 18) /* Bit 18: DPLL lock timeout */
#define OSCCTRL_INT_DPLLDRTO             (1 << 19) /* Bit 19: DPLL loop divider ratio update complete */

#define OSCCTRL_INT_ALL                  (0x000f1f11)

/* External multi-purpose crystal oscillator control register */

#define OSCCTRL_XOSCCTRL_ENABLE          (1 << 1)  /* Bit 1:  Oscillator enable */
#define OSCCTRL_XOSCCTRL_XTALEN          (1 << 2)  /* Bit 2:  Crystal oscillator enable */
#define OSCCTRL_XOSCCTRL_RUNSTDBY        (1 << 6)  /* Bit 6:  Run in standby */
#define OSCCTRL_XOSCCTRL_ONDEMAND        (1 << 7)  /* Bit 7:  On demand control */
#define OSCCTRL_XOSCCTRL_GAIN_SHIFT      (8)       /* Bits 8-10: Oscillator gain */
#define OSCCTRL_XOSCCTRL_GAIN_MASK       (7 << OSCCTRL_XOSCCTRL_GAIN_SHIFT)
#  define OSCCTRL_XOSCCTRL_GAIN(n)       ((n) << OSCCTRL_XOSCCTRL_GAIN_SHIFT)
#  define OSCCTRL_XOSCCTRL_GAIN_2MHZ     (0 << OSCCTRL_XOSCCTRL_GAIN_SHIFT) /* 2MHz */
#  define OSCCTRL_XOSCCTRL_GAIN_4MHZ     (1 << OSCCTRL_XOSCCTRL_GAIN_SHIFT) /* 4MHz */
#  define OSCCTRL_XOSCCTRL_GAIN_8MHZ     (2 << OSCCTRL_XOSCCTRL_GAIN_SHIFT) /* 8MHz */
#  define OSCCTRL_XOSCCTRL_GAIN_16MHZ    (3 << OSCCTRL_XOSCCTRL_GAIN_SHIFT) /* 16MHz */
#  define OSCCTRL_XOSCCTRL_GAIN_30MHZ    (4 << OSCCTRL_XOSCCTRL_GAIN_SHIFT) /* 30MHz */
#define OSCCTRL_XOSCCTRL_AMPGC           (1 << 11) /* Bit 11: Automatic amplitude gain control */
#define OSCCTRL_XOSCCTRL_STARTUP_SHIFT   (12)      /* Bits 12-15: Start-up time */
#define OSCCTRL_XOSCCTRL_STARTUP_MASK    (15 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)
#  define OSCCTRL_XOSCCTRL_STARTUP(n)    ((n) << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)
#  define OSCCTRL_XOSCCTRL_STARTUP_31US  (0 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 31탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_61US  (1 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 61탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_122US (2 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 122탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_244US (3 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 244탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_488US (4 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 488탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_977US (5 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 977탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_2MS   (6 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 1953탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_4MS   (7 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 3906탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_8MS   (8 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 7813탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_16MS  (9 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT)  /* 15625탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_31MS  (10 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 31250탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_63MS  (11 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 62500탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_125MS (12 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 125000탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_250MS (13 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 250000탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_500MS (14 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 500000탎 */
#  define OSCCTRL_XOSCCTRL_STARTUP_1S    (15 << OSCCTRL_XOSCCTRL_STARTUP_SHIFT) /* 1000000탎 */

/* 16MHz internal oscillator control register */

#define OSCCTRL_OSC16MCTRL_ENABLE          (1 << 1)  /* Bit 1:  Oscillator enable */
#define OSCCTRL_OSC16MCTRL_FSEL_SHIFT      (2)       /* Bits 2-3: Oscillator frequency selection */
#define OSCCTRL_OSC16MCTRL_FSEL_MASK       (3 << OSCCTRL_OSC16MCTRL_FSEL_SHIFT)
#  define OSCCTRL_OSC16MCTRL_FSEL_4MHZ     (0 << OSCCTRL_OSC16MCTRL_FSEL_SHIFT)
#  define OSCCTRL_OSC16MCTRL_FSEL_8MHZ     (1 << OSCCTRL_OSC16MCTRL_FSEL_SHIFT)
#  define OSCCTRL_OSC16MCTRL_FSEL_12MHZ    (2 << OSCCTRL_OSC16MCTRL_FSEL_SHIFT)
#  define OSCCTRL_OSC16MCTRL_FSEL_16MHZ    (3 << OSCCTRL_OSC16MCTRL_FSEL_SHIFT)
#define OSCCTRL_OSC16MCTRL_RUNSTDBY        (1 << 6)  /* Bit 6:  Run in standby */
#define OSCCTRL_OSC16MCTRL_ONDEMAND        (1 << 7)  /* Bit 7:  On demand control */
#define OSCCTRL_OSC16MCTRL_GAIN_SHIFT      (8)       /* Bits 8-10: Oscillator gain */
#define OSCCTRL_OSC16MCTRL_GAIN_MASK       (7 << OSCCTRL_OSC16MCTRL_GAIN_SHIFT)
#  define OSCCTRL_OSC16MCTRL_GAIN(n)       ((n) << OSCCTRL_OSC16MCTRL_GAIN_SHIFT)
#  define OSCCTRL_OSC16MCTRL_GAIN_2MHZ     (0 << OSCCTRL_OSC16MCTRL_GAIN_SHIFT) /* 2MHz */
#  define OSCCTRL_OSC16MCTRL_GAIN_4MHZ     (1 << OSCCTRL_OSC16MCTRL_GAIN_SHIFT) /* 4MHz */
#  define OSCCTRL_OSC16MCTRL_GAIN_8MHZ     (2 << OSCCTRL_OSC16MCTRL_GAIN_SHIFT) /* 8MHz */
#  define OSCCTRL_OSC16MCTRL_GAIN_16MHZ    (3 << OSCCTRL_OSC16MCTRL_GAIN_SHIFT) /* 16MHz */
#  define OSCCTRL_OSC16MCTRL_GAIN_30MHZ    (4 << OSCCTRL_OSC16MCTRL_GAIN_SHIFT) /* 30MHz */
#define OSCCTRL_OSC16MCTRL_AMPGC           (1 << 11) /* Bit 11: Automatic amplitude gain control */
#define OSCCTRL_OSC16MCTRL_STARTUP_SHIFT   (12)      /* Bits 12-15: Start-up time */
#define OSCCTRL_OSC16MCTRL_STARTUP_MASK    (15 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)
#  define OSCCTRL_OSC16MCTRL_STARTUP(n)    ((n) << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)
#  define OSCCTRL_OSC16MCTRL_STARTUP_31US  (0 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)  /* 31탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_61US  (1 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)  /* 61탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_122US (2 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)  /* 122탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_244US (3 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)  /* 244탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_488US (4 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)  /* 488탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_977US (5 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)  /* 977탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_2MS   (6 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)  /* 1953탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_4MS   (7 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)  /* 3906탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_8MS   (8 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)  /* 7813탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_16MS  (9 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT)  /* 15625탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_31MS  (10 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT) /* 31250탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_63MS  (11 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT) /* 62500탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_125MS (12 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT) /* 125000탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_250MS (13 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT) /* 250000탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_500MS (14 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT) /* 500000탎 */
#  define OSCCTRL_OSC16MCTRL_STARTUP_1S    (15 << OSCCTRL_OSC16MCTRL_STARTUP_SHIFT) /* 1000000탎 */

/* DFLL48M control register */

#define OSCCTRL_DFLLCTRL_ENABLE          (1 << 1)  /* Bit 1:  DFLL enable */
#define OSCCTRL_DFLLCTRL_MODE            (1 << 2)  /* Bit 2:  Operating mode selection */
#define OSCCTRL_DFLLCTRL_STABLE          (1 << 3)  /* Bit 3:  Stable DFLL frequency */
#define OSCCTRL_DFLLCTRL_LLAW            (1 << 4)  /* Bit 4:  Lose lock after wake */
#define OSCCTRL_DFLLCTRL_USBCRM          (1 << 5)  /* Bit 5:  USB clock recovery mode */
#define OSCCTRL_DFLLCTRL_RUNSTDBY        (1 << 6)  /* Bit 6:  Run in standby */
#define OSCCTRL_DFLLCTRL_ONDEMAND        (1 << 7)  /* Bit 7:  On demand control */
#define OSCCTRL_DFLLCTRL_CCDIS           (1 << 8)  /* Bit 8:  Chill cycle disable */
#define OSCCTRL_DFLLCTRL_QLDIS           (1 << 9)  /* Bit 9:  Quick Lock Disable */
#define OSCCTRL_DFLLCTRL_BPLCKC          (1 << 10) /* Bit 10: Bypass coarse clock */
#define OSCCTRL_DFLLCTRL_WAITLOCK        (1 << 11) /* Bit 11: Wait lock */

/* DFLL48M value register */

#define OSCCTRL_DFLLVAL_FINE_SHIFT       (0)       /* Bits 0-9: Fine value */
#define OSCCTRL_DFLLVAL_FINE_MASK        (0x3ff << OSCCTRL_DFLLVAL_FINE_SHIFT)
#  define OSCCTRL_DFLLVAL_FINE(n)        ((n) << OSCCTRL_DFLLVAL_FINE_SHIFT)
#define OSCCTRL_DFLLVAL_COARSE_SHIFT     (10)      /* Bits 10-15: Coarse value */
#define OSCCTRL_DFLLVAL_COARSE_MASK      (0x3f << OSCCTRL_DFLLVAL_COARSE_SHIFT)
#  define OSCCTRL_DFLLVAL_COARSE(n)      ((n) << OSCCTRL_DFLLVAL_COARSE_SHIFT)
#define OSCCTRL_DFLLVAL_DIFF_SHIFT       (16)      /* Bits 16-31: Multiplication ratio difference */
#define OSCCTRL_DFLLVAL_DIFF_MASK        (0xffff << OSCCTRL_DFLLVAL_DIFF_SHIFT)
#  define OSCCTRL_DFLLVAL_DIFF(n)        ((n) << OSCCTRL_DFLLVAL_DIFF_SHIFT)

/* DFLL48M multiplier register */

#define OSCCTRL_DFLLMUL_MUL_SHIFT        (0)       /* Bits 0-15: DFLL multiply factor */
#define OSCCTRL_DFLLMUL_MUL_MASK         (0xffff << OSCCTRL_DFLLMUL_MUL_SHIFT)
#  define OSCCTRL_DFLLMUL_MUL(n)         ((n) << OSCCTRL_DFLLMUL_MUL_SHIFT)
#define OSCCTRL_DFLLMUL_FSTEP_SHIFT      (16)      /* Bits 16-25: Fine maximum step */
#define OSCCTRL_DFLLMUL_FSTEP_MASK       (0x3ff << OSCCTRL_DFLLMUL_FSTEP_SHIFT)
#  define OSCCTRL_DFLLMUL_FSTEP(n)       ((n) << OSCCTRL_DFLLMUL_FSTEP_SHIFT)
#define OSCCTRL_DFLLMUL_CSTEP_SHIFT      (26)      /* Bits 26-31: Coarse maximum step */
#define OSCCTRL_DFLLMUL_CSTEP_MASK       (0x3f << OSCCTRL_DFLLMUL_CSTEP_SHIFT)
#  define OSCCTRL_DFLLMUL_CSTEP(n)       ((n) << OSCCTRL_DFLLMUL_CSTEP_SHIFT)

/* DFLL48M synchronization register */

#define OSCCTRL_DFLLSYNC_READREQ         (1 << 7)  /* Bit 7: Read request */

/* DPLL control A */

#define OSCCTRL_DPLLCTRLA_ENABLE         (1 << 1)  /* Bit 1:  DPLL enable */
#define OSCCTRL_DPLLCTRLA_RUNSTDBY       (1 << 6)  /* Bit 6:  Run in standby */
#define OSCCTRL_DPLLCTRLA_ONDEMAND       (1 << 7)  /* Bit 7:  On demand clock activation */

/* DPLL ratio control */

#define OSCCTRL_DPLLRATIO_LDR_SHIFT      (0)     /* Bits 0-11: Loop divider ratio */
#define OSCCTRL_DPLLRATIO_LDR_MASK       (0xfff << OSCCTRL_DPLLRATIO_LDR_SHIFT)
#  define OSCCTRL_DPLLRATIO_LDR(n)       ((uint32_t)(n) << OSCCTRL_DPLLRATIO_LDR_SHIFT)
#define OSCCTRL_DPLLRATIO_LDRFRAC_SHIFT  (16)    /* Bits 16-19: Loop divider fractional part */
#define OSCCTRL_DPLLRATIO_LDRFRAC_MASK   (15 << OSCCTRL_DPLLRATIO_LDRFRAC_SHIFT)
#  define OSCCTRL_DPLLRATIO_LDRFRAC(n)   ((uint32_t)(n) << OSCCTRL_DPLLRATIO_LDRFRAC_SHIFT)

/* DPLL control B */

#define OSCCTRL_DPLLCTRLB_FILTER_SHIFT   (0)       /* Bits 0-1: Proportional integer filter selection */
#define OSCCTRL_DPLLCTRLB_FILTER_MASK    (3 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT)
#  define OSCCTRL_DPLLCTRLB_FILTER_DEFAULT  (0 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* Default filter mode */
#  define OSCCTRL_DPLLCTRLB_FILTER_LBFILT   (1 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* Low bandwidth filter */
#  define OSCCTRL_DPLLCTRLB_FILTER_HBFILT   (2 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* High bandwidth filter */
#  define OSCCTRL_DPLLCTRLB_FILTER_HDFILT   (3 << OSCCTRL_DPLLCTRLB_FILTER_SHIFT) /* High damping filter */
#define OSCCTRL_DPLLCTRLB_LPEN           (1 << 2)  /* Bit 2: Low-power enable */
#define OSCCTRL_DPLLCTRLB_WUF            (1 << 3)  /* Bit 3: Wake up fast */
#define OSCCTRL_DPLLCTRLB_REFLCK_SHIFT   (4)       /* Bits 4-5: Reference clock selection */
#define OSCCTRL_DPLLCTRLB_REFLCK_MASK    (3 << OSCCTRL_DPLLCTRLB_REFLCK_SHIFT)
#  define OSCCTRL_DPLLCTRLB_REFLCK_XOSCK32K (0 << OSCCTRL_DPLLCTRLB_REFLCK_SHIFT) /* XOSC32K clock reference */
#  define OSCCTRL_DPLLCTRLB_REFLCK_XOSC     (1 << OSCCTRL_DPLLCTRLB_REFLCK_SHIFT) /* XOSC clock reference */
#  define OSCCTRL_DPLLCTRLB_REFLCK_GLCK     (1 << OSCCTRL_DPLLCTRLB_REFLCK_SHIFT) /* GCLK clock reference */
#define OSCCTRL_DPLLCTRLB_LTIME_SHIFT    (8)       /* Bits 8-10: Lock time */
#define OSCCTRL_DPLLCTRLB_LTIME_MASK     (7 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT)
#  define OSCCTRL_DPLLCTRLB_LTIME_NONE   (0 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT) /* No time-out. Automatic lock */
#  define OSCCTRL_DPLLCTRLB_LTIME_8MS    (4 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no locka within 8MS */
#  define OSCCTRL_DPLLCTRLB_LTIME_9MS    (5 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no locka within 9MS */
#  define OSCCTRL_DPLLCTRLB_LTIME_10MS   (6 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no locka within 10MS */
#  define OSCCTRL_DPLLCTRLB_LTIME_11MS   (7 << OSCCTRL_DPLLCTRLB_LTIME_SHIFT) /* Time-out if no locka within 11MS */
#define OSCCTRL_DPLLCTRLB_LBYPASS        (1 << 12) /* Bit 12: Lock bypass */
#define OSCCTRL_DPLLCTRLB_DIV_SHIFT      (16)      /* Bits 16-26: Clock divider */
#define OSCCTRL_DPLLCTRLB_DIV_MASK       (0x7ff << OSCCTRL_DPLLCTRLB_DIV_SHIFT)
#  define OSCCTRL_DPLLCTRLB_DIV(n)       ((uint32_t)(n) << OSCCTRL_DPLLCTRLB_DIV_SHIFT)

/* DPLL prescaler */

#define OSCCTRL_DPLLPRESC_SHIFT          (0)       /* Bit 0-1: Output clock prescaler */
#define OSCCTRL_DPLLPRESC_MASK           (3 << OSCCTRL_DPLLPRESC_SHIFT)
#  define OSCCTRL_DPLLPRESC_DIV1         (0 << OSCCTRL_DPLLPRESC_SHIFT) /* DPLL output is divided by 1 */
#  define OSCCTRL_DPLLPRESC_DIV2         (1 << OSCCTRL_DPLLPRESC_SHIFT) /* DPLL output is divided by 2 */
#  define OSCCTRL_DPLLPRESC_DIV4         (2 << OSCCTRL_DPLLPRESC_SHIFT) /* DPLL output is divided by 4 */

/* DPLL synchronization busy */

#define OSCCTRL_DPLLSYNCBUSY_ENABLE      (1 << 1)  /* Bit 1:  DPLL Enable synchonization status */
#define OSCCTRL_DPLLSYNCBUSY_DPLLRATIO   (1 << 2)  /* Bit 2:  DPLL Loop divider ration status */
#define OSCCTRL_DPLLSYNCBUSY_DPLLPRESC   (1 << 3)  /* Bit 3:  DPLL prescaler synchronization status */

/* DPLL status */

#define OSCCTRL_DPLLSTATUS_LOCK          (1 << 0)  /* Bit 0:  DPLL lock status */
#define OSCCTRL_DPLLSTATUS_CLKRDY        (1 << 1)  /* Bit 1:  Output clock ready */

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
#endif /* __ARCH_ARM_SRC_SAMDL_CHIP_SAML_OSCCTRL_H */
