/************************************************************************************
 * configs/samv71-xult/include/board.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_SAMV71_XULT_INCLUDE_BOARD_H
#define __CONFIGS_SAMV71_XULT_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Clocking *************************************************************************/
/* After power-on reset, the SAMV71Q device is running out of the Master Clock using
 * the Fast RC Oscillator running at 4 MHz.
 *
 *   MAINOSC:  Frequency = 12MHz (crystal)
 *
 * 300MHz Settings:
 *   PLLA: PLL Divider = 1, Multiplier = 20 to generate PLLACK = 240MHz
 *   Master Clock (MCK): Source = PLLACK, Prescalar = 1 to generate MCK = 120MHz
 *   CPU clock: 120MHz
 *
 */

/* Main oscillator register settings.
 *
 * The main oscillator could be either the embedded 4/8/12 MHz fast RC oscillators
 * or an external 3-20 MHz crystal or ceramic resonator.  The external clock source
 * is selected by default in sam_clockconfig.c.  Here we need to specify the main
 * oscillator start-up time.
 *
 * REVISIT... this is old information:
 * The start up time should be should be:
 *
 *   Start Up Time = 8 * MOSCXTST / SLCK = 56 Slow Clock Cycles.
 */

#define BOARD_CKGR_MOR_MOSCXTST    (62 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration.
 *
 *   Divider = 1
 *   Multiplier = 25
 *
 * Yields:
 *
 *   PLLACK = 25 * 12MHz / 1 = 300MHz
 */

#define BOARD_CKGR_PLLAR_STMODE    PMC_CKGR_PLLAR_STMODE_FAST
#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_CKGR_PLLAR_MUL       PMC_CKGR_PLLAR_MUL(25)
#define BOARD_CKGR_PLLAR_DIV       PMC_CKGR_PLLAR_DIV_BYPASS

/* PMC master clock register settings.
 *
 *  BOARD_PMC_MCKR_CSS - The source of main clock input.  This may be one of:
 *
 *    PMC_MCKR_CSS_SLOW   Slow Clock
 *    PMC_MCKR_CSS_MAIN   Main Clock
 *    PMC_MCKR_CSS_PLLA   PLLA Clock
 *    MC_MCKR_CSS_UPLL    Divided UPLL Clock
 *
 *  BOARD_PMC_MCKR_PRES - Source clock pre-scaler.  May be one of:
 *
 *    PMC_MCKR_PRES_DIV1  Selected clock
 *    PMC_MCKR_PRES_DIV2  Selected clock divided by 2
 *    PMC_MCKR_PRES_DIV4  Selected clock divided by 4
 *    PMC_MCKR_PRES_DIV8  Selected clock divided by 8
 *    PMC_MCKR_PRES_DIV16 Selected clock divided by 16
 *    MC_MCKR_PRES_DIV32  Selected clock divided by 32
 *    PMC_MCKR_PRES_DIV64 Selected clock divided by 64
 *    PMC_MCKR_PRES_DIV3  Selected clock divided by 3
 *
 *  The prescaler determines (1) the CPU clock and (2) the input into the
 *  second divider that then generates the Master Clock (MCK).  MCK is the
 *  source clock of the peripheral clocks.
 *
 *  BOARD_PMC_MCKR_MDIV - MCK divider.  May be one of:
 *
 *    PMC_MCKR_MDIV_DIV1  Master Clock is Prescaler Output Clock / 1
 *    PMC_MCKR_MDIV_DIV2  Master Clock = Prescaler Output Clock / 2
 *    PMC_MCKR_MDIV_DIV4  Master Clock = Prescaler Output Clock / 4
 *    PMC_MCKR_MDIV_DIV3  Master Clock = Prescaler Output Clock / 3
 */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA   /* Source = PLLA */
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV1  /* Prescaler = /1 */
#define BOARD_PMC_MCKR_MDIV        PMC_MCKR_MDIV_DIV2  /* MCK divider = /2 */

/* USB clocking
 * To be provided
 */

#define BOARD_PMC_MCKR_UPLLDIV2    0           /* UPLL clock not divided by 2 */

/* Resulting frequencies */

#define BOARD_MAINOSC_FREQUENCY    (12000000)  /* MAINOSC: 12MHz crystal on-board */
#define BOARD_PLLA_FREQUENCY       (300000000) /* PLLACK:  25 * 12Mhz / 1 */
#define BOARD_CPU_FREQUENCY        (300000000) /* CPU:     PLLACK / 1 */
#define BOARD_MCK_FREQUENCY        (150000000) /* MCK:     PLLACK / 1 / 2 */

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV+1)).
 *
 *   MCI_SPEED = MCK / (2*(CLKDIV+1))
 *   CLKDIV = MCK / MCI_SPEED / 2 - 1
 *
 * Where CLKDIV has a range of 0-255.
 */

/* FLASH wait states.
 *
 * Wait states Max frequency at 105 centigrade (STH conditions)
 *
 *           VDDIO
 *       1.62V     2.7V
 * --- -------  -------
 *  0   26 MHz   30 MHz
 *  1   52 MHz   62 MHz
 *  2   78 MHz   93 MHz
 *  3  104 MHz  124 MHz
 *  4  131 MHz  150 MHz
 *  5  150 MHz  --- MHz
 */

#define BOARD_FWS                  5

/* LED definitions ******************************************************************/

/* Button definitions ***************************************************************/

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

 /************************************************************************************
 * Name:  sam_ledinit, sam_setled, and sam_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfacesare available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void sam_ledinit(void);
void sam_setled(int led, bool ledon);
void sam_setleds(uint8_t ledset);
#endif

/************************************************************************************
 * Name:  sam_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the SAM4e-EK board.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   display is cleared by simply setting all GRAM memory to the specified color.
 *
 ************************************************************************************/

#if defined(CONFIG_SAM4EEK_LCD_RGB565)
void sam_lcdclear(uint16_t color);
#else /* if defined(CONFIG_SAM4EEK_LCD_RGB24) defined(CONFIG_SAM4EEK_LCD_RGB32) */
void sam_lcdclear(uint32_t color);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAMV71_XULT_INCLUDE_BOARD_H */
