/************************************************************************************
 * configs/open1788/src/lpc17_sdraminitialize.c
 * arch/arm/src/board/lpc17_sdraminitialize.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "chip/lpc17_syscon.h"
#include "lpc17_emc.h"

#include "open1788.h"

#if defined(CONFIG_LPC17_EMC) && defined(CONFIG_LPC17_EXTDRAM)

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* The core clock is LPC17_EMCCLK which may be either LPC17_CCLK* (undivided), or
 * LPC17_CCLK / 2 as determined by settings in the board.h header file.
 *
 * For example:
 *   LPC17_CCLCK      =  120,000,000
 *   EMCCLKSEL        -> LPC17_CCLK divided by 2
 *   LPC17_EMCCLK     =  60,000,000
 *   LPC17_EMCCLK_MHZ =  60 (Rounded to an integer)
 *   EMC_NSPERCLK     =  16.667 (Represented with 4 bits of fraction, 267)
 *
 *   EMC_NS2CLK(63)   = ((63 << 4) + 266) / 267 = 4 (actual 3.78)
 *   EMC_NS2CLK(20)   = ((20 << 4) + 266) / 267 = 2 (actual 1.20)
 */

#define LPC17_EMCCLK_MHZ    ((LPC17_EMCCLK + 500000) / 1000000)
#define EMC_NSPERCLK_B4     (((1000 << 4) + (LPC17_EMCCLK_MHZ >> 1)) / LPC17_EMCCLK_MHZ)
#define EMC_NS2CLK(ns)      (((ns << 4) + (EMC_NSPERCLK_B4 - 1)) / EMC_NSPERCLK_B4)
#define MDKCFG_RASCAS0VAL   0x00000303

/* Set up for 32-bit SDRAM at CS0 */

#define CONFIG_LPC17_SDRAM_32BIT

#ifdef CONFIG_LPC17_SDRAM_16BIT
#  define SDRAM_SIZE        0x02000000 /* 256Mbit */
#else /* if defined(CONFIG_LPC17_SDRAM_32BIT) */
#  undef CONFIG_LPC17_SDRAM_32BIT
#  define CONFIG_LPC17_SDRAM_32BIT 1
#  define SDRAM_SIZE        0x04000000 /* 512Mbit */
#endif

#define SDRAM_BASE          0xa0000000 /* CS0 */

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: open1788_sdram_initialize
 *
 * Description:
 *   Initialize SDRAM
 *
 ************************************************************************************/

void open1788_sdram_initialize(void)
{
  uint32_t regval;
#ifdef CONFIG_LPC17_SDRAM_16BIT
  volatile uint16_t dummy;
#else
  volatile uint32_t dummy;
#endif
  int i;

  /* Reconfigure delays:
   *
   * CMDDLY: Programmable delay value for EMC outputs in command delayed
   *   mode.  The delay amount is roughly CMDDLY * 250 picoseconds.
   * FBCLKDLY: Programmable delay value for the feedback clock that controls
   *   input data sampling.  The delay amount is roughly (FBCLKDLY+1) * 250
   *   picoseconds.
   * CLKOUT0DLY: Programmable delay value for the CLKOUT0 output. This would
   *   typically be used in clock delayed mode.  The delay amount is roughly
   *  (CLKOUT0DLY+1) * 250 picoseconds.
   * CLKOUT1DLY: Programmable delay value for the CLKOUT1 output. This would
   *  typically be used in clock delayed mode.  The delay amount is roughly
   *  (CLKOUT1DLY+1) * 250 picoseconds.
   */

  regval = SYSCON_EMCDLYCTL_CMDDLY(32) |
           SYSCON_EMCDLYCTL_FBCLKDLY(32) |
           SYSCON_EMCDLYCTL_CLKOUT0DLY(1) |
           SYSCON_EMCDLYCTL_CLKOUT1DLY(1);
  putreg32(regval, LPC17_SYSCON_EMCDLYCTL);

  /* Configure the SDRAM */

  putreg32(    EMC_NS2CLK(20), LPC17_EMC_DYNAMICRP);   /* TRP   = 20 nS */
  putreg32(                15, LPC17_EMC_DYNAMICRAS);  /* RAS   = 42ns to 100K ns,  */
  putreg32(                 0, LPC17_EMC_DYNAMICSREX); /* TSREX = 1 clock */
  putreg32(                 1, LPC17_EMC_DYNAMICAPR);  /* TAPR  = 2 clocks? */
  putreg32(EMC_NS2CLK(20) + 2, LPC17_EMC_DYNAMICDAL);  /* TDAL  = TRP + TDPL = 20ns + 2clk  */
  putreg32(                 1, LPC17_EMC_DYNAMICWR);   /* TWR   = 2 clocks */
  putreg32(    EMC_NS2CLK(63), LPC17_EMC_DYNAMICRC);   /* H57V2562GTR-75C TRC = 63ns(min)*/
  putreg32(    EMC_NS2CLK(63), LPC17_EMC_DYNAMICRFC);  /* H57V2562GTR-75C TRFC = TRC */
  putreg32(                15, LPC17_EMC_DYNAMICXSR);  /* Exit self-refresh to active */
  putreg32(    EMC_NS2CLK(63), LPC17_EMC_DYNAMICRRD);  /* 3 clock, TRRD = 15ns (min) */
  putreg32(                 1, LPC17_EMC_DYNAMICMRD);  /* 2 clock, TMRD = 2 clocks (min) */

  /* Command delayed strategy, using EMCCLKDELAY */

  putreg32(EMC_DYNAMICREADCONFIG_RD_CMD, LPC17_EMC_DYNAMICREADCONFIG);

  /* H57V2562GTR-75C: TCL=3CLK, TRCD = 20ns(min), 3 CLK = 24ns */

  putreg32(MDKCFG_RASCAS0VAL, LPC17_EMC_DYNAMICRASCAS0);

  #ifdef CONFIG_LPC17_SDRAM_16BIT
    /* For Manley lpc1778 SDRAM: H57V2562GTR-75C, 256Mb, 16Mx16, 4 banks, row=13, column=9:
     *
     * 256Mb, 16Mx16, 4 banks, row=13, column=9, RBC
     */

  putreg32(EMC_DYNAMICCONFIG_MD_SDRAM |  EMC_DYNAMICCONFIG_AM0(13),
           LPC17_EMC_DYNAMICCONFIG0);

#elif defined CONFIG_LPC17_SDRAM_32BIT
    /* 256Mb, 16Mx16, 4 banks, row=13, column=9, RBC */

  putreg32(EMC_DYNAMICCONFIG_MD_SDRAM |  EMC_DYNAMICCONFIG_AM0(13) | EMC_DYNAMICCONFIG_AM1,
           LPC17_EMC_DYNAMICCONFIG0);
#endif

  up_mdelay(100);

  /* Issue NOP command */

  putreg32(EMC_DYNAMICCONTROL_CE | EMC_DYNAMICCONTROL_CS | EMC_DYNAMICCONTROL_I_NOP,
           LPC17_EMC_DYNAMICCONTROL);

  /* Wait 200 Msec */

  up_mdelay(200);

  /* Issue PALL command */

  putreg32(EMC_DYNAMICCONTROL_CE | EMC_DYNAMICCONTROL_CS | EMC_DYNAMICCONTROL_I_PALL,
           LPC17_EMC_DYNAMICCONTROL);

  putreg32(2, LPC17_EMC_DYNAMICREFRESH); /* ( n * 16 ) -> 32 clock cycles */

  /* Wait 128 AHB clock cycles */

  for (i = 0; i < 128; i++);

  /* 64ms/8192 = 7.8125us, nx16x8.33ns < 7.8125us, n < 58.6*/

  regval = 64000000 / (1 << 13);
  regval -= 16;
  regval >>= 4;
  regval = regval * LPC17_EMCCLK_MHZ / 1000;
  putreg32(regval, LPC17_EMC_DYNAMICREFRESH);

  /* Issue MODE command */

  putreg32(EMC_DYNAMICCONTROL_CE | EMC_DYNAMICCONTROL_CS | EMC_DYNAMICCONTROL_I_MODE,
           LPC17_EMC_DYNAMICCONTROL);

#ifdef CONFIG_LPC17_SDRAM_16BIT
  dummy = getreg16(SDRAM_BASE | (0x33 << 12));  /* 8 burst, 3 CAS latency */
#elif defined CONFIG_LPC17_SDRAM_32BIT
  dummy = getreg32(SDRAM_BASE | (0x32 << 13)); /* 4 burst, 3 CAS latency */
#endif

  /* Issue NORMAL command */

  putreg32(EMC_DYNAMICCONTROL_I_NORMAL, LPC17_EMC_DYNAMICCONTROL);

  /* Enable buffer */

  regval = getreg32(LPC17_EMC_DYNAMICCONFIG0);
  regval |= EMC_DYNAMICCONFIG_B;
  putreg32(regval, LPC17_EMC_DYNAMICCONFIG0);
  up_mdelay(12);

  regval = getreg32(LPC17_SYSCON_EMCDLYCTL);
  regval &= ~SYSCON_EMCDLYCTL_CMDDLY_MASK;
  regval |= SYSCON_EMCDLYCTL_CMDDLY(18);
  putreg32(regval, LPC17_SYSCON_EMCDLYCTL);
}

#endif /* CONFIG_LPC17_EMC && CONFIG_LPC17_EXTDRAM */
