/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_clockconfig.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Parts of this file were adapted from sample code provided for the
 * LPC54xx family from NXP which has a compatible BSD license.
 *
 *   Copyright (c) 2016, Freescale Semiconductor, Inc.
 *   Copyright (c) 2016 - 2017 , NXP
 *   All rights reserved.
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC54XX_LPC54_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_LPC54XX_LPC54_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Frequencies of internal clocks */

#define LPC54_SYSTEM_CLOCK     12000000  /* Default System clock value */
#define LPC54_RTC_CLOCK           32768  /* RTC oscillator 32 kHz output (32k_clk )*/
#define LPC54_FRO_12MHZ        12000000  /* FRO 12 MHz (fro_12m) */
#define LPC54_FRO_48MHZ        48000000  /* FRO 48 MHz (fro_48m) */
#define LPC54_FRO_96MHZ        96000000  /* FRO 96 MHz (fro_96m) */
#define LPC54_CLKIN                   0  /* CLKIN pin clock */

/* PLL setup structure flags for pllflags field.  These flags control how
 * the PLL setup function sets up the PLL
 */

#define PLL_SETUPFLAG_POWERUP  (1 << 0)  /* Power on the PLL after setup */
#define PLL_SETUPFLAG_WAITLOCK (1 << 1)  /* Wait for PLL lock and power on */
#define PLL_SETUPFLAG_ADGVOLT  (1 << 2)  /* Optimize system voltage for PLL rate */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* PLL setup structure.
 *
 * This structure can be used to define a PLL configuration.  If powering
 * up or waiting for PLL lock, the PLL input clock source should be
 * configured prior to PLL setup.
 */

struct pll_setup_s
{
  uint32_t pllclksel;       /* PLL clock source register SYSPLLCLKSEL */
  uint32_t pllctrl;         /* PLL control register SYSPLLCTRL */
  uint32_t pllndec;         /* PLL NDEC register SYSPLLNDEC */
  uint32_t pllpdec;         /* PLL PDEC register SYSPLLPDEC */
  uint32_t pllmdec;         /* PLL MDEC registers SYSPLLPDEC */
  uint32_t pllfout;         /* Actual PLL output frequency */
  uint32_t pllfrac;         /* Only aduio PLL has this function */
  uint32_t pllflags;        /* PLL setup flags */
  uint32_t ahbdiv;          /* AHB divider */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_clockconfig
 *
 * Description:
 *   Called to initialize the LPC54xx.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 ****************************************************************************/

void lpc54_clockconfig(const struct pll_setup_s *pllsetup);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LPC54XX_LPC54_CLOCKCONFIG_H */
