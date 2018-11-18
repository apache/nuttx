/****************************************************************************
 * arch/arm/src/max326xx/max32660/max32660_clockconfig.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX_32660_MAX32660_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_MAX326XX_MAX_32660_MAX32660_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <arch/board/board.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Clock sources.  Values match the CLKSEL field of the GCR_CLK_CTRL
 * register
 */

enum clock_source_e
{
  CLKSRC_HFIO  = 0, /* High frequency internal oscillator */
  CLKSRC_8KHZ  = 3, /* 8kHz Internal Ultra-Low Power Nano-Ring Oscillator */
  CLKSRC_32KHZ = 6  /* 32.768kHz External Crystal Oscillator */
};

/* This structure can be used to define a clock configuration.
 *
 * Fhfio     Determined by Output Voltage Range.
 * Fsysosc   Determined by source clock selection.
 * Fsysclk = Fsysclk / (2^psc)
 * Fpclk   = Fsysclk / 2
 */

struct clock_setup_s
{
  uint8_t ovr;      /* Output voltage range for internal regulator */
  uint8_t clksrc;   /* See enum clock_source_e.  Determines Fsysosc */
  uint8_t psc;      /* System Oscillator Prescaler.  Derives Fsysclk */
  bool hfio;        /* True: Enable the High frequency internal oscillator. */
#ifdef BOARD_HAVE_X32K
  bool x32k;        /* True: Enable the 32.768KHz ext crystal oscillator. */
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_ARM_SRC_MAX326XX_MAX_32660_MAX32660_CLOCKCONFIG_H */
