/****************************************************************************
 * boards/arm/lpc2378/olimex-lpc2378/src/lpc2378_leds.c
 *
 *   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
 *   Author: Rommel Marcelo
 *
 * This is part of the NuttX RTOS and based on the LPC2148 port:
 *
 *   Copyright (C) 2010, 2014-2015 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <nuttx/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* P3.0 : P0.7 PINSEL6 LEDS 1-8 */

#define LEDBIT(led)     (0x01 << (led))
#define ALL_LEDS        (0xff)
#define STATLED         (0x08)

#define putled8(v,o)    putreg8((v), (LPC23XX_FIO_BASE+(o)))
#define putled32(v,r)   putreg32((v),(LPC23XX_FIO_BASE+(r)))
#define CLRLEDS         putled(ALL_LEDS,FIO3CLR0_OFFSET)

#define LED_SET_OFFSET  FIO3SET0_OFFSET
#define LED_CLR_OFFSET  FIO3CLR0_OFFSET
#define LED_DIR_OFFSET  FIO3DIR0_OFFSET
#define LED_MASK_OFFSET FIO3MASK0_OFFSET

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  /* Initialize GIOs P1.16-P1.23 */

  putled8(ALL_LEDS, LED_DIR_OFFSET);
  putled8(ALL_LEDS, LED_CLR_OFFSET);
  putled8(LEDBIT(0), LED_SET_OFFSET);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  putled8(~(LEDBIT(led)), LED_MASK_OFFSET);
  putled8(LEDBIT(led), LED_SET_OFFSET);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  putled8(LEDBIT(led), LED_CLR_OFFSET);
}

/****************************************************************************
 * olimex board STATUS LED
 ****************************************************************************/

void lpc2378_statledoff(void)
{
  putled8(~STATLED, FIO1MASK2_OFFSET);
  putled8(STATLED, FIO1CLR2_OFFSET);
}

void lpc2378_statledon(void)
{
  putled8(~STATLED, FIO1MASK2_OFFSET);
  putled8(STATLED, FIO1SET2_OFFSET);
}

#endif /* CONFIG_ARCH_LEDS */
