/****************************************************************************
 * configs/pic32-starterkit/src/up_leds.c
 * arch/arm/src/board/up_leds.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "pic32mx-internal.h"
#include "starterkit_internal.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* The Sure PIC32MX board has five LEDs.  One (D4, lablel "Power") is not
 * controllable by software.  Four are controllable by software:
 *
 * D7  "USB"    Yellow  RD7 Low illuminates
 * D8  "SD"     Yellow  RD6 Low illuminates
 * D9  "Flash"  Yellow  RF0 Low illuminates
 * D10 "Error"  Red     RF1 Low illuminates
 *
 *                           ON                  OFF
 *                           USB SD  FLASH ERROR USB SD  FLASH ERROR
 * LED_STARTED            0  OFF OFF OFF   OFF   --- --- ---   ---
 * LED_HEAPALLOCATE       1  ON  OFF N/C   N/C   --- --- ---   ---
 * LED_IRQSENABLED        2  OFF ON  N/C   N/C   --- --- ---   ---
 * LED_STACKCREATED       3  ON  ON  N/C   N/C   --- --- ---   ---
 * LED_INIRQ              4  N/C N/C ON    N/C   N/C N/C OFF   N/C
 * LED_SIGNAL             4  N/C N/C ON    N/C   N/C N/C OFF   N/C
 * LED_ASSERTION          4  N/C N/C ON    N/C   N/C N/C OFF   N/C
 * LED_PANIC              5  N/C N/C N/C   ON    N/C N/C N/C   OFF
 */

/* Enables debug output from this file (needs CONFIG_DEBUG with
 * CONFIG_DEBUG_VERBOSE too)
 */

#undef LED_DEBUG   /* Define to enable debug */
#undef LED_VERBOSE /* Define to enable verbose debug */

#ifdef LED_DEBUG
#  define leddbg  lldbg
#  ifdef LED_VERBOSE
#    define ledvdbg lldbg
#  else
#    define ledvdbg(x...)
#  endif
#else
#  undef LED_VERBOSE
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

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
 * Name: up_ledinit
 ****************************************************************************/

void up_ledinit(void)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
#warning "Missing logic"
}
#endif /* CONFIG_ARCH_LEDS */
