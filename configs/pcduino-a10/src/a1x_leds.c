/****************************************************************************
 * configs/pcduino-a10/src/a1x_leds.c
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

#include "pcduino_a10.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_LEDS enables debug output from this file (needs CONFIG_DEBUG
 * with CONFIG_DEBUG_VERBOSE too)
 */

#ifdef CONFIG_DEBUG_LEDS
#  define leddbg  lldbg
#  define ledvdbg llvdbg
#else
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
 * Name: a1x_led_initialize
 *
 * Description:
 *   Configure LEDs.  LEDs are left in the OFF state.
 *
 ****************************************************************************/

void a1x_led_initialize(void)
{
#warning Missing Logic
}

/****************************************************************************
 * Name: up_ledon
 *
 * Description:
 *   Select the "logical" ON state:
 *
 *    SYMBOL          Value  Meaning                     LED state
 *                                                   LED2     LED1
 *   ---------------- -----  -----------------------  -------- --------
 *   LED_STARTED        0  NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE   0  Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED    0  Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED   1  Idle stack created         ON       OFF
 *   LED_INIRQ          2  In an interrupt            N/C      N/C
 *   LED_SIGNAL         2  In a signal handler        N/C      N/C
 *   LED_ASSERTION      2  An assertion failed        N/C      N/C
 *   LED_PANIC          3  The system has crashed     N/C      Blinking
 *   LED_IDLE           -  MCU is is sleep mode         Not used
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledon(int led)
{
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: up_ledoff
 *
 * Description:
 *   Select the "logical" OFF state:
 *
 *    SYMBOL          Value  Meaning                     LED state
 *                                                   LED2     LED1
 *   ---------------- -----  -----------------------  -------- --------
 *   LED_STARTED        0  NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE   0  Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED    0  Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED   1  Idle stack created         ON       OFF
 *   LED_INIRQ          2  In an interrupt            N/C      N/C
 *   LED_SIGNAL         2  In a signal handler        N/C      N/C
 *   LED_ASSERTION      2  An assertion failed        N/C      N/C
 *   LED_PANIC          3  The system has crashed     N/C      Blinking
 *   LED_IDLE           -  MCU is is sleep mode         Not used
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void up_ledoff(int led)
{
#warning Missing logic
}
#endif

/************************************************************************************
 * Name:  a1x_setled and a1x_setleds
 *
 * Description:
 *   These interfaces allow user control of the board LEDs.
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control both on-board LEDs up
 *   until the completion of boot.  The it will continue to control LED2; LED1 is
 *   avaiable for application use.
 *
 *   If CONFIG_ARCH_LEDS is not defined, then both LEDs are available for application
 *   use.
 *
 ************************************************************************************/

void a1x_setled(int led, bool ledon)
{
#warning Missing logic
}

void a1x_setleds(uint8_t ledset)
{
#warning Missing logic
}
