/****************************************************************************
 * boards/z80/ez80/makerlisp/include/board.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_Z80_EZ80_MAKERLISP_INCLUDE_BOARD_H
#define __BOARDS_Z80_EZ80_MAKERLISP_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking
 *
 * The MakerLisp CPU board has a 5Mhz crystal. This is multiplied by 10 by
 * the PLL to obtain a system clock frequency of 50MHz.  See the PLL setup
 * in scripts/makerlisp.linkcmd.
 */

#define EZ80_SYS_CLK_FREQ           50000000

/* LEDs */

/* The D3 GREEN LED is driven by an eZ80 GPI/O pin.  However, it has some
 * additional properties:
 *
 * 1. On input, it will be '1' if the I/O expansion board is present.
 * 2. Setting it to an output of '0' will generate a system reset.
 * 3. Setting it to an output of '1' will not only illuminate the LED
 *    take the card out of reset and enable power to the SD card slot.
 *
 * As a consequence, the GREEN LED will not be illuminated if SD card
 * support or SPI is disabled.  The only effect of CONFIG_ARCH_LEDS is that
 * the GREEN LED will turned off in the event of a crash.
 */

#define LED_STARTED                 0
#define LED_HEAPALLOCATE            0
#define LED_IRQSENABLED             0
#define LED_STACKCREATED            0
#define LED_IDLE                    0
#define LED_INIRQ                   0
#define LED_ASSERTION               0
#define LED_SIGNAL                  0
#define LED_PANIC                   1

/* Button definitions
 * The MakerLisp CPU board has no on-board buttons that can be sensed by the
 * eZ80.
 */

/****************************************************************************
 * Public Functions
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

#endif /* __BOARDS_Z80_EZ80_MAKERLISP_INCLUDE_BOARD_H */
