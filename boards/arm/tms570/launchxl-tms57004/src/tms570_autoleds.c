/****************************************************************************
 * boards/arm/tms570/launchxl-tms57004/include/tms570_autoleds.c
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
 ****************************************************************************/

/* LEDs
 *
 * The launchpad has several LEDs:
 *
 *   - LEd D1 (white) that connects to the USB +5V supply,
 *   - LED D10 (red) that connects to the TMS570's NERROR pin,
 *   - D5 (blue), D6 (blue), and D8 (blue) connect to the XDS100 FT2322,
 *   - D7 (blue) connects to the XSD100 CPLD, and
 *   - Two white, user LEDs labeled D12 that connects to the NHET08
 *     pin and D11 that connects to GIOA2.
 *
 * NHET08 is one of 32 N2HET pins than can be available to the user if
 * not used by N2HET.  This implementation, however, uses only the single
 * LED driven by GIOA2.  That LED is tied to ground and illuminated
 * with a high level output value.
 *
 * This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/tms570_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   ------------------- ----------------------- ------
 *   SYMBOL              Meaning                 LED
 *   ------------------- ----------------------- ------
 *   LED_STARTED         NuttX has been started  OFF
 *   LED_HEAPALLOCATE    Heap has been allocated OFF
 *   LED_IRQSENABLED     Interrupts enabled      OFF
 *   LED_STACKCREATED    Idle stack created      ON
 *   LED_INIRQ           In an interrupt         N/C
 *   LED_SIGNAL          In a signal handler     N/C
 *   LED_ASSERTION       An assertion failed     N/C
 *   LED_PANIC           The system has crashed  FLASH
 *
 * Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "tms570_gio.h"
#include "launchxl-tms57004.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED GIOs for output */

  tms570_configgio(GIO_LED_D11);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led == 1 || led == 3)
    {
      tms570_giowrite(GIO_LED_D11, true); /* High illuminates */
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == 3)
    {
      tms570_giowrite(GIO_LED_D11, false);  /* Low extinguishes */
    }
}

#endif /* CONFIG_ARCH_LEDS */
