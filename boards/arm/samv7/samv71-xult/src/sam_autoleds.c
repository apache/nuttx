/****************************************************************************
 * boards/arm/samv7/samv71-xult/include/sam_autoleds.c
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
 * There are two yellow LED available on the SAM V71 Xplained Ultra board that
 * can be turned on and off.  The LEDs can be activated by driving the
 * connected I/O line to GND.
 *
 *   ------ ----------- ---------------------
 *   SAMV71 Function    Shared functionality
 *   GPIO
 *   ------ ----------- ---------------------
 *   PA23   Yellow LED0 EDBG GPIO
 *   PC09   Yellow LED1 LCD, and Shield
 *   ------ ----------- ---------------------
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *   -------------------  -----------------------  -------- --------
 *   SYMBOL                Meaning                     LED state
 *                                                   LED0     LED1
 *   -------------------  -----------------------  -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF
 *   LED_INIRQ            In an interrupt              No change
 *   LED_SIGNAL           In a signal handler          No change
 *   LED_ASSERTION        An assertion failed          No change
 *   LED_PANIC            The system has crashed     N/C      Blinking
 *   LED_IDLE             MCU is is sleep mode         Not used
 *   -------------------  -----------------------  -------- --------
 *
 * Thus if LED0 is statically on, NuttX has successfully booted and is,
 * apparently, running normally.  If LED1 is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 *
 * NOTE: That LED0 is not used after completion of booting and may
 * be used by other board-specific logic.
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

#include "sam_gpio.h"
#include "samv71-xult.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED GPIOs for output */

  sam_configgpio(GPIO_LED0);
  sam_configgpio(GPIO_LED1);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case 0:  /* LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED */
        break;

      case 1:  /* LED_STACKCREATED */
        sam_gpiowrite(GPIO_LED0, false); /* Low illuminates */
        break;

      default:
      case 2:  /* LED_INIRQ, LED_SIGNAL, LED_ASSERTION */
        return;

      case 3:  /* LED_PANIC */
        sam_gpiowrite(GPIO_LED1, false); /* Low illuminates */
        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == 3)
    {
      sam_gpiowrite(GPIO_LED1, true);  /* High extinguishes */
    }
}

#endif /* CONFIG_ARCH_LEDS */
