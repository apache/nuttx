/****************************************************************************
 * configs/imxrt1050-evk/include/imxrt_autoleds.c
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

/* There are four LED status indicators located on the EVK Board.  The
 * functions of these LEDs include:
 *
 *   - Main Power Supply(D3)
 *     Green: DC 5V main supply is normal.
 *     Red:   J2 input voltage is over 5.6V.
 *     Off:   The board is not powered.
 *   - Reset RED LED(D15)
 *   - OpenSDA LED(D16)
 *   - USER LED(D18)
 *
 * Only a single LED, D18, is under software control.
 *
 * This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/imxrt_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   -------------------- ----------------------- ------
 *   SYMBOL               Meaning                 LED
 *   -------------------- ----------------------- ------
 *
 *   LED_STARTED       0  NuttX has been started  OFF
 *   LED_HEAPALLOCATE  0  Heap has been allocated OFF
 *   LED_IRQSENABLED   0  Interrupts enabled      OFF
 *   LED_STACKCREATED  1  Idle stack created      ON
 *   LED_INIRQ         2  In an interrupt         N/C
 *   LED_SIGNAL        2  In a signal handler     N/C
 *   LED_ASSERTION     2  An assertion failed     N/C
 *   LED_PANIC         3  The system has crashed  FLASH
 *   LED_IDLE             Not used
 *
 * Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include "imxrt1050-evk.h"

#include <arch/board/board.h>

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_autoled_initialize
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imxrt_autoled_initialize(void)
{
  /* Configure LED GPIO for output */

  imxrt_config_gpio(GPIO_LED);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Turn on the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool ledoff = false;

  switch (led)
    {
      case 0:  /* LED Off */
        ledoff = true;
        break;

      case 2:  /* LED No change */
        return;

      case 1:  /* LED On */
      case 3:  /* LED On */
        break;
    }

  imxrt_gpio_write(GPIO_LED, ledoff); /* Low illuminates */
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Turn off the "logical" LED state
 *
 * Input Parameters:
 *   led - Identifies the "logical" LED state (see definitions in
 *         include/board.h)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      case 0:  /* LED Off */
      case 1:  /* LED Off */
      case 3:  /* LED Off */
        break;

      case 2:  /* LED No change */
        return;
    }

  imxrt_gpio_write(GPIO_LED, true); /* Low illuminates */
}

#endif /* CONFIG_ARCH_LEDS */
