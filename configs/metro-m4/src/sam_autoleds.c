/****************************************************************************
 * configs/metro-m4/src/sam_autoleds.c
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

/* The Adafruit Metro M4 has four LEDs, but only two are controllable by software:
 *
 *   1. The red LED on the Arduino D13 pin, and
 *   2. A NeoPixel RGB LED.
 *
 * Currently, only the red LED is supported.
 *
 *   ------ ----------------- -----------
 *   SHIELD SAMD5E5           FUNCTION
 *   ------ ----------------- -----------
 *   D13    PA16              GPIO output
 *
 * This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_autoleds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *   ------------------- ---------------------------- ------
 *   SYMBOL                  Meaning                  LED
 *   ------------------- ---------------------------- ------
 *
 *   LED_STARTED         NuttX has been started       OFF
 *   LED_HEAPALLOCATE    Heap has been allocated      OFF
 *   LED_IRQSENABLED     Interrupts enabled           OFF
 *   LED_STACKCREATED    Idle stack created           ON
 *   LED_INIRQ           In an interrupt              N/C
 *   LED_SIGNAL          In a signal handler          N/C
 *   LED_ASSERTION       An assertion failed          N/C
 *   LED_PANIC           The system has crashed       FLASH
 *   LED_IDLE            MCU is is sleep mode         Not used
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/power/pm.h>

#include "up_arch.h"
#include "up_internal.h"
#include "sam_port.h"
#include "metro-m4.h"

#include <arch/board/board.h>

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

 /* LED Power Management */

#ifdef CONFIG_PM
static void led_pm_notify(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
static int led_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PM
static struct pm_callback_s g_ledscb =
{
  .notify  = led_pm_notify,
  .prepare = led_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void led_pm_notify(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Restore normal LEDs operation */

        }
        break;

      case(PM_IDLE):
        {
          /* Entering IDLE mode - Turn leds off */

        }
        break;

      case(PM_STANDBY):
        {
          /* Entering STANDBY mode - Logic for PM_STANDBY goes here */

        }
        break;

      case(PM_SLEEP):
        {
          /* Entering SLEEP mode - Logic for PM_SLEEP goes here */

        }
        break;

      default:
        {
          /* Should not get here */

        }
        break;
    }
}
#endif

/****************************************************************************
 * Name: led_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int led_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate)
{
  /* No preparation to change power modes is required by the LEDs driver.
   * We always accept the state change by returning OK.
   */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  (void)sam_portconfig(PORT_RED_LED);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool ledstate = true;

  switch (led)
    {
    case 0:                   /* LED_STARTED:      NuttX has been started  STATUS LED=OFF */
                              /* LED_HEAPALLOCATE: Heap has been allocated STATUS LED=OFF */
                              /* LED_IRQSENABLED:  Interrupts enabled      STATUS LED=OFF */
      break;                  /* Leave ledstate == true to turn OFF */

    default:
    case 2:                   /* LED_INIRQ:        In an interrupt         STATUS LED=N/C */
                              /* LED_SIGNAL:       In a signal handler     STATUS LED=N/C */
                              /* LED_ASSERTION:    An assertion failed     STATUS LED=N/C */
      return;                 /* Return to leave STATUS LED unchanged */

    case 3:                   /* LED_PANIC:        The system has crashed  STATUS LED=FLASH */
    case 1:                   /* LED_STACKCREATED: Idle stack created      STATUS LED=ON */
      ledstate = false;       /* Set ledstate == false to turn ON */
      break;
    }

  sam_portwrite(PORT_RED_LED, ledstate);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
    /* These should not happen and are ignored */

    default:
    case 0:                   /* LED_STARTED:      NuttX has been started  STATUS LED=OFF */
                              /* LED_HEAPALLOCATE: Heap has been allocated STATUS LED=OFF */
                              /* LED_IRQSENABLED:  Interrupts enabled      STATUS LED=OFF */
    case 1:                   /* LED_STACKCREATED: Idle stack created      STATUS LED=ON */

    /* These result in no-change */

    case 2:                   /* LED_INIRQ:        In an interrupt         STATUS LED=N/C */
                              /* LED_SIGNAL:       In a signal handler     STATUS LED=N/C */
                              /* LED_ASSERTION:    An assertion failed     STATUS LED=N/C */
      return;                 /* Return to leave STATUS LED unchanged */

    /* Turn STATUS LED off set driving the output high */

    case 3:                   /* LED_PANIC:        The system has crashed  STATUS LED=FLASH */
      sam_portwrite(PORT_RED_LED, true);
      break;
    }
}

/****************************************************************************
 * Name: sam_led_pminitialize
 *
 * Description:
 *   Register LED power management features.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void sam_led_pminitialize(void)
{
  /* Register to receive power management callbacks */

  int ret = pm_register(&g_ledscb);
  if (ret != OK)
    {
      board_autoled_on(LED_ASSERTION);
    }
}
#endif /* CONFIG_PM */

#endif /* CONFIG_ARCH_LEDS */
