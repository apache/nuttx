/****************************************************************************
 * boards/arm/samd5e5/metro-m4/src/sam_autoleds.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* The Adafruit Metro M4 has four LEDs, but only two are controllable by
 * software:
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
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/power/pm.h>

#include "arm_internal.h"
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
  sam_portconfig(PORT_RED_LED);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool ledstate = true;

  switch (led)
    {
    case 0:   /* LED_STARTED:      NuttX has been started  STATUS LED=OFF
               * LED_HEAPALLOCATE: Heap has been allocated STATUS LED=OFF
               * LED_IRQSENABLED:  Interrupts enabled      STATUS LED=OFF
               */

      break;  /* Leave ledstate == true to turn OFF */

    default:
    case 2:   /* LED_INIRQ:        In an interrupt         STATUS LED=N/C
               * LED_SIGNAL:       In a signal handler     STATUS LED=N/C
               * LED_ASSERTION:    An assertion failed     STATUS LED=N/C
               */

      return; /* Return to leave STATUS LED unchanged */

    case 3:   /* LED_PANIC:        The system has crashed  STATUS LED=FLASH */
    case 1:   /* LED_STACKCREATED: Idle stack created      STATUS LED=ON */

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
    case 0:   /* LED_STARTED:      NuttX has been started  STATUS LED=OFF
               * LED_HEAPALLOCATE: Heap has been allocated STATUS LED=OFF
               * LED_IRQSENABLED:  Interrupts enabled      STATUS LED=OFF
               */

    case 1:   /* LED_STACKCREATED: Idle stack created      STATUS LED=ON */

    /* These result in no-change */

    case 2:   /* LED_INIRQ:        In an interrupt         STATUS LED=N/C
               * LED_SIGNAL:       In a signal handler     STATUS LED=N/C
               * LED_ASSERTION:    An assertion failed     STATUS LED=N/C
               */

      return; /* Return to leave STATUS LED unchanged */

    /* Turn STATUS LED off set driving the output high */

    case 3:   /* LED_PANIC:        The system has crashed  STATUS LED=FLASH */
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
