/****************************************************************************
 * boards/arm/samd2l2/arduino-m0/src/sam_autoleds.c
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

/* There is a LED on board the Arduino M0 board.
 *
 * When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 * control the LED as follows:
 *
 *   SYMBOL              Meaning                 LED
 *   ------------------- ----------------------- ------
 *   LED_STARTED         NuttX has been started  OFF
 *   LED_HEAPALLOCATE    Heap has been allocated OFF
 *   LED_IRQSENABLED     Interrupts enabled      OFF
 *   LED_STACKCREATED    Idle stack created      ON
 *   LED_INIRQ           In an interrupt**       N/C
 *   LED_SIGNAL          In a signal handler***  N/C
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
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "sam_port.h"
#include "arduino_m0.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  sam_configport(PORT_STATUS_LED);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool ledstate = false;

  switch (led)
    {
    case 0:   /* LED_STARTED:      NuttX has been started  STATUS LED=ON
               * LED_HEAPALLOCATE: Heap has been allocated STATUS LED=ON
               * LED_IRQSENABLED:  Interrupts enabled      STATUS LED=ON
               */

      break;   /* Leave ledstate == true to turn OFF */

    default:
    case 2:   /* LED_INIRQ:        In an interrupt         STATUS LED=N/C
               * LED_SIGNAL:       In a signal handler     STATUS LED=N/C
               * LED_ASSERTION:    An assertion failed     STATUS LED=N/C
               */

      return; /* Return to leave STATUS LED unchanged */

    case 3:   /* LED_PANIC:        The system has crashed  STATUS LED=FLASH */
    case 1:   /* LED_STACKCREATED: Idle stack created      STATUS LED=ON */

      ledstate = true;       /* Set ledstate == false to turn OFF */
      break;
    }

  sam_portwrite(PORT_STATUS_LED, ledstate);
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
      sam_portwrite(PORT_STATUS_LED, false);
      break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
