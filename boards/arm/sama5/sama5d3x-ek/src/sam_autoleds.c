/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/src/sam_autoleds.c
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

/* There are two LEDs on the SAMA5D3 series-CM board that can be controlled
 * by software.  A  blue LED is controlled via PIO pins.  A red LED normally
 * provides an indication that power is supplied to the board but can also
 * be controlled via software.
 *
 *   PE25.  This blue LED is pulled high and is illuminated by pulling PE25
 *   low.
 *
 *   PE24.  The red LED is also pulled high but is driven by a transistor so
 *   that it is illuminated when power is applied even if PE24 is not
 *   configured as an output.  If PE24 is configured as an output, then the
 *   LCD is illuminated by a low output.
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL            Val Meaning                     LED state
 *                                                    Blue     Red
 *   ----------------- --- -----------------------  -------- --------
 *   LED_STARTED        0  NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE   0  Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED    0  Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED   1  Idle stack created         ON       OFF
 *   LED_INIRQ          2  In an interrupt              No change
 *   LED_SIGNAL         2  In a signal handler          No change
 *   LED_ASSERTION      2  An assertion failed          No change
 *   LED_PANIC          3  The system has crashed     OFF      Blinking
 *   LED_IDLE          N/A MCU is is sleep mode         Not used
 *
 * Thus if the blue LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the red is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
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

#include "sam_pio.h"
#include "sama5d3x-ek.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED PIOs for output */

  sam_configpio(PIO_BLUE);
#ifndef CONFIG_SAMA5D3XEK_NOREDLED
  sam_configpio(PIO_RED);
#endif
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool blueoff = true;  /* Low illuminates */
#ifndef CONFIG_SAMA5D3XEK_NOREDLED
  bool redon   = false; /* High illuminates */
#endif

  switch (led)
    {
      case 0:  /* LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED */
        break;

      case 1:  /* LED_STACKCREATED */
        blueoff = false;
        break;

      default:
      case 2:  /* LED_INIRQ, LED_SIGNAL, LED_ASSERTION */
        return;

      case 3:  /* LED_PANIC */
#ifdef CONFIG_SAMA5D3XEK_NOREDLED
        blueoff = false;
#else
        redon = true;
#endif
        break;
    }

  sam_piowrite(PIO_BLUE, blueoff);
#ifndef CONFIG_SAMA5D3XEK_NOREDLED
  sam_piowrite(PIO_RED, redon);
#endif
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led != 2)
    {
      sam_piowrite(PIO_BLUE, true);  /* Low illuminates */
#ifndef CONFIG_SAMA5D3XEK_NOREDLED
      sam_piowrite(PIO_RED, false);  /* High illuminates */
#endif
    }
}

#endif /* CONFIG_ARCH_LEDS */
