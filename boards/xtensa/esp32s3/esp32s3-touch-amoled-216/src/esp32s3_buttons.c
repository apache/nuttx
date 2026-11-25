/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-touch-amoled-216/src/esp32s3_buttons.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "espressif/esp_gpio.h"

#include "esp32s3-touch-amoled-216.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct button_desc_s
{
  uint8_t gpio;       /* GPIO the key is wired to */
  bool    activehigh; /* True if the GPIO reads high while pressed */
  bool    pullup;     /* True if the internal pull-up should be enabled */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Indexed by the BUTTON_* identifiers in board.h.  BOOT and IO18 have
 * external 10K pull-ups to 3V3; the internal pull-up is enabled as well so
 * that the reading is defined even if those resistors are not fitted.
 * GPIO16 is driven by a MOSFET with an external pull-up, so no internal
 * bias is applied there.
 */

static const struct button_desc_s g_buttons[NUM_BUTTONS] =
{
  [BUTTON_BOOT] =
  {
    GPIO_BUTTON_BOOT, false, true
  },
  [BUTTON_IO18] =
  {
    GPIO_BUTTON_IO18, false, true
  },
  [BUTTON_PWR] =
  {
    GPIO_BUTTON_PWR, true, false
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  int i;

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      gpio_pinattr_t attr = INPUT_FUNCTION_2 | CHANGE;

      if (g_buttons[i].pullup)
        {
          attr |= PULLUP;
        }

      esp_configgpio(g_buttons[i].gpio, attr);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns a
 *   bit set with each bit associated with a button.  See the BUTTON_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i;

  /* This may be called from the GPIO interrupt: just sample the keys,
   * debouncing is left to the upper half
   * (CONFIG_INPUT_BUTTONS_DEBOUNCE_DELAY).
   */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      if (esp_gpioread(g_buttons[i].gpio) == g_buttons[i].activehigh)
        {
          ret |= (1 << i);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released.  The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  if (id < 0 || id >= NUM_BUTTONS)
    {
      return -EINVAL;
    }

  return esp_gpio_irq(g_buttons[id].gpio, irqhandler, arg);
}
#endif
