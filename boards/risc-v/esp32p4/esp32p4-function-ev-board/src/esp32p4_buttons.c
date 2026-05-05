/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_buttons.c
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

/* Config */

#include <nuttx/config.h>

/* Libc */

#include <assert.h>
#include <nuttx/debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>

/* NuttX */

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <arch/irq.h>
#include <sys/param.h>

/* Arch */

#include "espressif/esp_gpio.h"
#ifdef CONFIG_ESPRESSIF_TOUCH
#  include "espressif/esp_touch.h"
#endif

/* Board */

#include "esp32p4-function-ev-board.h"
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct button_type_s
{
  bool is_touchpad;
  union
  {
    int channel;
    int gpio;
  } input;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct button_type_s g_buttons[] =
{
  {
    .is_touchpad = false,
    .input.gpio = BUTTON_BOOT
  },
#ifdef CONFIG_ESPRESSIF_TOUCH
#  ifdef CONFIG_ESP_TOUCH_CHANNEL1
  {
    .is_touchpad = true,
    .input.channel = 1
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL2
  {
    .is_touchpad = true,
    .input.channel = 2
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL3
  {
    .is_touchpad = true,
    .input.channel = 3
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL4
  {
    .is_touchpad = true,
    .input.channel = 4
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL5
  {
    .is_touchpad = true,
    .input.channel = 5
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL6
  {
    .is_touchpad = true,
    .input.channel = 6
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL7
  {
    .is_touchpad = true,
    .input.channel = 7
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL8
  {
    .is_touchpad = true,
    .input.channel = 8
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL9
  {
    .is_touchpad = true,
    .input.channel = 9
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL10
  {
    .is_touchpad = true,
    .input.channel = 10
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL11
  {
    .is_touchpad = true,
    .input.channel = 11
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL12
  {
    .is_touchpad = true,
    .input.channel = 12
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL13
  {
    .is_touchpad = true,
    .input.channel = 13
  },
#  endif
#  ifdef CONFIG_ESP_TOUCH_CHANNEL14
  {
    .is_touchpad = true,
    .input.channel = 14
  },
#  endif
#endif
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
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The number of buttons that were initialized.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  int button_num = 1;
#ifdef CONFIG_ESPRESSIF_TOUCH
  int ret = esp_configtouch(&button_num);
  if (ret == OK)
    {
      button_num++;
    }
#endif

  esp_configgpio(BUTTON_BOOT, INPUT_FUNCTION_2 | PULLUP | CHANGE);
  return button_num;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   An 8-bit bit set with each bit associated with a button. See the
 *   BUTTON_*_BIT definitions in board.h for the meaning of each bit.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint8_t ret = 0;
  int i = 0;
  int n = 0;
  bool b0;
  bool b1;

  for (uint8_t btn_id = 0; btn_id < nitems(g_buttons); btn_id++)
    {
      iinfo("Reading button %d\n", btn_id);

      const struct button_type_s button_info = g_buttons[btn_id];

      n = 0;

#ifdef CONFIG_ESPRESSIF_TOUCH
      if (button_info.is_touchpad)
        {
          b0 = esp_touchread(button_info.input.channel);
        }
      else
#endif
        {
          b0 = esp_gpioread(button_info.input.gpio);

          for (i = 0; i < 10; i++)
            {
              up_mdelay(1);

              b1 = esp_gpioread(button_info.input.gpio);

              if (b0 == b1)
                {
                  n++;
                }
              else
                {
                  n = 0;
                }

              if (3 == n)
                {
                  break;
                }

              b0 = b1;
            }
        }

      iinfo("b=%d n=%d\n", b0, n);

      /* Low value means that the button is pressed */

      if (!b0)
        {
          ret |= (1 << btn_id);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released. The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 * Input Parameters:
 *   id         - Identifies the button to be monitored. It is equivalent to
 *                the bit used to report the button state in the return value
 *                from board_buttons().
 *   irqhandler - The handler that will be invoked when the interrupt occurs.
 *   arg        - Pointer to the arguments that will be provided to the
 *                interrupt handler.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  DEBUGASSERT(id < nitems(g_buttons));

  int ret = OK;
  struct button_type_s button_info = g_buttons[id];

#  ifdef CONFIG_ESP_TOUCH_IRQ
  if (button_info.is_touchpad)
    {
      int channel = button_info.input.channel;
      if (NULL != irqhandler)
        {
          /* Make sure the interrupt is disabled */

          ret = esp_touchirqattach(id, irqhandler, arg);
          if (ret < 0)
            {
              ierr("ERROR: esp_touchirqattach() failed: %d\n",
                   ret);
              return ret;
            }

          iinfo("Attach %p to touch pad %d\n", irqhandler, channel);
        }
    }
  else
#  endif
    {
      ret = esp_gpio_irq(button_info.input.gpio, irqhandler, arg);
    }

  return ret;
}
#endif
