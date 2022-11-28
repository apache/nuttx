/****************************************************************************
 * boards/xtensa/esp32/esp32-lyrat/src/esp32_buttons.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "esp32_gpio.h"

#include "esp32-lyrat.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const int g_buttons[] =
{
  BUTTON_REC,
  BUTTON_MODE
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
  /* GPIOs 36 and 39 do not support PULLUP/PULLDOWN */

  esp32_configgpio(BUTTON_MODE, INPUT_FUNCTION_3);
  esp32_configgpio(BUTTON_REC, INPUT_FUNCTION_3);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   8-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT  definitions in board.h for the meaning of each bit.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint8_t ret = 0;
  int n = 0;

  for (uint8_t btn_id = 0; btn_id < ARRAY_SIZE(g_buttons); btn_id++)
    {
      iinfo("Reading button %d\n", btn_id);

      const int button_gpio = g_buttons[btn_id];
      bool b0 = esp32_gpioread(button_gpio);

      for (int i = 0; i < 10; i++)
        {
          up_mdelay(1); /* TODO */

          bool b1 = esp32_gpioread(button_gpio);

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
 *   will be called when a button is depressed or released.  The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  DEBUGASSERT(id < ARRAY_SIZE(g_buttons));

  int ret;
  int pin = g_buttons[id];
  int irq = ESP32_PIN2IRQ(pin);

  if (NULL != irqhandler)
    {
      /* Make sure the interrupt is disabled */

      esp32_gpioirqdisable(irq);

      ret = irq_attach(irq, irqhandler, arg);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
          return ret;
        }

      gpioinfo("Attach %p to pin %d\n", irqhandler, pin);

      gpioinfo("Enabling the interrupt\n");

      /* Configure the interrupt for rising and falling edges */

      esp32_gpioirqenable(irq, CHANGE);
    }
  else
    {
      gpioinfo("Disabled interrupts from pin %d\n", pin);
      esp32_gpioirqdisable(irq);
    }

  return OK;
}
#endif
