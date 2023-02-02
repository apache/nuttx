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

#include <sys/param.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "esp32_gpio.h"
#include "esp32_touch.h"

#include "esp32-lyrat.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TOUCHPAD_REFH               (TOUCH_HVOLT_2V7)
#define TOUCHPAD_REFL               (TOUCH_LVOLT_0V5)
#define TOUCHPAD_ATTEN              (TOUCH_HVOLT_ATTEN_1V)
#define TOUCHPAD_SLOPE              (TOUCH_SLOPE_7)
#define TOUCHPAD_TIE_OPT            (TOUCH_TIE_OPT_LOW)
#ifdef CONFIG_ESP32_TOUCH_IRQ
#  define TOUCHPAD_FSM_MODE         (TOUCH_FSM_MODE_TIMER)
#else
#  define TOUCHPAD_FSM_MODE         (TOUCH_FSM_MODE_SW)
#endif
#define TOUCHPAD_THRESHOLD_CALC(v)  (v * 2 / 3)
#define TOUCHPAD_FILTER_PERIOD      (10)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  bool is_touchpad;
  union
  {
    int channel;
    int gpio;
  } input;
} button_type_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ESP32_TOUCH
static const struct touch_config_s tp_config =
{
  .refh = TOUCHPAD_REFH,
  .refl = TOUCHPAD_REFL,
  .atten = TOUCHPAD_ATTEN,
  .slope = TOUCHPAD_SLOPE,
  .tie_opt = TOUCHPAD_TIE_OPT,
  .fsm_mode = TOUCHPAD_FSM_MODE,
  .filter_period = TOUCHPAD_FILTER_PERIOD
};
#endif

static const button_type_t g_buttons[] =
{
  {
    .is_touchpad = false,
    .input.gpio = BUTTON_REC
  },
  {
    .is_touchpad = false,
    .input.gpio = BUTTON_MODE
  },
#ifdef CONFIG_ESP32_TOUCH
  {
    .is_touchpad = true,
    .input.channel = BUTTON_PLAY_TP_CHANNEL
  },
  {
    .is_touchpad = true,
    .input.channel = BUTTON_SET_TP_CHANNEL
  },
  {
    .is_touchpad = true,
    .input.channel = BUTTON_VOLM_TP_CHANNEL
  },
  {
    .is_touchpad = true,
    .input.channel = BUTTON_VOLP_TP_CHANNEL
  },
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
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
#ifdef CONFIG_ESP32_TOUCH
  uint16_t raw_value;

  esp32_configtouch(BUTTON_PLAY_TP_CHANNEL, tp_config);
  esp32_configtouch(BUTTON_SET_TP_CHANNEL, tp_config);
  esp32_configtouch(BUTTON_VOLM_TP_CHANNEL, tp_config);
  esp32_configtouch(BUTTON_VOLP_TP_CHANNEL, tp_config);

  raw_value = esp32_touchreadraw(BUTTON_PLAY_TP_CHANNEL);
  esp32_touchsetthreshold(BUTTON_PLAY_TP_CHANNEL,
                          TOUCHPAD_THRESHOLD_CALC(raw_value));

  raw_value = esp32_touchreadraw(BUTTON_SET_TP_CHANNEL);
  esp32_touchsetthreshold(BUTTON_SET_TP_CHANNEL,
                          TOUCHPAD_THRESHOLD_CALC(raw_value));

  raw_value = esp32_touchreadraw(BUTTON_VOLM_TP_CHANNEL);
  esp32_touchsetthreshold(BUTTON_VOLM_TP_CHANNEL,
                          TOUCHPAD_THRESHOLD_CALC(raw_value));

  raw_value = esp32_touchreadraw(BUTTON_VOLP_TP_CHANNEL);
  esp32_touchsetthreshold(BUTTON_VOLP_TP_CHANNEL,
                          TOUCHPAD_THRESHOLD_CALC(raw_value));
#endif

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
  bool b0;
  bool b1;
  int n;

  for (uint8_t btn_id = 0; btn_id < nitems(g_buttons); btn_id++)
    {
      iinfo("Reading button %d\n", btn_id);

      const button_type_t button_info = g_buttons[btn_id];

      n = 0;

      if (button_info.is_touchpad)
        {
          b0 = esp32_touchread(button_info.input.channel);
        }
      else
        {
          b0 = esp32_gpioread(button_info.input.gpio);

          for (int i = 0; i < 10; i++)
            {
              up_mdelay(1);

              if (button_info.is_touchpad)
                {
                  b1 = esp32_touchread(button_info.input.channel);
                }
              else
                {
                  b1 = esp32_gpioread(button_info.input.gpio);
                }

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
 *   will be called when a button is depressed or released.  The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  DEBUGASSERT(id < nitems(g_buttons));

  int ret;
  button_type_t button_info = g_buttons[id];

#ifdef CONFIG_ESP32_TOUCH_IRQ
  if (button_info.is_touchpad)
    {
      int channel = button_info.input.channel;
      int irq = ESP32_TOUCHPAD2IRQ(channel);

      if (NULL != irqhandler)
        {
          /* Make sure the interrupt is disabled */

          esp32_touchirqdisable(irq);

          esp32_touchregisterreleasecb(irqhandler);
          ret = irq_attach(irq, irqhandler, arg);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
              return ret;
            }

          iinfo("Attach %p to touch pad %d\n", irqhandler, channel);

          iinfo("Enabling the interrupt\n");

          esp32_touchirqenable(irq);
        }
      else
        {
          iinfo("Disabled interrupts from touch pad %d\n", channel);
          esp32_touchirqdisable(irq);
        }

      return OK;
    }
  else
#endif
    {
      int pin = button_info.input.gpio;
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
}
#endif
