/****************************************************************************
 * boards/xtensa/esp32s2/esp32s2-kaluga-1/src/esp32s2_buttons.c
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
#include <nuttx/signal.h>
#include <arch/irq.h>

#include "esp32s2_gpio.h"
#include "esp32s2_rtc_gpio.h"
#include "esp32s2_touch.h"

#include "esp32s2-kaluga-1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define TOUCHPAD_REFH               (TOUCH_HVOLT_2V7)
#define TOUCHPAD_REFL               (TOUCH_LVOLT_0V5)
#define TOUCHPAD_ATTEN              (TOUCH_HVOLT_ATTEN_1V)
#define TOUCHPAD_SLOPE              (TOUCH_SLOPE_7)
#define TOUCHPAD_TIE_OPT            (TOUCH_TIE_OPT_LOW)
#define TOUCHPAD_FSM_MODE           (TOUCH_FSM_MODE_TIMER)
#define TOUCHPAD_DENOISE_GRADE      (TOUCH_DENOISE_BIT4)
#define TOUCHPAD_DENOISE_CAP        (TOUCH_DENOISE_CAP_L4)
#define TOUCHPAD_FILTER_MODE        (TOUCH_FILTER_IIR_16)
#define TOUCHPAD_FILTER_DEBOUNCE    (1)
#define TOUCHPAD_FILTER_NOISE       (0)
#define TOUCHPAD_FILTER_JITTER      (4)
#define TOUCHPAD_FILTER_SMH         (TOUCH_SMOOTH_IIR_2)
#define TOUCHPAD_THRESHOLD          (30000)

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

#ifdef CONFIG_ESP32S2_TOUCH
static const struct touch_config_s tp_config =
{
  .refh = TOUCHPAD_REFH,
  .refl = TOUCHPAD_REFL,
  .atten = TOUCHPAD_ATTEN,
  .slope = TOUCHPAD_SLOPE,
  .tie_opt = TOUCHPAD_TIE_OPT,
  .fsm_mode = TOUCHPAD_FSM_MODE,
#  ifdef CONFIG_ESP32S2_TOUCH_FILTER
  .filter_mode = TOUCHPAD_FILTER_MODE,
  .filter_debounce_cnt = TOUCHPAD_FILTER_DEBOUNCE,
  .filter_noise_thr = TOUCHPAD_FILTER_NOISE,
  .filter_jitter_step = TOUCHPAD_FILTER_JITTER,
  .filter_smh_lvl = TOUCHPAD_FILTER_SMH,
#  endif
#  ifdef CONFIG_ESP32S2_TOUCH_DENOISE
  .denoise_grade = TOUCHPAD_DENOISE_GRADE,
  .denoise_cap_level = TOUCHPAD_DENOISE_CAP
#  endif
};
#endif

static const struct button_type_s g_buttons[] =
{
  {
    .is_touchpad = false,
    .input.gpio = BUTTON_BOOT
  },
#ifdef CONFIG_ESP32S2_TOUCH
  {
    .is_touchpad = true,
    .input.channel = TP_PHOTO_CHANNEL
  },
  {
    .is_touchpad = true,
    .input.channel = TP_PLAY_CHANNEL
  },
  {
    .is_touchpad = true,
    .input.channel = TP_RECORD_CHANNEL
  },
  {
    .is_touchpad = true,
    .input.channel = TP_NETWORK_CHANNEL
  },
  {
    .is_touchpad = true,
    .input.channel = TP_VOLUP_CHANNEL
  },
  {
    .is_touchpad = true,
    .input.channel = TP_VOLDN_CHANNEL
  }
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
#ifdef CONFIG_ESP32S2_TOUCH
  esp32s2_configtouch(TP_PHOTO_CHANNEL, tp_config);
  esp32s2_configtouch(TP_PLAY_CHANNEL, tp_config);
  esp32s2_configtouch(TP_RECORD_CHANNEL, tp_config);
  esp32s2_configtouch(TP_NETWORK_CHANNEL, tp_config);
  esp32s2_configtouch(TP_VOLUP_CHANNEL, tp_config);
  esp32s2_configtouch(TP_VOLDN_CHANNEL, tp_config);

  esp32s2_touchsetthreshold(TP_PHOTO_CHANNEL, TOUCHPAD_THRESHOLD);
  esp32s2_touchsetthreshold(TP_PLAY_CHANNEL, TOUCHPAD_THRESHOLD);
  esp32s2_touchsetthreshold(TP_RECORD_CHANNEL, TOUCHPAD_THRESHOLD);
  esp32s2_touchsetthreshold(TP_NETWORK_CHANNEL, TOUCHPAD_THRESHOLD);
  esp32s2_touchsetthreshold(TP_VOLUP_CHANNEL, TOUCHPAD_THRESHOLD);
  esp32s2_touchsetthreshold(TP_VOLDN_CHANNEL, TOUCHPAD_THRESHOLD);

#endif
  esp32s2_configgpio(BUTTON_BOOT, INPUT_FUNCTION_3 | PULLUP);
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

  for (uint8_t btn_id = 0; btn_id < ARRAY_SIZE(g_buttons); btn_id++)
    {
      iinfo("Reading button %d\n", btn_id);

      const struct button_type_s button_info = g_buttons[btn_id];

      n = 0;

      if (button_info.is_touchpad)
        {
          b0 = esp32s2_touchread(button_info.input.channel);
        }
      else
        {
          b0 = esp32s2_gpioread(button_info.input.gpio);

          for (int i = 0; i < 10; i++)
            {
              up_mdelay(1);

              b1 = esp32s2_gpioread(button_info.input.gpio);

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
  DEBUGASSERT(id < ARRAY_SIZE(g_buttons));

  int ret;
  struct button_type_s button_info = g_buttons[id];

#ifdef CONFIG_ESP32S2_TOUCH_IRQ
  if (button_info.is_touchpad)
    {
      int channel = button_info.input.channel;
      int irq = ESP32S2_TOUCHPAD2IRQ(channel);

      if (NULL != irqhandler)
        {
          /* Make sure the interrupt is disabled */

          esp32s2_touchirqdisable(irq);

          esp32s2_touchregisterreleasecb(irqhandler);
          ret = irq_attach(irq, irqhandler, arg);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
              return ret;
            }

          iinfo("Attach %p to touch pad %d\n", irqhandler, channel);

          iinfo("Enabling the interrupt\n");

          esp32s2_touchirqenable(irq);
        }
      else
        {
          iinfo("Disabled interrupts from touch pad %d\n", channel);
          esp32s2_touchirqdisable(irq);
        }

      return OK;
    }
  else
#endif
    {
      int pin = button_info.input.gpio;
      int irq = ESP32S2_PIN2IRQ(pin);

      if (NULL != irqhandler)
        {
          /* Make sure the interrupt is disabled */

          esp32s2_gpioirqdisable(irq);

          ret = irq_attach(irq, irqhandler, arg);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
              return ret;
            }

          gpioinfo("Attach %p to pin %d\n", irqhandler, pin);

          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising and falling edges */

          esp32s2_gpioirqenable(irq, GPIO_INTR_ANYEDGE);
        }
      else
        {
          gpioinfo("Disabled interrupts from pin %d\n", pin);
          esp32s2_gpioirqdisable(irq);
        }

      return OK;
    }
}
#endif
