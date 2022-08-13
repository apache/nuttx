/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/src/esp32s3_djoystick.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/input/djoystick.h>

#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "esp32s3-devkit.h"

#ifdef CONFIG_INPUT_DJOYSTICK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of Joystick discretes */

#define DJOY_NGPIOS  5

/* Bitset of supported Joystick discretes */

#define DJOY_SUPPORTED (DJOY_UP_BIT | DJOY_DOWN_BIT | DJOY_LEFT_BIT | \
                        DJOY_RIGHT_BIT | DJOY_BUTTON_SELECT_BIT)

/* Pins connected to the 5-WAY joystick */

#define GPIO_JOY_UP      4
#define GPIO_JOY_CENTER  5
#define GPIO_JOY_LEFT    6
#define GPIO_JOY_DOWN    7
#define GPIO_JOY_RIGHT   15

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static djoy_buttonset_t
  djoy_supported(const struct djoy_lowerhalf_s *lower);
static djoy_buttonset_t
  djoy_sample(const struct djoy_lowerhalf_s *lower);
static void djoy_enable(const struct djoy_lowerhalf_s *lower,
                        djoy_buttonset_t press, djoy_buttonset_t release,
                        djoy_interrupt_t handler, void *arg);

static void djoy_disable(void);
static int djoy_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each Olimex-P407 joystick "button."  Index using
 * DJOY_* definitions in include/nuttx/input/djoystick.h.
 */

static const uint16_t g_joygpio[DJOY_NGPIOS] =
{
  GPIO_JOY_UP, GPIO_JOY_DOWN, GPIO_JOY_LEFT, GPIO_JOY_RIGHT, GPIO_JOY_CENTER
};

/* Current interrupt handler and argument */

static djoy_interrupt_t g_djoyhandler;
static void *g_djoyarg;

/* This is the discrete joystick lower half driver interface */

static const struct djoy_lowerhalf_s g_djoylower =
{
  .dl_supported  = djoy_supported,
  .dl_sample     = djoy_sample,
  .dl_enable     = djoy_enable,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: djoy_supported
 *
 * Description:
 *   Return the set of buttons supported on the discrete joystick device
 *
 ****************************************************************************/

static djoy_buttonset_t
  djoy_supported(const struct djoy_lowerhalf_s *lower)
{
  iinfo("Supported: %02x\n", DJOY_SUPPORTED);
  return (djoy_buttonset_t)DJOY_SUPPORTED;
}

/****************************************************************************
 * Name: djoy_sample
 *
 * Description:
 *   Return the current state of all discrete joystick buttons
 *
 ****************************************************************************/

static djoy_buttonset_t djoy_sample(const struct djoy_lowerhalf_s *lower)
{
  djoy_buttonset_t ret = 0;
  int i;

  /* Read each joystick GPIO value */

  for (i = 0; i < DJOY_NGPIOS; i++)
    {
      bool released = esp32s3_gpioread(g_joygpio[i]);
      if (!released)
        {
           ret |= (1 << i);
        }
    }

  iinfo("Retuning: %02x\n", DJOY_SUPPORTED);
  return ret;
}

/****************************************************************************
 * Name: djoy_enable
 *
 * Description:
 *   Enable interrupts on the selected set of joystick buttons.  And empty
 *   set will disable all interrupts.
 *
 ****************************************************************************/

static void djoy_enable(const struct djoy_lowerhalf_s *lower,
                        djoy_buttonset_t press, djoy_buttonset_t release,
                        djoy_interrupt_t handler, void *arg)
{
  irqstate_t flags;
  djoy_buttonset_t either = press | release;
  djoy_buttonset_t bit;
  bool rising;
  bool falling;
  int i;

  /* Start with all interrupts disabled */

  flags = enter_critical_section();
  djoy_disable();

  iinfo("press: %02x release: %02x handler: %p arg: %p\n",
        press, release, handler, arg);

  /* If no events are indicated or if no handler is provided, then this
   * must really be a request to disable interrupts.
   */

  if (either && handler)
    {
      /* Save the new the handler and argument */

      g_djoyhandler = handler;
      g_djoyarg     = arg;

      /* Check each GPIO. */

      for (i = 0; i < DJOY_NGPIOS; i++)
        {
          int ret;

          /* Get the irq associated to each pin */

          int irq = ESP32S3_PIN2IRQ(g_joygpio[i]);

          /* Make sure the interrupt is disabled */

          esp32s3_gpioirqdisable(irq);

          ret = irq_attach(irq, djoy_interrupt, arg);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
              return;
            }

          /* Enable interrupts on each pin that has either a press or
           * release event associated with it.
           */

          bit = (1 << i);
          if ((either & bit) != 0)
            {
              /* Active low so a press corresponds to a falling edge and
               * a release corresponds to a rising edge.
               */

              falling = ((press & bit) != 0);
              rising  = ((release & bit) != 0);

              iinfo("GPIO %d: rising: %d falling: %d\n",
                     i, rising, falling);

              if (falling != 0 && rising != 0)
                {
                  esp32s3_gpioirqenable(irq, GPIO_INTR_ANYEDGE);
                }
              else if (falling != 0)
                {
                  esp32s3_gpioirqenable(irq, GPIO_INTR_NEGEDGE);
                }
              else
                {
                  esp32s3_gpioirqenable(irq, GPIO_INTR_POSEDGE);
                }
            }
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: djoy_disable
 *
 * Description:
 *   Disable all joystick interrupts
 *
 ****************************************************************************/

static void djoy_disable(void)
{
  irqstate_t flags;
  int i;

  /* Disable each joystick interrupt */

  flags = enter_critical_section();
  for (i = 0; i < DJOY_NGPIOS; i++)
    {
      /* Get the irq associated to each pin */

      int irq = ESP32S3_PIN2IRQ(g_joygpio[i]);

      /* Disable the interrupt */

      esp32s3_gpioirqdisable(irq);
    }

  leave_critical_section(flags);

  /* Nullify the handler and argument */

  g_djoyhandler = NULL;
  g_djoyarg     = NULL;
}

/****************************************************************************
 * Name: djoy_interrupt
 *
 * Description:
 *   Discrete joystick interrupt handler
 *
 ****************************************************************************/

static int djoy_interrupt(int irq, void *context, void *arg)
{
  DEBUGASSERT(g_djoyhandler);
  if (g_djoyhandler)
    {
      g_djoyhandler(&g_djoylower, g_djoyarg);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_djoy_initialize
 *
 * Description:
 *   Initialize and register the discrete joystick driver
 *
 ****************************************************************************/

int esp32s3_djoy_initialize(void)
{
  int i;

  /* Configure the GPIO pins as inputs */

  for (i = 0; i < DJOY_NGPIOS; i++)
    {
      esp32s3_configgpio(g_joygpio[i], INPUT_FUNCTION_2 | PULLUP);
    }

  /* Make sure that all interrupts are disabled */

  djoy_disable();

  /* Register the joystick device as /dev/djoy0 */

  return djoy_register("/dev/djoy0", &g_djoylower);
}

#endif /* CONFIG_INPUT_DJOYSTICK */
