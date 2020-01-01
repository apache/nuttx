/****************************************************************************
 * boards/arm/stm32/stm3210e-eval/src/stm32_djoystick.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/input/djoystick.h>

#include "stm32_gpio.h"
#include "stm3210e-eval.h"

#ifdef CONFIG_DJOYSTICK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Number of Joystick discretes */

#define DJOY_NGPIOS  5

/* Bitset of supported Joystick discretes */

#define DJOY_SUPPORTED (DJOY_UP_BIT | DJOY_DOWN_BIT | DJOY_LEFT_BIT | \
                        DJOY_RIGHT_BIT | DJOY_BUTTON_SELECT_BIT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static djoy_buttonset_t djoy_supported(FAR const struct djoy_lowerhalf_s *lower);
static djoy_buttonset_t djoy_sample(FAR const struct djoy_lowerhalf_s *lower);
static void djoy_enable(FAR const struct djoy_lowerhalf_s *lower,
                         djoy_buttonset_t press, djoy_buttonset_t release,
                         djoy_interrupt_t handler, FAR void *arg);

static void djoy_disable(void);
static int djoy_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Pin configuration for each STM3210E-EVAL joystick "button."  Index using
 * DJOY_* definitions in include/nuttx/input/djoystick.h.
 */

static const uint16_t g_joygpio[DJOY_NGPIOS] =
{
  GPIO_JOY_UP, GPIO_JOY_DOWN, GPIO_JOY_LEFT, GPIO_JOY_RIGHT, GPIO_JOY_SEL
};

/* Current interrupt handler and argument */

static djoy_interrupt_t g_djoyhandler;
static FAR void *g_djoyarg;

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

static djoy_buttonset_t djoy_supported(FAR const struct djoy_lowerhalf_s *lower)
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

static djoy_buttonset_t djoy_sample(FAR const struct djoy_lowerhalf_s *lower)
{
  djoy_buttonset_t ret = 0;
  int i;

  /* Read each joystick GPIO value */

  for (i = 0; i < DJOY_NGPIOS; i++)
    {
       bool released = stm32_gpioread(g_joygpio[i]);
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

static void djoy_enable(FAR const struct djoy_lowerhalf_s *lower,
                         djoy_buttonset_t press, djoy_buttonset_t release,
                         djoy_interrupt_t handler, FAR void *arg)
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

               stm32_gpiosetevent(g_joygpio[i], rising, falling,
                                  true, djoy_interrupt, NULL);
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
      stm32_gpiosetevent(g_joygpio[i], false, false, false, NULL, NULL);
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

static int djoy_interrupt(int irq, FAR void *context, FAR void *arg)
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
 * Name: stm32_djoy_initialization
 *
 * Description:
 *   Initialize and register the discrete joystick driver
 *
 ****************************************************************************/

int stm32_djoy_initialization(void)
{
  int i;

  /* Configure the GPIO pins as inputs.    NOTE: This is unnecessary for
   * interrupting pins since it will also be done by stm32_gpiosetevent().
   */

  for (i = 0; i < DJOY_NGPIOS; i++)
    {
      stm32_configgpio(g_joygpio[i]);
    }

  /* Make sure that all interrupts are disabled */

  djoy_disable();

  /* Register the joystick device as /dev/djoy0 */

  return djoy_register("/dev/djoy0", &g_djoylower);
}

#endif /* CONFIG_DJOYSTICK */
