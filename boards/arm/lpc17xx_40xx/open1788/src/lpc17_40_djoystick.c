/****************************************************************************
 * boards/arm/lpc17xx_40xx/open1788/src/lpc17_40_djoystick.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/input/djoystick.h>

#include "lpc17_40_gpio.h"
#include "open1788.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The Open1788 supports several buttons.  All will read "1" when open and "0"
 * when closed
 *
 *   USER1           Connected to P4[26]
 *   USER2           Connected to P2[22]
 *   USER3           Connected to P0[10]
 *
 * And a Joystick
 *
 *   JOY_A           Connected to P2[25]
 *   JOY_B           Connected to P2[26]
 *   JOY_C           Connected to P2[23]
 *   JOY_D           Connected to P2[19]
 *   JOY_CTR         Connected to P0[14]
 *
 * The switches are all connected to ground and should be pulled up and sensed
 * with a value of '0' when closed.
 *
 * Mapping to DJOYSTICK buttons:
 *
 *   DJOY_UP         JOY_B
 *   DJOY_DOWN       JOY_C
 *   DJOY_LEFT       JOY_A
 *   DJOY_RIGHT      JOY_D
 *   DJOY_BUTTON_1   JOY_CTR
 *   DJOY_BUTTON_2   USER1
 *   DJOY_BUTTON_3   USER2
 *   DJOY_BUTTON_4   USER3
 */

/* Number of Joystick discretes */

#define DJOY_NGPIOS  8

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
/* Pin configuration for each Open1788 joystick "button."  Indexed using
 * DJOY_* definitions in include/nuttx/input/djoystick.h.
 */

static const lpc17_40_pinset_t g_joygpio[DJOY_NGPIOS] =
{
  GPIO_JOY_B,   GPIO_JOY_C, GPIO_JOY_A, GPIO_JOY_D,
  GPIO_JOY_CTR, GPIO_USER1, GPIO_USER2, GPIO_USER3
};

#ifdef CONFIG_LPC17_40_GPIOIRQ
/* This array provides the mapping from button ID numbers to button IRQ
 * numbers.  Indexed using DJOY_* definitions in
 * include/nuttx/input/djoystick.h.
 */

static const uint8_t g_buttonirq[DJOY_NGPIOS] =
{
  GPIO_JOY_B_IRQ,   GPIO_JOY_C_IRQ, GPIO_JOY_A_IRQ, GPIO_JOY_D_IRQ,
  GPIO_JOY_CTR_IRQ, 0,              GPIO_USER2_IRQ, GPIO_USER3_IRQ
};
#endif

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
  iinfo("Supported: %02x\n", DJOY_ALLBITS);
  return (djoy_buttonset_t)DJOY_ALLBITS;
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
      /* A LOW value means that the key is pressed. */

      bool released = lpc17_40_gpioread(g_joygpio[i]);

      /* Accumulate the set of depressed (not released) keys */

      if (!released)
        {
            ret |= (1 << i);
        }
    }

  iinfo("Returning: %02x\n", DJOY_ALLBITS);
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
#ifdef CONFIG_LPC17_40_GPIOIRQ
  irqstate_t flags;
  djoy_buttonset_t either = press | release;
  int irq;
  int i;

  iinfo("press: %02x release: %02x handler: %p arg: %p\n",
        press, release, handler, arg);

  /* Start with all interrupts disabled */

  flags = enter_critical_section();
  djoy_disable();

  /* If no events are indicated or if no handler is provided, then this
   * must really be a request to disable interrupts.
   */

  /* REVISIT: Currently does not distinguish press/release selections */

  if (either && handler != NULL)
    {
      /* Save the new the handler and argument */

      g_djoyhandler = handler;
      g_djoyarg     = arg;

      /* Attach and enable interrupts each GPIO. */

      for (i = 0; i < DJOY_NGPIOS; i++)
        {
          irq = g_buttonirq[i];
          if (irq > 0)
            {
              irq_attach(irq, djoy_interrupt, arg);
              up_enable_irq(irq);
            }
        }
    }

  leave_critical_section(flags);
#endif
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
#ifdef CONFIG_LPC17_40_GPIOIRQ
  irqstate_t flags;
  int irq;
  int i;

  /* Disable and detach all button handlers for each GPIO */

  flags = enter_critical_section();
  for (i = 0; i < DJOY_NGPIOS; i++)
    {
      irq = g_buttonirq[i];
      if (irq > 0)
        {
          up_disable_irq(irq);
          irq_detach(irq);
        }
    }

  leave_critical_section(flags);
#endif

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
  DEBUGASSERT(g_djoyhandler != NULL);
  if (g_djoyhandler != NULL)
    {
      g_djoyhandler(&g_djoylower, g_djoyarg);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_djoy_initialization
 *
 * Description:
 *   Initialize and register the discrete joystick driver
 *
 ****************************************************************************/

int lpc17_40_djoy_initialization(void)
{
  int i;

  /* Configure the GPIO pins as inputs.    NOTE: This is unnecessary for
   * interrupting pins since it will also be done by lpc17_40_gpiosetevent().
   */

  for (i = 0; i < DJOY_NGPIOS; i++)
    {
      lpc17_40_configgpio(g_joygpio[i]);
    }

  /* Make sure that all interrupts are disabled */

  djoy_disable();

  /* Register the joystick device as /dev/djoy0 */

  return djoy_register(CONFIG_OPEN1788_DJOYDEV, &g_djoylower);
}
