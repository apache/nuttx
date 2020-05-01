/****************************************************************************
 * arch/arm/src/nrf52/nrf52_gpiote.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <arch/irq.h>
#include <nuttx/arch.h>

#include "arm_arch.h"

#include "nrf52_gpio.h"
#include "nrf52_gpiote.h"

#include "hardware/nrf52_gpiote.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIOTE_CHANNELS 8

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_gpiote_callback_s
{
  xcpt_t     callback;
  FAR void  *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to each GPIOTE */

static struct nrf52_gpiote_callback_s g_gpiote_callbacks[GPIOTE_CHANNELS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_gpiote_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void nrf52_gpiote_putreg(uint32_t offset, uint32_t value)
{
  putreg32(value, NRF52_GPIOTE_BASE + offset);
}

/****************************************************************************
 * Name: nrf52_gpiote_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t nrf52_gpiote_getreg(uint32_t offset)
{
  return getreg32(NRF52_GPIOTE_BASE + offset);
}

/****************************************************************************
 * Name: nrf52_gpiote_isr
 *
 * Description:
 *   Common GPIOTE interrupt handler
 *
 ****************************************************************************/

static int nrf52_gpiote_isr(int irq, FAR void *context, FAR void *arg)
{
  uint32_t regval = 0;
  int      ret    = OK;
  int      i      = 0;

  /* Scan all GPIOTE channels */

  for (i = 0; i < GPIOTE_CHANNELS; i += 1)
    {
      /* Only if callback is registered */

      if (g_gpiote_callbacks[i].callback != NULL)
        {
          /* Get input event register */

          regval = nrf52_gpiote_getreg(NRF52_GPIOTE_EVENTS_IN_OFFSET(i));
          if (regval == GPIOTE_EVENT_IN_EVENT)
            {
              /* Execute callback */

              xcpt_t callback = g_gpiote_callbacks[i].callback;
              FAR void *cbarg = g_gpiote_callbacks[i].arg;
              ret = callback(irq, context, cbarg);

              /* Clear event */

              nrf52_gpiote_putreg(NRF52_GPIOTE_EVENTS_IN_OFFSET(i), 0);
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - event:       Generate event when set
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int nrf52_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, FAR void *arg)
{
  int        ret    = OK;
  int        i      = 0;
  int        pin    = 0;
  int        port   = 0;
  uint32_t   regval = 0;
  bool       found  = false;
  irqstate_t flags;

  /* Find available GPIOTE channel */

  flags = enter_critical_section();

  for (i = 0; i < GPIOTE_CHANNELS; i += 1)
    {
      if (g_gpiote_callbacks[i].callback == NULL)
        {
          found = true;
          break;
        }
    }

  leave_critical_section(flags);

  /* Return error if there is no free channel */

  if (found == false)
    {
      ret = -ENODEV;
      goto errout;
    }

  /* NOTE: GPIOTE module has priority over GPIO module
   *       so GPIO configuration will be ignored
   */

  /* Select GPIOTE pin */

  pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

  regval = (pin << GPIOTE_CONFIG_PSEL_SHIFT);
  regval |= (port << GPIOTE_CONFIG_PORT_SHIFT);

  /* Select EVENT mode */

  if (event || func)
    {
      regval |= GPIOTE_CONFIG_MODE_EV;
    }

  /* Select polarity */

  if (risingedge == true && fallingedge == true)
    {
      regval |= GPIOTE_CONFIG_POL_TG;
    }
  else if (risingedge == true)
    {
      regval |= GPIOTE_CONFIG_POL_LTH;
    }
  else if (fallingedge == true)
    {
      regval |= GPIOTE_CONFIG_POL_HTL;
    }

  /* Write CONFIG register */

  nrf52_gpiote_putreg(NRF52_GPIOTE_CONFIG_OFFSET(i), regval);

  /* Enable interrupt for given event */

  nrf52_gpiote_putreg(NRF52_GPIOTE_INTENSET_OFFSET, GPIOTE_INT_IN(i));

  /* Connect callback */

  g_gpiote_callbacks[i].callback = func;
  g_gpiote_callbacks[i].arg      = arg;

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf52_gpiote_init
 *
 * Description:
 *   Initialize GPIOTE
 *
 ****************************************************************************/

int nrf52_gpiote_init(void)
{
  /* Reset GPIOTE data */

  memset(&g_gpiote_callbacks,
         0,
         sizeof(struct nrf52_gpiote_callback_s)*GPIOTE_CHANNELS);

  /* Attach GPIOTE interrupt handler */

  irq_attach(NRF52_IRQ_GPIOTE, nrf52_gpiote_isr, NULL);
  up_enable_irq(NRF52_IRQ_GPIOTE);

  return OK;
}
