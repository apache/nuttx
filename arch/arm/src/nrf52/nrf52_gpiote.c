/****************************************************************************
 * arch/arm/src/nrf52/nrf52_gpiote.c
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
#ifdef CONFIG_NRF52_HAVE_PORT1
  int        port   = 0;
#endif
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
  regval = (pin << GPIOTE_CONFIG_PSEL_SHIFT);

#ifdef CONFIG_NRF52_HAVE_PORT1
  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  regval |= (port << GPIOTE_CONFIG_PORT_SHIFT);
#endif

  /* Select EVENT mode */

  if (event || func)
    {
      regval &= ~GPIOTE_CONFIG_MODE_MASK;
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

/****************************************************************************
 * Name: nrf52_gpiotaskset
 *
 * Description:
 *   Configure GPIO in TASK mode (to be controlled via tasks).
 *   Note that a pin can only be either in TASK or EVENT mode (set by
 *   nrf52_gpiosetevent with event set to true). Also, once set to TASK mode,
 *   pin control is only possible via tasks on the via nrf52_gpio_write and
 *   will automatically set the output mode.
 *   Finally, a given pin should only be assigned to a given channel.
 *
 * Input Parameters:
 *  - pinset: gpio pin configuration (only port + pin is important here)
 *  - channel: the GPIOTE channel used to control the given pin
 *  - output_high: set pin initially to output HIGH or LOW.
 *  - outcfg: configure pin behavior one OUT task is triggered
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int nrf52_gpiotaskset(uint32_t pinset, int channel,
                       bool output_high, enum nrf52_gpiote_outcfg_e outcfg)
{
  uint32_t regval;
  int pin;
#ifdef CONFIG_NRF52_HAVE_PORT1
  int port;
#endif

  /* Select GPIOTE pin */

  pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  regval = (pin << GPIOTE_CONFIG_PSEL_SHIFT);

#ifdef CONFIG_NRF52_HAVE_PORT1
  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  regval |= (port << GPIOTE_CONFIG_PORT_SHIFT);
#endif

  /* Select TASK mode */

  regval |= GPIOTE_CONFIG_MODE_TS;

  /* Select pin number */

  regval |= (pin << GPIOTE_CONFIG_PSEL_SHIFT);

  /* Select initial output */

  if (output_high)
    {
      regval |= (1 << GPIOTE_CONFIG_OUTINIT_SHIFT);
    }

  /* Set polarity mode */

  switch (outcfg)
    {
      case NRF52_GPIOTE_SET:
        regval |= GPIOTE_CONFIG_POL_LTH;
        break;
      case NRF52_GPIOTE_CLEAR:
        regval |= GPIOTE_CONFIG_POL_HTL;
        break;
      case NRF52_GPIOTE_TOGGLE:
        regval |= GPIOTE_CONFIG_POL_TG;
        break;
    }

  /* Write register */

  nrf52_gpiote_putreg(NRF52_GPIOTE_CONFIG_OFFSET(channel), regval);

  return OK;
}
