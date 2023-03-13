/****************************************************************************
 * arch/arm/src/nrf53/nrf53_gpiote.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <arch/irq.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "nrf53_gpio.h"
#include "nrf53_gpiote.h"

#include "hardware/nrf53_gpiote.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIOTE_PER_CHANNEL (8)

#ifdef CONFIG_NRF53_HAVE_GPIOTE1
#  define GPIOTE_CHANNELS  (16)
#else
#  define GPIOTE_CHANNELS  (8)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf53_gpiote_callback_s
{
  xcpt_t callback;
  void  *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Callbacks attached to each GPIOTE channel */

static struct nrf53_gpiote_callback_s g_gpiote_ch_callbacks[GPIOTE_CHANNELS];

#ifdef CONFIG_NRF53_PER_PIN_INTERRUPTS
/* Callbacks attached to each GPIO pin */

static struct nrf53_gpiote_callback_s
    g_gpiote_pin_callbacks[NRF53_GPIO_NPORTS][NRF53_GPIO_NPINS];
#else
/* Callback for the PORT event */

static struct nrf53_gpiote_callback_s
    g_gpiote_port_callback[NRF53_GPIO_NPORTS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_gpiote_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset for a given GPIOTE instance
 *
 ****************************************************************************/

static inline void nrf53_gpiote_putreg(int inst, uint32_t offset,
                                       uint32_t value)
{
#ifdef CONFIG_NRF53_HAVE_GPIOTE1
  if (inst == 1)
    {
      putreg32(value, NRF53_GPIOTE1_BASE + offset);
    }
  else
#endif
    {
      putreg32(value, NRF53_GPIOTE0_BASE + offset);
    }
}

/****************************************************************************
 * Name: nrf53_gpiote_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset for a given GPIOTE instance
 *
 ****************************************************************************/

static inline uint32_t nrf53_gpiote_getreg(int inst, uint32_t offset)
{
#ifdef CONFIG_NRF53_HAVE_GPIOTE1
  if (inst == 1)
    {
      return getreg32(NRF53_GPIOTE1_BASE + offset);
    }
  else
#endif
    {
      return getreg32(NRF53_GPIOTE0_BASE + offset);
    }
}

/****************************************************************************
 * Name: nrf53_gpiote_isr
 *
 * Description:
 *   Common GPIOTE interrupt handler (GPIOTE0 and GPIOTE1)
 *
 ****************************************************************************/

static int nrf53_gpiote_isr(int irq, void *context, void *arg)
{
  uint32_t regval = 0;
  int      ret    = OK;
  int      i      = 0;
  int      inst   = 0;
  int      off    = 0;
#ifdef CONFIG_NRF53_PER_PIN_INTERRUPTS
  int      j      = 0;
#endif

  /* Get GPIOTE instnace */

#ifdef CONFIG_NRF53_HAVE_GPIOTE1
  ints = (irq == NRF53_IRQ_GPIOTE0) ? 0 : 1;
#else
  inst = 0;
#endif

  /* Scan all GPIOTE channels */

  for (i = 0; i < GPIOTE_CHANNELS; i += 1)
    {
      off = i + GPIOTE_PER_CHANNEL * inst;

      /* Only if callback is registered */

      if (g_gpiote_ch_callbacks[off].callback != NULL)
        {
          /* Get input event register */

          regval = nrf53_gpiote_getreg(inst,
                                       NRF53_GPIOTE_EVENTS_IN_OFFSET(i));
          if (regval == GPIOTE_EVENT_IN_EVENT)
            {
              /* Execute callback */

              xcpt_t callback = g_gpiote_ch_callbacks[off].callback;
              void *cbarg = g_gpiote_ch_callbacks[off].arg;
              ret = callback(irq, context, cbarg);

              /* Clear event */

              nrf53_gpiote_putreg(inst, NRF53_GPIOTE_EVENTS_IN_OFFSET(i), 0);
            }
        }
    }

  /* Check for PORT event */

  regval = nrf53_gpiote_getreg(inst, NRF53_GPIOTE_EVENTS_PORT_OFFSET);
  if (regval)
    {
      uint32_t addr = 0;

      /* Ack PORT event */

      nrf53_gpiote_putreg(inst, NRF53_GPIOTE_EVENTS_PORT_OFFSET, 0);

      /* For each GPIO port, get LATCH register */

      for (i = 0; i < NRF53_GPIO_NPORTS; i++)
        {
          switch (i)
            {
              case 0:
                addr = NRF53_GPIO_P0_BASE + NRF53_GPIO_LATCH_OFFSET;
              break;
              case 1:
                addr = NRF53_GPIO_P1_BASE + NRF53_GPIO_LATCH_OFFSET;
              break;
            }

          /* Retrieve LATCH register */

          regval = getreg32(addr);

          /* Clear LATCH register (this may set PORT again) */

          putreg32(0xffffffff, addr);

#ifdef CONFIG_NRF53_PER_PIN_INTERRUPTS
          /* Check for pins with DETECT bit high in LATCH register
           * and dispatch callback if set
           */

          for (j = 0; j < NRF53_GPIO_NPINS && regval; j++)
            {
              if (regval & (1 << j) && g_gpiote_pin_callbacks[i][j].callback)
                {
                  /* Run callback */

                  xcpt_t callback = g_gpiote_pin_callbacks[i][j].callback;
                  void *cbarg = g_gpiote_pin_callbacks[i][j].arg;

                  ret = callback(irq, context, cbarg);

                  /* Mark bit is as "visited", we can stop looping sooner
                   * this way
                   */

                  regval &= ~(1 << j);
                }
            }
#else
          if (g_gpiote_port_callback[i].callback)
            {
              xcpt_t callback = g_gpiote_port_callback[i].callback;
              void *cbarg = g_gpiote_port_callback[i].arg;

              ret = callback(irq, context, cbarg);
            }
#endif
       }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_NRF53_PER_PIN_INTERRUPTS
/****************************************************************************
 * Name: nrf53_gpiote_set_pin_event
 *
 * Description:
 *   Sets/clears a handler for a given pin for the GPIO PORT event. This
 *   will mean edge-sensitive or level-sensitive according to GPIO detect
 *   mode configuration for the port (see nrf53_gpio_detectmode()). Pin
 *   will be sensitive to high/low according to GPIO_SENSE_LOW/HIGH
 *   (set via nrf53_gpio_config()).
 *
 *   The passed handler will be invoked from the main ISR for the PORT
 *   event and will take care of clearing the LATCH register.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 ****************************************************************************/

void nrf53_gpiote_set_pin_event(uint32_t pinset, xcpt_t func, void *arg)
{
  int        pin    = 0;
  int        port   = 0;
  irqstate_t flags;

  pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

  flags = enter_critical_section();

  g_gpiote_pin_callbacks[port][pin].callback = func;
  g_gpiote_pin_callbacks[port][pin].arg = arg;

  leave_critical_section(flags);
}
#else
/****************************************************************************
 * Name: nrf53_gpiote_set_port_event
 *
 * Description:
 *   Sets/clears the handler for the GPIO PORT event.
 *
 *   The passed handler will be invoked from the main ISR for the PORT
 *   event and will take care of clearing the LATCH register.
 *
 *   Port events available only for GPIOTE0 for now.
 *
 * Input Parameters:
 *  - pinset:      GPIO port will be extracted from this parameter
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 ****************************************************************************/

void nrf53_gpiote_set_port_event(uint32_t pinset, xcpt_t func, void *arg)
{
  int        port   = 0;
  int        inst   = 0;
  irqstate_t flags;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

  flags = enter_critical_section();

  g_gpiote_port_callback[port].callback = func;
  g_gpiote_port_callback[port].arg      = arg;

  if (func)
    {
      /* Enable the ISR */

      nrf53_gpiote_putreg(inst, NRF53_GPIOTE_INTENSET_OFFSET,
                          GPIOTE_INT_PORT);
    }
  else
    {
      /* Check if we can disable the ISR */

      int i;

      for (i = 0; i < NRF53_GPIO_NPORTS; i++)
        {
          if (g_gpiote_port_callback[port].callback)
            {
              break;
            }
        }

      if (i == NRF53_GPIO_NPORTS)
        {
          nrf53_gpiote_putreg(inst, NRF53_GPIOTE_INTENCLR_OFFSET,
                              GPIOTE_INT_PORT);
        }
    }

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: nrf53_gpiote_set_ch_event
 *
 * Description:
 *   Configures a GPIOTE channel in EVENT mode, assigns it to a given pin
 *   and sets a handler for the corresponding channel events.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - channel:     GPIOTE channel used to capture events
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 ****************************************************************************/

void nrf53_gpiote_set_ch_event(uint32_t pinset, int channel,
                               bool risingedge, bool fallingedge,
                               xcpt_t func, void *arg)
{
  int        pin    = 0;
  int        port   = 0;
  int        inst   = 0;
  uint32_t   regval = 0;
  uint32_t   rchan  = 0;
  irqstate_t flags;

  DEBUGASSERT(channel < GPIOTE_CHANNELS);

  /* Get GPIOTE instnace */

#ifdef CONFIG_NRF53_HAVE_GPIOTE1
  inst = (channel < GPIOTE_PER_CHANNEL) ? 0 : 1;
#else
  inst = 0;
#endif

  rchan = (inst == 1) ? channel : (channel - GPIOTE_PER_CHANNEL);

  /* NOTE: GPIOTE module has priority over GPIO module
   *       so GPIO configuration will be ignored
   */

  flags = enter_critical_section();

  if (func)
    {
      /* Select EVENT mode */

      regval |= GPIOTE_CONFIG_MODE_EV;

      /* Select GPIOTE pin */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      regval |= (pin << GPIOTE_CONFIG_PSEL_SHIFT);

      port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
      regval |= (port << GPIOTE_CONFIG_PORT_SHIFT);

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

      /* Enable callback for channel */

      g_gpiote_ch_callbacks[channel].callback = func;
      g_gpiote_ch_callbacks[channel].arg      = arg;

      /* Enable interrupt for given event */

      nrf53_gpiote_putreg(inst, NRF53_GPIOTE_INTENSET_OFFSET,
                          GPIOTE_INT_IN(rchan));
    }
  else
    {
      /* Leave register as zero (disabled mode) */

      /* Disable interrupt for given event */

      nrf53_gpiote_putreg(inst, NRF53_GPIOTE_INTENCLR_OFFSET,
                          GPIOTE_INT_IN(rchan));

      /* Remove callback configuration */

      g_gpiote_ch_callbacks[channel].callback = NULL;
      g_gpiote_ch_callbacks[channel].arg      = NULL;
    }

  /* Write CONFIG register */

  nrf53_gpiote_putreg(inst, NRF53_GPIOTE_CONFIG_OFFSET(rchan), regval);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf53_gpiote_set_event
 *
 * Description:
 *   Configures a GPIOTE channel in EVENT mode, assigns it to a given pin
 *   and sets a handler for the first availalbe GPIOTE channel
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int nrf53_gpiote_set_event(uint32_t pinset,
                           bool risingedge, bool fallingedge,
                           xcpt_t func, void *arg)
{
  irqstate_t flags;
  int ret = -ENOMEM;
  int i = 0;

  flags = enter_critical_section();

  /* Get free channel */

  for (i = 0; i < GPIOTE_CHANNELS; i++)
    {
      if (g_gpiote_ch_callbacks[i].callback == NULL)
        {
          /* Configure channel */

          nrf53_gpiote_set_ch_event(pinset, i,
                                    risingedge, fallingedge,
                                    func, arg);

          /* Return the channel index */

          ret = i;

          break;
        }
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: nrf53_gpio_set_task
 *
 * Description:
 *   Configure GPIO in TASK mode (to be controlled via tasks).
 *   Note that a pin can only be either in TASK or EVENT mode (set by
 *   nrf53_gpiosetevent with event set to true). Also, once set to TASK mode,
 *   pin control is only possible via tasks on the via nrf53_gpio_write and
 *   will automatically set the output mode.
 *   Finally, a given pin should only be assigned to a given channel.
 *
 * Input Parameters:
 *  - pinset:      gpio pin configuration (only port + pin is important here)
 *  - channel:     the GPIOTE channel used to control the given pin
 *  - output_high: set pin initially to output HIGH or LOW.
 *  - outcfg:      configure pin behavior one OUT task is triggered
 *
 ****************************************************************************/

void nrf53_gpiote_set_task(uint32_t pinset, int channel,
                           bool output_high,
                           enum nrf53_gpiote_outcfg_e outcfg)
{
  uint32_t regval;
  uint32_t rchan;
  int pin;
  int port;
  int inst;

  /* Get GPIOTE instnace */

#ifdef CONFIG_NRF53_HAVE_GPIOTE1
  inst = (channel < GPIOTE_PER_CHANNEL) ? 0 : 1;
#else
  inst = 0;
#endif

  rchan = (inst == 1) ? channel : (channel - GPIOTE_PER_CHANNEL);

  /* Select GPIOTE pin */

  pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  regval = (pin << GPIOTE_CONFIG_PSEL_SHIFT);

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  regval |= (port << GPIOTE_CONFIG_PORT_SHIFT);

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
      case NRF53_GPIOTE_SET:
        regval |= GPIOTE_CONFIG_POL_LTH;
        break;
      case NRF53_GPIOTE_CLEAR:
        regval |= GPIOTE_CONFIG_POL_HTL;
        break;
      case NRF53_GPIOTE_TOGGLE:
        regval |= GPIOTE_CONFIG_POL_TG;
        break;
    }

  /* Write register */

  nrf53_gpiote_putreg(inst, NRF53_GPIOTE_CONFIG_OFFSET(rchan), regval);
}

/****************************************************************************
 * Name: nrf53_gpiote_init
 *
 * Description:
 *   Initialize GPIOTE
 *
 ****************************************************************************/

int nrf53_gpiote_init(void)
{
  /* Clear LATCH register(s) */

  putreg32(0, NRF53_GPIO_P0_BASE + NRF53_GPIO_LATCH_OFFSET);
  putreg32(0, NRF53_GPIO_P1_BASE + NRF53_GPIO_LATCH_OFFSET);

  /* Reset GPIOTE data */

  memset(&g_gpiote_ch_callbacks, 0, sizeof(g_gpiote_ch_callbacks));

#ifdef CONFIG_NRF53_PER_PIN_INTERRUPTS
  memset(&g_gpiote_pin_callbacks, 0, sizeof(g_gpiote_pin_callbacks));

  /* Enable PORT event interrupt */

  nrf53_gpiote_putreg(0, NRF53_GPIOTE_INTENSET_OFFSET, GPIOTE_INT_PORT);
#  ifdef CONFIG_NRF53_HAVE_GPIOTE1
  nrf53_gpiote_putreg(1, NRF53_GPIOTE_INTENSET_OFFSET, GPIOTE_INT_PORT);
#  endif
#else
  memset(&g_gpiote_port_callback, 0, sizeof(g_gpiote_port_callback));
#endif

  /* Attach GPIOTE interrupt handler */

  irq_attach(NRF53_IRQ_GPIOTE0, nrf53_gpiote_isr, NULL);
  up_enable_irq(NRF53_IRQ_GPIOTE0);
#ifdef CONFIG_NRF53_HAVE_GPIOTE1
  irq_attach(NRF53_IRQ_GPIOTE1, nrf53_gpiote_isr, NULL);
  up_enable_irq(NRF53_IRQ_GPIOTE1);
#endif

  return OK;
}
