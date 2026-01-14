/****************************************************************************
 * arch/avr/src/avrdx/avrdx_gpio_isr_mux.c
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

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <assert.h>
#include <syslog.h>
#include <avr/io.h>

#include "avrdx.h"
#include "avrdx_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_MUX_MAX_CHANNELS 4

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpio_pin_isr_s
{
  xcpt_t handler; /* handler given by the user */
  FAR void *arg;  /* arg to be given to the handler */
  uint8_t pins;   /* pins registered for this handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct gpio_pin_isr_s **g_gpio_pin_isrs;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_gpio_isr_mux
 *
 * Description:
 *   Interrupt handler for multiplexed I/O ports. Executes registered
 *   handlers if the pin(s) that raised the interrupt match pins
 *   the handler wanted for this port.
 *
 * Input Parameters:
 *   ISR parameters
 *
 * Assumptions:
 *   Always running in interrupt context.
 *
 ****************************************************************************/

static int avrdx_gpio_isr_mux(int irq, void *context, FAR void *arg)
{
  struct gpio_pin_isr_s *port_isrs;
  uint8_t port_index;
  uint8_t flags;
  uint8_t irq8;
  uint8_t i;

  /* AVR has less than 256 interrupts, save flash and clock ticks */

  irq8 = irq;

  /* Look the port up based on IRQ number */

  if (irq8 == AVRDX_IRQ_PORTA_PORT)
    {
      port_index = AVRDX_GPIO_PORTA_IDX;
    }
  else if (irq8 == AVRDX_IRQ_PORTC_PORT)
    {
      port_index = AVRDX_GPIO_PORTC_IDX;
    }
  else if (irq8 == AVRDX_IRQ_PORTD_PORT)
    {
      port_index = AVRDX_GPIO_PORTD_IDX;
    }
  else if (irq8 == AVRDX_IRQ_PORTF_PORT)
    {
      port_index = AVRDX_GPIO_PORTF_IDX;
    }

#ifdef CONFIG_AVR_HAS_PORTB
  else if (irq8 == AVRDX_IRQ_PORTB_PORT)
    {
      port_index = AVRDX_GPIO_PORTB_IDX;
    }
#endif

#ifdef CONFIG_AVR_HAS_PORTE
  else if (irq8 == AVRDX_IRQ_PORTE_PORT)
    {
      port_index = AVRDX_GPIO_PORTE_IDX;
    }
#endif

#ifdef CONFIG_AVR_HAS_PORTG
  else if (irq8 == AVRDX_IRQ_PORTG_PORT)
    {
      port_index = AVRDX_GPIO_PORTG_IDX;
    }
#endif

  else
    {
      /* Don't try to convert these to __memx (IPTR), syslog_add_intbuffer
       * (write to syslog interrupt buffer) does not support that
       */

      syslog(LOG_EMERG,
             "avrdx_gpio_isr_mux unknown IRQ %i\n", irq);
      PANIC(); /* How did we get here? */
    }

  port_isrs = g_gpio_pin_isrs[port_index];

  if (!port_isrs)
    {
      syslog(LOG_EMERG,
             "avrdx_gpio_isr_mux port handler not allocated\n");
      PANIC(); /* IRQ is attached when this is allocated */
    }

  /* Read pins which triggred the interrupt and clear their flags */

  flags = AVRDX_PORT(port_index).INTFLAGS;
  AVRDX_PORT(port_index).INTFLAGS = flags;

  for (i = 0; i < GPIO_MUX_MAX_CHANNELS; i++, port_isrs++)
    {
      if ((port_isrs) && (port_isrs->pins & flags))
        {
          port_isrs->handler(irq, context, port_isrs->arg);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_irq_attach_gpio_mux
 *
 * Description:
 *   Attaches given handler and arg to the GPIO IRQ multiplexer.
 *
 * Input Parameters:
 *   Requested port. Can use a number directly or preprocessor
 *     constant (one of AVRDX_GPIO_PORTA_IDX etc.)
 *   Pins handled by this handler. Given as bitmask.
 *   Handler itself
 *   Opaque arg argument
 *   ISC - input sense configuration (ie. what pin change triggers
 *     the interrupt. See eg. AVR128DA28 datasheet, chapter PORT
 *     I/O pin configuration, section Register Description, subsection
 *     Pin n Control (17.5.16 in rev C of the document)
 *
 * Returned Value:
 *   OK or EBUSY if another handler already uses the pin. EINVAL for port
 *   that does not exist.
 *
 ****************************************************************************/

int avrdx_irq_attach_gpio_mux(uint8_t port_idx, uint8_t pins,
                              xcpt_t handler, void *arg,
                              uint8_t isc)
{
  struct gpio_pin_isr_s *port_isrs;
  irqstate_t irqflags;
  uint8_t first_free_slot;
  uint8_t vector_num;
  int ret;
  uint8_t i;

  first_free_slot = GPIO_MUX_MAX_CHANNELS; /* No free slot was found yet */

#ifdef AVR_HAS_PORTG
  if (port_idx > AVRDX_GPIO_PORTG_IDX)
    {
      return -EINVAL;
    }
#else
  if (port_idx > AVRDX_GPIO_PORTF_IDX)
    {
      return -EINVAL;
    }
#endif

  vector_num = avrdx_gpio_irq_vectors[port_idx];
  if (!vector_num)
    {
      return -EINVAL;
    }

  irqflags = enter_critical_section();

  port_isrs = g_gpio_pin_isrs[port_idx];
  if (!port_isrs)
    {
      /* This port is not in use yet, also not attached. */

      g_gpio_pin_isrs[port_idx] = \
          (struct gpio_pin_isr_s *) kmm_zalloc \
          (sizeof(struct gpio_pin_isr_s) * GPIO_MUX_MAX_CHANNELS);
      port_isrs = g_gpio_pin_isrs[port_idx];
      if (!port_isrs)
        {
          ret = -ENOMEM;
          goto errout_lcs;
        }

      irq_attach(vector_num, avrdx_gpio_isr_mux, NULL);
    }

  /* Search existing handlers to find out if the new one
   * is actually new. Parameter arg must match too to reuse
   * existing handler.
   */

  for (i = 0; i < GPIO_MUX_MAX_CHANNELS; i++, port_isrs++)
    {
      if (!(port_isrs->handler))
        {
          if (first_free_slot == GPIO_MUX_MAX_CHANNELS)
            {
              first_free_slot = i; /* Record it for possible use later */
            }

          continue;
        }

      if ((port_isrs->handler == handler) && (port_isrs->arg == arg))
        {
          port_isrs->pins |= pins; /* Add pins, handler exists. */
          break;
        }
    }

  if (i == GPIO_MUX_MAX_CHANNELS)
    {
      /* Reached end of the array and did not find the handler
       * being in use already. Add new record - if there is space
       */

      if (first_free_slot == GPIO_MUX_MAX_CHANNELS)
        {
          ret = -ENOMEM; /* no space */
          goto errout_lcs;
        }

      port_isrs = g_gpio_pin_isrs[port_idx]; /* reset to start */
      port_isrs[first_free_slot].handler = handler;
      port_isrs[first_free_slot].arg = arg;
      port_isrs[first_free_slot].pins = pins;
    }

  /* One way or the other, if we got here, pins are now serviced
   * by an interrupt handler. Except the hardware is not configured
   * to to that yet. Do it now, clear existing interrupt flags first
   */

  AVRDX_PORT(port_idx).INTFLAGS = pins;
  AVRDX_PORT(port_idx).PINCONFIG = PORT_ISC_GM;
  AVRDX_PORT(port_idx).PINCTRLCLR = pins;
  AVRDX_PORT(port_idx).PINCONFIG = isc;
  AVRDX_PORT(port_idx).PINCTRLSET = pins;

  leave_critical_section(irqflags);

  return OK;

errout_lcs:
  leave_critical_section(irqflags);

  return ret;
}

/****************************************************************************
 * Name: avrdx_irq_detach_gpio_mux
 *
 * Description:
 *   Detaches handler from GPIO IRQ multiplexer.
 *
 * Input Parameters:
 *   Requested port
 *   Pins to be detached. Removes handler completely if it no longer
 *     services any pin. (Except if keep_handler flag is set)
 *   Keep handler flag. See above.
 *
 * Returned Value:
 *   OK. EINVAL when detaching port that was not in use or other
 *   incorrect input.
 *
 ****************************************************************************/

int avrdx_irq_detach_gpio_mux(uint8_t port_idx, uint8_t pins,
                              bool keep_handler)
{
  struct gpio_pin_isr_s *port_isrs;
  irqstate_t irqflags;
  int ret;
  uint8_t i;

#ifdef AVR_HAS_PORTG
  if (port_idx > AVRDX_GPIO_PORTG_IDX)
    {
      return -EINVAL;
    }
#else
  if (port_idx > AVRDX_GPIO_PORTF_IDX)
    {
      return -EINVAL;
    }
#endif

  irqflags = enter_critical_section();

  port_isrs = g_gpio_pin_isrs[port_idx];
  if (!port_isrs)
    {
      ret = -EINVAL;
      goto errout_lcs;
    }

  for (i = 0; i < GPIO_MUX_MAX_CHANNELS; i++, port_isrs++)
    {
      if (port_isrs->pins & pins)
        {
          port_isrs->pins &= ~pins;
          if (!(port_isrs->pins) && (!keep_handler))
            {
              port_isrs->handler = 0;
              port_isrs->arg = 0;
            }
        }

      /* Continue the loop, pins may touch more than one record */
    }

  /* Note that pins may contain bits that do not match any handler.
   * That's currently silently ignored
   */

  /* Disable interrupts for detached pins and clear their
   * interrupt flags
   */

  AVRDX_PORT(port_idx).INTFLAGS = pins;
  AVRDX_PORT(port_idx).PINCONFIG = PORT_ISC_GM;
  AVRDX_PORT(port_idx).PINCTRLCLR = pins;

  leave_critical_section(irqflags);

  return OK;

errout_lcs:
  leave_critical_section(irqflags);

  return ret;
}

/****************************************************************************
 * Name: avrdx_gpio_isr_mux_init
 *
 * Description:
 *   Initialize global variable holding ISR data
 *
 ****************************************************************************/

void avrdx_gpio_isr_mux_init(void)
{
#ifdef AVR_HAS_PORTG
  g_gpio_pin_isrs = (struct gpio_pin_isr_s **) kmm_zalloc \
      (sizeof(struct gpio_pin_isr_s *) * (AVRDX_GPIO_PORTG_IDX + 1));
#else
  g_gpio_pin_isrs = (struct gpio_pin_isr_s **) kmm_zalloc \
      (sizeof(struct gpio_pin_isr_s *) * (AVRDX_GPIO_PORTF_IDX + 1));
#endif
}
