/****************************************************************************
 * arch/avr/src/at32uc3/at32uc3_gpioirq.c
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
#include "at32uc3_config.h"

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "up_internal.h"
#include "irq/irq.h"
#include "at32uc3.h"
#include "at32uc3_gpio.h"

#ifdef CONFIG_AVR32_GPIOIRQ

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct g_gpiohandler_s
{
  xcpt_t handler;   /* Interrupt handler entry point */
  void  *arg;       /* The argument that accompanies the interrupt handler */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A table of handlers for each GPIO interrupt */

static struct g_gpiohandler_s g_gpiohandler[NR_GPIO_IRQS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_baseaddress
 *
 * Input Parameters:
 *   irq - A IRQ number in the range of 0 to NR_GPIO_IRQS.
 *
 * Description:
 *   Given a IRQ number, return the base address of the associated GPIO
 *   registers.
 *
 ****************************************************************************/

static inline uint32_t gpio_baseaddress(unsigned int irq)
{
#if CONFIG_AVR32_GPIOIRQSETA != 0
  if (irq < __IRQ_GPIO_PB0)
    {
      return AVR32_GPIO0_BASE;
    }
  else
#endif
#if CONFIG_AVR32_GPIOIRQSETB != 0
  if (irq < NR_GPIO_IRQS)
    {
      return AVR32_GPIO1_BASE;
    }
  else
#endif
    {
      return 0;
    }
}

/****************************************************************************
 * Name: gpio_pin
 *
 * Input Parameters:
 *   irq - A IRQ number in the range of 0 to NR_GPIO_IRQS.
 *
 * Description:
 *   Given a GPIO number, return the pin number in the range of 0-31 on the
 *   corresponding port
 *
 ****************************************************************************/

static inline int gpio_pin(unsigned int irq)
{
  uint32_t pinset;
  int pinirq;
  int pin;

#if CONFIG_AVR32_GPIOIRQSETA != 0
  if (irq < __IRQ_GPIO_PB0)
    {
      pinset = CONFIG_AVR32_GPIOIRQSETA;
      pinirq = __IRQ_GPIO_PA0;
    }
  else
#endif
#if CONFIG_AVR32_GPIOIRQSETB != 0
  if (irq < NR_GPIO_IRQS)
    {
      pinset = CONFIG_AVR32_GPIOIRQSETB;
      pinirq = __IRQ_GPIO_PB0;
    }
  else
#endif
    {
      return -EINVAL;
    }

  /* Now we have to search for the pin with matching IRQ.  Yech! We made
   * life difficult here by choosing a sparse representation of IRQs on
   * GPIO pins.
   */

  for (pin = 0; pin < 32 && pinset != 0; pin++)
    {
      /* Is this pin at bit 0 configured for interrupt support? */

      if ((pinset & 1) != 0)
        {
          /* Is it the on IRQ we are looking for? */

          if (pinirq == irq)
            {
              /* Yes, return the associated pin number */

              return pin;
            }

          /* No.. Increment the IRQ number for the next configured pin */

           pinirq++;
        }

      /* Shift the next pin to position bit 0 */

      pinset >>= 1;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: gpio_porthandler
 *
 * Description:
 *   Dispatch GPIO interrupts on a specific GPIO port
 *
 ****************************************************************************/

static void gpio_porthandler(uint32_t regbase, int irqbase,
                             uint32_t irqset, void *context)
{
  uint32_t ifr;
  int irq;
  int pin;

  /* Check each bit and dispatch each pending interrupt in the interrupt flag
   * register for this port.
   */

  ifr = getreg32(regbase + AVR32_GPIO_IFR_OFFSET);

  /* Dispatch each pending interrupt */

  irq = irqbase;
  for (pin = 0; pin < 32 && ifr != 0; pin++)
    {
      /* Is this pin configured for interrupt support? */

      uint32_t bit = (1 << pin);
      if ((irqset & bit) != 0)
        {
          /* Is an interrupt pending on this pin? */

          if ((ifr & bit) != 0)
            {
              /* Yes.. Clear the pending interrupt */

              putreg32(bit, regbase + AVR32_GPIO_IFRC_OFFSET);
              ifr &= ~bit;

              /* Dispatch handling for this pin */

              xcpt_t handler = g_gpiohandler[irq].handler;
              if (handler != NULL)
                {
                  handler(irq, context, g_gpiohandler[irq].arg);
                }
              else
                {
                  _err("ERROR: No handler: pin=%d ifr=%08x irqset=%08x",
                       pin, ifr, irqset);
                }
            }

          /* Increment the IRQ number on all configured pins */

          irq++;
        }

      /* Not configured.  An interrupt on this pin would be an error. */

      else if ((ifr & bit) != 0)
        {
          /* Clear the pending interrupt */

          putreg32(bit, regbase + AVR32_GPIO_IFRC_OFFSET);
          ifr &= ~bit;

          warn("WARNING:"
               " IRQ on unconfigured pin: pin=%d ifr=%08x irqset=%08x",
               pin, ifr, irqset);
        }
    }
}

/****************************************************************************
 * Name: gpio0/1_interrupt
 *
 * Description:
 *   Handle GPIO0/1 interrupts
 *
 ****************************************************************************/

#if CONFIG_AVR32_GPIOIRQSETA != 0
static int gpio0_interrupt(int irq, FAR void *context, FAR void *arg)
{
  gpio_porthandler(AVR32_GPIO0_BASE, __IRQ_GPIO_PA0,
                   CONFIG_AVR32_GPIOIRQSETA, context);
  return 0;
}
#endif

#if CONFIG_AVR32_GPIOIRQSETB != 0
static int gpio1_interrupt(int irq, FAR void *context, FAR void *arg)
{
  gpio_porthandler(AVR32_GPIO1_BASE, __IRQ_GPIO_PB0,
                   CONFIG_AVR32_GPIOIRQSETB, context);

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_irqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler.
 *
 * Assumptions:
 *   Called during the early boot sequence before global interrupts have
 *   been enabled.
 *
 ****************************************************************************/

void gpio_irqinitialize(void)
{
  int i;

  /* Point all interrupt vectors to the unexpected interrupt */

  for (i = 0; i < NR_GPIO_IRQS; i++)
    {
      g_gpiohandler[i].handler = irq_unexpected_isr;
      g_gpiohandler[i].arg     = NULL;
    }

  /* Then attach the GPIO interrupt handlers */

#if CONFIG_AVR32_GPIOIRQSETA != 0
  irq_attach(AVR32_IRQ_GPIO0, gpio0_interrupt, NULL);
#endif
#if CONFIG_AVR32_GPIOIRQSETB != 0
  irq_attach(AVR32_IRQ_GPIO1, gpio1_interrupt, NULL);
#endif
}

/****************************************************************************
 * Name: gpio_irqattach
 *
 * Description:
 *   Attach in GPIO interrupt to the provide 'isr'
 *
 ****************************************************************************/

int gpio_irqattach(int irq, xcpt_t handler, void *arg)
{
  irqstate_t flags;
  int        ret = -EINVAL;

  if ((unsigned)irq < NR_GPIO_IRQS)
    {
      /* If the new ISR is NULL, then the ISR is being detached. In this
       * case, disable the ISR and direct any interrupts
       * to the unexpected interrupt handler.
       */

      flags = enter_critical_section();
      if (handler == NULL)
        {
           gpio_irqdisable(irq);

           handler = irq_unexpected_isr;
           arg     = NULL;
        }

      /* Save the new ISR in the table. */

      g_gpiohandler[irq].handler = handler;
      g_gpiohandler[irq].arg     = arg;

      leave_critical_section(flags);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: gpio_irqenable
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'irq'
 *
 ****************************************************************************/

void gpio_irqenable(int irq)
{
  uint32_t base;
  int      pin;

  if ((unsigned)irq < NR_GPIO_IRQS)
    {
      /* Get the base address of the GPIO module associated with this IRQ */

      base = gpio_baseaddress(irq);

      /* Get the pin number associated with this IRQ.  We made life difficult
       * here by choosing a sparse representation of IRQs on GPIO pins.
       */

      pin  = gpio_pin(irq);
      DEBUGASSERT(pin >= 0);

      /* Enable the GPIO interrupt. */

      putreg32((1 << pin), base + AVR32_GPIO_IERS_OFFSET);
    }
}

/****************************************************************************
 * Name: gpio_irqdisable
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'irq'
 *
 ****************************************************************************/

void gpio_irqdisable(int irq)
{
  uint32_t base;
  int      pin;

  if ((unsigned)irq < NR_GPIO_IRQS)
    {
      /* Get the base address of the GPIO module associated with this IRQ */

      base = gpio_baseaddress(irq);

      /* Get the pin number associated with this IRQ.  We made life difficult
       * here by choosing a sparse representation of IRQs on GPIO pins.
       */

      pin  = gpio_pin(irq);
      DEBUGASSERT(pin >= 0);

      /* Disable the GPIO interrupt. */

      putreg32((1 << pin), base + AVR32_GPIO_IERC_OFFSET);
    }
}

#endif /* CONFIG_AVR32_GPIOIRQ */
