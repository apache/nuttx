/****************************************************************************
 * arch/arm/src/mcx-nxxx/nxxx_gpioirq.c
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "nxxx_gpio.h"
#include "arm_internal.h"

#ifdef CONFIG_NXXX_GPIO_IRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxxx_portisr_s
{
  struct
  {
    xcpt_t isr; /* The interrupt service routine */
    void  *arg; /* Argument passed to it */
  }
  pins[NXXX_GPIO_NPINS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nxxx_portisr_s g_isrtab[NXXX_GPIO_NPORTS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxxx_gpio_interrupt
 *
 * Description:
 *   GPIO interrupt handlers. NXXx has two interrupt sources for each pin,
 *   the NuttX driver uses source 0.
 *
 ****************************************************************************/

static int nxxx_gpio_interrupt(int irq, void *context, void *arg)
{
  uint32_t port = (uint32_t)((uintptr_t)arg) >> GPIO_PORT_SHIFT;
  uint32_t status;
  uint32_t pin;
  uint32_t regaddr;

  /* Get the pending interrupt indications */

  regaddr = NXXX_GPIO_ISFR0(port);
  status  = getreg32(regaddr);

  /* Decode the pending interrupts */

  for (pin = 0; pin < 32 && status != 0; pin++)
    {
      /* Is the IRQ associated with this pin pending? */

      uint32_t mask = (1 << pin);
      if ((status & mask) != 0)
        {
          struct nxxx_portisr_s *isrtab;

          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, regaddr);
          status &= ~mask;

          /* Get the interrupt table for this port */

          isrtab = &g_isrtab[port];
          if (isrtab->pins[pin].isr != NULL)
            {
              /* Run the user handler with the user's argument */

              isrtab->pins[pin].isr(irq, context, isrtab->pins[pin].arg);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxxx_gpioirq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void nxxx_gpioirq_initialize(void)
{
  uint32_t port;
  uint32_t pin;

  /* Disable all GPIO interrupts at the source */

  for (port = 0; port < NXXX_GPIO_NPORTS; port++)
    {
      for (pin = 0; pin < NXXX_GPIO_NPINS; pin++)
        {
          /* Reset the interrupt configuration, disabling the interrupt */

          putreg32(0, NXXX_GPIO_ICRN(port, pin));
        }
    }

  /* Disable all GPIO interrupts */

  up_disable_irq(NXXX_IRQ_GPIO00);
  up_disable_irq(NXXX_IRQ_GPIO01);

  up_disable_irq(NXXX_IRQ_GPIO10);
  up_disable_irq(NXXX_IRQ_GPIO11);

  up_disable_irq(NXXX_IRQ_GPIO20);
  up_disable_irq(NXXX_IRQ_GPIO21);

  up_disable_irq(NXXX_IRQ_GPIO30);
  up_disable_irq(NXXX_IRQ_GPIO31);

  up_disable_irq(NXXX_IRQ_GPIO40);
  up_disable_irq(NXXX_IRQ_GPIO41);

  up_disable_irq(NXXX_IRQ_GPIO50);
  up_disable_irq(NXXX_IRQ_GPIO51);

  /* Attach the common GPIO interrupt handler and enable the interrupt */

  DEBUGVERIFY(irq_attach(NXXX_IRQ_GPIO00,
                         nxxx_gpio_interrupt, (void *)GPIO_PORT0));
  up_enable_irq(NXXX_IRQ_GPIO00);

  DEBUGVERIFY(irq_attach(NXXX_IRQ_GPIO10,
                         nxxx_gpio_interrupt, (void *)GPIO_PORT1));
  up_enable_irq(NXXX_IRQ_GPIO10);

  DEBUGVERIFY(irq_attach(NXXX_IRQ_GPIO20,
                         nxxx_gpio_interrupt, (void *)GPIO_PORT2));
  up_enable_irq(NXXX_IRQ_GPIO20);

  DEBUGVERIFY(irq_attach(NXXX_IRQ_GPIO30,
                         nxxx_gpio_interrupt, (void *)GPIO_PORT3));
  up_enable_irq(NXXX_IRQ_GPIO30);

  DEBUGVERIFY(irq_attach(NXXX_IRQ_GPIO40,
                         nxxx_gpio_interrupt, (void *)GPIO_PORT4));
  up_enable_irq(NXXX_IRQ_GPIO40);

  DEBUGVERIFY(irq_attach(NXXX_IRQ_GPIO50,
                         nxxx_gpio_interrupt, (void *)GPIO_PORT5));
  up_enable_irq(NXXX_IRQ_GPIO50);
}

/****************************************************************************
 * Name: nxxx_gpioirq_attach
 *
 * Description:
 *   Attach a pin interrupt handler.
 *
 ****************************************************************************/

int nxxx_gpioirq_attach(gpio_pinset_t pinset, xcpt_t isr, void *arg)
{
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Atomically change the handler */

  irqstate_t flags = enter_critical_section();

  g_isrtab[port].pins[pin].isr = isr;
  g_isrtab[port].pins[pin].arg = arg;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: nxxx_gpioirq_configure
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ****************************************************************************/

int nxxx_gpioirq_configure(gpio_pinset_t pinset)
{
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Nothing much to do here, just reset the IRQ config */

  putreg32(0, NXXX_GPIO_ICRN(port, pin));

  return OK;
}

/****************************************************************************
 * Name: nxxx_gpioirq_enable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

int nxxx_gpioirq_enable(gpio_pinset_t pinset)
{
  uint32_t  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t  pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint32_t  both = (pinset & GPIO_INTBOTHCFG_MASK) >> GPIO_INTBOTHCFG_SHIFT;
  uint32_t  icr  = (pinset & GPIO_INTCFG_MASK);
  uint32_t  regval;
  uintptr_t regaddr;

  /* Perform RMW to the specific pin */

  regaddr = NXXX_GPIO_ICRN(port, pin);
  regval  = getreg32(regaddr);
  regval &= ~NXXX_GPIO_ICRN_MASK;

  if (both)
    {
      regval |= NXXX_GPIO_ICRN_BOTH;
    }
  else if (icr == GPIO_INT_LOWLEVEL)
    {
      regval |= NXXX_GPIO_ICRN_ZERO;
    }
  else if (icr == GPIO_INT_HIGHLEVEL)
    {
      regval |= NXXX_GPIO_ICRN_ONE;
    }
  else if (icr == GPIO_INT_RISINGEDGE)
    {
      regval |= NXXX_GPIO_ICRN_RISING;
    }
  else /* GPIO_INT_FALLINGEDGE */
    {
      regval |= NXXX_GPIO_ICRN_FALLING;
    }

  putreg32(regval, regaddr);
  return OK;
}

/****************************************************************************
 * Name: nxxx_gpioirq_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

int nxxx_gpioirq_disable(gpio_pinset_t pinset)
{
  uint32_t  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t  pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint32_t  regval;
  uintptr_t regaddr;

  /* Perform RMW to the specific pin */

  regaddr = NXXX_GPIO_ICRN(port, pin);
  regval  = getreg32(regaddr);
  regval &= ~NXXX_GPIO_ICRN_MASK;

  putreg32(regval, regaddr);
  return OK;
}

#endif /* CONFIG_NXXX_GPIO_IRQ */
