/****************************************************************************
 * arch/arm64/src/imx9/imx9_gpioirq.c
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

#include "arm64_internal.h"
#include "imx9_gpio.h"

#ifdef CONFIG_IMX9_GPIO_IRQ

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx9_portisr_s
{
  struct
  {
    xcpt_t isr; /* The interrupt service routine */
    void  *arg; /* Argument passed to it */
  }
  pins[IMX9_GPIO_NPINS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imx9_portisr_s g_isrtab[IMX9_GPIO_NPORTS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_gpioN_A_B_interrupt
 *
 * Description:
 *   GPIO interrupt handlers.
 *
 ****************************************************************************/

static int imx9_gpio_0_15_interrupt(int irq, void *context, void *arg)
{
  uint32_t port = *(uint32_t *)arg;
  uint32_t status;
  uint32_t pin;
  uint32_t regaddr;

  /* Get the pending interrupt indications */

  regaddr = IMX9_GPIO_ISFR0(port);
  status  = getreg32(regaddr) & 0x0000ffff;

  /* Decode the pending interrupts */

  for (pin = 0; pin < 16 && status != 0; pin++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << pin);
      if ((status & mask) != 0)
        {
          struct imx9_portisr_s *isrtab;

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

static int imx9_gpio_16_31_interrupt(int irq, void *context, void *arg)
{
  uint32_t port = *(uint32_t *)arg;
  uint32_t status;
  uint32_t pin;
  uint32_t regaddr;

  /* Get the pending interrupt indications */

  regaddr = IMX9_GPIO_ISFR0(port);
  status  = getreg32(regaddr) & 0xffff0000;

  /* Decode the pending interrupts */

  for (pin = 16; pin < 32 && status != 0; pin++)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << pin);
      if ((status & mask) != 0)
        {
          struct imx9_portisr_s *isrtab;

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
 * Name: imx9_gpioirq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void imx9_gpioirq_initialize(void)
{
  uint32_t port;
  uint32_t pin;

  /* Disable all GPIO interrupts at the source */

  for (port = 0; port < IMX9_GPIO_NPORTS; port++)
    {
      for (pin = 0; pin < IMX9_GPIO_NPINS; pin++)
        {
          /* Reset the interrupt configuration, disabling the interrupt */

          putreg32(0, IMX9_GPIO_ICRN(port, pin));
        }
    }

  /* Disable all unconfigured GPIO interrupts at the NVIC */

  up_disable_irq(IMX9_IRQ_GPIO1_0_15);
  up_disable_irq(IMX9_IRQ_GPIO1_16_31);

  up_disable_irq(IMX9_IRQ_GPIO2_0_15);
  up_disable_irq(IMX9_IRQ_GPIO2_16_31);

  up_disable_irq(IMX9_IRQ_GPIO3_0_15);
  up_disable_irq(IMX9_IRQ_GPIO3_16_31);

  up_disable_irq(IMX9_IRQ_GPIO4_0_15);
  up_disable_irq(IMX9_IRQ_GPIO4_16_31);

  /* Attach all configured GPIO interrupts and enable the interrupt at the
   * NVIC
   */

  DEBUGVERIFY(irq_attach(IMX9_IRQ_GPIO1_0_15,
                         imx9_gpio_0_15_interrupt, (void *)1));
  up_enable_irq(IMX9_IRQ_GPIO1_0_15);
  DEBUGVERIFY(irq_attach(IMX9_IRQ_GPIO1_16_31,
                         imx9_gpio_16_31_interrupt, (void *)1));
  up_enable_irq(IMX9_IRQ_GPIO1_16_31);

  DEBUGVERIFY(irq_attach(IMX9_IRQ_GPIO2_0_15,
                         imx9_gpio_0_15_interrupt, (void *)2));
  up_enable_irq(IMX9_IRQ_GPIO1_0_15);
  DEBUGVERIFY(irq_attach(IMX9_IRQ_GPIO2_16_31,
                         imx9_gpio_16_31_interrupt, (void *)2));
  up_enable_irq(IMX9_IRQ_GPIO1_16_31);

  DEBUGVERIFY(irq_attach(IMX9_IRQ_GPIO3_0_15,
                         imx9_gpio_0_15_interrupt, (void *)3));
  up_enable_irq(IMX9_IRQ_GPIO1_0_15);
  DEBUGVERIFY(irq_attach(IMX9_IRQ_GPIO3_16_31,
                         imx9_gpio_16_31_interrupt, (void *)3));
  up_enable_irq(IMX9_IRQ_GPIO1_16_31);

  DEBUGVERIFY(irq_attach(IMX9_IRQ_GPIO4_0_15,
                         imx9_gpio_0_15_interrupt, (void *)4));
  up_enable_irq(IMX9_IRQ_GPIO1_0_15);
  DEBUGVERIFY(irq_attach(IMX9_IRQ_GPIO4_16_31,
                         imx9_gpio_16_31_interrupt, (void *)4));
  up_enable_irq(IMX9_IRQ_GPIO1_16_31);
}

/****************************************************************************
 * Name: imx9_gpioirq_attach
 *
 * Description:
 *   Attach a pin interrupt handler.
 *
 ****************************************************************************/

int imx9_gpioirq_attach(gpio_pinset_t pinset, xcpt_t isr, void *arg)
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
 * Name: imx9_gpioirq_configure
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ****************************************************************************/

int imx9_gpioirq_configure(gpio_pinset_t pinset)
{
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Nothing much to do here, just reset the IRQ config */

  putreg32(0, IMX9_GPIO_ICRN(port, pin));

  return OK;
}

/****************************************************************************
 * Name: imx9_gpioirq_enable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

int imx9_gpioirq_enable(gpio_pinset_t pinset)
{
  uint32_t  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t  pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint32_t  icr  = (pinset & GPIO_INTCFG_MASK) >> GPIO_INTCFG_SHIFT;
  uint32_t  both = (pinset & GPIO_INTBOTHCFG_MASK) >> GPIO_INTBOTHCFG_SHIFT;
  uint32_t  regval;
  uintptr_t regaddr;

  /* Perform RMW to the specific pin */

  regaddr = IMX9_GPIO_ICRN(port, pin);
  regval  = getreg32(regaddr);
  regval &= ~IMX9_GPIO_ICRN_MASK;

  if (both)
    {
      regval |= IMX9_GPIO_ICRN_BOTH;
    }
  else if (icr == GPIO_INT_LOWLEVEL)
    {
      regval |= IMX9_GPIO_ICRN_ZERO;
    }
  else if (icr == GPIO_INT_HIGHLEVEL)
    {
      regval |= IMX9_GPIO_ICRN_ONE;
    }
  else if (icr == GPIO_INT_RISINGEDGE)
    {
      regval |= IMX9_GPIO_ICRN_RISING;
    }
  else /* GPIO_INT_FALLINGEDGE */
    {
      regval |= IMX9_GPIO_ICRN_FALLING;
    }

  putreg32(regval, regaddr);
  return OK;
}

/****************************************************************************
 * Name: imx9_gpioirq_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

int imx9_gpioirq_disable(gpio_pinset_t pinset)
{
  uint32_t  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t  pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint32_t  regval;
  uintptr_t regaddr;

  /* Perform RMW to the specific pin */

  regaddr = IMX9_GPIO_ICRN(port, pin);
  regval  = getreg32(regaddr);
  regval &= ~IMX9_GPIO_ICRN_MASK;

  putreg32(regval, regaddr);
  return OK;
}

#endif /* CONFIG_IMX9_GPIO_IRQ */
