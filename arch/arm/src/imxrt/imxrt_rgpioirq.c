/****************************************************************************
 * arch/arm/src/imxrt/imxrt_rgpioirq.c
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

#include "arm_internal.h"
#include "imxrt_gpio.h"

#ifdef CONFIG_IMXRT_GPIO_IRQ

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_portisr_s
{
  struct
  {
    xcpt_t isr;
    void  *arg;
  }
  pins[IMXRT_GPIO_NPINS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imxrt_portisr_s g_isrtab[IMXRT_GPIO_NPORTS];

/* NVIC IRQ per RGPIO instance (channel 0).  RGPIO1..3 also expose a
 * second interrupt (channel 1) at IMXRT_IRQ_GPIOn_1; the driver uses
 * channel 0 for all pins by leaving ICRN.IRQS at reset (0).
 */

static const int g_gpio_irq[IMXRT_GPIO_NPORTS] =
{
  IMXRT_IRQ_GPIO1_0,
  IMXRT_IRQ_GPIO2_0,
  IMXRT_IRQ_GPIO3_0,
  IMXRT_IRQ_GPIO4,
  IMXRT_IRQ_GPIO5,
  IMXRT_IRQ_GPIO6,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_gpio_interrupt
 *
 * Description:
 *   Common ISR shared by every RGPIO instance.  Scans the port's ISFR0
 *   flag word and dispatches per-pin user handlers.
 *
 ****************************************************************************/

static int imxrt_gpio_interrupt(int irq, void *context, void *arg)
{
  uint32_t port = (uint32_t)((uintptr_t)arg);
  uint32_t status;
  uint32_t pin;
  uintptr_t regaddr;

  regaddr = IMXRT_GPIO_ISFR0(port);
  status  = getreg32(regaddr);

  for (pin = 0; pin < IMXRT_GPIO_NPINS && status != 0; pin++)
    {
      uint32_t mask = (1u << pin);

      if ((status & mask) != 0)
        {
          struct imxrt_portisr_s *isrtab;

          putreg32(mask, regaddr);
          status &= ~mask;

          isrtab = &g_isrtab[port];
          if (isrtab->pins[pin].isr != NULL)
            {
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
 * Name: imxrt_gpioirq_initialize
 ****************************************************************************/

void imxrt_gpioirq_initialize(void)
{
  uint32_t port;
  uint32_t pin;

  /* Reset every pin's ICR so no interrupt configuration survives across
   * a warm reset from the M33.
   */

  for (port = 0; port < IMXRT_GPIO_NPORTS; port++)
    {
      for (pin = 0; pin < IMXRT_GPIO_NPINS; pin++)
        {
          putreg32(0, IMXRT_GPIO_ICRN(port, pin));
        }
    }

  /* Attach the common ISR to every RGPIO NVIC line. */

  for (port = 0; port < IMXRT_GPIO_NPORTS; port++)
    {
      up_disable_irq(g_gpio_irq[port]);
      DEBUGVERIFY(irq_attach(g_gpio_irq[port],
                             imxrt_gpio_interrupt,
                             (void *)(uintptr_t)port));
      up_enable_irq(g_gpio_irq[port]);
    }
}

/****************************************************************************
 * Name: imxrt_gpioirq_attach
 ****************************************************************************/

int imxrt_gpioirq_attach(gpio_pinset_t pinset, xcpt_t isr, void *arg)
{
  uint16_t   gpio = IMXRT_PINSET_GPIO(pinset);
  uint32_t   port = (gpio & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t   pin  = (gpio & GPIO_PIN_MASK)  >> GPIO_PIN_SHIFT;
  irqstate_t flags;

  DEBUGASSERT(port < IMXRT_GPIO_NPORTS && pin < IMXRT_GPIO_NPINS);

  flags = enter_critical_section();

  g_isrtab[port].pins[pin].isr = isr;
  g_isrtab[port].pins[pin].arg = arg;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: imxrt_gpioirq_configure
 *
 * Description:
 *   Reset ICRN for the pin.  Interrupt is enabled by imxrt_gpioirq_enable().
 *
 ****************************************************************************/

int imxrt_gpioirq_configure(gpio_pinset_t pinset)
{
  uint16_t gpio = IMXRT_PINSET_GPIO(pinset);
  uint32_t port = (gpio & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (gpio & GPIO_PIN_MASK)  >> GPIO_PIN_SHIFT;

  DEBUGASSERT(port < IMXRT_GPIO_NPORTS && pin < IMXRT_GPIO_NPINS);

  putreg32(0, IMXRT_GPIO_ICRN(port, pin));
  return OK;
}

/****************************************************************************
 * Name: imxrt_gpioirq_enable
 ****************************************************************************/

int imxrt_gpioirq_enable(gpio_pinset_t pinset)
{
  uint16_t  gpio = IMXRT_PINSET_GPIO(pinset);
  uint32_t  port = (gpio & GPIO_PORT_MASK)       >> GPIO_PORT_SHIFT;
  uint32_t  pin  = (gpio & GPIO_PIN_MASK)        >> GPIO_PIN_SHIFT;
  uint32_t  both = (gpio & GPIO_INTBOTHCFG_MASK) >> GPIO_INTBOTHCFG_SHIFT;
  uint32_t  icr  = (gpio & GPIO_INTCFG_MASK);
  uint32_t  regval;
  uintptr_t regaddr;

  DEBUGASSERT(port < IMXRT_GPIO_NPORTS && pin < IMXRT_GPIO_NPINS);

  regaddr = IMXRT_GPIO_ICRN(port, pin);
  regval  = getreg32(regaddr);
  regval &= ~IMXRT_GPIO_ICRN_MASK;

  if (both)
    {
      regval |= IMXRT_GPIO_ICRN_BOTH;
    }
  else if (icr == GPIO_INT_LOWLEVEL)
    {
      regval |= IMXRT_GPIO_ICRN_ZERO;
    }
  else if (icr == GPIO_INT_HIGHLEVEL)
    {
      regval |= IMXRT_GPIO_ICRN_ONE;
    }
  else if (icr == GPIO_INT_RISINGEDGE)
    {
      regval |= IMXRT_GPIO_ICRN_RISING;
    }
  else
    {
      regval |= IMXRT_GPIO_ICRN_FALLING;
    }

  putreg32(regval, regaddr);
  return OK;
}

/****************************************************************************
 * Name: imxrt_gpioirq_disable
 ****************************************************************************/

int imxrt_gpioirq_disable(gpio_pinset_t pinset)
{
  uint16_t  gpio = IMXRT_PINSET_GPIO(pinset);
  uint32_t  port = (gpio & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t  pin  = (gpio & GPIO_PIN_MASK)  >> GPIO_PIN_SHIFT;
  uint32_t  regval;
  uintptr_t regaddr;

  DEBUGASSERT(port < IMXRT_GPIO_NPORTS && pin < IMXRT_GPIO_NPINS);

  regaddr = IMXRT_GPIO_ICRN(port, pin);
  regval  = getreg32(regaddr);
  regval &= ~IMXRT_GPIO_ICRN_MASK;

  putreg32(regval, regaddr);
  return OK;
}

#endif /* CONFIG_IMXRT_GPIO_IRQ */
