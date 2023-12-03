/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_gpio.c
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
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "arm_internal.h"
#include "mx8mp_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* GPIO context for IRQ management */

struct mx8mp_gpio_s
{
  const uint32_t isr;
  const uint32_t imr;
  const uint16_t irq_start;
  const uint16_t bit;
  const uint32_t mask;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mx8mp_gpio_s g_gpio1_l =
{
  .isr        = GPIO_ISR(1),
  .imr        = GPIO_IMR(1),
  .irq_start  = MX8MP_IRQ_SOFT_GPIO1_0,
  .bit        = 0,
  .mask       = 0x0000ffff
};

static struct mx8mp_gpio_s g_gpio1_h =
{
  .isr        = GPIO_ISR(1),
  .imr        = GPIO_IMR(1),
  .irq_start  = MX8MP_IRQ_SOFT_GPIO1_16,
  .bit        = 16,
  .mask       = 0xffff0000
};

static  struct mx8mp_gpio_s g_gpio2_l =
{
  .isr        = GPIO_ISR(2),
  .imr        = GPIO_IMR(2),
  .irq_start  = MX8MP_IRQ_SOFT_GPIO2_0,
  .bit        = 0,
  .mask       = 0x0000ffff
};

static struct mx8mp_gpio_s g_gpio2_h =
{
  .isr        = GPIO_ISR(2),
  .imr        = GPIO_IMR(2),
  .irq_start  = MX8MP_IRQ_SOFT_GPIO2_16,
  .bit        = 16,
  .mask       = 0xffff0000
};

static struct mx8mp_gpio_s g_gpio3_l =
{
  .isr        = GPIO_ISR(3),
  .imr        = GPIO_IMR(3),
  .irq_start  = MX8MP_IRQ_SOFT_GPIO3_0,
  .bit        = 0,
  .mask       = 0x0000ffff
};

static struct mx8mp_gpio_s g_gpio3_h =
{
  .isr        = GPIO_ISR(3),
  .imr        = GPIO_IMR(3),
  .irq_start  = MX8MP_IRQ_SOFT_GPIO3_16,
  .bit        = 16,
  .mask       = 0xffff0000
};

static struct mx8mp_gpio_s g_gpio4_l =
{
  .isr        = GPIO_ISR(4),
  .imr        = GPIO_IMR(4),
  .irq_start  = MX8MP_IRQ_SOFT_GPIO4_0,
  .bit        = 0,
  .mask       = 0x0000ffff
};

static struct mx8mp_gpio_s g_gpio4_h =
{
  .isr        = GPIO_ISR(4),
  .imr        = GPIO_IMR(4),
  .irq_start  = MX8MP_IRQ_SOFT_GPIO4_16,
  .bit        = 16,
  .mask       = 0xffff0000
};

static struct mx8mp_gpio_s g_gpio5_l =
{
  .isr        = GPIO_ISR(5),
  .imr        = GPIO_IMR(5),
  .irq_start  = MX8MP_IRQ_SOFT_GPIO5_0,
  .bit        = 0,
  .mask       = 0x0000ffff
};

static struct mx8mp_gpio_s g_gpio5_h =
{
  .isr        = GPIO_ISR(5),
  .imr        = GPIO_IMR(5),
  .irq_start  = MX8MP_IRQ_SOFT_GPIO5_16,
  .bit        = 16,
  .mask       = 0xffff0000
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int mx8mp_gpio_interrupt(int irq, void *context, void *arg)
{
  struct mx8mp_gpio_s *cfg = (struct mx8mp_gpio_s *)(arg);

  uint32_t status;
  int i;

  /* Get the pending interrupt indications */

  status = getreg32(cfg->isr) & getreg32(cfg->imr) & cfg->mask;

  /* Decode the pending interrupts */

  for (i = 0; (i < 16) && (status != 0); ++i)
    {
      /* Is the IRQ associate with this pin pending? */

      uint32_t mask = (1 << (cfg->bit + i));
      if ((status & mask) != 0)
        {
          /* Yes, clear the status bit and dispatch the interrupt */

          putreg32(mask, cfg->isr);
          status &= ~mask;

          irq_dispatch(cfg->irq_start + i, context);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description of the pin.
 *
 ****************************************************************************/

int mx8mp_gpio_config(gpio_pinset_t pinset)
{
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK)  >> GPIO_PIN_SHIFT;
  bool value    = ((pinset & GPIO_OUTPUT_ONE) != 0);

  irqstate_t flags;
  int ret = OK;

  /* Configure the pin as an input initially to avoid any spurious outputs */

  flags = enter_critical_section();

  /* Configure based upon the pin mode */

  switch (pinset & GPIO_MODE_MASK)
    {
      case GPIO_INPUT:
        {
          /* Configure the pin as a GPIO input */

          modreg32(0, GPIO_PIN(pin), GPIO_GDIR(port));
        }
        break;

      case GPIO_OUTPUT:
        {
          /* First configure the pin as a GPIO input to avoid output
            * glitches.
            */

          modreg32(0, GPIO_PIN(pin), GPIO_GDIR(port));

          /* Set the output value */

          mx8mp_gpio_write(pinset, value);

          /* Convert the configured input GPIO to an output */

          modreg32(GPIO_PIN(pin), GPIO_PIN(pin), GPIO_GDIR(port));
        }
        break;

      case GPIO_INTERRUPT:
        {
          /* Configure the pin as a GPIO input then the IRQ behavior */

          modreg32(0, GPIO_PIN(pin), GPIO_GDIR(port));

          mx8mp_gpio_configure_irq(pinset);
        }
        break;

      default:
        ret = -EINVAL;
        break;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: mx8mp_gpio_irq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

void mx8mp_gpio_irq_initialize(void)
{
  /* Disable all GPIO interrupts at the source */

  putreg32(0, GPIO_IMR(1));
  putreg32(0, GPIO_IMR(2));
  putreg32(0, GPIO_IMR(3));
  putreg32(0, GPIO_IMR(4));
  putreg32(0, GPIO_IMR(5));

  /* Disable all GPIO interrupts at the NVIC */

  up_disable_irq(MX8MP_IRQ_GPIO1_0_15);
  up_disable_irq(MX8MP_IRQ_GPIO1_16_31);

  up_disable_irq(MX8MP_IRQ_GPIO2_0_15);
  up_disable_irq(MX8MP_IRQ_GPIO2_16_31);

  up_disable_irq(MX8MP_IRQ_GPIO3_0_15);
  up_disable_irq(MX8MP_IRQ_GPIO3_16_31);

  up_disable_irq(MX8MP_IRQ_GPIO4_0_15);
  up_disable_irq(MX8MP_IRQ_GPIO4_16_31);

  up_disable_irq(MX8MP_IRQ_GPIO5_0_15);
  up_disable_irq(MX8MP_IRQ_GPIO5_16_31);

  /* Attach all GPIO interrupts and enable the interrupt at the NVIC */

  irq_attach(MX8MP_IRQ_GPIO1_0_15, mx8mp_gpio_interrupt, &g_gpio1_l);
  up_enable_irq(MX8MP_IRQ_GPIO1_0_15);
  irq_attach(MX8MP_IRQ_GPIO1_16_31, mx8mp_gpio_interrupt, &g_gpio1_h);
  up_enable_irq(MX8MP_IRQ_GPIO1_16_31);

  irq_attach(MX8MP_IRQ_GPIO2_0_15, mx8mp_gpio_interrupt, &g_gpio2_l);
  up_enable_irq(MX8MP_IRQ_GPIO2_0_15);
  irq_attach(MX8MP_IRQ_GPIO2_16_31, mx8mp_gpio_interrupt, &g_gpio2_h);
  up_enable_irq(MX8MP_IRQ_GPIO2_16_31);

  irq_attach(MX8MP_IRQ_GPIO3_0_15, mx8mp_gpio_interrupt, &g_gpio3_l);
  up_enable_irq(MX8MP_IRQ_GPIO3_0_15);
  irq_attach(MX8MP_IRQ_GPIO3_16_31, mx8mp_gpio_interrupt, &g_gpio3_h);
  up_enable_irq(MX8MP_IRQ_GPIO3_16_31);

  irq_attach(MX8MP_IRQ_GPIO4_0_15, mx8mp_gpio_interrupt, &g_gpio4_l);
  up_enable_irq(MX8MP_IRQ_GPIO4_0_15);
  irq_attach(MX8MP_IRQ_GPIO4_16_31, mx8mp_gpio_interrupt, &g_gpio4_h);
  up_enable_irq(MX8MP_IRQ_GPIO4_16_31);

  irq_attach(MX8MP_IRQ_GPIO5_0_15, mx8mp_gpio_interrupt, &g_gpio5_l);
  up_enable_irq(MX8MP_IRQ_GPIO5_0_15);
  irq_attach(MX8MP_IRQ_GPIO5_16_31, mx8mp_gpio_interrupt, &g_gpio5_h);
  up_enable_irq(MX8MP_IRQ_GPIO5_16_31);
}

/****************************************************************************
 * Name: mx8mp_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void mx8mp_gpio_write(gpio_pinset_t pinset, bool value)
{
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint32_t regval;
  irqstate_t flags;

  flags = enter_critical_section();

  regval = getreg32(GPIO_DR(port));
  if (value)
    {
      regval |= GPIO_PIN(pin);
    }
  else
    {
      regval &= ~GPIO_PIN(pin);
    }

  putreg32(regval, GPIO_DR(port));

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: mx8mp_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool mx8mp_gpio_read(gpio_pinset_t pinset)
{
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint32_t regval;
  irqstate_t flags;

  flags = enter_critical_section();

  if ((pinset & (GPIO_OUTPUT)) == (GPIO_OUTPUT))
    {
      regval = getreg32(GPIO_PSR(port));
    }
  else
    {
      regval = getreg32(GPIO_DR(port));
    }

  leave_critical_section(flags);
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: mx8mp_gpio_configure_irq
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ****************************************************************************/

void mx8mp_gpio_configure_irq(gpio_pinset_t pinset)
{
  /* Decode information in the pin configuration */

  uint32_t port = (pinset & GPIO_PORT_MASK)   >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK)    >> GPIO_PIN_SHIFT;
  uint32_t icr  = (pinset & GPIO_INTCFG_MASK) >> GPIO_INTCFG_SHIFT;
  uint32_t both = (pinset & GPIO_INTBOTHCFG_MASK) >> GPIO_INTBOTHCFG_SHIFT;
  uintptr_t regaddr;
  uint32_t regval;

  /* Set the right field in the right ICR register */

  regaddr = pin < 16 ? GPIO_ICR1(port) : GPIO_ICR2(port);
  regval  = getreg32(regaddr);
  regval &= ~ICR_MASK(pin);
  regval |= ICR(icr, pin);
  putreg32(regval, regaddr);

  /* Add any both-edge setup (overrides above see User Manual 8.3.5.1.9) */

  regaddr = GPIO_EDGE(port);
  regval = getreg32(regaddr);
  regval &= ~GPIO_PIN(pin);
  regval |= (both << pin);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: mx8mp_gpio_irq_enable
 *
 * Description:
 *   Enable the interrupt for specified GPIO
 *
 ****************************************************************************/

void mx8mp_gpio_irq_enable(gpio_pinset_t pinset)
{
  uint32_t port = (pinset & GPIO_PORT_MASK)   >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK)    >> GPIO_PIN_SHIFT;

  modifyreg32(GPIO_IMR(port), 0, GPIO_PIN(pin));
}

/****************************************************************************
 * Name: mx8mp_gpio_irq_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO
 *
 ****************************************************************************/

void mx8mp_gpio_irq_disable(gpio_pinset_t pinset)
{
  uint32_t port = (pinset & GPIO_PORT_MASK)   >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK)    >> GPIO_PIN_SHIFT;

  modifyreg32(GPIO_IMR(port), GPIO_PIN(pin), 0);
}
