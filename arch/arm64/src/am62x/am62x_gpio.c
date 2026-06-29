/****************************************************************************
 * arch/arm64/src/am62x/am62x_gpio.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "hardware/am62x_gpio.h"
#include "hardware/am62x_memorymap.h"
#include "am62x_gpio.h"
#include "am62x_tisci.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am62x_gpio_callback_s
{
  xcpt_t handler;
  void *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uintptr_t g_gpiobase[AM62X_GPIO_NBANKS] =
{
  AM62X_GPIO0_BASE,
  AM62X_GPIO1_BASE,
};

static const int g_gpioirq[AM62X_GPIO_NBANKS] =
{
  AM62X_IRQ_GPIO0,
  AM62X_IRQ_GPIO1,
};

#ifdef CONFIG_AM62X_TISCI
/* TISCI power-domain / device id for each GPIO bank.  The bank must be
 * powered on through the system firmware before its registers are touched.
 */

static const uint32_t g_gpiopd[AM62X_GPIO_NBANKS] =
{
  AM62X_DEV_GPIO0,
  AM62X_DEV_GPIO1,
};

static const uint32_t g_gpioclk[AM62X_GPIO_NBANKS] =
{
  0,
  0,
};
#endif

static struct am62x_gpio_callback_s
  g_callbacks[AM62X_GPIO_NBANKS][AM62X_GPIO_NPINS];

static bool g_gpio_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int am62x_gpio_decode(gpio_pinset_t pinset, unsigned int *bank,
                             unsigned int *line)
{
  *bank = AM62X_GPIO_BANK(pinset);
  *line = AM62X_GPIO_LINE(pinset);

  if (*bank >= AM62X_GPIO_NBANKS || *line >= AM62X_GPIO_NPINS)
    {
      return -EINVAL;
    }

  return OK;
}

static int am62x_gpio_interrupt(int irq, void *context, void *arg)
{
  unsigned int bank = (uintptr_t)arg;
  uintptr_t base = g_gpiobase[bank];
  uint32_t pending;
  unsigned int line;

  pending = getreg32(AM62X_GPIO_INTSTAT(base, 0));
  putreg32(pending, AM62X_GPIO_INTSTAT(base, 0));

  for (line = 0; line < AM62X_GPIO_NPINS; line++)
    {
      if ((pending & AM62X_GPIO_BIT(line)) != 0 &&
          g_callbacks[bank][line].handler != NULL)
        {
          g_callbacks[bank][line].handler(irq, context,
                                          g_callbacks[bank][line].arg);
        }
    }

  putreg32(0, AM62X_GPIO_BINTEN(base));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int am62x_gpio_initialize(void)
{
  unsigned int bank;
  int ret;

  if (g_gpio_initialized)
    {
      return OK;
    }

  for (bank = 0; bank < AM62X_GPIO_NBANKS; bank++)
    {
      uintptr_t base = g_gpiobase[bank];

#ifdef CONFIG_AM62X_TISCI
      /* Power, release reset, and clock the GPIO bank before any register
       * access.  On K3 an access to a gated block faults the bus.
       */

      ret = am62x_tisci_module_enable(g_gpiopd[bank], g_gpioclk[bank]);
      if (ret < 0)
        {
          return ret;
        }
#endif

      putreg32(0xffffffff, AM62X_GPIO_CLR_RIS_TRIG(base, 0));
      putreg32(0xffffffff, AM62X_GPIO_CLR_FAL_TRIG(base, 0));
      putreg32(0xffffffff, AM62X_GPIO_INTSTAT(base, 0));
      putreg32(0, AM62X_GPIO_BINTEN(base));

      ret = irq_attach(g_gpioirq[bank], am62x_gpio_interrupt,
                       (void *)(uintptr_t)bank);
      if (ret < 0)
        {
          return ret;
        }

      /* GPIO bank interrupts on AM62x are not delivered directly to the GIC:
       * they pass through the main GPIO interrupt router, which must be
       * programmed via TISCI (RM_IRQ_SET) before the GIC input is valid.
       * Until that routing is wired up, enabling the GIC line delivers a
       * spurious/unrouted interrupt and faults.  GPIO input/output works
       * without it; gate the enable behind CONFIG_AM62X_GPIO_IRQ.
       */

#ifdef CONFIG_AM62X_GPIO_IRQ
#ifdef CONFIG_ARCH_IRQPRIO
      (void)up_prioritize_irq(g_gpioirq[bank], 0);
#endif
#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
      (void)up_set_irq_type(g_gpioirq[bank], IRQ_HIGH_LEVEL);
#endif
      up_enable_irq(g_gpioirq[bank]);
#endif
    }

  g_gpio_initialized = true;
  return OK;
}

int am62x_configgpio(gpio_pinset_t cfgset)
{
  irqstate_t flags;
  uintptr_t base;
  uint32_t bit;
  uint32_t regval;
  unsigned int bank;
  unsigned int line;
  int ret;

  ret = am62x_gpio_decode(cfgset, &bank, &line);
  if (ret < 0)
    {
      return ret;
    }

  base = g_gpiobase[bank];
  bit = AM62X_GPIO_BIT(line);

  flags = enter_critical_section();

  if (!AM62X_GPIO_IS_INPUT(cfgset))
    {
      putreg32(bit, AM62X_GPIO_INITVAL(cfgset) ?
                    AM62X_GPIO_SET_DATA(base, 0) :
                    AM62X_GPIO_CLR_DATA(base, 0));
    }

  regval = getreg32(AM62X_GPIO_DIR(base, 0));
  if (AM62X_GPIO_IS_INPUT(cfgset))
    {
      regval |= bit;
    }
  else
    {
      regval &= ~bit;
    }

  putreg32(regval, AM62X_GPIO_DIR(base, 0));

  putreg32(bit, AM62X_GPIO_CLR_RIS_TRIG(base, 0));
  putreg32(bit, AM62X_GPIO_CLR_FAL_TRIG(base, 0));

  if (AM62X_GPIO_IRQMODE(cfgset) == AM62X_GPIO_IRQ_RISING ||
      AM62X_GPIO_IRQMODE(cfgset) == AM62X_GPIO_IRQ_BOTH)
    {
      putreg32(bit, AM62X_GPIO_SET_RIS_TRIG(base, 0));
    }

  if (AM62X_GPIO_IRQMODE(cfgset) == AM62X_GPIO_IRQ_FALLING ||
      AM62X_GPIO_IRQMODE(cfgset) == AM62X_GPIO_IRQ_BOTH)
    {
      putreg32(bit, AM62X_GPIO_SET_FAL_TRIG(base, 0));
    }

  putreg32(bit, AM62X_GPIO_INTSTAT(base, 0));

  leave_critical_section(flags);
  return OK;
}

void am62x_gpiowrite(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  uintptr_t base;
  unsigned int bank;
  unsigned int line;

  if (am62x_gpio_decode(pinset, &bank, &line) < 0)
    {
      return;
    }

  base = g_gpiobase[bank];
  flags = enter_critical_section();
  putreg32(AM62X_GPIO_BIT(line), value ? AM62X_GPIO_SET_DATA(base, 0) :
                                    AM62X_GPIO_CLR_DATA(base, 0));
  leave_critical_section(flags);
}

bool am62x_gpioread(gpio_pinset_t pinset)
{
  irqstate_t flags;
  uintptr_t base;
  bool value;
  unsigned int bank;
  unsigned int line;

  if (am62x_gpio_decode(pinset, &bank, &line) < 0)
    {
      return false;
    }

  base = g_gpiobase[bank];
  flags = enter_critical_section();
  value = (getreg32(AM62X_GPIO_IN_DATA(base, 0)) &
           AM62X_GPIO_BIT(line)) != 0;
  leave_critical_section(flags);
  return value;
}

bool am62x_gpiooutread(gpio_pinset_t pinset)
{
  irqstate_t flags;
  uintptr_t base;
  bool value;
  unsigned int bank;
  unsigned int line;

  if (am62x_gpio_decode(pinset, &bank, &line) < 0)
    {
      return false;
    }

  base = g_gpiobase[bank];
  flags = enter_critical_section();
  value = (getreg32(AM62X_GPIO_OUT_DATA(base, 0)) &
           AM62X_GPIO_BIT(line)) != 0;
  leave_critical_section(flags);
  return value;
}

int am62x_gpio_irq_attach(gpio_pinset_t pinset, xcpt_t isr, void *arg)
{
  irqstate_t flags;
  unsigned int bank;
  unsigned int line;
  int ret;

  ret = am62x_gpio_decode(pinset, &bank, &line);
  if (ret < 0)
    {
      return ret;
    }

  ret = am62x_gpio_initialize();
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();
  g_callbacks[bank][line].handler = isr;
  g_callbacks[bank][line].arg = arg;
  leave_critical_section(flags);

  return OK;
}

void am62x_gpio_irq_detach(gpio_pinset_t pinset)
{
  irqstate_t flags;
  unsigned int bank;
  unsigned int line;

  if (am62x_gpio_decode(pinset, &bank, &line) < 0)
    {
      return;
    }

  flags = enter_critical_section();
  g_callbacks[bank][line].handler = NULL;
  g_callbacks[bank][line].arg = NULL;
  leave_critical_section(flags);
}

int am62x_pinmux_configure(uintptr_t offset, uint32_t value)
{
  putreg32(value, AM62X_PADCFG_BASE + offset);
  return OK;
}
