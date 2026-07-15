/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_gpio.c
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

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <arch/irq.h>

#include "riscv_internal.h"
#include "chip.h"
#include "gd32vw55x_gpio.h"
#include "hardware/gd32vw55x_gpio.h"
#include "hardware/gd32vw55x_rcu.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Base addresses for each GPIO port */

const uint32_t g_gpio_base[GD32VW55X_NGPIO_PORTS] =
{
  GD32VW55X_GPIOA,
  GD32VW55X_GPIOB,
  GD32VW55X_GPIOC,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_gpio_output_set
 *
 * Description:
 *   Set or clear the output of one GPIO pin.
 *
 ****************************************************************************/

static void gd32_gpio_output_set(uint32_t port_base, uint32_t pin,
                                 bool value)
{
  uintptr_t regaddr;
  uint32_t regval;

  if (value)
    {
      regval  = GPIO_BOP_SET(pin);
      regaddr = GD32VW55X_GPIO_BOP(port_base);
    }
  else
    {
      regval  = GPIO_BC_SET(pin);
      regaddr = GD32VW55X_GPIO_BC(port_base);
    }

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_gpio_input_get
 *
 * Description:
 *   Get the input state of one GPIO pin.
 *
 ****************************************************************************/

static inline bool gd32_gpio_input_get(uint32_t port_base, uint32_t pin)
{
  uint32_t regval;

  regval = getreg32(GD32VW55X_GPIO_ISTAT(port_base));

  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: gd32_gpio_af_config
 *
 * Description:
 *   Select the alternate function of a GPIO pin.
 *
 ****************************************************************************/

static void gd32_gpio_af_config(uint32_t cfgset, uint32_t port_base,
                                uint32_t pin)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t af_sel;
  uint32_t pos;

  af_sel = (cfgset & GPIO_CFG_AF_MASK) >> GPIO_CFG_AF_SHIFT;

  if (pin < 8)
    {
      regaddr = GD32VW55X_GPIO_AFSEL0(port_base);
      pos     = pin;
    }
  else
    {
      regaddr = GD32VW55X_GPIO_AFSEL1(port_base);
      pos     = pin - 8;
    }

  regval  = getreg32(regaddr);
  regval &= ~GPIO_AF_MASK(pos);
  regval |= (af_sel << GPIO_AF_SHIFT(pos));
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_gpio_af_unconfig
 *
 * Description:
 *   Reset the alternate function of a GPIO pin to its default value.
 *
 ****************************************************************************/

static void gd32_gpio_af_unconfig(uint32_t port_base, uint32_t pin)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t pos;

  if (pin < 8)
    {
      regaddr = GD32VW55X_GPIO_AFSEL0(port_base);
      pos     = pin;
    }
  else
    {
      regaddr = GD32VW55X_GPIO_AFSEL1(port_base);
      pos     = pin - 8;
    }

  regval  = getreg32(regaddr);
  regval &= ~GPIO_AF_MASK(pos);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_gpio_clock_enable
 *
 * Description:
 *   Enable the clock of the GPIO port that owns the given port base
 *   address.
 *
 ****************************************************************************/

void gd32_gpio_clock_enable(uint32_t port_base)
{
  uint32_t rcu_en;

  /* Determine which GPIO port to enable */

  switch (port_base)
    {
      case GD32VW55X_GPIOA:
        rcu_en = RCU_AHB1EN_PAEN;
        break;

      case GD32VW55X_GPIOB:
        rcu_en = RCU_AHB1EN_PBEN;
        break;

      case GD32VW55X_GPIOC:
        rcu_en = RCU_AHB1EN_PCEN;
        break;

      default:
        return;
    }

  /* Enable the AHB1 clock of the GPIO port if it is not already enabled */

  if ((getreg32(GD32VW55X_RCU_AHB1EN) & rcu_en) != rcu_en)
    {
      modifyreg32(GD32VW55X_RCU_AHB1EN, 0, rcu_en);
    }
}

/****************************************************************************
 * Name: gd32_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Input Parameters:
 *   cfgset - Bit-encoded description of the pin
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port or mode.
 *
 ****************************************************************************/

int gd32_gpio_config(uint32_t cfgset)
{
  uintptr_t regaddr;
  uint32_t port_base;
  uint32_t regval;
  uint32_t setting;
  uint32_t port;
  uint32_t pin;
  uint32_t pinmode;
  irqstate_t flags;

  /* Interrupts must be disabled from here on out so that we have mutually
   * exclusive access to all of the GPIO configuration registers.
   */

  flags = enter_critical_section();

  /* Verify that this hardware supports the selected GPIO port */

  port = (cfgset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;
  if (port >= GD32VW55X_NGPIO_PORTS)
    {
      leave_critical_section(flags);
      return -EINVAL;
    }

  /* Get the port base address */

  port_base = g_gpio_base[port];

  /* Enable the GPIO port clock */

  gd32_gpio_clock_enable(port_base);

  /* Get the pin number */

  pin = (cfgset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

  /* Set up the pin mode */

  switch (cfgset & GPIO_CFG_MODE_MASK)
    {
      /* Input mode */

      case GPIO_CFG_MODE_INPUT:
        pinmode = GPIO_MODE_INPUT;
        break;

      /* General purpose output mode */

      case GPIO_CFG_MODE_OUTPUT:

        /* Set the initial output value before enabling the output */

        gd32_gpio_output_set(port_base, pin,
                             (cfgset & GPIO_CFG_OUTPUT_SET) != 0);
        pinmode = GPIO_MODE_OUTPUT;
        break;

      /* Alternate function mode */

      case GPIO_CFG_MODE_AF:
        gd32_gpio_af_config(cfgset, port_base, pin);
        pinmode = GPIO_MODE_AF;
        break;

      /* Analog mode */

      case GPIO_CFG_MODE_ANALOG:
        pinmode = GPIO_MODE_ANALOG;
        break;

      default:
        leave_critical_section(flags);
        return -EINVAL;
    }

  /* Set push-pull/open-drain (only outputs and alternate function pins) */

  regaddr = GD32VW55X_GPIO_OMODE(port_base);
  regval  = getreg32(regaddr);

  if ((pinmode == GPIO_MODE_OUTPUT || pinmode == GPIO_MODE_AF) &&
      (cfgset & GPIO_CFG_ODPP_MASK) == GPIO_CFG_OD)
    {
      regval |= GPIO_OTYPE_OD(pin);
    }
  else
    {
      regval &= ~GPIO_OTYPE_OD(pin);
    }

  putreg32(regval, regaddr);

  /* Set the output speed (only outputs and alternate function pins) */

  setting = 0;
  if (pinmode == GPIO_MODE_OUTPUT || pinmode == GPIO_MODE_AF)
    {
      switch (cfgset & GPIO_CFG_SPEED_MASK)
        {
          default:

          /* 2 MHz low speed output */

          case GPIO_CFG_SPEED_2MHZ:
            setting = GPIO_OSPEED_2MHZ;
            break;

          /* 10 MHz medium speed output */

          case GPIO_CFG_SPEED_10MHZ:
            setting = GPIO_OSPEED_10MHZ;
            break;

          /* 25 MHz fast speed output */

          case GPIO_CFG_SPEED_25MHZ:
            setting = GPIO_OSPEED_25MHZ;
            break;

          /* Maximum speed output */

          case GPIO_CFG_SPEED_MAX:
            setting = GPIO_OSPEED_MAX;
            break;
        }
    }

  regaddr = GD32VW55X_GPIO_OSPD(port_base);
  regval  = getreg32(regaddr);
  regval &= ~GPIO_OSPEED_MASK(pin);
  regval |= (setting << GPIO_OSPEED_SHIFT(pin));
  putreg32(regval, regaddr);

  /* Set the pull-up/pull-down configuration */

  switch (cfgset & GPIO_CFG_PUPD_MASK)
    {
      default:

      /* No pull-up, no pull-down */

      case GPIO_CFG_PUPD_NONE:
        setting = GPIO_PUPD_NONE;
        break;

      /* With pull-up resistor */

      case GPIO_CFG_PUPD_PULLUP:
        setting = GPIO_PUPD_PULLUP;
        break;

      /* With pull-down resistor */

      case GPIO_CFG_PUPD_PULLDOWN:
        setting = GPIO_PUPD_PULLDOWN;
        break;
    }

  regaddr = GD32VW55X_GPIO_PUD(port_base);
  regval  = getreg32(regaddr);
  regval &= ~GPIO_PUPD_MASK(pin);
  regval |= (setting << GPIO_PUPD_SHIFT(pin));
  putreg32(regval, regaddr);

  /* Finally, set the pin mode.  This is done last so that the pin is only
   * driven once its output type, speed and pull configuration are in
   * place.
   */

  regaddr = GD32VW55X_GPIO_CTL(port_base);
  regval  = getreg32(regaddr);
  regval &= ~GPIO_MODE_MASK(pin);
  regval |= (pinmode << GPIO_MODE_SHIFT(pin));
  putreg32(regval, regaddr);

  /* Reset the alternate function selection if the pin is not an alternate
   * function pin.
   */

  if (pinmode != GPIO_MODE_AF)
    {
      gd32_gpio_af_unconfig(port_base, pin);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: gd32_gpio_unconfig
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default HiZ state.
 *
 * Input Parameters:
 *   cfgset - Bit-encoded description of the pin
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port or mode.
 *
 ****************************************************************************/

int gd32_gpio_unconfig(uint32_t cfgset)
{
  /* Keep only the port and pin number */

  cfgset &= GPIO_CFG_PORT_MASK | GPIO_CFG_PIN_MASK;

  /* Set the GPIO pin into floating input mode */

  cfgset |= GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE;

  return gd32_gpio_config(cfgset);
}

/****************************************************************************
 * Name: gd32_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Input Parameters:
 *   pinset - Bit-encoded description of the pin
 *   value  - The value to write to the pin
 *
 ****************************************************************************/

void gd32_gpio_write(uint32_t pinset, bool value)
{
  irqstate_t flags;
  uint32_t port_base;
  uint32_t port;
  uint32_t pin;

  port = (pinset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;
  if (port >= GD32VW55X_NGPIO_PORTS)
    {
      return;
    }

  flags = enter_critical_section();

  /* Get the port base address and the pin number */

  port_base = g_gpio_base[port];
  pin       = (pinset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

  /* Set or clear the output of the pin */

  gd32_gpio_output_set(port_base, pin, value);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: gd32_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Input Parameters:
 *   pinset - Bit-encoded description of the pin
 *
 * Returned Value:
 *   The input state of the pin
 *
 ****************************************************************************/

bool gd32_gpio_read(uint32_t pinset)
{
  irqstate_t flags;
  uint32_t port_base;
  uint32_t port;
  uint32_t pin;
  bool value;

  port = (pinset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;
  if (port >= GD32VW55X_NGPIO_PORTS)
    {
      return false;
    }

  flags = enter_critical_section();

  /* Get the port base address and the pin number */

  port_base = g_gpio_base[port];
  pin       = (pinset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

  value = gd32_gpio_input_get(port_base, pin);

  leave_critical_section(flags);

  return value;
}

/****************************************************************************
 * Name: gd32_dump_gpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the
 *   provided pinset.
 *
 * Input Parameters:
 *   pinset - Bit-encoded description of the pin
 *   msg    - A message to print with the register dump
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on an invalid port.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int gd32_dump_gpio(uint32_t pinset, const char *msg)
{
  irqstate_t flags;
  uint32_t port_base;
  uint32_t port;
  uint32_t pin;

  /* Verify that this hardware supports the selected GPIO port */

  port = (pinset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;
  if (port >= GD32VW55X_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  port_base = g_gpio_base[port];
  pin       = (pinset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

  flags = enter_critical_section();

  gpioinfo("GPIO%c pin%" PRIu32 " (base 0x%08" PRIx32 "): %s\n",
           (int)('A' + port), pin, port_base, msg);
  gpioinfo("  CTL: 0x%08" PRIx32 "  OMODE: 0x%08" PRIx32
           "  OSPD: 0x%08" PRIx32 "  PUD: 0x%08" PRIx32 "\n",
           getreg32(GD32VW55X_GPIO_CTL(port_base)),
           getreg32(GD32VW55X_GPIO_OMODE(port_base)),
           getreg32(GD32VW55X_GPIO_OSPD(port_base)),
           getreg32(GD32VW55X_GPIO_PUD(port_base)));
  gpioinfo("  ISTAT: 0x%08" PRIx32 "  OCTL: 0x%08" PRIx32
           "  LOCK: 0x%08" PRIx32 "\n",
           getreg32(GD32VW55X_GPIO_ISTAT(port_base)),
           getreg32(GD32VW55X_GPIO_OCTL(port_base)),
           getreg32(GD32VW55X_GPIO_LOCK(port_base)));
  gpioinfo("  AFSEL0: 0x%08" PRIx32 "  AFSEL1: 0x%08" PRIx32 "\n",
           getreg32(GD32VW55X_GPIO_AFSEL0(port_base)),
           getreg32(GD32VW55X_GPIO_AFSEL1(port_base)));

  leave_critical_section(flags);
  return OK;
}
#endif
