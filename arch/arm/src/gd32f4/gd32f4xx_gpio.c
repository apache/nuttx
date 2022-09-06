/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_gpio.c
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

#include <nuttx/irq.h>

#include "chip.h"
#include "gd32f4xx_syscfg.h"
#include "gd32f4xx_gpio.h"
#include "gd32f4xx.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Base addresses for each GPIO block */

const uint32_t g_gpio_base[GD32_NGPIO_PORTS] =
{
#if GD32_NGPIO_PORTS > 0
  GD32_GPIOA,
#endif
#if GD32_NGPIO_PORTS > 1
  GD32_GPIOB,
#endif
#if GD32_NGPIO_PORTS > 2
  GD32_GPIOC,
#endif
#if GD32_NGPIO_PORTS > 3
  GD32_GPIOD,
#endif
#if GD32_NGPIO_PORTS > 4
  GD32_GPIOE,
#endif
#if GD32_NGPIO_PORTS > 5
  GD32_GPIOF,
#endif
#if GD32_NGPIO_PORTS > 6
  GD32_GPIOG,
#endif
#if GD32_NGPIO_PORTS > 7
  GD32_GPIOH,
#endif
#if GD32_NGPIO_PORTS > 8
  GD32_GPIOI,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  gd32_gpio_clock_enable
 *
 * Description:
 *
 *   Enable GPIO clock
 *
 ****************************************************************************/

static void gd32_gpio_clock_enable(uint32_t port_base)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  /* Determine which GPIO port to configure */

  switch (port_base)
    {
    default:
      return;
#if GD32_NGPIO_PORTS > 0
    case GD32_GPIOA:
      rcu_en = RCU_AHB1EN_PAEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 1
    case GD32_GPIOB:
      rcu_en = RCU_AHB1EN_PBEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 2
    case GD32_GPIOC:
      rcu_en = RCU_AHB1EN_PCEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 3
    case GD32_GPIOD:
      rcu_en = RCU_AHB1EN_PDEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 4
    case GD32_GPIOE:
      rcu_en = RCU_AHB1EN_PEEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 5
    case GD32_GPIOF:
      rcu_en = RCU_AHB1EN_PFEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 6
    case GD32_GPIOG:
      rcu_en = RCU_AHB1EN_PGEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 7
    case GD32_GPIOH:
      rcu_en = RCU_AHB1EN_PHEN;
      break;
#endif
#if GD32_NGPIO_PORTS > 8
    case GD32_GPIOI:
      rcu_en = RCU_AHB1EN_PIEN;
      break;
#endif
    }

  regaddr = GD32_RCU_AHB1EN;

  /* Check clock if alreay enable. */

  if (rcu_en != (rcu_en & getreg32(regaddr)))
    {
      /* Enable/disable AHB clock for GPIO */

      modifyreg32(regaddr, 0, rcu_en);
    }
}

/****************************************************************************
 * Function:  gd32_gpio_output_set
 *
 * Description:
 *
 *   Set GPIO output
 *
 ****************************************************************************/

static void gd32_gpio_output_set(uint32_t port_base, uint32_t pin,
                                 bool value)
{
  uintptr_t regaddr;
  uint32_t regval;

  if (value)
    {
      regval = GPIO_BOP_SET(pin);
      regaddr = GD32_GPIO_BOP(port_base);
    }
  else
    {
      regval = GPIO_BC_SET(pin);
      regaddr = GD32_GPIO_BC(port_base);
    }

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Function:  gd32_gpio_input_get
 *
 * Description:
 *
 *   Get GPIO input status
 *
 ****************************************************************************/

static inline bool gd32_gpio_input_get(uint32_t port_base, uint32_t pin)
{
  uintptr_t regaddr;
  uint32_t regval;

  regaddr = GD32_GPIO_ISTAT(port_base);
  regval = getreg32(regaddr);

  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: gd32_gpio_af_config
 *
 * Description:
 *   Configure a GPIO pin as the Alternative function.
 ****************************************************************************/

static void gd32_gpio_af_config(uint32_t cfgset, uint32_t port_base,
                                uint32_t pin)
{
  uint32_t af_sel;
  uint32_t regaddr;
  uint32_t regval;
  uint32_t pos;

  af_sel = (cfgset & GPIO_CFG_AF_MASK) >> GPIO_CFG_AF_SHIFT;
  if (pin < 8)
    {
      regaddr = GD32_GPIO_AFSEL0(port_base);
      pos     = pin;
    }
  else
    {
      regaddr = GD32_GPIO_AFSEL1(port_base);
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
 *   Configure a GPIO pin's Alternative function as reset value.
 ****************************************************************************/

static void gd32_gpio_af_unconfig(uint32_t cfgset, uint32_t port_base,
                                  uint32_t pin)
{
  uint32_t regaddr;
  uint32_t regval;
  uint32_t pos;

  if (pin < 8)
    {
      regaddr = GD32_GPIO_AFSEL0(port_base);
      pos     = pin;
    }
  else
    {
      regaddr = GD32_GPIO_AFSEL1(port_base);
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
 * Name: gd32_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Return value:
 *   OK on success
 *   A negated errno value on invalid port or mode.
 *
 ****************************************************************************/

int gd32_gpio_config(uint32_t cfgset)
{
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

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;
  if (port >= GD32_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  /* Get the port base address */

  port_base = g_gpio_base[port];

  /* Eable the GPIO port clock */

  gd32_gpio_clock_enable(port_base);

  /* Get the pin number and select the port configuration register for that
   * pin
   */

  pin = (cfgset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

  /* Set up the mode register (and remember whether the pin mode) */

  switch (cfgset & GPIO_CFG_MODE_MASK)
    {
      /* Input mode */

      case GPIO_CFG_MODE_INPUT:
        pinmode = GPIO_MODE_INPUT;
        break;

      /* General purpose output mode */

      case GPIO_CFG_MODE_OUTPUT:

        /* Set the initial output value */

        gd32_gpio_write(cfgset, (cfgset & GPIO_CFG_OUTPUT_SET) != 0);
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
      return -EINVAL;
        break;
    }

  regval  = getreg32(GD32_GPIO_CTL(port_base));
  regval &= ~GPIO_MODE_MASK(pin);
  regval |= ((uint32_t)pinmode << GPIO_MODE_SHIFT(pin));
  putreg32(regval, GD32_GPIO_CTL(port_base));

  /* Unconfigure alternate function */

  if (pinmode != GPIO_MODE_AF)
    {
      gd32_gpio_af_unconfig(cfgset, port_base, pin);
    }

  /* Set up the mode register (and remember whether the pin mode) */

  switch (cfgset & GPIO_CFG_PUPD_MASK)
    {
      default:

      /* No pull-up, pull-down */

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

  regval  = getreg32(GD32_GPIO_PUD(port_base));
  regval &= ~GPIO_PUPD_MASK(pin);
  regval |= ((uint32_t)setting << GPIO_MODE_SHIFT(pin));
  putreg32(regval, GD32_GPIO_PUD(port_base));

  /* Set push-pull/open-drain (Only outputs and alternate function pins) */

  setting = GPIO_OTYPE_OD(pin);
  regval  = getreg32(GD32_GPIO_OMODE(port_base));

  if ((pinmode == GPIO_MODE_OUTPUT || pinmode == GPIO_MODE_AF) &&
      (cfgset & GPIO_CFG_ODPP_MASK) == GPIO_CFG_OD)
    {
      regval |= setting;
    }
  else
    {
      regval &= ~setting;
    }

  putreg32(regval, GD32_GPIO_OMODE(port_base));

  /* Set speed (Only outputs and alternate function pins) */

  setting = 0;
  if (pinmode == GPIO_MODE_OUTPUT || pinmode == GPIO_MODE_AF)
    {
      switch (cfgset & GPIO_CFG_SPEED_MASK)
        {
          default:

          /* 2 MHz Low speed output */

          case GPIO_CFG_SPEED_2MHZ:
            setting = GPIO_OSPEED_2MHZ;
            break;

          /* 25 MHz Medium speed output */

          case GPIO_CFG_SPEED_25MHZ:
            setting = GPIO_OSPEED_25MHZ;
            break;

          /* 50 MHz Fast speed output  */

          case GPIO_CFG_SPEED_50MHZ:
            setting = GPIO_OSPEED_50MHZ;
            break;

          /* 200 MHz High speed output */

          case GPIO_CFG_SPEED_200MHZ:
            setting = GPIO_OSPEED_200MHZ;
            break;
        }
    }

  regval  = getreg32(GD32_GPIO_OSPD(port_base));
  regval &= ~GPIO_OSPEED_MASK(pin);
  regval |= (setting << GPIO_OSPEED_SHIFT(pin));
  putreg32(regval, GD32_GPIO_OSPD(port_base));

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
 ****************************************************************************/

int gd32_gpio_unconfig(uint32_t cfgset)
{
  /* Reuse port and pin number */

  cfgset &= GPIO_CFG_PORT_MASK | GPIO_CFG_PIN_MASK;

  /*  Set GPIO in floating input mode */

  cfgset |= GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE;

  /* To-Do: Mark its unuse for automatic power saving options */

  return gd32_gpio_config(cfgset);
}

/****************************************************************************
 * Name: gd32_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void gd32_gpio_write(uint32_t pinset, bool value)
{
  irqstate_t flags;
  uint32_t port_base;
  uint32_t port;
  uint32_t pin;

  flags = enter_critical_section();

  port = (pinset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;

  /* Get the port base address */

  port_base = g_gpio_base[port];

  /* Get the pin number  */

  pin = (pinset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

  /* Set or clear the output on the pin */

  gd32_gpio_output_set(port_base, pin, value);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: gd32_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool gd32_gpio_read(uint32_t pinset)
{
  irqstate_t flags;
  uint32_t port_base;
  uint32_t port;
  uint32_t pin;
  bool value;

  flags = enter_critical_section();

  port = (pinset & GPIO_CFG_PORT_MASK) >> GPIO_CFG_PORT_SHIFT;

  /* Get the port base address */

  port_base = g_gpio_base[port];

  /* Get the pin number and return the input state of that pin */

  pin = (pinset & GPIO_CFG_PIN_MASK) >> GPIO_CFG_PIN_SHIFT;

  value = gd32_gpio_input_get(port_base, pin);

  leave_critical_section(flags);

  return value;
}
