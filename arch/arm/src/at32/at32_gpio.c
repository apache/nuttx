/****************************************************************************
 * arch/arm/src/at32/at32_gpio.c
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

#include "arm_internal.h"
#include "chip.h"
#include "at32_syscfg.h"
#include "at32_gpio.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Base addresses for each GPIO block */

const uint32_t g_gpiobase[AT32_NGPIO_PORTS] =
{
#if AT32_NGPIO_PORTS > 0
  AT32_GPIOA_BASE,
#endif
#if AT32_NGPIO_PORTS > 1
  AT32_GPIOB_BASE,
#endif
#if AT32_NGPIO_PORTS > 2
  AT32_GPIOC_BASE,
#endif
#if AT32_NGPIO_PORTS > 3
  AT32_GPIOD_BASE,
#endif
#if AT32_NGPIO_PORTS > 4
  AT32_GPIOE_BASE,
#endif
#if AT32_NGPIO_PORTS > 5
  AT32_GPIOF_BASE,
#endif
#if AT32_NGPIO_PORTS > 6
  AT32_GPIOG_BASE,
#endif
#if AT32_NGPIO_PORTS > 7
  AT32_GPIOH_BASE,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  at32_gpioremap
 *
 * Description:
 *
 *   Based on configuration within the .config file, this function will
 *   remaps positions of alternative functions.
 *
 ****************************************************************************/

static inline void at32_gpioremap(void)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  at32_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions.
 *
 *   Typically called from at32_start().
 *
 * Assumptions:
 *   This function is called early in the initialization sequence so that
 *   no mutual exclusion is necessary.
 *
 ****************************************************************************/

void at32_gpioinit(void)
{
  /* Remap according to the configuration within .config file */

  at32_gpioremap();
}

/****************************************************************************
 * Name: at32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with at32_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port, or when pin is locked as ALT
 *   function.
 *
 * To-Do: Auto Power Enable
 ****************************************************************************/

/****************************************************************************
 * Name: at32_configgpio (for the AT32F43xxx families).
 ****************************************************************************/

#if defined(CONFIG_AT32_AT32F43XX)
int at32_configgpio(uint32_t cfgset)
{
  uintptr_t base;
  uint32_t regval;
  uint32_t setting;
  uint32_t alt_setting;
  unsigned int regoffset;
  unsigned int port;
  unsigned int pin;
  unsigned int pos;
  unsigned int pinmode;
  irqstate_t flags;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= AT32_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  /* Get the port base address */

  base = g_gpiobase[port];

  /* Get the pin number and select the port configuration register for that
   * pin
   */

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Set up the mode register (and remember whether the pin mode) */

  switch (cfgset & GPIO_MODE_MASK)
    {
      default:
      case GPIO_INPUT:      /* Input mode */
        pinmode = GPIO_CFGR_INPUT;
        break;

      case GPIO_OUTPUT:     /* General purpose output mode */

        /* Set the initial output value */

        at32_gpiowrite(cfgset, (cfgset & GPIO_OUTPUT_SET) != 0);
        pinmode = GPIO_CFGR_OUTPUT;
        break;

      case GPIO_ALT:        /* Alternate function mode */
        pinmode = GPIO_CFGR_AF;
        break;

      case GPIO_ANALOG:     /* Analog mode */
        pinmode = GPIO_CFGR_ANALOG;
        break;
    }

  /* Interrupts must be disabled from here on out so that we have mutually
   * exclusive access to all of the GPIO configuration registers.
   */

  flags = enter_critical_section();

  /* Determine the alternate function (Only alternate function pins) */

  if (pinmode == GPIO_CFGR_AF)
    {
      alt_setting = (cfgset & GPIO_AF_MASK) >> GPIO_AF_SHIFT;
    }
  else
    {
      alt_setting = 0;
    }

  /* Set the alternate function (Only alternate function pins)
   * This is done before configuring the Outputs on a change to
   * an Alternate function.
   */

  if (alt_setting != 0)
    {
      if (pin < 8)
        {
          regoffset = AT32_GPIO_MUXL_OFFSET;
          pos       = pin;
        }
      else
        {
          regoffset = AT32_GPIO_MUXH_OFFSET;
          pos       = pin - 8;
        }

      regval  = getreg32(base + regoffset);
      regval &= ~GPIO_MUX_MASK(pos);
      regval |= (alt_setting << GPIO_MUX_SHIFT(pos));
      putreg32(regval, base + regoffset);
    }

  /* Now apply the configuration to the mode register */

  regval  = getreg32(base + AT32_GPIO_CFGR_OFFSET);
  regval &= ~GPIO_CFGR_MASK(pin);
  regval |= ((uint32_t)pinmode << GPIO_CFGR_SHIFT(pin));
  putreg32(regval, base + AT32_GPIO_CFGR_OFFSET);

  /* Set up the pull-up/pull-down configuration (all but analog pins) */

  setting = GPIO_PULL_NONE;
  if (pinmode != GPIO_CFGR_ANALOG)
    {
      switch (cfgset & GPIO_PUPD_MASK)
        {
          default:
          case GPIO_FLOAT:      /* No pull-up, pull-down */
            break;

          case GPIO_PULLUP:     /* Pull-up */
            setting = GPIO_PULL_PULLUP;
            break;

          case GPIO_PULLDOWN:   /* Pull-down */
            setting = GPIO_PULL_PULLDOWN;
            break;
        }
    }

  regval  = getreg32(base + AT32_GPIO_PULL_OFFSET);
  regval &= ~GPIO_PULL_MASK(pin);
  regval |= (setting << GPIO_PULL_SHIFT(pin));
  putreg32(regval, base + AT32_GPIO_PULL_OFFSET);

  /* Set the alternate function (Only alternate function pins)
   * This is done after configuring the the pin's connection
   * on a change away from an Alternate function.
   */

  if (alt_setting == 0)
      {
        if (pin < 8)
          {
            regoffset = AT32_GPIO_MUXL_OFFSET;
            pos       = pin;
          }
        else
          {
            regoffset = AT32_GPIO_MUXH_OFFSET;
            pos       = pin - 8;
          }

        regval  = getreg32(base + regoffset);
        regval &= ~GPIO_MUX_MASK(pos);
        regval |= (alt_setting << GPIO_MUX_SHIFT(pos));
        putreg32(regval, base + regoffset);
      }

  /* Set drive capability */

  if (pinmode == GPIO_CFGR_OUTPUT || pinmode == GPIO_CFGR_AF)
  {
    switch (cfgset & GPIO_DRV_MASK)
    {
      case GPIO_DRV_STRONG:
        setting = GPIO_ODRVR_STRONG;
        break;

      case GPIO_DRV_MODETATE:
        setting = GPIO_ODRVR_MODERATE;
        break;
    }
  }
  else
  {
    setting = 0;
  }

  regval  = getreg32(base + AT32_GPIO_ODRVR_OFFSET);
  regval &= ~GPIO_ODRVR_MASK(pin);
  regval |= (setting << GPIO_ODRVR_SHIFT(pin));
  putreg32(regval, base + AT32_GPIO_ODRVR_OFFSET);

  /* Set push-pull/open-drain (Only outputs and alternate function pins) */

  regval  = getreg32(base + AT32_GPIO_OMODER_OFFSET);
  setting = GPIO_OMODER_OD(pin);

  if ((pinmode == GPIO_CFGR_OUTPUT || pinmode == GPIO_CFGR_AF) &&
      (cfgset & GPIO_OPENDRAIN) != 0)
    {
      regval |= setting;
    }
  else
    {
      regval &= ~setting;
    }

  putreg32(regval, base + AT32_GPIO_OMODER_OFFSET);

  /* Otherwise, it is an input pin.  Should it configured as an EXTI
   * interrupt?
   */

  if (pinmode != GPIO_CFGR_OUTPUT && (cfgset & GPIO_EXTI) != 0)
    {
      uint32_t regaddr;
      int shift;

      /* Set the bits in the SYSCFG EXTICR register */

      regaddr = AT32_SCFG_EXTICR(pin);
      regval  = getreg32(regaddr);
      shift   = SCFG_EXINTCR_EXTI_SHIFT(pin);
      regval &= ~(SCFG_EXINTCR_PORT_MASK << shift);
      regval |= (((uint32_t)port) << shift);

      putreg32(regval, regaddr);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: at32_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default HiZ state (and possibly mark it's unused) and unlock it
 *   whether it was previously selected as an alternative function
 *   (GPIO_ALT | GPIO_CNF_AFPP | ...).
 *
 *   This is a safety function and prevents hardware from shocks, as
 *   unexpected write to the Timer Channel Output GPIO to fixed '1' or '0'
 *   while it should operate in PWM mode could produce excessive on-board
 *   currents and trigger over-current/alarm function.
 *
 * Returned Value:
 *  OK on success
 *  A negated errno value on invalid port
 *
 * To-Do: Auto Power Disable
 ****************************************************************************/

int at32_unconfiggpio(uint32_t cfgset)
{
  /* Reuse port and pin number and set it to default HiZ INPUT */

  cfgset &= GPIO_PORT_MASK | GPIO_PIN_MASK;

#if defined(CONFIG_AT32_AT32F43XX)
  cfgset |= GPIO_INPUT | GPIO_FLOAT;
#else
# error "Unsupported AT32 chip"
#endif

  /* To-Do: Mark its unuse for automatic power saving options */

  return at32_configgpio(cfgset);
}

/****************************************************************************
 * Name: at32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void at32_gpiowrite(uint32_t pinset, bool value)
{
  uint32_t base;

#if defined(CONFIG_AT32_AT32F43XX)
  uint32_t bit;
#endif
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < AT32_NGPIO_PORTS)
    {
      /* Get the port base address */

      base = g_gpiobase[port];

      /* Get the pin number  */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* Set or clear the output on the pin */

#if defined(CONFIG_AT32_AT32F43XX)

      if (value)
        {
          bit = GPIO_SCR_SET(pin);
        }
      else
        {
          bit = GPIO_SCR_RESET(pin);
        }

      putreg32(bit, base + AT32_GPIO_SCR_OFFSET);

#else
# error "Unsupported AT32 chip"
#endif
    }
}

/****************************************************************************
 * Name: at32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool at32_gpioread(uint32_t pinset)
{
  uint32_t base;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < AT32_NGPIO_PORTS)
    {
      /* Get the port base address */

      base = g_gpiobase[port];

      /* Get the pin number and return the input state of that pin */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      return ((getreg32(base + AT32_GPIO_IDT_OFFSET) & (1 << pin)) != 0);
    }

  return 0;
}

/****************************************************************************
 * Name: at32_iocompensation
 *
 * Description:
 *   Enable I/O compensation.
 *
 *   By default the I/O compensation cell is not used. However when the I/O
 *   output buffer speed is configured in 50 MHz or 100 MHz mode, it is
 *   recommended to use the compensation cell for slew rate control on I/O
 *   tf(IO)out)/tr(IO)out commutation to reduce the I/O noise on power
 *   supply.
 *
 *   The I/O compensation cell can be used only when the supply voltage
 *   ranges from 2.4 to 3.6 V.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_HAVE_IOCOMPENSATION
void at32_iocompensation(void)
{
}
#endif
