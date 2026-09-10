/****************************************************************************
 * arch/arm/src/n32h7/n32_gpio.c
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
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <arch/n32h7/chip.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "hardware/n32h7_afio.h"
#include "n32_gpio.h"

#if defined(CONFIG_N32H7_N32H76X)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_configgpio_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Base addresses for each GPIO block */

const uint32_t g_gpiobase[N32H7_NGPIO] =
{
  N32_GPIOA_BASE,
  N32_GPIOB_BASE,
  N32_GPIOC_BASE,
  N32_GPIOD_BASE,
#if N32H7_NGPIO > 4
  N32_GPIOE_BASE,
#endif
#if N32H7_NGPIO > 5
  N32_GPIOF_BASE,
#endif
#if N32H7_NGPIO > 6
  N32_GPIOG_BASE,
#endif
#if N32H7_NGPIO > 7
  N32_GPIOH_BASE,
#endif
#if N32H7_NGPIO > 8
  N32_GPIOI_BASE,
#endif
#if N32H7_NGPIO > 9
  N32_GPIOJ_BASE,
#endif
#if N32H7_NGPIO > 10
  N32_GPIOK_BASE,
#endif
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  n32_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions.
 *
 *   Typically called from n32_start().
 *
 * Assumptions:
 *   This function is called early in the initialization sequence so that
 *   no mutual exclusion is necessary.
 *
 ****************************************************************************/

void n32_gpioinit(void)
{
}

/****************************************************************************
 * Name: n32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with n32_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port, or when pin is locked as ALT
 *   function.
 *
 * To-Do: Auto Power Enable
 ****************************************************************************/

int n32_configgpio(uint32_t cfgset)
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
  if (port >= N32H7_NGPIO)
    {
      return -EINVAL;
    }

  /* Get the port base address */

  base = g_gpiobase[port];
  if (base == 0)
    {
      return -EINVAL;
    }

  /* Get the pin number and select the port configuration register for that
   * pin
   */

  pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Set up the mode register (and remember whether the pin mode) */

  switch (cfgset & GPIO_MODE_MASK)
    {
      default:
      case GPIO_INPUT:      /* Input mode */
        pinmode = GPIO_MODER_INPUT;
        break;

      case GPIO_OUTPUT:     /* General purpose output mode */
        pinmode = GPIO_MODER_OUTPUT;
        break;

      case GPIO_ALT:        /* Alternate function mode */
        pinmode = GPIO_MODER_ALT;
        break;

      case GPIO_ANALOG:     /* Analog mode */
        pinmode = GPIO_MODER_ANALOG;
        break;
    }

  /* Interrupts must be disabled from here on out so that we have mutually
   * exclusive access to all of the GPIO configuration registers.
   */

  flags = spin_lock_irqsave(&g_configgpio_lock);

  /* Determine the alternate function (Only alternate function pins) */

  if (pinmode == GPIO_MODER_ALT)
    {
      setting = (cfgset & GPIO_AF_MASK) >> GPIO_AF_SHIFT;
    }
  else
    {
      setting = 0;
    }

  if (pinmode == GPIO_MODER_ALT)
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

  if (pin < 8)
    {
      regoffset = N32_GPIO_AFL_OFFSET;
      pos       = pin;
    }
  else
    {
      regoffset = N32_GPIO_AFH_OFFSET;
      pos       = pin - 8;
    }

  regval  = getreg32(base + regoffset);
  regval &= ~GPIO_AFR_MASK(pos);
  regval |= (alt_setting << GPIO_AFR_SHIFT(pos));
  putreg32(regval, base + regoffset);

  /* Now apply the configuration to the mode register */

  regval  = getreg32(base + N32_GPIO_MODER_OFFSET);
  regval &= ~GPIO_MODER_MASK(pin);
  regval |= ((uint32_t)pinmode << GPIO_MODER_SHIFT(pin));
  putreg32(regval, base + N32_GPIO_MODER_OFFSET);

  /* Set up the pull-up/pull-down configuration (all but analog pins) */

  setting = GPIO_PUPDR_NONE;
  if (pinmode != GPIO_MODER_ANALOG)
    {
      switch (cfgset & GPIO_PUPD_MASK)
        {
          default:
          case GPIO_FLOAT:      /* No pull-up, pull-down */
            break;

          case GPIO_PULLUP:     /* Pull-up */
            setting = GPIO_PUPDR_PULLUP;
            break;

          case GPIO_PULLDOWN:   /* Pull-down */
            setting = GPIO_PUPDR_PULLDOWN;
            break;
        }
    }

  regval  = getreg32(base + N32_GPIO_PUPD_OFFSET);
  regval &= ~GPIO_PUPDR_MASK(pin);
  regval |= (setting << GPIO_PUPDR_SHIFT(pin));
  putreg32(regval, base + N32_GPIO_PUPD_OFFSET);

  /* Set drive strength (Only outputs and alternate function pins) */

  if (pinmode == GPIO_MODER_OUTPUT || pinmode == GPIO_MODER_ALT)
    {
      switch (cfgset & GPIO_DRIVE_MASK)
        {
          default:
          case GPIO_DRIVE_2mA:    /* 2 MHz Low speed output */
            setting = GPIO_DS_2mA;
            break;

          case GPIO_DRIVE_4mA:    /* 4 mA */
            setting = GPIO_DS_4mA;
            break;

          case GPIO_DRIVE_8mA:    /* 8 mA */
            setting = GPIO_DS_8mA;
            break;

          case GPIO_DRIVE_12mA:   /* 12 mA */
            setting = GPIO_DS_12mA;
            break;
        }
    }
  else
    {
      setting = 0;
    }

  regval  = getreg32(base + N32_GPIO_DS_OFFSET);
  regval &= ~GPIO_DS_MASK(pin);
  regval |= (setting << GPIO_DS_SHIFT(pin));
  putreg32(regval, base + N32_GPIO_DS_OFFSET);

  /* Set slew rate (Only outputs and alternate function pins) */

  regval  = getreg32(base + N32_GPIO_SR_OFFSET);
  setting = GPIO_SR_SLOW_SLEW(pin);

  if ((pinmode == GPIO_MODER_OUTPUT || pinmode == GPIO_MODER_ALT) &&
    (cfgset & GPIO_SLEW_RATE_SLOW) != 0)
    {
      regval |= setting;
    }
  else
    {
      regval &= ~setting;
    }

  putreg32(regval, base + N32_GPIO_SR_OFFSET);

  /* Set push-pull/open-drain (Only outputs and alternate function pins) */

  regval  = getreg32(base + N32_GPIO_OTYPER_OFFSET);
  setting = GPIO_OTYPER_OD(pin);

  if ((pinmode == GPIO_MODER_OUTPUT || pinmode == GPIO_MODER_ALT) &&
      (cfgset & GPIO_OPENDRAIN) != 0)
    {
      regval |= setting;
    }
  else
    {
      regval &= ~setting;
    }

  putreg32(regval, base + N32_GPIO_OTYPER_OFFSET);

  spin_unlock_irqrestore(&g_configgpio_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: n32_unconfiggpio
 *
 * Description:
 * Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 * it into default HiZ state (and possibly mark it's unused) and unlock it
 * whether it was previously selected as alternative function
 * (GPIO_ALT|GPIO_CNF_AFPP|...).
 *
 * This is a safety function and prevents hardware from shocks, as
 * unexpected write to the Timer Channel Output GPIO to fixed '1' or '0'
 * while it should operate in PWM mode could produce excessive on-board
 * currents and trigger over-current/alarm function.
 *
 * Returned Value:
 *  OK on success
 *  A negated errno value on invalid port
 *
 * To-Do: Auto Power Disable
 ****************************************************************************/

int n32_unconfiggpio(uint32_t cfgset)
{
  /* Reuse port and pin number and set it to default HiZ INPUT */

  cfgset &= GPIO_PORT_MASK | GPIO_PIN_MASK;
  cfgset |= GPIO_INPUT | GPIO_FLOAT;

  /* To-Do: Mark its unuse for automatic power saving options */

  return n32_configgpio(cfgset);
}

/****************************************************************************
 * Name: n32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void n32_gpiowrite(uint32_t pinset, bool value)
{
  uint32_t base;
  uint32_t bit;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < N32H7_NGPIO)
    {
      /* Get the port base address */

      base = g_gpiobase[port];
      if (base != 0)
        {
          /* Get the pin number  */

          pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

          /* Set or clear the output on the pin */

          if (value)
            {
              bit = GPIO_PBSC_SET(pin);
            }
          else
            {
              bit = GPIO_PBSC_RESET(pin);
            }

          putreg32(bit, base + N32_GPIO_PBSC_OFFSET);
        }
    }
}

/****************************************************************************
 * Name: n32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool n32_gpioread(uint32_t pinset)
{
  uint32_t base;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < N32H7_NGPIO)
    {
      /* Get the port base address */

      base = g_gpiobase[port];
      if (base != 0)
        {
          /* Get the pin number and return the input state of that pin */

          pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
          return ((getreg32(base + N32_GPIO_PID_OFFSET) &
                  (1 << pin)) != 0);
        }
    }

  return 0;
}

#endif /* CONFIG_N32H7_N32H76X */
