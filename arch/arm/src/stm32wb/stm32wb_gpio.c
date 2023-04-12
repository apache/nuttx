/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_gpio.c
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

#include "arm_internal.h"
#include "chip.h"
#include "stm32wb_gpio.h"

#include "hardware/stm32wb_syscfg.h"

#if defined(CONFIG_STM32WB_USE_LEGACY_PINMAP)
#  pragma message "CONFIG_STM32WB_USE_LEGACY_PINMAP will be deprecated migrate board.h see tools/stm32_pinmap_tool.py"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Base addresses for each GPIO block */

const uint32_t g_gpiobase[STM32WB_NPORTS] =
{
  STM32WB_GPIOA_BASE,
  STM32WB_GPIOB_BASE,
  STM32WB_GPIOC_BASE,
  STM32WB_GPIOD_BASE,
  STM32WB_GPIOE_BASE,
  STM32WB_GPIOH_BASE
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  stm32wb_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions.
 *
 *   Typically called from stm32wb_start().
 *
 * Assumptions:
 *   This function is called early in the initialization sequence so that
 *   no mutual exclusion is necessary.
 *
 ****************************************************************************/

void stm32wb_gpioinit(void)
{
}

/****************************************************************************
 * Name: stm32wb_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with stm32wb_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port, or when pin is locked as ALT
 *   function.
 *
 * To-Do: Auto Power Enable
 ****************************************************************************/

int stm32wb_configgpio(uint32_t cfgset)
{
  uintptr_t base;
  uint32_t regval;
  uint32_t setting;
  unsigned int port;
  unsigned int pin;
  unsigned int pinmode;
  irqstate_t flags;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= STM32WB_NPORTS)
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
        pinmode = GPIO_MODER_INPUT;
        break;

      case GPIO_OUTPUT:     /* General purpose output mode */

        /* Set the initial output value */

        stm32wb_gpiowrite(cfgset, (cfgset & GPIO_OUTPUT_SET) != 0);
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

  flags = enter_critical_section();

  /* Now apply the configuration to the mode register */

  regval  = getreg32(base + STM32WB_GPIO_MODER_OFFSET);
  regval &= ~GPIO_MODER_MASK(pin);
  regval |= ((uint32_t)pinmode << GPIO_MODER_SHIFT(pin));
  putreg32(regval, base + STM32WB_GPIO_MODER_OFFSET);

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

  regval  = getreg32(base + STM32WB_GPIO_PUPDR_OFFSET);
  regval &= ~GPIO_PUPDR_MASK(pin);
  regval |= (setting << GPIO_PUPDR_SHIFT(pin));
  putreg32(regval, base + STM32WB_GPIO_PUPDR_OFFSET);

  /* Set the alternate function (Only alternate function pins) */

  if (pinmode == GPIO_MODER_ALT)
    {
      setting = (cfgset & GPIO_AF_MASK) >> GPIO_AF_SHIFT;
    }
  else
    {
      setting = 0;
    }

  if (pin < 8)
    {
      regval  = getreg32(base + STM32WB_GPIO_AFRL_OFFSET);
      regval &= ~GPIO_AFRL_AFSEL_MASK(pin);
      regval |= (setting << GPIO_AFRL_AFSEL_SHIFT(pin));
      putreg32(regval, base + STM32WB_GPIO_AFRL_OFFSET);
    }
  else
    {
      regval  = getreg32(base + STM32WB_GPIO_AFRH_OFFSET);
      regval &= ~GPIO_AFRH_AFSEL_MASK(pin);
      regval |= (setting << GPIO_AFRH_AFSEL_SHIFT(pin));
      putreg32(regval, base + STM32WB_GPIO_AFRH_OFFSET);
    }

  /* Set speed (Only outputs and alternate function pins) */

  if (pinmode == GPIO_MODER_OUTPUT || pinmode == GPIO_MODER_ALT)
    {
      switch (cfgset & GPIO_SPEED_MASK)
        {
          default:
          case GPIO_SPEED_5MHz:    /* 5 MHz Low speed output */
            setting = GPIO_OSPEED_5MHz;
            break;

          case GPIO_SPEED_25MHz:   /* 25 MHz Medium speed output */
            setting = GPIO_OSPEED_25MHz;
            break;

          case GPIO_SPEED_50MHz:   /* 50 MHz Fast speed output  */
            setting = GPIO_OSPEED_50MHz;
            break;

          case GPIO_SPEED_120MHz:  /* 120 MHz High speed output */
            setting = GPIO_OSPEED_120MHz;
            break;
        }
    }
  else
    {
      setting = 0;
    }

  regval  = getreg32(base + STM32WB_GPIO_OSPEED_OFFSET);
  regval &= ~GPIO_OSPEED_MASK(pin);
  regval |= (setting << GPIO_OSPEED_SHIFT(pin));
  putreg32(regval, base + STM32WB_GPIO_OSPEED_OFFSET);

  /* Set push-pull/open-drain (Only outputs and alternate function pins) */

  regval  = getreg32(base + STM32WB_GPIO_OTYPER_OFFSET);
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

  putreg32(regval, base + STM32WB_GPIO_OTYPER_OFFSET);

  /* Otherwise, it is an input pin.  Should it configured as an
   * EXTI interrupt?
   */

  if (pinmode != GPIO_MODER_OUTPUT && (cfgset & GPIO_EXTI) != 0)
    {
      /* The selection of the EXTI line source is performed through the EXTIx
       * bits in the SYSCFG_EXTICRx registers.
       */

      uint32_t regaddr;
      int shift;

      /* Set the bits in the SYSCFG EXTICR register */

      regaddr = STM32WB_SYSCFG_EXTICR(pin);
      regval  = getreg32(regaddr);
      shift   = SYSCFG_EXTICR_EXTI_SHIFT(pin);
      regval &= ~(SYSCFG_EXTICR_PORT_MASK << shift);
      regval |= (((uint32_t)port) << shift);

      putreg32(regval, regaddr);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32wb_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default HiZ state (and possibly mark it's unused) and unlock it
 *   whether it was previsouly selected as alternative function
 *   (GPIO_ALT|GPIO_CNF_AFPP|...).
 *
 *   This is a safety function and prevents hardware from schocks, as
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

int stm32wb_unconfiggpio(uint32_t cfgset)
{
  /* Reuse port and pin number and set it to default HiZ INPUT */

  cfgset &= GPIO_PORT_MASK | GPIO_PIN_MASK;
  cfgset |= GPIO_INPUT | GPIO_FLOAT;

  /* To-Do: Mark its unuse for automatic power saving options */

  return stm32wb_configgpio(cfgset);
}

/****************************************************************************
 * Name: stm32wb_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void stm32wb_gpiowrite(uint32_t pinset, bool value)
{
  uint32_t base;
  uint32_t bit;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < STM32WB_NPORTS)
    {
      /* Get the port base address */

      base = g_gpiobase[port];

      /* Get the pin number  */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* Set or clear the output on the pin */

      if (value)
        {
          bit = GPIO_BSRR_SET(pin);
        }
      else
        {
          bit = GPIO_BSRR_RESET(pin);
        }

      putreg32(bit, base + STM32WB_GPIO_BSRR_OFFSET);
    }
}

/****************************************************************************
 * Name: stm32wb_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool stm32wb_gpioread(uint32_t pinset)
{
  uint32_t base;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < STM32WB_NPORTS)
    {
      /* Get the port base address */

      base = g_gpiobase[port];

      /* Get the pin number and return the input state of that pin */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      return ((getreg32(base + STM32WB_GPIO_IDR_OFFSET) & (1 << pin)) != 0);
    }

  return 0;
}
