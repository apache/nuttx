/****************************************************************************
 * arch/arm/src/common/stm32/stm32_gpio_v2.c
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
#include <nuttx/debug.h>

#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"

#if defined(CONFIG_STM32_HAVE_SYSCFG) && \
    !defined(CONFIG_STM32_HAVE_IP_EXTI_M33M55_V1)
#  include "hardware/stm32_syscfg.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_INDEX(p)  ((p) >> GPIO_PORT_SHIFT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_configgpio_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Base addresses for each GPIO block, indexed by the port number */

const uint32_t g_gpiobase[STM32_NPORTS] =
{
  [GPIO_INDEX(GPIO_PORTA)] = STM32_GPIOA_BASE,
  [GPIO_INDEX(GPIO_PORTB)] = STM32_GPIOB_BASE,
  [GPIO_INDEX(GPIO_PORTC)] = STM32_GPIOC_BASE,
#if defined(GPIO_PORTD) && defined(STM32_GPIOD_BASE) && \
    GPIO_INDEX(GPIO_PORTD) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTD)] = STM32_GPIOD_BASE,
#endif
#if defined(GPIO_PORTE) && defined(STM32_GPIOE_BASE) && \
    GPIO_INDEX(GPIO_PORTE) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTE)] = STM32_GPIOE_BASE,
#endif
#if defined(GPIO_PORTF) && defined(STM32_GPIOF_BASE) && \
    GPIO_INDEX(GPIO_PORTF) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTF)] = STM32_GPIOF_BASE,
#endif
#if defined(GPIO_PORTG) && defined(STM32_GPIOG_BASE) && \
    GPIO_INDEX(GPIO_PORTG) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTG)] = STM32_GPIOG_BASE,
#endif
#if defined(GPIO_PORTH) && defined(STM32_GPIOH_BASE) && \
    GPIO_INDEX(GPIO_PORTH) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTH)] = STM32_GPIOH_BASE,
#endif
#if defined(GPIO_PORTI) && defined(STM32_GPIOI_BASE) && \
    GPIO_INDEX(GPIO_PORTI) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTI)] = STM32_GPIOI_BASE,
#endif
#if defined(GPIO_PORTJ) && defined(STM32_GPIOJ_BASE) && \
    GPIO_INDEX(GPIO_PORTJ) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTJ)] = STM32_GPIOJ_BASE,
#endif
#if defined(GPIO_PORTK) && defined(STM32_GPIOK_BASE) && \
    GPIO_INDEX(GPIO_PORTK) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTK)] = STM32_GPIOK_BASE,
#endif
#if defined(GPIO_PORTL) && defined(STM32_GPIOL_BASE) && \
    GPIO_INDEX(GPIO_PORTL) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTL)] = STM32_GPIOL_BASE,
#endif
#if defined(GPIO_PORTM) && defined(STM32_GPIOM_BASE) && \
    GPIO_INDEX(GPIO_PORTM) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTM)] = STM32_GPIOM_BASE,
#endif
#if defined(GPIO_PORTN) && defined(STM32_GPION_BASE) && \
    GPIO_INDEX(GPIO_PORTN) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTN)] = STM32_GPION_BASE,
#endif
#if defined(GPIO_PORTO) && defined(STM32_GPIOO_BASE) && \
    GPIO_INDEX(GPIO_PORTO) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTO)] = STM32_GPIOO_BASE,
#endif
#if defined(GPIO_PORTP) && defined(STM32_GPIOP_BASE) && \
    GPIO_INDEX(GPIO_PORTP) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTP)] = STM32_GPIOP_BASE,
#endif
#if defined(GPIO_PORTQ) && defined(STM32_GPIOQ_BASE) && \
    GPIO_INDEX(GPIO_PORTQ) < STM32_NPORTS
  [GPIO_INDEX(GPIO_PORTQ)] = STM32_GPIOQ_BASE,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_gpio_setmode
 ****************************************************************************/

static void stm32_gpio_setmode(uintptr_t base, unsigned int pin,
                               unsigned int pinmode)
{
  uint32_t regval;

  regval  = getreg32(base + STM32_GPIO_MODER_OFFSET);
  regval &= ~GPIO_MODER_MASK(pin);
  regval |= ((uint32_t)pinmode << GPIO_MODER_SHIFT(pin));
  putreg32(regval, base + STM32_GPIO_MODER_OFFSET);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  stm32_gpioinit
 *
 * Description:
 *   Based on configuration within the .config file, it does:
 *    - Remaps positions of alternative functions.
 *
 *   Typically called from stm32_start().
 *
 * Assumptions:
 *   This function is called early in the initialization sequence so that
 *   no mutual exclusion is necessary.
 *
 ****************************************************************************/

void stm32_gpioinit(void)
{
}

/****************************************************************************
 * Name: stm32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with stm32_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port, or when pin is locked as ALT
 *   function.
 *
 * To-Do: Auto Power Enable
 ****************************************************************************/

int stm32_configgpio(uint32_t cfgset)
{
  uintptr_t base;
  uint32_t regval;
  uint32_t setting;
  unsigned int regoffset;
  unsigned int port;
  unsigned int pin;
  unsigned int pos;
  unsigned int pinmode;
  irqstate_t flags;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= STM32_NPORTS || g_gpiobase[port] == 0)
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

        stm32_gpiowrite(cfgset, (cfgset & GPIO_OUTPUT_SET) != 0);
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

  regval  = getreg32(base + STM32_GPIO_PUPDR_OFFSET);
  regval &= ~GPIO_PUPDR_MASK(pin);
  regval |= (setting << GPIO_PUPDR_SHIFT(pin));
  putreg32(regval, base + STM32_GPIO_PUPDR_OFFSET);

  /* Set speed (Only outputs and alternate function pins) */

  if (pinmode == GPIO_MODER_OUTPUT || pinmode == GPIO_MODER_ALT)
    {
      switch (cfgset & GPIO_SPEED_MASK)
        {
          default:
          case GPIO_SPEED_2MHz:    /* 2 MHz Low speed output */
            setting = GPIO_OSPEED_2MHz;
            break;

          case GPIO_SPEED_25MHz:   /* 25 MHz Medium speed output */
            setting = GPIO_OSPEED_25MHz;
            break;

          case GPIO_SPEED_50MHz:   /* 50 MHz High speed output  */
            setting = GPIO_OSPEED_50MHz;
            break;

          case GPIO_SPEED_100MHz:   /* 100 MHz Very High speed output */
            setting = GPIO_OSPEED_100MHz;
            break;
        }
    }
  else
    {
      setting = 0;
    }

  regval  = getreg32(base + STM32_GPIO_OSPEED_OFFSET);
  regval &= ~GPIO_OSPEED_MASK(pin);
  regval |= (setting << GPIO_OSPEED_SHIFT(pin));
  putreg32(regval, base + STM32_GPIO_OSPEED_OFFSET);

  /* Set push-pull/open-drain (Only outputs and alternate function pins) */

  regval  = getreg32(base + STM32_GPIO_OTYPER_OFFSET);
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

  putreg32(regval, base + STM32_GPIO_OTYPER_OFFSET);

#ifdef CONFIG_STM32_HAVE_GPIO_DELAY
  /* Set the I/O delay and the path that it applies to */

  setting = (cfgset & GPIO_DELAY_MASK) >> GPIO_DELAY_SHIFT;
  pos     = pin & 7;

  regoffset = pin < 8 ? STM32_GPIO_DELAYRL_OFFSET :
                        STM32_GPIO_DELAYRH_OFFSET;
  regval    = getreg32(base + regoffset);
  regval   &= ~GPIO_DELAYR_MASK(pos);
  regval   |= setting << GPIO_DELAYR_SHIFT(pos);
  putreg32(regval, base + regoffset);

  regoffset = pin < 8 ? STM32_GPIO_ADVCFGRL_OFFSET :
                        STM32_GPIO_ADVCFGRH_OFFSET;
  regval    = getreg32(base + regoffset);
  regval   &= ~GPIO_ADVCFGR_DLYPATH(pos);
  if ((cfgset & GPIO_DELAY_INPUT) != 0)
    {
      regval |= GPIO_ADVCFGR_DLYPATH(pos);
    }

  putreg32(regval, base + regoffset);
#endif

  /* Set the mode last.  The alternate function is selected before the pin
   * enters alternate function mode and cleared after it leaves it.
   */

  if (pinmode == GPIO_MODER_ALT)
    {
      setting = (cfgset & GPIO_AF_MASK) >> GPIO_AF_SHIFT;
    }
  else
    {
      setting = 0;
      stm32_gpio_setmode(base, pin, pinmode);
    }

  if (pin < 8)
    {
      regoffset = STM32_GPIO_AFRL_OFFSET;
      pos       = pin;
    }
  else
    {
      regoffset = STM32_GPIO_AFRH_OFFSET;
      pos       = pin - 8;
    }

  regval  = getreg32(base + regoffset);
  regval &= ~GPIO_AFR_MASK(pos);
  regval |= (setting << GPIO_AFR_SHIFT(pos));
  putreg32(regval, base + regoffset);

  if (pinmode == GPIO_MODER_ALT)
    {
      stm32_gpio_setmode(base, pin, pinmode);
    }

#ifdef STM32_SYSCFG_EXTICR
  /* Select the port of the EXTI line */

  if (pinmode != GPIO_MODER_OUTPUT && (cfgset & GPIO_EXTI) != 0)
    {
      uint32_t regaddr;
      int shift;

      regaddr = STM32_SYSCFG_EXTICR(pin);
      regval  = getreg32(regaddr);
      shift   = SYSCFG_EXTICR_EXTI_SHIFT(pin);
      regval &= ~(SYSCFG_EXTICR_PORT_MASK << shift);
      regval |= (((uint32_t)port) << shift);
      putreg32(regval, regaddr);
    }
#endif

#ifdef STM32_GPIO_ASCR_OFFSET
  /* Connect the analog switch of analog pins */

  if (pinmode == GPIO_MODER_ANALOG)
    {
      modifyreg32(base + STM32_GPIO_ASCR_OFFSET, 0, GPIO_ASCR(pin));
    }
  else
    {
      modifyreg32(base + STM32_GPIO_ASCR_OFFSET, GPIO_ASCR(pin), 0);
    }
#endif

  spin_unlock_irqrestore(&g_configgpio_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default HiZ state (and possibly mark it's unused) and unlock it
 *   whether it was previously selected as alternative function
 *   (GPIO_ALT|GPIO_CNF_AFPP|...).
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

int stm32_unconfiggpio(uint32_t cfgset)
{
  /* Reuse port and pin number and set it to default HiZ INPUT */

  cfgset &= GPIO_PORT_MASK | GPIO_PIN_MASK;
  cfgset |= GPIO_INPUT | GPIO_FLOAT;

  /* To-Do: Mark its unuse for automatic power saving options */

  return stm32_configgpio(cfgset);
}

/****************************************************************************
 * Name: stm32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void stm32_gpiowrite(uint32_t pinset, bool value)
{
  uint32_t base;
  uint32_t bit;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < STM32_NPORTS && g_gpiobase[port] != 0)
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

      putreg32(bit, base + STM32_GPIO_BSRR_OFFSET);
    }
}

/****************************************************************************
 * Name: stm32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool stm32_gpioread(uint32_t pinset)
{
  uint32_t base;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < STM32_NPORTS && g_gpiobase[port] != 0)
    {
      /* Get the port base address */

      base = g_gpiobase[port];

      /* Get the pin number and return the input state of that pin */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      return ((getreg32(base + STM32_GPIO_IDR_OFFSET) & (1 << pin)) != 0);
    }

  return 0;
}
