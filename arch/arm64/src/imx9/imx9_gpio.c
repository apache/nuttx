/****************************************************************************
 * arch/arm64/src/imx9/imx9_gpio.c
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
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include <imx9_gpiobase.c>

#include "chip.h"
#include "arm64_internal.h"
#include "imx9_iomuxc.h"
#include "imx9_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Spinlock */

static spinlock_t g_gpio_lock = SP_UNLOCKED;

/****************************************************************************
 * Name: imx9_gpio_dirout
 ****************************************************************************/

static inline void imx9_gpio_dirout(uint32_t port, uint32_t pin)
{
  uint32_t regval = getreg32(IMX9_GPIO_PDDR(port));
  regval |= GPIO_PIN(pin);
  putreg32(regval, IMX9_GPIO_PDDR(port));
}

/****************************************************************************
 * Name: imx9_gpio_dirin
 ****************************************************************************/

static inline void imx9_gpio_dirin(uint32_t port, uint32_t pin)
{
  uint32_t regval = getreg32(IMX9_GPIO_PDDR(port));
  regval &= ~GPIO_PIN(pin);
  putreg32(regval, IMX9_GPIO_PDDR(port));
}

/****************************************************************************
 * Name: imx9_gpio_setoutput
 ****************************************************************************/

static void imx9_gpio_setoutput(uint32_t port, uint32_t pin, bool value)
{
  uintptr_t regaddr = IMX9_GPIO_PDOR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  if (value)
    {
      regval |= GPIO_PIN(pin);
    }
  else
    {
      regval &= ~GPIO_PIN(pin);
    }

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: imx9_gpio_getpin_status
 ****************************************************************************/

static inline bool imx9_gpio_get_pinstatus(uint32_t port, uint32_t pin)
{
  uintptr_t regaddr = IMX9_GPIO_PSOR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: imx9_gpio_getinput
 ****************************************************************************/

static inline bool imx9_gpio_getinput(uint32_t port, uint32_t pin)
{
  uintptr_t regaddr = IMX9_GPIO_PDIR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: imx9_gpio_configinput
 ****************************************************************************/

static int imx9_gpio_configinput(gpio_pinset_t pinset)
{
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT((unsigned int)port < IMX9_GPIO_NPORTS);

  /* Configure pin as in input */

  imx9_gpio_dirin(port, pin);

  return OK;
}

/****************************************************************************
 * Name: imx9_gpio_configoutput
 ****************************************************************************/

static inline int imx9_gpio_configoutput(gpio_pinset_t pinset)
{
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value    = ((pinset & GPIO_OUTPUT_ONE) != 0);

  DEBUGASSERT((unsigned int)port < IMX9_GPIO_NPORTS);

  /* Set the output value */

  imx9_gpio_setoutput(port, pin, value);

  /* Convert the configured input GPIO to an output */

  imx9_gpio_dirout(port, pin);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description of the pin.
 *
 ****************************************************************************/

int imx9_config_gpio(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int ret;

  /* Configure the pin as an input initially to avoid any spurious outputs */

  flags = spin_lock_irqsave(&g_gpio_lock);

  /* Configure based upon the pin mode */

  switch (pinset & GPIO_MODE_MASK)
    {
      case GPIO_INPUT:
        {
          /* Configure the pin as a GPIO input */

          ret = imx9_gpio_configinput(pinset);
        }
        break;

      case GPIO_OUTPUT:
        {
          /* First configure the pin as a GPIO input to avoid output
           * glitches.
           */

          ret = imx9_gpio_configinput(pinset);
          if (ret >= 0)
            {
              /* Convert the input to an output */

              ret = imx9_gpio_configoutput(pinset);
            }
        }
        break;

#ifdef CONFIG_IMX9_GPIO_IRQ
      case GPIO_INTERRUPT:
        {
          /* Configure the pin as a GPIO input */

          ret = imx9_gpio_configinput(pinset);
          if (ret == OK)
            {
              ret = imx9_gpioirq_configure(pinset);
            }
        }
        break;
#endif

      default:
        ret = -EINVAL;
        break;
    }

  spin_unlock_irqrestore(&g_gpio_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: imx9_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void imx9_gpio_write(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT((unsigned int)port < IMX9_GPIO_NPORTS);

  flags = spin_lock_irqsave(&g_gpio_lock);
  imx9_gpio_setoutput(port, pin, value);
  spin_unlock_irqrestore(&g_gpio_lock, flags);
}

/****************************************************************************
 * Name: imx9_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool imx9_gpio_read(gpio_pinset_t pinset)
{
  irqstate_t flags;
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value;

  DEBUGASSERT((unsigned int)port < IMX9_GPIO_NPORTS);

  flags = spin_lock_irqsave(&g_gpio_lock);
  if ((pinset & (GPIO_OUTPUT)) == (GPIO_OUTPUT))
    {
      value = imx9_gpio_get_pinstatus(port, pin);
    }
  else
    {
      value = imx9_gpio_getinput(port, pin);
    }

  spin_unlock_irqrestore(&g_gpio_lock, flags);
  return value;
}
