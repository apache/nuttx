/****************************************************************************
 * arch/arm/src/mcx-nxxx/nxxx_gpio.c
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

#include "nxxx_gpio.h"

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "nxxx_gpiobase.c"
#include "arm_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxxx_gpio_dirout
 ****************************************************************************/

static inline void nxxx_gpio_dirout(uint32_t port, uint32_t pin)
{
  uint32_t regval = getreg32(NXXX_GPIO_PDDR(port));
  regval |= GPIO_PIN(pin);
  putreg32(regval, NXXX_GPIO_PDDR(port));
}

/****************************************************************************
 * Name: nxxx_gpio_dirin
 ****************************************************************************/

static inline void nxxx_gpio_dirin(uint32_t port, uint32_t pin)
{
  uint32_t regval = getreg32(NXXX_GPIO_PDDR(port));
  regval &= ~GPIO_PIN(pin);
  putreg32(regval, NXXX_GPIO_PDDR(port));
}

/****************************************************************************
 * Name: nxxx_gpio_setoutput
 ****************************************************************************/

static void nxxx_gpio_setoutput(uint32_t port, uint32_t pin, bool value)
{
  uintptr_t regaddr = NXXX_GPIO_PDOR(port);
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
 * Name: nxxx_gpio_getpin_status
 ****************************************************************************/

static inline bool nxxx_gpio_get_pinstatus(uint32_t port, uint32_t pin)
{
  uintptr_t regaddr = NXXX_GPIO_PSOR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: nxxx_gpio_getinput
 ****************************************************************************/

static inline bool nxxx_gpio_getinput(uint32_t port, uint32_t pin)
{
  uintptr_t regaddr = NXXX_GPIO_PDIR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: nxxx_gpio_configinput
 ****************************************************************************/

static int nxxx_gpio_configinput(gpio_pinset_t pinset)
{
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT((unsigned int)port < NXXX_GPIO_NPORTS);

  /* Configure pin as in input */

  nxxx_gpio_dirin(port, pin);

  return OK;
}

/****************************************************************************
 * Name: nxxx_gpio_configoutput
 ****************************************************************************/

static inline int nxxx_gpio_configoutput(gpio_pinset_t pinset)
{
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value    = ((pinset & GPIO_OUTPUT_ONE) != 0);

  DEBUGASSERT((unsigned int)port < NXXX_GPIO_NPORTS);

  /* Set the output value */

  nxxx_gpio_setoutput(port, pin, value);

  /* Convert the configured input GPIO to an output */

  nxxx_gpio_dirout(port, pin);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxxx_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description of the pin.
 *
 ****************************************************************************/

int nxxx_config_gpio(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int ret;

  /* Configure the pin as an input initially to avoid any spurious outputs */

  flags = enter_critical_section();

  /* Configure based upon the pin mode */

  switch (pinset & GPIO_MODE_MASK)
    {
      case GPIO_INPUT:
        {
          /* Configure the pin as a GPIO input */

          ret = nxxx_gpio_configinput(pinset);
        }
        break;

      case GPIO_OUTPUT:
        {
          /* First configure the pin as a GPIO input to avoid output
           * glitches.
           */

          ret = nxxx_gpio_configinput(pinset);
          if (ret >= 0)
            {
              /* Convert the input to an output */

              ret = nxxx_gpio_configoutput(pinset);
            }
        }
        break;

#ifdef CONFIG_NXXX_GPIO_IRQ
      case GPIO_INTERRUPT:
        {
          /* Configure the pin as a GPIO input */

          ret = nxxx_gpio_configinput(pinset);
          if (ret == OK)
            {
              ret = nxxx_gpioirq_configure(pinset);
            }
        }
        break;
#endif

      default:
        ret = -EINVAL;
        break;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: nxxx_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void nxxx_gpio_write(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT((unsigned int)port < NXXX_GPIO_NPORTS);

  flags = enter_critical_section();
  nxxx_gpio_setoutput(port, pin, value);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nxxx_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool nxxx_gpio_read(gpio_pinset_t pinset)
{
  irqstate_t flags;
  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value;

  DEBUGASSERT((unsigned int)port < NXXX_GPIO_NPORTS);

  flags = enter_critical_section();
  if ((pinset & (GPIO_OUTPUT)) == (GPIO_OUTPUT))
    {
      value = nxxx_gpio_get_pinstatus(port, pin);
    }
  else
    {
      value = nxxx_gpio_getinput(port, pin);
    }

  leave_critical_section(flags);
  return value;
}
