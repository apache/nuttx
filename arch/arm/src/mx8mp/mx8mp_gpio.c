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
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
          printf("output %lx -> %lx (%lu) (%lu)\n", GPIO_GDIR(port), getreg32(GPIO_GDIR(port)), port, pin);
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
  uint32_t regaddr = GPIO_DR(port);
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

  uint32_t regval = getreg32(GPIO_DR(port));
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
 * Name: mx8mp_gpio_enable_irq
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

int mx8mp_gpio_enable_irq(int irq)
{
#if 0
  uintptr_t regaddr;
  unsigned int pin;
  int ret;

  ret = imxrt_gpio_info(irq, &regaddr, &pin);
  if (ret >= 0)
    {
      modifyreg32(regaddr, 0, 1 << pin);
    }

  return ret;
#endif
  return OK;
}

/****************************************************************************
 * Name: mx8mp_gpio_disable_irq
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

int mx8mp_gpio_disable_irq(int irq)
{
#if 0
  uintptr_t regaddr;
  unsigned int pin;
  int ret;
  ret = imxrt_gpio_info(irq, &regaddr, &pin);
  if (ret >= 0)
    {
      modifyreg32(regaddr, 1 << pin, 0);
    }

  return ret;
  #endif
  return OK;
}
