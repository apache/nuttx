/***************************************************************************
 * arch/risc-v/src/hpm6000/hpm_gpio.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>

#include <chip.h>

#include "hardware/hpm_memorymap.h"
#include "hardware/hpm_ioc.h"
#include "hardware/hpm_gpio.h"
#include "hpm_gpio.h"
#include "riscv_internal.h"
#include "hpm_ioc.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#define HPM_GPIO_NPORTS 6

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/***************************************************************************
 * Public Data
 ***************************************************************************/

/***************************************************************************
 * Name: hpm_gpio_dirout
 ***************************************************************************/

static inline void hpm_gpio_dirout(int port, int pin)
{
  uint32_t regval = getreg32(HPM_GPIO_OE_SET(port));
  regval |= GPIO_PIN(pin);
  putreg32(regval, HPM_GPIO_OE_SET(port));
}

/***************************************************************************
 * Name: hpm_gpio_diin
 ***************************************************************************/

static inline void hpm_gpio_dirin(int port, int pin)
{
  uint32_t regval = getreg32(HPM_GPIO_OE_CLR(port));
  regval |= GPIO_PIN(pin);
  putreg32(regval, HPM_GPIO_OE_CLR(port));
}

/***************************************************************************
 * Name: hpm_gpio_setoutput
 ***************************************************************************/

static void hpm_gpio_setoutput(int port, int pin, bool value)
{
  uint32_t regval = 0;

  if (value)
    {
      regval = getreg32(HPM_GPIO_DO_SET(port));
      regval |= GPIO_PIN(pin);
      putreg32(regval, HPM_GPIO_DO_SET(port));
    }
  else
    {
      regval = getreg32(HPM_GPIO_DO_CLR(port));
      regval |= GPIO_PIN(pin);
      putreg32(regval, HPM_GPIO_DO_CLR(port));
    }
}

/***************************************************************************
 * Name: hpm_gpio_getinput
 ***************************************************************************/

static inline bool hpm_gpio_getinput(int port, int pin)
{
  uint32_t regval = getreg32(HPM_GPIO_DI_VAL(port));

  return ((regval & GPIO_PIN(pin)) != 0);
}

/***************************************************************************
 * Name: hpm_gpio_configinput
 ***************************************************************************/

static int hpm_gpio_configinput(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  ioc_pinset_t ioset;
  uintptr_t padctl;

  /* Configure pin as */

  hpm_gpio_dirin(port, pin);

  /* Configure pin as a GPIO */

  putreg32(IOC_PAD_FUNC_ALT_SELECT_ALT0, IOC_FUNC_CTL_(port, pin));

  /* Configure pin pad settings */

  padctl = IOC_PAD_CTL_(port, pin);

  ioset = (ioc_pinset_t)(pinset & GPIO_IOCPAD_MASK) >> GPIO_IOCPAD_SHIFT;

  return hpm_iocpad_configure(padctl, ioset);
}

/***************************************************************************
 * Name: hpm_gpio_configoutput
 ***************************************************************************/

static inline int hpm_gpio_configoutput(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value = ((pinset & GPIO_OUTPUT_ONE) != 0);

  /* Set the output value */

  hpm_gpio_setoutput(port, pin, value);

  /* Convert the configured input GPIO to an poutput */

  hpm_gpio_dirout(port, pin);

  return OK;
}

/***************************************************************************
 * Name: hpm_gpio_configperiph
 ***************************************************************************/

static inline int hpm_gpio_configperiph(gpio_pinset_t pinset)
{
  unsigned int index;
  ioc_pinset_t ioset;
  uintptr_t regaddr;

  index = ((pinset & GPIO_PADMUX_MASK) >> GPIO_PADMUX_SHIFT);
  ioset = (ioc_pinset_t)(pinset & GPIO_IOCPAD_MASK) >> GPIO_IOCPAD_SHIFT;

  regaddr = HPM_IOC_PAD_PAD_CTL_ADDRESS(index);

  hpm_iocpad_configure(regaddr, ioset);

  if (index >= HPM_IOC_PAD_PY00_INDEX)
    {
      regaddr = HPM_PIOC_PAD_PAD_CTL_ADDRESS(index);
      hpm_iocpad_configure(regaddr, PAD_ALT3);
    }

  return OK;
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: hpm_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description of the pin.
 *
 ***************************************************************************/

int hpm_gpio_config(gpio_pinset_t pinset)
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

          ret = hpm_gpio_configinput(pinset);
        }
        break;

      case GPIO_OUTPUT:
        {
          ret = hpm_gpio_configoutput(pinset);
        }
        break;

      case GPIO_PERIPH:
        {
          ret = hpm_gpio_configperiph(pinset);
        }
        break;

#ifdef CONFIG_HPM_GPIO_IRQ
    /* TODO: irq configure */
#endif
      default:
        ret = -EINVAL;
        break;
    }

  leave_critical_section(flags);
  return ret;
}

/***************************************************************************
 * Name: hpm_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ***************************************************************************/

void hpm_gpio_write(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  flags = enter_critical_section();
  hpm_gpio_setoutput(port, pin, value);
  leave_critical_section(flags);
}

/***************************************************************************
 * Name: hpm_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ***************************************************************************/

bool hpm_gpio_read(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value;

  flags = enter_critical_section();
  if ((pinset &(GPIO_OUTPUT | GPIO_LOOP_ENABLE)) ==
               (GPIO_OUTPUT | GPIO_LOOP_ENABLE))
    {
      /* TODO: read input state */
    }
  else
    {
      value = hpm_gpio_getinput(port, pin);
    }

  leave_critical_section(flags);

  return value;
}
