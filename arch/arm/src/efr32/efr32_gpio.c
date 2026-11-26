/****************************************************************************
 * arch/arm/src/efr32/efr32_gpio.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "efr32_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efr32_setmode
 *
 * Description:
 *   Set the mode of one pin.
 *
 ****************************************************************************/

static void efr32_setmode(int port, int pin, uint32_t mode)
{
  uintptr_t regaddr;

  if (pin < 8)
    {
      regaddr = EFR32_GPIO_PORT_MODEL(port);
    }
  else
    {
      regaddr = EFR32_GPIO_PORT_MODEH(port);
    }

  modifyreg32(regaddr, GPIO_MODE_PIN_MASK(pin),
              mode << GPIO_MODE_PIN_SHIFT(pin));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efr32_configgpio
 *
 * Description:
 *   Configure a pin according to its bit-encoded description.
 *
 * Returned Value:
 *   OK on success; a negated errno value if the pin is invalid.
 *
 ****************************************************************************/

int efr32_configgpio(gpio_pinset_t pinset)
{
  irqstate_t flags;
  uint32_t mode;
  int port;
  int pin;

  port = GPIO_PORTNO(pinset);
  pin  = GPIO_PINNO(pinset);
  mode = (pinset & GPIO_CFG_MASK) >> GPIO_CFG_SHIFT;

  if (port >= EFR32_NGPIO_PORTS)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  /* Set DOUT first so that an output starts at the right level and an
   * input gets the right pull.
   */

  efr32_gpiowrite(pinset, (pinset & GPIO_DOUT) != 0);
  efr32_setmode(port, pin, mode);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: efr32_unconfiggpio
 *
 * Description:
 *   Return a pin to its reset state (disabled, no pull).
 *
 ****************************************************************************/

int efr32_unconfiggpio(gpio_pinset_t pinset)
{
  pinset &= GPIO_PORT_MASK | GPIO_PIN_MASK;
  return efr32_configgpio(pinset | GPIO_DISABLED | GPIO_OUTPUT_CLEAR);
}

/****************************************************************************
 * Name: efr32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin.
 *
 ****************************************************************************/

void efr32_gpiowrite(gpio_pinset_t pinset, bool value)
{
  uintptr_t regaddr;
  int port;
  int pin;

  port    = GPIO_PORTNO(pinset);
  pin     = GPIO_PINNO(pinset);
  regaddr = EFR32_GPIO_PORT_DOUT(port) +
            (value ? EFR32_SET_OFFSET : EFR32_CLR_OFFSET);

  putreg32(1 << pin, regaddr);
}

/****************************************************************************
 * Name: efr32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin.
 *
 ****************************************************************************/

bool efr32_gpioread(gpio_pinset_t pinset)
{
  int port;
  int pin;

  port = GPIO_PORTNO(pinset);
  pin  = GPIO_PINNO(pinset);

  return (getreg32(EFR32_GPIO_PORT_DIN(port)) & (1 << pin)) != 0;
}
