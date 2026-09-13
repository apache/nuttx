/****************************************************************************
 * arch/arm/src/imxrt/imxrt_rgpio.c
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

#include "chip.h"
#include "arm_internal.h"
#include "imxrt_rgpio.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Look-up table that maps GPIO1..GPIOn indexes into RGPIO base addresses.
 * GPIO1 lives in the AON domain; GPIO2..GPIO6 are in the WAKEUPMIX.
 */

const uintptr_t g_gpio_base[IMXRT_GPIO_NPORTS] =
{
  IMXRT_GPIO1_BASE,
  IMXRT_GPIO2_BASE,
  IMXRT_GPIO3_BASE,
  IMXRT_GPIO4_BASE,
  IMXRT_GPIO5_BASE,
  IMXRT_GPIO6_BASE,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void imxrt_gpio_dirout(uint32_t port, uint32_t pin)
{
  uint32_t regval = getreg32(IMXRT_GPIO_PDDR(port));
  regval |= GPIO_PIN(pin);
  putreg32(regval, IMXRT_GPIO_PDDR(port));
}

static inline void imxrt_gpio_dirin(uint32_t port, uint32_t pin)
{
  uint32_t regval = getreg32(IMXRT_GPIO_PDDR(port));
  regval &= ~GPIO_PIN(pin);
  putreg32(regval, IMXRT_GPIO_PDDR(port));
}

static void imxrt_gpio_setoutput(uint32_t port, uint32_t pin, bool value)
{
  uintptr_t regaddr = value ? IMXRT_GPIO_PSOR(port) : IMXRT_GPIO_PCOR(port);
  putreg32(GPIO_PIN(pin), regaddr);
}

static inline bool imxrt_gpio_get_pinstatus(uint32_t port, uint32_t pin)
{
  return (getreg32(IMXRT_GPIO_PDOR(port)) & GPIO_PIN(pin)) != 0;
}

static inline bool imxrt_gpio_getinput(uint32_t port, uint32_t pin)
{
  return (getreg32(IMXRT_GPIO_PDIR(port)) & GPIO_PIN(pin)) != 0;
}

static int imxrt_gpio_configinput(uint16_t gpio)
{
  uint32_t port = (gpio & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (gpio & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT(port < IMXRT_GPIO_NPORTS);
  imxrt_gpio_dirin(port, pin);
  return OK;
}

static inline int imxrt_gpio_configoutput(uint16_t gpio)
{
  uint32_t port = (gpio & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (gpio & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value    = (gpio & GPIO_OUTPUT_ONE) != 0;

  DEBUGASSERT(port < IMXRT_GPIO_NPORTS);

  imxrt_gpio_setoutput(port, pin, value);
  imxrt_gpio_dirout(port, pin);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_config_gpio
 ****************************************************************************/

int imxrt_config_gpio(gpio_pinset_t pinset)
{
  irqstate_t flags;
  uint16_t   gpio = IMXRT_PINSET_GPIO(pinset);
  int        ret;

  flags = enter_critical_section();

  /* Program the pad routing (MUX_CTL/PAD_CTL/DAISY) first. */

  ret = imxrt_iomux_configure(pinset);
  if (ret < 0)
    {
      leave_critical_section(flags);
      return ret;
    }

  /* Then configure the RGPIO instance according to the gpio subset. */

  switch (gpio & GPIO_MODE_MASK)
    {
      case GPIO_INPUT:
        ret = imxrt_gpio_configinput(gpio);
        break;

      case GPIO_OUTPUT:

        /* Configure as input first to avoid glitches, then flip direction */

        ret = imxrt_gpio_configinput(gpio);
        if (ret >= 0)
          {
            ret = imxrt_gpio_configoutput(gpio);
          }
        break;

#ifdef CONFIG_IMXRT_GPIO_IRQ
      case GPIO_INTERRUPT:
        ret = imxrt_gpio_configinput(gpio);
        if (ret == OK)
          {
            ret = imxrt_gpioirq_configure(pinset);
          }
        break;
#endif

      default:

        /* Peripheral pinset (no GPIO bits set): pad routing done above,
         * nothing else to do.
         */

        ret = OK;
        break;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: imxrt_gpio_write
 ****************************************************************************/

void imxrt_gpio_write(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  uint16_t gpio = IMXRT_PINSET_GPIO(pinset);
  uint32_t port = (gpio & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (gpio & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT(port < IMXRT_GPIO_NPORTS);

  flags = enter_critical_section();
  imxrt_gpio_setoutput(port, pin, value);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: imxrt_gpio_read
 ****************************************************************************/

bool imxrt_gpio_read(gpio_pinset_t pinset)
{
  irqstate_t flags;
  uint16_t gpio = IMXRT_PINSET_GPIO(pinset);
  uint32_t port = (gpio & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin  = (gpio & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value;

  DEBUGASSERT(port < IMXRT_GPIO_NPORTS);

  flags = enter_critical_section();
  if ((gpio & GPIO_MODE_MASK) == GPIO_OUTPUT)
    {
      value = imxrt_gpio_get_pinstatus(port, pin);
    }
  else
    {
      value = imxrt_gpio_getinput(port, pin);
    }

  leave_critical_section(flags);
  return value;
}
