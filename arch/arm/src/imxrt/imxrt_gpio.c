/****************************************************************************
 * arch/arm/src/imxrt/imxrt_gpio.c
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
#include "imxrt_iomuxc.h"
#include "imxrt_gpio.h"
#include "hardware/imxrt_daisy.h"

#if defined(CONFIG_ARCH_FAMILY_IMXRT102x)
#  include "imxrt102x_gpio.c"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT105x)
#  include "imxrt105x_gpio.c"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT106x)
#  include "imxrt106x_gpio.c"
#elif defined(CONFIG_ARCH_FAMILY_IMXRT117x)
#  include "imxrt117x_gpio.c"
#else
#  error Unrecognized i.MX RT architecture
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Look-up table that maps GPIO1..GPIOn indexes into GPIO register base
 * addresses
 */

const uintptr_t g_gpio_base[IMXRT_GPIO_NPORTS] =
{
  IMXRT_GPIO1_BASE
#if IMXRT_GPIO_NPORTS > 1
  , IMXRT_GPIO2_BASE
#endif
#if IMXRT_GPIO_NPORTS > 2
  , IMXRT_GPIO3_BASE
#endif
#if IMXRT_GPIO_NPORTS > 3
#if defined(IMXRT_GPIO4_BASE)
  , IMXRT_GPIO4_BASE
#else
  , 0
#endif
#endif
#if IMXRT_GPIO_NPORTS > 4
  , IMXRT_GPIO5_BASE
#endif
#if IMXRT_GPIO_NPORTS > 5
  , IMXRT_GPIO6_BASE
#endif
#if IMXRT_GPIO_NPORTS > 6
  , IMXRT_GPIO7_BASE
#endif
#if IMXRT_GPIO_NPORTS > 7
  , IMXRT_GPIO8_BASE
#endif
#if IMXRT_GPIO_NPORTS > 8
  , IMXRT_GPIO9_BASE
#endif
#if IMXRT_GPIO_NPORTS > 9
  , IMXRT_GPIO10_BASE
#endif
#if IMXRT_GPIO_NPORTS > 10
  , IMXRT_GPIO11_BASE
#endif
#if IMXRT_GPIO_NPORTS > 11
  , IMXRT_GPIO12_BASE
#endif
#if IMXRT_GPIO_NPORTS > 12
  , IMXRT_GPIO13_BASE
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_padmux_address
 ****************************************************************************/

static uintptr_t imxrt_padmux_address(unsigned int index)
{
#if defined(IMXRT_PAD1MUX_OFFSET)
  if (index >= IMXRT_PADMUX_GPIO_SPI_B0_00_INDEX)
    {
      return (IMXRT_PAD1MUX_OFFSET(index -
                                   IMXRT_PADMUX_GPIO_SPI_B0_00_INDEX));
    }

#elif defined(IMXRT_PADMUX_OFFSET_LPSR)
  if (index >= IMXRT_PADMUX_GPIO_LPSR_00_INDEX)
    {
      return (IMXRT_PADMUX_ADDRESS_LPSR(index -
                                        IMXRT_PADMUX_GPIO_LPSR_00_INDEX));
    }

#endif
  if (index >= IMXRT_PADMUX_WAKEUP_INDEX)
    {
      return (IMXRT_PADMUX_ADDRESS_SNVS(index -
                                        IMXRT_PADMUX_WAKEUP_INDEX));
    }

  return (IMXRT_PADMUX_ADDRESS(index));
}

/****************************************************************************
 * Name: imxrt_padctl_address
 ****************************************************************************/

static uintptr_t imxrt_padctl_address(unsigned int index)
{
#if defined(IMXRT_PAD1CTL_OFFSET)
  if (index >= IMXRT_PADCTL_GPIO_SPI_B0_00_INDEX)
    {
      return (IMXRT_PAD1CTL_OFFSET(index -
                                   IMXRT_PADCTL_GPIO_SPI_B0_00_INDEX));
    }

#elif defined(IMXRT_PADCTL_OFFSET_LPSR)
  if (index >= IMXRT_PADCTL_GPIO_LPSR_00_INDEX)
    {
      return (IMXRT_PADCTL_ADDRESS_LPSR(index -
                                        IMXRT_PADCTL_GPIO_LPSR_00_INDEX));
    }

#endif
#if defined(IMXRT_PADCTL_TEST_MODE_INDEX)
  if (index >= IMXRT_PADCTL_TEST_MODE_INDEX)
    {
      return (IMXRT_PADCTL_ADDRESS_SNVS(index -
                                        IMXRT_PADCTL_TEST_MODE_INDEX));
    }
#else
  if (index >= IMXRT_PADCTL_WAKEUP_INDEX)
    {
      return (IMXRT_PADCTL_ADDRESS_SNVS(index -
                                        IMXRT_PADCTL_WAKEUP_INDEX));
    }
#endif

  return (IMXRT_PADCTL_ADDRESS(index));
}

/****************************************************************************
 * Name: imxrt_gpio_dirout
 ****************************************************************************/

static inline void imxrt_gpio_dirout(int port, int pin)
{
  uint32_t regval = getreg32(IMXRT_GPIO_GDIR(port));
  regval |= GPIO_PIN(pin);
  putreg32(regval, IMXRT_GPIO_GDIR(port));
}

/****************************************************************************
 * Name: imxrt_gpio_dirin
 ****************************************************************************/

static inline void imxrt_gpio_dirin(int port, int pin)
{
  uint32_t regval = getreg32(IMXRT_GPIO_GDIR(port));
  regval &= ~GPIO_PIN(pin);
  putreg32(regval, IMXRT_GPIO_GDIR(port));
}

/****************************************************************************
 * Name: imxrt_gpio_setoutput
 ****************************************************************************/

static void imxrt_gpio_setoutput(int port, int pin, bool value)
{
  uintptr_t regaddr = IMXRT_GPIO_DR(port);
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
 * Name: imxrt_gpio_getpin_status
 ****************************************************************************/

static inline bool imxrt_gpio_get_pinstatus(int port, int pin)
{
  uintptr_t regaddr = IMXRT_GPIO_PSR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: imxrt_gpio_getinput
 ****************************************************************************/

static inline bool imxrt_gpio_getinput(int port, int pin)
{
  uintptr_t regaddr = IMXRT_GPIO_DR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: imxrt_gpio_select
 * GPIO{1234}(l) and GPIO{6789}(h) share same IO MUX function, GPIO_MUXn
 * selects one GPIO function.
 * 0: GPIOl[n] is selected
 * 1: GPIOh[n] is selected
 ****************************************************************************/

static inline int imxrt_gpio_select(int port, int pin)
{
#if IMXRT_GPIO_NPORTS > 5 && defined(CONFIG_IMXRT_IOMUX_VER1)
  uint32_t gpr = port;
  uint32_t setbits = 1 << pin;
  uint32_t clearbits = 1 << pin;
  uintptr_t regaddr = (uintptr_t) IMXRT_IOMUXC_GPR_GPR26;

  if (port != GPIO5)
    {
      /* Uses GPR26 as the base */

      if (port >= GPIO6)
        {
          /* Map port to correct gpr index and set the GPIO_MUX3_GPIO[b]_SEL
           * bit
           */

          gpr = port - GPIO6;
          clearbits = 0;
        }
      else
        {
          /* The port is correct gpr index, so just clear the
           * GPIO_MUX3_GPIO[b]_SEL bit.
           */

          setbits = 0;
        }

      regaddr |= gpr * sizeof(uint32_t);
      modifyreg32(regaddr, clearbits, setbits);
    }

#endif
  return OK;
}

/****************************************************************************
 * Name: imxrt_gpio_configinput
 ****************************************************************************/

static int imxrt_gpio_configinput(gpio_pinset_t pinset)
{
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  const uint8_t *table;
  iomux_pinset_t ioset;
  uintptr_t regaddr;
  unsigned int index;
  uint32_t sion = 0;

  DEBUGASSERT((unsigned int)port < IMXRT_GPIO_NPORTS);

  /* Configure pin as in input */

  imxrt_gpio_dirin(port, pin);

  /* Configure pin as a GPIO */

  table = g_gpio_padmux[port];
  if (table == NULL)
    {
      return -EINVAL;
    }

  index = (unsigned int)table[pin];
  if (index >= IMXRT_PADMUX_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = imxrt_padmux_address(index);

  if ((pinset & GPIO_OUTPUT) == GPIO_OUTPUT)
    {
      sion |= (pinset & GPIO_SION_MASK) ? PADMUX_SION : 0;
    }

  putreg32(PADMUX_MUXMODE_ALT5 | sion, regaddr);

  imxrt_gpio_select(port, pin);

  /* Configure pin pad settings */

  index = imxrt_padmux_map(index);
  if (index >= IMXRT_PADCTL_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = imxrt_padctl_address(index);
  ioset   = (iomux_pinset_t)((pinset & GPIO_IOMUX_MASK) >> GPIO_IOMUX_SHIFT);
  return imxrt_iomux_configure(regaddr, ioset);
}

/****************************************************************************
 * Name: imxrt_gpio_configoutput
 ****************************************************************************/

static inline int imxrt_gpio_configoutput(gpio_pinset_t pinset)
{
  int port   = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin    = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value = ((pinset & GPIO_OUTPUT_ONE) != 0);

  DEBUGASSERT((unsigned int)port < IMXRT_GPIO_NPORTS);

  /* Set the output value */

  imxrt_gpio_setoutput(port, pin, value);

  /* Convert the configured input GPIO to an output */

  imxrt_gpio_dirout(port, pin);
  return OK;
}

/****************************************************************************
 * Name: imxrt_gpio_configperiph
 ****************************************************************************/

static inline int imxrt_gpio_configperiph(gpio_pinset_t pinset)
{
  iomux_pinset_t ioset;
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t alt;
  unsigned int index;

  /* Configure pin as a peripheral via SW MUX Control Register */

  index   = ((pinset & GPIO_PADMUX_MASK) >> GPIO_PADMUX_SHIFT);
  regaddr = imxrt_padmux_address(index);

  alt     = (pinset & GPIO_ALT_MASK) >> GPIO_ALT_SHIFT;
  regval  = alt << PADMUX_MUXMODE_SHIFT;
  regval |= (pinset & GPIO_SION_MASK) ? PADMUX_SION : 0;

  putreg32(regval, regaddr);

  /* Configure pin Daisy Select Input Daisy Register */

  imxrt_daisy_select(index, alt);

  /* Configure pin pad settings SW PAD Control Register */

  index = imxrt_padmux_map(index);
  if (index >= IMXRT_PADCTL_NREGISTERS)
    {
      return -EINVAL;
    }

  regaddr = imxrt_padctl_address(index);
  ioset   = (iomux_pinset_t)((pinset & GPIO_IOMUX_MASK) >> GPIO_IOMUX_SHIFT);
  return imxrt_iomux_configure(regaddr, ioset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description of the pin.
 *
 ****************************************************************************/

int imxrt_config_gpio(gpio_pinset_t pinset)
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

          ret = imxrt_gpio_configinput(pinset);
        }
        break;

      case GPIO_OUTPUT:
        {
          /* First configure the pin as a GPIO input to avoid output
           * glitches.
           */

          ret = imxrt_gpio_configinput(pinset);
          if (ret >= 0)
            {
              /* Convert the input to an output */

              ret = imxrt_gpio_configoutput(pinset);
            }
        }
        break;

      case GPIO_PERIPH:
        {
          /* Configure the pin as a peripheral */

          ret = imxrt_gpio_configperiph(pinset);
        }
        break;

#ifdef CONFIG_IMXRT_GPIO_IRQ
      case GPIO_INTERRUPT:
        {
          /* Configure the pin as a GPIO input */

          ret = imxrt_gpio_configinput(pinset);
          if (ret == OK)
            {
              ret = imxrt_gpioirq_configure(pinset);
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
 * Name: imxrt_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void imxrt_gpio_write(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT((unsigned int)port < IMXRT_GPIO_NPORTS);

  flags = enter_critical_section();
  imxrt_gpio_setoutput(port, pin, value);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: imxrt_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool imxrt_gpio_read(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value;

  DEBUGASSERT((unsigned int)port < IMXRT_GPIO_NPORTS);

  flags = enter_critical_section();
  if ((pinset & (GPIO_OUTPUT | GPIO_SION_ENABLE)) ==
                (GPIO_OUTPUT | GPIO_SION_ENABLE))
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
