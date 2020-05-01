/****************************************************************************
 * arch/arm/src/nuc1xx/nuc_gpio.c
 *
 *   Copyright (C) 2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <arch/nuc1xx/chip.h>

#include "arm_arch.h"

#include "chip.h"
#include "hardware/nuc_gpio.h"

#include "nuc_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nuc_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with nuc_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port, or when pin is locked as ALT function.
 *
 ****************************************************************************/

int nuc_configgpio(gpio_cfgset_t cfgset)
{
  uintptr_t base;
  uint32_t regaddr;
  uint32_t regval;
  uint32_t isrc;
  uint32_t imd;
  uint32_t ien;
  uint32_t value;
  int port;
  int pin;

  /* Decode the port and pin.  Use the port number to get the GPIO base
   * address.
   */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin  = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT((unsigned)port <= NUC_GPIO_PORTE);
  base = NUC_GPIO_CTRL_BASE(port);

  /* Set the GPIO PMD register */

  regaddr = base + NUC_GPIO_PMD_OFFSET;
  regval = getreg32(regaddr);
  regval &= ~GPIO_PMD_MASK(pin);

  switch (cfgset & GPIO_MODE_MASK)
    {
    default:
    case GPIO_INPUT:     /* Input */
      value = GPIO_PMD_INPUT;
      break;

    case GPIO_OUTPUT:    /* Push-pull output */
      value = GPIO_PMD_OUTPUT;
      break;

    case GPIO_OPENDRAIN: /* Open drain output */
      value = GPIO_PMD_OPENDRAIN;
      break;

    case GPIO_BIDI:      /* Quasi bi-directional */
      value = GPIO_PMD_BIDI;
      break;
    }

  regval |= GPIO_PMD(pin, value);
  putreg32(regval, regaddr);

  /* Check if we need to disable the digital input path */

  regaddr = base + NUC_GPIO_OFFD_OFFSET;
  regval  = getreg32(regaddr);
  regval &= ~GPIO_OFFD(pin);

  if ((cfgset & GPIO_ANALOG) != 0)
    {
      regval |= GPIO_OFFD(pin);
    }

  putreg32(regval, regaddr);

  /* Check if we need to enable debouncing */

  regaddr = base + NUC_GPIO_DBEN_OFFSET;
  regval  = getreg32(regaddr);
  regval &= ~GPIO_DBEN(pin);

  if ((cfgset & GPIO_DEBOUNCE) != 0)
    {
      regval |= GPIO_DBEN(pin);
    }

  putreg32(regval, regaddr);

  /* Configure interrupting pins */

  isrc = getreg32(base + NUC_GPIO_ISRC_OFFSET);
  isrc &= ~GPIO_ISRC(pin);

  imd  = getreg32(base + NUC_GPIO_IMD_OFFSET);
  imd &= ~GPIO_IMD(pin);

  ien  = getreg32(base + NUC_GPIO_IEN_OFFSET);
  ien &= ~(GPIO_IF_EN(pin) | GPIO_IR_EN(pin));

  switch (cfgset & GPIO_INTERRUPT_MASK)
    {
    case GPIO_INTERRUPT_RISING_EDGE:
      isrc |= GPIO_ISRC(pin);
      ien  |= GPIO_IR_EN(pin);
      break;

    case GPIO_INTERRUPT_FALLING_EDGE:
      isrc |= GPIO_ISRC(pin);
      ien  |= GPIO_IF_EN(pin);
      break;

    case GPIO_INTERRUPT_BOTH_EDGES:
      isrc |= GPIO_ISRC(pin);
      ien  |= (GPIO_IF_EN(pin) | GPIO_IR_EN(pin));
      break;

    case GPIO_INTERRUPT_HIGH_LEVEL:
      isrc |= GPIO_ISRC(pin);
      imd  |= GPIO_IMD(pin);
      ien  |= GPIO_IR_EN(pin);
      break;

    case GPIO_INTERRUPT_LOW_LEVEL:
      isrc |= GPIO_ISRC(pin);
      imd  |= GPIO_IMD(pin);
      ien  |= GPIO_IF_EN(pin);
      break;

    default:
      break;
    }

  putreg32(ien, base + NUC_GPIO_IEN_OFFSET);
  putreg32(imd, base + NUC_GPIO_IMD_OFFSET);
  putreg32(isrc, base + NUC_GPIO_ISRC_OFFSET);

  /* If the pin is an output, set the initial output value */

  if ((cfgset & GPIO_MODE_MASK) == GPIO_OUTPUT)
    {
      nuc_gpiowrite(cfgset, (cfgset & GPIO_OUTPUT_SET) != 0);
    }

  return 0;
}

/****************************************************************************
 * Name: nuc_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void nuc_gpiowrite(gpio_cfgset_t pinset, bool value)
{
#ifndef NUC_LOW
  irqstate_t flags;
  uintptr_t base;
#endif
  int port;
  int pin;

  /* Decode the port and pin.  Use the port number to get the GPIO base
   * address.
   */

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT((unsigned)port <= NUC_GPIO_PORTE);

  /* Only the low density NUC100/120 chips support bit-band access to GPIO
   * pins.
   */

#ifdef NUC_LOW
  putreg32((uint32_t)value, NUC_PORT_PDIO(port, pin));
#else
  /* Get the base address of the GPIO port registers */

  base = NUC_GPIO_CTRL_BASE(port);

  /* Disable interrupts -- the following operations must be atomic */

  flags = enter_critical_section();

  /* Allow writing only to the selected pin in the DOUT register */

  putreg32(~(1 << pin), base + NUC_GPIO_DMASK_OFFSET);

  /* Set the pin to the selected value and re-enable interrupts */

  putreg32(((uint32_t)value << pin), base + NUC_GPIO_DOUT_OFFSET);
  leave_critical_section(flags);
#endif
}

/****************************************************************************
 * Name: nuc_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool nuc_gpioread(gpio_cfgset_t pinset)
{
#ifndef NUC_LOW
  uintptr_t base;
#endif
  int port;
  int pin;

  /* Decode the port and pin.  Use the port number to get the GPIO base
   * address.
   */

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT((unsigned)port <= NUC_GPIO_PORTE);

  /* Only the low density NUC100/120 chips support bit-band access to GPIO
   * pins.
   */

#ifdef NUC_LOW
  return (getreg32(NUC_PORT_PDIO(port, pin)) & PORT_MASK) != 0;
#else
  /* Get the base address of the GPIO port registers */

  base = NUC_GPIO_CTRL_BASE(port);

  /* Return the state of the selected pin */

  return (getreg32(base + NUC_GPIO_PIN_OFFSET) & (1 << pin)) != 0;
#endif
}
