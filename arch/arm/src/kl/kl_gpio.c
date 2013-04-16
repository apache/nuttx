/****************************************************************************
 * arch/arm/src/kl/kl_gpio.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <arch/kl/chip.h>

#include "up_arch.h"

#include "chip.h"
#include "chip/kl_gpio.h"

#include "kl_gpio.h"

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
 * Name: kl_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with kl_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returns:
 *   OK on success
 *   ERROR on invalid port, or when pin is locked as ALT function.
 *
 ****************************************************************************/

int kl_configgpio(gpio_cfgset_t cfgset)
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

  DEBUGASSERT((unsigned)port <= KL_GPIO_PORTE);
  base = KL_GPIO_CTRL_BASE(port);

  /* Set the the GPIO PMD register */

  regaddr = base + KL_GPIO_PMD_OFFSET;
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

  regaddr = base + KL_GPIO_OFFD_OFFSET;
  regval  = getreg32(regaddr);
  regval &= ~GPIO_OFFD(pin);

  if ((cfgset & GPIO_ANALOG) != 0)
    {
      regval |= GPIO_OFFD(pin);
    }
 
  putreg32(regval, regaddr);

  /* Check if we need to enable debouncing */

  regaddr = base + KL_GPIO_DBEN_OFFSET;
  regval  = getreg32(regaddr);
  regval &= ~GPIO_DBEN(pin);

  if ((cfgset & GPIO_DEBOUNCE) != 0)
    {
      regval |= GPIO_DBEN(pin);
    }
 
  putreg32(regval, regaddr);

  /* Configure interrupting pins */

  isrc = getreg32(base + KL_GPIO_ISRC_OFFSET);
  isrc &= ~GPIO_ISRC(pin);

  imd  = getreg32(base + KL_GPIO_IMD_OFFSET);
  imd &= ~GPIO_IMD(pin);

  ien  = getreg32(base + KL_GPIO_IEN_OFFSET);
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

  putreg32(ien, base + KL_GPIO_IEN_OFFSET);
  putreg32(imd, base + KL_GPIO_IMD_OFFSET);
  putreg32(isrc, base + KL_GPIO_ISRC_OFFSET);

  /* If the pin is an output, set the initial output value */

  if ((cfgset & GPIO_MODE_MASK) == GPIO_OUTPUT)
    {
      kl_gpiowrite(cfgset, (cfgset & GPIO_OUTPUT_SET) != 0);
    }

  return 0;
}

/****************************************************************************
 * Name: kl_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void kl_gpiowrite(gpio_cfgset_t pinset, bool value)
{
#ifndef KL_LOW
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

  DEBUGASSERT((unsigned)port <= KL_GPIO_PORTE);

  /* Only the low density K25Z100/120 chips support bit-band access to GPIO
   * pins.
   */

#ifdef KL_LOW
  putreg32((uint32_t)value, KL_PORT_PDIO(port, pin));
#else
  /* Get the base address of the GPIO port registers */

  base = KL_GPIO_CTRL_BASE(port);

  /* Disable interrupts -- the following operations must be atomic */

  flags = irqsave();

  /* Allow writing only to the selected pin in the DOUT register */

  putreg32(~(1 << pin), base + KL_GPIO_DMASK_OFFSET);

  /* Set the pin to the selected value and re-enable interrupts */

  putreg32(((uint32_t)value << pin), base + KL_GPIO_DOUT_OFFSET);
  irqrestore(flags);
#endif
}

/****************************************************************************
 * Name: kl_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool kl_gpioread(gpio_cfgset_t pinset)
{
#ifndef KL_LOW
  uintptr_t base;
#endif
  int port;
  int pin;

  /* Decode the port and pin.  Use the port number to get the GPIO base
   * address.
   */

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  DEBUGASSERT((unsigned)port <= KL_GPIO_PORTE);

  /* Only the low density K25Z100/120 chips support bit-band access to GPIO
   * pins.
   */

#ifdef KL_LOW
  return (getreg32(KL_PORT_PDIO(port, pin)) & PORT_MASK) != 0;
#else
  /* Get the base address of the GPIO port registers */

  base = KL_GPIO_CTRL_BASE(port);

  /* Return the state of the selected pin */

 return (getreg32(base + KL_GPIO_PIN_OFFSET) & (1 << pin)) != 0;
#endif
}
