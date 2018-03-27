/****************************************************************************
 * arch/arm/src/nrf52/nrf52_gpio.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "chip/nrf52_gpio.h"
#include "nrf52_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_gpio_input
 *
 * Description:
 *   Configure a GPIO input pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf52_gpio_input(unsigned int port, unsigned int pin)
{
  /* Set as input */

  if (port == 0)
    {
      putreg32(1U << pin, NRF52_GPIO0_DIRCLR);
    }
}

/****************************************************************************
 * Name: nrf52_gpio_output
 *
 * Description:
 *   Configure a GPIO output pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf52_gpio_output(nrf52_pinset_t cfgset,
                                     unsigned int port, unsigned int pin)
{
  nrf52_gpio_write(cfgset, ((cfgset & GPIO_VALUE) != GPIO_VALUE_ZERO));

  /* Configure the pin as an output */

  if (port == 0)
    {
      putreg32(1U << pin, NRF52_GPIO0_DIRSET);
    }
}

/****************************************************************************
 * Name: nrf52_gpio_mode
 *
 * Description:
 *   Configure a GPIO mode based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void nrf52_gpio_mode(nrf52_pinset_t cfgset,
                                   unsigned int port, unsigned int pin)
{
  uint32_t mode;
  uint32_t regval;

  mode = cfgset & GPIO_MODE_MASK;
  regval = getreg32(NRF52_GPIO0_CNF(pin));

  regval &= NRF52_GPIO_CNF_PULL_MASK;

  if (mode == GPIO_PULLUP)
    {
      regval &= NRF52_GPIO_CNF_PULL_MASK;
      regval |= NRF52_GPIO_CNF_PULL_UP;
    }
  else if (mode == GPIO_PULLDOWN)
    {
      regval &= NRF52_GPIO_CNF_PULL_MASK;
      regval |= NRF52_GPIO_CNF_PULL_DOWN;
    }

  putreg32(regval, NRF52_GPIO0_CNF(pin));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int nrf52_gpio_config(nrf52_pinset_t cfgset)
{
  unsigned int port;
  unsigned int pin;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < NRF52_GPIO_NPORTS)
    {
      /* Get the pin number and select the port configuration register for
       * that pin.
       */

      pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* First, configure the port as a generic input so that we have a
       * known starting point and consistent behavior during the re-
       * configuration.
       */

      nrf52_gpio_input(port, pin);

      /* Set the mode bits */

      nrf52_gpio_mode(cfgset, port, pin);

      /* Handle according to pin function */

      switch (cfgset & GPIO_FUNC_MASK)
        {
        case GPIO_INPUT:   /* GPIO input pin */
          break;           /* Already configured */

#ifdef CONFIG_NRF52_GPIOIRQ
        case GPIO_INTFE:   /* GPIO interrupt falling edge */
        case GPIO_INTRE:   /* GPIO interrupt rising edge */
        case GPIO_INTBOTH: /* GPIO interrupt both edges */
        case GPIO_INTLOW:  /* GPIO interrupt low level */
        case GPIO_INTHIGH: /* GPIO interrupt high level */
          nrf52_gpio_interrupt(cfgset);
          break;
#endif

        case GPIO_OUTPUT:  /* GPIO outpout pin */
          nrf52_gpio_output(cfgset, port, pin);
          break;

        default:
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void nrf52_gpio_write(nrf52_pinset_t pinset, bool value)
{
  unsigned int pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  if (value)
    {
      putreg32(1 << pin, NRF52_GPIO0_OUTSET);
    }
  else
    {
      putreg32(1 << pin, NRF52_GPIO0_OUTCLR);
    }
}

/****************************************************************************
 * Name: nrf52_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool nrf52_gpio_read(nrf52_pinset_t pinset)
{
  uint32_t regval;
  unsigned int pin;

  pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  regval = getreg32(NRF52_GPIO0_IN);

  return (regval >> pin) & 1UL;
}
