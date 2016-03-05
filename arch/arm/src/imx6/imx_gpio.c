/****************************************************************************
 * arch/arm/src/imx6/imx_gpio.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "up_arch.h"
#include "imx_gpio.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_gpio_dirout
 ****************************************************************************/

static inline void imx_gpio_dirout(int port, int pin)
{
  uint32_t regval = getreg32(IMX_GPIO_GDIR(port));
  regval |= GPIO_PIN(pin);
  putreg32(regval, IMX_GPIO_GDIR(port));
}

/****************************************************************************
 * Name: imx_gpio_dirin
 ****************************************************************************/

static inline void imx_gpio_dirin(int port, int pin)
{
  uint32_t regval = getreg32(IMX_GPIO_GDIR(port));
  regval &= ~GPIO_PIN(pin);
  putreg32(regval, IMX_GPIO_GDIR(port));
}

/****************************************************************************
 * Name: imx_gpio_setoutput
 ****************************************************************************/

static inline void imx_gpio_setoutput(int port, int pin, bool value)
{
  uintptr_t regaddr = IMX_GPIO_DR(port);
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
 * Name: imx_gpio_getinput
 ****************************************************************************/

static inline bool imx_gpio_getinput(int port, int pin)
{
  uintptr_t regaddr = IMX_GPIO_DR(port);
  uint32_t regval;

  regval = getreg32(regaddr);
  return ((regval & GPIO_PIN(pin)) != 0);
}

/****************************************************************************
 * Name: imx_gpio_configinput
 ****************************************************************************/

static inline int imx_gpio_configinput(gpio_pinset_t pinset, int port, int pin)
{
  /* Configure pin as in input */

  imx_gpio_dirin(port, pin);

  /* Configure pin as a GPIO */
#warning Missing logic

  /* Configure pin pad settings */
#warning Missing logic

  return OK;
}

/****************************************************************************
 * Name: imx_gpio_configoutput
 ****************************************************************************/

static inline int imx_gpio_configoutput(gpio_pinset_t pinset, int port, int pin)
{
  bool value = ((pinset & GPIO_OUTPUT_ONE) != 0);

  /* Set the output value */

  imx_gpio_setoutput(port, pin, value);

  /* Convert the configured input GPIO to an output */

  imx_gpio_dirout(port, pin);
  return OK;
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description of the pin.
 *
 ****************************************************************************/

int imx_config_gpio(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  int ret;

  /* Configure the pin as an input initially to avoid any spurios outputs */

  flags = enter_critical_section();
  ret   = imx_gpio_configinput(pinset, port, pin);
  if (ret >= 0)
    {
      /* Was it really an output pin? */

      if ((pinset & GPIO_OUTPUT) != 0)
        {
          /* YES.. convert the input to an output */

          ret = imx_gpio_configoutput(pinset, port, pin);
        }
    }

  leave_critical_section(flags);
  return ret;
}

/************************************************************************************
 * Name: imx_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void imx_gpio_write(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  
  flags = enter_critical_section();
  imx_gpio_setoutput(port, pin, value);
  leave_critical_section(flags);
}

/************************************************************************************
 * Name: imx_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool imx_gpio_read(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  int pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  bool value;
  
  flags = enter_critical_section();
  value = imx_gpio_getinput(port, pin);
  leave_critical_section(flags);
  return value;
}
