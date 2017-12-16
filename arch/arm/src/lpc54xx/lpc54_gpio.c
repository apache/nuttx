/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_gpio.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "chip/lpc54_iocon.h"
#include "chip/lpc54_gpio.h"
#include "lpc54_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default input pin configuration */

#define PORTPIN_MASK      (GPIO_PORT_MASK|GPIO_PIN_MASK)

/* Pin types */

#define PIN_TYPE_UNKNOWN  0
#define PIN_TYPED         (1 << 0)
#define PIN_TYPEI         (1 << 1)
#define PIN_TYPEA         (1 << 2)

/* Helpers */

#define GPIO_PORTPIN_MASKGPIO_PORTPIN_MASK (GPIO_PORT_MASK | GPIO_PIN_MASK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_typed_mask[LPC54_GPIO_NPORTS] =
{
  IOCON_PIO0_TYPED_MASK,
  IOCON_PIO1_TYPED_MASK,
  IOCON_PIO2_TYPED_MASK,
  IOCON_PIO3_TYPED_MASK,
  IOCON_PIO4_TYPED_MASK,
  IOCON_PIO5_TYPED_MASK,
};

static const uint32_t g_typei_mask[LPC54_GPIO_NPORTS] =
{
  IOCON_PIO0_TYPEI_MASK,
  IOCON_PIO1_TYPEI_MASK,
  IOCON_PIO2_TYPEI_MASK,
  IOCON_PIO3_TYPEI_MASK,
  IOCON_PIO4_TYPEI_MASK,
  IOCON_PIO5_TYPEI_MASK,
};

static const uint32_t g_typea_mask[LPC54_GPIO_NPORTS] =
{
  IOCON_PIO0_TYPEA_MASK,
  IOCON_PIO1_TYPEA_MASK,
  IOCON_PIO2_TYPEA_MASK,
  IOCON_PIO3_TYPEA_MASK,
  IOCON_PIO4_TYPEA_MASK,
  IOCON_PIO5_TYPEA_MASK,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_pintype
 *
 * Description:
 *   Get the LPC548x IOCON register mask.
 *
 *   Type D: FUNC MODE         INVERT DIGIMODE FILTEROFF SLEW     OD
 *   Type I: FUNC      I2CSLEW INVERT DIGIMODE FILTEROFF I2CDRIVE I2CFILTEROFF
 *   Type A: FUNC MODE         INVERT DIGIMODE FILTEROFF          OD
 *
 ****************************************************************************/

static uint8_t lpc54_pintype(unsigned int port, unsigned int pin)
{
  uint8_t pintype = 0;

  if ((g_typed_mask[port] & (1 << pin)) != 0)
    {
      pintype |= PIN_TYPED;
    }

  if ((g_typei_mask[port] & (1 << pin)) != 0)
    {
      pintype |= PIN_TYPEI;
    }

  if ((g_typea_mask[port] & (1 << pin)) != 0)
    {
      pintype |= PIN_TYPEA;
    }

  return pintype;
}

/****************************************************************************
 * Name: lpc54_setpinfunction
 *
 * Description:
 *   Select pin function.
 *
 ****************************************************************************/

static void lpc54_setpinfunction(unsigned int port, unsigned int pin,
                                 unsigned int value)
{
  uintptr_t regaddr;
  uint32_t regval;

  regaddr = LPC54_IOCON_PIO(port, pin);
  regval  = getreg32(regaddr);
  regval &= ~IOCON_FUNC_MASK;
  regval |= (value << IOCON_FUNC_SHIFT);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: lpc54_gpio_input
 *
 * Description:
 *   Configure a GPIO input pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void lpc54_gpio_input(unsigned int port, unsigned int pin)
{
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t pinmask = (1 << pin);

  /* Set as input */

  regaddr = LPC54_GPIO_DIR(port);
  regval  = getreg32(regaddr);
  regval &= ~pinmask;
  putreg32(regval, regaddr);

  /* Configure as GPIO */

  lpc54_setpinfunction(port, pin, IOCON_FUNC_GPIO);
}

/****************************************************************************
 * Name: lpc54_gpio_output
 *
 * Description:
 *   Configure a GPIO output pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline void lpc54_gpio_output(lpc54_pinset_t cfgset,
                                     unsigned int port, unsigned int pin)
{
  uintptr_t regaddr;
  uint32_t regval;

  /* Configure the pin as an output */

  regaddr = LPC54_GPIO_DIR(port);
  regval  = getreg32(regaddr);
  regval |= (1 << pin);
  putreg32(regval, regaddr);

  /* Set the initial value of the output.  Apparently this cannot be done
   * before cofiguring the pin as an output.  I don't see anyway to avoid
   * glitch.
   */

  lpc54_gpio_write(cfgset, ((cfgset & GPIO_VALUE) != GPIO_VALUE_ZERO));
}

/****************************************************************************
 * Name: lpc54_gpio_alternate
 *
 * Description:
 *   Configure a GPIO alternate function pin based on bit-encoded description
 *   of the pin.
 *
 ****************************************************************************/

static inline void lpc54_gpio_alternate(lpc54_pinset_t cfgset,
                                        unsigned int port, unsigned int pin,
                                        uint32_t alt)
{
  /* Select the alternate pin function */

  lpc54_setpinfunction(port, pin, alt);
}

/****************************************************************************
 * Name: lpc54_gpio_iocon
 *
 * Description:
 *   Configure the pin IOCON register.
 *
 ****************************************************************************/

static void lpc54_gpio_iocon(lpc54_pinset_t cfgset, unsigned int port,
                             unsigned int pin)
{
  uintptr_t regaddr;
  uint32_t iocon;
  uint8_t pintype;

  /* Configure pins for supported pin type(s):
   *
   *   Type D: FUNC MODE         INVERT DIGIMODE FILTEROFF SLEW     OD
   *   Type I: FUNC      I2CSLEW INVERT DIGIMODE FILTEROFF I2CDRIVE I2CFILTEROFF
   *   Type A: FUNC MODE         INVERT DIGIMODE FILTEROFF          OD
   */

  pintype = lpc54_pintype(port, pin);
  iocon   = IOCON_FUNC(IOCON_FUNC_GPIO);

  /* MODE: Type D and A only */

  if ((pintype & (PIN_TYPED | PIN_TYPEA)) != 0)
    {
      uint32_t mode = (cfgset & GPIO_MODE_MASK) >> GPIO_MODE_SHIFT;
      iocon |= mode << IOCON_MODE_SHIFT;
    }

  /* I2CSLEW:  Type I only */

  if ((pintype & PIN_TYPEI) != 0 && (cfgset & GPIO_I2CSLEW_MASK) != 0)
    {
      iocon |= IOCON_I2CSLEW_GPIO;
    }

  /* INVERT:  All types */

  if ((cfgset & GPIO_INVERT_MASK) != 0)
    {
      iocon |= IOCON_INVERT;
    }

  /* DIGIMODE:  All types */

  if ((cfgset & GPIO_DIGIMODE_MASK) != 0)
    {
      iocon |= IOCON_DIGIMODE_DIGITAL;
    }

  /* FILTEROFF:  All types */

  if ((cfgset & GPIO_FILTEROFF_MASK) != 0)
    {
      iocon |= IOCON_FILTEROFF_OFF;
    }

  /* SLEW:  Type D only */

  if ((pintype & PIN_TYPED) != 0 && (cfgset & GPIO_SLEW_MASK) != 0)
    {
      iocon |= IOCON_SLEW_FAST;
    }

  /* I2CDRIVE:  Type I only */

  if ((pintype & PIN_TYPEI) != 0 && (cfgset & GPIO_I2CDRIVE_MASK) != 0)
    {
      iocon |= IOCON_I2CDRIVE_HIGH;
    }

  /* OD: Type D and A only */

  if ((pintype & (PIN_TYPED | PIN_TYPEA)) != 0 &&
      (cfgset & GPIO_OD_MASK) != 0)
    {
      iocon |= IOCON_OD_OPENDRAIN;
    }

  /* I2CFILTEROFF:  Type I only */

  if ((pintype & PIN_TYPEI) != 0 && (cfgset & GPIO_I2CFILTEROFF_MASK) != 0)
    {
      iocon |= IOCON_I2CFILTEROFF_OFF;
    }

  /* Now write the IOCON settings */

  regaddr = LPC54_IOCON_PIO(port, pin);
  putreg32(iocon, regaddr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int lpc54_gpio_config(lpc54_pinset_t cfgset)
{
  unsigned int port;
  unsigned int pin;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < LPC54_GPIO_NPORTS)
    {
      /* Get the pin number and select the port configuration register for
       * that pin.
       */

      pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* First, configure the port as a generic input so that we have a
       * known starting point and consistent behavior during the re-
       * configuration.
       */

      lpc54_gpio_input(port, pin);

      /* Set the IOCON bits */

      lpc54_gpio_iocon(cfgset, port, pin);

      /* Handle according to pin function */

      switch (cfgset & GPIO_FUNC_MASK)
        {
        case GPIO_INPUT:   /* GPIO input pin */
          break;           /* Already configured */

#ifdef CONFIG_LPC54_GPIOIRQ
        case GPIO_INTFE:   /* GPIO interrupt falling edge */
        case GPIO_INTRE:   /* GPIO interrupt rising edge */
        case GPIO_INTBOTH: /* GPIO interrupt both edges */
        case GPIO_INTLOW:  /* GPIO interrupt low level */
        case GPIO_INTHIGH: /* GPIO interrupt high level */
          lpc54_gpio_interrupt(cfgset);
          break;
#endif

        case GPIO_OUTPUT:  /* GPIO outpout pin */
          lpc54_gpio_output(cfgset, port, pin);
          break;

        case GPIO_ALT1:    /* Alternate function 1 */
          lpc54_gpio_alternate(cfgset, port, pin, IOCON_FUNC_ALT1);
          break;

        case GPIO_ALT2:    /* Alternate function 2 */
          lpc54_gpio_alternate(cfgset, port, pin, IOCON_FUNC_ALT2);
          break;

        case GPIO_ALT3:    /* Alternate function 3 */
          lpc54_gpio_alternate(cfgset, port, pin, IOCON_FUNC_ALT3);
          break;

        case GPIO_ALT4:    /* Alternate function 4 */
          lpc54_gpio_alternate(cfgset, port, pin, IOCON_FUNC_ALT4);
          break;

        case GPIO_ALT5:    /* Alternate function 5 */
          lpc54_gpio_alternate(cfgset, port, pin, IOCON_FUNC_ALT5);
          break;

        case GPIO_ALT6:    /* Alternate function 6 */
          lpc54_gpio_alternate(cfgset, port, pin, IOCON_FUNC_ALT6);
          break;

        case GPIO_ALT7:    /* Alternate function 7 */
          lpc54_gpio_alternate(cfgset, port, pin, IOCON_FUNC_ALT7);
          break;

        default:
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: lpc54_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void lpc54_gpio_write(lpc54_pinset_t pinset, bool value)
{
  unsigned int portpin = pinset & PORTPIN_MASK;
  putreg8((uint32_t)value, LPC54_GPIO_B(portpin));
}

/****************************************************************************
 * Name: lpc54_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool lpc54_gpio_read(lpc54_pinset_t pinset)
{
  unsigned int portpin = pinset & PORTPIN_MASK;
  return (bool)getreg8(LPC54_GPIO_B(portpin));
}
