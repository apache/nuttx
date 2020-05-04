/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc176x_gpio.c
 *
 *   Copyright (C) 2010-2011, 2013 Gregory Nutt. All rights reserved.
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

#include "arm_arch.h"
#include "chip.h"
#include "lpc17_40_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default input pin configuration */

#define DEFAULT_INPUT (GPIO_INPUT|GPIO_PULLUP)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* These tables have global scope because they are also used in
 * lpc17_40_gpiodbg.c
 */

/* We have to remember the configured interrupt setting.. PINs are not
 * actually set up to interrupt until the interrupt is enabled.
 */

#ifdef CONFIG_LPC17_40_GPIOIRQ
uint64_t g_intedge0;
uint64_t g_intedge2;
#endif

/* FIO register base addresses */

const uint32_t g_fiobase[GPIO_NPORTS] =
{
  LPC17_40_FIO0_BASE,
  LPC17_40_FIO1_BASE,
  LPC17_40_FIO2_BASE,
  LPC17_40_FIO3_BASE,
  LPC17_40_FIO4_BASE
#if GPIO_NPORTS > 5
  , LPC17_40_FIO5_BASE
#endif
};

/* Port 0 and Port 2 can provide a single interrupt for any combination of
 * port pins
 */

const uint32_t g_intbase[GPIO_NPORTS] =
{
  LPC17_40_GPIOINT0_BASE,
  0,
  LPC17_40_GPIOINT2_BASE,
  0,
  0
#if GPIO_NPORTS > 5
  , 0
#endif
};

const uint32_t g_lopinsel[GPIO_NPORTS] =
{
  LPC17_40_PINCONN_PINSEL0,
  LPC17_40_PINCONN_PINSEL2,
  LPC17_40_PINCONN_PINSEL4,
  0,
  0
#if GPIO_NPORTS > 5
  , 0
#endif
};

const uint32_t g_hipinsel[GPIO_NPORTS] =
{
  LPC17_40_PINCONN_PINSEL1,
  LPC17_40_PINCONN_PINSEL3,
  0,
  LPC17_40_PINCONN_PINSEL7,
  LPC17_40_PINCONN_PINSEL9
#if GPIO_NPORTS > 5
  , 0
#endif
};

const uint32_t g_lopinmode[GPIO_NPORTS] =
{
  LPC17_40_PINCONN_PINMODE0,
  LPC17_40_PINCONN_PINMODE2,
  LPC17_40_PINCONN_PINMODE4,
  0,
  0
#if GPIO_NPORTS > 5
  , 0
#endif
};

const uint32_t g_hipinmode[GPIO_NPORTS] =
{
  LPC17_40_PINCONN_PINMODE1,
  LPC17_40_PINCONN_PINMODE3,
  0,
  LPC17_40_PINCONN_PINMODE7,
  LPC17_40_PINCONN_PINMODE9
#if GPIO_NPORTS > 5
  , 0
#endif
};

const uint32_t g_odmode[GPIO_NPORTS] =
{
  LPC17_40_PINCONN_ODMODE0,
  LPC17_40_PINCONN_ODMODE1,
  LPC17_40_PINCONN_ODMODE2,
  LPC17_40_PINCONN_ODMODE3,
  LPC17_40_PINCONN_ODMODE4
#if GPIO_NPORTS > 5
  , 0
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_pinsel
 *
 * Description:
 *   Get the address of the PINSEL register corresponding to this port and
 *   pin number.
 *
 ****************************************************************************/

static int lpc17_40_pinsel(unsigned int port, unsigned int pin, unsigned int value)
{
  const uint32_t *table;
  uint32_t regaddr;
  uint32_t regval;
  unsigned int shift;

  /* Which table do we use */

  if (pin < 16)
    {
      table = g_lopinsel;
      shift = PINCONN_PINSELL_SHIFT(pin);
    }
  else
    {
      table  = g_hipinsel;
      shift = PINCONN_PINSELH_SHIFT(pin);
    }

  /* Fetch the PINSEL register address for this port/pin combination */

  regaddr = table[port];
  if (regaddr != 0)
    {
      /* Set the requested value in the PINSEL register */

      regval = getreg32(regaddr);
      regval &= ~(PINCONN_PINSEL_MASK << shift);
      regval |= (value << shift);
      putreg32(regval, regaddr);
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: lpc17_40_pullup
 *
 * Description:
 *   Get the address of the PINMODE register corresponding to this port and
 *   pin number.
 *
 ****************************************************************************/

static int lpc17_40_pullup(lpc17_40_pinset_t cfgset, unsigned int port,
                        unsigned int pin)
{
  const uint32_t *table;
  uint32_t regaddr;
  uint32_t regval;
  uint32_t value;
  unsigned int shift;

  switch (cfgset & GPIO_PUMODE_MASK)
    {
    default:
    case GPIO_PULLUP:       /* Pull-up resistor enabled */
      value = PINCONN_PINMODE_PU;
      break;

    case GPIO_REPEATER:     /* Repeater mode enabled */
      value = PINCONN_PINMODE_RM;
      break;

    case GPIO_FLOAT:        /* Neither pull-up nor -down */
      value = PINCONN_PINMODE_FLOAT;
      break;

    case GPIO_PULLDN:       /* Pull-down resistor enabled */
      value = PINCONN_PINMODE_PD;
      break;
   }

  /* Which table do we use */

  if (pin < 16)
    {
      table = g_lopinmode;
      shift = PINCONN_PINMODEL_SHIFT(pin);
    }
  else
    {
      table  = g_hipinmode;
      shift = PINCONN_PINMODEH_SHIFT(pin);
    }

  /* Fetch the PINSEL register address for this port/pin combination */

  regaddr = table[port];
  if (regaddr != 0)
    {
      /* Set the requested value in the PINSEL register */

      regval = getreg32(regaddr);
      regval &= ~(PINCONN_PINMODE_MASK << shift);
      regval |= (value << shift);
      putreg32(regval, regaddr);
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: lpc17_40_setintedge
 *
 * Description:
 *  Remember the configured interrupt edge.  We can't actually enable the
 *  the edge interrupts until the called calls IRQ enabled function.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_GPIOIRQ
static void lpc17_40_setintedge(unsigned int port, unsigned int pin,
                             unsigned int value)
{
  uint64_t *intedge;
  unsigned int shift;

  /* Which word to we use? */

  if (port == 0)
    {
      intedge = &g_intedge0;
    }
  else if (port == 2)
    {
      intedge  = &g_intedge2;
    }
  else
    {
      return;
    }

  /* Set the requested value in the PINSEL register */

  shift     = pin << 1;
  *intedge &= ~((uint64_t)3     << shift);
  *intedge |=  ((uint64_t)value << shift);
}
#endif /* CONFIG_LPC17_40_GPIOIRQ */

/****************************************************************************
 * Name: lpc17_40_setopendrain
 *
 * Description:
 *   Set the ODMODE register for open drain mode
 *
 ****************************************************************************/

static void lpc17_40_setopendrain(unsigned int port, unsigned int pin)
{
  uint32_t regaddr;
  uint32_t regval;

  regaddr = g_odmode[port];
  regval  = getreg32(regaddr);
  regval |= (1 << pin);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: lpc17_40_clropendrain
 *
 * Description:
 *   Reset the ODMODE register to disable open drain mode
 *
 ****************************************************************************/

static void lpc17_40_clropendrain(unsigned int port, unsigned int pin)
{
  uint32_t regaddr;
  uint32_t regval;

  regaddr = g_odmode[port];
  regval  = getreg32(regaddr);
  regval &= ~(1 << pin);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: lpc17_40_configinput
 *
 * Description:
 *   Configure a GPIO inpue pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int lpc17_40_configinput(lpc17_40_pinset_t cfgset, unsigned int port, unsigned int pin)
{
  uint32_t regval;
  uint32_t fiobase;
  uint32_t intbase;
  uint32_t pinmask = (1 << pin);

  /* Set up FIO registers */

  fiobase = g_fiobase[port];

  /* Set as input */

  regval = getreg32(fiobase + LPC17_40_FIO_DIR_OFFSET);
  regval &= ~pinmask;
  putreg32(regval, fiobase + LPC17_40_FIO_DIR_OFFSET);

  /* Set up interrupt registers */

  intbase = g_intbase[port];
  if (intbase != 0)
    {
      /* Disable any rising edge interrupts */

      regval = getreg32(intbase + LPC17_40_GPIOINT_INTENR_OFFSET);
      regval &= ~pinmask;
      putreg32(regval, intbase + LPC17_40_GPIOINT_INTENR_OFFSET);

      /* Disable any falling edge interrupts */

      regval = getreg32(intbase + LPC17_40_GPIOINT_INTENF_OFFSET);
      regval &= ~pinmask;
      putreg32(regval, intbase + LPC17_40_GPIOINT_INTENF_OFFSET);

      /* Forget about any falling/rising edge interrupt enabled */

#ifdef CONFIG_LPC17_40_GPIOIRQ
      lpc17_40_setintedge(port, pin, 0);
#endif
    }

  /* Set up PINSEL registers */
  /* Configure as GPIO */

  lpc17_40_pinsel(port, pin, PINCONN_PINSEL_GPIO);

  /* Set pull-up mode */

  lpc17_40_pullup(cfgset, port, pin);

  /* Open drain only applies to outputs */

  lpc17_40_clropendrain(port, pin);

  return OK;
}

/****************************************************************************
 * Name: lpc17_40_configinterrupt
 *
 * Description:
 *   Configure a GPIO interrupt pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int lpc17_40_configinterrupt(lpc17_40_pinset_t cfgset, unsigned int port,
                                        unsigned int pin)
{
  /* First, configure the port as a generic input so that we have a known
   * starting point and consistent behavior during the re-configuration.
   */

  lpc17_40_configinput(cfgset, port, pin);

  /* Then just remember the rising/falling edge interrupt enabled */

  DEBUGASSERT(port == 0 || port == 2);
#ifdef CONFIG_LPC17_40_GPIOIRQ
  lpc17_40_setintedge(port, pin, (cfgset & GPIO_EDGE_MASK) >> GPIO_EDGE_SHIFT);
#endif
  return OK;
}

/****************************************************************************
 * Name: lpc17_40_configoutput
 *
 * Description:
 *   Configure a GPIO output pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int lpc17_40_configoutput(lpc17_40_pinset_t cfgset, unsigned int port,
                                     unsigned int pin)
{
  uint32_t fiobase;
  uint32_t regval;

  /* First, configure the port as a generic input so that we have a known
   * starting point and consistent behavior during the re-configuration.
   */

  lpc17_40_configinput(DEFAULT_INPUT, port, pin);

  /* Check for open drain output */

  if ((cfgset & GPIO_OPEN_DRAIN) != 0)
    {
      /* Set pull-up mode.  This normally only applies to input pins, but does have
       * meaning if the port is an open drain output.
       */

      lpc17_40_pullup(cfgset, port, pin);

      /* Select open drain output */

      lpc17_40_setopendrain(port, pin);
    }

  /* Set the initial value of the output */

  lpc17_40_gpiowrite(cfgset, ((cfgset & GPIO_VALUE) != GPIO_VALUE_ZERO));

  /* Now, reconfigure the pin as an output */

  fiobase = g_fiobase[port];
  regval  = getreg32(fiobase + LPC17_40_FIO_DIR_OFFSET);
  regval |= (1 << pin);
  putreg32(regval, fiobase + LPC17_40_FIO_DIR_OFFSET);


  return OK;
}

/****************************************************************************
 * Name: lpc17_40_configalternate
 *
 * Description:
 *   Configure a GPIO alternate function pin based on bit-encoded description
 *   of the pin.
 *
 ****************************************************************************/

static int lpc17_40_configalternate(lpc17_40_pinset_t cfgset, unsigned int port,
                                 unsigned int pin, uint32_t alt)
{
  /* First, configure the port as an input so that we have a known
   * starting point and consistent behavior during the re-configuration.
   */

  lpc17_40_configinput(DEFAULT_INPUT, port, pin);

  /* Set up PINSEL registers */
  /* Configure as GPIO */

  lpc17_40_pinsel(port, pin, alt);

  /* Set pull-up mode */

  lpc17_40_pullup(cfgset, port, pin);

  /* Check for open drain output */

  if ((cfgset & GPIO_OPEN_DRAIN) != 0)
    {
      /* Select open drain output */

      lpc17_40_setopendrain(port, pin);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int lpc17_40_configgpio(lpc17_40_pinset_t cfgset)
{
  unsigned int port;
  unsigned int pin;
  int ret = -EINVAL;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < GPIO_NPORTS)
    {
      /* Get the pin number and select the port configuration register for that pin */

      pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* Handle according to pin function */

      switch (cfgset & GPIO_FUNC_MASK)
        {
        case GPIO_INPUT:   /* GPIO input pin */
          ret = lpc17_40_configinput(cfgset, port, pin);
          break;

        case GPIO_INTFE:   /* GPIO interrupt falling edge */
        case GPIO_INTRE:   /* GPIO interrupt rising edge */
        case GPIO_INTBOTH: /* GPIO interrupt both edges */
          ret = lpc17_40_configinterrupt(cfgset, port, pin);
          break;

        case GPIO_OUTPUT:  /* GPIO outpout pin */
          ret = lpc17_40_configoutput(cfgset, port, pin);
          break;

        case GPIO_ALT1:    /* Alternate function 1 */
          ret = lpc17_40_configalternate(cfgset, port, pin, PINCONN_PINSEL_ALT1);
          break;

        case GPIO_ALT2:    /* Alternate function 2 */
          ret = lpc17_40_configalternate(cfgset, port, pin, PINCONN_PINSEL_ALT2);
          break;

        case GPIO_ALT3:    /* Alternate function 3 */
          ret = lpc17_40_configalternate(cfgset, port, pin, PINCONN_PINSEL_ALT3);
          break;

        default:
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: lpc17_40_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void lpc17_40_gpiowrite(lpc17_40_pinset_t pinset, bool value)
{
  uint32_t fiobase;
  uint32_t offset;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < GPIO_NPORTS)
    {
      /* Get the port base address */

      fiobase = g_fiobase[port];

      /* Get the pin number  */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* Set or clear the output on the pin */

      if (value)
        {
          offset = LPC17_40_FIO_SET_OFFSET;
        }
      else
        {
          offset = LPC17_40_FIO_CLR_OFFSET;
        }

      putreg32((1 << pin), fiobase + offset);
    }
}

/****************************************************************************
 * Name: lpc17_40_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool lpc17_40_gpioread(lpc17_40_pinset_t pinset)
{
  uint32_t fiobase;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < GPIO_NPORTS)
    {
      /* Get the port base address */

      fiobase = g_fiobase[port];

      /* Get the pin number and return the input state of that pin */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      return ((getreg32(fiobase + LPC17_40_FIO_PIN_OFFSET) & (1 << pin)) != 0);
    }

  return false;
}
