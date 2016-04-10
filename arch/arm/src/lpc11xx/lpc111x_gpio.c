/****************************************************************************
 * arch/arm/src/lpc11xx/lpc111x_gpio.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
#include "chip.h"
#include "chip/lpc111x_iocon.h"
#include "lpc11_gpio.h"

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
 * lpc11_gpiodbg.c
 */

/* We have to remember the configured interrupt setting.. PINs are not
 * actually set up to interrupt until the interrupt is enabled.
 */

#ifdef CONFIG_GPIO_IRQ
uint64_t g_intedge0;
uint64_t g_intedge2;
#endif

/* GPIO register base addresses */

const uint32_t g_gpiobase[GPIO_NPORTS] =
{
  LPC11_GPIO0_BASE,
  LPC11_GPIO1_BASE,
  LPC11_GPIO2_BASE,
  LPC11_GPIO3_BASE
};

/* Port 0 and Port 2 can provide a single interrupt for any combination of
 * port pins
 */

const uint32_t g_intbase[GPIO_NPORTS] =
{
};

/* Note: The IOCON offset is not linear. See User manual UM10398 Page 74 */
/* Note: The IOCON base is not linear. See User manual UM10398 Page 74 */

const uint8_t iocon_port0[IOCON_NPINS] =
{
  LPC11_IOCON_P0_0_OFFSET,
  LPC11_IOCON_P0_1_OFFSET,
  LPC11_IOCON_P0_2_OFFSET,
  LPC11_IOCON_P0_3_OFFSET,
  LPC11_IOCON_P0_4_OFFSET,
  LPC11_IOCON_P0_5_OFFSET,
  LPC11_IOCON_P0_6_OFFSET,
  LPC11_IOCON_P0_7_OFFSET,
  LPC11_IOCON_P0_8_OFFSET,
  LPC11_IOCON_P0_9_OFFSET,
  LPC11_IOCON_P0_10_OFFSET,
  LPC11_IOCON_P0_11_OFFSET
};

const uint8_t iocon_port1[IOCON_NPINS] =
{
  LPC11_IOCON_P1_0_OFFSET,
  LPC11_IOCON_P1_1_OFFSET,
  LPC11_IOCON_P1_2_OFFSET,
  LPC11_IOCON_P1_3_OFFSET,
  LPC11_IOCON_P1_4_OFFSET,
  LPC11_IOCON_P1_5_OFFSET,
  LPC11_IOCON_P1_6_OFFSET,
  LPC11_IOCON_P1_7_OFFSET,
  LPC11_IOCON_P1_8_OFFSET,
  LPC11_IOCON_P1_9_OFFSET,
  LPC11_IOCON_P1_10_OFFSET,
  LPC11_IOCON_P1_11_OFFSET
};

const uint8_t iocon_port2[IOCON_NPINS] =
{
  LPC11_IOCON_P2_0_OFFSET,
  LPC11_IOCON_P2_1_OFFSET,
  LPC11_IOCON_P2_2_OFFSET,
  LPC11_IOCON_P2_3_OFFSET,
  LPC11_IOCON_P2_4_OFFSET,
  LPC11_IOCON_P2_5_OFFSET,
  LPC11_IOCON_P2_6_OFFSET,
  LPC11_IOCON_P2_7_OFFSET,
  LPC11_IOCON_P2_8_OFFSET,
  LPC11_IOCON_P2_9_OFFSET,
  LPC11_IOCON_P2_10_OFFSET,
  LPC11_IOCON_P2_11_OFFSET
};

/* There is only IOCON_P3_[0-5] */
const uint8_t iocon_port3[IOCON_NPINS - 6] =
{
  LPC11_IOCON_P3_0_OFFSET,
  LPC11_IOCON_P3_1_OFFSET,
  LPC11_IOCON_P3_2_OFFSET,
  LPC11_IOCON_P3_3_OFFSET,
  LPC11_IOCON_P3_4_OFFSET,
  LPC11_IOCON_P3_5_OFFSET
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc11_pinfunc
 *
 * Description:
 *   Set the PIN function in the IOCON register.
 *
 ****************************************************************************/

static int lpc11_pinfunc(unsigned int port, unsigned int pin,
                         unsigned int value)
{
  const uint8_t *table = NULL;
  uint32_t regaddr;
  uint32_t regval;

  switch (port)
    {
    case 0:
      table = iocon_port0;
      break;
    case 1:
      table = iocon_port1;
      break;
    case 2:
      table = iocon_port2;
      break;
    case 3:
      table = iocon_port3;
      break;
   }

  regaddr = LPC11_IOCON_BASE + table[pin];
  if (regaddr != 0)
    {
      /* Set the requested value in the IOCON register */

      regval = getreg32(regaddr);
      regval &= ~(IOCON_FUNC_MASK);
      regval |= (value << IOCON_FUNC_SHIFT);
      putreg32(regval, regaddr);
      return OK;

    }

  return -EINVAL;
}

/****************************************************************************
 * Name: lpc11_pullup
 *
 * Description:
 *   Get the address of the PINMODE register corresponding to this port and
 *   pin number.
 *
 ****************************************************************************/

static int lpc11_pullup(lpc11_pinset_t cfgset, unsigned int port,
                        unsigned int pin)
{
  const uint8_t *table = NULL;
  uint32_t regaddr;
  uint32_t regval;
  uint32_t value;

  switch (cfgset & GPIO_PUMODE_MASK)
    {
    default:
    case GPIO_PULLUP:       /* Pull-up resistor enabled */
      value = IOCON_MODE_PU;
      break;

    case GPIO_REPEATER:     /* Repeater mode enabled */
      value = IOCON_MODE_RM;
      break;

    case GPIO_FLOAT:        /* Neither pull-up nor -down */
      value = IOCON_MODE_FLOAT;
      break;

    case GPIO_PULLDN:       /* Pull-down resistor enabled */
      value = IOCON_MODE_PD;
      break;
   }

  switch (port)
    {
    case 0:
      table = iocon_port0;
      break;
    case 1:
      table = iocon_port1;
      break;
    case 2:
      table = iocon_port2;
      break;
    case 3:
      table = iocon_port3;
      break;
   }


  /* Fetch the IOCON register address for this port/pin combination */

  regaddr = LPC11_IOCON_BASE + table[pin];
  if (regaddr != 0)
    {
      /* Set the requested value in the IOCON register */

      regval = getreg32(regaddr);
      regval &= ~(IOCON_MODE_MASK);
      regval |= (value << IOCON_MODE_SHIFT);
      putreg32(regval, regaddr);
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: lpc11_setintedge
 *
 * Description:
 *  Remember the configured interrupt edge.  We can't actually enable the
 *  the edge interrupts until the called calls IRQ enabled function.
 *
 ****************************************************************************/

#ifdef CONFIG_GPIO_IRQ
static void lpc11_setintedge(unsigned int port, unsigned int pin,
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

  /* Set the requested value in the IOCON register */

  shift     = pin << 1;
  *intedge &= ~((uint64_t)3     << shift);
  *intedge |=  ((uint64_t)value << shift);
}
#endif /* CONFIG_GPIO_IRQ */

/****************************************************************************
 * Name: lpc11_setopendrain
 *
 * Description:
 *   Set the ODMODE register for open drain mode
 *
 ****************************************************************************/

static void lpc11_setopendrain(unsigned int port, unsigned int pin)
{
  const uint8_t *table = NULL;
  uint32_t regaddr;
  uint32_t regval;

  switch (port)
    {
    case 0:
      table = iocon_port0;
      break;
    case 1:
      table = iocon_port1;
      break;
    case 2:
      table = iocon_port2;
      break;
    case 3:
      table = iocon_port3;
      break;
   }

  regaddr = LPC11_IOCON_BASE + table[pin];
  if (regaddr != 0)
    {
      /* Set the requested value in the IOCON register */

      regval = getreg32(regaddr);
      regval &= ~(IOCON_OD_MASK);
      regval |= (1 << IOCON_OD_SHIFT);
      putreg32(regval, regaddr);
    }
}

/****************************************************************************
 * Name: lpc11_clropendrain
 *
 * Description:
 *   Reset the ODMODE register to disable open drain mode
 *
 ****************************************************************************/

static void lpc11_clropendrain(unsigned int port, unsigned int pin)
{
  const uint8_t *table = NULL;
  uint32_t regaddr;
  uint32_t regval;

  switch (port)
    {
    case 0:
      table = iocon_port0;
      break;
    case 1:
      table = iocon_port1;
      break;
    case 2:
      table = iocon_port2;
      break;
    case 3:
      table = iocon_port3;
      break;
   }

  regaddr = LPC11_IOCON_BASE + table[pin];
  if (regaddr != 0)
    {
      /* Set the requested value in the IOCON register */

      regval = getreg32(regaddr);
      regval &= ~(1 << IOCON_OD_SHIFT);
      putreg32(regval, regaddr);
    }
}

/****************************************************************************
 * Name: lpc11_configinput
 *
 * Description:
 *   Configure a GPIO input pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int lpc11_configinput(lpc11_pinset_t cfgset, unsigned int port,
                                    unsigned int pin)
{
  uint32_t regval;
  uint32_t gpiobase;
  uint32_t intbase;
  uint32_t pinmask = (1 << pin);

  /* Set up GPIO registers */

  gpiobase = g_gpiobase[port];

  /* Set as input */

  regval = getreg32(gpiobase + LPC11_GPIO_DIR_OFFSET);
  regval &= ~pinmask;
  putreg32(regval, gpiobase + LPC11_GPIO_DIR_OFFSET);

  /* Set up interrupt registers */

  intbase = g_intbase[port];
  if (intbase != 0)
    {
      /* Disable any rising edge interrupts */

      regval = getreg32(intbase + LPC11_GPIO_DIR_OFFSET);
      regval &= ~pinmask;
      putreg32(regval, intbase + LPC11_GPIO_DIR_OFFSET);

      /* Disable any falling edge interrupts */

      regval = getreg32(intbase + LPC11_GPIO_DIR_OFFSET);
      regval &= ~pinmask;
      putreg32(regval, intbase + LPC11_GPIO_DIR_OFFSET);

      /* Forget about any falling/rising edge interrupt enabled */

#ifdef CONFIG_GPIO_IRQ
      lpc11_setintedge(port, pin, 0);
#endif
    }

  /* Set up IOCON registers */
  /* Configure as GPIO */

  lpc11_pinfunc(port, pin, IOCON_FUNC_GPIO);

  /* Set pull-up mode */

  lpc11_pullup(cfgset, port, pin);

  /* Open drain only applies to outputs */

  lpc11_clropendrain(port, pin);

  return OK;
}

/****************************************************************************
 * Name: lpc11_configinterrupt
 *
 * Description:
 *   Configure a GPIO interrupt pin based on bit-encoded description of the
 *   pin.
 *
 ****************************************************************************/

static inline int lpc11_configinterrupt(lpc11_pinset_t cfgset, unsigned int port,
                                        unsigned int pin)
{
  /* First, configure the port as a generic input so that we have a known
   * starting point and consistent behavior during the re-configuration.
   */

  (void)lpc11_configinput(cfgset, port, pin);

  /* Then just remember the rising/falling edge interrupt enabled */

  DEBUGASSERT(port == 0 || port == 2);
#ifdef CONFIG_GPIO_IRQ
  lpc11_setintedge(port, pin, (cfgset & GPIO_EDGE_MASK) >> GPIO_EDGE_SHIFT);
#endif
  return OK;
}

/****************************************************************************
 * Name: lpc11_configoutput
 *
 * Description:
 *   Configure a GPIO output pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

static inline int lpc11_configoutput(lpc11_pinset_t cfgset, unsigned int port,
                                     unsigned int pin)
{
  uint32_t gpiobase;
  uint32_t regval;

  /* First, configure the port as a generic input so that we have a known
   * starting point and consistent behavior during the re-configuration.
   */

  (void)lpc11_configinput(DEFAULT_INPUT, port, pin);

  /* Now, reconfigure the pin as an output */

  gpiobase = g_gpiobase[port];
  regval  = getreg32(gpiobase + LPC11_GPIO_DIR_OFFSET);
  regval |= (1 << pin);
  putreg32(regval, gpiobase + LPC11_GPIO_DIR_OFFSET);

  /* Check for open drain output */

  if ((cfgset & GPIO_OPEN_DRAIN) != 0)
    {
      /* Set pull-up mode.  This normally only applies to input pins, but does have
       * meaning if the port is an open drain output.
       */

      lpc11_pullup(cfgset, port, pin);

      /* Select open drain output */

      lpc11_setopendrain(port, pin);
    }

  /* Set the initial value of the output */

  lpc11_gpiowrite(cfgset, ((cfgset & GPIO_VALUE) != GPIO_VALUE_ZERO));

  return OK;
}

/****************************************************************************
 * Name: lpc11_configalternate
 *
 * Description:
 *   Configure a GPIO alternate function pin based on bit-encoded description
 *   of the pin.
 *
 ****************************************************************************/

static int lpc11_configalternate(lpc11_pinset_t cfgset, unsigned int port,
                                 unsigned int pin, uint32_t alt)
{
  /* First, configure the port as an input so that we have a known
   * starting point and consistent behavior during the re-configuration.
   */

  (void)lpc11_configinput(DEFAULT_INPUT, port, pin);

  /* Set up IOCON registers */
  /* Configure as GPIO */

  lpc11_pinfunc(port, pin, alt);

  /* Set pull-up mode */

  lpc11_pullup(cfgset, port, pin);

  /* Check for open drain output */

  if ((cfgset & GPIO_OPEN_DRAIN) != 0)
    {
      /* Select open drain output */

      lpc11_setopendrain(port, pin);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc11_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int lpc11_configgpio(lpc11_pinset_t cfgset)
{
  unsigned int port;
  unsigned int pin;
  int ret = -EINVAL;

  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < GPIO_NPORTS)
    {
      /* Get the pin number and select the port configuration register for
       * that pin.
       */

      pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* Handle according to pin function */

      switch (cfgset & GPIO_FUNC_MASK)
        {
        case GPIO_INPUT:   /* GPIO input pin */
          ret = lpc11_configinput(cfgset, port, pin);
          break;

        case GPIO_INTFE:   /* GPIO interrupt falling edge */
        case GPIO_INTRE:   /* GPIO interrupt rising edge */
        case GPIO_INTBOTH: /* GPIO interrupt both edges */
          ret = lpc11_configinterrupt(cfgset, port, pin);
          break;

        case GPIO_OUTPUT:  /* GPIO outpout pin */
          ret = lpc11_configoutput(cfgset, port, pin);
          break;

        case GPIO_ALT1:    /* Alternate function 1 */
          ret = lpc11_configalternate(cfgset, port, pin, IOCON_FUNC_ALT1);
          break;

        case GPIO_ALT2:    /* Alternate function 2 */
          ret = lpc11_configalternate(cfgset, port, pin, IOCON_FUNC_ALT2);
          break;

        case GPIO_ALT3:    /* Alternate function 3 */
          ret = lpc11_configalternate(cfgset, port, pin, IOCON_FUNC_ALT3);
          break;

        default:
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: lpc11_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void lpc11_gpiowrite(lpc11_pinset_t pinset, bool value)
{
  uint32_t gpiobase;
  uint32_t offset;
  uint32_t regval;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < GPIO_NPORTS)
    {
      /* Get the port base address */

      gpiobase = g_gpiobase[port];

      /* Get the pin number  */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* Set or clear the output on the pin */

      offset = LPC11_GPIO_DATA_OFFSET;

      regval = getreg32(gpiobase + offset);
      regval &= ~(1 << pin);
      regval |= (value << pin);
      putreg32(regval, gpiobase + offset);
    }
}

/****************************************************************************
 * Name: lpc11_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool lpc11_gpioread(lpc11_pinset_t pinset)
{
  uint32_t gpiobase;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < GPIO_NPORTS)
    {
      /* Get the port base address */

      gpiobase = g_gpiobase[port];

      /* Get the pin number and return the input state of that pin */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      return ((getreg32(gpiobase + LPC11_GPIO_DATA_OFFSET) & (1 << pin)) != 0);
    }

  return false;
}
