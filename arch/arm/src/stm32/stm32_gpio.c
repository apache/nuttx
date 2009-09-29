/****************************************************************************
 * arch/arm/src/stm32/stm32_gpio.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <debug.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32 g_gpiobase[STM32_NGPIO] =
{
#if STM32_NGPIO > 0
	STM32_GPIOA_BASE,
#endif
#if STM32_NGPIO > 1
	STM32_GPIOB_BASE,
#endif
#if STM32_NGPIO > 2
	STM32_GPIOC_BASE,
#endif
#if STM32_NGPIO > 3
	STM32_GPIOD_BASE,
#endif
#if STM32_NGPIO > 4
	STM32_GPIOE_BASE,
#endif
#if STM32_NGPIO > 5
	STM32_GPIOF_BASE,
#endif
#if STM32_NGPIO > 6
	STM32_GPIOG_BASE,
#endif
};

#ifdef CONFIG_DEBUG
static const char g_portchar[8]   = { 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H' };
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int stm32_configgpio(uint32 cfgset)
{
  uint32 base;
  uint32 cr;
  uint32 regval;
  uint32 regaddr;
  unsigned int port;
  unsigned int pin;
  unsigned int pos;
  unsigned int modecnf;
  boolean output;
 
  /* Verify that this hardware supports the select GPIO port */

  port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < STM32_NGPIO)
    {
      /* Get the port base address */

      base = g_gpiobase[port];

      /* Get the pin number and select the port configuration register for that pin */

      pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      if (pin < 8)
        {
          cr  = base + STM32_GPIO_CRL_OFFSET;
          pos = pin;
        }
      else
        {
          cr  = base + STM32_GPIO_CRH_OFFSET;
          pos = pin - 8;
        }

      /* Input or output? */

      output = ((cfgset & GPIO_OUTPUT_PIN) != 0);

      /* Decode the mode and configuration */

      if (output)
        {
          modecnf = (cfgset & GPIO_MODE_MASK) >> GPIO_MODE_SHIFT;
        }
      else
        {
          modecnf = 0;
        }

      modecnf |= ((cfgset & GPIO_CNF_MASK) >> GPIO_CNF_SHIFT) << 2;
     
      /* Set the port configuration register */

      regval = getreg32(cr);
      regval &= ~(GPIO_CR_MODECNF_MASK(pos));
      regval |= (modecnf << GPIO_CR_MODECNF_SHIFT(pos));
      putreg32(regval, cr);

      /* Set or reset the corresponding BRR/BSRR bit */

      if (output)
        {
	        /* It is an output pin, we need to instantiate the initial
	         * pin output value
	         */

	        if ((cfgset & GPIO_OUTPUT_VALUE) != 0)
	          {
		          /* Use the BSRR register to set the output */

		          regaddr = base + STM32_GPIO_BSRR_OFFSET;
	          }
	        else
	          {
		          /* Use the BRR register to clear */

		          regaddr = base + STM32_GPIO_BRR_OFFSET;
	          }
        }
      else
        {
          /* It is an input pin... If it is pull-down or pull up,
           * then we need to set the ODR appropriately for that
           * function.
           */
        
          if ((cfgset & GPIO_MODE_MASK) == GPIO_CNF_INPULLUP)
            {
              /* Set the ODR bit (using BSRR) to one for the PULL-UP functionality */
            
	          regaddr = base + STM32_GPIO_BSRR_OFFSET;
            }
          else if ((cfgset & GPIO_MODE_MASK) == GPIO_CNF_INPULLDWN)
            {
              /* Clear the ODR bit (using BRR) to zero for the PULL-DOWN functionality */

	          regaddr = base + STM32_GPIO_BRR_OFFSET;
            }
            else
            {
              /* Neither... we can return early */

              return OK;
            }
        }

        regval = getreg32(regaddr);
        regval |= (1 << pin);
        putreg32(regval, regaddr);
        return OK;
    }
  return ERROR;
}

/****************************************************************************
 * Name: stm32_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void stm32_gpiowrite(uint32 pinset, boolean value)
{
  uint32 base;
  uint32 offset;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < STM32_NGPIO)
    {
      /* Get the port base address */

      base = g_gpiobase[port];

      /* Get the pin number  */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

      /* Set or clear the output on the pin */

      if (value)
        {
          offset = STM32_GPIO_BSRR_OFFSET;
        }
      else
          offset = STM32_GPIO_BRR_OFFSET;
        {
        }
      putreg32((1 << pin), base + offset);
    }
}

/****************************************************************************
 * Name: stm32_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

boolean stm32_gpioread(uint32 pinset)
{
  uint32 base;
  unsigned int port;
  unsigned int pin;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port < STM32_NGPIO)
    {
      /* Get the port base address */

      base = g_gpiobase[port];

      /* Get the pin number and return the input state of that pin */

      pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
      return ((getreg32(base + STM32_GPIO_IDR_OFFSET) & (1 << pin)) != 0);
    }
  return 0;
}

/****************************************************************************
 * Function:  stm32_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
int stm32_dumpgpio(uint32 pinset, const char *msg)
{
  irqstate_t   flags;
  uint32       base;
  unsigned int port;
  unsigned int pin;

  /* Get the base address associated with the GPIO port */

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin  = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  base = g_gpiobase[port];

  /* The following requires exclusive access to the GPIO registers */

  flags   = irqsave();
  lldbg("GPIO%c pinset: %08x base: %08x -- %s\n",
        g_portchar[port], pinset, base, msg);
  lldbg("  CR: %08x %08x IDR: %04x ODR: %04x LCKR: %04x\n",
        getreg32(base + STM32_GPIO_CRH_OFFSET), getreg32(base + STM32_GPIO_CRL_OFFSET),
        getreg32(base + STM32_GPIO_IDR_OFFSET), getreg32(base + STM32_GPIO_ODR_OFFSET),
        getreg32(base + STM32_GPIO_LCKR_OFFSET));
  lldbg("  EVCR: %02x MAPR: %08x CR: %04x %04x %04x %04x\n",
        getreg32(STM32_AFIO_EVCR), getreg32(STM32_AFIO_MAPR),
        getreg32(STM32_AFIO_EXTICR1), getreg32(STM32_AFIO_EXTICR2),
        getreg32(STM32_AFIO_EXTICR3), getreg32(STM32_AFIO_EXTICR4));
  irqrestore(flags);
  return OK;
}
#endif



