/****************************************************************************
 *  arch/arm/src/kinetis/kinetis_gpio.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <arch/board/board.h>

#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "up_arch.h"
#include "up_internal.h"

#include "kinetis_memorymap.h"
#include "kinetis_internal.h"
#include "kinetis_port.h"
#include "kinetis_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int kinetis_configgpio(uint32_t cfgset)
{
  uintptr_t    base;
  uint32_t     regval;
  unsigned int port;
  unsigned int pin;
  unsigned int mode;

  /* Get the port number and pin number */

  port = (cfgset & _GPIO_PORT_MASK) >> _GPIO_PORT_SHIFT;
  pin  = (cfgset & _GPIO_PIN_MASK)  >> _GPIO_PIN_SHIFT;

  DEBUGASSERT(port < KINETIS_NPORTS);
  if (port < KINETIS_NPORTS)
    {
      /* Get the base address of PORT block for this port */

      base =  KINETIS_PORT_BASE(port);

      /* Get the port mode */

      mode = (cfgset & _GPIO_MODE_MASK)  >> _GPIO_MODE_SHIFT;

      /* Special case analog port mode.  In this case, not of the digital
       * options are applicable.
       */

      if (mode == _GPIO_MODE_ANALOG)
        {
          /* Set the analog mode with all digital options zeroed */

          regval = PORT_PCR_MUX_ANALOG | PORT_PCR_IRQC_DISABLED;
          putreg32(regval, base + KINETIS_PORT_PCR_OFFSET(pin));
        }
      else
        {
          /* Configure the digital pin options */

          regval = (mode << PORT_PCR_MUX_SHIFT);
          if ((cfgset & _GPIO_IO_MASK) == _GPIO_INPUT)
            {
              /* Handle input-only digital options */
              /* Check for pull-up or pull-down */


              if ((cfgset & _GPIO_INPUT_PULLMASK) == _GPIO_INPUT_PULLDOWN)
                {
                  regval |= PORT_PCR_PE;
                }
              else if ((cfgset & _GPIO_INPUT_PULLMASK) == _GPIO_INPUT_PULLUP)
                {
                  regval |= (PORT_PCR_PE | PORT_PCR_PS);
                }

# warning "Missing interrupt configuration logic"

            }
          else
            {
              /* Handle output-only digital options */
              /* Check for slow slew rate setting */

              if ((cfgset & _GPIO_OUTPUT_SLEW_MASK) == _GPIO_OUTPUT_SLOW)
                {
                  regval |= PORT_PCR_SRE;
                }

              /* Check for open drain output */

              if ((cfgset & _GPIO_OUTPUT_OD_MASK) == _GPIO_OUTPUT_OPENDRAIN)
                {
                  regval |= PORT_PCR_ODE;
                }
              
              /* Check for high drive output */

              if ((cfgset & _GPIO_OUTPUT_DRIVE_MASK) == _GPIO_OUTPUT_HIGHDRIVE)
                {
                  regval |= PORT_PCR_DSE;
                }
            }

          /* Check for passive filter enable.  Passive Filter configuration
           * is valid in all digital pin muxing modes.
           */

          if ((cfgset & GPIO_PASV_FILTER) != 0)
            {
              regval |= PORT_PCR_PFE;
            }

          /* Set the digital mode with all of the selected options */

          putreg32(regval, base + KINETIS_PORT_PCR_OFFSET(pin));

          /* Check for digital filter enable.  Digital Filter configuration
           * is valid in all digital pin muxing modes.
           */

          regval = getreg32(base + KINETIS_PORT_DFER_OFFSET);
          if ((cfgset & GPIO_DIG_FILTER) != 0)
            {
              regval |= (1 << pin);
            }
          else
            {
              regval &= ~(1 << pin);
            }
          putreg32(regval, base + KINETIS_PORT_DFER_OFFSET);

          /* Additional configuration for the case of Alternative 1 (GPIO) modes */

          if (mode == _GPIO_MODE_GPIO)
            {
              /* Set the GPIO port direction */

              base   = KINETIS_GPIO_BASE(port);
              regval = getreg32(base + KINETIS_GPIO_PDDR_OFFSET);
              if ((cfgset & _GPIO_IO_MASK) == _GPIO_INPUT)
                {
                  /* Select GPIO input */

                  regval &= ~(1 << pin);
                  putreg32(regval, base + KINETIS_GPIO_PDDR_OFFSET);
                }
              else /* if ((cfgset & _GPIO_IO_MASK) == _GPIO_OUTPUT) */
                {
                  /* Select GPIO input */

                  regval |= (1 << pin);
                  putreg32(regval, base + KINETIS_GPIO_PDDR_OFFSET);

                  /* Set the initial value of the GPIO output */

                  kinetis_gpiowrite(cfgset, ((cfgset & GPIO_OUTPUT_ONE) != 0));
                }
            }
        }

      return OK;
    }
  return -EINVAL;
}

/************************************************************************************
 * Name: kinetis_configfilter
 *
 * Description:
 *   Configure the digital filter associated with a port. The digital filter
 *   capabilities of the PORT module are available in all digital pin muxing modes.
 *
 * Input parmeters:
 *   port  - Port number.  See KINETIS_PORTn definitions in kinetis_port.h
 *   lpo   - true: Digital Filters are clocked by the bus clock
 *           false: Digital Filters are clocked by the 1 kHz LPO clock
 *   width - Filter Length
 *
 ************************************************************************************/

int kinetis_configfilter(unsigned int port, bool lpo, unsigned int width)
{
  uintptr_t base;
  uint32_t  regval;

  DEBUGASSERT(port < KINETIS_NPORTS);
  if (port < KINETIS_NPORTS)
    {
      /* Get the base address of PORT block for this port */

      base =  KINETIS_PORT_BASE(port);

      /* Select clocking */

      regval = (lpo ? PORT_DFCR_CS : 0);
      putreg32(regval, base + KINETIS_PORT_DFCR_OFFSET);

      /* Select the filter width */

      DEBUGASSERT(width < 32);
      putreg32(width, base + KINETIS_PORT_DFWR_OFFSET);
      return OK;
    }
  return -EINVAL;
}

/****************************************************************************
 * Name: kinetis_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void kinetis_gpiowrite(uint32_t pinset, bool value)
{
  uintptr_t    base;
  unsigned int port;
  unsigned int pin;

  DEBUGASSERT((pinset & _GPIO_IO_MASK) == _GPIO_OUTPUT);

  /* Get the port number and pin number */

  port = (pinset & _GPIO_PORT_MASK) >> _GPIO_PORT_SHIFT;
  pin  = (pinset & _GPIO_PIN_MASK)  >> _GPIO_PIN_SHIFT;

  DEBUGASSERT(port < KINETIS_NPORTS);
  if (port < KINETIS_NPORTS)
    {
      /* Get the base address of GPIO block for this port */

      base = KINETIS_GPIO_BASE(port);

      /* Set or clear the output */

      if (value)
        {
          putreg32((1 << pin), base + KINETIS_GPIO_PSOR_OFFSET);
        }
      else
        {
          putreg32((1 << pin), base + KINETIS_GPIO_PCOR_OFFSET);
        }
    }
}

/****************************************************************************
 * Name: kinetis_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool kinetis_gpioread(uint32_t pinset)
{
  uintptr_t    base;
  uint32_t     regval;
  unsigned int port;
  unsigned int pin;
  bool         ret = false;

  DEBUGASSERT((pinset & _GPIO_IO_MASK) == _GPIO_INPUT);

  /* Get the port number and pin number */

  port = (pinset & _GPIO_PORT_MASK) >> _GPIO_PORT_SHIFT;
  pin  = (pinset & _GPIO_PIN_MASK)  >> _GPIO_PIN_SHIFT;

  DEBUGASSERT(port < KINETIS_NPORTS);
  if (port < KINETIS_NPORTS)
    {
      /* Get the base address of GPIO block for this port */

      base = KINETIS_GPIO_BASE(port);

      /* return the state of the pin */

      regval = getreg32(base + KINETIS_GPIO_PDIR_OFFSET);
      ret    = ((regval & (1 << pin)) != 0);
    }
  return ret;
}

