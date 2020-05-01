/****************************************************************************
 *  arch/arm/src/kl/kl_pin.c
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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
#include <arch/board/board.h>

#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "kl_gpio.h"

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
 * Name: kl_configgpio
 *
 * Description:
 *   Configure a PIN based on bit-encoded description of the pin.  NOTE that
 *   DMA/interrupts are disabled at the initial PIN configuration.
 *
 ****************************************************************************/

int kl_configgpio(uint32_t cfgset)
{
  uintptr_t    base;
  uint32_t     regval;
  unsigned int port;
  unsigned int pin;
  unsigned int mode;

  /* Get the port number and pin number */

  port = (cfgset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (cfgset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < KL_NPORTS);
  if (port < KL_NPORTS)
    {
      /* Get the base address of PORT block for this port */

      base =  KL_PORT_BASE(port);

      /* Get the port mode */

      mode = (cfgset & _PIN_MODE_MASK)  >> _PIN_MODE_SHIFT;

      /* Special case analog port mode.  In this case, not of the digital
       * options are applicable.
       */

      if (mode == PIN_MODE_ANALOG)
        {
          /* Set the analog mode with all digital options zeroed */

          regval = PORT_PCR_MUX_ANALOG | PORT_PCR_IRQC_DISABLED;
          putreg32(regval, base + KL_PORT_PCR_OFFSET(pin));
        }
      else
        {
          /* Configure the digital pin options */

          regval = (mode << PORT_PCR_MUX_SHIFT);
          if ((cfgset & _PIN_IO_MASK) == _PIN_INPUT)
            {
              /* Handle input-only digital options */
              /* Check for pull-up or pull-down */

              if ((cfgset & _PIN_INPUT_PULLMASK) == _PIN_INPUT_PULLDOWN)
                {
                  regval |= PORT_PCR_PE;
                }
              else if ((cfgset & _PIN_INPUT_PULLMASK) == _PIN_INPUT_PULLUP)
                {
                  regval |= (PORT_PCR_PE | PORT_PCR_PS);
                }
            }
          else
            {
              /* Handle output-only digital options */
              /* Check for slow slew rate setting */

              if ((cfgset & _PIN_OUTPUT_SLEW_MASK) == _PIN_OUTPUT_SLOW)
                {
                  regval |= PORT_PCR_SRE;
                }

              /* Check for open drain output */

              if ((cfgset & _PIN_OUTPUT_OD_MASK) == _PIN_OUTPUT_OPENDRAIN)
                {
                  regval |= PORT_PCR_ODE;
                }

              /* Check for high drive output */

              if ((cfgset & _PIN_OUTPUT_DRIVE_MASK) == _PIN_OUTPUT_HIGHDRIVE)
                {
                  regval |= PORT_PCR_DSE;
                }
            }

          /* Check for passive filter enable.  Passive Filter configuration
           * is valid in all digital pin muxing modes.
           */

          if ((cfgset & PIN_PASV_FILTER) != 0)
            {
              regval |= PORT_PCR_PFE;
            }

          /* Set the digital mode with all of the selected options */

          putreg32(regval, base + KL_PORT_PCR_OFFSET(pin));

          /* Check for digital filter enable.  Digital Filter configuration
           * is valid in all digital pin muxing modes.
           */

          regval = getreg32(base + KL_PORT_DFER_OFFSET);
          if ((cfgset & PIN_DIG_FILTER) != 0)
            {
              regval |= (1 << pin);
            }
          else
            {
              regval &= ~(1 << pin);
            }
          putreg32(regval, base + KL_PORT_DFER_OFFSET);

          /* Additional configuration for the case of Alternative 1 (GPIO) modes */

          if (mode == PIN_MODE_GPIO)
            {
              /* Set the GPIO port direction */

              base   = KL_GPIO_BASE(port);
              regval = getreg32(base + KL_GPIO_PDDR_OFFSET);
              if ((cfgset & _PIN_IO_MASK) == _PIN_INPUT)
                {
                  /* Select GPIO input */

                  regval &= ~(1 << pin);
                  putreg32(regval, base + KL_GPIO_PDDR_OFFSET);
                }
              else /* if ((cfgset & _PIN_IO_MASK) == _PIN_OUTPUT) */
                {
                  /* Select GPIO input */

                  regval |= (1 << pin);
                  putreg32(regval, base + KL_GPIO_PDDR_OFFSET);

                  /* Set the initial value of the GPIO output */

                  kl_gpiowrite(cfgset, ((cfgset & GPIO_OUTPUT_ONE) != 0));
                }
            }
        }

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: kl_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void kl_gpiowrite(uint32_t pinset, bool value)
{
  uintptr_t    base;
  unsigned int port;
  unsigned int pin;

  DEBUGASSERT((pinset & _PIN_MODE_MASK) == _PIN_MODE_GPIO);
  DEBUGASSERT((pinset & _PIN_IO_MASK) == _PIN_OUTPUT);

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < KL_NPORTS);
  if (port < KL_NPORTS)
    {
      /* Get the base address of GPIO block for this port */

      base = KL_GPIO_BASE(port);

      /* Set or clear the output */

      if (value)
        {
          putreg32((1 << pin), base + KL_GPIO_PSOR_OFFSET);
        }
      else
        {
          putreg32((1 << pin), base + KL_GPIO_PCOR_OFFSET);
        }
    }
}

/****************************************************************************
 * Name: kl_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool kl_gpioread(uint32_t pinset)
{
  uintptr_t    base;
  uint32_t     regval;
  unsigned int port;
  unsigned int pin;
  bool         ret = false;

  DEBUGASSERT((pinset & _PIN_MODE_MASK) == _PIN_MODE_GPIO);
  DEBUGASSERT((pinset & _PIN_IO_MASK) == _PIN_INPUT);

  /* Get the port number and pin number */

  port = (pinset & _PIN_PORT_MASK) >> _PIN_PORT_SHIFT;
  pin  = (pinset & _PIN_MASK)      >> _PIN_SHIFT;

  DEBUGASSERT(port < KL_NPORTS);
  if (port < KL_NPORTS)
    {
      /* Get the base address of GPIO block for this port */

      base = KL_GPIO_BASE(port);

      /* return the state of the pin */

      regval = getreg32(base + KL_GPIO_PDIR_OFFSET);
      ret    = ((regval & (1 << pin)) != 0);
    }
  return ret;
}
