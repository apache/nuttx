/****************************************************************************
 * arch/arm/src/samdl/saml_glck.c
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

#include <stdint.h>
#include <assert.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "sam_gclk.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

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
 * Name: sam_gclck_waitsyncbusy
 *
 * Description:
 *   What until the SYNCBUSY bit is cleared.  The SYNCBUSY bit was set when
 *   the synchronization of registers between clock domains is started.  The
 *   SYNCBUSY bit is cleared when the synchronization of registers between
 *   the clock domains is complete.
 *
 * Input Parameters:
 *   glck - GCLK clock index
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_gclck_waitsyncbusy(uint8_t gclk)
{
  uintptr_t gclkbit = GCLK_SYNCHBUSY_GENCTRL(gclk);
  while ((getreg8(SAM_GCLK_SYNCHBUSY) & gclkbit) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_gclk_config
 *
 * Description:
 *   Configure a single GCLK(s) based on settings in the config structure.
 *
 * Input Parameters:
 *   config - An instance of struct sam_gclkconfig describing the GCLK
 *            configuration.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_gclk_config(FAR const struct sam_gclkconfig_s *config)
{
  irqstate_t flags;
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t genctrl;

  /* Select the requested source clock for the generator */

  genctrl = ((uint32_t)config->clksrc << GCLK_GENCTRL_SRC_SHIFT);

#if 0 /* Not yet supported */
  /* Configure the clock to be either high or low when disabled */

  if (config->level)
    {
      genctrl |= GCLK_GENCTRL_OOV;
    }
#endif

  /* Configure if the clock output to I/O pin should be enabled */

  if (config->output)
    {
      genctrl |= GCLK_GENCTRL_OE;
    }

  /* Set the prescaler division factor */

  if (config->prescaler > 1)
    {
      /* Check if division is a power of two */

      if (((config->prescaler & (config->prescaler - 1)) == 0))
        {
          /* Determine the index of the highest bit set to get the
           * division factor that must be loaded into the division
           * register.
           */

          uint32_t count = 0;
          uint32_t mask;

          for (mask = 2; mask < (uint32_t)config->prescaler; mask <<= 1)
            {
              count++;
            }

          /* Set binary divider power of 2 division factor */

          genctrl |= count << GCLK_GENCTRL_DIV_SHIFT;
          genctrl |= GCLK_GENCTRL_DIVSEL;
        }
      else
        {
          /* Set integer division factor */

          genctrl |= GCLK_GENCTRL_DIV((uint32_t)config->prescaler);

          /* Enable non-binary division with increased duty cycle accuracy */

          genctrl |= GCLK_GENCTRL_IDC;
        }
    }

  /* Enable or disable the clock in standby mode */

  if (config->runstandby)
    {
      genctrl |= GCLK_GENCTRL_RUNSTDBY;
    }

  /* Wait for synchronization */

  sam_gclck_waitsyncbusy(config->gclk);

  /* Preserve the GENEN bit */

  regaddr  = SAM_GCLK_GENCTRL(config->gclk);

  flags    = irqsave();
  regval   = getreg32(regaddr);
  regval  &= GCLK_GENCTRL_GENEN;
  genctrl |= regval;

  /* Configure the generator */

  putreg32(genctrl, regaddr);

  /* Wait for synchronization */

  sam_gclck_waitsyncbusy(config->gclk);
  irqrestore(flags);
  sam_gclck_waitsyncbusy(config->gclk);

  /* Enable the clock generator */

  flags    = irqsave();
  genctrl |= GCLK_GENCTRL_GENEN;
  putreg32(genctrl, regaddr);

  /* Wait for synchronization */

  sam_gclck_waitsyncbusy(config->gclk);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_gclk_chan_enable
 *
 * Description:
 *  Configure and enable a GCLK peripheral channel.
 *
 * Input Parameters:
 *   channel - Index of the GCLK channel to be enabled
 *   srcgen  - The GCLK source generator index
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_gclk_chan_enable(uint8_t channel, uint8_t srcgen)
{
  irqstate_t flags;
  uint32_t regaddr;
  uint32_t regval;

  /* Get the address of the peripheral channel control register */

  regaddr = SAM_GCLK_PCHCTRL(channel);

  /* Disable generic clock channel */

  flags = irqsave();
  sam_gclk_chan_disable(channel);

  /* Configure the peripheral channel */

  regval =  GCLK_PCHCTRL_GEN(srcgen);
  putreg32(regval, regaddr);

  /* Enable the peripheral channel */

  regval |= GCLK_PCHCTRL_CHEN;
  putreg32(regval, regaddr);

  /* Wait for clock synchronization */

  while ((getreg32(regaddr) &GCLK_PCHCTRL_CHEN) == 0);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_gclk_chan_disable
 *
 * Description:
 *  Disable a GCLK peripheral channel.
 *
 * Input Parameters:
 *   channel - Index of the GCLK channel to be disabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_gclk_chan_disable(uint8_t channel)
{
  irqstate_t flags;
  uint32_t regaddr;
  uint32_t regval;

  /* Get the address of the peripheral channel control register */

  regaddr = SAM_GCLK_PCHCTRL(channel);

  /* Disable generic clock channel */

  flags   = irqsave();
  regval  = getreg32(regaddr);
  regval &= ~GCLK_PCHCTRL_CHEN;
  putreg32(regval, regaddr);

  /* Wait for clock synchronization */

  while ((getreg32(regaddr) &GCLK_PCHCTRL_CHEN) != 0);
  irqrestore(flags);
}

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
