/****************************************************************************
 * arch/arm/src/samd2l2/samd_glck.c
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

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)

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
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_gclck_waitsyncbusy(void)
{
  while ((getreg8(SAM_GCLK_STATUS) & GCLK_STATUS_SYNCBUSY) != 0);
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
  uint32_t genctrl;
  uint32_t gendiv;

  /* Select the requested source clock for the generator */

  genctrl = ((uint32_t)config->gclk << GCLK_GENCTRL_ID_SHIFT) |
            ((uint32_t)config->clksrc << GCLK_GENCTRL_SRC_SHIFT);
  gendiv  = ((uint32_t)config->gclk << GCLK_GENDIV_ID_SHIFT);

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

          gendiv  |= count << GCLK_GENDIV_DIV_SHIFT;
          genctrl |= GCLK_GENCTRL_DIVSEL;
        }
      else
        {
          /* Set integer division factor */

          gendiv  |= GCLK_GENDIV_DIV((uint32_t)config->prescaler);

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

  sam_gclck_waitsyncbusy();

  /* Select the generator */

  putreg32(((uint32_t)config->gclk << GCLK_GENDIV_ID_SHIFT),
           SAM_GCLK_GENDIV);

  /* Wait for synchronization */

  sam_gclck_waitsyncbusy();

  /* Write the new generator configuration */

  putreg32(gendiv, SAM_GCLK_GENDIV);

  /* Wait for synchronization */

  sam_gclck_waitsyncbusy();

  /* Enable the clock generator */

  genctrl |= GCLK_GENCTRL_GENEN;
  putreg32(genctrl, SAM_GCLK_GENCTRL);

  /* Wait for synchronization */

  sam_gclck_waitsyncbusy();
}

#endif /* CONFIG_ARCH_FAMILY_SAMD20 || CONFIG_ARCH_FAMILY_SAMD21 */
