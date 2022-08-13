/****************************************************************************
 * arch/arm/src/samd5e5/sam_gclk.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "sam_gclk.h"

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
 *   gclk - GCLK clock index
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_gclck_waitsyncbusy(uint8_t gclk)
{
  uintptr_t gclkbit = GCLK_SYNCHBUSY_GENCTRL(gclk);
  while ((getreg8(SAM_GCLK_SYNCHBUSY) & gclkbit) != 0)
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_gclk_configure
 *
 * Description:
 *   Configure a single GCLK(s) based on settings in the config structure.
 *
 * Input Parameters:
 *   gclk   - GCLK index
 *   config - An instance of struct sam_gclkconfig describing the GCLK
 *            configuration.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_gclk_configure(int gclk, const struct sam_gclk_config_s *config)
{
  irqstate_t flags;
  uintptr_t regaddr;
  uint32_t regval;

  /* Are we enabling or disabling the GCLK? */

  regval  = 0;
  regaddr = SAM_GCLK_GENCTRL(gclk);

  if (config->enable)
    {
      /* Select the requested source clock for the generator */

      regval = GCLK_GENCTRL_SRC(config->source);

      /* Clock output selection */

      if (config->oov)
        {
          regval |= GCLK_GENCTRL_OOV;
        }

      /* Output enable */

      if (config->oe)
        {
          regval |= GCLK_GENCTRL_OE;
        }

      /* Run in standby */

      if (config->runstdby)
        {
          regval |= GCLK_GENCTRL_RUNSTDBY;
        }

      /* Set the prescaler division factor */

      if (config->div > 1)
        {
          /* Check if division is a power of two */

          if (((config->div & (config->div - 1)) == 0))
            {
              /* Determine the index of the highest bit set to get the
               * division factor that must be loaded into the division
               * register.
               */

              uint32_t count = 0;
              uint32_t mask;

              for (mask = 2; mask < (uint32_t)config->div; mask <<= 1)
                {
                  count++;
                }

              /* Set binary divider power of 2 division factor */

              regval |= GCLK_GENCTRL1_DIV(count);
              regval |= GCLK_GENCTRL_DIVSEL;
            }
          else
            {
              /* Set integer division factor */

              regval |= GCLK_GENCTRL1_DIV((uint32_t)config->div);

              /* Enable non-binary division with increased duty cycle
               * accuracy
               */

              regval |= GCLK_GENCTRL_IDC;
            }
        }

      /* Don't disable GCLK0 */

      if (gclk == 0)
        {
          regval |= GCLK_GENCTRL_GENEN;
        }
    }

  /* Configure the generator */

  flags = enter_critical_section();
  putreg32(regval, regaddr);

  /* Wait for synchronization */

  sam_gclck_waitsyncbusy(gclk);
  leave_critical_section(flags);
  sam_gclck_waitsyncbusy(gclk);

  if (config->enable)
    {
      /* Enable the clock generator */

      flags    = enter_critical_section();
      regval |= GCLK_GENCTRL_GENEN;
      putreg32(regval, regaddr);

      /* Wait for synchronization */

      sam_gclck_waitsyncbusy(gclk);
      leave_critical_section(flags);
    }
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
 *   wrlock  - True: set writelock
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_gclk_chan_enable(uint8_t channel, uint8_t srcgen, bool wrlock)
{
  irqstate_t flags;
  uint32_t regaddr;
  uint32_t regval;

  /* Get the address of the peripheral channel control register */

  regaddr = SAM_GCLK_PCHCTRL(channel);

  /* Disable generic clock channel */

  flags = enter_critical_section();
  sam_gclk_chan_disable(channel);

  /* Configure the peripheral channel */

  regval = GCLK_PCHCTRL_GEN(srcgen);
  putreg32(regval, regaddr);

  /* Enable the peripheral channel, setting the writelock if so requested. */

  regval |= GCLK_PCHCTRL_CHEN;

  if (wrlock)
    {
      regval |= GCLK_PCHCTRL_WRTLOCK;
    }

  putreg32(regval, regaddr);

  /* Wait for clock synchronization */

  while ((getreg32(regaddr) & GCLK_PCHCTRL_CHEN) == 0)
    {
    }

  leave_critical_section(flags);
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

  flags   = enter_critical_section();
  regval  = getreg32(regaddr);
  regval &= ~GCLK_PCHCTRL_CHEN;
  putreg32(regval, regaddr);

  /* Wait for clock synchronization */

  while ((getreg32(regaddr) & GCLK_PCHCTRL_CHEN) != 0)
    {
    }

  leave_critical_section(flags);
}
