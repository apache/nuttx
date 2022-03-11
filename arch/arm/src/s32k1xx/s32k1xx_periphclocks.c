/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_periphclocks.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 * Much of the logic within this file derives from NXP sample code for
 * the S32K1xx MCUs.  That sample code has this licensing information:
 *
 *   Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 *   Copyright 2016-2018 NXP
 *   All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include "arm_internal.h"
#include "hardware/s32k1xx_scg.h"
#include "hardware/s32k1xx_pcc.h"
#include "s32k1xx_clockconfig.h"
#include "s32k1xx_periphclocks.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_get_pclkctrl
 *
 * Description:
 *   Given a clock name, this functions returns the address the the PCC
 *   control register for the peripheral.
 *
 * Input Parameters:
 *   clkname - The name of the peripheral clock.
 *
 * Returned Value:
 *   Address of peripheral control register.  NULL is returned if the clock
 *   name does not map to a PCC control register.
 *
 ****************************************************************************/

static uint32_t *s32k1xx_get_pclkctrl(enum clock_names_e clkname)
{
  /* Map the clock name to an index to the corresponding PCC control
   * register.
   */

  uintptr_t index = (uintptr_t)g_clkname_mapping[clkname];

  if (index != PCC_INVALID_INDEX)
    {
      /* Return the fall address of the PCC control register */

      return (uint32_t *)((uintptr_t)S32K1XX_PCC_BASE + (index << 2));
    }

  return NULL;
}

/****************************************************************************
 * Name: s32k1xx_set_pclkctrl
 *
 * Description:
 *   Sets PCC control register.
 *
 * Input Parameters:
 *   pclk - Describes the PCLK configuration.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void
s32k1xx_set_pclkctrl(const struct peripheral_clock_config_s *pclk)
{
  uint32_t *ctrlp = s32k1xx_get_pclkctrl(pclk->clkname);
  uint32_t regval;

  DEBUGASSERT(ctrlp != NULL);

  /* Configure the peripheral clock source, the fractional clock divider and
   * the clock gate.
   */

  regval =  PCC_PCS(pclk->clksrc);

  if (pclk->divider > 1)
    {
      regval |= PCC_PCD(pclk->divider);
    }

  if (pclk->frac == MULTIPLY_BY_TWO)
    {
      regval |= PCC_FRAC;
    }

  if (pclk->clkgate)
    {
      regval |= PCC_CGC;
    }

  *ctrlp = regval;
}

/****************************************************************************
 * Name: s32k1xx_get_pclkfreq_divided
 *
 * Description:
 *   This is part of the implementation of s32k1xx_get_pclkfreq.
 *
 * Input Parameters:
 *   clkname - Identifies the clock.
 *   divider - Identifies the divider to use.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static uint32_t
s32k1xx_get_pclkfreq_divided(enum clock_names_e clkname,
                             enum scg_async_clock_type_e divider)
{
  uint32_t *ctrlp;
  uint32_t frequency = 0;
  uint32_t frac;
  uint32_t div;

  ctrlp = s32k1xx_get_pclkctrl(clkname);
  frac  = ((*ctrlp & PCC_FRAC) == 0) ? 0 : 1;
  div   = (*ctrlp & PCC_PCD_MASK) >> PCC_PCD_SHIFT;

  /* Check division factor */

  if (frac <= div)
    {
      /* Check clock gate */

      if ((*ctrlp & PCC_CGC) != 0)
        {
          uint32_t clksrc;

          /* Check clock source */

          clksrc = (*ctrlp & PCC_PCS_MASK) >> PCC_PCS_SHIFT;
          switch (clksrc)
            {
              case CLK_SRC_SOSC:
                frequency = s32k1xx_get_asnchfreq(SOSC_CLK, divider);
                break;

              case CLK_SRC_SIRC:
                frequency = s32k1xx_get_asnchfreq(SIRC_CLK, divider);
                break;

              case CLK_SRC_FIRC:
                frequency = s32k1xx_get_asnchfreq(FIRC_CLK, divider);
                break;

#ifdef CONFIG_S32K1XX_HAVE_SPLL
              case CLK_SRC_SPLL:
                frequency = s32k1xx_get_asnchfreq(SPLL_CLK, divider);
                break;
#endif

              default:
                frequency = 0;
                break;
            }

          frequency = frequency / (div + 1);
          frequency = frequency * (frac + 1);
        }
    }

  return frequency;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_periphclocks
 *
 * Description:
 *   This function configures peripheral clocks in the PCC block.
 *
 * Input Parameters:
 *   count - Number of peripheral clocks to be configured
 *   pclks - Pointer to an array of peripheral clock configurations
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void s32k1xx_periphclocks(unsigned int count,
                          const struct peripheral_clock_config_s *pclks)
{
  unsigned int i;

  DEBUGASSERT(pclks != NULL);

  for (i = 0; i < count; i++, pclks++)
    {
      /* Disable the peripheral clock */

      s32k1xx_pclk_enable(pclks->clkname, false);

      /* Set peripheral clock control */

      s32k1xx_set_pclkctrl(pclks);
    }
}

/****************************************************************************
 * Name: s32k1xx_get_pclkfreq
 *
 * Description:
 *   This function returns the clock frequency of the specified peripheral
 *   functional clock.
 *
 * Input Parameters:
 *   clkname   - Identifies the peripheral clock of interest
 *   frequency - The location where the peripheral clock frequency will be
 *              returned
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int s32k1xx_get_pclkfreq(enum clock_names_e clkname, uint32_t *frequency)
{
  uint32_t *ctrlp;
  uint32_t freq = 0;
  int ret = -ENODEV;

  /* Check if the clock is enabled */

  ctrlp = s32k1xx_get_pclkctrl(clkname);
  if ((*ctrlp & PCC_CGC) != 0)
    {
      if ((g_periph_features[clkname] & HAS_INT_CLOCK_FROM_BUS_CLOCK) != 0)
        {
          uint32_t busclk;

          /* Check whether BUS CLOCK is clocked. */

          busclk = s32k1xx_get_sysclk(SCG_SYSTEM_CLOCK_BUS);
          ret    = (busclk == 0) ? -ENODEV : OK;
        }
      else if ((g_periph_features[clkname] &
                HAS_INT_CLOCK_FROM_SYS_CLOCK) != 0)
        {
          uint32_t sysclk;

          /* Check whether SYS CLOCK is clocked. */

          sysclk = s32k1xx_get_sysclk(SCG_SYSTEM_CLOCK_CORE);
          ret    = (sysclk == 0) ? -ENODEV : OK;
        }
      else if ((g_periph_features[clkname] &
                HAS_INT_CLOCK_FROM_SLOW_CLOCK) != 0)
        {
          uint32_t slowclk;

          /* Check whether SLOW CLOCK is clocked. */

          slowclk = s32k1xx_get_sysclk(SCG_SYSTEM_CLOCK_SLOW);
          ret     = (slowclk == 0) ? -ENODEV : OK;
        }
      else
        {
          /* It's an issue in peripheral features list, each peripheral must
           * have one interface clock.
           */

          DEBUGPANIC();
        }

      if (ret == OK)
        {
          /* Check whether peripheral has protocol clock (functional clock) */

          if ((g_periph_features[clkname] &
              (HAS_PROTOCOL_CLOCK_FROM_ASYNC1 |
               HAS_PROTOCOL_CLOCK_FROM_ASYNC2)) != 0)
            {
              if ((g_periph_features[clkname] &
                   HAS_PROTOCOL_CLOCK_FROM_ASYNC1) != 0)
                {
                  /* Check whether the functional clock is clocked */

                  freq = s32k1xx_get_pclkfreq_divided(clkname,
                                                      SCG_ASYNC_CLOCK_DIV1);
                }

              if ((g_periph_features[clkname] &
                   HAS_PROTOCOL_CLOCK_FROM_ASYNC2) != 0)
                {
                  /* Check whether the functional clock is clocked */

                  freq = s32k1xx_get_pclkfreq_divided(clkname,
                                                      SCG_ASYNC_CLOCK_DIV2);
                }

              if (freq == 0)
                {
                  ret = -ENODEV;
                }
            }
        }
    }

  /* If frequency reference is provided, write this value */

  if (frequency != NULL)
    {
      *frequency = freq;
    }

  return ret;
}

/****************************************************************************
 * Name: s32k1xx_pclk_enable
 *
 * Description:
 *   This function enables/disables the clock for a given peripheral.
 *
 * Input Parameters:
 *   clkname - The name of the peripheral clock to be disabled
 *   enable  - true:  Enable the peripheral clock.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void s32k1xx_pclk_enable(enum clock_names_e clkname, bool enable)
{
  uint32_t *ctrlp = s32k1xx_get_pclkctrl(clkname);
  DEBUGASSERT(ctrlp != NULL);

  /* check if it needs to be enabled */

  if (enable)
    {
      /* enable it */

      *ctrlp |= PCC_CGC;
    }
  else
    {
      /* disable it */

      *ctrlp &= ~PCC_CGC;
    }
}
