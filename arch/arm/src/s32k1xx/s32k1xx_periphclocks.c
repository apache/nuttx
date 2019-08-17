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

#include "up_arch.h"
#include "hardware/s32k1xx_pcc.h"
#include "s32k1xx_periphclocks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
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
  /* Map the clock name to an index to the corresponding PCC control register. */

  uintptr_t index = (uintptr_t)g_clkname_mapping[clkname];

  if (index != PCC_INVALID_INDEX)
    {
      /* Return the fall address of the PCC control register */

      return (uint32_t *)((uintptr_t)S32K1XX_PCC_BASE + (index << 2));
    }

  return NULL;
}

/****************************************************************************
 * Name: s32k1xx_pclk_disable
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

static void s32k1xx_pclk_disable(enum clock_names_e clkname)
{
  uint32_t *ctrlp = s32k1xx_get_pclkctrl(clkname);
  DEBUGASSERT(ctrlp != NULL);

  *ctrlp &= ~PCC_CGC;
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

static inline void s32k1xx_set_pclkctrl(struct peripheral_clock_config_s *pclk)
{
  uint32_t *ctrlp = s32k1xx_get_pclkctrl(pclk->clkname);
  uint32_t regval;

  DEBUGASSERT(ctrlp != NULL);

   /* Configure the peripheral clock source, the fractional clock divider
    * and the clock gate.
    */

   regval =  PCC_PCS(pclk->clksrc) | PCC_PCD( pclk->divider);

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
                          struct peripheral_clock_config_s *pclks)
{
  unsigned int i;

  DEBUGASSERT(pclks != NULL);

  for (i = 0; i < count; i++, pclks++)
    {
      /* Disable the peripheral clock */

      s32k1xx_pclk_disable(pclks->clkname);

      /* Set peripheral clock control */

      s32k1xx_set_pclkctrl(pclks);
    }
}
