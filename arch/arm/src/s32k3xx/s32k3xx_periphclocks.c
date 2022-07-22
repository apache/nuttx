/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_periphclocks.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include "arm_internal.h"
#include "s32k3xx_clockconfig.h"
#include "s32k3xx_periphclocks.h"
#include "hardware/s32k3xx_mc_me.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t s32k3xx_get_cofb_clken(enum clock_names_e clkname)
{
  if (clkname < 64)
    {
      return (uint32_t)(S32K3XX_MC_ME_PRTN0_COFB1_CLKEN);
    }
  else if(clkname < 160)
    {
      return (uint32_t)(S32K3XX_MC_ME_PRTN1_COFB0_CLKEN);
    }
  else if(clkname < 192)
    {
      return (uint32_t)(S32K3XX_MC_ME_PRTN1_COFB1_CLKEN);
    }
  else if(clkname < 224)
    {
      return (uint32_t)(S32K3XX_MC_ME_PRTN1_COFB2_CLKEN);
    }
  else if(clkname < 256)
    {
      return (uint32_t)(S32K3XX_MC_ME_PRTN1_COFB3_CLKEN);
    }
  else if(clkname < 288)
    {
      return (uint32_t)(S32K3XX_MC_ME_PRTN2_COFB0_CLKEN);
    }
  else
    {
      return (uint32_t)(S32K3XX_MC_ME_PRTN2_COFB1_CLKEN);
    }
}

static inline uint32_t s32k3xx_get_cofb_clken_index(
                             enum clock_names_e clkname)
{
  return (clkname - ((clkname / 128) * 128)) % 32;
}

/****************************************************************************
 * Name: s32k3xx_set_pclkctrl
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

static void
s32k3xx_cofb_clken(const struct peripheral_clock_config_s *pclk)
{
  uint32_t regval;
  uint32_t clken_reg = s32k3xx_get_cofb_clken(pclk->clkname);
  uint32_t clken_index = s32k3xx_get_cofb_clken_index(pclk->clkname);

  regval = getreg32(clken_reg);

  if (pclk->clkgate)
    {
      regval |= (1 << clken_index);
    }
  else
    {
      regval &= ~(1 << clken_index);
    }

  putreg32(regval, clken_reg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_periphclocks
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

void s32k3xx_periphclocks(unsigned int count,
                          const struct peripheral_clock_config_s *pclks)
{
  unsigned int i;

  DEBUGASSERT(pclks != NULL);

  for (i = 0; i < count; i++, pclks++)
    {
      /* Set peripheral clock control */

      s32k3xx_cofb_clken(pclks);
    }

  /* Update Process update register */

  putreg32(MC_ME_PRTN_PUPD_PCUD, S32K3XX_MC_ME_PRTN0_PUPD);
  putreg32(MC_ME_PRTN_PUPD_PCUD, S32K3XX_MC_ME_PRTN1_PUPD);
  putreg32(MC_ME_PRTN_PUPD_PCUD, S32K3XX_MC_ME_PRTN2_PUPD);

  /* Control key register */

  putreg32(MC_ME_CTL_KEY(0x5af0), S32K3XX_MC_ME_CTL_KEY);
  putreg32(MC_ME_CTL_KEY(~0x5af0), S32K3XX_MC_ME_CTL_KEY);
}
