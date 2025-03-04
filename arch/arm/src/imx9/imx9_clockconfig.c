/****************************************************************************
 * arch/arm/src/imx9/imx9_clockconfig.c
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#include <stdbool.h>
#include <stdint.h>

#include <sys/param.h>
#include <sys/types.h>

#include <arch/board/board.h>
#include <arch/barriers.h>

#include "arm_internal.h"
#include "hardware/imx9_gpc.h"
#include "imx9_clockconfig.h"
#include "imx9_scmi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The base oscillator frequency is 24MHz */

#define XTAL_FREQ 24000000u

#define ROOT_CLOCK_OFFSET 41

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_clockconfig
 *
 * Description:
 *   Called to initialize the i.IMX9.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void imx9_clockconfig(void)
{
#ifdef CONFIG_IMX9_WFI_AWAKES_AT_SYSTICK

  /* Keep the system clock running so SYSTICK can wake up the system from
   * wfi.
   */

  modifyreg32(
      IMX9_GPC_CTRL_CMC_MODE_CTRL(IMX9_GPC_CTRL_CM7_BASE),
      IMX9_GPC_CTRL_CMC_MODE_CTRL_CPU_MODE_TARGET_MASK,
      IMX9_GPC_CTRL_CMC_MODE_CTRL_CPU_MODE_TARGET(
          IMX9_GPC_CTRL_CMC_MODE_CTRL_CPU_MODE_TARGET_STAY_IN_RUN_MODE));

  modifyreg32(IMX9_GPC_CTRL_CMC_MISC(IMX9_GPC_CTRL_CM7_BASE),
              IMX9_GPC_CTRL_CMC_MISC_SLEEP_HOLD_EN_FLAG, 0);
#endif

  /* Cortex-M33 with SM does PLL initalization */
}

#ifdef CONFIG_IMX9_CLK_OVER_SCMI
int imx9_sm_setrootclock(sm_clock_t *sm_clk)
{
  scmi_clock_rate_t rate = /* clang-format off */
    {
      0, 0
    }; /* clang-format on */

  uint32_t channel        = sm_clk->channel;
  uint32_t clock_id       = sm_clk->clk_id;
  uint32_t pclk_id        = sm_clk->pclk_id;
  uint32_t div            = sm_clk->div;
  uint32_t attributes     = sm_clk->attributes;
  uint32_t oem_config_val = sm_clk->oem_config_val;
  uint32_t flags          = sm_clk->flags;
  uint32_t old_pclk_id    = 0; /* parent clock id */
  uint64_t src_rate, root_rate;
  int32_t status = -1;

  if (div == 0)
    {
      return -EINVAL;
    }

  status = imx9_scmi_clockparentget(channel, clock_id, &old_pclk_id);
  if (status != 0)
    {
      return status;
    }

  if (old_pclk_id != pclk_id)
    {
      status = imx9_scmi_clockparentset(channel, clock_id, pclk_id);
      if (status != 0)
        {
          return status;
        }
    }

  status = imx9_scmi_clockrateget(channel, pclk_id, &rate);
  if (status != 0)
    {
      return status;
    }

  src_rate = rate.upper;
  src_rate = (src_rate << 32);
  src_rate |= rate.lower;

  root_rate = src_rate / div;

  rate.lower = root_rate & SM_CLOCK_RATE_MASK;
  rate.upper = (root_rate >> 32) & SM_CLOCK_RATE_MASK;

  status = imx9_scmi_clockrateset(channel, clock_id, flags, rate);
  if (status != 0)
    {
      return status;
    }

  status = imx9_scmi_clockconfigset(channel, clock_id, attributes,
                                    oem_config_val);
  if (status != 0)
    {
      return status;
    }

  return OK;
}

int imx9_sm_getipfreq(sm_clock_t *sm_clk)
{
  scmi_clock_rate_t rate = /* clang-format off */
    {
      0, 0
    }; /* clang-format on */

  uint32_t channel  = sm_clk->channel;
  uint32_t clock_id = sm_clk->clk_id;
  uint32_t pclk_id  = sm_clk->pclk_id;
  int status        = 0;

  status = imx9_scmi_clockparentget(channel, clock_id, &pclk_id);
  if (status < 0)
    {
      return status;
    }

  status = imx9_scmi_clockrateget(channel, clock_id, &rate);
  if (status < 0)
    {
      return status;
    }

  return rate.lower;
}
#endif

int imx9_configure_clock(clock_config_t clk_config, bool enabled)
{
  sm_clock_t sm_clk = /* clang-format off */
    {
      0
    }; /* clang-format on */

  sm_clk.clk_id  = GET_CLOCK_ROOT(clk_config) + ROOT_CLOCK_OFFSET;
  sm_clk.pclk_id = GET_CLOCK_ID(clk_config);
  sm_clk.channel = SM_PLATFORM_A2P;
  sm_clk.div     = GET_CLOCK_DIV(clk_config);

  if (sm_clk.div == 0)
    {
      /* Make sure div is always 1 */

      sm_clk.div = 1;
    }

  sm_clk.attributes = SCMI_CLOCK_CONFIG_SET_ENABLE(enabled);
  sm_clk.flags      = SCMI_CLOCK_RATE_FLAGS_ROUND(SCMI_CLOCK_ROUND_AUTO);

  return imx9_sm_setrootclock(&sm_clk);
}

/****************************************************************************
 * Name: imx9_get_rootclock
 *
 * Description:
 *   This function returns the clock frequency of the specified root
 *   functional clock.
 *
 * Input Parameters:
 *   clkroot   - Identifies the peripheral clock of interest
 *   frequency - The location where the peripheral clock frequency will be
 *              returned
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int imx9_get_rootclock(int clkroot, uint32_t *frequency)
{
  if (clkroot <= CCM_CR_COUNT)
    {
      uint32_t ret = 0;

      sm_clock_t sm_clk = /* clang-format off */
        {
          0
        }; /* clang-format on */

      sm_clk.clk_id  = (uint32_t)(clkroot + ROOT_CLOCK_OFFSET);
      sm_clk.channel = SM_PLATFORM_A2P;

      ret = imx9_sm_getipfreq(&sm_clk);

      if (ret < 0)
        {
          return -ENODEV;
        }
      else
        {
          *frequency = ret;
          return OK;
        }
    }

  return -ENODEV;
}
