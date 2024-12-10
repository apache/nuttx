/****************************************************************************
 * arch/arm64/src/imx9/imx9_ccm.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <stdbool.h>

#include <sys/param.h>
#include <sys/types.h>

#include <arch/barriers.h>

#include "arm64_internal.h"
#include "imx9_ccm.h"

#include "hardware/imx9_ccm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ccm_clk_src_tz_access
 *
 * Description:
 *   Clock source access contol enable.
 *
 * Input Parameters:
 *   pscll      -  Clock source
 *   non_secure -  Grant non-secure access
 *   user_mode  -  Grant user mode access
 *   lock_tz    -  Lock settings
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int imx9_ccm_clk_src_tz_access(uint32_t oscpll, bool non_secure,
                                      bool user_mode, bool lock_tz)
{
  if (oscpll > CCM_OSCPLL_COUNT)
    {
      return -EINVAL;
    }

  modifyreg32(IMX9_CCM_OSCPLL_AUTH(oscpll), 0,
              CCM_AUTH_TZ_USER(user_mode) |
              CCM_AUTH_TZ_NS(non_secure) |
              CCM_AUTH_LOCK_TZ(lock_tz));

  return 0;
}

/****************************************************************************
 * Name: ccm_clk_root_tz_access
 *
 * Description:
 *   Root clock access control enable.
 *
 * Input Parameters:
 *   clk_root_id  -  Root clock id
 *   non_secure   -  Grant non-secure access
 *   user_mode    -  Grant user mode access
 *   lock_tz      -  Lock settings
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int imx9_ccm_clk_root_tz_access(uint32_t clk_root_id, bool non_secure,
                                       bool user_mode, bool lock_tz)
{
  if (clk_root_id > CCM_CR_COUNT)
    {
      return -EINVAL;
    }

  modifyreg32(IMX9_CCM_CR_AUTH(clk_root_id), 0,
              CCM_AUTH_TZ_USER(user_mode) |
              CCM_AUTH_TZ_NS(non_secure) |
              CCM_AUTH_LOCK_TZ(lock_tz));

  return 0;
}

/****************************************************************************
 * Name: imx9_ccm_lpcg_tz_access
 *
 * Description:
 *   Low power clock gatig unit access enable.
 *
 * Input Parameters:
 *   lpcg       -  Clock id
 *   non_secure -  Grant non-secure access
 *   user_mode  -  Grant user mode access
 *   lock_tz    -  Lock settings
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int imx9_ccm_lpcg_tz_access(uint32_t lpcg, bool non_secure,
                                   bool user_mode, bool lock_tz)
{
  if (lpcg > CCM_LPCG_COUNT)
    {
      return -EINVAL;
    }

  modifyreg32(IMX9_CCM_LPCG_AUTH(lpcg), 0,
              CCM_AUTH_TZ_USER(user_mode) |
              CCM_AUTH_TZ_NS(non_secure) |
              CCM_AUTH_LOCK_TZ(lock_tz));

  return 0;
}

/****************************************************************************
 * Name: imx9_ccm_shared_gpr_tz_access
 *
 * Description:
 *   General purpose access enable.
 *
 * Input Parameters:
 *   grp        -  Gpr id
 *   non_secure -  Grant non-secure access
 *   user_mode  -  Grant user mode access
 *   lock_tz    -  Lock settings
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

static int imx9_ccm_shared_gpr_tz_access(uint32_t gpr, bool non_secure,
                                         bool user_mode, bool lock_tz)
{
  if (gpr > CCM_SHARED_GPR_COUNT)
    {
      return -EINVAL;
    }

  modifyreg32(IMX9_CCM_GPR_SH_AUTH(gpr), 0,
              CCM_AUTH_TZ_USER(user_mode) |
              CCM_AUTH_TZ_NS(non_secure) |
              CCM_AUTH_LOCK_TZ(lock_tz));

  return 0;
}

/****************************************************************************
 * Name: imx9_ccm_clock_prepare
 *
 * Description:
 *   Prepares the clocks, grants non-secure access for clocks.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success, a negated error code otherwise
 *
 ****************************************************************************/

static int imx9_ccm_clock_prepare(void)
{
  int ret;
  int i;

  /* allow for non-secure access */

  for (i = 0; i < CCM_OSCPLL_END; i++)
    {
      ret = imx9_ccm_clk_src_tz_access(i, true, false, false);
      if (ret != 0)
        {
          return ret;
        }
    }

  for (i = 0; i < CCM_CLK_ROOT_NUM; i++)
    {
      ret = imx9_ccm_clk_root_tz_access(i, true, false, false);
      if (ret != 0)
        {
          return ret;
        }
    }

  for (i = 0; i < CCM_CCGR_NUM; i++)
    {
      ret = imx9_ccm_lpcg_tz_access(i, true, false, false);
      if (ret != 0)
        {
          return ret;
        }
    }

  for (i = 0; i < CCM_SHARED_GPR_NUM; i++)
    {
      ret = imx9_ccm_shared_gpr_tz_access(i, true, false, false);
      if (ret != 0)
        {
          return ret;
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_ccm_configure_root_clock
 *
 * Description:
 *   Change root clock source and divider. Leaves the clock running state
 *   unaltered.
 *
 * Input Parameters:
 *   root - The root clock index.
 *   src  - The root clock MUX source.
 *   div  - The root clock divider.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ccm_configure_root_clock(int root, int src, uint32_t div)
{
  uint32_t value;
  int i;

  if (root >= CCM_CR_COUNT || div == 0 || div > 255)
    {
      return -EINVAL;
    }

  /* Find the corresponding MUX register value for root and source */

  for (i = 0; i < ROOT_MUX_MAX; i++)
    {
      if (g_ccm_root_mux[root][i] == src)
        {
          break;
        }
    }

  if (i == ROOT_MUX_MAX)
    {
      return -EINVAL;
    }

  /* Set the new value */

  value = CCM_CR_CTRL_MUX_SRCSEL(i) | CCM_CR_CTRL_DIV(div);
  putreg32(value, IMX9_CCM_CR_CTRL(root));
  UP_MB();

  /* Wait for the clock state change */

  while (getreg32(IMX9_CCM_CR_STAT0(root)) & CCM_CR_STAT0_CHANGING);

  return OK;
}

/****************************************************************************
 * Name: imx9_ccm_root_clock_on
 *
 * Description:
 *   Enable / disable root clock.
 *
 * Input Parameters:
 *   root    - The root clock index.
 *   enabled - True enables the clock; false disables it.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ccm_root_clock_on(int root, bool enabled)
{
  if (root >= CCM_CR_COUNT)
    {
      return -EINVAL;
    }

  if (enabled)
    {
      putreg32(CCM_CR_CTRL_OFF, IMX9_CCM_CR_CTRL_CLR(root));
    }
  else
    {
      putreg32(CCM_CR_CTRL_OFF, IMX9_CCM_CR_CTRL_SET(root));
    }

  UP_MB();

  /* Wait for the clock state change */

  while (getreg32(IMX9_CCM_CR_STAT0(root)) & CCM_CR_STAT0_CHANGING);

  return OK;
}

/****************************************************************************
 * Name: imx9_ccm_gate_on
 *
 * Description:
 *   Enable / disable clock.
 *
 * Input Parameters:
 *   gate    - The clock gate index.
 *   enabled - True enables the clock; false disables it.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ccm_gate_on(int gate, bool enabled)
{
  uint32_t value;

  if (gate >= CCM_LPCG_COUNT)
    {
      return -EINVAL;
    }

  /* Make sure direct mode is on, which is what we support */

  value = getreg32(IMX9_CCM_LPCG_AUTH(gate));
  if (value & CCM_LPCG_AUTH_CPULPM)
    {
      value &= ~CCM_LPCG_AUTH_CPULPM;
      putreg32(value, IMX9_CCM_LPCG_AUTH(gate));
      UP_MB();
    }

  value = enabled ? 1 : 0;
  putreg32(value, IMX9_CCM_LPCG_DIR(gate));
  UP_MB();

  /* Wait for the clock state change */

  while ((getreg32(IMX9_CCM_LPCG_STAT0(gate)) & CCM_LPCG_STAT0_ON) != value);

  return OK;
}

/****************************************************************************
 * Name: imx9_ccm_shared_gpr_set
 *
 * Description:
 *   Set shared gpr clock register value
 *
 * Input Parameters:
 *   gpr    -  General purpose clock index
 *   val    -  Value
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int imx9_ccm_shared_gpr_set(uint32_t gpr, uint32_t val)
{
  if (gpr > CCM_SHARED_GPR_COUNT)
    {
      return -EINVAL;
    }

  putreg32(val, IMX9_CCM_GPR_SH(gpr));

  return 0;
}

/****************************************************************************
 * Name: imx9_ccm_clock_init
 *
 * Description:
 *   Initializes bus clocks for a known default state.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success, a negated error value otherwise
 *
 ****************************************************************************/

int imx9_ccm_clock_init(void)
{
  int ret;

  ret = imx9_ccm_clock_prepare();
  if (ret != 0)
    {
      return ret;
    }

  /* Set A55 clk to 500M. This clock root is normally used as intermediate
   * clock source for A55 core / DSU when doing ARM PLL reconfig. Set it to
   * 500 MHz.
   */

  ret = imx9_ccm_configure_root_clock(CCM_ARM_A55_CLK_ROOT, SYS_PLL1PFD0, 2);
  if (ret != 0)
    {
      return ret;
    }

  /* Set A55 periphal to 333 MHz */

  ret = imx9_ccm_configure_root_clock(CCM_ARM_A55_PERIPH_CLK_ROOT,
                                      SYS_PLL1PFD0, 3);
  if (ret != 0)
    {
      return ret;
    }

  /* Set A55 mtr bus to 133 MHz */

  ret = imx9_ccm_configure_root_clock(CCM_ARM_A55_MTR_BUS_CLK_ROOT,
                                      SYS_PLL1PFD1DIV2, 3);
  if (ret != 0)
    {
      return ret;
    }

  /* ELE to 200 MHz */

  ret = imx9_ccm_configure_root_clock(CCM_ELE_CLK_ROOT, SYS_PLL1PFD1DIV2, 2);
  if (ret != 0)
    {
      return ret;
    }

  /* Bus_wakeup to 133 MHz */

  ret = imx9_ccm_configure_root_clock(CCM_BUS_WAKEUP_CLK_ROOT,
                                      SYS_PLL1PFD1DIV2, 3);
  if (ret != 0)
    {
      return ret;
    }

  /* Bus_AON to 133 MHz */

  ret = imx9_ccm_configure_root_clock(CCM_BUS_AON_CLK_ROOT, SYS_PLL1PFD1DIV2,
                                      3);
  if (ret != 0)
    {
      return ret;
    }

  /* M33 to 200 MHz */

  ret = imx9_ccm_configure_root_clock(CCM_M33_CLK_ROOT, SYS_PLL1PFD1DIV2, 2);
  if (ret != 0)
    {
      return ret;
    }

  /* WAKEUP_AXI to 312.5 MHz, because of FEC only can support to 320M for
   * generating MII clock at 2.5 MHz
   */

  ret = imx9_ccm_configure_root_clock(CCM_WAKEUP_AXI_CLK_ROOT, SYS_PLL1PFD2,
                                      2);
  if (ret != 0)
    {
      return ret;
    }

  /* SWO TRACE to 133 MHz */

  ret = imx9_ccm_configure_root_clock(CCM_SWO_TRACE_CLK_ROOT,
                                      SYS_PLL1PFD1DIV2, 3);
  if (ret != 0)
    {
      return ret;
    }

  /* M33 systetick to 24 MHz */

  ret = imx9_ccm_configure_root_clock(CCM_M33_SYSTICK_CLK_ROOT, OSC_24M, 1);
  if (ret != 0)
    {
      return ret;
    }

  /* NIC to 400 MHz */

  ret = imx9_ccm_configure_root_clock(CCM_NIC_CLK_ROOT, SYS_PLL1PFD1, 2);
  if (ret != 0)
    {
      return ret;
    }

  /* NIC_APB to 133 MHz */

  ret = imx9_ccm_configure_root_clock(CCM_NIC_APB_CLK_ROOT, SYS_PLL1PFD1DIV2,
                                      3);
  if (ret != 0)
    {
      return ret;
    }

  /* ENET to 125MHz */

  imx9_ccm_gate_on(CCM_LPCG_ENET1, false);

  ret = imx9_ccm_configure_root_clock(CCM_ENET_REF_CLK_ROOT,
                                      SYS_PLL1PFD0DIV2, 2);
  if (ret != 0)
    {
      return ret;
    }

  ret = imx9_ccm_configure_root_clock(CCM_ENET_TIMER_CLK_ROOT,
                                      SYS_PLL1PFD0DIV2, 5);
  if (ret != 0)
    {
      return ret;
    }

  imx9_ccm_gate_on(CCM_LPCG_ENET1, true);

  return OK;
}
