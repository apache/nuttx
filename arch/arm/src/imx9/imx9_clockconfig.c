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

#ifdef CONFIG_IMX9_WFI_AWAKES_AT_SYSTICK
#include "hardware/imx9_gpc.h"
#endif

#include "imx9_clockconfig.h"
#include "imx9_scmi.h"

#include "hardware/imx9_memorymap.h"

#ifndef CONFIG_IMX9_CLK_OVER_SCMI
#include "imx9_ccm.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_IMX9_CLK_OVER_SCMI
#define PLL_PARMS(_rdiv, _odiv, _mfi, _mfn, _mfd) \
  {                                               \
    .rdiv = (_rdiv),                              \
    .odiv = (_odiv),                              \
    .mfi  = (_mfi),                               \
    .mfn  = (_mfn),                               \
    .mfd  = (_mfd),                               \
  }

#define PLL_CFG(_reg, _frac, _parms) \
  {                                  \
    .reg   = (_reg),                 \
    .frac  = (_frac),                \
    .parms = _parms,                 \
  }

#define PFD_PARMS(_mfi, _mfn, _div2) \
  {                                  \
    .mfi       = (_mfi),             \
    .mfn       = (_mfn),             \
    .divby2_en = (_div2)             \
  }

#define PFD_CFG(_reg, _pfd, _parms) \
  {                                 \
    .reg   = (_reg),                \
    .pfd   = (_pfd),                \
    .parms = _parms,                \
  }

#endif /* !CONFIG_IMX9_CLK_OVER_SCMI */

/****************************************************************************
 * Types
 ****************************************************************************/

#ifndef CONFIG_IMX9_CLK_OVER_SCMI
struct pll_parms_s
{
  /* Integer part (DIV) */

  struct
  {
    uint32_t rdiv; /* Input clock divider */
    uint32_t odiv; /* PLL output divider */
    uint32_t mfi;  /* PLL integer divider */
  };

  /* Fractional part (NUMERATOR / DENOMINATOR) */

  struct
  {
    uint32_t mfn;  /* PLL fractional divider numerator */
    uint32_t mfd;  /* PLL fractional divider denominator */
  };
};

struct pfd_parms_s
{
  uint32_t mfi;       /* PLL integer divider */
  uint32_t mfn;       /* PLL fractional divider numerator */
  bool     divby2_en; /* Enable the divide-by-2 output */
};

struct imx9_pll_cfg_s
{
  uintptr_t          reg;   /* The PLL register base */
  bool               frac;  /* Fractional PLL ? */
  struct pll_parms_s parms; /* The PLL parameters */
};

struct imx9_pfd_cfg_s
{
  uintptr_t          reg;   /* The PLL register base */
  int                pfd;   /* The PFD number */
  struct pfd_parms_s parms; /* The PFD parameters */
};
#endif /* !CONFIG_IMX9_CLK_OVER_SCMI */

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

  /* Cortex-M33 with SM does PLL initialization */
}

#ifdef CONFIG_IMX9_CLK_OVER_SCMI
static int imx9_sm_setrootclock(sm_clock_t *sm_clk)
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

static int imx9_sm_getipfreq(sm_clock_t *sm_clk)
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

#else  /* !CONFIG_IMX9_CLK_OVER_SCMI */

static uint32_t calculate_vco_freq(const struct pll_parms_s *parm, bool frac)
{
  /* Base clock is common for all VCO:s */

  if (frac)
    {
      return (uint64_t)XTAL_FREQ * (parm->mfi * parm->mfd + parm->mfn) /
             parm->mfd / parm->rdiv;
    }
  else
    {
      return (uint64_t)XTAL_FREQ * parm->mfi / parm->rdiv;
    }
}

static uint32_t vco_freq_out(uintptr_t reg, bool frac)
{
  struct pll_parms_s parm;
  uint32_t ctrl;
  uint32_t status;
  uint32_t div;

  /* Check if the PLL on or off */

  ctrl = getreg32(PLL_CTRL(reg));
  if ((ctrl & PLL_CTRL_POWERUP) == 0)
    {
      return 0;
    }

  /* Check if the PLL is stable */

  status = getreg32(PLL_PLL_STATUS(reg));
  if ((status & PLL_PLL_STATUS_PLL_LOCK) == 0)
    {
      return 0;
    }

  /* Populate the integer and fractional PLL parameters */

  div       = getreg32(PLL_DIV(reg));
  parm.rdiv = (div & PLL_DIV_RDIV_MASK) >> PLL_DIV_RDIV_SHIFT;
  parm.mfi  = (div & PLL_DIV_MFI_MASK) >> PLL_DIV_MFI_SHIFT;

  /* RDIV values 0 and 1 both mean a divisor of 1 */

  if (parm.rdiv == 0)
    {
      parm.rdiv = 1;
    }

  if (frac)
    {
      /* Fill the fractional parameters */

      parm.mfn   = getreg32(PLL_NUMERATOR(reg)) & PLL_NUMERATOR_MFN_MASK;
      parm.mfn >>= PLL_NUMERATOR_MFN_SHIFT;
      parm.mfd   = getreg32(PLL_DENOMINATOR(reg)) & PLL_DENOMINATOR_MFD_MASK;
      parm.mfd >>= PLL_DENOMINATOR_MFD_SHIFT;
    }

  return calculate_vco_freq(&parm, frac);
}

static uint32_t pll_freq_out(uintptr_t reg, bool frac)
{
  uint32_t ctrl;
  uint32_t div;
  uint32_t vco;

  /* Read the MUX control register and check if bypass mode is enabled */

  ctrl = getreg32(PLL_CTRL(reg));
  if (ctrl & PLL_CTRL_CLKMUX_BYPASS)
    {
      return XTAL_FREQ;
    }

  /* If the mux is disabled output frequency is 0 */

  if ((ctrl & PLL_CTRL_CLKMUX_EN) == 0)
    {
      return 0;
    }

  /* Get input VCO frequency */

  vco = vco_freq_out(reg, frac);
  if (vco == 0)
    {
      /* The VCO is off or unstable */

      return 0;
    }

  /* Calculate the output clock divider */

  div = (getreg32(PLL_DIV(reg)) & PLL_DIV_ODIV_MASK) >> PLL_DIV_ODIV_SHIFT;

  /* According to spec, div0 = 2 and div1 = 3 */

  if (div == 0)
    {
      div = 2;
    }
  else if (div == 1)
    {
      div = 3;
    }

  return vco / div;
}

static uint32_t pll_pfd_freq_out(uintptr_t reg, int pfd, int div2)
{
  struct pfd_parms_s parm;
  uint32_t ctrl;
  uint32_t div;
  uint32_t vco;

  /* Read the correct PFD register set */

  switch (pfd)
    {
      case 0:
        ctrl = getreg32(PLL_DFS_CTRL_0(reg));
        div  = getreg32(PLL_DFS_DIV_0(reg));
        break;

      case 1:
        ctrl = getreg32(PLL_DFS_CTRL_1(reg));
        div  = getreg32(PLL_DFS_DIV_1(reg));
        break;

      case 2:
        ctrl = getreg32(PLL_DFS_CTRL_2(reg));
        div  = getreg32(PLL_DFS_DIV_2(reg));
        break;

      default:
        return 0;
    }

  /* Get input VCO frequency */

  vco = vco_freq_out(reg, true);
  if (vco == 0)
    {
      /* The VCO is off or unstable */

      return 0;
    }

  /* If the DFS part is bypassed, the output is the VCO directly */

  if (ctrl & PLL_DFS_BYPASS_EN)
    {
      return vco;
    }

  /* Check if the DFS part is disabled */

  if ((ctrl & PLL_DFS_ENABLE) == 0)
    {
      return 0;
    }

  /* Populate the DFS parameters */

  parm.mfi = (div & PLL_DFS_MFI_MASK) >> PLL_DFS_MFI_SHIFT;
  parm.mfn = (div & PLL_DFS_MFN_MASK) >> PLL_DFS_MFN_SHIFT;

  return ((uint64_t)vco * 5) / (parm.mfi * 5 + parm.mfn) / div2;
}

/****************************************************************************
 * Name: imx9_get_clock
 *
 * Description:
 *   This function returns the clock frequency of the specified functional
 *   clock.
 *
 * Input Parameters:
 *   clkname   - Identifies the clock of interest
 *   frequency - The location where the peripheral clock frequency will be
 *              returned
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

static int imx9_get_clock(int clkname, uint32_t *frequency)
{
  switch (clkname)
    {
      case OSC_24M:
        *frequency = XTAL_FREQ;
        break;

      case ARM_PLL:
        *frequency = pll_freq_out(IMX9_ARMPLL_BASE, false);
        break;

      case SYS_PLL1_IN:
        *frequency = pll_freq_out(IMX9_SYSPLL_BASE, false);
        break;

      case SYS_PLL1PFD0:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 0, 1);
        break;

      case SYS_PLL1PFD0DIV2:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 0, 2);
        break;

      case SYS_PLL1PFD1:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 1, 1);
        break;

      case SYS_PLL1PFD1DIV2:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 1, 2);
        break;

      case SYS_PLL1PFD2:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 2, 1);
        break;

      case SYS_PLL1PFD2DIV2:
        *frequency = pll_pfd_freq_out(IMX9_SYSPLL_BASE, 2, 2);
        break;

      case AUDIO_PLL1OUT:
        *frequency = pll_freq_out(IMX9_AUDIOPLL_BASE, true);
        break;

      case DRAM_PLLOUT:
        *frequency = pll_freq_out(IMX9_DRAMPLL_BASE, true);
        break;

      case VIDEO_PLL1OUT:
        *frequency = pll_freq_out(IMX9_VIDEOPLL_BASE, true);
        break;

      default:
        return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_IMX9_CLK_OVER_SCMI */

int imx9_configure_clock(clock_config_t clk_config, bool enabled)
{
#ifdef CONFIG_IMX9_CLK_OVER_SCMI
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

#else

  int ret;
  int root = GET_CLOCK_ROOT(clk_config) + ROOT_CLOCK_OFFSET;
  int gate = GET_CLOCK_GATE(clk_config);

  ret = imx9_ccm_configure_root_clock(
                root,
                GET_ROOT_MUX(clk_config),
                GET_CLOCK_DIV(clk_config));
  if (ret)
    {
      return ret;
    }

  ret = imx9_ccm_root_clock_on(root, enabled);
  if (ret)
    {
      return ret;
    }

  if (gate != CCM_LPCG_NONE)
    {
      ret = imx9_ccm_gate_on(gate, enabled);
      if (ret)
        {
          return ret;
        }
    }

  return OK;

#endif
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
#ifdef CONFIG_IMX9_CLK_OVER_SCMI
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

#else
  uint32_t reg;
  uint32_t div;
  uint32_t mux;
  int clk_name;

  if (clkroot <= CCM_CR_COUNT)
    {
      reg = getreg32(IMX9_CCM_CR_CTRL(clkroot));

      if ((reg & CCM_CR_CTRL_OFF) == CCM_CR_CTRL_OFF)
        {
          *frequency = 0;
        }
      else
        {
          mux = (reg & CCM_CR_CTRL_MUX_MASK) >> CCM_CR_CTRL_MUX_SHIFT;
          clk_name = g_ccm_root_mux[clkroot][mux];
          imx9_get_clock(clk_name, frequency);
          div = ((reg & CCM_CR_CTRL_DIV_MASK) >> CCM_CR_CTRL_DIV_SHIFT) + 1;
          *frequency = *frequency / div;
        }

      return OK;
    }

#endif
  return -ENODEV;
}
