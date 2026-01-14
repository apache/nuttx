/****************************************************************************
 * arch/arm/src/mcx-nxxx/nxxx_clockconfig.c
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

#include <arch/board/board.h>

#include "arm_internal.h"
#include "nxxx_clockconfig.h"

#include "hardware/nxxx_fmu.h"
#include "hardware/nxxx_clock.h"
#include "hardware/nxxx_scg.h"
#include "hardware/nxxx_spc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The base oscillator frequency is 24MHz */

#define XTAL_FREQ       24000000u
#define HFRCO_FREQ      48000000u
#define SYSCLOCK_FREQ   150000000u

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pll_params_s
{
  uint32_t pllctrl;    /* PLL Control */
  uint32_t pllndiv;    /* PLL N Divider */
  uint32_t pllpdiv;    /* PLL P Divider */
  uint32_t pllmdiv;    /* PLL M Divider */
  uint32_t pllsscg[2]; /* PLL Spread Spectrum Control */
  uint32_t pllrate;    /* PLL rate */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxxx_sysinit
 *
 * Description:
 *   Perform system init for MCX-NXXX. This prepares the SoC for use.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int nxxx_sysinit(void)
{
  /* Disable ECC to get maximum RAM */

  putreg32(0, SYSCON_ECC_ENABLE_CTRL);

  /* Enable the flash cache LPCAC */

  modifyreg32(SYSCON_LPCAC_CTRL, SYSCON_LPCAC_CTRL_DIS_LPCAC_MASK, 0);

  return OK;
}

/****************************************************************************
 * Name: pll0_setup
 *
 * Description:
 *   Setup PLL0 as per given parameters.
 *
 * Input Parameters:
 *   params   - PLL0 parameters.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int pll0_setup(struct pll_params_s *params)
{
  uint32_t inclk;
  uint32_t outclk;
  uint32_t prediv;
  uint32_t cnfg;
  uint32_t clksrc;

  /* Enable LDO */

  modifyreg32(NXXX_CFG_LDOCSR, 0, SCG_LDOCSR_LDOEN);

  /* Shut down PLL0 when we are configuring it */

  modifyreg32(NXXX_SCG_APLLCSR,
              SCG_APLLCSR_APLLPWREN_MASK | SCG_APLLCSR_APLLCLKEN_MASK,
              0);

  /* Configure PLL as per given parameters */

  putreg32(params->pllctrl, NXXX_SCG_APLLCTRL);
  putreg32(params->pllndiv, NXXX_SCG_APLLNDIV);
  putreg32(params->pllndiv | SCG_APLLNDIV_NREQ_MASK, NXXX_SCG_APLLNDIV);
  putreg32(params->pllpdiv, NXXX_SCG_APLLPDIV);
  putreg32(params->pllpdiv | SCG_APLLPDIV_PREQ_MASK, NXXX_SCG_APLLPDIV);
  putreg32(params->pllmdiv, NXXX_SCG_APLLMDIV);
  putreg32(params->pllmdiv | SCG_APLLMDIV_MREQ_MASK, NXXX_SCG_APLLMDIV);
  putreg32(params->pllsscg[0], NXXX_SCG_APLLSSCG0);
  putreg32(params->pllsscg[1], NXXX_SCG_APLLSSCG1);

  /* Unlock APLLLOCK_CNFG register */

  putreg32(0x5a5a0001, NXXX_SCG_TRIM_LOCK);

  /* Configure lock time for APLL */

  clksrc = getreg32(NXXX_SCG_APLLCTRL);
  clksrc = clksrc & SCG_APLLCTRL_SOURCE_MASK >> SCG_APLLCTRL_SOURCE_SHIFT;

  switch (clksrc)
    {
      case 0:
        inclk = XTAL_FREQ;
        break;

      case 1:
        inclk = HFRCO_FREQ;
        break;

      default:
        return -EINVAL;
    }

  if ((getreg32(NXXX_SCG_APLLCTRL) & SCG_APLLCTRL_BYPASSPREDIV_MASK) == 0)
    {
      prediv = getreg32(NXXX_SCG_APLLNDIV) & SCG_APLLNDIV_NDIV_MASK;
      if (prediv == 0)
        {
          prediv = 1;
        }
    }

  /* Adjust input clock */

  outclk = inclk / prediv;
  cnfg   = SCG_APLLLOCK_CNFG_LOCK_TIME(outclk / 2000 + 300);

  putreg32(cnfg, NXXX_SCG_APLLLOCK_CNFG);

  /* Power on PLL0 and enable PLL0 clock */

  modifyreg32(NXXX_SCG_APLLCSR,
              0,
              SCG_APLLCSR_APLLPWREN_MASK | SCG_APLLCSR_APLLCLKEN_MASK);

  /* Wait for APLL lock */

  while ((getreg32(NXXX_SCG_APLLCSR) & SCG_APLLCSR_APLL_LOCK_MASK) != 0)
    {
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxxx_clockconfig
 *
 * Description:
 *   Called to initialize the clocks for MCX-NXXX. This does whatever setup
 *   is needed to put the SoC in a usable state. This includes the
 *   initialization of clocking using the settings in board.h.
 *
 ****************************************************************************/

void nxxx_clockconfig(void)
{
  /* Prepare the SoC for use */

  nxxx_sysinit();

  /* This sets up the system clock at 150MHz */

  nxxx_set_clock_gate(CLOCK_GATE_SCG, true);

  /* Do the clock configuration with the 12MHz FRO main clock */

  putreg32(SCG_RCCR_SCS_SIRC, NXXX_SCG_RCCR);

  while ((getreg32(NXXX_SCG_CSR) & SCG_CSR_SCS_MASK) != SCG_RCCR_SCS_SIRC)
    {
    }

  /* Set the DCDC VDD regulator to 1.2 V voltage level */

  modifyreg32(NXXX_SPC_ACTIVE_CFG,
              SPC_ACTIVE_CFG_DCDC_VDD_DS_MASK,
              SPC_ACTIVE_CFG_DCDC_VDD_DS(SPC_DCDC_NORMALDRIVESTRENGTH));
  modifyreg32(NXXX_SPC_ACTIVE_CFG,
              SPC_ACTIVE_CFG_DCDC_VDD_LVL_MASK,
              SPC_ACTIVE_CFG_DCDC_VDD_LVL(SPC_DCDC_OVERDRIVEVOLTAGE));

  while ((getreg32(NXXX_SPC_SC) & SPC_SC_BUSY_MASK) != 0)
    {
    }

  /* Set the LDO_CORE VDD regulator to 1.2 V voltage level */

  modifyreg32(NXXX_SPC_ACTIVE_CFG,
              SPC_ACTIVE_CFG_CORELDO_VDD_DS_MASK,
              SPC_ACTIVE_CFG_CORELDO_VDD_DS(
                SPC_CORELDO_NORMALDRIVESTRENGTH));
  modifyreg32(NXXX_SPC_ACTIVE_CFG,
              SPC_ACTIVE_CFG_CORELDO_VDD_LVL_MASK,
              SPC_ACTIVE_CFG_CORELDO_VDD_LVL(SPC_CORELDO_OVERDRIVEVOLTAGE));

  while ((getreg32(NXXX_SPC_SC) & SPC_SC_BUSY_MASK) != 0)
    {
    }

  /* Configure Flash wait-states for 150MHz */

  modifyreg32(NXXX_FMU_FCTRL, FMU_FCTRL_RWSC_MASK, FMU_FCTRL_RWSC(3));

  /* Set 1.2V SRAM voltage */

  putreg32(SPC_SRAMCTL_VSM(SPC_SRAM1V2), NXXX_SPC_SRAMCTL);

  /* Request for voltage update */

  modifyreg32(NXXX_SPC_SRAMCTL, 0, SPC_SRAMCTL_REQ_MASK);

  while ((getreg32(NXXX_SPC_SRAMCTL) & SPC_SRAMCTL_ACK_MASK) == 0)
    {
    }

  modifyreg32(NXXX_SPC_SRAMCTL, SPC_SRAMCTL_REQ_MASK, 0);

  /* Select 48MHz for FIRC clock */

  putreg32(SCG_FIRCCFG_48MHZ, NXXX_SCG_FIRCCFG);

  /* Unlock FIRCCSR */

  modifyreg32(NXXX_SCG_FIRCCSR, SCG_FIRCCSR_LK, 0);

  /* Enable FIRC 48 MHz clock for peripheral use */

  modifyreg32(NXXX_SCG_FIRCCSR, 0, SCG_FIRCCSR_SCLK_PERIPH_EN);

  /* Enable FIRC 144 MHz clock for peripheral use */

  modifyreg32(NXXX_SCG_FIRCCSR, 0, SCG_FIRCCSR_FCLK_PERIPH_EN);

  /* Enable FIRC */

  modifyreg32(NXXX_SCG_FIRCCSR, 0, SCG_FIRCCSR_FIRCEN);

  /* Wait for FIRC clock to be valid. */

  while ((getreg32(NXXX_SCG_FIRCCSR) & SCG_FIRCCSR_FIRCVLD) == 0)
    {
    }

  /* Setup PLL0 for 150MHz */

  struct pll_params_s pll0_params =
    {
     .pllctrl = SCG_APLLCTRL_SOURCE(1) |
                SCG_APLLCTRL_SELI(27) |
                SCG_APLLCTRL_SELP(13),
     .pllndiv = SCG_APLLNDIV_NDIV(8),
     .pllpdiv = SCG_APLLPDIV_PDIV(1),
     .pllmdiv = SCG_APLLMDIV_MDIV(50),
     .pllrate = SYSCLOCK_FREQ
    };

  pll0_setup(&pll0_params);

  /* Disable Pll0 monitor */

  modifyreg32(NXXX_SCG_APLLCSR,
              SCG_APLLCSR_APLLCM_MASK | SCG_APLLCSR_APLLCMRE_MASK,
              0);

  /* Take the PLL0 clock into use */

  modifyreg32(SYSCON_DIVAHBCLK, 0, SYSCON_CLKDIV_RESET | SYSCON_CLKDIV_HALT);
  putreg32(0, SYSCON_DIVAHBCLK);
  putreg32(SCG_RCCR_SCS_APLL, NXXX_SCG_RCCR);

  while ((getreg32(NXXX_SCG_CSR) & SCG_CSR_SCS_MASK) != SCG_RCCR_SCS_APLL)
    {
    }

  /* Enable PORT clocks, otherwise pin configuration will not work */

  nxxx_set_clock_gate(CLOCK_GATE_PORT0, true);
  nxxx_set_clock_gate(CLOCK_GATE_PORT1, true);
  nxxx_set_clock_gate(CLOCK_GATE_PORT2, true);
  nxxx_set_clock_gate(CLOCK_GATE_PORT3, true);
  nxxx_set_clock_gate(CLOCK_GATE_PORT4, true);

  /* Enable GPIO clocks */

  nxxx_set_clock_gate(CLOCK_GATE_GPIO0, true);
  nxxx_set_clock_gate(CLOCK_GATE_GPIO1, true);
  nxxx_set_clock_gate(CLOCK_GATE_GPIO2, true);
  nxxx_set_clock_gate(CLOCK_GATE_GPIO3, true);
  nxxx_set_clock_gate(CLOCK_GATE_GPIO4, true);
}

/****************************************************************************
 * Name: nxxx_set_periphclock
 *
 * Description:
 *   This function sets the clock frequency of the specified peripheral
 *   functional clock.
 *
 * Input Parameters:
 *   clock - Identifies the peripheral clock of interest
 *   sel   - Selected clock source (every peripheral has its own set of
 *           possible sources)
 *   div   - Divider for the clock
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int nxxx_set_periphclock(struct clock_regs_s clock, uint32_t sel,
                         uint32_t div)
{
  /* First, halt the clock divider */

  modifyreg32(clock.div, 0, SYSCON_CLKDIV_RESET | SYSCON_CLKDIV_HALT);

  /* Then set the divider, divider of 0 means to halt the clock */

  if (div > 0)
    {
      putreg32(div - 1, clock.div);
    }

  /* Then apply the clock selector */

  putreg32(sel, clock.mux);

  return OK;
}

/****************************************************************************
 * Name: nxxx_set_clock_gate
 *
 * Description:
 *   Open or close a specific clock gate.
 *
 * Input Parameters:
 *   gate    - Identifies the peripheral clock gate of interest.
 *   enabled - True enables the clock; false disables it.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.  -ENODEV is returned if the clock is not enabled or is not
 *   being clocked.
 *
 ****************************************************************************/

int nxxx_set_clock_gate(struct clock_gate_reg_s gate, bool enabled)
{
  /* Note: we use true to enable the clock, not the clock gate. This might
   * be confusing to some, but the API is more intuitive when true enables
   * the clock.
   */

  if (enabled)
    {
      putreg32(1 << gate.bit, AHB_CLK_CTRL_SET0 + (gate.reg & 0xf));
    }
  else
    {
      putreg32(1 << gate.bit, AHB_CLK_CTRL_CLR0 + (gate.reg & 0xf));
    }

  return OK;
}

/****************************************************************************
 * Name: nxxx_get_coreclk
 *
 * Description:
 *   Return the current value of the CORE clock frequency.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   The current value of the CORE clock frequency.  Zero is returned on any
 *   failure.
 *
 ****************************************************************************/

uint32_t nxxx_get_coreclk(void)
{
  return SYSCLOCK_FREQ;
}
