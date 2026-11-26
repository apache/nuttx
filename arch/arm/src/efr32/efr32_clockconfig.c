/****************************************************************************
 * arch/arm/src/efr32/efr32_clockconfig.c
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

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/efr32_cmu.h"
#include "hardware/efr32_devinfo.h"
#include "hardware/efr32_dpll.h"
#include "hardware/efr32_hfrco.h"
#include "hardware/efr32_hfxo.h"
#include "hardware/efr32_msc.h"
#include "efr32_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The board provides the crystal and the DPLL setting:
 *
 *   BOARD_HFXO_FREQUENCY    - HFXO crystal frequency
 *   BOARD_DPLL_N, _M        - SYSCLK = HFXO * (N + 1) / (M + 1)
 *   BOARD_SYSCLK_FREQUENCY  - the resulting SYSCLK
 *   BOARD_HCLK_FREQUENCY    - SYSCLK or SYSCLK / 2, 4, 8, 16
 *   BOARD_PCLK_FREQUENCY    - HCLK or HCLK / 2
 */

#if !defined(BOARD_HFXO_FREQUENCY) || !defined(BOARD_DPLL_N) || \
    !defined(BOARD_DPLL_M) || !defined(BOARD_SYSCLK_FREQUENCY) || \
    !defined(BOARD_HCLK_FREQUENCY) || !defined(BOARD_PCLK_FREQUENCY)
#  error "board.h must define the EFR32 clock configuration"
#endif

/* The HFRCODPLL is started from the factory calibration of the band that
 * contains the target frequency, then the DPLL locks it to the HFXO.  Only
 * the 80 MHz band (72.6 to 80 MHz) is supported here.
 */

#if BOARD_SYSCLK_FREQUENCY < 72600000 || BOARD_SYSCLK_FREQUENCY > 80000000
#  error "BOARD_SYSCLK_FREQUENCY must be within the 80 MHz HFRCODPLL band"
#endif

#define HFRCODPLL_DEVINFO_BAND   EFR32_DEVINFO_HFRCODPLLCAL_80MHZ

/* HCLK prescaler */

#if BOARD_HCLK_FREQUENCY == BOARD_SYSCLK_FREQUENCY
#  define SYSCLKCTRL_HCLKPRESC   CMU_SYSCLKCTRL_HCLKPRESC_DIV1
#elif BOARD_HCLK_FREQUENCY == (BOARD_SYSCLK_FREQUENCY / 2)
#  define SYSCLKCTRL_HCLKPRESC   CMU_SYSCLKCTRL_HCLKPRESC_DIV2
#elif BOARD_HCLK_FREQUENCY == (BOARD_SYSCLK_FREQUENCY / 4)
#  define SYSCLKCTRL_HCLKPRESC   CMU_SYSCLKCTRL_HCLKPRESC_DIV4
#elif BOARD_HCLK_FREQUENCY == (BOARD_SYSCLK_FREQUENCY / 8)
#  define SYSCLKCTRL_HCLKPRESC   CMU_SYSCLKCTRL_HCLKPRESC_DIV8
#elif BOARD_HCLK_FREQUENCY == (BOARD_SYSCLK_FREQUENCY / 16)
#  define SYSCLKCTRL_HCLKPRESC   CMU_SYSCLKCTRL_HCLKPRESC_DIV16
#else
#  error "Unsupported BOARD_HCLK_FREQUENCY"
#endif

/* PCLK prescaler.  PCLK must not exceed 50 MHz. */

#if BOARD_PCLK_FREQUENCY == BOARD_HCLK_FREQUENCY
#  define SYSCLKCTRL_PCLKPRESC   0
#elif BOARD_PCLK_FREQUENCY == (BOARD_HCLK_FREQUENCY / 2)
#  define SYSCLKCTRL_PCLKPRESC   CMU_SYSCLKCTRL_PCLKPRESC
#else
#  error "Unsupported BOARD_PCLK_FREQUENCY"
#endif

#if BOARD_PCLK_FREQUENCY > 50000000
#  error "PCLK must not exceed 50 MHz"
#endif

/* The radio AHB clock (RHCLK) must not exceed 40 MHz */

#if BOARD_SYSCLK_FREQUENCY > 40000000
#  define SYSCLKCTRL_RHCLKPRESC  CMU_SYSCLKCTRL_RHCLKPRESC
#else
#  define SYSCLKCTRL_RHCLKPRESC  0
#endif

/* Flash wait states: one wait state above 40 MHz */

#if BOARD_HCLK_FREQUENCY > 40000000
#  define MSC_READCTRL_MODE      MSC_READCTRL_MODE_WS1
#else
#  define MSC_READCTRL_MODE      MSC_READCTRL_MODE_WS0
#endif

/* HFXO start-up settings for the crystal (core bias and timeouts) */

#define HFXO_COREBIASSTARTUP     32
#define HFXO_COREBIASSTARTUPI    32
#define HFXO_COREBIASANA         60

/* The EFR32xG24 has an internal load imbalance between XI and XO, which
 * is compensated by a larger tuning capacitance on XO.
 */

#define HFXO_CTUNE_XO_DELTA      40
#define HFXO_CTUNE_MAX           255

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efr32_hfxo_ctune
 *
 * Description:
 *   Return the HFXO tuning capacitance: the value that the factory
 *   programming stored in the user data page if there is one, otherwise
 *   CONFIG_EFR32_HFXO_CTUNE.
 *
 ****************************************************************************/

static uint32_t efr32_hfxo_ctune(void)
{
  uint16_t ctune = getreg16(EFR32_USERDATA_MFG_CTUNE);

  if (ctune > HFXO_CTUNE_MAX)
    {
      ctune = CONFIG_EFR32_HFXO_CTUNE;
    }

  return ctune;
}

/****************************************************************************
 * Name: efr32_hfxo_start
 *
 * Description:
 *   Configure the HFXO for the crystal, start it and wait until it is
 *   ready and its core bias current is optimized.  The HFXO is then kept
 *   running (FORCEEN).
 *
 ****************************************************************************/

static void efr32_hfxo_start(void)
{
  uint32_t ctunexi;
  uint32_t ctunexo;
  uint32_t waitfor;
  uint32_t regval;

  putreg32(HFXO_LOCK_LOCKKEY_UNLOCK, EFR32_HFXO0_LOCK);

  /* The configuration must be written while the HFXO is off */

  putreg32(HFXO_CTRL_DISONDEMAND | HFXO_CTRL_DISONDEMANDBUFOUT,
           EFR32_HFXO0_CTRL_SET);
  putreg32(HFXO_CTRL_FORCEEN, EFR32_HFXO0_CTRL_CLR);

  while ((getreg32(EFR32_HFXO0_STATUS) & HFXO_STATUS_ENS) != 0)
    {
    }

  while ((getreg32(EFR32_HFXO0_STATUS) & HFXO_STATUS_SYNCBUSY) != 0)
    {
    }

  ctunexi = efr32_hfxo_ctune();
  ctunexo = ctunexi + HFXO_CTUNE_XO_DELTA;
  if (ctunexo > HFXO_CTUNE_MAX)
    {
      ctunexo = HFXO_CTUNE_MAX;
    }

  putreg32(HFXO_XTALCFG_TIMEOUTCBLSB_T416US |
           HFXO_XTALCFG_TIMEOUTSTEADY_T833US |
           HFXO_XTALCFG_CTUNEXOSTARTUP(0) |
           HFXO_XTALCFG_CTUNEXISTARTUP(0) |
           HFXO_XTALCFG_COREBIASSTARTUP(HFXO_COREBIASSTARTUP) |
           HFXO_XTALCFG_COREBIASSTARTUPI(HFXO_COREBIASSTARTUPI),
           EFR32_HFXO0_XTALCFG);

  regval  = getreg32(EFR32_HFXO0_XTALCTRL) & HFXO_XTALCTRL_SKIPCOREBIASOPT;
  regval |= HFXO_XTALCTRL_COREDGENANA_NONE |
            HFXO_XTALCTRL_CTUNEFIXANA_BOTH |
            HFXO_XTALCTRL_CTUNEXOANA(ctunexo) |
            HFXO_XTALCTRL_CTUNEXIANA(ctunexi) |
            HFXO_XTALCTRL_COREBIASANA(HFXO_COREBIASANA);
  putreg32(regval, EFR32_HFXO0_XTALCTRL);

  modifyreg32(EFR32_HFXO0_CFG,
              HFXO_CFG_SQBUFSCHTRGANA | HFXO_CFG_ENXIDCBIASANA |
              HFXO_CFG_MODE_MASK,
              HFXO_CFG_MODE_XTAL);

  /* Start the crystal and wait for the steady state, and for the core
   * bias optimization unless it is skipped.
   */

  modifyreg32(EFR32_HFXO0_CTRL,
              HFXO_CTRL_FORCEXO2GNDANA | HFXO_CTRL_FORCEXI2GNDANA |
              HFXO_CTRL_DISONDEMAND | HFXO_CTRL_EM23ONDEMAND,
              HFXO_CTRL_FORCEEN);

  waitfor = HFXO_STATUS_RDY | HFXO_STATUS_ENS;
  if ((regval & HFXO_XTALCTRL_SKIPCOREBIASOPT) == 0)
    {
      waitfor |= HFXO_STATUS_COREBIASOPTRDY;
    }

  while ((getreg32(EFR32_HFXO0_STATUS) & waitfor) != waitfor)
    {
    }

  /* Use a short steady-state timeout and skip the optimization on later
   * starts.  The registers are only writable with DISONDEMAND set.
   */

  putreg32(HFXO_CTRL_DISONDEMAND, EFR32_HFXO0_CTRL_SET);
  modifyreg32(EFR32_HFXO0_XTALCFG, HFXO_XTALCFG_TIMEOUTSTEADY_MASK,
              HFXO_XTALCFG_TIMEOUTSTEADY_T83US);
  putreg32(HFXO_XTALCTRL_SKIPCOREBIASOPT, EFR32_HFXO0_XTALCTRL_SET);
  putreg32(HFXO_CTRL_DISONDEMAND, EFR32_HFXO0_CTRL_CLR);
}

/****************************************************************************
 * Name: efr32_dpll_lock
 *
 * Description:
 *   Load the factory calibration of the HFRCODPLL band, then lock the
 *   HFRCODPLL to the HFXO with the DPLL.  SYSCLK must not be running from
 *   the HFRCODPLL meanwhile.
 *
 ****************************************************************************/

static void efr32_dpll_lock(void)
{
  uint32_t regval;

  /* Stop the DPLL */

  putreg32(DPLL_LOCK_LOCKKEY_UNLOCK, EFR32_DPLL0_LOCK);
  putreg32(DPLL_EN_EN, EFR32_DPLL0_EN_CLR);

  while ((getreg32(EFR32_DPLL0_EN) & DPLL_EN_DISABLING) != 0)
    {
    }

  /* Factory calibration of the band, undivided */

  while ((getreg32(EFR32_HFRCO0_STATUS) &
          (HFRCO_STATUS_SYNCBUSY | HFRCO_STATUS_FREQBSY)) != 0)
    {
    }

  regval  = getreg32(EFR32_DEVINFO_HFRCODPLLCAL(HFRCODPLL_DEVINFO_BAND));
  regval &= ~HFRCO_CAL_CLKDIV_MASK;
  putreg32(regval, EFR32_HFRCO0_CAL);

  /* SYSCLK = HFXO * (N + 1) / (M + 1), phase lock on the falling edge */

  putreg32(DPLL_CFG1_N(BOARD_DPLL_N) | DPLL_CFG1_M(BOARD_DPLL_M),
           EFR32_DPLL0_CFG1);
  modifyreg32(EFR32_CMU_DPLLREFCLKCTRL, CMU_DPLLREFCLKCTRL_CLKSEL_MASK,
              CMU_DPLLREFCLKCTRL_CLKSEL_HFXO);
  putreg32(DPLL_CFG_MODE_PLL | DPLL_CFG_AUTORECOVER, EFR32_DPLL0_CFG);

  /* Enable and wait for the lock.  If the lock fails, the DPLL retries
   * on its own (AUTORECOVER) and the HFRCODPLL keeps the calibrated band
   * frequency meanwhile.
   */

  putreg32(DPLL_IF_LOCK | DPLL_IF_LOCKFAILLOW | DPLL_IF_LOCKFAILHIGH,
           EFR32_DPLL0_IF_CLR);
  putreg32(DPLL_EN_EN, EFR32_DPLL0_EN_SET);

  while ((getreg32(EFR32_DPLL0_IF) &
          (DPLL_IF_LOCK | DPLL_IF_LOCKFAILLOW | DPLL_IF_LOCKFAILHIGH)) == 0)
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: efr32_clockconfig
 *
 * Description:
 *   Called early in the boot sequence to start the HFXO, lock the DPLL
 *   and switch SYSCLK to the board's frequency (board.h).  It also enables
 *   the clocks of the GPIO block and of the flash controller.
 *
 ****************************************************************************/

void efr32_clockconfig(void)
{
  uint32_t regval;

  /* Bus clocks of the blocks used here and by the GPIO driver */

  putreg32(CMU_CLKEN0_GPIO | CMU_CLKEN0_HFXO0 | CMU_CLKEN0_DPLL0 |
           CMU_CLKEN0_HFRCO0 | CMU_CLKEN0_FSRCO, EFR32_CMU_CLKEN0_SET);
  putreg32(CMU_CLKEN1_MSC, EFR32_CMU_CLKEN1_SET);

  /* Run from the FSRCO while the HFRCODPLL is being reconfigured, so that
   * its overshoot cannot overclock the core.
   */

  modifyreg32(EFR32_CMU_SYSCLKCTRL, CMU_SYSCLKCTRL_CLKSEL_MASK,
              CMU_SYSCLKCTRL_CLKSEL_FSRCO);

  /* Flash wait states for the new frequency (the FSRCO is slower) */

  putreg32(MSC_LOCK_LOCKKEY_UNLOCK, EFR32_MSC_LOCK);
  modifyreg32(EFR32_MSC_READCTRL, MSC_READCTRL_MODE_MASK,
              MSC_READCTRL_MODE);

  efr32_hfxo_start();
  efr32_dpll_lock();

  /* Set the prescalers first, then switch SYSCLK to the HFRCODPLL */

  regval  = getreg32(EFR32_CMU_SYSCLKCTRL);
  regval &= ~(CMU_SYSCLKCTRL_HCLKPRESC_MASK | CMU_SYSCLKCTRL_PCLKPRESC |
              CMU_SYSCLKCTRL_RHCLKPRESC);
  regval |= SYSCLKCTRL_HCLKPRESC | SYSCLKCTRL_PCLKPRESC |
            SYSCLKCTRL_RHCLKPRESC;
  putreg32(regval, EFR32_CMU_SYSCLKCTRL);

  modifyreg32(EFR32_CMU_SYSCLKCTRL, CMU_SYSCLKCTRL_CLKSEL_MASK,
              CMU_SYSCLKCTRL_CLKSEL_HFRCODPLL);
}
