/****************************************************************************
 * arch/arm/src/tiva/tm4c/tm4c129_sysctrl.c
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
#include <debug.h>

#include <nuttx/init.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "tiva_sysctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if XTAL_FREQUENCY < 5000000 || XTAL_FREQUENCY > 25000000
#  error Crystal frequency is not supported
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure supports mapping of a frequency to optimal memory timing */

struct f2memtim0_s
{
  uint32_t frequency;
  uint32_t memtim0;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure supports mapping of a frequency to optimal memory timing */

static const struct f2memtim0_s g_f2memtim0[] =
{
  {
    16000000,
    (SYSCON_MEMTIM0_FWS(0) | SYSCON_MEMTIM0_FBCE | SYSCON_MEMTIM0_FBCHT_0p5 |
     SYSCON_MEMTIM0_EWS(0) | SYSCON_MEMTIM0_EBCE | SYSCON_MEMTIM0_EBCHT_0p5 |
     SYSCON_MEMTIM0_MB1)
  },
  {
    40000000,
    (SYSCON_MEMTIM0_FWS(1) | SYSCON_MEMTIM0_FBCHT_1p5 |
     SYSCON_MEMTIM0_EWS(1) | SYSCON_MEMTIM0_EBCHT_1p5 |
     SYSCON_MEMTIM0_MB1)
  },
  {
    60000000,
    (SYSCON_MEMTIM0_FWS(2) | SYSCON_MEMTIM0_FBCHT_2 |
     SYSCON_MEMTIM0_EWS(2) | SYSCON_MEMTIM0_EBCHT_2 |
     SYSCON_MEMTIM0_MB1)
  },
  {
    80000000,
    (SYSCON_MEMTIM0_FWS(3) | SYSCON_MEMTIM0_FBCHT_2p5 |
     SYSCON_MEMTIM0_EWS(3) | SYSCON_MEMTIM0_EBCHT_2p5 |
     SYSCON_MEMTIM0_MB1)
  },
  {
    100000000,
    (SYSCON_MEMTIM0_FWS(4) | SYSCON_MEMTIM0_FBCHT_3 |
     SYSCON_MEMTIM0_EWS(4) | SYSCON_MEMTIM0_EBCHT_3 |
     SYSCON_MEMTIM0_MB1)
  },
  {
    120000000,
    (SYSCON_MEMTIM0_FWS(5) | SYSCON_MEMTIM0_FBCHT_3p5 |
     SYSCON_MEMTIM0_EWS(5) | SYSCON_MEMTIM0_EBCHT_3p5 |
     SYSCON_MEMTIM0_MB1)
  },
};

#define NMEMTIM0_SETTINGS (sizeof(g_f2memtim0) / sizeof(struct f2memtim0_s))

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_memtim0
 *
 * Description:
 *   Given a SysClk frequency, perform a table lookup to select the optimal
 *   FLASH and EEPROM configuration for the MEMTIM0 register.
 *
 ****************************************************************************/

static uint32_t tiva_memtim0(uint32_t sysclk)
{
  int i;

  /* Search for the optimal memory timing */

  for (i = 0; i < NMEMTIM0_SETTINGS; i++)
    {
      /* Check if  if the SysClk is less than the maximum frequency for this
       * flash memory timing.
       */

      if (sysclk <= g_f2memtim0[i].frequency)
        {
          /* Yes.. then this FLASH memory timing is the best choice for the
           * given system clock frequency.
           */

          return(g_f2memtim0[i].memtim0);
        }
    }

  /* No appropriate flash memory timing could be found.  The device is
   * being clocked too fast.
   */

  DEBUGPANIC();
  return 0;
}

/****************************************************************************
 * Name: tiva_vco_frequency
 *
 * Description:
 *   Given the crystal frequency and the PLLFREQ0 and PLLFREQ1 register
 *   settings, return the SysClk frequency.
 *
 ****************************************************************************/

static uint32_t tiva_vco_frequency(uint32_t pllfreq0, uint32_t pllfreq1)
{
  uint64_t fvcob10;
  uint32_t mint;
  uint32_t mfrac;
  uint32_t q;
  uint32_t n;
  uint32_t mdivb10;

  /* Extract all of the values from the hardware register values. */

  mfrac =  (pllfreq0 & SYSCON_PLLFREQ0_MFRAC_MASK) >>
           SYSCON_PLLFREQ0_MFRAC_SHIFT;

  mint  =  (pllfreq0 & SYSCON_PLLFREQ0_MINT_MASK) >>
           SYSCON_PLLFREQ0_MINT_SHIFT;

  q     = ((pllfreq1 & SYSCON_PLLFREQ1_Q_MASK) >>
           SYSCON_PLLFREQ1_Q_SHIFT) + 1;

  n     = ((pllfreq1 & SYSCON_PLLFREQ1_N_MASK) >>
           SYSCON_PLLFREQ1_N_SHIFT) + 1;

  /* Algorithm:
   *
   *     Fin  = Fxtal / Q / N (-OR- Fpiosc / Q / N)
   *     Mdiv = Mint + (MFrac / 1024)
   *     Fvco = Fin * Mdiv
   */

  mdivb10 = (mint << 10) + mfrac;
  fvcob10 = (mdivb10 * (uint64_t)XTAL_FREQUENCY) / (q * n);
  return (uint32_t)(fvcob10 >> 10);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_clock_reconfigure
 *
 * Description:
 *   Called to change to new clock based on desired pllfreq0, pllfreq1, and
 *   sysdiv settings.  This is use to set up the initial clocking but can be
 *   used later to support slow clocked, low power consumption modes.
 *
 *   The pllfreq0 and pllfreq1 settings derive from the PLL M, N, and Q
 *   values to generate Fvco like:
 *
 *     Fin  = Fxtal / Q / N -OR- Fpiosc / Q / N
 *     Mdiv = Mint + (MFrac / 1024)
 *     Fvco = Fin * Mdiv
 *
 *   When the PLL is active, the system clock frequency (SysClk) is
 *   calculated using the following equation:
 *
 *     SysClk = Fvco/ sysdiv
 *
 *   NOTE: The input clock to the PLL may be either the external crystal
 *   (Fxtal) or PIOSC (Fpiosc).  This logic supports only the external
 *   crystal as the PLL source clock.
 *
 * Input Parameters:
 *   pllfreq0 - PLLFREQ0 register value (see helper macro M2PLLFREQ0()
 *   pllfreq1 - PLLFREQ1 register value (see helper macro QN2PLLFREQ1()
 *   sysdiv   - Fvco divider value
 *
 * Returned Value:
 *   The resulting SysClk frequency
 *
 ****************************************************************************/

uint32_t tiva_clock_reconfigure(uint32_t pllfreq0, uint32_t pllfreq1,
                                uint32_t sysdiv)
{
  uint32_t sysclk;
  uint32_t regval;
  int32_t timeout;
  bool newpll;

  /* Clear MOSC power down, high oscillator range setting, and no crystal
   * present settings.
   */

  regval  = getreg32(TIVA_SYSCON_MOSCCTL);
  regval &= ~(SYSCON_MOSCCTL_OSCRNG | SYSCON_MOSCCTL_PWRDN |
              SYSCON_MOSCCTL_NOXTAL);

#if XTAL_FREQUENCY >= 10000000
  /* Increase the drive strength for MOSC of 10 MHz and above. */

  regval |= SYSCON_MOSCCTL_OSCRNG;
#endif

  putreg32(regval, TIVA_SYSCON_MOSCCTL);

  /* Set the memory timings for the maximum external frequency since this
   * could be a switch to PIOSC or possibly to MOSC which can be up to
   * 25MHz.
   */

  regval = tiva_memtim0(25000000);
  putreg32(regval, TIVA_SYSCON_MEMTIM0);

  /* Clear any previous PLL divider and source setup.  Update the clock
   * configuration to switch back to PIOSC.
   */

  regval  = getreg32(TIVA_SYSCON_RSCLKCFG);
  regval &= ~(SYSCON_RSCLKCFG_PSYSDIV_MASK | SYSCON_RSCLKCFG_OSCSRC_MASK |
              SYSCON_RSCLKCFG_PLLSRC_MASK | SYSCON_RSCLKCFG_USEPLL);
  regval |= SYSCON_RSCLKCFG_MEMTIMU;
  putreg32(regval, TIVA_SYSCON_RSCLKCFG);

  /* If there were no changes to the PLL do not force the PLL to lock by
   * writing the PLL settings.
   */

  newpll = (getreg32(TIVA_SYSCON_PLLFREQ1) != pllfreq1 ||
            getreg32(TIVA_SYSCON_PLLFREQ0) != pllfreq0);

  /* If there are new PLL settings write them. */

  if (newpll)
    {
      /* Set the oscillator source. */

      regval  = getreg32(TIVA_SYSCON_RSCLKCFG);
      regval |= (SYSCON_RSCLKCFG_OSCSRC_MOSC | SYSCON_RSCLKCFG_PLLSRC_MOSC);
      putreg32(regval, TIVA_SYSCON_RSCLKCFG);

      /* Set the M, N and Q values provided by the pllfreq0 and pllfreq1
       * parameters.
       */

      putreg32(pllfreq1, TIVA_SYSCON_PLLFREQ1);

      regval    = getreg32(TIVA_SYSCON_PLLFREQ0);
      regval   &= SYSCON_PLLFREQ0_PLLPWR;
      pllfreq0 |= regval;
      putreg32(pllfreq0, TIVA_SYSCON_PLLFREQ0);
    }

  /* Calculate the actual system clock. */

  sysclk = tiva_vco_frequency(pllfreq0, pllfreq1) / sysdiv;

  /* Set the Flash and EEPROM timing values. */

  regval = tiva_memtim0(sysclk);
  putreg32(regval, TIVA_SYSCON_MEMTIM0);

  /* Was the PLL already powered up? */

  if ((getreg32(TIVA_SYSCON_PLLFREQ0) & SYSCON_PLLFREQ0_PLLPWR) != 0)
    {
      /* Yes.. Is this a new PLL setting? */

      if (newpll == true)
        {
          /* Yes.. Trigger the PLL to lock to the new frequency. */

          regval  = getreg32(TIVA_SYSCON_RSCLKCFG);
          regval |= SYSCON_RSCLKCFG_NEWFREQ;
          putreg32(regval, TIVA_SYSCON_RSCLKCFG);
        }
    }
  else
    {
      /* No... Not already powered.  Power up the PLL now. */

      regval  = getreg32(TIVA_SYSCON_PLLFREQ0);
      regval |= SYSCON_PLLFREQ0_PLLPWR;
      putreg32(regval, TIVA_SYSCON_PLLFREQ0);
    }

  /* Wait until the PLL has locked. */

  for (timeout = 32768; timeout > 0; timeout--)
    {
      /* Check if the PLL has locked */

      if ((getreg32(TIVA_SYSCON_PLLSTAT) & SYSCON_PLLSTAT_LOCK) != 0)
        {
          /* The PLL has reported that it is locked.  Switch over to the
           * PLL.
           */

          regval  = getreg32(TIVA_SYSCON_RSCLKCFG);
          regval |= SYSCON_RSCLKCFG_PSYSDIV(sysdiv - 1) |
                    SYSCON_RSCLKCFG_OSCSRC_MOSC |
                    SYSCON_RSCLKCFG_PLLSRC_MOSC |
                    SYSCON_RSCLKCFG_USEPLL |
                    SYSCON_RSCLKCFG_MEMTIMU;
          putreg32(regval, TIVA_SYSCON_RSCLKCFG);

          /* And return the new SysClk frequency */

          return sysclk;
        }
    }

  /* We get here on a timeout, failing to get the PLL lock indication */

  DEBUGPANIC();
  return 0;
}

/****************************************************************************
 * Name: tiva_clock_configure
 *
 * Description:
 *   Called early in the boot sequence (before .data and .bss are available)
 *   in order to configure initial clocking.
 *
 ****************************************************************************/

void tiva_clock_configure(void)
{
  uint32_t pllfreq0;
  uint32_t pllfreq1;

  /* Set the clocking to run with the default settings provided in the
   * board.h header file
   */

  pllfreq0 = M2PLLFREQ0(BOARD_PLL_MINT, BOARD_PLL_MFRAC);
  pllfreq1 = QN2PLLFREQ1(BOARD_PLL_Q, BOARD_PLL_N);
  tiva_clock_reconfigure(pllfreq0, pllfreq1, BOARD_PLL_SYSDIV);

  /* Set up the alternate clock source
   *
   * The ALTCLK provides a clock source of numerous frequencies to the
   * general-purpose timer, SSI, and UART modules.  The default source for
   * the ALTCLK is the Precision Internal Oscillator (PIOSC).  The
   * Hibernation Real-time Clock (RTCOSC) and Low Frequency Internal
   * Oscillator (LFIOSC) are alternatives.  If the RTCOSC Output is
   * selected, the clock source must also be enabled in the Hibernation
   * module.
   */

  putreg32(BOARD_ALTCLKCFG, TIVA_SYSCON_ALTCLKCFG);
}
