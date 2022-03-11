/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_clockconfig.c
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

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"
#include "hardware/rv32m1_scg.h"
#include "hardware/rv32m1_smc.h"
#include "rv32m1_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rv32m1_modifyreg32
 ****************************************************************************/

static void rv32m1_modifyreg32(uint32_t addr, uint32_t bitclr,
                               uint32_t bitset)
{
  uint32_t regval = getreg32(addr);
  regval &= ~bitclr;
  regval |= bitset;
  putreg32(regval, addr);
}

/****************************************************************************
 * Name: rv32m1_sircfreq
 ****************************************************************************/

static unsigned rv32m1_sircfreq(void)
{
  uint32_t regval = getreg32(RV32M1_SCG_SIRCCSR);

  /* If the SIRC is Invalid or Disabled */

  if (!(regval & SCG_SIRCCSR_VLD))
    {
      return 0u;
    }

  regval = getreg32(RV32M1_SCG_SIRCCFG) & SCG_SIRCCFG_RANGE_MASK;
  if (regval == SCG_SIRCCFG_RANGE_8MHZ)
    {
      return 8000000u;
    }

  /* SIRC provides 2 options of frequency: 8MHz and 2MHz,
   * In this case, it has to be 2MHz.
   */

  return 2000000u;
}

/****************************************************************************
 * Name: rv32m1_fircfreq
 ****************************************************************************/

static unsigned rv32m1_fircfreq(void)
{
  uint32_t regval = getreg32(RV32M1_SCG_FIRCCSR);

  /* If the FIRC is Invalid or Disabled */

  if (!(regval & SCG_FIRCCSR_VLD))
    {
      return 0u;
    }

  regval = getreg32(RV32M1_SCG_FIRCCFG) & SCG_FIRCCFG_RANGE_MASK;
  if (regval == SCG_FIRCCFG_RANGE_48MHZ)
    {
      return 48000000u;
    }

  if (regval == SCG_FIRCCFG_RANGE_52MHZ)
    {
      return 52000000u;
    }

  if (regval == SCG_FIRCCFG_RANGE_56MHZ)
    {
      return 56000000u;
    }

  /* FIRC provides 4 options of frequency: 48MHz, 52MHz, 56MHz, 60MHz,
   * In this case, it has to be 60MHz.
   */

  return 60000000u;
}

/****************************************************************************
 * Name: rv32m1_lpfllfreq
 ****************************************************************************/

static unsigned rv32m1_lpfllfreq(void)
{
  uint32_t regval = getreg32(RV32M1_SCG_LPFLLCSR);

  /* If the LPFLL is Invalid or Disabled */

  if (!(regval & SCG_LPFLLCSR_VLD))
    {
      return 0u;
    }

  regval = getreg32(RV32M1_SCG_LPFLLCFG) & SCG_LPFLLCFG_RANGE_MASK;
  if (regval == SCG_LPFLLCFG_RANGE_48MHZ)
    {
      return 48000000u;
    }

  /* LPFLL provides 2 options of frequency: 48MHz, 72MHz,
   * In this case, it has to be 72MHz.
   */

  return 72000000u;
}

/****************************************************************************
 * Name: rv32m1_soscfreq
 ****************************************************************************/

static unsigned rv32m1_soscfreq(void)
{
  uint32_t regval = getreg32(RV32M1_SCG_SOSCCSR);

  /* If the SOSC is Invalid or Disabled */

  if (!(regval & SCG_SOSCCSR_VLD))
    {
      return 0u;
    }

#ifdef RV32M1_BOARD_XTAL
  return RV32M1_BOARD_XTAL;
#else
  return 0u;
#endif
}

/****************************************************************************
 * Name: rv32m1_lpocfreq
 ****************************************************************************/

static unsigned rv32m1_lpocfreq(void)
{
  return 1000u;
}

/****************************************************************************
 * Name: rv32m1_roscfreq
 ****************************************************************************/

static unsigned rv32m1_roscfreq(void)
{
  uint32_t regval = getreg32(RV32M1_SCG_ROSCCSR);

  /* If the ROSC is Invalid or Disabled */

  if (!(regval & SCG_ROSCCSR_VLD))
    {
      return 0;
    }

  return 32768u;
}

/****************************************************************************
 * Name: rv32m1_corefreq
 ****************************************************************************/

static unsigned rv32m1_corefreq(void)
{
  uint32_t scs;
  uint32_t div;

  uint32_t regval = getreg32(RV32M1_SCG_ROSCCSR);
  uint32_t freq = 0;

  scs = regval & SCG_CSR_SCS_MASK;

  switch (scs)
    {
      case SCG_CSR_SCS_SOSC:
        freq = rv32m1_soscfreq();
        break;

      case SCG_CSR_SCS_SIRC:
        freq = rv32m1_sircfreq();
        break;

      case SCG_CSR_SCS_FIRC:
        freq = rv32m1_fircfreq();
        break;

      case SCG_CSR_SCS_ROSC:
        freq = rv32m1_roscfreq();
        break;

      case SCG_CSR_SCS_LPFLL:
        freq = rv32m1_lpfllfreq();
        break;

      default:
        freq = 0;
        break;
    }

  div = (regval & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT;
  div += 1;

  return freq / div;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rv32m1_clockconfig
 ****************************************************************************/

void rv32m1_clockconfig(void)
{
  /* Initialize SIRC */

  putreg32(SCG_SIRCDIV_DIV1_DISABLED |
           SCG_SIRCDIV_DIV2_DIVBY1   |
           SCG_SIRCDIV_DIV3_DIVBY1 ,
           RV32M1_SCG_SIRCDIV);

  /* 8MHZ is wanted */

  putreg32(SCG_SIRCCFG_RANGE_8MHZ, RV32M1_SCG_SIRCCFG);

  /* Enable SIRC with Lower Power Mode support */

  rv32m1_modifyreg32(RV32M1_SCG_SIRCCSR, 0,
                     SCG_SIRCCSR_EN | SCG_SIRCCSR_LPEN);

  /* Wait for SIRC to be valid */

  while (!(getreg32(RV32M1_SCG_SIRCCSR) & SCG_SIRCCSR_VLD)) ;

  /* Switch to SIRC clock as System Clock */

  putreg32(SCG_RCCR_SCS_SIRC     |
           SCG_RCCR_DIVCORE_DIV1 |
           SCG_RCCR_DIVSLOW_DIV4 ,
           RV32M1_SCG_RCCR);

  /* Wait for SIRC as System Clock */

  while ((getreg32(RV32M1_SCG_CSR) & SCG_CSR_SCS_MASK) !=
         SCG_CSR_SCS_SIRC);

  /* Initialize FIRC */

  putreg32(SCG_FIRCDIV_DIV1_DIVBY1 |
           SCG_FIRCDIV_DIV2_DIVBY1 |
           SCG_FIRCDIV_DIV3_DIVBY1 ,
           RV32M1_SCG_FIRCDIV);

  /* 48MHz is wanted */

  putreg32(SCG_FIRCCFG_RANGE_48MHZ, RV32M1_SCG_FIRCCFG);

  /* Enable FIRC */

  rv32m1_modifyreg32(RV32M1_SCG_FIRCCSR, 0, SCG_FIRCCSR_EN);

  /* Wait for FIRC to be valid */

  while (!(getreg32(RV32M1_SCG_FIRCCSR) & SCG_FIRCCSR_VLD)) ;

  /* Switch FIRC as the RUN Mode System Clock */

  putreg32(SCG_RCCR_SCS_FIRC     |
           SCG_RCCR_DIVCORE_DIV1 |
           SCG_RCCR_DIVBUS_DIV1  |
           SCG_RCCR_DIVEXT_DIV1  |
           SCG_RCCR_DIVSLOW_DIV4 ,
           RV32M1_SCG_RCCR);

  /* Wait for FIRC as System Clock */

  while ((getreg32(RV32M1_SCG_CSR) & SCG_CSR_SCS_MASK) !=
         SCG_CSR_SCS_FIRC);

  /* Prepare LPFLL for HSRUN Mode */

  putreg32(SCG_LPFLLDIV_DIV1_DIVBY1 |
           SCG_LPFLLDIV_DIV2_DIVBY1 |
           SCG_LPFLLDIV_DIV3_DIVBY1 ,
           RV32M1_SCG_LPFLLDIV);

  /* 72MHz is wanted */

  putreg32(SCG_LPFLLCFG_RANGE_72MHZ, RV32M1_SCG_LPFLLCFG);

  /* Trim LPFLL input source */

  rv32m1_modifyreg32(RV32M1_SCG_LPFLLTCFG,
                     SCG_LPFLLTCFG_TRIMDIV_MASK |
                     SCG_LPFLLTCFG_TRIMSRC_MASK,
                     SCG_LPFLLTCFG_TRIMDIV_BY4  |
                     SCG_LPFLLTCFG_TRIMSRC_SIRC);

  /* Enable LPFLL */

  rv32m1_modifyreg32(RV32M1_SCG_LPFLLCSR, 0, SCG_LPFLLCSR_EN);

  /* Wait for LPFLL to be valid */

  while (!(getreg32(RV32M1_SCG_LPFLLCSR) & SCG_LPFLLCSR_VLD)) ;

  /* Set LPFLL as the HSRUN Mode System Clock */

  putreg32(SCG_HCCR_SCS_LPFLL    |
           SCG_HCCR_DIVCORE_DIV1 |
           SCG_HCCR_DIVBUS_DIV1  |
           SCG_HCCR_DIVEXT_DIV1  |
           SCG_HCCR_DIVSLOW_DIV4 ,
           RV32M1_SCG_HCCR);

  /* Remove the power mode protection */

  putreg32(SMC_PMPROT_PM_ALL_ALLOWED, RV32M1_SMC_PMPROT);

  /* Enable HSRUN Mode */

  rv32m1_modifyreg32(RV32M1_SMC_PMCTRL,
                     SMC_PMCTRL_RUNM_MASK,
                     SMC_PMCTRL_RUNM_HSRUN);

  /* Wait for High Speed Run Mode stable */

  while ((getreg32(RV32M1_SMC_PMSTAT) & SMC_PMSTAT_PMSTAT_MASK) !=
         SMC_PMSTAT_HSRUN);

  /* Wait for LPFLL as System Clock */

  while ((getreg32(RV32M1_SCG_CSR) & SCG_CSR_SCS_MASK) !=
         SCG_CSR_SCS_LPFLL);
}

/****************************************************************************
 * Name: rv32m1_clockfreq
 *
 * Description:
 *   Query the frequecy of a given clock source.
 *
 ****************************************************************************/

unsigned rv32m1_clockfreq(enum clk_e clk)
{
  uint32_t freq;
  uint32_t div;

  switch (clk)
    {
      case CLK_SIRC:
        return rv32m1_sircfreq();

      case CLK_SIRCDIV1:
        {
          freq = rv32m1_sircfreq();
          div = (getreg32(RV32M1_SCG_SIRCDIV) & SCG_SIRCDIV_DIV1_MASK) >>
                 SCG_SIRCDIV_DIV1_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_SIRCDIV2:
        {
          freq = rv32m1_sircfreq();
          div = (getreg32(RV32M1_SCG_SIRCDIV) & SCG_SIRCDIV_DIV2_MASK) >>
                 SCG_SIRCDIV_DIV2_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_SIRCDIV3:
        {
          freq = rv32m1_sircfreq();
          div = (getreg32(RV32M1_SCG_SIRCDIV) & SCG_SIRCDIV_DIV3_MASK) >>
                 SCG_SIRCDIV_DIV3_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_FIRC:
        return freq = rv32m1_fircfreq();

      case CLK_FIRCDIV1:
        {
          freq = rv32m1_fircfreq();
          div = (getreg32(RV32M1_SCG_FIRCDIV) & SCG_FIRCDIV_DIV1_MASK) >>
                 SCG_FIRCDIV_DIV1_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_FIRCDIV2:
        {
          freq = rv32m1_fircfreq();
          div = (getreg32(RV32M1_SCG_FIRCDIV) & SCG_FIRCDIV_DIV2_MASK) >>
                 SCG_FIRCDIV_DIV2_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_FIRCDIV3:
        {
          freq = rv32m1_fircfreq();
          div = (getreg32(RV32M1_SCG_FIRCDIV) & SCG_FIRCDIV_DIV3_MASK) >>
                 SCG_FIRCDIV_DIV3_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_LPFLL:
        return freq = rv32m1_lpfllfreq();

      case CLK_LPFLLDIV1:
        {
          freq = rv32m1_lpfllfreq();
          div = (getreg32(RV32M1_SCG_LPFLLDIV) & SCG_LPFLLDIV_DIV1_MASK) >>
                 SCG_LPFLLDIV_DIV1_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_LPFLLDIV2:
        {
          freq = rv32m1_lpfllfreq();
          div = (getreg32(RV32M1_SCG_LPFLLDIV) & SCG_LPFLLDIV_DIV2_MASK) >>
                 SCG_LPFLLDIV_DIV2_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_LPFLLDIV3:
        {
          freq = rv32m1_lpfllfreq();
          div = (getreg32(RV32M1_SCG_LPFLLDIV) & SCG_LPFLLDIV_DIV3_MASK) >>
                 SCG_LPFLLDIV_DIV3_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_SOSC:
        return rv32m1_soscfreq();

      case CLK_SOSCDIV1:
        {
          freq = rv32m1_soscfreq();
          div = (getreg32(RV32M1_SCG_SOSCDIV) & SCG_SOSCDIV_DIV1_MASK) >>
                 SCG_LPFLLDIV_DIV1_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_SOSCDIV2:
        {
          freq = rv32m1_soscfreq();
          div = (getreg32(RV32M1_SCG_SOSCDIV) & SCG_SOSCDIV_DIV2_MASK) >>
                 SCG_LPFLLDIV_DIV2_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_SOSCDIV3:
        {
          freq = rv32m1_soscfreq();
          div = (getreg32(RV32M1_SCG_SOSCDIV) & SCG_SOSCDIV_DIV3_MASK) >>
                 SCG_LPFLLDIV_DIV3_SHIFT;

          /* If div is Zero, the clock source is disabled */

          if (div == 0)
            {
              return 0u;
            }

          return freq / (1 << (div - 1));
        }
        break;

      case CLK_LPOC:
        return rv32m1_lpocfreq();

      case CLK_ROSC:
        return rv32m1_roscfreq();

      case CLK_CORE:
      case CLK_PLAT:
      case CLK_SYS:
        return rv32m1_corefreq();

      case CLK_BUS:
        {
          freq = rv32m1_corefreq();
          div = (getreg32(RV32M1_SCG_CSR) & SCG_CSR_DIVBUS_MASK) >>
                 SCG_CSR_DIVBUS_SHIFT;

          div += 1;
          return freq / div;
        }
        break;

      case CLK_EXT:
        {
          freq = rv32m1_corefreq();
          div = (getreg32(RV32M1_SCG_CSR) & SCG_CSR_DIVEXT_MASK) >>
                 SCG_CSR_DIVEXT_SHIFT;

          div += 1;
          return freq / div;
        }
        break;

      case CLK_SLOW:
        {
          freq = rv32m1_corefreq();
          div = (getreg32(RV32M1_SCG_CSR) & SCG_CSR_DIVSLOW_MASK) >>
                 SCG_CSR_DIVSLOW_SHIFT;

          div += 1;
          return freq / div;
        }
        break;

      default:
        return 0u;
    }

  return 0u;
}
