/****************************************************************************
 * arch/arm/src/gd32w51x/gd32w51x_clockconfig.c
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
#include "gd32w51x_clockconfig.h"
#include "hardware/gd32w51x_rcu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PLL: 1 MHz VCO input, x360, /2 */

#define PLL_PSC                 (GD32W51X_HXTAL_FREQ / 1000000)
#define PLL_N                   360
#define PLL_P                   2

#if (GD32W51X_HXTAL_FREQ % 1000000) != 0 || PLL_PSC < 2 || PLL_PSC > 63
#  error "BOARD_HXTAL_FREQUENCY must be a whole number of MHz, 2..63 MHz"
#endif

/* PMU control register 0: the LDO output voltage must be raised before
 * running above 120 MHz.
 */

#define GD32W51X_PMU_CTL0       (GD32W51X_PMU_BASE + 0x0000)
#define PMU_CTL0_LDOVS_MASK     (3 << 14)

/* The HXTAL needs a short delay between power up and the READY handshake
 * (HXTALSTB_DELAY in the vendor library).
 */

#define HXTAL_PU_DELAY          0x2000
#define PLL_EXIT_DELAY          0x50
#define HXTAL_STARTUP_TIMEOUT   0xffff

/* ICACHE control register */

#define GD32W51X_ICACHE_CTL     (GD32W51X_ICACHE_BASE + 0x0000)
#define ICACHE_CTL_EN           (1 << 0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32w51x_clockconfig
 ****************************************************************************/

void gd32w51x_clockconfig(void)
{
  volatile uint32_t delay;
  uint32_t timeout;
  uint32_t regval;

  /* Run from the IRC16M while the PLL is (re)configured: a bootloader
   * (or the debugger) may have left the PLL selected.
   */

  modifyreg32(GD32W51X_RCU_CTL, 0, RCU_CTL_IRC16MEN);
  while ((getreg32(GD32W51X_RCU_CTL) & RCU_CTL_IRC16MSTB) == 0)
    {
    }

  if ((getreg32(GD32W51X_RCU_CFG0) & RCU_CFG0_SCSS_MASK) ==
      RCU_CFG0_SCSS_PLLP)
    {
      /* Step the AHB clock down before leaving the PLL, as the vendor
       * library does, so that the core voltage does not dip.
       */

      for (delay = 0; delay < PLL_EXIT_DELAY; delay++)
        {
        }

      modifyreg32(GD32W51X_RCU_CFG0, RCU_CFG0_AHBPSC_MASK,
                  RCU_CFG0_AHBPSC_DIV2);

      for (delay = 0; delay < PLL_EXIT_DELAY; delay++)
        {
        }

      modifyreg32(GD32W51X_RCU_CFG0, RCU_CFG0_AHBPSC_MASK,
                  RCU_CFG0_AHBPSC_DIV4);

      for (delay = 0; delay < PLL_EXIT_DELAY; delay++)
        {
        }
    }

  modifyreg32(GD32W51X_RCU_CFG0, RCU_CFG0_SCS_MASK, RCU_CFG0_SCS_IRC16M);
  while ((getreg32(GD32W51X_RCU_CFG0) & RCU_CFG0_SCSS_MASK) != 0)
    {
    }

  modifyreg32(GD32W51X_RCU_CTL, RCU_CTL_PLLEN, 0);

  /* Power up and enable the HXTAL, for the PLL */

  modifyreg32(GD32W51X_RCU_CTL, 0, RCU_CTL_HXTALPU);
  modifyreg32(GD32W51X_RCU_CTL, 0, RCU_CTL_HXTALEN);
  modifyreg32(GD32W51X_RCU_CTL, 0, RCU_CTL_HXTALENPLL);

  for (delay = 0; delay < HXTAL_PU_DELAY; delay++)
    {
    }

  modifyreg32(GD32W51X_RCU_CTL, 0, RCU_CTL_HXTALREADY);

  timeout = 0;
  while ((getreg32(GD32W51X_RCU_CTL) & RCU_CTL_HXTALSTB) == 0)
    {
      if (++timeout >= HXTAL_STARTUP_TIMEOUT)
        {
          /* No crystal: stay on the IRC16M.  Nothing will be timed right,
           * but the console still comes up.
           */

          return;
        }
    }

  /* Raise the core voltage for 180 MHz */

  modifyreg32(GD32W51X_RCU_APB1EN, 0, RCU_APB1EN_PMUEN);
  modifyreg32(GD32W51X_PMU_CTL0, 0, PMU_CTL0_LDOVS_MASK);

  /* AHB = SYSCLK, APB2 = AHB / 2, APB1 = AHB / 4 */

  regval  = getreg32(GD32W51X_RCU_CFG0);
  regval &= ~(RCU_CFG0_AHBPSC_MASK | RCU_CFG0_APB1PSC_MASK |
              RCU_CFG0_APB2PSC_MASK);
  regval |= RCU_CFG0_AHBPSC_DIV1 | RCU_CFG0_APB2PSC_DIV2 |
            RCU_CFG0_APB1PSC_DIV4;
  putreg32(regval, GD32W51X_RCU_CFG0);

  /* Main PLL from the HXTAL */

  putreg32((PLL_PSC << RCU_PLL_PLLPSC_SHIFT) |
           (PLL_N << RCU_PLL_PLLN_SHIFT) |
           (((PLL_P / 2) - 1) << RCU_PLL_PLLP_SHIFT) |
           RCU_PLL_PLLSEL, GD32W51X_RCU_PLL);

  modifyreg32(GD32W51X_RCU_CTL, 0, RCU_CTL_PLLEN);
  while ((getreg32(GD32W51X_RCU_CTL) & RCU_CTL_PLLSTB) == 0)
    {
    }

  /* Switch SYSCLK to the PLL */

  modifyreg32(GD32W51X_RCU_CFG0, RCU_CFG0_SCS_MASK, RCU_CFG0_SCS_PLLP);
  while ((getreg32(GD32W51X_RCU_CFG0) & RCU_CFG0_SCSS_MASK) !=
         RCU_CFG0_SCSS_PLLP)
    {
    }
}

/****************************************************************************
 * Name: gd32w51x_icache_enable
 ****************************************************************************/

void gd32w51x_icache_enable(void)
{
  modifyreg32(GD32W51X_ICACHE_CTL, 0, ICACHE_CTL_EN);
}
