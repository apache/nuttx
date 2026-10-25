/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_clockconfig.c
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

#include "riscv_internal.h"
#include "gd32vw55x_clockconfig.h"
#include "hardware/gd32vw55x_rcu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SysTimer registers (Nuclei core-local, not in the RCU) */

#define GD32VW55X_SYSTIMER_MTIME     (GD32VW55X_SYSTIMER_BASE + 0x0000)
#define GD32VW55X_SYSTIMER_MTIMECTL  (GD32VW55X_SYSTIMER_BASE + 0x0ff8)
#define SYSTIMER_MTIMECTL_CLKSRC     (1 << 2)  /* 1: mtime = sysclock */

/* The HXTAL stable flag never sets, and the firmware library does not wait
 * for it.  But the PLL locked to the crystal while it starts runs about 12
 * percent slow for about 0.5 ms (measured after deep-sleep, against the
 * RTC): wait for the crystal on the IRC16M, counted by the system timer.
 */

#define GD32VW55X_IRC16M_FREQ        16000000
#define GD32VW55X_HXTAL_SETTLE_US    1000

/* Busy loop counts while the system clock changes: after each AHB
 * prescaler step (RCU_MODIFY(0x50)) and after the switch back to the
 * IRC16M (_soft_delay_(200)) in the firmware library V1.6.0.
 */

#define GD32VW55X_AHB_STEP_DELAY     0x50
#define GD32VW55X_SWITCH_DELAY       2000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32vw55x_clockdelay
 *
 * Description:
 *   Busy loop used while the system clock changes, before any timer runs
 *   (the counts of the firmware library V1.6.0).
 *
 ****************************************************************************/

static void gd32vw55x_clockdelay(uint32_t loops)
{
  volatile uint32_t i;

  for (i = 0; i < loops; i++)
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32vw55x_clockconfig
 *
 * Description:
 *   Bring the chip from the reset clock (IRC16M) to 160 MHz from the
 *   40 MHz HXTAL through the digital PLL, reproducing the vendor SDK
 *   system_clock_160m_40m_hxtal() sequence.
 *
 ****************************************************************************/

void gd32vw55x_clockconfig(void)
{
  uint32_t regval;
  uint32_t start;

  /* Reset the RCU to a known state (SDK SystemInit sequence).  When the
   * system still runs from the PLL, started by a bootloader, lower the AHB
   * clock in steps before going back to the IRC16M, and let the core
   * voltage settle after the switch, as the firmware library V1.6.0 does.
   */

  modifyreg32(GD32VW55X_RCU_CTL, 0, RCU_CTL_IRC16MEN);

  while ((getreg32(GD32VW55X_RCU_CTL) & RCU_CTL_IRC16MSTB) == 0)
    {
    }

  if ((getreg32(GD32VW55X_RCU_CFG0) & RCU_CFG0_SCSS_MASK) ==
      RCU_CFG0_SCSS_PLLDIG)
    {
      modifyreg32(GD32VW55X_RCU_CFG0, RCU_CFG0_AHBPSC_MASK,
                  RCU_CFG0_AHBPSC_DIV2);
      gd32vw55x_clockdelay(GD32VW55X_AHB_STEP_DELAY);
      modifyreg32(GD32VW55X_RCU_CFG0, RCU_CFG0_AHBPSC_MASK,
                  RCU_CFG0_AHBPSC_DIV4);
      gd32vw55x_clockdelay(GD32VW55X_AHB_STEP_DELAY);
    }

  modifyreg32(GD32VW55X_RCU_CFG0, RCU_CFG0_SCS_MASK, 0);
  gd32vw55x_clockdelay(GD32VW55X_SWITCH_DELAY);
  putreg32(0, GD32VW55X_RCU_CFG0);
  modifyreg32(GD32VW55X_RCU_CFG1,
              RCU_CFG1_RFPLLCALEN | RCU_CFG1_RFPLLPU, 0);
  modifyreg32(GD32VW55X_RCU_CTL,
              RCU_CTL_PLLDIGEN | RCU_CTL_PLLDIGPU |
              RCU_CTL_RFCKMEN | RCU_CTL_HXTALEN, 0);
  putreg32(0, GD32VW55X_RCU_PLL);
  putreg32(0, GD32VW55X_RCU_PLLDIGCFG0);
  putreg32(0x07800000, GD32VW55X_RCU_PLLDIGCFG1);
  putreg32(0, GD32VW55X_RCU_INT);

  /* Power up and enable the 40 MHz crystal */

  modifyreg32(GD32VW55X_RCU_CTL, 0, RCU_CTL_HXTALPU);
  modifyreg32(GD32VW55X_RCU_CTL, 0, RCU_CTL_HXTALEN);

  /* SysTimer clock source = sysclock (not sysclock/4), still the IRC16M,
   * then the PLL: the mtime tick runs at 160 MHz
   */

  modifyreg32(GD32VW55X_SYSTIMER_MTIMECTL, 0, SYSTIMER_MTIMECTL_CLKSRC);

  start = getreg32(GD32VW55X_SYSTIMER_MTIME);
  while (getreg32(GD32VW55X_SYSTIMER_MTIME) - start <
         GD32VW55X_IRC16M_FREQ / 1000000 * GD32VW55X_HXTAL_SETTLE_US)
    {
    }

  modifyreg32(GD32VW55X_RCU_CTL, 0, RCU_CTL_HXTALREADY);

  /* Bus prescalers: AHB = SYSCLK, APB2 = AHB, APB1 = AHB/2 */

  modifyreg32(GD32VW55X_RCU_CFG0, 0,
              RCU_CFG0_AHBPSC_DIV1 |
              RCU_CFG0_APB2PSC_DIV1 |
              RCU_CFG0_APB1PSC_DIV2);

  /* PLLDIG source = HXTAL */

  modifyreg32(GD32VW55X_RCU_PLL, 0, RCU_PLL_PLLDIGSEL);

  /* Multiplication factor: 960 MHz / 40 MHz, fixed-point <<21 */

  regval = ((960 << 21) / (GD32VW55X_HXTAL_FREQ / 1000000)) & 0x7fffffff;
  putreg32(regval, GD32VW55X_RCU_PLLDIGCFG1);

  /* PLLDIG output 480 MHz, sysclk divider /3 -> 160 MHz */

  modifyreg32(GD32VW55X_RCU_PLLDIGCFG0, 0, RCU_PLLDIGCFG0_OSEL_480M);
  modifyreg32(GD32VW55X_RCU_PLLDIGCFG0, 0, RCU_PLLDIGCFG0_SYSDIV(2));

  /* Enable the PLL and wait for lock */

  modifyreg32(GD32VW55X_RCU_CFG1, 0, RCU_CFG1_RFPLLCALEN | RCU_CFG1_BGPU);
  modifyreg32(GD32VW55X_RCU_CTL, 0, RCU_CTL_PLLDIGEN | RCU_CTL_PLLDIGPU);

  while ((getreg32(GD32VW55X_RCU_CTL) & RCU_CTL_PLLDIGSTB) == 0)
    {
    }

  /* Switch the system clock to the PLL */

  modifyreg32(GD32VW55X_RCU_CFG0, RCU_CFG0_SCS_MASK, RCU_CFG0_SCS_PLLDIG);

  while ((getreg32(GD32VW55X_RCU_CFG0) & RCU_CFG0_SCSS_MASK) !=
         RCU_CFG0_SCSS_PLLDIG)
    {
    }
}
