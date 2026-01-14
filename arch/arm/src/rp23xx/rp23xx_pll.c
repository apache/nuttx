/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_pll.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include <arch/board/board.h>

#include "rp23xx_pll.h"
#include "hardware/rp23xx_pll.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_pll_init
 *
 * Description:
 *   Initialize PLL.
 *
 ****************************************************************************/

void rp23xx_pll_init(uint32_t base, uint32_t refdiv, uint32_t vco_freq,
                     uint32_t post_div1, uint8_t post_div2)
{
  /* Turn off PLL in case it is already running */

  putreg32(0xffffffff, base + RP23XX_PLL_PWR_OFFSET);
  putreg32(0, base + RP23XX_PLL_FBDIV_INT_OFFSET);

  uint32_t ref_mhz = BOARD_XOSC_FREQ / refdiv;
  putreg32(refdiv, base + RP23XX_PLL_CS_OFFSET);

  /* What are we multiplying the reference clock by to get the vco freq
   * (The regs are called div, because you divide the vco output and compare
   *  it to the refclk)
   */

  uint32_t fbdiv = vco_freq / ref_mhz;

  /* Check parameter ranges */

  ASSERT(fbdiv >= 16 && fbdiv <= 320);
  ASSERT((post_div1 >= 1 && post_div1 <= 7) &&
         (post_div2 >= 1 && post_div2 <= 7));
  ASSERT(post_div2 <= post_div1);
  ASSERT(ref_mhz <= (vco_freq / 16));

  /* Put calculated value into feedback divider */

  putreg32(fbdiv, base + RP23XX_PLL_FBDIV_INT_OFFSET);

  /* Turn on PLL */

  clrbits_reg32(RP23XX_PLL_PWR_PD | RP23XX_PLL_PWR_VCOPD,
                base + RP23XX_PLL_PWR_OFFSET);

  /* Wait for PLL to lock */

  while (!(getreg32(base + RP23XX_PLL_CS_OFFSET) & RP23XX_PLL_CS_LOCK))
    ;

  /* Set up post dividers */

  putreg32((post_div1 << RP23XX_PLL_PRIM_POSTDIV1_SHIFT) |
           (post_div2 << RP23XX_PLL_PRIM_POSTDIV2_SHIFT),
           base + RP23XX_PLL_PRIM_OFFSET);

  /* Turn on post divider */

  clrbits_reg32(RP23XX_PLL_PWR_POSTDIVPD, base + RP23XX_PLL_PWR_OFFSET);
}
