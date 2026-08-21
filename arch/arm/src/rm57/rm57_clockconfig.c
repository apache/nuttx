/****************************************************************************
 * arch/arm/src/rm57/rm57_clockconfig.c
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

/* Register write sequence ported from TI's HALCoGen-generated
 * HL_system.c (setupPLL/periphInit/setupFlash/mapClocks), translated to
 * NuttX register-header macros. Board-specific PLL parameters
 * (BOARD_PLL_*) come from arch/board/board.h, following the same
 * chip/board split tms570_clockconfig.c uses.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "arm_internal.h"

#include "hardware/rm57_sys.h"
#include "hardware/rm57_pcr.h"
#include "hardware/rm57_flash.h"
#include "rm57_clockconfig.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_setup_pll
 *
 * Description:
 *   Disable PLL1/PLL2, configure their control registers from the board-
 *   supplied BOARD_PLL_* parameters, then re-enable them.  The PLL output
 *   divider is deliberately left at its maximum (slowest) setting here;
 *   rm57_map_clocks() switches it to the final board.h value only after
 *   the PLL has locked.
 *
 ****************************************************************************/

static void rm57_setup_pll(void)
{
  /* Disable PLL1 and PLL2 */

  putreg32(SYS_CSDIS_CLKSR1OFF | SYS_CSDIS_CLKSR6OFF, RM57_SYS_CSDISSET);
  while ((getreg32(RM57_SYS_CSDIS) &
         (SYS_CSDIS_CLKSR1OFF | SYS_CSDIS_CLKSR6OFF)) !=
         (SYS_CSDIS_CLKSR1OFF | SYS_CSDIS_CLKSR6OFF))
    {
    }

  /* Clear Global Status Register (oscillator fail / PLL slip flags) */

  putreg32(0x301, RM57_SYS_GBLSTAT);

  /* PLL1: output divider forced to max until locked, reset-on-osc-fail,
   * reference clock divider and multiplier from board.h
   */

  putreg32(SYS_PLLCTL1_PLLDIV_MASK |
           SYS_PLLCTL1_REFCLKDIV(BOARD_PLL_NR - 1) |
           SYS_PLLCTL1_PLLMUL(BOARD_PLL_PLLMUL),
           RM57_SYS_PLLCTL1);

  putreg32(((uint32_t)255 << 22) | ((uint32_t)7 << 12) |
           SYS_PLLCTL2_ODPLL(BOARD_PLL_OD - 1) | 61,
           RM57_SYS_PLLCTL2);

  /* PLL2: same output-divider-forced-to-max pattern */

  putreg32(SYS_PLLCTL1_PLLDIV_MASK |
           SYS_PLLCTL1_REFCLKDIV(BOARD_PLL_NR - 1) |
           SYS_PLLCTL1_PLLMUL(BOARD_PLL_PLLMUL),
           RM57_SYS2_PLLCTL3);

  /* Enable PLL1 and PLL2 to start up / lock (also re-enable oscillator
   * and the two LPO clock sources, matching the HALCoGen default)
   */

  putreg32(SYS_CSDIS_CLKSR3OFF | SYS_CSDIS_CLKSR7OFF, RM57_SYS_CSDIS);
}

/****************************************************************************
 * Name: rm57_periph_init
 *
 * Description:
 *   Release all peripherals from local reset and enable their clocks.
 *
 ****************************************************************************/

static void rm57_periph_init(void)
{
  uint32_t regval;

  /* Disable peripherals before peripheral power-up */

  regval = getreg32(RM57_SYS_CLKCNTL);
  regval &= ~(1 << 8);
  putreg32(regval, RM57_SYS_CLKCNTL);

  /* Power up all peripherals on PCR frame 1 */

  putreg32(0xffffffff, RM57_PCR1_PSPWRDWNCLR0);
  putreg32(0xffffffff, RM57_PCR1_PSPWRDWNCLR1);
  putreg32(0xffffffff, RM57_PCR1_PSPWRDWNCLR2);
  putreg32(0xffffffff, RM57_PCR1_PSPWRDWNCLR3);

  /* Enable peripherals */

  regval = getreg32(RM57_SYS_CLKCNTL);
  regval |= (1 << 8);
  putreg32(regval, RM57_SYS_CLKCNTL);
}

/****************************************************************************
 * Name: rm57_setup_flash
 *
 * Description:
 *   Configure flash read wait-states for the target CPU clock frequency.
 *
 ****************************************************************************/

static void rm57_setup_flash(void)
{
  putreg32(FLASH_FRDCNTL_RWAIT(BOARD_FLASH_RWAIT) | 3, RM57_FLASH_FRDCNTL);
}

/****************************************************************************
 * Name: rm57_map_clocks
 *
 * Description:
 *   Wait for the PLLs to lock, then map GCLK/HCLK/VCLK/RTICLK to their
 *   final clock sources and switch the PLL output dividers to their
 *   final (board.h) values.
 *
 ****************************************************************************/

static void rm57_map_clocks(void)
{
  uint32_t csvstat;
  uint32_t csdis;
  uint32_t regval;

  /* Setup HCLK divider */

  putreg32(1, RM57_SYS2_HCLKCNTL);

  /* Wait until the enabled clock sources are valid.
   *
   * Bit 2 of both CSDIS and CSVSTAT is reserved (there is no clock
   * source 2) and reads back as 0 on this silicon. Comparing the full
   * 0xff byte therefore always requires bit 2 of CSVSTAT to become 1,
   * which it never does - confirmed on real hardware via JTAG
   * (CSDIS=0x88, CSVSTAT=0xfb read while stuck here: oscillator, PLL1
   * and PLL2 were all already valid, only the reserved-bit compare was
   * failing). Mask it out of the comparison.
   */

  do
    {
      csvstat = getreg32(RM57_SYS_CSVSTAT);
      csdis   = getreg32(RM57_SYS_CSDIS);
    }
  while ((csvstat & ((csdis ^ 0xff) & 0xfb)) != ((csdis ^ 0xff) & 0xfb));

  /* Map GCLK/HCLK/VCLK/VCLK2 to PLL1 */

  putreg32(SYS_GHVSRC_GHVWAKE(SYS_CLKSRC_PLL1) |
           SYS_GHVSRC_HVLPM(SYS_CLKSRC_PLL1) |
           SYS_GHVSRC_GHVSRC_SRC(SYS_CLKSRC_PLL1),
           RM57_SYS_GHVSRC);

  /* Map RTICLK1 to VCLK */

  putreg32(SYS_RCLKSRC_RTI1DIV_DIV2 | SYS_RCLKSRC_RTI1SRC(SYS_CLKSRC_VCLK),
           RM57_SYS_RCLKSRC);

  /* Now that the PLLs are locked, switch the output dividers from their
   * max (slow) startup value to the final board.h value
   */

  regval  = getreg32(RM57_SYS_PLLCTL1);
  regval &= ~SYS_PLLCTL1_PLLDIV_MASK;
  regval |= SYS_PLLCTL1_PLLDIV(BOARD_PLL_R - 1);
  putreg32(regval, RM57_SYS_PLLCTL1);

  regval  = getreg32(RM57_SYS2_PLLCTL3);
  regval &= ~SYS_PLLCTL1_PLLDIV_MASK;
  regval |= SYS_PLLCTL1_PLLDIV(BOARD_PLL_R - 1);
  putreg32(regval, RM57_SYS2_PLLCTL3);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_clockconfig
 ****************************************************************************/

void rm57_clockconfig(void)
{
  rm57_setup_pll();
  rm57_periph_init();
  rm57_setup_flash();
  rm57_map_clocks();
}
