/****************************************************************************
 * arch/arm/src/rp2040/rp2040_pll.c
 *
 * Based upon the software originally developed by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "arm_internal.h"
#include "chip.h"

#include "rp2040_pll.h"
#include "hardware/rp2040_pll.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_pll_init
 *
 * Description:
 *   Initialize PLL.
 *
 ****************************************************************************/

void rp2040_pll_init(uint32_t base, uint32_t refdiv, uint32_t vco_freq,
                     uint32_t post_div1, uint8_t post_div2)
{
  /* Turn off PLL in case it is already running */

  putreg32(0xffffffff, base + RP2040_PLL_PWR_OFFSET);
  putreg32(0, base + RP2040_PLL_FBDIV_INT_OFFSET);

  uint32_t ref_mhz = BOARD_XOSC_FREQ / refdiv;
  putreg32(refdiv, base + RP2040_PLL_CS_OFFSET);

  /* What are we multiplying the reference clock by to get the vco freq
   * (The regs are called div, because you divide the vco output and compare
   *  it to the refclk)
   */

  uint32_t fbdiv = vco_freq / ref_mhz;

  /* Check parameter ranges */

  assert(fbdiv >= 16 && fbdiv <= 320);
  assert((post_div1 >= 1 && post_div1 <= 7) &&
         (post_div2 >= 1 && post_div2 <= 7));
  assert(post_div2 <= post_div1);
  assert(ref_mhz <= (vco_freq / 16));

  /* Put calculated value into feedback divider */

  putreg32(fbdiv, base + RP2040_PLL_FBDIV_INT_OFFSET);

  /* Turn on PLL */

  clrbits_reg32(RP2040_PLL_PWR_PD | RP2040_PLL_PWR_VCOPD,
                base + RP2040_PLL_PWR_OFFSET);

  /* Wait for PLL to lock */

  while (!(getreg32(base + RP2040_PLL_CS_OFFSET) & RP2040_PLL_CS_LOCK))
    ;

  /* Set up post dividers */

  putreg32((post_div1 << RP2040_PLL_PRIM_POSTDIV1_SHIFT) |
           (post_div2 << RP2040_PLL_PRIM_POSTDIV2_SHIFT),
           base + RP2040_PLL_PRIM_OFFSET);

  /* Turn on post divider */

  clrbits_reg32(RP2040_PLL_PWR_POSTDIVPD, base + RP2040_PLL_PWR_OFFSET);
}
