/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_pll.c
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

#include "rp23xx_pll.h"

#include "hardware/address_mapped.h"
#include "hardware/structs/pll.h"

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

void rp23xx_pll_init(pll_hw_t *base, uint32_t refdiv, uint32_t vco_freq,
                     uint32_t post_div1, uint8_t post_div2)
{
  /* Turn off PLL in case it is already running */

  base->pwr = 0xffffffff;
  base->fbdiv_int = 0;

  uint32_t ref_mhz = BOARD_XOSC_FREQ / refdiv;
  base->cs = refdiv;

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

  base->fbdiv_int = fbdiv;

  /* Turn on PLL */

  hw_clear_bits(&base->pwr, PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS);

  /* Wait for PLL to lock */

  while (!(base->cs & PLL_CS_LOCK_BITS))
    ;

  /* Set up post dividers */

  base->prim = 
           (post_div1 << PLL_PRIM_POSTDIV1_LSB) |
           (post_div2 << PLL_PRIM_POSTDIV2_LSB);

  /* Turn on post divider */

  hw_clear_bits(&base->pwr, PLL_PWR_POSTDIVPD_BITS);
}
