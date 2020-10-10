/****************************************************************************
 * arch/risc-v/src/gap8/gap8_fll.c
 * GAP8 FLL clock generator
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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
 *  FC can run up to 250MHz@1.2V, and 150MHz@1.0V. While the default voltage
 *  of PMU is 1.2V, it's okay to boost up without considering PMU.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gap8_fll.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/* Log2(FLL_REF_CLK=32768) */

#define LOG2_REFCLK     15

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gap8_setfreq
 *
 * Description:
 *   Set frequency up to 250MHz. Input frequency counted by Hz.
 *
 ****************************************************************************/

void gap8_setfreq(uint32_t frequency)
{
  uint32_t mult;
  uint32_t mult_factor_diff;

  /* FreqOut = Fref * mult/2^(div-1)
   * With 16-bit mult and 4-bit div
   * div = 1
   */

  mult = frequency >> LOG2_REFCLK;

  /* Gain : 2-1 - 2-10 (0x2-0xB)
   * Return to close loop mode and give gain to feedback loop
   */

  FLL_CTRL->SOC_CONF2 = FLL_CTRL_CONF2_LOOPGAIN(0x7) |
          FLL_CTRL_CONF2_DEASSERT_CYCLES(0x10) |
          FLL_CTRL_CONF2_ASSERT_CYCLES(0x10)   |
          FLL_CTRL_CONF2_LOCK_TOLERANCE(0x100) |
          FLL_CTRL_CONF2_CONF_CLK_SEL(0x0)     |
          FLL_CTRL_CONF2_OPEN_LOOP(0x0)        |
          FLL_CTRL_CONF2_DITHERING(0x1);

  /* Configure mult and div */

  FLL_CTRL->SOC_CONF1 = FLL_CTRL_CONF1_MODE(1) |
          FLL_CTRL_CONF1_MULTI_FACTOR(mult) |
          FLL_CTRL_CONF1_CLK_OUT_DIV(1);

  /* Check FLL converge by compare status register with multiply factor */

  do
    {
      mult_factor_diff = __builtin_pulp_abs(FLL_CTRL->SOC_FLL_STATUS - mult);
    }
  while (mult_factor_diff > 0x10);

  FLL_CTRL->SOC_CONF2 = FLL_CTRL_CONF2_LOOPGAIN(0xb) |
                        FLL_CTRL_CONF2_DEASSERT_CYCLES(0x10) |
                        FLL_CTRL_CONF2_ASSERT_CYCLES(0x10)   |
                        FLL_CTRL_CONF2_LOCK_TOLERANCE(0x100) |
                        FLL_CTRL_CONF2_CONF_CLK_SEL(0x0)     |
                        FLL_CTRL_CONF2_OPEN_LOOP(0x0)        |
                        FLL_CTRL_CONF2_DITHERING(0x1);
}

/****************************************************************************
 * Name: gap8_getfreq
 *
 * Description:
 *   Get current system clock frequency in Hz.
 *
 ****************************************************************************/

uint32_t gap8_getfreq(void)
{
  /* FreqOut = Fref * mult/2^(div-1), where div = 1 */

  return FLL_REF_CLK * (FLL_CTRL->SOC_FLL_STATUS & 0xffff);
}
