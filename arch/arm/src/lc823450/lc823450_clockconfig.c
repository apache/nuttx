/****************************************************************************
 * arch/arm/src/lc823450/lc823450_clockconfig.c
 *
 *   Copyright 2014,2015,2017 Sony Video & Sound Products Inc.
 *   Author: Masatoshi Tateishi <Masatoshi.Tateishi@jp.sony.com>
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "lc823450_clockconfig.h"
#include "lc823450_syscontrol.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MAX_CPU_CLOCK_150
#  define SYSCLK 150 /* MHz */
#else
#  define SYSCLK 160 /* MHz */
#endif

#ifdef CONFIG_AHB_CLOCK_IS_CPU_CLOCK
#  define HCLKDIV 0  /* AHB = system / (HCLKIDV + 1) */
#else
#  define HCLKDIV 3  /* AHB = system / (HCLKIDV + 1) */
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

unsigned int XT1OSC_CLK;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_get_systemfreq
 ****************************************************************************/

uint32_t lc823450_get_systemfreq(void)
{
    return SYSCLK * 1000000;
}

/****************************************************************************
 * Name: lc823450_get_apb
 ****************************************************************************/

#ifndef CONFIG_DVFS
uint32_t lc823450_get_apb(void)
{
    return SYSCLK * 1000000;
}
#endif

/****************************************************************************
 * Name: lc823450_get_ahb
 ****************************************************************************/

uint32_t lc823450_get_ahb(void)
{
    return (SYSCLK * 1000000) / (HCLKDIV + 1);
}

/****************************************************************************
 * Name: lc823450_clockconfig
 ****************************************************************************/

void lc823450_clockconfig()
{
  uint32_t val;

  val = getreg32(BMODE_DT) & BMODE_DT_XTALINFO_MASK;

  if (val == BMODE_DT_XTALINFO_20)
    {
      XT1OSC_CLK = (20 * 1000 * 1000);
    }
  else
    {
      XT1OSC_CLK = (24 * 1000 * 1000);
    }

  /* XT1 enable */

  modifyreg32(OSCCNT, 0, OSCCNT_XT1EN);

  /* Clear MAINDIV */

  modifyreg32(OSCCNT, OSCCNT_MAINDIV_MASK, 0);

  /* Select system clock source from MAIN(XT1) */

  val = getreg32(OSCCNT);
  val &= ~OSCCNT_SCKSEL_MASK;
  val |= OSCCNT_SCKSEL_MAIN;
  putreg32(val, OSCCNT);

#ifdef CONFIG_LC823450_IPL2
  /* set the common PLL values */
  /* XTAL / XT1OSC_CLK = 1MHz */

  putreg32((XT1OSC_CLK / 1000000) - 1, PLL1MDIV);

  /* 1MHz * SYSCLK = system clock */

  putreg32(SYSCLK * 2 - 1, PLL1NDIV);
#else
  if (SYSCLK == 150 && XT1OSC_CLK == 24000000)
    {
      putreg32(0x01, PLL1MDIV);
      putreg32(0x18, PLL1NDIV);
    }
  else if (SYSCLK == 160 && XT1OSC_CLK == 24000000)
    {
      putreg32(0x02, PLL1MDIV);
      putreg32(0x27, PLL1NDIV);
    }
  else if (SYSCLK == 160 && XT1OSC_CLK == 20000000)
    {
      putreg32(0x00, PLL1MDIV);
      putreg32(0x0f, PLL1NDIV);
    }
  else
    {
      DEBUGASSERT(false);
    }
#endif

  /* enable PLL */

  val = getreg32(PLL1CNT);
  val |= PLL1CNT_STYB;
  val |= PLL1CNT_RSTB;
  putreg32(val, PLL1CNT);

  /* wait for lock */

  up_udelay(10000);

#if 0
  /* To check basic clock by PHI pin.
   * Don't forget to change GPIO09 pinmux.
   */

  val = getreg32(FCLKCNT);
  val |= (1 << 26);
  putreg32(val, FCLKCNT);
#endif

  /* S-Flash fclock = sysclk / 4 */

  val = getreg32(FCLKCNT);
  val |= FCLKCNT_SFDIV4;
  putreg32(val, FCLKCNT);

  /* set AHB with HCLKDIV */

  modifyreg32(PERICLKDIV,
        PERICLKDIV_HCLKDIV_MASK,
        HCLKDIV);

  /* EXT4 =  sysclk / 3 (53.3MHz=18.8n) */

  val = getreg32(PERICLKDIV);
  val |= (3 - 1) << 16;
  putreg32(val, PERICLKDIV);

  /* MAIN clock source = PLL */

  val = getreg32(OSCCNT);
  val |= OSCCNT_MCSEL;
  putreg32(val, OSCCNT);

  /* Set ROM wait cycle (DSP=1wait, CPU=1wait) */

  modifyreg32(MEMEN4, 0, MEMEN4_DWAIT | MEMEN4_HWAIT);

  /* Select system clock source from MAIN(PLL) */

  val = getreg32(OSCCNT);
  val &= ~OSCCNT_SCKSEL_MASK;
  val |= OSCCNT_SCKSEL_MAIN;
  putreg32(val, OSCCNT);
}
