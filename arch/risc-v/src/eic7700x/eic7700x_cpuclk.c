/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_cpuclk.c
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

/* The core clock, measured rather than believed.
 *
 * The manual and the vendor's code disagree by a factor of two about
 * which of the CPU PLL's outputs reaches the cores, and each is
 * internally consistent, so only the hardware settles it.  The cycle
 * counter counts core clocks and the time counter counts a megahertz
 * derived from the crystal, so their ratio is the core frequency against
 * a reference no CPU register can touch.
 */

/* The cores run from the PLL being reprogrammed, so they are parked on a
 * slower clock first, through a selector the vendor names as glitch free.
 * While parked the PLL is stopped, given new dividers, restarted and
 * watched until it locks; if it never locks the cores stay parked, since
 * returning them to an unlocked PLL does not fail safely.
 *
 * Above a gigahertz the bus ratio must be two to one before the cores
 * return to the PLL: the bus fabric does not reach beyond about eight
 * hundred megahertz.  Alone in this sequence, that one is not recoverable
 * from software.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/clk/clk.h>
#include <nuttx/mutex.h>

#include "eic7700x_cpuclk.h"
#include "hardware/eic7700x_clk.h"
#include "riscv_internal.h"

#ifdef CONFIG_EIC7700X_CPUCLK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How long to count for.  The time counter steps at one megahertz, so ten
 * thousand steps is ten milliseconds: long enough that the odd stall or
 * interrupt is noise, short enough not to matter at boot.
 */

#define CPUCLK_SAMPLE_TICKS  10000
#define CPUCLK_TICK_HZ       1000000ul

/* The output formula, solved for the dividers.
 *
 * FOUT = FREF * (FBDIV + FRAC / 2^24) / (4 * REFDIV) on the undivided
 * output the cores use, and with the reference divider left at one that
 * is six megahertz per unit of feedback.  So the feedback value is the
 * target over six megahertz, integer part in FBDIV and the remainder
 * scaled into the twenty four bit fraction.
 */

#define CPUCLK_HZ_PER_FBDIV  6000000ul

/* How long to wait for lock.  The vendor says fifty microseconds is
 * usual and gives up somewhere over a millisecond; this polls in twenty
 * microsecond steps and allows five, since giving up early leaves the
 * system parked at four hundred megahertz.
 */

#define CPUCLK_LOCK_POLL_US  20
#define CPUCLK_LOCK_LIMIT_US 5000

/* The selector's three inputs, and the bus ratio's two */

#define CPUCLK_SEL_PLL       0    /* The undivided CPU PLL output         */
#define CPUCLK_SEL_LP        1    /* 400 MHz parking leg off spll0        */
#define CPUCLK_SEL_MASK      (0x3)
#define CPUCLK_BUS_RATIO_BIT (1ul << U84_BUSCLK_RATIO_SEL_SHIFT)

/* Above this the bus must run at half the core clock (vendor rule) */

#define CPUCLK_BUS_LIMIT_HZ  1000000000ul

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* One transition at a time.  The registers are shared and the sequence
 * has a parked middle, so two callers interleaving would be a mess.
 */

static mutex_t g_cpuclk_lock = NXMUTEX_INITIALIZER;

/* The speeds worth asking for: the set the vendor ships as operating
 * points, every one of them at the same core voltage, which is why this
 * file never touches a regulator.  The PLL can hit other values; these
 * are the ones anyone has validated.
 */

static const uint32_t g_cpuclk_rates[] =
{
  400000000,  500000000,  600000000,  700000000, 800000000,
  1000000000, 1100000000, 1200000000, 1300000000, 1400000000,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_cpuclk_measure
 *
 * Description:
 *   Measure the core clock against the crystal derived time counter.
 *   See eic7700x_cpuclk.h.
 *
 ****************************************************************************/

uint32_t eic7700x_cpuclk_measure(void)
{
  uint64_t time0;
  uint64_t time1;
  uint64_t cycle0;
  uint64_t cycle1;

  time0 = READ_CSR(CSR_TIME);

  /* Align to a tick edge so the window is a whole number of ticks */

  do
    {
      time1 = READ_CSR(CSR_TIME);
    }
  while (time1 == time0);

  cycle0 = READ_CSR(CSR_CYCLE);
  time0  = time1;

  do
    {
      time1 = READ_CSR(CSR_TIME);
    }
  while (time1 - time0 < CPUCLK_SAMPLE_TICKS);

  cycle1 = READ_CSR(CSR_CYCLE);

  return (uint32_t)((cycle1 - cycle0) * CPUCLK_TICK_HZ / (time1 - time0));
}

/****************************************************************************
 * Name: eic7700x_cpuclk_rates
 *
 * Description:
 *   The speeds the cores may be asked for.  See eic7700x_cpuclk.h.
 *
 ****************************************************************************/

const uint32_t *eic7700x_cpuclk_rates(size_t *count)
{
  *count = sizeof(g_cpuclk_rates) / sizeof(g_cpuclk_rates[0]);
  return g_cpuclk_rates;
}

/****************************************************************************
 * Name: eic7700x_cpuclk_setrate
 *
 * Description:
 *   Move the cores to one of those speeds.  See eic7700x_cpuclk.h.
 *
 ****************************************************************************/

int eic7700x_cpuclk_setrate(uint32_t hz)
{
  uint32_t ctrl;
  uint32_t cfg;
  uint32_t fbdiv;
  uint32_t frac;
  uint64_t rem;
  bool valid = false;
  int waited;
  int ret;
  size_t i;

  for (i = 0; i < sizeof(g_cpuclk_rates) / sizeof(g_cpuclk_rates[0]); i++)
    {
      if (g_cpuclk_rates[i] == hz)
        {
          valid = true;
          break;
        }
    }

  if (!valid)
    {
      return -EINVAL;
    }

  /* Solve for the dividers.  The fraction floors, which loses under a
   * third of a hertz and matches the values the vendor ships.
   */

  fbdiv = hz / CPUCLK_HZ_PER_FBDIV;
  rem   = hz % CPUCLK_HZ_PER_FBDIV;
  frac  = (uint32_t)((rem << PLL_CFG1_FRAC_BITS) / CPUCLK_HZ_PER_FBDIV);

  ret = nxmutex_lock(&g_cpuclk_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Make sure the parking leg is really ticking before trusting the
   * cores to it.  Its PLL always runs and feeds most of the chip, but the
   * output has both an enable bit and a gate, and parking four cores on a
   * stopped clock reports no error.
   */

  ctrl  = getreg32(EIC7700X_SPLL0_BASE + EIC7700X_PLL_CFG1);
  ctrl |= (1 << (PLL_CFG1_FOUTEN_SHIFT + 2));
  putreg32(ctrl, EIC7700X_SPLL0_BASE + EIC7700X_PLL_CFG1);

  ctrl  = getreg32(EIC7700X_SPLL0_BASE + EIC7700X_PLL_CFG2);
  ctrl |= (1ul << 31);
  putreg32(ctrl, EIC7700X_SPLL0_BASE + EIC7700X_PLL_CFG2);

  /* Park the cores on the slow leg.  STRICTLY mux first, ratio second:
   * the ratio change is only safe once the cores are at four hundred
   * megahertz, because at one to one the bus runs at core speed, and a
   * core still at full speed would put the bus fabric far above its
   * limit: the one mistake in this sequence the vendor marks as
   * unrecoverable.  Getting these two writes backwards hangs all four
   * cores so completely that even the debugger cannot halt them.
   */

  ctrl  = getreg32(EIC7700X_CLK_BASE + EIC7700X_U84_CLK_CTRL);
  ctrl  = (ctrl & ~CPUCLK_SEL_MASK) | CPUCLK_SEL_LP;
  putreg32(ctrl, EIC7700X_CLK_BASE + EIC7700X_U84_CLK_CTRL);

  ctrl |= CPUCLK_BUS_RATIO_BIT;
  putreg32(ctrl, EIC7700X_CLK_BASE + EIC7700X_U84_CLK_CTRL);

  /* Stop the PLL, feed it the new dividers, start it again.  The
   * reference divider stays at one and the post dividers are left alone:
   * the output the cores use is undivided, and the fraction mode bits
   * are already right for a fractional value.
   */

  cfg = getreg32(EIC7700X_CPUPLL_BASE + EIC7700X_PLL_CFG0);
  putreg32(cfg & ~PLL_CFG0_EN, EIC7700X_CPUPLL_BASE + EIC7700X_PLL_CFG0);

  cfg &= ~(PLL_CFG0_FBDIV_MASK | PLL_CFG0_REFDIV_MASK);
  cfg |= (fbdiv << PLL_CFG0_FBDIV_SHIFT) | (1 << PLL_CFG0_REFDIV_SHIFT);
  putreg32(cfg & ~PLL_CFG0_EN, EIC7700X_CPUPLL_BASE + EIC7700X_PLL_CFG0);

  ctrl = getreg32(EIC7700X_CPUPLL_BASE + EIC7700X_PLL_CFG1);
  ctrl = (ctrl & ~PLL_CFG1_FRAC_MASK) | (frac << PLL_CFG1_FRAC_SHIFT);
  putreg32(ctrl, EIC7700X_CPUPLL_BASE + EIC7700X_PLL_CFG1);

  putreg32(cfg | PLL_CFG0_EN, EIC7700X_CPUPLL_BASE + EIC7700X_PLL_CFG0);

  /* Wait for lock, and if it never comes, stay parked.  Slow and alive
   * beats fast and gone; the caller finds out, the log says why, and the
   * board is still a board.
   */

  for (waited = 0; waited < CPUCLK_LOCK_LIMIT_US;
       waited += CPUCLK_LOCK_POLL_US)
    {
      if ((getreg32(EIC7700X_PLL_STATUS) &
           (1 << PLL_STATUS_CPUPLL_LOCK)) != 0)
        {
          break;
        }

      up_udelay(CPUCLK_LOCK_POLL_US);
    }

  if (waited >= CPUCLK_LOCK_LIMIT_US)
    {
      /* Note the clock tree still reports the PLL rate after this: the
       * framework caches the selector's parent and there is no honest way
       * to tell it about a parent it would refuse to choose.  The log
       * line and the measurement are the truth; this is the one state
       * where they and the tree disagree, and it is already an ERROR.
       */

      syslog(LOG_ERR, "cpu: PLL will not lock at %lu MHz, parked at "
             "400 MHz\n", (unsigned long)(hz / 1000000));
      nxmutex_unlock(&g_cpuclk_lock);
      return -ETIMEDOUT;
    }

  /* The bus rule, before the cores come back: above a gigahertz the bus
   * must be half the core clock.  At or below, full speed is inside the
   * fabric's limit and is what the vendor runs.
   */

  ctrl = getreg32(EIC7700X_CLK_BASE + EIC7700X_U84_CLK_CTRL);
  if (hz >= CPUCLK_BUS_LIMIT_HZ)
    {
      ctrl &= ~CPUCLK_BUS_RATIO_BIT;
      putreg32(ctrl, EIC7700X_CLK_BASE + EIC7700X_U84_CLK_CTRL);
    }

  ctrl = (ctrl & ~CPUCLK_SEL_MASK) | CPUCLK_SEL_PLL;
  putreg32(ctrl, EIC7700X_CLK_BASE + EIC7700X_U84_CLK_CTRL);

  nxmutex_unlock(&g_cpuclk_lock);

  /* Tell the clock tree.  Its CPU PLL outputs are never cached, and
   * asking for one recomputes it and everything below it, so one query
   * per output brings the whole reported subtree back to the truth.
   */

  clk_get_rate(clk_get("cpupll_fout1"));
  clk_get_rate(clk_get("cpupll_fout2"));
  clk_get_rate(clk_get("cpupll_fout3"));

  return OK;
}

/****************************************************************************
 * Name: eic7700x_cpuclk_initialize
 *
 * Description:
 *   Measure the cores, report the result beside what the clock tree
 *   computes, and move them to the configured speed.  See
 *   eic7700x_cpuclk.h.
 *
 ****************************************************************************/

void eic7700x_cpuclk_initialize(void)
{
  uint32_t measured;
  uint32_t wanted = (uint32_t)CONFIG_EIC7700X_CPU_FREQ_MHZ * 1000000ul;

  measured = eic7700x_cpuclk_measure();

  syslog(LOG_INFO, "cpu: measured %lu MHz, clock tree says %lu MHz\n",
         (unsigned long)(measured / 1000000),
         (unsigned long)(clk_get_rate(clk_get("clk_u84_core")) / 1000000));

  /* Within a couple of megahertz is the same speed.  The measurement
   * jitters by a megahertz either way: an interrupt landing inside the
   * sample window stretches it, and the table has no two entries within
   * a hundred, so the tolerance is safe and stops a needless transition
   * at every boot.
   */

  if (measured + 2000000 > wanted && measured < wanted + 2000000)
    {
      return;
    }

  if (eic7700x_cpuclk_setrate(wanted) == OK)
    {
      syslog(LOG_INFO, "cpu: now measured %lu MHz\n",
             (unsigned long)(eic7700x_cpuclk_measure() / 1000000));
    }
}

#endif /* CONFIG_EIC7700X_CPUCLK */
