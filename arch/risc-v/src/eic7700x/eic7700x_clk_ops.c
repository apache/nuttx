/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_clk_ops.c
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
#include <inttypes.h>

#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk_provider.h>
#include <nuttx/debug.h>

#include "riscv_internal.h"
#include "eic7700x_clk.h"
#include "hardware/eic7700x_clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_pll(_clk) (FAR struct eic7700x_clk_pll_s *)(_clk->private_data)
#define to_clk_div(_clk) (FAR struct eic7700x_clk_div_s *)(_clk->private_data)
#define to_clk_mux(_clk) (FAR struct eic7700x_clk_mux_s *)(_clk->private_data)

#define FIELD(v, s, w)   (((v) >> (s)) & ((1ul << (w)) - 1))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_clk_pll_recalc_rate
 *
 * Description:
 *   Compute one PLL output frequency from the CRG configuration words.
 *   The TRM (Part 1 section 3.2.3.3.2, p78) gives two formulas:
 *
 *     integer     FOUT = FREF * FBDIV
 *                      / (4 * REFDIV * (POSTDIV_A + 1) * (POSTDIV_B + 1))
 *     fractional  FOUT = FREF * (FBDIV + FRAC / 2^24)
 *                      / (4 * REFDIV * (POSTDIV_A + 1) * (POSTDIV_B + 1))
 *
 *   Only one expression is needed because FRAC is forced to zero when the
 *   PLL is in integer mode, which collapses the second formula onto the
 *   first.  All arithmetic is done in 64 bits: the numerator reaches
 *   roughly 1.7e18 for the largest dividers the fields can hold, far
 *   beyond the uint32_t that carries rates through the framework.
 *
 * Input Parameters:
 *   clk         - The clock being queried
 *   parent_rate - Frequency of the PLL reference clock
 *
 * Returned Value:
 *   The output frequency in Hz, or zero if the configuration is not
 *   representable.
 *
 ****************************************************************************/

static uint32_t eic7700x_clk_pll_recalc_rate(FAR struct clk_s *clk,
                                             uint32_t parent_rate)
{
  FAR struct eic7700x_clk_pll_s *pll = to_clk_pll(clk);
  uint32_t cfg0;
  uint32_t cfg1;
  uint32_t cfg2;
  uint32_t fbdiv;
  uint32_t refdiv;
  uint32_t frac;
  uint32_t posta;
  uint32_t postb;
  uint64_t num;
  uint64_t den;

  cfg0 = getreg32(pll->reg + EIC7700X_PLL_CFG0);
  cfg1 = getreg32(pll->reg + EIC7700X_PLL_CFG1);
  cfg2 = getreg32(pll->reg + EIC7700X_PLL_CFG2);

  /* A bypassed output passes the reference clock through untouched */

  if ((cfg0 & (1ul << (PLL_CFG0_BYP_SHIFT + pll->out))) != 0)
    {
      return parent_rate;
    }

  fbdiv  = FIELD(cfg0, PLL_CFG0_FBDIV_SHIFT, 12);
  refdiv = FIELD(cfg0, PLL_CFG0_REFDIV_SHIFT, 6);
  posta  = FIELD(cfg2, PLL_CFG2_POSTDIV_A_SHIFT(pll->out),
                 PLL_CFG2_POSTDIV_WIDTH);
  postb  = FIELD(cfg2, PLL_CFG2_POSTDIV_B_SHIFT(pll->out),
                 PLL_CFG2_POSTDIV_WIDTH);

  frac = 0;
  if ((cfg0 & PLL_CFG0_DSMEN) != 0)
    {
      frac = FIELD(cfg1, PLL_CFG1_FRAC_SHIFT, PLL_CFG1_FRAC_BITS);
    }

  if (refdiv == 0 || parent_rate == 0)
    {
      return 0;
    }

  num = (uint64_t)parent_rate *
        (((uint64_t)fbdiv << PLL_CFG1_FRAC_BITS) + frac);
  den = ((uint64_t)4 * refdiv * (posta + 1) * (postb + 1)) <<
        PLL_CFG1_FRAC_BITS;

  num = (num + den / 2) / den;

  if (num > UINT32_MAX)
    {
      clkerr("%s: rate %" PRIu64 " out of range\n", clk->name, num);
      return 0;
    }

  return (uint32_t)num;
}

/****************************************************************************
 * Name: eic7700x_clk_pll_is_enabled
 *
 * Description:
 *   Report whether a PLL output is running.  All three conditions must
 *   hold: the PLL is enabled, this particular output is enabled, and the
 *   PLL reports lock.
 *
 ****************************************************************************/

static int eic7700x_clk_pll_is_enabled(FAR struct clk_s *clk)
{
  FAR struct eic7700x_clk_pll_s *pll = to_clk_pll(clk);
  uint32_t cfg0 = getreg32(pll->reg + EIC7700X_PLL_CFG0);
  uint32_t cfg1 = getreg32(pll->reg + EIC7700X_PLL_CFG1);
  uint32_t status = getreg32(EIC7700X_PLL_STATUS);

  return (cfg0 & PLL_CFG0_EN) != 0 &&
         (cfg1 & (1ul << (PLL_CFG1_FOUTEN_SHIFT + pll->out))) != 0 &&
         (status & (1ul << pll->lockbit)) != 0;
}

/****************************************************************************
 * Name: eic7700x_clk_div_recalc_rate
 *
 * Description:
 *   Read a CRG divisor field and divide the parent rate by it.  Field
 *   values below two are aliases handled by the lowdiv member; see the
 *   comment on struct eic7700x_clk_div_s.
 *
 ****************************************************************************/

static uint32_t eic7700x_clk_div_recalc_rate(FAR struct clk_s *clk,
                                             uint32_t parent_rate)
{
  FAR struct eic7700x_clk_div_s *divider = to_clk_div(clk);
  uint32_t val;
  uint32_t div;

  val = FIELD(getreg32(divider->reg), divider->shift, divider->width);
  div = val < 2 ? divider->lowdiv : val;

  if (div == 0)
    {
      return parent_rate;
    }

  return (uint32_t)(((uint64_t)parent_rate + div / 2) / div);
}

/****************************************************************************
 * Name: eic7700x_clk_div_divisor
 *
 * Description:
 *   Which divisor this field would have to hold to bring the parent rate
 *   down to the one asked for, clamped to what the field can express.
 *
 *   Rounded up rather than to nearest, so that the result is never faster
 *   than what was asked for.  A consumer asking for a rate is usually
 *   asking for a limit: the ethernet controllers ask for the transmit
 *   clock a link speed needs, and being under it is recoverable in a way
 *   that being over it is not.
 *
 ****************************************************************************/

static uint32_t eic7700x_clk_div_divisor(FAR struct eic7700x_clk_div_s *div,
                                         uint32_t parent_rate, uint32_t rate)
{
  uint32_t maxdiv = (1ul << div->width) - 1;
  uint32_t divisor;

  if (rate == 0)
    {
      return maxdiv;
    }

  divisor = (parent_rate + rate - 1) / rate;

  /* Field values below two are aliases for one divisor, so nothing between
   * that alias and two can be asked for.
   */

  if (divisor < div->lowdiv)
    {
      divisor = div->lowdiv;
    }

  return divisor > maxdiv ? maxdiv : divisor;
}

/****************************************************************************
 * Name: eic7700x_clk_div_round_rate
 *
 * Description:
 *   The rate this divider would actually produce if asked for this one.
 *
 ****************************************************************************/

static uint32_t eic7700x_clk_div_round_rate(FAR struct clk_s *clk,
                                            uint32_t rate,
                                            FAR uint32_t *parent_rate)
{
  FAR struct eic7700x_clk_div_s *divider = to_clk_div(clk);
  uint32_t divisor;

  divisor = eic7700x_clk_div_divisor(divider, *parent_rate, rate);
  return (uint32_t)(((uint64_t)*parent_rate + divisor / 2) / divisor);
}

/****************************************************************************
 * Name: eic7700x_clk_div_set_rate
 *
 * Description:
 *   Write a CRG divisor field.  This is the one place this provider
 *   changes the hardware rather than reporting it, and it is deliberately
 *   the only one: a divider belongs to a single peripheral, so writing it
 *   affects only the driver that asked, whereas the muxes and the PLLs are
 *   shared and reprogramming one underneath its other users would be a
 *   different and much less local kind of change.
 *
 *   Read modify write, because most of these registers carry a gate, a
 *   selector or a second peripheral's divisor in the same word.
 *
 ****************************************************************************/

static int eic7700x_clk_div_set_rate(FAR struct clk_s *clk, uint32_t rate,
                                     uint32_t parent_rate)
{
  FAR struct eic7700x_clk_div_s *divider = to_clk_div(clk);
  uint32_t mask = ((1ul << divider->width) - 1) << divider->shift;
  uint32_t divisor;
  uint32_t regval;

  divisor = eic7700x_clk_div_divisor(divider, parent_rate, rate);

  regval  = getreg32(divider->reg);
  regval &= ~mask;
  regval |= (divisor << divider->shift) & mask;
  putreg32(regval, divider->reg);

  clkinfo("%s: %" PRIu32 " / %" PRIu32 " for %" PRIu32 "\n",
          clk->name, parent_rate, divisor, rate);
  return 0;
}

/****************************************************************************
 * Name: eic7700x_clk_mux_get_parent
 *
 * Description:
 *   Map a CRG selector field onto a parent index.  Unlike the generic
 *   clk_mux this never returns an index beyond the parent count: the core
 *   uses the result to subscript parent_names[] without checking it, and
 *   several selectors are two bits wide with only three valid values.
 *
 ****************************************************************************/

static uint8_t eic7700x_clk_mux_get_parent(FAR struct clk_s *clk)
{
  FAR struct eic7700x_clk_mux_s *mux = to_clk_mux(clk);
  uint32_t val;
  uint8_t i;

  val = FIELD(getreg32(mux->reg), mux->shift, mux->width);

  for (i = 0; i < mux->nparents; i++)
    {
      if (mux->table[i] == val)
        {
          return i;
        }
    }

  clkwarn("%s: unexpected selector %" PRIu32 ", using parent 0\n",
          clk->name, val);
  return 0;
}

/****************************************************************************
 * Name: eic7700x_clk_mux_set_parent
 *
 * Description:
 *   Point a mux at one of its parents, the write side of the read above.
 *
 *   This exists because the boot loader's configuration cannot be taken on
 *   trust.  Several muxes on this chip reset to a slow or idle parent and
 *   reach their working one only because firmware moved them, and until
 *   now the tree could observe that and not correct it, which left any
 *   peripheral the boot loader had not set up stuck that way.
 *
 *   A clock the system itself runs on is only allowed to speed up.  Going
 *   the other way pulls the ground out from under whatever is executing:
 *   the timing every downstream driver was configured for changes while
 *   they are using it.  Recovering a mux that came up on the wrong parent
 *   is always the raising direction, so refusing the other one costs
 *   nothing and is the same bargain the reset driver strikes, where a line
 *   that would take the system down may be released and read but not
 *   asserted.
 *
 * Input Parameters:
 *   clk   - The mux
 *   index - Which of its parents to select
 *
 * Returned Value:
 *   Zero on success, or a negated errno:
 *
 *     -EINVAL  the index names no parent of this mux
 *     -EPERM   the switch would slow a clock the system is running on
 *     -EIO     the selector did not read back as written
 *
 ****************************************************************************/

static int eic7700x_clk_mux_set_parent(FAR struct clk_s *clk, uint8_t index)
{
  FAR struct eic7700x_clk_mux_s *mux = to_clk_mux(clk);
  uint32_t mask;
  uint32_t regval;

  if (index >= mux->nparents)
    {
      return -EINVAL;
    }

  if ((clk->flags & CLK_IS_CRITICAL) != 0)
    {
      FAR struct clk_s *target = clk_get_parent_by_index(clk, index);
      uint32_t now = clk_get_rate(clk);

      if (target == NULL)
        {
          return -EINVAL;
        }

      if (clk_get_rate(target) < now)
        {
          clkerr("%s: refusing %" PRIu32 " to %" PRIu32
                 ", the system is running on it\n",
                 clk->name, now, clk_get_rate(target));
          return -EPERM;
        }
    }

  mask    = ((1ul << mux->width) - 1) << mux->shift;
  regval  = getreg32(mux->reg);
  regval &= ~mask;
  regval |= ((uint32_t)mux->table[index] << mux->shift) & mask;
  putreg32(regval, mux->reg);

  /* Read it back.  Some of these selectors are held by hardware that is
   * not ready to switch, and a mux that quietly ignored the write would
   * otherwise leave the tree describing a parent the clock does not have.
   */

  if (FIELD(getreg32(mux->reg), mux->shift, mux->width) != mux->table[index])
    {
      clkerr("%s: selector will not move to %u\n", clk->name,
             mux->table[index]);
      return -EIO;
    }

  clkinfo("%s: now on parent %u\n", clk->name, index);
  return 0;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The PLL and divider clocks report a rate; the mux has no recalc_rate so
 * the core passes the selected parent rate through unchanged, which is
 * exactly what a selector does.
 *
 * Only the divider can be written, and only its divisor field.  A divider
 * in this block feeds one peripheral, so a driver that sets one is
 * changing its own clock and nobody else's; the PLLs and the selectors are
 * shared, and moving one under its other users would be a different kind
 * of change entirely.  So those two stay read only, and the gates are left
 * to the generic gate implementation the tree already provides.
 */

const struct clk_ops_s g_eic7700x_clk_pll_ops =
{
  .recalc_rate = eic7700x_clk_pll_recalc_rate,
  .is_enabled  = eic7700x_clk_pll_is_enabled,
};

const struct clk_ops_s g_eic7700x_clk_div_ops =
{
  .recalc_rate = eic7700x_clk_div_recalc_rate,
  .round_rate  = eic7700x_clk_div_round_rate,
  .set_rate    = eic7700x_clk_div_set_rate,
};

const struct clk_ops_s g_eic7700x_clk_mux_ops =
{
  .get_parent = eic7700x_clk_mux_get_parent,
  .set_parent = eic7700x_clk_mux_set_parent,
};
