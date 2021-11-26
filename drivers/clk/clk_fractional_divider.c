/****************************************************************************
 * drivers/clk/clk_fractional_divider.c
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

#include <nuttx/clk/clk_provider.h>

#include "clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_fd(_clk) (FAR struct clk_fractional_divider_s *) \
                        (_clk->private_data)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t clk_fd_recalc_rate(FAR struct clk_s *clk,
                                   uint32_t parent_rate)
{
  FAR struct clk_fractional_divider_s *fd = to_clk_fd(clk);
  uint32_t mmask = MASK(fd->mwidth) << fd->mshift;
  uint32_t nmask = MASK(fd->nwidth) << fd->nshift;
  uint32_t val;
  uint32_t m;
  uint32_t n;
  uint64_t ret;

  val = clk_read(fd->reg);

  m = (val & mmask) >> fd->mshift;
  n = (val & nmask) >> fd->nshift;

  ret = (uint64_t)parent_rate * m;
  ret /= (fd->flags & CLK_FRAC_DIV_DOUBLE ? 2 * n : n);

  return ret;
}

static uint32_t clk_fd_round_rate(FAR struct clk_s *clk,
                                  uint32_t rate, FAR uint32_t *prate)
{
  FAR struct clk_fractional_divider_s *fd = to_clk_fd(clk);
  uint32_t maxn = BIT(fd->nwidth);
  uint32_t maxm = BIT(fd->mwidth);
  uint32_t div;
  uint32_t m;
  uint32_t n;
  uint64_t ret;

  if (!rate || rate >= *prate)
    {
      return *prate;
    }

  if (fd->flags & CLK_FRAC_DIV_DOUBLE)
    {
      rate *= 2;
    }

  div = gcd(*prate, rate);

  do
    {
      m = rate / div;
      n = *prate / div;

      if ((m % 2 != 0) &&
          (fd->flags & CLK_FRAC_MUL_NEED_EVEN))
        {
          m *= 2;
          n *= 2;
        }

      div <<= 1;
    }
  while (n > maxn || m > maxm);

  ret = (uint64_t)*prate * m;
  ret /= (fd->flags & CLK_FRAC_DIV_DOUBLE ? 2 * n : n);

  return ret;
}

static int clk_fd_set_rate(FAR struct clk_s *clk, uint32_t rate,
                           uint32_t parent_rate)
{
  FAR struct clk_fractional_divider_s *fd = to_clk_fd(clk);
  uint32_t mmask = MASK(fd->mwidth) << fd->mshift;
  uint32_t nmask = MASK(fd->nwidth) << fd->nshift;
  uint32_t div;
  uint32_t n;
  uint32_t m;
  uint32_t val;

  if (fd->flags & CLK_FRAC_DIV_DOUBLE)
    {
      rate *= 2;
    }

  div = gcd(parent_rate, rate);
  m = rate / div;
  n = parent_rate / div;

  if ((m % 2 != 0) &&
      (fd->flags & CLK_FRAC_MUL_NEED_EVEN))
    {
      m *= 2;
      n *= 2;
    }

  val = clk_read(fd->reg);
  val &= ~(mmask | nmask);
  val |= (m << fd->mshift) | (n << fd->nshift);
  clk_write(fd->reg, val);

  return 0;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops_s g_clk_fractional_divider_ops =
{
  .recalc_rate = clk_fd_recalc_rate,
  .round_rate = clk_fd_round_rate,
  .set_rate = clk_fd_set_rate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct clk_s *
clk_register_fractional_divider(FAR const char *name,
                                FAR const char *parent_name,
                                uint8_t flags, uint32_t reg,
                                uint8_t mshift, uint8_t mwidth,
                                uint8_t nshift, uint8_t nwidth,
                                uint8_t clk_divider_flags)
{
  struct clk_fractional_divider_s fd;
  FAR const char **parent_names;
  uint8_t num_parents;

  parent_names = parent_name ? &parent_name : NULL;
  num_parents = parent_name ? 1 : 0;

  fd.reg = reg;
  fd.mshift = mshift;
  fd.mwidth = mwidth;
  fd.nshift = nshift;
  fd.nwidth = nwidth;
  fd.flags = clk_divider_flags;

  return clk_register(name, parent_names, num_parents, flags,
                      &g_clk_fractional_divider_ops, &fd, sizeof(fd));
}
