/****************************************************************************
 * drivers/clk/clk_multiplier.c
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

#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk_provider.h>

#include <debug.h>
#include <stdlib.h>

#include "clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_multiplier(_clk) (FAR struct clk_multiplier_s *) \
                                (_clk->private_data)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t _get_maxmult(FAR struct clk_multiplier_s *multiplier)
{
  if (multiplier->flags & CLK_MULT_ONE_BASED)
    {
      return MASK(multiplier->width);
    }

  if (multiplier->flags & CLK_MULT_MAX_HALF)
    {
      return 1 << (multiplier->width - 1);
    }

  return MASK(multiplier->width) + 1;
}

static uint32_t _get_mult(FAR struct clk_multiplier_s *multiplier,
                          uint32_t val)
{
  if (multiplier->flags & CLK_MULT_ONE_BASED)
    {
      return val;
    }

  return val + 1;
}

static uint32_t _get_val(FAR struct clk_multiplier_s *multiplier,
                         uint32_t mult)
{
  if (multiplier->flags & CLK_MULT_ONE_BASED)
    {
      return mult;
    }

  return mult - 1;
}

static uint32_t clk_multiplier_recalc_rate(FAR struct clk_s *clk,
                                           uint32_t parent_rate)
{
  FAR struct clk_multiplier_s *multiplier = to_clk_multiplier(clk);
  uint32_t mult;
  uint32_t val;

  val = clk_read(multiplier->reg) >> multiplier->shift;
  val &= MASK(multiplier->width);

  mult = _get_mult(multiplier, val);
  if (!mult)
    {
      if (!(multiplier->flags & CLK_MULT_ALLOW_ZERO))
        {
          return -EINVAL;
        }

      return parent_rate;
    }

  return parent_rate * mult;
}

static bool __is_best_rate(uint32_t rate, uint32_t new,
                           uint32_t best, uint16_t flags)
{
  if (flags & CLK_MULT_ROUND_CLOSEST)
    {
      return abs(rate - new) < abs(rate - best);
    }

  return new >= rate && new < best;
}

static uint32_t clk_multiplier_bestmult(FAR struct clk_s *clk, uint32_t rate,
                                        FAR uint32_t *best_parent_rate)
{
  FAR struct clk_multiplier_s *multiplier = to_clk_multiplier(clk);
  uint32_t i;
  uint32_t bestmult = 0;
  uint32_t parent_rate;
  uint32_t best = 0;
  uint32_t now;
  uint32_t maxmult;
  uint32_t parent_rate_saved = *best_parent_rate;

  if (!rate)
    {
      rate = 1;
    }

  maxmult = _get_maxmult(multiplier);

  if (!(clk->flags & CLK_SET_RATE_PARENT))
    {
      parent_rate = *best_parent_rate;
      bestmult = rate / parent_rate;
      bestmult = bestmult == 0 ? 1 : bestmult;
      bestmult = bestmult > maxmult ? maxmult : bestmult;
      return bestmult;
    }

  for (i = maxmult; i >= 1; i--)
    {
      if (rate == parent_rate_saved * i)
        {
          *best_parent_rate = parent_rate_saved;
          return i;
        }

      parent_rate = clk_round_rate(clk_get_parent(clk),
                                   rate / i);
      now = parent_rate * i;
      if (__is_best_rate(rate, now, best, multiplier->flags))
        {
          bestmult = i;
          best = now;
          *best_parent_rate = parent_rate;
        }
    }

  if (!bestmult)
    {
      bestmult = 1;
      *best_parent_rate = clk_round_rate(clk_get_parent(clk), 1);
    }

  return bestmult;
}

static uint32_t clk_multiplier_round_rate(FAR struct clk_s *clk,
                                          uint32_t rate, FAR uint32_t *prate)
{
  return *prate * clk_multiplier_bestmult(clk, rate, prate);
}

static int clk_multiplier_set_rate(FAR struct clk_s *clk, uint32_t rate,
                                   uint32_t parent_rate)
{
  FAR struct clk_multiplier_s *multiplier = to_clk_multiplier(clk);
  uint32_t mult;
  uint32_t value;
  uint32_t val;

  mult = rate / parent_rate;
  value = _get_val(multiplier, mult);

  if (value > MASK(multiplier->width))
    {
      value = MASK(multiplier->width);
    }

  if (multiplier->flags & CLK_MULT_HIWORD_MASK)
    {
      val = MASK(multiplier->width) << (multiplier->shift + 16);
    }

  else
    {
      val = clk_read(multiplier->reg);
      val &= ~(MASK(multiplier->width) << multiplier->shift);
    }

  val |= value << multiplier->shift;
  clk_write(multiplier->reg, val);

  return 0;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops_s g_clk_multiplier_ops =
{
  .recalc_rate = clk_multiplier_recalc_rate,
  .round_rate = clk_multiplier_round_rate,
  .set_rate = clk_multiplier_set_rate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct clk_s *clk_register_multiplier(FAR const char *name,
                                          FAR const char *parent_name,
                                          uint8_t flags, uint32_t reg,
                                          uint8_t shift,
                                          uint8_t width,
                                          uint8_t clk_multiplier_flags)
{
  struct clk_multiplier_s mult;
  FAR const char **parent_names;
  uint8_t num_parents;

  if (clk_multiplier_flags & CLK_MULT_HIWORD_MASK)
    {
      if (width + shift > 16)
        {
          return NULL;
        }
    }

  parent_names = parent_name ? &parent_name : NULL;
  num_parents  = parent_name ? 1 : 0;

  mult.reg   = reg;
  mult.shift = shift;
  mult.width = width;
  mult.flags = clk_multiplier_flags;

  return clk_register(name, parent_names, num_parents, flags,
                      &g_clk_multiplier_ops, &mult, sizeof(mult));
}
