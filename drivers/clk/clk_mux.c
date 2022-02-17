/****************************************************************************
 * drivers/clk/clk_mux.c
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

#include <stdlib.h>

#include "clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_mux(_clk) (FAR struct clk_mux_s *)(_clk->private_data)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool mux_is_better_rate(uint32_t rate, uint32_t now,
                               uint32_t best, uint8_t flags)
{
  if (flags & CLK_MUX_ROUND_CLOSEST)
    {
      return abs(now - rate) < abs(best - rate);
    }

  return now <= rate && now > best;
}

static uint8_t clk_mux_get_parent(FAR struct clk_s *clk)
{
  FAR struct clk_mux_s *mux = to_clk_mux(clk);
  uint32_t val;

  val = clk_read(mux->reg) >> mux->shift;
  val &= MASK(mux->width);

  return val;
}

static int clk_mux_set_parent(FAR struct clk_s *clk, uint8_t index)
{
  FAR struct clk_mux_s *mux = to_clk_mux(clk);
  uint32_t mask = MASK(mux->width);
  uint32_t val;

  if (mux->flags & CLK_MUX_HIWORD_MASK)
    {
      val = mask << (mux->shift + 16);
    }
  else
    {
      val = clk_read(mux->reg);
      val &= ~(mask << mux->shift);
    }

  val |= index << mux->shift;
  clk_write(mux->reg, val);

  return 0;
}

static uint32_t
clk_mux_determine_rate(FAR struct clk_s *clk, uint32_t rate,
                       uint32_t *best_parent_rate,
                       struct clk_s **best_parent_p)
{
  FAR struct clk_mux_s *mux = to_clk_mux(clk);
  FAR struct clk_s *parent;
  struct clk_s *best_parent = NULL;
  uint8_t i;
  uint8_t num_parents;
  uint32_t parent_rate;
  uint32_t best = 0;

  if (clk->flags & CLK_SET_RATE_NO_REPARENT)
    {
      parent = clk->parent;
      if (clk->flags & CLK_SET_RATE_PARENT)
        {
          best = clk_round_rate(parent, rate);
        }
      else if (parent)
        {
          best = clk_get_rate(parent);
        }
      else
        {
          best = clk_get_rate(clk);
        }

      goto out;
    }

  num_parents = clk->num_parents;
  for (i = 0; i < num_parents; i++)
    {
      parent = clk_get_parent_by_index(clk, i);
      if (!parent)
        {
          continue;
        }

      if (clk->flags & CLK_SET_RATE_PARENT)
        {
          parent_rate = clk_round_rate(parent, rate);
        }

      else
        {
          parent_rate = clk_get_rate(parent);
        }

      if (mux_is_better_rate(rate, parent_rate, best, mux->flags))
        {
          best_parent = parent;
          best = parent_rate;
        }
    }

out:
  if (best_parent)
    {
      *best_parent_p = best_parent;
    }

  *best_parent_rate = best;

  return best;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops_s g_clk_mux_ops =
{
  .get_parent = clk_mux_get_parent,
  .set_parent = clk_mux_set_parent,
  .determine_rate = clk_mux_determine_rate,
};

const struct clk_ops_s g_clk_mux_ro_ops =
{
  .get_parent = clk_mux_get_parent,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct clk_s *clk_register_mux(FAR const char *name,
                                   const char * const *parent_names,
                                   uint8_t num_parents,
                                   uint8_t flags, uint32_t reg,
                                   uint8_t shift, uint8_t width,
                                   uint8_t clk_mux_flags)
{
  struct clk_mux_s mux;

  mux.reg   = reg;
  mux.shift = shift;
  mux.width = width;
  mux.flags = clk_mux_flags;

  if (clk_mux_flags & CLK_MUX_READ_ONLY)
    {
      return clk_register(name, parent_names, num_parents, flags,
                          &g_clk_mux_ro_ops, &mux, sizeof(mux));
    }
  else
    {
      return clk_register(name, parent_names, num_parents, flags,
                          &g_clk_mux_ops, &mux, sizeof(mux));
    }
}
