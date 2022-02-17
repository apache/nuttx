/****************************************************************************
 * drivers/clk/clk_divider.c
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

#define to_clk_divider(_clk) (FAR struct clk_divider_s *) \
                             (_clk->private_data)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t _get_maxdiv(uint8_t width, uint16_t flags)
{
  if (flags & CLK_DIVIDER_MAX_HALF)
    {
      return MASK(width - 1) + 1;
    }

  if (flags & CLK_DIVIDER_ONE_BASED)
    {
      return MASK(width);
    }

  if (flags & CLK_DIVIDER_DIV_NEED_EVEN)
    {
      return MASK(width) - 1;
    }

  if (flags & CLK_DIVIDER_POWER_OF_TWO)
    {
      return 1 << MASK(width);
    }

  return MASK(width) + 1;
}

static uint32_t _get_div(uint32_t val, uint16_t flags)
{
  if (flags & CLK_DIVIDER_ONE_BASED)
    {
      return val;
    }

  if (flags & CLK_DIVIDER_POWER_OF_TWO)
    {
      return 1 << val;
    }

  return val + 1;
}

static uint32_t _get_val(uint32_t div, uint16_t flags)
{
  if (flags & CLK_DIVIDER_ONE_BASED)
    {
      return div;
    }

  if (flags & CLK_DIVIDER_POWER_OF_TWO)
    {
      return ffs(div);
    }

  return div - 1;
}

static uint32_t divider_recalc_rate(uint32_t parent_rate,
                                    uint32_t val, uint16_t flags)
{
  uint32_t div;

  div = _get_div(val, flags);
  if (!div)
    {
      return parent_rate;
    }

  return DIV_ROUND_UP(parent_rate, div);
}

static uint32_t clk_divider_recalc_rate(FAR struct clk_s *clk,
                                        uint32_t parent_rate)
{
  FAR struct clk_divider_s *divider = to_clk_divider(clk);
  uint32_t val;

  val = clk_read(divider->reg) >> divider->shift;
  val &= MASK(divider->width);

  return divider_recalc_rate(parent_rate, val, divider->flags);
}

static uint32_t _div_round_up(uint32_t parent_rate, uint32_t rate,
                              uint16_t flags)
{
  uint32_t div = DIV_ROUND_UP(parent_rate, rate);

  if (flags & CLK_DIVIDER_POWER_OF_TWO)
    {
      div = roundup_pow_of_two(div);
    }

  return div;
}

static uint32_t _div_round_closest(uint32_t parent_rate,
                                   uint32_t rate, uint16_t flags)
{
  uint32_t up;
  uint32_t down;
  uint32_t up_rate;
  uint32_t down_rate;

  up = DIV_ROUND_UP(parent_rate, rate);
  down = parent_rate / rate;

  if (flags & CLK_DIVIDER_POWER_OF_TWO)
    {
      up = roundup_pow_of_two(up);
      down = rounddown_pow_of_two(down);
    }

  up_rate = DIV_ROUND_UP(parent_rate, up);
  down_rate = DIV_ROUND_UP(parent_rate, down);

  return (rate - up_rate) <= (down_rate - rate) ? up : down;
}

static uint32_t _div_round(uint32_t parent_rate, uint32_t rate,
                           uint16_t flags)
{
  if (flags & CLK_DIVIDER_ROUND_CLOSEST)
    {
      return _div_round_closest(parent_rate, rate, flags);
    }

  return _div_round_up(parent_rate, rate, flags);
}

static bool _is_best_div(uint32_t rate, uint32_t now,
                         uint32_t best, uint16_t flags)
{
  if (flags & CLK_DIVIDER_ROUND_CLOSEST)
    {
      return abs(rate - now) < abs(rate - best);
    }

  return now <= rate && now > best;
}

static uint32_t _next_div(uint32_t div, uint16_t flags)
{
  div++;

  if (flags & CLK_DIVIDER_POWER_OF_TWO)
    {
      return roundup_pow_of_two(div);
    }

  if (flags & CLK_DIVIDER_DIV_NEED_EVEN)
    {
      div++;
    }

  return div;
}

static uint32_t clk_divider_bestdiv(FAR struct clk_s *clk, uint32_t rate,
                                    uint32_t *best_parent_rate,
                                    uint8_t width)
{
  uint32_t i;
  uint32_t bestdiv = 0;
  uint32_t maxdiv;
  uint32_t mindiv;
  uint32_t parent_rate;
  uint32_t best = 0;
  uint32_t now;
  FAR struct clk_divider_s *divider = to_clk_divider(clk);

  if (!rate)
    {
      rate = 1;
    }

  if (divider->flags & CLK_DIVIDER_READ_ONLY)
    {
      bestdiv = clk_read(divider->reg) >> divider->shift;
      bestdiv &= MASK(divider->width);
      bestdiv = _get_div(bestdiv, divider->flags);
      return bestdiv;
    }

  maxdiv = _get_maxdiv(width, divider->flags);

  if (!(clk->flags & CLK_SET_RATE_PARENT))
    {
      parent_rate = *best_parent_rate;
      if (parent_rate > rate)
        {
          bestdiv = _div_round(parent_rate, rate, divider->flags);
          bestdiv = bestdiv > maxdiv ? maxdiv : bestdiv;
        }
      else
        {
          bestdiv = 1;
        }

      return bestdiv;
    }

  mindiv = 0;
  if (divider->flags & CLK_DIVIDER_MINDIV_MSK)
    {
      mindiv = (divider->flags & CLK_DIVIDER_MINDIV_MSK)
                >> CLK_DIVIDER_MINDIV_OFF;
      mindiv -= 1;
    }

  maxdiv = MIN(UINT32_MAX / rate, maxdiv);
  for (i = _next_div(mindiv, divider->flags); i <= maxdiv;
       i = _next_div(i, divider->flags))
    {
      parent_rate = clk_round_rate(clk_get_parent(clk),
          rate * i);
      now = DIV_ROUND_UP(parent_rate, i);
      if (_is_best_div(rate, now, best, divider->flags))
        {
          bestdiv = i;
          best = now;
          *best_parent_rate = parent_rate;
        }
    }

  if (!bestdiv)
    {
      bestdiv = _get_maxdiv(width, divider->flags);
      *best_parent_rate = clk_round_rate(clk_get_parent(clk), 1);
    }

  return bestdiv;
}

static uint32_t divider_round_rate(FAR struct clk_s *clk, uint32_t rate,
                                   uint32_t *prate, uint8_t width)
{
  uint32_t div;

  div = clk_divider_bestdiv(clk, rate, prate, width);

  return DIV_ROUND_UP(*prate, div);
}

static uint32_t clk_divider_round_rate(FAR struct clk_s *clk, uint32_t rate,
                                       uint32_t *prate)
{
  FAR struct clk_divider_s *divider = to_clk_divider(clk);

  return divider_round_rate(clk, rate, prate, divider->width);
}

static int32_t divider_get_val(uint32_t rate, uint32_t parent_rate,
                               uint8_t width, uint16_t flags)
{
  uint32_t div;
  uint32_t value;

  div = DIV_ROUND_UP(parent_rate, rate);

  if ((flags & CLK_DIVIDER_POWER_OF_TWO) && !is_power_of_2(div))
    {
      return -EINVAL;
    }

  value = _get_val(div, flags);

  return MIN(value, MASK(width));
}

static int clk_divider_set_rate(FAR struct clk_s *clk, uint32_t rate,
                                uint32_t parent_rate)
{
  FAR struct clk_divider_s *divider = to_clk_divider(clk);
  int32_t value;
  uint32_t val;

  value = divider_get_val(rate, parent_rate, divider->width, divider->flags);

  if (value < 0)
    {
      return value;
    }

  if (divider->flags & CLK_DIVIDER_HIWORD_MASK)
    {
      val = MASK(divider->width) << (divider->shift + 16);
    }
  else
    {
      val = clk_read(divider->reg);
      val &= ~(MASK(divider->width) << divider->shift);
    }

  val |= value << divider->shift;
  clk_write(divider->reg, val);

  return 0;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops_s g_clk_divider_ops =
{
  .recalc_rate = clk_divider_recalc_rate,
  .round_rate = clk_divider_round_rate,
  .set_rate = clk_divider_set_rate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct clk_s *clk_register_divider(FAR const char *name,
                                       FAR const char *parent_name,
                                       uint8_t flags, uint32_t reg,
                                       uint8_t shift, uint8_t width,
                                       uint16_t clk_divider_flags)
{
  struct clk_divider_s div;
  FAR const char **parent_names;
  uint8_t num_parents;

  if (clk_divider_flags & CLK_DIVIDER_HIWORD_MASK)
    {
      if (width + shift > 16)
        {
          return NULL;
        }
    }

  parent_names = parent_name ? &parent_name: NULL;
  num_parents = parent_name ? 1 : 0;

  div.reg = reg;
  div.shift = shift;
  div.width = width;
  div.flags = clk_divider_flags;

  return clk_register(name, parent_names, num_parents, flags,
                      &g_clk_divider_ops, &div, sizeof(div));
}
