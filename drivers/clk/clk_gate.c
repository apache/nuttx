/****************************************************************************
 * drivers/clk/clk_gate.c
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

#include <debug.h>

#include "clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_gate(_clk) (FAR struct clk_gate_s *)(_clk->private_data)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void clk_gate_endisable(FAR struct clk_s *clk, int32_t enable)
{
  FAR struct clk_gate_s *gate = to_clk_gate(clk);
  int32_t set = gate->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;
  uint32_t val;

  set ^= enable;

  if (gate->flags & CLK_GATE_HIWORD_MASK)
    {
      val = BIT(gate->bit_idx + 16);
      if (set)
        {
          val |= BIT(gate->bit_idx);
        }
    }

  else
    {
      val = clk_read(gate->reg);

      if (set)
        {
          val |= BIT(gate->bit_idx);
        }
      else
        {
          val &= ~BIT(gate->bit_idx);
        }
    }

  clk_write(gate->reg, val);
}

static int clk_gate_enable(FAR struct clk_s *clk)
{
  clk_gate_endisable(clk, 1);
  return 0;
}

static void clk_gate_disable(FAR struct clk_s *clk)
{
  clk_gate_endisable(clk, 0);
}

static int clk_gate_is_enabled(FAR struct clk_s *clk)
{
  FAR struct clk_gate_s *gate = to_clk_gate(clk);
  uint32_t val;

  val = clk_read(gate->reg);

  if (gate->flags & CLK_GATE_SET_TO_DISABLE)
    {
      val ^= BIT(gate->bit_idx);
    }

  val &= BIT(gate->bit_idx);

  return val ? 1 : 0;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops_s g_clk_gate_ops =
{
  .enable = clk_gate_enable,
  .disable = clk_gate_disable,
  .is_enabled = clk_gate_is_enabled,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct clk_s *clk_register_gate(FAR const char *name,
                                    FAR const char *parent_name,
                                    uint8_t flags, uint32_t reg,
                                    uint8_t bit_idx,
                                    uint8_t clk_gate_flags)
{
  FAR struct clk_gate_s gate;
  FAR const char **parent_names;
  uint8_t num_parents;

  if (clk_gate_flags & CLK_GATE_HIWORD_MASK)
    {
      if (bit_idx > 16)
        {
          return NULL;
        }
    }

  parent_names = parent_name ? &parent_name : NULL;
  num_parents = parent_name ? 1 : 0;

  gate.reg = reg;
  gate.bit_idx = bit_idx;
  gate.flags = clk_gate_flags;

  return clk_register(name, parent_names, num_parents, flags,
                      &g_clk_gate_ops, &gate, sizeof(gate));
}
