/****************************************************************************
 * drivers/clk/clk_phase.c
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

#define to_clk_phase(_clk) (FAR struct clk_phase_s *)(_clk->private_data)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int clk_phase_get_phase(FAR struct clk_s *clk)
{
  FAR struct clk_phase_s *phase = to_clk_phase(clk);
  uint32_t val;

  val = (clk_read(phase->reg) >> phase->shift) & MASK(phase->width);
  return DIV_ROUND_CLOSEST(360 * val, MASK(phase->width) + 1);
}

static int clk_phase_set_phase(FAR struct clk_s *clk, int degrees)
{
  FAR struct clk_phase_s *phase = to_clk_phase(clk);
  uint32_t pha;
  uint32_t val;

  pha = DIV_ROUND_CLOSEST((MASK(phase->width) + 1) * degrees, 360);

  if (pha > MASK(phase->width))
    {
      pha = MASK(phase->width);
    }

  if (phase->flags & CLK_PHASE_HIWORD_MASK)
    {
      val = MASK(phase->width) << (phase->shift + 16);
    }
  else
    {
      val = clk_read(phase->reg);
      val &= ~(MASK(phase->width) << phase->shift);
    }

  val |= pha << phase->shift;
  clk_write(phase->reg, val);

  return 0;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops_s g_clk_phase_ops =
{
  .get_phase = clk_phase_get_phase,
  .set_phase = clk_phase_set_phase,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct clk_s *clk_register_phase(FAR const char *name,
                                     FAR const char *parent_name,
                                     uint8_t flags, uint32_t reg,
                                     uint8_t shift, uint8_t width,
                                     uint8_t clk_phase_flags)
{
  struct clk_phase_s phase;
  FAR const char **parent_names;
  uint8_t num_parents;

  parent_names = parent_name ? &parent_name : NULL;
  num_parents = parent_name ? 1 : 0;

  phase.reg = reg;
  phase.shift = shift;
  phase.width = width;
  phase.flags = clk_phase_flags;

  return clk_register(name, parent_names, num_parents, flags,
                      &g_clk_phase_ops, &phase, sizeof(phase));
}
