/****************************************************************************
 * drivers/clk/clk.h
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

#ifndef __DRIVER_CLK_CLK_H
#define __DRIVER_CLK_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <strings.h>
#include <sys/param.h>

#include <nuttx/bits.h>
#include <nuttx/lib/math32.h>

#ifdef CONFIG_CLK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MASK(width)                 (BIT(width) - 1)
#define MULT_ROUND_UP(r, m)         ((r) * (m) + (m) - 1)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline void clk_write(uint32_t reg, uint32_t value)
{
  *((volatile uint32_t *)(uintptr_t)reg) = value;
}

static inline uint32_t clk_read(uint32_t reg)
{
  return *((volatile uint32_t *)(uintptr_t)reg);
}

static inline bool clk_is_best_rate_closest(uint32_t rate, uint32_t now,
                                            uint32_t best)
{
  uint32_t rate1 = rate > now ? (rate - now) : (now - rate);
  uint32_t rate2 = rate > best ? (rate - best) : (best - rate);
  return rate1 < rate2;
}

static inline uint32_t gcd(uint32_t a, uint32_t b)
{
  uint32_t r;
  uint32_t tmp;

  if (a < b)
    {
      tmp = a;
      a = b;
      b = tmp;
    }

  if (!b)
    {
      return a;
    }

  while ((r = a % b) != 0)
    {
      a = b;
      b = r;
    }

  return b;
}

static inline uint32_t roundup_double(double n)
{
  uint32_t intn = (uint32_t)n;
  if (n == (double)intn)
    {
      return intn;
    }

  return intn + 1;
}

#endif /* CONFIG_CLK */
#endif /* __DRIVER_CLK_CLK_H */
