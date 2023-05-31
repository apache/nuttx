/****************************************************************************
 * arch/arm/src/armv8-r/cp15_cacheops.c
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
#include <nuttx/cache.h>
#include <nuttx/irq.h>

#include "cp15_cacheops.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t ilog2(uint32_t u)
{
  int i = 0;

  while (u >>= 1)
    {
      i++;
    }

  return i;
}

static inline uint32_t cp15_cache_get_info(uint32_t *sets, uint32_t *ways)
{
  uint32_t ccsidr = CP15_GET(CCSIDR);

  if (sets)
    {
      *sets = ((ccsidr >> 13) & 0x7fff) + 1;
    }

  if (ways)
    {
      *ways = ((ccsidr >> 3) & 0x3ff) + 1;
    }

  return (1 << ((ccsidr & 0x7) + 2)) * 4;
}

static void cp15_dcache_op(int op)
{
  uint32_t clidr = CP15_GET(CLIDR);
  int level;

  for (level = 0; level < 7; level++)
    {
      uint32_t ctype = clidr & 0x7;

      switch (ctype)
        {
          case 0x2:
          case 0x3:
          case 0x4:
            cp15_dcache_op_level(level, op);
            break;
          default:
            break;
        }

      clidr >>= 3;
      if (clidr == 0)
        {
          break;
        }
    }
}

static void cp15_dcache_op_mva(uintptr_t start, uintptr_t end, int op)
{
  uint32_t line;

  line = cp15_cache_get_info(NULL, NULL);

  ARM_DSB();

  if ((start & (line - 1)) != 0)
    {
      start &= ~(line - 1);
      if (op == CP15_CACHE_INVALIDATE)
        {
          cp15_cleaninvalidate_dcacheline_bymva(start);
          start += line;
        }
    }

  while (start < end)
    {
      switch (op)
        {
          case CP15_CACHE_INVALIDATE:
            if (start + line <= end)
              {
                cp15_invalidate_dcacheline_bymva(start);
              }
            else
              {
                cp15_cleaninvalidate_dcacheline_bymva(start);
              }
            break;
          case CP15_CACHE_CLEAN:
            cp15_clean_dcache_bymva(start);
            break;
          case CP15_CACHE_CLEANINVALIDATE:
            cp15_cleaninvalidate_dcacheline_bymva(start);
            break;
          default:
            break;
        }

      start += line;
    }

  ARM_ISB();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void cp15_dcache_op_level(uint32_t level, int op)
{
  uint32_t sets;
  uint32_t ways;
  uint32_t set;
  uint32_t way;
  uint32_t line;
  uint32_t way_shift;
  uint32_t set_shift;
  uint32_t val = level << 1;

  /* Select by CSSELR */

  CP15_SET(CSSELR, val);

  /* Get cache info */

  line = cp15_cache_get_info(&sets, &ways);

  way_shift = 32 - ilog2(ways);
  set_shift = ilog2(line);

  ARM_DSB();

  /* A: Log2(ways)
   * B: L+S
   * L: Log2(line)
   * S: Log2(sets)
   *
   * The bits are packed as follows:
   *  31  31-A        B B-1    L L-1   4 3   1 0
   * |---|-------------|--------|-------|-----|-|
   * |Way|    zeros    |   Set  | zeros |level|0|
   * |---|-------------|--------|-------|-----|-|
   */

  for (way = 0; way < ways; way++)
    {
      for (set = 0; set < sets; set++)
        {
          val  = level << 1;
          val |= way << way_shift;
          val |= set << set_shift;

          switch (op)
            {
              case CP15_CACHE_INVALIDATE:
                cp15_invalidate_dcacheline_bysetway(val);
                break;
              case CP15_CACHE_CLEAN:
                cp15_clean_dcache_bysetway(val);
                break;
              case CP15_CACHE_CLEANINVALIDATE:
                cp15_cleaninvalidate_dcacheline(val);
                break;
              default:
                break;
            }
        }
    }

  ARM_ISB();
}

void cp15_invalidate_icache(uintptr_t start, uintptr_t end)
{
  uint32_t line;

  line = cp15_cache_get_info(NULL, NULL);
  start &= ~(line - 1);

  ARM_DSB();

  while (start < end)
    {
      cp15_invalidate_icache_bymva(start);
      start += line;
    }

  ARM_ISB();
}

void cp15_coherent_dcache(uintptr_t start, uintptr_t end)
{
  cp15_dcache_op_mva(start, end, CP15_CACHE_CLEANINVALIDATE);
  cp15_invalidate_icache_all();
}

void cp15_invalidate_dcache(uintptr_t start, uintptr_t end)
{
  cp15_dcache_op_mva(start, end, CP15_CACHE_INVALIDATE);
}

void cp15_invalidate_dcache_all(void)
{
  cp15_dcache_op(CP15_CACHE_INVALIDATE);
}

void cp15_clean_dcache(uintptr_t start, uintptr_t end)
{
  cp15_dcache_op_mva(start, end, CP15_CACHE_CLEAN);
}

void cp15_clean_dcache_all(void)
{
  cp15_dcache_op(CP15_CACHE_CLEAN);
}

void cp15_flush_dcache(uintptr_t start, uintptr_t end)
{
  cp15_dcache_op_mva(start, end, CP15_CACHE_CLEANINVALIDATE);
}

void cp15_flush_dcache_all(void)
{
  cp15_dcache_op(CP15_CACHE_CLEANINVALIDATE);
}

uint32_t cp15_cache_size(void)
{
  uint32_t sets;
  uint32_t ways;
  uint32_t line;

  line = cp15_cache_get_info(&sets, &ways);

  return sets * ways * line;
}

uint32_t cp15_cache_linesize(void)
{
  return cp15_cache_get_info(NULL, NULL);
}
