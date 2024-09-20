/****************************************************************************
 * mm/kasan/global.c
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

#include <nuttx/nuttx.h>
#include <nuttx/mm/kasan.h>

#include <assert.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define KASAN_BYTES_PER_WORD (sizeof(uintptr_t))
#define KASAN_BITS_PER_WORD  (KASAN_BYTES_PER_WORD * 8)

#define KASAN_GLOBAL_FIRST_WORD_MASK(start) \
  (UINTPTR_MAX << ((start) & (KASAN_BITS_PER_WORD - 1)))
#define KASAN_GLOBAL_LAST_WORD_MASK(end) \
  (UINTPTR_MAX >> (-(end) & (KASAN_BITS_PER_WORD - 1)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kasan_global_region_s
{
  uintptr_t begin;
  uintptr_t end;
  uintptr_t shadow[1];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct kasan_global_region_s *g_global_region[];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline_function FAR uintptr_t *
kasan_global_mem_to_shadow(FAR const void *ptr, size_t size,
                           FAR unsigned int *bit)
{
  uintptr_t addr = (uintptr_t)ptr;
  size_t i;

  for (i = 0; g_global_region[i]; i++)
    {
      if (addr >= g_global_region[i]->begin &&
          addr < g_global_region[i]->end)
        {
          DEBUGASSERT(addr + size <= g_global_region[i]->end);
          addr -= g_global_region[i]->begin;
          addr /= CONFIG_MM_KASAN_GLOBAL_ALIGN;
          *bit  = addr % KASAN_BITS_PER_WORD;
          return (FAR uintptr_t *)
                 &g_global_region[i]->shadow[addr / KASAN_BITS_PER_WORD];
        }
    }

  return NULL;
}

static inline_function bool
kasan_global_is_poisoned(FAR const void *addr, size_t size)
{
  FAR uintptr_t *p;
  unsigned int bit;
  unsigned int nbit;
  uintptr_t mask;

  p = kasan_global_mem_to_shadow(addr, size, &bit);
  if (p == NULL)
    {
      return false;
    }

  if (size <= CONFIG_MM_KASAN_GLOBAL_ALIGN)
    {
      return ((*p >> bit) & 1);
    }

  nbit = KASAN_BITS_PER_WORD - bit % KASAN_BITS_PER_WORD;
  mask = KASAN_GLOBAL_FIRST_WORD_MASK(bit);
  size = ALIGN_UP(size, CONFIG_MM_KASAN_GLOBAL_ALIGN);
  size /= CONFIG_MM_KASAN_GLOBAL_ALIGN;

  while (size >= nbit)
    {
      if ((*p++ & mask) != 0)
        {
          return true;
        }

      bit  += nbit;
      size -= nbit;

      nbit = KASAN_BITS_PER_WORD;
      mask = UINTPTR_MAX;
    }

  if (size)
    {
      mask &= KASAN_GLOBAL_LAST_WORD_MASK(bit + size);
      if ((*p & mask) != 0)
        {
          return true;
        }
    }

  return false;
}
