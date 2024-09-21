/****************************************************************************
 * mm/kasan/generic.c
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

#include <nuttx/nuttx.h>
#include <nuttx/mm/kasan.h>
#include <nuttx/compiler.h>
#include <nuttx/spinlock.h>

#include <assert.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define KASAN_BYTES_PER_WORD (sizeof(uintptr_t))
#define KASAN_BITS_PER_WORD  (KASAN_BYTES_PER_WORD * 8)

#define KASAN_FIRST_WORD_MASK(start) \
  (UINTPTR_MAX << ((start) & (KASAN_BITS_PER_WORD - 1)))
#define KASAN_LAST_WORD_MASK(end) \
  (UINTPTR_MAX >> (-(end) & (KASAN_BITS_PER_WORD - 1)))

#define KASAN_SHADOW_SCALE (sizeof(uintptr_t))

#define KASAN_SHADOW_SIZE(size) \
  (KASAN_BYTES_PER_WORD * ((size) / KASAN_SHADOW_SCALE / KASAN_BITS_PER_WORD))
#define KASAN_REGION_SIZE(size) \
  (sizeof(struct kasan_region_s) + KASAN_SHADOW_SIZE(size))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kasan_region_s
{
  uintptr_t begin;
  uintptr_t end;
  uintptr_t shadow[1];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct kasan_region_s *g_region[CONFIG_MM_KASAN_REGIONS];
static size_t g_region_count;
static spinlock_t g_lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline_function FAR uintptr_t *
kasan_mem_to_shadow(FAR const void *ptr, size_t size,
                    FAR unsigned int *bit)
{
  uintptr_t addr = (uintptr_t)ptr;
  size_t i;

  for (i = 0; i < g_region_count; i++)
    {
      if (addr >= g_region[i]->begin && addr < g_region[i]->end)
        {
          DEBUGASSERT(addr + size <= g_region[i]->end);
          addr -= g_region[i]->begin;
          addr /= KASAN_SHADOW_SCALE;
          *bit  = addr % KASAN_BITS_PER_WORD;
          return &g_region[i]->shadow[addr / KASAN_BITS_PER_WORD];
        }
    }

  return NULL;
}

static inline_function bool
kasan_is_poisoned(FAR const void *addr, size_t size)
{
  FAR uintptr_t *p;
  unsigned int bit;
  unsigned int nbit;
  uintptr_t mask;

  p = kasan_mem_to_shadow(addr, size, &bit);
  if (p == NULL)
    {
      return kasan_global_is_poisoned(addr, size);
    }

  if (size <= KASAN_SHADOW_SCALE)
    {
      return ((*p >> bit) & 1);
    }

  nbit = KASAN_BITS_PER_WORD - bit % KASAN_BITS_PER_WORD;
  mask = KASAN_FIRST_WORD_MASK(bit);
  size = ALIGN_UP(size, KASAN_SHADOW_SCALE);
  size /= KASAN_SHADOW_SCALE;

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
      mask &= KASAN_LAST_WORD_MASK(bit + size);
      if ((*p & mask) != 0)
        {
          return true;
        }
    }

  return false;
}

static void kasan_set_poison(FAR const void *addr, size_t size,
                             bool poisoned)
{
  FAR uintptr_t *p;
  irqstate_t flags;
  unsigned int bit;
  unsigned int nbit;
  uintptr_t mask;

  p = kasan_mem_to_shadow(addr, size, &bit);
  if (p == NULL)
    {
      return;
    }

  nbit = KASAN_BITS_PER_WORD - bit % KASAN_BITS_PER_WORD;
  mask = KASAN_FIRST_WORD_MASK(bit);
  size /= KASAN_SHADOW_SCALE;

  flags = spin_lock_irqsave(&g_lock);
  while (size >= nbit)
    {
      if (poisoned)
        {
          *p++ |= mask;
        }
      else
        {
          *p++ &= ~mask;
        }

      bit  += nbit;
      size -= nbit;

      nbit = KASAN_BITS_PER_WORD;
      mask = UINTPTR_MAX;
    }

  if (size)
    {
      mask &= KASAN_LAST_WORD_MASK(bit + size);
      if (poisoned)
        {
          *p |= mask;
        }
      else
        {
          *p &= ~mask;
        }
    }

  spin_unlock_irqrestore(&g_lock, flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *kasan_reset_tag(FAR const void *addr)
{
  return (FAR void *)addr;
}

void kasan_poison(FAR const void *addr, size_t size)
{
  kasan_set_poison(addr, size, true);
}

FAR void *kasan_unpoison(FAR const void *addr, size_t size)
{
  kasan_set_poison(addr, size, false);
  return (FAR void *)addr;
}

void kasan_register(FAR void *addr, FAR size_t *size)
{
  FAR struct kasan_region_s *region;
  irqstate_t flags;

  region = (FAR struct kasan_region_s *)
    ((FAR char *)addr + *size - KASAN_REGION_SIZE(*size));

  region->begin = (uintptr_t)addr;
  region->end   = region->begin + *size;

  flags = spin_lock_irqsave(&g_lock);

  DEBUGASSERT(g_region_count <= CONFIG_MM_KASAN_REGIONS);
  g_region[g_region_count++] = region;

  spin_unlock_irqrestore(&g_lock, flags);

  kasan_start();
  kasan_poison(addr, *size);
  *size -= KASAN_REGION_SIZE(*size);
}

void kasan_unregister(FAR void *addr)
{
  irqstate_t flags;
  size_t i;

  flags = spin_lock_irqsave(&g_lock);
  for (i = 0; i < g_region_count; i++)
    {
      if (g_region[i]->begin == (uintptr_t)addr)
        {
          g_region_count--;
          memmove(&g_region[i], &g_region[i + 1],
                  (g_region_count - i) * sizeof(g_region[0]));
          break;
        }
    }

  spin_unlock_irqrestore(&g_lock, flags);
}
