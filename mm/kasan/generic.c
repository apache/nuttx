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

#ifdef CONFIG_MM_KASAN_GLOBAL

#  define KASAN_GLOBAL_SHADOW_SCALE (32)

#  define KASAN_GLOBAL_NEXT_REGION(region) \
  (FAR struct kasan_region_s *) \
  ((FAR char *)region->shadow + (size_t)region->next)

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kasan_region_s
{
  FAR struct kasan_region_s *next;
  uintptr_t begin;
  uintptr_t end;
  uintptr_t shadow[1];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_lock;
static FAR struct kasan_region_s *g_region;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN_GLOBAL
extern const unsigned char g_globals_region[];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline_function FAR uintptr_t *
kasan_mem_to_shadow(FAR const void *ptr, size_t size,
                    FAR unsigned int *bit, FAR size_t *align)
{
  FAR struct kasan_region_s *region;
  uintptr_t addr = (uintptr_t)ptr;

  for (region = g_region; region != NULL; region = region->next)
    {
      if (addr >= region->begin && addr < region->end)
        {
          DEBUGASSERT(addr + size <= region->end);
          addr -= region->begin;
          *align = KASAN_SHADOW_SCALE;
          addr /= KASAN_SHADOW_SCALE;
          *bit  = addr % KASAN_BITS_PER_WORD;
          return &region->shadow[addr / KASAN_BITS_PER_WORD];
        }
    }

#ifdef CONFIG_MM_KASAN_GLOBAL
  for (region = (FAR struct kasan_region_s *)g_globals_region;
       region->next;
       region = KASAN_GLOBAL_NEXT_REGION(region))
    {
      if (addr >= region->begin && addr < region->end)
        {
          DEBUGASSERT(addr + size <= region->end);
          addr -= region->begin;
          *align = KASAN_GLOBAL_SHADOW_SCALE;
          addr /= KASAN_GLOBAL_SHADOW_SCALE;
          *bit  = addr % KASAN_BITS_PER_WORD;
          return &region->shadow[addr / KASAN_BITS_PER_WORD];
        }
    }
#endif

  return NULL;
}

static inline_function bool
kasan_is_poisoned(FAR const void *addr, size_t size)
{
  FAR uintptr_t *p;
  unsigned int bit;
  unsigned int nbit;
  uintptr_t mask;
  size_t align;

  p = kasan_mem_to_shadow(addr, size, &bit, &align);
  if (p == NULL)
    {
      return false;
    }

  if (size <= align)
    {
      return ((*p >> bit) & 1);
    }

  nbit = KASAN_BITS_PER_WORD - bit % KASAN_BITS_PER_WORD;
  mask = KASAN_FIRST_WORD_MASK(bit);
  size = ALIGN_UP(size, align);
  size /= align;

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
  size_t align;

  p = kasan_mem_to_shadow(addr, size, &bit, &align);
  if (p == NULL)
    {
      return;
    }

  nbit = KASAN_BITS_PER_WORD - bit % KASAN_BITS_PER_WORD;
  mask = KASAN_FIRST_WORD_MASK(bit);
  size /= align;

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
  region->next  = g_region;
  g_region      = region;
  spin_unlock_irqrestore(&g_lock, flags);

  kasan_start();
  kasan_poison(addr, *size);
  *size -= KASAN_REGION_SIZE(*size);
}

void kasan_unregister(FAR void *addr)
{
  FAR struct kasan_region_s *prev = NULL;
  FAR struct kasan_region_s *region;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_lock);
  for (region = g_region; region != NULL; region = region->next)
    {
      if (region->begin == (uintptr_t)addr)
        {
          if (region == g_region)
            {
              g_region = region->next;
            }
          else
            {
              prev->next = region->next;
            }

          break;
        }

      prev = region;
    }

  spin_unlock_irqrestore(&g_lock, flags);
}
