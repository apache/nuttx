/****************************************************************************
 * mm/kasan/sw_tags.c
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

#include <nuttx/mm/kasan.h>
#include <nuttx/compiler.h>
#include <nuttx/spinlock.h>

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define KASAN_TAG_SHIFT 56

#define kasan_get_tag(addr) \
  ((uint8_t)((uint64_t)(addr) >> KASAN_TAG_SHIFT))

#define kasan_set_tag(addr, tag) \
  (FAR void *)((((uint64_t)(addr)) & ~((uint64_t)0xff << KASAN_TAG_SHIFT)) | \
               (((uint64_t)(tag)) << KASAN_TAG_SHIFT))

#define kasan_random_tag() (1 + rand() % ((1 << (64 - KASAN_TAG_SHIFT)) - 2))

#define KASAN_SHADOW_SCALE (sizeof(uintptr_t))

#define KASAN_SHADOW_SIZE(size) \
  ((size) + KASAN_SHADOW_SCALE - 1) / KASAN_SHADOW_SCALE
#define KASAN_REGION_SIZE(size) \
  (sizeof(struct kasan_region_s) + KASAN_SHADOW_SIZE(size))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kasan_region_s
{
  uintptr_t begin;
  uintptr_t end;
  uint8_t   shadow[1];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct kasan_region_s *g_region[CONFIG_MM_KASAN_REGIONS];
static int g_region_count;
static spinlock_t g_lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline_function FAR uint8_t *
kasan_mem_to_shadow(FAR const void *ptr, size_t size)
{
  uintptr_t addr;
  int i;

  addr = (uintptr_t)kasan_reset_tag(ptr);

  for (i = 0; i < g_region_count; i++)
    {
      if (addr >= g_region[i]->begin && addr < g_region[i]->end)
        {
          DEBUGASSERT(addr + size <= g_region[i]->end);
          addr -= g_region[i]->begin;
          return &g_region[i]->shadow[addr / KASAN_SHADOW_SCALE];
        }
    }

  return NULL;
}

static inline_function bool
kasan_is_poisoned(FAR const void *addr, size_t size)
{
  FAR uint8_t *p;
  uint8_t tag;

  tag = kasan_get_tag(addr);

#ifdef CONFIG_MM_KASAN_SKIP_ZERO_TAGS
  if (tag == 0)
    {
      return false;
    }
#endif

  p = kasan_mem_to_shadow(addr, size);
  if (p == NULL)
    {
      return kasan_global_is_poisoned(addr, size);
    }

  size = KASAN_SHADOW_SIZE(size);
  while (size--)
    {
      if (p[size] != tag)
        {
          return true;
        }
    }

  return false;
}

static void kasan_set_poison(FAR const void *addr,
                             size_t size, uint8_t value)
{
  irqstate_t flags;
  FAR uint8_t *p;

  p = kasan_mem_to_shadow(addr, size);
  if (p == NULL)
    {
      return;
    }

  size = KASAN_SHADOW_SIZE(size);
  flags = spin_lock_irqsave(&g_lock);

  while (size--)
    {
      p[size] = value;
    }

  spin_unlock_irqrestore(&g_lock, flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *kasan_reset_tag(FAR const void *addr)
{
  return (FAR void *)
         (((uint64_t)(addr)) & ~((uint64_t)0xff << KASAN_TAG_SHIFT));
}

void kasan_poison(FAR const void *addr, size_t size)
{
  kasan_set_poison(addr, size, 0xff);
}

FAR void *kasan_unpoison(FAR const void *addr, size_t size)
{
  uint8_t tag = kasan_random_tag();

  kasan_set_poison(addr, size, tag);
  return kasan_set_tag(addr, tag);
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
