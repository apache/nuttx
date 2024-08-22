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

#define kasan_random_tag() (rand() % ((1 << (64 - KASAN_TAG_SHIFT)) - 1))

#define KASAN_SHADOW_SCALE (sizeof(uintptr_t))

#define KASAN_SHADOW_SIZE(size) \
  ((size) + KASAN_SHADOW_SCALE - 1) / KASAN_SHADOW_SCALE
#define KASAN_REGION_SIZE(size) \
  (sizeof(struct kasan_region_s) + KASAN_SHADOW_SIZE(size))

#define KASAN_INIT_VALUE 0xdeadcafe

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kasan_region_s
{
  FAR struct kasan_region_s *next;
  uintptr_t begin;
  uintptr_t end;
  uint8_t   shadow[1];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_lock;
static FAR struct kasan_region_s *g_region;
static uint32_t g_region_init;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR uint8_t *kasan_mem_to_shadow(FAR const void *ptr, size_t size)
{
  FAR struct kasan_region_s *region;
  uintptr_t addr;

  addr = (uintptr_t)kasan_reset_tag(ptr);
  if (size == 0 || g_region_init != KASAN_INIT_VALUE)
    {
      return NULL;
    }

  for (region = g_region; region != NULL; region = region->next)
    {
      if (addr >= region->begin && addr < region->end)
        {
          DEBUGASSERT(addr + size <= region->end);
          addr -= region->begin;
          return &region->shadow[addr / KASAN_SHADOW_SCALE];
        }
    }

  return NULL;
}

static void kasan_set_poison(FAR const void *addr,
                             size_t size, uint8_t value)
{
  irqstate_t flags;
  FAR uint8_t *p;

  addr = kasan_reset_tag(addr);
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

bool kasan_is_poisoned(FAR const void *addr, size_t size)
{
  FAR uint8_t *p;
  uint8_t tag;

  tag = kasan_get_tag(addr);
  p = kasan_mem_to_shadow(addr, size);
  if (p == NULL)
    {
      return false;
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
  region->next  = g_region;
  g_region      = region;
  spin_unlock_irqrestore(&g_lock, flags);

  g_region_init = KASAN_INIT_VALUE;
  kasan_poison(addr, *size);
  *size -= KASAN_REGION_SIZE(*size);
}

void kasan_init_early(void)
{
  g_region_init = 0;
}
