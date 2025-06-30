/****************************************************************************
 * mm/kasan/hw_tags.c
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

#include <nuttx/arch.h>

/****************************************************************************
 * Private Function
 ****************************************************************************/

static FAR void *
kasan_poison_tag(FAR const void *addr, size_t size, uint8_t tag)
{
  FAR void *tag_addr;

  /* Label this address pointer */

  tag_addr = up_memtag_set_tag(addr, tag);

  /* Add MTE hardware label to memory block */

  up_memtag_tag_mem(tag_addr, size);

  return tag_addr;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool kasan_bypass(bool state)
{
  return up_memtag_bypass(state);
}

FAR void *kasan_clear_tag(FAR const void *addr)
{
  return up_memtag_set_tag(addr, 0);
}

void kasan_poison(FAR const void *addr, size_t size)
{
  uint8_t tag = up_memtag_get_random_tag(addr);

  kasan_poison_tag(addr, size, tag);
}

uint8_t kasan_get_tag(FAR const void *addr)
{
  return up_memtag_get_tag(addr);
}

FAR void *kasan_set_tag(FAR const void *addr, uint8_t tag)
{
  return up_memtag_set_tag(addr, tag);
}

FAR void *kasan_unpoison(FAR const void *addr, size_t size)
{
  uint8_t tag = up_memtag_get_random_tag(addr);

  return kasan_poison_tag(addr, size, tag);
}

void kasan_register(FAR void *addr, FAR size_t *size)
{
  uint8_t tag = up_memtag_get_random_tag(addr);

  kasan_poison_tag(addr, *size, tag);
}

void kasan_unregister(FAR void *addr)
{
}
