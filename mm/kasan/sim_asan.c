/****************************************************************************
 * mm/kasan/sim_asan.c
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

#include <sanitizer/asan_interface.h>

#include <nuttx/nuttx.h>
#include <stdint.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint8_t kasan_get_tag(FAR const void *addr)
{
  return 0;
}

FAR void *kasan_set_tag(FAR const void *addr, uint8_t tag)
{
  return (FAR void *)addr;
}

FAR void *kasan_clear_tag(FAR const void *addr)
{
  return (FAR void *)addr;
}

void kasan_poison(FAR const void *addr, size_t size)
{
  __asan_poison_memory_region(addr, size);
}

FAR void *kasan_unpoison(FAR const void *addr, size_t size)
{
  __asan_unpoison_memory_region(addr, size);
  return (FAR void *)addr;
}

bool kasan_bypass(bool state)
{
  return false;
}

void kasan_register(FAR void *addr, FAR size_t *size)
{
  kasan_poison(addr, *size);
}

void kasan_unregister(FAR void *addr)
{
}
