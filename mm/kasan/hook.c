/****************************************************************************
 * mm/kasan/hook.c
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

#include <nuttx/irq.h>

#include <assert.h>
#include <debug.h>
#include <execinfo.h>
#include <stdint.h>
#include <stdio.h>

#include "kasan.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void kasan_show_memory(FAR const uint8_t *addr, size_t size,
                              size_t dumpsize)
{
  FAR const uint8_t *start = (FAR const uint8_t *)
                             (((uintptr_t)addr) & ~0xf) - dumpsize;
  FAR const uint8_t *end = start + 2 * dumpsize;
  FAR const uint8_t *p = start;
  char buffer[256];

  _alert("Shadow bytes around the buggy address:\n");
  for (p = start; p < end; p += 16)
    {
      int ret = sprintf(buffer, "  %p: ", p);
      int i;

      for (i = 0; i < 16; i++)
        {
          if (kasan_is_poisoned(p + i, 1))
            {
              if (p + i == addr)
                {
                  ret += sprintf(buffer + ret,
                                 "\b[\033[31m%02x\033[0m ", p[i]);
                }
              else if (p + i == addr + size - 1)
                {
                  ret += sprintf(buffer + ret, "\033[31m%02x\033[0m]", p[i]);
                }
              else
                {
                  ret += sprintf(buffer + ret, "\033[31m%02x\033[0m ", p[i]);
                }
            }
          else
            {
              ret += sprintf(buffer + ret, "\033[37m%02x\033[0m ", p[i]);
            }
        }

      _alert("%s\n", buffer);
    }
}

static void kasan_report(FAR const void *addr, size_t size,
                         bool is_write, FAR void *return_address)
{
  static int recursion;
  irqstate_t flags;

  flags = enter_critical_section();
  if (++recursion == 1)
    {
      _alert("kasan detected a %s access error, address at %p,"
             "size is %zu, return address: %p\n",
             is_write ? "write" : "read",
             addr, size, return_address);

      kasan_show_memory(addr, size, 80);
#ifndef CONFIG_MM_KASAN_DISABLE_PANIC
      PANIC();
#else
      dump_stack();
#endif
    }

  --recursion;
  leave_critical_section(flags);
}

static inline void kasan_check_report(FAR const void *addr, size_t size,
                                      bool is_write,
                                      FAR void *return_address)
{
  if (kasan_is_poisoned(addr, size))
    {
      kasan_report(addr, size, is_write, return_address);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void __asan_before_dynamic_init(FAR const void *module_name)
{
  /* Shut up compiler complaints */
}

void __asan_after_dynamic_init(void)
{
  /* Shut up compiler complaints */
}

void __asan_handle_no_return(void)
{
  /* Shut up compiler complaints */
}

void __asan_register_globals(void *ptr, size_t size)
{
  /* Shut up compiler complaints */
}

void __asan_unregister_globals(void *ptr, size_t size)
{
  /* Shut up compiler complaints */
}

void __sanitizer_annotate_contiguous_container(FAR const void *beg,
                                               FAR const void *end,
                                               FAR const void *old_mid,
                                               FAR const void *new_mid)
{
  /* Shut up compiler complaints */
}

void __asan_report_load_n_noabort(FAR void *addr, size_t size)
{
  kasan_report(addr, size, false, return_address(0));
}

void __asan_report_store_n_noabort(FAR void *addr, size_t size)
{
  kasan_report(addr, size, true, return_address(0));
}

void __asan_loadN_noabort(FAR void *addr, size_t size)
{
  kasan_check_report(addr, size, false, return_address(0));
}

void __asan_storeN_noabort(FAR void * addr, size_t size)
{
  kasan_check_report(addr, size, true, return_address(0));
}

void __asan_loadN(FAR void *addr, size_t size)
{
  kasan_check_report(addr, size, false, return_address(0));
}

void __asan_storeN(FAR void *addr, size_t size)
{
  kasan_check_report(addr, size, true, return_address(0));
}

#define DEFINE_ASAN_LOAD_STORE(size) \
  void __asan_report_load##size##_noabort(FAR void *addr) \
  { \
    kasan_report(addr, size, false, return_address(0)); \
  } \
  void __asan_report_store##size##_noabort(FAR void *addr) \
  { \
    kasan_report(addr, size, true, return_address(0)); \
  } \
  void __asan_load##size##_noabort(FAR void *addr) \
  { \
    kasan_check_report(addr, size, false, return_address(0)); \
  } \
  void __asan_store##size##_noabort(FAR void *addr) \
  { \
    kasan_check_report(addr, size, true, return_address(0)); \
  } \
  void __asan_load##size(FAR void *addr) \
  { \
    kasan_check_report(addr, size, false, return_address(0)); \
  } \
  void __asan_store##size(FAR void *addr) \
  { \
    kasan_check_report(addr, size, true, return_address(0)); \
  }

DEFINE_ASAN_LOAD_STORE(1)
DEFINE_ASAN_LOAD_STORE(2)
DEFINE_ASAN_LOAD_STORE(4)
DEFINE_ASAN_LOAD_STORE(8)
DEFINE_ASAN_LOAD_STORE(16)
