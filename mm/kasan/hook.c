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

#include <nuttx/mm/kasan.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>

#include <assert.h>
#include <debug.h>
#include <execinfo.h>
#include <stdint.h>
#include <stdio.h>

#ifdef CONFIG_MM_KASAN_GLOBAL
#  include "global.c"
#else
#  define kasan_global_is_poisoned(addr, size) false
#endif

#ifdef CONFIG_MM_KASAN_GENERIC
#  include "generic.c"
#elif defined(CONFIG_MM_KASAN_SW_TAGS)
#  include "sw_tags.c"
#else
#  define kasan_is_poisoned(addr, size) false
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

#ifdef CONFIG_MM_KASAN_DISABLE_READ_PANIC
#  define MM_KASAN_DISABLE_READ_PANIC 1
#else
#  define MM_KASAN_DISABLE_READ_PANIC 0
#endif

#ifdef CONFIG_MM_KASAN_DISABLE_WRITE_PANIC
#  define MM_KASAN_DISABLE_WRITE_PANIC 1
#else
#  define MM_KASAN_DISABLE_WRITE_PANIC 0
#endif

#ifdef CONFIG_MM_KASAN_WATCHPOINT
#  define MM_KASAN_WATCHPOINT CONFIG_MM_KASAN_WATCHPOINT
#else
#  define MM_KASAN_WATCHPOINT 0
#endif

#define KASAN_INIT_VALUE 0xcafe

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kasan_watchpoint_s
{
  FAR void *addr;
  size_t size;
  int type;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if MM_KASAN_WATCHPOINT > 0
static struct kasan_watchpoint_s g_watchpoint[MM_KASAN_WATCHPOINT];
#endif

#ifdef CONFIG_MM_KASAN
static uint32_t g_region_init;
#endif

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
  irqstate_t flags;

  flags = enter_critical_section();
  _alert("kasan detected a %s access error, address at %p,"
         "size is %zu, return address: %p\n",
         is_write ? "write" : "read",
         addr, size, return_address);

  kasan_show_memory(addr, size, 80);

  if ((is_write && MM_KASAN_DISABLE_WRITE_PANIC) ||
      (!is_write && MM_KASAN_DISABLE_READ_PANIC))
    {
      dump_stack();
    }
  else
    {
      PANIC();
    }

  leave_critical_section(flags);
}

#if MM_KASAN_WATCHPOINT > 0
static void kasan_check_watchpoint(FAR const void *addr, size_t size,
                                   bool is_write,
                                   FAR void *return_address)
{
  int i;

  for (i = 0; i < MM_KASAN_WATCHPOINT; i++)
    {
      FAR struct kasan_watchpoint_s *watchpoint = &g_watchpoint[i];

      if (predict_false(watchpoint->type == DEBUGPOINT_NONE))
        {
          break;
        }

      if (addr + size <= watchpoint->addr ||
          addr > watchpoint->addr + watchpoint->size)
        {
          continue;
        }

      if ((is_write && (watchpoint->type & DEBUGPOINT_WATCHPOINT_WO)) ||
          (!is_write && (watchpoint->type & DEBUGPOINT_WATCHPOINT_RO)))
        {
          kasan_report(addr, size, is_write, return_address);
        }
    }
}
#endif

static inline void kasan_check_report(FAR const void *addr, size_t size,
                                      bool is_write,
                                      FAR void *return_address)
{
#ifdef CONFIG_MM_KASAN
  if (predict_false(size == 0 || g_region_init != KASAN_INIT_VALUE))
    {
      return;
    }

#  ifndef CONFIG_MM_KASAN_DISABLE_NULL_POINTER_CHECK
  if (predict_false(addr == NULL))
    {
      kasan_report(addr, size, is_write, return_address);
    }
#  endif

#  ifndef CONFIG_MM_KASAN_NONE
  if (predict_false(kasan_is_poisoned(addr, size)))
    {
      kasan_report(addr, size, is_write, return_address);
    }
#  endif

#  if MM_KASAN_WATCHPOINT > 0
  kasan_check_watchpoint(addr, size, is_write, return_address);
#  endif
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN
void kasan_start(void)
{
  g_region_init = KASAN_INIT_VALUE;
}

void kasan_stop(void)
{
  g_region_init = 0;
}
#endif

/****************************************************************************
 * Name: kasan_debugpoint
 *
 * Description:
 *   Monitor the memory range for invalid access check
 *
 * Input Parameters:
 *   type - DEBUGPOINT_NONE         : remove
 *          DEBUGPOINT_WATCHPOINT_RO: read
 *          DEBUGPOINT_WATCHPOINT_WO: write
 *          DEBUGPOINT_WATCHPOINT_RW: read/write
 *   addr - range start address
 *   size - range size
 *
 * Returned Value:
 *   If the setting is successful, it returns 0, otherwise it
 *   returns an error code.
 *
 ****************************************************************************/

#if MM_KASAN_WATCHPOINT > 0
int kasan_debugpoint(int type, FAR void *addr, size_t size)
{
  FAR struct kasan_watchpoint_s *watchpoint;
  int i;
  int j;

  if (addr == NULL || size == 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < MM_KASAN_WATCHPOINT; i++)
    {
      watchpoint = &g_watchpoint[i];
      if (watchpoint->type == DEBUGPOINT_NONE || watchpoint->addr == addr)
        {
          if (type != DEBUGPOINT_NONE)
            {
              watchpoint->addr = addr;
              watchpoint->size = size;
              watchpoint->type = type;
              return 0;
            }

          for (j = MM_KASAN_WATCHPOINT - 1; j > i; j--)
            {
              if (g_watchpoint[j].type != DEBUGPOINT_NONE)
                {
                  watchpoint->addr = g_watchpoint[j].addr;
                  watchpoint->size = g_watchpoint[j].size;
                  watchpoint->type = g_watchpoint[j].type;
                  watchpoint = &g_watchpoint[j];
                  break;
                }
            }

          watchpoint->type = DEBUGPOINT_NONE;
          return 0;
        }
    }

  return -ENOMEM;
}
#endif

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

/* Generic KASan will instrument the following functions */

DEFINE_ASAN_LOAD_STORE(1)
DEFINE_ASAN_LOAD_STORE(2)
DEFINE_ASAN_LOAD_STORE(4)
DEFINE_ASAN_LOAD_STORE(8)
DEFINE_ASAN_LOAD_STORE(16)
