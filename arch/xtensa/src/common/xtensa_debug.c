/****************************************************************************
 * arch/xtensa/src/common/xtensa_debug.c
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

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRIGGER_TYPE_NONE  0
#define TRIGGER_TYPE_CODE  1
#define TRIGGER_TYPE_DATA  2

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct xtensa_debug_trigger
{
  int trigger_type;
  int index;
  int type;
  void *address;
  size_t size;
  debug_callback_t callback;
  void *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_trigger_count = 0;
static struct xtensa_debug_trigger *g_trigger_map;
static bool g_debug_initiliazed = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void xtensa_enable_ibreak(int index, uintptr_t address)
{
  uint32_t ibreakenable = 0;

  __asm__ __volatile__
  (
    "rsr %0, IBREAKENABLE\n"
    : "=r"(ibreakenable)
  );

  ibreakenable |= 1 << index;

  if (index == 0)
    {
      __asm__ __volatile__
      (
        "wsr %0, IBREAKA0\n"
        "wsr %1, IBREAKENABLE\n"
        "isync\n"
        :
        : "r"(address), "r"(ibreakenable)
      );
    }
  else if (index == 1)
    {
      __asm__ __volatile__
      (
        "wsr %0, IBREAKA1\n"
        "wsr %1, IBREAKENABLE\n"
        "isync\n"
        :
        : "r"(address), "r"(ibreakenable)
      );
    }
}

static void xtensa_disable_ibreak(int index)
{
  uint32_t ibreakenable = 0;

  __asm__ __volatile__
  (
    "rsr %0, IBREAKENABLE\n"
    : "=r"(ibreakenable)
  );

  ibreakenable &= ~(1 << index);

  __asm__ __volatile__
  (
    "wsr %0, IBREAKENABLE\n"
    "isync\n"
    :
    : "r"(ibreakenable)
  );
}

static void xtensa_enable_dbreak(int index, int type, uintptr_t address,
                                 size_t size)
{
  uint32_t dbreakc = 0;

  if (type == DEBUGPOINT_WATCHPOINT_RO)
    {
      dbreakc |= DBREAKC_LOADBREAK_MASK;
    }
  else if (type == DEBUGPOINT_WATCHPOINT_WO)
    {
      dbreakc |= DBREAKC_STOREBREAK_MASK;
    }
  else if (type == DEBUGPOINT_WATCHPOINT_RW)
    {
      dbreakc |= DBREAKC_LOADBREAK_MASK | DBREAKC_STOREBREAK_MASK;
    }

  dbreakc |= (~(size - 1)) & DBREAKC_MASK_MASK;

  if (index == 0)
    {
      __asm__ __volatile__
      (
        "wsr %0, DBREAKA0\n"
        "wsr %1, DBREAKC0\n"
        "dsync\n"
        :
        : "r"(address), "r"(dbreakc)
      );
    }
  else if (index == 1)
    {
      __asm__ __volatile__
      (
        "wsr %0, DBREAKA1\n"
        "wsr %1, DBREAKC1\n"
        "dsync\n"
        :
        : "r"(address), "r"(dbreakc)
      );
    }
}

static void xtensa_disable_dbreak(int index)
{
  uint32_t dbreakc = 0;

  if (index == 0)
    {
      __asm__ __volatile__
      (
        "wsr %0, DBREAKC0\n"
        "dsync\n"
        :
        : "r"(dbreakc)
      );
    }
  else if (index == 1)
    {
      __asm__ __volatile__
      (
        "wsr %0, DBREAKC1\n"
        "dsync\n"
        :
        : "r"(dbreakc)
      );
    }
}

static struct xtensa_debug_trigger *
xtensa_debug_find_slot(int trigger_type, int type, void *address,
                       size_t size)
{
  int i;
  struct xtensa_debug_trigger *trigger_map;

  for (i = 0; i < g_trigger_count; i++)
    {
      trigger_map = &g_trigger_map[i];
      if (trigger_map->trigger_type == trigger_type &&
          trigger_map->type == type &&
          trigger_map->address == address &&
          trigger_map->size == size)
        {
          return trigger_map;
        }
    }

  return NULL;
}

static int xtensa_debug_init(void)
{
  int i;
  int index;

  #if defined(XCHAL_NUM_DBREAK) && XCHAL_NUM_DBREAK > 0
  g_trigger_count += XCHAL_NUM_DBREAK;
  #endif
  #if defined(XCHAL_NUM_IBREAK) && XCHAL_NUM_IBREAK > 0
  g_trigger_count += XCHAL_NUM_IBREAK;
  #endif

  if (g_trigger_count == 0)
    {
      return -ENOENT;
    }

  g_trigger_map = kmm_zalloc(sizeof(struct xtensa_debug_trigger) *
                             g_trigger_count);

  if (!g_trigger_map)
    {
      return -ENOMEM;
    }

  i = 0;
  index = 0;

  #if defined(XCHAL_NUM_DBREAK) && XCHAL_NUM_DBREAK > 0
  for (; i < XCHAL_NUM_DBREAK; i++)
    {
      g_trigger_map[i].trigger_type = TRIGGER_TYPE_DATA;
      g_trigger_map[i].index = index++;
    }
  #endif

  index = 0;

  #if defined(XCHAL_NUM_IBREAK) && XCHAL_NUM_IBREAK > 0
  for (; i < g_trigger_count; i++)
    {
      g_trigger_map[i].trigger_type = TRIGGER_TYPE_CODE;
      g_trigger_map[i].index = index++;
    }
  #endif

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_debugpoint_add
 ****************************************************************************/

int up_debugpoint_add(int type, void *addr, size_t size,
                      debug_callback_t callback, void *arg)
{
  int ret = OK;
  struct xtensa_debug_trigger *map;

  if (g_debug_initiliazed == false)
    {
      ret = xtensa_debug_init();
      if (ret < 0)
        {
          return ret;
        }

      g_debug_initiliazed = true;
    }

  switch (type)
    {
      case DEBUGPOINT_BREAKPOINT:
        map = xtensa_debug_find_slot(TRIGGER_TYPE_CODE, 0, 0, 0);
        if (map == NULL)
          {
            return -ENOENT;
          }

        xtensa_enable_ibreak(map->index, (uintptr_t)addr);
        break;
      case DEBUGPOINT_WATCHPOINT_RO:
      case DEBUGPOINT_WATCHPOINT_WO:
      case DEBUGPOINT_WATCHPOINT_RW:
        map = xtensa_debug_find_slot(TRIGGER_TYPE_DATA, 0, 0, 0);
        if (map == NULL)
          {
            return -ENOENT;
          }

        xtensa_enable_dbreak(map->index, type, (uintptr_t)addr, size);
        break;
      default:
        return -EINVAL;
    }

  /* Register the callback and arg */

  map->type     = type;
  map->address  = addr;
  map->size     = size;
  map->callback = callback;
  map->arg      = arg;

  return 0;
}

/****************************************************************************
 * Name: up_debugpoint_remove
 ****************************************************************************/

int up_debugpoint_remove(int type, void *addr, size_t size)
{
  struct xtensa_debug_trigger *map;

  switch (type)
    {
      case DEBUGPOINT_BREAKPOINT:
        map = xtensa_debug_find_slot(TRIGGER_TYPE_CODE, type, addr, size);
        if (map == NULL)
          {
            return -ENOENT;
          }

        xtensa_disable_ibreak(map->index);
        break;
      case DEBUGPOINT_WATCHPOINT_RO:
      case DEBUGPOINT_WATCHPOINT_WO:
      case DEBUGPOINT_WATCHPOINT_RW:
        map = xtensa_debug_find_slot(TRIGGER_TYPE_DATA, type, addr, size);
        if (map == NULL)
          {
            return -ENOENT;
          }

        xtensa_disable_dbreak(map->index);
        break;
      default:
        return -EINVAL;
  }

  /* Clear the callback and arg */

  map->type     = 0;
  map->address  = 0;
  map->size     = 0;
  map->callback = 0;
  map->arg      = 0;

  return 0;
}
