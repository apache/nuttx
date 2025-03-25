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

#include <assert.h>
#include <debug.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct xtensa_debug_trigger
{
  int type;
  void *address;
  size_t size;
  debug_callback_t callback;
  void *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct xtensa_debug_trigger g_code_trigger_map[XCHAL_NUM_IBREAK];
static struct xtensa_debug_trigger g_data_trigger_map[XCHAL_NUM_DBREAK];
static struct xtensa_debug_trigger g_singlestep_trigger;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_enable_ibreak
 *
 * Description:
 *   Enable the instruction breakpoint.
 *
 ****************************************************************************/

static void xtensa_enable_ibreak(int index, uintptr_t address)
{
  uint32_t ibreakenable = 0;

  DEBUGASSERT(index < XCHAL_NUM_IBREAK && XCHAL_NUM_IBREAK <= 2);

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

/****************************************************************************
 * Name: xtensa_disable_ibreak
 *
 * Description:
 *   Disable the instruction breakpoint.
 *
 ****************************************************************************/

static void xtensa_disable_ibreak(int index)
{
  uint32_t ibreakenable = 0;

  DEBUGASSERT(index < XCHAL_NUM_IBREAK);

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

/****************************************************************************
 * Name: xtensa_enable_singlestep
 *
 * Description:
 *   Enable or disable the single step mode.
 *
 ****************************************************************************/

static void xtensa_enable_singlestep(bool enable)
{
  uint32_t icountlevel;
  uint32_t icount;

  if (enable)
    {
      icountlevel = XCHAL_EXCM_LEVEL;
      icount = 0xfffffffe;
    }
  else
    {
      icountlevel = 0;
      icount = 0;
    }

  __asm__ __volatile__
  (
    "wsr %0, ICOUNT\n"
    "wsr %1, ICOUNTLEVEL\n"
    "isync\n"
    :
    : "r"(icount), "r"(icountlevel)
  );
}

/****************************************************************************
 * Name: xtensa_enable_dbreak
 *
 * Description:
 *   Enable the data breakpoint.
 *
 ****************************************************************************/

static void xtensa_enable_dbreak(int index, int type, uintptr_t address,
                                 size_t size)
{
  uint32_t dbreakc = 0;

  DEBUGASSERT(index < XCHAL_NUM_DBREAK && XCHAL_NUM_DBREAK <= 2);

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

/****************************************************************************
 * Name: xtensa_disable_dbreak
 *
 * Description:
 *   Disable the data breakpoint.
 *
 ****************************************************************************/

static void xtensa_disable_dbreak(int index)
{
  uint32_t dbreakc = 0;

  DEBUGASSERT(index < XCHAL_NUM_DBREAK && XCHAL_NUM_DBREAK <= 2);

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

/****************************************************************************
 * Name: xtensa_find_code_slot
 *
 * Description:
 *   Find the trigger slot by type, address and size, return the index of the
 *   slot or -ENOENT if not found.
 *
 ****************************************************************************/

static int xtensa_find_code_slot(int type, void *address, size_t size)
{
  int i;

  for (i = 0; i < XCHAL_NUM_IBREAK; i++)
    {
      if (g_code_trigger_map[i].type == type &&
          g_code_trigger_map[i].address == address &&
          g_code_trigger_map[i].size == size)
        {
          return i;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: xtensa_find_data_slot
 *
 * Description:
 *   Find the trigger slot by type, address and size, return the index of the
 *   slot or -ENOENT if not found.
 *
 ****************************************************************************/

static int xtensa_find_data_slot(int type, void *address, size_t size)
{
  int i;

  for (i = 0; i < XCHAL_NUM_DBREAK; i++)
    {
      if (g_data_trigger_map[i].type == type &&
          g_data_trigger_map[i].address == address &&
          g_data_trigger_map[i].size == size)
        {
          return i;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: xtensa_breakpoint_handler
 *
 * Description:
 *   This function is called when a breakpoint is triggered.
 *
 ****************************************************************************/

static void xtensa_breakpoint_handler(uint32_t pc)
{
  int i;

  for (i = 0; i < XCHAL_NUM_IBREAK; i++)
    {
      if (g_code_trigger_map[i].address == (void *)pc &&
          g_code_trigger_map[i].callback != NULL)
        {
          g_code_trigger_map[i].callback(g_code_trigger_map[i].type,
                                         g_code_trigger_map[i].address,
                                         g_code_trigger_map[i].size,
                                         g_code_trigger_map[i].arg);
        }
    }
}

/****************************************************************************
 * Name: xtensa_singlestep_handler
 *
 * Description:
 *   This function is called when a singlestep is triggered.
 *
 ****************************************************************************/

static void xtensa_singlestep_handler(void)
{
  if (g_singlestep_trigger.callback != NULL)
    {
      g_singlestep_trigger.callback(g_singlestep_trigger.type,
                                    g_singlestep_trigger.address,
                                    g_singlestep_trigger.size,
                                    g_singlestep_trigger.arg);
    }
}

/****************************************************************************
 * Name: xtensa_watchpoint_handler
 *
 * Description:
 *   This function is called when a watchpoint is triggered.
 *
 ****************************************************************************/

static void xtensa_watchpoint_handler(uint32_t dbnum)
{
  int i = dbnum;

  if (g_data_trigger_map[i].callback != NULL)
    {
      g_data_trigger_map[i].callback(g_data_trigger_map[i].type,
                                     g_data_trigger_map[i].address,
                                     g_data_trigger_map[i].size,
                                     g_data_trigger_map[i].arg);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_debugpoint_add
 *
 * Description:
 *   Add a debugpoint.
 *
 ****************************************************************************/

int up_debugpoint_add(int type, void *addr, size_t size,
                      debug_callback_t callback, void *arg)
{
  int slot;
  int ret = OK;

  switch (type)
    {
      case DEBUGPOINT_BREAKPOINT:
        slot = xtensa_find_code_slot(0, 0, 0);
        if (slot < 0)
          {
            return -ENOENT;
          }

        xtensa_enable_ibreak(slot, (uintptr_t)addr);
        g_code_trigger_map[slot].type     = type;
        g_code_trigger_map[slot].address  = addr;
        g_code_trigger_map[slot].size     = size;
        g_code_trigger_map[slot].callback = callback;
        g_code_trigger_map[slot].arg      = arg;
        break;
      case DEBUGPOINT_WATCHPOINT_RO:
      case DEBUGPOINT_WATCHPOINT_WO:
      case DEBUGPOINT_WATCHPOINT_RW:
        slot = xtensa_find_data_slot(0, 0, 0);
        if (slot < 0)
          {
            return -ENOENT;
          }

        xtensa_enable_dbreak(slot, type, (uintptr_t)addr, size);
        g_data_trigger_map[slot].type     = type;
        g_data_trigger_map[slot].address  = addr;
        g_data_trigger_map[slot].size     = size;
        g_data_trigger_map[slot].callback = callback;
        g_data_trigger_map[slot].arg      = arg;
        break;
      case DEBUGPOINT_STEPPOINT:
        xtensa_enable_singlestep(true);
        g_singlestep_trigger.type     = type;
        g_singlestep_trigger.address  = addr;
        g_singlestep_trigger.size     = size;
        g_singlestep_trigger.callback = callback;
        g_singlestep_trigger.arg      = arg;
        break;
      default:
        return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: up_debugpoint_remove
 *
 * Description:
 *   Remove a debugpoint.
 *
 ****************************************************************************/

int up_debugpoint_remove(int type, void *addr, size_t size)
{
  int slot;

  switch (type)
    {
      case DEBUGPOINT_BREAKPOINT:
        slot = xtensa_find_code_slot(type, addr, size);
        if (slot < 0)
          {
            return slot;
          }

        xtensa_disable_ibreak(slot);
        memset(&g_code_trigger_map[slot], 0,
               sizeof(struct xtensa_debug_trigger));
        break;
      case DEBUGPOINT_WATCHPOINT_RO:
      case DEBUGPOINT_WATCHPOINT_WO:
      case DEBUGPOINT_WATCHPOINT_RW:
        slot = xtensa_find_data_slot(type, addr, size);
        if (slot < 0)
          {
            return slot;
          }

        xtensa_disable_dbreak(slot);
        memset(&g_data_trigger_map[slot], 0,
               sizeof(struct xtensa_debug_trigger));
        break;
      case DEBUGPOINT_STEPPOINT:
        xtensa_enable_singlestep(false);
        memset(&g_singlestep_trigger, 0,
               sizeof(struct xtensa_debug_trigger));
        break;
      default:
        return -EINVAL;
  }

  return 0;
}

/****************************************************************************
 * Name: xtensa_debug_handler
 *
 * Description:
 *   This is the debug exception handler.  It will be called when a debug
 *   exception occurs.  The debug exception handler will handle debug events,
 *   and resume execution.
 *
 ****************************************************************************/

uint32_t *xtensa_debug_handler(uint32_t *regs)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  bool irq = up_interrupt_context();
  uint32_t *saved_regs = NULL;
  uint32_t cause;
  uint32_t dbnum;

  if (!irq)
    {
      up_set_interrupt_context(true);
    }
  else
    {
      saved_regs = (*running_task)->xcp.regs;
    }

  (*running_task)->xcp.regs = regs;

  __asm__ __volatile__
  (
    "rsr %0, DEBUGCAUSE\n"
    : "=r"(cause)
  );

  if (cause & XCHAL_DEBUGCAUSE_IBREAK_MASK)
    {
      xtensa_breakpoint_handler(regs[REG_PC]);
    }
  else if (cause & XCHAL_DEBUGCAUSE_DBREAK_MASK)
    {
      /* The DBNUM field of the DEBUGCAUSE register records which of the
       * possible DBREAK registers raised the exception
       */

      dbnum = ((cause & 0x0f00) >> 8);
      xtensa_watchpoint_handler(dbnum);
    }
  else if (cause & XCHAL_DEBUGCAUSE_ICOUNT_MASK)
    {
      xtensa_singlestep_handler();
    }
  else
    {
      _alert("Unhandled debug cause 0x%x\n", cause);
    }

  if (!irq)
    {
      up_set_interrupt_context(false);
      (*running_task)->xcp.regs = NULL;
    }
  else
    {
      (*running_task)->xcp.regs = saved_regs;
    }

  return regs;
}
