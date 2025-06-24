/****************************************************************************
 * arch/arm64/src/common/arm64_hwdebug.c
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
#include <nuttx/mm/kasan.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include "arm64_fatal.h"
#include "arm64_hwdebug.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARM64_DBGBWCR_VAL(type, len) \
  (arm64_convert_type(type) | arm64_convert_size(len) | \
   ARM64_DBGBCR_PAC_ALL | ARM64_DBGBWCR_E)

#define ARM64_MASK_ADDR(addr) \
  ((uint64_t)kasan_clear_tag((void *)addr) & ~0x3)

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct arm64_debugpoint_s
{
  int type;
  void *addr;
  size_t size;
  debug_callback_t callback;
  void *arg;
};

struct arm64_debug_s
{
  /* Breakpoint currently in use for each BRP, WRP */

  struct arm64_debugpoint_s brps[ID_AA64DFR0_MAX_BRPS];
  struct arm64_debugpoint_s wrps[ID_AA64DFR0_MAX_WRPS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct arm64_debug_s g_arm64_debug[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Function
 ****************************************************************************/

static uint32_t arm64_convert_type(int type)
{
  switch (type)
    {
      case DEBUGPOINT_WATCHPOINT_RO:
        return ARM64_DBGBWCR_LSC_LOAD << ARM64_DBGBWCR_LSC_OFFSET;

      case DEBUGPOINT_WATCHPOINT_WO:
        return ARM64_DBGBWCR_LSC_STORE << ARM64_DBGBWCR_LSC_OFFSET;

      case DEBUGPOINT_WATCHPOINT_RW:
        return (ARM64_DBGBWCR_LSC_LOAD | ARM64_DBGBWCR_LSC_STORE)
               << ARM64_DBGBWCR_LSC_OFFSET;

      case DEBUGPOINT_BREAKPOINT:
      case DEBUGPOINT_STEPPOINT:
      default:
        return ARM64_DBGBWCR_LSC_EXECUTE << ARM64_DBGBWCR_LSC_OFFSET;
    }
}

static uint32_t arm64_convert_size(size_t len)
{
  switch (len)
    {
      case 1:
        return ARM64_DBGBWCR_BAS_LEN_1 << ARM64_DBGBWCR_BAS_OFFSET;
      case 2:
        return ARM64_DBGBWCR_BAS_LEN_2 << ARM64_DBGBWCR_BAS_OFFSET;
      case 3:
        return ARM64_DBGBWCR_BAS_LEN_3 << ARM64_DBGBWCR_BAS_OFFSET;
      case 4:
        return ARM64_DBGBWCR_BAS_LEN_4 << ARM64_DBGBWCR_BAS_OFFSET;
      case 5:
        return ARM64_DBGBWCR_BAS_LEN_5 << ARM64_DBGBWCR_BAS_OFFSET;
      case 6:
        return ARM64_DBGBWCR_BAS_LEN_6 << ARM64_DBGBWCR_BAS_OFFSET;
      case 7:
        return ARM64_DBGBWCR_BAS_LEN_7 << ARM64_DBGBWCR_BAS_OFFSET;
      case 8:
      default:
        return ARM64_DBGBWCR_BAS_LEN_8 << ARM64_DBGBWCR_BAS_OFFSET;
    }
}

/* Determine number of usable WRPs available. */

static int arm64_get_num_wrps(void)
{
  return (read_sysreg(id_aa64dfr0_el1) >> ID_AA64DFR0_EL1_WRPS_OFFSET)
         & ID_AA64DFR0_EL1_WRPS_MASK;
}

/* Determine number of usable BRPs available. */

static int arm64_get_num_brps(void)
{
  return (read_sysreg(id_aa64dfr0_el1) >> ID_AA64DFR0_EL1_BRPS_OFFSET)
         & ID_AA64DFR0_EL1_BRPS_MASK;
}

/****************************************************************************
 * Name: arm64_watchpoint_add
 *
 * Description:
 *   Add a watchpoint on the address.
 *
 * Input Parameters:
 *  type - The type of the watchpoint
 *  addr - The address to be watched
 *  size - The size of the address to be watched
 *
 * Returned Value:
 *  Index in wprs array on success; a negated errno value on failure
 *
 * Notes:
 *  The size of the watchpoint is determined by the hardware.
 *
 ****************************************************************************/

static int arm64_watchpoint_add(int type, uint64_t addr, size_t size)
{
  int num = arm64_get_num_wrps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (!(ARM64_DBG_GETN(wcr, i) & ARM64_DBGBWCR_E))
        {
          ARM64_DBG_SETN(wvr, i, ARM64_MASK_ADDR(addr));
          ARM64_DBG_SETN(wcr, i, ARM64_DBGBWCR_VAL(type, size));
          return i;
        }
    }

  return -ENOSPC;
}

/****************************************************************************
 * Name: arm64_watchpoint_remove
 *
 * Description:
 *   Remove a watchpoint on the address.
 *
 * Input Parameters:
 *   addr - The address to be watched.
 *
 * Returned Value:
 *  Index in wprs array on success; a negated errno value on failure
 *
 ****************************************************************************/

static int arm64_watchpoint_remove(uint64_t addr)
{
  int num = arm64_get_num_wrps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (ARM64_DBG_GETN(wvr, i) == ARM64_MASK_ADDR(addr))
        {
          ARM64_DBG_SETN(wcr, i, 0);
          return i;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: arm64_breakpoint_add
 *
 * Description:
 *   Add a breakpoint on addr.
 *
 * Input Parameters:
 *  addr - The address to break.
 *
 * Returned Value:
 *  Index in bprs array on success; a negated errno value on failure
 *
 * Notes:
 *  1. If breakpoint is already set, it will do nothing.
 *  2. If all comparators are in use, it will return -1.
 *  3. When the breakpoint trigger, if enable monitor exception already ,
 *     will cause a debug monitor exception, oaddr=0x4020392ctherwise will
 *     cause a hard fault.
 *
 ****************************************************************************/

static int arm64_breakpoint_add(uintptr_t addr)
{
  int num = arm64_get_num_brps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (!(ARM64_DBG_GETN(bcr, i) & ARM64_DBGBWCR_E))
        {
          ARM64_DBG_SETN(bvr, i, ARM64_MASK_ADDR(addr));
          ARM64_DBG_SETN(bcr, i,
                         ARM64_DBGBWCR_VAL(DEBUGPOINT_BREAKPOINT, 8));
          return i;
        }
    }

  return -ENOSPC;
}

/****************************************************************************
 * Name: arm64_breakpoint_remove
 *
 * Description:
 *   Remove a breakpoint on addr.
 *
 * Input Parameters:
 *  addr - The address to remove.
 *
 * Returned Value:
 *  Index in bprs array on success; a negated errno value on failure
 *
 ****************************************************************************/

static int arm64_breakpoint_remove(uintptr_t addr)
{
  int num = arm64_get_num_brps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (ARM64_DBG_GETN(bvr, i) == ARM64_MASK_ADDR(addr))
        {
          ARM64_DBG_SETN(bcr, i, 0);
          return i;
        }
    }

  return -ENOENT;
}

static int arm64_watchpoint_match(uint64_t *regs, uint64_t far, uint64_t esr)
{
  struct arm64_debugpoint_s *dp = g_arm64_debug[this_cpu()].wrps;
  int num = arm64_get_num_wrps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (ARM64_MASK_ADDR(dp[i].addr) == ARM64_MASK_ADDR(far))
        {
          dp[i].callback(dp[i].type, dp[i].addr,
                         dp[i].size, dp[i].arg);
          return OK;
        }
    }

  return EFAULT;
}

static int arm64_breakpoint_match(uint64_t *regs, uint64_t far, uint64_t esr)
{
  struct arm64_debugpoint_s *dp = g_arm64_debug[this_cpu()].brps;
  int num = arm64_get_num_brps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (ARM64_MASK_ADDR(dp[i].addr) == ARM64_MASK_ADDR(up_getusrpc(regs)))
        {
          dp[i].callback(dp[i].type, dp[i].addr,
                         dp[i].size, dp[i].arg);
          return OK;
        }
    }

  return EFAULT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_enable_dbgmonitor
 *
 * Description:
 *   This function enables the debug monitor exception.
 *
 ****************************************************************************/

int arm64_enable_dbgmonitor(void)
{
  uint32_t mdscr;

  /* Determine how many BRPs/WRPs are available.
   * And work out the maximum supported watchpoint length.
   */

  binfo("found %d breakpoint and %d watchpoint registers.\n",
        arm64_get_num_brps(), arm64_get_num_wrps());

  mdscr = read_sysreg(mdscr_el1);
  mdscr |= ARM64_MDSCR_EL1_MDE | ARM64_MDSCR_EL1_KDE;
  write_sysreg(mdscr, mdscr_el1);

  arm64_register_debug_hook(DBG_ESR_EVT_HWBP, arm64_breakpoint_match);
  arm64_register_debug_hook(DBG_ESR_EVT_HWWP, arm64_watchpoint_match);
  return OK;
}

/****************************************************************************
 * Name: up_debugpoint_add
 *
 * Description:
 *   Add a debugpoint.
 *
 * Input Parameters:
 *   type     - The debugpoint type. optional value:
 *              DEBUGPOINT_WATCHPOINT_RO - Read only watchpoint.
 *              DEBUGPOINT_WATCHPOINT_WO - Write only watchpoint.
 *              DEBUGPOINT_WATCHPOINT_RW - Read and write watchpoint.
 *              DEBUGPOINT_BREAKPOINT    - Breakpoint.
 *              DEBUGPOINT_STEPPOINT     - Single step.
 *   addr     - The address to be debugged.
 *   size     - The watchpoint size. only for watchpoint.
 *   callback - The callback function when debugpoint triggered.
 *              if NULL, the debugpoint will be removed.
 *   arg      - The argument of callback function.
 *
 * Returned Value:
 *  Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int up_debugpoint_add(int type, void *addr, size_t size,
                      debug_callback_t callback, void *arg)
{
  struct arm64_debugpoint_s *dp;
  int cpu = this_cpu();
  int ret = -EINVAL;

  if (type == DEBUGPOINT_BREAKPOINT)
    {
      ret = arm64_breakpoint_add((uintptr_t)addr);
      dp = g_arm64_debug[cpu].brps;
    }
  else if (type == DEBUGPOINT_WATCHPOINT_RO ||
           type == DEBUGPOINT_WATCHPOINT_WO ||
           type == DEBUGPOINT_WATCHPOINT_RW)
    {
      ret = arm64_watchpoint_add(type, (uintptr_t)addr, size);
      dp = g_arm64_debug[cpu].wrps;
    }

  if (ret < 0)
    {
      return ret;
    }

  dp[ret].type = type;
  dp[ret].addr = addr;
  dp[ret].size = size;
  dp[ret].callback = callback;
  dp[ret].arg = arg;

  return OK;
}

/****************************************************************************
 * Name: up_debugpoint_remove
 *
 * Description:
 *   Remove a debugpoint.
 *
 * Input Parameters:
 *   type     - The debugpoint type. optional value:
 *              DEBUGPOINT_WATCHPOINT_RO - Read only watchpoint.
 *              DEBUGPOINT_WATCHPOINT_WO - Write only watchpoint.
 *              DEBUGPOINT_WATCHPOINT_RW - Read and write watchpoint.
 *              DEBUGPOINT_BREAKPOINT    - Breakpoint.
 *              DEBUGPOINT_STEPPOINT     - Single step.
 *   addr     - The address to be debugged.
 *   size     - The watchpoint size. only for watchpoint.
 *
 * Returned Value:
 *  Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int up_debugpoint_remove(int type, void *addr, size_t size)
{
  struct arm64_debugpoint_s *dp;
  int cpu = this_cpu();
  int ret = -EINVAL;

  if (type == DEBUGPOINT_BREAKPOINT)
    {
      ret = arm64_breakpoint_remove((uintptr_t)addr);
      dp = g_arm64_debug[cpu].brps;
    }
  else if (type == DEBUGPOINT_WATCHPOINT_RO ||
           type == DEBUGPOINT_WATCHPOINT_WO ||
           type == DEBUGPOINT_WATCHPOINT_RW)
    {
      ret = arm64_watchpoint_remove((uintptr_t)addr);
      dp = g_arm64_debug[cpu].wrps;
    }

  if (ret < 0)
    {
      return ret;
    }

  dp[ret].type = 0;
  dp[ret].addr = 0;
  dp[ret].size = 0;
  dp[ret].callback = NULL;
  dp[ret].arg = NULL;

  return OK;
}
