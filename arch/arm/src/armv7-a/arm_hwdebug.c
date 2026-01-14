/****************************************************************************
 * arch/arm/src/armv7-a/arm_hwdebug.c
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

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <arch/armv7-a/cp14.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Watchpoint and breakpoint control register share bits */

#define CP14_DBGBWCR_E                       CP14_DBGBCR_E
#define CP14_DBGBWCR_PAC_USER                CP14_DBGBCR_PAC_USER
#define CP14_DBGBWCR_LSC_OFFSET              CP14_DBGBCR_LSC_OFFSET
#define CP14_DBGBWCR_LSC_EXECUTE             CP14_DBGBCR_LSC_EXECUTE
#define CP14_DBGBWCR_LSC_LOAD                CP14_DBGBCR_LSC_LOAD
#define CP14_DBGBWCR_LSC_STORE               CP14_DBGBCR_LSC_STORE
#define CP14_DBGBWCR_BAS_OFFSET              CP14_DBGBCR_BAS_OFFSET
#define CP14_DBGBWCR_BAS_LEN_1               CP14_DBGBCR_BAS_LEN_1
#define CP14_DBGBWCR_BAS_LEN_2               CP14_DBGBCR_BAS_LEN_2
#define CP14_DBGBWCR_BAS_LEN_4               CP14_DBGBCR_BAS_LEN_4
#define CP14_DBGBWCR_BAS_LEN_8               CP14_DBGBCR_BAS_LEN_8

/* Encode the bp control register field as a 32-bit value */

#define CP14_DBGBWCR_VAL(type, len) \
  (arm_convert_type(type) | arm_convert_size(len) | \
   CP14_DBGBCR_PAC_ALL | CP14_DBGBWCR_E)

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct arm_debugpoint_s
{
  int type;
  void *addr;
  size_t size;
  debug_callback_t callback;
  void *arg;
};

struct arm_debug_s
{
  /* Breakpoint currently in use for each BRP, WRP */

  struct arm_debugpoint_s brps[CP14_DBGDIDR_MAX_BRP];
  struct arm_debugpoint_s wrps[CP14_DBGDIDR_MAX_WRP];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct arm_debug_s g_arm_debug[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Function
 ****************************************************************************/

static uint32_t arm_convert_type(int type)
{
  switch (type)
    {
      case DEBUGPOINT_WATCHPOINT_RO:
        return CP14_DBGBWCR_LSC_LOAD << CP14_DBGBWCR_LSC_OFFSET;

      case DEBUGPOINT_WATCHPOINT_WO:
        return CP14_DBGBWCR_LSC_STORE << CP14_DBGBWCR_LSC_OFFSET;

      case DEBUGPOINT_WATCHPOINT_RW:
        return (CP14_DBGBWCR_LSC_LOAD | CP14_DBGBWCR_LSC_STORE)
               << CP14_DBGBWCR_LSC_OFFSET;

      case DEBUGPOINT_BREAKPOINT:
      case DEBUGPOINT_STEPPOINT:
      default:
        return CP14_DBGBWCR_LSC_EXECUTE << CP14_DBGBWCR_LSC_OFFSET;
    }
}

static uint32_t arm_convert_size(size_t len)
{
  switch (len)
    {
      case 1:
        return CP14_DBGBWCR_BAS_LEN_1 << CP14_DBGBWCR_BAS_OFFSET;

      case 2:
        return CP14_DBGBWCR_BAS_LEN_2 << CP14_DBGBWCR_BAS_OFFSET;

      case 4:
        return CP14_DBGBWCR_BAS_LEN_4 << CP14_DBGBWCR_BAS_OFFSET;

      case 8:
      default:
        return CP14_DBGBWCR_BAS_LEN_8 << CP14_DBGBWCR_BAS_OFFSET;
    }
}

/* Determine number of usable WRPs available. */

static int arm_get_num_wrps(void)
{
  return ((CP14_GET(DBGDIDR) >> CP14_DBGDIDR_WRPS_OFFSET)
          & CP14_DBGDIDR_WRPS_MASK) + 1;
}

/* Determine number of usable BRPs available. */

static int arm_get_num_brps(void)
{
  return ((CP14_GET(DBGDIDR) >> CP14_DBGDIDR_BRPS_OFFSET)
          & CP14_DBGDIDR_BRPS_MASK) + 1;
}

/****************************************************************************
 * Name: up_watchpoint_add
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

static int arm_watchpoint_add(int type, uint32_t addr, size_t size)
{
  int num = arm_get_num_wrps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (!(CP14_GETN(DBGWCR, i) & CP14_DBGBWCR_E))
        {
          CP14_SETN(DBGWVR, i, CP14_MASK_ADDR(addr));
          CP14_SETN(DBGWCR, i, CP14_DBGBWCR_VAL(type, size));
          return i;
        }
    }

  return -ENOSPC;
}

/****************************************************************************
 * Name: arm_watchpoint_remove
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

static int arm_watchpoint_remove(uint32_t addr)
{
  int num = arm_get_num_wrps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (CP14_GETN(DBGWVR, i) == CP14_MASK_ADDR(addr))
        {
          CP14_SETN(DBGWCR, i, 0);
          return i;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: arm_breakpoint_add
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

static int arm_breakpoint_add(uintptr_t addr)
{
  int num = arm_get_num_brps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (!(CP14_GETN(DBGBCR, i) & CP14_DBGBWCR_E))
        {
          CP14_SETN(DBGBVR, i, CP14_MASK_ADDR(addr));
          CP14_SETN(DBGBCR, i, CP14_DBGBWCR_VAL(DEBUGPOINT_BREAKPOINT, 8));
          return i;
        }
    }

  return -ENOSPC;
}

/****************************************************************************
 * Name: arm_breakpoint_remove
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

static int arm_breakpoint_remove(uintptr_t addr)
{
  int num = arm_get_num_brps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (CP14_GETN(DBGBVR, i) == CP14_MASK_ADDR(addr))
        {
          CP14_SETN(DBGBCR, i, 0);
          return i;
        }
    }

  return -ENOENT;
}

static void arm_watchpoint_match(uint32_t addr)
{
  struct arm_debugpoint_s *dp = g_arm_debug[this_cpu()].wrps;
  int num = arm_get_num_wrps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (CP14_MASK_ADDR(dp[i].addr) == CP14_MASK_ADDR(addr))
        {
          dp[i].callback(dp[i].type, dp[i].addr,
                         dp[i].size, dp[i].arg);
          break;
        }
    }
}

static void arm_breakpoint_match(uint32_t addr)
{
  struct arm_debugpoint_s *dp = g_arm_debug[this_cpu()].brps;
  int num = arm_get_num_brps();
  int i;

  for (i = 0; i < num; i++)
    {
      if (CP14_MASK_ADDR(dp[i].addr) == CP14_MASK_ADDR(addr))
        {
          dp[i].callback(dp[i].type, dp[i].addr,
                         dp[i].size, dp[i].arg);
          break;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_enable_dbgmonitor
 *
 * Description:
 *   This function enables the debug monitor exception.
 *
 ****************************************************************************/

int arm_enable_dbgmonitor(void)
{
  /* Determine how many BRPs/WRPs are available.
   * And work out the maximum supported watchpoint length.
   */

  binfo("found %d breakpoint and %d watchpoint registers.\n",
        arm_get_num_brps(), arm_get_num_wrps());

  /* If monitor mode is already enabled, just return. */

  if (CP14_GET(DBGDSCRINT) & CP14_DBGDSCRINT_MDBGEN)
    {
      return OK;
    }

  CP14_MOD(DBGDSCREXT, CP14_DBGDSCRINT_MDBGEN, CP14_DBGDSCRINT_MDBGEN);

  /* Check that the write made it through. */

  if (!(CP14_GET(DBGDSCRINT) & CP14_DBGDSCRINT_MDBGEN))
    {
      return -EPERM;
    }

  return OK;
}

/****************************************************************************
 * Name: arm_dbgmonitor
 *
 * Description:
 *   This is Debug Monitor exception handler.  This function is entered when
 *   the processor enters debug mode.  The debug monitor handler will handle
 *   debug events, and resume execution.
 *
 ****************************************************************************/

int arm_dbgmonitor(int irq, void *context, void *arg)
{
  switch (((CP14_GET(DBGDSCRINT)) >> CP14_DBGDSCRINT_MOE_OFFSET)
          & CP14_DBGDSCRINT_MOE_MASK)
    {
      case CP14_DBGDSCRINT_MOE_BREAKPOINT:
      case CP14_DBGDSCRINT_MOE_CFI_BREAKPOINT:
        arm_breakpoint_match((uintptr_t)context);
        break;
      case CP14_DBGDSCRINT_MOE_ASYNC_WATCHPOINT:
      case CP14_DBGDSCRINT_MOE_SYNC_WATCHPOINT:
        arm_watchpoint_match((uintptr_t)context);
        break;
    }

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
  struct arm_debugpoint_s *dp;
  int cpu = this_cpu();
  int ret = -EINVAL;

  if (type == DEBUGPOINT_BREAKPOINT)
    {
      ret = arm_breakpoint_add((uintptr_t)addr);
      dp = g_arm_debug[cpu].brps;
    }
  else if (type == DEBUGPOINT_WATCHPOINT_RO ||
           type == DEBUGPOINT_WATCHPOINT_WO ||
           type == DEBUGPOINT_WATCHPOINT_RW)
    {
      ret = arm_watchpoint_add(type, (uintptr_t)addr, size);
      dp = g_arm_debug[cpu].wrps;
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
  struct arm_debugpoint_s *dp;
  int cpu = this_cpu();
  int ret = -EINVAL;

  if (type == DEBUGPOINT_BREAKPOINT)
    {
      ret = arm_breakpoint_remove((uintptr_t)addr);
      dp = g_arm_debug[cpu].brps;
    }
  else if (type == DEBUGPOINT_WATCHPOINT_RO ||
           type == DEBUGPOINT_WATCHPOINT_WO ||
           type == DEBUGPOINT_WATCHPOINT_RW)
    {
      ret = arm_watchpoint_remove((uintptr_t)addr);
      dp = g_arm_debug[cpu].wrps;
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
