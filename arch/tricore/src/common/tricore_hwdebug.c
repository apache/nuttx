/****************************************************************************
 * arch/tricore/src/common/tricore_hwdebug.c
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

#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Core Special Function Register (CSFR) numbers used below.  These were
 * previously pulled in from the iLLD SFR headers; define them locally so
 * this file no longer depends on iLLD.
 */

#define TC_DBGSR                      0xfd00  /* Debug Status Register */
#define TC_DBGACT                     0xfd14  /* Debug Action Register */
#define TC_DMS                        0xfd40  /* Debug Monitor Start Addr */
#define TC_DCX                        0xfd44  /* Debug Context Save Addr */
#define TC_DBGTCR                     0xfd48  /* Debug Trap Control Register */
#define TC_DBGCFG                     0xfd4c  /* Debug Configuration Register */

/* Trigger event registers TR0..TR7: EVT at 0xf000 + n * 8, ADR at + 4 */

#define TC_TR0_EVT                    0xf000
#define TC_TR1_EVT                    0xf008
#define TC_TR2_EVT                    0xf010
#define TC_TR3_EVT                    0xf018
#define TC_TR4_EVT                    0xf020
#define TC_TR5_EVT                    0xf028
#define TC_TR6_EVT                    0xf030
#define TC_TR7_EVT                    0xf038

#define TC_TR0_ADR                    0xf004
#define TC_TR1_ADR                    0xf00c
#define TC_TR2_ADR                    0xf014
#define TC_TR3_ADR                    0xf01c
#define TC_TR4_ADR                    0xf024
#define TC_TR5_ADR                    0xf02c
#define TC_TR6_ADR                    0xf034
#define TC_TR7_ADR                    0xf03c

/* CBS_OEC (OCDS Enable Control) memory-mapped register address */

#ifdef CONFIG_ARCH_TC1V6
#  define CBS_OEC_ADDR                0xf0000478
#elif defined(CONFIG_ARCH_TC1V8)
#  define CBS_OEC_ADDR                0xfa180068
#endif

/* Trigger event register (TREVT) bit field offsets/masks.  The address
 * match/type bits share the same layout on TC1V6 and TC1V8; only the
 * enable field differs (see TREVT_CFG_REG_EN_* below).
 */

#define TREVT_BBM_OFF                 3       /* Break Before Make */
#define TREVT_TYP_OFF                 12      /* 0: watchpoint 1: breakpoint */
#define TREVT_AST_OFF                 27      /* Address store trigger */
#define TREVT_ALD_OFF                 28      /* Address load trigger */

/* Debug Status Register (DBGSR) event source field */

#define DBGSR_EVTSRC_OFF              8
#define DBGSR_EVTSRC_MSK              0x1f

/* Debug Configuration Register (DBGCFG) - TC1V8 only */

#define DBGCFG_EN_OFF                 15      /* Debug enable */

/* Watchpoint and breakpoint share a set of trigger events */

#define TREVT_DEBUG_NUM               8
#define TREVT_DEBUG_REGS              4
#define TREVT_DEBUG_MATCH_TRAPMODE    0x03
#define TREVT_DEBUG_EVTSRC_BASE       0x10

#define TREVT_DEBUG_BBM               (1 << TREVT_BBM_OFF)
#define TREVT_DEBUG_WP                (0 << TREVT_TYP_OFF)
#define TREVT_DEBUG_BP                (1 << TREVT_TYP_OFF)
#define TREVT_DEBUG_WP_AST            (1 << TREVT_AST_OFF)
#define TREVT_DEBUG_WP_ALD            (1 << TREVT_ALD_OFF)
#define TREVT_DEBUG_WP_ASTLD          (TREVT_DEBUG_WP_AST | TREVT_DEBUG_WP_ALD)

/* Register TREVT[2:0] have difference between tc3xx and tc4xx */

#ifdef CONFIG_ARCH_TC1V6
#  define TREVT_CFG_REG_EN_MASK       0x7     /* EVTA field mask */
#  define TREVT_CFG_REG_EN_VALUE      TREVT_DEBUG_MATCH_TRAPMODE
#elif defined(CONFIG_ARCH_TC1V8)
#  define TREVT_CFG_REG_EN_MASK       0x1     /* EN bit mask */
#  define TREVT_CFG_REG_EN_VALUE      0x01
#endif

#define TREVT_SET_CASE(reg, n, val) \
  case n: tricore_mtcr(TC_TR##n##_##reg, val); break;
#define TREVT_GET_CASE(reg, n, val) \
  case n: val = tricore_mfcr(TC_TR##n##_##reg); break;

#define TREVT_GET_CFG_REG(n)          TREVT_GET(EVT, n)
#define TREVT_SET_CFG_REG(n, val)     TREVT_SET(EVT, n, val)
#define TREVT_GET_ADDR_REG(n)         TREVT_GET(ADR, n)
#define TREVT_SET_ADDR_REG(n, val)    TREVT_SET(ADR, n, val)

#define TREVT_SET(reg, n, val)                    \
  ({                                              \
    switch (n)                                    \
    {                                             \
      TREVT_SET_CASE(reg, 0, val)                 \
      TREVT_SET_CASE(reg, 1, val)                 \
      TREVT_SET_CASE(reg, 2, val)                 \
      TREVT_SET_CASE(reg, 3, val)                 \
      TREVT_SET_CASE(reg, 4, val)                 \
      TREVT_SET_CASE(reg, 5, val)                 \
      TREVT_SET_CASE(reg, 6, val)                 \
      TREVT_SET_CASE(reg, 7, val)                 \
    }                                             \
  })

#define TREVT_GET(reg, n)                         \
  ({                                              \
    uint32_t _val = 0;                            \
    switch (n)                                    \
    {                                             \
      TREVT_GET_CASE(reg, 0, _val)                \
      TREVT_GET_CASE(reg, 1, _val)                \
      TREVT_GET_CASE(reg, 2, _val)                \
      TREVT_GET_CASE(reg, 3, _val)                \
      TREVT_GET_CASE(reg, 4, _val)                \
      TREVT_GET_CASE(reg, 5, _val)                \
      TREVT_GET_CASE(reg, 6, _val)                \
      TREVT_GET_CASE(reg, 7, _val)                \
    }                                             \
   _val;                                          \
  })

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct tricore_debugpoint_s
{
  int type;
  void *addr;
  size_t size;
  debug_callback_t callback;
  void *arg;
};

/* The debug context save area pointed to by DCX must be aligned to a full
 * context (upper + lower CSA = 32 registers).  This matches the value that
 * arch/irq.h exports as XCPTCONTEXT_SIZE, defined locally to avoid pulling
 * in the iLLD-tainted header.
 */

#define TRICORE_DBG_CTX_SIZE (sizeof(uintptr_t) * 32)

struct tricore_debug_s
{
  uintptr_t aligned_data(TRICORE_DBG_CTX_SIZE) dcx[TREVT_DEBUG_REGS];
  struct tricore_debugpoint_s dp[TREVT_DEBUG_NUM];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct tricore_debug_s g_trevt_debug;

/****************************************************************************
 * Private Function
 ****************************************************************************/

static uint32_t tricore_convert_type(int type)
{
  switch (type)
    {
      case DEBUGPOINT_WATCHPOINT_RO:
        return TREVT_DEBUG_WP | TREVT_DEBUG_WP_ALD;

      case DEBUGPOINT_WATCHPOINT_WO:
        return TREVT_DEBUG_WP | TREVT_DEBUG_WP_AST;

      case DEBUGPOINT_WATCHPOINT_RW:
        return TREVT_DEBUG_WP | TREVT_DEBUG_WP_ASTLD;

      case DEBUGPOINT_BREAKPOINT:
      case DEBUGPOINT_STEPPOINT:
      default:
        return TREVT_DEBUG_BP;
    }
}

/****************************************************************************
 * Name: tricore_trevt_add
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
 ****************************************************************************/

static int tricore_trevt_add(int type, uint32_t addr, size_t size)
{
  int i;

  for (i = 0; i < TREVT_DEBUG_NUM; i++)
    {
      if (!(TREVT_GET_CFG_REG(i) & TREVT_CFG_REG_EN_MASK))
        {
          TREVT_SET_ADDR_REG(i, addr);
          TREVT_SET_CFG_REG(i, tricore_convert_type(type) |
                               TREVT_CFG_REG_EN_VALUE);
          return i;
        }
    }

  return -ENOSPC;
}

/****************************************************************************
 * Name: tricore_trevt_remove
 *
 * Description:
 *   Remove a trevt on the address.
 *
 * Input Parameters:
 *   addr - The address to be debugged.
 *
 * Returned Value:
 *  Index of trevt on success; a negated errno value on failure
 *
 ****************************************************************************/

static int tricore_trevt_remove(uint32_t addr)
{
  int i;

  for (i = 0; i < TREVT_DEBUG_NUM; i++)
    {
      if (TREVT_GET_ADDR_REG(i) == addr)
        {
          TREVT_SET_CFG_REG(i, 0);
          TREVT_SET_ADDR_REG(i, 0);
          return i;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: tricore_dbgmonitor
 *
 * Description:
 *   This is Debug Monitor exception handler.  This function is entered when
 *   the processor enters debug mode.  The debug monitor handler will handle
 *   debug events, and resume execution.
 *
 ****************************************************************************/

static void tricore_dbgmonitor(void)
{
  __asm__ __volatile__ (
    "svlcx\n\t"
    "call  tricore_trevt_match\n\t"
    "rslcx\n\t"
    "rfm\n\t"
  );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_trevt_match
 ****************************************************************************/

void tricore_trevt_match(void)
{
  struct tricore_debugpoint_s *dp;
  int evtsrc;

  dp = g_trevt_debug.dp;

  evtsrc = ((tricore_mfcr(TC_DBGSR) >> DBGSR_EVTSRC_OFF) &
            DBGSR_EVTSRC_MSK) - TREVT_DEBUG_EVTSRC_BASE;

  dp[evtsrc].callback(dp[evtsrc].type, dp[evtsrc].addr,
                      dp[evtsrc].size, dp[evtsrc].arg);
}

/****************************************************************************
 * Name: tricore_init_dbgmonitor
 *
 * Description:
 *   This function init the debug monitor exception.
 *
 ****************************************************************************/

int tricore_init_dbgmonitor(void)
{
  /* DBGTCR: DTA (bit 0) = 0, all other fields cleared */

  uint32_t dbgtcr = 0;
#ifdef CONFIG_ARCH_TC1V8
  /* DBGCFG: EN (bit 15) = 1, TC (bit 0) = 0, TCP (bit 1) = 0.
   * DBGACT: EVTA (bits 2:0) = trap mode.
   */

  uint32_t dbgcfg = 1 << DBGCFG_EN_OFF;
  uint32_t dbgact = TREVT_DEBUG_MATCH_TRAPMODE;
#endif

  if (!(tricore_mfcr(TC_DBGSR) & 0x1))
    {
      /* Unlock sequence for the OCDS Enable Control register */

      putreg32(0xa1, CBS_OEC_ADDR);
      putreg32(0x5e, CBS_OEC_ADDR);
      putreg32(0xa1, CBS_OEC_ADDR);
      putreg32(0x5e, CBS_OEC_ADDR);
    }

#ifdef CONFIG_ARCH_TC1V8
  /* Set debug configuration register */

  tricore_mtcr(TC_DBGCFG, dbgcfg);

  /* Set debug action configuration register */

  tricore_mtcr(TC_DBGACT, dbgact);
#endif

  /* Set debug trap control register */

  tricore_mtcr(TC_DBGTCR, dbgtcr);

  /* Set trevt trap handler */

  tricore_mtcr(TC_DMS, (uintptr_t)tricore_dbgmonitor);

  /* Set dcx register */

  tricore_mtcr(TC_DCX, (uintptr_t)g_trevt_debug.dcx);

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
  struct tricore_debugpoint_s *dp;
  int ret;

  ret = tricore_trevt_add(type, (uint32_t)addr, size);

  if (ret < 0)
    {
      return ret;
    }

  dp = g_trevt_debug.dp;
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
  struct tricore_debugpoint_s *dp;
  int ret;

  ret = tricore_trevt_remove((uintptr_t)addr);

  if (ret < 0)
    {
      return ret;
    }

  dp = g_trevt_debug.dp;
  dp[ret].type = 0;
  dp[ret].addr = 0;
  dp[ret].size = 0;
  dp[ret].callback = NULL;
  dp[ret].arg = NULL;

  return OK;
}
