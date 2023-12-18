/****************************************************************************
 * arch/arm/src/armv8-m/arm_dbgmonitor.c
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

#include <arch/irq.h>

#include "nvic.h"
#include "fpb.h"
#include "dwt.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 4 watchpoint, 4 breakpoint, 1 single step */

#define ARM_DEBUG_MAX (4 + 4 + 1)

#define ARM_FPB_NUM() \
        (((getreg32(FPB_CTRL) & FPB_CTRL_NUM_CODE2_MASK) >> \
          FPB_CTRL_NUM_CODE2_SHIFT << FPB_CTRL_NUM_CODE1_SHIFT) | \
         ((getreg32(FPB_CTRL) & FPB_CTRL_NUM_CODE1_MASK) >> \
          FPB_CTRL_NUM_CODE1_SHIFT))

#define ARM_FPB_REVISION() \
        ((getreg32(FPB_CTRL) & FPB_CTRL_REV_MASK) >> FPB_CTRL_REV_SHIFT)

#define ARM_DWT_NUM() \
        ((getreg32(DWT_CTRL) & DWT_CTRL_NUMCOMP_MASK) >> \
         DWT_CTRL_NUMCOMP_SHIFT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct arm_debug_s
{
  int type;
  void *addr;
  size_t size;
  debug_callback_t callback;
  void *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct arm_debug_s g_arm_debug[ARM_DEBUG_MAX];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_fpb_init
 ****************************************************************************/

static void arm_fpb_init(void)
{
  uint32_t num = ARM_FPB_NUM();
  uint32_t i;

  for (i = 0; i < num; i++)
    {
      putreg32(0, FPB_COMP0 + i * 4);
    }

  modifyreg32(FPB_CTRL, 0, FPB_CTRL_ENABLE_MASK | FPB_CTRL_KEY_MASK);
}

/****************************************************************************
 * Name: arm_dwt_init
 ****************************************************************************/

static void arm_dwt_init(void)
{
  uint32_t num = ARM_DWT_NUM();
  uint32_t i;

  for (i = 0; i < num; i++)
    {
      putreg32(0, DWT_COMP0 + 16 * i);
      putreg32(0, DWT_MASK0 + 16 * i);
      putreg32(0, DWT_FUNCTION0 + 16 * i);
    }
}

/****************************************************************************
 * Name: arm_watchpoint_add
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
 *  Zero on success; a negated errno value on failure
 *
 * Notes:
 *  The size of the watchpoint is determined by the hardware.
 *
 ****************************************************************************/

static int arm_watchpoint_add(int type, uintptr_t addr, size_t size)
{
  uint32_t num = ARM_DWT_NUM();
  uint32_t i;

  for (i = 0; i < num; i++)
    {
      if (getreg32(DWT_COMP0 + 16 * i) == 0)
        {
          uint32_t fun;

          switch (type)
            {
              case DEBUGPOINT_WATCHPOINT_RO:
                fun = DWT_FUNCTION_WATCHPOINT_RO;
                break;
              case DEBUGPOINT_WATCHPOINT_WO:
                fun = DWT_FUNCTION_WATCHPOINT_WO;
                break;
              case DEBUGPOINT_WATCHPOINT_RW:
                fun = DWT_FUNCTION_WATCHPOINT_RW;
                break;
              default:
                return -EINVAL;
            }

        if (size > 4)
            {
              if (i + 1 >= num || i % 2 != 0 ||
                  getreg32(DWT_COMP0 + 16 * (i + 1)) != 0)
                {
                  continue;
                }

              putreg32(addr + size, DWT_COMP0 + 16 * (i + 1));
              putreg32(DWT_FUNCTION_WATCHPOINT_CO,
                       DWT_FUNCTION0 + 16 * (i + 1));
            }

          putreg32(addr, DWT_COMP0 + 16 * i);
          putreg32(fun, DWT_FUNCTION0 + 16 * i);
          putreg32(0, DWT_MASK0 + 16 * i);

          return 0;
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
 *   type - The type of the watchpoint.
 *   addr - The address to be watched.
 *   size - The size of the address to be watched.
 *
 * Returned Value:
 *  Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int arm_watchpoint_remove(int type, uintptr_t addr, size_t size)
{
  uint32_t num = ARM_DWT_NUM();
  uint32_t i;

  for (i = 0; i < num; i++)
    {
      uint32_t fun_type = (getreg32(DWT_FUNCTION0 + 16 * i) &
                          DWT_FUNCTION_FUNCTION_MASK) >>
                          DWT_FUNCTION_FUNCTION_SHIFT;

      switch (fun_type)
        {
          case DWT_FUNCTION_WATCHPOINT_RO:
            if (type == DEBUGPOINT_WATCHPOINT_RO)
              {
                break;
              }

            continue;
          case DWT_FUNCTION_WATCHPOINT_WO:
            if (type == DEBUGPOINT_WATCHPOINT_WO)
              {
                break;
              }

            continue;
          case DWT_FUNCTION_WATCHPOINT_RW:
            if (type == DEBUGPOINT_WATCHPOINT_RW)
              {
                break;
              }

            continue;

          default:
            continue;
        }

      if (addr != getreg32(DWT_COMP0 + 16 * i))
        {
          continue;
        }

      if (size > 4)
        {
          if (i + 1 >= num || i % 2 != 0 ||
              getreg32(DWT_COMP0 + 16 * (i + 1)) == 0)
            {
              continue;
            }

          putreg32(0, DWT_COMP0 + 16 * (i + 1));
          putreg32(0, DWT_FUNCTION0 + 16 * (i + 1));
          putreg32(0, DWT_MASK0 + 16 * (i + 1));
        }

      putreg32(0, DWT_COMP0 + 16 * i);
      putreg32(0, DWT_FUNCTION0 + 16 * i);
      putreg32(0, DWT_MASK0 + 16 * i);
      return 0;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: arm_watchpoint_match
 *
 * Description:
 *   This function will be called when watchpoint match.
 *
 ****************************************************************************/

static void arm_watchpoint_match(void)
{
  uint32_t num = ARM_DWT_NUM();
  uint32_t addr = 0;
  uint32_t i;
  int type = 0;

  for (i = 0; i < num; i++)
    {
      uint32_t fun = getreg32(DWT_FUNCTION0 + 16 * i);

      if (fun & DWT_FUNCTION_MATCHED_MASK)
        {
          uint32_t fun_type = (fun & DWT_FUNCTION_FUNCTION_MASK) >>
                              DWT_FUNCTION_FUNCTION_SHIFT;

          addr = getreg32(DWT_COMP0 + 16 * i);
          switch (fun_type)
            {
              case DWT_FUNCTION_WATCHPOINT_RO:
                type = DEBUGPOINT_WATCHPOINT_RO;
                break;
              case DWT_FUNCTION_WATCHPOINT_WO:
                type = DEBUGPOINT_WATCHPOINT_WO;
                break;
              case DWT_FUNCTION_WATCHPOINT_RW:
                type = DEBUGPOINT_WATCHPOINT_RW;
                break;

              default:
                continue;
            }

          break;
        }
    }

  for (i = 0; i < ARM_DEBUG_MAX; i++)
    {
      if (g_arm_debug[i].addr == (void *)addr &&
          g_arm_debug[i].type == type && g_arm_debug[i].callback)
        {
          g_arm_debug[i].callback(g_arm_debug[i].type,
                                  g_arm_debug[i].addr,
                                  g_arm_debug[i].size,
                                  g_arm_debug[i].arg);
          break;
        }
    }
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
 *  Zero on success; a negated errno value on failure
 *
 * Notes:
 *  1. If breakpoint is already set, it will do nothing.
 *  2. If all comparators are in use, it will return -ENOSPC.
 *  3. When the breakpoint trigger, if enable monitor exception already ,
 *     will cause a debug monitor exception, otherwise will cause
 *     a hard fault.
 *
 ****************************************************************************/

static int arm_breakpoint_add(uintptr_t addr)
{
  uint32_t revision = ARM_FPB_REVISION();
  uint32_t num = ARM_FPB_NUM();
  uint32_t fpb_comp;
  uint32_t i;

  if (revision == 0)
    {
      uint32_t replace = (addr & 0x2) == 0 ? 1 : 2;
      fpb_comp = (addr & ~0x3) | FPB_COMP0_ENABLE_MASK |
                 (replace << FPB_COMP0_REPLACE_SHIFT);
    }
  else
    {
      fpb_comp = addr | FPB_COMP0_ENABLE_MASK;
    }

  for (i = 0; i < num; i++)
    {
      uint32_t comp = getreg32(FPB_COMP0 + i * 4);

      if (comp == fpb_comp) /* Already set */
        {
          return 0;
        }
      else if (comp & FPB_COMP0_ENABLE_MASK) /* Comparators is in use */
        {
          continue;
        }
      else /* Find a free comparators */
        {
          putreg32(fpb_comp, FPB_COMP0 + i * 4);
          return 0;
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
 *  Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int arm_breakpoint_remove(uintptr_t addr)
{
  uint32_t revision = ARM_FPB_REVISION();
  uint32_t num = ARM_FPB_NUM();
  uint32_t fpb_comp;
  uint32_t i;

  if (revision == 0)
    {
      uint32_t replace = (addr & 0x2) == 0 ? 1 : 2;
      fpb_comp = (addr & ~0x3) | FPB_COMP0_ENABLE_MASK |
                (replace << FPB_COMP0_REPLACE_SHIFT);
    }
  else
    {
      fpb_comp = addr | FPB_COMP0_ENABLE_MASK;
    }

  for (i = 0; i < num; i++)
    {
      if (fpb_comp == getreg32(FPB_COMP0 + i * 4))
        {
          putreg32(0, FPB_COMP0 + i * 4);
          return 0;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: arm_breakpoint_match
 *
 * Description:
 *   This function will be called when breakpoint match.
 *
 ****************************************************************************/

static void arm_breakpoint_match(uintptr_t pc)
{
  int i;

  for (i = 0; i < ARM_DEBUG_MAX; i++)
    {
      if (g_arm_debug[i].type == DEBUGPOINT_BREAKPOINT &&
          g_arm_debug[i].addr == (void *)pc &&
          g_arm_debug[i].callback != NULL)
        {
          g_arm_debug[i].callback(g_arm_debug[i].type,
                                  g_arm_debug[i].addr,
                                  g_arm_debug[i].size,
                                  g_arm_debug[i].arg);
          break;
        }
    }
}

/****************************************************************************
 * Name: arm_steppoint
 *
 * Description:
 *   Enable/disable single step.
 *
 * Input Parameters:
 *  enable - True: enable single step; False: disable single step.
 *
 * Returned Value:
 *  Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int arm_steppoint(bool enable)
{
  if (enable)
    {
      modifyreg32(NVIC_DEMCR, 0, NVIC_DEMCR_MONSTEP);
    }
  else
    {
      modifyreg32(NVIC_DEMCR, NVIC_DEMCR_MONSTEP, 0);
    }

  return 0;
}

/****************************************************************************
 * Name: arm_steppoint_match
 *
 * Description:
 *   This function will be called when single step match.
 *
 ****************************************************************************/

static void arm_steppoint_match(void)
{
  int i;

  for (i = 0; i < ARM_DEBUG_MAX; i++)
    {
      if (g_arm_debug[i].type == DEBUGPOINT_STEPPOINT &&
          g_arm_debug[i].callback != NULL)
        {
          g_arm_debug[i].callback(g_arm_debug[i].type,
                                  g_arm_debug[i].addr,
                                  g_arm_debug[i].size,
                                  g_arm_debug[i].arg);
          break;
        }
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
  int ret = -EINVAL;
  int i;

  if (type == DEBUGPOINT_BREAKPOINT)
    {
      ret = arm_breakpoint_add((uintptr_t)addr);

      /* Thumb mode breakpoint address must be word-aligned */

      addr = (void *)((uintptr_t)addr & ~0x1);
    }
  else if (type == DEBUGPOINT_WATCHPOINT_RO ||
           type == DEBUGPOINT_WATCHPOINT_WO ||
           type == DEBUGPOINT_WATCHPOINT_RW)
    {
      ret = arm_watchpoint_add(type, (uintptr_t)addr, size);
    }
  else if (type == DEBUGPOINT_STEPPOINT)
    {
      ret = arm_steppoint(true);
    }

  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < ARM_DEBUG_MAX; i++)
    {
      if (g_arm_debug[i].type == DEBUGPOINT_NONE)
        {
          g_arm_debug[i].type = type;
          g_arm_debug[i].addr = addr;
          g_arm_debug[i].size = size;
          g_arm_debug[i].callback = callback;
          g_arm_debug[i].arg = arg;
          break;
        }
    }

  return ret;
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
  int ret = -EINVAL;
  int i;

  if (type == DEBUGPOINT_BREAKPOINT)
    {
      ret = arm_breakpoint_remove((uintptr_t)addr);

      /* Thumb mode breakpoint address must be word-aligned */

      addr = (void *)((uintptr_t)addr & ~0x1);
    }
  else if (type == DEBUGPOINT_WATCHPOINT_RO ||
           type == DEBUGPOINT_WATCHPOINT_WO ||
           type == DEBUGPOINT_WATCHPOINT_RW)
    {
      ret = arm_watchpoint_remove(type, (uintptr_t)addr, size);
    }
  else if (type == DEBUGPOINT_STEPPOINT)
    {
      ret = arm_steppoint(false);
    }

  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < ARM_DEBUG_MAX; i++)
    {
      if (g_arm_debug[i].type == type &&
          g_arm_debug[i].size == size &&
          g_arm_debug[i].addr == addr)
        {
          g_arm_debug[i].type = DEBUGPOINT_NONE;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: arm_enable_dbgmonitor
 *
 * Description:
 *   This function enables the debug monitor exception.
 *
 ****************************************************************************/

int arm_enable_dbgmonitor(void)
{
  if (getreg32(NVIC_DHCSR) & NVIC_DHCSR_C_DEBUGEN)
    {
      /* If already on debug mode(jtag/swo), just return */

      return OK;
    }

  arm_fpb_init();
  arm_dwt_init();
  modifyreg32(NVIC_DEMCR, 0, NVIC_DEMCR_MONEN | NVIC_DEMCR_TRCENA);

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
  uint32_t dfsr = getreg32(NVIC_DFAULTS);
  uint32_t *regs = (uint32_t *)context;

  if ((dfsr & NVIC_DFAULTS_HALTED) != 0)
    {
      arm_steppoint_match();
    }

  if ((dfsr & NVIC_DFAULTS_BKPT) != 0)
    {
      arm_breakpoint_match(regs[REG_PC]);
    }

  if ((dfsr & NVIC_DFAULTS_DWTTRAP) != 0)
    {
      arm_watchpoint_match();
    }

  return OK;
}
