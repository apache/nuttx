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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Breakpoint */

#define ARM_BREAKPOINT_EXECUTE   0

/* Watchpoints */

#define ARM_BREAKPOINT_LOAD      1
#define ARM_BREAKPOINT_STORE     2

/* Privilege Levels */

#define ARM_BREAKPOINT_PRIV      1
#define ARM_BREAKPOINT_USER      2

/* Lengths */

#define ARM_BREAKPOINT_LEN_1     0x1
#define ARM_BREAKPOINT_LEN_2     0x3
#define ARM_BREAKPOINT_LEN_4     0xf
#define ARM_BREAKPOINT_LEN_8     0xff

/* Limits */

#define ARM_MAX_BRP              16
#define ARM_MAX_WRP              16

/* DSCR monitor/halting bits. */

#define ARM_DSCR_HDBGEN          (1 << 14)
#define ARM_DSCR_MDBGEN          (1 << 15)

/* opcode2 numbers for the co-processor instructions. */

#define ARM_OP2_BVR              4
#define ARM_OP2_BCR              5
#define ARM_OP2_WVR              6
#define ARM_OP2_WCR              7

/* Base register numbers for the debug registers. */

#define ARM_BASE_BVR             64
#define ARM_BASE_BCR             80
#define ARM_BASE_WVR             96
#define ARM_BASE_WCR             112

#define ARM_ADDBRP_EVENT         0
#define ARM_ADDWRP_EVENT         1

/* Accessor macros for the debug registers. */
#define ARM_DBG_READ(n, m, op2, val) \
  do { \
    asm volatile("mrc p14, 0, %0, " #n "," #m ", " #op2 : "=r" (val)); \
  } while (0)

#define ARM_DBG_WRITE(n, m, op2, val) \
  do { \
    asm volatile("mcr p14, 0, %0, " #n "," #m ", " #op2 : : "r" (val)); \
  } while (0)

#define ARM_READ_WB_REG_CASE(op2, m, val)    \
  case ((op2 << 4) + m):                 \
    ARM_DBG_READ(c0, c ## m, op2, val);  \
    break

#define ARM_WRITE_WB_REG_CASE(op2, m, val)   \
  case ((op2 << 4) + m):                 \
    ARM_DBG_WRITE(c0, c ## m, op2, val); \
    break

#define ARM_GEN_READ_WB_REG_CASES(op2, val) \
  ARM_READ_WB_REG_CASE(op2, 0, val);    \
  ARM_READ_WB_REG_CASE(op2, 1, val);    \
  ARM_READ_WB_REG_CASE(op2, 2, val);    \
  ARM_READ_WB_REG_CASE(op2, 3, val);    \
  ARM_READ_WB_REG_CASE(op2, 4, val);    \
  ARM_READ_WB_REG_CASE(op2, 5, val);    \
  ARM_READ_WB_REG_CASE(op2, 6, val);    \
  ARM_READ_WB_REG_CASE(op2, 7, val);    \
  ARM_READ_WB_REG_CASE(op2, 8, val);    \
  ARM_READ_WB_REG_CASE(op2, 9, val);    \
  ARM_READ_WB_REG_CASE(op2, 10, val);   \
  ARM_READ_WB_REG_CASE(op2, 11, val);   \
  ARM_READ_WB_REG_CASE(op2, 12, val);   \
  ARM_READ_WB_REG_CASE(op2, 13, val);   \
  ARM_READ_WB_REG_CASE(op2, 14, val);   \
  ARM_READ_WB_REG_CASE(op2, 15, val)

#define ARM_GEN_WRITE_WB_REG_CASES(op2, val) \
  ARM_WRITE_WB_REG_CASE(op2, 0, val);     \
  ARM_WRITE_WB_REG_CASE(op2, 1, val);     \
  ARM_WRITE_WB_REG_CASE(op2, 2, val);     \
  ARM_WRITE_WB_REG_CASE(op2, 3, val);     \
  ARM_WRITE_WB_REG_CASE(op2, 4, val);     \
  ARM_WRITE_WB_REG_CASE(op2, 5, val);     \
  ARM_WRITE_WB_REG_CASE(op2, 6, val);     \
  ARM_WRITE_WB_REG_CASE(op2, 7, val);     \
  ARM_WRITE_WB_REG_CASE(op2, 8, val);     \
  ARM_WRITE_WB_REG_CASE(op2, 9, val);     \
  ARM_WRITE_WB_REG_CASE(op2, 10, val);    \
  ARM_WRITE_WB_REG_CASE(op2, 11, val);    \
  ARM_WRITE_WB_REG_CASE(op2, 12, val);    \
  ARM_WRITE_WB_REG_CASE(op2, 13, val);    \
  ARM_WRITE_WB_REG_CASE(op2, 14, val);    \
  ARM_WRITE_WB_REG_CASE(op2, 15, val)

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct arm_hw_breakpoint_ctrl
{
  uint32_t __reserved  : 9,
  mismatch  : 1,
            : 9,
  len       : 8,
  type      : 2,
  privilege : 2,
  enabled   : 1;
};

struct arm_hw_breakpoint
{
  uint32_t  addr;
  struct    arm_hw_breakpoint_ctrl ctrl;
};

struct arm_breakpoint_slot
{
  /* Breakpoint currently in use for each BRP, WRP */

  struct arm_hw_breakpoint brps[ARM_MAX_BRP];
  struct arm_hw_breakpoint wrps[ARM_MAX_WRP];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct arm_breakpoint_slot g_cpu_bp_slot[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t arm_read_wb_reg(int n)
{
  uint32_t val = 0;

  switch (n)
    {
      ARM_GEN_READ_WB_REG_CASES(ARM_OP2_BVR, val);
      ARM_GEN_READ_WB_REG_CASES(ARM_OP2_BCR, val);
      ARM_GEN_READ_WB_REG_CASES(ARM_OP2_WVR, val);
      ARM_GEN_READ_WB_REG_CASES(ARM_OP2_WCR, val);
      default:
        binfo("attempt to read from unknown breakpoint register %d\n", n);
    }

  return val;
}

static void arm_write_wb_reg(int n, uint32_t val)
{
  switch (n)
    {
      ARM_GEN_WRITE_WB_REG_CASES(ARM_OP2_BVR, val);
      ARM_GEN_WRITE_WB_REG_CASES(ARM_OP2_BCR, val);
      ARM_GEN_WRITE_WB_REG_CASES(ARM_OP2_WVR, val);
      ARM_GEN_WRITE_WB_REG_CASES(ARM_OP2_WCR, val);
      default:
        binfo("attempt to write to unknown breakpoint register %d\n", n);
    }
}

static uint32_t arm_encode_ctrl_reg(struct arm_hw_breakpoint_ctrl *ctrl)
{
  return (ctrl->mismatch << 22) | (ctrl->len << 5) | (ctrl->type << 3) |
         (ctrl->privilege << 1) | ctrl->enabled;
}

/* Check if 8-bit byte-address select is available.
 * This clobbers WRP 0.
 */

static uint8_t arm_get_max_wp_len(void)
{
  struct arm_hw_breakpoint_ctrl ctrl;
  uint32_t ctrl_reg;
  uint8_t size = 4;

  memset(&ctrl, 0, sizeof(ctrl));
  ctrl.len = ARM_BREAKPOINT_LEN_8;
  ctrl_reg = arm_encode_ctrl_reg(&ctrl);

  arm_write_wb_reg(ARM_BASE_WVR, 0);
  arm_write_wb_reg(ARM_BASE_WCR, ctrl_reg);
  if ((arm_read_wb_reg(ARM_BASE_WCR) & ctrl_reg) == ctrl_reg)
    {
      size = 8;
    }

  return size;
}

/* Determine number of BRP registers available. */

static int arm_get_num_brp_resources(void)
{
  uint32_t didr;
  ARM_DBG_READ(c0, c0, 0, didr);
  return ((didr >> 24) & 0xf) + 1;
}

/* Does this core support mismatch breakpoints? */

static int core_has_mismatch_brps(void)
{
  return arm_get_num_brp_resources() > 1;
}

/* Determine number of usable WRPs available. */

static int arm_get_num_wrps(void)
{
  uint32_t didr;
  ARM_DBG_READ(c0, c0, 0, didr);
  return ((didr >> 28) & 0xf) + 1;
}

/* Determine number of usable BRPs available. */

static int arm_get_num_brps(void)
{
  int brps = arm_get_num_brp_resources();
  return core_has_mismatch_brps() ? brps - 1 : brps;
}

/* This function attempts to enable the monitor mode on an ARM processor.
 * Monitor mode is a debugging mode that allows the debugger to access
 * and modify processor registers for more in-depth debugging.
 */

static int arm_enable_monitor_mode(void)
{
  uint32_t dscr;
  ARM_DBG_READ(c0, c1, 0, dscr);

  /* If monitor mode is already enabled, just return. */

  if (dscr & ARM_DSCR_MDBGEN)
    {
      return 0;
    }

  ARM_DBG_WRITE(c0, c2, 2, (dscr | ARM_DSCR_MDBGEN));

  /* Check that the write made it through. */

  ARM_DBG_READ(c0, c1, 0, dscr);
  if (!(dscr & ARM_DSCR_MDBGEN))
    {
      return -EPERM;
    }

  return 0;
}

static int add_debugpoint(int event, struct arm_hw_breakpoint *bp)
{
  struct arm_hw_breakpoint *p;
  int size;
  int cpu;
  int i;

  cpu = this_cpu();
  if (event == ARM_ADDBRP_EVENT)
    {
      p = g_cpu_bp_slot[cpu].brps;
      size = arm_get_num_brps();
    }
  else
    {
      p = g_cpu_bp_slot[cpu].wrps;
      size = arm_get_num_wrps();
    }

  for (i = 0; i < size; ++i, p += sizeof(struct arm_hw_breakpoint))
    {
      if (!p->ctrl.enabled)
        {
          break;
        }
    }

  if (i == size)
    {
      goto out;
    }

  binfo("CPU[%d] add %s[%d] at %p size %d\n",
        cpu, (event == ARM_ADDBRP_EVENT) ? "Breakpoint" : "Watchpoint",
        i, bp->addr,
        ((bp->ctrl.len == ARM_BREAKPOINT_LEN_1) ? 1 :
         (bp->ctrl.len == ARM_BREAKPOINT_LEN_2) ? 2 :
         (bp->ctrl.len == ARM_BREAKPOINT_LEN_4) ? 4 : 8));

  memcpy(p, bp, sizeof(struct arm_hw_breakpoint));
  return i;

out:
  return -1;
}

static int remove_debugpoint(int event, struct arm_hw_breakpoint *bp)
{
  struct arm_hw_breakpoint *p;
  int size;
  int cpu;
  int i;

  cpu = this_cpu();
  if (event == ARM_ADDBRP_EVENT)
    {
      p = g_cpu_bp_slot[cpu].brps;
      size = arm_get_num_brps();
    }
  else
    {
      p = g_cpu_bp_slot[cpu].wrps;
      size = arm_get_num_wrps();
    }

  for (i = 0; i < size; i++, p += sizeof(struct arm_hw_breakpoint))
    {
      if (p->addr == bp->addr)
        {
          break;
        }
    }

  if (i == size)
    {
      goto out;
    }

  binfo("CPU[%d] remove %s[%d] at %p size %d\n",
        cpu, (event == ARM_ADDBRP_EVENT) ? "Breakpoint" : "Watchpoint",
        i, bp->addr,
        ((bp->ctrl.len == ARM_BREAKPOINT_LEN_1) ? 1 :
         (bp->ctrl.len == ARM_BREAKPOINT_LEN_2) ? 2 :
         (bp->ctrl.len == ARM_BREAKPOINT_LEN_4) ? 4 : 8));

  memset(p, 0, sizeof(struct arm_hw_breakpoint));
  return i;

out:
  return -1;
}

/* Install a perf counter breakpoint. */

static int arm_install_hw_breakpoint(struct arm_hw_breakpoint *bp)
{
  int ctrl_base;
  int val_base;
  int index;

  bp->ctrl.enabled = 1;

  if (bp->ctrl.type == ARM_BREAKPOINT_EXECUTE)
    {
      /* Breakpoint for code execution */

      index = add_debugpoint(ARM_ADDBRP_EVENT, bp);
      ctrl_base = ARM_BASE_BCR;
      val_base = ARM_BASE_BVR;
    }
  else
    {
      /* Watchpoint for memory access */

      index = add_debugpoint(ARM_ADDWRP_EVENT, bp);
      ctrl_base = ARM_BASE_WCR;
      val_base = ARM_BASE_WVR;
    }

  if (index < 0)
    {
      return -EINVAL;
    }

  arm_write_wb_reg(val_base + index, bp->addr);
  arm_write_wb_reg(ctrl_base + index, arm_encode_ctrl_reg(&bp->ctrl));

  return 0;
}

/* Uninstall a perf counter breakpoint. */

static int arm_uninstall_hw_breakpoint(struct arm_hw_breakpoint *bp)
{
  int index;
  int base;

  if (bp->ctrl.type == ARM_BREAKPOINT_EXECUTE)
    {
      /* Breakpoint */

      index = remove_debugpoint(ARM_ADDBRP_EVENT, bp);
      base = ARM_BASE_BCR;
    }
  else
    {
      /* Watchpoint */

      index = remove_debugpoint(ARM_ADDWRP_EVENT, bp);
      base = ARM_BASE_WCR;
    }

  if (index < 0)
    {
      return -EINVAL;
    }

  arm_write_wb_reg(base + index, 0);
  return 0;
}

static int arm_build_bp_info(struct arm_hw_breakpoint *bp, void *addr,
                             size_t size, int type)
{
  memset(bp, 0, sizeof(struct arm_hw_breakpoint));

  bp->addr = (uint32_t)addr;
  bp->ctrl.privilege = ARM_BREAKPOINT_PRIV | ARM_BREAKPOINT_USER;

  switch (type)
    {
      case DEBUGPOINT_WATCHPOINT_RO:
        bp->ctrl.type = ARM_BREAKPOINT_LOAD;
        break;

      case DEBUGPOINT_WATCHPOINT_WO:
        bp->ctrl.type = ARM_BREAKPOINT_STORE;
        break;

      case DEBUGPOINT_WATCHPOINT_RW:
        bp->ctrl.type = ARM_BREAKPOINT_LOAD | ARM_BREAKPOINT_STORE;
        break;

      case DEBUGPOINT_BREAKPOINT:
      case DEBUGPOINT_STEPPOINT:
        bp->ctrl.type = ARM_BREAKPOINT_EXECUTE;
        break;

      default:
        return -EINVAL;
    }

  switch (size)
    {
      case 1:
        bp->ctrl.len = ARM_BREAKPOINT_LEN_1;
        break;

      case 2:
        bp->ctrl.len = ARM_BREAKPOINT_LEN_2;
        break;

      case 4:
        bp->ctrl.len = ARM_BREAKPOINT_LEN_4;
        break;

      case 8:
      default:
        bp->ctrl.len = ARM_BREAKPOINT_LEN_8;
        break;
    }

  return 0;
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
  int ret;

  /* Determine how many BRPs/WRPs are available. */

  binfo("found %d " "%s" "breakpoint and %d watchpoint registers.\n",
        arm_get_num_brps(), core_has_mismatch_brps() ? "(+1 reserved) " :
        "", arm_get_num_wrps());

  /* Work out the maximum supported watchpoint length. */

  binfo("maximum watchpoint size is %u bytes.\n",
         arm_get_max_wp_len());

  ret = arm_enable_monitor_mode();
  if (ret)
    {
      binfo("Failed to enable monitor mode on CPU %d.\n", this_cpu());
    }

  return ret;
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
  struct arm_hw_breakpoint bp;

  if (arm_build_bp_info(&bp, addr, size, type))
    {
      binfo("Failed to build breakpoint info\n");
      return -EINVAL;
    }

  return arm_install_hw_breakpoint(&bp);
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
  struct arm_hw_breakpoint bp;

  if (arm_build_bp_info(&bp, addr, size, type))
    {
      binfo("Failed to build breakpoint info\n");
      return -EINVAL;
    }

  return arm_uninstall_hw_breakpoint(&bp);
}
