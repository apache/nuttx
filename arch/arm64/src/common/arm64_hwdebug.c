/****************************************************************************
 * arch/arm64/src/common/arm64_hwdebug.c
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
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/sched.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <sched/sched.h>

#include "arm64_hwdebug.h"
#include "arm64_fatal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define READ_WB_REG_CASE(OFF, N, REG, VAL) \
  case (OFF + N):                          \
    AARCH64_DBG_READ(N, REG, VAL);         \
    break

#define WRITE_WB_REG_CASE(OFF, N, REG, VAL) \
  case (OFF + N):                           \
    AARCH64_DBG_WRITE(N, REG, VAL);         \
    break

#define GEN_READ_WB_REG_CASES(OFF, REG, VAL) \
  READ_WB_REG_CASE(OFF,  0, REG, VAL);       \
  READ_WB_REG_CASE(OFF,  1, REG, VAL);       \
  READ_WB_REG_CASE(OFF,  2, REG, VAL);       \
  READ_WB_REG_CASE(OFF,  3, REG, VAL);       \
  READ_WB_REG_CASE(OFF,  4, REG, VAL);       \
  READ_WB_REG_CASE(OFF,  5, REG, VAL);       \
  READ_WB_REG_CASE(OFF,  6, REG, VAL);       \
  READ_WB_REG_CASE(OFF,  7, REG, VAL);       \
  READ_WB_REG_CASE(OFF,  8, REG, VAL);       \
  READ_WB_REG_CASE(OFF,  9, REG, VAL);       \
  READ_WB_REG_CASE(OFF, 10, REG, VAL);       \
  READ_WB_REG_CASE(OFF, 11, REG, VAL);       \
  READ_WB_REG_CASE(OFF, 12, REG, VAL);       \
  READ_WB_REG_CASE(OFF, 13, REG, VAL);       \
  READ_WB_REG_CASE(OFF, 14, REG, VAL);       \
  READ_WB_REG_CASE(OFF, 15, REG, VAL)

#define GEN_WRITE_WB_REG_CASES(OFF, REG, VAL) \
  WRITE_WB_REG_CASE(OFF,  0, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF,  1, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF,  2, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF,  3, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF,  4, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF,  5, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF,  6, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF,  7, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF,  8, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF,  9, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF, 10, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF, 11, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF, 12, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF, 13, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF, 14, REG, VAL);       \
  WRITE_WB_REG_CASE(OFF, 15, REG, VAL)

enum hw_breakpoint_ops
{
  HW_BREAKPOINT_INSTALL,
  HW_BREAKPOINT_UNINSTALL,
  HW_BREAKPOINT_RESTORE
};

enum dbg_active_el
{
  DBG_ACTIVE_EL0 = 0,
  DBG_ACTIVE_EL1,
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct arm64_breakpoint_context g_cpu_bp_ctx[CONFIG_SMP_NCPUS];
static struct arm64_breakpoint_context g_cpu_wp_ctx[CONFIG_SMP_NCPUS];
static struct arm64_debugpoint_slot    g_debugpoint_slots[CONFIG_SMP_NCPUS];

static int g_mde_ref_count[CONFIG_SMP_NCPUS];
static int g_kde_ref_count[CONFIG_SMP_NCPUS];

static struct list_node g_break_inst_hook_list_el0;
static struct list_node g_break_inst_hook_list_el1;

static spinlock_t g_debugpoint_slots_lock;
static spinlock_t g_debug_hook_lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t read_wb_reg(int reg, int n)
{
  uint64_t val = 0;

  switch (reg + n)
  {
    GEN_READ_WB_REG_CASES(AARCH64_DBG_REG_BVR,
                          AARCH64_DBG_REG_NAME_BVR, val);
    GEN_READ_WB_REG_CASES(AARCH64_DBG_REG_BCR,
                          AARCH64_DBG_REG_NAME_BCR, val);
    GEN_READ_WB_REG_CASES(AARCH64_DBG_REG_WVR,
                          AARCH64_DBG_REG_NAME_WVR, val);
    GEN_READ_WB_REG_CASES(AARCH64_DBG_REG_WCR,
                          AARCH64_DBG_REG_NAME_WCR, val);

    default:
    {
      sinfo("attempt to read from unknown breakpoint register %d\n", n);
    }
  }

  return val;
}

static void write_wb_reg(int reg, int n, uint64_t val)
{
  switch (reg + n)
  {
    GEN_WRITE_WB_REG_CASES(AARCH64_DBG_REG_BVR,
                           AARCH64_DBG_REG_NAME_BVR, val);
    GEN_WRITE_WB_REG_CASES(AARCH64_DBG_REG_BCR,
                           AARCH64_DBG_REG_NAME_BCR, val);
    GEN_WRITE_WB_REG_CASES(AARCH64_DBG_REG_WVR,
                           AARCH64_DBG_REG_NAME_WVR, val);
    GEN_WRITE_WB_REG_CASES(AARCH64_DBG_REG_WCR,
                           AARCH64_DBG_REG_NAME_WCR, val);

    default:
    {
      sinfo("attempt to write to unknown breakpoint register %d\n", n);
    }
  }

  ARM64_ISB();
}

static uint8_t hw_breakpoint_count(void)
{
  uint64_t dfr0  = read_sysreg(id_aa64dfr0_el1);
  uint8_t  count = (uint8_t)(((dfr0 & ARM64_ID_AADFR0_EL1_BRPS) >>
                   ARM64_ID_AADFR0_EL1_BRPS_SHIFT) + 1lu);

  return count;
}

static uint8_t hw_watchpoint_count(void)
{
  uint64_t dfr0  = read_sysreg(id_aa64dfr0_el1);
  uint8_t  count = (uint8_t)(((dfr0 & ARM64_ID_AADFR0_EL1_WRPS) >>
                   ARM64_ID_AADFR0_EL1_WRPS_SHIFT) + 1lu);

  return count;
}

/* MDSCR access routines. */

static void mdscr_write(uint32_t mdscr)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_debug_hook_lock);
  write_sysreg(mdscr, mdscr_el1);
  spin_unlock_irqrestore(&g_debug_hook_lock, flags);
}

static uint32_t mdscr_read(void)
{
  return read_sysreg(mdscr_el1);
}

static void enable_debug_monitors(enum dbg_active_el el)
{
  uint32_t mdscr;
  uint32_t enable = 0;
  uint8_t  cpu;

  cpu = this_cpu();
  g_mde_ref_count[cpu]++;
  g_kde_ref_count[cpu]++;

  if (g_mde_ref_count[cpu] == 1)
    {
      enable = DBG_MDSCR_MDE;
    }

  if (el == DBG_ACTIVE_EL1 && g_kde_ref_count[cpu] == 1)
    {
      enable |= DBG_MDSCR_KDE;
    }

  if (enable)
    {
      mdscr = mdscr_read();
      mdscr |= enable;
      mdscr_write(mdscr);
    }
}

static void disable_debug_monitors(enum dbg_active_el el)
{
  uint32_t mdscr;
  uint32_t disable = 0;
  uint8_t  cpu;

  cpu = this_cpu();

  g_mde_ref_count[cpu]--;
  g_kde_ref_count[cpu]--;

  if (g_mde_ref_count[cpu] == 0)
    {
      disable = ~DBG_MDSCR_MDE;
    }

  if (el == DBG_ACTIVE_EL1 && g_kde_ref_count[cpu] == 0)
    {
      disable &= ~DBG_MDSCR_KDE;
    }

  if (disable)
    {
      mdscr = mdscr_read();
      mdscr &= disable;
      mdscr_write(mdscr);
    }
}

static void register_debug_hook(struct list_node *list_head,
                                struct list_node *entry)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_debug_hook_lock);
  if (list_is_empty(entry))
    {
      list_add_tail(list_head, entry);
    }

  spin_unlock_irqrestore(&g_debug_hook_lock, flags);
}

static void unregister_debug_hook(struct list_node *node)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_debug_hook_lock);
  list_delete_init(node);
  spin_unlock_irqrestore(&g_debug_hook_lock, flags);
}

static int call_break_inst_hook(struct regs_context *regs, uint64_t esr)
{
  struct list_node       *list;
  break_func_t            func = NULL;
  struct break_inst_hook *curr;
  struct break_inst_hook *next;
  uint64_t                comment;
  int                     el;
  irqstate_t              flags;

  el = arm64_current_el();
  switch (el)
  {
    case MODE_EL1:
    {
      list = &g_break_inst_hook_list_el1;
      break;
    }

    case MODE_EL0:
    {
      list = &g_break_inst_hook_list_el0;
      break;
    }

    case MODE_EL2:
    default:
    {
      return DBG_HOOK_ERROR;
    }
  }

  /* brk exception disables interrupt, this function is
   * entirely not preemptible and wait, and we can use
   * list safely here.
   */

  flags = spin_lock_irqsave(&g_debug_hook_lock);
  list_for_every_entry_safe(list, curr, next, struct break_inst_hook, entry)
  {
    comment = esr & ESR_ELX_BRK64_ISS_COMMENT_MASK;

    if ((comment & ~curr->mask) == curr->imm)
      {
        func = curr->func;
      }
  }

  spin_unlock_irqrestore(&g_debug_hook_lock, flags);

  return func ? func(regs, esr) : DBG_HOOK_ERROR;
}

static int arm64_brk_handler(struct regs_context *regs, uint64_t far,
                             uint64_t esr)
{
  int el;
  int ret;

  ret = call_break_inst_hook(regs, esr);

  if (ret == 0)
    {
      return ret;
    }

  el = arm64_current_el();
  sinfo("Unexpected kernel BRK exception at EL%d\n", el);
  return -EFAULT;
}

static struct arch_hw_breakpoint *arm64_hw_breakpoint_find(int handle)
{
  uint8_t cpu;

  VERIFY(handle > 0 && handle <= ARM64_MAX_HBP_SLOTS);

  cpu = this_cpu();

  return (handle < ARM64_MAX_BRP) ?
          &g_cpu_bp_ctx[cpu].on_reg[handle]:
          &g_cpu_wp_ctx[cpu].on_reg[handle - ARM64_MAX_BRP];
}

static void arm64_hw_breakpoint_put(int handle)
{
  uint8_t cpu;
  struct arm64_breakpoint_context *ctx;

  VERIFY(handle > 0 && handle <= ARM64_MAX_HBP_SLOTS);

  cpu = this_cpu();
  ctx = (handle < ARM64_MAX_BRP) ?
         &g_cpu_bp_ctx[cpu] : &g_cpu_wp_ctx[cpu];

  handle = handle < ARM64_MAX_BRP ? handle: handle - ARM64_MAX_BRP;
  ctx->on_reg[handle].in_used = 0;
}

static int arm64_hw_breakpoint_getslot(struct arch_hw_breakpoint **info,
                                       int type)
{
  int     i;
  int     handle = -ENOSPC;
  uint8_t cpu;

  struct arm64_breakpoint_context *ctx;

  cpu = this_cpu();
  ctx = (type == DEBUGPOINT_BREAKPOINT) ?
         &g_cpu_bp_ctx[cpu] : &g_cpu_wp_ctx[cpu];

  for (i = 0; ctx->core_num; i++)
    {
      if (!ctx->on_reg[i].in_used)
        {
          ctx->on_reg[i].in_used = 1;
          *info = &ctx->on_reg[i];
          handle = i;
          break;
        }
    }

  return (type == DEBUGPOINT_BREAKPOINT) ?
          handle : handle + ARM64_MAX_BRP;
}

/* Construct an arch_hw_breakpoint */

static int arch_build_bp_info(struct arch_hw_breakpoint *hw, uintptr_t addr,
                              size_t size, int type)
{
  /* Type */

  switch (type)
  {
    case DEBUGPOINT_BREAKPOINT:
    {
      hw->ctrl.type = ARM_BREAKPOINT_EXECUTE;
      break;
    }

    case DEBUGPOINT_WATCHPOINT_RO:
    {
      hw->ctrl.type = ARM_BREAKPOINT_LOAD;
      break;
    }

    case DEBUGPOINT_WATCHPOINT_WO:
    {
      hw->ctrl.type = ARM_BREAKPOINT_STORE;
      break;
    }

    case DEBUGPOINT_WATCHPOINT_RW:
    {
      hw->ctrl.type = ARM_BREAKPOINT_LOAD | ARM_BREAKPOINT_STORE;
      break;
    }

    default:
    {
      return -EINVAL;
    }
  }

  /* Len */

  switch (size)
  {
    case BREAKPOINT_LEN_1:
    {
      hw->ctrl.len = ARM_BREAKPOINT_LEN_1;
      break;
    }

    case BREAKPOINT_LEN_2:
    {
      hw->ctrl.len = ARM_BREAKPOINT_LEN_2;
      break;
    }

    case BREAKPOINT_LEN_3:
    {
      hw->ctrl.len = ARM_BREAKPOINT_LEN_3;
      break;
    }

    case BREAKPOINT_LEN_4:
    {
      hw->ctrl.len = ARM_BREAKPOINT_LEN_4;
      break;
    }

    case BREAKPOINT_LEN_5:
    {
      hw->ctrl.len = ARM_BREAKPOINT_LEN_5;
      break;
    }

    case BREAKPOINT_LEN_6:
    {
      hw->ctrl.len = ARM_BREAKPOINT_LEN_6;
      break;
    }

    case BREAKPOINT_LEN_7:
    {
      hw->ctrl.len = ARM_BREAKPOINT_LEN_7;
      break;
    }

    case BREAKPOINT_LEN_8:
    {
      hw->ctrl.len = ARM_BREAKPOINT_LEN_8;
      break;
    }

    default:
    {
      return -EINVAL;
    }
  }

  /* On AArch64, we only permit breakpoints of length 4, whereas
   * AArch32 also requires breakpoints of length 2 for Thumb.
   * Watchpoints can be of length 1, 2, 4 or 8 bytes.
   */

  if (hw->ctrl.type == ARM_BREAKPOINT_EXECUTE)
    {
      if (hw->ctrl.len != ARM_BREAKPOINT_LEN_2 &&
          hw->ctrl.len != ARM_BREAKPOINT_LEN_4)
        {
          return -EINVAL;
        }
    }

  /* Address */

  hw->address = addr;
  hw->type    = type;
  hw->size    = size;

  /* Privilege
   * Note that we disallow combined EL0/EL1 breakpoints because
   * that would complicate the stepping code.
   */

  hw->ctrl.privilege = AARCH64_BREAKPOINT_EL1;

  /* Enabled? */

  hw->ctrl.enabled = 0;

  return 0;
}

/* Validate the arch-specific HW Breakpoint register settings. */

int arm64_hw_breakpoint_build(struct arch_hw_breakpoint **hw_ret,
                              uintptr_t addr,
                              size_t size, int type)
{
  int                        ret;
  int                        handle;
  uint64_t                   alignment_mask;
  uint64_t                   offset;
  struct arch_hw_breakpoint *hw = NULL;

  handle = arm64_hw_breakpoint_getslot(&hw, type);
  if (handle < 0)
    {
      return handle;
    }

  /* Build the arch_hw_breakpoint. */

  ret = arch_build_bp_info(hw, addr, size, type);
  if (ret < 0)
    {
      goto error_return;
    }

  /* Check address alignment.
   * We don't do any clever alignment correction for watchpoints
   * because using 64-bit unaligned addresses is deprecated for
   * AArch64.
   *
   * AArch32 tasks expect some simple alignment fixups, so emulate
   * that here.
   */

  if (hw->ctrl.len == ARM_BREAKPOINT_LEN_8)
    {
      alignment_mask = 0x7;
    }
  else
    {
      alignment_mask = 0x3;
    }

  offset = hw->address & alignment_mask;
  switch (offset)
  {
    case 0:
    {
      /* Aligned */

      break;
    }

    case 1:
    case 2:
    {
      /* Allow halfword watchpoints and breakpoints. */

      if (hw->ctrl.len == ARM_BREAKPOINT_LEN_2)
        {
          break;
        }
    }

    case 3:
    {
      /* Allow single byte watchpoint. */

      if (hw->ctrl.len == ARM_BREAKPOINT_LEN_1)
        {
          break;
        }
    }

    default:
    {
      ret = -EINVAL;
      goto error_return;
    }
  }

  hw->address   &= ~alignment_mask;
  hw->ctrl.len  <<= offset;
  *hw_ret = hw;

  return handle;

error_return:
  arm64_hw_breakpoint_put(handle);
  return ret;
}

static int arm64_hw_breakpoint_control(int handle,
                                       enum hw_breakpoint_ops ops)
{
  struct arm64_breakpoint_context *ctx;
  struct arch_hw_breakpoint       *info;
  enum dbg_active_el               dbg_el = DBG_ACTIVE_EL1;

  int      i;
  int      ctrl_reg;
  int      val_reg;
  int      reg_enable;
  uint32_t ctrl;
  uint8_t  cpu;

  VERIFY(handle > 0 && handle <= ARM64_MAX_HBP_SLOTS);

  cpu = this_cpu();
  ctx = (handle < ARM64_MAX_BRP) ?
         &g_cpu_bp_ctx[cpu] : &g_cpu_wp_ctx[cpu];

  VERIFY(handle > 0 && handle <= ARM64_MAX_HBP_SLOTS);

  info = arm64_hw_breakpoint_find(handle);

  if (info->ctrl.type == ARM_BREAKPOINT_EXECUTE)
    {
      /* Breakpoint */

      ctrl_reg      = AARCH64_DBG_REG_BCR;
      val_reg       = AARCH64_DBG_REG_BVR;
      reg_enable    = !ctx->disabled;
    }
  else
    {
      /* Watchpoint */

      ctrl_reg      = AARCH64_DBG_REG_WCR;
      val_reg       = AARCH64_DBG_REG_WVR;
      reg_enable    = !ctx->disabled;
    }

  i = (handle < ARM64_MAX_BRP) ? handle: handle - ARM64_MAX_BRP;

  switch (ops)
  {
    case HW_BREAKPOINT_INSTALL:
    {
      /* Ensure debug monitors are enabled at the correct exception
       * level.
       */

      enable_debug_monitors(dbg_el);
    }

    case HW_BREAKPOINT_RESTORE:
    {
      /* Setup the address register. */

      write_wb_reg(val_reg, i, info->address);

      /* Setup the control register. */

      ctrl = encode_ctrl_reg(info->ctrl);
      write_wb_reg(ctrl_reg, i, reg_enable ? ctrl | 0x1 : ctrl & ~0x1);
      break;
    }

    case HW_BREAKPOINT_UNINSTALL:
    {
      /* Reset the control register. */

      write_wb_reg(ctrl_reg, i, 0);

      /* Release the debug monitors for the correct exception
       * level.
       */

      disable_debug_monitors(dbg_el);
      break;
    }
  }

  return 0;
}

static int arm64_breakpoint_report(struct arch_hw_breakpoint *bp,
                                   uint64_t addr, struct regs_context *regs)
{
  bp->trigger = addr;
  bp->handle_fn(bp->type, (void *)addr, bp->size, bp->arg);

  return 0;
}

static int arm64_watchpoint_report(struct arch_hw_breakpoint *wp,
                                   uint64_t addr, struct regs_context *regs)
{
  wp->trigger = addr;
  wp->handle_fn(wp->type, (void *)addr, wp->size, wp->arg);

  return 0;
}

/* Enable/disable all of the breakpoints active at the specified
 * exception level at the register level.
 * This is used when single-stepping after a breakpoint exception.
 */

static void arm64_toggle_bp_registers(int reg, enum dbg_active_el el,
                                      int enable)
{
  struct arm64_breakpoint_context *ctx;
  int      i;
  uint32_t ctrl;
  uint8_t  cpu;

  cpu = this_cpu();

  switch (reg)
  {
    case AARCH64_DBG_REG_BCR:
    {
      ctx = &g_cpu_bp_ctx[cpu];
      break;
    }

    case AARCH64_DBG_REG_WCR:
    {
      ctx = &g_cpu_wp_ctx[cpu];
      break;
    }

    default:
    {
      return;
    }
  }

  for (i = 0; i < ctx->core_num; ++i)
    {
      if (!ctx->on_reg[i].in_used)
        {
          continue;
        }

      ctrl = read_wb_reg(reg, i);
      if (enable)
        {
          ctrl |= 0x1;
        }
      else
        {
          ctrl &= ~0x1;
        }

      write_wb_reg(reg, i, ctrl);
    }
}

static int arm64_breakpoint_handler(struct regs_context *regs,
                                    uint64_t unused, uint64_t esr)
{
  struct arm64_breakpoint_context *ctx;
  struct arch_hw_breakpoint       *bp;
  struct arch_hw_breakpoint_ctrl   ctrl;
  int      i;
  int      step = 0;
  uint32_t ctrl_reg;
  uint64_t addr;
  uint64_t val;
  uint8_t  cpu;
  int      el;

  addr = regs->elr;

  cpu = this_cpu();
  ctx = &g_cpu_bp_ctx[cpu];

  for (i = 0; i < ctx->core_num; ++i)
    {
      if (!ctx->on_reg[i].in_used)
        {
          continue;
        }

      bp = &ctx->on_reg[i];

      /* Check if the breakpoint value matches. */

      val = read_wb_reg(AARCH64_DBG_REG_BVR, i);
      if (val != (addr & ~0x3))
        {
          continue;
        }

      /* Possible match, check the byte address select to confirm. */

      ctrl_reg = read_wb_reg(AARCH64_DBG_REG_BCR, i);
      decode_ctrl_reg(ctrl_reg, &ctrl);

      if (!((1 << (addr & 0x3)) & ctrl.len))
        {
          continue;
        }

      bp->trigger = addr;
      step        = arm64_breakpoint_report(bp, addr, regs);
    }

  if (step == 1)
    {
      return 0;
    }

  el = arm64_current_el();
  switch (el)
  {
    case MODE_EL1:
    {
      arm64_toggle_bp_registers(AARCH64_DBG_REG_BCR, DBG_ACTIVE_EL1, 0);
      break;
    }

    case MODE_EL0:
    {
      arm64_toggle_bp_registers(AARCH64_DBG_REG_BCR, DBG_ACTIVE_EL0, 0);
      break;
    }

    case MODE_EL2:
    default:
    {
      break;
    }
  }

  return 0;
}

static int arm64_watchpoint_handler(struct regs_context *regs, uint64_t addr,
                                    uint64_t esr)
{
  struct   arm64_breakpoint_context *ctx;
  struct   arch_hw_breakpoint       *wp;
  struct   arch_hw_breakpoint_ctrl  ctrl;
  int      i;
  int      step = 0;
  int      access;
  uint32_t ctrl_reg;
  uint64_t val;
  uint8_t  cpu;
  int      el;

  cpu   = this_cpu();
  ctx   = &g_cpu_wp_ctx[cpu];

  /* Find all watchpoints that match the reported address. If no exact
   * match is found. Attribute the hit to the closest watchpoint.
   */

  for (i = 0; i < ctx->core_num; ++i)
    {
      if (!ctx->on_reg[i].in_used)
        {
          continue;
        }

      wp = &ctx->on_reg[i];

      /* Check that the access type matches.
       * 0 => load, otherwise => store
       */

      access = (esr & AARCH64_ESR_ACCESS_MASK) ?
               ARM_BREAKPOINT_STORE :ARM_BREAKPOINT_LOAD;

      if (!(access & wp->ctrl.type))
        {
          continue;
        }

      /* Check if the watchpoint value and byte select match. */

      val       = read_wb_reg(AARCH64_DBG_REG_WVR, i);
      ctrl_reg  = read_wb_reg(AARCH64_DBG_REG_WCR, i);
      decode_ctrl_reg(ctrl_reg, &ctrl);

      if (val != addr)
        {
          continue;
        }

      step = arm64_watchpoint_report(wp, addr, regs);
    }

  if (step == 1)
    {
      return 0;
    }

  /* We always disable EL0 watchpoints because the kernel can
   * cause these to fire via an unprivileged access.
   */

  arm64_toggle_bp_registers(AARCH64_DBG_REG_WCR, DBG_ACTIVE_EL0, 0);

  el = arm64_current_el();
  switch (el)
  {
    case MODE_EL1:
    {
      arm64_toggle_bp_registers(AARCH64_DBG_REG_WCR, DBG_ACTIVE_EL1, 0);
      break;
    }

    case MODE_EL0:
    case MODE_EL2:
    default:
    {
      break;
    }
  }

  return 0;
}

static int arm64_single_step_handler(struct regs_context *regs,
                                     uint64_t far, uint64_t esr)
{
  return 0;
}

static int arm64_clear_os_lock(unsigned int cpu)
{
  write_sysreg(0, osdlr_el1);
  write_sysreg(0, oslar_el1);

  ARM64_ISB();
  return 0;
}

/****************************************************************************
 * Name: arm64_hw_breakpoint_enable
 *
 * Description:
 *   enable a debugpoint.
 *
 * Input Parameters:
 *     handle    - The Handle number for this breakpoint
 *     enable    - enable/disable this breakpoint
 *                  true  - enable
 *                  false - disable
 *
 *  Returned Value:
 *     Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int arm64_hw_breakpoint_enable(int handle, bool enable)
{
  if (enable)
    {
      return arm64_hw_breakpoint_control(handle, HW_BREAKPOINT_INSTALL);
    }
  else
    {
      return arm64_hw_breakpoint_control(handle, HW_BREAKPOINT_UNINSTALL);
    }
}

static struct arm64_debugpoint *arm64_debugpoint_slot_find(int type,
                                                           void *addr,
                                                           size_t size)
{
  int     i;
  uint8_t cpu;
  struct arm64_debugpoint      *dbpoint = NULL;
  struct arm64_debugpoint_slot *slots;

  cpu   = this_cpu();
  slots = &g_debugpoint_slots[cpu];

  for (i = 0; i < ARM64_MAX_HBP_SLOTS; i++)
    {
      if (slots->slot[i].in_used)
        {
          if ((slots->slot[i].type == type) &&
              (slots->slot[i].addr == addr) &&
              (slots->slot[i].size == size))
            {
              dbpoint = &slots->slot[i];
              break;
            }
        }
    }

  return dbpoint;
}

static struct arm64_debugpoint *arm64_debugpoint_slot_getfree(void)
{
  int     i;
  uint8_t cpu;
  struct arm64_debugpoint      *dbpoint = NULL;
  struct arm64_debugpoint_slot *slots;

  cpu   = this_cpu();
  slots = &g_debugpoint_slots[cpu];

  for (i = 0; i < ARM64_MAX_HBP_SLOTS; i++)
    {
      if (!slots->slot[i].in_used)
        {
          dbpoint = &slots->slot[i];
          dbpoint->in_used = 1;
          break;
        }
    }

  return dbpoint;
}

static void arm64_debugpoint_slot_release(struct arm64_debugpoint *dbpoint)
{
  dbpoint->in_used  = 0;
  dbpoint->addr     = 0;
  dbpoint->type     = 0;
  dbpoint->size     = 0;
  dbpoint->hbp_slot = 0;
  dbpoint->in_used  = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm64_register_user_break_hook(struct break_inst_hook *hook)
{
  register_debug_hook(&hook->entry, &g_break_inst_hook_list_el0);
}

void arm64_unregister_user_break_hook(struct break_inst_hook *hook)
{
  unregister_debug_hook(&hook->entry);
}

void arm64_register_kernel_break_hook(struct break_inst_hook *hook)
{
  register_debug_hook(&hook->entry, &g_break_inst_hook_list_el1);
}

void arm64_unregister_kernel_break_hook(struct break_inst_hook *hook)
{
  unregister_debug_hook(&hook->entry);
}

/****************************************************************************
 * Name: up_debugpoint_add
 *
 * Description:
 *   Add a debugpoint.
 *
 * Input Parameters:
 *   type     - The debugpoint type. optional value:
 *                 DEBUGPOINT_WATCHPOINT_RO - Read only watchpoint.
 *                 DEBUGPOINT_WATCHPOINT_WO - Write only watchpoint.
 *                 DEBUGPOINT_WATCHPOINT_RW - Read and write watchpoint.
 *                 DEBUGPOINT_BREAKPOINT    - Breakpoint.
 *                 DEBUGPOINT_STEPPOINT     - Single step.
 *   addr     - The address to be debugged.
 *   size     - The watchpoint size. only for watchpoint(arm64 specific).
 *                 BREAKPOINT_LEN_1 = 1,
 *                 BREAKPOINT_LEN_2 = 2,
 *                 BREAKPOINT_LEN_3 = 3,
 *                 BREAKPOINT_LEN_4 = 4,
 *                 BREAKPOINT_LEN_5 = 5,
 *                 BREAKPOINT_LEN_6 = 6,
 *                 BREAKPOINT_LEN_7 = 7,
 *                 BREAKPOINT_LEN_8 = 8
 *   callback - The callback function when debugpoint triggered.
 *              if NULL, the debugpoint will be removed.
 *   arg      - The argument of callback function.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int up_debugpoint_add(int type, void *addr, size_t size,
                      debug_callback_t callback, void *arg)
{
  struct arch_hw_breakpoint *bp;
  struct arm64_debugpoint   *dbpoint;

  int        handle;
  int        ret;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_debugpoint_slots_lock);
  dbpoint = arm64_debugpoint_slot_find(type, addr, size);
  if (dbpoint != NULL)
    {
      sinfo("the debugpoint has been register\n");
      ret = -EEXIST;
      goto error_return;
    }

  /* get free debugpoint */

  dbpoint = arm64_debugpoint_slot_getfree();
  if (dbpoint == 0)
    {
      sinfo("Not more slot can be use\n");
      ret = -ENOSPC;
      goto error_return;
    }

  handle = arm64_hw_breakpoint_build(&bp, (uintptr_t)addr, size, type);
  if (handle < 0)
    {
      arm64_debugpoint_slot_release(dbpoint);
      ret = handle;
      goto error_return;
    }

  bp->handle_fn = callback;
  bp->arg       = arg;

  dbpoint->addr     = addr;
  dbpoint->size     = size;
  dbpoint->type     = type;
  dbpoint->hbp_slot = handle;

  ret = arm64_hw_breakpoint_enable(handle, true);

error_return:
  spin_unlock_irqrestore(&g_debugpoint_slots_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: up_debugpoint_remove
 *
 * Description:
 *   Remove a debugpoint.before remove the watchpoint/breakpoint,
 * it will disable frist
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
  struct arm64_debugpoint *dbpoint;
  int        ret;
  int        handle;
  irqstate_t flags;

  /* ret == 0, the breakpoint hasn't been registered
   * ret != 0, the breakpoint has been register
   */

  flags = spin_lock_irqsave(&g_debugpoint_slots_lock);
  dbpoint = arm64_debugpoint_slot_find(type, addr, size);
  if (dbpoint == NULL)
    {
      sinfo("the debugpoint hasn't been register\n");
      spin_unlock_irqrestore(&g_debugpoint_slots_lock, flags);
      return -ENODEV;
    }

  handle = dbpoint->hbp_slot;

  VERIFY(handle > 0 && handle <= ARM64_MAX_HBP_SLOTS);

  ret = arm64_hw_breakpoint_enable(handle, false);
  arm64_hw_breakpoint_put(handle);
  arm64_debugpoint_slot_release(dbpoint);

  spin_unlock_irqrestore(&g_debugpoint_slots_lock, flags);

  return ret;
}

/* One-time initialisation. */

void arm64_hwdebug_init(void)
{
  struct arm64_breakpoint_context *bp_ctx;
  struct arm64_breakpoint_context *wp_ctx;
  uint8_t cpu;

  cpu = this_cpu();
  bp_ctx = &g_cpu_bp_ctx[cpu];
  wp_ctx = &g_cpu_wp_ctx[cpu];

  bp_ctx->core_num = hw_breakpoint_count();
  wp_ctx->core_num = hw_watchpoint_count();

  sinfo("found %d breakpoint and %d watchpoint registers.\n",
        bp_ctx->core_num, wp_ctx->core_num);

  list_initialize(&g_break_inst_hook_list_el0);
  list_initialize(&g_break_inst_hook_list_el1);
  spin_initialize(&g_debug_hook_lock, SP_UNLOCKED);

  spin_initialize(&g_debugpoint_slots_lock, SP_UNLOCKED);

  bp_ctx->disabled = 0;
  wp_ctx->disabled = 0;

  /* Register debug fatal handlers. */

  arm64_register_debug_hook(DBG_ESR_EVT_HWBP, arm64_breakpoint_handler);
  arm64_register_debug_hook(DBG_ESR_EVT_HWWP, arm64_watchpoint_handler);
  arm64_register_debug_hook(DBG_ESR_EVT_HWSS, arm64_single_step_handler);
  arm64_register_debug_hook(DBG_ESR_EVT_BRK, arm64_brk_handler);

  arm64_clear_os_lock(cpu);
}
