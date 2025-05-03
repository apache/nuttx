/****************************************************************************
 * arch/x86_64/src/common/x86_64_hwdebug.c
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

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/gdbstub.h>

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define X86_64_HWBRKP_COUNT       (4)

/* Conditions for DRx */

#define X86_64_DR7_RW_BREAK       (0)
#define X86_64_DR7_RW_WATCH_W     (1)
#define X86_64_DR7_RW_WATCH_IO_RW (2)
#define X86_64_DR7_RW_WATCH_RW    (3)

/* Size of DRx */

#define X86_64_DR7_LEN_1B         (0)
#define X86_64_DR7_LEN_2B         (1)
#define X86_64_DR7_LEN_8B         (2)
#define X86_64_DR7_LEN_4B         (3)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DR7 register */

struct x86_64_dr7_reg_s
{
  uint64_t l0:1;              /* Bit 0: Local DR0 breakpoint */
  uint64_t g0:1;              /* Bit 1: Global DR0 breakpoint */
  uint64_t l1:1;              /* Bit 2: Local DR1 breakpoint */
  uint64_t g1:1;              /* Bit 3: Global DR1 breakpoint */
  uint64_t l2:1;              /* Bit 4: Local DR2 breakpoint */
  uint64_t g2:1;              /* Bit 5: Global DR2 breakpoint */
  uint64_t l3:1;              /* Bit 6: Local DR3 breakpoint */
  uint64_t g3:1;              /* Bit 7: Global DR3 breakpoint */
  uint64_t le:1;              /* Bit 8: Local Exact Breakpoint enable */
  uint64_t ge:1;              /* Bit 9: Global Extract Breakpoint enable */
  uint64_t _zeros0:6;         /* Bits 10-15: Zeros */
  uint64_t rw0:2;             /* Bits 16-17: Conditions for DR0 */
  uint64_t len0:2;            /* Bits 18-19: Size of DR0 breakpoint */
  uint64_t rw1:2;             /* Bits 20-21: Conditions for DR1 */
  uint64_t len1:2;            /* Bits 22-23: Size of DR1 breakpoint */
  uint64_t rw2:2;             /* Bits 24-25: Conditions for DR2 */
  uint64_t len2:2;            /* Bits 26:27: Size of DR2 breakpoint */
  uint64_t rw3:2;             /* Bits 28-29: Conditions for DR3 */
  uint64_t len3:2;            /* Bits 30-31: Size of DR3 breakpoint */
  uint64_t _zeros1:32;        /* Bits 32-64: Zeros */
};

union x86_64_dr7_reg_u
{
  struct x86_64_dr7_reg_s s;
  uint64_t                u64;
};

/* DR6 register */

struct x86_64_dr6_reg_s
{
  uint64_t b0:1;              /* Bit 0: BRK0 set */
  uint64_t b1:1;              /* Bit 1: BRK1 set */
  uint64_t b2:1;              /* Bit 2: BRK2 set */
  uint64_t b3:1;              /* Bit 3: BRK3 set */
  uint64_t _zeros0:9;         /* Bits 4-12: Zeros */
  uint64_t bd:1;              /* Bit 13: Debug Register Address Detected */
  uint64_t bs:1;              /* Bit 14: Singe-Step execution */
  uint64_t bt:1;              /* Bit 15: Task Switch breakpoint */
  uint64_t _zeros1:48;        /* Bits 16-64: Zeros */
};

union x86_64_dr6_reg_u
{
  struct x86_64_dr6_reg_s s;
  uint64_t                u64;
};

/* Debug trigger */

struct x86_64_debug_trigger_s
{
  bool              used;     /* Trigger is used */
  int               type;     /* Trigger type */
  void             *address;  /* Trigger address */
  size_t            size;     /* Trigger region size */
  debug_callback_t  callback; /* Debug callback */
  void             *arg;      /* Debug callback argument */
};

/* Debug context */

struct x86_64_debug_ctx_s
{
  struct x86_64_debug_trigger_s trigger[X86_64_HWBRKP_COUNT];
  struct x86_64_debug_trigger_s step;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Save the trigger address info */

static struct x86_64_debug_ctx_s g_dbg_ctx[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_drX() / set_drX()
 ****************************************************************************/

static inline void set_dr0(uint64_t dr0)
{
  __asm__ volatile("\tmov %0, %%dr0" :: "r"(dr0));
}

static inline uint64_t get_dr0(void)
{
  uint64_t regval;
  __asm__ volatile("\tmov %%dr0, %0\n" : "=r" (regval));
  return regval;
}

static inline void set_dr1(uint64_t dr1)
{
  __asm__ volatile("\tmov %0, %%dr1" :: "r"(dr1));
}

static inline uint64_t get_dr1(void)
{
  uint64_t regval;
  __asm__ volatile("\tmov %%dr1, %0\n" : "=r" (regval));
  return regval;
}

static inline void set_dr2(uint64_t dr2)
{
  __asm__ volatile("\tmov %0, %%dr2" :: "r"(dr2));
}

static inline uint64_t get_dr2(void)
{
  uint64_t regval;
  __asm__ volatile("\tmov %%dr2, %0\n" : "=r" (regval));
  return regval;
}

static inline void set_dr3(uint64_t dr3)
{
  __asm__ volatile("\tmov %0, %%dr3" :: "r"(dr3));
}

static inline uint64_t get_dr3(void)
{
  uint64_t regval;
  __asm__ volatile("\tmov %%dr3, %0\n" : "=r" (regval));
  return regval;
}

static inline void set_dr6(uint64_t dr6)
{
  __asm__ volatile("\tmov %0, %%dr6" :: "r"(dr6));
}

static inline uint64_t get_dr6(void)
{
  uint64_t regval;
  __asm__ volatile("\tmov %%dr6, %0\n" : "=r" (regval));
  return regval;
}

static inline void set_dr7(uint64_t dr7)
{
  __asm__ volatile("\tmov %0, %%dr7" :: "r"(dr7));
}

static inline uint64_t get_dr7(void)
{
  uint64_t regval;
  __asm__ volatile("\tmov %%dr7, %0\n" : "=r" (regval));
  return regval;
}

/****************************************************************************
 * Name: x86_64_debug_step
 ****************************************************************************/

static void x86_64_debug_step(bool enable)
{
  uint64_t *regs = g_running_tasks[this_cpu()]->xcp.regs;

  /* Reset or set Trap flag */

  if (enable)
    {
      regs[REG_RFLAGS] |= X86_64_RFLAGS_TF;
    }
  else
    {
      regs[REG_RFLAGS] &= ~X86_64_RFLAGS_TF;
    }

  /* Request full context switch so we update RFLAGS */

  regs[REG_AUX] |= REG_AUX_FULLCONTEXT;
}

/****************************************************************************
 * Name: x86_64_set_dr
 ****************************************************************************/

static void x86_64_set_dr(uint8_t i, uint8_t g, uint8_t rw, uint8_t len,
                          uint64_t addr)
{
  union x86_64_dr7_reg_u dr7;

  /* Get DR7 */

  dr7.u64 = get_dr7();

  switch (i)
    {
      case 0:
        {
          set_dr0(addr);
          dr7.s.g0   = 1;
          dr7.s.rw0  = rw;
          dr7.s.len0 = len;
          break;
        }

      case 1:
        {
          set_dr1(addr);
          dr7.s.g1   = 1;
          dr7.s.rw1  = rw;
          dr7.s.len1 = len;
          break;
        }

      case 2:
        {
          set_dr2(addr);
          dr7.s.g2   = 1;
          dr7.s.rw2  = rw;
          dr7.s.len2 = len;
          break;
        }

      case 3:
        {
          set_dr3(addr);
          dr7.s.g3   = 1;
          dr7.s.rw3  = rw;
          dr7.s.len3 = len;
          break;
        }

      default:
        {
          _err("unsupported DR %d\n", i);
          PANIC();
        }
    }

  /* Update DR7 */

  set_dr7(dr7.u64);
}

/****************************************************************************
 * Name: x86_64_debug_handler
 ****************************************************************************/

static int x86_64_debug_handler(int irq, void *c, void *arg)
{
  uint8_t                cpu = this_cpu();
  union x86_64_dr6_reg_u dr6;
  int                    i;

  dr6.u64 = get_dr6();

  /* Single step execution */

  if (dr6.s.bs)
    {
      dr6.s.bs = 0;
      set_dr6(dr6.u64);

      /* Free step trigger */

      g_dbg_ctx[cpu].step.used = false;

      /* Call the step callback */

      g_dbg_ctx[cpu].step.callback(
        g_dbg_ctx[cpu].step.type,
        g_dbg_ctx[cpu].step.address,
        g_dbg_ctx[cpu].step.size,
        g_dbg_ctx[cpu].step.arg);

      return 0;
    }

  /* Get the trigger index */

  for (i = 0; i < X86_64_HWBRKP_COUNT; i++)
    {
      if (dr6.u64 & (1 << i))
        {
          /* Clear flag */

          dr6.u64 &= ~(1 << i);

          /* Call the trigger callback */

          if (g_dbg_ctx[cpu].trigger[i].callback)
            {
              g_dbg_ctx[cpu].trigger[i].callback(
                g_dbg_ctx[cpu].trigger[i].type,
                g_dbg_ctx[cpu].trigger[i].address,
                g_dbg_ctx[cpu].trigger[i].size,
                g_dbg_ctx[cpu].trigger[i].arg);
            }
        }
    }

  /* Set DR6 */

  set_dr6(dr6.u64);

  /* Set Resume flag so we don't stuck in debug handler */

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
  uint8_t cpu = this_cpu();
  uint8_t rw;
  int     i;

  /* Get type */

  switch (type)
    {
      case GDB_STOPREASON_BREAKPOINT:
      case GDB_STOPREASON_STEPPOINT:
        {
          rw = X86_64_DR7_RW_BREAK;
          break;
        }

      case GDB_STOPREASON_WATCHPOINT_WO:
        {
          rw = X86_64_DR7_RW_WATCH_W;
          break;
        }

      case GDB_STOPREASON_WATCHPOINT_RW:
        {
          rw = X86_64_DR7_RW_WATCH_RW;
          break;
        }

      default:
        {
          _err("unsupported debugpoint type %d\n", type);
          PANIC();
        }
    }

  /* Single step execution */

  if (type == GDB_STOPREASON_STEPPOINT)
    {
      if (g_dbg_ctx[cpu].step.used)
        {
          return -EBUSY;
        }

      g_dbg_ctx[cpu].step.used     = true;
      g_dbg_ctx[cpu].step.type     = type;
      g_dbg_ctx[cpu].step.address  = addr;
      g_dbg_ctx[cpu].step.size     = size;
      g_dbg_ctx[cpu].step.callback = callback;
      g_dbg_ctx[cpu].step.arg      = arg;

      /* Enable step mode */

      x86_64_debug_step(true);

      return OK;
    }

  /* Hardware triggers */

  for (i = 0; i < X86_64_HWBRKP_COUNT; i++)
    {
      if (!g_dbg_ctx[cpu].trigger[i].used)
        {
          /* Update local table */

          g_dbg_ctx[cpu].trigger[i].used     = true;
          g_dbg_ctx[cpu].trigger[i].type     = type;
          g_dbg_ctx[cpu].trigger[i].address  = addr;
          g_dbg_ctx[cpu].trigger[i].size     = size;
          g_dbg_ctx[cpu].trigger[i].callback = callback;
          g_dbg_ctx[cpu].trigger[i].arg      = arg;

          /* Configure DR */

          x86_64_set_dr(i, 1, rw, X86_64_DR7_LEN_1B, (uintptr_t)addr);

          return OK;
        }
    }

  return -ENOSPC;
}

/****************************************************************************
 * Name: up_debugpoint_remove
 ****************************************************************************/

int up_debugpoint_remove(int type, void *addr, size_t size)
{
  uint8_t cpu = this_cpu();
  int     i;

  /* Single step execution */

  if (type == GDB_STOPREASON_STEPPOINT)
    {
      g_dbg_ctx[cpu].step.used = false;

      /* Disable step mode */

      x86_64_debug_step(false);

      return OK;
    }

  /* Hardware triggers */

  for (i = 0; i < X86_64_HWBRKP_COUNT; i++)
    {
      if (g_dbg_ctx[cpu].trigger[i].address == addr)
        {
          /* Mark as free */

          g_dbg_ctx[cpu].trigger[i].used = false;

          /* Clear DR */

          x86_64_set_dr(i, 0, 0, 0, 0);

          return OK;
        }
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: x86_64_hwdebug_init
 *
 * Description:
 *   One-time initialization for hardware debug interface.
 *
 ****************************************************************************/

void x86_64_hwdebug_init(void)
{
  /* Attach debug interrupt and breakpoint interrupt */

  irq_attach(ISR1, x86_64_debug_handler, NULL);
  irq_attach(ISR3, x86_64_debug_handler, NULL);

  /* Disable all breakpoints */

  set_dr7(0);
  set_dr0(0);
  set_dr1(0);
  set_dr2(0);
  set_dr3(0);
}
