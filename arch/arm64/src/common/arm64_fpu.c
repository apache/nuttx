/***************************************************************************
 * arch/arm64/src/common/arm64_fpu.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "sched/sched.h"
#include "arm64_arch.h"
#include "arm64_vfork.h"
#include "arm64_internal.h"
#include "arm64_fatal.h"
#include "arm64_fpu.h"

/***************************************************************************
 * Private Data
 ***************************************************************************/

static struct fpu_reg g_idle_thread_fpu[CONFIG_SMP_NCPUS];
static struct arm64_cpu_fpu_context g_cpu_fpu_ctx[CONFIG_SMP_NCPUS];

/***************************************************************************
 * Private Functions
 ***************************************************************************/

/* enable FPU access trap */

static void arm64_fpu_access_trap_enable(void)
{
  uint64_t cpacr;

  cpacr = read_sysreg(cpacr_el1);
  cpacr &= ~CPACR_EL1_FPEN_NOTRAP;
  write_sysreg(cpacr, cpacr_el1);

  ARM64_ISB();
}

/* disable FPU access trap */

static void arm64_fpu_access_trap_disable(void)
{
  uint64_t cpacr;

  cpacr = read_sysreg(cpacr_el1);
  cpacr |= CPACR_EL1_FPEN_NOTRAP;
  write_sysreg(cpacr, cpacr_el1);

  ARM64_ISB();
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

void arm64_init_fpu(struct tcb_s *tcb)
{
  struct fpu_reg *fpu_reg;

  if (tcb->pid < CONFIG_SMP_NCPUS)
    {
      memset(&g_cpu_fpu_ctx[this_cpu()], 0,
             sizeof(struct arm64_cpu_fpu_context));
      g_cpu_fpu_ctx[this_cpu()].idle_thread = tcb;

      tcb->xcp.fpu_regs = (uint64_t *)&g_idle_thread_fpu[this_cpu()];
    }

  memset(tcb->xcp.fpu_regs, 0, sizeof(struct fpu_reg));
  fpu_reg = (struct fpu_reg *)tcb->xcp.fpu_regs;
  fpu_reg->fpu_trap = 0;
}

void arm64_destory_fpu(struct tcb_s * tcb)
{
  struct tcb_s * owner;

  /* save current fpu owner's context */

  owner = g_cpu_fpu_ctx[this_cpu()].fpu_owner;

  if (owner == tcb)
    {
      g_cpu_fpu_ctx[this_cpu()].fpu_owner = NULL;
    }
}

/***************************************************************************
 * Name: arm64_fpu_enter_exception
 *
 * Description:
 *   called at every time get into a exception
 *
 ***************************************************************************/

void arm64_fpu_enter_exception(void)
{
}

void arm64_fpu_exit_exception(void)
{
}

void arm64_fpu_trap(struct regs_context * regs)
{
  struct tcb_s * owner;
  struct fpu_reg *fpu_reg;

  UNUSED(regs);

  /* disable fpu trap access */

  arm64_fpu_access_trap_disable();

  /* save current fpu owner's context */

  owner = g_cpu_fpu_ctx[this_cpu()].fpu_owner;

  if (owner != NULL)
    {
      arm64_fpu_save((struct fpu_reg *)owner->xcp.fpu_regs);
      ARM64_DSB();
      g_cpu_fpu_ctx[this_cpu()].save_count++;
      g_cpu_fpu_ctx[this_cpu()].fpu_owner = NULL;
    }

  if (arch_get_exception_depth() > 1)
    {
      /* if get_exception_depth > 1
       * it means FPU access exception occurred in exception context
       * switch FPU owner to idle thread
       */

      owner = g_cpu_fpu_ctx[this_cpu()].idle_thread;
    }
  else
    {
      owner = (struct tcb_s *)arch_get_current_tcb();
    }

  /* restore our context */

  arm64_fpu_restore((struct fpu_reg *)owner->xcp.fpu_regs);
  g_cpu_fpu_ctx[this_cpu()].restore_count++;

  /* become new owner */

  g_cpu_fpu_ctx[this_cpu()].fpu_owner   = owner;
  fpu_reg = (struct fpu_reg *)owner->xcp.fpu_regs;
  fpu_reg->fpu_trap = 0;
}

void arm64_fpu_context_restore(void)
{
  struct tcb_s *new_tcb = (struct tcb_s *)arch_get_current_tcb();
  struct fpu_reg *fpu_reg = (struct fpu_reg *)new_tcb->xcp.fpu_regs;

  arm64_fpu_access_trap_enable();

  if (fpu_reg->fpu_trap == 0)
    {
      /* FPU trap hasn't happened at this task */

      arm64_fpu_access_trap_enable();
    }
  else
    {
      /* FPU trap has happened at this task */

      if (new_tcb == g_cpu_fpu_ctx[this_cpu()].fpu_owner)
        {
          arm64_fpu_access_trap_disable();
        }
      else
        {
          arm64_fpu_access_trap_enable();
        }
    }

  g_cpu_fpu_ctx[this_cpu()].switch_count++;
}

void arm64_fpu_enable(void)
{
  irqstate_t flags = up_irq_save();

  arm64_fpu_access_trap_enable();
  up_irq_restore(flags);
}

void arm64_fpu_disable(void)
{
  irqstate_t flags = up_irq_save();

  arm64_fpu_access_trap_disable();
  up_irq_restore(flags);
}

/***************************************************************************
 * Name: up_fpucmp
 *
 * Description:
 *   Compare FPU areas from thread context.
 *
 * Input Parameters:
 *   saveregs1 - Pointer to the saved FPU registers.
 *   saveregs2 - Pointer to the saved FPU registers.
 *
 * Returned Value:
 *   True if FPU areas compare equal, False otherwise.
 *
 ***************************************************************************/

bool up_fpucmp(const void *saveregs1, const void *saveregs2)
{
  const uint64_t *regs1  = saveregs1 + XCPTCONTEXT_GP_SIZE;
  const uint64_t *regs2  = saveregs2 + XCPTCONTEXT_GP_SIZE;

  return memcmp(regs1, regs2, 8 * XCPTCONTEXT_FPU_REGS) == 0;
}
