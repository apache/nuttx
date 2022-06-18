/****************************************************************************
 * arch/arm64/src/common/arm64_copystate.c
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

#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <arch/syscall.h>
#include <arch/irq.h>

#include "arm64_internal.h"

#ifdef CONFIG_ARCH_FPU
#include "arm64_fpu.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
int arch_save_fpucontext(void *saveregs)
{
  irqstate_t    flags;
  uint64_t      *p_save;

  /* Take a snapshot of the thread context right now */

  flags = enter_critical_section();

  p_save = saveregs + XCPTCONTEXT_GP_SIZE;
  arm64_fpu_save((struct fpu_reg *)p_save);
  ARM64_DSB();

  leave_critical_section(flags);
  return 0;
}
#endif

int arm64_syscall_save_context(uint64_t * regs)
{
  struct regs_context   *f_regs;
  uint64_t              *p_save;
  int                   i;

#ifdef CONFIG_ARCH_FPU
  uint64_t              *p_fpu;
  struct tcb_s          *rtcb;
  struct tcb_s          *rtcb_cur =
                           (struct tcb_s *)arch_get_current_tcb();
#endif

  DEBUGASSERT(regs);

  f_regs = (struct regs_context *)regs;
  DEBUGASSERT(f_regs->regs[REG_X1] != 0 && f_regs->regs[REG_X2] != 0);

  p_save    = (uint64_t *)f_regs->regs[REG_X2];

  for (i = 0; i < XCPTCONTEXT_GP_REGS; i++)
    {
      p_save[i] = regs[i];
    }

#ifdef CONFIG_ARCH_FPU
  rtcb      = (struct tcb_s *)f_regs->regs[REG_X1];
  p_save += XCPTCONTEXT_GP_SIZE;
  if (rtcb_cur == rtcb)
    {
      arch_save_fpucontext(p_save);
    }
  else
    {
      p_fpu = (uint64_t *)rtcb->xcp.fpu_regs;
      for (i = 0; i < XCPTCONTEXT_FPU_REGS; i++)
        {
          p_save[i] = p_fpu[i];
        }
    }
#endif

  return OK;
}
