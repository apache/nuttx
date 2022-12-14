/****************************************************************************
 * arch/arm64/src/common/arm64_assert.c
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

#include <stdio.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <debug.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "chip.h"

#ifdef CONFIG_ARCH_FPU
#include "arm64_fpu.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_dump_fatal
 ****************************************************************************/

void arm64_dump_fatal(struct regs_context *regs)
{
#ifdef CONFIG_SCHED_BACKTRACE
  struct tcb_s *rtcb = (struct tcb_s *)regs->tpidr_el1;

  /* Show back trace */

  sched_dumpstack(rtcb->pid);
#endif

  /* Dump the registers */

  arm64_registerdump(regs);
}

void up_mdelay(unsigned int milliseconds)
{
  volatile unsigned int i;
  volatile unsigned int j;

  for (i = 0; i < milliseconds; i++)
    {
      for (j = 0; j < CONFIG_BOARD_LOOPSPERMSEC; j++)
        {
        }
    }
}

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(void)
{
  struct tcb_s *rtcb = (struct tcb_s *)arch_get_current_tcb();

  /* Update the xcp context */

  if (CURRENT_REGS)
    {
      /* in interrupt */

      rtcb->xcp.regs = (uint64_t *)CURRENT_REGS;
    }
  else
    {
      up_saveusercontext(rtcb->xcp.regs);
    }

  /* Dump the registers */

  arm64_registerdump((struct regs_context *)rtcb->xcp.regs);
}
