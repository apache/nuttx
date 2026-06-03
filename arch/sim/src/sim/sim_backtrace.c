/****************************************************************************
 * arch/sim/src/sim/sim_backtrace.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <sched/sched.h>

#include <setjmp.h>
#include <stdint.h>
#include <string.h>

#include "sim_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: backtrace_fp
 *
 * Description:
 *   Walk the frame-pointer chain to recover return addresses for a task
 *   that is not currently running on the host thread.  This relies on the
 *   uniform frame layout used by every host ABI sim supports (x86, x86_64,
 *   ARM, ARM64):
 *     [fp + 0] = previous fp
 *     [fp + 1] = return address
 *   where one slot is sizeof(uintptr_t).
 *
 *   Because sim's setjmp implementation is provided by NuttX itself
 *   (libs/libc/machine/sim/arch_setjmp_*.S) rather than by host libc, the
 *   saved fp/sp/pc in tcb->xcp.regs are plain pointers and contain no
 *   pointer-mangling, so the chain can be followed directly on Linux,
 *   macOS and Windows hosts.
 *
 ****************************************************************************/

nosanitize_address
static int backtrace_fp(uintptr_t *base, uintptr_t *limit,
                        uintptr_t *fp, uintptr_t *pc,
                        void **buffer, int size, int *skip)
{
  int i = 0;

  if (pc != NULL)
    {
      if ((*skip)-- <= 0)
        {
          buffer[i++] = pc;
        }
    }

  while (i < size)
    {
      /* Validate fp lies within [base, limit) and is properly aligned. */

      if (fp < base || fp >= limit ||
          ((uintptr_t)fp & (sizeof(uintptr_t) - 1)) != 0)
        {
          break;
        }

      if (fp[0] == 0)
        {
          break;
        }

      if ((*skip)-- <= 0)
        {
          buffer[i++] = (void *)fp[1];
        }

      fp = (uintptr_t *)fp[0];
    }

  return i;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_backtrace
 *
 * Description:
 *   up_backtrace() returns a backtrace for the TCB, in the array pointed to
 *   by buffer.  A backtrace is the series of currently active function
 *   calls for the program.  Each item in the array pointed to by buffer is
 *   of type void *, and is the return address from the corresponding stack
 *   frame.
 *
 *   For the running task we keep using host_backtrace() (which wraps
 *   glibc's backtrace()) so that we benefit from DWARF unwinding through
 *   host libraries.  For any other task host_backtrace() cannot help -- it
 *   only knows how to walk the host thread that is calling it -- so we
 *   instead walk the frame-pointer chain starting from the registers saved
 *   in tcb->xcp.regs by setjmp() at the last context switch.  This requires
 *   CONFIG_FRAME_POINTER=y so that the compiler emits a usable fp link.
 *
 ****************************************************************************/

nosanitize_address
int up_backtrace(struct tcb_s *tcb, void **buffer, int size, int skip)
{
  struct tcb_s *rtcb = running_task();
  irqstate_t flags;
  int ret;

  if (size <= 0 || buffer == NULL)
    {
      return 0;
    }

  if (tcb == NULL)
    {
      tcb = rtcb;
    }

  if (tcb == rtcb)
    {
      void *buf[skip + size];

      ret = host_backtrace(buf, skip + size);
      if (ret <= skip)
        {
          return ret < 0 ? ret : 0;
        }

      ret -= skip;
      memcpy(buffer, &buf[skip], ret * sizeof(void *));
      return ret;
    }

  flags = enter_critical_section();

  ret = backtrace_fp((uintptr_t *)tcb->stack_base_ptr,
                     (uintptr_t *)((uintptr_t)tcb->stack_base_ptr +
                                   tcb->adj_stack_size),
                     (uintptr_t *)tcb->xcp.regs[JB_FP],
                     (uintptr_t *)tcb->xcp.regs[JB_PC],
                     buffer, size, &skip);

  leave_critical_section(flags);

  return ret;
}
