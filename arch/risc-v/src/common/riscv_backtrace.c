/****************************************************************************
 * arch/risc-v/src/common/riscv_backtrace.c
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
#include <stdbool.h>
#include <nuttx/arch.h>
#include <nuttx/addrenv.h>
#include <nuttx/compiler.h>
#include "sched/sched.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum number of independent starting points up_backtrace() can splice
 * together.  Current worst case: the main fp chain (possibly starting on
 * the interrupt or kernel stack) plus the xcp.sregs splice used to reach
 * the user call site of a task that is mid-syscall.
 */

#define BACKTRACE_MAX_SEGMENTS 2

/* Upper bound on how many candidate ranges a single segment can
 * register: interrupt stack, kernel stack, user stack.
 */

#define BACKTRACE_MAX_RANGES 3

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One candidate stack range a segment's fp chain may be walking. */

struct backtrace_range_s
{
  uintptr_t *base;
  uintptr_t *limit;
#ifdef CONFIG_ARCH_ADDRENV
  struct addrenv_s *addrenv;
#endif
};

/* One independent fp/ra starting point for backtrace_segments() to walk,
 * together with every stack range its chain may legitimately be found on.
 */

struct backtrace_segment_s
{
  uintptr_t *fp;
  uintptr_t *ra;
  struct backtrace_range_s ranges[BACKTRACE_MAX_RANGES];
  int nranges;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static struct backtrace_segment_s *
backtrace_new_segment(struct backtrace_segment_s *segments,
                      int *nsegments, uintptr_t *fp, uintptr_t *ra);
static void backtrace_add_range(struct backtrace_segment_s *segment,
                                uintptr_t *base, uintptr_t *limit
#ifdef CONFIG_ARCH_ADDRENV
                                , struct addrenv_s *addrenv
#endif
                                );

/* BACKTRACE_ADD_RANGE() drops the trailing addrenv argument when
 * CONFIG_ARCH_ADDRENV is not configured, so call sites never need their own
 * #ifdef.
 */

#ifdef CONFIG_ARCH_ADDRENV
#  define BACKTRACE_ADD_RANGE(segment, base, limit, addrenv) \
          backtrace_add_range(segment, base, limit, addrenv)
#else
#  define BACKTRACE_ADD_RANGE(segment, base, limit, addrenv) \
          backtrace_add_range(segment, base, limit)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getfp
 *
 * Description:
 *  getfp() returns current frame pointer
 *
 ****************************************************************************/

static always_inline_function uintptr_t getfp(void)
{
  register uintptr_t fp;

  __asm__
  (
    "\tadd  %0, x0, fp\n"
    : "=r"(fp)
  );

  return fp;
}

/****************************************************************************
 * Name: backtrace_add_range
 *
 * Description:
 *  Append one candidate stack range to a segment.
 *
 ****************************************************************************/

static void backtrace_add_range(struct backtrace_segment_s *segment,
                                uintptr_t *base, uintptr_t *limit
#ifdef CONFIG_ARCH_ADDRENV
                                , struct addrenv_s *addrenv
#endif
                                )
{
  struct backtrace_range_s *range;

  DEBUGASSERT(segment->nranges < BACKTRACE_MAX_RANGES);
  range = &segment->ranges[segment->nranges++];

  range->base  = base;
  range->limit = limit;
#ifdef CONFIG_ARCH_ADDRENV
  range->addrenv = addrenv;
#endif
}

/****************************************************************************
 * Name: backtrace_new_segment
 *
 * Description:
 *  Allocate the next free segment slot and initialize its fp/ra starting
 *  point.
 *
 * Returned Value:
 *  A pointer to the newly allocated segment, ready for BACKTRACE_ADD_RANGE()
 *  calls.
 *
 ****************************************************************************/

static struct backtrace_segment_s *
backtrace_new_segment(struct backtrace_segment_s *segments,
                      int *nsegments, uintptr_t *fp, uintptr_t *ra)
{
  struct backtrace_segment_s *segment;

  segment = &segments[(*nsegments)++];

  segment->fp = fp;
  segment->ra = ra;
  segment->nranges = 0;

  return segment;
}

/****************************************************************************
 * Name: backtrace
 *
 * Description:
 *  backtrace() parsing the return address through frame pointer
 *
 ****************************************************************************/

nosanitize_address noinline_function
static int backtrace(uintptr_t *base, uintptr_t *limit,
                     uintptr_t *fp, uintptr_t *ra,
                     void **buffer, int size, int *skip,
                     uintptr_t **bridge
#ifdef CONFIG_ARCH_ADDRENV
                     , struct addrenv_s *addrenv
#endif
                     )
{
  int i = 0;
  uintptr_t *next_fp;
#ifdef CONFIG_ARCH_ADDRENV
  struct addrenv_s *oldenv;
#endif

  *bridge = NULL;

  if (ra)
    {
      if ((*skip)-- <= 0)
        {
          buffer[i++] = ra;
        }
    }

  for (; i < size && fp != NULL; fp = next_fp)
    {
      if (fp > limit || fp < base)
        {
          /* fp is outside [base, limit].  Report it back through
           * *bridge and stop; the caller decides what to do with it.
           */

          *bridge = fp;
          break;
        }

      /* fp-1/fp-2 are addresses in the *target* tcb's stack.  If that
       * tcb has its own address environment, switch to it only for these
       * two dereferences.  buffer belongs to the caller, not the target,
       * and must always be written using the caller's own address
       * environment.
       */

#ifdef CONFIG_ARCH_ADDRENV
      if (addrenv != NULL)
        {
          addrenv_select(addrenv, &oldenv);
        }
#endif

      ra = (uintptr_t *)*(fp - 1);
      next_fp = (uintptr_t *)*(fp - 2);

#ifdef CONFIG_ARCH_ADDRENV
      if (addrenv != NULL)
        {
          addrenv_restore(oldenv);
        }
#endif

      if (ra == NULL)
        {
          break;
        }

      if ((*skip)-- <= 0)
        {
          buffer[i++] = ra;
        }
    }

  return i;
}

/****************************************************************************
 * Name: backtrace_walk_segment
 *
 * Description:
 *  Walk one segment's fp chain, switching to the next candidate range
 *  whenever backtrace() bridges out of the one it was given.  Ranges are
 *  only ever crossed outward (e.g. interrupt stack -> kernel stack -> user
 *  stack, never backwards), so the search for the next range picks up from
 *  where the previous one left off instead of rescanning from the start.
 *
 ****************************************************************************/

static int backtrace_walk_segment(struct backtrace_segment_s *segment,
                                  void **buffer, int size, int *skip)
{
  uintptr_t *fp = segment->fp;
  uintptr_t *ra = segment->ra;
  uintptr_t *bridge;
  int total = 0;
  int ir = 0;

  for (; ; )
    {
      for (; ir < segment->nranges; ir++)
        {
          if (fp <= segment->ranges[ir].limit &&
              fp >= segment->ranges[ir].base)
            {
              break;
            }
        }

      if (ir == segment->nranges)
        {
          break;
        }

      total += backtrace(segment->ranges[ir].base, segment->ranges[ir].limit,
                         fp, ra, &buffer[total], size - total, skip,
                         &bridge
#ifdef CONFIG_ARCH_ADDRENV
                         , segment->ranges[ir].addrenv
#endif
                         );

      if (bridge == NULL || total >= size)
        {
          break;
        }

      fp = bridge;
      ra = NULL;
    }

  return total;
}

/****************************************************************************
 * Name: backtrace_segments
 *
 * Description:
 *  Walk a short list of stack segments in order, carrying over into the
 *  next one whenever the current segment stops producing frames before
 *  the buffer is full.
 *
 ****************************************************************************/

static int backtrace_segments(struct backtrace_segment_s *segments,
                              int nsegments, void **buffer, int size,
                              int *skip)
{
  int ret = 0;
  int i;

  for (i = 0; i < nsegments && ret < size; i++)
    {
      ret += backtrace_walk_segment(&segments[i], &buffer[ret],
                                    size - ret, skip);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_backtrace
 *
 * Description:
 *  up_backtrace()  returns  a backtrace for the TCB, in the array
 *  pointed to by buffer.  A backtrace is the series of currently active
 *  function calls for the program.  Each item in the array pointed to by
 *  buffer is of type void *, and is the return address from the
 *  corresponding stack frame.  The size argument specifies the maximum
 *  number of addresses that can be stored in buffer.   If  the backtrace is
 *  larger than size, then the addresses corresponding to the size most
 *  recent function calls are returned; to obtain the complete backtrace,
 *  make sure that buffer and size are large enough.
 *
 *  A task that is (or was, at the last trap) mid-syscall has its call
 *  chain split across two stacks: the kernel stack (from the trap that
 *  entered the syscall up to the ecall itself) and, continuing from
 *  there, the user stack (the application code that issued the ecall).
 *  Both segments are walked and spliced together into one buffer.
 *
 * Input Parameters:
 *   tcb    - Address of the task's TCB
 *   buffer - Return address from the corresponding stack frame
 *   size   - Maximum number of addresses that can be stored in buffer
 *   skip   - number of addresses to be skipped
 *
 * Returned Value:
 *   up_backtrace() returns the number of addresses returned in buffer
 *
 * Assumptions:
 *   Have to make sure tcb keep safe during function executing, it means
 *   1. Tcb have to be self or not-running.  In SMP case, the running task
 *      PC & SP cannot be backtrace, as whose get from tcb is not the newest.
 *   2. Tcb have to keep not be freed.  In task exiting case, have to
 *      make sure the tcb get from pid and up_backtrace in one critical
 *      section procedure.
 *
 ****************************************************************************/

int up_backtrace(struct tcb_s *tcb, void **buffer, int size, int skip)
{
  struct tcb_s *rtcb = running_task();
  struct backtrace_segment_s segments[BACKTRACE_MAX_SEGMENTS];
  struct backtrace_segment_s *segment;
  uintptr_t *ubase;
  uintptr_t *ulimit;
  bool self = (tcb == NULL || tcb == rtcb);
  int nsegments = 0;
#if CONFIG_ARCH_INTERRUPTSTACK > 15
  int cpu;
  uintptr_t *ilimit;
  uintptr_t *ibase;
#endif
#ifdef CONFIG_LIB_SYSCALL
  bool insyscall;
#endif

  if (size <= 0 || !buffer)
    {
      return 0;
    }

  if (self)
    {
      tcb = rtcb;
    }

  ubase  = tcb->stack_base_ptr;
  ulimit = (uintptr_t *)((uintptr_t)tcb->stack_base_ptr +
                         tcb->adj_stack_size);

#ifdef CONFIG_LIB_SYSCALL
  insyscall = (tcb->flags & TCB_FLAG_SYSCALL) != 0;
#endif

  /* The main fp chain may start on (and naturally unwind through) any
   * stack the tcb was last executing on, innermost first: the interrupt
   * stack, then the kernel stack, then finally the user stack.  Every
   * range it could legitimately cross into is listed up front instead of
   * picking just one ahead of time, since the trap entry copies the
   * pre-trap fp/ra into each synthetic frame, letting the chain unwind
   * straight through a trap boundary onto the next stack out.
   */

  segment = backtrace_new_segment(segments, &nsegments,
                                  self ? (uintptr_t *)getfp() :
                                  (uintptr_t *)tcb->xcp.regs[REG_FP],
                                  self ? NULL :
                                  (uintptr_t *)tcb->xcp.regs[REG_EPC]);

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  if (self)
    {
      /* Only a self chain can genuinely start on the interrupt stack: it
       * is per-CPU scratch space, reused on every dispatch, so a snapshot
       * taken from a not-running tcb can never legitimately point into
       * it.
       */

      cpu = up_cpu_index();
      ilimit = (uintptr_t *)((uintptr_t)g_intstacktop -
                             (CONFIG_ARCH_INTERRUPTSTACK * cpu));
      ibase = (uintptr_t *)((uintptr_t)ilimit - CONFIG_ARCH_INTERRUPTSTACK);

      BACKTRACE_ADD_RANGE(segment, ibase, ilimit, NULL);
    }
#endif

#if defined(CONFIG_ARCH_KERNEL_STACK) && defined(CONFIG_ARCH_ADDRENV)
  if (tcb->xcp.kstack != NULL)
    {
      BACKTRACE_ADD_RANGE(segment, tcb->xcp.kstack,
                         (uintptr_t *)((uintptr_t)tcb->xcp.kstack +
                                       ARCH_KERNEL_STACKSIZE), NULL);
    }
#endif

  BACKTRACE_ADD_RANGE(segment, ubase, ulimit,
                      self ? NULL : tcb->addrenv_own);

#ifdef CONFIG_LIB_SYSCALL
  /* The main chain above stops at the nearest ecall boundary --
   * dispatch_syscall()'s own call frame has no relation to the user fp
   * chain, so there is nothing for it to unwind into.  xcp.sregs is
   * stable (written once by dispatch_syscall(), never by reserved
   * syscalls) and gives the real user call site, so splice it in as an
   * independent starting point.
   */

  if (insyscall)
    {
      segment = backtrace_new_segment(segments, &nsegments,
                                      (uintptr_t *)tcb->xcp.sregs[REG_FP],
                                      (uintptr_t *)tcb->xcp.sregs[REG_EPC]);

      BACKTRACE_ADD_RANGE(segment, ubase, ulimit,
                          self ? NULL : tcb->addrenv_own);
    }
#endif

  return backtrace_segments(segments, nsegments, buffer, size, &skip);
}
