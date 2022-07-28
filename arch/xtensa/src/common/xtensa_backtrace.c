/****************************************************************************
 * arch/xtensa/src/common/xtensa_backtrace.c
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

#include <arch/irq.h>
#include <arch/xtensa/core.h>

#include "sched/sched.h"
#include "xtensa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Convert return address to a valid pc  */

#define MAKE_PC_FROM_RA(ra)   (uintptr_t *)((ra) & 0x3fffffff)

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifndef __XTENSA_CALL0_ABI__
struct xtensa_windowregs_s
{
  uint32_t windowbase;
  uint32_t windowstart;
  uint32_t areg[16];
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void inline get_window_regs(struct xtensa_windowregs_s *frame)
inline_function;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_window_regs
 *
 * Description:
 *  getfp() returns current frame pointer
 *
 ****************************************************************************/

#ifndef __XTENSA_CALL0_ABI__
static void get_window_regs(struct xtensa_windowregs_s *frame)
{
  __asm__ __volatile__("\trsr %0, WINDOWSTART\n": "=r"(frame->windowstart));
  __asm__ __volatile__("\trsr %0, WINDOWBASE\n": "=r"(frame->windowbase));

  __asm__ __volatile__("\tmov %0, a0\n": "=r"(frame->areg[0]));
  __asm__ __volatile__("\tmov %0, a1\n": "=r"(frame->areg[1]));
  __asm__ __volatile__("\tmov %0, a2\n": "=r"(frame->areg[2]));
  __asm__ __volatile__("\tmov %0, a3\n": "=r"(frame->areg[3]));
  __asm__ __volatile__("\tmov %0, a4\n": "=r"(frame->areg[4]));
  __asm__ __volatile__("\tmov %0, a5\n": "=r"(frame->areg[5]));
  __asm__ __volatile__("\tmov %0, a6\n": "=r"(frame->areg[6]));
  __asm__ __volatile__("\tmov %0, a7\n": "=r"(frame->areg[7]));
  __asm__ __volatile__("\tmov %0, a8\n": "=r"(frame->areg[8]));
  __asm__ __volatile__("\tmov %0, a9\n": "=r"(frame->areg[9]));
  __asm__ __volatile__("\tmov %0, a10\n": "=r"(frame->areg[10]));
  __asm__ __volatile__("\tmov %0, a11\n": "=r"(frame->areg[11]));
  __asm__ __volatile__("\tmov %0, a12\n": "=r"(frame->areg[12]));
  __asm__ __volatile__("\tmov %0, a13\n": "=r"(frame->areg[13]));
  __asm__ __volatile__("\tmov %0, a14\n": "=r"(frame->areg[14]));
  __asm__ __volatile__("\tmov %0, a15\n": "=r"(frame->areg[15]));
}
#endif

/****************************************************************************
 * Name: backtrace_window
 *
 * Description:
 *  backtrace_window() parsing the return address in register window
 *
 ****************************************************************************/

#ifndef __XTENSA_CALL0_ABI__
static int backtrace_window(uintptr_t *base, uintptr_t *limit,
                            struct xtensa_windowregs_s *frame,
                            void **buffer, int size, int *skip)
{
  uint32_t windowstart;
  uint32_t ra;
  uint32_t sp;
  int index;
  int i;

  /* Rotate WINDOWSTART to move the bit corresponding to
   * the current window to the bit #0
   */

  windowstart = (frame->windowstart << WSBITS | frame->windowstart) >>
        frame->windowbase;

  /* Look for bits that are set, they correspond to valid windows. */

  for (i = 0, index = WSBITS - 1; (index > 0) && (i < size); index--)
    {
      if (windowstart & (1 << index))
        {
          ra = frame->areg[index * 4];
          sp = frame->areg[index * 4 + 1];

          if (sp > (uint32_t)limit || sp < (uint32_t)base || ra == 0)
            {
              continue;
            }

          if ((*skip)-- <= 0)
            {
              buffer[i++] = MAKE_PC_FROM_RA(ra);
            }
        }
    }

  return i;
}
#endif

/****************************************************************************
 * Name: backtrace_stack
 *
 * Description:
 * backtrace_stack() parsing the return address in stack
 *
 ****************************************************************************/

static int backtrace_stack(uintptr_t *base, uintptr_t *limit,
                           uintptr_t *sp, uintptr_t *ra,
                           void **buffer, int size, int *skip)
{
  int i = 0;

  if (ra)
    {
      if ((*skip)-- <= 0)
        {
          buffer[i++] = MAKE_PC_FROM_RA((uintptr_t)ra);
        }
    }

  for (; i < size;)
    {
      ra = (uintptr_t *)*(sp - 4);
      sp = (uintptr_t *)*(sp - 3);

      if (sp >= limit || sp < base || ra == NULL)
        {
          break;
        }

      if ((*skip)-- <= 0)
        {
          buffer[i++] = MAKE_PC_FROM_RA((uintptr_t)ra);
        }
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
 * Input Parameters:
 *   tcb    - Address of the task's TCB
 *   buffer - Return address from the corresponding stack frame
 *   size   - Maximum number of addresses that can be stored in buffer
 *   skip   - number of addresses to be skipped
 *
 * Returned Value:
 *   up_backtrace() returns the number of addresses returned in buffer
 *
 ****************************************************************************/

int up_backtrace(struct tcb_s *tcb, void **buffer, int size, int skip)
{
  struct tcb_s *rtcb = running_task();
  irqstate_t flags;
  int ret;

  if (size <= 0 || !buffer)
    {
      return 0;
    }

  if (tcb == NULL || tcb == rtcb)
    {
      if (up_interrupt_context())
        {
#if CONFIG_ARCH_INTERRUPTSTACK > 15
          void *istackbase;
#ifdef CONFIG_SMP
          istackbase = xtensa_intstack_alloc();
#else
          istackbase = &g_intstackalloc;
#endif
          xtensa_window_spill();

          ret = backtrace_stack(istackbase,
                                (istackbase + CONFIG_ARCH_INTERRUPTSTACK),
                                (void *)up_getsp(), NULL,
                                buffer, size, &skip);
#else
          ret = backtrace_stack(rtcb->stack_base_ptr,
                                rtcb->stack_base_ptr + rtcb->adj_stack_size,
                                (void *)up_getsp(), NULL,
                                buffer, size, &skip);
#endif
          ret += backtrace_stack(rtcb->stack_base_ptr,
                                 rtcb->stack_base_ptr + rtcb->adj_stack_size,
                                 (void *)rtcb->xcp.regs[REG_A1],
                                 (void *)rtcb->xcp.regs[REG_A0],
                                 &buffer[ret], size - ret, &skip);
        }
      else
        {
          /* Two steps for current task:
           *
           * 1. Look through the register window for the
           * previous PCs in the call trace.
           *
           * 2. Look on the stack.
           */

#ifndef __XTENSA_CALL0_ABI__
          static struct xtensa_windowregs_s frame;

          xtensa_window_spill();

          get_window_regs(&frame);

          ret = backtrace_window(rtcb->stack_base_ptr,
                                 rtcb->stack_base_ptr + rtcb->adj_stack_size,
                                 &frame, buffer, size, &skip);
#endif
          ret += backtrace_stack(rtcb->stack_base_ptr,
                                 rtcb->stack_base_ptr + rtcb->adj_stack_size,
                                 (void *)up_getsp(), NULL,
                                 buffer, size - ret, &skip);
        }
    }
  else
    {
      /* For non-current task, only check in stack. */

      flags = enter_critical_section();

      ret = backtrace_stack(tcb->stack_base_ptr,
                            tcb->stack_base_ptr + tcb->adj_stack_size,
                            (void *)tcb->xcp.regs[REG_A1],
                            (void *)tcb->xcp.regs[REG_A0],
                            buffer, size, &skip);

      leave_critical_section(flags);
    }

  return ret;
}
