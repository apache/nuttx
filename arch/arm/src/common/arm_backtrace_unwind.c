/****************************************************************************
 * arch/arm/src/common/arm_backtrace_unwind.c
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

#include <arch/elf.h>

#include "sched/sched.h"
#include "arm_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum regs
{
#ifdef CONFIG_ARM_THUMB
  FP = 7,
#else
  FP = 11,
#endif /* CONFIG_ARM_THUMB */
  SP = 13,
  LR = 14,
  PC = 15
};

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct unwind_frame_s
{
  unsigned long fp;
  unsigned long sp;
  unsigned long lr;
  unsigned long pc;

  /* Address of the LR value on the stack */

  unsigned long *lr_addr;

  /* Highest value of sp allowed */

  unsigned long stack_top;
};

struct unwind_ctrl_s
{
  unsigned long        vrs[16];   /* Virtual register set */
  const unsigned long *insn;      /* Pointer to the current instructions word */
  unsigned long        stack_top; /* Highest value of sp allowed */
  unsigned long       *lr_addr;   /* Address of LR value on the stack */
  int                  entries;   /* Number of entries left to interpret */
  int                  byte;      /* Current byte number in the instructions word */

  /* 1 : Check for stack overflow for each register pop.
   * 0 : Save overhead if there is plenty of stack remaining.
   */

  int check_each_pop;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Convert a prel31 symbol to an absolute address */

#define prel31_to_addr(ptr)  \
  ({  \
  /* Sign-extend to 32 bits */  \
  long offset = (((long)*(ptr)) << 1) >> 1;  \
  (unsigned long)(ptr) + offset;  \
  })

/****************************************************************************
 * Name: search_index
 *
 * Description:
 *  Binary search in the unwind index. The entries are
 *  guaranteed to be sorted in ascending order by the linker.
 *
 *  start    = first entry
 *  origin   = first entry with positive offset
 *             (or stop if there is no such entry)
 *  stop - 1 = last entry
 *
 ****************************************************************************/

static const struct __EIT_entry *
search_index(unsigned long addr, const struct __EIT_entry *start,
             const struct __EIT_entry *origin,
             const struct __EIT_entry *stop)
{
  unsigned long addr_prel31;

  /* Only search in the section with the matching sign. This way the
   * prel31 numbers can be compared as unsigned longs.
   */

  if (addr < (unsigned long)start)
    {
      /* Negative offsets: [start; origin) */

      stop = origin;
    }
  else
    {
      /* Positive offsets: [origin; stop) */

      start = origin;
    }

  /* Prel31 for address relavive to start */

  addr_prel31 = (addr - (unsigned long)start) & 0x7fffffff;

  while (start < stop - 1)
    {
      const struct __EIT_entry *mid = start + ((stop - start) >> 1);

      /* As addr_prel31 is relative to start an offset is needed to
       * make it relative to mid.
       */

      if (addr_prel31 -
          ((unsigned long)mid - (unsigned long)start) < mid->fnoffset)
        {
          stop = mid;
        }
      else
        {
          /* Keep addr_prel31 relative to start */

          addr_prel31 -= ((unsigned long)mid - (unsigned long)start);
          start = mid;
        }
    }

  return (start->fnoffset <= addr_prel31) ? start : NULL;
}

static const struct __EIT_entry *
unwind_find_origin(const struct __EIT_entry *start,
                   const struct __EIT_entry *stop)
{
  const struct __EIT_entry *mid;

  while (start < stop)
    {
      mid = start + ((stop - start) >> 1);

      if (mid->fnoffset >= 0x40000000)
        {
          /* Negative offset */

          start = mid + 1;
        }
      else
        {
          /* Positive offset */

          stop = mid;
        }
    }

  return stop;
}

static const struct __EIT_entry *unwind_find_entry(unsigned long addr)
{
  /* Main unwind table */

  return search_index(addr, __exidx_start,
                      unwind_find_origin(__exidx_start, __exidx_end),
                      __exidx_end);
}

static unsigned long unwind_get_byte(struct unwind_ctrl_s *ctrl)
{
  unsigned long ret;

  if (ctrl->entries <= 0)
    {
      return 0;
    }

  ret = (*ctrl->insn >> (ctrl->byte * 8)) & 0xff;

  if (ctrl->byte == 0)
    {
      ctrl->insn++;
      ctrl->entries--;
      ctrl->byte = 3;
    }
  else
    {
      ctrl->byte--;
    }

  return ret;
}

/****************************************************************************
 * Name: unwind_pop_register
 *
 * Description:
 *  Before poping a register check whether it is feasible or not
 *
 ****************************************************************************/

static int unwind_pop_register(struct unwind_ctrl_s *ctrl,
                               unsigned long **vsp, unsigned int reg)
{
  if (ctrl->check_each_pop)
    {
      if (*vsp >= (unsigned long *)ctrl->stack_top)
        {
          return -1;
        }
    }

  ctrl->vrs[reg] = *(*vsp);
  if (reg == LR)
    {
      ctrl->lr_addr = *vsp;
    }

  (*vsp)++;

  return 0;
}

/****************************************************************************
 * Name: unwind_pop_register
 *
 * Description:
 *  Helper functions to execute the instructions
 *
 ****************************************************************************/

static int unwind_exec_pop_subset_r4_to_r13(struct unwind_ctrl_s *ctrl,
                                            unsigned long mask)
{
  unsigned long *vsp = (unsigned long *)ctrl->vrs[SP];
  int load_sp;
  int reg = 4;

  load_sp = mask & (1 << (13 - 4));
  while (mask)
    {
      if ((mask & 1) && unwind_pop_register(ctrl, &vsp, reg))
        {
          return -1;
        }

      mask >>= 1;
      reg++;
    }

  if (!load_sp)
    {
      ctrl->vrs[SP] = (unsigned long)vsp;
    }

  return 0;
}

static int unwind_exec_pop_r4_to_rn(struct unwind_ctrl_s *ctrl,
                                    unsigned long content)
{
  unsigned long *vsp = (unsigned long *)ctrl->vrs[SP];
  int reg;

  /* Pop R4-R[4+bbb] */

  for (reg = 4; reg <= 4 + (content & 7); reg++)
    {
      if (unwind_pop_register(ctrl, &vsp, reg))
        {
          return -1;
        }
    }

  if ((content & 0x8) && unwind_pop_register(ctrl, &vsp, 14))
    {
      return -1;
    }

  ctrl->vrs[SP] = (unsigned long)vsp;

  return 0;
}

static int unwind_exec_pop_subset_r0_to_r3(struct unwind_ctrl_s *ctrl,
                                           unsigned long mask)
{
  unsigned long *vsp = (unsigned long *)ctrl->vrs[SP];
  int reg = 0;

  /* Pop R0-R3 according to mask */

  while (mask)
    {
      if ((mask & 1) && unwind_pop_register(ctrl, &vsp, reg))
        {
          return -1;
        }

      mask >>= 1;
      reg++;
    }

  ctrl->vrs[SP] = (unsigned long)vsp;

  return 0;
}

/****************************************************************************
 * Name: unwind_pop_register
 *
 * Description:
 *  Execute the current unwind instruction
 *
 ****************************************************************************/

static int unwind_exec_content(struct unwind_ctrl_s *ctrl)
{
  unsigned long content = unwind_get_byte(ctrl);
  int ret = 0;

  if ((content & 0xc0) == 0x00)
    {
      ctrl->vrs[SP] += ((content & 0x3f) << 2) + 4;
    }
  else if ((content & 0xc0) == 0x40)
    {
      ctrl->vrs[SP] -= ((content & 0x3f) << 2) + 4;
    }
  else if ((content & 0xf0) == 0x80)
    {
      unsigned long mask;

      content = (content << 8) | unwind_get_byte(ctrl);
      mask = content & 0x0fff;
      ret = (mask == 0) ? -1 :
            unwind_exec_pop_subset_r4_to_r13(ctrl, mask);
    }
  else if ((content & 0xf0) == 0x90 &&
           (content & 0x0d) != 0x0d)
    {
      ctrl->vrs[SP] = ctrl->vrs[content & 0x0f];
    }
  else if ((content & 0xf0) == 0xa0)
    {
      ret = unwind_exec_pop_r4_to_rn(ctrl, content);
    }
  else if (content == 0xb0)
    {
      if (ctrl->vrs[PC] == 0)
        {
          ctrl->vrs[PC] = ctrl->vrs[LR];
        }

      /* No further processing */

      ctrl->entries = 0;
    }
  else if (content == 0xb1)
    {
      unsigned long mask = unwind_get_byte(ctrl);

      if (mask == 0 || mask & 0xf0)
        {
          ret = -1;
        }
      else
        {
          ret = unwind_exec_pop_subset_r0_to_r3(ctrl, mask);
        }
    }
  else if (content == 0xb2)
    {
      unsigned long uleb128 = unwind_get_byte(ctrl);

      ctrl->vrs[SP] += 0x204 + (uleb128 << 2);
    }
  else
    {
      ret = -1;
    }

  return ret;
}

int unwind_frame(struct unwind_frame_s *frame)
{
  const struct __EIT_entry *entry;
  struct unwind_ctrl_s ctrl;

  entry = unwind_find_entry(frame->pc);
  if (!entry || entry->content == 1)
    {
      return -1;
    }

  ctrl.vrs[FP] = frame->fp;
  ctrl.vrs[SP] = frame->sp;
  ctrl.vrs[LR] = frame->lr;
  ctrl.vrs[PC] = 0;
  ctrl.stack_top = frame->stack_top;

  if (frame->pc == prel31_to_addr(&entry->fnoffset))
    {
      /* Unwinding is tricky when we're halfway through the prologue,
       * since the stack frame that the unwinder expects may not be
       * fully set up yet. However, one thing we do know for sure is
       * that if we are unwinding from the very first instruction of
       * a function, we are still effectively in the stack frame of
       * the caller, and the unwind info has no relevance yet.
       */

      if (frame->pc == frame->lr)
        {
          return -1;
        }

      frame->pc = frame->lr;

      return 0;
    }
  else if ((entry->content & 0x80000000) == 0)
    {
      /* Prel31 to the unwind table */

      ctrl.insn = (unsigned long *)prel31_to_addr(&entry->content);
    }
  else if ((entry->content & 0xff000000) == 0x80000000)
    {
      /* Only personality routine 0 supported in the index */

      ctrl.insn = &entry->content;
    }
  else
    {
      return -1;
    }

  /* Check the personality routine */

  if ((*ctrl.insn & 0xff000000) == 0x80000000)
    {
      ctrl.byte = 2;
      ctrl.entries = 1;
    }
  else if ((*ctrl.insn & 0xff000000) == 0x81000000)
    {
      ctrl.byte = 1;
      ctrl.entries = 1 + ((*ctrl.insn & 0x00ff0000) >> 16);
    }
  else
    {
      return -1;
    }

  ctrl.check_each_pop = 0;

  while (ctrl.entries > 0)
    {
      int urc;

      if ((ctrl.stack_top - ctrl.vrs[SP]) < sizeof(ctrl.vrs))
        {
          ctrl.check_each_pop = 1;
        }

      urc = unwind_exec_content(&ctrl);
      if (urc < 0)
        {
          return urc;
        }

      if (ctrl.vrs[SP] < frame->sp ||
          ctrl.vrs[SP] > ctrl.stack_top)
        {
          return -1;
        }
    }

  if (ctrl.vrs[PC] == 0)
    {
      ctrl.vrs[PC] = ctrl.vrs[LR];
    }

  /* Check for infinite loop */

  if (frame->pc == ctrl.vrs[PC] && frame->sp == ctrl.vrs[SP])
    {
      return -1;
    }

  frame->fp = ctrl.vrs[FP];
  frame->sp = ctrl.vrs[SP];
  frame->lr = ctrl.vrs[LR];
  frame->pc = ctrl.vrs[PC];
  frame->lr_addr = ctrl.lr_addr;

  return 0;
}

nosanitize_address
static int backtrace_unwind(struct unwind_frame_s *frame,
                            void **buffer, int size, int *skip)
{
  int cnt = 0;

  if (frame->pc && cnt < size && (*skip)-- <= 0)
    {
      buffer[cnt++] = (void *)((frame->pc & ~1) - 2);
    }

  if (frame->lr && cnt < size && (*skip)-- <= 0)
    {
      buffer[cnt++] = (void *)((frame->lr & ~1) - 2);
    }

  while (cnt < size)
    {
      if (unwind_frame(frame) < 0 || frame->pc == 0)
        {
          break;
        }

      if ((*skip)-- <= 0)
        {
          frame->pc = (frame->pc & ~1) - 2;

          if (cnt == 0 || (void *)frame->pc != buffer[cnt - 1])
            {
              buffer[cnt++] = (void *)frame->pc;
            }
        }
    }

  return cnt > 0 ? cnt : 0;
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

int up_backtrace(struct tcb_s *tcb,
                 void **buffer, int size, int skip)
{
  struct tcb_s *rtcb = running_task();
  struct unwind_frame_s frame;
  irqstate_t flags;
  int ret;

  if (size <= 0 || !buffer)
    {
      return 0;
    }

  if (tcb == NULL || tcb == rtcb)
    {
      frame.fp = (unsigned long)__builtin_frame_address(0);
      frame.lr = (unsigned long)__builtin_return_address(0);
      frame.pc = (unsigned long)&up_backtrace;
      frame.sp = frame.fp;
      frame.stack_top = (unsigned long)rtcb->stack_base_ptr +
                                       rtcb->adj_stack_size;
      if (up_interrupt_context())
        {
#if CONFIG_ARCH_INTERRUPTSTACK > 7
#  ifdef CONFIG_SMP
          frame.stack_top = arm_intstack_top();
#  else
          frame.stack_top = &g_intstacktop;
#  endif /* CONFIG_SMP */
#endif /* CONFIG_ARCH_INTERRUPTSTACK > 7 */

          ret = backtrace_unwind(&frame, buffer, size, &skip);
          if (ret < size)
            {
              frame.fp = rtcb->xcp.regs[REG_FP];
              frame.sp = rtcb->xcp.regs[REG_SP];
              frame.pc = rtcb->xcp.regs[REG_PC];
              frame.lr = 0;
              frame.stack_top = (unsigned long)rtcb->stack_base_ptr +
                                               rtcb->adj_stack_size;
              ret += backtrace_unwind(&frame, &buffer[ret],
                                      size - ret, &skip);
            }
        }
      else
        {
          ret = backtrace_unwind(&frame, buffer, size, &skip);
        }
    }
  else
    {
      flags = enter_critical_section();

      frame.fp = tcb->xcp.regs[REG_FP];
      frame.sp = tcb->xcp.regs[REG_SP];
      frame.lr = tcb->xcp.regs[REG_LR];
      frame.pc = tcb->xcp.regs[REG_PC];
      frame.stack_top = (unsigned long)tcb->stack_base_ptr +
                                       tcb->adj_stack_size;

      ret = backtrace_unwind(&frame, buffer, size, &skip);

      leave_critical_section(flags);
    }

  return ret;
}
