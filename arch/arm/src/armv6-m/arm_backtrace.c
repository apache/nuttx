/****************************************************************************
 * arch/arm/src/armv6-m/arm_backtrace.c
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

#include "sched/sched.h"

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macro and definitions for simple decoding of instuctions.
 * To check an instruction, it is ANDed with the IMASK_ and
 * the result is compared with the IOP_. The macro INSTR_IS
 * does this and returns !0 to indicate a match.
 */

#define INSTR_IS(i, o)      (((i) & (IMASK_##o)) == (IOP_##o))

#define IMASK_T_BLX         0xff80      /* blx */
#define IOP_T_BLX           0x4780

#define IMASK_T_BL          0xf800      /* blx */
#define IOP_T_BL            0xf000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR void **g_backtrace_code_regions;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: in_code_region
 *
 * Description:
 *  in_code_region()  check if the program counter is in the program
 *  section, program counter should always be within the view of executable
 *  sections.
 *
 * Input Parameters:
 *   pc    - Program counter address
 *
 * Returned Value:
 *   A boolean value: true the counter is vaild
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN
__attribute__((no_sanitize_address))
#endif
static bool in_code_region(FAR void *pc)
{
  int i = 0;

  if ((uintptr_t)pc >= (uintptr_t)_START_TEXT &&
      (uintptr_t)pc <  (uintptr_t)_END_TEXT)
    {
      return true;
    }

  if (g_backtrace_code_regions)
    {
      while (g_backtrace_code_regions[i] &&
             (g_backtrace_code_regions[i] !=
              g_backtrace_code_regions[i + 1]))
        {
          if (g_backtrace_code_regions[i] <= pc &&
              g_backtrace_code_regions[i + 1] > pc)
            {
              return true;
            }

          i += 2;
        }
    }

  return false;
}

/****************************************************************************
 * Name: backtrace_branch
 *
 * Description:
 *  backtrace() parsing the return address through branch instruction
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN
__attribute__((no_sanitize_address))
#endif
static int backtrace_branch(FAR void *limit, FAR void *sp,
                            FAR void **buffer, int size)
{
  uint16_t ins16;
  uint32_t addr;
  int i = 0;

  for (; i < size && sp < limit; sp += sizeof(uint32_t))
    {
      addr = *(FAR uint32_t *)sp;
      if (!in_code_region(addr))
        {
          continue;
        }

      addr = (addr & ~1) - 2;
      ins16 = *(FAR uint16_t *)addr;
      if (INSTR_IS(ins16, T_BLX))
        {
          buffer[i++] = addr;
        }

      /* BL Instruction
       * OFFSET: 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
       * VALUE :  1  1  1  1  0  -  -  -  -  -  -  -  -  -  -  -
       * OFFSET: 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
       * VALUE :  1  1  -  1  -  -  -  -  -  -  -  -  -  -  -  -
       */

      else if ((ins16 & 0xd000) == 0xd000)
        {
          addr -= 2;
          ins16 = *(FAR uint16_t *)addr;
          if (INSTR_IS(ins16, T_BL))
            {
              buffer[i++] = addr;
            }
        }
    }

  return i;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_backtrace_init_code_regions
 *
 * Description:
 *  The up call arm_backtrace_init_code_regions() will set the start
 *  and end addresses of the customized program sections, this method
 *  will help the different boards to configure the current text
 *  sections for some complicate platfroms
 *
 * Input Parameters:
 *   regions  The start and end address of the text segment
 *            This interface supports the input of multiple
 *            groups of sections, Each set of the sections
 *            must be a pair, the end of the area must specify
 *            two NULL porint, e.g :
 *
 *            static void *g_code_regions[] =
 *              {
 *                _START_TEXT,  _END_TEXT,
 *                _START2_TEXT, _END2_TEXT,
 *                _START3_TEXT, _END3_TEXT,
 *                NULL,         NULL,
 *              };
 *
 *              arm_backtrace_init_code_regions(g_code_regions);
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN
__attribute__((no_sanitize_address))
#endif
void arm_backtrace_init_code_regions(FAR void **regions)
{
  g_backtrace_code_regions = regions;
}

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
 *
 * Returned Value:
 *   up_backtrace() returns the number of addresses returned in buffer
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN
__attribute__((no_sanitize_address))
#endif
int up_backtrace(FAR struct tcb_s *tcb, FAR void **buffer, int size)
{
  FAR struct tcb_s *rtcb = running_task();
  irqstate_t flags;
  FAR void *sp;
  int ret;

  if (size <= 0 || !buffer)
    {
      return 0;
    }

  if (tcb == NULL)
    {
      tcb = rtcb;
    }

  if (tcb == rtcb)
    {
      sp = (FAR void *)up_getsp();
      if (up_interrupt_context())
        {
#if CONFIG_ARCH_INTERRUPTSTACK > 7
          ret = backtrace_branch(
#  ifdef CONFIG_SMP
                                 (uint32_t)arm_intstack_alloc()
#  else
                                 (uint32_t)&g_intstackalloc
#  endif
                                 + (CONFIG_ARCH_INTERRUPTSTACK & ~7), sp,
                                 buffer, size);
          if (ret < size)
            {
              sp = (FAR void *)CURRENT_REGS[REG_SP];
              ret += backtrace_branch(rtcb->stack_base_ptr +
                                      rtcb->adj_stack_size, sp,
                                      &buffer[ret], size - ret);
            }
#else
          sp = (FAR void *)CURRENT_REGS[REG_SP];
          ret = backtrace_branch(rtcb->stack_base_ptr +
                                 rtcb->adj_stack_size, sp,
                                 buffer, size);
#endif
        }
      else
        {
          ret = backtrace_branch(rtcb->stack_base_ptr +
                                 rtcb->adj_stack_size, sp,
                                 buffer, size);
        }
    }
  else
    {
      flags = enter_critical_section();

      ret = backtrace_branch(tcb->stack_base_ptr +
                             tcb->adj_stack_size,
                             (FAR void *)tcb->xcp.regs[REG_SP],
                             buffer, size);

      leave_critical_section(flags);
    }

  return ret;
}
