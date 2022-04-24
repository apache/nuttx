/****************************************************************************
 * arch/arm/src/tlsr82/tc32/tc32_backtrace.c
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
#include <nuttx/allsyms.h>

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

#define IMASK_T_PUSH_LO     0xff00      /* push {reglist} (not LR) */
#define IOP_T_PUSH_LO       0x6400

#define IMASK_T_PUSH        0xff00      /* push {reglist} (inc LR) */
#define IOP_T_PUSH          0x6500

#define IMASK_T_SUB_SP_16   0xff80      /* sub sp, # */
#define IOP_T_SUB_SP_16     0x6080

#define IMASK_T_BL          0xf000      /* bl */
#define IOP_T_BL            0x9000

#define INSTR_LIMIT         0x2000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void **g_backtrace_code_regions;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getlroffset
 *
 * Description:
 *  getlroffset()  returns the currect link address offset.
 *
 * Input Parameters:
 *   lr    - Link register address
 *
 * Returned Value:
 *   Link address offset, 0 is returned if the lr is invalid.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN
__attribute__((no_sanitize_address))
#endif
static int getlroffset(uint8_t *lr)
{
  lr = (uint8_t *)((uintptr_t)lr & 0xfffffffe);

  if (((uintptr_t)lr & 0xffffffe0) == 0xffffffe0)
    {
      return 0;
    }

  /* Always return 4, only tjl (32bit instruction) jump with lr in tc32
   * return (*(uint16_t *)(lr - 4) & 0xf000) == 0xf000 ? 5 : 3;
   */

  return 4;
}

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
static bool in_code_region(void *pc)
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
 * Name: backtrace_push_internal
 *
 * Description:
 *  backtrace_push_internal()  returns the currect link address from
 *  program counter and stack pointer
 *
 * Input Parameters:
 *   psp    - Double poninter to the SP, this parameter will be changed if
 *            the corresponding LR address is successfully found.
 *   ppc    - Double poninter to the PC, this parameter will be changed if
 *            the corresponding LR address is successfully found.
 *
 * Returned Value:
 *   Link address should be returned if successful
 *   Otherwise, NULL is returned
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN
__attribute__((no_sanitize_address))
#endif
static void *backtrace_push_internal(void **psp, void **ppc)
{
  uint8_t *sp = *psp;
  uint8_t *pc = *ppc;
  uint8_t *base;
  uint8_t *lr;
  uint16_t ins16;
  int offset = 1;
  bool found;
  int frame;
  int i;
#ifdef CONFIG_ALLSYMS
  const struct symtab_s *symbol;
  size_t symbolsize;

  symbol = allsyms_findbyvalue(pc, &symbolsize);
  if (!symbol || !in_code_region(symbol->sym_value))
    {
      return NULL;
    }
#endif

  found = false;

  for (i = 0; i < INSTR_LIMIT; i += 2)
    {
      base  = pc - i;
      ins16 = *(uint16_t *)(base);
      if (INSTR_IS(ins16, T_PUSH))
        {
          /* Bit 1 number in low byte indicates the number of pushed
           * low register, +1 for LR is alao pushed into the stack.
           */

          frame = __builtin_popcount(ins16 & 0xff) + 1;
          ins16 = *(uint16_t *)(base - 2);
          if (INSTR_IS(ins16, T_PUSH_LO))
            {
              offset += __builtin_popcount(ins16 & 0xff);
              frame  += offset - 1;
            }

          found = true;
        }

      if (found)
        {
#ifdef CONFIG_ALLSYMS
          if (base >= (uint8_t *)symbol->sym_value &&
              base - (uint8_t *)symbol->sym_value < 8)
            {
              break;
            }
          else
            {
              found = false;
            }
#else
          break;
#endif
        }
    }

  if (!found)
    {
      return NULL;
    }

  i = 0;

  while (base + i < pc)
    {
      ins16 = *(uint16_t *)(base + i);
      if (INSTR_IS(ins16, T_SUB_SP_16))
        {
          /* Low 7 bit indicates the number sub to sp */

          frame += (ins16 & 0x7f);
          break;
        }

      if (INSTR_IS(ins16, T_PUSH_LO))
        {
          /* Bit 1 number in low 8bit indicated the number of pushed
           * low register.
           */

          frame += __builtin_popcount(ins16 & 0xff);
        }

      /* 16bit instruction */

      i += 2;
    }

  lr = (uint8_t *)*((uint32_t *)sp + frame - offset);
  if (!in_code_region(lr))
    {
      return NULL;
    }

  offset = getlroffset(lr);
  if (offset == 0)
    {
      return NULL;
    }

  *psp = (uint32_t *)sp + frame;

  return lr - offset;
}

/****************************************************************************
 * Name: backtrace_push
 *
 * Description:
 *  backtrace_push() parsing the return address through instruction
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN
__attribute__((no_sanitize_address))
#endif
static int backtrace_push(void *limit, void **sp, void *pc,
                          void **buffer, int size, int *skip)
{
  int i = 0;

  if (!in_code_region(pc))
    {
      return 0;
    }

  pc = (void *)((uintptr_t)pc & 0xfffffffe);

  if ((*skip)-- <= 0)
    {
      buffer[i++] = pc;
    }

  while (i < size)
    {
      if (*sp >= limit)
        {
          break;
        }

      pc = backtrace_push_internal(sp, &pc);
      if (!pc)
        {
          break;
        }

      if ((*skip)-- <= 0)
        {
          buffer[i++] = pc;
        }
    }

  return i;
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
static int backtrace_branch(void *limit, void *sp,
                            void **buffer, int size, int *skip)
{
#if 1
  uint16_t ins16;
  uint32_t addr;
  int i;

  for (i = 0; i < size && sp < limit; sp += sizeof(uint32_t))
    {
      addr = *(uint32_t *)sp;
      if (!in_code_region((void *)addr))
        {
          continue;
        }

      addr = (addr & ~1) - 2;
      ins16 = *(uint16_t *)addr;

      /* TJL Instruction
       * OFFSET: 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
       * VALUE :  1  0  0  1  -  -  -  -  -  -  -  -  -  -  -  -
       * OFFSET: 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
       * VALUE :  1  0  0  1  1  -  -  -  -  -  -  -  -  -  -  -
       */

      if ((ins16 & 0x9800) == 0x9800)
        {
          addr -= 2;
          ins16 = *(uint16_t *)addr;
          if (INSTR_IS(ins16, T_BL))
            {
              if ((*skip)-- <= 0)
                {
                  buffer[i++] = (void *)addr;
                }
            }
        }
    }

  return i;
#else
  /* Not implement backtrace branch method temporarily
   * Wait for the instruction tjl encode document from telink.
   */

  return 0;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_backtrace_init_code_regions
 *
 * Description:
 *  The up call up_backtrace_init_code_regions() will set the start
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
 *              up_backtrace_init_code_regions(g_code_regions);
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN
__attribute__((no_sanitize_address))
#endif
void up_backtrace_init_code_regions(void **regions)
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
int up_backtrace(struct tcb_s *tcb, void **buffer, int size, int skip)
{
  struct tcb_s *rtcb = running_task();
  irqstate_t flags;
  void *sp;
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
      sp = (void *)up_getsp();

      if (up_interrupt_context())
        {
#if CONFIG_ARCH_INTERRUPTSTACK > 7
          ret = backtrace_push(
#  ifdef CONFIG_SMP
                               arm_intstack_top(),
#  else
                               &g_intstacktop,
#  endif /* CONFIG_SMP */
                               &sp, (void *)up_backtrace + 10,
                               buffer, size, &skip);
#else
          ret = backtrace_push(rtcb->stack_base_ptr +
                               rtcb->adj_stack_size,
                               &sp, (void *)up_backtrace + 10,
                               buffer, size, &skip);
#endif
          if (ret < size)
            {
              sp = (void *)CURRENT_REGS[REG_SP];
              ret += backtrace_push(rtcb->stack_base_ptr +
                                    rtcb->adj_stack_size, &sp,
                                    (void *)CURRENT_REGS[REG_PC],
                                    &buffer[ret], size - ret, &skip);
            }
        }
      else
        {
          ret = backtrace_push(rtcb->stack_base_ptr +
                               rtcb->adj_stack_size, &sp,
                               (void *)up_backtrace + 10,
                               buffer, size, &skip);
        }

      if (ret < size)
        {
          ret += backtrace_branch(rtcb->stack_base_ptr +
                                  rtcb->adj_stack_size, sp,
                                  &buffer[ret], size - ret, &skip);
        }
    }
  else
    {
      ret = 0;

      flags = enter_critical_section();

      if (skip-- <= 0)
        {
          buffer[ret++] = (void *)tcb->xcp.regs[REG_PC];
        }

      if (ret < size)
        {
          sp = (void *)tcb->xcp.regs[REG_SP];
          ret += backtrace_push(tcb->stack_base_ptr +
                                tcb->adj_stack_size, &sp,
                                (void *)tcb->xcp.regs[REG_LR],
                                &buffer[ret], size - ret, &skip);

          if (ret < size)
            {
              ret += backtrace_branch(tcb->stack_base_ptr +
                                      tcb->adj_stack_size, sp,
                                      &buffer[ret], size - ret, &skip);
            }
        }

      leave_critical_section(flags);
    }

  return ret;
}
