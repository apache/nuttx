/****************************************************************************
 * arch/arm/src/armv7-m/arm_backtrace.c
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

#define INSTR_IS(i, o) (((i) & (IMASK_##o)) == (IOP_##o))

#define IMASK_T_STMDB       0xfffff000  /* stmdb sp!,{..lr} */
#define IOP_T_STMDB         0xe92d4000

#define IMASK_T_PUSH_LO     0xff00      /* push {reglist} (not LR) */
#define IOP_T_PUSH_LO       0xb400

#define IMASK_T_PUSH        0xff00      /* push {reglist} (inc LR) */
#define IOP_T_PUSH          0xb500

#define IMASK_T_VPUSH_16    0xffbf8f00  /* vpush d */
#define IOP_T_VPUSH_16      0xed2d8b00

#define IMASK_T_VPUSH_8     0xffbf8f00  /* vpush s */
#define IOP_T_VPUSH_8       0xed2d8a00

#define IMASK_T_SUB_SP_16   0xff80      /* sub sp, # */
#define IOP_T_SUB_SP_16     0xb080

#define IMASK_T_SUB_SP_32   0xf2ff8f00  /* subw sp, sp, # */
#define IOP_T_SUB_SP_32     0xf2ad0d00

#define IMASK_T_SUB_W_SP_32 0xfbff8f00  /* sub.w sp, sp, # */
#define IOP_T_SUB_W_SP_32   0xf1ad0d00

#define INSTR_LIMIT         0x2000

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

static int getlroffset(FAR uint8_t *lr)
{
  lr = (FAR uint8_t *)((uintptr_t)lr & 0xfffffffe);

  if (((uintptr_t)lr & 0xffffffe0) == 0xffffffe0)
    {
      return 0;
    }

  return (*(FAR uint16_t *)(lr - 4) & 0xf000) == 0xf000 ? 5 : 3;
}

/****************************************************************************
 * Name: backtrace_internal
 *
 * Description:
 *  backtrace_internal()  returns the currect link address from input program
 *  counter and stack pointer register.
 *
 * Input Parameters:
 *   psp    - Double poninter to the SP, this parameter will be changed if
 *            the corresponding LR address is successfully found.
 *   ppc    - Double poninter to the PC, this parameter will be changed if
 *            the corresponding LR address is successfully found.
 *   addr   - Stack bottom address
 *
 * Returned Value:
 *   Link address should be returned if successful
 *   Otherwise, NULL is returned
 *
 ****************************************************************************/

static FAR void *backtrace_internal(FAR void **psp, FAR void **ppc,
                                    FAR void *ip, FAR void *addr)
{
  FAR uint8_t *sp = *psp;
  FAR uint8_t *pc = *ppc;
  FAR uint8_t *base;
  FAR uint8_t *lr;
  uint32_t ins32;
  uint16_t ins16;
  int offset = 1;
  int frame;
  int i;

  if (*psp >= addr)
    {
      return NULL;
    }

  for (i = 0; i < INSTR_LIMIT; i += 2)
    {
      ins16 = *(FAR uint16_t *)(pc - i);
      if (INSTR_IS(ins16, T_PUSH))
        {
          frame = __builtin_popcount(ins16 & 0xff) + 1;
          ins16 = *(FAR uint16_t *)(pc - i - 2);
          if (INSTR_IS(ins16, T_PUSH_LO))
            {
              offset += __builtin_popcount(ins16 & 0xff);
              frame  += offset - 1;
            }

          break;
        }

      ins32  = ins16 << 16;
      ins32 |= *(FAR uint16_t *)(pc - i + 2);
      if (INSTR_IS(ins32, T_STMDB))
        {
          frame = __builtin_popcount(ins32 & 0xfff) + 1;
          ins16 = *(FAR uint16_t *)(pc - i - 2);
          if (INSTR_IS(ins16, T_PUSH_LO))
            {
              offset += __builtin_popcount(ins16 & 0xff);
              frame  += offset - 1;
            }

          break;
        }

      if (ip && ip >= (FAR void *)(pc - i))
        {
          return NULL;
        }
    }

  if (i >= INSTR_LIMIT)
    {
      return NULL;
    }

  base = pc - i;

  for (i = 0; i < INSTR_LIMIT && base + i < pc; )
    {
      ins16 = *(FAR uint16_t *)(base + i);
      if (INSTR_IS(ins16, T_SUB_SP_16))
        {
          frame += (ins16 & 0x7f);
          break;
        }

      ins32  = ins16 << 16;
      ins32 |= *(FAR uint16_t *)(base + i + 2);
      if (INSTR_IS(ins32, T_SUB_SP_32))
        {
          uint32_t shift = ins32 >> 24 & 0x4;
          uint32_t sub = 0;

          if (shift)
            {
              sub = 1 << (shift - 1 + 8);
            }

          frame += (sub + (ins32 & 0xff) +
              ((ins32 & 0x7000) >> 4)) / sizeof(uint32_t);
          break;
        }
      else if (INSTR_IS(ins32, T_SUB_W_SP_32))
        {
          uint32_t shift;
          uint32_t sub;

          sub    = (ins32 & 0x7f) + 0x80;
          shift  = (ins32 >> 7) & 0x1;
          shift += ((ins32 >> 12) & 0x7) << 1;
          shift += ((ins32 >> 26) & 0x1) << 4;

          frame += sub << (30 - shift);
          break;
        }
      else if (INSTR_IS(ins32, T_VPUSH_16))
        {
          frame += (ins32 & 0xff);
        }
      else if (INSTR_IS(ins32, T_VPUSH_8))
        {
          frame += (ins32 & 0xff) / 2;
        }

      i += ((ins16 & 0xf800) >= 0xe800) ? 4 : 2;
    }

  lr = (FAR uint8_t *)*((FAR uint32_t *)sp + frame - offset);
  if (lr == NULL)
    {
      return NULL;
    }

  offset = getlroffset(lr);
  if (offset == 0)
    {
      return NULL;
    }

  *psp   = (FAR uint32_t *)sp + frame;
  *ppc   = lr - offset;

  return *ppc;
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
 *
 * Returned Value:
 *   up_backtrace() returns the number of addresses returned in buffer
 *
 ****************************************************************************/

int up_backtrace(FAR struct tcb_s *tcb, FAR void **buffer, int size)
{
  FAR struct tcb_s *rtcb;
  irqstate_t flags;
  FAR void *sp;
  FAR void *pc;
  FAR void *ip;
  int i = 0;

  if (!tcb || size <= 0 || !buffer)
    {
      return 0;
    }

  flags = enter_critical_section();

  rtcb = this_task();

  if (rtcb == tcb)
    {
      if (up_interrupt_context() == false)
        {
          pc = (FAR uint8_t *)up_backtrace + 0x10;
          sp = (FAR void *)up_getsp();
          ip = (FAR void *)up_backtrace;

          buffer[i++] = up_backtrace;
        }
      else
        {
          pc = (FAR void *)CURRENT_REGS[REG_PC];
          sp = (FAR void *)CURRENT_REGS[REG_SP];
          ip = (FAR void *)CURRENT_REGS[REG_IP];

          buffer[i++] = (FAR void *)CURRENT_REGS[REG_PC];
        }
    }
  else
    {
      pc = (FAR void *)tcb->xcp.regs[REG_PC];
      sp = (FAR void *)tcb->xcp.regs[REG_SP];
      ip = (FAR void *)tcb->xcp.regs[REG_IP];

      buffer[i++] = pc;
    }

  if (size == 1)
    {
      leave_critical_section(flags);
      return size;
    }

  for (; i < size; i++)
    {
      if ((uintptr_t)pc & 0x1)
        {
          pc = (uintptr_t)pc & 0xfffffffe;
        }

      buffer[i] = backtrace_internal(&sp, &pc, ip,
                                     tcb->stack_alloc_ptr +
                                     tcb->adj_stack_size);
      if (ip != NULL)
        {
          ip = NULL;
        }

      if (!buffer[i])
        {
          /* Try LR further if the PC can not trace
           * correctly at first time
           */

          if (i == 1 &&
              rtcb != tcb &&
              pc != (FAR void *)tcb->xcp.regs[REG_LR])
            {
              pc = (FAR void *)tcb->xcp.regs[REG_LR];
              buffer[i] = pc;
              continue;
            }

          break;
        }
    }

  leave_critical_section(flags);

  return i;
}
