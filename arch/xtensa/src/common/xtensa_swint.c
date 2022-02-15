/****************************************************************************
 * arch/xtensa/src/common/xtensa_swint.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/xtensa/xtensa_specregs.h>

#include "syscall.h"
#include "xtensa.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_swint
 *
 * Description:
 *   This is software interrupt exception handler that performs context
 *   switching and manages system calls
 *
 ****************************************************************************/

int xtensa_swint(int irq, void *context, void *arg)
{
  uint32_t *regs = (uint32_t *)context;
  uint32_t cmd;
#if XCHAL_CP_NUM > 0
  uintptr_t cpstate;
  uint32_t cpstate_off;

  cpstate_off = offsetof(struct xcptcontext, cpstate) -
                offsetof(struct xcptcontext, regs);
#endif

  DEBUGASSERT(regs && regs == CURRENT_REGS);
  cmd = regs[REG_A2];

  /* The SYSCall software interrupt is called with A2 = system call command
   * and A3..A9 = variable number of arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
# ifndef CONFIG_DEBUG_SYSCALL
  if (cmd > SYS_switch_context)
# endif
    {
      svcinfo("SYSCALL Entry: regs: %p cmd: %d\n", regs, cmd);
      svcinfo("  A0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              regs[REG_A0],  regs[REG_A1],  regs[REG_A2],  regs[REG_A3],
              regs[REG_A4],  regs[REG_A5],  regs[REG_A6],  regs[REG_A7]);
      svcinfo("  A8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              regs[REG_A8],  regs[REG_A9],  regs[REG_A10], regs[REG_A11],
              regs[REG_A12], regs[REG_A13], regs[REG_A14], regs[REG_A15]);
      svcinfo(" PC: %08x PS: %08x\n",
              regs[REG_PC], regs[REG_PS]);
    }
#endif

  /* Handle the SYSCall according to the command in A2 */

  switch (cmd)
    {
      /* A2=SYS_save_context:  This is a save context command:
       *
       * int xtensa_saveusercontext(uint32_t *saveregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_save_context
       *   A3 = saveregs
       *
       * In this case, we simply need to copy the current registers to the
       * save register space references in the saved A3 and return.
       */

      case SYS_save_context:
        {
          DEBUGASSERT(regs[REG_A3] != 0);
          memcpy((uint32_t *)regs[REG_A3], regs, (4 * XCPTCONTEXT_REGS));
#if XCHAL_CP_NUM > 0
          cpstate = (uintptr_t)regs[REG_A3] + cpstate_off;
          xtensa_coproc_savestate((struct xtensa_cpstate_s *)cpstate);
#endif
        }

        break;

      /* A2=SYS_restore_context:  This is a restore context command:
       *
       * void xtensa_fullcontextrestore(uint32_t *restoreregs)
       *      noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_restore_context
       *   A3 = restoreregs
       *
       * In this case, we simply need to set CURRENT_REGS to restore
       * register area referenced in the saved A3. context == CURRENT_REGS
       * is the normal exception return.  By setting CURRENT_REGS =
       * context[A3], we force the return to the saved context referenced
       * in A3.
       */

      case SYS_restore_context:
        {
#if XCHAL_CP_NUM > 0
          cpstate = (uintptr_t)regs[REG_A3] + cpstate_off;
          xtensa_coproc_restorestate((struct xtensa_cpstate_s *)cpstate);
#endif
          DEBUGASSERT(regs[REG_A3] != 0);
          CURRENT_REGS = (uint32_t *)regs[REG_A3];
        }

        break;

      /* A2=SYS_switch_context:  This is a switch context command:
       *
       * void xtensa_switchcontext
       *      (uint32_t *saveregs, uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A2 = SYS_switch_context
       *   A3 = saveregs
       *   A4 = restoreregs
       *
       * In this case, we do both: We save the context registers to the save
       * register area reference by the saved contents of A3 and then set
       * CURRENT_REGS to the save register area referenced by the saved
       * contents of A4.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_A3] != 0 && regs[REG_A4] != 0);

          memcpy((uint32_t *)regs[REG_A3], regs, (4 * XCPTCONTEXT_REGS));
          CURRENT_REGS = (uint32_t *)regs[REG_A4];
        }

        break;
    }

  if ((CURRENT_REGS[REG_PS] & PS_EXCM_MASK) != 0)
    {
      CURRENT_REGS[REG_PS] &= ~PS_EXCM_MASK;
    }

  /* Report what happened.  That might difficult in the case of a context
   * switch.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
# ifndef CONFIG_DEBUG_SYSCALL
  if (cmd > SYS_switch_context)
# else
  if (regs != CURRENT_REGS)
# endif
    {
      svcinfo("SYSCall Return:\n");
      svcinfo("  A0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              CURRENT_REGS[REG_A0],  CURRENT_REGS[REG_A1],
              CURRENT_REGS[REG_A2],  CURRENT_REGS[REG_A3],
              CURRENT_REGS[REG_A4],  CURRENT_REGS[REG_A5],
              CURRENT_REGS[REG_A6],  CURRENT_REGS[REG_A7]);
      svcinfo("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              CURRENT_REGS[REG_A8],  CURRENT_REGS[REG_A9],
              CURRENT_REGS[REG_A10], CURRENT_REGS[REG_A11],
              CURRENT_REGS[REG_A12], CURRENT_REGS[REG_A13],
              CURRENT_REGS[REG_A14], CURRENT_REGS[REG_A15]);
      svcinfo(" PC: %08x PS: %08x\n",
              regs[REG_PC], regs[REG_PS]);
    }
# ifdef CONFIG_DEBUG_SYSCALL
  else
    {
      svcinfo("SYSCall Return: %d\n", regs[REG_A2]);
    }
# endif
#endif

  return OK;
}
