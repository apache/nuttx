/****************************************************************************
 * arch/arm/src/cortexm3/up_svcall.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef DEBUG_SVCALL         /* Define to debug SVCall */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_svcall
 *
 * Description:
 *   This is SVCall exception handler that performs context switching
 *
 ****************************************************************************/

int up_svcall(int irq, FAR void *context)
{
  uint32 *regs   = (uint32*)context;

  DEBUGASSERT(regs && regs == current_regs);
  DEBUGASSERT(regs[REG_R1] != 0);

  /* The SVCall software interrupt is called with R0 = SVC command and R1 = 
   * the TCB register save area.
   */

  sllvdbg("Command: %d regs: %p R1: %08x R2: %08x\n",
          regs[REG_R0], regs, regs[REG_R1], regs[REG_R2]);

#ifdef DEBUG_SVCALL
  lldbg("SVCall Entry:\n");
  lldbg("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
        regs[REG_R0],  regs[REG_R1],  regs[REG_R2],  regs[REG_R3],
        regs[REG_R4],  regs[REG_R5],  regs[REG_R6],  regs[REG_R7]);
  lldbg("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
        regs[REG_R8],  regs[REG_R9],  regs[REG_R10], regs[REG_R11],
        regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
  lldbg("  PSR=%08x\n", regs[REG_XPSR]);
#endif

  /* Handle the SVCall according to the command in R0 */

  switch (regs[REG_R0])
    {
      /* R0=0:  This is a save context command:
       *
       *   int up_saveusercontext(uint32 *saveregs);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = 0
       *   R1 = saveregs
       *
       * In this case, we simply need to copy the current regsters to the
       * save regiser space references in the saved R1 and return.
       */

      case 0:
        {
          memcpy((uint32*)regs[REG_R1], regs, XCPTCONTEXT_SIZE);
        }
        break;

      /* R0=1: This a restore context command:
       *
       *   void up_fullcontextrestore(uint32 *restoreregs) __attribute__ ((noreturn));
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = 1
       *   R1 = restoreregs
       *
       * In this case, we simply need to set current_regs to restore register
       * area referenced in the saved R1. context == current_regs is the normal
       * exception return.  By setting current_regs = context[R1], we force
       * the return to the saved context referenced in R1.
       */

      case 1:
        {
          current_regs = (uint32*)regs[REG_R1];
        }
        break;

      /* R0=2: This a switch context command:
       *
       *   void up_switchcontext(uint32 *saveregs, uint32 *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = 1
       *   R1 = saveregs
       *   R2 = restoreregs
       *
       * In this case, we do both: We save the context registers to the save
       * register area reference by the saved contents of R1 nad then set
       * current_regs to to the save register area referenced by the saved
       * contents of R2.
       */

      case 2:
        {
          DEBUGASSERT(regs[REG_R2] != 0);
          memcpy((uint32*)regs[REG_R1], regs, XCPTCONTEXT_SIZE);
          current_regs = (uint32*)regs[REG_R2];
        }
        break;

        
      default:
        PANIC(OSERR_INTERNAL);
        break;
    }
    
#ifdef DEBUG_SVCALL
  lldbg("SVCall Return:\n");
  lldbg("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
        current_regs[REG_R0],  current_regs[REG_R1],  current_regs[REG_R2],  current_regs[REG_R3],
        current_regs[REG_R4],  current_regs[REG_R5],  current_regs[REG_R6],  current_regs[REG_R7]);
  lldbg("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
        current_regs[REG_R8],  current_regs[REG_R9],  current_regs[REG_R10], current_regs[REG_R11],
        current_regs[REG_R12], current_regs[REG_R13], current_regs[REG_R14], current_regs[REG_R15]);
  lldbg("  PSR=%08x\n", current_regs[REG_XPSR]);
#endif

  return OK;
}
