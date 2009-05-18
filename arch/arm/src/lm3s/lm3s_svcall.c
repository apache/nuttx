/****************************************************************************
 * arch/arm/src/lm3s/lm3s_svcall.c
 * arch/arm/src/chip/lm3s_svcall.c
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
#include "lm3s_internal.h"

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
 * Name: lm3s_svcall
 *
 * Description:
 *   This is SVCall exception handler that performs context switching
 *
 ****************************************************************************/

int lm3s_svcall(int irq, FAR void *context)
{
  uint32 *svregs  = (uint32*)context;
  uint32 *tcbregs = (uint32*)svregs[REG_R1];

  DEBUGASSERT(svregs && svregs == current_regs && tcbregs);

  /* The SVCall software interrupt is called with R0 = SVC command and R1 = 
   * the TCB register save area.
   */

  sllvdbg("Command: %d svregs: %p tcbregs: %08x\n", svregs[REG_R0], svregs, tcbregs);

#ifdef DEBUG_SVCALL
  lldbg("SVCall Entry:\n");
  lldbg("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
        svregs[REG_R0],  svregs[REG_R1],  svregs[REG_R2],  svregs[REG_R3],
        svregs[REG_R4],  svregs[REG_R5],  svregs[REG_R6],  svregs[REG_R7]);
  lldbg("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
        svregs[REG_R8],  svregs[REG_R9],  svregs[REG_R10], svregs[REG_R11],
        svregs[REG_R12], svregs[REG_R13], svregs[REG_R14], svregs[REG_R15]);
  lldbg("  PSR=%08x\n", svregs[REG_XPSR]);
#endif

  /* Handle the SVCall according to the command in R0 */

  switch (svregs[REG_R0])
    {
      /* R0=0:  This is a save context command.  In this case, we simply need
       * to copy the svregs to the tdbregs and return.
       */

      case 0:
        memcpy(tcbregs, svregs, XCPTCONTEXT_SIZE);
        break;

      /* R1=1: This a restore context command.  In this case, we simply need to
       * set current_regs to tcbrgs.  svregs == current_regs is the normal exception
       * turn.  By setting current_regs = tcbregs, we force the return through
       * the saved context.
       */

      case 1:
        current_regs = tcbregs;
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
