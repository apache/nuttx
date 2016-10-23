/****************************************************************************
 * arch/xtensa/src/common/xtensa_initialstate.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_corebits.h>

#include "xtensa.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB has been created. This
 *   function is called to initialize the processor specific portions of the
 *   new TCB.
 *
 *   This function must setup the intial architecture registers and/or stack
 *   so that execution will begin at tcb->start on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;
#if 0 /* REVISIT */
#if CONFIG_XTENSA_NCOPROCESSORS > 0
  uint32_t *ptr;
#endif
#endif /* REVISIT */

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  /* Set initial values of registers */

  xcp->regs[REG_PC]   = (uint32_t)tcb->start;         /* Task entrypoint                */
  xcp->regs[REG_A0]   = 0;                            /* To terminate GDB backtrace     */
  xcp->regs[REG_A1]   = (uint32_t)tcb->adj_stack_ptr; /* Physical top of stack frame    */

  /* Set initial PS to int level 0, EXCM disabled ('rfe' will enable), user
   * mode.
   */

#ifdef CONFIG_XTENSA_CALL0_ABI
  xcp->regs[REG_PS]   = PS_UM | PS_EXCM;

#else
  /* For windowed ABI set WOE and CALLINC (pretend task was 'call4'd). */

  xcp->regs[REG_PS]   = PS_UM | PS_EXCM | PS_WOE | PS_CALLINC(1);
#endif

#warning REVISIT co-processor support
#if 0 /* REVISIT */
#if CONFIG_XTENSA_NCOPROCESSORS > 0
  /* Init the coprocessor save area (see xtensa_context.h)
   *
   * No access to TCB here, so derive indirectly. Stack growth is top to bottom.
   * //ptr = (uint32_t *) xMPUSettings->coproc_area;
   */

  ptr = (uint32_t *)(((uint32_t)tcb->adj_stack_ptr - XT_CP_SIZE) & ~0xf);
  ptr[0] = 0;
  ptr[1] = 0;
  ptr[2] = (((uint32_t)ptr) + 12 + XTENSA_TOTAL_SA_ALIGN - 1) & -XTENSA_TOTAL_SA_ALIGN;
#endif
#endif /* REVISIT */
}
