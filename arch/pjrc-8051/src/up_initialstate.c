/************************************************************
 * up_initialstate.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <sched.h>
#include "up_internal.h"

/************************************************************
 * Private Definitions
 ************************************************************/

/************************************************************
 * Private Data
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB
 *   has been created. This function is called to initialize
 *   the processor specific portions of the new TCB.
 *
 *   This function must setup the intial architecture registers
 *   and/or  stack so that execution will begin at tcb->start
 *   on the next context switch.
 *
 ************************************************************/

void up_initial_state(FAR _TCB *tcb)
{
  FAR ubyte *frame = tcb->xcp.stack;

  /* This is the form of initial stack frame
   *
   * This initial stack frame will be configured to hold.
   * (1) The 16-bit return address of either:
   *
   *     void task_start(void);
   *     void pthread_start(void)
   *
   *     The return address is stored at the top of stack.
   *     so that the RETI instruction will work:
   *
   *     PC15-8 <- ((SP))
   *     (SP)   <- (SP) -1
   *     PC7-0  <- ((SP))
   *     (SP)   <- (SP) -1
   */

   frame[FRAME_RETLS] = (((uint16)tcb->start) & 0xff);
   frame[FRAME_RETMS] = (((uint16)tcb->start) >> 8);

  /* Then the context save area which can be indexed with
   * the following definitions (relative to the beginning of
   * the initial frame.
   */

  frame[FRAME_IE]     = 0x80;
  frame[FRAME_PSW]    = 0;

  tcb->xcp.nbytes     = FRAME_SIZE;
}
