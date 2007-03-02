/**************************************************************************
 * up_restorecontext.c
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/irq.h>
#include "up_internal.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_popcontext
 *
 * Description:
 *   Pop the current execution context from the stack and return to the
 *   execution context.  Similar operations are executed from the interrupt
 *   state restore
 *
 * Inputs:
 *   None
 *
 * Return:
 *   This function does not return
 *
 **************************************************************************/

static void up_popcontext(ubyte newsp) __naked

{
  _asm 
	pop	_bp 
	pop	psw
	pop	ar1 
	pop	ar0 
	pop	ar7 
	pop	ar6 
	pop	ar5 
	pop	ar4 
	pop	ar3 
	pop	ar2 
	pop	b 
	pop	dph
	pop	dpl 

	/* Restore the interrupt state per the stored IE value */

	pop	acc
	jb	acc.7,00001$
	clr	ie.7
	sjmp	00002$
  00001$:
	setb	ie.7
  00002$:

	pop acc
	ret
  _endasm;
}

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_restorecontext
 *
 * Description:
 *   Restore the stack specified in the context structure and return to
 *   that context
 *
 * Inputs:
 *   context - Holds the stack content of the context to return to
 *
 * Return:
 *   This function does not return.
 *
 **************************************************************************/

void up_restorecontext(FAR struct xcptcontext *context)
{
  int nbytes       = context->nbytes;
  FAR  ubyte *src  = context->stack;
  NEAR ubyte *dest = (NEAR ubyte*)STACK_BASE;

  /* Interrupts should be disabled for the following.  up_popcontext() will
   * set the new interrupt state correctly.
   */

  (void)irqsave();

  while (nbytes--)
    {
      *src++ = *dest++;
    }

  /* Then return to the restored context (probably restoring interrupts) */

  up_popcontext(context->nbytes + (STACK_BASE-1));
}

/**************************************************************************
 * Name: up_restorestack
 *
 * Description:
 *   Restore the entire interrupt stack contents in the provided context
 *   structure.
 *
 * Inputs:
 *   context - the context structure from  which to restore the stack info
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   - We are in an interrupt handler with g_irqtos set
 *   - Interrupts are disabled
 *
 **************************************************************************/

void up_restorestack(FAR struct xcptcontext *context)
{
  /* Now copy the current stack frame (including the saved execution
   * context) from internal RAM to XRAM.
   */

  ubyte nbytes      = context->nbytes;
  FAR  ubyte *src   = context->stack;
  NEAR ubyte *dest  = (NEAR ubyte*)STACK_BASE;

  while (nbytes--)
    {
      *dest++ = *src++;
    }

  /* We are still in the interrupt context, but the size of the interrupt
   * stack has changed.
   */

  g_irqtos = context->nbytes + (STACK_BASE-1);
}


