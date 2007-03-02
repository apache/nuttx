/**************************************************************************
 * up_savecontext.c
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

#include <sys/types.h>
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
 * Name: up_pushcontext
 *
 * Description:
 *   Push the current execution context onto the before the stack.  Similar
 *   operations are executed from the interrupt state save.
 *
 * Inputs:
 *   None
 *
 * Return:
 *   Returns the stack pointer (always non-zero).  However, when
 *   up_popcontext() executes, it will appear as a return from this
 *   function with return value == 0
 *
 **************************************************************************/

static ubyte up_pushcontext(void) __naked
{
  /* Push the current execution context onto the stack. */

  _asm
	push	acc
	push	ie
	mov	dptr, #0 /* Save return value (dpl) = 0 */
	push	dpl
	push	dph
	push	b
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	push	ar7
	push	ar0
	push	ar1
	push	psw
	clr	psw
	push	_bp

	/* And return the current stack pointer value in dpl */

	mov	dpl, sp
	ret
  _endasm;
}

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_savecontext
 *
 * Description:
 *   Push the current execution context onto the stack, then save the
 *   entire stack contents in the provided context structure.
 *
 * Inputs:
 *   context - the context structure in which to save the stack info
 *
 * Return
 *   0 = Normal state save return
 *   1 = This is the matching return from up_restorecontext()
 *
 **************************************************************************/

ubyte up_savecontext(FAR struct xcptcontext *context)
{
  irqstate_t flags = irqsave();
  ubyte sp = up_pushcontext();
  if (sp)
    {
      /* Now copy the current stack frame (including the saved execution
       * context) from internal RAM to XRAM.
       */

      ubyte nbytes    = sp - (STACK_BASE-1);
      FAR ubyte *src  = (FAR ubyte*)STACK_BASE;
      FAR ubyte *dest = context->stack;

      /* Then copy the stack info into the context structure */

      context->nbytes = nbytes;
      while (nbytes--)
        {
          *dest++ = *src++;
        }

      /* Return zero so that the return behavior is similar to setjmp */

      irqrestore(flags);
      return 0;
    }

  /* Return one so that the return behavior is similar to setjmp */

  irqrestore(flags);
  return 1;
}
