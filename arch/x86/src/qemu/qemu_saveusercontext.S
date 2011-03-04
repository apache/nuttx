/**************************************************************************
 * arch/x86/src/qemu/up_saveusercontext.S
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 **************************************************************************/

/**************************************************************************
 * Conditional Compilation Options
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include "up_internal.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/
 
#ifdef __CYGWIN__
# define SYMBOL(s) _##s
#else
# define SYMBOL(s) s
#endif

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
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_saveusercontext
 *
 * Full C prototype:
 *  int up_saveusercontext(uint32_t *regs);
 *
 **************************************************************************/

#ifdef CONFIG_X86_NASM
#  warning "No Nasm support"
#else
	.text
	.globl	SYMBOL(up_saveusercontext)
#ifndef __CYGWIN__
	.type	SYMBOL(up_saveusercontext), @function
#endif
SYMBOL(up_saveusercontext):

	/* %ebx, %esi, %edi, and %ebp must be preserved.
	 * save %ebx, $esi, and %edi now... */

	movl	4(%esp), %eax
	movl	%ebx, (REG_EBX)(%eax)
	movl	%esi, (REG_ESI)(%eax)
	movl	%edi, (REG_EDI)(%eax)

	/* Save the value of SP as will be after we return */

	leal	4(%esp), %ecx
	movl	%ecx, (REG_SP)(%eax)

	/* Save the return PC */

	movl	0(%esp), %ecx
	movl	%ecx, (REG_PC)(%eax)

	/* Save the framepointer */

	movl %ebp, (REG_EBP)(%eax)

	/* And return 0 */

	xorl    %eax, %eax
	ret
#ifndef __CYGWIN__
	.size	SYMBOL(up_saveusercontext), . - SYMBOL(up_saveusercontext)
#endif
    .end
#endif
