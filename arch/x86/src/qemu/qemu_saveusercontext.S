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
 * Description:
 *  Save the "user" context.  It is not necessary to save all of the
 *  registers because it is acceptable for certain registers to be
 *  modified upon return from a subroutine call.  On a context switch
 *  back to user mode, it will appear as a return from this function.
 *
 *  According to the Intel ABI, the EAX, EDX, and ECX are to be free for
 *  use within a procedure or function, and need not be preserved.  These
 *  are the so-called caller-saved registers are EAX, ECX, EDX.
 *
 *  On entry,
 *    sp points to the return address
 *    sp+4 points to register save array
 *
 **************************************************************************/

#ifdef CONFIG_X86_NASM
#  warning "No Nasm support"
#else
	.file	"qemu_saveusercontext.S"
	.text
	.globl	SYMBOL(up_saveusercontext)
#ifndef __CYGWIN__
	.type	SYMBOL(up_saveusercontext), @function
#endif
SYMBOL(up_saveusercontext):
	/* Fetch the pointer to the register save array.  %eax is a available
	 * because it must be modified later to provide the return value.
	 */

	movl	4(%esp), %eax

	/* %ebx, %esi, %edi, and %ebp must be preserved.  We can freely used %eax
	 * because it will be the return value from this function.
	 */

	movl	%ebx, (4*REG_EBX)(%eax)
	movl	%esi, (4*REG_ESI)(%eax)
	movl	%edi, (4*REG_EDI)(%eax)

	/* Save the segment registers */

	mov		%ss, (4*REG_SS)(%eax)
	mov		%cs, (4*REG_CS)(%eax)
	mov		%ds, (4*REG_DS)(%eax)

	/* Save the value of SP as will be after we return (don't bother to save
	 * REG_ESP).
	 */

	leal	4(%esp), %ecx
	movl	%ecx, (4*REG_SP)(%eax)

	/* Fetch the PC from the stack and save it in the save block */

	movl	0(%esp), %ecx
	movl	%ecx, (4*REG_EIP)(%eax)

	/* Save the framepointer */

	movl	%ebp, (4*REG_EBP)(%eax)

    /* Get and save the interrupt state */

    pushf
    pop		%ecx
	movl	%ecx, (4*REG_EFLAGS)(%eax)
 
	/* And return 0.  'ret' will remove the EIP from the top of the stack. */

	xorl    %eax, %eax
	ret
#ifndef __CYGWIN__
	.size	SYMBOL(up_saveusercontext), . - SYMBOL(up_saveusercontext)
#endif
    .end
#endif
