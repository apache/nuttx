/**************************************************************************
 * arch/x86/src/qemu/qemu_fullcontextrestore.S
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
 * Pre-processor Definitions
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
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_fullcontextrestore
 *
 * Full C prototype:
 *  void up_fullcontextrestore(uint32_t *regs) __attribute__ ((noreturn));
 *
 **************************************************************************/

#ifdef CONFIG_X86_NASM
#  warning "No Nasm support"
#else
	.file	"qemu_fullcontextrestore.S"
	.text
	.globl	up_fullcontextrestore
	.type	up_fullcontextrestore, @function
up_fullcontextrestore:
	/* Fetch the pointer to the register save array in EAX. */

	movl	4(%esp), %eax

	/* Disable interrupts now (the correct EFLAGS will be restored before we
	 * return
	 */

	cli

	/* We now have everything we need from the old stack.  Now get the new
	 * stack pointer.
	 */

	movl	(4*REG_SP)(%eax), %esp

	/* Save the return address, EFLAGS, and the values  as well the
	 * values of EBX and EAC on the new stack.
	 */

	movl	(4*REG_EIP)(%eax), %ebx
	push	%ebx
	movl	(4*REG_EFLAGS)(%eax), %ebx
	push	%ebx
	movl	(4*REG_EAX)(%eax), %ebx
	push	%ebx
	movl	(4*REG_EBX)(%eax), %ebx
	push	%ebx

	/* Now restore the remaining registers */

	movl	(4*REG_EDI)(%ebx), %edi
	movl	(4*REG_ESI)(%ebx), %esi
	movl	(4*REG_EBP)(%ebx), %ebp
	movl	(4*REG_EDX)(%ebx), %edx
	movl	(4*REG_ECX)(%ebx), %ecx

	/* Restore the segment registers */

	mov		(4*REG_DS)(%ebx), %ds
	mov		(4*REG_CS)(%ebx), %cs
	mov		(4*REG_SS)(%ebx), %ss

	/* Restore the correct value of EAX, EBX, and the EFLAGS then return */

	popl	%ebx
	popl	%eax
	popf
	ret
	.size up_fullcontextrestore, . - up_fullcontextrestore
    .end
#endif /* CONFIG_X86_NASM */

