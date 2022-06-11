/**************************************************************************
 * arch/x86/src/qemu/up_saveusercontext.S
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include "up_internal.h"

	.file	"qemu_saveusercontext.S"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Trace macros, use like trace 'i' to print char to serial port. */

	.macro	chout, addr, ch
#ifdef CONFIG_DEBUG_FEATURES
	mov		$\addr, %dx
	mov		$\ch, %al
	out		%al, %dx
#endif
	.endm

	.macro	trace, ch
#ifdef CONFIG_DEBUG_FEATURES
	push	%eax
	push	%edx
	chout	0x3f8, \ch
	pop		%edx
	pop		%eax
#endif
	.endm

/**************************************************************************
 * .text
 **************************************************************************/

	.text

/**************************************************************************
 * Name: up_saveusercontext
 *
 * Full C prototype:
 *  int up_saveusercontext(void *regs);
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

	.globl	up_saveusercontext
	.type	up_saveusercontext, @function
up_saveusercontext:
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

	/* Save the value of SP as will be at the time of the IRET that will
	 * appear to be the return from this function.
	 *
	 *
	 * 		CURRENT STACK				IRET STACK
	 *							PRIO CHANGE			No PRIO CHANGE
	 *		---------------		---------------		-----------------
	 *							EIP
	 *							CS					...
	 *							EFLAGS				EIP
	 *							ESP					CS
	 * ESP->Return address		SS					EFLAGS
	 *		Argument		->	Argument			Argument
	 *
	 * NOTE:  We don't yet know the value for REG_ESP!  That depends upon
	 * if a priority change occurs or not.
	 */

	leal	4(%esp), %ecx
	movl	%ecx, (4*REG_SP)(%eax)

	/* Fetch the PC from the stack and save it in the save block */

	movl	0(%esp), %ecx
	movl	%ecx, (4*REG_EIP)(%eax)

	/* Save the framepointer */

	movl	%ebp, (4*REG_EBP)(%eax)

	/* Save EAX=1.  This will be the "apparent" return value from this
	 * function when context is switch back to this thread.  The non-zero
	 * return value is the indication that we have been resumed.
	 */

	movl	$1, (4*REG_EAX)(%eax)

	/* Get and save the interrupt state */

	pushf
	pop		%ecx
	movl	%ecx, (4*REG_EFLAGS)(%eax)

	/* And return 0 -- The zero return value is the indication that that
	 * this is the original, "true" return from the function.
	 *
	 * 'ret' will remove the EIP from the top of the stack.
	 */

	xorl	%eax, %eax
	ret
	.size	up_saveusercontext, . - up_saveusercontext
	.end
