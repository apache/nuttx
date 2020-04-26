/**************************************************************************
 * arch/sim/src/sim/up_setjmp64.S
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include "up_internal.h"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/* The Microsoft x64 calling convention is followed on Microsoft Windows and
 * pre-boot UEFI (for long mode on x86-64). It uses registers RCX, RDX, R8,
 * R9 for the first four integer or pointer arguments (in that order), and
 * XMM0, XMM1, XMM2, XMM3 are used for floating point arguments. Additional
 * arguments are pushed onto the stack (right to left). Integer return
 * values (similar to x86) are returned in RAX if 64 bits or less. Floating
 * point return values are returned in XMM0. Parameters less than 64 bits
 * long are not zero extended; the high bits are not zeroed.
 */

#ifdef CONFIG_SIM_X8664_MICROSOFT
#  define REGS %rcx

/* The calling convention of the System V AMD64 ABI is followed on Solaris,
 * Linux, FreeBSD, macOS, and other UNIX-like or POSIX-compliant operating
 * systems. The first six integer or pointer arguments are passed in registers
 * RDI, RSI, RDX, RCX, R8, and R9, while XMM0, XMM1, XMM2, XMM3, XMM4, XMM5,
 * XMM6 and XMM7 are used for floating point arguments. For system calls, R10
 * is used instead of RCX.   As in the Microsoft x64 calling convention,
 * additional arguments are passed on the stack and the return value is stored
 * in RAX.
 *
 * Registers RBP, RBX, and R12-R15 are callee-save registers; all others must
 * be saved by the caller if they wish to preserve their values.
 *
 * Unlike the Microsoft calling convention, a shadow space is not provided; on
 * function entry, the return address is adjacent to the seventh integer
 * argument on the stack.
 */

#else /* if defined(CONFIG_SIM_X8664_SYSTEMV) */
#  define REGS %rdi
#endif

#ifdef __CYGWIN__
//# define SYMBOL(s) _##s
# define SYMBOL(s) s
#else
#ifdef __ELF__
# define SYMBOL(s) s
#else
# define SYMBOL(s) _##s
#endif
#endif

/**************************************************************************
 * Public Functions
 **************************************************************************/

	.text
	.align	4
	.globl	SYMBOL(up_setjmp)
#ifdef __ELF__
	.type	SYMBOL(up_setjmp), @function
#endif
SYMBOL(up_setjmp):

	/* Get the return address, adjusting the stack pointer */

	pop		%rsi

	/* Set up the return value */

	xorl	%eax,%eax

	/* Save 1: rbx */

	movq	%rbx, JB_RBX(REGS)

	/* Save 2: Value of the rsp *after* returning */

	movq	%rsp, JB_RSP(REGS)

	/* Fix up the return stack */

	push	%rsi

	/* Save registers */
	/* Storage order: %rbx, %rsp, %rbp, %r12, %r13, %r14, %r15, %rip */

	movq	%rbp, JB_RBP(REGS)	/* Save 3: rbp */
	movq	%r12, JB_R12(REGS)	/* Save 4: r12 */
	movq	%r13, JB_R13(REGS)	/* Save 5: r13 */
	movq	%r14, JB_R14(REGS)	/* Save 6: r14 */
	movq	%r15, JB_R15(REGS)	/* Save 7: r15 */
	movq	%rsi, JB_RSI(REGS)	/* Save 8: Return address */

	ret

#ifdef __ELF__
	.size	SYMBOL(up_setjmp), . - SYMBOL(up_setjmp)
#endif

	.align	4
	.globl	SYMBOL(up_longjmp)
#ifdef __ELF__
	.type	SYMBOL(up_longjmp), @function
#endif
SYMBOL(up_longjmp):

	/* Setup return value */

	movl	%esi,%eax

	/* Restore registers */

	movq	JB_RBX(REGS),%rbx	/* Save 1: rbx */
	movq	JB_RSP(REGS),%rsp	/* Save 2: rsp */
	movq	JB_RBP(REGS),%rbp	/* Save 3: rdi */
	movq	JB_R12(REGS),%r12	/* Save 4: r12 */
	movq	JB_R13(REGS),%r13	/* Save 5: r13 */
	movq	JB_R14(REGS),%r14	/* Save 6: r14 */
	movq	JB_R15(REGS),%r15	/* Save 7: rbp */

	/* And return */

	jmp		*JB_RSI(REGS)	/* Save 8: rsi */

#ifdef __ELF__
	.size SYMBOL(up_longjmp), . - SYMBOL(up_longjmp)
#endif
