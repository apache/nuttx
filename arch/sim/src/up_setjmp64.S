/**************************************************************************
 * arch/sim/src/up_setjmp64.S
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

#ifdef __CYGWIN__
//# define SYMBOL(s) _##s
# define SYMBOL(s) s
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
 * Public Variables
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

	.text
	.align	4
	.globl	SYMBOL(up_setjmp)
#ifndef __CYGWIN__
	.type	SYMBOL(up_setjmp), @function
#endif
SYMBOL(up_setjmp):

	/* Get the return address, adjusting the stack pointer */

	pop		%rsi

	/* Set up the return value */

	xorl	%eax,%eax

	/* Save 1: rbx */

	movq	%rbx, JB_RBX(%rdi)

	/* Save 2: Value of the rsp *after* returning */

	movq	%rsp, JB_RSP(%rdi)

	/* Fix up the return stack */

	push	%rsi

	/* Save registers */
	/* Storage order: %rbx, %rsp, %rbp, %r12, %r13, %r14, %r15, %rip */

	movq	%rbp, JB_RBP(%rdi)	/* Save 3: rbp */
	movq	%r12, JB_R12(%rdi)	/* Save 4: r12 */
	movq	%r13, JB_R13(%rdi)	/* Save 5: r13 */
	movq	%r14, JB_R14(%rdi)	/* Save 6: r14 */
	movq	%r15, JB_R15(%rdi)	/* Save 7: r15 */
	movq	%rsi, JB_RSI(%rdi)	/* Save 8: Return address */

	ret

#ifndef __CYGWIN__
	.size	SYMBOL(up_setjmp), . - SYMBOL(up_setjmp)
#endif

	.align	4
	.globl	SYMBOL(up_longjmp)
#ifndef __CYGWIN__
	.type	SYMBOL(up_longjmp), @function
#endif
SYMBOL(up_longjmp):

	/* Setup return value */

	movl	%esi,%eax

	/* Restore registers */

	movq	JB_RBX(%rdi),%rbx	/* Save 1: rbx */
	movq	JB_RSP(%rdi),%rsp	/* Save 2: rsp */
	movq	JB_RBP(%rdi),%rbp	/* Save 3: rdi */
	movq	JB_R12(%rdi),%r12	/* Save 4: r12 */
	movq	JB_R13(%rdi),%r13	/* Save 5: r13 */
	movq	JB_R14(%rdi),%r14	/* Save 6: r14 */
	movq	JB_R15(%rdi),%r15	/* Save 7: rbp */

	/* And return */

	jmp		*JB_RSI(%rdi)	/* Save 8: rsi */

#ifndef __CYGWIN__
	.size SYMBOL(up_longjmp), . - SYMBOL(up_longjmp)
#endif

