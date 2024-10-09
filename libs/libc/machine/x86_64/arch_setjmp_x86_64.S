/**************************************************************************
 * libs/libc/machine/x86_64/arch_setjmp_x86_64.S
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

#include <arch/setjmp.h>

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

#  define REGS %rdi

#ifdef __CYGWIN__
#  define SYMBOL(s) _##s
#elif defined(__ELF__)
#  define SYMBOL(s) s
#else
#  define SYMBOL(s) _##s
#endif

/**************************************************************************
 * Public Functions
 **************************************************************************/

	.text
	.align	4
	.globl	SYMBOL(setjmp)
#ifdef __ELF__
	.type	SYMBOL(setjmp), @function
#endif
SYMBOL(setjmp):

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
	movq	%rsi, JB_RIP(REGS)	/* Save 8: Return address */

	ret

#ifdef __ELF__
	.size	SYMBOL(setjmp), . - SYMBOL(setjmp)
#endif

	.align	4
	.globl	SYMBOL(longjmp)
#ifdef __ELF__
	.type	SYMBOL(longjmp), @function
#endif
SYMBOL(longjmp):

	/* Setup return value */

	movl	%esi,%eax
	testl	%eax,%eax
	jnz	1f
	incl	%eax

	/* Restore registers */
1:
	movq	JB_RBX(REGS),%rbx	/* Load 1: rbx */
	movq	JB_RSP(REGS),%rsp	/* Load 2: rsp */
	movq	JB_RBP(REGS),%rbp	/* Load 3: rdi */
	movq	JB_R12(REGS),%r12	/* Load 4: r12 */
	movq	JB_R13(REGS),%r13	/* Load 5: r13 */
	movq	JB_R14(REGS),%r14	/* Load 6: r14 */
	movq	JB_R15(REGS),%r15	/* Load 7: rbp */

	/* Jump to the saved return address (rip)  */

	jmp		*JB_RIP(REGS)

#ifdef __ELF__
	.size SYMBOL(longjmp), . - SYMBOL(longjmp)
#endif
