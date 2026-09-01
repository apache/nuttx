/**************************************************************************
 * arch/x86_64/src/intel64/intel64_fullcontextrestore.S
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include "x86_64_internal.h"

	.file "intel64_fullcontextrestore.S"

/**************************************************************************
 * Public Functions
 **************************************************************************/

	.text

/**************************************************************************
 * Name: x86_64_fullcontextrestore
 *
 * Full C prototype:
 *  void x86_64_fullcontextrestore(uint32_t *regs) noreturn_function;
 *
 **************************************************************************/

	.globl   x86_64_fullcontextrestore
	.type    x86_64_fullcontextrestore, @function
x86_64_fullcontextrestore:
	/* The pointer to the register save array in RDI. */

	/* Disable interrupts now (the correct RFLAGS will be restored before we
	* return
	*/

	cli

#ifndef CONFIG_ARCH_X86_64_HAVE_XSAVE
	/* Restore xmm registers */
	fxrstorq (%rdi)
#else
	movl     $XSAVE_STATE_COMPONENTS, %eax
	xor      %edx, %edx
	xrstor   (%rdi)
#endif

	/* Restore the remaining registers.  The current stack may belong to
	 * the outgoing task, which can already be running on another CPU, so
	 * it must not be written from here on.
	 */

	movq    (8*REG_RSI)(%rdi), %rsi
	movq    (8*REG_RDX)(%rdi), %rdx
	movq    (8*REG_RCX)(%rdi), %rcx
	movq    (8*REG_R8 )(%rdi), %r8
	movq    (8*REG_R9 )(%rdi), %r9

	movq    (8*REG_R15)(%rdi), %r15
	movq    (8*REG_R14)(%rdi), %r14
	movq    (8*REG_R13)(%rdi), %r13
	movq    (8*REG_R12)(%rdi), %r12
	movq    (8*REG_R11)(%rdi), %r11
	movq    (8*REG_R10)(%rdi), %r10
	movq    (8*REG_RBP)(%rdi), %rbp
	movq    (8*REG_RBX)(%rdi), %rbx

	/* Restore the data segment register.  I think there is an issue that will
	* need to be address here at some time:  If the register save area is in
	* one data segment and the stack is in another, then the above would not
	* work (and, conversely, if they are in the same data segment, the
	* following is unnecessary and redundant).
	*/

	mov     (8*REG_DS)(%rdi), %ds
	/* mov        (8*REG_ES)(%rdi), %es // Not used in 64 bit
	* mov        (8*REG_GS)(%rdi), %gs // Disabled, otherwise we will destroy MSR_GS_BASE
	* mov        (8*REG_FS)(%rdi), %fs // Disabled, otherwise we will destroy MSR_FS_BASE
	* XXX: Should use wrgsbase and wrfsbase to restore the gs and fs register
	*/

#ifdef CONFIG_SCHED_THREAD_LOCAL
	mov     (8*REG_FSBASE)(%rdi), %rax
	wrfsbase  %rax
#endif

	movq    (8*REG_RAX)(%rdi), %rax

	/* REG_RIP..REG_SS are contiguous and match the iretq frame layout,
	 * so iretq can pop them straight from the register save area.
	 */

	leaq    (8*REG_RIP)(%rdi), %rsp

	/* Restore the correct value of RDI */

	movq    (8*REG_RDI)(%rdi), %rdi

	/* Pops 5 things at once: RIP, CS, RFLAGS, RSP and SS */

	iretq
	.size x86_64_fullcontextrestore, . - x86_64_fullcontextrestore
	.end
