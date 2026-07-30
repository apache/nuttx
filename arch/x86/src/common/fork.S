/****************************************************************************
 * arch/x86/src/common/fork.S
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "x86_fork.h"

	.file	"fork.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fork
 *
 * Description:
 *   The architecture-specific entry point of fork() and vfork().  It records
 *   what the caller was doing and hands that to x86_fork().  See x86_fork.c
 *   for what happens from there.
 *
 *   This architecture has no address environment that can be duplicated, so
 *   CONFIG_ARCH_HAVE_FORK is never set for it and only vfork() reaches here.
 *   The flag is passed on all the same, because the entry point is shared.
 *
 * Input Parameters:
 *   vfork - true for vfork(), false for fork().  cdecl puts it on the stack,
 *           so unlike the register-passing architectures it is loaded here.
 *
 * Returned Value:
 *   Upon successful completion, 0 is returned to the child and the process
 *   ID of the child is returned to the caller.  Otherwise, -1 is returned to
 *   the caller, no child is created, and errno is set to indicate the error.
 *
 ****************************************************************************/

/* The snapshot, as it sits on the caller's stack once it is built.  The entry
 * point `jmp's here rather than calling, so the return address at the top on
 * entry is the original caller's and %esp + 4 is the stack pointer it had
 * before it called us.
 *
 *   | .........  |
 *   | ret addr   | <- %esp on entry here;  caller's %esp is this + 4
 *   | eip        |  \
 *   | eflags     |   |
 *   | ss         |   |
 *   | cs         |   |
 *   | ds         |   | struct fork_s
 *   | esp        |   |
 *   | ebp        |   |
 *   | ebx        |   |
 *   | esi        |   |
 *   | edi        |  /  <- %esp, and the `context' argument to x86_fork()
 */

	.globl	up_fork
	.type	up_fork, @function

up_fork:
	/* The flag is the first and only argument.  At entry (%esp) holds the
	 * return address and 4(%esp) holds the argument, which is also the stack
	 * pointer the caller had:  it pushed the argument, then `call' pushed the
	 * return address.  So the LEA below still names the caller's own stack.
	 */

	movl	4(%esp), %edx
	jmp		x86_fork_common
	.size	up_fork, . - up_fork

	.type	x86_fork_common, @function

x86_fork_common:
	/* Take the two things that are read off this frame before any of it is
	 * disturbed:  where the caller would have returned to, and the stack
	 * pointer it had -- one slot above the return address that the `call'
	 * reaching the entry point left behind.  That stack pointer is the low
	 * end of the region x86_fork() copies.
	 *
	 * Neither MOV nor LEA writes EFLAGS, so the PUSHFL below still records
	 * the caller's own flags.
	 */

	movl	(%esp), %ecx
	leal	4(%esp), %eax

	/* Build struct fork_s, highest-addressed field first.  Segment
	 * selectors go through a zeroed register:  `pushl %<sreg>' leaves the
	 * upper half of the stack slot unmodified on most implementations
	 * rather than zero-extending it, and the C side reads all 32 bits.
	 */

	pushl	%ecx			/* eip -- the caller's return address */
	pushfl					/* eflags */
	xorl	%ecx, %ecx
	movw	%ss, %cx
	pushl	%ecx			/* ss */
	movw	%cs, %cx
	pushl	%ecx			/* cs */
	movw	%ds, %cx
	pushl	%ecx			/* ds */
	pushl	%eax			/* esp */
	pushl	%ebp			/* ebp */
	pushl	%ebx			/* ebx */
	pushl	%esi			/* esi */
	pushl	%edi			/* edi */

	/* x86_fork(context, type) */

	movl	%esp, %eax
	pushl	%edx
	pushl	%eax
	call	x86_fork
	addl	$8, %esp

	/* The return value is in %eax and stays there.  Restore the
	 * callee-saved registers from the snapshot and discard the rest of it;
	 * the caller's return address is then back on top for the `ret'.
	 */

	popl	%edi
	popl	%esi
	popl	%ebx
	popl	%ebp
	addl	$24, %esp		/* esp, ds, cs, ss, eflags, eip */
	ret
	.size	x86_fork_common, . - x86_fork_common
	.end
