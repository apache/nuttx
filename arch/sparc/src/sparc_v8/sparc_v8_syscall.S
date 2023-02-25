/****************************************************************************
 * arch/sparc/src/sparc_v8/sparc_v8_syscall.S
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

#include <arch/irq.h>

#include "sparc_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file	"sparc_v8_syscall.S"
	.global sys_call0
	.global sys_call1
	.global sys_call2
	.global sys_call3
	.global sys_call4
	.global sys_call5

	.global sparc_swint0
	.global sparcsyscall_exit
	.global syscall_irqdis
	.global syscall_irqen

/****************************************************************************
 * Private Data
 ****************************************************************************/
#define SYS_exit        1
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sparc_swint0
 *
 * Description:
 *  This file contains emulated system calls using software trap 0.
 *  The following calls are supported:
 *
 *    + SYS_exit	(halt)
 *    + SYS_irqdis	(disable interrupts)
 *    + SYS_irqset	(set interrupt level)
 *
 ****************************************************************************/

	.seg    "text"
	/*
	 *  system call - halt
	 *
	 *  On entry:
	 *
	 *    l0 = psr (from trap table)
	 *    l1 = pc
	 *    l2 = npc
	 *    g1 = system call id (1)
	 *
	 *  System Call 1 (exit):
	 *    g2 = additional exit code 1
	 *    g3 = additional exit code 2
	 */

sparc_swint0:
	ta	0				! syscall 0, halt with %g1,%g2,%g3 info

sparcsyscall_exit:

	mov	SYS_exit, %g1
	mov	%o0, %g2			! Additional exit code 1
	mov	%o1, %g3			! Additional exit code 2
	ta	SPARC_SWTRAP_SYSCALL

	/*
	 *  system call - Interrupt Disable
	 *
	 *  On entry:
	 *
	 *    l0 = psr (from trap table)
	 *    l1 = pc
	 *    l2 = npc
	 *    l3 = psr | SPARC_PSR_PIL_MASK
	 *
	 *  On exit:
	 *    g1 = old psr (to user)
	 */

	.align 32					! Align to 32-byte cache-line

syscall_irqdis:
	mov	%l3, %psr			! Set PSR. Write delay 3 instr
	or	%l0, SPARC_PSR_ET_MASK, %g1	! return old PSR with ET=1
	nop; nop				! PSR write delay;
	jmp	%l2				! Return to after TA 9.
	 rett	%l2 + 4

	/*
	 *  system call - Interrupt Enable
	 *
	 *  On entry:
	 *
	 *    l0 = psr (from trap table)
	 *    l1 = pc
	 *    l2 = npc
	 *    l3 = psr & ~0x0f00
	 *    g1 = new PIL to write (from user)
	 */

	.align 32					! Align to 32-byte cache-line

syscall_irqen:
	and	%g1, SPARC_PSR_PIL_MASK, %l4	! %l4 = (%g1 & 0xf00)
	wr	%l3, %l4, %psr			! PSR = (PSR & ~0xf00) ^ %l4
	nop; nop				! PSR write delay;
	jmp	%l2				! Return to after TA 10.
	 rett	%l2 + 4

/************************************************************************************
 * Name: sparc_syscall0, sparc_syscall1, sparc_syscall2, sparc_syscall3, sparc_syscall4, sparc_syscall5
 *
 * Description:
 *   sparc_syscall0 - System call SYS_ argument and no additional parameters.
 *   sparc_syscall1 - System call SYS_ argument and one additional parameter.
 *   sparc_syscall2 - System call SYS_ argument and two additional parameters.
 *   sparc_syscall3 - System call SYS_ argument and three additional parameters.
 *   sparc_syscall4 - System call SYS_ argument and four additional parameters.
 *   sparc_syscall5 - System call SYS_ argument and five additional parameters.
 *
 * Assumption:
 *   All interrupts are disabled except for the software interrupts.
 *
 ************************************************************************************/
sys_call0:	/* %o0 holds the syscall number */
sys_call1:	/* %o0 holds the syscall number, argument in %o1 */
sys_call2:	/* %o0 holds the syscall number, arguments in %o1 and %o2 */
sys_call3:	/* %o0 holds the syscall number, arguments in %o1, %o2, and %o3 */
sys_call4:	/* %o0 holds the syscall number, arguments in %o1, %o2, a3 and %o4 */
sys_call5:	/* %o0 holds the syscall number, arguments in %o1, %o2, %o3, %o4 and %o5 */

	/* Issue the ECALL opcode to perform a SW interrupt to the OS */
	ta 8;   ! syscall 8
	nop;
	nop;
	nop;

	jmp     %o7 + 8
	nop

