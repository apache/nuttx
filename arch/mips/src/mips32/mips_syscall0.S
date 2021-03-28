/****************************************************************************
 * arch/mips/src/mips32/mips_syscall0.S
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

#include <arch/mips32/registers.h>
#include <arch/mips32/cp0.h>

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file	"mips_syscall0.S"
	.global sys_call0
	.global sys_call1
	.global sys_call2
	.global sys_call3

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_syscall0, up_syscall1, up_syscall2, up_syscall3
 *
 * Description:
 *   up_syscall0 - System call SYS_ argument and no additional parameters.
 *   up_syscall1 - System call SYS_ argument and one additional parameter.
 *   up_syscall2 - System call SYS_ argument and two additional parameters.
 *   up_syscall3 - System call SYS_ argument and three additional parameters.
 *
 * Assumption:
 *   All interrupts are disabled except for the software interrupts.
 *
 ****************************************************************************/

	.text
	.set	noreorder
	.set	nomips16
#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips
#endif
	.ent	sys_call0

sys_call0:	/* r4 holds the syscall number */
sys_call1:	/* r4 holds the syscall number, argument in r5 */
sys_call2:	/* r4 holds the syscall number, arguments in r5 and r6 */
sys_call3:	/* r4 holds the syscall number, arguments in r5, r6, and r7 */

	.set	push
	.set	noat

	/* Set Bit 8 to request the software interrupt */

	mfc0	t3, MIPS32_CP0_CAUSE	/* t3 = CP0 cause register */
	ori		t3, (1 << 8)			/* Bit 8: Request software interrupt 0 */
	.set	noreorder
	mtc0	t3, MIPS32_CP0_CAUSE	/* Trigger the software interrupt */

	/* The actual interrupt will not a occur for a few more cycles.  Let's
	 * put a few nop's here in hope that the SW interrupt occurs during
	 * the sequence of nops.
	 */

	nop
	nop
	nop
	nop

	/* Then return with the result of the software interrupt in v0 */

	j		ra
	nop
	.end	sys_call0
