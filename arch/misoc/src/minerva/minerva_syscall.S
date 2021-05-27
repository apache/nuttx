/****************************************************************************
 * arch/misoc/src/minerva/minerva_syscall.S
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

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file	"up_syscall.S"
	.global sys_call0
	.global sys_call1
	.global sys_call2
	.global sys_call3
	.global sys_call4
	.global sys_call5

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
 *   up_syscall4 - System call SYS_ argument and four additional parameters.
 *   up_syscall5 - System call SYS_ argument and five additional parameters.
 *
 * Assumption:
 *   All interrupts are disabled except for the software interrupts.
 *
 ****************************************************************************/

	.text

sys_call0:	/* a0 holds the syscall number */
sys_call1:	/* a0 holds the syscall number, argument in a1 */
sys_call2:	/* a0 holds the syscall number, arguments in a1 and a2 */
sys_call3:	/* a0 holds the syscall number, arguments in a1, a2, and a3 */
sys_call4:	/* a0 holds the syscall number, arguments in a1, a2, a3 and a4 */
sys_call5:	/* a0 holds the syscall number, arguments in a1, a2, a3, a4 and a5 */

	/* Issue the ECALL opcode to perform a SW interrupt to the OS */

	ecall

	/* The actual interrupt may not a occur for a few more cycles.  Let's
	 * put a few nop's here in hope that the SW interrupt occurs during
	 * the sequence of nops.
	 */

	nop
	nop

	/* Then return with the result of the software interrupt in v0 */

	ret
	nop
