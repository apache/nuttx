/****************************************************************************
 * arch/misoc/src/minerva/minerva_syscall.S
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Ramtin Amin <keytwo@gmail.com>
 *
 * Derives from RISC-V version:
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
