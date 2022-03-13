/****************************************************************************
 * arch/ceva/src/xc5/up_svcall_handler.S
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

#include <arch/syscall.h>

#ifdef CONFIG_LIB_SYSCALL

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file		"up_svcall_handler.S"
	.extern		_g_stublookup

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_svcall_handler
 *
 * Description:
 *   This function is the kernel-space, syscall handler trampoline function.  It
 *   is called from up_svcall() in interrupt handler.
 *
 *   Call the stub function corresponding to the system call.  NOTE the non-
 *   standard parameter passing:
 *
 *     r0 = SYS_ call number
 *     r1 = parm0
 *     r2 = parm1
 *     r3 = parm2
 *     r4 = parm3
 *     r5 = parm4
 *     r6 = parm5
 *
 * Returned Value:
 *   None.  This function does not return in the normal sense.  It returns
 *   via the SYS_syscall_return (see syscall.h)
 *
 ****************************************************************************/

	.text
	.public		_up_svcall_handler
	.func_start	3 _up_svcall_handler

_up_svcall_handler:

	/* Create a stack frame to hold LR */

	push {dw}	retreg

	/* Call the stub function */

	mov		#_g_stublookup, a8
	shift		#0x02, a8
	add		a8, a8
	nop
	nop
	mov		a8, r4
	ld {dw}		(r4), a8
	nop
	nop
	nop
	nop
	nop
	mov		a8, r4
	callar		r4

	/* Destroy the stack frame */

	pop {dw}	retreg

	/* Execute the SYS_syscall_return SVCall (will not return) */
						/* Save return value in r2 */
	mov		a0, a2			/* will restore in up_svcall */
	mov		#SYS_syscall_return, a0
	trap

	.func_end	3 _up_svcall_handler

#else	/* Add dummy symbol to avoid cofflib crash */

	.text
dummy_svcall_handler:

#endif /* CONFIG_LIB_SYSCALL */
