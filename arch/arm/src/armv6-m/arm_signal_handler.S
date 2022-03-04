/****************************************************************************
 * arch/arm/srcm/armv6-m/arm_signal_handler.S
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

#if defined(CONFIG_BUILD_PROTECTED) && !defined(__KERNEL__)

/****************************************************************************
 * File info
 ****************************************************************************/

	.file		"arm_signal_handler.S"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_signal_handler
 *
 * Description:
 *   This function is the user-space, signal handler trampoline function.  It
 *   is called from up_signal_dispatch() in user-mode.
 *
 *     R0-R3, R11 - volatile registers need not be preserved.
 *     R4-R10 - static registers must be preserved
 *     R12-R14 - LR and SP must be preserved
 *
 * Input Parameters:
 *   R0 = sighand
 *     The address user-space signal handling function
 *   R1-R3 = signo, info, and ucontext
 *     Standard arguments to be passed to the signal handling function.
 *
 * Returned Value:
 *   None.  This function does not return in the normal sense.  It returns
 *   via the SYS_signal_handler_return (see syscall.h)
 *
 ****************************************************************************/

	.text
	.align	2
	.code	16
	.thumb_func
	.globl	up_signal_handler
	.type	up_signal_handler, function
up_signal_handler:

	/* Save some register */

	push		{r4, r5}		/* Save R4 and R5 on the stack */
	mov		r5, lr			/* Save LR in R5 */

	/* Call the signal handler */

	mov		r4, r0			/* R4=sighand */
	mov		r0, r1			/* R0=signo */
	mov		r1, r2			/* R1=info */
	mov		r2, r3			/* R2=ucontext */
	blx		r4			/* Call the signal handler */

	/* Restore the registers */

	mov		lr, r5			/* Restore LR */
	pop		{r4, r5}		/* Restore R4 and R5 */

	/* Execute the SYS_signal_handler_return SVCall (will not return) */

	mov		r0, #SYS_signal_handler_return
	svc		#SYS_syscall
	nop

	.size	up_signal_handler, .-up_signal_handler
	.end

#endif /* CONFIG_BUILD_PROTECTED && !__KERNEL__ */
