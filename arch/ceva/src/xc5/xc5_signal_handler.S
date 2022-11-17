/****************************************************************************
 * arch/ceva/src/xc5/xc5_signal_handler.S
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

#if (defined(CONFIG_BUILD_PROTECTED) && !defined(__KERNEL__)) && \
    !defined(CONFIG_DISABLE_SIGNALS)

/****************************************************************************
 * File info
 ****************************************************************************/

	.file		"xc5_signal_handler.S"

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
 * Input Parameters:
 *   r0 = sighand
 *     The address user-space signal handling function
 *   r1, r2, r3 = signo, info, and ucontext
 *     Standard arguments to be passed to the signal handling function.
 *
 * Returned Value:
 *   None.  This function does not return in the normal sense.  It returns
 *   via the SYS_signal_handler_return (see syscall.h)
 *
 ****************************************************************************/

	.text
	.public		_up_signal_handler
	.func_start	3 _up_signal_handler

_up_signal_handler:

	/* Save some register */

	push		retreg.ui		/* Save LR on the stack */
	push {dw}	retreg
	/* Call the signal handler */

	mov		a0, a4			/* r4=sighand */
	mov		a1, a0			/* r0=signo */
	mov		a2, a1			/* r1=info */
	mov		a3, a2			/* r2=ucontext */
	mov		a4, r4
	nop
	nop
	nop
	callar		r4			/* Call the signal handler */

	/* Restore the registers */

	pop {dw}	retreg

	/* Execute the SYS_signal_handler_return SVCall (will not return) */

	mov		#SYS_signal_handler_return, a0
	trap

	.func_end	3 _up_signal_handler

#else	/* Add dummy symbol to avoid cofflib crash */

	.text
dummy_signal_handler:

#endif
