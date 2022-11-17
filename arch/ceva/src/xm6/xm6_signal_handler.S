/****************************************************************************
 * arch/ceva/src/xm6/xm6_signal_handler.S
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

	.file		"xm6_signal_handler.S"

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
 * Inputs:
 *   r0 = sighand
 *     The address user-space signal handling function
 *   r1, r2, r3 = signo, info, and ucontext
 *     Standard arguments to be passed to the signal handling function.
 *
 * Return:
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

	/* Call the signal handler */

	mov		r0.ui, r4.ui		/* r4=sighand */
	mov		r1.ui, r0.ui		/* r0=signo */
	mov		r2.ui, r1.ui		/* r1=info */
	mov		r3.ui, r2.ui		/* r2=ucontext */
	nop		#0x03
	callar		r4.ui			/* Call the signal handler */

	/* Restore the registers */

	pop		retreg.ui

	/* Execute the SYS_signal_handler_return SVCall (will not return) */

	mov		#SYS_signal_handler_return, r0.ui
	trap		{t0}

	.func_end	3 _up_signal_handler

#else	/* Add dummy symbol to avoid cofflib crash */

	.text
dummy_signal_handler:

#endif
