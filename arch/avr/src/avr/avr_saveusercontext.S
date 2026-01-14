/************************************************************************************
 * arch/avr/src/avr/avr_saveusercontext.S
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#include "excptmacros.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

	.file		"avr_saveusercontext.S"

/************************************************************************************
 * Macros
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: up_saveusercontext
 *
 * Description:
 *   Save the current thread context
 *
 ****************************************************************************/

	.text
	.globl	up_saveusercontext
	.func	up_saveusercontext
up_saveusercontext:
	/* Use X [r26:r27] to reference the save structure.  (X is Call-used) */

	movw	r26, r24

	/* Save the context to saveregs */

	USER_SAVE

	/* Main purpose of USER_SAVE macro is to serve as a first half
	 * of a context switch. To be able to do that, it - among other
	 * things - pops return address from the stack. The other half
	 * of context switch procedure then reverses that and pushes
	 * (different) return address back to the stack and returns
	 *
	 * The same needs to be done here, albeit simplified (we are
	 * not restoring context, just need return address back on the stack
	 * to be able to return.)
	 *
	 * Return address is loaded to r18-r20 in USER_SAVE
	 */

	push r18
	push r19
#if AVR_PC_SIZE > 16
	push r20
#endif /* AVR_PC_SIZE */

	ret

	.endfunc
	.end
