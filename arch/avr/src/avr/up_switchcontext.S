/************************************************************************************
 * arch/avr/src/avr/up_switchcontext.S
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

	.file		"up_switchcontext.S"

/************************************************************************************
 * Macros
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_switchcontext
 *
 * Description:
 *   Save the current thread context and restore the specified context.  The full
 *   C function prototype is:
 *
 *   void up_switchcontext(uint8_t *saveregs, uint8_t *restoreregs);
 *
 * On Entry:
 *   r24-r25: savregs
 *   r22-r23: restoreregs
 *
 * Returned Value:
 *   up_switchcontext forces a context switch to the task "canned" in restoreregs.
 *   It does not 'return' in the normal sense, rather, it will context switch back
 *   to the function point.  When it does 'return,' it is because the blocked
 *   task hat was "pickeled" in the saveregs "can" is again ready to run and has
 *   execution priority.
 *
 * Assumptions:
 *   global interrupts disabled by the caller.
 *
 ************************************************************************************/

	.text
	.globl	up_switchcontext
	.func	up_switchcontext
up_switchcontext:
	/* Use X [r26:r27] to reference the save structure.  (X is Call-used) */

	movw	r26, r24

	/* Save the context to saveregs */

	USER_SAVE

	/* Then fall through to do the full context restore with r24-r5 = restoreregs */

	movw	r24, r22
	.endfunc

/****************************************************************************
 * Name: up_fullcontextrestore
 *
 * Description:
 *   Restore the full-running context of a thread.
 *
 * Input Parameters:
 *   r24-r25 = A pointer to the register save area of the thread to be restored.
 *
 * C Prototype:
 *  void up_fullcontextrestore(uint8_t *regs);
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

	.text
	.global	up_fullcontextrestore
	.func	up_fullcontextrestore
up_fullcontextrestore:
	/* Use X [r26:r27] to reference the restore structure.  */

	movw	r26, r24

	/* Restore the context from the TCB saved registers */

	TCB_RESTORE

	/* Returning from the function is handled in TCB_RESTORE */

	.endfunc
	.end
