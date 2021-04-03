/************************************************************************************
 * arch/avr/src/avr32/up_switchcontext.S
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
#include <arch/avr32/avr32.h>

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
 *   void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
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
	.globl		up_switchcontext
	.type		up_switchcontext, @function
up_switchcontext:
	/* "Pickle" the current thread context in the saveregs "can."  r12=saveregs.    */
	/*     xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx                       */
	/*     ^r12                                                                     */
	/* Sample SR and set r12 to just after the LR storage location.                 */
	/*     xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx xx                       */
	/*                                         ^r12                                 */

	mfsr	r10, AVR32_SR
	sub		r12, -4*(REG_LR+1)

	/* Then "push" PC=LR, LR, SR, and SP as they are on entry.                      */
	/*     xx xx xx xx xx xx xx xx SP SR PC LR xx xx xx xx xx                       */
	/*                             ^r12                                             */

	st.w	--r12, lr
	st.w	--r12, lr
	st.w	--r12, r10
	st.w	--r12, sp

	/* Save the preserved/static registers, r0-r7.  There is no reason to save the  */
	/* scratch/volatile registers, r8-r12, in this context.                         */
	/*     07 06 05 04 03 02 01 00 SP SR PC LR xx xx xx xx xx                       */
	/*     ^r12                                                                     */

	stm		--r12, r0-r7

	/* Finally, let up_fullcontextrestore handling the re-instatement of the thread */
	/* "canned" in restoregs.                                                       */

	mov		r12, r11
	lddpc	pc, .Lup_fullcontextrestore

.Lup_fullcontextrestore:
	.word	up_fullcontextrestore
	.size	up_switchcontext, .-up_switchcontext
	.end
