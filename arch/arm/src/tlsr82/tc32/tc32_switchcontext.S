/****************************************************************************
 * arch/arm/src/tlsr82/tc32/tc32_switchcontext.S
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

#include <arch/chip/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file	"tc32_switchcontext.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_switchcontext
 *
 * Description:
 *   Restore the current thread context.  Full prototype is:
 *
 *   void arm_switchcontext(uint32_t **saveregs, uint32_t *restoreregs);
 *
 *   R0 = saveregs    , pointer to the current task tcb->regs, the sp after
                        current task context save should be saved in it.
 *   R1 = restoreregs , next task sp, using it to restore next task context.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.align	2
	.code	16
	.thumb_func
	.section .ram_code,"ax"
	.extern arm_fullcontextrestore
	.global	arm_switchcontext
	.type	arm_switchcontext, function
arm_switchcontext:

	/* Save LR, R0 ~ R7 into current task stack */

	tpush		{lr}
	tpush		{r0}
	tpush		{r1}
	tpush		{r2}
	tpush		{r3}
	tpush		{r4}
	tpush		{r5}
	tpush		{r6}
	tpush		{r7}

	/* Get current CPSR and save it into current task stack */

	tmrcs		r3
	tpush		{r3}

	/* Save IRQ_STATE into current task stack */

	tloadr 		r3, _REG_IRQ_EN3
	tloadrb		r2, [r3]
	tpush		{r2}

	/* Save r8 ~ r12 into current task stack */

	tmov		r3, r8
	tpush		{r3}
	tmov		r3, r9
	tpush		{r3}
	tmov		r3, r10
	tpush		{r3}
	tmov		r3, r11
	tpush		{r3}
	tmov		r3, r12
	tpush		{r3}

	/* Get the SP (before context save) value and
	 * save it into current task stack
	 */

	tmov		r2, #(XCPTCONTEXT_SIZE - 8)
	tmov		r3, r13
	tadd		r3, r3, r2
	tpush		{r3}

	/* Save PC into current task stack */

	tmov		r3, r14
	tpush		{r3}

	/* Save back current sp to the tcb->regs */

	tmov		r3, r13
	tstorer		r3, [r0]

	/* r0 = r1 = next task tcb->regs (actualy sp) */

	tmov		r0, r1

	/* Call arm_fullcontextrestore to restore the context */

	tjl			arm_fullcontextrestore

	.align 		2
_REG_IRQ_EN3:
	.word		0x00800643
	.size		arm_switchcontext, .-arm_switchcontext
	.end
