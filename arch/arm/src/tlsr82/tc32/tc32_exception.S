/****************************************************************************
 * arch/arm/src/tlsr82/tc32/tc32_exception.S
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

	.file	"tc32_exception.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc32_exception
 *
 * Description:
 *   TC32 Common exception
 *
 ****************************************************************************/

	.align	2
	.code	16
	.thumb_func
	.section .ram_code,"ax"
	.extern irq_handler
	.global	tc32_exception
	.type	tc32_exception, function
tc32_exception:

	/* Restore the R14 (LR) saved in __irq */

	tpop		{r3}
	tmov		r14, r3

	/* Switch to SVC mode to get the SVC mode SP, then save context
	 * into the interrupted task stack.
	 */

	/* Disable interrupt, because cpu will switch into SVC mode, interrupt
	 * nested should be prevented, this is not supported by tc32
	 * architecture.
	 */

	tloadr		r0, _REG_IRQ_EN 		/* disable irq */
	tloadrb		r1, [r0]			/* get irq state in R1 */
	tmov		r2, #0				/* disable irq */
	tstorerb	r2, [r0]			/* disable irq */

	/* Switch to SVC mode and get the LR and SP in SVC mode */

	tloadr		r0, _REG_IRQ_EN + 4		/* switch to SVC mode */
	tnop
	tmcsr		r0				/* switch to SVC mode */
	tnop
	tmov		r2, r14				/* get SVC mode LR in R2 */
	tmov		r3, r13 			/* get SVC mode SP in R3 */
	tloadr		r0, _REG_IRQ_EN + 8 		/* return to IRQ mode */
	tnop
	tmcsr		r0				/* return to IRQ mode */
	tnop

	/* R0 = the interrupted task SP after context save */

	tmov		r0, #XCPTCONTEXT_SIZE
	tsub		r0, r3, r0

	/* Save IRQ_STATE, SVC mode SP and SVC mode LR as PC */

	tstorer		r1, [r0, #(4 * REG_IRQ_EN)]
	tstorer		r3, [r0, #(4 * REG_SP)]
	tstorer		r2, [r0, #(4 * REG_PC)]

	/* Get SPSR and save as CPSR */

	tmrss		r1
	tstorer		r1, [r0, #(4 * REG_CPSR)]

	/* Save IRQ mode LR as LR */

	tmov		r1, r14
	tstorer		r1, [r0, #(4 * REG_LR)]

	/* Pop the saved R1 ~ R3 (pushed in beginning of irq),
	 * then save R1 ~ R7.
	 */

	tpop		{r3}
	tpop		{r2}
	tpop		{r1}
	tstorer		r1, [r0, #(4 * REG_R1)]
	tstorer		r2, [r0, #(4 * REG_R2)]
	tstorer		r3, [r0, #(4 * REG_R3)]
	tstorer		r4, [r0, #(4 * REG_R4)]
	tstorer		r5, [r0, #(4 * REG_R5)]
	tstorer		r6, [r0, #(4 * REG_R6)]
	tstorer		r7, [r0, #(4 * REG_R7)]

	/* Pop R0 and save it */

	tpop		{r1}
	tstorer		r1, [r0, #(4 * REG_R0)]

	/* Save R8 ~ R12 */

	tmov		r1, r8
	tstorer		r1, [r0, #(4 * REG_R8)]
	tmov		r1, r9
	tstorer		r1, [r0, #(4 * REG_R9)]
	tmov		r1, r10
	tstorer		r1, [r0, #(4 * REG_R10)]
	tmov		r1, r11
	tstorer		r1, [r0, #(4 * REG_R11)]
	tmov		r1, r12
	tstorer		r1, [r0, #(4 * REG_R12)]

	/* R0 = interrupted task SP after the context save */

	tjl 		irq_handler

	/* R0 = interrupted task SP after the context save (no context switch)
	 *    = next task tcb->regs (with context switch)
	 * Restore all the register according to R0
	 */

	/* Dsiable interrupt to protect the SVC mode */

	tloadr		r2, _REG_IRQ_EN 		/* disable irq */
	tmov		r3, #0				/* disable irq */
	tstorerb	r3, [r2]			/* disable irq */

	/* Restore SVC mode SP (R13), SVC mode LR (R14, based saved PC)
	 * PC is not need to retore */

	tloadr		r2, [r0, #(4 * REG_SP)]
	tloadr		r3, [r0, #(4 * REG_PC)]

	tloadr		r1, _REG_IRQ_EN + 4		/* switch to SVC mode */
	tnop
	tmcsr		r1				/* switch to SVC mode */
	tnop
	tmov		r14, r3				/* restore SVC mode LR */
	tmov		r13, r2 			/* restore SVC mode SP */
	tloadr		r1, _REG_IRQ_EN + 8 		/* return to IRQ mode */
	tnop
	tmcsr		r1				/* return to IRQ mode */
	tnop

	/* Restore CPSR to SPSR, IRQ_STATE */

	tloadr		r2, [r0, #(4 * REG_CPSR)]
	tloadr		r3, [r0, #(4 * REG_IRQ_EN)]

	tmssr		r2				/* restore CPSR to SPSR */

	tloadr		r1, _REG_IRQ_EN 		/* restore IRQ enable flag */
	tstorerb	r3, [r1]			/* restore IRQ enable flag */

	/* Restore R8 ~ R12, IRQ mode LR */

	tloadr		r2, [r0, #(4 * REG_R8)]
	tloadr		r3, [r0, #(4 * REG_R9)]
	tloadr		r4, [r0, #(4 * REG_R10)]
	tloadr		r5, [r0, #(4 * REG_R11)]
	tloadr		r6, [r0, #(4 * REG_R12)]
	tloadr		r7, [r0, #(4 * REG_LR)]

	tmov		r8, r2
	tmov		r9, r3
	tmov		r10, r4
	tmov		r11, r5
	tmov		r12, r6
	tmov		r14, r7

	/* Restore R1 ~ R7 */

	tloadr		r1, [r0, #(4 * REG_R1)]
	tloadr		r2, [r0, #(4 * REG_R2)]
	tloadr		r3, [r0, #(4 * REG_R3)]
	tloadr		r4, [r0, #(4 * REG_R4)]
	tloadr		r5, [r0, #(4 * REG_R5)]
	tloadr		r6, [r0, #(4 * REG_R6)]
	tloadr		r7, [r0, #(4 * REG_R7)]

	/* Restore R0 */

	tloadr		r0, [r0, #(4 * REG_R0)]

	/* Return to interrupted task or next task */

	tpush		{r14}
	treti		{r15}

	.align		4
_REG_IRQ_EN:
	.word		0x00800643
	.word		0x00000093
	.word		0x00000092
	.size		tc32_exception, .-tc32_exception
	.end
