/****************************************************************************
 * arch/arm/src/tlsr82/tc32/tc32_fullcontextrestore.S
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
#include <arch/chip/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file	"tc32_fullcontextrestore.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_fullcontextrestore
 *
 * Description:
 *   Restore the current thread context.  Full prototype is:
 *
 *   void arm_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.align	2
	.code	16
	.thumb_func
	.section .ram_code,"ax"
	.global	arm_fullcontextrestore
	.type	arm_fullcontextrestore, function
arm_fullcontextrestore:

	/* Move saved register to register R1 ~ R7
	 * R8       --> R1
	 * R9       --> R2
	 * R10      --> R3
	 * R11      --> R4
	 * R12      --> R5
	 * R13 (SP) --> R6
	 * R15 (PC) --> R7
	 */

	tloadr 		r1, [r0, #(4 * REG_R8)]
	tloadr 		r2, [r0, #(4 * REG_R9)]
	tloadr 		r3, [r0, #(4 * REG_R10)]
	tloadr 		r4, [r0, #(4 * REG_R11)]
	tloadr 		r5, [r0, #(4 * REG_R12)]
	tloadr 		r6, [r0, #(4 * REG_R13)]
	tloadr 		r7, [r0, #(4 * REG_R15)]

	/* Restore register R8 ~ R12 */

	tmov		r8,  r1
	tmov		r9,  r2
	tmov		r10, r3
	tmov		r11, r4
	tmov		r12, r5

	/* Restore register SP (SVC mode)*/

	tmov		r13, r6

	/* Restore LR (R14, SVC mode) as PC (R15), not accurate though */

	tmov		r14, r7

	/* Set CPSR be 0x00000092 to switch to IRQ mode */

	tloadr 		r1, _REG_IRQ_EN2 + 4
	tnop
	tmcsr		r1
	tnop

	/* Get the saved IRQ_STATE and restore to [0x00800643] */

	tloadr 		r1, _REG_IRQ_EN2
	tloadr 		r2, [r0, #(4 * REG_IRQ_EN)]
	tstorerb	r2, [r1]

	/* Get the saved CPSR and restore to SPSR, when return
	 * to SVC mode, the value of CPSR will be automatically
	 * updated to the value of SPSR.
	 */

	tloadr 		r1,  [r0, #(4 * REG_CPSR)]
	tmssr		r1

	/* Get the saved R14 (LR, IRQ mode), clear the bit0 in LR to make
	 * return address be even, then restore it to LR
	 */

	tloadr 		r1, [r0, #(4 * REG_R14)]
	tloadr 		r2, _REG_IRQ_EN2 + 8
	tand 		r1, r2
	tmov		r14, r1

	/* Restore saved registers R1 ~ R7 */

	tloadr 		r1, [r0, #(4 * REG_R1)]
	tloadr 		r2, [r0, #(4 * REG_R2)]
	tloadr 		r3, [r0, #(4 * REG_R3)]
	tloadr 		r4, [r0, #(4 * REG_R4)]
	tloadr 		r5, [r0, #(4 * REG_R5)]
	tloadr 		r6, [r0, #(4 * REG_R6)]
	tloadr 		r7, [r0, #(4 * REG_R7)]

	/* Restore saved register R0 */

	tloadr 		r0, [r0, #(4 * REG_R0)]

	/* Push the R14 and pop to pc, return from IRQ mode */

	tpush		{r14}
	treti		{r15}

	.align 		2
_REG_IRQ_EN2:
	.word		0x00800643
	.word		0x00000092
	.word		0xfffffffe
	.size		arm_fullcontextrestore, .-arm_fullcontextrestore
	.end
