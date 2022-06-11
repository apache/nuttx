/****************************************************************************
 * arch/arm/src/tlsr82/tc32/tc32_saveusercontext.S
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file	"tc32_saveusercontext.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_saveusercontext
 *
 * Description:
 *   Restore the current thread context.  Full prototype is:
 *
 *   int arm_saveusercontext(uint32_t *saveregs);
 *
 *   R0 = saveregs = pinter saved array
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.align	2
	.code	16
	.thumb_func
	.section .ram_code,"ax"
	.global	up_saveusercontext
	.type	up_saveusercontext, function
up_saveusercontext:

	/*
	 * r0 = pointer to the saveregs array
	 * Save r0 ~ r7 into the saved register array
	 */

	tstorer		r0, [r0, #64]
	tstorer		r1, [r0, #60]
	tstorer		r2, [r0, #56]
	tstorer		r3, [r0, #52]
	tstorer		r4, [r0, #48]
	tstorer		r5, [r0, #44]
	tstorer		r6, [r0, #40]
	tstorer		r7, [r0, #36]

	/* Save r14 (lr) into the saved register array */

	tmov		r1, r14
	tstorer		r1, [r0, #68]

	/* Get CPSR and save it into the saved register array */

	tmrcs		r1
	tstorer		r1, [r0, #32]

	/* Save IRQ_STATE into the saved register array */

	tloadr 		r1, _REG_IRQ_EN1
	tloadrb		r2, [r1]
	tstorer		r2, [r0, #28]

	/* Save r8 ~ r14 into the saved register array, r13(sp) r14 (lr/pc) */

	tmov		r1, r8
	tstorer		r1, [r0, #24]
	tmov		r1, r9
	tstorer		r1, [r0, #20]
	tmov		r1, r10
	tstorer		r1, [r0, #16]
	tmov		r1, r11
	tstorer		r1, [r0, #12]
	tmov		r1, r12
	tstorer		r1, [r0, #8]
	tmov		r1, r13
	tstorer		r1, [r0, #4]
	tmov		r1, r14
	tstorer		r1, [r0, #0]

	/* Return 0 */

	tmov		r0, #0
	tjex		lr

	.align 		2
_REG_IRQ_EN1:
	.word		0x00800643
	.size		up_saveusercontext, .-up_saveusercontext
	.end

