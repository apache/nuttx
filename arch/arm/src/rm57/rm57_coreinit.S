/****************************************************************************
 * arch/arm/src/rm57/rm57_coreinit.S
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
 ****************************************************************************/

/* Ported from TI's HALCoGen _coreInitRegisters_ (HL_sys_core.s),
 * which that file documents as required "to avoid CCM Error". RM57L843
 * is a lockstep dual-CPU part: if the two redundant cores' register
 * files hold different power-on-reset garbage, the CPU Compare Module
 * (CCM) can flag an immediate mismatch and force a hardware reset that
 * bypasses all software exception handling - confirmed on real
 * hardware as the cause of an early boot reset loop that occurred even
 * after the exception vector table itself was fixed (see
 * boards/.../scripts/flash-sram.ld and its EXTERN(_vector_start) fix).
 *
 * Must run as early as possible in boot - called as the first thing
 * arm_boot() does, before any C code that could cause the two lockstep
 * cores' internal state to diverge.
 */

	.syntax unified
	.text

	.global	rm57_core_init_registers
	.type	rm57_core_init_registers, %function

rm57_core_init_registers:

	mov	r0, lr			/* Preserve the return address across mode switches */

	mov	r1, #0
	mov	r2, #0
	mov	r3, #0
	mov	r4, #0
	mov	r5, #0
	mov	r6, #0
	mov	r7, #0
	mov	r8, #0
	mov	r9, #0
	mov	r10, #0
	mov	r11, #0
	mov	r12, #0
	mrs	r1, cpsr
	msr	spsr_cxsf, r1

	cps	#0x11			/* FIQ mode */
	mov	lr, r0
	mov	r8, #0
	mov	r9, #0
	mov	r10, #0
	mov	r11, #0
	mov	r12, #0
	mrs	r1, cpsr
	msr	spsr_cxsf, r1

	cps	#0x12			/* IRQ mode */
	mov	lr, r0
	mrs	r1, cpsr
	msr	spsr_cxsf, r1

	cps	#0x17			/* Abort mode */
	mov	lr, r0
	mrs	r1, cpsr
	msr	spsr_cxsf, r1

	cps	#0x1b			/* Undefined mode */
	mov	lr, r0
	mrs	r1, cpsr
	msr	spsr_cxsf, r1

	cps	#0x13			/* Supervisor mode */
	mov	lr, r0
	mrs	r1, cpsr
	msr	spsr_cxsf, r1

	cps	#0x1f			/* Back to System mode (the caller's mode) */
	mov	lr, r0
	bx	lr

	.size	rm57_core_init_registers, . - rm57_core_init_registers
	.end
