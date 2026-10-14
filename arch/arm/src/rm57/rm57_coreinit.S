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

/* arm_boot is the first RM57 code to run after reset: armv7-r/arm_head.S
 * branches (not calls) here once it has loaded the SYS mode stack pointer
 * and written SCTLR.
 *
 * RM57L843 runs two Cortex-R5F cores in lockstep, and the CPU Compare
 * Module (CCM-R5F) flags any difference between their bus outputs.  The
 * core registers do not come out of reset with defined values, so the two
 * cores can hold different garbage, and the first time such a register
 * reaches a bus - a push in a C function prologue is enough - the compare
 * fails (SPNU562A 13.2.1.1).  Every register therefore has to be written
 * before any C code runs, which is why this is the assembly entry point
 * and the C part of the boot sequence is rm57_boot_c().  arm_head.S itself
 * is safe: it only ever writes registers before reading them and never
 * touches the stack.
 *
 * Modelled on TI's HALCoGen _coreInitRegisters_, extended
 * to the banked stack pointers and link registers as TI's SafeTI library
 * does, and to FPSCR.  SPSR exists only in the exception modes, so unlike
 * HALCoGen (which starts out in SVC mode) it is not written in SYS mode.
 */

#include <nuttx/config.h>

	.syntax unified
	.arm
	.text

	.global	arm_boot
	.global	rm57_boot_c
	.type	arm_boot, %function

arm_boot:

	/* SYS/USR mode registers.  sp was loaded by arm_head.S and is kept. */

	mov	r0, #0
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
	mov	lr, #0

	/* A defined value for every SPSR: the current CPSR (SYS mode, IRQ and
	 * FIQ masked), which arm_head.S established with a cpsid.
	 */

	mrs	r1, cpsr

	cps	#0x11			/* FIQ mode */
	mov	r8, #0
	mov	r9, #0
	mov	r10, #0
	mov	r11, #0
	mov	r12, #0
	mov	sp, #0
	mov	lr, #0
	msr	spsr_cxsf, r1

	cps	#0x12			/* IRQ mode */
	mov	sp, #0
	mov	lr, #0
	msr	spsr_cxsf, r1

	cps	#0x13			/* Supervisor mode */
	mov	sp, #0
	mov	lr, #0
	msr	spsr_cxsf, r1

	cps	#0x17			/* Abort mode */
	mov	sp, #0
	mov	lr, #0
	msr	spsr_cxsf, r1

	cps	#0x1b			/* Undefined mode */
	mov	sp, #0
	mov	lr, #0
	msr	spsr_cxsf, r1

	cps	#0x1f			/* Back to System mode */

#ifdef CONFIG_ARCH_FPU
	/* The FPU register file is compared as well.  Grant CP10/CP11 access
	 * and set FPEXC.EN so that it can be written; arm_fpuconfig() repeats
	 * the enable later, which is harmless.
	 */

	mrc	p15, 0, r2, c1, c0, 2
	orr	r2, r2, #0x00f00000
	mcr	p15, 0, r2, c1, c0, 2
	isb
	mov	r2, #0x40000000
	vmsr	fpexc, r2

	mov	r2, #0
	vmsr	fpscr, r2
	vmov	d0, r2, r2
	vmov	d1, r2, r2
	vmov	d2, r2, r2
	vmov	d3, r2, r2
	vmov	d4, r2, r2
	vmov	d5, r2, r2
	vmov	d6, r2, r2
	vmov	d7, r2, r2
	vmov	d8, r2, r2
	vmov	d9, r2, r2
	vmov	d10, r2, r2
	vmov	d11, r2, r2
	vmov	d12, r2, r2
	vmov	d13, r2, r2
	vmov	d14, r2, r2
	vmov	d15, r2, r2
#endif

	/* Prime the four-entry return stack so that both cores predict the
	 * same (known) addresses for the first function returns.
	 */

	bl	1f
1:
	bl	2f
2:
	bl	3f
3:
	bl	4f
4:
	mov	r1, #0
	mov	r2, #0
	mov	lr, #0

	/* Continue in C.  rm57_boot_c() does not return. */

	b	rm57_boot_c

	.size	arm_boot, . - arm_boot
	.end
