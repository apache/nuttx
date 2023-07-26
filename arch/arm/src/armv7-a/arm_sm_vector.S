/****************************************************************************
 * arch/arm/src/armv7-a/arm_vectorsm.S
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
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2016-2020, Linaro Limited
 * Copyright (c) 2014, STMicroelectronics International N.V.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "arm.h"
#include "cp15.h"
#include "sctlr.h"

	.file	"arm_vectorsm.S"

/****************************************************************************
 * Configuration
 ****************************************************************************/

#define SM_EXIT_TO_NON_SECURE       0
#define SM_CTX_SEC                  0   /* offsetof(struct arm_sm_ctx, sec) */
#define SM_CTX_NSEC                 112 /* offsetof(struct arm_sm_ctx, nsec) */
#define SM_CTX_SIZE                 244 /* sizeof(struct arm_sm_ctx) */
#define SM_CTX_SEC_SIZE             112 /* sizeof(struct arm_sm_sec_ctx) */
#define SM_CTX_SEC_END              (SM_CTX_SEC + SM_CTX_SEC_SIZE)
#define SM_SEC_CTX_R0               72  /* offsetof(struct arm_sm_sec_ctx, r0) */
#define SM_SEC_CTX_MON_LR           104 /* offsetof(struct arm_sm_sec_ctx, mon_lr) */
#define SM_NSEC_CTX_R0              92  /* offsetof(struct arm_sm_nsec_ctx, r0) */
#define SM_NSEC_CTX_R8              72  /* offsetof(struct arm_sm_nsec_ctx, r8) */

#define TEESMC_RETURN_ENTRY_DONE             0

/* Issued when returning from "std_smc" or "fast_smc" vector */
#define TEESMC_FUNCID_RETURN_CALL_DONE       1

/* Issued when returning from "fiq" vector */
#define TEESMC_RETURN_FIQ_DONE               2

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

.macro save_regs mode
	cps	\mode
	mrs	r2, spsr
	str	r2, [r0], #4
	str	sp, [r0], #4
	str	lr, [r0], #4
.endm

.macro restore_regs mode
	cps	\mode
	ldr	r2, [r0], #4
	ldr	sp, [r0], #4
	ldr	lr, [r0], #4
	msr	spsr_fsxc, r2
.endm

/****************************************************************************
 * Name: mov_imm
 *
 * Parameter:
 *   reg - the dest register to contains the immediate value
 *   val - the immediate value
 *
 * Description:
 *   Move the immediate value to the register, but this macro can handle the
 *   case that when the immediate value is larger than 2^16
 *
 ****************************************************************************/

.macro mov_imm reg, val
	.if ((\val) & 0xffff0000) == 0
		movw	\reg, #(\val)
	.else
		movw	\reg, #((\val) & 0xffff)
		movt	\reg, #((\val) >> 16)
	.endif
.endm

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text
	.syntax	unified
	.arm

/****************************************************************************
 * Name: arm_sm_vect_start
 *
 * Description:
 *   Secure monitor vector initialization block
 ****************************************************************************/

	.local	arm_sm_vect_start
	.balign 32

arm_sm_vect_start:
	ldr		pc, .				/* 0x00: Reset */
	ldr		pc, .				/* 0x04: Undefined instruction */
	ldr		pc, .Lsm_smchandler	/* 0x08: Secure monitor call */
	ldr		pc, .				/* 0x0c: Prefetch abort */
	ldr		pc, .				/* 0x10: Data abort */
	ldr		pc, .				/* 0x14: Reserved */
	ldr		pc, .				/* 0x18: IRQ */
	ldr		pc, .Lsm_fiqhandler	/* 0x1c: FIQ */

	.local	arm_sm_vectorsmc
	.local	arm_sm_vectorfiq

.Lsm_smchandler:
	.long	arm_sm_vectorsmc
.Lsm_fiqhandler:
	.long	arm_sm_vectorfiq

	.size	arm_sm_vect_start, . - arm_sm_vect_start
	.align	4

/****************************************************************************
 * Name: arm_sm_save_banked_regs
 ****************************************************************************/

	.globl	arm_sm_save_banked_regs
	.type	arm_sm_save_banked_regs, %function

arm_sm_save_banked_regs:
	/* User mode registers has to be saved from system mode */

	cps	#PSR_MODE_SYS
	str	sp, [r0], #4
	str	lr, [r0], #4

	save_regs	#PSR_MODE_IRQ
	save_regs	#PSR_MODE_FIQ
	save_regs	#PSR_MODE_SVC
	save_regs	#PSR_MODE_ABT
	save_regs	#PSR_MODE_UND

	cps	#PSR_MODE_MON
	bx	lr

	.size	arm_sm_save_banked_regs, . - arm_sm_save_banked_regs
	.align	4

/****************************************************************************
 * Name: arm_sm_restore_banked_regs
 ****************************************************************************/

/* Restores the mode specific registers */

	.globl	arm_sm_restore_banked_regs
	.type	arm_sm_restore_banked_regs, #function

arm_sm_restore_banked_regs:
	/* User mode registers has to be saved from system mode */

	cps	#PSR_MODE_SYS
	ldr	sp, [r0], #4
	ldr	lr, [r0], #4

	restore_regs	#PSR_MODE_IRQ
	restore_regs	#PSR_MODE_FIQ
	restore_regs	#PSR_MODE_SVC
	restore_regs	#PSR_MODE_ABT
	restore_regs	#PSR_MODE_UND

	cps	#PSR_MODE_MON
	bx	lr

	.size	arm_sm_restore_banked_regs, . - arm_sm_restore_banked_regs
	.align	4

/****************************************************************************
 * Name: arm_sm_vectorsmc
 ****************************************************************************/

/* the arm_sm_vectorsmc is called at "smc #0" is invoked. As each world can
 * trigger the smc call, so this method internal will detect which world
 * we come from:
 * if come from non-secure world, will save the non-secure
 * context, and restore the secure world context, and then call the
 * "arm_sm_from_nsec" to handle the following work, and after "arm_sm_from_nsec"
 * method is processing finished, will take the responsibility to return
 * to the non-secure world by calling "smc #0";
 * if come from secure world, will save the secure world context, and then
 * restore the non-secure world context, and then just perform "rfefd" to
 * exit the smc exception, and the non-secure world will resume, continue
 * running.
 * g_monitor_stack is used as stack, the top of the stack is reserved to hold
 * struct arm_sm_ctx, everything below is for normal stack usage. As several
 * different CPU modes are using the same stack it's important that switch
 * of CPU mode isn't done until one mode is done. This means FIQ, IRQ and
 * Async abort has to be masked while using g_monitor_stack.
 */

	.local	arm_sm_vectorsmc
	.type	arm_sm_vectorsmc, %function

arm_sm_vectorsmc:
	srsdb	sp!, #PSR_MODE_MON
	push	{r0-r7}

	/* Clear the exclusive monitor */

	clrex

	/* Find out if we're doing an secure or non-secure entry */

	mrc	CP15_SCR(r1)
	tst	r1, #SCR_NS
	bne	.smc_from_nsec

	/* As we're coming from secure world (NS bit cleared) the stack
	 * pointer points to arm_sm_ctx.sec.r0 at this stage. After the
	 * instruction below the stack pointer points to arm_sm_ctx.
	 */

	sub	sp, sp, #(SM_CTX_SEC + SM_SEC_CTX_R0)

	/* Save secure context */

	add	r0, sp, #SM_CTX_SEC

	/* the arm_sm_save_banked_regs() function will execute "cps #PSR_MODE_MON"
	 * and help switch to monitor mode
	 */
	bl	arm_sm_save_banked_regs

	/* On FIQ exit we're restoring the non-secure context unchanged, on
	 * all other exits we're shifting r1-r4 from secure context into
	 * r0-r3 in non-secure context.
	 */

	add	r8, sp, #(SM_CTX_SEC + SM_SEC_CTX_R0)
	ldm	r8, {r0-r4}
	mov_imm	r9, TEESMC_RETURN_FIQ_DONE
	cmp	r0, r9
	addne	r8, sp, #(SM_CTX_NSEC + SM_NSEC_CTX_R0)
	stmne	r8, {r1-r4}

	/* Restore non-secure context */

	add	r0, sp, #SM_CTX_NSEC
	bl	arm_sm_restore_banked_regs

.sm_ret_to_nsec:
	/* Return to non-secure world */

	add	r0, sp, #(SM_CTX_NSEC + SM_NSEC_CTX_R8)
	ldm	r0, {r8-r12}

	/* Update SCR */

	mrc	CP15_SCR(r0)
	orr	r0, r0, #(SCR_NS | SCR_FIQ) /* Set NS and FIQ bit in SCR */
	mcr	CP15_SCR(r0)

	/* isb not needed since we're doing an exception return below
	 * without dependency to the changes in SCR before that.
	 */

	add	sp, sp, #(SM_CTX_NSEC + SM_NSEC_CTX_R0)
	b	.sm_exit

.smc_from_nsec:
	/* As we're coming from non-secure world (NS bit set) the stack
	 * pointer points to arm_sm_ctx.nsec.r0 at this stage. After the
	 * instruction below the stack pointer points to arm_sm_ctx.
	 */

	sub	sp, sp, #(SM_CTX_NSEC + SM_NSEC_CTX_R0)

	bic	r1, r1, #(SCR_NS | SCR_FIQ) /* Clear NS and FIQ bit in SCR */
	mcr	CP15_SCR(r1)
	isb

	add	r0, sp, #(SM_CTX_NSEC + SM_NSEC_CTX_R8)
	stm	r0, {r8-r12}

	mov	r0, sp
	bl	arm_sm_from_nsec
	cmp	r0, #SM_EXIT_TO_NON_SECURE
	beq	.sm_ret_to_nsec

	/* Continue into secure world */

	add	sp, sp, #(SM_CTX_SEC + SM_SEC_CTX_R0)

.sm_exit:
	pop	{r0-r7}
	rfefd	sp!

	.size	arm_sm_vectorsmc, . - arm_sm_vectorsmc
	.align	4

/****************************************************************************
 * Name: arm_sm_vectorfiq
 ****************************************************************************/

/* FIQ handling
 *
 * Saves CPU context in the same way as arm_sm_vectorsmc() above. The CPU
 * context will later be restored by arm_sm_vectorsmc() when handling a return
 * from FIQ.
 *
 * When cpu is running in non-secure mode, then there are fiq is triggered
 * in secure mode, then the non-secure mode will need to transfer this fiq
 * to the secure mode, in order to implement this, this fiq will trapped into
 * monitor mode, and the monitor mode will invoke the arm_sm_vectorfiq func to
 * handle this fiq. The arm_sm_vectorfiq will save the non-secure context, and
 * then restore the non-secure context, and then exit current monitor mode,
 * and then will running the secure mode, then the secure mode will handle
 * this fiq
 */

	.local	arm_sm_vectorfiq
	.type	arm_sm_vectorfiq, %function

arm_sm_vectorfiq:
	/* FIQ has a +4 offset for lr compared to preferred return address */

	sub	lr, lr, #4
	/* sp points just past struct arm_sm_sec_ctx */

	srsdb	sp!, #PSR_MODE_MON
	push	{r0-r7}

	clrex /* Clear the exclusive monitor */

	/*
	 * As we're coming from non-secure world the stack pointer points
	 * to arm_sm_ctx.nsec.r0 at this stage. After the instruction below the
	 * stack pointer points to arm_sm_ctx.
	 */

	sub	sp, sp, #(SM_CTX_NSEC + SM_NSEC_CTX_R0)

	/* Update SCR */

	mrc	CP15_SCR(r1)
	bic	r1, r1, #(SCR_NS | SCR_FIQ) /* Clear NS and FIQ bit in SCR */
	mcr	CP15_SCR(r1)
	isb

	/* Save non-secure context */

	add	r0, sp, #SM_CTX_NSEC
	bl	arm_sm_save_banked_regs
	add	r0, sp, #(SM_CTX_NSEC + SM_NSEC_CTX_R8)
	stm	r0!, {r8-r12}

	/* Set FIQ entry */
	ldr	r0, =arm_vectorfiq_entry
	str	r0, [sp, #(SM_CTX_SEC + SM_SEC_CTX_MON_LR)]

	/* Restore secure context */

	add	r0, sp, #SM_CTX_SEC
	bl	arm_sm_restore_banked_regs

	/* return to the secure world */

	add	sp, sp, #(SM_CTX_SEC + SM_SEC_CTX_MON_LR)

	rfefd	sp!

	.size	arm_sm_vectorfiq, . - arm_sm_vectorfiq
	.align	4

/****************************************************************************
 * Name: arm_vectorsmc
 ****************************************************************************/

/* this function is declared in sm.h:
 * void arm_vectorsmc(uint32_t a0, uint32_t a1, uint32_t a2,
 *                    uint32_t a3, uint32_t a4, uint32_t a5,
 *                    uint32_t a6, uint32_t a7);
 * There are two ways to switch from REE to TEE, SMC exception and FIQ exception,
 * if we use FIQ exception to trap into the MONITOR mode, then before enter to
 * TEE mode, need to setup the ctx->sec.mon_lr as "arm_vectorfiq_entry", then the
 * "arm_vectorfiq_entry" is using as the entry to the TEE mode;
 * if we use SMC exception to trap into the MONITOR mode, then before enter to
 * TEE mode, need to setup the ctx->sec.mon_lr as "arm_vectorsmc", then
 * the "arm_vectorsmc" is using as the entry to the TEE mode;
 * the "arm_vectorsmc" is working same as "arm_vectorfiq_entry"
 */

	.global	arm_vectorsmc
	.type	arm_vectorsmc, %function

arm_vectorsmc:
	mov	r1, r0
	ldr	r0, =TEESMC_FUNCID_RETURN_CALL_DONE
	smc	#0
	/* SMC should not return */

	b	.

	.size	arm_vectorsmc, . - arm_vectorsmc
	.align	4

/****************************************************************************
 * Name: arm_vectorfiq_entry
 ****************************************************************************/

/* this function is declared in sm.h:
 * void arm_vectorfiq_entry(uint32_t a0, uint32_t a1, uint32_t a2,
 *                          uint32_t a3, uint32_t a4, uint32_t a5,
 *                          uint32_t a6, uint32_t a7);
 * Secure Monitor received a FIQ and passed control to us.
 * When the fiq is process finished in secure world, then the arm_vectorfiq_entry
 * function will continue running, then could perform some other actions on
 * arm_vectorfiq_entry. At the end of the arm_vectorfiq_entry, will call smc to
 * switch to the non-secure world
 */

	.local	arm_vectorfiq_entry
	.type	arm_vectorfiq_entry, %function

arm_vectorfiq_entry:
	ldr	r0, =TEESMC_RETURN_FIQ_DONE
	smc	#0
	/* SMC should not return */

	b	.

	.size	arm_vectorfiq_entry, . - arm_vectorfiq_entry
	.align	4

/****************************************************************************
 * Name: arm_sm_init_stack
 ****************************************************************************/

/* this function is declared in sm.h
 * void arm_sm_init_stack(int stack);
 *
 * this function is using to init the monitor runtime
 * this function will write the adress of arm_sm_vect_start
 * to the mvbar register
 */

	.globl	arm_sm_init_stack
	.type	arm_sm_init_stack, %function

arm_sm_init_stack:
	/* Set monitor stack */

	mrs	r1, cpsr
	cps	#PSR_MODE_MON

	/* Point just beyond arm_sm_ctx.sec */

	sub	sp, r0, #0
	msr	cpsr, r1

	/* Set monitor vector (MVBAR) */

1:
	ldr	r0, =arm_sm_vect_start
2:
	mcr	CP15_MVBAR(r0)

	bx	lr

	.size	arm_sm_init_stack, . - arm_sm_init_stack
	.align	4

/****************************************************************************
 * Name: arm_sm_get_nsec_ctx
 ****************************************************************************/

/* This function is declared in sm.h:
 * struct arm_sm_nsec_ctx *arm_sm_get_nsec_ctx(void);
 *
 * this function are using to fetch the non-secure context
 * the non-secure context are stored in g_monitor_stack
 */

	.globl	arm_sm_get_nsec_ctx
	.type	arm_sm_get_nsec_ctx, %function

arm_sm_get_nsec_ctx:
	mrs	r1, cpsr
	cps	#PSR_MODE_MON

	/* As we're in secure mode mon_sp points just beyond arm_sm_ctx.sec,
	 * which allows us to calculate the address of arm_sm_ctx.nsec.
	 */

	add	r0, sp, #0
	msr	cpsr, r1

	bx	lr

	.size	arm_sm_get_nsec_ctx, . - arm_sm_get_nsec_ctx
	.align	4

/****************************************************************************
 * Name: arm_sm_switch_nsec
 ****************************************************************************/

/* this function is declared in sm.h:
 * void arm_sm_switch_nsec(void);
 *
 * This simple method using to switch to non-secure world at system boot, the
 * switch is simply by calling "smc, #0"
 */

	.globl	arm_sm_switch_nsec
	.type	arm_sm_switch_nsec, %function

arm_sm_switch_nsec:
	ldr	r0, =TEESMC_RETURN_ENTRY_DONE
	smc	#0

	/* SMC should not return */

	b	.

	.size	arm_sm_switch_nsec, . - arm_sm_switch_nsec
	.align	4
