/****************************************************************************
 * arch/arm/src/pp502x/pp502x_head.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm.h"
#include "chip.h"
#include "arm_internal.h"

	.file	"pp502x_head.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REMAP0_PHYSICAL  (PP502X_SDRAM_PHYS | PP502X_REMAP_PHYS_SDRAM_FLAGS)

/****************************************************************************
 * Text
 ****************************************************************************/

	.syntax	unified
	.arm

/* The image is linked at logical address 0 with this section first, so the
 * vector table below is also the one the core fetches once window 0 maps
 * SDRAM at 0.  Everything up to .Lrelocated runs at the load address,
 * which can differ from the link address, and must be position
 * independent.
 */

	.section .vectors, "ax"

	.globl	_vector_start
	.globl	_vector_end
	.globl	pp502x_start

/****************************************************************************
 * Name: _vector_start
 *
 * Description:
 *   Exception vectors.  Every slot is a relative branch: the loader enters
 *   the first word of the image wherever it placed it, and loaders find
 *   the start of an image by its two leading branch instructions.
 *
 ****************************************************************************/

_vector_start:
	b	pp502x_start			/* 0x00: Reset */
	b	arm_vectorundefinsn		/* 0x04: Undefined instruction */
	b	arm_vectorsvc			/* 0x08: Software interrupt */
	b	arm_vectorprefetch		/* 0x0c: Prefetch abort */
	b	arm_vectordata			/* 0x10: Data abort */
	b	arm_vectoraddrexcptn		/* 0x14: Address exception */
	b	arm_vectorirq			/* 0x18: IRQ */
	b	arm_vectorfiq			/* 0x1c: FIQ */

_vector_end:

/****************************************************************************
 * Name: pp502x_start
 *
 * Description:
 *   Entry point for both cores.  The COP masks its interrupts and sleeps
 *   for good.  The CPU masks its interrupts, waits for the COP to sleep
 *   (for a bounded time), turns off the cache and the local vector table,
 *   sizes the SDRAM, maps SDRAM at logical 0, moves the image to the start
 *   of SDRAM if it was loaded elsewhere, and continues at its link address,
 *   where it turns the cache on again.
 *
 ****************************************************************************/

	.type	pp502x_start, %function

pp502x_start:
	mov	r0, #(PSR_MODE_SVC | PSR_I_BIT | PSR_F_BIT)
	msr	cpsr_c, r0

	ldr	r0, =PP502X_PROC_ID
	ldrb	r0, [r0]
	cmp	r0, #PP502X_PROCID_CPU
	beq	.Lcpu

	mvn	r2, #0
	ldr	r1, =PP502X_INTC_COP_DIS
	str	r2, [r1]
	ldr	r1, =PP502X_INTC_HI_COP_DIS
	str	r2, [r1]

	ldr	r1, =PP502X_COP_CTL
	mov	r2, #PP502X_PROCCTL_SLEEP
.Lcopsleep:
	str	r2, [r1]
	nop
	nop
	nop
	nop
	b	.Lcopsleep

.Lcpu:
	mvn	r2, #0
	ldr	r1, =PP502X_INTC_CPU_DIS
	str	r2, [r1]
	ldr	r1, =PP502X_INTC_HI_CPU_DIS
	str	r2, [r1]

	/* The wait is bounded: a COP that never sleeps must not hang the
	 * boot before any output.
	 */

	ldr	r1, =PP502X_COP_CTL
	ldr	r3, =PP502X_COP_WAIT_COUNT
.Lwaitcop:
	ldr	r2, [r1]
	tst	r2, #PP502X_PROCCTL_SLEEP
	bne	.Lcopwaited
	subs	r3, r3, #1
	bne	.Lwaitcop
.Lcopwaited:

	ldr	r1, =PP502X_CACHE_CTL
	mov	r2, #0
	str	r2, [r1]

	/* Size the SDRAM.  A 32 MiB part repeats in the upper half of the
	 * 64 MiB window, so a word written there reads back at the same offset
	 * in the lower half.  Both words are restored.  fp keeps the size
	 * until .Lremapped.
	 */

	ldr	r1, =(PP502X_SDRAM_PHYS + PP502X_SDRAM_PROBE_OFFSET)
	ldr	r3, =PP502X_SDRAM_SIZE_32M
	ldr	r4, [r1]
	ldr	r5, [r1, r3]
	ldr	r6, =PP502X_SDRAM_PROBE_LOW
	ldr	r7, =PP502X_SDRAM_PROBE_HIGH
	str	r6, [r1]
	str	r7, [r1, r3]
	ldr	r2, [r1]
	cmp	r2, r7
	str	r5, [r1, r3]
	str	r4, [r1]
	ldr	fp, =PP502X_SDRAM_SIZE_64M
	ldr	r2, =PP502X_REMAP_MASK_64MB
	ldreq	fp, =PP502X_SDRAM_SIZE_32M
	ldreq	r2, =PP502X_REMAP_MASK_32MB

	/* Window 0 only translates logical addresses below the SDRAM size,
	 * so this code, which runs at its physical address, is not moved by
	 * it.
	 */

	ldr	r1, =PP502X_REMAP_LOGICAL(0)
	str	r2, [r1]
	ldr	r2, =REMAP0_PHYSICAL
	str	r2, [r1, #4]

	/* Copy downwards to the start of SDRAM.  The copy is safe as long as
	 * the load address is at least the image size above the destination.
	 */

	adr	r0, _vector_start
	ldr	r1, =PP502X_SDRAM_PHYS
	cmp	r0, r1
	cmpne	r0, #0
	beq	.Lrelocated

	ldr	r2, =_edata
	add	r2, r1, r2
.Lcopy:
	ldmia	r0!, {r3-r10}
	stmia	r1!, {r3-r10}
	cmp	r1, r2
	blo	.Lcopy

.Lrelocated:
	ldr	pc, =.Lremapped

	.ltorg
	.size	pp502x_start, . - pp502x_start

/****************************************************************************
 * Name: .Lremapped
 *
 * Description:
 *   Runs at the link address: store the SDRAM size, turn the cache on and
 *   set up the clocks and the console UART on a temporary stack (neither
 *   call uses static data), then continue in the common entry point, which
 *   sets the idle stack, clears BSS and starts the OS.
 *
 ****************************************************************************/

	.text

.Lremapped:
	ldr	r0, =g_pp502x_ramsize
	str	fp, [r0]

	ldr	sp, =(_ebss + CONFIG_IDLETHREAD_STACKSIZE)

	bl	pp502x_cache_init
	bl	pp502x_lowsetup

	b	__start

	.ltorg
	.end
