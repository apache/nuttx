/****************************************************************************
 * arch/tricore/src/common/tricore_head.S
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

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 32-bit address to address register */

.macro mov32a areg, val
	movh.a	\areg, hi:\val
	lea	\areg, [\areg]lo:\val
.endm

/* 32-bit address to data register */

.macro mov32d dreg, val
	movh	\dreg, hi:\val
	addi	\dreg, \dreg, lo:\val
.endm

/* address to pcxi word */

.macro addr2csa out, in
	sh	\out, \in, -12
	movh	%d4, 15
	and	\out, %d4
	extr.u	%d4, \in, 6, 16
	or	\out, %d4
.endm

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Per-core istack and ustack, indexed by coreid. */

	.section .rodata, "a"
	.align 4
	.global	__ustack_table
	.global	__istack_table

__ustack_table:
#ifndef CONFIG_SMP
	.word	__USTACK
#else
	.word	__USTACK0
#if CONFIG_SMP_NCPUS > 1
	.word	__USTACK1
#if CONFIG_SMP_NCPUS > 2
	.word	__USTACK2
#if CONFIG_SMP_NCPUS > 3
	.word	__USTACK3
#if CONFIG_SMP_NCPUS > 4
	.word	__USTACK4
#if CONFIG_SMP_NCPUS > 5
	.word	__USTACK5
#endif
#endif
#endif
#endif
#endif
#endif

__istack_table:
#ifndef CONFIG_SMP
	.word	__ISTACK
#else
	.word	__ISTACK0
#if CONFIG_SMP_NCPUS > 1
	.word	__ISTACK1
#if CONFIG_SMP_NCPUS > 2
	.word	__ISTACK2
#if CONFIG_SMP_NCPUS > 3
	.word	__ISTACK3
#if CONFIG_SMP_NCPUS > 4
	.word	__ISTACK4
#if CONFIG_SMP_NCPUS > 5
	.word	__ISTACK5
#endif
#endif
#endif
#endif
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_virtualization_exit
 ****************************************************************************/

#ifndef CONFIG_ARCH_CHIP_TC3XX
	.text
	.type	tricore_virtualization_exit, %function
tricore_virtualization_exit:
	mfcr	%d8, CPU_CORE_ID
	extr.u	%d8, %d8, 8, 3
	jnz	%d8, 1f
	mtcr	CPU_VCON0, %d8
	mov	%d8, TRICORE_PSW_IS
	mtcr	$psw, %d8
	isync
	rfh
1:
	ret
	.size	tricore_virtualization_exit, . - tricore_virtualization_exit
#endif

/****************************************************************************
 * Name: tricore_cinit
 *
 * Description:
 *   Clear the bss sections and copy the data sections
 *
 ****************************************************************************/

	.type	tricore_cinit, %function
tricore_cinit:
	mov32a	%a12, __clear_table
	mov	%e12, 0
.Lclr_entry:
	ld.a	%a13, [%a12+]4
	ld.w	%d8, [%a12+]4
	jeq	%d8, -1, .Lclr_done
	mov.d	%d9, %a13
	add	%d9, %d9, %d8
.Lclr_dw:					/* double-word blocks */
	mov.d	%d10, %a13
	add	%d11, %d10, 8
	jlt.u	%d9, %d11, .Lclr_b
	st.d	[%a13+]8, %e12
	j	.Lclr_dw
.Lclr_b:					/* byte tail */
	mov.d	%d10, %a13
	jge.u	%d10, %d9, .Lclr_entry
	st.b	[%a13+]1, %d12
	j	.Lclr_b
.Lclr_done:

	mov32a	%a12, __copy_table
.Lcpy_entry:
	ld.a	%a14, [%a12+]4
	ld.a	%a13, [%a12+]4
	ld.w	%d8, [%a12+]4
	jeq	%d8, -1, .Lcpy_done
	mov.d	%d9, %a13
	add	%d9, %d9, %d8
.Lcpy_dw:					/* double-word blocks */
	mov.d	%d10, %a13
	add	%d11, %d10, 8
	jlt.u	%d9, %d11, .Lcpy_b
	ld.d	%e12, [%a14+]8
	st.d	[%a13+]8, %e12
	j	.Lcpy_dw
.Lcpy_b:					/* byte tail */
	mov.d	%d10, %a13
	jge.u	%d10, %d9, .Lcpy_entry
	ld.bu	%d12, [%a14+]1
	st.b	[%a13+]1, %d12
	j	.Lcpy_b
.Lcpy_done:
	ret
	.size	tricore_cinit, . - tricore_cinit

/****************************************************************************
 * Name: tricore_csainit
 ****************************************************************************/

	.type	tricore_csainit, %function
tricore_csainit:
1:
	add	%d5, %d1, TC_CONTEXT_SIZE
	jge.u	%d5, %d2, 2f
	addr2csa	%d6, %d5
	mov.a	%a2, %d1
	st.w	[%a2], %d6
	mov	%d1, %d5
	j	1b
2:
	ji	%a11
	.size	tricore_csainit, . - tricore_csainit

/****************************************************************************
 * Name: tricore_boot_cpu
 *
 * Description:
 *   Bring up this cpu core registers and chain the csas.
 *
 ****************************************************************************/

	.type	tricore_boot_cpu, %function
tricore_boot_cpu:
	mov.aa	%a14, %a11			/* save return address (jli inside) */

	mov.d	%d0, %a13
	mtcr	$isp, %d0			/* isp = istack */

	mov32d	%d0, __TRAPTAB_CPU
	mtcr	$btv, %d0			/* btv = traptab_cpu */

	mov32d	%d0, __INTTAB_CPU
	mtcr	$biv, %d0			/* biv = inttab_cpu */

	mov	%d0, TRICORE_PSW_IS
	mtcr	$psw, %d0			/* psw */
	isync

#ifdef CONFIG_ARCH_BIST
	/* bist_k1: bist only the ram needed to boot (dspr and friends),
	 * borrow the pspr scratch area since the test destroys ram.
	 */

	mov32d	%d0, PSPR_USTACK
	mov.a	%a10, %d0			/* sp = pspr scratch */

	mov32d	%d1, PSPR_CSA			/* d1 = csa base */
	mov32d	%d2, PSPR_CSA + PSPR_SIZE	/* d2 = csa end */

	addr2csa	%d3, %d1
	mtcr	$fcx, %d3			/* fcx */

	addi	%d0, %d2, -FCD_RESERVE_SIZE
	addr2csa	%d3, %d0
	mtcr	$lcx, %d3			/* lcx */

	mov32a	%a2, tricore_csainit
	jli	%a2				/* csa list init */

	call	tricore_bist_k1
#endif

	mov.aa	%a10, %a12			/* sp = ustack */

	mov32d	%d1, __CSA			/* d1 = csa base */
	mov32d	%d2, __CSA_END			/* d2 = csa end */

	addr2csa	%d3, %d1
	mtcr	$fcx, %d3			/* fcx */

	addi	%d0, %d2, -FCD_RESERVE_SIZE
	addr2csa	%d3, %d0
	mtcr	$lcx, %d3			/* lcx */

	mov32a	%a2, tricore_csainit
	jli	%a2				/* csa list init */

#ifdef CONFIG_ARCH_BIST
	/* bist_k2: bist the remaining ram */

	call	tricore_bist_k2
#endif
	ji	%a14
	.size	tricore_boot_cpu, . - tricore_boot_cpu

/****************************************************************************
 * Name: __start_cpu
 *
 * Description:
 *   Common entry of all cores
 *
 ****************************************************************************/

	.type	__start_cpu, %function
__start_cpu:

	mov32a	%a12, __ustack_table
	mov32a	%a13, __istack_table

	/* index the per-core tables by coreid */

#if defined(CONFIG_SMP)
	mfcr	%d8, CPU_CORE_ID
	extr.u	%d8, %d8, 0, 3
#ifdef CONFIG_ARCH_CHIP_TC3XX
	min.u	%d8, %d8, 5
#endif
	addsc.a	%a12, %a12, %d8, 2
	addsc.a	%a13, %a13, %d8, 2
#endif
	ld.a	%a12, [%a12]
	ld.a	%a13, [%a13]

	/* virtualization environment exit(call and rfh must be used in pairs) */

#ifndef CONFIG_ARCH_CHIP_TC3XX
	mov.aa	%a10, %a12			/* sp = ustack */
	mov.d	%d9, %a12
	addi	%d9, %d9, -CONFIG_IDLETHREAD_STACKSIZE
	addr2csa	%d10, %d9
	mtcr	$fcx, %d10			/* fcx */
	isync
	call	tricore_virtualization_exit
#endif

	/* Bring up this cpu core registers and chain the csas */

	mov32a	%a2, tricore_boot_cpu
	jli	%a2

	/* Icache and Dcache enable */

#ifdef CONFIG_ARCH_ICACHE
	call	up_invalidate_icache_all
	call	up_enable_icache
#endif
#ifdef CONFIG_ARCH_DCACHE
	call	up_invalidate_dcache_all
	call	up_enable_dcache
#endif

#if defined(CONFIG_SMP)
	jnz	%d8, 1f
#endif

	/* Clear bss section and copy data section */

	call	tricore_cinit

	call	tricore_boot_primary
	j	2f

#if defined(CONFIG_SMP)
1:
	call	tricore_boot_secondary
#endif
2:
	j	2b				/* should never get here */
	.size	__start_cpu, . - __start_cpu

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 ****************************************************************************/

	.section .start, "ax"
	.global	__start
	.type	__start, %function
__start:
	mov32a	%a2, __start_cpu
	ji	%a2
	.size	__start, . - __start
