;****************************************************************************
; arch/tricore/src/common/tricore_head.asm
;
; SPDX-License-Identifier: Apache-2.0
;
; Licensed to the Apache Software Foundation (ASF) under one or more
; contributor license agreements.  See the NOTICE file distributed with
; this work for additional information regarding copyright ownership.  The
; ASF licenses this file to you under the Apache License, Version 2.0 (the
; "License"); you may not use this file except in compliance with
; the License.  You may obtain a copy of the License at
;
;   http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
; WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.
;
;****************************************************************************

;****************************************************************************
; Included Files
;****************************************************************************

#include <nuttx/config.h>

#include <arch/irq.h>

;****************************************************************************
; Pre-processor Definitions
;****************************************************************************

; 32-bit address to address register

mov32a	.macro	areg, val
	movh.a	areg, #@his(val)
	lea	areg, [areg]@los(val)
.endm

; 32-bit address to data register

mov32d	.macro	dreg, val
	movh	dreg, #@his(val)
	addi	dreg, dreg, #@los(val)
.endm

; address to pcxi word

addr2csa	.macro	out, in
	sh	out, in, #-12
	movh	d4, #15
	and	out, d4
	extr.u	d4, in, #6, #16
	or	out, d4
.endm

;****************************************************************************
; Private Data
;****************************************************************************

; Per-core istack and ustack, indexed by coreid.

	.sdecl	'.rodata',data
	.sect	'.rodata'
	.align	4
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

;****************************************************************************
; Public Data
;****************************************************************************

.extern	_lc_ub_table

;****************************************************************************
; Public Function Prototypes
;****************************************************************************

#ifdef CONFIG_ARCH_ICACHE
	.extern	up_invalidate_icache_all, up_enable_icache
#endif
#ifdef CONFIG_ARCH_DCACHE
	.extern	up_invalidate_dcache_all, up_enable_dcache
#endif
	.extern	tricore_boot_primary
#if defined(CONFIG_SMP)
	.extern	tricore_boot_secondary
#endif
#ifdef CONFIG_ARCH_BIST
	.extern	tricore_bist_k1, tricore_bist_k2
#endif

;****************************************************************************
; Private Functions
;****************************************************************************

;****************************************************************************
; Name: tricore_virtualization_exit
;****************************************************************************

	.sdecl	'.text',code
	.sect	'.text'
#ifndef CONFIG_ARCH_CHIP_TC3XX
tricore_virtualization_exit:	.type	func
	mfcr	d8, #CPU_CORE_ID
	extr.u	d8, d8, #8, #3
	jnz	d8, 1n
	mtcr	#CPU_VCON0, d8
	mov	d8, #TRICORE_PSW_IS
	mtcr	#CPU_PSW, d8
	isync
	rfh
1:
	ret
	.size	tricore_virtualization_exit, *-tricore_virtualization_exit
#endif

;****************************************************************************
; Name: tricore_cinit
;
; Description:
;   Clear the bss sections and copy the data sections.
;
;****************************************************************************

tricore_cinit:	.type	func
	mov32a	a12, _lc_ub_table
	mov	e12, #0
.Lcinit_entry:
	ld.w	d15, [a12+]4			; action | write_unit << 16
	ld.a	a13, [a12+]4			; dst
	ld.a	a14, [a12+]4			; src
	ld.w	d8, [a12+]4			; size
	jz	d15, .Lcinit_done
	mov.d	d9, a13
	add	d9, d9, d8			; d9 = dst end
	jeq	d15, #1, .Lcpy_dw		; copy
	jeq	d15, #2, .Lclr_dw		; clear
.Lcinit_bad:
	j	.Lcinit_bad			; write_unit != 0 or unknown action

.Lclr_dw:					; double-word blocks
	mov.d	d10, a13
	add	d11, d10, #8
	jlt.u	d9, d11, .Lclr_b
	st.d	[a13+]8, e12
	j	.Lclr_dw
.Lclr_b:					; byte tail
	mov.d	d10, a13
	jge.u	d10, d9, .Lcinit_entry
	st.b	[a13+]1, d12
	j	.Lclr_b

.Lcpy_dw:					; double-word blocks
	mov.d	d10, a13
	add	d11, d10, #8
	jlt.u	d9, d11, .Lcpy_b
	ld.d	e10, [a14+]8
	st.d	[a13+]8, e10
	j	.Lcpy_dw
.Lcpy_b:					; byte tail
	mov.d	d10, a13
	jge.u	d10, d9, .Lcinit_entry
	ld.bu	d11, [a14+]1
	st.b	[a13+]1, d11
	j	.Lcpy_b
.Lcinit_done:
	ret
	.size	tricore_cinit, *-tricore_cinit

;****************************************************************************
; Name: tricore_csainit
;****************************************************************************

tricore_csainit:	.type	func
1:
	add	d5, d1, #TC_CONTEXT_SIZE
	jge.u	d5, d2, 2n
	addr2csa	d6, d5
	mov.a	a2, d1
	st.w	[a2], d6
	mov	d1, d5
	j	1p
2:
	ji	a11
	.size	tricore_csainit, *-tricore_csainit

;****************************************************************************
; Name: tricore_boot_cpu
;
; Description:
;   Bring up this cpu core registers and chain the csas.
;
;****************************************************************************

tricore_boot_cpu:	.type	func
	mov.aa	a14, a11			; save return address (jli inside)

	mov.d	d0, a13
	mtcr	#CPU_ISP, d0			; isp = istack

	mov32d	d0, __TRAPTAB_CPU
	mtcr	#CPU_BTV, d0			; btv = traptab_cpu

	mov32d	d0, __INTTAB_CPU
	mtcr	#CPU_BIV, d0			; biv = inttab_cpu

	mov	d0, #TRICORE_PSW_IS
	mtcr	#CPU_PSW, d0			; psw
	isync

#ifdef CONFIG_ARCH_BIST
	; bist_k1: bist only the ram needed to boot (dspr and friends),
	; borrow the pspr scratch area since the test destroys ram.

	mov32d	d0, PSPR_USTACK
	mov.a	a10, d0				; sp = pspr scratch

	mov32d	d1, PSPR_CSA			; d1 = csa base
	mov32d	d2, PSPR_CSA + PSPR_SIZE	; d2 = csa end

	addr2csa	d3, d1
	mtcr	#CPU_FCX, d3			; fcx

	addi	d0, d2, #-FCD_RESERVE_SIZE
	addr2csa	d3, d0
	mtcr	#CPU_LCX, d3			; lcx

	mov32a	a2, tricore_csainit
	jli	a2				; csa list init

	call	tricore_bist_k1
#endif

	mov.aa	a10, a12			; sp = ustack

	mov32d	d1, __CSA			; d1 = csa base
	mov32d	d2, __CSA_END			; d2 = csa end

	addr2csa	d3, d1
	mtcr	#CPU_FCX, d3			; fcx

	addi	d0, d2, #-FCD_RESERVE_SIZE
	addr2csa	d3, d0
	mtcr	#CPU_LCX, d3			; lcx

	mov32a	a2, tricore_csainit
	jli	a2				; csa list init

#ifdef CONFIG_ARCH_BIST
	; bist_k2: bist the remaining ram

	call	tricore_bist_k2
#endif
	ji	a14
	.size	tricore_boot_cpu, *-tricore_boot_cpu

;****************************************************************************
; Name: __start_cpu
;
; Description:
;   Common entry of all cores
;
;****************************************************************************

__start_cpu:	.type	func

	mov32a	a12, __ustack_table
	mov32a	a13, __istack_table

	; index the per-core tables by coreid

#if defined(CONFIG_SMP)
	mfcr	d8, #CPU_CORE_ID
	extr.u	d8, d8, #0, #3
#ifdef CONFIG_ARCH_CHIP_TC3XX
	min.u	d8, d8, #5
#endif
	addsc.a	a12, a12, d8, #2
	addsc.a	a13, a13, d8, #2
#endif
	ld.a	a12, [a12]
	ld.a	a13, [a13]

	; virtualization environment exit(call and rfh must be used in pairs)

#ifndef CONFIG_ARCH_CHIP_TC3XX
	mov.aa	a10, a12			; sp = ustack
	mov.d	d9, a12
	addi	d9, d9, #-CONFIG_IDLETHREAD_STACKSIZE
	addr2csa	d10, d9
	mtcr	#CPU_FCX, d10			; fcx
	isync
	call	tricore_virtualization_exit
#endif

	; Bring up this cpu core registers and chain the csas

	mov32a	a2, tricore_boot_cpu
	jli	a2

	; Icache and Dcache enable

#ifdef CONFIG_ARCH_ICACHE
	call	up_invalidate_icache_all
	call	up_enable_icache
#endif
#ifdef CONFIG_ARCH_DCACHE
	call	up_invalidate_dcache_all
	call	up_enable_dcache
#endif

#if defined(CONFIG_SMP)
	jnz	d8, 1n
#endif

	; Clear bss section and copy data section

	call	tricore_cinit

	call	tricore_boot_primary
	j	2n

#if defined(CONFIG_SMP)
1:
	call	tricore_boot_secondary
#endif
2:
	j	2p				; should never get here
	.size	__start_cpu, *-__start_cpu

;****************************************************************************
; Public Functions
;****************************************************************************

;****************************************************************************
; Name: __start
;****************************************************************************

	.sdecl	'.text.start',code
	.sect	'.text.start'
	.global	__start
__start:	.type	func
	mov32a	a2, __start_cpu
	ji	a2
	.size	__start, *-__start

	.end
