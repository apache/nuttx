;****************************************************************************
; arch/tricore/src/common/tasking/tricore_vectors.asm
;
; SPDX-License-Identifier: Apache-2.0
;
; Licensed to the Apache Software Foundation (ASF) under one or more
; contributor license agreements.  See the NOTICE file distributed with
; this work for additional information regarding copyright ownership.  The
; ASF licenses this file to you under the Apache License, Version 2.0 (the
; "License"); you may not use this file except in compliance with the
; License.  You may obtain a copy of the License at
;
;   http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
; WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
; License for the specific language governing permissions and limitations
; under the License.
;
;****************************************************************************

;****************************************************************************
; Trap vector table (BTV) in .text.traptab and interrupt vector table (BIV)
; in .text.inttab; the linker script places both sections.
;****************************************************************************

;****************************************************************************
; Included Files
;****************************************************************************

#include <nuttx/config.h>
#include <arch/irq.h>

;****************************************************************************
; Pre-processor Definitions
;****************************************************************************

	.extern	tricore_trapcall
	.extern	tricore_svcall
	.extern	tricore_fcutrap_recover
	.extern	g_fcu_csa_base
	.extern	tricore_doirq

TRAPSLOT	.macro	cls, entry
	svlcx
	mov	d5, #cls
	j	entry
	.align	32
.endm

INTSLOT	.macro	handler
	svlcx
	jl	handler
	rslcx
	rfe
	.align	32
.endm

;****************************************************************************
; Private Functions
;****************************************************************************

;****************************************************************************
; Tricore_trap_entry
;****************************************************************************

	.sdecl	".text.tricore_vectors",code
	.sect	".text.tricore_vectors"
	.align	2

	.global	tricore_trap_entry
tricore_trap_entry:	.type	func
	mov.aa	a4, a11
	mov	d4, d15
	jl	tricore_trapcall
	rslcx
	rfe
	.size	tricore_trap_entry, * - tricore_trap_entry

;****************************************************************************
; Tricore_syscall_entry
;****************************************************************************

tricore_syscall_entry:	.type	func
	jl	tricore_svcall
	rslcx
	rfe
	.size	tricore_syscall_entry, * - tricore_syscall_entry

;****************************************************************************
; Tricore_fcu_entry
;****************************************************************************

tricore_fcu_entry:	.type	func
	movh.a	a4, #@HIS(g_fcu_csa_base)
	ld.a	a4, [a4]@LOS(g_fcu_csa_base)
fcu_nopool:
	jz.a	a4, fcu_nopool
	stucx	[a4]0
	stlcx	[a4]64
	jl	tricore_fcutrap_recover
	movh.a	a4, #@HIS(g_fcu_csa_base)
	ld.a	a4, [a4]@LOS(g_fcu_csa_base)
	ld.a	a11, [a4](REG_UA11 * 4)
	mov	d15, #4
	mov	d5, #3
	j	tricore_trap_entry
	.size	tricore_fcu_entry, * - tricore_fcu_entry

;****************************************************************************
; Public Functions
;****************************************************************************

;****************************************************************************
; Trap vector table
;****************************************************************************

	.sdecl	".text.traptab",code,protect
	.sect	".text.traptab"
	.align	256
	.global	tricore_traptab
tricore_traptab:	.type	func
	TRAPSLOT	0, tricore_trap_entry
	TRAPSLOT	1, tricore_trap_entry
	TRAPSLOT	2, tricore_trap_entry

	jne	d15, #4, ctx_notfcu
	j	tricore_fcu_entry
ctx_notfcu:
	TRAPSLOT	3, tricore_trap_entry

	TRAPSLOT	4, tricore_trap_entry
	TRAPSLOT	5, tricore_trap_entry
	TRAPSLOT	6, tricore_syscall_entry
	TRAPSLOT	7, tricore_trap_entry
	.size	tricore_traptab, * - tricore_traptab

;****************************************************************************
; Interrupt vector table.  One entry serves all priorities; the startup
; code sets the BIV priority bits accordingly.
;****************************************************************************

	.sdecl	".text.inttab",code,protect
	.sect	".text.inttab"
	.align	32
	.global	tricore_inttab
tricore_inttab:	.type	func
	INTSLOT	tricore_doirq
	.size	tricore_inttab, * - tricore_inttab

	.end
