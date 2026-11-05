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
;
; TASKING (astc) version of gnu/tricore_vectors.S, same code and layout; see
; there for the description.  astc does not run the C preprocessor, so the
; CORE_ID offset (TRICORE_CPU_CORE_ID in arch/csfr.h) is written out.  The
; tables are protected, as nothing references them by symbol.
;
;****************************************************************************

	.extern	tricore_trapcall
	.extern	tricore_svcall
	.extern	tricore_doirq
	.extern	tricore_fcu_recover
	.extern	g_tricore_fcu_pool

TRAPSLOT	.macro	cls, entry
	svlcx
	mov	d5, #cls
	j	entry
	.align	32
	.endm

TRAPJUMP	.macro	entry
	j	entry
	.align	32
	.endm

TRAPTAB	.macro	cpu, label
	.sdecl	".text.traptab_cpu\cpu",code,protect
	.sect	".text.traptab_cpu\cpu"
	.align	256
	.global	label
label:
	TRAPSLOT	0, tricore_trap_entry
	TRAPSLOT	1, tricore_trap_entry
	TRAPSLOT	2, tricore_trap_entry
	TRAPJUMP	tricore_ctx_entry
	TRAPSLOT	4, tricore_trap_entry
	TRAPSLOT	5, tricore_trap_entry
	TRAPSLOT	6, tricore_syscall_entry
	TRAPSLOT	7, tricore_trap_entry
	.endm

	TRAPTAB	0, tricore_trap_vectors_cpu0
	TRAPTAB	1, tricore_trap_vectors_cpu1
	TRAPTAB	2, tricore_trap_vectors_cpu2
	TRAPTAB	3, tricore_trap_vectors_cpu3
	TRAPTAB	4, tricore_trap_vectors_cpu4
	TRAPTAB	5, tricore_trap_vectors_cpu5

	.sdecl	".text.intvec",code,protect
	.sect	".text.intvec"
	.align	32
	.global	tricore_int_vector
tricore_int_vector:
	svlcx
	j	tricore_int_entry

; struct tricore_trap_s: taddr at 0, tid at 4, tclass at 5, tcpu at 6

TRAPENTRY	.macro	name, handler
	.global	name
name:
	sub.a	sp, #8
	st.a	[sp]0, a11
	st.b	[sp]4, d15
	st.b	[sp]5, d5
	mfcr	d4, #0xfe1c		; TRICORE_CPU_CORE_ID
	and	d4, d4, #7
	st.b	[sp]6, d4
	mov.aa	a4, sp
	movh.a	a2, #@HIS(handler)
	lea	a2, [a2]@LOS(handler)
	jli	a2
	lea	sp, [sp]8
	rslcx
	rfe
	.endm

	.sdecl	".text.tricore_trap_entry",code
	.sect	".text.tricore_trap_entry"

	TRAPENTRY	tricore_trap_entry, tricore_trapcall
	TRAPENTRY	tricore_syscall_entry, tricore_svcall

tricore_int_entry:
	movh.a	a2, #@HIS(tricore_doirq)
	lea	a2, [a2]@LOS(tricore_doirq)
	jli	a2
	rslcx
	rfe

tricore_ctx_entry:
	jeq	d15, #4, tricore_fcu_entry
	svlcx
	mov	d5, #3
	j	tricore_trap_entry

tricore_fcu_entry:
	movh.a	a4, #@HIS(g_tricore_fcu_pool)
	ld.a	a4, [a4]@LOS(g_tricore_fcu_pool)
fcu_nopool:
	jz.a	a4, fcu_nopool
	stucx	[a4]0
	stlcx	[a4]64
	j	tricore_fcu_recover

	.end
