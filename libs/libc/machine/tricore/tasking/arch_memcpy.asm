;****************************************************************************
; libs/libc/machine/tricore/tasking/arch_memcpy.asm
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
; Tasking (astc) port of the GNU arch_memcpy.S.  Same instruction sequence;
; only the assembler syntax differs (section directives, '#' immediates).
;
; 8-byte alignment is mandatory for DLMU targets (0xB0/0x90): the DLMU has
; 64-bit ECC granularity, so a quad st.dd starting at a 4-mod-8 offset would
; partially write two 64-bit ECC words as sub-doubleword read-modify-write,
; corrupting shared memory.  Aligning to 8 makes every quad write two whole
; 64-bit ECC words.  Buffers only 4-byte congruent fall back to byte copy.
;
; void *memcpy(void *dst, const void *src, size_t n)
;   dst -> a4   src -> a5   n -> d4   return (dst) -> a2
;
;****************************************************************************

	.sdecl	'.text.memcpy',code
	.sect	'.text.memcpy'
	.global	memcpy

memcpy:	.type	func
	mov.aa	a2, a4			; return value = original dst
	jeq	d4, #0, .Ldone		; n == 0 -> nothing to do (32-bit form, far target)
	mov.aa	a15, a4			; a15 = working dst pointer

	mov	d15, #16
	jlt.u	d4, d15, .Lbyte_setup	; n < 16 -> byte copy

	; Require src/dst same 8-byte congruence (DLMU ECC)
	mov.d	d0, a5
	mov.d	d1, a15
	xor	d0, d0, d1
	and	d0, d0, #7
	jne	d0, #0, .Lshiftmerge	; different congruence -> word shift-merge

	; Prologue: copy (8 - (dst & 7)) bytes to reach 8-byte boundary
	mov.d	d15, a15
	and	d15, d15, #7
	jeq	d15, #0, .Laligned	; already aligned (32-bit form, far target)
	rsub	d15, d15, #8
	sub	d4, d15
	mov.a	a3, d15
	add.a	a3, #-1
.Lpre:
	ld.bu	d0, [a5+]
	st.b	[a15+], d0
	loop	a3, .Lpre

.Laligned:
	; Main loop: 32 bytes/iter (two quad copies unrolled) to halve branch overhead
	sh	d15, d4, #-5
	jz	d15, .Lquad1
	mov.a	a3, d15
	add.a	a3, #-1
.Lqloop:
	ld.d	e0, [a5+]8		; 2x 8-byte (64-bit) block loads:
	ld.d	e2, [a5+]8		; flash aperture rejects 16B burst
	st.dd	[a15+]16, e0/e2
	ld.d	e0, [a5+]8		; 2x 8-byte (64-bit) block loads:
	ld.d	e2, [a5+]8		; flash aperture rejects 16B burst
	st.dd	[a15+]16, e0/e2
	loop	a3, .Lqloop

.Lquad1:
	; Leftover single 16-byte block (n & 16), straight-line, no setup
	jz.t	d4, #4, .Ltail
	ld.d	e0, [a5+]8		; 2x 8-byte (64-bit) block loads:
	ld.d	e2, [a5+]8		; flash aperture rejects 16B burst
	st.dd	[a15+]16, e0/e2

.Ltail:
	and	d4, d4, #15
	jz	d4, .Ldone

.Lbyte_setup:
	mov.a	a3, d4
	add.a	a3, #-1
.Lbyte:
	ld.bu	d0, [a5+]
	st.b	[a15+], d0
	loop	a3, .Lbyte

.Ldone:
	ret

	; Word-granular copy for src/dst not sharing 8-byte congruence
	; (previously slow byte copy).  Align dst to 4, then plain word copy
	; (src aligned) or shift-merge funnel (src misaligned).  All ld.w
	; (4-byte single transfer), safe for memory-mapped flash.
.Lshiftmerge:
	mov.d	d15, a15
	and	d15, d15, #3
	jeq	d15, #0, .Lsm_dstok
	rsub	d15, d15, #4		; prologue bytes to 4-align dst
	sub	d4, d15
	mov.a	a3, d15
	add.a	a3, #-1
.Lsm_pre:
	ld.bu	d0, [a5+]
	st.b	[a15+], d0
	loop	a3, .Lsm_pre
.Lsm_dstok:
	mov.d	d0, a5
	and	d0, d0, #3
	jeq	d0, #0, .Lsm_aligned
	sh	d8, d0, #3		; shift_bits = misalign*8
	rsub	d9, d8, #32		; 32 - shift_bits
	rsub	d7, d8, #0		; -shift_bits (logical right)
	mov.d	d0, a5
	andn	d0, d0, #3		; src_al = src & ~3
	mov.a	a6, d0
	ld.w	d5, [a6+]4		; preload first aligned word
	sh	d15, d4, #-2		; word count = n >> 2
	jeq	d15, #0, .Lsm_tail
	mov.a	a3, d15
	add.a	a3, #-1
.Lsm_mloop:
	ld.w	d6, [a6+]4
	sh	d0, d5, d7		; preload >> shift_bits
	sh	d1, d6, d9		; next    << (32 - shift_bits)
	or	d0, d0, d1
	st.w	[a15+]4, d0
	mov	d5, d6
	add.a	a5, #4			; advance logical src for tail
	loop	a3, .Lsm_mloop
	j	.Lsm_tail
.Lsm_aligned:
	sh	d15, d4, #-2		; word count = n >> 2
	jeq	d15, #0, .Lsm_tail
	mov.a	a3, d15
	add.a	a3, #-1
.Lsm_aloop:
	ld.w	d0, [a5+]4
	st.w	[a15+]4, d0
	loop	a3, .Lsm_aloop
.Lsm_tail:
	and	d4, d4, #3		; remaining tail bytes
	jeq	d4, #0, .Ldone
	j	.Lbyte_setup		; reuse byte-copy tail (a5->a15)
.Lmemcpy_end:

	.size	memcpy, .Lmemcpy_end-memcpy
	.end
