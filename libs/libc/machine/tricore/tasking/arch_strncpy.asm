;****************************************************************************
; libs/libc/machine/tricore/tasking/arch_strncpy.asm
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
; Tasking (astc) port of the GNU arch_strncpy.S.  Same instruction sequence;
; only the assembler syntax differs.
;
; char *strncpy(char *dest, const char *src, size_t n)
;   dest -> a4   src -> a5   n -> d4   return (dest) -> a2
;
; Copy phase (word-accelerated, bounded by n, stop at NUL) + pad phase
; (zero-fill remaining n bytes if the source NUL was reached first).  All
; source reads are ld.w (4-byte aligned) and never exceed the n budget, safe
; for memory-mapped flash.
;
;****************************************************************************

	.sdecl	'.text.strncpy',code
	.sect	'.text.strncpy'
	.global	strncpy

strncpy:	.type	func
	mov.aa	a2, a4			; return value = original dest
	mov.aa	a15, a4			; a15 = working dst pointer

	; DETECTNULL constants: d2 = 0x01010101, d3 = 0x80808080
	movh	d2, #0x0101
	addi	d2, d2, #0x0101
	sh	d3, d2, #7

	; Byte prologue: copy until dst 8-aligned (bounded by n, stop at NUL).
.Lpre:
	jeq	d4, #0, .Ldone		; n exhausted -> done (no pad)
	mov.d	d0, a15
	and	d0, d0, #7
	jeq	d0, #0, .Ldst_ok	; dst 8-aligned -> bulk path
	ld.bu	d0, [a5+]
	st.b	[a15+], d0
	add	d4, #-1
	jeq	d0, #0, .Lpad		; NUL copied -> zero-fill rest
	j	.Lpre

.Ldst_ok:
	; dst 8-aligned.  Is src also 8-aligned (same congruence)?
	mov.d	d0, a5
	and	d0, d0, #7
	jne	d0, #0, .Lsrc_lt8

	; 8-byte aligned copy: ld.d + two 32-bit DETECTNULL, st.d.
.Lword8:
	mov	d1, #8
	jlt.u	d4, d1, .Lbyte		; < 8 budget -> byte tail
	ld.d	e0, [a5]		; d0:d1 = 8 bytes src (8-aligned)
	sub	d6, d0, d2		; low: zero-byte test
	andn	d6, d6, d0
	and	d6, d6, d3
	sub	d7, d1, d2		; high: zero-byte test
	andn	d7, d7, d1
	and	d7, d7, d3
	or	d6, d6, d7		; NUL anywhere in these 8?
	jne	d6, #0, .Lbyte		; NUL -> finish byte-wise (a5 unchanged)
	st.d	[a15+]8, e0		; store whole 8 bytes (dst 8-aligned)
	lea	a5, [a5]8
	add	d4, #-8
	j	.Lword8

.Lsrc_lt8:
	; src not 8-aligned.  4-aligned -> word copy; else shift-merge.
	and	d0, d0, #3		; d0 = src & 3 (d0 held src & 7)
	jne	d0, #0, .Lshiftmerge

	; Same 4-byte congruence: aligned word copy with NUL detection.
.Lword:
	mov	d1, #4
	jlt.u	d4, d1, .Lbyte		; < 4 budget -> byte tail
	ld.w	d1, [a5]		; w = *src (4-byte aligned)
	sub	d0, d1, d2		; zero-byte test
	andn	d0, d0, d1
	and	d0, d0, d3
	jne	d0, #0, .Lbyte		; NUL in word -> finish byte-wise
	st.w	[a15+]4, d1		; store whole word
	add.a	a5, #4
	add	d4, #-4
	j	.Lword

.Lbyte:
	; Byte tail: copy while budget remains, stop and pad at NUL.
	jeq	d4, #0, .Ldone
	ld.bu	d0, [a5+]
	st.b	[a15+], d0
	add	d4, #-1
	jeq	d0, #0, .Lpad		; NUL copied -> zero-fill rest
	j	.Lbyte

.Lpad:
	; Zero-fill the remaining d4 bytes.  a15 points past the NUL.
	jeq	d4, #0, .Ldone
	mov.d	d0, a15
	and	d0, d0, #3
	jeq	d0, #0, .Lpad_word	; aligned -> word fill
	mov	d0, #0
	st.b	[a15+], d0
	add	d4, #-1
	j	.Lpad

.Lpad_word:
	mov	d0, #0			; zero source for word/byte fill
.Lpad_wloop:
	mov	d1, #4
	jlt.u	d4, d1, .Lpad_byte	; < 4 left -> byte fill
	st.w	[a15+]4, d0
	add	d4, #-4
	j	.Lpad_wloop

.Lpad_byte:
	jeq	d4, #0, .Ldone
	st.b	[a15+], d0
	add	d4, #-1
	j	.Lpad_byte

.Ldone:
	ret

.Lshiftmerge:
	; Misaligned src.  sb = (src & 3) * 8, sc = 32 - sb.
	sh	d8, d0, #3		; d8 = shift_bits (misalign*8)
	rsub	d9, d8, #32		; d9 = 32 - shift_bits
	rsub	d7, d8, #0		; d7 = -shift_bits (logical >>)
	mov.d	d0, a5
	andn	d0, d0, #3		; src_al = src & ~3
	mov.a	a6, d0
	ld.w	d5, [a6+]4		; preload first aligned word

.Lsm_loop:
	mov	d1, #4
	jlt.u	d4, d1, .Lbyte		; < 4 budget -> byte tail (from a5)
	ld.w	d6, [a6+]4		; next aligned word
	sh	d0, d5, d7		; preload >> shift_bits
	sh	d1, d6, d9		; next    << (32 - shift_bits)
	or	d1, d0, d1		; d1 = funnel-merged logical word
	sub	d0, d1, d2		; zero-byte test
	andn	d0, d0, d1
	and	d0, d0, d3
	jne	d0, #0, .Lbyte		; NUL -> finish byte-wise (a5)
	st.w	[a15+]4, d1		; store merged word
	mov	d5, d6			; preload = next
	add.a	a5, #4			; advance logical src for tail
	add	d4, #-4
	j	.Lsm_loop
.Lstrncpy_end:

	.size	strncpy, .Lstrncpy_end-strncpy
	.end
