;****************************************************************************
; libs/libc/machine/tricore/tasking/arch_memmove.asm
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
; Tasking (astc) port of the GNU arch_memmove.S.  Same instruction sequence;
; only the assembler syntax differs.  Machine code matches the GNU version.
;
; 8-byte alignment is mandatory for DLMU targets (0xB0/0x90): the DLMU has
; 64-bit ECC granularity, so a quad st.dd starting at a 4-mod-8 offset would
; partially write two 64-bit ECC words as sub-doubleword read-modify-write,
; corrupting shared memory.  Buffers only 4-byte congruent fall back to byte
; copy.
;
; void *memmove(void *dest, const void *src, size_t n)
;   dest -> a4   src -> a5   n -> d4   return (dest) -> a2
;
;****************************************************************************

	.sdecl	'.text.memmove',code
	.sect	'.text.memmove'
	.global	memmove

memmove: .type	func
	mov.aa	a2, a4			; return value = original dest
	jeq	d4, #0, .Ldone		; n == 0 -> nothing to do (far)
	mov.aa	a15, a4			; a15 = working dst pointer

	; Direction: if src >= dst, forward copy is safe
	ge.a	d15, a5, a4
	jz	d15, .Lbackward

	; ---- Forward path (same as memcpy) ----
	mov	d15, #16
	jlt.u	d4, d15, .Lfbyte	; n < 16 -> byte copy

	mov.d	d0, a5
	mov.d	d1, a15
	xor	d0, d0, d1
	and	d0, d0, #7		; 8-byte congruence (DLMU ECC)
	jne	d0, #0, .Lfshiftmerge	; different congruence -> shift-merge

	mov.d	d15, a15
	and	d15, d15, #7
	jeq	d15, #0, .Lfaligned
	rsub	d15, d15, #8
	sub	d4, d15
	mov.a	a3, d15
	add.a	a3, #-1
.Lfpre:
	ld.bu	d0, [a5+]
	st.b	[a15+], d0
	loop	a3, .Lfpre

.Lfaligned:
	sh	d15, d4, #-5
	jz	d15, .Lfquad1
	mov.a	a3, d15
	add.a	a3, #-1
.Lfqloop:
	ld.d	e0, [a5+]8		; 2x 8-byte (64-bit) block loads:
	ld.d	e2, [a5+]8		; flash aperture rejects 16B burst
	st.dd	[a15+]16, e0/e2
	ld.d	e0, [a5+]8		; 2x 8-byte (64-bit) block loads:
	ld.d	e2, [a5+]8		; flash aperture rejects 16B burst
	st.dd	[a15+]16, e0/e2
	loop	a3, .Lfqloop

.Lfquad1:
	jz.t	d4, #4, .Lftail
	ld.d	e0, [a5+]8		; 2x 8-byte (64-bit) block loads:
	ld.d	e2, [a5+]8		; flash aperture rejects 16B burst
	st.dd	[a15+]16, e0/e2

.Lftail:
	and	d4, d4, #15
	jeq	d4, #0, .Ldone		; far

.Lfbyte:
	mov.a	a3, d4
	add.a	a3, #-1
.Lfbyteloop:
	ld.bu	d0, [a5+]
	st.b	[a15+], d0
	loop	a3, .Lfbyteloop
	ret

	; Forward word-granular copy for src/dst not sharing 8-byte
	; congruence (previously slow byte copy).  Same as memcpy shift-merge:
	; align dst to 4, then plain word copy (src aligned) or shift-merge
	; funnel (src misaligned).  All ld.w (4-byte single transfer), safe
	; for memory-mapped flash.
.Lfshiftmerge:
	mov.d	d15, a15
	and	d15, d15, #3
	jeq	d15, #0, .Lfsm_dstok
	rsub	d15, d15, #4		; prologue bytes to 4-align dst
	sub	d4, d15
	mov.a	a3, d15
	add.a	a3, #-1
.Lfsm_pre:
	ld.bu	d0, [a5+]
	st.b	[a15+], d0
	loop	a3, .Lfsm_pre
.Lfsm_dstok:
	mov.d	d0, a5
	and	d0, d0, #3
	jeq	d0, #0, .Lfsm_aligned
	sh	d8, d0, #3		; shift_bits = misalign*8
	rsub	d9, d8, #32		; 32 - shift_bits
	rsub	d7, d8, #0		; -shift_bits (logical right)
	mov.d	d0, a5
	andn	d0, d0, #3		; src_al = src & ~3
	mov.a	a6, d0
	ld.w	d5, [a6+]4		; preload first aligned word
	sh	d15, d4, #-2		; word count = n >> 2
	jeq	d15, #0, .Lfsm_tail
	mov.a	a3, d15
	add.a	a3, #-1
.Lfsm_mloop:
	ld.w	d6, [a6+]4
	sh	d0, d5, d7		; preload >> shift_bits
	sh	d1, d6, d9		; next    << (32 - shift_bits)
	or	d0, d0, d1
	st.w	[a15+]4, d0
	mov	d5, d6
	add.a	a5, #4			; advance logical src for tail
	loop	a3, .Lfsm_mloop
	j	.Lfsm_tail
.Lfsm_aligned:
	sh	d15, d4, #-2		; word count = n >> 2
	jeq	d15, #0, .Lfsm_tail
	mov.a	a3, d15
	add.a	a3, #-1
.Lfsm_aloop:
	ld.w	d0, [a5+]4
	st.w	[a15+]4, d0
	loop	a3, .Lfsm_aloop
.Lfsm_tail:
	and	d4, d4, #3		; remaining tail bytes
	jeq	d4, #0, .Ldone
	j	.Lfbyte			; reuse forward byte tail (a5->a15)

	; ---- Backward path (high-to-low, overlap-safe) ----
.Lbackward:
	addsc.a	a15, a4, d4, #0		; a15 = dst + n
	addsc.a	a6, a5, d4, #0		; a6  = src + n

	mov	d15, #16
	jlt.u	d4, d15, .Lbbyte	; n < 16 -> byte copy

	mov.d	d0, a5
	mov.d	d1, a4
	xor	d0, d0, d1
	and	d0, d0, #7		; 8-byte congruence (DLMU ECC)
	jne	d0, #0, .Lbbyte		; different congruence -> byte copy

	mov.d	d15, a15
	and	d15, d15, #7
	jeq	d15, #0, .Lbaligned
	sub	d4, d15
	mov.a	a3, d15
	add.a	a3, #-1
.Lbpre:
	ld.bu	d0, [+a6]-1
	st.b	[+a15]-1, d0
	loop	a3, .Lbpre

.Lbaligned:
	sh	d15, d4, #-5
	jz	d15, .Lbquad1
	mov.a	a3, d15
	add.a	a3, #-1
.Lbqloop:
	ld.d	e2, [+a6]-8		; 2x 8-byte block loads (high then
	ld.d	e0, [+a6]-8		; low) to match ld.dd byte layout
	st.dd	[+a15]-16, e0/e2
	ld.d	e2, [+a6]-8		; 2x 8-byte block loads (high then
	ld.d	e0, [+a6]-8		; low) to match ld.dd byte layout
	st.dd	[+a15]-16, e0/e2
	loop	a3, .Lbqloop

.Lbquad1:
	jz.t	d4, #4, .Lbtail
	ld.d	e2, [+a6]-8		; 2x 8-byte block loads (high then
	ld.d	e0, [+a6]-8		; low) to match ld.dd byte layout
	st.dd	[+a15]-16, e0/e2

.Lbtail:
	and	d4, d4, #15
	jeq	d4, #0, .Ldone		; far

.Lbbyte:
	mov.a	a3, d4
	add.a	a3, #-1
.Lbbyteloop:
	ld.bu	d0, [+a6]-1
	st.b	[+a15]-1, d0
	loop	a3, .Lbbyteloop

.Ldone:
	ret
.Lmemmove_end:

	.size	memmove, .Lmemmove_end-memmove
	.end
