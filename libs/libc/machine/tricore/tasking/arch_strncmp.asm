;****************************************************************************
; libs/libc/machine/tricore/tasking/arch_strncmp.asm
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
; Tasking (astc) port of the GNU arch_strncmp.S.  Same instruction sequence;
; only the assembler syntax differs.
;
; int strncmp(const char *s1, const char *s2, size_t n)
;   s1 -> a4   s2 -> a5   n -> d4   return -> d2
;
; Word-at-a-time compare bounded by n, with NUL early-out.  All word reads
; are ld.w (4-byte aligned), safe for memory-mapped flash, never past a page,
; the NUL, or the n budget.
;
;****************************************************************************

	.sdecl	'.text.strncmp',code
	.sect	'.text.strncmp'
	.global	strncmp

strncmp:	.type	func
	jeq	d4, #0, .Lret0		; n == 0 -> equal

	mov	d15, #8
	jlt.u	d4, d15, .Lbyte		; n < 8 -> byte compare

	; Same 8-byte congruence?  (s1 ^ s2) & 7 == 0
	mov.d	d0, a4
	mov.d	d1, a5
	xor	d0, d0, d1
	and	d0, d0, #7
	jne	d0, #0, .Lbyte		; different alignment -> byte

	; Byte prologue to reach an 8-byte boundary (n >= 8, so safe).
	mov.d	d0, a4
	and	d0, d0, #7
	jeq	d0, #0, .Lword_setup
	rsub	d0, d0, #8		; prologue bytes = 1..7
.Lpre:
	ld.bu	d15, [a4+]
	ld.bu	d1, [a5+]
	jne	d15, d1, .Lretbyte	; differ -> return d15 - d1
	add	d4, #-1			; one byte consumed
	jeq	d15, #0, .Lret0		; NUL -> equal
	add	d0, #-1
	jne	d0, #0, .Lpre

.Lword_setup:
.Lword:
	; 8 bytes/iter: ld.d s1 -> d0:d1, ld.d s2 -> d2:d3 (8-aligned).
	mov	d15, #8
	jlt.u	d4, d15, .Lbyte		; < 8 budget -> byte tail
	ld.d	e0, [a4+]8		; d0:d1 = 8 bytes from s1
	ld.d	e2, [a5+]8		; d2:d3 = 8 bytes from s2
	jne	d0, d2, .Lworddiff	; low word differs
	jne	d1, d3, .Lworddiff	; high word differs
	add	d4, #-8			; 8 bytes consumed
	mov	d15, #0			; zero register for eq.b NUL test
	eq.b	d6, d0, d15		; NUL bytes in low word
	eq.b	d7, d1, d15		; NUL bytes in high word
	or	d6, d6, d7		; any NUL in these 8 equal bytes?
	jeq	d6, #0, .Lword		; no NUL -> keep scanning
	j	.Lret0			; equal dword has NUL -> equal

.Lworddiff:
	; Dwords differ within 8 bytes; back up and resolve byte-wise.
	lea	a4, [a4]-8
	lea	a5, [a5]-8
	; fall through to byte compare

.Lbyte:
	jeq	d4, #0, .Lret0		; budget exhausted -> equal
	ld.bu	d15, [a4+]
	ld.bu	d1, [a5+]
	jne	d15, d1, .Lretbyte
	add	d4, #-1
	jeq	d15, #0, .Lret0		; NUL (both equal) -> equal
	jne	d4, #0, .Lbyte
	; fall through: budget exhausted -> equal

.Lret0:
	mov	d2, #0
	ret

.Lretbyte:
	sub	d2, d15, d1		; return (unsigned char) diff
	ret
.Lstrncmp_end:

	.size	strncmp, .Lstrncmp_end-strncmp
	.end
