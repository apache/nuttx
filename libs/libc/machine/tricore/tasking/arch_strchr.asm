;****************************************************************************
; libs/libc/machine/tricore/tasking/arch_strchr.asm
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
; Tasking (astc) port of the GNU arch_strchr.S.  Same instruction sequence;
; only the assembler syntax differs.
;
; char *strchr(const char *s, int c)
;   s -> a4   c -> d4   return -> a2
;
; Word-at-a-time dual detection: DETECTNULL on the word (find NUL) and on
; (word ^ cccc) (find c).  All word reads are ld.w, 4-byte aligned, safe for
; memory-mapped flash.
;
;****************************************************************************

	.sdecl	'.text.strchr',code
	.sect	'.text.strchr'
	.global	strchr

strchr:	.type	func
	mov.aa	a15, a4			; a15 = working pointer

	; Target byte kept two ways: d4 = c & 0xff, d6 = broadcast.
	and	d4, d4, #0xff
	sh	d6, d4, #8
	or	d6, d6, d4
	sh	d0, d6, #16
	or	d6, d6, d0		; d6 = 0xcccccccc

.Lpre:
	mov.d	d1, a15
	and	d1, d1, #7
	jeq	d1, #0, .Lword_setup	; 8-aligned -> dword loop
	ld.bu	d1, [a15]
	jeq	d1, d4, .Lfound		; byte == c
	jeq	d1, #0, .Lnul		; terminator, c not found
	add.a	a15, #1
	j	.Lpre

.Lword_setup:
	; DETECTNULL constants: d2 = 0x01010101, d3 = 0x80808080
	movh	d2, #0x0101
	addi	d2, d2, #0x0101
	sh	d3, d2, #7

.Lword_loop:
	; 8 bytes/iter via ld.d (8-aligned).  Dual SWAR (NUL + c) on both the
	; low (d0) and high (d1) words, OR-combined into d5.
	ld.d	e0, [a15]		; d0:d1 = 8 bytes (8-aligned)
	sub	d5, d0, d2		; low: NUL test
	andn	d5, d5, d0
	and	d5, d5, d3		; d5 = hz_nul(low)
	xor	d7, d0, d6		; x = low ^ cccc
	sub	d15, d7, d2		; low: c test
	andn	d15, d15, d7
	and	d15, d15, d3		; d15 = hz_c(low)
	or	d5, d5, d15		; d5 = hit(low)
	sub	d15, d1, d2		; high: NUL test
	andn	d15, d15, d1
	and	d15, d15, d3
	or	d5, d5, d15		; accumulate hz_nul(high)
	xor	d7, d1, d6		; x = high ^ cccc
	sub	d15, d7, d2		; high: c test
	andn	d15, d15, d7
	and	d15, d15, d3
	or	d5, d5, d15		; d5 = any NUL or c in these 8 bytes?
	jne	d5, #0, .Lbyte		; hit -> resolve byte-wise
	lea	a15, [a15]8		; a15 += 8 (add.a const4 range is -8..7)
	j	.Lword_loop

.Lbyte:
	ld.bu	d1, [a15]
	jeq	d1, d4, .Lfound		; byte == c
	jeq	d1, #0, .Lnul		; terminator, c not found
	add.a	a15, #1
	j	.Lbyte

.Lfound:
	mov.aa	a2, a15			; return pointer to match
	ret

.Lnul:
	mov.a	a2, #0			; return NULL
	ret
.Lstrchr_end:

	.size	strchr, .Lstrchr_end-strchr
	.end
