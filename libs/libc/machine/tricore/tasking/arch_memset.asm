;****************************************************************************
; libs/libc/machine/tricore/tasking/arch_memset.asm
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
; Tasking (astc) port of the GNU arch_memset.S.  Same instruction sequence;
; only the assembler syntax differs.  Machine code matches the GNU version.
;
; 8-byte alignment is mandatory for DLMU targets (0xB0/0x90): the DLMU has
; 64-bit ECC granularity, so a quad st.dd starting at a 4-mod-8 offset would
; partially write two 64-bit ECC words as sub-doubleword read-modify-write,
; corrupting shared memory.  Aligning to 8 makes every quad store write two
; whole 64-bit ECC words.
;
; void *memset(void *s, int c, size_t n)
;   s -> a4   c -> d4   n -> d5   return (s) -> a2
;
;****************************************************************************

	.sdecl	'.text.memset',code
	.sect	'.text.memset'
	.global	memset

memset:	.type	func
	mov.aa	a2, a4			; return value = original s
	jeq	d5, #0, .Ldone		; n == 0 -> nothing to do (far)
	mov.aa	a15, a4			; a15 = working dst pointer

	; Build a 16-byte fill pattern in quad Q0 (d0:d1:d2:d3)
	and	d0, d4, #255
	sh	d1, d0, #8
	or	d0, d0, d1
	sh	d1, d0, #16
	or	d0, d0, d1		; d0 = 4-byte fill word
	mov	d1, d0
	mov	d2, d0
	mov	d3, d0			; Q0 = 16-byte fill

	mov	d15, #16
	jlt.u	d5, d15, .Lbyte_setup	; n < 16 -> byte fill (far)

	; Prologue: fill (8 - (dst & 7)) bytes to reach 8-byte boundary
	mov.d	d15, a15
	and	d15, d15, #7
	jeq	d15, #0, .Laligned	; already aligned (far)
	rsub	d15, d15, #8
	sub	d5, d15
	mov.a	a3, d15
	add.a	a3, #-1
.Lpre:
	st.b	[a15+], d4
	loop	a3, .Lpre

.Laligned:
	; Main loop: 32 bytes/iter (two quad stores unrolled)
	sh	d15, d5, #-5
	jz	d15, .Lquad1
	mov.a	a3, d15
	add.a	a3, #-1
.Lqloop:
	st.dd	[a15+]16, e0/e2
	st.dd	[a15+]16, e0/e2
	loop	a3, .Lqloop

.Lquad1:
	; Leftover single 16-byte block (n & 16), straight-line
	jz.t	d5, #4, .Ltail
	st.dd	[a15+]16, e0/e2

.Ltail:
	and	d5, d5, #15
	jz	d5, .Ldone

.Lbyte_setup:
	mov.a	a3, d5
	add.a	a3, #-1
.Lbyte:
	st.b	[a15+], d4
	loop	a3, .Lbyte

.Ldone:
	ret
.Lmemset_end:

	.size	memset, .Lmemset_end-memset
	.end
