;****************************************************************************
; libs/libc/machine/tricore/tasking/arch_strcmp.asm
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
; Tasking (astc) port of the GNU arch_strcmp.S.  Same instruction sequence;
; only the assembler syntax differs.  Machine code matches the GNU version.
;
; int strcmp(const char *s1, const char *s2)
;   s1 -> a4   s2 -> a5   return -> d2
;
;****************************************************************************

	.sdecl	'.text.strcmp',code
	.sect	'.text.strcmp'
	.global	strcmp

strcmp:	.type	func
	; Same 8-byte congruence?  (s1 ^ s2) & 7 == 0
	mov.d	d0, a4
	mov.d	d1, a5
	xor	d0, d0, d1
	and	d0, d0, #7
	jne	d0, #0, .Lbyte		; different alignment -> byte compare (far)

	; Byte prologue to bring s1/s2 to an 8-byte boundary
	mov.d	d0, a4
	and	d0, d0, #7
	jeq	d0, #0, .Lword_pre
	rsub	d0, d0, #8		; d0 = 8 - (s1 & 7) = 1..7 bytes
	mov.a	a3, d0
	add.a	a3, #-1
.Lpre:
	ld.bu	d15, [a4+]
	ld.bu	d0, [a5+]
	jne	d15, d0, .Lretbyte
	jeq	d15, #0, .Lret0
	loop	a3, .Lpre

.Lword_pre:
	mov	d0, #0			; zero register for eq.b NUL detect
.Lword:
	; 8 bytes/iter: ld.d s1 -> d4:d5, ld.d s2 -> d6:d7 (8-aligned).
	ld.d	e4, [a4+]8		; d4:d5 = 8 bytes from s1
	ld.d	e6, [a5+]8		; d6:d7 = 8 bytes from s2
	jne	d4, d6, .Lworddiff	; low word differs
	jne	d5, d7, .Lworddiff	; high word differs
	eq.b	d15, d4, d0		; NUL bytes in low word
	eq.b	d1, d5, d0		; NUL bytes in high word
	or	d15, d15, d1		; any NUL in these 8 equal bytes?
	jz	d15, .Lword		; no NUL -> keep scanning
	j	.Lret0			; equal dword has NUL -> equal

.Lworddiff:
	; Dwords differ within 8 bytes; back up and resolve byte-wise
	lea	a4, [a4]-8
	lea	a5, [a5]-8
	; fall through to byte compare

.Lbyte:
	ld.bu	d15, [a4+]
	ld.bu	d0, [a5+]
	jne	d15, d0, .Lretbyte
	jnz	d15, .Lbyte
	mov	d2, #0
	ret

.Lretbyte:
	sub	d2, d15, d0
	ret

.Lret0:
	mov	d2, #0
	ret
.Lstrcmp_end:

	.size	strcmp, .Lstrcmp_end-strcmp
	.end
