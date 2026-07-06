;****************************************************************************
; libs/libc/machine/tricore/tasking/arch_memcmp.asm
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
; Tasking (astc) port of the GNU arch_memcmp.S.  Same instruction sequence;
; only the assembler syntax differs.  Machine code matches the GNU version.
;
; int memcmp(const void *s1, const void *s2, size_t n)
;   s1 -> a4   s2 -> a5   n -> d4   return -> d2
;
;****************************************************************************

	.sdecl	'.text.memcmp',code
	.sect	'.text.memcmp'
	.global	memcmp

memcmp:	.type	func
	jeq	d4, #0, .Lret0		; n == 0 -> equal (far)

	mov	d15, #8
	jlt.u	d4, d15, .Lbyte		; n < 8 -> byte compare

	; Same 8-byte congruence?  (s1 ^ s2) & 7 == 0
	mov.d	d0, a4
	mov.d	d1, a5
	xor	d0, d0, d1
	and	d0, d0, #7
	jne	d0, #0, .Lbyte		; different alignment -> byte compare

	; Byte prologue to reach an 8-byte boundary
	mov.d	d0, a4
	and	d0, d0, #7
	jz	d0, .Lword
	rsub	d0, d0, #8
	mov.a	a3, d0
	add.a	a3, #-1
.Lpre:
	ld.bu	d15, [a4+]
	ld.bu	d1, [a5+]
	jne	d15, d1, .Lretbyte1
	add	d4, #-1
	loop	a3, .Lpre

.Lword:
	; Main loop: 8 bytes/iter via ld.d (64-bit single transfer, 8-aligned;
	; flash aperture rejects 16B burst, so ld.d not ld.dd).
	sh	d15, d4, #-3		; d15 = dword count = n >> 3
	jz	d15, .Lbyte		; < 8 bytes left -> byte compare
	and	d4, d4, #7		; d4 = tail bytes after whole dwords
	mov.a	a3, d15
	add.a	a3, #-1
.Lwloop:
	ld.d	e0, [a4+]8		; d0:d1 = 8 bytes from s1
	ld.d	e2, [a5+]8		; d2:d3 = 8 bytes from s2
	jne	d0, d2, .Lworddiff	; low word differs
	jne	d1, d3, .Lworddiff	; high word differs
	loop	a3, .Lwloop
	j	.Lbyte			; all dwords equal -> compare tail

.Lworddiff:
	; Dwords differ within 8 bytes; back up and resolve byte-wise.
	; First differing byte is in these 8 bytes, so byte loop exits early.
	lea	a4, [a4]-8
	lea	a5, [a5]-8
	mov	d4, #8
	; fall through to byte compare

.Lbyte:
	jeq	d4, #0, .Lret0		; far
	mov.a	a3, d4
	add.a	a3, #-1
.Lbyteloop:
	ld.bu	d15, [a4+]
	ld.bu	d1, [a5+]
	jne	d15, d1, .Lretbyte1
	loop	a3, .Lbyteloop

.Lret0:
	mov	d2, #0
	ret

.Lretbyte1:
	sub	d2, d15, d1
	ret
.Lmemcmp_end:

	.size	memcmp, .Lmemcmp_end-memcmp
	.end
