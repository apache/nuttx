;****************************************************************************
; libs/libc/machine/tricore/tasking/arch_setjmp.asm
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
; Tasking (astc) port of the GNU arch_setjmp.S.  Same instruction sequence;
; only the assembler syntax differs.  Machine code matches the GNU version.
;
;****************************************************************************
; Name: setjmp
;
; Description:
;   int setjmp(jmp_buf env)
;
;   TriCore EABI:
;     env -> a4   return value -> d2
;
;   jmp_buf layout (struct setjmp_buf_s, arch/tricore/include/setjmp.h):
;     0: pcxi   4: psw    8: sp    12: a11
;    16: d8    20: d9    24: d10   28: d11
;    32: a12   36: a13   40: a14   44: a15
;    48: d12   52: d13   56: d14   60: d15
;
;   On entry a11 holds the return address of the caller (TriCore call ABI).
;   Save it into env[12] first, then repoint a11 at .Lsaveregs and execute
;   `ret` which jumps through a11.  The fall-through at .Lsaveregs stores
;   the remaining callee-saved registers and the PCXI/PSW control registers,
;   loads the saved return address into a2, sets d2 = 0, and jumps back
;   to the caller.  longjmp() returns through the same path with a nonzero
;   d2.
;
;****************************************************************************

	.sdecl	'.text.setjmp',code
	.sect	'.text.setjmp'
	.global	setjmp

setjmp:	.type	func
	st.a	[a4]12, a11		; save return address
	movh.a	a11, #@HIS(.Lsaveregs)
	lea	a11, [a11]@LOS(.Lsaveregs)
	ret				; jump to .Lsaveregs via a11
.Lsaveregs:
	st.a	[a4]8, sp
	st.a	[a4]32, a12
	st.a	[a4]36, a13
	st.a	[a4]40, a14
	st.a	[a4]44, a15
	st.w	[a4]16, d8
	st.w	[a4]20, d9
	st.w	[a4]24, d10
	st.w	[a4]28, d11
	st.w	[a4]48, d12
	st.w	[a4]52, d13
	st.w	[a4]56, d14
	st.w	[a4]60, d15
	mfcr	d2, #0xfe00
	st.w	[a4]0, d2		; save PCXI
	mfcr	d2, #0xfe04
	st.w	[a4]4, d2		; save PSW
	ld.a	a2, [a4]12		; a2 = return address
	mov	d2, #0			; setjmp returns 0
	ji	a2			; back to caller
.Lsetjmp_end:

	.size	setjmp, .Lsetjmp_end-setjmp

;****************************************************************************
; Name: longjmp
;
; Description:
;   void longjmp(jmp_buf env, int val) noreturn
;
;   TriCore EABI:
;     env -> a4   val -> d4
;
;   Unwind the CSA chain (releasing lower/upper contexts one at a time)
;   until the active PCXI matches the one saved by setjmp.  At that point
;   the register banks are at the same depth as at setjmp time, so
;   restoring the callee-saved registers, PSW and return address proceeds.
;   The result passed back equals max(val, 1) per POSIX.
;
;   In Supervisor mode, the saved PSW of each outgoing upper context is
;   rewritten to Supervisor/PRS0 before `ret` so that the kernel unwind loop
;   does not drop into user mode.  User-mode longjmp keeps the original
;   release path.
;   Once the loop exits, Supervisor restores the PSW saved at setjmp time
;   before jumping back to the setjmp call site.
;
;   PSW layout (TC1.6.2 / TC1.8):
;     bit 15      = PRS[2]
;     bit 14      = S    (alternate master tag selector, preserve)
;     bit 13:12   = PRS[1:0]
;     bit 11:10   = IO   (0b10 = Supervisor)
;
;****************************************************************************

	.sdecl	'.text.longjmp',code
	.sect	'.text.longjmp'
	.global	longjmp

longjmp:	.type	func
	ld.w	d5, [a4]0		; d5 = saved PCXI
.Lloop:
	mfcr	d2, #0xfe00		; d2 = current PCXI
	jne	d2, d5, .Lrelease	; depth mismatch -> unwind one

	; Matching depth: restore registers and return.

	max.u	d2, d4, #1		; d2 = max(val, 1)
	ld.a	a2, [a4]12		; a2 = saved return address
	ld.a	sp, [a4]8
	ld.a	a12, [a4]32
	ld.a	a13, [a4]36
	ld.a	a14, [a4]40
	ld.a	a15, [a4]44
	ld.w	d8, [a4]16
	ld.w	d9, [a4]20
	ld.w	d10, [a4]24
	ld.w	d11, [a4]28
	ld.w	d12, [a4]48
	ld.w	d13, [a4]52
	ld.w	d14, [a4]56
	ld.w	d15, [a4]60
	mfcr	d3, #0xfe04
	jz.t	d3, #11, .Lreturn	; only Supervisor may write PSW
	ld.w	d3, [a4]4		; saved PSW
	mtcr	#0xfe04, d3
	isync
.Lreturn:
	ji	a2			; return to setjmp call site

.Lrelease:
	jnz.t	d2, #20, .Lupper	; PCXI.UL = 1 -> release upper

	; Release one lower context, preserving env/val across rslcx.

	mov.aa	a15, a4
	mov	d15, d4
	mov	d14, d5
	rslcx
	mov	d4, d15
	mov	d5, d14
	mov.aa	a4, a15
	j	.Lloop

.Lupper:
	; In Supervisor mode, rewrite the PSW of the outgoing upper context so
	; that ret restores Supervisor/PRS0, not the discarded frame PSW.

	mfcr	d3, #0xfe04
	jz.t	d3, #11, .Lreleaseupper
	mov	d0, #0
	insert	d0, d0, d2, #6, #16
	extr.u	d1, d2, #16, #4
	insert	d0, d0, d1, #28, #4
	mov.a	a2, d0			; a2 = current upper CSA
	ld.w	d3, [a2]4
	mov	d0, #2
	insert	d3, d3, d0, #10, #4	; PSW.IO = 2, PSW.PRS[1:0] = 0
	mov	d0, #0
	insert	d3, d3, d0, #15, #1	; PSW.PRS[2] = 0
	st.w	[a2]4, d3
	dsync

.Lreleaseupper:
	; Point a11 back at .Lloop so ret re-enters the depth check after
	; releasing the upper context.

	movh.a	a11, #@HIS(.Lloop)
	lea	a11, [a11]@LOS(.Lloop)
	ret
.Llongjmp_end:

	.size	longjmp, .Llongjmp_end-longjmp
	.end
