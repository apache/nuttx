/****************************************************************************
 * libs/libc/machine/tricore/gnu/arch_setjmp.S
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: setjmp
 *
 * Description:
 *   int setjmp(jmp_buf env)
 *
 *   TriCore EABI:
 *     env -> %a4   return value -> %d2
 *
 *   jmp_buf layout (struct setjmp_buf_s, see arch/tricore/include/setjmp.h):
 *     0: pcxi   4: psw    8: sp    12: a11
 *    16: d8    20: d9    24: d10   28: d11
 *    32: a12   36: a13   40: a14   44: a15
 *    48: d12   52: d13   56: d14   60: d15
 *
 *   On entry %a11 holds the caller's return address (TriCore call ABI).
 *   Save it into env[12] first, then repoint %a11 at .Lsaveregs and execute
 *   `ret' which jumps through %a11.  The fall-through at .Lsaveregs stores
 *   the remaining callee-saved registers and the PCXI/PSW control registers,
 *   loads the saved return address into %a2, sets %d2 = 0, and jumps back
 *   to the caller.  longjmp() returns through the same path with a nonzero
 *   %d2.
 *
 ****************************************************************************/

	.text
	.global	setjmp
	.type	setjmp, %function

setjmp:
	st.a	[%a4]12, %a11		/* save return address             */
	movh.a	%a11, hi:.Lsaveregs
	lea	%a11, [%a11] lo:.Lsaveregs
	ret				/* jump to .Lsaveregs via %a11     */
.Lsaveregs:
	st.a	[%a4]8, %sp
	st.a	[%a4]32, %a12
	st.a	[%a4]36, %a13
	st.a	[%a4]40, %a14
	st.a	[%a4]44, %a15
	st.w	[%a4]16, %d8
	st.w	[%a4]20, %d9
	st.w	[%a4]24, %d10
	st.w	[%a4]28, %d11
	st.w	[%a4]48, %d12
	st.w	[%a4]52, %d13
	st.w	[%a4]56, %d14
	st.w	[%a4]60, %d15
	mfcr	%d2, 0xfe00
	st.w	[%a4]0, %d2		/* save PCXI                       */
	mfcr	%d2, 0xfe04
	st.w	[%a4]4, %d2		/* save PSW                        */
	ld.a	%a2, [%a4]12		/* %a2 = return address            */
	mov	%d2, 0			/* setjmp returns 0                */
	ji	%a2			/* back to caller                  */
	.size	setjmp, . - setjmp

/****************************************************************************
 * Name: longjmp
 *
 * Description:
 *   void longjmp(jmp_buf env, int val) noreturn
 *
 *   TriCore EABI:
 *     env -> %a4   val -> %d4
 *
 *   Unwind the CSA chain (releasing lower/upper contexts one at a time)
 *   until the active PCXI matches the one saved by setjmp.  At that point
 *   the register banks are at the same depth as at setjmp time, so
 *   restoring the callee-saved registers, PSW and return address proceeds.
 *   The result passed back equals max(val, 1) per POSIX.
 *   The unwind loop forces PSW.IO = Supervisor and PSW.PRS[2] = 0 on every
 *   iteration so that the `ret'-released upper contexts do not drop the CPU
 *   into user mode while env is still being accessed.  Once the loop exits,
 *   the original PSW saved at setjmp time is restored before the indirect
 *   jump back to the setjmp call site.
 *
 *   PSW layout (TC1.6.2 / TC1.8):
 *     bit 15      = PRS[2]
 *     bit 14      = S    (alternate master tag selector, preserve)
 *     bit 13:12   = PRS[1:0]
 *     bit 11:10   = IO   (0b10 = Supervisor)
 *
 ****************************************************************************/

	.global	longjmp
	.type	longjmp, %function

longjmp:
	ld.w	%d5, [%a4]0		/* %d5 = saved PCXI                */
.Lloop:
	mfcr	%d3, 0xfe04		/* %d3 = current PSW               */
	mov	%d2, 2
	insert	%d3, %d3, %d2, 10, 4	/* PSW.IO = 2 (supervisor)         */
	mov	%d2, 0
	insert	%d3, %d3, %d2, 15, 1	/* clear PSW.PRS[2]                */
	mtcr	0xfe04, %d3
	isync
	mfcr	%d2, 0xfe00		/* %d2 = current PCXI              */
	jne	%d2, %d5, .Lrelease	/* depth mismatch -> unwind one    */

	/* Matching depth: restore registers and return. */

	max.u	%d2, %d4, 1		/* %d2 = max(val, 1)               */
	ld.a	%a2, [%a4]12		/* %a2 = saved return address      */
	ld.a	%sp, [%a4]8
	ld.a	%a12, [%a4]32
	ld.a	%a13, [%a4]36
	ld.a	%a14, [%a4]40
	ld.a	%a15, [%a4]44
	ld.w	%d8, [%a4]16
	ld.w	%d9, [%a4]20
	ld.w	%d10, [%a4]24
	ld.w	%d11, [%a4]28
	ld.w	%d12, [%a4]48
	ld.w	%d13, [%a4]52
	ld.w	%d14, [%a4]56
	ld.w	%d15, [%a4]60
	ld.w	%d3, [%a4]4		/* saved PSW                       */
	mtcr	0xfe04, %d3
	isync
	ji	%a2			/* return to setjmp call site      */

.Lrelease:
	jnz.t	%d2, 20, .Lupper	/* PCXI.UL = 1 -> release upper    */

	/* Release one lower context, preserving env/val across rslcx. */

	mov.aa	%a15, %a4
	mov	%d15, %d4
	mov	%d14, %d5
	rslcx
	mov	%d4, %d15
	mov	%d5, %d14
	mov.aa	%a4, %a15
	j	.Lloop

.Lupper:
	/* Release one upper context via `ret' (restores the upper context
	 * from CSA and jumps through %a11).  Point %a11 back at .Lloop so
	 * we re-enter the depth check after the release.
	 */

	movh.a	%a11, hi:.Lloop
	lea	%a11, [%a11] lo:.Lloop
	ret
	.size	longjmp, . - longjmp
