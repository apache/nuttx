/****************************************************************************
 * arch/arm/src/tlsr82/tc32/div_mod.S
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

#define 	UDIV		#0
#define 	SDIV		#1
#define 	UMOD		#2
#define 	SMOD		#3

#define 	MUL2_STEP		8

	.code	16
	.text

	.align	2
	.global	__modsi3
	.code 16
	.thumb_func
	.type	__modsi3, %function
__modsi3:
	tmov	r2, SMOD
	tj	div
	.size	__modsi3, .-__modsi3

	.align	2
	.global	__divsi3
	.code 16
	.thumb_func
	.type	__divsi3, %function
__divsi3:
	tmov	r2, SDIV
	tj	div
	.size	__divsi3, .-__divsi3

	.align	2
	.global	__umodsi3
	.code 16
	.thumb_func
	.type	__umodsi3, %function
__umodsi3:
	tmov	r2, UMOD
	tj	div
	.size	__umodsi3, .-__umodsi3

	.align	2
	.global	__udivsi3
	.code 16
	.thumb_func
	.type	__udivsi3, %function
__udivsi3:
	tmov	r2, UDIV
	tj	div
	.size	__udivsi3, .-__udivsi3

	.align	2
	.global	div
	.code 16
	.thumb_func
	.type	div, %function
div:
	tmrcs	r3
	tpush	{r3, r4}
	tmov	r4, #0x80
	tor	r3, r4
	tmcsr	r3

	tloadr	r3, .L11
	tstorer	r0, [r3]
	tadd	r3, r3, #4
	tstorer	r1, [r3]
	tsub	r3, r3, #8
	tstorerb	r2, [r3]

.L2:
	tloadrb	r0, [r3]
	tcmp	r0, #0
	tjne	.L2
	tcmp	r2, #1
	tjls	.L4
	tadd	r3, r3, #8
	tloadr	r0, [r3]
	tj	.L6
.L4:
	tadd	r3, r3, #4
	tloadr	r0, [r3]

.L6:
	tpop	{r3, r4}
	tmcsr	r3
	tjex	lr

	.align	4
.L11:
	.word(0x800664)
	.word(0x800660)
	.word(0x800668)
	.size	div, .-div

#if 1
	.align	4
	.global	mul32x32_64
	.thumb_func
	.type	mul32x32_64, %function
mul32x32_64:
    tmul	r0, r1
    tloadr	r1, [pc, #4]
    tloadr	r1, [r1, #0]
    tjex	lr
    .word(0x008006fc)
#endif

#if 1
	.align	4
	.global	mz_mul1
	.thumb_func
	.type	mz_mul1, %function
mz_mul1:
	tpush 	{r4, r5, r6, r7}
	tmov	r4, r8
	tpush	{r4}
	tmov	r4, #1
	tmov	r8, r4				//r8 = 1
	tloadr	r6, [pc, #4]     	//r6 REG_ADDR32(0x6fc)
	tmovs	r5, #0				//clear carry
	tj	MZ_MUL1_END
    .word(0x008006fc)
MZ_MUL1_START:
	tmul	r4, r3				// l0 = a0 * b
	tloadr	r7, [r0, #0]		// y0
	tadd	r4, r5				// l0 + c => c0
	tsubc	r5, r5				// c0 - 1
	tadd	r4, r7				// l0 + c + y0 => c1
	tloadr	r7, [r6, #0]		// r7 = h0
	tadd	r5,	r8				// c0 - 1 + 1 = c0 (nc)
	tstorem	r0!, {r4}			// store y0
	taddc	r5, r7				// cn = c0 + h1 + c1
MZ_MUL1_END:
	tloadm	r1!, {r4}			// load *a
	tsub	r2, #1				// r2--
	tcmp	r2, #0
	tjge		MZ_MUL1_START	// carry set
	tstorem	r0!, {r5}

	tpop	{r4}
	tmov	r8, r4
	tpop 	{r4, r5, r6, r7}
    tjex	lr
#endif

	.align	4
	.global	mz_mul2
	.thumb_func
	.type	mz_mul2, %function
mz_mul2:
	tpush 	{r4, r5, r6, r7}
	tmov	r4, r8
	tmov	r5, r9
	tmov	r6, r10

	tmov	r7, r11
	tpush	{r4, r5, r6, r7}
	tmov	r8, r2				//r8 = n, loop number
	tmov	r2,	#1

	tmov	r10, r2				// r10 = 1
	tsub	r2, #(MUL2_STEP + 1)
	tmov	r9, r2				//r9 = -MUL2_STEP
	tmov	r2, #0

	tmov	r2, #0
	tloadr	r6, [pc, #4]     	 //r6 REG_ADDR32(0x6fc)
	tmov	r11,r2				//r11 = 0
	tj	MZ_MUL2_LOOP
	//tj	MZ_MUL2_LOOP2
	.word(0x008006fc)

MZ_MUL2_START:
	//a0
	tmul	r4, r3				// l0 = a0 * b
	tloadr	r7, [r0, #0]		// y0
	tadd	r4, r2				// l0 + c => c0
	tsubc	r2, r2				// c0 - 1
	tadd	r4, r7				// l0 + c + y0 => c1
	tloadr	r7, [r6, #0]		// r7 = h0
	tadd	r2,	r10				// c0 - 1 + 1 = c0 (nc)
	tstorem	r0!, {r4}			// store y0 y1
	taddc	r2, r7				// cn = c0 + h1 + c1

	tmul	r5, r3				// l1 = a1 * b
	tloadr	r7, [r0, #0]		// y1
	tadd	r5, r2				// l1 + cn => c2
	tsubc	r2, r2				// c2 - 1
	tadd	r5, r7				// l1 + c + y1 => c3
	tloadr	r7, [r6, #0]		// r7 = h0
	tadd	r2,	r10				// c0 - 1 + 1 = c0 (nc)
	tstorem	r0!, {r5}			// store y0 y1
	tloadm	r1!, {r4, r5}		// load *a
	taddc	r2, r7				// cn2 = c2 + h1 + c3

	//a0
	tmul	r4, r3				// l0 = a0 * b
	tloadr	r7, [r0, #0]		// y0
	tadd	r4, r2				// l0 + c => c0
	tsubc	r2, r2				// c0 - 1
	tadd	r4, r7				// l0 + c + y0 => c1
	tloadr	r7, [r6, #0]		// r7 = h0
	tadd	r2,	r10				// c0 - 1 + 1 = c0 (nc)
	tstorem	r0!, {r4}			// store y0 y1
	taddc	r2, r7				// cn = c0 + h1 + c1

	tmul	r5, r3				// l1 = a1 * b
	tloadr	r7, [r0, #0]		// y1
	tadd	r5, r2				// l1 + cn => c2
	tsubc	r2, r2				// c2 - 1
	tadd	r5, r7				// l1 + c + y1 => c3
	tloadr	r7, [r6, #0]		// r7 = h0
	tadd	r2,	r10				// c0 - 1 + 1 = c0 (nc)
	tstorem	r0!, {r5}			// store y0 y1
	tloadm	r1!, {r4, r5}		// load *a
	taddc	r2, r7				// cn2 = c2 + h1 + c3

	//a0
	tmul	r4, r3				// l0 = a0 * b
	tloadr	r7, [r0, #0]		// y0
	tadd	r4, r2				// l0 + c => c0
	tsubc	r2, r2				// c0 - 1
	tadd	r4, r7				// l0 + c + y0 => c1
	tloadr	r7, [r6, #0]		// r7 = h0
	tadd	r2,	r10				// c0 - 1 + 1 = c0 (nc)
	tstorem	r0!, {r4}			// store y0 y1
	taddc	r2, r7				// cn = c0 + h1 + c1

	tmul	r5, r3				// l1 = a1 * b
	tloadr	r7, [r0, #0]		// y1
	tadd	r5, r2				// l1 + cn => c2
	tsubc	r2, r2				// c2 - 1
	tadd	r5, r7				// l1 + c + y1 => c3
	tloadr	r7, [r6, #0]		// r7 = h0
	tadd	r2,	r10				// c0 - 1 + 1 = c0 (nc)
	tstorem	r0!, {r5}			// store y0 y1
	tloadm	r1!, {r4, r5}		// load *a
	taddc	r2, r7				// cn2 = c2 + h1 + c3

///// next 2
	tmul	r4, r3				// l0 = a0 * b
	tloadr	r7, [r0, #0]		// y0
	tadd	r4, r2				// l0 + c => c0
	tsubc	r2, r2				// c0 - 1
	tadd	r4, r7				// l0 + c + y0 => c1
	tloadr	r7, [r6, #0]		// r7 = h0
	tadd	r2,	r10				// c0 - 1 + 1 = c0 (nc)
	tstorem	r0!, {r4}			// store y0 y1
	taddc	r2, r7				// cn = c0 + h1 + c1

	tmul	r5, r3				// l1 = a1 * b
	tloadr	r7, [r0, #0]		// y1
	tadd	r5, r2				// l1 + cn => c2
	tsubc	r2, r2				// c2 - 1
	tadd	r5, r7				// l1 + c + y1 => c3
	tloadr	r7, [r6, #0]		// r7 = h0
	tadd	r2,	r10				// c0 - 1 + 1 = c0 (nc)
	tstorem	r0!, {r5}			// store y0 y1
	taddc	r2, r7				// cn2 = c2 + h1 + c3

MZ_MUL2_LOOP:
	tloadm	r1!, {r4, r5}		// load *a
	tadd	r8, r9				// r8 -= MUL2_STEP
	tcmp	r8, r11				// const 0
	tjge		MZ_MUL2_START	// carry set

	tmov	r5,	r8
	tadd	r5, #MUL2_STEP
	tsub	r1, #8
	tj	MZ_MUL2_LOOP2

MZ_MUL2_START2:
	tmul	r4, r3				// l0 = a0 * b
	tloadr	r7, [r0, #0]		// y0
	tadd	r4, r2				// l0 + c => c0
	tsubc	r2, r2				// c0 - 1
	tadd	r4, r7				// l0 + c + y0 => c1
	tloadr	r7, [r6, #0]		// r7 = h0
	tadd	r2,	r10				// c0 - 1 + 1 = c0 (nc)
	tstorem	r0!, {r4}			// store y0
	taddc	r2, r7				// cn = c0 + h1 + c1

MZ_MUL2_LOOP2:
	tloadm	r1!, {r4}		// load *a
	tsub	r5, #1				// r7--
	tcmp	r5, #0
	tjge		MZ_MUL2_START2	// carry set

MZ_MUL2_END:
	//tmov	r2, #13
	tstorem	r0!, {r2}

	tpop 	{r4, r5, r6, r7}
	tmov	r8, r4
	tmov	r9, r5
	tmov	r10, r6
	tmov	r11, r7
	tpop 	{r4, r5, r6, r7}
    tjex	lr
    tnop
