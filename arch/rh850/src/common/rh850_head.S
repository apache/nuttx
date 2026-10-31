/****************************************************************************
 * arch/rh850/src/common/rh850_head.S
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "rh850_internal.h"
#include "rh850_sysreg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
#  define RH850_PSW_INIT    (RH850_PSW_ID | RH850_PSW_EBV | RH850_PSW_CU0)
#else
#  define RH850_PSW_INIT    (RH850_PSW_ID | RH850_PSW_EBV)
#endif

/* One vector entry: 16 bytes */

	.macro	vector target
	.balign	16
	jr	\target
	.endm

/****************************************************************************
 * Vector Table
 ****************************************************************************/

/* RBASE = EBASE = 0: reset and the exception vectors at 16-byte steps, then
 * the 16 EIINT priority vectors (direct vector method).
 */

	.section .vectors, "ax"
	.globl	_rh850_vectors
_rh850_vectors:
	vector	__start			/* 000H RESET */
	vector	_rh850_exc_fe		/* 010H SYSERR */
	vector	_rh850_exc_fe		/* 020H reserved */
	vector	_rh850_exc_fe		/* 030H FETRAP */
	vector	_rh850_trap0		/* 040H TRAP0 */
	vector	_rh850_exc_ei		/* 050H TRAP1 */
	vector	_rh850_exc_fe		/* 060H RIE */
	vector	_rh850_exc_ei		/* 070H FPE, FPI */
	vector	_rh850_exc_fe		/* 080H UCPOP */
	vector	_rh850_exc_fe		/* 090H MIP, MDP */
	vector	_rh850_exc_fe		/* 0A0H PIE */
	vector	_rh850_exc_fe		/* 0B0H reserved */
	vector	_rh850_exc_fe		/* 0C0H MAE */
	vector	_rh850_exc_fe		/* 0D0H reserved */
	vector	_rh850_exc_fe		/* 0E0H FENMI */
	vector	_rh850_exc_fe		/* 0F0H FEINT */

	.rept	16
	vector	_rh850_eiint		/* 100H-1F0H EIINT */
	.endr

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Reset entry: registers that are undefined after reset are written before
 * use, then .data and .bss, then C.
 */

	.text
	.globl	__start
	.type	__start, @function
__start:
	.irp	n, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, \
		20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
	mov	r0, r\n
	.endr

	ldsr	r0, RH850_SR_EIPC
	ldsr	r0, RH850_SR_FEPC
	ldsr	r0, RH850_SR_CTPC
	ldsr	r0, RH850_SR_EIWR
	ldsr	r0, RH850_SR_FEWR
	ldsr	r0, RH850_SR_INTBP
	ldsr	r0, RH850_SR_MEA
	ldsr	r0, RH850_SR_MEI
	ldsr	r0, RH850_SR_RBIP

	mov	hilo(_rh850_vectors), r10
	ldsr	r10, RH850_SR_EBASE

	/* FPST and FPCC are views of FPSR and need PSW.CU0 */

	mov	hilo(RH850_PSW_ID | RH850_PSW_EBV | RH850_PSW_CU0), r10
	ldsr	r10, RH850_SR_PSW
	ldsr	r0, RH850_SR_FPST
	ldsr	r0, RH850_SR_FPCC
	mov	hilo(RH850_FPSR_FS), r10
	ldsr	r10, RH850_SR_FPSR
	ldsr	r0, RH850_SR_FPEPC
	mov	hilo(RH850_PSW_INIT), r10
	ldsr	r10, RH850_SR_PSW

	ldsr	r0, RH850_SR_MCA
	ldsr	r0, RH850_SR_MCS
	ldsr	r0, RH850_SR_MCR
	ldsr	r0, RH850_SR_MCI
	mov	r0, r10
	mov	RH850_MPU_ENTRIES, r11
1:
	ldsr	r10, RH850_SR_MPIDX
	ldsr	r0, RH850_SR_MPLA
	ldsr	r0, RH850_SR_MPUA
	ldsr	r0, RH850_SR_MPAT
	add	1, r10
	cmp	r11, r10
	bl	1b
	ldsr	r0, RH850_SR_MPIDX
	.irp	n, 24, 25, 26, 27, 28, 29, 30, 31
	ldsr	r0, \n, 5
	.endr

	ldsr	r0, RH850_SR_ICTAGL
	ldsr	r0, RH850_SR_ICTAGH
	ldsr	r0, RH850_SR_ICDATL
	ldsr	r0, RH850_SR_ICDATH
	ldsr	r0, RH850_SR_ICERR

	mov	hilo(_g_idle_topstack), sp

		mov	hilo(_eronly), r10
	mov	hilo(_sdata), r11
	mov	hilo(_edata), r12
2:
	cmp	r12, r11
	bnl	3f
	ld.w	0[r10], r13
	st.w	r13, 0[r11]
	add	4, r10
	add	4, r11
	br	2b
3:

		mov	hilo(_sbss), r11
	mov	hilo(_ebss), r12
4:
	cmp	r12, r11
	bnl	5f
	st.w	r0, 0[r11]
	add	4, r11
	br	4b
5:

	jarl	_rh850_start, lp
	di
6:
	halt
	br	6b
	.size	__start, . - __start
