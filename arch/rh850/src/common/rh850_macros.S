/****************************************************************************
 * arch/rh850/src/common/rh850_macros.S
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

#ifndef __ARCH_RH850_SRC_COMMON_RH850_MACROS_S
#define __ARCH_RH850_SRC_COMMON_RH850_MACROS_S

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#include "rh850_sysreg.h"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

	.macro	rh850_stsr regid, selid, reg
	stsr	\regid, \reg, \selid
	.endm

/* fe = 1 saves FEPC/FEPSW in the EIPC/EIPSW slots.  exc = 1 sets CU0 so
 * that FPSR can be read after UCPOP, and EP so that EIRET keeps ISPR.
 */

	.macro	rh850_frame_save fe=0, exc=0
	addi	-XCPTCONTEXT_SIZE, sp, sp
	st.w	r1, (4 * REG_R1)[sp]
	st.w	r2, (4 * REG_R2)[sp]
	.irp	n, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, \
		20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
	st.w	r\n, (4 * (REG_R4 + \n - 4))[sp]
	.endr
	.if	\fe
	rh850_stsr	RH850_SR_FEPC, r10
	rh850_stsr	RH850_SR_FEPSW, r11
	.else
	rh850_stsr	RH850_SR_EIPC, r10
	rh850_stsr	RH850_SR_EIPSW, r11
	.endif
	st.w	r10, (4 * REG_EIPC)[sp]
	st.w	r11, (4 * REG_EIPSW)[sp]
	addi	XCPTCONTEXT_SIZE, sp, r10
	st.w	r10, (4 * REG_SP)[sp]
	.if	\exc
	rh850_stsr	RH850_SR_PSW, r10
	mov	hilo(RH850_PSW_CU0 | RH850_PSW_EP), r11
	or	r11, r10
	ldsr	r10, RH850_SR_PSW
	.endif
#ifdef CONFIG_ARCH_FPU
	rh850_stsr	RH850_SR_FPSR, r10
	st.w	r10, (4 * REG_FPSR)[sp]
	rh850_stsr	RH850_SR_FPEPC, r10
	st.w	r10, (4 * REG_FPEPC)[sp]
#else
	st.w	r0, (4 * REG_FPSR)[sp]
	st.w	r0, (4 * REG_FPEPC)[sp]
#endif
	.endm

/* Restore the frame at sp and return with EIRET.  sp is loaded last from
 * REG_SP, the frame base is no longer needed after that.  A pending FPI
 * (FPEC.FPIVD = 1) keeps its FPEPC and FPSR cause bits.
 */

	.macro	rh850_frame_restore
#ifdef CONFIG_ARCH_FPU
	ld.w	(4 * REG_FPSR)[sp], r10
	rh850_stsr	RH850_SR_FPEC, r11
	andi	RH850_FPEC_FPIVD, r11, r11
	bz	.Lfpu_frame\@
	rh850_stsr	RH850_SR_FPSR, r11
	andi	RH850_FPSR_XC_MASK, r11, r11
	andi	RH850_FPSR_XC_MASK, r10, r12
	xor	r12, r10
	or	r11, r10
	ldsr	r10, RH850_SR_FPSR
	br	.Lfpu_done\@
.Lfpu_frame\@:
	ldsr	r10, RH850_SR_FPSR
	ld.w	(4 * REG_FPEPC)[sp], r10
	ldsr	r10, RH850_SR_FPEPC
.Lfpu_done\@:
#endif
	ld.w	(4 * REG_EIPC)[sp], r10
	ldsr	r10, RH850_SR_EIPC
	ld.w	(4 * REG_EIPSW)[sp], r10
	ldsr	r10, RH850_SR_EIPSW
	ld.w	(4 * REG_R1)[sp], r1
	ld.w	(4 * REG_R2)[sp], r2
	.irp	n, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, \
		20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
	ld.w	(4 * (REG_R4 + \n - 4))[sp], r\n
	.endr
	ld.w	(4 * REG_SP)[sp], sp
	eiret
	.endm

#endif /* __ARCH_RH850_SRC_COMMON_RH850_MACROS_S */
