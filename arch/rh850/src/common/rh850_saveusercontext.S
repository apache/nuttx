/****************************************************************************
 * arch/rh850/src/common/rh850_saveusercontext.S
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

#include "rh850_macros.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* int up_saveusercontext(void *saveregs): frame of the caller, EIPC = the
 * return address, returns 0.
 */

	.text
	.globl	_up_saveusercontext
	.type	_up_saveusercontext, @function
_up_saveusercontext:
	st.w	r1, (4 * REG_R1)[r6]
	st.w	r2, (4 * REG_R2)[r6]
	.irp	n, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, \
		20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
	st.w	r\n, (4 * (REG_R4 + \n - 4))[r6]
	.endr
	st.w	lp, (4 * REG_EIPC)[r6]
	st.w	sp, (4 * REG_SP)[r6]
	rh850_stsr	RH850_SR_PSW, r10
	st.w	r10, (4 * REG_EIPSW)[r6]
#ifdef CONFIG_ARCH_FPU
	rh850_stsr	RH850_SR_FPSR, r10
	st.w	r10, (4 * REG_FPSR)[r6]
	rh850_stsr	RH850_SR_FPEPC, r10
	st.w	r10, (4 * REG_FPEPC)[r6]
#else
	st.w	r0, (4 * REG_FPSR)[r6]
	st.w	r0, (4 * REG_FPEPC)[r6]
#endif
	mov	0, r10
	jmp	[lp]
	.size	_up_saveusercontext, . - _up_saveusercontext
