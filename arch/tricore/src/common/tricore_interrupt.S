/****************************************************************************
 * arch/tricore/src/common/tricore_interrupt.S
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

#include <arch/arch.h>
	.section .ivtab, "ax", %progbits
	.globl  biv_vector
	.type   biv_vector, %object
	.p2align 5

biv_vector:
	.set N, 255

	.rept N
1:
	svlcx
	j _isr_wrapper

	.space (32 - (.-1b))
	.endr

_isr_wrapper:
	mov	%d14, lo:TRICORE_IR_LASR_ADDR
	addih	%d14, %d14, hi:TRICORE_IR_LASR_ADDR

	mfcr	%d15, TRICORE_CPU_CORE_ID
	extr.u	%d15, %d15, 0, 3
	madd	%d14, %d14, %d15, TRICORE_IR_LASR_TOS_STRIDE

	mov.a	%a2, %d14
	ld.w	%d15, [%a2]
	extr.u	%d15, %d15, 16, TRICORE_IR_LASR_PIPN_WIDTH

	mov	%d4, %d15
        movh.a  %a14, hi:tricore_isr_handler
        lea     %a14, [%a14]lo:tricore_isr_handler
	ji	%a14
