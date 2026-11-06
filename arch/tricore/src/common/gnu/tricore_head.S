/****************************************************************************
 * arch/tricore/src/common/gnu/tricore_head.S
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

#include <arch/csfr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRICORE_PSW_RESET  0x980   /* Supervisor, call depth counting on */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_head
 *
 * Description:
 *   Set the small data base registers and the PSW and jump to
 *   tricore_start, which links the CSAs before it calls anything.
 *
 ****************************************************************************/

	.text
	.type	tricore_head, %function
tricore_head:
	movh.a	%a1, hi:__A1_MEM
	lea	%a1, [%a1]lo:__A1_MEM
	movh.a	%a0, hi:__A0_MEM
	lea	%a0, [%a0]lo:__A0_MEM
	movh.a	%a8, hi:__A8_MEM
	lea	%a8, [%a8]lo:__A8_MEM
	movh.a	%a9, hi:__A9_MEM
	lea	%a9, [%a9]lo:__A9_MEM

	mov	%d0, TRICORE_PSW_RESET
	mtcr	TRICORE_CPU_PSW, %d0
	isync

	movh.a	%a15, hi:tricore_start
	lea	%a15, [%a15]lo:tricore_start
	ji	%a15

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   Reset entry of CPU0.  It only sets the stack and jumps to
 *   tricore_head, as the linker script keeps .start to 0x20 bytes
 *   ahead of the interface constants.
 *
 *   tasking/tricore_head.asm is the same code in the TASKING syntax.
 *
 ****************************************************************************/

	.section .start, "ax", %progbits
	.global	__start
	.type	__start, %function
__start:
	movh.a	%sp, hi:__USTACK0
	lea	%sp, [%sp]lo:__USTACK0
	dsync
	movh.a	%a15, hi:tricore_head
	lea	%a15, [%a15]lo:tricore_head
	ji	%a15
