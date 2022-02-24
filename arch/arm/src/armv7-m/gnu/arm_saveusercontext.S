/****************************************************************************
 * arch/arm/src/armv7-m/gnu/arm_saveusercontext.S
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
#include <arch/irq.h>
#include <arch/syscall.h>

#include "nvic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.syntax	unified
	.thumb
	.file	"arm_saveusercontext.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_saveusercontext
 *
 * Description:
 *   Save the current thread context.  Full prototype is:
 *
 *   int  arm_saveusercontext(uint32_t *saveregs);
 *
 * Returned Value:
 *   0: Normal return
 *   1: Context switch return
 *
 ****************************************************************************/

	.text
	.thumb_func
	.globl	arm_saveusercontext
	.type	arm_saveusercontext, function
arm_saveusercontext:

	/* Perform the System call with R0=0 and R1=regs */

	mov		r1, r0			/* R1: regs */
	mov		r0, #SYS_save_context	/* R0: save context (also return value) */
	svc		#SYS_syscall		/* Force synchronous SVCall (or Hard Fault) */

	/* There are two return conditions.  On the first return, R0 (the
	 * return value will be zero.  On the second return we need to
	 * force R0 to be 1.
	 */

	add		r2, r1, #(4*REG_R0)
	mov		r3, #1
	str		r3, [r2, #0]
	bx		lr			/* "normal" return with r0=0 or
						 * context switch with r0=1 */
	.size	arm_saveusercontext, .-arm_saveusercontext
	.end
