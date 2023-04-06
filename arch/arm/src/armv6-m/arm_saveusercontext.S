/****************************************************************************
 * arch/arm/src/armv6-m/arm_saveusercontext.S
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
#include <nuttx/irq.h>

	.file	"arm_saveusercontext.S"

	.text
	.syntax	unified
	.thumb

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_saveusercontext
 *
 * Description:
 *   Save the current context.  Full prototype is:
 *
 *   int up_saveusercontext(void *saveregs);
 *
 *   R0 = saveregs = pinter saved array
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	up_saveusercontext
	.globl	up_saveusercontext
	.type	up_saveusercontext, %function

up_saveusercontext:

	/* Save r0~r3 */

	str		r0, [r0, #(4*REG_R0)]
	str		r1, [r0, #(4*REG_R1)]
	str		r2, [r0, #(4*REG_R2)]
	str		r3, [r0, #(4*REG_R3)]

	/* Save r12，r14, pc */

	mov		r1, r12
	mov		r2, r14
	str		r1, [r0, #(4*REG_R12)]
	str		r2, [r0, #(4*REG_R14)]
	str		r2, [r0, #(4*REG_R15)]

	/* Save xpsr */

	mrs		r1, XPSR
	str		r1, [r0, #(4*REG_XPSR)]

	/* Save r13, primask, r4~r7 */

	mov		r2, sp
	mrs		r3, primask
	stmia		r0!, {r2-r7}

	/* Save r8~r10 */

	mov		r1, r8
	mov		r2, r9
	mov		r3, r10
	stmia		r0!, {r1-r3}

	/* Save r11 and EXC_RETURN */

	mov		r1, r11
#ifdef CONFIG_BUILD_PROTECTED
	/* Save EXC_RETURN to 0xffffffff */

	movs		r2, #-1
	stmia		r0!, {r1-r2}
#else
	stmia		r0!, {r1}
#endif

	movs		r0, #0
	bx		lr

	.size	up_saveusercontext, . - up_saveusercontext
	.end
