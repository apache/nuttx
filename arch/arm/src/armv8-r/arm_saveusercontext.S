/****************************************************************************
 * arch/arm/src/armv8-r/arm_saveusercontext.S
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
	.arm

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

	/* Save r0, r1 */

	str		r0, [r0, #(4*REG_R0)]
	str		r1, [r0, #(4*REG_R1)]

#ifdef CONFIG_ARCH_FPU
	/* Save fpu */

#  ifdef CONFIG_ARM_DPFPU32
	vstmia.64	r0!, {d0-d15}
	vstmia.64	r0!, {d16-d31}
#  else
	vstmia		r0!, {s0-s31}
#  endif
	/* Save fpscr, r13 and r14 */

	vmrs		r1, fpscr
	stmia		r0!, {r1, r13, r14}
#else
	/* Save r13, r14 */

	stmia		r0!, {r13, r14}
#endif

	/* Save r2~r12, and store the return address as PC */

	add		r0, r0, #8
	stmia		r0!, {r2-r12, r14}

	/* Save cpsr */

	mrs		r1, CPSR
	str		r1, [r0]

	mov		r0, #0
	bx		lr

	.size	up_saveusercontext, . - up_saveusercontext
	.end
