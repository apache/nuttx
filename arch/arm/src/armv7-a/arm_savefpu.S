/****************************************************************************
 * arch/arm/src/armv7-a/arm_savefpu.S
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

#ifdef CONFIG_ARCH_FPU

	.file		"arm_savefpu.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl		arm_savefpu

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: arm_savefpu
 *
 * Description:
 *   Given the pointer to a register save area (in R0), save the state of the
 *   floating point registers.
 *
 * C Function Prototype:
 *   void arm_savefpu(uint32_t *regs);
 *
 * Input Parameters:
 *   regs - A pointer to the register save area in which to save the floating point
 *     registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	arm_savefpu
	.type	arm_savefpu, function

arm_savefpu:

	add		r1, r0, #(4*REG_S0)		/* R1=Address of FP register storage */

	/* Store all floating point registers.  Registers are stored in numeric order,
	 * s0, s1, ... in increasing address order.
	 */

#ifdef CONFIG_ARM_HAVE_FPU_D32
	vstmia.64	r1!, {d0-d15}			/* Save the full FP context */
	vstmia.64	r1!, {d16-d31}
#else
	vstmia		r1!, {s0-s31}			/* Save the full FP context */
#endif

	/* Store the floating point control and status register.  At the end of the
	 * vstmia, r1 will point to the FPSCR storage location.
	 */

	vmrs		r2, fpscr			/* Fetch the FPSCR */
	str		r2, [r1], #4			/* Save the floating point control and status register */
	bx		lr

	.size	arm_savefpu, .-arm_savefpu
#endif /* CONFIG_ARCH_FPU */
	.end
