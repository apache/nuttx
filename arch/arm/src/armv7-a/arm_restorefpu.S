/****************************************************************************
 * arch/arm/src/armv7-a/arm_restorefpu.S
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

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	arm_restorefpu
	.file	"arm_restorefpu.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: arm_restorefpu
 *
 * Description:
 *   Given the pointer to a register save area (in R0), restore the state of the
 *   floating point registers.
 *
 * C Function Prototype:
 *   void arm_restorefpu(const uint32_t *regs);
 *
 * Input Parameters:
 *   regs - A pointer to the register save area containing the floating point
 *     registers.
 *
 * Returned Value:
 *   This function does not return anything explicitly.  However, it is called from
 *   interrupt level assembly logic that assumes that r0 is preserved.
 *
 ****************************************************************************/

	.globl	arm_restorefpu
	.type	arm_restorefpu, function

arm_restorefpu:

	add		r1, r0, #(4*REG_S0)		/* R1=Address of FP register storage */

	/* Load all floating point registers.  Registers are loaded in numeric order,
	 * s0, s1, ... in increasing address order.
	 */

#ifdef CONFIG_ARM_HAVE_FPU_D32
	vldmia.64	r1!, {d0-d15}			/* Restore the full FP context */
	vldmia.64	r1!, {d16-d31}
#else
	vldmia		r1!, {s0-s31}			/* Restore the full FP context */
#endif

	/* Load the floating point control and status register.   At the end of the
	 * vstmia, r1 will point to the FPSCR storage location.
	 */

	ldr		r2, [r1], #4			/* Fetch the floating point control and status register */
	vmsr		fpscr, r2			/* Restore the FPSCR */
	bx		lr

	.size	arm_restorefpu, .-arm_restorefpu
#endif /* CONFIG_ARCH_FPU */
	.end
