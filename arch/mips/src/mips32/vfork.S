/************************************************************************************
 * arch/mips/src/mips32/vfork.S
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "mips_vfork.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

	.file	"vfork.S"
	.globl	up_vfork

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: vfork
 *
 * Description:
 *   The vfork() function has the same effect as fork(), except that the behavior is
 *   undefined if the process created by vfork() either modifies any data other than
 *   a variable of type pid_t used to store the return value from vfork(), or returns
 *   from the function in which vfork() was called, or calls any other function before
 *   successfully calling _exit() or one of the exec family of functions.
 *
 *   This thin layer implements vfork by simply calling up_vfork() with the vfork()
 *   context as an argument.  The overall sequence is:
 *
 *   1) User code calls vfork().  vfork() collects context information and
 *      transfers control up up_vfork().
 *   2) up_vfork() and calls nxtask_setup_vfork().
 *   3) nxtask_setup_vfork() allocates and configures the child task's TCB.  This
 *      consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_vfork() then calls nxtask_start_vfork()
 *   6) nxtask_start_vfork() then executes the child thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Upon successful completion, vfork() returns 0 to the child process and returns
 *   the process ID of the child process to the parent process. Otherwise, -1 is
 *   returned to the parent, no child process is created, and errno is set to
 *   indicate the error.
 *
 ************************************************************************************/

	.text
	.align  2
	.globl	vfork
	.type	vfork, function
	.set	nomips16
#ifdef CONFIG_MIPS_MICROMIPS
	.set	micromips
#endif
	.ent	vfork

vfork:
	/* Create a stack frame */

	move	$t0, $sp					/* Save the value of the stack on entry */
	addiu	$sp, $sp, -VFORK_SIZEOF		/* Allocate the structure on the stack */

	/* CPU registers */
	/* Save the saved registers */

	sw		$s0, VFORK_S0_OFFSET($sp)
	sw		$s1, VFORK_S1_OFFSET($sp)
	sw		$s2, VFORK_S2_OFFSET($sp)
	sw		$s3, VFORK_S3_OFFSET($sp)
	sw		$s4, VFORK_S4_OFFSET($sp)
	sw		$s5, VFORK_S5_OFFSET($sp)
	sw		$s6, VFORK_S6_OFFSET($sp)
	sw		$s7, VFORK_S7_OFFSET($sp)

#ifdef CONFIG_MIPS32_FRAMEPOINTER
	sw		$fp, VFORK_FP_OFFSET($sp)
#else
	sw		$s8, VFORK_S8_OFFSET($sp)
#endif

	/* Save the global pointer, stack pointer, and return address */

	sw		$t0, VFORK_SP_OFFSET($sp)
	sw		$ra, VFORK_RA_OFFSET($sp)
#ifdef MIPS32_SAVE_GP
	sw		$gp, VFORK_GP_OFFSET($sp)
#endif

	/* Floating point registers (not yet) */

	/* Then, call up_vfork(), passing it a pointer to the stack structure */

	move	$a0, $sp
	jal		up_vfork
	nop

	/* Release the stack data and return the value returned by up_vfork */

	lw		$ra, VFORK_RA_OFFSET($sp)
	addiu	$sp, $sp, VFORK_SIZEOF
	j		$ra

	.end	vfork
	.size	vfork, .-vfork
