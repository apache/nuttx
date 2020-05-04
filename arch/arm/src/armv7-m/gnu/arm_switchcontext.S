/************************************************************************************
 * arch/arm/src/armv7-m/gnu/arm_switchcontext.S
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
#include <arch/irq.h>

#include "nvic.h"
#include "svcall.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

	.syntax	unified
	.thumb
	.file	"arm_switchcontext.S"

/************************************************************************************
 * Macros
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: arm_switchcontext
 *
 * Description:
 *   Save the current thread context and restore the specified context.
 *   Full prototype is:
 *
 *   void arm_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

	.thumb_func
	.globl	arm_switchcontext
	.type	arm_switchcontext, function
arm_switchcontext:

	/* Perform the System call with R0=1, R1=saveregs, R2=restoreregs */

	mov		r2, r1					/* R2: restoreregs */
	mov		r1, r0					/* R1: saveregs */
	mov		r0, #SYS_switch_context	/* R0: context switch */
	svc		0						/* Force synchronous SVCall (or Hard Fault) */

	/* We will get here only after the rerturn from the context switch */

	bx		lr
	.size	arm_switchcontext, .-arm_switchcontext
	.end
