/************************************************************************************
 * arch/arm/src/armv7-m/iar/arm_switchcontext.S
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

	MODULE arm_switchcontext
	SECTION .text:CODE:NOROOT(2)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

	PUBLIC arm_switchcontext

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

	THUMB

arm_switchcontext:

	/* Perform the System call with R0=1, R1=saveregs, R2=restoreregs */

	mov		r2, r1					/* R2: restoreregs */
	mov		r1, r0					/* R1: saveregs */
	mov		r0, #SYS_switch_context	/* R0: context switch */
	svc		0						/* Force synchronous SVCall (or Hard Fault) */

	/* We will get here only after the rerturn from the context switch */

	bx		lr

	END
