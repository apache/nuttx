/****************************************************************************
 * arch/arm/src/armv7-a/arm_fullcontextrestore.S
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

#include "arm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file	"arm_fullcontextrestore.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_fullcontextrestore
 *
 * Description:
 *   Restore the current thread context.  Full prototype is:
 *
 *   void arm_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	arm_fullcontextrestore
	.type	arm_fullcontextrestore, function
arm_fullcontextrestore:

	/* Perform the System call with R0=1 and R1=regs */

	mov		r1, r0				/* R1: regs */
	mov		r0, #SYS_restore_context	/* R0: restore context */
	svc		#SYS_syscall			/* Force synchronous SVCall (or Hard Fault) */

	/* This call should not return */

	bx		lr				/* Unnecessary ... will not return */
	.size	arm_fullcontextrestore, .-arm_fullcontextrestore
	.end
