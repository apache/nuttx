/****************************************************************************
 * arch/avr/src/avr32/up_syscall6.S
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

/*   Based on Bran's kernel development tutorials. Rewritten for JamesM's
 *   kernel development tutorials.
 */

	.file	"up_syscall6.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

/****************************************************************************
 * .text
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: sys_call6
 *
 * C Prototype:
 *   uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
 *                       uintptr_t parm2, uintptr_t parm3,
 *                       uintptr_t parm4, uintptr_t parm5,
 *                       uintptr_t parm6);
 *
 ****************************************************************************/

	.global	sys_call6
	.type	sys_call6, @function
	.align	2

sys_call6:
	stm		--sp, r3, r5, r6, lr
	sub		lr, sp, -16
	mov		r8, r12
	ldm		lr, r3, r5, r9-r12
	scall
	ldm		sp++, r3, r5, r6, pc

	.size	sys_call6, . - sys_call6
