/****************************************************************************
 * arch/arm/src/armv7-a/arm_smccc.S
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "arm_smccc.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* The SMC instruction is used to generate a synchronous exception that is
 * handled by Secure Monitor code running in EL3.
 */

	.text
	.syntax	unified

	.global	arm_smccc_smc
	.type	arm_smccc_smc, function

arm_smccc_smc:
	mov		r12, sp
	push		{r4-r7}
	ldm		r12, {r4-r7}
	smc		#0
	pop		{r4-r7}
	ldr		r12, [sp, #(4 * 4)]
	stm		r12, {r0-r3}
	bx 		lr

	.size	arm_smccc_smc, . - arm_smccc_smc

	/* The HVC instruction is used to generate a synchronous exception that is
	 * handled by a hypervisor running in EL2.
	 */

	.global	arm_smccc_hvc
	.type	arm_smccc_hvc, function

arm_smccc_hvc:
	mov		r12, sp
	push		{r4-r7}
	ldm		r12, {r4-r7}
	hvc		#0
	pop		{r4-r7}
	ldr		r12, [sp, #(4 * 4)]
	stm		r12, {r0-r3}
	bx		lr

	.size	arm_smccc_hvc, . - arm_smccc_hvc
	.end
