/****************************************************************************
 * arch/xtensa/src/common/xtensa_macros.S
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

.file "xtensa_macros.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_abi.h>
#include <arch/xtensa/xtensa_specregs.h>

/****************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the "top" the interrupt stack.
 *   Single CPU case.
 *   Must be provided by MCU-specific logic in the SMP case.
 *
 ****************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 15
	.macro	setintstack tmp1 tmp2
	movi		a1, g_intstacktop
	.endm
#endif

/****************************************************************************
 * Macro: ps_setup
 *
 * Description:
 *   Set up PS for C, enable interrupts above this level and clear EXCM.
 *
 * Entry Conditions:
 *   level - interrupt level
 *   tmp   - scratch register
 *
 * Side Effects:
 *   PS and scratch register modified
 *
 * Assumptions:
 *   - PS.EXCM = 1, C calling disabled
 *
 ****************************************************************************/

	.macro	ps_setup	level tmp

#if 0 /* Nested interrupts no yet supported */
#  ifdef __XTENSA_CALL0_ABI__
	/* Disable interrupts at level and below */

	movi	\tmp, PS_INTLEVEL(\level) | PS_UM
#  else
	movi	\tmp, PS_INTLEVEL(\level) | PS_UM | PS_WOE
#  endif
#else
#  ifdef __XTENSA_CALL0_ABI__
	/* Disable all low- and medium-priority interrupts.  Nested are not yet
	 * supported.
	 */

	movi	\tmp, PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM
#  else
	movi	\tmp, PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_WOE
#  endif
#endif

	wsr		\tmp, PS
	rsync

	.endm

