/****************************************************************************
 * arch/x86/src/i486/up_syscall6.S
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

/* Based on Bran's kernel development tutorials. Rewritten for JamesM's
 * kernel development tutorials.
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
	.type	sys_call6, %function

sys_call6:
	pushl	%ebp			/* Save ebx, esi, edi, and ebp */
	pushl	%edi
	pushl	%esi
	pushl	%ebx

	movl	44(%esp),%ebp	/* Save parm6 in ebp */
	movl	40(%esp),%edi	/* Save parm5 in edi */
	movl	36(%esp),%esi	/* Save parm4 in esi */
	movl	32(%esp),%edx	/* Save parm3 in edx */
	movl	28(%esp),%ecx	/* Save parm2 in ecx */
	movl	24(%esp),%ebx	/* Save parm1 in ebx */
	movl	20(%esp),%eax	/* Save syscall number in eax */
	int		$0x80			/* Execute the trap */
							/* Return value is in %eax */
	popl	%ebx			/* Restore ebx, esi, edi, and ebp */
	popl	%esi
	popl	%edi
	popl	%ebp
	ret						/* And return with result in %eax */

	.size	sys_call6,.-sys_call6
	.end
