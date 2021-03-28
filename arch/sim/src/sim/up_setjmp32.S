/**************************************************************************
 * arch/sim/src/sim/up_setjmp32.S
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include "up_internal.h"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

#ifdef __CYGWIN__
# define SYMBOL(s) _##s
#elif defined(__ELF__)
# define SYMBOL(s) s
#else
# define SYMBOL(s) _##s
#endif

/**************************************************************************
 * Public Functions
 **************************************************************************/

	.text
	.globl	SYMBOL(up_setjmp)
#ifdef __ELF__
	.type	SYMBOL(up_setjmp), @function
#endif
SYMBOL(up_setjmp):

	/* %ebx, %esi, %edi, and %ebp must be preserved.
	 * save %ebx, $esi, and %edi now... */

	movl	4(%esp), %eax
	movl	%ebx, (JB_EBX)(%eax)
	movl	%esi, (JB_ESI)(%eax)
	movl	%edi, (JB_EDI)(%eax)

	/* Save the value of SP as will be after we return */

	leal	4(%esp), %ecx
	movl	%ecx, (JB_SP)(%eax)

	/* Save the return PC */

	movl	0(%esp), %ecx
	movl	%ecx, (JB_PC)(%eax)

	/* Save the framepointer */

	movl	%ebp, (JB_EBP)(%eax)

	/* And return 0 */

	xorl	%eax, %eax
	ret
#ifdef __ELF__
	.size	SYMBOL(up_setjmp), . - SYMBOL(up_setjmp)
#endif
	.globl	SYMBOL(up_longjmp)
#ifdef __ELF__
	.type	SYMBOL(up_longjmp), @function
#endif
SYMBOL(up_longjmp):
	movl	4(%esp), %ecx      /* jmpbuf in %ecx.  */
	movl	8(%esp), %eax      /* Second argument is return value.  */

	/* Save the return address now.  */

	movl	(JB_PC)(%ecx), %edx

	/* Restore registers.  */

	movl	(JB_EBX)(%ecx), %ebx
	movl	(JB_ESI)(%ecx), %esi
	movl	(JB_EDI)(%ecx), %edi
	movl	(JB_EBP)(%ecx), %ebp
	movl	(JB_SP)(%ecx), %esp

	/* Jump to saved PC.  */

	jmp		*%edx
#ifdef __ELF__
	.size SYMBOL(up_longjmp), . - SYMBOL(up_longjmp)
#endif
