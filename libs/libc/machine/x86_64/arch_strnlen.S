/****************************************************************************
 * libs/libc/machine/x86_64/arch_strnlen.S
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
 * Pre-processor Definitions
 ****************************************************************************/

#include "libc.h"

#ifndef L
# define L(label)	.L##label
#endif

#ifndef cfi_startproc
# define cfi_startproc	.cfi_startproc
#endif

#ifndef cfi_endproc
# define cfi_endproc	.cfi_endproc
#endif

#ifndef ENTRY
# define ENTRY(name)		\
	.type name, @function;	\
	.globl name;		\
	.p2align 4;		\
name:				\
	cfi_startproc
#endif

#ifndef END
# define END(name)		\
	cfi_endproc;		\
	.size name, .-name
#endif

#ifndef STRNLEN
#  define STRNLEN	ARCH_LIBCFUN(strnlen)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* size_t arch_strnlen(const char *s, size_t maxlen)
 *
 * Returns the length of string s, but at most maxlen.
 *
 * Input:
 *   %rdi - pointer to string s
 *   %rsi - maxlen
 *
 * Output:
 *   %rax - min(strlen(s), maxlen)
 */

	.section .text,"ax",@progbits
ENTRY (STRNLEN)
	xor	%eax, %eax	/* rax = 0, also null byte to search */
	mov	%rsi, %rcx	/* rcx = maxlen (counter for repnz) */
	test	%rcx, %rcx	/* if maxlen == 0 */
	jz	L(ret)		/* return 0 */
	mov	%rdi, %rdx	/* save start pointer */
	repnz	scasb		/* scan for null byte, up to rcx bytes */
	jnz	L(not_found)	/* ZF=0: count exhausted, no null found */

	/* Null found: %rdi points one past the null byte */

	sub	%rdx, %rdi	/* rdi = bytes consumed (including null) */
	lea	-1(%rdi), %rax	/* length = bytes consumed - 1 */
	ret

L(not_found):
	mov	%rsi, %rax	/* return maxlen */
L(ret):
	ret

END (STRNLEN)
