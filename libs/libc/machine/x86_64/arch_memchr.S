/****************************************************************************
 * libs/libc/machine/x86_64/arch_memchr.S
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

/* Page-safe SSE2 memchr (128-bit; used when AVX2 is not enabled). Bounded
 * scan of exactly n bytes [s, s+n). 16-byte-aligned-down loads never cross
 * into the next page; a match is only honored if its byte index is < n. The
 * loop stops once the aligned pointer passes s+n.
 *
 * void *memchr(const void *s /rdi/, int c /esi/, size_t n /rdx/)
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#include "libc.h"

#ifndef MEMCHR
# define MEMCHR		ARCH_LIBCFUN(memchr)
#endif

#ifndef L
# define L(label)	.L##label
#endif

#ifndef cfi_startproc
# define cfi_startproc			.cfi_startproc
#endif

#ifndef cfi_endproc
# define cfi_endproc			.cfi_endproc
#endif

#ifndef ENTRY
# define ENTRY(name)			\
	.type name,  @function;	\
	.globl name;            \
	.p2align 4;			        \
name:					            \
	cfi_startproc
#endif

#ifndef END
# define END(name)  \
	cfi_endproc;			\
	.size name, .-name
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text
ENTRY (MEMCHR)
	test	%rdx, %rdx
	jz	L(not_found)

	movd	%esi, %xmm1
	punpcklbw %xmm1, %xmm1
	punpcklwd %xmm1, %xmm1
	pshufd	$0, %xmm1, %xmm1

	lea	(%rdi, %rdx), %r11	/* end = s + n (exclusive) */
	mov	%rdi, %rax
	and	$-16, %rax
	mov	%edi, %ecx
	and	$15, %ecx

	movdqa	(%rax), %xmm0
	pcmpeqb	%xmm1, %xmm0
	pmovmskb %xmm0, %r8d
	shr	%cl, %r8d
	mov	%rdi, %r10
	test	%r8d, %r8d
	jnz	L(check_first)
	add	$16, %rax
L(loop):
	cmp	%r11, %rax
	jae	L(not_found)
	movdqa	(%rax), %xmm0
	pcmpeqb	%xmm1, %xmm0
	pmovmskb %xmm0, %r8d
	mov	%rax, %r10
	test	%r8d, %r8d
	jz	L(advance)
	bsf	%r8d, %edx
	lea	(%r10, %rdx), %rax
	cmp	%r11, %rax
	jb	L(ret)
	jmp	L(not_found)
L(advance):
	add	$16, %rax
	jmp	L(loop)

	.p2align 4
L(check_first):
	bsf	%r8d, %edx
	lea	(%r10, %rdx), %rax
	cmp	%r11, %rax
	jb	L(ret)
	jmp	L(not_found)

	.p2align 4
L(ret):
	ret
	.p2align 4
L(not_found):
	xor	%eax, %eax
	ret
END (MEMCHR)
