/****************************************************************************
 * libs/libc/machine/x86_64/arch_strchrnul.S
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

/* Page-safe SSE2 strchrnul (128-bit; used when AVX2 is not enabled). Returns
 * a pointer to the terminating NUL if c is not found. First load aligned down
 * to a 16-byte boundary so it never crosses into an unmapped page.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#include "libc.h"

#ifndef STRCHRNUL
# define STRCHRNUL	ARCH_LIBCFUN(strchrnul)
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
ENTRY (STRCHRNUL)
	movd	%esi, %xmm1
	punpcklbw %xmm1, %xmm1
	punpcklwd %xmm1, %xmm1
	pshufd	$0, %xmm1, %xmm1
	pxor	%xmm2, %xmm2

	mov	%rdi, %rax
	and	$-16, %rax
	mov	%edi, %ecx
	and	$15, %ecx

	movdqa	(%rax), %xmm0
	movdqa	%xmm0, %xmm3
	pcmpeqb	%xmm1, %xmm3
	pcmpeqb	%xmm2, %xmm0
	pmovmskb %xmm3, %r8d
	pmovmskb %xmm0, %r9d
	shr	%cl, %r8d
	shr	%cl, %r9d
	mov	%rdi, %r10
	jmp	L(resolve)

	.p2align 4
L(loop):
	add	$16, %rax
	movdqa	(%rax), %xmm0
	movdqa	%xmm0, %xmm3
	pcmpeqb	%xmm1, %xmm3
	pcmpeqb	%xmm2, %xmm0
	pmovmskb %xmm3, %r8d
	pmovmskb %xmm0, %r9d
	mov	%rax, %r10

L(resolve):
	mov	%r8d, %edx
	or	%r9d, %edx
	test	%edx, %edx
	jz	L(loop)

	test	%r8d, %r8d
	jz	L(nul_only)
	bsf	%r8d, %r8d
	test	%r9d, %r9d
	jz	L(c_hit)
	bsf	%r9d, %r9d
	cmp	%r9d, %r8d
	ja	L(nul_pos)
L(c_hit):
	lea	(%r10, %r8), %rax
	ret

	.p2align 4
L(nul_only):
	bsf	%r9d, %r9d
L(nul_pos):
	lea	(%r10, %r9), %rax
	ret
END (STRCHRNUL)
