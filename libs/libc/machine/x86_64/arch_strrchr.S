/****************************************************************************
 * libs/libc/machine/x86_64/arch_strrchr.S
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

/* Page-safe SSE2 strrchr (128-bit; used when AVX2 is not enabled). Returns a
 * pointer to the LAST occurrence of c in s, or NULL. c may be '\0'. First
 * load aligned down to a 16-byte boundary so it never crosses into an
 * unmapped page; a block past the NUL is never loaded.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef STRRCHR
# define STRRCHR	strrchr
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
ENTRY (STRRCHR)
	movd	%esi, %xmm1
	punpcklbw %xmm1, %xmm1
	punpcklwd %xmm1, %xmm1
	pshufd	$0, %xmm1, %xmm1
	pxor	%xmm2, %xmm2
	xor	%r9, %r9		/* last match address, 0 = none */

	mov	%rdi, %rax
	and	$-16, %rax
	mov	%edi, %ecx
	and	$15, %ecx

	movdqa	(%rax), %xmm0
	movdqa	%xmm0, %xmm3
	pcmpeqb	%xmm1, %xmm3
	pcmpeqb	%xmm2, %xmm0
	pmovmskb %xmm3, %r8d
	pmovmskb %xmm0, %r11d
	shr	%cl, %r8d
	shr	%cl, %r11d
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
	pmovmskb %xmm0, %r11d
	mov	%rax, %r10

L(resolve):
	test	%r11d, %r11d
	jnz	L(has_nul)
	test	%r8d, %r8d
	jz	L(loop)
	bsr	%r8d, %edx
	lea	(%r10, %rdx), %r9
	jmp	L(loop)

	.p2align 4
L(has_nul):
	bsf	%r11d, %ecx
	test	%sil, %sil
	jnz	L(not_c0)
	lea	(%r10, %rcx), %r9
	jmp	L(return)
L(not_c0):
	mov	$1, %edx
	shl	%cl, %edx
	sub	$1, %edx
	and	%edx, %r8d
	jz	L(return)
	bsr	%r8d, %edx
	lea	(%r10, %rdx), %r9
L(return):
	mov	%r9, %rax
	ret
END (STRRCHR)
