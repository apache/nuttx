/****************************************************************************
 * libs/libc/machine/x86_64/arch_strrchr_avx2.S
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

/* Page-safe AVX2 strrchr. Returns a pointer to the LAST occurrence of c in
 * s, or NULL. c may be '\0' (then returns a pointer to the terminating NUL).
 * Scans to the NUL, tracking the most recent match. First vector load is
 * aligned down to a 32-byte boundary so it never crosses into an unmapped
 * page; a block past the NUL is never loaded.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#include "libc.h"

#ifndef STRRCHR
# define STRRCHR	ARCH_LIBCFUN(strrchr)
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

	.section .text.avx2,"ax",@progbits
ENTRY (STRRCHR)
	vmovd	%esi, %xmm1
	vpbroadcastb %xmm1, %ymm1
	vpxor	%xmm2, %xmm2, %xmm2
	xor	%r9, %r9		/* last match address, 0 = none */

	mov	%rdi, %rax
	and	$-32, %rax
	mov	%edi, %ecx
	and	$31, %ecx

	vmovdqa	(%rax), %ymm0
	vpcmpeqb %ymm1, %ymm0, %ymm3	/* == c */
	vpcmpeqb %ymm2, %ymm0, %ymm4	/* == NUL */
	vpmovmskb %ymm3, %r8d
	vpmovmskb %ymm4, %r11d
	shr	%cl, %r8d		/* start-relative */
	shr	%cl, %r11d
	mov	%rdi, %r10
	jmp	L(resolve)

	.p2align 4
L(loop):
	add	$32, %rax
	vmovdqa	(%rax), %ymm0
	vpcmpeqb %ymm1, %ymm0, %ymm3
	vpcmpeqb %ymm2, %ymm0, %ymm4
	vpmovmskb %ymm3, %r8d
	vpmovmskb %ymm4, %r11d
	mov	%rax, %r10

L(resolve):
	test	%r11d, %r11d
	jnz	L(has_nul)
	/* no NUL in this block: if any c, remember the LAST (highest bit) */
	test	%r8d, %r8d
	jz	L(loop)
	bsr	%r8d, %edx
	lea	(%r10, %rdx), %r9
	jmp	L(loop)

	.p2align 4
L(has_nul):
	/* NUL ends the string in this block. */
	bsf	%r11d, %ecx		/* first NUL bit position */
	test	%sil, %sil
	jnz	L(not_c0)
	/* Searching for '\0': the terminating NUL itself is the match. */
	lea	(%r10, %rcx), %r9
	jmp	L(return)
L(not_c0):
	/* Keep only c-matches strictly before the first NUL; highest = final. */
	mov	$1, %edx
	shl	%cl, %edx
	sub	$1, %edx		/* mask of bits before NUL */
	and	%edx, %r8d
	jz	L(return)		/* none here -> use prior r9 */
	bsr	%r8d, %edx
	lea	(%r10, %rdx), %r9
L(return):
	mov	%r9, %rax		/* r9 = last match (0 if never) */
	ret
END (STRRCHR)
