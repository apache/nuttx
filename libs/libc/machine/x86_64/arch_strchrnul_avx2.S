/****************************************************************************
 * libs/libc/machine/x86_64/arch_strchrnul_avx2.S
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

/* Page-safe AVX2 strchrnul. Same scan as strchr but when c is not found it
 * returns a pointer to the terminating NUL (not NULL). First vector load is
 * aligned down to a 32-byte boundary so it never crosses into an unmapped
 * page; later loads are naturally aligned.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef STRCHRNUL
# define STRCHRNUL	strchrnul
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
ENTRY (STRCHRNUL)
	vmovd	%esi, %xmm1
	vpbroadcastb %xmm1, %ymm1
	vpxor	%xmm2, %xmm2, %xmm2

	mov	%rdi, %rax
	and	$-32, %rax
	mov	%edi, %ecx
	and	$31, %ecx

	vmovdqa	(%rax), %ymm0
	vpcmpeqb %ymm1, %ymm0, %ymm3
	vpcmpeqb %ymm2, %ymm0, %ymm4
	vpmovmskb %ymm3, %r8d
	vpmovmskb %ymm4, %r9d
	shr	%cl, %r8d
	shr	%cl, %r9d
	mov	%rdi, %r10
	jmp	L(resolve)

	.p2align 4
L(loop):
	add	$32, %rax
	vmovdqa	(%rax), %ymm0
	vpcmpeqb %ymm1, %ymm0, %ymm3
	vpcmpeqb %ymm2, %ymm0, %ymm4
	vpmovmskb %ymm3, %r8d
	vpmovmskb %ymm4, %r9d
	mov	%rax, %r10

L(resolve):
	mov	%r8d, %edx
	or	%r9d, %edx
	test	%edx, %edx
	jz	L(loop)

	test	%r8d, %r8d
	jz	L(nul_only)		/* no c; return NUL position */
	bsf	%r8d, %r8d
	test	%r9d, %r9d
	jz	L(c_hit)		/* no NUL -> c is the hit */
	bsf	%r9d, %r9d
	cmp	%r9d, %r8d
	ja	L(nul_pos)		/* NUL earlier than c -> return NUL pos */
L(c_hit):
	lea	(%r10, %r8), %rax
	ret

	.p2align 4
L(nul_only):
	bsf	%r9d, %r9d
L(nul_pos):
	lea	(%r10, %r9), %rax	/* return pointer to NUL */
	ret
END (STRCHRNUL)
