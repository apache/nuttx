/****************************************************************************
 * libs/libc/machine/x86_64/arch_strchr_avx2.S
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

/* Page-safe AVX2 strchr. The first vector load is aligned DOWN to a 32-byte
 * boundary so it can never cross into an unmapped page (32 divides the 4096
 * page size); bytes before the real start are masked off with shr. Later
 * loads are naturally 32-byte aligned and therefore also page-safe. The
 * decision is made purely from the c-match and NUL masks without re-reading
 * memory: if the earliest c-match is at or before the earliest NUL it is the
 * hit, otherwise the string ended first and c is not present.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef STRCHR
# define STRCHR		strchr
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
ENTRY (STRCHR)
	vmovd	%esi, %xmm1
	vpbroadcastb %xmm1, %ymm1	/* ymm1 = c in every byte */
	vpxor	%xmm2, %xmm2, %xmm2	/* ymm2 = 0 (NUL detector) */

	mov	%rdi, %rax
	and	$-32, %rax		/* aligned-down base (page-safe load) */
	mov	%edi, %ecx
	and	$31, %ecx		/* bytes to drop at front */

	vmovdqa	(%rax), %ymm0
	vpcmpeqb %ymm1, %ymm0, %ymm3	/* == c */
	vpcmpeqb %ymm2, %ymm0, %ymm4	/* == NUL */
	vpmovmskb %ymm3, %r8d		/* c-mask (block-relative) */
	vpmovmskb %ymm4, %r9d		/* NUL-mask */
	shr	%cl, %r8d		/* start-relative */
	shr	%cl, %r9d
	mov	%rdi, %r10		/* base for these bits = real start */
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
	jz	L(loop)			/* neither c nor NUL in this block */

	test	%r8d, %r8d
	jz	L(not_found)		/* no c here; NUL present -> not found */
	bsf	%r8d, %r8d		/* first c position */
	test	%r9d, %r9d
	jz	L(c_hit)		/* no NUL here -> c is the hit */
	bsf	%r9d, %r9d		/* first NUL position */
	cmp	%r9d, %r8d
	ja	L(not_found)		/* NUL earlier than c -> not found */
L(c_hit):
	lea	(%r10, %r8), %rax
	ret

	.p2align 4
L(not_found):
	xor	%eax, %eax
	ret
END (STRCHR)
