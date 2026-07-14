/****************************************************************************
 * libs/libc/machine/x86_64/arch_strnlen_avx2.S
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

/* Page-safe AVX2 strnlen. Returns min(strlen(s), maxlen). Bounded scan for
 * NUL in [s, s+maxlen). 32-byte-aligned-down loads never cross a page; a NUL
 * bit is only honored if its address is < s+maxlen, and the loop stops once
 * the aligned pointer passes the end.
 *
 * size_t strnlen(const char *s /rdi/, size_t maxlen /rsi/)
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#include "libc.h"

#ifndef STRNLEN
# define STRNLEN	ARCH_LIBCFUN(strnlen)
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
ENTRY (STRNLEN)
	test	%rsi, %rsi
	jz	L(ret_zero)		/* maxlen == 0 -> 0 */

	vpxor	%xmm2, %xmm2, %xmm2
	mov	%rdi, %r11
	add	%rsi, %r11		/* end = s + maxlen; CF set on wrap */
	sbb	%rax, %rax		/* rax = 0, or -1 if maxlen wrapped */
	or	%rax, %r11		/* saturate end to max address on wrap */
	mov	%rdi, %rax
	and	$-32, %rax
	mov	%edi, %ecx
	and	$31, %ecx

	vmovdqa	(%rax), %ymm0
	vpcmpeqb %ymm2, %ymm0, %ymm3
	vpmovmskb %ymm3, %r8d
	shr	%cl, %r8d		/* start-relative NUL mask */
	mov	%rdi, %r10
	test	%r8d, %r8d
	jnz	L(check_first)
	add	$32, %rax
L(loop):
	cmp	%r11, %rax
	jae	L(ret_max)		/* scanned to end, no NUL -> maxlen */
	vmovdqa	(%rax), %ymm0
	vpcmpeqb %ymm2, %ymm0, %ymm3
	vpmovmskb %ymm3, %r8d
	mov	%rax, %r10
	test	%r8d, %r8d
	jz	L(advance)
	bsf	%r8d, %edx
	lea	(%r10, %rdx), %rdx	/* NUL address */
	cmp	%r11, %rdx
	jae	L(ret_max)		/* NUL at/after end -> maxlen */
	sub	%rdi, %rdx		/* length = NUL - s */
	mov	%rdx, %rax
	ret
L(advance):
	add	$32, %rax
	jmp	L(loop)

	.p2align 4
L(check_first):
	bsf	%r8d, %edx
	lea	(%r10, %rdx), %rdx
	cmp	%r11, %rdx
	jae	L(ret_max)
	sub	%rdi, %rdx
	mov	%rdx, %rax
	ret

	.p2align 4
L(ret_max):
	mov	%rsi, %rax		/* maxlen */
	ret
	.p2align 4
L(ret_zero):
	xor	%eax, %eax
	ret
END (STRNLEN)
