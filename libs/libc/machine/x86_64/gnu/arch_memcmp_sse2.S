/* memcmp with SSE2.
   Copyright (C) 2017-2022 Free Software Foundation, Inc.
   This file is part of the GNU C Library.

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, see
   <https://www.gnu.org/licenses/>.  */

/* MINIMUM_X86_ISA_LEVEL <= 2 because there is no V2 implementation
   so we need this to build for ISA V2 builds. */

#include "sysdep.h"

# ifndef MEMCMP
#  define MEMCMP __memcmp_sse2
# endif

# ifdef USE_AS_WMEMCMP
#  define PCMPEQ	pcmpeqd
#  define CHAR_SIZE	4
#  define SIZE_OFFSET	(0)
# else
#  define PCMPEQ	pcmpeqb
#  define CHAR_SIZE	1
# endif

# ifdef USE_AS_MEMCMPEQ
#  define SIZE_OFFSET	(0)
#  define CHECK_CMP(x, y)	subl x, y
# else
#  ifndef SIZE_OFFSET
#   define SIZE_OFFSET	(CHAR_PER_VEC * 2)
#  endif
#  define CHECK_CMP(x, y)	cmpl x, y
# endif

# define VEC_SIZE	16
# define CHAR_PER_VEC	(VEC_SIZE / CHAR_SIZE)

# ifndef MEMCMP
#  define MEMCMP	memcmp
# endif

	.text
ENTRY(MEMCMP)
#  ifdef __ILP32__
	/* Clear the upper 32 bits.  */
	movl	%edx, %edx
#  endif
# ifdef USE_AS_WMEMCMP
	/* Use 0xffff to test for mismatches on pmovmskb bitmask. Store
	   in ecx for code size. This is preferable to using `incw` as
	   it avoids partial register stalls on older hardware (pre
	   SnB).  */
	movl	$0xffff, %ecx
# endif
	cmpq	$CHAR_PER_VEC, %rdx
	ja	L(more_1x_vec)

# ifdef USE_AS_WMEMCMP
	/* saves a byte of code keeping the fall through path n = [2, 4]
	   in the initial cache line.  */
	decl	%edx
	jle	L(cmp_0_1)

	movq	(%rsi), %xmm0
	movq	(%rdi), %xmm1
	PCMPEQ	%xmm0, %xmm1
	pmovmskb %xmm1, %eax
	subl	%ecx, %eax
	jnz	L(ret_nonzero_vec_start_0)

	movq	-4(%rsi, %rdx, CHAR_SIZE), %xmm0
	movq	-4(%rdi, %rdx, CHAR_SIZE), %xmm1
	PCMPEQ	%xmm0, %xmm1
	pmovmskb %xmm1, %eax
	subl	%ecx, %eax
	jnz	L(ret_nonzero_vec_end_0_adj)
# else
	cmpl	$8, %edx
	ja	L(cmp_9_16)

	cmpl	$4, %edx
	jb	L(cmp_0_3)

#  ifdef USE_AS_MEMCMPEQ
	movl	(%rsi), %eax
	subl	(%rdi), %eax

	movl	-4(%rsi, %rdx), %esi
	subl	-4(%rdi, %rdx), %esi

	orl	%esi, %eax
	ret
#  else
	/* Combine comparisons for lo and hi 4-byte comparisons.  */
	movl	-4(%rsi, %rdx), %ecx
	movl	-4(%rdi, %rdx), %eax
	shlq	$32, %rcx
	shlq	$32, %rax
	movl	(%rsi), %esi
	movl	(%rdi), %edi
	orq	%rsi, %rcx
	orq	%rdi, %rax
	/* Only compute proper return if not-equal.  */
	cmpq	%rcx, %rax
	jnz	L(ret_nonzero)
	xorl	%eax, %eax
	ret
#  endif

	.p2align 4,, 10
L(cmp_9_16):
#  ifdef USE_AS_MEMCMPEQ
	movq	(%rsi), %rax
	subq	(%rdi), %rax

	movq	-8(%rsi, %rdx), %rcx
	subq	-8(%rdi, %rdx), %rcx
	orq	%rcx, %rax
	/* Convert 64 bit -> 32 bit boolean (we should have made the ABI
	   return long).  */
	setnz	%cl
	movzbl	%cl, %eax
#  else
	movq	(%rsi), %rcx
	movq	(%rdi), %rax
	/* Only compute proper return if not-equal.  */
	cmpq	%rcx, %rax
	jnz	L(ret_nonzero)

	movq	-8(%rsi, %rdx, CHAR_SIZE), %rcx
	movq	-8(%rdi, %rdx, CHAR_SIZE), %rax
	/* Only compute proper return if not-equal.  */
	cmpq	%rcx, %rax
	jnz	L(ret_nonzero)
	xorl	%eax, %eax
#  endif
# endif
	ret

	.p2align 4,, 8
L(cmp_0_1):
	/* Flag set by earlier comparison against 1.  */
	jne	L(cmp_0_0)
# ifdef USE_AS_WMEMCMP
	movl	(%rdi), %ecx
	xorl	%edx, %edx
	cmpl	(%rsi), %ecx
	je	L(cmp_0_0)
	setg	%dl
	leal	-1(%rdx, %rdx), %eax
# else
	movzbl	(%rdi), %eax
	movzbl	(%rsi), %ecx
	subl	%ecx, %eax
# endif
	ret

	/* Fits in aligning bytes.  */
L(cmp_0_0):
	xorl	%eax, %eax
	ret

# ifdef USE_AS_WMEMCMP
	.p2align 4
L(ret_nonzero_vec_start_0):
	bsfl	%eax, %eax
	movl	(%rdi, %rax), %ecx
	xorl	%edx, %edx
	cmpl	(%rsi, %rax), %ecx
	/* NB: no partial register stall here because xorl zero idiom
	   above.  */
	setg	%dl
	leal	-1(%rdx, %rdx), %eax
	ret
# else

#  ifndef USE_AS_MEMCMPEQ
	.p2align 4,, 14
L(ret_nonzero):
	/* Need to bswap to get proper return without branch.  */
	bswapq	%rcx
	bswapq	%rax
	subq	%rcx, %rax
	sbbl	%eax, %eax
	orl	$1, %eax
	ret
#  endif

	.p2align 4
L(cmp_0_3):
#  ifdef USE_AS_MEMCMPEQ
	/* No reason to add to dependency chain on rdx. Saving a the
	   bytes here doesn't change number of fetch blocks.  */
	cmpl	$1, %edx
	jbe	L(cmp_0_1)
#  else
	/* We need the code size to prevent taking an extra fetch block.
	 */
	decl	%edx
	jle	L(cmp_0_1)
#  endif
	movzwl	(%rsi), %ecx
	movzwl	(%rdi), %eax

#  ifdef USE_AS_MEMCMPEQ
	subl	%ecx, %eax

	movzbl	-1(%rsi, %rdx), %esi
	movzbl	-1(%rdi, %rdx), %edi
	subl	%edi, %esi
	orl	%esi, %eax
#  else
	bswapl	%ecx
	bswapl	%eax

	/* Implicit right shift by one. We just need to displace the
	   sign bits.  */
	shrl	%ecx
	shrl	%eax

	/* Eat a partial register stall here. Saves code stopping
	   L(cmp_0_3) from bleeding into the next fetch block and saves
	   an ALU.  */
	movb	(%rsi, %rdx), %cl
	movzbl	(%rdi, %rdx), %edi
	orl	%edi, %eax
	subl	%ecx, %eax
#  endif
	ret
# endif

	.p2align 5
L(more_1x_vec):
# ifndef USE_AS_WMEMCMP
	/* Use 0xffff to test for mismatches on pmovmskb bitmask. Store
	   in ecx for code size. This is preferable to using `incw` as
	   it avoids partial register stalls on older hardware (pre
	   SnB).  */
	movl	$0xffff, %ecx
# endif
	movups	(%rsi), %xmm0
	movups	(%rdi), %xmm1
	PCMPEQ	%xmm0, %xmm1
	pmovmskb %xmm1, %eax
	subl	%ecx, %eax
	jnz	L(ret_nonzero_vec_start_0)
# if SIZE_OFFSET == 0
	cmpq	$(CHAR_PER_VEC * 2), %rdx
# else
	/* Offset rdx. Saves just enough code size to keep the
	   L(last_2x_vec) case and the non-zero return in a single
	   cache line.  */
	subq	$(CHAR_PER_VEC * 2), %rdx
# endif
	ja	L(more_2x_vec)

	movups	(VEC_SIZE * -1 + SIZE_OFFSET)(%rsi, %rdx, CHAR_SIZE), %xmm0
	movups	(VEC_SIZE * -1 + SIZE_OFFSET)(%rdi, %rdx, CHAR_SIZE), %xmm1
	PCMPEQ	%xmm0, %xmm1
	pmovmskb %xmm1, %eax
	subl	%ecx, %eax
# ifndef USE_AS_MEMCMPEQ
	/* Don't use `incw ax` as machines this code runs on are liable
	   to have partial register stall.  */
	jnz	L(ret_nonzero_vec_end_0)
# else
	/* Various return targets for memcmpeq. Will always be hot in
	   Icache and get short encoding.  */
L(ret_nonzero_vec_start_1):
L(ret_nonzero_vec_start_0):
L(ret_nonzero_vec_end_0):
# endif
	ret

# ifndef USE_AS_MEMCMPEQ
#  ifdef USE_AS_WMEMCMP
	.p2align 4
L(ret_nonzero_vec_end_0_adj):
	addl	$3, %edx
#  else
	.p2align 4,, 8
#  endif
L(ret_nonzero_vec_end_0):
	bsfl	%eax, %eax
#  ifdef USE_AS_WMEMCMP
	leal	(%rax, %rdx, CHAR_SIZE), %eax
	movl	(VEC_SIZE * -1 + SIZE_OFFSET)(%rdi, %rax), %ecx
	xorl	%edx, %edx
	cmpl	(VEC_SIZE * -1 + SIZE_OFFSET)(%rsi, %rax), %ecx
	/* NB: no partial register stall here because xorl zero idiom
	   above.  */
	setg	%dl
	leal	-1(%rdx, %rdx), %eax
#  else
	addl	%edx, %eax
	movzbl	(VEC_SIZE * -1 + SIZE_OFFSET)(%rsi, %rax), %ecx
	movzbl	(VEC_SIZE * -1 + SIZE_OFFSET)(%rdi, %rax), %eax
	subl	%ecx, %eax
#  endif
	ret
#  ifndef USE_AS_WMEMCMP
	.p2align 4,, 10
L(ret_nonzero_vec_start_0):
	bsfl	%eax, %eax
	movzbl	(%rsi, %rax), %ecx
	movzbl	(%rdi, %rax), %eax
	subl	%ecx, %eax
	ret
#  endif
# else
# endif

	.p2align 5
L(more_2x_vec):
	movups	(VEC_SIZE * 1)(%rsi), %xmm0
	movups	(VEC_SIZE * 1)(%rdi), %xmm1
	PCMPEQ	%xmm0, %xmm1
	pmovmskb %xmm1, %eax
	subl	%ecx, %eax
	jnz	L(ret_nonzero_vec_start_1)

	cmpq	$(CHAR_PER_VEC * 4 - SIZE_OFFSET), %rdx
	jbe	L(last_2x_vec)

	cmpq	$(CHAR_PER_VEC * 8 - SIZE_OFFSET), %rdx
	ja	L(more_8x_vec)

	/* Do comparisons for [65, 96] and [97, 128] 2x VEC at a time.
	   This can harm performance if non-zero return in [65, 80] or
	   [97, 112] but helps performance otherwise. Generally zero-
	   return is hotter.  */
	movups	(VEC_SIZE * 2)(%rsi), %xmm0
	movups	(VEC_SIZE * 2)(%rdi), %xmm1
	PCMPEQ	%xmm0, %xmm1
	movups	(VEC_SIZE * 3)(%rsi), %xmm2
	movups	(VEC_SIZE * 3)(%rdi), %xmm3
	PCMPEQ	%xmm2, %xmm3
	pand	%xmm1, %xmm3

	pmovmskb %xmm3, %eax
	CHECK_CMP (%ecx, %eax)
	jnz	L(ret_nonzero_vec_start_2_3)

	cmpl	$(CHAR_PER_VEC * 6 - SIZE_OFFSET), %edx
	jbe	L(last_2x_vec)

	movups	(VEC_SIZE * 4)(%rsi), %xmm0
	movups	(VEC_SIZE * 4)(%rdi), %xmm1
	PCMPEQ	%xmm0, %xmm1
	movups	(VEC_SIZE * 5)(%rsi), %xmm2
	movups	(VEC_SIZE * 5)(%rdi), %xmm3
	PCMPEQ	%xmm2, %xmm3
	pand	%xmm1, %xmm3

	pmovmskb %xmm3, %eax
	CHECK_CMP (%ecx, %eax)
# ifdef USE_AS_MEMCMPEQ
	jz	L(last_2x_vec)
	ret
# else
	jnz	L(ret_nonzero_vec_start_4_5)
# endif
	.p2align 4
L(last_2x_vec):
	movups	(VEC_SIZE * -2 + SIZE_OFFSET)(%rsi, %rdx, CHAR_SIZE), %xmm0
	movups	(VEC_SIZE * -2 + SIZE_OFFSET)(%rdi, %rdx, CHAR_SIZE), %xmm1
	PCMPEQ	%xmm0, %xmm1
	movups	(VEC_SIZE * -1 + SIZE_OFFSET)(%rsi, %rdx, CHAR_SIZE), %xmm2
	movups	(VEC_SIZE * -1 + SIZE_OFFSET)(%rdi, %rdx, CHAR_SIZE), %xmm3
	PCMPEQ	%xmm2, %xmm3
	pand	%xmm1, %xmm3
	pmovmskb %xmm3, %eax
	subl	%ecx, %eax
# ifdef USE_AS_MEMCMPEQ
	/* Various return targets for memcmpeq. Will always be hot in
	   Icache and get short encoding.  */
L(ret_nonzero_vec_start_2_3):
L(ret_nonzero_vec_start_4_5):
	ret
# else
	jnz	L(ret_nonzero_vec_end_1)
	ret

	.p2align 4,, 8
L(ret_nonzero_vec_end_1):
	pmovmskb %xmm1, %ecx
	/* High 16 bits of eax guranteed to be all ones. Rotate them in
	   to we can do `or + not` with just `xor`.  */
	rorl	$16, %eax
	xorl	%ecx, %eax
	/* Partial register stall.  */

	bsfl	%eax, %eax
#  ifdef USE_AS_WMEMCMP
	leal	(%rax, %rdx, CHAR_SIZE), %eax
	movl	(VEC_SIZE * -2 + SIZE_OFFSET)(%rdi, %rax), %ecx
	xorl	%edx, %edx
	cmpl	(VEC_SIZE * -2 + SIZE_OFFSET)(%rsi, %rax), %ecx
	/* NB: no partial register stall here because xorl zero idiom
	   above.  */
	setg	%dl
	leal	-1(%rdx, %rdx), %eax
#  else
	addl	%edx, %eax
	movzbl	(VEC_SIZE * -2 + SIZE_OFFSET)(%rsi, %rax), %ecx
	movzbl	(VEC_SIZE * -2 + SIZE_OFFSET)(%rdi, %rax), %eax
	subl	%ecx, %eax
#  endif
	ret

	.p2align 4
L(ret_nonzero_vec_start_4_5):
	pmovmskb %xmm1, %edx
	sall	$16, %eax
	leal	1(%rax, %rdx), %eax
	bsfl	%eax, %eax
#  ifdef USE_AS_WMEMCMP
	movl	(VEC_SIZE * 4)(%rdi, %rax), %ecx
	xorl	%edx, %edx
	cmpl	(VEC_SIZE * 4)(%rsi, %rax), %ecx
	/* NB: no partial register stall here because xorl zero idiom
	   above.  */
	setg	%dl
	leal	-1(%rdx, %rdx), %eax
#  else
	movzbl	(VEC_SIZE * 4)(%rsi, %rax), %ecx
	movzbl	(VEC_SIZE * 4)(%rdi, %rax), %eax
	subl	%ecx, %eax
#  endif
	ret

	.p2align 4,, 8
L(ret_nonzero_vec_start_1):
	bsfl	%eax, %eax
#  ifdef USE_AS_WMEMCMP
	movl	(VEC_SIZE * 1)(%rdi, %rax), %ecx
	xorl	%edx, %edx
	cmpl	(VEC_SIZE * 1)(%rsi, %rax), %ecx
	/* NB: no partial register stall here because xorl zero idiom
	   above.  */
	setg	%dl
	leal	-1(%rdx, %rdx), %eax
#  else
	movzbl	(VEC_SIZE * 1)(%rsi, %rax), %ecx
	movzbl	(VEC_SIZE * 1)(%rdi, %rax), %eax
	subl	%ecx, %eax
#  endif
	ret
# endif

	.p2align 4
L(more_8x_vec):
	subq	%rdi, %rsi
	leaq	(VEC_SIZE * -6 + SIZE_OFFSET)(%rdi, %rdx, CHAR_SIZE), %rdx
	andq	$(VEC_SIZE * -1), %rdi
	addq	%rdi, %rsi
	.p2align 4
L(loop_4x):
	movups	(VEC_SIZE * 2)(%rsi), %xmm0
	movups	(VEC_SIZE * 3)(%rsi), %xmm1

	PCMPEQ	(VEC_SIZE * 2)(%rdi), %xmm0
	PCMPEQ	(VEC_SIZE * 3)(%rdi), %xmm1

	movups	(VEC_SIZE * 4)(%rsi), %xmm2
	movups	(VEC_SIZE * 5)(%rsi), %xmm3

	PCMPEQ	(VEC_SIZE * 4)(%rdi), %xmm2
	PCMPEQ	(VEC_SIZE * 5)(%rdi), %xmm3

	pand	%xmm0, %xmm1
	pand	%xmm2, %xmm3
	pand	%xmm1, %xmm3

	pmovmskb %xmm3, %eax
	subl	%ecx, %eax
	jnz	L(ret_nonzero_loop)

	addq	$(VEC_SIZE * 4), %rdi
	addq	$(VEC_SIZE * 4), %rsi
	cmpq	%rdi, %rdx
	ja	L(loop_4x)
	/* Get remaining length in edx.  */
	subl	%edi, %edx
	/* Restore offset so we can reuse L(last_2x_vec).  */
	addl	$(VEC_SIZE * 6 - SIZE_OFFSET), %edx
# ifdef USE_AS_WMEMCMP
	shrl	$2, %edx
# endif
	cmpl	$(CHAR_PER_VEC * 4 - SIZE_OFFSET), %edx
	jbe	L(last_2x_vec)


	movups	(VEC_SIZE * 2)(%rsi), %xmm0
	movups	(VEC_SIZE * 2)(%rdi), %xmm1
	PCMPEQ	%xmm0, %xmm1
	movups	(VEC_SIZE * 3)(%rsi), %xmm2
	movups	(VEC_SIZE * 3)(%rdi), %xmm3
	PCMPEQ	%xmm2, %xmm3
	pand	%xmm1, %xmm3

	pmovmskb %xmm3, %eax
	CHECK_CMP (%ecx, %eax)
	jz	L(last_2x_vec)
# ifdef USE_AS_MEMCMPEQ
L(ret_nonzero_loop):
	ret
# else

	.p2align 4
L(ret_nonzero_vec_start_2_3):
	pmovmskb %xmm1, %edx
	sall	$16, %eax
	leal	1(%rax, %rdx), %eax

	bsfl	%eax, %eax
#  ifdef USE_AS_WMEMCMP
	movl	(VEC_SIZE * 2)(%rdi, %rax), %ecx
	xorl	%edx, %edx
	cmpl	(VEC_SIZE * 2)(%rsi, %rax), %ecx
	/* NB: no partial register stall here because xorl zero idiom
	   above.  */
	setg	%dl
	leal	-1(%rdx, %rdx), %eax
#  else
	movzbl	(VEC_SIZE * 2)(%rsi, %rax), %ecx
	movzbl	(VEC_SIZE * 2)(%rdi, %rax), %eax
	subl	%ecx, %eax
#  endif
	ret

	.p2align 4
L(ret_nonzero_loop):
	pmovmskb %xmm0, %ecx
	pmovmskb %xmm1, %edx
	sall	$(VEC_SIZE * 1), %edx
	leal	1(%rcx, %rdx), %edx
	pmovmskb %xmm2, %ecx
	/* High 16 bits of eax guranteed to be all ones. Rotate them in
	   to we can do `or + not` with just `xor`.  */
	rorl	$16, %eax
	xorl	%ecx, %eax

	salq	$32, %rax
	orq	%rdx, %rax

	bsfq	%rax, %rax
#  ifdef USE_AS_WMEMCMP
	movl	(VEC_SIZE * 2)(%rdi, %rax), %ecx
	xorl	%edx, %edx
	cmpl	(VEC_SIZE * 2)(%rsi, %rax), %ecx
	/* NB: no partial register stall here because xorl zero idiom
	   above.  */
	setg	%dl
	leal	-1(%rdx, %rdx), %eax
#  else
	movzbl	(VEC_SIZE * 2)(%rsi, %rax), %ecx
	movzbl	(VEC_SIZE * 2)(%rdi, %rax), %eax
	subl	%ecx, %eax
#  endif
	ret
# endif
END(MEMCMP)
