/* memrchr optimized with SSE2.
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

# ifndef MEMRCHR
#  define MEMRCHR memrchr
# endif

# include "sysdep.h"
# define VEC_SIZE			16
# define PAGE_SIZE			4096

	.text
ENTRY_P2ALIGN(MEMRCHR, 6)
# ifdef __ILP32__
	/* Clear upper bits.  */
	mov	%RDX_LP, %RDX_LP
# endif
	movd	%esi, %xmm0

	/* Get end pointer.  */
	leaq	(%rdx, %rdi), %rcx

	punpcklbw %xmm0, %xmm0
	punpcklwd %xmm0, %xmm0
	pshufd	$0, %xmm0, %xmm0

	/* Check if we can load 1x VEC without cross a page.  */
	testl	$(PAGE_SIZE - VEC_SIZE), %ecx
	jz	L(page_cross)

	/* NB: This load happens regardless of whether rdx (len) is zero. Since
	   it doesn't cross a page and the standard gurantees any pointer have
	   at least one-valid byte this load must be safe. For the entire
	   history of the x86 memrchr implementation this has been possible so
	   no code "should" be relying on a zero-length check before this load.
	   The zero-length check is moved to the page cross case because it is
	   1) pretty cold and including it pushes the hot case len <= VEC_SIZE
	   into 2-cache lines.  */
	movups	-(VEC_SIZE)(%rcx), %xmm1
	pcmpeqb	%xmm0, %xmm1
	pmovmskb %xmm1, %eax

	subq	$VEC_SIZE, %rdx
	ja	L(more_1x_vec)
L(ret_vec_x0_test):
	/* Zero-flag set if eax (src) is zero. Destination unchanged if src is
	   zero.  */
	bsrl	%eax, %eax
	jz	L(ret_0)
	/* Check if the CHAR match is in bounds. Need to truly zero `eax` here
	   if out of bounds.  */
	addl	%edx, %eax
	jl	L(zero_0)
	/* Since we subtracted VEC_SIZE from rdx earlier we can just add to base
	   ptr.  */
	addq	%rdi, %rax
L(ret_0):
	ret

	.p2align 4,, 5
L(ret_vec_x0):
	bsrl	%eax, %eax
	leaq	-(VEC_SIZE)(%rcx, %rax), %rax
	ret

	.p2align 4,, 2
L(zero_0):
	xorl	%eax, %eax
	ret


	.p2align 4,, 8
L(more_1x_vec):
	testl	%eax, %eax
	jnz	L(ret_vec_x0)

	/* Align rcx (pointer to string).  */
	decq	%rcx
	andq	$-VEC_SIZE, %rcx

	movq	%rcx, %rdx
	/* NB: We could consistenyl save 1-byte in this pattern with `movaps
	   %xmm0, %xmm1; pcmpeq IMM8(r), %xmm1; ...`. The reason against it is
	   it adds more frontend uops (even if the moves can be eliminated) and
	   some percentage of the time actual backend uops.  */
	movaps	-(VEC_SIZE)(%rcx), %xmm1
	pcmpeqb	%xmm0, %xmm1
	subq	%rdi, %rdx
	pmovmskb %xmm1, %eax

	cmpq	$(VEC_SIZE * 2), %rdx
	ja	L(more_2x_vec)
L(last_2x_vec):
	subl	$VEC_SIZE, %edx
	jbe	L(ret_vec_x0_test)

	testl	%eax, %eax
	jnz	L(ret_vec_x0)

	movaps	-(VEC_SIZE * 2)(%rcx), %xmm1
	pcmpeqb	%xmm0, %xmm1
	pmovmskb %xmm1, %eax

	subl	$VEC_SIZE, %edx
	bsrl	%eax, %eax
	jz	L(ret_1)
	addl	%edx, %eax
	jl	L(zero_0)
	addq	%rdi, %rax
L(ret_1):
	ret

	/* Don't align. Otherwise lose 2-byte encoding in jump to L(page_cross)
	   causes the hot pause (length <= VEC_SIZE) to span multiple cache
	   lines.  Naturally aligned % 16 to 8-bytes.  */
L(page_cross):
	/* Zero length check.  */
	testq	%rdx, %rdx
	jz	L(zero_0)

	leaq	-1(%rcx), %r8
	andq	$-(VEC_SIZE), %r8

	movaps	(%r8), %xmm1
	pcmpeqb	%xmm0, %xmm1
	pmovmskb %xmm1, %esi
	/* Shift out negative alignment (because we are starting from endptr and
	   working backwards).  */
	negl	%ecx
	/* 32-bit shift but VEC_SIZE=16 so need to mask the shift count
	   explicitly.  */
	andl	$(VEC_SIZE - 1), %ecx
	shl	%cl, %esi
	movzwl	%si, %eax
	leaq	(%rdi, %rdx), %rcx
	cmpq	%rdi, %r8
	ja	L(more_1x_vec)
	subl	$VEC_SIZE, %edx
	bsrl	%eax, %eax
	jz	L(ret_2)
	addl	%edx, %eax
	jl	L(zero_1)
	addq	%rdi, %rax
L(ret_2):
	ret

	/* Fits in aliging bytes.  */
L(zero_1):
	xorl	%eax, %eax
	ret

	.p2align 4,, 5
L(ret_vec_x1):
	bsrl	%eax, %eax
	leaq	-(VEC_SIZE * 2)(%rcx, %rax), %rax
	ret

	.p2align 4,, 8
L(more_2x_vec):
	testl	%eax, %eax
	jnz	L(ret_vec_x0)

	movaps	-(VEC_SIZE * 2)(%rcx), %xmm1
	pcmpeqb	%xmm0, %xmm1
	pmovmskb %xmm1, %eax
	testl	%eax, %eax
	jnz	L(ret_vec_x1)


	movaps	-(VEC_SIZE * 3)(%rcx), %xmm1
	pcmpeqb	%xmm0, %xmm1
	pmovmskb %xmm1, %eax

	subq	$(VEC_SIZE * 4), %rdx
	ja	L(more_4x_vec)

	addl	$(VEC_SIZE), %edx
	jle	L(ret_vec_x2_test)

L(last_vec):
	testl	%eax, %eax
	jnz	L(ret_vec_x2)

	movaps	-(VEC_SIZE * 4)(%rcx), %xmm1
	pcmpeqb	%xmm0, %xmm1
	pmovmskb %xmm1, %eax

	subl	$(VEC_SIZE), %edx
	bsrl	%eax, %eax
	jz	L(ret_3)
	addl	%edx, %eax
	jl	L(zero_2)
	addq	%rdi, %rax
L(ret_3):
	ret

	.p2align 4,, 6
L(ret_vec_x2_test):
	bsrl	%eax, %eax
	jz	L(zero_2)
	addl	%edx, %eax
	jl	L(zero_2)
	addq	%rdi, %rax
	ret

L(zero_2):
	xorl	%eax, %eax
	ret


	.p2align 4,, 5
L(ret_vec_x2):
	bsrl	%eax, %eax
	leaq	-(VEC_SIZE * 3)(%rcx, %rax), %rax
	ret

	.p2align 4,, 5
L(ret_vec_x3):
	bsrl	%eax, %eax
	leaq	-(VEC_SIZE * 4)(%rcx, %rax), %rax
	ret

	.p2align 4,, 8
L(more_4x_vec):
	testl	%eax, %eax
	jnz	L(ret_vec_x2)

	movaps	-(VEC_SIZE * 4)(%rcx), %xmm1
	pcmpeqb	%xmm0, %xmm1
	pmovmskb %xmm1, %eax

	testl	%eax, %eax
	jnz	L(ret_vec_x3)

	addq	$-(VEC_SIZE * 4), %rcx
	cmpq	$(VEC_SIZE * 4), %rdx
	jbe	L(last_4x_vec)

	/* Offset everything by 4x VEC_SIZE here to save a few bytes at the end
	   keeping the code from spilling to the next cache line.  */
	addq	$(VEC_SIZE * 4 - 1), %rcx
	andq	$-(VEC_SIZE * 4), %rcx
	leaq	(VEC_SIZE * 4)(%rdi), %rdx
	andq	$-(VEC_SIZE * 4), %rdx

	.p2align 4,, 11
L(loop_4x_vec):
	movaps	(VEC_SIZE * -1)(%rcx), %xmm1
	movaps	(VEC_SIZE * -2)(%rcx), %xmm2
	movaps	(VEC_SIZE * -3)(%rcx), %xmm3
	movaps	(VEC_SIZE * -4)(%rcx), %xmm4
	pcmpeqb	%xmm0, %xmm1
	pcmpeqb	%xmm0, %xmm2
	pcmpeqb	%xmm0, %xmm3
	pcmpeqb	%xmm0, %xmm4

	por	%xmm1, %xmm2
	por	%xmm3, %xmm4
	por	%xmm2, %xmm4

	pmovmskb %xmm4, %esi
	testl	%esi, %esi
	jnz	L(loop_end)

	addq	$-(VEC_SIZE * 4), %rcx
	cmpq	%rdx, %rcx
	jne	L(loop_4x_vec)

	subl	%edi, %edx

	/* Ends up being 1-byte nop.  */
	.p2align 4,, 2
L(last_4x_vec):
	movaps	-(VEC_SIZE)(%rcx), %xmm1
	pcmpeqb	%xmm0, %xmm1
	pmovmskb %xmm1, %eax

	cmpl	$(VEC_SIZE * 2), %edx
	jbe	L(last_2x_vec)

	testl	%eax, %eax
	jnz	L(ret_vec_x0)


	movaps	-(VEC_SIZE * 2)(%rcx), %xmm1
	pcmpeqb	%xmm0, %xmm1
	pmovmskb %xmm1, %eax

	testl	%eax, %eax
	jnz	L(ret_vec_end)

	movaps	-(VEC_SIZE * 3)(%rcx), %xmm1
	pcmpeqb	%xmm0, %xmm1
	pmovmskb %xmm1, %eax

	subl	$(VEC_SIZE * 3), %edx
	ja	L(last_vec)
	bsrl	%eax, %eax
	jz	L(ret_4)
	addl	%edx, %eax
	jl	L(zero_3)
	addq	%rdi, %rax
L(ret_4):
	ret

	/* Ends up being 1-byte nop.  */
	.p2align 4,, 3
L(loop_end):
	pmovmskb %xmm1, %eax
	sall	$16, %eax
	jnz	L(ret_vec_end)

	pmovmskb %xmm2, %eax
	testl	%eax, %eax
	jnz	L(ret_vec_end)

	pmovmskb %xmm3, %eax
	/* Combine last 2 VEC matches. If ecx (VEC3) is zero (no CHAR in VEC3)
	   then it won't affect the result in esi (VEC4). If ecx is non-zero
	   then CHAR in VEC3 and bsrq will use that position.  */
	sall	$16, %eax
	orl	%esi, %eax
	bsrl	%eax, %eax
	leaq	-(VEC_SIZE * 4)(%rcx, %rax), %rax
	ret

L(ret_vec_end):
	bsrl	%eax, %eax
	leaq	(VEC_SIZE * -2)(%rax, %rcx), %rax
	ret
	/* Use in L(last_4x_vec). In the same cache line. This is just a spare
	   aligning bytes.  */
L(zero_3):
	xorl	%eax, %eax
	ret
	/* 2-bytes from next cache line.  */
END(MEMRCHR)
