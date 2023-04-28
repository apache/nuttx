/****************************************************************************
 * libs/libc/machine/arm/armv7-a/gnu/arch_memmove.S
 *
 * Copyright (c) 2015 ARM Ltd
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the company may not be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ARM LTD ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ARM LTD BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "libc.h"

#ifdef LIBC_BUILD_STRING

	.thumb
	.syntax unified
	.global memmove
	.type	memmove, %function
memmove:
	cmp	r0, r1
	push	{r4}
	bls	3f
	adds	r3, r1, r2
	cmp	r0, r3
	bcs	3f
	adds	r1, r0, r2
	cbz	r2, 2f
	subs	r2, r3, r2
1:
	ldrb	r4, [r3, #-1]!
	cmp	r2, r3
	strb	r4, [r1, #-1]!
	bne	1b
2:
	pop	{r4}
	bx	lr
3:
	cmp	r2, #0
	beq	2b
	add	r2, r2, r1
	subs	r3, r0, #1
4:
	ldrb	r4, [r1], #1
	cmp	r2, r1
	strb	r4, [r3, #1]!
	bne	4b
	pop	{r4}
	bx	lr
	.size memmove, . - memmove

#endif
