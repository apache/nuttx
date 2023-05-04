/****************************************************************************
 * libs/libc/machine/arm/armv7-m/gnu/arch_strcmp.S
 *
 * Copyright (c) 2012-2014 ARM Ltd
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

/* Very similar to the generic code, but uses Thumb2 as implemented
   in ARMv7-M.  */

#include "arm_asm.h"

#ifdef __ARM_BIG_ENDIAN
#define S2LO lsl
#define S2LOEQ lsleq
#define S2HI lsr
#define MSB 0x000000ff
#define LSB 0xff000000
#define BYTE0_OFFSET 24
#define BYTE1_OFFSET 16
#define BYTE2_OFFSET 8
#define BYTE3_OFFSET 0
#else /* not  __ARM_BIG_ENDIAN */
#define S2LO lsr
#define S2LOEQ lsreq
#define S2HI lsl
#define BYTE0_OFFSET 0
#define BYTE1_OFFSET 8
#define BYTE2_OFFSET 16
#define BYTE3_OFFSET 24
#define MSB 0xff000000
#define LSB 0x000000ff
#endif /* not  __ARM_BIG_ENDIAN */

/* Parameters and result.  */
#define src1		r0
#define src2		r1
#define result		r0	/* Overlaps src1.  */

/* Internal variables.  */
#define data1		r2
#define data2		r3
#define tmp2		r5
#define tmp1		r12
#define syndrome	r12	/* Overlaps tmp1 */

	.macro def_fn f p2align=0
	.text
	.p2align \p2align
	.global \f
	.type \f, %function
\f:
	.endm

	.thumb
	.syntax unified
def_fn strcmp
	.fnstart
	.cfi_sections .debug_frame
	.cfi_startproc
	prologue push_ip=HAVE_PAC_LEAF
	eor	tmp1, src1, src2
	tst	tmp1, #3
	/* Strings not at same byte offset from a word boundary.  */
	bne	.Lstrcmp_unaligned
	ands	tmp1, src1, #3
	bic	src1, src1, #3
	bic	src2, src2, #3
	ldr	data1, [src1], #4
	it	eq
	ldreq	data2, [src2], #4
	beq	4f
	/* Although s1 and s2 have identical initial alignment, they are
	   not currently word aligned.	Rather than comparing bytes,
	   make sure that any bytes fetched from before the addressed
	   bytes are forced to 0xff.  Then they will always compare
	   equal.  */
	eor	tmp1, tmp1, #3
	mvn	data2, #MSB
	lsl	tmp1, tmp1, #3
	S2LO	tmp1, data2, tmp1
	ldr	data2, [src2], #4
	orr	data1, data1, tmp1
	orr	data2, data2, tmp1
	.p2align	2
	/* Critical loop.  */
4:
	sub	syndrome, data1, #0x01010101
	cmp	data1, data2
	/* check for any zero bytes in first word */
	itttt	eq
	biceq	syndrome, syndrome, data1
	tsteq	syndrome, #0x80808080
	ldreq	data1, [src1], #4
	ldreq	data2, [src2], #4
	beq	4b
2:
	.cfi_remember_state
	/* There's a zero or a different byte in the word */
	S2HI	result, data1, #24
	S2LO	data1, data1, #8
	cmp	result, #1
	it	cs
	cmpcs	result, data2, S2HI #24
	it	eq
	S2LOEQ	data2, data2, #8
	beq	2b
	/* On a big-endian machine, RESULT contains the desired byte in bits
	   0-7; on a little-endian machine they are in bits 24-31.  In
	   both cases the other bits in RESULT are all zero.  For DATA2 the
	   interesting byte is at the other end of the word, but the
	   other bits are not necessarily zero.	 We need a signed result
	   representing the differnece in the unsigned bytes, so for the
	   little-endian case we can't just shift the interesting bits
	   up.	*/
#ifdef __ARM_BIG_ENDIAN
	sub	result, result, data2, lsr #24
#else
	and	data2, data2, #255
	lsrs	result, result, #24
	subs	result, result, data2
#endif
	epilogue push_ip=HAVE_PAC_LEAF


#if 0
	/* The assembly code below is based on the following alogrithm.	 */
#ifdef __ARM_BIG_ENDIAN
#define RSHIFT <<
#define LSHIFT >>
#else
#define RSHIFT >>
#define LSHIFT <<
#endif

#define body(shift)							\
  mask = 0xffffffffU RSHIFT shift;					\
  data1 = *src1++;							\
  data2 = *src2++;							\
  do									\
    {									\
      tmp2 = data1 & mask;						\
      if (__builtin_expect(tmp2 != data2 RSHIFT shift, 0))		\
	{								\
	  data2 RSHIFT= shift;						\
	  break;							\
	}								\
      if (__builtin_expect(((data1 - b1) & ~data1) & (b1 << 7), 0))	\
	{								\
	  /* See comment in assembler below re syndrome on big-endian */\
	  if ((((data1 - b1) & ~data1) & (b1 << 7)) & mask)		\
	    data2 RSHIFT= shift;					\
	  else								\
	    {								\
	      data2 = *src2;						\
	      tmp2 = data1 RSHIFT (32 - shift);				\
	      data2 = (data2 LSHIFT (32 - shift)) RSHIFT (32 - shift);	\
	    }								\
	  break;							\
	}								\
      data2 = *src2++;							\
      tmp2 ^= data1;							\
      if (__builtin_expect(tmp2 != data2 LSHIFT (32 - shift), 0))	\
	{								\
	  tmp2 = data1 >> (32 - shift);					\
	  data2 = (data2 << (32 - shift)) RSHIFT (32 - shift);		\
	  break;							\
	}								\
      data1 = *src1++;							\
    } while (1)

  const unsigned* src1;
  const unsigned* src2;
  unsigned data1, data2;
  unsigned mask;
  unsigned shift;
  unsigned b1 = 0x01010101;
  char c1, c2;
  unsigned tmp2;

  while (((unsigned) s1) & 3)
    {
      c1 = *s1++;
      c2 = *s2++;
      if (c1 == 0 || c1 != c2)
	return c1 - (int)c2;
    }
  src1 = (unsigned*) (((unsigned)s1) & ~3);
  src2 = (unsigned*) (((unsigned)s2) & ~3);
  tmp2 = ((unsigned) s2) & 3;
  if (tmp2 == 1)
    {
      body(8);
    }
  else if (tmp2 == 2)
    {
      body(16);
    }
  else
    {
      body (24);
    }

  do
    {
#ifdef __ARM_BIG_ENDIAN
      c1 = (char) tmp2 >> 24;
      c2 = (char) data2 >> 24;
#else /* not  __ARM_BIG_ENDIAN */
      c1 = (char) tmp2;
      c2 = (char) data2;
#endif /* not  __ARM_BIG_ENDIAN */
      tmp2 RSHIFT= 8;
      data2 RSHIFT= 8;
    } while (c1 != 0 && c1 == c2);
  return c1 - c2;
#endif /* 0 */


	/* First of all, compare bytes until src1(sp1) is word-aligned. */
.Lstrcmp_unaligned:
	.cfi_restore_state
	tst	src1, #3
	beq	2f
	.cfi_remember_state
	ldrb	data1, [src1], #1
	ldrb	data2, [src2], #1
	cmp	data1, #1
	it	cs
	cmpcs	data1, data2
	beq	.Lstrcmp_unaligned
	sub	result, data1, data2
	epilogue push_ip=HAVE_PAC_LEAF

2:
	.cfi_restore_state
	stmfd	sp!, {r5}
	.cfi_adjust_cfa_offset 4
	.cfi_rel_offset 5, 0

	ldr	data1, [src1], #4
	and	tmp2, src2, #3
	bic	src2, src2, #3
	ldr	data2, [src2], #4
	cmp	tmp2, #2
	beq	.Loverlap2
	bhi	.Loverlap1

	/* Critical inner Loop: Block with 3 bytes initial overlap */
	.p2align	2
.Loverlap3:
	bic	tmp2, data1, #MSB
	cmp	tmp2, data2, S2LO #8
	sub	syndrome, data1, #0x01010101
	bic	syndrome, syndrome, data1
	bne	4f
	ands	syndrome, syndrome, #0x80808080
	it	eq
	ldreq	data2, [src2], #4
	bne	5f
	eor	tmp2, tmp2, data1
	cmp	tmp2, data2, S2HI #24
	bne	6f
	ldr	data1, [src1], #4
	b	.Loverlap3
4:
	S2LO	data2, data2, #8
	b	.Lstrcmp_tail

5:
#ifdef __ARM_BIG_ENDIAN
	/* The syndrome value may contain false ones if the string ends
	with the bytes 0x01 0x00.  */
	tst	data1, #0xff000000
	itt	ne
	tstne	data1, #0x00ff0000
	tstne	data1, #0x0000ff00
	beq	.Lstrcmp_done_equal
#else
	bics	syndrome, syndrome, #0xff000000
	bne	.Lstrcmp_done_equal
#endif
	ldrb	data2, [src2]
	S2LO	tmp2, data1, #24
#ifdef __ARM_BIG_ENDIAN
	lsl	data2, data2, #24
#endif
	b	.Lstrcmp_tail

6:
	S2LO	tmp2, data1, #24
	and	data2, data2, #LSB
	b	.Lstrcmp_tail

	/* Critical inner Loop: Block with 2 bytes initial overlap.  */
	.p2align	2
.Loverlap2:
	S2HI	tmp2, data1, #16
	sub	syndrome, data1, #0x01010101
	S2LO	tmp2, tmp2, #16
	bic	syndrome, syndrome, data1
	cmp	tmp2, data2, S2LO #16
	bne	4f
	ands	syndrome, syndrome, #0x80808080
	it	eq
	ldreq	data2, [src2], #4
	bne	5f
	eor	tmp2, tmp2, data1
	cmp	tmp2, data2, S2HI #16
	bne	6f
	ldr	data1, [src1], #4
	b	.Loverlap2

5:
#ifdef __ARM_BIG_ENDIAN
	/* The syndrome value may contain false ones if the string ends
	with the bytes 0x01 0x00 */
	tst	data1, #0xff000000
	it	ne
	tstne	data1, #0x00ff0000
	beq	.Lstrcmp_done_equal
#else
	lsls	syndrome, syndrome, #16
	bne	.Lstrcmp_done_equal
#endif
	ldrh	data2, [src2]
	S2LO	tmp2, data1, #16
#ifdef __ARM_BIG_ENDIAN
	lsl	data2, data2, #16
#endif
	b	.Lstrcmp_tail

6:
	S2HI	data2, data2, #16
	S2LO	tmp2, data1, #16
4:
	S2LO	data2, data2, #16
	b	.Lstrcmp_tail

	/* Critical inner Loop: Block with 1 byte initial overlap.  */
	.p2align	2
.Loverlap1:
	and	tmp2, data1, #LSB
	cmp	tmp2, data2, S2LO #24
	sub	syndrome, data1, #0x01010101
	bic	syndrome, syndrome, data1
	bne	4f
	ands	syndrome, syndrome, #0x80808080
	it	eq
	ldreq	data2, [src2], #4
	bne	5f
	eor	tmp2, tmp2, data1
	cmp	tmp2, data2, S2HI #8
	bne	6f
	ldr	data1, [src1], #4
	b	.Loverlap1
4:
	S2LO	data2, data2, #24
	b	.Lstrcmp_tail
5:
	/* The syndrome value may contain false ones if the string ends
	   with the bytes 0x01 0x00.  */
	tst	data1, #LSB
	beq	.Lstrcmp_done_equal
	ldr	data2, [src2], #4
6:
	S2LO	tmp2, data1, #8
	bic	data2, data2, #MSB
	b	.Lstrcmp_tail
.Lstrcmp_done_equal:
	mov	result, #0
	.cfi_remember_state
	ldmfd	sp!, {r5}
	.cfi_restore 5
	.cfi_adjust_cfa_offset -4
	epilogue push_ip=HAVE_PAC_LEAF

.Lstrcmp_tail:
	.cfi_restore_state
	and	r2, tmp2, #LSB
	and	result, data2, #LSB
	cmp	result, #1
	it	cs
	cmpcs	result, r2
	itt	eq
	S2LOEQ	tmp2, tmp2, #8
	S2LOEQ	data2, data2, #8
	beq	.Lstrcmp_tail
	sub	result, r2, result
	ldmfd	sp!, {r5}
	.cfi_restore 5
	.cfi_adjust_cfa_offset -4
	epilogue push_ip=HAVE_PAC_LEAF
	.cfi_endproc
	.cantunwind
	.fnend
	.size strcmp, . - strcmp
