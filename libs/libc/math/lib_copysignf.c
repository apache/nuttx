/****************************************************************************
 * libs/libc/math/lib_copysignf.c
 *
 *   Copyright (C) 2015, 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Dave Marples <dave@marples.net>
 *
 * Replaced on 2016-07-30 by David Alession with a faster version of
 * copysignf() from NetBSD with the following Copyright:
 *
 *   Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 *   Developed at SunPro, a Sun Microsystems, Inc. business.
 *   Permission to use, copy, modify, and distribute this
 *   software is freely granted, provided that this notice
 *   is preserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <math.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32 bit int from a float.  */

#define GET_FLOAT_WORD(i,d) \
  do \
    { \
      ieee_float_shape_type gf_u; \
      gf_u.value = (d); \
      (i) = gf_u.word; \
    } while (0)

/* Set a float from a 32 bit int.  */

#define SET_FLOAT_WORD(d,i) \
  do \
    { \
      ieee_float_shape_type sf_u; \
      sf_u.word = (i); \
      (d) = sf_u.value; \
    } while (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* union which permits us to convert between a float and a 32 bit int.  */

typedef union
{
  float   value;
  uint32_t word;
}
ieee_float_shape_type;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

float copysignf(float x, float y)
{
  uint32_t ix;
  uint32_t iy;

  GET_FLOAT_WORD(ix, x);
  GET_FLOAT_WORD(iy, y);
  SET_FLOAT_WORD(x, (ix & 0x7fffffff) | (iy & 0x80000000));

  return x;
}
