/****************************************************************************
 * libs/libc/math/lib_erff.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Brennan Ashton. All rights reserved.
 *   Author: Brennan Ashton <bashton@brennanashton.com>
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  A1     0.254829592F
#define  A2   (-0.284496736F)
#define  A3     1.421413741F
#define  A4   (-1.453152027F)
#define  A5     1.061405429F
#define  P      0.3275911F

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: erff
 *
 * Description:
 *   This implementation comes from the Handbook of Mathmatical Functions
 *   The implementations in this book are not protected by copyright.
 *   erf comes from formula 7.1.26
 *
 ****************************************************************************/

float erff(float x)
{
  float t;
  float z;

  z = fabsf(x);
  t = 1.0F / (1.0F + P * z);
  t = 1.0F - (((((A5 * t + A4) * t) + A3) * t + A2) * t + A1) * t * expf(-z * z);
  return copysignf(t, x);
}
