/****************************************************************************
 * libs/libc/math/lib_erf.c
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

#ifdef CONFIG_HAVE_DOUBLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  A1     0.254829592
#define  A2   (-0.284496736)
#define  A3     1.421413741
#define  A4   (-1.453152027)
#define  A5     1.061405429
#define  P      0.3275911

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: erf
 *
 * Description:
 *   This implementation comes from the Handbook of Mathmatical Functions
 *   The implementations in this book are not protected by copyright.
 *   erf comes from formula 7.1.26
 *
 ****************************************************************************/

double erf(double x)
{
  double t;
  double z;

  z = fabs(x);
  t = 1.0 / (1.0 + P * z);
  t = 1.0 - (((((A5 * t + A4) * t) + A3) * t + A2) * t + A1) * t * exp(-z * z);
  return copysign(t, x);
}

#endif /* CONFIG_HAVE_DOUBLE */
