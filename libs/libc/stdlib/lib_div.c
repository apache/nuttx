/****************************************************************************
 * libs/libc/stdlib/lib_div.c
 *
 *   Copyright (C) 2015 Stavros Polymenis. All rights reserved.
 *   Author: Stavros Polymenis <sp@orbitalfox.com>
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

#include <stdlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: div
 *
 * Description:
 *   The div() function computes the quotient and remainder of the division
 *   of the numerator 'numer' by the denominator 'denom". If the division is
 *   inexact, the resulting quotient is the integer of lesser magnitude that
 *   is the nearest to the algebraic quotient. If the result cannot be
 *   represented, the behavior is undefined; otherwise, quot * denom + rem
 *   will equal 'numer'.
 *
 * Input Parameters:
 *   numer - Numerator of the Division
 *   denom - Denominator of the division
 *
 * Returned Value:
 *   The result of the division represented as values of type div_t
 *
 ****************************************************************************/

div_t div(int numer, int denom)
{
  div_t f;

  f.quot = numer / denom;
  f.rem  = numer % denom;
  return f;
}
