/****************************************************************************
 * libs/libm/libm/risc-v/lib_fmaf_riscv.c
 *
 * SPDX-License-Identifier: MIT
 *
 * Adapted from musl 9b2d8a1646391d5217f9a358555aebcaab5b8aaf.
 *
 * Copyright © 2005-2020 Rich Felker, et al.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <math.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Use the hardware instruction directly. Calling __builtin_fmaf from the
 * implementation of fmaf can become a recursive library call when the
 * compiler cannot lower it. fmadd.s rounds once in the current frm mode and
 * sets the FPU exception flags, including cancellation and subnormal cases.
 * The asm is volatile because the compiler does not see that it depends on
 * the rounding mode and sets the flags, and must not cache or reorder it
 * across fesetround() and fetestexcept() when fmaf() is inlined.
 */

float fmaf(float x, float y, float z)
{
  float result;

  __asm__ __volatile__("fmadd.s %0, %1, %2, %3"
                       : "=f"(result) : "f"(x), "f"(y), "f"(z));
  return result;
}
