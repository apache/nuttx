/****************************************************************************
 * libs/libm/libm/risc-v/lib_fmaf_riscv.c
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
