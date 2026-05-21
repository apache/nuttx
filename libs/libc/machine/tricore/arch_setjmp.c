/****************************************************************************
 * libs/libc/machine/tricore/arch_setjmp.c
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
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/compiler.h>
#include <arch/setjmp.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int setjmp(jmp_buf env)
{
  __asm__ __volatile__ (
#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
    "st.a   [a4]12, a11\n\t"
    "movh.a a11, #@HIS(.Lsaveregs)\n\t"
    "lea    a11, [a11]@LOS(.Lsaveregs)\n\t"
    "ret\n\t"
    ".Lsaveregs:\n\t"
    "st.a   [a4]8, sp\n\t"
    "st.a   [a4]32, a12\n\t"
    "st.a   [a4]36, a13\n\t"
    "st.a   [a4]40, a14\n\t"
    "st.a   [a4]44, a15\n\t"
    "st.w   [a4]16, d8\n\t"
    "st.w   [a4]20, d9\n\t"
    "st.w   [a4]24, d10\n\t"
    "st.w   [a4]28, d11\n\t"
    "st.w   [a4]48, d12\n\t"
    "st.w   [a4]52, d13\n\t"
    "st.w   [a4]56, d14\n\t"
    "st.w   [a4]60, d15\n\t"
    "mfcr   d2, #0xfe00\n\t"
    "st.w   [a4]0, d2\n\t"
    "mfcr   d2, #0xfe04\n\t"
    "st.w   [a4]4, d2\n\t"
    "ld.a   a2, [a4]12\n\t"
    "mov    d2, #0\n\t"
    "ji     a2\n\t"
#else
    "st.a   [%%a4]12, %%a11\n\t"
    "movh.a %%a11, hi:.Lsaveregs\n\t"
    "lea    %%a11, [%%a11] lo:.Lsaveregs\n\t"
    "ret\n\t"
    ".Lsaveregs:\n\t"
    "st.a   [%%a4]8, %%sp\n\t"
    "st.a   [%%a4]32, %%a12\n\t"
    "st.a   [%%a4]36, %%a13\n\t"
    "st.a   [%%a4]40, %%a14\n\t"
    "st.a   [%%a4]44, %%a15\n\t"
    "st.w   [%%a4]16, %%d8\n\t"
    "st.w   [%%a4]20, %%d9\n\t"
    "st.w   [%%a4]24, %%d10\n\t"
    "st.w   [%%a4]28, %%d11\n\t"
    "st.w   [%%a4]48, %%d12\n\t"
    "st.w   [%%a4]52, %%d13\n\t"
    "st.w   [%%a4]56, %%d14\n\t"
    "st.w   [%%a4]60, %%d15\n\t"
    "mfcr   %%d2, 0xfe00\n\t"
    "st.w   [%%a4]0, %%d2\n\t"
    "mfcr   %%d2, 0xfe04\n\t"
    "st.w   [%%a4]4, %%d2\n\t"
    "ld.a   %%a2, [%%a4]12\n\t"
    "mov    %%d2, 0\n\t"
    "ji     %%a2\n\t"
#endif
    ::: "memory"
  );
  return 0;
}

void longjmp(jmp_buf env, int val)
{
  __asm__ __volatile__ (
#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
    "ld.w   d5, [a4]0\n\t"
    ".Lloop:\n\t"
    "mfcr   d2, #0xfe00\n\t"
    "jne    d2, d5, .Lrelease\n\t"
    "max.u  d2, d4, #1\n\t"
    "ld.a   a2, [a4]12\n\t"
    "ld.a   sp, [a4]8\n\t"
    "ld.a   a12, [a4]32\n\t"
    "ld.a   a13, [a4]36\n\t"
    "ld.a   a14, [a4]40\n\t"
    "ld.a   a15, [a4]44\n\t"
    "ld.w   d8, [a4]16\n\t"
    "ld.w   d9, [a4]20\n\t"
    "ld.w   d10, [a4]24\n\t"
    "ld.w   d11, [a4]28\n\t"
    "ld.w   d12, [a4]48\n\t"
    "ld.w   d13, [a4]52\n\t"
    "ld.w   d14, [a4]56\n\t"
    "ld.w   d15, [a4]60\n\t"
    "ji     a2\n\t"
    ".Lrelease:\n\t"
    "jnz.t  d2, #20, .Lupper\n\t"
    "mov.aa a15, a4\n\t"
    "mov    d15, d4\n\t"
    "mov    d14, d5\n\t"
    "rslcx\n\t"
    "mov    d4, d15\n\t"
    "mov    d5, d14\n\t"
    "mov.aa a4, a15\n\t"
    "j      .Lloop\n\t"
    ".Lupper:\n\t"
    "movh.a a11, #@HIS(.Lloop)\n\t"
    "lea    a11, [a11]@LOS(.Lloop)\n\t"
    "ret\n\t"
#else
    "ld.w   %%d5, [%%a4]0\n\t"
    ".Lloop:\n\t"
    "mfcr   %%d2, 0xfe00\n\t"
    "jne    %%d2, %%d5, .Lrelease\n\t"
    "max.u  %%d2, %%d4, 1\n\t"
    "ld.a   %%a2, [%%a4]12\n\t"
    "ld.a   %%sp, [%%a4]8\n\t"
    "ld.a   %%a12, [%%a4]32\n\t"
    "ld.a   %%a13, [%%a4]36\n\t"
    "ld.a   %%a14, [%%a4]40\n\t"
    "ld.a   %%a15, [%%a4]44\n\t"
    "ld.w   %%d8, [%%a4]16\n\t"
    "ld.w   %%d9, [%%a4]20\n\t"
    "ld.w   %%d10, [%%a4]24\n\t"
    "ld.w   %%d11, [%%a4]28\n\t"
    "ld.w   %%d12, [%%a4]48\n\t"
    "ld.w   %%d13, [%%a4]52\n\t"
    "ld.w %%d14, [%%a4]56\n\t"
    "ld.w   %%d15, [%%a4]60\n\t"
    "ji     %%a2\n\t"
    ".Lrelease:\n\t"
    "jnz.t  %%d2, 20, .Lupper\n\t"
    "mov.aa %%a15, %%a4\n\t"
    "mov    %%d15, %%d4\n\t"
    "mov    %%d14, %%d5\n\t"
    "rslcx\n\t"
    "mov    %%d4, %%d15\n\t"
    "mov    %%d5, %%d14\n\t"
    "mov.aa %%a4, %%a15\n\t"
    "j      .Lloop\n\t"
    ".Lupper:\n\t"
    "movh.a %%a11, hi:.Lloop\n\t"
    "lea    %%a11, [%%a11] lo:.Lloop\n\t"
    "ret\n\t"
#endif
    ::: "memory"
  );

  __builtin_unreachable();
}
