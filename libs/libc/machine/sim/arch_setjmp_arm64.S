/****************************************************************************
 * libs/libc/machine/sim/arch_setjmp_arm64.S
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
#include <arch/setjmp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_HOST_MACOS)
    #define SYMBOL(x) _##x
#else
    #define SYMBOL(x) x
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

    .text
    .globl  SYMBOL(setjmp)
    .align  4

SYMBOL(setjmp):

    stp     x19, x20, [x0, JB_X19_X20]
    stp     x21, x22, [x0, JB_X21_X22]
    stp     x23, x24, [x0, JB_X23_X24]
    stp     x25, x26, [x0, JB_X25_X26]
    stp     x27, x28, [x0, JB_X27_X28]
    stp     x29, lr,  [x0, JB_X29_XLR]
    mov     x1, sp                          /* STP can't access SP */
    stp     fp, x1,   [x0, JB_XFP_XSP]
    stp     d8, d9,   [x0, JB_D08_D09]
    stp     d10, d11, [x0, JB_D10_D11]
    stp     d12, d13, [x0, JB_D12_D13]
    stp     d14, d15, [x0, JB_D14_D15]

    mov     x0,#0                           /* return value */
    ret

    .text
    .globl  SYMBOL(longjmp)
    .align  4

SYMBOL(longjmp):

    ldp     x19, x20, [x0, JB_X19_X20]
    ldp     x21, x22, [x0, JB_X21_X22]
    ldp     x23, x24, [x0, JB_X23_X24]
    ldp     x25, x26, [x0, JB_X25_X26]
    ldp     x27, x28, [x0, JB_X27_X28]
    ldp     x29, lr,  [x0, JB_X29_XLR]
    ldp     fp, x2,   [x0, JB_XFP_XSP]
    ldp     d8, d9,   [x0, JB_D08_D09]
    ldp     d10, d11, [x0, JB_D10_D11]
    ldp     d12, d13, [x0, JB_D12_D13]
    ldp     d14, d15, [x0, JB_D14_D15]
    mov     sp, x2                          /* LDP can't access SP */

    mov     x0, x1
    cmp     x0, #0
    b.ne    1f                              /* return 1, if val is 0 */

    mov     x0,#1                           /* return value */

1:
    ret

    .end
