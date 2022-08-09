/****************************************************************************
 * arch/arm64/src/common/arm64_smccc.S
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

#include "arch/syscall.h"
#include "arm64_macro.inc"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "arm64_smccc.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* The SMC instruction is used to generate a synchronous exception that is
 * handled by Secure Monitor code running in EL3.
 */

GTEXT(arm64_smccc_smc)
SECTION_FUNC(text, arm64_smccc_smc)
    smc   #0
    ldr   x4, [sp]
    stp   x0, x1, [x4, #8 * ARM_SMCC_RES_A0]
    stp   x2, x3, [x4, #8 * ARM_SMCC_RES_A2]
    stp   x4, x5, [x4, #8 * ARM_SMCC_RES_A4]
    stp   x6, x7, [x4, #8 * ARM_SMCC_RES_A6]
    ret

/* The HVC instruction is used to generate a synchronous exception that is
 * handled by a hypervisor running in EL2.
 */

GTEXT(arm64_smccc_hvc)
SECTION_FUNC(text, arm64_smccc_hvc)
    hvc   #0
    ldr   x4, [sp]
    stp   x0, x1, [x4, #8 * ARM_SMCC_RES_A0]
    stp   x2, x3, [x4, #8 * ARM_SMCC_RES_A2]
    stp   x4, x5, [x4, #8 * ARM_SMCC_RES_A4]
    stp   x6, x7, [x4, #8 * ARM_SMCC_RES_A6]
    ret
