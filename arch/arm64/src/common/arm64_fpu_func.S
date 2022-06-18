/****************************************************************************
 * arch/arm64/src/common/arm64_fpu_func.S
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

#include "arm64_macro.inc"
#include "arch/irq.h"
#include "arm64_fatal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "arm64_fpu_func.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

GTEXT(arm64_fpu_save)
SECTION_FUNC(text, arm64_fpu_save)

    stp    q0,  q1,  [x0, #(16 * FPU_REG_Q0)]
    stp    q2,  q3,  [x0, #(16 * FPU_REG_Q2)]
    stp    q4,  q5,  [x0, #(16 * FPU_REG_Q4)]
    stp    q6,  q7,  [x0, #(16 * FPU_REG_Q6)]
    stp    q8,  q9,  [x0, #(16 * FPU_REG_Q8)]
    stp    q10, q11, [x0, #(16 * FPU_REG_Q10)]
    stp    q12, q13, [x0, #(16 * FPU_REG_Q12)]
    stp    q14, q15, [x0, #(16 * FPU_REG_Q14)]
    stp    q16, q17, [x0, #(16 * FPU_REG_Q16)]
    stp    q18, q19, [x0, #(16 * FPU_REG_Q18)]
    stp    q20, q21, [x0, #(16 * FPU_REG_Q20)]
    stp    q22, q23, [x0, #(16 * FPU_REG_Q22)]
    stp    q24, q25, [x0, #(16 * FPU_REG_Q24)]
    stp    q26, q27, [x0, #(16 * FPU_REG_Q26)]
    stp    q28, q29, [x0, #(16 * FPU_REG_Q28)]
    stp    q30, q31, [x0, #(16 * FPU_REG_Q30)]

    mrs    x1, fpsr
    mrs    x2, fpcr
    str    w1, [x0, #(16 * 32 + 0)]
    str    w2, [x0, #(16 * 32 + 4)]

    ret

GTEXT(arm64_fpu_restore)
SECTION_FUNC(text, arm64_fpu_restore)

    ldp    q0,  q1,  [x0, #(16 * FPU_REG_Q0)]
    ldp    q2,  q3,  [x0, #(16 * FPU_REG_Q2)]
    ldp    q4,  q5,  [x0, #(16 * FPU_REG_Q4)]
    ldp    q6,  q7,  [x0, #(16 * FPU_REG_Q6)]
    ldp    q8,  q9,  [x0, #(16 * FPU_REG_Q8)]
    ldp    q10, q11, [x0, #(16 * FPU_REG_Q10)]
    ldp    q12, q13, [x0, #(16 * FPU_REG_Q12)]
    ldp    q14, q15, [x0, #(16 * FPU_REG_Q14)]
    ldp    q16, q17, [x0, #(16 * FPU_REG_Q16)]
    ldp    q18, q19, [x0, #(16 * FPU_REG_Q18)]
    ldp    q20, q21, [x0, #(16 * FPU_REG_Q20)]
    ldp    q22, q23, [x0, #(16 * FPU_REG_Q22)]
    ldp    q24, q25, [x0, #(16 * FPU_REG_Q24)]
    ldp    q26, q27, [x0, #(16 * FPU_REG_Q26)]
    ldp    q28, q29, [x0, #(16 * FPU_REG_Q28)]
    ldp    q30, q31, [x0, #(16 * FPU_REG_Q30)]

    ldr    w1, [x0, #(16 * 32 + 0)]
    ldr    w2, [x0, #(16 * 32 + 4)]
    msr    fpsr, x1
    msr    fpcr, x2

    ret
