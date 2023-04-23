/****************************************************************************
 * arch/arm64/src/common/macro.inc
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
 ****************************************************************************
 *    DESCRIPTION
 *       macro define for arm64 assembler
 *
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_MACRO_INC
#define __ARCH_ARM64_SRC_COMMON_ARM64_MACRO_INC

.macro  switch_el, xreg, el3_label, el2_label, el1_label
    mrs    \xreg, CurrentEL
    cmp    \xreg, 0xc
    beq    \el3_label
    cmp    \xreg, 0x8
    beq    \el2_label
    cmp    \xreg, 0x4
    beq    \el1_label
.endm

/*
 * macro to support mov of immediate constant to 64 bit register
 * It will generate instruction sequence of 'mov'/ 'movz' and one
 * to three 'movk' depending on the immediate value.
 */
.macro  mov_imm, xreg, imm
    .if ((\imm) == 0)
        mov    \xreg, \imm
    .else
        .if (((\imm) >> 31) == 0 || ((\imm) >> 31) == 0x1ffffffff)
            movz    \xreg, (\imm >> 16) & 0xffff, lsl 16
        .else
            .if (((\imm) >> 47) == 0 || ((\imm) >> 47) == 0x1ffff)
                movz    \xreg, (\imm >> 32) & 0xffff, lsl 32
            .else
                movz    \xreg, (\imm >> 48) & 0xffff, lsl 48
                movk    \xreg, (\imm >> 32) & 0xffff, lsl 32
            .endif
            movk    \xreg, (\imm >> 16) & 0xffff, lsl 16
        .endif
        movk    \xreg, (\imm) & 0xffff, lsl 0
    .endif
.endm

#define GTEXT(sym) .global sym; .type sym, %function
#define PERFOPT_ALIGN .balign  4

#define SECTION_FUNC(sect, sym)  \
  .section .sect.sym, "ax";      \
    PERFOPT_ALIGN; sym :

#define SECTION_SUBSEC_FUNC(sect, subsec, sym)   \
  .section .sect.subsec, "ax"; PERFOPT_ALIGN; sym :

#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_MACRO_INC */
