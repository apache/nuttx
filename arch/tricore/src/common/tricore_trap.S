/****************************************************************************
 * arch/tricore/src/common/tricore_trap.S
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

#include <arch/arch.h>

        .section .traptab, "ax", %progbits
        .global btv_vector
        .type   btv_vector, %object
        .p2align 8

        .macro TRAP_SLOT I, handler
1:
        mov     %d4, \I
        mov     %d5, %d15
        movh.a  %a2, hi:\handler
        lea     %a2, [%a2]lo:\handler
        ji      %a2
        .space  (32 - (.-1b))
        .endm

btv_vector:
        TRAP_SLOT 0, tricore_trap_dispatch
        TRAP_SLOT 1, tricore_trap_dispatch
        TRAP_SLOT 2, tricore_trap_dispatch
        TRAP_SLOT 3, tricore_trap_dispatch
        TRAP_SLOT 4, tricore_trap_dispatch
        TRAP_SLOT 5, tricore_trap_dispatch
        TRAP_SLOT 6, tricore_svcall_local
        TRAP_SLOT 7, tricore_trap_dispatch

        .text
        .p2align 2

        .global tricore_trap_dispatch
        .type   tricore_trap_dispatch, %function
tricore_trap_dispatch:
        svlcx
        call    tricore_trap_handler
        rslcx
        rfe

        .global tricore_svcall_local
        .type   tricore_svcall_local, %function
tricore_svcall_local:
        svlcx
        call    tricore_svcall
        rslcx
        rfe
