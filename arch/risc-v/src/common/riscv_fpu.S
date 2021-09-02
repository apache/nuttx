/************************************************************************************
 * arch/risc-v/src/common/riscv_fpu.S
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/arch.h>
#include <arch/csr.h>
#include <arch/irq.h>
#include <arch/mode.h>

#ifdef CONFIG_ARCH_FPU

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

    .globl        riscv_fpuconfig
    .file         "riscv_fpu.S"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: riscv_fpuconfig
 *
 * Description:
 *   init fpu
 *
 * C Function Prototype:
 *   void riscv_fpuconfig(void);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   This function does not return anything explicitly.
 *
 ************************************************************************************/

    .type        riscv_fpuconfig, function

riscv_fpuconfig:
    li           a0, MSTATUS_FS_INIT
    csrs         CSR_STATUS, a0

    fsflags      zero
    fsrm         zero

    fence.i
    ret

#endif /* CONFIG_ARCH_FPU */
