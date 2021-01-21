/****************************************************************************
 * arch/risc-v/include/rv32im/mcause.h
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

#ifndef __ARCH_RISCV_INCLUDE_RV32IM_MCAUSE_H
#define __ARCH_RISCV_INCLUDE_RV32IM_MCAUSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt(BIT31) or Exception(0) */

#define MCAUSE_INTERRUPT           (1 << 31)
#define MCAUSE_INTERRUPT_MASK      (~MCAUSE_INTERRUPT)

/* Exception values *********************************************************/

#define MCAUSE_ADDE_MISALIGNED     (0)  /* Instruction address misaligned */
#define MCAUSE_INST_ACCESS_FAULT   (1)  /* Instruction access fault */
#define MCAUSE_ILLEGAL_INST        (2)  /* Illegal instruction */
#define MCAUSE_BREAKPOINT          (3)  /* Breakpoint */
#define MCAUSE_LOAD_MISALIGNED     (4)  /* Load address misaligned */
#define MCAUSE_LOAD_ACCESS_FAULT   (5)  /* Load access fault */
#define MCAUSE_STORE_MISALIGNED    (6)  /* Store/AMO address misaligned */
#define MCAUSE_STORE_ACCESS_FAULT  (7)  /* Store/AMO access fault */
#define MCAUSE_ECALL_U             (8)  /* Environment call from U-mode */
#define MCAUSE_ECALL_S             (9)  /* Environment call from S-mode */
#define MCAUSE_RESERVED            (10) /* Reserved */
#define MCAUSE_ECALL_M             (11) /* Environment call from M-mode */
#define MCAUSE_INST_PAGE_FAULT     (12) /* Instruction page fault */
#define MCAUSE_LOAD_PAGE_FAULT     (13) /* Load page fault */
#define MCAUSE_RESERVED            (14) /* Reserved */
#define MCAUSE_STORE_PAGE_FAULT    (15) /* Store/AMO page fault */

/* Max RISC-V defined mcause exception values. */

#define MCAUSE_MAX_EXCEPTION       (15)

#endif /* __ARCH_RISCV_INCLUDE_RV32IM_MCAUSE_H */

