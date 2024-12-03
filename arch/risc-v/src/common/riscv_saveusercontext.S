/****************************************************************************
 * arch/risc-v/src/common/riscv_saveusercontext.S
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

#include "riscv_macros.S"

#ifdef CONFIG_ARCH_RISCV_INTXCPT_EXTENSIONS
#  include "riscv_extctx.S"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

/****************************************************************************
 * Name: up_saveusercontext
 *
 * Description:
 *   Save the current thread context
 *
 ****************************************************************************/

  .section .text
  .global up_saveusercontext
  .align  8

up_saveusercontext:

  save_ctx   a0

#ifdef CONFIG_ARCH_RISCV_INTXCPT_EXTENSIONS
  save_extctx a0
#endif

  csrr       a1, CSR_STATUS
  REGSTORE   a1, REG_INT_CTX(a0)  /* status */

  REGSTORE   sp, REG_X2(a0)       /* original SP */
  REGSTORE   x1, REG_EPC(a0)

#ifdef CONFIG_ARCH_FPU
  addi       a0, a0, INT_XCPT_SIZE /* Save FPU after integer regs */
  riscv_savefpu a0
#endif

  li         a0, 0
  jr         ra

  .size	up_saveusercontext, . - up_saveusercontext
  .end
