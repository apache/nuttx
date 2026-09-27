/****************************************************************************
 * arch/risc-v/src/erbium/erbium_head.S
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
#include <arch/arch.h>
#include <arch/irq.h>

#include "chip.h"
#include "riscv_macros.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   Initialize the stack, global pointer and trap vector on hart 0 before
 *   entering C startup. Park other harts with their interrupts disabled.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Does not return.
 *
 ****************************************************************************/

  .section .text.__start, "ax", @progbits
  .global __start
  .type __start, function

__start:
  csrw CSR_MIE, zero
  csrr a0, CSR_MHARTID
  bnez a0, 2f

  /* Only hart 0 owns the NuttX image and its stack. */

  .option push
  .option norelax
  la   gp, __global_pointer$
  .option pop

  riscv_set_inital_sp ERBIUM_IDLESTACK_BASE, SMP_STACK_SIZE, a0
  la   t0, __erbium_trap_vec
  csrw CSR_MTVEC, t0
  j    erbium_start

2:
  wfi
  j    2b

/****************************************************************************
 * Name: __erbium_trap_vec
 *
 * Description:
 *   Transfer control to the common RISC-V exception entry point.
 *
 * Input Parameters:
 *   None. Hardware supplies the trap state in the machine CSRs.
 *
 * Returned Value:
 *   Returns through the common exception handler.
 *
 ****************************************************************************/

  .section .text.exceptions, "ax", @progbits
  .balign 4
  .global __erbium_trap_vec

__erbium_trap_vec:
  j    exception_common
