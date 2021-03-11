/****************************************************************************
 * arch/xtensa/src/common/xtensa_backtrace.S
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

#include <arch/xtensa/core.h>
#include <arch/xtensa/xtensa_specregs.h>

#include "xtensa_abi.h"

/*
 * This is how the call stack looks like when calling the function below
 * from xtensa_dumpstate():
 *
 *         High Addr
 *     ..................
 *     |    i-3 BSA     |
 *     |   i-1 locals   | xtensa_assert()
 *     .................. i-1 SP
 *     |    i-2 BSA     |
 *     |    i locals    | xtensa_dumpstate()
 *     ------------------ i SP
 *     |    i-1 BSA     |
 *     |   i+1 locals   | xtensa_btdump()
 *     ------------------ i+1 SP
 *     |     i BSA      |
 *     |   i+2 locals   | xtensa_backtrace_start()
 *     ------------------ i+2 SP
 *     |    i+1 BSA     |
 *     |   i+3 locals   | xtensa_window_spill()
 *     ------------------ i+3 SP
 *     .................. Low Addr
 */

/****************************************************************************
 * Name: xtensa_backtrace_start
 *
 * Description:
 *   Get the first frame of the current stack's backtrace.
 *
 *   Given the following function call flow (B -> A -> X -> xtensa_backtrace_start),
 *   this function will do the following:
 *     - Flush CPU registers and window frames onto the current stack
 *     - Return PC and SP of function A (i.e. start of the stack's backtrace)
 *     - Return PC of function B (i.e. next_pc)
 *
 * Input Parameters:
 *   pc      - PC of the first frame in the backtrace
 *   sp      - SP of the first frame in the backtrace
 *   next_pc - PC of the first frame's caller
 *
 *   C callable as:
 *     void xtensa_backtrace_start(uint32_t *pc, uint32_t *sp, uint32_t *next_pc)
 *
 ****************************************************************************/

    .section    .iram1, "ax"
    .global     xtensa_backtrace_start
    .type       xtensa_backtrace_start, @function
    .align      4

xtensa_backtrace_start:

  ENTRY(32)

  call8   xtensa_window_spill  /* Spill registers onto stack (excluding this
                                * function) */

  /* a2, a3, a4 should be out arguments for i SP, i PC, i-1 PC respectively.
   * Use a5 and a6 as scratch.
   */

  l32e    a5, sp, -16     /* Get i PC, which is ret address of i+1 */
  s32i    a5, a2, 0       /* Store i PC to arg *pc */
  l32e    a6, sp, -12     /* Get i+1 SP. Used to access i BS */
  l32e    a5, a6, -12     /* Get i SP */
  s32i    a5, a3, 0       /* Store i SP to arg *sp */
  l32e    a5, a6, -16     /* Get i-1 PC, which is ret address of i */
  s32i    a5, a4, 0       /* Store i-1 PC to arg *next_pc */
  RET0

	.size	xtensa_backtrace_start, . - xtensa_backtrace_start

