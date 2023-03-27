/****************************************************************************
 * arch/risc-v/src/litex/litex_shead.S
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

#include <nuttx/irq.h>

#include "chip.h"
#include "litex_memorymap.h"
#include "riscv_internal.h"


/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Imported symbols */

  .extern __trap_vec

  .section .text
  .global __start

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   Supervisor mode start function.
 *
 * Input Parameters:
 *    a0 - hartid
 *
 ****************************************************************************/

__start:
  /* Disable all interrupts in sie */


  csrw sie, zero
  csrw sip, zero


  /* Set the S-mode trap vector */

  la   t0, __trap_vec
  csrw stvec, t0

  /* Clear sscratch */

  csrw sscratch, zero
  csrw scause, zero
  csrw sepc, zero

  /* initialize global pointer, global data */

  
.option push
.option norelax
  la  gp, __global_pointer$

.option pop

  lui  sp, %hi(LITEX_IDLESTACK_TOP)
  addi sp, sp, %lo(LITEX_IDLESTACK_TOP)

  /* Make sure the writes to CSR stick before continuing */

  fence

  /* Set stack pointer and jump to start */

  /*la sp, LITEX_IDLESTACK_TOP*/
  j __litex_start
