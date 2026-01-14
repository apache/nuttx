/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_opensbi_utils.S
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

#include <sbi/riscv_asm.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_trap.h>
#include <sbi/riscv_encoding.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .global mpfs_opensbi_prepare_hart

/****************************************************************************
 * Private Data
 ****************************************************************************/

mpfs_global_pointer:
  .dword __global_pointer$

/****************************************************************************
 * Name: mpfs_opensbi_prepare_hart
 *
 * Description:
 *   Prepares the hart for OpenSBI execution.  This installs the proper
 *   mtvec, global pointer and the stack (per hart) for the OpenSBI.
 *   mpfs_global_pointer is used to store the real __global_pointer$ as
 *   seen in the .map file.  Loading gp, _global_pointer$ would default to
 *   mv gp, gp -instruction which isn't what we want. External libraries seem
 *   to link relative to gp. When trapping from the kernel, the gp has been
 *   utilized for other purposes, so we need to save and restore gp at all
 *   times.
 *
 * Input Parameters:
 *   a0 - hartid
 *   a1 - next_addr
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

  .align 3
mpfs_opensbi_prepare_hart:

  /* Setup OpenSBI exception handler */

  la   t0, mpfs_exception_opensbi
  csrw CSR_MTVEC, t0

  /* la gp, __global_pointer$ will not work. We want to have the gp as seen
   * in the .map file exactly. We need to restore gp in the trap handler.
   */

  la   t0, mpfs_global_pointer
  ld   gp, 0(t0)

  /* Setup stacks per hart, the stack top is the end of the hart's scratch */

  csrr t0, CSR_MHARTID
  li   t1, SBI_SCRATCH_SIZE
  mul  t0, t0, t1
  la   sp, g_scratches
  add  sp, sp, t0
  tail mpfs_opensbi_setup
