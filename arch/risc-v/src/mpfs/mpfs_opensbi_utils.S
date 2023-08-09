/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_opensbi_utils.S
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
  .global mpfs_opensbi_relocate_from_envm

  /* Add some weak default values to make this compile without */

  .weak  _sbi_zerodev_loadaddr
  .weak  _ssbi_zerodev
  .weak  _esbi_zerodev

  /* These are the weak pointless variables, only to get this PR compile */

_sbi_zerodev_loadaddr:
  .dword  0x0a006000
_ssbi_zerodev:
  .dword  _stext
_esbi_zerodev:
  .dword  _stext

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
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

  .align 3
mpfs_opensbi_relocate_from_envm:

  /* Relocate the code from eNVM into L2 zero device */

  la    a0, _sbi_zerodev_loadaddr
  la    a1, _ssbi_zerodev
  la    a2, _esbi_zerodev
.check_if_opensbi_copy_done:
  bge   a1, a2, .opensbi_copy_done
  ld    a3, 0(a0)
  sd    a3, 0(a1)
  addi	a0, a0, 8
  addi	a1, a1, 8
  j     .check_if_opensbi_copy_done
.opensbi_copy_done:

  /* To make a store to instruction memory visible to all RISC-V harts, the
   * writing hart has to execute a data FENCE before requesting that all
   * remote RISC-V harts execute a FENCE.I. This is hart0 only.
   */

  fence
  ret

  .align 3
mpfs_opensbi_prepare_hart:

  /* These are harts other than 0. Thus, fence.i */

  fence.i

  /* Setup OpenSBI exception handler */

  la   t0, mpfs_exception_opensbi
  csrw mtvec, t0

  /* Setup stacks per hart, the stack top is the end of the hart's scratch */

  csrr a0, mhartid
  li   t1, SBI_SCRATCH_SIZE
  mul  t0, a0, t1
  la   sp, g_scratches
  add  sp, sp, t0
  tail mpfs_opensbi_setup
