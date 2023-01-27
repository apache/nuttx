/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_opensbi_utils.S
 *
 * mpfs_exception_opensbi function is based on OpenSBI fw_base.S
 *
 * The 2-Clause BSD License
 * SPDX short identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates and other
 * contributors.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
  .global mpfs_exception_opensbi

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
mpfs_opensbi_prepare_hart:

  /* Setup OpenSBI exception handler */

  la   t0, mpfs_exception_opensbi
  csrw mtvec, t0

  /* la gp, __global_pointer$ will not work. We want to have the gp as seen
   * in the .map file exactly. We need to restore gp in the trap handler.
   */

  la   t0, mpfs_global_pointer
  ld   gp, 0(t0)

  /* Setup stacks per hart, the stack top is the end of the hart's scratch */

  csrr a0, mhartid
  li   t1, SBI_SCRATCH_SIZE
  mul  t0, a0, t1
  la   sp, g_scratches
  add  sp, sp, t0
  jal  mpfs_opensbi_setup

/****************************************************************************
 * Name: mpfs_exception_opensbi:
 *
 * Description:
 *   This is the trap entry into OpenSBI.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

  .align 3
mpfs_exception_opensbi:

  /* Swap TP and MSCRATCH */

  csrrw   tp, CSR_MSCRATCH, tp

  /* Save T0 in scratch space */

  REG_S   t0, SBI_SCRATCH_TMP0_OFFSET(tp)

  /*
   * Set T0 to appropriate exception stack
   *
   * Came_From_M_Mode = ((MSTATUS.MPP < PRV_M) ? 1 : 0) - 1;
   * Exception_Stack = TP ^ (Came_From_M_Mode & (SP ^ TP))
   *
   * Came_From_M_Mode = 0    ==>    Exception_Stack = TP
   * Came_From_M_Mode = -1   ==>    Exception_Stack = SP
   */

  csrr    t0, CSR_MSTATUS
  srl     t0, t0, MSTATUS_MPP_SHIFT
  and     t0, t0, PRV_M
  slti    t0, t0, PRV_M
  add     t0, t0, -1
  xor     sp, sp, tp
  and     t0, t0, sp
  xor     sp, sp, tp
  xor     t0, tp, t0

  /* Save original SP on exception stack */

  REG_S   sp, (SBI_TRAP_REGS_OFFSET(sp) - SBI_TRAP_REGS_SIZE)(t0)

  /* Set SP to exception stack and make room for trap registers */

  add     sp, t0, -(SBI_TRAP_REGS_SIZE)

  /* Restore T0 from scratch space */

  REG_L   t0, SBI_SCRATCH_TMP0_OFFSET(tp)

  /* Save T0 on stack */

  REG_S   t0, SBI_TRAP_REGS_OFFSET(t0)(sp)

  /* Swap TP and MSCRATCH */

  csrrw   tp, CSR_MSCRATCH, tp

  /* Save MEPC and MSTATUS CSRs */

  csrr    t0, CSR_MEPC
  REG_S   t0, SBI_TRAP_REGS_OFFSET(mepc)(sp)
  csrr    t0, CSR_MSTATUS
  REG_S   t0, SBI_TRAP_REGS_OFFSET(mstatus)(sp)
  REG_S   zero, SBI_TRAP_REGS_OFFSET(mstatusH)(sp)

  /* Save all general regisers except SP and T0 */

  REG_S   zero, SBI_TRAP_REGS_OFFSET(zero)(sp)
  REG_S   ra, SBI_TRAP_REGS_OFFSET(ra)(sp)
  REG_S   gp, SBI_TRAP_REGS_OFFSET(gp)(sp)
  REG_S   tp, SBI_TRAP_REGS_OFFSET(tp)(sp)
  REG_S   t1, SBI_TRAP_REGS_OFFSET(t1)(sp)
  REG_S   t2, SBI_TRAP_REGS_OFFSET(t2)(sp)
  REG_S   s0, SBI_TRAP_REGS_OFFSET(s0)(sp)
  REG_S   s1, SBI_TRAP_REGS_OFFSET(s1)(sp)
  REG_S   a0, SBI_TRAP_REGS_OFFSET(a0)(sp)
  REG_S   a1, SBI_TRAP_REGS_OFFSET(a1)(sp)
  REG_S   a2, SBI_TRAP_REGS_OFFSET(a2)(sp)
  REG_S   a3, SBI_TRAP_REGS_OFFSET(a3)(sp)
  REG_S   a4, SBI_TRAP_REGS_OFFSET(a4)(sp)
  REG_S   a5, SBI_TRAP_REGS_OFFSET(a5)(sp)
  REG_S   a6, SBI_TRAP_REGS_OFFSET(a6)(sp)
  REG_S   a7, SBI_TRAP_REGS_OFFSET(a7)(sp)
  REG_S   s2, SBI_TRAP_REGS_OFFSET(s2)(sp)
  REG_S   s3, SBI_TRAP_REGS_OFFSET(s3)(sp)
  REG_S   s4, SBI_TRAP_REGS_OFFSET(s4)(sp)
  REG_S   s5, SBI_TRAP_REGS_OFFSET(s5)(sp)
  REG_S   s6, SBI_TRAP_REGS_OFFSET(s6)(sp)
  REG_S   s7, SBI_TRAP_REGS_OFFSET(s7)(sp)
  REG_S   s8, SBI_TRAP_REGS_OFFSET(s8)(sp)
  REG_S   s9, SBI_TRAP_REGS_OFFSET(s9)(sp)
  REG_S   s10, SBI_TRAP_REGS_OFFSET(s10)(sp)
  REG_S   s11, SBI_TRAP_REGS_OFFSET(s11)(sp)
  REG_S   t3, SBI_TRAP_REGS_OFFSET(t3)(sp)
  REG_S   t4, SBI_TRAP_REGS_OFFSET(t4)(sp)
  REG_S   t5, SBI_TRAP_REGS_OFFSET(t5)(sp)
  REG_S   t6, SBI_TRAP_REGS_OFFSET(t6)(sp)

  /*  Restore GP */

  la      a0, mpfs_global_pointer
  ld      gp, 0(a0)

  /* Call C routine */

  add     a0, sp, zero
  call    sbi_trap_handler

  /* Restore all general regisers except A0 and T0 */

  REG_L   ra, SBI_TRAP_REGS_OFFSET(ra)(a0)
  REG_L   sp, SBI_TRAP_REGS_OFFSET(sp)(a0)
  REG_L   gp, SBI_TRAP_REGS_OFFSET(gp)(a0)
  REG_L   tp, SBI_TRAP_REGS_OFFSET(tp)(a0)
  REG_L   t1, SBI_TRAP_REGS_OFFSET(t1)(a0)
  REG_L   t2, SBI_TRAP_REGS_OFFSET(t2)(a0)
  REG_L   s0, SBI_TRAP_REGS_OFFSET(s0)(a0)
  REG_L   s1, SBI_TRAP_REGS_OFFSET(s1)(a0)
  REG_L   a1, SBI_TRAP_REGS_OFFSET(a1)(a0)
  REG_L   a2, SBI_TRAP_REGS_OFFSET(a2)(a0)
  REG_L   a3, SBI_TRAP_REGS_OFFSET(a3)(a0)
  REG_L   a4, SBI_TRAP_REGS_OFFSET(a4)(a0)
  REG_L   a5, SBI_TRAP_REGS_OFFSET(a5)(a0)
  REG_L   a6, SBI_TRAP_REGS_OFFSET(a6)(a0)
  REG_L   a7, SBI_TRAP_REGS_OFFSET(a7)(a0)
  REG_L   s2, SBI_TRAP_REGS_OFFSET(s2)(a0)
  REG_L   s3, SBI_TRAP_REGS_OFFSET(s3)(a0)
  REG_L   s4, SBI_TRAP_REGS_OFFSET(s4)(a0)
  REG_L   s5, SBI_TRAP_REGS_OFFSET(s5)(a0)
  REG_L   s6, SBI_TRAP_REGS_OFFSET(s6)(a0)
  REG_L   s7, SBI_TRAP_REGS_OFFSET(s7)(a0)
  REG_L   s8, SBI_TRAP_REGS_OFFSET(s8)(a0)
  REG_L   s9, SBI_TRAP_REGS_OFFSET(s9)(a0)
  REG_L   s10, SBI_TRAP_REGS_OFFSET(s10)(a0)
  REG_L   s11, SBI_TRAP_REGS_OFFSET(s11)(a0)
  REG_L   t3, SBI_TRAP_REGS_OFFSET(t3)(a0)
  REG_L   t4, SBI_TRAP_REGS_OFFSET(t4)(a0)
  REG_L   t5, SBI_TRAP_REGS_OFFSET(t5)(a0)
  REG_L   t6, SBI_TRAP_REGS_OFFSET(t6)(a0)

  /* Restore MEPC and MSTATUS CSRs */

  REG_L   t0, SBI_TRAP_REGS_OFFSET(mepc)(a0)
  csrw    CSR_MEPC, t0
  REG_L   t0, SBI_TRAP_REGS_OFFSET(mstatus)(a0)
  csrw    CSR_MSTATUS, t0

  /* Restore T0 */

  REG_L   t0, SBI_TRAP_REGS_OFFSET(t0)(a0)

  /* Restore A0 */

  REG_L   a0, SBI_TRAP_REGS_OFFSET(a0)(a0)

  mret
