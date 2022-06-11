/****************************************************************************
 * arch/risc-v/src/common/vfork.S
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

#include "riscv_vfork.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .file  "vfork.S"
  .globl up_vfork
  .globl vfork

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vfork
 *
 * Description:
 *   The vfork() function has the same effect as fork(), except that the
 *   behavior is undefined if the process created by vfork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from vfork(), or returns from the function in which vfork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   This thin layer implements vfork by simply calling up_vfork() with the
 *   vfork() context as an argument.  The overall sequence is:
 *
 *   1) User code calls vfork().  vfork() collects context information and
 *      transfers control up up_vfork().
 *   2) up_vfork() and calls nxtask_setup_vfork().
 *   3) nxtask_setup_vfork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_vfork() then calls nxtask_start_vfork()
 *   6) nxtask_start_vfork() then executes the child thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Upon successful completion, vfork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

.type vfork, function

vfork:
  /* Create a stack frame */

  addi        sp, sp, -VFORK_SIZEOF

  /* CPU registers */
  /* Save the volatile registers */

  REGSTORE    s1, VFORK_S1_OFFSET(sp)
  REGSTORE    s2, VFORK_S2_OFFSET(sp)
  REGSTORE    s3, VFORK_S3_OFFSET(sp)
  REGSTORE    s4, VFORK_S4_OFFSET(sp)
  REGSTORE    s5, VFORK_S5_OFFSET(sp)
  REGSTORE    s6, VFORK_S6_OFFSET(sp)
  REGSTORE    s7, VFORK_S7_OFFSET(sp)
  REGSTORE    s8, VFORK_S8_OFFSET(sp)
  REGSTORE    s9, VFORK_S9_OFFSET(sp)
  REGSTORE    s10, VFORK_S10_OFFSET(sp)
  REGSTORE    s11, VFORK_S11_OFFSET(sp)

  /* Save the frame pointer, stack pointer, and return address */

#ifdef CONFIG_RISCV_FRAMEPOINTER
  REGSTORE    fp, VFORK_FP_OFFSET(sp)
#else
  REGSTORE    s0, VFORK_S0_OFFSET(sp)
#endif

  addi        s0, sp, VFORK_SIZEOF
  REGSTORE    s0, VFORK_SP_OFFSET(sp) /* original SP */
  REGSTORE    x1, VFORK_RA_OFFSET(sp) /* return address */

  /* Floating point registers */

#ifdef CONFIG_ARCH_FPU
  FSTORE      fs0, VFORK_FS0_OFFSET(sp)
  FSTORE      fs1, VFORK_FS1_OFFSET(sp)
  FSTORE      fs2, VFORK_FS2_OFFSET(sp)
  FSTORE      fs3, VFORK_FS3_OFFSET(sp)
  FSTORE      fs4, VFORK_FS4_OFFSET(sp)
  FSTORE      fs5, VFORK_FS5_OFFSET(sp)
  FSTORE      fs6, VFORK_FS6_OFFSET(sp)
  FSTORE      fs7, VFORK_FS7_OFFSET(sp)
  FSTORE      fs8, VFORK_FS8_OFFSET(sp)
  FSTORE      fs9, VFORK_FS9_OFFSET(sp)
  FSTORE      fs10, VFORK_FS10_OFFSET(sp)
  FSTORE      fs11, VFORK_FS11_OFFSET(sp)
#endif

  /* Then, call up_vfork(), passing it a pointer to the stack frame */

  mv          a0, sp
  jal         x1, up_vfork

  /* Release the stack frame and return the value returned by up_vfork */

  REGLOAD     x1, VFORK_RA_OFFSET(sp)
  addi        sp, sp, VFORK_SIZEOF
  ret

  .size  vfork, .-vfork
  .end
