/****************************************************************************
 * arch/risc-v/src/common/fork.S
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

#include "riscv_fork.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .file  "fork.S"
  .globl riscv_fork
  .globl up_fork

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fork
 *
 * Description:
 *   The up_fork() function is the base of fork() function that provided in
 *   libc, and fork() is implemented as a wrapper of up_fork() function.
 *   The fork() function has the same effect as posix fork(), except that the
 *   behavior is undefined if the process created by fork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from fork(), or returns from the function in which fork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   This thin layer implements fork by simply calling up_fork() with the
 *   fork() context as an argument.  The overall sequence is:
 *
 *   1) User code calls fork().  fork() collects context information and
 *      transfers control up up_fork().
 *   2) riscv_fork() and calls nxtask_setup_fork().
 *   3) nxtask_setup_fork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Allocate and initialize the stack
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state())
 *   4) riscv_fork() provides any additional operating context. riscv_fork must:
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) riscv_fork() then calls nxtask_start_fork()
 *   6) nxtask_start_fork() then executes the child thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Upon successful completion, fork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

.type up_fork, function

up_fork:

#ifdef CONFIG_LIB_SYSCALL
  /* When coming via system call, everything is in place already */

  tail        riscv_fork
#else
  /* Create a stack frame */

  addi        sp, sp, -FORK_SIZEOF

  /* CPU registers */
  /* Save the volatile registers */

  REGSTORE    s1, FORK_S1_OFFSET(sp)
  REGSTORE    s2, FORK_S2_OFFSET(sp)
  REGSTORE    s3, FORK_S3_OFFSET(sp)
  REGSTORE    s4, FORK_S4_OFFSET(sp)
  REGSTORE    s5, FORK_S5_OFFSET(sp)
  REGSTORE    s6, FORK_S6_OFFSET(sp)
  REGSTORE    s7, FORK_S7_OFFSET(sp)
  REGSTORE    s8, FORK_S8_OFFSET(sp)
  REGSTORE    s9, FORK_S9_OFFSET(sp)
  REGSTORE    s10, FORK_S10_OFFSET(sp)
  REGSTORE    s11, FORK_S11_OFFSET(sp)

  /* Save the frame pointer, stack pointer, and return address */

#ifdef CONFIG_RISCV_FRAMEPOINTER
  REGSTORE    fp, FORK_FP_OFFSET(sp)
#else
  REGSTORE    s0, FORK_S0_OFFSET(sp)
#endif

#ifdef RISCV_SAVE_GP
  REGSTORE    gp, FORK_GP_OFFSET(sp)
#endif

  addi        a0, sp, FORK_SIZEOF
  REGSTORE    a0, FORK_SP_OFFSET(sp) /* original SP */
  REGSTORE    x1, FORK_RA_OFFSET(sp) /* return address */

  /* Floating point registers */

#ifdef CONFIG_ARCH_FPU
  FSTORE      fs0, FORK_FS0_OFFSET(sp)
  FSTORE      fs1, FORK_FS1_OFFSET(sp)
  FSTORE      fs2, FORK_FS2_OFFSET(sp)
  FSTORE      fs3, FORK_FS3_OFFSET(sp)
  FSTORE      fs4, FORK_FS4_OFFSET(sp)
  FSTORE      fs5, FORK_FS5_OFFSET(sp)
  FSTORE      fs6, FORK_FS6_OFFSET(sp)
  FSTORE      fs7, FORK_FS7_OFFSET(sp)
  FSTORE      fs8, FORK_FS8_OFFSET(sp)
  FSTORE      fs9, FORK_FS9_OFFSET(sp)
  FSTORE      fs10, FORK_FS10_OFFSET(sp)
  FSTORE      fs11, FORK_FS11_OFFSET(sp)
#endif

  /* Then, call riscv_fork(), passing it a pointer to the stack frame */

  mv          a0, sp
  call        riscv_fork

  /* Release the stack frame and return the value returned by riscv_fork */

  REGLOAD     x1, FORK_RA_OFFSET(sp)
  addi        sp, sp, FORK_SIZEOF
  ret
#endif

  .size  up_fork, .-up_fork
  .end
