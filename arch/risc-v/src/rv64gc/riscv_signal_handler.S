/****************************************************************************
 * arch/risc-v/src/rv64gc/up_signal_handler.S
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

#include <arch/syscall.h>

#if defined(CONFIG_BUILD_PROTECTED) && !defined(__KERNEL__)

/****************************************************************************
 * File info
 ****************************************************************************/

  .file   "up_signal_handler.S"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_signal_handler
 *
 * Description:
 *   This function is the user-space, signal handler trampoline function.  It
 *   is called from up_signal_dispatch() in user-mode.
 *
 * Input Parameters:
 *   a0 = sighand
 *     The address user-space signal handling function
 *   a1-a3 = signo, info, and ucontext
 *     Standard arguments to be passed to the signal handling function.
 *
 * Returned Value:
 *   None.  This function does not return in the normal sense.  It returns
 *   via the SYS_signal_handler_return (see svcall.h)
 *
 ****************************************************************************/

  .text
  .globl up_signal_handler
  .type  up_signal_handler, function

up_signal_handler:

  /* Save ra on the stack */

  addi sp, sp, -8
  sd   ra, (sp)

  /* Call the signal handler */

  mv   t0, a0  /* t0=sighand */
  mv   a0, a1  /* a0=signo */
  mv   a1, a2  /* a1=info */
  mv   a2, a3  /* a2=ucontext */
  jalr t0      /* Call the signal handler (modifies ra) */

  /* Restore the register */

  ld   ra, (sp)  /* Restore ra in sp */
  addi sp, sp, 8

  /* Execute the SYS_signal_handler_return SVCall (will not return) */

  li   a0, SYS_signal_handler_return
  ecall
  nop

  .size up_signal_handler, .-up_signal_handler
  .end

#endif /* CONFIG_BUILD_PROTECTED && !__KERNEL__ */
