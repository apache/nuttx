/****************************************************************************
 * arch/xtensa/src/common/xtensa_signal_handler.S
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

  .file   "xtensa_signal_handler.S"

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
 *   This function is the user-space, signal handler trampoline function. It
 *   is called from up_signal_dispatch() in user-mode.
 *
 * Input Parameters:
 *   a2 = sighand
 *     The address user-space signal handling function
 *   a3-a5 = signo, info, and ucontext
 *     Standard arguments to be passed to the signal handling function.
 *
 * Returned Value:
 *   None. This function does not return in the normal sense. It returns
 *   via the SYS_signal_handler_return (see syscall.h)
 *
 ****************************************************************************/

  .text
  .global up_signal_handler
  .type   up_signal_handler, @function
  .align  4

up_signal_handler:
  /* Call the signal handler */

  mov    a6, a3  /* Move signo into callee's a2 */
  mov    a7, a4  /* Move info into callee's a3 */
  mov    a8, a5  /* Move ucontext into callee's a4 */
  callx4 a2      /* Call the signal handler */

  /* Execute the SYS_signal_handler_return syscall (will not return) */

  movi   a2, SYS_signal_handler_return
  movi   a3, XCHAL_SWINT_CALL
  wsr    a3, INTSET
  rsync

  .size up_signal_handler, .-up_signal_handler

#endif /* CONFIG_BUILD_PROTECTED && !__KERNEL__ */
