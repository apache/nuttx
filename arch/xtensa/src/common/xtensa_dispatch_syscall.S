/****************************************************************************
 * arch/xtensa/src/common/xtensa_dispatch_syscall.S
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
#include <arch/xtensa/xtensa_abi.h>

#ifdef CONFIG_LIB_SYSCALL

/****************************************************************************
 * File info
 ****************************************************************************/

  .file   "xtensa_dispatch_syscall.S"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_dispatch_syscall
 *
 * Description:
 *   Call the stub function corresponding to the system call. NOTE the non-
 *   standard parameter passing:
 *
 *     A2 = SYS_ call number
 *     A3 = parm0
 *     A4 = parm1
 *     A5 = parm2
 *     A6 = parm3
 *     A7 = parm4
 *     A8 = parm5
 *
 ****************************************************************************/

  .text
  .global xtensa_dispatch_syscall
  .type   xtensa_dispatch_syscall, @function
  .align  4

xtensa_dispatch_syscall:
  /* Allocate parm5 in stack */

  s32i   a8, sp, LOCAL_OFFSET(0)

  mov    a11, a7 /* Move parm4 into callee's a7 */
  mov    a10, a6 /* Move parm3 into callee's a6 */
  mov    a9, a5  /* Move parm2 into callee's a5 */
  mov    a8, a4  /* Move parm1 into callee's a4 */
  mov    a7, a3  /* Move parm0 into callee's a3 */
  mov    a6, a2  /* Move SYS_ call number into callee's a2 */

  /* Load the stub address into A3 */

  movi   a3, g_stublookup
  slli   a2, a2, 2
  add    a3, a3, a2
  l32i   a3, a3, 0

  /* Call the stub */

  callx4 a3

  /* Move into A3 the return value from the stub */

  mov    a3, a6

  /* Execute the SYS_signal_handler_return syscall (will not return) */

  movi   a2, SYS_syscall_return
  movi   a4, XCHAL_SWINT_CALL
  wsr    a4, INTSET
  rsync

  .size xtensa_dispatch_syscall, .-xtensa_dispatch_syscall

#endif /* CONFIG_LIB_SYSCALL */
