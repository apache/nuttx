/****************************************************************************
 * arch/arm/src/armv7-a/crt0.c
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

#include <sys/types.h>
#include <stdlib.h>

#include <nuttx/addrenv.h>

#include <arch/syscall.h>

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int main(int argc, char *argv[]);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_signal_handler
 *
 * Description:
 *   This function is the user-space, signal handler trampoline function.  It
 *   is called from up_signal_dispatch() in user-mode.
 *
 *     R0-R3, R11 - volatile registers need not be preserved.
 *     R4-R10 - static registers must be preserved
 *     R12-R14 - LR and SP must be preserved
 *
 * Input Parameters:
 *   R0 = sighand
 *     The address user-space signal handling function
 *   R1-R3 = signo, info, and ucontext
 *     Standard arguments to be passed to the signal handling function.
 *
 * Returned Value:
 *   None.  This function does not return in the normal sense.  It returns
 *   via the SYS_signal_handler_return (see syscall.h)
 *
 ****************************************************************************/

static void sig_trampoline(void) naked_function;
static void sig_trampoline(void)
{
  __asm__ __volatile__
  (
    " push {lr}\n"       /* Save LR on the stack */
    " mov  ip, r0\n"     /* IP=sighand */
    " mov  r0, r1\n"     /* R0=signo */
    " mov  r1, r2\n"     /* R1=info */
    " mov  r2, r3\n"     /* R2=ucontext */
    " blx  ip\n"         /* Call the signal handler */
    " pop  {r2}\n"       /* Recover LR in R2 */
    " mov  lr, r2\n"     /* Restore LR */
    " mov  r0, %0\n"     /* SYS_signal_handler_return */
    " svc %1\n"          /* Return from the SYSCALL */
    ::"i"(SYS_signal_handler_return),
      "i"(SYS_syscall)
  );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _start
 *
 * Description:
 *   This function is the low level entry point into the main thread of
 *   execution of a task.  It receives initial control when the task is
 *   started and calls main entry point of the newly started task.
 *
 * Input Parameters:
 *   argc - The number of parameters being passed.
 *   argv - The parameters being passed. These lie in kernel-space memory
 *     and will have to be reallocated  in user-space memory.
 *
 * Returned Value:
 *   This function should not return.  It should call the user-mode start-up
 *   main() function.  If that function returns, this function will call
 *   exit.
 *
 ****************************************************************************/

void _start(int argc, FAR char *argv[])
{
  int ret;

  /* Initialize the reserved area at the beginning of the .bss/.data region
   * that is visible to the RTOS.
   */

  ARCH_DATA_RESERVE->ar_sigtramp = (addrenv_sigtramp_t)sig_trampoline;

  /* Call C++ constructors */

  /* Setup so that C++ destructors called on task exit */

  /* REVISIT: Missing logic */

  /* Call the main() entry point passing argc and argv. */

  ret = main(argc, argv);

  /* Call exit() if/when the main() returns */

  exit(ret);
}

#endif /* CONFIG_BUILD_KERNEL */
