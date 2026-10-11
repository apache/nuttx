/****************************************************************************
 * libs/libc/machine/arm64/arch_makecontext.c
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

#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <ucontext.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* In arch_ucontext.S, and called from there */

void arm64_ucontext_start(void);
int arm64_ucontext_getmask(ucontext_t *ucp);
void arm64_ucontext_setmask(const ucontext_t *ucp);
void arm64_ucontext_swapmask(ucontext_t *oucp,
                             const ucontext_t *ucp);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm64_ucontext_getmask, arm64_ucontext_setmask and
 *       arm64_ucontext_swapmask
 *
 * Description:
 *   Save and restore the signal mask of a context, for getcontext(),
 *   setcontext() and swapcontext().
 *
 ****************************************************************************/

int arm64_ucontext_getmask(ucontext_t *ucp)
{
  return sigprocmask(SIG_BLOCK, NULL, &ucp->uc_sigmask);
}

void arm64_ucontext_setmask(const ucontext_t *ucp)
{
  sigprocmask(SIG_SETMASK, &ucp->uc_sigmask, NULL);
}

void arm64_ucontext_swapmask(ucontext_t *oucp,
                             const ucontext_t *ucp)
{
  sigprocmask(SIG_SETMASK, &ucp->uc_sigmask, &oucp->uc_sigmask);
}

/****************************************************************************
 * Name: makecontext
 *
 * Description:
 *   Make ucp run func(argc integer arguments) on ucp->uc_stack.  When func
 *   returns, the context continues with ucp->uc_link, or the task exits if
 *   it is NULL.
 *
 ****************************************************************************/

void makecontext(ucontext_t *ucp, CODE void (*func)(void), int argc,
                 ...)
{
  uint64_t *stack;
  uintptr_t sp;
  va_list ap;
  int i;

  sp  = (uintptr_t)ucp->uc_stack.ss_sp + ucp->uc_stack.ss_size;
  sp &= ~(STACKFRAME_ALIGN - 1);

  /* The first eight arguments go in x0-x7, the others on the stack */

  if (argc > 8)
    {
      sp -= (argc - 8) * sizeof(uint64_t);
      sp &= ~(STACKFRAME_ALIGN - 1);
    }

  stack = (uint64_t *)sp;

  va_start(ap, argc);
  for (i = 0; i < argc; i++)
    {
      if (i < 8)
        {
          ucp->uc_mcontext.args[i] = va_arg(ap, int64_t);
        }
      else
        {
          stack[i - 8] = va_arg(ap, int64_t);
        }
    }

  va_end(ap);

  ucp->uc_mcontext.regs[0]  = (uintptr_t)ucp->uc_link;         /* x19 */
  ucp->uc_mcontext.regs[10] = 0;                               /* x29 */
  ucp->uc_mcontext.regs[11] = (uintptr_t)arm64_ucontext_start; /* x30 */
  ucp->uc_mcontext.sp       = sp;
  ucp->uc_mcontext.pc       = (uintptr_t)func;
}
