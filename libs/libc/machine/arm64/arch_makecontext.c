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
int arm64_ucontext_getmask(FAR ucontext_t *ucp);
void arm64_ucontext_setmask(FAR const ucontext_t *ucp);
void arm64_ucontext_swapmask(FAR const ucontext_t *ucp,
                             FAR ucontext_t *oucp);

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

int arm64_ucontext_getmask(FAR ucontext_t *ucp)
{
  return sigprocmask(SIG_BLOCK, NULL, &ucp->uc_sigmask);
}

void arm64_ucontext_setmask(FAR const ucontext_t *ucp)
{
  sigprocmask(SIG_SETMASK, &ucp->uc_sigmask, NULL);
}

void arm64_ucontext_swapmask(FAR const ucontext_t *ucp,
                             FAR ucontext_t *oucp)
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

void makecontext(FAR ucontext_t *ucp, CODE void (*func)(void), int argc,
                 ...)
{
  FAR uint64_t *stack;
  uintptr_t sp;
  va_list ap;
  int i;

  sp = ((uintptr_t)ucp->uc_stack.ss_sp + ucp->uc_stack.ss_size) & ~15;

  /* The first eight arguments go in x0-x7, the others on the stack */

  if (argc > 8)
    {
      sp = (sp - (argc - 8) * sizeof(uint64_t)) & ~15;
    }

  stack = (FAR uint64_t *)sp;

  va_start(ap, argc);
  for (i = 0; i < argc; i++)
    {
      if (i < 8)
        {
          ucp->uc_mcontext.regs[i] = (uint64_t)va_arg(ap, int);
        }
      else
        {
          stack[i - 8] = (uint64_t)va_arg(ap, int);
        }
    }

  va_end(ap);

  ucp->uc_mcontext.regs[19] = (uintptr_t)ucp->uc_link;
  ucp->uc_mcontext.regs[29] = 0;
  ucp->uc_mcontext.regs[30] = (uintptr_t)arm64_ucontext_start;
  ucp->uc_mcontext.sp       = sp;
  ucp->uc_mcontext.pc       = (uintptr_t)func;
}
