/****************************************************************************
 * include/ucontext.h
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

#ifndef __INCLUDE_UCONTEXT_H
#define __INCLUDE_UCONTEXT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <signal.h>

#ifdef CONFIG_ARCH_HAVE_UCONTEXT
#  include <arch/ucontext.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* uc_mcontext comes first: the architecture code uses it at offset 0 */

typedef struct ucontext_s
{
  mcontext_t              uc_mcontext;  /* Machine context */
  FAR struct ucontext_s  *uc_link;      /* Context to resume when this one
                                         * returns */
  sigset_t                uc_sigmask;   /* Signals blocked in the context */
  stack_t                 uc_stack;     /* Stack of the context */
} ucontext_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int  getcontext(FAR ucontext_t *ucp);
int  setcontext(FAR const ucontext_t *ucp);
void makecontext(FAR ucontext_t *ucp, CODE void (*func)(void), int argc,
                 ...);
int  swapcontext(FAR ucontext_t *oucp, FAR const ucontext_t *ucp);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_ARCH_HAVE_UCONTEXT */
#endif /* __INCLUDE_UCONTEXT_H */
