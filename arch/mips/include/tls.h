/****************************************************************************
 * arch/mips/include/tls.h
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

#ifndef __ARCH_MIPS_INCLUDE_TLS_H
#define __ARCH_MIPS_INCLUDE_TLS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <assert.h>
#include <nuttx/arch.h>
#include <nuttx/tls.h>

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_tls_info
 *
 * Description:
 *   Return the TLS information structure for the currently executing thread.
 *   When TLS is enabled, up_createstack() will align allocated stacks to
 *   the TLS_STACK_ALIGN value.  An instance of the following structure will
 *   be implicitly positioned at the "lower" end of the stack.  Assuming a
 *   "push down" stack, this is at the "far" end of the stack (and can be
 *   clobbered if the stack overflows).
 *
 *   If an MCU has a "push up" then that TLS structure will lie at the top
 *   of the stack and stack allocation and initialization logic must take
 *   care to preserve this structure content.
 *
 *   The stack memory is fully accessible to user mode threads.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A pointer to TLS info structure at the beginning of the STACK memory
 *   allocation.  This is essentially an application of the TLS_INFO(sp)
 *   macro and has a platform dependency only in the manner in which the
 *   stack pointer (sp) is obtained and interpreted.
 *
 ****************************************************************************/

#ifdef CONFIG_TLS_ALIGNED
static inline FAR struct tls_info_s *up_tls_info(void)
{
  DEBUGASSERT(!up_interrupt_context());
  return TLS_INFO((uintptr_t)mips_getsp());
}
#else
#  define up_tls_info() tls_get_info()
#endif

#endif /* __ARCH_MIPS_INCLUDE_TLS_H */
