/****************************************************************************
 * libs/libc/assert/lib_assert.c
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

#include <nuttx/arch.h>

#include <assert.h>
#include <stdlib.h>
#include <syscall.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void __assert(FAR const char *filename, int linenum, FAR const char *msg)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  if (up_interrupt_context())
    {
      _assert(filename, linenum, msg, NULL, true);
    }
  else
#endif
    {
#ifdef CONFIG_ARCH_HAVE_SYSCALL
      up_assert(filename, linenum, msg);
#else
      _assert(filename, linenum, msg, NULL, false);
#endif
    }

  abort();
}
