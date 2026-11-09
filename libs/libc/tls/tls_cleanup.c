/****************************************************************************
 * libs/libc/tls/tls_cleanup.c
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

#include <stdint.h>
#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/tls.h>
#include <nuttx/pthread.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tls_cleanup_push(FAR struct tls_info_s *tls,
                      tls_cleanup_t routine, FAR void *arg)
{
  DEBUGASSERT(tls != NULL);

  if (tls->tl_tos < CONFIG_TLS_NCLEANUP)
    {
      tls->tl_stack[tls->tl_tos].tc_cleaner = routine;
      tls->tl_stack[tls->tl_tos].tc_arg = arg;
    }

  tls->tl_tos++;
}

void tls_cleanup_pop(FAR struct tls_info_s *tls, int execute)
{
  DEBUGASSERT(tls != NULL);

  if (tls->tl_tos > 0)
    {
      tls->tl_tos--;

      /* Only access the stack array when the index is within bounds.
       * Entries that overflowed during push were silently dropped,
       * so there is nothing to execute for those slots.
       */

      if (tls->tl_tos < CONFIG_TLS_NCLEANUP && execute != 0)
        {
          FAR struct tls_cleanup_s *cb;

          cb = &tls->tl_stack[tls->tl_tos];
          cb->tc_cleaner(cb->tc_arg);
        }
    }
}

void tls_cleanup_popall(FAR struct tls_info_s *tls)
{
  DEBUGASSERT(tls != NULL);

  while (tls->tl_tos > 0)
    {
      tls_cleanup_pop(tls, 1);
    }
}
