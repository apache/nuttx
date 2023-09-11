/****************************************************************************
 * libs/libc/sched/sched_gettid.c
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
#include <nuttx/tls.h>

#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_gettid
 *
 * Description:
 *   Utilise thread local storage to get the thread ID of the currently
 *   executing thread and cache the result for further queries. Caching this
 *   result reduces system call loading when using CONFIG_BUILD_KERNEL.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   On success, returns the thread ID of the calling process.
 *
 ****************************************************************************/

#if !defined(CONFIG_BUILD_FLAT) && defined(CONFIG_SCHED_THREAD_LOCAL) && !defined(__KERNEL__)

pid_t sched_gettid(void)
{
  FAR struct tls_info_s *tlsinfo = tls_get_info();

  DEBUGASSERT(tlsinfo);

  if (tlsinfo->tl_tid <= 0)
    {
        tlsinfo->tl_tid = gettid();
    }

  return tlsinfo->tl_tid;
}
#endif
