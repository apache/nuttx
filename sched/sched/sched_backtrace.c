/****************************************************************************
 * sched/sched/sched_backtrace.c
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

#include "sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_backtrace
 *
 * Description:
 *  Get thread backtrace from specified tid.
 *  Store up to SIZE return address of the current program state in
 *  ARRAY and return the exact number of values stored.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_BACKTRACE
int sched_backtrace(pid_t tid, FAR void **buffer, int size, int skip)
{
  FAR struct tcb_s *rtcb;
  irqstate_t flags;
  int ret = 0;
  if (tid >= 0)
    {
      flags = enter_critical_section();
      rtcb  = nxsched_get_tcb(tid);
      if (rtcb != NULL)
        {
          ret = up_backtrace(rtcb, buffer, size, skip);
        }

      leave_critical_section(flags);
    }

  return ret;
}
#endif
