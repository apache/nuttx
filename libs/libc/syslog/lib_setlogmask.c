/****************************************************************************
 * libs/libc/syslog/lib_setlogmask.c
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
#include <syslog.h>

#include <nuttx/tls.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setlogmask
 *
 * Description:
 *   The setlogmask() function sets the logmask and returns the previous
 *   mask. If the mask argument is 0, the current logmask is not modified.
 *
 *   The SYSLOG priorities are: LOG_EMERG, LOG_ALERT, LOG_CRIT, LOG_ERR,
 *   LOG_WARNING, LOG_NOTICE, LOG_INFO, and LOG_DEBUG.  The bit corresponding
 *   to a priority p is LOG_MASK(p); LOG_UPTO(p) provides the mask of all
 *   priorities in the above list up to and including p.
 *
 *   NOTE:  setlogmask is not a thread-safe, re-entrant function.  Concurrent
 *   use of setlogmask() will have undefined behavior.
 *
 *   As POSIX requires, the mask is a per-process value: it controls the
 *   SYSLOG output of the calling task group only.  A new task group starts
 *   with the system wide mask, which nx_setlogmask() sets.
 *
 ****************************************************************************/

int setlogmask(int mask)
{
  FAR struct task_info_s *info = task_get_info();
  uint8_t oldmask;

  /* Without task info (early boot), vsyslog() lets every priority pass */

  if (info == NULL)
    {
      return LOG_ALL;
    }

  oldmask = info->ta_syslog_mask;
  if (mask != 0)
    {
      /* If the mask argument is 0, the current logmask is not modified. */

      info->ta_syslog_mask = (uint8_t)mask;
    }

  return oldmask;
}
