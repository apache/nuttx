/****************************************************************************
 * lib/syslog/lib_setlogmask.c
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

#include "syslog/syslog.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The currently enabled set of syslog priorities */

uint8_t g_syslog_mask = LOG_ALL;

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
 *   Per OpenGroup.org "If the maskpri argument is 0, the current log mask
 *   is not modified."  In this implementation, the value zero is permitted
 *   in order to disable all syslog levels.
 *
 *   NOTE:  setlogmask is not a thread-safe, re-entrant function.  Concurrent
 *   use of setlogmask() will have undefined behavior.
 *
 *   REVISIT: Per POSIX the syslog mask should be a per-process value but in
 *   NuttX, the scope of the mask is dependent on the nature of the build:
 *
 *   Flat Build:  There is one, global SYSLOG mask that controls all output.
 *   Protected Build:  There are two SYSLOG masks.  One within the kernel
 *     that controls only kernel output.  And one in user-space that controls
 *     only user SYSLOG output.
 *   Kernel Build:  The kernel build is compliant with the POSIX requirement:
 *     There will be one mask for each user process, controlling the SYSLOG
 *     output only form that process.  There will be a separate mask
 *     accessible only in the kernel code to control kernel SYSLOG output.
 *
 ****************************************************************************/

int setlogmask(int mask)
{
  uint8_t oldmask;

  oldmask       = g_syslog_mask;
  g_syslog_mask = (uint8_t)mask;

  return oldmask;
}
