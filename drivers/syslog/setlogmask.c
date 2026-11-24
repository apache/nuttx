/****************************************************************************
 * drivers/syslog/setlogmask.c
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

#include <stdarg.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/addrenv.h>
#include <nuttx/sched.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/tls.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

uint8_t g_syslog_mask = CONFIG_SYSLOG_DEFAULT_MASK;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void task_syslogmask(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct task_group_s *group = tcb->group;
#ifdef CONFIG_ARCH_ADDRENV
  FAR struct addrenv_s *oldenv;
#endif

  /* The task info of an exiting task may be gone already */

  if (group == NULL || group->tg_info == NULL)
    {
      return;
    }

#ifdef CONFIG_ARCH_ADDRENV
  /* The task info of an unprivileged group lives in its own address
   * environment.
   */

  if (tcb->addrenv_own != NULL)
    {
      addrenv_select(tcb->addrenv_own, &oldenv);
    }
  else if ((group->tg_flags & GROUP_FLAG_PRIVILEGED) == 0)
    {
      return;
    }
#endif

  group->tg_info->ta_syslog_mask = (uint8_t)(*(FAR const int *)arg);

#ifdef CONFIG_ARCH_ADDRENV
  if (tcb->addrenv_own != NULL)
    {
      addrenv_restore(oldenv);
    }
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_setlogmask
 *
 * Description:
 *   Set the system wide syslog mask, which new task groups start with, and
 *   the setlogmask() mask of every existing task group.  Unlike
 *   setlogmask(), a mask of zero is applied: LOG_NONE disables all output.
 *
 * Input Parameters:
 *   mask - The new mask
 *
 * Returned Value:
 *   The previous system wide mask.
 *
 ****************************************************************************/

int nx_setlogmask(int mask)
{
  uint8_t oldmask;

  oldmask = g_syslog_mask;
  g_syslog_mask = (uint8_t)mask;

  nxsched_foreach(task_syslogmask, &mask);
  return oldmask;
}

/****************************************************************************
 * Name: nx_syslog
 *
 * Description:
 *   The syslog() of the kernel, see include/syslog.h.  Only the system wide
 *   mask filters it.
 *
 ****************************************************************************/

void nx_syslog(int priority, FAR const IPTR char *fmt, ...)
{
  va_list ap;

  if ((g_syslog_mask & LOG_MASK(LOG_PRI(priority))) != 0)
    {
      va_start(ap, fmt);
      nx_vsyslog(priority, fmt, &ap);
      va_end(ap);
    }
}
