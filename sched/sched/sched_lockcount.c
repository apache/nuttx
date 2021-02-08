/****************************************************************************
 * sched/sched/sched_lockcount.c
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

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_lockcount
 *
 * Description:
 *   This function returns the current value of the lockcount. If zero,
 *   pre-emption is enabled; if non-zero, this value indicates the number
 *   of times that sched_lock() has been called on this thread of
 *   execution.  sched_unlock() will have to called that many times from
 *   this thread in order to re-enable pre-emption.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   lockcount
 *
 ****************************************************************************/

int sched_lockcount(void)
{
  FAR struct tcb_s *rtcb = this_task();
  return (int)rtcb->lockcount;
}
