/****************************************************************************
 * sched/wdog/wd_recover.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/sched.h>

#include "wdog/wdog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wd_recover
 *
 * Description:
 *   This function is called from nxtask_recover() when a task is deleted via
 *   task_delete() or via pthread_cancel(). It checks if the deleted task
 *   is waiting for a timed event and if so cancels the timeout
 *
 * Input Parameters:
 *   tcb - The TCB of the terminated task or thread
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function is called from task deletion logic in a safe context.
 *
 ****************************************************************************/

void wd_recover(FAR struct tcb_s *tcb)
{
  /* The task is being deleted.  If it is waiting for any timed event, then
   * cancel the watchdog now so that no events occur after the watchdog
   * expires. Obviously there are lots of race conditions here so this will
   * most certainly have to be revisited in the future.
   *
   */

  wd_cancel(&tcb->waitdog);
}
