/****************************************************************************
 * sched/task/task_join.c
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

#include <nuttx/nuttx.h>
#include <sys/types.h>
#include <stdbool.h>
#include <pthread.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "sched/sched.h"
#include "group/group.h"
#include "pthread/pthread.h"

#ifndef CONFIG_DISABLE_PTHREAD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_joindestroy
 ****************************************************************************/

void nxtask_joindestroy(FAR struct tcb_s *tcb)
{
  nxsem_destroy(&tcb->join_sem);
}

/****************************************************************************
 * Name: nxtask_joininit
 ****************************************************************************/

void nxtask_joininit(FAR struct tcb_s *tcb)
{
  sq_init(&tcb->join_queue);
  nxsem_init(&tcb->join_sem, 0, 0);
}

#endif /* !CONFIG_DISABLE_PTHREAD */
