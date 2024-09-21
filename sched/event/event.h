/****************************************************************************
 * sched/event/event.h
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

#ifndef __SCHED_EVENT_EVENT_H
#define __SCHED_EVENT_EVENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>
#include <nuttx/list.h>
#include <nuttx/semaphore.h>

#include <nuttx/event.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef struct nxevent_wait_s nxevent_wait_t;

struct nxevent_wait_s
{
  struct list_node        node;    /* Wait node of current task */
  nxevent_mask_t          expect;  /* Expect events of wait task */
  nxevent_flags_t         eflags;  /* Event flags of wait task */
  sem_t                   sem;     /* Wait sem of current task */
};

#endif /* __SCHED_EVENT_EVENT_H */
