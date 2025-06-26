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

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxevent_tickwait_core
 *
 * Description:
 *   Core implementation for various nxevent_*wait APIs.
 *
 *   This internal function is separated from wrapper APIs to provide
 *   flexibility in supplying wait object pointers. Avoid using it directly
 *   unless absolutely necessary. In rare cases where thread stacks are
 *   strictly isolated, stack-allocated wait objects become inaccessible.
 *
 * Input Parameters:
 *   event     - Address of the event object
 *   events    - Event set to wait for:
 *               - 0 indicates waiting for any event
 *   nxevent_wait_t
 *             - Per-thread wait object (internal implementation detail)
 *   eflags    - Event flags
 *   delay     - Ticks to wait from start time until event posting:
 *               - 0 ticks behaves identically to nxevent_trywait()
 *
 * Returned Value:
 *   Internal OS interface - not for application use.
 *   Returns matching event set on success.
 *   Returns 0 if no matching events occurred within specified time.
 *
 ****************************************************************************/

nxevent_mask_t nxevent_tickwait_core(FAR nxevent_t *event,
                                     nxevent_mask_t events,
                                     FAR nxevent_wait_t *wait,
                                     nxevent_flags_t eflags, uint32_t delay);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __SCHED_EVENT_EVENT_H */
