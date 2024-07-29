/****************************************************************************
 * include/nuttx/event.h
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

#ifndef __INCLUDE_NUTTX_EVENT_H
#define __INCLUDE_NUTTX_EVENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/list.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Initializers */

#define NXEVENT_INITIALIZER(e, v) {LIST_INITIAL_VALUE((e).list), (v)}

/* Event Wait Flags */

#define NXEVENT_WAIT_ALL     (1 << 0) /* Bit 0: Wait ALL */
#define NXEVENT_WAIT_RESET   (1 << 1) /* Bit 1: Reset events before wait */
#define NXEVENT_WAIT_NOCLEAR (1 << 2) /* Bit 2: Do not clear events after wait */

/* Event Post Flags */

#define NXEVENT_POST_ALL     (1 << 0) /* Bit 0: Post ALL */
#define NXEVENT_POST_SET     (1 << 1) /* Bit 1: Set event after post */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef struct nxevent_s      nxevent_t;
typedef unsigned long         nxevent_mask_t;
typedef unsigned long         nxevent_flags_t;

struct nxevent_s
{
  struct list_node         list;    /* Waiting list of nxevent_wait_t */
  volatile nxevent_mask_t  events;  /* Pending Events */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name: nxevent_init
 *
 * Description:
 *   This routine initializes an event object, Set of default events to post
 *   to event.
 *
 * Input Parameters:
 *   event  - Address of the event object
 *   events - Set of events to post to event
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void nxevent_init(FAR nxevent_t *event, nxevent_mask_t events);

/****************************************************************************
 * Name: nxevent_destroy
 *
 * Description:
 *   This function is used to destroy the event.
 *
 * Input Parameters:
 *   event  - Address of the event object
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxevent_destroy(FAR nxevent_t *event);

/****************************************************************************
 * Name: nxevent_reset
 *
 * Description:
 *   Reset events mask to a specific value.
 *
 * Input Parameters:
 *   event  - Address of the event object
 *   events - Set of events to post to event
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications, and
 *   hence follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxevent_reset(FAR nxevent_t *event, nxevent_mask_t events);

/****************************************************************************
 * Name: nxevent_post
 *
 * Description:
 *   Post one or more events to an event object.
 *
 *   This routine posts one or more events to an event object. All tasks
 *   waiting on the event object event whose waiting conditions become
 *   met by this posting immediately unpend.
 *
 *   Posting differs from setting in that posted events are merged together
 *   with the current set of events tracked by the event object.
 *
 * Input Parameters:
 *   event  - Address of the event object
 *   events - Set of events to post to event
 *          - Set events to 0 will be considered as any,
 *            waking up the waiting thread immediately.
 *   eflags - Events flags
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int nxevent_post(FAR nxevent_t *event, nxevent_mask_t events,
                 nxevent_flags_t eflags);

/****************************************************************************
 * Name: nxevent_wait
 *
 * Description:
 *   Wait for all of the specified events.
 *
 *   This routine waits on event object event until all of the specified
 *   events have been delivered to the event object. A thread may wait on
 *   up to 32 distinctly numbered events that are expressed as bits in a
 *   single 32-bit word.
 *
 * Input Parameters:
 *   event  - Address of the event object
 *   events - Set of events to wait, 0 will indicate wait from any events
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   Return of matching events upon success, Otherwise, 0 is returned if OS
 *   internal error.
 *
 ****************************************************************************/

nxevent_mask_t nxevent_wait(FAR nxevent_t *event, nxevent_mask_t events,
                            nxevent_flags_t eflags);

/****************************************************************************
 * Name: nxevent_tickwait
 *
 * Description:
 *   Wait for all of the specified events for the specified tick time.
 *
 *   This routine waits on event object event until all of the specified
 *   events have been delivered to the event object, or the maximum wait time
 *   timeout has expired. A thread may wait on up to 32 distinctly numbered
 *   events that are expressed as bits in a single 32-bit word.
 *
 * Input Parameters:
 *   event  - Address of the event object
 *   events - Set of events to wait
 *          - Set events to 0 will indicate wait from any events
 *   eflags - Events flags
 *   delay  - Ticks to wait from the start time until the event is
 *            posted.  If ticks is zero, then this function is equivalent
 *            to nxevent_trywait().
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   Return of matching events upon success.
 *   0 if matching events were not received within the specified time.
 *
 ****************************************************************************/

nxevent_mask_t nxevent_tickwait(FAR nxevent_t *event, nxevent_mask_t events,
                                nxevent_flags_t eflags, uint32_t delay);

/****************************************************************************
 * Name: nxevent_trywait
 *
 * Description:
 *   Try wait for all of the specified events.
 *
 *   This routine try to waits on event object event if any of the specified
 *   events have been delivered to the event object. A thread may wait on
 *   up to 32 distinctly numbered events that are expressed as bits in a
 *   single 32-bit word.
 *
 * Input Parameters:
 *   event  - Address of the event object
 *   events - Set of events to wait
 *          - Set events to 0 will indicate wait from any events
 *   eflags - Events flags
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   Return of matching events upon success.
 *   0 if matching events were not received.
 *
 ****************************************************************************/

nxevent_mask_t nxevent_trywait(FAR nxevent_t *event, nxevent_mask_t events,
                               nxevent_flags_t eflags);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_EVENT_H */
