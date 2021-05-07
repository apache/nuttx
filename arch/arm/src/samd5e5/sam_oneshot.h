/****************************************************************************
 * arch/arm/src/samd5e5/sam_oneshot.h
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

#ifndef __ARCH_ARM_SRC_SAMD5E5_SAM_ONESHOT_H
#define __ARCH_ARM_SRC_SAMD5E5_SAM_ONESHOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include "sam_tc.h"

#ifdef CONFIG_SAMD5E5_ONESHOT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ONESHOT_INITIALIZED(s) (((s)->tch) != NULL)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes the callback function that will be invoked when the oneshot
 * timer expires.  The oneshot fires, the client will receive:
 *
 *   arg - The opaque argument provided when the interrupt was registered
 */

typedef void (*oneshot_handler_t)(void *arg);

/* The oneshot client must allocate an instance of this structure and called
 * sam_oneshot_initialize() before using the oneshot facilities.  The client
 * should not access the contents of this structure directly since the
 * contents are subject to change.
 */

struct sam_oneshot_s
{
  uint8_t chan;                       /* The timer/counter in use */
  volatile bool running;              /* True: the timer is running */
  TC_HANDLE tch;                      /* Handle returned by
                                       * sam_tc_initialize() */
  volatile oneshot_handler_t handler; /* Oneshot expiration callback */
  volatile void *arg;                 /* The argument that will accompany
                                       * the callback */
#ifdef CONFIG_SAMD5E5_FREERUN
  volatile uint32_t start_count;      /* Stores the value of the freerun counter,
                                       * at each start of the onshot timer. Is neccesary
                                       * to find out if the onshot counter was updated
                                       * correctly at the time of the call to
                                       * sam_oneshot_cancel or not. */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
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
 * Name: sam_oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer wrapper
 *
 * Input Parameters:
 *   oneshot    Caller allocated instance of the oneshot state structure
 *   chan       Timer counter channel to be used.  See the TC_CHAN*
 *              definitions in arch/arm/src/samd5e5/sam_tc.h.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int sam_oneshot_initialize(struct sam_oneshot_s *oneshot, int chan,
                           uint16_t resolution);

/****************************************************************************
 * Name: sam_oneshot_max_delay
 *
 * Description:
 *   Return the maximum delay supported by the one shot timer (in
 *   microseconds).
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           sam_oneshot_initialize();
 *   usec    The location in which to return the maximum delay.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int sam_oneshot_max_delay(struct sam_oneshot_s *oneshot, uint64_t *usec);

/****************************************************************************
 * Name: sam_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           sam_oneshot_initialize();
 *   freerun Caller allocated instance of the freerun state structure. This
 *           structure must have been previously initialized via a call to
 *           sam_freerun_initialize().  May be NULL if there is no matching
 *           free-running timer.
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

struct sam_freerun_s;
int sam_oneshot_start(struct sam_oneshot_s *oneshot,
                      struct sam_freerun_s *freerun,
                      oneshot_handler_t handler, void *arg,
                      const struct timespec *ts);

/****************************************************************************
 * Name: sam_oneshot_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           sam_oneshot_initialize();
 *   freerun Caller allocated instance of the freerun state structure. This
 *           structure must have been previously initialized via a call to
 *           sam_freerun_initialize().  May be NULL if there is no matching
 *           free-running timer.
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.  A time of zero is returned if the timer is
 *           not running.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

struct sam_freerun_s;
int sam_oneshot_cancel(struct sam_oneshot_s *oneshot,
                       struct sam_freerun_s *freerun, struct timespec *ts);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SAMD5E5_ONESHOT */
#endif /* __ARCH_ARM_SRC_SAMD5E5_SAM_ONESHOT_H */
