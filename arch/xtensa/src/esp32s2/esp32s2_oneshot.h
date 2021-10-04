/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_oneshot.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_ONESHOT_H
#define __ARCH_XTENSA_SRC_ESP32S2_ONESHOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include "esp32s2_tim.h"

#ifdef CONFIG_ESP32S2_ONESHOT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This describes the callback function that will be invoked when the oneshot
 * timer expires.  The oneshot fires, the client will receive:
 *
 *   arg - The opaque argument provided when the interrupt was registered
 */

typedef void (*oneshot_handler_t)(void *arg);

/* The oneshot client must allocate an instance of this structure and call
 * esp32s2_oneshot_initialize() before using the oneshot facilities.  The
 * client should not access the contents of this structure directly since
 * the contents are subject to change.
 */

struct esp32s2_oneshot_s
{
  uint8_t chan;                         /* The timer/counter in use */
  volatile bool running;                /* True: the timer is running */
  struct esp32s2_tim_dev_s        *tim; /* Pointer returned by
                                         * esp32s2_tim_init() */
  volatile oneshot_handler_t   handler; /* Oneshot expiration callback */
  volatile void                   *arg; /* The argument that will accompany
                                         * the callback */
  uint32_t                  resolution; /* us */
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
 * Name: esp32s2_oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer wrapper.
 *
 * Input Parameters:
 *   oneshot    Allocated instance of the oneshot state structure.
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32s2_oneshot_initialize(struct esp32s2_oneshot_s *oneshot,
                               int chan, uint16_t resolution);

/****************************************************************************
 * Name: esp32s2_oneshot_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds).
 *
 * Input Parameters:
 *   oneshot    Allocated instance of the oneshot state structure.
 *
 * Output Parameters:
 *   usec    The location in which to return the maximum delay in us.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32s2_oneshot_max_delay(struct esp32s2_oneshot_s *oneshot,
                              uint64_t *usec);

/****************************************************************************
 * Name: esp32s2_oneshot_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   oneshot Allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           esp32s2_oneshot_initialize();
 *   handler The function to call when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32s2_oneshot_start(struct esp32s2_oneshot_s *oneshot,
                          oneshot_handler_t handler, void *arg,
                          const struct timespec *ts);

/****************************************************************************
 * Name: esp32s2_oneshot_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *
 * Input Parameters:
 *   oneshot Allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           esp32s2_oneshot_initialize();
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

int esp32s2_oneshot_cancel(struct esp32s2_oneshot_s *oneshot,
                           struct timespec *ts);

/****************************************************************************
 * Name: esp32s2_oneshot_current
 *
 * Description:
 *   Get the current time.
 *
 * Input Parameters:
 *   oneshot Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           esp32s2_oneshot_initialize();
 *
 * Output Parameters:
 *   usec    The location in which to return the current time in us.
 *
 * Returned Value:
 *   Zero (OK).
 *
 ****************************************************************************/

int esp32s2_oneshot_current(struct esp32s2_oneshot_s *oneshot,
                            uint64_t *usec);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ESP32S2_ONESHOT */
#endif /* __ARCH_XTENSA_SRC_ESP32S2_ONESHOT_H */
