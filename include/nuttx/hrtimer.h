/****************************************************************************
 * include/nuttx/hrtimer.h
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

#ifndef __INCLUDE_NUTTX_HRTIMER_H
#define __INCLUDE_NUTTX_HRTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/spinlock.h>

#include <stdint.h>
#include <sys/tree.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* High-resolution timer modes:
 *
 * HRTIMER_MODE_ABS - Absolute expiration time
 * HRTIMER_MODE_REL - Relative timeout from current time
 */

enum hrtimer_mode_e
{
  HRTIMER_MODE_ABS = 0x0,  /* Absolute expiration time */
  HRTIMER_MODE_REL = 0x1   /* Relative delay from now   */
};

/* Forward declarations */

struct hrtimer_s;
struct hrtimer_node_s;
typedef struct hrtimer_s hrtimer_t;
typedef struct hrtimer_node_s hrtimer_node_t;

/* Callback type for high-resolution timer expiration */

typedef void (*hrtentry_t)(FAR struct hrtimer_s *);

/* Red-black tree node for hrtimer */

struct hrtimer_node_s
{
  RB_ENTRY(hrtimer_node_s) entry;  /* RB-tree linkage for sorted insertion */
};

/* High-resolution timer instance */

struct hrtimer_s
{
  struct hrtimer_node_s node;     /* RB-tree node for sorted insertion */
  hrtentry_t            func;     /* Expiration callback function       */
  FAR void             *arg;      /* Argument passed to callback        */
  uint64_t              expired;  /* Expiration time in absolute ns     */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: hrtimer_init
 *
 * Description:
 *   Initialize a high-resolution timer instance. Sets the expiration
 *   callback and its argument. The timer is not started by this function.
 *
 * Input Parameters:
 *   hrtimer - Pointer to hrtimer instance
 *   func    - Expiration callback function
 *   arg     - Callback argument
 *
 * Returned Value:
 *   None
 ****************************************************************************/

static inline_function
void hrtimer_init(FAR hrtimer_t *hrtimer,
                  hrtentry_t func,
                  FAR void *arg)
{
  hrtimer->func = func;
  hrtimer->arg = arg;
}

/****************************************************************************
 * Name: hrtimer_cancel
 *
 * Description:
 *   Cancel a high-resolution timer if it is pending. The timer callback
 *   will not be called if the timer was successfully canceled.
 *
 * Input Parameters:
 *   hrtimer - Timer instance to cancel
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 ****************************************************************************/

int hrtimer_cancel(FAR hrtimer_t *hrtimer);

/****************************************************************************
 * Name: hrtimer_start
 *
 * Description:
 *   Start a high-resolution timer in absolute or relative mode.
 *
 * Input Parameters:
 *   hrtimer - Timer instance
 *   ns      - Timer expiration in nanoseconds (absolute or relative)
 *   mode    - HRTIMER_MODE_ABS or HRTIMER_MODE_REL
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 ****************************************************************************/

int hrtimer_start(FAR hrtimer_t *hrtimer,
                  uint64_t ns,
                  enum hrtimer_mode_e mode);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_HRTIMER_H */
