/****************************************************************************
 * sched/clock/clock_internal.h
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

#ifndef __SCHED_CLOCK_INTERNAL_CLOCK_H
#define __SCHED_CLOCK_INTERNAL_CLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: clock_systime_timespec_nolock
 *
 * Description:
 *   Return the current value of the system timer counter as a struct
 *   timespec.
 *
 * Input Parameters:
 *   ts - Location to return the time
 *
 * Returned Value:
 *   OK (0) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int clock_systime_timespec_nolock(FAR struct timespec *ts);

#endif /* __SCHED_CLOCK_INTERNAL_CLOCK_H */
