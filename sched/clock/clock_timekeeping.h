/****************************************************************************
 * sched/clock/clock_timekeeping.h
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

#ifndef __SCHED_CLOCK_CLOCK_TIMEKEEPING_H
#define __SCHED_CLOCK_CLOCK_TIMEKEEPING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int clock_timekeeping_get_wall_time(FAR struct timespec *ts);
int clock_timekeeping_set_wall_time(FAR const struct timespec *ts);

void clock_update_wall_time(void);

void clock_inittimekeeping(FAR const struct timespec *tp);

#endif /* __SCHED_CLOCK_CLOCK_TIMEKEEPING_H */
