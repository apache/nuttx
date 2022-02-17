/****************************************************************************
 * include/sys/timerfd.h
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

#ifndef __INCLUDE_SYS_TIMERFD_H
#define __INCLUDE_SYS_TIMERFD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <fcntl.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TFD_NONBLOCK O_NONBLOCK
#define TFD_CLOEXEC  O_CLOEXEC

#define TFD_TIMER_ABSTIME TIMER_ABSTIME

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* Type for timer counter */

/* Type for event counter */

#ifdef __INT64_DEFINED
typedef uint64_t timerfd_t;
#else
typedef uint32_t timerfd_t;
#endif

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

int timerfd_create(int clockid, int flags);

int timerfd_settime(int fd, int flags,
                    FAR const struct itimerspec *new_value,
                    FAR struct itimerspec *old_value);

int timerfd_gettime(int fd, FAR struct itimerspec *curr_value);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SYS_TIMERFD_H */
