/****************************************************************************
 * include/sys/eventfd.h
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

#ifndef __INCLUDE_SYS_EVENTFD_H
#define __INCLUDE_SYS_EVENTFD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <fcntl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EFD_NONBLOCK  O_NONBLOCK
#define EFD_SEMAPHORE O_SYNC
#define EFD_CLOEXEC   O_CLOEXEC

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* Type for event counter */

#ifdef __INT64_DEFINED
typedef uint64_t eventfd_t;
#else
typedef uint32_t eventfd_t;
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

int eventfd(unsigned int count, int flags);

int eventfd_read(int fd, FAR eventfd_t *value);
int eventfd_write(int fd, eventfd_t value);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SYS_EVENTFD_H */
