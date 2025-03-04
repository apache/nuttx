/****************************************************************************
 * include/malloc.h
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

#ifndef __INCLUDE_MALLOC_H
#define __INCLUDE_MALLOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Special PID to query the info about alloc, free and mempool */

#define PID_MM_ORPHAN  ((pid_t)-6)
#define PID_MM_BIGGEST ((pid_t)-5)
#define PID_MM_FREE    ((pid_t)-4)
#define PID_MM_ALLOC   ((pid_t)-3)
#define PID_MM_LEAK    ((pid_t)-2)
#define PID_MM_MEMPOOL ((pid_t)-1)

/* For Linux and MacOS compatibility */

#define malloc_usable_size malloc_size

/* mallopt options that actually do something */

#define M_TRIM_THRESHOLD    -1
#define M_TOP_PAD           -2
#define M_MMAP_THRESHOLD    -3
#define M_MMAP_MAX          -4
#define M_CHECK_ACTION      -5
#define M_PERTURB           -6
#define M_ARENA_TEST        -7
#define M_ARENA_MAX         -8

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct mallinfo
{
  int arena;    /* This is the total size of memory allocated
                 * for use by malloc in bytes. */
  int ordblks;  /* This is the number of free (not in use) chunks */
  int aordblks; /* This is the number of allocated (in use) chunks */
  int mxordblk; /* Size of the largest free (not in use) chunk */
  int uordblks; /* This is the total size of memory occupied by
                 * chunks handed out by malloc. */
  int fordblks; /* This is the total size of memory occupied
                 * by free (not in use) chunks. */
  int usmblks;  /* This is the largest amount of space ever allocated */
};

struct malltask
{
  pid_t pid; /* Process id */
#if CONFIG_MM_BACKTRACE >= 0
  unsigned long seqmin; /* The minimum sequence */
  unsigned long seqmax; /* The maximum sequence */
#endif
};

struct mallinfo_task
{
  int aordblks; /* This is the number of allocated (in use) chunks for task */
  int uordblks; /* This is the total size of memory occupied for task */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

int mallopt(int param, int value);
struct mallinfo mallinfo(void);
size_t malloc_size(FAR void *ptr);
struct mallinfo_task mallinfo_task(FAR const struct malltask *task);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_MALLOC_H */
