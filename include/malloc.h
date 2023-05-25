/****************************************************************************
 * include/malloc.h
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

/* For Linux and MacOS compatibility */

#define malloc_usable_size malloc_size

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
};

struct mm_memdump_s
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
 * Public data
 ****************************************************************************/

#if CONFIG_MM_BACKTRACE >= 0
extern unsigned long g_mm_seqno;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

struct mallinfo mallinfo(void);
size_t malloc_size(FAR void *ptr);
struct mallinfo_task mallinfo_task(FAR const struct mm_memdump_s *dump);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_MALLOC_H */
