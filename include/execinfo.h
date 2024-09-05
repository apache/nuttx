/****************************************************************************
 * include/execinfo.h
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

#ifndef __INCLUDE_EXECINFO_H
#define __INCLUDE_EXECINFO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sched.h>

#include <assert.h>
#include <stdio.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 3: ' 0x' prefix */

#define BACKTRACE_PTR_FMT_WIDTH  ((int)sizeof(uintptr_t) * 2 + 3)

/* Buffer size needed to hold formatted `depth` backtraces */

#define BACKTRACE_BUFFER_SIZE(d) (BACKTRACE_PTR_FMT_WIDTH * (d) + 1)

#define backtrace(b, s) sched_backtrace(_SCHED_GETTID(), b, s, 0)
#define dump_stack()    sched_dumpstack(_SCHED_GETTID())

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

FAR char **backtrace_symbols(FAR void *const *buffer, int size);
void backtrace_symbols_fd(FAR void *const *buffer, int size, int fd);
int backtrace_format(FAR char *buffer, int size,
                     FAR void *backtrace[], int depth);

#  if CONFIG_LIBC_BACKTRACE_BUFFSIZE > 0
int backtrace_record(int skip);
int backtrace_remove(int index);
FAR void **backtrace_get(int index, FAR int *size);
void backtrace_dump(void);
#  else
#    define backtrace_record(skip) (-ENOSYS)
#    define backtrace_remove(index) (-ENOSYS)
#    define backtrace_get(index, size) (*(size)=0)
#    define backtrace_dump()
#  endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_EXECINFO_H */
