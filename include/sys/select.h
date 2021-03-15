/****************************************************************************
 * include/sys/select.h
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

#ifndef __INCLUDE_SYS_SELECT_H
#define __INCLUDE_SYS_SELECT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <signal.h>
#include <sys/time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get the total number of descriptors that we will have to support */

#define FD_SETSIZE 256

/* We will use a 32-bit bitsets to represent the set of descriptors.  How
 * many uint32_t's do we need to span all descriptors?
 */

#if FD_SETSIZE <= 32
#  define __SELECT_NUINT32 1
#elif FD_SETSIZE <= 64
#  define __SELECT_NUINT32 2
#elif FD_SETSIZE <= 96
#  define __SELECT_NUINT32 3
#elif FD_SETSIZE <= 128
#  define __SELECT_NUINT32 4
#elif FD_SETSIZE <= 160
#  define __SELECT_NUINT32 5
#elif FD_SETSIZE <= 192
#  define __SELECT_NUINT32 6
#elif FD_SETSIZE <= 224
#  define __SELECT_NUINT32 7
#elif FD_SETSIZE <= 256
#  define __SELECT_NUINT32 8
#else
#  warning "Larger fd_set needed"
#endif

/* These macros map a file descriptor to an index and bit number */

#define _FD_NDX(fd) ((fd) >> 5)
#define _FD_BIT(fd) ((fd) & 0x1f)

/* Standard helper macros */

#define FD_CLR(fd,set) \
  ((((fd_set*)(set))->arr)[_FD_NDX(fd)] &= ~(UINT32_C(1)<< _FD_BIT(fd)))
#define FD_SET(fd,set) \
  ((((fd_set*)(set))->arr)[_FD_NDX(fd)] |= (UINT32_C(1) << _FD_BIT(fd)))
#define FD_ISSET(fd,set) \
 (((((fd_set*)(set))->arr)[_FD_NDX(fd)] & (UINT32_C(1) << _FD_BIT(fd))) != 0)
#define FD_ZERO(set) \
   memset((set), 0, sizeof(fd_set))

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct fd_set_s
{
  uint32_t arr[__SELECT_NUINT32];
};

typedef struct fd_set_s fd_set;

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

struct timeval;
int select(int nfds, FAR fd_set *readfds, FAR fd_set *writefds,
           FAR fd_set *exceptfds, FAR struct timeval *timeout);

int pselect(int nfds, FAR fd_set *readfds, FAR fd_set *writefds,
            FAR fd_set *exceptfds, FAR const struct timespec *timeout,
            FAR const sigset_t *sigmask);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_SELECT_H */
