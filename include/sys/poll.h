/****************************************************************************
 * include/sys/poll.h
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

#ifndef __INCLUDE_SYS_POLL_H
#define __INCLUDE_SYS_POLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <signal.h>
#include <semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Poll event definitions:
 *
 *   POLLIN
 *     Data other than high-priority data may be read without blocking.
 *   POLLRDNORM
 *     Normal data may be read without blocking.
 *   POLLRDBAND
 *     Priority data may be read without blocking.
 *   POLLPRI
 *     High priority data may be read without blocking.
 *
 *   POLLOUT
 *     Normal data may be written without blocking.
 *   POLLWRNORM
 *     Equivalent to POLLOUT.
 *   POLLWRBAND
 *     Priority data may be written.
 *
 *   POLLERR
 *     An error has occurred (revents only).
 *   POLLHUP
 *     Device has been disconnected (revents only).
 *   POLLNVAL
 *     Invalid fd member (revents only).
 */

#define POLLIN       (0x01)  /* NuttX does not make priority distinctions */
#define POLLRDNORM   (0x01)
#define POLLRDBAND   (0x01)

#define POLLPRI      (0x02)

#define POLLOUT      (0x04)  /* NuttX does not make priority distinctions */
#define POLLWRNORM   (0x04)
#define POLLWRBAND   (0x04)

#define POLLERR      (0x08)
#define POLLHUP      (0x10)
#define POLLRDHUP    (0x10)  /* NuttX does not support shutdown(fd, SHUT_RD) */
#define POLLNVAL     (0x20)

#define POLLFD       (0x00)
#define POLLFILE     (0x40)
#define POLLSOCK     (0x80)
#define POLLMASK     (0xC0)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* The number of poll descriptors (required by poll() specification */

typedef unsigned int nfds_t;

/* In the standard poll() definition, the size of the event set is 'short'.
 * Here we pick the uint32_t type to be consistent with the linux to support
 * EPOLLRDHUP, EPOLLWAKEUP, EPOLLONESHOT...
 */

typedef uint32_t pollevent_t;

/* This is the NuttX variant of the standard pollfd structure.  The poll()
 * interfaces receive a variable length array of such structures.
 *
 * REVISIT: In a multi-threaded environment, one use case might be to share
 * a single, array of struct pollfd in poll calls on different threads.
 * That use case is not supportable with this variant due way in which the
 * non-standard internal fields are used in the implementation of poll().
 */

struct pollfd
{
  /* Standard fields */

  int          fd;      /* The descriptor being polled */
  pollevent_t  events;  /* The input event flags */
  pollevent_t  revents; /* The output event flags */

  /* Non-standard fields used internally by NuttX. */

  FAR void    *ptr;     /* The psock or file being polled */
  FAR sem_t   *sem;     /* Pointer to semaphore used to post output event */
  FAR void    *priv;    /* For use by drivers */
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

int poll(FAR struct pollfd *fds, nfds_t nfds, int timeout);

int ppoll(FAR struct pollfd *fds, nfds_t nfds,
          FAR const struct timespec *timeout_ts,
          FAR const sigset_t *sigmask);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_POLL_H */
