/****************************************************************************
 * include/nuttx/poll.h
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

#ifndef __INCLUDE_NUTTX_POLL_H
#define __INCLUDE_NUTTX_POLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stdint.h>
#include <poll.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* POLLALWAYS: call the poll callback whether or not the driver reported an
 * event that the waiter asked for (events only, kernel only).
 */

#define POLLALWAYS   (0x10000) /* Does not conflict with Linux */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The kernel keeps the event sets in 32 bits to support EPOLLRDHUP,
 * EPOLLWAKEUP, EPOLLONESHOT...
 */

typedef uint32_t pollevent_t;

/* The poll callback type */

struct pollfd_s;
typedef CODE void (*pollcb_t)(FAR struct pollfd_s *fds);

/* The kernel entry for one polled file descriptor.  The poll() method of a
 * driver, file_poll() and poll_notify() use it.
 */

struct pollfd_s
{
  int          fd;      /* The descriptor being polled */
  pollevent_t  events;  /* The input event flags */
  pollevent_t  revents; /* The output event flags */
  FAR void    *arg;     /* The poll callback function argument */
  pollcb_t     cb;      /* The poll callback function */
  FAR void    *priv;    /* For use by drivers */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

void poll_default_cb(FAR struct pollfd_s *fds);
void poll_notify(FAR struct pollfd_s **afds, int nfds,
                 pollevent_t eventset);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_POLL_H */
