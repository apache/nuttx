/****************************************************************************
 * arch/sim/src/sim/posix/sim_hostepoll.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/epoll.h>

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "sim_internal.h"
#include "sim_hostepoll.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_epoll_create1(int flags)
{
  int hostflags = 0;
  int ret;

  if (flags & NUTTX_EPOLL_CLOEXEC)
    {
      hostflags |= EPOLL_CLOEXEC;
      flags &= ~NUTTX_EPOLL_CLOEXEC;
    }

  if (flags != 0)
    {
      return -EINVAL;
    }

  ret = epoll_create1(hostflags);
  return ret < 0 ? host_errno_convert(-errno) : ret;
}

int host_epoll_ctl(int epoll_fd, int op, int fd, uint32_t events,
                   uintptr_t data)
{
  struct epoll_event event;
  struct epoll_event *eventptr = NULL;
  int ret;

  memset(&event, 0, sizeof(event));

  if (op != EPOLL_CTL_DEL)
    {
      event.events = events;
      event.data.ptr = (void *)data;
      eventptr = &event;
    }

  ret = epoll_ctl(epoll_fd, op, fd, eventptr);
  return ret < 0 ? host_errno_convert(-errno) : ret;
}

int host_epoll_wait(int epoll_fd, struct host_epoll_event *events,
                    int maxevents, int timeout_ms)
{
  struct epoll_event *hostevents;
  int ret;
  int i;

  if (events == NULL || maxevents <= 0 ||
      (size_t)maxevents > SIZE_MAX / sizeof(*hostevents))
    {
      return -EINVAL;
    }

  hostevents = malloc(maxevents * sizeof(*hostevents));
  if (hostevents == NULL)
    {
      return -ENOMEM;
    }

  ret = epoll_wait(epoll_fd, hostevents, maxevents, timeout_ms);
  if (ret < 0)
    {
      ret = host_errno_convert(-errno);
      goto out;
    }

  for (i = 0; i < ret; i++)
    {
      events[i].events = hostevents[i].events;
      events[i].data = (uintptr_t)hostevents[i].data.ptr;
    }

out:
  free(hostevents);
  return ret;
}
