/****************************************************************************
 * arch/sim/src/sim/sim_hostepoll.h
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

#ifndef __ARCH_SIM_SRC_SIM_HOSTEPOLL_H
#define __ARCH_SIM_SRC_SIM_HOSTEPOLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include "hostfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NUTTX_EPOLL_CLOEXEC NUTTX_O_CLOEXEC

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct host_epoll_event
{
  uint32_t  events;
  uintptr_t data;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int host_epoll_create1(int flags);
int host_epoll_ctl(int epoll_fd, int op, int fd, uint32_t events,
                   uintptr_t data);
int host_epoll_wait(int epoll_fd, struct host_epoll_event *events,
                    int maxevents, int timeout_ms);

#endif /* __ARCH_SIM_SRC_SIM_HOSTEPOLL_H */
