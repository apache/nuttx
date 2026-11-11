/****************************************************************************
 * include/nuttx/signalfd.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SIGNALFD_H
#define __INCLUDE_NUTTX_SIGNALFD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct task_group_s;

/****************************************************************************
 * Name: signalfd_notify
 *
 * Description:
 *   Notify the signalfd poll waiters of the target task group that a signal
 *   has been delivered.  Called from the signal dispatch path (which may run
 *   in interrupt context), so this must not take any sleeping lock.
 *
 *   The poll waiters are anchored on the long-lived task group rather than
 *   the fd-scoped device, so this path never dereferences a device that a
 *   concurrent close() may have freed.
 *
 * Input Parameters:
 *   group - The task group the signal was delivered to.
 *   signo - The signal number that was delivered.
 *
 ****************************************************************************/

#ifdef CONFIG_SIGNAL_FD
void signalfd_notify(FAR struct task_group_s *group, int signo);
#else
#  define signalfd_notify(group, signo)
#endif

#endif /* __INCLUDE_NUTTX_SIGNALFD_H */
