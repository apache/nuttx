/****************************************************************************
 * include/nuttx/net/phy.h
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

#ifndef __INCLUDE_NUTTX_NET_PHY_H
#define __INCLUDE_NUTTX_NET_PHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <signal.h>
#include <sys/types.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Maximum number of phy_notify clients */

#ifndef CONFIG_PHY_NOTIFICATION_NCLIENTS
#  define CONFIG_PHY_NOTIFICATION_NCLIENTS 1
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: phy_notify_subscribe
 *
 * Description:
 *   Setup up to deliver signals to the task identified by 'pid' when
 *   there is any change indicated by an interrupt from the PHY associated
 *   with 'intf'
 *
 *   NOTE: This function is intended to be called only from an Ethernet
 *   driver in support of the SIOCMIISIG ioctl command.  It should never
 *   by called directly by application logic.
 *
 * Input Parameters:
 *   intf  - Provides the name of the network interface, for example, "eth0".
 *   pid   - Identifies the task to receive the signal.  The special value
 *           of zero means to use the pid of the current task.
 *   event - Describe the way a task is to be notified
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_PHY_INTERRUPT
int phy_notify_subscribe(FAR const char *intf, pid_t pid,
                         FAR struct sigevent *event);
#endif

/****************************************************************************
 * Name: phy_notify_unsubscribe
 *
 * Description:
 *   Stop the deliver of signals for events from the PHY associated with
 *   'intf' to the task identified by 'pid'
 *
 *   NOTE: This function is intended to be called only from an Ethernet
 *   driver in support of the SIOCMIISIG ioctl command.  It should never
 *   by called directly by application logic.
 *
 * Input Parameters:
 *   intf  - Provides the name of the network interface, for example, "eth0".
 *   pid   - Identifies the task that was receiving notifications.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_PHY_INTERRUPT
int phy_notify_unsubscribe(FAR const char *intf, pid_t pid);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_PHY_H */
