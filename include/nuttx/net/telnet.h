/****************************************************************************
 * include/nuttx/net/telnet.h
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

#ifndef __INCLUDE_NUTTX_NET_TELNET_H
#define __INCLUDE_NUTTX_NET_TELNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/net/ioctl.h>

#ifdef CONFIG_NETDEV_TELNET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The minimum size of a device name.  Big enough to hold /dev/telnet65535. */

#define TELNET_DEVPATH_MAX 18

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* NETDEV ioctl command:
 *
 * Command:      SIOCTELNET
 * Description:  Create a Telnet sessions.
 * Argument:     A pointer to a write-able instance of struct
 *               telnet_session_s.
 * Dependencies: CONFIG_NETDEV_TELNET
 */

struct telnet_session_s
{
  int ts_sd;                           /* Socket descriptor for session. */
  char ts_devpath[TELNET_DEVPATH_MAX]; /* Path to new session driver */
};

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: telnet_initialize
 *
 * Description:
 *   Create the Telnet factory at /dev/telnet.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef __KERNEL__
int telnet_initialize(void);
#endif

#endif /* CONFIG_NETDEV_TELNET */
#endif /* __INCLUDE_NUTTX_NET_TELNET_H */
