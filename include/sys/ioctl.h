/****************************************************************************
 * include/sys/ioctl.h
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

#ifndef __INCLUDE_SYS_IOCTL_H
#define __INCLUDE_SYS_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Get NuttX configuration and NuttX-specific IOCTL definitions */

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_NET
/* Include network IOCTL definitions */

#  include <nuttx/net/ioctl.h>

#ifdef CONFIG_NETDEV_WIRELESS_IOCTL
/* Include wireless network IOCTL definitions */

#  include <nuttx/wireless/wireless.h>
#endif
#endif /* CONFIG_NET */

#ifdef CONFIG_INPUT
/* Include input driver IOCTL definitions */

#  include <nuttx/input/ioctl.h>
#endif

#ifdef CONFIG_DRIVERS_WIRELESS
/* Include wireless character driver IOCTL definitions */

#  include <nuttx/wireless/ioctl.h>
#endif

#ifdef CONFIG_WIRELESS_IEEE802154

/* Include ieee802.15.4 MAC IOCTL definitions */

#  include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#endif /* CONFIG_WIRELESS_IEEE802154 */

/****************************************************************************
 * Pre-processor Definitions
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

/****************************************************************************
 * Name: ioctl
 *
 * Description:
 *   Perform device specific operations.
 *
 *   ioctl() is a non-standard UNIX-like API
 *
 * Input Parameters:
 *   fd       File/socket descriptor of device
 *   req      The ioctl command
 *   arg      The argument of the ioctl cmd, OR
 *   ...      A third argument of type unsigned long is still expected.
 *
 * Returned Value:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   -1 on failure with errno set properly:
 *
 *   EBADF
 *     'fd' is not a valid descriptor.
 *   EFAULT
 *     'arg' references an inaccessible memory area.
 *   EINVAL
 *     'cmd' or 'arg' is not valid.
 *   ENOTTY
 *     'fd' is not associated with a character special device.
 *   ENOTTY
 *      The specified request does not apply to the kind of object that the
 *      descriptor 'fd' references.
 *
 ****************************************************************************/

int ioctl(int fd, int req, ...);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_IOCTL_H */
