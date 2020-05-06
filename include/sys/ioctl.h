/****************************************************************************
 * include/sys/ioctl.h
 *
 *   Copyright (C) 2007, 2008, 2012, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
