/****************************************************************************
 * include/nuttx/net/telnet.h
 *
 *   Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 * Argument:     A pointer to a write-able instance of struct telnet_session_s.
 * Dependencies: CONFIG_NETDEV_TELNET
 */

struct telnet_session_s
{
  int ts_sd;                           /* Socket descriptor for session. */
  char ts_devpath[TELNET_DEVPATH_MAX]; /* Path to new session driver */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: telnet_initialize
 *
 * Description:
 *   Create the Telnet factory at /dev/telnet.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef __KERNEL__
int telnet_initialize(void);
#endif

#endif /* CONFIG_NETDEV_TELNET */
#endif /* __INCLUDE_NUTTX_NET_TELNET_H */
