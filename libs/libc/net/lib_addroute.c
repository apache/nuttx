/****************************************************************************
 * libs/libc/net/lib_addroute.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <string.h>
#include <net/route.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_addroute
 *
 * Description:
 *   Add a new route to the routing table.  This is just a convenience
 *   wrapper for the SIOCADDRT ioctl call.
 *
 * Input Parameters:
 *   sockfd   - Any socket descriptor
 *   target   - Target address on external network(required)
 *   netmask  - Network mask defining the external network (required)
 *   router   - Router address that on our network that can forward to the
 *              external network.
 *
 * Returned Value:
 *   OK on success; -1 on failure with the errno variable set appropriately.
 *
 ****************************************************************************/

int addroute(int sockfd, FAR struct sockaddr_storage *target,
             FAR struct sockaddr_storage *netmask,
             FAR struct sockaddr_storage *router)
{
  struct rtentry entry;

  /* Set up the rtentry structure */

  memset(&entry, 0, sizeof(struct rtentry));
  memcpy(&entry.rt_dst, target, sizeof(struct sockaddr_storage));
  memcpy(&entry.rt_genmask, netmask, sizeof(struct sockaddr_storage));
  memcpy(&entry.rt_gateway, router, sizeof(struct sockaddr_storage));

  /* Then perform the ioctl */

  return ioctl(sockfd, SIOCADDRT, (unsigned long)((uintptr_t)&entry));
}
