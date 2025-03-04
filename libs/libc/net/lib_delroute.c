/****************************************************************************
 * libs/libc/net/lib_delroute.c
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

#include <nuttx/config.h>

#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <string.h>
#include <net/route.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_delroute
 *
 * Description:
 *   Add a new route to the routing table.  This is just a convenience
 *   wrapper for the SIOCADDRT ioctl call.
 *
 * Input Parameters:
 *   sockfd   - Any socket descriptor
 *   target   - Target address on the remote network (required)
 *   netmask  - Network mask defining the external network (required)
 *   len      - The address struct length.
 *
 * Returned Value:
 *   OK on success; -1 on failure with the errno variable set appropriately.
 *
 ****************************************************************************/

int delroute(int sockfd, FAR void *target, FAR void *netmask, socklen_t len)
{
  struct rtentry entry;

  if (len < sizeof(struct in_addr) || len > sizeof(struct sockaddr_storage))
    {
      return -EINVAL;
    }

  /* Set up the rtentry structure */

  memset(&entry, 0, sizeof(struct rtentry));
  memcpy(&entry.rt_dst, target, len);
  memcpy(&entry.rt_genmask, netmask, len);

  /* Then perform the ioctl */

  return ioctl(sockfd, SIOCDELRT, (unsigned long)((uintptr_t)&entry));
}
