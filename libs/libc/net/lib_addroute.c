/****************************************************************************
 * libs/libc/net/lib_addroute.c
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
