/****************************************************************************
 * include/net/route.h
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

#ifndef __INCLUDE_NET_ROUTE_H
#define __INCLUDE_NET_ROUTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <netinet/in.h>

#include <nuttx/net/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTF_UP          0x0001   /* Route usable. */
#define RTF_GATEWAY     0x0002   /* Destination is a gateway. */
#define RTF_HOST        0x0004   /* Host entry (net otherwise). */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes the route information passed with the SIOCADDRT
 * and SIOCDELRT ioctl commands (see include/nuttx/net/ioctl.h).
 */

struct rtentry
{
  struct sockaddr_storage rt_dst;     /* Address of the network */
  struct sockaddr_storage rt_gateway; /* Gateway address associated with
                                       * the hop */
  struct sockaddr_storage rt_genmask; /* Network mask defining the sub-net */
  uint16_t rt_flags;
  FAR char *rt_dev;                   /* Forcing the device at add. */
};

struct in6_rtmsg
{
  struct in6_addr rtmsg_dst;
  struct in6_addr rtmsg_src;
  struct in6_addr rtmsg_gateway;
  uint32_t rtmsg_type;
  uint16_t rtmsg_dst_len;
  uint16_t rtmsg_src_len;
  uint32_t rtmsg_metric;
  unsigned long int rtmsg_info;
  uint32_t rtmsg_flags;
  int rtmsg_ifindex;
};

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
             FAR struct sockaddr_storage *router);

/****************************************************************************
 * Name: net_delroute
 *
 * Description:
 *   Delete a route from the routing table.  This is just a convenience
 *   wrapper for the SIOCDELRT ioctl call.
 *
 * Input Parameters:
 *   sockfd   - Any socket descriptor
 *   target   - Target address on the remote network (required)
 *   netmask  - Network mask defining the external network (required)
 *
 * Returned Value:
 *   OK on success; -1 on failure with the errno variable set appropriately.
 *
 ****************************************************************************/

int delroute(int sockfd, FAR struct sockaddr_storage *target,
             FAR struct sockaddr_storage *netmask);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NET_ROUTE_H */
