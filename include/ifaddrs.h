/****************************************************************************
 * include/ifaddrs.h
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

#ifndef __INCLUDE_IFADDRS_H
#define __INCLUDE_IFADDRS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ifa_broadaddr
#  define ifa_broadaddr ifa_ifu.ifu_broadaddr /* Broadcast address of this interface. */
#endif
#ifndef ifa_dstaddr
#  define ifa_dstaddr   ifa_ifu.ifu_dstaddr   /* Point-to-point destination address. */
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct ifaddrs
{
  FAR struct ifaddrs  *ifa_next;
  FAR char            *ifa_name;
  unsigned int         ifa_flags;
  FAR struct sockaddr *ifa_addr;
  FAR struct sockaddr *ifa_netmask;
  union
  {
  /* At most one of the following two is valid. If the IFF_BROADCAST
   * bit is set in `ifa_flags', then `ifa_broadaddr' is valid. If the
   * IFF_POINTOPOINT bit is set, then `ifa_dstaddr' is valid.
   * It is never the case that both these bits are set at once.
   */

    FAR struct sockaddr *ifu_broadaddr; /* Broadcast address of this interface. */
    FAR struct sockaddr *ifu_dstaddr;   /* Point-to-point destination address. */
  } ifa_ifu;
  FAR void            *ifa_data;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

extern int getifaddrs(FAR struct ifaddrs **addrs);
extern void freeifaddrs(FAR struct ifaddrs *addrs);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_IFADDRS_H */
