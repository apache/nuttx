/****************************************************************************
 * include/netinet/ether.h
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

#ifndef __INCLUDE_NETINET_ETHER_H
#define __INCLUDE_NETINET_ETHER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <net/ethernet.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

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

FAR char *ether_ntoa(FAR const struct ether_addr *addr);
FAR char *ether_ntoa_r(FAR const struct ether_addr *addr, FAR char *buf);
FAR struct ether_addr *ether_aton(FAR const char *asc);
int ether_ntohost(FAR char *hostname, FAR const struct ether_addr *addr);
int ether_hostton(FAR const char *hostname, FAR struct ether_addr *addr);
int ether_line(FAR const char *line, FAR struct ether_addr *addr,
               FAR char *hostname);
FAR struct ether_addr *ether_aton_r(FAR const char *asc,
                                    FAR struct ether_addr *addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NETINET_ETHER_H */
