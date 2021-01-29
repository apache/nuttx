/****************************************************************************
 * include/nuttx/net/loopback.h
 * Definitions for use with local loopback device
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

#ifndef __INCLUDE_NUTTX_NET_LOOPBACK_H
#define __INCLUDE_NUTTX_NET_LOOPBACK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/ip.h>

#ifdef CONFIG_NET_LOOPBACK

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

#ifdef CONFIG_LIBC_NETDB
/* Local loopback hostname */

EXTERN const char           g_lo_hostname[];
#endif

/* Local loopback addresses */

#ifdef CONFIG_NET_IPv4
EXTERN const in_addr_t      g_lo_ipv4addr;
EXTERN const in_addr_t      g_lo_ipv4mask;
#endif

#ifdef CONFIG_NET_IPv6
EXTERN const net_ipv6addr_t g_lo_ipv6addr;
EXTERN const net_ipv6addr_t g_lo_ipv6mask;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: localhost_initialize
 *
 * Description:
 *   Initialize the localhost, loopback network driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int localhost_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_LOOPBACK */
#endif /* __INCLUDE_NUTTX_NET_LOOPBACK_H */
