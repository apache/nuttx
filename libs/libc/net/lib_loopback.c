/****************************************************************************
 * libs/libc/net/lib_loopback.c
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

#include <nuttx/net/loopback.h>

#ifdef CONFIG_NET_LOOPBACK

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_LIBC_NETDB
/* Local loopback hostname */

const char           g_lo_hostname[] = "localhost";
#endif

/* Local loopback addresses */

#ifdef CONFIG_NET_IPv4
const in_addr_t      g_lo_ipv4addr   = HTONL(0x7f000001);
const in_addr_t      g_lo_ipv4mask   = HTONL(0xff000000);
#endif

#ifdef CONFIG_NET_IPv6
const net_ipv6addr_t g_lo_ipv6addr   =
{
  HTONS(0), HTONS(0), HTONS(0), HTONS(0),
  HTONS(0), HTONS(0), HTONS(0), HTONS(1)
};
const net_ipv6addr_t g_lo_ipv6mask   =
{
  HTONS(0xffff), HTONS(0xffff), HTONS(0xffff), HTONS(0xffff),
  HTONS(0xffff), HTONS(0xffff), HTONS(0xffff), HTONS(0xffff)
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_NET_LOOPBACK */
