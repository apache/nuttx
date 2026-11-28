/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gdwifi/net_compat/dhcpd.h
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

/* dhcpd.h - NuttX port shim (the SDK DHCP server belongs to lwIP and drops
 * out of the build; only the softAP would use it).  Stubs so that the
 * wifi_netlink compiles.
 */

#ifndef _GDWIFI_COMPAT_DHCPD_H
#define _GDWIFI_COMPAT_DHCPD_H

#include <stdint.h>

/* lwIP macros used by wifi_netlink.c (softAP IP configuration) */

#ifndef PP_HTONL
#  define PP_HTONL(x) ((uint32_t)((((x) & 0x000000ffUL) << 24) | \
                                  (((x) & 0x0000ff00UL) <<  8) | \
                                  (((x) & 0x00ff0000UL) >>  8) | \
                                  (((x) & 0xff000000UL) >> 24)))
#endif

#ifndef LWIP_MAKEU32
#  define LWIP_MAKEU32(a, b, c, d) (((uint32_t)((a) & 0xff) << 24) | \
                                    ((uint32_t)((b) & 0xff) << 16) | \
                                    ((uint32_t)((c) & 0xff) <<  8) | \
                                     (uint32_t)((d) & 0xff))
#endif

void     dhcpd_delete_ipaddr_by_macaddr(uint8_t *mac_addr);
uint32_t dhcpd_find_ipaddr_by_macaddr(uint8_t *mac_addr);
void     dhcpd_set_dns_server(uint32_t dns);

#endif /* _GDWIFI_COMPAT_DHCPD_H */
