/****************************************************************************
 * arch/arm/src/common/ameba/wifi/include/ameba_lwip_off.h
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

/* Force the SDK's CONFIG_LWIP_LAYER OFF for the NuttX-compiled WiFi sources.
 *
 * NuttX owns the network stack (its native netdev, ameba_wlan.c) -- NOT
 * lwIP.  The SDK ships with CONFIG_LWIP_LAYER=1 in platform_autoconf.h,
 * which makes the SDK event source (component/wifi/common/rtw_event.c) pull
 * in the lwIP netif / DHCP-server headers (dhcp/dhcps.h -> ip_addr_t ...)
 * and call lwip_netif_set_link_up()/lwip_dhcp_stop() on connect/disconnect.
 * None of that applies here, so we compile those rtw_event.c paths out.  The
 * pieces NuttX DOES need from rtw_event.c (the event dispatch table, the
 * WPA/SAE handlers, and posting join_block_param->sema for a synchronous
 * connect) are NOT guarded by CONFIG_LWIP_LAYER, so they remain.
 *
 * This header is force-included (-include) AFTER platform_autoconf.h so the
 * #undef overrides the SDK default.  The precompiled WHC libs still
 * reference the lwip_* accessors (lwip_get_ip ...), which
 * ameba_wifi_depend.c defines unconditionally regardless of this switch.
 */

#undef CONFIG_LWIP_LAYER
