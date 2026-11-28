/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gdwifi/net_compat/lwip/lwip_napt.h
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

/* Empty shim: the SDK lwIP drops out of the build when CONFIG_NET=y.
 * The few functions still referenced live in wifi_netif.h/dhcpd.h.
 */

#ifndef _GDWIFI_COMPAT_LWIP_STUB_H
#define _GDWIFI_COMPAT_LWIP_STUB_H
#endif
