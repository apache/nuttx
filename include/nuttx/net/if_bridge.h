/****************************************************************************
 * include/nuttx/net/if_bridge.h
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

#ifndef __INCLUDE_NUTTX_NET_IF_BRIDGE_H
#define __INCLUDE_NUTTX_NET_IF_BRIDGE_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BRCTL_VERSION                  1

#define BRCTL_GET_VERSION              0
#define BRCTL_GET_BRIDGES              1
#define BRCTL_ADD_BRIDGE               2
#define BRCTL_DEL_BRIDGE               3
#define BRCTL_ADD_IF                   4
#define BRCTL_DEL_IF                   5
#define BRCTL_GET_BRIDGE_INFO          6
#define BRCTL_GET_PORT_LIST            7
#define BRCTL_SET_BRIDGE_FORWARD_DELAY 8
#define BRCTL_SET_BRIDGE_HELLO_TIME    9
#define BRCTL_SET_BRIDGE_MAX_AGE       10
#define BRCTL_SET_AGEING_TIME          11
#define BRCTL_SET_GC_INTERVAL          12
#define BRCTL_GET_PORT_INFO            13
#define BRCTL_SET_BRIDGE_STP_STATE     14
#define BRCTL_SET_BRIDGE_PRIORITY      15
#define BRCTL_SET_PORT_PRIORITY        16
#define BRCTL_SET_PATH_COST            17
#define BRCTL_GET_FDB_ENTRIES          18

#endif /* __INCLUDE_NUTTX_NET_IF_BRIDGE_H */
