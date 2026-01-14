/****************************************************************************
 * drivers/rpmsg/rpmsg_router.h
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

#ifndef __DRIVERS_RPMSG_RPMSG_ROUTER_H
#define __DRIVERS_RPMSG_RPMSG_ROUTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/rpmsg/rpmsg.h>

#ifdef CONFIG_RPMSG_ROUTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_ROUTER_NAME            "router:"
#define RPMSG_ROUTER_NAME_LEN        7
#define RPMSG_ROUTER_NAME_PREFIX     "r:"
#define RPMSG_ROUTER_NAME_PREFIX_LEN 2
#define RPMSG_ROUTER_CPUNAME_LEN     8

#define RPMSG_ROUTER_CREATE          1
#define RPMSG_ROUTER_DESTROY         2

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct rpmsg_router_s
{
  uint32_t cmd;
  uint32_t tx_len;
  uint32_t rx_len;
  char     cpuname[RPMSG_ROUTER_CPUNAME_LEN];
} end_packed_struct;

#endif /* CONFIG_RPMSG_ROUTER */
#endif /* __DRIVERS_RPMSG_RPMSG_ROUTER_H */
