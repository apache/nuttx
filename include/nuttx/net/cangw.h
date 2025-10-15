/****************************************************************************
 * include/nuttx/net/cangw.h
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

#ifndef __INCLUDE_NUTTX_NET_CANGW_H
#define __INCLUDE_NUTTX_NET_CANGW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/can.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CAN gateway types */

enum
{
  CGW_TYPE_UNSPEC,
  CGW_TYPE_CAN_CAN,     /* CAN->CAN routing */
  CGW_TYPE_CAN_ETH1722, /* CAN->Ethernet routing */
  CGW_TYPE_ETH1722_CAN, /* Ethernet->CAN routing */
  __CGW_TYPE_MAX
};

#define CGW_TYPE_MAX (__CGW_TYPE_MAX - 1)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* CAN-CAN gateway filter */

struct can_can_gw_s
{
  struct can_filter filter;
};

/* The structure define the cangw rule rule that will be used to check if the
 * incoming packet match the rule and need to be forwarded.
 */

struct cangw_rule_s
{
  /* This node is used to maintain a single-linked list of gateway rules. */

  sq_entry_t node;
  FAR struct net_driver_s *dest_dev;    /* Destination Device */
  union
  {
    struct can_can_gw_s ccgw;       /* CAN to CAN gateway check rule */
  } gw;
  uint8_t gwtype;                   /* Route type: CAN-CAN/CAN-ETH/ETH-CAN */
  uint16_t flags;                   /* GW rule flags */
};

/* The structure holding the cangw statistics that are gathered if
 * CONFIG_NET_STATISTICS is defined.
 */

#ifdef CONFIG_NET_STATISTICS
struct cangw_stats_s
{
  net_stats_t drop;         /* Number of dropped msg */
  net_stats_t forward;      /* Number of forward msg */
  net_stats_t recv;         /* Number of received msg */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_CANGW_H */
