/****************************************************************************
 * include/nuttx/net/pkt.h
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

#ifndef __INCLUDE_NUTTX_NET_PKT_H
#define __INCLUDE_NUTTX_NET_PKT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/net/netconfig.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_input
 *
 * Description:
 *   Handle incoming packet input
 *
 *   This function provides the interface between Ethernet device drivers and
 *   packet socket logic.  All frames that are received should be provided to
 *   pkt_input() prior to other routing.
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received packet
 *
 * Returned Value:
 *   OK    The packet has been processed  and can be deleted
 *   ERROR There is a matching connection, but could not dispatch the packet
 *         yet.  Useful when a packet arrives before a recv call is in
 *         place.
 *
 * Assumptions:
 *   Called from the network diver with the network locked.
 *
 ****************************************************************************/

struct net_driver_s; /* Forward reference */
int pkt_input(FAR struct net_driver_s *dev);

#endif /* __INCLUDE_NUTTX_NET_PKT_H */
