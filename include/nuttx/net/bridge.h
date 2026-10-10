/****************************************************************************
 * include/nuttx/net/bridge.h
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

#ifndef __INCLUDE_NUTTX_NET_BRIDGE_H
#define __INCLUDE_NUTTX_NET_BRIDGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/net/netdev_lowerhalf.h>

#ifdef CONFIG_NET_BRIDGE

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bridge_port_s; /* Opaque, defined by the bridge driver */

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

/****************************************************************************
 * Name: bridge_ioctl
 *
 * Description:
 *   Handle the SIOCBRADDBR, SIOCBRDELBR, SIOCBRADDIF and SIOCBRDELIF
 *   ioctl commands.
 *
 * Input Parameters:
 *   cmd - The ioctl command
 *   arg - The argument of the command: the bridge name for SIOCBRADDBR
 *         and SIOCBRDELBR, a pointer to a struct ifreq for the others
 *
 * Returned Value:
 *   OK on success, -ENOTTY if cmd is not a bridge command, or another
 *   negated errno value on failure.
 *
 ****************************************************************************/

int bridge_ioctl(int cmd, unsigned long arg);

/****************************************************************************
 * Name: bridge_input
 *
 * Description:
 *   Called by the upper half of a bridge port for every received packet.
 *   The bridge takes the packet: it forwards it to the other ports and/or
 *   passes it to the network stack through the bridge device.
 *
 * Input Parameters:
 *   port - The bridge port that received the packet
 *   pkt  - The packet, owned by the bridge from now on
 *
 * Assumptions:
 *   Called from the RX path of the port, with the port device locked.
 *
 ****************************************************************************/

void bridge_input(FAR struct bridge_port_s *port, FAR netpkt_t *pkt);

/****************************************************************************
 * Name: bridge_txdone
 *
 * Description:
 *   Called by the upper half of a bridge port when the port has completed a
 *   transmission, so that the bridge device resumes its own transmission.
 *   The bridge device receives and polls in the context of the caller
 *   (NETDEV_RX_DIRECT), so the caller must not be an interrupt handler.
 *
 ****************************************************************************/

void bridge_txdone(FAR struct bridge_port_s *port);

/****************************************************************************
 * Name: bridge_port_detach
 *
 * Description:
 *   Called by the upper half when a port device is unregistered, to remove
 *   it from its bridge.
 *
 * Input Parameters:
 *   dev - The lower half device driver structure of the port
 *
 ****************************************************************************/

void bridge_port_detach(FAR struct netdev_lowerhalf_s *dev);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_BRIDGE */
#endif /* __INCLUDE_NUTTX_NET_BRIDGE_H */
