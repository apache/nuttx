/****************************************************************************
 * net/bridge/bridge.h
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

#ifndef __NET_BRIDGE_BRIDGE_H
#define __NET_BRIDGE_BRIDGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/mm/iob.h>
#include <nuttx/mutex.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/netdev.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_NET_BRIDGE

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Locking.  Locks are always taken in this order, never the reverse:
 *
 *   br_cfglock -> port d_lock -> bridge d_lock -> br_lock
 *
 * br_cfglock serializes port configuration and the TX notification work.
 * br_lock protects the filtering database and the port TX queues and is
 * never held while calling into a network device.  A port is never
 * notified while its RX path or the bridge device is locked: transmission
 * is always started from the bridge work, which holds no device lock.
 */

struct bridge_s;

/* A bridge port */

struct bridge_port_s
{
  FAR struct bridge_s *bp_bridge;       /* The bridge of this port */
  FAR struct net_driver_s *bp_dev;      /* The port device, NULL = unused */
  struct iob_queue_s bp_txq;            /* Frames waiting to be sent */
  uint16_t bp_txqlen;                   /* Number of frames in bp_txq */
};

/* A filtering database entry */

struct bridge_fdb_s
{
  FAR struct bridge_port_s *fdb_port;   /* Port of the address, NULL = free */
  clock_t fdb_time;                     /* Last time the address was seen */
  uint8_t fdb_mac[ETHER_ADDR_LEN];      /* The learned MAC address */
};

/* A bridge */

struct bridge_s
{
  struct net_driver_s br_dev;           /* The bridge device, must be first */
  FAR struct bridge_s *br_flink;        /* Next bridge in the list */
  mutex_t br_cfglock;                   /* Configuration lock */
  mutex_t br_lock;                      /* FDB and TX queue lock */
  struct work_s br_work;                /* TX poll and notification work */
  clock_t br_ageing;                    /* FDB ageing time in ticks */
  struct bridge_port_s br_ports[CONFIG_NET_BRIDGE_MAX_PORTS];
  struct bridge_fdb_s br_fdb[CONFIG_NET_BRIDGE_FDB_SIZE];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bridge_output
 *
 * Description:
 *   Forward a frame sent by the local host through the bridge device.
 *
 * Input Parameters:
 *   br  - The bridge
 *   iob - The frame, the Ethernet header precedes IOB_DATA(iob).  The
 *         bridge takes ownership of the IOB.
 *
 * Assumptions:
 *   Called with the bridge device locked.
 *
 ****************************************************************************/

void bridge_output(FAR struct bridge_s *br, FAR struct iob_s *iob);

/****************************************************************************
 * Name: bridge_local_input
 *
 * Description:
 *   Pass a frame to the network stack through the bridge device.
 *
 * Input Parameters:
 *   br  - The bridge
 *   iob - The frame, the Ethernet header precedes IOB_DATA(iob).  The
 *         bridge takes ownership of the IOB.
 *
 ****************************************************************************/

void bridge_local_input(FAR struct bridge_s *br, FAR struct iob_s *iob);

/****************************************************************************
 * Name: bridge_schedule
 *
 * Description:
 *   Schedule the bridge work, which polls the bridge device and notifies
 *   the ports that have frames to send.
 *
 ****************************************************************************/

void bridge_schedule(FAR struct bridge_s *br);

/****************************************************************************
 * Name: bridge_port_flush
 *
 * Description:
 *   Drop the queued frames and the learned addresses of a port.
 *
 * Assumptions:
 *   Called with br_lock held.
 *
 ****************************************************************************/

void bridge_port_flush(FAR struct bridge_port_s *port);

/****************************************************************************
 * Name: bridge_poll
 *
 * Description:
 *   Send the frames queued on a bridge port.  Called from devif_poll() when
 *   BRIDGE_POLL is set.
 *
 * Input Parameters:
 *   dev      - The port device
 *   callback - The driver TX poll callback
 *
 * Returned Value:
 *   Non-zero if the driver stopped the poll.
 *
 * Assumptions:
 *   Called with the port device locked.
 *
 ****************************************************************************/

int bridge_poll(FAR struct net_driver_s *dev,
                devif_poll_callback_t callback);

/****************************************************************************
 * Name: bridge_ioctl
 *
 * Description:
 *   Handle the SIOCBRADDBR, SIOCBRDELBR, SIOCBRADDIF and SIOCBRDELIF
 *   ioctl commands.
 *
 * Returned Value:
 *   OK on success, -ENOTTY if cmd is not a bridge command, or another
 *   negated errno value on failure.
 *
 ****************************************************************************/

int bridge_ioctl(int cmd, unsigned long arg);

/****************************************************************************
 * Name: bridge_netdev_unregister
 *
 * Description:
 *   Remove a device from its bridge before it is unregistered.
 *
 ****************************************************************************/

void bridge_netdev_unregister(FAR struct net_driver_s *dev);

#endif /* CONFIG_NET_BRIDGE */
#endif /* __NET_BRIDGE_BRIDGE_H */
