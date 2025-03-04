/****************************************************************************
 * include/nuttx/net/netdev_lowerhalf.h
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

#ifndef __INCLUDE_NUTTX_NET_NETDEV_LOWERHALF_H
#define __INCLUDE_NUTTX_NET_NETDEV_LOWERHALF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/spinlock.h>
#include <nuttx/atomic.h>

#include <net/if.h>
#include <netinet/in.h>

#include <nuttx/net/ip.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/wireless/wireless.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NETDEV_WORK_THREAD_POLLING_PERIOD
#  define CONFIG_NETDEV_WORK_THREAD_POLLING_PERIOD 0
#endif

/* Layout for net packet:
 *
 * | <-------------- NETPKT_BUFLEN ---------------> |
 * +---------------------+-------------------+------+      +-------------+
 * | reserved for driver |       data        | free | ---> | next netpkt |
 * +---------------------+-------------------+------+      +-------------+
 * |                     | <--- datalen ---> |
 * ^base                 ^data
 */

/* Layout for linked net packet, you can get list of (data, len) by
 * netpkt_to_iov() interface:
 *
 *            | <----------- datalen = sum(len) ------------> |
 * +----------+-----------+     +-----------+     +-----------+------+
 * | reserved |   data    | --> |   data    | --> |   data    | free |
 * +----------+-----------+     +-----------+     +-----------+------+
 * |          | <- len -> |     | <- len -> |     | <- len -> |
 * ^base      ^data             ^data             ^data
 */

#define NETPKT_BUFLEN   CONFIG_IOB_BUFSIZE
#define NETPKT_BUFNUM   CONFIG_IOB_NBUFFERS

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* We use IOB as our buffer now, we may change to some other structure when
 * needed, so define a type netpkt_t for lower half.
 * TODO: Provide interface of its queue, maybe a simple wrapper of iob_queue.
 */

typedef struct iob_s netpkt_t;
typedef struct iob_queue_s netpkt_queue_t;

enum netpkt_type_e
{
  NETPKT_TX,
  NETPKT_RX,
  NETPKT_TYPENUM
};

/* This structure is the generic form of state structure used by lower half
 * netdev driver. This state structure is passed to the netdev driver when
 * the driver is initialized. Then, on subsequent callbacks into the lower
 * half netdev logic, this structure is provided so that the netdev logic can
 * maintain state information.
 *
 * Normally that netdev logic will have its own, custom state structure
 * that is simply cast to struct netdev_lowerhalf_s. In order to perform such
 * casts, the initial fields of the custom state structure match the initial
 * fields of the following generic netdev state structure.
 */

struct netdev_ops_s;
struct wireless_ops_s;
struct netdev_lowerhalf_s
{
  FAR const struct netdev_ops_s *ops;

  /* Extended operations. */

#ifdef CONFIG_NETDEV_WIRELESS_HANDLER
  FAR const struct wireless_ops_s *iw_ops;
#endif

  /* Max # of buffer held by driver */

  atomic_t quota[NETPKT_TYPENUM];

  /* The structure used by net stack.
   * Note: Do not change its fields unless you know what you are doing.
   *
   * Fields that lowerhalf should never touch (used by upper half):
   *   d_ifup, d_ifdown, d_txavail, d_addmac, d_rmmac, d_ioctl, d_private
   */

  struct net_driver_s netdev;
};

/* This structure is a set a callback functions used to call from the upper-
 * half, generic netdev driver into lower-half, platform-specific logic that
 * supports the low-level functionality.
 */

struct netdev_ops_s
{
  CODE int (*ifup)(FAR struct netdev_lowerhalf_s *dev);
  CODE int (*ifdown)(FAR struct netdev_lowerhalf_s *dev);

  /* transmit - Try to send a packet, non-blocking, own the netpkt and
   *            need to call netpkt_free to free it sometime later.
   *   Returned Value:
   *     OK for successfully sent the packet, driver can take pkt to its
   *       own queue and return OK (remember to free it later).
   *     Negated errno value for failure, will stop current sending, the pkt
   *       will be recycled by upper half.
   */

  CODE int (*transmit)(FAR struct netdev_lowerhalf_s *dev,
                       FAR netpkt_t *pkt);

  /* receive - Try to receive a packet, non-blocking
   *   Returned Value:
   *     A netpkt contains the packet, or NULL if no more packets.
   */

  CODE FAR netpkt_t *(*receive)(FAR struct netdev_lowerhalf_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
  CODE int (*addmac)(FAR struct netdev_lowerhalf_s *dev,
                     FAR const uint8_t *mac);
  CODE int (*rmmac)(FAR struct netdev_lowerhalf_s *dev,
                    FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
  CODE int (*ioctl)(FAR struct netdev_lowerhalf_s *dev, int cmd,
                    unsigned long arg);
#endif

  /* reclaim - try to reclaim packets sent by netdev. */

  CODE void (*reclaim)(FAR struct netdev_lowerhalf_s *dev);
};

/* This structure is a set of wireless handlers, leave unsupported operations
 * as NULL is OK.
 */

#ifdef CONFIG_NETDEV_WIRELESS_HANDLER
typedef CODE int (*iw_handler_rw)(FAR struct netdev_lowerhalf_s *dev,
                                 FAR struct iwreq *iwr, bool set);
typedef CODE int (*iw_handler_ro)(FAR struct netdev_lowerhalf_s *dev,
                                 FAR struct iwreq *iwr);

struct wireless_ops_s
{
  /* Connect / disconnect operation, should exist if essid or bssid exists */

  CODE int (*connect)(FAR struct netdev_lowerhalf_s *dev);
  CODE int (*disconnect)(FAR struct netdev_lowerhalf_s *dev);

  /* The following attributes need both set and get. */

  iw_handler_rw essid;
  iw_handler_rw bssid;
  iw_handler_rw passwd;
  iw_handler_rw mode;
  iw_handler_rw auth;
  iw_handler_rw freq;
  iw_handler_rw bitrate;
  iw_handler_rw txpower;
  iw_handler_rw country;
  iw_handler_rw sensitivity;

  /* Scan operation: start scan (set=1) / get scan result (set=0). */

  iw_handler_rw scan;

  /* Get-only attributes. */

  iw_handler_ro range;
};
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_lower_register
 *
 * Description:
 *   Register a network device driver and assign a name to it so that it can
 *   be found in subsequent network ioctl operations on the device.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure to be registered.
 *   lltype - Link level protocol used by the driver (Ethernet, SLIP, TUN,
 *            ...
 *
 * Returned Value:
 *   0:Success; negated errno on failure.
 *
 ****************************************************************************/

int netdev_lower_register(FAR struct netdev_lowerhalf_s *dev,
                          enum net_lltype_e lltype);

/****************************************************************************
 * Name: netdev_lower_unregister
 *
 * Description:
 *   Unregister a network device driver.
 *
 * Input Parameters:
 *   dev - The lower half device driver structure to un-register
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netdev_lower_unregister(FAR struct netdev_lowerhalf_s *dev);

/****************************************************************************
 * Name: netdev_lower_carrier_on
 *
 * Description:
 *   Notifies the networking layer about an available carrier.
 *   (e.g. a cable was plugged in)
 *
 * Input Parameters:
 *   dev - The lower half device driver structure
 *
 ****************************************************************************/

void netdev_lower_carrier_on(FAR struct netdev_lowerhalf_s *dev);

/****************************************************************************
 * Name: netdev_lower_carrier_off
 *
 * Description:
 *   Notifies the networking layer about an disappeared carrier.
 *   (e.g. a cable was unplugged)
 *
 * Input Parameters:
 *   dev - The lower half device driver structure
 *
 ****************************************************************************/

void netdev_lower_carrier_off(FAR struct netdev_lowerhalf_s *dev);

/****************************************************************************
 * Name: netdev_lower_rxready
 *
 * Description:
 *   Notifies the networking layer about an RX packet is ready to read.
 *
 * Input Parameters:
 *   dev - The lower half device driver structure
 *
 ****************************************************************************/

void netdev_lower_rxready(FAR struct netdev_lowerhalf_s *dev);

/****************************************************************************
 * Name: netdev_lower_txdone
 *
 * Description:
 *   Notifies the networking layer about a TX packet is sent.
 *
 * Input Parameters:
 *   dev - The lower half device driver structure
 *
 ****************************************************************************/

void netdev_lower_txdone(FAR struct netdev_lowerhalf_s *dev);

/****************************************************************************
 * Name: netdev_lower_quota_load
 *
 * Description:
 *   Fetch the quota, use this interface when device is running.
 *
 * Input Parameters:
 *   dev  - The lower half device driver structure
 *   type - Whether get quota for TX or RX
 *
 ****************************************************************************/

#define netdev_lower_quota_load(dev, type) atomic_read(&dev->quota[type])

/****************************************************************************
 * Name: netpkt_alloc
 *
 * Description:
 *   Allocate a netpkt structure.
 *
 * Input Parameters:
 *   dev  - The lower half device driver structure
 *   type - Whether used for TX or RX
 *
 * Returned Value:
 *   Pointer to the packet, NULL on failure
 *
 ****************************************************************************/

FAR netpkt_t *netpkt_alloc(FAR struct netdev_lowerhalf_s *dev,
                           enum netpkt_type_e type);

/****************************************************************************
 * Name: netpkt_free
 *
 * Description:
 *   Release a netpkt structure.
 *
 * Input Parameters:
 *   dev  - The lower half device driver structure
 *   pkt  - The packet to release
 *   type - Whether used for TX or RX
 *
 ****************************************************************************/

void netpkt_free(FAR struct netdev_lowerhalf_s *dev, FAR netpkt_t *pkt,
                 enum netpkt_type_e type);

/****************************************************************************
 * Name: netpkt_copyin
 *
 * Description:
 *   Copy 'len' bytes of data from a buffer into the netpkt, starting at
 *   'offset' of netpkt.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *   src    - The source buffer
 *   len    - How many bytes to copy
 *   offset - The offset of netpkt to put the data
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netpkt_copyin(FAR struct netdev_lowerhalf_s *dev, FAR netpkt_t *pkt,
                  FAR const uint8_t *src, unsigned int len, int offset);

/****************************************************************************
 * Name: netpkt_copyout
 *
 * Description:
 *   Copy 'len' bytes of data from netpkt into a buffer, starting at
 *   'offset' of netpkt.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   dest   - The destination buffer
 *   pkt    - The net packet
 *   len    - How many bytes to copy
 *   offset - The offset of netpkt to get the data
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netpkt_copyout(FAR struct netdev_lowerhalf_s *dev, FAR uint8_t *dest,
                   FAR const netpkt_t *pkt, unsigned int len, int offset);

/****************************************************************************
 * Name: netpkt_getdata/getbase
 *
 * Description:
 *   Get the pointer of data/base in a netpkt, used when NETPKT_BUFLEN is
 *   big enough to fit a full packet in.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *
 * Returned Value:
 *   Pointer data/base, NULL on failure.
 *
 ****************************************************************************/

FAR uint8_t *netpkt_getdata(FAR struct netdev_lowerhalf_s *dev,
                            FAR netpkt_t *pkt);
FAR uint8_t *netpkt_getbase(FAR netpkt_t *pkt);

/****************************************************************************
 * Name: netpkt_setdatalen
 *
 * Description:
 *   Set the length of data in netpkt, used when data is written into
 *   netpkt by data/base pointer, no need to set this length after
 *   copyin.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *   len    - The length of data in netpkt
 *
 * Returned Value:
 *   The new effective data length, or a negated errno value on error.
 *
 ****************************************************************************/

int netpkt_setdatalen(FAR struct netdev_lowerhalf_s *dev,
                      FAR netpkt_t *pkt, unsigned int len);

/****************************************************************************
 * Name: netpkt_getdatalen
 *
 * Description:
 *   Get the length of data in netpkt.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *
 * Returned Value:
 *   The length of data in netpkt.
 *
 ****************************************************************************/

unsigned int netpkt_getdatalen(FAR struct netdev_lowerhalf_s *dev,
                               FAR netpkt_t *pkt);

/****************************************************************************
 * Name: netpkt_reset_reserved
 *
 * Description:
 *   Reset the reserved length (the starting point of data) of netpkt
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *   len    - The reserved length
 *
 ****************************************************************************/

void netpkt_reset_reserved(FAR struct netdev_lowerhalf_s *dev,
                           FAR netpkt_t *pkt, unsigned int len);

/****************************************************************************
 * Name: netpkt_is_fragmented
 *
 * Description:
 *   Returns whether the netpkt is fragmented into different blocks.
 *     In other words, NETPKT_BUFLEN < reserved + total data
 *
 * Input Parameters:
 *   pkt - The net packet
 *
 ****************************************************************************/

bool netpkt_is_fragmented(FAR netpkt_t *pkt);

/****************************************************************************
 * Name: netpkt_to_iov
 *
 * Description:
 *   Write each piece of data/len into iov array.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *   iov    - The iov array to write
 *   iovcnt - The number of elements in the iov array
 *
 * Returned Value:
 *   The actual written count of iov entries.
 *
 ****************************************************************************/

int netpkt_to_iov(FAR struct netdev_lowerhalf_s *dev, FAR netpkt_t *pkt,
                  FAR struct iovec *iov, int iovcnt);

/****************************************************************************
 * Name: netpkt_tryadd_queue
 *
 * Description:
 *   Add one net packet to the end of a queue without waiting for resources
 *   to become free.
 *
 * Input Parameters:
 *   pkt   - The packet to add
 *   queue - The queue to add the packet to
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

#define netpkt_tryadd_queue(pkt, queue) iob_tryadd_queue(pkt, queue)

/****************************************************************************
 * Name: netpkt_remove_queue
 *
 * Description:
 *   Remove one net packet from the head of a queue.
 *
 * Input Parameters:
 *   queue - The queue to remove the packet from
 *
 * Returned Value:
 *   The packet removed from the queue.  NULL is returned if the queue is
 *   empty.
 *
 ****************************************************************************/

#define netpkt_remove_queue(queue) iob_remove_queue(queue)

/****************************************************************************
 * Name: netpkt_free_queue
 *
 * Description:
 *   Free all net packets in a queue.
 *
 * Input Parameters:
 *   queue - The queue to free
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define netpkt_free_queue(queue) iob_free_queue(queue)

#endif /* __INCLUDE_NUTTX_NET_NETDEV_LOWERHALF_H */
