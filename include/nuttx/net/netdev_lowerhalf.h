/****************************************************************************
 * include/nuttx/net/netdev_lowerhalf.h
 * Defines architecture-specific device driver interfaces to the NuttX
 * network.
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

#include <net/if.h>
#include <netinet/in.h>

/* Compiler may not have stdatomic.h before C11 or with __STDC_NO_ATOMICS__
 * But we can also get atomic support from libmetal.
 */

#if defined(CONFIG_OPENAMP)
#include <metal/atomic.h>
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L && \
      !defined(__STDC_NO_ATOMICS__)
#include <stdatomic.h>
#endif

#include <nuttx/net/ip.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* | <-------------- NETPKT_BUFLEN ---------------> |
 * +---------------------+-------------------+------+      +-------------+
 * | reserved for driver |       data        | free | ---> | next netpkt |
 * +---------------------+-------------------+------+      +-------------+
 * |                     | <--- datalen ---> |
 * ^base                 ^data
 */

#define NETPKT_BUFLEN   CONFIG_IOB_BUFSIZE

#if defined(CONFIG_OPENAMP) || \
    (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L && \
    !defined(__STDC_NO_ATOMICS__))
#define HAVE_ATOMIC
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* We use IOB as our buffer now, we may change to some other structure when
 * needed, so define a type netpkt_t for lower half.
 * TODO: Provide interface of its queue, maybe a simple wrapper of iob_queue.
 */

typedef struct iob_s netpkt_t;

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
struct netdev_lowerhalf_s
{
  FAR const struct netdev_ops_s *ops;

  /* Max # of buffer held by driver */

#ifdef HAVE_ATOMIC
  atomic_int quota[NETPKT_TYPENUM];
#else
  int quota[NETPKT_TYPENUM];
#endif

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
  int (*ifup)(FAR struct netdev_lowerhalf_s *dev);
  int (*ifdown)(FAR struct netdev_lowerhalf_s *dev);

  /* transmit - Try to send a packet, non-blocking, own the netpkt and
   *            need to call netpkt_free to free it sometime later.
   *   Returned Value:
   *     OK for successfully sent the packet, driver can take pkt to its
   *       own queue and return OK (remember to free it later).
   *     Negated errno value for failure, will stop current sending, the pkt
   *       will be recycled by upper half.
   */

  int (*transmit)(FAR struct netdev_lowerhalf_s *dev, FAR netpkt_t *pkt);

  /* receive - Try to receive a packet, non-blocking
   *   Returned Value:
   *     A netpkt contains the packet, or NULL if no more packets.
   */

  FAR netpkt_t *(*receive)(FAR struct netdev_lowerhalf_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
  int (*addmac)(FAR struct netdev_lowerhalf_s *dev, FAR const uint8_t *mac);
  int (*rmmac)(FAR struct netdev_lowerhalf_s *dev, FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
  int (*ioctl)(FAR struct netdev_lowerhalf_s *dev, int cmd,
               unsigned long arg);
#endif
};

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
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netdev_lower_carrier_on(FAR struct netdev_lowerhalf_s *dev);

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
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netdev_lower_carrier_off(FAR struct netdev_lowerhalf_s *dev);

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

int netdev_lower_quota_load(FAR struct netdev_lowerhalf_s *dev,
                            enum netpkt_type_e type);

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
 ****************************************************************************/

void netpkt_setdatalen(FAR struct netdev_lowerhalf_s *dev,
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

#endif /* __INCLUDE_NUTTX_NET_NETDEV_LOWERHALF_H */
