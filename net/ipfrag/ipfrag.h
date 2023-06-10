/****************************************************************************
 * net/ipfrag/ipfrag.h
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

#ifndef __NET_IPFRAG_IPFRAG_H
#define __NET_IPFRAG_IPFRAG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>

#include <nuttx/mutex.h>
#include <nuttx/queue.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"

#if defined(CONFIG_NET_IPFRAG)

/****************************************************************************
 * Public types
 ****************************************************************************/

enum ip_fragverify_e
{
  /* Indicates whether received all fragments */

  IP_FRAGVERIFY_RECVDALLFRAGS  = 0x01 << 0,

  /* Indicates whether received the first fragment which is used to:
   * 1.construct the ICMP time exceeded msg(type=11, code=1) when reassembly
   *   timeout, but if the first fragment has not been received when timeout,
   *   no ICMP error message will be sent;
   * 2.build NAT entry with the L4 port number and do forwarding.
   */

  IP_FRAGVERIFY_RECVDZEROFRAG  = 0x01 << 1,

  /* Indicates whether the tail fragment is received(which morefrag flag is
   * set to 0)
   */

  IP_FRAGVERIFY_RECVDTAILFRAG  = 0x01 << 2,
};

struct ip_fraglink_s
{
  /* This link is used to maintain a single-linked list of ip_fraglink_s,
   * it links all framgents with the same IP ID
   */

  FAR struct ip_fraglink_s  *flink;

  FAR struct ip_fragsnode_s *fragsnode; /* Point to parent struct */
  FAR struct iob_s          *frag;      /* Point to fragment data */
  uint8_t                    isipv4;    /* IPv4 or IPv6 */
  uint16_t                   fragoff;   /* Fragment offset */
  uint16_t                   fraglen;   /* Payload length */
  uint16_t                   morefrags; /* The more frag flag */

  /* The identification field is 16 bits in IPv4 header but 32 bits in IPv6
   * fragment header
   */

  uint32_t                   ipid;
};

struct ip_fragsnode_s
{
  /* This link is used to maintain a single-linked list of ip_fragsnode_s.
   * Must be the first field in the structure due to flink type casting.
   */

  FAR struct ip_fragsnode_s *flink;

  /* Another link which connects all ip_fragsnode_s in order of addition
   * time
   */

  FAR sq_entry_t            *flinkat;

  /* Interface understood by the network */

  FAR struct net_driver_s   *dev;

  /* IP Identification (IP ID) field defined in ipv4 header or in ipv6
   * fragment header.
   */

  uint32_t                   ipid;

  /* Count ticks, used by ressembly timer */

  clock_t                    tick;

  /* Remember some running flags */

  uint16_t                   verifyflag;

  /* Remember the total number of I/O buffers of this node */

  uint32_t                   bufcnt;

  /* Linked all fragments with the same IP ID. */

  FAR struct ip_fraglink_s  *frags;

  /* Points to the reassembled outgoing IP frame */

  FAR struct iob_s          *outgoframe;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/* Only one thread can access g_assemblyhead_ipid and g_assemblyhead_time
 * at a time
 */

extern mutex_t g_ipfrag_lock;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ip_frag_remnode
 *
 * Description:
 *   free ip_fragsnode_s
 *
 * Input Parameters:
 *   node - node of the upper-level linked list, it maintains
 *          information about all fragments belonging to an IP datagram
 *
 * Returned Value:
 *   I/O buffer count of this node
 *
 ****************************************************************************/

uint32_t ip_frag_remnode(FAR struct ip_fragsnode_s *node);

/****************************************************************************
 * Name: ip_fragin_enqueue
 *
 * Description:
 *   Enqueue one fragment.
 *   All fragments belonging to one IP frame are organized in a linked list
 *   form, that is a ip_fragsnode_s node. All ip_fragsnode_s nodes are also
 *   organized in an upper-level linked list.
 *
 * Input Parameters:
 *   dev         - NIC Device instance
 *   curfraglink - node of the lower-level linked list, it maintains
 *                 information of one fragment
 *
 * Returned Value:
 *   Whether queue is empty before enqueue the new node
 *
 ****************************************************************************/

bool ip_fragin_enqueue(FAR struct net_driver_s *dev,
                       FAR struct ip_fraglink_s *curfraglink);

/****************************************************************************
 * Name: ipv4_fragin
 *
 * Description:
 *   Handling incoming IPv4 fragment input, the input data
 *   (dev->d_iob) can be an I/O buffer chain
 *
 * Input Parameters:
 *   dev    - The NIC device that the fragmented data comes from
 *
 * Returned Value:
 *   ENOMEM - No memory
 *   OK     - The input fragment is processed as expected
 *
 ****************************************************************************/

int32_t ipv4_fragin(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: ipv6_fragin
 *
 * Description:
 *   Handling incoming IPv6 fragment input, the input data
 *   (dev->d_iob) can be an I/O buffer chain
 *
 * Input Parameters:
 *   dev    - The NIC device that the fragmented data comes from
 *
 * Returned Value:
 *   ENOMEM - No memory
 *   OK     - The input fragment is processed as expected
 *
 ****************************************************************************/

int32_t ipv6_fragin(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: ip_fragout_slice
 *
 * Description:
 *  According to the MTU of a given NIC, split the original data into
 *  multiple data pieces, and the space for filling the L3 header is
 *  reserved at the forefront of each piece. Each piece is stored in
 *  independent I/O buffer(s) and eventually forms an I/O buffer queue.
 *  Note:
 *  1.About the 'piece' above
 *    1).If MTU < CONFIG_IOB_BUFSIZE, a piece consists of an I/O buffer;
 *    2).If MTU >= CONFIG_IOB_BUFSIZE, a piece consists of multiple I/O
 *       buffers.
 *  2.This function split and gathers the incoming data into outgoing
 *  I/O buffers according to the MTU, but is not responsible for
 *  building the L3 header related to the fragmentation.
 *
 * Input Parameters:
 *   iob       - The data comes from
 *   domain    - PF_INET or PF_INET6
 *   mtu       - MTU of given NIC
 *   unfraglen - The starting position to fragmentation processing
 *   fragq     - Those output slices
 *
 * Returned Value:
 *   Number of fragments
 *
 * Assumptions:
 *   Data length(iob->io_pktlen) is grater than the MTU of current NIC
 *
 ****************************************************************************/

int32_t ip_fragout_slice(FAR struct iob_s *iob, uint8_t domain, uint16_t mtu,
                         uint16_t unfraglen, FAR struct iob_queue_s *fragq);

/****************************************************************************
 * Name: ipv4_fragout
 *
 * Description:
 *   Execute the ipv4 fragment function. After this work is done, all
 *   fragments are maintained by dev->d_fragout. In order to reduce the
 *   cyclomatic complexity and facilitate maintenance, fragmentation is
 *   performed in two steps:
 *   1. Reconstruct I/O Buffer according to MTU, which will reserve
 *      the space for the L3 header;
 *   2. Fill the L3 header into the reserved space.
 *
 * Input Parameters:
 *   dev    - The NIC device
 *   mtu    - The MTU of current NIC
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 * Assumptions:
 *   Data length(dev->d_iob->io_pktlen) is grater than the MTU of
 *   current NIC
 *
 ****************************************************************************/

int32_t ipv4_fragout(FAR struct net_driver_s *dev, uint16_t mtu);

/****************************************************************************
 * Name: ipv6_fragout
 *
 * Description:
 *   Execute the ipv6 fragment function. After this work is done, all
 *   fragments are maintained by dev->d_fragout. In order to reduce the
 *   cyclomatic complexity and facilitate maintenance, fragmentation is
 *   performed in two steps:
 *   1. Reconstruct I/O Buffer according to MTU, which will reserve
 *      the space for the L3 header;
 *   2. Fill the L3 header into the reserved space.
 *
 * Input Parameters:
 *   dev    - The NIC device
 *   mtu    - The MTU of current NIC
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 * Assumptions:
 *   Data length(dev->d_iob->io_pktlen) is grater than the MTU of
 *   current NIC
 *
 ****************************************************************************/

int32_t ipv6_fragout(FAR struct net_driver_s *dev, uint16_t mtu);

/****************************************************************************
 * Name: ip_frag_startwdog
 *
 * Description:
 *   Start the reassembly timeout timer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ip_frag_startwdog(void);

/****************************************************************************
 * Name: ip_frag_stop
 *
 * Description:
 *   Stop the fragment process function for the specified NIC.
 *
 * Input Parameters:
 *   dev    - NIC Device instance which will be bring down
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ip_frag_stop(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: ip_frag_remallfrags
 *
 * Description:
 *   Release all I/O Buffers used by fragment processing module when
 *   I/O Buffer resources are exhausted.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ip_frag_remallfrags(void);

/****************************************************************************
 * Name: ip_fragout
 *
 * Description:
 *   Fragout processing
 *
 * Input Parameters:
 *   dev    - The NIC device
 *
 * Returned Value:
 *   A non-negative value is returned on success; negative value on failure.
 *
 ****************************************************************************/

int32_t ip_fragout(FAR struct net_driver_s *dev);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_IPFRAG */
#endif /* __NET_IPFRAG_IPFRAG_H */
