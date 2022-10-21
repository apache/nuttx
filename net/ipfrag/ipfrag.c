/****************************************************************************
 * net/ipfrag/ipfrag.c
 * Handling incoming IPv4 and IPv6 fragment input
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if (defined(CONFIG_NET_IPv4) || defined(CONFIG_NET_IPv6)) &&   \
    defined(CONFIG_NET_IPFRAG)

#include <sys/ioctl.h>
#include <stdint.h>
#include <stdlib.h>
#include <debug.h>
#include <string.h>
#include <errno.h>
#include <netinet/in.h>
#include <net/if.h>

#include <nuttx/nuttx.h>
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/ipv6ext.h>

#include "netdev/netdev.h"
#include "inet/inet.h"
#include "icmp/icmp.h"
#include "icmpv6/icmpv6.h"
#include "ipfrag.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GOTO_IF(expression, to) \
if (expression) \
  { \
    goto to;  \
  } \

#define UPDATE_IOB(iob, off, len) \
do  \
  { \
    iob->io_offset = off; \
    iob->io_len    = len; \
    iob->io_pktlen = len; \
  } while (0);  \

/* Defined the minimal timeout interval to avoid triggering timeout timer
 * too frequently, default: 0.5 seconds.
 */

#define REASSEMBLY_TIMEOUT_MINIMAL      5

#if CONFIG_NET_IPFRAG_REASS_MAXAGE < REASSEMBLY_TIMEOUT_MINIMAL
#  define REASSEMBLY_TIMEOUT            REASSEMBLY_TIMEOUT_MINIMAL
#else
#  define REASSEMBLY_TIMEOUT            CONFIG_NET_IPFRAG_REASS_MAXAGE
#endif

#define REASSEMBLY_TIMEOUT_MINIMALTICKS DSEC2TICK(REASSEMBLY_TIMEOUT_MINIMAL)
#define REASSEMBLY_TIMEOUT_TICKS        DSEC2TICK(REASSEMBLY_TIMEOUT)

#define IPFRAGWORK                      LPWORK

/* Helper macro to count I/O buffer count for a given I/O buffer chain */

#define IOBUF_CNT(ptr)    ((ptr->io_pktlen + CONFIG_IOB_BUFSIZE - 1)/ \
                          CONFIG_IOB_BUFSIZE)

/* The maximum I/O buffer occupied by fragment reassembly cache */

#define REASSEMBLY_MAXOCCUPYIOB        CONFIG_IOB_NBUFFERS / 5

/* Deciding whether to fragment outgoing packets which target is to ourself */

#define LOOPBACK_IPFRAME_NOFRAGMENT    0

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A timeout timer used to start a worker which is used to check
 * whether the assembly time of those fragments within one node is expired,
 * if so, free all resources of this node.
 */

static struct wdog_s  g_wdfragtimeout;

/* Reassembly timeout work */

static struct work_s  g_wkfragtimeout;

/* Remember the number of I/O buffers currently in reassembly cache */

static uint8_t        g_bufoccupy;

/* Queue header definition, it links all fragments of all NICs by ascending
 * ipid.
 */

static sq_queue_t     g_assemblyhead_ipid;

/* Queue header definition, which connects all fragments of all NICs in order
 * of addition time.
 */

static sq_queue_t     g_assemblyhead_time;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Only one thread can access g_assemblyhead_ipid and g_assemblyhead_time
 * at a time.
 */

sem_t                 g_ipfrag_mutex = SEM_INITIALIZER(1);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ip_fragin_timerout_expiry(wdparm_t arg);
static void ip_fragin_timerwork(FAR void *arg);
static inline FAR struct ip_fraglink_s *
ip_fragin_freelink(FAR struct ip_fraglink_s *fraglink);
static void ip_fragin_check(FAR struct ip_fragsnode_s *fragsnode);
static void ip_fragin_cachemonitor(FAR struct ip_fragsnode_s *curnode);
static inline FAR struct iob_s *
ip_fragout_allocfragbuf(FAR struct iob_queue_s *fragq);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ip_fragin_timerout_expiry
 *
 * Description:
 *   Schedule the timeout checking and handling on the low priority work
 *   queue.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ip_fragin_timerout_expiry(wdparm_t arg)
{
  assert(g_wkfragtimeout.worker == NULL);
  work_queue(IPFRAGWORK, &g_wkfragtimeout, ip_fragin_timerwork, NULL, 0);
}

/****************************************************************************
 * Name: ip_fragin_timerwork
 *
 * Description:
 *   The really work of fragment timeout checking and handling.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ip_fragin_timerwork(FAR void *arg)
{
  clock_t         curtick = clock_systime_ticks();
  sclock_t        interval;
  FAR sq_entry_t *entry;
  FAR sq_entry_t *entrynext;
  FAR struct ip_fragsnode_s *node;

  ninfo("Start reassembly work queue\n");

  nxsem_wait_uninterruptible(&g_ipfrag_mutex);

  /* Walk through the list, check the timetout and calculate the next timer
   * interval
   */

  entry = sq_peek(&g_assemblyhead_time);
  while (entry)
    {
      entrynext = sq_next(entry);

      node = (FAR struct ip_fragsnode_s *)
             container_of(entry, FAR struct ip_fragsnode_s, flinkat);

      /* Check for timeout, be careful with the calculation formula,
       * the tick counter may overflow
       */

      interval = curtick - node->tick;

      if (interval >= REASSEMBLY_TIMEOUT_TICKS)
        {
          /* If this timeout expires, the partially-reassembled datagram
           * MUST be discarded and an ICMP Time Exceeded message sent to
           * the source host (if fragment zero has been received).
           */

          ninfo("Reassembly timeout occurs!");
#if defined(CONFIG_NET_ICMP) && !defined(CONFIG_NET_ICMP_NO_STACK)
          if (node->verifyflag & IP_FRAGVERIFY_RECVDZEROFRAG)
            {
              FAR struct net_driver_s *dev = node->dev;

              net_lock();

              netdev_iob_replace(dev, node->frags->frag);
              node->frags->frag = NULL;

#ifdef CONFIG_NET_IPv4
              if (node->frags->isipv4)
                {
                  icmp_reply(dev, ICMP_TIME_EXCEEDED,
                            ICMP_EXC_FRAGTIME);
                }
#endif

#ifdef CONFIG_NET_IPv6
              if (!node->frags->isipv4)
                {
                  icmpv6_reply(dev, ICMPv6_PACKET_TIME_EXCEEDED,
                              ICMPV6_EXC_FRAGTIME, 0);
                }
#endif

              if (iob_tryadd_queue(dev->d_iob, &dev->d_fragout) == 0)
                {
                  netdev_iob_clear(dev);

                  /* Send ICMP Time Exceeded message via dev->d_fragout
                   * queue
                   */

                  ninfo("Send Time Exceeded ICMP%s Message to source "
                        "host\n", node->frags->isipv4 ? "v4" : "v6");
                  netdev_txnotify_dev(dev);
                }

              net_unlock();
            }
#endif

          /* Remove fragments of this node */

          if (node->frags != NULL)
            {
              FAR struct ip_fraglink_s *fraglink = node->frags;

              while (fraglink)
                {
                  fraglink = ip_fragin_freelink(fraglink);
                }
            }

          /* Remove node from single-list and free node memory */

          ip_frag_remnode(node);
          kmm_free(node);
        }
      else
        {
          /* Because fragment nodes have been sorted to g_assemblyhead_time
           * according to the added time, so enter here, we can get the
           * 'interval' of the earliest time node that has not timed out.
           * There is no need to continue the loop here, and use time
           * REASSEMBLY_TIMEOUT_TICKS - 'interval' as the input for the next
           * Timer starting.
           */

          break;
        }

      entry = entrynext;
    }

  /* Be sure to start the timer, if there are nodes in the linked list */

  if (sq_peek(&g_assemblyhead_time) != NULL)
    {
      clock_t delay = REASSEMBLY_TIMEOUT_MINIMALTICKS;

      /* The interval for the next timer is REASSEMBLY_TIMEOUT_TICKS -
       * interval, if it is less than the minimum timeout interval,
       * fix it to REASSEMBLY_TIMEOUT_MINIMALTICKS
       */

      if (delay < REASSEMBLY_TIMEOUT_TICKS - interval)
        {
          delay = REASSEMBLY_TIMEOUT_TICKS - interval;
        }

      ninfo("Reschedule reassembly work queue\n");
      wd_start(&g_wdfragtimeout, delay, ip_fragin_timerout_expiry,
              (wdparm_t)NULL);
    }
  else
    {
      ninfo("Stop reassembly work queue\n");
    }

  nxsem_post(&g_ipfrag_mutex);
}

/****************************************************************************
 * Name: ip_fragin_freelink
 *
 * Description:
 *   Free the I/O buffer and ip_fraglink_s buffer at the head of a
 *   ip_fraglink_s chain.
 *
 * Returned Value:
 *   The link to the next ip_fraglink_s buffer in the chain.
 *
 ****************************************************************************/

static inline FAR struct ip_fraglink_s *
ip_fragin_freelink(FAR struct ip_fraglink_s *fraglink)
{
  FAR struct ip_fraglink_s *next = fraglink->flink;

  if (fraglink->frag != NULL)
    {
      iob_free_chain(fraglink->frag);
    }

  kmm_free(fraglink);

  return next;
}

/****************************************************************************
 * Name: ip_fragin_check
 *
 * Description:
 *   Audit whether the fragment zero has been received or all fragments have
 *   been received.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ip_fragin_check(FAR struct ip_fragsnode_s *fragsnode)
{
  uint16_t formerlen = 0;
  FAR struct ip_fraglink_s *entry;

  if (fragsnode->verifyflag & IP_FRAGVERIFY_RECVDTAILFRAG)
    {
      entry = fragsnode->frags;
      while (entry)
        {
          if (entry->morefrags)
            {
              formerlen += entry->fraglen;
            }
          else
            {
              /* Only the last entry has a 0 morefrags flag */

              if (entry->fragoff == formerlen)
                {
                  fragsnode->verifyflag |= IP_FRAGVERIFY_RECVDALLFRAGS;
                }
            }

          entry = entry->flink;
        }
    }
}

/****************************************************************************
 * Name: ip_fragin_cachemonitor
 *
 * Description:
 *   Check the reassembly cache buffer size, if it exceeds the configured
 *   threshold, some I/O buffers need to be freed
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void ip_fragin_cachemonitor(FAR struct ip_fragsnode_s *curnode)
{
  uint32_t        cleancnt = 0;
  uint32_t        bufcnt;
  FAR sq_entry_t *entry;
  FAR sq_entry_t *entrynext;
  FAR struct ip_fragsnode_s *node;

  /* Start cache cleaning if g_bufoccupy exceeds the cache threshold */

  if (g_bufoccupy > REASSEMBLY_MAXOCCUPYIOB)
    {
      cleancnt = g_bufoccupy - REASSEMBLY_MAXOCCUPYIOB;
      entry = sq_peek(&g_assemblyhead_time);

      while (entry && cleancnt > 0)
        {
          entrynext = sq_next(entry);

          node = (FAR struct ip_fragsnode_s *)
                 container_of(entry, FAR struct ip_fragsnode_s, flinkat);

          /* Skip specified node */

          if (node != curnode)
            {
              /* Remove fragments of this node */

              if (node->frags != NULL)
                {
                  FAR struct ip_fraglink_s *fraglink = node->frags;

                  while (fraglink)
                    {
                      fraglink = ip_fragin_freelink(fraglink);
                    }
                }

              /* Remove node from single-list and free node memory */

              bufcnt = ip_frag_remnode(node);
              kmm_free(node);

              cleancnt = cleancnt > bufcnt ? cleancnt - bufcnt : 0;
            }

          entry = entrynext;
        }
    }
}

/****************************************************************************
 * Name: ip_fragout_allocfragbuf
 *
 * Description:
 *   Prepare one I/O buffer and enqueue it to a specified queue
 *
 * Returned Value:
 *   The pointer to I/O buffer
 *
 ****************************************************************************/

static inline FAR struct iob_s *
ip_fragout_allocfragbuf(FAR struct iob_queue_s *fragq)
{
  FAR struct iob_s *iob;

  iob = iob_tryalloc(false);
  if (iob != NULL)
    {
      if (iob_tryadd_queue(iob, fragq) < 0)
        {
          iob_free(iob);
          iob = NULL;
        }
    }

  return iob;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ip_frag_remnode
 *
 * Description:
 *   free ip_fragsnode_s
 *
 * Returned Value:
 *   I/O buffer count of this node
 *
 ****************************************************************************/

uint32_t ip_frag_remnode(FAR struct ip_fragsnode_s *node)
{
  g_bufoccupy -= node->bufcnt;
  assert(g_bufoccupy < CONFIG_IOB_NBUFFERS);

  sq_rem((FAR sq_entry_t *)node, &g_assemblyhead_ipid);
  sq_rem((FAR sq_entry_t *)&node->flinkat, &g_assemblyhead_time);

  return node->bufcnt;
}

/****************************************************************************
 * Name: ip_fragin_enqueue
 *
 * Description:
 *   Enqueue one fragment.
 *   All fragments belonging to one IP frame are organized in a linked list
 *   form, that is a ip_fragsnode_s node. All ip_fragsnode_s nodes are also
 *   organized in an upper-level linked list.
 *
 * Returned Value:
 *   Whether queue is empty before enqueue the new node
 *
 ****************************************************************************/

bool ip_fragin_enqueue(FAR struct net_driver_s *dev,
                       FAR struct ip_fraglink_s *curfraglink)
{
  FAR struct ip_fragsnode_s *node;
  FAR sq_entry_t         *entry;
  FAR sq_entry_t         *entrylast = NULL;
  bool                    empty;

  /* The linked list is ordered by IPID value, walk through it and try to
   * find a node that has the same IPID value, otherwise need to create a
   * new node and insert it into the linked list.
   */

  entry = sq_peek(&g_assemblyhead_ipid);
  empty = (entry == NULL) ? true : false;

  while (entry)
    {
      node = (struct ip_fragsnode_s *)entry;

      if (dev == node->dev && curfraglink->ipid <= node->ipid)
        {
          break;
        }

      entrylast = entry;
      entry = sq_next(entry);
    }

  node = (struct ip_fragsnode_s *)entry;

  if (node != NULL && curfraglink->ipid == node->ipid)
    {
      FAR struct ip_fraglink_s *fraglink;
      FAR struct ip_fraglink_s *lastlink = NULL;

      /* Found a previously created ip_fragsnode_s, insert this new
       * ip_fraglink_s to the subchain of this node.
       */

      fraglink = node->frags;

      /* A ip_fragsnode_s must have a ip_fraglink_s because we allocate a new
       * ip_fraglink_s when caching a new ip_fraglink_s with a new IPID
       */

      while (fraglink)
        {
          /* The fragment list is ordered by fragment offset value */

          if (curfraglink->fragoff <= fraglink->fragoff)
            {
              break;
            }

          lastlink = fraglink;
          fraglink = fraglink->flink;
        }

      if (fraglink == NULL)
        {
          /* This fragment offset is greater than the previous fragments,
           * added to the last position
           */

          lastlink->flink      = curfraglink;

          /* Remember I/O buffer count */

          node->bufcnt += IOBUF_CNT(curfraglink->frag);
          g_bufoccupy  += IOBUF_CNT(curfraglink->frag);
        }
      else if (curfraglink->fragoff == fraglink->fragoff)
        {
          /* Fragments with same offset value contain the same data, use the
           * more recently arrived copy. Refer to RFC791, Section3.2, Page29.
           * Replace and removed the old packet from the fragment list
           */

          curfraglink->flink = fraglink->flink;
          if (lastlink == NULL)
            {
              node->frags = curfraglink;
            }
          else
            {
              lastlink->flink = curfraglink;
            }

          iob_free_chain(fraglink->frag);
          kmm_free(fraglink);
        }
      else
        {
          /* Insert into the fragment list */

          if (lastlink == NULL)
            {
              /* Insert before the first node */

              curfraglink->flink = node->frags;
              node->frags = curfraglink;
            }
          else
            {
              /* Insert this node after lastlink */

              curfraglink->flink = lastlink->flink;
              lastlink->flink = curfraglink;
            }

          /* Remember I/O buffer count */

          node->bufcnt += IOBUF_CNT(curfraglink->frag);
          g_bufoccupy  += IOBUF_CNT(curfraglink->frag);
        }
    }
  else
    {
      /* It's a new IPID fragment, malloc a new node and insert it into the
       * linked list
       */

      node = kmm_malloc(sizeof(struct ip_fragsnode_s));
      if (node == NULL)
        {
          nerr("ERROR: Failed to allocate buffer.\n");
          return -ENOMEM;
        }

      node->flink      = NULL;
      node->flinkat    = NULL;
      node->dev        = dev;
      node->ipid       = curfraglink->ipid;
      node->frags      = curfraglink;
      node->tick       = clock_systime_ticks();
      node->bufcnt     = IOBUF_CNT(curfraglink->frag);
      g_bufoccupy     += IOBUF_CNT(curfraglink->frag);
      node->verifyflag = 0;
      node->outgoframe = NULL;

      /* Insert this new node into linked list identified by
       * g_assemblyhead_ipid with correct position
       */

      if (sq_peek(&g_assemblyhead_ipid) == NULL || entrylast == NULL)
        {
          sq_addfirst((FAR sq_entry_t *)node, &g_assemblyhead_ipid);
        }
      else
        {
          sq_addafter(entrylast, (FAR sq_entry_t *)node,
                      &g_assemblyhead_ipid);
        }

      /* Add this new node to the tail of linked list identified by
       * g_assemblyhead_time
       */

      sq_addlast((FAR sq_entry_t *)&node->flinkat, &g_assemblyhead_time);
    }

  if (curfraglink->fragoff == 0)
    {
      /* Have received the zero fragment */

      node->verifyflag |= IP_FRAGVERIFY_RECVDZEROFRAG;
    }
  else if (!curfraglink->morefrags)
    {
      /* Have received the tail fragment */

      node->verifyflag |= IP_FRAGVERIFY_RECVDTAILFRAG;
    }

  /* For indexing convenience */

  curfraglink->fragsnode = node;

  /* Check receiving status */

  ip_fragin_check(node);

  /* Buffer is take away, clear original pointers in NIC */

  netdev_iob_clear(dev);

  /* Perform cache cleaning when reassembly cache size exceeds the configured
   * threshold
   */

  ip_fragin_cachemonitor(node);

  return empty;
}

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
 *   iob    - The data comes from
 *   domain - PF_INET or PF_INET6
 *   mtu    - MTU of current NIC
 *   unfraglen - The starting position to fragmentation processing
 *   fragq  - Those output slices
 *
 * Returned Value:
 *   Number of fragments
 *
 * Assumptions:
 *   Data length(iob->io_pktlen) is grater than the MTU of current NIC
 *
 ****************************************************************************/

int32_t ip_fragout_slice(FAR struct iob_s *iob, uint8_t domain, uint16_t mtu,
                         uint16_t unfraglen, FAR struct iob_queue_s *fragq)
{
  FAR uint8_t *leftstart;
  uint16_t     leftlen = 0;
  uint16_t     ncopy;
  uint16_t     navail;
  uint32_t     nfrags = 0;
  bool         expand = false;
  FAR struct iob_s *orig = NULL;
  FAR struct iob_s *reorg = NULL;
  FAR struct iob_s *head = NULL;

  if (iob == NULL || fragq == NULL)
    {
      nerr("ERROR: Invalid parameters! iob: %p, fragq: %p\n", iob, fragq);
      return 0;
    }

  assert(iob->io_pktlen > mtu);

#ifdef CONFIG_NET_IPv4
  if (domain == PF_INET)
    {
      uint16_t nreside;

      /* Fragmentation requires that the data length after the IP header
       * must be a multiple of 8
       */

      mtu = ((mtu - IPv4_HDRLEN) >> 3 << 3) + IPv4_HDRLEN;

      /* Remember the number of resident bytes */

      nreside = mtu;

      /* For IPv4, fragmented frames and non-fragmented frames have the
       * same length L3 header. So process it as follows:
       * the zero fragment use the original I/O buffer and reorganize
       * the non-zero fragments (copy to new I/O buffers), space for the
       * L3 IP header must be reserved for all fragments
       */

      head = iob;
      while (iob != NULL && nreside > iob->io_len)
        {
          nreside -= iob->io_len;
          iob      = iob->io_flink;
        }

      leftstart = iob->io_data + iob->io_offset + nreside;
      leftlen   = iob->io_len - nreside;

      orig = iob->io_flink;
      if (orig != NULL)
        {
          orig->io_pktlen = head->io_pktlen - (mtu + leftlen);
          iob->io_flink   = NULL;
        }

      head->io_pktlen = mtu;
      iob->io_len     = nreside;

      if (iob_tryadd_queue(head, fragq) < 0)
        {
          goto allocfail;
        }

      head = NULL;
      nfrags++;

      if (leftlen == 0 && orig != NULL)
        {
          reorg = ip_fragout_allocfragbuf(fragq);
          GOTO_IF(reorg == NULL, allocfail);

          nfrags++;

          /* This is a new fragment buffer, reserve L2&L3 header space
           * in the front of this buffer
           */

          UPDATE_IOB(reorg, CONFIG_NET_LL_GUARDSIZE, unfraglen);
        }
      else
        {
          /* If the MTU is relatively small, the remaining data of the first
           * I/O buffer may need to be fragmented multiple times.
           * For IPv4, the first I/O Buffer is reused, which have reserved
           * the L4 header space, the following fragmentation flow is only
           * for non-zero fragments, so following flow does not need to
           * consider the L4 header
           */

          while (leftlen > 0)
            {
              reorg = ip_fragout_allocfragbuf(fragq);
              GOTO_IF(reorg == NULL, allocfail);

              nfrags++;

              if (leftlen + unfraglen > mtu)
                {
                  ncopy = mtu - unfraglen;
                }
              else
                {
                  ncopy = leftlen;
                }

              /* This is a new fragment buffer, reserve L2&L3 header space
               * in the front of this buffer
               */

              UPDATE_IOB(reorg, CONFIG_NET_LL_GUARDSIZE, unfraglen);

              /* Then copy L4 data */

              GOTO_IF(iob_trycopyin(reorg, leftstart, ncopy,
                    reorg->io_pktlen, false) < 0, allocfail);

              leftstart     += ncopy;
              leftlen       -= ncopy;
            }
        }
    }
#ifdef CONFIG_NET_IPv6
  else
#endif
#endif
#ifdef CONFIG_NET_IPv6
  if (domain == PF_INET6)
    {
      unfraglen += EXTHDR_FRAG_LEN;

      /* Fragmentation requires the length field to be a multiples of 8,
       * and the length of the IPv6 basic header and all extended headers
       * is a multiples of 8, so here directly fix the MTU to 8-byte
       * alignment.
       */

      mtu = mtu >> 3 << 3;

      /* For IPv6 fragment, a fragment header needs to be inserted before
       * the l4 header, so all data must be reorganized, a space for IPv6
       * header and fragment external header is reserved before l4 header
       * for all fragments
       */

      reorg = ip_fragout_allocfragbuf(fragq);
      GOTO_IF(reorg == NULL, allocfail);

      nfrags++;

      /* Reserve L3 header space */

      UPDATE_IOB(reorg, CONFIG_NET_LL_GUARDSIZE, unfraglen);

      /* Copy L3 header(include unfragmentable extention header if present)
       * from original I/O buffer
       */

      orig = iob;
      memcpy(reorg->io_data + reorg->io_offset,
             orig->io_data + orig->io_offset, unfraglen - EXTHDR_FRAG_LEN);
      iob_trimhead(orig, unfraglen - EXTHDR_FRAG_LEN);
    }
#endif

  /* Copy data from original I/O buffer chain 'orig' to new reorganized
   * I/O buffer chain 'reorg'
   */

  while (orig)
    {
      leftstart = orig->io_data + orig->io_offset;
      leftlen = orig->io_len;

      /* For each I/O buffer data of the 'orig' chain */

      while (leftlen > 0)
        {
          /* Calculate target area size */

          navail = mtu - reorg->io_pktlen;

          if (navail > 0)
            {
              if (leftlen > navail)
                {
                  /* Target area is too small, need expand the destination
                   * chain
                   */

                  expand = true;
                  ncopy  = navail;
                }
              else
                {
                  ncopy = leftlen;
                }

              if (iob_trycopyin(reorg, leftstart, ncopy,
                                reorg->io_pktlen, false) < 0)
                {
                  goto allocfail;
                }

              leftlen   -= ncopy;
              leftstart += ncopy;
            }
          else
            {
              expand = true;
            }

          if (expand)
            {
              reorg = ip_fragout_allocfragbuf(fragq);
              GOTO_IF(reorg == NULL, allocfail);

              nfrags++;

              /* This is a new fragment buffer, reserve L2&L3 header space
               * in the front of this buffer
               */

              UPDATE_IOB(reorg, CONFIG_NET_LL_GUARDSIZE, unfraglen);

              expand = false;
            }
        }

      orig = iob_free(orig);
    }

  return nfrags;

allocfail:
  nerr("ERROR: Fragout fail! No I/O buffer available!");
  iob_free_chain(head);
  iob_free_chain(orig);
  iob_free_chain(reorg);
  iob_free_queue(fragq);

  return 0;
}

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

void ip_frag_startwdog(void)
{
  if (g_wdfragtimeout.func == NULL)
    {
      wd_start(&g_wdfragtimeout, REASSEMBLY_TIMEOUT_TICKS,
               ip_fragin_timerout_expiry, (wdparm_t)NULL);
    }
}

/****************************************************************************
 * Name: ip_frag_uninit
 *
 * Description:
 *   Uninitialize the fragment processing module.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int32_t ip_frag_uninit(void)
{
  FAR struct net_driver_s *dev;

  ninfo("Uninitialize frag proccessing module\n");

  /* Stop work queue */

  if (!work_available(&g_wkfragtimeout))
    {
      ninfo("Cancel reassembly work queue\n");
      work_cancel(IPFRAGWORK, &g_wkfragtimeout);
    }

  /* Release frag processing resources of each NIC */

  net_lock();
  for (dev = g_netdevices; dev; dev = dev->flink)
    {
      /* Is the interface in the "up" state? */

      if ((dev->d_flags & IFF_UP) != 0)
        {
          ip_frag_stop(dev);
        }
    }

  net_unlock();

  return OK;
}

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

void ip_frag_stop(FAR struct net_driver_s *dev)
{
  FAR sq_entry_t *entry = NULL;
  FAR sq_entry_t *entrynext;

  ninfo("Stop frag processing for NIC:%p\n", dev);

  nxsem_wait_uninterruptible(&g_ipfrag_mutex);

  entry = sq_peek(&g_assemblyhead_ipid);

  /* Drop those unassembled incoming fragments belonging to this NIC */

  while (entry)
    {
      FAR struct ip_fragsnode_s *node = (FAR struct ip_fragsnode_s *)entry;
      entrynext = sq_next(entry);

      if (dev == node->dev)
        {
          if (node->frags != NULL)
            {
              FAR struct ip_fraglink_s *fraglink = node->frags;

              while (fraglink)
                {
                  fraglink = ip_fragin_freelink(fraglink);
                }
            }

          ip_frag_remnode(node);
          kmm_free(entry);
        }

      entry = entrynext;
    }

  nxsem_post(&g_ipfrag_mutex);

  /* Drop those unsent outgoing fragments belonging to this NIC */

  iob_free_queue(&dev->d_fragout);
}

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

void ip_frag_remallfrags(void)
{
  FAR sq_entry_t *entry = NULL;
  FAR sq_entry_t *entrynext;
  FAR struct net_driver_s *dev;

  nxsem_wait_uninterruptible(&g_ipfrag_mutex);

  entry = sq_peek(&g_assemblyhead_ipid);

  /* Drop all unassembled incoming fragments */

  while (entry)
    {
      FAR struct ip_fragsnode_s *node = (FAR struct ip_fragsnode_s *)entry;
      entrynext = sq_next(entry);

      if (node->frags != NULL)
        {
          FAR struct ip_fraglink_s *fraglink = node->frags;

          while (fraglink)
            {
              fraglink = ip_fragin_freelink(fraglink);
            }
        }

      /* Because nodes managed by the two queues are the same,
       * and g_assemblyhead_ipid will be cleared after this loop ends,
       * so only reset g_assemblyhead_time is needed after this loop ends
       */

      sq_rem(entry, &g_assemblyhead_ipid);
      kmm_free(entry);

      entry = entrynext;
    }

  sq_init(&g_assemblyhead_time);
  g_bufoccupy = 0;

  nxsem_post(&g_ipfrag_mutex);

  /* Drop all unsent outgoing fragments */

  net_lock();
  for (dev = g_netdevices; dev; dev = dev->flink)
    {
      /* Is the interface in the "up" state? */

      if ((dev->d_flags & IFF_UP) != 0)
        {
          iob_free_queue(&dev->d_fragout);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: ip_fragout
 *
 * Description:
 *   Fragout processing
 *
 * Input Parameters:
 *   dev    - The NIC device
 *
 ****************************************************************************/

int32_t ip_fragout(FAR struct net_driver_s *dev)
{
  uint16_t mtu = dev->d_pktsize - dev->d_llhdrlen;

  if (dev->d_iob == NULL)
    {
      return -EINVAL;
    }

  if (dev->d_iob->io_pktlen <= mtu)
    {
      return OK;
    }

#ifdef CONFIG_NET_6LOWPAN
  if (dev->d_lltype == NET_LL_IEEE802154 ||
      dev->d_lltype == NET_LL_PKTRADIO)
    {
      return -EINVAL;
    }
#endif

  if (devif_is_loopback(dev))
    {
      return OK;
    }

  ninfo("pkt size: %d, MTU: %d\n", dev->d_iob->io_pktlen, mtu);

#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv4(dev->d_flags))
    {
      return ipv4_fragout(dev, mtu);
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (IFF_IS_IPv6(dev->d_flags))
    {
      return ipv6_fragout(dev, mtu);
    }
#endif

  return -EINVAL;
}

#endif /* (CONFIG_NET_IPv4 || CONFIG_NET_IPv6) && CONFIG_NET_IPFRAG */
