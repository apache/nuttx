/****************************************************************************
 * include/nuttx/mm/iob.h
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

#ifndef _INCLUDE_NUTTX_MM_IOB_H
#define _INCLUDE_NUTTX_MM_IOB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_IOB_NOTIFIER
#  include <nuttx/wqueue.h>
#endif

#ifdef CONFIG_MM_IOB

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* I/O buffer allocation logic supports a throttle value for read-ahead
 * buffering to prevent the read-ahead logic from consuming all available I/O
 * buffers and blocking the write buffering logic.  This throttle logic
 * is only needed if both write buffering and read-ahead buffering are used.
 *
 * The correct way to disable throttling is to set the throttle value to
 * zero.
 */

#if !defined(CONFIG_IOB_THROTTLE)
#  define CONFIG_IOB_THROTTLE 0
#endif

/* Some I/O buffers should be allocated */

#if !defined(CONFIG_IOB_NBUFFERS)
#  warning CONFIG_IOB_NBUFFERS not defined
#  define CONFIG_IOB_NBUFFERS 0
#endif

#if CONFIG_IOB_NBUFFERS < 1
#  error CONFIG_IOB_NBUFFERS is zero
#endif

#if CONFIG_IOB_NBUFFERS <= CONFIG_IOB_THROTTLE
#  error CONFIG_IOB_NBUFFERS <= CONFIG_IOB_THROTTLE
#endif

/* IOB helpers */

#define IOB_DATA(p)      (&(p)->io_data[(p)->io_offset])
#define IOB_FREESPACE(p) (CONFIG_IOB_BUFSIZE - (p)->io_len - (p)->io_offset)

#if CONFIG_IOB_NCHAINS > 0
/* Queue helpers */

#  define IOB_QINIT(q)   do { (q)->qh_head = 0; (q)->qh_tail = 0; } while (0)
#  define IOB_QEMPTY(q)  ((q)->qh_head == NULL)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Represents one I/O buffer.  A packet is contained by one or more I/O
 * buffers in a chain.  The io_pktlen is only valid for the I/O buffer at
 * the head of the chain.
 */

struct iob_s
{
  /* Singly-link list support */

  FAR struct iob_s *io_flink;

  /* Payload */

#if CONFIG_IOB_BUFSIZE < 256
  uint8_t  io_len;      /* Length of the data in the entry */
  uint8_t  io_offset;   /* Data begins at this offset */
#else
  uint16_t io_len;      /* Length of the data in the entry */
  uint16_t io_offset;   /* Data begins at this offset */
#endif
  uint16_t io_pktlen;   /* Total length of the packet */

  uint8_t  io_data[CONFIG_IOB_BUFSIZE];
};

#if CONFIG_IOB_NCHAINS > 0
/* This container structure supports queuing of I/O buffer chains.  This
 * structure is intended only for internal use by the IOB module.
 */

struct iob_qentry_s
{
  /* Singly-link list support */

  FAR struct iob_qentry_s *qe_flink;

  /* Payload -- Head of the I/O buffer chain */

  FAR struct iob_s *qe_head;
};

/* The I/O buffer queue head structure */

struct iob_queue_s
{
  /* Head of the I/O buffer chain list */

  FAR struct iob_qentry_s *qh_head;
  FAR struct iob_qentry_s *qh_tail;
};
#endif /* CONFIG_IOB_NCHAINS > 0 */

/* NOTE: When you change any logic here, you must change the logic in
 * fs/procfs/fs_procfsiobinfo.c as it depends on having matching sequential
 * logic.
 */

enum iob_user_e
{
  IOBUSER_UNKNOWN = -1,
#ifdef CONFIG_SYSLOG_BUFFER
  IOBUSER_SYSLOG,
#endif
#ifdef CONFIG_IOB_UNITTEST
  IOBUSER_UNITTEST,
#endif
#ifdef CONFIG_NET_6LOWPAN
  IOBUSER_NET_6LOWPAN,
#endif
#ifdef CONFIG_NET_ICMP_SOCKET
  IOBUSER_NET_SOCK_ICMP,
#endif
#ifdef CONFIG_NET_ICMPv6_SOCKET
  IOBUSER_NET_SOCK_ICMPv6,
#endif
#ifdef CONFIG_NET_UDP
  IOBUSER_NET_SOCK_UDP,
#endif
#ifdef CONFIG_NET_TCP
  IOBUSER_NET_SOCK_TCP,
#endif
#ifdef CONFIG_NET_IEEE802154
  IOBUSER_NET_SOCK_IEEE802154,
#endif
#ifdef CONFIG_NET_BLUETOOTH
  IOBUSER_NET_SOCK_BLUETOOTH,
#endif
#if defined(CONFIG_NET_UDP) && !defined(NET_UDP_NO_STACK)
  IOBUSER_NET_UDP_READAHEAD,
#endif
#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
  IOBUSER_NET_UDP_WRITEBUFFER,
#endif
#if defined(CONFIG_NET_TCP) && !defined(NET_TCP_NO_STACK)
  IOBUSER_NET_TCP_READAHEAD,
#endif
#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  IOBUSER_NET_TCP_WRITEBUFFER,
#endif
#ifdef CONFIG_NET_IPFORWARD
  IOBUSER_NET_IPFORWARD,
#endif
#ifdef CONFIG_WIRELESS_IEEE802154
  IOBUSER_WIRELESS_RAD802154,
#endif
#ifdef CONFIG_IEEE802154_MAC
  IOBUSER_WIRELESS_MAC802154,
#endif
#ifdef CONFIG_IEEE802154_MACDEV
  IOBUSER_WIRELESS_MAC802154_CHARDEV,
#endif
#ifdef CONFIG_IEEE802154_NETDEV
  IOBUSER_WIRELESS_MAC802154_NETDEV,
#endif
 #ifdef CONFIG_WL_SPIRIT
  IOBUSER_WIRELESS_PACKETRADIO,
#endif
#ifdef CONFIG_WIRELESS_BLUETOOTH
  IOBUSER_WIRELESS_BLUETOOTH,
#endif
#ifdef CONFIG_NET_CAN
  IOBUSER_NET_CAN_READAHEAD,
#endif
  IOBUSER_GLOBAL,
  IOBUSER_NENTRIES /* MUST BE LAST ENTRY */
};

struct iob_userstats_s
{
  int totalconsumed;
  int totalproduced;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: iob_initialize
 *
 * Description:
 *   Set up the I/O buffers for normal operations.
 *
 ****************************************************************************/

void iob_initialize(void);

/****************************************************************************
 * Name: iob_alloc
 *
 * Description:
 *   Allocate an I/O buffer by taking the buffer at the head of the free
 *   list.
 *
 ****************************************************************************/

FAR struct iob_s *iob_alloc(bool throttled, enum iob_user_e consumerid);

/****************************************************************************
 * Name: iob_tryalloc
 *
 * Description:
 *   Try to allocate an I/O buffer by taking the buffer at the head of the
 *   free list without waiting for a buffer to become free.
 *
 ****************************************************************************/

FAR struct iob_s *iob_tryalloc(bool throttled, enum iob_user_e consumerid);

/****************************************************************************
 * Name: iob_navail
 *
 * Description:
 *   Return the number of available IOBs.
 *
 ****************************************************************************/

int iob_navail(bool throttled);

/****************************************************************************
 * Name: iob_qentry_navail
 *
 * Description:
 *   Return the number of available IOB chains.
 *
 ****************************************************************************/

int iob_qentry_navail(void);

/****************************************************************************
 * Name: iob_free
 *
 * Description:
 *   Free the I/O buffer at the head of a buffer chain returning it to the
 *   free list.  The link to  the next I/O buffer in the chain is return.
 *
 ****************************************************************************/

FAR struct iob_s *iob_free(FAR struct iob_s *iob,
                           enum iob_user_e producerid);

/****************************************************************************
 * Name: iob_notifier_setup
 *
 * Description:
 *   Set up to perform a callback to the worker function when an IOB is
 *   available.  The worker function will execute on the selected priority
 *   worker thread.
 *
 * Input Parameters:
 *   qid    - Selects work queue.  Must be HPWORK or LPWORK.
 *   worker - The worker function to execute on the high priority work queue
 *            when the event occurs.
 *   arg    - A user-defined argument that will be available to the worker
 *            function when it runs.
 *
 * Returned Value:
 *   > 0   - The signal notification is in place.  The returned value is a
 *           key that may be used later in a call to
 *           iob_notifier_teardown().
 *   == 0  - There are already free IOBs.  No signal notification will be
 *           provided.
 *   < 0   - An unexpected error occurred and no signal will be sent.  The
 *           returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_IOB_NOTIFIER
int iob_notifier_setup(int qid, worker_t worker, FAR void *arg);
#endif

/****************************************************************************
 * Name: iob_notifier_teardown
 *
 * Description:
 *   Eliminate an IOB notification previously setup by iob_notifier_setup().
 *   This function should only be called if the notification should be
 *   aborted prior to the notification.  The notification will automatically
 *   be torn down after the signal is sent.
 *
 * Input Parameters:
 *   key - The key value returned from a previous call to
 *         iob_notifier_setup().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_IOB_NOTIFIER
int iob_notifier_teardown(int key);
#endif

/****************************************************************************
 * Name: iob_free_chain
 *
 * Description:
 *   Free an entire buffer chain, starting at the beginning of the I/O
 *   buffer chain
 *
 ****************************************************************************/

void iob_free_chain(FAR struct iob_s *iob, enum iob_user_e producerid);

/****************************************************************************
 * Name: iob_add_queue
 *
 * Description:
 *   Add one I/O buffer chain to the end of a queue.  May fail due to lack
 *   of resources.
 *
 ****************************************************************************/

#if CONFIG_IOB_NCHAINS > 0
int iob_add_queue(FAR struct iob_s *iob, FAR struct iob_queue_s *iobq);
#endif /* CONFIG_IOB_NCHAINS > 0 */

/****************************************************************************
 * Name: iob_tryadd_queue
 *
 * Description:
 *   Add one I/O buffer chain to the end of a queue without waiting for
 *   resources to become free.
 *
 ****************************************************************************/

#if CONFIG_IOB_NCHAINS > 0
int iob_tryadd_queue(FAR struct iob_s *iob, FAR struct iob_queue_s *iobq);
#endif /* CONFIG_IOB_NCHAINS > 0 */

/****************************************************************************
 * Name: iob_remove_queue
 *
 * Description:
 *   Remove and return one I/O buffer chain from the head of a queue.
 *
 * Returned Value:
 *   Returns a reference to the I/O buffer chain at the head of the queue.
 *
 ****************************************************************************/

#if CONFIG_IOB_NCHAINS > 0
FAR struct iob_s *iob_remove_queue(FAR struct iob_queue_s *iobq);
#endif /* CONFIG_IOB_NCHAINS > 0 */

/****************************************************************************
 * Name: iob_peek_queue
 *
 * Description:
 *   Return a reference to the I/O buffer chain at the head of a queue. This
 *   is similar to iob_remove_queue except that the I/O buffer chain is in
 *   place at the head of the queue.  The I/O buffer chain may safely be
 *   modified by the caller but must be removed from the queue before it can
 *   be freed.
 *
 * Returned Value:
 *   Returns a reference to the I/O buffer chain at the head of the queue.
 *
 ****************************************************************************/

#if CONFIG_IOB_NCHAINS > 0
FAR struct iob_s *iob_peek_queue(FAR struct iob_queue_s *iobq);
#endif

/****************************************************************************
 * Name: iob_free_queue
 *
 * Description:
 *   Free an entire queue of I/O buffer chains.
 *
 ****************************************************************************/

#if CONFIG_IOB_NCHAINS > 0
void iob_free_queue(FAR struct iob_queue_s *qhead,
                    enum iob_user_e producerid);
#endif /* CONFIG_IOB_NCHAINS > 0 */

/****************************************************************************
 * Name: iob_copyin
 *
 * Description:
 *  Copy data 'len' bytes from a user buffer into the I/O buffer chain,
 *  starting at 'offset', extending the chain as necessary.
 *
 ****************************************************************************/

int iob_copyin(FAR struct iob_s *iob, FAR const uint8_t *src,
               unsigned int len, unsigned int offset, bool throttled,
               enum iob_user_e consumerid);

/****************************************************************************
 * Name: iob_trycopyin
 *
 * Description:
 *  Copy data 'len' bytes from a user buffer into the I/O buffer chain,
 *  starting at 'offset', extending the chain as necessary BUT without
 *  waiting if buffers are not available.
 *
 ****************************************************************************/

int iob_trycopyin(FAR struct iob_s *iob, FAR const uint8_t *src,
                  unsigned int len, unsigned int offset, bool throttled,
                  enum iob_user_e consumerid);

/****************************************************************************
 * Name: iob_copyout
 *
 * Description:
 *  Copy data 'len' bytes of data into the user buffer starting at 'offset'
 *  in the I/O buffer, returning that actual number of bytes copied out.
 *
 ****************************************************************************/

int iob_copyout(FAR uint8_t *dest, FAR const struct iob_s *iob,
                unsigned int len, unsigned int offset);

/****************************************************************************
 * Name: iob_clone
 *
 * Description:
 *   Duplicate (and pack) the data in iob1 in iob2.  iob2 must be empty.
 *
 ****************************************************************************/

int iob_clone(FAR struct iob_s *iob1, FAR struct iob_s *iob2, bool throttled,
              enum iob_user_e consumerid);

/****************************************************************************
 * Name: iob_concat
 *
 * Description:
 *   Concatenate iob_s chain iob2 to iob1.
 *
 ****************************************************************************/

void iob_concat(FAR struct iob_s *iob1, FAR struct iob_s *iob2);

/****************************************************************************
 * Name: iob_trimhead
 *
 * Description:
 *   Remove bytes from the beginning of an I/O chain.  Emptied I/O buffers
 *   are freed and, hence, the beginning of the chain may change.
 *
 ****************************************************************************/

FAR struct iob_s *iob_trimhead(FAR struct iob_s *iob, unsigned int trimlen,
                               enum iob_user_e producerid);

/****************************************************************************
 * Name: iob_trimhead_queue
 *
 * Description:
 *   Remove bytes from the beginning of an I/O chain at the head of the
 *   queue.  Emptied I/O buffers are freed and, hence, the head of the
 *   queue may change.
 *
 *   This function is just a wrapper around iob_trimhead() that assures that
 *   the iob at the head of queue is modified with the trimming operations.
 *
 * Returned Value:
 *   The new iob at the head of the queue is returned.
 *
 ****************************************************************************/

#if CONFIG_IOB_NCHAINS > 0
FAR struct iob_s *iob_trimhead_queue(FAR struct iob_queue_s *qhead,
                                     unsigned int trimlen,
                                     enum iob_user_e producerid);
#endif

/****************************************************************************
 * Name: iob_trimtail
 *
 * Description:
 *   Remove bytes from the end of an I/O chain.  Emptied I/O buffers are
 *   freed NULL will be returned in the special case where the entry I/O
 *   buffer chain is freed.
 *
 ****************************************************************************/

FAR struct iob_s *iob_trimtail(FAR struct iob_s *iob, unsigned int trimlen,
                               enum iob_user_e producerid);

/****************************************************************************
 * Name: iob_pack
 *
 * Description:
 *   Pack all data in the I/O buffer chain so that the data offset is zero
 *   and all but the final buffer in the chain are filled.  Any emptied
 *   buffers at the end of the chain are freed.
 *
 ****************************************************************************/

FAR struct iob_s *iob_pack(FAR struct iob_s *iob,
                           enum iob_user_e producerid);

/****************************************************************************
 * Name: iob_contig
 *
 * Description:
 *   Ensure that there is 'len' bytes of contiguous space at the beginning
 *   of the I/O buffer chain starting at 'iob'.
 *
 ****************************************************************************/

int iob_contig(FAR struct iob_s *iob, unsigned int len,
               enum iob_user_e producerid);

/****************************************************************************
 * Name: iob_dump
 *
 * Description:
 *   Dump the contents of a I/O buffer chain
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
void iob_dump(FAR const char *msg, FAR struct iob_s *iob, unsigned int len,
              unsigned int offset);
#else
#  define iob_dump(wrb)
#endif

/****************************************************************************
 * Name: iob_getuserstats
 *
 * Description:
 *   Return a reference to the IOB usage statistics for the IOB
 *   consumer/producer
 *
 * Input Parameters:
 *   userid - id representing the IOB producer/consumer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS) && \
    !defined(CONFIG_FS_PROCFS_EXCLUDE_IOBINFO)
FAR struct iob_userstats_s * iob_getuserstats(enum iob_user_e userid);
#endif

#endif /* CONFIG_MM_IOB */
#endif /* _INCLUDE_NUTTX_MM_IOB_H */
