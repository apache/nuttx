/****************************************************************************
 * include/nuttx/net/iob.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef _INCLUDE_NUTTX_NET_IOB_H
#define _INCLUDE_NUTTX_NET_IOB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/net/iob.h>

#ifdef CONFIG_NET_IOB

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* I/O buffer allocation logic supports a throttle value for the TCP
 * read-ahead buffering to prevent the read-ahead from consuming all
 * available I/O buffers.  This throttle only applies if both TCP write
 * buffering and TCP read-ahead buffering are enabled.
 */

#if !defined(CONFIG_NET_TCP_WRITE_BUFFERS) || !defined(CONFIG_NET_TCP_READAHEAD)
#  undef CONFIG_IOB_THROTTLE
#  define CONFIG_IOB_THROTTLE 0
#endif

/* The correct way to disable throttling is to the the throttle value to
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

/****************************************************************************
 * Global Data
 ****************************************************************************/

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
 *   Allocate an I/O buffer by taking the buffer at the head of the free list.
 *
 ****************************************************************************/

FAR struct iob_s *iob_alloc(bool throttled);

/****************************************************************************
 * Name: iob_free
 *
 * Description:
 *   Free the I/O buffer at the head of a buffer chain returning it to the
 *   free list.  The link to  the next I/O buffer in the chain is return.
 *
 ****************************************************************************/

FAR struct iob_s *iob_free(FAR struct iob_s *iob);

/****************************************************************************
 * Name: iob_free_chain
 *
 * Description:
 *   Free an entire buffer chain, starting at the beginning of the I/O
 *   buffer chain
 *
 ****************************************************************************/

void iob_free_chain(FAR struct iob_s *iob);

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
void iob_free_queue(FAR struct iob_queue_s *qhead);
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
               unsigned int len, unsigned int offset, bool throttled);

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
                  unsigned int len, unsigned int offset, bool throttled);

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

int iob_clone(FAR struct iob_s *iob1, FAR struct iob_s *iob2, bool throttled);

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

FAR struct iob_s *iob_trimhead(FAR struct iob_s *iob, unsigned int trimlen);

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
                                     unsigned int trimlen);
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

FAR struct iob_s *iob_trimtail(FAR struct iob_s *iob, unsigned int trimlen);

/****************************************************************************
 * Name: iob_pack
 *
 * Description:
 *   Pack all data in the I/O buffer chain so that the data offset is zero
 *   and all but the final buffer in the chain are filled.  Any emptied
 *   buffers at the end of the chain are freed.
 *
 ****************************************************************************/

FAR struct iob_s *iob_pack(FAR struct iob_s *iob);

/****************************************************************************
 * Name: iob_contig
 *
 * Description:
 *   Ensure that there is 'len' bytes of contiguous space at the beginning
 *   of the I/O buffer chain starting at 'iob'.
 *
 ****************************************************************************/

int iob_contig(FAR struct iob_s *iob, unsigned int len);

/****************************************************************************
 * Function: iob_dump
 *
 * Description:
 *   Dump the contents of a I/O buffer chain
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
void iob_dump(FAR const char *msg, FAR struct iob_s *iob, unsigned int len,
              unsigned int offset);
#else
#  define iob_dump(wrb)
#endif

#endif /* CONFIG_NET_IOB */
#endif /* _INCLUDE_NUTTX_NET_IOB_H */

