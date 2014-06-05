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
#include <queue.h>

#include <nuttx/net/iob.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOB flags */

#define IOBFLAGS_MCAST   (1 << 0) /* Multicast packet */
#define IOBFLAGS_VLANTAG (1 << 1) /* VLAN tag is value (not supported) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Represents one I/O buffer.  A packet is contained by one or more I/O
 * buffers in a chain.  The io_flags, io_pktlen, io_vtag and io_priv
 * fields are only valid for the I/O buffer at the head of the chain.
 */

struct iob_s
{
  sq_entry_t io_link;    /* Link to the next I/O buffer in the chain */
  uint8_t    io_flags;   /* Flags associated with the I/O buffer */
#if CONFIG_IOB_BUFSIZE < 256
  uint8_t    io_len;     /* Length of the data in the entry */
  uint8_t    io_offset;  /* Data begins at this offset */
#else
  uint16_t   io_len;     /* Length of the data in the entry */
  uint16_t   io_offset;  /* Data begins at this offset */
#endif
  uint16_t   io_pktlen;  /* Total length of the packet */
#ifdef __NO_VLAN__
  uint16_t   io_vtag;    /* VLAN tag (not supported) */
#endif
  void      *io_priv;    /* User private data attached to the I/O buffer */
  uint8_t    io_data[CONFIG_IOB_BUFSIZE];
};

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
 *   Allocate an I/O buffer by take the buffer at the head of the free list.
 *
 ****************************************************************************/

FAR struct iob_s *iob_alloc(void);

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
 * Name: iob_freechain
 *
 * Description:
 *   Free an entire buffer chain, starting at the beginning of the I/O
 *   buffer chain
 *
 ****************************************************************************/

void iob_freechain(FAR struct iob_s *iob);

/****************************************************************************
 * Name: iob_freeq
 *
 * Description:
 *   Free an entire buffer chain, starting at a queue head
 *
 ****************************************************************************/

void iob_freeq(FAR sq_queue_t *q);

/****************************************************************************
 * Name: iob_copyin
 *
 * Description:
 *  Copy data 'len' bytes from a user buffer into the I/O buffer chain,
 *  starting at 'offset', extending the chain as necessary.
 *
 ****************************************************************************/

int iob_copyin(FAR struct iob_s *iob, FAR const uint8_t *src,
               unsigned int len, unsigned int offset);

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

int iob_clone(FAR struct iob_s *iob1, FAR struct iob_s *iob2);

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

#endif /* _INCLUDE_NUTTX_NET_IOB_H */
