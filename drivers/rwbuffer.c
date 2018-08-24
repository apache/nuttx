/****************************************************************************
 * drivers/rwbuffer.c
 *
 *   Copyright (C) 2009, 2011, 2013-2014, 2017 Gregory Nutt. All rights
 *     reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/drivers/rwbuffer.h>

#if defined(CONFIG_DRVR_WRITEBUFFER) || defined(CONFIG_DRVR_READAHEAD)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

#ifndef CONFIG_DRVR_WRDELAY
#  define CONFIG_DRVR_WRDELAY 350
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rwb_semtake
 ****************************************************************************/

static void rwb_semtake(sem_t *sem)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(sem);

      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/****************************************************************************
 * Name: rwb_semgive
 ****************************************************************************/

#define rwb_semgive(s) nxsem_post(s)

/****************************************************************************
 * Name: rwb_overlap
 ****************************************************************************/

static inline bool rwb_overlap(off_t blockstart1, size_t nblocks1,
                               off_t blockstart2, size_t nblocks2)
{
  off_t blockend1 = blockstart1 + nblocks1 - 1;
  off_t blockend2 = blockstart2 + nblocks2 - 1;

  /* If the buffer 1 is wholly outside of buffer 2, return false */

  if ((blockend1   < blockstart2) || /* Wholly "below" */
      (blockstart1 > blockend2))     /* Wholly "above" */
    {
      return false;
    }
  else
    {
      return true;
    }
}

/****************************************************************************
 * Name: rwb_resetwrbuffer
 ****************************************************************************/

#ifdef CONFIG_DRVR_WRITEBUFFER
static inline void rwb_resetwrbuffer(struct rwbuffer_s *rwb)
{
  /* We assume that the caller holds the wrsem */

  rwb->wrnblocks       = 0;
  rwb->wrblockstart    = (off_t)-1;
  rwb->wrexpectedblock = (off_t)-1;
}
#endif

/****************************************************************************
 * Name: rwb_wrflush
 *
 * Assumptions:
 *   The caller holds the wrsem semaphore.
 *
 ****************************************************************************/

#ifdef CONFIG_DRVR_WRITEBUFFER
static void rwb_wrflush(struct rwbuffer_s *rwb)
{
  int ret;

  finfo("Timeout!\n");

  if (rwb->wrnblocks > 0)
    {
      finfo("Flushing: blockstart=0x%08lx nblocks=%d from buffer=%p\n",
      (long)rwb->wrblockstart, rwb->wrnblocks, rwb->wrbuffer);

      /* Flush cache.  On success, the flush method will return the number
       * of blocks written.  Anything other than the number requested is
       * an error.
       */

      ret = rwb->wrflush(rwb->dev, rwb->wrbuffer, rwb->wrblockstart, rwb->wrnblocks);
      if (ret != rwb->wrnblocks)
        {
          ferr("ERROR: Error flushing write buffer: %d\n", ret);
        }

      rwb_resetwrbuffer(rwb);
    }

}
#endif

/****************************************************************************
 * Name: rwb_wrtimeout
 ****************************************************************************/

static void rwb_wrtimeout(FAR void *arg)
{
  /* The following assumes that the size of a pointer is 4-bytes or less */

  FAR struct rwbuffer_s *rwb = (struct rwbuffer_s *)arg;
  DEBUGASSERT(rwb != NULL);

  /* If a timeout elapses with with write buffer activity, this watchdog
   * handler function will be evoked on the thread of execution of the
   * worker thread.
   */

  rwb_semtake(&rwb->wrsem);
  rwb_wrflush(rwb);
  rwb_semgive(&rwb->wrsem);
}

/****************************************************************************
 * Name: rwb_wrstarttimeout
 ****************************************************************************/

static void rwb_wrstarttimeout(FAR struct rwbuffer_s *rwb)
{
  /* CONFIG_DRVR_WRDELAY provides the delay period in milliseconds. CLK_TCK
   * provides the clock tick of the system (frequency in Hz).
   */

  int ticks = (CONFIG_DRVR_WRDELAY + CLK_TCK/2) / CLK_TCK;
  (void)work_queue(LPWORK, &rwb->work, rwb_wrtimeout, (FAR void *)rwb, ticks);
}

/****************************************************************************
 * Name: rwb_wrcanceltimeout
 ****************************************************************************/

static inline void rwb_wrcanceltimeout(struct rwbuffer_s *rwb)
{
  (void)work_cancel(LPWORK, &rwb->work);
}

/****************************************************************************
 * Name: rwb_writebuffer
 ****************************************************************************/

#ifdef CONFIG_DRVR_WRITEBUFFER
static ssize_t rwb_writebuffer(FAR struct rwbuffer_s *rwb,
                               off_t startblock, uint32_t nblocks,
                               FAR const uint8_t *wrbuffer)
{
  int ret;

  /* Write writebuffer Logic */

  rwb_wrcanceltimeout(rwb);

  /* First: Should we flush out our cache? We would do that if (1) we already
   * buffering blocks and the next block writing is not in the same sequence,
   * or (2) the number of blocks would exceed our allocated buffer capacity
   */

  if (((startblock != rwb->wrexpectedblock) && (rwb->wrnblocks)) ||
      ((rwb->wrnblocks + nblocks) > rwb->wrmaxblocks))
    {
      finfo("writebuffer miss, expected: %08x, given: %08x\n",
            rwb->wrexpectedblock, startblock);

      /* Flush the write buffer */

      ret = rwb->wrflush(rwb->dev, rwb->wrbuffer, rwb->wrblockstart, rwb->wrnblocks);
      if (ret < 0)
        {
          ferr("ERROR: Error writing multiple from cache: %d\n", -ret);
          return ret;
        }

      rwb_resetwrbuffer(rwb);
    }

  /* writebuffer is empty? Then initialize it */

  if (rwb->wrnblocks == 0)
    {
      finfo("Fresh cache starting at block: 0x%08x\n", startblock);
      rwb->wrblockstart = startblock;
    }

  /* Add data to cache */

  finfo("writebuffer: copying %d bytes from %p to %p\n",
        nblocks * rwb->blocksize, wrbuffer,
        &rwb->wrbuffer[rwb->wrnblocks * rwb->blocksize]);
  memcpy(&rwb->wrbuffer[rwb->wrnblocks * rwb->blocksize],
         wrbuffer, nblocks * rwb->blocksize);

  rwb->wrnblocks      += nblocks;
  rwb->wrexpectedblock = rwb->wrblockstart + rwb->wrnblocks;
  rwb_wrstarttimeout(rwb);
  return nblocks;
}
#endif

/****************************************************************************
 * Name: rwb_resetrhbuffer
 ****************************************************************************/

#ifdef CONFIG_DRVR_READAHEAD
static inline void rwb_resetrhbuffer(struct rwbuffer_s *rwb)
{
  /* We assume that the caller holds the readAheadBufferSemphore */

  rwb->rhnblocks    = 0;
  rwb->rhblockstart = (off_t)-1;
}
#endif

/****************************************************************************
 * Name: rwb_bufferread
 ****************************************************************************/

#ifdef CONFIG_DRVR_READAHEAD
static inline void
rwb_bufferread(struct rwbuffer_s *rwb,  off_t startblock,
               size_t nblocks, uint8_t **rdbuffer)
{
  /* We assume that (1) the caller holds the readAheadBufferSemphore, and (2)
   * that the caller already knows that all of the blocks are in the
   * read-ahead buffer.
   */

  /* Convert the units from blocks to bytes */

  off_t  blockoffset = startblock - rwb->rhblockstart;
  off_t  byteoffset  = rwb->blocksize * blockoffset;
  size_t nbytes      = rwb->blocksize * nblocks;

  /* Get the byte address in the read-ahead buffer */

  uint8_t *rhbuffer    = rwb->rhbuffer + byteoffset;

  /* Copy the data from the read-ahead buffer into the IO buffer */

  memcpy(*rdbuffer, rhbuffer, nbytes);

  /* Update the caller's copy for the next address */

  *rdbuffer += nbytes;
}
#endif

/****************************************************************************
 * Name: rwb_rhreload
 ****************************************************************************/

#ifdef CONFIG_DRVR_READAHEAD
static int rwb_rhreload(struct rwbuffer_s *rwb, off_t startblock)
{
  off_t  endblock;
  size_t nblocks;
  int    ret;

  /* Check for attempts to read beyond the end of the media */

  if (startblock >= rwb->nblocks)
    {
      return -ESPIPE;
    }

  /* Get the block number +1 of the last block that will fit in the
   * read-ahead buffer
   */

  endblock = startblock + rwb->rhmaxblocks;

  /* Make sure that we don't read past the end of the device */

  if (endblock > rwb->nblocks)
    {
      endblock = rwb->nblocks;
    }

  nblocks = endblock - startblock;

  /* Reset the read buffer */

  rwb_resetrhbuffer(rwb);

  /* Now perform the read */

  ret = rwb->rhreload(rwb->dev, rwb->rhbuffer, startblock, nblocks);
  if (ret == nblocks)
    {
      /* Update information about what is in the read-ahead buffer */

      rwb->rhnblocks    = nblocks;
      rwb->rhblockstart = startblock;

      /* The return value is not the number of blocks we asked to be loaded. */

      return nblocks;
    }

  return -EIO;
}
#endif

/****************************************************************************
 * Name: rwb_invalidate_writebuffer
 *
 * Description:
 *   Invalidate a region of the write buffer
 *
 ****************************************************************************/

#if defined(CONFIG_DRVR_WRITEBUFFER) && defined(CONFIG_DRVR_INVALIDATE)
int rwb_invalidate_writebuffer(FAR struct rwbuffer_s *rwb,
                               off_t startblock, size_t blockcount)
{
  int ret = OK;

  /* Is there a write buffer?  Is data saved in the write buffer? */

  if (rwb->wrmaxblocks > 0 && rwb->wrnblocks > 0)
    {
      off_t wrbend;
      off_t invend;

      finfo("startblock=%d blockcount=%p\n", startblock, blockcount);

      rwb_semtake(&rwb->wrsem);

      /* Now there are five cases:
       *
       * 1. We invalidate nothing
       */

      wrbend = rwb->wrblockstart + rwb->wrnblocks;
      invend = startblock + blockcount;

      if (rwb->wrblockstart > invend || wrbend < startblock)
        {
          ret = OK;
        }

      /* 2. We invalidate the entire write buffer. */

      else if (rwb->wrblockstart >= startblock && wrbend <= invend)
        {
          rwb->wrnblocks = 0;
          ret = OK;
        }

      /* We are going to invalidate a subset of the write buffer.  Three
       * more cases to consider:
       *
       * 2. We invalidate a portion in the middle of the write buffer
       */

      else if (rwb->wrblockstart < startblock && wrbend > invend)
        {
          uint8_t *src;
          off_t    block;
          off_t    offset;
          size_t   nblocks;

          /* Write the blocks at the end of the media to hardware */

          nblocks = wrbend - invend;
          block   = invend;
          offset  = block - rwb->wrblockstart;
          src     = rwb->wrbuffer + offset * rwb->blocksize;

          ret = rwb->wrflush(rwb->dev, src, block, nblocks);
          if (ret < 0)
            {
              ferr("ERROR: wrflush failed: %d\n", ret);
            }

          /* Keep the blocks at the beginning of the buffer up the
           * start of the invalidated region.
           */
          else
            {
              rwb->wrnblocks = startblock - rwb->wrblockstart;
              ret = OK;
            }
        }

      /* 3. We invalidate a portion at the end of the write buffer */

      else if (wrbend > startblock && wrbend <= invend)
        {
          rwb->wrnblocks = wrbend - startblock;
          ret = OK;
        }

      /* 4. We invalidate a portion at the beginning of the write buffer */

      else /* if (rwb->wrblockstart >= startblock && wrbend > invend) */
        {
          uint8_t *src;
          size_t   ninval;
          size_t   nkeep;

          DEBUGASSERT(rwb->wrblockstart >= startblock && wrbend > invend);

          /* Copy the data from the uninvalidated region to the beginning
           * of the write buffer.
           *
           * First calculate the source and destination of the transfer.
           */

          ninval = invend - rwb->wrblockstart;
          src    = rwb->wrbuffer + ninval * rwb->blocksize;

          /* Calculate the number of blocks we are keeping.  We keep
           * the ones that we don't invalidate.
           */

          nkeep  = rwb->wrnblocks - ninval;

          /* Then move the data that we are keeping to the beginning
           * the write buffer.
           */

          memcpy(rwb->wrbuffer, src, nkeep * rwb->blocksize);

          /* Update the block info.  The first block is now the one just
           * after the invalidation region and the number buffered blocks
           * is the number that we kept.
           */

          rwb->wrblockstart = invend;
          rwb->wrnblocks    = nkeep;
          ret = OK;
        }

      rwb_semgive(&rwb->wrsem);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: rwb_invalidate_readahead
 *
 * Description:
 *   Invalidate a region of the read-ahead buffer
 *
 ****************************************************************************/

#if defined(CONFIG_DRVR_READAHEAD)  && defined(CONFIG_DRVR_INVALIDATE)
int rwb_invalidate_readahead(FAR struct rwbuffer_s *rwb,
                               off_t startblock, size_t blockcount)
{
  int ret;

  if (rwb->rhmaxblocks > 0 && rwb->rhnblocks > 0)
    {
      off_t rhbend;
      off_t invend;

      finfo("startblock=%d blockcount=%p\n", startblock, blockcount);

      rwb_semtake(&rwb->rhsem);

      /* Now there are five cases:
       *
       * 1. We invalidate nothing
       */

      rhbend = rwb->rhblockstart + rwb->rhnblocks;
      invend = startblock + blockcount;

      if (rhbend <= startblock || rwb->rhblockstart >= invend)
        {
          ret = OK;
        }

      /* 2. We invalidate the entire read-ahead buffer. */

      else if (rwb->rhblockstart >= startblock && rhbend <= invend)
        {
          rwb->rhnblocks = 0;
          ret = OK;
        }

      /* We are going to invalidate a subset of the read-ahead buffer.
       * Three more cases to consider:
       *
       * 2. We invalidate a portion in the middle of the read-ahead buffer
       */

      else if (rwb->rhblockstart < startblock && rhbend > invend)
        {
          /* Keep the blocks at the beginning of the buffer up the
           * start of the invalidated region.
           */

          rwb->rhnblocks = startblock - rwb->rhblockstart;
          ret = OK;
        }

      /* 3. We invalidate a portion at the end of the read-ahead buffer */

      else if (rhbend > startblock && rhbend <= invend)
        {
          rwb->rhnblocks = rhbend - startblock;
          ret = OK;
        }

      /* 4. We invalidate a portion at the beginning of the write buffer */

      else /* if (rwb->rhblockstart >= startblock && rhbend > invend) */
        {
          /* Let's just force the whole read-ahead buffer to be reloaded.
           * That might cost s small amount of performance, but well worth
           * the lower complexity.
           */

          DEBUGASSERT(rwb->rhblockstart >= startblock && rhbend > invend);
          rwb->rhnblocks = 0;
          ret = OK;
        }

      rwb_semgive(&rwb->rhsem);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: rwb_initialize
 ****************************************************************************/

int rwb_initialize(FAR struct rwbuffer_s *rwb)
{
  uint32_t allocsize;

  /* Sanity checking */

  DEBUGASSERT(rwb != NULL);
  DEBUGASSERT(rwb->blocksize > 0);
  DEBUGASSERT(rwb->nblocks > 0);
  DEBUGASSERT(rwb->dev != NULL);

  /* Setup so that rwb_uninitialize can handle a failure */

#ifdef CONFIG_DRVR_WRITEBUFFER
  DEBUGASSERT(rwb->wrflush != NULL);
  rwb->wrbuffer = NULL;
#endif
#ifdef CONFIG_DRVR_READAHEAD
  DEBUGASSERT(rwb->rhreload != NULL);
  rwb->rhbuffer = NULL;
#endif

#ifdef CONFIG_DRVR_WRITEBUFFER
  if (rwb->wrmaxblocks > 0)
    {
      finfo("Initialize the write buffer\n");

      /* Initialize the write buffer access semaphore */

      nxsem_init(&rwb->wrsem, 0, 1);

      /* Initialize write buffer parameters */

      rwb_resetwrbuffer(rwb);

      /* Allocate the write buffer */

      rwb->wrbuffer = NULL;
      if (rwb->wrmaxblocks > 0)
        {
          allocsize     = rwb->wrmaxblocks * rwb->blocksize;
          rwb->wrbuffer = kmm_malloc(allocsize);
          if (!rwb->wrbuffer)
            {
              ferr("Write buffer kmm_malloc(%d) failed\n", allocsize);
              return -ENOMEM;
            }
        }

      finfo("Write buffer size: %d bytes\n", allocsize);
    }
#endif /* CONFIG_DRVR_WRITEBUFFER */

#ifdef CONFIG_DRVR_READAHEAD
  if (rwb->rhmaxblocks > 0)
    {
      finfo("Initialize the read-ahead buffer\n");

      /* Initialize the read-ahead buffer access semaphore */

      nxsem_init(&rwb->rhsem, 0, 1);

      /* Initialize read-ahead buffer parameters */

      rwb_resetrhbuffer(rwb);

      /* Allocate the read-ahead buffer */

      rwb->rhbuffer = NULL;
      if (rwb->rhmaxblocks > 0)
        {
          allocsize     = rwb->rhmaxblocks * rwb->blocksize;
          rwb->rhbuffer = kmm_malloc(allocsize);
          if (!rwb->rhbuffer)
            {
              ferr("Read-ahead buffer kmm_malloc(%d) failed\n", allocsize);
              return -ENOMEM;
            }
        }

      finfo("Read-ahead buffer size: %d bytes\n", allocsize);
    }
#endif /* CONFIG_DRVR_READAHEAD */

  return OK;
}

/****************************************************************************
 * Name: rwb_uninitialize
 ****************************************************************************/

void rwb_uninitialize(FAR struct rwbuffer_s *rwb)
{
#ifdef CONFIG_DRVR_WRITEBUFFER
  if (rwb->wrmaxblocks > 0)
    {
      rwb_wrcanceltimeout(rwb);
      nxsem_destroy(&rwb->wrsem);
      if (rwb->wrbuffer)
        {
          kmm_free(rwb->wrbuffer);
        }
    }
#endif

#ifdef CONFIG_DRVR_READAHEAD
  if (rwb->rhmaxblocks > 0)
    {
      nxsem_destroy(&rwb->rhsem);
      if (rwb->rhbuffer)
        {
          kmm_free(rwb->rhbuffer);
        }
    }
#endif
}

/****************************************************************************
 * Name: rwb_read
 ****************************************************************************/

ssize_t rwb_read(FAR struct rwbuffer_s *rwb, off_t startblock,
                 size_t nblocks, FAR uint8_t *rdbuffer)
{
#ifdef CONFIG_DRVR_READAHEAD
  size_t remaining;
#endif
  int ret = OK;

  finfo("startblock=%ld nblocks=%ld rdbuffer=%p\n",
        (long)startblock, (long)nblocks, rdbuffer);

#ifdef CONFIG_DRVR_WRITEBUFFER
  /* If the new read data overlaps any part of the write buffer, then
   * flush the write data onto the physical media before reading.  We
   * could attempt some more exotic handling -- but this simple logic
   * is well-suited for simple streaming applications.
   */

  if (rwb->wrmaxblocks > 0)
    {
      /* If the write buffer overlaps the block(s) requested, then flush the
       * write buffer.
       */

      rwb_semtake(&rwb->wrsem);
      if (rwb_overlap(rwb->wrblockstart, rwb->wrnblocks, startblock, nblocks))
        {
          rwb_wrflush(rwb);
        }

      rwb_semgive(&rwb->wrsem);
    }
#endif

#ifdef CONFIG_DRVR_READAHEAD
  if (rwb->rhmaxblocks > 0)
    {
      /* Loop until we have read all of the requested blocks */

      rwb_semtake(&rwb->rhsem);
      for (remaining = nblocks; remaining > 0; )
        {
          /* Is there anything in the read-ahead buffer? */

          if (rwb->rhnblocks > 0)
            {
              off_t  bufferend;

              /* How many blocks are available in this buffer? */

              bufferend = rwb->rhblockstart + rwb->rhnblocks;
              if (startblock >= rwb->rhblockstart && startblock < bufferend)
                {
                  size_t rdblocks = bufferend - startblock;
                  if (rdblocks > remaining)
                    {
                      rdblocks = remaining;
                    }

                  /* Then read the data from the read-ahead buffer */

                  rwb_bufferread(rwb, startblock, rdblocks, &rdbuffer);
                  startblock += rdblocks;
                  remaining  -= rdblocks;
                }
            }

          /* If we did not get all of the data from the buffer, then we have
           * to refill the buffer and try again.
           */

          if (remaining > 0)
            {
              ret = rwb_rhreload(rwb, startblock);
              if (ret < 0)
                {
                  ferr("ERROR: Failed to fill the read-ahead buffer: %d\n", ret);
                  return (ssize_t)ret;
                }
            }
        }

      /* On success, return the number of blocks that we were requested to
       * read. This is for compatibility with the normal return of a block
       * driver read method
       */

      rwb_semgive(&rwb->rhsem);
      ret = nblocks;
    }
  else
#else
    {
      /* No read-ahead buffering, (re)load the data directly into
       * the user buffer.
       */

      ret = rwb->rhreload(rwb->dev, rdbuffer, startblock, nblocks);
    }
#endif

  return (ssize_t)ret;
}

/****************************************************************************
 * Name: rwb_write
 ****************************************************************************/

ssize_t rwb_write(FAR struct rwbuffer_s *rwb, off_t startblock,
                  size_t nblocks, FAR const uint8_t *wrbuffer)
{
  int ret = OK;

#ifdef CONFIG_DRVR_READAHEAD
  if (rwb->rhmaxblocks > 0)
    {
      /* If the new write data overlaps any part of the read buffer, then
       * flush the data from the read buffer.  We could attempt some more
       * exotic handling -- but this simple logic is well-suited for simple
       * streaming applications.
       */

      rwb_semtake(&rwb->rhsem);
      if (rwb_overlap(rwb->rhblockstart, rwb->rhnblocks, startblock, nblocks))
        {
          rwb_resetrhbuffer(rwb);
        }

      rwb_semgive(&rwb->rhsem);
    }
#endif

#ifdef CONFIG_DRVR_WRITEBUFFER
  if (rwb->wrmaxblocks > 0)
    {
      finfo("startblock=%d wrbuffer=%p\n", startblock, wrbuffer);

      /* Use the block cache unless the buffer size is bigger than block cache */

      if (nblocks > rwb->wrmaxblocks)
        {
          /* First flush the cache */

          rwb_semtake(&rwb->wrsem);
          rwb_wrflush(rwb);
          rwb_semgive(&rwb->wrsem);

          /* Then transfer the data directly to the media */

          ret = rwb->wrflush(rwb->dev, wrbuffer, startblock, nblocks);
        }
      else
        {
          /* Buffer the data in the write buffer */

          ret = rwb_writebuffer(rwb, startblock, nblocks, wrbuffer);
        }

      /* On success, return the number of blocks that we were requested to
       * write.  This is for compatibility with the normal return of a block
       * driver write method
       */
    }
  else
#endif /* CONFIG_DRVR_WRITEBUFFER */
    {
      /* No write buffer.. just pass the write operation through via the
       * flush callback.
       */

      ret = rwb->wrflush(rwb->dev, wrbuffer, startblock, nblocks);
    }

  return (ssize_t)ret;
}

/****************************************************************************
 * Name: rwb_readbytes
 *
 * Description:
 *   Character-oriented read
 *
 ****************************************************************************/

#ifdef CONFIG_DRVR_READBYTES
ssize_t rwb_readbytes(FAR struct rwbuffer_s *dev, off_t offset,
                      size_t nbytes, FAR uint8_t *buffer)
{
  /* Loop while there are bytes still be be read */

  /* Make sure that the sector containing the next bytes to transfer is in
   * memory.
   */

  /* How many bytes can be transfer from the in-memory data? */

  /* Transfer the bytes */

  /* Adjust counts and offsets for the next time through the loop */

#warning Not Implemented
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: rwb_mediaremoved
 *
 * Description:
 *   The following function is called when media is removed
 *
 ****************************************************************************/

#ifdef CONFIG_DRVR_REMOVABLE
int rwb_mediaremoved(FAR struct rwbuffer_s *rwb)
{
#ifdef CONFIG_DRVR_WRITEBUFFER
  if (rwb->wrmaxblocks > 0)
    {
      rwb_semtake(&rwb->wrsem);
      rwb_resetwrbuffer(rwb);
      rwb_semgive(&rwb->wrsem);
    }
#endif

#ifdef CONFIG_DRVR_READAHEAD
  if (rwb->rhmaxblocks > 0)
    {
      rwb_semtake(&rwb->rhsem);
      rwb_resetrhbuffer(rwb);
      rwb_semgive(&rwb->rhsem);
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: rwb_invalidate
 *
 * Description:
 *   Invalidate a region of the caches
 *
 ****************************************************************************/

#ifdef CONFIG_DRVR_INVALIDATE
int rwb_invalidate(FAR struct rwbuffer_s *rwb,
                   off_t startblock, size_t blockcount)
{
  int ret;

#ifdef CONFIG_DRVR_WRITEBUFFER
  ret = rwb_invalidate_writebuffer(rwb, startblock, blockcount);
  if (ret < 0)
    {
      ferr("ERROR: rwb_invalidate_writebuffer failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_DRVR_READAHEAD
  ret = rwb_invalidate_readahead(rwb, startblock, blockcount);
  if (ret < 0)
    {
      ferr("ERROR: rwb_invalidate_readahead failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}
#endif

#endif /* CONFIG_DRVR_WRITEBUFFER || CONFIG_DRVR_READAHEAD */

