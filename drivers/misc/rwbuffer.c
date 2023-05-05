/****************************************************************************
 * drivers/misc/rwbuffer.c
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

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/drivers/rwbuffer.h>

#if defined(CONFIG_DRVR_WRITEBUFFER) || defined(CONFIG_DRVR_READAHEAD)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_DRVR_WRDELAY
#  define CONFIG_DRVR_WRDELAY 350
#endif

#if !defined(CONFIG_SCHED_WORKQUEUE) && CONFIG_DRVR_WRDELAY != 0
#  error "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t rwb_read_(FAR struct rwbuffer_s *rwb, off_t startblock,
                         size_t nblocks, FAR uint8_t *rdbuffer);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rwb_lock
 ****************************************************************************/

#if defined(CONFIG_DRVR_WRITEBUFFER)
#  define rwb_lock(l) nxmutex_lock(l)
#else
#  define rwb_lock(l) OK
#endif

/****************************************************************************
 * Name: rwb_unlock
 ****************************************************************************/

#if defined(CONFIG_DRVR_WRITEBUFFER)
#  define rwb_unlock(l) nxmutex_unlock(l)
#else
#  define rwb_unlock(l)
#endif

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
static inline void rwb_resetwrbuffer(FAR struct rwbuffer_s *rwb)
{
  /* We assume that the caller holds the wrlock */

  rwb->wrnblocks    = 0;
  rwb->wrblockstart = -1;
}
#endif

/****************************************************************************
 * Name: rwb_wrflush
 *
 * Assumptions:
 *   The caller holds the wrlock mutex.
 *
 ****************************************************************************/

#ifdef CONFIG_DRVR_WRITEBUFFER
static void rwb_wrflush(FAR struct rwbuffer_s *rwb)
{
  int ret;

  if (rwb->wrnblocks > 0)
    {
      size_t padblocks;

      finfo("Flushing: blockstart=0x%08lx nblocks=%d from buffer=%p\n",
            (long)rwb->wrblockstart, rwb->wrnblocks, rwb->wrbuffer);

      padblocks = rwb->wrnblocks % rwb->wralignblocks;
      if (padblocks)
        {
          padblocks = rwb->wralignblocks - padblocks;
          rwb_read_(rwb, rwb->wrblockstart + rwb->wrnblocks, padblocks,
                    &rwb->wrbuffer[rwb->wrnblocks * rwb->blocksize]);
          rwb->wrnblocks += padblocks;
        }

      /* Flush cache.  On success, the flush method will return the number
       * of blocks written.  Anything other than the number requested is
       * an error.
       */

      ret = rwb->wrflush(rwb->dev, rwb->wrbuffer, rwb->wrblockstart,
                         rwb->wrnblocks);
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

#if defined(CONFIG_DRVR_WRITEBUFFER) && CONFIG_DRVR_WRDELAY != 0
static void rwb_wrtimeout(FAR void *arg)
{
  /* The following assumes that the size of a pointer is 4-bytes or less */

  FAR struct rwbuffer_s *rwb = (FAR struct rwbuffer_s *)arg;
  DEBUGASSERT(rwb != NULL);

  finfo("Timeout!\n");

  /* If a timeout elapses with write buffer activity, this watchdog
   * handler function will be evoked on the thread of execution of the
   * worker thread.
   */

  rwb_lock(&rwb->wrlock);
  rwb_wrflush(rwb);
  rwb_unlock(&rwb->wrlock);
}
#endif

/****************************************************************************
 * Name: rwb_wrstarttimeout
 ****************************************************************************/

#ifdef CONFIG_DRVR_WRITEBUFFER
static void rwb_wrstarttimeout(FAR struct rwbuffer_s *rwb)
{
#if CONFIG_DRVR_WRDELAY != 0
  /* CONFIG_DRVR_WRDELAY provides the delay period in milliseconds. CLK_TCK
   * provides the clock tick of the system (frequency in Hz).
   */

  int ticks = MSEC2TICK(CONFIG_DRVR_WRDELAY);
  work_queue(LPWORK, &rwb->work, rwb_wrtimeout, rwb, ticks);
#endif
}
#endif

/****************************************************************************
 * Name: rwb_wrcanceltimeout
 ****************************************************************************/

#ifdef CONFIG_DRVR_WRITEBUFFER
static inline void rwb_wrcanceltimeout(FAR struct rwbuffer_s *rwb)
{
#if CONFIG_DRVR_WRDELAY != 0
  work_cancel(LPWORK, &rwb->work);
#endif
}
#endif

/****************************************************************************
 * Name: rwb_writebuffer
 ****************************************************************************/

#ifdef CONFIG_DRVR_WRITEBUFFER
static ssize_t rwb_writebuffer(FAR struct rwbuffer_s *rwb,
                               off_t startblock, uint32_t nblocks,
                               FAR const uint8_t *wrbuffer)
{
  uint32_t nwritten = nblocks;

  /* Write writebuffer Logic */

  rwb_wrcanceltimeout(rwb);

  /* Is data saved in the write buffer? */

  if (rwb->wrnblocks > 0)
    {
      off_t wrbend;
      off_t newend;

      /* Now there are five cases:
       *
       * 1. We update the non-overlapping region
       */

      wrbend = rwb->wrblockstart + rwb->wrnblocks;
      newend = startblock + nblocks;

      if (wrbend < startblock || rwb->wrblockstart > newend)
        {
          /* Nothing to do */;
        }

      /* 2. We update the entire write buffer. */

      else if (rwb->wrblockstart > startblock && wrbend < newend)
        {
          rwb->wrnblocks = 0;
        }

      /* We are going to update a subset of the write buffer.  Three
       * more cases to consider:
       *
       * 3. We update a portion in the middle of the write buffer
       */

      else if (rwb->wrblockstart <= startblock && wrbend >= newend)
        {
          FAR uint8_t *dest;
          size_t offset;

          /* Copy the data to the middle of write buffer */

          offset = startblock - rwb->wrblockstart;
          dest   = rwb->wrbuffer + offset * rwb->blocksize;
          memcpy(dest, wrbuffer, nblocks * rwb->blocksize);

          nblocks = 0;
        }

      /* 4. We update a portion at the end of the write buffer */

      else if (wrbend >= startblock && wrbend <= newend)
        {
          FAR uint8_t *dest;
          size_t offset;
          size_t ncopy;

          /* Copy the data from the updating region to the end
           * of the write buffer.
           */

          offset = rwb->wrnblocks - (wrbend - startblock);
          ncopy  = rwb->wrmaxblocks - offset;
          if (ncopy > nblocks)
            {
              ncopy = nblocks;
            }

          dest = rwb->wrbuffer + offset * rwb->blocksize;
          memcpy(dest, wrbuffer, ncopy * rwb->blocksize);

          rwb->wrnblocks = offset + ncopy;
          wrbuffer      += ncopy * rwb->blocksize;
          startblock    += ncopy;
          nblocks       -= ncopy;
        }

      /* 5. We update a portion at the beginning of the write buffer */

      else /* if (rwb->wrblockstart >= startblock && wrbend >= newend) */
        {
          FAR uint8_t *dest;
          FAR const uint8_t *src;
          size_t ncopy;

          DEBUGASSERT(rwb->wrblockstart >= startblock && wrbend >= newend);

          /* Move the cached data to the end of the write buffer */

          ncopy = rwb->wrblockstart - startblock;
          if (ncopy > rwb->wrmaxblocks - rwb->wrnblocks)
            {
              ncopy = rwb->wrmaxblocks - rwb->wrnblocks;
            }

          dest = rwb->wrbuffer + ncopy * rwb->blocksize;
          memmove(dest, rwb->wrbuffer, ncopy * rwb->blocksize);

          rwb->wrblockstart -= ncopy;
          rwb->wrnblocks    += ncopy;

          /* Copy the data from the updating region to the beginning
           * of the write buffer.
           */

          ncopy = newend - rwb->wrblockstart;
          src   = wrbuffer + (nblocks - ncopy) * rwb->blocksize;
          memcpy(rwb->wrbuffer, src, ncopy * rwb->blocksize);

          nblocks -= ncopy;
        }
    }

  /* Use the block cache unless the buffer size is bigger than block cache */

  if (nblocks > rwb->wrmaxblocks)
    {
      ssize_t ret = rwb->wrflush(rwb->dev, wrbuffer, startblock, nblocks);
      if (ret < 0)
        {
          return ret;
        }
    }
  else if (nblocks)
    {
      /* Flush the write buffer */

      rwb_wrflush(rwb);

      /* Buffer the data in the write buffer */

      memcpy(rwb->wrbuffer, wrbuffer, nblocks * rwb->blocksize);
      rwb->wrblockstart = startblock;
      rwb->wrnblocks    = nblocks;
    }

  if (rwb->wrnblocks > 0)
    {
      rwb_wrstarttimeout(rwb);
    }

  return nwritten;
}
#endif

/****************************************************************************
 * Name: rwb_resetrhbuffer
 ****************************************************************************/

#ifdef CONFIG_DRVR_READAHEAD
static inline void rwb_resetrhbuffer(FAR struct rwbuffer_s *rwb)
{
  /* We assume that the caller holds the readAheadBufferSemaphore */

  rwb->rhnblocks    = 0;
  rwb->rhblockstart = -1;
}
#endif

/****************************************************************************
 * Name: rwb_bufferread
 ****************************************************************************/

#ifdef CONFIG_DRVR_READAHEAD
static inline void
rwb_bufferread(FAR struct rwbuffer_s *rwb,  off_t startblock,
               size_t nblocks, FAR uint8_t **rdbuffer)
{
  FAR uint8_t *rhbuffer;

  /* We assume that:
   * (1) the caller holds the readAheadBufferSemaphore, and
   * (2) the caller already knows that all of the blocks are in the
   *     read-ahead buffer.
   */

  /* Convert the units from blocks to bytes */

  off_t  blockoffset = startblock - rwb->rhblockstart;
  off_t  byteoffset  = rwb->blocksize * blockoffset;
  size_t nbytes      = rwb->blocksize * nblocks;

  /* Get the byte address in the read-ahead buffer */

  rhbuffer           = rwb->rhbuffer + byteoffset;

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
static int rwb_rhreload(FAR struct rwbuffer_s *rwb, off_t startblock)
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

      /* The return value is not the number of blocks we asked to be
       * loaded.
       */

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

      finfo("startblock=%" PRIdOFF " blockcount=%zu\n",
            startblock, blockcount);

      ret = rwb_lock(&rwb->wrlock);
      if (ret < 0)
        {
          return ret;
        }

      /* Now there are five cases:
       *
       * 1. We invalidate nothing
       */

      wrbend = rwb->wrblockstart + rwb->wrnblocks;
      invend = startblock + blockcount;

      if (wrbend <= startblock || rwb->wrblockstart >= invend)
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
       * 3. We invalidate a portion in the middle of the write buffer
       */

      else if (rwb->wrblockstart < startblock && wrbend > invend)
        {
          FAR uint8_t *src;
          off_t block;
          off_t offset;
          size_t nblocks;

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

      /* 4. We invalidate a portion at the end of the write buffer */

      else if (wrbend > startblock && wrbend <= invend)
        {
          rwb->wrnblocks -= wrbend - startblock;
          ret = OK;
        }

      /* 5. We invalidate a portion at the beginning of the write buffer */

      else /* if (rwb->wrblockstart >= startblock && wrbend > invend) */
        {
          FAR uint8_t *src;
          size_t ninval;
          size_t nkeep;

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

      rwb_unlock(&rwb->wrlock);
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
  int ret = OK;

  if (rwb->rhmaxblocks > 0 && rwb->rhnblocks > 0)
    {
      off_t rhbend;
      off_t invend;

      finfo("startblock=%" PRIdOFF " blockcount=%zu\n",
            startblock, blockcount);

      ret = rwb_lock(&rwb->rhlock);
      if (ret < 0)
        {
          return ret;
        }

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
          rwb->rhnblocks -= rhbend - startblock;
          ret = OK;
        }

      /* 4. We invalidate a portion at the begin of the read-ahead buffer */

      else /* if (rwb->rhblockstart >= startblock && rhbend > invend) */
        {
          FAR uint8_t *src;
          size_t ninval;
          size_t nkeep;

          DEBUGASSERT(rwb->rhblockstart >= startblock && rhbend > invend);
          /* Copy the data from the uninvalidated region to the beginning
           * of the read buffer.
           *
           * First calculate the source and destination of the transfer.
           */

          ninval = invend - rwb->rhblockstart;
          src    = rwb->rhbuffer + ninval * rwb->blocksize;

          /* Calculate the number of blocks we are keeping.  We keep
           * the ones that we don't invalidate.
           */

          nkeep  = rwb->rhnblocks - ninval;

          /* Then move the data that we are keeping to the beginning
           * the read buffer.
           */

          memmove(rwb->rhbuffer, src, nkeep * rwb->blocksize);

          /* Update the block info.  The first block is now the one just
           * after the invalidation region and the number buffered blocks
           * is the number that we kept.
           */

          rwb->rhblockstart = invend;
          rwb->rhnblocks    = nkeep;
        }

      rwb_unlock(&rwb->rhlock);
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

      if (rwb->wralignblocks == 0)
        {
          rwb->wralignblocks = 1;
        }

      DEBUGASSERT(rwb->wralignblocks <= rwb->wrmaxblocks &&
                  rwb->wrmaxblocks % rwb->wralignblocks == 0);

      /* Initialize the write buffer access mutex */

      nxmutex_init(&rwb->wrlock);

      /* Initialize write buffer parameters */

      rwb_resetwrbuffer(rwb);

      /* Allocate the write buffer */

      allocsize     = rwb->wrmaxblocks * rwb->blocksize;
      rwb->wrbuffer = kmm_malloc(allocsize);
      if (!rwb->wrbuffer)
        {
          ferr("Write buffer kmm_malloc(%" PRIu32 ") failed\n", allocsize);
          nxmutex_destroy(&rwb->wrlock);
          return -ENOMEM;
        }

      finfo("Write buffer size: %" PRIu32 " bytes\n", allocsize);
    }
#endif /* CONFIG_DRVR_WRITEBUFFER */

#ifdef CONFIG_DRVR_READAHEAD
  if (rwb->rhmaxblocks > 0)
    {
      finfo("Initialize the read-ahead buffer\n");

      /* Initialize the read-ahead buffer access mutex */

      nxmutex_init(&rwb->rhlock);

      /* Initialize read-ahead buffer parameters */

      rwb_resetrhbuffer(rwb);

      /* Allocate the read-ahead buffer */

      allocsize     = rwb->rhmaxblocks * rwb->blocksize;
      rwb->rhbuffer = kmm_malloc(allocsize);
      if (!rwb->rhbuffer)
        {
          ferr("Read-ahead buffer kmm_malloc(%" PRIu32 ") failed\n",
          allocsize);
          nxmutex_destroy(&rwb->rhlock);
#ifdef CONFIG_DRVR_WRITEBUFFER
          if (rwb->wrmaxblocks > 0)
            {
              nxmutex_destroy(&rwb->wrlock);
            }

          if (rwb->wrbuffer != NULL)
            {
              kmm_free(rwb->wrbuffer);
            }
#endif

          return -ENOMEM;
        }

      finfo("Read-ahead buffer size: %" PRIu32 " bytes\n", allocsize);
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
      rwb_wrflush(rwb);
      nxmutex_destroy(&rwb->wrlock);
      if (rwb->wrbuffer)
        {
          kmm_free(rwb->wrbuffer);
        }
    }
#endif

#ifdef CONFIG_DRVR_READAHEAD
  if (rwb->rhmaxblocks > 0)
    {
      nxmutex_destroy(&rwb->rhlock);
      if (rwb->rhbuffer)
        {
          kmm_free(rwb->rhbuffer);
        }
    }
#endif
}

/****************************************************************************
 * Name: rwb_read_
 ****************************************************************************/

static ssize_t rwb_read_(FAR struct rwbuffer_s *rwb, off_t startblock,
                         size_t nblocks, FAR uint8_t *rdbuffer)
{
  int ret = OK;

#ifdef CONFIG_DRVR_READAHEAD
  if (rwb->rhmaxblocks > 0)
    {
      size_t remaining;

      ret = rwb_lock(&rwb->rhlock);
      if (ret < 0)
        {
          return ret;
        }

      /* Loop until we have read all of the requested blocks */

      for (remaining = nblocks; remaining > 0; )
        {
          /* Is there anything in the read-ahead buffer? */

          if (rwb->rhnblocks > 0)
            {
              off_t bufferend;

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
                  ferr("ERROR: Failed to fill the read-ahead buffer: %d\n",
                       ret);

                  rwb_unlock(&rwb->rhlock);
                  return ret;
                }
            }
        }

      /* On success, return the number of blocks that we were requested to
       * read. This is for compatibility with the normal return of a block
       * driver read method
       */

      rwb_unlock(&rwb->rhlock);
      ret = nblocks;
    }
  else
#endif
    {
      /* No read-ahead buffering, (re)load the data directly into
       * the user buffer.
       */

      ret = rwb->rhreload(rwb->dev, rdbuffer, startblock, nblocks);
    }

  return ret;
}

/****************************************************************************
 * Name: rwb_read
 ****************************************************************************/

ssize_t rwb_read(FAR struct rwbuffer_s *rwb, off_t startblock,
                 size_t nblocks, FAR uint8_t *rdbuffer)
{
  int ret = OK;
  size_t readblocks = 0;

  finfo("startblock=%ld nblocks=%ld rdbuffer=%p\n",
        (long)startblock, (long)nblocks, rdbuffer);

#ifdef CONFIG_DRVR_WRITEBUFFER
  /* If the new read data overlaps any part of the write buffer, we
   * directly copy write buffer to read buffer. This boost performance.
   */

  if (rwb->wrmaxblocks > 0)
    {
      ret = rwb_lock(&rwb->wrlock);
      if (ret < 0)
        {
          return ret;
        }

      /* If the write buffer overlaps the block(s) requested */

      if (rwb_overlap(rwb->wrblockstart, rwb->wrnblocks, startblock,
                      nblocks))
        {
          size_t rdblocks = 0;
          size_t wrnpass  = 0;

          if (rwb->wrblockstart > startblock)
            {
              rdblocks = rwb->wrblockstart - startblock;
              ret = rwb_read_(rwb, startblock, rdblocks, rdbuffer);
              if (ret < 0)
                {
                  rwb_unlock(&rwb->wrlock);
                  return ret;
                }

              startblock += ret;
              nblocks    -= ret;
              rdbuffer   += ret * rwb->blocksize;
              readblocks += ret;
            }

          if (rwb->wrblockstart < startblock)
            {
              wrnpass = startblock - rwb->wrblockstart;
            }

          rdblocks = nblocks > (rwb->wrnblocks - wrnpass) ?
                    (rwb->wrnblocks - wrnpass) : nblocks;
          memcpy(rdbuffer, &rwb->wrbuffer[wrnpass * rwb->blocksize],
                rdblocks * rwb->blocksize);

          startblock += rdblocks;
          nblocks    -= rdblocks;
          rdbuffer   += rdblocks * rwb->blocksize;
          readblocks += rdblocks;
        }

      rwb_unlock(&rwb->wrlock);
    }
#endif

  ret = rwb_read_(rwb, startblock, nblocks, rdbuffer);
  if (ret < 0)
    {
      return ret;
    }

  return readblocks + ret;
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

      ret = rwb_lock(&rwb->rhlock);
      if (ret < 0)
        {
          return ret;
        }

      if (rwb_overlap(rwb->rhblockstart, rwb->rhnblocks, startblock,
                      nblocks))
        {
#ifdef CONFIG_DRVR_INVALIDATE
          /* Just invalidate the read buffer startblock + nblocks data */

          ret = rwb_invalidate_readahead(rwb, startblock, nblocks);
          if (ret < 0)
            {
              ferr("ERROR: rwb_invalidate_readahead failed: %d\n", ret);
              rwb_unlock(&rwb->rhlock);
              return ret;
            }
#else
          rwb_resetrhbuffer(rwb);
#endif
        }

      rwb_unlock(&rwb->rhlock);
    }
#endif

#ifdef CONFIG_DRVR_WRITEBUFFER
  if (rwb->wrmaxblocks > 0)
    {
      finfo("startblock=%" PRIdOFF " wrbuffer=%p\n", startblock, wrbuffer);

      ret = rwb_lock(&rwb->wrlock);
      if (ret < 0)
        {
          return ret;
        }

      ret = rwb_writebuffer(rwb, startblock, nblocks, wrbuffer);
      rwb_unlock(&rwb->wrlock);

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

  return ret;
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
  int ret;

#ifdef CONFIG_DRVR_WRITEBUFFER
  if (rwb->wrmaxblocks > 0)
    {
      ret = rwb_lock(&rwb->wrlock);
      if (ret < 0)
        {
          return ret;
        }

      rwb_resetwrbuffer(rwb);
      rwb_unlock(&rwb->wrlock);
    }
#endif

#ifdef CONFIG_DRVR_READAHEAD
  if (rwb->rhmaxblocks > 0)
    {
      ret = rwb_lock(&rwb->rhlock);
      if (ret < 0)
        {
          return ret;
        }

      rwb_resetrhbuffer(rwb);
      rwb_unlock(&rwb->rhlock);
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

/****************************************************************************
 * Name: rwb_flush
 *
 * Description:
 *   Flush the write buffer
 *
 ****************************************************************************/

#ifdef CONFIG_DRVR_WRITEBUFFER
int rwb_flush(FAR struct rwbuffer_s *rwb)
{
  int ret;

  ret = rwb_lock(&rwb->wrlock);
  rwb_wrcanceltimeout(rwb);
  rwb_wrflush(rwb);
  rwb_unlock(&rwb->wrlock);

  return ret;
}
#endif

#endif /* CONFIG_DRVR_WRITEBUFFER || CONFIG_DRVR_READAHEAD */
