/****************************************************************************
 * drivers/rwbuffer.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wqueue.h>
#include <nuttx/rwbuffer.h>

#if defined(CONFIG_FS_WRITEBUFFER) || defined(CONFIG_FS_READAHEAD)

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

#ifndef CONFIG_FS_WRDELAY
#  define CONFIG_FS_WRDELAY 350
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rwb_semtake
 ****************************************************************************/

static void rwb_semtake(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if
       * the wait was awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: rwb_semgive
 ****************************************************************************/

#define rwb_semgive(s) sem_post(s)

/****************************************************************************
 * Name: rwb_overlap
 ****************************************************************************/

static inline boolean rwb_overlap(off_t bufferstart1, size_t buffersize1,
                                  off_t bufferstart2, size_t buffersize2)
{
  off_t bufferend1 = bufferstart1 + buffersize1;
  off_t bufferend2 = bufferstart2 + buffersize2;

  /* If the buffer 1 is wholly outside of buffer 2, return false */

  if ((bufferend1   < bufferstart2) || /* Wholly "below" */
      (bufferstart1 > bufferend2))     /* Wholly "above" */
    {
      return FALSE;
    }
  else
    {
      return TRUE;
    }
}

/****************************************************************************
 * Name: rwb_resetwrbuffer
 ****************************************************************************/

#ifdef CONFIG_FS_WRITEBUFFER
static inline void rwb_resetwrbuffer(struct rwbuffer_s *rwb)
{
  /* We assume that the caller holds the wrsem */

  rwb->wrnbytes        = 0;
  rwb->wrblockstart    = (off_t)-1;
  rwb->wrexpectedblock = (off_t)-1;
}
#endif

/****************************************************************************
 * Name: rwb_wrflush
 ****************************************************************************/

#ifdef CONFIG_FS_WRITEBUFFER
static void rwb_wrflush(struct rwbuffer_s *rwb)
{
  int ret;

  fvdbg("Timeout!");
  
  rwb_semtake(&rwb->wrsem);
  if (rwb->wrnbytes)
    {
      fvdbg("Flushing: blockstart=0x%08lx nbytes=%d from buffer=%p",
      (long)rwb->wrblockstart, (long)rwb->wrnbytes, (long)rwb->wrbuffer);

      /* Flush cache */

      ret = rwb->wrflush(rwb->dev, rwb->wrbuffer, rwb->wrblockstart, rwb->wrnbytes);
      if (ret < 0)
        {
          fdbg("ERROR: Error flushing write buffer: %d\n", -ret);
        }
      
      rwb_resetwrbuffer(rwb);
    }

  rwb_semgive(&rwb->wrsem);
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

  /* If a timeout elpases with with write buffer activity, this watchdog
   * handler function will be evoked on the thread of execution of the
   * worker thread.
   */

  rwb_wrflush(rwb);
}

/****************************************************************************
 * Name: rwb_wrstarttimeout
 ****************************************************************************/

static void rwb_wrstarttimeout(FAR struct rwbuffer_s *rwb)
{
  /* CONFIG_FS_WRDELAY provides the delay period in milliseconds. CLK_TCK
   * provides the clock tick of the system (frequency in Hz).
   */

  int ticks = (CONFIG_FS_WRDELAY + CLK_TCK/2) / CLK_TCK;
  (void)work_queue(&rwb->work, rwb_wrtimeout, (FAR void *)rwb, ticks);
}

/****************************************************************************
 * Name: rwb_wrcanceltimeout
 ****************************************************************************/

static inline void rwb_wrcanceltimeout(struct rwbuffer_s *rwb)
{
  (void)work_cancel(&rwb->work);
}

/****************************************************************************
 * Name: rwb_writebuffer
 ****************************************************************************/

#ifdef CONFIG_FS_WRITEBUFFER
static ssize_t rwb_writebuffer(FAR struct rwbuffer_s *rwb,
                               off_t startblock, uint32 nbytes,
                               FAR const ubyte *wrbuffer)
{
  int ret;

  /* Write writebuffer Logic */

  rwb_wrcanceltimeout(rwb);
  
  /* First: Should we flush out our cache? */

  if (((startblock != rwb->wrexpectedblock) && (rwb->wrnbytes)) ||
      ((rwb->wrnbytes + nbytes) > rwb->wrallocsize))
    {
      fvdbg("writebuffer miss, expected: 0x%08x, given: 0x%08x",
      rwb->wrexpectedblock, startblock);
    
      /* Flush cache */

      ret = rwb->wrflush(rwb, rwb->wrbuffer, rwb->wrblockstart, rwb->wrnbytes);
      if (ret < 0)
        {
          fdbg("ERROR: Error writing multiple from cache: %d\n", -ret);
          return ret;
        }

      rwb_resetwrbuffer(rwb);
    }
  
  /* writebuffer is clear? Then initialize it */

  if (!rwb->wrnbytes)
    {
      fvdbg("Fresh cache starting at block: 0x%08x",startblock);
      rwb->wrblockstart = startblock;
    }
  
  /* Add data to cache */

  fvdbg("writebuffer: copying from 0x%p to 0x%p",
        wrbuffer, &rwb->wrbuffer[rwb->wrnbytes]);
  memcpy(&rwb->wrbuffer[rwb->wrnbytes], wrbuffer, nbytes);

  rwb->wrnbytes        += nbytes;
  rwb->wrexpectedblock = rwb->wrblockstart + rwb->wrnbytes;
  rwb_wrstarttimeout(rwb);
  return nbytes;
}
#endif

/****************************************************************************
 * Name: 
 ****************************************************************************/

#ifdef CONFIG_FS_READAHEAD
static inline void rwb_resetrhbuffer(struct rwbuffer_s *rwb)
{
  /* We assume that the caller holds the readAheadBufferSemphore */

  rwb->rhnbytes     = 0;
  rwb->rhblockstart = (off_t)-1;
}
#endif

/****************************************************************************
 * Name: 
 ****************************************************************************/

#ifdef CONFIG_FS_READAHEAD
static inline void
rwb_bufferread(struct rwbuffer_s *rwb,  off_t startblock,
               size_t nblocks, ubyte **rdbuffer)
{
  /* We assume that (1) the caller holds the readAheadBufferSemphore, and (2)
   * that the caller already knows that all of the blocks are in the
   * read-ahead buffer.
   */

  /* Convert the units from blocks to bytes */

  uint32 dwStartBlockOffset = startblock - rwb->rhblockstart;
  uint32 dwStartByteOffset  = rwb->blocksize * dwStartBlockOffset;
  uint32 dwByteCount        = rwb->blocksize * nblocks;

  /* Get the byte address in the read-ahead buffer */

  ubyte *pchBuffer          = rwb->rhbuffer + dwStartByteOffset;

  /* Copy the data from the read-ahead buffer into the IO buffer */

  memcpy(*rdbuffer, pchBuffer, dwByteCount);

  /* Update the caller's copy for the next address */

  *rdbuffer += dwByteCount;
}
#endif

/****************************************************************************
 * Name: 
 ****************************************************************************/

#ifdef CONFIG_FS_READAHEAD
static int rwb_rhreload(struct rwbuffer_s *rwb, off_t startblock)
{
  /* Get the maximum number of blocks that will fit in the read-ahead buffer */

  size_t maxblocks = rwb->rhallocsize / rwb->blocksize;
  off_t  endblock  = startblock + maxblocks;
  size_t nblocks;
  int ret;

  /* Reset the read buffer */

  rwb_resetrhbuffer(rwb);

  /* Make sure that we don't read past the end of the device */

  if (endblock > rwb->nblocks)
    {
      endblock = rwb->nblocks;
    }
  nblocks = endblock - startblock;

  /* Now perform the read */

  ret = rwb->rhreload(rwb->dev, rwb->rhbuffer, startblock, nblocks);
  if (ret == 0)
    {
      /* Update information about what is in the read-ahead buffer */

      rwb->rhnbytes   = nblocks;
      rwb->rhblockstart = startblock;
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
  /* Sanity checking */

  DEBUGASSERT(rwb != NULL);
  DEBUGASSERT(rwb->blocksize > 0);
  DEBUGASSERT(rwb->nblocks > 0);
  DEBUGASSERT(rwb->dev != NULL);

  /* Setup so that rwb_uninitialize can handle a failure */

#ifdef CONFIG_FS_WRITEBUFFER
  DEBUGASSERT(rwb->wrblocks > 0);
  DEBUGASSERT(rwb->wrflush!= NULL);
  rwb->wrbuffer = NULL;
#endif
#ifdef CONFIG_FS_READAHEAD
  DEBUGASSERT(rwb->rhnblocks > 0);
  DEBUGASSERT(rwb->rhreload != NULL);
  rwb->rhbuffer = NULL;
#endif

#ifdef CONFIG_FS_WRITEBUFFER
  fvdbg("Initialize the write buffer");

  /* Initialize the write buffer access semaphore */

  sem_init(&rwb->wrsem, 0, 1);

  /* Initialize write buffer parameters */

  rwb_resetwrbuffer(rwb);

  /* Allocate the write buffer */

  rwb->wrallocsize = rwb->wrblocks * rwb->blocksize;
  rwb->wrbuffer    = malloc(rwb->wrallocsize);
  if (!rwb->wrbuffer)
    {
      fdbg("Write buffer malloc(%ld) failed\n", (long)rwb->wrallocsize);
      return -ENOMEM;
    }
  
  fvdbg("Write buffer size: %d bytes\n", rwb->wrallocsize);
#endif /* CONFIG_FS_WRITEBUFFER */

#ifdef CONFIG_FS_READAHEAD
  fvdbg("Initialize the read-ahead buffer");

  /* Initialize the read-ahead buffer access semaphore */

  sem_init(&rwb->rhsem, 0, 1);

  /* Initialize read-ahead buffer parameters */

  rwb_resetrhbuffer(rwb);

  /* Allocate the read-ahead buffer */

  rwb->rhallocsize = rwb->rhblocks * rwb->blocksize;
  rwb->rhbuffer    = malloc(rwb->rhallocsize);
  if (!rwb->rhbuffer)
    {
      fdbg("Read-ahead buffer malloc(%ld) failed\n", (long)rwb->rhallocsize);
      return -ENOMEM;
    }
  
  fvdbg("Read-ahead buffer size: %d bytes\n", rwb->rhallocsize);
#endif /* CONFIG_FS_READAHEAD */
  return 0;
}

/****************************************************************************
 * Name: rwb_uninitialize
 ****************************************************************************/

void rwb_uninitialize(FAR struct rwbuffer_s *rwb)
{
#ifdef CONFIG_FS_WRITEBUFFER
  rwb_wrcanceltimeout(rwb);
  sem_destroy(&rwb->wrsem);
  if (rwb->wrbuffer)
    {
      free(rwb->wrbuffer);
    }
#endif

#ifdef CONFIG_FS_READAHEAD
  sem_destroy(&rwb->rhsem);
  if (rwb->rhbuffer)
    {
      free(rwb->rhbuffer);
    }
#endif
}

/****************************************************************************
 * Name: 
 ****************************************************************************/

int rwb_read(FAR struct rwbuffer_s *rwb, off_t startblock, uint32 blockcount,
             FAR ubyte *rdbuffer)
{
  fvdbg("startblock=%ld blockcount=%ld rdbuffer=%p",
        (long)startblock, (long)blockcount, rdbuffer);

#ifdef CONFIG_FS_WRITEBUFFER
  /* If the new read data overlaps any part of the write buffer, then
   * flush the write data onto the physical media before reading.  We
   * could attempt some more exotic handling -- but this simple logic
   * is well-suited for simple streaming applications.
   */

  /* If the write buffer overlaps the block(s) requested, then flush the
   * write buffer.
   */

  rwb_semtake(&rwb->wrsem);
  if (rwb_overlap(rwb->wrblockstart, rwb->wrnbytes, startblock, blockcount))
    {
      rwb_wrflush(rwb);
    }
  rwb_semgive(&rwb->wrsem);
#endif

#ifdef CONFIG_FS_READAHEAD
  /* Loop until we have read all of the requested blocks */

  rwb_semtake(&rwb->rhsem);
  while (blockcount > 0)
    {
      /* Is there anything in the read-ahead buffer? */

      if (rwb->rhnbytes > 0)
        {
          off_t  startblock = startblock;
          size_t nblocks    = 0;
          off_t  bufferend;

          /* Loop for each block we find in the read-head buffer.   Count the
           * number of buffers that we can read from read-ahead buffer.
           */

          bufferend = rwb->rhblockstart + rwb->rhnbytes;

          while ((startblock >= rwb->rhblockstart) &&
                 (startblock < bufferend) &&
                 (blockcount > 0))
            {
              /* This is one more that we will read from the read ahead buffer */

              nblocks++;

              /* And one less that we will read from the media */

              startblock++;
              blockcount--;
            }

          /* Then read the data from the read-ahead buffer */

          rwb_bufferread(rwb, startblock, nblocks, &rdbuffer);
        }

      /* If we did not get all of the data from the buffer, then we have to refill
       * the buffer and try again.
       */

      if (blockcount > 0)
        {
          int ret = rwb_rhreload(rwb, startblock);
          if (ret < 0)
            {
              fdbg("ERROR: Failed to fill the read-ahead buffer: %d", -ret);
              return ret;
            }
        }
    }

  rwb_semgive(&rwb->rhsem);
  return 0;
#else
  return rwb->rhreload(rwb->dev, startblock, blockcount, rdbuffer);
#endif
}

/****************************************************************************
 * Name: rwb_write
 ****************************************************************************/

int rwb_write(FAR struct rwbuffer_s *rwb, off_t startblock,
              size_t blockcount, FAR const ubyte *wrbuffer)
{
  int ret;

#ifdef CONFIG_FS_READAHEAD
  /* If the new write data overlaps any part of the read buffer, then
   * flush the data from the read buffer.  We could attempt some more
   * exotic handling -- but this simple logic is well-suited for simple
   * streaming applications.
   */

  rwb_semtake(&rwb->rhsem);
  if (rwb_overlap(rwb->rhblockstart, rwb->rhnbytes, startblock, blockcount))
    {
      rwb_resetrhbuffer(rwb);
    }
  rwb_give(&rwb->rhsem);
#endif

#ifdef CONFIG_FS_WRITEBUFFER
  fvdbg("startblock=%d wrbuffer=%p", startblock, wrbuffer);

  /* Use the block cache unless the buffer size is bigger than block cache */   
     
  if (blockcount > rwb->wrallocsize)
    {
      /* First flush the cache */

      rwb_semtake(&rwb->wrsem);
      rwb_wrflush(rwb);
      rwb_semgive(&rwb->wrsem);

      /* Then transfer the data directly to the media */

      ret = rwb->wrflush(rwb->dev, startblock, blockcount, wrbuffer);
    }
  else
    {
      /* Buffer the data in the write buffer */

      ret = rwb_writebuffer(rwb, startblock, blockcount, wrbuffer);
    }
  return ret;

#else

  return rwb->wrflush(rwb->dev, startblock, blockcount, wrbuffer);

#endif
}

/****************************************************************************
 * Name: rwb_mediaremoved
 ****************************************************************************/

/* The following function is called when media is removed */

int rwb_mediaremoved(FAR struct rwbuffer_s *rwb)
{
#ifdef CONFIG_FS_WRITEBUFFER
  rwb_semtake(&rwb->wrsem);
  rwb_resetwrbuffer(rwb);
  rwb_semgive(&rwb->wrsem);
#endif

#ifdef CONFIG_FS_READAHEAD
  rwb_semtake(&rwb->rhsem);
  rwb_resetrhbuffer(rwb);
  rwb_semgive(&rwb->rhsem);
#endif
  return 0;
}

#endif /* CONFIG_FS_WRITEBUFFER || CONFIG_FS_READAHEAD */

