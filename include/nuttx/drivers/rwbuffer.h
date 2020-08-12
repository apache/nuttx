/****************************************************************************
 * include/nuttx/drivers/rwbuffer.h
 *
 *   Copyright (C) 2009, 2014, 2020 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_DRIVERS_RWBUFFER_H
#define __INCLUDE_NUTTX_DRIVERS_RWBUFFER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>

#if defined(CONFIG_DRVR_WRITEBUFFER) || defined(CONFIG_DRVR_READAHEAD)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Data transfer callouts.  These must be provided by the block driver
 * logic in order to flush the write buffer when appropriate or to
 * reload the read-ahead buffer, when appropriate.
 */

typedef CODE ssize_t (*rwbreload_t)(FAR void *dev, FAR uint8_t *buffer,
                                    off_t startblock, size_t nblocks);
typedef CODE ssize_t (*rwbflush_t)(FAR void *dev, FAR const uint8_t *buffer,
                                   off_t startblock, size_t nblocks);

/* This structure holds the state of the buffers.  In typical usage,
 * an instance of this structure is declared within each block driver
 * status structure like:
 *
 * struct foo_dev_s
 * {
 *   ...
 *   struct rwbuffer_s rwbuffer;
 *   ...
 * };
 *
 * Note that this supports buffering for multiple block devices or for
 * multiple instances of same block device, because each rwbuffer instance
 * supports independent buffering.
 *
 * A reference to the struct rwbuffer_s instance is then passed to each
 * interface like:
 *
 *  FAR struct foo_dev_s *priv;
 *  ...
 *  ... [Setup blocksize, nblocks, dev, wrmaxblocks, wrflush,
 *       rhmaxblocks, rhreload] ...
 *  ret = rwb_initialize(&priv->rwbuffer);
 */

struct rwbuffer_s
{
  /**************************************************************************/

  /* These values must be provided by the user prior to calling
   * rwb_initialize()
   */

  /* Supported geometry */

  uint16_t      blocksize;       /* The size of one block */
  size_t        nblocks;         /* The total number blocks supported */

  /* Read-ahead/Write buffer sizes.  Buffering can be disabled (even if it
   * is enabled in the configuration) by setting the buffer size to zero
   * blocks.
   */

#ifdef CONFIG_DRVR_WRITEBUFFER
  uint16_t      wrmaxblocks;     /* The number of blocks to buffer in memory */
  uint16_t      wralignblocks;   /* The buffer to be flash is always multiplied by this
                                  * number. It must be 0 or divisible by wrmaxblocks.
                                  */
#endif
#ifdef CONFIG_DRVR_READAHEAD
  uint16_t      rhmaxblocks;     /* The number of blocks to buffer in memory */
#endif

  /* Callback functions.
   *
   * wrflush.  This callback is normally used to flush the contents of
   *   the write buffer.  If write buffering is disabled, then this
   *   function will instead be used to perform unbuffered writes.
   * rhrelad.  This callback is normally used to read new data into the
   *   read-ahead buffer.  If read-ahead buffering is disabled, then this
   *   function will instead be used to perform unbuffered reads.
   */

  FAR void     *dev;             /* Device state passed to callout functions */
  rwbflush_t    wrflush;         /* Callout to flush the write buffer */
  rwbreload_t   rhreload;        /* Callout to reload the read-ahead buffer */

  /**************************************************************************/

  /* The user should never modify any of the remaining fields */

  /* This is the state of the write buffering */

#ifdef CONFIG_DRVR_WRITEBUFFER
  sem_t         wrsem;           /* Enforces exclusive access to the write buffer */
  struct work_s work;            /* Delayed work to flush buffer after a delay with no activity */
  uint8_t      *wrbuffer;        /* Allocated write buffer */
  uint16_t      wrnblocks;       /* Number of blocks in write buffer */
  off_t         wrblockstart;    /* First block in write buffer */
#endif

  /* This is the state of the read-ahead buffering */

#ifdef CONFIG_DRVR_READAHEAD
  sem_t         rhsem;           /* Enforces exclusive access to the write buffer */
  uint8_t      *rhbuffer;        /* Allocated read-ahead buffer */
  uint16_t      rhnblocks;       /* Number of blocks in read-ahead buffer */
  off_t         rhblockstart;    /* First block in read-ahead buffer */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Buffer initialization */

int rwb_initialize(FAR struct rwbuffer_s *rwb);
void rwb_uninitialize(FAR struct rwbuffer_s *rwb);

/* Block oriented transfers */

ssize_t rwb_read(FAR struct rwbuffer_s *rwb, off_t startblock,
                 size_t blockcount, FAR uint8_t *rdbuffer);
ssize_t rwb_write(FAR struct rwbuffer_s *rwb,
                  off_t startblock, size_t blockcount,
                  FAR const uint8_t *wrbuffer);

/* Character oriented transfers */

#ifdef CONFIG_DRVR_READBYTES
ssize_t rwb_readbytes(FAR struct rwbuffer_s *dev, off_t offset,
                      size_t nbytes, FAR uint8_t *buffer);
#endif

/* Media events */

#ifdef CONFIG_DRVR_REMOVABLE
int rwb_mediaremoved(FAR struct rwbuffer_s *rwb);
#endif

#ifdef CONFIG_DRVR_INVALIDATE
int rwb_invalidate(FAR struct rwbuffer_s *rwb,
                   off_t startblock, size_t blockcount);
#endif

#ifdef CONFIG_DRVR_WRITEBUFFER
int rwb_flush(FAR struct rwbuffer_s *rwb);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_DRVR_WRITEBUFFER || CONFIG_DRVR_READAHEAD */
#endif /* __INCLUDE_NUTTX_DRIVERS_RWBUFFER_H */
