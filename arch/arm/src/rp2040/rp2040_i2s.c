/****************************************************************************
 * arch/arm/src/rp2040/rp2040_i2s.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/spi/spi.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "rp2040_gpio.h"
#include "rp2040_dmac.h"
#include "rp2040_i2s_pio.h"

#ifdef CONFIG_RP2040_I2S

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO required by this driver
#endif

#ifndef CONFIG_RP2040_I2S_MAXINFLIGHT
#  define CONFIG_RP2040_I2S_MAXINFLIGHT 16
#endif

/* Debug ********************************************************************/

/* Check if SSC debug is enabled (non-standard.. no support in
 * include/debug.h
 */

#ifndef CONFIG_DEBUG_I2S_INFO
#  undef CONFIG_RP2040_I2S_DUMPBUFFERS
#endif

/* The I2S can handle most any bit width from 8 to 32.  However, the DMA
 * logic here is constrained to byte, half-word, and word sizes.
 */

#ifndef CONFIG_RP2040_I2S_DATALEN
#  define CONFIG_RP2040_I2S_DATALEN 16
#endif

#if CONFIG_RP2040_I2S_DATALEN == 8
#  define RP2040_I2S_DATAMASK  0
#elif CONFIG_RP2040_I2S_DATALEN == 16
#  define RP2040_I2S_DATAMASK  1
#elif  CONFIG_RP2040_I2S_DATALEN < 8 || CONFIG_RP2040_I2S_DATALEN > 16
#  error Invalid value for CONFIG_RP2040_I2S_DATALEN
#else
#  error Valid but supported value for CONFIG_RP2040_I2S_DATALEN
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2S buffer container */

struct rp2040_buffer_s
{
  struct rp2040_buffer_s *flink; /* Supports a singly linked list */
  i2s_callback_t callback;       /* Function to call when the transfer
                                  * completes */
  uint32_t timeout;              /* The timeout value to use with DMA
                                  * transfers */
  void *arg;                     /* The argument to be returned with the
                                  * callback */
  struct ap_buffer_s *apb;       /* The audio buffer */
  int result;                    /* The result of the transfer */
};

/* This structure describes the state of one receiver or transmitter
 * transport.
 */

struct rp2040_transport_s
{
  DMA_HANDLE dma;               /* I2S DMA handle */
  struct wdog_s dog;            /* Watchdog that handles DMA timeouts */
  sq_queue_t pend;              /* A queue of pending transfers */
  sq_queue_t act;               /* A queue of active transfers */
  sq_queue_t done;              /* A queue of completed transfers */
  struct work_s work;           /* Supports worker thread operations */
  uint32_t timeout;             /* Current DMA timeout value */
};

/* The state of the one I2S peripheral */

struct rp2040_i2s_s
{
  struct i2s_dev_s  dev;            /* Externally visible I2S interface */
  mutex_t           lock;           /* Assures mutually exclusive access to I2S */
  bool              initialized;    /* Has I2S interface been initialized */
  uint8_t           datalen;        /* Data width (8 or 16) */
#ifdef CONFIG_DEBUG_FEATURES
  uint8_t           align;          /* Log2 of data width (0 or 1) */
#endif
  uint32_t          samplerate;     /* Data sample rate */
  uint32_t          channels;       /* Audio channels (1:mono or 2:stereo) */
  dma_config_t      txconfig;       /* TX DMA configuration */
  struct rp2040_transport_s tx;     /* TX transport state */

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                     /* Buffer wait semaphore */
  struct rp2040_buffer_s *freelist; /* A list a free buffer containers */
  struct rp2040_buffer_s containers[CONFIG_RP2040_I2S_MAXINFLIGHT];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register helpers */

#ifdef CONFIG_RP2040_I2S_DUMPBUFFERS
#  define       i2s_dump_buffer(m,b,s) lib_dumpbuffer(m,b,s)
#else
#  define       i2s_dump_buffer(m,b,s)
#endif

/* Buffer container helpers */

static struct rp2040_buffer_s *
                i2s_buf_allocate(struct rp2040_i2s_s *priv);
static void     i2s_buf_free(struct rp2040_i2s_s *priv,
                             struct rp2040_buffer_s *bfcontainer);
static void     i2s_buf_initialize(struct rp2040_i2s_s *priv);

/* DMA support */

static void     i2s_txdma_timeout(wdparm_t arg);
static int      i2s_txdma_setup(struct rp2040_i2s_s *priv);
static void     i2s_tx_worker(void *arg);
static void     i2s_tx_schedule(struct rp2040_i2s_s *priv, int result);
static void     i2s_txdma_callback(DMA_HANDLE handle, uint8_t result,
                                   void *arg);

/* I2S methods (and close friends) */

static int      i2s_checkwidth(struct rp2040_i2s_s *priv, int bits);

static int      rp2040_i2s_txchannels(struct i2s_dev_s *dev,
                                      uint8_t channels);
static uint32_t rp2040_i2s_txsamplerate(struct i2s_dev_s *dev,
                                        uint32_t rate);
static uint32_t rp2040_i2s_txdatawidth(struct i2s_dev_s *dev, int bits);
static int      rp2040_i2s_send(struct i2s_dev_s *dev,
                                struct ap_buffer_s *apb,
                                i2s_callback_t callback, void *arg,
                                uint32_t timeout);
static int      rp2040_i2s_ioctl(struct i2s_dev_s *dev, int cmd,
                                 unsigned long arg);

/* Initialization */

static int      i2s_dma_flags(struct rp2040_i2s_s *priv);
static int      i2s_dma_allocate(struct rp2040_i2s_s *priv);
static void     i2s_dma_free(struct rp2040_i2s_s *priv);
static void     i2s_configure(struct rp2040_i2s_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2S device operations */

static const struct i2s_ops_s g_i2sops =
{
  .i2s_txchannels   = rp2040_i2s_txchannels,
  .i2s_txsamplerate = rp2040_i2s_txsamplerate,
  .i2s_txdatawidth  = rp2040_i2s_txdatawidth,
  .i2s_send         = rp2040_i2s_send,
  .i2s_ioctl        = rp2040_i2s_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2s_buf_allocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - I2S state instance
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive access to the I2S state structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct rp2040_buffer_s *i2s_buf_allocate(struct rp2040_i2s_s *priv)
{
  struct rp2040_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  ret = nxsem_wait_uninterruptible(&priv->bufsem);
  if (ret < 0)
    {
      return NULL;
    }

  /* Get the buffer from the head of the free list */

  flags = enter_critical_section();
  bfcontainer = priv->freelist;
  DEBUGASSERT(bfcontainer);

  /* Unlink the buffer from the freelist */

  priv->freelist = bfcontainer->flink;
  leave_critical_section(flags);
  return bfcontainer;
}

/****************************************************************************
 * Name: i2s_buf_free
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv - I2S state instance
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the I2S state structure
 *
 ****************************************************************************/

static void i2s_buf_free(struct rp2040_i2s_s *priv,
                         struct rp2040_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list */

  flags = enter_critical_section();
  bfcontainer->flink  = priv->freelist;
  priv->freelist = bfcontainer;
  leave_critical_section(flags);

  /* Wake up any threads waiting for a buffer container */

  nxsem_post(&priv->bufsem);
}

/****************************************************************************
 * Name: i2s_buf_initialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - I2S state instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in I2S initialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static void i2s_buf_initialize(struct rp2040_i2s_s *priv)
{
  int i;

  priv->freelist = NULL;
  nxsem_init(&priv->bufsem, 0, CONFIG_RP2040_I2S_MAXINFLIGHT);

  for (i = 0; i < CONFIG_RP2040_I2S_MAXINFLIGHT; i++)
    {
      i2s_buf_free(priv, &priv->containers[i]);
    }
}

/****************************************************************************
 * Name: i2s_txdma_timeout
 *
 * Description:
 *   The TX watchdog timeout without completion of the TX DMA.
 *
 * Input Parameters:
 *   arg - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void i2s_txdma_timeout(wdparm_t arg)
{
  struct rp2040_i2s_s *priv = (struct rp2040_i2s_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Cancel the DMA */

  rp2040_dmastop(priv->tx.dma);

  /* Then schedule completion of the transfer to occur on the worker thread.
   */

  i2s_tx_schedule(priv, -ETIMEDOUT);
}

/****************************************************************************
 * Name: i2s_txdma_setup
 *
 * Description:
 *   Setup and initiate the next TX DMA transfer
 *
 * Input Parameters:
 *   priv - I2S state instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

static int i2s_txdma_setup(struct rp2040_i2s_s *priv)
{
  struct rp2040_buffer_s *bfcontainer;
  struct ap_buffer_s *apb;
  uintptr_t samp;
  uint32_t timeout;
  apb_samp_t nbytes;
  int ret;

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (!sq_empty(&priv->tx.act))
    {
      return OK;
    }

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->tx.pend))
    {
      return OK;
    }

  /* Adding the pending DMA */

  /* Remove the pending TX transfer at the head of the TX pending
   * queue.
   */

  bfcontainer = (struct rp2040_buffer_s *)sq_remfirst(&priv->tx.pend);
  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  apb = bfcontainer->apb;

  /* Get the transfer information, accounting for any data offset */

  samp   = (uintptr_t)&apb->samp[apb->curbyte];
  nbytes = apb->nbytes - apb->curbyte;
#ifdef CONFIG_DEBUG_FEATURES
  DEBUGASSERT((samp & priv->align) == 0 && (nbytes & priv->align) == 0);
#endif

  /* Configure DMA stream */

  rp2040_txdmasetup(priv->tx.dma,
                    rp2040_i2s_pio_getdmaaddr(),
                    (uint32_t)samp, nbytes,
                    priv->txconfig);

  timeout = bfcontainer->timeout;

  /* Add the container to the list of active DMAs */

  sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.act);

  /* Start the DMA, saving the container as the current active transfer */

  rp2040_dmastart(priv->tx.dma, i2s_txdma_callback, priv);
  rp2040_i2s_pio_enable(true);

  /* Start a watchdog to catch DMA timeouts */

  if (timeout > 0)
    {
      ret = wd_start(&priv->tx.dog, timeout,
                     i2s_txdma_timeout, (wdparm_t)priv);

      priv->tx.timeout = timeout;

      /* Check if we have successfully started the watchdog timer.  Note
       * that we do nothing in the case of failure to start the timer.  We
       * are already committed to the DMA anyway.  Let's just hope that the
       * DMA does not hang.
       */

      if (ret < 0)
        {
          i2serr("ERROR: wd_start failed: %d\n", ret);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: i2s_tx_worker
 *
 * Description:
 *   TX transfer done worker
 *
 * Input Parameters:
 *   arg - the I2S device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2s_tx_worker(void *arg)
{
  struct rp2040_i2s_s *priv = (struct rp2040_i2s_s *)arg;
  struct rp2040_buffer_s *bfcontainer;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer containers were removed
   * from the tx.pend queue and saved in the tx.act queue.  We get here when
   * the DMA is finished... either successfully, with a DMA error, or with a
   * DMA timeout.
   *
   * In any case, the buffer containers in tx.act will be moved to the end
   * of the tx.done queue and tx.act will be emptied before this worker is
   * started.
   */

  i2sinfo("tx.act.head=%p tx.done.head=%p\n",
           priv->tx.act.head, priv->tx.done.head);

  /* Check if the DMA is IDLE */

  if (sq_empty(&priv->tx.act))
    {
      /* Then start the next DMA.  This must be done with interrupts
       * disabled.
       */

      flags = enter_critical_section();
      i2s_txdma_setup(priv);
      leave_critical_section(flags);
    }

  /* Process each buffer in the tx.done queue */

  while (sq_peek(&priv->tx.done) != NULL)
    {
      /* Remove the buffer container from the tx.done queue.  NOTE that
       * interrupts must be enabled to do this because the tx.done queue is
       * also modified from the interrupt level.
       */

      flags = enter_critical_section();
      bfcontainer = (struct rp2040_buffer_s *)sq_remfirst(&priv->tx.done);
      leave_critical_section(flags);

      /* Perform the TX transfer done callback */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release our reference on the audio buffer.  This may very likely
       * cause the audio buffer to be freed.
       */

      apb_free(bfcontainer->apb);

      /* And release the buffer container */

      i2s_buf_free(priv, bfcontainer);
    }
}

/****************************************************************************
 * Name: i2s_tx_schedule
 *
 * Description:
 *   An TX DMA completion or timeout has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   priv - I2S state instance
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - The TX timeout has been canceled.
 *
 ****************************************************************************/

static void i2s_tx_schedule(struct rp2040_i2s_s *priv, int result)
{
  struct rp2040_buffer_s *bfcontainer;
  int ret;

  /* Upon entry, the transfer(s) that just completed are the ones in the
   * priv->tx.act queue.
   */

  /* Move all entries from the tx.act queue to the tx.done queue */

  while (!sq_empty(&priv->tx.act))
    {
      /* Remove the next buffer container from the tx.act list */

      bfcontainer = (struct rp2040_buffer_s *)sq_remfirst(&priv->tx.act);

      /* Report the result of the transfer */

      bfcontainer->result = result;

      /* Add the completed buffer container to the tail of the tx.done
       * queue
       */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.done);
    }

  /* If the worker has completed running, then reschedule the working thread.
   * REVISIT:  There may be a race condition here.  So we do nothing is the
   * worker is not available.
   */

  if (work_available(&priv->tx.work))
    {
      /* Schedule the TX DMA done processing to occur on the worker thread. */

      ret = work_queue(HPWORK, &priv->tx.work, i2s_tx_worker, priv, 0);
      if (ret != 0)
        {
          i2serr("ERROR: Failed to queue TX work: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: i2s_txdma_callback
 *
 * Description:
 *   This callback function is invoked at the completion of the I2S TX DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   result - The result of the DMA transfer
 *   arg - A pointer to the chip select struction
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2s_txdma_callback(DMA_HANDLE handle, uint8_t result, void *arg)
{
  struct rp2040_i2s_s *priv = (struct rp2040_i2s_s *)arg;
  DEBUGASSERT(priv != NULL);

  /* Cancel the watchdog timeout */

  if (priv->tx.timeout > 0)
    {
      wd_cancel(&priv->tx.dog);
    }

  /* Then schedule completion of the transfer to occur on the worker thread */

  i2s_tx_schedule(priv, result);
}

/****************************************************************************
 * Name: i2s_checkwidth
 *
 * Description:
 *   Check for a valid bit width.  The I2S is capable of handling most any
 *   bit width from 8 to 16, but the DMA logic in this driver is constrained
 *   to 8- and 16-bit data widths
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   bits - The I2S data with in bits.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_checkwidth(struct rp2040_i2s_s *priv, int bits)
{
  /* The I2S can handle most any bit width from 8 to 32.  However, the DMA
   * logic here is constrained to byte, half-word, and word sizes.
   */

  switch (bits)
    {
    case 8:
#ifdef CONFIG_DEBUG
      priv->align = 0;
#endif
      break;

    case 16:
#ifdef CONFIG_DEBUG
      priv->align = 1;
#endif
      break;

    default:
      i2serr("ERROR: Unsupported or invalid data width: %d\n", bits);
      return (bits < 8 || bits > 16) ? -EINVAL : -ENOSYS;
    }

  /* Save the new data width */

  priv->datalen = bits;
  return OK;
}

/****************************************************************************
 * Name: rp2040_i2s_txchannels
 *
 * Description:
 *   Set the I2S TX number of channels.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   channels - The I2S numbers of channels
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rp2040_i2s_txchannels(struct i2s_dev_s *dev, uint8_t channels)
{
  struct rp2040_i2s_s *priv = (struct rp2040_i2s_s *)dev;

  if (channels != 1 && channels != 2)
    {
      return -EINVAL;
    }

  priv->channels = channels;
  return OK;
}

/****************************************************************************
 * Name: rp2040_i2s_txsamplerate
 *
 * Description:
 *   Set the I2S TX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static uint32_t rp2040_i2s_txsamplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct rp2040_i2s_s *priv = (struct rp2040_i2s_s *)dev;

  DEBUGASSERT(priv && priv->samplerate >= 0 && rate > 0);

  if (rate < 8000)
    {
      return -EINVAL;
    }

  priv->samplerate = rate;
  return 0;
}

/****************************************************************************
 * Name: rp2040_i2s_txdatawidth
 *
 * Description:
 *   Set the I2S TX data width.  The TX bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   bits - The I2S data with in bits.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static uint32_t rp2040_i2s_txdatawidth(struct i2s_dev_s *dev, int bits)
{
  struct rp2040_i2s_s *priv = (struct rp2040_i2s_s *)dev;
  int ret;

  i2sinfo("Data width bits of tx = %d\n", bits);
  DEBUGASSERT(priv && bits > 1);

  /* Check if this is a bit width that we are configured to handle */

  ret = i2s_checkwidth(priv, bits);
  if (ret < 0)
    {
      i2serr("ERROR: i2s_checkwidth failed: %d\n", ret);
      return 0;
    }

  /* Update the DMA flags */

  ret = i2s_dma_flags(priv);
  if (ret < 0)
    {
      i2serr("ERROR: i2s_dma_flags failed: %d\n", ret);
      return 0;
    }

  return 0;
}

/****************************************************************************
 * Name: rp2040_i2s_send
 *
 * Description:
 *   Send a block of data on I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer from which to send data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

static int rp2040_i2s_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                    i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct rp2040_i2s_s *priv = (struct rp2040_i2s_s *)dev;
  struct rp2040_buffer_s *bfcontainer;
  irqstate_t flags;
  int ret;

  /* Make sure that we have valid pointers that that the data has uint32_t
   * alignment.
   */

  DEBUGASSERT(priv && apb);
  i2sinfo("apb=%p nbytes=%d arg=%p timeout=%" PRId32 "\n",
          apb, apb->nbytes - apb->curbyte, arg, timeout);

  i2s_dump_buffer("Sending", &apb->samp[apb->curbyte],
                  apb->nbytes - apb->curbyte);
#ifdef CONFIG_DEBUG_FEATURES
  DEBUGASSERT(((uintptr_t)&apb->samp[apb->curbyte] & priv->align) == 0);
#endif

  /* Allocate a buffer container in advance */

  bfcontainer = i2s_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the I2S driver data */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      goto errout_with_buf;
    }

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /* Initialize the buffer container structure */

  bfcontainer->callback = (void *)callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  /* Add the buffer container to the end of the TX pending queue */

  flags = enter_critical_section();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->tx.pend);

  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);
  return OK;

errout_with_buf:
  i2s_buf_free(priv, bfcontainer);
  return ret;
}

/****************************************************************************
 * Name: rp2040_i2s_cleanup_queues
 *
 * Description:
 *   Clean up the all buffers in the queues.
 *
 * Input Parameters:
 *   priv - I2S state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2s_cleanup_queues(struct rp2040_i2s_s *priv)
{
  irqstate_t flags;
  struct rp2040_buffer_s *bfcontainer;

  while (sq_peek(&priv->tx.done) != NULL)
    {
      flags = enter_critical_section();
      bfcontainer = (struct rp2040_buffer_s *)sq_remfirst(&priv->tx.done);
      leave_critical_section(flags);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, OK);
      apb_free(bfcontainer->apb);
      i2s_buf_free(priv, bfcontainer);
    }

  while (sq_peek(&priv->tx.act) != NULL)
    {
      flags = enter_critical_section();
      bfcontainer = (struct rp2040_buffer_s *)sq_remfirst(&priv->tx.act);
      leave_critical_section(flags);
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, OK);
      apb_free(bfcontainer->apb);
      i2s_buf_free(priv, bfcontainer);
    }

  while (sq_peek(&priv->tx.pend) != NULL)
    {
      flags = enter_critical_section();
      bfcontainer = (struct rp2040_buffer_s *)sq_remfirst(&priv->tx.pend);
      leave_critical_section(flags);
      bfcontainer->apb->flags |= AUDIO_APB_FINAL;
      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, OK);
      apb_free(bfcontainer->apb);
      i2s_buf_free(priv, bfcontainer);
    }
}

/****************************************************************************
 * Name: rp2040_i2s_ioctl
 *
 * Description:
 *   Perform a device ioctl
 *
 ****************************************************************************/

static int rp2040_i2s_ioctl(struct i2s_dev_s *dev, int cmd,
                            unsigned long arg)
{
  struct rp2040_i2s_s *priv = (struct rp2040_i2s_s *)dev;
  struct audio_buf_desc_s *bufdesc;
  int ret = -ENOTTY;

  switch (cmd)
    {
      /* AUDIOIOC_START - Start the audio stream.
       *
       *   ioctl argument:  Audio session
       */

      case AUDIOIOC_START:
        {
          irqstate_t flags;
          int mode;

          i2sinfo("AUDIOIOC_START\n");

          if (priv->channels == 1)
            {
              if (priv->datalen == 16)
                mode = RP2040_I2S_PIO_16BIT_MONO;
              else
                mode = RP2040_I2S_PIO_8BIT_MONO;
            }
          else
            {
              if (priv->datalen == 16)
                mode = RP2040_I2S_PIO_16BIT_STEREO;
              else
                mode = RP2040_I2S_PIO_8BIT_STEREO;
           }

          rp2040_i2s_pio_configure(mode, priv->samplerate);

          flags = enter_critical_section();
          ret = i2s_txdma_setup(priv);
          leave_critical_section(flags);
        }
        break;

      /* AUDIOIOC_STOP - Stop the audio stream.
       *
       *   ioctl argument:  Audio session
       */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
      case AUDIOIOC_STOP:
        {
          irqstate_t flags;

          i2sinfo("AUDIOIOC_STOP\n");

          flags = enter_critical_section();
          if (priv->tx.timeout > 0)
            {
              wd_cancel(&priv->tx.dog);
            }

          rp2040_dmastop(priv->tx.dma);
          leave_critical_section(flags);

          i2s_cleanup_queues(priv);

          ret = 0;
        }
        break;
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

      /* AUDIOIOC_PAUSE - Pause the audio stream.
       *
       *   ioctl argument:  Audio session
       */

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME

      case AUDIOIOC_PAUSE:
        {
          irqstate_t flags;

          i2sinfo("AUDIOIOC_PAUSE\n");

          flags = enter_critical_section();
          if (priv->tx.timeout > 0)
            {
              priv->tx.timeout = wd_gettime(&priv->tx.dog);
              wd_cancel(&priv->tx.dog);
            }

          rp2040_i2s_pio_enable(false);
          leave_critical_section(flags);

          ret = 0;
        }
        break;

      /* AUDIOIOC_RESUME - Resume the audio stream.
       *
       *   ioctl argument:  Audio session
       */

      case AUDIOIOC_RESUME:
        {
          irqstate_t flags;

          i2sinfo("AUDIOIOC_RESUME\n");

          flags = enter_critical_section();
          if (priv->tx.timeout > 0)
            {
              wd_start(&priv->tx.dog, priv->tx.timeout,
                       i2s_txdma_timeout, (wdparm_t)priv);
            }

          rp2040_i2s_pio_enable(true);
          leave_critical_section(flags);

          ret = 0;
        }
        break;

#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

      /* AUDIOIOC_ALLOCBUFFER - Allocate an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_ALLOCBUFFER:
        {
          i2sinfo("AUDIOIOC_ALLOCBUFFER\n");

          bufdesc = (struct audio_buf_desc_s *)arg;
          ret = apb_alloc(bufdesc);
        }
        break;

      /* AUDIOIOC_FREEBUFFER - Free an audio buffer
       *
       *   ioctl argument:  pointer to an audio_buf_desc_s structure
       */

      case AUDIOIOC_FREEBUFFER:
        {
          i2sinfo("AUDIOIOC_FREEBUFFER\n");

          bufdesc = (struct audio_buf_desc_s *)arg;
          DEBUGASSERT(bufdesc->u.buffer != NULL);
          apb_free(bufdesc->u.buffer);
          ret = sizeof(struct audio_buf_desc_s);
        }
        break;

      default:
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: i2s_dma_flags
 *
 * Description:
 *   Determine DMA FLAGS based on PID and data width
 *
 * Input Parameters:
 *   priv - Partially initialized I2S device structure.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int i2s_dma_flags(struct rp2040_i2s_s *priv)
{
  switch (priv->datalen)
    {
    case 8:
      priv->txconfig.size = RP2040_DMA_SIZE_BYTE;
      break;

    case 16:
      priv->txconfig.size = RP2040_DMA_SIZE_HALFWORD;
      break;

    default:
      i2serr("ERROR: Unsupported data width: %d\n", priv->datalen);
      return -ENOSYS;
    }

  priv->txconfig.noincr = false;
  priv->txconfig.dreq = rp2040_i2s_pio_getdreq();

  return OK;
}

/****************************************************************************
 * Name: i2s_dma_allocate
 *
 * Description:
 *   Allocate I2S DMA channels
 *
 * Input Parameters:
 *   priv - Partially initialized I2S device structure.  This function
 *          will complete the DMA specific portions of the initialization
 *
 * Returned Value:
 *   OK on success; A negated errno value on failure.
 *
 ****************************************************************************/

static int i2s_dma_allocate(struct rp2040_i2s_s *priv)
{
  /* Allocate a TX DMA channel */

  priv->tx.dma = rp2040_dmachannel();
  if (!priv->tx.dma)
    {
      i2serr("ERROR: Failed to allocate the TX DMA channel\n");
      goto errout;
    }

  /* Success exit */

  return OK;

  /* Error exit */

errout:
  i2s_dma_free(priv);
  return -ENOMEM;
}

/****************************************************************************
 * Name: i2s_dma_free
 *
 * Description:
 *   Release DMA-related resources allocated by i2s_dma_allocate()
 *
 * Input Parameters:
 *   priv - Partially initialized I2S device structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2s_dma_free(struct rp2040_i2s_s *priv)
{
  if (priv->tx.timeout > 0)
    {
      wd_cancel(&priv->tx.dog);
    }

  if (priv->tx.dma)
    {
      rp2040_dmafree(priv->tx.dma);
    }
}

/****************************************************************************
 * Name: i2s_configure
 *
 * Description:
 *   Configure I2S
 *
 * Input Parameters:
 *   priv - Partially initialized I2S device structure.  These functions
 *          will complete the I2S specific portions of the initialization
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2s_configure(struct rp2040_i2s_s *priv)
{
  /* Only configure if the port is not already configured */

  if (!priv->initialized)
    {
      rp2040_gpio_set_function(CONFIG_RP2040_I2S_DATA,
                               RP2040_GPIO_FUNC_PIO0);
      rp2040_gpio_set_function(CONFIG_RP2040_I2S_CLOCK,
                               RP2040_GPIO_FUNC_PIO0);
      rp2040_gpio_set_function(CONFIG_RP2040_I2S_CLOCK + 1,
                               RP2040_GPIO_FUNC_PIO0);

      priv->initialized = true;
    }

  /* Configure driver state specific to this I2S peripheral */

  priv->channels = 2;
  priv->samplerate = 44100;
  priv->datalen = CONFIG_RP2040_I2S_DATALEN;
#ifdef CONFIG_DEBUG
  priv->align   = RP2040_I2S_DATAMASK;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_i2sbus_initialize
 *
 * Description:
 *   Initialize the selected i2S port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2S interfaces)
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *rp2040_i2sbus_initialize(int port)
{
  struct rp2040_i2s_s *priv = NULL;
  irqstate_t flags;
  int ret;

  i2sinfo("port: %d\n", port);

  priv = (struct rp2040_i2s_s *)kmm_zalloc(sizeof(struct rp2040_i2s_s));
  if (!priv)
    {
      i2serr("ERROR: Failed to allocate a chip select structure\n");
      return NULL;
    }

  /* Set up the initial state for this chip select structure.  Other fields
   * were zeroed by kmm_zalloc().
   */

  /* Initialize the common parts for the I2S device structure */

  nxmutex_init(&priv->lock);
  priv->dev.ops = &g_i2sops;

  /* Initialize buffering */

  i2s_buf_initialize(priv);

  flags = enter_critical_section();

  i2s_configure(priv);

  /* Allocate DMA channels */

  ret = i2s_dma_allocate(priv);
  if (ret < 0)
    {
      goto errout_with_alloc;
    }

  leave_critical_section(flags);

  /* Success exit */

  return &priv->dev;

  /* Failure exits */

errout_with_alloc:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return NULL;
}

#endif /* CONFIG_RP2040_I2S */
