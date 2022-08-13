/****************************************************************************
 * arch/arm/src/rp2040/rp2040_dmac.c
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
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "hardware/rp2040_dma.h"
#include "rp2040_dmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct dma_channel_s
{
  uint8_t        chan;          /* DMA channel number (0-RP2040_DMA_NCHANNELS) */
  bool           inuse;         /* TRUE: The DMA channel is in use */
  dma_config_t   config;        /* Current configuration */
  dma_callback_t callback;      /* Callback invoked when the DMA completes */
  void          *arg;           /* Argument passed to callback function */
};

/* This structure describes the state of the DMA controller */

struct dma_controller_s
{
  sem_t exclsem; /* Protects channel table */
  sem_t chansem; /* Count of free channels */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the overall state of the DMA controller */

static struct dma_controller_s g_dmac;

/* This is the array of all DMA channels */

static struct dma_channel_s g_dmach[RP2040_DMA_NCHANNELS];

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp2040_dmac_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int rp2040_dmac_interrupt(int irq, void *context, void *arg)
{
  struct dma_channel_s *dmach;
  int result = OK;
  unsigned int ch;
  uint32_t stat;
  uint32_t ctrl;

  /* Get and clear pending DMA interrupt status */

  stat = getreg32(RP2040_DMA_INTS0) & RP2040_DMA_INTS0_MASK;
  putreg32(stat, RP2040_DMA_INTS0);

  while (stat != 0)
    {
      ch = ffs(stat) - 1;
      stat &= ~(1 << ch);

      ctrl = getreg32(RP2040_DMA_CTRL_TRIG(ch));

      if (ctrl & RP2040_DMA_CTRL_TRIG_AHB_ERROR)
        {
          setbits_reg32(RP2040_DMA_CTRL_TRIG_READ_ERROR |
                        RP2040_DMA_CTRL_TRIG_WRITE_ERROR,
                        RP2040_DMA_CTRL_TRIG(ch));
          result = EIO;
        }

      dmach = &g_dmach[ch];

      /* Call the DMA completion callback */

      if (dmach->callback)
        {
          dmach->callback((DMA_HANDLE)dmach, result, dmach->arg);
          dmach->callback = NULL;
        }

      dmach->arg = NULL;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dma_initialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  int i;

  dmainfo("Initialize DMAC\n");

  /* Initialize the channel list  */

  nxsem_init(&g_dmac.exclsem, 0, 1);
  nxsem_init(&g_dmac.chansem, 0, RP2040_DMA_NCHANNELS);

  for (i = 0; i < RP2040_DMA_NCHANNELS; i++)
    {
      g_dmach[i].chan = i;
      putreg32(0, RP2040_DMA_CTRL_TRIG(i));
    }

  putreg32(0, RP2040_DMA_INTE0);
  putreg32(RP2040_DMA_INTS0_MASK, RP2040_DMA_INTS0);

  /* Attach DMA completion interrupt handler */

  irq_attach(RP2040_DMA_IRQ_0, rp2040_dmac_interrupt, NULL);
  up_enable_irq(RP2040_DMA_IRQ_0);
}

/****************************************************************************
 * Name: rp2040_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to a DMA channel.
 *
 *   If no DMA channel is available, then rp2040_dmachannel() will wait
 *   until the holder of a channel relinquishes the channel by calling
 *   rp2040_dmafree().
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   This function ALWAYS returns a non-NULL, void* DMA channel handle.
 *
 * Assumptions:
 *   - The caller can wait for a DMA channel to be freed if it is not
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE rp2040_dmachannel(void)
{
  struct dma_channel_s *dmach;
  unsigned int ch;
  uint32_t bit = 0;
  int ret;

  /* Take a count from the channel counting semaphore.  We may block
   * if there are no free channels.  When we get the count, then we can
   * be assured that a channel is available in the channel list and is
   * reserved for us.
   */

  ret = nxsem_wait_uninterruptible(&g_dmac.chansem);
  if (ret < 0)
    {
      return NULL;
    }

  /* Get exclusive access to the DMA channel list */

  ret = nxsem_wait_uninterruptible(&g_dmac.exclsem);
  if (ret < 0)
    {
      nxsem_post(&g_dmac.chansem);
      return NULL;
    }

  /* Search for an available DMA channel */

  for (ch = 0, dmach = NULL; ch < RP2040_DMA_NCHANNELS; ch++)
    {
      struct dma_channel_s *candidate = &g_dmach[ch];
      if (!candidate->inuse)
        {
          dmach        = candidate;
          dmach->inuse = true;

          bit = 1 << ch;
          break;
        }
    }

  nxsem_post(&g_dmac.exclsem);

  setbits_reg32(bit, RP2040_DMA_INTS0);
  setbits_reg32(bit, RP2040_DMA_INTE0);

  /* Since we have reserved a DMA descriptor by taking a count from chansem,
   * it would be a serious logic failure if we could not find a free channel
   * for our use.
   */

  DEBUGASSERT(dmach);
  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: rp2040_dmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA
 *   channel in a call to rp2040_dmachannel, then this function will
 *   re-assign the DMA channel to that thread and wake it up.  NOTE:  The
 *   'handle' used in this argument must NEVER be used again until
 *   rp2040_dmachannel() is called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void rp2040_dmafree(DMA_HANDLE handle)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  unsigned int ch;

  DEBUGASSERT(dmach != NULL && dmach->inuse);
  ch  = dmach->chan;
  dmainfo("DMA channel %d\n", ch);

  /* Disable the channel */

  setbits_reg32(1 << dmach->chan, RP2040_DMA_CHAN_ABORT);
  putreg32(0, RP2040_DMA_CTRL_TRIG(ch));
  clrbits_reg32(1 << dmach->chan, RP2040_DMA_INTE0);
  clrbits_reg32(1 << dmach->chan, RP2040_DMA_INTS0);

  /* Mark the channel no longer in use.  Clearing the in-use flag is an
   * atomic operation and so should be safe.
   */

  dmach->inuse = false;

  /* And increment the count of free channels... possibly waking up a
   * thread that may be waiting for a channel.
   */

  nxsem_post(&g_dmac.chansem);
}

/****************************************************************************
 * Name: rp2040_rxdmasetup
 *
 * Description:
 *   Configure an RX (peripheral-to-memory) DMA before starting the transfer.
 *
 * Input Parameters:
 *   paddr  - Peripheral address (source)
 *   maddr  - Memory address (destination)
 *   nbytes - Number of bytes to transfer.  Must be an even multiple of the
 *            configured transfer size.
 *   config - Channel configuration selections
 *
 ****************************************************************************/

void rp2040_rxdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                       size_t nbytes, dma_config_t config)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  unsigned int ch;
  uint32_t count;
  uint32_t mask;
  uint32_t ctrl;

  DEBUGASSERT(dmach != NULL && dmach->inuse);
  ch  = dmach->chan;

  /* Save the configuration (for rp2040_dmastart()). */

  dmach->config = config;

  DEBUGASSERT(config.size >= RP2040_DMA_SIZE_BYTE &&
              config.size <= RP2040_DMA_SIZE_WORD);

  mask = (1 << config.size) - 1;
  count = nbytes >> config.size;

  DEBUGASSERT(count > 0);

  /* Set DMA registers */

  putreg32(paddr & ~mask, RP2040_DMA_READ_ADDR(ch));
  putreg32(maddr & ~mask, RP2040_DMA_WRITE_ADDR(ch));
  putreg32(count, RP2040_DMA_TRANS_COUNT(ch));

  ctrl = RP2040_DMA_CTRL_TRIG_READ_ERROR |
         RP2040_DMA_CTRL_TRIG_WRITE_ERROR |
         ((config.dreq << RP2040_DMA_CTRL_TRIG_TREQ_SEL_SHIFT) &
          RP2040_DMA_CTRL_TRIG_TREQ_SEL_MASK) |
         ((ch << RP2040_DMA_CTRL_TRIG_CHAIN_TO_SHIFT) &
          RP2040_DMA_CTRL_TRIG_CHAIN_TO_MASK) |
         (config.size << RP2040_DMA_CTRL_TRIG_DATA_SIZE_SHIFT);

  if (!config.noincr)
    {
      ctrl |= RP2040_DMA_CTRL_TRIG_INCR_WRITE;
    }

  putreg32(ctrl, RP2040_DMA_CTRL_TRIG(ch));
}

/****************************************************************************
 * Name: rp2040_txdmasetup
 *
 * Description:
 *   Configure an TX (memory-to-peripheral) DMA before starting the transfer.
 *
 * Input Parameters:
 *   paddr  - Peripheral address (destination)
 *   maddr  - Memory address (source)
 *   nbytes - Number of bytes to transfer.  Must be an even multiple of the
 *            configured transfer size.
 *   config - Channel configuration selections
 *
 ****************************************************************************/

void rp2040_txdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr,
                       size_t nbytes, dma_config_t config)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  unsigned int ch;
  uint32_t count;
  uint32_t mask;
  uint32_t ctrl;

  DEBUGASSERT(dmach != NULL && dmach->inuse);
  ch  = dmach->chan;

  /* Save the configuration (for rp2040_dmastart()). */

  dmach->config = config;

  DEBUGASSERT(config.size >= RP2040_DMA_SIZE_BYTE &&
              config.size <= RP2040_DMA_SIZE_WORD);

  mask = (1 << config.size) - 1;
  count = nbytes >> config.size;

  DEBUGASSERT(count > 0);

  /* Set DMA registers */

  putreg32(maddr & ~mask, RP2040_DMA_READ_ADDR(ch));
  putreg32(paddr & ~mask, RP2040_DMA_WRITE_ADDR(ch));
  putreg32(count, RP2040_DMA_TRANS_COUNT(ch));

  ctrl = RP2040_DMA_CTRL_TRIG_READ_ERROR |
         RP2040_DMA_CTRL_TRIG_WRITE_ERROR |
         ((config.dreq << RP2040_DMA_CTRL_TRIG_TREQ_SEL_SHIFT) &
          RP2040_DMA_CTRL_TRIG_TREQ_SEL_MASK) |
         ((ch << RP2040_DMA_CTRL_TRIG_CHAIN_TO_SHIFT) &
          RP2040_DMA_CTRL_TRIG_CHAIN_TO_MASK) |
         (config.size << RP2040_DMA_CTRL_TRIG_DATA_SIZE_SHIFT);

  if (!config.noincr)
    {
      ctrl |= RP2040_DMA_CTRL_TRIG_INCR_READ;
    }

  putreg32(ctrl, RP2040_DMA_CTRL_TRIG(ch));
}

/****************************************************************************
 * Name: rp2040_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by rp2040_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void rp2040_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  uint32_t ch;

  DEBUGASSERT(dmach && dmach->inuse);
  ch = dmach->chan;

  /* Save the DMA complete callback info */

  dmach->callback = callback;
  dmach->arg      = arg;

  /* Enable the channel */

  setbits_reg32(RP2040_DMA_CTRL_TRIG_EN, RP2040_DMA_CTRL_TRIG(ch));
}

/****************************************************************************
 * Name: rp2040_dmastop
 *
 * Description:
 *   Cancel the DMA.  After rp2040_dmastop() is called, the DMA channel is
 *   reset and rp2040_dmasetup() must be called before rp2040_dmastart()
 *   can be called again
 *
 * Assumptions:
 *   - DMA handle allocated by rp2040_dmachannel()
 *
 ****************************************************************************/

void rp2040_dmastop(DMA_HANDLE handle)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;
  uint32_t bit;
  uint32_t stat;

  DEBUGASSERT(dmach);
  bit = 1 << dmach->chan;

  /* Disable the channel */

  setbits_reg32(bit, RP2040_DMA_CHAN_ABORT);

  do
    {
      stat = getreg32(RP2040_DMA_CHAN_ABORT);
    }
  while (stat & bit);
}
