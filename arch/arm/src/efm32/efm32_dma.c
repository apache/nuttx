/*****************************************************************************
 * arch/arm/src/efm32/efm32_dma.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <queue.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "efm32_dma.h"

/*****************************************************************************
 * Public Types
 *****************************************************************************/
/* This union allows us to keep free DMA descriptors in in a list */

union dma_descriptor_u
{
  sq_entry_t link;
  struct dma_desriptor_s desc;
};

/* This structure describes one DMA channel */

struct dma_channel_s
{
  uint8_t chan;                  /* DMA channel number (0-EFM_DMA_NCHANNELS) */
  bool inuse;                    /* TRUE: The DMA channel is in use */
  struct dms_descriptor_s *desc; /* DMA descriptor assigned to the channel */
  dma_callback_t callback;       /* Callback invoked when the DMA completes */
  void *arg;                     /* Argument passed to callback function */
};

/* This structure describes the state of the DMA controller */

struct dma_controller_s
{
  sem_t exclsem;                 /* Protects channel table */
  sem_t chansem;                 /* Count of free channels */
  sem_t freesem;                 /* Count of free descriptors */
  sq_queue_t freedesc;           /* List of free DMA descriptors */
};

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/* This is the overall state of the DMA controller */

static struct dma_controller_s g_dmac;

/* This is the array of all DMA channels */

static struct dma_channel_s g_dmach[EFM_DMA_NCHANNELS];

/* This array describes the available channel control data structures.
 * Each structure must be aligned to the size of the DMA descriptor.
 */

static union dma_descriptor_u g_descriptors[CONFIG_EFM32_DMA_NDESCR]
  __attribute__((aligned(16)));

/*****************************************************************************
 * Public Data
 *****************************************************************************/

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/****************************************************************************
 * Name: efm32_alloc_descriptor
 *
 * Description:
 *   Allocate one DMA descriptor by removing it from the free list.
 *
 * Returned Value:
 *   The allocated DMA descriptor.  If no DMA descriptors are available, this
 *   function will wait until one is returned to the freelist.
 *
 ****************************************************************************/

static struct dma_descriptor_s *efm32_alloc_descriptor(void)
{
  struct dma_descriptor_s *desc;
  irqstate_t flags;

  /* Take a count from from the descriptor counting semaphore.  We may block
   * if there are no free descriptors.  When we get the count, then we can
   * be assured that a DMA descriptor is available in the free list and is
   * reserved for us.
   */

  flags = irqsave();
  while (sem_wait(&g_dmac.freesem) < 0)
    {
      /* sem_wait should fail only if it is awakened by a a signal */

      DEBUGASSERT(errno == EINTR);
    }

  /* Get our descriptor to the free list.  Interrupts mus be disabled here
   * because the free list may also be accessed from the DMA interrupt
   * handler.
   */

  desc = (struct dma_descriptor_s *)sq_remfirst(&g_dmac.freedesc);
  irqrestore(flags);

  /* And return the descriptor */

  DEBUGASSERT(desc && ((uintptr_t)desc & ~15) == 0);
  return desc;
}

/****************************************************************************
 * Name: efm32_free_descriptor
 *
 * Description:
 *   Free one DMA descriptor by returning it to the free list.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void efm32_free_descriptor(struct dma_descriptor_s *desc)
{
  union dma_descriptor_u *udesc = (union dma_descriptor_u *)desc;
  irqstate_t flags;

  /* Return the descriptor to the free list.  This may be called from the
   * the DMA completion interrupt handler.
   */

  flags = irqsave();
  sq_addlast(&udesc->link, &g_dmac.freedesc);
  irqrestore(flags);

  /* Notify any waiters that a new DMA descriptor is available in the free
   * list.
   */

  sem_post(&g_dmac.freesem);
}

/****************************************************************************
 * Name: efm32_dmac_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int efm32_dmac_interrupt(int irq, void *context)
{
#warning Missing logic
  return OK;
}

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/****************************************************************************
 * Name: up_dmainitialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function up_dmainitialize(void)
{
  int i;

  dmallvdbg("Initialize XDMAC0\n");

  /* Add the pre-allocated DMA descriptors to the free list */

  sq_init(&g_dmac.freedesc);
  sem_init(&g_dmac.freesem, 0, CONFIG_EFM32_DMA_NDESCR);

  for (i = 0; i < CONFIG_EFM32_DMA_NDESCR; i++)
    {
      sq_addlast(&g_descriptors[i].link, &g_dmac.freedesc);
    }

  /* Initialize the channel list  */

  sem_init(&g_dmac.exclsem, 0, 1);
  sem_init(&g_dmac.chansem, 0, EFM_DMA_NCHANNELS);

  for (i = 0; i < EFM_DMA_NCHANNELS; i++)
    {
      g_dmach[i].chan = i;
    }

  /* Enable clock to the DMA module */
#warning Missing logic

  /* Attach DMA interrupt vector */

  (void)irq_attach(EFM32_IRQ_DMA, efm32_dmac_interrupt);

  /* Initialize the controller */
#warning Missing logic

  /* Enable the IRQ at the AIC (still disabled at the DMA controller) */

  up_enable_irq(EFM32_IRQ_DMA);
}

/****************************************************************************
 * Name: efm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to a DMA channel.
 *
 *   If no DMA channel is available, then efm32_dmachannel() will wait
 *   until the holder of a channel relinquishes the channel by calling
 *   efm32_dmafree().
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

DMA_HANDLE efm32_dmachannel(void)
{
  struct dma_channel_s *dmach;
  unsigned int chndx;

  /* Take a count from from the channel counting semaphore.  We may block
   * if there are no free channels.  When we get the count, then we can
   * be assured that a channel is available in the channel list and is
   * reserved for us.
   */

  while (sem_wait(&g_dmac.chansem) < 0)
    {
      /* sem_wait should fail only if it is awakened by a a signal */

      DEBUGASSERT(errno == EINTR);
    }

  /* Get exclusive access to the DMA channel list */

  while (sem_wait(&g_dmac.exclsem) < 0)
    {
      /* sem_wait should fail only if it is awakened by a a signal */

      DEBUGASSERT(errno == EINTR);
    }

  /* Search for an available DMA channel */

  for (chndx = 0, dmach = NULL; chndx < EFM_DMA_NCHANNELS; chndx++)
    {
      struct dma_channel_s *candidate = &g_dmach[chndx];
      if (!candidate->inuse)
        {
          dmach        = candidate;
          dmach->inuse = true;

          /* Clear any pending channel interrupts */
#warning Missing logic

          /* Disable the channel */
#warning Missing logic
          break;
        }
    }

  sem_post(&g_dmac.exclsem);

  /* Show the result of the allocation */

  if (dmach)
    {
      dmavdbg("Allocated DMA channel %d\n", dmach->chan);
    }
  else
    {
      dmadbg("ERROR: Failed allocate DMA channel\n");
    }

  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: efm32_dmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA channel
 *   in a call to efm32_dmachannel, then this function will re-assign the
 *   DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
 *   in this argument must NEVER be used again until efm32_dmachannel() is
 *   called again to re-gain access to the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void efm32_dmafree(DMA_HANDLE handle)
{
  struct dma_channel_s *dmach = (struct dma_channel_s *)handle;

  DEBUGASSERT(dmach != NULL && dmach->inuse);
  dmavdbg("DMA channel %d\n", dmach->chan);

  /* Disable the channel */
#warning Missing logic

  /* Mark the channel no longer in use.  Clearing the inuse flag is an atomic
   * operation and so should be safe.
   */

  dmach->inuse = false;

  /* And increment the count of free channels... possibly waking up a
   * thread that may be waiting for a channel.
   */

  sem_post(&g_dmac.chansem);
}

/****************************************************************************
 * Name: efm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void efm32_dmasetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                    size_t ntransfers, uint32_t ccr)
{
#warning Missing logic
}

/****************************************************************************
 * Name: efm32_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void efm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half)
{
#warning Missing logic
}

/****************************************************************************
 * Name: efm32_dmastop
 *
 * Description:
 *   Cancel the DMA.  After efm32_dmastop() is called, the DMA channel is
 *   reset and efm32_dmasetup() must be called before efm32_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

void efm32_dmastop(DMA_HANDLE handle)
{
#warning Missing logic
}

/****************************************************************************
 * Name: efm32_dmaresidual
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

size_t efm32_dmaresidual(DMA_HANDLE handle)
{
#warning Missing logic
  return 0;
}

/****************************************************************************
 * Name: efm32_dmacapable
 *
 * Description:
 *   Check if the DMA controller can transfer data to/from given memory
 *   address with the given configuration. This depends on the internal
 *   connections in the ARM bus matrix of the processor. Note that this
 *   only applies to memory addresses, it will return false for any peripheral
 *   address.
 *
 * Returned value:
 *   True, if transfer is possible.
 *
 ****************************************************************************/

#ifdef CONFIG_EFM32_DMACAPABLE
bool efm32_dmacapable(uintptr_t maddr, uint32_t count, uint32_t ccr)
{
#warning Missing logic
return false;
}
#endif

/****************************************************************************
 * Name: efm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void efm32_dmasample(DMA_HANDLE handle, struct efm32_dmaregs_s *regs)
{
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: efm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by efm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void efm32_dmadump(DMA_HANDLE handle, const struct efm32_dmaregs_s *regs,
                   const char *msg)
{
#warning Missing logic
}
#endif
