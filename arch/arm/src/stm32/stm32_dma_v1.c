/****************************************************************************
 * arch/arm/src/stm32/stm32_dma_v1.c
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

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "sched/sched.h"
#include "chip.h"
#include "stm32_dma.h"
#include "stm32.h"

/* This file supports the STM32 DMA IP core version 1 - F0, F1, F3, G4, L0,
 * L1, L4.
 *
 * F0, L0 and L4 have the additional CSELR register which is used to remap
 * the DMA requests for each channel.
 *
 * G4 has additional channels in DMA1 and DMA2.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_STM32_HAVE_DMA1_CHAN8)
#  define DMA1_NCHANNELS     8
#else
#  define DMA1_NCHANNELS     7
#endif

#if STM32_NDMA > 1
#  if defined(CONFIG_STM32_HAVE_DMA2_CHAN678)
#    define DMA2_NCHANNELS   8
#  else
#    define DMA2_NCHANNELS   5
#  endif
#  define DMA_NCHANNELS      (DMA1_NCHANNELS + DMA2_NCHANNELS)
#else
#  define DMA_NCHANNELS      DMA1_NCHANNELS
#endif

/* Convert the DMA channel base address to the DMA register block address */

#define DMA_BASE(ch)     (ch & 0xfffffc00)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct stm32_dma_s
{
  uint8_t        chan;     /* DMA channel number (0-6) */
#ifdef DMA_HAVE_CSELR
  uint8_t        function; /* DMA peripheral connected to this channel (0-7) */
#endif
  uint8_t        irq;      /* DMA channel IRQ number */
  sem_t          sem;      /* Used to wait for DMA channel to become available */
  uint32_t       base;     /* DMA register channel base address */
  dma_callback_t callback; /* Callback invoked when the DMA completes */
  void          *arg;      /* Argument passed to callback function */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array describes the state of each DMA */

static struct stm32_dma_s g_dma[DMA_NCHANNELS] =
{
#if DMA1_NCHANNELS > 0
  {
    .chan     = 0,
    .irq      = STM32_IRQ_DMA1CH1,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMACHAN_OFFSET(0),
  },
#endif /* DMA1_NCHANNELS > 0 */
#if DMA1_NCHANNELS > 1
  {
    .chan     = 1,
    .irq      = STM32_IRQ_DMA1CH2,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMACHAN_OFFSET(1),
  },
#endif /* DMA1_NCHANNELS > 1 */
#if DMA1_NCHANNELS > 2
  {
    .chan     = 2,
    .irq      = STM32_IRQ_DMA1CH3,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMACHAN_OFFSET(2),
  },
#endif /* DMA1_NCHANNELS > 2 */
#if DMA1_NCHANNELS > 3
  {
    .chan     = 3,
    .irq      = STM32_IRQ_DMA1CH4,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMACHAN_OFFSET(3),
  },
#endif /* DMA1_NCHANNELS > 3 */
#if DMA1_NCHANNELS > 4
  {
    .chan     = 4,
    .irq      = STM32_IRQ_DMA1CH5,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMACHAN_OFFSET(4),
  },
#endif /* DMA1_NCHANNELS > 4 */
#if DMA1_NCHANNELS > 5
  {
    .chan     = 5,
    .irq      = STM32_IRQ_DMA1CH6,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMACHAN_OFFSET(5),
  },
#endif /* DMA1_NCHANNELS > 5 */
#if DMA1_NCHANNELS > 6
  {
    .chan     = 6,
    .irq      = STM32_IRQ_DMA1CH7,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMACHAN_OFFSET(6),
  },
#endif /* DMA1_NCHANNELS > 6 */
#if DMA1_NCHANNELS > 7
  {
    .chan     = 7,
    .irq      = STM32_IRQ_DMA1CH8,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMACHAN_OFFSET(7),
  },
#endif /* DMA1_NCHANNELS > 7 */
#if STM32_NDMA > 1
#if DMA2_NCHANNELS > 0
  {
    .chan     = 0,
    .irq      = STM32_IRQ_DMA2CH1,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMACHAN_OFFSET(0),
  },
#endif /* DMA2_NCHANNELS > 0 */
#if DMA2_NCHANNELS > 1
  {
    .chan     = 1,
    .irq      = STM32_IRQ_DMA2CH2,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMACHAN_OFFSET(1),
  },
#endif /* DMA2_NCHANNELS > 1 */
#if DMA2_NCHANNELS > 2
  {
    .chan     = 2,
    .irq      = STM32_IRQ_DMA2CH3,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMACHAN_OFFSET(2),
  },
#endif /* DMA2_NCHANNELS > 2 */
#if DMA2_NCHANNELS > 3
  {
    .chan     = 3,
#if defined(CONFIG_STM32_CONNECTIVITYLINE) || \
    defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX) || \
    defined(CONFIG_STM32_STM32G4XXX) || defined(CONFIG_STM32_STM32L15XX)
    .irq      = STM32_IRQ_DMA2CH4,
#else
    .irq      = STM32_IRQ_DMA2CH45,
#endif
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMACHAN_OFFSET(3),
  },
#endif /* DMA2_NCHANNELS > 3 */
#if DMA2_NCHANNELS > 4
  {
    .chan     = 4,
#if defined(CONFIG_STM32_CONNECTIVITYLINE) || \
    defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX) || \
    defined(CONFIG_STM32_STM32G4XXX) || defined(CONFIG_STM32_STM32L15XX)
    .irq      = STM32_IRQ_DMA2CH5,
#else
    .irq      = STM32_IRQ_DMA2CH45,
#endif
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMACHAN_OFFSET(4),
  },
#endif /* DMA2_NCHANNELS > 4 */
#if DMA2_NCHANNELS > 5
  {
    .chan     = 5,
    .irq      = STM32_IRQ_DMA2CH5,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMACHAN_OFFSET(5),
  },
#endif /* DMA2_NCHANNELS > 5 */
#if DMA2_NCHANNELS > 6
  {
    .chan     = 6,
    .irq      = STM32_IRQ_DMA2CH6,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMACHAN_OFFSET(6),
  },
#endif /* DMA2_NCHANNELS > 6 */
#if DMA2_NCHANNELS > 7
  {
    .chan     = 7,
    .irq      = STM32_IRQ_DMA2CH7,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMACHAN_OFFSET(7),
  },
#endif /* DMA2_NCHANNELS > 7 */
#endif /* STM32_NDMA > 1 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * DMA register access functions
 ****************************************************************************/

/* Get non-channel register from DMA1 or DMA2 */

static inline uint32_t dmabase_getreg(struct stm32_dma_s *dmach,
                                      uint32_t offset)
{
  return getreg32(DMA_BASE(dmach->base) + offset);
}

/* Write to non-channel register in DMA1 or DMA2 */

static inline void dmabase_putreg(struct stm32_dma_s *dmach,
                                  uint32_t offset, uint32_t value)
{
  putreg32(value, DMA_BASE(dmach->base) + offset);
}

/* Get channel register from DMA1 or DMA2 */

static inline uint32_t dmachan_getreg(struct stm32_dma_s *dmach,
                                      uint32_t offset)
{
  return getreg32(dmach->base + offset);
}

/* Write to channel register in DMA1 or DMA2 */

static inline void dmachan_putreg(struct stm32_dma_s *dmach,
                                  uint32_t offset, uint32_t value)
{
  putreg32(value, dmach->base + offset);
}

/****************************************************************************
 * Name: stm32_dmachandisable
 *
 * Description:
 *  Disable the DMA channel
 *
 ****************************************************************************/

static void stm32_dmachandisable(struct stm32_dma_s *dmach)
{
  uint32_t regval;

  /* Disable all interrupts at the DMA controller */

  regval = dmachan_getreg(dmach, STM32_DMACHAN_CCR_OFFSET);
  regval &= ~DMA_CCR_ALLINTS;

  /* Disable the DMA channel */

  regval &= ~DMA_CCR_EN;
  dmachan_putreg(dmach, STM32_DMACHAN_CCR_OFFSET, regval);

  /* Clear pending channel interrupts */

  dmabase_putreg(dmach, STM32_DMA_IFCR_OFFSET,
                 DMA_ISR_CHAN_MASK(dmach->chan));
}

/****************************************************************************
 * Name: irq_to_channel_index
 *
 * Description:
 *   Given an IRQ number, find the channel index in the g_dma array.
 *
 * Parameters:
 *   irq: IRQ number as passed to stm32_dmainterrupt.
 *
 * Returned Value:
 *   On success (IRQ matches a DMA channel), returns index in the g_dma
 *   array from 0 to DMA_NCHANNELS - 1.  On failure (IRQ does not match
 *   a DMA channel), returns -1.
 *
 ****************************************************************************/

static int irq_to_channel_index(int irq)
{
  int chndx;

  /* Find the DMA channel that matches this IRQ */

  for (chndx = 0; chndx < DMA_NCHANNELS; chndx++)
    {
      if (irq == g_dma[chndx].irq)
        {
          return chndx;
        }
    }

  /* Failed to find the DMA channel for this IRQ */

  return -1;
}

/****************************************************************************
 * Name: stm32_dmainterrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int stm32_dmainterrupt(int irq, void *context, void *arg)
{
  struct stm32_dma_s *dmach;
  uint32_t isr;
  int chndx = 0;

  /* Get the channel structure from the interrupt number */

  chndx = irq_to_channel_index(irq);
  if (chndx < 0)
    {
      DEBUGPANIC();
    }

  dmach = &g_dma[chndx];

  /* Get the interrupt status (for this channel only) */

  isr = dmabase_getreg(dmach, STM32_DMA_ISR_OFFSET) &
        DMA_ISR_CHAN_MASK(dmach->chan);

  /* Clear the interrupts we are handling */

  dmabase_putreg(dmach, STM32_DMA_IFCR_OFFSET, isr);

  /* Invoke the callback */

  if (dmach->callback)
    {
      dmach->callback(dmach, isr >> DMA_ISR_CHAN_SHIFT(dmach->chan),
                      dmach->arg);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dmainitialize
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
  struct stm32_dma_s *dmach;
  int chndx;

  /* Initialize each DMA channel */

  for (chndx = 0; chndx < DMA_NCHANNELS; chndx++)
    {
      dmach = &g_dma[chndx];

      /* Attach DMA interrupt vectors */

      irq_attach(dmach->irq, stm32_dmainterrupt, NULL);

      /* Disable the DMA channel */

      stm32_dmachandisable(dmach);

      /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

      up_enable_irq(dmach->irq);
    }
}

/****************************************************************************
 * Name: stm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'chndx' argument.
 *   DMA channels are shared on the STM32:  Devices sharing the same DMA
 *   channel cannot do DMA concurrently!  See the DMACHAN_* definitions in
 *   stm32_dma.h.
 *
 *   If the DMA channel is not available, then stm32_dmachannel() will wait
 *   until the holder of the channel relinquishes the channel by calling
 *   stm32_dmafree().  WARNING: If you have two devices sharing a DMA
 *   channel and the code never releases the channel, the stm32_dmachannel
 *   call for the other will hang forever in this function!  Don't let your
 *   design do that!
 *
 *   Hmm.. I suppose this interface could be extended to make a non-blocking
 *   version.  Feel free to do that if that is what you need.
 *
 * Input Parameters:
 *   chndx - Identifies the stream/channel resource. For the STM32 F1, this
 *     is simply the channel number as provided by the DMACHAN_* definitions
 *     in chip/stm32f10xxx_dma.h.
 *
 * Returned Value:
 *   Provided that 'chndx' is valid, this function ALWAYS returns a non-NULL,
 *   void* DMA channel handle.  (If 'chndx' is invalid, the function will
 *   assert if debug is enabled or do something ignorant otherwise).
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE stm32_dmachannel(unsigned int chndef)
{
  int chndx = 0;
  struct stm32_dma_s *dmach = NULL;
  int ret;

#ifdef DMA_HAVE_CSELR
  chndx = (chndef & DMACHAN_SETTING_CHANNEL_MASK) >>
          DMACHAN_SETTING_CHANNEL_SHIFT;
#else
  chndx = chndef;
#endif

  dmach = &g_dma[chndx];

  DEBUGASSERT(chndx < DMA_NCHANNELS);

  /* Get exclusive access to the DMA channel -- OR wait until the channel
   * is available if it is currently being used by another driver
   */

  ret = nxsem_wait_uninterruptible(&dmach->sem);
  if (ret < 0)
    {
      return NULL;
    }

  /* The caller now has exclusive use of the DMA channel */

#ifdef DMA_HAVE_CSELR
  /* Define the peripheral that will use the channel. This is stored until
   * dmasetup is called.
   */

  dmach->function = (chndef & DMACHAN_SETTING_FUNCTION_MASK) >>
                     DMACHAN_SETTING_FUNCTION_SHIFT;
#endif

  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel. If another thread is waiting for this DMA channel
 *   in a call to stm32_dmachannel, then this function will re-assign the
 *   DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
 *   in this argument must NEVER be used again until stm32_dmachannel() is
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

void stm32_dmafree(DMA_HANDLE handle)
{
  struct stm32_dma_s *dmach = (struct stm32_dma_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Release the channel */

  nxsem_post(&dmach->sem);
}

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void stm32_dmasetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                    size_t ntransfers, uint32_t ccr)
{
  struct stm32_dma_s *dmach = (struct stm32_dma_s *)handle;
  uint32_t regval;

  /* Then DMA_CNDTRx register can only be modified if the DMA channel is
   * disabled.
   */

  regval  = dmachan_getreg(dmach, STM32_DMACHAN_CCR_OFFSET);
  regval &= ~(DMA_CCR_EN);
  dmachan_putreg(dmach, STM32_DMACHAN_CCR_OFFSET, regval);

  /* Set the peripheral register address in the DMA_CPARx register. The data
   * will be moved from/to this address to/from the memory after the
   * peripheral event.
   */

  dmachan_putreg(dmach, STM32_DMACHAN_CPAR_OFFSET, paddr);

  /* Set the memory address in the DMA_CMARx register. The data will be
   * written to or read from this memory after the peripheral event.
   */

  dmachan_putreg(dmach, STM32_DMACHAN_CMAR_OFFSET, maddr);

  /* Configure the total number of data to be transferred in the DMA_CNDTRx
   * register.  After each peripheral event, this value will be decremented.
   */

  dmachan_putreg(dmach, STM32_DMACHAN_CNDTR_OFFSET, ntransfers);

  /* Configure the channel priority using the PL[1:0] bits in the DMA_CCRx
   * register.  Configure data transfer direction, circular mode,
   * peripheral & memory incremented mode, peripheral & memory data size,
   * and interrupt after half and/or full transfer in the DMA_CCRx register.
   */

  regval  = dmachan_getreg(dmach, STM32_DMACHAN_CCR_OFFSET);
  regval &= ~(DMA_CCR_MEM2MEM | DMA_CCR_PL_MASK | DMA_CCR_MSIZE_MASK |
              DMA_CCR_PSIZE_MASK | DMA_CCR_MINC | DMA_CCR_PINC |
              DMA_CCR_CIRC | DMA_CCR_DIR);
  ccr    &=  (DMA_CCR_MEM2MEM | DMA_CCR_PL_MASK | DMA_CCR_MSIZE_MASK |
              DMA_CCR_PSIZE_MASK | DMA_CCR_MINC | DMA_CCR_PINC |
              DMA_CCR_CIRC | DMA_CCR_DIR);
  regval |= ccr;
  dmachan_putreg(dmach, STM32_DMACHAN_CCR_OFFSET, regval);

#ifdef DMA_HAVE_CSELR
  /* Define peripheral indicated in dmach->function */

  regval  = dmabase_getreg(dmach, STM32_DMA_CSELR_OFFSET);
  regval &= ~(0x0f << (dmach->chan << 2));
  regval |= (dmach->function << (dmach->chan << 2));
  dmabase_putreg(dmach, STM32_DMA_CSELR_OFFSET, regval);
#endif
}

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback,
                    void *arg, bool half)
{
  struct stm32_dma_s *dmach = (struct stm32_dma_s *)handle;
  uint32_t ccr;

  DEBUGASSERT(handle != NULL);

  /* Save the callback info.  This will be invoked whent the DMA completes */

  dmach->callback = callback;
  dmach->arg      = arg;

  /* Activate the channel by setting the ENABLE bit in the DMA_CCRx register.
   * As soon as the channel is enabled, it can serve any DMA request from the
   * peripheral connected on the channel.
   */

  ccr  = dmachan_getreg(dmach, STM32_DMACHAN_CCR_OFFSET);
  ccr |= DMA_CCR_EN;

  /* In normal mode, interrupt at either half or full completion. In circular
   * mode, always interrupt on buffer wrap, and optionally interrupt at the
   * halfway point.
   */

  if ((ccr & DMA_CCR_CIRC) == 0)
    {
      /* Once half of the bytes are transferred, the half-transfer flag
       * (HTIF) is set and an interrupt is generated if the Half-Transfer
       * Interrupt Enable bit (HTIE) is set. At the end of the transfer, the
       * Transfer Complete Flag (TCIF) is set and an interrupt is generated
       * if the Transfer Complete Interrupt Enable bit (TCIE) is set.
       */

      ccr |= (half ?
              (DMA_CCR_HTIE | DMA_CCR_TEIE) : (DMA_CCR_TCIE | DMA_CCR_TEIE));
    }
  else
    {
      /* In nonstop mode, when the transfer completes it immediately resets
       * and starts again.  The transfer-complete interrupt is thus always
       * enabled, and the half-complete interrupt can be used in circular
       * mode to determine when the buffer is half-full or in double-buffered
       * mode to determine when one of the two buffers is full.
       */

      ccr |= (half ? DMA_CCR_HTIE : 0) | DMA_CCR_TCIE | DMA_CCR_TEIE;
    }

  dmachan_putreg(dmach, STM32_DMACHAN_CCR_OFFSET, ccr);
}

/****************************************************************************
 * Name: stm32_dmastop
 *
 * Description:
 *   Cancel the DMA.  After stm32_dmastop() is called, the DMA channel is
 *   reset and stm32_dmasetup() must be called before stm32_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

void stm32_dmastop(DMA_HANDLE handle)
{
  struct stm32_dma_s *dmach = (struct stm32_dma_s *)handle;
  stm32_dmachandisable(dmach);
}

/****************************************************************************
 * Name: stm32_dmaresidual
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

size_t stm32_dmaresidual(DMA_HANDLE handle)
{
  struct stm32_dma_s *dmach = (struct stm32_dma_s *)handle;

  return dmachan_getreg(dmach, STM32_DMACHAN_CNDTR_OFFSET);
}

/****************************************************************************
 * Name: stm32_dmacapable
 *
 * Description:
 *   Check if the DMA controller can transfer data to/from given memory
 *   address. This depends on the internal connections in the ARM bus matrix
 *   of the processor. Note that this only applies to memory addresses, it
 *   will return false for any peripheral address.
 *
 * Returned Value:
 *   True, if transfer is possible.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_DMACAPABLE
bool stm32_dmacapable(uintptr_t maddr, uint32_t count, uint32_t ccr)
{
  uint32_t transfer_size;
  uint32_t mend;

  /* Verify that the address conforms to the memory transfer size.
   * Transfers to/from memory performed by the DMA controller are
   * required to be aligned to their size.
   *
   * See ST RM0090 rev4, section 9.3.11
   *
   * Compute mend inline to avoid a possible non-constant integer
   * multiply.
   */

  switch (ccr & DMA_CCR_MSIZE_MASK)
    {
      case DMA_CCR_MSIZE_8BITS:
        transfer_size = 1;
        mend = maddr + count - 1;
        break;

      case DMA_CCR_MSIZE_16BITS:
        transfer_size = 2;
        mend = maddr + (count << 1) - 1;
        break;

      case DMA_CCR_MSIZE_32BITS:
        transfer_size = 4;
        mend = maddr + (count << 2) - 1;
        break;

      default:
        return false;
    }

  if ((maddr & (transfer_size - 1)) != 0)
    {
      return false;
    }

  /* Verify that the transfer is to a memory region that supports DMA. */

  if ((maddr & STM32_REGION_MASK) != (mend & STM32_REGION_MASK))
    {
      return false;
    }

  switch (maddr & STM32_REGION_MASK)
    {
#if defined(CONFIG_STM32_STM32F10XX)
      case STM32_FSMC_BANK1:
      case STM32_FSMC_BANK2:
      case STM32_FSMC_BANK3:
      case STM32_FSMC_BANK4:
#endif
      case STM32_SRAM_BASE:
      case STM32_CODE_BASE:

        /* All RAM and flash is supported */

        return true;

      default:

        /* Everything else is unsupported by DMA */

        return false;
    }
}
#endif

/****************************************************************************
 * Name: stm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32_dmasample(DMA_HANDLE handle, struct stm32_dmaregs_s *regs)
{
  struct stm32_dma_s *dmach = (struct stm32_dma_s *)handle;
  irqstate_t flags;

  flags       = enter_critical_section();
  regs->isr   = dmabase_getreg(dmach, STM32_DMA_ISR_OFFSET);
#ifdef DMA_HAVE_CSELR
  regs->cselr = dmabase_getreg(dmach, STM32_DMA_CSELR_OFFSET);
#endif
  regs->ccr   = dmachan_getreg(dmach, STM32_DMACHAN_CCR_OFFSET);
  regs->cndtr = dmachan_getreg(dmach, STM32_DMACHAN_CNDTR_OFFSET);
  regs->cpar  = dmachan_getreg(dmach, STM32_DMACHAN_CPAR_OFFSET);
  regs->cmar  = dmachan_getreg(dmach, STM32_DMACHAN_CMAR_OFFSET);
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: stm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32_dmadump(DMA_HANDLE handle, const struct stm32_dmaregs_s *regs,
                   const char *msg)
{
  struct stm32_dma_s *dmach = (struct stm32_dma_s *)handle;
  uint32_t dmabase = DMA_BASE(dmach->base);

  dmainfo("DMA Registers: %s\n", msg);
  dmainfo("   ISRC[%08x]: %08x\n",
          dmabase + STM32_DMA_ISR_OFFSET, regs->isr);
#ifdef DMA_HAVE_CSELR
  dmainfo("  CSELR[%08x]: %08x\n",
          dmabase + STM32_DMA_CSELR_OFFSET, regs->cselr);
#endif
  dmainfo("    CCR[%08x]: %08x\n",
          dmach->base + STM32_DMACHAN_CCR_OFFSET, regs->ccr);
  dmainfo("  CNDTR[%08x]: %08x\n",
          dmach->base + STM32_DMACHAN_CNDTR_OFFSET, regs->cndtr);
  dmainfo("   CPAR[%08x]: %08x\n",
          dmach->base + STM32_DMACHAN_CPAR_OFFSET, regs->cpar);
  dmainfo("   CMAR[%08x]: %08x\n",
          dmach->base + STM32_DMACHAN_CMAR_OFFSET, regs->cmar);
}
#endif

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT

/****************************************************************************
 * Name: stm32_dma_intack
 *
 * Description:
 *   Public visible interface to acknowledge interrupts on DMA channel
 *
 ****************************************************************************/

void stm32_dma_intack(unsigned int chndx, uint32_t isr)
{
  struct stm32_dma_s *dmach = &g_dma[chndx];

  dmabase_putreg(dmach, STM32_DMA_IFCR_OFFSET, isr);
}

/****************************************************************************
 * Name: stm32_dma_intget
 *
 * Description:
 *   Public visible interface to get pending interrupts from DMA channel
 *
 ****************************************************************************/

uint32_t stm32_dma_intget(unsigned int chndx)
{
  struct stm32_dma_s *dmach = &g_dma[chndx];

  return dmabase_getreg(dmach, STM32_DMA_ISR_OFFSET) &
         DMA_ISR_CHAN_MASK(dmach->chan);
}
#endif /* CONFIG_ARCH_HIPRI_INTERRUPT */
