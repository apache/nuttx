/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4x6xx_dma.c
 *
 *   Copyright (C) 2009, 2011-2013, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Sebastien Lorquet <sebastien@lorquet.fr>
 *           dev@ziggurat29.com
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include <nuttx/semaphore.h>

#include "arm_arch.h"
#include "arm_internal.h"
#include "sched/sched.h"
#include "chip.h"
#include "stm32l4_dma.h"
#include "stm32l4.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMA1_NCHANNELS   7
#if STM32L4_NDMA > 1
#  define DMA2_NCHANNELS 7
#  define DMA_NCHANNELS  (DMA1_NCHANNELS+DMA2_NCHANNELS)
#else
#  define DMA_NCHANNELS  DMA1_NCHANNELS
#endif

/* Convert the DMA channel base address to the DMA register block address */

#define DMA_BASE(ch)     (ch & 0xfffffc00)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct stm32l4_dma_s
{
  uint8_t        chan;     /* DMA channel number (0-6) */
  uint8_t        function; /* DMA peripheral connected to this channel (0-7) */
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

static struct stm32l4_dma_s g_dma[DMA_NCHANNELS] =
{
  {
    .chan     = 0,
    .irq      = STM32L4_IRQ_DMA1CH1,
    .base     = STM32L4_DMA1_BASE + STM32L4_DMACHAN_OFFSET(0),
  },
  {
    .chan     = 1,
    .irq      = STM32L4_IRQ_DMA1CH2,
    .base     = STM32L4_DMA1_BASE + STM32L4_DMACHAN_OFFSET(1),
  },
  {
    .chan     = 2,
    .irq      = STM32L4_IRQ_DMA1CH3,
    .base     = STM32L4_DMA1_BASE + STM32L4_DMACHAN_OFFSET(2),
  },
  {
    .chan     = 3,
    .irq      = STM32L4_IRQ_DMA1CH4,
    .base     = STM32L4_DMA1_BASE + STM32L4_DMACHAN_OFFSET(3),
  },
  {
    .chan     = 4,
    .irq      = STM32L4_IRQ_DMA1CH5,
    .base     = STM32L4_DMA1_BASE + STM32L4_DMACHAN_OFFSET(4),
  },
  {
    .chan     = 5,
    .irq      = STM32L4_IRQ_DMA1CH6,
    .base     = STM32L4_DMA1_BASE + STM32L4_DMACHAN_OFFSET(5),
  },
  {
    .chan     = 6,
    .irq      = STM32L4_IRQ_DMA1CH7,
    .base     = STM32L4_DMA1_BASE + STM32L4_DMACHAN_OFFSET(6),
  },
#if STM32L4_NDMA > 1
  {
    .chan     = 0,
    .irq      = STM32L4_IRQ_DMA2CH1,
    .base     = STM32L4_DMA2_BASE + STM32L4_DMACHAN_OFFSET(0),
  },
  {
    .chan     = 1,
    .irq      = STM32L4_IRQ_DMA2CH2,
    .base     = STM32L4_DMA2_BASE + STM32L4_DMACHAN_OFFSET(1),
  },
  {
    .chan     = 2,
    .irq      = STM32L4_IRQ_DMA2CH3,
    .base     = STM32L4_DMA2_BASE + STM32L4_DMACHAN_OFFSET(2),
  },
  {
    .chan     = 3,
    .irq      = STM32L4_IRQ_DMA2CH4,
    .base     = STM32L4_DMA2_BASE + STM32L4_DMACHAN_OFFSET(3),
  },
  {
    .chan     = 4,
    .irq      = STM32L4_IRQ_DMA2CH5,
    .base     = STM32L4_DMA2_BASE + STM32L4_DMACHAN_OFFSET(4),
  },
  {
    .chan     = 5,
    .irq      = STM32L4_IRQ_DMA2CH6,
    .base     = STM32L4_DMA2_BASE + STM32L4_DMACHAN_OFFSET(5),
  },
  {
    .chan     = 6,
    .irq      = STM32L4_IRQ_DMA2CH7,
    .base     = STM32L4_DMA2_BASE + STM32L4_DMACHAN_OFFSET(6),
  },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * DMA register access functions
 ****************************************************************************/

/* Get non-channel register from DMA1 or DMA2 */

static inline uint32_t dmabase_getreg(struct stm32l4_dma_s *dmach,
                                      uint32_t offset)
{
  return getreg32(DMA_BASE(dmach->base) + offset);
}

/* Write to non-channel register in DMA1 or DMA2 */

static inline void dmabase_putreg(struct stm32l4_dma_s *dmach,
                                  uint32_t offset, uint32_t value)
{
  putreg32(value, DMA_BASE(dmach->base) + offset);
}

/* Get channel register from DMA1 or DMA2 */

static inline uint32_t dmachan_getreg(struct stm32l4_dma_s *dmach,
                                      uint32_t offset)
{
  return getreg32(dmach->base + offset);
}

/* Write to channel register in DMA1 or DMA2 */

static inline void dmachan_putreg(struct stm32l4_dma_s *dmach,
                                  uint32_t offset, uint32_t value)
{
  putreg32(value, dmach->base + offset);
}

/****************************************************************************
 * Name: stm32l4_dmatake() and stm32l4_dmagive()
 *
 * Description:
 *   Used to get exclusive access to a DMA channel.
 *
 ****************************************************************************/

static int stm32l4_dmatake(FAR struct stm32l4_dma_s *dmach)
{
  return nxsem_wait_uninterruptible(&dmach->sem);
}

static inline void stm32l4_dmagive(FAR struct stm32l4_dma_s *dmach)
{
  nxsem_post(&dmach->sem);
}

/****************************************************************************
 * Name: stm32l4_dmachandisable
 *
 * Description:
 *  Disable the DMA channel
 *
 ****************************************************************************/

static void stm32l4_dmachandisable(struct stm32l4_dma_s *dmach)
{
  uint32_t regval;

  /* Disable all interrupts at the DMA controller */

  regval = dmachan_getreg(dmach, STM32L4_DMACHAN_CCR_OFFSET);
  regval &= ~DMA_CCR_ALLINTS;

  /* Disable the DMA channel */

  regval &= ~DMA_CCR_EN;
  dmachan_putreg(dmach, STM32L4_DMACHAN_CCR_OFFSET, regval);

  /* Clear pending channel interrupts */

  dmabase_putreg(dmach, STM32L4_DMA_IFCR_OFFSET,
                 DMA_ISR_CHAN_MASK(dmach->chan));
}

/****************************************************************************
 * Name: stm32l4_dmainterrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int stm32l4_dmainterrupt(int irq, void *context, FAR void *arg)
{
  struct stm32l4_dma_s *dmach;
  uint32_t isr;
  int chndx = 0;

  /* Get the channel structure from the interrupt number */

  if (irq >= STM32L4_IRQ_DMA1CH1 && irq <= STM32L4_IRQ_DMA1CH7)
    {
      chndx = irq - STM32L4_IRQ_DMA1CH1;
    }
  else
#if STM32L4_NDMA > 1
  if (irq >= STM32L4_IRQ_DMA2CH1 && irq <= STM32L4_IRQ_DMA2CH5)
    {
      chndx = irq - STM32L4_IRQ_DMA2CH1 + DMA1_NCHANNELS;
    }
  else if (irq >= STM32L4_IRQ_DMA2CH6 && irq <= STM32L4_IRQ_DMA2CH7)
    {
      chndx = irq - STM32L4_IRQ_DMA2CH6 + DMA1_NCHANNELS + 5;
    }
#endif
  else
    {
      DEBUGPANIC();
    }

  dmach = &g_dma[chndx];

  /* Get the interrupt status (for this channel only) */

  isr = dmabase_getreg(dmach, STM32L4_DMA_ISR_OFFSET) &
        DMA_ISR_CHAN_MASK(dmach->chan);

  /* Invoke the callback */

  if (dmach->callback)
    {
      dmach->callback(dmach, isr >> DMA_ISR_CHAN_SHIFT(dmach->chan),
                      dmach->arg);
    }

  /* Clear the interrupts we are handling */

  dmabase_putreg(dmach, STM32L4_DMA_IFCR_OFFSET, isr);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_dmainitialize
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
  struct stm32l4_dma_s *dmach;
  int chndx;

  /* Initialize each DMA channel */

  for (chndx = 0; chndx < DMA_NCHANNELS; chndx++)
    {
      dmach = &g_dma[chndx];
      nxsem_init(&dmach->sem, 0, 1);

      /* Attach DMA interrupt vectors */

      irq_attach(dmach->irq, stm32l4_dmainterrupt, NULL);

      /* Disable the DMA channel */

      stm32l4_dmachandisable(dmach);

      /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

      up_enable_irq(dmach->irq);
    }
}

/****************************************************************************
 * Name: stm32l4_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'chndx' argument.
 *   DMA channels are shared on the STM32L4:  Devices sharing the same DMA
 *   channel cannot do DMA concurrently!  See the DMACHAN_* definitions in
 *   stm32l4_dma.h.
 *
 *   If the DMA channel is not available, then stm32l4_dmachannel() will wait
 *   until the holder of the channel relinquishes the channel by calling
 *   stm32l4_dmafree().  WARNING: If you have two devices sharing a DMA
 *   channel and the code never releases the channel, the stm32l4_dmachannel
 *   call for the other will hang forever in this function!  Don't let your
 *   design do that!
 *
 *   Hmm.. I suppose this interface could be extended to make a non-blocking
 *   version.  Feel free to do that if that is what you need.
 *
 * Input Parameters:
 *   chan - Identifies the stream/channel resource
 *     This is a bit encoded value as provided by the DMACHAN_* definitions
 *     in chip/stm32l4x6xx_dma.h
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

DMA_HANDLE stm32l4_dmachannel(unsigned int chndef)
{
  int ret;
  int chndx = (chndef & DMACHAN_SETTING_CHANNEL_MASK) >>
               DMACHAN_SETTING_CHANNEL_SHIFT;
  struct stm32l4_dma_s *dmach = &g_dma[chndx];

  DEBUGASSERT(chndx < DMA_NCHANNELS);

  /* Get exclusive access to the DMA channel -- OR wait until the channel
   * is available if it is currently being used by another driver
   */

  ret = stm32l4_dmatake(dmach);
  if (ret < 0)
    {
      return NULL;
    }

  /* The caller now has exclusive use of the DMA channel */

  /* Define the peripheral that will use the channel. This is stored until
   * dmasetup is called.
   */

  dmach->function = (chndef & DMACHAN_SETTING_FUNCTION_MASK) >>
                     DMACHAN_SETTING_FUNCTION_SHIFT;

  return (DMA_HANDLE)dmach;
}

/****************************************************************************
 * Name: stm32l4_dmafree
 *
 * Description:
 *   Release a DMA channel. If another thread is waiting for this DMA channel
 *   in a call to stm32l4_dmachannel, then this function will re-assign the
 *   DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
 *   in this argument must NEVER be used again until stm32l4_dmachannel() is
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

void stm32l4_dmafree(DMA_HANDLE handle)
{
  struct stm32l4_dma_s *dmach = (struct stm32l4_dma_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Release the channel */

  stm32l4_dmagive(dmach);
}

/****************************************************************************
 * Name: stm32l4_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void stm32l4_dmasetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                    size_t ntransfers, uint32_t ccr)
{
  struct stm32l4_dma_s *dmach = (struct stm32l4_dma_s *)handle;
  uint32_t regval;

  DEBUGASSERT(handle != NULL);
  DEBUGASSERT(ntransfers < 65536);

  /* Then DMA_CNDTRx register can only be modified if the DMA channel is
   * disabled.
   */

  regval  = dmachan_getreg(dmach, STM32L4_DMACHAN_CCR_OFFSET);
  regval &= ~(DMA_CCR_EN);
  dmachan_putreg(dmach, STM32L4_DMACHAN_CCR_OFFSET, regval);

  /* Set the peripheral register address in the DMA_CPARx register. The data
   * will be moved from/to this address to/from the memory after the
   * peripheral event.
   */

  dmachan_putreg(dmach, STM32L4_DMACHAN_CPAR_OFFSET, paddr);

  /* Set the memory address in the DMA_CMARx register. The data will be
   * written to or read from this memory after the peripheral event.
   */

  dmachan_putreg(dmach, STM32L4_DMACHAN_CMAR_OFFSET, maddr);

  /* Configure the total number of data to be transferred in the DMA_CNDTRx
   * register.  After each peripheral event, this value will be decremented.
   */

  dmachan_putreg(dmach, STM32L4_DMACHAN_CNDTR_OFFSET, ntransfers);

  /* Configure the channel priority using the PL[1:0] bits in the DMA_CCRx
   * register.  Configure data transfer direction, circular mode, peripheral
   * and memory incremented mode, peripheral & memory data size, and
   * interrupt after half and/or full transfer in the DMA_CCRx register.
   */

  regval  = dmachan_getreg(dmach, STM32L4_DMACHAN_CCR_OFFSET);
  regval &= ~(DMA_CCR_MEM2MEM | DMA_CCR_PL_MASK | DMA_CCR_MSIZE_MASK |
              DMA_CCR_PSIZE_MASK | DMA_CCR_MINC | DMA_CCR_PINC |
              DMA_CCR_CIRC | DMA_CCR_DIR);
  ccr    &=  (DMA_CCR_MEM2MEM | DMA_CCR_PL_MASK | DMA_CCR_MSIZE_MASK |
              DMA_CCR_PSIZE_MASK | DMA_CCR_MINC | DMA_CCR_PINC |
              DMA_CCR_CIRC | DMA_CCR_DIR);
  regval |= ccr;
  dmachan_putreg(dmach, STM32L4_DMACHAN_CCR_OFFSET, regval);

  /* define peripheral indicated in dmach->function */

  regval  = dmabase_getreg(dmach, STM32L4_DMA_CSELR_OFFSET);
  regval &= ~(0x0f << (dmach->chan << 2));
  regval |= (dmach->function << (dmach->chan << 2));
  dmabase_putreg(dmach, STM32L4_DMA_CSELR_OFFSET, regval);
}

/****************************************************************************
 * Name: stm32l4_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by stm32l4_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void stm32l4_dmastart(DMA_HANDLE handle, dma_callback_t callback,
                    void *arg, bool half)
{
  struct stm32l4_dma_s *dmach = (struct stm32l4_dma_s *)handle;
  uint32_t ccr;

  DEBUGASSERT(handle != NULL);

  /* Save the callback info.  This will be invoked when the DMA completes */

  dmach->callback = callback;
  dmach->arg      = arg;

  /* Activate the channel by setting the ENABLE bit in the DMA_CCRx register.
   * As soon as the channel is enabled, it can serve any DMA request from the
   * peripheral connected on the channel.
   */

  ccr  = dmachan_getreg(dmach, STM32L4_DMACHAN_CCR_OFFSET);
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
       * mode to determine when the buffer is half-full, or in
       * double-buffered mode to determine when one of the two buffers is
       * full.
       */

      ccr |= (half ? DMA_CCR_HTIE : 0) | DMA_CCR_TCIE | DMA_CCR_TEIE;
    }

  dmachan_putreg(dmach, STM32L4_DMACHAN_CCR_OFFSET, ccr);
}

/****************************************************************************
 * Name: stm32l4_dmastop
 *
 * Description:
 *   Cancel the DMA.  After stm32l4_dmastop() is called, the DMA channel is
 *   reset and stm32l4_dmasetup() must be called before stm32l4_dmastart()
 *   can be called again
 *
 * Assumptions:
 *   - DMA handle allocated by stm32l4_dmachannel()
 *
 ****************************************************************************/

void stm32l4_dmastop(DMA_HANDLE handle)
{
  struct stm32l4_dma_s *dmach = (struct stm32l4_dma_s *)handle;
  stm32l4_dmachandisable(dmach);
}

/****************************************************************************
 * Name: stm32l4_dmaresidual
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred
 *
 * Assumptions:
 *   - DMA handle allocated by stm32l4_dmachannel()
 *
 ****************************************************************************/

size_t stm32l4_dmaresidual(DMA_HANDLE handle)
{
  struct stm32l4_dma_s *dmach = (struct stm32l4_dma_s *)handle;

  return dmachan_getreg(dmach, STM32L4_DMACHAN_CNDTR_OFFSET);
}

/****************************************************************************
 * Name: stm32l4_dmacapable
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

#ifdef CONFIG_STM32L4_DMACAPABLE
bool stm32l4_dmacapable(uint32_t maddr, uint32_t count, uint32_t ccr)
{
  uint32_t transfer_size;
  uint32_t mend;

  /* Verify that the address conforms to the memory transfer size.
   * Transfers to/from memory performed by the DMA controller are
   * required to be aligned to their size.
   *
   * Datasheet 3.13 claims
   *   "Access to Flash, SRAM, APB and AHB peripherals as source
   *   and destination"
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

  if ((maddr & STM32L4_REGION_MASK) != (mend & STM32L4_REGION_MASK))
    {
      return false;
    }

  switch (maddr & STM32L4_REGION_MASK)
    {
      case STM32L4_PERIPH_BASE:
      case STM32L4_FSMC_BASE:
      case STM32L4_FSMC_BANK1:
      case STM32L4_FSMC_BANK2:
      case STM32L4_FSMC_BANK3:
      case STM32L4_FSMC_BANK4:
      case STM32L4_SRAM_BASE:
      case STM32L4_SRAM2_BASE:
      case STM32L4_CODE_BASE:

        /* All RAM and flash is supported */

        return true;

      default:

        /* Everything else is unsupported by DMA */

        return false;
    }
}
#endif

/****************************************************************************
 * Name: stm32l4_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32l4_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32l4_dmasample(DMA_HANDLE handle, struct stm32l4_dmaregs_s *regs)
{
  struct stm32l4_dma_s *dmach = (struct stm32l4_dma_s *)handle;
  irqstate_t flags;

  flags       = enter_critical_section();
  regs->isr   = dmabase_getreg(dmach, STM32L4_DMA_ISR_OFFSET);
  regs->cselr = dmabase_getreg(dmach, STM32L4_DMA_CSELR_OFFSET);
  regs->ccr   = dmachan_getreg(dmach, STM32L4_DMACHAN_CCR_OFFSET);
  regs->cndtr = dmachan_getreg(dmach, STM32L4_DMACHAN_CNDTR_OFFSET);
  regs->cpar  = dmachan_getreg(dmach, STM32L4_DMACHAN_CPAR_OFFSET);
  regs->cmar  = dmachan_getreg(dmach, STM32L4_DMACHAN_CMAR_OFFSET);
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: stm32l4_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32l4_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32l4_dmadump(DMA_HANDLE handle, const struct stm32l4_dmaregs_s *regs,
                   const char *msg)
{
  struct stm32l4_dma_s *dmach = (struct stm32l4_dma_s *)handle;
  uint32_t dmabase = DMA_BASE(dmach->base);

  dmainfo("DMA Registers: %s\n", msg);
  dmainfo("    ISR[%08x]: %08x\n",
          dmabase + STM32L4_DMA_ISR_OFFSET, regs->isr);
  dmainfo("  CSELR[%08x]: %08x\n",
          dmabase + STM32L4_DMA_CSELR_OFFSET, regs->cselr);
  dmainfo("    CCR[%08x]: %08x\n",
          dmach->base + STM32L4_DMACHAN_CCR_OFFSET, regs->ccr);
  dmainfo("  CNDTR[%08x]: %08x\n",
          dmach->base + STM32L4_DMACHAN_CNDTR_OFFSET, regs->cndtr);
  dmainfo("   CPAR[%08x]: %08x\n",
          dmach->base + STM32L4_DMACHAN_CPAR_OFFSET, regs->cpar);
  dmainfo("   CMAR[%08x]: %08x\n",
          dmach->base + STM32L4_DMACHAN_CMAR_OFFSET, regs->cmar);
}
#endif
