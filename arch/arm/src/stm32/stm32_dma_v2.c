/****************************************************************************
 * arch/arm/src/stm32/stm32_dma_v2.c
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

#include <inttypes.h>
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

/* This file supports the STM32 DMA IP core version 2 - F2, F4, F7, H7
 * NOTE: F7 and H7 need support for DCACHE which is not implemented here
 *       but otherwise DMA IP cores look the same.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMA1_NSTREAMS    8
#if STM32_NDMA > 1
#  define DMA2_NSTREAMS  8
#  define DMA_NSTREAMS   (DMA1_NSTREAMS+DMA2_NSTREAMS)
#else
#  define DMA_NSTREAMS   DMA1_NSTREAMS
#endif

/* Convert the DMA stream base address to the DMA register block address */

#define DMA_BASE(ch)     (ch & 0xfffffc00)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct stm32_dma_s
{
  uint8_t        stream;   /* DMA stream number (0-7) */
  uint8_t        irq;      /* DMA stream IRQ number */
  uint8_t        shift;    /* ISR/IFCR bit shift value */
  uint8_t        channel;  /* DMA channel number (0-7) */
  sem_t          sem;      /* Used to wait for DMA channel to become available */
  uint32_t       base;     /* DMA register channel base address */
  dma_callback_t callback; /* Callback invoked when the DMA completes */
  void          *arg;      /* Argument passed to callback function */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array describes the state of each DMA */

static struct stm32_dma_s g_dma[DMA_NSTREAMS] =
{
  {
    .stream   = 0,
    .irq      = STM32_IRQ_DMA1S0,
    .shift    = DMA_INT_STREAM0_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(0),
  },
  {
    .stream   = 1,
    .irq      = STM32_IRQ_DMA1S1,
    .shift    = DMA_INT_STREAM1_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(1),
  },
  {
    .stream   = 2,
    .irq      = STM32_IRQ_DMA1S2,
    .shift    = DMA_INT_STREAM2_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(2),
  },
  {
    .stream   = 3,
    .irq      = STM32_IRQ_DMA1S3,
    .shift    = DMA_INT_STREAM3_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(3),
  },
  {
    .stream   = 4,
    .irq      = STM32_IRQ_DMA1S4,
    .shift    = DMA_INT_STREAM4_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(4),
  },
  {
    .stream   = 5,
    .irq      = STM32_IRQ_DMA1S5,
    .shift    = DMA_INT_STREAM5_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(5),
  },
  {
    .stream   = 6,
    .irq      = STM32_IRQ_DMA1S6,
    .shift    = DMA_INT_STREAM6_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(6),
  },
  {
    .stream   = 7,
    .irq      = STM32_IRQ_DMA1S7,
    .shift    = DMA_INT_STREAM7_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA1_BASE + STM32_DMA_OFFSET(7),
  },
#if STM32_NDMA > 1
  {
    .stream   = 0,
    .irq      = STM32_IRQ_DMA2S0,
    .shift    = DMA_INT_STREAM0_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(0),
  },
  {
    .stream   = 1,
    .irq      = STM32_IRQ_DMA2S1,
    .shift    = DMA_INT_STREAM1_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(1),
  },
  {
    .stream   = 2,
    .irq      = STM32_IRQ_DMA2S2,
    .shift    = DMA_INT_STREAM2_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(2),
  },
  {
    .stream   = 3,
    .irq      = STM32_IRQ_DMA2S3,
    .shift    = DMA_INT_STREAM3_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(3),
  },
  {
    .stream   = 4,
    .irq      = STM32_IRQ_DMA2S4,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(4),
  },
  {
    .stream   = 5,
    .irq      = STM32_IRQ_DMA2S5,
    .shift    = DMA_INT_STREAM5_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(5),
  },
  {
    .stream   = 6,
    .irq      = STM32_IRQ_DMA2S6,
    .shift    = DMA_INT_STREAM6_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(6),
  },
  {
    .stream   = 7,
    .irq      = STM32_IRQ_DMA2S7,
    .shift    = DMA_INT_STREAM7_SHIFT,
    .sem      = SEM_INITIALIZER(1),
    .base     = STM32_DMA2_BASE + STM32_DMA_OFFSET(7),
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

static inline uint32_t dmabase_getreg(struct stm32_dma_s *dmast,
                                      uint32_t offset)
{
  return getreg32(DMA_BASE(dmast->base) + offset);
}

/* Write to non-channel register in DMA1 or DMA2 */

static inline void dmabase_putreg(struct stm32_dma_s *dmast, uint32_t offset,
                                  uint32_t value)
{
  putreg32(value, DMA_BASE(dmast->base) + offset);
}

/* Get channel register from DMA1 or DMA2 */

static inline uint32_t dmast_getreg(struct stm32_dma_s *dmast,
                                    uint32_t offset)
{
  return getreg32(dmast->base + offset);
}

/* Write to channel register in DMA1 or DMA2 */

static inline void dmast_putreg(struct stm32_dma_s *dmast, uint32_t offset,
                                uint32_t value)
{
  putreg32(value, dmast->base + offset);
}

/****************************************************************************
 * Name: stm32_dmastream
 *
 * Description:
 *   Get the g_dma table entry associated with a DMA controller and a stream
 *   number
 *
 ****************************************************************************/

static inline struct stm32_dma_s *stm32_dmastream(unsigned int stream,
                                                  unsigned int controller)
{
  int index;

  DEBUGASSERT(stream < DMA_NSTREAMS && controller < STM32_NDMA);

  /* Convert the controller + stream based on the fact that there are
   * 8 streams per controller.
   */

#if STM32_NDMA > 1
  index = controller << 3 | stream;
#else
  index = stream;
#endif

  /* Then return the stream structure associated with the stream index */

  return &g_dma[index];
}

/****************************************************************************
 * Name: stm32_dmamap
 *
 * Description:
 *   Get the g_dma table entry associated with a bit-encoded DMA selection
 *
 ****************************************************************************/

static inline struct stm32_dma_s *stm32_dmamap(unsigned long dmamap)
{
  /* Extract the DMA controller number from the bit encoded value */

  unsigned int controller = STM32_DMA_CONTROLLER(dmamap);

  /* Extract the stream number from the bit encoded value */

  unsigned int stream = STM32_DMA_STREAM(dmamap);

  /* Return the table entry associated with the controller + stream */

  return stm32_dmastream(stream, controller);
}

/****************************************************************************
 * Name: stm32_dmastreamdisable
 *
 * Description:
 *  Disable the DMA stream
 *
 ****************************************************************************/

static void stm32_dmastreamdisable(struct stm32_dma_s *dmast)
{
  uint32_t regoffset;
  uint32_t regval;

  /* Disable all interrupts at the DMA controller */

  regval = dmast_getreg(dmast, STM32_DMA_SCR_OFFSET);
  regval &= ~DMA_SCR_ALLINTS;

  /* Disable the DMA stream */

  regval &= ~DMA_SCR_EN;
  dmast_putreg(dmast, STM32_DMA_SCR_OFFSET, regval);

  /* Clear pending stream interrupts by setting bits in the upper or lower
   * IFCR register.
   */

  if (dmast->stream < 4)
    {
      regoffset = STM32_DMA_LIFCR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HIFCR_OFFSET;
    }

  dmabase_putreg(dmast, regoffset, (DMA_STREAM_MASK << dmast->shift));
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
  struct stm32_dma_s *dmast;
  uint32_t status;
  uint32_t regoffset = 0;
  unsigned int stream = 0;
  unsigned int controller = 0;

  /* Get the stream and the controller that generated the interrupt */

  if (irq >= STM32_IRQ_DMA1S0 && irq <= STM32_IRQ_DMA1S6)
    {
      stream     = irq - STM32_IRQ_DMA1S0;
      controller = DMA1;
    }
  else if (irq == STM32_IRQ_DMA1S7)
    {
      stream     = 7;
      controller = DMA1;
    }
  else
#if STM32_NDMA > 1
  if (irq >= STM32_IRQ_DMA2S0 && irq <= STM32_IRQ_DMA2S4)
    {
      stream     = irq - STM32_IRQ_DMA2S0;
      controller = DMA2;
    }
  else if (irq >= STM32_IRQ_DMA2S5 && irq <= STM32_IRQ_DMA2S7)
    {
      stream     = irq - STM32_IRQ_DMA2S5 + 5;
      controller = DMA2;
    }
  else
#endif
    {
      DEBUGPANIC();
    }

  /* Get the stream structure from the stream and controller numbers */

  dmast = stm32_dmastream(stream, controller);

  /* Select the interrupt status register (either the LISR or HISR)
   * based on the stream number that caused the interrupt.
   */

  if (stream < 4)
    {
      regoffset = STM32_DMA_LISR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HISR_OFFSET;
    }

  /* Get the interrupt status for this stream */

  status = (dmabase_getreg(dmast, regoffset) >> dmast->shift) &
            DMA_STREAM_MASK;

  /* Clear fetched stream interrupts by setting bits in the upper or lower
   * IFCR register.
   */

  if (stream < 4)
    {
      regoffset = STM32_DMA_LIFCR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HIFCR_OFFSET;
    }

  dmabase_putreg(dmast, regoffset, (status << dmast->shift));

  /* Invoke the callback */

  if (dmast->callback)
    {
      dmast->callback(dmast, status, dmast->arg);
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
  struct stm32_dma_s *dmast;
  int stream;

  /* Initialize each DMA stream */

  for (stream = 0; stream < DMA_NSTREAMS; stream++)
    {
      dmast = &g_dma[stream];

      /* Attach DMA interrupt vectors */

      irq_attach(dmast->irq, stm32_dmainterrupt, dmast);

      /* Disable the DMA stream */

      stm32_dmastreamdisable(dmast);

      /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

      up_enable_irq(dmast->irq);
    }
}

/****************************************************************************
 * Name: stm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'dmamap' argument.
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
 *   dmamap - Identifies the stream/channel resource. For the STM32 F4, this
 *     is a bit-encoded  value as provided by the DMAMAP_* definitions
 *     in chip/stm32f40xxx_dma.h
 *
 * Returned Value:
 *   Provided that 'dmamap' is valid, this function ALWAYS returns a non-NULL
 *   void* DMA channel handle.  (If 'dmamap' is invalid, the function will
 *   assert if debug is enabled or do something ignorant otherwise).
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE stm32_dmachannel(unsigned int dmamap)
{
  struct stm32_dma_s *dmast;
  int ret;

  /* Get the stream index from the bit-encoded channel value */

  dmast = stm32_dmamap(dmamap);
  DEBUGASSERT(dmast != NULL);

  /* Get exclusive access to the DMA channel -- OR wait until the channel
   * is available if it is currently being used by another driver
   */

  ret = nxsem_wait_uninterruptible(&dmast->sem);
  if (ret < 0)
    {
      return NULL;
    }

  /* The caller now has exclusive use of the DMA channel.  Assign the
   * channel to the stream and return an opaque reference to the stream
   * structure.
   */

  dmast->channel = STM32_DMA_CHANNEL(dmamap);
  return (DMA_HANDLE)dmast;
}

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA
 *   channel in a call to stm32_dmachannel, then this function will re-assign
 *   the DMA channel to that thread and wake it up.  NOTE:  The 'handle' used
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
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Release the channel */

  nxsem_post(&dmast->sem);
}

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void stm32_dmasetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                    size_t ntransfers, uint32_t scr)
{
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  uint32_t regoffset;
  uint32_t regval;

  dmainfo("paddr: %08" PRIx32 " maddr: %08" PRIx32
          " ntransfers: %zu scr: %08" PRIx32 "\n",
          paddr, maddr, ntransfers, scr);

#ifdef CONFIG_STM32_DMACAPABLE
  DEBUGASSERT(stm32_dmacapable(maddr, ntransfers, scr));
#endif

  /* "If the stream is enabled, disable it by resetting the EN bit in the
   * DMA_SxCR register, then read this bit in order to confirm that there is
   * no ongoing stream operation. Writing this bit to 0 is not immediately
   * effective since it is actually written to 0 once all the current
   * transfers have finished. When the EN bit is read as 0, this means that
   * the stream is ready to be configured. It is therefore necessary to wait
   * for the EN bit to be cleared before starting any stream
   * configuration. ..."
   */

  while ((dmast_getreg(dmast, STM32_DMA_SCR_OFFSET) & DMA_SCR_EN) != 0);

  /* "... All the stream dedicated bits set in the status register (DMA_LISR
   * and DMA_HISR) from the previous data block DMA transfer should be
   * cleared before the stream can be re-enabled."
   *
   * Clear pending stream interrupts by setting bits in the upper or lower
   * IFCR register.
   */

  if (dmast->stream < 4)
    {
      regoffset = STM32_DMA_LIFCR_OFFSET;
    }
  else
    {
      regoffset = STM32_DMA_HIFCR_OFFSET;
    }

  dmabase_putreg(dmast, regoffset, (DMA_STREAM_MASK << dmast->shift));

  /* "Set the peripheral register address in the DMA_SPARx register. The data
   *  will be moved from/to this address to/from the memory after the
   *  peripheral event.
   */

  dmast_putreg(dmast, STM32_DMA_SPAR_OFFSET, paddr);

  /* "Set the memory address in the DMA_SM0ARx ... register. The data will be
   *  written to or read from this memory after the peripheral event."
   *
   * Note that in double-buffered mode it is explicitly assumed that the
   * second buffer immediately follows the first.
   */

  dmast_putreg(dmast, STM32_DMA_SM0AR_OFFSET, maddr);
  if (scr & DMA_SCR_DBM)
    {
      dmast_putreg(dmast, STM32_DMA_SM1AR_OFFSET, maddr + ntransfers);
    }

  /* "Configure the total number of data items to be transferred in the
   *  DMA_SNDTRx register.  After each peripheral event, this value will be
   *  decremented."
   *
   * "When the peripheral flow controller is used for a given stream,
   * the value written into the DMA_SxNDTR has no effect on the DMA
   * transfer. Actually, whatever the value written, it will be forced by
   * hardware to 0xFFFF as soon as the stream is enabled..."
   */

  dmast_putreg(dmast, STM32_DMA_SNDTR_OFFSET, ntransfers);

  /* "Select the DMA channel (request) using CHSEL[2:0] in the DMA_SxCR
   *  register."
   *
   * "Configure the stream priority using the PL[1:0] bits in the DMA_SCRx"
   *  register."
   */

  regval  = dmast_getreg(dmast, STM32_DMA_SCR_OFFSET);
  regval &= ~(DMA_SCR_PL_MASK | DMA_SCR_CHSEL_MASK);
  regval |= scr & DMA_SCR_PL_MASK;
  regval |= (uint32_t)dmast->channel << DMA_SCR_CHSEL_SHIFT;
  dmast_putreg(dmast, STM32_DMA_SCR_OFFSET, regval);

  /* "Configure the FIFO usage (enable or disable, threshold in transmission
   *  and reception)"
   *
   * "Caution is required when choosing the FIFO threshold (bits FTH[1:0] of
   *  the DMA_SxFCR register) and the size of the memory burst (MBURST[1:0]
   *  of the DMA_SxCR register): The content pointed by the FIFO threshold
   *  must exactly match to an integer number of memory burst transfers.
   *  If this is not in the case, a FIFO error (flag FEIFx of the DMA_HISR
   *  or DMA_LISR register) will be generated when the stream is enabled,
   *  then the stream will be automatically disabled."
   *
   * The FIFO is disabled in circular mode when transferring data from a
   * peripheral to memory, as in this case it is usually desirable to know
   * that every byte from the peripheral is transferred immediately to memory
   * It is not practical to flush the DMA FIFO, as this requires disabling
   * the channel which triggers the transfer-complete interrupt.
   *
   * NOTE: The FEIFx error interrupt is not enabled because the FEIFx seems
   * to be reported spuriously causing good transfers to be marked as
   * failures.
   */

  regval  = dmast_getreg(dmast, STM32_DMA_SFCR_OFFSET);
  regval &= ~(DMA_SFCR_FTH_MASK | DMA_SFCR_FS_MASK | DMA_SFCR_FEIE);
  if (!((scr & (DMA_SCR_CIRC | DMA_SCR_DIR_MASK)) ==
        (DMA_SCR_CIRC | DMA_SCR_DIR_P2M)))
    {
      regval |= (DMA_SFCR_FTH_FULL | DMA_SFCR_DMDIS);
    }

  dmast_putreg(dmast, STM32_DMA_SFCR_OFFSET, regval);

  /* "Configure data transfer direction, circular mode, peripheral & memory
   *  incremented mode, peripheral & memory data size, and interrupt after
   *  half and/or full transfer in the DMA_CCRx register."
   *
   * Note: The CT bit is always reset.
   */

  regval  = dmast_getreg(dmast, STM32_DMA_SCR_OFFSET);
  regval &= ~(DMA_SCR_PFCTRL | DMA_SCR_DIR_MASK | DMA_SCR_PINC |
              DMA_SCR_MINC | DMA_SCR_PSIZE_MASK | DMA_SCR_MSIZE_MASK |
              DMA_SCR_PINCOS | DMA_SCR_CIRC | DMA_SCR_DBM | DMA_SCR_CT |
              DMA_SCR_PBURST_MASK | DMA_SCR_MBURST_MASK);
  scr    &=  (DMA_SCR_PFCTRL | DMA_SCR_DIR_MASK | DMA_SCR_PINC |
              DMA_SCR_MINC | DMA_SCR_PSIZE_MASK | DMA_SCR_MSIZE_MASK |
              DMA_SCR_PINCOS | DMA_SCR_DBM | DMA_SCR_CIRC |
              DMA_SCR_PBURST_MASK | DMA_SCR_MBURST_MASK);
  regval |= scr;
  dmast_putreg(dmast, STM32_DMA_SCR_OFFSET, regval);
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

void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half)
{
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  uint32_t scr;

  DEBUGASSERT(handle != NULL);

  /* Save the callback info.  This will be invoked whent the DMA completes */

  dmast->callback = callback;
  dmast->arg      = arg;

  /* Activate the stream by setting the ENABLE bit in the DMA_SCRx register.
   * As soon as the stream is enabled, it can serve any DMA request from the
   * peripheral connected on the stream.
   */

  scr  = dmast_getreg(dmast, STM32_DMA_SCR_OFFSET);
  scr |= DMA_SCR_EN;

  /* In normal mode, interrupt at either half or full completion. In circular
   * and double-buffered modes, always interrupt on buffer wrap, and
   * optionally interrupt at the halfway point.
   */

  if ((scr & (DMA_SCR_DBM | DMA_SCR_CIRC)) == 0)
    {
      /* Once half of the bytes are transferred, the half-transfer flag
       * (HTIF) is set and an interrupt is generated if the Half-Transfer
       * Interrupt Enable bit (HTIE) is set. At the end of the transfer,
       * the Transfer Complete Flag (TCIF) is set and an interrupt is
       * generated if the Transfer Complete Interrupt Enable bit (TCIE)
       * is set.
       */

      scr |= (half ?
              (DMA_SCR_HTIE | DMA_SCR_TEIE) : (DMA_SCR_TCIE | DMA_SCR_TEIE));
    }
  else
    {
      /* In non-stop modes, when the transfer completes it immediately resets
       * and starts again.  The transfer-complete interrupt is thus always
       * enabled, and the half-complete interrupt can be used in circular
       * mode to determine when the buffer is half-full or in double-buffered
       * mode to determine when one of the two buffers is full.
       */

      scr |= (half ? DMA_SCR_HTIE : 0) | DMA_SCR_TCIE | DMA_SCR_TEIE;
    }

  dmast_putreg(dmast, STM32_DMA_SCR_OFFSET, scr);
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
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  stm32_dmastreamdisable(dmast);
}

/****************************************************************************
 * Name: stm32_dmaresidual
 *
 * Description:
 *   Read the DMA bytes-remaining register.
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

size_t stm32_dmaresidual(DMA_HANDLE handle)
{
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  uint32_t residual;

  /* Fetch the count of bytes remaining to be transferred.
   *
   * If the FIFO is enabled, this count may be inaccurate.  ST don't
   * appear to document whether this counts the peripheral or the memory
   * side of the channel, and they don't make the memory pointer
   * available either.
   *
   * For reception in circular mode the FIFO is disabled in order that
   * this value can be useful.
   */

  residual = dmast_getreg(dmast, STM32_DMA_SNDTR_OFFSET);

  return (size_t)residual;
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
  uint32_t burst_length;
  uint32_t mend;

  dmainfo("stm32_dmacapable: 0x%08" PRIxPTR "/%" PRIu32 " 0x%08" PRIx32 "\n",
          maddr, count, ccr);

  /* Verify that the address conforms to the memory transfer size.
   * Transfers to/from memory performed by the DMA controller are
   * required to be aligned to their size.
   *
   * See ST RM0090 rev4, section 9.3.11
   *
   * Compute mend inline to avoid a possible non-constant integer
   * multiply.
   */

  switch (ccr & DMA_SCR_MSIZE_MASK)
    {
      case DMA_SCR_MSIZE_8BITS:
        transfer_size = 1;
        mend = maddr + count - 1;
        break;

      case DMA_SCR_MSIZE_16BITS:
        transfer_size = 2;
        mend = maddr + (count << 1) - 1;
        break;

      case DMA_SCR_MSIZE_32BITS:
        transfer_size = 4;
        mend = maddr + (count << 2) - 1;
        break;

      default:
        dmainfo("stm32_dmacapable: bad transfer size in CCR\n");
        return false;
    }

  if ((maddr & (transfer_size - 1)) != 0)
    {
      dmainfo("stm32_dmacapable: transfer unaligned\n");
      return false;
    }

  /* Verify that burst transfers do not cross a 1KiB boundary. */

  if ((maddr / 1024) != (mend / 1024))
    {
      /* The transfer as a whole crosses a 1KiB boundary.
       * Verify that no burst does by asserting that the address
       * is aligned to the burst length.
       */

      switch (ccr & DMA_SCR_MBURST_MASK)
        {
          case DMA_SCR_MBURST_SINGLE:
            burst_length = transfer_size;
            break;

          case DMA_SCR_MBURST_INCR4:
            burst_length = transfer_size << 2;
            break;

          case DMA_SCR_MBURST_INCR8:
            burst_length = transfer_size << 3;
            break;

          case DMA_SCR_MBURST_INCR16:
            burst_length = transfer_size << 4;
            break;

          default:
          dmainfo("stm32_dmacapable: bad burst size in CCR\n");
            return false;
        }

      if ((maddr & (burst_length - 1)) != 0)
        {
          dmainfo("stm32_dmacapable: burst crosses 1KiB\n");
          return false;
        }
    }

  /* Verify that the transfer is to a memory region that supports DMA. */

  if ((maddr & STM32_REGION_MASK) != (mend & STM32_REGION_MASK))
    {
      dmainfo("stm32_dmacapable: transfer crosses memory region\n");
      return false;
    }

  switch (maddr & STM32_REGION_MASK)
    {
      case STM32_FSMC_BANK1:
      case STM32_FSMC_BANK2:
      case STM32_FSMC_BANK3:
      case STM32_FSMC_BANK4:
      case STM32_SRAM_BASE:

        /* All RAM is supported */

        break;

      case STM32_CODE_BASE:

        /* Everything except the CCM ram is supported */

        if (maddr >= STM32_CCMRAM_BASE &&
            (maddr - STM32_CCMRAM_BASE) < 65536)
          {
            dmainfo("stm32_dmacapable: transfer targets CCMRAM\n");
            return false;
          }

        break;

      default:

        /* Everything else is unsupported by DMA */

        dmainfo("stm32_dmacapable:"
                " transfer targets unknown/unsupported region\n");
        return false;
    }

    dmainfo("stm32_dmacapable: transfer OK\n");
  return true;
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
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  irqstate_t flags;

  flags       = enter_critical_section();
  regs->lisr  = dmabase_getreg(dmast, STM32_DMA_LISR_OFFSET);
  regs->hisr  = dmabase_getreg(dmast, STM32_DMA_HISR_OFFSET);
  regs->scr   = dmast_getreg(dmast, STM32_DMA_SCR_OFFSET);
  regs->sndtr = dmast_getreg(dmast, STM32_DMA_SNDTR_OFFSET);
  regs->spar  = dmast_getreg(dmast, STM32_DMA_SPAR_OFFSET);
  regs->sm0ar = dmast_getreg(dmast, STM32_DMA_SM0AR_OFFSET);
  regs->sm1ar = dmast_getreg(dmast, STM32_DMA_SM1AR_OFFSET);
  regs->sfcr  = dmast_getreg(dmast, STM32_DMA_SFCR_OFFSET);
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
  struct stm32_dma_s *dmast = (struct stm32_dma_s *)handle;
  uint32_t dmabase = DMA_BASE(dmast->base);

  dmainfo("DMA Registers: %s\n", msg);
  dmainfo("   LISR[%08x]: %08x\n",
          dmabase + STM32_DMA_LISR_OFFSET, regs->lisr);
  dmainfo("   HISR[%08x]: %08x\n",
          dmabase + STM32_DMA_HISR_OFFSET, regs->hisr);
  dmainfo("    SCR[%08x]: %08x\n",
          dmast->base + STM32_DMA_SCR_OFFSET, regs->scr);
  dmainfo("  SNDTR[%08x]: %08x\n",
          dmast->base + STM32_DMA_SNDTR_OFFSET, regs->sndtr);
  dmainfo("   SPAR[%08x]: %08x\n",
          dmast->base + STM32_DMA_SPAR_OFFSET, regs->spar);
  dmainfo("  SM0AR[%08x]: %08x\n",
          dmast->base + STM32_DMA_SM0AR_OFFSET, regs->sm0ar);
  dmainfo("  SM1AR[%08x]: %08x\n",
          dmast->base + STM32_DMA_SM1AR_OFFSET, regs->sm1ar);
  dmainfo("   SFCR[%08x]: %08x\n",
          dmast->base + STM32_DMA_SFCR_OFFSET, regs->sfcr);
}
#endif

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT

/****************************************************************************
 * Name: stm32_dma_intack
 *
 * Description:
 *   Public visible interface to acknowledge interrupts on DMA stream
 *
 ****************************************************************************/

void stm32_dma_intack(unsigned int controller, uint8_t stream, uint32_t isr)
{
  struct stm32_dma_s *dmast = stm32_dmastream(stream, controller);
  uint32_t regval = 0;
  uint32_t offset = 0;

  /* Select the interrupt flag clear register (either the LIFCR or HIFCR)
   * based on the stream number
   */

  if (stream < 4)
    {
      offset = STM32_DMA_LIFCR_OFFSET;
    }
  else
    {
      offset = STM32_DMA_HIFCR_OFFSET;
    }

  /* Get value to write */

  regval |= ((isr & DMA_STREAM_MASK) << dmast->shift);

  /* Write register */

  dmabase_putreg(dmast, offset, regval);
}

/****************************************************************************
 * Name: stm32_dma_intget
 *
 * Description:
 *   Public visible interface to get pending interrupts from DMA stream
 *
 ****************************************************************************/

uint8_t stm32_dma_intget(unsigned int controller, uint8_t stream)
{
  struct stm32_dma_s *dmast = stm32_dmastream(stream, controller);
  uint32_t regval = 0;
  uint32_t offset = 0;

  /* Select the interrupt status register (either the LISR or HISR)
   * based on the stream number
   */

  if (stream < 4)
    {
      offset = STM32_DMA_LISR_OFFSET;
    }
  else
    {
      offset = STM32_DMA_HISR_OFFSET;
    }

  /* Get register value */

  regval = dmabase_getreg(dmast, offset);

  /* Get stream status */

  regval = ((regval >> dmast->shift) & DMA_STREAM_MASK);

  return (uint8_t)regval;
}
#endif /* CONFIG_ARCH_HIPRI_INTERRUPT */
