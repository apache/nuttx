/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dma.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/signal.h>

#include "arm_internal.h"
#include "sched/sched.h"
#include "stm32_dma.h"
#include "hardware/stm32_gpdma.h"
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* For GPDMA peripheral, each channel has channel specific addresses that are
 * at a base offset based on channel. Channel reg references will be based
 * off this in this file.
 */

#define CH_BASE_OFFSET(ch)  (0x80*(ch)) 
#define CH_CXLBAR_OFFSET     0x50
#define CH_CXFCR_OFFSET      0x5C
#define CH_CXSR_OFFSET       0x60
#define CH_CXCR_OFFSET       0x64
#define CH_CXTR1_OFFSET      0x90
#define CH_CXTR2_OFFSET      0x94
#define CH_CXBR1_OFFSET      0x98
#define CH_CXSAR_OFFSET      0x9C
#define CH_CXDAR_OFFSET      0xA0
#define CH_CXTR3_OFFSET      0xA4
#define CH_CXBR2_OFFSET      0xA8
#define CH_CXLLR_OFFSET      0xCC

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_gpdma_lli_s
{
  uint32_t tr1;   /* GPDMA_CxTR1 value */
  uint32_t tr2;   /* GPDMA_CxTR2 value */
  uint32_t br1;   /* GPDMA_CxBR1 value (block size in bytes) */
  uint32_t sar;   /* GPDMA_CxSAR (source address) */
  uint32_t dar;   /* GPDMA_CxDAR (dest address) */
  uint32_t llr;   /* GPDMA_CxLLR (pointer+update bits) */
}
__attribute__ ((aligned(32)));

struct gpdma_ch_s
{
  uint8_t            dma_instance; /* GPDMA1 or GPDMA2 */
  uint8_t            channel;
  uint8_t            irq;
  enum gpdma_ttype_e type;
  bool               free;         /* Is this channel free to use. */
  uint32_t           base;         /* Channel base address */
  dma_callback_t     callback;
  void              *arg;
  struct stm32_gpdma_cfg_s cfg;   /* Configuration passed at channel setup */
  struct stm32_gpdma_lli_s lli[2];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t gpdmach_getreg(struct gpdma_ch_s *chan,
                                      uint32_t offset);
static inline void gpdmach_putreg(struct gpdma_ch_s *chan, uint32_t offset,
                                  uint32_t value);
static inline void gpdmach_modifyreg32(struct gpdma_ch_s *chan,
                                       uint32_t offset, uint32_t clrbits,
                                       uint32_t setbits);
static void gpdma_ch_abort(struct gpdma_ch_s *chan);
static void gpdma_ch_disable(struct gpdma_ch_s *chan);

static int gpdma_setup(struct gpdma_ch_s *chan,
                       struct stm32_gpdma_cfg_s *cfg);
static int gpdma_setup_circular(struct gpdma_ch_s *chan,
                                struct stm32_gpdma_cfg_s *cfg);
static int gpdma_dmainterrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32H5_DMA1
static struct gpdma_ch_s g_chan[] =
{
  {
    .dma_instance = 1,
    .channel = 0,
    .irq = STM32_IRQ_GPDMA1_CH0,
    .free = true,
    .base = STM32_DMA1_BASE + CH_BASE_OFFSET(0)
  },
  {
    .dma_instance = 1,
    .channel = 1,
    .irq = STM32_IRQ_GPDMA1_CH1,
    .free = true,
    .base = STM32_DMA1_BASE + CH_BASE_OFFSET(1)
  },
  {
    .dma_instance = 1,
    .channel = 2,
    .irq = STM32_IRQ_GPDMA1_CH2,
    .free = true,
    .base = STM32_DMA1_BASE + CH_BASE_OFFSET(2)
  },
  {
    .dma_instance = 1,
    .channel = 3,
    .irq = STM32_IRQ_GPDMA1_CH3,
    .free = true,
    .base = STM32_DMA1_BASE + CH_BASE_OFFSET(3)
  },
#endif
#ifdef CONFIG_STM32H5_DMA2
  {
    .dma_instance = 2,
    .channel = 0,
    .irq = STM32_IRQ_GPDMA2_CH0,
    .free = true,
    .base = STM32_DMA2_BASE + CH_BASE_OFFSET(0)
  },
  {
    .dma_instance = 2,
    .channel = 1,
    .irq = STM32_IRQ_GPDMA2_CH1,
    .free = true,
    .base = STM32_DMA2_BASE + CH_BASE_OFFSET(1)
  },
  {
    .dma_instance = 2,
    .channel = 2,
    .irq = STM32_IRQ_GPDMA2_CH2,
    .free = true,
    .base = STM32_DMA2_BASE + CH_BASE_OFFSET(2)
  },
  {
    .dma_instance = 2,
    .channel = 3,
    .irq = STM32_IRQ_GPDMA2_CH3,
    .free = true,
    .base = STM32_DMA2_BASE + CH_BASE_OFFSET(3)
  }
#endif
};

static uint32_t circ_addr_1;

static uint32_t circ_addr_1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t gpdmach_getreg(struct gpdma_ch_s *chan,
                                      uint32_t offset)
{
  return getreg32(chan->base + offset);
}

static inline void gpdmach_putreg(struct gpdma_ch_s *chan, uint32_t offset,
                                  uint32_t value)
{
  putreg32(value, chan->base + offset);
}

static inline void gpdmach_modifyreg32(struct gpdma_ch_s *chan,
                                       uint32_t offset, uint32_t clrbits,
                                       uint32_t setbits)
{
  modifyreg32(chan->base + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: gpdma_dmainterrupt
 *
 * Description:
 *   DMA interrupt handler.
 *
 ****************************************************************************/

static int gpdma_dmainterrupt(int irq, void *context, void *arg)
{
  struct gpdma_ch_s *chan = NULL;
  uint32_t status;

  /* Get the channel that generated the interrupt */

  if (irq >= STM32_IRQ_GPDMA1_CH0 && irq <= STM32_IRQ_GPDMA1_CH7)
    {
      chan = &g_chan[irq - STM32_IRQ_GPDMA1_CH0];
    }
  else if (irq >= STM32_IRQ_GPDMA2_CH0 && irq <= STM32_IRQ_GPDMA2_CH7)
    {
      chan = &g_chan[irq - STM32_IRQ_GPDMA2_CH0 + 4];
    }
  else
    {
      DEBUGPANIC();
    }

  /* Get the interrupt status for this channel */

  status = (gpdmach_getreg(chan, CH_CXSR_OFFSET) >> 8) & 0x7f;

  /* Clear the fetched channel interrupts by setting bits in the flag
   * clear register
   */

  gpdmach_putreg(chan, CH_CXFCR_OFFSET, ~0);

  /* Invoke the callback */

  if (chan->callback)
    {
      chan->callback(chan, (uint8_t)status, chan->arg);
    }

  return 0;
}

/****************************************************************************
 * Name: gpdma_ch_abort
 *
 * Description:
 *   For the given channel, suspend and abort any ongoing channel transfers.
 *   Returns after the abort has complete and taken effect.
 *
 ****************************************************************************/

static void gpdma_ch_abort(struct gpdma_ch_s *chan)
{
  if ((gpdmach_getreg(chan, CH_CXCR_OFFSET) & GPDMA_CXCR_EN) == 0)
    {
      return;
    }

  /* 1. Software writes 1 to the GPDMA_CxCR.SUSP bit */

  gpdmach_putreg(chan, CH_CXCR_OFFSET, GPDMA_CXCR_SUSP);

  /* 2. Polls suspend flag GPDMA_CxSR.SUSPF until SUSPF = 1, or waits for an
   *    interrupt previously enabled by writing 1 to GPDMA_CxCR.SUSPIE.
   */

  while ((gpdmach_getreg(chan, CH_CXSR_OFFSET) & GPDMA_CXSR_SUSPF) == 0)
    {
    }

  /* 3. Reset chan by writing 1 to GPDMA_CxCR.RESET */

  gpdmach_putreg(chan, CH_CXCR_OFFSET, GPDMA_CXCR_RESET);

  /* 4. Wait for GPDMA_CxCR.EN and GPDMA_CxCR.SUSP bits to be reset */

  while ((gpdmach_getreg(chan, CH_CXCR_OFFSET) &
         (GPDMA_CXCR_EN | GPDMA_CXCR_SUSP)) != 0)
    {
    }
}

/****************************************************************************
 * Name: gpdma_ch_disable
 *
 * Description:
 *   Disable the DMA channel.
 *
 ****************************************************************************/

static void gpdma_ch_disable(struct gpdma_ch_s *chan)
{
  DEBUGASSERT(chan != NULL);

  gpdma_ch_abort(chan);

  /* Disable and clear all interrupts. */

  gpdmach_modifyreg32(chan, CH_CXCR_OFFSET, GPDMA_CXCR_ALLINTS, 0);
  gpdmach_modifyreg32(chan, CH_CXFCR_OFFSET, 0, ~0);
}

/****************************************************************************
 * Name: gpdma_setup
 *
 * Assumptions:
 *   - EN bit not set. Channel must have been aborted before this is called.
 *
 ****************************************************************************/

static int gpdma_setup(struct gpdma_ch_s *chan,
                       struct stm32_gpdma_cfg_s *cfg)
{
  uint32_t reg;

  /* Make sure not to use linked list mode. */

  gpdmach_modifyreg32(chan, CH_CXLLR_OFFSET, ~0, 0);
  gpdmach_putreg(chan, CH_CXLBAR_OFFSET, 0);

  /* Set source and destination addresses. */

  gpdmach_putreg(chan, CH_CXSAR_OFFSET, cfg->src_addr);
  gpdmach_putreg(chan, CH_CXDAR_OFFSET, cfg->dest_addr);

  /* Set the channel priority according to configuration. */

  gpdmach_modifyreg32(chan, CH_CXCR_OFFSET, GPDMA_CXCR_PRIO_MASK,
                      cfg->priority << GPDMA_CXCR_PRIO_SHIFT);

  /* Set channels TR1 register based on configuration provided. */

  gpdmach_putreg(chan, CH_CXTR1_OFFSET, cfg->tr1);

  /* Assemble the required config for TR2 */

  reg = (uint32_t)cfg->request &
        (GPDMA_CXTR2_REQSEL_MASK | GPDMA_CXTR2_DREQ | GPDMA_CXTR2_SWREQ);
  gpdmach_putreg(chan, CH_CXTR2_OFFSET, reg);

  /* Calculate block number of data bytes to transfer, update BR1 */

  reg = cfg->ntransfers;

  if (cfg->mode & GPDMACFG_MODE_CIRC)
    {
      /* This only targets peripheral to memory, with memory increment */

      circ_addr_1 = cfg->dest_addr;
      gpdmach_putreg(chan, CH_CXLBAR_OFFSET,
                    (uint32_t)&circ_addr_1 & (0xffff << 16));

      reg = GPDMA_CXLLR_UDA | ((uint32_t)&circ_addr_1 & GPDMA_CXLLR_LA_MASK);
      gpdmach_putreg(chan, CH_CXLLR_OFFSET, reg);
    }

  gpdmach_putreg(chan, CH_CXBR1_OFFSET, reg);

  return 0;
}

/****************************************************************************
 * Name: gpdma_setup_circular
 *
 * Description:
 *   Circular DMA requires linked list items (LLI) to setup properly. This
 *   function handles LLI allocation and handling necessary to implement
 *   circular DMA.
 *
 ****************************************************************************/

static int gpdma_setup_circular(struct gpdma_ch_s *chan,
                                struct stm32_gpdma_cfg_s *cfg)
{
  struct stm32_gpdma_lli_s *lli = chan->lli;

  lli[0].tr1 = cfg->tr1;
  lli[0].tr2 = (2U << GPDMA_CXTR2_TCEM_SHIFT)
             | (cfg->request & GPDMA_CXTR2_REQSEL_MASK);
  lli[0].br1 = cfg->ntransfers;
  lli[0].sar = cfg->src_addr;
  lli[0].dar = cfg->dest_addr;
  lli[0].llr = (GPDMA_CXLLR_UT1  /* reload TR1 */
             | GPDMA_CXLLR_UT2   /* reload TR2 */
             | GPDMA_CXLLR_UB1   /* reload BR1 */
             | GPDMA_CXLLR_USA   /* reload SAR */
             | GPDMA_CXLLR_UDA   /* reload DAR */
             | GPDMA_CXLLR_ULL)  /* reload LLR */
             | (((uint32_t)&lli[1]) & GPDMA_CXLLR_LA_MASK);

  lli[1].tr1 = lli[0].tr1;
  lli[1].tr2 = lli[0].tr2;
  lli[1].br1 = lli[0].br1;
  lli[1].sar = lli[0].sar;
  lli[1].dar = lli[0].dar;
  lli[1].llr = (lli[0].llr & ~GPDMA_CXLLR_LA_MASK)
             | (((uint32_t)&lli[0]) & GPDMA_CXLLR_LA_MASK);

  gpdmach_putreg(chan, CH_CXSAR_OFFSET,   lli[0].sar);
  gpdmach_putreg(chan, CH_CXDAR_OFFSET,   lli[0].dar);
  gpdmach_putreg(chan, CH_CXTR1_OFFSET,   lli[0].tr1);
  gpdmach_putreg(chan, CH_CXTR2_OFFSET,   lli[0].tr2);
  gpdmach_putreg(chan, CH_CXBR1_OFFSET,   lli[0].br1);
  gpdmach_putreg(chan, CH_CXLBAR_OFFSET,  (uint32_t)&lli[0]);
  gpdmach_putreg(chan, CH_CXLLR_OFFSET,   lli[0].llr);

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
  struct gpdma_ch_s *chan;
  int i;

  /* Initialize each DMA stream */

  for (i = 0; i < sizeof(g_chan) / sizeof(struct gpdma_ch_s); i++)
    {
      chan = &g_chan[i];

      /* Attach DMA interrupt vectors */

      irq_attach(chan->irq, gpdma_dmainterrupt, chan);

      /* Disable the DMA channel */

      gpdma_ch_disable(chan);

      /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

      up_enable_irq(chan->irq);
    }
}

/****************************************************************************
 * Name: stm32_dmachannel
 *
 ****************************************************************************/

DMA_HANDLE stm32_dmachannel(enum gpdma_ttype_e type)
{
  /* On the H5, peripherals can kind of be mapped to any number of different
   * channels.
   *
   * P2M and M2P should be on channels with 8 byte FIFOs. GPDMA1/2 CH[0-3]
   *
   * Linear M2M should be on channels with 32 byte FIFOs. GPDMA1/2 CH[4-5]
   * Some peripherals (e.g. OCTOSPI) can also use these for burst transfers.
   *
   * 2D addressed transfers should be on GPDMA1/2 CH[6-7]
   *
   * Note: Currently, only P2M and M2P modes are supported and implemented.
   */

  DMA_HANDLE handle = NULL;
  irqstate_t flags;
  int i;

  /* Currently no support for M2M or 2D addressing modes.
   * TODO: Remove when support is added!
   */

  DEBUGASSERT(type != GPDMA_TTYPE_M2M_LINEAR);
  DEBUGASSERT(type != GPDMA_TTYPE_2D);

  flags = enter_critical_section();

  if (type == GPDMA_TTYPE_M2P || type == GPDMA_TTYPE_P2M)
    {
      for (i = 0; i < (sizeof(g_chan) / sizeof(struct gpdma_ch_s)); i++)
        {
          struct gpdma_ch_s *chan = &g_chan[i];

          if (chan->free && chan->channel <= 3)
            {
              chan->free = false;
              chan->type = type;
              handle = (DMA_HANDLE)chan;
              break;
            }
        }
    }

  leave_critical_section(flags);

  if (handle == NULL)
    {
      /* Failed to allocate channel. */

      dmainfo("No available DMA chan for transfer type=%" PRIu8 "\n",
              type);
    }

  return handle;
}

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel and unmap DMAMUX if required.
 *
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *   until stm32_dmachannel() is called again to re-gain access to the
 *   channel.
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
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;

  DEBUGASSERT(handle != NULL);

  chan->free = true;
}

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 * Input Parameters:
 *   TODO: Figure out what the input parameter needs to be!!! H7 and F7 have
 *         very different implementations.
 *
 ****************************************************************************/

void stm32_dmasetup(DMA_HANDLE handle, struct stm32_gpdma_cfg_s *cfg)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;

  DEBUGASSERT(handle != NULL);

  /* Store the configuration so it can be referenced later by start. */

  chan->cfg = *cfg;

  /* Drivers using DMA should manage channel usage. If a DMA request is not
   * made on an error or an abort occurs, the driver should stop the DMA. If
   * it fails to do so, we can not just hang waiting on the HW that will not
   * change state.
   *
   * If at the end of waiting the HW is still not ready, there is a HW
   * or software usage problem.
   */

  gpdma_ch_disable(chan);

  /*  if ((gpdmach_getreg(chan, CH_CXCR_OFFSET) & GPDMA_CXCR_EN) != 0)
   *    {
   *      gpdma_ch_abort(chan);
   *    }
   */

  /* Clear any unhandled flags from previous transactions */

  /* gpdmach_putreg(chan, CH_CXFCR_OFFSET, 0x7f << 8); */

  if (cfg->mode & GPDMACFG_MODE_CIRC)
    {
      gpdma_setup_circular(chan, cfg);
    }
  else
    {
      gpdma_setup(chan, cfg);
    }
}

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA transfer.
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;
  uint32_t cr;

  DEBUGASSERT(handle != NULL);

  /* Save the callback info. This will be invoked when the DMA completes */

  chan->callback = callback;
  chan->arg = arg;

  /* Activate channel by setting ENABLE bin in the GPDMA_CXCR register.
   * As soon as the channel is enabled, it can serve any DMA request from the
   * peripheral connected to the stream.
   */

  cr = gpdmach_getreg(chan, CH_CXCR_OFFSET);
  cr |= GPDMA_CXCR_EN;

  /* In normal mode, interrupt at either half or full completion. In circular
   * mode, always interrupt on wrap and optionally interrupt at halfway.
   */

  if (chan->cfg.mode & (GPDMACFG_MODE_CIRC))
    {
      /* In circular mode, when transfer completes it resets and starts
       * again. The transfer-complete interrupt is thus always enabled, and
       * the half-complete interrupt can be used to determine when the buffer
       * is half-full.
       */

      cr |= ((half ? GPDMA_CXCR_HTIE : 0) |
                          GPDMA_CXCR_TCIE |
                          GPDMA_CXCR_DTEIE);
    }
  else
    {
      /* Once half of the bytes are transferred, the half-transfer flag
       * (HTIF) is set and an interrupt is generated if the
       * Half-Transfer Interrupt Enable bit (HTIE) is set. At the end of the
       * transfer, the Transfer Complete Flag (TCIF) is set and an interrupt
       * is generated if the Transfer Complete Interrupt Enable bit (TCIE) is
       * set.
       */

      cr |= (half ? (GPDMA_CXCR_HTIE | GPDMA_CXCR_DTEIE) :
                    (GPDMA_CXCR_TCIE | GPDMA_CXCR_DTEIE));
    }

  gpdmach_putreg(chan, CH_CXCR_OFFSET, cr);
}

/****************************************************************************
 * Name: stm32_dmastop
 *
 * Description:
 *   Cancel the DMA. After stm32_dmastop() is called, the DMA channel is
 *   reset and stm32_dmasetup() must be called before stm32_dmastart() can be
 *   called again.
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

void stm32_dmastop(DMA_HANDLE handle)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;
  gpdma_ch_disable(chan);

  /* gpdma_ch_abort(chan); */
}

#ifdef CONFIG_STM32H5_DMACAPABLE
/****************************************************************************
 * Name: stm32_dmacapable
 *
 * Description:
 *   Check if the DMA controller can transfer data to/from given memory
 *   address. This depends on the internal connections in the ARM bus matrix
 *   of the processor. Note that this only applies to memory addresses, it
 *   will return false for any peripheral address.
 *
 * Input Parameters:
 *   cfg - DMA transfer configuration
 *
 * Returned Value:
 *   True, if transfer is possible.
 *
 * NOTE: No implementation yet.
 *
 ****************************************************************************/

bool stm32_dmacapable(DMA_HANDLE handle, stm32_gpdma_cfg_s *cfg)
{
# warning "stm32_dma_capable not implemented yet"
  return false;
}
#endif

#ifdef CONFIG_DEBUG_DMA_INFO
/****************************************************************************
 * Name: stm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 * NOTE: No implementation yet.
 *
 ****************************************************************************/

static void stm32_dmadump(DMA_HANDLE handle, const char *msg)
{
#  warning "stm32_dma_dump not implemented yet!"
}
#endif

#ifdef CONFIG_DEBUG_DMA_INFO
/****************************************************************************
 * Name: stm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Input Parameters:
 *   TODO: Figure these out!! Not sure if I need the struct like F7 or not?
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

void stm32_dmasample(DMA_HANDLE handle, struct stm32_dmaregs_s *regs)
{
  #warning "stm32_dmasample() not implemented yet!"
}
#endif