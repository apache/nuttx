/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_dma.c
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
#include <debug.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "sched/sched.h"
#include "stm32wb_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32WB_DMAMUX
#  error "Configuration error, CONFIG_STM32WB_DMAMUX not defined!"
#endif

#define DMAMUX_NUM      1
#define DMA_CONTROLLERS 2

#ifdef CONFIG_STM32WB_DMA1
#  define DMA1_NCHAN    7
#else
#  define DMA1_NCHAN    0
#endif
#ifdef CONFIG_STM32WB_DMA2
#  define DMA2_NCHAN    7
#else
#  define DMA2_NCHAN    0
#endif

#define DMA1_FIRST       (0)
#define DMA1_LAST        (DMA1_FIRST+DMA1_NCHAN)
#define DMA2_FIRST       (DMA1_LAST)
#define DMA2_LAST        (DMA2_FIRST+DMA2_NCHAN)

/* All available DMA channels */

#define DMA_NCHANNELS    (DMA1_NCHAN+DMA2_NCHAN)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure described one DMAMUX device */

struct stm32wb_dmamux_s
{
  uint8_t  id;                  /* DMAMUX id */
  uint8_t  nchan;               /* DMAMUX channels */
  uint32_t base;                /* DMAMUX base address */
};

typedef const struct stm32wb_dmamux_s *DMA_MUX;

/* This structure describes one DMA controller */

struct stm32wb_dma_s
{
  uint8_t       first;           /* Offset in stm32wb_dmach_s array */
  uint8_t       nchan;           /* Number of channels */
  uint8_t       dmamux_offset;   /* DMAMUX channel offset */
  uint32_t      base;            /* Base address */
  DMA_MUX       dmamux;          /* DMAMUX associated with controller */
};

/* This structure describes one DMA channel (DMA1, DMA2) */

struct stm32wb_dmach_s
{
  bool             used;         /* Channel in use */
  uint8_t          dmamux_req;   /* Configured DMAMUX input request */
  uint8_t          ctrl;         /* DMA controller */
  uint8_t          chan;         /* DMA channel channel id */
  uint8_t          irq;          /* DMA channel IRQ number */
  uint8_t          shift;        /* IFCR bit shift value */
  uint32_t         base;         /* DMA register channel base address */
  dma_callback_t   callback;     /* Callback invoked when the DMA completes */
  void             *arg;         /* Argument passed to callback function */
};

typedef struct stm32wb_dmach_s *DMA_CHANNEL;

/* DMA operations */

struct stm32wb_dma_ops_s
{
  /* Disable the DMA transfer */

  void (*dma_disable)(DMA_CHANNEL dmachan);

  /* DMA interrupt */

  int (*dma_interrupt)(int irq, void *context, void *arg);

  /* Setup the DMA */

  void (*dma_setup)(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                    size_t ntransfers, uint32_t ccr);

  /* Start the DMA */

  void (*dma_start)(DMA_HANDLE handle, dma_callback_t callback,
                    void *arg, bool half);

  /* Read remaining DMA bytes */

  size_t (*dma_residual)(DMA_HANDLE handle);

  /* Check the DMA configuration  */

  bool (*dma_capable)(uint32_t maddr, uint32_t count, uint32_t ccr);

#ifdef CONFIG_DEBUG_DMA_INFO
  /* Sample the DMA registers */

  void (*dma_sample)(DMA_HANDLE handle, struct stm32wb_dmaregs_s *regs);

  /* Dump the DMA registers */

  void (*dma_dump)(DMA_HANDLE handle,
                   const struct stm32wb_dmaregs_s *regs,
                   const char *msg);
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_STM32WB_DMA1) || defined(CONFIG_STM32WB_DMA2)
static void stm32wb_dma12_disable(DMA_CHANNEL dmachan);
static int stm32wb_dma12_interrupt(int irq, void *context, void *arg);
static void stm32wb_dma12_setup(DMA_HANDLE handle, uint32_t paddr,
                                uint32_t maddr, size_t ntransfers,
                                uint32_t ccr);
static void stm32wb_dma12_start(DMA_HANDLE handle, dma_callback_t callback,
                                void *arg, bool half);
static size_t stm32wb_dma12_residual(DMA_HANDLE handle);
#ifdef CONFIG_DEBUG_DMA_INFO
static void stm32wb_dma12_sample(DMA_HANDLE handle,
                                 struct stm32wb_dmaregs_s *regs);
static void stm32wb_dma12_dump(DMA_HANDLE handle,
                               const struct stm32wb_dmaregs_s *regs,
                               const char *msg);
#endif
#endif

static uint32_t dmachan_getbase(DMA_CHANNEL dmachan);
static uint32_t dmabase_getreg(DMA_CHANNEL dmachan, uint32_t offset);
static void dmabase_putreg(DMA_CHANNEL dmachan, uint32_t offset,
                           uint32_t value);
static uint32_t dmachan_getreg(DMA_CHANNEL dmachan, uint32_t offset);
static void dmachan_putreg(DMA_CHANNEL dmachan, uint32_t offset,
                           uint32_t value);
static void dmamux_putreg(DMA_MUX dmamux, uint32_t offset, uint32_t value);
#ifdef CONFIG_DEBUG_DMA_INFO
static uint32_t dmamux_getreg(DMA_MUX dmamux, uint32_t offset);
static void stm32wb_dmamux_sample(DMA_MUX dmamux, uint8_t chan,
                                  struct stm32wb_dmaregs_s *regs);
static void stm32wb_dmamux_dump(DMA_MUX dmamux, uint8_t channel,
                                const struct stm32wb_dmaregs_s *regs);
#endif
static DMA_CHANNEL stm32wb_dma_channel_get(uint8_t channel,
                                           uint8_t controller);
static void stm32wb_gdma_limits_get(uint8_t controller, uint8_t *first,
                                    uint8_t *last);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Operations specific to DMA controller */

static const struct stm32wb_dma_ops_s g_dma_ops[DMA_CONTROLLERS] =
{
#ifdef CONFIG_STM32WB_DMA1
  /* 0 - DMA1 */

    {
      .dma_disable   = stm32wb_dma12_disable,
      .dma_interrupt = stm32wb_dma12_interrupt,
      .dma_setup     = stm32wb_dma12_setup,
      .dma_start     = stm32wb_dma12_start,
      .dma_residual  = stm32wb_dma12_residual,
#ifdef CONFIG_DEBUG_DMA_INFO
      .dma_sample    = stm32wb_dma12_sample,
      .dma_dump      = stm32wb_dma12_dump,
#endif
    },
#else
    {
      NULL
    },
#endif

#ifdef CONFIG_STM32WB_DMA2
  /* 1 - DMA2 */

    {
      .dma_disable   = stm32wb_dma12_disable,
      .dma_interrupt = stm32wb_dma12_interrupt,
      .dma_setup     = stm32wb_dma12_setup,
      .dma_start     = stm32wb_dma12_start,
      .dma_residual  = stm32wb_dma12_residual,
#ifdef CONFIG_DEBUG_DMA_INFO
      .dma_sample    = stm32wb_dma12_sample,
      .dma_dump      = stm32wb_dma12_dump,
#endif
    }
#else
    {
      NULL
    }
#endif
};

/* This array describes the state of DMAMUX controller */

static const struct stm32wb_dmamux_s g_dmamux[DMAMUX_NUM] =
{
    {
      .id      = 1,
      .nchan   = 14,              /* 0-6 - DMA1, 7-13 - DMA2 */
      .base    = STM32WB_DMAMUX1_BASE
    }
};

/* This array describes the state of each controller */

static const struct stm32wb_dma_s g_dma[DMA_NCHANNELS] =
{
  /* 0 - DMA1 */

    {
      .base   = STM32WB_DMA1_BASE,
      .first  = DMA1_FIRST,
      .nchan  = DMA1_NCHAN,
      .dmamux = &g_dmamux[DMAMUX1], /* DMAMUX1 channels 0-6 */
      .dmamux_offset = 0
    },

  /* 1 - DMA2 */

    {
      .base   = STM32WB_DMA2_BASE,
      .first  = DMA2_FIRST,
      .nchan  = DMA2_NCHAN,
      .dmamux = &g_dmamux[DMAMUX1], /* DMAMUX1 channels 7-13 */
      .dmamux_offset = 7
    }
};

/* This array describes the state of each DMA channel. */

static struct stm32wb_dmach_s g_dmach[DMA_NCHANNELS] =
{
#ifdef CONFIG_STM32WB_DMA1
  /* DMA1 */

    {
      .ctrl     = DMA1,
      .chan     = 0,
      .irq      = STM32WB_IRQ_DMA1CH1,
      .shift    = DMA_CHAN_SHIFT(0),
      .base     = STM32WB_DMA1_BASE + STM32WB_DMACHAN_OFFSET(0),
    },

    {
      .ctrl     = DMA1,
      .chan     = 1,
      .irq      = STM32WB_IRQ_DMA1CH2,
      .shift    = DMA_CHAN_SHIFT(1),
      .base     = STM32WB_DMA1_BASE + STM32WB_DMACHAN_OFFSET(1),
    },

    {
      .ctrl     = DMA1,
      .chan     = 2,
      .irq      = STM32WB_IRQ_DMA1CH3,
      .shift    = DMA_CHAN_SHIFT(2),
      .base     = STM32WB_DMA1_BASE + STM32WB_DMACHAN_OFFSET(2),
    },

    {
      .ctrl     = DMA1,
      .chan     = 3,
      .irq      = STM32WB_IRQ_DMA1CH4,
      .shift    = DMA_CHAN_SHIFT(3),
      .base     = STM32WB_DMA1_BASE + STM32WB_DMACHAN_OFFSET(3),
    },

    {
      .ctrl     = DMA1,
      .chan     = 4,
      .irq      = STM32WB_IRQ_DMA1CH5,
      .shift    = DMA_CHAN_SHIFT(4),
      .base     = STM32WB_DMA1_BASE + STM32WB_DMACHAN_OFFSET(4),
    },

    {
      .ctrl     = DMA1,
      .chan     = 5,
      .irq      = STM32WB_IRQ_DMA1CH6,
      .shift    = DMA_CHAN_SHIFT(5),
      .base     = STM32WB_DMA1_BASE + STM32WB_DMACHAN_OFFSET(5),
    },

    {
      .ctrl     = DMA1,
      .chan     = 6,
      .irq      = STM32WB_IRQ_DMA1CH7,
      .shift    = DMA_CHAN_SHIFT(6),
      .base     = STM32WB_DMA1_BASE + STM32WB_DMACHAN_OFFSET(6),
    },
#endif

#ifdef CONFIG_STM32WB_DMA2
  /* DMA2 */

    {
      .ctrl     = DMA2,
      .chan     = 0,
      .irq      = STM32WB_IRQ_DMA2CH1,
      .shift    = DMA_CHAN_SHIFT(0),
      .base     = STM32WB_DMA2_BASE + STM32WB_DMACHAN_OFFSET(0),
    },

    {
      .ctrl     = DMA2,
      .chan     = 1,
      .irq      = STM32WB_IRQ_DMA2CH2,
      .shift    = DMA_CHAN_SHIFT(1),
      .base     = STM32WB_DMA2_BASE + STM32WB_DMACHAN_OFFSET(1),
    },

    {
      .ctrl     = DMA2,
      .chan     = 2,
      .irq      = STM32WB_IRQ_DMA2CH3,
      .shift    = DMA_CHAN_SHIFT(2),
      .base     = STM32WB_DMA2_BASE + STM32WB_DMACHAN_OFFSET(2),
    },

    {
      .ctrl     = DMA2,
      .chan     = 3,
      .irq      = STM32WB_IRQ_DMA2CH4,
      .shift    = DMA_CHAN_SHIFT(3),
      .base     = STM32WB_DMA2_BASE + STM32WB_DMACHAN_OFFSET(3),
    },

    {
      .ctrl     = DMA2,
      .chan     = 4,
      .irq      = STM32WB_IRQ_DMA2CH5,
      .shift    = DMA_CHAN_SHIFT(4),
      .base     = STM32WB_DMA2_BASE + STM32WB_DMACHAN_OFFSET(4),
    },

    {
      .ctrl     = DMA2,
      .chan     = 5,
      .irq      = STM32WB_IRQ_DMA2CH6,
      .shift    = DMA_CHAN_SHIFT(5),
      .base     = STM32WB_DMA2_BASE + STM32WB_DMACHAN_OFFSET(5),
    },

    {
      .ctrl     = DMA2,
      .chan     = 6,
      .irq      = STM32WB_IRQ_DMA2CH7,
      .shift    = DMA_CHAN_SHIFT(6),
      .base     = STM32WB_DMA2_BASE + STM32WB_DMACHAN_OFFSET(6),
    },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * DMA register access functions
 ****************************************************************************/

/****************************************************************************
 * Name: dmachan_getbase
 *
 * Description:
 *  Get base DMA address for dmachan
 *
 ****************************************************************************/

static uint32_t dmachan_getbase(DMA_CHANNEL dmachan)
{
  uint8_t controller = dmachan->ctrl;

  return g_dma[controller].base;
}

/****************************************************************************
 * Name: dmabase_getreg
 *
 * Description:
 *  Get non-channel register from DMA controller
 *
 ****************************************************************************/

static uint32_t dmabase_getreg(DMA_CHANNEL dmachan, uint32_t offset)
{
  uint32_t dmabase = dmachan_getbase(dmachan);

  return getreg32(dmabase + offset);
}

/****************************************************************************
 * Name: dmabase_putreg
 *
 * Description:
 *  Write to non-channel register in DMA controller
 *
 ****************************************************************************/

static void dmabase_putreg(DMA_CHANNEL dmachan, uint32_t offset,
                           uint32_t value)
{
  uint32_t dmabase = dmachan_getbase(dmachan);

  putreg32(value, dmabase + offset);
}

/****************************************************************************
 * Name: dmachan_getreg
 *
 * Description:
 *  Get channel register.
 *
 ****************************************************************************/

static uint32_t dmachan_getreg(DMA_CHANNEL dmachan, uint32_t offset)
{
  return getreg32(dmachan->base + offset);
}

/****************************************************************************
 * Name: dmachan_putreg
 *
 * Description:
 *  Write to channel register.
 *
 ****************************************************************************/

static void dmachan_putreg(DMA_CHANNEL dmachan, uint32_t offset,
                           uint32_t value)
{
  putreg32(value, dmachan->base + offset);
}

/****************************************************************************
 * Name: dmamux_getreg
 *
 * Description:
 *  Write to DMAMUX
 *
 ****************************************************************************/

static void dmamux_putreg(DMA_MUX dmamux, uint32_t offset, uint32_t value)
{
  putreg32(value, dmamux->base + offset);
}

/****************************************************************************
 * Name: dmamux_getreg
 *
 * Description:
 *  Get DMAMUX register.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
static uint32_t dmamux_getreg(DMA_MUX dmamux, uint32_t offset)
{
  return getreg32(dmamux->base + offset);
}
#endif

/****************************************************************************
 * Name: stm32wb_dma_channel_get
 *
 * Description:
 *  Get the g_dmach table entry associated with a given DMA controller
 *  and channel number.
 *
 ****************************************************************************/

static DMA_CHANNEL stm32wb_dma_channel_get(uint8_t channel,
                                           uint8_t controller)
{
  uint8_t first = 0;
  uint8_t nchan = 0;

  /* Get limits for g_dma array */

  stm32wb_gdma_limits_get(controller, &first, &nchan);

  DEBUGASSERT(channel <= nchan);

  return &g_dmach[first + channel];
}

/****************************************************************************
 * Name: stm32wb_gdma_limits_get
 *
 * Description:
 *  Get g_dma array limits for a given DMA controller.
 *
 ****************************************************************************/

static void stm32wb_gdma_limits_get(uint8_t controller, uint8_t *first,
                                    uint8_t *nchan)
{
  DEBUGASSERT(first != NULL);
  DEBUGASSERT(nchan != NULL);

  DEBUGASSERT(controller >= DMA1 && controller <= DMA2);

  *first = g_dma[controller].first;
  *nchan = g_dma[controller].nchan;
}

/****************************************************************************
 * DMA controller functions
 ****************************************************************************/

#if defined(CONFIG_STM32WB_DMA1) || defined(CONFIG_STM32WB_DMA2)

/****************************************************************************
 * Name: stm32wb_dma12_disable
 *
 * Description:
 *  Disable DMA channel (DMA1/DMA2)
 *
 ****************************************************************************/

static void stm32wb_dma12_disable(DMA_CHANNEL dmachan)
{
  uint32_t regval;

  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  /* Disable all interrupts at the DMA controller */

  regval = dmachan_getreg(dmachan, STM32WB_DMACHAN_CCR_OFFSET);
  regval &= ~DMA_CCR_ALLINTS;

  /* Disable the DMA channel */

  regval &= ~DMA_CCR_EN;
  dmachan_putreg(dmachan, STM32WB_DMACHAN_CCR_OFFSET, regval);

  /* Clear pending channel interrupts */

  dmabase_putreg(dmachan, STM32WB_DMA_IFCR_OFFSET,
                 DMA_ISR_CHAN_MASK(dmachan->chan));
}

/****************************************************************************
 * Name: stm32wb_dma12_interrupt
 *
 * Description:
 *  DMA channel interrupt handler
 *
 ****************************************************************************/

static int stm32wb_dma12_interrupt(int irq, void *context, void *arg)
{
  DMA_CHANNEL dmachan;
  uint32_t isr;
  uint8_t channel;
  uint8_t controller;

  /* Get the channel and the controller that generated the interrupt */

  if (0)
    {
    }
#ifdef CONFIG_STM32WB_DMA1
  else if (irq >= STM32WB_IRQ_DMA1CH1 && irq <= STM32WB_IRQ_DMA1CH7)
    {
      channel = irq - STM32WB_IRQ_DMA1CH1;
      controller = DMA1;
    }
#endif
#ifdef CONFIG_STM32WB_DMA2
  else if (irq >= STM32WB_IRQ_DMA2CH1 && irq <= STM32WB_IRQ_DMA2CH5)
    {
      channel = irq - STM32WB_IRQ_DMA2CH1;
      controller = DMA2;
    }
  else if (irq >= STM32WB_IRQ_DMA2CH6 && irq <= STM32WB_IRQ_DMA2CH7)
    {
      channel = irq - STM32WB_IRQ_DMA2CH6 + (6 - 1);
      controller = DMA2;
    }
#endif
  else
    {
      DEBUGPANIC();
      return OK;
    }

  /* Get the channel structure from the stream and controller numbers */

  dmachan = stm32wb_dma_channel_get(channel, controller);

  /* Get the interrupt status (for this channel only) */

  isr = dmabase_getreg(dmachan, STM32WB_DMA_ISR_OFFSET) &
        DMA_ISR_CHAN_MASK(dmachan->chan);

  /* Invoke the callback */

  if (dmachan->callback)
    {
      dmachan->callback(dmachan, isr >> DMA_ISR_CHAN_SHIFT(dmachan->chan),
                        dmachan->arg);
    }

  /* Clear the interrupts we are handling */

  dmabase_putreg(dmachan, STM32WB_DMA_IFCR_OFFSET, isr);

  return OK;
}

/****************************************************************************
 * Name: stm32wb_dma12_setup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

static void stm32wb_dma12_setup(DMA_HANDLE handle, uint32_t paddr,
                                uint32_t maddr, size_t ntransfers,
                                uint32_t ccr)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint32_t regval;

  DEBUGASSERT(handle != NULL);
  DEBUGASSERT(ntransfers < 65536);

  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  dmainfo("paddr: %08" PRIx32 " maddr: %08" PRIx32
          " ntransfers: %zd ccr: %08" PRIx32 "\n",
          paddr, maddr, ntransfers, ccr);

#ifdef CONFIG_STM32WB_DMACAPABLE
  DEBUGASSERT(g_dma_ops[dmachan->ctrl].dma_capable(maddr, ntransfers, ccr));
#endif

  /* Then DMA_CNDTRx register can only be modified if the DMA channel is
   * disabled.
   */

  regval  = dmachan_getreg(dmachan, STM32WB_DMACHAN_CCR_OFFSET);
  regval &= ~(DMA_CCR_EN);
  dmachan_putreg(dmachan, STM32WB_DMACHAN_CCR_OFFSET, regval);

  /* Set the peripheral register address in the DMA_CPARx register. The data
   * will be moved from/to this address to/from the memory after the
   * peripheral event.
   */

  dmachan_putreg(dmachan, STM32WB_DMACHAN_CPAR_OFFSET, paddr);

  /* Set the memory address in the DMA_CMARx register. The data will be
   * written to or read from this memory after the peripheral event.
   */

  dmachan_putreg(dmachan, STM32WB_DMACHAN_CMAR_OFFSET, maddr);

  /* Configure the total number of data to be transferred in the DMA_CNDTRx
   * register.  After each peripheral event, this value will be decremented.
   */

  dmachan_putreg(dmachan, STM32WB_DMACHAN_CNDTR_OFFSET, ntransfers);

  /* Configure the channel priority using the PL[1:0] bits in the DMA_CCRx
   * register.  Configure data transfer direction, circular mode, peripheral
   * & memory incremented mode, peripheral & memory data size, and interrupt
   * after half and/or full transfer in the DMA_CCRx register.
   */

  regval  = dmachan_getreg(dmachan, STM32WB_DMACHAN_CCR_OFFSET);
  regval &= ~(DMA_CCR_MEM2MEM | DMA_CCR_PL_MASK | DMA_CCR_MSIZE_MASK |
              DMA_CCR_PSIZE_MASK | DMA_CCR_MINC | DMA_CCR_PINC |
              DMA_CCR_CIRC | DMA_CCR_DIR);
  ccr    &=  (DMA_CCR_MEM2MEM | DMA_CCR_PL_MASK | DMA_CCR_MSIZE_MASK |
              DMA_CCR_PSIZE_MASK | DMA_CCR_MINC | DMA_CCR_PINC |
              DMA_CCR_CIRC | DMA_CCR_DIR);
  regval |= ccr;
  dmachan_putreg(dmachan, STM32WB_DMACHAN_CCR_OFFSET, regval);
}

/****************************************************************************
 * Name: stm32wb_dma12_start
 *
 * Description:
 *   Start the standard DMA transfer
 ****************************************************************************/

static void stm32wb_dma12_start(DMA_HANDLE handle, dma_callback_t callback,
                                void *arg, bool half)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint32_t ccr;

  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  /* Save the callback info.  This will be invoked when the DMA completes */

  dmachan->callback = callback;
  dmachan->arg      = arg;

  /* Activate the channel by setting the ENABLE bit in the DMA_CCRx register.
   * As soon as the channel is enabled, it can serve any DMA request from the
   * peripheral connected on the channel.
   */

  ccr  = dmachan_getreg(dmachan, STM32WB_DMACHAN_CCR_OFFSET);
  ccr |= DMA_CCR_EN;

  /* In normal mode, interrupt at either half or full completion. In circular
   * mode, always interrupt on buffer wrap, and optionally interrupt at the
   * halfway point.
   */

  if ((ccr & DMA_CCR_CIRC) == 0)
    {
      /* Once half of the bytes are transferred, the half-transfer flag
       * (HTIF) is set and an interrupt is generated if the Half-Transfer
       * Interrupt Enable bit (HTIE) is set. At the end of the transfer,
       * the Transfer Complete Flag (TCIF) is set and an interrupt is
       * generated if the Transfer Complete Interrupt Enable bit (TCIE)
       * is set.
       */

      ccr |= (half ? (DMA_CCR_HTIE | DMA_CCR_TEIE) :
                     (DMA_CCR_TCIE | DMA_CCR_TEIE));
    }
  else
    {
      /* In nonstop mode, when the transfer completes it immediately resets
       * and starts again.  The transfer-complete interrupt is thus always
       * enabled, and the half-complete interrupt can be used in circular
       * mode to determine when the buffer is half-full, or in
       * double-buffered mode to determine when one of the two buffers
       * is full.
       */

      ccr |= (half ? DMA_CCR_HTIE : 0) | DMA_CCR_TCIE | DMA_CCR_TEIE;
    }

  dmachan_putreg(dmachan, STM32WB_DMACHAN_CCR_OFFSET, ccr);
}

/****************************************************************************
 * Name: stm32wb_dma12_residual
 ****************************************************************************/

static size_t stm32wb_dma12_residual(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;

  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  return dmachan_getreg(dmachan, STM32WB_DMACHAN_CNDTR_OFFSET);
}

/****************************************************************************
 * Name: stm32wb_dma12_sample
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32wb_dma12_sample(DMA_HANDLE handle, struct stm32wb_dmaregs_s *regs)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  irqstate_t flags;

  flags = enter_critical_section();

  regs->isr = dmabase_getreg(dmachan, STM32WB_DMA_ISR_OFFSET);
  regs->ccr = dmachan_getreg(dmachan, STM32WB_DMACHAN_CCR_OFFSET);
  regs->cndtr = dmachan_getreg(dmachan, STM32WB_DMACHAN_CNDTR_OFFSET);
  regs->cpar = dmachan_getreg(dmachan, STM32WB_DMACHAN_CPAR_OFFSET);
  regs->cmar = dmachan_getreg(dmachan, STM32WB_DMACHAN_CMAR_OFFSET);

  stm32wb_dmamux_sample(g_dma[dmachan->ctrl].dmamux,
                        dmachan->chan + g_dma[dmachan->ctrl].dmamux_offset,
                        regs);

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: stm32wb_dma12_dump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
static void stm32wb_dma12_dump(DMA_HANDLE handle,
                               const struct stm32wb_dmaregs_s *regs,
                               const char *msg)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;

  DEBUGASSERT(dmachan->ctrl == DMA1 || dmachan->ctrl == DMA2);

  uint32_t dmabase = dmachan_getbase(dmachan);

  dmainfo("DMA%d Registers: %s\n",
          dmachan->ctrl + 1,
          msg);
  dmainfo("    ISR[%08x]: %08x\n",
          dmabase + STM32WB_DMA_ISR_OFFSET,
          regs->isr);
  dmainfo("    CCR[%08x]: %08x\n",
          dmachan->base + STM32WB_DMACHAN_CCR_OFFSET,
          regs->ccr);
  dmainfo("  CNDTR[%08x]: %08x\n",
          dmachan->base + STM32WB_DMACHAN_CNDTR_OFFSET,
          regs->cndtr);
  dmainfo("   CPAR[%08x]: %08x\n",
          dmachan->base + STM32WB_DMACHAN_CPAR_OFFSET,
          regs->cpar);
  dmainfo("   CMAR[%08x]: %08x\n",
          dmachan->base + STM32WB_DMACHAN_CMAR_OFFSET,
          regs->cmar);

  stm32wb_dmamux_dump(g_dma[dmachan->ctrl].dmamux,
                      dmachan->chan + g_dma[dmachan->ctrl].dmamux_offset,
                      regs);
}
#endif

#endif /* CONFIG_STM32WB_DMA1 || CONFIG_STM32WB_DMA2 */

/****************************************************************************
 * Name: stm32wb_dmamux_sample
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
static void stm32wb_dmamux_sample(DMA_MUX dmamux, uint8_t chan,
                                  struct stm32wb_dmaregs_s *regs)
{
  regs->dmamux.ccr = dmamux_getreg(dmamux, STM32WB_DMAMUX_CXCR_OFFSET(chan));
  regs->dmamux.csr = dmamux_getreg(dmamux, STM32WB_DMAMUX_CSR_OFFSET);
  regs->dmamux.rg0cr = dmamux_getreg(dmamux, STM32WB_DMAMUX_RG0CR_OFFSET);
  regs->dmamux.rg1cr = dmamux_getreg(dmamux, STM32WB_DMAMUX_RG1CR_OFFSET);
  regs->dmamux.rg2cr = dmamux_getreg(dmamux, STM32WB_DMAMUX_RG2CR_OFFSET);
  regs->dmamux.rg3cr = dmamux_getreg(dmamux, STM32WB_DMAMUX_RG3CR_OFFSET);
  regs->dmamux.rgsr = dmamux_getreg(dmamux, STM32WB_DMAMUX_RGSR_OFFSET);
}
#endif

/****************************************************************************
 * Name: stm32wb_dmamux_dump
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
static void stm32wb_dmamux_dump(DMA_MUX dmamux, uint8_t channel,
                                const struct stm32wb_dmaregs_s *regs)
{
  dmainfo("DMAMUX%d CH=%d\n", dmamux->id, channel);
  dmainfo("    CCR[%08x]: %08x\n",
          dmamux->base + STM32WB_DMAMUX_CXCR_OFFSET(channel),
          regs->dmamux.ccr);
  dmainfo("    CSR[%08x]: %08x\n",
          dmamux->base + STM32WB_DMAMUX_CSR_OFFSET, regs->dmamux.csr);
  dmainfo("  RG0CR[%08x]: %08x\n",
          dmamux->base + STM32WB_DMAMUX_RG0CR_OFFSET, regs->dmamux.rg0cr);
  dmainfo("  RG1CR[%08x]: %08x\n",
          dmamux->base + STM32WB_DMAMUX_RG1CR_OFFSET, regs->dmamux.rg1cr);
  dmainfo("  RG2CR[%08x]: %08x\n",
          dmamux->base + STM32WB_DMAMUX_RG2CR_OFFSET, regs->dmamux.rg2cr);
  dmainfo("  RG3CR[%08x]: %08x\n",
          dmamux->base + STM32WB_DMAMUX_RG3CR_OFFSET, regs->dmamux.rg3cr);
  dmainfo("   RGSR[%08x]: %08x\n",
          dmamux->base + STM32WB_DMAMUX_RGSR_OFFSET, regs->dmamux.rgsr);
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dma_initialize
 *
 * Description:
 *   Initialize the DMA subsystem (DMA1, DMA2)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function arm_dma_initialize(void)
{
  DMA_CHANNEL dmachan;
  uint8_t controller;
  int channel;

  dmainfo("Initialize DMA\n");

  /* Initialize DMA channels */

  for (channel = 0; channel < DMA_NCHANNELS; channel++)
    {
      dmachan = &g_dmach[channel];

      /* Initialize flag */

      dmachan->used = false;

      /* Get DMA controller associated with channel */

      controller = dmachan->ctrl;

      DEBUGASSERT(controller >= DMA1 && controller <= DMA2);

      /* Attach standard DMA interrupt vectors */

      irq_attach(dmachan->irq, g_dma_ops[controller].dma_interrupt,
                 dmachan);

      /* Disable the DMA channel */

      g_dma_ops[controller].dma_disable(dmachan);

      /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

      up_enable_irq(dmachan->irq);
    }
}

/****************************************************************************
 * Name: stm32wb_dmachannel
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'dmamap' argument.
 *   It is common for both DMA controllers (DMA1 and DMA2).
 *
 * Input Parameters:
 *   dmamap - Identifies the stream/channel resource. For the STM32WB, this
 *     is a bit-encoded value as provided by the DMAMAP_* definitions
 *     in hardware/stm32wb_dmamux.h
 *
 * Returned Value:
 *   One success, this function returns a non-NULL, void* DMA channel
 *   handle.  NULL is returned on any failure.  This function can fail only
 *   if no DMA channel is available.
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE stm32wb_dmachannel(unsigned int dmamap)
{
  DMA_CHANNEL dmachan;
  uint8_t dmamux_req;
  irqstate_t flags;
  uint8_t controller;
  uint8_t first = 0;
  uint8_t nchan = 0;
  int item = -1;
  int i;

  /* Get DMA controller from encoded DMAMAP value */

  controller = DMAMAP_CONTROLLER(dmamap);
  DEBUGASSERT(controller >= DMA1 && controller <= DMA2);

  /* Get DMAMUX channel from encoded DMAMAP value */

  dmamux_req = DMAMAP_REQUEST(dmamap);

  /* Get g_dma array limits for given controller */

  stm32wb_gdma_limits_get(controller, &first, &nchan);

  /* Find available channel for given controller */

  flags = enter_critical_section();
  for (i = first; i < first + nchan; i++)
    {
      if (!g_dmach[i].used)
        {
          item = i;
          g_dmach[i].used = true;
          g_dmach[i].dmamux_req = dmamux_req;
          break;
        }
    }

  leave_critical_section(flags);

  dmainfo("ctrl=%d item=%d\n", controller, item);

  if (item == -1)
    {
      dmainfo("No available DMA chan for CTRL=%d\n",
              controller);

      /* No available channel */

      return NULL;
    }

  /* Assign DMA item */

  dmachan = &g_dmach[item];

  dmainfo("Get g_dmach[%d] CTRL=%d CH=%d\n", i, controller, dmachan->chan);

  /* Be sure that we have proper DMA controller */

  DEBUGASSERT(dmachan->ctrl == controller);

  return (DMA_HANDLE)dmachan;
}

/****************************************************************************
 * Name: stm32wb_dmafree
 *
 * Description:
 *   Release a DMA channel and unmap DMAMUX if required.
 *
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *   until stm32wb_dmachannel() is called again to re-gain access to the
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

void stm32wb_dmafree(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint8_t controller;
  irqstate_t flags;

  DEBUGASSERT(handle != NULL);

  /* Get DMA controller */

  controller = dmachan->ctrl;
  DEBUGASSERT(controller >= DMA1 && controller <= DMA2);

  dmainfo("Free g_dmach[%d] CTRL=%d CH=%d\n", dmachan - g_dmach, controller,
          dmachan->chan);
  UNUSED(controller);

  /* Release the channel */

  flags = enter_critical_section();
  dmachan->used = false;
  dmachan->dmamux_req = 0;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: stm32wb_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void stm32wb_dmasetup(DMA_HANDLE handle, uint32_t paddr, uint32_t maddr,
                      size_t ntransfers, uint32_t ccr)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint8_t controller;

  DEBUGASSERT(handle != NULL);

  /* Get DMA controller */

  controller = dmachan->ctrl;
  DEBUGASSERT(controller >= DMA1 && controller <= DMA2);

  g_dma_ops[controller].dma_setup(handle, paddr, maddr, ntransfers, ccr);
}

/****************************************************************************
 * Name: stm32wb_dmastart
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by stm32wb_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void stm32wb_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                      bool half)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  DMA_MUX dmamux;
  uint32_t regval;
  uint8_t dmamux_chan;
  uint8_t controller;

  DEBUGASSERT(handle != NULL);

  /* Get DMA controller */

  controller = dmachan->ctrl;
  DEBUGASSERT(controller >= DMA1 && controller <= DMA2);

  /* Recommended channel configure procedure in reference manual:
   *  1. Set and configure the DMA channel y, except enabling the channel y.
   *  2. Set and configure the related DMAMUX y channel.
   *  3. Last, activate the DMA channel y.
   */

  /* Get DMAMUX associated with DMA controller */

  dmamux = g_dma[controller].dmamux;
  dmamux_chan = dmachan->chan + g_dma[controller].dmamux_offset;

  /* DMAMUX Set DMA channel source */

  regval = dmachan->dmamux_req << DMAMUX_CCR_DMAREQID_SHIFT;
  dmamux_putreg(dmamux, STM32WB_DMAMUX_CXCR_OFFSET(dmamux_chan), regval);

  /* Enable DMA channel */

  g_dma_ops[controller].dma_start(handle, callback, arg, half);
}

/****************************************************************************
 * Name: stm32wb_dmastop
 *
 * Description:
 *   Cancel the DMA.  After stm32wb_dmastop() is called, the DMA channel is
 *   reset and stm32wb_dmasetup() must be called before stm32wb_dmastart()
 *   can be called again
 *
 * Assumptions:
 *   - DMA handle allocated by stm32wb_dmachannel()
 *
 ****************************************************************************/

void stm32wb_dmastop(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  DMA_MUX dmamux;
  uint8_t dmamux_chan;
  uint8_t controller;

  DEBUGASSERT(handle != NULL);

  /* Get DMA controller */

  controller = dmachan->ctrl;
  DEBUGASSERT(controller >= DMA1 && controller <= DMA2);

  /* Get DMAMUX associated with DMA controller */

  dmamux = g_dma[controller].dmamux;
  dmamux_chan = dmachan->chan + g_dma[controller].dmamux_offset;

  /* Disable DMA channel */

  g_dma_ops[controller].dma_disable(dmachan);

  /* DMAMUX Clear DMA channel source */

  dmamux_putreg(dmamux, STM32WB_DMAMUX_CXCR_OFFSET(dmamux_chan), 0);
}

/****************************************************************************
 * Name: stm32wb_dmaresidual
 *
 * Description:
 *   Read the DMA bytes-remaining register.
 *
 * Assumptions:
 *   - DMA handle allocated by stm32wb_dmachannel()
 *
 ****************************************************************************/

size_t stm32wb_dmaresidual(DMA_HANDLE handle)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint8_t controller;

  DEBUGASSERT(handle != NULL);

  /* Get DMA controller */

  controller = dmachan->ctrl;
  DEBUGASSERT(controller >= DMA1 && controller <= DMA2);

  return g_dma_ops[controller].dma_residual(handle);
}

/****************************************************************************
 * Name: stm32wb_dmacapable
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
 ****************************************************************************/

#ifdef CONFIG_STM32WB_DMACAPABLE
bool stm32wb_dmacapable(uint32_t maddr, uint32_t count, uint32_t ccr)
{
  unsigned int msize_shift;
  uint32_t transfer_size;
  uint32_t mend;

  /* Verify that the address conforms to the memory transfer size.
   * Transfers to/from memory performed by the DMA controller are
   * required to be aligned to their size.
   *
   * Datasheet 3.13 claims
   *   "Access to Flash, SRAM, APB and AHB peripherals as source
   *   and destination"
   */

  switch (ccr & DMA_CCR_MSIZE_MASK)
    {
      case DMA_CCR_MSIZE_8BITS:
        msize_shift = 0;
        break;

      case DMA_CCR_MSIZE_16BITS:
        msize_shift = 1;
        break;

      case DMA_CCR_MSIZE_32BITS:
        msize_shift = 2;
        break;

      default:
        return false;
    }

  transfer_size = 1 << msize_shift;

  if ((maddr & (transfer_size - 1)) != 0)
    {
      return false;
    }

  /* Verify that the transfer is to a memory region that supports DMA. */

  mend = maddr + (count << msize_shift) - 1;

  if ((maddr & STM32WB_REGION_MASK) != (mend & STM32WB_REGION_MASK))
    {
      return false;
    }

  switch (maddr & STM32WB_REGION_MASK)
    {
      case STM32WB_PERIPH_BASE:
      case STM32WB_FSMC_BASE:
      case STM32WB_FSMC_BANK1:
      case STM32WB_FSMC_BANK2:
      case STM32WB_FSMC_BANK3:
      case STM32WB_QSPI_BANK:
      case STM32WB_SRAM_BASE:
      case STM32WB_SRAM2_BASE:
      case STM32WB_SRAM3_BASE:
      case STM32WB_CODE_BASE:

        /* All RAM and flash is supported */

        return true;

      default:

        /* Everything else is unsupported by DMA */

        return false;
    }
}
#endif

/****************************************************************************
 * Name: stm32wb_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32wb_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32wb_dmasample(DMA_HANDLE handle, struct stm32wb_dmaregs_s *regs)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint8_t controller;

  DEBUGASSERT(handle != NULL);

  /* Get DMA controller */

  controller = dmachan->ctrl;
  DEBUGASSERT(controller >= DMA1 && controller <= DMA2);

  g_dma_ops[controller].dma_sample(handle, regs);
}
#endif

/****************************************************************************
 * Name: stm32wb_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32wb_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32wb_dmadump(DMA_HANDLE handle, const struct stm32wb_dmaregs_s *regs,
                     const char *msg)
{
  DMA_CHANNEL dmachan = (DMA_CHANNEL)handle;
  uint8_t controller;

  DEBUGASSERT(handle != NULL);

  /* Get DMA controller */

  controller = dmachan->ctrl;
  DEBUGASSERT(controller >= DMA1 && controller <= DMA2);

  dmainfo("DMA %d CH%d Registers: %s\n", dmachan->ctrl, dmachan->ctrl, msg);

  g_dma_ops[controller].dma_dump(handle, regs, msg);
}
#endif
