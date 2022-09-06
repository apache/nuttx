/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_dma.c
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

#include "sched/sched.h"
#include "chip.h"
#include "gd32f4xx_dma.h"
#include "gd32f4xx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if GD32_NDMA > 1

#define DMA0_NPERIPHS       (8)

#define DMA0_NCHANNELS      (8)
#define DMA1_NCHANNELS      (8)
#define DMA_NCHANNELS       (DMA0_NCHANNELS + DMA1_NCHANNELS)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct gd32_dma_channel_s
{
  uint8_t        chan_num;         /* DMA channel number (0-7) */
  uint8_t        irq;              /* DMA channel IRQ number */
  uint8_t        periph;           /* DMA peripheral number (0-7) */
  sem_t          chsem;            /* Used to wait for DMA channel to become available */
  uint32_t       dmabase;          /* DMA base address */
  dma_callback_t callback;         /* Callback invoked when the DMA completes */
  void           *arg;             /* Argument passed to callback function */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t gd32_dma_interrupt_flag_get(uint32_t dma_periph,
                                           uint8_t channelx);
static void gd32_dma_interrupt_flag_clear(uint32_t dma_periph,
                                          uint8_t channelx,
                                          uint8_t flag);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array describes the state of each DMA */

static struct gd32_dma_channel_s g_dmachan[DMA_NCHANNELS] =
{
  {
    .chan_num  = GD32_DMA_CH0,
    .irq       = GD32_IRQ_DMA0_CHANNEL0,
    .dmabase   = GD32_DMA0,
  },
  {
    .chan_num  = GD32_DMA_CH1,
    .irq       = GD32_IRQ_DMA0_CHANNEL1,
    .dmabase   = GD32_DMA0,
  },
  {
    .chan_num  = GD32_DMA_CH2,
    .irq       = GD32_IRQ_DMA0_CHANNEL2,
    .dmabase   = GD32_DMA0,
  },
  {
    .chan_num  = GD32_DMA_CH3,
    .irq       = GD32_IRQ_DMA0_CHANNEL3,
    .dmabase   = GD32_DMA0,
  },
  {
    .chan_num  = GD32_DMA_CH4,
    .irq       = GD32_IRQ_DMA0_CHANNEL4,
    .dmabase   = GD32_DMA0,
  },
  {
    .chan_num  = GD32_DMA_CH5,
    .irq       = GD32_IRQ_DMA0_CHANNEL5,
    .dmabase   = GD32_DMA0,
  },
  {
    .chan_num  = GD32_DMA_CH6,
    .irq       = GD32_IRQ_DMA0_CHANNEL6,
    .dmabase   = GD32_DMA0,
  },
  {
    .chan_num  = GD32_DMA_CH7,
    .irq       = GD32_IRQ_DMA0_CHANNEL7,
    .dmabase   = GD32_DMA0,
  },

  {
    .chan_num  = GD32_DMA_CH0,
    .irq       = GD32_IRQ_DMA1_CHANNEL0,
    .dmabase   = GD32_DMA1,
  },
  {
    .chan_num  = GD32_DMA_CH1,
    .irq       = GD32_IRQ_DMA1_CHANNEL1,
    .dmabase   = GD32_DMA1,
  },
  {
    .chan_num  = GD32_DMA_CH2,
    .irq       = GD32_IRQ_DMA1_CHANNEL2,
    .dmabase   = GD32_DMA1,
  },
  {
    .chan_num  = GD32_DMA_CH3,
    .irq       = GD32_IRQ_DMA1_CHANNEL3,
    .dmabase   = GD32_DMA1,
  },
  {
    .chan_num  = GD32_DMA_CH4,
    .irq       = GD32_IRQ_DMA1_CHANNEL4,
    .dmabase   = GD32_DMA1,
  },
  {
    .chan_num  = GD32_DMA_CH5,
    .irq       = GD32_IRQ_DMA1_CHANNEL5,
    .dmabase   = GD32_DMA1,
  },
  {
    .chan_num  = GD32_DMA_CH6,
    .irq       = GD32_IRQ_DMA1_CHANNEL6,
    .dmabase    = GD32_DMA1,
  },
  {
    .chan_num  = GD32_DMA_CH7,
    .irq       = GD32_IRQ_DMA1_CHANNEL7,
    .dmabase   = GD32_DMA1,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_dma_clock_enable
 *
 * Description:
 *   Enable DMA clock
 ****************************************************************************/

static void gd32_dma_clock_enable(uint32_t dmabase)
{
  uint32_t rcu_en;
  uint32_t regaddr;

  /* Determine which DMA to configure */

  switch (dmabase)
    {
    default:
      return;
#ifdef CONFIG_GD32F4_DMA0
    case GD32_DMA0:
      rcu_en = RCU_AHB1EN_DMA0EN;
      regaddr = GD32_RCU_AHB1EN;
      break;
#endif
#ifdef CONFIG_GD32F4_DMA1
    case GD32_DMA1:
      rcu_en = RCU_AHB1EN_DMA1EN;
      regaddr = GD32_RCU_AHB1EN;
      break;
#endif
    }

  /* Enable AHB1 clock for DMA */

  modifyreg32(regaddr, 0, rcu_en);
}

/****************************************************************************
 * Name: gd32_dma_channel_get
 *
 * Description:
 *   Get the g_dmachan table entry associated with a DMA controller and
 *   a channel number
 ****************************************************************************/

static inline struct gd32_dma_channel_s
                        *gd32_dma_channel_get(uint32_t channelx,
                                              uint32_t dma_periph)
{
  int index;

  DEBUGASSERT(channelx < DMA0_NCHANNELS);

  DEBUGASSERT(dma_periph == GD32_DMA0_BASE && dma_periph == GD32_DMA1_BASE);

  /* Convert the dma_periph + chan_num based on the fact that there are
   * 8 channel per dma_periph.
   */

  if (dma_periph == GD32_DMA0_BASE)
    {
      index = channelx;
    }
  else
    {
      index = channelx + DMA0_NCHANNELS;
    }

  /* Then return the chan_num structure associated with the chan_num index */

  return &g_dmachan[index];
}

/****************************************************************************
 * Name: gd32_channel_enable
 *
 * Description:
 *  Enable the DMA channelx
 *
 ****************************************************************************/

static void gd32_channel_enable(uint32_t dma_periph, uint8_t channelx)
{
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT(channelx < DMA0_NCHANNELS);

  DEBUGASSERT(dma_periph == GD32_DMA0_BASE && dma_periph == GD32_DMA1_BASE);

  /* Get DMA channel control register address */

  regaddr = GD32_DMA_CHCTL(dma_periph, channelx);

  /* Enable DMA channelx */

  regval = getreg32(regaddr);
  regval |= DMA_CHXCTL_CHEN;

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_channel_disable
 *
 * Description:
 *  Disable the DMA channelx
 *
 ****************************************************************************/

static void gd32_channel_disable(uint32_t dma_periph, uint8_t channelx)
{
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT(channelx < DMA0_NCHANNELS);

  DEBUGASSERT(dma_periph == GD32_DMA0_BASE && dma_periph == GD32_DMA1_BASE);

  /* Get DMA channel control register address */

  regaddr = GD32_DMA_CHCTL(dma_periph, channelx);

  /* Disable DMA channelx */

  regval = getreg32(regaddr);
  regval &= ~DMA_CHXCTL_CHEN;

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_channel_interrupt_enable
 *
 * Description:
 *  Enable the DMA channelx interrupt
 *
 ****************************************************************************/

static void gd32_channel_interrupt_enable(uint32_t dma_periph,
                                          uint8_t channelx,
                                          uint32_t interrupt)
{
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT(channelx < DMA0_NCHANNELS);

  DEBUGASSERT(dma_periph == GD32_DMA0_BASE && dma_periph == GD32_DMA1_BASE);

  /* Get DMA channel control register address */

  regaddr = GD32_DMA_CHCTL(dma_periph, channelx);

  /* Disable all interrupts at the DMA controller */

  regval = getreg32(regaddr);
  regval &= ~(DMA_CHXCTL_SDEIE | DMA_CHXCTL_TAEIE |
              DMA_CHXCTL_HTFIE | DMA_CHXCTL_FTFIE);

  /* Set DMA channelx interrupt */

  regval |= interrupt;

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_channel_interrupt_disable
 *
 * Description:
 *  Disable the DMA channelx interrupt
 *
 ****************************************************************************/

static void gd32_channel_interrupt_disable(uint32_t dma_periph,
                                           uint8_t channelx)
{
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT(channelx < DMA0_NCHANNELS);

  DEBUGASSERT(dma_periph == GD32_DMA0_BASE && dma_periph == GD32_DMA1_BASE);

  /* Get DMA channel control register address */

  regaddr = GD32_DMA_CHCTL(dma_periph, channelx);

  /* Disable all interrupts at the DMA controller */

  regval = getreg32(regaddr);
  regval &= ~(DMA_CHXCTL_SDEIE | DMA_CHXCTL_TAEIE |
              DMA_CHXCTL_HTFIE | DMA_CHXCTL_FTFIE);

  putreg32(regval, regaddr);

  /* Clear channelx interrupt flag */

  regval = (DMA_INTF_FEEIF | DMA_INTF_SDEIF | DMA_INTF_TAEIF |
           DMA_INTF_HTFIF | DMA_INTF_FTFIF);
  gd32_dma_interrupt_flag_clear(dma_periph, channelx, (uint8_t)regval);
}

/****************************************************************************
 * Name: gd32_dma_interrupt_flag_get
 *
 * Description:
 *  Get DMA interrupt flag
 *
 ****************************************************************************/

static uint8_t gd32_dma_interrupt_flag_get(uint32_t dma_periph,
                                           uint8_t channelx)
{
  uint32_t regval;
  uint8_t chan_intf_shift = 0;
  uint8_t status = 0;

  DEBUGASSERT(channelx < DMA0_NCHANNELS);

  DEBUGASSERT(dma_periph == GD32_DMA0_BASE && dma_periph == GD32_DMA1_BASE);

  /* Get the interrupt status for this channel */

  if (channelx < 4)
    {
      /* Calculate channelx interrupt flag shift */

      chan_intf_shift = channelx * 8 - (channelx % 2) * 2;

      regval = getreg32(dma_periph + GD32_DMA_INTF0_OFFSET);
      status = (regval >> chan_intf_shift) & DMA_INTF_MASK;
    }
  else
    {
      /* Calculate channelx interrupt flag shift */

      channelx = channelx - 4;
      chan_intf_shift = channelx * 8 - (channelx % 2) * 2;

      regval = getreg32(dma_periph + GD32_DMA_INTF1_OFFSET);
      status = (regval >> chan_intf_shift) & DMA_INTF_MASK;
    }

  return status;
}

/****************************************************************************
 * Name: gd32_dma_interrupt_flag_clear
 *
 * Description:
 *  Clear DMA interrupt flag
 *
 ****************************************************************************/

static void gd32_dma_interrupt_flag_clear(uint32_t dma_periph,
                                          uint8_t channelx, uint8_t flag)
{
  uint32_t regval;
  uint8_t chan_intf_shift = 0;

  DEBUGASSERT(channelx < DMA0_NCHANNELS);

  DEBUGASSERT(dma_periph == GD32_DMA0_BASE && dma_periph == GD32_DMA1_BASE);

  /* Clear the interrupt status for this channel */

  if (channelx < 4)
    {
      /* Calculate channelx interrupt flag shift */

      chan_intf_shift = channelx * 8 - (channelx % 2) * 2;

      if (flag)
        {
          regval = flag << chan_intf_shift;

          /* Clear channelx interrupt flag */

          putreg32(regval, (dma_periph + GD32_DMA_INTC0_OFFSET));
        }
    }
  else
    {
      /* Calculate channelx interrupt flag shift */

      channelx = channelx - 4;
      chan_intf_shift = channelx * 8 - (channelx % 2) * 2;

      if (flag)
        {
          regval = flag << chan_intf_shift;

          /* Clear channelx interrupt flag */

          putreg32(regval, (dma_periph + GD32_DMA_INTC1_OFFSET));
        }
    }
}

/****************************************************************************
 * Name: gd32_dma_interrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int gd32_dma_interrupt(int irq, void *context, void *arg)
{
  struct gd32_dma_channel_s *dmachan;
  uint8_t status;

  /* 'arg' should the DMA channel instance. */

  dmachan = (struct gd32_dma_channel_s *)arg;

  DEBUGASSERT(dmachan != NULL);

  if (dmachan->dmabase == GD32_DMA0_BASE)
    {
      DEBUGASSERT(dmachan == &g_dmachan[dmachan->chan_num]);
    }
  else
    {
      DEBUGASSERT(dmachan == &g_dmachan[dmachan->chan_num + DMA0_NCHANNELS]);
    }

  /* Get the interrupt status for this channel */

  status = gd32_dma_interrupt_flag_get(dmachan->dmabase, dmachan->chan_num);

  if (status)
    {
      gd32_dma_interrupt_flag_clear(dmachan->dmabase, dmachan->chan_num,
                                    status);

      /* Call the DMA callback */

      if (dmachan->callback)
        {
          dmachan->callback(dmachan, status, dmachan->arg);
        }
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
  struct gd32_dma_channel_s *dmachan;
  uint8_t channelx;

  dmainfo("Initialize DMA\n");

  /* Initialize each DMA channel */

  for (channelx = 0; channelx < DMA_NCHANNELS; channelx++)
    {
      dmachan = &g_dmachan[channelx];

      DEBUGASSERT(dmachan != NULL);

      nxsem_init(&dmachan->chsem, 0, 1);

      /* Attach DMA interrupt vectors */

      irq_attach(dmachan->irq, gd32_dma_interrupt, dmachan);

      /* Disable the DMA channel and channel interrupt */

      gd32_channel_interrupt_disable(dmachan->dmabase, dmachan->chan_num);
      gd32_channel_disable(dmachan->dmabase, dmachan->chan_num);

      /* Enable the channel interrupts at the NVIC (still disabled
       * at the DMA controller)
       */

      up_enable_irq(dmachan->irq);
    }
}

/****************************************************************************
 * Name: gd32_dma_channel_alloc
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'dmamap' argument.
 *   Should note that a DMA channel only can be used by on peripheral at the
 *   time.
 *
 *   If the DMA channel is not available, then gd32_dma_channel_alloc() will
 *   wait until the holder of the channel relinquishes the channel by calling
 *   gd32_dma_channel_free().
 *
 * Input Parameters:
 *   periph_req - Identifies the DMA channle is request by which peripheral
 *
 * Returned Value:
 *   If periph_req is valid, this function ALWAYS returns a non-NULL
 *   void* DMA channel handle.
 *
 ****************************************************************************/

DMA_HANDLE gd32_dma_channel_alloc(uint8_t periph_req)
{
  struct gd32_dma_channel_s *dmachan;
  int ret;
  uint8_t index = 0;
  uint8_t subperiph = 0;

  /* Get the peripheral numbers from the peripheral request */

  subperiph = (periph_req >> PERIPH_SHIFT) & PERIPH_MASK;

  DEBUGASSERT(subperiph < DMA0_NPERIPHS);

  /* Get the channel index from the peripheral request */

  index = (periph_req >> CHANNEL_SHIFT) & CHANNEL_MASK;

  DEBUGASSERT(index < DMA_NCHANNELS);

  /* Get the chan_num index from the bit-encoded channel value */

  dmachan = &g_dmachan[index];

  DEBUGASSERT(dmachan != NULL);

  /* Get exclusive access to the DMA channel */

  ret = nxsem_wait_uninterruptible(&dmachan->chsem);
  if (ret < 0)
    {
      return NULL;
    }

  dmachan->periph = subperiph;

  return (DMA_HANDLE)dmachan;
}

/****************************************************************************
 * Name: gd32_dma_channel_free
 *
 * Description:
 *   Release a DMA channel.  If another thread is waiting for this DMA
 *   channel in a call to gd32_dma_channel_alloc, then this function will
 *   re-assign the DMA channel to that thread and wake it up.
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *   until gd32_dma_channel_alloc() is called again to re-gain access to
 *   the channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void gd32_dma_channel_free(DMA_HANDLE handle)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;

  DEBUGASSERT(dmachan != NULL);

  /* Disable the DMA channel and channel interrupt */

  gd32_channel_interrupt_disable(dmachan->dmabase, dmachan->chan_num);
  gd32_channel_disable(dmachan->dmabase, dmachan->chan_num);

  /* Release the channel */

  nxsem_post(&dmachan->chsem);
}

/****************************************************************************
 * Name: gd32_dma_singlemode_setup
 *
 * Description:
 *   Configure DMA single data mode before using
 *
 ****************************************************************************/

void gd32_dma_singlemode_setup(struct gd32_dma_channel_s *dmachan,
                               dma_single_data_parameter_struct *init_struct)
{
  uint32_t regaddr;
  uint32_t regval;

  dmainfo("peripheral address: %08" PRIx32 "memory address: %08" PRIx32
          "transfer numbers: %08" PRIx16 "\n",
          init_struct->periph_addr, init_struct->memory0_addr,
          init_struct->number);

  /* Clear CHEN bit in channel x control register */

  regaddr = GD32_DMA_CHCTL(dmachan->dmabase, dmachan->chan_num);

  regval = getreg32(regaddr);
  regval &= ~DMA_CHXCTL_CHEN;
  putreg32(regval, regaddr);

  /* Clear channelx interrupt flag */

  regval = (DMA_INTF_FEEIF | DMA_INTF_SDEIF | DMA_INTF_TAEIF |
            DMA_INTF_HTFIF | DMA_INTF_FTFIF);
  gd32_dma_interrupt_flag_clear(dmachan->dmabase, dmachan->chan_num,
                               (uint8_t)regval);

  /* Select single data mode */

  regaddr = GD32_DMA_CHFCTL(dmachan->dmabase, dmachan->chan_num);
  regval = getreg32(regaddr);
  regval &= ~DMA_CHXFCTL_MDMEN;
  putreg32(regval, regaddr);

  /* Configure peripheral base address */

  regaddr = GD32_DMA_CHPADDR(dmachan->dmabase, dmachan->chan_num);
  putreg32(init_struct->periph_addr, regaddr);

  /* Configure memory base address */

  regaddr = GD32_DMA_CHM0ADDR(dmachan->dmabase, dmachan->chan_num);
  putreg32(init_struct->memory0_addr, regaddr);

  /* Configure the number of remaining data to be transferred */

  regaddr = GD32_DMA_CHCNT(dmachan->dmabase, dmachan->chan_num);
  putreg32(init_struct->number, regaddr);

  /* Configure peripheral and memory transfer width, channel priotity,
   * transfer mode
   */

  regaddr = GD32_DMA_CHCTL(dmachan->dmabase, dmachan->chan_num);
  regval = getreg32(regaddr);

  regval &= ~(DMA_CHXCTL_PWIDTH_MASK | DMA_CHXCTL_MWIDTH_MASK |
              DMA_CHXCTL_PRIO_MASK | DMA_CHXCTL_TM_MASK);

  regval |= init_struct->direction;

  if (DMA_WIDTH_8BITS_SELECT == init_struct->periph_memory_width)
    {
      regval |= DMA_MEMORY_WIDTH_8BIT;
    }
  else if (DMA_WIDTH_16BITS_SELECT == init_struct->periph_memory_width)
    {
      regval |= DMA_MEMORY_WIDTH_16BIT;
    }
  else if (DMA_WIDTH_32BITS_SELECT == init_struct->periph_memory_width)
    {
      regval |= DMA_MEMORY_WIDTH_32BIT;
    }
  else
    {
      regval |= DMA_MEMORY_WIDTH_8BIT;
    }

  if (DMA_PRIO_LOW_SELECT == init_struct->priority)
    {
      regval |= DMA_PRIORITY_LOW;
    }
  else if (DMA_PRIO_MEDIUM_SELECT == init_struct->priority)
    {
      regval |= DMA_PRIORITY_MEDIUM;
    }
  else if (DMA_PRIO_HIGH_SELECT == init_struct->priority)
    {
      regval |= DMA_PRIORITY_HIGH;
    }
  else if (DMA_PRIO_ULTRA_HIGHSELECT == init_struct->priority)
    {
      regval |= DMA_PRIORITY_ULTRA_HIGH;
    }
  else
    {
      regval |= DMA_PRIORITY_MEDIUM;
    }

  /* Configure peripheral, memory increasing mode and DMA circular mode */

  if (DMA_PERIPH_INCREASE_ENABLE == init_struct->periph_inc)
    {
      regval |= DMA_CHXCTL_PNAGA;
    }
  else if (DMA_PERIPH_INCREASE_DISABLE == init_struct->periph_inc)
    {
      regval &= ~DMA_CHXCTL_PNAGA;
    }
  else
    {
      regval |= DMA_CHXCTL_PAIF;
    }

  if (DMA_MEMORY_INCREASE_ENABLE == init_struct->memory_inc)
    {
      regval |= DMA_CHXCTL_MNAGA;
    }
  else
    {
      regval &= ~DMA_CHXCTL_MNAGA;
    }

  if (DMA_CIRCULAR_MODE_ENABLE == init_struct->circular_mode)
    {
      regval |= DMA_CHXCTL_CMEN;
    }
  else
    {
      regval &= ~DMA_CHXCTL_CMEN;
    }

  /* DMA channel peripheral select */

  regval &= ~DMA_CHXCTL_PERIEN_MASK;
  regval |= (dmachan->periph << DMA_CHXCTL_PERIEN_SHIFT);

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_dma_setup
 *
 * Description:
 *   Configure DMA before using
 *
 ****************************************************************************/

void gd32_dma_setup(DMA_HANDLE handle, void *arg, uint8_t data_mode)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;

  gd32_dma_clock_enable(dmachan->dmabase);

  data_mode = 1;

  if (data_mode)
    {
      dma_single_data_parameter_struct *init_struct =
                                    (dma_single_data_parameter_struct *)arg;
      gd32_dma_singlemode_setup(dmachan, init_struct);
    }
}

/****************************************************************************
 * Name: gd32_dma_start
 *
 * Description:
 *   Start the DMA transfer
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *   - No DMA in progress
 *
 ****************************************************************************/

void gd32_dma_start(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    uint32_t interrupt)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;
  uint32_t regval;

  DEBUGASSERT(dmachan != NULL);

  DEBUGASSERT(dmachan->chan_num < DMA0_NCHANNELS);

  DEBUGASSERT(dmachan->dmabase == GD32_DMA0_BASE &&
              dmachan->dmabase == GD32_DMA1_BASE);

  /* Save the callback info.  This will be invoked when the DMA completes */

  dmachan->callback = callback;
  dmachan->arg      = arg;

  if (interrupt & DMA_INT_MASK)
    {
      regval = (interrupt & DMA_INT_MASK);
    }
  else
    {
      /* If interrupt flag is not set, then set full transfer finish
       * interrupt as default
       */

      regval = DMA_CHXCTL_FTFIE;
    }

  /* Enable DMA channel and channel interrupt */

  gd32_channel_interrupt_enable(dmachan->dmabase, dmachan->chan_num, regval);

  gd32_channel_enable(dmachan->dmabase, dmachan->chan_num);
}

/****************************************************************************
 * Name: gd32_dma_stop
 *
 * Description:
 *   Cancel the DMA.  After gd32_dma_stop() is called, the DMA channel is
 *   reset and gd32_dma_setup() must be called before gd32_dma_start()
 *   can be called again
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

void gd32_dma_stop(DMA_HANDLE handle)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;

  DEBUGASSERT(dmachan != NULL);

  DEBUGASSERT(dmachan->chan_num < DMA0_NCHANNELS);

  DEBUGASSERT(dmachan->dmabase == GD32_DMA0_BASE &&
              dmachan->dmabase == GD32_DMA1_BASE);

  gd32_channel_interrupt_disable(dmachan->dmabase, dmachan->chan_num);
  gd32_channel_disable(dmachan->dmabase, dmachan->chan_num);
}

/****************************************************************************
 * Name: gd32_dma_tansnum_get
 *
 * Description:
 *   Get the number of remaining data to be transferred by the DMA
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

size_t gd32_dma_tansnum_get(DMA_HANDLE handle)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;
  uint32_t remain_num;

  DEBUGASSERT(dmachan != NULL);

  DEBUGASSERT(dmachan->chan_num < DMA0_NCHANNELS);

  DEBUGASSERT(dmachan->dmabase == GD32_DMA0_BASE &&
              dmachan->dmabase == GD32_DMA1_BASE);

  remain_num = getreg32(GD32_DMA_CHCNT(dmachan->dmabase, dmachan->chan_num));

  return (size_t)remain_num;
}

/****************************************************************************
 * Name: gd32_dma_sample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void gd32_dma_sample(DMA_HANDLE handle, struct gd32_dmaregs_s *regs)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;
  irqstate_t flags;

  DEBUGASSERT(dmachan != NULL);

  DEBUGASSERT(dmachan->chan_num < DMA0_NCHANNELS);

  DEBUGASSERT(dmachan->dmabase == GD32_DMA0_BASE &&
              dmachan->dmabase == GD32_DMA1_BASE);

  flags       = enter_critical_section();

  regs->intf0    = getreg32(dmachan->dmabase + GD32_DMA_INTF0_OFFSET);
  regs->intf1    = getreg32(dmachan->dmabase + GD32_DMA_INTF1_OFFSET);
  regs->chctl    = getreg32(GD32_DMA_CHCTL(dmachan->dmabase,
                            dmachan->chan_num));
  regs->chcnt    = getreg32(GD32_DMA_CHCNT(dmachan->dmabase,
                            dmachan->chan_num));
  regs->chpaddr  = getreg32(GD32_DMA_CHPADDR(dmachan->dmabase,
                            dmachan->chan_num));
  regs->chm0addr = getreg32(GD32_DMA_CHM0ADDR(dmachan->dmabase,
                            dmachan->chan_num));
  regs->chm1addr = getreg32(GD32_DMA_CHM1ADDR(dmachan->dmabase,
                            dmachan->chan_num));
  regs->chfctl   = getreg32(GD32_DMA_CHFCTL(dmachan->dmabase,
                            dmachan->chan_num));

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: gd32_dma_dump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by gd32_dma_channel_alloc()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void gd32_dma_dump(DMA_HANDLE handle, const struct gd32_dmaregs_s *regs,
                   const char *msg)
{
  struct gd32_dma_channel_s *dmachan = (struct gd32_dma_channel_s *)handle;

  DEBUGASSERT(dmachan != NULL);

  dmainfo("DMA Registers: %s\n", msg);
  dmainfo("     INTF0: %08x\n", regs->intf0);
  dmainfo("     INTF1: %08x\n", regs->intf1);
  dmainfo("     CHCTL: %08x\n", regs->chctl);
  dmainfo("     CHCNT: %08x\n", regs->chcnt);
  dmainfo("   CHPADDR: %08x\n", regs->chpaddr);
  dmainfo("  CHM0ADDR: %08x\n", regs->chm0addr);
  dmainfo("  CHM1ADDR: %08x\n", regs->chm1addr);
  dmainfo("    CHFCTL: %08x\n", regs->chfctl);
}
#endif

#endif /* GD32_NDMA */
