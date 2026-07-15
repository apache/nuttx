/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_dma.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>

#include "riscv_internal.h"
#include "chip.h"
#include "hardware/gd32vw55x_rcu.h"
#include "gd32vw55x_dma.h"

#ifdef CONFIG_GD32VW55X_DMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD32VW55x has a single DMA controller with 8 channels and each
 * channel can be connected to one of 8 sub-peripheral requests.
 */

#define DMA_NCHANNELS      GD32VW55X_NDMA_CHANNELS
#define DMA_NPERIPHS       GD32VW55X_NDMA_SUBPERIPHS

/* The interrupt flags of the channels 0-3 are held in the INTF0/INTC0
 * registers and the interrupt flags of the channels 4-7 in the INTF1/INTC1
 * registers.
 */

#define DMA_INTF_NCHANNELS (4)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes one DMA channel */

struct gd32_dma_channel_s
{
  uint8_t        chan_num;  /* DMA channel number (0-7) */
  uint8_t        irq;       /* DMA channel IRQ number */
  uint8_t        periph;    /* DMA sub-peripheral number (0-7) */
  sem_t          chsem;     /* Used to wait for the channel to be free */
  dma_callback_t callback;  /* Callback invoked when the DMA completes */
  void          *arg;       /* Argument passed to the callback function */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t gd32_dma_interrupt_flag_get(uint8_t channelx);
static void gd32_dma_interrupt_flag_clear(uint8_t channelx, uint8_t flag);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array describes the state of each DMA channel */

static struct gd32_dma_channel_s g_dmachan[DMA_NCHANNELS] =
{
  {
    .chan_num = GD32_DMA_CH0,
    .irq      = GD32VW55X_IRQ_DMA_CH0,
    .chsem    = SEM_INITIALIZER(1),
  },
  {
    .chan_num = GD32_DMA_CH1,
    .irq      = GD32VW55X_IRQ_DMA_CH1,
    .chsem    = SEM_INITIALIZER(1),
  },
  {
    .chan_num = GD32_DMA_CH2,
    .irq      = GD32VW55X_IRQ_DMA_CH2,
    .chsem    = SEM_INITIALIZER(1),
  },
  {
    .chan_num = GD32_DMA_CH3,
    .irq      = GD32VW55X_IRQ_DMA_CH3,
    .chsem    = SEM_INITIALIZER(1),
  },
  {
    .chan_num = GD32_DMA_CH4,
    .irq      = GD32VW55X_IRQ_DMA_CH4,
    .chsem    = SEM_INITIALIZER(1),
  },
  {
    .chan_num = GD32_DMA_CH5,
    .irq      = GD32VW55X_IRQ_DMA_CH5,
    .chsem    = SEM_INITIALIZER(1),
  },
  {
    .chan_num = GD32_DMA_CH6,
    .irq      = GD32VW55X_IRQ_DMA_CH6,
    .chsem    = SEM_INITIALIZER(1),
  },
  {
    .chan_num = GD32_DMA_CH7,
    .irq      = GD32VW55X_IRQ_DMA_CH7,
    .chsem    = SEM_INITIALIZER(1),
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_dma_clock_enable
 *
 * Description:
 *   Enable the DMA clock
 *
 ****************************************************************************/

static void gd32_dma_clock_enable(void)
{
  modifyreg32(GD32VW55X_RCU_AHB1EN, 0, RCU_AHB1EN_DMAEN);
}

/****************************************************************************
 * Name: gd32_dma_width_get
 *
 * Description:
 *   Convert a DMA_WIDTH_*_SELECT selection into the register encoded
 *   transfer width value.  32-bit is selected if the width selection is
 *   not valid.
 *
 ****************************************************************************/

static uint32_t gd32_dma_width_get(uint8_t width_select)
{
  uint32_t width;

  switch (width_select)
    {
      case DMA_WIDTH_8BITS_SELECT:
        width = 0;
        break;

      case DMA_WIDTH_16BITS_SELECT:
        width = 1;
        break;

      default:
      case DMA_WIDTH_32BITS_SELECT:
        width = 2;
        break;
    }

  return width;
}

/****************************************************************************
 * Name: gd32_dma_priority_get
 *
 * Description:
 *   Convert a DMA_PRIO_*_SELECT selection into the register encoded channel
 *   priority value.  The medium priority is selected if the priority
 *   selection is not valid.
 *
 ****************************************************************************/

static uint32_t gd32_dma_priority_get(uint8_t prio_select)
{
  uint32_t priority;

  switch (prio_select)
    {
      case DMA_PRIO_LOW_SELECT:
        priority = DMA_PRIORITY_LOW;
        break;

      default:
      case DMA_PRIO_MEDIUM_SELECT:
        priority = DMA_PRIORITY_MEDIUM;
        break;

      case DMA_PRIO_HIGH_SELECT:
        priority = DMA_PRIORITY_HIGH;
        break;

      case DMA_PRIO_ULTRA_HIGHSELECT:
        priority = DMA_PRIORITY_ULTRA_HIGH;
        break;
    }

  return priority;
}

/****************************************************************************
 * Name: gd32_channel_enable
 *
 * Description:
 *   Enable the DMA channelx
 *
 ****************************************************************************/

static void gd32_channel_enable(uint8_t channelx)
{
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT(channelx < DMA_NCHANNELS);

  regaddr = GD32_DMA_CHCTL(channelx);

  regval  = getreg32(regaddr);
  regval |= DMA_CHXCTL_CHEN;

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_channel_disable
 *
 * Description:
 *   Disable the DMA channelx
 *
 ****************************************************************************/

static void gd32_channel_disable(uint8_t channelx)
{
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT(channelx < DMA_NCHANNELS);

  regaddr = GD32_DMA_CHCTL(channelx);

  regval  = getreg32(regaddr);
  regval &= ~DMA_CHXCTL_CHEN;

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_channel_interrupt_enable
 *
 * Description:
 *   Enable the DMA channelx interrupts
 *
 ****************************************************************************/

static void gd32_channel_interrupt_enable(uint8_t channelx,
                                          uint32_t interrupt)
{
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT(channelx < DMA_NCHANNELS);

  regaddr = GD32_DMA_CHCTL(channelx);

  /* Disable all the interrupts at the DMA controller */

  regval  = getreg32(regaddr);
  regval &= ~DMA_INT_MASK;

  /* Set the requested DMA channelx interrupts */

  regval |= (interrupt & DMA_INT_MASK);

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_channel_interrupt_disable
 *
 * Description:
 *   Disable all the DMA channelx interrupts and clear the pending flags
 *
 ****************************************************************************/

static void gd32_channel_interrupt_disable(uint8_t channelx)
{
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT(channelx < DMA_NCHANNELS);

  /* Disable all the interrupts of the channel control register */

  regaddr = GD32_DMA_CHCTL(channelx);

  regval  = getreg32(regaddr);
  regval &= ~DMA_INT_MASK;

  putreg32(regval, regaddr);

  /* Disable the FIFO error interrupt.  Note that, unlike the other channel
   * interrupts, it is enabled in the channel FIFO control register.
   */

  regaddr = GD32_DMA_CHFCTL(channelx);

  regval  = getreg32(regaddr);
  regval &= ~DMA_CHXFCTL_FEEIE;

  putreg32(regval, regaddr);

  /* Clear all the channelx interrupt flags */

  gd32_dma_interrupt_flag_clear(channelx, DMA_INTF_MASK);
}

/****************************************************************************
 * Name: gd32_dma_interrupt_flag_get
 *
 * Description:
 *   Get the DMA channelx interrupt flags
 *
 ****************************************************************************/

static uint8_t gd32_dma_interrupt_flag_get(uint8_t channelx)
{
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT(channelx < DMA_NCHANNELS);

  /* The flags of the channels 0-3 are held in the INTF0 register and the
   * flags of the channels 4-7 in the INTF1 register.
   */

  if (channelx < DMA_INTF_NCHANNELS)
    {
      regaddr = GD32_DMA_INTF0;
    }
  else
    {
      regaddr   = GD32_DMA_INTF1;
      channelx -= DMA_INTF_NCHANNELS;
    }

  regval = getreg32(regaddr) >> ((channelx * 6) + ((channelx >> 1) & 1) * 4);

  return (uint8_t)(regval & DMA_INTF_MASK);
}

/****************************************************************************
 * Name: gd32_dma_interrupt_flag_clear
 *
 * Description:
 *   Clear the DMA channelx interrupt flags
 *
 ****************************************************************************/

static void gd32_dma_interrupt_flag_clear(uint8_t channelx, uint8_t flag)
{
  uint32_t regaddr;

  DEBUGASSERT(channelx < DMA_NCHANNELS);

  if (flag == 0)
    {
      return;
    }

  /* The flags of the channels 0-3 are cleared in the INTC0 register and the
   * flags of the channels 4-7 in the INTC1 register.
   */

  if (channelx < DMA_INTF_NCHANNELS)
    {
      regaddr = GD32_DMA_INTC0;
    }
  else
    {
      regaddr   = GD32_DMA_INTC1;
      channelx -= DMA_INTF_NCHANNELS;
    }

  putreg32(GD32_DMA_FLAG_ADD(flag & DMA_INTF_MASK, channelx), regaddr);
}

/****************************************************************************
 * Name: gd32_dma_interrupt
 *
 * Description:
 *   DMA channel interrupt handler
 *
 ****************************************************************************/

static int gd32_dma_interrupt(int irq, void *context, void *arg)
{
  struct gd32_dma_channel_s *dmachan;
  uint8_t status;

  /* 'arg' should be the DMA channel instance */

  dmachan = (struct gd32_dma_channel_s *)arg;

  DEBUGASSERT(dmachan != NULL);
  DEBUGASSERT(dmachan == &g_dmachan[dmachan->chan_num]);

  /* Get the interrupt status of this channel */

  status = gd32_dma_interrupt_flag_get(dmachan->chan_num);

  if (status != 0)
    {
      gd32_dma_interrupt_flag_clear(dmachan->chan_num, status);

      /* Invoke the DMA callback */

      if (dmachan->callback != NULL)
        {
          dmachan->callback(dmachan, (uint16_t)status, dmachan->arg);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: gd32_dma_singlemode_setup
 *
 * Description:
 *   Configure the DMA channel in single data mode
 *
 ****************************************************************************/

static void gd32_dma_singlemode_setup(struct gd32_dma_channel_s *dmachan,
                                      struct gd32_dma_config_s *cfg)
{
  uint32_t regaddr;
  uint32_t regval;
  uint32_t width;

  dmainfo("periph address: %08" PRIx32 " memory address: %08" PRIx32
          " transfer number: %" PRIu16 "\n",
          cfg->periph_addr, cfg->memory0_addr, cfg->number);

  /* Select the single data mode, i.e. disable the multi data mode */

  regaddr = GD32_DMA_CHFCTL(dmachan->chan_num);

  regval  = getreg32(regaddr);
  regval &= ~DMA_CHXFCTL_MDMEN;

  putreg32(regval, regaddr);

  /* Configure the peripheral base address */

  putreg32(cfg->periph_addr, GD32_DMA_CHPADDR(dmachan->chan_num));

  /* Configure the memory 0 base address */

  putreg32(cfg->memory0_addr, GD32_DMA_CHM0ADDR(dmachan->chan_num));

  /* Configure the number of data to be transferred */

  putreg32(cfg->number, GD32_DMA_CHCNT(dmachan->chan_num));

  /* Configure the transfer mode, the transfer width of the peripheral and
   * of the memory and the channel priority.  In single data mode the
   * peripheral and the memory transfer widths must be the same.
   */

  regaddr = GD32_DMA_CHCTL(dmachan->chan_num);
  regval  = getreg32(regaddr);

  regval &= ~(DMA_CHXCTL_PWIDTH_MASK | DMA_CHXCTL_MWIDTH_MASK |
              DMA_CHXCTL_PRIO_MASK | DMA_CHXCTL_TM_MASK);

  regval |= ((uint32_t)cfg->direction & DMA_CHXCTL_TM_MASK);

  width   = gd32_dma_width_get(cfg->periph_memory_width);
  regval |= DMA_CHXCTL_PWIDTH(width) | DMA_CHXCTL_MWIDTH(width);

  regval |= gd32_dma_priority_get(cfg->priority);

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: gd32_dma_multimode_setup
 *
 * Description:
 *   Configure the DMA channel in multi data mode
 *
 ****************************************************************************/

static void gd32_dma_multimode_setup(struct gd32_dma_channel_s *dmachan,
                                     struct gd32_dma_config_s *cfg)
{
  uint32_t regaddr;
  uint32_t regval;

  dmainfo("periph address: %08" PRIx32 " memory address: %08" PRIx32
          " transfer number: %" PRIu16 "\n",
          cfg->periph_addr, cfg->memory0_addr, cfg->number);

  /* Select the multi data mode and configure the FIFO critical value */

  regaddr = GD32_DMA_CHFCTL(dmachan->chan_num);

  regval  = getreg32(regaddr);
  regval &= ~DMA_CHXFCTL_FCCV_MASK;
  regval |= DMA_CHXFCTL_MDMEN | (cfg->fifo_critical &
                                 DMA_CHXFCTL_FCCV_MASK);

  putreg32(regval, regaddr);

  /* Configure the peripheral base address */

  putreg32(cfg->periph_addr, GD32_DMA_CHPADDR(dmachan->chan_num));

  /* Configure the memory 0 base address */

  putreg32(cfg->memory0_addr, GD32_DMA_CHM0ADDR(dmachan->chan_num));

  /* Configure the number of data to be transferred */

  putreg32(cfg->number, GD32_DMA_CHCNT(dmachan->chan_num));

  /* Configure the transfer mode, the transfer width and the burst type of
   * the peripheral and of the memory and the channel priority
   */

  regaddr = GD32_DMA_CHCTL(dmachan->chan_num);
  regval  = getreg32(regaddr);

  regval &= ~(DMA_CHXCTL_PWIDTH_MASK | DMA_CHXCTL_MWIDTH_MASK |
              DMA_CHXCTL_PRIO_MASK | DMA_CHXCTL_TM_MASK |
              DMA_CHXCTL_PBURST_MASK | DMA_CHXCTL_MBURST_MASK);

  regval |= ((uint32_t)cfg->direction & DMA_CHXCTL_TM_MASK);

  regval |= DMA_CHXCTL_PWIDTH(gd32_dma_width_get(cfg->periph_width));
  regval |= DMA_CHXCTL_MWIDTH(gd32_dma_width_get(cfg->memory_width));

  regval |= (cfg->periph_burst & DMA_CHXCTL_PBURST_MASK);
  regval |= (cfg->memory_burst & DMA_CHXCTL_MBURST_MASK);

  regval |= gd32_dma_priority_get(cfg->priority);

  putreg32(regval, regaddr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_dma_initialize
 *
 * Description:
 *   Initialize the DMA subsystem
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void weak_function riscv_dma_initialize(void)
{
  struct gd32_dma_channel_s *dmachan;
  uint8_t channelx;

  dmainfo("Initialize DMA\n");

  /* Enable the DMA clock before touching any DMA register */

  gd32_dma_clock_enable();

  /* Initialize each DMA channel */

  for (channelx = 0; channelx < DMA_NCHANNELS; channelx++)
    {
      dmachan = &g_dmachan[channelx];

      /* Attach the DMA channel interrupt vector */

      irq_attach(dmachan->irq, gd32_dma_interrupt, dmachan);

      /* Disable the DMA channel and the channel interrupts */

      gd32_channel_interrupt_disable(dmachan->chan_num);
      gd32_channel_disable(dmachan->chan_num);

      /* Enable the channel interrupt at the ECLIC.  The interrupts are
       * still disabled at the DMA controller.
       */

      up_enable_irq(dmachan->irq);
    }
}

/****************************************************************************
 * Name: gd32_dma_channel_alloc
 *
 * Description:
 *   Allocate a DMA channel.  This function gives the caller mutually
 *   exclusive access to the DMA channel specified by the 'periph_req'
 *   argument.  Should note that a DMA channel only can be used by one
 *   peripheral at the time.
 *
 *   If the DMA channel is not available, then gd32_dma_channel_alloc() will
 *   wait until the holder of the channel relinquishes the channel by calling
 *   gd32_dma_channel_free().
 *
 * Input Parameters:
 *   periph_req - Identifies the DMA channel is request by which peripheral
 *
 * Returned Value:
 *   If periph_req is valid, this function ALWAYS returns a non-NULL
 *   void* DMA channel handle.
 *
 ****************************************************************************/

DMA_HANDLE gd32_dma_channel_alloc(uint8_t periph_req)
{
  struct gd32_dma_channel_s *dmachan;
  uint8_t subperiph;
  uint8_t index;
  int ret;

  /* Get the sub-peripheral number from the peripheral request */

  subperiph = (periph_req >> PERIPH_SHIFT) & PERIPH_MASK;

  DEBUGASSERT(subperiph < DMA_NPERIPHS);

  /* Get the channel number from the peripheral request */

  index = (periph_req >> CHANNEL_SHIFT) & CHANNEL_MASK;

  DEBUGASSERT(index < DMA_NCHANNELS);

  dmachan = &g_dmachan[index];

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
  DEBUGASSERT(dmachan->chan_num < DMA_NCHANNELS);

  /* Disable the DMA channel and the channel interrupts */

  gd32_channel_interrupt_disable(dmachan->chan_num);
  gd32_channel_disable(dmachan->chan_num);

  dmachan->callback = NULL;
  dmachan->arg      = NULL;

  /* Release the channel */

  nxsem_post(&dmachan->chsem);
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
  struct gd32_dma_config_s *cfg = (struct gd32_dma_config_s *)arg;
  uint32_t regaddr;
  uint32_t regval;

  DEBUGASSERT(dmachan != NULL && cfg != NULL);
  DEBUGASSERT(dmachan->chan_num < DMA_NCHANNELS);

  /* Make sure that the DMA clock is enabled */

  gd32_dma_clock_enable();

  /* Disable the channel before configuring it */

  gd32_channel_disable(dmachan->chan_num);

  /* Clear all the pending channelx interrupt flags */

  gd32_dma_interrupt_flag_clear(dmachan->chan_num, DMA_INTF_MASK);

  /* Configure the data mode dependent settings */

  if (data_mode == GD32_DMA_MULTI_DATA_MODE)
    {
      gd32_dma_multimode_setup(dmachan, cfg);
    }
  else
    {
      gd32_dma_singlemode_setup(dmachan, cfg);
    }

  /* Configure the address generation algorithm of the peripheral and of the
   * memory, the circular mode and the sub-peripheral selection
   */

  regaddr = GD32_DMA_CHCTL(dmachan->chan_num);
  regval  = getreg32(regaddr);

  if (cfg->periph_inc == DMA_PERIPH_INCREASE_ENABLE)
    {
      regval |= DMA_CHXCTL_PNAGA;
      regval &= ~DMA_CHXCTL_PAIF;
    }
  else if (cfg->periph_inc == DMA_PERIPH_INCREASE_DISABLE)
    {
      regval &= ~(DMA_CHXCTL_PNAGA | DMA_CHXCTL_PAIF);
    }
  else
    {
      regval |= (DMA_CHXCTL_PNAGA | DMA_CHXCTL_PAIF);
    }

  if (cfg->memory_inc == DMA_MEMORY_INCREASE_ENABLE)
    {
      regval |= DMA_CHXCTL_MNAGA;
    }
  else
    {
      regval &= ~DMA_CHXCTL_MNAGA;
    }

  if (cfg->circular_mode == DMA_CIRCULAR_MODE_ENABLE)
    {
      regval |= DMA_CHXCTL_CMEN;
    }
  else
    {
      regval &= ~DMA_CHXCTL_CMEN;
    }

  /* The DMA is always the flow controller */

  regval &= ~DMA_CHXCTL_TFCS;

  /* Select the sub-peripheral of the DMA channel */

  regval &= ~DMA_CHXCTL_PERIEN_MASK;
  regval |= DMA_CHXCTL_PERIEN(dmachan->periph);

  putreg32(regval, regaddr);
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
  DEBUGASSERT(dmachan->chan_num < DMA_NCHANNELS);

  /* Save the callback info.  It will be invoked when the DMA completes */

  dmachan->callback = callback;
  dmachan->arg      = arg;

  regval = interrupt & DMA_INT_MASK;
  if (regval == 0)
    {
      /* If no interrupt is selected, then enable the full transfer finish
       * interrupt as default
       */

      regval = DMA_CHXCTL_FTFIE;
    }

  /* Enable the channel interrupts and the DMA channel */

  gd32_channel_interrupt_enable(dmachan->chan_num, regval);

  gd32_channel_enable(dmachan->chan_num);
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
  DEBUGASSERT(dmachan->chan_num < DMA_NCHANNELS);

  gd32_channel_interrupt_disable(dmachan->chan_num);
  gd32_channel_disable(dmachan->chan_num);
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
  DEBUGASSERT(dmachan->chan_num < DMA_NCHANNELS);

  remain_num = getreg32(GD32_DMA_CHCNT(dmachan->chan_num));

  return (size_t)(remain_num & DMA_CHXCNT_CNT_MASK);
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
  DEBUGASSERT(dmachan->chan_num < DMA_NCHANNELS);

  flags = enter_critical_section();

  regs->intf0    = getreg32(GD32_DMA_INTF0);
  regs->intf1    = getreg32(GD32_DMA_INTF1);
  regs->chctl    = getreg32(GD32_DMA_CHCTL(dmachan->chan_num));
  regs->chcnt    = getreg32(GD32_DMA_CHCNT(dmachan->chan_num));
  regs->chpaddr  = getreg32(GD32_DMA_CHPADDR(dmachan->chan_num));
  regs->chm0addr = getreg32(GD32_DMA_CHM0ADDR(dmachan->chan_num));
  regs->chm1addr = getreg32(GD32_DMA_CHM1ADDR(dmachan->chan_num));
  regs->chfctl   = getreg32(GD32_DMA_CHFCTL(dmachan->chan_num));

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

  dmainfo("DMA channel %u registers: %s\n", dmachan->chan_num, msg);
  dmainfo("     INTF0: %08" PRIx32 "\n", regs->intf0);
  dmainfo("     INTF1: %08" PRIx32 "\n", regs->intf1);
  dmainfo("     CHCTL: %08" PRIx32 "\n", regs->chctl);
  dmainfo("     CHCNT: %08" PRIx32 "\n", regs->chcnt);
  dmainfo("   CHPADDR: %08" PRIx32 "\n", regs->chpaddr);
  dmainfo("  CHM0ADDR: %08" PRIx32 "\n", regs->chm0addr);
  dmainfo("  CHM1ADDR: %08" PRIx32 "\n", regs->chm1addr);
  dmainfo("    CHFCTL: %08" PRIx32 "\n", regs->chfctl);
}
#endif

#endif /* CONFIG_GD32VW55X_DMA */
