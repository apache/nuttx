/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_dma.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "hardware/mpfs_dma.h"
#include "mpfs_dma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMA_MAX_TRANSACTION_SIZE (0x0f)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_channel_nextcfg_wsize[MPFS_DMA_NUM_CHANNELS] =
{
  DMA_MAX_TRANSACTION_SIZE, DMA_MAX_TRANSACTION_SIZE,
  DMA_MAX_TRANSACTION_SIZE, DMA_MAX_TRANSACTION_SIZE
};

static uint8_t g_channel_nextcfg_rsize[MPFS_DMA_NUM_CHANNELS] =
{
  DMA_MAX_TRANSACTION_SIZE, DMA_MAX_TRANSACTION_SIZE,
  DMA_MAX_TRANSACTION_SIZE, DMA_MAX_TRANSACTION_SIZE
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_dma_setup_transfer
 *
 * Description:
 *   Set DMA transfer config for channel.
 *
 * Parameters:
 *   channel   - Channel number 0-3
 *   config    - Pointer to the config structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int mpfs_dma_setup_transfer(unsigned int channel,
                            struct mpfs_dma_channel_config *config)
{
  if (channel >= MPFS_DMA_NUM_CHANNELS)
    {
      dmawarn("Illegal channel");
      return -EINVAL;
    }

  if (config->src_addr == 0)
    {
      dmawarn("Illegal source address\n");
      return -EINVAL;
    }

  if (config->dest_addr == 0)
    {
      dmawarn("Illegal destination address\n");
      return -EINVAL;
    }

  /* If transaction is in progress, return error. */

  if (getreg32(MPFS_DMA_REG_OFFSET(channel)) & DMA_CONTROL_RUN_MASK)
    {
      dmawarn("channel busy\n");
      return -EBUSY;
    }

  /* Set or clear the interrupts for the transfer. */

  if (config->enable_done_int)
    {
      dmainfo("enable done irq\n");
      modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_CONTROL_OFFSET,
                  0, DMA_CONTROL_DONEIE);
    }
  else
    {
      dmainfo("disable done irq\n");
      modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_CONTROL_OFFSET,
                  DMA_CONTROL_DONEIE_MASK, 0);
    }

  if (config->enable_err_int)
    {
      dmainfo("enable err irq\n");
      modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_CONTROL_OFFSET,
                  0, DMA_CONTROL_ERRORIE);
    }
  else
    {
      dmainfo("disable err irq\n");
      modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_CONTROL_OFFSET,
                  DMA_CONTROL_ERRORIE_MASK, 0);
    }

  /* clear Next registers. */

  modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_CONTROL_OFFSET,
              0, DMA_CONTROL_CLAIM);

  /* clear Done and Error */

  modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_CONTROL_OFFSET,
              DMA_CONTROL_DONE_MASK | DMA_CONTROL_ERROR_MASK, 0);

  /* Setup the source and destination addresses. */

  putreg64(config->dest_addr,
           MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_NEXT_DESTINATION_OFFSET);
  putreg64(config->src_addr,
           MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_NEXT_SOURCE_OFFSET);

  /* Set the transfer size. */

  putreg64(config->num_bytes,
           MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_NEXT_BYTES_OFFSET);

  /* Setup repeat and force order requirements. */

  if (config->repeat)
    {
      modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_NEXT_CONFIG_OFFSET,
                  0, DMA_NEXT_CONFIG_REPEAT);
    }
  else
    {
      modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_NEXT_CONFIG_OFFSET,
                  DMA_NEXT_CONFIG_REPEAT_MASK, 0);
    }

  if (config->force_order)
    {
      modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_NEXT_CONFIG_OFFSET,
                  0, DMA_NEXT_CONFIG_ORDER);
    }
  else
    {
      modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_NEXT_CONFIG_OFFSET,
                  DMA_NEXT_CONFIG_ORDER_MASK, 0);
    }

  /* Set PDMA transaction size to maximum. */

  modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_NEXT_CONFIG_OFFSET,
              DMA_NEXT_CONFIG_WSIZE_MASK,
              DMA_NEXT_CONFIG_WSIZE(g_channel_nextcfg_wsize[channel]));

  modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_NEXT_CONFIG_OFFSET,
              DMA_NEXT_CONFIG_RSIZE_MASK,
              DMA_NEXT_CONFIG_RSIZE(g_channel_nextcfg_wsize[channel]));

  dmainfodumpbuffer("dma regs", (uint8_t *)MPFS_DMA_REG_OFFSET(channel),
                    MPFS_DMA_NEXT_SOURCE_OFFSET + 8);

  return OK;
}

/****************************************************************************
 * Name: mpfs_dma_set_transaction_size
 *
 * Description:
 *   Set read and write transaction size for mpfs_dma_setup_transfer()
 *
 * Parameters:
 *   channel     - Channel number 0-3
 *   write_size  - Write transaction size
 *   read_size   - Read transaction size
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int mpfs_dma_set_transaction_size(unsigned int channel,
                                  uint8_t write_size, uint8_t read_size)
{
  if (channel >= MPFS_DMA_NUM_CHANNELS)
    {
      dmawarn("Illegal channel\n");
      return -EINVAL;
    }

  if (write_size > DMA_MAX_TRANSACTION_SIZE)
    {
      dmawarn("Illegal write size\n");
      return -EINVAL;
    }

  if (read_size > DMA_MAX_TRANSACTION_SIZE)
    {
      dmawarn("Illegal write size\n");
      return -EINVAL;
    }

  dmainfo("new default dma transaction size. channel:%d write:%d, read:%d\n",
          channel, write_size, read_size);
  g_channel_nextcfg_wsize[channel] = write_size;
  g_channel_nextcfg_rsize[channel] = read_size;

  return OK;
}

/****************************************************************************
 * Name: mpfs_dma_start
 *
 * Description:
 *   Start DMA transfer.
 *
 * Parameters:
 *   channel     - Channel number 0-3
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int mpfs_dma_start(unsigned int channel)
{
  if (channel >= MPFS_DMA_NUM_CHANNELS)
    {
      dmawarn("Illegal channel\n");
      return -EINVAL;
    }

  dmainfo("start dma channel:%d\n", channel);
  modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_CONTROL_OFFSET,
              0, DMA_CONTROL_RUN);

  return OK;
}

/****************************************************************************
 * Name: mpfs_dma_get_transfer_type
 *
 * Description:
 *   Return channels active config.
 *
 * Parameters:
 *   channel     - Channel number 0-3
 *
 * Returned Value:
 *   0      - Illegal channel
 *   bit 2  - Repeat
 *   bit 3  - Strict ordering
 *
 ****************************************************************************/

uint32_t mpfs_dma_get_transfer_type(unsigned int channel)
{
  if (channel >= MPFS_DMA_NUM_CHANNELS)
    {
      dmawarn("Illegal channel\n");
      return 0;
    }

  return getreg32(MPFS_DMA_REG_OFFSET(channel) +
                  MPFS_DMA_EXEC_CONFIG_OFFSET);
}

/****************************************************************************
 * Name: mpfs_dma_get_bytes_remaining
 *
 * Description:
 *   Return number of bytes remaining on transfer.
 *
 * Parameters:
 *   channel     - Channel number 0-3
 *
 * Returned Value:
 *   Number of bytes remaining
 *
 ****************************************************************************/

uint64_t mpfs_dma_get_bytes_remaining(unsigned int channel)
{
  if (channel >= MPFS_DMA_NUM_CHANNELS)
    {
      dmawarn("Illegal channel\n");
      return 0;
    }

  return getreg64(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_EXEC_BYTES_OFFSET);
}

/****************************************************************************
 * Name: mpfs_dma_get_current_dest
 *
 * Description:
 *   Return current destination address of transfer
 *
 * Parameters:
 *   channel     - Channel number 0-3
 *
 * Returned Value:
 *   Current destination address
 *
 ****************************************************************************/

uint64_t mpfs_dma_get_current_destination(unsigned int channel)
{
  if (channel >= MPFS_DMA_NUM_CHANNELS)
    {
      dmawarn("Illegal channel\n");
      return 0;
    }

  return getreg64(MPFS_DMA_REG_OFFSET(channel) +
                  MPFS_DMA_EXEC_DESTINATION_OFFSET);
}

/****************************************************************************
 * Name: mpfs_dma_get_current_source
 *
 * Description:
 *   Return current sourceaddress of transfer
 *
 * Parameters:
 *   channel     - Channel number 0-3
 *
 * Returned Value:
 *   Current source address
 *
 ****************************************************************************/

uint64_t mpfs_dma_get_current_source(unsigned int channel)
{
  if (channel >= MPFS_DMA_NUM_CHANNELS)
    {
      dmawarn("Illegal channel\n");
      return 0;
    }

  return getreg64(MPFS_DMA_REG_OFFSET(channel) +
                  MPFS_DMA_EXEC_SOURCE_OFFSET);
}

/****************************************************************************
 * Name: mpfs_dma_get_complete_status
 *
 * Description:
 *   Return complete status for DMA channel
 *
 * Parameters:
 *   channel     - Channel number 0-3
 *
 * Returned Value:
 *   0  - Not completed
 *   1  - Transfer completed
 *
 ****************************************************************************/

int mpfs_dma_get_complete_status(unsigned int channel)
{
  uint32_t control;

  if (channel >= MPFS_DMA_NUM_CHANNELS)
    {
      dmawarn("Illegal channel\n");
      return 0;
    }

  control = getreg32(MPFS_DMA_REG_OFFSET(channel) +
                       MPFS_DMA_CONTROL_OFFSET);

  return ((control & DMA_CONTROL_DONE_MASK) >> DMA_CONTROL_DONE_SHIFT);
}

/****************************************************************************
 * Name: mpfs_dma_get_error_status
 *
 * Description:
 *   Return error status for DMA channel
 *
 * Parameters:
 *   channel     - Channel number 0-3
 *
 * Returned Value:
 *   0  - No Error
 *   1  - Transfer Errror
 *
 ****************************************************************************/

int mpfs_dma_get_error_status(unsigned int channel)
{
  uint32_t control;

  if (channel >= MPFS_DMA_NUM_CHANNELS)
    {
      dmawarn("Illegal channel\n");
      return 0;
    }

  control = getreg32(MPFS_DMA_REG_OFFSET(channel) +
                       MPFS_DMA_CONTROL_OFFSET);

  return ((control & DMA_CONTROL_ERROR_MASK) >> DMA_CONTROL_ERROR_SHIFT);
}

/****************************************************************************
 * Name: mpfs_dma_clear_complete_status
 *
 * Description:
 *   Clear and return complete status for DMA channel
 *
 * Parameters:
 *   channel     - Channel number 0-3
 *
 * Returned Value:
 *   0  - Not completed
 *   1  - Transfer completed
 *
 ****************************************************************************/

int mpfs_dma_clear_complete_status(unsigned int channel)
{
  int status = 0;
  uint32_t control;

  if (channel >= MPFS_DMA_NUM_CHANNELS)
    {
      dmawarn("Illegal channel\n");
      return 0;
    }

  control = getreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_CONTROL_OFFSET);
  if (control & DMA_CONTROL_DONE)
    {
      status = 1;
    }

  modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_CONTROL_OFFSET,
              DMA_CONTROL_DONE_MASK, 0);

  return status;
}

/****************************************************************************
 * Name: mpfs_dma_clear_error_status
 *
 * Description:
 *   Clear and return error status for DMA channel
 *
 * Parameters:
 *   channel     - Channel number 0-3
 *
 * Returned Value:
 *   0  - No Error
 *   1  - Transfer Errror
 *
 ****************************************************************************/

int mpfs_dma_clear_error_status(unsigned int channel)
{
  int status = 0;
  uint32_t control;

  if (channel >= MPFS_DMA_NUM_CHANNELS)
    {
      dmawarn("Illegal channel\n");
      return 0;
    }

  control = getreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_CONTROL_OFFSET);
  if (control & DMA_CONTROL_ERROR)
    {
      status = 1;
    }

  modifyreg32(MPFS_DMA_REG_OFFSET(channel) + MPFS_DMA_CONTROL_OFFSET,
              DMA_CONTROL_ERROR, 0);

  return status;
}
