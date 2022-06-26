/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_sdpcm.c
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
#include <nuttx/compiler.h>

#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>

#include <stddef.h>
#include <string.h>
#include <queue.h>

#include "bcmf_sdio.h"
#include "bcmf_core.h"
#include "bcmf_sdpcm.h"
#include "bcmf_cdc.h"
#include "bcmf_bdc.h"
#include "bcmf_utils.h"

 #include "bcmf_netdev.h"

#include "bcmf_sdio_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SDPCM_CONTROL_CHANNEL 0  /* Control frame id */
#define SDPCM_EVENT_CHANNEL   1  /* Asynchronous event frame id */
#define SDPCM_DATA_CHANNEL    2  /* Data frame id */

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct bcmf_sdpcm_header
{
  uint16_t size;
  uint16_t checksum;
  uint8_t  sequence;
  uint8_t  channel;
  uint8_t  next_length;
  uint8_t  data_offset;
  uint8_t  flow_control;
  uint8_t  credit;
  uint16_t padding;
} end_packed_struct;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bcmf_sdpcm_rxfail(FAR struct bcmf_sdio_dev_s *sbus, bool retry);

static int bcmf_sdpcm_process_header(FAR struct bcmf_sdio_dev_s *sbus,
                              struct bcmf_sdpcm_header *header);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int bcmf_sdpcm_rxfail(FAR struct bcmf_sdio_dev_s *sbus, bool retry)
{
  /* issue abort command for F2 through F0 */

  bcmf_write_reg(sbus, 0, SDIO_CCCR_IOABORT, 2);

  bcmf_write_reg(sbus, 1, SBSDIO_FUNC1_FRAMECTRL, SFC_RF_TERM);

  /* TODO Wait until the packet has been flushed (device/FIFO stable) */

  if (retry)
    {
      /* Send NAK to retry to read frame */

      bcmf_write_sbregb(sbus,
                  CORE_BUS_REG(sbus->chip->core_base[SDIOD_CORE_ID],
                  tosbmailbox), SMB_NAK);
    }

  return 0;
}

int bcmf_sdpcm_process_header(FAR struct bcmf_sdio_dev_s *sbus,
                              struct bcmf_sdpcm_header *header)
{
  if (header->data_offset < sizeof(struct bcmf_sdpcm_header) ||
      header->data_offset > header->size)
    {
      wlerr("Invalid data offset\n");
      bcmf_sdpcm_rxfail(sbus, false);
      return -ENXIO;
    }

  /* Update tx credits */

  sbus->max_seq = header->credit;

  /* Update flow control status */

  sbus->flow_ctrl = (header->flow_control != 0);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bcmf_sdpcm_readframe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint16_t len;
  uint16_t checksum;
  struct bcmf_sdpcm_header *header;
  struct bcmf_sdpcm_header tmp_hdr;
  struct bcmf_sdio_frame *sframe;
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)priv->bus;

  /* Read the first 4 bytes of sdpcm header
   * to get the length of the following data to be read
   */

  ret = bcmf_transfer_bytes(sbus, false, 2, 0,
                            (uint8_t *)&tmp_hdr,
                            FIRST_WORD_SIZE);
  if (ret != OK)
    {
      wlinfo("Failed to read size\n");
      bcmf_sdpcm_rxfail(sbus, false);
      return -EIO;
    }

  len = tmp_hdr.size;
  checksum = tmp_hdr.checksum;

  /* All zero means no more to read */

  if (!(len | checksum))
    {
      return -ENODATA;
    }

  if (((~len & 0xffff) ^ checksum) || len < sizeof(struct bcmf_sdpcm_header))
    {
      wlerr("Invalid header checksum or len %x %x\n", len, checksum);
      bcmf_sdpcm_rxfail(sbus, false);
      return -EINVAL;
    }

  if (len == FC_UPDATE_PKT_LENGTH)
    {
      /* Flow control update packet with no data */

      ret = bcmf_transfer_bytes(sbus, false, 2, 0,
                                (uint8_t *)&tmp_hdr + FIRST_WORD_SIZE,
                                FC_UPDATE_PKT_LENGTH - FIRST_WORD_SIZE);
      if (ret != OK)
        {
          wlinfo("Failed to read the rest 8 bytes\n");
          bcmf_sdpcm_rxfail(sbus, false);
          return -EIO;
        }

      ret = bcmf_sdpcm_process_header(sbus, &tmp_hdr);

      if (ret != OK)
        {
          wlerr("Error while processing header %d\n", ret);
          return -EINVAL;
        }

      return OK;
    }

  /* Request free frame buffer */

  sframe = bcmf_sdio_allocate_frame(priv, true, false);

  if (sframe == NULL)
    {
      wlinfo("fail alloc\n");

      /* Read out the rest of the header to get the bus credit information */

      ret = bcmf_transfer_bytes(sbus, false, 2, 0,
                                (uint8_t *)&tmp_hdr + FIRST_WORD_SIZE,
                                FC_UPDATE_PKT_LENGTH - FIRST_WORD_SIZE);

      if (ret != OK)
        {
          wlinfo("Failed to read the rest 8 bytes\n");
          bcmf_sdpcm_rxfail(sbus, false);
          return -EIO;
        }

      bcmf_sdpcm_rxfail(sbus, false);

      ret = bcmf_sdpcm_process_header(sbus, &tmp_hdr);

      if (ret != OK)
        {
          wlerr("Error while processing header %d\n", ret);
          return -EINVAL;
        }

      return -EAGAIN;
    }

  header = (struct bcmf_sdpcm_header *)sframe->data;

  /* Read the remaining frame data (the buffer is DMA aligned here) */

  if (len <= FIRST_WORD_SIZE)
    {
      ret = OK;
      goto exit_free_frame;
    }

  ret = bcmf_transfer_bytes(sbus, false, 2, 0,
                           (uint8_t *)header + FIRST_WORD_SIZE,
                           len - FIRST_WORD_SIZE);
  if (ret != OK)
    {
      wlinfo("Failed to read remaining frame data\n");
      ret = -EIO;
      goto exit_abort;
    }

  memcpy(header, &tmp_hdr, FIRST_WORD_SIZE);

  if (len > sframe->header.len)
    {
      wlerr("Frame is too large, cancel %d %d\n", len, sframe->header.len);
      ret = -ENOMEM;
      goto exit_abort;
    }

#if 0
  wlinfo("Receive frame %p %d\n", sframe, len);

  bcmf_hexdump((uint8_t *)header, header->size, (unsigned int)header);
#endif

  /* Process and validate header */

  ret = bcmf_sdpcm_process_header(sbus, header);
  if (ret != OK)
    {
      wlerr("Error while processing header %d\n", ret);
      ret = -EINVAL;
      goto exit_free_frame;
    }

  /* Update frame structure */

  sframe->header.len = header->size;
  sframe->header.data += header->data_offset;

  /* Process received frame content */

  switch (header->channel & 0x0f)
    {
      case SDPCM_CONTROL_CHANNEL:
        ret = bcmf_cdc_process_control_frame(priv, &sframe->header);
        goto exit_free_frame;

      case SDPCM_EVENT_CHANNEL:
        if (header->data_offset == header->size)
          {
            /* Empty event, ignore */

            ret = OK;
          }
        else
          {
            ret = bcmf_bdc_process_event_frame(priv, &sframe->header);
          }

        goto exit_free_frame;

      case SDPCM_DATA_CHANNEL:

        /* Queue frame and notify network layer frame is available */

        if (nxsem_wait_uninterruptible(&sbus->queue_mutex) < 0)
          {
            DEBUGPANIC();
          }

        bcmf_dqueue_push(&sbus->rx_queue, &sframe->list_entry);
        nxsem_post(&sbus->queue_mutex);

        bcmf_netdev_notify_rx(priv);

        /* Upper layer have to free all received frames */

        ret = OK;
        break;

      default:
        wlerr("Got unexpected message type %d\n", header->channel);
        ret = -EINVAL;
        goto exit_free_frame;
    }

  return ret;

exit_abort:
  bcmf_sdpcm_rxfail(sbus, false);
exit_free_frame:
  bcmf_sdio_free_frame(priv, sframe);
  return ret;
}

int bcmf_sdpcm_sendframe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  bool is_txframe;
  dq_entry_t *entry;
  struct bcmf_sdio_frame *sframe;
  struct bcmf_sdpcm_header *header;
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)priv->bus;

  if (sbus->tx_queue.tail == NULL)
    {
      /* No more frames to send */

      return -ENODATA;
    }

  if (sbus->flow_ctrl)
    {
      return -EAGAIN;
    }

  if (sbus->tx_seq == sbus->max_seq)
    {
      /* TODO handle this case */

      wlinfo("No credit to send frame\n");
      return -EAGAIN;
    }

  if (nxsem_wait_uninterruptible(&sbus->queue_mutex) < 0)
    {
      DEBUGPANIC();
    }

  entry = sbus->tx_queue.tail;
  sframe = container_of(entry, struct bcmf_sdio_frame, list_entry);
  header = (struct bcmf_sdpcm_header *)sframe->header.base;

  /* Set frame sequence id */

  header->sequence = sbus->tx_seq++;

#if 0
  wlinfo("Send frame %p\n", sframe);

  bcmf_hexdump(sframe->header.base, sframe->header.len,
               (unsigned long)sframe->header.base);
#endif

  /* Write the frame data (the buffer is DMA aligned here) */

  ret = bcmf_transfer_bytes(sbus, true, 2, 0,
                            sframe->header.base,
                            sframe->header.len);
  if (ret != OK)
    {
      /* TODO handle retry count and remove frame from queue + abort TX */

      wlinfo("fail send frame %d\n", ret);
      ret = -EIO;
      goto exit_abort;
    }

  /* Frame sent, remove it from queue */

  bcmf_dqueue_pop_tail(&sbus->tx_queue);
  nxsem_post(&sbus->queue_mutex);
  is_txframe = sframe->tx;

  /* Free frame buffer */

  bcmf_sdio_free_frame(priv, sframe);

  if (is_txframe)
    {
      /* Notify upper layer at least one TX buffer is available */

      bcmf_netdev_notify_tx(priv);
    }

  return OK;

exit_abort:
#if 0
  bcmf_sdpcm_txfail(sbus, false);
#endif

  nxsem_post(&sbus->queue_mutex);
  return ret;
}

int bcmf_sdpcm_queue_frame(FAR struct bcmf_dev_s *priv,
                           struct bcmf_frame_s *frame, bool control)
{
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)priv->bus;
  struct bcmf_sdio_frame *sframe = (struct bcmf_sdio_frame *)frame;
  struct bcmf_sdpcm_header *header =
    (struct bcmf_sdpcm_header *)sframe->data;
  int semcount;

  /* Prepare sw header */

  memset(header, 0, sizeof(struct bcmf_sdpcm_header));
  header->size = frame->len;
  header->checksum = ~header->size;
  header->data_offset = (uint8_t)(frame->data - frame->base);

  if (control)
    {
      header->channel = SDPCM_CONTROL_CHANNEL;
    }
  else
    {
      header->channel = SDPCM_DATA_CHANNEL;
    }

  /* Add frame in tx queue */

  if (nxsem_wait_uninterruptible(&sbus->queue_mutex) < 0)
    {
      DEBUGPANIC();
    }

  bcmf_dqueue_push(&sbus->tx_queue, &sframe->list_entry);

  nxsem_post(&sbus->queue_mutex);

  /* Notify bcmf thread tx frame is ready */

  nxsem_get_value(&sbus->thread_signal, &semcount);
  if (semcount < 1)
    {
      nxsem_post(&sbus->thread_signal);
    }

  return OK;
}

struct bcmf_frame_s *bcmf_sdpcm_alloc_frame(FAR struct bcmf_dev_s *priv,
                                            unsigned int len, bool block,
                                            bool control)
{
  struct bcmf_sdio_frame *sframe;
  unsigned int header_len = sizeof(struct bcmf_sdpcm_header);

  if (!control)
    {
      header_len += 2; /* Data frames need alignment padding */
    }

  if (len + header_len > MAX_NETDEV_PKTSIZE + HEADER_SIZE)
    {
      wlerr("Invalid size %d\n", len);
      return NULL;
    }

  /* Allocate a frame for RX in case of control frame */

  sframe = bcmf_sdio_allocate_frame(priv, block, !control);

  if (sframe == NULL)
    {
      return NULL;
    }

  sframe->header.len   = header_len + len;
  sframe->header.data += header_len;
  return &sframe->header;
}

void bcmf_sdpcm_free_frame(FAR struct bcmf_dev_s *priv,
                           struct bcmf_frame_s *frame)
{
  bcmf_sdio_free_frame(priv, (struct bcmf_sdio_frame *)frame);
}

struct bcmf_frame_s *bcmf_sdpcm_get_rx_frame(FAR struct bcmf_dev_s *priv)
{
  dq_entry_t *entry;
  struct bcmf_sdio_frame *sframe;
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)priv->bus;

  if (nxsem_wait_uninterruptible(&sbus->queue_mutex) < 0)
    {
      DEBUGPANIC();
    }

  entry = bcmf_dqueue_pop_tail(&sbus->rx_queue);

  nxsem_post(&sbus->queue_mutex);

  if (entry == NULL)
    {
      return NULL;
    }

  sframe = container_of(entry, struct bcmf_sdio_frame, list_entry);
  return &sframe->header;
}
