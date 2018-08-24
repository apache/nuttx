/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_sdpcm.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Simon Piriou <spiriou31@gmail.com>
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
#include <nuttx/compiler.h>

#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <stddef.h>
#include <string.h>
#include <queue.h>
#include <semaphore.h>

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

struct __attribute__((packed)) bcmf_sdpcm_header {
    uint16_t size;
    uint16_t checksum;
    uint8_t  sequence;
    uint8_t  channel;
    uint8_t  next_length;
    uint8_t  data_offset;
    uint8_t  flow_control;
    uint8_t  credit;
    uint16_t padding;
};

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

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bcmf_sdpcm_readframe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint16_t len, checksum;
  struct bcmf_sdpcm_header *header;
  struct bcmf_sdio_frame *sframe;
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)priv->bus;

  /* Request free frame buffer */

  sframe = bcmf_sdio_allocate_frame(priv, false, false);

  if (sframe == NULL)
    {
      wlinfo("fail alloc\n");
      return -EAGAIN;
    }

  header = (struct bcmf_sdpcm_header *)sframe->data;

  /* Read header */

  ret = bcmf_transfer_bytes(sbus, false, 2, 0, (uint8_t *)header, 4);
  if (ret != OK)
    {
      wlinfo("failread size\n");
      ret = -EIO;
      goto exit_abort;
    }

  len = header->size;
  checksum = header->checksum;

  /* All zero means no more to read */

  if (!(len | checksum))
    {
      ret = -ENODATA;
      goto exit_free_frame;
    }

  if (((~len & 0xffff) ^ checksum) || len < sizeof(struct bcmf_sdpcm_header))
    {
      wlerr("Invalid header checksum or len %x %x\n", len, checksum);
      ret = -EINVAL;
      goto exit_abort;
    }

  if (len > sframe->header.len)
    {
      wlerr("Frame is too large, cancel %d %d\n", len, sframe->header.len);
      ret = -ENOMEM;
      goto exit_abort;
    }

  /* Read remaining frame data */

  ret = bcmf_transfer_bytes(sbus, false, 2, 0, (uint8_t *)header + 4, len - 4);
  if (ret != OK)
    {
      ret = -EIO;
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

        if (nxsem_wait(&sbus->queue_mutex) < 0)
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

  if (sbus->tx_seq == sbus->max_seq)
    {
      /* TODO handle this case */

      wlerr("No credit to send frame\n");
      return -EAGAIN;
    }


  if (nxsem_wait(&sbus->queue_mutex) < 0)
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

  ret = bcmf_transfer_bytes(sbus, true, 2, 0, sframe->header.base,
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

      bcmf_netdev_notify_tx_done(priv);
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
  struct bcmf_sdpcm_header *header = (struct bcmf_sdpcm_header *)sframe->data;

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

  if (nxsem_wait(&sbus->queue_mutex) < 0)
    {
      DEBUGPANIC();
    }

  bcmf_dqueue_push(&sbus->tx_queue, &sframe->list_entry);

  nxsem_post(&sbus->queue_mutex);

  /* Notify bcmf thread tx frame is ready */

  nxsem_post(&sbus->thread_signal);

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

  if (len + header_len > MAX_NETDEV_PKTSIZE + HEADER_SIZE ||
      len > len + header_len)
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

  sframe->header.len = header_len + len;
  sframe->header.data += header_len;
  return &sframe->header;
}


void bcmf_sdpcm_free_frame(FAR struct bcmf_dev_s *priv,
                     struct bcmf_frame_s *frame)
{
  return bcmf_sdio_free_frame(priv, (struct bcmf_sdio_frame *)frame);
}

struct bcmf_frame_s *bcmf_sdpcm_get_rx_frame(FAR struct bcmf_dev_s *priv)
{
  dq_entry_t *entry;
  struct bcmf_sdio_frame *sframe;
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s *)priv->bus;

  if (nxsem_wait(&sbus->queue_mutex) < 0)
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
