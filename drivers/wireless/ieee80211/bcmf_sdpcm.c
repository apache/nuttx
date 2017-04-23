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

#include <nuttx/arch.h>
#include <stddef.h>
#include <string.h>
#include <queue.h>
#include <semaphore.h>

#include "bcmf_sdio.h"
#include "bcmf_core.h"
#include "bcmf_sdpcm.h"
#include "bcmf_cdc.h"
#include "bcmf_utils.h"

#include "bcmf_sdio_regs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SDA_FRAMECTRL */
#define SFC_RF_TERM  (1 << 0)  /* Read Frame Terminate */
#define SFC_WF_TERM  (1 << 1)  /* Write Frame Terminate */
#define SFC_CRC4WOOS (1 << 2)  /* CRC error for write out of sync */
#define SFC_ABORTALL (1 << 3)  /* Abort all in-progress frames */

/* tosbmailbox bits corresponding to intstatus bits */
#define SMB_NAK     (1 << 0)  /* Frame NAK */
#define SMB_INT_ACK (1 << 1)  /* Host Interrupt ACK */
#define SMB_USE_OOB (1 << 2)  /* Use OOB Wakeup */
#define SMB_DEV_INT (1 << 3)  /* Miscellaneous Interrupt */

#define SDPCM_CONTROL_CHANNEL 0  /* Control */
#define SDPCM_EVENT_CHANNEL   1  /* Asyc Event Indication */
#define SDPCM_DATA_CHANNEL    2  /* Data Xmit/Recv */
#define SDPCM_GLOM_CHANNEL    3  /* Coalesced packets */
#define SDPCM_TEST_CHANNEL    15 /* Test/debug packets */

#define container_of(ptr, type, member) \
        (type *)( (uint8_t *)(ptr) - offsetof(type,member) )

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bcmf_sdpcm_header {
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

struct bcmf_sdpcm_frame {
    struct bcmf_frame_s          frame_header;
    dq_entry_t                   list_entry;
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

  /* Send NAK to retry to read frame */
  if (retry)
    {
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
      _err("Invalid data offset\n");
      bcmf_sdpcm_rxfail(sbus, false);
      return -ENXIO;
    }

  /* Update tx credits */

  // _info("update credit %x %x %x\n", header->credit,
  //                                   sbus->tx_seq, sbus->max_seq);

  if (header->credit - sbus->tx_seq > 0x40)
    {
      _err("seq %d: max tx seq number error\n", sbus->tx_seq);
      sbus->max_seq = sbus->tx_seq + 2;
    }
  else
    {
      sbus->max_seq = header->credit;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

// FIXME remove
uint8_t tmp_buffer[1024];
int bcmf_sdpcm_readframe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  uint16_t len, checksum;
  struct bcmf_sdpcm_header *header;
  struct bcmf_sdpcm_frame *sframe;
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s*)priv->bus;

  /* TODO request free frame buffer */

  sframe = (struct bcmf_sdpcm_frame*)tmp_buffer;
  header = (struct bcmf_sdpcm_header*)&sframe[1];

  /* Read header */

  ret = bcmf_transfer_bytes(sbus, false, 2, 0, (uint8_t*)header, 4);
  if (ret != OK)
    {
      _info("failread size\n");
      ret = -EIO;
      goto exit_abort;
    }

  len = header->size;
  checksum = header->checksum;

  /* All zero means no more to read */

  if (!(len | checksum))
    {
      return -ENODATA;
    }

  if (((~len & 0xffff) ^ checksum) || len < sizeof(struct bcmf_sdpcm_header))
    {
      _err("Invalid header checksum or len %x %x\n", len, checksum);
      ret = -EINVAL;
      goto exit_abort;
    }

  // FIXME define for size
  if (len > sizeof(tmp_buffer))
    {
      _err("Frame is too large, cancel %d\n", len);
      ret = -ENOMEM;
      goto exit_abort;
    }

  /* Read remaining frame data */
  // TODO allocate buffer

  ret = bcmf_transfer_bytes(sbus, false, 2, 0, (uint8_t*)header+4, len - 4);
  if (ret != OK)
    {
      ret = -EIO;
      goto exit_free_abort;
    }

  // _info("Receive frame\n");
  // bcmf_hexdump((uint8_t*)header, header->size, (unsigned int)header);

  /* Process and validate header */

  ret = bcmf_sdpcm_process_header(sbus, header);
  if (ret != OK)
    {
      _err("Error while processing header %d\n", ret);
      ret = -EINVAL;
      goto exit_free_frame;
    }

  /* Setup new frame structure */  

  sframe->frame_header.len = header->size;
  sframe->frame_header.data = (uint8_t*)header + header->data_offset;
  sframe->frame_header.base = (uint8_t*)header;

  /* Process received frame content */

  switch (header->channel & 0x0f)
    {
      case SDPCM_CONTROL_CHANNEL:
        ret = bcmf_cdc_process_control_frame(priv, &sframe->frame_header);
        break;

      case SDPCM_EVENT_CHANNEL:
        ret = bcmf_cdc_process_event_frame(priv, &sframe->frame_header);
        break;

      case SDPCM_DATA_CHANNEL:
        ret = bcmf_cdc_process_data_frame(priv, &sframe->frame_header);
        break;

      default:
        _err("Got unexpected message type %d\n", header->channel);
        ret = OK;
    }

exit_free_frame:
  // TODO free frame buffer
  return ret;

exit_free_abort:
  // TODO free frame buffer
exit_abort:
  bcmf_sdpcm_rxfail(sbus, false);
  return ret;
}

int bcmf_sdpcm_sendframe(FAR struct bcmf_dev_s *priv)
{
  int ret;
  struct bcmf_sdpcm_frame *sframe;
  struct bcmf_sdpcm_header *header;
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s*)priv->bus;

  if (sbus->tx_queue.tail == NULL)
    {
      /* No more frames to send */

      return -ENODATA;
    }

  if (sbus->tx_seq == sbus->max_seq)
    {
      // TODO handle this case
      _err("No credit to send frame\n");
      return -EAGAIN;
    }


  if ((ret = sem_wait(&sbus->tx_queue_mutex)) != OK)
    {
      return ret;
    }

  sframe = container_of(sbus->tx_queue.tail,
                        struct bcmf_sdpcm_frame, list_entry);
  header = (struct bcmf_sdpcm_header*)sframe->frame_header.base;

  /* Set frame sequence id */

  header->sequence = sbus->tx_seq++;

  // _info("Send frame\n");
  // bcmf_hexdump(sframe->frame_header.base, sframe->frame_header.len,
  //              (unsigned long)sframe->frame_header.base);

  ret = bcmf_transfer_bytes(sbus, true, 2, 0, sframe->frame_header.base,
                            sframe->frame_header.len);
  if (ret != OK)
    {
      _info("fail send frame %d\n", ret);
      ret = -EIO;
      goto exit_abort;
      // TODO handle retry count and remove frame from queue + abort TX
    }

  /* Frame sent, remove it from queue */

  if (sbus->tx_queue.head == &sframe->list_entry)
    {
      /* List is empty */

      sbus->tx_queue.head = NULL;
      sbus->tx_queue.tail = NULL;
    }
  else
    {
      sbus->tx_queue.tail = sframe->list_entry.blink;
      sframe->list_entry.blink->flink = sbus->tx_queue.head;
    }

  /* TODO free frame buffer */

  goto exit_post_sem;

exit_abort:
  // bcmf_sdpcm_txfail(sbus, false);
exit_post_sem:
  sem_post(&sbus->tx_queue_mutex);
  return ret;
}

// FIXME remove
uint8_t tmp_buffer2[512];
struct bcmf_frame_s* bcmf_sdpcm_allocate_frame(FAR struct bcmf_dev_s *priv,
                                   unsigned int len, bool control, bool block)
{
  unsigned int frame_len;

  /* Integer overflow check */

  if (len > 512)
    {
      return NULL;
    }

  frame_len = len + sizeof(struct bcmf_sdpcm_frame)
                  + sizeof(struct bcmf_sdpcm_header);
  if (!control)
    {
      /* Data frames needs 2 bytes padding */

      frame_len += 2;
    }

  if (frame_len > 512)
    {
      return NULL;
    }

  // FIXME allocate buffer and use max_size instead of 512
  // allocate buffer len + sizeof(struct bcmf_sdpcm_frame)

  struct bcmf_sdpcm_frame *sframe = (struct bcmf_sdpcm_frame*)tmp_buffer2;
  struct bcmf_sdpcm_header *header = (struct bcmf_sdpcm_header*)&sframe[1];

  /* Prepare sw header */

  memset(header, 0, sizeof(struct bcmf_sdpcm_header));
  header->size = frame_len - sizeof(struct bcmf_sdpcm_frame);
  header->checksum = ~header->size;

  if (control)
    {
      header->channel = SDPCM_CONTROL_CHANNEL;
      header->data_offset = sizeof(struct bcmf_sdpcm_header);
    }
  else
    {
      header->channel = SDPCM_DATA_CHANNEL;
      header->data_offset = sizeof(struct bcmf_sdpcm_header)+2;
    }

  sframe->frame_header.len = header->size;
  sframe->frame_header.base = (uint8_t*)header;
  sframe->frame_header.data = (uint8_t*)header + header->data_offset;

  return &sframe->frame_header;
}

int bcmf_sdpcm_queue_frame(FAR struct bcmf_dev_s *priv,
                           struct bcmf_frame_s *frame)
{
  int ret;
  FAR struct bcmf_sdio_dev_s *sbus = (FAR struct bcmf_sdio_dev_s*)priv->bus;
  struct bcmf_sdpcm_frame *sframe = (struct bcmf_sdpcm_frame*)frame;

  /* Add frame in tx queue */

  if ((ret = sem_wait(&sbus->tx_queue_mutex)) != OK)
    {
      return ret;
    }

  if (sbus->tx_queue.head == NULL)
    {
      /* List is empty */

      sbus->tx_queue.head = &sframe->list_entry;
      sbus->tx_queue.tail = &sframe->list_entry;

      sframe->list_entry.flink = &sframe->list_entry;
      sframe->list_entry.blink = &sframe->list_entry;
    }
  else
    {
      /* Insert entry at list head */

      sframe->list_entry.flink = sbus->tx_queue.head;
      sframe->list_entry.blink = sbus->tx_queue.tail;

      sbus->tx_queue.head->blink = &sframe->list_entry;
      sbus->tx_queue.head = &sframe->list_entry;
    }

  sem_post(&sbus->tx_queue_mutex);

  /* Notify bcmf thread tx frame is ready */

  sem_post(&sbus->thread_signal);

  return OK;
}