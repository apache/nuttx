/****************************************************************************
 * drivers/wireless/ieee80211/bcmf_cdc.c
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
#include <semaphore.h>

#include "bcmf_driver.h"
#include "bcmf_ioctl.h"
#include "bcmf_utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CDC flag definitions */
#define CDC_DCMD_ERROR    0x01       /* 1=cmd failed */
#define CDC_DCMD_SET      0x02       /* 0=get, 1=set cmd */
#define CDC_DCMD_IF_MASK  0xF000     /* I/F index */
#define CDC_DCMD_IF_SHIFT 12
#define CDC_DCMD_ID_MASK  0xFFFF0000 /* id an cmd pairing */
#define CDC_DCMD_ID_SHIFT 16         /* ID Mask shift bits */
#define CDC_DCMD_ID(flags)  \
  (((flags) & CDC_DCMD_ID_MASK) >> CDC_DCMD_ID_SHIFT)

#define CDC_CONTROL_TIMEOUT_MS 1000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bcmf_cdc_header {
    uint32_t cmd;    /* dongle command value */
    uint32_t len;    /* lower 16: output buflen;
                      * upper 16: input buflen (excludes header) */
    uint32_t flags;  /* flag defns given below */
    uint32_t status; /* status code returned from the device */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static struct bcmf_frame_s* bcmf_cdc_allocate_frame(
                                FAR struct bcmf_dev_s *priv, char *name,
                                uint8_t *data, uint32_t len);

static int bcmf_cdc_sendframe(FAR struct bcmf_dev_s *priv, uint32_t cmd,
                         int ifidx, bool set, struct bcmf_frame_s *frame);

static int bcmf_cdc_control_request(FAR struct bcmf_dev_s *priv,
                               uint32_t ifidx, bool set, uint32_t cmd,
                               char *name, uint8_t *data, uint32_t *len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

struct bcmf_frame_s* bcmf_cdc_allocate_frame(FAR struct bcmf_dev_s *priv,
                                char *name, uint8_t *data, uint32_t len)
{
  uint32_t data_len;
  uint16_t name_len;
  struct bcmf_frame_s *frame;

  if (name)
    {
      name_len = strlen(name) + 1;
    }
  else
    {
      name_len = 0;
    }

  if (data)
    {
      data_len = len;
    }
  else
    {
      data_len = 0;
    }

  if (data_len + name_len + sizeof(struct bcmf_cdc_header) < data_len)
    {
      /* Integer overflow */

      return NULL;
    }

  /* Allocate control frame */

  frame = priv->bus->allocate_frame(priv,
                sizeof(struct bcmf_cdc_header) + data_len + name_len,
                true, true);
  
  if (!frame)
    {
      return NULL;
    }

  /* Copy name string and data */

  memcpy(frame->data + sizeof(struct bcmf_cdc_header), name, name_len);
  memcpy(frame->data + sizeof(struct bcmf_cdc_header)
                     + name_len, data, data_len);

  return frame;
}

int bcmf_cdc_sendframe(FAR struct bcmf_dev_s *priv, uint32_t cmd,
                         int ifidx, bool set, struct bcmf_frame_s *frame)
{
  struct bcmf_cdc_header* header =
                  (struct bcmf_cdc_header*)frame->data;

  /* Setup cdc_dcmd header */

  uint32_t cdc_data_len = frame->len - (uint32_t)(frame->data-frame->base);
  header->cmd = cmd;
  header->len = cdc_data_len-sizeof(struct bcmf_cdc_header);
  header->status = 0;
  header->flags = ++priv->control_reqid << CDC_DCMD_ID_SHIFT;
  header->flags |= ifidx << CDC_DCMD_IF_SHIFT;

  if (set)
  {
    header->flags |= CDC_DCMD_SET;
  }

  /* Queue frame */

  return priv->bus->txframe(priv, frame);
}

int bcmf_cdc_control_request(FAR struct bcmf_dev_s *priv,
                               uint32_t ifidx, bool set, uint32_t cmd,
                               char *name, uint8_t *data, uint32_t *len)
{
  int ret;
  struct bcmf_frame_s *frame;
  uint32_t out_len = *len;

  *len = 0;

  /* Take device control mutex */

  if ((ret = sem_wait(&priv->control_mutex)) != OK)
   {
      return ret;
   }

  /* Prepare control frame */

  frame = bcmf_cdc_allocate_frame(priv, name, data, out_len);
  if (!frame)
    {
      _err("Cannot allocate cdc frame\n");
      ret = -ENOMEM;
      goto exit_sem_post;
    }

  /* Setup buffer to store response */

  priv->control_rxdata = set ? NULL : data;
  priv->control_rxdata_len = out_len;

  /* Send control frame. iovar buffer is freed when sent */

  ret = bcmf_cdc_sendframe(priv, cmd, ifidx, set, frame);
  if (ret != OK)
    {
      // TODO free allocated iovar buffer here
      goto exit_sem_post;
    }

  ret = bcmf_sem_wait(&priv->control_timeout, CDC_CONTROL_TIMEOUT_MS);
  if (ret != OK)
    {
      _err("Error while waiting for control response %d\n", ret);
      goto exit_sem_post;
    }

  *len = priv->control_rxdata_len;

  /* Check frame status */

  if (priv->control_status != 0)
    {
      _err("Invalid cdc status 0x%x\n", priv->control_status);
      ret = -EINVAL;
    }

exit_sem_post:
  sem_post(&priv->control_mutex);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bcmf_cdc_iovar_request(FAR struct bcmf_dev_s *priv,
                             uint32_t ifidx, bool set, char *name,
                             uint8_t *data, uint32_t *len)
{
  return bcmf_cdc_control_request(priv, ifidx, set,
                                  set ? WLC_SET_VAR : WLC_GET_VAR, name,
                                  data, len);
}

int bcmf_cdc_ioctl(FAR struct bcmf_dev_s *priv,
                     uint32_t ifidx, bool set, uint32_t cmd,
                     uint8_t *data, uint32_t *len)
{
  return bcmf_cdc_control_request(priv, ifidx, set, cmd, NULL, data, len);
}

int bcmf_cdc_process_control_frame(FAR struct bcmf_dev_s *priv,
                   struct bcmf_frame_s *frame)
{
  unsigned int data_size;
  struct bcmf_cdc_header *cdc_header;

  /* Check frame */

  data_size = frame->len - (unsigned int)(frame->data - frame->base);

  if (data_size < sizeof(struct bcmf_cdc_header))
    {
      _err("Control frame too small\n");
      return -EINVAL;
    }

  cdc_header = (struct bcmf_cdc_header*)frame->data;

  if (data_size < cdc_header->len ||
      data_size < sizeof(struct bcmf_cdc_header) + cdc_header->len)
    {
      _err("Invalid control frame size\n");
      return -EINVAL;
    }

  // TODO check interface ?

  if (cdc_header->flags >> CDC_DCMD_ID_SHIFT == priv->control_reqid)
    {
      /* Expected frame received, send it back to user */

      priv->control_status = cdc_header->status;

      if (priv->control_rxdata)
        {
          if (priv->control_rxdata_len > cdc_header->len)
            {
              _err("Not enough data %d %d\n",
                      priv->control_rxdata_len, cdc_header->len);
              priv->control_rxdata_len = cdc_header->len;
            }
          memcpy(priv->control_rxdata, (uint8_t*)&cdc_header[1],
                                       priv->control_rxdata_len);
        }

      sem_post(&priv->control_timeout);
      return OK;
    }

  _info("Got unexpected control frame\n");
  return -EINVAL;
}

int bcmf_cdc_process_event_frame(FAR struct bcmf_dev_s *priv,
                   struct bcmf_frame_s *frame)
{
  _info("Event message\n");
  bcmf_hexdump(frame->base, frame->len, (unsigned long)frame->base);
  return OK;
}

int bcmf_cdc_process_data_frame(FAR struct bcmf_dev_s *priv,
                   struct bcmf_frame_s *frame)
{
  _info("Data message\n");
  bcmf_hexdump(frame->base, frame->len, (unsigned long)frame->base);
  return OK;
}