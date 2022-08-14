/****************************************************************************
 * drivers/wireless/ieee80211/bcm43xxx/bcmf_cdc.c
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

#include <nuttx/arch.h>
#include <inttypes.h>
#include <stddef.h>
#include <string.h>

#include "bcmf_driver.h"
#include "bcmf_ioctl.h"
#include "bcmf_utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Control header flags */

#define BCMF_CONTROL_ERROR    0x01      /* Command failure flag */
#define BCMF_CONTROL_SET      0x02      /* Command type: SET = 1, GET = 0 */
#define BCMF_CONTROL_INTERFACE_SHIFT 12
#define BCMF_CONTROL_REQID_SHIFT     16

#define BCMF_CONTROL_TIMEOUT_MS 2000

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct bcmf_cdc_header
{
  uint32_t cmd;    /* Command to be sent */
  uint32_t len;    /* Size of command data */
  uint32_t flags;  /* cdc request flags, see above */
  uint32_t status; /* Returned status code from chip */
} end_packed_struct;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static struct bcmf_frame_s *bcmf_cdc_allocate_frame(
                                FAR struct bcmf_dev_s *priv, char *name,
                                uint8_t *data, uint32_t len);

static int bcmf_cdc_sendframe(FAR struct bcmf_dev_s *priv, uint32_t cmd,
                         int ifidx, bool set, struct bcmf_frame_s *frame);

static int bcmf_cdc_control_request(FAR struct bcmf_dev_s *priv,
                               uint32_t ifidx, bool set, uint32_t cmd,
                               char *name, uint8_t *data, uint32_t *len);

static int bcmf_cdc_control_request_unsafe(FAR struct bcmf_dev_s *priv,
                               uint32_t ifidx, bool set, uint32_t cmd,
                               char *name, uint8_t *data, uint32_t *len);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

struct bcmf_frame_s *bcmf_cdc_allocate_frame(FAR struct bcmf_dev_s *priv,
                                             char *name, uint8_t *data,
                                             uint32_t len)
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
  struct bcmf_cdc_header *header =
                  (struct bcmf_cdc_header *)frame->data;
  uint32_t cdc_data_len;

  /* Setup control frame header */

  cdc_data_len = frame->len - (uint32_t)(frame->data - frame->base);

  header->cmd = cmd;
  header->len = cdc_data_len - sizeof(struct bcmf_cdc_header);
  header->status = 0;
  header->flags = ++priv->control_reqid << BCMF_CONTROL_REQID_SHIFT;
  header->flags |= ifidx << BCMF_CONTROL_INTERFACE_SHIFT;

  if (set)
    {
      header->flags |= BCMF_CONTROL_SET;
    }

  /* Send frame */

  return priv->bus->txframe(priv, frame, true);
}

int bcmf_cdc_control_request(FAR struct bcmf_dev_s *priv,
                               uint32_t ifidx, bool set, uint32_t cmd,
                               char *name, uint8_t *data, uint32_t *len)
{
  int ret;

  /* Take device control mutex */

  if ((ret = nxsem_wait_uninterruptible(&priv->control_mutex)) < 0)
    {
      return ret;
    }

  ret = bcmf_cdc_control_request_unsafe(priv, ifidx, set, cmd,
                                        name, data, len);

  nxsem_post(&priv->control_mutex);
  return ret;
}

int bcmf_cdc_control_request_unsafe(FAR struct bcmf_dev_s *priv,
                               uint32_t ifidx, bool set, uint32_t cmd,
                               char *name, uint8_t *data, uint32_t *len)
{
  int ret;
  struct bcmf_frame_s *frame;
  uint32_t out_len = *len;

  *len = 0;

  /* Prepare control frame */

  frame = bcmf_cdc_allocate_frame(priv, name, data, out_len);
  if (!frame)
    {
      wlerr("Cannot allocate cdc frame\n");
      return -ENOMEM;
    }

#ifdef CONFIG_DEBUG_WIRELESS_INFO
  if (cmd == WLC_SET_VAR  ||  cmd == WLC_GET_VAR)
    {
      wlinfo(">>> Sending control %d %d 0x%08lX [%d] %s %s \n",
             ifidx,
             set,
             cmd,
             out_len,
             set ? "set" : "get",
             name);
    }
  else
    {
      wlinfo(">>> Sending control %d %d 0x%08lX [%d] %s cmd: %d\n",
             ifidx,
             set,
             cmd,
             out_len,
             set ? "set" : "get",
             cmd);
    }
#endif

  /* Setup buffer to store response */

  priv->control_rxdata = set ? NULL : data;
  priv->control_rxdata_len = out_len;

  /* Send control frame. iovar buffer is freed when sent */

  ret = bcmf_cdc_sendframe(priv, cmd, ifidx, set, frame);
  if (ret != OK)
    {
      /* Free allocated iovar buffer */

      wlerr("cdc send frame failed: %d\n", ret);
      priv->bus->free_frame(priv, frame);
      return ret;
    }

  ret = nxsem_tickwait_uninterruptible(&priv->control_timeout,
                                       MSEC2TICK(BCMF_CONTROL_TIMEOUT_MS));
  if (ret < 0)
    {
      wlerr("Error while waiting for control response %d\n", ret);
      return ret;
    }

  *len = priv->control_rxdata_len;

  /* Check frame status */

  if (priv->control_status != 0)
    {
      wlerr("Invalid cdc status 0x%" PRIx32 "\n", priv->control_status);
      return -EINVAL;
    }

  return OK;
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

int bcmf_cdc_iovar_request_unsafe(FAR struct bcmf_dev_s *priv,
                             uint32_t ifidx, bool set, char *name,
                             uint8_t *data, uint32_t *len)
{
  return bcmf_cdc_control_request_unsafe(priv, ifidx, set,
                                  set ? WLC_SET_VAR : WLC_GET_VAR, name,
                                  data, len);
}

int bcmf_cdc_ioctl(FAR struct bcmf_dev_s *priv,
                     uint32_t ifidx, bool set, uint32_t cmd,
                     uint8_t *data, uint32_t *len)
{
  return bcmf_cdc_control_request(priv, ifidx, set, cmd, NULL, data, len);
}

/****************************************************************************
 * Name: bcmf_cdc_process_control_frame
 ****************************************************************************/

int bcmf_cdc_process_control_frame(FAR struct bcmf_dev_s *priv,
                   struct bcmf_frame_s *frame)
{
  unsigned int data_size;
  struct bcmf_cdc_header *cdc_header;

  /* Check frame */

  data_size = frame->len - (unsigned int)(frame->data - frame->base);

  if (data_size < sizeof(struct bcmf_cdc_header))
    {
      wlerr("Control frame too small\n");
      return -EINVAL;
    }

  cdc_header = (struct bcmf_cdc_header *)frame->data;

  if (data_size < cdc_header->len ||
      data_size < sizeof(struct bcmf_cdc_header) + cdc_header->len)
    {
      wlerr("Invalid control frame size\n");
      return -EINVAL;
    }

  /* TODO check interface ? */

  if (cdc_header->flags >> BCMF_CONTROL_REQID_SHIFT == priv->control_reqid)
    {
      /* Expected frame received, send it back to user */

      priv->control_status = cdc_header->status;

      if (priv->control_rxdata)
        {
          if (priv->control_rxdata_len > cdc_header->len)
            {
              wlerr("Not enough data %d %" PRId32 "\n",
                    priv->control_rxdata_len, cdc_header->len);
              priv->control_rxdata_len = cdc_header->len;
            }

          memcpy(priv->control_rxdata, (uint8_t *)&cdc_header[1],
                 priv->control_rxdata_len);
        }

      nxsem_post(&priv->control_timeout);
      return OK;
    }

  wlinfo("Got unexpected control frame\n");
  return -EINVAL;
}
