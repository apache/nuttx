/****************************************************************************
 * net/bt/audio/bap_iso.c
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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

/* bap_iso.c - BAP ISO handling */

/* Copyright (c) 2022 Codecoup
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/config.h>
#include <nuttx/net/bluetooth/audio/audio.h>
#include <nuttx/net/bluetooth/audio/bap.h>
#include <nuttx/net/bluetooth/hci_types.h>
#include <nuttx/net/bluetooth/iso.h>
#include <nuttx/net/bluetooth/compat/logging/log.h>
#include <assert.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>

#include "bap_iso.h"
#include "audio_internal.h"
#include "bap_endpoint.h"
#include "bap_internal.h"

LOG_MODULE_REGISTER(bt_bap_iso, CONFIG_BT_BAP_ISO_LOG_LEVEL);

/* TODO: Optimize the ISO_POOL_SIZE */
#define ISO_POOL_SIZE CONFIG_BT_ISO_MAX_CHAN

static struct bt_bap_iso iso_pool[ISO_POOL_SIZE];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct bt_bap_iso *bt_bap_iso_new(void)
{
  struct bt_bap_iso *iso = NULL;

  for (size_t i = 0U; i < ARRAY_SIZE(iso_pool); i++)
    {
      if (atomic_cmpxchg_value(&iso_pool[i].ref, 0, 1))
        {
          iso = &iso_pool[i];
          break;
        }
    }

  if (!iso)
    {
      return NULL;
    }

  (void)memset(iso, 0, offsetof(struct bt_bap_iso, ref));

  return iso;
}

struct bt_bap_iso *bt_bap_iso_ref(struct bt_bap_iso *iso)
{
  int32_t old;

  DEBUGASSERT(iso != NULL);

  /* Reference counter must be checked to avoid incrementing ref from
   * zero, then we should return NULL instead. Loop on clear-and-set in case
   * someone has modified the reference count since the read, and start over
   * again when that happens.
   */

  do
    {
      old = atomic_read(&iso->ref);

      if (!old)
        {
          return NULL;
        }
    }
  while (!atomic_cmpxchg_value(&iso->ref, old, old + 1));

  return iso;
}

void bt_bap_iso_unref(struct bt_bap_iso *iso)
{
  int32_t old;

  DEBUGASSERT(iso != NULL);

  old = atomic_sub(&iso->ref, 1);

  DEBUGASSERT(old > 0);
}

void bt_bap_iso_foreach(bt_bap_iso_func_t func, void *user_data)
{
  for (size_t i = 0U; i < ARRAY_SIZE(iso_pool); i++)
    {
      struct bt_bap_iso *iso = bt_bap_iso_ref(&iso_pool[i]);
      bool iter;

      if (!iso)
        {
          continue;
        }

      iter = func(iso, user_data);
      bt_bap_iso_unref(iso);

      if (!iter)
        {
          return;
        }
    }
}

struct bt_bap_iso_find_param
{
  struct bt_bap_iso *iso;
  bt_bap_iso_func_t func;
  void *user_data;
};

static bool bt_bap_iso_find_cb(struct bt_bap_iso *iso, void *user_data)
{
  struct bt_bap_iso_find_param *param = user_data;
  bool found;

  found = param->func(iso, param->user_data);
  if (found)
    {
      param->iso = bt_bap_iso_ref(iso);
    }

  return !found;
}

struct bt_bap_iso *bt_bap_iso_find(bt_bap_iso_func_t func, void *user_data)
{
  struct bt_bap_iso_find_param param =
    {
          .iso = NULL,
          .func = func,
          .user_data = user_data,
    };

  bt_bap_iso_foreach(bt_bap_iso_find_cb, &param);

  return param.iso;
}

void bt_bap_iso_init(struct bt_bap_iso *iso, struct bt_iso_chan_ops *ops)
{
  iso->chan.ops = ops;
  iso->chan.qos = &iso->qos;

  /* Setup the QoS for both Tx and Rx
   * This is due to the limitation in the ISO API where pointers like the
   * `qos->tx` shall be initialized before the CIS is created
   */

  iso->chan.qos->rx = &iso->rx.qos;
  iso->chan.qos->tx = &iso->tx.qos;
}

static struct bt_bap_iso_dir *bap_iso_get_iso_dir(
  bool unicast_client, struct bt_bap_iso *iso, enum bt_audio_dir dir)
{
  /* TODO FIX FOR CLIENT */

  if (IS_ENABLED(CONFIG_BT_BAP_UNICAST_CLIENT) && unicast_client)
    {
      /* For the unicast client, the direction and tx/rx is reversed */

      if (dir == BT_AUDIO_DIR_SOURCE)
        {
          return &iso->rx;
        }
      else
        {
          return &iso->tx;
        }
    }

  if (dir == BT_AUDIO_DIR_SINK)
    {
      return &iso->rx;
    }
  else
    {
      return &iso->tx;
    }
}

void bt_bap_setup_iso_data_path(struct bt_bap_stream *stream)
{
  const struct bt_audio_codec_cfg *codec_cfg = stream->codec_cfg;
  struct bt_bap_ep *ep = stream->ep;
  struct bt_bap_iso *bap_iso = ep->iso;
  const bool is_unicast_client =
          IS_ENABLED(CONFIG_BT_BAP_UNICAST_CLIENT) &&
          bt_bap_unicast_client_has_ep(ep);
  struct bt_bap_iso_dir *iso_dir = bap_iso_get_iso_dir(is_unicast_client,
          bap_iso, ep->dir);
  struct bt_iso_chan_path path =
    {
        0
    };

  uint8_t dir;
  int err;

  if (iso_dir == &bap_iso->rx)
    {
      dir = BT_HCI_DATAPATH_DIR_CTLR_TO_HOST;
    }
  else
    {
      dir = BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;
    }

  path.pid = codec_cfg->path_id;

  /* Configure the data path to either use the controller for transcoding, or
   * set the path to be transparent to indicate that the transcoding happens
   * somewhere else
   */

  if (codec_cfg->ctlr_transcode)
    {
      path.format = codec_cfg->id;
      path.cid = codec_cfg->cid;
      path.vid = codec_cfg->vid;
      path.cc_len = codec_cfg->data_len;
      path.cc = codec_cfg->data;
    }
  else
    {
      path.format = BT_HCI_CODING_FORMAT_TRANSPARENT;
    }

  err = bt_iso_setup_data_path(&bap_iso->chan, dir, &path);
  if (err != 0)
    {
      LOG_ERR("Failed to set ISO data path for ep %p and codec_cfg %p: %d",
              ep,
              codec_cfg, err);
    }
}

void bt_bap_remove_iso_data_path(struct bt_bap_stream *stream)
{
  struct bt_bap_ep *ep = stream->ep;
  struct bt_bap_iso *bap_iso = ep->iso;
  const bool is_unicast_client =
          IS_ENABLED(CONFIG_BT_BAP_UNICAST_CLIENT) &&
          bt_bap_unicast_client_has_ep(ep);
  struct bt_bap_iso_dir *iso_dir = bap_iso_get_iso_dir(is_unicast_client,
          bap_iso, ep->dir);
  uint8_t dir;
  int err;

  if (iso_dir == &bap_iso->rx)
    {
      dir = BT_HCI_DATAPATH_DIR_CTLR_TO_HOST;
    }
  else
    {
      dir = BT_HCI_DATAPATH_DIR_HOST_TO_CTLR;
    }

  err = bt_iso_remove_data_path(&bap_iso->chan, dir);
  if (err != 0)
    {
      LOG_ERR("Failed to remove ISO data path for ep %p: %d", ep, err);
    }
}

static bool is_unicast_client_ep(struct bt_bap_ep *ep)
{
  return IS_ENABLED(CONFIG_BT_BAP_UNICAST_CLIENT) &&
      bt_bap_unicast_client_has_ep(ep);
}

void bt_bap_iso_bind_ep(struct bt_bap_iso *iso, struct bt_bap_ep *ep)
{
  const struct bt_bap_ep *paired_ep;
  struct bt_bap_iso_dir *iso_dir;

  DEBUGASSERT(ep != NULL);
  DEBUGASSERT(iso != NULL);
  DEBUGASSERT(ep->iso == NULL);
  DEBUGASSERT(ep->dir == BT_AUDIO_DIR_SINK || ep->dir ==
          BT_AUDIO_DIR_SOURCE);

  LOG_DBG("iso %p ep %p dir %s", iso, ep, bt_audio_dir_to_str(ep->dir));

  iso_dir = bap_iso_get_iso_dir(is_unicast_client_ep(ep), iso, ep->dir);
  DEBUGASSERT(iso_dir->ep == NULL);
  iso_dir->ep = ep;

  ep->iso = bt_bap_iso_ref(iso);

  paired_ep = bt_bap_iso_get_paired_ep(ep);
  if (paired_ep != NULL && paired_ep->stream != NULL &&
      paired_ep->stream->conn != NULL &&
      ep->stream != NULL && ep->stream->conn != NULL)
    {
      DEBUGASSERT(paired_ep->stream->conn == ep->stream->conn);
    }
}

void bt_bap_iso_unbind_ep(struct bt_bap_iso *iso, struct bt_bap_ep *ep)
{
  struct bt_bap_iso_dir *iso_dir;

  DEBUGASSERT(ep != NULL);
  DEBUGASSERT(iso != NULL);
  DEBUGASSERT(ep->iso == iso);
  DEBUGASSERT(ep->dir == BT_AUDIO_DIR_SINK || ep->dir ==
          BT_AUDIO_DIR_SOURCE);

  LOG_DBG("iso %p ep %p dir %s", iso, ep, bt_audio_dir_to_str(ep->dir));

  iso_dir = bap_iso_get_iso_dir(is_unicast_client_ep(ep), iso, ep->dir);
  DEBUGASSERT(iso_dir->ep == ep);
  iso_dir->ep = NULL;

  bt_bap_iso_unref(ep->iso);
  ep->iso = NULL;
}

struct bt_bap_ep *bt_bap_iso_get_ep(
  bool unicast_client, struct bt_bap_iso *iso, enum bt_audio_dir dir)
{
  struct bt_bap_iso_dir *iso_dir;

  DEBUGASSERT(dir == BT_AUDIO_DIR_SINK || dir == BT_AUDIO_DIR_SOURCE);

  LOG_DBG("iso %p dir %s", iso, bt_audio_dir_to_str(dir));

  iso_dir = bap_iso_get_iso_dir(unicast_client, iso, dir);

  return iso_dir->ep;
}

struct bt_bap_ep *bt_bap_iso_get_paired_ep(const struct bt_bap_ep *ep)
{
  if (ep->iso->rx.ep == ep)
    {
      return ep->iso->tx.ep;
    }
  else
    {
      return ep->iso->rx.ep;
    }
}

#if defined(CONFIG_BT_BAP_UNICAST_CLIENT)
void bt_bap_iso_bind_stream(
  struct bt_bap_iso *bap_iso, struct bt_bap_stream *stream,
  enum bt_audio_dir dir)
{
  struct bt_bap_iso_dir *bap_iso_ep;

  DEBUGASSERT(stream != NULL);
  DEBUGASSERT(bap_iso != NULL);
  DEBUGASSERT(stream->iso == NULL);

  LOG_DBG("bap_iso %p stream %p dir %s", bap_iso, stream,
          bt_audio_dir_to_str(dir));

  /* For the unicast client, the direction and tx/rx is reversed */

  if (dir == BT_AUDIO_DIR_SOURCE)
    {
      bap_iso_ep = &bap_iso->rx;
    }
  else
    {
      bap_iso_ep = &bap_iso->tx;
    }

  DEBUGASSERT(bap_iso_ep->stream == NULL);
  bap_iso_ep->stream = stream;

  stream->iso = &bt_bap_iso_ref(bap_iso)->chan;
}

void bt_bap_iso_unbind_stream(struct bt_bap_stream *stream,
                              enum bt_audio_dir dir)
{
  struct bt_bap_iso_dir *bap_iso_ep;
  struct bt_bap_iso *bap_iso;

  DEBUGASSERT(stream != NULL);
  DEBUGASSERT(stream->iso != NULL);

  bap_iso = CONTAINER_OF(stream->iso, struct bt_bap_iso, chan);

  LOG_DBG("bap_iso %p stream %p dir %s", bap_iso, stream,
          bt_audio_dir_to_str(dir));

  /* For the unicast client, the direction and tx/rx is reversed */

  if (dir == BT_AUDIO_DIR_SOURCE)
    {
      bap_iso_ep = &bap_iso->rx;
    }
  else
    {
      bap_iso_ep = &bap_iso->tx;
    }

  DEBUGASSERT(bap_iso_ep->stream == stream);
  bap_iso_ep->stream = NULL;

  bt_bap_iso_unref(bap_iso);
  stream->iso = NULL;
}

struct bt_bap_stream *bt_bap_iso_get_stream(struct bt_bap_iso *iso,
                                            enum bt_audio_dir dir)
{
  DEBUGASSERT(dir == BT_AUDIO_DIR_SINK || dir == BT_AUDIO_DIR_SOURCE);

  LOG_DBG("iso %p dir %s", iso, bt_audio_dir_to_str(dir));

  /* For the unicast client, the direction and tx/rx is reversed */

  if (dir == BT_AUDIO_DIR_SOURCE)
    {
      return iso->rx.stream;
    }
  else
    {
      return iso->tx.stream;
    }
}
#endif /* CONFIG_BT_BAP_UNICAST_CLIENT */
