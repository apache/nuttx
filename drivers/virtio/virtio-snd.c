/****************************************************************************
 * drivers/virtio/virtio-snd.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <sys/param.h>

#include <nuttx/audio/audio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/semaphore.h>

#include "virtio-snd.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Map for converting VirtIO frame rate to nuttx frame rate */

struct virtio_snd_rate_map_s
{
  unsigned int nrate;
  unsigned int sps;
};

/* Map for nuttx bps to virtio pcm subformat */

struct virtio_snd_format_map_s
{
  uint8_t nformat;
  unsigned int bps;
};

/* Buffer for pcm data tx/rx transfer */

struct virtio_snd_buffer_s
{
  struct ap_buffer_s apb;
  struct virtio_snd_pcm_xfer xfer;
  struct virtio_snd_pcm_status status;
  FAR struct audio_lowerhalf_s *dev;
};

/* Include struct audio_lowerhalf_s, use to get struct virtio_snd_s */

struct virtio_snd_dev_s
{
  struct audio_lowerhalf_s dev;
  uint32_t cache_buffers;
  uint32_t period_bytes;
  uint32_t frame_size;
  uint32_t index;
  bool running;
  FAR void *priv;
};

/* Virtio snd card struct for virtio driver */

struct virtio_snd_s
{
  FAR struct virtio_device *vdev;
  FAR struct virtio_snd_dev_s *dev;
  FAR struct virtio_snd_pcm_info *info;
  struct virtio_snd_config config;
  spinlock_t lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void virtio_snd_pcm_notify_cb(FAR struct virtqueue *vqueue);
static void virtio_snd_ctl_notify_cb(FAR struct virtqueue *vqueue);
static void virtio_snd_event_notify_cb(FAR struct virtqueue *vqueue);

static unsigned int virtio_snd_get_period_bytes(unsigned int rate,
                                                unsigned int ch,
                                                unsigned int bps,
                                                unsigned int period_time);
static unsigned int
virtio_snd_get_support_rates(FAR const struct virtio_snd_pcm_info *info);
static void
virtio_snd_get_support_formats(FAR const struct virtio_snd_pcm_info *info,
                               FAR struct audio_caps_s *caps);

static int virtio_snd_send_pcm(FAR struct virtio_snd_dev_s *sdev,
                               FAR struct virtio_snd_buffer_s *buf);
static int virtio_snd_send_ctl(FAR struct virtio_snd_s *priv,
                               FAR struct virtqueue_buf *vb,
                               int readable,
                               int writable);
static int virtio_snd_query_info(FAR struct virtio_snd_s *priv,
                                 int cmd,
                                 size_t size,
                                 size_t count,
                                 FAR struct virtio_snd_pcm_info *info);
static int virtio_snd_set_params(FAR struct virtio_snd_dev_s *sdev,
                                 unsigned int ch,
                                 unsigned int rate,
                                 unsigned int bps);
static int virtio_snd_send_cmd(FAR struct virtio_snd_dev_s *sdev,
                               int cmd);

static int virtio_snd_getcaps(FAR struct audio_lowerhalf_s *dev,
                              int type,
                              FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int virtio_snd_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR void *session,
                                FAR const struct audio_caps_s *caps);
static int virtio_snd_start(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int virtio_snd_stop(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int virtio_snd_pause(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session);
static int virtio_snd_resume(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session);
#endif
static int virtio_snd_reserve(FAR struct audio_lowerhalf_s *dev,
                              FAR void **session);
static int virtio_snd_release(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session);
#else
static int virtio_snd_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR const struct audio_caps_s *caps);
static int virtio_snd_start(FAR struct audio_lowerhalf_s *dev);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int virtio_snd_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int virtio_snd_pause(FAR struct audio_lowerhalf_s *dev);
static int virtio_snd_resume(FAR struct audio_lowerhalf_s *dev);
#endif
static int virtio_snd_reserve(FAR struct audio_lowerhalf_s *dev);
static int virtio_snd_release(FAR struct audio_lowerhalf_s *dev);
#endif
static int virtio_snd_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                                  FAR struct audio_buf_desc_s *apb);
static int virtio_snd_freebuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct audio_buf_desc_s *apb);
static int virtio_snd_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                    FAR struct ap_buffer_s *apb);
static int virtio_snd_ioctl(FAR struct audio_lowerhalf_s *dev,
                            int cmd,
                            unsigned long arg);
static int virtio_snd_shutdown(FAR struct audio_lowerhalf_s *dev);

static int virtio_snd_probe(FAR struct virtio_device *vdev);
static void virtio_snd_remove(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct virtio_snd_rate_map_s g_rate_map[] =
{
  [VIRTIO_SND_PCM_RATE_8000] =
    {
      AUDIO_SAMP_RATE_8K, 8000
    },
  [VIRTIO_SND_PCM_RATE_11025] =
    {
      AUDIO_SAMP_RATE_11K, 11025
    },
  [VIRTIO_SND_PCM_RATE_16000] =
    {
      AUDIO_SAMP_RATE_16K, 16000
    },
  [VIRTIO_SND_PCM_RATE_22050] =
    {
      AUDIO_SAMP_RATE_22K, 22050
    },
  [VIRTIO_SND_PCM_RATE_32000] =
    {
      AUDIO_SAMP_RATE_32K, 32000
    },
  [VIRTIO_SND_PCM_RATE_44100] =
    {
      AUDIO_SAMP_RATE_44K, 44100
    },
  [VIRTIO_SND_PCM_RATE_48000] =
    {
      AUDIO_SAMP_RATE_48K, 48000
    },
  [VIRTIO_SND_PCM_RATE_96000] =
    {
      AUDIO_SAMP_RATE_96K, 96000
    },
  [VIRTIO_SND_PCM_RATE_192000] =
    {
      AUDIO_SAMP_RATE_192K, 192000
    }
};

static const struct virtio_snd_format_map_s g_format_map[] =
{
  [VIRTIO_SND_PCM_FMT_S8] =
    {
      AUDIO_SUBFMT_PCM_S8, 8
    },
  [VIRTIO_SND_PCM_FMT_S16] =
    {
      AUDIO_SUBFMT_PCM_S16_LE, 16
    },
  [VIRTIO_SND_PCM_FMT_S32] =
    {
      AUDIO_SUBFMT_PCM_S32_LE, 32
    }
};

static struct virtio_driver g_virtio_snd_driver =
{
  LIST_INITIAL_VALUE(g_virtio_snd_driver.node), /* node */
  VIRTIO_ID_SOUND,                              /* device id */
  virtio_snd_probe,                             /* probe */
  virtio_snd_remove,                            /* remove */
};

static const struct audio_ops_s g_virtio_snd_ops =
{
  virtio_snd_getcaps,        /* getcaps        */
  virtio_snd_configure,      /* configure      */
  virtio_snd_shutdown,       /* shutdown       */
  virtio_snd_start,          /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  virtio_snd_stop,           /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  virtio_snd_pause,          /* pause          */
  virtio_snd_resume,         /* resume         */
#endif
  virtio_snd_allocbuffer,    /* allocbuffer    */
  virtio_snd_freebuffer,     /* freebuffer     */
  virtio_snd_enqueuebuffer,  /* enqueuebuffer */
  NULL,                      /* cancelbuffer  */
  virtio_snd_ioctl,          /* ioctl          */
  NULL,                      /* read           */
  NULL,                      /* write          */
  virtio_snd_reserve,        /* reserve        */
  virtio_snd_release         /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_snd_get_period_bytes
 ****************************************************************************/

static unsigned int virtio_snd_get_period_bytes(unsigned int rate,
                                                unsigned int ch,
                                                unsigned int bps,
                                                unsigned int period_time)
{
  return rate * ch * (bps / 8) * period_time / 1000;
}

/****************************************************************************
 * Name: virtio_snd_get_support_rates
 ****************************************************************************/

static unsigned int
virtio_snd_get_support_rates(FAR const struct virtio_snd_pcm_info *info)
{
  unsigned int rates = 0;
  int i;

  for (i = VIRTIO_SND_PCM_RATE_5512; i < VIRTIO_SND_PCM_RATE_384000; i++)
    {
      if (info->rates & (1 << i))
        {
          rates |= g_rate_map[i].nrate;
        }
    }

  return rates;
}

/****************************************************************************
 * Name: virtio_snd_get_support_formats
 ****************************************************************************/

static void
virtio_snd_get_support_formats(FAR const struct virtio_snd_pcm_info *info,
                               FAR struct audio_caps_s *caps)
{
  size_t subformats = 0;
  size_t i;

  for (i = 0; i < nitems(g_format_map); i++)
    {
      if (info->formats & (1 << i))
        {
          caps->ac_controls.b[subformats++] = g_format_map[i].nformat;
          if (subformats >= nitems(caps->ac_controls.b))
            break;
        }
    }
}

/****************************************************************************
 * Name: virtio_snd_pcm_notify_cb
 ****************************************************************************/

static void virtio_snd_pcm_notify_cb(FAR struct virtqueue *vq)
{
  FAR struct virtio_snd_s *priv = vq->vq_dev->priv;

  for (; ; )
    {
      FAR struct virtio_snd_buffer_s *buf;
      FAR struct virtio_snd_dev_s *sdev;
      buf = virtqueue_get_buffer_lock(vq, NULL, NULL, &priv->lock);
      if (buf == NULL)
        {
          break;
        }

#ifdef CONFIG_AUDIO_MULTI_SESSION
      buf->dev->upper(buf->dev->priv, AUDIO_CALLBACK_DEQUEUE,
                      &buf->apb, OK, NULL);
#else
      buf->dev->upper(buf->dev->priv, AUDIO_CALLBACK_DEQUEUE,
                      &buf->apb, OK);
#endif
      sdev = (FAR struct virtio_snd_dev_s *)buf->dev;
      sdev->cache_buffers--;
    }
}

/****************************************************************************
 * Name: virtio_snd_ctl_notify_cb
 ****************************************************************************/

static void virtio_snd_ctl_notify_cb(FAR struct virtqueue *vq)
{
  FAR struct virtio_snd_s *priv = vq->vq_dev->priv;
  FAR sem_t *ctl_sem;

  ctl_sem = virtqueue_get_buffer_lock(vq, NULL, NULL, &priv->lock);
  nxsem_post(ctl_sem);
}

/****************************************************************************
 * Name: virtio_snd_event_notify_cb
 ****************************************************************************/

static void virtio_snd_event_notify_cb(FAR struct virtqueue *vqueue)
{
  vrtinfo("recvive jack/pcm event\n");
}

/****************************************************************************
 * Name: virtio_snd_send_pcm
 ****************************************************************************/

static int virtio_snd_send_pcm(FAR struct virtio_snd_dev_s *sdev,
                               FAR struct virtio_snd_buffer_s *buf)
{
  FAR struct virtio_snd_s *priv = sdev->priv;
  FAR struct virtio_snd_pcm_info *info = &priv->info[sdev->index];
  int idx = info->direction == VIRTIO_SND_D_INPUT ?
                               VIRTIO_SND_VQ_RX : VIRTIO_SND_VQ_TX;
  FAR struct virtqueue *vq = priv->vdev->vrings_info[idx].vq;
  struct virtqueue_buf vb[3];
  irqstate_t flags;

  vb[0].buf = &buf->xfer;
  vb[0].len = sizeof(buf->xfer);
  vb[1].buf = buf->apb.samp;
  vb[1].len = sdev->period_bytes;
  vb[2].buf = &buf->status;
  vb[2].len = sizeof(buf->status);

  flags = spin_lock_irqsave(&priv->lock);
  if (idx == VIRTIO_SND_VQ_RX)
    {
      virtqueue_add_buffer(vq, vb, 1, 2, buf);
    }
  else
    {
      virtqueue_add_buffer(vq, vb, 2, 1, buf);
    }

  virtqueue_kick(vq);
  spin_unlock_irqrestore(&priv->lock, flags);
  sdev->cache_buffers++;

  return OK;
}

/****************************************************************************
 * Name: virtio_snd_send_ctl
 ****************************************************************************/

static int virtio_snd_send_ctl(FAR struct virtio_snd_s *priv,
                               FAR struct virtqueue_buf *vb,
                               int readable,
                               int writable)
{
  FAR struct virtqueue *vq =
    priv->vdev->vrings_info[VIRTIO_SND_VQ_CONTROL].vq;
  irqstate_t flags;
  sem_t ctl_sem;
  int ret;

  nxsem_init(&ctl_sem, 0, 0);

  flags = spin_lock_irqsave(&priv->lock);
  virtqueue_add_buffer(vq, vb, readable, writable, &ctl_sem);
  virtqueue_kick(vq);
  spin_unlock_irqrestore(&priv->lock, flags);

  ret = nxsem_wait_uninterruptible(&ctl_sem);
  nxsem_destroy(&ctl_sem);
  if (ret < 0)
    {
      vrterr("nxsem wait error:%d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_snd_query_info
 ****************************************************************************/

static int virtio_snd_query_info(FAR struct virtio_snd_s *priv,
                                 int cmd,
                                 size_t size,
                                 size_t count,
                                 FAR struct virtio_snd_pcm_info *info)
{
  FAR struct virtio_snd_query_info *req;
  FAR struct virtio_snd_hdr *resp;
  struct virtqueue_buf vb[3];
  int ret;

  req = virtio_zalloc_buf(priv->vdev, sizeof(*req), 16);
  if (req == NULL)
    {
      vrterr("virtio audio driver cmd request alloc failed\n");
      return -ENOMEM;
    }

  req->hdr.code = cmd;
  req->count = count;
  req->size = size;

  resp = virtio_alloc_buf(priv->vdev, sizeof(*resp), 16);
  if (resp == NULL)
    {
      vrterr("virtio audio driver cmd response alloc failed\n");
      ret = -ENOMEM;
      goto out;
    }

  resp->code = VIRTIO_SND_S_IO_ERR;

  vb[0].buf = req;
  vb[0].len = sizeof(*req);
  vb[1].buf = resp;
  vb[1].len = sizeof(*resp);
  vb[2].buf = info;
  vb[2].len = count * size;

  ret = virtio_snd_send_ctl(priv, vb, 1, 2);
  if (ret < 0)
    {
      vrterr("send msg error:%d\n", ret);
      goto out;
    }

  ret = resp->code == VIRTIO_SND_S_OK ? OK : -EIO;
  vrtinfo("send cmd:0x%x and resp:0x%"PRIu32"\n", cmd, resp->code);

out:
  virtio_free_buf(priv->vdev, req);
  virtio_free_buf(priv->vdev, resp);
  return ret;
}

/****************************************************************************
 * Name: virtio_snd_dev_set_params
 ****************************************************************************/

static int virtio_snd_set_params(FAR struct virtio_snd_dev_s *sdev,
                                 unsigned int ch,
                                 unsigned int rate,
                                 unsigned int bps)
{
  FAR struct virtio_snd_s *priv = sdev->priv;
  FAR struct virtio_snd_pcm_set_params *req;
  FAR struct virtio_snd_hdr *resp;
  struct virtqueue_buf vb[2];
  size_t i;
  int ret;

  req = virtio_zalloc_buf(priv->vdev, sizeof(*req), 16);
  if (req == NULL)
    {
      vrterr("zalloc for request error\n");
      return -ENOMEM;
    }

  req->hdr.hdr.code = VIRTIO_SND_R_PCM_SET_PARAMS;
  req->hdr.stream_id = sdev->index;
  req->channels = ch;

  req->rate = VIRTIO_SND_PCM_RATE_44100;
  for (i = 0; i < nitems(g_rate_map); i++)
    {
      if (rate == g_rate_map[i].sps)
        {
          req->rate = i;
          break;
        }
    }

  req->format = VIRTIO_SND_PCM_FMT_S16;
  for (i = 0; i < nitems(g_format_map); i++)
    {
      if (bps == g_format_map[i].bps)
        {
          req->format = i;
          break;
        }
    }

  req->period_bytes = sdev->period_bytes;
  req->buffer_bytes = req->period_bytes *
                      CONFIG_DRIVERS_VIRTIO_SND_BUFFER_COUNT;

  resp = virtio_alloc_buf(priv->vdev, sizeof(*resp), 16);
  if (resp == NULL)
    {
      vrterr("zalloc for request error\n");
      ret = -ENOMEM;
      goto out;
    }

  resp->code = VIRTIO_SND_S_IO_ERR;

  vb[0].buf = req;
  vb[0].len = sizeof(*req);
  vb[1].buf = resp;
  vb[1].len = sizeof(*resp);

  ret = virtio_snd_send_ctl(priv, vb, 1, 1);
  if (ret < 0)
    {
      vrterr("send msg error:%d\n", ret);
      goto out;
    }

  ret = resp->code == VIRTIO_SND_S_OK ? OK : -EIO;

  vrtinfo("send cmd:0x%x and resp:0x%"PRIu32"\n",
          VIRTIO_SND_R_PCM_SET_PARAMS, resp->code);

out:
  virtio_free_buf(priv->vdev, req);
  virtio_free_buf(priv->vdev, resp);
  return ret;
}

/****************************************************************************
 * Name: virtio_snd_send_cmd
 ****************************************************************************/

static int virtio_snd_send_cmd(FAR struct virtio_snd_dev_s *sdev,
                               int cmd)
{
  FAR struct virtio_snd_s *priv = sdev->priv;
  FAR struct virtio_device *vdev = priv->vdev;
  FAR struct virtio_snd_pcm_hdr *req;
  FAR struct virtio_snd_hdr *resp;
  struct virtqueue_buf vb[2];
  int ret;

  req = virtio_alloc_buf(vdev, sizeof(*req), 16);
  if (req == NULL)
    {
      vrterr("zalloc for request error\n");
      return -ENOMEM;
    }

  req->hdr.code = cmd;
  req->stream_id = sdev->index;

  resp = virtio_alloc_buf(vdev, sizeof(*resp), 16);
  if (resp == NULL)
    {
      vrterr("zalloc for request error\n");
      ret = -ENOMEM;
      goto out;
    }

  resp->code = VIRTIO_SND_S_IO_ERR;

  vb[0].buf = req;
  vb[0].len = sizeof(*req);
  vb[1].buf = resp;
  vb[1].len = sizeof(*resp);

  ret = virtio_snd_send_ctl(priv, vb, 1, 1);
  if (ret < 0)
    {
      vrterr("send msg error:%d\n", ret);
      goto out;
    }

  ret = resp->code == VIRTIO_SND_S_OK ? OK : -EIO;
  if (ret < 0)
    {
      vrterr("check response error:%d\n", ret);
    }

  vrtinfo("send cmd:0x%x and resp:0x%"PRIu32"\n", cmd, resp->code);

out:
  virtio_free_buf(vdev, req);
  virtio_free_buf(vdev, resp);
  return ret;
}

/****************************************************************************
 * Name: virtio_snd_getcaps
 ****************************************************************************/

static int virtio_snd_getcaps(FAR struct audio_lowerhalf_s *dev,
                              int type,
                              FAR struct audio_caps_s *caps)
{
  FAR struct virtio_snd_dev_s *sdev = (FAR struct virtio_snd_dev_s *)dev;
  FAR struct virtio_snd_s *priv = sdev->priv;
  FAR struct virtio_snd_pcm_info *info = &priv->info[sdev->index];

  DEBUGASSERT(caps->ac_len >= sizeof(struct audio_caps_s));

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_QUERY:
        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:
              caps->ac_controls.b[0] =
              info->direction == VIRTIO_SND_D_INPUT ?
                                 AUDIO_TYPE_INPUT : AUDIO_TYPE_OUTPUT;
              caps->ac_format.hw = 1 << (AUDIO_FMT_PCM - 1);
              break;

            case AUDIO_FMT_PCM:
              virtio_snd_get_support_formats(info, caps);
              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }
         break;

      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:
        {
          caps->ac_channels = (info->channels_min << 4) |
                              (info->channels_max & 0x0f);
          switch (caps->ac_subtype)
            {
              case AUDIO_TYPE_QUERY:
                caps->ac_controls.b[0] = virtio_snd_get_support_rates(info);
                break;

              default:
                break;
            }
          break;
        }

      default:
        caps->ac_subtype = 0;
        caps->ac_channels = 0;
        break;
    }

  return caps->ac_len;
}

/****************************************************************************
 * Name: virtio_snd_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int virtio_snd_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR void *session,
                                FAR const struct audio_caps_s *caps)
#else
static int virtio_snd_configure(FAR struct audio_lowerhalf_s *dev,
                                FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct virtio_snd_dev_s *sdev = (FAR struct virtio_snd_dev_s *)dev;
  int ret = -ENOTTY;
  uint32_t rate;
  uint8_t bps;
  uint8_t ch;

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:
        vrtinfo("Number of channels: %"PRIu8"\n", caps->ac_channels);
        vrtinfo("Sample rate:        %"PRIu16"\n", caps->ac_controls.hw[0]);
        vrtinfo("Sample width:       %"PRIu8"\n", caps->ac_controls.b[2]);
        vrtinfo("channel map: 0x%x\n", caps->ac_chmap);
        rate = caps->ac_controls.hw[0] | (caps->ac_controls.b[3] << 16);
        bps = caps->ac_controls.b[2];
        ch = caps->ac_channels;
        sdev->frame_size = ch * bps / 8;
        sdev->period_bytes =
        virtio_snd_get_period_bytes(rate, ch, bps,
                                    CONFIG_DRIVERS_VIRTIO_SOUND_PERIOD_TIME);
        vrtinfo("period_bytes:%"PRIu32"\n", sdev->period_bytes);
        ret = virtio_snd_set_params(sdev, ch, rate, bps);
        if (ret < 0)
          {
            break;
          }

        ret = virtio_snd_send_cmd(sdev, VIRTIO_SND_R_PCM_PREPARE);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_snd_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int virtio_snd_start(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session)
#else
static int virtio_snd_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct virtio_snd_dev_s *sdev = (FAR struct virtio_snd_dev_s *)dev;
  int ret = OK;

  if (!sdev->running)
    {
      ret = virtio_snd_send_cmd(sdev, VIRTIO_SND_R_PCM_START);
      if (ret < 0)
        {
          return ret;
        }

      sdev->running = true;
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_snd_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int virtio_snd_stop(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session)
#else
static int virtio_snd_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct virtio_snd_dev_s *sdev = (FAR struct virtio_snd_dev_s *)dev;
  int ret = OK;

  if (sdev->running)
    {
      ret = virtio_snd_send_cmd(sdev, VIRTIO_SND_R_PCM_STOP);
      if (ret < 0)
        {
          return ret;
        }

      sdev->running = false;
    }

  ret = virtio_snd_send_cmd(sdev, VIRTIO_SND_R_PCM_RELEASE);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_AUDIO_MULTI_SESSION
  dev->upper(dev->priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  dev->upper(dev->priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  return ret;
}
#endif

/****************************************************************************
 * Name: virtio_snd_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int virtio_snd_pause(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session)
#else
static int virtio_snd_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct virtio_snd_dev_s *sdev = (FAR struct virtio_snd_dev_s *)dev;
  int ret = OK;

  if (sdev->running)
    {
      ret = virtio_snd_send_cmd(sdev, VIRTIO_SND_R_PCM_STOP);
      if (ret < 0)
        {
          return ret;
        }

      sdev->running = false;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: virtio_snd_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int virtio_snd_resume(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session)
#else
static int virtio_snd_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct virtio_snd_dev_s *sdev = (FAR struct virtio_snd_dev_s *)dev;
  int ret = OK;

  if (!sdev->running)
    {
      ret = virtio_snd_send_cmd(sdev, VIRTIO_SND_R_PCM_START);
      if (ret < 0)
        {
          return ret;
        }

      sdev->running = true;
    }

  return ret;
}
#endif

static int virtio_snd_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                                  FAR struct audio_buf_desc_s *desc)
{
  FAR struct virtio_snd_dev_s *sdev = (FAR struct virtio_snd_dev_s *)dev;
  FAR struct virtio_snd_s *priv = sdev->priv;
  FAR struct virtio_snd_buffer_s *buf;

  DEBUGASSERT(desc->u.pbuffer != NULL);

  buf = virtio_zalloc_buf(priv->vdev, sizeof(*buf) + desc->numbytes, 16);
  if (buf == NULL)
    {
      vrterr("failed to allocate apb buffer\n");
      return -ENOMEM;
    }

  *desc->u.pbuffer = &buf->apb;

  buf->apb.crefs = 1;
  buf->apb.nmaxbytes = desc->numbytes;
  buf->apb.samp = (FAR uint8_t *)(buf + 1);
#ifdef CONFIG_AUDIO_MULTI_SESSION
  buf->apb.session = desc->session;
#endif
  buf->xfer.stream_id = sdev->index;
  buf->status.status = VIRTIO_SND_S_IO_ERR;
  buf->dev = dev;
  nxmutex_init(&buf->apb.lock);

  return sizeof(struct audio_buf_desc_s);
}

static int virtio_snd_freebuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct audio_buf_desc_s *desc)
{
  FAR struct virtio_snd_dev_s *sdev = (FAR struct virtio_snd_dev_s *)dev;
  FAR struct virtio_snd_s *priv = sdev->priv;
  FAR struct virtio_device *vdev = priv->vdev;
  FAR struct ap_buffer_s *apb = desc->u.buffer;
  int refcount;

  nxmutex_lock(&apb->lock);
  refcount = apb->crefs--;
  nxmutex_unlock(&apb->lock);
  if (refcount <= 1)
    {
      nxmutex_destroy(&apb->lock);
      virtio_free_buf(vdev, apb);
    }

  return sizeof(struct audio_buf_desc_s);
}

/****************************************************************************
 * Name: virtio_snd_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/

static int virtio_snd_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                    FAR struct ap_buffer_s *apb)
{
  FAR struct virtio_snd_dev_s *sdev = (FAR struct virtio_snd_dev_s *)dev;
  FAR struct virtio_snd_buffer_s *buf =
    (FAR struct virtio_snd_buffer_s *)apb;

  return virtio_snd_send_pcm(sdev, buf);
}

/****************************************************************************
 * Name: virtio_snd_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int virtio_snd_ioctl(FAR struct audio_lowerhalf_s *dev,
                            int cmd,
                            unsigned long arg)
{
  FAR struct virtio_snd_dev_s *sdev = (FAR struct virtio_snd_dev_s *)dev;
  FAR struct ap_buffer_info_s *bufinfo;
  FAR long *latency = (FAR long *)arg;

  switch (cmd)
    {
      case AUDIOIOC_GETBUFFERINFO:
        bufinfo = (FAR struct ap_buffer_info_s *)arg;
        bufinfo->nbuffers = CONFIG_DRIVERS_VIRTIO_SND_BUFFER_COUNT;
        bufinfo->buffer_size = sdev->period_bytes;
        break;

      case AUDIOIOC_GETLATENCY:
        *latency = sdev->cache_buffers * sdev->period_bytes /
                   sdev->frame_size;
        break;

      default:
        return -ENOTTY;
    }

  return OK;
}

/****************************************************************************
 * Name: virtio_snd_shutdown
 *
 * Description:
 *   Shutdown the driver and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int virtio_snd_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  vrtinfo("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: virtio_snd_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int virtio_snd_reserve(FAR struct audio_lowerhalf_s *dev,
                              FAR void **session)
#else
static int virtio_snd_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  vrtinfo("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: virtio_snd_release
 *
 * Description: Releases the session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int virtio_snd_release(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session)
#else
static int virtio_snd_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  vrtinfo("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: virtio_snd_init
 ****************************************************************************/

static int virtio_snd_init(FAR struct virtio_snd_s *priv)
{
  FAR const char *vqnames[VIRTIO_SND_VQ_MAX];
  vq_callback callbacks[VIRTIO_SND_VQ_MAX];
  int ret;
  int i;

  vqnames[VIRTIO_SND_VQ_CONTROL] = "virtsnd-ctl";
  vqnames[VIRTIO_SND_VQ_EVENT] = "virtsnd-event";
  vqnames[VIRTIO_SND_VQ_TX] = "virtsnd-tx";
  vqnames[VIRTIO_SND_VQ_RX] = "virtsnd-rx";

  callbacks[VIRTIO_SND_VQ_CONTROL] = virtio_snd_ctl_notify_cb;
  callbacks[VIRTIO_SND_VQ_EVENT] = virtio_snd_event_notify_cb;
  callbacks[VIRTIO_SND_VQ_TX] = virtio_snd_pcm_notify_cb;
  callbacks[VIRTIO_SND_VQ_RX] = virtio_snd_pcm_notify_cb;

  ret = virtio_create_virtqueues(priv->vdev, 0, VIRTIO_SND_VQ_MAX,
                                 vqnames, callbacks, NULL);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      return ret;
    }

  for (i = 0; i < VIRTIO_SND_VQ_MAX; i++)
    {
      virtqueue_enable_cb(priv->vdev->vrings_info[i].vq);
    }

  virtio_set_status(priv->vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);

  virtio_read_config(priv->vdev, 0, &priv->config,
                     sizeof(struct virtio_snd_config));
  vrtinfo("jacks:%"PRIu32" streams:%"PRIu32" chmap:%"PRIu32"\n",
           priv->config.jacks, priv->config.streams, priv->config.chmaps);

  priv->info = virtio_alloc_buf(priv->vdev,
                                priv->config.streams *
                                sizeof(struct virtio_snd_pcm_info),
                                16);
  if (priv->info == NULL)
    {
      vrterr("virtio audio driver query pcm info alloc failed\n");
      ret = -ENOMEM;
      goto err_with_vq;
    }

  /* Query pcm info */

  ret = virtio_snd_query_info(priv, VIRTIO_SND_R_PCM_INFO,
                              sizeof(struct virtio_snd_pcm_info),
                              priv->config.streams,
                              priv->info);
  if (ret < 0)
    {
      vrterr("virtio snd query pcm info failed,ret:%d\n", ret);
      goto err_with_info;
    }

  return ret;

err_with_info:
  virtio_free_buf(priv->vdev, priv->info);
err_with_vq:
  virtio_delete_virtqueues(priv->vdev);
  return ret;
}

static int
virtio_snd_register_audio_driver(FAR struct virtio_snd_s *priv)
{
  char devname[32];
  int tx_minor = 0;
  int rx_minor = 0;
  int ret = -ENODEV;
  uint32_t i;

  priv->dev = kmm_zalloc(priv->config.streams *
                         sizeof(struct virtio_snd_dev_s));
  if (priv->dev == NULL)
    {
      vrterr("zalloc for virtio audio device failed\n");
      return -ENOMEM;
    }

  for (i = 0; i < priv->config.streams; i++)
    {
      switch (priv->info[i].direction)
        {
          case VIRTIO_SND_D_OUTPUT:
            snprintf(devname, sizeof(devname), "pcm%dp", tx_minor++);
            break;

          case VIRTIO_SND_D_INPUT:
            snprintf(devname, sizeof(devname), "pcm%dc", rx_minor++);
            break;
        }

      priv->dev[i].index = i;
      priv->dev[i].running = false;
      priv->dev[i].priv = priv;
      priv->dev[i].dev.ops = &g_virtio_snd_ops;
      ret = audio_register(devname, &priv->dev[i].dev);
      if (ret < 0)
        {
          vrterr("failed to register /dev/%s device: %d\n",
                  devname, ret);
          break;
        }
    }

  if (i == 0)
    {
      kmm_free(priv->dev);
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_snd_probe
 ****************************************************************************/

static int virtio_snd_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_snd_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("virtio net driver priv alloc failed\n");
      return -ENOMEM;
    }

  spin_lock_init(&priv->lock);
  priv->vdev = vdev;
  vdev->priv = priv;

  ret = virtio_snd_init(priv);
  if (ret < 0)
    {
      goto err_with_virtsnd;
    }

  ret = virtio_snd_register_audio_driver(priv);
  if (ret < 0)
    {
      goto err_with_vq;
    }

  return ret;

err_with_vq:
  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
  virtio_free_buf(vdev, priv->info);
err_with_virtsnd:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: virtio_snd_remove
 ****************************************************************************/

static void virtio_snd_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_snd_s *priv = vdev->priv;

  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
  virtio_free_buf(vdev, priv->info);
  kmm_free(priv->dev);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_snd_driver
 ****************************************************************************/

int virtio_register_snd_driver(void)
{
  return virtio_register_driver(&g_virtio_snd_driver);
}
