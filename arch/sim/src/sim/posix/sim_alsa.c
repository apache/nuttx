/****************************************************************************
 * arch/sim/src/sim/posix/sim_alsa.c
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
#include <nuttx/audio/audio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/nuttx.h>

#include <debug.h>

#include <alsa/asoundlib.h>

#include "sim_offload.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AUDMIN(a,b)     ((a) > (b) ? (b) : (a))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_audio_s
{
  struct audio_lowerhalf_s dev;
  struct dq_queue_s pendq;

  sq_entry_t link;

  bool playback;
  bool offload;
  uint32_t frame_size;
  uint32_t nbuffers;
  uint32_t buffer_size;

  uint32_t sample_rate;
  uint32_t channels;
  uint32_t bps;

  snd_pcm_t *pcm;
  snd_mixer_t *mixer;
  snd_mixer_elem_t *volume;

  void *codec;
  const sim_codec_ops_s *ops;

  struct ap_buffer_s *aux;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sim_audio_getcaps(struct audio_lowerhalf_s *dev, int type,
                             struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int sim_audio_configure(struct audio_lowerhalf_s *dev,
                               void *session,
                               const struct audio_caps_s *caps);
#else
static int sim_audio_configure(struct audio_lowerhalf_s *dev,
                               const struct audio_caps_s *caps);
#endif
static int sim_audio_shutdown(struct audio_lowerhalf_s *dev);
static int sim_audio_start(struct audio_lowerhalf_s *dev);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int sim_audio_stop(struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int sim_audio_pause(struct audio_lowerhalf_s *dev);
static int sim_audio_resume(struct audio_lowerhalf_s *dev);
#endif
static int sim_audio_enqueuebuffer(struct audio_lowerhalf_s *dev,
                                   struct ap_buffer_s *apb);
static int sim_audio_ioctl(struct audio_lowerhalf_s *dev, int cmd,
                           unsigned long arg);
static int sim_audio_reserve(struct audio_lowerhalf_s *dev);
static int sim_audio_release(struct audio_lowerhalf_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_sim_audio_ops =
{
  .getcaps       = sim_audio_getcaps,
  .configure     = sim_audio_configure,
  .shutdown      = sim_audio_shutdown,
  .start         = sim_audio_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop          = sim_audio_stop,
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  .pause         = sim_audio_pause,
  .resume        = sim_audio_resume,
#endif
  .enqueuebuffer = sim_audio_enqueuebuffer,
  .ioctl         = sim_audio_ioctl,
  .reserve       = sim_audio_reserve,
  .release       = sim_audio_release,
};

static sq_queue_t g_sim_audio;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sim_audio_config_format(struct sim_audio_s *priv, snd_pcm_t *pcm)
{
  snd_pcm_hw_params_t *hw_params;
  snd_pcm_uframes_t pframes;
  snd_pcm_format_t format;
  uint32_t total_size;
  int ret;

  switch (priv->bps)
    {
      case 8:
        format = SND_PCM_FORMAT_S8;
        break;
      case 24:
        format = SND_PCM_FORMAT_S24;
        break;
      case 32:
        format = SND_PCM_FORMAT_S32;
        break;
      default:
        format = SND_PCM_FORMAT_S16;
        break;
    }

  ret = snd_pcm_hw_params_malloc(&hw_params);
  if (ret < 0)
    {
      return ret;
    }

  ret = snd_pcm_hw_params_any(pcm, hw_params);
  if (ret < 0)
    {
      goto fail;
    }

  ret = snd_pcm_hw_params_set_access(pcm, hw_params,
                                     SND_PCM_ACCESS_RW_INTERLEAVED);
  if (ret < 0)
    {
      goto fail;
    }

  ret = snd_pcm_hw_params_set_format(pcm, hw_params, format);
  if (ret < 0)
    {
      goto fail;
    }

  ret = snd_pcm_hw_params_set_rate(pcm, hw_params,
                                   priv->sample_rate, 0);
  if (ret < 0)
    {
      goto fail;
    }

  ret = snd_pcm_hw_params_set_channels(pcm, hw_params,
                                       priv->channels);
  if (ret < 0)
    {
      goto fail;
    }

  total_size = priv->nbuffers * priv->buffer_size;

  pframes = priv->buffer_size / priv->frame_size;
  ret = snd_pcm_hw_params_set_period_size_near(pcm, hw_params,
                                               &pframes, NULL);
  if (ret < 0)
    {
      goto fail;
    }

  priv->buffer_size = pframes * priv->frame_size;
  priv->nbuffers    = total_size / priv->buffer_size;
  ret = snd_pcm_hw_params_set_periods_near(pcm, hw_params,
                                           &priv->nbuffers, NULL);
  if (ret < 0)
    {
      goto fail;
    }

  ret = snd_pcm_hw_params(pcm, hw_params);

fail:
  snd_pcm_hw_params_free(hw_params);
  return ret;
}

static void sim_audio_config_ops(struct sim_audio_s *priv, uint8_t fmt)
{
  int i;

  for (i = 0; g_codec_ops[i].format != AUDIO_FMT_UNDEF; i++)
    {
      if (g_codec_ops[i].format == fmt &&
          ((priv->playback && g_codec_ops[i].flags & AUDCODEC_DEC) ||
           (!priv->playback && g_codec_ops[i].flags & AUDCODEC_ENC)))
        {
          priv->ops = &g_codec_ops[i];
          break;
        }
    }

  if (!priv->ops)
    {
      priv->ops = &g_codec_ops[0];
    }
}

static int sim_audio_open(struct sim_audio_s *priv)
{
  irqstate_t flags;
  snd_pcm_t *pcm = NULL;
  int direction;
  int ret;

  if (priv->pcm)
    {
      return 0;
    }

  flags = up_irq_save();

  direction = priv->playback ? SND_PCM_STREAM_PLAYBACK
                             : SND_PCM_STREAM_CAPTURE;

  ret = snd_pcm_open(&pcm, "default", direction, 0);
  if (ret < 0)
    {
      goto fail;
    }

  ret = sim_audio_config_format(priv, pcm);
  if (ret < 0)
    {
      goto fail;
    }

  ret = snd_pcm_start(pcm);
  if (ret < 0)
    {
      goto fail;
    }

  priv->pcm = pcm;

  up_irq_restore(flags);
  return 0;

fail:
  snd_pcm_close(pcm);
  up_irq_restore(flags);
  return ret;
}

static int sim_audio_close(struct sim_audio_s *priv)
{
  if (!priv->pcm)
    {
      return 0;
    }

  snd_pcm_close(priv->pcm);

  priv->pcm = NULL;

  return 0;
}

static int sim_audio_getcaps(struct audio_lowerhalf_s *dev, int type,
                             struct audio_caps_s *caps)
{
  struct sim_audio_s *priv = (struct sim_audio_s *)dev;
  long val;

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_QUERY:

        caps->ac_channels = 2;       /* Stereo output */

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:
              caps->ac_controls.b[0] = (priv->playback ?
                                       AUDIO_TYPE_OUTPUT :
                                       AUDIO_TYPE_INPUT) |
                                       AUDIO_TYPE_FEATURE |
                                       AUDIO_TYPE_PROCESSING;
              if (priv->offload)
                {
                   caps->ac_format.hw = (1 << (AUDIO_FMT_MP3 - 1));
                }
              else
                {
                   caps->ac_format.hw = (1 << (AUDIO_FMT_PCM - 1));
                }
              break;
            case AUDIO_FMT_MP3:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_PCM_MP3;
              caps->ac_controls.b[1] = AUDIO_SUBFMT_END;
              break;
            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }

        break;

      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:
        caps->ac_channels = 2;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report the Sample rates we support */

              caps->ac_controls.b[0] = AUDIO_SAMP_RATE_8K |
                                       AUDIO_SAMP_RATE_11K |
                                       AUDIO_SAMP_RATE_16K |
                                       AUDIO_SAMP_RATE_22K |
                                       AUDIO_SAMP_RATE_32K |
                                       AUDIO_SAMP_RATE_44K |
                                       AUDIO_SAMP_RATE_48K;
              break;

            default:
              break;
          }

        break;

      case AUDIO_TYPE_FEATURE:

        switch (caps->ac_format.hw)
          {
            case AUDIO_FU_VOLUME:
              snd_mixer_selem_get_playback_volume(priv->volume,
                                        SND_MIXER_SCHN_UNKNOWN,
                                        &val);
              caps->ac_controls.w = val;
              break;
            case AUDIO_FU_INP_GAIN:
              snd_mixer_selem_get_capture_volume(priv->volume,
                                        SND_MIXER_SCHN_MONO,
                                        &val);
              caps->ac_controls.w = val;
              break;
            default:
              break;
          }

        break;

      default:
        caps->ac_subtype = 0;
        caps->ac_channels = 0;
        break;
    }

  /* Return the length of the audio_caps_s struct for validation of
   * proper Audio device type.
   */

  audinfo("Return %d\n", caps->ac_len);
  return caps->ac_len;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int sim_audio_configure(struct audio_lowerhalf_s *dev,
                               void *session,
                               const struct audio_caps_s *caps)
#else
static int sim_audio_configure(struct audio_lowerhalf_s *dev,
                               const struct audio_caps_s *caps)
#endif
{
  struct sim_audio_s *priv = (struct sim_audio_s *)dev;
  int ret = 0;

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_FEATURE:
        switch (caps->ac_format.hw)
          {
            case AUDIO_FU_VOLUME:
              ret = snd_mixer_selem_set_playback_volume_all(priv->volume,
                                                caps->ac_controls.hw[0]);
              break;
            case AUDIO_FU_INP_GAIN:
              ret = snd_mixer_selem_set_capture_volume_all(priv->volume,
                                                caps->ac_controls.hw[0]);
              break;
            default:
              ret = -ENOTTY;
              break;
          }
        break;

      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:
        priv->sample_rate = caps->ac_controls.hw[0] |
                            (caps->ac_controls.b[3] << 16);
        priv->channels    = caps->ac_channels;
        priv->bps         = caps->ac_controls.b[2];
        priv->frame_size  = priv->bps / 8 * priv->channels;

        sim_audio_config_ops(priv, caps->ac_subtype);
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int sim_audio_shutdown(struct audio_lowerhalf_s *dev)
{
  return 0;
}

static int sim_audio_start(struct audio_lowerhalf_s *dev)
{
  struct sim_audio_s *priv = (struct sim_audio_s *)dev;
  struct audio_buf_desc_s buf_desc;
  int ret;

  /* reserved aux buffer. */

  buf_desc.numbytes  = priv->buffer_size * 2;
  buf_desc.u.pbuffer = &priv->aux;

  ret = apb_alloc(&buf_desc);
  if (ret != sizeof(buf_desc))
    {
      return -ENOMEM;
    }

  priv->codec = priv->ops->init(NULL);
  if (priv->codec == NULL)
    {
      return -ENOSYS;
    }

  return sim_audio_open(priv);
}

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int sim_audio_stop(struct audio_lowerhalf_s *dev)
{
  struct sim_audio_s *priv = (struct sim_audio_s *)dev;

  sim_audio_close(priv);

  while (!dq_empty(&priv->pendq))
    {
      struct ap_buffer_s *apb;

      apb = (struct ap_buffer_s *)dq_remfirst(&priv->pendq);
#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
    }

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  apb_free(priv->aux);
  priv->aux = NULL;

  priv->ops->uninit(priv->codec);
  priv->ops = NULL;

  return 0;
}
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int sim_audio_pause(struct audio_lowerhalf_s *dev)
{
  struct sim_audio_s *priv = (struct sim_audio_s *)dev;

  if (!priv->pcm)
    {
      return 0;
    }

  snd_pcm_pause(priv->pcm, 0);

  return 0;
}

static int sim_audio_resume(struct audio_lowerhalf_s *dev)
{
  struct sim_audio_s *priv = (struct sim_audio_s *)dev;

  if (!priv->pcm)
    {
      return 0;
    }

  snd_pcm_resume(priv->pcm);

  return 0;
}
#endif

static int sim_audio_enqueuebuffer(struct audio_lowerhalf_s *dev,
                                   struct ap_buffer_s *apb)
{
  struct sim_audio_s *priv = (struct sim_audio_s *)dev;

  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;
  dq_addlast(&apb->dq_entry, &priv->pendq);

  return 0;
}

static int sim_audio_ioctl(struct audio_lowerhalf_s *dev, int cmd,
                           unsigned long arg)
{
  struct sim_audio_s *priv = (struct sim_audio_s *)dev;
  int ret = 0;

  switch (cmd)
    {
      case AUDIOIOC_SETBUFFERINFO:
        {
          struct ap_buffer_info_s *info =
              (struct ap_buffer_info_s *)arg;

          priv->nbuffers    = info->nbuffers;
          priv->buffer_size = info->buffer_size;
        }
        break;

      case AUDIOIOC_GETBUFFERINFO:
        {
          struct ap_buffer_info_s *info =
              (struct ap_buffer_info_s *)arg;

          info->nbuffers    = priv->nbuffers;
          info->buffer_size = priv->buffer_size;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int sim_audio_reserve(struct audio_lowerhalf_s *dev)
{
  return 0;
}

static int sim_audio_release(struct audio_lowerhalf_s *dev)
{
  return 0;
}

static int sim_audio_process_playback(struct sim_audio_s *priv,
                                      struct ap_buffer_s *apb,
                                      bool *dequeue)
{
  struct ap_buffer_s *aux = priv->aux;
  uint8_t *out = NULL;
  uint8_t *in = NULL;
  uint32_t outsize;
  uint32_t insize;
  int ret;

  /* 1, copy apb buffer to aux when the apb buffer just enqueued.
   * */

  if (apb->flags & AUDIO_APB_OUTPUT_ENQUEUED)
    {
      memcpy(aux->samp + aux->nbytes, apb->samp, apb->nbytes);
      aux->nbytes += apb->nbytes;
      apb->flags  &= ~AUDIO_APB_OUTPUT_ENQUEUED;
    }

  in     = aux->samp   + aux->curbyte;
  insize = aux->nbytes - aux->curbyte;

  /* 2, decode or passthrough. */

  ret = priv->ops->process(priv->codec, in, insize, &out, &outsize);
  if (ret < 0)
    {
      /* 3, if return -ENODATA, means there is no enough data in aux apb.
       * memmove the remaining data to aux->samp.
       * */

      if (ret == -ENODATA)
        {
            memmove(aux->samp, in, insize);
            aux->curbyte = 0;
            aux->nbytes  = insize;
            *dequeue     = true;
        }
      else
        {
          /* 4, if other error, increase apb->curbyte and try again. */

          aux->curbyte++;
        }

      ret = 0;
    }
  else
    {
      /* 5, decode success, process remain data. write to alsa. */

      aux->curbyte += ret;

      ret  = snd_pcm_writei(priv->pcm, out, outsize / priv->frame_size);
      ret *= priv->frame_size;
    }

  /* 6, whether send DEQUEUE msg to apps. */

  if (aux->curbyte == aux->nbytes)
    {
      aux->curbyte = 0;
      aux->nbytes  = 0;
      *dequeue     = true;
    }

  /* 7, return actual bytes which write to alsa driver. */

  return ret;
}

static void sim_audio_process(struct sim_audio_s *priv)
{
  snd_pcm_sframes_t expect;
  struct ap_buffer_s *apb;
  snd_pcm_sframes_t avail;
  bool dequeue = false;
  uint32_t outsize;
  int ret = 0;

  if (!priv->pcm)
    {
      return;
    }

  apb = (struct ap_buffer_s *)dq_peek(&priv->pendq);
  if (!apb)
    {
      return;
    }

  if (priv->ops->get_samples)
    {
      expect = priv->ops->get_samples(priv->codec);
    }
  else
    {
      expect = priv->playback ? apb->nbytes / priv->frame_size
                              : AUDMIN(apb->nmaxbytes, priv->buffer_size)
                              / priv->frame_size;
    }

  avail = snd_pcm_avail(priv->pcm);
  if (avail < expect)
    {
      ret = avail;
      goto out;
    }

  if (priv->playback)
    {
      ret = sim_audio_process_playback(priv, apb, &dequeue);
    }
  else
    {
      ret = snd_pcm_readi(priv->pcm, apb->samp, expect);
      if (ret < 0)
        {
          goto out;
        }

      ret = priv->ops->process(priv->codec, apb->samp,
                               ret * priv->frame_size, &apb->samp, &outsize);
      if (ret < 0)
        {
          return;
        }

      apb->curbyte += ret;
      dequeue = true;
    }

  if (dequeue)
    {
      bool final = false;

      dq_remfirst(&priv->pendq);

      apb->nbytes = apb->curbyte;
      if (apb->flags & AUDIO_APB_FINAL)
        {
          final = true;
        }

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif

      if (final)
        {
          snd_pcm_drain(priv->pcm);
          sim_audio_stop(&priv->dev);
        }
    }

out:
  if (ret == -EPIPE)
    {
      awarn("ALSA buffer xrun.\n");
      snd_pcm_prepare(priv->pcm);
      snd_pcm_start(priv->pcm);
    }
  else if (ret < 0 && ret != -EAGAIN)
    {
      aerr("pcm writei/readi failed %d, %s\n", ret, snd_strerror(ret));
    }
}

static int sim_mixer_open(struct sim_audio_s *priv)
{
  snd_mixer_selem_id_t *sid = NULL;
  irqstate_t flags = up_irq_save();
  int ret;

  ret = snd_mixer_open(&priv->mixer, 0);
  if (ret < 0)
    {
      goto fail;
    }

  ret = snd_mixer_attach(priv->mixer, "default");
  if (ret < 0)
    {
      goto fail;
    }

  ret = snd_mixer_selem_register(priv->mixer, NULL, NULL);
  if (ret < 0)
    {
      goto fail;
    }

  ret = snd_mixer_load(priv->mixer);
  if (ret < 0)
    {
      goto fail;
    }

  ret = snd_mixer_selem_id_malloc(&sid);
  if (ret < 0)
    {
      goto fail;
    }

  if (priv->playback)
    {
      snd_mixer_selem_id_set_index(sid, 0);
      snd_mixer_selem_id_set_name(sid, "Master");

      priv->volume = snd_mixer_find_selem(priv->mixer, sid);
      snd_mixer_selem_id_free(sid);
      if (!priv->volume)
        {
          goto fail;
        }

      ret = snd_mixer_selem_set_playback_volume_range(priv->volume, 0, 1000);
      if (ret < 0)
        {
          goto fail;
        }
    }
  else
    {
      snd_mixer_selem_id_set_index(sid, 0);
      snd_mixer_selem_id_set_name(sid, "Capture");

      priv->volume = snd_mixer_find_selem(priv->mixer, sid);
      snd_mixer_selem_id_free(sid);
      if (!priv->volume)
        {
          goto fail;
        }

      ret = snd_mixer_selem_set_capture_volume_range(priv->volume, 0, 1000);
      if (ret < 0)
        {
          goto fail;
        }
    }

  up_irq_restore(flags);
  return 0;
fail:
  snd_mixer_close(priv->mixer);
  up_irq_restore(flags);
  priv->mixer = NULL;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sim_audio_loop(void)
{
  sq_entry_t *entry;

  for (entry = sq_peek(&g_sim_audio); entry; entry = sq_next(entry))
    {
      struct sim_audio_s *priv =
        container_of(entry, struct sim_audio_s, link);

      sim_audio_process(priv);
    }
}

struct audio_lowerhalf_s *sim_audio_initialize(bool playback, bool offload)
{
  struct sim_audio_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct sim_audio_s));
  if (!priv)
    {
      return NULL;
    }

  priv->playback = playback;
  priv->offload  = offload;
  priv->dev.ops  = &g_sim_audio_ops;

  ret = sim_mixer_open(priv);
  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }

  sq_addlast(&priv->link, &g_sim_audio);

  /* Setting default config */

  priv->nbuffers    = CONFIG_AUDIO_NUM_BUFFERS;
  priv->buffer_size = CONFIG_AUDIO_BUFFER_NUMBYTES;

  priv->sample_rate = 48000;
  priv->channels    = 2;
  priv->bps         = 16;
  priv->frame_size  = 4;

  return &priv->dev;
}
