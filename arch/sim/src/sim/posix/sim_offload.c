/****************************************************************************
 * arch/sim/src/sim/posix/sim_offload.c
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

#include <nuttx/kmalloc.h>
#include <debug.h>

#include <mad.h>

#include "sim_offload.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void *sim_audio_mp3_init(struct audio_info_s *info);
static int   sim_audio_mp3_samples(void *handle);
static int   sim_audio_mp3_decode(void *handle,
                                  uint8_t *in, uint32_t insize,
                                  uint8_t **out, uint32_t *outsize);
static void  sim_audio_mp3_uninit(void *handle);

static void *sim_audio_pcm_init(struct audio_info_s *info);
static int   sim_audio_pcm_process(void *handle,
                                   uint8_t *in, uint32_t insize,
                                   uint8_t **out, uint32_t *outsize);
static void  sim_audio_pcm_uninit(void *handle);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sim_codec_ops_s g_codec_ops[] =
{
  {
      AUDIO_FMT_PCM,
      AUDCODEC_DEC | AUDCODEC_ENC,
      sim_audio_pcm_init,
      NULL,
      sim_audio_pcm_process,
      sim_audio_pcm_uninit,
  },
  {
      AUDIO_FMT_MP3,
      AUDCODEC_DEC,
      sim_audio_mp3_init,
      sim_audio_mp3_samples,
      sim_audio_mp3_decode,
      sim_audio_mp3_uninit
  },
  {
      AUDIO_FMT_UNDEF,
      0,
      NULL,
      NULL,
      NULL,
      NULL
  },
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_decoder_mp3_s
{
  uint8_t *out;
  struct mad_stream stream;
  struct mad_frame frame;
  struct mad_synth synth;
};

struct sim_codec_pcm_s
{
  uint32_t frame_size;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_mpa_freq_tab[3] =
{
  44100, 48000, 32000
};

static const uint16_t g_mpa_bitrate_tab[2][3][15] =
{
  {
    {
      0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448
    },
    {
      0, 32, 48, 56,  64,  80,  96, 112, 128, 160, 192, 224, 256, 320, 384
    },
    {
      0, 32, 40, 48,  56,  64,  80,  96, 112, 128, 160, 192, 224, 256, 320
    }
  },
  {
    {
      0, 32, 48, 56,  64,  80,  96, 112, 128, 144, 160, 176, 192, 224, 256
    },
    {
      0,  8, 16, 24,  32,  40,  48,  56,  64,  80,  96, 112, 128, 144, 160
    },
    {
      0,  8, 16, 24,  32,  40,  48,  56,  64,  80,  96, 112, 128, 144, 160
    }
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sim_audio_mp3_scale(mad_fixed_t sample)
{
  sample += 1L << (MAD_F_FRACBITS - 16);

  if (sample >= MAD_F_ONE)
    {
      sample = MAD_F_ONE - 1;
    }
  else if (sample < -MAD_F_ONE)
    {
      sample = -MAD_F_ONE;
    }

  return sample >> (MAD_F_FRACBITS + 1 - 16);
}

static int sim_audio_check_mpeg(uint32_t header)
{
  /* header */

  if ((header & 0xffe00000) != 0xffe00000)
    {
      return -EINVAL;
    }

  /* version check */

  if ((header & (3 << 19)) == 1 << 19)
    {
      return -EINVAL;
    }

  /* layer check */

  if ((header & (3 << 17)) == 0)
    {
      return -EINVAL;
    }

  /* bit rate */

  if ((header & (0xf << 12)) == 0xf << 12)
    {
      return -EINVAL;
    }

  /* frequency */

  if ((header & (3 << 10)) == 3 << 10)
    {
      return -EINVAL;
    }

  return 0;
}

static int sim_audio_mp3_check(uint32_t header)
{
  int sample_rate;
  int frame_size;
  int padding;
  int mpeg25;
  int sr_idx;
  int br_idx;
  int layer;
  int lsf;
  int ret;

  ret = sim_audio_check_mpeg(header);
  if (ret < 0)
    {
      return ret;
    }

  if (header & (1 << 20))
    {
      lsf = (header & (1 << 19)) ? 0 : 1;
      mpeg25 = 0;
    }
  else
    {
      lsf = 1;
      mpeg25 = 1;
    }

  layer   = 4 - ((header >> 17) & 3);
  br_idx  = (header >> 12) & 0xf;
  sr_idx  = (header >> 10) & 3;
  padding = (header >> 9) & 1;

  if (sr_idx >= sizeof(g_mpa_freq_tab) / sizeof(g_mpa_freq_tab[0]) ||
      br_idx >= 0xf)
    {
      return -EINVAL;
    }

  sample_rate = g_mpa_freq_tab[sr_idx] >> (lsf + mpeg25);

  if (br_idx != 0)
    {
      frame_size = g_mpa_bitrate_tab[lsf][layer - 1][br_idx];

      switch (layer)
        {
          case 1:
            frame_size = (frame_size * 12000) / sample_rate;
            frame_size = (frame_size + padding) * 4;
            break;

          case 2:
            frame_size = (frame_size * 144000) / sample_rate;
            frame_size += padding;
            break;

          default:
          case 3:
            frame_size = (frame_size * 144000) / (sample_rate << lsf);
            frame_size += padding;
            break;
        }
    }
  else
    {
      /* if no frame size computed, signal it */

      return -EINVAL;
    }

  return frame_size;
}

static void *sim_audio_mp3_init(struct audio_info_s *info)
{
  struct sim_decoder_mp3_s *codec;

  codec = kmm_malloc(sizeof(struct sim_decoder_mp3_s));
  if (codec == NULL)
    {
      return NULL;
    }

  mad_stream_init(&codec->stream);
  mad_frame_init(&codec->frame);
  mad_synth_init(&codec->synth);

  codec->out = kmm_malloc(sizeof(codec->synth.pcm.samples));
  if (codec->out == NULL)
    {
      goto out;
    }

  return codec;

out:
  mad_synth_finish(&(codec->synth));
  mad_frame_finish(&(codec->frame));
  mad_stream_finish(&(codec->stream));
  kmm_free(codec);

  return NULL;
}

static int sim_audio_mp3_samples(void *handle)
{
  struct sim_decoder_mp3_s *codec = (struct sim_decoder_mp3_s *)handle;

  return sizeof(codec->synth.pcm.samples[0]) / sizeof(mad_fixed_t);
}

static int sim_audio_mp3_decode(void *handle,
                         uint8_t *in, uint32_t insize,
                         uint8_t **out, uint32_t *outsize)
{
  struct sim_decoder_mp3_s *codec = (struct sim_decoder_mp3_s *)handle;
  const mad_fixed_t *right_ch;
  const mad_fixed_t *left_ch;
  int mpa_header;
  int nchannels;
  int nsamples;
  uint8_t *ptr;
  int i = 0;
  int size;
  int ret;

  if (insize < 4)
    {
      return -ENODATA;
    }

  mpa_header = in[0] << 24 | in[1] << 16 | in[2] << 8 | in[3];
  size = sim_audio_mp3_check(mpa_header);
  if (size < 0)
    {
      return -EINVAL;
    }

  if (insize < size + 8)
    {
      return -ENODATA;
    }

  mad_stream_buffer(&codec->stream, in, insize);
  ret = mad_frame_decode(&codec->frame, &codec->stream);
  if (ret < 0)
    {
      aerr("%s mp3 decode failed error %d\n", __func__, codec->stream.error);
      return ret;
    }

  mad_synth_frame(&codec->synth, &codec->frame);

  nchannels = codec->synth.pcm.channels;
  nsamples  = codec->synth.pcm.length;
  left_ch   = codec->synth.pcm.samples[0];
  right_ch  = codec->synth.pcm.samples[1];

  ptr = codec->out;
  while (nsamples--)
    {
      int sample;

      /* output sample(s) in 16-bit signed little-endian PCM */

      sample     = sim_audio_mp3_scale(*left_ch++);
      ptr[i]     = (sample >> 0) & 0xff;
      ptr[i + 1] = (sample >> 8) & 0xff;

      if (nchannels == 2)
        {
          sample     = sim_audio_mp3_scale(*right_ch++);
          ptr[i + 2] = (sample >> 0) & 0xff;
          ptr[i + 3] = (sample >> 8) & 0xff;
        }

      i += sizeof(short) * nchannels;
    }

  *out = ptr;
  *outsize = codec->synth.pcm.length * sizeof(short) * nchannels;

  return size;
}

static void sim_audio_mp3_uninit(void *handle)
{
  struct sim_decoder_mp3_s *codec = (struct sim_decoder_mp3_s *)handle;

  mad_synth_finish(&(codec->synth));
  mad_frame_finish(&(codec->frame));
  mad_stream_finish(&(codec->stream));

  kmm_free(codec->out);
  kmm_free(codec);
}

static void *sim_audio_pcm_init(struct audio_info_s *info)
{
  return kmm_malloc(sizeof(struct sim_codec_pcm_s));
}

static int sim_audio_pcm_process(void *handle,
                          uint8_t *in, uint32_t insize,
                          uint8_t **out, uint32_t *outsize)
{
  *out     = in;
  *outsize = insize;

  return *outsize;
}

static void sim_audio_pcm_uninit(void *handle)
{
  kmm_free(handle);
}
