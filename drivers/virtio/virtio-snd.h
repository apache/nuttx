/****************************************************************************
 * drivers/virtio/virtio-snd.h
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

#ifndef __DRIVERS_VIRTIO_VIRTIO_SOUND_H
#define __DRIVERS_VIRTIO_VIRTIO_SOUND_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#ifdef CONFIG_DRIVERS_VIRTIO_SOUND

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_SND_CHMAP_MAX_SIZE 18

enum
{
  /* Device virtqueue indexes */

  VIRTIO_SND_VQ_CONTROL = 0,
  VIRTIO_SND_VQ_EVENT,
  VIRTIO_SND_VQ_TX,
  VIRTIO_SND_VQ_RX,

  /* Of device virtqueues */

  VIRTIO_SND_VQ_MAX
};

/* Supported dataflow directions */

enum
{
  VIRTIO_SND_D_OUTPUT = 0,
  VIRTIO_SND_D_INPUT
};

enum
{
  /* Jack control request types */

  VIRTIO_SND_R_JACK_INFO = 1,
  VIRTIO_SND_R_JACK_REMAP,

  /* Pcm control request types */

  VIRTIO_SND_R_PCM_INFO = 0x0100,
  VIRTIO_SND_R_PCM_SET_PARAMS,
  VIRTIO_SND_R_PCM_PREPARE,
  VIRTIO_SND_R_PCM_RELEASE,
  VIRTIO_SND_R_PCM_START,
  VIRTIO_SND_R_PCM_STOP,

  /* Channel map control request types */

  VIRTIO_SND_R_CHMAP_INFO = 0x0200,

  /* Jack event types */

  VIRTIO_SND_EVT_JACK_CONNECTED = 0x1000,
  VIRTIO_SND_EVT_JACK_DISCONNECTED,

  /* Pcm event types */

  VIRTIO_SND_EVT_PCM_PERIOD_ELAPSED = 0x1100,
  VIRTIO_SND_EVT_PCM_XRUN,

  /* Common status codes */

  VIRTIO_SND_S_OK = 0x8000,
  VIRTIO_SND_S_BAD_MSG,
  VIRTIO_SND_S_NOT_SUPP,
  VIRTIO_SND_S_IO_ERR
};

/* Virtio snd configuration */

struct virtio_snd_config
{
  uint32_t jacks;
  uint32_t streams;
  uint32_t chmaps;
};

/* Common header */

struct virtio_snd_hdr
{
  uint32_t code;
};

/* Event notification */

struct virtio_snd_event
{
  struct virtio_snd_hdr hdr;
  uint32_t data;
};

/* Common control request to query an item information */

struct virtio_snd_query_info
{
  struct virtio_snd_hdr hdr;
  uint32_t start_id;
  uint32_t count;
  uint32_t size;
};

/* Common item information header */

struct virtio_snd_info
{
  uint32_t hda_fn_nid;
};

/* Jack control messages */

struct virtio_snd_jack_hdr
{
  struct virtio_snd_hdr hdr;
  uint32_t jack_id;
};

/* Supported jack features */

enum
{
  VIRTIO_SND_JACK_F_REMAP = 0
};

/* Query jack info,device->driver */

struct virtio_snd_jack_info
{
  struct virtio_snd_info hdr;
  uint32_t features;
  uint32_t hda_reg_defconf;
  uint32_t hda_reg_caps;
  uint8_t connected;
  uint8_t padding[7];
};

/* Jack remapping control request */

struct virtio_snd_jack_remap
{
  struct virtio_snd_jack_hdr hdr;
  uint32_t association;
  uint32_t sequence;
};

/* Pcm control messages */

struct virtio_snd_pcm_hdr
{
  struct virtio_snd_hdr hdr;
  uint32_t stream_id;
};

/* Supported PCM stream features */

enum
{
  VIRTIO_SND_PCM_F_SHMEM_HOST = 0,
  VIRTIO_SND_PCM_F_SHMEM_GUEST,
  VIRTIO_SND_PCM_F_MSG_POLLING,
  VIRTIO_SND_PCM_F_EVT_SHMEM_PERIODS,
  VIRTIO_SND_PCM_F_EVT_XRUNS
};

/* Supported PCM sample formats */

enum
{
  VIRTIO_SND_PCM_FMT_IMA_ADPCM = 0,  /*  4 /  4 bits */
  VIRTIO_SND_PCM_FMT_MU_LAW,         /*  8 /  8 bits */
  VIRTIO_SND_PCM_FMT_A_LAW,          /*  8 /  8 bits */
  VIRTIO_SND_PCM_FMT_S8,             /*  8 /  8 bits */
  VIRTIO_SND_PCM_FMT_U8,             /*  8 /  8 bits */
  VIRTIO_SND_PCM_FMT_S16,            /* 16 / 16 bits */
  VIRTIO_SND_PCM_FMT_U16,            /* 16 / 16 bits */
  VIRTIO_SND_PCM_FMT_S18_3,          /* 18 / 24 bits */
  VIRTIO_SND_PCM_FMT_U18_3,          /* 18 / 24 bits */
  VIRTIO_SND_PCM_FMT_S20_3,          /* 20 / 24 bits */
  VIRTIO_SND_PCM_FMT_U20_3,          /* 20 / 24 bits */
  VIRTIO_SND_PCM_FMT_S24_3,          /* 24 / 24 bits */
  VIRTIO_SND_PCM_FMT_U24_3,          /* 24 / 24 bits */
  VIRTIO_SND_PCM_FMT_S20,            /* 20 / 32 bits */
  VIRTIO_SND_PCM_FMT_U20,            /* 20 / 32 bits */
  VIRTIO_SND_PCM_FMT_S24,            /* 24 / 32 bits */
  VIRTIO_SND_PCM_FMT_U24,            /* 24 / 32 bits */
  VIRTIO_SND_PCM_FMT_S32,            /* 32 / 32 bits */
  VIRTIO_SND_PCM_FMT_U32,            /* 32 / 32 bits */
  VIRTIO_SND_PCM_FMT_FLOAT,          /* 32 / 32 bits */
  VIRTIO_SND_PCM_FMT_FLOAT64,        /* 64 / 64 bits */
  VIRTIO_SND_PCM_FMT_DSD_U8,         /*  8 /  8 bits */
  VIRTIO_SND_PCM_FMT_DSD_U16,        /* 16 / 16 bits */
  VIRTIO_SND_PCM_FMT_DSD_U32,        /* 32 / 32 bits */
  VIRTIO_SND_PCM_FMT_IEC958_SUBFRAME /* 32 / 32 bits */
};

/* Supported PCM frame rates */

enum
{
  VIRTIO_SND_PCM_RATE_5512 = 0,
  VIRTIO_SND_PCM_RATE_8000,
  VIRTIO_SND_PCM_RATE_11025,
  VIRTIO_SND_PCM_RATE_16000,
  VIRTIO_SND_PCM_RATE_22050,
  VIRTIO_SND_PCM_RATE_32000,
  VIRTIO_SND_PCM_RATE_44100,
  VIRTIO_SND_PCM_RATE_48000,
  VIRTIO_SND_PCM_RATE_64000,
  VIRTIO_SND_PCM_RATE_88200,
  VIRTIO_SND_PCM_RATE_96000,
  VIRTIO_SND_PCM_RATE_176400,
  VIRTIO_SND_PCM_RATE_192000,
  VIRTIO_SND_PCM_RATE_384000
};

/* Query pcm info,device->driver */

struct virtio_snd_pcm_info
{
  struct virtio_snd_info hdr;
  uint32_t features;
  uint64_t formats;
  uint64_t rates;
  uint8_t direction;
  uint8_t channels_min;
  uint8_t channels_max;
  uint8_t padding[5];
};

/* Set pcm stream format */

struct virtio_snd_pcm_set_params
{
  struct virtio_snd_pcm_hdr hdr;
  uint32_t buffer_bytes;
  uint32_t period_bytes;
  uint32_t features;
  uint8_t channels;
  uint8_t format;
  uint8_t rate;
  uint8_t padding;
};

/* I/O request header */

struct virtio_snd_pcm_xfer
{
  uint32_t stream_id;
};

/* I/O request status */

struct virtio_snd_pcm_status
{
  uint32_t status;
  uint32_t latency_bytes;
};

/* Standard channel position definition */

enum
{
  VIRTIO_SND_CHMAP_NONE = 0,    /* Undefined */
  VIRTIO_SND_CHMAP_NA,          /* Silent */
  VIRTIO_SND_CHMAP_MONO,        /* Mono stream */
  VIRTIO_SND_CHMAP_FL,          /* Front left */
  VIRTIO_SND_CHMAP_FR,          /* Front right */
  VIRTIO_SND_CHMAP_RL,          /* Rear left */
  VIRTIO_SND_CHMAP_RR,          /* Rear right */
  VIRTIO_SND_CHMAP_FC,          /* Front center */
  VIRTIO_SND_CHMAP_LFE,         /* Low frequency (LFE) */
  VIRTIO_SND_CHMAP_SL,          /* Side left */
  VIRTIO_SND_CHMAP_SR,          /* Side right */
  VIRTIO_SND_CHMAP_RC,          /* Rear center */
  VIRTIO_SND_CHMAP_FLC,         /* Front left center */
  VIRTIO_SND_CHMAP_FRC,         /* Front right center */
  VIRTIO_SND_CHMAP_RLC,         /* Rear left center */
  VIRTIO_SND_CHMAP_RRC,         /* Rear right center */
  VIRTIO_SND_CHMAP_FLW,         /* Front left wide */
  VIRTIO_SND_CHMAP_FRW,         /* Front right wide */
  VIRTIO_SND_CHMAP_FLH,         /* Front left high */
  VIRTIO_SND_CHMAP_FCH,         /* Front center high */
  VIRTIO_SND_CHMAP_FRH,         /* Front right high */
  VIRTIO_SND_CHMAP_TC,          /* Top center */
  VIRTIO_SND_CHMAP_TFL,         /* Top front left */
  VIRTIO_SND_CHMAP_TFR,         /* Top front right */
  VIRTIO_SND_CHMAP_TFC,         /* Top front center */
  VIRTIO_SND_CHMAP_TRL,         /* Top rear left */
  VIRTIO_SND_CHMAP_TRR,         /* Top rear right */
  VIRTIO_SND_CHMAP_TRC,         /* Top rear center */
  VIRTIO_SND_CHMAP_TFLC,        /* Top front left center */
  VIRTIO_SND_CHMAP_TFRC,        /* Top front right center */
  VIRTIO_SND_CHMAP_TSL,         /* Top side left */
  VIRTIO_SND_CHMAP_TSR,         /* Top side right */
  VIRTIO_SND_CHMAP_LLFE,        /* Left LFE */
  VIRTIO_SND_CHMAP_RLFE,        /* Right LFE */
  VIRTIO_SND_CHMAP_BC,          /* Bottom center */
  VIRTIO_SND_CHMAP_BLC,         /* Bottom left center */
  VIRTIO_SND_CHMAP_BRC          /* Bottom right center */
};

/* Query channel map info,device->driver */

struct virtio_snd_chmap_info
{
  struct virtio_snd_info hdr;
  uint8_t direction;
  uint8_t channels;
  uint8_t positions[VIRTIO_SND_CHMAP_MAX_SIZE];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int virtio_register_snd_driver(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_VIRTIO_SOUND */
#endif /* __DRIVERS_VIRTIO_VIRTIO_SOUND_H */
