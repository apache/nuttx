/****************************************************************************
 * drivers/audio/cxd56.h
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

#ifndef __DRIVERS_AUDIO_CXD56_H
#define __DRIVERS_AUDIO_CXD56_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <pthread.h>
#include <mqueue.h>

#include <nuttx/audio/audio.h>
#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spinlock.h>

#ifdef CONFIG_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CXD56_AUDIO_WORKER_STACKSIZE
#  define CONFIG_CXD56_AUDIO_WORKER_STACKSIZE  768
#endif

#ifndef CONFIG_CXD56_MSG_PRIO
#  define CONFIG_CXD56_MSG_PRIO  1
#endif

#ifdef CONFIG_CXD56_AUDIO_I2S_DEVICE_1_MASTER
#  define CXD56_I2S1_MODE    0    /* Master */
#else
#  define CXD56_I2S1_MODE    1    /* Slave */
#endif

#ifdef CONFIG_CXD56_AUDIO_I2S_FORMAT_1_LEFT
#  define CXD56_I2S1_FORMAT  1    /* Left */
#else
#  define CXD56_I2S1_FORMAT  0    /* Normal */
#endif

#ifdef CONFIG_CXD56_AUDIO_I2S_BYPASS_MODE_1_ENABLE
#  define CXD56_I2S1_BYPASS  0    /* Disable */
#else
#  define CXD56_I2S1_BYPASS  1    /* Enable */
#endif

/* I2S data rate of I2S1 */

#if defined(CONFIG_CXD56_I2S0)
#  define CXD56_I2S1_DATA_RATE   CONFIG_CXD56_AUDIO_I2S_RATE_1
#else
#  define CXD56_I2S1_DATA_RATE   0
#endif

#define CXD56_XTAL_24_576MHZ  0
#define CXD56_XTAL_49_152MHZ  1

#ifdef CONFIG_CXD56_AUDIO_XTAL_SEL_49_152MHZ
#  define CXD56_AUDIO_MCLK   CXD56_XTAL_49_152MHZ
#else
#  define CXD56_AUDIO_MCLK   CXD56_XTAL_24_576MHZ
#endif

/* Drive strength levels */
#define CXD56_DS_WEAKEST    1
#define CXD56_DS_WEAKER     2
#define CXD56_DS_STRONGER   3
#define CXD56_DS_STRONGEST  4

/* Drive strength of global pin output-A */
#if defined(CONFIG_CXD56_AUDIO_GPO_A_WEAKEST)
#  define CXD56_GPO_A_DS    CXD56_DS_WEAKEST
#elif defined(CONFIG_CXD56_AUDIO_GPO_A_WEAKER)
#  define CXD56_GPO_A_DS    CXD56_DS_WEAKER
#elif defined(CONFIG_CXD56_AUDIO_GPO_A_STRONGER)
#  define CXD56_GPO_A_DS    CXD56_DS_STRONGER
#else
#  define CXD56_GPO_A_DS    CXD56_DS_STRONGEST
#endif

/* Drive strength of D/A converted data */
#if defined(CONFIG_CXD56_AUDIO_DA_DATA_WEAKEST)
#  define CXD56_DA_DS    CXD56_DS_WEAKEST
#elif defined(CONFIG_CXD56_AUDIO_DA_DATA_WEAKER)
#  define CXD56_DA_DS    CXD56_DS_WEAKER
#elif defined(CONFIG_CXD56_AUDIO_DA_DATA_STRONGER)
#  define CXD56_DA_DS    CXD56_DS_STRONGER
#else
#  define CXD56_DA_DS    CXD56_DS_STRONGEST
#endif

/* Drive strength of digital mic clock */
#if defined(CONFIG_CXD56_AUDIO_CLKOUT_DMIC_WEAKEST)
#  define CXD56_DMIC_CLK_DS    CXD56_DS_WEAKEST
#elif defined(CONFIG_CXD56_AUDIO_CLKOUT_DMIC_WEAKER)
#  define CXD56_DMIC_CLK_DS    CXD56_DS_WEAKER
#elif defined(CONFIG_CXD56_AUDIO_CLKOUT_DMIC_STRONGER)
#  define CXD56_DMIC_CLK_DS    CXD56_DS_STRONGER
#else
#  define CXD56_DMIC_CLK_DS    CXD56_DS_STRONGEST
#endif

/* Drive strength of master clock */
#if defined(CONFIG_CXD56_AUDIO_MCLKOUT_WEAKEST)
#  define CXD56_MCLKOUT_DS    CXD56_DS_WEAKEST
#elif defined(CONFIG_CXD56_AUDIO_MCLKOUT_WEAKER)
#  define CXD56_MCLKOUT_DS    CXD56_DS_WEAKER
#elif defined(CONFIG_CXD56_AUDIO_MCLKOUT_STRONGER)
#  define CXD56_MCLKOUT_DS    CXD56_DS_STRONGER
#else
#  define CXD56_MCLKOUT_DS    CXD56_DS_STRONGEST
#endif

/* Speaker time split on drive */
#if defined(CONFIG_CXD56_AUDIO_SP_SPLIT_LONGEST)
#  define CXD56_SP_SPLIT_ON    4
#elif defined(CONFIG_CXD56_AUDIO_SP_SPLIT_LONG)
#  define CXD56_SP_SPLIT_ON    3
#elif defined(CONFIG_CXD56_AUDIO_SP_SPLIT_SHORT)
#  define CXD56_SP_SPLIT_ON    2
#else
#  define CXD56_SP_SPLIT_ON    1
#endif

/* Speaker drive mode */

#if defined(CONFIG_CXD56_AUDIO_SP_DRV_LINEOUT)
#  define CXD56_SP_DRIVER   0
#elif defined(CONFIG_CXD56_AUDIO_SP_DRV_1DRIVERT)
#  define CXD56_SP_DRIVER   1
#elif defined(CONFIG_CXD56_AUDIO_SP_DRV_2DRIVERT)
#  define CXD56_SP_DRIVER   2
#else
#  define CXD56_SP_DRIVER   3
#endif

/* Mic bias voltage select */

#define  CXD56_MIC_BIAS_20V  1
#define  CXD56_MIC_BIAS_28V  2

#if defined(CONFIG_CXD56_AUDIO_MICBIAS_20V)
#  define CXD56_MIC_BIAS  CXD56_MIC_BIAS_20V
#else
#  define CXD56_MIC_BIAS  CXD56_MIC_BIAS_28V
#endif

/* Mic select */

#define CXD56_AUDIO_CFG_MIC CONFIG_CXD56_AUDIO_MIC_CHANNEL_SEL

/* Mic boot wait time */

#if defined(CONFIG_CXD56_AUDIO_MIC_BOOT_WAIT)
#define CXD56_MIC_BOOT_WAIT  CONFIG_CXD56_AUDIO_MIC_BOOT_WAIT
#else
#define CXD56_MIC_BOOT_WAIT  1100
#endif

/* CIC filter input path */

#define CXD56_AUDIO_CFG_CIC_IN_SEL_NONE   0
#define CXD56_AUDIO_CFG_CIC_IN_SEL_CXD    1
#define CXD56_AUDIO_CFG_CIC_IN_SEL_DMICIF 2

#if defined(CONFIG_CXD56_AUDIO_CIC_IN_SEL_CXD)
#  define CXD56_AUDIO_CFG_CIC_IN  CXD56_AUDIO_CFG_CIC_IN_SEL_CXD
#elif defined (CONFIG_CXD56_AUDIO_CIC_IN_SEL_DMIC)
#  define CXD56_AUDIO_CFG_CIC_IN  CXD56_AUDIO_CFG_CIC_IN_SEL_DMICIF
#else
#  define CXD56_AUDIO_CFG_CIC_IN  CXD56_AUDIO_CFG_CIC_IN_SEL_NONE
#endif

/* DMA format */

#define CXD56_DMA_FORMAT_LR 0
#define CXD56_DMA_FORMAT_RL 1

#if defined(CONFIG_CXD56_AUDIO_DMA_DATA_FORMAT_LR)
#  define CXD56_DMA_FORMAT  CXD56_DMA_FORMAT_LR
#else
#  define CXD56_DMA_FORMAT  CXD56_DMA_FORMAT_RL
#endif

/* Audio buffer configuration */

#ifndef CONFIG_CXD56_AUDIO_BUFFER_SIZE
#  define CONFIG_CXD56_AUDIO_BUFFER_SIZE  4096
#endif

#ifndef CONFIG_CXD56_AUDIO_NUM_BUFFERS
#  define CONFIG_CXD56_AUDIO_NUM_BUFFERS  4
#endif

/* Queue helpers */

#define dq_get(q)    (dq_remfirst(q))
#define dq_put(q,n) (dq_addlast((dq_entry_t*)n,(q)))
#define dq_put_back(q,n) (dq_addfirst((dq_entry_t*)n,(q)))
#define dq_clear(q) \
  do \
    { \
      dq_remlast(q); \
    } \
  while (!dq_empty(q))

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum cxd56_dmahandle_e
{
  CXD56_AUDIO_DMA_MIC,
  CXD56_AUDIO_DMA_I2S0_DOWN,
  CXD56_AUDIO_DMA_COUNT
};
typedef enum cxd56_dmahandle_e cxd56_dmahandle_t;

enum cxd56_devstate_e
{
  CXD56_DEV_STATE_OFF,
  CXD56_DEV_STATE_PAUSED,
  CXD56_DEV_STATE_BUFFERING,
  CXD56_DEV_STATE_STARTED,
  CXD56_DEV_STATE_STOPPED
};

struct cxd56_dev_s
{
  /* We are an audio lower half driver (We are also the upper "half" of
   * the CS43L22 driver with respect to the board lower half driver).
   *
   * Terminology:
   * Our "lower" half audio instances will be called dev for the publicly
   * visible version and "priv" for the version that only this driver
   * knows.  From the point of view of this driver, it is the board lower
   * "half" that is referred to as "lower".
   */

  struct audio_lowerhalf_s dev;             /* CXD56 audio lower half (this device) */

  /* Our specific driver data goes here */

  FAR const struct cxd56_lower_s *lower;    /* Pointer to the board lower functions */
  enum cxd56_devstate_e   state;            /* Driver state */
  enum cxd56_dmahandle_e  dma_handle;       /* DMA handle */
  struct file             mq;               /* Message queue for receiving messages */
  char                    mqname[16];       /* Our message queue name */
  pthread_t               threadid;         /* ID of our thread */
  sem_t                   pendsem;          /* Protect pendq */

  struct dq_queue_s       up_pendq;         /* Pending buffers from app to process */
  struct dq_queue_s       up_runq;          /* Buffers from app being played */

#ifdef CONFIG_AUDIO_CXD56_SRC
  struct dq_queue_s       down_pendq;       /* Pending SRC buffers to be DMA'd */
  struct dq_queue_s       down_runq;        /* SRC buffers being processed */
  struct dq_queue_s       down_doneq;       /* Done SRC buffers to be re-used */
#endif

  uint16_t                samplerate;       /* Sample rate */
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
  int16_t                 volume;           /* Output volume {0..63} */
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */
  uint8_t                 channels;         /* Number of channels (1..8) */

  uint16_t                mic_gain;         /* Mic gain */
  uint64_t                mic_boot_start;   /* Mic startup wait time */

  uint8_t                 bitwidth;         /* Bits per sample (16 or 24) */
  bool                    muted;            /* True: Output is muted */

  bool                    running;          /* True: Worker thread is running */
  bool                    paused;           /* True: Playing is paused */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  bool                    terminating;      /* True: Stop requested */
#endif
  bool                    reserved;         /* True: Device is reserved */
  volatile int            result;           /* The result of the last transfer */
  spinlock_t              lock;             /* Spinlock for SMP */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* CONFIG_AUDIO */

#endif /* __DRIVERS_AUDIO_CXD56_H */
