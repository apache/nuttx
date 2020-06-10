/****************************************************************************
 * drivers/audio/cxd56.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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

  const FAR struct cxd56_lower_s *lower;    /* Pointer to the board lower functions */
  enum cxd56_devstate_e   state;            /* Driver state */
  enum cxd56_dmahandle_e  dma_handle;       /* DMA handle */
  mqd_t                   mq;               /* Message queue for receiving messages */
  char                    mqname[16];       /* Our message queue name */
  pthread_t               threadid;         /* ID of our thread */
  sem_t                   pendsem;          /* Protect pendq */

  struct dq_queue_s       pendingq;         /* Queue of pending buffers to be sent */
  struct dq_queue_s       runningq;         /* Queue of buffers being played */

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
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* CONFIG_AUDIO */

#endif /* __DRIVERS_AUDIO_CXD56_H */
