/****************************************************************************
 * drivers/audio/wm8776.h
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

#ifndef __DRIVERS_AUDIO_WM8776_H
#define __DRIVERS_AUDIO_WM8776_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <pthread.h>
#include <mqueue.h>

#include <nuttx/wqueue.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WM8776_MASTER_ATT   0x02
#define WM8776_DAC_CC       0x07
#define WM8776_DAC_IF       0x0a
#define WM8776_MASTER_MODE  0x0c
#define WM8776_PWR_DOWN     0x0d
#define WM8776_SOFT_RESET   0x17

#define WM8776_DEFAULT_SAMPRATE      44100     /* Initial sample rate */
#define WM8776_DEFAULT_NCHANNELS     2         /* Initial number of channels */
#define WM8776_DEFAULT_BPSAMP        16        /* Initial bits per sample */

#define WM8776_HPLZCEN (0x1 << 7)
#define WM8776_UPDATE  (0x1 << 8)

#define WM8776_HPOUT_VOL_SHIFT       (0)       /* Bits 0-6: Headphone output volume */
#define WM8776_HPOUT_VOL_MASK        (0x7f << WM8776_HPOUT_VOL_SHIFT)
#  define WM8776_HPOUT_VOL(n)        ((uint16_t)(n) << WM8776_HPOUT_VOL_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct wm8776_dev_s
{
  /* We are an audio lower half driver (We are also the upper "half" of
   * the WM8776 driver with respect to the board lower half driver).
   *
   * Terminology:
   * Our "lower" half audio instances will be called dev for the publicly
   * visible version and "priv" for the version that only this driver knows
   * From the point of view of this driver, it is the board lower "half"
   * that is referred to as "lower".
   */

  struct audio_lowerhalf_s dev;             /* WM8776 audio lower half (this device) */

  const FAR struct wm8776_lower_s *lower;   /* Pointer to the board lower functions */
  FAR struct i2c_master_s *i2c;             /* I2C driver to use */
  FAR struct i2s_dev_s   *i2s;              /* I2S driver to use */
  struct dq_queue_s       pendq;            /* Queue of pending buffers to be sent */
  struct dq_queue_s       doneq;            /* Queue of sent buffers to be returned */
  mqd_t                   mq;               /* Message queue for receiving messages */
  char                    mqname[16];       /* Our message queue name */
  pthread_t               threadid;         /* ID of our thread */
  uint32_t                bitrate;          /* Actual programmed bit rate */
  sem_t                   pendsem;          /* Protect pendq */
  uint16_t                samprate;         /* Configured samprate (samples/sec) */
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  uint16_t                balance;          /* Current balance level (b16) */
#endif /* CONFIG_AUDIO_EXCLUDE_BALANCE */
  uint8_t                 volume;           /* Current volume level {0..63} */
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */
  uint8_t                 nchannels;        /* Number of channels (1 or 2) */
  uint8_t                 bpsamp;           /* Bits per sample (8 or 16) */
  volatile uint8_t        inflight;         /* Number of audio buffers in-flight */
  bool                    running;          /* True: Worker thread is running */
  bool                    paused;           /* True: Playing is paused */
  bool                    mute;             /* True: Output is muted */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  bool                    terminating;      /* True: Stop requested */
#endif
  bool                    reserved;         /* True: Device is reserved */
  volatile int            result;           /* The result of the last transfer */
};

#endif /* CONFIG_AUDIO */
#endif /* __DRIVERS_AUDIO_WM8776_H */
