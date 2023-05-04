/****************************************************************************
 * drivers/audio/es8311.h
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

#ifndef __DRIVERS_AUDIO_ES8311_H
#define __DRIVERS_AUDIO_ES8311_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <pthread.h>

#include <nuttx/mqueue.h>
#include <nuttx/wqueue.h>
#include <nuttx/fs/ioctl.h>

#include "esxxxx_common.h"

#ifdef CONFIG_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers Addresses ******************************************************/

#define ES8311_RESET_REG00              0x00  /* Reset digital, csm, clock manager etc. */

/* Clock Scheme Register definition */

#define ES8311_CLK_MANAGER_REG01        0x01 /* Select CLK src for MCLK, enable clock for codec */
#define ES8311_CLK_MANAGER_REG02        0x02 /* Clk divider and clk multiplier */
#define ES8311_CLK_MANAGER_REG03        0x03 /* ADC fsmode and osr  */
#define ES8311_CLK_MANAGER_REG04        0x04 /* DAC osr */
#define ES8311_CLK_MANAGER_REG05        0x05 /* CLK divider for ADC and DAC */
#define ES8311_CLK_MANAGER_REG06        0x06 /* BCLK inverter and divider */
#define ES8311_CLK_MANAGER_REG07        0x07 /* Tri-state, lrck divider */
#define ES8311_CLK_MANAGER_REG08        0x08 /* LRCK divider */

/* SDP */

#define ES8311_SDPIN_REG09              0x09 /* DAC serial digital port */
#define ES8311_SDPOUT_REG0A             0x0a /* ADC serial digital port */

/* System */

#define ES8311_SYSTEM_REG0B             0x0b /* System */
#define ES8311_SYSTEM_REG0C             0x0c /* System */
#define ES8311_SYSTEM_REG0D             0x0d /* System, power up/down */
#define ES8311_SYSTEM_REG0E             0x0e /* System, power up/down */
#define ES8311_SYSTEM_REG0F             0x0f /* System, low power */
#define ES8311_SYSTEM_REG10             0x10 /* System */
#define ES8311_SYSTEM_REG11             0x11 /* System */
#define ES8311_SYSTEM_REG12             0x12 /* System, Enable DAC */
#define ES8311_SYSTEM_REG13             0x13 /* System */
#define ES8311_SYSTEM_REG14             0x14 /* System, select DMIC, select analog pga gain */

/* ADC */

#define ES8311_ADC_REG15                0x15 /* ADC, adc ramp rate, dmic sense */
#define ES8311_ADC_REG16                0x16 /* ADC */
#define ES8311_ADC_REG17                0x17 /* ADC, volume */
#define ES8311_ADC_REG18                0x18 /* ADC, alc enable and winsize */
#define ES8311_ADC_REG19                0x19 /* ADC, alc maxlevel */
#define ES8311_ADC_REG1A                0x1a /* ADC, alc automute */
#define ES8311_ADC_REG1B                0x1b /* ADC, alc automute, adc hpf s1 */
#define ES8311_ADC_REG1C                0x1c /* ADC, equalizer, hpf s2 */

/* DAC */

#define ES8311_DAC_REG31                0x31 /* DAC, mute */
#define ES8311_DAC_REG32                0x32 /* DAC, volume */
#define ES8311_DAC_REG33                0x33 /* DAC, offset */
#define ES8311_DAC_REG34                0x34 /* DAC, drc enable, drc winsize */
#define ES8311_DAC_REG35                0x35 /* DAC, drc maxlevel, minilevel */
#define ES8311_DAC_REG37                0x37 /* DAC, ramprate */

/* GPIO */

#define ES8311_GPIO_REG44               0x44 /* GPIO, dac2adc for test */
#define ES8311_GP_REG45                 0x45 /* GP CONTROL */

/* CHIP */

#define ES8311_CHD1_REGFD               0xfd /* CHIP ID1 */
#define ES8311_CHD2_REGFE               0xfe /* CHIP ID2 */
#define ES8311_CHVER_REGFF              0xff /* VERSION */

#define ES8311_MAX_REGISTER             0xff

/* Codec Default Parameters *************************************************/

#define ES8311_DEFAULT_SAMPRATE      48000
#define ES8311_DEFAULT_BPSAMP        16

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct es8311_dev_s
{
  /* We are an audio lower half driver (We are also the upper "half" of
   * the ES8311 driver with respect to the board lower half driver).
   *
   * Terminology:
   * Our "lower" half audio instances will be called dev for the publicly
   * visible version and "priv" for the version that only this driver knows
   * From the point of view of this driver, it is the board lower "half"
   * that is referred to as "lower".
   */

  struct audio_lowerhalf_s            dev;              /* ES8311 audio lower half (this device) */

  FAR const struct es8311_lower_s    *lower;            /* Pointer to the board lower functions */
  FAR struct i2c_master_s            *i2c;              /* I2C driver to use */
  FAR struct i2s_dev_s               *i2s;              /* I2S driver to use */
  struct dq_queue_s                   pendq;            /* Queue of pending buffers to be processed */
  struct dq_queue_s                   doneq;            /* Queue of sent buffers to be returned */
  struct file                         mq;               /* Message queue for receiving messages */
  char                                mqname[NAME_MAX]; /* Our message queue name */
  pthread_t                           threadid;         /* ID of our thread */
  mutex_t                             pendlock;         /* Protect pendq */
  uint32_t                            samprate;         /* Configured samprate (samples/sec) */
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
  uint16_t                            volume_out;       /* Current output volume level {0..1000} */
  uint16_t                            volume_in;        /* Current input volume level {0..1000} */
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */
  uint8_t                             bpsamp;           /* Bits per sample */
  volatile uint8_t                    inflight;         /* Number of audio buffers in-flight */
  bool                                running;          /* True: Worker thread is running */
  bool                                paused;           /* True: Playing is paused */
  bool                                mute;             /* True: Output is muted */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  bool                                terminating;      /* True: Stop requested */
#endif
  bool                                reserved;         /* True: Device is reserved */
  volatile int                        result;           /* The result of the last transfer */
  es_module_e                         audio_mode;       /* The current audio mode of the ES8311 chip */
  es_mic_gain_e                       mic_gain;         /* The current microphone gain */
  uint32_t                            mclk;             /* The current MCLK frequency */
};

/* Clock coefficient struct */

struct es8311_coeff_div_s
{
  uint32_t mclk;        /* MCLK frequency */
  uint32_t rate;        /* Sample rate */
  uint8_t pre_div;      /* The pre divider with range from 1 to 8 */
  uint8_t pre_multi;    /* The pre multiplier with x1, x2, x4 and x8 selection */
  uint8_t adc_div;      /* ADCCLK divider */
  uint8_t dac_div;      /* DACCLK divider */
  uint8_t fs_mode;      /* Double speed or single speed */
  uint8_t lrck_h;       /* ADCLRCK divider and DACLRCK divider */
  uint8_t lrck_l;
  uint8_t bclk_div;     /* SCLK divider */
  uint8_t adc_osr;      /* ADC osr */
  uint8_t dac_osr;      /* DAC osr */
};

typedef enum
{
  ES8311_MIC_GAIN_0DB,
  ES8311_MIC_GAIN_6DB,
  ES8311_MIC_GAIN_12DB,
  ES8311_MIC_GAIN_18DB,
  ES8311_MIC_GAIN_24DB,
  ES8311_MIC_GAIN_30DB,
  ES8311_MIC_GAIN_36DB,
  ES8311_MIC_GAIN_42DB
} es8311_mic_gain_e;

typedef enum
{
  ES8311_MCLK_FROM_MCLK_PIN,
  ES8311_MCLK_FROM_SCLK_PIN
} es8311_mclk_src_e;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Codec hifi MCLK clock divider coefficients */

static const struct es8311_coeff_div_s es8311_coeff_div[] =
{
  /* 8k */

  {
    12288000,
    8000,
    0x06,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    18432000,
    8000,
    0x03,
    0x02,
    0x03,
    0x03,
    0x00,
    0x05,
    0xff,
    0x18,
    0x10,
    0x20
  },
  {
    16384000,
    8000,
    0x08,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    8192000,
    8000,
    0x04,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    6144000,
    8000,
    0x03,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    4096000,
    8000,
    0x02,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    3072000,
    8000,
    0x01,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    2048000,
    8000,
    0x01,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    1536000,
    8000,
    0x03,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    1024000,
    8000,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },

  /* 11.025k */

  {
    11289600,
    11025,
    0x04,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    5644800,
    11025,
    0x02,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    2822400,
    11025,
    0x01,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    1411200,
    11025,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },

  /* 12k */

  {
    12288000,
    12000,
    0x04,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    6144000,
    12000,
    0x02,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    3072000,
    12000,
    0x01,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    1536000,
    12000,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },

  /* 16k */

  {
    12288000,
    16000,
    0x03,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    18432000,
    16000,
    0x03,
    0x02,
    0x03,
    0x03,
    0x00,
    0x02,
    0xff,
    0x0c,
    0x10,
    0x20
  },
  {
    16384000,
    16000,
    0x04,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    8192000,
    16000,
    0x02,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    6144000,
    16000,
    0x03,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    4096000,
    16000,
    0x01,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    3072000,
    16000,
    0x03,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    2048000,
    16000,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    1536000,
    16000,
    0x03,
    0x08,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },
  {
    1024000,
    16000,
    0x01,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x20
  },

  /* 22.05k */

  {
    11289600,
    22050,
    0x02,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    5644800,
    22050,
    0x01,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    2822400,
    22050,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    1411200,
    22050,
    0x01,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },

  /* 24k */

  {
    12288000,
    24000,
    0x02,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    18432000,
    24000,
    0x03,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    6144000,
    24000,
    0x01,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    3072000,
    24000,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    1536000,
    24000,
    0x01,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },

  /* 32k */

  {
    12288000,
    32000,
    0x03,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    18432000,
    32000,
    0x03,
    0x04,
    0x03,
    0x03,
    0x00,
    0x02,
    0xff,
    0x0c,
    0x10,
    0x10
  },
  {
    16384000,
    32000,
    0x02,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    8192000,
    32000,
    0x01,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    6144000,
    32000,
    0x03,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    4096000,
    32000,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    3072000,
    32000,
    0x03,
    0x08,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    2048000,
    32000,
    0x01,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    1536000,
    32000,
    0x03,
    0x08,
    0x01,
    0x01,
    0x01,
    0x00,
    0x7f,
    0x02,
    0x10,
    0x10
  },
  {
    1024000,
    32000,
    0x01,
    0x08,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },

  /* 44.1k */

  {
    11289600,
    44100,
    0x01,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    5644800,
    44100,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    2822400,
    44100,
    0x01,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    1411200,
    44100,
    0x01,
    0x08,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },

  /* 48k */

  {
    12288000,
    48000,
    0x01,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    18432000,
    48000,
    0x03,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    6144000,
    48000,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    3072000,
    48000,
    0x01,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    1536000,
    48000,
    0x01,
    0x08,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },

  /* 64k */

  {
    12288000,
    64000,
    0x03,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    18432000,
    64000,
    0x03,
    0x04,
    0x03,
    0x03,
    0x01,
    0x01,
    0x7f,
    0x06,
    0x10,
    0x10
  },
  {
    16384000,
    64000,
    0x01,
    0x01,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    8192000,
    64000,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    6144000,
    64000,
    0x01,
    0x04,
    0x03,
    0x03,
    0x01,
    0x01,
    0x7f,
    0x06,
    0x10,
    0x10
  },
  {
    4096000,
    64000,
    0x01,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    3072000,
    64000,
    0x01,
    0x08,
    0x03,
    0x03,
    0x01,
    0x01,
    0x7f,
    0x06,
    0x10,
    0x10
  },
  {
    2048000,
    64000,
    0x01,
    0x08,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    1536000,
    64000,
    0x01,
    0x08,
    0x01,
    0x01,
    0x01,
    0x00,
    0xbf,
    0x03,
    0x18,
    0x18
  },
  {
    1024000,
    64000,
    0x01,
    0x08,
    0x01,
    0x01,
    0x01,
    0x00,
    0x7f,
    0x02,
    0x10,
    0x10
  },

  /* 88.2k */

  {
    11289600,
    88200,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    5644800,
    88200,
    0x01,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    2822400,
    88200,
    0x01,
    0x08,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    1411200,
    88200,
    0x01,
    0x08,
    0x01,
    0x01,
    0x01,
    0x00,
    0x7f,
    0x02,
    0x10,
    0x10
  },

  /* 96k */

  {
    12288000,
    96000,
    0x01,
    0x02,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    18432000,
    96000,
    0x03,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    6144000,
    96000,
    0x01,
    0x04,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    3072000,
    96000,
    0x01,
    0x08,
    0x01,
    0x01,
    0x00,
    0x00,
    0xff,
    0x04,
    0x10,
    0x10
  },
  {
    1536000,
    96000,
    0x01,
    0x08,
    0x01,
    0x01,
    0x01,
    0x00,
    0x7f,
    0x02,
    0x10,
    0x10
  }
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: es8311_readreg
 *
 * Description:
 *    Read the specified 8-bit register from the ES8311 device.
 *
 ****************************************************************************/

#if defined(CONFIG_ES8311_REGDUMP)
struct es8311_dev_s;
uint8_t es8311_readreg(FAR struct es8311_dev_s *priv, uint8_t regaddr);
#endif

#endif /* CONFIG_AUDIO */
#endif /* __DRIVERS_AUDIO_ES8311_H */
