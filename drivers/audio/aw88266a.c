/****************************************************************************
 * drivers/audio/aw88266a.c
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
#include <nuttx/arch.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/aw88266a.h>
#include "aw88266a.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AW88266A_CHIPID             (0x2013)
#define AW88266A_SOFT_RESET         (0x55aa)
#define MAX_RETRIES                 3
#define AW88266A_SYSST_CHECK_MAX    10
#define ARRAY_SIZE(array)           sizeof(array) / sizeof(array[0])

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

typedef enum
{
  DEFAULT_OUTPUT_SPEAKER = 0,
  SCO_OUTPUT,
  VOIP,
}aw88266a_dev_t;

typedef struct
{
  uint8_t  addr;
  uint16_t data;
}aw88266a_reg_cfg_t;

typedef enum
{
  FREQUENCY_08K = 0,
  FREQUENCY_11K = 1,
  FREQUENCY_16K = 2,
  FREQUENCY_22K = 3,
  FREQUENCY_24K = 4,
  FREQUENCY_32K = 5,
  FREQUENCY_44K = 6,
  FREQUENCY_48K = 7,
  FREQUENCY_96K = 8,
  FREQUENCY_192K = 9,
  AW_FREQUENCY_NUM,
}aw_i2s_sample_rate_t;

typedef enum
{
  WIDTH_16BITS = 16,
  WIDTH_24BITS = 24,
  WIDTH_32BITS = 32,
  AW_WIDTH_NUM,
}aw_i2s_width_t;

struct aw88266a_dev_s
{
  /* We are an audio lower half driver (We are also the upper "half" of
   * the AW88266A driver with respect to the board lower half driver).
   *
   * Terminology: Our "lower" half audio instances will be called dev for the
   * publicly visible version and "priv" for the version that only this
   * driver knows.  From the point of view of this driver, it is the board
   * lower "half" that is referred to as "lower".
   */

  struct audio_lowerhalf_s dev;             /* AW88266A audio lower half (this device) */

  /* Our specific driver data goes here */

  const FAR struct aw88266a_lower_s *lower;   /* Pointer to the board lower functions */
  FAR struct i2c_master_s *i2c;               /* I2C driver to use */

  uint16_t                samprate;         /* Configured samprate (samples/sec) */
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  uint16_t                balance;          /* Current balance level (b16) */
#endif  /* CONFIG_AUDIO_EXCLUDE_BALANCE */
  uint8_t                 volume;           /* Current volume level {0..63} */
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */
  uint8_t                 nchannels;        /* Number of channels (1 or 2) */
  uint8_t                 bpsamp;           /* Bits per sample (8 or 16) */
  uint32_t                bclk;             /* IIS BCLK */

  bool                    paused;           /* True: Playing is paused */
  bool                    mute;             /* True: Output is muted */
};

typedef struct
{
  aw88266a_dev_t dev;       /* aw88266a device type */
  FAR char       *dev_str;  /* aw88266a device string */
}aw88266a_dev_info_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void aw88266a_write_reg(FAR struct aw88266a_dev_s *priv,
                               uint8_t regaddr, uint16_t regval);

static void aw88266a_write_reg_bit(FAR struct aw88266a_dev_s *priv,
                                   uint8_t regaddr, uint16_t mask,
                                   uint16_t regval);

static int16_t aw88266a_read_reg(FAR struct aw88266a_dev_s *priv,
                                 uint8_t regaddr);

static int aw88266a_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                            FAR struct audio_caps_s *caps);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
aw88266a_configure(FAR struct audio_lowerhalf_s *dev,
                   FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int aw88266a_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR const struct audio_caps_s *caps);
#endif

static int aw88266a_shutdown(FAR struct audio_lowerhalf_s *dev);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
aw88266a_start(FAR struct audio_lowerhalf_s *dev,
               FAR void *session);
#else
static int aw88266a_start(FAR struct audio_lowerhalf_s *dev);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int
  aw88266a_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#  else
static int aw88266a_stop(FAR struct audio_lowerhalf_s *dev);
#  endif
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int aw88266a_pause(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session);
#  else
static int aw88266a_pause(FAR struct audio_lowerhalf_s *dev);
#  endif
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int aw88266a_resume(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
#  else
static int aw88266a_resume(FAR struct audio_lowerhalf_s *dev);
#  endif
#endif

static int aw88266a_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                          unsigned long arg);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
aw88266a_reserve(FAR struct audio_lowerhalf_s *dev, FAR void **session);
#else
static int aw88266a_reserve(FAR struct audio_lowerhalf_s *dev);
#endif

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int aw88266a_release(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session);
#else
static int aw88266a_release(FAR struct audio_lowerhalf_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const aw88266a_reg_cfg_t g_aw88266a_spk_cfg[] =
{
  {0x03, 0xffff},
  {0x04, 0xb240},
  {0x05, 0x6007},
  {0x06, 0x84e8},
  {0x07, 0x0010},
  {0x08, 0x3940},
  {0x09, 0x0030},
  {0x0a, 0x01e0},
  {0x0b, 0x1c64},
  {0x0c, 0x001b},
  {0x0d, 0x4abb},
  {0x0e, 0x4500},
  {0x10, 0x5700},
  {0x11, 0x2101},
  {0x12, 0x6003},
  {0x38, 0x002c},
  {0x51, 0x00d8},
  {0x52, 0x00e8},
  {0x53, 0x3f08},
  {0x54, 0x0202},
  {0x55, 0x3020},
  {0x56, 0x9411},
  {0x57, 0x1105},
  {0x58, 0x9884},
  {0x60, 0x9b19},
  {0x61, 0x6b3b},
  {0x62, 0x7d30},
  {0x63, 0x3047},
  {0x64, 0xa008},
  {0x65, 0x2abf},
  {0x66, 0x460c},
  {0x67, 0xc989},
  {0x68, 0x3541},
  {0x69, 0x4db8},
  {0x6a, 0xd6f9},
  {0x6b, 0x7ace},
  {0x6c, 0xec7c},
  {0x6d, 0x000c},
};

static const struct audio_ops_s g_audioops =
{
  .getcaps        = aw88266a_getcaps,
  .configure      = aw88266a_configure,
  .shutdown       = aw88266a_shutdown,
  .start          = aw88266a_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop           = aw88266a_stop,
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  .pause          = aw88266a_pause,
  .resume         = aw88266a_resume,
#endif
  .allocbuffer    = NULL,
  .freebuffer     = NULL,
  .enqueuebuffer  = NULL,
  .cancelbuffer   = NULL,
  .ioctl          = aw88266a_ioctl,
  .read           = NULL,
  .write          = NULL,
  .reserve        = aw88266a_reserve,
  .release        = aw88266a_release,
};

static const aw88266a_dev_info_t g_aw88266a_device_mode[] =
{
  {DEFAULT_OUTPUT_SPEAKER,  "speaker"},
  {SCO_OUTPUT,              "sco"},
  {VOIP,                    "voip"}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aw88266a_set_device
 *
 * Description:
 *   set aw88266a device type
 *
 ****************************************************************************/

static void aw88266a_set_device(FAR struct aw88266a_dev_s *priv,
                                aw88266a_dev_t device)
{
  uint8_t i;

  if (device == DEFAULT_OUTPUT_SPEAKER)
    {
      for (i = 0; i < ARRAY_SIZE(g_aw88266a_spk_cfg); i++)
        {
          aw88266a_write_reg(priv, g_aw88266a_spk_cfg[i].addr,
                             g_aw88266a_spk_cfg[i].data);
        }
    }
}

/****************************************************************************
 * Name: aw88266a_run_pwd
 *
 * Description:
 *   set aw88266a power down or work normally
 *
 ****************************************************************************/

static void aw88266a_run_pwd(FAR struct aw88266a_dev_s *priv,
                             bool status)
{
  if (status)
    {
      aw88266a_write_reg_bit(priv, AW88266A_SYSCTRL_REG,
                             AW88266A_PWDN_MASK,
                             AW88266A_PWDN_POWER_DOWN_VALUE);
    }
  else
    {
      aw88266a_write_reg_bit(priv, AW88266A_SYSCTRL_REG,
                             AW88266A_PWDN_MASK,
                             AW88266A_PWDN_WORKING_VALUE);
    }
}

/****************************************************************************
 * Name: aw88266a_set_channel
 *
 * Description:
 *   set aw88266a i2s channel
 *
 ****************************************************************************/

static void aw88266a_set_channel(FAR struct aw88266a_dev_s *priv,
                                 uint8_t channel)
{
  uint16_t  reg_value = 0;

  if (channel == CHSEL_LEFT || channel == CHSEL_RIGHT)
    {
      if (priv->lower->channelfmt == CHSEL_LEFT)
        {
          reg_value = AW88266A_CHSEL_LEFT_VALUE;
        }
      else if (priv->lower->channelfmt == CHSEL_RIGHT)
        {
          reg_value = AW88266A_CHSEL_RIGHT_VALUE;
        }
    }
  else
    {
      reg_value = AW88266A_CHSEL_MONO_VALUE;
    }

  aw88266a_write_reg_bit(priv, AW88266A_I2SCTRL1_REG,
                         AW88266A_CHSEL_MASK, reg_value);
}

/****************************************************************************
 * Name: aw88266a_set_rate
 *
 * Description:
 *   set aw88266a i2s sample rate
 *
 ****************************************************************************/

static void aw88266a_set_rate(FAR struct aw88266a_dev_s *priv,
                              uint16_t rate)
{
  uint16_t  reg_value;

  switch (rate)
  {
    case 8000:
      {
        reg_value = AW88266A_I2SSR_8_KHZ_VALUE;
      }
      break;

    case 11000:
      {
        reg_value = AW88266A_I2SSR_11_KHZ_VALUE;
      }
      break;

    case 16000:
      {
        reg_value = AW88266A_I2SSR_16_KHZ_VALUE;
      }
      break;

    case 22000:
      {
        reg_value = AW88266A_I2SSR_22_KHZ_VALUE;
      }
      break;

    case 24000:
      {
        reg_value = AW88266A_I2SSR_24_KHZ_VALUE;
      }
      break;

    case 32000:
      {
        reg_value = AW88266A_I2SSR_32_KHZ_VALUE;
      }
      break;

    case 44000:
      {
        reg_value = AW88266A_I2SSR_44_KHZ_VALUE;
      }
      break;

    case 48000:
      {
        reg_value = AW88266A_I2SSR_48_KHZ_VALUE;
      }
      break;

    default:
      {
        reg_value = AW88266A_I2SSR_48_KHZ_VALUE;
      }
      break;
  }

  /* set rate */

  aw88266a_write_reg_bit(priv, AW88266A_I2SCTRL1_REG,
                         AW88266A_I2SSR_MASK, reg_value);
}

/****************************************************************************
 * Name: aw88266a_set_width
 *
 * Description:
 *   set aw88266a i2s width
 *
 ****************************************************************************/

static void aw88266a_set_width(FAR struct aw88266a_dev_s *priv,
                               uint8_t width)
{
  uint16_t  reg_value;

  switch (width)
  {
    case 16:
      {
        reg_value = AW88266A_I2SFS_16_BITS_VALUE;
      }
      break;

    case 24:
      {
        reg_value = AW88266A_I2SFS_24_BITS_VALUE;
      }
      break;

    case 32:
      {
        reg_value = AW88266A_I2SFS_32_BITS_VALUE;
      }
      break;

    default:
      {
        reg_value = AW88266A_I2SFS_16_BITS_VALUE;
      }
      break;
  }

  /* set width */

  aw88266a_write_reg_bit(priv, AW88266A_I2SCTRL1_REG,
                         AW88266A_I2SFS_MASK, reg_value);
}

/****************************************************************************
 * Name: aw88266a_set_blck
 *
 * Description:
 *   set aw88266a i2s bclk
 *
 ****************************************************************************/

static void aw88266a_set_blck(FAR struct aw88266a_dev_s *priv,
                              uint8_t bclk_factor)
{
  uint16_t  reg_value;

  switch (bclk_factor)
  {
    case 32:
      {
        reg_value = AW88266A_I2SBCK_32FS_VALUE;
      }
      break;

    case 48:
      {
        reg_value = AW88266A_I2SBCK_48FS_VALUE;
      }
      break;

    case 64:
      {
        reg_value = AW88266A_I2SBCK_64FS_VALUE;
      }
      break;

    default:
      {
        reg_value = AW88266A_I2SBCK_64FS_VALUE;
      }
      break;
  }

  /* set fs */

  aw88266a_write_reg_bit(priv, AW88266A_I2SCTRL1_REG,
                         AW88266A_I2SBCK_MASK, reg_value);
}

/****************************************************************************
 * Name: aw88266a_run_mute
 *
 * Description:
 *   set aw88266a mute or unmute
 *
 ****************************************************************************/

static void aw88266a_run_mute(FAR struct aw88266a_dev_s *priv, bool mute)
{
  if (mute)
    {
      aw88266a_write_reg_bit(priv, AW88266A_SYSCTRL2_REG,
                             AW88266A_HMUTE_MASK,
                             AW88266A_HMUTE_ENABLE_VALUE);
    }
  else
    {
      aw88266a_write_reg_bit(priv, AW88266A_SYSCTRL2_REG,
                             AW88266A_HMUTE_MASK,
                             AW88266A_HMUTE_DISABLE_VALUE);
    }
}

/****************************************************************************
 * Name: aw88266a_pll_check
 *
 * Description:
 *   check aw88266a pll clock
 *
 ****************************************************************************/

static int aw88266a_pll_check(FAR struct aw88266a_dev_s *priv)
{
  int ret;
  uint8_t i;
  uint16_t reg_val;

  for (i = 0; i < AW88266A_SYSST_CHECK_MAX; i++)
    {
      reg_val = aw88266a_read_reg(priv, AW88266A_SYSST_REG);
      if (((reg_val & (~AW88266A_SYSST_CHECK_MASK)) &
            AW88266A_SYSST_CHECK) == AW88266A_SYSST_CHECK)
        {
          ret = OK;
          break;
        }
      else
        {
          auderr("ERROR: check pll fail:cnt=%d reg_val=0x%04x\n\n",
                 i, reg_val);
          ret = ERROR;
          up_mdelay(2);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: aw88266a_run_amp
 *
 * Description:
 *   enable/disable aw88266a amp running status
 *
 ****************************************************************************/

static void aw88266a_run_amp(FAR struct aw88266a_dev_s *priv, bool status)
{
  if (status)
    {
      aw88266a_write_reg_bit(priv, AW88266A_SYSCTRL_REG,
                             AW88266A_AMPPD_MASK,
                             AW88266A_AMPPD_WORKING_VALUE);
    }
  else
    {
      aw88266a_write_reg_bit(priv, AW88266A_SYSCTRL_REG,
                             AW88266A_AMPPD_MASK,
                             AW88266A_AMPPD_POWER_DOWN_VALUE);
    }
}

/****************************************************************************
 * Name: aw88266a_set_gain
 *
 * Description:
 *   set aw88266a gain
 *
 ****************************************************************************/

static int aw88266a_set_gain(FAR struct aw88266a_dev_s *priv, int gain)
{
  uint16_t reg_value;

  if (gain > AW88266A_GAIN_MAX || gain < 0)
    {
      return ERROR;
    }

  gain = AW88266A_GAIN_MAX - gain;

  /* cal real gain */

  gain = (((gain / AW88266A_VOL_STEP_DB) << 6)
          + (gain % AW88266A_VOL_STEP_DB));

  /* get reg_val form real gain */

  gain = (gain & (~0xfc00));
  reg_value = aw88266a_read_reg(priv, AW88266A_SYSCTRL2_REG);
  gain = gain | (reg_value & 0xfc00);
  aw88266a_write_reg(priv, AW88266A_SYSCTRL2_REG, gain);

  return OK;
}

/****************************************************************************
 * Name: aw88266a_get_gain
 *
 * Description:
 *   get aw88266a gain
 *
 ****************************************************************************/

int aw88266a_get_gain(FAR struct aw88266a_dev_s *priv, FAR int *gain)
{
  uint16_t reg_val;

  reg_val = aw88266a_read_reg(priv, AW88266A_SYSCTRL2_REG);

  reg_val = (reg_val & (~0xfc00));

  *gain = AW88266A_GAIN_MAX -
          ((reg_val >> 6) * AW88266A_VOL_STEP_DB + (reg_val & 0x3f));

  return OK;
}

/****************************************************************************
 * Name: aw88266a_set_volume
 *
 * Description:
 *   set aw88266a volume
 *
 ****************************************************************************/

int aw88266a_set_volume(FAR struct aw88266a_dev_s *priv, int volume)
{
  int ret;
  uint16_t reg_val;

  if (volume > 100 || volume < 0)
    {
      auderr("param scope (0-100), keep last volume.\r\n");

      return ERROR;
    }

  reg_val = (volume / 100) * AW88266A_GAIN_MAX +
            (1 - (volume / 100)) * (volume / 10) * 80;

  ret = aw88266a_set_gain(priv, reg_val);

  return ret;
}

/****************************************************************************
 * Name: aw88266a_get_volume
 *
 * Description:
 *   get aw88266a volume
 *
 ****************************************************************************/

int aw88266a_get_volume(FAR struct aw88266a_dev_s *priv, int *volume)
{
  int ret;
  int gain;

  ret = aw88266a_get_gain(priv, &gain);

  *volume = (gain / AW88266A_GAIN_MAX) * 100 +
            (1 - (gain / AW88266A_GAIN_MAX)) * (gain / 80) * 10;

  return ret;
}

/****************************************************************************
 * Name: aw88266a_getcaps
 *
 * Description:
 *   audio driver of get aw88266a caps
 *
 ****************************************************************************/

static int aw88266a_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                            FAR struct audio_caps_s *caps)
{
  /* Validate the structure */

  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("type=%d ac_type=%d\n", type, caps->ac_type);

  /* Fill in the caller's structure based on requested info */

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      /* Caller is querying for the types of units we support */

      case AUDIO_TYPE_QUERY:

        /* Provide our overall capabilities.  The interfacing software
         * must then call us back for specific info for each capability.
         */

        caps->ac_channels = 2;       /* Stereo output */

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* We don't decode any formats!  Only something above us in
               * the audio stream can perform decoding on our behalf.
               */

              /* The types of audio units we implement */

              caps->ac_controls.b[0] =
                AUDIO_TYPE_OUTPUT | AUDIO_TYPE_FEATURE |
                AUDIO_TYPE_PROCESSING;

              caps->ac_format.hw = (1 << (AUDIO_FMT_PCM - 1));
              break;

            case AUDIO_FMT_MIDI:

              /* We only support Format 0 */

              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }

        break;

      /* Provide capabilities of our OUTPUT unit */

      case AUDIO_TYPE_OUTPUT:

        caps->ac_channels = 2;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report the Sample rates we support */

              caps->ac_controls.b[0] =
                AUDIO_SAMP_RATE_8K | AUDIO_SAMP_RATE_11K |
                AUDIO_SAMP_RATE_16K | AUDIO_SAMP_RATE_22K |
                AUDIO_SAMP_RATE_32K | AUDIO_SAMP_RATE_44K |
                AUDIO_SAMP_RATE_48K;
              break;

            case AUDIO_FMT_MP3:
            case AUDIO_FMT_WMA:
            case AUDIO_FMT_PCM:
              break;

            default:
              break;
          }

        break;

      /* Provide capabilities of our FEATURE units */

      case AUDIO_TYPE_FEATURE:

        /* If the sub-type is UNDEF, then report the Feature Units we
         * support.
         */

        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            /* Fill in the ac_controls section with the Feature Units we
             * have.
             */

            caps->ac_controls.b[0] = AUDIO_FU_VOLUME | AUDIO_FU_BASS |
                                     AUDIO_FU_TREBLE;
            caps->ac_controls.b[1] = AUDIO_FU_BALANCE >> 8;
          }
        else
          {
            /* TODO:  Do we need to provide specific info for the Feature
             * Units, such as volume setting ranges, etc.?
             */
          }

        break;

      /* Provide capabilities of our PROCESSING unit */

      case AUDIO_TYPE_PROCESSING:

        switch (caps->ac_subtype)
          {
            case AUDIO_PU_UNDEF:

              /* Provide the type of Processing Units we support */

              caps->ac_controls.b[0] = AUDIO_PU_STEREO_EXTENDER;
              break;

            case AUDIO_PU_STEREO_EXTENDER:

              /* Provide capabilities of our Stereo Extender */

              caps->ac_controls.b[0] =
                AUDIO_STEXT_ENABLE | AUDIO_STEXT_WIDTH;
              break;

            default:

              /* Other types of processing uint we don't support */

              break;
          }
        break;

      /* All others we don't support */

      default:

        /* Zero out the fields to indicate no support */

        caps->ac_subtype = 0;
        caps->ac_channels = 0;

        break;
    }

  /* Return the length of the audio_caps_s struct for validation of
   * proper Audio device type.
   */

  return caps->ac_len;
}

/****************************************************************************
 * Name: aw88266a_configure
 *
 * Description:
 *   audio driver configure aw88266a
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
aw88266a_configure(FAR struct audio_lowerhalf_s *dev,
                   FAR void *session, FAR const struct audio_caps_s *caps)
#else
static int aw88266a_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR const struct audio_caps_s *caps)
#endif
{
  audinfo("ac_type: %d\n", caps->ac_type);
  FAR struct aw88266a_dev_s *priv = (FAR struct aw88266a_dev_s *)dev;

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_FEATURE:
        {
          audinfo("  AUDIO_TYPE_FEATURE\n");

          /* Process based on Feature Unit */

          switch (caps->ac_format.hw)
            {
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
              case AUDIO_FU_VOLUME:
                {
                  audinfo("Volume: %d\n", caps->ac_controls.hw[0]);
                }
                break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
              case AUDIO_FU_BASS:
                {
                  audinfo("Bass: %d\n", caps->ac_controls.b[0]);
                }
                break;

              case AUDIO_FU_TREBLE:
                {
                  audinfo("Treble: %d\n", caps->ac_controls.b[0]);
                }
                break;
#endif /* CONFIG_AUDIO_EXCLUDE_TONE */

              default:
                {
                  auderr("ERROR: Unrecognized feature unit:0x%x\n",
                        caps->ac_format.hw);
                }
                break;
            }
        }
        break;

      case AUDIO_TYPE_OUTPUT:
        {
          audinfo("  AUDIO_TYPE_OUTPUT:\n");
          audinfo("    Number of channels: %u\n", caps->ac_channels);
          audinfo("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
          audinfo("    Sample width:       %u\n", caps->ac_controls.b[2]);
          audinfo("    Output channel map: 0x%x\n", caps->ac_chmap);

          /* Verify that all of the requested values are supported */

          priv->samprate  = caps->ac_controls.hw[0];
          priv->nchannels = caps->ac_channels;
          priv->bpsamp  = caps->ac_controls.b[2];
          priv->bclk    = caps->ac_controls.hw[0] *
                          (priv->lower->bclk_factor);
          aw88266a_set_device(priv, DEFAULT_OUTPUT_SPEAKER);
          aw88266a_set_channel(priv, priv->nchannels);
          aw88266a_set_rate(priv, priv->samprate);
          aw88266a_set_width(priv, priv->bpsamp);
          aw88266a_set_blck(priv, priv->lower->bclk_factor);
        }
        break;

      case AUDIO_TYPE_PROCESSING:
        {
          audinfo("AUDIO_TYPE_PROCESSING:\n");
        }
        break;
    }

  audinfo("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: aw88266a_shutdown
 *
 * Description:
 *   aduio driver of shutdown aw88266a
 *
 ****************************************************************************/

static int aw88266a_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct aw88266a_dev_s *priv = (FAR struct aw88266a_dev_s *)dev;

  aw88266a_run_mute(priv, true);
  aw88266a_run_amp(priv, false);
  aw88266a_run_pwd(priv, true);

  audinfo("shutdown OK\n");
  return OK;
}

/****************************************************************************
 * Name: aw88266a_start
 *
 * Description:
 *   audio driver of start aw88266a
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
aw88266a_start(FAR struct audio_lowerhalf_s *dev,
               FAR void *session)
#else
static int aw88266a_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct aw88266a_dev_s *priv = (FAR struct aw88266a_dev_s *)dev;

  aw88266a_run_pwd(priv, false);

  if (aw88266a_pll_check(priv) < 0)
    {
      auderr("ERROR: aw88266a pll check failed\n");
    }

  aw88266a_run_amp(priv, true);
  aw88266a_run_mute(priv, false);
  aw88266a_set_volume(priv, 100);

  audinfo("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: aw88266a_stop
 *
 * Description:
 *   audio driver of stop aw88266a
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int
  aw88266a_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#  else
static int aw88266a_stop(FAR struct audio_lowerhalf_s *dev)
#  endif
{
  return OK;
}
#endif

/****************************************************************************
 * Name: aw88266a_pause
 *
 * Description:
 *   audio driver of pause aw88266a
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int aw88266a_pause(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#  else
static int aw88266a_pause(FAR struct audio_lowerhalf_s *dev)
#  endif
{
  return OK;
}
#endif

/****************************************************************************
 * Name: aw88266a_resume
 *
 * Description:
 *   audio driver of resume aw88266a
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int aw88266a_resume(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session)
#  else
static int aw88266a_resume(FAR struct audio_lowerhalf_s *dev)
#  endif
{
  return OK;
}
#endif

/****************************************************************************
 * Name: aw88266a_ioctl
 *
 * Description:
 *   audio driver of ioctl aw88266a
 *
 ****************************************************************************/

static int aw88266a_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                          unsigned long arg)
{
  FAR const char *ptr = (FAR const char *)arg;
  uint8_t i;
  FAR struct aw88266a_dev_s *priv = (FAR struct aw88266a_dev_s *)dev;

  switch (cmd)
    {
      case AUDIOIOC_SETPARAMTER:
        if ((strncmp(ptr, "mode=", 5)) == 0)
          {
            ptr = ptr + 5;

            for (i = 0; i < ARRAY_SIZE(g_aw88266a_device_mode); i++)
              {
                if (strncmp(ptr, g_aw88266a_device_mode[i].dev_str,
                            strlen(g_aw88266a_device_mode[i].dev_str)) == 0)
                  {
                    aw88266a_set_device(priv, g_aw88266a_device_mode[i].dev);
                  }
              }
          }
        break;

      case AUDIOIOC_HWRESET:
        audinfo("AUDIOIOC_HWRESET\n");
        break;

      default:
        break;
    }

  audinfo("Return OK\n");
  return OK;
}

/****************************************************************************
 * Name: aw88266a_reserve
 *
 * Description:
 *   audio driver of reserve
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
aw88266a_reserve(FAR struct audio_lowerhalf_s *dev, FAR void **session)
#else
static int aw88266a_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  audinfo("Reserve OK\n");

  return OK;
}

/****************************************************************************
 * Name: aw88266a_release
 *
 * Description:
 *   audio driver of release aw88266a
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int aw88266a_release(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session)
#else
static int aw88266a_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}

/****************************************************************************
 * Name: aw88266a_write_reg
 *
 * Description:
 *   aw88266a write register
 *
 ****************************************************************************/

static void aw88266a_write_reg(FAR struct aw88266a_dev_s *priv,
                               uint8_t regaddr, uint16_t regval)
{
  struct i2c_config_s config;
  int retries;
  int ret;

  /* Setup up the I2C configuration */

  config.frequency = priv->lower->frequency;
  config.address   = priv->lower->address;
  config.addrlen   = 7;

  for (retries = 0; retries < MAX_RETRIES; retries++)
    {
      uint8_t data[4];

      /* Set up the data to write */

      data[0] = regaddr & 0xff;
      data[1] = regval >> 8;
      data[2] = regval & 0xff;

      /* Read the register data.  The returned value is the number messages
       * completed.
       */

      ret = i2c_write(priv->i2c, &config, data, 3);

      if (ret < 0)
        {
          auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
        }
      else
        {
          break;
        }

      audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }
}

/****************************************************************************
 * Name: aw88266a_read_reg
 *
 * Description:
 *   aw88266a read register
 *
 ****************************************************************************/

static int16_t aw88266a_read_reg(FAR struct aw88266a_dev_s *priv,
                                 uint8_t regaddr)
{
  int retries;
  struct i2c_config_s config;
  uint8_t buffer    = regaddr;
  uint8_t data[2];
  int ret;

  config.address   = priv->lower->address;
  config.frequency = priv->lower->frequency;
  config.addrlen   = 7;

  for (retries = 0; retries < MAX_RETRIES; retries++)
    {
      ret = i2c_writeread(priv->i2c, &config, &buffer, 1, data, 2);

      if (ret < 0)
        {
          auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
        }
      else
        {
          int16_t regval;

          /* The I2C transfer was successful... break out of the loop and
           * return the value read.
           */

          regval = ((int16_t)data[0] << 8) | (int16_t)data[1];
          return regval;
        }

        audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }

  return ERROR;
}

/****************************************************************************
 * Name: aw88266a_write_reg_bit
 *
 * Description:
 *   aw88266a write bit of register
 *
 ****************************************************************************/

static void aw88266a_write_reg_bit(FAR struct aw88266a_dev_s *priv,
                                   uint8_t regaddr, uint16_t mask,
                                   uint16_t regval)
{
  uint16_t t_val = 0;

  t_val = aw88266a_read_reg(priv, regaddr);
  t_val &= mask;
  t_val |= regval;
  aw88266a_write_reg(priv, regaddr, t_val);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aw88266a_initialize
 *
 * Description:
 *   Initializa awinic88266a smart pa
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
aw88266a_initialize(FAR struct i2c_master_s *i2c,
                    FAR struct aw88266a_lower_s * lower)
{
  FAR struct aw88266a_dev_s *priv;
  uint16_t regval;
  int16_t ret;

  if (lower == NULL)
    {
      auderr("ERROR: lower is NULL\n");
      return NULL;
    }

  ret = lower->power_en(true);
  if (ret < 0)
    {
      auderr("ERROR: AW88266A power_en\n");
      return NULL;
    }

  ret = lower->reset_en(true);
  if (ret < 0)
    {
      auderr("ERROR: AW88266A reset_en\n");
      return NULL;
    }

  /* Allocate a WM8994 device structure */

  priv = kmm_zalloc(sizeof(struct aw88266a_dev_s));

  if (priv)
    {
      priv->dev.ops = &g_audioops;
      priv->lower   = lower;
      priv->i2c     = i2c;

      regval = aw88266a_read_reg(priv, AW88266A_ID_REG);
      if (regval != AW88266A_CHIPID)
        {
          auderr("ERROR: AW88266A not found: ID=%04x\n", regval);
          goto errout_with_dev;
        }

      return &priv->dev;
    }

errout_with_dev:
  kmm_free(priv);
  return NULL;
}
