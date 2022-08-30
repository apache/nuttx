/****************************************************************************
 * drivers/audio/fs1818.c
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
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>

#include "fs1818.h"
#include <nuttx/audio/fs1818u.h>
#include "fs1818_reg.h"
#include "fs1818_fw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAY_SIZE(x)     (sizeof(x) / sizeof(x[0]))

#define NORMAL_MODE       (0)
#define DSP_MODE          (1)
#define CAL_MODE          (2)

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

typedef int (*fs1818_ioctl_handler_t)(FAR struct fs1818u_dev_s *priv,
                                      FAR char *key, FAR char *value,
                                      FAR void *arg);

typedef struct
{
  char *key;
  fs1818_ioctl_handler_t handler;
} fs1818_info_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int fs1818_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                            FAR struct audio_caps_s *caps);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
fs1818_configure(FAR struct audio_lowerhalf_s *dev,
                   FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int fs1818_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR const struct audio_caps_s *caps);
#endif

static int fs1818_shutdown(FAR struct audio_lowerhalf_s *dev);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
fs1818_start(FAR struct audio_lowerhalf_s *dev,
               FAR void *session);
#else
static int fs1818_start(FAR struct audio_lowerhalf_s *dev);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int
  fs1818_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#  else
static int fs1818_stop(FAR struct audio_lowerhalf_s *dev);
#  endif
#endif

static int fs1818_ioctl(FAR struct audio_lowerhalf_s *dev,
                        int cmd, unsigned long arg);

static int fs1818_start_hw(FAR struct fs1818u_dev_s *priv, uint8_t mode);
static int fs1818_read_otp(FAR struct fs1818u_dev_s *priv);
static int fs1818_start_up(FAR struct fs1818u_dev_s *priv);
#ifdef CONFIG_AUDIO_FS1818U_DEBUG
static int fs1818_dump_reg(FAR struct fs1818u_dev_s *priv);
#endif
static int fs1818_config_i2s_and_pll(FAR struct fs1818u_dev_s *priv,
                                     audio_out_format_t * pconfig);
static int fs1818_set_volume(FAR struct fs1818u_dev_s *priv, uint8_t volume);
static int fs1818_shut_down(FAR struct fs1818u_dev_s *priv);
static int fs1818_dsp_bypass(FAR struct fs1818u_dev_s *priv, int mode);
static int fs1818_usleep(int usec);
static int fs1818_force_calibrate(FAR struct fs1818u_dev_s *priv,
                                  uint8_t force, bool store);
static int fs1818_calibrate_config(FAR struct fs1818u_dev_s *priv);
static int fs1818_reg_init(FAR struct fs1818u_dev_s *priv);
static int fs1818_write_fw(FAR struct fs1818u_dev_s *priv, uint8_t mode);
static int fs1818_update_caliberate_count(FAR struct fs1818u_dev_s *priv);

static int fs1818_set_scenario_handler(FAR struct fs1818u_dev_s *priv,
                                       FAR char *key, FAR char *value,
                                       FAR void *arg);
#ifdef CONFIG_AUDIO_FS1818U_DEBUG
static int fs1818_set_print_info_handler(FAR struct fs1818u_dev_s *priv,
                                         FAR char *key, FAR char *value,
                                         FAR void *arg);
#endif
static int fs1818_set_mode_handler(FAR struct fs1818u_dev_s *priv,
                                  FAR char *key, FAR char *value,
                                  FAR void *arg);
static int fs1818_get_cali_value_handler(FAR struct fs1818u_dev_s *priv,
                                        FAR char *key, FAR char *value,
                                        FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const static fsm_pll_config_t g_fs1818_pll_tbl[] =
{
  /* bclk,    0xC1,   0xC2,   0xC3 */

  { 256000,  0x01a0, 0x0180, 0x0001 }, /* 8000*16*2 */
  { 512000,  0x01a0, 0x0180, 0x0002 }, /* 16000*16*2 & 8000*32*2 */
  { 1024000, 0x0260, 0x0120, 0x0003 }, /* 16000*32*2 */
  { 1024032, 0x0260, 0x0120, 0x0003 }, /* 32000*16*2+32 */
  { 1411200, 0x01a0, 0x0100, 0x0004 }, /* 44100*16*2 */
  { 1536000, 0x0260, 0x0100, 0x0004 }, /* 48000*16*2 */
  { 2048032, 0x0260, 0x0120, 0x0006 }, /* 32000*32*2+32 */
  { 2822400, 0x01a0, 0x0100, 0x0008 }, /* 44100*32*2 */
  { 3072000, 0x0260, 0x0100, 0x0008 }, /* 48000*32*2 */
};

static const struct audio_ops_s g_audioops =
{
  .getcaps        = fs1818_getcaps,
  .configure      = fs1818_configure,
  .shutdown       = fs1818_shutdown,
  .start          = fs1818_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop           = fs1818_stop,
#endif
  .ioctl          = fs1818_ioctl,
};

static const fs1818_info_t fs1818_info[] =
{
  {
    .key        = "set_scenario",
    .handler    = fs1818_set_scenario_handler,
  },

  {
    .key        = "set_mode",
    .handler    = fs1818_set_mode_handler,
  },

  {
    .key        = "get_caliberate_value",
    .handler    = fs1818_get_cali_value_handler,
  },

#ifdef CONFIG_AUDIO_FS1818U_DEBUG
  {
    .key        = "set_printinfo",
    .handler    = fs1818_set_print_info_handler,
  },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs35l41b_set_scenario_handler
 *
 * Description:
 *   set scenario handler
 *
 ****************************************************************************/

static int fs1818_set_scenario_handler(FAR struct fs1818u_dev_s *priv,
                                       FAR char *key, FAR char *value,
                                       FAR void *arg)
{
  if (!strncmp(value, "music", strlen("music")))
    {
      priv->scenario_mode = PRESET_MUSIC;
    }
  else if (!strncmp(value, "sco", strlen("sco")))
    {
      priv->scenario_mode = PRESET_VOICE;
    }
  else
    {
      auderr("%s value is invaild!\n", key);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_parse_string
 *
 * Description:
 *   parse string
 *
 ****************************************************************************/

static void fs1818_parse_string(FAR char *src, FAR char *key,
                                FAR char **value)
{
  FAR char *parse_ptr = src;

  if (!strncmp(parse_ptr, key, strlen(key)))
    {
      parse_ptr += strlen(key);
      if (!strncmp(parse_ptr, "=", strlen("=")))
        {
          parse_ptr += strlen("=");

          *value = parse_ptr;
        }
      else
        {
          *value = NULL;
        }
    }
}

/****************************************************************************
 * Name: fs1818_set_print_info_handler
 *
 * Description:
 *   enable/disable print information
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_FS1818U_DEBUG
static int fs1818_set_print_info_handler(FAR struct fs1818u_dev_s *priv,
                                         FAR char *key, FAR char *value,
                                         FAR void *arg)
{
  if (!strncmp(value, "on", strlen("on")))
    {
      priv->dump_info = true;
    }
  else if (!strncmp(value, "off", strlen("off")))
    {
      priv->dump_info = false;
    }
  else
    {
      auderr("%s value is invaild!\n", key);
      return ERROR;
    }

  audwarn("priv->dump_info:%d\n", priv->dump_info);

  return OK;
}
#endif

/****************************************************************************
 * Name: fs1818_set_mode_handler
 *
 * Description:
 *   set mode handler
 *
 ****************************************************************************/

static int fs1818_set_mode_handler(FAR struct fs1818u_dev_s *priv,
                                  FAR char *key, FAR char *value,
                                  FAR void *arg)
{
  if (!strncmp(value, "normal", strlen("normal")))
    {
      priv->mode = NORMAL_MODE;
    }
  else if (!strncmp(value, "dsp", strlen("dsp")))
    {
      priv->mode = DSP_MODE;
    }
  else if (!strncmp(value, "cal", strlen("cal")))
    {
      priv->mode = CAL_MODE;
      priv->store_cali_value = true;
    }
  else
    {
      auderr("%s value is invaild!\n", key);
      return ERROR;
    }

  if (fs1818_reg_init(priv) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_fw(priv, priv->scenario_mode) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_shut_down(priv) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_get_cali_value_handler
 *
 * Description:
 *   get caliberate value handler
 *
 ****************************************************************************/

static int fs1818_get_cali_value_handler(FAR struct fs1818u_dev_s *priv,
                                        FAR char *key, FAR char *value,
                                        FAR void *arg)
{
  FAR char *parse_ptr = (char *)arg;

  memset(parse_ptr, 0, strlen(parse_ptr) + 1);
  if (priv->caliberate_result == -1)
    {
      sprintf(parse_ptr, "caliberate:ERROR");
    }
  else
    {
      sprintf(parse_ptr, "caliberate:%d:%d", priv->caliberate_result,
              priv->caliberate_count);
    }

  auderr("%s\n", parse_ptr);

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_ioctl
 *
 * Description:
 *   fs1818 ioctl
 *
 ****************************************************************************/

static int fs1818_ioctl(FAR struct audio_lowerhalf_s *dev,
                        int cmd, unsigned long arg)
{
  FAR struct fs1818u_dev_s *priv = (FAR struct fs1818u_dev_s *)dev;
  FAR char *parse_ptr = (FAR char *)arg;
  FAR char *value = NULL;
  int ret;
  int i;

  if (cmd == AUDIOIOC_SETPARAMTER)
    {
      for (i = 0; i < ARRAY_SIZE(fs1818_info); i++)
        {
          if (!strncmp(parse_ptr,
              fs1818_info[i].key, strlen(fs1818_info[i].key)))
            {
              fs1818_parse_string(parse_ptr, fs1818_info[i].key, &value);

              if (fs1818_info[i].handler != NULL)
                {
                  ret = fs1818_info[i].handler(priv,
                                               fs1818_info[i].key,
                                               value,
                                               (void *)arg);
                  if (ret < 0)
                    {
                      auderr("fs1818 ioctl handler failed!\n");
                      return ERROR;
                    }

                  return OK;
                }
              else
                {
                  return ERROR;
                }
            }
        }

      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41fs1818_getcapsb_start
 *
 * Description:
 *   fs1818 get caps
 *
 ****************************************************************************/

static int fs1818_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
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

              caps->ac_controls.b[0] = AUDIO_SAMP_RATE_44K;
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
 * Name: fs1818_configure
 *
 * Description:
 *   audio driver configure fs1818
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
fs1818_configure(FAR struct audio_lowerhalf_s *dev,
                   FAR void *session, FAR const struct audio_caps_s *caps)
#else
static int fs1818_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct fs1818u_dev_s *priv = (FAR struct fs1818u_dev_s *)dev;

  audinfo("ac_type: %d\n", caps->ac_type);

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
                  audinfo("Unrecognized feature unit:0x%x\n",
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
          priv->bpsamp    = caps->ac_controls.b[2];
          priv->bclk      = caps->ac_controls.hw[0] *
                            (priv->lower->bclk_factor);
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
 * Name: cs35l41b_shutdown
 *
 * Description:
 *   aduio driver of shutdown cs35l41b
 *
 ****************************************************************************/

static int fs1818_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct fs1818u_dev_s *priv = (FAR struct fs1818u_dev_s *)dev;

  audinfo("cs35l41b shutdown\n");

  if (fs1818_shut_down(priv) == ERROR)
    {
      auderr("fs1818u shutdown failed!\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_start
 *
 * Description:
 *   audio driver of start cs35l41b
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
fs1818_start(FAR struct audio_lowerhalf_s *dev,
               FAR void *session)
#else
static int fs1818_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct fs1818u_dev_s *priv = (FAR struct fs1818u_dev_s *)dev;

  audinfo("fs1818 start ....\n");

  if (priv->mode == CAL_MODE)
    {
      if (fs1818_start_hw(priv, priv->scenario_mode) == ERROR)
        {
          auderr("fs1818 start failed!!!\n");
          return ERROR;
        }

      fs1818_usleep(1000 * 10);

      if (fs1818_calibrate_config(priv) == ERROR)
        {
          return ERROR;
        }
    }
  else
    {
      if (fs1818_dsp_bypass(priv, priv->mode) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_start_hw(priv, priv->scenario_mode) == ERROR)
        {
          auderr("fs1818 start failed!!!\n");
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cs35l41b_stop
 *
 * Description:
 *   audio driver of stop cs35l41b
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int
  fs1818_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#  else
static int fs1818_stop(FAR struct audio_lowerhalf_s *dev)
#  endif
#endif
{
  FAR struct fs1818u_dev_s *priv = (FAR struct fs1818u_dev_s *)dev;
  int ret;

  auderr("fs1818u stop!\n");

  if (priv->mode == CAL_MODE)
    {
      ret = fs1818_force_calibrate(priv, 1, priv->store_cali_value);
      auderr("caliberate results:%d\n", ret);
      priv->caliberate_result = ret;
      fs1818_update_caliberate_count(priv);
    }

  if (fs1818_shut_down(priv) == ERROR)
    {
      auderr("fs1818u shutdown failed!\n");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_usleep
 *
 * Description:
 *   fs1818 usleep
 *
 ****************************************************************************/

static int fs1818_usleep(int usec)
{
  struct timespec rqtp;
  struct timespec rmtp;
  int ret;

  rqtp.tv_sec  = 0;
  rqtp.tv_nsec = usec * 1000;

  while (1)
    {
      ret = nxsig_nanosleep(&rqtp, &rmtp);
      if (ret < 0)
        {
          rqtp.tv_nsec = rmtp.tv_nsec;
        }
      else
        {
          break;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: fs1818_read_reg_data
 *
 * Description:
 *   fs1818 read register data
 *
 ****************************************************************************/

static int fs1818_read_reg_data(FAR struct fs1818u_dev_s *priv,
                                uint8_t regaddr,
                                uint16_t *value)
{
  int retries;
  struct i2c_config_s config;
  uint8_t buffer    = regaddr;
  uint8_t data[2];
  int ret;

  config.address   = priv->lower->address;
  config.frequency = priv->lower->frequency;
  config.addrlen   = 7;

  for (retries = 0; retries < FS1818_I2C_RETRY; retries++)
    {
      ret = i2c_writeread(priv->i2c, &config, &buffer, 1, data, 2);

      if (ret < 0)
        {
          auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
        }
      else
        {
          /* The I2C transfer was successful... break out of the loop and
           * return the value read.
           */

          *value = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
          return OK;
        }

        audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }

  return ERROR;
}

/****************************************************************************
 * Name: fs1818_write_reg_data
 *
 * Description:
 *   fs1818 write register data
 *
 ****************************************************************************/

static int fs1818_write_reg_data(FAR struct fs1818u_dev_s *priv,
                                 uint8_t regaddr,
                                 uint16_t regval)
{
  struct i2c_config_s config;
  int retries;
  int ret;

  /* Setup up the I2C configuration */

  config.frequency = priv->lower->frequency;
  config.address   = priv->lower->address;
  config.addrlen   = 7;

  for (retries = 0; retries < FS1818_I2C_RETRY; retries++)
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
          return OK;
        }

      audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }

  return ERROR;
}

/****************************************************************************
 * Name: fs1818_burst_write
 *
 * Description:
 *   fs1818 burst write
 *
 ****************************************************************************/

static int fs1818_burst_write(FAR struct fs1818u_dev_s *priv, uint32_t value)
{
  struct i2c_config_s config;
  int retries;
  int ret;
  uint16_t temp;

  /* Setup up the I2C configuration */

  config.frequency = priv->lower->frequency;
  config.address   = priv->lower->address;
  config.addrlen   = 7;

  for (retries = 0; retries < FS1818_I2C_RETRY; retries++)
    {
      uint8_t data[5];

      /* Set up the data to write */

      data[0] = FS1818_DACEQWL_REG;

      temp = value & 0xffff;
      data[1] = (temp >> 8) & 0xff;
      data[2] = temp & 0xff;

      temp = (value >> 16) & 0xffff;
      data[3] = (temp >> 8) & 0xff;
      data[4] = temp & 0xff;

      /* Read the register data.  The returned value is the number messages
       * completed.
       */

      ret = i2c_write(priv->i2c, &config, data, 5);

      if (ret < 0)
        {
          auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
        }
      else
        {
          return OK;
        }

      audinfo("retries=%d\n", retries);
    }

  return ERROR;
}

/****************************************************************************
 * Name: fs1818_check_id
 *
 * Description:
 *   fs1818 check chip id
 *
 ****************************************************************************/

static int fs1818_check_id(FAR struct fs1818u_dev_s *priv)
{
  int ret = 0;
  uint16_t id;
  uint8_t mask = 0xff;
  uint8_t shift = 8;

  ret = fs1818_read_reg_data(priv, FS1818_ID_REG, &id);
  if (ret < 0)
    {
      auderr("fs1818 read register failed\n");
      return ERROR;
    }

  if (((id >> shift) & mask) == FS1818_DEV_ID && ((id & mask) != 0))
    {
      audwarn("fs1818 device detected, id = 0x%04x\n", id);
    }
  else
    {
      auderr("fs1818 invalid id, id:0x%04x\n", id);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_reg_init
 *
 * Description:
 *   fs1818 registers initialize
 *
 ****************************************************************************/

static int fs1818_reg_init(FAR struct fs1818u_dev_s *priv)
{
  uint8_t i = 0;
  uint8_t retry_max = 10;
  uint16_t val;
  uint16_t ini_hl   = 0x0003;
  uint16_t ini_lh   = 0x0300;
  uint16_t reset    = 0x0002;
  uint16_t power_down = 0x0001;

  audinfo("fs1818_reg_init enter\n");

  while (i++ < retry_max)
    {
      if (fs1818_write_reg_data(priv, FS1818_SYSCTRL_REG, reset) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_read_reg_data(priv, FS1818_STATUS_REG, &val) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv,
                                FS1818_SYSCTRL_REG,
                                power_down) == ERROR)
        {
          return ERROR;
        }

      fs1818_usleep(1000 * 15);

      if (fs1818_read_reg_data(priv, FS1818_CHIPINI_REG, &val) == ERROR)
        {
          return ERROR;
        }

      if ((val == ini_hl) || (val == ini_lh))
        {
          break;
        }
    }

  audwarn("fs1818_reg_init reset count:%d\n", i);

  if (fs1818_write_reg_data(priv,
                            FS1818_PLLCTRL4_REG,
                            FS1818_PLLCTRL4_ENABLE) == ERROR)
    {
      return ERROR;
    }

  fs1818_usleep(1000 * 15);

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPACC_REG,
                            FS1818_OTPACC_ENABLE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_DSPCTRL_REG,
                            FS1818_DSPCTRL_DEFAULT) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_ADCENV_REG,
                            FS1818_ADCENV_REG_AMP_DTEN_ALL) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_ADCTIME_REG,
                            FS1818_ADCTIME_REG_VALUE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_BSTCTRL_REG,
                            FS1818_BSTCTRL_DEFAULT) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_SYSCTRL_REG,
                            FS1818_SYSCTRL_REG_PWON) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_SPKMDB_REG,
                            FS1818_SPKMDB_SET) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_DACCTRL_REG,
                            FS1818_DACCTRL_UNMUTE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_CLDCTRL_REG,
                            FS1818_CLDCTRL_DEFAULT) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPACC_REG,
                            FS1818_OTPACC_DISABLE) == ERROR)
    {
      return ERROR;
    }

  audinfo("fs1818_reg_init exit\n");

  return OK;
}

/****************************************************************************
 * Name: fs1818_write_fw
 *
 * Description:
 *   fs1818 write firmware
 *
 ****************************************************************************/

static int fs1818_write_fw(FAR struct fs1818u_dev_s *priv, uint8_t mode)
{
  uint16_t value09;
  uint16_t valuec4;
  int i;

  audinfo("fs1818_write_fw enter\n");

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPACC_REG,
                            FS1818_OTPACC_ENABLE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_read_reg_data(priv,
                           FS1818_PLLCTRL4_REG,
                           &valuec4) == ERROR)
    {
      return ERROR;
    }

  if (valuec4 != FS1818_PLLCTRL4_ENABLE)
    {
      if (fs1818_write_reg_data(priv,
                                FS1818_PLLCTRL4_REG,
                                FS1818_PLLCTRL4_ENABLE) == ERROR)
        {
          return ERROR;
        }
    }

  if (fs1818_read_reg_data(priv,
                           FS1818_SYSCTRL_REG,
                           &value09) == ERROR)
    {
      return ERROR;
    }

  if (value09 != FS1818_SYSCTRL_REG_PWON)
    {
      if (fs1818_write_reg_data(priv,
                                FS1818_SYSCTRL_REG,
                                FS1818_SYSCTRL_REG_PWON) == ERROR)
        {
          return ERROR;
        }

      fs1818_usleep(1000 * 20);
    }

  if (mode == PRESET_MUSIC)
    {
      if (fs1818_write_reg_data(priv,
                                FS1818_DACEQA_REG,
                                FS1818_DACEQA_REG_ACS0) == ERROR)
        {
          return ERROR;
        }

      audwarn("fs1818 write music len = %d\n", DATA_SIZE(fs1818_fw_music));

      for (i = 0; i < DATA_SIZE(fs1818_fw_music); i++)
        {
          if (fs1818_burst_write(priv, fs1818_fw_music[i]) == ERROR)
            {
              return ERROR;
            }
        }

      audwarn("fs1818 write music reg len = %d\n",
              DATA_SIZE(fs1818_reg_music));

      for (i = 0; i < DATA_SIZE(fs1818_reg_music); i++)
        {
          if (fs1818_write_reg_data(priv,
                                    fs1818_reg_music[i].reg,
                                    fs1818_reg_music[i].value) == ERROR)
            {
              return ERROR;
            }
        }

      if (fs1818_write_reg_data(priv,
                                FS1818_CLDCTRL_REG,
                                FS1818_CLDCTRL_DEFAULT) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv,
                                FS1818_ACSCTRL_REG,
                                FS1818_ACSCTRL_MUSIC) == ERROR)
        {
          return ERROR;
        }
    }
  else if (mode == PRESET_VOICE)
    {
      if (fs1818_write_reg_data(priv,
                                FS1818_DACEQA_REG,
                                FS1818_DACEQA_REG_ACS1) == ERROR)
        {
          return ERROR;
        }

      audwarn("fs1818 write voice len = %d\n", DATA_SIZE(fs1818_fw_voice));

      for (i = 0; i < DATA_SIZE(fs1818_fw_voice); i++)
        {
          if (fs1818_burst_write(priv, fs1818_fw_voice[i]) == ERROR)
            {
              return ERROR;
            }
        }

      audwarn("fs1818 write voice reg len = %d\n",
              DATA_SIZE(fs1818_reg_voice));

      for (i = 0; i < DATA_SIZE(fs1818_reg_voice); i++)
        {
          if (fs1818_write_reg_data(priv,
                                    fs1818_reg_voice[i].reg,
                                    fs1818_reg_voice[i].value) == ERROR)
            {
              return ERROR;
            }
        }

      if (fs1818_write_reg_data(priv,
                                FS1818_CLDCTRL_REG,
                                FS1818_CLDCTRL_DISABLE) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv,
                                FS1818_ACSCTRL_REG,
                                FS1818_ACSCTRL_VOICE) == ERROR)
        {
          return ERROR;
        }
    }
  else
    {
      auderr("invalid preset mode = %d\n", mode);
    }

  audwarn("fs1818 write spk model len = %d\n",
          DATA_SIZE(fs1818_fw_spk_model));

  if (fs1818_write_reg_data(priv,
                            FS1818_DACEQA_REG,
                            FS1818_DACEQA_REG_SPKMDL) == ERROR)
    {
      return ERROR;
    }

  for (i = 0; i < DATA_SIZE(fs1818_fw_spk_model); i++)
    {
      if (fs1818_burst_write(priv, fs1818_fw_spk_model[i]) == ERROR)
        {
          return ERROR;
        }
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPACC_REG,
                            FS1818_OTPACC_DISABLE) == ERROR)
    {
      return ERROR;
    }

  audinfo("fs1818_write_fw exit\n");

  return OK;
}

/****************************************************************************
 * Name: fs1818_wait_stable
 *
 * Description:
 *   fs1818 wait stable
 *
 ****************************************************************************/

static int fs1818_wait_stable(FAR struct fs1818u_dev_s *priv, uint16_t type)
{
  uint16_t status;
  uint16_t digstat;
  uint16_t tsctrl;
  uint16_t bstctrl;
  uint16_t otpcmd;
  uint16_t ret = 0;
  uint16_t ts_adcretry = 30;
  uint16_t ready = 0;
  int retries = 100;

  audinfo("fs1818_wait_stable enter\n");

  fs1818_usleep(1000 * 5);

  if (type == FSM_WAIT_TSIGNAL_OFF || type == FSM_WAIT_AMP_ADC_PLL_OFF)
    {
      retries = ts_adcretry;
    }

  while (retries-- > 0)
    {
      switch (type)
        {
          case FSM_WAIT_AMP_ON:
            if (fs1818_read_reg_data(priv,
                                    FS1818_STATUS_REG,
                                    &status) == ERROR)
              {
                return ERROR;
              }

            if (fs1818_read_reg_data(priv,
                                     FS1818_BSTCTRL_REG,
                                     &bstctrl) == ERROR)
              {
                return ERROR;
              }

            if ((status & FS1818_STATUS_REG_PLLS) &&
               (bstctrl & FS1818_BSTCTRL_REG_SSEND))
              {
                ready = 1;
              }
            break;

          case FSM_WAIT_AMP_OFF:
            if (fs1818_read_reg_data(priv,
                                    FS1818_DIGSTAT_REG,
                                    &digstat) == ERROR)
              {
                return ERROR;
              }

            ready = !(digstat & FS1818_DIGSTAT_REG_DACRUN);
            break;

          case FSM_WAIT_AMP_ADC_OFF:
          case FSM_WAIT_AMP_ADC_PLL_OFF:
            if (fs1818_read_reg_data(priv,
                                    FS1818_DIGSTAT_REG,
                                    &digstat) == ERROR)
              {
                return ERROR;
              }

            ready = !(digstat & FS1818_DIGSTAT_REG_ADCRUN);
            break;

          case FSM_WAIT_TSIGNAL_OFF:
            if (fs1818_read_reg_data(priv,
                                    FS1818_TSCTRL_REG,
                                    &tsctrl) == ERROR)
              {
                return ERROR;
              }

            ready = (tsctrl & FS1818_TSCTRL_REG_OFFSTA);
            break;

          case FSM_WAIT_OTP_READY:
            if (fs1818_read_reg_data(priv,
                                    FS1818_OTPCMD_REG,
                                    &otpcmd) == ERROR)
              {
                return ERROR;
              }

            ready = !(otpcmd & FS1818_OTPCMD_REG_BUSY);
            break;

          default:
            break;
        }

      if (!ret && ready)
        {
            break;
        }

      fs1818_usleep(1000 * 1);
      audwarn("fs1818_wait_stable wait time: %d\n", retries);
    }

  if (retries <= 0)
    {
      auderr("fs1818_wait_stable wait timeout!\n");

      return ERROR;
    }

  audinfo("fs1818_wait_stable exit\n");

  return OK;
}

/****************************************************************************
 * Name: fs1818_set_mute
 *
 * Description:
 *   fs1818 set mute
 *
 ****************************************************************************/

static int fs1818_set_mute(FAR struct fs1818u_dev_s *priv, uint16_t mute)
{
  uint16_t sysctrl;
  uint16_t tsctrl;
  uint16_t status;

  audwarn("fs1818_set_mute enter %s device\n",
         ((mute == 1) ? "mute" : "unmute"));

  if (fs1818_read_reg_data(priv, FS1818_SYSCTRL_REG, &sysctrl) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_read_reg_data(priv, FS1818_TSCTRL_REG, &tsctrl) == ERROR)
    {
      return ERROR;
    }

  if (mute == FSM_UNMUTE)
    {
      /* set volume */

      if (priv->mode == NORMAL_MODE)
        {
          if (fs1818_write_reg_data(priv,
              FS1818_AUDIOCTRL_REG,
              (priv->nor_volume << 8) & 0xff00) == ERROR)
            {
              return ERROR;
            }
        }
      else
        {
          if (fs1818_write_reg_data(priv,
              FS1818_AUDIOCTRL_REG,
              (priv->dsp_volume << 8) & 0xff00) == ERROR)
            {
              return ERROR;
            }
        }

      /* amp on */

      sysctrl |= FS1818_SYSCTRL_REG_AMPE;
      if (fs1818_write_reg_data(priv, FS1818_SYSCTRL_REG, sysctrl) == ERROR)
        {
          return ERROR;
        }

      /* enable ts */

      tsctrl |= FS1818_TSCTRL_REG_EN | FS1818_TSCTRL_REG_OFF_AUTOEN;
      if (fs1818_write_reg_data(priv, FS1818_TSCTRL_REG, tsctrl) == ERROR)
        {
          return ERROR;
        }

      /* get status */

      if (fs1818_read_reg_data(priv, FS1818_STATUS_REG, &status) == ERROR)
        {
          return ERROR;
        }

      audwarn("fs1818_set_mute status: 0x%04x\n", status);
    }
  else
    {
      /* disable ts */

      tsctrl &= ~(FS1818_TSCTRL_REG_EN);
      tsctrl |=  FS1818_TSCTRL_REG_OFF_AUTOEN;
      if (fs1818_write_reg_data(priv, FS1818_TSCTRL_REG, tsctrl) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_wait_stable(priv, FSM_WAIT_TSIGNAL_OFF) == ERROR)
        {
          return ERROR;
        }
    }

  audinfo("fs1818_set_mute exit\n");

  return OK;
}

/****************************************************************************
 * Name: fs1818_power_on
 *
 * Description:
 *   fs1818 power on
 *
 ****************************************************************************/

static int fs1818_power_on(FAR struct fs1818u_dev_s *priv, uint16_t pwr_on)
{
  uint16_t val;

  audwarn("fs1818_power_on enter, power %s device\n",
         ((pwr_on == 1) ? "on" : "off"));

  if (fs1818_read_reg_data(priv, FS1818_SYSCTRL_REG, &val) == ERROR)
    {
      return ERROR;
    }

  if (pwr_on)
    {
      val &= (~FS1818_SYSCTRL_REG_PWDN);
    }
  else
    {
      val |= FS1818_SYSCTRL_REG_PWDN;
      val &= (~FS1818_SYSCTRL_REG_AMPE);
    }

  if (fs1818_write_reg_data(priv, FS1818_SYSCTRL_REG, val) == ERROR)
    {
      return ERROR;
    }

  audinfo("fs1818_power_on exit\n");

  return OK;
}

/****************************************************************************
 * Name: fs1818_config_pll_off
 *
 * Description:
 *   fs1818 pll off
 *
 ****************************************************************************/

static int fs1818_config_pll_off(FAR struct fs1818u_dev_s *priv)
{
  if (fs1818_write_reg_data(priv,
                            FS1818_PLLCTRL4_REG,
                            FS1818_PLLCTRL4_DISABLE) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_shut_down
 *
 * Description:
 *   fs1818 shut down
 *
 ****************************************************************************/

static int fs1818_shut_down(FAR struct fs1818u_dev_s *priv)
{
  audinfo("fs1818_shut_down enter\n");

  if (fs1818_set_mute(priv, FSM_MUTE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_power_on(priv, FSM_DISABLE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_wait_stable(priv, FSM_WAIT_AMP_ADC_PLL_OFF) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_config_pll_off(priv) == ERROR)
    {
      return ERROR;
    }

  audinfo("fs1818_shut_down exit\n");

  return OK;
}

/****************************************************************************
 * Name: fs1818_write_default_threshold
 *
 * Description:
 *   fs1818 write default threshold
 *
 ****************************************************************************/

static int fs1818_write_default_threshold(FAR struct fs1818u_dev_s *priv)
{
  if (fs1818_write_reg_data(priv,
                            FS1818_SPKERR_REG,
                            FS1818_DEF_SPKERR) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_SPKM24_REG,
                            FS1818_DEF_SPKM24) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_SPKM6_REG,
                            FS1818_DEF_SPKM6) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_SPKRE_REG,
                            FS1818_DEF_SPKREV) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_write_default_threshold
 *
 * Description:
 *   fs1818 write default threshold
 *
 ****************************************************************************/

static int fs1818_update_threshold(FAR struct fs1818u_dev_s *priv,
                                  uint16_t r25)
{
  uint16_t value;
  uint16_t zm;
  uint16_t spk_m24;
  uint16_t spk_err;
  uint16_t spk_m6;
  uint16_t spk_rec;
  uint16_t bit_mask = 0xff;

  if (fs1818_read_reg_data(priv, FS1818_OTPPG1W2_REG, &value) == ERROR)
    {
      return ERROR;
    }

  if (value != 0)
    {
      value = value & bit_mask;
    }
  else
    {
      auderr("OTPPG1W2_REG unexpected! Set default value.\n", 0);
      value = RS_TRIM_DEFAULT;
    }

  zm = value * RS2RL_RATIO * MAGNIF_FACTOR / r25;

  /* 50011 * 65535 / (65535 + 0.0035 * 65535 * (104 - 25)) = 39178 -> 0x990A
   */

  spk_m24 = (unsigned short)((unsigned int)zm * MAGNIF_TEMPR_COEF /
  (MAGNIF_TEMPR_COEF + TEMPR_COEF * MAGNIF_TEMPR_COEF *
  (FS1818_TEMP_M24 - EXT_TEMPERATURE)));

  spk_m6 = (unsigned short)((unsigned int)zm * MAGNIF_TEMPR_COEF /
  (MAGNIF_TEMPR_COEF + TEMPR_COEF * MAGNIF_TEMPR_COEF *
  (FS1818_TEMP_M6 - EXT_TEMPERATURE)));

  spk_rec = (unsigned short)((unsigned int)zm * MAGNIF_TEMPR_COEF /
  (MAGNIF_TEMPR_COEF + TEMPR_COEF * MAGNIF_TEMPR_COEF *
  (FS1818_TEMP_REC - EXT_TEMPERATURE)));

  spk_err = (unsigned short)((unsigned int)zm * MAGNIF_TEMPR_COEF /
  (MAGNIF_TEMPR_COEF + TEMPR_COEF * MAGNIF_TEMPR_COEF *
  (FS1818_TEMP_ERR - EXT_TEMPERATURE)));

  if (fs1818_write_reg_data(priv, FS1818_SPKERR_REG, spk_err) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv, FS1818_SPKM24_REG, spk_m24) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv, FS1818_SPKM6_REG, spk_m6) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv, FS1818_SPKRE_REG, spk_rec) == ERROR)
    {
      return ERROR;
    }

  audwarn("fs1818_update_threshold spk_err:0x%04X\n", spk_err);
  audwarn("fs1818_update_threshold spk_m24:0x%04X\n", spk_m24);
  audwarn("fs1818_update_threshold spk_m6:0x%04X\n", spk_m6);
  audwarn("fs1818_update_threshold spk_rec:0x%04X\n", spk_rec);

  return ERROR;
}

/****************************************************************************
 * Name: fs1818_parse_otp
 *
 * Description:
 *   fs1818 parse otp
 *
 ****************************************************************************/

static int fs1818_parse_otp(FAR struct fs1818u_dev_s *priv, uint16_t otp_val)
{
  uint16_t cal_cnt;
  uint16_t ret;
  uint8_t otp_h8;
  uint8_t bit_mask = 0xff;
  uint8_t shift = 8;
  uint8_t cal_cntbase = 15;
  uint8_t sig_mask = 0x80;
  uint8_t max_step = 0x7f;
  uint32_t r25;

  if (otp_val == FS1818_OTPPG2_REG_DEFAULT)
    {
      /* if calibrated, high byte will not be FF */

      cal_cnt = 0;

      auderr("fs1818_parse_otp Chip Not Calibrated\n", 0);

      /* write default threshold */

      if (fs1818_write_default_threshold(priv) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv,
                                FS1818_SPKMDB_REG,
                                FS1818_SPKMDB_REG_INIT) == ERROR)
        {
          return ERROR;
        }
    }

  cal_cnt = (otp_val & bit_mask) - cal_cntbase;

  auderr("fs1818_parse_otp Calibrate count: %d\n", cal_cnt);

  priv->caliberate_count = cal_cnt;

  otp_h8 = (otp_val >> shift) & bit_mask;

  if ((otp_h8 & sig_mask) != 0)
    {
      r25 = R0_DEFAULT - CALIB_OTP_R25_STEP *
            (otp_h8 & max_step) / CALIB_MAGNIF_FACTOR;
    }
  else
    {
      r25 =  R0_DEFAULT + CALIB_OTP_R25_STEP * otp_h8 / CALIB_MAGNIF_FACTOR;
    }

  auderr("fs1818_parse_otp read r25 from otp: %d\n", r25);

  if (r25 < R0_ALLOWANCE_LOW && r25 > R0_ALLOWANCE_HIGH)
    {
      auderr("fs1818_parse_otp r25 out of range: %d\n", r25);
      return ERROR;
    }

  ret |= fs1818_update_threshold(priv, r25);

  priv->caliberate_done = true;

  audinfo("fs1818_parse_otp exit\n");

  return OK;
}

/****************************************************************************
 * Name: fs1818_read_otp
 *
 * Description:
 *   fs1818 read otp
 *
 ****************************************************************************/

static int fs1818_read_otp(FAR struct fs1818u_dev_s *priv)
{
  uint16_t pll;
  uint16_t otpe8;

  if (fs1818_read_reg_data(priv, FS1818_PLLCTRL4_REG, &pll) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_PLLCTRL4_REG,
                            FS1818_PLLCTRL4_ENABLE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPACC_REG,
                            FS1818_OTPACC_ENABLE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_read_reg_data(priv, FS1818_OTPPG2_REG, &otpe8) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_parse_otp(priv, otpe8) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPACC_REG,
                            FS1818_OTPACC_DISABLE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv, FS1818_PLLCTRL4_REG, pll) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

static uint16_t set_bf_val(uint16_t *pval,
                          const uint16_t bf,
                          const uint16_t bf_val)
{
  uint8_t len = (bf >> 12) & 0x0f;
  uint8_t pos = (bf >> 8) & 0x0f;
  uint16_t new_val;
  uint16_t old_val;
  uint16_t msk;

  if (!pval)
    {
      return 0xffff;
    }

  old_val = new_val = *pval;
  msk = ((1 << (len + 1)) - 1) << pos;
  new_val &= ~msk;
  new_val |= bf_val << pos;
  *pval = new_val;

  return old_val;
}

/****************************************************************************
 * Name: fs1818_start_up
 *
 * Description:
 *   fs1818 start up
 *
 ****************************************************************************/

static int fs1818_start_up(FAR struct fs1818u_dev_s *priv)
{
  uint16_t i2sctrl;

  audio_out_format_t pconfig;
  pconfig.sample_rate  = priv->samprate;
  pconfig.bits         = priv->bpsamp;
  pconfig.channel_num  = 2;

  audinfo("fs1818_start_up enter\n");

#ifdef CONFIG_AUDIO_FS1818U_DEBUG
  fs1818_dump_reg(priv);
#endif

  if (fs1818_read_reg_data(priv, FS1818_I2SCTRL_REG, &i2sctrl) == ERROR)
    {
      return ERROR;
    }

  set_bf_val(&i2sctrl, FS1818_I2SCTRL_I2SDOE, 0x1);

  if (fs1818_write_reg_data(priv, FS1818_I2SCTRL_REG, i2sctrl) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_config_i2s_and_pll(priv, &pconfig) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_power_on(priv, FSM_ENABLE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_wait_stable(priv, FSM_WAIT_AMP_ON) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_set_mute(priv, FSM_UNMUTE) == ERROR)
    {
      return ERROR;
    }

  audinfo("fs1818_start_up exit\n");

#ifdef CONFIG_AUDIO_FS1818U_DEBUG
  fs1818_dump_reg(priv);
#endif

  return OK;
}

/****************************************************************************
 * Name: fs1818_start_hw
 *
 * Description:
 *   fs1818 driver startup
 *
 ****************************************************************************/

static int fs1818_start_hw(FAR struct fs1818u_dev_s *priv, uint8_t mode)
{
  static uint8_t g_mode = PRESET_MUSIC;

  audinfo("fs1818_start_hw enter\n");

  if (mode != g_mode)
    {
      syslog(LOG_WARNING, "scenario mode need to be change\n");

      if (fs1818_write_fw(priv, mode) == ERROR)
        {
          auderr("fs1818_write_fw failed\n");
          return ERROR;
        }
      else
        {
          g_mode = mode;
        }
    }

  if (fs1818_read_otp(priv) == ERROR)
    {
      return ERROR;
    }

  if (priv->mode == NORMAL_MODE)
    {
      if (fs1818_set_volume(priv, priv->nor_volume) == ERROR)
        {
          return ERROR;
        }
    }
  else
    {
      if (fs1818_set_volume(priv, priv->dsp_volume) == ERROR)
        {
          return ERROR;
        }
    }

  if (fs1818_start_up(priv) == ERROR)
    {
      return ERROR;
    }

#ifdef CONFIG_AUDIO_FS1818U_DEBUG
  fs1818_dump_reg(priv);
#endif

  auderr("fs1818_start_hw exit\n");

  return OK;
}

/****************************************************************************
 * Name: fs1818_config_i2s
 *
 * Description:
 *   fs1818 configure i2s
 *
 ****************************************************************************/

static int fs1818_config_i2s(FAR struct fs1818u_dev_s *priv,
                            FAR audio_out_format_t *pconfig)
{
  uint16_t i2s_rate;
  uint16_t i2s_width;
  uint16_t i2sctrl;

  switch (pconfig->sample_rate)
    {
      case AUD_SAMPRATE_8000:
        i2s_rate = 0;
        break;

      case AUD_SAMPRATE_16000:
        i2s_rate = 3;
        break;

      case AUD_SAMPRATE_32000:
        i2s_rate = 8;
        break;

      case AUD_SAMPRATE_44100:
        i2s_rate = 7;
        break;

      case AUD_SAMPRATE_48000:
        i2s_rate = 8;
        break;

      default:
        i2s_rate = 3;
        break;
    }

  switch (pconfig->bits)
    {
      case AUD_BITS_16:
        i2s_width = 3;
        break;

      case AUD_BITS_24:
        i2s_width = 3;
        break;

      default:
        i2s_width = 3;
        break;
    }

  if (fs1818_read_reg_data(priv, FS1818_I2SCTRL_REG, &i2sctrl) == ERROR)
    {
      return ERROR;
    }

  set_bf_val(&i2sctrl, FS1818_I2SCTRL_I2SSR, i2s_rate);
  set_bf_val(&i2sctrl, FS1818_I2SCTRL_I2SF, i2s_width);
  set_bf_val(&i2sctrl, FS1818_I2SCTRL_CHS12, 3);

  if (fs1818_write_reg_data(priv, FS1818_I2SCTRL_REG, i2sctrl) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_config_pll_on
 *
 * Description:
 *   fs1818 configure pll on
 *
 ****************************************************************************/

static int fs1818_config_pll_on(FAR struct fs1818u_dev_s *priv,
                                FAR audio_out_format_t *pconfig)
{
  int idx;
  uint32_t i2s_bclk = pconfig->sample_rate *
                      pconfig->bits * pconfig->channel_num;

  /* config pll need disable pll firstly */

  if (fs1818_write_reg_data(priv,
                            FS1818_PLLCTRL4_REG,
                            FS1818_PLLCTRL4_DISABLE) == ERROR)
    {
      return ERROR;
    }

  fs1818_usleep(1000 * 1);

  for (idx = 0; idx < ARRAY_SIZE(g_fs1818_pll_tbl); idx++)
    {
      if (g_fs1818_pll_tbl[idx].bclk == i2s_bclk)
        {
          break;
        }
    }

  syslog(LOG_WARNING, "fs1818_config_pll_on enter, bclk[%d]: %ld\n",
        idx, i2s_bclk);

  if (idx >= ARRAY_SIZE(g_fs1818_pll_tbl))
    {
      auderr("Not found bclk: %d, rate: %d", i2s_bclk, pconfig->sample_rate);
      return -EINVAL;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPACC_REG,
                            FS1818_OTPACC_ENABLE) == ERROR)
    {
      return ERROR;
    }

  if (pconfig->sample_rate == 32000)
    {
      if (fs1818_write_reg_data(priv, FS1818_ANACTRL_REG, 0x0101) == ERROR)
        {
          return ERROR;
        }
    }
  else
    {
      if (fs1818_write_reg_data(priv, FS1818_ANACTRL_REG, 0x0100) == ERROR)
        {
          return ERROR;
        }
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_PLLCTRL1_REG,
                            g_fs1818_pll_tbl[idx].c1) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_PLLCTRL2_REG,
                            g_fs1818_pll_tbl[idx].c2) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_PLLCTRL3_REG,
                            g_fs1818_pll_tbl[idx].c3) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_PLLCTRL4_REG,
                            FS1818_PLLCTRL4_ENABLE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPACC_REG,
                            FS1818_OTPACC_DISABLE) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_config_i2s_and_pll
 *
 * Description:
 *   fs1818 configure i2s and pll
 *
 ****************************************************************************/

static int fs1818_config_i2s_and_pll(FAR struct fs1818u_dev_s *priv,
                                     FAR audio_out_format_t * pconfig)
{
  if (fs1818_config_i2s(priv, pconfig) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_config_pll_on(priv, pconfig) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_dump_reg
 *
 * Description:
 *   fs1818 dump registers
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_FS1818U_DEBUG
static int fs1818_dump_reg(FAR struct fs1818u_dev_s *priv)
{
  uint16_t reg_cnt;
  uint16_t value;
  int ret;

  if (!priv->dump_info)
    {
      return ERROR;
    }

  syslog(LOG_WARNING, "----fs1818_dump_reg enter----\n");

  for (reg_cnt = 0; reg_cnt < 0xff; reg_cnt++)
    {
      ret = fs1818_read_reg_data(priv, reg_cnt, &value);

      if (ret == OK)
        {
          syslog(LOG_ERR, "reg:0x%02x--->0x%04x\n", reg_cnt, value);
        }
      else
        {
          auderr("read register failed\n");
          return ERROR;
        }
    }

  syslog(LOG_WARNING, "----fs1818_dump_reg exit----\n");

  return OK;
}
#endif

/****************************************************************************
 * Name: fs1818_set_volume
 *
 * Description:
 *   fs1818 set volume
 *
 ****************************************************************************/

static int fs1818_set_volume(FAR struct fs1818u_dev_s *priv, uint8_t volume)
{
  if (volume < 0 || volume > 0xff)
    {
      auderr("invalid volume: %d, default 0dB\n", volume);
      volume = 0xff;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_AUDIOCTRL_REG,
                            (volume << 8) & 0xff00) == ERROR)
    {
      return ERROR;
    }

  syslog(LOG_WARNING, "fs1818_set_volume: %d\n", volume);

  return OK;
}

/****************************************************************************
 * Name: fs1818_set_volume
 *
 * Description:
 *   fs1818 set volume
 *
 ****************************************************************************/

static int fs1818_dsp_bypass(FAR struct fs1818u_dev_s *priv, int mode)
{
  uint16_t dsp_ctl;

  if (mode == NORMAL_MODE)
    {
      auderr("fs1818_dsp_bypass bypass dsp\n");

      if (fs1818_read_reg_data(priv, FS1818_DSPCTRL_REG, &dsp_ctl) == ERROR)
        {
          return ERROR;
        }

      dsp_ctl &= ~FS1818_DSPCTRL_REG_DSPEN;

      if (fs1818_write_reg_data(priv, FS1818_DSPCTRL_REG, dsp_ctl) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv, 0x0b, 0xca91) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv, 0x06,
                                (priv->nor_volume << 8) & 0xff00) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv, 0xc0, 0x19ee) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv, 0xd7, 0x1020) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv, 0x0b, 0x0000) == ERROR)
        {
          return ERROR;
        }
    }
  else
    {
      auderr("fs1818_dsp_bypass unbypass dsp\n");

      if (fs1818_read_reg_data(priv, FS1818_DSPCTRL_REG, &dsp_ctl) == ERROR)
        {
          return ERROR;
        }

      dsp_ctl |= FS1818_DSPCTRL_REG_DSPEN;

      if (fs1818_write_reg_data(priv, FS1818_DSPCTRL_REG, dsp_ctl) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv, 0x0b, 0xca91) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv, 0xc0, 0x19ee) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv, 0xd7, 0x0020) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv, 0x06,
                                (priv->dsp_volume << 8) & 0xff00) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_reg_data(priv, 0x0b, 0x0000) == ERROR)
        {
          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_calibrate_config
 *
 * Description:
 *   fs1818 caliberate configuration
 *
 ****************************************************************************/

static int fs1818_calibrate_config(FAR struct fs1818u_dev_s *priv)
{
  uint16_t status;
  uint16_t first_delay = 2500;
  uint16_t norm_delay = 500;

  priv->nor_volume = 0xf3;

  if (fs1818_dsp_bypass(priv, DSP_MODE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPACC_REG,
                            FS1818_OTPACC_ENABLE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_ZMCONFIG_REG,
                            FS1818_ZMCONFIG_REG_CALIB_BIT) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_TSCTRL_REG,
                            FS1818_TSCTRL_TS_NOFF) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_SPKERR_REG,
                            FS1818_SPKERR_REG_CLEAR) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_SPKM24_REG,
                            FS1818_SPKM24_REG_CLEAR) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_SPKM6_REG,
                            FS1818_SPKM6_REG_CLEAR) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_SPKRE_REG,
                            FS1818_SPKRE_REG_CLEAR) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_ADCENV_REG,
                            FS1818_ADCENV_REG_AMP_DTEN_ALL) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_ADCTIME_REG,
                            FS1818_ADCTIME_REG_VALUE) == ERROR)
    {
      return ERROR;
    }

  if (!priv->caliberate_done)
    {
      fs1818_usleep(first_delay);
    }
  else
    {
      fs1818_usleep(norm_delay);
    }

  /* check i2s clk */

  if (fs1818_read_reg_data(priv, FS1818_STATUS_REG, &status) == ERROR)
    {
      return ERROR;
    }

  auderr("fs1818 status reg: 00 0x%04x\n", status);

  return OK;
}

/****************************************************************************
 * Name: fs1818_get_zmdata
 *
 * Description:
 *   fs1818 get zmdata
 *
 ****************************************************************************/

static uint16_t fs1818_get_zmdata(FAR struct fs1818u_dev_s *priv,
                                  uint16_t *zm)
{
  int ret;
  uint16_t zmdata;
  uint16_t count  = 0;
  uint16_t done   = 0;
  uint16_t minval = 0;
  uint16_t retry  = 0;
  uint16_t delay_ms = 100;
  uint16_t preval   = 0;
  uint16_t retry_max = 80;
  uint16_t zm_def   = 0xffff;
  uint16_t zm_done  = 10;

  auderr("fs1818_get_zmdata enter\n");

  while (retry < retry_max)
    {
      fs1818_usleep(delay_ms * 1000);

      ret = fs1818_read_reg_data(priv, FS1818_ZMDATA_REG, &zmdata);

      if ((zmdata == 0) || (zmdata == zm_def) || ret)
        {
          auderr("fs1818_get_zmdata invalid data, zmdata:%d ret:%d\n",
                  zmdata, ret);

          fs1818_usleep(delay_ms * 1000);
          retry++;
          continue;
        }

      if (!preval || abs(preval - zmdata) > FS1818_ZMDELTA_MAX)
        {
          preval = zmdata;
          count = 1;
          minval = zmdata;
        }
      else
        {
          count++;
          if (zmdata < minval)
            {
              minval = zmdata;
            }
        }

      if (count >= zm_done)
        {
          done = 1;
          break;
        }

      retry++;
    }

  if (retry >= retry_max && !done)
    {
      auderr("fs1818 get zmdata failed\n");
      return 0;
    }

  *zm = minval;

  auderr("fs1818_get_zmdata exit\n");

  return done;
}

/****************************************************************************
 * Name: fs1818_calc_threshold
 *
 * Description:
 *   fs1818 calc threshold
 *
 ****************************************************************************/

static int fs1818_calc_threshold(FAR struct fs1818u_dev_s *priv,
                                uint16_t zmdata)
{
  uint16_t spk_m6;
  uint16_t spk_m24;
  uint16_t spk_err;
  uint16_t spk_rec;

  spk_m24 = (unsigned short)((unsigned int)zmdata * MAGNIF_TEMPR_COEF /
        (MAGNIF_TEMPR_COEF + TEMPR_COEF * MAGNIF_TEMPR_COEF *
        (FS1818_TEMP_M24 - EXT_TEMPERATURE)));
  auderr("fs1818 spk_m24 = 0x%04X\n", spk_m24);

  spk_m6 = (unsigned short)((unsigned int)zmdata * MAGNIF_TEMPR_COEF /
      (MAGNIF_TEMPR_COEF + TEMPR_COEF * MAGNIF_TEMPR_COEF *
      (FS1818_TEMP_M6 - EXT_TEMPERATURE)));
  auderr("fs1818 spk_m6 = 0x%04X\n", spk_m6);

  spk_rec = (unsigned short)((unsigned int)zmdata * MAGNIF_TEMPR_COEF /
        (MAGNIF_TEMPR_COEF + TEMPR_COEF * MAGNIF_TEMPR_COEF *
        (FS1818_TEMP_REC - EXT_TEMPERATURE)));
  auderr("fs1818 spk_rec = 0x%04X\n", spk_rec);

  spk_err = (unsigned short)((unsigned int)zmdata * MAGNIF_TEMPR_COEF /
        (MAGNIF_TEMPR_COEF + TEMPR_COEF * MAGNIF_TEMPR_COEF *
        (FS1818_TEMP_ERR - EXT_TEMPERATURE)));
  auderr("fs1818 spk_err = 0x%04X\n", spk_err);

  /* update threshold */

  if (fs1818_write_reg_data(priv, FS1818_SPKERR_REG, spk_err) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv, FS1818_SPKM24_REG, spk_m24) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv, FS1818_SPKM6_REG, spk_m6) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv, FS1818_SPKRE_REG, spk_rec) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_ADCTIME_REG,
                            FS1818_ADCTIME_REG_VALUE) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_TSCTRL_REG,
                            FS1818_TSCTRL_TSAUTO_OFF) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_get_r25_byte
 *
 * Description:
 *   fs1818 get r25 byte
 *
 ****************************************************************************/

static uint16_t fs1818_get_r25_byte(uint16_t r25)
{
  uint8_t val;
  uint8_t step_max = 0x7f;
  uint8_t sig_bit = 0x80;

  val = (unsigned char)((int)abs(r25 - R0_DEFAULT) *
        CALIB_MAGNIF_FACTOR / CALIB_OTP_R25_STEP);

  if (val > step_max)
    {
      val = step_max;
    }

  val = (r25 >= R0_DEFAULT) ? val : (unsigned char)(val | sig_bit);

  if (0xff == val)
    {
      val -= 1;
    }

  return val;
}

/****************************************************************************
 * Name: fs1818_store_config
 *
 * Description:
 *   fs1818 store configuration
 *
 ****************************************************************************/

static int fs1818_store_config(FAR struct fs1818u_dev_s *priv,
                               FAR uint16_t *count, FAR uint16_t *value)
{
  uint8_t bit_mask    = 0xff;
  uint8_t calcnt_base = 15;
  int ret = OK;

  if (fs1818_write_reg_data(priv,
                            FS1818_SYSCTRL_REG,
                            FS1818_SYSCTRL_REG_PWON) == ERROR)
    {
      ret = ERROR;
      goto error_flag;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_BSTCTRL_REG,
                            FS1818_BSTCTRL_BST_DISABLE_ALL) == ERROR)
    {
      ret = ERROR;
      goto error_flag;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_PLLCTRL4_REG,
                            FS1818_PLLCTRL4_DISABLE_OSC) == ERROR)
    {
      ret = ERROR;
      goto error_flag;
    }

  /* OTP page */

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPADDR_REG,
                            FS1818_OTPADDR_ADDR) == ERROR)
    {
      ret = ERROR;
      goto error_flag;
    }

  /* OTP read clear */

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPCMD_REG,
                            FS1818_OTPCMD_CMD1) == ERROR)
    {
      ret = ERROR;
      goto error_flag;
    }

  /* OTP read */

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPCMD_REG,
                            FS1818_OTPCMD_CMD2) == ERROR)
    {
      ret = ERROR;
      goto error_flag;
    }

  if (fs1818_wait_stable(priv, FSM_WAIT_OTP_READY) == ERROR)
    {
      ret = ERROR;
      goto error_flag;
    }

error_flag:

  if (ret == ERROR)
    {
      auderr("fs1818_force_calibrate otp busy1\n");
      return ERROR;
    }

  if (fs1818_read_reg_data(priv, FS1818_OTPRDATA_REG, value) == ERROR)
    {
      return ERROR;
    }

  if (value == FS1818_OTPPG2_REG_DEFAULT)
    {
      *count = 0;
      auderr("fs1818 Calibrate not start\n");
    }
  else
    {
      *count = (*value & bit_mask) - calcnt_base;
      auderr("fs1818 Calibrate count: %d\n", *count);
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_store_otp
 *
 * Description:
 *   fs1818 store otp
 *
 ****************************************************************************/

static int fs1818_store_otp(FAR struct fs1818u_dev_s *priv, uint8_t valotp)
{
  if (fs1818_write_reg_data(priv,
                            FS1818_PLLCTRL4_REG,
                            FS1818_PLLCTRL4_DISABLE_ZMADC) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_BSTCTRL_REG,
                            FS1818_BSTCTRL_BST_BIT_EN) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPADDR_REG,
                            FS1818_OTPADDR_ADDR) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv, FS1818_OTPWDATA_REG, valotp) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPCMD_REG,
                            FS1818_OTPCMD_CMD1) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPCMD_REG, FS1818_OTPCMD_CMD3) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_wait_stable(priv, FSM_WAIT_AMP_ON) == ERROR)
    {
      auderr("fs1818 failed to wait boost ssend\n");
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPCMD_REG,
                            FS1818_OTPCMD_CMD4) == ERROR)
    {
      return ERROR;
    }

  fs1818_usleep(1000 * 5);

  if (fs1818_wait_stable(priv, FSM_WAIT_OTP_READY) == ERROR)
    {
      auderr("fs1818_force_calibrate otp busy2\n");
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPCMD_REG,
                            FS1818_OTPCMD_CMD1) == ERROR)
    {
      return ERROR;
    }

  fs1818_usleep(1000 * 1);

  if (fs1818_write_reg_data(priv,
                            FS1818_BSTCTRL_REG,
                            FS1818_BSTCTRL_BST_DISABLE_ALL) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_PLLCTRL4_REG,
                            FS1818_PLLCTRL4_DISABLE_OSC) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_BSTCTRL_REG,
                            FS1818_BSTCTRL_BST_DISABLE_ALL) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPADDR_REG,
                            FS1818_OTPADDR_ADDR) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPCMD_REG,
                            FS1818_OTPCMD_CMD1) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPCMD_REG,
                            FS1818_OTPCMD_CMD2) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: fs1818_get_byte_r25
 *
 * Description:
 *   fs1818 get r25 byte
 *
 ****************************************************************************/

static uint16_t fs1818_get_byte_r25(uint8_t byte)
{
  uint8_t sig_bit = 0x80;
  uint8_t step_max = 0x7f;

  if ((byte & sig_bit) != 0)
    {
      /* minus value */

      return R0_DEFAULT - CALIB_OTP_R25_STEP *
            (byte & step_max) / CALIB_MAGNIF_FACTOR;
    }

  return R0_DEFAULT + CALIB_OTP_R25_STEP * byte / CALIB_MAGNIF_FACTOR;
}

/****************************************************************************
 * Name: fs1818_store_to_otp
 *
 * Description:
 *   fs1818 store value to otp
 *
 ****************************************************************************/

static int fs1818_store_to_otp(FAR struct fs1818u_dev_s *priv,
                              uint8_t valotp)
{
  uint16_t value = 0;
  uint16_t valpwr;
  uint16_t valuec0;
  uint16_t valuec4;
  uint16_t count;
  uint16_t delta;
  uint8_t shift = 8;
  uint8_t bit_mask = 0xff;
  uint8_t calcnt_base = 15;

  auderr("fs1818_store_to_otp enter, otp:0x%04x\n", valotp);

  if (fs1818_read_reg_data(priv, FS1818_SYSCTRL_REG, &valpwr) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_read_reg_data(priv, FS1818_BSTCTRL_REG, &valuec0) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_read_reg_data(priv, FS1818_PLLCTRL4_REG, &valuec4) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_store_config(priv, &count, &value) == ERROR)
    {
      goto otp_exit;
    }

  delta = fs1818_get_byte_r25(valotp) -
          fs1818_get_byte_r25((value >> shift) & bit_mask);

  if (delta < 0)
    {
      delta *= -1;
    }

  auderr("fs1818 Calibrate delta: %d\n", delta);
  auderr("fs1818 Calibrate otp-delta:%d\n", OTP_MIN_DELTA);

  if (count > 0 && delta < OTP_MIN_DELTA)
    {
      auderr("fs1818 no need to write otp\n");
      goto otp_exit;
    }

  if (count >= CALIB_MAX_USER_TRY)
    {
      auderr("fs1818 otp write count exceeds max\n");
      goto otp_exit;
    }

  if (fs1818_store_otp(priv, valotp) == ERROR)
    {
      goto otp_exit;
    }

  if (fs1818_wait_stable(priv, FSM_WAIT_OTP_READY) == ERROR)
    {
      auderr("fs1818_force_calibrate otp busy3\n");
      goto otp_exit;
    }

  if (fs1818_read_reg_data(priv, FS1818_OTPRDATA_REG, &value) == ERROR)
    {
      return ERROR;
    }

  if (((value >> shift) & bit_mask) == valotp)
    {
      /* Update count */

      count = (value & bit_mask) - calcnt_base;
      auderr("fs1818 readback succeeded with count = %d\n", count);
    }
  else
    {
      auderr("fs1818 calibrated read back failed with value = 0x%0,"
             "Should not happen\n", value);
      auderr("fs1818 calibrated read back failed with expected = 0x%04X--,"
             "Should not happen\n", valotp);
    }

otp_exit:

  if (fs1818_write_reg_data(priv, FS1818_BSTCTRL_REG, valuec0) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv, FS1818_PLLCTRL4_REG, valuec4) == ERROR)
    {
      return ERROR;
    }

  /* Recover power state */

  if (fs1818_write_reg_data(priv, FS1818_SYSCTRL_REG, valpwr) == ERROR)
    {
      return ERROR;
    }

  auderr("fs1818_store_to_otp exit");

  return OK;
}

/****************************************************************************
 * Name: fs1818_force_calibrate
 *
 * Description:
 *   fs1818 force caliberate
 *
 ****************************************************************************/

static int fs1818_force_calibrate(FAR struct fs1818u_dev_s *priv,
                                  uint8_t force, bool store)
{
  uint16_t zmdata;
  uint16_t vale6;
  uint16_t r25_byte;
  uint16_t r25;
  uint16_t bit_mask = 0xff;

  auderr("fs1818_force_calibrate enter\n");

  if (fs1818_get_zmdata(priv, &zmdata) == 0)
    {
      if (fs1818_write_reg_data(priv,
                                FS1818_OTPACC_REG,
                                FS1818_OTPACC_DISABLE) == ERROR)
        {
          return ERROR;
        }

      if (fs1818_write_default_threshold(priv) == ERROR)
        {
          return ERROR;
        }

      auderr("fs1818_force_calibrate read data failed\n");

      return ERROR;
    }

  /* get r25 */

  if (fs1818_read_reg_data(priv, FS1818_OTPPG1W2_REG, &vale6) == ERROR)
    {
      return ERROR;
    }

  if (vale6 != 0)
    {
      vale6 = vale6 & bit_mask;
    }
  else
    {
      auderr("fs1818 OTPPG1W2_REG unexpected! Set default value\n");
      vale6 = RS_TRIM_DEFAULT;
    }

  r25 = vale6 * RS2RL_RATIO * MAGNIF_FACTOR / zmdata;

  if (r25 < R0_ALLOWANCE_HIGH && r25 > R0_ALLOWANCE_LOW)
    {
      auderr("fs1818 SPK R25 = %d\n", r25);
    }
  else
    {
      if (fs1818_write_reg_data(priv,
                                FS1818_OTPACC_REG,
                                FS1818_OTPACC_DISABLE) == ERROR)
        {
          return ERROR;
        }

      auderr("fs1818 speaker impedance out of range %d\n", r25);

      if (fs1818_write_default_threshold(priv) == ERROR)
        {
          return ERROR;
        }

      return ERROR;
    }

  if (fs1818_calc_threshold(priv, zmdata) == ERROR)
    {
      return ERROR;
    }

  if (store)
    {
      r25_byte = fs1818_get_r25_byte(r25);

      /* store to otp */

      if (fs1818_store_to_otp(priv, r25_byte) == ERROR)
        {
          return ERROR;
        }
    }

  auderr("fs1818 recoverty settings\n");
  if (fs1818_write_reg_data(priv,
                            FS1818_ZMCONFIG_REG,
                            FS1818_ZMCONFIG_REG_NORMAL_BIT) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_reg_data(priv,
                            FS1818_OTPACC_REG,
                            FS1818_OTPACC_DISABLE) == ERROR)
    {
      return ERROR;
    }

  auderr("fs1818_force_calibrate exit\n");

  return (int) r25;
}

static int fs1818_update_caliberate_count(FAR struct fs1818u_dev_s *priv)
{
  if (fs1818_read_otp(priv) == ERROR)
    {
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fs1818_late_initialize
 *
 * Description:
 *   fs1818 late initialize
 *
 ****************************************************************************/

int fs1818_late_initialize(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct fs1818u_dev_s *priv = (FAR struct fs1818u_dev_s *)dev;

  if (fs1818_reg_init(priv) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_write_fw(priv, priv->scenario_mode) == ERROR)
    {
      return ERROR;
    }

  if (fs1818_shut_down(priv) == ERROR)
    {
      return ERROR;
    }

#ifdef CONFIG_AUDIO_FS1818U_DEBUG
  fs1818_dump_reg(priv);
#endif

  return OK;
}

/****************************************************************************
 * Name: fs1818_initialize
 *
 * Description:
 *   fs1818 early initialize
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
fs1818_initialize(FAR struct i2c_master_s *i2c,
                  FAR struct fs1818u_lower_s * lower)
{
  FAR struct fs1818u_dev_s *priv;
  int ret;

  /* Allocate a WM8994 device structure */

  priv = kmm_zalloc(sizeof(struct fs1818u_dev_s));

  if (priv)
    {
      priv->dev.ops = &g_audioops;
      priv->lower   = lower;
      priv->i2c     = i2c;

      ret = fs1818_check_id(priv);
      if (ret < 0)
        {
          auderr("fs1818 check failed!!!\n");
          goto errout_with_dev;
        }

      priv->scenario_mode = PRESET_MUSIC;
      priv->dsp_volume    = 0xbf;
      priv->nor_volume    = 0xef;
      priv->dump_info     = false;
      priv->mode          = DSP_MODE;
      priv->caliberate_done   = false;
      priv->store_cali_value  = false;
      priv->caliberate_result = 0;
      priv->caliberate_count  = 0;

      return &priv->dev;
    }

errout_with_dev:
  kmm_free(priv);
  return NULL;
}
