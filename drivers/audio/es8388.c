/****************************************************************************
 * drivers/audio/es8388.c
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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <fixedmath.h>
#include <debug.h>
#include <pthread.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/queue.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/es8388.h>

#include "es8388.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if !defined(CONFIG_ES8388_REGDUMP)
static
#endif
uint8_t     es8388_readreg(FAR struct es8388_dev_s *priv, uint8_t regaddr);
static void es8388_writereg(FAR struct es8388_dev_s *priv,
                            uint8_t regaddr,
                            uint16_t regval);
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static void es8388_setvolume(FAR struct es8388_dev_s *priv,
                             uint16_t volume);
#endif
static void es8388_setmclkfrequency(FAR struct es8388_dev_s *priv);
static void es8388_setmute(FAR struct es8388_dev_s *priv, bool enable);
static void es8388_setbitspersample(FAR struct es8388_dev_s *priv);
static void es8388_setsamplerate(FAR struct es8388_dev_s *priv);
static int  es8388_getcaps(FAR struct audio_lowerhalf_s *dev,
                           int type,
                           FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8388_configure(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session,
                             FAR const struct audio_caps_s *caps);
#else
static int  es8388_configure(FAR struct audio_lowerhalf_s *dev,
                             FAR const struct audio_caps_s *caps);
#endif
static int  es8388_shutdown(FAR struct audio_lowerhalf_s *dev);
static void es8388_senddone(FAR struct i2s_dev_s *i2s,
                            FAR struct ap_buffer_s *apb,
                            FAR void *arg,
                            int result);
static void es8388_returnbuffers(FAR struct es8388_dev_s *priv);
static int  es8388_sendbuffer(FAR struct es8388_dev_s *priv);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8388_start(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session);
#else
static int  es8388_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8388_stop(FAR struct audio_lowerhalf_s *dev,
                        FAR void *session);
#else
static int  es8388_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8388_pause(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session);
static int  es8388_resume(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session);
#else
static int  es8388_pause(FAR struct audio_lowerhalf_s *dev);
static int  es8388_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int  es8388_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb);
static int  es8388_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb);
static int  es8388_ioctl(FAR struct audio_lowerhalf_s *dev,
                         int cmd,
                         unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8388_reserve(FAR struct audio_lowerhalf_s *dev,
                          FAR void **session);
#else
static int  es8388_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8388_release(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
#else
static int  es8388_release(FAR struct audio_lowerhalf_s *dev);
#endif
static void *es8388_workerthread(pthread_addr_t pvarg);
static void es8388_audio_output(FAR struct es8388_dev_s *priv);
#if 0
static void es8388_audio_input(FAR struct es8388_dev_s *priv);
#endif
static void es8388_reset(FAR struct es8388_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops =
{
  es8388_getcaps,       /* getcaps        */
  es8388_configure,     /* configure      */
  es8388_shutdown,      /* shutdown       */
  es8388_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  es8388_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  es8388_pause,         /* pause          */
  es8388_resume,        /* resume         */
#endif
  NULL,                 /* allocbuffer    */
  NULL,                 /* freebuffer     */
  es8388_enqueuebuffer, /* enqueue_buffer */
  es8388_cancelbuffer,  /* cancel_buffer  */
  es8388_ioctl,         /* ioctl          */
  NULL,                 /* read           */
  NULL,                 /* write          */
  es8388_reserve,       /* reserve        */
  es8388_release        /* release        */
};

/****************************************************************************
 * Name: es8388_readreg
 *
 * Description:
 *    Read the specified 8-bit register from the ES8388 device through I2C.
 *
 *   priv - A reference to the driver state structure.
 *   regaddr - Address of the register to be read.
 *
 * Returned Value:
 *   On success, the byte stored in the register.
 *   On failure, 0.
 *
 ****************************************************************************/

#if !defined(CONFIG_ES8388_REGDUMP)
static
#endif
uint8_t es8388_readreg(FAR struct es8388_dev_s *priv, uint8_t regaddr)
{
  int retries;

  /* Try up to three times to read the register */

  for (retries = 0; retries < 3; retries++)
    {
      struct i2c_msg_s msg[2];
      uint8_t data;
      int ret;

      /* Set up to write the address */

      msg[0].frequency = priv->lower->frequency;
      msg[0].addr      = priv->lower->address;
      msg[0].flags     = 0;
      msg[0].buffer    = &regaddr;
      msg[0].length    = 1;

      /* Followed by the read data */

      msg[1].frequency = priv->lower->frequency;
      msg[1].addr      = priv->lower->address;
      msg[1].flags     = I2C_M_READ;
      msg[1].buffer    = &data;
      msg[1].length    = 12;

      /* Read the register data. The returned value is the number messages
       * completed.
       */

      ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up? Try to shake the bus free */

          audwarn("WARNING: I2C_TRANSFER failed: %d ... Resetting\n", ret);

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              auderr("ERROR: I2C_RESET failed: %d\n", ret);
              break;
            }
#else
          auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
#endif
        }
      else
        {
          /* The I2C transfer was successful... break out of the loop and
           * return the value read.
           */

          audinfo("Read: %02x -> %02x\n", regaddr, data);
          return data;
        }

      audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }

  /* No error indication is returned on a failure... just return zero */

  return 0;
}

/****************************************************************************
 * Name: es8388_writereg
 *
 * Description:
 *   Write the specified 8-bit register to the ES8388 device through I2C.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   regaddr - Address of the register to be written.
 *   regval - Value to be written.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void es8388_writereg(FAR struct es8388_dev_s *priv,
                            uint8_t regaddr,
                            uint16_t regval)
{
  struct i2c_config_s config;
  int retries;

  /* Setup up the I2C configuration */

  config.frequency = priv->lower->frequency;
  config.address   = priv->lower->address;
  config.addrlen   = 7;

  /* Try up to three times to write the register */

  for (retries = 0; retries < 3; retries++)
    {
      uint8_t data[2];
      int ret;

      /* Set up the data to write */

      data[0] = regaddr;
      data[1] = regval;

      /* Read the register data. The returned value is the number messages
       * completed.
       */

      ret = i2c_write(priv->i2c, &config, data, 2);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up? Try to shake the bus free */

          audwarn("WARNING: i2c_write failed: %d ... Resetting\n", ret);

          ret = I2C_RESET(priv->i2c);
          if (ret < 0)
            {
              auderr("ERROR: I2C_RESET failed: %d\n", ret);
              break;
            }
#else
          auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
#endif
        }
      else
        {
          /* The I2C transfer was successful... break out of the loop and
           * return.
           */

          audinfo("Write: %02x <- %02x\n", regaddr, regval);
          return;
        }

      audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }
}

/****************************************************************************
 * Name: es8388_setvolume
 *
 * Description:
 *   Set the right and left volume values in the ES8388 device based on the
 *   desired volume and balance settings.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   volume - The volume to be set in the codec (0..1000).
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static void es8388_setvolume(FAR struct es8388_dev_s *priv, uint16_t volume)
{
  uint16_t leftlvl;
  int16_t dbleftlvl;
  uint16_t rightlvl;
  int16_t dbrightlvl;

  if (volume > AUDIO_VOLUME_MAX)
    {
      audwarn("Warning: Volume > AUDIO_VOLUME_MAX!\n");
      volume = AUDIO_VOLUME_MAX;
    }

  audinfo("Volume = %u\n", volume);

#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  /* Calculate the left channel volume level {0..1000} */

  if (priv->balance <= AUDIO_BALANCE_CENTER)
    {
      leftlvl = volume;
    }
  else if (priv->balance == AUDIO_BALANCE_RIGHT)
    {
      leftlvl = AUDIO_VOLUME_MIN;
    }
  else
    {
      leftlvl = ((((AUDIO_BALANCE_RIGHT - priv->balance) * 100) /
        AUDIO_BALANCE_CENTER) * volume) / 100;
    }

  /* Calculate the right channel volume level {0..1000} */

  if (priv->balance >= AUDIO_BALANCE_CENTER)
    {
      rightlvl = volume;
    }
  else if (priv->balance == AUDIO_BALANCE_LEFT)
    {
      rightlvl = AUDIO_VOLUME_MIN;
    }
  else
    {
      rightlvl = (((priv->balance * 100) / AUDIO_BALANCE_CENTER) * volume) /
        100;
    }

#  else
  leftlvl = priv->volume;
  rightlvl = priv->volume;
#  endif

  /* Convert from (0..1000) to (-96..0) */

  dbleftlvl = (int16_t)
    (leftlvl ? (20 * log10f(rightlvl / AUDIO_VOLUME_MAX_FLOAT)) : -96);
  dbrightlvl = (int16_t)
    (rightlvl ? (20 * log10f(rightlvl / AUDIO_VOLUME_MAX_FLOAT)) : -96);

  audinfo("Volume: dbleftlvl = %d, dbrightlvl = %d\n",
          dbleftlvl, dbrightlvl);

  /* Convert and truncate to 1 byte */

  dbleftlvl = ((-dbleftlvl) << 1) & 0xff;
  dbrightlvl = ((-dbrightlvl) << 1) & 0xff;

  /* Set the volume */

  if (priv->audio_mode == ES8388_MODULE_DAC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      es8388_writereg(priv, ES8388_DACCONTROL4, ES8388_LDACVOL(dbleftlvl));
      es8388_writereg(priv, ES8388_DACCONTROL5, ES8388_RDACVOL(dbrightlvl));
    }

  if (priv->audio_mode == ES8388_MODULE_ADC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      es8388_writereg(priv, ES8388_ADCCONTROL8, ES8388_LADCVOL(dbleftlvl));
      es8388_writereg(priv, ES8388_ADCCONTROL9, ES8388_RADCVOL(dbrightlvl));
    }

  /* Remember the volume level and mute settings */

  priv->volume = volume;
}
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

/****************************************************************************
 * Name: es8388_setmclkfrequency
 *
 * Description:
 *   Set the frequency of the I2S' Master Clock (MCLK).
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static void es8388_setmclkfrequency(FAR struct es8388_dev_s *priv)
{
  priv->mclk = 0;

  for (int i = 0; i < ARRAY_SIZE(es8388_mclk_rate); i++)
    {
      if (es8388_mclk_rate[i].sample_rate == priv->samprate)
        {
          /* Normally master clock should be multiple of the sample rate
           * and bclk at the same time. The field mclk_rate_s::multiple
           * means the multiple of mclk to the sample rate. If data width
           * is 24 bits, in order to keep mclk a multiple to the bclk,
           * mclk_rate_s::multiple should be a divisible by 3, otherwise
           * the ws signal will be inaccurate.
           */

          priv->mclk = es8388_mclk_rate[i].mclk;

          if (es8388_mclk_rate[i].multiple % (priv->bpsamp / 8) == 0)
            {
              break;
            }
        }
    }

  if (priv->mclk)
    {
      audinfo("MCLK Freq: %u\n", priv->mclk);

      int ret = I2S_MCLKFREQUENCY(priv->i2s, priv->mclk);

      if (ret < 0)
        {
          if (ret != -ENOTTY)
            {
              auderr("ERROR: Failed to set the MCLK on lower half\n");
            }
          else
            {
              priv->mclk = 0;
              auderr("WARNING: MCLK cannot be set on lower half\n");
            }
        }
    }
  else
    {
      auderr("ERROR: Unsupported combination of sample rate and"
                       " data width\n");
    }
}

/****************************************************************************
 * Name: es8388_setmute
 *
 * Description:
 *   Mute/unmute the ADC or DAC of the codec based on the current settings.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   enable - Boolean to enable or disable the mute function.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void es8388_setmute(FAR struct es8388_dev_s *priv, bool enable)
{
  uint8_t reg = 0;

  audinfo("Volume: mute=%d\n", (int)enable);

  priv->mute = enable;

  if (priv->audio_mode == ES8388_MODULE_DAC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      reg = es8388_readreg(priv, ES8388_DACCONTROL3) &
        (~ES8388_DACMUTE_BITMASK);
      es8388_writereg(priv, ES8388_DACCONTROL3,
                      reg | ES8388_DACMUTE(enable));
    }

  if (priv->audio_mode == ES8388_MODULE_ADC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      reg = es8388_readreg(priv, ES8388_ADCCONTROL7) &
        (~ES8388_ADCMUTE_BITMASK);
      es8388_writereg(priv, ES8388_ADCCONTROL7,
                      reg | ES8388_ADCMUTE(enable));
    }
}

/****************************************************************************
 * Name: es8388_setbitspersample
 *
 * Description:
 *   Set the number of bits per sample used by the I2S driver and the codec.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void es8388_setbitspersample(FAR struct es8388_dev_s *priv)
{
  uint8_t reg;
  uint8_t bit_config;

  DEBUGASSERT(priv && priv->lower);

  switch (priv->bpsamp)
    {
      case 16:
        bit_config = ES8388_WORD_LENGTH_16BITS;
        break;

      case 18:
        bit_config = ES8388_WORD_LENGTH_18BITS;
        break;

      case 20:
        bit_config = ES8388_WORD_LENGTH_20BITS;
        break;

      case 24:
        bit_config = ES8388_WORD_LENGTH_24BITS;
        break;

      case 32:
        bit_config = ES8388_WORD_LENGTH_32BITS;
        break;

      default:
        audwarn("ERROR: Data length not supported.\n");
        return;
    }

  I2S_TXDATAWIDTH(priv->i2s, priv->bpsamp);
  I2S_RXDATAWIDTH(priv->i2s, priv->bpsamp);

  if (priv->audio_mode == ES8388_MODULE_ADC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      reg = es8388_readreg(priv, ES8388_ADCCONTROL4) &
        (~ES8388_ADCWL_BITMASK);
      es8388_writereg(priv, ES8388_ADCCONTROL4,
                      reg | ES8388_ADCWL(bit_config));
    }

  if (priv->audio_mode == ES8388_MODULE_DAC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      reg = es8388_readreg(priv, ES8388_DACCONTROL1) &
        (~ES8388_DACWL_BITMASK);
      es8388_writereg(priv, ES8388_DACCONTROL1,
                      reg | ES8388_DACWL(bit_config));
    }

  audinfo("Datawidth set to %u\n", priv->bpsamp);
}

/****************************************************************************
 * Name: es8388_setsamplerate
 *
 * Description:
 *  Sets the sample frequency for the codec ADC/DAC and the I2S driver.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void es8388_setsamplerate(FAR struct es8388_dev_s *priv)
{
  DEBUGASSERT(priv && priv->lower);

  uint16_t regval;

  switch (priv->samprate)
    {
      case 8000:
        regval = ES8388_LCLK_DIV_1536;
        break;

      case 11025:
      case 12000:
        regval = ES8388_LCLK_DIV_1024;
        break;

      case 16000:
        regval = ES8388_LCLK_DIV_768;
        break;

      case 22050:
      case 24000:
        regval = ES8388_LCLK_DIV_512;
        break;

      case 32000:
        regval = ES8388_LCLK_DIV_384;
        break;

      case 44100:
      case 48000:
        regval = ES8388_LCLK_DIV_256;
        break;

      case 88200:
      case 96000:
        regval = ES8388_LCLK_DIV_128;
        break;

      default:
        audwarn("ERROR: Sample rate not supported.\n");
        return;
    }

  /* es8388_setmclkfrequency needs to be called before I2S_**SAMPLERATE */

  es8388_setmclkfrequency(priv);

  I2S_TXSAMPLERATE(priv->i2s, priv->samprate);
  I2S_RXSAMPLERATE(priv->i2s, priv->samprate);

  if (priv->audio_mode == ES8388_MODULE_ADC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      es8388_writereg(priv, ES8388_ADCCONTROL5, ES8388_ADCFSRATIO(regval));
    }

  if (priv->audio_mode == ES8388_MODULE_DAC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      es8388_writereg(priv, ES8388_DACCONTROL2, ES8388_DACFSRATIO(regval));
    }

  audinfo("Sample rate set to %d\n", priv->samprate);
}

/****************************************************************************
 * Name: es8388_getcaps
 *
 * Description:
 *   Get the audio device capabilities.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   type - The type of query.
 *   caps - A reference to an audio caps structure.
 *
 * Returned Value:
 *   Length of the caps structure.
 *
 ****************************************************************************/

static int es8388_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                           FAR struct audio_caps_s *caps)
{
  /* Validate the structure */

  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("getcaps: type=%d ac_type=%d\n", type, caps->ac_type);

  /* Fill in the caller's structure based on requested info */

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      /* Caller is querying for the types of units we support */

      case AUDIO_TYPE_QUERY:

        /* Provide our overall capabilities. The interfacing software
         * must then call us back for specific info for each capability.
         */

        caps->ac_channels = 2;       /* Stereo output */

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* We don't decode any formats! Only something above us in
               * the audio stream can perform decoding on our behalf.
               */

              /* The types of audio units we implement */

              caps->ac_controls.b[0] =
                AUDIO_TYPE_OUTPUT | AUDIO_TYPE_FEATURE |
                AUDIO_TYPE_PROCESSING;
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

              /* 8kHz is hardware dependent */

              caps->ac_controls.b[0] =
                AUDIO_SAMP_RATE_11K | AUDIO_SAMP_RATE_16K |
                AUDIO_SAMP_RATE_22K | AUDIO_SAMP_RATE_32K |
                AUDIO_SAMP_RATE_44K | AUDIO_SAMP_RATE_48K;
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

            caps->ac_controls.b[0] = AUDIO_FU_VOLUME;
            caps->ac_controls.b[1] = AUDIO_FU_BALANCE >> 8;
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
 * Name: es8388_configure
 *
 * Description:
 *   Configure the audio device for the specified mode of operation.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *   caps - A reference to an audio caps structure.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8388_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session,
                            FAR const struct audio_caps_s *caps)
#else
static int es8388_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct es8388_dev_s *priv = (FAR struct es8388_dev_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv != NULL && caps != NULL);
  audinfo("configure: ac_type: %d\n", caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
    case AUDIO_TYPE_FEATURE:
      audinfo("  AUDIO_TYPE_FEATURE\n");

      /* Process based on Feature Unit */

      switch (caps->ac_format.hw)
        {
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
        case AUDIO_FU_VOLUME:
          {
            /* Set the volume */

            uint16_t volume = caps->ac_controls.hw[0];
            audinfo("    Volume: %d\n", volume);

            if (volume >= 0 && volume <= 1000)
              {
                es8388_setvolume(priv, volume);
              }
            else
              {
                ret = -EDOM;
              }
          }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
        case AUDIO_FU_BALANCE:
          {
            /* Set the Balance */

            uint16_t balance = caps->ac_controls.hw[0];
            audinfo("    Balance: %d\n", balance);
            if (balance >= 0 && balance <= 1000)
              {
                priv->balance = balance;
                es8388_setvolume(priv, priv->volume);
              }
            else
              {
                ret = -EDOM;
              }
          }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

        default:
          auderr("    ERROR: Unrecognized feature unit\n");
          ret = -ENOTTY;
          break;
        }
        break;

    case AUDIO_TYPE_OUTPUT:
      {
        audinfo("  AUDIO_TYPE_OUTPUT:\n");
        audinfo("    Number of channels: %u\n", caps->ac_channels);
        audinfo("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
        audinfo("    Sample width:       %u\n", caps->ac_controls.b[2]);

        /* Verify that all of the requested values are supported */

        ret = -ERANGE;
        if (caps->ac_channels != 1 && caps->ac_channels != 2)
          {
            auderr("ERROR: Unsupported number of channels: %d\n",
                   caps->ac_channels);
            break;
          }

        if (caps->ac_controls.b[2] != 16 &&
            caps->ac_controls.b[2] != 18 &&
            caps->ac_controls.b[2] != 20 &&
            caps->ac_controls.b[2] != 24 &&
            caps->ac_controls.b[2] != 32)
          {
            auderr("ERROR: Unsupported bits per sample: %d\n",
                   caps->ac_controls.b[2]);
            break;
          }

        /* Save the current stream configuration */

        priv->samprate  = caps->ac_controls.hw[0];
        priv->nchannels = caps->ac_channels;
        priv->bpsamp    = caps->ac_controls.b[2];

        es8388_setsamplerate(priv);
        es8388_setbitspersample(priv);

        ret = OK;
      }
      break;

    case AUDIO_TYPE_PROCESSING:
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: es8388_shutdown
 *
 * Description:
 *   Shutdown the ES8388 chip and reset it.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *
 * Returned Value:
 *   Returns OK.
 *
 ****************************************************************************/

static int es8388_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct es8388_dev_s *priv = (FAR struct es8388_dev_s *)dev;

  DEBUGASSERT(priv);

  audinfo("Shutdown triggered\n");

  /* Now issue a software reset. This puts all ES8388 registers back in
   * their default state.
   */

  es8388_reset(priv);
  return OK;
}

/****************************************************************************
 * Name: es8388_senddone
 *
 * Description:
 *   This is the I2S callback function that is invoked when the transfer
 *   completes.
 *
 * Input Parameters:
 *   i2s - A reference to the I2S interface.
 *   apb - A reference to the audio pipeline buffer.
 *   arg - A void reference to the driver state structure.
 *   result - The result of the last transfer.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void  es8388_senddone(FAR struct i2s_dev_s *i2s,
                             FAR struct ap_buffer_s *apb, FAR void *arg,
                             int result)
{
  FAR struct es8388_dev_s *priv = (FAR struct es8388_dev_s *)arg;
  struct audio_msg_s msg;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(i2s && priv && priv->running && apb);
  audinfo("senddone: apb=%p inflight=%d result=%d\n",
          apb, priv->inflight, result);

  /* We do not place any restriction on the context in which this function
   * is called. It may be called from an interrupt handler. Therefore, the
   * doneq and in-flight values might be accessed from the interrupt level.
   * Not the best design. But we will use interrupt controls to protect
   * against that possibility.
   */

  flags = enter_critical_section();

  /* Add the completed buffer to the end of our doneq. We do not yet
   * decrement the reference count.
   */

  dq_addlast((FAR dq_entry_t *)apb, &priv->doneq);

  /* And decrement the number of buffers in-flight */

  DEBUGASSERT(priv->inflight > 0);
  priv->inflight--;

  /* Save the result of the transfer */

  priv->result = result;
  leave_critical_section(flags);

  /* Now send a message to the worker thread, informing it that there are
   * buffers in the done queue that need to be cleaned up.
   */

  msg.msg_id = AUDIO_MSG_COMPLETE;
  ret = file_mq_send(&priv->mq, (FAR const char *)&msg, sizeof(msg),
                     CONFIG_ES8388_MSG_PRIO);
  if (ret < 0)
    {
      auderr("ERROR: file_mq_send failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: es8388_returnbuffers
 *
 * Description:
 *   This function is called after the completion of one or more data
 *   transfers. This function will empty the done queue and release our
 *   reference to each buffer.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void es8388_returnbuffers(FAR struct es8388_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  /* The doneq and in-flight values might be accessed from the interrupt
   * level in some implementations. Not the best design. But we will
   * use interrupt controls to protect against that possibility.
   */

  flags = enter_critical_section();
  while (dq_peek(&priv->doneq) != NULL)
    {
      /* Take the next buffer from the queue of completed transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->doneq);
      leave_critical_section(flags);

      audinfo("Returning: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
              apb, apb->curbyte, apb->nbytes, apb->flags);

      /* Are we returning the final buffer in the stream? */

      if ((apb->flags & AUDIO_APB_FINAL) != 0)
        {
          /* Both the pending and the done queues should be empty and there
           * should be no buffers in-flight.
           */

          DEBUGASSERT(dq_empty(&priv->doneq) && dq_empty(&priv->pendq) &&
                      priv->inflight == 0);

          /* Set the terminating flag. This will, eventually, cause the
           * worker thread to exit (if it is not already terminating).
           */

          audinfo("Terminating\n");
          priv->terminating = true;
        }

      /* Release our reference to the audio buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
      flags = enter_critical_section();
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: es8388_sendbuffer
 *
 * Description:
 *   Start the transfer an audio buffer to the ES8388 via I2S. This
 *   will not wait for the transfer to complete but will return immediately.
 *   the es8388_senddone called will be invoked when the transfer
 *   completes, stimulating the worker thread to call this function again.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8388_sendbuffer(FAR struct es8388_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;
  uint32_t timeout;
  int shift;
  int ret;

  /* Loop while there are audio buffers to be sent and we have few than
   * CONFIG_ES8388_INFLIGHT then "in-flight"
   *
   * The 'inflight' value might be modified from the interrupt level in some
   * implementations. We will use interrupt controls to protect against
   * that possibility.
   *
   * The 'pendq', on the other hand, is protected via a mutex. Let's
   * hold the mutex while we are busy here and disable the interrupts
   * only while accessing 'inflight'.
   */

  ret = nxmutex_lock(&priv->pendlock);
  if (ret < 0)
    {
      return ret;
    }

  while (priv->inflight < CONFIG_ES8388_INFLIGHT &&
         dq_peek(&priv->pendq) != NULL && !priv->paused)
    {
      /* Take next buffer from the queue of pending transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq);
      audinfo("Sending apb=%p, size=%d inflight=%d\n",
              apb, apb->nbytes, priv->inflight);

      /* Increment the number of buffers in-flight before sending in order
       * to avoid a possible race condition.
       */

      flags = enter_critical_section();
      priv->inflight++;
      leave_critical_section(flags);

      /* Send the entire audio buffer via I2S. What is a reasonable timeout
       * to use? This would depend on the bit rate and size of the buffer.
       *
       * Samples in the buffer (samples):
       *   = buffer_size * 8 / bpsamp                           samples
       * Sample rate (samples/second):
       *   = samplerate * nchannels
       * Expected transfer time (seconds):
       *   = (buffer_size * 8) / bpsamp / samplerate / nchannels
       *
       * We will set the timeout about twice that.
       *
       * NOTES:
       * - The multiplier of 8 becomes 16000 for 2x and units of
       *   milliseconds.
       * - 16000 is a approximately 16384 (1 << 14).
       *   nchannels is either (1 << 0) or (1 << 1).
       *   So this can be simplifies to (milliseconds):
       *
       *   = (buffer_size << shift) / bpsamp / samplerate
       */

      shift = 14;

      shift -= (priv->nchannels > 1) ? 1 : 0;

      timeout = MSEC2TICK(((uint32_t)(apb->nbytes - apb->curbyte) << shift) /
                          (uint32_t)priv->samprate / (uint32_t)priv->bpsamp);

      ret = I2S_SEND(priv->i2s, apb, es8388_senddone, priv, timeout);
      if (ret < 0)
        {
          auderr("ERROR: I2S_SEND failed: %d\n", ret);
          break;
        }
    }

  nxmutex_unlock(&priv->pendlock);
  return ret;
}

/****************************************************************************
 * Name: es8388_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8388_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int es8388_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es8388_dev_s *priv = (FAR struct es8388_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  FAR void *value;
  int ret;
  uint8_t prev_regval = 0;
  uint8_t regval = 0;

  audinfo("ES8388 Start\n");

  prev_regval = es8388_readreg(priv, ES8388_DACCONTROL21);

  if (priv->audio_mode == ES8388_MODULE_LINE)
    {
      es8388_writereg(priv, ES8388_DACCONTROL16,
                      ES8388_RMIXSEL_RIN2 | ES8388_LMIXSEL_LIN2);

      es8388_writereg(priv, ES8388_DACCONTROL17,
                      ES8388_LI2LO_ENABLE  |
                      ES8388_LD2LO_DISABLE |
                      ES8388_LI2LOVOL(ES8388_MIXER_GAIN_0DB));

      es8388_writereg(priv, ES8388_DACCONTROL20,
                      ES8388_RI2RO_ENABLE  |
                      ES8388_RD2RO_DISABLE |
                      ES8388_RI2ROVOL(ES8388_MIXER_GAIN_0DB));

      es8388_writereg(priv, ES8388_DACCONTROL21,
                      ES8388_DAC_DLL_PWD_NORMAL |
                      ES8388_ADC_DLL_PWD_NORMAL |
                      ES8388_MCLK_DIS_NORMAL    |
                      ES8388_OFFSET_DIS_DISABLE |
                      ES8388_SLRCK_SAME         |
                      ES8388_LRCK_SEL_ADC);
    }
  else
    {
      es8388_writereg(priv, ES8388_DACCONTROL21,
                      ES8388_DAC_DLL_PWD_NORMAL |
                      ES8388_ADC_DLL_PWD_NORMAL |
                      ES8388_MCLK_DIS_NORMAL    |
                      ES8388_OFFSET_DIS_DISABLE |
                      ES8388_SLRCK_SAME         |
                      ES8388_LRCK_SEL_DAC);
    }

  regval = es8388_readreg(priv, ES8388_DACCONTROL21);

  if (regval != prev_regval)
    {
      es8388_writereg(priv, ES8388_CHIPPOWER,
                      ES8388_DACVREF_PDN_PWRUP |
                      ES8388_ADCVREF_PDN_PWRUP |
                      ES8388_DACDLL_PDN_NORMAL |
                      ES8388_ADCDLL_PDN_NORMAL |
                      ES8388_DAC_STM_RST_RESET |
                      ES8388_ADC_STM_RST_RESET |
                      ES8388_DAC_DIGPDN_RESET  |
                      ES8388_ADC_DIGPDN_RESET);

      es8388_writereg(priv, ES8388_CHIPPOWER,
                      ES8388_DACVREF_PDN_PWRUP  |
                      ES8388_ADCVREF_PDN_PWRUP  |
                      ES8388_DACDLL_PDN_NORMAL  |
                      ES8388_ADCDLL_PDN_NORMAL  |
                      ES8388_DAC_STM_RST_NORMAL |
                      ES8388_ADC_STM_RST_NORMAL |
                      ES8388_DAC_DIGPDN_NORMAL  |
                      ES8388_ADC_DIGPDN_NORMAL);
    }

  if (priv->audio_mode == ES8388_MODULE_LINE    ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC ||
      priv->audio_mode == ES8388_MODULE_ADC)
    {
      es8388_writereg(priv, ES8388_ADCPOWER,
                      ES8388_INT1LP_NORMAL        |
                      ES8388_FLASHLP_NORMAL       |
                      ES8388_PDNADCBIASGEN_NORMAL |
                      ES8388_PDNMICB_PWRON        |
                      ES8388_PDNADCR_PWRUP        |
                      ES8388_PDNADCL_PWRUP        |
                      ES8388_PDNAINR_NORMAL       |
                      ES8388_PDNAINL_NORMAL);
    }

  if (priv->audio_mode == ES8388_MODULE_LINE    ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC ||
      priv->audio_mode == ES8388_MODULE_DAC)
    {
      es8388_writereg(priv, ES8388_DACPOWER,
                      ES8388_ROUT2_ENABLE  |
                      ES8388_LOUT2_ENABLE  |
                      ES8388_ROUT1_ENABLE  |
                      ES8388_LOUT1_ENABLE  |
                      ES8388_PDNDACR_PWRUP |
                      ES8388_PDNDACL_PWRUP);

      es8388_setmute(priv, false);
    }

  /* Create a message queue for the worker thread */

  snprintf(priv->mqname, sizeof(priv->mqname), "/regconfig/%" PRIXPTR,
           (uintptr_t)priv);

  attr.mq_maxmsg  = 16;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  ret = file_mq_open(&priv->mq, priv->mqname,
                     O_RDWR | O_CREAT, 0644, &attr);
  if (ret < 0)
    {
      /* Error creating message queue! */

      auderr("ERROR: Couldn't allocate message queue\n");
      return ret;
    }

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      audinfo("Joining old thread\n");
      pthread_join(priv->threadid, &value);
    }

  /* Start our thread for sending data to the device */

  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  pthread_attr_setschedparam(&tattr, &sparam);
  pthread_attr_setstacksize(&tattr, CONFIG_ES8388_WORKER_STACKSIZE);

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, es8388_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("ERROR: pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "es8388");
      audinfo("Created worker thread\n");
    }

  return ret;
}

/****************************************************************************
 * Name: es8388_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8388_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int es8388_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es8388_dev_s *priv = (FAR struct es8388_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;

  audinfo("ES8388 Stop\n");

  if (priv->audio_mode == ES8388_MODULE_LINE)
    {
      es8388_writereg(priv, ES8388_DACCONTROL21,
                      ES8388_DAC_DLL_PWD_NORMAL |
                      ES8388_ADC_DLL_PWD_NORMAL |
                      ES8388_MCLK_DIS_NORMAL    |
                      ES8388_OFFSET_DIS_DISABLE |
                      ES8388_SLRCK_SAME         |
                      ES8388_LRCK_SEL_DAC);

      es8388_writereg(priv, ES8388_DACCONTROL16,
                      ES8388_RMIXSEL_RIN1 | ES8388_LMIXSEL_LIN1);

      es8388_writereg(priv, ES8388_DACCONTROL17,
                      ES8388_LD2LO_ENABLE  |
                      ES8388_LI2LO_DISABLE |
                      ES8388_LI2LOVOL(ES8388_MIXER_GAIN_0DB));

      es8388_writereg(priv, ES8388_DACCONTROL20,
                      ES8388_RD2RO_ENABLE  |
                      ES8388_RI2RO_DISABLE |
                      ES8388_RI2ROVOL(ES8388_MIXER_GAIN_0DB));

      goto stop_msg;
    }

  if (priv->audio_mode == ES8388_MODULE_DAC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      es8388_writereg(priv, ES8388_DACPOWER,
                      ES8388_ROUT2_DISABLE |
                      ES8388_LOUT2_DISABLE |
                      ES8388_ROUT1_DISABLE |
                      ES8388_LOUT1_DISABLE |
                      ES8388_PDNDACR_PWRUP |
                      ES8388_PDNDACL_PWRUP);

      es8388_setmute(priv, true);
    }

  if (priv->audio_mode == ES8388_MODULE_ADC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      es8388_writereg(priv, ES8388_ADCPOWER,
                      ES8388_INT1LP_LP        |
                      ES8388_FLASHLP_LP       |
                      ES8388_PDNADCBIASGEN_LP |
                      ES8388_PDNMICB_PWRDN    |
                      ES8388_PDNADCR_PWRDN    |
                      ES8388_PDNADCL_PWRDN    |
                      ES8388_PDNAINR_PWRDN    |
                      ES8388_PDNAINL_PWRDN);
    }

  if (priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      es8388_writereg(priv, ES8388_DACCONTROL21,
                      ES8388_DAC_DLL_PWD_PWRDN  |
                      ES8388_ADC_DLL_PWD_PWRDN  |
                      ES8388_MCLK_DIS_DISABLE   |
                      ES8388_OFFSET_DIS_DISABLE |
                      ES8388_LRCK_SEL_DAC       |
                      ES8388_SLRCK_SAME);
    }

stop_msg:

  /* Send a message to stop all audio streaming */

  term_msg.msg_id = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  file_mq_send(&priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
               CONFIG_ES8388_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  es8388_dump_registers(&priv->dev, "After stop");

  return OK;
}
#endif

/****************************************************************************
 * Name: es8388_pause
 *
 * Description: Pauses the playback.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8388_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int es8388_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es8388_dev_s *priv = (FAR struct es8388_dev_s *)dev;

  audinfo("ES8388 Pause\n");

  if (priv->running && !priv->paused)
    {
      priv->paused = true;
      es8388_setmute(priv, true);
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: es8388_resume
 *
 * Description: Resumes the playback.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8388_resume(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session)
#else
static int es8388_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es8388_dev_s *priv = (FAR struct es8388_dev_s *)dev;

  audinfo("ES8388 Resume\n");

  if (priv->running && priv->paused)
    {
      priv->paused = false;
      es8388_setmute(priv, false);
      es8388_sendbuffer(priv);
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: es8388_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   apb - A reference to the audio pipeline buffer.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8388_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  FAR struct es8388_dev_s *priv = (FAR struct es8388_dev_s *)dev;
  struct audio_msg_s term_msg;
  int ret;

  audinfo("Enqueueing: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
          apb, apb->curbyte, apb->nbytes, apb->flags);

  ret = nxmutex_lock(&priv->pendlock);
  if (ret < 0)
    {
      return ret;
    }

  /* Take a reference on the new audio buffer */

  apb_reference(apb);

  /* Add the new buffer to the tail of pending audio buffers */

  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;
  dq_addlast(&apb->dq_entry, &priv->pendq);
  nxmutex_unlock(&priv->pendlock);

  /* Send a message to the worker thread indicating that a new buffer has
   * been enqueued. If mq is NULL, then the playing has not yet started.
   * In that case we are just "priming the pump" and we don't need to send
   * any message.
   */

  ret = OK;
  if (priv->mq.f_inode != NULL)
    {
      term_msg.msg_id  = AUDIO_MSG_ENQUEUE;
      term_msg.u.data = 0;

      ret = file_mq_send(&priv->mq, (FAR const char *)&term_msg,
                         sizeof(term_msg), CONFIG_ES8388_MSG_PRIO);
      if (ret < 0)
        {
          auderr("ERROR: file_mq_send failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: es8388_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   apb - A reference to the audio pipeline buffer.
 *
 * Returned Value:
 *   Returns OK.
 *
 ****************************************************************************/

static int es8388_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                               FAR struct ap_buffer_s *apb)
{
  audinfo("Cancelled apb=%p\n", apb);
  return OK;
}

/****************************************************************************
 * Name: es8388_ioctl
 *
 * Description: Perform a device IOCTL.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   cmd - The IOCTL command.
 *   arg - The argument of the IOCTL command.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8388_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
  int ret = OK;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
#endif

  /* Deal with ioctls passed from the upper-half driver */

  switch (cmd)
    {
       /* Report our preferred buffer size and quantity */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:
        {
          audinfo("AUDIOIOC_GETBUFFERINFO:\n");
          bufinfo              = (FAR struct ap_buffer_info_s *) arg;
          bufinfo->buffer_size = CONFIG_ES8388_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_ES8388_NUM_BUFFERS;
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        audwarn("IOCTL not available\n");
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: es8388_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8388_reserve(FAR struct audio_lowerhalf_s *dev,
                          FAR void **session)
#else
static int es8388_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es8388_dev_s *priv = (FAR struct es8388_dev_s *) dev;
  int ret = OK;

  audinfo("ES8388 Reserve\n");

  /* Borrow the APBQ semaphore for thread sync */

  ret = nxmutex_lock(&priv->pendlock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->reserved)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Initialize the session context */

#ifdef CONFIG_AUDIO_MULTI_SESSION
     *session           = NULL;
#endif
      priv->inflight    = 0;
      priv->running     = false;
      priv->paused      = false;
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
      priv->terminating = false;
#endif
      priv->reserved    = true;
    }

  nxmutex_unlock(&priv->pendlock);
  return ret;
}

/****************************************************************************
 * Name: es8388_release
 *
 * Description: Releases the session (the only one we have).
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8388_release(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#else
static int es8388_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct es8388_dev_s *priv = (FAR struct es8388_dev_s *)dev;
  FAR void *value;
  int ret;

  audinfo("ES8388 Release\n");

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  /* Borrow the APBQ semaphore for thread sync */

  ret = nxmutex_lock(&priv->pendlock);

  /* Really we should free any queued buffers here */

  priv->reserved = false;
  nxmutex_unlock(&priv->pendlock);

  return ret;
}

/****************************************************************************
 * Name: es8388_audio_output
 *
 * Description:
 *   Initialize and configure the ES8388 device as an audio output device.
 *   This will be mostly used when adding input support.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   None. No failures are detected.
 *
 ****************************************************************************/

static void es8388_audio_output(FAR struct es8388_dev_s *priv)
{
  audinfo("ES8388 set to output mode\n");

  priv->audio_mode = ES8388_MODULE_DAC;
}

/****************************************************************************
 * Name: es8388_audio_input
 *
 * Description:
 *   Initialize and configure the ES8388 device as an audio input device.
 *   This will be mostly used when adding input support.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   None. No failures are detected.
 *
 ****************************************************************************/

#if 0

static void es8388_audio_input(FAR struct es8388_dev_s *priv)
{
  audinfo("ES8388 set to input mode\n");

  priv->audio_mode = ES8388_MODULE_ADC;
}

#endif

/****************************************************************************
 * Name: es8388_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 * Input Parameters:
 *   pvarg - The thread arguments.
 *
 * Returned Value:
 *   Returns NULL.
 *
 ****************************************************************************/

static void *es8388_workerthread(pthread_addr_t pvarg)
{
  FAR struct es8388_dev_s *priv = (struct es8388_dev_s *) pvarg;
  struct audio_msg_s msg;
  FAR struct ap_buffer_s *apb;
  int msglen;
  unsigned int prio;

  audinfo("ES8388 worker thread starting\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminating = false;
#endif

  priv->running = true;

  /* Loop as long as we are supposed to be running and as long as we have
   * buffers in-flight.
   */

  while (priv->running || priv->inflight > 0)
    {
      /* Check if we have been asked to terminate. We have to check if we
       * still have buffers in-flight. If we do, then we can't stop until
       * birds come back to roost.
       */

      if (priv->terminating && priv->inflight <= 0)
        {
          /* We are IDLE. Break out of the loop and exit. */

          break;
        }
      else
        {
          /* Check if we can send more audio buffers to the ES8388 */

          es8388_sendbuffer(priv);
        }

      /* Wait for messages from our message queue */

      msglen = file_mq_receive(&priv->mq, (FAR char *)&msg,
                               sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (msglen < sizeof(struct audio_msg_s))
        {
          auderr("ERROR: Message too small: %d\n", msglen);
          continue;
        }

      /* Process the message */

      switch (msg.msg_id)
        {
          /* The ISR has requested more data. We will catch this case at
           * the top of the loop.
           */

          case AUDIO_MSG_DATA_REQUEST:
            audinfo("AUDIO_MSG_DATA_REQUEST\n");
            break;

          /* Stop the playback */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
          case AUDIO_MSG_STOP:

            /* Indicate that we are terminating */

            audinfo("AUDIO_MSG_STOP: Terminating\n");
            priv->terminating = true;
            break;
#endif

          /* We have a new buffer to send. We will catch this case at
           * the top of the loop.
           */

          case AUDIO_MSG_ENQUEUE:
            audinfo("AUDIO_MSG_ENQUEUE\n");
            break;

          /* We will wake up from the I2S callback with this message */

          case AUDIO_MSG_COMPLETE:
            audinfo("AUDIO_MSG_COMPLETE\n");
            es8388_returnbuffers(priv);
            break;

          default:
            auderr("ERROR: Ignoring message ID %d\n", msg.msg_id);
            break;
        }
    }

  /* Reset the ES8388 hardware */

  es8388_reset(priv);

  /* Return any pending buffers in our pending queue */

  nxmutex_lock(&priv->pendlock);
  while ((apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq)) != NULL)
    {
      /* Release our reference to the buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
    }

  nxmutex_unlock(&priv->pendlock);

  /* Return any pending buffers in our done queue */

  es8388_returnbuffers(priv);

  /* Close the message queue */

  file_mq_close(&priv->mq);
  file_mq_unlink(priv->mqname);

  /* Send an AUDIO_MSG_COMPLETE message to the client */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  audinfo("ES8388 worker thread finishing\n");
  return NULL;
}

/****************************************************************************
 * Name: es8388_reset
 *
 * Description:
 *   Reset and re-initialize the ES8388.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void es8388_reset(FAR struct es8388_dev_s *priv)
{
  /* Put audio output back to its initial configuration */

  audinfo("ES8388 Reset\n");

  priv->dac_output = ES8388_DAC_OUTPUT_ALL;
  priv->adc_input  = ES8388_ADC_INPUT_ALL;
  priv->samprate   = ES8388_DEFAULT_SAMPRATE;
  priv->nchannels  = ES8388_DEFAULT_NCHANNELS;
  priv->bpsamp     = ES8388_DEFAULT_BPSAMP;
#if !defined(CONFIG_AUDIO_EXCLUDE_VOLUME) && !defined(CONFIG_AUDIO_EXCLUDE_BALANCE)
  priv->balance    = 500;            /* Center balance */
#endif

  /* Software reset. This puts all ES8388 registers back in their
   * default state.
   */

  uint8_t regconfig;

  es8388_audio_output(priv);

  es8388_writereg(priv, ES8388_DACCONTROL3,
                  ES8388_DACMUTE_MUTED       |
                  ES8388_DACLER_NORMAL       |
                  ES8388_DACSOFTRAMP_DISABLE |
                  ES8388_DACRAMPRATE_4LRCK);

  es8388_writereg(priv, ES8388_CONTROL2,
                  ES8388_PDNVREFBUF_NORMAL  |
                  ES8388_VREFLO_NORMAL      |
                  ES8388_PDNIBIASGEN_NORMAL |
                  ES8388_PDNANA_NORMAL      |
                  ES8388_LPVREFBUF_LP       |
                  ES8388_LPVCMMOD_NORMAL    |
                  (1 << 6)); /* Default value of undocumented bit */

  es8388_writereg(priv, ES8388_CHIPPOWER,
                  ES8388_DACVREF_PDN_SHIFT  |
                  ES8388_ADCVREF_PDN_PWRUP  |
                  ES8388_DACDLL_PDN_NORMAL  |
                  ES8388_ADCDLL_PDN_NORMAL  |
                  ES8388_DAC_STM_RST_NORMAL |
                  ES8388_ADC_STM_RST_NORMAL |
                  ES8388_DAC_DIGPDN_NORMAL  |
                  ES8388_ADC_DIGPDN_NORMAL);

  es8388_writereg(priv, ES8388_MASTERMODE,
                  ES8388_BCLKDIV(ES8388_MCLK_DIV_AUTO) |
                  ES8388_BCLK_INV_NORMAL               |
                  ES8388_MCLKDIV2_NODIV                |
                  ES8388_MSC_SLAVE);

  es8388_writereg(priv, ES8388_DACPOWER,
                  ES8388_ROUT2_DISABLE |
                  ES8388_LOUT2_DISABLE |
                  ES8388_ROUT1_DISABLE |
                  ES8388_LOUT1_DISABLE |
                  ES8388_PDNDACR_PWRDN |
                  ES8388_PDNDACL_PWRDN);

  es8388_writereg(priv, ES8388_DACCONTROL1,
                  ES8388_DACFORMAT(ES8388_I2S_NORMAL)     |
                  ES8388_DACWL(ES8388_WORD_LENGTH_16BITS) |
                  ES8388_DACLRP_NORM_2ND                  |
                  ES8388_DACLRSWAP_NORMAL);

  es8388_writereg(priv, ES8388_DACCONTROL2,
                  ES8388_DACFSRATIO(ES8388_LCLK_DIV_256) |
                  ES8388_DACFSMODE_SINGLE);

  es8388_writereg(priv, ES8388_DACCONTROL16,
                  ES8388_RMIXSEL_RIN1 | ES8388_LMIXSEL_LIN1);

  es8388_writereg(priv, ES8388_DACCONTROL17,
                  ES8388_LD2LO_ENABLE  |
                  ES8388_LI2LO_DISABLE |
                  ES8388_LI2LOVOL(ES8388_MIXER_GAIN_0DB));

  es8388_writereg(priv, ES8388_DACCONTROL20,
                  ES8388_RD2RO_ENABLE  |
                  ES8388_RI2RO_DISABLE |
                  ES8388_RI2ROVOL(ES8388_MIXER_GAIN_0DB));

  es8388_writereg(priv, ES8388_DACCONTROL21,
                  ES8388_DAC_DLL_PWD_NORMAL |
                  ES8388_ADC_DLL_PWD_NORMAL |
                  ES8388_MCLK_DIS_NORMAL    |
                  ES8388_OFFSET_DIS_DISABLE |
                  ES8388_SLRCK_SAME         |
                  ES8388_LRCK_SEL_DAC);

  es8388_writereg(priv, ES8388_DACCONTROL23, ES8388_VROI_1_5K);

  es8388_writereg(priv, ES8388_DACCONTROL24,
                  ES8388_LOUT1VOL(ES8388_DAC_CHVOL_DB(0)));

  es8388_writereg(priv, ES8388_DACCONTROL25,
                  ES8388_ROUT1VOL(ES8388_DAC_CHVOL_DB(0)));

  es8388_writereg(priv, ES8388_DACCONTROL26,
                  ES8388_LOUT2VOL(ES8388_DAC_CHVOL_DB(0)));

  es8388_writereg(priv, ES8388_DACCONTROL27,
                  ES8388_ROUT2VOL(ES8388_DAC_CHVOL_DB(0)));

  es8388_setmute(priv, true);

  regconfig = 0;

  if (priv->dac_output == ES8388_DAC_OUTPUT_LINE2)
    {
      regconfig = ES8388_DAC_CHANNEL_LOUT1 | ES8388_DAC_CHANNEL_ROUT1;
    }
  else if (priv->dac_output == ES8388_DAC_OUTPUT_LINE1)
    {
      regconfig = ES8388_DAC_CHANNEL_LOUT2 | ES8388_DAC_CHANNEL_ROUT2;
    }
  else
    {
      regconfig = ES8388_DAC_CHANNEL_LOUT1 | ES8388_DAC_CHANNEL_ROUT1 |
            ES8388_DAC_CHANNEL_LOUT2 | ES8388_DAC_CHANNEL_ROUT2;
    }

  es8388_writereg(priv, ES8388_DACPOWER, regconfig);

  es8388_writereg(priv, ES8388_ADCPOWER,
                  ES8388_INT1LP_LP        |
                  ES8388_FLASHLP_LP       |
                  ES8388_PDNADCBIASGEN_LP |
                  ES8388_PDNMICB_PWRDN    |
                  ES8388_PDNADCR_PWRDN    |
                  ES8388_PDNADCL_PWRDN    |
                  ES8388_PDNAINR_PWRDN    |
                  ES8388_PDNAINL_PWRDN);

  es8388_writereg(priv, ES8388_ADCCONTROL1,
                  ES8388_MICAMPR(ES8388_MIC_GAIN_0DB) |
                  ES8388_MICAMPL(ES8388_MIC_GAIN_0DB));

  regconfig = 0;

  if (priv->adc_input == ES8388_ADC_INPUT_LINE1)
    {
      regconfig = ES8388_ADC_CHANNEL_LINPUT1_RINPUT1;
    }
  else if (priv->adc_input == ES8388_ADC_INPUT_LINE2)
    {
      regconfig = ES8388_ADC_CHANNEL_LINPUT2_RINPUT2;
    }
  else
    {
      regconfig = ES8388_ADC_CHANNEL_DIFFERENCE;
    }

  es8388_writereg(priv, ES8388_ADCCONTROL2, regconfig);

  es8388_writereg(priv, ES8388_ADCCONTROL3,
                  ES8388_TRI_NORMAL         |
                  ES8388_MONOMIX_STEREO     |
                  ES8388_DS_LINPUT1_RINPUT1 |
                  (1 << 1)); /* Default value of undocumented bit */

  es8388_writereg(priv, ES8388_ADCCONTROL4,
                  ES8388_ADCFORMAT(ES8388_I2S_NORMAL)     |
                  ES8388_ADCWL(ES8388_WORD_LENGTH_16BITS) |
                  ES8388_ADCLRP_NORM_2ND                  |
                  ES8388_DATSEL_LL);

  es8388_writereg(priv, ES8388_ADCCONTROL5,
                  ES8388_ADCFSRATIO(ES8388_LCLK_DIV_256) |
                  ES8388_ADCFSMODE_SINGLE);

  es8388_writereg(priv, ES8388_ADCPOWER,
                  ES8388_INT1LP_LP            |
                  ES8388_FLASHLP_NORMAL       |
                  ES8388_PDNADCBIASGEN_NORMAL |
                  ES8388_PDNMICB_PWRDN        |
                  ES8388_PDNADCR_PWRUP        |
                  ES8388_PDNADCL_PWRUP        |
                  ES8388_PDNAINR_NORMAL       |
                  ES8388_PDNAINL_NORMAL);

  /* Stop sequence to avoid noise at boot */

  if (priv->audio_mode == ES8388_MODULE_LINE)
    {
      es8388_writereg(priv, ES8388_DACCONTROL21,
                      ES8388_DAC_DLL_PWD_NORMAL |
                      ES8388_ADC_DLL_PWD_NORMAL |
                      ES8388_MCLK_DIS_NORMAL    |
                      ES8388_OFFSET_DIS_DISABLE |
                      ES8388_SLRCK_SAME         |
                      ES8388_LRCK_SEL_DAC);

      es8388_writereg(priv, ES8388_DACCONTROL16,
                      ES8388_RMIXSEL_RIN1 | ES8388_LMIXSEL_LIN1);

      es8388_writereg(priv, ES8388_DACCONTROL17,
                      ES8388_LD2LO_ENABLE  |
                      ES8388_LI2LO_DISABLE |
                      ES8388_LI2LOVOL(ES8388_MIXER_GAIN_0DB));

      es8388_writereg(priv, ES8388_DACCONTROL20,
                      ES8388_RD2RO_ENABLE  |
                      ES8388_RI2RO_DISABLE |
                      ES8388_RI2ROVOL(ES8388_MIXER_GAIN_0DB));

      goto reset_finish;
    }

  if (priv->audio_mode == ES8388_MODULE_DAC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      es8388_writereg(priv, ES8388_DACPOWER,
                      ES8388_ROUT2_DISABLE |
                      ES8388_LOUT2_DISABLE |
                      ES8388_ROUT1_DISABLE |
                      ES8388_LOUT1_DISABLE |
                      ES8388_PDNDACR_PWRUP |
                      ES8388_PDNDACL_PWRUP);

      es8388_setmute(priv, true);
    }

  if (priv->audio_mode == ES8388_MODULE_ADC ||
      priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      es8388_writereg(priv, ES8388_ADCPOWER,
                      ES8388_INT1LP_LP        |
                      ES8388_FLASHLP_LP       |
                      ES8388_PDNADCBIASGEN_LP |
                      ES8388_PDNMICB_PWRDN    |
                      ES8388_PDNADCR_PWRDN    |
                      ES8388_PDNADCL_PWRDN    |
                      ES8388_PDNAINR_PWRDN    |
                      ES8388_PDNAINL_PWRDN);
    }

  if (priv->audio_mode == ES8388_MODULE_ADC_DAC)
    {
      es8388_writereg(priv, ES8388_DACCONTROL21,
                      ES8388_DAC_DLL_PWD_PWRDN  |
                      ES8388_ADC_DLL_PWD_PWRDN  |
                      ES8388_MCLK_DIS_DISABLE   |
                      ES8388_OFFSET_DIS_DISABLE |
                      ES8388_LRCK_SEL_DAC       |
                      ES8388_SLRCK_SAME);
    }

reset_finish:

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
  es8388_setvolume(priv, CONFIG_ES8388_OUTPUT_INITVOLUME);
#endif

  es8388_dump_registers(&priv->dev, "After reset");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: es8388_initialize
 *
 * Description:
 *   Initialize the ES8388 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance.
 *   i2s     - An I2S driver instance.
 *   lower   - Persistent board configuration data.
 *
 * Returned Value:
 *   A new lower half audio interface for the ES8388 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
  es8388_initialize(FAR struct i2c_master_s *i2c,
                    FAR struct i2s_dev_s *i2s,
                    FAR const struct es8388_lower_s *lower)
{
  FAR struct es8388_dev_s *priv;

  audinfo("Initializing ES8388\n");

  /* Sanity check */

  DEBUGASSERT(i2c && i2s && lower);

  /* Allocate a ES8388 device structure */

  priv = (FAR struct es8388_dev_s *)kmm_zalloc(sizeof(struct es8388_dev_s));

  if (priv)
    {
      priv->dev.ops    = &g_audioops;
      priv->lower      = lower;
      priv->i2c        = i2c;
      priv->i2s        = i2s;

      nxmutex_init(&priv->pendlock);
      dq_init(&priv->pendq);
      dq_init(&priv->doneq);

      audinfo("ES8388: address=%02x frequency=%" PRId32 "\n",
              lower->address, lower->frequency);

      /* Reset and reconfigure the ES8388 hardware */

      es8388_dump_registers(&priv->dev, "Before reset");

      es8388_reset(priv);
      return &priv->dev;
    }

  nxmutex_destroy(&priv->pendlock);
  kmm_free(priv);
  return NULL;
}
