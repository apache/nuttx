/****************************************************************************
 * drivers/audio/es8311.c
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

#include <sys/ioctl.h>
#include <sys/param.h>
#include <sys/types.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <fixedmath.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/es8311.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>

#include "es8311.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if !defined(CONFIG_ES8311_REGDUMP)
static
#endif /* !CONFIG_ES8311_REGDUMP */
uint8_t     es8311_readreg(FAR struct es8311_dev_s *priv, uint8_t regaddr);
static int  es8311_writereg(FAR struct es8311_dev_s *priv,
                            uint8_t regaddr,
                            uint16_t regval);
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static int  es8311_setvolume(FAR struct es8311_dev_s *priv,
                             es_module_e module,
                             uint16_t volume);
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */
static int  es8311_getcoeff(FAR struct es8311_dev_s *priv,
                            uint32_t samplerate);
#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
static int  es8311_setmute(FAR struct es8311_dev_s *priv,
                           es_module_e module,
                           bool enable);
#endif /* CONFIG_AUDIO_EXCLUDE_MUTE */
static int  es8311_setmicgain(FAR struct es8311_dev_s *priv, uint32_t gain);
static int  es8311_setbitspersample(FAR struct es8311_dev_s *priv);
static int  es8311_setsamplerate(FAR struct es8311_dev_s *priv);
static int  es8311_getcaps(FAR struct audio_lowerhalf_s *dev,
                           int type,
                           FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8311_configure(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session,
                             FAR const struct audio_caps_s *caps);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_configure(FAR struct audio_lowerhalf_s *dev,
                             FAR const struct audio_caps_s *caps);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_shutdown(FAR struct audio_lowerhalf_s *dev);
static void es8311_processdone(FAR struct i2s_dev_s *i2s,
                                  FAR struct ap_buffer_s *apb,
                                  FAR void *arg,
                                  int result);
static void es8311_returnbuffers(FAR struct es8311_dev_s *priv);
static int  es8311_processbegin(FAR struct es8311_dev_s *priv);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8311_start(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_start(FAR struct audio_lowerhalf_s *dev);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8311_stop(FAR struct audio_lowerhalf_s *dev,
                        FAR void *session);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_stop(FAR struct audio_lowerhalf_s *dev);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8311_pause(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session);
static int  es8311_resume(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_pause(FAR struct audio_lowerhalf_s *dev);
static int  es8311_resume(FAR struct audio_lowerhalf_s *dev);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */
static int  es8311_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb);
static int  es8311_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb);
static int  es8311_ioctl(FAR struct audio_lowerhalf_s *dev,
                         int cmd,
                         unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8311_reserve(FAR struct audio_lowerhalf_s *dev,
                          FAR void **session);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_reserve(FAR struct audio_lowerhalf_s *dev);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  es8311_release(FAR struct audio_lowerhalf_s *dev,
                           FAR void *session);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int  es8311_release(FAR struct audio_lowerhalf_s *dev);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
static void *es8311_workerthread(pthread_addr_t pvarg);
static void es8311_audio_output(FAR struct es8311_dev_s *priv);
static void es8311_audio_input(FAR struct es8311_dev_s *priv);
static int  es8311_reset(FAR struct es8311_dev_s *priv);
static int  es8311_get_mclk_src(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops =
{
  es8311_getcaps,       /* getcaps        */
  es8311_configure,     /* configure      */
  es8311_shutdown,      /* shutdown       */
  es8311_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  es8311_stop,          /* stop           */
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  es8311_pause,         /* pause          */
  es8311_resume,        /* resume         */
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */
  NULL,                 /* allocbuffer    */
  NULL,                 /* freebuffer     */
  es8311_enqueuebuffer, /* enqueue_buffer */
  es8311_cancelbuffer,  /* cancel_buffer  */
  es8311_ioctl,         /* ioctl          */
  NULL,                 /* read           */
  NULL,                 /* write          */
  es8311_reserve,       /* reserve        */
  es8311_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: es8311_get_mclk_src
 *
 * Description:
 *    Get the MCLK source.
 *
 * Returned Value:
 *   ES8311_MCLK_FROM_MCLK_PIN or ES8311_MCLK_FROM_SCLK_PIN.
 *
 ****************************************************************************/

static int es8311_get_mclk_src(void)
{
#ifdef CONFIG_ES8311_SRC_MCLK
  return ES8311_MCLK_FROM_MCLK_PIN;
#else
  return ES8311_MCLK_FROM_SCLK_PIN;
#endif
}

/****************************************************************************
 * Name: es8311_readreg
 *
 * Description:
 *   Read the specified 8-bit register from the ES8311 device through I2C.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   regaddr - Address of the register to be read.
 *
 * Returned Value:
 *   On success, the byte stored in the register.
 *   On failure, 0.
 *
 ****************************************************************************/

#if !defined(CONFIG_ES8311_REGDUMP)
static
#endif /* !CONFIG_ES8311_REGDUMP */
uint8_t es8311_readreg(FAR struct es8311_dev_s *priv, uint8_t regaddr)
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
              auderr("I2C_RESET failed: %d\n", ret);
              break;
            }
#else /* !CONFIG_I2C_RESET */
          auderr("I2C_TRANSFER failed: %d\n", ret);
#endif /* !CONFIG_I2C_RESET */
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
 * Name: es8311_writereg
 *
 * Description:
 *   Write the specified 8-bit register to the ES8311 device through I2C.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   regaddr - Address of the register to be written.
 *   regval - Value to be written.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_writereg(FAR struct es8311_dev_s *priv,
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
              auderr("I2C_RESET failed: %d\n", ret);
              break;
            }
#else /* !CONFIG_I2C_RESET */
          auderr("I2C_TRANSFER failed: %d\n", ret);
#endif /* !CONFIG_I2C_RESET */
        }
      else
        {
          /* The I2C transfer was successful... break out of the loop and
           * return.
           */

          audinfo("Write: %02x <- %02x\n", regaddr, regval);
          return OK;
        }

      audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }

  return -EIO;
}

/****************************************************************************
 * Name: es8311_setmicgain
 *
 * Description:
 *   Set the microphone gain.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   gain - The microphone gain to be set in the codec (0..42).
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_setmicgain(FAR struct es8311_dev_s *priv, uint32_t gain)
{
  int ret;
  static const es8311_mic_gain_e gain_map[] =
  {
    ES8311_MIC_GAIN_0DB,
    ES8311_MIC_GAIN_6DB,
    ES8311_MIC_GAIN_12DB,
    ES8311_MIC_GAIN_18DB,
    ES8311_MIC_GAIN_24DB,
    ES8311_MIC_GAIN_30DB,
    ES8311_MIC_GAIN_36DB,
    ES8311_MIC_GAIN_42DB
  };

  /* Our range is from 0 to 42 dB with steps of 6 dB. We map this to the
   * ES8311 gain values.
   */

  priv->mic_gain = gain_map[MIN(gain, 42) / 6];

  ret = es8311_writereg(priv, ES8311_ADC_REG16, priv->mic_gain);

  if (ret < 0)
    {
      auderr("Failed to set mic gain: %d\n", ret);
    }
  else
    {
      audinfo("Set mic gain: %d\n", priv->mic_gain);
    }

  return ret;
}

/****************************************************************************
 * Name: es8311_setvolume
 *
 * Description:
 *   Set the right and left volume values in the ES8311 device based on the
 *   desired volume settings.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure;
 *   module - The module to change the volume for;
 *   volume - The volume to be set in the codec (0..1000).
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static int es8311_setvolume(FAR struct es8311_dev_s *priv,
                            es_module_e module,
                            uint16_t volume)
{
  /* Convert from (0..1000) to (0..200) ln curve to avoid distortion */

  uint16_t dblvl = (int16_t) (volume ? (29 * logf(volume)) : 0);
  int ret = 0;

  /* Set the volume */

  if (module == ES_MODULE_DAC || module == ES_MODULE_ADC_DAC)
    {
      ret |= es8311_writereg(priv, ES8311_DAC_REG32, dblvl);
      priv->volume_out = volume;
    }

  if (module == ES_MODULE_ADC || module == ES_MODULE_ADC_DAC)
    {
      ret |= es8311_writereg(priv, ES8311_ADC_REG17, dblvl);
      priv->volume_in = volume;
    }

  if (ret < 0)
    {
      auderr("Failed to set volume.\n");
      return -EIO;
    }
  else
    {
      audinfo("Volume set to %d\n", volume);
      return OK;
    }
}
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

/****************************************************************************
 * Name: es8311_getcoeff
 *
 * Description:
 *   Get the coefficient values for the current MCLK and given sample rate.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   samplerate - The sample rate to be set in the codec.
 *
 * Returned Value:
 *   The index of the divider in the es8311_coeff_div array or -EINVAL if
 *   the combination of MCLK and sample rate is not supported.
 *
 ****************************************************************************/

static int es8311_getcoeff(FAR struct es8311_dev_s *priv,
                           uint32_t samplerate)
{
  int i;

  for (i = 0; i < nitems(es8311_coeff_div); i++)
    {
      if (priv->mclk == es8311_coeff_div[i].mclk &&
          samplerate == es8311_coeff_div[i].rate)
        {
          audinfo("MCLK: %d, samplerate: %d\n", priv->mclk, samplerate);
          return i;
        }
    }

  auderr("MCLK = %d and samplerate = %d not supported.\n",
         priv->mclk, samplerate);

  return -EINVAL;
}

/****************************************************************************
 * Name: es8311_setmute
 *
 * Description:
 *   Mute/unmute the ADC or DAC of the codec based on the current settings.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *   enable - Boolean to enable or disable the mute function.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
static int es8311_setmute(FAR struct es8311_dev_s *priv,
                          es_module_e module,
                          bool enable)
{
  uint8_t reg = 0;
  int ret = 0;

  audinfo("module=%d, mute=%d\n", module, (int)enable);

  priv->mute = enable;

  if (module == ES_MODULE_DAC || module == ES_MODULE_ADC_DAC)
    {
      reg = es8311_readreg(priv, ES8311_DAC_REG31) & 0x9f;
      if (enable)
        {
          ret |= es8311_writereg(priv, ES8311_SYSTEM_REG12, 0x02);
          ret |= es8311_writereg(priv, ES8311_DAC_REG31,    reg | 0x60);
          ret |= es8311_writereg(priv, ES8311_DAC_REG32,    0x00);
          ret |= es8311_writereg(priv, ES8311_DAC_REG37,    0x08);
        }
      else
        {
          ret |= es8311_writereg(priv, ES8311_DAC_REG31,    reg);
          ret |= es8311_writereg(priv, ES8311_SYSTEM_REG12, 0x00);
        }
    }

  if (module == ES_MODULE_ADC || module == ES_MODULE_ADC_DAC)
    {
      reg = es8311_readreg(priv, ES8311_SDPOUT_REG0A) & 0xbf;
      ret |= es8311_writereg(priv, ES8311_SDPOUT_REG0A,
                             reg | ((int) enable << 6));
    }

  if (ret < 0)
    {
      auderr("Failed to set mute.\n");
      return -EIO;
    }
  else
    {
      audinfo("Set mute to: %d\n", (int)enable);
      return OK;
    }
}
#endif

/****************************************************************************
 * Name: es8311_setbitspersample
 *
 * Description:
 *   Set the number of bits per sample used by the I2S driver and the codec.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_setbitspersample(FAR struct es8311_dev_s *priv)
{
  uint8_t reg;
  uint8_t bit_config;
  int ret;

  DEBUGASSERT(priv && priv->lower);

  switch (priv->bpsamp)
    {
      case 16:
        bit_config = ES_WORD_LENGTH_16BITS;
        break;

      case 24:
        bit_config = ES_WORD_LENGTH_24BITS;
        break;

      case 32:
        bit_config = ES_WORD_LENGTH_32BITS;
        break;

      default:
        audwarn("Data length not supported.\n");
        return -EINVAL;
    }

  ret = I2S_RXDATAWIDTH(priv->i2s, priv->bpsamp);
  if (ret < 0)
    {
      auderr("I2S_RXDATAWIDTH failed.\n");
      return ret;
    }

  ret = I2S_TXDATAWIDTH(priv->i2s, priv->bpsamp);
  if (ret < 0)
    {
      auderr("I2S_TXDATAWIDTH failed.\n");
      return ret;
    }

  ret = 0;
  if (priv->audio_mode == ES_MODULE_ADC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      reg = es8311_readreg(priv, ES8311_SDPOUT_REG0A) & 0xe3;
      ret |= es8311_writereg(priv, ES8311_SDPOUT_REG0A,
                             reg | (bit_config << 2));
    }

  if (priv->audio_mode == ES_MODULE_DAC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      reg = es8311_readreg(priv, ES8311_SDPIN_REG09) & 0xe3;
      ret |= es8311_writereg(priv, ES8311_SDPIN_REG09,
                             reg | (bit_config << 2));
    }

  if (ret < 0)
    {
      auderr("Failed to set bits per sample\n");
      return -EIO;
    }
  else
    {
      audinfo("Bits per sample set to: %d\n", priv->bpsamp);
      return OK;
    }
}

/****************************************************************************
 * Name: es8311_setsamplerate
 *
 * Description:
 *  Sets the sample frequency for the codec ADC/DAC and the I2S driver.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_setsamplerate(FAR struct es8311_dev_s *priv)
{
  DEBUGASSERT(priv && priv->lower);

  uint8_t regconfig;
  uint8_t datmp;
  int coeff_index;
  int ret;

  priv->mclk = I2S_GETMCLKFREQUENCY(priv->i2s);
  coeff_index = es8311_getcoeff(priv, priv->samprate);

  if (coeff_index < 0)
    {
      auderr("Failed to set sample rate: %d\n", -EINVAL);
      return -EINVAL;
    }

  ret = I2S_RXSAMPLERATE(priv->i2s, priv->samprate);
  if (ret < 0)
    {
      auderr("I2S_RXSAMPLERATE failed.\n");
      return ret;
    }

  ret = I2S_TXSAMPLERATE(priv->i2s, priv->samprate);
  if (ret < 0)
    {
      auderr("I2S_TXSAMPLERATE failed.\n");
      return ret;
    }

  ret = 0;
  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG02) & 0x07;
  regconfig |= (es8311_coeff_div[coeff_index].pre_div - 1) << 5;

  datmp = 0;
  switch (es8311_coeff_div[coeff_index].pre_multi)
    {
      case 1:
        datmp = 0;
        break;
      case 2:
        datmp = 1;
        break;
      case 4:
        datmp = 2;
        break;
      case 8:
        datmp = 3;
        break;
      default:
        break;
    }

  if (es8311_get_mclk_src() == ES8311_MCLK_FROM_SCLK_PIN)
    {
      datmp = 3;     /* DIG_MCLK = LRCK * 256 = BCLK * 8 */
    }

  regconfig |= (datmp) << 3;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG02, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG05) & 0x00;
  regconfig |= (es8311_coeff_div[coeff_index].adc_div - 1) << 4;
  regconfig |= (es8311_coeff_div[coeff_index].dac_div - 1) << 0;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG05, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG03) & 0x80;
  regconfig |= es8311_coeff_div[coeff_index].fs_mode << 6;
  regconfig |= es8311_coeff_div[coeff_index].adc_osr << 0;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG03, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG04) & 0x80;
  regconfig |= es8311_coeff_div[coeff_index].dac_osr << 0;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG04, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG07) & 0xc0;
  regconfig |= es8311_coeff_div[coeff_index].lrck_h << 0;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG07, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG08) & 0x00;
  regconfig |= es8311_coeff_div[coeff_index].lrck_l << 0;
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG08, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG06) & 0xe0;
  if (es8311_coeff_div[coeff_index].bclk_div < 19)
    {
      regconfig |= (es8311_coeff_div[coeff_index].bclk_div - 1) << 0;
    }
  else
    {
      regconfig |= (es8311_coeff_div[coeff_index].bclk_div) << 0;
    }

  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG06, regconfig);

  if (ret < 0)
    {
      auderr("Failed to set sample rate.\n");
      return -EIO;
    }
  else
    {
      audinfo("Sample rate set to: %d.\n", priv->samprate);
      return OK;
    }
}

/****************************************************************************
 * Name: es8311_getcaps
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

static int es8311_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
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

        caps->ac_channels = 1;       /* Mono output */

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* The types of audio units we implement */

              caps->ac_controls.b[0] = AUDIO_TYPE_INPUT |
                                       AUDIO_TYPE_OUTPUT |
                                       AUDIO_TYPE_FEATURE;
              break;

            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }

        break;

      /* Provide capabilities of our OUTPUT unit */

      case AUDIO_TYPE_OUTPUT:

        caps->ac_channels = 1;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report the Sample rates we support */

              /* 8kHz is hardware dependent */

              caps->ac_controls.b[0] =
                AUDIO_SAMP_RATE_11K | AUDIO_SAMP_RATE_16K |
                AUDIO_SAMP_RATE_22K | AUDIO_SAMP_RATE_32K |
                AUDIO_SAMP_RATE_44K | AUDIO_SAMP_RATE_48K;
              caps->ac_controls.b[1] = 0;
              break;

            default:
              break;
          }

        break;

      case AUDIO_TYPE_INPUT:

        caps->ac_channels = 1;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report supported input sample rates */

              caps->ac_controls.b[0] =
                AUDIO_SAMP_RATE_11K | AUDIO_SAMP_RATE_16K |
                AUDIO_SAMP_RATE_22K | AUDIO_SAMP_RATE_32K |
                AUDIO_SAMP_RATE_44K | AUDIO_SAMP_RATE_48K;
              caps->ac_controls.b[1] = 0;
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

            caps->ac_controls.b[0] = AUDIO_FU_VOLUME | AUDIO_FU_MUTE;
            caps->ac_controls.b[1] = (AUDIO_FU_INP_GAIN) >> 8;
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
 * Name: es8311_configure
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
static int es8311_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session,
                            FAR const struct audio_caps_s *caps)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR const struct audio_caps_s *caps)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
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
                es8311_setvolume(priv, priv->audio_mode, volume);
              }
            else
              {
                ret = -EDOM;
              }
          }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
        case AUDIO_FU_MUTE:
          {
            /* Mute/Unmute */

            bool mute = (bool)caps->ac_controls.hw[0];
            audinfo("    Mute: %d\n", mute);

            es8311_setmute(priv, ES_MODULE_DAC, mute);
          }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_MUTE */

        case AUDIO_FU_INP_GAIN:
          {
            /* Set the mic gain */

            uint32_t mic_gain = caps->ac_controls.hw[0];
            audinfo("    Mic gain: %" PRIu32 "\n", mic_gain);

            es8311_setmicgain(priv, mic_gain);
          }
          break;

        default:
          auderr("    Unrecognized feature unit\n");
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

        /* The codec can take stereo audio and play only one channel */

        if (caps->ac_channels != 1 && caps->ac_channels != 2)
          {
            auderr("Unsupported number of channels: %d\n",
                   caps->ac_channels);
            break;
          }

        if (caps->ac_controls.b[2] != 16 &&
            caps->ac_controls.b[2] != 24 &&
            caps->ac_controls.b[2] != 32)
          {
            auderr("Unsupported bits per sample: %d\n",
                   caps->ac_controls.b[2]);
            break;
          }

        /* Save the current stream configuration */

        priv->samprate  = caps->ac_controls.hw[0];
        priv->bpsamp    = caps->ac_controls.b[2];

        es8311_audio_output(priv);
        ret |= es8311_reset(priv);
        es8311_setsamplerate(priv);
        es8311_setbitspersample(priv);

        ret = OK;
      }
      break;

        case AUDIO_TYPE_INPUT:
      {
        audinfo("  AUDIO_TYPE_INPUT:\n");
        audinfo("    Number of channels: %u\n", caps->ac_channels);
        audinfo("    Sample rate:        %u\n", caps->ac_controls.hw[0]);
        audinfo("    Sample width:       %u\n", caps->ac_controls.b[2]);

        /* Verify that all of the requested values are supported */

        ret = -ERANGE;

        /* The codec can take stereo audio and play only one channel */

        if (caps->ac_channels != 1 && caps->ac_channels != 2)
          {
            auderr("Unsupported number of channels: %d\n",
                   caps->ac_channels);
            break;
          }

        if (caps->ac_controls.b[2] != 16 &&
            caps->ac_controls.b[2] != 24 &&
            caps->ac_controls.b[2] != 32)
          {
            auderr("Unsupported bits per sample: %d\n",
                   caps->ac_controls.b[2]);
            break;
          }

        /* Save the current stream configuration */

        priv->samprate  = caps->ac_controls.hw[0];
        priv->bpsamp    = caps->ac_controls.b[2];

        es8311_audio_input(priv);
        es8311_reset(priv);
        es8311_setsamplerate(priv);
        es8311_setbitspersample(priv);

        ret = OK;
      }
      break;

    case AUDIO_TYPE_PROCESSING:
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: es8311_shutdown
 *
 * Description:
 *   Shutdown the ES8311 chip and reset it.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *
 * Returned Value:
 *   Returns OK.
 *
 ****************************************************************************/

static int es8311_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;

  DEBUGASSERT(priv);

  audinfo("Shutdown triggered\n");

  /* Now issue a software reset. This puts all ES8311 registers back in
   * their default state.
   */

  es8311_reset(priv);
  return OK;
}

/****************************************************************************
 * Name: es8311_processdone
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

static void es8311_processdone(FAR struct i2s_dev_s *i2s,
                               FAR struct ap_buffer_s *apb,
                               FAR void *arg,
                               int result)
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)arg;
  struct audio_msg_s msg;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(i2s && priv && priv->running && apb);
  audinfo("Transfer done: apb=%p inflight=%d result=%d\n",
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
                     CONFIG_ES8311_MSG_PRIO);
  if (ret < 0)
    {
      auderr("file_mq_send failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: es8311_returnbuffers
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

static void es8311_returnbuffers(FAR struct es8311_dev_s *priv)
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
#else /* !CONFIG_AUDIO_MULTI_SESSION */
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
      flags = enter_critical_section();
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: es8311_processbegin
 *
 * Description:
 *   Start the transfer an audio buffer to the ES8311 via I2S. This
 *   will not wait for the transfer to complete but will return immediately.
 *   the es8311_processdone called will be invoked when the transfer
 *   completes, stimulating the worker thread to call this function again.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_processbegin(FAR struct es8311_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;
  uint32_t timeout;
  int ret;

  /* Loop while there are audio buffers to be sent and we have few than
   * CONFIG_ES8311_INFLIGHT then "in-flight"
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

  while (priv->inflight < CONFIG_ES8311_INFLIGHT &&
         dq_peek(&priv->pendq) != NULL && !priv->paused)
    {
      /* Take next buffer from the queue of pending transfers */

      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq);
      audinfo("Transferring apb=%p, size=%d inflight=%d\n",
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
       *   = buffer_size * 8 / bpsamp
       * Sample rate (samples/second):
       *   = samplerate * nchannels
       * Expected transfer time (seconds):
       *   = (buffer_size * 8) / bpsamp / samplerate
       *
       * We will set the timeout about twice that.
       *
       * NOTES:
       * - The multiplier of 8 becomes 16000 for 2x and units of
       *   milliseconds.
       * - 16000 is a approximately 16384 (1 << 14).
       *   So this can be simplifies to (milliseconds):
       *
       *   = (buffer_size << 14) / bpsamp / samplerate
       */

      timeout = MSEC2TICK(((uint32_t)(apb->nbytes - apb->curbyte) << 14) /
                          (uint32_t)priv->samprate / (uint32_t)priv->bpsamp);

      if (priv->audio_mode == ES_MODULE_DAC)
        {
          ret = I2S_SEND(priv->i2s, apb, es8311_processdone,
                         priv, timeout);
        }
      else
        {
          ret = I2S_RECEIVE(priv->i2s, apb, es8311_processdone,
                            priv, timeout);
        }

      if (ret < 0)
        {
          auderr("I2S transfer failed: %d\n", ret);
          break;
        }
    }

  nxmutex_unlock(&priv->pendlock);
  return ret;
}

/****************************************************************************
 * Name: es8311_start
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
static int es8311_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_start(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  FAR void *value;
  int ret = OK;
  uint8_t dac_iface;
  uint8_t adc_iface;

  audinfo("ES8311 Start\n");

  if (priv->audio_mode == ES_MODULE_LINE)
    {
      auderr("The codec ES8311 doesn't support ES_MODULE_LINE mode");
      return -EINVAL;
    }

  if (priv->audio_mode == ES_MODULE_ADC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      adc_iface = es8311_readreg(priv, ES8311_SDPOUT_REG0A) & 0xbf;
      ret |= es8311_writereg(priv, ES8311_SDPOUT_REG0A, adc_iface);
    }

  if (priv->audio_mode == ES_MODULE_DAC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      dac_iface = es8311_readreg(priv, ES8311_SDPIN_REG09) & 0xbf;
      ret |= es8311_writereg(priv, ES8311_SDPIN_REG09,  dac_iface);
    }

  ret |= es8311_writereg(priv, ES8311_ADC_REG17,    0xbf);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG0E, 0x02);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG12, 0x00);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG14, 0x1a);

  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG0D, 0x01);
  ret |= es8311_writereg(priv, ES8311_ADC_REG15,    0x40);
  ret |= es8311_writereg(priv, ES8311_DAC_REG37,    0x48);
  ret |= es8311_writereg(priv, ES8311_GP_REG45,     0x00);

  /* Set internal reference signal (ADCL + DACR) */

  ret |= es8311_writereg(priv, ES8311_GPIO_REG44, 0x50);

  if (ret < 0)
    {
      auderr("Failed to start operation.\n");
      return -EIO;
    }
  else
    {
      audinfo("Operation has been started.\n");
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

      auderr("Couldn't allocate message queue\n");
      return ret;
    }

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      audinfo("Joining old thread\n");
      pthread_join(priv->threadid, &value);
    }

  /* Start our thread for processing device data */

  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  pthread_attr_setschedparam(&tattr, &sparam);
  pthread_attr_setstacksize(&tattr, CONFIG_ES8311_WORKER_STACKSIZE);

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, es8311_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "es8311");
      audinfo("Created worker thread\n");
    }

  return ret;
}

/****************************************************************************
 * Name: es8311_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8311_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_stop(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;
  int ret = 0;
  uint8_t dac_iface;
  uint8_t adc_iface;

  audinfo("ES8311 Stop\n");

  if (priv->audio_mode == ES_MODULE_ADC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      adc_iface = es8311_readreg(priv, ES8311_SDPOUT_REG0A) | 0x40;
      ret |= es8311_writereg(priv, ES8311_SDPOUT_REG0A, adc_iface);
    }

  if (priv->audio_mode == ES_MODULE_DAC ||
      priv->audio_mode == ES_MODULE_ADC_DAC)
    {
      dac_iface = es8311_readreg(priv, ES8311_SDPIN_REG09) | 0x40;
      ret |= es8311_writereg(priv, ES8311_SDPIN_REG09,  dac_iface);
    }

  ret |= es8311_writereg(priv, ES8311_DAC_REG32,    0x00);
  ret |= es8311_writereg(priv, ES8311_ADC_REG17,    0x00);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG0E, 0xff);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG12, 0x02);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG14, 0x00);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG0D, 0xfa);
  ret |= es8311_writereg(priv, ES8311_ADC_REG15,    0x00);
  ret |= es8311_writereg(priv, ES8311_DAC_REG37,    0x08);
  ret |= es8311_writereg(priv, ES8311_GP_REG45,     0x01);

  /* Send a message to stop all audio streaming */

  term_msg.msg_id = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  file_mq_send(&priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
               CONFIG_ES8311_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  es8311_dump_registers(&priv->dev, "After stop");

  if (ret < 0)
    {
      auderr("Failed to stop operation.\n");
    }
  else
    {
      audinfo("Operation has been stopped.\n");
    }

  return ret;
}
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

/****************************************************************************
 * Name: es8311_pause
 *
 * Description: Pauses the playback.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8311_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_pause(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  int ret = OK;

  audinfo("ES8311 Pause\n");

  if (priv->running && !priv->paused)
    {
      priv->paused = true;
#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
      ret = es8311_setmute(priv, priv->audio_mode, true);
#endif
    }

  if (ret < 0)
    {
      auderr("Failed to pause operation.\n");
    }
  else
    {
      audinfo("Operation has been paused.\n");
    }

  return ret;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: es8311_resume
 *
 * Description: Resumes the playback.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   session - The current audio session.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int es8311_resume(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_resume(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  int ret = OK;

  audinfo("ES8311 Resume\n");

  if (priv->running && priv->paused)
    {
      priv->paused = false;
#ifndef CONFIG_AUDIO_EXCLUDE_MUTE
      ret = es8311_setmute(priv, priv->audio_mode, false);
#endif /* CONFIG_AUDIO_EXCLUDE_MUTE */
      es8311_processbegin(priv);
    }

  if (ret < 0)
    {
      auderr("Failed to resume operation.\n");
    }
  else
    {
      audinfo("Operation has been resumed.\n");
    }

  return ret;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: es8311_enqueuebuffer
 *
 * Description:
 *   Enqueue an Audio Pipeline Buffer for processing.
 *
 * Input Parameters:
 *   dev - A reference to the lower half state structure.
 *   apb - A reference to the audio pipeline buffer.
 *
 * Returned Value:
 *   Returns OK or a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
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
                         sizeof(term_msg), CONFIG_ES8311_MSG_PRIO);
      if (ret < 0)
        {
          auderr("file_mq_send failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: es8311_cancelbuffer
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

static int es8311_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                               FAR struct ap_buffer_s *apb)
{
  audinfo("Cancelled apb=%p\n", apb);
  return OK;
}

/****************************************************************************
 * Name: es8311_ioctl
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

static int es8311_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
  int ret = OK;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
#endif /* CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS */

  /* Deal with ioctls passed from the upper-half driver */

  switch (cmd)
    {
      /* Report our preferred buffer size and quantity */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:
        {
          audinfo("AUDIOIOC_GETBUFFERINFO:\n");
          bufinfo              = (FAR struct ap_buffer_info_s *) arg;
          bufinfo->buffer_size = CONFIG_ES8311_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_ES8311_NUM_BUFFERS;
        }
        break;
#endif /* CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS */

      default:
        ret = -ENOTTY;
        audinfo("Unhandled ioctl: %d\n", cmd);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: es8311_reserve
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
static int es8311_reserve(FAR struct audio_lowerhalf_s *dev,
                          FAR void **session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_reserve(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *) dev;
  int ret = OK;

  audinfo("ES8311 Reserve\n");

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
#endif /* CONFIG_AUDIO_MULTI_SESSION */
      priv->inflight    = 0;
      priv->running     = false;
      priv->paused      = false;
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
      priv->terminating = false;
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */
      priv->reserved    = true;
    }

  nxmutex_unlock(&priv->pendlock);
  return ret;
}

/****************************************************************************
 * Name: es8311_release
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
static int es8311_release(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#else /* !CONFIG_AUDIO_MULTI_SESSION */
static int es8311_release(FAR struct audio_lowerhalf_s *dev)
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
{
  FAR struct es8311_dev_s *priv = (FAR struct es8311_dev_s *)dev;
  FAR void *value;
  int ret;

  audinfo("ES8311 Release\n");

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
 * Name: es8311_audio_output
 *
 * Description:
 *   Initialize and configure the ES8311 device as an audio output device.
 *   This will be mostly used when adding input support.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   None. No failures are detected.
 *
 ****************************************************************************/

static void es8311_audio_output(FAR struct es8311_dev_s *priv)
{
  audinfo("ES8311 set to output mode\n");

  priv->audio_mode = ES_MODULE_DAC;
}

/****************************************************************************
 * Name: es8311_audio_input
 *
 * Description:
 *   Initialize and configure the ES8311 device as an audio input device.
 *   This will be mostly used when adding input support.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   None. No failures are detected.
 *
 ****************************************************************************/

static void es8311_audio_input(FAR struct es8311_dev_s *priv)
{
  audinfo("ES8311 set to input mode\n");

  priv->audio_mode = ES_MODULE_ADC;
}

/****************************************************************************
 * Name: es8311_workerthread
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

static void *es8311_workerthread(pthread_addr_t pvarg)
{
  FAR struct es8311_dev_s *priv = (struct es8311_dev_s *) pvarg;
  struct audio_msg_s msg;
  FAR struct ap_buffer_s *apb;
  int msglen;
  unsigned int prio;

  audinfo("ES8311 worker thread starting\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminating = false;
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

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
          /* Check if we can process more audio buffers */

          es8311_processbegin(priv);
        }

      /* Wait for messages from our message queue */

      msglen = file_mq_receive(&priv->mq, (FAR char *)&msg,
                               sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (msglen < sizeof(struct audio_msg_s))
        {
          auderr("Message too small: %d\n", msglen);
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
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

          /* We have a new buffer to process. We will catch this case at
           * the top of the loop.
           */

          case AUDIO_MSG_ENQUEUE:
            audinfo("AUDIO_MSG_ENQUEUE\n");
            break;

          /* We will wake up from the I2S callback with this message */

          case AUDIO_MSG_COMPLETE:
            audinfo("AUDIO_MSG_COMPLETE\n");
            es8311_returnbuffers(priv);
            break;

          default:
            auderr("Ignoring message ID %d\n", msg.msg_id);
            break;
        }
    }

  /* Reset the ES8311 hardware */

  es8311_reset(priv);

  /* Return any pending buffers in our pending queue */

  nxmutex_lock(&priv->pendlock);
  while ((apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq)) != NULL)
    {
      /* Release our reference to the buffer */

      apb_free(apb);

      /* Send the buffer back up to the previous level. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
      priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */
    }

  nxmutex_unlock(&priv->pendlock);

  /* Return any pending buffers in our done queue */

  es8311_returnbuffers(priv);

  /* Close the message queue */

  file_mq_close(&priv->mq);
  file_mq_unlink(priv->mqname);

  /* Send an AUDIO_MSG_COMPLETE message to the client */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else /* !CONFIG_AUDIO_MULTI_SESSION */
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif /* !CONFIG_AUDIO_MULTI_SESSION */

  audinfo("ES8311 worker thread finishing\n");
  return NULL;
}

/****************************************************************************
 * Name: es8311_reset
 *
 * Description:
 *   Reset and re-initialize the ES8311.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int es8311_reset(FAR struct es8311_dev_s *priv)
{
  /* Put audio back to its initial configuration */

  audinfo("ES8311 reset triggered.\n");
  priv->samprate   = ES8311_DEFAULT_SAMPRATE;
  priv->bpsamp     = ES8311_DEFAULT_BPSAMP;

  /* Software reset. This puts all ES8311 registers back in their
   * default state.
   */

  uint8_t regconfig;
  int ret = 0;

  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG01, 0x30);
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG02, 0x00);
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG03, 0x10);
  ret |= es8311_writereg(priv, ES8311_ADC_REG16,         0x24);
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG04, 0x10);
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG05, 0x00);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG0B,      0x00);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG0C,      0x00);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG10,      0x1f);
  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG11,      0x7f);
  ret |= es8311_writereg(priv, ES8311_RESET_REG00,       0x80);

  regconfig = es8311_readreg(priv, ES8311_RESET_REG00);
  regconfig &= 0xbf;
  ret |= es8311_writereg(priv, ES8311_RESET_REG00, regconfig);
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG01, 0x3f);

  if (es8311_get_mclk_src() == ES8311_MCLK_FROM_MCLK_PIN)
    {
      regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG01);
      regconfig &= 0x7f;
      ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG01, regconfig);
    }
  else
    {
      regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG01);
      regconfig |= 0x80;
      ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG01, regconfig);
    }

  ret |= es8311_setsamplerate(priv);
  ret |= es8311_setbitspersample(priv);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG01);
  regconfig &= ~(0x40);
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG01, regconfig);

  regconfig = es8311_readreg(priv, ES8311_CLK_MANAGER_REG06);
  regconfig &= ~(0x20);
  ret |= es8311_writereg(priv, ES8311_CLK_MANAGER_REG06, regconfig);

  ret |= es8311_writereg(priv, ES8311_SYSTEM_REG13, 0x10);
  ret |= es8311_writereg(priv, ES8311_ADC_REG1B, 0x0a);
  ret |= es8311_writereg(priv, ES8311_ADC_REG1C, 0x6a);

  ret |= es8311_setvolume(priv, ES_MODULE_ADC,
                          CONFIG_ES8311_INPUT_INITVOLUME);
  ret |= es8311_setvolume(priv, ES_MODULE_DAC,
                          CONFIG_ES8311_OUTPUT_INITVOLUME);

  es8311_dump_registers(&priv->dev, "After reset");

  if (ret < 0)
    {
      auderr("Failed to reset the ES8311.\n");
      return -EIO;
    }
  else
    {
      audinfo("ES8311 reset complete.\n");
      return OK;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: es8311_initialize
 *
 * Description:
 *   Initialize the ES8311 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance.
 *   i2s     - An I2S driver instance.
 *   lower   - Persistent board configuration data.
 *
 * Returned Value:
 *   A new lower half audio interface for the ES8311 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
  es8311_initialize(FAR struct i2c_master_s *i2c,
                    FAR struct i2s_dev_s *i2s,
                    FAR const struct es8311_lower_s *lower)
{
  FAR struct es8311_dev_s *priv;

  audinfo("Initializing ES8311\n");

  /* Sanity check */

  DEBUGASSERT(i2c && i2s && lower);

  /* Allocate a ES8311 device structure */

  priv = (FAR struct es8311_dev_s *)kmm_zalloc(sizeof(struct es8311_dev_s));

  if (priv)
    {
      priv->dev.ops    = &g_audioops;
      priv->lower      = lower;
      priv->i2c        = i2c;
      priv->i2s        = i2s;

      nxmutex_init(&priv->pendlock);
      dq_init(&priv->pendq);
      dq_init(&priv->doneq);

      audinfo("ES8311: address=%02x frequency=%" PRId32 "\n",
              lower->address, lower->frequency);

      /* Reset and reconfigure the ES8311 hardware */

      es8311_dump_registers(&priv->dev, "Before reset");

      es8311_audio_output(priv);
      es8311_reset(priv);
      audinfo("ES8311 initialized.\n");
      return &priv->dev;
    }

  return NULL;
}
