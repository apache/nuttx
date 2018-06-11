/****************************************************************************
 * drivers/audio/cs43l22.c
 * Audio device driver for Cirrus logic CS43L22 Audio codec.
 *
 *   Copyright (C) 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Taras Drozdovskiy <t.drozdovskiy@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <fixedmath.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/cs43l22.h>
#include <nuttx/lib/math.h>

#include "cs43l22.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if !defined(CONFIG_CS43L22_REGDUMP) && !defined(CONFIG_CS43L22_CLKDEBUG)
static
#endif
uint8_t cs43l22_readreg(FAR struct cs43l22_dev_s *priv, uint8_t regaddr);
static void cs43l22_writereg(FAR struct cs43l22_dev_s *priv, uint8_t regaddr,
                             uint8_t regval);
static void cs43l22_takesem(sem_t * sem);
#define     cs43l22_givesem(s) nxsem_post(s)

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static inline uint16_t cs43l22_scalevolume(uint16_t volume, b16_t scale);
static void cs43l22_setvolume(FAR struct cs43l22_dev_s *priv, uint16_t volume,
                              bool mute);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static void cs43l22_setbass(FAR struct cs43l22_dev_s *priv, uint8_t bass);
static void cs43l22_settreble(FAR struct cs43l22_dev_s *priv, uint8_t treble);
#endif

static void cs43l22_setdatawidth(FAR struct cs43l22_dev_s *priv);
static void cs43l22_setbitrate(FAR struct cs43l22_dev_s *priv);

/* Audio lower half methods (and close friends) */

static int  cs43l22_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                            FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs43l22_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session,
                              FAR const struct audio_caps_s *caps);
#else
static int  cs43l22_configure(FAR struct audio_lowerhalf_s *dev,
                              FAR const struct audio_caps_s *caps);
#endif
static int  cs43l22_shutdown(FAR struct audio_lowerhalf_s *dev);
static void cs43l22_senddone(FAR struct i2s_dev_s *i2s,
                             FAR struct ap_buffer_s *apb, FAR void *arg,
                             int result);
static void cs43l22_returnbuffers(FAR struct cs43l22_dev_s *priv);
static int  cs43l22_sendbuffer(FAR struct cs43l22_dev_s *priv);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs43l22_start(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  cs43l22_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs43l22_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  cs43l22_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs43l22_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session);
static int  cs43l22_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int  cs43l22_pause(FAR struct audio_lowerhalf_s *dev);
static int  cs43l22_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int  cs43l22_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                  FAR struct ap_buffer_s *apb);
static int  cs43l22_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb);
static int  cs43l22_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                          unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs43l22_reserve(FAR struct audio_lowerhalf_s *dev,
                           FAR void **session);
#else
static int  cs43l22_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  cs43l22_release(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session);
#else
static int  cs43l22_release(FAR struct audio_lowerhalf_s *dev);
#endif

/* Interrupt handling an worker thread */

#ifdef CS43L22_USE_FFLOCK_INT
static void cs43l22_interrupt_work(FAR void *arg);
static int  cs43l22_interrupt(FAR const struct cs43l22_lower_s *lower,
                             FAR void *arg);
#endif

static void *cs43l22_workerthread(pthread_addr_t pvarg);

/* Initialization */

static void cs43l22_audio_output(FAR struct cs43l22_dev_s *priv);
#if 0 /* Not used */
static void cs43l22_audio_input(FAR struct cs43l22_dev_s *priv);
#endif
#ifdef CS43L22_USE_FFLOCK_INT
static void cs43l22_configure_ints(FAR struct cs43l22_dev_s *priv);
#else
#  define   cs43l22_configure_ints(p)
#endif
static void cs43l22_reset(FAR struct cs43l22_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops =
{
  cs43l22_getcaps,       /* getcaps */
  cs43l22_configure,     /* configure */
  cs43l22_shutdown,      /* shutdown */
  cs43l22_start,         /* start */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  cs43l22_stop,          /* stop */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  cs43l22_pause,         /* pause */
  cs43l22_resume,        /* resume */
#endif
  NULL,                  /* allocbuffer */
  NULL,                  /* freebuffer */
  cs43l22_enqueuebuffer, /* enqueue_buffer */
  cs43l22_cancelbuffer,  /* cancel_buffer */
  cs43l22_ioctl,         /* ioctl */
  NULL,                  /* read */
  NULL,                  /* write */
  cs43l22_reserve,       /* reserve */
  cs43l22_release        /* release */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs43l22_readreg
 *
 * Description:
 *    Read the specified 16-bit register from the CS43L22 device.
 *
 ****************************************************************************/

#if !defined(CONFIG_CS43L22_REGDUMP) && !defined(CONFIG_CS43L22_CLKDEBUG)
static
#endif
uint8_t cs43l22_readreg(FAR struct cs43l22_dev_s *priv, uint8_t regaddr)
{
  int retries;

  /* Try up to three times to read the register */

  for (retries = 1; retries <= 3; retries++)
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

      /* Read the register data.  The returned value is the number messages
       * completed.
       */

      ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up?  Try to shake the bus free */

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

/************************************************************************************
 * Name: cs43l22_writereg
 *
 * Description:
 *   Write the specified 16-bit register to the CS43L22 device.
 *
 ************************************************************************************/

static void
cs43l22_writereg(FAR struct cs43l22_dev_s *priv, uint8_t regaddr,
                 uint8_t regval)
{
  struct i2c_config_s config;
  int retries;

  /* Setup up the I2C configuration */

  config.frequency = priv->lower->frequency;
  config.address   = priv->lower->address;
  config.addrlen   = 7;

  /* Try up to three times to read the register */

  for (retries = 1; retries <= 3; retries++)
    {
      uint8_t data[2];
      int ret;

      /* Set up the data to write */

      data[0] = regaddr;
      data[1] = regval;

      /* Read the register data.  The returned value is the number messages
       * completed.
       */

      ret = i2c_write(priv->i2c, &config, data, 2);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up?  Try to shake the bus free */

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
           * return the value read.
           */

          audinfo("Write: %02x <- %02x\n", regaddr, regval);
          return;
        }

      audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }
}

/************************************************************************************
 * Name: cs43l22_takesem
 *
 * Description:
 *  Take a semaphore count, handling the nasty EINTR return if we are interrupted
 *  by a signal.
 *
 ************************************************************************************/

static void cs43l22_takesem(sem_t * sem)
{
  int ret;

  do
    {
      ret = nxsem_wait(sem);
      DEBUGASSERT(ret == 0 || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/************************************************************************************
 * Name: cs43l22_scalevolume
 *
 * Description:
 *   Set the right and left volume values in the CS43L22 device based on the current
 *   volume and balance settings.
 *
 ************************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static inline uint16_t cs43l22_scalevolume(uint16_t volume, b16_t scale)
{
  return b16toi((b16_t) volume * scale);
}
#endif

/************************************************************************************
 * Name: cs43l22_setvolume
 *
 * Description:
 *   Set the right and left volume values in the CS43L22 device based on the current
 *   volume and balance settings.
 *
 ************************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static void
cs43l22_setvolume(FAR struct cs43l22_dev_s *priv, uint16_t volume, bool mute)
{
  uint32_t leftlevel;
  uint32_t rightlevel;
  uint8_t regval;

  audinfo("volume=%u mute=%u\n", volume, mute);

#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  /* Calculate the left channel volume level {0..1000} */

  if (priv->balance <= 500)
    {
      leftlevel = volume;
    }
  else if (priv->balance == 1000)
    {
      leftlevel = 0;
    }
  else
    {
      leftlevel = ((((1000 - priv->balance) * 100) / 500) * volume) / 100;
    }

/* Calculate the right channel volume level {0..1000} */

  if (priv->balance >= 500)
    {
      rightlevel = volume;
    }
  else if (priv->balance == 0)
    {
      rightlevel = 0;
    }
  else
    {
      rightlevel = (((priv->balance * 100) / 500) * volume) / 100;
    }

#  else
  leftlevel = priv->volume;
  rightlevel = priv->volume;
#  endif

  /* Set the volume */

   regval = (rightlevel + 0x19) & 0xff;
   cs43l22_writereg(priv, CS43L22_MS_VOL_CTRL_A, regval);
   regval = ((leftlevel + 0x19) & 0xff);
   cs43l22_writereg(priv, CS43L22_MS_VOL_CTRL_B, regval);

#if 0
  regval = (rightlevel + 0x01) & 0xff;
  cs43l22_writereg(priv, CS43L22_HP_VOL_CTRL_A, regval);
  regval = (leftlevel + 0x01) & 0xff;
  cs43l22_writereg(priv, CS43L22_HP_VOL_CTRL_B, regval);
#endif

  regval = cs43l22_readreg(priv, CS43L22_PLAYBACK_CTRL2);

  if (mute)
    {
      regval |= (CS43L22_HPAMUTE | CS43L22_HPBMUTE);
    }
  else
    {
      regval &= ~(CS43L22_HPAMUTE | CS43L22_HPBMUTE);
    }

  cs43l22_writereg(priv, CS43L22_PLAYBACK_CTRL2, regval);

  /* Remember the volume level and mute settings */

  priv->volume = volume;
  priv->mute   = mute;
}
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

/************************************************************************************
 * Name: cs43l22_setbass
 *
 * Description:
 *   Set the bass level.
 *
 *   The level and range are in whole percentage levels (0-100).
 *
 ************************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static void cs43l22_setbass(FAR struct cs43l22_dev_s *priv, uint8_t bass)
{
  audinfo("bass=%u\n", bass);
#warning Missing logic
}
#endif /* CONFIG_AUDIO_EXCLUDE_TONE */

/************************************************************************************
 * Name: cs43l22_settreble
 *
 * Description:
 *   Set the treble level .
 *
 *   The level and range are in whole percentage levels (0-100).
 *
 ************************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static void cs43l22_settreble(FAR struct cs43l22_dev_s *priv, uint8_t treble)
{
  audinfo("treble=%u\n", treble);
#warning Missing logic
}
#endif /* CONFIG_AUDIO_EXCLUDE_TONE */

/****************************************************************************
 * Name: cs43l22_setdatawidth
 *
 * Description:
 *   Set the 8- or 16-bit data modes
 *
 ****************************************************************************/

static void cs43l22_setdatawidth(FAR struct cs43l22_dev_s *priv)
{
  if (priv->bpsamp == 16)
    {
      /* Reset default default setting */
      priv->i2s->ops->i2s_txdatawidth(priv->i2s, 16);
    }
  else
    {
      /* This should select 8-bit with no companding */
      priv->i2s->ops->i2s_txdatawidth(priv->i2s, 8);
    }
}

/****************************************************************************
 * Name: cs43l22_setbitrate
 *
 ****************************************************************************/

static void cs43l22_setbitrate(FAR struct cs43l22_dev_s *priv)
{
  DEBUGASSERT(priv && priv->lower);

  priv->i2s->ops->i2s_txsamplerate(priv->i2s, priv->samprate);

  audinfo("sample rate=%u nchannels=%u bpsamp=%u\n",
          priv->samprate, priv->nchannels, priv->bpsamp);
}

/****************************************************************************
 * Name: cs43l22_getcaps
 *
 * Description:
 *   Get the audio device capabilities
 *
 ****************************************************************************/

static int cs43l22_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
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

              caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT | AUDIO_TYPE_FEATURE |
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

              caps->ac_controls.b[0] = AUDIO_SAMP_RATE_8K | AUDIO_SAMP_RATE_11K |
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

        /* If the sub-type is UNDEF, then report the Feature Units we support */

        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            /* Fill in the ac_controls section with the Feature Units we have */

            caps->ac_controls.b[0] = AUDIO_FU_VOLUME | AUDIO_FU_BASS | AUDIO_FU_TREBLE;
            caps->ac_controls.b[1] = AUDIO_FU_BALANCE >> 8;
          }
        else
          {
            /* TODO:  Do we need to provide specific info for the Feature Units,
             * such as volume setting ranges, etc.?
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

              caps->ac_controls.b[0] = AUDIO_STEXT_ENABLE | AUDIO_STEXT_WIDTH;
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
 * Name: cs43l22_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
cs43l22_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session, FAR const struct audio_caps_s *caps)
#else
static int
cs43l22_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct cs43l22_dev_s *priv = (FAR struct cs43l22_dev_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv != NULL && caps != NULL);
  audinfo("ac_type: %d\n", caps->ac_type);

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
                /* Scale the volume setting to the range {76..255} */

                cs43l22_setvolume(priv, (179 * volume / 1000) + 76, priv->mute);
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
                /* Scale the volume setting to the range {76..255} */

                cs43l22_setvolume(priv, (179 * priv->volume / 1000) + 76,
                                  priv->mute);
              }
            else
              {
                ret = -EDOM;
              }
           }
          break;
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
        case AUDIO_FU_BASS:
          {
            /* Set the bass.  The percentage level (0-100) is in the
             * ac_controls.b[0] parameter.
             */

            uint8_t bass = caps->ac_controls.b[0];
            audinfo("    Bass: %d\n", bass);

            if (bass <= 100)
              {
                cs43l22_setbass(priv, bass);
              }
            else
              {
                ret = -EDOM;
              }
          }
          break;

        case AUDIO_FU_TREBLE:
          {
            /* Set the treble.  The percentage level (0-100) is in the
             * ac_controls.b[0] parameter.
             */

            uint8_t treble = caps->ac_controls.b[0];
            audinfo("    Treble: %d\n", treble);

            if (treble <= 100)
              {
                cs43l22_settreble(priv, treble);
              }
            else
              {
                ret = -EDOM;
              }
          }
          break;
#endif  /* CONFIG_AUDIO_EXCLUDE_TONE */

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

        if (caps->ac_controls.b[2] != 8 && caps->ac_controls.b[2] != 16)
          {
            auderr("ERROR: Unsupported bits per sample: %d\n",
                   caps->ac_controls.b[2]);
            break;
          }

        /* Save the current stream configuration */

        priv->samprate  = caps->ac_controls.hw[0];
        priv->nchannels = caps->ac_channels;
        priv->bpsamp    = caps->ac_controls.b[2];

        /* Reconfigure the FLL to support the resulting number or channels,
         * bits per sample, and bitrate.
         */

        cs43l22_setdatawidth(priv);
        cs43l22_setbitrate(priv);
        cs43l22_clock_analysis(&priv->dev, "AUDIO_TYPE_OUTPUT");
        ret = OK;
      }
      break;

    case AUDIO_TYPE_PROCESSING:
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: cs43l22_shutdown
 *
 * Description:
 *   Shutdown the CS43L22 chip and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int cs43l22_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct cs43l22_dev_s *priv = (FAR struct cs43l22_dev_s *)dev;

  DEBUGASSERT(priv);

  /* First disable interrupts */

  CS43L22_DISABLE(priv->lower);

  /* Now issue a software reset. This puts all CS43L22 registers back in
   * their default state.
   */

  cs43l22_reset(priv);
  return OK;
}

/****************************************************************************
 * Name: cs43l22_senddone
 *
 * Description:
 *   This is the I2S callback function that is invoked when the transfer
 *   completes.
 *
 ****************************************************************************/

static void
cs43l22_senddone(FAR struct i2s_dev_s *i2s,
                 FAR struct ap_buffer_s *apb, FAR void *arg, int result)
{
  FAR struct cs43l22_dev_s *priv = (FAR struct cs43l22_dev_s *)arg;
  struct audio_msg_s msg;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(i2s && priv && priv->running && apb);
  audinfo("apb=%p inflight=%d result=%d\n", apb, priv->inflight, result);

  /* We do not place any restriction on the context in which this function
   * is called.  It may be called from an interrupt handler.  Therefore, the
   * doneq and in-flight values might be accessed from the interrupt level.
   * Not the best design.  But we will use interrupt controls to protect
   * against that possibility.
   */

  flags = enter_critical_section();

  /* Add the completed buffer to the end of our doneq.  We do not yet
   * decrement the reference count.
   */

  dq_addlast((FAR dq_entry_t *)apb, &priv->doneq);

  /* And decrement the number of buffers in-flight */

  DEBUGASSERT(priv->inflight > 0);
  priv->inflight--;

  /* Save the result of the transfer */
  /* REVISIT:  This can be overwritten */

  priv->result = result;
  leave_critical_section(flags);

  /* Now send a message to the worker thread, informing it that there are
   * buffers in the done queue that need to be cleaned up.
   */

  msg.msgId = AUDIO_MSG_COMPLETE;
  ret = nxmq_send(priv->mq, (FAR const char *)&msg, sizeof(msg),
                  CONFIG_CS43L22_MSG_PRIO);
  if (ret < 0)
    {
      auderr("ERROR: nxmq_send failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: cs43l22_returnbuffers
 *
 * Description:
 *   This function is called after the complete of one or more data
 *   transfers.  This function will empty the done queue and release our
 *   reference to each buffer.
 *
 ****************************************************************************/

static void cs43l22_returnbuffers(FAR struct cs43l22_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;

  /* The doneq and in-flight values might be accessed from the interrupt
   * level in some implementations.  Not the best design.  But we will
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

          /* Set the terminating flag.  This will, eventually, cause the
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
 * Name: cs43l22_sendbuffer
 *
 * Description:
 *   Start the transfer an audio buffer to the CS43L22 via I2S.  This
 *   will not wait for the transfer to complete but will return immediately.
 *   the wmd8904_senddone called will be invoked when the transfer
 *   completes, stimulating the worker thread to call this function again.
 *
 ****************************************************************************/

static int cs43l22_sendbuffer(FAR struct cs43l22_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;
  uint32_t timeout;
  int shift;
  int ret = OK;

  /* Loop while there are audio buffers to be sent and we have few than
   * CONFIG_CS43L22_INFLIGHT then "in-flight"
   *
   * The 'inflight' value might be modified from the interrupt level in some
   * implementations.  We will use interrupt controls to protect against
   * that possibility.
   *
   * The 'pendq', on the other hand, is protected via a semaphore.  Let's
   * hold the semaphore while we are busy here and disable the interrupts
   * only while accessing 'inflight'.
   */

  cs43l22_takesem(&priv->pendsem);
  while (priv->inflight < CONFIG_CS43L22_INFLIGHT &&
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

      /* Send the entire audio buffer via I2S.  What is a reasonable timeout
       * to use?  This would depend on the bit rate and size of the buffer.
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
       * - 16000 is a approximately 16384 (1 << 14), bpsamp is either
       *   (1 << 3) or (1 << 4), and nchannels is either (1 << 0) or
       *   (1 << 1).  So this can be simplifies to (milliseconds):
       *
       *   = (buffer_size << shift) / samplerate
       */

      shift  = (priv->bpsamp == 8) ? 14 - 3 : 14 - 4;
      shift -= (priv->nchannels > 1) ? 1 : 0;

      timeout = MSEC2TICK(((uint32_t)(apb->nbytes - apb->curbyte) << shift) /
                           (uint32_t)priv->samprate);

      ret = I2S_SEND(priv->i2s, apb, cs43l22_senddone, priv, timeout);
      if (ret < 0)
        {
          auderr("ERROR: I2S_SEND failed: %d\n", ret);
          break;
        }
    }

  cs43l22_givesem(&priv->pendsem);
  return ret;
}

/****************************************************************************
 * Name: cs43l22_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cs43l22_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int cs43l22_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct cs43l22_dev_s *priv = (FAR struct cs43l22_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  FAR void *value;
  int ret;

  audinfo("Entry\n");

  /* Exit reduced power modes of operation */
  /* REVISIT */

  /* Create a message queue for the worker thread */

  snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%X", priv);

  attr.mq_maxmsg  = 16;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  priv->mq = mq_open(priv->mqname, O_RDWR | O_CREAT, 0644, &attr);
  if (priv->mq == NULL)
    {
      /* Error creating message queue! */

      auderr("ERROR: Couldn't allocate message queue\n");
      return -ENOMEM;
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
  (void)pthread_attr_setschedparam(&tattr, &sparam);
  (void)pthread_attr_setstacksize(&tattr, CONFIG_CS43L22_WORKER_STACKSIZE);

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, cs43l22_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("ERROR: pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "cs43l22");
      audinfo("Created worker thread\n");
    }

  return ret;
}

/****************************************************************************
 * Name: cs43l22_stop
 *
 * Description:
 *   Stop the configured operation (audio streaming, volume disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int cs43l22_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#  else
static int cs43l22_stop(FAR struct audio_lowerhalf_s *dev)
#  endif
{
  FAR struct cs43l22_dev_s *priv = (FAR struct cs43l22_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;

  /* Send a message to stop all audio streaming */

  term_msg.msgId = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  (void)nxmq_send(priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
                  CONFIG_CS43L22_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  /* Enter into a reduced power usage mode */
  /* REVISIT: */

  return OK;
}
#endif

/****************************************************************************
 * Name: cs43l22_pause
 *
 * Description:
 *   Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int cs43l22_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#  else
static int cs43l22_pause(FAR struct audio_lowerhalf_s *dev)
#  endif
{
  FAR struct cs43l22_dev_s *priv = (FAR struct cs43l22_dev_s *)dev;

  if (priv->running && !priv->paused)
    {
      /* Disable interrupts to prevent us from suppling any more data */

      priv->paused = true;
      cs43l22_setvolume(priv, priv->volume, true);
      CS43L22_DISABLE(priv->lower);
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: cs43l22_resume
 *
 * Description:
 *   Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#  ifdef CONFIG_AUDIO_MULTI_SESSION
static int cs43l22_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#  else
static int cs43l22_resume(FAR struct audio_lowerhalf_s *dev)
#  endif
{
  FAR struct cs43l22_dev_s *priv = (FAR struct cs43l22_dev_s *)dev;

  if (priv->running && priv->paused)
    {
      priv->paused = false;
      cs43l22_setvolume(priv, priv->volume, false);

      /* Enable interrupts to allow sampling data */

      cs43l22_sendbuffer(priv);
#ifdef CS43L22_USE_FFLOCK_INT
      CS43L22_ENABLE(priv->lower);
#endif
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: cs43l22_enqueuebuffer
 *
 * Description:
 *   Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/

static int cs43l22_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                 FAR struct ap_buffer_s *apb)
{
  FAR struct cs43l22_dev_s *priv = (FAR struct cs43l22_dev_s *)dev;
  struct audio_msg_s term_msg;
  int ret;

  audinfo("Enqueueing: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
          apb, apb->curbyte, apb->nbytes, apb->flags);

  /* Take a reference on the new audio buffer */

  apb_reference(apb);

  /* Add the new buffer to the tail of pending audio buffers */

  cs43l22_takesem(&priv->pendsem);
  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;
  dq_addlast(&apb->dq_entry, &priv->pendq);
  cs43l22_givesem(&priv->pendsem);

  /* Send a message to the worker thread indicating that a new buffer has been
   * enqueued.  If mq is NULL, then the playing has not yet started.  In that
   * case we are just "priming the pump" and we don't need to send any message.
   */

  ret = OK;
  if (priv->mq != NULL)
    {
      term_msg.msgId  = AUDIO_MSG_ENQUEUE;
      term_msg.u.data = 0;

      ret = nxmq_send(priv->mq, (FAR const char *)&term_msg,
                      sizeof(term_msg), CONFIG_CS43L22_MSG_PRIO);
      if (ret < 0)
        {
          auderr("ERROR: nxmq_send failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: cs43l22_cancelbuffer
 *
 * Description:
 *   Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int cs43l22_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  audinfo("apb=%p\n", apb);
  return OK;
}

/****************************************************************************
 * Name: cs43l22_ioctl
 *
 * Description:
 *   Perform a device ioctl
 *
 ****************************************************************************/

static int cs43l22_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                         unsigned long arg)
{
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
  FAR struct ap_buffer_info_s *bufinfo;
#endif

  /* Deal with ioctls passed from the upper-half driver */

  switch (cmd)
    {
      /* Check for AUDIOIOC_HWRESET ioctl.  This ioctl is passed straight
       * through from the upper-half audio driver.
       */

      case AUDIOIOC_HWRESET:
        {
          /* REVISIT:  Should we completely re-initialize the chip?   We
           * can't just issue a software reset; that would puts all WM8904
           * registers back in their default state.
           */

          audinfo("AUDIOIOC_HWRESET:\n");
        }
        break;

       /* Report our preferred buffer size and quantity */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:
        {
          audinfo("AUDIOIOC_GETBUFFERINFO:\n");
          bufinfo              = (FAR struct ap_buffer_info_s *)arg;
          bufinfo->buffer_size = CONFIG_CS43L22_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_CS43L22_NUM_BUFFERS;
        }
        break;
#endif

      default:
        audinfo("Ignored\n");
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: cs43l22_reserve
 *
 * Description:
 *   Reserves a session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int
cs43l22_reserve(FAR struct audio_lowerhalf_s *dev, FAR void **session)
#else
static int cs43l22_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct cs43l22_dev_s *priv = (FAR struct cs43l22_dev_s *)dev;
  int ret = OK;

  /* Borrow the APBQ semaphore for thread sync */

  cs43l22_takesem(&priv->pendsem);
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

  cs43l22_givesem(&priv->pendsem);

  return ret;
}

/****************************************************************************
 * Name: cs43l22_release
 *
 * Description:
 *   Releases the session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int cs43l22_release(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int cs43l22_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct cs43l22_dev_s *priv = (FAR struct cs43l22_dev_s *)dev;
  void *value;

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  /* Borrow the APBQ semaphore for thread sync */

  cs43l22_takesem(&priv->pendsem);

  /* Really we should free any queued buffers here */

  priv->reserved = false;
  cs43l22_givesem(&priv->pendsem);

  return OK;
}

/****************************************************************************
 * Name: cs43l22_interrupt_work
 *
 * Description:
 *   CS43L22 interrupt actions cannot be performed in the interrupt handler
 *   because I2C access is not possible in that context.  Instead, all I2C
 *   operations are deferred to the work queue.
 *
 * Assumptions:
 *   CS43L22 interrupts were disabled in the interrupt handler.
 *
 ****************************************************************************/

#ifdef CS43L22_USE_FFLOCK_INT
static void cs43l22_interrupt_work(FAR void *arg)
{
  /* TODO */
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: cs43l22_interrupt
 *
 * Description:
 *   This is the ISR that services the GPIO1/IRQ pin from the CS43L22.  It
 *   signals CS43L22 events such FLL lock.
 *
 ****************************************************************************/

#ifdef CS43L22_USE_FFLOCK_INT
static int
cs43l22_interrupt(FAR const struct cs43l22_lower_s *lower, FAR void *arg)
{
  /* TODO */
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: cs43l22_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 ****************************************************************************/

static void *cs43l22_workerthread(pthread_addr_t pvarg)
{
  FAR struct cs43l22_dev_s *priv = (struct cs43l22_dev_s *)pvarg;
  struct audio_msg_s msg;
  FAR struct ap_buffer_s *apb;
  int msglen;
  int prio;

  audinfo("Entry\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminating = false;
#endif

/* Mark ourself as running and make sure that CS43L22 interrupts are
 * enabled.
 */

  priv->running = true;
#ifdef CS43L22_USE_FFLOCK_INT
  CS43L22_ENABLE(priv->lower);
#endif
  cs43l22_setvolume(priv, priv->volume, false);

  /* Loop as long as we are supposed to be running and as long as we have
   * buffers in-flight.
   */

  while (priv->running || priv->inflight > 0)
    {
      /* Check if we have been asked to terminate.  We have to check if we
       * still have buffers in-flight.  If we do, then we can't stop until
       * birds come back to roost.
       */

      if (priv->terminating && priv->inflight <= 0)
        {
          /* We are IDLE.  Break out of the loop and exit. */

          break;
        }
      else
        {
          /* Check if we can send more audio buffers to the CS43L22 */

          cs43l22_sendbuffer(priv);
        }

      /* Wait for messages from our message queue */

      msglen = nxmq_receive(priv->mq, (FAR char *)&msg, sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (msglen < sizeof(struct audio_msg_s))
        {
          auderr("ERROR: Message too small: %d\n", msglen);
          continue;
        }

      /* Process the message */

      switch (msg.msgId)
        {
          /* The ISR has requested more data.  We will catch this case at
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

          /* We have a new buffer to send.  We will catch this case at
           * the top of the loop.
           */

          case AUDIO_MSG_ENQUEUE:
            audinfo("AUDIO_MSG_ENQUEUE\n");
            break;

          /* We will wake up from the I2S callback with this message */

          case AUDIO_MSG_COMPLETE:
            audinfo("AUDIO_MSG_COMPLETE\n");
            cs43l22_returnbuffers(priv);
            break;

          default:
            auderr("ERROR: Ignoring message ID %d\n", msg.msgId);
            break;
        }
    }

  /* Reset the CS43L22 hardware */

  cs43l22_reset(priv);

  /* Return any pending buffers in our pending queue */

  cs43l22_takesem(&priv->pendsem);
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

  cs43l22_givesem(&priv->pendsem);

  /* Return any pending buffers in our done queue */

  cs43l22_returnbuffers(priv);

  /* Close the message queue */

  mq_close(priv->mq);
  mq_unlink(priv->mqname);
  priv->mq = NULL;

  /* Send an AUDIO_MSG_COMPLETE message to the client */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  audinfo("Exit\n");
  return NULL;
}

/****************************************************************************
 * Name: cs43l22_audio_output
 *
 * Description:
 *   Initialize and configure the CS43L22 device as an audio output device.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None.  No failures are detected.
 *
 ****************************************************************************/

static void cs43l22_audio_output(FAR struct cs43l22_dev_s *priv)
{
  uint8_t regval;

  /* Keep codec powered off */

  regval = CS43L22_POWER_DOWN;
  cs43l22_writereg(priv, CS43L22_POWER_CTRL1, regval);

  /* SPK always off and HP always on */

  regval = CS43L22_PDN_HPB_ON | CS43L22_PDN_HPA_ON | CS43L22_PDN_SPKB_OFF | CS43L22_PDN_SPKA_OFF;
  cs43l22_writereg(priv, CS43L22_POWER_CTRL2, regval);

  /* Clock configuration: Auto detection */

  regval = CS43L22_AUTO_DETECT_ENABLE | CS43L22_CLKDIV2_ENABLE;
  cs43l22_writereg(priv, CS43L22_CLOCK_CTRL, regval);

  /* Set slave mode and Philips audio standard */

  regval = CS43L22_DAC_IF_I2S;
  cs43l22_writereg(priv, CS43L22_INTERFACE_CTRL1, regval);

  /* Power on the codec */

  regval = CS43L22_POWER_UP;
  cs43l22_writereg(priv, CS43L22_POWER_CTRL1, regval);

  /* Disable the analog soft ramp */

  regval = 0x00;
  cs43l22_writereg(priv, CS43L22_ANLG_ZC_SR_SEL, regval);

  /* Disable the digital soft ramp */

  regval = CS43L22_DEEMPHASIS_ENABLE;
  cs43l22_writereg(priv, CS43L22_MISCLLNS_CTRL, regval);

  /* Disable the limiter attack level */

  regval = 0x00;
  cs43l22_writereg(priv, CS43L22_LIM_CTRL1, regval);

  /* Adjust bass and treble levels */

  regval = CS43L22_BASS_GAIN(0x0f);
  cs43l22_writereg(priv, CS43L22_TONE_CTRL, regval);

  /* Adjust PCM volume level */

  regval = 0x0a;
  cs43l22_writereg(priv, CS43L22_PCM_VOL_A, regval);
  cs43l22_writereg(priv, CS43L22_PCM_VOL_B, regval);

  cs43l22_setdatawidth(priv);

  cs43l22_setvolume(priv, CONFIG_CS43L22_INITVOLUME, false);
}

/****************************************************************************
 * Name: cs43l22_audio_input
 *
 * Description:
 *   Initialize and configure the CS43L22 device as an audio output device
 *   (Right input only).  cs43l22_audio_output() must be called first, this
 *   function then modifies the configuration to support audio input.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None.  No failures are detected.
 *
 ****************************************************************************/

#if 0                           /* Not used */
static void cs43l22_audio_input(FAR struct cs43l22_dev_s *priv)
{
  /* TODO */
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: cs43l22_configure_ints
 *
 * Description:
 *   Configure the GPIO/IRQ interrupt
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CS43L22_USE_FFLOCK_INT
static void cs43l22_configure_ints(FAR struct cs43l22_dev_s *priv)
{
  /* TODO */
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: cs43l22_reset
 *
 * Description:
 *   Reset and re-initialize the CS43L22
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cs43l22_reset(FAR struct cs43l22_dev_s *priv)
{
  /* Put audio output back to its initial configuration */

  /* Put audio output back to its initial configuration */

  priv->samprate   = CS43L22_DEFAULT_SAMPRATE;
  priv->nchannels  = CS43L22_DEFAULT_NCHANNELS;
  priv->bpsamp     = CS43L22_DEFAULT_BPSAMP;
#if !defined(CONFIG_AUDIO_EXCLUDE_VOLUME) && !defined(CONFIG_AUDIO_EXCLUDE_BALANCE)
  priv->balance    = 500;          // b16HALF; /* Center balance */
#endif

  /* Software reset.  This puts all CS43L22 registers back in their
   * default state.
   */

  /* cs43l22_writereg(priv, CS43L22_SWRST, 0); */

  /* Configure the CS43L22 hardware as an audio input device */

  cs43l22_audio_output(priv);

  /* Configure interrupts */

  /* cs43l22_configure_ints(priv); */

  /* Configure the FLL and the LRCLK */

    cs43l22_setbitrate(priv);

  /* Dump some information and return the device instance */

    cs43l22_dump_registers(&priv->dev, "After configuration");
    cs43l22_clock_analysis(&priv->dev, "After configuration");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cs43l22_initialize
 *
 * Description:
 *   Initialize the CS43L22 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the CS43L22 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *cs43l22_initialize(FAR struct i2c_master_s *i2c,
                                                 FAR struct i2s_dev_s *i2s,
                                                 FAR const struct
                                                 cs43l22_lower_s *lower)
{
  FAR struct cs43l22_dev_s *priv;
  uint16_t regval;

  /* Sanity check */

  DEBUGASSERT(i2c && i2s && lower);

  /* Allocate a CS43L22 device structure */

  priv = (FAR struct cs43l22_dev_s *)kmm_zalloc(sizeof(struct cs43l22_dev_s));
  if (priv)
    {
      /* Initialize the CS43L22 device structure.  Since we used kmm_zalloc,
       * only the non-zero elements of the structure need to be initialized.
       */

      priv->dev.ops    = &g_audioops;
      priv->lower      = lower;
      priv->i2c        = i2c;
      priv->i2s        = i2s;

      nxsem_init(&priv->pendsem, 0, 1);
      dq_init(&priv->pendq);
      dq_init(&priv->doneq);

      /* Initialize I2C */

      audinfo("address=%02x frequency=%d\n", lower->address, lower->frequency);

      /* Software reset.  This puts all CS43L22 registers back in their default
       * state. */

      CS43L22_HW_RESET(priv->lower);

      cs43l22_dump_registers(&priv->dev, "After reset");

      /* Verify that CS43L22 is present and available on this I2C */

      regval = cs43l22_readreg(priv, CS43L22_ID_REV);
      if ((regval & 0xff) != CS43L22_DEV_ID_REV)
        {
          auderr("ERROR: CS43L22 not found: ID=%02x\n", regval);
          goto errout_with_dev;
        }

      /* Reset and reconfigure the CS43L22 hardware */

      cs43l22_reset(priv);
      return &priv->dev;
    }

  return NULL;

errout_with_dev:
  nxsem_destroy(&priv->pendsem);
  kmm_free(priv);
  return NULL;
}
