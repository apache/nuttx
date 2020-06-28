/****************************************************************************
 * drivers/audio/wm8904.c
 *
 * Audio device driver for Wolfson Microelectronics WM8904 Audio codec.
 *
 *   Copyright (C) 2014, 2016-2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 * - "WM8904 Ultra Low Power CODEC for Portable Audio Applications, Pre-
 *    Production", September 2012, Rev 3.3, Wolfson Microelectronics
 *
 * -  The framework for this driver is based on Ken Pettit's VS1053 driver.
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

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <fixedmath.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/mqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/wm8904.h>

#include "wm8904.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum number of retries */

#define MAX_RETRIES  3

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if !defined(CONFIG_WM8904_REGDUMP) && !defined(CONFIG_WM8904_CLKDEBUG)
static
#endif
       uint16_t wm8904_readreg(FAR struct wm8904_dev_s *priv,
                  uint8_t regaddr);
static void     wm8904_writereg(FAR struct wm8904_dev_s *priv,
                  uint8_t regaddr, uint16_t regval);
static int      wm8904_takesem(FAR sem_t *sem);
static int      wm8904_forcetake(FAR sem_t *sem);
#define         wm8904_givesem(s) nxsem_post(s)

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static inline uint16_t wm8904_scalevolume(uint16_t volume, b16_t scale);
static void     wm8904_setvolume(FAR struct wm8904_dev_s *priv,
                 uint16_t volume, bool mute);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static void     wm8904_setbass(FAR struct wm8904_dev_s *priv, uint8_t bass);
static void     wm8904_settreble(FAR struct wm8904_dev_s *priv,
                  uint8_t treble);
#endif

static void     wm8904_setdatawidth(FAR struct wm8904_dev_s *priv);
static void     wm8904_setbitrate(FAR struct wm8904_dev_s *priv);

/* Audio lower half methods (and close friends) */

static int      wm8904_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                  FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8904_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int      wm8904_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR const struct audio_caps_s *caps);
#endif
static int      wm8904_shutdown(FAR struct audio_lowerhalf_s *dev);
static void     wm8904_senddone(FAR struct i2s_dev_s *i2s,
                  FAR struct ap_buffer_s *apb, FAR void *arg, int result);
static void     wm8904_returnbuffers(FAR struct wm8904_dev_s *priv);
static int      wm8904_sendbuffer(FAR struct wm8904_dev_s *priv);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8904_start(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8904_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8904_stop(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8904_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8904_pause(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
static int      wm8904_resume(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8904_pause(FAR struct audio_lowerhalf_s *dev);
static int      wm8904_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int      wm8904_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                  FAR struct ap_buffer_s *apb);
static int      wm8904_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                  FAR struct ap_buffer_s *apb);
static int      wm8904_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                  unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8904_reserve(FAR struct audio_lowerhalf_s *dev,
                  FAR void **session);
#else
static int      wm8904_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8904_release(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8904_release(FAR struct audio_lowerhalf_s *dev);
#endif

/* Interrupt handling an worker thread */

#ifdef WM8904_USE_FFLOCK_INT
static void     wm8904_interrupt_work(FAR void *arg);
static int      wm8904_interrupt(FAR const struct wm8904_lower_s *lower,
                  FAR void *arg);
#endif

static void    *wm8904_workerthread(pthread_addr_t pvarg);

/* Initialization */

static void     wm8904_audio_output(FAR struct wm8904_dev_s *priv);
#if 0 /* Not used */
static void     wm8904_audio_input(FAR struct wm8904_dev_s *priv);
#endif
#ifdef WM8904_USE_FFLOCK_INT
static void     wm8904_configure_ints(FAR struct wm8904_dev_s *priv);
#else
#  define       wm8904_configure_ints(p)
#endif
static void     wm8904_hw_reset(FAR struct wm8904_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops =
{
  wm8904_getcaps,       /* getcaps        */
  wm8904_configure,     /* configure      */
  wm8904_shutdown,      /* shutdown       */
  wm8904_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  wm8904_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  wm8904_pause,         /* pause          */
  wm8904_resume,        /* resume         */
#endif
  NULL,                 /* allocbuffer    */
  NULL,                 /* freebuffer     */
  wm8904_enqueuebuffer, /* enqueue_buffer */
  wm8904_cancelbuffer,  /* cancel_buffer  */
  wm8904_ioctl,         /* ioctl          */
  NULL,                 /* read           */
  NULL,                 /* write          */
  wm8904_reserve,       /* reserve        */
  wm8904_release        /* release        */
};

#ifndef CONFIG_WM8904_CLKDEBUG
static
#endif
const uint8_t g_sysclk_scaleb1[WM8904_BCLK_MAXDIV + 1] =
{
  2,  3,  4,  6,  8,  10, 11, /*  1,  1.5,  2,  3,  4,  5,  5.5 */
  12, 16, 20, 22, 24, 32, 40, /*  6,  8,   10, 11, 12, 16, 20   */
  44, 48, 50, 60, 64, 88, 96  /* 22, 24,   25, 30, 32, 44, 48   */
};

#ifndef CONFIG_WM8904_CLKDEBUG
static
#endif
const uint8_t g_fllratio[WM8904_NFLLRATIO] =
{
  1, 2, 4, 8, 16
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wm8904_readreg
 *
 * Description:
 *    Read the specified 16-bit register from the WM8904 device.
 *
 ****************************************************************************/

#if !defined(CONFIG_WM8904_REGDUMP) && !defined(CONFIG_WM8904_CLKDEBUG)
static
#endif
uint16_t wm8904_readreg(FAR struct wm8904_dev_s *priv, uint8_t regaddr)
{
  int retries;

  /* Try up to three times to read the register */

  for (retries = 1; retries <= MAX_RETRIES; retries++)
    {
      struct i2c_msg_s msg[2];
      uint8_t data[2];
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
      msg[1].buffer    = data;
      msg[1].length    = 2;

      /* Read the register data.  The returned value is the number messages
       * completed.
       */

      ret = I2C_TRANSFER(priv->i2c, msg, 2);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up?  Try to shake the bus free.
           * Don't bother with the reset if this was the last attempt.
           */

          if (retries < MAX_RETRIES)
            {
              audwarn("WARNING: I2C_TRANSFER failed: %d ... Resetting\n",
                      ret);

              ret = I2C_RESET(priv->i2c);
              if (ret < 0)
                {
                  auderr("ERROR: I2C_RESET failed: %d\n", ret);
                  break;
                }
            }
#else
          auderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
#endif
        }
      else
        {
          uint16_t regval;

          /* The I2C transfer was successful... break out of the loop and
           * return the value read.
           */

          regval = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
          audinfo("Read: %02x -> %04x\n", regaddr, regval);
          return regval;
        }

      audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }

  /* No error indication is returned on a failure... just return zero */

  return 0;
}

/****************************************************************************
 * Name: wm8904_writereg
 *
 * Description:
 *   Write the specified 16-bit register to the WM8904 device.
 *
 ****************************************************************************/

static void wm8904_writereg(FAR struct wm8904_dev_s *priv, uint8_t regaddr,
                            uint16_t regval)
{
  struct i2c_config_s config;
  int retries;

  /* Setup up the I2C configuration */

  config.frequency = priv->lower->frequency;
  config.address   = priv->lower->address;
  config.addrlen   = 7;

  /* Try up to three times to read the register */

  for (retries = 1; retries <= MAX_RETRIES; retries++)
    {
      uint8_t data[3];
      int ret;

      /* Set up the data to write */

      data[0] = regaddr;
      data[1] = regval >> 8;
      data[2] = regval & 0xff;

      /* Read the register data.  The returned value is the number messages
       * completed.
       */

      ret = i2c_write(priv->i2c, &config, data, 3);
      if (ret < 0)
        {
#ifdef CONFIG_I2C_RESET
          /* Perhaps the I2C bus is locked up?  Try to shake the bus free.
           * Don't bother with the reset if this was the last attempt.
           */

          if (retries < MAX_RETRIES)
            {
              audwarn("WARNING: i2c_write failed: %d ... Resetting\n", ret);

              ret = I2C_RESET(priv->i2c);
              if (ret < 0)
                {
                  auderr("ERROR: I2C_RESET failed: %d\n", ret);
                  break;
                }
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

          audinfo("Write: %02x <- %04x\n", regaddr, regval);
          return;
        }

      audinfo("retries=%d regaddr=%02x\n", retries, regaddr);
    }
}

/****************************************************************************
 * Name: wm8904_takesem
 *
 * Description:
 *  Take a semaphore count, handling the nasty EINTR return if we are
 *  interrupted by a signal.
 *
 ****************************************************************************/

static int wm8904_takesem(sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

/****************************************************************************
 * Name: wm8904_forcetake
 *
 * Description:
 *   This is just another wrapper but this one continues even if the thread
 *   is canceled.  This must be done in certain conditions where were must
 *   continue in order to clean-up resources.
 *
 ****************************************************************************/

static int wm8904_forcetake(FAR sem_t *sem)
{
  int result;
  int ret = OK;

  do
    {
      result = nxsem_wait_uninterruptible(sem);

      /* The only expected error would -ECANCELED meaning that the
       * parent thread has been canceled.  We have to continue and
       * terminate the poll in this case.
       */

      DEBUGASSERT(result == OK || result == -ECANCELED);
      if (ret == OK && result < 0)
        {
          /* Remember the first failure */

          ret = result;
        }
    }
  while (result < 0);

  return ret;
}

/****************************************************************************
 * Name: wm8904_scalevolume
 *
 * Description:
 *   Set the right and left volume values in the WM8904 device based on the
 *   current volume and balance settings.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static inline uint16_t wm8904_scalevolume(uint16_t volume, b16_t scale)
{
  return b16toi((b16_t)volume * scale);
}
#endif

/****************************************************************************
 * Name: wm8904_setvolume
 *
 * Description:
 *   Set the right and left volume values in the WM8904 device based on the
 *   current volume and balance settings.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static void wm8904_setvolume(FAR struct wm8904_dev_s *priv, uint16_t volume,
                             bool mute)
{
  uint32_t leftlevel;
  uint32_t rightlevel;
  uint16_t regval;

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
      leftlevel = wm8904_scalevolume(volume, b16ONE - (b16_t)priv->balance);
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
      rightlevel = wm8904_scalevolume(volume, (b16_t)priv->balance);
    }
#else
  leftlevel  = priv->volume;
  rightlevel = priv->volume;
#endif

  /* Set the volume */

  regval = WM8904_HPOUTZC | WM8904_HPOUT_VOL(leftlevel);
  if (mute)
    {
      regval |= WM8904_HPOUT_MUTE;
    }

  wm8904_writereg(priv, WM8904_ANA_LEFT_OUT1, regval);

  regval = WM8904_HPOUTZC | WM8904_HPOUT_VOL(rightlevel);
  if (mute)
    {
      regval |= WM8904_HPOUT_MUTE;
    }

  wm8904_writereg(priv, WM8904_ANA_RIGHT_OUT1, regval);

  /* Remember the volume level and mute settings */

  priv->volume = volume;
  priv->mute   = mute;
}
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

/****************************************************************************
 * Name: wm8904_setbass
 *
 * Description:
 *   Set the bass level.
 *
 *   The level and range are in whole percentage levels (0-100).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static void wm8904_setbass(FAR struct wm8904_dev_s *priv, uint8_t bass)
{
  audinfo("bass=%u\n", bass);
#warning Missing logic
}
#endif /* CONFIG_AUDIO_EXCLUDE_TONE */

/****************************************************************************
 * Name: wm8904_settreble
 *
 * Description:
 *   Set the treble level .
 *
 *   The level and range are in whole percentage levels (0-100).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static void wm8904_settreble(FAR struct wm8904_dev_s *priv, uint8_t treble)
{
  audinfo("treble=%u\n", treble);
#warning Missing logic
}
#endif /* CONFIG_AUDIO_EXCLUDE_TONE */

/****************************************************************************
 * Name: wm8904_setdatawidth
 *
 * Description:
 *   Set the 8- or 16-bit data modes
 *
 ****************************************************************************/

static void wm8904_setdatawidth(FAR struct wm8904_dev_s *priv)
{
  uint16_t regval;

  /* "8-bit mode is selected whenever DAC_COMP=1 or ADC_COMP=1. The use of
   *  8-bit data allows samples to be passed using as few as 8 BCLK cycles
   *  per LRCLK frame. When using DSP mode B, 8-bit data words may be
   *  transferred consecutively every 8 BCLK cycles.
   *
   * "8-bit mode (without Companding) may be enabled by setting
   *  DAC_COMPMODE=1 or ADC_COMPMODE=1, when DAC_COMP=0 and ADC_COMP=0.
   */

  if (priv->bpsamp == 16)
    {
      /* Reset default default setting */

      regval = (WM8904_AIFADCR_SRC | WM8904_AIFDACR_SRC);
      wm8904_writereg(priv, WM8904_AIF0, regval);
    }
  else
    {
      /* This should select 8-bit with no companding */

      regval = (WM8904_AIFADCR_SRC  | WM8904_AIFDACR_SRC |
                WM8904_ADC_COMPMODE | WM8904_DAC_COMPMODE);
      wm8904_writereg(priv, WM8904_AIF0, regval);
    }
}

/****************************************************************************
 * Name: wm8904_setbitrate
 *
 * Description:
 *   Program the FLL to achieve the requested bitrate (fout).  Given:
 *
 *     samprate  - Samples per second
 *     nchannels - Number of channels of data
 *     bpsamp    - Bits per sample
 *
 *   Then
 *     fout = samprate * nchannels * bpsamp
 *
 *   For example:
 *     samplerate = 11,025 samples/sec
 *     nchannels  = 1
 *     bpsamp     = 16     bits
 *
 *   Then
 *     fout    = 11025 samples/sec * 1 * 16 bits/sample = 176.4 bits/sec
 *
 *   The clocking is configured like this:
 *     MCLK   is the FLL source clock
 *     Fref   is the scaled down version of MCLK
 *     Fvco   is the output frequency from the FLL
 *     Fout   is the final output from the FLL that drives the SYSCLK
 *     SYSCLK can be divided down to generate the BCLK
 *
 *   The FLL output frequency is generated at that fout by:
 *
 *     Fout = (Fvco / FLL_OUTDIV)
 *
 *   The FLL operating frequency is set according to:
 *
 *     Fvco = Fref * N.K * FLL_RATIO
 *
 *   Where Fref is the input frequency frequency as determined by
 *   FLL_CLK_REF_DIV. Fvco must be in the range of 90-100MHz.
 *
 *   As an example:
 *     FLL_CLK_REF_DIV = 16
 *     FLL_OUTDIV = 8
 *     N.K = 187.25
 *     FLL_RATIO=16
 *     Fref =32,768
 *
 *     Fvco = 32,768 * 187.25 / 16 = 383,488 Hz
 *     Fout = 383,488 / 8 = 47,936 Hz (approx. 48Khz)
 *
 ****************************************************************************/

static void wm8904_setbitrate(FAR struct wm8904_dev_s *priv)
{
  uint64_t tmp64;
  uint32_t fref;
  uint32_t fvco;
  uint32_t fout;
  uint32_t minfout;
  uint16_t regval;
  b16_t nk;
  unsigned int fllndx;
  unsigned int divndx;
  unsigned int outdiv;
  unsigned int framelen;
#ifdef WM8904_USE_FFLOCK_INT
  bool enabled;
  int retries;
#endif

  DEBUGASSERT(priv && priv->lower);

  /* First calculate the desired bitrate (fout).  This is based on
   *
   * 1. The I2S frame length (in bits)
   * 2. The number of frames per second = nchannels * samplerate
   */

  framelen = (priv->bpsamp == 8) ? WM8904_FRAMELEN8 : WM8904_FRAMELEN16;
  fout = (uint32_t)priv->samprate * (uint32_t)priv->nchannels * framelen;

  regval = WM8904_LRCLK_DIR | WM8904_LRCLK_RATE(framelen << 1);
  wm8904_writereg(priv, WM8904_AIF3, regval);

  audinfo("sample rate=%u nchannels=%u bpsamp=%u framelen=%d fout=%lu\n",
          priv->samprate, priv->nchannels, priv->bpsamp, framelen,
          (unsigned long)fout);

  /* Disable the SYSCLK.
   *
   * "The SYSCLK signal is enabled by register bit CLK_SYS_ENA. This bit
   * should be set to 0 when reconfiguring clock sources. ... "
   *
   * REVISIT:  This does not appear necessary if we are just reconfiguring
   * the FLL.  Disabling the FLL will stop the SYSCLK input just fine.
   */

  regval = WM8904_SYSCLK_SRCFLL | WM8904_CLK_DSP_ENA;
  wm8904_writereg(priv, WM8904_CLKRATE2, regval);

#if 0 /* Unnecessary */
  /* Unlock forced oscillator control and switch it off */

  wm8904_writereg(priv, WM8904_CTRLIF_TEST_1, WM8904_USER_KEY);
  wm8904_writereg(priv, WM8904_FLL_NCO_TEST1, 0);
  wm8904_writereg(priv, WM8904_CTRLIF_TEST_1, 0);
#endif

  /* "The FLL is enabled using the FLL_ENA register bit. Note that, when
   * changing FLL settings, it is recommended that the digital circuit be
   * disabled via FLL_ENA and then re-enabled after the other register
   * settings have been updated."
   */

  wm8904_writereg(priv, WM8904_FLL_CTRL1, 0);

  /* Determine Fref.  The source reference clock should be the MCLK */

  fref   = priv->lower->mclk;
  regval = (WM8904_FLL_CLK_REF_SRC_MCLK | WM8904_FLL_CLK_REF_DIV1);

  /* MCLK must be divided down so that fref <=13.5MHz */

  if (fref > 4 * 13500000)
    {
      fref >>= 3;
      regval = (WM8904_FLL_CLK_REF_SRC_MCLK | WM8904_FLL_CLK_REF_DIV8);
    }
  else if (fref > 2 * 13500000)
    {
      fref >>= 2;
      regval = (WM8904_FLL_CLK_REF_SRC_MCLK | WM8904_FLL_CLK_REF_DIV4);
    }
  else if (fref > 13500000)
    {
      fref >>= 1;
      regval = (WM8904_FLL_CLK_REF_SRC_MCLK | WM8904_FLL_CLK_REF_DIV2);
    }

  wm8904_writereg(priv, WM8904_FLL_CTRL5, regval);

  /* Fvco must be between 90 and 100Mhz.  In order to meet this
   * requirement, the value of FLL_OUTDIV should be selected according
   * to the desired output Fout.  The divider, FLL_OUTDIV, must be set
   * so that Fvco is in the range 90-100MHz.  The available divisions
   * are integers from 4 to 64.
   *
   *   Fout = Fvco /FLL_OUTDIV
   *
   *
   * Is this Fout realizable?  This often happens for very low frequencies.
   * If so, we can select a different final SYSCLK scaling frequency.
   */

  minfout = WM8904_FVCO_MAX / WM8904_MAXOUTDIV;
  divndx = 0;

  for (; ; )
    {
      /* Calculate the new value of Fout that we would need to provide
       * with this SYSCLK divider in place.
       */

      uint32_t newfout = (g_sysclk_scaleb1[divndx] * fout) >> 1;

      /* Is this increased Fout realizable?  Or are we just just out of
       * dividers?
       */

      if (newfout >= minfout || divndx == WM8904_BCLK_MAXDIV)
        {
          /* In either case, this is the Fout and divider that we will be
           * using.
           */

          fout = newfout;
          break;
        }

      /* We have more.. Try the next divider */

      divndx++;
    }

  /* When we get here, divndx holds the register value for the new SYSCLK
   * divider.  Set the divider value in the Audio Interface 2 register.
   */

  regval = WM8904_OPCLK_DIV1 | WM8904_BCLK_DIV(divndx);
  wm8904_writereg(priv, WM8904_AIF2, regval);

  /* Now lets make our best guess for FLL_OUTDIV
   *
   *   FLL_OUTDIV = 95000000 / Fout
   */

  outdiv = ((WM8904_FVCO_MAX + WM8904_FVCO_MAX) >> 1) / fout;
  if (outdiv < 4)
    {
      outdiv = 4;
    }
  else if (outdiv > 64)
    {
      outdiv = 64;
    }

  /* The WM8904 suggests the selecting FLL_RATIO via the following
   * range checks:
   */

  if (fref >= 1000000)
    {
      fllndx = WM8904_NFLLRATIO_DIV1;
    }
  else if (fref > 256000)
    {
      fllndx = WM8904_NFLLRATIO_DIV2;
    }
  else if (fref > 128000)
    {
      fllndx = WM8904_NFLLRATIO_DIV4;
    }
  else if (fref > 64000)
    {
      fllndx = WM8904_NFLLRATIO_DIV8;
    }
  else
    {
      fllndx = WM8904_NFLLRATIO_DIV16;
    }

  /* Finally, we need to determine the value of N.K
   *
   *   Fvco = (Fout * FLL_OUTDIV)
   *   N.K  = Fvco / (FLL_FRATIO * FREF)
   */

  fvco  = fout * outdiv;
  tmp64 = ((uint64_t)fvco << 16) / (g_fllratio[fllndx] * fref);
  nk    = (b16_t)tmp64;

  audinfo("mclk=%lu fref=%lu fvco=%lu fout=%lu divndx=%u\n",
          (unsigned long)priv->lower->mclk, (unsigned long)fref,
          (unsigned long)fvco, (unsigned long)fout, divndx);
  audinfo("N.K=%08lx outdiv=%u fllratio=%u\n",
          (unsigned long)nk, outdiv, g_fllratio[fllndx]);

  /* Save the actual bit rate that we are using.  This will be used by the
   * LRCLCK calculations.
   */

  priv->bitrate = fout;

  /* Now, Configure the FLL */

  /* FLL Control 1
   *
   * FLL_FRACN_ENA=1        : Enables fractional mode
   * FLL_OSC_EN=0           : FLL internal oscillator disabled
   * FLL_ENA=0              : The FLL is not enabled
   *
   * FLL_OSC_ENA must be enabled before enabling FLL_ENA (FLL_OSC_ENA is
   * only required for free-running modes).
   */

  wm8904_writereg(priv, WM8904_FLL_CTRL1, 0);
  wm8904_writereg(priv, WM8904_FLL_CTRL1, WM8904_FLL_FRACN_ENA);

  /* FLL Control 2
   *
   * FLL_OUTDIV             : FLL Fout clock divider
   *                        : Fout = Fvco / FLL_OUTDIV
   *                        : Calculated above
   * FLL_CTRL_RATE=1        : Frequency of the FLL control block,
   *                        : = Fvco / FLL_CTRL_RATE
   * FLL_FRATIO             : Fvco clock divider
   *                        : Determined by MCLK tests above
   */

  regval = WM8904_FLL_OUTDIV(outdiv) | WM8904_FLL_CTRL_RATE(1) |
           WM8904_FLL_FRATIO(fllndx);
  wm8904_writereg(priv, WM8904_FLL_CTRL2, regval);

  /* FLL Control 3
   *
   * Fractional multiply for Fref
   */

  wm8904_writereg(priv, WM8904_FLL_CTRL3, b16frac(nk));

  /* FLL Control 4
   *
   * FLL_N                  : Integer multiply for Fref
   * FLL_GAIN               : Gain applied to error
   */

  regval = WM8904_FLL_N(b16toi(nk)) | WM8904_FLL_GAIN_X1;
  wm8904_writereg(priv, WM8904_FLL_CTRL4, regval);

  /* FLL Control 5
   *
   * FLL_CLK_REF_DIV        : FLL Clock Reference Divider
   *
   * Already set above
   */

  /* Enable the FLL */

  regval = WM8904_FLL_FRACN_ENA | WM8904_FLL_ENA;
  wm8904_writereg(priv, WM8904_FLL_CTRL1, regval);

#if defined(WM8904_USE_FFLOCK_INT)
  /* Make sure that interrupts are enabled */

  enabled = WM8904_ENABLE(priv->lower);

  /* Enable the FLL lock interrupt.  Here we can be sloppy since the FLL
   * lock is the only interrupt every enabled.
   */

  priv->locked = false;
  regval = WM8904_ALL_INTS & ~WM8904_FLL_LOCK_INT;
  wm8904_writereg(priv, WM8904_INT_MASK, regval);

  /* Allow time for FLL lock.  Typical is 2 MSec.  No exotic interlock
   * here; we just poll a flag set by the interrupt handler.
   * REVISIT: Probably not necessary.
   */

  retries = 5;
  do
    {
      nxsig_usleep(5 * 5000);
    }
  while (priv->locked == false && --retries > 0);

  /* Make sure that the FLL lock interrupt is disabled and clear any pending
   * interrupt status (again cutting* some corners).  NOTE: The interrupt
   * handler will do these things if there is no timeout.
   */

  WM8904_DISABLE(priv->lower);
  wm8904_writereg(priv, WM8904_INT_MASK, WM8904_ALL_INTS);
  wm8904_writereg(priv, WM8904_INT_STATUS, WM8904_ALL_INTS);

  /* Restore the interrupt state.  */

  WM8904_RESTORE(priv->lower, enabled)

#elif defined(WM8904_USE_FFLOCK_POLL)
  /* Allow time for FLL lock.  Typical is 2 MSec. */

  retries = 5;
  do
    {
       nxsig_usleep(5 * 5000);
    }
  while ((wm8904_readreg(priv, WM8904_INT_STATUS) &
         WM8904_FLL_LOCK_INT) != 0 ||
         --retries > 0);

  /* Clear all pending status bits by writing 1's into the interrupt status
   * register.
   */

  wm8904_writereg(priv, WM8904_INT_STATUS, WM8904_ALL_INTS);

#endif /* !WM8904_USE_FFLOCK_INT && !WM8904_USE_FFLOCK_POLL */

  /* Re-enable the SYSCLK. */

  regval = WM8904_SYSCLK_SRCFLL | WM8904_CLK_SYS_ENA | WM8904_CLK_DSP_ENA;
  wm8904_writereg(priv, WM8904_CLKRATE2, regval);
}

/****************************************************************************
 * Name: wm8904_getcaps
 *
 * Description:
 *   Get the audio device capabilities
 *
 ****************************************************************************/

static int wm8904_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
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

        /* If the sub-type is UNDEF,
         * then report the Feature Units we support
         */

        if (caps->ac_subtype == AUDIO_FU_UNDEF)
          {
            /* Fill in the ac_controls section with
             * the Feature Units we have
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
 * Name: wm8904_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8904_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session,
                            FAR const struct audio_caps_s *caps)
#else
static int wm8904_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;
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
                /* Scale the volume setting to the range {0.. 63} */

                wm8904_setvolume(priv, (63 * volume / 1000), priv->mute);
              }
            else
              {
                ret = -EDOM;
              }
           }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

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
                wm8904_setbass(priv, bass);
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
                wm8904_settreble(priv, treble);
              }
            else
              {
                ret = -EDOM;
              }
          }
          break;
#endif /* CONFIG_AUDIO_EXCLUDE_TONE */

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

        wm8904_setdatawidth(priv);
        wm8904_setbitrate(priv);
        wm8904_writereg(priv, WM8904_DUMMY, 0x55aa);

        wm8904_clock_analysis(&priv->dev, "AUDIO_TYPE_OUTPUT");
        ret = OK;
      }
      break;

    case AUDIO_TYPE_PROCESSING:
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: wm8904_shutdown
 *
 * Description:
 *   Shutdown the WM8904 chip and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int wm8904_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;

  DEBUGASSERT(priv);

  /* First disable interrupts */

  WM8904_DISABLE(priv->lower);

  /* Now issue a software reset.  This puts all WM8904 registers back in
   * their default state.
   */

  wm8904_hw_reset(priv);
  return OK;
}

/****************************************************************************
 * Name: wm8904_senddone
 *
 * Description:
 *   This is the I2S callback function that is invoked when the transfer
 *   completes.
 *
 ****************************************************************************/

static void  wm8904_senddone(FAR struct i2s_dev_s *i2s,
                             FAR struct ap_buffer_s *apb, FAR void *arg,
                             int result)
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)arg;
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

  msg.msg_id = AUDIO_MSG_COMPLETE;
  ret = nxmq_send(priv->mq, (FAR const char *)&msg, sizeof(msg),
                  CONFIG_WM8904_MSG_PRIO);
  if (ret < 0)
    {
      auderr("ERROR: nxmq_send failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: wm8904_returnbuffers
 *
 * Description:
 *   This function is called after the complete of one or more data
 *   transfers.  This function will empty the done queue and release our
 *   reference to each buffer.
 *
 ****************************************************************************/

static void wm8904_returnbuffers(FAR struct wm8904_dev_s *priv)
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
 * Name: wm8904_sendbuffer
 *
 * Description:
 *   Start the transfer an audio buffer to the WM8904 via I2S.  This
 *   will not wait for the transfer to complete but will return immediately.
 *   the wmd8904_senddone called will be invoked when the transfer
 *   completes, stimulating the worker thread to call this function again.
 *
 ****************************************************************************/

static int wm8904_sendbuffer(FAR struct wm8904_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;
  uint32_t timeout;
  int shift;
  int ret;

  /* Loop while there are audio buffers to be sent and we have few than
   * CONFIG_WM8904_INFLIGHT then "in-flight"
   *
   * The 'inflight' value might be modified from the interrupt level in some
   * implementations.  We will use interrupt controls to protect against
   * that possibility.
   *
   * The 'pendq', on the other hand, is protected via a semaphore.  Let's
   * hold the semaphore while we are busy here and disable the interrupts
   * only while accessing 'inflight'.
   */

  ret = wm8904_takesem(&priv->pendsem);
  if (ret < 0)
    {
      return ret;
    }

  while (priv->inflight < CONFIG_WM8904_INFLIGHT &&
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

      ret = I2S_SEND(priv->i2s, apb, wm8904_senddone, priv, timeout);
      if (ret < 0)
        {
          auderr("ERROR: I2S_SEND failed: %d\n", ret);
          break;
        }
    }

  wm8904_givesem(&priv->pendsem);
  return ret;
}

/****************************************************************************
 * Name: wm8904_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8904_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int wm8904_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;
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
  pthread_attr_setschedparam(&tattr, &sparam);
  pthread_attr_setstacksize(&tattr, CONFIG_WM8904_WORKER_STACKSIZE);

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, wm8904_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("ERROR: pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "wm8904");
      audinfo("Created worker thread\n");
    }

  return ret;
}

/****************************************************************************
 * Name: wm8904_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8904_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int wm8904_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;

  /* Send a message to stop all audio streaming */

  term_msg.msg_id = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  nxmq_send(priv->mq, (FAR const char *)&term_msg, sizeof(term_msg),
            CONFIG_WM8904_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  /* Enter into a reduced power usage mode */

  /* REVISIT: */

  return OK;
}
#endif

/****************************************************************************
 * Name: wm8904_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8904_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int wm8904_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;

  if (priv->running && !priv->paused)
    {
      /* Disable interrupts to prevent us from suppling any more data */

      priv->paused = true;
      wm8904_setvolume(priv, priv->volume, true);
      WM8904_DISABLE(priv->lower);
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: wm8904_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8904_resume(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session)
#else
static int wm8904_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;

  if (priv->running && priv->paused)
    {
      priv->paused = false;
      wm8904_setvolume(priv, priv->volume, false);

      /* Enable interrupts to allow sampling data */

      wm8904_sendbuffer(priv);
#ifdef WM8904_USE_FFLOCK_INT
      WM8904_ENABLE(priv->lower);
#endif
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: wm8904_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/

static int wm8904_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;
  struct audio_msg_s  term_msg;
  int ret;

  audinfo("Enqueueing: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
          apb, apb->curbyte, apb->nbytes, apb->flags);

  /* Take a reference on the new audio buffer */

  apb_reference(apb);

  /* Add the new buffer to the tail of pending audio buffers */

  ret = wm8904_takesem(&priv->pendsem);
  if (ret < 0)
    {
      return ret;
    }

  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;
  dq_addlast(&apb->dq_entry, &priv->pendq);
  wm8904_givesem(&priv->pendsem);

  /* Send a message to the worker thread indicating that a new buffer has
   * been enqueued.  If mq is NULL, then the playing has not yet started.
   * In that case we are just "priming the pump" and we don't need to send
   * any message.
   */

  ret = OK;
  if (priv->mq != NULL)
    {
      term_msg.msg_id  = AUDIO_MSG_ENQUEUE;
      term_msg.u.data = 0;

      ret = nxmq_send(priv->mq, (FAR const char *)&term_msg,
                      sizeof(term_msg), CONFIG_WM8904_MSG_PRIO);
      if (ret < 0)
        {
          auderr("ERROR: nxmq_send failed: %d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: wm8904_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int wm8904_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                               FAR struct ap_buffer_s *apb)
{
  audinfo("apb=%p\n", apb);
  return OK;
}

/****************************************************************************
 * Name: wm8904_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int wm8904_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
  int ret = OK;
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
          bufinfo              = (FAR struct ap_buffer_info_s *) arg;
          bufinfo->buffer_size = CONFIG_WM8904_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_WM8904_NUM_BUFFERS;
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        audinfo("Ignored\n");
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: wm8904_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8904_reserve(FAR struct audio_lowerhalf_s *dev,
                          FAR void **session)
#else
static int wm8904_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *) dev;
  int ret;

  /* Borrow the APBQ semaphore for thread sync */

  ret = wm8904_takesem(&priv->pendsem);
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

  wm8904_givesem(&priv->pendsem);

  return ret;
}

/****************************************************************************
 * Name: wm8904_release
 *
 * Description: Releases the session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8904_release(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#else
static int wm8904_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;
  FAR void *value;
  int ret;

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  /* Borrow the APBQ semaphore for thread sync */

  ret = wm8904_forcetake(&priv->pendsem);

  /* Really we should free any queued buffers here */

  priv->reserved = false;
  wm8904_givesem(&priv->pendsem);

  return ret;
}

/****************************************************************************
 * Name: wm8904_interrupt_work
 *
 * Description:
 *   WM8904 interrupt actions cannot be performed in the interrupt handler
 *   because I2C access is not possible in that context.  Instead, all I2C
 *   operations are deferred to the work queue.
 *
 * Assumptions:
 *   WM8904 interrupts were disabled in the interrupt handler.
 *
 ****************************************************************************/

#ifdef WM8904_USE_FFLOCK_INT
static void wm8904_interrupt_work(FAR void *arg)
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)arg;
  uint16_t regval;

  DEBUGASSERT(priv && priv->lower);

  /* Sample the interrupt status */

  regval = wm8904_readreg(priv, WM8904_INT_STATUS);
  audinfo("INT_STATUS: %04x\n", regval);

  /* Check for the FLL lock interrupt.  We are sloppy here since at
   * present, only the FLL lock interrupt is used.
   */

  DEBUGASSERT((regval & WM8904_FLL_LOCK_INT) != 0 && !priv->locked);
  UNUSED(regval);

  priv->locked = true;

  /* Clear all pending interrupts by write 1's to the interrupt status
   * register.
   *
   * REVISIT: Since I2C is slow and not atomic with respect to WM8904 event,
   * could this not cause the lost of interrupts?
   */

  wm8904_writereg(priv, WM8904_INT_STATUS, WM8904_ALL_INTS);

  /* Disable further FLL lock interrupts.  We are sloppy here since at
   * present, only the FLL lock interrupt is used.
   */

  wm8904_writereg(priv, WM8904_INT_MASK, WM8904_ALL_INTS);

#ifdef WM8904_USE_FFLOCK_INT
  /* Re-enable WM8904 interrupts */

  WM8904_ENABLE(priv->lower);
#endif
}
#endif

/****************************************************************************
 * Name: wm8904_interrupt
 *
 * Description:
 *   This is the ISR that services the GPIO1/IRQ pin from the WM8904.  It
 *   signals WM8904 events such FLL lock.
 *
 ****************************************************************************/

#ifdef WM8904_USE_FFLOCK_INT
static int wm8904_interrupt(FAR const struct wm8904_lower_s *lower,
                            FAR void *arg)
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)arg;
  int ret;

  DEBUGASSERT(lower && priv);

  /* Disable further interrupts and perform all interrupt related activities
   * on the work thread.  There is nothing that we can do from the interrupt
   * handler because we cannot perform I2C operations here.
   */

  WM8904_DISABLE(priv->lower);

  DEBUGASSERT(work_available(&priv->work));
  ret = work_queue(LPWORK, &priv->work, wm8904_interrupt_work, priv, 0);
  if (ret < 0)
    {
      auderr("ERROR: Failed to schedule work\n");
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: wm8904_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 ****************************************************************************/

static void *wm8904_workerthread(pthread_addr_t pvarg)
{
  FAR struct wm8904_dev_s *priv = (struct wm8904_dev_s *) pvarg;
  struct audio_msg_s msg;
  FAR struct ap_buffer_s *apb;
  int msglen;
  unsigned int prio;

  audinfo("Entry\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminating = false;
#endif

  /* Mark ourself as running and make sure that WM8904 interrupts are
   * enabled.
   */

  priv->running = true;
#ifdef WM8904_USE_FFLOCK_INT
  WM8904_ENABLE(priv->lower);
#endif
  wm8904_setvolume(priv, priv->volume, false);

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
          /* Check if we can send more audio buffers to the WM8904 */

          wm8904_sendbuffer(priv);
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

      switch (msg.msg_id)
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
            wm8904_returnbuffers(priv);
            break;

          default:
            auderr("ERROR: Ignoring message ID %d\n", msg.msg_id);
            break;
        }
    }

  /* Reset the WM8904 hardware */

  wm8904_hw_reset(priv);

  /* Return any pending buffers in our pending queue */

  wm8904_forcetake(&priv->pendsem);
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

  wm8904_givesem(&priv->pendsem);

  /* Return any pending buffers in our done queue */

  wm8904_returnbuffers(priv);

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
 * Name: wm8904_audio_output
 *
 * Description:
 *   Initialize and configure the WM8904 device as an audio output device.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None.  No failures are detected.
 *
 ****************************************************************************/

static void wm8904_audio_output(FAR struct wm8904_dev_s *priv)
{
  uint16_t regval;

  /* Bias Control.
   * Preserve undocumented default bit.WM8904_DUMMY
   */

  regval = WM8904_ISEL_HIGH | WM8904_BIAS_ENA | 0x0010;
  wm8904_writereg(priv, WM8904_BIAS_CTRL, regval);

  /* VMID Control */

  regval = WM8904_VMID_BUF_ENA | WM8904_VMID_RES_NORMAL | WM8904_VMID_ENA;
  wm8904_writereg(priv, WM8904_VMID_CTRL, regval);

  /* Mic Bias Control 0 */

  /* MICDET_ENA=1, MICBIAS_ENA=1   */

  regval = WM8904_MICDET_ENA | WM8904_MICBIAS_ENA;
  wm8904_writereg(priv, WM8904_MIC_BIAS_CTRL0, regval);

  /* Mic Bias Control 1. */

  wm8904_writereg(priv, WM8904_MIC_BIAS_CTRL1, 0xc000);

  /* Power Management 0 */

  regval = WM8904_INL_ENA | WM8904_INR_ENA;
  wm8904_writereg(priv, WM8904_PM0, regval);

  /* Power Management 2 */

  regval = WM8904_HPL_PGA_ENA | WM8904_HPR_PGA_ENA;
  wm8904_writereg(priv, WM8904_PM2, regval);

  /* Power Management 6 */

  /* DACL_ENA=1, DACR_ENA=1, ADCL_ENA=1, ADCR_ENA=1  */

  regval = WM8904_DACL_ENA | WM8904_DACR_ENA | WM8904_ADCL_ENA |
           WM8904_ADCR_ENA;
  wm8904_writereg(priv, WM8904_PM6, regval);

  /* Clock Rates 0.
   *
   * This value sets TOCLK_RATE_DIV16=0, TOCLK_RATE_X4=0, and MCLK_DIV=0
   * while preserving the state of some undocumented bits (see wm8904.h).
   *
   *   MCLK_DIV=0           : MCLK is is not divided by 2.
   */

  wm8904_writereg(priv, WM8904_CLKRATE0, 0x845e);

  /* Clock Rates 1.
   *
   * Contains settings the control the sample rate.
   */

  /* Clock Rates 2
   *
   * Contains various controls.  Some that are controlled here include:
   *
   *   WM8904_MCLK_INV=0    : MCLK is not inverted
   *   WM8904_SYSCLK_SRC=1  : SYSCLK source is FLL
   *   WM8904_TOCLK_RATE=0  :
   *   WM8904_OPCLK_ENA=0   :
   *   WM8904_CLK_SYS_ENA=1 : SYSCLK is enabled
   *   WM8904_CLK_DSP_ENA=1 : DSP clock is enabled
   *   WM8904_TOCLK_ENA=0   :
   */

  regval = WM8904_SYSCLK_SRCFLL | WM8904_CLK_SYS_ENA | WM8904_CLK_DSP_ENA;
  wm8904_writereg(priv, WM8904_CLKRATE2, regval);

  /* Audio Interface 0.
   *
   * Reset value is:
   *   No DAC invert
   *   No volume boost
   *   No loopback
   *   Left/Right ADC/DAC channels output on Left/Right
   *   Companding options set by wm8904_setdatawidth()
   */

  wm8904_setdatawidth(priv);

  /* Audio Interface 1.
   *
   * This value sets AIFADC_TDM=0, AIFADC_TDM_CHAN=0, BCLK_DIR=1 while
   * preserving the state of some undocumented bits (see wm8904.h).
   *
   *   Digital audio interface format      : I2S
   *   Digital audio interface word length : 24
   *   AIF_LRCLK_INV=0                     : LRCLK not inverted
   *   BCLK_DIR=1                          : BCLK is an output (will clock
   *                                         I2S).
   *   AIF_BCLK_INV=0                      : BCLK not inverted
   *   AIF_TRIS=0                          : Outputs not tri-stated
   *   AIFADC_TDM_CHAN=0                   : ADCDAT outputs data on slot 0
   *   AIFADC_TDM=0                        : Normal ADCDAT operation
   *   AIFDAC_TDM_CHAN=0                   : DACDAT data input on slot 0
   *   AIFDAC_TDM=0                        : Normal DACDAT operation
   *   Bit 14:                             : Undocumented
   */

  regval = WM8904_AIF_FMT_I2S | WM8904_AIF_WL_24BITS | WM8904_BCLK_DIR |
           0x4000;
  wm8904_writereg(priv, WM8904_AIF1, regval);

  /* Audio Interface 2.
   *
   * Holds GPIO clock divider and the SYSCLK divider needed to generate BCLK.
   * This will get initialized by wm8904_setbitrate().
   */

  /* Audio Interface 3
   *
   * Set LRCLK as an output with rate = BCLK / (2*WM8904_FRAMELENn).  This
   * is a value that varies with bits per sample, n=8 or 16.  Since I2S will
   * send a word on each edge of LRCLK (after a delay), this essentially
   * means that each audio frame is WM8904_FRAMELENn bits in length.
   */

  regval = WM8904_LRCLK_DIR | WM8904_LRCLK_RATE(2*WM8904_FRAMELEN16);
  wm8904_writereg(priv, WM8904_AIF3, regval);

  /* DAC Digital 1 */

  wm8904_writereg(priv, WM8904_DAC_DIGI1, 0);

  /* Analogue Left Input 0 */

  /* Analogue Right Input 0 */

  regval =  WM8904_IN_VOL(5);
  wm8904_writereg(priv, WM8904_ANA_LEFT_IN0, regval);
  wm8904_writereg(priv, WM8904_ANA_RIGHT_IN0, regval);

  /* Analogue Left Input 1 */

  wm8904_writereg(priv, WM8904_ANA_LEFT_IN1, 0);
  wm8904_writereg(priv, WM8904_ANA_RIGHT_IN1, 0);

  /* Analogue OUT1 Left */

  /* Analogue OUT1 Right */

  wm8904_setvolume(priv, CONFIG_WM8904_INITVOLUME, true);

  /* DC Servo 0 */

  regval = WM8904_DCS_ENA_CHAN_1 | WM8904_DCS_ENA_CHAN_0;
  wm8904_writereg(priv, WM8904_DC_SERVO0, regval);

  /* Analogue HP 0 */

  regval = WM8904_HPL_RMV_SHORT | WM8904_HPL_ENA_OUTP | WM8904_HPL_ENA_DLY |
           WM8904_HPL_ENA | WM8904_HPR_RMV_SHORT | WM8904_HPR_ENA_OUTP |
           WM8904_HPR_ENA_DLY | WM8904_HPR_ENA;
  wm8904_writereg(priv, WM8904_ANA_HP0, regval);

  /* Charge Pump 0 */

  wm8904_writereg(priv, WM8904_CHG_PUMP0, WM8904_CP_ENA);

  /* Class W 0 */

  regval = WM8904_CP_DYN_PWR | 0x0004;
  wm8904_writereg(priv, WM8904_CLASS_W0, regval);
}

/****************************************************************************
 * Name: wm8904_audio_input
 *
 * Description:
 *   Initialize and configure the WM8904 device as an audio output device
 *   (Right input only).  wm8904_audio_output() must be called first, this
 *   function then modifies the configuration to support audio input.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None.  No failures are detected.
 *
 ****************************************************************************/

#if 0 /* Not used */
static void wm8904_audio_input(FAR struct wm8904_dev_s *priv)
{
  /* Analogue Left Input 0  */

  wm8904_writereg(priv, WM8904_ANA_LEFT_IN0, WM8904_INMUTE);

  /* Analogue Right Input 0 */

  wm8904_writereg(priv, WM8904_ANA_RIGHT_IN0, WM8904_IN_VOL(5));

  /* Analogue Left Input 1 */

  wm8904_writereg(priv, WM8904_ANA_LEFT_IN1, 0);

  /* Analogue Right Input 1 */

  wm8904_writereg(priv, WM8904_ANA_RIGHT_IN1, WM8904_IP_SEL_N_IN2L);
}
#endif

/****************************************************************************
 * Name: wm8904_configure_ints
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

#ifdef WM8904_USE_FFLOCK_INT
static void  wm8904_configure_ints(FAR struct wm8904_dev_s *priv)
{
  uint16_t regval;

  /* Configure GPIO1 as an IRQ
   *
   *   WM8904_GPIO1_PU=0               : No pull-up
   *   WM8904_GPIO1_PD=1               : Pulled-down
   *   WM8904_GPIO1_SEL_IRQ            : Configured as IRQ
   */

  regval = (WM8904_GPIO1_SEL_IRQ | WM8904_GPIO1_PD);
  wm8904_writereg(priv, WM8904_GPIO_CTRL1, regval);

  /* Attach our handler to the GPIO1/IRQ interrupt */

  WM8904_ATTACH(lower, wm8904_interrupt, priv);

  /* Configure interrupts.  wm8904_setbitrate() depends on FLL interrupts. */

  wm8904_writereg(priv, WM8904_INT_STATUS, WM8904_ALL_INTS);
  wm8904_writereg(priv, WM8904_INT_MASK, WM8904_ALL_INTS);
  wm8904_writereg(priv, WM8904_INT_POL, 0);
  wm8904_writereg(priv, WM8904_INT_DEBOUNCE, WM8904_ALL_INTS);
}
#endif

/****************************************************************************
 * Name: wm8904_hw_reset
 *
 * Description:
 *   Reset and re-initialize the WM8904
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wm8904_hw_reset(FAR struct wm8904_dev_s *priv)
{
  /* Put audio output back to its initial configuration */

  priv->samprate   = WM8904_DEFAULT_SAMPRATE;
  priv->nchannels  = WM8904_DEFAULT_NCHANNELS;
  priv->bpsamp     = WM8904_DEFAULT_BPSAMP;
#if !defined(CONFIG_AUDIO_EXCLUDE_VOLUME) && !defined(CONFIG_AUDIO_EXCLUDE_BALANCE)
  priv->balance    = b16HALF;            /* Center balance */
#endif

  /* Software reset.  This puts all WM8904 registers back in their
   * default state.
   */

  wm8904_writereg(priv, WM8904_SWRST, 0);

  /* Configure the WM8904 hardware as an audio input device */

  wm8904_audio_output(priv);

  /* Configure interrupts */

  wm8904_configure_ints(priv);

  /* Configure the FLL and the LRCLK */

  wm8904_setbitrate(priv);
  wm8904_writereg(priv, WM8904_DUMMY, 0x55aa);

  /* Dump some information and return the device instance */

  wm8904_dump_registers(&priv->dev, "After configuration");
  wm8904_clock_analysis(&priv->dev, "After configuration");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wm8904_initialize
 *
 * Description:
 *   Initialize the WM8904 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the WM8904 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
  wm8904_initialize(FAR struct i2c_master_s *i2c, FAR struct i2s_dev_s *i2s,
                    FAR const struct wm8904_lower_s *lower)
{
  FAR struct wm8904_dev_s *priv;
  uint16_t regval;

  /* Sanity check */

  DEBUGASSERT(i2c && i2s && lower);

  /* Allocate a WM8904 device structure */

  priv = (FAR struct wm8904_dev_s *)kmm_zalloc(sizeof(struct wm8904_dev_s));
  if (priv)
    {
      /* Initialize the WM8904 device structure.  Since we used kmm_zalloc,
       * only the non-zero elements of the structure need to be initialized.
       */

      priv->dev.ops    = &g_audioops;
      priv->lower      = lower;
      priv->i2c        = i2c;
      priv->i2s        = i2s;

      nxsem_init(&priv->pendsem, 0, 1);
      dq_init(&priv->pendq);
      dq_init(&priv->doneq);

      /* Software reset.  This puts all WM8904 registers back in their
       * default state.
       */

      wm8904_writereg(priv, WM8904_SWRST, 0);
      wm8904_dump_registers(&priv->dev, "After reset");

      /* Verify that WM8904 is present and available on this I2C */

      regval = wm8904_readreg(priv, WM8904_ID);
      if (regval != WM8904_SW_RST_DEV_ID1)
        {
          auderr("ERROR: WM8904 not found: ID=%04x\n", regval);
          goto errout_with_dev;
        }

      /* Reset and reconfigure the WM8904 hardwaqre */

      wm8904_hw_reset(priv);
      return &priv->dev;
    }

  return NULL;

errout_with_dev:
  nxsem_destroy(&priv->pendsem);
  kmm_free(priv);
  return NULL;
}
