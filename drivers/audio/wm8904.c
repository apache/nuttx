/****************************************************************************
 * drivers/audio/wm8904.c
 *
 * Audio device driver for Wolfson Microelectronics WM8904 Audio codec.
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   "WM8904 Ultra Low Power CODEC for Portable Audio Applications, Pre-
 *    Production", September 2012, Rev 3.3, Wolfson Microelectronics
 *
 * The framework for this driver is based on Ken Pettit's VS1053 driver.
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
#include <debug.h>
#include <errno.h>
#include <queue.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/wm8904.h>
#include <nuttx/math.h>

#include "wm8904.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#ifndef CONFIG_WM8904_XTALI
#  define CONFIG_WM8904_XTALI             12288000
#endif

#ifndef CONFIG_WM8904_MP3_DECODE_FREQ
#  define CONFIG_WM8904_MP3_DECODE_FREQ   43000000
#endif

#ifndef CONFIG_WM8904_MSG_PRIO
#  define CONFIG_WM8904_MSG_PRIO          1
#endif

#ifndef CONFIG_WM8904_BUFFER_SIZE
#  define CONFIG_WM8904_BUFFER_SIZE       8192
#endif

#ifndef CONFIG_WM8904_NUM_BUFFERS
#  define CONFIG_WM8904_NUM_BUFFERS       2
#endif

#ifndef CONFIG_WM8904_WORKER_STACKSIZE
#  define CONFIG_WM8904_WORKER_STACKSIZE  768
#endif

#define WM8904_DUMMY                      0xFF
#define WM8904_DEFAULT_XTALI              12288000
#define WM8904_DATA_FREQ                  20000000
#define WM8904_RST_USECS                  2000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct wm8904_dev_s
{
  /* We are an audio lower half driver (We are also the upper "half" of
   * the WM8904 driver with respect to the board lower half driver).
   *
   * Terminology: Our "lower" half audio instances will be called dev for the
   * publicly visible version and "priv" for the version that only this driver
   * knows.  From the point of view of this driver, it is the board lower
   * "half" that is referred to as "lower".
   */

  struct audio_lowerhalf_s dev;             /* WM8904 audio lower half (this drive) */

  /* Our specific driver data goes here */

  const FAR struct wm8904_lower_s *lower;   /* Pointer to the board lower functions */
  FAR struct i2c_dev_s   *i2c;              /* I2C driver to use */
  FAR struct i2s_dev_s   *i2s;              /* I2S driver to use */
  FAR struct ap_buffer_s *apb;              /* Pointer to the buffer we are processing */
  struct dq_queue_s       apbq;             /* Our queue for enqueued buffers */
  unsigned long           frequency;        /* Frequency to run the I2C bus at. */
  unsigned long           actual;           /* Current chip frequency */
  mqd_t                   mq;               /* Message queue for receiving messages */
  char                    mqname[16];       /* Our message queue name */
  pthread_t               threadid;         /* ID of our thread */
  sem_t                   apbq_sem;         /* Audio Pipeline Buffer Queue sem access */
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
  int16_t                 volume;           /* Current volume level */
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  int16_t                 balance;          /* Current balance level */
#endif  /* CONFIG_AUDIO_EXCLUDE_BALANCE */
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */
#ifndef CONFIG_AUDIO_EXCLUDE_TONE
  uint8_t                 bass;             /* Bass level */
  uint8_t                 treble;           /* Bass level */
#endif
  uint16_t                endfillbytes;
  uint8_t                 endfillchar;      /* Fill char to send when no more data */
  uint8_t                 running;
  uint8_t                 paused;
  uint8_t                 endmode;
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  uint8_t                 cancelmode;
#endif
  uint8_t                 busy;             /* Set true when device reserved */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     wm8904_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                 FAR struct audio_caps_s *caps);
static int     wm8904_shutdown(FAR struct audio_lowerhalf_s *dev);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int     wm8904_configure(FAR struct audio_lowerhalf_s *dev,
                 FAR void *session, FAR const struct audio_caps_s *caps);
static int     wm8904_start(FAR struct audio_lowerhalf_s *dev,
                 FAR void *session);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int     wm8904_stop(FAR struct audio_lowerhalf_s *dev,
                 FAR void *session);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int     wm8904_pause(FAR struct audio_lowerhalf_s *dev,
                 FAR void *session);
static int     wm8904_resume(FAR struct audio_lowerhalf_s *dev,
                 FAR void *session);
#endif  /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */
static int     wm8904_reserve(FAR struct audio_lowerhalf_s *dev,
                 FAR void** ppContext);
static int     wm8904_release(FAR struct audio_lowerhalf_s *dev,
                 FAR void* pContext);
#else
static int     wm8904_configure(FAR struct audio_lowerhalf_s *dev,
                 FAR const struct audio_caps_s *caps);
static int     wm8904_start(FAR struct audio_lowerhalf_s *dev);
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int     wm8904_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int     wm8904_pause(FAR struct audio_lowerhalf_s *dev);
static int     wm8904_resume(FAR struct audio_lowerhalf_s *dev);
#endif  /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */
static int     wm8904_reserve(FAR struct audio_lowerhalf_s *dev);
static int     wm8904_release(FAR struct audio_lowerhalf_s *dev);
#endif /* CONFIG_AUDIO_MULTI_SESION */
static int     wm8904_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                 FAR struct ap_buffer_s *apb);
static int     wm8904_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                 FAR struct ap_buffer_s *apb);
static int     wm8904_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                 unsigned long arg);

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
  NULL,                 /* alloc_buffer   */
  NULL,                 /* free_buffer    */
  wm8904_enqueuebuffer, /* enqueue_buffer */
  wm8904_cancelbuffer,  /* cancel_buffer  */
  wm8904_ioctl,         /* ioctl          */
  NULL,                 /* read           */
  NULL,                 /* write          */
  wm8904_reserve,       /* reserve        */
  wm8904_release        /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wm8904_readreg
 *
 * Description
 *    Read the specified 16-bit register from the WM8904 device.
 *
 ****************************************************************************/

static uint16_t wm8904_readreg(FAR struct wm8904_dev_s *priv, uint8_t regaddr)
{
  FAR struct i2c_dev_s *i2c = priv->i2c;
  uint16_t regval;
#warning Missing logic
  audvdbg("Read: %02x -> %04x\n", regaddr, regval);
  return regval;
}

/************************************************************************************
 * Name: wm8904_writereg
 *
 * Description:
 *   Write the specified 16-bit register to the WM8904 device.
 *
 ************************************************************************************/

static void wm8904_writereg(FAR struct wm8904_dev_s *priv, uint8_t regaddr, uint16_t regval)
{
  FAR struct i2c_dev_s *i2c = priv->i2c;

  /* Select the AUDIO_CTRL device on the I2C bus */

  audvdbg("Write: %02x <- %04x\n", regaddr, regval);
#warning "Missing logic"

  /* Short delay after a write for WM8904 processing time */
  /* REVISIT: Necessary for the WM8904? */
  usleep(10);
}

/************************************************************************************
 * Name: wm8904_setvolume
 *
 * Description:
 *   Set the right and left volume values in the WM8904 device based on the current
 *   volume and balance settings.
 *
 ************************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static void wm8904_setvolume(FAR struct wm8904_dev_s *priv)
{
  FAR struct i2c_dev_s *i2c = priv->i2c;
  uint32_t              leftlevel;
  uint32_t              rightlevel;

  /* Constrain balance */

#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
  if (priv->balance > 1000)
    {
      priv->balance = 1000;
    }

  /* Calculate the left channel volume level */

  if (priv->balance <= 500)
    {
      leftlevel = priv->volume;
    }
  else if (priv->balance == 1000)
    {
      leftlevel = 0;
    }
  else
    {
      leftlevel = priv->volume * (1000 - priv->balance) / 500;
    }

  /* Calculate the right channel volume level */

  if (priv->balance >= 500)
    {
      rightlevel = priv->volume;
    }
  else if (priv->balance == 0)
    {
      rightlevel = 0;
    }
  else
    {
      rightlevel = priv->volume * priv->balance / 500;
    }
#else
  leftlevel = rightlevel = priv->volume;
#endif

  /* Set the volume */
#warning Missing logic
}
#endif /* CONFIG_AUDIO_EXCLUDE_VOLUME */

/************************************************************************************
 * Name: wm8904_setbass
 *
 * Description:
 *   Set the bass and treble level as specified in the context's bass and treble
 *   variables..
 *
 *   The level and range are in whole percentage levels (0-100).
 *
 ************************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static void wm8904_setbass(FAR struct wm8904_dev_s *priv)
{
  FAR struct i2c_dev_s *i2c = priv->i2c;

#warning Missing logic
}
#endif /* CONFIG_AUDIO_EXCLUDE_TONE */

/****************************************************************************
 * Name: wm8904_getcaps
 *
 * Description: Get the audio device capabilities
 *
 ****************************************************************************/

static int wm8904_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                          FAR struct audio_caps_s *caps)
{
  audvdbg("Entry\n");

  /* Validate the structure */

  DEBUGASSERT(caps->ac_len >= sizeof(struct audio_caps_s));

  /* Fill in the caller's structure based on requested info */

  caps->ac_format[0]   = 0;
  caps->ac_format[1]   = 0;
  caps->ac_controls[0] = 0;
  caps->ac_controls[1] = 0;
  caps->ac_controls[2] = 0;
  caps->ac_controls[3] = 0;

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
              /* The input formats we can decode / accept */

              *((uint16_t *) &caps->ac_format[0]) = 0
#ifdef CONFIG_AUDIO_FORMAT_AC3
                    | (1 << (AUDIO_FMT_AC3 - 1))
#endif
#ifdef CONFIG_AUDIO_FORMAT_MP3
                    | (1 << (AUDIO_FMT_MP3 - 1))
#endif
#ifdef CONFIG_AUDIO_FORMAT_WMA
                    | (1 << (AUDIO_FMT_WMA - 1))
#endif
#ifdef CONFIG_AUDIO_FORMAT_MIDI
                    | (1 << (AUDIO_FMT_MIDI - 1))
#endif
#ifdef CONFIG_AUDIO_FORMAT_PCM
                    | (1 << (AUDIO_FMT_PCM - 1))
#endif
#ifdef CONFIG_AUDIO_FORMAT_OGG_VORBIS
                    | (1 << (AUDIO_FMT_OGG_VORBIS - 1))
#endif
                ;

              /* The types of audio units we implement */

              caps->ac_controls[0] = AUDIO_TYPE_OUTPUT | AUDIO_TYPE_FEATURE |
                                     AUDIO_TYPE_PROCESSING;

              break;

            /* Report sub-formats for MIDI if requested */

#ifdef CONFIG_AUDIO_FORMAT_MIDI
            case AUDIO_FMT_MIDI:
              /* We only support Format 0 */

              caps->ac_controls[0] = AUDIO_SUBFMT_MIDI_0;
              caps->ac_controls[1] = AUDIO_SUBFMT_END;
              break;
#endif

            default:
              caps->ac_controls[0] = AUDIO_SUBFMT_END;
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

              caps->ac_controls[0] = AUDIO_SAMP_RATE_8K | AUDIO_SAMP_RATE_11K |
                                      AUDIO_SAMP_RATE_16K | AUDIO_SAMP_RATE_22K |
                                      AUDIO_SAMP_RATE_32K | AUDIO_SAMP_RATE_44K |
                                      AUDIO_SAMP_RATE_48K;
              break;

            case AUDIO_FMT_MP3:
            case AUDIO_FMT_WMA:
            case AUDIO_FMT_PCM:
              /* Report the Bit rates we support.  The bit rate support is actually a
               * complex function of the format and selected sample rate, and the datasheet
               * has multiple tables to indicate the supported bit rate vs sample rate vs
               * format.  The selected sample rate should be provided in the ac_format
               * field of the query, and only a single sample rate should be given.
               */

              /* TODO:  Create a table or set of tables to report this! */

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

            caps->ac_controls[0] = AUDIO_FU_VOLUME | AUDIO_FU_BASS | AUDIO_FU_TREBLE;
            caps->ac_controls[1] = AUDIO_FU_BALANCE >> 8;
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

              caps->ac_controls[0] = AUDIO_PU_STEREO_EXTENDER;
              break;

            case AUDIO_PU_STEREO_EXTENDER:

              /* Provide capabilities of our Stereo Extender */

              caps->ac_controls[0] = AUDIO_STEXT_ENABLE | AUDIO_STEXT_WIDTH;
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
  int     ret = OK;
#if !defined(CONFIG_AUDIO_EXCLUDE_VOLUME) || !defined(CONFIG_AUDIO_EXCLUDE_TONE)
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;
#endif

  audvdbg("Entry\n");

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_FEATURE:

        /* Process based on Feature Unit */

        switch (*((uint16_t *) caps->ac_format))
          {
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
            case AUDIO_FU_VOLUME:
              /* Set the volume */

              priv->volume = *((uint16_t *) caps->ac_controls);
              wm8904_setvolume(priv);

              break;
#endif  /* CONFIG_AUDIO_EXCLUDE_VOLUME */

#if !defined(CONFIG_AUDIO_EXCLUDE_TONE) && !defined(CONFIG_AUDIO_EXCLUDE_VOLUME)
            case AUDIO_FU_BALANCE:
              /* Set the volume */

              priv->balance = *((uint16_t *) caps->ac_controls);
              wm8904_setvolume(priv);

              break;
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
            case AUDIO_FU_BASS:
              /* Set the bass.  The percentage level (0-100) is in the
               * ac_controls[0] parameter.
               */

              priv->bass = caps->ac_controls[0];
              if (priv->bass > 100)
                priv->bass = 100;
              wm8904_setbass(priv);

              break;

            case AUDIO_FU_TREBLE:
              /* Set the treble.  The percentage level (0-100) is in the
               * ac_controls[0] parameter.
               */

              priv->treble = caps->ac_controls[0];
              if (priv->treble > 100)
                priv->treble = 100;
              wm8904_setbass(priv);

              break;
#endif  /* CONFIG_AUDIO_EXCLUDE_TONE */

            default:
              /* Others we don't support */

              break;
          }

        break;

      case AUDIO_TYPE_PROCESSING:

        /* We only support STEREO_EXTENDER */

        if (*((uint16_t *) caps->ac_format) == AUDIO_PU_STEREO_EXTENDER)
          {
          }

        break;
    }

  return ret;
}

/****************************************************************************
 * Name: wm8904_softreset
 *
 * Description:
 *   Performs a soft reset on the WM8904 chip by setting the RESET bit of
 *   the MODE register.
 *
 ****************************************************************************/

static int wm8904_softreset(FAR struct wm8904_dev_s *priv)
{
  /* First disable interrupts */

  WM8904_DISABLE(priv->lower);

  /* Now issue a reset command */
#warning Missing logic

  /* Switch to the lowest frequency */
#warning Missing logic
  return OK;
}

/****************************************************************************
 * Name: wm8904_hardreset
 *
 * Description:
 *   Performs a hardware reset on the WM8904 chip by toggling the RST line,
 *   disabling interrupts, and setting the default operating frequency.
 *
 ****************************************************************************/

static int wm8904_hardreset(FAR struct wm8904_dev_s *priv)
{
  WM8904_DISABLE(priv->lower);
  return OK;
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
  FAR struct i2c_dev_s  *i2c = priv->i2c;

#warning Missing logic
  return OK;
}

/****************************************************************************
 * Name: wm8904_feeddata
 *
 * Description:
 *   Feeds more data to the wm8904 chip from the enqueued buffers.  It will
 *   continue feeding data until the WM8904 line indicates it can't accept
 *   any more data.
 *
 ****************************************************************************/

static void wm8904_feeddata(FAR struct wm8904_dev_s *priv)
{
  FAR struct i2s_dev_s *i2s = priv->i2s;
  FAR struct ap_buffer_s *apb;
  FAR uint8_t *samp = NULL;
  uint16_t regval;
  int nbytes;
  int ret;

  /* Local stack copy of our active buffer */

  apb = priv->apb;
  //auddbg("Entry apb=%p, Bytes left=%d\n", apb, apb->nbytes - apb->curbyte);

  /* Setup pointer to the next sample in the buffer */

  if (apb)
    {
      samp = &apb->samp[apb->curbyte];
    }
  else if (!priv->endmode)
    {
      return;
    }

  /* Loop until the FIFO is full */

#warning Missing/bogus logic
  while (false)
    {
      /* If endmode, then send fill characters */

      if (priv->endmode)
        {
          nbytes = 32;
          while (nbytes)
            {
#warning BOGUS I2S_SEND call
              I2S_SEND(i2s, NULL, NULL, priv, 0);
              nbytes--;
            }

          /* For the WM8904, after the file has been played, we must
           * send 2052 bytes of endfillchar per the datasheet.
           */

          priv->endfillbytes += 32;

          /* Process end mode logic.  We send 2080 bytes of endfillchar as
           * directed by the datasheet, then set SM_CANCEL.  Then we wait
           * until the chip clears SM_CANCEL while sending endfillchar
           * 32 bytes at a time.
           */

          if (priv->endfillbytes == 32*65)
            {
              /* After at least 2052 bytes, we send an SM_CANCEL */

              WM8904_DISABLE(priv->lower);
#warning Missing logic
              WM8904_ENABLE(priv->lower);
            }
          else if (priv->endfillbytes >= 32*130)
            {
              /* Do a hard reset and terminate */

              wm8904_hardreset(priv);
              priv->running = false;
              priv->endmode = false;
              break;
            }
          else if (priv->endfillbytes > 32*65)
            {
              /* After each 32 byte of endfillchar, check the status
               * register to see if SM_CANCEL has been cleared.  If
               * it has been cleared, then we're done.
               */
#warning Missing logic
              priv->running = false;
              priv->endmode = false;
              break;
            }
        }
      else
        {
          /* Send 32 more bytes.  We only send 32 at a time because this is
           * the meaning of WM8904 active from the chip ... that it can
           * accept at least 32 more bytes.  After each 32 byte block, we
           * will recheck the WM8904 line again.
           */

          nbytes = apb->nbytes - apb->curbyte;
          if (nbytes > 32)
            {
              nbytes = 32;
            }
#if 1
          I2S_SNDBLOCK(priv->i2s, samp, nbytes);
          samp += nbytes;
#else
          nbytes = nbytes;
          while (nbytes--)
            {
              /* Send next byte from the buffer */

#warning BOGUS I2S_SEND call
              I2S_SEND(i2s, NULL, NULL, priv, 0);
              samp++;
            }
#endif
          apb->curbyte += nbytes;

          /* Test if we are in cancel mode.  If we are, then we need
           * to continue sending file data and check for the SM_CANCEL
           * bit going inactive.
           */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
          if (priv->cancelmode)
            {
              /* Check the SM_CANCEL bit */
#warning Missing logic

              /* Cancel has begun.  Switch to endmode */

              apb->curbyte = apb->nbytes = 0;
            }
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

          /* Test if we are at the end of the buffer */

          if (apb->curbyte >= apb->nbytes)
            {
              if (apb->nbytes != apb->nmaxbytes)
                {
                  /* Mark the device as endmode */

                  priv->endmode = true;
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
                  if (priv->cancelmode)
                    {
                      /* If we are in cancel mode, then we don't dequeue the buffer
                       * or need to send another SM_CANCEL, so jump into the middle
                       * of the stop sequence.
                       */

                      priv->endfillbytes = 32*65+1;
                      continue;
                    }
                  else
#endif  /* CONFIG_AUDIO_EXCLUDE_STOP */
                    {
                      priv->endfillbytes = 0;
                    }
                }

              /* We referenced the buffer so we must free it */

              apb_free(apb);
#ifdef CONFIG_AUDIO_MULTI_SESSION
              priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                              apb, OK, NULL);
#else
              priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                              apb, OK);
#endif

              /* Lock the buffer queue to pop the next buffer */

              if ((ret = sem_wait(&priv->apbq_sem)) != OK)
                {
#ifdef CONFIG_AUDIO_MULTI_SESSION
                  priv->dev.upper(priv->dev.priv,
                                  AUDIO_CALLBACK_IOERR, NULL, ret, NULL);
#else
                  priv->dev.upper(priv->dev.priv,
                                  AUDIO_CALLBACK_IOERR, NULL, ret);
#endif
                  auddbg("I/O error!\n");
                  break;
                }

              /* Pop the next entry */

              apb = (struct ap_buffer_s *) dq_remfirst(&priv->apbq);
              priv->apb = apb;

              //auddbg("Next Buffer = %p, bytes = %d\n", apb, apb ? apb->nbytes : 0);
              if (apb == NULL)
                {
                  sem_post(&priv->apbq_sem);
                  break;
                }

              samp = &apb->samp[apb->curbyte];
              apb_reference(apb);                /* Add our buffer reference */
              sem_post(&priv->apbq_sem);
            }
        }
    }
}

/****************************************************************************
 * Name: wm8904_interrupt
 *
 *  This is the ISR that services the WM8904 pin from the WM8904, which
 *  indicates the chip is ready to receive additional data.  We use it to
 *  send a message to our worker thread message queue so it knows to wake
 *  up and send more data.
 *
 ****************************************************************************/

static int wm8904_interrupt(FAR const struct wm8904_lower_s *lower,
                            FAR void *arg)
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)arg;
  struct audio_msg_s msg;

  DEBUGASSERT(lower && priv);

  /* Create a message and send it to the worker thread */

  if (priv->running)
    {
      msg.msgId = AUDIO_MSG_DATA_REQUEST;
      mq_send(priv->mq, &msg, sizeof(msg), CONFIG_WM8904_MSG_PRIO);
    }
  else
    {
      msg.msgId = AUDIO_MSG_DATA_REQUEST;
    }

  return 0;
}

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
  struct audio_msg_s      msg;
  FAR struct ap_buffer_s *apb;
  int                     size;
  int                     prio;
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  uint16_t                regval;
#endif

  auddbg("Entry\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->cancelmode = 0;
#endif
  priv->endmode = priv->endfillbytes = 0;

  /* Fill the WM8904 FIFO with initial data.  */

  wm8904_feeddata(priv);

  /* Loop as long as we are supposed to be running */

  priv->running = true;
  WM8904_ENABLE(priv->lower);
  while (priv->running || priv->endmode)
    {
      /* Check if the WM8904 can accept more data */
#warning Missing logic
        {
          wm8904_feeddata(priv);    /* Feed more data to the WM8904 FIFO */
        }

      /* Wait for messages from our message queue */

      size = mq_receive(priv->mq, &msg, sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (size == 0)
        {
          /* Should we just stop running? */

          priv->running = false;
          break;
        }

      /* Process the message */

      switch (msg.msgId)
        {
          /* The ISR has requested more data */

          case AUDIO_MSG_DATA_REQUEST:
            usleep(500);
            wm8904_feeddata(priv);   /* Feed more data to the WM8904 FIFO */
            break;

          /* Stop the playback */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
          case AUDIO_MSG_STOP:
            /* Send CANCEL message to WM8904 */

            WM8904_DISABLE(priv->lower);
#warning Missing logic
            WM8904_ENABLE(priv->lower);

            /* Set cancelmode */

            priv->cancelmode = true;

            break;
#endif

          /* We will wake up when a new buffer enqueued just in case */

          case AUDIO_MSG_ENQUEUE:
            break;

          default:
            break;
        }
    }

  /* Disable the WM8904 interrupt */

  WM8904_DISABLE(priv->lower);

  /* Cancel any leftover buffer in our queue */

  if (sem_wait(&priv->apbq_sem) == OK)
    {
      /* Get the next buffer from the queue */

      while ((apb = (FAR struct ap_buffer_s *) dq_remfirst(&priv->apbq)) != NULL)
        ;
    }

  sem_post(&priv->apbq_sem);

  /* Free the active buffer */

  if (priv->apb != NULL)
    {
      apb_free(priv->apb);
      priv->apb = NULL;
    }

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

  auddbg("Exit\n");

  return NULL;
}

/****************************************************************************
 * Name: wm8904_start
 *
 * Description: Start the configured operation (audio streaming, volume
 *              enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8904_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int wm8904_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;
  struct mq_attr      attr;
  struct sched_param  sparam;
  pthread_attr_t      tattr;
  int                 ret;
  FAR void           *value;

  auddbg("Entry\n");

  /* Do a soft reset, just in case */

  wm8904_softreset(priv);

  /* Increase the frequency of the part during processing */
#warning Missing logic

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

      auddbg("Couldn't allocate message queue\n");
      return -ENOMEM;
    }

  /* Pop the first enqueued buffer */

  if ((ret = sem_wait(&priv->apbq_sem)) == OK)
    {
      priv->apb = (FAR struct ap_buffer_s *) dq_remfirst(&priv->apbq);
      apb_reference(priv->apb);               /* Add our buffer reference */
      sem_post(&priv->apbq_sem);
    }
  else
    {
      auddbg("Error getting APB Queue sem\n");
      return ret;
    }

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      auddbg("Joining old thread\n");
      pthread_join(priv->threadid, &value);
    }

  /* Start our thread for sending data to the device */

  pthread_attr_init(&tattr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  (void)pthread_attr_setschedparam(&tattr, &sparam);
  (void)pthread_attr_setstacksize(&tattr, CONFIG_WM8904_WORKER_STACKSIZE);

  auddbg("Starting workerthread\n");
  ret = pthread_create(&priv->threadid, &tattr, wm8904_workerthread,
      (pthread_addr_t) priv);
  if (ret != OK)
    {
      auddbg("Can't create worker thread, errno=%d\n", errno);
    }
  else
    {
      pthread_setname_np(priv->threadid, "wm8904");
      auddbg("Created worker thread\n");
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
static int wm8904_stop(FAR struct audio_lowerhalf_s *dev, FAR void* session)
#else
static int wm8904_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;
  struct audio_msg_s  term_msg;
  FAR void*           value;

  /* Send a message to stop all audio streaming */

  term_msg.msgId = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  mq_send(priv->mq, &term_msg, sizeof(term_msg), CONFIG_WM8904_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  /* Reduce the decoder's operating frequency to save power */
#warning Missing logic

  /* Wait for a bit */

  up_mdelay(40);
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
static int wm8904_pause(FAR struct audio_lowerhalf_s *dev, FAR void* session)
#else
static int wm8904_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;

  if (!priv->running)
    return OK;

  /* Disable interrupts to prevent us from suppling any more data */

  priv->paused = true;
  WM8904_DISABLE(priv->lower);
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: wm8904_resume
 *
 * Description: Resuems the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8904_resume(FAR struct audio_lowerhalf_s *dev, FAR void* session)
#else
static int wm8904_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;

  if (!priv->running)
    return OK;

  /* Enable interrupts to allow suppling data */

  priv->paused = false;
  wm8904_feeddata(priv);
  WM8904_ENABLE(priv->lower);
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
                                FAR struct ap_buffer_s *apb )
{
  FAR struct wm8904_dev_s *priv = (FAR struct wm8904_dev_s *)dev;
  struct audio_msg_s  term_msg;
  int ret;

  audvdbg("Entry\n");

  /* Lock access to the apbq */

  if ((ret = sem_wait(&priv->apbq_sem)) == OK)
    {
      /* We can now safely add the buffer to the queue */

      apb->curbyte = 0;
      apb->flags = AUDIO_APB_OUTPUT_ENQUEUED;
      dq_addlast(&apb->dq_entry, &priv->apbq);
      sem_post(&priv->apbq_sem);

      /* Send a message indicating a new buffer enqueued */

      if (priv->mq != NULL)
        {
          term_msg.msgId = AUDIO_MSG_ENQUEUE;
          term_msg.u.data = 0;
          mq_send(priv->mq, &term_msg, sizeof(term_msg), CONFIG_WM8904_MSG_PRIO);
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
                               FAR struct ap_buffer_s *apb )
{
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
        wm8904_hardreset((FAR struct wm8904_dev_s *)dev);
        break;

       /* Report our preferred buffer size and quantity */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
      case AUDIOIOC_GETBUFFERINFO:

        bufinfo = (FAR struct ap_buffer_info_s *) arg;
        bufinfo->buffer_size = CONFIG_WM8904_BUFFER_SIZE;
        bufinfo->nbuffers = CONFIG_WM8904_NUM_BUFFERS;
        break;
#endif

      default:
        break;
    }

  return OK;
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
  int   ret = OK;

  /* Borrow the APBQ semaphore for thread sync */

  if (sem_wait(&priv->apbq_sem) != OK)
    {
      return -EBUSY;
    }

  if (priv->busy)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Initialize the session context.  We don't really use it. */

#ifdef CONFIG_AUDIO_MULTI_SESSION
      *session = NULL;
#endif
      priv->busy    = true;
      priv->running = false;
      priv->paused  = false;
    }

  sem_post(&priv->apbq_sem);

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
  void  *value;

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  /* Borrow the APBQ semaphore for thread sync */

  if (sem_wait(&priv->apbq_sem) != OK)
    {
      return -EBUSY;
    }

  /* Really we should free any queued buffers here */

  priv->busy = false;
  sem_post(&priv->apbq_sem);

  return OK;
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
 *   minor   - The input device minor number
 *   session - Returned if multi-sessions are supported
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *
  wm8904_initialize(FAR struct i2c_dev_s *i2c, FAR struct i2s_dev_s *i2s,
                    FAR const struct wm8904_lower_s *lower, unsigned int devno)
{
  FAR struct wm8904_dev_s *priv;

  /* Sanity check */

  DEBUGASSERT(i2c && i2s && lower);

  /* Allocate a WM8904 device structure */

  priv = (FAR struct wm8904_dev_s *)kmalloc(sizeof(struct wm8904_dev_s));
  if (priv)
    {
      /* Initialize the WM8904 device structure */

      priv->dev.ops    = &g_audioops;
      priv->dev.upper  = NULL;
      priv->dev.priv   = NULL;
      priv->lower      = lower;
      priv->frequency  = CONFIG_WM8904_XTALI / 7;
      priv->i2c        = i2c;
      priv->mq         = NULL;
      priv->busy       = false;
      priv->threadid   = 0;
      priv->running    = 0;

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
      priv->volume     = 250;            /* 25% volume as default */
#ifndef CONFIG_AUDIO_EXCLUDE_BALANCE
      priv->balance    = 500;            /* Center balance */
#endif
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
      priv->bass       = 0;
      priv->treble     = 0;
#endif
      sem_init(&priv->apbq_sem, 0, 1);
      dq_init(&priv->apbq);

      /* Initialize I2C */

      I2C_SETFREQUENCY(i2c, lower->frequency);
      I2C_SETADDRESS(i2c, lower->address, 7);

      /* Verify the device ID read from the chip */
#warning Missing logic

      /* Initialize the WM8904 hardware */
#warning Missing logic

      /* Attach our ISR to this device */

      WM8904_ATTACH(lower, wm8904_interrupt, priv);

      /* Put the drive in the 'shutdown' status */

      wm8904_shutdown(&priv->dev);
    }

  return &priv->dev;
}
