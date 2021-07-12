/****************************************************************************
 * drivers/audio/wm8994.c
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

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
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
#include <nuttx/audio/wm8994.h>

#include "wm8994.h"

/* Pre-processor Definitions */

/* Maximum number of retries */

#define MAX_RETRIES  3

#define WM8994_OUTPUT_DEVICE_SPEAKER                          ((uint16_t)0x0001)
#define WM8994_OUTPUT_DEVICE_HEADPHONE                        ((uint16_t)0x0002)
#define WM8994_OUTPUT_DEVICE_BOTH                             ((uint16_t)0x0003)
#define WM8994_OUTPUT_DEVICE_AUTO                             ((uint16_t)0x0004)

#define WM8994_INPUT_DEVICE_DIGITAL_MICROPHONE_1              ((uint16_t)0x0100)
#define WM8994_INPUT_DEVICE_DIGITAL_MICROPHONE_2              ((uint16_t)0x0200)
#define WM8994_INPUT_DEVICE_INPUT_LINE_1                      ((uint16_t)0x0300)
#define WM8994_INPUT_DEVICE_INPUT_LINE_2                      ((uint16_t)0x0400)
#define WM8994_INPUT_DEVICE_DIGITAL_MIC1_MIC2                 ((uint16_t)0x0800)

#define WM8994_DEFAULT_OUTPUT_DEVICE                          (WM8994_OUTPUT_DEVICE_SPEAKER)
#define WM8994_DEFAULT_INPUT_DEVICE                           (WM8994_INPUT_DEVICE_DIGITAL_MIC1_MIC2)
#define WM8994_STARTUP_MODE_COLD                              (1)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if !defined(CONFIG_WM8994_REGDUMP) && !defined(CONFIG_WM8994_CLKDEBUG)
static
#endif
       uint16_t wm8994_readreg(FAR struct wm8994_dev_s *priv,
                  uint16_t regaddr);
static void     wm8994_writereg(FAR struct wm8994_dev_s *priv,
                  uint16_t regaddr, uint16_t regval);
static void     wm8994_takesem(sem_t *sem);
#define         wm8994_givesem(s) nxsem_post(s)

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static inline uint16_t wm8994_scalevolume(uint16_t volume, b16_t scale);
#endif

/* Audio lower half methods (and close friends) */

static int      wm8994_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                  FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8994_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int      wm8994_configure(FAR struct audio_lowerhalf_s *dev,
                  FAR const struct audio_caps_s *caps);
#endif
static int      wm8994_shutdown(FAR struct audio_lowerhalf_s *dev);
static void     wm8994_senddone(FAR struct i2s_dev_s *i2s,
                  FAR struct ap_buffer_s *apb, FAR void *arg, int result);
static void     wm8994_returnbuffers(FAR struct wm8994_dev_s *priv);
static int      wm8994_sendbuffer(FAR struct wm8994_dev_s *priv);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8994_start(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8994_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8994_stop(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8994_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8994_pause(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
static int      wm8994_resume(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8994_pause(FAR struct audio_lowerhalf_s *dev);
static int      wm8994_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int      wm8994_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                  FAR struct ap_buffer_s *apb);
static int      wm8994_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                  FAR struct ap_buffer_s *apb);
static int      wm8994_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                  unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8994_reserve(FAR struct audio_lowerhalf_s *dev,
                  FAR void **session);
#else
static int      wm8994_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int      wm8994_release(FAR struct audio_lowerhalf_s *dev,
                  FAR void *session);
#else
static int      wm8994_release(FAR struct audio_lowerhalf_s *dev);
#endif

/* Interrupt handling an worker thread */

#ifdef WM8994_USE_FFLOCK_INT
static void     wm8994_interrupt_work(FAR void *arg);
static int      wm8994_interrupt(FAR const struct wm8994_lower_s *lower,
                  FAR void *arg);
#endif

static void    *wm8994_workerthread(pthread_addr_t pvarg);

/* Initialization */

static void     wm8994_audio_output(FAR struct wm8994_dev_s *priv);
static void     wm8994_audio_input(FAR struct wm8994_dev_s *priv);
#if 0 /* Not used */
static void     wm8994_audio_input(FAR struct wm8994_dev_s *priv);
#endif
#ifdef WM8994_USE_FFLOCK_INT
static void     wm8994_configure_ints(FAR struct wm8994_dev_s *priv);
#else
#  define       wm8994_configure_ints(p)
#endif
static void     wm8994_hw_reset(FAR struct wm8994_dev_s *priv);

/* Private Data */

static const struct audio_ops_s g_audioops =
{
  wm8994_getcaps,       /* getcaps        */
  wm8994_configure,     /* configure      */
  wm8994_shutdown,      /* shutdown       */
  wm8994_start,         /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  wm8994_stop,          /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  wm8994_pause,         /* pause          */
  wm8994_resume,        /* resume         */
#endif
  NULL,                 /* allocbuffer    */
  NULL,                 /* freebuffer     */
  wm8994_enqueuebuffer, /* enqueue_buffer */
  wm8994_cancelbuffer,  /* cancel_buffer  */
  wm8994_ioctl,         /* ioctl          */
  NULL,                 /* read           */
  NULL,                 /* write          */
  wm8994_reserve,       /* reserve        */
  wm8994_release        /* release        */
};

/* Private Functions */

/* Name: wm8994_readreg
 *
 * Description:
 *    Read the specified 16-bit register from the WM8994 device.
 *
 */

#if !defined(CONFIG_WM8994_REGDUMP) && !defined(CONFIG_WM8994_CLKDEBUG)
static
#endif
uint16_t wm8994_readreg(FAR struct wm8994_dev_s *priv, uint16_t regaddr)
{
  int retries;

  /* Try up to three times to read the register */

  for (retries = 1; retries <= MAX_RETRIES; retries++)
    {
      struct i2c_msg_s msg[2];
      uint8_t data[2];
      uint16_t buffer = ((regaddr >> 8) & 0xff) | ((regaddr << 8) & 0xff00);
      int ret;

      /* Set up to write the address */

      msg[0].frequency = priv->lower->frequency;
      msg[0].addr      = priv->lower->address;
      msg[0].flags     = 0;
      msg[0].buffer    = (uint8_t *)&buffer;
      msg[0].length    = 2;

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
            audwarn("WARNING: I2C_TRANSFER failed: %d ... Resetting\n", ret);

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

/* Name: wm8994_writereg
 *
 * Description:
 *   Write the specified 16-bit register to the WM8994 device.
 *
 */

static void wm8994_writereg(FAR struct wm8994_dev_s *priv, uint16_t regaddr,
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
      uint8_t data[4];
      int ret;

      /* Set up the data to write */

      data[0] = regaddr >> 8;
      data[1] = regaddr & 0xff;
      data[2] = regval >> 8;
      data[3] = regval & 0xff;

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

/* Name: wm8994_takesem
 *
 * Description:
 *  Take a semaphore count, handling the nasty EINTR return if we are
 *  interrupted by a signal.
 *
 */

static void wm8994_takesem(sem_t *sem)
{
  int ret;

  do
    {
      ret = nxsem_wait(sem);
      DEBUGASSERT(ret == 0 || ret == -EINTR);
    }
  while (ret == -EINTR);
}

/* Name: wm8994_scalevolume
 *
 * Description:
 *   Set the right and left volume values in the WM8994 device based
 *   on the current volume and balance settings.
 *
 */

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static inline uint16_t wm8994_scalevolume(uint16_t volume, b16_t scale)
{
  return b16toi((b16_t)volume * scale);
}
#endif

/* Name: wm8994_setbitrate
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
 *     fout = 11025 samples/sec * 1 * 16 bits/sample = 176.4 bits/sec
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
 */

static void wm8994_setbitrate(FAR struct wm8994_dev_s *priv)
{
  DEBUGASSERT(priv && priv->lower);

  /* TODO */
}

/* Name: wm8994_getcaps
 *
 * Description:
 *   Get the audio device capabilities
 *
 */

static int wm8994_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                          FAR struct audio_caps_s *caps)
{
  /* Validate the structure */

  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("type=%d ac_type=%d\n", type, caps->ac_type);
  return 0;
}

/* Name: wm8994_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 */

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8994_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR void *session,
                            FAR const struct audio_caps_s *caps)
#else
static int wm8994_configure(FAR struct audio_lowerhalf_s *dev,
                            FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv != NULL && caps != NULL);
  audinfo("ac_type: %d\n", caps->ac_type);

  /* TODO */

  return ret;
}

/* Name: wm8994_shutdown
 *
 * Description:
 *   Shutdown the WM8994 chip and put it in the lowest power state possible.
 *
 */

static int wm8994_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)dev;

  DEBUGASSERT(priv);

  /* First disable interrupts */

  WM8994_DISABLE(priv->lower);

  /* Now issue a software reset.  This puts all WM8994 registers back in
   * their default state.
   */

  wm8994_hw_reset(priv);
  return OK;
}

/* Name: wm8994_senddone
 *
 * Description:
 *   This is the I2S callback function that is invoked when the transfer
 *   completes.
 *
 */

static void  wm8994_senddone(FAR struct i2s_dev_s *i2s,
                             FAR struct ap_buffer_s *apb, FAR void *arg,
                             int result)
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)arg;
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
  ret = file_mq_send(&priv->mq, (FAR const char *)&msg, sizeof(msg),
                     CONFIG_WM8994_MSG_PRIO);
  if (ret < 0)
    {
      auderr("ERROR: file_mq_send failed: %d\n", ret);
    }
}

/* Name: wm8994_returnbuffers
 *
 * Description:
 *   This function is called after the complete of one or more data
 *   transfers.  This function will empty the done queue and release our
 *   reference to each buffer.
 *
 */

static void wm8994_returnbuffers(FAR struct wm8994_dev_s *priv)
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

/* Name: wm8994_sendbuffer
 *
 * Description:
 *   Start the transfer an audio buffer to the WM8994 via I2S.  This
 *   will not wait for the transfer to complete but will return immediately.
 *   the wmd8994_senddone called will be invoked when the transfer
 *   completes, stimulating the worker thread to call this function again.
 *
 */

static int wm8994_sendbuffer(FAR struct wm8994_dev_s *priv)
{
  FAR struct ap_buffer_s *apb;
  irqstate_t flags;
  uint32_t timeout;
  int shift;
  int ret = OK;

  /* Loop while there are audio buffers to be sent and we have few than
   * CONFIG_WM8994_INFLIGHT then "in-flight"
   *
   * The 'inflight' value might be modified from the interrupt level in some
   * implementations.  We will use interrupt controls to protect against
   * that possibility.
   *
   * The 'pendq', on the other hand, is protected via a semaphore.  Let's
   * hold the semaphore while we are busy here and disable the interrupts
   * only while accessing 'inflight'.
   */

  wm8994_takesem(&priv->pendsem);
  while (priv->inflight < CONFIG_WM8994_INFLIGHT &&
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

      ret = I2S_SEND(priv->i2s, apb, wm8994_senddone, priv, timeout);
      if (ret < 0)
        {
          auderr("ERROR: I2S_SEND failed: %d\n", ret);
          break;
        }
    }

  wm8994_givesem(&priv->pendsem);
  return ret;
}

/* Name: wm8994_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 */

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8994_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int wm8994_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)dev;
  struct sched_param sparam;
  struct mq_attr attr;
  pthread_attr_t tattr;
  FAR void *value;
  int ret;

  audinfo("Entry\n");

  /* Exit reduced power modes of operation */

  /* REVISIT */

  /* Create a message queue for the worker thread */

  snprintf(priv->mqname, sizeof(priv->mqname), "/tmp/%p", priv);

  attr.mq_maxmsg  = 16;
  attr.mq_msgsize = sizeof(struct audio_msg_s);
  attr.mq_curmsgs = 0;
  attr.mq_flags   = 0;

  ret = file_mq_open(&priv->mq, priv->mqname, O_RDWR | O_CREAT, 0644, &attr);
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
  (void)pthread_attr_setschedparam(&tattr, &sparam);
  (void)pthread_attr_setstacksize(&tattr, CONFIG_WM8994_WORKER_STACKSIZE);

  audinfo("Starting worker thread\n");
  ret = pthread_create(&priv->threadid, &tattr, wm8994_workerthread,
                       (pthread_addr_t)priv);
  if (ret != OK)
    {
      auderr("ERROR: pthread_create failed: %d\n", ret);
    }
  else
    {
      pthread_setname_np(priv->threadid, "wm8994");
      audinfo("Created worker thread\n");
    }

  return ret;
}

/* Name: wm8994_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 */

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8994_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int wm8994_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)dev;
  struct audio_msg_s term_msg;
  FAR void *value;

  /* Send a message to stop all audio streaming */

  term_msg.msg_id = AUDIO_MSG_STOP;
  term_msg.u.data = 0;
  (void)file_mq_send(&priv->mq, (FAR const char *)&term_msg,
                     sizeof(term_msg),
                     CONFIG_WM8994_MSG_PRIO);

  /* Join the worker thread */

  pthread_join(priv->threadid, &value);
  priv->threadid = 0;

  /* Enter into a reduced power usage mode */

  /* REVISIT: */

  return OK;
}
#endif

/* Name: wm8994_pause
 *
 * Description: Pauses the playback.
 *
 */

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8994_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int wm8994_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)dev;

  if (priv->running && !priv->paused)
    {
      /* Disable interrupts to prevent us from suppling any more data */

      priv->paused = true;
      WM8994_DISABLE(priv->lower);
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/* Name: wm8994_resume
 *
 * Description: Resumes the playback.
 *
 */

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8994_resume(FAR struct audio_lowerhalf_s *dev,
    FAR void *session)
#else
static int wm8994_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)dev;

  if (priv->running && priv->paused)
    {
      priv->paused = false;

      /* Enable interrupts to allow sampling data */

      wm8994_sendbuffer(priv);
#ifdef WM8994_USE_FFLOCK_INT
      WM8994_ENABLE(priv->lower);
#endif
    }

  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/* Name: wm8994_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 */

static int wm8994_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                FAR struct ap_buffer_s *apb)
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)dev;
  struct audio_msg_s  term_msg;
  int ret;

  audinfo("Enqueueing: apb=%p curbyte=%d nbytes=%d flags=%04x\n",
          apb, apb->curbyte, apb->nbytes, apb->flags);

  /* Take a reference on the new audio buffer */

  apb_reference(apb);

  /* Add the new buffer to the tail of pending audio buffers */

  wm8994_takesem(&priv->pendsem);
  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;
  dq_addlast(&apb->dq_entry, &priv->pendq);
  wm8994_givesem(&priv->pendsem);

  /* Send a message to the worker thread indicating that a new buffer has
   * been enqueued.  If mq is NULL, then the playing has not yet started.
   * In that case we are just "priming the pump" and we don't need to send
   * any message.
   */

  ret = OK;
  if (priv->mq.f_inode != NULL)
    {
      term_msg.msg_id  = AUDIO_MSG_ENQUEUE;
      term_msg.u.data = 0;

      ret = file_mq_send(&priv->mq, (FAR const char *)&term_msg,
                         sizeof(term_msg), CONFIG_WM8994_MSG_PRIO);
      if (ret < 0)
        {
          auderr("ERROR: file_mq_send failed: %d\n", ret);
        }
    }

  return ret;
}

/** Name: wm8994_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 */

static int wm8994_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                               FAR struct ap_buffer_s *apb)
{
  audinfo("apb=%p\n", apb);
  return OK;
}

/* Name: wm8994_ioctl
 *
 * Description: Perform a device ioctl
 *
 */

static int wm8994_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
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
           * can't just issue a software reset; that would puts all WM8994
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
          bufinfo->buffer_size = CONFIG_WM8994_BUFFER_SIZE;
          bufinfo->nbuffers    = CONFIG_WM8994_NUM_BUFFERS;
        }
        break;
#endif

      default:
        audinfo("Ignored\n");
        break;
    }

  return OK;
}

/* Name: wm8994_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 */

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8994_reserve(FAR struct audio_lowerhalf_s *dev,
                          FAR void **session)
#else
static int wm8994_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *) dev;
  int   ret = OK;

  /* Borrow the APBQ semaphore for thread sync */

  wm8994_takesem(&priv->pendsem);
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

  wm8994_givesem(&priv->pendsem);

  return ret;
}

/* Name: wm8994_release
 *
 * Description: Releases the session (the only one we have).
 *
 */

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int wm8994_release(FAR struct audio_lowerhalf_s *dev,
                          FAR void *session)
#else
static int wm8994_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)dev;
  void  *value;

  /* Join any old worker thread we had created to prevent a memory leak */

  if (priv->threadid != 0)
    {
      pthread_join(priv->threadid, &value);
      priv->threadid = 0;
    }

  /* Borrow the APBQ semaphore for thread sync */

  wm8994_takesem(&priv->pendsem);

  /* Really we should free any queued buffers here */

  priv->reserved = false;
  wm8994_givesem(&priv->pendsem);

  return OK;
}

/* Name: wm8994_interrupt_work
 *
 * Description:
 *   WM8994 interrupt actions cannot be performed in the interrupt handler
 *   because I2C access is not possible in that context.  Instead, all I2C
 *   operations are deferred to the work queue.
 *
 * Assumptions:
 *   WM8994 interrupts were disabled in the interrupt handler.
 *
 */

#ifdef WM8994_USE_FFLOCK_INT
static void wm8994_interrupt_work(FAR void *arg)
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)arg;
  uint16_t regval;

  DEBUGASSERT(priv && priv->lower);

  /* Sample the interrupt status */

  /* regval = wm8994_readreg(priv, WM8994_INT_STATUS); */

  audinfo("INT_STATUS: %04x\n", regval);

  /* Check for the FLL lock interrupt.  We are sloppy here since at
   * present, only the FLL lock interrupt is used.
   */

  DEBUGASSERT((regval & WM8994_FLL_LOCK_INT) != 0 && !priv->locked);
  UNUSED(regval);

  priv->locked = true;

  /* Clear all pending interrupts by write 1's to the interrupt status
   * register.
   *
   * REVISIT: Since I2C is slow and not atomic with respect to WM8994 event,
   * could this not cause the lost of interrupts?
   */

  /* wm8994_writereg(priv, WM8994_INT_STATUS, WM8994_ALL_INTS); */

  /* Disable further FLL lock interrupts.  We are sloppy here since at
   * present, only the FLL lock interrupt is used.
   */

  /* wm8994_writereg(priv, WM8994_INT_MASK, WM8994_ALL_INTS); */

#ifdef WM8994_USE_FFLOCK_INT
  /* Re-enable WM8994 interrupts */

  WM8994_ENABLE(priv->lower);
#endif
}
#endif

/* Name: wm8994_interrupt
 *
 * Description:
 *   This is the ISR that services the GPIO1/IRQ pin from the WM8994.  It
 *   signals WM8994 events such FLL lock.
 *
 */

#ifdef WM8994_USE_FFLOCK_INT
static int wm8994_interrupt(FAR const struct wm8994_lower_s *lower,
                            FAR void *arg)
{
  FAR struct wm8994_dev_s *priv = (FAR struct wm8994_dev_s *)arg;
  int ret;

  DEBUGASSERT(lower && priv);

  /* Disable further interrupts and perform all interrupt related activities
   * on the work thread.  There is nothing that we can do from the interrupt
   * handler because we cannot perform I2C operations here.
   */

  WM8994_DISABLE(priv->lower);

  DEBUGASSERT(work_available(&priv->work));
  ret = work_queue(LPWORK, &priv->work, wm8994_interrupt_work, priv, 0);
  if (ret < 0)
    {
      auderr("ERROR: Failed to schedule work\n");
    }

  return OK;
}
#endif

/* Name: wm8994_workerthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 */

static void *wm8994_workerthread(pthread_addr_t pvarg)
{
  FAR struct wm8994_dev_s *priv = (struct wm8994_dev_s *) pvarg;
  struct audio_msg_s msg;
  FAR struct ap_buffer_s *apb;
  int msglen;
  unsigned int prio;

  audinfo("Entry\n");

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  priv->terminating = false;
#endif

  /* Mark ourself as running and make sure that WM8994 interrupts are
   * enabled.
   */

  priv->running = true;
#ifdef WM8994_USE_FFLOCK_INT
  WM8994_ENABLE(priv->lower);
#endif

  /* Loop as long as we are supposed to be running and as long as we have
   * buffers in-flight.
   */

  while (priv->running || priv->inflight > 0)
    {
      /* Check if we have been asked to terminate.  e have to check if we
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
          /* Check if we can send more audio buffers to the WM8994 */

          wm8994_sendbuffer(priv);
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
            wm8994_returnbuffers(priv);
            break;

          default:
            auderr("ERROR: Ignoring message ID %d\n", msg.msg_id);
            break;
        }
    }

  /* Reset the WM8994 hardware */

  wm8994_hw_reset(priv);

  /* Return any pending buffers in our pending queue */

  wm8994_takesem(&priv->pendsem);
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

  wm8994_givesem(&priv->pendsem);

  /* Return any pending buffers in our done queue */

  wm8994_returnbuffers(priv);

  /* Close the message queue */

  file_mq_close(&priv->mq);
  file_mq_unlink(priv->mqname);

  /* Send an AUDIO_MSG_COMPLETE message to the client */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  audinfo("Exit\n");
  return NULL;
}

/* Name: wm8994_audio_output
 *
 * Description:
 *   Initialize and configure the WM8994 device as an audio output device.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None.  No failures are detected.
 *
 */

static void wm8994_audio_output(FAR struct wm8994_dev_s *priv)
{
  /* TODO */
}

/* Name: wm8994_audio_input
 *
 * Description:
 *   Initialize and configure the WM8994 device as an audio input device.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None.  No failures are detected.
 *
 */

static void wm8994_audio_input(FAR struct wm8994_dev_s *priv)
{
}

/* Name: wm8994_audio_input
 *
 * Description:
 *   Initialize and configure the WM8994 device as an audio output device
 *   (Right input only).  wm8994_audio_output() must be called first, this
 *   function then modifies the configuration to support audio input.
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None.  No failures are detected.
 *
 */

#if 0 /* Not used */
static void wm8994_audio_input(FAR struct wm8994_dev_s *priv)
{
  /* Analogue Left Input 0  */

  wm8994_writereg(priv, WM8994_ANA_LEFT_IN0, WM8994_INMUTE);

  /* Analogue Right Input 0 */

  wm8994_writereg(priv, WM8994_ANA_RIGHT_IN0, WM8994_IN_VOL(5));

  /* Analogue Left Input 1 */

  wm8994_writereg(priv, WM8994_ANA_LEFT_IN1, 0);

  /* Analogue Right Input 1 */

  wm8994_writereg(priv, WM8994_ANA_RIGHT_IN1, WM8994_IP_SEL_N_IN2L);
}
#endif

/* Name: wm8994_configure_ints
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
 */

#ifdef WM8994_USE_FFLOCK_INT
static void  wm8994_configure_ints(FAR struct wm8994_dev_s *priv)
{
  uint16_t regval;

  /* Configure GPIO1 as an IRQ
   *
   *   WM8994_GPIO1_PU=0               : No pull-up
   *   WM8994_GPIO1_PD=1               : Pulled-down
   *   WM8994_GPIO1_SEL_IRQ            : Configured as IRQ
   */

  /* Attach our handler to the GPIO1/IRQ interrupt */

  WM8994_ATTACH(lower, wm8994_interrupt, priv);

  /* Configure interrupts.  wm8994_setbitrate() depends on FLL interrupts. */
}
#endif

/* Name: wm8994_hw_reset
 *
 * Description:
 *   Reset and re-initialize the WM8994
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 */

static void wm8994_hw_reset(FAR struct wm8994_dev_s *priv)
{
  /* Put audio output back to its initial configuration */

  priv->samprate   = WM8994_DEFAULT_SAMPRATE;
  priv->nchannels  = WM8994_DEFAULT_NCHANNELS;
  priv->bpsamp     = WM8994_DEFAULT_BPSAMP;
#if !defined(CONFIG_AUDIO_EXCLUDE_VOLUME) && !defined(CONFIG_AUDIO_EXCLUDE_BALANCE)
  priv->balance    = b16HALF;            /* Center balance */
#endif

  /* Software reset.  This puts all WM8994 registers back in their
   * default state.
   */

  /* 0x0000 = 0 */

  wm8994_writereg(priv, WM8994_SWRST, 0);

  /* wm8994 Errata Work-Arounds */

  /* copy code from STM32Cube_FW_F7_V1.15.0 */

  wm8994_writereg(priv, 0x102, 0x0003);
  wm8994_writereg(priv, 0x817, 0x0000);
  wm8994_writereg(priv, 0x102, 0x0000);
  uint16_t regval;

  /* regval=0x006c */

  regval = WM8994_VMID_RAMP_SOFT_FAST_START | WM8994_VMID_BUF_ENA
            | WM8994_STARTUP_BIAS_ENA;

  /* 0x39 = 0x006c */

  wm8994_writereg(priv, WM8994_ANTI_POP2, regval);

  if (WM8994_DEFAULT_INPUT_DEVICE > 0)
    regval = 0x0013;
  else
    regval = 0x0003;

  /* 0x01 = 0x0013 */

  wm8994_writereg(priv, WM8994_PM1, regval);
  up_mdelay(50);

  /* Configure the WM8994 hardware as an audio output device */

  wm8994_audio_output(priv);
  {
    switch (WM8994_DEFAULT_OUTPUT_DEVICE)
    {
        case WM8994_OUTPUT_DEVICE_SPEAKER:

          /* regval = 0x0c0c */

          /* regval = WM8994_AIF1_DAC2L_ENA | WM8994_AIF1_DAC2R_ENA
           *           | WM8994_DAC2L_ENA | WM8994_DAC2R_ENA;
           */

          regval = 0x0c0c;
          wm8994_writereg(priv, WM8994_PM5, regval); /* 0x05 */

          /* regval = 0x0000 */

          regval = 0;

          /* 0x601 = 0x0000 */

          wm8994_writereg(priv, WM8994_DAC1_LEFT_MIXER_ROUTING, regval);

          regval = 0;           /* regval = 0x0000 */

          /* 0x602 = 0x0000 */

          wm8994_writereg(priv, WM8994_DAC1_RIGHT_MIXER_ROUTING, regval);

          regval = WM8994_AIF1DAC2L_TO_DAC2L_ENA; /* regval = 0x0002 */

          /* 0x604=0x0002 */

          wm8994_writereg(priv, WM8994_DAC2_LEFT_MIXER_ROUTING, regval);

          regval = WM8994_AIF1DAC2R_TO_DAC2R_ENA; /* regval = 0x0002 */

          /* 0x605=0x0002 */

          wm8994_writereg(priv, WM8994_DAC2_RIGHT_MIXER_ROUTING, regval);
          break;
        case WM8994_OUTPUT_DEVICE_HEADPHONE:
          /* regval = WM8994_AIF1_DAC1L_ENA | WM8994_AIF1_DAC1R_ENA
           *            | WM8994_DAC1L_ENA | WM8994_DAC1R_ENA;
           */

          regval = 0x0303;
          wm8994_writereg(priv, WM8994_PM5, regval); /* 0x05 = 0x0303 */

          regval = WM8994_AIF1DAC1L_TO_DAC1L_ENA;
          wm8994_writereg(priv, WM8994_DAC1_LEFT_MIXER_ROUTING, regval); /* 0x601=0x0001 */

          regval = WM8994_AIF1DAC1R_TO_DAC1R_ENA;
          wm8994_writereg(priv, WM8994_DAC1_RIGHT_MIXER_ROUTING, regval); /* 0x602=0x0001 */

          regval = 0;
          wm8994_writereg(priv, WM8994_DAC2_LEFT_MIXER_ROUTING, regval); /* 0x604=0x0000 */

          regval = 0;

          /* 0x605=0x0000 */

          wm8994_writereg(priv, WM8994_DAC2_RIGHT_MIXER_ROUTING, regval);
          break;
        case WM8994_OUTPUT_DEVICE_BOTH:
          if (WM8994_DEFAULT_INPUT_DEVICE ==
              WM8994_INPUT_DEVICE_DIGITAL_MIC1_MIC2)
          {
            wm8994_writereg(priv, 0x005, 0x0303 | 0x0c0c);
            wm8994_writereg(priv, 0x601, 0x0003);
            wm8994_writereg(priv, 0x602, 0x0003);
            wm8994_writereg(priv, 0x604, 0x0003);
            wm8994_writereg(priv, 0x605, 0x0003);
          }
          else
          {
            wm8994_writereg(priv, 0x005, 0x0303 | 0x0c0c);
            wm8994_writereg(priv, 0x601, 0x0001);
            wm8994_writereg(priv, 0x602, 0x0001);
            wm8994_writereg(priv, 0x604, 0x0002);
            wm8994_writereg(priv, 0x605, 0x0002);
          }

          break;
        case WM8994_OUTPUT_DEVICE_AUTO:
          wm8994_writereg(priv, 0x005, 0x0303);
          wm8994_writereg(priv, 0x601, 0x0001);
          wm8994_writereg(priv, 0x602, 0x0001);
          wm8994_writereg(priv, 0x604, 0x0000);
          wm8994_writereg(priv, 0x605, 0x0000);
          break;
        default:
          break;
    }
  }

  /* Configure the WM8994 hardware as an audio input device */

  wm8994_audio_input(priv);
  switch (WM8994_DEFAULT_INPUT_DEVICE)
  {
    case WM8994_INPUT_DEVICE_DIGITAL_MICROPHONE_2:
      wm8994_writereg(priv, 0x004, 0x0c30);
      wm8994_writereg(priv, 0x450, 0x00db);
      wm8994_writereg(priv, 0x002, 0x6000);
      wm8994_writereg(priv, 0x608, 0x0002);
      wm8994_writereg(priv, 0x700, 0x000b);
      break;
  case WM8994_INPUT_DEVICE_INPUT_LINE_1:
      wm8994_writereg(priv, 0x028, 0x0011);
      wm8994_writereg(priv, 0x029, 0x0035);
      wm8994_writereg(priv, 0x02a, 0x0035);
      wm8994_writereg(priv, 0x004, 0x0303);
      wm8994_writereg(priv, 0x440, 0x00db);
      wm8994_writereg(priv, 0x002, 0x6350);
      wm8994_writereg(priv, 0x606, 0x0002);
      wm8994_writereg(priv, 0x607, 0x0002);
      wm8994_writereg(priv, 0x700, 0x000d);
      break;
  case WM8994_INPUT_DEVICE_DIGITAL_MICROPHONE_1:
      wm8994_writereg(priv, 0x004, 0x030c);
      wm8994_writereg(priv, 0x440, 0x00db);
      wm8994_writereg(priv, 0x002, 0x6350);
      wm8994_writereg(priv, 0x606, 0x0002);
      wm8994_writereg(priv, 0x607, 0x0002);
      wm8994_writereg(priv, 0x700, 0x000d);
      break;
  case WM8994_INPUT_DEVICE_DIGITAL_MIC1_MIC2:
      wm8994_writereg(priv, 0x004, 0x0f3c);
      wm8994_writereg(priv, 0x450, 0x00db);
      wm8994_writereg(priv, 0x440, 0x00db);
      wm8994_writereg(priv, 0x002, 0x63a0);
      wm8994_writereg(priv, 0x606, 0x0002);
      wm8994_writereg(priv, 0x607, 0x0002);
      wm8994_writereg(priv, 0x608, 0x0002);
      wm8994_writereg(priv, 0x609, 0x0002);
      wm8994_writereg(priv, 0x700, 0x000d);
      break;
  case WM8994_INPUT_DEVICE_INPUT_LINE_2:
  default:
      break;
  }

  {
    switch (WM8994_DEFAULT_SAMPRATE)
    {
      case WM8994_AUDIO_FREQUENCY_8K:
        regval = WM8994_AIF1CLK_RATE_2 | WM8994_AIF1_SR_8K;

        /* 0x210 = 0x0003 */

        wm8994_writereg(priv, WM8994_AIF1_RATE, regval);
        break;
      case WM8994_AUDIO_FREQUENCY_16K:
        regval = WM8994_AIF1CLK_RATE_2 | WM8994_AIF1_SR_16K;

        /* 0x210 = 0x0033 */

        wm8994_writereg(priv, WM8994_AIF1_RATE, regval);
        break;
      case WM8994_AUDIO_FREQUENCY_22_050K:
        regval = WM8994_AIF1CLK_RATE_2 | WM8994_AIF1_SR_22K;

        /* 0x210 = 0x0063 */

        wm8994_writereg(priv, WM8994_AIF1_RATE, regval);
        break;
        #if 0
      case WM8994_AUDIO_FREQUENCY_48K:
        regval = WM8994_AIF1CLK_RATE_2 | WM8994_AIF1_SR_24K;
        wm8994_writereg(priv, WM8994_AIF1_RATE, regval); /* 0x210 = 0x0083 */
        break;
        #endif
      case WM8994_AUDIO_FREQUENCY_32K:
        regval = WM8994_AIF1CLK_RATE_2 | WM8994_AIF1_SR_32K;
        wm8994_writereg(priv, WM8994_AIF1_RATE, regval); /* 0x210 = 0x00a3 */
        break;
      case WM8994_AUDIO_FREQUENCY_44_100K:
        regval = WM8994_AIF1CLK_RATE_2 | WM8994_AIF1_SR_44K;
        wm8994_writereg(priv, WM8994_AIF1_RATE, regval); /* 0x210 = 0x0013 */
        break;
      case WM8994_AUDIO_FREQUENCY_48K:
        regval = WM8994_AIF1CLK_RATE_2 | WM8994_AIF1_SR_48K;
        wm8994_writereg(priv, WM8994_AIF1_RATE, regval); /* 0x210 = 0x0043 */
        break;
    #if 0
      case WM8994_AUDIO_FREQUENCY_44_100K:
        regval = WM8994_AIF1CLK_RATE_2 | WM8994_AIF1_SR_88K;
        wm8994_writereg(priv, WM8994_AIF1_RATE, regval); /* 0x210 = 0x0073 */
        break;
    #endif
      case WM8994_AUDIO_FREQUENCY_96K:
        regval = WM8994_AIF1CLK_RATE_2 | WM8994_AIF1_SR_96K;

        /* 0x210 = 0x00a3 */

        wm8994_writereg(priv, WM8994_AIF1_RATE, regval);
        break;
      default:
        regval = WM8994_AIF1CLK_RATE_2 | WM8994_AIF1_SR_48K;

        /* 0x210 = 0x0083 */

        wm8994_writereg(priv, WM8994_AIF1_RATE, regval);
        break;
    }

    if (WM8994_DEFAULT_INPUT_DEVICE == WM8994_INPUT_DEVICE_DIGITAL_MIC1_MIC2)

        /* regval = 0x4018 */

        regval = WM8994_AIF1ADCR_RIGHT_ADC | WM8994_AIF1_WL_16BITS
                                           | WM8994_AIF1_FMT_I2S;
    else

        /* regval = 0x4010 */

        regval = WM8994_AIF1ADCR_RIGHT_ADC | WM8994_AIF1_WL_16BITS
                                           | WM8994_AIF1_FMT_DSP;

    /* 0x300 = */

    wm8994_writereg(priv, WM8994_AIF1_CTL1, regval);

    regval = WM8994_AIF1_TRI_NORMAL | WM8994_AIF1_MSTR_SLAVE_MODE
        | WM8994_AIF1_CLK_FRC_NORMAL | WM8994_AIF1_LRCLK_FRC_NORMAL;

    /* 0x302 = 0x0000 */

    wm8994_writereg(priv, WM8994_AIF1_MASTER_SLAVE, regval);

    regval = WM8994_AIF1DSPCLK_ENA | WM8994_SYSDSPCLK_ENA
                                   | WM8994_SYSCLK_SRC_AIF1CLK;

    /* 0x208 = 0x000a */

    wm8994_writereg(priv, WM8994_CLK1, regval);

    regval = WM8994_AIF1CLK_ENA;

    /* 0x200 = 0x0001 */

    wm8994_writereg(priv, WM8994_AIF1_CLK1, regval);

    if (WM8994_DEFAULT_OUTPUT_DEVICE == WM8994_OUTPUT_DEVICE_HEADPHONE)
    {
      regval = WM8994_DAC1L_TO_HPOUT1L_DAC1L;

      /* 0x2d = 0x0100 */

      wm8994_writereg(priv, WM8994_OUTPUT_MIXER1, regval);

      regval = 0;
      wm8994_writereg(priv, WM8994_OUTPUT_MIXER2, regval); /* 0x2e = 0x0100 */

      if (WM8994_STARTUP_MODE_COLD)
      {
        regval = 0x8100;
        wm8994_writereg(priv, WM8994_WR_CTL_SEQ1, regval);

        /* 0x110 = regval */

        up_mdelay(300);
      }
      else
      {
        regval = 0x8108;
        wm8994_writereg(priv, WM8994_WR_CTL_SEQ1, regval); /* 0x110 = regval */
        up_mdelay(50);
      }

      regval = 0;
      wm8994_writereg(priv, WM8994_AIF1_DAC1_FILTERS1, regval); /* 0x420 = 0x0000 */
    }

    regval = 0;
    wm8994_writereg(priv, WM8994_PM3, regval); /* 0x03 = 0x0300 */

    regval = 0;
    wm8994_writereg(priv, WM8994_SPKMIXL_ATT, regval); /* 0x22 = 0x0000 */

    regval = 0;
    wm8994_writereg(priv, WM8994_SPKMIXR_ATT, regval); /* 0x23 = 0x0000 */

    regval = 0;
    wm8994_writereg(priv, WM8994_SPEAKER_MIXER, regval); /* 0x36 = 0x0300 */

    regval = 0;
    wm8994_writereg(priv, WM8994_PM1, regval);  /* 0x01 = 0x3003 */

    if (WM8994_DEFAULT_INPUT_DEVICE == WM8994_INPUT_DEVICE_DIGITAL_MIC1_MIC2)
      regval = 0x0205;
    else
      regval = 0x0005;
    wm8994_writereg(priv, WM8994_CLASS_W_1, regval); /* 0x51 = regval */

    priv->power_mgnt_reg_1 |= 0x0303 | 0x3003;
    regval = priv->power_mgnt_reg_1;
    wm8994_writereg(priv, WM8994_PM1, regval); /* 0x01 = power_mgnt_reg_1 */

    regval = 0;
    wm8994_writereg(priv, WM8994_ANA_HP1, regval); /* 0x60 = 0x0022 */

    regval = 0;
    wm8994_writereg(priv, WM8994_CHARGE_PUMP1, regval); /* 0x4c = 0x9F25 */

    up_mdelay(15);

    regval = 0;
    wm8994_writereg(priv, WM8994_OUTPUT_MIXER1, regval); /* 0x2d = 0x0001 */

    regval = 0;
    wm8994_writereg(priv, 0x2e, regval); /* 0x2e = 0x0001 */

    regval = 0;
    wm8994_writereg(priv, 0x03, regval); /* 0x03 = 0x0030 | 0x0300 */

    regval = 0x0033;
    wm8994_writereg(priv, 0x54, regval); /* 0x54 = 0x0033 */

    up_mdelay(257);

    regval = 0x00ee;
    wm8994_writereg(priv, 0x60, 0x00ee); /* 0x60 = 0x00ee */

    regval = 0x00c0;
    wm8994_writereg(priv, 0x610, regval); /* 0x610 = 0x00c0 */

    regval = 0x00c0;
    wm8994_writereg(priv, 0x611, regval); /* 0x611 = 0x00c0 */

    regval = 0x0010;
    wm8994_writereg(priv, 0x420, regval); /* 0x420 = 0x0010 */

    regval = 0x00c0;
    wm8994_writereg(priv, 0x612, regval); /* 0x612 = 0x00c0 */

    regval = 0x00c0;
    wm8994_writereg(priv, 0x613, regval); /* 0x613 = 0x00c0 */

    regval = 0x0010;
    wm8994_writereg(priv, 0x422, regval); /* 0x422 = 0x0010 */

    if ((WM8994_DEFAULT_INPUT_DEVICE ==
                WM8994_INPUT_DEVICE_DIGITAL_MICROPHONE_1)
        || (WM8994_DEFAULT_INPUT_DEVICE ==
            WM8994_INPUT_DEVICE_DIGITAL_MICROPHONE_2))
    {
        priv->power_mgnt_reg_1 |= 0x0013;
        wm8994_writereg(priv, 0x01, priv->power_mgnt_reg_1); /* 0x01 = power_mgnt_reg_1 */

        regval = 0x0002;
        wm8994_writereg(priv, 0x620, 0x0002); /* 0x620 = 0x0002 */

        regval = 0x3800;
        wm8994_writereg(priv, 0x411, 0x3800); /* 0x411 = 0x3800 */
    }
    else if (WM8994_DEFAULT_INPUT_DEVICE ==
            WM8994_INPUT_DEVICE_DIGITAL_MIC1_MIC2)
    {
        priv->power_mgnt_reg_1 |= 0x0013;
        wm8994_writereg(priv, 0x01, priv->power_mgnt_reg_1); /* 0x01 = power_mgnt_reg_1 */

        regval = 0x0002;
        wm8994_writereg(priv, 0x620, regval); /* 0x620 = 0x0002; */

        regval = 0x1800;
        wm8994_writereg(priv, 0x410, regval); /* 0x410 = 0x1800 */

        regval = 0x1800;
        wm8994_writereg(priv, 0x411, regval); /* 0x411 = 0x1800 */
    }
    else if ((WM8994_DEFAULT_INPUT_DEVICE ==
                WM8994_INPUT_DEVICE_INPUT_LINE_1)
        || (WM8994_DEFAULT_INPUT_DEVICE ==
            WM8994_INPUT_DEVICE_INPUT_LINE_2))

    {
        regval = 0x000b;
        wm8994_writereg(priv, 0x18, regval); /* 0x18 = 0x000b */

        regval = 0x000b;
        wm8994_writereg(priv, 0x1a, regval); /* 0x1A = 0x000B */

        regval = 0x1800;
        wm8994_writereg(priv, 0x410, regval); /* 0x410 = 0x1800 */
    }
  }

  /* Configure interrupts */

  wm8994_configure_ints(priv);

  /* Configure the FLL and the LRCLK */

  wm8994_setbitrate(priv);

  /* Dump some information and return the device instance */

  wm8994_dump_registers(&priv->dev, "After configuration");
  wm8994_clock_analysis(&priv->dev, "After configuration");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Name: wm8994_initialize
 *
 * Description:
 *   Initialize the WM8994 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the WM8994 device is returned on
 *   success; NULL is returned on failure.
 *
 */

FAR struct audio_lowerhalf_s *
  wm8994_initialize(FAR struct i2c_master_s *i2c, FAR struct i2s_dev_s *i2s,
                    FAR const struct wm8994_lower_s *lower)
{
  FAR struct wm8994_dev_s *priv;
  uint16_t regval;

  /* Sanity check */

  DEBUGASSERT(i2c && i2s && lower);

  /* Allocate a WM8994 device structure */

  priv = (FAR struct wm8994_dev_s *)kmm_zalloc(sizeof(struct wm8994_dev_s));
  if (priv)
    {
      /* Initialize the WM8994 device structure.  Since we used kmm_zalloc,
       * only the non-zero elements of the structure need to be initialized.
       */

      priv->dev.ops    = &g_audioops;
      priv->lower      = lower;
      priv->i2c        = i2c;
      priv->i2s        = i2s;

      nxsem_init(&priv->pendsem, 0, 1);
      dq_init(&priv->pendq);
      dq_init(&priv->doneq);

      /* Verify that WM8994 is present and available on this I2C */

      regval = wm8994_readreg(priv, WM8994_ID);

      if (regval != WM8994_SW_RST_DEV_ID1)
        {
          auderr("ERROR: WM8994 not found: ID=%04x\n", regval);
          goto errout_with_dev;
        }

      /* Software reset.  This puts all WM8994 registers back in their
       * default state.
       */

      wm8994_writereg(priv, WM8994_SWRST, 0);
      wm8994_dump_registers(&priv->dev, "After reset");

      /* chip revision */

      audinfo("wm8994 chip revision: %d\n",
                wm8994_readreg(priv, WM8994_CHIP_REV));

      /* Reset and reconfigure the WM8994 hardwaqre */

      wm8994_hw_reset(priv);
      return &priv->dev;
    }

  return NULL;

errout_with_dev:
  nxsem_destroy(&priv->pendsem);
  kmm_free(priv);
  return NULL;
}
