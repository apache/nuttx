/****************************************************************************
 * drivers/audio/cxd56_src.c
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mqueue.h>
#include <nuttx/queue.h>

#include "cxd56.h"
#include "cxd56_src.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* For debugging: dump pre/post SRC data to sdcard */

/* #define DUMP_DATA */

/* Note: 24 bit samples not currently supported by SRC */

#define BUFFER_SAMPLES  (CONFIG_CXD56_AUDIO_BUFFER_SIZE / 2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum cxd56_srcstate_e
{
  CXD56_SRC_OFF,
  CXD56_SRC_RUNNING,
  CXD56_SRC_STOPPING,
  CXD56_SRC_STOPPED
};

struct cxd56_srcdata_s
{
  enum cxd56_srcstate_e state;

  float float_in[BUFFER_SAMPLES];
  float float_out[BUFFER_SAMPLES];
  int float_in_offset;

  SRC_DATA src_data;
  SRC_STATE *src_state;

  struct dq_queue_s *inq;
  struct dq_queue_s *outq;

  char mqname[32];
  struct file mq;
  pthread_t threadid;

  uint8_t bytewidth;
  uint8_t channels;

  float buf_count;
  float buf_increment;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct cxd56_srcdata_s g_src;

#ifdef DUMP_DATA
static char *dump_name_pre  = "/mnt/sd0/dump/nx_player_dump_pre.pcm";
static char *dump_name_post = "/mnt/sd0/dump/nx_player_dump_post.pcm";
static struct file dump_file_pre;
static struct file dump_file_post;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

extern void src_short_to_float_array (const short *in, float *out, int len);
extern void src_float_to_short_array (const float *in, short *out, int len);
extern void src_int_to_float_array (const int *in, float *out, int len);
extern void src_float_to_int_array (const float *in, int *out, int len);

static struct ap_buffer_s *cxd56_src_get_apb(void)
{
  struct ap_buffer_s *src_apb;
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

  if (dq_count(g_src.inq) == 0)
    {
      size_t bufsize = sizeof(struct ap_buffer_s) +
                              CONFIG_CXD56_AUDIO_BUFFER_SIZE;

      spin_unlock_irqrestore(NULL, flags);

      src_apb = kmm_zalloc(bufsize);

      flags = spin_lock_irqsave(NULL);

      if (!src_apb)
        {
          auderr("ERROR: Couldn't allocate SRC APB (size %d)\n", bufsize);

          goto errorout_with_lock;
        }

      src_apb->nmaxbytes = CONFIG_CXD56_AUDIO_BUFFER_SIZE;
      src_apb->nbytes = 0;
      src_apb->samp = (FAR uint8_t *)(&src_apb->samp + 1);
    }
  else
    {
      src_apb = (struct ap_buffer_s *) dq_get(g_src.inq);
    }

  src_apb->flags = 0;

errorout_with_lock:
  spin_unlock_irqrestore(NULL, flags);
  return src_apb;
}

/* Apply SRC on incoming APB and add one or more APBs to the
 * out queue accordingly.
 */

static int cxd56_src_process(FAR struct ap_buffer_s *apb)
{
  int ret = OK;
  irqstate_t flags;
  struct ap_buffer_s *src_apb;

  /* audinfo("SRC: Process (size = %d)\n", apb->nbytes); */

#ifdef DUMP_DATA
  file_write(&dump_file_pre,
        (char *) (apb->samp + apb->curbyte),
        apb->nbytes - apb->curbyte);
#endif

  /* Special case of one-to-one ratio */

  if (g_src.src_data.src_ratio == 1.0f)
    {
      src_apb = cxd56_src_get_apb();
      if (!src_apb)
        {
          ret = -ENOMEM;
          goto exit;
        }

      memcpy(src_apb->samp, apb->samp, apb->nbytes);
      src_apb->nbytes = apb->nbytes;
      src_apb->flags |= AUDIO_APB_SRC_FINAL;

      flags = spin_lock_irqsave(NULL);
      dq_put(g_src.outq, &src_apb->dq_entry);
      spin_unlock_irqrestore(NULL, flags);

      goto exit;
    }

  /* Perform SRC on new buffer and left overs from previous ones */

  while (apb->curbyte < apb->nbytes)
    {
      int float_in_left;
      int frames_in;

      const short *apb_addr = (const short *)(apb->samp + apb->curbyte);

      /* Fill up incoming float buffer */

      float_in_left = BUFFER_SAMPLES - g_src.float_in_offset;

      src_short_to_float_array(apb_addr,
                               (g_src.float_in + g_src.float_in_offset),
                               float_in_left);
      g_src.src_data.output_frames = BUFFER_SAMPLES / g_src.channels;
      g_src.src_data.input_frames = BUFFER_SAMPLES / g_src.channels;

      /* Incoming data larger than ingoing float buffer? */

      frames_in = (apb->nbytes - apb->curbyte) / g_src.bytewidth;

      if (frames_in >= float_in_left || g_src.state == CXD56_SRC_STOPPING)
        {
          int apb_nframes;
          int apb_nmaxframes;
          int src_nframes;
          int src_copyframes;

          float *float_out_src;
          short *src_apb_dest;

          /* Run SRC */

          g_src.src_data.data_out = g_src.float_out;
          g_src.src_data.data_in = g_src.float_in;

          ret = src_process(g_src.src_state, &g_src.src_data);
          if (ret != 0)
            {
              auderr("ERROR: SRC failed (\"%s\")\n", src_strerror(ret));
            }

          /* Move unused data to start of float_in for next round */

          g_src.float_in_offset =
              g_src.src_data.input_frames_used * g_src.channels;
          memcpy((void *)g_src.float_in,
                 (void *)(g_src.float_in + g_src.float_in_offset),
                 (BUFFER_SAMPLES - g_src.float_in_offset) * sizeof(float));

          g_src.float_in_offset = BUFFER_SAMPLES - g_src.float_in_offset;

          /* Prepare apb to dma */

          src_apb = cxd56_src_get_apb();
          if (!src_apb)
            {
              ret = -ENOMEM;
              goto exit;
            }

          apb_nframes =
              src_apb->nbytes / g_src.bytewidth / g_src.channels;
          apb_nmaxframes =
              src_apb->nmaxbytes / g_src.bytewidth / g_src.channels;

          src_nframes = g_src.src_data.output_frames_gen;
          src_copyframes = apb_nmaxframes - apb_nframes;

          /* Generated frames will exceed apb size left */

          if (apb_nframes + src_nframes >= apb_nmaxframes
              || g_src.state == CXD56_SRC_STOPPING)
            {
              /* Convert SRC float data into APB to be sent */

              float_out_src = g_src.float_out;
              src_apb_dest = (short *)(src_apb->samp + src_apb->nbytes);

              src_float_to_short_array(float_out_src, src_apb_dest,
                                       src_copyframes * g_src.channels);
              src_nframes -= src_copyframes;
              src_apb->nbytes = src_apb->nmaxbytes;

              /* Increase SRC buffer processing counter */

              g_src.buf_count += g_src.buf_increment;
              if (g_src.buf_count > 1.0f)
                {
                  src_apb->flags |= AUDIO_APB_SRC_FINAL;
                  g_src.buf_count -= 1.0f;
                }

              /* Put in out queue to be DMA'd */

              flags = spin_lock_irqsave(NULL);
              dq_put(g_src.outq, &src_apb->dq_entry);
              spin_unlock_irqrestore(NULL, flags);

#ifdef DUMP_DATA
              file_write(&dump_file_post, src_apb->samp, src_apb->nbytes);
#endif

              /* Fetch the next APB to fill up */

              src_apb = cxd56_src_get_apb();
              if (!src_apb)
                {
                  ret = -ENOMEM;
                  goto exit;
                }

              apb_nframes =
                  src_apb->nbytes / g_src.bytewidth / g_src.channels;
            }

          /* Convert remaining SRC float data into next APB */

          float_out_src = g_src.float_out + src_copyframes * g_src.channels;
          src_apb_dest = (short *)(src_apb->samp + src_apb->nbytes);

          src_float_to_short_array(float_out_src, src_apb_dest,
                                   src_nframes * g_src.channels);

          src_apb->nbytes += g_src.bytewidth * src_nframes * g_src.channels;

          flags = spin_lock_irqsave(NULL);
          dq_put_back(g_src.inq, &src_apb->dq_entry);
          spin_unlock_irqrestore(NULL, flags);

          apb->curbyte += (float_in_left * g_src.bytewidth);
        }
      else
        {
          g_src.float_in_offset += frames_in;
          apb->curbyte = apb->nbytes - 1;

          break;
        }
    }

exit:
  return ret;
}

/* SRC control and processing thread */

static void *cxd56_src_thread(pthread_addr_t pvarg)
{
  struct audio_msg_s msg;
  unsigned int prio;
  int ret;
  int size;

  audinfo("SRC: Thread started\n");

  g_src.state = CXD56_SRC_RUNNING;

  while (g_src.state == CXD56_SRC_RUNNING)
    {
      size = file_mq_receive(&g_src.mq, (FAR char *)&msg,
                             sizeof(msg), &prio);

      /* Handle the case when we return with no message */

      if (size == 0)
        {
          audinfo("SRC: Zero message, stop\n");
          g_src.state = CXD56_SRC_STOPPED;
          break;
        }

      /* Process the message */

      switch (msg.msg_id)
        {
          case AUDIO_MSG_START:
            break;
          case AUDIO_MSG_STOP:
            g_src.state = CXD56_SRC_STOPPED;
            break;
          case AUDIO_MSG_ENQUEUE:
            ret = cxd56_src_process(msg.u.ptr);
            if (ret != OK)
              {
                auderr("ERROR: SRC processing failed (%d)\n", ret);
                g_src.state = CXD56_SRC_STOPPED;
              }
            break;
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_src_init
 *
 * Description: Initializes the SRC using audio settings set in the given
 *              audio device. When SRC is running the resulting buffers will
 *              be put into the queue "outq" for playback, and after it has
 *              been played it is expected to be put in the "inq" queue to be
 *              filled up with new data.
 *
 ****************************************************************************/

int cxd56_src_init(FAR struct cxd56_dev_s *dev,
                   FAR struct dq_queue_s *inq,
                   FAR struct dq_queue_s *outq)
{
  struct sched_param sparam;
  struct mq_attr m_attr;
  pthread_attr_t t_attr;
  void *value;
  int error;
  int ret = OK;

  g_src.buf_count = 0.0f;
  if (dev->samplerate < 48000)
    {
      g_src.buf_increment = dev->samplerate / 48000.0f;
    }

  g_src.inq = inq;
  g_src.outq = outq;
  g_src.bytewidth = dev->bitwidth / 8;
  g_src.channels = dev->channels;
  g_src.float_in_offset = 0;
  snprintf(g_src.mqname, sizeof(g_src.mqname), "/tmp/%X",
           (unsigned int) &g_src);

  audinfo("SRC: Init (rate = %d, channels = %d, width = %d)\n",
          dev->samplerate, g_src.channels, g_src.bytewidth);

  m_attr.mq_maxmsg  = 16;
  m_attr.mq_msgsize = sizeof(struct audio_msg_s);
  m_attr.mq_curmsgs = 0;
  m_attr.mq_flags   = 0;

  ret = file_mq_open(&g_src.mq, g_src.mqname,
                     O_RDWR | O_CREAT, 0644, &m_attr);
  if (ret < 0)
    {
      auderr("ERROR: Could not allocate SRC message queue.\n");
      return ret;
    }

#ifdef DUMP_DATA
  nx_unlink(dump_name_pre);
  nx_unlink(dump_name_post);
  file_open(&dump_file_pre, dump_name_pre, O_WRONLY | O_CREAT | O_APPEND);
  file_open(&dump_file_post, dump_name_post, O_WRONLY | O_CREAT | O_APPEND);
#endif

  /* Join any old worker threads to prevent memory leaks */

  if (g_src.threadid != 0)
    {
      pthread_join(g_src.threadid, &value);
    }

  pthread_attr_init(&t_attr);
  sparam.sched_priority = sched_get_priority_max(SCHED_FIFO) - 3;
  pthread_attr_setschedparam(&t_attr, &sparam);
  pthread_attr_setstacksize(&t_attr,
                            CONFIG_CXD56_AUDIO_SRC_STACKSIZE);

  ret = pthread_create(&g_src.threadid, &t_attr, cxd56_src_thread,
                       (pthread_addr_t)&g_src);
  if (ret != OK)
    {
      auderr("ERROR: SRC pthread_create failed (%d)\n", ret);
      return ret;
    }

  pthread_setname_np(g_src.threadid, "cxd56_src");

  /* Initialize sample rate converter */

  g_src.src_data.src_ratio = (double) (48000.0f / dev->samplerate);
  if (g_src.src_data.src_ratio == 1.0f)
    {
      audinfo("SRC in and out rate is the same, will copy only.\n");
    }

  g_src.src_state = src_new(SRC_LINEAR, g_src.channels, &error);
  if (g_src.src_state == NULL)
    {
      auderr("ERROR: Could not initialize SRC (%s)\n", src_strerror(error));
      ret = error;
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_src_deinit
 *
 * Description: Releases the SRC instance and related resources.
 *
 ****************************************************************************/

int cxd56_src_deinit(void)
{
  struct ap_buffer_s *src_apb;

  audinfo("SRC: Deinit\n");

  /* Free SRC buffers */

  while (dq_count(g_src.inq))
    {
      src_apb = (struct ap_buffer_s *) dq_get(g_src.inq);
      kmm_free(src_apb);
    }

  while (dq_count(g_src.outq))
    {
      src_apb = (struct ap_buffer_s *) dq_get(g_src.outq);
      kmm_free(src_apb);
    }

  src_delete(g_src.src_state);

#ifdef DUMP_DATA
  if (dump_file_pre.f_inode)
    file_close(&dump_file_pre);

  if (dump_file_post.f_inode)
    file_close(&dump_file_post);
#endif

  file_mq_close(&g_src.mq);

  return OK;
}

/****************************************************************************
 * Name: cxd56_src_enqueue
 *
 * Description: Enqueues a audio buffer for SRC processing. The result will
 *              be put in the outgoing queue given during initialization.
 *
 ****************************************************************************/

int cxd56_src_enqueue(FAR struct ap_buffer_s *apb)
{
  int ret;
  struct audio_msg_s msg;

  audinfo("SRC: Enqueue %x\n", (unsigned int) apb);

  msg.msg_id = AUDIO_MSG_ENQUEUE;
  msg.u.ptr = apb;
  ret = file_mq_send(&g_src.mq, (FAR const char *)&msg,
                     sizeof(msg), CONFIG_CXD56_MSG_PRIO);
  if (ret != OK)
    {
      auderr("ERROR: SRC APB enqueue failed (%d)\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: cxd56_src_stop
 *
 * Description: Stops the SRC processing thread.
 *
 ****************************************************************************/

int cxd56_src_stop(void)
{
  int ret;
  void *value;
  struct audio_msg_s msg;

  audinfo("SRC: Stop\n");

  msg.msg_id = AUDIO_MSG_STOP;
  msg.u.data = 0;
  ret = file_mq_send(&g_src.mq, (FAR const char *)&msg,
                     sizeof(msg), CONFIG_CXD56_MSG_PRIO);
  if (ret != OK)
    {
      auderr("ERROR: SRC stop failed (%d)\n", ret);
    }

  pthread_join(g_src.threadid, &value);
  g_src.threadid = 0;

  return ret;
}
