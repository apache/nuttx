/****************************************************************************
 * drivers/syslog/syslog_rpmsg.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <sys/boardctl.h>
#include <syslog.h>

#ifdef CONFIG_ARCH_LOWPUTC
#include <nuttx/arch.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/syslog/syslog_rpmsg.h>

#include "syslog_rpmsg.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#if CONFIG_SYSLOG_RPMSG_WORK_DELAY
#  define SYSLOG_RPMSG_WORK_DELAY MSEC2TICK(CONFIG_SYSLOG_RPMSG_WORK_DELAY)
#else
#  define SYSLOG_RPMSG_WORK_DELAY MSEC2TICK(100)
#endif

#define SYSLOG_RPMSG_COUNT(p)       ((p)->head >= (p)->tail ? \
                                     (p)->head - (p)->tail : \
                                     (p)->head + (p)->size - (p)->tail)
#define SYSLOG_RPMSG_SPACE(p)       ((p)->size - 1 - SYSLOG_RPMSG_COUNT(p))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void syslog_rpmsg_work(FAR void *priv_);
static void syslog_rpmsg_addbuf(FAR struct syslog_rpmsg_s *priv,
                                FAR const char *buffer, size_t len);
static void syslog_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv_);
static void syslog_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                        FAR void *priv_);
static int  syslog_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len, uint32_t src,
                                FAR void *priv_);
#ifdef CONFIG_SYSLOG_RPMSG_CHARDEV
static ssize_t syslog_rpmsg_file_read(FAR struct file *filep,
                                      FAR char *buffer, size_t len);
static ssize_t syslog_rpmsg_file_write(FAR struct file *filep,
                                       FAR const char *buffer, size_t len);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_SYSLOG_RPMSG_BUFSIZE > 0
#  ifdef SYSLOG_RPMSG_BUFFER_SECTION
static char g_sysbuffer[CONFIG_SYSLOG_RPMSG_BUFSIZE]
                       locate_data(CONFIG_SYSLOG_RPMSG_BUFFER_SECTION);
#  else
static char g_sysbuffer[CONFIG_SYSLOG_RPMSG_BUFSIZE];
#  endif

struct syslog_rpmsg_s g_syslog_rpmsg =
{
  .header = (FAR struct syslog_rpmsg_buffer_header_s *)g_sysbuffer,
  .lock = SP_UNLOCKED,
};
#else
struct syslog_rpmsg_s g_syslog_rpmsg =
{
  .lock = SP_UNLOCKED,
};
#endif

#ifdef CONFIG_SYSLOG_RPMSG_CHARDEV
static const struct file_operations g_syslog_rpmsgfops =
{
  NULL,                    /* open */
  NULL,                    /* close */
  syslog_rpmsg_file_read,  /* read */
  syslog_rpmsg_file_write, /* write */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void syslog_rpmsg_init_buffer(FAR struct syslog_rpmsg_s *priv)
{
  FAR struct syslog_rpmsg_buffer_header_s *header = priv->header;
  irqstate_t flags;

  DEBUGASSERT(priv->header != NULL);

  flags = spin_lock_irqsave(&priv->lock);
  if (header->magic != SYSLOG_RPMSG_MAGIC_NUMBER)
    {
      header->head = 0;
      header->tail = 0;

#if CONFIG_SYSLOG_RPMSG_BUFSIZE > 0
      header->size = CONFIG_SYSLOG_RPMSG_BUFSIZE -
                     sizeof(struct syslog_rpmsg_buffer_header_s);
#endif

      header->magic = SYSLOG_RPMSG_MAGIC_NUMBER;
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

static bool syslog_rpmsg_transfer(FAR struct syslog_rpmsg_s *priv, bool wait)
{
  FAR struct syslog_rpmsg_transfer_s *msg = NULL;
  FAR struct syslog_rpmsg_buffer_header_s *header = priv->header;
  irqstate_t flags;
  uint32_t space;
  uint32_t len_send;
  uint32_t len_end;
  uint32_t len;

  if (!is_rpmsg_ept_ready(&priv->ept))
    {
      return false;
    }

  do
    {
      msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, wait);
      if (!msg)
        {
          return false;
        }

      memset(msg, 0, sizeof(*msg));

      flags = spin_lock_irqsave(&priv->lock);

      space  -= sizeof(*msg);
      len     = SYSLOG_RPMSG_COUNT(header);
      len_end = header->size - header->tail;

      if (len > space)
        {
          len = space;
        }

      if (len > len_end)
        {
          memcpy(msg->data, &header->buffer[header->tail], len_end);
          memcpy(msg->data + len_end, header->buffer, len - len_end);
          memset(&header->buffer[header->tail], 0, len_end);
          memset(header->buffer, 0, len - len_end);
        }
      else
        {
          memcpy(msg->data, &header->buffer[header->tail], len);
          memset(&header->buffer[header->tail], 0, len);
        }

      msg->count          = len;
      header->tail        = (header->tail + len) % header->size;
      msg->header.command = SYSLOG_RPMSG_TRANSFER;
      len_send            = len;
      len = SYSLOG_RPMSG_COUNT(header);
      spin_unlock_irqrestore(&priv->lock, flags);

      if (rpmsg_send_nocopy(&priv->ept, msg, sizeof(*msg) + len_send) < 0)
        {
          rpmsg_release_tx_buffer(&priv->ept, msg);
        }
    }
  while (len > 0);

  return true;
}

static void syslog_rpmsg_work(FAR void *priv_)
{
  FAR struct syslog_rpmsg_s *priv = priv_;

  if (!syslog_rpmsg_transfer(priv, false))
    {
      work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv,
                 SYSLOG_RPMSG_WORK_DELAY);
    }
}

static void syslog_rpmsg_addbuf(FAR struct syslog_rpmsg_s *priv,
                                FAR const char *buffer, size_t len)
{
  FAR struct syslog_rpmsg_buffer_header_s *header;
  bool overwritten = false;
  irqstate_t flags;
  uint32_t tail;

  if (len <= 0)
    {
      return;
    }

  syslog_rpmsg_init_buffer(priv);

  header = priv->header;
  flags = spin_lock_irqsave(&priv->lock);

  if (len > header->size)
    {
      buffer += len - header->size;
      len = header->size;
    }

  if (SYSLOG_RPMSG_SPACE(header) < len)
    {
      bool ret = false;

      if (!up_interrupt_context() && !sched_idletask())
        {
          /* Unlock before entering syslog_rpmsg_transfer,
           * it is already locked internally
           */

          spin_unlock_irqrestore(&priv->lock, flags);
          ret = syslog_rpmsg_transfer(priv, true);
          flags = spin_lock_irqsave(&priv->lock);
        }

      if (!ret)
        {
          overwritten = true;
        }
    }

  tail = header->size - header->head;

  if (len > tail)
    {
      memcpy(&header->buffer[header->head], buffer, tail);
      memcpy(header->buffer, buffer + tail, len - tail);
    }
  else
    {
      memcpy(&header->buffer[header->head], buffer, len);
    }

  header->head = (header->head + len) % header->size;
  if (overwritten)
    {
      header->tail = header->head;
    }

  if (!priv->suspend && is_rpmsg_ept_ready(&priv->ept))
    {
      clock_t delay = SYSLOG_RPMSG_WORK_DELAY;
      uint32_t space = SYSLOG_RPMSG_SPACE(header);

      /* Start work immediately when data more then 75% and meet last */

      if (space < header->size / 4)
        {
          delay = 0;
        }
#if CONFIG_SYSLOG_RPMSG_WORK_DELAY == 0
      else
        {
          spin_unlock_irqrestore(&priv->lock, flags);
          return;
        }
#endif

      spin_unlock_irqrestore(&priv->lock, flags);
      work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, delay);
    }
  else
    {
      spin_unlock_irqrestore(&priv->lock, flags);
    }
}

static void syslog_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv_)
{
  FAR struct syslog_rpmsg_s *priv = priv_;
  int ret;

  if (priv->header && strcmp(CONFIG_SYSLOG_RPMSG_SERVER_NAME,
                             rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = priv;

      ret = rpmsg_create_ept(&priv->ept, rdev, SYSLOG_RPMSG_EPT_NAME,
                             RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                             syslog_rpmsg_ept_cb, NULL);
      if (ret == 0)
        {
          /* The first transfer may run before anything was logged */

          syslog_rpmsg_init_buffer(priv);
          work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, 0);
        }
    }
}

static void syslog_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                        FAR void *priv_)
{
  FAR struct syslog_rpmsg_s *priv = priv_;

  if (priv->header && strcmp(CONFIG_SYSLOG_RPMSG_SERVER_NAME,
                             rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_destroy_ept(&priv->ept);
    }
}

static int syslog_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len, uint32_t src,
                               FAR void *priv_)
{
  FAR struct syslog_rpmsg_s *priv = priv_;
  FAR struct syslog_rpmsg_header_s *header = data;

  if (header->command == SYSLOG_RPMSG_SUSPEND)
    {
      work_cancel(HPWORK, &priv->work);
      priv->suspend = true;
    }
  else if (header->command == SYSLOG_RPMSG_RESUME)
    {
      priv->suspend = false;
      work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, 0);
    }
  else if (header->command == SYSLOG_RPMSG_SYNC)
    {
      syslog_rpmsg_transfer(priv, true);
      rpmsg_send(ept, data, len);
    }

  return 0;
}

#ifdef CONFIG_SYSLOG_RPMSG_CHARDEV
static ssize_t syslog_rpmsg_file_read(FAR struct file *filep,
                                      FAR char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct syslog_rpmsg_s *priv;

  /* Some sanity checking */

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  if (!priv->suspend && is_rpmsg_ept_ready(&priv->ept))
    {
      work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, 0);
    }

  return 0;
}

static ssize_t syslog_rpmsg_file_write(FAR struct file *filep,
                                       FAR const char *buffer, size_t len)
{
  syslog(LOG_INFO, "%.*s", (int)len, buffer);
  return len;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int syslog_rpmsg_putc(FAR syslog_channel_t *channel, int ch)
{
  char tmp = ch;

  UNUSED(channel);

  syslog_rpmsg_addbuf(&g_syslog_rpmsg, &tmp, 1);
  return ch;
}

ssize_t syslog_rpmsg_write(FAR syslog_channel_t *channel,
                           FAR const char *buffer, size_t buflen)
{
  FAR struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;

  syslog_rpmsg_addbuf(priv, buffer, buflen);
  return buflen;
}

int syslog_rpmsg_init(void)
{
#ifdef CONFIG_SYSLOG_RPMSG_CHARDEV
  int ret;

  ret = register_driver(CONFIG_SYSLOG_RPMSG_DEVPATH, &g_syslog_rpmsgfops,
                        0600, &g_syslog_rpmsg);
  if (ret < 0)
    {
      return ret;
    }
#endif

  return rpmsg_register_callback(&g_syslog_rpmsg,
                                 syslog_rpmsg_device_created,
                                 syslog_rpmsg_device_destroy,
                                 NULL,
                                 NULL);
}
