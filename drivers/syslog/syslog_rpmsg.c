/****************************************************************************
 * drivers/syslog/syslog_rpmsg.c
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

#ifdef CONFIG_ARCH_LOWPUTC
#include <nuttx/arch.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/wqueue.h>

#include "syslog_rpmsg.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#if CONFIG_SYSLOG_RPMSG_WORK_DELAY
#  define SYSLOG_RPMSG_WORK_DELAY MSEC2TICK(CONFIG_SYSLOG_RPMSG_WORK_DELAY)
#else
#  define SYSLOG_RPMSG_WORK_DELAY MSEC2TICK(100)
#endif

#define SYSLOG_RPMSG_COUNT(p)       ((p)->head - (p)->tail)
#define SYSLOG_RPMSG_SPACE(p)       ((p)->size - 1 - SYSLOG_RPMSG_COUNT(p))
#define SYSLOG_RPMSG_HEADOFF(p)     ((p)->head & ((p)->size -1))
#define SYSLOG_RPMSG_TAILOFF(p)     ((p)->tail & ((p)->size -1))
#define SYSLOG_RPMSG_FLUSHOFF(p)    ((p)->flush & ((p)->size -1))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct syslog_rpmsg_s
{
  volatile size_t       head;       /* The head index (where data is added) */
  volatile size_t       tail;       /* The tail index (where data is removed) */
  volatile size_t       flush;      /* The tail index of flush (where data is removed) */
  size_t                size;       /* Size of the RAM buffer */
  FAR char              *buffer;    /* Circular RAM buffer */
  struct work_s         work;       /* Used for deferred callback work */

  struct rpmsg_endpoint ept;
  bool                  suspend;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void syslog_rpmsg_work(FAR void *priv_);
static void syslog_rpmsg_putchar(FAR struct syslog_rpmsg_s *priv, int ch,
                                 bool last);
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

static struct syslog_rpmsg_s g_syslog_rpmsg;

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

static bool syslog_rpmsg_transfer(FAR struct syslog_rpmsg_s *priv, bool wait)
{
  FAR struct syslog_rpmsg_transfer_s *msg = NULL;
  irqstate_t flags;
  uint32_t space;
  size_t len;
  size_t off;
  size_t len_end;

  do
    {
      msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, wait);
      if (!msg)
        {
          return false;
        }

      memset(msg, 0, sizeof(*msg));

      flags = enter_critical_section();

      space  -= sizeof(*msg);
      len     = SYSLOG_RPMSG_COUNT(priv);
      off     = SYSLOG_RPMSG_TAILOFF(priv);
      len_end = priv->size - off;

      if (len > space)
        {
          len = space;
        }

      if (len > len_end)
        {
          memcpy(msg->data, &priv->buffer[off], len_end);
          memcpy(msg->data + len_end, priv->buffer, len - len_end);
          memset(&priv->buffer[off], 0, len_end);
          memset(priv->buffer, 0, len - len_end);
        }
      else
        {
          memcpy(msg->data, &priv->buffer[off], len);
          memset(&priv->buffer[off], 0, len);
        }

      msg->count          = len;
      priv->tail         += len;
      msg->header.command = SYSLOG_RPMSG_TRANSFER;
      rpmsg_send_nocopy(&priv->ept, msg, sizeof(*msg) + len);
      len                 = SYSLOG_RPMSG_COUNT(priv);

      leave_critical_section(flags);
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

static void syslog_rpmsg_putchar(FAR struct syslog_rpmsg_s *priv, int ch,
                                 bool last)
{
  if (priv->head + 1 - priv->tail >= priv->size)
    {
      if (!priv->flush && !up_interrupt_context() && !sched_idletask())
        {
          syslog_rpmsg_transfer(priv, true);
        }
      else
        {
          /* Overwrite */

          priv->buffer[SYSLOG_RPMSG_TAILOFF(priv)] = 0;
          priv->tail++;
        }
    }

  priv->buffer[SYSLOG_RPMSG_HEADOFF(priv)] = ch & 0xff;
  priv->head++;

  if (priv->flush)
    {
#if defined(CONFIG_ARCH_LOWPUTC)
      up_putc(ch);
#endif
      priv->flush++;
      return;
    }

  if (last && !priv->suspend && is_rpmsg_ept_ready(&priv->ept))
    {
      clock_t delay = SYSLOG_RPMSG_WORK_DELAY;
      size_t space = SYSLOG_RPMSG_SPACE(priv);

      /* Start work immediately when data more then 75% and meet last */

      if (space < priv->size / 4)
        {
          delay = 0;
        }
#if CONFIG_SYSLOG_RPMSG_WORK_DELAY == 0
      else
        {
          return;
        }
#endif

      work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, delay);
    }
}

static void syslog_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv_)
{
  FAR struct syslog_rpmsg_s *priv = priv_;
  int ret;

  if (priv->buffer && strcmp(CONFIG_SYSLOG_RPMSG_SERVER_NAME,
                             rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = priv;

      ret = rpmsg_create_ept(&priv->ept, rdev, SYSLOG_RPMSG_EPT_NAME,
                             RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                             syslog_rpmsg_ept_cb, NULL);
      if (ret == 0)
        {
          work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, 0);
        }
    }
}

static void syslog_rpmsg_device_destroy(FAR struct rpmsg_device *rdev,
                                        FAR void *priv_)
{
  FAR struct syslog_rpmsg_s *priv = priv_;

  if (priv->buffer && strcmp(CONFIG_SYSLOG_RPMSG_SERVER_NAME,
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
  irqstate_t flags;

  /* Some sanity checking */

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct syslog_rpmsg_s *)inode->i_private;

  flags = enter_critical_section();
  if (!priv->suspend && is_rpmsg_ept_ready(&priv->ept))
    {
      work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, 0);
    }

  leave_critical_section(flags);
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

int syslog_rpmsg_putc(FAR struct syslog_channel_s *channel, int ch)
{
  FAR struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;
  irqstate_t flags;

  flags = enter_critical_section();
  syslog_rpmsg_putchar(priv, ch, true);
  leave_critical_section(flags);

  return ch;
}

int syslog_rpmsg_flush(FAR struct syslog_channel_s *channel)
{
  FAR struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;
  irqstate_t flags;

  flags = enter_critical_section();

  if (priv->head - priv->flush > priv->size)
    {
      priv->flush = priv->tail;
    }

  while (priv->flush < priv->head)
    {
#if defined(CONFIG_ARCH_LOWPUTC)
      up_putc(priv->buffer[SYSLOG_RPMSG_FLUSHOFF(priv)]);
#endif
      priv->flush++;
    }

  leave_critical_section(flags);

  return OK;
}

ssize_t syslog_rpmsg_write(FAR struct syslog_channel_s *channel,
                           FAR const char *buffer, size_t buflen)
{
  FAR struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;
  irqstate_t flags;
  size_t nwritten;

  flags = enter_critical_section();
  for (nwritten = 1; nwritten <= buflen; nwritten++)
    {
      syslog_rpmsg_putchar(priv, *buffer++, nwritten == buflen);
    }

  leave_critical_section(flags);

  return buflen;
}

void syslog_rpmsg_init_early(FAR void *buffer, size_t size)
{
  FAR struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;
  char prev;
  char cur;
  size_t i;

  DEBUGASSERT((size & (size - 1)) == 0);

  priv->buffer = buffer;
  priv->size   = size;

  prev = priv->buffer[size - 1];

  for (i = 0; i < size; i++)
    {
      cur = priv->buffer[i];

      if (!isascii(cur))
        {
          memset(priv->buffer, 0, size);
          break;
        }
      else if (prev && !cur)
        {
          priv->head = i;
        }
      else if (!prev && cur)
        {
          priv->tail = i;
        }

      prev = cur;
    }

  if (i != size)
    {
      priv->head = priv->tail = 0;
    }
  else if (priv->head < priv->tail)
    {
      priv->head += priv->size;
    }
}

int syslog_rpmsg_init(void)
{
#ifdef CONFIG_SYSLOG_RPMSG_CHARDEV
  int ret;

  ret = register_driver(CONFIG_SYSLOG_DEVPATH, &g_syslog_rpmsgfops,
                        0666, &g_syslog_rpmsg);
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
