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

#include <ctype.h>
#include <errno.h>
#include <string.h>

#include <nuttx/irq.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/wqueue.h>
#include <nuttx/compiler.h>

#include "syslog.h"
#include "syslog_rpmsg.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define SYSLOG_RPMSG_WORK_DELAY         MSEC2TICK(CONFIG_SYSLOG_RPMSG_WORK_DELAY)

#define SYSLOG_RPMSG_COUNT(h, t, size)  (((h)>=(t)) ? (h)-(t) : (size)-((t)-(h)))
#define SYSLOG_RPMSG_SPACE(h, t, size)  ((size) - 1 - SYSLOG_RPMSG_COUNT(h, t, size))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct syslog_rpmsg_s
{
  volatile size_t       head;         /* The head index (where data is added) */
  volatile size_t       tail;         /* The tail index (where data is removed) */
  size_t                size;         /* Size of the RAM buffer */
  FAR char              *buffer;      /* Circular RAM buffer */
  struct work_s         work;         /* Used for deferred callback work */

  struct rpmsg_endpoint ept;
  bool                  suspend;
  bool                  transfer;     /* The transfer flag */
  ssize_t               trans_len;    /* The data length when transfer */

  sem_t                 sem;
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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct syslog_rpmsg_s g_syslog_rpmsg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void syslog_rpmsg_work(FAR void *priv_)
{
  FAR struct syslog_rpmsg_transfer_s *msg = NULL;
  FAR struct syslog_rpmsg_s *priv = priv_;
  irqstate_t flags;
  uint32_t space;
  size_t len;
  size_t len_end;

  if (is_rpmsg_ept_ready(&priv->ept))
    {
      msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, false);
    }

  if (!msg)
    {
      work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv,
                 SYSLOG_RPMSG_WORK_DELAY);
      return;
    }

  memset(msg, 0, sizeof(*msg));

  flags = enter_critical_section();

  space  -= sizeof(*msg);
  len     = SYSLOG_RPMSG_COUNT(priv->head, priv->tail, priv->size);
  len_end = priv->size - priv->tail;

  if (len > space)
    {
      len = space;
    }

  if (len > len_end)
    {
      memcpy(msg->data, &priv->buffer[priv->tail], len_end);
      memcpy(msg->data + len_end, priv->buffer, len - len_end);
    }
  else
    {
      memcpy(msg->data, &priv->buffer[priv->tail], len);
    }

  priv->trans_len = len;
  priv->transfer  = true;

  leave_critical_section(flags);

  msg->header.command = SYSLOG_RPMSG_TRANSFER;
  msg->count          = len;
  rpmsg_send_nocopy(&priv->ept, msg, sizeof(*msg) + len);
}

static void syslog_rpmsg_putchar(FAR struct syslog_rpmsg_s *priv, int ch,
                                 bool last)
{
  size_t next;

  while (1)
    {
      next = priv->head + 1;
      if (next >= priv->size)
        {
          next = 0;
        }

      if (next == priv->tail)
        {
#ifndef CONFIG_SYSLOG_RPMSG_OVERWRITE
          if (!up_interrupt_context() && !sched_idletask())
            {
              nxsem_wait(&priv->sem);
            }
          else
#endif
            {
              /* Overwrite */

              priv->buffer[priv->tail] = 0;
              priv->tail += 1;

              if (priv->tail >= priv->size)
                {
                  priv->tail = 0;
                }

              if (priv->transfer)
                {
                  priv->trans_len--;
                }

              break;
            }
        }
      else
        {
          break;
        }
    }

  priv->buffer[priv->head] = ch & 0xff;
  priv->head = next;

  if (last && !priv->suspend && !priv->transfer &&
          is_rpmsg_ept_ready(&priv->ept))
    {
      clock_t delay = SYSLOG_RPMSG_WORK_DELAY;
      size_t space = SYSLOG_RPMSG_SPACE(priv->head, priv->tail, priv->size);

      /* Start work immediately when data more then 75% and meet '\n' */

      if (space < priv->size / 4 && ch == '\n')
        {
          delay = 0;
        }

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
          work_queue(HPWORK, &priv->work,
                     syslog_rpmsg_work, priv, SYSLOG_RPMSG_WORK_DELAY);
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
      work_queue(HPWORK, &priv->work,
        syslog_rpmsg_work, priv, SYSLOG_RPMSG_WORK_DELAY);
    }
  else if (header->command == SYSLOG_RPMSG_TRANSFER_DONE)
    {
      irqstate_t flags;
      ssize_t len_end;
      int sval;

      flags = enter_critical_section();

      if (priv->trans_len > 0)
        {
          len_end = priv->size - priv->tail;

          if (priv->trans_len > len_end)
            {
              memset(&priv->buffer[priv->tail], 0, len_end);
              memset(priv->buffer, 0, priv->trans_len - len_end);
            }
          else
            {
              memset(&priv->buffer[priv->tail], 0, priv->trans_len);
            }

          priv->tail += priv->trans_len;
          if (priv->tail >= priv->size)
            {
              priv->tail -= priv->size;
            }

          nxsem_get_value(&priv->sem, &sval);
          while (sval++ < 0)
            {
              nxsem_post(&priv->sem);
            }
        }

      priv->transfer = false;

      if (SYSLOG_RPMSG_COUNT(priv->head, priv->tail, priv->size))
        {
          work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, 0);
        }

      leave_critical_section(flags);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int syslog_rpmsg_putc(FAR struct syslog_channel_s *channel, int ch)
{
  FAR struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;
  irqstate_t flags;

  UNUSED(channel);

  flags = enter_critical_section();
  syslog_rpmsg_putchar(priv, ch, true);
  leave_critical_section(flags);

  return ch;
}

int syslog_rpmsg_flush(FAR struct syslog_channel_s *channel)
{
  UNUSED(channel);

  FAR struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;

  work_queue(HPWORK, &priv->work, syslog_rpmsg_work, priv, 0);
  return OK;
}

ssize_t syslog_rpmsg_write(FAR struct syslog_channel_s *channel,
                           FAR const char *buffer, size_t buflen)
{
  FAR struct syslog_rpmsg_s *priv = &g_syslog_rpmsg;
  irqstate_t flags;
  size_t nwritten;

  UNUSED(channel);

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

  nxsem_init(&priv->sem, 0, 0);
  nxsem_set_protocol(&priv->sem, SEM_PRIO_NONE);

  priv->buffer  = buffer;
  priv->size    = size;

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
}

int syslog_rpmsg_init(void)
{
  return rpmsg_register_callback(&g_syslog_rpmsg,
                                 syslog_rpmsg_device_created,
                                 syslog_rpmsg_device_destroy,
                                 NULL);
}
