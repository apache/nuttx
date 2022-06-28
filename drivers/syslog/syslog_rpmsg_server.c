/****************************************************************************
 * drivers/syslog/syslog_rpmsg_server.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/syslog/syslog_rpmsg.h>

#include "syslog.h"
#include "syslog_rpmsg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYSLOG_RPMSG_MAXLEN             256

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct syslog_rpmsg_server_s
{
  struct rpmsg_endpoint ept;
  FAR char              *tmpbuf;
  unsigned int          nextpos;
  unsigned int          alloced;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void syslog_rpmsg_write(FAR const char *buf1, size_t len1,
                               FAR const char *buf2, size_t len2);
static void syslog_rpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                                 FAR void *priv_, FAR const char *name,
                                 uint32_t dest);
static void syslog_rpmsg_ns_unbind(FAR struct rpmsg_endpoint *ept);
static int  syslog_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                                FAR void *data, size_t len, uint32_t src,
                                FAR void *priv_);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void syslog_rpmsg_write(FAR const char *buf1, size_t len1,
                               FAR const char *buf2, size_t len2)
{
  FAR const char *nl;
  size_t len;

  nl = memchr(buf2, '\n', len2);
  DEBUGASSERT(nl != NULL);
  len = nl + 1 - buf2;

  if (len1 + len <= SYSLOG_RPMSG_MAXLEN)
    {
      char tmpbuf[SYSLOG_RPMSG_MAXLEN];

      /* Ensure each syslog_write's buffer end with '\n' */

      memcpy(tmpbuf, buf1, len1);
      memcpy(tmpbuf + len1, buf2, len);
      syslog_write(tmpbuf, len1 + len);

      if (len < len2)
        {
          syslog_write(nl + 1, len2 - len);
        }
    }
  else
    {
      /* Give up, the merge buffer is too big */

      syslog_write(buf1, len1);
      syslog_write(buf2, len2);
    }
}

static void syslog_rpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                                 FAR void *priv_, FAR const char *name,
                                 uint32_t dest)
{
  FAR struct syslog_rpmsg_server_s *priv;
  int ret;

  if (strcmp(name, SYSLOG_RPMSG_EPT_NAME))
    {
      return;
    }

  priv = kmm_zalloc(sizeof(struct syslog_rpmsg_server_s));
  if (!priv)
    {
      return;
    }

  priv->ept.priv = priv;

  ret = rpmsg_create_ept(&priv->ept, rdev, SYSLOG_RPMSG_EPT_NAME,
                         RPMSG_ADDR_ANY, dest,
                         syslog_rpmsg_ept_cb, syslog_rpmsg_ns_unbind);
  if (ret)
    {
      kmm_free(priv);
    }
}

static void syslog_rpmsg_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct syslog_rpmsg_server_s *priv = ept->priv;

  if (priv->nextpos)
    {
      syslog_rpmsg_write(priv->tmpbuf, priv->nextpos, "\n", 1);
    }

  rpmsg_destroy_ept(ept);

  kmm_free(priv->tmpbuf);
  kmm_free(priv);
}

static int syslog_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len,
                               uint32_t src, FAR void *priv_)
{
  FAR struct syslog_rpmsg_server_s *priv = priv_;
  FAR struct syslog_rpmsg_header_s *header = data;

  if (header->command == SYSLOG_RPMSG_TRANSFER)
    {
      FAR struct syslog_rpmsg_transfer_s *msg = data;
      unsigned int copied = msg->count;
      unsigned int printed = 0;
      FAR const char *nl;

      nl = memrchr(msg->data, '\n', msg->count);
      if (nl != NULL)
        {
          printed = nl + 1 - msg->data;
          copied = msg->count - printed;

          if (priv->nextpos)
            {
              syslog_rpmsg_write(priv->tmpbuf, priv->nextpos,
                                 msg->data, printed);
              priv->nextpos = 0;
            }
          else
            {
              syslog_write(msg->data, printed);
            }
        }

      if (copied != 0)
        {
          unsigned int newsize = priv->nextpos + copied;
          if (newsize > priv->alloced)
            {
              char *newbuf = kmm_realloc(priv->tmpbuf, newsize);
              if (newbuf != NULL)
                {
                  priv->tmpbuf  = newbuf;
                  priv->alloced = newsize;
                }
              else
                {
                  copied = priv->alloced - priv->nextpos;
                }
            }

          memcpy(priv->tmpbuf + priv->nextpos, msg->data + printed, copied);
          priv->nextpos += copied;
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int syslog_rpmsg_server_init(void)
{
  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 syslog_rpmsg_ns_bind);
}
