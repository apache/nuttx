/****************************************************************************
 * drivers/rpmsg/rpmsg_wakelock.c
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

#include <inttypes.h>
#include <sys/param.h>
#include <string.h>

#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/power/pm.h>
#include <nuttx/rpmsg/rpmsg.h>

#include "rpmsg_wakelock.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RPMSG_WAKELOCK_NAME         "rpmsg-wakelock"

#define RPMSG_WAKELOCK_ACQUIRE_REQ  1
#define RPMSG_WAKELOCK_RELEASE_REQ  2

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct rpmsg_wakelock_msg_s
{
  uint32_t cmd;
  uint32_t len;
  int32_t  err;
  uint8_t  data[1];
} end_packed_struct;

struct rpmsg_wakelock_node_s
{
  struct pm_wakelock_s wakelock;          /* wakelock */
  struct list_node     node;              /* list node */
};

struct rpmsg_wakelock_s
{
  FAR struct rpmsg_endpoint  *ept;        /* Rpmsg endpoint */
  mutex_t                    lock;        /* List lock */
  struct list_node           list;        /* List node */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Functions handle the responses from the remote cpu */

static int rpmsg_wakelock_acquire_req_handler(FAR struct rpmsg_endpoint *ept,
                                              FAR void *data, size_t len,
                                              uint32_t src, FAR void *priv);

static int rpmsg_wakelock_release_req_handler(FAR struct rpmsg_endpoint *ept,
                                              FAR void *data, size_t len,
                                              uint32_t src, FAR void *priv);

/* Functions for creating communication with remote cpu */

static int rpmsg_wakelock_ept_cb(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len, uint32_t src,
                                 FAR void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_rpmsg_wakelock_handler[] =
{
  [RPMSG_WAKELOCK_ACQUIRE_REQ] = rpmsg_wakelock_acquire_req_handler,
  [RPMSG_WAKELOCK_RELEASE_REQ] = rpmsg_wakelock_release_req_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rpmsg_wakelock_control(FAR struct rpmsg_wakelock_s *priv,
                                  FAR const char *name, bool acquire)
{
  FAR struct rpmsg_wakelock_node_s *tmp;
  FAR struct rpmsg_wakelock_node_s *wl = NULL;

  nxmutex_lock(&priv->lock);

  list_for_every_entry(&priv->list, tmp,
                       struct rpmsg_wakelock_node_s, node)
    {
      if (strcmp(tmp->wakelock.name, name) == 0)
        {
           wl = tmp;
           break;
        }
    }

  if (acquire)
    {
      if (wl == NULL)
        {
          wl = kmm_zalloc(sizeof(*wl));
          if (wl == NULL)
            {
              rpmsgerr("rpmsg_wakelock_ctl_wakelock: kmm_zalloc failed\n");
              nxmutex_unlock(&priv->lock);
              return -ENOMEM;
            }

          rpmsginfo("wakelock created: %s\n", name);
          pm_wakelock_init(&wl->wakelock, name, PM_IDLE_DOMAIN, PM_NORMAL);
          list_add_tail(&priv->list, &wl->node);
        }

      rpmsginfo("acquire wake lock: %s\n", name);
      pm_wakelock_stay(&wl->wakelock);
    }
  else
    {
      if (wl != NULL)
        {
          rpmsginfo("release wake lock: %s\n", name);
          pm_wakelock_relax(&wl->wakelock);
          if (pm_wakelock_staycount(&wl->wakelock) == 0)
            {
              rpmsginfo("wakelock destroyed: %s\n", name);
              list_delete(&wl->node);
              pm_wakelock_uninit(&wl->wakelock);
              kmm_free(wl);
            }
        }
    }

  nxmutex_unlock(&priv->lock);
  return 0;
}

/****************************************************************************
 * Name: rpmsg_wakelock_acquire_req_handler
 *
 * Description:
 *   rpmsg-wakelock acquire req handler.
 *
 * Parameters:
 *   ept  - The rpmsg endpoint
 *   data - The return message
 *   len  - The return message length
 *
 * Returned Values:
 *   Always OK
 *
 ****************************************************************************/

static int rpmsg_wakelock_acquire_req_handler(FAR struct rpmsg_endpoint *ept,
                                              FAR void *data, size_t len,
                                              uint32_t src, FAR void *priv_)
{
  FAR struct rpmsg_wakelock_s *priv = priv_;
  FAR struct rpmsg_wakelock_msg_s *msg = data;
  FAR const char *name = (FAR const char *)msg->data;

  msg->err = rpmsg_wakelock_control(priv, name, true);
  return 0;
}

/****************************************************************************
 * Name: rpmsg_wakelock_release_req_handler
 *
 * Description:
 *   rpmsg-wakelock release req handler.
 *
 * Parameters:
 *   ept  - The rpmsg endpoint
 *   data - The return message
 *   len  - The return message length
 *
 * Returned Values:
 *   Always OK
 *
 ****************************************************************************/

static int rpmsg_wakelock_release_req_handler(FAR struct rpmsg_endpoint *ept,
                                              FAR void *data, size_t len,
                                              uint32_t src, FAR void *priv_)
{
  FAR struct rpmsg_wakelock_s *priv = ept->priv;
  FAR struct rpmsg_wakelock_msg_s *msg = data;
  FAR const char *name = (FAR const char *)msg->data;

  msg->err = rpmsg_wakelock_control(priv, name, false);
  return 0;
}

/****************************************************************************
 * Name: rpmsg_wakelock_ept_cb
 *
 * Description:
 *   Rpmsg device end point callback function, this function will be called
 *   when receive the remote cpu message.
 *
 * Parameters:
 *   ept  - The rpmsg-device end point
 *   data - The received data
 *   len  - The received data length
 *   src  - The source address of the message
 *   priv - Private data of the endpoint
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsg_wakelock_ept_cb(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len, uint32_t src,
                                 FAR void *priv)
{
  FAR struct rpmsg_wakelock_msg_s *msg = data;
  uint32_t command;

  /* The name must end with a NUL inside the received message */

  if (len < sizeof(*msg) ||
      memchr(msg->data, '\0', len - offsetof(struct rpmsg_wakelock_msg_s,
                                             data)) == NULL)
    {
      rpmsgerr("rpmsg_wakelock_ept_cb: bad message, len %zu\n", len);
      return -EINVAL;
    }

  command = msg->cmd;
  if (command < nitems(g_rpmsg_wakelock_handler) &&
      g_rpmsg_wakelock_handler[command] != NULL)
    {
      return g_rpmsg_wakelock_handler[command](ept, data, len, src, priv);
    }

  rpmsgerr("rpmsg_wakelock_ept_cb: invalid command: %" PRIu32 "\n",
           command);
  return -EINVAL;
}

/****************************************************************************
 * Name: rpmsg_wakelock
 *
 * Description:
 *   rpmsg-wakelock ioctl operation
 *
 * Parameters:
 *   ept    - The rpmsg endpoint
 *   cmd    - the ioctl command
 *   arg    - the ioctl argument
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsg_wakelock(FAR struct rpmsg_endpoint *ept,
                   int cmd, unsigned long arg)
{
  FAR struct rpmsg_wakelock_s *priv = ept->priv;
  FAR const char *str = (FAR const char *)arg;
  FAR struct rpmsg_wakelock_msg_s *msg;
  uint32_t msglen;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* pm_wakelock_init() keeps a shorter name only, which the peer could
   * not match on release.
   */

  if (strlen(str) >= sizeof(((FAR struct pm_wakelock_s *)0)->name))
    {
      return -ENAMETOOLONG;
    }

  msg = rpmsg_get_tx_payload_buffer(ept, &msglen, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  msglen = MIN(msglen, sizeof(*msg) + strlen(str));

  if (cmd == RPMSGIOC_ACQUIRE_WAKELOCK)
    {
      msg->cmd = RPMSG_WAKELOCK_ACQUIRE_REQ;
    }
  else if (cmd == RPMSGIOC_RELEASE_WAKELOCK)
    {
      msg->cmd = RPMSG_WAKELOCK_RELEASE_REQ;
    }
  else
    {
      rpmsgerr("rpmsg_wakelock_ioctl: invalid command: %d\n", cmd);
      rpmsg_release_tx_buffer(ept, msg);
      return -EINVAL;
    }

  memcpy(msg->data, str, msglen - sizeof(*msg) + 1);

  msg->len = msglen;
  msg->err = 0;

  ret = rpmsg_send_nocopy(priv->ept, msg, msglen);
  if (ret < 0)
    {
      rpmsgerr("rpmsg_wakelock_ioctl: rpmsg_send_nocopy failed: %d\n", ret);
      rpmsg_release_tx_buffer(priv->ept, msg);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: rpmsg_wakelock_init
 *
 * Description:
 *   Using remote wakelock to wake/sleep CPU
 *
 * Parameters:
 *   rdev - rpmsg device
 *   ept  - rpmsg endpoint
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 ****************************************************************************/

int rpmsg_wakelock_init(FAR struct rpmsg_device *rdev,
                        FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_wakelock_s *priv;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      rpmsgerr("rpmsg_wakelock_initialize: kmm_zalloc failed\n");
      return -ENOMEM;
    }

  /* Initialize rpmsg device */

  nxmutex_init(&priv->lock);
  list_initialize(&priv->list);

  priv->ept = ept;
  ept->priv = priv;

  return rpmsg_create_ept(ept, rdev, RPMSG_WAKELOCK_NAME,
                          RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                          rpmsg_wakelock_ept_cb, NULL);
}

/****************************************************************************
 * Name: rpmsg_wakelock_deinit
 *
 * Description:
 *   rpmsg wakelock deinit
 *
 * Parameters:
 *   ept  - rpmsg endpoint
 *
 * Returned Values:
 *   None
 ****************************************************************************/

void rpmsg_wakelock_deinit(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_wakelock_s *priv = ept->priv;
  FAR struct rpmsg_wakelock_node_s *wl;
  FAR struct rpmsg_wakelock_node_s *tmp;

  rpmsg_destroy_ept(ept);

  /* Drop what a peer that went away still holds, or this CPU never
   * sleeps again.
   */

  list_for_every_entry_safe(&priv->list, wl, tmp,
                            struct rpmsg_wakelock_node_s, node)
    {
      list_delete(&wl->node);
      pm_wakelock_uninit(&wl->wakelock);
      kmm_free(wl);
    }

  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
}
