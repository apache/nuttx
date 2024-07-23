/****************************************************************************
 * drivers/rpmsg/rpmsg.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/rpmsg/rpmsg.h>

#include "rpmsg_ping.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_bind_s
{
  char              name[RPMSG_NAME_SIZE];
  uint32_t          dest;
  struct metal_list node;
};

struct rpmsg_cb_s
{
  FAR void          *priv;
  rpmsg_dev_cb_t    device_created;
  rpmsg_dev_cb_t    device_destroy;
  rpmsg_match_cb_t  ns_match;
  rpmsg_bind_cb_t   ns_bind;
  struct metal_list node;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rpmsg_dev_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static METAL_DECLARE_LIST(g_rpmsg_cb);
static METAL_DECLARE_LIST(g_rpmsg);

static rmutex_t g_rpmsg_lock = NXRMUTEX_INITIALIZER;

static const struct file_operations g_rpmsg_dev_ops =
{
  NULL,             /* open */
  NULL,             /* close */
  NULL,             /* read */
  NULL,             /* write */
  NULL,             /* seek */
  rpmsg_dev_ioctl,  /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline FAR struct rpmsg_s *
rpmsg_get_by_rdev(FAR struct rpmsg_device *rdev)
{
  if (!rdev)
    {
      return NULL;
    }

  return metal_container_of(rdev, struct rpmsg_s, rdev);
}

static int rpmsg_dev_ioctl_(FAR struct rpmsg_s *rpmsg, int cmd,
                            unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      case RPMSGIOC_PANIC:
        rpmsg->ops->panic(rpmsg);
        break;
      case RPMSGIOC_DUMP:
        rpmsg->ops->dump(rpmsg);
        break;
#ifdef CONFIG_RPMSG_PING
      case RPMSGIOC_PING:
        ret = rpmsg_ping(&rpmsg->ping, (FAR const struct rpmsg_ping_s *)arg);
        break;
#endif
      default:
        ret = rpmsg->ops->ioctl(rpmsg, cmd, arg);
        break;
    }

  return ret;
}

static int rpmsg_dev_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct rpmsg_s *rpmsg = filep->f_inode->i_private;

  return rpmsg_dev_ioctl_(rpmsg, cmd, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rpmsg_wait(FAR struct rpmsg_endpoint *ept, FAR sem_t *sem)
{
  FAR struct rpmsg_s *rpmsg;

  if (!ept || !sem)
    {
      return -EINVAL;
    }

  rpmsg = rpmsg_get_by_rdev(ept->rdev);
  if (!rpmsg || !rpmsg->ops->wait)
    {
      return nxsem_wait_uninterruptible(sem);
    }

  return rpmsg->ops->wait(rpmsg, sem);
}

int rpmsg_post(FAR struct rpmsg_endpoint *ept, FAR sem_t *sem)
{
  FAR struct rpmsg_s *rpmsg;

  if (!ept || !sem)
    {
      return -EINVAL;
    }

  rpmsg = rpmsg_get_by_rdev(ept->rdev);
  if (!rpmsg || !rpmsg->ops->post)
    {
      return nxsem_post(sem);
    }

  return rpmsg->ops->post(rpmsg, sem);
}

FAR const char *rpmsg_get_cpuname(FAR struct rpmsg_device *rdev)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);

  if (!rpmsg)
    {
      return NULL;
    }

  return rpmsg->ops->get_cpuname(rpmsg);
}

int rpmsg_get_tx_buffer_size(FAR struct rpmsg_device *rdev)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);

  if (!rpmsg)
    {
      return -EINVAL;
    }

  return rpmsg->ops->get_tx_buffer_size(rpmsg);
}

int rpmsg_get_rx_buffer_size(FAR struct rpmsg_device *rdev)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);

  if (!rpmsg)
    {
      return -EINVAL;
    }

  return rpmsg->ops->get_rx_buffer_size(rpmsg);
}

int rpmsg_register_callback(FAR void *priv,
                            rpmsg_dev_cb_t device_created,
                            rpmsg_dev_cb_t device_destroy,
                            rpmsg_match_cb_t ns_match,
                            rpmsg_bind_cb_t ns_bind)
{
  FAR struct metal_list *node;
  FAR struct metal_list *bnode;
  FAR struct rpmsg_cb_s *cb;
  FAR struct rpmsg_s *rpmsg;

  cb = kmm_zalloc(sizeof(struct rpmsg_cb_s));
  if (!cb)
    {
      return -ENOMEM;
    }

  cb->priv           = priv;
  cb->device_created = device_created;
  cb->device_destroy = device_destroy;
  cb->ns_match       = ns_match;
  cb->ns_bind        = ns_bind;

  nxrmutex_lock(&g_rpmsg_lock);

  metal_list_for_each(&g_rpmsg, node)
    {
      rpmsg = metal_container_of(node, struct rpmsg_s, node);
      if (!rpmsg->rdev->ns_unbind_cb)
        {
          continue;
        }

      if (device_created)
        {
          device_created(rpmsg->rdev, priv);
        }

      if (ns_bind == NULL)
        {
          continue;
        }

      DEBUGASSERT(ns_match != NULL);
again:

      nxrmutex_lock(&rpmsg->lock);

      metal_list_for_each(&rpmsg->bind, bnode)
        {
          FAR struct rpmsg_bind_s *bind;

          bind = metal_container_of(bnode, struct rpmsg_bind_s, node);
          if (ns_match(rpmsg->rdev, priv, bind->name, bind->dest))
            {
              metal_list_del(bnode);
              nxrmutex_unlock(&rpmsg->lock);
              ns_bind(rpmsg->rdev, priv, bind->name, bind->dest);

              kmm_free(bind);
              goto again;
            }
        }

       nxrmutex_unlock(&rpmsg->lock);
    }

  metal_list_add_tail(&g_rpmsg_cb, &cb->node);
  nxrmutex_unlock(&g_rpmsg_lock);

  return 0;
}

void rpmsg_unregister_callback(FAR void *priv,
                               rpmsg_dev_cb_t device_created,
                               rpmsg_dev_cb_t device_destroy,
                               rpmsg_match_cb_t ns_match,
                               rpmsg_bind_cb_t ns_bind)
{
  FAR struct metal_list *node;
  FAR struct metal_list *pnode;

  nxrmutex_lock(&g_rpmsg_lock);

  metal_list_for_each(&g_rpmsg_cb, node)
    {
      FAR struct rpmsg_cb_s *cb = NULL;

      cb = metal_container_of(node, struct rpmsg_cb_s, node);
      if (cb->priv == priv &&
          cb->device_created == device_created &&
          cb->device_destroy == device_destroy &&
          cb->ns_match == ns_match &&
          cb->ns_bind == ns_bind)
        {
          metal_list_del(&cb->node);
          kmm_free(cb);

          break;
        }
    }

  if (device_destroy)
    {
      metal_list_for_each(&g_rpmsg, pnode)
        {
          FAR struct rpmsg_s *rpmsg;

          rpmsg = metal_container_of(pnode, struct rpmsg_s, node);
          if (rpmsg->rdev->ns_unbind_cb)
            {
              device_destroy(rpmsg->rdev, priv);
            }
        }
    }

  nxrmutex_unlock(&g_rpmsg_lock);
}

void rpmsg_ns_bind(FAR struct rpmsg_device *rdev,
                   FAR const char *name, uint32_t dest)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);
  FAR struct rpmsg_bind_s *bind;
  FAR struct metal_list *node;

  nxrmutex_lock(&g_rpmsg_lock);

  metal_list_for_each(&g_rpmsg_cb, node)
    {
      FAR struct rpmsg_cb_s *cb;

      cb = metal_container_of(node, struct rpmsg_cb_s, node);
      if (cb->ns_match && cb->ns_match(rdev, cb->priv, name, dest))
        {
          rpmsg_bind_cb_t ns_bind = cb->ns_bind;
          FAR void *cb_priv = cb->priv;

          nxrmutex_unlock(&g_rpmsg_lock);
          DEBUGASSERT(ns_bind != NULL);
          ns_bind(rdev, cb_priv, name, dest);

          return;
        }
    }

  nxrmutex_unlock(&g_rpmsg_lock);

  bind = kmm_malloc(sizeof(struct rpmsg_bind_s));
  if (bind == NULL)
    {
      return;
    }

  bind->dest = dest;
  strlcpy(bind->name, name, RPMSG_NAME_SIZE);

  nxrmutex_lock(&rpmsg->lock);
  metal_list_add_tail(&rpmsg->bind, &bind->node);
  nxrmutex_unlock(&rpmsg->lock);
}

void rpmsg_ns_unbind(FAR struct rpmsg_device *rdev,
                     FAR const char *name, uint32_t dest)
{
  FAR struct rpmsg_s *rpmsg = rpmsg_get_by_rdev(rdev);
  FAR struct metal_list *node;

  nxrmutex_lock(&rpmsg->lock);

  metal_list_for_each(&rpmsg->bind, node)
    {
      FAR struct rpmsg_bind_s *bind;

      bind = metal_container_of(node, struct rpmsg_bind_s, node);

      if (bind->dest == dest && !strncmp(bind->name, name, RPMSG_NAME_SIZE))
        {
          metal_list_del(node);
          kmm_free(bind);
          break;
        }
    }

  nxrmutex_unlock(&rpmsg->lock);
}

void rpmsg_device_created(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_cb_s *cb;
  FAR struct metal_list *node;

  nxrmutex_lock(&g_rpmsg_lock);

  metal_list_for_each(&g_rpmsg_cb, node)
    {
      cb = metal_container_of(node, struct rpmsg_cb_s, node);
      if (cb->device_created)
        {
          cb->device_created(rpmsg->rdev, cb->priv);
        }
    }

  nxrmutex_unlock(&g_rpmsg_lock);

#ifdef CONFIG_RPMSG_PING
  rpmsg_ping_init(rpmsg->rdev, &rpmsg->ping);
#endif
}

void rpmsg_device_destory(FAR struct rpmsg_s *rpmsg)
{
  FAR struct rpmsg_cb_s *cb;
  FAR struct metal_list *node;
  FAR struct metal_list *tmp;

#ifdef CONFIG_RPMSG_PING
  rpmsg_ping_deinit(&rpmsg->ping);
#endif

  nxrmutex_lock(&rpmsg->lock);

  metal_list_for_each_safe(&rpmsg->bind, node, tmp)
    {
      FAR struct rpmsg_bind_s *bind;

      bind = metal_container_of(node, struct rpmsg_bind_s, node);
      metal_list_del(node);
      kmm_free(bind);
    }

  nxrmutex_unlock(&rpmsg->lock);

  nxrmutex_lock(&g_rpmsg_lock);

  /* Broadcast device_destroy to all registers */

  metal_list_for_each(&g_rpmsg_cb, node)
    {
      cb = metal_container_of(node, struct rpmsg_cb_s, node);
      if (cb->device_destroy)
        {
          cb->device_destroy(rpmsg->rdev, cb->priv);
        }
    }

  nxrmutex_unlock(&g_rpmsg_lock);
}

int rpmsg_register(FAR const char *path, FAR struct rpmsg_s *rpmsg,
                   FAR const struct rpmsg_ops_s *ops)
{
  int ret;

  ret = register_driver(path, &g_rpmsg_dev_ops, 0222, rpmsg);
  if (ret < 0)
    {
      return ret;
    }

  metal_list_init(&rpmsg->bind);
  nxrmutex_init(&rpmsg->lock);
  rpmsg->ops = ops;

  /* Add priv to list */

  nxrmutex_lock(&g_rpmsg_lock);
  metal_list_add_tail(&g_rpmsg, &rpmsg->node);
  nxrmutex_unlock(&g_rpmsg_lock);

  return ret;
}

void rpmsg_unregister(FAR const char *path, FAR struct rpmsg_s *rpmsg)
{
  nxrmutex_lock(&g_rpmsg_lock);
  metal_list_del(&rpmsg->node);
  nxrmutex_unlock(&g_rpmsg_lock);

  nxrmutex_destroy(&rpmsg->lock);

  unregister_driver(path);
}

int rpmsg_ioctl(FAR const char *cpuname, int cmd, unsigned long arg)
{
  FAR struct metal_list *node;
  int ret = OK;

  nxrmutex_lock(&g_rpmsg_lock);

  metal_list_for_each(&g_rpmsg, node)
    {
      FAR struct rpmsg_s *rpmsg;

      rpmsg = metal_container_of(node, struct rpmsg_s, node);

      if (!cpuname || !strcmp(rpmsg_get_cpuname(rpmsg->rdev), cpuname))
        {
          ret = rpmsg_dev_ioctl_(rpmsg, cmd, arg);
          if (ret < 0)
            {
              break;
            }
        }
    }

  nxrmutex_unlock(&g_rpmsg_lock);
  return ret;
}

int rpmsg_panic(FAR const char *cpuname)
{
  return rpmsg_ioctl(cpuname, RPMSGIOC_PANIC, 0);
}

void rpmsg_dump_all(void)
{
  rpmsg_ioctl(NULL, RPMSGIOC_DUMP, 0);
}
