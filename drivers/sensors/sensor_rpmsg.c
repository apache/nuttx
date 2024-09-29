/****************************************************************************
 * drivers/sensors/sensor_rpmsg.c
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

#include <fcntl.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/list.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/rptun/openamp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SENSOR_RPMSG_EPT_NAME      "rpmsg-sensor"
#define SENSOR_RPMSG_ADVERTISE     0
#define SENSOR_RPMSG_ADVERTISE_ACK 1
#define SENSOR_RPMSG_UNADVERTISE   2
#define SENSOR_RPMSG_SUBSCRIBE     3
#define SENSOR_RPMSG_SUBSCRIBE_ACK 4
#define SENSOR_RPMSG_UNSUBSCRIBE   5
#define SENSOR_RPMSG_PUBLISH       6
#define SENSOR_RPMSG_IOCTL         7
#define SENSOR_RPMSG_IOCTL_ACK     8

#define SENSOR_RPMSG_FUNCTION(name, cmd, arg1, arg2, size, wait, type) \
static int sensor_rpmsg_##name(FAR struct sensor_lowerhalf_s *lower, \
                               FAR struct file *filep, \
                               type arg1) \
{ \
  FAR struct sensor_rpmsg_dev_s *dev = lower->priv; \
  FAR struct sensor_lowerhalf_s *drv = dev->drv; \
  int ret; \
\
  if (drv->ops->name) \
    { \
      return drv->ops->name(drv, filep, arg2); \
    } \
  else if (!(filep->f_oflags & SENSOR_REMOTE)) \
    { \
      ret = sensor_rpmsg_ioctl(dev, cmd, arg1, size, wait); \
      return wait ? ret : 0; \
    } \
  else \
    { \
      return wait ? -ENOTSUP : 0; \
    } \
} \

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the context of sensor rpmsg device driver. */

struct sensor_rpmsg_dev_s
{
  struct sensor_lowerhalf_s      lower;
  FAR struct sensor_lowerhalf_s *drv;
  struct list_node               node;
  struct list_node               stublist;
  struct list_node               proxylist;
  uint8_t                        nadvertisers;
  uint8_t                        nsubscribers;
  FAR void                      *upper;
  sensor_push_event_t            push_event;
  char                           path[1];
};

/* This structure describes the context of sensor rpmsg endpoint. */

struct sensor_rpmsg_ept_s
{
  struct list_node               node;
  struct rpmsg_endpoint          ept;
  FAR struct rpmsg_device       *rdev;
  struct work_s                  work;
  rmutex_t                       lock;
  FAR void                      *buffer;
  uint64_t                       expire;
  uint32_t                       space;
  size_t                         written;
};

/* This structure describes the stub info about remote subscribers. */

struct sensor_rpmsg_stub_s
{
  struct list_node               node;
  FAR struct rpmsg_endpoint     *ept;
  uint64_t                       cookie;
  struct file                    file;
  bool                           flushing;
};

/* This structure describes the proxy info about remote advertisers. */

struct sensor_rpmsg_proxy_s
{
  struct list_node               node;
  FAR struct rpmsg_endpoint     *ept;
  uint64_t                       cookie;
};

/* Remote message structure */

/* This structure describes the message about initiating a remote
 * subscription and remote advertisement.
 */

struct sensor_rpmsg_advsub_s
{
  uint32_t                       command;
  uint32_t                       nbuffer;
  uint64_t                       cookie;
  uint32_t                       persist;
  char                           path[1];
};

/* The structure sensor_rpmsg_cell_s describes a data message,
 * include remote receiver, the length of data and the data payload.
 * The structure sensor_rpmsg_data_s describes a set of data message.
 */

struct sensor_rpmsg_cell_s
{
  uint64_t                       cookie;
  uint32_t                       nbuffer;
  uint32_t                       len;
  char                           data[0];
};

struct sensor_rpmsg_data_s
{
  uint32_t                       command;
  uint32_t                       reserved;
  struct sensor_rpmsg_cell_s     cell[0];
};

/* This structure uses to send ioctl from remote device to physical device,
 * it supports pass pointer to remove device by cookie_xx member.
 */

struct sensor_rpmsg_ioctl_cookie_s
{
  sem_t                          sem;
  FAR void                      *data;
  int                            result;
};

struct sensor_rpmsg_ioctl_s
{
  uint32_t                       command;
  int32_t                        result;
  uint64_t                       cookie;
  uint64_t                       proxy;

  int32_t                        request;
  uint32_t                       arglen;
  union
    {
      uint64_t                   arg;
      char                       argbuf[0];
    };
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sensor_rpmsg_open(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep);
static int sensor_rpmsg_close(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep);
static int sensor_rpmsg_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable);
static int sensor_rpmsg_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR uint32_t *period_us);
static int sensor_rpmsg_batch(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR uint32_t *latency_us);
static int sensor_rpmsg_flush(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep);
static int sensor_rpmsg_selftest(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 unsigned long arg);
static int sensor_rpmsg_set_calibvalue(FAR struct sensor_lowerhalf_s *lower,
                                       FAR struct file *filep,
                                       unsigned long arg);
static int sensor_rpmsg_calibrate(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  unsigned long arg);
static int sensor_rpmsg_get_info(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR struct sensor_device_info_s *info);
static int sensor_rpmsg_control(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                int cmd, unsigned long arg);
static int sensor_rpmsg_adv_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv);
static int sensor_rpmsg_advack_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);
static int sensor_rpmsg_unadv_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);
static int sensor_rpmsg_sub_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv);
static int sensor_rpmsg_suback_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);
static int sensor_rpmsg_unsub_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);
static int sensor_rpmsg_publish_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv);
static int sensor_rpmsg_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);
static int sensor_rpmsg_ioctlack_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_rpmsg_ops =
{
  .open           = sensor_rpmsg_open,
  .close          = sensor_rpmsg_close,
  .activate       = sensor_rpmsg_activate,
  .set_interval   = sensor_rpmsg_set_interval,
  .batch          = sensor_rpmsg_batch,
  .flush          = sensor_rpmsg_flush,
  .selftest       = sensor_rpmsg_selftest,
  .set_calibvalue = sensor_rpmsg_set_calibvalue,
  .calibrate      = sensor_rpmsg_calibrate,
  .get_info       = sensor_rpmsg_get_info,
  .control        = sensor_rpmsg_control
};

static const rpmsg_ept_cb g_sensor_rpmsg_handler[] =
{
  [SENSOR_RPMSG_ADVERTISE]     = sensor_rpmsg_adv_handler,
  [SENSOR_RPMSG_ADVERTISE_ACK] = sensor_rpmsg_advack_handler,
  [SENSOR_RPMSG_UNADVERTISE]   = sensor_rpmsg_unadv_handler,
  [SENSOR_RPMSG_SUBSCRIBE]     = sensor_rpmsg_sub_handler,
  [SENSOR_RPMSG_SUBSCRIBE_ACK] = sensor_rpmsg_suback_handler,
  [SENSOR_RPMSG_UNSUBSCRIBE]   = sensor_rpmsg_unsub_handler,
  [SENSOR_RPMSG_PUBLISH]       = sensor_rpmsg_publish_handler,
  [SENSOR_RPMSG_IOCTL]         = sensor_rpmsg_ioctl_handler,
  [SENSOR_RPMSG_IOCTL_ACK]     = sensor_rpmsg_ioctlack_handler,
};

static struct list_node g_devlist = LIST_INITIAL_VALUE(g_devlist);
static struct list_node g_eptlist = LIST_INITIAL_VALUE(g_eptlist);
static rmutex_t g_ept_lock = NXRMUTEX_INITIALIZER;
static rmutex_t g_dev_lock = NXRMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sensor_rpmsg_lock(FAR struct sensor_rpmsg_dev_s *dev)
{
  dev->lower.sensor_lock(dev->upper);
}

static void sensor_rpmsg_unlock(FAR struct sensor_rpmsg_dev_s *dev)
{
  dev->lower.sensor_unlock(dev->upper);
}

static void sensor_rpmsg_advsub_one(FAR struct sensor_rpmsg_dev_s *dev,
                                    FAR struct rpmsg_endpoint *ept,
                                    int command)
{
  FAR struct sensor_rpmsg_advsub_s *msg;
  uint32_t space;
  int len = strlen(dev->path) + 1;
  int ret;

  msg = rpmsg_get_tx_payload_buffer(ept, &space, true);
  if (!msg)
    {
      snerr("ERROR: advsub:%d get buffer failed:%s, %s\n",
            command, dev->path, rpmsg_get_cpuname(ept->rdev));
      return;
    }

  msg->command = command;
  msg->cookie  = (uint64_t)(uintptr_t)dev;
  msg->nbuffer = dev->lower.nbuffer;
  msg->persist = dev->lower.persist;
  memcpy(msg->path, dev->path, len);
  ret = rpmsg_send_nocopy(ept, msg, sizeof(*msg) + len);
  if (ret < 0)
    {
      snerr("ERROR: advsub:%d rpmsg send failed:%s, %d, %s\n",
            command, dev->path, ret, rpmsg_get_cpuname(ept->rdev));
    }
}

static void sensor_rpmsg_advsub(FAR struct sensor_rpmsg_dev_s *dev,
                                int command)
{
  FAR struct sensor_rpmsg_ept_s *sre;

  /* Broadcast advertise/subscribe message to all ready ept */

  nxrmutex_lock(&g_ept_lock);
  list_for_every_entry(&g_eptlist, sre, struct sensor_rpmsg_ept_s,
                       node)
    {
      sensor_rpmsg_advsub_one(dev, &sre->ept, command);
    }

  nxrmutex_unlock(&g_ept_lock);
}

static int sensor_rpmsg_ioctl(FAR struct sensor_rpmsg_dev_s *dev,
                              int cmd, unsigned long arg, size_t len,
                              bool wait)
{
  struct sensor_rpmsg_ioctl_cookie_s cookie;
  FAR struct sensor_rpmsg_proxy_s *proxy;
  FAR struct sensor_rpmsg_proxy_s *ptmp;
  FAR struct sensor_rpmsg_ioctl_s *msg;
  FAR struct rpmsg_endpoint *ept;
  uint64_t pcookie;
  uint32_t space;
  int ret = -ENOTTY;

  if (wait)
    {
      cookie.data   = (FAR void *)(uintptr_t)arg;
      cookie.result = -ENXIO;
      nxsem_init(&cookie.sem, 0, 0);
    }

  /* All control is always send to own proxy(remote advertisers),
   * if device doesn't have proxy, it must return -ENOTTY.
   */

  sensor_rpmsg_lock(dev);
  list_for_every_entry_safe(&dev->proxylist, proxy, ptmp,
                            struct sensor_rpmsg_proxy_s, node)
    {
      ept = proxy->ept;
      pcookie = proxy->cookie;
      msg = rpmsg_get_tx_payload_buffer(ept, &space, true);
      if (!msg)
        {
          ret = -ENOMEM;
          snerr("ERROR: ioctl get buffer failed:%s, %s\n",
                dev->path, rpmsg_get_cpuname(ept->rdev));
          break;
        }

      msg->command = SENSOR_RPMSG_IOCTL;
      msg->cookie  = wait ? (uint64_t)(uintptr_t)&cookie : 0;
      msg->proxy   = pcookie;
      msg->request = cmd;
      msg->arglen  = len;
      if (len > 0)
        {
          memcpy(msg->argbuf, (FAR void *)(uintptr_t)arg, len);
        }
      else
        {
          msg->arg = arg;
        }

      ret = rpmsg_send_nocopy(ept, msg, sizeof(*msg) + len);
      if (ret < 0)
        {
          snerr("ERROR: ioctl rpmsg send failed:%s, %d, %s\n",
                dev->path, ret, rpmsg_get_cpuname(ept->rdev));
          break;
        }

      if (!wait)
        {
          continue;
        }

      sensor_rpmsg_unlock(dev);
      ret = rpmsg_wait(ept, &cookie.sem);
      sensor_rpmsg_lock(dev);
      if (ret < 0)
        {
          snerr("ERROR: ioctl rpmsg wait failed:%s, %d, %s\n",
                dev->path, ret, rpmsg_get_cpuname(ept->rdev));
          break;
        }

      ret = cookie.result;
      if (ret < 0 && ret != -ENOTTY)
        {
          break;
        }
    }

  sensor_rpmsg_unlock(dev);
  if (wait)
    {
      nxsem_destroy(&cookie.sem);
    }

  return ret;
}

static FAR struct sensor_rpmsg_proxy_s *
sensor_rpmsg_alloc_proxy(FAR struct sensor_rpmsg_dev_s *dev,
                         FAR struct rpmsg_endpoint *ept,
                         FAR struct sensor_rpmsg_advsub_s *msg)
{
  FAR struct sensor_rpmsg_proxy_s *proxy;
  struct sensor_state_s state;
  struct file file;
  int ret;

  sensor_rpmsg_lock(dev);
  list_for_every_entry(&dev->proxylist, proxy,
                       struct sensor_rpmsg_proxy_s, node)
    {
      if (proxy->ept == ept && proxy->cookie == msg->cookie)
        {
          sensor_rpmsg_unlock(dev);
          return proxy;
        }
    }

  sensor_rpmsg_unlock(dev);

  /* Create new proxy to represent a remote advertiser */

  proxy = kmm_malloc(sizeof(*proxy));
  if (!proxy)
    {
      return NULL;
    }

  proxy->ept = ept;
  proxy->cookie = msg->cookie;
  ret = file_open(&file, dev->path, SENSOR_REMOTE | O_CLOEXEC);
  if (ret < 0)
    {
      kmm_free(proxy);
      return NULL;
    }

  file_ioctl(&file, SNIOC_SET_BUFFER_NUMBER, msg->nbuffer);
  file_ioctl(&file, SNIOC_GET_STATE, &state);
  file_close(&file);

  sensor_rpmsg_lock(dev);
  if (msg->persist)
    {
      dev->drv->persist = true;
      dev->lower.persist = true;
    }

  list_add_tail(&dev->proxylist, &proxy->node);
  sensor_rpmsg_unlock(dev);

  /* sync interval and latency */

  if (state.min_interval != UINT32_MAX)
    {
      sensor_rpmsg_ioctl(dev, SNIOC_SET_INTERVAL, state.min_interval,
                         0, false);
    }

  if (state.min_latency != UINT32_MAX)
    {
      sensor_rpmsg_ioctl(dev, SNIOC_BATCH, state.min_latency, 0, false);
    }

  return proxy;
}

static
void sensor_rpmsg_push_event_persist(FAR struct sensor_rpmsg_dev_s *dev,
                                     FAR struct sensor_rpmsg_stub_s *stub)
{
  FAR struct sensor_rpmsg_cell_s *cell;
  FAR struct sensor_rpmsg_data_s *msg;
  FAR struct sensor_rpmsg_ept_s *sre;
  uint32_t space;
  int ret;

  sre = container_of(stub->ept, struct sensor_rpmsg_ept_s, ept);
  msg = rpmsg_get_tx_payload_buffer(&sre->ept, &space, true);
  if (!msg)
    {
      snerr("ERROR: push event persist get buffer failed:%s\n",
            rpmsg_get_cpuname(sre->ept.rdev));
      return;
    }

  msg->command = SENSOR_RPMSG_PUBLISH;
  cell = (FAR struct sensor_rpmsg_cell_s *)(msg + 1);
  ret = file_read(&stub->file, cell->data, space - sizeof(*msg) -
                  sizeof(*cell));
  if (ret > 0)
    {
      cell->len     = ret;
      cell->cookie  = stub->cookie;
      cell->nbuffer = dev->lower.nbuffer;
      rpmsg_send_nocopy(&sre->ept, msg, sizeof(*msg) +
                        ((sizeof(*cell) + ret + 0x7) & ~0x7));
    }
  else
    {
      rpmsg_release_tx_buffer(&sre->ept, msg);
    }
}

static FAR struct sensor_rpmsg_stub_s *
sensor_rpmsg_alloc_stub(FAR struct sensor_rpmsg_dev_s *dev,
                        FAR struct rpmsg_endpoint *ept,
                        uint64_t cookie)
{
  FAR struct sensor_rpmsg_stub_s *stub;
  int ret;

  sensor_rpmsg_lock(dev);
  list_for_every_entry(&dev->stublist, stub,
                       struct sensor_rpmsg_stub_s, node)
    {
      if (stub->ept == ept && stub->cookie == cookie)
        {
          sensor_rpmsg_unlock(dev);
          return stub;
        }
    }

  sensor_rpmsg_unlock(dev);

  /* Create new stub to represent a remote subscribers */

  stub = kmm_malloc(sizeof(*stub));
  if (!stub)
    {
      return NULL;
    }

  stub->ept = ept;
  stub->cookie = cookie;
  ret = file_open(&stub->file, dev->path,
                  O_RDOK | O_NONBLOCK | O_CLOEXEC | SENSOR_REMOTE);
  if (ret < 0)
    {
      kmm_free(stub);
      return NULL;
    }

  sensor_rpmsg_lock(dev);
  list_add_tail(&dev->stublist, &stub->node);

  if (dev->lower.persist)
    {
      sensor_rpmsg_push_event_persist(dev, stub);
    }

  sensor_rpmsg_unlock(dev);

  return stub;
}

static void sensor_rpmsg_free_proxy(FAR struct sensor_rpmsg_proxy_s *proxy)
{
  list_delete(&proxy->node);
  kmm_free(proxy);
}

static void sensor_rpmsg_free_stub(FAR struct sensor_rpmsg_stub_s *stub)
{
  list_delete(&stub->node);
  file_close(&stub->file);
  kmm_free(stub);
}

static int sensor_rpmsg_open(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep)
{
  FAR struct sensor_rpmsg_dev_s *dev = lower->priv;
  FAR struct sensor_lowerhalf_s *drv = dev->drv;
  int ret;

  if (drv->ops->open)
    {
      ret = drv->ops->open(drv, filep);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (filep->f_oflags & SENSOR_REMOTE)
    {
      return 0;
    }

  sensor_rpmsg_lock(dev);
  if (filep->f_oflags & O_WROK)
    {
      if (dev->nadvertisers++ == 0)
        {
          sensor_rpmsg_advsub(dev, SENSOR_RPMSG_ADVERTISE);
        }
    }

  if (filep->f_oflags & O_RDOK)
    {
      if (dev->nsubscribers++ == 0)
        {
          sensor_rpmsg_advsub(dev, SENSOR_RPMSG_SUBSCRIBE);
        }
    }

  sensor_rpmsg_unlock(dev);
  return 0;
}

static int sensor_rpmsg_close(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep)
{
  FAR struct sensor_rpmsg_dev_s *dev = lower->priv;
  FAR struct sensor_lowerhalf_s *drv = dev->drv;
  FAR struct sensor_rpmsg_proxy_s *proxy;
  FAR struct sensor_rpmsg_proxy_s *ptmp;
  FAR struct sensor_rpmsg_stub_s *stub;
  FAR struct sensor_rpmsg_stub_s *stmp;
  int ret = 0;

  if (drv->ops->close)
    {
      ret = drv->ops->close(drv, filep);
    }

  if (filep->f_oflags & SENSOR_REMOTE)
    {
      return ret;
    }

  sensor_rpmsg_lock(dev);
  if (filep->f_oflags & O_WROK)
    {
      if (dev->nadvertisers == 1)
        {
          sensor_rpmsg_advsub(dev, SENSOR_RPMSG_UNADVERTISE);
          list_for_every_entry_safe(&dev->stublist, stub, stmp,
                                    struct sensor_rpmsg_stub_s, node)
            {
              sensor_rpmsg_free_stub(stub);
            }
        }

      dev->nadvertisers--;
    }

  if (filep->f_oflags & O_RDOK)
    {
      if (dev->nsubscribers == 1)
        {
          sensor_rpmsg_advsub(dev, SENSOR_RPMSG_UNSUBSCRIBE);
          list_for_every_entry_safe(&dev->proxylist, proxy, ptmp,
                                    struct sensor_rpmsg_proxy_s, node)
            {
              sensor_rpmsg_free_proxy(proxy);
            }
        }

      dev->nsubscribers--;
    }

  sensor_rpmsg_unlock(dev);
  return ret;
}

static int sensor_rpmsg_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 bool enable)
{
  FAR struct sensor_rpmsg_dev_s *dev = lower->priv;
  FAR struct sensor_lowerhalf_s *drv = dev->drv;

  if (drv->ops->activate)
    {
      return drv->ops->activate(drv, filep, enable);
    }

  return 0;
}

SENSOR_RPMSG_FUNCTION(set_interval, SNIOC_SET_INTERVAL,
                      *interval, interval, 0, false, uint32_t)
SENSOR_RPMSG_FUNCTION(batch, SNIOC_BATCH, *latency, latency, 0,
                      false, uint32_t)
SENSOR_RPMSG_FUNCTION(selftest, SNIOC_SELFTEST, arg, arg, 0,
                      true, unsigned long)
SENSOR_RPMSG_FUNCTION(set_calibvalue, SNIOC_SET_CALIBVALUE,
                      arg, arg, 256, true, unsigned long)
SENSOR_RPMSG_FUNCTION(calibrate, SNIOC_CALIBRATE, arg, arg, 256,
                      true, unsigned long)

static int sensor_rpmsg_flush(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep)
{
  FAR struct sensor_rpmsg_dev_s *dev = lower->priv;
  FAR struct sensor_lowerhalf_s *drv = dev->drv;
  int ret = -ENOTTY;

  if (drv->ops->flush)
    {
      ret = drv->ops->flush(drv, filep);
    }
  else if (!(filep->f_oflags & SENSOR_REMOTE))
    {
      ret = sensor_rpmsg_ioctl(dev, SNIOC_FLUSH, 0, 0, true);
    }

  return ret;
}

static int sensor_rpmsg_get_info(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR struct sensor_device_info_s *info)
{
  FAR struct sensor_rpmsg_dev_s *dev = lower->priv;
  FAR struct sensor_lowerhalf_s *drv = dev->drv;
  int ret = -ENOTTY;

  if (drv->ops->get_info)
    {
      ret = drv->ops->get_info(drv, filep, info);
    }
  else if (!(filep->f_oflags & SENSOR_REMOTE))
    {
      ret = sensor_rpmsg_ioctl(dev, SNIOC_GET_INFO,
                               (unsigned long)(uintptr_t)info,
                               sizeof(struct sensor_device_info_s), true);
    }

  return ret;
}

static int sensor_rpmsg_control(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                int cmd, unsigned long arg)
{
  FAR struct sensor_rpmsg_dev_s *dev = lower->priv;
  FAR struct sensor_lowerhalf_s *drv = dev->drv;
  FAR struct sensor_ioctl_s *ioctl = (FAR void *)(uintptr_t)arg;

  if (drv->ops->control)
    {
      return drv->ops->control(drv, filep, cmd, arg);
    }
  else if (!(filep->f_oflags & SENSOR_REMOTE) && _SNIOCVALID(cmd))
    {
      return sensor_rpmsg_ioctl(dev, cmd, arg,
                                sizeof(*ioctl) + ioctl->len, true);
    }

  return -ENOTTY;
}

static void sensor_rpmsg_data_worker(FAR void *arg)
{
  FAR struct sensor_rpmsg_ept_s *sre = arg;

  nxrmutex_lock(&sre->lock);
  if (sre->buffer)
    {
      rpmsg_send_nocopy(&sre->ept, sre->buffer, sre->written);
      sre->buffer = NULL;
    }

  nxrmutex_unlock(&sre->lock);
}

static void sensor_rpmsg_push_event_one(FAR struct sensor_rpmsg_dev_s *dev,
                                        FAR struct sensor_rpmsg_stub_s *stub,
                                        bool flushed)
{
  FAR struct sensor_rpmsg_cell_s *cell;
  FAR struct sensor_rpmsg_ept_s *sre;
  FAR struct sensor_rpmsg_data_s *msg;
  struct sensor_ustate_s state;
  uint64_t now;
  bool updated;
  int ret;

  /* Get state of device to do send data with timeout */

  ret = file_ioctl(&stub->file, SNIOC_GET_USTATE, &state);
  if (ret < 0)
    {
      return;
    }

  if (state.interval == UINT32_MAX)
    {
      state.interval = 0;
    }

  sre = container_of(stub->ept, struct sensor_rpmsg_ept_s, ept);
  nxrmutex_lock(&sre->lock);

  /* Cancel work to fill new data to buffer */

  if (sre->buffer)
    {
      work_cancel(HPWORK, &sre->work);
    }

  for (; ; )
    {
      ret = file_ioctl(&stub->file, SNIOC_UPDATED, &updated);
      if (ret < 0 || (!updated && !flushed))
        {
          break;
        }

      /* If buffer isn't created or it doesn't have enough space to fill
       * new data, you should create or send this buffer at once.
       */

      if (!sre->buffer ||
          sre->written + sizeof(*cell) + state.esize > sre->space)
        {
          if (sre->buffer)
            {
              rpmsg_send_nocopy(&sre->ept, sre->buffer, sre->written);
              sre->buffer = NULL;
            }

          msg = rpmsg_get_tx_payload_buffer(&sre->ept, &sre->space, true);
          sre->buffer = msg;
          if (!msg)
            {
              snerr("ERROR: push event get buffer failed:%s\n",
                    rpmsg_get_cpuname(sre->ept.rdev));
              nxrmutex_unlock(&sre->lock);
              return;
            }

          msg->command = SENSOR_RPMSG_PUBLISH;
          sre->written = sizeof(*msg);
          sre->expire  = UINT64_MAX;
        }

      cell = sre->buffer + sre->written;
      if (flushed)
        {
          flushed = false;
          stub->flushing = false;
        }
      else
        {
          ret  = file_read(&stub->file, cell->data,
                           sre->space - sre->written - sizeof(*cell));
          if (ret <= 0)
            {
              break;
            }
        }

      cell->len     = ret;
      cell->cookie  = stub->cookie;
      cell->nbuffer = dev->lower.nbuffer;

      sre->written += (sizeof(*cell) + ret + 0x7) & ~0x7;
    }

  /* If buffer timeout is expired, do rpmsg_send_nocopy, otherwise using
   * delay work to send data.
   */

  now = sensor_get_timestamp();
  if (sre->expire <= now && sre->buffer)
    {
      ret = rpmsg_send_nocopy(&sre->ept, sre->buffer, sre->written);
      sre->buffer = NULL;
      if (ret < 0)
        {
          snerr("ERROR: push event rpmsg send failed:%d, %s\n",
                ret, rpmsg_get_cpuname(sre->ept.rdev));
        }
    }
  else
    {
      if (sre->expire == UINT64_MAX ||
          sre->expire - now > state.interval / 2)
        {
          sre->expire = now + state.interval / 2;
        }

      work_queue(HPWORK, &sre->work, sensor_rpmsg_data_worker, sre,
                 (sre->expire - now) / USEC_PER_TICK);
    }

  nxrmutex_unlock(&sre->lock);
}

static ssize_t sensor_rpmsg_push_event(FAR void *priv, FAR const void *data,
                                       size_t bytes)
{
  FAR struct sensor_rpmsg_dev_s *dev = priv;
  FAR struct sensor_rpmsg_stub_s *stub;
  FAR struct sensor_rpmsg_stub_s *stmp;
  ssize_t ret;

  /* Push new data to upperhalf driver's circular buffer */

  ret = dev->push_event(dev->upper, data, bytes);
  if (ret < 0)
    {
      return ret;
    }

  /* Send new data to own proxy(remote subscribers), don't care whether
   * is successful, and must return length of written.
   */

  sensor_rpmsg_lock(dev);
  list_for_every_entry_safe(&dev->stublist, stub, stmp,
                            struct sensor_rpmsg_stub_s, node)
    {
      sensor_rpmsg_push_event_one(dev, stub,
                                  stub->flushing && bytes == 0);
    }

  sensor_rpmsg_unlock(dev);
  return ret;
}

static FAR struct sensor_rpmsg_dev_s *
sensor_rpmsg_find_dev(FAR const char *path)
{
  FAR struct sensor_rpmsg_dev_s *dev;

  nxrmutex_lock(&g_dev_lock);
  list_for_every_entry(&g_devlist, dev, struct sensor_rpmsg_dev_s, node)
    {
      if (strcmp(dev->path, path) == 0)
        {
          nxrmutex_unlock(&g_dev_lock);
          return dev;
        }
    }

  nxrmutex_unlock(&g_dev_lock);
  return NULL;
}

static int sensor_rpmsg_adv_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv)
{
  FAR struct sensor_rpmsg_advsub_s *msg = data;
  FAR struct sensor_rpmsg_proxy_s *proxy;
  FAR struct sensor_rpmsg_dev_s *dev;
  int ret;

  dev = sensor_rpmsg_find_dev(msg->path);
  if (!dev || !dev->nsubscribers)
    {
      return 0;
    }

  proxy = sensor_rpmsg_alloc_proxy(dev, ept, msg);
  if (!proxy)
    {
      snerr("ERROR: adv alloc proxy failed:%s\n", dev->path);
    }
  else
    {
      msg->cookie = (uint64_t)(uintptr_t)dev;
      msg->command = SENSOR_RPMSG_ADVERTISE_ACK;
      ret = rpmsg_send(ept, msg, len);
      if (ret < 0)
        {
          sensor_rpmsg_lock(dev);
          sensor_rpmsg_free_proxy(proxy);
          sensor_rpmsg_unlock(dev);
          snerr("ERROR: adv rpmsg send failed:%s, %d, %s\n",
                dev->path, ret, rpmsg_get_cpuname(ept->rdev));
        }
    }

  return 0;
}

static int sensor_rpmsg_advack_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv)
{
  FAR struct sensor_rpmsg_advsub_s *msg = data;
  FAR struct sensor_rpmsg_dev_s *dev;

  dev = sensor_rpmsg_find_dev(msg->path);
  if (dev && !sensor_rpmsg_alloc_stub(dev, ept, msg->cookie))
    {
      sensor_rpmsg_advsub_one(dev, ept, SENSOR_RPMSG_UNADVERTISE);
      snerr("ERROR: advack failed:%s, %s\n", dev->path,
            rpmsg_get_cpuname(ept->rdev));
    }

  return 0;
}

static int sensor_rpmsg_unadv_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv)
{
  FAR struct sensor_rpmsg_advsub_s *msg = data;
  FAR struct sensor_rpmsg_proxy_s *proxy;
  FAR struct sensor_rpmsg_dev_s *dev;

  dev = sensor_rpmsg_find_dev(msg->path);
  if (!dev)
    {
      return 0;
    }

  sensor_rpmsg_lock(dev);
  list_for_every_entry(&dev->proxylist, proxy,
                       struct sensor_rpmsg_proxy_s, node)
    {
      if (proxy->ept == ept && proxy->cookie == msg->cookie)
        {
          sensor_rpmsg_free_proxy(proxy);
          break;
        }
    }

  sensor_rpmsg_unlock(dev);
  return 0;
}

static int sensor_rpmsg_sub_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv)
{
  FAR struct sensor_rpmsg_advsub_s *msg = data;
  FAR struct sensor_rpmsg_dev_s *dev;
  FAR struct sensor_rpmsg_stub_s *stub;
  int ret;

  dev = sensor_rpmsg_find_dev(msg->path);
  if (!dev || (dev->nadvertisers == 0 && !dev->lower.persist))
    {
      return 0;
    }

  stub = sensor_rpmsg_alloc_stub(dev, ept, msg->cookie);
  if (!stub)
    {
      snerr("ERROR: sub alloc stub failed:%s\n", dev->path);
    }
  else
    {
      msg->cookie  = (uint64_t)(uintptr_t)dev;
      msg->command = SENSOR_RPMSG_SUBSCRIBE_ACK;
      msg->nbuffer = dev->lower.nbuffer;
      msg->persist = dev->lower.persist;
      ret = rpmsg_send(ept, msg, len);
      if (ret < 0)
        {
          sensor_rpmsg_lock(dev);
          sensor_rpmsg_free_stub(stub);
          sensor_rpmsg_unlock(dev);
          snerr("ERROR: sub rpmsg send failed:%s, %d, %s\n",
                dev->path, ret, rpmsg_get_cpuname(ept->rdev));
        }
    }

  return 0;
}

static int sensor_rpmsg_suback_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv)
{
  FAR struct sensor_rpmsg_advsub_s *msg = data;
  FAR struct sensor_rpmsg_dev_s *dev;

  dev = sensor_rpmsg_find_dev(msg->path);
  if (dev && (!dev->nsubscribers ||
      !sensor_rpmsg_alloc_proxy(dev, ept, msg)))
    {
      sensor_rpmsg_advsub_one(dev, ept, SENSOR_RPMSG_UNSUBSCRIBE);
      snerr("ERROR: suback failed:%s\n", dev->path);
    }

  return 0;
}

static int sensor_rpmsg_unsub_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv)
{
  FAR struct sensor_rpmsg_advsub_s *msg = data;
  FAR struct sensor_rpmsg_dev_s *dev;
  FAR struct sensor_rpmsg_stub_s *stub;

  dev = sensor_rpmsg_find_dev(msg->path);
  if (!dev)
    {
      return 0;
    }

  sensor_rpmsg_lock(dev);
  list_for_every_entry(&dev->stublist, stub,
                       struct sensor_rpmsg_stub_s, node)
    {
      if (stub->ept == ept && stub->cookie == msg->cookie)
        {
          sensor_rpmsg_free_stub(stub);
          break;
        }
    }

  sensor_rpmsg_unlock(dev);
  return 0;
}

static int sensor_rpmsg_publish_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv)
{
  FAR struct sensor_rpmsg_data_s *msg = data;
  FAR struct sensor_rpmsg_cell_s *cell;
  FAR struct sensor_rpmsg_stub_s *stub;
  FAR struct sensor_rpmsg_stub_s *stmp;
  FAR struct sensor_rpmsg_dev_s *dev;
  size_t written = sizeof(*msg);

  while (written < len)
    {
      cell = (FAR struct sensor_rpmsg_cell_s *)
             ((FAR char *)data + written);
      dev = (FAR struct sensor_rpmsg_dev_s *)(uintptr_t)cell->cookie;
      if (cell->nbuffer > dev->lower.nbuffer)
        {
          struct file file;
          int ret;

          ret = file_open(&file, dev->path, SENSOR_REMOTE | O_CLOEXEC);
          if (ret >= 0)
            {
              ret = file_ioctl(&file, SNIOC_SET_BUFFER_NUMBER,
                               cell->nbuffer);
              if (ret >= 0)
                {
                  dev->lower.nbuffer = cell->nbuffer;
                }

              file_close(&file);
            }
        }

      dev->push_event(dev->upper, cell->data, cell->len);

      /* When the remote core publishes a message, the subscribed cores will
       * receive the message. When the subscribed core publishes a new
       * message, it will take away the message published by the remote core,
       * so all stublist information needs to be updated.
       */

      sensor_rpmsg_lock(dev);
      list_for_every_entry_safe(&dev->stublist, stub, stmp,
                                struct sensor_rpmsg_stub_s, node)
        {
          file_read(&stub->file, NULL, cell->len);
        }

      sensor_rpmsg_unlock(dev);

      written += sizeof(*cell) + cell->len + 0x7;
      written &= ~0x7;
    }

  return 0;
}

static int sensor_rpmsg_ioctl_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv)
{
  FAR struct sensor_rpmsg_ioctl_s *msg = data;
  FAR struct sensor_rpmsg_stub_s *stub;
  FAR struct sensor_rpmsg_stub_s *stmp;
  FAR struct sensor_rpmsg_dev_s *dev;
  unsigned long arg;
  int ret;

  arg = msg->arglen > 0 ? (unsigned long)(uintptr_t)msg->argbuf :
                          msg->arg;
  dev = (FAR struct sensor_rpmsg_dev_s *)(uintptr_t)msg->proxy;
  sensor_rpmsg_lock(dev);
  list_for_every_entry_safe(&dev->stublist, stub, stmp,
                            struct sensor_rpmsg_stub_s, node)
    {
      if (stub->ept == ept)
        {
          msg->result = file_ioctl(&stub->file, msg->request, arg);
          if (msg->result >= 0 && msg->request == SNIOC_FLUSH)
            {
              stub->flushing = true;
            }

          if (msg->cookie)
            {
              msg->command = SENSOR_RPMSG_IOCTL_ACK;
              ret = rpmsg_send(ept, msg, len);
              if (ret < 0)
                {
                  snerr("ERROR: ioctl rpmsg send failed:%s, %d, %s\n",
                        dev->path, ret, rpmsg_get_cpuname(ept->rdev));
                }
            }

          break;
        }
    }

  sensor_rpmsg_unlock(dev);
  return 0;
}

static int sensor_rpmsg_ioctlack_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv)
{
  FAR struct sensor_rpmsg_ioctl_cookie_s *cookie;
  FAR struct sensor_rpmsg_ioctl_s *msg = data;

  cookie = (FAR struct sensor_rpmsg_ioctl_cookie_s *)
           (uintptr_t)msg->cookie;
  cookie->result = msg->result;
  if (msg->result >= 0 && msg->arglen > 0)
    {
      memcpy(cookie->data, msg->argbuf, msg->arglen);
    }

  rpmsg_post(ept, &cookie->sem);
  return 0;
}

static int sensor_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len, uint32_t src,
                               FAR void *priv)
{
  FAR struct sensor_rpmsg_advsub_s *msg = data;

  if (msg->command < sizeof(g_sensor_rpmsg_handler) /
                     sizeof(g_sensor_rpmsg_handler[0]))
    {
      return g_sensor_rpmsg_handler[msg->command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

static void sensor_rpmsg_ns_unbind_cb(FAR struct rpmsg_endpoint *ept)
{
  FAR struct sensor_rpmsg_ept_s *sre;
  FAR struct sensor_rpmsg_dev_s *dev;
  FAR struct sensor_rpmsg_stub_s *stub;
  FAR struct sensor_rpmsg_proxy_s *proxy;

  sre = container_of(ept, struct sensor_rpmsg_ept_s, ept);

  /* Remove all proxy and stub info in sensor device with the ept
   * destoryed.
   */

  nxrmutex_lock(&g_dev_lock);
  list_for_every_entry(&g_devlist, dev,
                       struct sensor_rpmsg_dev_s, node)
    {
      sensor_rpmsg_lock(dev);
      list_for_every_entry(&dev->proxylist, proxy,
                           struct sensor_rpmsg_proxy_s, node)
        {
          if (proxy->ept == ept)
            {
              sensor_rpmsg_free_proxy(proxy);
              break;
            }
        }

      list_for_every_entry(&dev->stublist, stub,
                           struct sensor_rpmsg_stub_s, node)
        {
          if (stub->ept == ept)
            {
              sensor_rpmsg_free_stub(stub);
              break;
            }
        }

      sensor_rpmsg_unlock(dev);
    }

  nxrmutex_unlock(&g_dev_lock);

  nxrmutex_lock(&g_ept_lock);
  if (list_in_list(&sre->node))
    {
      list_delete(&sre->node);
    }

  nxrmutex_unlock(&g_ept_lock);

  nxrmutex_destroy(&sre->lock);
  rpmsg_destroy_ept(ept);
  kmm_free(sre);
}

static void sensor_rpmsg_device_ns_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct sensor_rpmsg_ept_s *sre;
  FAR struct sensor_rpmsg_dev_s *dev;

  sre = container_of(ept, struct sensor_rpmsg_ept_s, ept);

  nxrmutex_lock(&g_ept_lock);
  list_add_tail(&g_eptlist, &sre->node);
  nxrmutex_unlock(&g_ept_lock);

  /* Broadcast all device to ready ept */

  nxrmutex_lock(&g_dev_lock);
  list_for_every_entry(&g_devlist, dev,
                       struct sensor_rpmsg_dev_s, node)
    {
      sensor_rpmsg_lock(dev);
      if (dev->nadvertisers > 0)
        {
          sensor_rpmsg_advsub_one(dev, ept, SENSOR_RPMSG_ADVERTISE);
        }

      if (dev->nsubscribers > 0)
        {
          sensor_rpmsg_advsub_one(dev, ept, SENSOR_RPMSG_SUBSCRIBE);
        }

      sensor_rpmsg_unlock(dev);
    }

  nxrmutex_unlock(&g_dev_lock);
}

static void sensor_rpmsg_device_created(FAR struct rpmsg_device *rdev,
                                        FAR void *priv)
{
  FAR struct sensor_rpmsg_ept_s *sre;

  sre = kmm_zalloc(sizeof(*sre));
  if (!sre)
    {
      return;
    }

  sre->rdev = rdev;
  sre->ept.priv = sre;
  nxrmutex_init(&sre->lock);
  sre->ept.ns_bound_cb = sensor_rpmsg_device_ns_bound;
  if (rpmsg_create_ept(&sre->ept, rdev, SENSOR_RPMSG_EPT_NAME,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       sensor_rpmsg_ept_cb,
                       sensor_rpmsg_ns_unbind_cb) < 0)
    {
      nxrmutex_destroy(&sre->lock);
      kmm_free(sre);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sensor_rpmsg_register
 *
 * Description:
 *   This function registers rpmsg takeover for the real lower half, and
 *   initialize rpmsg resource.
 *
 * Input Parameters:
 *   lower - The instance of lower half sensor driver.
 *   path  - The path of character node, ex: /dev/uorb/xxx.
 *
 * Returned Value:
 *   The takeover rpmsg lowerhalf returned on success, NULL on failure.
 ****************************************************************************/

FAR struct sensor_lowerhalf_s *
sensor_rpmsg_register(FAR struct sensor_lowerhalf_s *lower,
                      FAR const char *path)
{
  FAR struct sensor_rpmsg_ept_s *sre;
  FAR struct sensor_rpmsg_dev_s *dev;
  size_t size;

  if (lower->ops->fetch)
    {
      return lower;
    }

  size = strlen(path);
  dev = kmm_zalloc(sizeof(*dev) + size);
  if (!dev)
    {
      return NULL;
    }

  /* Initialize the sensor rpmsg device structure */

  list_initialize(&dev->stublist);
  list_initialize(&dev->proxylist);
  strlcpy(dev->path, path, size + 1);

  dev->nadvertisers   = !!lower->ops->activate;
  dev->push_event     = lower->push_event;
  dev->upper          = lower->priv;
  lower->push_event   = sensor_rpmsg_push_event;
  lower->priv         = dev;
  memcpy(&dev->lower, lower, sizeof(*lower));
  dev->lower.ops      = &g_sensor_rpmsg_ops;
  dev->drv            = lower;

  /* If openamp is ready, send advertisement to remote proc */

  nxrmutex_lock(&g_dev_lock);
  list_add_tail(&g_devlist, &dev->node);
  nxrmutex_unlock(&g_dev_lock);
  if (lower->ops->activate)
    {
      nxrmutex_lock(&g_ept_lock);
      list_for_every_entry(&g_eptlist, sre, struct sensor_rpmsg_ept_s,
                           node)
        {
          sensor_rpmsg_advsub_one(dev, &sre->ept, SENSOR_RPMSG_ADVERTISE);
        }

      nxrmutex_unlock(&g_ept_lock);
    }

  return &dev->lower;
}

/****************************************************************************
 * Name: sensor_rpmsg_unregister
 *
 * Description:
 *   This function unregisters rpmsg takeover for the real lower half, and
 *   release rpmsg resource. This API corresponds to sensor_rpmsg_register.
 *
 * Input Parameters:
 *   lower - The instance of lower half sensor driver.
 ****************************************************************************/

void sensor_rpmsg_unregister(FAR struct sensor_lowerhalf_s *lower)
{
  FAR struct sensor_rpmsg_dev_s *dev = lower->priv;

  if (lower->ops != &g_sensor_rpmsg_ops)
    {
      return;
    }

  nxrmutex_lock(&g_dev_lock);
  list_delete(&dev->node);
  nxrmutex_unlock(&g_dev_lock);

  kmm_free(dev);
}

/****************************************************************************
 * Name: sensor_rpmsg_initialize
 *
 * Description:
 *   This function initializes the context of sensor rpmsg, registers
 *   rpmsg callback and prepares enviroment to intercat with remote sensor.
 *
 * Returned Value:
 *   OK on success; A negated errno value is returned on any failure.
 ****************************************************************************/

int sensor_rpmsg_initialize(void)
{
  return rpmsg_register_callback(NULL, sensor_rpmsg_device_created,
                                 NULL, NULL, NULL);
}
