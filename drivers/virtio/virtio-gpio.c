/****************************************************************************
 * drivers/virtio/virtio-gpio.c
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

#include <debug.h>
#include <errno.h>
#include <sys/endian.h>

#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/gpio_virtio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/percpu.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/virtio/virtio-gpio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_GPIO_REQUESTQ 0
#define VIRTIO_GPIO_EVENTQ   1
#define VIRTIO_GPIO_NUM_VQ   2

#define VIRTIO_GPIO_NCB      8
#define VIRTIO_GPIO_MAXDEV   4

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* Beyond 64 pins ioe_pinset_t is a pin number, not a bitmap. */

#  if CONFIG_IOEXPANDER_NPINS > 64
#    error "virtio-gpio requires CONFIG_IOEXPANDER_NPINS <= 64"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
struct virtio_gpio_cb_s
{
  ioe_pinset_t   pinset;
  ioe_callback_t cbfunc;
  FAR void      *cbarg;
};

/* Per-line eventq state: one spec-defined buffer pair per unmasked line,
 * re-armed after each delivered interrupt.
 */

struct virtio_gpio_evline_s
{
  struct virtio_gpio_irq_request_s  req;    /* Device-readable line number */
  struct virtio_gpio_irq_response_s resp;   /* Device-writable irq status */
  uint8_t                           type;   /* VIRTIO_GPIO_IRQ_TYPE_* */
  uint8_t                           dir;    /* VIRTIO_GPIO_DIRECTION_* */
  bool                              queued; /* Pair is on the eventq */
};
#endif

/* Registry of probed frontends, looked up by virtio_gpio_get_ioe(). */

struct virtio_gpio_bus_s
{
  spinlock_t                     lock;
  FAR struct virtio_gpio_priv_s *devs[VIRTIO_GPIO_MAXDEV];
};

struct virtio_gpio_priv_s
{
  /* This must remain the first member: it is cast-compatible with
   * struct ioexpander_dev_s.
   */

  struct ioexpander_dev_s     ioe;

  FAR struct virtio_device   *vdev;
  spinlock_t                  lock;   /* Protect the request virtqueue */
  uint16_t                    ngpio;

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  spinlock_t                  evlock; /* Protect the event virtqueue and cb */
  struct work_s               eventwork;

  /* Per-line eventq buffer pairs in DMA-able memory, allocated once at
   * probe time; NULL when VIRTIO_GPIO_F_IRQ was not negotiated.
   */

  FAR struct virtio_gpio_evline_s *evlines;

  struct virtio_gpio_cb_s     cb[VIRTIO_GPIO_NCB];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int virtio_gpio_direction(FAR struct ioexpander_dev_s *dev,
                                 uint8_t pin, int dir);
static int virtio_gpio_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                              int opt, FAR void *val);
static int virtio_gpio_writepin(FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, bool value);
static int virtio_gpio_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                               FAR bool *value);
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *virtio_gpio_attach(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset,
                                    ioe_callback_t callback, FAR void *arg);
static int virtio_gpio_detach(FAR struct ioexpander_dev_s *dev,
                              FAR void *handle);
#endif

static int  virtio_gpio_probe(FAR struct virtio_device *vdev);
static void virtio_gpio_remove(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DEFINE_PER_CPU_BMP(struct virtio_driver, g_virtio_gpio_driver) =
{
  VIRTIO_ID_GPIO,     /* Device id */
  virtio_gpio_probe,  /* Probe */
  virtio_gpio_remove, /* Remove */
};
#define g_virtio_gpio_driver this_cpu_var_bmp(g_virtio_gpio_driver)

static const struct ioexpander_ops_s g_virtio_gpio_ops =
{
  virtio_gpio_direction,
  virtio_gpio_option,
  virtio_gpio_writepin,
  virtio_gpio_readpin,
  virtio_gpio_readpin,
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  NULL,
  NULL,
  NULL,
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  virtio_gpio_attach,
  virtio_gpio_detach,
#endif
};

/* Lower-halves published to board code, indexed by probe order. */

static DEFINE_PER_CPU_BMP(struct virtio_gpio_bus_s, g_virtio_gpio_bus) =
{
  SP_UNLOCKED,
};
#define g_virtio_gpio_bus this_cpu_var_bmp(g_virtio_gpio_bus)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_gpio_request
 *
 * Description:
 *   Submit one requestq message as a request/response descriptor chain
 *   and wait for completion.  Both headers are stack-allocated and mapped
 *   for the device.
 *
 ****************************************************************************/

static int virtio_gpio_request(FAR struct virtio_gpio_priv_s *priv,
                               uint16_t type, uint16_t gpio, uint32_t value,
                               FAR uint8_t *out)
{
  FAR struct virtio_device *vdev = priv->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[VIRTIO_GPIO_REQUESTQ].vq;
  struct virtio_gpio_request_s req;
  struct virtio_gpio_response_s resp;
  struct virtqueue_buf vb[2];
  FAR void *reqmap;
  FAR void *respmap;
  irqstate_t flags;
  sem_t sem;
  int ret;

  req.type    = htole16(type);
  req.gpio    = htole16(gpio);
  req.value   = htole32(value);
  resp.status = VIRTIO_GPIO_STATUS_ERR;

  ret = virtio_map_buf(vdev, &req, sizeof(req), VIRTIO_DMA_TO_DEVICE,
                       &reqmap);
  if (ret < 0)
    {
      return ret;
    }

  vb[0].buf = reqmap;
  vb[0].len = sizeof(req);

  ret = virtio_map_buf(vdev, &resp, sizeof(resp), VIRTIO_DMA_FROM_DEVICE,
                       &respmap);
  if (ret < 0)
    {
      goto out_req;
    }

  vb[1].buf = respmap;
  vb[1].len = sizeof(resp);

  nxsem_init(&sem, 0, 0);

  /* The lock only guards the ring: requests from different threads may be
   * in flight in parallel, each completion is routed by its sem cookie.
   */

  flags = spin_lock_irqsave(&priv->lock);
  ret = virtqueue_add_buffer(vq, vb, 1, 1, &sem);
  if (ret >= 0)
    {
      virtqueue_kick(vq);
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  if (ret >= 0)
    {
      nxsem_wait_uninterruptible(&sem);
    }

  nxsem_destroy(&sem);

  /* unmap copies the device-written response back before it is read. */

  virtio_unmap_buf(vdev, &resp, sizeof(resp), VIRTIO_DMA_FROM_DEVICE,
                   respmap);
out_req:
  virtio_unmap_buf(vdev, &req, sizeof(req), VIRTIO_DMA_TO_DEVICE, reqmap);

  if (ret < 0)
    {
      return ret;
    }

  if (out != NULL)
    {
      *out = resp.value;
    }

  return resp.status == VIRTIO_GPIO_STATUS_OK ? 0 : -EIO;
}

/****************************************************************************
 * Name: virtio_gpio_req_done
 ****************************************************************************/

static void virtio_gpio_req_done(FAR struct virtqueue *vq)
{
  FAR struct virtio_gpio_priv_s *priv = vq->vq_dev->priv;
  FAR sem_t *sem;

  while ((sem = virtqueue_get_buffer_lock(vq, NULL, NULL,
                                          &priv->lock)) != NULL)
    {
      nxsem_post(sem);
    }
}

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/****************************************************************************
 * Name: virtio_gpio_event_arm
 *
 * Description:
 *   Post the line's buffer pair on the eventq, unmasking it.  Skipped if
 *   the line is masked or its pair is already outstanding.
 *
 ****************************************************************************/

static void virtio_gpio_event_arm(FAR struct virtio_gpio_priv_s *priv,
                                  FAR struct virtio_gpio_evline_s *line)
{
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_GPIO_EVENTQ].vq;
  struct virtqueue_buf vb[2];
  irqstate_t flags;

  vb[0].buf = &line->req;
  vb[0].len = sizeof(line->req);
  vb[1].buf = &line->resp;
  vb[1].len = sizeof(line->resp);

  flags = spin_lock_irqsave(&priv->evlock);
  if (line->type != VIRTIO_GPIO_IRQ_TYPE_NONE && !line->queued)
    {
      line->resp.status = VIRTIO_GPIO_IRQ_STATUS_INVALID;
      if (virtqueue_add_buffer(vq, vb, 1, 1, line) >= 0)
        {
          line->queued = true;
          spin_unlock_irqrestore(&priv->evlock, flags);
          virtqueue_kick_lock(vq, &priv->evlock);
          return;
        }
    }

  spin_unlock_irqrestore(&priv->evlock, flags);
}

/****************************************************************************
 * Name: virtio_gpio_event_dispatch
 *
 * Description:
 *   Invoke the callbacks attached to the delivered line, snapshotted
 *   under evlock so a callback may re-enter this driver.
 *
 ****************************************************************************/

static void virtio_gpio_event_dispatch(FAR struct virtio_gpio_priv_s *priv,
                                       uint16_t gpio)
{
  struct virtio_gpio_cb_s cb[VIRTIO_GPIO_NCB];
  ioe_pinset_t pinset;
  irqstate_t flags;
  int ncb = 0;
  int i;

  if (gpio >= CONFIG_IOEXPANDER_NPINS)
    {
      return;
    }

  pinset = (ioe_pinset_t)1 << gpio;

  flags = spin_lock_irqsave(&priv->evlock);
  for (i = 0; i < VIRTIO_GPIO_NCB; i++)
    {
      if (priv->cb[i].cbfunc != NULL &&
          (priv->cb[i].pinset & pinset) != 0)
        {
          cb[ncb++] = priv->cb[i];
        }
    }

  spin_unlock_irqrestore(&priv->evlock, flags);

  for (i = 0; i < ncb; i++)
    {
      cb[i].cbfunc(&priv->ioe, pinset, cb[i].cbarg);
    }
}

/****************************************************************************
 * Name: virtio_gpio_eventwork
 *
 * Description:
 *   Process eventq completions outside the vring callback (IRQ) context.
 *
 ****************************************************************************/

static void virtio_gpio_eventwork(FAR void *arg)
{
  FAR struct virtio_gpio_priv_s *priv = arg;
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_GPIO_EVENTQ].vq;
  FAR struct virtio_gpio_evline_s *line;
  irqstate_t flags;

  for (; ; )
    {
      /* Clear queued under the same lock hold as the pop, so that a
       * concurrent event_arm() never sees a popped pair still marked
       * queued.
       */

      flags = spin_lock_irqsave(&priv->evlock);
      line = virtqueue_get_buffer(vq, NULL, NULL);
      if (line == NULL)
        {
          spin_unlock_irqrestore(&priv->evlock, flags);
          break;
        }

      line->queued = false;
      spin_unlock_irqrestore(&priv->evlock, flags);

      if (line->resp.status == VIRTIO_GPIO_IRQ_STATUS_VALID)
        {
          virtio_gpio_event_dispatch(priv, le16toh(line->req.gpio));
        }

      /* Re-arm unconditionally: event_arm() skips a masked line, and a
       * line re-enabled before this reap would otherwise be stranded.
       */

      virtio_gpio_event_arm(priv, line);
    }

  virtqueue_enable_cb_lock(vq, &priv->evlock);
}

/****************************************************************************
 * Name: virtio_gpio_eventready
 ****************************************************************************/

static void virtio_gpio_eventready(FAR struct virtqueue *vq)
{
  FAR struct virtio_gpio_priv_s *priv = vq->vq_dev->priv;

  virtqueue_disable_cb_lock(vq, &priv->evlock);
  work_queue(HPWORK, &priv->eventwork, virtio_gpio_eventwork, priv, 0);
}
#endif /* CONFIG_IOEXPANDER_INT_ENABLE */

/****************************************************************************
 * Name: virtio_gpio_direction
 ****************************************************************************/

static int virtio_gpio_direction(FAR struct ioexpander_dev_s *dev,
                                 uint8_t pin, int dir)
{
  FAR struct virtio_gpio_priv_s *priv = (FAR struct virtio_gpio_priv_s *)dev;
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  irqstate_t flags;
#endif
  uint32_t vdir;
  int ret;

  if (pin >= priv->ngpio)
    {
      return -EINVAL;
    }

  vdir = (dir == IOEXPANDER_DIRECTION_IN ||
          dir == IOEXPANDER_DIRECTION_IN_PULLUP ||
          dir == IOEXPANDER_DIRECTION_IN_PULLDOWN) ?
         VIRTIO_GPIO_DIRECTION_IN : VIRTIO_GPIO_DIRECTION_OUT;

  ret = virtio_gpio_request(priv, VIRTIO_GPIO_MSG_SET_DIRECTION, pin,
                            vdir, NULL);

  /* Remembered so that virtio_gpio_option() can honour "IRQ messages MUST
   * NOT be sent for an output line".
   */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  if (ret >= 0 && priv->evlines != NULL)
    {
      flags = spin_lock_irqsave(&priv->evlock);
      priv->evlines[pin].dir = vdir;
      spin_unlock_irqrestore(&priv->evlock, flags);
    }
#endif

  return ret;
}

#ifdef CONFIG_IOEXPANDER_INT_ENABLE

/****************************************************************************
 * Name: virtio_gpio_irq_disable
 *
 * Description:
 *   Drive the trigger type back to NONE, as the spec requires both to
 *   change an active type and once done with the line.
 *
 ****************************************************************************/

static int virtio_gpio_irq_disable(FAR struct virtio_gpio_priv_s *priv,
                                   uint8_t pin)
{
  FAR struct virtio_gpio_evline_s *line = &priv->evlines[pin];
  irqstate_t flags;
  int ret;

  if (line->type == VIRTIO_GPIO_IRQ_TYPE_NONE)
    {
      return OK;
    }

  ret = virtio_gpio_request(priv, VIRTIO_GPIO_MSG_SET_IRQ_TYPE, pin,
                            VIRTIO_GPIO_IRQ_TYPE_NONE, NULL);
  if (ret < 0)
    {
      return ret;
    }

  flags = spin_lock_irqsave(&priv->evlock);
  line->type = VIRTIO_GPIO_IRQ_TYPE_NONE;
  spin_unlock_irqrestore(&priv->evlock, flags);
  return OK;
}

/****************************************************************************
 * Name: virtio_gpio_option
 ****************************************************************************/

static int virtio_gpio_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                              int opt, FAR void *val)
{
  FAR struct virtio_gpio_priv_s *priv = (FAR struct virtio_gpio_priv_s *)dev;
  FAR struct virtio_gpio_evline_s *line;
  irqstate_t flags;
  uint32_t type;
  int ret;

  if (opt != IOEXPANDER_OPTION_INTCFG)
    {
      return -ENOSYS;
    }

  /* Line interrupts need the negotiated VIRTIO_GPIO_F_IRQ eventq. */

  if (priv->evlines == NULL || pin >= priv->ngpio)
    {
      return -ENOTSUP;
    }

  switch ((uintptr_t)val)
    {
      case IOEXPANDER_VAL_RISING:
        type = VIRTIO_GPIO_IRQ_TYPE_EDGE_RISING;
        break;
      case IOEXPANDER_VAL_FALLING:
        type = VIRTIO_GPIO_IRQ_TYPE_EDGE_FALLING;
        break;
      case IOEXPANDER_VAL_BOTH:
        type = VIRTIO_GPIO_IRQ_TYPE_EDGE_BOTH;
        break;
      case IOEXPANDER_VAL_HIGH:
        type = VIRTIO_GPIO_IRQ_TYPE_LEVEL_HIGH;
        break;
      case IOEXPANDER_VAL_LOW:
        type = VIRTIO_GPIO_IRQ_TYPE_LEVEL_LOW;
        break;
      default:
        type = VIRTIO_GPIO_IRQ_TYPE_NONE;
        break;
    }

  line = &priv->evlines[pin];

  /* A trigger type is always disabled before a new one is enabled, as the
   * spec requires for a type change.
   */

  ret = virtio_gpio_irq_disable(priv, pin);
  if (ret < 0 || type == VIRTIO_GPIO_IRQ_TYPE_NONE)
    {
      return ret;
    }

  if (line->dir == VIRTIO_GPIO_DIRECTION_OUT)
    {
      return -EINVAL;
    }

  ret = virtio_gpio_request(priv, VIRTIO_GPIO_MSG_SET_IRQ_TYPE, pin,
                            type, NULL);
  if (ret < 0)
    {
      return ret;
    }

  /* Enabled; arming the eventq pair unmasks it. */

  flags = spin_lock_irqsave(&priv->evlock);
  line->type = type;
  spin_unlock_irqrestore(&priv->evlock, flags);

  virtio_gpio_event_arm(priv, line);
  return OK;
}
#else
static int virtio_gpio_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                              int opt, FAR void *val)
{
  return -ENOSYS;
}
#endif /* CONFIG_IOEXPANDER_INT_ENABLE */

/****************************************************************************
 * Name: virtio_gpio_writepin
 ****************************************************************************/

static int virtio_gpio_writepin(FAR struct ioexpander_dev_s *dev,
                                uint8_t pin, bool value)
{
  FAR struct virtio_gpio_priv_s *priv = (FAR struct virtio_gpio_priv_s *)dev;

  if (pin >= priv->ngpio)
    {
      return -EINVAL;
    }

  return virtio_gpio_request(priv, VIRTIO_GPIO_MSG_SET_VALUE, pin,
                             value, NULL);
}

/****************************************************************************
 * Name: virtio_gpio_readpin
 ****************************************************************************/

static int virtio_gpio_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                               FAR bool *value)
{
  FAR struct virtio_gpio_priv_s *priv = (FAR struct virtio_gpio_priv_s *)dev;
  uint8_t val = 0;
  int ret;

  if (pin >= priv->ngpio)
    {
      return -EINVAL;
    }

  ret = virtio_gpio_request(priv, VIRTIO_GPIO_MSG_GET_VALUE, pin, 0, &val);
  if (ret >= 0)
    {
      *value = val != 0;
    }

  return ret;
}

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/****************************************************************************
 * Name: virtio_gpio_attach
 ****************************************************************************/

static FAR void *virtio_gpio_attach(FAR struct ioexpander_dev_s *dev,
                                    ioe_pinset_t pinset,
                                    ioe_callback_t callback, FAR void *arg)
{
  FAR struct virtio_gpio_priv_s *priv = (FAR struct virtio_gpio_priv_s *)dev;
  FAR void *handle = NULL;
  irqstate_t flags;
  int i;

  if (priv->evlines == NULL)
    {
      return NULL;
    }

  flags = spin_lock_irqsave(&priv->evlock);
  for (i = 0; i < VIRTIO_GPIO_NCB; i++)
    {
      if (priv->cb[i].cbfunc == NULL)
        {
          priv->cb[i].pinset = pinset;
          priv->cb[i].cbarg  = arg;
          priv->cb[i].cbfunc = callback;
          handle             = &priv->cb[i];
          break;
        }
    }

  spin_unlock_irqrestore(&priv->evlock, flags);
  return handle;
}

/****************************************************************************
 * Name: virtio_gpio_detach
 *
 * Description:
 *   Drop the callback and disable every line it was the last subscriber
 *   of, as the spec requires once done with a line.
 *
 ****************************************************************************/

static int virtio_gpio_detach(FAR struct ioexpander_dev_s *dev,
                              FAR void *handle)
{
  FAR struct virtio_gpio_priv_s *priv = (FAR struct virtio_gpio_priv_s *)dev;
  FAR struct virtio_gpio_cb_s *cb = handle;
  ioe_pinset_t orphan;
  irqstate_t flags;
  int i;

  flags = spin_lock_irqsave(&priv->evlock);
  orphan = cb->pinset;
  cb->cbfunc = NULL;
  cb->pinset = 0;

  for (i = 0; i < VIRTIO_GPIO_NCB; i++)
    {
      if (priv->cb[i].cbfunc != NULL)
        {
          orphan &= ~priv->cb[i].pinset;
        }
    }

  spin_unlock_irqrestore(&priv->evlock, flags);

  for (i = 0; i < priv->ngpio && orphan != 0; i++)
    {
      if ((orphan & ((ioe_pinset_t)1 << i)) != 0)
        {
          virtio_gpio_irq_disable(priv, i);
          orphan &= ~((ioe_pinset_t)1 << i);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: virtio_gpio_probe
 ****************************************************************************/

static int virtio_gpio_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_gpio_priv_s *priv;
  FAR const char *vqnames[VIRTIO_GPIO_NUM_VQ];
  vq_callback callback[VIRTIO_GPIO_NUM_VQ];
  irqstate_t flags;
  int nvq = 1;
  int ret;
  int i;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  priv->ioe.ops = &g_virtio_gpio_ops;
  priv->vdev    = vdev;
  vdev->priv    = priv;
  spin_lock_init(&priv->lock);

  /* Without ioexpander interrupt support the driver cannot service line
   * interrupts, so VIRTIO_GPIO_F_IRQ is declined and the device is polled.
   */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  spin_lock_init(&priv->evlock);
  virtio_negotiate_features(vdev, (1UL << VIRTIO_GPIO_F_IRQ), NULL);
#else
  virtio_negotiate_features(vdev, 0, NULL);
#endif
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  virtio_read_config_member(vdev, struct virtio_gpio_config_s, ngpio,
                            &priv->ngpio);
  priv->ngpio = le16toh(priv->ngpio);

  vqnames[VIRTIO_GPIO_REQUESTQ]  = "virtio_gpio_req";
  callback[VIRTIO_GPIO_REQUESTQ] = virtio_gpio_req_done;

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  /* The eventq only exists when the device offers VIRTIO_GPIO_F_IRQ;
   * without it the device degrades to polled line access.
   */

  if (virtio_has_feature(vdev, VIRTIO_GPIO_F_IRQ) && priv->ngpio > 0)
    {
      priv->evlines = virtio_zalloc_buf(vdev, priv->ngpio *
                                        sizeof(*priv->evlines), 16);
      if (priv->evlines == NULL)
        {
          vrterr("No enough memory\n");
          ret = -ENOMEM;
          goto err_with_priv;
        }

      for (i = 0; i < priv->ngpio; i++)
        {
          priv->evlines[i].req.gpio = htole16(i);
        }

      vqnames[VIRTIO_GPIO_EVENTQ]  = "virtio_gpio_event";
      callback[VIRTIO_GPIO_EVENTQ] = virtio_gpio_eventready;
      nvq = VIRTIO_GPIO_NUM_VQ;
    }
#endif

  ret = virtio_create_virtqueues(vdev, 0, nvq, vqnames, callback, NULL);
  if (ret < 0)
    {
      vrterr("virtio_create_virtqueues failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
  virtqueue_enable_cb(vdev->vrings_info[VIRTIO_GPIO_REQUESTQ].vq);

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  /* Eventq pairs are armed on demand when a line interrupt type is
   * configured; nothing is posted here.
   */

  /* No lock: the device is not published yet, nothing else can reach
   * the eventq.
   */

  if (nvq > 1)
    {
      virtqueue_enable_cb(vdev->vrings_info[VIRTIO_GPIO_EVENTQ].vq);
    }
#endif

  /* Publish for virtio_gpio_get_ioe().  Fail rather than come up
   * unreachable.
   */

  flags = spin_lock_irqsave(&g_virtio_gpio_bus.lock);
  for (i = 0; i < VIRTIO_GPIO_MAXDEV; i++)
    {
      if (g_virtio_gpio_bus.devs[i] == NULL)
        {
          g_virtio_gpio_bus.devs[i] = priv;
          break;
        }
    }

  spin_unlock_irqrestore(&g_virtio_gpio_bus.lock, flags);

  if (i >= VIRTIO_GPIO_MAXDEV)
    {
      vrterr("No free slot, increase VIRTIO_GPIO_MAXDEV\n");
      ret = -ENOSPC;
      goto err_with_vq;
    }

  return OK;

err_with_vq:
  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
err_with_priv:
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  if (priv->evlines != NULL)
    {
      virtio_free_buf(vdev, priv->evlines);
    }
#endif

  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: virtio_gpio_remove
 ****************************************************************************/

static void virtio_gpio_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_gpio_priv_s *priv = vdev->priv;
  irqstate_t flags;
  int i;

  flags = spin_lock_irqsave(&g_virtio_gpio_bus.lock);
  for (i = 0; i < VIRTIO_GPIO_MAXDEV; i++)
    {
      if (g_virtio_gpio_bus.devs[i] == priv)
        {
          g_virtio_gpio_bus.devs[i] = NULL;
          break;
        }
    }

  spin_unlock_irqrestore(&g_virtio_gpio_bus.lock, flags);

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  /* eventwork touches what the teardown below frees. */

  work_cancel_sync(HPWORK, &priv->eventwork);
#endif

  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  if (priv->evlines != NULL)
    {
      virtio_free_buf(vdev, priv->evlines);
    }
#endif

  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_gpio_get_ioe
 ****************************************************************************/

FAR struct ioexpander_dev_s *virtio_gpio_get_ioe(int index)
{
  FAR struct ioexpander_dev_s *ioe;
  irqstate_t flags;

  if (index < 0 || index >= VIRTIO_GPIO_MAXDEV)
    {
      return NULL;
    }

  flags = spin_lock_irqsave(&g_virtio_gpio_bus.lock);
  ioe = (FAR struct ioexpander_dev_s *)g_virtio_gpio_bus.devs[index];
  spin_unlock_irqrestore(&g_virtio_gpio_bus.lock, flags);
  return ioe;
}

/****************************************************************************
 * Name: virtio_register_gpio_driver
 ****************************************************************************/

int virtio_register_gpio_driver(void)
{
  return virtio_register_driver(&g_virtio_gpio_driver);
}
