/****************************************************************************
 * drivers/vhost/vhost-gpio.c
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
#include <sys/param.h>

#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/kmalloc.h>
#include <nuttx/percpu.h>
#include <nuttx/spinlock.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/virtio/virtio-gpio.h>
#include <nuttx/wqueue.h>

#include "vhost-gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VHOST_GPIO_REQUESTQ 0
#define VHOST_GPIO_EVENTQ   1

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
#  define VHOST_GPIO_NUM_VQ 2

/* Beyond 64 pins ioe_pinset_t is a pin number, not a bitmap. */

#  if CONFIG_IOEXPANDER_NPINS > 64
#    error "vhost-gpio requires CONFIG_IOEXPANDER_NPINS <= 64"
#  endif
#else
#  define VHOST_GPIO_NUM_VQ 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* Per-line eventq state: the guest arms one pair per unmasked line,
 * completed with STATUS_VALID when the line fires, INVALID when masked.
 */

struct vhost_gpio_evline_s
{
  bool                                   pending; /* Edge fired while unarmed */
  uint8_t                                type;    /* VIRTIO_GPIO_IRQ_TYPE_* */
  uint16_t                               head;    /* Descriptor chain head */
  FAR struct virtio_gpio_irq_response_s *resp;    /* Guest pair held if set */
};
#endif

struct vhost_gpio_priv_s
{
  FAR struct vhost_device     *hdev;
  FAR struct ioexpander_dev_s *ioe;
  FAR void                    *ihandle;
  struct work_s                work;
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  spinlock_t                   evlock; /* Protect the event virtqueue */
  struct vhost_gpio_evline_s   evlines[CONFIG_IOEXPANDER_NPINS];
#endif

  /* Last direction set by the guest (VIRTIO_GPIO_DIRECTION_*), returned
   * on GET_DIRECTION: the ioexpander interface has no direction query.
   */

  uint8_t                      dir[CONFIG_IOEXPANDER_NPINS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  vhost_gpio_probe(FAR struct vhost_device *hdev);
static void vhost_gpio_remove(FAR struct vhost_device *hdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DEFINE_PER_CPU_BMP(struct vhost_driver, g_vhost_gpio_driver) =
{
  VIRTIO_ID_GPIO,    /* Device id */
  vhost_gpio_probe,  /* Probe */
  vhost_gpio_remove, /* Remove */
};
#define g_vhost_gpio_driver this_cpu_var_bmp(g_vhost_gpio_driver)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/****************************************************************************
 * Name: vhost_gpio_event_handler
 *
 * Description:
 *   Drain the pairs the guest queued on the eventq.  A valid pair is held
 *   until the line fires or gets masked; malformed or duplicate pairs are
 *   completed at once with STATUS_INVALID.
 *
 ****************************************************************************/

static void vhost_gpio_event_handler(FAR struct virtqueue *vq)
{
  FAR struct vhost_gpio_priv_s *priv = vq->vq_dev->priv;
  irqstate_t flags;
  bool kick = false;

  flags = spin_lock_irqsave(&priv->evlock);
  for (; ; )
    {
      FAR struct vhost_gpio_evline_s *line;
      FAR struct virtio_gpio_irq_request_s *ireq;
      FAR struct virtio_gpio_irq_response_s *ires;
      struct virtqueue_buf vb[2];
      size_t vbcnt = 0;
      uint16_t gpio;
      int head;

      head = vhost_get_vq_buffers(vq, vb, nitems(vb), &vbcnt);
      if (head < 0)
        {
          break;
        }

      if (vbcnt < 2 || vb[0].len < sizeof(*ireq) ||
          vb[1].len < sizeof(*ires))
        {
          virtqueue_add_consumed_buffer(vq, head, 0);
          kick = true;
          continue;
        }

      ireq = vb[0].buf;
      ires = vb[1].buf;
      gpio = le16toh(ireq->gpio);

      if (gpio >= CONFIG_IOEXPANDER_NPINS ||
          priv->evlines[gpio].resp != NULL)
        {
          ires->status = VIRTIO_GPIO_IRQ_STATUS_INVALID;
          virtqueue_add_consumed_buffer(vq, head, sizeof(*ires));
          kick = true;
          continue;
        }

      line = &priv->evlines[gpio];
      if (line->pending)
        {
          /* The line fired while unarmed: deliver the latched event. */

          line->pending = false;
          ires->status = VIRTIO_GPIO_IRQ_STATUS_VALID;
          virtqueue_add_consumed_buffer(vq, head, sizeof(*ires));
          kick = true;
          continue;
        }

      line->head = head;
      line->resp = ires;
    }

  if (kick)
    {
      virtqueue_kick(vq);
    }

  spin_unlock_irqrestore(&priv->evlock, flags);
}

/****************************************************************************
 * Name: vhost_gpio_event_flush
 *
 * Description:
 *   Mask a line: complete its armed pair with STATUS_INVALID and drop any
 *   latched event.
 *
 ****************************************************************************/

static void vhost_gpio_event_flush(FAR struct vhost_gpio_priv_s *priv,
                                   uint16_t gpio)
{
  FAR struct virtqueue *vq = priv->hdev->vrings_info[VHOST_GPIO_EVENTQ].vq;
  FAR struct vhost_gpio_evline_s *line = &priv->evlines[gpio];
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->evlock);
  line->pending = false;
  line->type    = VIRTIO_GPIO_IRQ_TYPE_NONE;
  if (line->resp != NULL)
    {
      line->resp->status = VIRTIO_GPIO_IRQ_STATUS_INVALID;
      virtqueue_add_consumed_buffer(vq, line->head, sizeof(*line->resp));
      virtqueue_kick(vq);
      line->resp = NULL;
    }

  spin_unlock_irqrestore(&priv->evlock, flags);
}

/****************************************************************************
 * Name: vhost_gpio_irq_cb
 *
 * Description:
 *   A local line fired: complete its armed pair with STATUS_VALID, else
 *   latch it.  Only enabled edge triggers latch -- the spec forbids
 *   latching level triggers, and a disabled line has no subscriber.
 *
 ****************************************************************************/

static int vhost_gpio_irq_cb(FAR struct ioexpander_dev_s *dev,
                             ioe_pinset_t pinset, FAR void *arg)
{
  FAR struct vhost_gpio_priv_s *priv = arg;
  FAR struct virtqueue *vq = priv->hdev->vrings_info[VHOST_GPIO_EVENTQ].vq;
  irqstate_t flags;
  bool kick = false;
  int pin;

  flags = spin_lock_irqsave(&priv->evlock);
  for (pin = 0; pin < CONFIG_IOEXPANDER_NPINS; pin++)
    {
      FAR struct vhost_gpio_evline_s *line = &priv->evlines[pin];

      if ((pinset & ((ioe_pinset_t)1 << pin)) == 0 ||
          line->type == VIRTIO_GPIO_IRQ_TYPE_NONE)
        {
          continue;
        }

      if (line->resp != NULL)
        {
          line->resp->status = VIRTIO_GPIO_IRQ_STATUS_VALID;
          virtqueue_add_consumed_buffer(vq, line->head,
                                        sizeof(*line->resp));
          line->resp = NULL;
          kick = true;
        }
      else if ((line->type & VIRTIO_GPIO_IRQ_TYPE_EDGE_BOTH) != 0)
        {
          line->pending = true;
        }
    }

  if (kick)
    {
      virtqueue_kick(vq);
    }

  spin_unlock_irqrestore(&priv->evlock, flags);
  return 0;
}
#endif

/****************************************************************************
 * Name: vhost_gpio_dispatch
 ****************************************************************************/

static uint8_t
vhost_gpio_dispatch(FAR struct vhost_gpio_priv_s *priv,
                    FAR const struct virtio_gpio_request_s *req,
                    FAR uint8_t *value)
{
  FAR struct ioexpander_dev_s *ioe = priv->ioe;
  uint16_t gpio = le16toh(req->gpio);
  uint32_t val  = le32toh(req->value);
  int ret = -ENOSYS;

  if (gpio >= CONFIG_IOEXPANDER_NPINS)
    {
      return VIRTIO_GPIO_STATUS_ERR;
    }

  switch (le16toh(req->type))
    {
      case VIRTIO_GPIO_MSG_SET_DIRECTION:
        switch (val)
          {
            case VIRTIO_GPIO_DIRECTION_IN:
              ret = IOEXP_SETDIRECTION(ioe, gpio, IOEXPANDER_DIRECTION_IN);
              break;

            case VIRTIO_GPIO_DIRECTION_OUT:
              ret = IOEXP_SETDIRECTION(ioe, gpio, IOEXPANDER_DIRECTION_OUT);
              break;

            case VIRTIO_GPIO_DIRECTION_NONE:

              /* The guest released the line: park it as a high-impedance
               * input and discard all state held for it.
               */

              ret = IOEXP_SETDIRECTION(ioe, gpio, IOEXPANDER_DIRECTION_IN);
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
              if (ret >= 0)
                {
                  IOEXP_SETOPTION(ioe, gpio, IOEXPANDER_OPTION_INTCFG,
                                  (FAR void *)IOEXPANDER_VAL_DISABLE);
                  vhost_gpio_event_flush(priv, gpio);
                }
#endif
              break;

            default:
              ret = -EINVAL;
              break;
          }

        if (ret >= 0)
          {
            priv->dir[gpio] = val;
          }

        break;

      case VIRTIO_GPIO_MSG_GET_DIRECTION:
        *value = priv->dir[gpio];
        ret = 0;
        break;

      case VIRTIO_GPIO_MSG_SET_VALUE:
        ret = IOEXP_WRITEPIN(ioe, gpio, val != 0);
        break;

      case VIRTIO_GPIO_MSG_GET_VALUE:
        {
          bool pinval = false;

          ret = IOEXP_READPIN(ioe, gpio, &pinval);
          *value = pinval;
        }
        break;

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
      case VIRTIO_GPIO_MSG_SET_IRQ_TYPE:
        {
          irqstate_t flags;
          int intcfg;

          switch (val)
            {
              case VIRTIO_GPIO_IRQ_TYPE_EDGE_RISING:
                intcfg = IOEXPANDER_VAL_RISING;
                break;
              case VIRTIO_GPIO_IRQ_TYPE_EDGE_FALLING:
                intcfg = IOEXPANDER_VAL_FALLING;
                break;
              case VIRTIO_GPIO_IRQ_TYPE_EDGE_BOTH:
                intcfg = IOEXPANDER_VAL_BOTH;
                break;
              case VIRTIO_GPIO_IRQ_TYPE_LEVEL_HIGH:
                intcfg = IOEXPANDER_VAL_HIGH;
                break;
              case VIRTIO_GPIO_IRQ_TYPE_LEVEL_LOW:
                intcfg = IOEXPANDER_VAL_LOW;
                break;
              default:
                intcfg = IOEXPANDER_VAL_DISABLE;
                break;
            }

          ret = IOEXP_SETOPTION(ioe, gpio, IOEXPANDER_OPTION_INTCFG,
                                (FAR void *)(uintptr_t)intcfg);
          if (ret < 0)
            {
              break;
            }

          /* Key the flush on what was programmed: an unrecognized type
           * masks the line too.
           */

          if (intcfg == IOEXPANDER_VAL_DISABLE)
            {
              vhost_gpio_event_flush(priv, gpio);
            }
          else
            {
              flags = spin_lock_irqsave(&priv->evlock);
              priv->evlines[gpio].type = val;
              spin_unlock_irqrestore(&priv->evlock, flags);
            }
        }
        break;
#endif

      default:
        break;
    }

  return ret < 0 ? VIRTIO_GPIO_STATUS_ERR : VIRTIO_GPIO_STATUS_OK;
}

/****************************************************************************
 * Name: vhost_gpio_work
 *
 * Description:
 *   Serve the guest requests queued on the requestq.  Always runs on
 *   HPWORK -- the only context that touches this virtqueue, so no lock is
 *   needed.
 *
 ****************************************************************************/

static void vhost_gpio_work(FAR void *arg)
{
  FAR struct vhost_gpio_priv_s *priv = arg;
  FAR struct virtqueue *vq =
    priv->hdev->vrings_info[VHOST_GPIO_REQUESTQ].vq;

  for (; ; )
    {
      FAR struct virtio_gpio_response_s *resp;
      FAR struct virtio_gpio_request_s *req;
      struct virtqueue_buf vb[2];
      uint8_t value = 0;
      size_t vbcnt = 0;
      size_t reqlen = 0;
      int head;
      size_t i;

      head = vhost_get_vq_buffers(vq, vb, nitems(vb), &vbcnt);
      if (head < 0)
        {
          break;
        }

      for (i = 0; i < vbcnt; i++)
        {
          reqlen += vb[i].len;
        }

      if (vbcnt >= 2 && vb[0].len >= sizeof(*req) &&
          vb[vbcnt - 1].len >= sizeof(*resp))
        {
          req  = vb[0].buf;
          resp = vb[vbcnt - 1].buf;
          resp->status = vhost_gpio_dispatch(priv, req, &value);
          resp->value  = value;
        }

      virtqueue_add_consumed_buffer(vq, head, reqlen);
    }

  virtqueue_kick(vq);
}

/****************************************************************************
 * Name: vhost_gpio_handler
 ****************************************************************************/

static void vhost_gpio_handler(FAR struct virtqueue *vq)
{
  FAR struct vhost_gpio_priv_s *priv = vq->vq_dev->priv;

  if (work_available(&priv->work))
    {
      work_queue(HPWORK, &priv->work, vhost_gpio_work, priv, 0);
    }
}

/****************************************************************************
 * Name: vhost_gpio_probe
 ****************************************************************************/

static int vhost_gpio_probe(FAR struct vhost_device *hdev)
{
  FAR struct vhost_gpio_priv_s *priv;
  FAR const char *vqnames[VHOST_GPIO_NUM_VQ];
  vq_callback callback[VHOST_GPIO_NUM_VQ];
  uint8_t bus;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vhosterr("No enough memory\n");
      return -ENOMEM;
    }

  priv->hdev = hdev;
  hdev->priv = priv;
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  spin_lock_init(&priv->evlock);
#endif

  vhost_read_config_member(hdev, struct vhost_gpio_config_s, bus, &bus);
  priv->ioe = vhost_gpio_get_ioe(bus);
  if (priv->ioe == NULL)
    {
      vhosterr("GPIO device not found\n");
      kmm_free(priv);
      return -ENODEV;
    }

  vqnames[VHOST_GPIO_REQUESTQ]  = "virtio_gpio_req";
  callback[VHOST_GPIO_REQUESTQ] = vhost_gpio_handler;
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  vqnames[VHOST_GPIO_EVENTQ]    = "virtio_gpio_event";
  callback[VHOST_GPIO_EVENTQ]   = vhost_gpio_event_handler;
#endif

  ret = vhost_create_virtqueues(hdev, 0, VHOST_GPIO_NUM_VQ, vqnames,
                                callback, NULL);
  if (ret < 0)
    {
      vhosterr("vhost_create_virtqueues failed, ret=%d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Serve anything the guest queued before the callbacks were live.  The
   * requestq drain goes through the handler so it only ever runs on
   * HPWORK.
   */

  virtqueue_enable_cb(hdev->vrings_info[VHOST_GPIO_REQUESTQ].vq);
  vhost_gpio_handler(hdev->vrings_info[VHOST_GPIO_REQUESTQ].vq);

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  virtqueue_enable_cb(hdev->vrings_info[VHOST_GPIO_EVENTQ].vq);
  priv->ihandle = IOEP_ATTACH(priv->ioe, PINSET_ALL, vhost_gpio_irq_cb,
                              priv);
  vhost_gpio_event_handler(hdev->vrings_info[VHOST_GPIO_EVENTQ].vq);
#endif

  return OK;
}

/****************************************************************************
 * Name: vhost_gpio_remove
 ****************************************************************************/

static void vhost_gpio_remove(FAR struct vhost_device *hdev)
{
  FAR struct vhost_gpio_priv_s *priv = hdev->priv;

  virtqueue_disable_cb(hdev->vrings_info[VHOST_GPIO_REQUESTQ].vq);
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  virtqueue_disable_cb(hdev->vrings_info[VHOST_GPIO_EVENTQ].vq);
#endif
  work_cancel_sync(HPWORK, &priv->work);
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  if (priv->ihandle != NULL)
    {
      IOEP_DETACH(priv->ioe, priv->ihandle);
    }
#endif

  vhost_delete_virtqueues(hdev);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_register_gpio_driver
 ****************************************************************************/

int vhost_register_gpio_driver(void)
{
  return vhost_register_driver(&g_vhost_gpio_driver);
}
