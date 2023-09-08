/****************************************************************************
 * drivers/virtio/virtio-serial.c
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
#include <stdio.h>

#include <nuttx/serial/serial.h>
#include <nuttx/virtio/virtio.h>

#include "virtio-serial.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_SERIAL_RX           0
#define VIRTIO_SERIAL_TX           1
#define VIRTIO_SERIAL_NUM          2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtio_serial_priv_s
{
  /* Virtio device informations */

  FAR struct virtio_device *vdev;

  /* Nuttx uart device informations */

  FAR struct uart_dev_s     udev;
  char                      name[NAME_MAX];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Uart operation functions */

static int  virtio_serial_setup(FAR struct uart_dev_s *dev);
static void virtio_serial_shutdown(FAR struct uart_dev_s *dev);
static int  virtio_serial_attach(FAR struct uart_dev_s *dev);
static void virtio_serial_detach(FAR struct uart_dev_s *dev);
static int  virtio_serial_ioctl(FAR struct file *filep, int cmd,
                                unsigned long arg);
static void virtio_serial_rxint(FAR struct uart_dev_s *dev, bool enable);
static void virtio_serial_send(FAR struct uart_dev_s *dev, int ch);
static void virtio_serial_txint(FAR struct uart_dev_s *dev, bool enable);
static bool virtio_serial_txready(FAR struct uart_dev_s *dev);
static bool virtio_serial_txempty(FAR struct uart_dev_s *dev);
static void virtio_serial_dmasend(FAR struct uart_dev_s *dev);
static void virtio_serial_dmatxavail(FAR struct uart_dev_s *dev);
static void virtio_serial_dmareceive(FAR struct uart_dev_s *dev);
static void virtio_serial_dmarxfree(FAR struct uart_dev_s *dev);

/* Other functions */

static void virtio_serial_rxready(FAR struct virtqueue *vq);
static void virtio_serial_txdone(FAR struct virtqueue *vq);

static int  virtio_serial_probe(FAR struct virtio_device *vdev);
static void virtio_serial_remove(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_serial_driver =
{
  LIST_INITIAL_VALUE(g_virtio_serial_driver.node), /* node */
  VIRTIO_ID_CONSOLE,                               /* device id */
  virtio_serial_probe,                             /* probe */
  virtio_serial_remove,                            /* remove */
};

static struct virtio_driver g_virtio_rprocserial_driver =
{
  LIST_INITIAL_VALUE(g_virtio_rprocserial_driver.node), /* node */
  VIRTIO_ID_RPROC_SERIAL,                               /* device id */
  virtio_serial_probe,                                  /* probe */
  virtio_serial_remove,                                 /* remove */
};

static const struct uart_ops_s g_virtio_serial_ops =
{
  virtio_serial_setup,       /* setup */
  virtio_serial_shutdown,    /* shutdown */
  virtio_serial_attach,      /* attach */
  virtio_serial_detach,      /* detach */
  virtio_serial_ioctl,       /* ioctl */
  NULL,                      /* receive */
  virtio_serial_rxint,       /* rxint */
  NULL,                      /* rxavailable */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  NULL,                      /* rxflowcontrol */
#endif
  virtio_serial_dmasend,     /* dmasend */
  virtio_serial_dmareceive,  /* dmareceive */
  virtio_serial_dmarxfree,   /* dmarxfree */
  virtio_serial_dmatxavail,  /* dmatxavail */
  virtio_serial_send,        /* send */
  virtio_serial_txint,       /* txint */
  virtio_serial_txready,     /* txready */
  virtio_serial_txempty,     /* txempty */
};

static int g_virtio_serial_idx = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_serial_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int virtio_serial_setup(FAR struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: virtio_serial_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void virtio_serial_shutdown(FAR struct uart_dev_s *dev)
{
  /* Nothing */
}

/****************************************************************************
 * Name: virtio_serial_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int virtio_serial_attach(FAR struct uart_dev_s *dev)
{
  FAR struct virtio_serial_priv_s *priv = dev->priv;
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_SERIAL_RX].vq;

  virtqueue_enable_cb(vq);
  return 0;
}

/****************************************************************************
 * Name: virtio_serial_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void virtio_serial_detach(FAR struct uart_dev_s *dev)
{
  FAR struct virtio_serial_priv_s *priv = dev->priv;
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_SERIAL_RX].vq;

  virtqueue_disable_cb(vq);
}

/****************************************************************************
 * Name: virtio_serial_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int virtio_serial_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: virtio_serial_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void virtio_serial_rxint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: virtio_serial_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void virtio_serial_send(FAR struct uart_dev_s *dev, int ch)
{
  int nexthead;

  nexthead = dev->xmit.head + 1;
  if (nexthead >= dev->xmit.size)
    {
      nexthead = 0;
    }

  if (nexthead != dev->xmit.tail)
    {
      /* No.. not full.  Add the character to the TX buffer and return. */

      dev->xmit.buffer[dev->xmit.head] = ch;
      dev->xmit.head = nexthead;
    }

  uart_dmatxavail(dev);
}

/****************************************************************************
 * Name: virtio_serial_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void virtio_serial_txint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: uart_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool virtio_serial_txready(FAR struct uart_dev_s *dev)
{
  int nexthead = dev->xmit.head + 1;

  if (nexthead >= dev->xmit.size)
    {
      nexthead = 0;
    }

  return nexthead != dev->xmit.tail;
}

/****************************************************************************
 * Name: virtio_serial_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool virtio_serial_txempty(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: virtio_serial_dmasend
 ****************************************************************************/

static void virtio_serial_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct virtio_serial_priv_s *priv = dev->priv;
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_SERIAL_TX].vq;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;
  struct virtqueue_buf vb[2];
  uintptr_t len;
  int num = 1;

  /* Get the total send length */

  len = xfer->length + xfer->nlength;

  /* Set the virtqueue buffer */

  vb[0].buf = xfer->buffer;
  vb[0].len = xfer->length;

  if (xfer->nlength != 0)
    {
      vb[1].buf = xfer->nbuffer;
      vb[1].len = xfer->nlength;
      num = 2;
    }

  /* Add buffer to TX virtiqueue and notify the other size */

  virtqueue_add_buffer(vq, vb, num, 0, (FAR void *)len);
  virtqueue_kick(vq);
}

/****************************************************************************
 * Name: virtio_serial_dmatxavail
 ****************************************************************************/

static void virtio_serial_dmatxavail(FAR struct uart_dev_s *dev)
{
  if (dev->dmatx.length == 0)
    {
      uart_xmitchars_dma(dev);
    }
}

/****************************************************************************
 * Name: virtio_serial_dmareceive
 ****************************************************************************/

static void virtio_serial_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct virtio_serial_priv_s *priv = dev->priv;
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_SERIAL_RX].vq;
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  struct virtqueue_buf vb[2];
  int num = 1;

  vb[0].buf = xfer->buffer;
  vb[0].len = xfer->length;

  if (xfer->nlength != 0)
    {
      vb[num].buf = xfer->nbuffer;
      vb[num].len = xfer->nlength;
      num = 2;
    }

  /* Add buffer to the RX virtqueue and notify the device side */

  virtqueue_add_buffer(vq, vb, 0, num, xfer);
  virtqueue_kick(vq);
}

/****************************************************************************
 * Name: virtio_serial_dmarxfree
 ****************************************************************************/

static void virtio_serial_dmarxfree(FAR struct uart_dev_s *dev)
{
  if (dev->dmarx.length == 0)
    {
      uart_recvchars_dma(dev);
    }
}

/****************************************************************************
 * Name: virtio_serial_rxready
 *
 * Description:
 *   The virt serial receive virtqueue callback funtion
 *
 ****************************************************************************/

static void virtio_serial_rxready(FAR struct virtqueue *vq)
{
  FAR struct virtio_serial_priv_s *priv = vq->vq_dev->priv;
  FAR struct uart_dmaxfer_s *xfer;
  uint32_t len;

  /* Received some data, call uart_recvchars_done() */

  xfer = virtqueue_get_buffer(vq, &len, NULL);
  if (xfer == NULL)
    {
      return;
    }

  xfer->nbytes = len;
  uart_recvchars_done(&priv->udev);
  uart_dmarxfree(&priv->udev);
}

/****************************************************************************
 * Name: virtio_serial_txdone
 *
 * Description:
 *   The virt serial transimit virtqueue callback funtion
 *
 ****************************************************************************/

static void virtio_serial_txdone(FAR struct virtqueue *vq)
{
  FAR struct virtio_serial_priv_s *priv = vq->vq_dev->priv;
  uintptr_t len;

  /* Call uart_xmitchars_done to notify the upperhalf */

  len = (uintptr_t)virtqueue_get_buffer(vq, NULL, NULL);
  priv->udev.dmatx.nbytes = len;
  uart_xmitchars_done(&priv->udev);
  uart_dmatxavail(&priv->udev);
}

/****************************************************************************
 * Name: virtio_serial_init
 ****************************************************************************/

static int virtio_serial_init(FAR struct virtio_serial_priv_s *priv,
                              FAR struct virtio_device *vdev)
{
  FAR const char *vqnames[VIRTIO_SERIAL_NUM];
  vq_callback callbacks[VIRTIO_SERIAL_NUM];
  FAR struct uart_dev_s *udev;
  int ret;

  priv->vdev = vdev;
  vdev->priv = priv;

  /* Uart device buffer and ops init */

  udev              = &priv->udev;
  udev->priv        = priv;
  udev->ops         = &g_virtio_serial_ops;
  udev->recv.size   = CONFIG_DRIVERS_VIRTIO_SERIAL_BUFSIZE;
  udev->recv.buffer = virtio_zalloc_buf(vdev, udev->recv.size, 16);
  if (udev->recv.buffer == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  udev->xmit.size   = CONFIG_DRIVERS_VIRTIO_SERIAL_BUFSIZE;
  udev->xmit.buffer = virtio_zalloc_buf(vdev, udev->xmit.size, 16);
  if (udev->xmit.buffer == NULL)
    {
      vrterr("No enough memory\n");
      ret = -ENOMEM;
      goto err_with_recv;
    }

  /* Initialize the virtio device */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_set_features(vdev, 0);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqnames[VIRTIO_SERIAL_RX]   = "virtio_serial_rx";
  vqnames[VIRTIO_SERIAL_TX]   = "virtio_serial_tx";
  callbacks[VIRTIO_SERIAL_RX] = virtio_serial_rxready;
  callbacks[VIRTIO_SERIAL_TX] = virtio_serial_txdone;
  ret = virtio_create_virtqueues(vdev, 0, VIRTIO_SERIAL_NUM, vqnames,
                                 callbacks);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      goto err_with_xmit;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
  return OK;

err_with_xmit:
  virtio_free_buf(vdev, udev->xmit.buffer);
err_with_recv:
  virtio_free_buf(vdev, udev->recv.buffer);
  virtio_reset_device(vdev);
  return ret;
}

/****************************************************************************
 * Name: virtio_serial_uninit
 ****************************************************************************/

static void virtio_serial_uninit(FAR struct virtio_serial_priv_s *priv)
{
  FAR struct virtio_device *vdev = priv->vdev;

  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
  virtio_free_buf(vdev, priv->udev.xmit.buffer);
  virtio_free_buf(vdev, priv->udev.recv.buffer);
}

/****************************************************************************
 * Name: virtio_serial_probe
 ****************************************************************************/

static int virtio_serial_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_serial_priv_s *priv;
  int ret;

  /* Alloc the virtio serial driver and uart buffer */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  ret = virtio_serial_init(priv, vdev);
  if (ret < 0)
    {
      vrterr("virtio_serial_init failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  /* Uart driver register */

  snprintf(priv->name, NAME_MAX, "/dev/ttyV%d", g_virtio_serial_idx);
  ret = uart_register(priv->name, &priv->udev);
  if (ret < 0)
    {
      vrterr("uart_register failed, ret=%d\n", ret);
      goto err_with_init;
    }

  g_virtio_serial_idx++;
  return ret;

err_with_init:
  virtio_serial_uninit(priv);
err_with_priv:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: virtio_serial_remove
 ****************************************************************************/

static void virtio_serial_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_serial_priv_s *priv = vdev->priv;

  unregister_driver(priv->name);
  virtio_serial_uninit(priv);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_serial_driver
 ****************************************************************************/

int virtio_register_serial_driver(void)
{
  int ret1 = virtio_register_driver(&g_virtio_serial_driver);
  int ret2 = virtio_register_driver(&g_virtio_rprocserial_driver);
  return ret1 < 0 ? ret1 : ret2;
}
