/****************************************************************************
 * arch/sim/src/sim/posix/sim_libusb.c
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

#include <errno.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <pthread.h>

#include <linux/usb/ch9.h>
#include <libusb-1.0/libusb.h>

#include "sim_usbhost.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ERROR(fmt, ...) \
        syslog(LOG_ERR, "sim_libusb: " fmt "\n", ##__VA_ARGS__)
#define INFO(fmt, ...) \
        syslog(LOG_INFO, "sim_libusb: " fmt "\n", ##__VA_ARGS__)
#define DEBUG(fmt, ...)

#define HOST_LIBUSB_EP_NUM(addr)   ((addr) & USB_ENDPOINT_NUMBER_MASK)
#define HOST_LIBUSB_EP_DIR(addr)   ((addr) & USB_ENDPOINT_DIR_MASK)

#ifdef CONFIG_SIM_USB_VID
#  define USB_VID               CONFIG_SIM_USB_VID
#else
#  define USB_VID               0x18d1
#endif

#ifdef CONFIG_SIM_USB_PID
#  define USB_PID               CONFIG_SIM_USB_PID
#else
#  define USB_PID               0x4e11
#endif

#define HOST_LIBUSB_FIFO_NUM    8

#define HOST_LIBUSB_FIFO_USED(fifo) \
  ((fifo)->write - (fifo)->read)
#define HOST_LIBUSB_FIFO_PUSH(fifo) \
  ((fifo)->write++)
#define HOST_LIBUSB_FIFO_POP(fifo) \
  ((fifo)->read++)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct host_libusb_fifo_s
{
  uint16_t                    read;
  uint16_t                    write;
  struct host_usb_datareq_s  *datareq[HOST_LIBUSB_FIFO_NUM];
};

struct host_libusb_hostdev_s
{
  libusb_device                    *priv;
  libusb_device_handle             *handle;
  struct libusb_device_descriptor   dev_desc;
  struct libusb_config_descriptor **config_desc;
  libusb_hotplug_callback_handle    callback_handle;
  struct host_libusb_fifo_s         completed;
  pthread_t                         handle_thread;
  bool                              connected;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static libusb_context *g_libusb_context;
static struct host_libusb_hostdev_s g_libusb_dev;
static libusb_device **g_libusb_dev_list;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void host_libusb_fifoinit(struct host_libusb_fifo_s *fifo)
{
  fifo->write = 0;
  fifo->read = 0;
}

static struct host_usb_datareq_s *
host_libusb_fifopop(struct host_libusb_fifo_s *fifo)
{
  struct host_usb_datareq_s *datareq;
  uint16_t r_idx;

  if (HOST_LIBUSB_FIFO_USED(fifo) == 0)
    {
      return NULL;
    }

  r_idx = fifo->read & (HOST_LIBUSB_FIFO_NUM - 1);

  datareq = fifo->datareq[r_idx];

  fifo->read++;

  return datareq;
}

static bool host_libusb_fifopush(struct host_libusb_fifo_s *fifo,
                                 struct host_usb_datareq_s *datareq)
{
  uint16_t w_idx;

  if (HOST_LIBUSB_FIFO_USED(fifo) == HOST_LIBUSB_FIFO_NUM)
    {
      ERROR("USB get fifo fail");
      return false;
    }

  w_idx = fifo->write & (HOST_LIBUSB_FIFO_NUM - 1);

  fifo->datareq[w_idx] = datareq;

  fifo->write++;

  return true;
}

static void
host_libusb_getctrlreq(struct usb_ctrlrequest *libusb_req,
                       const struct host_usb_ctrlreq_s *host_req)
{
  libusb_req->bRequestType = host_req->type;
  libusb_req->bRequest = host_req->req;
  libusb_req->wValue = host_req->value;
  libusb_req->wIndex = host_req->index;
  libusb_req->wLength = host_req->len;
}

static void host_libusb_ep0transfer_cb(struct libusb_transfer *transfer)
{
  struct host_libusb_hostdev_s *dev = &g_libusb_dev;
  struct host_usb_datareq_s *datareq;
  uint8_t *buffer;

  if (!transfer)
    {
      ERROR("host_libusb_ep0transfer_cb() fail: %s\n",
            libusb_strerror(LIBUSB_ERROR_INVALID_PARAM));
      return;
    }

  datareq = (struct host_usb_datareq_s *)transfer->user_data;

  buffer = libusb_control_transfer_get_data(transfer);

  if (transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
      datareq->success = true;
      datareq->xfer += transfer->actual_length;
      memcpy(datareq->data, buffer, transfer->actual_length);
    }
  else
    {
      datareq->success = false;
    }

  free(buffer - LIBUSB_CONTROL_SETUP_SIZE);

  host_libusb_fifopush(&dev->completed, datareq);
  libusb_free_transfer(transfer);
}

static void host_libusb_bulktransfer_cb(struct libusb_transfer *transfer)
{
  struct host_libusb_hostdev_s *dev = &g_libusb_dev;
  struct host_usb_datareq_s *datareq;

  if (!transfer)
    {
      ERROR("host_libusb_bulktransfer_cb() fail: %s\n",
            libusb_strerror(LIBUSB_ERROR_INVALID_PARAM));
      return;
    }

  datareq = (struct host_usb_datareq_s *)transfer->user_data;

  if (transfer->status != LIBUSB_TRANSFER_COMPLETED)
    {
      datareq->success = false;
      goto transfer_end;
    }

  datareq->xfer += transfer->actual_length;
  if ((datareq->xfer >= datareq->len) ||
      (datareq->addr & USB_DIR_IN) != 0)
    {
      datareq->success = true;
      goto transfer_end;
    }

  libusb_fill_bulk_transfer(transfer, dev->handle, datareq->addr,
                            datareq->data + datareq->xfer,
                            datareq->len - datareq->xfer,
                            host_libusb_bulktransfer_cb,
                            datareq, 0);
  if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
    {
      datareq->success = false;
      goto transfer_end;
    }

  return;

transfer_end:
  host_libusb_fifopush(&dev->completed, datareq);
  libusb_free_transfer(transfer);
}

static void host_libusb_inttransfer_cb(struct libusb_transfer *transfer)
{
  struct host_libusb_hostdev_s *dev = &g_libusb_dev;
  struct host_usb_datareq_s *datareq;

  if (!transfer)
    {
      ERROR("host_libusb_inttransfer_cb() fail: %s\n",
            libusb_strerror(LIBUSB_ERROR_INVALID_PARAM));
      return;
    }

  datareq = (struct host_usb_datareq_s *)transfer->user_data;

  if (transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
      datareq->success = true;
      datareq->xfer += transfer->actual_length;
    }
  else
    {
      datareq->success = false;
    }

  host_libusb_fifopush(&dev->completed, datareq);
  libusb_free_transfer(transfer);
}

static int host_libusb_ep0inhandle(struct host_libusb_hostdev_s *dev,
                                   struct usb_ctrlrequest *ctrlreq,
                                   struct host_usb_datareq_s *datareq,
                                   int timeout)
{
  struct libusb_transfer *transfer;
  uint8_t *buffer;
  int ret = LIBUSB_SUCCESS;

  if (!dev->handle)
    {
      ERROR("host_libusb_ep0inhandle() fail: %s\n",
            libusb_strerror(LIBUSB_ERROR_NO_DEVICE));
      return LIBUSB_ERROR_NO_DEVICE;
    }

  buffer = malloc(LIBUSB_CONTROL_SETUP_SIZE + ctrlreq->wLength);
  if (!buffer)
    {
      ERROR("control data buffer malloc() fail: %s\n",
            libusb_strerror(LIBUSB_ERROR_NO_MEM));
      return LIBUSB_ERROR_NO_MEM;
    }

  transfer = libusb_alloc_transfer(0);
  if (!transfer)
    {
      ERROR("libusb_alloc_transfer() fail: %s\n",
            libusb_strerror(LIBUSB_ERROR_NO_MEM));
      ret = LIBUSB_ERROR_NO_MEM;
      goto err_with_buffer;
    }

  libusb_fill_control_setup(buffer, ctrlreq->bRequestType,
                            ctrlreq->bRequest, ctrlreq->wValue,
                            ctrlreq->wIndex, ctrlreq->wLength);
  libusb_fill_control_transfer(transfer, dev->handle, buffer,
                               host_libusb_ep0transfer_cb,
                               datareq, timeout);

  ret = libusb_submit_transfer(transfer);
  if (ret != LIBUSB_SUCCESS)
    {
      goto err_with_transfer;
    }

  return LIBUSB_SUCCESS;

err_with_transfer:
  libusb_free_transfer(transfer);

err_with_buffer:
  free(buffer);

  return ret;
}

static int host_libusb_ep0outhandle(struct host_libusb_hostdev_s *dev,
                                    struct usb_ctrlrequest *ctrlreq)
{
  int ret = LIBUSB_SUCCESS;

  if (!dev->handle)
    {
      ERROR("host_libusb_control_request() fail: %s\n",
            libusb_strerror(LIBUSB_ERROR_NO_DEVICE));
      return LIBUSB_ERROR_NO_DEVICE;
    }

  if ((ctrlreq->bRequestType & USB_TYPE_MASK) !=
      USB_TYPE_STANDARD)
    {
      return ret;
    }

  switch (ctrlreq->bRequest)
    {
      case USB_REQ_SET_CONFIGURATION:
        ret = libusb_detach_kernel_driver(dev->handle, 0);
        if (ret == LIBUSB_SUCCESS)
          {
            ret = libusb_set_configuration(dev->handle,
                                           ctrlreq->wValue);
            ret |= libusb_claim_interface(dev->handle, 0);
          }
        break;
      case USB_REQ_SET_INTERFACE: /* TODO */
        break;
      default:
        break;
    }

  return ret;
}

static int
host_libusb_bulktransfer(struct host_libusb_hostdev_s *dev, uint8_t addr,
                     struct host_usb_datareq_s *datareq)
{
  struct libusb_transfer *transfer;
  int ret = LIBUSB_SUCCESS;

  transfer = libusb_alloc_transfer(0);
  if (!transfer)
    {
      ERROR("libusb_alloc_transfer() fail: %s\n",
            libusb_strerror(LIBUSB_ERROR_NO_MEM));
      return LIBUSB_ERROR_NO_MEM;
    }

  libusb_fill_bulk_transfer(transfer, dev->handle, addr,
                            datareq->data, datareq->len,
                            host_libusb_bulktransfer_cb,
                            datareq, 0);
  ret = libusb_submit_transfer(transfer);
  if (ret != LIBUSB_SUCCESS)
    {
      libusb_free_transfer(transfer);
    }

  return ret;
}

static int
host_libusb_inttransfer(struct host_libusb_hostdev_s *dev, uint8_t addr,
                    struct host_usb_datareq_s *datareq)
{
  struct libusb_transfer *transfer;
  int ret = LIBUSB_SUCCESS;

  transfer = libusb_alloc_transfer(0);
  if (!transfer)
    {
      ERROR("libusb_alloc_transfer() fail: %s\n",
            libusb_strerror(LIBUSB_ERROR_NO_MEM));
      return LIBUSB_ERROR_NO_MEM;
    }

  libusb_fill_interrupt_transfer(transfer, dev->handle, addr,
                                 datareq->data, datareq->len,
                                 host_libusb_inttransfer_cb,
                                 datareq, 0);
  ret = libusb_submit_transfer(transfer);
  if (ret != LIBUSB_SUCCESS)
    {
      libusb_free_transfer(transfer);
    }

  return ret;
}

static void *host_libusb_event_handle(void *arg)
{
  while (1)
    {
      libusb_handle_events(g_libusb_context);
    }

  return NULL;
}

static bool host_libusb_connectdevice(void)
{
  int dev_cnt;
  int i;

  dev_cnt = libusb_get_device_list(g_libusb_context,
                                   &g_libusb_dev_list);
  if (dev_cnt < 0)
    {
      ERROR("libusb_get_device_list() failed: %s\n",
            libusb_strerror(dev_cnt));
      return false;
    }

  for (i = 0; i < dev_cnt; i++)
    {
      libusb_device *dev = g_libusb_dev_list[i];
      struct libusb_device_descriptor dev_desc;
      int ret = libusb_get_device_descriptor(dev, &dev_desc);
      if (ret != LIBUSB_SUCCESS)
        {
          ERROR("libusb_get_device_descriptor() failed: %s\n",
                libusb_strerror(ret));
          continue;
        }

      if (dev_desc.bDeviceClass == LIBUSB_CLASS_HUB)
        {
          continue;
        }

      if (dev_desc.idVendor == USB_VID ||
          dev_desc.idProduct == USB_PID)
        {
          g_libusb_dev.priv = dev;
          memcpy(&g_libusb_dev.dev_desc, &dev_desc,
                 sizeof(struct libusb_device_descriptor));
          return true;
        }
    }

  libusb_free_device_list(g_libusb_dev_list, 1);
  g_libusb_dev_list = NULL;
  return false;
}

static int host_libusb_hotplug_callback(libusb_context *ctx,
                                        libusb_device *device,
                                        libusb_hotplug_event event,
                                        void *user_data)
{
  struct host_libusb_hostdev_s *dev = &g_libusb_dev;

  INFO("Hotplug event: %d\n", event);

  if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED)
    {
      dev->connected = true;
    }
  else
    {
      dev->connected = false;
    }

  return LIBUSB_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_usbhost_ep0trans(struct host_usb_ctrlreq_s *ctrlreq,
                          struct host_usb_datareq_s *datareq)
{
  struct host_libusb_hostdev_s *dev = &g_libusb_dev;
  struct usb_ctrlrequest libusb_ctrlreq;
  int ret;

  host_libusb_getctrlreq(&libusb_ctrlreq, ctrlreq);

  if (!(libusb_ctrlreq.bRequestType & USB_DIR_IN))
    {
      ret = host_libusb_ep0outhandle(dev, &libusb_ctrlreq);
      datareq->success = (ret != LIBUSB_SUCCESS) ? false : true;
      host_libusb_fifopush(&dev->completed, datareq);
    }
  else
    {
      ret = host_libusb_ep0inhandle(dev, &libusb_ctrlreq,
                                    datareq, 0);
      if (ret != LIBUSB_SUCCESS)
        {
          datareq->success = false;
          host_libusb_fifopush(&dev->completed, datareq);
        }
    }

  return ret;
}

int host_usbhost_eptrans(struct host_usb_datareq_s *datareq)
{
  struct host_libusb_hostdev_s *dev = &g_libusb_dev;
  int ret = LIBUSB_SUCCESS;

  switch (datareq->xfrtype & USB_ENDPOINT_XFERTYPE_MASK)
    {
      case USB_ENDPOINT_XFER_BULK:
        ret = host_libusb_bulktransfer(dev, datareq->addr, datareq);
        break;
      case USB_ENDPOINT_XFER_INT:
        ret = host_libusb_inttransfer(dev, datareq->addr, datareq);
        break;
      default:
        ERROR("Unsupported transfer type");
        ret = LIBUSB_ERROR_NOT_SUPPORTED;
        break;
    }

  if (ret != LIBUSB_SUCCESS)
    {
      datareq->success = false;
      host_libusb_fifopush(&dev->completed, datareq);
    }

  return ret;
}

int host_usbhost_open(void)
{
  struct host_libusb_hostdev_s *dev = &g_libusb_dev;
  uint8_t cnt;
  int ret;

  if (!dev->priv)
    {
      if (!host_libusb_connectdevice())
        {
          ERROR("host_libusb_connectdevice() failed\n");
          goto err_out;
        }
    }

  ret = libusb_open(dev->priv, &dev->handle);
  if (ret != LIBUSB_SUCCESS)
    {
      ERROR("libusb_open() failed: %s\n", libusb_strerror(ret));
      goto err_out;
    }

  ret = libusb_set_auto_detach_kernel_driver(dev->handle, 1);
  if (ret != LIBUSB_SUCCESS)
    {
      ERROR("libusb_set_auto_detach_kernel_driver() failed: %s\n",
            libusb_strerror(ret));
      goto err_out;
    }

  dev->config_desc = (struct libusb_config_descriptor **)
                      malloc(dev->dev_desc.bNumConfigurations
                      *(sizeof(struct libusb_config_descriptor *)));
  if (!dev->config_desc)
    {
      ERROR("host_libusb_devinit() malloc failed: %s\n",
            libusb_strerror(LIBUSB_ERROR_NO_MEM));
      ret = LIBUSB_ERROR_NO_MEM;
      goto err_out;
    }

  for (cnt = 0; cnt < dev->dev_desc.bNumConfigurations; cnt++)
    {
      ret = libusb_get_config_descriptor(dev->priv, cnt,
              &dev->config_desc[cnt]);
      if (ret != LIBUSB_SUCCESS)
        {
          ERROR("libusb_get_config_descriptor() failed: %s\n",
                libusb_strerror(ret));
          goto err_out;
        }
    }

  host_libusb_fifoinit(&dev->completed);

  return LIBUSB_SUCCESS;

err_out:
  host_usbhost_close();
  return ret;
}

void host_usbhost_close(void)
{
  struct host_libusb_hostdev_s *dev = &g_libusb_dev;

  dev->priv = NULL;

  if (dev->config_desc)
    {
      free(dev->config_desc);
      dev->config_desc = NULL;
    }

  if (dev->handle)
    {
      libusb_close(dev->handle);
      dev->handle = NULL;
    }

  if (g_libusb_dev_list)
    {
      libusb_free_device_list(g_libusb_dev_list, 1);
      g_libusb_dev_list = NULL;
    }
}

struct host_usb_datareq_s *host_usbhost_getcomplete(void)
{
  struct host_libusb_hostdev_s *dev = &g_libusb_dev;
  return host_libusb_fifopop(&dev->completed);
}

bool host_usbhost_getconnstate(void)
{
  return g_libusb_dev.connected;
}

int host_usbhost_init(void)
{
  int ret;

  ret = libusb_init(&g_libusb_context);
  if (ret < 0)
    {
      ERROR("libusb_init() failed: %s\n", libusb_strerror(ret));
      return ret;
    }

  ret = libusb_hotplug_register_callback(g_libusb_context,
          (libusb_hotplug_event) (LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT |
          LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED),
          (libusb_hotplug_flag) 0, USB_VID, USB_PID,
          LIBUSB_HOTPLUG_MATCH_ANY, host_libusb_hotplug_callback,
          NULL, NULL);
  if (ret != LIBUSB_SUCCESS)
    {
      ERROR("libusb_hotplug_register_callback() failed: %s\n",
            libusb_strerror(ret));
      return ret;
    }

  g_libusb_dev.connected = host_libusb_connectdevice();

  ret = pthread_create(&g_libusb_dev.handle_thread, NULL,
                       host_libusb_event_handle, NULL);
  if (ret < 0)
    {
      ERROR("pthread_create() failed\n");
      return LIBUSB_ERROR_INVALID_PARAM;
    }

  return LIBUSB_SUCCESS;
}
