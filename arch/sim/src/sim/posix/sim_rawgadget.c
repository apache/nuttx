/****************************************************************************
 * arch/sim/src/sim/posix/sim_rawgadget.c
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

#include "sim_internal.h"
#include "sim_usbdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ERROR(fmt, ...) \
        syslog(LOG_ERR, "sim_rawgadget: " fmt "\n", ##__VA_ARGS__)
#define INFO(fmt, ...) \
        syslog(LOG_INFO, "sim_rawgadget: " fmt "\n", ##__VA_ARGS__)
#define DEBUG(fmt, ...)

#define USB_RAW_IOCTL_INIT          _IOW('U', 0, struct usb_raw_init_s)
#define USB_RAW_IOCTL_RUN           _IO('U', 1)
#define USB_RAW_IOCTL_EVENT_FETCH   _IOR('U', 2, struct usb_raw_event_s)
#define USB_RAW_IOCTL_EP0_WRITE     _IOW('U', 3, struct usb_raw_ep_io_s)
#define USB_RAW_IOCTL_EP0_READ      _IOWR('U', 4, struct usb_raw_ep_io_s)
#define USB_RAW_IOCTL_EP_ENABLE     _IOW('U', 5, struct usb_endpoint_descriptor)
#define USB_RAW_IOCTL_EP_DISABLE    _IOW('U', 6, __u32)
#define USB_RAW_IOCTL_EP_WRITE      _IOW('U', 7, struct usb_raw_ep_io_s)
#define USB_RAW_IOCTL_EP_READ       _IOWR('U', 8, struct usb_raw_ep_io_s)
#define USB_RAW_IOCTL_CONFIGURE     _IO('U', 9)
#define USB_RAW_IOCTL_VBUS_DRAW     _IOW('U', 10, __u32)
#define USB_RAW_IOCTL_EPS_INFO      _IOR('U', 11, struct usb_raw_eps_info_s)
#define USB_RAW_IOCTL_EP0_STALL     _IO('U', 12)
#define USB_RAW_IOCTL_EP_SET_HALT   _IOW('U', 13, __u32)
#define USB_RAW_IOCTL_EP_CLEAR_HALT _IOW('U', 14, __u32)
#define USB_RAW_IOCTL_EP_SET_WEDGE  _IOW('U', 15, __u32)

#define USB_RAW_EP_NUM(addr)        ((addr) & USB_ENDPOINT_NUMBER_MASK)
#define USB_RAW_EP_DIR(addr)        ((addr) & USB_ENDPOINT_DIR_MASK)

#define USB_RAW_EPS_NUM_MAX         30
#define USB_RAW_EP_NAME_MAX         16
#define USB_RAW_EP_ADDR_ANY         0xff

#define UDC_NAME_LENGTH_MAX         128

#define USB_RAW_EP0_MAX_LEN         256
#define USB_RAW_EP_MAX_LEN          1024

#define USB_RAW_RX_BUF_NUM          8

#define USB_RAW_DEVICE              "dummy_udc.0"
#define USB_RAW_DRIVER              "dummy_udc"

#define USB_RAW_FIFO_USED(fifo)     ((fifo)->write - (fifo)->read)
#define USB_RAW_FIFO_UNUSED(fifo)   ((fifo)->elem_num - USB_RAW_FIFO_USED(fifo))
#define USB_RAW_FIFO_MASK(fifo)     ((fifo)->elem_num - 1)
#define USB_RAW_FIFO_PUSH(fifo)     ((fifo)->write++)
#define USB_RAW_FIFO_POP(fifo)      ((fifo)->read++)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum usb_raw_event_type_e
{
  USB_RAW_EVENT_INVALID,
  USB_RAW_EVENT_CONNECT,
  USB_RAW_EVENT_CONTROL,
};

struct usb_raw_init_s
{
  uint8_t   driver_name[UDC_NAME_LENGTH_MAX];
  uint8_t   device_name[UDC_NAME_LENGTH_MAX];
  uint8_t   speed;
};

struct usb_raw_event_s
{
  uint32_t    type;
  uint32_t    length;
  uint8_t     data[0];
};

struct usb_raw_ep_io_s
{
  uint16_t  ep;
  uint16_t  flags;
  uint32_t  length;
  uint8_t   data[0];
};

struct usb_raw_control_io_s
{
  struct usb_raw_ep_io_s  inner;
  uint8_t                 data[USB_RAW_EP0_MAX_LEN];
};

struct usb_raw_data_io_s
{
  struct usb_raw_ep_io_s  inner;
  uint8_t                 data[USB_RAW_EP_MAX_LEN];
};

struct usb_raw_ep_caps_s
{
  uint32_t  type_control  : 1;
  uint32_t  type_iso      : 1;
  uint32_t  type_bulk     : 1;
  uint32_t  type_int      : 1;
  uint32_t  dir_in        : 1;
  uint32_t  dir_out       : 1;
};

struct usb_raw_ep_limits_s
{
  uint16_t  maxpacket_limit;
  uint16_t  max_streams;
  uint32_t  reserved;
};

struct usb_raw_ep_info_s
{
  uint8_t                     name[USB_RAW_EP_NAME_MAX];
  uint32_t                    addr;
  struct usb_raw_ep_caps_s    caps;
  struct usb_raw_ep_limits_s  limits;
};

struct usb_raw_eps_info_s
{
  struct usb_raw_ep_info_s  eps[USB_RAW_EPS_NUM_MAX];
};

struct usb_raw_control_event_s
{
  struct usb_raw_event_s    inner;
  struct usb_ctrlrequest    ctrl;
};

struct usb_raw_fifo_s
{
  uint16_t    read;
  uint16_t    write;
  uint16_t    elem_size;
  uint16_t    elem_num;
  uint8_t    *elems;
};

struct usb_raw_ep_entry_s
{
  bool                  halted;
  uint16_t              addr;
  uint16_t              raw_epaddr;
  uint16_t              raw_epid;
  struct usb_raw_fifo_s fifo;
  pthread_t             ep_thread;
};

struct usb_raw_gadget_dev_t
{
  int                         fd;
  uint16_t                    eps_num;
  pthread_t                   ep0_thread;
  struct usb_raw_control_io_s ep0_ctrl;
  struct usb_raw_ep_entry_s   eps_entry[USB_RAW_EPS_NUM_MAX];
  struct usb_raw_eps_info_s   eps_info;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usb_raw_gadget_dev_t g_raw_gadget_dev =
{
  .fd = -1,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void host_raw_fifocreate(struct usb_raw_fifo_s *fifo,
                                   uint16_t elem_size, uint16_t elem_num)
{
  if (elem_num & (elem_num - 1))
    {
      ERROR("USB raw fifo num error");
      return;
    }

  fifo->write = 0;
  fifo->read = 0;
  fifo->elem_size = elem_size;
  fifo->elem_num = elem_num;
  fifo->elems = (uint8_t *)malloc(elem_size * elem_num);
}

static void host_raw_fifodelete(struct usb_raw_fifo_s *fifo)
{
  fifo->write = 0;
  fifo->read = 0;
  free(fifo->elems);
}

static uint8_t *host_raw_fiforead(struct usb_raw_fifo_s *fifo)
{
  uint16_t r_idx;

  if (USB_RAW_FIFO_USED(fifo) == 0)
    {
      return NULL;
    }

  r_idx = fifo->read & USB_RAW_FIFO_MASK(fifo);
  return &fifo->elems[fifo->elem_size * r_idx];
}

static uint8_t *host_raw_fifoalloc(struct usb_raw_fifo_s *fifo)
{
  uint16_t w_idx;

  if (USB_RAW_FIFO_UNUSED(fifo) == 0)
    {
      ERROR("USB raw get fifo fail");
      return NULL;
    }

  w_idx = fifo->write & USB_RAW_FIFO_MASK(fifo);
  return &fifo->elems[fifo->elem_size * w_idx];
}

static int host_raw_open(void)
{
  int fd = open("/dev/raw-gadget", O_RDWR);
  if (fd < 0)
    {
      ERROR("open fail");
    }

  return fd;
}

static void host_raw_close(int fd)
{
  if (fd >= 0)
    {
      close(fd);
    }
}

static void host_raw_init(int fd, enum usb_device_speed speed,
                          const char *driver, const char *device)
{
  struct usb_raw_init_s arg;
  strcpy((char *)&arg.driver_name[0], driver);
  strcpy((char *)&arg.device_name[0], device);
  arg.speed = speed;
  int rv = ioctl(fd, USB_RAW_IOCTL_INIT, &arg);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_INIT) fail");
    }
}

static int host_raw_run(int fd)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_RUN, 0);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_RUN) fail");
    }

  return rv;
}

static int host_raw_eventfetch(int fd, struct usb_raw_event_s *event)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_EVENT_FETCH, event);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_EVENT_FETCH) fail");
    }

  return rv;
}

static int host_raw_ep0read(int fd, struct usb_raw_ep_io_s *io)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_EP0_READ, io);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_EP0_READ)");
    }

  return rv;
}

static int host_raw_ep0write(int fd, struct usb_raw_ep_io_s *io)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_EP0_WRITE, io);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_EP0_WRITE) fail");
    }

  return rv;
}

static int host_raw_epenable(int fd, struct usb_endpoint_descriptor *desc)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_EP_ENABLE, desc);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_EP_ENABLE) fail");
    }

  return rv;
}

static int host_raw_epdisable(int fd, uint8_t epno)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_EP_DISABLE, epno);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_EP_DISABLE) fail");
    }

  return rv;
}

static int host_raw_epread(int fd, struct usb_raw_ep_io_s *io)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_EP_READ, io);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_EP_READ) fail");
    }

  return rv;
}

static int host_raw_epwrite(int fd, struct usb_raw_ep_io_s *io)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_EP_WRITE, io);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_EP_WRITE) fail");
    }

  return rv;
}

static void host_raw_configure(int fd)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_CONFIGURE, 0);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_CONFIGURED) fail");
    }
}

static void host_raw_vbusdraw(int fd, uint32_t power)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_VBUS_DRAW, power);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_VBUS_DRAW) fail");
    }
}

static int host_raw_epsinfo(int fd, struct usb_raw_eps_info_s *info)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_EPS_INFO, info);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_EPS_INFO) fail");
    }

  return rv;
}

static int host_raw_ep0stall(int fd)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_EP0_STALL, 0);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_EP0_STALL) fail");
    }

  return rv;
}

static int host_raw_epsethalt(int fd, int ep)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_EP_SET_HALT, ep);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_EP_SET_HALT) fail");
    }

  return rv;
}

static int host_raw_epclearhalt(int fd, int ep)
{
  int rv = ioctl(fd, USB_RAW_IOCTL_EP_CLEAR_HALT, ep);
  if (rv < 0)
    {
      ERROR("ioctl(USB_RAW_IOCTL_EP_CLEAR_HALT) fail");
    }

  return rv;
}

static void
host_raw_setctrlreq(struct host_usb_ctrlreq_s *host_req,
                    const struct usb_ctrlrequest *raw_req)
{
  host_req->type = raw_req->bRequestType;
  host_req->req = raw_req->bRequest;
  host_req->value = raw_req->wValue;
  host_req->index = raw_req->wIndex;
  host_req->len = raw_req->wLength;
}

static void
host_raw_getepdesc(struct usb_endpoint_descriptor *raw_epdesc,
                   const struct host_usb_epdesc_s *host_epdesc)
{
  raw_epdesc->bLength = host_epdesc->len;
  raw_epdesc->bDescriptorType = host_epdesc->type;
  raw_epdesc->bEndpointAddress = host_epdesc->addr;
  raw_epdesc->bmAttributes = host_epdesc->attr;
  raw_epdesc->wMaxPacketSize = host_epdesc->mxpacketsize;
  raw_epdesc->bInterval = host_epdesc->interval;
}

static int host_raw_connecthandle(struct usb_raw_gadget_dev_t *dev)
{
  struct usb_raw_eps_info_s *info = &dev->eps_info;
  int i;

  memset(info, 0, sizeof(struct usb_raw_eps_info_s));

  dev->eps_num = host_raw_epsinfo(dev->fd, info);
  for (i = 0; i < dev->eps_num; i++)
    {
      INFO("ep #%d:", i);
      INFO("  name: %s", &info->eps[i].name[0]);
      INFO("  addr: %u", info->eps[i].addr);
      INFO("  type: %s %s %s",
           info->eps[i].caps.type_iso ? "iso" : "___",
           info->eps[i].caps.type_bulk ? "blk" : "___",
           info->eps[i].caps.type_int ? "int" : "___");
      INFO("  dir : %s %s",
           info->eps[i].caps.dir_in ? "in " : "___",
           info->eps[i].caps.dir_out ? "out" : "___");
      INFO("  maxpacket_limit: %u",
           info->eps[i].limits.maxpacket_limit);
      INFO("  max_streams: %u", info->eps[i].limits.max_streams);
    }

  return 0;
}

static bool
host_raw_check_epaddress(struct usb_endpoint_descriptor *epd)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;
  struct usb_raw_eps_info_s *eps_info = &dev->eps_info;
  uint16_t ep_cnt;

  for (ep_cnt = 0; ep_cnt < dev->eps_num; ep_cnt++)
    {
      struct usb_raw_ep_info_s *ep = &eps_info->eps[ep_cnt];

      if (ep->addr != USB_RAW_EP_NUM(epd->bEndpointAddress) &&
          ep->addr != USB_RAW_EP_ADDR_ANY)
        {
          continue;
        }

      if ((usb_endpoint_dir_in(epd) && !ep->caps.dir_in) ||
          (usb_endpoint_dir_out(epd) && !ep->caps.dir_out))
        {
          continue;
        }

      if ((usb_endpoint_type(epd) == USB_ENDPOINT_XFER_BULK &&
          !ep->caps.type_bulk) ||
          (usb_endpoint_type(epd) == USB_ENDPOINT_XFER_INT &&
          !ep->caps.type_int) ||
          (usb_endpoint_type(epd) == USB_ENDPOINT_XFER_ISOC &&
          !ep->caps.type_iso))
        {
          continue;
        }

      return true;
    }

  return false;
}

static int host_raw_ctrlhandle(struct usb_raw_gadget_dev_t *dev,
                               struct usb_raw_control_event_s *event)
{
  struct usb_raw_ep_entry_s *entry = &dev->eps_entry[0];
  struct usb_raw_ep_io_s *io = &dev->ep0_ctrl.inner;
  struct host_usb_ctrlreq_s *host_ctrl_req;
  int ret = -1;

  host_ctrl_req = (struct host_usb_ctrlreq_s *)
                   host_raw_fifoalloc(&entry->fifo);

  if (!host_ctrl_req)
    {
      ERROR("EP0 get raw fifo error");
      return ret;
    }

  host_raw_setctrlreq(host_ctrl_req, &event->ctrl);

  if (!(event->ctrl.bRequestType & USB_DIR_IN))
    {
      io->ep = 0;
      io->flags = 0;
      io->length = USB_RAW_EP0_MAX_LEN;

      ret = host_raw_ep0read(dev->fd, io);
      if (ret < 0)
        {
          ERROR("EP0 read out data error");
          return ret;
        }

      memcpy(host_ctrl_req->data, io->data, ret);
    }

  USB_RAW_FIFO_PUSH(&entry->fifo);

  return ret;
}

static void *host_raw_ep0handle(void *arg)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;
  struct usb_raw_ep_entry_s *entry = &dev->eps_entry[0];
  struct usb_raw_control_event_s event;

  while (dev->fd >= 0)
    {
      event.inner.type = 0;
      event.inner.length = sizeof(event.ctrl);
      if (host_raw_eventfetch(dev->fd, &event.inner) < 0)
        {
          ERROR("EP0 event fetch fail.");
          continue;
        }

      if (event.inner.type == USB_RAW_EVENT_CONNECT)
        {
          host_raw_connecthandle(dev);
        }
      else if (event.inner.type == USB_RAW_EVENT_CONTROL)
        {
          host_raw_ctrlhandle(dev, &event);
        }
      else
        {
          ERROR("EP0 receive wrong event.");
        }
    }

  host_raw_fifodelete(&entry->fifo);

  return NULL;
}

static void *host_raw_ephandle(void *arg)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;
  struct usb_raw_ep_entry_s *entry = arg;
  struct usb_raw_data_io_s *io;

  while (dev->fd >= 0)
    {
      io = (struct usb_raw_data_io_s *)
            host_raw_fifoalloc(&entry->fifo);

      if (io)
        {
          if (entry->halted)
            {
              host_raw_epclearhalt(dev->fd, entry->raw_epid);
              entry->halted = false;
            }

          io->inner.ep = entry->raw_epid;
          io->inner.flags = 0;
          io->inner.length = USB_RAW_EP_MAX_LEN;
          io->inner.length = host_raw_epread(dev->fd, &io->inner);
          USB_RAW_FIFO_PUSH(&entry->fifo);
        }
      else
        {
          if (!entry->halted)
            {
              host_raw_epsethalt(dev->fd, entry->raw_epid);
              entry->halted = true;
            }

          usleep(10);
        }
    }

  host_raw_fifodelete(&entry->fifo);

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_usbdev_init(uint32_t speed)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;
  const char *device = USB_RAW_DEVICE;
  const char *driver = USB_RAW_DRIVER;
  int fd;

  fd = host_raw_open();
  if (fd < 0)
    {
      ERROR("USB raw open error");
      return -1;
    }

  host_raw_init(fd, speed, driver, device);
  host_raw_run(fd);
  host_raw_vbusdraw(fd, 0x32);
  host_raw_configure(fd);
  dev->fd = fd;

  host_raw_fifocreate(&dev->eps_entry[0].fifo,
                      (sizeof(struct host_usb_ctrlreq_s)
                      + USB_RAW_EP0_MAX_LEN),
                      USB_RAW_RX_BUF_NUM);

  return pthread_create(&dev->ep0_thread, NULL,
                        host_raw_ep0handle, NULL);
}

int host_usbdev_deinit(void)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;
  host_raw_close(dev->fd);
  return 0;
}

int host_usbdev_epconfig(uint8_t epno,
                         const struct host_usb_epdesc_s *epdesc)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;
  struct usb_endpoint_descriptor raw_epdesc;
  struct usb_raw_ep_entry_s *entry;
  int ret = -1;

  if (dev->fd < 0)
    {
      ERROR("USB raw not enable");
      return ret;
    }

  if (epno > USB_RAW_EPS_NUM_MAX)
    {
      ERROR("USB raw ep num error");
      return ret;
    }

  if (epdesc->mxpacketsize > USB_RAW_EP_MAX_LEN)
    {
      ERROR("USB raw ep max packet size error");
      return ret;
    }

  host_raw_getepdesc(&raw_epdesc, epdesc);

  if (!host_raw_check_epaddress(&raw_epdesc))
    {
      ERROR("USB raw check ep address fail");
      return ret;
    }

  ret = host_raw_epenable(dev->fd, &raw_epdesc);
  if (ret < 0)
    {
      ERROR("USB raw ep enable fail");
      return ret;
    }

  entry = &dev->eps_entry[epno];
  entry->addr = epdesc->addr;
  entry->raw_epaddr = raw_epdesc.bEndpointAddress;
  entry->raw_epid = ret;
  entry->halted = false;

  if (USB_RAW_EP_DIR(epdesc->addr) == USB_DIR_OUT)
    {
      host_raw_fifocreate(&entry->fifo,
                          sizeof(struct usb_raw_data_io_s),
                          USB_RAW_RX_BUF_NUM);

      ret = pthread_create(&entry->ep_thread, NULL,
                           host_raw_ephandle,
                           (void *)entry);
    }

  return ret;
}

int host_usbdev_epdisable(uint8_t epno)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;
  int ret = -1;

  if (dev->fd >= 0)
    {
      struct usb_raw_ep_entry_s *entry = &dev->eps_entry[epno];
      ret = host_raw_epdisable(dev->fd, entry->raw_epid);
    }

  return ret;
}

int host_usbdev_pullup(bool enable)
{
  /* not support */

  return -1;
}

int host_usbdev_epstall(uint8_t epno, bool resume)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;
  int ret = -1;

  if (dev->fd >= 0)
    {
      if (!resume && epno == 0)
        {
          ret = host_raw_ep0stall(dev->fd);
        }
    }

  return ret;
}

int host_usbdev_epcancel(uint8_t epno)
{
  /* not support */

  return -1;
}

int host_usbdev_epwrite(uint8_t epno, uint8_t flags,
                        uint8_t *data, uint16_t len)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;
  struct usb_raw_ep_entry_s *entry;
  struct usb_raw_ep_io_s *io;
  int ret = -1;

  if (dev->fd < 0)
    {
      ERROR("USB raw not enable");
      return -1;
    }

  if (epno > USB_RAW_EPS_NUM_MAX)
    {
      ERROR("USB raw ep num error");
      return ret;
    }

  entry = &dev->eps_entry[epno];

  io = malloc(sizeof(struct usb_raw_ep_io_s) + len);
  if (!io)
    {
      ERROR("Host usb malloc ep write io fail");
      return -1;
    }

  io->flags = flags;
  io->length = len;
  io->ep = entry->raw_epid;
  memcpy(io->data, data, len);

  if (epno == 0)
    {
      ret = host_raw_ep0write(dev->fd, io);
    }
  else
    {
      ret = host_raw_epwrite(dev->fd, io);
    }

  free(io);

  return ret;
}

uint8_t *host_usbdev_epread(uint8_t epno, uint16_t *len)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;
  struct usb_raw_ep_entry_s *entry;
  struct usb_raw_ep_io_s *io;

  if (dev->fd < 0)
    {
      ERROR("USB raw not enable");
      return NULL;
    }

  if (epno > USB_RAW_EPS_NUM_MAX)
    {
      ERROR("USB raw ep num error");
      return NULL;
    }

  entry = &dev->eps_entry[epno];

  io = (struct usb_raw_ep_io_s *)
        host_raw_fiforead(&entry->fifo);
  if (io)
    {
      *len = io->length;
      return io->data;
    }

  return NULL;
}

struct host_usb_ctrlreq_s *host_usbdev_ep0read(void)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;

  if (dev->fd < 0)
    {
      ERROR("USB raw not enable");
      return NULL;
    }

  return (struct host_usb_ctrlreq_s *)
          host_raw_fiforead(&dev->eps_entry[0].fifo);
}

void host_usbdev_epread_end(uint8_t epno)
{
  struct usb_raw_gadget_dev_t *dev = &g_raw_gadget_dev;
  USB_RAW_FIFO_POP(&dev->eps_entry[epno].fifo);
}
