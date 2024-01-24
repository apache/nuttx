/****************************************************************************
 * drivers/usbdev/usbdev_fs.c
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
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/mutex.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/composite.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wqueue.h>

#include "composite.h"
#include "usbdev_fs.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container to support a list of requests */

struct usbdev_fs_req_s
{
  sq_entry_t               node;    /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;     /* The contained request */
  uint16_t                 offset;  /* Offset to valid data in the RX request */
};

/* Manage char device non blocking io */

typedef struct usbdev_fs_waiter_sem_s
{
  sem_t                              sem;
  FAR struct usbdev_fs_waiter_sem_s *next;
} usbdev_fs_waiter_sem_t;

/* This structure describes the char device */

struct usbdev_fs_ep_s
{
  uint8_t                     crefs;      /* Count of opened instances */
  bool                        unlinked;   /* Indicates if the driver has been unlinked */
  mutex_t                     lock;       /* Enforces device exclusive access */
  FAR struct usbdev_ep_s     *ep;         /* EP entry */
  FAR struct usbdev_fs_dev_s *dev;        /* USB device */
  FAR usbdev_fs_waiter_sem_t *sems;       /* List of blocking request */
  struct sq_queue_s           reqq;       /* Available request containers */
  FAR struct usbdev_fs_req_s *reqbuffer;  /* Request buffer */
  FAR struct pollfd          *fds[CONFIG_USBDEV_FS_NPOLLWAITERS];
};

struct usbdev_fs_dev_s
{
  FAR struct composite_dev_s *cdev;
  uint8_t                     config;
  struct work_s               work;
  struct usbdev_devinfo_s     devinfo;
  FAR struct usbdev_fs_ep_s  *eps;
  bool                        uninitialized;
};

struct usbdev_fs_driver_s
{
  struct usbdevclass_driver_s drvr;
  struct usbdev_fs_dev_s      dev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* USB class device *********************************************************/

static int usbdev_fs_classbind(FAR struct usbdevclass_driver_s *driver,
                               FAR struct usbdev_s *dev);
static void usbdev_fs_classunbind(FAR struct usbdevclass_driver_s *driver,
                                  FAR struct usbdev_s *dev);
static int usbdev_fs_classsetup(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev,
                                FAR const struct usb_ctrlreq_s *ctrl,
                                FAR uint8_t *dataout, size_t outlen);
static void
usbdev_fs_classdisconnect(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev);
static void usbdev_fs_classsuspend(FAR struct usbdevclass_driver_s *driver,
                                   FAR struct usbdev_s *dev);
static void usbdev_fs_classresume(FAR struct usbdevclass_driver_s *driver,
                                  FAR struct usbdev_s *dev);

/* Char device Operations ***************************************************/

static int usbdev_fs_open(FAR struct file *filep);
static int usbdev_fs_close(FAR struct file *filep);
static ssize_t usbdev_fs_read(FAR struct file *filep, FAR char *buffer,
                              size_t len);
static ssize_t usbdev_fs_write(FAR struct file *filep,
                               FAR const char *buffer, size_t len);
static int usbdev_fs_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB class device *********************************************************/

static const struct usbdevclass_driverops_s g_usbdev_fs_classops =
{
  usbdev_fs_classbind,        /* bind */
  usbdev_fs_classunbind,      /* unbind */
  usbdev_fs_classsetup,       /* setup */
  usbdev_fs_classdisconnect,  /* disconnect */
  usbdev_fs_classsuspend,     /* suspend */
  usbdev_fs_classresume       /* resume */
};

/* Char device **************************************************************/

static const struct file_operations g_usbdev_fs_fops =
{
  usbdev_fs_open,  /* open */
  usbdev_fs_close, /* close */
  usbdev_fs_read,  /* read */
  usbdev_fs_write, /* write */
  NULL,            /* seek */
  NULL,            /* ioctl */
  NULL,            /* mmap */
  NULL,            /* truncate */
  usbdev_fs_poll   /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_fs_notify
 *
 * Description:
 *   Notify threads waiting to read device. This function must be called
 *   with interrupt disabled.
 *
 ****************************************************************************/

static void usbdev_fs_notify(FAR struct usbdev_fs_ep_s *fs_ep,
                             pollevent_t eventset)
{
  /* Notify all of the waiting readers */

  FAR usbdev_fs_waiter_sem_t *cur_sem = fs_ep->sems;
  while (cur_sem != NULL)
    {
      nxsem_post(&cur_sem->sem);
      cur_sem = cur_sem->next;
    }

  fs_ep->sems = NULL;

  /* Notify all poll/select waiters */

  poll_notify(fs_ep->fds, CONFIG_USBDEV_FS_NPOLLWAITERS, eventset);
}

/****************************************************************************
 * Name: usbdev_fs_submit_wrreq
 *
 * Description:
 *   Handle completion of write request on the bulk IN endpoint.
 *
 ****************************************************************************/

static int usbdev_fs_submit_wrreq(FAR struct usbdev_ep_s *ep,
                                  FAR struct usbdev_fs_req_s *container,
                                  uint16_t len)
{
  FAR struct usbdev_req_s *req = container->req;

  req->len   = len;
  req->flags = 0;
  req->priv  = container;
  return EP_SUBMIT(ep, req);
}

/****************************************************************************
 * Name: usbdev_fs_submit_rdreq
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.
 *
 ****************************************************************************/

static int usbdev_fs_submit_rdreq(FAR struct usbdev_ep_s *ep,
                                  FAR struct usbdev_fs_req_s *container)
{
  FAR struct usbdev_req_s *req = container->req;

  req->len = ep->maxpacket;
  return EP_SUBMIT(ep, req);
}

/****************************************************************************
 * Name: usbdev_fs_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.
 *
 ****************************************************************************/

static void usbdev_fs_rdcomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  FAR struct usbdev_fs_req_s *container;
  FAR struct usbdev_fs_ep_s *fs_ep;
  irqstate_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract references to private data */

  fs_ep     = (FAR struct usbdev_fs_ep_s *)ep->fs;
  container = (FAR struct usbdev_fs_req_s *)req->priv;

  /* Process the received data unless this is some unusual condition */

  switch (req->result)
    {
      case 0: /* Normal completion */

      usbtrace(TRACE_CLASSRDCOMPLETE, sq_count(&fs_ep->reqq));

      /* Restart request due to empty frame received */

      if (req->xfrd <= 0)
        {
          goto restart_req;
        }

      /* Queue request and notify readers */

      flags = enter_critical_section();

      /* Put request on RX pending queue */

      container->offset = 0;
      sq_addlast(&container->node, &fs_ep->reqq);

      usbdev_fs_notify(fs_ep, POLLIN);

      leave_critical_section(flags);
      return;

      case -ESHUTDOWN: /* Disconnection */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
      return;

      default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED),
               (uint16_t)-req->result);
      break;
    };

restart_req:

  /* Restart request */

  usbdev_fs_submit_rdreq(fs_ep->ep, container);
}

/****************************************************************************
 * Name: usbdev_fs_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void usbdev_fs_wrcomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  FAR struct usbdev_fs_req_s *container;
  FAR struct usbdev_fs_ep_s *fs_ep;
  irqstate_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req || !req->priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract references to private data */

  fs_ep     = (FAR struct usbdev_fs_ep_s *)ep->fs;
  container = (FAR struct usbdev_fs_req_s *)req->priv;

  /* Return the write request to the free list */

  flags = enter_critical_section();
  sq_addlast(&container->node, &fs_ep->reqq);

  /* Check for termination condition */

  switch (req->result)
    {
      case OK: /* Normal completion */
        {
          usbtrace(TRACE_CLASSWRCOMPLETE, sq_count(&fs_ep->reqq));

          /* Notify all waiting writers that write req is available */

          usbdev_fs_notify(fs_ep, POLLOUT);
        }
        break;

      case -ESHUTDOWN: /* Disconnection */
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRSHUTDOWN),
                   sq_count(&fs_ep->reqq));
        }
        break;

      default: /* Some other error occurred */
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRUNEXPECTED),
                   (uint16_t)-req->result);
        }
        break;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbdev_fs_blocking_io
 *
 * Description:
 *   Handle read/write blocking io.
 *
 ****************************************************************************/

static int usbdev_fs_blocking_io(FAR struct usbdev_fs_ep_s *fs_ep,
                                 FAR usbdev_fs_waiter_sem_t **list,
                                 FAR struct sq_queue_s *queue)
{
  usbdev_fs_waiter_sem_t sem;
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();

  if (!sq_empty(queue))
    {
      /* Queue not empty after all */

      leave_critical_section(flags);
      return 0;
    }

  nxsem_init(&sem.sem, 0, 0);

  /* Register waiter semaphore */

  sem.next = *list;
  *list = &sem;

  leave_critical_section(flags);

  nxmutex_unlock(&fs_ep->lock);

  /* Wait for USB device to notify */

  ret = nxsem_wait(&sem.sem);

  /* Interrupted wait, unregister semaphore
   * TODO ensure that lock wait does not fail (ECANCELED)
   */

  nxmutex_lock(&fs_ep->lock);

  if (ret < 0)
    {
      flags = enter_critical_section();

      FAR usbdev_fs_waiter_sem_t *cur_sem = *list;

      if (cur_sem == &sem)
        {
          *list = sem.next;
        }
      else
        {
          while (cur_sem)
            {
              if (cur_sem->next == &sem)
                {
                  cur_sem->next = sem.next;
                  break;
                }

              cur_sem = cur_sem->next;
            }
        }

      leave_critical_section(flags);
      nxmutex_unlock(&fs_ep->lock);
    }

  nxsem_destroy(&sem.sem);
  return ret;
}

/****************************************************************************
 * Name: usbdev_fs_open
 *
 * Description:
 *   Open usbdev fs device. Only one open() instance is supported.
 *
 ****************************************************************************/

static int usbdev_fs_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_fs_ep_s *fs_ep = inode->i_private;
  int ret;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&fs_ep->lock);
  if (ret < 0)
    {
      return ret;
    }

  finfo("entry: <%s> %d\n", inode->i_name, fs_ep->crefs);

  fs_ep->crefs += 1;

  assert(fs_ep->crefs != 0);

  nxmutex_unlock(&fs_ep->lock);
  return ret;
}

/****************************************************************************
 * Name: usbdev_fs_close
 *
 * Description:
 *   Close usbdev fs device.
 *
 ****************************************************************************/

static int usbdev_fs_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_fs_ep_s *fs_ep = inode->i_private;
  FAR struct usbdev_fs_dev_s *fs = fs_ep->dev;
  int ret;
  int i;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&fs_ep->lock);
  if (ret < 0)
    {
      return ret;
    }

  finfo("entry: <%s> %d\n", inode->i_name, fs_ep->crefs);

  fs_ep->crefs -= 1;

  assert(fs_ep->crefs >= 0);

  if (fs_ep->unlinked && fs_ep->crefs == 0)
    {
      bool do_free = true;

      nxmutex_destroy(&fs_ep->lock);
      for (i = 0; i < fs->devinfo.nendpoints; i++)
        {
          if (fs->eps[i].crefs > 0)
            {
              do_free = false;
            }
        }

      if (do_free && fs->uninitialized)
        {
          FAR struct usbdev_fs_driver_s *alloc = container_of(
                       fs, FAR struct usbdev_fs_driver_s, dev);

          kmm_free(fs->eps);
          fs->eps = NULL;
          kmm_free(alloc);
        }
    }
  else
    {
      nxmutex_unlock(&fs_ep->lock);
    }

  return OK;
}

/****************************************************************************
 * Name: usbdev_fs_read
 *
 * Description:
 *   Read usbdev fs device.
 *
 ****************************************************************************/

static ssize_t usbdev_fs_read(FAR struct file *filep, FAR char *buffer,
                              size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_fs_ep_s *fs_ep = inode->i_private;
  size_t retlen = 0;
  irqstate_t flags;
  int ret;

  ret = nxmutex_lock(&fs_ep->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the usbdev device has been unbind */

  if (fs_ep->unlinked)
    {
      nxmutex_unlock(&fs_ep->lock);
      return -ENOTCONN;
    }

  /* Check for available data */

  if (sq_empty(&fs_ep->reqq))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          nxmutex_unlock(&fs_ep->lock);
          return -EAGAIN;
        }

      do
        {
          /* RX queue seems empty. Check again with interrupts disabled */

          ret = usbdev_fs_blocking_io(
            fs_ep, &fs_ep->sems, &fs_ep->reqq);
          if (ret < 0)
            {
              return ret;
            }
        }
      while (sq_empty(&fs_ep->reqq));
    }

  /* Device ready for read */

  while (!sq_empty(&fs_ep->reqq))
    {
      FAR struct usbdev_fs_req_s *container;
      uint16_t reqlen;

      /* Process each packet in the priv->reqq list */

      container = container_of(sq_peek(&fs_ep->reqq),
                               struct usbdev_fs_req_s, node);

      reqlen = container->req->xfrd - container->offset;

      if (reqlen > len)
        {
          /* Output buffer full */

          if (buffer != NULL)
            {
              memcpy(&buffer[retlen],
                     &container->req->buf[container->offset],
                     len);
            }

          container->offset += len;
          retlen += len;
          break;
        }

      if (buffer != NULL)
        {
          memcpy(&buffer[retlen],
                 &container->req->buf[container->offset], reqlen);
        }

      retlen += reqlen;
      len -= reqlen;

      /* The entire packet was processed and may be removed from the
       * pending RX list.
       */

      /* FIXME use atomic queue primitives ? */

      flags = enter_critical_section();
      sq_remfirst(&fs_ep->reqq);
      leave_critical_section(flags);

      ret = usbdev_fs_submit_rdreq(fs_ep->ep, container);
      if (ret < 0)
        {
          /* TODO handle error */

          PANIC();
        }

      /* The container buffer length is less than the maximum length.
       * It is an independent packet of requests and needs to be
       * returned directly.
       */

      if (reqlen < fs_ep->ep->maxpacket)
        {
          break;
        }
    }

  nxmutex_unlock(&fs_ep->lock);
  return retlen;
}

/****************************************************************************
 * Name: usbdev_fs_write
 *
 * Description:
 *   Write usbdev fs device.
 *
 ****************************************************************************/

static ssize_t usbdev_fs_write(FAR struct file *filep,
                               FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_fs_ep_s *fs_ep = inode->i_private;
  FAR struct usbdev_fs_req_s *container;
  FAR struct usbdev_req_s *req;
  irqstate_t flags;
  int wlen = 0;
  int ret;

  ret = nxmutex_lock(&fs_ep->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the usbdev device has been unbind */

  if (fs_ep->unlinked)
    {
      nxmutex_unlock(&fs_ep->lock);
      return -ENOTCONN;
    }

  /* Check for available write request */

  if (sq_empty(&fs_ep->reqq))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
        }

      do
        {
          /* TX queue seems empty. Check again with interrupts disabled */

          ret = usbdev_fs_blocking_io(
            fs_ep, &fs_ep->sems, &fs_ep->reqq);
          if (ret < 0)
            {
              return ret;
            }
        }
      while (sq_empty(&fs_ep->reqq));
    }

  /* Device ready for write */

  while (!sq_empty(&fs_ep->reqq))
    {
      uint16_t cur_len;

      /* Get available TX request slot */

      flags = enter_critical_section();

      container = container_of(sq_remfirst(&fs_ep->reqq),
                               struct usbdev_fs_req_s, node);

      leave_critical_section(flags);

      req = container->req;

      /* Fill the request with data */

      if (len > fs_ep->ep->maxpacket)
        {
          cur_len = fs_ep->ep->maxpacket;
        }
      else
        {
          cur_len = len;
        }

      memcpy(req->buf, &buffer[wlen], cur_len);

      /* Then submit the request to the endpoint */

      ret = usbdev_fs_submit_wrreq(fs_ep->ep, container, cur_len);
      if (ret < 0)
        {
          /* TODO add tx request back in txfree queue */

          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL),
                   (uint16_t)-ret);
          PANIC();
          break;
        }

      wlen += cur_len;
      len -= cur_len;
      if (len == 0)
        {
          break;
        }
    }

  ret = wlen;

errout:
  nxmutex_unlock(&fs_ep->lock);
  return ret;
}

/****************************************************************************
 * Name: usbdev_fs_poll
 *
 * Description:
 *   Poll usbdev fs device.
 *
 ****************************************************************************/

static int usbdev_fs_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_fs_ep_s *fs_ep = inode->i_private;
  pollevent_t eventset;
  irqstate_t flags;
  int ret;
  int i;

  ret = nxmutex_lock(&fs_ep->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!setup)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
      goto errout;
    }

  /* FIXME only parts of this function required interrupt disabled */

  flags = enter_critical_section();

  /* This is a request to set up the poll. Find an available
   * slot for the poll structure reference
   */

  for (i = 0; i < CONFIG_USBDEV_FS_NPOLLWAITERS; i++)
    {
      /* Find an available slot */

      if (!fs_ep->fds[i])
        {
          /* Bind the poll structure and this slot */

          fs_ep->fds[i] = fds;
          fds->priv    = &fs_ep->fds[i];
          break;
        }
    }

  if (i >= CONFIG_USBDEV_FS_NPOLLWAITERS)
    {
      fds->priv = NULL;
      ret       = -EBUSY;
      goto exit_leave_critical;
    }

  eventset = 0;

  /* Check if the usbdev device has been unbind */

  if (fs_ep->unlinked)
    {
      eventset |= POLLHUP;
    }

  /* Notify the POLLIN/POLLOUT event if at least one request is available */

  else if (!sq_empty(&fs_ep->reqq))
    {
      if (USB_ISEPIN(fs_ep->ep->eplog))
        {
          eventset |= POLLOUT;
        }
      else
        {
          eventset |= POLLIN;
        }
    }

  poll_notify(fs_ep->fds, CONFIG_USBDEV_FS_NPOLLWAITERS, eventset);

exit_leave_critical:
  leave_critical_section(flags);
errout:
  nxmutex_unlock(&fs_ep->lock);
  return ret;
}

/****************************************************************************
 * Name: usbdev_fs_connect
 *
 * Description:
 *   Notify usbdev fs device connect state.
 *
 ****************************************************************************/

static void usbdev_fs_connect(FAR struct usbdev_fs_dev_s *fs, int connect)
{
  FAR struct usbdev_devinfo_s *devinfo = &fs->devinfo;
  FAR struct usbdev_fs_ep_s *fs_ep;
  uint16_t cnt;

  irqstate_t flags = enter_critical_section();

  if (connect)
    {
      /* Notify poll/select with POLLPRI */

      for (cnt = 0; cnt < devinfo->nendpoints; cnt++)
        {
          fs_ep = &fs->eps[cnt];
          poll_notify(fs_ep->fds, CONFIG_USBDEV_FS_NPOLLWAITERS, POLLPRI);
        }
    }
  else
    {
      /* Notify all of the char device */

      for (cnt = 0; cnt < devinfo->nendpoints; cnt++)
        {
          fs_ep = &fs->eps[cnt];
          usbdev_fs_notify(fs_ep, POLLERR | POLLHUP);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: usbdev_fs_ep_bind
 *
 * Description:
 *   Bind usbdev fs device.
 *
 ****************************************************************************/

static int usbdev_fs_ep_bind(FAR struct usbdev_s *dev, uint8_t epno,
                             FAR const struct usbdev_epinfo_s *epinfo,
                             FAR struct usbdev_fs_ep_s *fs_ep)
{
#ifdef CONFIG_USBDEV_DUALSPEED
  uint16_t reqsize = epinfo->hssize;
#else
  uint16_t reqsize = epinfo->fssize;
#endif
  uint16_t i;

  /* Initialize fs ep lock */

  nxmutex_init(&fs_ep->lock);

  /* Initialize request queue */

  sq_init(&fs_ep->reqq);

  /* Pre-allocate the endpoint */

  fs_ep->ep = DEV_ALLOCEP(dev, epno,
                          USB_ISEPIN(epinfo->desc.addr),
                          epinfo->desc.attr & USB_EP_ATTR_XFERTYPE_MASK);
  if (fs_ep->ep == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINALLOCFAIL), 0);
      return -ENODEV;
    }

  fs_ep->ep->fs = fs_ep;

  /* Initialize request buffer */

  fs_ep->reqbuffer =
    kmm_zalloc(sizeof(struct usbdev_fs_req_s) * epinfo->reqnum);
  if (!fs_ep->reqbuffer)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), 0);
      return -ENOMEM;
    }

  for (i = 0; i < epinfo->reqnum; i++)
    {
      FAR struct usbdev_fs_req_s *container;

      container = &fs_ep->reqbuffer[i];
      container->req = usbdev_allocreq(fs_ep->ep, reqsize);
      if (container->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
          return -ENOMEM;
        }

      container->req->priv = container;

      if (USB_ISEPIN(epinfo->desc.addr))
        {
          container->req->callback = usbdev_fs_wrcomplete;
          sq_addlast(&container->node, &fs_ep->reqq);
        }
    }

  fs_ep->crefs = 0;
  return 0;
}

/****************************************************************************
 * Name: usbdev_fs_ep_unbind
 *
 * Description:
 *   Unbind usbdev fs endpoint.
 *
 ****************************************************************************/

static void usbdev_fs_ep_unbind(FAR const char *devname,
                                FAR struct usbdev_s *dev,
                                FAR const struct usbdev_epinfo_s *epinfo,
                                FAR struct usbdev_fs_ep_s *fs_ep)
{
  uint16_t i;

  /* Release request buffer */

  nxmutex_lock(&fs_ep->lock);

  if (fs_ep->reqbuffer)
    {
      for (i = 0; i < epinfo->reqnum; i++)
        {
          FAR struct usbdev_fs_req_s *container =
            &fs_ep->reqbuffer[i];
          if (container->req)
            {
              usbdev_freereq(fs_ep->ep, container->req);
            }
        }

      kmm_free(fs_ep->reqbuffer);
      fs_ep->reqbuffer = NULL;
    }

  sq_init(&fs_ep->reqq);

  /* Release endpoint */

  if (fs_ep->ep != NULL)
    {
      fs_ep->ep->fs = NULL;
      DEV_FREEEP(dev, fs_ep->ep);
      fs_ep->ep = NULL;
    }

  unregister_driver(devname);
  fs_ep->unlinked = true;

  /* Notify the usbdev device has been unbind */

  poll_notify(fs_ep->fds, CONFIG_USBDEV_FS_NPOLLWAITERS, POLLHUP | POLLERR);

  if (fs_ep->crefs <= 0)
    {
      nxmutex_unlock(&fs_ep->lock);
      nxmutex_destroy(&fs_ep->lock);
    }
  else
    {
      nxmutex_unlock(&fs_ep->lock);
    }
}

/****************************************************************************
 * Name: usbdev_fs_classresetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void usbdev_fs_classresetconfig(FAR struct usbdev_fs_dev_s *fs)
{
  FAR struct usbdev_devinfo_s *devinfo = &fs->devinfo;
  uint16_t i;

  /* Are we configured? */

  if (fs->config != COMPOSITE_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      usbdev_fs_connect(fs, 0);

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

      for (i = 0; i < devinfo->nendpoints; i++)
        {
          EP_DISABLE(fs->eps[i].ep);
        }
    }
}

/****************************************************************************
 * Name: usbdev_fs_register_driver
 *
 * Description:
 *   Register the driver after successful set configuration.
 *
 ****************************************************************************/

static void usbdev_fs_register_driver(FAR void *arg)
{
  FAR struct usbdev_fs_dev_s *fs = arg;
  FAR struct usbdev_devinfo_s *devinfo = &fs->devinfo;
  int i;

  for (i = 0; i < devinfo->nendpoints; i++)
    {
      char devname[32];
      int ret;

      snprintf(devname, sizeof(devname), "%s/ep%d",
               devinfo->name, i + 1);
      ret = register_driver(devname, &g_usbdev_fs_fops, 0666, &fs->eps[i]);
      if (ret < 0)
        {
          uerr("Failed to register driver:%s, ret:%d\n", devname, ret);
          while (i--)
            {
              snprintf(devname, sizeof(devname), "%s/ep%d",
                       devinfo->name, i + 1);
              unregister_driver(devname);
            }

          break;
        }
    }
}

/****************************************************************************
 * Name: usbdev_fs_classsetconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int usbdev_fs_classsetconfig(FAR struct usbdev_fs_dev_s *fs,
                                    uint8_t config)
{
  FAR struct usbdev_devinfo_s *devinfo = &fs->devinfo;
  struct usb_epdesc_s epdesc;
  bool hispeed = false;
  uint16_t i;
  uint16_t j;
  int ret;

  /* Discard the previous configuration data */

  usbdev_fs_classresetconfig(fs);

  /* Was this a request to simply discard the current configuration? */

  if (config == COMPOSITE_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

#ifdef CONFIG_USBDEV_DUALSPEED
  hispeed = (fs->cdev->usbdev->speed == USB_SPEED_HIGH);
#endif

  for (i = 0; i < devinfo->nendpoints; i++)
    {
      FAR struct usbdev_fs_ep_s *fs_ep = &fs->eps[i];

      usbdev_copy_epdesc(&epdesc, devinfo->epno[i],
                         hispeed, devinfo->epinfos[i]);
      ret = EP_CONFIGURE(fs_ep->ep, &epdesc,
                         (i == (devinfo->nendpoints - 1)));
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINCONFIGFAIL), 0);
          goto errout;
        }

      fs_ep->ep->priv = fs;

      if (USB_ISEPOUT(fs_ep->ep->eplog))
        {
          for (j = 0; j < devinfo->epinfos[i]->reqnum; j++)
            {
              FAR struct usbdev_fs_req_s *container;
              container = &fs_ep->reqbuffer[j];
              container->req->callback = usbdev_fs_rdcomplete;
              usbdev_fs_submit_rdreq(fs_ep->ep, container);
            }
        }
    }

  fs->config = config;
  work_queue(HPWORK, &fs->work, usbdev_fs_register_driver, fs, 0);

  /* We are successfully configured. Char device is now active */

  usbdev_fs_connect(fs, 1);
  return OK;

errout:
  usbdev_fs_classresetconfig(fs);
  return ret;
}

/****************************************************************************
 * Name: usbdev_fs_classbind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int usbdev_fs_classbind(FAR struct usbdevclass_driver_s *driver,
                               FAR struct usbdev_s *dev)
{
  FAR struct usbdev_fs_driver_s *fs_drvr = container_of(
    driver, FAR struct usbdev_fs_driver_s, drvr);
  FAR struct usbdev_fs_dev_s *fs = &fs_drvr->dev;
  FAR struct usbdev_devinfo_s *devinfo = &fs->devinfo;
  uint16_t i;
  int ret;

  /* Bind the composite device */

  fs->cdev = dev->ep0->priv;

  /* Initialize fs eqs */

  fs->eps = kmm_zalloc(devinfo->nendpoints * sizeof(struct usbdev_fs_ep_s));
  if (fs->eps == NULL)
    {
      uerr("Failed to malloc fs eqs");
      return -ENOMEM;
    }

  for (i = 0; i < devinfo->nendpoints; i++)
    {
      fs->eps[i].dev = fs;
      ret = usbdev_fs_ep_bind(dev,
                              devinfo->epno[i],
                              devinfo->epinfos[i],
                              &fs->eps[i]);
      if (ret < 0)
        {
          uerr("Failed to bind fs ep");
          goto errout;
        }
    }

  return OK;

errout:
  usbdev_fs_classunbind(driver, dev);
  return ret;
}

/****************************************************************************
 * Name: usbdev_fs_classunbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void usbdev_fs_classunbind(FAR struct usbdevclass_driver_s *driver,
                                  FAR struct usbdev_s *dev)
{
  FAR struct usbdev_fs_driver_s *fs_drvr = container_of(
    driver, FAR struct usbdev_fs_driver_s, drvr);
  FAR struct usbdev_fs_dev_s *fs = &fs_drvr->dev;
  FAR struct usbdev_devinfo_s *devinfo = &fs->devinfo;
  bool do_free = true;
  char devname[32];
  uint16_t i;

  if (fs->eps != NULL)
    {
      for (i = 0; i < devinfo->nendpoints; i++)
        {
          snprintf(devname, sizeof(devname), "%s/ep%d",
                   devinfo->name, i + 1);
          usbdev_fs_ep_unbind(devname, dev,
                              devinfo->epinfos[i],
                              &fs->eps[i]);
          if (fs->eps[i].crefs > 0)
            {
              do_free = false;
            }
        }

      if (do_free)
        {
          kmm_free(fs->eps);
          fs->eps = NULL;
        }
    }

  fs->cdev = NULL;
}

/****************************************************************************
 * Name: usbdev_fs_classsetup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int usbdev_fs_classsetup(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev,
                                FAR const struct usb_ctrlreq_s *ctrl,
                                FAR uint8_t *dataout, size_t outlen)
{
  FAR struct usbdev_fs_driver_s *fs_drvr = container_of(
    driver, FAR struct usbdev_fs_driver_s, drvr);
  FAR struct usbdev_fs_dev_s *fs = &fs_drvr->dev;
  uint16_t value;
  int ret = -EOPNOTSUPP;

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD &&
      ctrl->req == USB_REQ_SETCONFIGURATION &&
      ctrl->type == 0)
    {
      ret = usbdev_fs_classsetconfig(fs, value);
    }
  else
    {
      /* send to userspace???? */
    }

  /* Returning a negative value will cause a STALL */

  return ret;
}

/****************************************************************************
 * Name: usbdev_fs_classdisconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void
usbdev_fs_classdisconnect(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  FAR struct usbdev_fs_driver_s *fs_drvr = container_of(
    driver, FAR struct usbdev_fs_driver_s, drvr);
  FAR struct usbdev_fs_dev_s *fs = &fs_drvr->dev;

  /* Reset the configuration */

  usbdev_fs_classresetconfig(fs);
}

/****************************************************************************
 * Name: usbdev_fs_classsuspend
 *
 * Description:
 *   Handle the USB suspend event.
 *
 ****************************************************************************/

static void usbdev_fs_classsuspend(FAR struct usbdevclass_driver_s *driver,
                                   FAR struct usbdev_s *dev)
{
  FAR struct usbdev_fs_driver_s *fs_drvr = container_of(
    driver, FAR struct usbdev_fs_driver_s, drvr);
  FAR struct usbdev_fs_dev_s *fs = &fs_drvr->dev;

  usbdev_fs_connect(fs, 0);
}

/****************************************************************************
 * Name: usbdev_fs_classresume
 *
 * Description:
 *   Handle the USB resume event.
 *
 ****************************************************************************/

static void usbdev_fs_classresume(FAR struct usbdevclass_driver_s *driver,
                                  FAR struct usbdev_s *dev)
{
  FAR struct usbdev_fs_driver_s *fs_drvr = container_of(
    driver, FAR struct usbdev_fs_driver_s, drvr);
  FAR struct usbdev_fs_dev_s *fs = &fs_drvr->dev;

  usbdev_fs_connect(fs, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbclass_classobject
 *
 * Description:
 *   Register USB driver and return the class object.
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

int usbdev_fs_classobject(int minor,
                          FAR struct usbdev_devinfo_s *devinfo,
                          FAR struct usbdevclass_driver_s **classdev)
{
  FAR struct usbdev_fs_driver_s *alloc;

  if (devinfo->nendpoints > CONFIG_USBDEV_FS_EPNUM)
    {
      uerr("class epnum error");
      return -EINVAL;
    }

  alloc = kmm_zalloc(sizeof(struct usbdev_fs_driver_s));
  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Save the caller provided device description */

  memcpy(&alloc->dev.devinfo, devinfo,
         sizeof(struct usbdev_devinfo_s));

  /* Initialize the USB class driver structure */

  alloc->drvr.ops = &g_usbdev_fs_classops;

  *classdev = &alloc->drvr;
  return OK;
}

/****************************************************************************
 * Name: usbdev_fs_classuninitialize
 *
 * Description:
 *   Free allocated class memory
 *
 ****************************************************************************/

void usbdev_fs_classuninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  FAR struct usbdev_fs_driver_s *alloc = container_of(
    classdev, FAR struct usbdev_fs_driver_s, drvr);
  FAR struct usbdev_fs_dev_s *fs = &alloc->dev;
  int i;

  fs->uninitialized = true;
  for (i = 0; i < fs->devinfo.nendpoints; i++)
    {
      if (fs->eps != NULL && fs->eps[i].crefs > 0)
        {
          return;
        }
    }

  kmm_free(alloc);
}

/****************************************************************************
 * Name: usbdev_fs_initialize
 *
 * Description:
 *   USBDEV fs initialize
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

FAR void *usbdev_fs_initialize(FAR const struct usbdev_devdescs_s *devdescs,
                               FAR struct composite_devdesc_s *pdevice)
{
  return composite_initialize(devdescs, pdevice, 1);
}

/****************************************************************************
 * Name: usbdev_fs_uninitialize
 *
 * Description:
 *   USBDEV fs uninitialize
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

void usbdev_fs_uninitialize(FAR void *handle)
{
  composite_uninitialize(handle);
}
