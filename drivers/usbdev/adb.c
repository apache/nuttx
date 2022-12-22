/****************************************************************************
 * drivers/usbdev/adb.c
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
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <poll.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/mutex.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/adb.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_USBADB_BOARD_SERIALSTR
#include <nuttx/board.h>
#endif

#ifdef CONFIG_USBADB_COMPOSITE
#  include <nuttx/usb/composite.h>
#  include "composite.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FIXME use minor for char device npath */

#define USBADB_CHARDEV_PATH "/dev/adb0"

/* USB Controller */

#ifdef CONFIG_USBDEV_SELFPOWERED
#  define USBADB_SELFPOWERED USB_CONFIG_ATTR_SELFPOWER
#else
#  define USBADB_SELFPOWERED (0)
#endif

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
#  define USBADB_REMOTEWAKEUP USB_CONFIG_ATTR_WAKEUP
#else
#  define USBADB_REMOTEWAKEUP (0)
#endif

/* Buffer big enough for any of our descriptors (the config descriptor is the
 * biggest).
 */

#define USBADB_MXDESCLEN           (64)
#define USBADB_MAXSTRLEN           (USBADB_MXDESCLEN-2)

/* Device descriptor values */

#define USBADB_VERSIONNO           (0x0101) /* Device version number 1.1 (BCD) */

/* String language */

#define USBADB_STR_LANGUAGE        (0x0409) /* en-us */

/* Descriptor strings.  If there serial device is part of a composite device
 * then the manufacturer, product, and serial number strings will be provided
 * by the composite logic.
 */

#ifndef CONFIG_USBADB_COMPOSITE
#  define USBADB_MANUFACTURERSTRID (1)
#  define USBADB_PRODUCTSTRID      (2)
#  define USBADB_SERIALSTRID       (3)
#  define USBADB_CONFIGSTRID       (4)
#  define USBADB_INTERFACESTRID    (5)
#else
#  define USBADB_INTERFACESTRID    (1)
#  define USBADB_NSTRIDS           (1)
#endif

#define USBADB_NCONFIGS          (1)
#define USBADB_CONFIGID          (1)
#define USBADB_CONFIGIDNONE      (0)

/* Length of ADB descriptor */

#define USBADB_DESC_TOTALLEN 32

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Manage char device non blocking io */

typedef struct adb_char_waiter_sem_s
{
  sem_t sem;
  FAR struct adb_char_waiter_sem_s *next;
} adb_char_waiter_sem_t;

/* Container to support a list of requests */

struct usbadb_wrreq_s
{
  FAR sq_entry_t node;          /* Implements a singly linked list */
  FAR struct usbdev_req_s *req; /* The contained request */
};

struct usbadb_rdreq_s
{
  FAR sq_entry_t node;          /* Implements a singly linked list */
  FAR struct usbdev_req_s *req; /* The contained request */
  uint16_t offset;              /* Offset to valid data in the RX request */
};

/* This structure describes the internal state of the driver */

struct usbdev_adb_s
{
  FAR struct usbdev_s *usbdev;      /* usbdev driver pointer */
#ifdef CONFIG_USBADB_COMPOSITE
  struct usbdev_devinfo_s devinfo;
#endif

  FAR struct usbdev_ep_s  *epbulkin;  /* Bulk IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkout; /* Bulk OUT endpoint structure */

  FAR struct usbdev_req_s *ctrlreq;   /* Preallocated control request */

  uint8_t config;                     /* USB Configuration number */

  struct sq_queue_s txfree;           /* Available write request containers */
  struct sq_queue_s rxpending;        /* Pending read request containers */

  /* Pre-allocated request containers. The write requests will be
   * linked in a free list (txfree), and used to send requests to
   * EPBULKIN; Read requests will be queued in the EBULKOUT.
   */

  struct usbadb_wrreq_s wrreqs[CONFIG_USBADB_NWRREQS];
  struct usbadb_rdreq_s rdreqs[CONFIG_USBADB_NRDREQS];

  /* Char device driver */

  mutex_t                lock;     /* Enforces device exclusive access */
  adb_char_waiter_sem_t *rdsems;   /* List of blocking readers */
  adb_char_waiter_sem_t *wrsems;   /* List of blocking writers */
  uint8_t               crefs;     /* Count of opened instances */
  FAR struct pollfd *fds[CONFIG_USBADB_NPOLLWAITERS];
};

struct adb_driver_s
{
  struct usbdevclass_driver_s drvr;
  struct usbdev_adb_s dev;
};

struct adb_cfgdesc_s
{
#ifndef CONFIG_USBADB_COMPOSITE
  struct usb_cfgdesc_s cfgdesc;        /* Configuration descriptor */
#endif
  struct usb_ifdesc_s  ifdesc;         /* ADB interface descriptor */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* USB class device *********************************************************/

static int     usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static int     usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev,
                 FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout,
                 size_t outlen);
static void    usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);

static void    usbclass_suspend(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    usbclass_resume(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);

static FAR struct usbdev_req_s *usbclass_allocreq(FAR struct usbdev_ep_s *ep,
                                                  uint16_t len);

/* Char device Operations ***************************************************/

static int adb_char_open(FAR struct file *filep);
static int adb_char_close(FAR struct file *filep);

static ssize_t adb_char_read(FAR struct file *filep, FAR char *buffer,
                               size_t len);
static ssize_t adb_char_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len);
static int adb_char_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup);

static void adb_char_notify_readers(FAR struct usbdev_adb_s *priv);

static void adb_char_on_connect(FAR struct usbdev_adb_s *priv, int connect);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB class device *********************************************************/

static const struct usbdevclass_driverops_s g_adb_driverops =
{
  usbclass_bind,       /* bind */
  usbclass_unbind,     /* unbind */
  usbclass_setup,      /* setup */
  usbclass_disconnect, /* disconnect */
  usbclass_suspend,    /* suspend */
  usbclass_resume      /* resume */
};

/* Char device **************************************************************/

static const struct file_operations g_adb_fops =
{
  adb_char_open,  /* open */
  adb_char_close, /* close */
  adb_char_read,  /* read */
  adb_char_write, /* write */
  NULL,           /* seek */
  NULL,           /* ioctl */
  NULL,           /* truncate */
  adb_char_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/* USB descriptor ***********************************************************/

#ifndef CONFIG_USBADB_COMPOSITE
static const struct usb_devdesc_s g_adb_devdesc =
{
  .len = USB_SIZEOF_DEVDESC,         /* Descriptor length */
  .type = USB_DESC_TYPE_DEVICE,      /* Descriptor type */
  .usb =                             /* USB version */
  {
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  .classid = 0,                               /* Device class */
  .subclass = 0,                              /* Device sub-class */
  .protocol = 0,                              /* Device protocol */
  .mxpacketsize = CONFIG_USBADB_EP0MAXPACKET, /* Max packet size (ep0) */
  .vendor =                                   /* Vendor ID */
  {
    LSBYTE(CONFIG_USBADB_VENDORID),
    MSBYTE(CONFIG_USBADB_VENDORID)
  },
  .product =                         /* Product ID */
  { LSBYTE(CONFIG_USBADB_PRODUCTID),
    MSBYTE(CONFIG_USBADB_PRODUCTID)
  },
  .device =                          /* Device ID */
  { LSBYTE(USBADB_VERSIONNO),
    MSBYTE(USBADB_VERSIONNO)
  },
  .imfgr = USBADB_MANUFACTURERSTRID, /* Manufacturer */
  .iproduct = USBADB_PRODUCTSTRID,   /* Product */
  .serno = USBADB_SERIALSTRID,       /* Serial number */
  .nconfigs = 1                      /* Number of configurations */
};
#endif

static const struct adb_cfgdesc_s g_adb_cfgdesc =
{
#ifndef CONFIG_USBADB_COMPOSITE
  {
    .len          = USB_SIZEOF_CFGDESC,   /* Descriptor length    */
    .type         = USB_DESC_TYPE_CONFIG, /* Descriptor type      */
    .totallen     =
    {
      LSBYTE(USBADB_DESC_TOTALLEN),       /* LS Total length      */
      MSBYTE(USBADB_DESC_TOTALLEN)        /* MS Total length      */
    },
    .ninterfaces  = 1,                    /* Number of interfaces */
    .cfgvalue     = 1,                    /* Configuration value  */
    .icfg         = USBADB_CONFIGSTRID,   /* Configuration        */
    .attr         = USB_CONFIG_ATTR_ONE |
                    USBADB_SELFPOWERED  |
                    USBADB_REMOTEWAKEUP,  /* Attributes           */

    .mxpower      = (CONFIG_USBDEV_MAXPOWER + 1) / 2 /* Max power (mA/2) */
  },
#endif
  {
    .len            = USB_SIZEOF_IFDESC,
    .type           = USB_DESC_TYPE_INTERFACE,
    .ifno           = 0,
    .alt            = 0,
    .neps           = 2,
    .classid        = USB_CLASS_VENDOR_SPEC,
    .subclass       = 0x42,
    .protocol       = 0x01,
    .iif            = USBADB_INTERFACESTRID
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbclass_allocreq
 *
 * Description:
 *   Allocate request buffer for a specified endpoint.
 *
 ****************************************************************************/

static FAR struct usbdev_req_s *usbclass_allocreq(FAR struct usbdev_ep_s *ep,
                                                  uint16_t len)
{
  FAR struct usbdev_req_s *req;

  req = EP_ALLOCREQ(ep);
  if (req != NULL)
    {
      req->len = len;
      req->buf = EP_ALLOCBUFFER(ep, len);

      if (req->buf == NULL)
        {
          EP_FREEREQ(ep, req);
          req = NULL;
        }
    }

  return req;
}

/****************************************************************************
 * Name: usbclass_copy_epdesc
 *
 * Description:
 *   Copies the requested Endpoint Description into the buffer given.
 *   Returns the number of Bytes filled in ( sizeof(struct usb_epdesc_s) ).
 *
 ****************************************************************************/

static int usbclass_copy_epdesc(int epid, FAR struct usb_epdesc_s *epdesc,
                                FAR struct usbdev_devinfo_s *devinfo,
                                bool hispeed)
{
#ifndef CONFIG_USBDEV_DUALSPEED
  UNUSED(hispeed);
#endif

  epdesc->len  = USB_SIZEOF_EPDESC;           /* Descriptor length */
  epdesc->type = USB_DESC_TYPE_ENDPOINT;      /* Descriptor type */
  epdesc->attr = USB_EP_ATTR_XFER_BULK |      /* Endpoint attributes */
                 USB_EP_ATTR_NO_SYNC   |
                 USB_EP_ATTR_USAGE_DATA;
  epdesc->interval = 0;                       /* Interval */

  if (epid == USBADB_EP_BULKIN_IDX) /* Bulk IN endpoint */
    {
      /* Endpoint address */

#ifdef CONFIG_USBADB_COMPOSITE
      epdesc->addr = USB_EPIN(devinfo->epno[USBADB_EP_BULKIN_IDX]);
#else
      epdesc->addr = USB_EPIN(CONFIG_USBADB_EPBULKIN);
#endif

#ifdef CONFIG_USBDEV_DUALSPEED
      if (hispeed)
        {
          /* Maximum packet size (high speed) */

          epdesc->mxpacketsize[0] = LSBYTE(CONFIG_USBADB_EPBULKIN_HSSIZE);
          epdesc->mxpacketsize[1] = MSBYTE(CONFIG_USBADB_EPBULKIN_HSSIZE);
        }
      else
#endif
        {
          /* Maximum packet size (full speed) */

          epdesc->mxpacketsize[0] = LSBYTE(CONFIG_USBADB_EPBULKIN_FSSIZE);
          epdesc->mxpacketsize[1] = MSBYTE(CONFIG_USBADB_EPBULKIN_FSSIZE);
        }
    }
  else /* USBADB_EP_BULKOUT_IDX: Bulk OUT endpoint */
    {
      /* Endpoint address */

#ifdef CONFIG_USBADB_COMPOSITE
      epdesc->addr = USB_EPOUT(devinfo->epno[USBADB_EP_BULKOUT_IDX]);
#else
      epdesc->addr = USB_EPOUT(CONFIG_USBADB_EPBULKOUT);
#endif

#ifdef CONFIG_USBDEV_DUALSPEED
      if (hispeed)
        {
          /* Maximum packet size (high speed) */

          epdesc->mxpacketsize[0] = LSBYTE(CONFIG_USBADB_EPBULKOUT_HSSIZE);
          epdesc->mxpacketsize[1] = MSBYTE(CONFIG_USBADB_EPBULKOUT_HSSIZE);
        }
      else
#endif
        {
          /* Maximum packet size (full speed) */

          epdesc->mxpacketsize[0] = LSBYTE(CONFIG_USBADB_EPBULKOUT_FSSIZE);
          epdesc->mxpacketsize[1] = MSBYTE(CONFIG_USBADB_EPBULKOUT_FSSIZE);
        }
    }

  return sizeof(struct usb_epdesc_s);
}

/****************************************************************************
 * Name: usb_adb_submit_rdreq
 *
 * Description:
 *   Submits the bulk OUT read request. Takes care not to submit the request
 *   when the RX packet buffer is already in use.
 *
 * Input Parameters:
 *   priv: pointer to ADB device driver structure
 *
 * Returned Value:
 *   The return value of the EP_SUBMIT operation
 *
 ****************************************************************************/

static int usb_adb_submit_rdreq(FAR struct usbdev_adb_s *priv,
                                FAR struct usbadb_rdreq_s *rdcontainer)
{
  FAR struct usbdev_req_s *req;
  FAR struct usbdev_ep_s *ep;
  int ret;

  DEBUGASSERT(priv != NULL && rdcontainer != NULL);

  req      = rdcontainer->req;
  DEBUGASSERT(req != NULL);

  /* Requeue the read request */

  ep       = priv->epbulkout;
  req->len = ep->maxpacket;
  ret      = EP_SUBMIT(ep, req);
  if (ret != OK)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT),
                              (uint16_t)-req->result);
    }

  return ret;
}

/****************************************************************************
 * Name: usb_adb_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void usb_adb_wrcomplete(FAR struct usbdev_ep_s *ep,
                               FAR struct usbdev_req_s *req)
{
  FAR struct usbadb_wrreq_s *wrcontainer;
  FAR struct usbdev_adb_s *priv;
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

  priv        = (FAR struct usbdev_adb_s *)ep->priv;
  wrcontainer = (FAR struct usbadb_wrreq_s *)req->priv;

  /* Return the write request to the free list */

  flags = enter_critical_section();
  sq_addlast(&wrcontainer->node, &priv->txfree);

  /* Check for termination condition */

  switch (req->result)
    {
    case OK: /* Normal completion */
      {
        usbtrace(TRACE_CLASSWRCOMPLETE, sq_count(&priv->txfree));

        /* Notify all waiting writers that write req is available */

        adb_char_waiter_sem_t *cur_sem = priv->wrsems;
        while (cur_sem != NULL)
          {
            nxsem_post(&cur_sem->sem);
            cur_sem = cur_sem->next;
          }

        priv->wrsems = NULL;

        /* Notify all poll/select waiters */

        poll_notify(priv->fds, CONFIG_USBADB_NPOLLWAITERS, POLLOUT);
      }
      break;

    case -ESHUTDOWN: /* Disconnection */
      {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRSHUTDOWN),
                 sq_count(&priv->txfree));
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
 * Name: usb_adb_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.
 *
 ****************************************************************************/

static void usb_adb_rdcomplete(FAR struct usbdev_ep_s *ep,
                               FAR struct usbdev_req_s *req)
{
  FAR struct usbadb_rdreq_s *rdcontainer;
  FAR struct usbdev_adb_s *priv;
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

  priv        = (FAR struct usbdev_adb_s *)ep->priv;
  rdcontainer = (FAR struct usbadb_rdreq_s *)req->priv;

  /* Process the received data unless this is some unusual condition */

  switch (req->result)
    {
    case 0: /* Normal completion */

      usbtrace(TRACE_CLASSRDCOMPLETE, sq_count(&priv->rxpending));

      /* Restart request due to either no reader or
       * empty frame received.
       */

      if (priv->crefs == 0)
        {
          uwarn("drop frame\n");
          goto restart_req;
        }

      if (req->xfrd <= 0)
        {
          goto restart_req;
        }

      /* Queue request and notify readers */

      flags = enter_critical_section();

      /* Put request on RX pending queue */

      rdcontainer->offset = 0;
      sq_addlast(&rdcontainer->node, &priv->rxpending);

      adb_char_notify_readers(priv);

      leave_critical_section(flags);
      return;

    case -ESHUTDOWN: /* Disconnection */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
      return;

    default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED),
               (uint16_t)-req->result);
      goto restart_req;
    };

restart_req:

  /* Restart request */

  usb_adb_submit_rdreq(priv, rdcontainer);
}

/****************************************************************************
 * Name: usbclass_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void usbclass_resetconfig(FAR struct usbdev_adb_s *priv)
{
  /* Are we configured? */

  if (priv->config != USBADB_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      adb_char_on_connect(priv, 0);

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

      EP_DISABLE(priv->epbulkin);
      EP_DISABLE(priv->epbulkout);
    }

  priv->config = USBADB_CONFIGIDNONE;
}

/****************************************************************************
 * Name: usbclass_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int usbclass_setconfig(FAR struct usbdev_adb_s *priv, uint8_t config)
{
  struct usb_epdesc_s epdesc;
  bool hispeed = false;
  int i;
  int ret = 0;

#ifdef CONFIG_DEBUG_FEATURES
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

#ifdef CONFIG_USBDEV_DUALSPEED
  hispeed = (priv->usbdev->speed == USB_SPEED_HIGH);
#endif

  if (config == priv->config)
    {
      /* Already configured -- Do nothing */

      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALREADYCONFIGURED), 0);
      return 0;
    }

  /* Discard the previous configuration data */

  usbclass_resetconfig(priv);

  /* Was this a request to simply discard the current configuration? */

  if (config == USBADB_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

  /* We only accept one configuration */

  if (config != USBADB_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

  /* Configure the IN bulk endpoint */

#ifdef CONFIG_USBADB_COMPOSITE
  usbclass_copy_epdesc(USBADB_EP_BULKIN_IDX, &epdesc,
                       &priv->devinfo, hispeed);
#else
  usbclass_copy_epdesc(USBADB_EP_BULKIN_IDX, &epdesc, NULL, hispeed);
#endif

  ret = EP_CONFIGURE(priv->epbulkin, &epdesc, false);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Configure the OUT bulk endpoint */

#ifdef CONFIG_USBADB_COMPOSITE
  usbclass_copy_epdesc(USBADB_EP_BULKOUT_IDX, &epdesc,
                       &priv->devinfo, hispeed);
#else
  usbclass_copy_epdesc(USBADB_EP_BULKOUT_IDX, &epdesc, NULL, hispeed);
#endif
  ret = EP_CONFIGURE(priv->epbulkout, &epdesc, true);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Queue read requests in the bulk OUT endpoint */

  for (i = 0; i < CONFIG_USBADB_NRDREQS; i++)
    {
      priv->rdreqs[i].req->callback = usb_adb_rdcomplete;
      ret = usb_adb_submit_rdreq(priv, &priv->rdreqs[i]);
      if (ret != OK)
        {
          /* TODO cancel submitted requests */

          goto errout;
        }
    }

  /* We are successfully configured. Char device is now active */

  priv->config = config;
  adb_char_on_connect(priv, 1);
  return OK;

errout:
  usbclass_resetconfig(priv);
  return ret;
}

/****************************************************************************
 * Name: usbclass_ep0incomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void usbclass_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT),
               (uint16_t)-req->result);
    }
}

#ifdef CONFIG_USBDEV_DUALSPEED
static int16_t usbclass_mkcfgdesc(FAR uint8_t *buf,
                                  FAR struct usbdev_devinfo_s *devinfo,
                                  uint8_t speed, uint8_t type)
#else
static int16_t usbclass_mkcfgdesc(FAR uint8_t *buf,
                                  FAR struct usbdev_devinfo_s *devinfo)
#endif
{
  bool hispeed = false;
  FAR struct usb_epdesc_s *epdesc;
  FAR struct adb_cfgdesc_s *dest;

#ifdef CONFIG_USBDEV_DUALSPEED
  hispeed = (speed == USB_SPEED_HIGH);

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      hispeed = !hispeed;
    }
#endif

  dest = (FAR struct adb_cfgdesc_s *)buf;
  epdesc = (FAR struct usb_epdesc_s *)(buf + sizeof(g_adb_cfgdesc));

  memcpy(dest, &g_adb_cfgdesc, sizeof(g_adb_cfgdesc));

#ifdef CONFIG_USBADB_COMPOSITE
  usbclass_copy_epdesc(USBADB_EP_BULKIN_IDX, &epdesc[0], devinfo, hispeed);
  usbclass_copy_epdesc(USBADB_EP_BULKOUT_IDX, &epdesc[1], devinfo, hispeed);
#else
  usbclass_copy_epdesc(USBADB_EP_BULKIN_IDX, &epdesc[0], NULL, hispeed);
  usbclass_copy_epdesc(USBADB_EP_BULKOUT_IDX, &epdesc[1], NULL, hispeed);
#endif

#ifdef CONFIG_USBADB_COMPOSITE
  /* For composite device, apply possible offset to the interface numbers */

  dest->ifdesc.ifno = devinfo->ifnobase;
  dest->ifdesc.iif  = devinfo->strbase + USBADB_INTERFACESTRID;
#endif

  return sizeof(g_adb_cfgdesc)+2*USB_SIZEOF_EPDESC;
}

static int usbclass_mkstrdesc(uint8_t id, FAR struct usb_strdesc_s *strdesc)
{
  FAR uint8_t *data = (FAR uint8_t *)(strdesc + 1);
  FAR const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_USBADB_COMPOSITE
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len  = 4;
        strdesc->type = USB_DESC_TYPE_STRING;
        data[0] = LSBYTE(USBADB_STR_LANGUAGE);
        data[1] = MSBYTE(USBADB_STR_LANGUAGE);
        return 4;
      }

    case USBADB_MANUFACTURERSTRID:
      str = CONFIG_USBADB_VENDORSTR;
      break;

    case USBADB_PRODUCTSTRID:
      str = CONFIG_USBADB_PRODUCTSTR;
      break;

    case USBADB_SERIALSTRID:
#ifdef CONFIG_USBADB_BOARD_SERIALSTR
      str = board_usbdev_serialstr();
#else
      str = CONFIG_USBADB_SERIALSTR;
#endif
      break;

    case USBADB_CONFIGSTRID:
      str = CONFIG_USBADB_CONFIGSTR;
      break;
#endif

    /* Composite driver removes offset before calling mkstrdesc() */

    case USBADB_INTERFACESTRID:
      str = CONFIG_USBADB_INTERFACESTR;
      break;

    default:
      return -EINVAL;
    }

  /* The string is utf16-le.  The poor man's utf-8 to utf16-le
   * conversion below will only handle 7-bit en-us ascii
   */

  len = strlen(str);
  if (len > (USBADB_MAXSTRLEN / 2))
    {
      len = (USBADB_MAXSTRLEN / 2);
    }

  for (i = 0, ndata = 0; i < len; i++, ndata += 2)
    {
      data[ndata]     = str[i];
      data[ndata + 1] = 0;
    }

  strdesc->len  = ndata + 2;
  strdesc->type = USB_DESC_TYPE_STRING;
  return strdesc->len;
}

/****************************************************************************
 * USB Class Driver Methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbclass_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                         FAR struct usbdev_s *dev)
{
  int ret;
  int i;
  uint16_t reqlen;
  irqstate_t flags;
  FAR struct usbdev_adb_s *priv = &((FAR struct adb_driver_s *)driver)->dev;

  usbtrace(TRACE_CLASSBIND, 0);

  priv->usbdev = dev;

  priv->ctrlreq = usbclass_allocreq(dev->ep0, USBADB_MXDESCLEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      return -ENOMEM;
    }

  priv->ctrlreq->callback = usbclass_ep0incomplete;

  /* Pre-allocate all endpoints... the endpoints will not be functional
   * until the SET CONFIGURATION request is processed in usbclass_setconfig.
   * This is done here because there may be calls to kmm_malloc and the SET
   * CONFIGURATION processing probably occurs within interrupt handling
   * logic where kmm_malloc calls will fail.
   */

  /* Pre-allocate the IN bulk endpoint */

  priv->epbulkin = DEV_ALLOCEP(dev,
#ifdef CONFIG_USBADB_COMPOSITE
      USB_EPIN(priv->devinfo.epno[USBADB_EP_BULKIN_IDX]),
#else
      USB_EPIN(CONFIG_USBADB_EPBULKIN),
#endif
      true,
      USB_EP_ATTR_XFER_BULK);

  if (!priv->epbulkin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Pre-allocate the OUT bulk endpoint */

  priv->epbulkout = DEV_ALLOCEP(dev,
#ifdef CONFIG_USBADB_COMPOSITE
      USB_EPOUT(priv->devinfo.epno[USBADB_EP_BULKOUT_IDX]),
#else
      USB_EPOUT(CONFIG_USBADB_EPBULKOUT),
#endif
      false,
      USB_EP_ATTR_XFER_BULK);

  if (!priv->epbulkout)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Pre-allocate read requests. The buffer size is one full packet. */

#ifdef CONFIG_USBDEV_DUALSPEED
  reqlen = CONFIG_USBADB_EPBULKOUT_HSSIZE;
#else
  reqlen = CONFIG_USBADB_EPBULKOUT_FSSIZE;
#endif

  for (i = 0; i < CONFIG_USBADB_NRDREQS; i++)
    {
      FAR struct usbadb_rdreq_s *rdcontainer;

      rdcontainer      = &priv->rdreqs[i];
      rdcontainer->req = usbclass_allocreq(priv->epbulkout, reqlen);
      if (rdcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }

      rdcontainer->offset        = 0;
      rdcontainer->req->priv     = rdcontainer;
      rdcontainer->req->callback = usb_adb_rdcomplete;
    }

  /* Pre-allocate write requests. The buffer size is one full packet. */

#ifdef CONFIG_USBDEV_DUALSPEED
  reqlen = CONFIG_USBADB_EPBULKIN_HSSIZE;
#else
  reqlen = CONFIG_USBADB_EPBULKIN_FSSIZE;
#endif

  for (i = 0; i < CONFIG_USBADB_NWRREQS; i++)
    {
      FAR struct usbadb_wrreq_s *wrcontainer;

      wrcontainer      = &priv->wrreqs[i];
      wrcontainer->req = usbclass_allocreq(priv->epbulkin, reqlen);
      if (wrcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }

      wrcontainer->req->priv     = wrcontainer;
      wrcontainer->req->callback = usb_adb_wrcomplete;

      flags = enter_critical_section();
      sq_addlast(&wrcontainer->node, &priv->txfree);
      leave_critical_section(flags);
    }

  /* Report if we are selfpowered (unless we are part of a
   * composite device)
   */

#ifndef CONFIG_USBADB_COMPOSITE
#ifdef CONFIG_USBDEV_SELFPOWERED
  DEV_SETSELFPOWERED(dev);
#endif

  /* And pull-up the data line for the soft connect function (unless we are
   * part of a composite device)
   */

  DEV_CONNECT(dev);
#endif
  return OK;

errout:
  usbclass_unbind(driver, dev);
  return ret;
}

/****************************************************************************
 * Name: usbclass_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev)
{
  usbtrace(TRACE_CLASSUNBIND, 0);

  #warning Missing logic
}

/****************************************************************************
 * Name: usbclass_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev,
                          FAR const struct usb_ctrlreq_s *ctrl,
                          FAR uint8_t *dataout, size_t outlen)
{
  uint16_t value;
  uint16_t len;
  int ret = -EOPNOTSUPP;
  bool cfg_req = true;

  FAR struct usbdev_adb_s *priv;
  FAR struct usbdev_req_s *ctrlreq;

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0 || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = &((FAR struct adb_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv || !priv->ctrlreq)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return -ENODEV;
    }
#endif

  ctrlreq = priv->ctrlreq;

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  len   = GETUINT16(ctrl->len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, GETUINT16(ctrl->index), len);

  switch (ctrl->type & USB_REQ_TYPE_MASK)
    {
      case USB_REQ_TYPE_STANDARD:
        {
          switch (ctrl->req)
            {
#ifndef CONFIG_USBADB_COMPOSITE
            case USB_REQ_GETDESCRIPTOR:
              {
                /* The value field specifies the descriptor type in the
                 * MS byte and the descriptor index in the LS byte
                 * (order is little endian)
                 */

                switch (ctrl->value[1])
                  {
                  /* If the device is used in as part of a composite
                   * device, then the device descriptor is provided by logic
                   * in the composite device implementation.
                   */

                  case USB_DESC_TYPE_DEVICE:
                    {
                      ret = USB_SIZEOF_DEVDESC;
                      memcpy(ctrlreq->buf, &g_adb_devdesc, ret);
                    }
                    break;

                  case USB_DESC_TYPE_DEVICEQUALIFIER:
                    break;
                  case USB_DESC_TYPE_OTHERSPEEDCONFIG:
                    break;

                  /* If the serial device is used in as part of a composite
                   * device, then the configuration descriptor is provided by
                   * logic in the composite device implementation.
                   */

                  case USB_DESC_TYPE_CONFIG:
                    {
#ifndef CONFIG_USBDEV_DUALSPEED
                      ret = usbclass_mkcfgdesc(ctrlreq->buf, NULL);
#else
                      ret = usbclass_mkcfgdesc(ctrlreq->buf, NULL,
                                               dev->speed, ctrl->req);
#endif
                    }
                    break;

                  /* If the serial device is used in as part of a composite
                   * device, then the language string descriptor is provided
                   * by logic in the composite device implementation.
                   */

                  case USB_DESC_TYPE_STRING:
                    {
                      /* index == language code. */

                      ret =
                      usbclass_mkstrdesc(ctrl->value[0],
                                         (FAR struct usb_strdesc_s *)
                                         ctrlreq->buf);
                    }
                    break;

                  default:
                    {
                      usbtrace(
                        TRACE_CLSERROR(USBSER_TRACEERR_GETUNKNOWNDESC),
                        value);
                    }
                    break;
                  }
              }
              break;

            /* If the serial device is used in as part of a composite device,
             * then the overall composite class configuration is managed by
             * logic in the composite device implementation.
             */

            case USB_REQ_GETCONFIGURATION:
              {
                if (ctrl->type == USB_DIR_IN)
                  {
                    *(FAR uint8_t *)ctrlreq->buf = priv->config;
                    ret = 1;
                  }
              }
              break;
#endif /* !CONFIG_USBADB_COMPOSITE */

            case USB_REQ_SETCONFIGURATION:
              {
                if (ctrl->type == 0)
                  {
                    ret = usbclass_setconfig(priv, value);
                    cfg_req = false;
                  }
              }
              break;

            default:
              usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ),
                       ctrl->req);
              break;
            }
        }
        break;

      case USB_REQ_TYPE_CLASS:
        {
          /* ADB-Specific Requests */

          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                   ctrl->req);
          break;
        }

      default:
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDTYPE),
                   ctrl->type);
        }
    }

#ifndef CONFIG_USBADB_COMPOSITE
  /* Respond to the setup command if data was returned.  On an error return
   * value (ret < 0), the USB driver will stall.
   */

  if (ret >= 0 && cfg_req)
    {
      ctrlreq->len   = (len < ret) ? len : ret;
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;

      /* Send the response -- either directly to the USB controller or
       * indirectly in the case where this class is a member of a composite
       * device.
       */

      ret = EP_SUBMIT(dev->ep0, ctrlreq);

      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
          usbclass_ep0incomplete(dev->ep0, ctrlreq);
        }
    }
#else
  /* Composite should send only one request for USB_REQ_SETCONFIGURATION.
   * Hence ADB driver cannot submit to ep0; composite has to handle it.
   */

  #warning composite_ep0submit() seems broken so skip it in case of composite
#endif /* !CONFIG_USBADB_COMPOSITE */

  /* Returning a negative value will cause a STALL */

  return ret;
}

/****************************************************************************
 * Name: usbclass_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev)
{
  FAR struct usbdev_adb_s *priv;
  irqstate_t flags;

  usbtrace(TRACE_CLASSDISCONNECT, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = &((FAR struct adb_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* FIXME do we have to lock interrupts here ? */

  flags = enter_critical_section();

  /* Reset the configuration */

  usbclass_resetconfig(priv);

  leave_critical_section(flags);

  /* Perform the soft connect function so that we will we can be
   * re-enumerated (unless we are part of a composite device)
   */

#ifndef CONFIG_USBDEV_COMPOSITE
  DEV_CONNECT(dev);
#endif
}

/****************************************************************************
 * Name: usbclass_suspend
 *
 * Description:
 *   Handle the USB suspend event.
 *
 ****************************************************************************/

static void usbclass_suspend(FAR struct usbdevclass_driver_s *driver,
                             FAR struct usbdev_s *dev)
{
  FAR struct usbdev_adb_s *priv = &((FAR struct adb_driver_s *)driver)->dev;

  usbtrace(TRACE_CLASSSUSPEND, 0);

  if (priv->config != USBADB_CONFIGIDNONE)
    {
      adb_char_on_connect(priv, 0);
    }
}

/****************************************************************************
 * Name: usbclass_resume
 *
 * Description:
 *   Handle the USB resume event.
 *
 ****************************************************************************/

static void usbclass_resume(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev)
{
  FAR struct usbdev_adb_s *priv = &((FAR struct adb_driver_s *)driver)->dev;

  usbtrace(TRACE_CLASSRESUME, 0);

  if (priv->config != USBADB_CONFIGIDNONE)
    {
      adb_char_on_connect(priv, 1);
    }
}

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

static int usbclass_classobject(int minor,
                                FAR struct usbdev_devinfo_s *devinfo,
                                FAR struct usbdevclass_driver_s **classdev)
{
  int ret;
  FAR struct adb_driver_s *alloc;

  alloc = (FAR struct adb_driver_s *)
    kmm_zalloc(sizeof(struct adb_driver_s));

  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Initialize the USB class driver structure */

#ifdef CONFIG_USBDEV_DUALSPEED
  alloc->drvr.speed          = USB_SPEED_HIGH;
#else
  alloc->drvr.speed          = USB_SPEED_FULL;
#endif

  alloc->drvr.ops   = &g_adb_driverops;

  sq_init(&alloc->dev.rxpending);
  sq_init(&alloc->dev.txfree);

#ifdef CONFIG_USBADB_COMPOSITE
  /* Save the caller provided device description (composite only) */

  memcpy(&alloc->dev.devinfo, devinfo,
         sizeof(struct usbdev_devinfo_s));
#endif

  /* Initialize the char device structure */

  nxmutex_init(&alloc->dev.lock);
  alloc->dev.crefs = 0;

  /* Register char device driver */

  /* FIXME use minor in device name */

  ret = register_driver(USBADB_CHARDEV_PATH, &g_adb_fops, 0666, &alloc->dev);
  if (ret < 0)
    {
      uerr("Failed to register char device");
      goto exit_free_driver;
    }

  *classdev = &alloc->drvr;
  return OK;

exit_free_driver:
  nxmutex_destroy(&alloc->dev.lock);
  kmm_free(alloc);
  return ret;
}

/****************************************************************************
 * Name: usbclass_uninitialize
 *
 * Description:
 *   Free allocated memory
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

static void usbclass_uninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  FAR struct adb_driver_s *alloc = container_of(
    classdev, FAR struct adb_driver_s, drvr);

  #warning FIXME Maybe missing logic here

  unregister_driver(USBADB_CHARDEV_PATH);

  kmm_free(alloc);
}

/****************************************************************************
 * Char Device Driver Methods
 ****************************************************************************/

/****************************************************************************
 * Name: adb_char_notify_readers
 *
 * Description:
 *   Notify threads waiting to read device. This function must be called
 *   with interrupt disabled.
 *
 ****************************************************************************/

static void adb_char_notify_readers(FAR struct usbdev_adb_s *priv)
{
  /* Notify all of the waiting readers */

  adb_char_waiter_sem_t *cur_sem = priv->rdsems;
  while (cur_sem != NULL)
    {
      nxsem_post(&cur_sem->sem);
      cur_sem = cur_sem->next;
    }

  priv->rdsems = NULL;

  /* Notify all poll/select waiters */

  poll_notify(priv->fds, CONFIG_USBADB_NPOLLWAITERS, POLLIN);
}

/****************************************************************************
 * Name: adb_char_open
 *
 * Description:
 *   Open adb device. Only one open() instance is supported.
 *
 ****************************************************************************/

static int adb_char_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_adb_s *priv = inode->i_private;
  int ret;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  finfo("entry: <%s> %d\n", inode->i_name, priv->crefs);

  priv->crefs += 1;

  assert(priv->crefs != 0);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: adb_char_close
 *
 * Description:
 *   Close adb device.
 *
 ****************************************************************************/

static int adb_char_close(FAR struct file *filep)
{
  int ret;
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_adb_s *priv = inode->i_private;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  finfo("entry: <%s> %d\n", inode->i_name, priv->crefs);

  priv->crefs -= 1;

  assert(priv->crefs >= 0);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: adb_char_blocking_io
 *
 * Description:
 *   Handle read/write blocking io.
 *
 ****************************************************************************/

static int adb_char_blocking_io(FAR struct usbdev_adb_s *priv,
                                FAR adb_char_waiter_sem_t *sem,
                                FAR adb_char_waiter_sem_t **slist,
                                FAR struct sq_queue_s *queue)
{
  int ret;
  irqstate_t flags;

  flags = enter_critical_section();

  if (!sq_empty(queue))
    {
      /* Queue not empty after all */

      leave_critical_section(flags);
      return 0;
    }

  /* Register waiter semaphore */

  sem->next = *slist;
  *slist = sem;

  leave_critical_section(flags);

  nxmutex_unlock(&priv->lock);

  /* Wait for USB device to notify */

  ret = nxsem_wait(&sem->sem);

  if (ret < 0)
    {
      /* Interrupted wait, unregister semaphore
       * TODO ensure that lock wait does not fail (ECANCELED)
       */

      nxmutex_lock(&priv->lock);

      flags = enter_critical_section();

      adb_char_waiter_sem_t *cur_sem = *slist;

      if (cur_sem == sem)
        {
          *slist = sem->next;
        }
      else
        {
          while (cur_sem)
            {
              if (cur_sem->next == sem)
                {
                  cur_sem->next = sem->next;
                  break;
                }
            }
        }

      leave_critical_section(flags);
      return ret;
    }

  return nxmutex_lock(&priv->lock);
}

/****************************************************************************
 * Name: adb_char_read
 *
 * Description:
 *   Read adb device.
 *
 ****************************************************************************/

static ssize_t adb_char_read(FAR struct file *filep, FAR char *buffer,
                             size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_adb_s *priv = inode->i_private;
  ssize_t ret;
  size_t retlen;
  irqstate_t flags;

  assert(len > 0 && buffer != NULL);

  if (priv->config == USBADB_CONFIGIDNONE)
    {
      /* USB device not connected */

      return -EPIPE;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check for available data */

  if (sq_empty(&priv->rxpending))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          nxmutex_unlock(&priv->lock);
          return -EAGAIN;
        }

      adb_char_waiter_sem_t sem;
      nxsem_init(&sem.sem, 0, 0);

      do
        {
          /* RX queue seems empty. Check again with interrupts disabled */

          ret = adb_char_blocking_io(
            priv, &sem, &priv->rdsems, &priv->rxpending);
          if (ret < 0)
            {
              nxsem_destroy(&sem.sem);
              return ret;
            }
        }
      while (sq_empty(&priv->rxpending));

      /* RX queue not empty and lock locked so we are the only reader */

      nxsem_destroy(&sem.sem);
    }

  /* Device ready for read */

  retlen = 0;

  while (!sq_empty(&priv->rxpending) && len > 0)
    {
      FAR struct usbadb_rdreq_s *rdcontainer;
      uint16_t reqlen;

      /* Process each packet in the priv->rxpending list */

      rdcontainer = container_of(
        sq_peek(&priv->rxpending),
        struct usbadb_rdreq_s,
        node);

      reqlen = rdcontainer->req->xfrd - rdcontainer->offset;

      if (reqlen > len)
        {
          /* Output buffer full */

          memcpy(&buffer[retlen],
                 &rdcontainer->req->buf[rdcontainer->offset],
                 len);
          rdcontainer->offset += len;
          retlen += len;
          break;
        }

      memcpy(&buffer[retlen],
             &rdcontainer->req->buf[rdcontainer->offset],
             reqlen);
      retlen += reqlen;
      len -= reqlen;

      /* The entire packet was processed and may be removed from the
       * pending RX list.
       */

      /* FIXME use atomic queue primitives ? */

      flags = enter_critical_section();
      sq_remfirst(&priv->rxpending);
      leave_critical_section(flags);

      ret = usb_adb_submit_rdreq(priv, rdcontainer);

      if (ret != OK)
        {
          /* TODO handle error */

          PANIC();
        }
    }

  nxmutex_unlock(&priv->lock);
  return retlen;
}

/****************************************************************************
 * Name: adb_char_write
 *
 * Description:
 *   Write adb device.
 *
 ****************************************************************************/

static ssize_t adb_char_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len)
{
  int ret;
  int wlen;
  FAR struct usbdev_req_s *req;
  FAR struct usbadb_wrreq_s *wrcontainer;
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_adb_s *priv = inode->i_private;

  irqstate_t flags;

  if (priv->config == USBADB_CONFIGIDNONE)
    {
      /* USB device not connected */

      return -EPIPE;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check for available write request */

  if (sq_empty(&priv->txfree))
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          ret = -EAGAIN;
          goto errout;
        }

      adb_char_waiter_sem_t sem;
      nxsem_init(&sem.sem, 0, 0);

      do
        {
          /* TX queue seems empty. Check again with interrupts disabled */

          ret = adb_char_blocking_io(
            priv, &sem, &priv->wrsems, &priv->txfree);
          if (ret < 0)
            {
              nxsem_destroy(&sem.sem);
              return ret;
            }
        }
      while (sq_empty(&priv->txfree));

      nxsem_destroy(&sem.sem);
    }

  /* Device ready for write */

  wlen = 0;

  while (len > 0 && !sq_empty(&priv->txfree))
    {
      int cur_len;

      /* Get available TX request slot */

      flags = enter_critical_section();

      wrcontainer = container_of(
        sq_remfirst(&priv->txfree),
        struct usbadb_wrreq_s,
        node);

      leave_critical_section(flags);

      req = wrcontainer->req;

      /* Fill the request with data */

      if (len > priv->epbulkin->maxpacket)
        {
          cur_len = priv->epbulkin->maxpacket;
        }
      else
        {
          cur_len = len;
        }

      memcpy(req->buf, &buffer[wlen], cur_len);

      /* Then submit the request to the endpoint */

      req->len     = cur_len;
      req->flags   = 0;
      req->priv    = wrcontainer;
      ret          = EP_SUBMIT(priv->epbulkin, req);

      if (ret != OK)
        {
          /* TODO add tx request back in txfree queue */

          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL),
                   (uint16_t)-ret);
          PANIC();
          break;
        }

      wlen += cur_len;
      len -= cur_len;
    }

  assert(wlen > 0);
  ret = wlen;

errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

static int adb_char_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct usbdev_adb_s *priv = inode->i_private;
  int ret;
  int i;
  pollevent_t eventset;
  irqstate_t flags;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = OK;

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

  for (i = 0; i < CONFIG_USBADB_NPOLLWAITERS; i++)
    {
      /* Find an available slot */

      if (!priv->fds[i])
        {
          /* Bind the poll structure and this slot */

          priv->fds[i] = fds;
          fds->priv   = &priv->fds[i];
          break;
        }
    }

  if (i >= CONFIG_USBADB_NPOLLWAITERS)
    {
      fds->priv = NULL;
      ret       = -EBUSY;
      goto exit_leave_critical;
    }

  eventset = 0;

  /* Notify the POLLOUT event if at least one request is available */

  if (!sq_empty(&priv->txfree))
    {
      eventset |= POLLOUT;
    }

  /* Notify the POLLIN event if at least one read request is pending */

  if (!sq_empty(&priv->rxpending))
    {
      eventset |= POLLIN;
    }

  poll_notify(priv->fds, CONFIG_USBADB_NPOLLWAITERS, eventset);

exit_leave_critical:
  leave_critical_section(flags);
errout:
  nxmutex_unlock(&priv->lock);
  return ret;
}

static void adb_char_on_connect(FAR struct usbdev_adb_s *priv, int connect)
{
  irqstate_t flags;
  adb_char_waiter_sem_t *cur_sem;

  flags = enter_critical_section();

  if (connect)
    {
      /* Notify poll/select with POLLIN */

      poll_notify(priv->fds, CONFIG_USBADB_NPOLLWAITERS, POLLIN);
    }
  else
    {
      /* Notify all of the char device waiting readers */

      cur_sem = priv->rdsems;
      while (cur_sem != NULL)
        {
          nxsem_post(&cur_sem->sem);
          cur_sem = cur_sem->next;
        }

      priv->rdsems = NULL;

      /* Notify all of the char device waiting writers */

      cur_sem = priv->wrsems;
      while (cur_sem != NULL)
        {
          nxsem_post(&cur_sem->sem);
          cur_sem = cur_sem->next;
        }

      priv->wrsems = NULL;

      /* Notify all poll/select waiters that a hangup occurred */

      poll_notify(priv->fds, CONFIG_USBADB_NPOLLWAITERS, POLLERR | POLLHUP);
    }

    leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_adb_initialize
 *
 * Description:
 *   Initialize the Android Debug Bridge USB device driver.
 *
 * Returned Value:
 *   0 on success, -errno on failure
 *
 ****************************************************************************/

#ifndef CONFIG_USBADB_COMPOSITE
int usbdev_adb_initialize(void)
{
  int ret;
  FAR struct usbdevclass_driver_s *classdev;
  FAR struct adb_driver_s *drvr;

  ret = usbclass_classobject(0, NULL, &classdev);
  if (ret)
    {
      nerr("usbclass_classobject failed: %d\n", ret);
      return ret;
    }

  drvr = (FAR struct adb_driver_s *)classdev;

  ret = usbdev_register(&drvr->drvr);
  if (ret)
    {
      nerr("usbdev_register failed: %d\n", ret);
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_DEVREGISTER), (uint16_t)-ret);
      usbclass_uninitialize(classdev);
      return ret;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: usbdev_adb_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration struct.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_USBADB_COMPOSITE)
void usbdev_adb_get_composite_devdesc(struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  dev->mkconfdesc          = usbclass_mkcfgdesc;
  dev->mkstrdesc           = usbclass_mkstrdesc;
  dev->classobject         = usbclass_classobject;
  dev->uninitialize        = usbclass_uninitialize;
  dev->nconfigs            = USBADB_NCONFIGS;
  dev->configid            = 1;
  dev->cfgdescsize         = sizeof(g_adb_cfgdesc)+2*USB_SIZEOF_EPDESC;
  dev->devinfo.ninterfaces = 1;
  dev->devinfo.nstrings    = USBADB_NSTRIDS;
  dev->devinfo.nendpoints  = USBADB_NUM_EPS;
}
#endif
