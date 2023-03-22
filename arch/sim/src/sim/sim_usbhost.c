/****************************************************************************
 * arch/sim/src/sim/sim_usbhost.c
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

#include <sys/types.h>
#include <sys/param.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_trace.h>

#include "sim_usbhost.h"
#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_USBHOST_BUFSIZE     256

#define RHPNDX(rh)              ((rh)->hport.hport.port)
#define RHPORT(rh)              (RHPNDX(rh)+1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum sim_hoststate_e
{
  USB_HOST_DETACHED = 0,  /* Not attached to a device */
  USB_HOST_ATTACHED,      /* Attached to a device */
  USB_HOST_ENUM,          /* Attached, enumerating */
  USB_HOST_CLASS_BOUND,   /* Enumeration complete, class bound */
};

struct sim_epinfo_s
{
  uint8_t epno:7;              /* Endpoint number */
  uint8_t dirin:1;             /* 1:IN endpoint 0:OUT endpoint */
  uint8_t devaddr:7;           /* Device address */
  uint8_t toggle:1;            /* Next data toggle */
  uint8_t interval;            /* Polling interval */
  uint8_t status;              /* Retained token status bits (for debug purposes) */
  uint16_t maxpacket:11;       /* Maximum packet size */
  uint16_t xfrtype:2;          /* See USB_EP_ATTR_XFER_* definitions in usb.h */
  uint16_t speed:2;            /* See USB_*_SPEED definitions */
  int result;                  /* The result of the transfer */
  uint32_t xfrd;               /* On completion, will hold the number of bytes transferred */
  sem_t iocsem;                /* Semaphore used to wait for transfer completion */
};

struct sim_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to struct sim_usbhost_s.
   */

  struct usbhost_driver_s       drvr;

  /* This is the hub port description understood by class drivers */

  struct usbhost_roothubport_s  hport;

  /* Root hub port status */

  volatile bool                 connected;          /* Connected to device */
  struct                        sim_epinfo_s ep0;   /* EP0 endpoint info */

  uint8_t                       state;

  volatile bool                 pscwait;            /* TRUE: Thread is waiting for port status change event */

  mutex_t                       lock;               /* Support mutually exclusive access */
  sem_t                         pscsem;             /* Semaphore to wait for port status change events */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* USB host connection operations *******************************************/

static int sim_usbhost_wait(struct usbhost_connection_s *conn,
                            struct usbhost_hubport_s **hport);
static int sim_usbhost_enumerate(struct usbhost_connection_s *conn,
                                 struct usbhost_hubport_s *hport);

/* USB host driver operations ***********************************************/

static int sim_usbhost_ep0configure(FAR struct usbhost_driver_s *drvr,
                                    usbhost_ep_t ep0, uint8_t funcaddr,
                                    uint8_t speed, uint16_t maxpacketsize);
static int sim_usbhost_epalloc(FAR struct usbhost_driver_s *drvr,
                               FAR const struct usbhost_epdesc_s *epdesc,
                               FAR usbhost_ep_t *ep);
static int sim_usbhost_epfree(FAR struct usbhost_driver_s *drvr,
                              FAR usbhost_ep_t ep);
static int sim_usbhost_alloc(FAR struct usbhost_driver_s *drvr,
                             FAR uint8_t **buffer, FAR size_t *maxlen);
static int sim_usbhost_free(FAR struct usbhost_driver_s *drvr,
                            FAR uint8_t *buffer);
static int sim_usbhost_ioalloc(FAR struct usbhost_driver_s *drvr,
                               FAR uint8_t **buffer, size_t buflen);
static int sim_usbhost_iofree(FAR struct usbhost_driver_s *drvr,
                              FAR uint8_t *buffer);
static int sim_usbhost_ctrlin(FAR struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep0,
                              FAR const struct usb_ctrlreq_s *req,
                              FAR uint8_t *buffer);
static int sim_usbhost_ctrlout(FAR struct usbhost_driver_s *drvr,
                               usbhost_ep_t ep0,
                               FAR const struct usb_ctrlreq_s *req,
                               FAR const uint8_t *buffer);
static ssize_t sim_usbhost_transfer(FAR struct usbhost_driver_s *drvr,
                                    usbhost_ep_t ep, FAR uint8_t *buffer,
                                    size_t buflen);
static int sim_usbhost_cancel(FAR struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep);
static void sim_usbhost_disconnect(FAR struct usbhost_driver_s *drvr,
                                   FAR struct usbhost_hubport_s *hport);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sim_usbhost_s g_sim_usbhost =
{
  .lock = NXMUTEX_INITIALIZER,
  .pscsem = SEM_INITIALIZER(0),
  .ep0.iocsem = SEM_INITIALIZER(1),
};

static struct usbhost_connection_s g_sim_usbconn =
{
  .wait      = sim_usbhost_wait,
  .enumerate = sim_usbhost_enumerate,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_usbhost_allocrq
 ****************************************************************************/

static struct host_usb_datareq_s *sim_usbhost_allocrq(void)
{
  struct host_usb_datareq_s *req;

  req = zalloc(sizeof(struct host_usb_datareq_s));
  if (req)
    {
      req->success = false;
    }

  return req;
}

/****************************************************************************
 * Name: sim_usbhost_freerq
 ****************************************************************************/

static void sim_usbhost_freerq(struct host_usb_datareq_s *req)
{
  DEBUGASSERT(req);
  free(req);
}

/****************************************************************************
 * Name: sim_usbhost_getctrlreq
 ****************************************************************************/

static void
sim_usbhost_getctrlreq(struct host_usb_ctrlreq_s *host_req,
                       const struct usb_ctrlreq_s *ctr_req)
{
  host_req->type = ctr_req->type;
  host_req->req = ctr_req->req;
  host_req->value = GETUINT16(ctr_req->value);
  host_req->index = GETUINT16(ctr_req->index);
  host_req->len = GETUINT16(ctr_req->len);
}

/****************************************************************************
 * Name: sim_usbhost_waittask
 ****************************************************************************/

static int sim_usbhost_waittask(int argc, char *argv[])
{
  struct usbhost_connection_s *conn = &g_sim_usbconn;
  struct usbhost_hubport_s *hport;

  for (; ; )
    {
      /* Wait for the device to change state */

      CONN_WAIT(conn, &hport);
      uinfo("%s\n", hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

          CONN_ENUMERATE(conn, hport);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: sim_usbhost_wait
 ****************************************************************************/

static int sim_usbhost_wait(struct usbhost_connection_s *conn,
                            struct usbhost_hubport_s **hport)
{
  struct sim_usbhost_s *priv = &g_sim_usbhost;
  irqstate_t flags;
  int ret;

  /* Loop until the connection state changes on one of the root hub ports or
   * until an error occurs.
   */

  flags = enter_critical_section();
  for (; ; )
    {
      /* Check for a change in the connection state on any root hub port */

      struct usbhost_hubport_s *connport;

      /* Has the connection state changed on the RH port? */

      connport = &priv->hport.hport;
      if (priv->connected != connport->connected)
        {
          /* Yes.. Return the RH port to inform the caller which
           * port has the connection change.
           */

          connport->connected = priv->connected;
          *hport = connport;
          leave_critical_section(flags);
          return OK;
        }

      /* No changes on any port. Wait for a connection/disconnection event
       * and check again
       */

      priv->pscwait = true;
      ret = nxsem_wait_uninterruptible(&priv->pscsem);
      if (ret < 0)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: sim_usbhost_enumerate
 ****************************************************************************/

static int sim_usbhost_enumerate(struct usbhost_connection_s *conn,
                                 struct usbhost_hubport_s *hport)
{
  struct sim_usbhost_s *priv = &g_sim_usbhost;
  int ret;

  DEBUGASSERT(hport);

  /* Then let the common usbhost_enumerate do the real enumeration. */

  uinfo("Enumerate the device\n");
  priv->state = USB_HOST_ENUM;
  ret = usbhost_enumerate(hport, &hport->devclass);

  /* The enumeration may fail either because of some HCD interfaces failure
   * or because the device class is not supported.  In either case, we just
   * need to perform the disconnection operation and make ready for a new
   * enumeration.
   */

  if (ret < 0)
    {
      /* Return to the disconnected state */

      uerr("ERROR: Enumeration failed: %d\n", ret);
      hport->connected = false;
    }

  return ret;
}

/****************************************************************************
 * Name: sim_usbhost_ep0configure
 ****************************************************************************/

static int sim_usbhost_ep0configure(FAR struct usbhost_driver_s *drvr,
                                    usbhost_ep_t ep0, uint8_t funcaddr,
                                    uint8_t speed, uint16_t maxpacketsize)
{
  struct sim_usbhost_s *priv = &g_sim_usbhost;
  struct sim_epinfo_s *epinfo = (struct sim_epinfo_s *)ep0;
  int ret;

  DEBUGASSERT(drvr != NULL && epinfo != NULL);

  ret = nxmutex_lock(&priv->lock);
  if (ret >= 0)
    {
      /* Remember the new device address and max packet size */

      epinfo->devaddr   = funcaddr;
      epinfo->speed     = speed;
      epinfo->maxpacket = maxpacketsize;

      nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: sim_usbhost_epalloc
 ****************************************************************************/

static int sim_usbhost_epalloc(FAR struct usbhost_driver_s *drvr,
                               FAR const struct usbhost_epdesc_s *epdesc,
                               FAR usbhost_ep_t *ep)
{
  struct sim_epinfo_s *epinfo;
  struct usbhost_hubport_s *hport;

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(drvr != 0 && epdesc != NULL && epdesc->hport != NULL &&
              ep != NULL);
  hport = epdesc->hport;

  uinfo("EP%d DIR=%s FA=%08x TYPE=%d Interval=%d MaxPacket=%d\n",
        epdesc->addr, epdesc->in ? "IN" : "OUT", hport->funcaddr,
        epdesc->xfrtype, epdesc->interval, epdesc->mxpacketsize);

  /* Allocate a endpoint information structure */

  epinfo = (struct sim_epinfo_s *)kmm_zalloc(sizeof(struct sim_epinfo_s));
  if (!epinfo)
    {
      return -ENOMEM;
    }

  /* Initialize the endpoint container (which is really just another form of
   * 'struct usbhost_epdesc_s', packed differently and with additional
   * information.  A cleaner design might just embed struct usbhost_epdesc_s
   * inside of struct sim_epinfo_s and just memcpy here.
   */

  epinfo->epno      = epdesc->addr;
  epinfo->dirin     = epdesc->in;
  epinfo->devaddr   = hport->funcaddr;
  epinfo->interval  = epdesc->interval;
  epinfo->maxpacket = epdesc->mxpacketsize;
  epinfo->xfrtype   = epdesc->xfrtype;
  epinfo->speed     = hport->speed;
  nxsem_init(&epinfo->iocsem, 0, 0);

  /* Success.. return an opaque reference to the endpoint information
   * structure instance
   */

  *ep = (usbhost_ep_t)epinfo;
  return OK;
}

/****************************************************************************
 * Name: sim_usbhost_epfree
 ****************************************************************************/

static int sim_usbhost_epfree(FAR struct usbhost_driver_s *drvr,
                              FAR usbhost_ep_t ep)
{
  struct sim_epinfo_s *epinfo = (struct sim_epinfo_s *)ep;

  /* There should not be any pending, transfers */

  DEBUGASSERT(drvr && epinfo);

  /* Free the container */

  kmm_free(epinfo);
  return OK;
}

/****************************************************************************
 * Name: sim_usbhost_alloc
 ****************************************************************************/

static int sim_usbhost_alloc(FAR struct usbhost_driver_s *drvr,
                             FAR uint8_t **buffer, FAR size_t *maxlen)
{
  DEBUGASSERT(drvr && buffer && maxlen);

  *buffer = (uint8_t *)kmm_malloc(SIM_USBHOST_BUFSIZE);
  if (*buffer)
    {
      *maxlen = SIM_USBHOST_BUFSIZE;
      return OK;
    }

  return -ENOMEM;
}

/****************************************************************************
 * Name: sim_usbhost_free
 ****************************************************************************/

static int sim_usbhost_free(FAR struct usbhost_driver_s *drvr,
                            FAR uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: sim_usbhost_ioalloc
 ****************************************************************************/

static int sim_usbhost_ioalloc(FAR struct usbhost_driver_s *drvr,
                               FAR uint8_t **buffer, size_t buflen)
{
  DEBUGASSERT(drvr && buffer && buflen > 0);

  *buffer = (uint8_t *)kumm_malloc(buflen);
  return *buffer ? OK : -ENOMEM;
}

/****************************************************************************
 * Name: sim_usbhost_iofree
 ****************************************************************************/

static int sim_usbhost_iofree(FAR struct usbhost_driver_s *drvr,
                              FAR uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

  kumm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: sim_usbhost_ctrlin
 ****************************************************************************/

static int sim_usbhost_ctrlin(FAR struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep0,
                              FAR const struct usb_ctrlreq_s *req,
                              FAR uint8_t *buffer)
{
  struct sim_usbhost_s *priv = (struct sim_usbhost_s *)drvr;
  struct sim_epinfo_s *ep0info = (struct sim_epinfo_s *)ep0;
  struct host_usb_ctrlreq_s hostreq;
  struct host_usb_datareq_s *datareq;
  uint16_t len;
  ssize_t nbytes;
  sem_t iocsem;

  DEBUGASSERT(ep0info != NULL && req != NULL);

  datareq = sim_usbhost_allocrq();
  if (!datareq)
    {
      uerr("sim_usbhost_allocrq fail\n");
      return -ENOMEM;
    }

  len = GETUINT16(req->len);

  /* Terse output only if we are tracing */

  uinfo("type: %02x req: %02x value: %02x%02x index: %02x%02x "
        "len: %04x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], len);

  sim_usbhost_getctrlreq(&hostreq, req);

  nxsem_init(&iocsem, 0, 0);

  datareq->addr = 0;
  datareq->data = buffer;
  datareq->len = len;
  datareq->priv = &iocsem;

  /* We must have exclusive access to the data structures. */

  nxmutex_lock(&priv->lock);

  nbytes = host_usbhost_ep0trans(&hostreq, datareq);

  nxmutex_unlock(&priv->lock);

  /* Wait for transfer completion */

  nxsem_wait(&iocsem);

  nbytes = datareq->xfer;

  sim_usbhost_freerq(datareq);

  return (int)nbytes;
}

/****************************************************************************
 * Name: sim_usbhost_ctrlout
 ****************************************************************************/

static int sim_usbhost_ctrlout(FAR struct usbhost_driver_s *drvr,
                               usbhost_ep_t ep0,
                               FAR const struct usb_ctrlreq_s *req,
                               FAR const uint8_t *buffer)
{
  return sim_usbhost_ctrlin(drvr, ep0, req, (uint8_t *)buffer);
}

/****************************************************************************
 * Name: sim_usbhost_transfer
 ****************************************************************************/

static ssize_t sim_usbhost_transfer(FAR struct usbhost_driver_s *drvr,
                                    usbhost_ep_t ep, FAR uint8_t *buffer,
                                    size_t buflen)
{
  struct sim_usbhost_s *priv = (struct sim_usbhost_s *)drvr;
  struct sim_epinfo_s *epinfo = (struct sim_epinfo_s *)ep;
  struct host_usb_datareq_s *datareq;
  sem_t iocsem;
  int nbytes;

  DEBUGASSERT(epinfo && buffer && buflen > 0);

  datareq = sim_usbhost_allocrq();
  if (!datareq)
    {
      uerr("sim_usbhost_allocrq fail\n");
      return -ENOMEM;
    }

  nbytes = MIN(buflen, epinfo->maxpacket);

  nxsem_init(&iocsem, 0, 0);

  datareq->addr = (epinfo->dirin << 7) + epinfo->epno;
  datareq->xfrtype = epinfo->xfrtype;
  datareq->data = buffer;
  datareq->len = nbytes;
  datareq->priv = &iocsem;

  /* We must have exclusive access to and data structures. */

  nxmutex_lock(&priv->lock);

  nbytes = host_usbhost_eptrans(datareq);

  nxmutex_unlock(&priv->lock);

  /* Wait for transfer completion */

  nxsem_wait(&iocsem);

  nbytes = datareq->xfer;

  sim_usbhost_freerq(datareq);

  return (ssize_t)nbytes;
}

/****************************************************************************
 * Name: sim_usbhost_cancel
 ****************************************************************************/

static int sim_usbhost_cancel(FAR struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep)
{
  return 0;
}

/****************************************************************************
 * Name: sim_usbhost_disconnect
 ****************************************************************************/

static void sim_usbhost_disconnect(FAR struct usbhost_driver_s *drvr,
                                   FAR struct usbhost_hubport_s *hport)
{
  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;
}

/****************************************************************************
 * Name: sim_usbhost_rqcomplete
 ****************************************************************************/

static void sim_usbhost_rqcomplete(FAR struct sim_usbhost_s *drvr)
{
  struct host_usb_datareq_s *datareq;

  while ((datareq = host_usbhost_getcomplete()) != NULL)
    {
      sem_t *iocsem = datareq->priv;
      nxsem_post(iocsem);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_usbhost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 ****************************************************************************/

int sim_usbhost_initialize(void)
{
  struct sim_usbhost_s *priv = &g_sim_usbhost;
  struct usbhost_hubport_s *hport;
  int ret;

#ifdef CONFIG_USBHOST_CDCACM
  ret = usbhost_cdcacm_initialize();
#endif

  /* Initialize the device operations */

  priv->drvr.ep0configure   = sim_usbhost_ep0configure;
  priv->drvr.epalloc        = sim_usbhost_epalloc;
  priv->drvr.epfree         = sim_usbhost_epfree;
  priv->drvr.alloc          = sim_usbhost_alloc;
  priv->drvr.free           = sim_usbhost_free;
  priv->drvr.ioalloc        = sim_usbhost_ioalloc;
  priv->drvr.iofree         = sim_usbhost_iofree;
  priv->drvr.ctrlin         = sim_usbhost_ctrlin;
  priv->drvr.ctrlout        = sim_usbhost_ctrlout;
  priv->drvr.transfer       = sim_usbhost_transfer;
  priv->drvr.cancel         = sim_usbhost_cancel;
  priv->drvr.disconnect     = sim_usbhost_disconnect;

  /* Initialize the public port representation */

  hport                       = &priv->hport.hport;
  hport->drvr                 = &priv->drvr;
  hport->ep0                  = &priv->ep0;
  hport->port                 = 0;
  hport->speed                = USB_SPEED_HIGH;

  /* Initialize function address generation logic */

  usbhost_devaddr_initialize(&priv->hport);

  /* Initialize host usb controller */

  host_usbhost_init();

  /* Initialize the driver state data */

  priv->state = USB_HOST_DETACHED;

  ret = kthread_create("usbhost monitor", CONFIG_SIM_USB_PRIO,
                       CONFIG_SIM_USB_STACKSIZE,
                       sim_usbhost_waittask, NULL);
  if (ret < 0)
    {
      uerr("ERROR: Failed to create sim_usbhost_waittask: %d\n", ret);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: sim_usbhost_loop
 *
 * Description:
 *   USB host loop process.
 *
 ****************************************************************************/

int sim_usbhost_loop(void)
{
  struct sim_usbhost_s *priv = &g_sim_usbhost;
  struct usbhost_hubport_s *hport;
  bool connect;

  /* Handle root hub status change on each root port */

  connect = host_usbhost_getconnstate();

  /* Check current connect status */

  if (connect)
    {
      /* Connected ... Did we just become connected? */

      if (!priv->connected)
        {
          host_usbhost_open();

          /* Yes.. connected. */

          priv->connected = true;

          /* Notify any waiters */

          nxsem_post(&priv->pscsem);
          priv->pscwait = false;
        }

      sim_usbhost_rqcomplete(priv);
    }
  else
    {
      /* Disconnected... Did we just become disconnected? */

      if (priv->connected)
        {
          sim_usbhost_rqcomplete(priv);

          host_usbhost_close();

          /* Yes.. disconnect the device */

          priv->connected = false;

          /* Are we bound to a class instance? */

          hport = &priv->hport.hport;
          if (hport->devclass)
            {
              /* Yes.. Disconnect the class */

              CLASS_DISCONNECTED(hport->devclass);
              hport->devclass = NULL;
            }

          /* Notify any waiters for the Root Hub Status change
           * event.
           */

          nxsem_post(&priv->pscsem);
          priv->pscwait = false;
        }
    }

  return OK;
}
