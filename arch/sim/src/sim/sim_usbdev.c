/****************************************************************************
 * arch/sim/src/sim/sim_usbdev.c
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
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include "sim_internal.h"
#include "sim_usbdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USB trace ****************************************************************/

/* Trace error codes */

#define SIM_TRACEERR_ALLOCFAIL            0x0001
#define SIM_TRACEERR_BADCLEARFEATURE      0x0002
#define SIM_TRACEERR_BADDEVGETSTATUS      0x0003
#define SIM_TRACEERR_BADEPGETSTATUS       0x0004
#define SIM_TRACEERR_BADEPNO              0x0005
#define SIM_TRACEERR_BADEPTYPE            0x0006
#define SIM_TRACEERR_BADGETCONFIG         0x0007
#define SIM_TRACEERR_BADGETSETDESC        0x0008
#define SIM_TRACEERR_BADGETSTATUS         0x0009
#define SIM_TRACEERR_BADSETADDRESS        0x000a
#define SIM_TRACEERR_BADSETCONFIG         0x000b
#define SIM_TRACEERR_BADSETFEATURE        0x000c
#define SIM_TRACEERR_BINDFAILED           0x000d
#define SIM_TRACEERR_DISPATCHSTALL        0x000e
#define SIM_TRACEERR_DRIVER               0x000f
#define SIM_TRACEERR_DRIVERREGISTERED     0x0010
#define SIM_TRACEERR_EP0BADCTR            0x0011
#define SIM_TRACEERR_EP0SETUPSTALLED      0x0012
#define SIM_TRACEERR_EPBUFFER             0x0013
#define SIM_TRACEERR_EPDISABLED           0x0014
#define SIM_TRACEERR_EPOUTNULLPACKET      0x0015
#define SIM_TRACEERR_EPRESERVE            0x0016
#define SIM_TRACEERR_INVALIDCTRLREQ       0x0017
#define SIM_TRACEERR_INVALIDPARMS         0x0018
#define SIM_TRACEERR_IRQREGISTRATION      0x0019
#define SIM_TRACEERR_NOTCONFIGURED        0x001a
#define SIM_TRACEERR_REQABORTED           0x001b

#define SIM_USB_EPNUM                     (15)
#define SIM_USB_EP0MAXSIZE                64
#define SIM_USB_MAXPACKETSIZE(ep)         (((uint16_t)(ep) < 1) ? 64 : 512)
#define SIM_USB_EPMAXSIZE                 512

#define SIM_EPSET_ALL                     ((1 << SIM_USB_EPNUM) - 1)   /* All endpoints */
#define SIM_EPSET_NOEP0                   (SIM_EPSET_ALL - 1)          /* All endpoints except EP0 */
#define SIM_EP_BIT(ep)                    (1 << (ep))

#define sim_rqempty(q)                    ((q)->head == NULL)
#define sim_rqpeek(q)                     ((q)->head)

#ifdef CONFIG_USBDEV_DUALSPEED
#  define SIM_USB_SPEED                   USB_SPEED_HIGH
#else
#  define SIM_USB_SPEED                   USB_SPEED_FULL
#endif

#ifndef MIN
#  define MIN(a,b)                        ((a) > (b) ? (b) : (a))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* State of an endpoint */

enum sim_epstate_e
{
  SIM_EPSTATE_DISABLED = 0,   /* Endpoint is disabled */
  SIM_EPSTATE_STALLED,        /* Endpoint is stalled */
  SIM_EPSTATE_IDLE,           /* Endpoint is idle (i.e. ready for transmission) */
  SIM_EPSTATE_SENDING,        /* Endpoint is sending data */
  SIM_EPSTATE_RECEIVING,      /* Endpoint is receiving data */
  SIM_EPSTATE_EP0DATAOUT,     /* Endpoint 0 is receiving SETUP OUT data */
  SIM_EPSTATE_EP0STATUSIN,    /* Endpoint 0 is sending SETUP status */
  SIM_EPSTATE_EP0ADDRESS      /* Address change is pending completion of status */
};

/* The overall state of the device */

enum sim_devstate_e
{
  SIM_DEVSTATE_SUSPENDED = 0,  /* The device is currently suspended */
  SIM_DEVSTATE_POWERED,        /* Host is providing +5V through the USB cable */
  SIM_DEVSTATE_DEFAULT,        /* Device has been reset */
  SIM_DEVSTATE_ADDRESSED,      /* The device has been given an address on the bus */
  SIM_DEVSTATE_CONFIGURED      /* A valid configuration has been selected. */
};

/* The head of a queue of requests */

struct sim_rqhead_s
{
  struct sim_req_s   *head;           /* Requests are added to the head of the list */
  struct sim_req_s   *tail;           /* Requests are removed from the tail of the list */
};

struct sim_usbdev_s;
struct sim_ep_s
{
  struct usbdev_ep_s    ep;           /* Standard endpoint structure */
  struct sim_usbdev_s  *dev;          /* Reference to private driver data */
  volatile uint8_t      epstate;      /* State of the endpoint (see enum sim_epstate_e) */
  struct sim_rqhead_s   reqq;         /* Read/write request queue */
};

struct sim_usbdev_s
{
  struct usbdev_s               usbdev;
  struct usbdevclass_driver_s  *driver;               /* The bound device class driver */
  struct usb_ctrlreq_s          ctrl;                 /* Last EP0 request */
  uint8_t                       devstate;             /* State of the device (see enum sim_devstate_e) */
  uint8_t                       prevstate;            /* Previous state of the device before SUSPEND */
  uint8_t                       selfpowered:1;        /* 1: Device is self powered */
  uint16_t                      epavail;              /* Bitset of available endpoints */
  struct sim_ep_s               eps[SIM_USB_EPNUM];
};

struct sim_req_s
{
  struct usbdev_req_s    req;           /* Standard USB request */
  struct sim_req_s      *flink;         /* Supports a singly linked list */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Endpoint operations ******************************************************/

static int sim_ep_configure(struct usbdev_ep_s *ep,
                            const struct usb_epdesc_s *desc,
                            bool last);
static int sim_ep_disable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *sim_ep_allocreq(struct usbdev_ep_s *ep);
static void sim_ep_freereq(struct usbdev_ep_s *ep,
                          struct usbdev_req_s *);
static int sim_ep_submit(struct usbdev_ep_s *ep,
                        struct usbdev_req_s *req);
static int sim_ep_cancel(struct usbdev_ep_s *ep,
                        struct usbdev_req_s *req);
static int sim_ep_stall(struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations *****************************************/

static struct usbdev_ep_s *sim_usbdev_allocep(struct usbdev_s *dev,
                                           uint8_t epno, bool in,
                                           uint8_t eptype);
static void sim_usbdev_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int sim_usbdev_getframe(struct usbdev_s *dev);
static int sim_usbdev_wakeup(struct usbdev_s *dev);
static int sim_usbdev_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int sim_usbdev_pullup(struct usbdev_s *dev, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sim_usbdev_s g_sim_usbdev;

static const struct usbdev_epops_s g_epops =
{
  .configure   = sim_ep_configure,
  .disable     = sim_ep_disable,
  .allocreq    = sim_ep_allocreq,
  .freereq     = sim_ep_freereq,
  .submit      = sim_ep_submit,
  .stall       = sim_ep_stall,
  .cancel      = sim_ep_cancel,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = sim_usbdev_allocep,
  .freeep      = sim_usbdev_freeep,
  .selfpowered = sim_usbdev_selfpowered,
  .pullup      = sim_usbdev_pullup,
  .getframe    = sim_usbdev_getframe,
  .wakeup      = sim_usbdev_wakeup,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_rqdequeue
 ****************************************************************************/

static struct sim_req_s *sim_rqdequeue(struct sim_rqhead_s *queue)
{
  struct sim_req_s *ret = queue->head;

  if (ret)
    {
      queue->head = ret->flink;
      if (!queue->head)
        {
          queue->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: sim_rqenqueue
 ****************************************************************************/

static void sim_rqenqueue(struct sim_rqhead_s *queue,
                           struct sim_req_s *req)
{
  req->flink = NULL;
  if (!queue->head)
    {
      queue->head = req;
      queue->tail = req;
    }
  else
    {
      queue->tail->flink = req;
      queue->tail        = req;
    }
}

/****************************************************************************
 * Name: sim_reqabort
 ****************************************************************************/

static void sim_reqabort(struct sim_ep_s *privep,
                         struct sim_req_s *privreq,
                         int16_t result)
{
  usbtrace(TRACE_DEVERROR(SIM_TRACEERR_REQABORTED),
           (uint16_t)USB_EPNO(privep->ep.eplog));

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/****************************************************************************
 * Name: sim_reqcomplete
 ****************************************************************************/

static void sim_reqcomplete(struct sim_ep_s *privep, int16_t result)
{
  struct sim_req_s *privreq;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags = enter_critical_section();
  privreq = sim_rqdequeue(&privep->reqq);
  leave_critical_section(flags);

  if (privreq)
    {
      /* Save the result in the request structure */

      privreq->req.result = result;

      privep->epstate = SIM_EPSTATE_IDLE;

      /* Callback to the request completion handler */

      privreq->req.callback(&privep->ep, &privreq->req);
    }
}

/****************************************************************************
 * Name: sim_reqwrite
 ****************************************************************************/

static int sim_reqwrite(struct sim_usbdev_s *priv, struct sim_ep_s *privep)
{
  struct sim_req_s *privreq;
  uint8_t *write_data;
  int write_len;
  uint8_t epno;

  /* Get the unadorned endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* We get here when an IN endpoint interrupt occurs.  So now we know that
   * there is no TX transfer in progress.
   */

  while (privep->epstate == SIM_EPSTATE_IDLE)
    {
      /* Check the request from the head of the endpoint request queue */

      privreq = sim_rqpeek(&privep->reqq);
      if (!privreq)
        {
          return OK;
        }

      while (privreq->req.xfrd < privreq->req.len)
        {
          /* Handle any bytes in flight. */

          write_data = privreq->req.buf + privreq->req.xfrd;
          write_len = privreq->req.len - privreq->req.xfrd;

          write_len = host_usbdev_epwrite(epno, privreq->req.flags,
                                          write_data, write_len);

          if (write_len < 0)
            {
              return -EPERM;
            }

          privreq->req.xfrd += write_len;
        }

      /* If all of the bytes were sent (including any final zero length
       * packet) then we are finished with the request buffer and we can
       * return the request buffer to the class driver.  The state will
       * remain IDLE only if nothing else was put in flight.
       *
       * Note that we will then loop to check to check the next queued
       * write request.
       */

      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               privreq->req.xfrd);
      sim_reqcomplete(privep, OK);
    }

  return OK;
}

/****************************************************************************
 * Name: sim_ep_configure
 ****************************************************************************/

static void sim_usbdev_getctrlreq(struct usb_ctrlreq_s *usb_req,
                                  const struct host_usb_ctrlreq_s *host_req)
{
  usb_req->type = host_req->type;
  usb_req->req = host_req->req;
  usb_req->value[0] = LSBYTE(host_req->value);
  usb_req->value[1] = MSBYTE(host_req->value);
  usb_req->index[0] = LSBYTE(host_req->index);
  usb_req->index[1] = MSBYTE(host_req->index);
  usb_req->len[0] = LSBYTE(host_req->len);
  usb_req->len[1] = MSBYTE(host_req->len);
}

/****************************************************************************
 * Name: sim_ep_configure
 ****************************************************************************/

static void sim_usbdev_setepdesc(struct host_usb_epdesc_s *host_epdesc,
                                 const struct usb_epdesc_s *usb_epdesc)
{
  host_epdesc->len = usb_epdesc->len;
  host_epdesc->type = usb_epdesc->type;
  host_epdesc->addr = usb_epdesc->addr;
  host_epdesc->attr = usb_epdesc->attr;
  host_epdesc->mxpacketsize = GETUINT16(usb_epdesc->mxpacketsize);
  host_epdesc->interval = usb_epdesc->interval;
}

/****************************************************************************
 *
 * Endpoint operations
 *
 ****************************************************************************/

/****************************************************************************
 * Name: sim_usbdev_ep0read
 *
 * Description:
 *   USB Dev receive ep0 control request.
 *
 ****************************************************************************/

static int sim_usbdev_ep0read(struct host_usb_ctrlreq_s *req)
{
  struct sim_usbdev_s *priv = &g_sim_usbdev;
  uint8_t *outdata = NULL;
  size_t outlen = 0;
  int ret = -EINVAL;

  if (priv && priv->driver)
    {
      /* Save ctrl req */

      sim_usbdev_getctrlreq(&priv->ctrl, req);

      /* Was this an OUT SETUP command? */

      if (USB_REQ_ISOUT(priv->ctrl.type))
        {
          outlen = GETUINT16(priv->ctrl.len);
          outdata = req->data;
        }

      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl,
                        outdata, outlen);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(SIM_TRACEERR_DISPATCHSTALL), 0);
          sim_ep_stall(&priv->eps[0].ep, false);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: sim_usbdev_epread
 *
 * Description:
 *   USB Dev receive ep data request.
 *
 ****************************************************************************/

static int sim_usbdev_epread(uint16_t addr, uint8_t *data, uint16_t len)
{
  struct sim_usbdev_s *priv = &g_sim_usbdev;
  struct sim_ep_s *privep;
  struct sim_req_s *privreq;
  uint8_t *dest;
  uint8_t epno;
  int readlen;

  /* Get the unadorned endpoint number */

  epno = USB_EPNO(addr);
  privep = &priv->eps[epno];

  usbtrace(TRACE_READ(USB_EPNO(privep->ep.eplog)), len);

  /* We get here when an IN endpoint interrupt occurs.  So now we know that
   * there is no TX transfer in progress.
   */

  while (privep->epstate == SIM_EPSTATE_IDLE && len > 0)
    {
      /* Check the request from the head of the endpoint request queue */

      privreq = sim_rqpeek(&privep->reqq);
      if (!privreq)
        {
          return -ENOENT;
        }

      /* Get the source and destination transfer addresses */

      dest = privreq->req.buf + privreq->req.xfrd;

      /* Get the number of bytes to read from packet memory */

      readlen = MIN(privreq->req.len - privreq->req.xfrd, len);

      /* Receive the next packet */

      memcpy(dest, data, readlen);

      /* If the receive buffer is full or this is a partial packet,
       * then we are finished with the request buffer).
       */

      privreq->req.xfrd += readlen;
      len -= readlen;
      data += readlen;

      if (len < privep->ep.maxpacket ||
          privreq->req.xfrd >= privreq->req.len)
        {
          /* Return the read request to the class driver. */

          usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
          sim_reqcomplete(privep, OK);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sim_ep_configure
 ****************************************************************************/

static int sim_ep_configure(struct usbdev_ep_s *ep,
                            const struct usb_epdesc_s *desc,
                            bool last)
{
  struct sim_ep_s *privep = (struct sim_ep_s *)ep;
  struct host_usb_epdesc_s host_epdesc;
  uint8_t  epno;

  /* Get the unadorned endpoint address */

  epno = USB_EPNO(desc->addr);
  usbtrace(TRACE_EPCONFIGURE, (uint16_t)epno);
  DEBUGASSERT(epno == USB_EPNO(ep->eplog));

  /* Get the maxpacket size of the endpoint. */

  ep->maxpacket = GETUINT16(desc->mxpacketsize);
  DEBUGASSERT(ep->maxpacket <= SIM_USB_EPMAXSIZE);

  /* Get the subset matching the requested direction */

  if (USB_ISEPIN(desc->addr))
    {
      ep->eplog = USB_EPIN(epno);
    }
  else
    {
      ep->eplog = USB_EPOUT(epno);
    }

  sim_usbdev_setepdesc(&host_epdesc, desc);
  host_usbdev_epconfig(epno, &host_epdesc);

  privep->epstate = SIM_EPSTATE_IDLE;

  return OK;
}

/****************************************************************************
 * Name: sim_ep_disable
 ****************************************************************************/

static int sim_ep_disable(struct usbdev_ep_s *ep)
{
  struct sim_ep_s *privep = (struct sim_ep_s *)ep;
  irqstate_t flags;
  uint8_t epno;

  epno = USB_EPNO(ep->eplog);
  usbtrace(TRACE_EPDISABLE, epno);

  /* Cancel any ongoing activity */

  flags = enter_critical_section();

  /* Disable TX; disable RX */

  host_usbdev_epdisable(epno);

  privep->epstate = SIM_EPSTATE_DISABLED;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sim_ep_allocreq
 ****************************************************************************/

static struct usbdev_req_s *sim_ep_allocreq(struct usbdev_ep_s *ep)
{
  struct sim_req_s *privreq;

  usbtrace(TRACE_EPALLOCREQ, USB_EPNO(ep->eplog));

  privreq = (struct sim_req_s *)kmm_malloc(sizeof(struct sim_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(SIM_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct sim_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: sim_ep_freereq
 ****************************************************************************/

static void sim_ep_freereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct sim_req_s *privreq = (struct sim_req_s *)req;

  usbtrace(TRACE_EPFREEREQ, USB_EPNO(ep->eplog));

  kmm_free(privreq);
}

/****************************************************************************
 * Name: sim_ep_stall
 ****************************************************************************/

static int sim_ep_stall(struct usbdev_ep_s *ep, bool resume)
{
  struct sim_ep_s *privep = (struct sim_ep_s *)ep;
  uint8_t epno;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(ep != NULL);

  epno = USB_EPNO(ep->eplog);

  /* STALL or RESUME the endpoint */

  flags = enter_critical_section();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, epno);

  ret = host_usbdev_epstall(epno, resume);

  privep->epstate = SIM_EPSTATE_STALLED;

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: sim_ep_submit
 ****************************************************************************/

static int sim_ep_submit(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct sim_req_s *privreq = (struct sim_req_s *)req;
  struct sim_ep_s *privep = (struct sim_ep_s *)ep;
  struct sim_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;
  int ret = OK;

  DEBUGASSERT(ep != NULL && req != NULL && req->callback !=
              NULL && req->buf != NULL);
  usbtrace(TRACE_EPSUBMIT, USB_EPNO(ep->eplog));
  priv = privep->dev;
  DEBUGASSERT(priv->driver != NULL);

  /* Handle the request from the class driver */

  epno              = USB_EPNO(ep->eplog);
  req->result       = -EINPROGRESS;
  req->xfrd         = 0;
  flags             = enter_critical_section();

  if (privep->epstate == SIM_EPSTATE_STALLED)
    {
      sim_reqabort(privep, privreq, -EBUSY);
      return -EPERM;
    }

  /* Handle IN (device-to-host) requests.  NOTE:  If the class device is
   * using the bi-directional EP0, then we assume that they intend the EP0
   * IN functionality (EP0 SETUP OUT data receipt does not use requests).
   */

  if (USB_ISEPIN(ep->eplog) || epno == 0)
    {
      usbtrace(TRACE_INREQQUEUED(epno), req->len);

      /* Add the new request to the request queue for the endpoint */

      sim_rqenqueue(&privep->reqq, privreq);

      /* If the IN endpoint is IDLE, then transfer the data now */

      if (privep->epstate == SIM_EPSTATE_IDLE)
        {
          ret = sim_reqwrite(priv, privep);
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      usbtrace(TRACE_OUTREQQUEUED(epno), req->len);
      sim_rqenqueue(&privep->reqq, privreq);
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: sim_ep_cancel
 ****************************************************************************/

static int sim_ep_cancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  irqstate_t flags;

  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));

  flags = enter_critical_section();
  host_usbdev_epcancel(USB_EPNO(ep->eplog));
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Device Controller Operations
 ****************************************************************************/

/****************************************************************************
 *
 * Name: sim_ep_reserve
 *
 * Description:
 *   Find an un-reserved endpoint number and reserve it for the caller.
 *
 ****************************************************************************/

static struct sim_ep_s *sim_ep_reserve(struct sim_usbdev_s *priv,
                                       uint16_t epset)
{
  struct sim_ep_s *privep = NULL;
  irqstate_t flags;
  int epndx = 0;

  flags = enter_critical_section();
  epset &= priv->epavail;
  if (epset)
    {
      /* Select the lowest bit in the set of matching, available endpoints
       * (skipping EP0)
       */

      for (epndx = 1; epndx < SIM_USB_EPNUM; epndx++)
        {
          uint16_t bit = SIM_EP_BIT(epndx);
          if ((epset & bit) != 0)
            {
              /* Mark the endpoint no longer available */

              priv->epavail &= ~bit;

              /* And return the pointer to the standard endpoint structure */

              privep = &priv->eps[epndx];
              break;
            }
        }
    }

  leave_critical_section(flags);
  return privep;
}

/****************************************************************************
 *
 * Name: sim_ep_unreserve
 *
 * Description:
 *   The endpoint is no long in-used.  It will be un-reserved and can be
 *   re-used if needed.
 *
 ****************************************************************************/

static void sim_ep_unreserve(struct sim_usbdev_s *priv,
                             struct sim_ep_s *privep)
{
  irqstate_t flags = enter_critical_section();
  priv->epavail |= SIM_EP_BIT(USB_EPNO(privep->ep.eplog));
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sim_usbdev_allocep
 ****************************************************************************/

static struct usbdev_ep_s *sim_usbdev_allocep(struct usbdev_s *dev,
                                              uint8_t epno, bool in,
                                              uint8_t eptype)
{
  struct sim_usbdev_s *priv = (struct sim_usbdev_s *)dev;
  struct sim_ep_s *privep = NULL;
  uint16_t epset = SIM_EPSET_NOEP0;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)epno);
  DEBUGASSERT(dev != NULL);

  /* Ignore any direction bits in the logical address */

  epno = USB_EPNO(epno);

  /* A logical address of 0 means that any endpoint will do */

  if (epno > 0)
    {
      /* Otherwise, we will return the endpoint
       * structure only for the requested 'logical' endpoint.
       * All of the other checks will still be performed.
       *
       * First, verify that the logical endpoint is in the
       * range supported by the hardware.
       */

      if (epno >= SIM_USB_EPNUM)
        {
          usbtrace(TRACE_DEVERROR(SIM_TRACEERR_BADEPNO), (uint16_t)epno);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset = SIM_EP_BIT(epno);
    }

  /* Check if the selected endpoint number is available */

  privep = sim_ep_reserve(priv, epset);
  if (!privep)
    {
      usbtrace(TRACE_DEVERROR(SIM_TRACEERR_EPRESERVE), (uint16_t)epset);
      return NULL;
    }

  return &privep->ep;
}

/****************************************************************************
 * Name: sim_usbdev_freeep
 ****************************************************************************/

static void sim_usbdev_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct sim_usbdev_s *priv;
  struct sim_ep_s *privep;

  DEBUGASSERT(dev != NULL && ep != NULL);

  priv   = (struct sim_usbdev_s *)dev;
  privep = (struct sim_ep_s *)ep;
  usbtrace(TRACE_DEVFREEEP, (uint16_t)USB_EPNO(ep->eplog));

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      sim_ep_unreserve(priv, privep);
    }
}

/****************************************************************************
 *
 * Name: sim_usbdev_getframe
 *
 * Description:
 *   This is the getframe() method of the USB device driver interface
 *
 ****************************************************************************/

static int sim_usbdev_getframe(struct usbdev_s *dev)
{
  return 0;
}

/****************************************************************************
 *
 * Name: sim_usbdev_wakeup
 *
 * Description:
 *   This is the wakeup() method of the USB device driver interface
 *
 ****************************************************************************/

static int sim_usbdev_wakeup(struct usbdev_s *dev)
{
  return OK;
}

/****************************************************************************
 *
 * Name: sim_usbdev_selfpowered
 *
 * Description:
 *   This is the selfpowered() method of the USB device driver interface
 *
 ****************************************************************************/

static int sim_usbdev_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct sim_usbdev_s *priv = (struct sim_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);
  DEBUGASSERT(dev != NULL);

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 *
 * Name: sim_usbdev_pullup
 *
 * Description:
 *   This is the pullup() method of the USB device driver interface
 *
 ****************************************************************************/

static int sim_usbdev_pullup(struct usbdev_s *dev, bool enable)
{
  irqstate_t flags;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  flags = enter_critical_section();
  host_usbdev_pullup(enable);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sim_usbdevinit
 *
 * Description:
 *   Initialize the USB dev
 *
 ****************************************************************************/

static void sim_usbdev_devinit(struct sim_usbdev_s *dev)
{
  uint8_t epno;

  memset(dev, 0, sizeof(struct sim_usbdev_s));

  dev->usbdev.ep0 = &dev->eps[0].ep;
  dev->usbdev.ops = &g_devops;
  dev->usbdev.speed = SIM_USB_SPEED;

  for (epno = 0; epno < SIM_USB_EPNUM; epno++)
    {
      dev->eps[epno].dev       = dev;
      dev->eps[epno].ep.ops    = &g_epops;
      dev->eps[epno].ep.eplog  = epno;
      dev->eps[epno].ep.maxpacket = SIM_USB_MAXPACKETSIZE(epno);
    }

  dev->eps[0].epstate = SIM_EPSTATE_IDLE;

  dev->epavail = SIM_EPSET_NOEP0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_usbdev_initialize
 *
 * Description:
 *   Initialize the USB driver
 *
 ****************************************************************************/

void sim_usbdev_initialize(void)
{
}

/****************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method
 *   will be called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  struct sim_usbdev_s *priv = &g_sim_usbdev;
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

  /* First hook up the driver */

  sim_usbdev_devinit(priv);
  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(SIM_TRACEERR_BINDFAILED), (uint16_t) - ret);
    }
  else
    {
      /* Setup the USB host controller */

#ifdef CONFIG_USBDEV_DUALSPEED
      host_usbdev_init(SIM_USB_SPEED);
#else
      host_usbdev_init(USB_SPEED_FULL);
#endif
    }

  return ret;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected to a USB
 *   host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *   returns.
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  /* At present, there is only a single OTG FS device support. Hence it is
   * pre-allocated as g_otgfsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple
   * devices.
   */

  struct sim_usbdev_s *priv = &g_sim_usbdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = enter_critical_section();
  host_usbdev_deinit();
  leave_critical_section(flags);

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts */

  flags = enter_critical_section();

  /* Disconnect device */

  host_usbdev_pullup(false);

  /* Unhook the driver */

  priv->driver = NULL;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: sim_usbdev_loop
 *
 * Description:
 *   USB Dev receive ep0 control request.
 *
 ****************************************************************************/

int sim_usbdev_loop(void)
{
  struct sim_usbdev_s *priv = &g_sim_usbdev;
  struct sim_ep_s *privep;
  struct host_usb_ctrlreq_s *ctrlreq;
  uint8_t *recv_data;
  uint16_t data_len;
  uint8_t epcnt;

  /* Loop ep0 */

  ctrlreq = host_usbdev_ep0read();
  if (ctrlreq)
    {
      sim_usbdev_ep0read(ctrlreq);
      host_usbdev_epread_end(0);
    }

  /* Loop other eps */

  for (epcnt = 1; epcnt < SIM_USB_EPNUM; epcnt++)
    {
      privep = &priv->eps[epcnt];
      if (privep->epstate == SIM_EPSTATE_IDLE &&
          !USB_ISEPIN(privep->ep.eplog))
        {
          recv_data = host_usbdev_epread(epcnt, &data_len);
          if (recv_data)
            {
              sim_usbdev_epread(privep->ep.eplog, recv_data, data_len);
              host_usbdev_epread_end(epcnt);
            }
        }
    }

  return OK;
}
