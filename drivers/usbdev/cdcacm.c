/****************************************************************************
 * drivers/usbdev/cdcacm.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbdev_trace.h>

#include "cdcacm.h"

#ifdef CONFIG_CDCACM_COMPOSITE
#  include <nuttx/usb/composite.h>
#  include "composite.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RX poll delay = 200 milliseconds. CLK_TCK is the number of clock ticks per
 * second
 */

#define CDCACM_RXDELAY   (CLK_TCK / 5)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container to support a list of requests */

struct cdcacm_wrreq_s
{
  FAR struct cdcacm_wrreq_s *flink;    /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;        /* The contained request */
};

struct cdcacm_rdreq_s
{
  FAR struct cdcacm_rdreq_s *flink;    /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;        /* The contained request */
  uint16_t offset;                     /* Offset to valid data in the RX request */
};

/* This structure describes the internal state of the driver */

struct cdcacm_dev_s
{
  FAR struct uart_dev_s serdev;        /* Serial device structure */
  FAR struct usbdev_s *usbdev;         /* usbdev driver pointer */

  uint8_t config;                      /* Configuration number */
  uint8_t nwrq;                        /* Number of queue write requests (in txfree) */
  uint8_t nrdq;                        /* Number of queue read requests (in epbulkout) */
  uint8_t minor;                       /* The device minor number */
  uint8_t ctrlline;                    /* Buffered control line state */
#ifdef CONFIG_CDCACM_IFLOWCONTROL
  uint8_t serialstate;                 /* State of the DSR/DCD */
  bool iflow;                          /* True: input flow control is enabled */
  bool iactive;                        /* True: input flow control is active */
  bool upper;                          /* True: RX buffer is (nearly) full */
#endif
  bool rxenabled;                      /* true: UART RX "interrupts" enabled */
  bool ispolling;

  struct cdc_linecoding_s linecoding;  /* Buffered line status */
  cdcacm_callback_t callback;          /* Serial event callback function */

  FAR struct usbdev_ep_s *epintin;     /* Interrupt IN endpoint structure */
  FAR struct usbdev_ep_s *epbulkin;    /* Bulk IN endpoint structure */
  FAR struct usbdev_ep_s *epbulkout;   /* Bulk OUT endpoint structure */
  FAR struct usbdev_req_s *ctrlreq;    /* Allocated control request */
  struct wdog_s rxfailsafe;            /* Failsafe timer to prevent RX stalls */
  struct sq_queue_s txfree;            /* Available write request containers */
  struct sq_queue_s rxpending;         /* Pending read request containers */

  struct usbdev_devinfo_s devinfo;

  /* Pre-allocated write request containers.  The write requests will
   * be linked in a free list (txfree), and used to send requests to
   * EPBULKIN; Read requests will be queued in the EBULKOUT.
   */

  struct cdcacm_wrreq_s wrreqs[CONFIG_CDCACM_NWRREQS];
  struct cdcacm_rdreq_s rdreqs[CONFIG_CDCACM_NRDREQS];

  /* Serial I/O buffers */

  char rxbuffer[CONFIG_CDCACM_RXBUFSIZE];
  char txbuffer[CONFIG_CDCACM_TXBUFSIZE];
};

/* The internal version of the class driver */

struct cdcacm_driver_s
{
  struct usbdevclass_driver_s drvr;
  FAR struct cdcacm_dev_s     *dev;
};

/* This is what is allocated */

struct cdcacm_alloc_s
{
  struct cdcacm_dev_s    dev;
  struct cdcacm_driver_s drvr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Transfer helpers *********************************************************/

static int     cdcacm_sndpacket(FAR struct cdcacm_dev_s *priv);
static int     cdcacm_requeue_rdrequest(FAR struct cdcacm_dev_s *priv,
                 FAR struct cdcacm_rdreq_s *rdcontainer);
static int     cdcacm_release_rxpending(FAR struct cdcacm_dev_s *priv);
static void    cdcacm_rxtimeout(wdparm_t arg);

/* Flow Control *************************************************************/

#ifdef CONFIG_CDCACM_IFLOWCONTROL
static int     cdcacm_serialstate(FAR struct cdcacm_dev_s *priv);
#endif

/* Configuration ************************************************************/

static void    cdcacm_resetconfig(FAR struct cdcacm_dev_s *priv);
static int     cdcacm_epconfigure(FAR struct usbdev_ep_s *ep,
                 enum cdcacm_epdesc_e epid, bool last,
                 FAR struct usbdev_devinfo_s *devinfo,
                 uint8_t speed);
static int     cdcacm_setconfig(FAR struct cdcacm_dev_s *priv,
                 uint8_t config);

/* Completion event handlers ************************************************/

static void    cdcacm_ep0incomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static void    cdcacm_rdcomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static void    cdcacm_wrcomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);

/* USB class device *********************************************************/

static int     cdcacm_bind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    cdcacm_unbind(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static int     cdcacm_setup(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev,
                 FAR const struct usb_ctrlreq_s *ctrl, FAR uint8_t *dataout,
                 size_t outlen);
static void    cdcacm_disconnect(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
#ifdef CONFIG_SERIAL_REMOVABLE
static void    cdcacm_suspend(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
static void    cdcacm_resume(FAR struct usbdevclass_driver_s *driver,
                 FAR struct usbdev_s *dev);
#endif

/* UART Operations **********************************************************/

static int     cdcuart_setup(FAR struct uart_dev_s *dev);
static void    cdcuart_shutdown(FAR struct uart_dev_s *dev);
static int     cdcuart_attach(FAR struct uart_dev_s *dev);
static void    cdcuart_detach(FAR struct uart_dev_s *dev);
static int     cdcuart_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
static void    cdcuart_rxint(FAR struct uart_dev_s *dev, bool enable);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool    cdcuart_rxflowcontrol(FAR struct uart_dev_s *dev,
                 unsigned int nbuffered, bool upper);
#endif
static void    cdcuart_txint(FAR struct uart_dev_s *dev, bool enable);
static bool    cdcuart_txempty(FAR struct uart_dev_s *dev);
static int     cdcuart_release(FAR struct uart_dev_s *dev);
static bool    cdcuart_rxavailable(FAR struct uart_dev_s *dev);
static ssize_t cdcuart_recvbuf(FAR struct uart_dev_s *dev,
                               FAR void *buf, size_t len);
static bool    cdcuart_txready(FAR struct uart_dev_s *dev);
static ssize_t cdcuart_sendbuf(FAR struct uart_dev_s *dev,
                               FAR const void *buf, size_t len);
static void    cdcuart_dmasend(FAR struct uart_dev_s *dev);
static void    cdcuart_dmareceive(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB class device *********************************************************/

static const struct usbdevclass_driverops_s g_driverops =
{
  cdcacm_bind,           /* bind */
  cdcacm_unbind,         /* unbind */
  cdcacm_setup,          /* setup */
  cdcacm_disconnect,     /* disconnect */
#ifdef CONFIG_SERIAL_REMOVABLE
  cdcacm_suspend,        /* suspend */
  cdcacm_resume,         /* resume */
#else
  NULL,                  /* suspend */
  NULL,                  /* resume */
#endif
};

/* Serial port **************************************************************/

static const struct uart_ops_s g_uartops =
{
  cdcuart_setup,         /* setup */
  cdcuart_shutdown,      /* shutdown */
  cdcuart_attach,        /* attach */
  cdcuart_detach,        /* detach */
  cdcuart_ioctl,         /* ioctl */
  NULL,                  /* receive */
  cdcuart_rxint,         /* rxinit */
  cdcuart_rxavailable,   /* rxavailable */
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  cdcuart_rxflowcontrol, /* rxflowcontrol */
#endif
#ifdef CONFIG_SERIAL_TXDMA
  cdcuart_dmasend,       /* dmasend */
#endif
#ifdef CONFIG_SERIAL_RXDMA
  cdcuart_dmareceive,    /* dmareceive */
  NULL,                  /* dmarxfree */
#endif
#ifdef CONFIG_SERIAL_TXDMA
  NULL,                  /* dmatxavail */
#endif
  NULL,                  /* send */
  cdcuart_txint,         /* txint */
  cdcuart_txready,       /* txready */
  cdcuart_txempty,       /* txempty */
  cdcuart_release,       /* release */
  cdcuart_recvbuf,       /* recvbuf */
  cdcuart_sendbuf        /* sendbuf */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcuart_txready
 *
 * Description:
 *   Check if tx buf is ready or not.
 *
 ****************************************************************************/

static bool cdcuart_txready(FAR struct uart_dev_s *dev)
{
  FAR struct cdcacm_dev_s *priv = dev->priv;
  FAR struct usbdev_ep_s *ep = priv->epbulkin;

  if (sq_empty(&priv->txfree))
    {
      priv->ispolling = true;
      EP_POLL(ep);
      priv->ispolling = false;
    }

  return !sq_empty(&priv->txfree);
}

/****************************************************************************
 * Name: cdcuart_sendbuf
 *
 * Description:
 *   This function transfers the TX data into the request, and submits the
 *   requests to the USB controller.
 *
 ****************************************************************************/

static ssize_t cdcuart_sendbuf(FAR struct uart_dev_s *dev,
                               FAR const void *buf, size_t len)
{
  FAR struct cdcacm_dev_s *priv = dev->priv;
  FAR struct usbdev_ep_s *ep = priv->epbulkin;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  FAR struct usbdev_req_s *req;
  size_t reqlen;
  size_t nbytes;
  int ret;

  /* Get the maximum number of bytes that will fit into one bulk IN request */

  reqlen = MIN(CONFIG_CDCACM_BULKIN_REQLEN, ep->maxpacket);

  /* Peek at the request in the container at the head of the list */

  wrcontainer = (FAR struct cdcacm_wrreq_s *)sq_remfirst(&priv->txfree);
  req = wrcontainer->req;
  priv->nwrq--;

  /* Fill the request with serial TX data */

  nbytes = MIN(reqlen, len);
  memcpy(req->buf, buf, nbytes);

  /* Submit the request to the endpoint */

  req->len   = nbytes;
  req->priv  = wrcontainer;
  req->flags = USBDEV_REQFLAGS_NULLPKT;
  priv->ispolling = true;
  ret        = EP_SUBMIT(ep, req);
  priv->ispolling = false;
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL),
               (uint16_t)-ret);
      return ret;
    }

  return nbytes;
}

/****************************************************************************
 * Name: cdcacm_sndpacket
 *
 * Description:
 *   This function obtains write requests, transfers the TX data into the
 *   request, and submits the requests to the USB controller.  This
 *   continues until either (1) there are no further packets available, or
 *   (2) there is no further data to send.
 *
 ****************************************************************************/

static int cdcacm_sndpacket(FAR struct cdcacm_dev_s *priv)
{
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EINVAL;
    }
#endif

  flags = enter_critical_section();
  if (priv->ispolling)
    {
      goto out;
    }

  uinfo("head=%d tail=%d nwrq=%d empty=%d\n",
        priv->serdev.xmit.head, priv->serdev.xmit.tail,
        priv->nwrq, sq_empty(&priv->txfree));

  if (!sq_empty(&priv->txfree))
    {
      uart_xmitchars_dma(&priv->serdev);
    }

out:
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: cdcuart_rxavailable
 *
 * Description:
 *   Check if data has been saved in rx buf.
 *
 ****************************************************************************/

static bool cdcuart_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct cdcacm_dev_s *priv = dev->priv;
  FAR struct usbdev_ep_s *ep = priv->epbulkout;

  if (sq_empty(&priv->rxpending))
    {
      priv->ispolling = true;
      EP_POLL(ep);
      priv->ispolling = false;
    }

  return !sq_empty(&priv->rxpending);
}

/****************************************************************************
 * Name: cdcuart_recvbuf
 *
 * Description:
 *   This function handles the USB packet and provides the received data to
 *   the uart RX buffer.
 *
 ****************************************************************************/

static ssize_t cdcuart_recvbuf(FAR struct uart_dev_s *dev,
                               FAR void *buf, size_t len)
{
  FAR struct cdcacm_dev_s *priv = dev->priv;
  FAR struct cdcacm_rdreq_s *rdcontainer;
  FAR struct usbdev_req_s *req;
  FAR uint8_t *reqbuf;
  size_t reqlen;
  size_t nbytes;
  int ret;

  /* Process each packet in the priv->rxpending list */

  rdcontainer = (FAR struct cdcacm_rdreq_s *)sq_peek(&priv->rxpending);
  DEBUGASSERT(rdcontainer != NULL);

  req = rdcontainer->req;
  DEBUGASSERT(req != NULL);

  reqbuf = &req->buf[rdcontainer->offset];
  reqlen = req->xfrd - rdcontainer->offset;

  nbytes = MIN(reqlen, len);
  memcpy(buf, reqbuf, nbytes);
  rdcontainer->offset += nbytes;

  /* The entire packet was processed and may be removed from the
   * pending RX list.
   */

  if (rdcontainer->offset >= req->xfrd)
    {
      sq_remfirst(&priv->rxpending);
      ret = cdcacm_requeue_rdrequest(priv, rdcontainer);
      if (ret < 0)
        {
          return ret;
        }
    }

  return nbytes;
}

/****************************************************************************
 * Name: cdcacm_requeue_rdrequest
 *
 * Description:
 *   Add any pending RX packets to the upper half serial drivers RX buffer.
 *
 ****************************************************************************/

static int cdcacm_requeue_rdrequest(FAR struct cdcacm_dev_s *priv,
                                    FAR struct cdcacm_rdreq_s *rdcontainer)
{
  FAR struct usbdev_req_s *req;
  FAR struct usbdev_ep_s *ep;
  int ret;

  DEBUGASSERT(priv != NULL && rdcontainer != NULL);
  rdcontainer->offset = 0;

  req      = rdcontainer->req;
  DEBUGASSERT(req != NULL);

  /* Requeue the read request */

  ep       = priv->epbulkout;
  req->len = MIN(CONFIG_CDCACM_BULKOUT_REQLEN, ep->maxpacket);
  ret      = EP_SUBMIT(ep, req);
  if (ret != OK)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT),
                              (uint16_t)-req->result);
    }

  return ret;
}

/****************************************************************************
 * Name: cdcacm_release_rxpending
 *
 * Description:
 *   Add any pending RX packets to the upper half serial drivers RX buffer.
 *
 ****************************************************************************/

static int cdcacm_release_rxpending(FAR struct cdcacm_dev_s *priv)
{
  irqstate_t flags;
  int ret = -EBUSY;

  /* Note that the priv->rxpending queue, priv->rxenabled, priv->iactive
   * may be modified by interrupt level processing and, hence, interrupts
   * must be disabled throughout the following.
   */

  flags = enter_critical_section();

  if (priv->ispolling)
    {
      goto out;
    }

  /* Cancel any pending failsafe timer */

  wd_cancel(&priv->rxfailsafe);

  /* If RX "interrupts" are enabled and if input flow control is not in
   * effect, then pass the packet at the head of the pending RX packet list
   * to the upper serial layer.  Otherwise, let the packet continue to pend
   * the priv->rxpending list until the upper serial layer is able to buffer
   * it.
   */

#ifdef CONFIG_CDCACM_IFLOWCONTROL
  if (priv->rxenabled && !priv->iactive)
#else
  if (priv->rxenabled)
#endif
    {
      /* Process pending RX packets while the queue is not empty and while
       * no errors occur.  NOTE that the priv->rxpending queue is accessed
       * from interrupt level processing and, hence, interrupts must be
       * disabled throughout the following.
       */

      ret = OK;

      if (!sq_empty(&priv->rxpending))
        {
          uart_recvchars_dma(&priv->serdev);
        }
    }

  /* Restart the RX failsafe timer if there are RX packets in
   * priv->rxpending.  This could happen if either RX "interrupts" are
   * disable, RX flow control is in effect of if the upper serial drivers
   * RX buffer is full and cannot accept additional data.
   *
   * If/when the timer expires, cdcacm_release_rxpending() will be called
   * the timer handler (at interrupt level).
   *
   * The timer may not be necessary, but it is a failsafe to be certain
   * that data cannot stall in priv->rxpending.
   */

  if (!sq_empty(&priv->rxpending))
    {
      wd_start(&priv->rxfailsafe, CDCACM_RXDELAY,
               cdcacm_rxtimeout, (wdparm_t)priv);
    }

out:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: cdcacm_rxtimeout
 *
 * Description:
 *   Timer expiration handler.  Whenever cdcacm_release_rxpending()
 *   terminates with  pending RX data in priv->rxpending, it will set a
 *   timer to recheck the queued RX data can be processed later.  This
 *   failsafe timer may not be necessary, but this reduces my  paranoia
 *   about stalls in the RX pending FIFO .
 *
 ****************************************************************************/

static void cdcacm_rxtimeout(wdparm_t arg)
{
  FAR struct cdcacm_dev_s *priv = (FAR struct cdcacm_dev_s *)arg;

  DEBUGASSERT(priv != NULL);
  cdcacm_release_rxpending(priv);
}

/****************************************************************************
 * Name: cdcacm_serialstate
 *
 * Description:
 *   Send the serial state message.
 *
 * 1. Format and send a request header with:
 *
 *   bmRequestType:
 *    USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
 *    USB_REQ_RECIPIENT_INTERFACE
 *   bRequest: ACM_SERIAL_STATE
 *   wValue: 0
 *   wIndex: 0
 *   wLength: Length of data = 2
 *
 * 2. Followed by the notification data
 *
 ****************************************************************************/

#ifdef CONFIG_CDCACM_IFLOWCONTROL
static int cdcacm_serialstate(FAR struct cdcacm_dev_s *priv)
{
  FAR struct usbdev_ep_s *ep;
  FAR struct usbdev_req_s *req;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  FAR struct cdc_notification_s *notify;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL && priv->epintin != NULL);
#ifdef CONFIG_DEBUG_FEATURES
  if (priv == NULL || priv->epintin == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EINVAL;
    }
#endif

  usbtrace(CDCACM_CLASSAPI_FLOWCONTROL, (uint16_t)priv->serialstate);

  flags = enter_critical_section();

  /* Use our interrupt IN endpoint for the transfer */

  ep = priv->epintin;

  /* Remove the next container from the request list */

  wrcontainer = (FAR struct cdcacm_wrreq_s *)sq_remfirst(&priv->txfree);
  if (wrcontainer == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_flags;
    }

  /* Decrement the count of write requests */

  priv->nwrq--;

  /* Format the SerialState notification */

  DEBUGASSERT(wrcontainer->req != NULL);
  req                  = wrcontainer->req;

  DEBUGASSERT(req->buf != NULL);
  notify               = (FAR struct cdc_notification_s *)req->buf;

  notify->type         = (USB_REQ_DIR_IN | USB_REQ_TYPE_CLASS |
                          USB_REQ_RECIPIENT_INTERFACE);
  notify->notification = ACM_SERIAL_STATE;
  notify->value[0]     = 0;
  notify->value[1]     = 0;
  notify->index[0]     = 0;
  notify->index[1]     = 0;
  notify->len[0]       = 2;
  notify->len[1]       = 0;
  notify->data[0]      = priv->serialstate;
  notify->data[1]      = 0;

  /* Then submit the request to the endpoint */

  req->len             = SIZEOF_NOTIFICATION_S(2);
  req->priv            = wrcontainer;
  req->flags           = USBDEV_REQFLAGS_NULLPKT;
  ret                  = EP_SUBMIT(ep, req);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL), (uint16_t)-ret);
    }

errout_with_flags:

  /* Reset all of the "irregular" notification */

  priv->serialstate &= CDC_UART_CONSISTENT;

  leave_critical_section(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: cdcacm_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void cdcacm_resetconfig(FAR struct cdcacm_dev_s *priv)
{
  /* When the USB is pulled out, if there is an unprocessed buffer,
   * it needs to be push them to upper half serial drivers RX buffer.
   */

  if (priv->nrdq != 0)
    {
      cdcacm_release_rxpending(priv);
      priv->nrdq = 0;
    }

  /* Are we configured? */

  if (priv->config != CDCACM_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      priv->config = CDCACM_CONFIGIDNONE;

      /* Inform the "upper half" driver that there is no (functional) USB
       * connection.
       */

#ifdef CONFIG_SERIAL_REMOVABLE
      uart_connected(&priv->serdev, false);
#endif

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

#ifdef CONFIG_CDCACM_HAVE_EPINTIN
      EP_DISABLE(priv->epintin);
#endif
      EP_DISABLE(priv->epbulkin);
      EP_DISABLE(priv->epbulkout);
    }
}

/****************************************************************************
 * Name: cdcacm_epconfigure
 *
 * Description:
 *   Configure one endpoint.
 *
 ****************************************************************************/

static int cdcacm_epconfigure(FAR struct usbdev_ep_s *ep,
                              enum cdcacm_epdesc_e epid, bool last,
                              FAR struct usbdev_devinfo_s *devinfo,
                              uint8_t speed)
{
  struct usb_ss_epdesc_s epdesc;
  cdcacm_copy_epdesc(epid, &epdesc.epdesc, devinfo, speed);
  return EP_CONFIGURE(ep, &epdesc.epdesc, last);
}

/****************************************************************************
 * Name: cdcacm_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int cdcacm_setconfig(FAR struct cdcacm_dev_s *priv, uint8_t config)
{
  FAR struct usbdev_req_s *req;
  int i;
  int ret = 0;

#ifdef CONFIG_DEBUG_FEATURES
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EINVAL;
    }
#endif

  if (config == priv->config)
    {
      /* Already configured -- Do nothing */

      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALREADYCONFIGURED), 0);
      return 0;
    }

  /* Discard the previous configuration data */

  cdcacm_resetconfig(priv);

  /* Was this a request to simply discard the current configuration? */

  if (config == CDCACM_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

  /* We only accept one configuration */

  if (config != CDCACM_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

#ifdef CONFIG_CDCACM_HAVE_EPINTIN
  /* Configure the IN interrupt endpoint */

  ret = cdcacm_epconfigure(priv->epintin, CDCACM_EPINTIN, false,
                           &priv->devinfo, priv->usbdev->speed);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epintin->priv = priv;
#endif

  /* Configure the IN bulk endpoint */

  ret = cdcacm_epconfigure(priv->epbulkin, CDCACM_EPBULKIN, false,
                           &priv->devinfo, priv->usbdev->speed);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Configure the OUT bulk endpoint */

  ret = cdcacm_epconfigure(priv->epbulkout, CDCACM_EPBULKOUT, true,
                           &priv->devinfo, priv->usbdev->speed);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Queue read requests in the bulk OUT endpoint */

  DEBUGASSERT(priv->nrdq == 0);
  for (i = 0; i < CONFIG_CDCACM_NRDREQS; i++)
    {
      req           = priv->rdreqs[i].req;
      req->callback = cdcacm_rdcomplete;
      ret           = EP_SUBMIT(priv->epbulkout, req);
      if (ret != OK)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT),
                   (uint16_t)-ret);
          goto errout;
        }

      priv->nrdq++;
    }

  /* We are successfully configured */

  priv->config = config;

  /* Inform the "upper half" driver that we are "open for business" */

#ifdef CONFIG_SERIAL_REMOVABLE
  uart_connected(&priv->serdev, true);
#endif

  return OK;

errout:
  cdcacm_resetconfig(priv);
  return ret;
}

/****************************************************************************
 * Name: cdcacm_ep0incomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void cdcacm_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT),
               (uint16_t)-req->result);
    }
}

/****************************************************************************
 * Name: cdcacm_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.  This
 *   is handled like the receipt of serial data on the "UART"
 *
 ****************************************************************************/

static void cdcacm_rdcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
  FAR struct cdcacm_rdreq_s *rdcontainer;
  FAR struct cdcacm_dev_s *priv;
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

  priv = (FAR struct cdcacm_dev_s *)ep->priv;

  /* Get the container of the read request */

  rdcontainer = (FAR struct cdcacm_rdreq_s *)req->priv;
  DEBUGASSERT(rdcontainer != NULL);

  /* Process the received data unless this is some unusual condition */

  flags = enter_critical_section();
  switch (req->result)
    {
    case 0: /* Normal completion */
      {
        usbtrace(TRACE_CLASSRDCOMPLETE, priv->nrdq);

        /* Place the incoming packet at the end of pending RX packet list. */

        rdcontainer->offset = 0;
        sq_addlast((FAR sq_entry_t *)rdcontainer, &priv->rxpending);

        /* Then process all pending RX packet starting at the head of the
         * list
         */

        cdcacm_release_rxpending(priv);
      }
      break;

    case -ESHUTDOWN: /* Disconnection */
      {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
        if (priv->nrdq != 0)
          {
            priv->nrdq--;
          }
      }
      break;

    default: /* Some other error occurred */
      {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED),
                                (uint16_t)-req->result);
        cdcacm_requeue_rdrequest(priv, rdcontainer);
        break;
      }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cdcacm_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void cdcacm_wrcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
  FAR struct cdcacm_dev_s *priv;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  irqstate_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req || !req->priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract references to our private data */

  priv        = (FAR struct cdcacm_dev_s *)ep->priv;
  wrcontainer = (FAR struct cdcacm_wrreq_s *)req->priv;

  /* Return the write request to the free list */

  flags = enter_critical_section();
  sq_addlast((FAR sq_entry_t *)wrcontainer, &priv->txfree);
  priv->nwrq++;
  leave_critical_section(flags);

  /* Send the next packet unless this was some unusual termination
   * condition
   */

  switch (req->result)
    {
    case OK: /* Normal completion */
      {
        usbtrace(TRACE_CLASSWRCOMPLETE, priv->nwrq);
        cdcacm_sndpacket(priv);
      }
      break;

    case -ESHUTDOWN: /* Disconnection */
      {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRSHUTDOWN), priv->nwrq);
      }
      break;

    default: /* Some other error occurred */
      {
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRUNEXPECTED),
                 (uint16_t)-req->result);
      }
      break;
    }
}

/****************************************************************************
 * USB Class Driver Methods
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int cdcacm_bind(FAR struct usbdevclass_driver_s *driver,
                       FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv =
    ((FAR struct cdcacm_driver_s *)driver)->dev;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  FAR struct cdcacm_rdreq_s *rdcontainer;
  irqstate_t flags;
  size_t reqlen;
  int ret;
  int i;

  usbtrace(TRACE_CLASSBIND, 0);

  /* Bind the structures */

  priv->usbdev   = dev;

  /* Save the reference to our private data structure in EP0 so that it
   * can be recovered in ep0 completion events (Unless we are part of
   * a composite device and, in that case, the composite device owns
   * EP0).
   */

#ifndef CONFIG_CDCACM_COMPOSITE
  dev->ep0->priv = priv;
#endif

  /* Preallocate control request */

  priv->ctrlreq = usbdev_allocreq(dev->ep0, CDCACM_MXDESCLEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      ret = -ENOMEM;
      goto errout;
    }

  priv->ctrlreq->callback = cdcacm_ep0incomplete;

  /* Pre-allocate all endpoints... the endpoints will not be functional
   * until the SET CONFIGURATION request is processed in cdcacm_setconfig.
   * This is done here because there may be calls to kmm_malloc and the SET
   * CONFIGURATION processing probably occurs within interrupt handling
   * logic where kmm_malloc calls will fail.
   */

#ifdef CONFIG_CDCACM_HAVE_EPINTIN
  /* Pre-allocate the IN interrupt endpoint */

  priv->epintin = DEV_ALLOCEP(dev, CDCACM_MKEPINTIN(&priv->devinfo),
                              true, USB_EP_ATTR_XFER_INT);
  if (!priv->epintin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epintin->priv = priv;
#endif

  /* Pre-allocate the IN bulk endpoint */

  priv->epbulkin = DEV_ALLOCEP(dev, CDCACM_MKEPBULKIN(&priv->devinfo),
                               true, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Pre-allocate the OUT bulk endpoint */

  priv->epbulkout = DEV_ALLOCEP(dev, CDCACM_MKEPBULKOUT(&priv->devinfo),
                                false, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkout)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Pre-allocate read requests.  The buffer size is one full packet. */
#if defined(CONFIG_USBDEV_SUPERSPEED)
  if (dev->speed == USB_SPEED_SUPER ||
      dev->speed == USB_SPEED_SUPER_PLUS)
    {
      if (CONFIG_CDCACM_EPBULKOUT_MAXBURST < USB_SS_BULK_EP_MAXBURST)
        {
          reqlen = CONFIG_CDCACM_EPBULKOUT_SSSIZE *
                   (CONFIG_CDCACM_EPBULKOUT_MAXBURST + 1);
        }
      else
        {
          reqlen = CONFIG_CDCACM_EPBULKOUT_SSSIZE *
                   USB_SS_BULK_EP_MAXBURST;
        }
    }
  else
#endif
#if defined(CONFIG_USBDEV_DUALSPEED)
  if (dev->speed == USB_SPEED_HIGH)
    {
      reqlen = CONFIG_CDCACM_EPBULKOUT_HSSIZE;
    }
  else
#endif
    {
      reqlen = CONFIG_CDCACM_EPBULKOUT_FSSIZE;
    }

  if (CONFIG_CDCACM_BULKOUT_REQLEN > reqlen)
    {
      reqlen = CONFIG_CDCACM_BULKOUT_REQLEN;
    }

  for (i = 0; i < CONFIG_CDCACM_NRDREQS; i++)
    {
      rdcontainer      = &priv->rdreqs[i];
      rdcontainer->req = usbdev_allocreq(priv->epbulkout, reqlen);
      if (rdcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }

      rdcontainer->offset        = 0;
      rdcontainer->req->priv     = rdcontainer;
      rdcontainer->req->callback = cdcacm_rdcomplete;
    }

  /* Pre-allocate write request containers and put in a free list.  The
   * buffer size should be larger than a full build IN packet.  Otherwise,
   * we will send a bogus null packet at the end of each packet.
   *
   * Pick the larger of the max packet size and the configured request size.
   *
   * NOTE: These write requests are sized for the bulk IN endpoint but are
   * shared with interrupt IN endpoint which does not need a large buffer.
   */

#if defined(CONFIG_USBDEV_SUPERSPEED)
  if (dev->speed == USB_SPEED_SUPER ||
      dev->speed == USB_SPEED_SUPER_PLUS)
    {
      if (CONFIG_CDCACM_EPBULKIN_MAXBURST < USB_SS_BULK_EP_MAXBURST)
        {
          reqlen = CONFIG_CDCACM_EPBULKOUT_SSSIZE *
                   (CONFIG_CDCACM_EPBULKIN_MAXBURST + 1);
        }
      else
        {
          reqlen = CONFIG_CDCACM_EPBULKOUT_SSSIZE *
                   USB_SS_BULK_EP_MAXBURST;
        }
    }
  else
#endif
#if defined(CONFIG_USBDEV_DUALSPEED)
  if  (dev->speed == USB_SPEED_HIGH)
    {
      reqlen = CONFIG_CDCACM_EPBULKIN_HSSIZE;
    }
  else
#endif
    {
      reqlen = CONFIG_CDCACM_EPBULKIN_FSSIZE;
    }

  if (CONFIG_CDCACM_BULKIN_REQLEN > reqlen)
    {
      reqlen = CONFIG_CDCACM_BULKIN_REQLEN;
    }

  for (i = 0; i < CONFIG_CDCACM_NWRREQS; i++)
    {
      wrcontainer      = &priv->wrreqs[i];
      wrcontainer->req = usbdev_allocreq(priv->epbulkin, reqlen);
      if (wrcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }

      wrcontainer->req->priv     = wrcontainer;
      wrcontainer->req->callback = cdcacm_wrcomplete;

      flags = enter_critical_section();
      sq_addlast((FAR sq_entry_t *)wrcontainer, &priv->txfree);
      priv->nwrq++;     /* Count of write requests available */
      leave_critical_section(flags);
    }

  /* Report if we are selfpowered (unless we are part of a
   * composite device)
   */

#ifndef CONFIG_CDCACM_COMPOSITE
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
  cdcacm_unbind(driver, dev);
  return ret;
}

/****************************************************************************
 * Name: cdcacm_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void cdcacm_unbind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  FAR struct cdcacm_rdreq_s *rdcontainer;
  irqstate_t flags;
  int i;

  usbtrace(TRACE_CLASSUNBIND, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Make sure that we are not already unbound */

  if (priv != NULL)
    {
      /* Make sure that the endpoints have been unconfigured.  If
       * we were terminated gracefully, then the configuration should
       * already have been reset.  If not, then calling cdcacm_resetconfig
       * should cause the endpoints to immediately terminate all
       * transfers and return the requests to us (with result == -ESHUTDOWN)
       */

      cdcacm_resetconfig(priv);

      /* Free the pre-allocated control request */

      if (priv->ctrlreq != NULL)
        {
          usbdev_freereq(dev->ep0, priv->ctrlreq);
          priv->ctrlreq = NULL;
        }

      /* Free pre-allocated read requests (which should all have
       * been returned to the free list at this time -- we don't check)
       */

      DEBUGASSERT(priv->nrdq == 0);
      for (i = 0; i < CONFIG_CDCACM_NRDREQS; i++)
        {
          rdcontainer = &priv->rdreqs[i];
          if (rdcontainer->req)
            {
              usbdev_freereq(priv->epbulkout, rdcontainer->req);
              rdcontainer->req = NULL;
            }
        }

      /* Free write requests that are not in use (which should be all
       * of them)
       */

      flags = enter_critical_section();
      DEBUGASSERT(priv->nwrq == CONFIG_CDCACM_NWRREQS);

      while (!sq_empty(&priv->txfree))
        {
          wrcontainer = (struct cdcacm_wrreq_s *)sq_remfirst(&priv->txfree);
          if (wrcontainer->req != NULL)
            {
              usbdev_freereq(priv->epbulkin, wrcontainer->req);
              priv->nwrq--;     /* Number of write requests queued */
            }
        }

      DEBUGASSERT(priv->nwrq == 0);
      leave_critical_section(flags);

#ifdef CONFIG_CDCACM_HAVE_EPINTIN
      /* Free the interrupt IN endpoint */

      if (priv->epintin)
        {
          DEV_FREEEP(dev, priv->epintin);
          priv->epintin = NULL;
        }
#endif

      /* Free the bulk OUT endpoint */

      if (priv->epbulkout)
        {
          DEV_FREEEP(dev, priv->epbulkout);
          priv->epbulkout = NULL;
        }

      /* Free the bulk IN endpoint */

      if (priv->epbulkin)
        {
          DEV_FREEEP(dev, priv->epbulkin);
          priv->epbulkin = NULL;
        }

      /* Clear out all data in the circular buffer */

      priv->serdev.xmit.head = 0;
      priv->serdev.xmit.tail = 0;
    }
}

/****************************************************************************
 * Name: cdcacm_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int cdcacm_setup(FAR struct usbdevclass_driver_s *driver,
                        FAR struct usbdev_s *dev,
                        FAR const struct usb_ctrlreq_s *ctrl,
                        FAR uint8_t *dataout, size_t outlen)
{
  FAR struct cdcacm_dev_s *priv;
  FAR struct usbdev_req_s *ctrlreq;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  int ret = -EOPNOTSUPP;

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EINVAL;
    }
#endif

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

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
  index = GETUINT16(ctrl->index);
  len   = GETUINT16(ctrl->len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      /**********************************************************************
       * Standard Requests
       **********************************************************************/

      switch (ctrl->req)
        {
        case USB_REQ_GETDESCRIPTOR:
          {
            /* The value field specifies the descriptor type in the MS byte
             * and the descriptor index in the LS byte (order is little
             * endian)
             */

            switch (ctrl->value[1])
              {
              /* If the serial device is used in as part of a composite
               * device, then the device descriptor is provided by logic in
               * the composite device implementation.
               */

#ifndef CONFIG_CDCACM_COMPOSITE
              case USB_DESC_TYPE_DEVICE:
                {
                  ret = usbdev_copy_devdesc(ctrlreq->buf,
                                            cdcacm_getdevdesc(),
                                            dev->speed);
                }
                break;
#endif

              /* If the serial device is used in as part of a composite
               * device, then the device qualifier descriptor is provided by
               * logic in the composite device implementation.
               */

#if !defined(CONFIG_CDCACM_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
              case USB_DESC_TYPE_DEVICEQUALIFIER:
                {
                  ret = USB_SIZEOF_QUALDESC;
                  memcpy(ctrlreq->buf, cdcacm_getqualdesc(), ret);
                }
                break;

              case USB_DESC_TYPE_OTHERSPEEDCONFIG:
#endif

              /* If the serial device is used in as part of a composite
               * device, then the configuration descriptor is provided by
               * logic in the composite device implementation.
               */

#ifndef CONFIG_CDCACM_COMPOSITE
              case USB_DESC_TYPE_CONFIG:
                {
                  ret = cdcacm_mkcfgdesc(ctrlreq->buf, &priv->devinfo,
                                         dev->speed, ctrl->value[1]);
                }
                break;
#endif

              /* If the serial device is used in as part of a composite
               * device, then the language string descriptor is provided by
               * logic in the composite device implementation.
               */

#ifndef CONFIG_CDCACM_COMPOSITE
              case USB_DESC_TYPE_STRING:
                {
                  /* index == language code. */

                  ret =
                  cdcacm_mkstrdesc(ctrl->value[0],
                                  (FAR struct usb_strdesc_s *)
                                    ctrlreq->buf);
                }
                break;
#endif

              default:
                {
                  usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_GETUNKNOWNDESC),
                           value);
                }
                break;
              }
          }
          break;

        case USB_REQ_SETCONFIGURATION:
          {
            if (ctrl->type == 0)
              {
                ret = cdcacm_setconfig(priv, value);
              }
          }
          break;

        /* If the serial device is used in as part of a composite device,
         * then the overall composite class configuration is managed by logic
         * in the composite device implementation.
         */

#ifndef CONFIG_CDCACM_COMPOSITE
        case USB_REQ_GETCONFIGURATION:
          {
            if (ctrl->type == USB_DIR_IN)
              {
                *(FAR uint8_t *)ctrlreq->buf = priv->config;
                ret = 1;
              }
          }
          break;
#endif

        case USB_REQ_SETINTERFACE:
          {
            if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE &&
                priv->config == CDCACM_CONFIGID)
              {
                  if ((index == priv->devinfo.ifnobase &&
                       value == CDCACM_NOTALTIFID) ||
                      (index == (priv->devinfo.ifnobase + 1) &&
                       value == CDCACM_DATAALTIFID))
                  {
                    cdcacm_resetconfig(priv);
                    cdcacm_setconfig(priv, CDCACM_CONFIGID);
                    ret = 0;
                  }
              }
          }
          break;

        case USB_REQ_GETINTERFACE:
          {
            if (ctrl->type == (USB_DIR_IN | USB_REQ_RECIPIENT_INTERFACE) &&
                priv->config == CDCACM_CONFIGIDNONE)
              {
                  if ((index == priv->devinfo.ifnobase &&
                       value == CDCACM_NOTALTIFID) ||
                      (index == (priv->devinfo.ifnobase + 1) &&
                       value == CDCACM_DATAALTIFID))
                   {
                    *(FAR uint8_t *) ctrlreq->buf = value;
                    ret = 1;
                  }
                else
                  {
                    ret = -EDOM;
                  }
              }
           }
           break;

        default:
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ),
                   ctrl->req);
          break;
        }
    }

  else if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    {
      /**********************************************************************
       * CDC ACM-Specific Requests
       **********************************************************************/

      switch (ctrl->req)
        {
        /* ACM_GET_LINE_CODING requests current DTE rate, stop-bits, parity,
         * and number-of-character bits. (Optional)
         */

        case ACM_GET_LINE_CODING:
          {
            if (ctrl->type == (USB_DIR_IN | USB_REQ_TYPE_CLASS |
                               USB_REQ_RECIPIENT_INTERFACE) &&
                index == priv->devinfo.ifnobase)
              {
                /* Return the current line status from the private data
                 * structure.
                 */

                memcpy(ctrlreq->buf, &priv->linecoding,
                       SIZEOF_CDC_LINECODING);
                ret = SIZEOF_CDC_LINECODING;
              }
            else
              {
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                        ctrl->type);
              }
          }
          break;

        /* ACM_SET_LINE_CODING configures DTE rate, stop-bits, parity, and
         * number-of-character bits. (Optional)
         */

        case ACM_SET_LINE_CODING:
          {
            if (ctrl->type == (USB_DIR_OUT | USB_REQ_TYPE_CLASS |
                               USB_REQ_RECIPIENT_INTERFACE) &&
                len == SIZEOF_CDC_LINECODING && /* dataout && len == outlen && */
                index == priv->devinfo.ifnobase)
              {
                /* Save the new line coding in the private data structure.
                 * NOTE: that this is conditional now because not all device
                 * controller drivers supported provision of EP0 OUT data
                 * with the setup command.
                 */

                /* REVISIT */

                if (dataout && len <= SIZEOF_CDC_LINECODING)
                  {
                    memcpy(&priv->linecoding,
                           dataout, SIZEOF_CDC_LINECODING);
                  }

                /* Respond with a zero length packet */

                ret = 0;

                /* If there is a registered callback to receive line status
                 * info, then callout now.
                 */

                if (priv->callback)
                  {
                    priv->callback(CDCACM_EVENT_LINECODING);
                  }
              }
            else
              {
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                         ctrl->type);
              }
          }
          break;

        /* ACM_SET_CTRL_LINE_STATE: RS-232 signal used to tell the DCE
         * device the DTE device is now present. (Optional)
         */

        case ACM_SET_CTRL_LINE_STATE:
          {
            if (ctrl->type == (USB_DIR_OUT | USB_REQ_TYPE_CLASS |
                               USB_REQ_RECIPIENT_INTERFACE) &&
                index == priv->devinfo.ifnobase)
              {
                /* Save the control line state in the private data
                 * structure. Only bits 0 and 1 have meaning.  Respond with
                 * a zero length packet.
                 */

                priv->ctrlline = value & 3;
                ret = 0;

                /* If there is a registered callback to receive control line
                 * status info, then call out now.
                 */

                if (priv->callback)
                  {
                    priv->callback(CDCACM_EVENT_CTRLLINE);
                  }
              }
            else
              {
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                         ctrl->type);
              }
          }
          break;

        /*  Sends special carrier */

        case ACM_SEND_BREAK:
          {
            if (ctrl->type == (USB_DIR_OUT | USB_REQ_TYPE_CLASS |
                               USB_REQ_RECIPIENT_INTERFACE) &&
                index == priv->devinfo.ifnobase)
              {
                /* If there is a registered callback to handle the SendBreak
                 * request, then call out now.  Respond with a zero length
                 * packet.
                 */

                ret = 0;
                if (priv->callback)
                  {
                    priv->callback(CDCACM_EVENT_SENDBREAK);
                  }
              }
            else
              {
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                         ctrl->type);
              }
          }
          break;

        default:
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ),
                   ctrl->req);
          break;
        }
    }
  else
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
    }

  /* Respond to the setup command if data was returned.  On an error return
   * value (ret < 0), the USB driver will stall.
   */

  if (ret >= 0)
    {
      /* Configure the response */

      ctrlreq->len   = MIN(len, ret);
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;

      /* Send the response -- either directly to the USB controller or
       * indirectly in the case where this class is a member of a composite
       * device.
       */

#ifndef CONFIG_CDCACM_COMPOSITE
      ret = EP_SUBMIT(dev->ep0, ctrlreq);
#else
      ret = composite_ep0submit(driver, dev, ctrlreq, ctrl);
#endif
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
          cdcacm_ep0incomplete(dev->ep0, ctrlreq);
        }
    }

  /* Returning a negative value will cause a STALL */

  return ret;
}

/****************************************************************************
 * Name: cdcacm_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void cdcacm_disconnect(FAR struct usbdevclass_driver_s *driver,
                              FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv;
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

  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Inform the "upper half serial driver that we have lost the USB serial
   * connection.
   */

  flags = enter_critical_section();
#ifdef CONFIG_SERIAL_REMOVABLE
  uart_connected(&priv->serdev, false);
#endif

  /* Reset the configuration */

  cdcacm_resetconfig(priv);

  /* Clear out all outgoing data in the circular buffer */

  priv->serdev.xmit.head = 0;
  priv->serdev.xmit.tail = 0;
  leave_critical_section(flags);

  /* Perform the soft connect function so that we will we can be
   * re-enumerated (unless we are part of a composite device)
   */

#ifndef CONFIG_CDCACM_COMPOSITE
  DEV_CONNECT(dev);
#endif
}

/****************************************************************************
 * Name: cdcacm_suspend
 *
 * Description:
 *   Handle the USB suspend event.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_REMOVABLE
static void cdcacm_suspend(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv;

  usbtrace(TRACE_CLASSSUSPEND, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

  /* And let the "upper half" driver now that we are suspended */

  uart_connected(&priv->serdev, false);
}
#endif

/****************************************************************************
 * Name: cdcacm_resume
 *
 * Description:
 *   Handle the USB resume event.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_REMOVABLE
static void cdcacm_resume(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  FAR struct cdcacm_dev_s *priv;

  usbtrace(TRACE_CLASSRESUME, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct cdcacm_driver_s *)driver)->dev;

  /* Are we still configured? */

  if (priv->config != CDCACM_CONFIGIDNONE)
    {
      /* Yes.. let the "upper half" know that have resumed */

      uart_connected(&priv->serdev, true);
    }
}
#endif

/****************************************************************************
 * Serial Device Methods
 ****************************************************************************/

/****************************************************************************
 * Name: cdcuart_setup
 *
 * Description:
 *   This method is called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int cdcuart_setup(FAR struct uart_dev_s *dev)
{
  FAR struct cdcacm_dev_s *priv;

  usbtrace(CDCACM_CLASSAPI_SETUP, 0);

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev || !dev->priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EINVAL;
    }
#endif

  /* Extract reference to private data */

  priv = (FAR struct cdcacm_dev_s *)dev->priv;

  /* Check if we have been configured */

  if (priv->config == CDCACM_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SETUPNOTCONNECTED), 0);
      return -ENOTCONN;
    }

  return OK;
}

/****************************************************************************
 * Name: cdcuart_shutdown
 *
 * Description:
 *   This method is called when the serial port is closed.  This operation
 *   is very simple for the USB serial back-end because the serial driver
 *   has already assured that the TX data has full drained -- it calls
 *   cdcuart_txempty() until that function returns true before calling this
 *   function.
 *
 ****************************************************************************/

static void cdcuart_shutdown(FAR struct uart_dev_s *dev)
{
  usbtrace(CDCACM_CLASSAPI_SHUTDOWN, 0);

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
    }
#endif
}

/****************************************************************************
 * Name: cdcuart_attach
 *
 * Description:
 *   Does not apply to the USB serial class device
 *
 ****************************************************************************/

static int cdcuart_attach(FAR struct uart_dev_s *dev)
{
  usbtrace(CDCACM_CLASSAPI_ATTACH, 0);
  return OK;
}

/****************************************************************************
 * Name: cdcuart_detach
 *
 * Description:
 *   Does not apply to the USB serial class device
 *
 ****************************************************************************/

static void cdcuart_detach(FAR struct uart_dev_s *dev)
{
  usbtrace(CDCACM_CLASSAPI_DETACH, 0);
}

/****************************************************************************
 * Name: cdcuart_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int cdcuart_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  struct inode        *inode  = filep->f_inode;
  struct cdcacm_dev_s *priv   = inode->i_private;
  int                  ret    = OK;

  switch (cmd)
    {
    /* CAICO_REGISTERCB
     *   Register a callback for serial event notification. Argument:
     *   cdcacm_callback_t.  See cdcacm_callback_t type definition below.
     *   NOTE:  The callback will most likely invoked at the interrupt level.
     *   The called back function should, therefore, limit its operations to
     *   invoking some kind of IPC to handle the serial event in some normal
     *   task environment.
     */

    case CAIOC_REGISTERCB:
      {
        /* Save the new callback function */

        priv->callback = (cdcacm_callback_t)((uintptr_t)arg);
      }
      break;

    /* CAIOC_GETLINECODING
     *   Get current line coding.  Argument: struct cdc_linecoding_s*.
     *   See include/nuttx/usb/cdc.h for structure definition.  This IOCTL
     *   should be called to get the data associated with the
     *   CDCACM_EVENT_LINECODING event).
     */

    case CAIOC_GETLINECODING:
      {
        FAR struct cdc_linecoding_s *ptr =
          (FAR struct cdc_linecoding_s *)((uintptr_t)arg);
        if (ptr != NULL)
          {
            memcpy(ptr, &priv->linecoding, sizeof(struct cdc_linecoding_s));
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

    /* CAIOC_GETCTRLLINE
     *   Get control line status bits. Argument FAR int*.  See
     *   include/nuttx/usb/cdc.h for bit definitions.  This IOCTL should be
     *   called to get the data associated CDCACM_EVENT_CTRLLINE event.
     */

    case CAIOC_GETCTRLLINE:
      {
        FAR int *ptr = (FAR int *)((uintptr_t)arg);
        if (ptr != NULL)
          {
            *ptr = priv->ctrlline;
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

#ifdef CONFIG_CDCACM_IFLOWCONTROL
    /* CAIOC_NOTIFY
     *   Send a serial state to the host via the Interrupt IN endpoint.
     *   Argument: int.  This includes the current state of the carrier
     *   detect, DSR, break, and ring signal.  See "Table 69: UART State
     *   Bitmap Values" and CDC_UART_definitions in include/nuttx/usb/cdc.h.
     */

    case CAIOC_NOTIFY:
      {
        DEBUGASSERT(arg < UINT8_MAX);

        priv->serialstate = (uint8_t)arg;
        ret = cdcacm_serialstate(priv);
      }
      break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (FAR struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* And update with flags from this layer */

        termiosp->c_cflag =
            ((priv->linecoding.parity != CDC_PARITY_NONE) ? PARENB : 0) |
            ((priv->linecoding.parity == CDC_PARITY_ODD) ? PARODD : 0) |
            ((priv->linecoding.stop == CDC_CHFMT_STOP2) ? CSTOPB : 0) |
            CS8;

#ifdef CONFIG_CDCACM_OFLOWCONTROL
        /* Report state of output flow control */

#  warning Missing logic
#endif
#ifdef CONFIG_CDCACM_IFLOWCONTROL
        /* Report state of input flow control */

        termiosp->c_cflag |= (priv->iflow) ? CRTS_IFLOW : 0;
#endif
      cfsetispeed(termiosp, (speed_t)priv->linecoding.baud[3] << 24 |
                            (speed_t)priv->linecoding.baud[2] << 16 |
                            (speed_t)priv->linecoding.baud[1] << 8  |
                            (speed_t)priv->linecoding.baud[0]);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (FAR struct termios *)arg;
#ifdef CONFIG_CDCACM_IFLOWCONTROL
        bool iflow;
#endif

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Update the flags we keep at this layer */

#ifdef CONFIG_CDCACM_OFLOWCONTROL
        /* Handle changes to output flow control */

#  warning Missing logic
#endif

#ifdef CONFIG_CDCACM_IFLOWCONTROL
        /* Handle changes to input flow control */

        iflow = ((termiosp->c_cflag & CRTS_IFLOW) != 0);
        if (iflow != priv->iflow)
          {
            /* Check if flow control has been disabled. */

            if (!iflow)
              {
                /* Flow control has been disabled.  We need to make sure
                 * that DSR is set unconditionally.
                 */

                if ((priv->serialstate & CDCACM_UART_DSR) == 0)
                  {
                    priv->serialstate |= (CDCACM_UART_DSR | CDCACM_UART_DCD);
                    ret = cdcacm_serialstate(priv);
                  }

                /* Save the new flow control setting. */

                priv->iflow   = false;
                priv->iactive = false;

                /* During the time that flow control was disabled, incoming
                 * packets were queued in priv->rxpending.  We must now
                 * process all of them (unless RX interrupts are also
                 * disabled)
                 */

                cdcacm_release_rxpending(priv);
              }

            /* Flow control has been enabled. */

            else
              {
                /* Save the new flow control setting. */

                priv->iflow        = true;
                priv->iactive      = false;

                /* If the RX buffer is already (nearly) full, the we need to
                 * make sure the DSR is clear.
                 *
                 * NOTE: Here we assume that DSR is set so we don't check its
                 * current value nor to we handle the case where we would set
                 * DSR because the RX buffer is (nearly) empty!
                 */

                if (priv->upper)
                  {
                    priv->serialstate &= ~CDCACM_UART_DSR;
                    priv->serialstate |= CDCACM_UART_DCD;
                    ret = cdcacm_serialstate(priv);

                    /* Input flow control is now active */

                    priv->iactive      = true;
                  }
              }

            /* RX "interrupts are no longer disabled */

            priv->rxenabled = true;
          }
#endif
      }
      break;
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: cdcuart_rxint
 *
 * Description:
 *   Called by the serial driver to enable or disable RX interrupts.  We, of
 *   course, have no RX interrupts but must behave consistently.  This method
 *   is called under the conditions:
 *
 *   1. With enable==true when the port is opened (just after cdcuart_setup
 *      and cdcuart_attach are called called)
 *   2. With enable==false while transferring data from the RX buffer
 *   2. With enable==true while waiting for more incoming data
 *   3. With enable==false when the port is closed (just before
 *      cdcuart_detach and cdcuart_shutdown are called).
 *
 * Assumptions:
 *   Called from the serial upper-half driver running on the thread of
 *   execution of the caller of the driver or, possibly, on from the
 *   USB interrupt handler (at least for the case where the RX interrupt
 *   is disabled)
 *
 ****************************************************************************/

static void cdcuart_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct cdcacm_dev_s *priv;
  irqstate_t flags;

  usbtrace(CDCACM_CLASSAPI_RXINT, (uint16_t)enable);

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
       return;
    }
#endif

  /* Extract reference to private data */

  priv = (FAR struct cdcacm_dev_s *)dev->priv;

  /* We need exclusive access to the RX buffer and private structure
   * in the following.
   */

  flags = enter_critical_section();
  if (enable)
    {
      /* RX "interrupts" are enabled.  Is this a transition from disabled
       * to enabled state?
       */

      if (!priv->rxenabled)
        {
          /* Yes.. RX "interrupts are no longer disabled */

          priv->rxenabled = true;
        }

      /* During the time that RX interrupts was disabled, incoming
       * packets were queued in priv->rxpending.  We must now process
       * all of them (unless flow control is enabled)
       *
       * NOTE: This action may cause this function to be re-entered
       * with enable == false , anyway the pend-list should be flushed
       */

      cdcacm_release_rxpending(priv);
    }

  /* RX "interrupts" are disabled.  Nothing special needs to be done on a
   * transition from the enabled to the disabled state.
   */

  else
    {
      priv->rxenabled = false;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: cdcuart_rxflowcontrol
 *
 * Description:
 *   Called when Rx buffer is full (or exceeds configured watermark levels
 *   if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is defined).
 *   Return true if UART activated RX flow control to block more incoming
 *   data
 *
 * Input Parameters:
 *   dev       - UART device instance
 *   nbuffered - the number of characters currently buffered
 *               (if CONFIG_SERIAL_IFLOWCONTROL_WATERMARKS is
 *               not defined the value will be 0 for an empty buffer or the
 *               defined buffer size for a full buffer)
 *   upper     - true indicates the upper watermark was crossed where
 *               false indicates the lower watermark has been crossed
 *
 * Returned Value:
 *   true if RX flow control activated.
 *
 ****************************************************************************/

#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool cdcuart_rxflowcontrol(FAR struct uart_dev_s *dev,
                                  unsigned int nbuffered, bool upper)
{
#ifdef CONFIG_CDCACM_IFLOWCONTROL
  FAR struct cdcacm_dev_s *priv;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (dev == NULL || dev->priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return false;
    }
#endif

  /* Extract reference to private data */

  priv = (FAR struct cdcacm_dev_s *)dev->priv;

  /* Is input flow control enabled? */

  priv->upper = upper;
  if (priv->iflow)
    {
      /* Yes.. Set DSR (TX carrier) if the lower water mark has been crossed
       * or clear it if the upper water mark has been crossed.
       */

      if (upper)
        {
          /* Don't do anything unless this results in a change in the
           * setting of DSR.
           */

          if ((priv->serialstate & CDCACM_UART_DSR) != 0)
            {
              /* Clear DSR (set DCD in any case). */

              priv->serialstate &= ~CDCACM_UART_DSR;
              priv->serialstate |= CDCACM_UART_DCD;

              /* And send the SerialState message.
               * REVISIT: Error return case.  Would an error mean DSR is not
               * set?
               */

              cdcacm_serialstate(priv);
            }

          /* Flow control is active */

          priv->iactive = true;
        }

      /* Lower watermark crossing.  Don't do anything unless this results in
       * a change in the setting of DSR.
       */

      else
        {
          /* Flow control is not active (Needed before calling
           * cdcacm_release_rxpending())
           */

          priv->iactive = false;

          /* Set DSR if it is not already set */

          if ((priv->serialstate & CDCACM_UART_DSR) == 0)
            {
              priv->serialstate |= (CDCACM_UART_DSR | CDCACM_UART_DCD);

              /* And send the SerialState message.
               * REVISIT: Error return case.  Would an error mean DSR is
               *  still clear?
               */

              cdcacm_serialstate(priv);
            }

          /* During the time that flow control ws disabled, incoming packets
           * were queued in priv->rxpending.  We must now process all of
           * them (unless RX interrupts becomes enabled)
           *
           * NOTE: This action may cause this function to be re-entered with
           * upper == false.
           */

          cdcacm_release_rxpending(priv);
        }
    }
  else
    {
      /* Flow control is disabled ... DSR must be set */

      if ((priv->serialstate & CDCACM_UART_DSR) == 0)
        {
          /* Set DSR and DCD */

          priv->serialstate |= (CDCACM_UART_DSR | CDCACM_UART_DCD);

          /* And send the SerialState message
           * REVISIT: Error return case.  Would an error mean DSR is still
           * not set?
           */

          cdcacm_serialstate(priv);

          /* Flow control is not active */

          priv->iactive = false;
        }
    }

  /* Return true flow control is active */

  return priv->iactive;
#else

  return false;
#endif
}
#endif

/****************************************************************************
 * Name: cdcuart_txint
 *
 * Description:
 *   Called by the serial driver to enable or disable TX interrupts.  We, of
 *   course, have no TX interrupts but must behave consistently.  Initially,
 *   TX interrupts are disabled.  This method is called under the conditions:
 *
 *   1. With enable==false while transferring data into the TX buffer
 *   2. With enable==true when data may be taken from the buffer.
 *   3. With enable==false when the TX buffer is empty
 *
 ****************************************************************************/

static void cdcuart_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct cdcacm_dev_s *priv;

  usbtrace(CDCACM_CLASSAPI_TXINT, (uint16_t)enable);

  /* Sanity checks */

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
       return;
    }
#endif

  /* Extract references to private data */

  priv = (FAR struct cdcacm_dev_s *)dev->priv;

  /* If the new state is enabled and if there is data in the XMIT buffer,
   * send the next packet now.
   */

  uinfo("enable=%d head=%d tail=%d\n",
        enable, priv->serdev.xmit.head, priv->serdev.xmit.tail);

  if (enable && priv->serdev.xmit.head != priv->serdev.xmit.tail)
    {
      cdcacm_sndpacket(priv);
    }
}

/****************************************************************************
 * Name: cdcuart_txempty
 *
 * Description:
 *   Return true when all data has been sent.  This is called from the
 *   serial driver when the driver is closed.  It will call this API
 *   periodically until it reports true.  NOTE that the serial driver takes
 *   all responsibility for flushing TX data through the hardware so we can
 *   be a bit sloppy about that.
 *
 ****************************************************************************/

static bool cdcuart_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct cdcacm_dev_s *priv = (FAR struct cdcacm_dev_s *)dev->priv;
  FAR struct usbdev_ep_s *ep = priv->epbulkin;
  irqstate_t flags;
  bool empty;

  usbtrace(CDCACM_CLASSAPI_TXEMPTY, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return true;
    }
#endif

  flags = enter_critical_section();
  priv->ispolling = true;
  EP_POLL(ep);
  priv->ispolling = false;

  /* When all of the allocated write requests have been returned to the
   * txfree, then there is no longer any TX data in flight.
   */

  empty = priv->nwrq >= CONFIG_CDCACM_NWRREQS;
  leave_critical_section(flags);

  return empty;
}

/****************************************************************************
 * Name: cdcuart_release
 *
 * Description:
 *   This is called to release some resource about the device when device
 *   was close and unregistered.
 *
 ****************************************************************************/

static int cdcuart_release(FAR struct uart_dev_s *dev)
{
  FAR struct cdcacm_dev_s *priv = (FAR struct cdcacm_dev_s *)dev->priv;

  usbtrace(CDCACM_CLASSAPI_RELEASE, 0);

  /* And free the memory resources. */

  wd_cancel(&priv->rxfailsafe);
  kmm_free(priv);
  return OK;
}

/****************************************************************************
 * Name: cdcuart_dmasend
 *
 * Description:
 *   Set up to transfer bytes from the TX circular buffer.
 *
 ****************************************************************************/

static void cdcuart_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct uart_dmaxfer_s *xfer = &dev->dmatx;
  FAR struct cdcacm_dev_s *priv = dev->priv;
  FAR struct usbdev_ep_s *ep = priv->epbulkin;
  FAR struct cdcacm_wrreq_s *wrcontainer;
  FAR struct usbdev_req_s *req;
  size_t nbytes;
  size_t reqlen;
  int ret;

  /* Get the maximum number of bytes that will fit into one bulk IN request */

  reqlen = MIN(CONFIG_CDCACM_BULKIN_REQLEN, ep->maxpacket);

  /* Peek at the request in the container at the head of the list */

  wrcontainer = (FAR struct cdcacm_wrreq_s *)sq_remfirst(&priv->txfree);
  req = wrcontainer->req;
  priv->nwrq--;

  /* Fill the request with serial TX data */

  nbytes = MIN(reqlen, xfer->length);
  memcpy(req->buf, xfer->buffer, nbytes);
  req->len = nbytes;

  nbytes = MIN(reqlen - nbytes, xfer->nlength);
  memcpy(req->buf + req->len, xfer->nbuffer, nbytes);
  req->len += nbytes;
  xfer->nbytes = req->len;

  uart_xmitchars_done(dev);

  /* Then submit the request to the endpoint */

  req->priv  = wrcontainer;
  req->flags = USBDEV_REQFLAGS_NULLPKT;
  ret        = EP_SUBMIT(ep, req);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL),
               (uint16_t)-ret);
    }
}

/****************************************************************************
 * Name: cdcuart_dmareceive
 *
 * Description:
 *   Set up to receive bytes into the RX circular buffer.
 *
 ****************************************************************************/

static void cdcuart_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct uart_dmaxfer_s *xfer = &dev->dmarx;
  FAR struct cdcacm_dev_s *priv = dev->priv;
  FAR struct cdcacm_rdreq_s *rdcontainer;
  FAR struct usbdev_req_s *req;
  FAR uint8_t *reqbuf;
  size_t nbytes = 0;
  size_t reqlen;

  /* Process each packet in the priv->rxpending list */

  rdcontainer = (FAR struct cdcacm_rdreq_s *)
    sq_peek(&priv->rxpending);
  DEBUGASSERT(rdcontainer != NULL);

  req = rdcontainer->req;
  DEBUGASSERT(req != NULL);

  reqbuf = &req->buf[rdcontainer->offset];
  reqlen = req->xfrd - rdcontainer->offset;

  nbytes = MIN(reqlen, xfer->length);
  memcpy(xfer->buffer, reqbuf, nbytes);
  rdcontainer->offset += nbytes;
  xfer->nbytes = nbytes;

  if (xfer->nbuffer)
    {
      nbytes = MIN(reqlen - nbytes, xfer->nlength);
      memcpy(xfer->nbuffer, reqbuf + xfer->nbytes, nbytes);
      rdcontainer->offset += nbytes;
      xfer->nbytes += nbytes;
    }

  uart_recvchars_done(dev);

  /* The entire packet was processed and may be removed from the
   * pending RX list.
   */

  if (rdcontainer->offset >= rdcontainer->req->xfrd)
    {
      sq_remfirst(&priv->rxpending);
      cdcacm_requeue_rdrequest(priv, rdcontainer);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_classobject
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured) and
 *   return the class object.
 *
 * Input Parameters:
 *   minor - Device minor number.  E.g., minor 0 would correspond to
 *     /dev/ttyACM0.
 *   classdev - The location to return the CDC serial class' device
 *     instance.
 *
 * Returned Value:
 *   A pointer to the allocated class object (NULL on failure).
 *
 ****************************************************************************/

#ifndef CONFIG_CDCACM_COMPOSITE
static
#endif
int cdcacm_classobject(int minor, FAR struct usbdev_devinfo_s *devinfo,
                       FAR struct usbdevclass_driver_s **classdev)
{
  FAR struct cdcacm_alloc_s *alloc;
  FAR struct cdcacm_dev_s *priv;
  FAR struct cdcacm_driver_s *drvr;
  char devname[CDCACM_DEVNAME_SIZE];
  int ret;

  /* Allocate the structures needed */

  alloc = (FAR struct cdcacm_alloc_s *)
    kmm_malloc(sizeof(struct cdcacm_alloc_s));

  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Convenience pointers into the allocated blob */

  priv = &alloc->dev;
  drvr = &alloc->drvr;

  /* Initialize the USB serial driver structure */

  memset(priv, 0, sizeof(struct cdcacm_dev_s));
  sq_init(&priv->txfree);
  sq_init(&priv->rxpending);

  priv->minor               = minor;

  /* Save the caller provided device description (composite only) */

  memcpy(&priv->devinfo, devinfo,
         sizeof(struct usbdev_devinfo_s));

#ifdef CONFIG_CDCACM_IFLOWCONTROL
  /* SerialState */

  priv->serialstate         = (CDCACM_UART_DCD | CDCACM_UART_DSR);
#endif

  /* Fake line status */

  priv->linecoding.baud[0]  = (115200) & 0xff;       /* Baud=115200 */
  priv->linecoding.baud[1]  = (115200 >> 8) & 0xff;
  priv->linecoding.baud[2]  = (115200 >> 16) & 0xff;
  priv->linecoding.baud[3]  = (115200 >> 24) & 0xff;
  priv->linecoding.stop     = CDC_CHFMT_STOP1;       /* One stop bit */
  priv->linecoding.parity   = CDC_PARITY_NONE;       /* No parity */
  priv->linecoding.nbits    = 8;                     /* 8 data bits */

  /* Initialize the serial driver sub-structure */

  /* The initial state is disconnected */

#ifdef CONFIG_SERIAL_REMOVABLE
  priv->serdev.disconnected = true;
#endif
  priv->serdev.recv.size    = CONFIG_CDCACM_RXBUFSIZE;
  priv->serdev.recv.buffer  = priv->rxbuffer;
  priv->serdev.xmit.size    = CONFIG_CDCACM_TXBUFSIZE;
  priv->serdev.xmit.buffer  = priv->txbuffer;
  priv->serdev.ops          = &g_uartops;
  priv->serdev.priv         = priv;

  /* Initialize the USB class driver structure */
#if defined(CONFIG_USBDEV_SUPERSPEED)
  drvr->drvr.speed          = USB_SPEED_SUPER;
#elif defined(CONFIG_USBDEV_DUALSPEED)
  drvr->drvr.speed          = USB_SPEED_HIGH;
#else
  drvr->drvr.speed          = USB_SPEED_FULL;
#endif
  drvr->drvr.ops            = &g_driverops;
  drvr->dev                 = priv;

  /* Register the USB serial console */

#ifdef CONFIG_CDCACM_CONSOLE
  if (minor == 0)
    {
      priv->serdev.isconsole = true;

      ret = uart_register("/dev/console", &priv->serdev);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONSOLEREGISTER),
                   (uint16_t)-ret);
          goto errout_with_class;
        }
    }
#endif

  /* Register the CDC/ACM TTY device */

  snprintf(devname, sizeof(devname), CDCACM_DEVNAME_FORMAT, minor);
  ret = uart_register(devname, &priv->serdev);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UARTREGISTER),
               (uint16_t)-ret);
      goto errout_with_class;
    }

  *classdev = &drvr->drvr;
  return OK;

errout_with_class:
  kmm_free(alloc);
  return ret;
}

/****************************************************************************
 * Name: cdcacm_initialize
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured).
 *
 * Input Parameters:
 *   minor - Device minor number.  E.g., minor 0 would correspond to
 *     /dev/ttyACM0.
 *   handle - An optional opaque reference to the CDC/ACM class object that
 *     may subsequently be used with cdcacm_uninitialize().
 *
 * Returned Value:
 *   Zero (OK) means that the driver was successfully registered.  On any
 *   failure, a negated errno value is returned.
 *
 ****************************************************************************/

#ifndef CONFIG_CDCACM_COMPOSITE
int cdcacm_initialize(int minor, FAR void **handle)
{
  FAR struct usbdevclass_driver_s *drvr = NULL;
  struct usbdev_devinfo_s devinfo;
  int ret;

  memset(&devinfo, 0, sizeof(struct usbdev_devinfo_s));

  /* Interfaces.
   *
   * ifnobase must be provided by board-specific logic
   */

  devinfo.ninterfaces = CDCACM_NINTERFACES; /* Number of interfaces in the configuration */

  /* Strings.
   *
   * strbase must be provided by board-specific logic
   */

  devinfo.nstrings    = CDCACM_NSTRIDS;     /* Number of Strings */

  /* Endpoints.
   *
   * Endpoint numbers must be provided by board-specific logic when
   * CDC/ACM is used in a composite device.
   */

  devinfo.nendpoints  = CDCACM_NUM_EPS;
  devinfo.epno[CDCACM_EP_INTIN_IDX]   = CONFIG_CDCACM_EPINTIN;
  devinfo.epno[CDCACM_EP_BULKIN_IDX]  = CONFIG_CDCACM_EPBULKIN;
  devinfo.epno[CDCACM_EP_BULKOUT_IDX] = CONFIG_CDCACM_EPBULKOUT;

  /* Get an instance of the serial driver class object */

  ret = cdcacm_classobject(minor, &devinfo, &drvr);
  if (ret == OK)
    {
      /* Register the USB serial class driver */

      ret = usbdev_register(drvr);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_DEVREGISTER),
                   (uint16_t)-ret);
        }
    }

  /* Return the driver instance (if any) if the caller has requested it
   * by provided a pointer to the location to return it.
   */

  if (handle)
    {
      *handle = (FAR void *)drvr;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: cdcacm_uninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.  This function is used
 *   internally by the USB composite driver to uninitialize the CDC/ACM
 *   driver.  This same interface is available (with an untyped input
 *   parameter) when the CDC/ACM driver is used standalone.
 *
 * Input Parameters:
 *   There is one parameter, it differs in typing depending upon whether the
 *   CDC/ACM driver is an internal part of a composite device, or a
 *   standalone USB driver:
 *
 *     classdev - The class object returned by cdcacm_classobject()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void cdcacm_uninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  FAR struct cdcacm_driver_s *drvr = (FAR struct cdcacm_driver_s *)classdev;
  FAR struct cdcacm_dev_s    *priv = drvr->dev;
  char devname[CDCACM_DEVNAME_SIZE];
  int ret;

#ifndef CONFIG_CDCACM_COMPOSITE
  usbdev_unregister(&drvr->drvr);
#endif

  /* Un-register the CDC/ACM TTY device */

  snprintf(devname, sizeof(devname), CDCACM_DEVNAME_FORMAT, priv->minor);
  ret = unregister_driver(devname);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UARTUNREGISTER),
               (uint16_t)-ret);
    }
}

/****************************************************************************
 * Name: cdcacm_get_composite_devdesc
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

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_CDCACM_COMPOSITE)
void cdcacm_get_composite_devdesc(struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  /* The callback functions for the CDC/ACM class.
   *
   * classobject() and uninitialize() must be provided by board-specific
   * logic
   */

  dev->mkconfdesc   = cdcacm_mkcfgdesc;
  dev->mkstrdesc    = cdcacm_mkstrdesc;

  dev->nconfigs     = CDCACM_NCONFIGS;           /* Number of configurations supported */
  dev->configid     = CDCACM_CONFIGID;           /* The only supported configuration ID */

  /* Let the construction function calculate the size of config descriptor */

  dev->cfgdescsize  = cdcacm_mkcfgdesc(NULL, NULL, USB_SPEED_UNKNOWN, 0);

  /* Board-specific logic must provide the device minor */

  /* Interfaces.
   *
   * ifnobase must be provided by board-specific logic
   */

  dev->devinfo.ninterfaces = CDCACM_NINTERFACES; /* Number of interfaces in the configuration */

  /* Strings.
   *
   * strbase must be provided by board-specific logic
   */

  dev->devinfo.nstrings    = CDCACM_NSTRIDS;     /* Number of Strings */

  /* Endpoints.
   *
   * Endpoint numbers must be provided by board-specific logic.
   */

  dev->devinfo.nendpoints  = CDCACM_NUM_EPS;
}
#endif
