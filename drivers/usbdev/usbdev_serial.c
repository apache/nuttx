/****************************************************************************
 * drivers/usbdev/usbdev_serial.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * This logic emulates the Prolific PL2303 serial/USB converter
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/serial.h>
#include <nuttx/usb.h>
#include <nuttx/usbdev.h>
#include <nuttx/usbdev_trace.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Number of requests in the write queue */

#ifndef CONFIG_USBSER_NWRREQS
#  define CONFIG_USBSER_NWRREQS 4
#endif

/* Number of requests in the read queue */

#ifndef CONFIG_USBSER_NRDREQS
#  define CONFIG_USBSER_NRDREQS 4
#endif

/* Write buffer size */

#ifndef CONFIG_USBSER_WRBUFFERSIZE
#  define CONFIG_USBSER_WRBUFFERSIZE 1024
#endif

/* Logical endpoint numbers / max packet sizes */

#ifndef CONFIG_USBSER_EPIN
#  define CONFIG_USBSER_EPIN 2
#endif

#ifndef CONFIG_USBSER_EPOUT
#  define CONFIG_USBSER_EPOUT 1
#endif

#ifndef CONFIG_USBSER_EP0MAXPACKET
#  define CONFIG_USBSER_EP0MAXPACKET 64
#endif

/* Vendor and product IDs and strings */

#ifndef CONFIG_USBSER_VENDORID
#  define CONFIG_USBSER_VENDORID  0x067b
#endif

#ifndef CONFIG_USBSER_PRODUCTID
#  define CONFIG_USBSER_PRODUCTID 0x2303
#endif

#ifndef CONFIG_USBSER_VENDORSTR
#  warning "No Vendor string specified"
#  define CONFIG_USBSER_VENDORSTR  "NuttX"
#endif

#ifndef CONFIG_USBSER_PRODUCTSTR
#  warning "No Product string specified"
#  define CONFIG_USBSER_PRODUCTSTR "USBdev Serial"
#endif

#undef CONFIG_USBSER_SERIALSTR
#define CONFIG_USBSER_SERIALSTR "0"

#undef CONFIG_USBSER_CONFIGSTR
#define CONFIG_USBSER_CONFIGSTR "Bulk"

/* USB Controller */

#ifndef CONFIG_USBDEV_SELFPOWERED
#  define SELFPOWERED USB_CONFIG_ATT_SELFPOWER
#else
#  define SELFPOWERED (0)
#endif

#ifndef  CONFIG_USBDEV_REMOTEWAKEUP
#  define REMOTEWAKEUP USB_CONFIG_ATTR_WAKEUP
#else
#  define REMOTEWAKEUP (0)
#endif

/* These settings are not modifiable via the NuttX configuration */

#define USBSER_VERSIONNO           (0x0202) /* Device version number */
#define USBSER_CONFIGIDNONE        (0)      /* Config ID means to return to address mode */
#define USBSER_CONFIGID            (1)      /* The only supported configuration ID */
#define USBSER_NCONFIGS            (1)      /* Number of configurations supported */
#define USBSER_INTERFACEID         (0)
#define USBSER_ALTINTERFACEID      (0)
#define USBSER_NINTERFACES         (1)      /* Number of interfaces in the configuration */
#define USBSER_NENDPOINTS          (3)      /* Number of endpoints in the interface  */

/* Endpoint configuration */

#define USBSER_EPINTIN_ADDR        (USB_DIR_IN|1)
#define USBSER_EPINTIN_ATTR        (USB_EP_ATTR_XFER_INT)
#define USBSER_EPINTIN_MXPACKET    (10)
#define USBSER_EPOUTBULK_ADDR      (2)
#define USBSER_EPOUTBULK_ATTR      (USB_EP_ATTR_XFER_BULK)
#define USBSER_EPINBULK_ADDR       (USB_DIR_IN|3)
#define USBSER_EPINBULK_ATTR       (USB_EP_ATTR_XFER_BULK)

/* Vender specific control requests */

#define PL2303_CONTROL_TYPE            0x20
#define PL2303_SETLINEREQUEST          0x20 /* OUT, Recipient interface */
#define PL2303_GETLINEREQUEST          0x21 /* IN, Recipient interface */
#define PL2303_SETCONTROLREQUEST       0x22 /* OUT, Recipient interface */
#define PL2303_BREAKREQUEST            0x23 /* OUT, Recipient interface */

/* Vendor read/write */

#define PL2303_RWREQUEST_TYPE          0x40
#define PL2303_RWREQUEST               0x01 /* IN/OUT, Recipient device */

/* Values *********************************************************************/

/* String language */

#define USBSER_STR_LANGUAGE        0x0409 /* en-us */

/* Descriptor strings */

#define USBSER_MANUFACTURERSTRID    1
#define USBSER_PRODUCTSTRID         2
#define USBSER_SERIALSTRID          3
#define USBSER_CONFIGSTRID          4

/* Buffer big enough for any of our descriptors */

#define USBSER_MXDESCLEN           256

/* min/max macros */

#ifndef min
#  define min(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef max
#  define max(a,b) ((a)>(b)?(a):(b))
#endif

/* Trace values *************************************************************/

#define USBSER_CLASSAPI_SETUP       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SETUP)
#define USBSER_CLASSAPI_SHUTDOWN    TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SHUTDOWN)
#define USBSER_CLASSAPI_ATTACH      TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_ATTACH)
#define USBSER_CLASSAPI_DETACH      TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_DETACH)
#define USBSER_CLASSAPI_IOCTL       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_IOCTL)
#define USBSER_CLASSAPI_RECEIVE     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RECEIVE)
#define USBSER_CLASSAPI_RXINT       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RXINT)
#define USBSER_CLASSAPI_RXAVAILABLE TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RXAVAILABLE)
#define USBSER_CLASSAPI_SEND        TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SEND)
#define USBSER_CLASSAPI_TXINT       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXINT)
#define USBSER_CLASSAPI_TXREADY     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXREADY)
#define USBSER_CLASSAPI_TXEMPTY     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXEMPTY)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container to support a list of requests */

struct usbser_req_s
{
  struct usbser_req_s *flink; /* Implements a singly linked list */
  struct usbdev_req_s *req;
};

/* This structure describes the internal state of the driver */

struct usbser_dev_s
{
  struct uart_dev_s    serdev;    /* Serial device structure */
  struct usbdev_s     *usbdev;    /* usbdev driver pointer */
  ubyte                config;    /* Configuration number */
  ubyte                nwralloc;  /* Number of write requests allocated */
  ubyte                nwrq;      /* Number of queue write requests */
  boolean              open;      /* TRUE: Driver has been opened */
  boolean              wravail;   /* TRUE: write data is buffered */
  ubyte                linest[7]; /* Fake line status */
  struct usbdev_ep_s  *epintin;   /* Address of Interrupt IN endpoint */
  struct usbdev_ep_s  *epbulkin;  /* Address of Bulk IN endpoint */
  struct usbdev_ep_s  *epbulkout; /* Address of Bulk OUT endpoint */
  struct usbdev_req_s *ctrlreq;   /* Control request */
  struct sq_queue_s    reqlist;   /* List of write request containers */

  /* Pre-allocated write requests (linked in reqlist) */

  struct usbser_req_s wrreqs[CONFIG_USBSER_NWRREQS];

  /* Serial I/O buffers */

  char rxbuffer[CONFIG_USBSER_RXBUFSIZE];
  char txbuffer[CONFIG_USBSER_TXBUFSIZE];
};

/* The internal version of the class driver */

struct usbser_driver_s
{
  struct usbdevclass_driver_s drvr;
  struct usbser_dev_s        *dev;
};

/* This is what is allocated */

struct usbser_alloc_s
{
  struct usbser_dev_s    dev;
  struct usbser_driver_s drvr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Transfer helpers *********************************************************/

static uint16  usbclass_fillpacket(FAR struct usbser_dev_s *priv,
                 char *packet, uint16 size);
static int     usbclass_sndpacket(FAR struct usbser_dev_s *priv);
static int     usbclass_recvpacket(FAR struct usbser_dev_s *priv,
                 char *packet, uint16 size);

/* Request helpers *********************************************************/

static struct usbdev_req_s *usbclass_allocreq(FAR struct usbdev_ep_s *ep,
                 uint16 len);
static void    usbclass_freereq(FAR struct usbdev_ep_s *ep,
                 struct usbdev_req_s *req);

/* Configuration ***********************************************************/

static int     usbclass_mkstrdesc(ubyte id, struct usb_strdesc_s *strdesc);
#ifdef CONFIG_USBDEV_DUALSPEED
static void    usbclass_mkepbulkdesc(const struct up_epdesc *indesc,
                 uint16 mxpacket, struct usb_epdesc_s *outdesc)
static sint16  usbclass_mkcfgdesc(ubyte *buf, ubyte speed);
#else
static sint16  usbclass_mkcfgdesc(ubyte *buf);
#endif
static void    usbclass_resetconfig(FAR struct usbser_dev_s *priv);
static int     usbclass_setconfig(FAR struct usbser_dev_s *priv,
                 ubyte config);

/* Completion event handlers ***********************************************/

static void    usbclass_setupcomplete(FAR struct usbdev_ep_s *ep,
                 struct usbdev_req_s *req);
static void    usbclass_rdcomplete(FAR struct usbdev_ep_s *ep,
                 struct usbdev_req_s *req);
static void    usbclass_wrcomplete(FAR struct usbdev_ep_s *ep,
                 struct usbdev_req_s *req);

/* USB class device ********************************************************/

static int     usbclass_bind(FAR struct usbdev_s *dev,
                 FAR struct usbdevclass_driver_s *driver);
static void    usbclass_unbind(FAR struct usbdev_s *dev);
static int     usbclass_setup(FAR struct usbdev_s *dev,
                 const struct usb_ctrlreq_s *ctrl);
static void    usbclass_disconnect(FAR struct usbdev_s *dev);

/* Serial port *************************************************************/

static int     usbser_setup(FAR struct uart_dev_s *dev);
static void    usbser_shutdown(FAR struct uart_dev_s *dev);
static int     usbser_attach(FAR struct uart_dev_s *dev);
static void    usbser_detach(FAR struct uart_dev_s *dev);
static void    usbser_rxint(FAR struct uart_dev_s *dev, boolean enable);
static void    usbser_txint(FAR struct uart_dev_s *dev, boolean enable);
static boolean usbser_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* USB class device ********************************************************/

static const struct usbdevclass_driverops_s g_driverops =
{
  usbclass_bind,        /* bind */
  usbclass_unbind,      /* unbind */
  usbclass_setup,       /* setup */
  usbclass_disconnect,  /* disconnect */
  NULL,                 /* suspend */
  NULL,                 /* resume */
};

/* Serial port *************************************************************/

static const struct uart_ops_s g_uartops =
{
  usbser_setup,         /* setup */
  usbser_shutdown,      /* shutdown */
  usbser_attach,        /* attach */
  usbser_detach,        /* detach */
  NULL,                 /* ioctl */
  NULL,                 /* receive */
  usbser_rxint,         /* rxinit */
  NULL,                 /* rxavailable */
  NULL,                 /* send */
  usbser_txint,         /* txinit */
  NULL,                 /* txready */
  usbser_txempty        /* txempty */
};

/* USB descriptor templates these will be copied and modified */

static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {LSBYTE(0x0200), MSBYTE(0x0200)},             /* usb */
  USB_CLASS_PER_INTERFACE,                      /* class */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_USBSER_EP0MAXPACKET,                   /* maxpacketsize */
  { LSBYTE(CONFIG_USBSER_VENDORID),             /* vendor */
    MSBYTE(CONFIG_USBSER_VENDORID) },
  { LSBYTE(CONFIG_USBSER_PRODUCTID),            /* product */
    MSBYTE(CONFIG_USBSER_PRODUCTID) },
  { LSBYTE(USBSER_VERSIONNO),                   /* device */
    MSBYTE(USBSER_VERSIONNO) },
  USBSER_MANUFACTURERSTRID,                     /* imfgr */
  USBSER_PRODUCTSTRID,                          /* iproduct */
  USBSER_SERIALSTRID,                           /* serno */
  USBSER_NCONFIGS                               /* nconfigs */
};

static const struct usb_cfgdesc_s g_cfgdesc =
{
  USB_SIZEOF_CFGDESC,                           /* len */
  USB_DESC_TYPE_CONFIG,                         /* type */
  {0, 0},                                       /* totallen -- to be provided */
  USBSER_NINTERFACES,                           /* ninterfaces */
  USBSER_CONFIGID,                              /* cfgvalue */
  USBSER_CONFIGSTRID,                           /* icfg */
  USB_CONFIG_ATTR_ONE|SELFPOWERED|REMOTEWAKEUP, /* attr */
  (CONFIG_USBDEV_MAXPOWER + 1) / 2              /* mxpower */
};

static const struct usb_ifdesc_s g_ifdesc =
{
  USB_SIZEOF_IFDESC,                            /* len */
  USB_DESC_TYPE_INTERFACE,                      /* type */
  0,                                            /* ifno */
  0,                                            /* alt */
  USBSER_NENDPOINTS,                            /* neps */
  USB_CLASS_VENDOR_SPEC,                        /* class */
  0,                                            /* subclass */
  0,                                            /* protocol */
  USBSER_CONFIGSTRID                            /* iif */
};

static const struct usb_epdesc_s g_epintindesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  USBSER_EPINTIN_ADDR,                          /* addr */
  USBSER_EPINTIN_ATTR,                          /* attr */
  { LSBYTE(USBSER_EPINTIN_MXPACKET),            /* maxpacket */
    MSBYTE(USBSER_EPINTIN_MXPACKET) },
  1                                             /* interval */
};

static const struct usb_epdesc_s g_epbulkoutdesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  USBSER_EPOUTBULK_ADDR,                        /* addr */
  USBSER_EPOUTBULK_ATTR,                        /* attr */
  { LSBYTE(64), MSBYTE(64) },                   /* maxpacket -- might change to 512*/
  0                                             /* interval */
};

static const struct usb_epdesc_s g_epbulkindesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  USBSER_EPINBULK_ADDR,                         /* addr */
  USBSER_EPINBULK_ATTR,                         /* attr */
  { LSBYTE(64), MSBYTE(64) },                   /* maxpacket -- might change to 512*/
  0                                             /* interval */
};

#ifdef CONFIG_USBDEV_DUALSPEED
static const struct usb_qualdesc_s g_qualdesc =
{
  USB_SIZEOF_QUALDESC,                          /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
  {LSBYTE(0x0200), MSBYTE(0x0200) },            /* USB */
  USB_CLASS_VENDOR_SPEC,                        /* class */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_USBSER_EP0MAXPACKET,                   /* mxpacketsize */
  USBSER_NCONFIGS,                              /* nconfigs */
  0,                                            /* reserved */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: usbclass_fillpacket
 *
 * Description:
 *   If there is data to send, a packet is built in the given buffer.  Called either
 *   to initiate the first write operation, or from the completion interrupt handler
 *   service consecutive write operations.
 *
 * NOTE: The USB serial driver does not use the serial drivers uart_xmitchars()
 *   API.  That logic is essentially duplicated here because unlike UART hardware,
 *   we need to be able to handle writes not byte-by-byte, but packet-by-packet.
 *   Unfortunately, that decision also exposes some internals of the serial driver
 *   in the following.
 *
 ************************************************************************************/

static uint16 usbclass_fillpacket(FAR struct usbser_dev_s *priv, char *packet, uint16 size)
{
  uart_dev_t *serdev = &priv->serdev;
  struct uart_buffer_s *xmit = &serdev->xmit;
  irqstate_t flags;
  uint16 nbytes = 0;

  /* Disable interrupts */

  flags = irqsave();

  /* Transfer bytes while we have bytes available and there is room in the packet */

  while (xmit->head != xmit->tail && nbytes < size)
    {
      *packet++ = xmit->buffer[xmit->tail];
      nbytes++;

      /* Increment the tail pointer */

      if (++(xmit->tail) >= xmit->size)
        {
          xmit->tail = 0;
        }

      /* Check if we have to wake up the serial driver */

      if (serdev->xmitwaiting)
        {
          serdev->xmitwaiting = FALSE;
          sem_post(&serdev->xmitsem);
        }
    }

  /* When all of the characters have been sent from the buffer
   * disable the "TX interrupt".
   */

  if (xmit->head == xmit->tail)
    {
      priv->wravail = FALSE;
      uart_disabletxint(serdev);
    }

  irqrestore(flags);
  return nbytes;
}

/************************************************************************************
 * Name: usbclass_sndpacket
 *
 * Description:
 *   This function obtains write requests, transfers the TX data into the packet,
 *   and submits the packets to the USB controller.  This continues untils either
 *   (1) there are no further packets available, or (2) thre is not further data
 *   to send.
 *
 ************************************************************************************/

static int usbclass_sndpacket(FAR struct usbser_dev_s *priv)
{
  struct usbdev_ep_s *ep;
  struct usbdev_req_s *req;
  struct usbser_req_s *reqcontainer;
  irqstate_t flags;
  int len;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (priv == NULL)
    {
      return -ENODEV;
    }
#endif

  flags = irqsave();

  /* Use our IN endpoint for the transfer */

  ep = priv->epbulkin;

  /* Loop until either (1) we run out or write requests, or (2) usbclass_fillpacket()
   * is unable to fill the packet with data (i.e., untilthere is no more data
   * to be sent).
   */

  while (sq_peek(&priv->reqlist))
    {
      /* Peek at the request in the container at the head of the list */

      reqcontainer = (struct usbser_req_s *)sq_peek(&priv->reqlist);
      req          = reqcontainer->req;

      /* Fill the packet with serial TX data */

      len = usbclass_fillpacket(priv, req->buf, ep->maxpacket);
      if (len > 0)
        {
          /* Then submit the request to the endpoint */

          req->len     = len;
          req->private = reqcontainer;
          ret          = EP_SUBMIT(ep, req);
          if (ret != OK)
            {
              usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL), (uint16)-ret);
              break;
            }

          /* Remove the empty contained from the request list */

          reqcontainer = (struct usbser_req_s *)sq_remfirst(&priv->reqlist);
          priv->nwrq--;
        }
      else
        {
          break;
        }
    }

  irqrestore(flags);
  return ret;
}

/************************************************************************************
 * Name: usbclass_recvpacket
 *
 * Description:
 *   A normal completion event was received by the read completion handler at the
 *   interrupt level (with interrupts disabled).  This function handles the USB packet
 *   and provides the received data to the uart RX buffer.
 *
 ************************************************************************************/

static int usbclass_recvpacket(FAR struct usbser_dev_s *priv, char *packet, uint16 size)
{
  uart_dev_t *serdev = &priv->serdev;
  struct uart_buffer_s *recv = &serdev->recv;
  uint16 nexthead;

  /* Get the next head index */

  nexthead = recv->head + 1;
  if (nexthead >= recv->size)
    {
      nexthead = 0;
    }

  /* Then copy data into the RX buffer until either: (1) all of the data has been
   * copied, or (2) the RX buffer is full.  NOTE:  If the RX buffer becomes full,
   * then we have overrun the serial driver and data will be lost.
   */

  while (nexthead != recv->tail && size > 0)
    {
      /* Copy one byte to the head of the circular RX buffer */

      recv->buffer[recv->head] = *packet++;

      /* Update counts and indices */

      recv->head = nexthead;
      size--;
      if (++nexthead >= recv->size)
        {
           nexthead = 0;
        }

      /* Wake up the serial driver if it is waiting for incoming data */

      if (serdev->recvwaiting)
        {
          serdev->recvwaiting = FALSE;
          sem_post(&serdev->recvsem);
        }
    }
  return OK;
}

/****************************************************************************
 * Name: usbclass_allocreq
 *
 * Description:
 *   Allocate a request instance along with its buffer
 *
 ****************************************************************************/

static struct usbdev_req_s *usbclass_allocreq(FAR struct usbdev_ep_s *ep, uint16 len)
{
  struct usbdev_req_s *req;

  req = EP_ALLOCREQ(ep);
  if (req != NULL)
    {
      req->len = len;
      req->buf = EP_ALLOCBUFFER(ep, len);
      if (!req->buf)
        {
          EP_FREEREQ(ep, req);
          req = NULL;
        }
    }
  return req;
}

/****************************************************************************
 * Name: usbclass_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

static void usbclass_freereq(FAR struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  if (ep != NULL && req != NULL)
    {
      if (req->buf != NULL)
        {
          EP_FREEBUFFER(ep, req->buf);
        }
      EP_FREEREQ(ep, req);
    }
}

/****************************************************************************
 * Name: usbclass_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

static int usbclass_mkstrdesc(ubyte id, struct usb_strdesc_s *strdesc)
{
  const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len     = 4;
        strdesc->type    = USB_DESC_TYPE_STRING;
        strdesc->data[0] = LSBYTE(USBSER_STR_LANGUAGE);
        strdesc->data[1] = MSBYTE(USBSER_STR_LANGUAGE);
        return 4;
      }

    case USBSER_MANUFACTURERSTRID:
      str = CONFIG_USBSER_VENDORSTR;
      break;

    case USBSER_PRODUCTSTRID:
      str = CONFIG_USBSER_PRODUCTSTR;
      break;

    case USBSER_SERIALSTRID:
      str = CONFIG_USBSER_SERIALSTR;
      break;

    case USBSER_CONFIGSTRID:
      str = CONFIG_USBSER_CONFIGSTR;
      break;

    default:
      return -EINVAL;
    }

   /* The string is utf16-le.  The poor man's utf-8 to utf16-le
    * conversion below will only handle 7-bit en-us ascii
    */

   len = strlen(str);
   for (i = 0, ndata = 0; i < len; i++, ndata += 2)
     {
       strdesc->data[ndata]   = str[i];
       strdesc->data[ndata+1] = 0;
     }

   strdesc->len  = ndata+2;
   strdesc->type = USB_DESC_TYPE_STRING;
   return strdesc->len;
}

/****************************************************************************
 * Name: usbclass_mkepbulkdesc
 *
 * Description:
 *   Construct the endpoint descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
static inline void usbclass_mkepbulkdesc(const struct up_epdesc *indesc,
                                         uint16 mxpacket,
                                         struct usb_epdesc_s *outdesc)
{
  /* Copy the canned descriptor */

  memcpy(outdesc, indesc, USB_SIZEOF_EPDESC);

  /* Then add the correct max packet size */

  outdesc->mxpacketsize[0] = LSBYTE(mxpacket);
  outdesc->mxpacketsize[1] = MSBYTE(mxpacket);
}
#endif

/****************************************************************************
 * Name: usbclass_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
static sint16  usbclass_mkcfgdesc(ubyte *buf, ubyte speed)
#else
static sint16  usbclass_mkcfgdesc(ubyte *buf)
#endif
{
  struct usb_cfgdesc_s *cfgdesc = (struct usb_cfgdesc_s*)buf;
#ifdef CONFIG_USBDEV_DUALSPEED
  boolean highspeed = (speed == USB_SPEED_HIGH);
  uint16 bulkmxpacket;
#endif
  uint16 totallen;

  /* This is the total length of the configuration (not necessarily the
   * size that we will be sending now.
   */

  totallen = USB_SIZEOF_CFGDESC + USB_SIZEOF_IFDESC + USBSER_NENDPOINTS * USB_SIZEOF_EPDESC;

  /* Configuration descriptor -- Copy the canned descriptor and fill in the
   * type (we'll also need to update the size below
   */

  memcpy(cfgdesc, &g_cfgdesc, USB_SIZEOF_CFGDESC);
  buf += USB_SIZEOF_CFGDESC;

  /*  Copy the canned interface descriptor */

  memcpy(buf, &g_ifdesc, USB_SIZEOF_IFDESC);
  buf += USB_SIZEOF_IFDESC;

  /* Make the two endpoint configurations.  First, check for switches
   * between high and full speed
   */

#ifdef CONFIG_USBDEV_DUALSPEED
  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      hispeed = !hispeed;
    }
#endif

  memcpy(buf, &g_epintindesc, USB_SIZEOF_EPDESC);
  buf += USB_SIZEOF_EPDESC;

#ifdef CONFIG_USBDEV_DUALSPEED
  if (hispeed)
    {
      bulkmxpacket = 512;
    }
  else
    {
      bulkmxpacket = 64;
    }

  usbclass_mkepbulkdesc(&g_epbulkoutdesc, bulkmxpacket, (struct usb_epdesc_s*)buf);
  buf += USB_SIZEOF_EPDESC;
  usbclass_mkepbulkdesc(&g_epbulkindesc, bulkmxpacket, (struct usb_epdesc_s*)buf);
#else
  memcpy(buf, &g_epbulkoutdesc, USB_SIZEOF_EPDESC);
  buf += USB_SIZEOF_EPDESC;
  memcpy(buf, &g_epbulkindesc, USB_SIZEOF_EPDESC);
#endif

  /* Finally, fill in the total size of the configuration descriptor */

  cfgdesc->totallen[0] = LSBYTE(totallen);
  cfgdesc->totallen[1] = MSBYTE(totallen);
  return totallen;
}

/****************************************************************************
 * Name: usbclass_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void usbclass_resetconfig(FAR struct usbser_dev_s *priv)
{
  FAR struct usbdev_s *dev = priv->usbdev;
  struct usbser_req_s *reqcontainer;
  irqstate_t flags;

  /* Are we configured? */

  if (priv->config != USBSER_CONFIGIDNONE)
    {
      priv->config = USBSER_CONFIGIDNONE;

      /* Free write requests that are not in use */

      flags = irqsave();
      while (!sq_empty(&priv->reqlist))
        {
          reqcontainer = (struct usbser_req_s *)sq_remfirst(&priv->reqlist);
          if (reqcontainer->req != NULL)
            {
              usbclass_freereq(priv->epbulkin, reqcontainer->req);

              priv->nwralloc--; /* Number of write requests allocated */
              priv->nwrq--;     /* Number of write requests queued */
            }
        }
      irqrestore(flags);

      /* Disable and free endpoints.  This should force completion of all pending
       * transfers.
       */

      if (priv->epintin)
        {
          EP_DISABLE(priv->epintin);
          DEV_FREEEP(dev, priv->epintin);
          priv->epintin = NULL;
        }

      if (priv->epbulkin)
        {
          EP_DISABLE(priv->epbulkin);
          DEV_FREEEP(dev, priv->epbulkin);
          priv->epbulkin = NULL;
        }

      if (priv->epbulkout)
        {
          EP_DISABLE(priv->epbulkout);
          DEV_FREEEP(dev, priv->epbulkout);
          priv->epbulkout = NULL;
        }
    }
}

/****************************************************************************
 * Name: usbclass_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int usbclass_setconfig(FAR struct usbser_dev_s *priv, ubyte config)
{
  struct usbdev_s *dev = priv->usbdev;
  struct usbdev_req_s *req;
  struct usbser_req_s *reqcontainer;
#ifdef CONFIG_USBDEV_DUALSPEED
  struct usb_epdesc_s epdesc;
  uint16 bulkmxpacket;
#endif
  irqstate_t flags;
  int i;
  int ret = 0;

#if CONFIG_DEBUG
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
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

  if (config == USBSER_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

  /* We only accept one configuration */

  if (config != USBSER_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

  /* Configure the IN interrupt endpoint */

  priv->epintin = DEV_ALLOCEP(dev, 0, TRUE, USB_EP_ATTR_XFER_INT);
  if (!priv->epintin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  ret = EP_CONFIGURE(priv->epintin, &g_epintindesc);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epintin->private = priv;

  /* Configure the IN bulk endpoint */

  priv->epbulkin = DEV_ALLOCEP(dev, 0, TRUE, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

#ifdef CONFIG_USBDEV_DUALSPEED
  if (dev->speed == USB_SPEED_HIGH)
    {
      bulkmxpacket = 512;
    }
  else
    {
      bulkmxpacket = 64;
    }

  usbclass_mkepbulkdesc(&g_epbulkindesc, bulkmxpacket, &epdesc);
  ret = EP_CONFIGURE(priv->epbulkin, &epdesc);
#else
  ret = EP_CONFIGURE(priv->epbulkin, &g_epbulkindesc);
#endif
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkin->private = priv;

  /* Configure the OUT bulk endpoint */

  priv->epbulkout = DEV_ALLOCEP(dev, 0, FALSE, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkout)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

#ifdef CONFIG_USBDEV_DUALSPEED
  usbclass_mkepbulkdesc(&g_epbulkoutdesc, bulkmxpacket, &epdesc);
  ret = EP_CONFIGURE(priv->epbulkout, &epdesc);
#else
  ret = EP_CONFIGURE(priv->epbulkout, &g_epbulkoutdesc);
#endif
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkout->private = priv;

  /* Allocate and queue read requests */

  for (i = 0; i < CONFIG_USBSER_NRDREQS && ret == 0; i++)
    {
      req = usbclass_allocreq(priv->epbulkout, priv->epbulkout->maxpacket);
      if (ret == 0)
        {
          req->callback = usbclass_rdcomplete;
          ret           = EP_SUBMIT(priv->epbulkout, req);
          if (ret != OK)
            {
              usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16)-ret);
            }
        }
      else
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), (uint16)-ret);
          usbclass_resetconfig(priv);
          return -ENOMEM;
        }
    }

  /* Allocate write request containers and put in a free list */

  for (i = 0; i < CONFIG_USBSER_NWRREQS; i++)
    {
      reqcontainer      = &priv->wrreqs[i];
      reqcontainer->req = usbclass_allocreq(priv->epbulkin, priv->epbulkin->maxpacket);
      if (reqcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRALLOCREQ), (uint16)-ret);
          usbclass_resetconfig(priv);
          return -ENOMEM;
        }
      reqcontainer->req->private  = reqcontainer;
      reqcontainer->req->callback = usbclass_wrcomplete;

      flags = irqsave();
      sq_addlast((sq_entry_t*)reqcontainer, &priv->reqlist);

      priv->nwralloc++; /* Count of write requests allocated */
      priv->nwrq++;     /* Count of write requests available */
      irqrestore(flags);
    }

  priv->config = config;
  return OK;

errout:
  usbclass_resetconfig(priv);
  return ret;
}

/****************************************************************************
 * Name: usbclass_setupcomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void usbclass_setupcomplete(FAR struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT), (uint16)-req->result);
    }
}

/****************************************************************************
 * Name: usbclass_rdcomplete
 *
 * Description:
 *   Handle completion of read request
 *
 ****************************************************************************/

static void usbclass_rdcomplete(FAR struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct usbser_dev_s *priv = (FAR struct usbser_dev_s*)ep->private;
  int ret;

  if (priv != NULL)
    {
      switch (req->result)
        {
        case 0: /* Normal completion */
          usbclass_recvpacket(priv, req->buf, req->xfrd);
          break;

        case -ESHUTDOWN: /* Disconnection */
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
          usbclass_freereq(ep, req);
          return;

        default:
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED), (uint16)-req->result);
          break;
        };

      /* Requeue the read request */

      req->len = ep->maxpacket;
      ret      = EP_SUBMIT(ep, req);
      if (ret != OK)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16)-req->result);
        }
    }
}

/****************************************************************************
 * Name: usbclass_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void usbclass_wrcomplete(FAR struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct usbser_dev_s *priv = (FAR struct usbser_dev_s *)ep->private;
  struct usbser_req_s *reqcontainer = req->private;
  irqstate_t flags;

  if (priv != NULL)
    {
      switch (req->result)
        {
        case 0: /* Normal completion */
          break;

        case -ESHUTDOWN: /* Disconnection */
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRSHUTDOWN), 0);
          usbclass_freereq(ep, req);
          priv->nwralloc--; /* Number of write requests allocated */
          return;

        default:
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRUNEXPECTED), (uint16)-req->result);
          break;
        }

      /* Put write request back into the free list */

      if (reqcontainer == NULL)
        {
          flags = irqsave();
          sq_addlast((sq_entry_t*)reqcontainer, &priv->reqlist);
          priv->nwrq++;
          irqrestore(flags);

          /* And send another packet if: TX output is enabled */

          usbclass_sndpacket(priv);
        }
    }
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

static int usbclass_bind(FAR struct usbdev_s *dev, FAR struct usbdevclass_driver_s *driver)
{
  struct usbser_dev_s *priv = ((struct usbser_driver_s*)driver)->dev;

  usbtrace(TRACE_CLASSBIND, 0);

  /* Preallocate control request */

  priv->ctrlreq = usbclass_allocreq(dev->ep0, USBSER_MXDESCLEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      usbclass_unbind(dev);
      return -ENOMEM;
    }
  priv->ctrlreq->callback = usbclass_setupcomplete;

  /* Bind the structures */

  priv->usbdev      = dev;
  dev->ep0->private = priv;
  return OK;
}

/****************************************************************************
 * Name: usbclass_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void usbclass_unbind(FAR struct usbdev_s *dev)
{
  struct usbser_dev_s *priv;

  usbtrace(TRACE_CLASSUNBIND, 0);

#ifdef CONFIG_DEBUG
  if (!dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif
  priv = (FAR struct usbser_dev_s *)dev->ep0->private;

#ifdef CONFIG_DEBUG
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  if (priv != NULL)
    {
      if (priv->ctrlreq != NULL)
        {
          usbclass_freereq(dev->ep0, priv->ctrlreq);
        }

      /* Clear out all data in the circular buffer */

      priv->serdev.xmit.head = 0;
      priv->serdev.xmit.tail = 0;
    }
}

/****************************************************************************
 * Name: usbclass_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int usbclass_setup(FAR struct usbdev_s *dev, const struct usb_ctrlreq_s *ctrl)
{
  struct usbser_dev_s *priv;
  struct usbdev_req_s *ctrlreq;
  uint16 value;
  uint16 index;
  uint16 len;
  int ret = -EOPNOTSUPP;

#ifdef CONFIG_DEBUG
  if (!dev || !dev->ep0 || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
     }
#endif
  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = (FAR struct usbser_dev_s *)dev->ep0->private;

#ifdef CONFIG_DEBUG
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

  uvdbg("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  switch (ctrl->type & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_STANDARD:
      {
        switch (ctrl->req)
          {
          case USB_REQ_GETDESCRIPTOR:
            {
              /* The value field specifies the descriptor type in the MS byte and the
               * descriptor index in the LS byte (order is little endian)
               */

              switch (ctrl->value[1])
                {
                case USB_DESC_TYPE_DEVICE:
                  {
                    ret = USB_SIZEOF_DEVDESC;
                    memcpy(ctrlreq->buf, &g_devdesc, ret);
                  }
                  break;

#ifdef CONFIG_USBDEV_DUALSPEED
                case USB_DESC_TYPE_DEVICEQUALIFIER:
                  {
                    ret = USB_SIZEOF_QUALDESC;
                    memcpy(ctrlreq->buf, &g_qualdesc, ret);
                  }
                  break;

                case USB_DESC_TYPE_OTHERSPEEDCONFIG:
#endif /* CONFIG_USBDEV_DUALSPEED */

                case USB_DESC_TYPE_CONFIG:
                  {
#ifdef CONFIG_USBDEV_DUALSPEED
                    ret = usbclass_mkcfgdesc(ctrlreq->buf, dev->speed, len);
#else
                    ret = usbclass_mkcfgdesc(ctrlreq->buf);
#endif
                  }
                  break;

                case USB_DESC_TYPE_STRING:
                  {
                    /* index == language code. */

                    ret = usbclass_mkstrdesc(ctrl->value[0], (struct usb_strdesc_s *)ctrlreq->buf);
                  }
                  break;

                default:
                  {
                    usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_GETUNKNOWNDESC), value);
                  }
                  break;
                }
            }
            break;

          case USB_REQ_SETCONFIGURATION:
            {
              if (ctrl->type == 0)
                {
                  ret = usbclass_setconfig(priv, value);
                }
            }
            break;

          case USB_REQ_GETCONFIGURATION:
            {
              if (ctrl->type == USB_DIR_IN)
                {
                  *(ubyte*)ctrlreq->buf = priv->config;
                  ret = 1;
                }
            }
            break;

          case USB_REQ_SETINTERFACE:
            {
              if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE)
                {
                  if (priv->config == USBSER_CONFIGID &&
                      index == USBSER_INTERFACEID &&
                      value == USBSER_ALTINTERFACEID)
                    {
                      usbclass_resetconfig(priv);
                      usbclass_setconfig(priv, priv->config);
                      ret = 0;
                    }
                }
            }
            break;

          case USB_REQ_GETINTERFACE:
            {
              if (ctrl->type == (USB_DIR_IN|USB_REQ_RECIPIENT_INTERFACE) &&
                  priv->config == USBSER_CONFIGIDNONE)
                {
                  if (index != USBSER_INTERFACEID)
                    {
                      ret = -EDOM;
                    }
                  else
                     {
                      *(ubyte*) ctrlreq->buf = USBSER_ALTINTERFACEID;
                      ret = 1;
                    }
                }
             }
             break;

          default:
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ), ctrl->req);
            break;
          }
      }
      break;

    case PL2303_CONTROL_TYPE:
      {
        if ((ctrl->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_INTERFACE)
          {
            switch (ctrl->req)
              {
              case PL2303_SETLINEREQUEST:
                {
                   memcpy(priv->linest, ctrlreq->buf, min(len, 7));
                   ret = 0;
                }
                break;


              case PL2303_GETLINEREQUEST:
                {
                   memcpy(ctrlreq->buf, priv->linest, 7);
                   ret = 7;
                }
                break;

              case PL2303_SETCONTROLREQUEST:
              case PL2303_BREAKREQUEST:
                {
                  ret = 0;
                }
                break;

              default:
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCTRLREQ), ctrl->type);
                break;
              }
          }
      }
      break;

    case PL2303_RWREQUEST_TYPE:
      {
        if ((ctrl->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
          {
            if (ctrl->req == PL2303_RWREQUEST)
              {
                if ((ctrl->type & USB_DIR_IN) != 0)
                  {
                    *(uint32*)ctrlreq->buf = 0xdeadbeef;
                    ret = 4;
                  }
                else
                  {
                     ret = 0;
                  }
              }
            else
              {
                usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDRWREQ), ctrl->type);
              }
          }
      }
      break;

    default:
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
      break;
    }

  /* Respond with data transfer before status phase? */

  if (ret >= 0)
    {
      ctrlreq->len = min(len, ret);
      ret          = EP_SUBMIT(dev->ep0, ctrlreq);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16)-ret);
          ctrlreq->result = OK;
          usbclass_setupcomplete(dev->ep0, ctrlreq);
        }
    }

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

static void usbclass_disconnect(FAR struct usbdev_s *dev)
{
  struct usbser_dev_s *priv;
  irqstate_t flags;

  usbtrace(TRACE_CLASSDISCONNECT, 0);

#ifdef CONFIG_DEBUG
  if (!dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif
  priv = (FAR struct usbser_dev_s *)dev->ep0->private;

#ifdef CONFIG_DEBUG
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  flags = irqsave();
  usbclass_resetconfig(priv);

  /* Clear out all data in the circular buffer */

  priv->serdev.xmit.head = 0;
  priv->serdev.xmit.tail = 0;
  irqrestore(flags);
}

/****************************************************************************
 * Serial Device Methods
 ****************************************************************************/

/****************************************************************************
 * Name: usbser_setup
 *
 * Description:
 *   This method is called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int usbser_setup(FAR struct uart_dev_s *dev)
{
  struct usbser_dev_s *priv = (FAR struct usbser_dev_s*)dev->priv;

  usbtrace(USBSER_CLASSAPI_SETUP, 0);

#if CONFIG_DEBUG
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

  if (priv->config == USBSER_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SETUPNOTCONNECTED), 0);
      return -ENOTCONN;
    }

  priv->open = TRUE;
  return OK;
}

/****************************************************************************
 * Name: usbser_shutdown
 *
 * Description:
 *   This method is called when the serial port is closed.  This operation
 *   is very simple for the USB serial backend because the serial driver
 *   has already assured that the TX data has full drained -- it calls
 *   usbser_txempty() until that function returns TRUE before calling this
 *   function.
 *
 ****************************************************************************/

static void usbser_shutdown(FAR struct uart_dev_s *dev)
{
  struct usbser_dev_s *priv = (FAR struct usbser_dev_s*)dev->priv;
  irqstate_t flags;

  usbtrace(USBSER_CLASSAPI_SHUTDOWN, 0);

#if CONFIG_DEBUG
  if (!priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
       return;
    }
#endif

  flags = irqsave();

#if CONFIG_DEBUG
  if (!priv->open)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALREADYCLOSED), 0);
      irqrestore(flags);
      return;
    }
#endif

  /* Make sure that we are disconnected from the host */

  usbclass_resetconfig(priv);
  priv->open = FALSE;
  irqrestore(flags);
}

/****************************************************************************
 * Name: usbser_attach
 *
 * Description:
 *   Does not apply to the USB serial class device
 *
 ****************************************************************************/

static int usbser_attach(FAR struct uart_dev_s *dev)
{
  usbtrace(USBSER_CLASSAPI_ATTACH, 0);
  return OK;
}

/****************************************************************************
 * Name: usbser_detach
 *
 * Description:
*   Does not apply to the USB serial class device
  *
 ****************************************************************************/

static void usbser_detach(FAR struct uart_dev_s *dev)
{
  usbtrace(USBSER_CLASSAPI_DETACH, 0);
}

/****************************************************************************
 * Name: usbser_rxint
 *
 * Description:
 *   Called by the serial driver to enable or disable RX interrupts.  We, of
 *   course, have no RX interrupts but must behave consistently.  This method
 *   is called under the conditions:
 *
 *   1. With enable==TRUE when the port is opened (just after usbser_setup
 *      and usbser_attach are called called)
 *   2. With enable==FALSE while transferring data from the RX buffer
 *   2. With enable==TRUE while waiting for more incoming data
 *   3. With enable==FALSE when the port is closed (just before usbser_detach
 *      and usbser_shutdown are called).
 *
 ****************************************************************************/

static void usbser_rxint(FAR struct uart_dev_s *dev, boolean enable)
{
  struct usbser_dev_s *priv = (FAR struct usbser_dev_s*)dev->priv;

  usbtrace(USBSER_CLASSAPI_RXINT, (uint16)enable);

#if CONFIG_DEBUG
  if (!priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
       return;
    }
#endif

  /* I think we can simulate this behavior by stalling and/or resuming the
   * OUT endpoint.
   */

  if (enable)
    {
      EP_RESUME(priv->epbulkout);
    }
  else
    {
      EP_STALL(priv->epbulkout);
    }
}

/****************************************************************************
 * Name: usbser_txint
 *
 * Description:
 *   Called by the serial driver to enable or disable TX interrupts.  We, of
 *   course, have no TX interrupts but must behave consistently.  Initially,
 *   TX interrupts are disabled.  This method is called under the conditions:
 *
 *   1. With enable==FALSE while transferring data into the TX buffer
 *   2. With enable==TRUE when data may be taken from the buffer.
 *   3. With enable==FALSE when the TX buffer is empty
 *
 ****************************************************************************/

static void usbser_txint(FAR struct uart_dev_s *dev, boolean enable)
{
  struct usbser_dev_s *priv = (FAR struct usbser_dev_s*)dev->priv;

  usbtrace(USBSER_CLASSAPI_TXINT, (uint16)enable);

#if CONFIG_DEBUG
  if (!priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
       return;
    }
#endif

  /* Save the TX enable/disable state */

  priv->wravail = enable;

  /* If the new state is enabled and if there is data in the XMIT buffer,
   * send the next packet now.
   */

  if (enable && priv->serdev.xmit.head != priv->serdev.xmit.tail)
    {
      usbclass_sndpacket(priv);
    }
}

/****************************************************************************
 * Name: usbser_txempty
 *
 * Description:
 *   Return TRUE when all data has been sent.  This is called from the
 *   serial driver when the driver is closed.  It will call this API
 *   periodically until it reports TRUE.  NOTE that the serial driver takes all
 *   responsibility for flushing TX data through the hardware so we can be
 *   a bit sloppy about that.
 *
 ****************************************************************************/

static boolean usbser_txempty(FAR struct uart_dev_s *dev)
{
  struct usbser_dev_s *priv = (FAR struct usbser_dev_s*)dev->priv;

  usbtrace(USBSER_CLASSAPI_TXEMPTY, 0);

#if CONFIG_DEBUG
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return TRUE;
    }
#endif

  /* When all of the allocated write requests have been returned to the
   * reqlist, then there is no longer any TX data in flight.
   */

  return priv->nwrq >= priv->nwralloc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_serialinitialize
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured).
 *
 ****************************************************************************/

int usbdev_serialinitialize(int minor)
{
  FAR struct usbser_alloc_s *alloc;
  FAR struct usbser_dev_s *priv;
  FAR struct usbser_driver_s *drvr;
  char devname[16];
  int ret;

  /* Allocate the structures needed */

  alloc = (FAR struct usbser_alloc_s*)malloc(sizeof(struct usbser_alloc_s));
  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Convenience pointers into the allocated blob */

  priv = &alloc->dev;
  drvr = &alloc->drvr;

  /* Initialize the USB serial driver structure */

  memset(priv, 0, sizeof(struct usbser_dev_s));
  sq_init(&priv->reqlist);

  /* Fake line status */

  priv->linest[0] = (115200) & 0xff;       /* Baud=115200 */
  priv->linest[1] = (115200 >> 8) & 0xff;
  priv->linest[2] = (115200 >> 16) & 0xff;
  priv->linest[3] = (115200 >> 24) & 0xff;
  priv->linest[4] = 0;                     /* One stop bit */
  priv->linest[5] = 0;                     /* No parity */
  priv->linest[6] = 8;                     /*8 data bits */

  /* Initialize the serial driver sub-structure */

  priv->serdev.recv.size   = CONFIG_USBSER_RXBUFSIZE;
  priv->serdev.recv.buffer = priv->rxbuffer;
  priv->serdev.xmit.size   = CONFIG_USBSER_RXBUFSIZE;
  priv->serdev.xmit.buffer = priv->rxbuffer;
  priv->serdev.ops         = &g_uartops;
  priv->serdev.priv        = priv;

  /* Initialize the USB class driver structure */

#ifdef CONFIG_USBDEV_DUALSPEED
  drvr->drvr.speed         = USB_SPEED_HIGH;
#else
  drvr->drvr.speed         = USB_SPEED_FULL;
#endif
  drvr->drvr.ops           = &g_driverops;
  drvr->dev                = priv;

  /* Register the USB serial class driver */

  ret = usbdev_register(&drvr->drvr);
  if (ret)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_DEVREGISTER), (uint16)-ret);
      goto errout_with_alloc;
    }

  /* Register the USB serial console */

#ifdef CONFIG_USBSER_CONSOLE
  g_usbserialport.isconsole = TRUE;
  ret = uart_register("/dev/console", &pri->serdev);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONSOLEREGISTER), (uint16)-ret);
      goto errout_with_class;
    }
#endif

  /* Register the single port supported by this implementation */

  sprintf(devname, "/dev/ttyUSB%d", minor);
  ret = uart_register(devname, &priv->serdev);
  if (ret)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UARTREGISTER), (uint16)-ret);
      goto errout_with_class;
    }
  return OK;

errout_with_class:
  usbdev_unregister(&drvr->drvr);
errout_with_alloc:
  free(alloc);
  return ret;
}
