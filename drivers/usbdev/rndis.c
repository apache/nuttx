/****************************************************************************
 * drivers/usbdev/rndis.c
 *
 *   Copyright (C) 2011-2018 Gregory Nutt. All rights reserved.
 *   Authors: Sakari Kapanen <sakari.m.kapanen@gmail.com>,
 *            Petteri Aimonen <jpa@git.mail.kapsi.fi>
 *
 * References:
 *   [MS-RNDIS]:
 *     Remote Network Driver Interface Specification (RNDIS) Protocol
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

#include <queue.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/rndis.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#include "rndis_std.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define CONFIG_RNDIS_EP0MAXPACKET 64

#ifndef CONFIG_RNDIS_NWRREQS
#  define CONFIG_RNDIS_NWRREQS  (2)
#endif

#define RNDIS_PACKET_HDR_SIZE   (sizeof(struct rndis_packet_msg))
#define CONFIG_RNDIS_BULKIN_REQLEN \
  (CONFIG_NET_ETH_PKTSIZE + CONFIG_NET_GUARDSIZE + RNDIS_PACKET_HDR_SIZE)
#define CONFIG_RNDIS_BULKOUT_REQLEN CONFIG_RNDIS_BULKIN_REQLEN

#define RNDIS_NCONFIGS          (1)
#define RNDIS_CONFIGID          (1)
#define RNDIS_CONFIGIDNONE      (0)
#define RNDIS_NINTERFACES       (2)

#define RNDIS_EPINTIN_ADDR      USB_EPIN(3)
#define RNDIS_EPBULKIN_ADDR     USB_EPIN(1)
#define RNDIS_EPBULKOUT_ADDR    USB_EPOUT(2)
#define RNDIS_NUM_EPS           (3)

#define RNDIS_MANUFACTURERSTRID (1)
#define RNDIS_PRODUCTSTRID      (2)
#define RNDIS_SERIALSTRID       (3)

#define RNDIS_STR_LANGUAGE      (0x0409) /* en-us */

#define RNDIS_MXDESCLEN         (128)
#define RNDIS_MAXSTRLEN         (RNDIS_MXDESCLEN-2)
#define RNDIS_CTRLREQ_LEN       (256)
#define RNDIS_RESP_QUEUE_LEN    (256)

#define RNDIS_BUFFER_SIZE       CONFIG_NET_ETH_PKTSIZE
#define RNDIS_BUFFER_COUNT      4

/* TX poll delay = 1 seconds.
 * CLK_TCK is the number of clock ticks per second
 */

#define RNDIS_WDDELAY           (1*CLK_TCK)

/* Work queue to use for network operations. LPWORK should be used here */

#define ETHWORK                 LPWORK

#ifndef min
#  define min(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef max
#  define max(a,b) ((a)>(b)?(a):(b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container to support a list of requests */

struct rndis_req_s
{
  FAR struct rndis_req_s  *flink;  /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;    /* The contained request */
};

/* This structure describes the internal state of the driver */

struct rndis_dev_s
{
  struct net_driver_s      netdev;       /* Network driver structure */
  struct usbdev_devinfo_s  devinfo;

  FAR struct usbdev_s     *usbdev;       /* usbdev driver pointer */
  FAR struct usbdev_ep_s  *epintin;      /* Interrupt IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkin;     /* Bulk IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkout;    /* Bulk OUT endpoint structure */
  FAR struct usbdev_req_s *ctrlreq;      /* Pointer to preallocated control request */
  FAR struct usbdev_req_s *epintin_req;  /* Pointer to preallocated interrupt in endpoint request */
  FAR struct usbdev_req_s *rdreq;        /* Pointer to Preallocated control endpoint read request */
  struct sq_queue_s reqlist;             /* List of free write request containers */

  /* Preallocated USB request buffers */

  struct rndis_req_s wrreqs[CONFIG_RNDIS_NWRREQS];

  struct work_s rxwork;                  /* Worker for dispatching RX packets */
  struct wdog_s txpoll;                  /* TX poll watchdog */
  struct work_s pollwork;                /* TX poll worker */

  bool registered;                       /* Has netdev_register() been called */

  uint8_t config;                        /* USB Configuration number */
  FAR struct rndis_req_s *net_req;       /* Pointer to request whose buffer is assigned to network */
  FAR struct rndis_req_s *rx_req;        /* Pointer request container that holds RX buffer */
  size_t current_rx_received;            /* Number of bytes of current RX datagram received over USB */
  size_t current_rx_datagram_size;       /* Total number of bytes of the current RX datagram */
  size_t current_rx_datagram_offset;     /* Offset of current RX datagram */
  size_t current_rx_msglen;              /* Length of the entire message to be received */
  bool rdreq_submitted;                  /* Indicates if the read request is submitted */
  bool rx_blocked;                       /* Indicates if we can receive packets on bulk in endpoint */
  bool connected;                        /* Connection status indicator */
  uint32_t rndis_packet_filter;          /* RNDIS packet filter value */
  uint32_t rndis_host_tx_count;          /* TX packet counter */
  uint32_t rndis_host_rx_count;          /* RX packet counter */
  uint8_t host_mac_address[6];           /* Host side MAC address */

  uint8_t response_queue[RNDIS_RESP_QUEUE_LEN];
  size_t response_queue_bytes;   /* Count of bytes waiting in response_queue. */
};

/* The internal version of the class driver */

struct rndis_driver_s
{
  struct usbdevclass_driver_s drvr;
  FAR struct rndis_dev_s      *dev;
};

/* This is what is allocated */

struct rndis_alloc_s
{
  struct rndis_dev_s    dev;
  struct rndis_driver_s drvr;
};

/* RNDIS USB configuration descriptor */

struct rndis_cfgdesc_s
{
#ifndef CONFIG_RNDIS_COMPOSITE
  struct usb_cfgdesc_s cfgdesc;        /* Configuration descriptor */
#endif
  struct usb_iaddesc_s assoc_desc;     /* Interface association descriptor */
  struct usb_ifdesc_s  comm_ifdesc;    /* Communication interface descriptor */
  struct usb_epdesc_s  epintindesc;    /* Interrupt endpoint descriptor */
  struct usb_ifdesc_s  data_ifdesc;    /* Data interface descriptor */
  struct usb_epdesc_s  epbulkindesc;   /* Bulk in interface descriptor */
  struct usb_epdesc_s  epbulkoutdesc;  /* Bulk out interface descriptor */
};

/* RNDIS object ID - value pair structure */

struct rndis_oid_value_s
{
  uint32_t objid;
  uint32_t length;
  uint32_t value;
  FAR const void *data;                /* Data pointer overrides value if non-NULL. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Netdev driver callbacks */

static int rndis_ifup(FAR struct net_driver_s *dev);
static int rndis_ifdown(FAR struct net_driver_s *dev);
static int rndis_txavail(FAR struct net_driver_s *dev);
static int rndis_transmit(FAR struct rndis_dev_s *priv);
static int rndis_txpoll(FAR struct net_driver_s *dev);
static void rndis_polltimer(wdparm_t arg);

/* usbclass callbacks */

static int  usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                           FAR struct usbdev_s *dev,
                           FAR const struct usb_ctrlreq_s *ctrl,
                           FAR uint8_t *dataout, size_t outlen);
static int  usbclass_bind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev);
static void usbclass_unbind(FAR struct usbdevclass_driver_s *driver,
                            FAR struct usbdev_s *dev);
static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev);
static int  usbclass_setconfig(FAR struct rndis_dev_s *priv, uint8_t config);
static void usbclass_resetconfig(FAR struct rndis_dev_s *priv);

/* usbclass helpers  */

static int usbclass_copy_epdesc(int epid, FAR struct usb_epdesc_s *epdesc,
                                FAR struct usbdev_devinfo_s *devinfo,
                                bool hispeed);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB driver operations */

const static struct usbdevclass_driverops_s g_driverops =
{
  &usbclass_bind,
  &usbclass_unbind,
  &usbclass_setup,
  &usbclass_disconnect,
  NULL,
  NULL
};

#ifndef CONFIG_RNDIS_COMPOSITE
static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {LSBYTE(0x0200), MSBYTE(0x0200)},             /* usb */
  0,                                            /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_RNDIS_EP0MAXPACKET,                    /* maxpacketsize */
  { LSBYTE(CONFIG_RNDIS_VENDORID),              /* vendor */
    MSBYTE(CONFIG_RNDIS_VENDORID) },
  { LSBYTE(CONFIG_RNDIS_PRODUCTID),             /* product */
    MSBYTE(CONFIG_RNDIS_PRODUCTID) },
  { LSBYTE(CONFIG_RNDIS_VERSIONNO),             /* device */
    MSBYTE(CONFIG_RNDIS_VERSIONNO) },
  RNDIS_MANUFACTURERSTRID,                      /* imfgr */
  RNDIS_PRODUCTSTRID,                           /* iproduct */
  RNDIS_SERIALSTRID,                            /* serno */
  RNDIS_NCONFIGS                                /* nconfigs */
};
#endif

const static struct rndis_cfgdesc_s g_rndis_cfgdesc =
{
#ifndef CONFIG_RNDIS_COMPOSITE
  {
    .len          = USB_SIZEOF_CFGDESC,
    .type         = USB_DESC_TYPE_CONFIG,
    .totallen     =
    {
      0, 0
    },
    .ninterfaces  = RNDIS_NINTERFACES,
    .cfgvalue     = RNDIS_CONFIGID,
    .icfg         = 0,
    .attr         = USB_CONFIG_ATTR_ONE | USB_CONFIG_ATTR_SELFPOWER,
    .mxpower      = (CONFIG_USBDEV_MAXPOWER + 1) / 2
  },
#endif
  {
    .len          = USB_SIZEOF_IADDESC,
    .type         = USB_DESC_TYPE_INTERFACEASSOCIATION,
    .firstif      = 0,
    .nifs         = RNDIS_NINTERFACES,
    .classid      = 0xef,
    .subclass     = 0x04,
    .protocol     = 0x01,
    .ifunction    = 0
  },
  {
    .len          = USB_SIZEOF_IFDESC,
    .type         = USB_DESC_TYPE_INTERFACE,
    .ifno         = 0,
    .alt          = 0,
    .neps         = 1,
    .classid      = USB_CLASS_CDC,
    .subclass     = CDC_SUBCLASS_ACM,
    .protocol     = CDC_PROTO_VENDOR,
    .iif          = 0
  },
  {
    .len          = USB_SIZEOF_EPDESC,
    .type         = USB_DESC_TYPE_ENDPOINT,
    .addr         = RNDIS_EPINTIN_ADDR,
    .attr         = USB_EP_ATTR_XFER_INT,
    .mxpacketsize =
    {
      LSBYTE(16), MSBYTE(16)
    },
    .interval = 1
  },
  {
    .len          = USB_SIZEOF_IFDESC,
    .type         = USB_DESC_TYPE_INTERFACE,
    .ifno         = 1,
    .alt          = 0,
    .neps         = 2,
    .classid      = USB_CLASS_CDC_DATA,
    .subclass     = 0,
    .protocol     = 0,
    .iif          = 0
  },
  {
    .len          = USB_SIZEOF_EPDESC,
    .type         = USB_DESC_TYPE_ENDPOINT,
    .addr         = RNDIS_EPBULKIN_ADDR,
    .attr         = USB_EP_ATTR_XFER_BULK,
#ifdef CONFIG_USBDEV_DUALSPEED
    .mxpacketsize =
    {
      LSBYTE(512), MSBYTE(512)
    },
    .interval     = 0
#else
    .mxpacketsize =
    {
      LSBYTE(64), MSBYTE(64)
    },
    .interval     = 1
#endif
  },
  {
    .len          = USB_SIZEOF_EPDESC,
    .type         = USB_DESC_TYPE_ENDPOINT,
    .addr         = RNDIS_EPBULKOUT_ADDR,
    .attr         = USB_EP_ATTR_XFER_BULK,
#ifdef CONFIG_USBDEV_DUALSPEED
    .mxpacketsize =
    {
      LSBYTE(512), MSBYTE(512)
    },
    .interval     = 0
#else
    .mxpacketsize =
    {
      LSBYTE(64), MSBYTE(64)
    },
    .interval     = 1
#endif
  }
};

/* Default MAC address given to the host side of the interface. */

static uint8_t g_rndis_default_mac_addr[6] =
{
  0x02, 0x00, 0x00, 0x11, 0x22, 0x33
};

/* These lists give dummy responses to be returned to PC. The values are
 * chosen so that Windows is happy - other operating systems don't really
 * care much.
 */

static const uint32_t g_rndis_supported_oids[] =
{
  RNDIS_OID_GEN_SUPPORTED_LIST,
  RNDIS_OID_GEN_HARDWARE_STATUS,
  RNDIS_OID_GEN_MEDIA_SUPPORTED,
  RNDIS_OID_GEN_MEDIA_IN_USE,
  RNDIS_OID_GEN_MAXIMUM_FRAME_SIZE,
  RNDIS_OID_GEN_LINK_SPEED,
  RNDIS_OID_GEN_TRANSMIT_BLOCK_SIZE,
  RNDIS_OID_GEN_RECEIVE_BLOCK_SIZE,
  RNDIS_OID_GEN_VENDOR_ID,
  RNDIS_OID_GEN_VENDOR_DESCRIPTION,
  RNDIS_OID_GEN_VENDOR_DRIVER_VERSION,
  RNDIS_OID_GEN_CURRENT_PACKET_FILTER,
  RNDIS_OID_GEN_MAXIMUM_TOTAL_SIZE,
  RNDIS_OID_GEN_MEDIA_CONNECT_STATUS,
  RNDIS_OID_GEN_PHYSICAL_MEDIUM,
  RNDIS_OID_GEN_XMIT_OK,
  RNDIS_OID_GEN_RCV_OK,
  RNDIS_OID_GEN_XMIT_ERROR,
  RNDIS_OID_GEN_RCV_ERROR,
  RNDIS_OID_GEN_RCV_NO_BUFFER,
  RNDIS_OID_802_3_PERMANENT_ADDRESS,
  RNDIS_OID_802_3_CURRENT_ADDRESS,
  RNDIS_OID_802_3_MULTICAST_LIST,
  RNDIS_OID_802_3_MAC_OPTIONS,
  RNDIS_OID_802_3_MAXIMUM_LIST_SIZE,
  RNDIS_OID_802_3_RCV_ERROR_ALIGNMENT,
  RNDIS_OID_802_3_XMIT_ONE_COLLISION,
  RNDIS_OID_802_3_XMIT_MORE_COLLISION,
};

static const struct rndis_oid_value_s g_rndis_oid_values[] =
{
  {
    RNDIS_OID_GEN_SUPPORTED_LIST,
    sizeof(g_rndis_supported_oids), 0,
    g_rndis_supported_oids
  },
  {RNDIS_OID_GEN_MAXIMUM_FRAME_SIZE,    4, CONFIG_NET_ETH_PKTSIZE,  NULL},
#ifdef CONFIG_USBDEV_DUALSPEED
  {RNDIS_OID_GEN_LINK_SPEED,            4, 100000,              NULL},
#else
  {RNDIS_OID_GEN_LINK_SPEED,            4, 2000000,             NULL},
#endif
  {RNDIS_OID_GEN_TRANSMIT_BLOCK_SIZE,   4, CONFIG_NET_ETH_PKTSIZE,  NULL},
  {RNDIS_OID_GEN_RECEIVE_BLOCK_SIZE,    4, CONFIG_NET_ETH_PKTSIZE,  NULL},
  {RNDIS_OID_GEN_VENDOR_ID,             4, 0x00ffffff,          NULL},
  {RNDIS_OID_GEN_VENDOR_DESCRIPTION,    6, 0,                   "RNDIS"},
  {RNDIS_OID_GEN_CURRENT_PACKET_FILTER, 4, 0,                   NULL},
  {RNDIS_OID_GEN_MAXIMUM_TOTAL_SIZE,    4, 2048,                NULL},
  {RNDIS_OID_GEN_XMIT_OK,               4, 0,                   NULL},
  {RNDIS_OID_GEN_RCV_OK,                4, 0,                   NULL},
  {RNDIS_OID_802_3_PERMANENT_ADDRESS,   6, 0,                   NULL},
  {RNDIS_OID_802_3_CURRENT_ADDRESS,     6, 0,                   NULL},
  {RNDIS_OID_802_3_MULTICAST_LIST,      4, 0xe0000000,          NULL},
  {RNDIS_OID_802_3_MAXIMUM_LIST_SIZE,   4, 1,                   NULL},
  {0x0,                                 4, 0,                   NULL}, /* Default fallback */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Buffering of data is implemented in the following manner:
 *
 * RNDIS driver holds a number of preallocated bulk IN endpoint write
 * requests along with buffers large enough to hold an Ethernet packet and
 * the corresponding RNDIS header.
 *
 * One of these is always reserved for packet reception - when data arrives
 * on the bulk OUT endpoint, it is copied to the reserved request buffer.
 * When the reception of an Ethernet packet is complete, a worker to process
 * the packet is scheduled and bulk OUT endpoint is set to NAK.
 *
 * The processing worker passes the buffer to the network. When the network
 * is done processing the packet, the buffer might contain data to be sent.
 * If so, the corresponding write request is queued on the bulk IN endpoint.
 * The NAK state on bulk OUT endpoint is cleared to allow new packets to
 * arrive. If there's no data to send, the request is returned to the list of
 * free requests.
 *
 * When a bulk IN write operation is complete, the request is added to the
 * list of free requests.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: rndis_submit_rdreq
 *
 * Description:
 *   Submits the bulk OUT read request. Takes care not to submit the request
 *   when the RX packet buffer is already in use.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returned Value:
 *   The return value of the EP_SUBMIT operation
 *
 ****************************************************************************/

static int rndis_submit_rdreq(FAR struct rndis_dev_s *priv)
{
  irqstate_t flags = enter_critical_section();
  int ret = OK;

  if (!priv->rdreq_submitted && !priv->rx_blocked)
    {
      priv->rdreq->len = priv->epbulkout->maxpacket;
      ret = EP_SUBMIT(priv->epbulkout, priv->rdreq);
      if (ret != OK)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT),
                   (uint16_t)-priv->rdreq->result);
        }
      else
        {
          priv->rdreq_submitted = true;
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: rndis_cancel_rdreq
 *
 * Description:
 *   Cancels the bulk OUT endpoint read request.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 ****************************************************************************/

static void rndis_cancel_rdreq(FAR struct rndis_dev_s *priv)
{
  irqstate_t flags = enter_critical_section();
  if (priv->rdreq_submitted)
    {
      EP_CANCEL(priv->epbulkout, priv->rdreq);
      priv->rdreq_submitted = false;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rndis_block_rx
 *
 * Description:
 *   Blocks reception of further bulk OUT endpoint data.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 ****************************************************************************/

static void rndis_block_rx(FAR struct rndis_dev_s *priv)
{
  irqstate_t flags = enter_critical_section();

  priv->rx_blocked = true;
  rndis_cancel_rdreq(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rndis_unblock_rx
 *
 * Description:
 *   Unblocks reception of bulk OUT endpoint data.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static void rndis_unblock_rx(FAR struct rndis_dev_s *priv)
{
  priv->rx_blocked = false;
}

/****************************************************************************
 * Name: rndis_allocwrreq
 *
 * Description:
 *   Allocates a bulk IN endpoint request from the list of free request
 *   buffers.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returned Value:
 *   NULL if allocation failed; pointer to allocated request if succeeded
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static FAR struct rndis_req_s *rndis_allocwrreq(FAR struct rndis_dev_s *priv)
{
  return (FAR struct rndis_req_s *)sq_remfirst(&priv->reqlist);
}

/****************************************************************************
 * Name: rndis_hasfreereqs
 *
 * Description:
 *   Checks if there are free requests usable for TX data.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returned Value:
 *   true if requests available; false if no requests available
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static bool rndis_hasfreereqs(FAR struct rndis_dev_s *priv)
{
  return sq_count(&priv->reqlist) > 1;
}

/****************************************************************************
 * Name: rndis_freewrreq
 *
 * Description:
 *   Returns a bulk IN endpoint write requests to the list of free requests.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *   req: pointer to the request
 *
 * Assumptions:
 *   Called with interrupts disabled.
 *
 ****************************************************************************/

static void rndis_freewrreq(FAR struct rndis_dev_s *priv,
                            FAR struct rndis_req_s *req)
{
  DEBUGASSERT(req != NULL);
  sq_addlast((FAR sq_entry_t *)req, &priv->reqlist);
  rndis_submit_rdreq(priv);
}

/****************************************************************************
 * Name: rndis_allocnetreq
 *
 * Description:
 *   Allocates a request buffer to be used on the network.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returned Value:
 *   true if succeeded; false if failed
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static bool rndis_allocnetreq(FAR struct rndis_dev_s *priv)
{
  irqstate_t flags = enter_critical_section();
  DEBUGASSERT(priv->net_req == NULL);

  if (!rndis_hasfreereqs(priv))
    {
      leave_critical_section(flags);
      return false;
    }

  priv->net_req = rndis_allocwrreq(priv);
  if (priv->net_req)
    {
      priv->netdev.d_buf = &priv->net_req->req->buf[RNDIS_PACKET_HDR_SIZE];
      priv->netdev.d_len = CONFIG_NET_ETH_PKTSIZE;
    }

  leave_critical_section(flags);
  return priv->net_req != NULL;
}

/****************************************************************************
 * Name: rndis_sendnetreq
 *
 * Description:
 *   Submits the request buffer held by the network.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static void rndis_sendnetreq(FAR struct rndis_dev_s *priv)
{
  irqstate_t flags = enter_critical_section();

  DEBUGASSERT(priv->net_req != NULL);

  priv->net_req->req->priv = priv->net_req;
  EP_SUBMIT(priv->epbulkin, priv->net_req->req);

  priv->net_req            = NULL;
  priv->netdev.d_buf       = NULL;
  priv->netdev.d_len       = 0;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rndis_freenetreq
 *
 * Description:
 *   Frees the request buffer held by the network.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static void rndis_freenetreq(FAR struct rndis_dev_s *priv)
{
  irqstate_t flags = enter_critical_section();

  rndis_freewrreq(priv, priv->net_req);
  priv->net_req      = NULL;
  priv->netdev.d_buf = NULL;
  priv->netdev.d_len = 0;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rndis_allocrxreq
 *
 * Description:
 *   Allocates a buffer for packet reception if there already isn't one.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returned Value:
 *   true if succeeded; false if failed
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static bool rndis_allocrxreq(FAR struct rndis_dev_s *priv)
{
  if (priv->rx_req != NULL)
    {
      return true;
    }

  priv->rx_req = rndis_allocwrreq(priv);
  return priv->rx_req != NULL;
}

/****************************************************************************
 * Name: rndis_giverxreq
 *
 * Description:
 *   Passes the RX packet buffer to the network
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static void rndis_giverxreq(FAR struct rndis_dev_s *priv)
{
  DEBUGASSERT(priv->rx_req != NULL);
  DEBUGASSERT(priv->net_req == NULL);

  priv->net_req      = priv->rx_req;
  priv->netdev.d_buf = &priv->net_req->req->buf[RNDIS_PACKET_HDR_SIZE];
  priv->netdev.d_len = CONFIG_NET_ETH_PKTSIZE;
  priv->rx_req       = NULL;
}

/****************************************************************************
 * Name: rndis_fillrequest
 *
 * Description:
 *   Fills the RNDIS header to the request buffer
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *   req: the request whose buffer we should fill
 *
 * Returned Value:
 *   The total length of the request data
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static uint16_t rndis_fillrequest(FAR struct rndis_dev_s *priv,
                                  FAR struct usbdev_req_s *req)
{
  size_t datalen;

  req->len = 0;

  datalen = min(priv->netdev.d_len,
                CONFIG_RNDIS_BULKIN_REQLEN - RNDIS_PACKET_HDR_SIZE);
  if (datalen > 0)
    {
      /* Send the required headers */

      FAR struct rndis_packet_msg *msg =
        (FAR struct rndis_packet_msg *)req->buf;
      memset(msg, 0, RNDIS_PACKET_HDR_SIZE);

      msg->msgtype    = RNDIS_PACKET_MSG;
      msg->msglen     = RNDIS_PACKET_HDR_SIZE + datalen;
      msg->dataoffset = RNDIS_PACKET_HDR_SIZE - 8;
      msg->datalen    = datalen;

      req->flags      = USBDEV_REQFLAGS_NULLPKT;
      req->len        = datalen + RNDIS_PACKET_HDR_SIZE;
    }

  return req->len;
}

/****************************************************************************
 * Name: rndis_rxdispatch
 *
 * Description:
 *   Processes the received Ethernet packet. Called from work queue.
 *
 * Input Parameters:
 *   arg: pointer to RNDIS device driver structure
 *
 ****************************************************************************/

static void rndis_rxdispatch(FAR void *arg)
{
  FAR struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)arg;
  FAR struct eth_hdr_s *hdr;
  irqstate_t flags;

  net_lock();
  flags = enter_critical_section();
  rndis_giverxreq(priv);
  priv->netdev.d_len = priv->current_rx_datagram_size;
  leave_critical_section(flags);

  hdr = (FAR struct eth_hdr_s *)priv->netdev.d_buf;

  /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
  if (hdr->type == HTONS(ETHTYPE_IP))
    {
      NETDEV_RXIPV4(&priv->netdev);

      /* Handle ARP on input then give the IPv4 packet to the network
       * layer
       */

      arp_ipin(&priv->netdev);
      ipv4_input(&priv->netdev);

      if (priv->netdev.d_len > 0)
        {
          /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
          if (IFF_IS_IPv4(priv->netdev.d_flags))
#endif
            {
              arp_out(&priv->netdev);
            }
#ifdef CONFIG_NET_IPv6
          else
            {
              neighbor_out(&priv->netdev);
            }
#endif

          /* And send the packet */

          rndis_transmit(priv);
        }
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (hdr->type == HTONS(ETHTYPE_IP6))
    {
      NETDEV_RXIPV6(&priv->netdev);

      /* Give the IPv6 packet to the network layer */

      arp_ipin(&priv->netdev);
      ipv6_input(&priv->netdev);

      if (priv->netdev.d_len > 0)
        {
          /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
          if (IFF_IS_IPv4(priv->netdev.d_flags))
            {
              arp_out(&priv->netdev);
            }
          else
#endif
#ifdef CONFIG_NET_IPv6
            {
              neighbor_out(&priv->netdev);
            }
#endif

          /* And send the packet */

          rndis_transmit(priv);
        }
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (hdr->type == htons(ETHTYPE_ARP))
    {
      NETDEV_RXARP(&priv->netdev);

      arp_arpin(&priv->netdev);

      if (priv->netdev.d_len > 0)
        {
          rndis_transmit(priv);
        }
    }
  else
#endif
    {
      uerr("ERROR: Unsupported packet type dropped (%02x)\n",
           htons(hdr->type));
      NETDEV_RXDROPPED(&priv->netdev);
      priv->netdev.d_len = 0;
    }

  priv->current_rx_datagram_size = 0;
  rndis_unblock_rx(priv);

  if (priv->net_req != NULL)
    {
      rndis_freenetreq(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Name: rndis_txpoll
 *
 * Description:
 *   Sends the packet that is stored in the network packet buffer. Called
 *   from work queue by e.g. txavail and txpoll callbacks.
 *
 * Input Parameters:
 *   dev: pointer to network driver structure
 *
 * Assumptions:
 *   Caller holds the network lock
 *
 ****************************************************************************/

static int rndis_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)dev->d_private;
  int ret = OK;

  if (!priv->connected)
    {
      return -EBUSY;
    }

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  ninfo("Poll result: d_len=%d\n", priv->netdev.d_len);
  if (priv->netdev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->netdev.d_flags))
#endif
        {
          arp_out(&priv->netdev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->netdev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&priv->netdev))
        {
          ret = rndis_transmit(priv);
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return ret;
}

/****************************************************************************
 * Name: rndis_transmit
 *
 * Description:
 *   Start hardware transmission.
 *
 ****************************************************************************/

static int rndis_transmit(FAR struct rndis_dev_s *priv)
{
  int ret = OK;

  /* Queue the packet */

  rndis_fillrequest(priv, priv->net_req->req);
  rndis_sendnetreq(priv);

  if (!rndis_allocnetreq(priv))
    {
      ret = -EBUSY;
    }

  return ret;
}

/****************************************************************************
 * Name: rndis_pollworker
 *
 * Description:
 *   Worker function called by txpoll worker.
 *
 ****************************************************************************/

static void rndis_pollworker(FAR void *arg)
{
  FAR struct rndis_dev_s *priv = (struct rndis_dev_s *)arg;

  DEBUGASSERT(priv != NULL);

  net_lock();

  if (rndis_allocnetreq(priv))
    {
      devif_timer(&priv->netdev, RNDIS_WDDELAY, rndis_txpoll);

      if (priv->net_req != NULL)
        {
          rndis_freenetreq(priv);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: rndis_polltimer
 *
 * Description:
 *   Network poll watchdog timer callback
 *
 ****************************************************************************/

static void rndis_polltimer(wdparm_t arg)
{
  FAR struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)arg;
  int ret;

  if (work_available(&priv->pollwork))
    {
      ret = work_queue(ETHWORK, &priv->pollwork, rndis_pollworker,
                       (FAR void *)priv, 0);
      DEBUGASSERT(ret == OK);
      UNUSED(ret);
    }

  /* Setup the watchdog poll timer again */

  wd_start(&priv->txpoll, RNDIS_WDDELAY,
           rndis_polltimer, (wdparm_t)arg);
}

/****************************************************************************
 * Name: rndis_ifup
 *
 * Description:
 *   Network ifup callback
 *
 ****************************************************************************/

static int rndis_ifup(FAR struct net_driver_s *dev)
{
  FAR struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)dev->d_private;

  wd_start(&priv->txpoll, RNDIS_WDDELAY,
           rndis_polltimer, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Name: rndis_ifdown
 *
 * Description:
 *   Network ifdown callback
 *
 ****************************************************************************/

static int rndis_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)dev->d_private;

  wd_cancel(&priv->txpoll);
  return OK;
}

/****************************************************************************
 * Name: rndis_txavail_work
 *
 * Description:
 *   txavail worker function
 *
 ****************************************************************************/

static void rndis_txavail_work(FAR void *arg)
{
  FAR struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)arg;

  net_lock();

  if (rndis_allocnetreq(priv))
    {
      devif_timer(&priv->netdev, 0, rndis_txpoll);
      if (priv->net_req != NULL)
        {
          rndis_freenetreq(priv);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: rndis_txavail
 *
 * Description:
 *   Network txavail callback that's called when there are buffers available
 *   for sending data. May be called from an interrupt, so we must queue a
 *   worker to do the actual processing.
 *
 ****************************************************************************/

static int rndis_txavail(FAR struct net_driver_s *dev)
{
  FAR struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)dev->d_private;

  if (work_available(&priv->pollwork))
    {
      work_queue(ETHWORK, &priv->pollwork, rndis_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: rndis_recvpacket
 *
 * Description:
 *   Handles a USB packet arriving on the data bulk out endpoint.
 *
 * Assumptions:
 *   Called from the USB interrupt handler with interrupts disabled.
 *
 ****************************************************************************/

static inline int rndis_recvpacket(FAR struct rndis_dev_s *priv,
                                   FAR uint8_t *reqbuf, uint16_t reqlen)
{
  if (!rndis_allocrxreq(priv))
    {
      return -ENOMEM;
    }

  if (!priv->connected)
    {
      return -EBUSY;
    }

  if (!priv->current_rx_datagram_size)
    {
      if (reqlen < 16)
        {
          /* Packet too small to contain a message header */
        }
      else
        {
          /* The packet contains a RNDIS packet message header */

          FAR struct rndis_packet_msg *msg =
            (FAR struct rndis_packet_msg *)reqbuf;
          if (msg->msgtype == RNDIS_PACKET_MSG)
            {
              priv->current_rx_received = reqlen;
              priv->current_rx_datagram_size = msg->datalen;
              priv->current_rx_msglen = msg->msglen;

              /* According to RNDIS-over-USB send, if the message length is a
               * multiple of endpoint max packet size, the host must send an
               * additional single-byte zero packet. Take that in account
               * here.
               */

              if (!(priv->current_rx_msglen % priv->epbulkout->maxpacket))
                {
                  priv->current_rx_msglen += 1;
                }

              /* Data offset is defined as an offset from the beginning of
               * the offset field itself
               */

              priv->current_rx_datagram_offset = msg->dataoffset + 8;
              if (priv->current_rx_datagram_offset < reqlen)
                {
                  memcpy(&priv->rx_req->req->buf[RNDIS_PACKET_HDR_SIZE],
                         &reqbuf[priv->current_rx_datagram_offset],
                         reqlen - priv->current_rx_datagram_offset);
                }
            }
          else
            {
              uerr("Unknown RNDIS message type %" PRIu32 "\n", msg->msgtype);
            }
        }
    }
  else
    {
      if (priv->current_rx_received >= priv->current_rx_datagram_offset &&
          priv->current_rx_received <= priv->current_rx_datagram_size +
          priv->current_rx_datagram_offset)
        {
          size_t index = priv->current_rx_received -
                         priv->current_rx_datagram_offset;
          size_t copysize = min(reqlen,
                                priv->current_rx_datagram_size - index);

          /* Check if the received packet exceeds request buffer */

          if ((index + copysize) <= CONFIG_NET_ETH_PKTSIZE)
            {
              memcpy(&priv->rx_req->req->buf[RNDIS_PACKET_HDR_SIZE + index],
                     reqbuf, copysize);
            }
          else
            {
              uerr("The packet exceeds request buffer (reqlen=%d) \n",
                   reqlen);
            }
        }
      priv->current_rx_received += reqlen;
    }

  if (priv->current_rx_received >= priv->current_rx_msglen)
    {
      /* Check for a usable packet length (4 added for the CRC) */

      if (priv->current_rx_datagram_size > (CONFIG_NET_ETH_PKTSIZE + 4) ||
          priv->current_rx_datagram_size <= (ETH_HDRLEN + 4))
        {
          uerr("ERROR: Bad packet size dropped (%d)\n",
               priv->current_rx_datagram_size);
          NETDEV_RXERRORS(&priv->netdev);
          priv->current_rx_datagram_size = 0;
        }
      else
        {
          int ret;

          DEBUGASSERT(work_available(&priv->rxwork));
          ret = work_queue(ETHWORK, &priv->rxwork, rndis_rxdispatch,
                           priv, 0);
          DEBUGASSERT(ret == 0);
          UNUSED(ret);

          rndis_block_rx(priv);
          priv->rndis_host_tx_count++;
          return -EBUSY;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rndis_prepare_response
 *
 * Description:
 *   Passes the RX packet buffer to the network
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Returns:
 *   pointer to response buffer
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static FAR void *
rndis_prepare_response(FAR struct rndis_dev_s *priv, size_t size,
                       FAR struct rndis_command_header *request_hdr)
{
  uint8_t *buf = priv->response_queue + priv->response_queue_bytes;
  FAR struct rndis_response_header *hdr =
    (FAR struct rndis_response_header *)buf;

  if (priv->response_queue_bytes + size > RNDIS_RESP_QUEUE_LEN)
    {
      uerr("RNDIS response queue full, dropping command %08x",
           (unsigned int)request_hdr->msgtype);
      return NULL;
    }

  hdr->msgtype = request_hdr->msgtype | RNDIS_MSG_COMPLETE;
  hdr->msglen  = size;
  hdr->reqid   = request_hdr->reqid;
  hdr->status  = RNDIS_STATUS_SUCCESS;

  return hdr;
}

/****************************************************************************
 * Name: rndis_send_encapsulated_response
 *
 * Description:
 *   Give a notification to the host that there is an encapsulated response
 *   available.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static int rndis_send_encapsulated_response(FAR struct rndis_dev_s *priv,
                                            size_t size)
{
  FAR struct rndis_notification *notif =
    (FAR struct rndis_notification *)priv->epintin_req->buf;

  /* Mark the response as available in the queue */

  priv->response_queue_bytes += size;
  DEBUGASSERT(priv->response_queue_bytes <= RNDIS_RESP_QUEUE_LEN);

  /* Send notification on IRQ endpoint, to tell host to read the data. */

  notif->notification = RNDIS_NOTIFICATION_RESPONSE_AVAILABLE;
  notif->reserved = 0;
  priv->epintin_req->len = sizeof(struct rndis_notification);

  EP_CANCEL(priv->epintin, priv->epintin_req);
  EP_SUBMIT(priv->epintin, priv->epintin_req);

  return OK;
}

/****************************************************************************
 * Name: rndis_handle_control_message
 *
 * Description:
 *   Handle a RNDIS control message.
 *
 * Input Parameters:
 *   priv: pointer to RNDIS device driver structure
 *
 * Assumptions:
 *   Called from critical section
 *
 ****************************************************************************/

static int rndis_handle_control_message(FAR struct rndis_dev_s *priv,
                                        FAR uint8_t *dataout,
                                        uint16_t outlen)
{
  FAR struct rndis_command_header *cmd_hdr =
    (FAR struct rndis_command_header *)dataout;

  switch (cmd_hdr->msgtype)
    {
      case RNDIS_INITIALIZE_MSG:
        {
          FAR struct rndis_initialize_cmplt *resp;
          size_t respsize = sizeof(struct rndis_initialize_cmplt);

          resp = rndis_prepare_response(priv, respsize, cmd_hdr);
          if (!resp)
            {
              return -ENOMEM;
            }

          resp->major      = RNDIS_MAJOR_VERSION;
          resp->minor      = RNDIS_MINOR_VERSION;
          resp->devflags   = RNDIS_DEVICEFLAGS;
          resp->medium     = RNDIS_MEDIUM_802_3;
          resp->pktperxfer = 1;
          resp->xfrsize    = (4 + 44 + 22) + RNDIS_BUFFER_SIZE;
          resp->pktalign   = 2;

          rndis_send_encapsulated_response(priv, respsize);
        }
        break;

      case RNDIS_HALT_MSG:
        {
          priv->response_queue_bytes = 0;
          priv->connected = false;
        }
        break;

      case RNDIS_QUERY_MSG:
        {
          int i;
          size_t max_reply_size = sizeof(struct rndis_query_cmplt) +
                                  sizeof(g_rndis_supported_oids);
          FAR struct rndis_query_cmplt *resp;
          FAR struct rndis_query_msg *req =
            (FAR struct rndis_query_msg *)dataout;

          resp = rndis_prepare_response(priv, max_reply_size, cmd_hdr);
          if (!resp)
            {
              return -ENOMEM;
            }

          resp->hdr.msglen = sizeof(struct rndis_query_cmplt);
          resp->bufoffset  = 0;
          resp->buflen     = 0;
          resp->hdr.status = RNDIS_STATUS_NOT_SUPPORTED;

          for (i = 0;
               i < sizeof(g_rndis_oid_values) /
                   sizeof(g_rndis_oid_values[0]);
               i++)
            {
              bool match = (g_rndis_oid_values[i].objid == req->objid);

              if (!match && g_rndis_oid_values[i].objid == 0)
                {
                  int j;

                  /* Check whether to apply the fallback entry */

                  for (j = 0;
                       j < sizeof(g_rndis_supported_oids) / sizeof(uint32_t);
                       j++)
                    {
                      if (g_rndis_supported_oids[j] == req->objid)
                        {
                          match = true;
                          break;
                        }
                    }
                }

              if (match)
                {
                  resp->hdr.status = RNDIS_STATUS_SUCCESS;
                  resp->bufoffset  = 16;
                  resp->buflen     = g_rndis_oid_values[i].length;

                  if (req->objid == RNDIS_OID_GEN_CURRENT_PACKET_FILTER)
                    {
                      resp->buffer[0] = priv->rndis_packet_filter;
                    }
                  else if (req->objid == RNDIS_OID_GEN_XMIT_OK)
                    {
                      resp->buffer[0] = priv->rndis_host_tx_count;
                    }
                  else if (req->objid == RNDIS_OID_GEN_RCV_OK)
                    {
                      resp->buffer[0] = priv->rndis_host_rx_count;
                    }
                  else if (req->objid == RNDIS_OID_802_3_CURRENT_ADDRESS ||
                           req->objid == RNDIS_OID_802_3_PERMANENT_ADDRESS)
                    {
                      memcpy(resp->buffer, priv->host_mac_address, 6);
                    }
                  else if (g_rndis_oid_values[i].data)
                    {
                      memcpy(resp->buffer, g_rndis_oid_values[i].data,
                             resp->buflen);
                    }
                  else
                    {
                      memcpy(resp->buffer, &g_rndis_oid_values[i].value,
                             resp->buflen);
                    }
                  break;
                }
            }

          uinfo("RNDIS Query RID=%08x OID=%08x LEN=%d DAT=%08x",
                (unsigned)req->hdr.reqid, (unsigned)req->objid,
                (int)resp->buflen, (unsigned)resp->buffer[0]);

          resp->hdr.msglen += resp->buflen;

          rndis_send_encapsulated_response(priv, resp->hdr.msglen);
        }
        break;

      case RNDIS_SET_MSG:
        {
          FAR struct rndis_set_msg *req;
          FAR struct rndis_response_header *resp;
          size_t respsize = sizeof(struct rndis_response_header);

          resp = rndis_prepare_response(priv, respsize, cmd_hdr);
          req  = (FAR struct rndis_set_msg *)dataout;

          if (!resp)
            {
              return -ENOMEM;
            }

          uinfo("RNDIS SET RID=%08x OID=%08x LEN=%d DAT=%08x",
                (unsigned)req->hdr.reqid, (unsigned)req->objid,
                (int)req->buflen, (unsigned)req->buffer[0]);

          if (req->objid == RNDIS_OID_GEN_CURRENT_PACKET_FILTER)
            {
              priv->rndis_packet_filter = req->buffer[0];

              if (req->buffer[0] == 0)
                {
                  priv->connected = false;
                }
              else
                {
                  uinfo("RNDIS is now connected");
                  priv->connected = true;
                }
            }
          else if (req->objid == RNDIS_OID_802_3_MULTICAST_LIST)
            {
              uinfo("RNDIS multicast list ignored");
            }
          else
            {
              uinfo("RNDIS unsupported set %08x", (unsigned)req->objid);
              resp->status = RNDIS_STATUS_NOT_SUPPORTED;
            }

          rndis_send_encapsulated_response(priv, respsize);
        }
        break;

      case RNDIS_RESET_MSG:
        {
          FAR struct rndis_reset_cmplt *resp;
          size_t respsize = sizeof(struct rndis_reset_cmplt);

          priv->response_queue_bytes = 0;
          resp = rndis_prepare_response(priv, respsize, cmd_hdr);

          if (!resp)
            {
              return -ENOMEM;
            }

          resp->addreset  = 0;
          priv->connected = false;
          rndis_send_encapsulated_response(priv, respsize);
        }
        break;

      case RNDIS_KEEPALIVE_MSG:
        {
          FAR struct rndis_response_header *resp;
          size_t respsize = sizeof(struct rndis_response_header);
          resp = rndis_prepare_response(priv, respsize, cmd_hdr);
          if (!resp)
            {
              return -ENOMEM;
            }

          rndis_send_encapsulated_response(priv, respsize);
        }
        break;

      default:
        uwarn("Unsupported RNDIS control message: %" PRIu32 "\n",
              cmd_hdr->msgtype);
    }

  return OK;
}

/****************************************************************************
 * Name: rndis_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.
 *
 ****************************************************************************/

static void rndis_rdcomplete(FAR struct usbdev_ep_s *ep,
                             FAR struct usbdev_req_s *req)
{
  FAR struct rndis_dev_s *priv;
  irqstate_t flags;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !ep->priv || !req)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract references to private data */

  priv = (FAR struct rndis_dev_s *)ep->priv;

  /* Process the received data unless this is some unusual condition */

  ret = OK;

  flags = enter_critical_section();
  priv->rdreq_submitted = false;

  switch (req->result)
    {
    case 0: /* Normal completion */
      ret = rndis_recvpacket(priv, req->buf, req->xfrd);
      DEBUGASSERT(ret != -ENOMEM);
      break;

    case -ESHUTDOWN: /* Disconnection */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
      leave_critical_section(flags);
      return;

    default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED),
               (uint16_t)-req->result);
      break;
    };

  if (ret == OK)
    {
      rndis_submit_rdreq(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rndis_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void rndis_wrcomplete(FAR struct usbdev_ep_s *ep,
                             FAR struct usbdev_req_s *req)
{
  FAR struct rndis_dev_s *priv;
  FAR struct rndis_req_s *reqcontainer;
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

  priv         = (FAR struct rndis_dev_s *)ep->priv;
  reqcontainer = (FAR struct rndis_req_s *)req->priv;

  /* Return the write request to the free list */

  flags = enter_critical_section();
  rndis_freewrreq(priv, reqcontainer);
  if (rndis_hasfreereqs(priv))
    {
      rndis_txavail(&priv->netdev);
    }

  switch (req->result)
    {
    case OK: /* Normal completion */
      priv->rndis_host_rx_count++;
      break;

    case -ESHUTDOWN: /* Disconnection */
      break;

    default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRUNEXPECTED),
               (uint16_t)-req->result);
      break;
    }

  leave_critical_section(flags);
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
  struct rndis_dev_s *priv = (FAR struct rndis_dev_s *)ep->priv;
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT),
               (uint16_t)-req->result);
    }
  else if (req->len > 0 && req->priv == priv->response_queue)
    {
      /* This transfer was from the response queue,
       * subtract remaining byte count.
       */

      req->priv = 0;
      if (req->len >= priv->response_queue_bytes)
      {
        /* Queue now empty */

        priv->response_queue_bytes = 0;
      }
      else
      {
        /* Copy the remaining responses to beginning of buffer. */

        priv->response_queue_bytes -= req->len;
        memcpy(priv->response_queue, priv->response_queue + req->len,
               priv->response_queue_bytes);
      }
    }
}

/****************************************************************************
 * Name: usbclass_ep0incomplete
 *
 * Description:
 *   Handle completion of interrupt IN endpoint operations
 *
 ****************************************************************************/

static void usbclass_epintin_complete(FAR struct usbdev_ep_s *ep,
                                      FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT),
               (uint16_t)-req->result);
    }
}

/****************************************************************************
 * Name: usbclass_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

static void usbclass_freereq(FAR struct usbdev_ep_s *ep,
                             FAR struct usbdev_req_s *req)
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
 * Name: usbclass_allocreq
 *
 * Description:
 *   Allocate a request instance along with its buffer
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
 * Name: usbclass_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

static int usbclass_mkstrdesc(uint8_t id, FAR struct usb_strdesc_s *strdesc)
{
  FAR const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_RNDIS_COMPOSITE
      case 0:
        {
          /* Descriptor 0 is the language id */

          strdesc->len     = 4;
          strdesc->type    = USB_DESC_TYPE_STRING;
          strdesc->data[0] = LSBYTE(RNDIS_STR_LANGUAGE);
          strdesc->data[1] = MSBYTE(RNDIS_STR_LANGUAGE);
          return 4;
        }

      case RNDIS_MANUFACTURERSTRID:
        str = CONFIG_RNDIS_VENDORSTR;
        break;

      case RNDIS_PRODUCTSTRID:
        str = CONFIG_RNDIS_PRODUCTSTR;
        break;

      case RNDIS_SERIALSTRID:
        str = CONFIG_RNDIS_SERIALSTR;
        break;
#endif

      default:
        return -EINVAL;
    }

  /* The string is utf16-le.  The poor man's utf-8 to utf16-le
   * conversion below will only handle 7-bit en-us ascii
   */

  len = strlen(str);
  if (len > (RNDIS_MAXSTRLEN / 2))
    {
      len = (RNDIS_MAXSTRLEN / 2);
    }

  for (i = 0, ndata = 0; i < len; i++, ndata += 2)
    {
      strdesc->data[ndata]     = str[i];
      strdesc->data[ndata + 1] = 0;
    }

  strdesc->len  = ndata + 2;
  strdesc->type = USB_DESC_TYPE_STRING;
  return strdesc->len;
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

    switch (epid)
    {
    case RNDIS_EP_INTIN_IDX:  /* Interrupt IN endpoint */
        {
          epdesc->len  = USB_SIZEOF_EPDESC;         /* Descriptor length */
          epdesc->type = USB_DESC_TYPE_ENDPOINT;    /* Descriptor type */
          epdesc->addr = RNDIS_MKEPINTIN(devinfo);  /* Endpoint address */
          epdesc->attr = RNDIS_EPINTIN_ATTR;        /* Endpoint attributes */

#ifdef CONFIG_USBDEV_DUALSPEED
          if (hispeed)
            {
              /* Maximum packet size (high speed) */

              epdesc->mxpacketsize[0] = LSBYTE(CONFIG_RNDIS_EPINTIN_HSSIZE);
              epdesc->mxpacketsize[1] = MSBYTE(CONFIG_RNDIS_EPINTIN_HSSIZE);
            }
          else
#endif
            {
              /* Maximum packet size (full speed) */

              epdesc->mxpacketsize[0] = LSBYTE(CONFIG_RNDIS_EPINTIN_FSSIZE);
              epdesc->mxpacketsize[1] = MSBYTE(CONFIG_RNDIS_EPINTIN_FSSIZE);
            }

          epdesc->interval = 10;                       /* Interval */
      }
      break;

    case RNDIS_EP_BULKOUT_IDX:  /* Bulk OUT endpoint */
      {
        epdesc->len  = USB_SIZEOF_EPDESC;           /* Descriptor length */
        epdesc->type = USB_DESC_TYPE_ENDPOINT;      /* Descriptor type */
        epdesc->addr = RNDIS_MKEPBULKOUT(devinfo);  /* Endpoint address */
        epdesc->attr = RNDIS_EPOUTBULK_ATTR;        /* Endpoint attributes */

#ifdef CONFIG_USBDEV_DUALSPEED
        if (hispeed)
          {
            /* Maximum packet size (high speed) */

            epdesc->mxpacketsize[0] = LSBYTE(CONFIG_RNDIS_EPBULKOUT_HSSIZE);
            epdesc->mxpacketsize[1] = MSBYTE(CONFIG_RNDIS_EPBULKOUT_HSSIZE);
          }
        else
#endif
          {
            /* Maximum packet size (full speed) */

            epdesc->mxpacketsize[0] = LSBYTE(CONFIG_RNDIS_EPBULKOUT_FSSIZE);
            epdesc->mxpacketsize[1] = MSBYTE(CONFIG_RNDIS_EPBULKOUT_FSSIZE);
          }

        epdesc->interval = 0;                       /* Interval */
      }
      break;

    case RNDIS_EP_BULKIN_IDX:  /* Bulk IN endpoint */
      {
        epdesc->len  = USB_SIZEOF_EPDESC;           /* Descriptor length */
        epdesc->type = USB_DESC_TYPE_ENDPOINT;      /* Descriptor type */
        epdesc->addr = RNDIS_MKEPBULKIN(devinfo);   /* Endpoint address */
        epdesc->attr = RNDIS_EPINBULK_ATTR;         /* Endpoint attributes */

#ifdef CONFIG_USBDEV_DUALSPEED
        if (hispeed)
          {
            /* Maximum packet size (high speed) */

            epdesc->mxpacketsize[0] = LSBYTE(CONFIG_RNDIS_EPBULKIN_HSSIZE);
            epdesc->mxpacketsize[1] = MSBYTE(CONFIG_RNDIS_EPBULKIN_HSSIZE);
          }
        else
#endif
          {
            /* Maximum packet size (full speed) */

            epdesc->mxpacketsize[0] = LSBYTE(CONFIG_RNDIS_EPBULKIN_FSSIZE);
            epdesc->mxpacketsize[1] = MSBYTE(CONFIG_RNDIS_EPBULKIN_FSSIZE);
          }

        epdesc->interval = 0;                       /* Interval */
      }
      break;

    default:
        return 0;
    }

  return sizeof(struct usb_epdesc_s);
}

/****************************************************************************
 * Name: usbclass_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

static int16_t usbclass_mkcfgdesc(FAR uint8_t *buf,
                                  FAR struct usbdev_devinfo_s *devinfo)
{
  FAR struct rndis_cfgdesc_s *dest = (FAR struct rndis_cfgdesc_s *)buf;
  uint16_t totallen;

  /* This is the total length of the configuration (not necessarily the
   * size that we will be sending now).
   */

  totallen = sizeof(g_rndis_cfgdesc);
  memcpy(dest, &g_rndis_cfgdesc, totallen);

#ifndef CONFIG_RNDIS_COMPOSITE
  /* For a stand-alone device, just fill in the total length */

  dest->cfgdesc.totallen[0] = LSBYTE(totallen);
  dest->cfgdesc.totallen[1] = MSBYTE(totallen);
#else
  /* For composite device, apply possible offset to the interface numbers */

  dest->assoc_desc.firstif += devinfo->ifnobase;
  dest->comm_ifdesc.ifno   += devinfo->ifnobase;
  dest->epintindesc.addr    = USB_EPIN(devinfo->epno[RNDIS_EP_INTIN_IDX]);
  dest->data_ifdesc.ifno   += devinfo->ifnobase;
  dest->epbulkindesc.addr   = USB_EPIN(devinfo->epno[RNDIS_EP_BULKIN_IDX]);
  dest->epbulkoutdesc.addr  = USB_EPOUT(devinfo->epno[RNDIS_EP_BULKOUT_IDX]);
#endif

  return totallen;
}

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
  FAR struct rndis_dev_s *priv = ((FAR struct rndis_driver_s *)driver)->dev;
  FAR struct rndis_req_s *reqcontainer;
  irqstate_t flags;
  uint16_t reqlen;
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

  dev->ep0->priv = priv;

  /* Preallocate control request */

  priv->ctrlreq = usbclass_allocreq(dev->ep0, RNDIS_CTRLREQ_LEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      ret = -ENOMEM;
      goto errout;
    }

  priv->ctrlreq->callback = usbclass_ep0incomplete;

  /* Pre-allocate all endpoints... the endpoints will not be functional
   * until the SET CONFIGURATION request is processed in usbclass_setconfig.
   * This is done here because there may be calls to kmm_malloc and the SET
   * CONFIGURATION processing probably occurs within interrupt handling
   * logic where kmm_malloc calls will fail.
   */

  /* Pre-allocate the IN interrupt endpoint */

  priv->epintin = DEV_ALLOCEP(dev,
                    USB_EPIN(priv->devinfo.epno[RNDIS_EP_INTIN_IDX]),
                    true, USB_EP_ATTR_XFER_INT);
  if (!priv->epintin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epintin->priv = priv;

  priv->epintin_req =
    usbclass_allocreq(priv->epintin, sizeof(struct rndis_notification));
  if (priv->epintin_req == NULL)
  {
    usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
    ret = -ENOMEM;
    goto errout;
  }

  priv->epintin_req->callback = usbclass_epintin_complete;

  /* Pre-allocate the IN bulk endpoint */

  priv->epbulkin = DEV_ALLOCEP(dev,
                      USB_EPIN(priv->devinfo.epno[RNDIS_EP_BULKIN_IDX]),
                      true, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Pre-allocate the OUT bulk endpoint */

  priv->epbulkout =
    DEV_ALLOCEP(dev, USB_EPOUT(priv->devinfo.epno[RNDIS_EP_BULKOUT_IDX]),
                false, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkout)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Pre-allocate read requests.  The buffer size is one full packet. */

  reqlen = 64;

  if (CONFIG_RNDIS_BULKOUT_REQLEN > reqlen)
    {
      reqlen = CONFIG_RNDIS_BULKOUT_REQLEN;
    }

  priv->rdreq = usbclass_allocreq(priv->epbulkout, reqlen);
  if (priv->rdreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
      ret = -ENOMEM;
      goto errout;
    }

  priv->rdreq->callback = rndis_rdcomplete;

  /* Pre-allocate write request containers and put in a free list.
   * The buffer size should be larger than a full packet.  Otherwise,
   * we will send a bogus null packet at the end of each packet.
   *
   * Pick the larger of the max packet size and the configured request
   * size.
   */

  reqlen = 64;

  if (CONFIG_RNDIS_BULKIN_REQLEN > reqlen)
    {
      reqlen = CONFIG_RNDIS_BULKIN_REQLEN;
    }

  for (i = 0; i < CONFIG_RNDIS_NWRREQS; i++)
    {
      reqcontainer      = &priv->wrreqs[i];
      reqcontainer->req = usbclass_allocreq(priv->epbulkin, reqlen);

      if (reqcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }

      reqcontainer->req->priv     = reqcontainer;
      reqcontainer->req->callback = rndis_wrcomplete;

      flags = enter_critical_section();
      sq_addlast((FAR sq_entry_t *)reqcontainer, &priv->reqlist);
      leave_critical_section(flags);
    }

  /* Initialize response queue to empty */

  priv->response_queue_bytes = 0;

  /* Report if we are selfpowered */

#ifndef CONFIG_RNDIS_COMPOSITE
#ifdef CONFIG_USBDEV_SELFPOWERED
  DEV_SETSELFPOWERED(dev);
#endif

  /* And pull-up the data line for the soft connect function */

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
  FAR struct rndis_dev_s *priv;
  FAR struct rndis_req_s *reqcontainer;
  irqstate_t flags;

  usbtrace(TRACE_CLASSUNBIND, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Extract reference to private data */

  priv = ((FAR struct rndis_driver_s *)driver)->dev;

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
       * already have been reset.  If not, then calling usbclass_resetconfig
       * should cause the endpoints to immediately terminate all
       * transfers and return the requests to us (with result == -ESHUTDOWN)
       */

      usbclass_resetconfig(priv);
      up_mdelay(50);

      /* Free the interrupt IN endpoint */

      if (priv->epintin)
        {
          DEV_FREEEP(dev, priv->epintin);
          priv->epintin = NULL;
        }

      /* Free the bulk IN endpoint */

      if (priv->epbulkin)
        {
          DEV_FREEEP(dev, priv->epbulkin);
          priv->epbulkin = NULL;
        }

      /* Free the pre-allocated control request */

      if (priv->ctrlreq != NULL)
        {
          usbclass_freereq(dev->ep0, priv->ctrlreq);
          priv->ctrlreq = NULL;
        }

      if (priv->epintin_req != NULL)
        {
          usbclass_freereq(priv->epintin, priv->epintin_req);
          priv->epintin_req = NULL;
        }

      /* Free pre-allocated read requests (which should all have
       * been returned to the free list at this time -- we don't check)
       */

      if (priv->rdreq)
      {
        usbclass_freereq(priv->epbulkout, priv->rdreq);
      }

      /* Free the bulk OUT endpoint */

      if (priv->epbulkout)
        {
          DEV_FREEEP(dev, priv->epbulkout);
          priv->epbulkout = NULL;
        }

      netdev_unregister(&priv->netdev);

      /* Free write requests that are not in use (which should be all
       * of them
       */

      flags = enter_critical_section();
      while (!sq_empty(&priv->reqlist))
        {
          reqcontainer = (struct rndis_req_s *)sq_remfirst(&priv->reqlist);
          if (reqcontainer->req != NULL)
            {
              usbclass_freereq(priv->epbulkin, reqcontainer->req);
            }
        }

      leave_critical_section(flags);
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

static int usbclass_setup(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev,
                          FAR const struct usb_ctrlreq_s *ctrl,
                          FAR uint8_t *dataout, size_t outlen)
{
  FAR struct rndis_dev_s *priv;
  FAR struct usbdev_req_s *ctrlreq;
  uint16_t value;
  uint16_t len;
  int ret = -EOPNOTSUPP;

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev || !dev->ep0 || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = ((FAR struct rndis_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv || !priv->ctrlreq)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return -ENODEV;
    }
#endif
  ctrlreq = priv->ctrlreq;
  ctrlreq->priv = 0;

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  len   = GETUINT16(ctrl->len);

  uinfo("type=%02x req=%02x value=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, len);

  switch (ctrl->type & USB_REQ_TYPE_MASK)
    {
     /***********************************************************************
      * Standard Requests
      ***********************************************************************/

    case USB_REQ_TYPE_STANDARD:
      {
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
#ifndef CONFIG_RNDIS_COMPOSITE
                case USB_DESC_TYPE_DEVICE:
                  {
                    ret = USB_SIZEOF_DEVDESC;
                    memcpy(ctrlreq->buf, &g_devdesc, ret);
                  }
                  break;
#endif

                case USB_DESC_TYPE_CONFIG:
                  {
                    ret = usbclass_mkcfgdesc(ctrlreq->buf, &priv->devinfo);
                  }
                  break;

                case USB_DESC_TYPE_STRING:
                  {
                    /* index == language code. */

                    ret = usbclass_mkstrdesc(ctrl->value[0],
                                  (FAR struct usb_strdesc_s *)ctrlreq->buf);
                  }
                  break;

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
                  ret = usbclass_setconfig(priv, value);
                }
            }
            break;

          case USB_REQ_GETCONFIGURATION:
            {
              if (ctrl->type == USB_DIR_IN)
                {
                  *(FAR uint8_t *)ctrlreq->buf = priv->config;
                  ret = 1;
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

    /* Class requests */

    case USB_REQ_TYPE_CLASS:
      {
        if ((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_INTERFACE)
          {
            if (ctrl->req == RNDIS_SEND_ENCAPSULATED_COMMAND)
              {
                ret = rndis_handle_control_message(priv, dataout, outlen);
              }
            else if (ctrl->req == RNDIS_GET_ENCAPSULATED_RESPONSE)
              {
                if (priv->response_queue_bytes == 0)
                  {
                    /* No reply available is indicated with a single
                     * 0x00 byte.
                     */

                    ret = 1;
                    ctrlreq->buf[0] = 0;
                  }
                else
                  {
                    /* Retrieve a single reply from the response queue to
                     * control request buffer.
                     */

                    FAR struct rndis_response_header *hdr =
                      (struct rndis_response_header *)priv->response_queue;
                    memcpy(ctrlreq->buf, hdr, hdr->msglen);
                    ctrlreq->priv = priv->response_queue;
                    ret = hdr->msglen;
                  }
              }
          }
      }
      break;

    default:
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
      break;
    }

  /* Respond to the setup command if data was returned.  On an error return
   * value (ret < 0), the USB driver will stall.
   */

  if (ret >= 0)
    {
      ctrlreq->len   = min(len, ret);
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
      ret            = EP_SUBMIT(dev->ep0, ctrlreq);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
          usbclass_ep0incomplete(dev->ep0, ctrlreq);
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

static void usbclass_disconnect(FAR struct usbdevclass_driver_s *driver,
                                FAR struct usbdev_s *dev)
{
  FAR struct rndis_dev_s *priv;
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

  priv = ((FAR struct rndis_driver_s *)driver)->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Inform the "upper half" network driver that we have lost the USB
   * connection.
   */

  priv->netdev.d_ifdown(&priv->netdev);

  flags = enter_critical_section();

  /* Reset the configuration */

  usbclass_resetconfig(priv);

  leave_critical_section(flags);

  /* Perform the soft connect function so that we will we can be
   * re-enumerated.
   */

  DEV_CONNECT(dev);
}

/****************************************************************************
 * Name: usbclass_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void usbclass_resetconfig(FAR struct rndis_dev_s *priv)
{
  /* Are we configured? */

  if (priv->config != RNDIS_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      priv->config = RNDIS_CONFIGIDNONE;

      priv->netdev.d_ifdown(&priv->netdev);

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

      EP_DISABLE(priv->epintin);
      EP_DISABLE(priv->epbulkin);
      EP_DISABLE(priv->epbulkout);
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

static int usbclass_setconfig(FAR struct rndis_dev_s *priv, uint8_t config)
{
  struct usb_epdesc_s epdesc;
  bool hispeed = false;
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

  if (config == RNDIS_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

  /* We only accept one configuration */

  if (config != RNDIS_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

  /* Configure the IN interrupt endpoint */

  usbclass_copy_epdesc(RNDIS_EP_INTIN_IDX, &epdesc, &priv->devinfo, hispeed);
  ret = EP_CONFIGURE(priv->epintin, &epdesc, false);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epintin->priv = priv;

  /* Configure the IN bulk endpoint */

  usbclass_copy_epdesc(RNDIS_EP_BULKIN_IDX,
                       &epdesc, &priv->devinfo, hispeed);
  ret = EP_CONFIGURE(priv->epbulkin, &epdesc, false);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Configure the OUT bulk endpoint */

  usbclass_copy_epdesc(RNDIS_EP_BULKOUT_IDX,
                       &epdesc, &priv->devinfo, hispeed);
  ret = EP_CONFIGURE(priv->epbulkout, &epdesc, true);

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Queue read requests in the bulk OUT endpoint */

  priv->rdreq->callback = rndis_rdcomplete;
  ret = rndis_submit_rdreq(priv);
  if (ret != OK)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16_t)-ret);
      goto errout;
    }

  /* We are successfully configured */

  priv->config = config;
  if (priv->netdev.d_ifup(&priv->netdev) == OK)
    {
      priv->netdev.d_flags |= IFF_UP;
    }

  return OK;

errout:
  usbclass_resetconfig(priv);
  return ret;
}

/****************************************************************************
 * Name: usbclass_classobject
 *
 * Description:
 *   Allocate memory for the RNDIS driver class object
 *
 * Returned Value:
 *   0 on success, negative error code on failure.
 *
 ****************************************************************************/

static int usbclass_classobject(int minor,
                                FAR struct usbdev_devinfo_s *devinfo,
                                FAR struct usbdevclass_driver_s **classdev)
{
  FAR struct rndis_alloc_s *alloc;
  FAR struct rndis_dev_s *priv;
  FAR struct rndis_driver_s *drvr;
  int ret;

  /* Allocate the structures needed */

  alloc = kmm_zalloc(sizeof(struct rndis_alloc_s));
  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Convenience pointers into the allocated blob */

  priv = &alloc->dev;
  drvr = &alloc->drvr;
  *classdev = &drvr->drvr;

#ifdef CONFIG_RNDIS_COMPOSITE
  priv->devinfo = *devinfo;
#else
  priv->devinfo.epno[RNDIS_EP_INTIN_IDX] = USB_EPNO(RNDIS_EPINTIN_ADDR);
  priv->devinfo.epno[RNDIS_EP_BULKIN_IDX] = USB_EPNO(RNDIS_EPBULKIN_ADDR);
  priv->devinfo.epno[RNDIS_EP_BULKOUT_IDX] = USB_EPNO(RNDIS_EPBULKOUT_ADDR);
#endif

  /* Initialize the USB ethernet driver structure */

  sq_init(&priv->reqlist);
  memcpy(priv->host_mac_address, g_rndis_default_mac_addr, 6);
  priv->netdev.d_private = priv;
  priv->netdev.d_ifup = &rndis_ifup;
  priv->netdev.d_ifdown = &rndis_ifdown;
  priv->netdev.d_txavail = &rndis_txavail;

  /* MAC address filtering is purposefully left out of this driver. Since
   * in the RNDIS USB scenario there are only two devices in the network
   * (host and us), there shouldn't be any packets received that don't
   * belong to us.
   */

  /* Initialize the USB class driver structure */

  drvr->drvr.speed         = USB_SPEED_FULL;
  drvr->drvr.ops           = &g_driverops;
  drvr->dev                = priv;

  ret = netdev_register(&priv->netdev, NET_LL_ETHERNET);
  if (ret)
    {
      uerr("Failed to register net device");
      return ret;
    }

  drvr->dev->registered = true;

  return OK;
}

static void usbclass_uninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  FAR struct rndis_driver_s *drvr = (FAR struct rndis_driver_s *)classdev;
  FAR struct rndis_alloc_s *alloc = (FAR struct rndis_alloc_s *)drvr->dev;

  if (drvr->dev->registered)
    {
      netdev_unregister(&drvr->dev->netdev);
      drvr->dev->registered = false;
    }
  else
    {
      kmm_free(alloc);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_rndis_initialize
 *
 * Description:
 *   Initialize the RNDIS USB device driver.
 *
 * Input Parameters:
 *   mac_address: pointer to an array of six octets which is the MAC address
 *                of the host side of the interface. May be NULL to use the
 *                default MAC address.
 *
 * Returned Value:
 *   0 on success, -errno on failure
 *
 ****************************************************************************/

#ifndef CONFIG_RNDIS_COMPOSITE
int usbdev_rndis_initialize(FAR const uint8_t *mac_address)
{
  int ret;
  FAR struct usbdevclass_driver_s *classdev;
  FAR struct rndis_driver_s *drvr;

  ret = usbclass_classobject(0, NULL, &classdev);
  if (ret)
    {
      nerr("usbclass_classobject failed: %d\n", ret);
      return ret;
    }

  drvr = (FAR struct rndis_driver_s *)classdev;

  if (mac_address)
    {
      memcpy(drvr->dev->host_mac_address, mac_address, 6);
    }

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
 * Name: usbdev_rndis_set_host_mac_addr
 *
 * Description:
 *   Set host MAC address. Mainly for use with composite devices where
 *   the MAC cannot be given directly to usbdev_rndis_initialize().
 *
 * Input Parameters:
 *   netdev:      pointer to the network interface. Can be obtained from
 *                e.g. netdev_findbyname().
 *
 *   mac_address: pointer to an array of six octets which is the MAC address
 *                of the host side of the interface. May be NULL to use the
 *                default MAC address.
 *
 * Returned Value:
 *   0 on success, -errno on failure
 *
 ****************************************************************************/

int usbdev_rndis_set_host_mac_addr(FAR struct net_driver_s *netdev,
                                   FAR const uint8_t *mac_address)
{
  FAR struct rndis_dev_s *dev = (FAR struct rndis_dev_s *)netdev;

  if (mac_address)
    {
      memcpy(dev->host_mac_address, mac_address, 6);
    }
  else
    {
      memcpy(dev->host_mac_address, g_rndis_default_mac_addr, 6);
    }

  return OK;
}

/****************************************************************************
 * Name: usbdev_rndis_get_composite_devdesc
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

#ifdef CONFIG_RNDIS_COMPOSITE
void usbdev_rndis_get_composite_devdesc(struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  dev->mkconfdesc          = usbclass_mkcfgdesc;
  dev->mkstrdesc           = usbclass_mkstrdesc;
  dev->classobject         = usbclass_classobject;
  dev->uninitialize        = usbclass_uninitialize;
  dev->nconfigs            = RNDIS_NCONFIGS;
  dev->configid            = RNDIS_CONFIGID;
  dev->cfgdescsize         = sizeof(g_rndis_cfgdesc);
  dev->devinfo.ninterfaces = RNDIS_NINTERFACES;
  dev->devinfo.nstrings    = 0;
  dev->devinfo.nendpoints  = RNDIS_NUM_EPS;

  /* Default endpoint indexes, board-specific logic can override these */

  dev->devinfo.epno[RNDIS_EP_INTIN_IDX] = USB_EPNO(RNDIS_EPINTIN_ADDR);
  dev->devinfo.epno[RNDIS_EP_BULKIN_IDX] = USB_EPNO(RNDIS_EPBULKIN_ADDR);
  dev->devinfo.epno[RNDIS_EP_BULKOUT_IDX] = USB_EPNO(RNDIS_EPBULKOUT_ADDR);
}
#endif
