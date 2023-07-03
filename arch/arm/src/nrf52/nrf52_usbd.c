/****************************************************************************
 * arch/arm/src/nrf52/nrf52_usbd.c
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

#include <sys/param.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "nrf52_usbd.h"

#include "hardware/nrf52_usbd.h"
#include "hardware/nrf52_power.h"
#include "hardware/nrf52_utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USB trace ****************************************************************/

/* Trace error codes */

#define NRF52_TRACEERR_ALLOCFAIL            0x01
#define NRF52_TRACEERR_BADEPNO              0x02
#define NRF52_TRACEERR_BINDFAILED           0x03
#define NRF52_TRACEERR_DRIVER               0x04
#define NRF52_TRACEERR_DRIVERREGISTERED     0x05
#define NRF52_TRACEERR_INVALIDPARMS         0x06
#define NRF52_TRACEERR_NOEP                 0x07
#define NRF52_TRACEERR_NOTCONFIGURED        0x08
#define NRF52_TRACEERR_EPOUTQEMPTY          0x09
#define NRF52_TRACEERR_EPOUTNULLPACKET      0x0b
#define NRF52_TRACEERR_EP0NOSETUP           0x0c
#define NRF52_TRACEERR_EP0SETUPSTALLED      0x0d
#define NRF52_TRACEERR_DISPATCHSTALL        0x0e
#define NRF52_TRACEERR_BADEPGETSTATUS       0x0f
#define NRF52_TRACEERR_BADDEVGETSTATUS      0x10
#define NRF52_TRACEERR_BADCLEARFEATURE      0x11
#define NRF52_TRACEERR_BADSETFEATURE        0x12
#define NRF52_TRACEERR_INVALIDCTRLREQ       0x17
#define NRF52_TRACEERR_BADGETSTATUS         0x18
#define NRF52_TRACEERR_EPINREQEMPTY         0x19
#define NRF52_TRACEERR_DMABUSY              0x20

/* Trace interrupt codes */

/* USB Interrupt entry/exit */

#define NRF52_TRACEINTID_USB                1

/* On each pass through the loop */

#define NRF52_TRACEINTID_INTPENDING         2

/* First level interrupt decode */

#define NRF52_TRACEINTID_DEVRESET           (10 + 0)
#define NRF52_TRACEINTID_STARTED            (10 + 1)
#define NRF52_TRACEINTID_ENDEPIN            (10 + 2)
#define NRF52_TRACEINTID_ENDISOIN           (10 + 4)
#define NRF52_TRACEINTID_ENDEPOUT           (10 + 5)
#define NRF52_TRACEINTID_ENDISOOUT          (10 + 6)
#define NRF52_TRACEINTID_SOF                (10 + 7)
#define NRF52_TRACEINTID_USBEVENT           (10 + 8)
#define NRF52_TRACEINTID_EP0SETUP           (10 + 9)
#define NRF52_TRACEINTID_EP0DATADONE        (10 + 10)
#define NRF52_TRACEINTID_EPDATA             (10 + 11)

/* USBEVENT second level decode */

#define NRF52_TRACEINTID_ISOOUTCRC          (30 + 0)
#define NRF52_TRACEINTID_SUSPEND            (30 + 1)
#define NRF52_TRACEINTID_RESUME             (30 + 2)
#define NRF52_TRACEINTID_USBWUALLOWED       (30 + 3)
#define NRF52_TRACEINTID_READY              (30 + 4)

/* EPOUT second level decode */

#define NRF52_TRACEINTID_DISPATCH           (50 + 0)
#define NRF52_TRACEINTID_GETSTATUS          (50 + 1)
#define NRF52_TRACEINTID_DEVGETSTATUS       (50 + 2)
#define NRF52_TRACEINTID_IFGETSTATUS        (50 + 3)
#define NRF52_TRACEINTID_CLEARFEATURE       (50 + 4)
#define NRF52_TRACEINTID_SETFEATURE         (50 + 5)
#define NRF52_TRACEINTID_SETADDRESS         (50 + 6)
#define NRF52_TRACEINTID_GETSETDESC         (50 + 7)
#define NRF52_TRACEINTID_GETSETIFCONFIG     (50 + 8)
#define NRF52_TRACEINTID_SYNCHFRAME         (50 + 9)
#define NRF52_TRACEINTID_EPGETSTATUS        (50 + 10)
#define NRF52_TRACEINTID_TESTMODE           (50 + 11)

/* Low level USBD decode */

#define NRF52_TRACEINTID_DMATASK            (70 + 0)
#define NRF52_TRACEINTID_DMAACK             (70 + 1)
#define NRF52_TRACEINTID_EPINSTART          (70 + 2)
#define NRF52_TRACEINTID_EPINSTOP           (70 + 3)
#define NRF52_TRACEINTID_EPOUTSTART         (70 + 4)
#define NRF52_TRACEINTID_EPOUTSTOP          (70 + 5)
#define NRF52_TRACEINTID_EP0RCVOUT          (70 + 6)
#define NRF52_TRACEINTID_EP0STATUS          (70 + 7)

/* Configuration ************************************************************/

#define USBDEV_EP0_MAXSIZE       (64)
#define USBDEV_SETUP_MAXDATASIZE (USBDEV_EP0_MAXSIZE * 4)

#ifdef CONFIG_USBDEV_ISOCHRONOUS
#  error not supported yet
#endif

#ifdef CONFIG_USBDEV_LOWPOWER
#  error not supported yet
#endif

/* Endpoints ****************************************************************/

/* Odd physical endpoint numbers are IN; even are OUT */

#define NRF52_EPPHYIN2LOG(epphy)     ((uint8_t)(epphy)|USB_DIR_IN)
#define NRF52_EPPHYOUT2LOG(epphy)    ((uint8_t)(epphy)|USB_DIR_OUT)

/* Endpoint 0 - control */

#define EP0                   (0)

/* The set of all endpoints available to the class implementation (1-7) */

#define NRF52_EP_AVAILABLE    (0x0fe)       /* All available endpoints */

/* Maximum packet sizes for full speed endpoints */

#define NRF52_MAXPACKET_SIZE  (64)

/* Request queue operations *************************************************/

#define nrf52_rqempty(ep)     ((ep)->head == NULL)
#define nrf52_rqpeek(ep)      ((ep)->head)

/* Interrupts ***************************************************************/

#ifdef CONFIG_USBDEV_SOFINTERRUPT
#  error TODO
#else
#define NRF52_INT_DEFAULT     (USBD_INT_USBRESET |                     \
                               USBD_INT_EP0DATADONE |                  \
                               USBD_INT_USBEVENT |                     \
                               USBD_INT_EP0SETUP |                     \
                               USBD_INT_EPDATA)
#endif

/* Interrupts that signal DMA transfer complete:
 *   ENDEPIN0-7, ENDEPOUT0-7, ENDISOIN, ENDISOOUT
 */

#define NRF52_USBD_DMAIRQ (USBD_INT_ENDEPIN(0) | USBD_INT_ENDEPIN(1)   | \
                           USBD_INT_ENDEPIN(2) | USBD_INT_ENDEPIN(3)   | \
                           USBD_INT_ENDEPIN(4) | USBD_INT_ENDEPIN(5)   | \
                           USBD_INT_ENDEPIN(6) | USBD_INT_ENDEPIN(7)   | \
                           USBD_INT_ENDEPOUT(0) | USBD_INT_ENDEPOUT(1) | \
                           USBD_INT_ENDEPOUT(2) | USBD_INT_ENDEPOUT(3) | \
                           USBD_INT_ENDEPOUT(4) | USBD_INT_ENDEPOUT(5) | \
                           USBD_INT_ENDEPOUT(6) | USBD_INT_ENDEPOUT(7) | \
                           USBD_INT_ENDISOIN | USBD_INT_ENDISOOUT)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Parsed control request */

struct nrf52_ctrlreq_s
{
  uint8_t  type;
  uint8_t  req;
  uint16_t value;
  uint16_t index;
  uint16_t len;
};

/* A container for a request so that the request may be retained in a list */

struct nrf52_req_s
{
  struct usbdev_req_s  req;           /* Standard USB request */
  struct nrf52_req_s  *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct nrf52_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct nrf52_ep_s.
   */

  struct usbdev_ep_s     ep;            /* Standard endpoint structure */

  /* NRF52-specific fields */

  struct nrf52_usbdev_s *dev;          /* Reference to private driver data */
  struct nrf52_req_s    *head;         /* Request list for this endpoint */
  struct nrf52_req_s    *tail;
  uint8_t               *rxbuff;       /* RX buffer */
  uint8_t                epphy;        /* Physical EP address */
  uint8_t                eptype:2;     /* Endpoint type */
  uint8_t                stalled:1;    /* 1: Endpoint is stalled */
  uint8_t                isin:1;       /* 1: IN Endpoint */
};

/* NRF52 USB device */

struct nrf52_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to structnrf52_usbdev_s.
   */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* NRF52-specific fields */

  uint8_t                 stalled:1;     /* 1: Protocol stalled */
  uint8_t                 selfpowered:1; /* 1: Device is self powered */
  uint8_t                 addressed:1;   /* 1: Peripheral address has been set */
  uint8_t                 wakeup:1;      /* 1: Device remote wake-up */
  uint8_t                 ep0indone;     /* 1: EP0 IN transfer complete */
  uint8_t                 ep0outdone;    /* 1: EP0 OUT transfer complete */
  uint8_t                 epavail[2];    /* Bitset of available OUT/IN endpoints */

  /* DMA access control */

  bool                    dmanow;        /* DMA transfer pending */
  uint16_t                dmaepinwait;   /* EP IN waiting for DMA */
  uint16_t                dmaepoutwait;  /* EP OUT waitning for DMA */

  /* E0 SETUP data buffering.
   *
   * ctrl
   *   The 8-byte SETUP request is received on the EP0 OUT endpoint and is
   *   saved.
   *
   * ep0data
   *   For OUT SETUP requests, the SETUP data phase must also complete
   *   before the SETUP command can be processed.
   *
   * ep0datlen
   *   Length of OUT DATA received in ep0data[]
   */

  struct usb_ctrlreq_s   ctrlreq;
  uint8_t                ep0data[USBDEV_SETUP_MAXDATASIZE];
  uint16_t               ep0datlen;
  uint16_t               ep0reqlen;

  /* The endpoint list */

  struct nrf52_ep_s      epin[NRF52_NENDPOINTS];
  struct nrf52_ep_s      epout[NRF52_NENDPOINTS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#define nrf52_getreg(addr)     getreg32(addr)
#define nrf52_putreg(val,addr) putreg32(val,addr)

/* DMA operations ***********************************************************/

static void nrf52_startdma_task(struct nrf52_usbdev_s *priv, uint32_t addr);
static void nrf52_startdma_ack(struct nrf52_usbdev_s *priv);

static void nrf52_epout_start(struct nrf52_usbdev_s *priv, int epno);
static void nrf52_epout_stop(struct nrf52_usbdev_s *priv, int epno);
static void nrf52_epin_start(struct nrf52_usbdev_s *priv, int epno);
static void nrf52_epin_stop(struct nrf52_usbdev_s *priv, int epno);
static void nrf52_ep0rcvout_start(struct nrf52_usbdev_s *priv);
static void nrf52_ep0status_start(struct nrf52_usbdev_s *priv);

/* Request queue operations *************************************************/

static struct nrf52_req_s *nrf52_req_remfirst(struct nrf52_ep_s *privep);
static bool nrf52_req_addlast(struct nrf52_ep_s *privep,
                              struct nrf52_req_s *req);

/* Low level data transfers and request operations **************************/

/* Special endpoint 0 data transfer logic */

static void nrf52_ep0out_stdrequest(struct nrf52_usbdev_s *priv,
                                    struct nrf52_ctrlreq_s *ctrlreq);
static void nrf52_ep0setup(struct nrf52_usbdev_s *priv);

/* IN request handling */

static void nrf52_epin_transfer(struct nrf52_ep_s *privep, uint8_t *buf,
                                int nbytes);
static void nrf52_epin_request(struct nrf52_usbdev_s *priv,
                               struct nrf52_ep_s *privep);

/* OUT request handling */

static void nrf52_epout_allow(struct nrf52_ep_s *privep);
static void nrf52_epout_transfer(struct nrf52_ep_s *privep);

static void nrf52_epout_complete(struct nrf52_ep_s *privep);
static void nrf52_ep0out_receive(struct nrf52_usbdev_s *priv);
static void nrf52_epout_receive(struct nrf52_ep_s *privep);
static void nrf52_epout_handle(struct nrf52_usbdev_s *priv,
                                struct nrf52_ep_s *privep);

/* General request handling */

static void nrf52_req_complete(struct nrf52_ep_s *privep, int16_t result);
static void nrf52_req_cancel(struct nrf52_ep_s *privep, int16_t status);
static int nrf52_req_dispatch(struct nrf52_usbdev_s *priv,
                              const struct usb_ctrlreq_s *ctrl);

/* Interrupt handling *******************************************************/

static struct nrf52_ep_s *
nrf52_ep_findbyaddr(struct nrf52_usbdev_s *priv, uint16_t eplog);
static void nrf52_usbreset(struct nrf52_usbdev_s *priv);

/* Other second level interrupt processing */

static void nrf52_resumeinterrupt(struct nrf52_usbdev_s *priv);
static void nrf52_suspendinterrupt(struct nrf52_usbdev_s *priv);
static void nrf52_eventinterrupt(struct nrf52_usbdev_s *priv);
static void nrf52_ep0setupinterrupt(struct nrf52_usbdev_s *priv);
static void nrf52_ep0datainterrupt(struct nrf52_usbdev_s *priv);
static void nrf52_epdatainterrupt(struct nrf52_usbdev_s *priv);
static void nrf52_endepininterrupt(struct nrf52_usbdev_s *priv,
                                   uint32_t irqnow);
static void nrf52_endepoutinterrupt(struct nrf52_usbdev_s *priv,
                                    uint32_t irqnow);

/* First level interrupt processing */

static int nrf52_usbinterrupt(int irq, void *context, void *arg);

/* Endpoint operations ******************************************************/

/* Endpoint configuration */

static int nrf52_epout_configure(struct nrf52_ep_s *privep, uint8_t eptype,
                                 uint16_t maxpacket);
static int nrf52_epin_configure(struct nrf52_ep_s *privep, uint8_t eptype,
                                uint16_t maxpacket);
static int nrf52_ep_configure(struct usbdev_ep_s *ep,
                              const struct usb_epdesc_s *desc, bool last);
static void nrf52_ep0_configure(struct nrf52_usbdev_s *priv);

/* Endpoint disable */

static void nrf52_epout_disable(struct nrf52_ep_s *privep);
static void nrf52_epin_disable(struct nrf52_ep_s *privep);
static int  nrf52_ep_disable(struct usbdev_ep_s *ep);

/* Endpoint request management */

static struct usbdev_req_s *nrf52_ep_allocreq(struct usbdev_ep_s *ep);
static void nrf52_ep_freereq(struct usbdev_ep_s *ep, struct usbdev_req_s *);

/* Endpoint buffer management */

#ifdef CONFIG_USBDEV_DMA
static void *nrf52_ep_allocbuffer(struct usbdev_ep_s *ep, unsigned bytes);
static void nrf52_ep_freebuffer(struct usbdev_ep_s *ep, void *buf);
#endif

/* Endpoint request submission */

static int nrf52_ep_submit(struct usbdev_ep_s *ep, struct usbdev_req_s *req);

/* Endpoint request cancellation */

static int nrf52_ep_cancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req);

/* Stall handling */

static int nrf52_ep_setstall(struct nrf52_ep_s *privep);
static int nrf52_ep_clrstall(struct nrf52_ep_s *privep);
static int nrf52_ep_stall(struct usbdev_ep_s *ep, bool resume);
static void nrf52_ep0_stall(struct nrf52_usbdev_s *priv);

/* Endpoint allocation */

static struct usbdev_ep_s *
nrf52_ep_alloc(struct usbdev_s *dev, uint8_t epno, bool in, uint8_t eptype);
static void nrf52_ep_free(struct usbdev_s *dev, struct usbdev_ep_s *ep);

/* USB device controller operations *****************************************/

static int nrf52_getframe(struct usbdev_s *dev);
static int nrf52_wakeup(struct usbdev_s *dev);
static int nrf52_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int nrf52_pullup(struct usbdev_s *dev, bool enable);
static void nrf52_setaddress(struct nrf52_usbdev_s *priv, uint16_t address);

/* Initialization ***********************************************************/

static void nrf52_swinitialize(struct nrf52_usbdev_s *priv);
static void nrf52_hwinitialize(struct nrf52_usbdev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct nrf52_usbdev_s g_usbdev;

/* USB endpoint ops */

static const struct usbdev_epops_s g_epops =
{
  .configure   = nrf52_ep_configure,
  .disable     = nrf52_ep_disable,
  .allocreq    = nrf52_ep_allocreq,
  .freereq     = nrf52_ep_freereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = nrf52_ep_allocbuffer,
  .freebuffer  = nrf52_ep_freebuffer,
#endif
  .submit      = nrf52_ep_submit,
  .cancel      = nrf52_ep_cancel,
  .stall       = nrf52_ep_stall,
};

/* USB device ops */

static const struct usbdev_ops_s g_devops =
{
  .allocep     = nrf52_ep_alloc,
  .freeep      = nrf52_ep_free,
  .getframe    = nrf52_getframe,
  .wakeup      = nrf52_wakeup,
  .selfpowered = nrf52_selfpowered,
  .pullup      = nrf52_pullup,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(NRF52_TRACEERR_ALLOCFAIL),
  TRACE_STR(NRF52_TRACEERR_BADEPNO),
  TRACE_STR(NRF52_TRACEERR_BINDFAILED),
  TRACE_STR(NRF52_TRACEERR_DRIVER),
  TRACE_STR(NRF52_TRACEERR_DRIVERREGISTERED),
  TRACE_STR(NRF52_TRACEERR_INVALIDPARMS),
  TRACE_STR(NRF52_TRACEERR_NOEP),
  TRACE_STR(NRF52_TRACEERR_NOTCONFIGURED),
  TRACE_STR(NRF52_TRACEERR_EPOUTQEMPTY),
  TRACE_STR(NRF52_TRACEERR_EPOUTNULLPACKET),
  TRACE_STR(NRF52_TRACEERR_EP0NOSETUP),
  TRACE_STR(NRF52_TRACEERR_EP0SETUPSTALLED),
  TRACE_STR(NRF52_TRACEERR_DISPATCHSTALL),
  TRACE_STR(NRF52_TRACEERR_BADEPGETSTATUS),
  TRACE_STR(NRF52_TRACEERR_BADEPGETSTATUS),
  TRACE_STR(NRF52_TRACEERR_BADDEVGETSTATUS),
  TRACE_STR(NRF52_TRACEERR_BADCLEARFEATURE),
  TRACE_STR(NRF52_TRACEERR_BADSETFEATURE),
  TRACE_STR(NRF52_TRACEERR_INVALIDCTRLREQ),
  TRACE_STR(NRF52_TRACEERR_BADGETSTATUS),
  TRACE_STR(NRF52_TRACEERR_EPINREQEMPTY),
  TRACE_STR_END
};
#endif

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(NRF52_TRACEINTID_USB),
  TRACE_STR(NRF52_TRACEINTID_INTPENDING),

  TRACE_STR(NRF52_TRACEINTID_DEVRESET),
  TRACE_STR(NRF52_TRACEINTID_STARTED),
  TRACE_STR(NRF52_TRACEINTID_ENDEPIN),
  TRACE_STR(NRF52_TRACEINTID_ENDISOIN),
  TRACE_STR(NRF52_TRACEINTID_ENDEPOUT),
  TRACE_STR(NRF52_TRACEINTID_ENDISOOUT),
  TRACE_STR(NRF52_TRACEINTID_SOF),
  TRACE_STR(NRF52_TRACEINTID_USBEVENT),
  TRACE_STR(NRF52_TRACEINTID_EP0SETUP),
  TRACE_STR(NRF52_TRACEINTID_EP0DATADONE),
  TRACE_STR(NRF52_TRACEINTID_EPDATA),

  TRACE_STR(NRF52_TRACEINTID_ISOOUTCRC),
  TRACE_STR(NRF52_TRACEINTID_SUSPEND),
  TRACE_STR(NRF52_TRACEINTID_RESUME),
  TRACE_STR(NRF52_TRACEINTID_USBWUALLOWED),
  TRACE_STR(NRF52_TRACEINTID_READY),

  TRACE_STR(NRF52_TRACEINTID_DISPATCH),
  TRACE_STR(NRF52_TRACEINTID_GETSTATUS),
  TRACE_STR(NRF52_TRACEINTID_DEVGETSTATUS),
  TRACE_STR(NRF52_TRACEINTID_IFGETSTATUS),
  TRACE_STR(NRF52_TRACEINTID_CLEARFEATURE),
  TRACE_STR(NRF52_TRACEINTID_SETFEATURE),
  TRACE_STR(NRF52_TRACEINTID_SETADDRESS),
  TRACE_STR(NRF52_TRACEINTID_GETSETDESC),
  TRACE_STR(NRF52_TRACEINTID_GETSETIFCONFIG),
  TRACE_STR(NRF52_TRACEINTID_SYNCHFRAME),
  TRACE_STR(NRF52_TRACEINTID_EPGETSTATUS),
  TRACE_STR(NRF52_TRACEINTID_TESTMODE),

  TRACE_STR(NRF52_TRACEINTID_DMATASK),
  TRACE_STR(NRF52_TRACEINTID_DMAACK),
  TRACE_STR(NRF52_TRACEINTID_EPINSTART),
  TRACE_STR(NRF52_TRACEINTID_EPINSTOP),
  TRACE_STR(NRF52_TRACEINTID_EPOUTSTART),
  TRACE_STR(NRF52_TRACEINTID_EPOUTSTOP),
  TRACE_STR(NRF52_TRACEINTID_EP0RCVOUT),
  TRACE_STR(NRF52_TRACEINTID_EP0STATUS),

  TRACE_STR_END
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_startdma_task and nrf52_startdma_ack
 *
 * Description:
 *   Errata [199] USBD: USBD cannot receive tasks during DMA
 *
 ****************************************************************************/

static void nrf52_startdma_task(struct nrf52_usbdev_s *priv, uint32_t addr)
{
  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_DMATASK), addr);
  nrf52_putreg(1, addr);
  priv->dmanow = true;
}

static void nrf52_startdma_ack(struct nrf52_usbdev_s *priv)
{
  if (priv->dmanow == true)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_DMAACK), 0);
      priv->dmanow = false;
    }
}

/****************************************************************************
 * Name: nrf52_xxx_start and nrf52_xxx_stop
 *
 * Description:
 *   Start or stop tasks
 *
 ****************************************************************************/

static void nrf52_epout_start(struct nrf52_usbdev_s *priv, int epno)
{
  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EPOUTSTART), epno);
  nrf52_startdma_task(priv, NRF52_USBD_TASKS_STARTEPOUT(epno));
}

static void nrf52_epout_stop(struct nrf52_usbdev_s *priv, int epno)
{
  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EPOUTSTOP), epno);
  nrf52_putreg(0, NRF52_USBD_TASKS_STARTEPOUT(epno));
}

static void nrf52_epin_start(struct nrf52_usbdev_s *priv, int epno)
{
  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EPINSTART), epno);
  nrf52_startdma_task(priv, NRF52_USBD_TASKS_STARTEPIN(epno));
}

static void nrf52_epin_stop(struct nrf52_usbdev_s *priv, int epno)
{
  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EPINSTOP), epno);
  nrf52_putreg(0, NRF52_USBD_TASKS_STARTEPIN(epno));
}

static void nrf52_ep0rcvout_start(struct nrf52_usbdev_s *priv)
{
  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0RCVOUT), 0);
  nrf52_putreg(1, NRF52_USBD_TASKS_EP0RCVOUT);
}

static void nrf52_ep0status_start(struct nrf52_usbdev_s *priv)
{
  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0STATUS), 0);
  nrf52_putreg(1, NRF52_USBD_TASKS_EP0STATUS);
}

/****************************************************************************
 * Name: nrf52_req_remfirst
 *
 * Description:
 *   Remove a request from the head of an endpoint request queue
 *
 ****************************************************************************/

static struct nrf52_req_s *nrf52_req_remfirst(struct nrf52_ep_s *privep)
{
  struct nrf52_req_s *ret = privep->head;

  if (ret)
    {
      privep->head = ret->flink;
      if (!privep->head)
        {
          privep->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_req_addlast
 *
 * Description:
 *   Add a request to the end of an endpoint request queue
 *
 ****************************************************************************/

static bool nrf52_req_addlast(struct nrf52_ep_s *privep,
                              struct nrf52_req_s *req)
{
  bool is_empty = !privep->head;

  req->flink = NULL;
  if (is_empty)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail        = req;
    }

  return is_empty;
}

/****************************************************************************
 * Name: nrf52_ep0out_stdrequest
 *
 * Description:
 *   Handle a stanard request on EP0.  Pick off the things of interest to the
 *   USB device controller driver; pass what is left to the class driver.
 *
 ****************************************************************************/

static void
nrf52_ep0out_stdrequest(struct nrf52_usbdev_s *priv,
                        struct nrf52_ctrlreq_s *ctrlreq)
{
  struct nrf52_ep_s *privep    = NULL;
  uint8_t            recipient = 0;

  /* Handle standard request */

  switch (ctrlreq->req)
    {
    case USB_REQ_GETSTATUS:
      {
        /* type:  device-to-host; recipient = device, interface, endpoint
         * value: 0
         * index: zero interface endpoint
         * len:   2; data = status
         */

        usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_GETSTATUS), 0);
        if (!priv->addressed || ctrlreq->len != 2 ||
            USB_REQ_ISOUT(ctrlreq->type) || ctrlreq->value != 0)
          {
            priv->stalled = true;
          }
        else
          {
            switch (ctrlreq->type & USB_REQ_RECIPIENT_MASK)
              {
              case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EPGETSTATUS), 0);
                  privep = nrf52_ep_findbyaddr(priv, ctrlreq->index);
                  if (!privep)
                    {
                      usbtrace(
                        TRACE_DEVERROR(NRF52_TRACEERR_BADEPGETSTATUS), 0);
                      priv->stalled = true;
                    }
                  break;
                }

              case USB_REQ_RECIPIENT_DEVICE:
              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  if (ctrlreq->index == 0)
                    {
                      usbtrace(TRACE_INTDECODE(
                                 NRF52_TRACEINTID_DEVGETSTATUS), 0);
                    }
                  else
                    {
                      usbtrace(TRACE_DEVERROR(
                               NRF52_TRACEERR_BADDEVGETSTATUS), 0);
                      priv->stalled = true;
                    }
                  break;
                }

              default:
                {
                  usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADGETSTATUS), 0);
                  priv->stalled = true;
                  break;
                }
              }
          }
        break;
      }

    case USB_REQ_CLEARFEATURE:
      {
        /* type:  host-to-device; recipient = device, interface or endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   zero, data = none
         */

        usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_CLEARFEATURE), 0);
        if (priv->addressed != 0 && ctrlreq->len == 0)
          {
            recipient = ctrlreq->type & USB_REQ_RECIPIENT_MASK;
            if (recipient == USB_REQ_RECIPIENT_ENDPOINT &&
                ctrlreq->value == USB_FEATURE_ENDPOINTHALT &&
                (privep = nrf52_ep_findbyaddr(priv, ctrlreq->index)) != NULL)
              {
                nrf52_ep_clrstall(privep);
              }
            else if (recipient == USB_REQ_RECIPIENT_DEVICE &&
                     ctrlreq->value == USB_FEATURE_REMOTEWAKEUP)
              {
                priv->wakeup = false;
              }
            else
              {
                nrf52_req_dispatch(priv, &priv->ctrlreq);
              }
          }
        else
          {
            usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADCLEARFEATURE), 0);
            priv->stalled = true;
          }
        break;
      }

    case USB_REQ_SETFEATURE:
      {
        /* type:  host-to-device; recipient = device, interface, endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   0; data = none
         */

        usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_SETFEATURE), 0);
        if (priv->addressed != 0 && ctrlreq->len == 0)
          {
            recipient = ctrlreq->type & USB_REQ_RECIPIENT_MASK;
            if (recipient == USB_REQ_RECIPIENT_ENDPOINT &&
                ctrlreq->value == USB_FEATURE_ENDPOINTHALT &&
                (privep = nrf52_ep_findbyaddr(priv, ctrlreq->index)) != NULL)
              {
                nrf52_ep_setstall(privep);
              }
            else if (recipient == USB_REQ_RECIPIENT_DEVICE &&
                     ctrlreq->value == USB_FEATURE_REMOTEWAKEUP)
              {
                priv->wakeup = true;
              }
            else if (recipient == USB_REQ_RECIPIENT_DEVICE &&
                     ctrlreq->value == USB_FEATURE_TESTMODE &&
                     ((ctrlreq->index & 0xff) == 0))
              {
                usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_TESTMODE), 0);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADSETFEATURE), 0);
                priv->stalled = true;
              }
          }
        else
          {
            usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADSETFEATURE), 0);
            priv->stalled = true;
          }
        break;
      }

    case USB_REQ_SETADDRESS:
      {
        /* type:  host-to-device; recipient = device
         * value: device address
         * index: 0
         * len:   0; data = none
         */

        usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_SETADDRESS),
                  ctrlreq->value);
        nrf52_setaddress(priv, (uint16_t)priv->ctrlreq.value[0]);

        /* NOTE: The USBD peripheral handles the SetAddress
         * transfer by itself.
         *
         * REVISTI: Documentation states that we shall not handle this
         *          command in the software at all, but if we don't
         *          send EP0STATUS response then enumeration fails.
         */

        break;
      }

    case USB_REQ_GETDESCRIPTOR:
      /* type:  device-to-host; recipient = device
       * value: descriptor type and index
       * index: 0 or language ID;
       * len:   descriptor len; data = descriptor
       */

    case USB_REQ_SETDESCRIPTOR:
      /* type:  host-to-device; recipient = device
       * value: descriptor type and index
       * index: 0 or language ID;
       * len:   descriptor len; data = descriptor
       */

      {
        usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_GETSETDESC),
                 priv->ctrlreq.req);
        nrf52_req_dispatch(priv, &priv->ctrlreq);
        break;
      }

    case USB_REQ_GETCONFIGURATION:
      /* type:  device-to-host; recipient = device
       * value: 0;
       * index: 0;
       * len:   1; data = configuration value
       */

    case USB_REQ_SETCONFIGURATION:
      /* type:  host-to-device; recipient = device
       * value: configuration value
       * index: 0;
       * len:   0; data = none
       */

    case USB_REQ_GETINTERFACE:
      /* type:  device-to-host; recipient = interface
       * value: 0
       * index: interface;
       * len:   1; data = alt interface
       */

    case USB_REQ_SETINTERFACE:
      /* type:  host-to-device; recipient = interface
       * value: alternate setting
       * index: interface;
       * len:   0; data = none
       */

      {
        usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_GETSETIFCONFIG),
                 priv->ctrlreq.req);
        nrf52_req_dispatch(priv, &priv->ctrlreq);
        break;
      }

    case USB_REQ_SYNCHFRAME:
      /* type:  device-to-host; recipient = endpoint
       * value: 0
       * index: endpoint;
       * len:   2; data = frame number
       */

      {
        usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_SYNCHFRAME), 0);
        break;
      }

    default:
      {
        usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDCTRLREQ), 0);
        priv->stalled = true;
      }
      break;
    }
}

/****************************************************************************
 * Name: nrf52_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event
 *
 ****************************************************************************/

static void nrf52_ep0setup(struct nrf52_usbdev_s *priv)
{
  struct nrf52_ctrlreq_s ctrlreq;

  /* Terminate any pending requests */

  nrf52_req_cancel(&priv->epout[EP0], -EPROTO);
  nrf52_req_cancel(&priv->epin[EP0],  -EPROTO);

  /* Assume NOT stalled */

  priv->epout[EP0].stalled = false;
  priv->epin[EP0].stalled  = false;
  priv->stalled            = false;

  /* Get ctrlreq */

  ctrlreq.type  = priv->ctrlreq.type;
  ctrlreq.req   = priv->ctrlreq.req;
  ctrlreq.value = GETUINT16(priv->ctrlreq.value);
  ctrlreq.index = GETUINT16(priv->ctrlreq.index);
  ctrlreq.len   = GETUINT16(priv->ctrlreq.len);

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrlreq.type, ctrlreq.req, ctrlreq.value,
        ctrlreq.index, ctrlreq.len);

  /* Check for a standard request */

  if ((ctrlreq.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      /* Dispatch any non-standard requests */

      nrf52_req_dispatch(priv, &priv->ctrlreq);
    }
  else
    {
      /* Handle standard requests. */

      nrf52_ep0out_stdrequest(priv, &ctrlreq);
    }

  /* Check if the setup processing resulted in a STALL */

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_EP0SETUPSTALLED), 0);
      nrf52_ep0_stall(priv);
    }
}

/****************************************************************************
 * Name: nrf52_epin_transfer
 *
 * Description:
 *   Start the Tx data transfer
 *
 ****************************************************************************/

static void nrf52_epin_transfer(struct nrf52_ep_s *privep, uint8_t *buf,
                                int nbytes)
{
  struct nrf52_usbdev_s *priv = NULL;

  /* Sanity Checking */

  DEBUGASSERT(privep && privep->dev);
  priv = (struct nrf52_usbdev_s *)privep->dev;

  /* ISOC not supported yet */

  DEBUGASSERT(privep->eptype != USB_EP_ATTR_XFER_ISOC);
  DEBUGASSERT(nbytes <= 64);

  /* Configure EasyDMA */

  if (buf)
    {
      DEBUGASSERT(nrf52_easydma_valid((uint32_t)buf));
    }

  nrf52_putreg((uint32_t)buf, NRF52_USBD_EPIN_PTR(privep->epphy));
  nrf52_putreg(nbytes, NRF52_USBD_EPIN_MAXCNT(privep->epphy));

  /* Start EPIN task - DMA transfer */

  nrf52_epin_start(priv, privep->epphy);

  /* Busy wait for ENDEPIN to prevent any access to USBD registers during
   * EasyDMA transfer. Otherwise USBD is not stable.
   */

  while (nrf52_getreg(NRF52_USBD_EVENTS_ENDEPIN(privep->epphy)) == 0 &&
         nrf52_getreg(NRF52_USBD_EVENTS_USBRESET) == 0);
}

/****************************************************************************
 * Name: nrf52_epout_allow
 *
 * Description:
 *   Allow OUT trafic on this endpoint
 *
 ****************************************************************************/

static void nrf52_epout_allow(struct nrf52_ep_s *privep)
{
  /* Write to any value to accept further OUT traffic on this endpoint */

  nrf52_putreg(0, NRF52_USBD_SIZE_EPOUT(privep->epphy));
}

/****************************************************************************
 * Name: nrf52_epout_transfer
 *
 * Description:
 *   Start the Rx data transfer
 *
 ****************************************************************************/

static void nrf52_epout_transfer(struct nrf52_ep_s *privep)
{
  struct nrf52_usbdev_s *priv   = NULL;
  int                    nbytes = 0;

  /* Sanity Checking */

  DEBUGASSERT(privep && privep->dev);
  priv = (struct nrf52_usbdev_s *)privep->dev;

  if (privep->epphy == 0)
    {
      nbytes = USBDEV_EP0_MAXSIZE;
    }
  else
    {
      /* Number of bytes received last in the data stage of this OUT
       * endpoint
       */

      nbytes = nrf52_getreg(NRF52_USBD_SIZE_EPOUT(privep->epphy));
    }

  /* Configure EasyDMA */

  DEBUGASSERT(nrf52_easydma_valid((uint32_t)privep->rxbuff));
  nrf52_putreg((uint32_t)privep->rxbuff,
               NRF52_USBD_EPOUT_PTR(privep->epphy));
  nrf52_putreg(nbytes, NRF52_USBD_EPOUT_MAXCNT(privep->epphy));

  /* Start EPOUT task */

  nrf52_epout_start(priv, privep->epphy);

  /* Busy wait for ENDEPOUT to prevent any access to USBD registers during
   * EasyDMA transfer. Otherwise USBD is not stable.
   */

  while (nrf52_getreg(NRF52_USBD_EVENTS_ENDEPOUT(privep->epphy)) == 0 &&
         nrf52_getreg(NRF52_USBD_EVENTS_USBRESET) == 0);
}

/****************************************************************************
 * Name: nrf52_epin_request
 *
 * Description:
 *   Begin or continue write request processing.
 *
 ****************************************************************************/

static void nrf52_epin_request(struct nrf52_usbdev_s *priv,
                               struct nrf52_ep_s *privep)
{
  struct nrf52_req_s *privreq   = NULL;
  uint8_t            *buf       = NULL;
  int                 bytesleft = 0;
  int                 nbytes    = 0;

  /* If DMA is busy, add EP IN to the waiting list */

  if (priv->dmanow == true)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_DMABUSY), privep->epphy);
      priv->dmaepinwait |= (1 << privep->epphy);
      return;
    }

  /* Check the request from the head of the endpoint request queue */

  privreq = nrf52_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_EPINREQEMPTY), privep->epphy);
      return;
    }

  uinfo("EP%"  PRId8 " req=%p: len=%" PRId16 " xfrd=%"  PRId16 "\n",
        privep->epphy, privreq, privreq->req.len, privreq->req.xfrd);

  /* Add one more packet.  We will wait for the transfer
   * complete event before we add the next packet.
   */

  if (privreq->req.xfrd < privreq->req.len)
    {
      /* Get the number of bytes left to be sent in the request */

      bytesleft = privreq->req.len - privreq->req.xfrd;
      nbytes    = bytesleft;

      /* Limit the size of the transfer to one full packet */

      if (nbytes > 0)
        {
          /* Either send the maxpacketsize or all of the remaining data in
           * the request.
           */

          if (nbytes >= privep->ep.maxpacket)
            {
              nbytes =  privep->ep.maxpacket;
            }
        }

      /* Transfer data */

      buf = privreq->req.buf + privreq->req.xfrd;
      nrf52_epin_transfer(privep, buf, nbytes);

      /* Update for the next time through the loop */

      privreq->req.xfrd += nbytes;
    }
  else if (privreq->req.len == 0)
    {
      /* Zero-length packet */

      nrf52_epin_transfer(privep, NULL, 0);

      /* ACK zero-length DMA transfer right away */

      nrf52_startdma_ack(priv);
    }

  /* Has all the request data been sent? */

  if (privreq->req.xfrd >= privreq->req.len)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);

      /* We are finished with the request (although the transfer has not
       * yet completed).
       */

      nrf52_req_complete(privep, OK);

      if (privep->epphy == EP0)
        {
          priv->ep0indone = true;
        }
    }
}

/****************************************************************************
 * Name: nrf52_epout_complete
 *
 * Description:
 *   This function is called when an OUT transfer complete interrupt is
 *   received.  It completes the read request at the head of the endpoint's
 *   request queue.
 *
 ****************************************************************************/

static void nrf52_epout_complete(struct nrf52_ep_s *privep)
{
  struct nrf52_req_s *privreq = NULL;

  /* Sanity Checking */

  DEBUGASSERT(privep);

  /* Since a transfer just completed, there must be a read request at the
   * head of the endpoint request queue.
   */

  privreq = nrf52_rqpeek(privep);
  DEBUGASSERT(privreq);
  if (!privreq)
    {
      /* An OUT transfer completed, but no packet to receive the data.  This
       * should not happen.
       */

      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_EPOUTQEMPTY),
               privep->epphy);
      return;
    }

  uinfo("EP%d: len=%d xfrd=%d\n", privep->epphy, privreq->req.len,
        privreq->req.xfrd);

  /* Return the completed read request to the class driver and mark the
   * state IDLE.
   */

  usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
  nrf52_req_complete(privep, OK);
}

/****************************************************************************
 * Name: nrf52_ep0out_receive
 *
 * Description:
 *   This function will simply copy the incoming data into pending request's
 *   data buffer.
 *
 ****************************************************************************/

static void nrf52_ep0out_receive(struct nrf52_usbdev_s *priv)
{
  struct nrf52_ep_s *privep  = &priv->epout[EP0];
  uint16_t           bcnt    = 0;

  /* Get data size */

  bcnt = nrf52_getreg(NRF52_USBD_EPOUT_AMOUNT(0));

  uinfo("EP0: bcnt=%d\n", bcnt);
  usbtrace(TRACE_READ(EP0), bcnt);

  /* Transfer the data from the RX buffer */

  DEBUGASSERT(priv->ep0datlen + bcnt <= USBDEV_SETUP_MAXDATASIZE);
  memcpy(priv->ep0data + priv->ep0datlen, privep->rxbuff, bcnt);
  priv->ep0datlen += bcnt;

  if (priv->ep0datlen >= priv->ep0reqlen)
    {
      /* Now we can process the setup command */

      nrf52_ep0setup(priv);
      priv->ep0datlen = 0;
      priv->ep0outdone = true;
    }
  else
    {
      priv->ep0outdone = false;
    }
}

/****************************************************************************
 * Name: nrf52_epout_receive
 *
 * Description:
 *   This function will simply copy the incoming data into pending request's
 *   data buffer.
 *
 ****************************************************************************/

static void nrf52_epout_receive(struct nrf52_ep_s *privep)
{
  struct nrf52_req_s *privreq = NULL;
  uint8_t            *dest    = NULL;
  int                 buflen  = 0;
  int                 readlen = 0;
  int                 bcnt    = 0;

  /* Get data size */

  bcnt = nrf52_getreg(NRF52_USBD_EPOUT_AMOUNT(privep->epphy));

  /* Get a reference to the request at the head of the endpoint's request
   * queue.
   */

  privreq = nrf52_rqpeek(privep);
  if (!privreq)
    {
      /* Otherwise, the data is lost. This really should not happen if
       * NAKing is working as expected.
       */

      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_EPOUTQEMPTY),
               privep->epphy);

      return;
    }

  uinfo("EP%d: len=%d xfrd=%d\n", privep->epphy,
        privreq->req.len, privreq->req.xfrd);
  usbtrace(TRACE_READ(privep->epphy), bcnt);

  /* Get the number of bytes to transfer */

  buflen  = privreq->req.len - privreq->req.xfrd;
  DEBUGASSERT(buflen > 0 && buflen >= bcnt);
  readlen = MIN(buflen, bcnt);

  /* Get the destination of the data transfer */

  dest = privreq->req.buf + privreq->req.xfrd;

  /* Transfer the data from the RX buffer */

  memcpy(dest, privep->rxbuff, readlen);

  /* Update the number of bytes transferred */

  privreq->req.xfrd += readlen;

  if (privreq->req.xfrd >= privreq->req.len ||
      readlen < privep->ep.maxpacket)
    {
      /* Complete OUT transfer */

      nrf52_epout_complete(privep);
    }
}

/****************************************************************************
 * Name: nrf52_epout_handle
 *
 * Description:
 *
 ****************************************************************************/

static void nrf52_epout_handle(struct nrf52_usbdev_s *priv,
                               struct nrf52_ep_s *privep)
{
  struct nrf52_req_s *privreq = NULL;

  /* Not for EP0 */

  DEBUGASSERT(privep->epphy != EP0);

  /* If DMA is busy, add EP OUT to the waiting list */

  if (priv->dmanow == true)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_DMABUSY), privep->epphy);
      priv->dmaepoutwait |= (1 << privep->epphy);
      return;
    }

  /* Loop until a valid request is found (or the request queue is empty).
   * The loop is only need to look at the request queue again is an
   * invalid read request is encountered.
   */

  for (; ; )
    {
      /* Get a reference to the request at the head of the endpoint's
       * request queue
       */

      privreq = nrf52_rqpeek(privep);
      if (!privreq)
        {
          usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_EPOUTQEMPTY),
                   privep->epphy);
          return;
        }

      uinfo("EP%d: len=%d\n", privep->epphy, privreq->req.len);

      /* Ignore any attempt to receive a zero length packet (this really
       * should not happen).
       */

      if (privreq->req.len <= 0)
        {
          usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_EPOUTNULLPACKET), 0);
          nrf52_req_complete(privep, OK);
        }

      /* Otherwise, we have a usable read request...
       * break out of the loop
       */

      else
        {
          break;
        }
    }

  /* Prepare EP OUT DMA transfer */

  nrf52_epout_transfer(privep);
}

/****************************************************************************
 * Name: nrf52_req_complete
 *
 * Description:
 *   Handle termination of the request at the head of the endpoint request
 *   queue.
 *
 ****************************************************************************/

static void nrf52_req_complete(struct nrf52_ep_s *privep, int16_t result)
{
  struct nrf52_req_s *privreq = NULL;
  bool                stalled = false;

  /* Remove the request at the head of the request list */

  privreq = nrf52_req_remfirst(privep);
  DEBUGASSERT(privreq != NULL);

  /* If endpoint 0, temporarily reflect the state of protocol stalled
   * in the callback.
   */

  stalled = privep->stalled;
  if (privep->epphy == EP0)
    {
      privep->stalled = privep->dev->stalled;
    }

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);

  /* Restore the stalled indication */

  privep->stalled = stalled;
}

/****************************************************************************
 * Name: nrf52_req_cancel
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 ****************************************************************************/

static void nrf52_req_cancel(struct nrf52_ep_s *privep, int16_t status)
{
  while (!nrf52_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(privep->epphy),
               (nrf52_rqpeek(privep))->req.xfrd);
      nrf52_req_complete(privep, status);
    }
}

/****************************************************************************
 * Name: nrf52_req_dispatch
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically
 *   part of the USB interrupt handler.
 *
 ****************************************************************************/

static int nrf52_req_dispatch(struct nrf52_usbdev_s *priv,
                              const struct usb_ctrlreq_s *ctrl)
{
  int ret = -EIO;

  usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_DISPATCH), 0);
  if (priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl,
                        priv->ep0data, priv->ep0datlen);
    }

  if (ret < 0)
    {
      /* Stall on failure */

      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_DISPATCHSTALL), 0);
      priv->stalled = true;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_ep_findbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 ****************************************************************************/

static struct nrf52_ep_s *nrf52_ep_findbyaddr(struct nrf52_usbdev_s *priv,
                                              uint16_t eplog)
{
  struct nrf52_ep_s *privep = NULL;
  uint8_t            epphy  = USB_EPNO(eplog);

  if (epphy >= NRF52_NENDPOINTS)
    {
      return NULL;
    }

  /* Is this an IN or an OUT endpoint? */

  if (USB_ISEPIN(eplog))
    {
      privep = &priv->epin[epphy];
    }
  else
    {
      privep = &priv->epout[epphy];
    }

  /* Return endpoint reference */

  DEBUGASSERT(privep->epphy == epphy);
  return privep;
}

/****************************************************************************
 * Name: nrf52_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 ****************************************************************************/

static void nrf52_usbreset(struct nrf52_usbdev_s *priv)
{
  struct nrf52_ep_s *privep = NULL;
  uint32_t           regval = 0;
  int                i      = 0;

  uinfo("nrf52_usbreset\n");

  /* Tell the class driver that we are disconnected. The class
   * driver should then accept any new configurations.
   */

  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  /* Mark all endpoints as available */

  priv->epavail[0] = NRF52_EP_AVAILABLE;
  priv->epavail[1] = NRF52_EP_AVAILABLE;

  /* Disable all end points */

  for (i = 0; i < NRF52_NENDPOINTS ; i++)
    {
      /* Return write requests to the class implementation */

      privep = &priv->epin[i];
      nrf52_req_cancel(privep, -ESHUTDOWN);

      /* Reset IN endpoint status */

      privep->stalled = false;

      /* Stop EPIN taks */

      nrf52_epin_stop(priv, i);

      /* Return read requests to the class implementation */

      privep = &priv->epout[i];
      nrf52_req_cancel(privep, -ESHUTDOWN);

      /* Reset endpoint status */

      privep->stalled = false;

      /* Stop EPOUT taks */

      nrf52_epout_stop(priv, i);
    }

  /* Enable the interrupts */

  regval = NRF52_INT_DEFAULT;
  nrf52_putreg(regval, NRF52_USBD_INTEN);

  /* Reset device address to 0 */

  nrf52_setaddress(priv, 0);
  priv->usbdev.speed = USB_SPEED_FULL;

  /* Re-configure EP0 */

  nrf52_ep0_configure(priv);
}

/****************************************************************************
 * Name: nrf52_resumeinterrupt
 *
 * Description:
 *   Resume/remote wakeup detected interrupt
 *
 ****************************************************************************/

static void nrf52_resumeinterrupt(struct nrf52_usbdev_s *priv)
{
#ifdef CONFIG_USBDEV_LOWPOWER
  /* Force normal state */

  nrf52_putreg(0, NRF52_USBD_LOWPOWER);

  /* TODO: wait for event */
#endif

  /* Restore full power -- whatever that means for this particular board */

  nrf52_usbsuspend((struct usbdev_s *)priv, true);

  /* Notify the class driver of the resume event */

  if (priv->driver)
    {
      CLASS_RESUME(priv->driver, &priv->usbdev);
    }
}

/****************************************************************************
 * Name: nrf52_suspendinterrupt
 *
 * Description:
 *   USB suspend interrupt
 *
 ****************************************************************************/

static void nrf52_suspendinterrupt(struct nrf52_usbdev_s *priv)
{
  /* Notify the class driver of the suspend event */

  if (priv->driver)
    {
      CLASS_SUSPEND(priv->driver, &priv->usbdev);
    }

#ifdef CONFIG_USBDEV_LOWPOWER
  /* Force low power state */

  nrf52_putreg(1, NRF52_USBD_LOWPOWER);
#endif

  /* Let the board-specific logic know that we have entered the suspend
   * state
   */

  nrf52_usbsuspend((struct usbdev_s *)priv, false);
}

/****************************************************************************
 * Name: nrf52_eventinterrupt
 *
 * Description:
 *   Handle EVENT events
 *
 ****************************************************************************/

static void nrf52_eventinterrupt(struct nrf52_usbdev_s *priv)
{
  uint32_t regval = 0;
  uint32_t clr    = 0;

  regval = nrf52_getreg(NRF52_USBD_EVENTCAUSE);

  if (regval & USBD_EVENTCAUSE_ISOOUTCRC)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_ISOOUTCRC), 0);
      clr |= USBD_EVENTCAUSE_ISOOUTCRC;
    }

  if (regval & USBD_EVENTCAUSE_SUSPEND)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_SUSPEND), 0);
      nrf52_suspendinterrupt(priv);
      clr |= USBD_EVENTCAUSE_SUSPEND;
    }

  if (regval & USBD_EVENTCAUSE_RESUME)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_RESUME), 0);
      nrf52_resumeinterrupt(priv);
      clr |= USBD_EVENTCAUSE_RESUME;
    }

  if (regval & USBD_EVENTCAUSE_USBWUALLOWED)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_USBWUALLOWED), 0);
      clr |= USBD_EVENTCAUSE_USBWUALLOWED;
    }

  if (regval & USBD_EVENTCAUSE_READY)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_READY), 0);
      clr |= USBD_EVENTCAUSE_READY;
    }

  /* Clear all pending events */

  nrf52_putreg(clr, NRF52_USBD_EVENTCAUSE);
}

/****************************************************************************
 * Name: nrf52_ep0setupinterrupt
 *
 * Description:
 *   Handle EP0SETUP event
 *
 ****************************************************************************/

static void nrf52_ep0setupinterrupt(struct nrf52_usbdev_s *priv)
{
  /* Get SETUP data */

  priv->ctrlreq.type     = nrf52_getreg(NRF52_USBD_BMREQUESTTYPE);
  priv->ctrlreq.req      = nrf52_getreg(NRF52_USBD_BREQUEST);
  priv->ctrlreq.value[0] = nrf52_getreg(NRF52_USBD_WVALUEL);
  priv->ctrlreq.value[1] = nrf52_getreg(NRF52_USBD_WVALUEH);
  priv->ctrlreq.index[0] = nrf52_getreg(NRF52_USBD_WINDEXL);
  priv->ctrlreq.index[1] = nrf52_getreg(NRF52_USBD_WINDEXH);
  priv->ctrlreq.len[0]   = nrf52_getreg(NRF52_USBD_WLENGTHL);
  priv->ctrlreq.len[1]   = nrf52_getreg(NRF52_USBD_WLENGTHH);

  /* Store request len */

  priv->ep0reqlen = GETUINT16(priv->ctrlreq.len);

  if (USB_REQ_ISOUT(priv->ctrlreq.type))
    {
      priv->ep0datlen = 0;

      if (priv->ep0reqlen == 0)
        {
          nrf52_ep0setup(priv);
        }
      else
        {
          /* Allows OUT data stage on control endpoint 0 */

          nrf52_ep0rcvout_start(priv);
        }
    }
  else
    {
      /* Start the setup */

      nrf52_ep0setup(priv);
    }

  /* Handle zero length packets for IN and OUT */

  if (priv->ep0reqlen == 0)
    {
      /* Allows status stage on control endpoint 0 */

      nrf52_ep0status_start(priv);
    }
}

/****************************************************************************
 * Name: nrf52_ep0datainterrupt
 *
 * Description:
 *   Handle EP0DATADONE event
 *
 ****************************************************************************/

static void nrf52_ep0datainterrupt(struct nrf52_usbdev_s *priv)
{
  struct nrf52_ep_s *privep = NULL;

  if (USB_REQ_ISOUT(priv->ctrlreq.type))
    {
      /* Prepare EP OUT DMA transfer */

      privep = &priv->epout[EP0];
      nrf52_epout_transfer(privep);
    }
  else
    {
      if (priv->ep0indone)
        {
          /* Allows status stage on control endpoint 0 */

          nrf52_ep0status_start(priv);
          priv->ep0indone  = false;
        }
      else
        {
          /* Handle IN request */

          privep = &priv->epin[EP0];
          nrf52_epin_request(priv, privep);
        }
    }
}

/****************************************************************************
 * Name: nrf52_epdatainterrupt
 *
 * Description:
 *   Handle EPDATA event
 *
 ****************************************************************************/

static void nrf52_epdatainterrupt(struct nrf52_usbdev_s *priv)
{
  struct nrf52_ep_s *privep     = NULL;
  uint32_t           datastatus = 0;
  int                epno       = 0;

  /* Get pending data status */

  datastatus = nrf52_getreg(NRF52_USBD_EPDATASTATUS);

  /* Clear register */

  nrf52_putreg(datastatus, NRF52_USBD_EPDATASTATUS);

  /* Ignore EP0 */

  for (epno = 1; epno < NRF52_NENDPOINTS; epno += 1)
    {
      /* A data transfer has occurred on a IN endpoint */

      if (datastatus & USBD_EPDATASTATUS_EPIN(epno))
        {
          privep = &priv->epin[epno];
          nrf52_epin_request(priv, privep);
        }

      /* A data transfer has occurred on a OUT endpoint */

      if (datastatus & USBD_EPDATASTATUS_EPOUT(epno))
        {
          privep = &priv->epout[epno];
          nrf52_epout_handle(priv, privep);
        }
    }
}

/****************************************************************************
 * Name: nrf52_endepininterrupt
 *
 * Description:
 *   Handle ENDEPIN events
 *
 ****************************************************************************/

static void nrf52_endepininterrupt(struct nrf52_usbdev_s *priv,
                                   uint32_t irqnow)
{
  int epno = 0;

  /* Process each pending IN endpoint interrupt */

  for (epno = 0; epno < NRF52_NENDPOINTS; epno += 1)
    {
      if (irqnow & USBD_INT_ENDEPIN(epno))
        {
          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_ENDEPIN), epno);
        }
    }
}

/****************************************************************************
 * Name: nrf52_endepoutinterrupt
 *
 * Description:
 *   Handle ENDEPOUT events
 *
 ****************************************************************************/

static void nrf52_endepoutinterrupt(struct nrf52_usbdev_s *priv,
                                    uint32_t irqnow)
{
  struct nrf52_ep_s *privep = NULL;
  int                epno   = 0;

  /* Process each pending OUT endpoint interrupt */

  for (epno = 0; epno < NRF52_NENDPOINTS; epno += 1)
    {
      if (irqnow & USBD_INT_ENDEPOUT(epno))
        {
          usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_ENDEPOUT), epno);

          /* The whole EPOUT buffer has ben consumed */

          if (epno == 0)
            {
              nrf52_ep0out_receive(priv);

              if (priv->ep0outdone)
                {
                  /* Allows status stage on control endpoint 0 */

                  nrf52_ep0status_start(priv);
                  priv->ep0outdone = false;
                }
              else
                {
                  /* Allows OUT data stage on control endpoint 0 */

                  nrf52_ep0rcvout_start(priv);
                }
            }
          else
            {
              privep = &priv->epout[epno];
              nrf52_epout_receive(privep);
            }
        }
    }
}

/****************************************************************************
 * Name: nrf52_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 ****************************************************************************/

static int nrf52_usbinterrupt(int irq, void *context, void *arg)
{
  struct nrf52_usbdev_s *priv   = &g_usbdev;
  struct nrf52_ep_s     *privep = NULL;
  uint32_t               irqnow = 0;
  uint32_t               offset = 0;
  int                    i      = 0;

  usbtrace(TRACE_INTENTRY(NRF52_TRACEINTID_USB), 0);

  /* Handle all events */

  for (i = 0; i < USBD_INT_ALL_NUM; i++)
    {
      /* Get EVENT offset */

      offset = NRF52_USBD_EVENTS_USBRESET + 0x04 * i;

      /* Get EVENT state */

      irqnow |= nrf52_getreg(offset) << i;

      /* Clear EVENT */

      nrf52_putreg(0, offset);
    }

  /* USB reset interrupt */

  if (irqnow & USBD_INT_USBRESET)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_DEVRESET), 0);
      nrf52_usbreset(priv);
      goto intout;
    }

#ifdef CONFIG_USBDEV_SOFINTERRUPT
  /* Handle SOF */

  if (irqnow & USBD_INT_SOF)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_SOF), 0);
    }
#endif

  /* Handle USBEVENT */

  if (irqnow & USBD_INT_USBEVENT)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_USBEVENT), 0);
      nrf52_eventinterrupt(priv);
    }

  /* DMA transfer complete */

  if (irqnow & NRF52_USBD_DMAIRQ)
    {
      nrf52_startdma_ack(priv);
    }

  /* Handle EP0SETUP */

  if (irqnow & USBD_INT_EP0SETUP)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0SETUP), 0);
      nrf52_ep0setupinterrupt(priv);
    }

  /* Handle EP0DATADONE */

  if (irqnow & USBD_INT_EP0DATADONE)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EP0DATADONE), 0);
      nrf52_ep0datainterrupt(priv);
    }

  /* Handle ENDEPOUT */

  nrf52_endepoutinterrupt(priv, irqnow);

  /* Handle ENDEPIN */

  nrf52_endepininterrupt(priv, irqnow);

  /* Handle EPDATA */

  if (irqnow & USBD_INT_EPDATA)
    {
      usbtrace(TRACE_INTDECODE(NRF52_TRACEINTID_EPDATA), 0);
      nrf52_epdatainterrupt(priv);
    }

  /* Try to handle waiting OUT DMA requests */

  if (priv->dmanow == false)
    {
      if (priv->dmaepoutwait)
        {
          for (i = 0; i < NRF52_NENDPOINTS; i++)
            {
              if (priv->dmaepoutwait & (1 << i))
                {
                  priv->dmaepoutwait &= ~(1 << i);

                  privep = &priv->epout[i];
                  nrf52_epout_handle(priv, privep);
                  break;
                }
            }
        }
    }

  /* Try to handle waiting IN DMA requests */

  if (priv->dmanow == false)
    {
      if (priv->dmaepinwait)
        {
          for (i = 0; i < NRF52_NENDPOINTS; i++)
            {
              if (priv->dmaepinwait & (1 << i))
                {
                  priv->dmaepinwait &= ~(1 << i);

                  privep = &priv->epin[i];
                  nrf52_epin_request(priv, privep);
                  break;
                }
            }
        }
    }

intout:
  usbtrace(TRACE_INTEXIT(NRF52_TRACEINTID_USB), 0);
  return OK;
}

/****************************************************************************
 * Name: nrf52_epout_configure
 *
 * Description:
 *   Configure an OUT endpoint, making it usable
 *
 * Input Parameters:
 *   privep    - a pointer to an internal endpoint structure
 *   eptype    - The type of the endpoint
 *   maxpacket - The max packet size of the endpoint
 *
 ****************************************************************************/

static int nrf52_epout_configure(struct nrf52_ep_s *privep, uint8_t eptype,
                                 uint16_t maxpacket)
{
  uint32_t regval = 0;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);

  /* The packet size is in bytes for all EP */

  if (privep->epphy == EP0)
    {
      DEBUGASSERT(eptype == USB_EP_ATTR_XFER_CONTROL);

      /* EP0OUT EPTYP is read only ! */

      eptype = 0;
    }

  /* Enable the endpoint */

  regval = nrf52_getreg(NRF52_USBD_EPOUTEN);
  regval |= USBD_EPOUTEN_OUT(privep->epphy);
  nrf52_putreg(regval, NRF52_USBD_EPOUTEN);

  /* Save the endpoint configuration */

  privep->ep.maxpacket = maxpacket;
  privep->eptype       = eptype;
  privep->stalled      = false;

  /* Allocate buffer for EPOUT */

  privep->rxbuff = kmm_malloc(maxpacket);

  /* Enable the interrupt for this endpoint */

  regval = nrf52_getreg(NRF52_USBD_INTEN);
  regval |= USBD_INT_ENDEPOUT(privep->epphy);
  nrf52_putreg(regval, NRF52_USBD_INTEN);

  return OK;
}

/****************************************************************************
 * Name: nrf52_epin_configure
 *
 * Description:
 *   Configure an IN endpoint, making it usable
 *
 * Input Parameters:
 *   privep    - a pointer to an internal endpoint structure
 *   eptype    - The type of the endpoint
 *   maxpacket - The max packet size of the endpoint
 *
 ****************************************************************************/

static int nrf52_epin_configure(struct nrf52_ep_s *privep, uint8_t eptype,
                                uint16_t maxpacket)
{
  uint32_t regval = 0;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);

  if (privep->epphy == EP0)
    {
      DEBUGASSERT(eptype == USB_EP_ATTR_XFER_CONTROL);
    }
  else
    {
      DEBUGASSERT(eptype == USB_EP_ATTR_XFER_BULK || USB_EP_ATTR_XFER_INT);
    }

  /* Enable the endpoint */

  regval = nrf52_getreg(NRF52_USBD_EPINEN);
  regval |= USBD_EPINEN_IN(privep->epphy);
  nrf52_putreg(regval, NRF52_USBD_EPINEN);

  /* Save the endpoint configuration */

  privep->ep.maxpacket = maxpacket;
  privep->eptype       = eptype;
  privep->stalled      = false;

  /* NOTE: don't enable EPIN END interrupts as they cause EPIN locks */

  return OK;
}

/****************************************************************************
 * Name: nrf52_ep_configure
 *
 * Description:
 *   Configure endpoint, making it usable
 *
 * Input Parameters:
 *   ep   - the struct usbdev_ep_s instance obtained from allocep()
 *   desc - A struct usb_epdesc_s instance describing the endpoint
 *   last - true if this last endpoint to be configured.  Some hardware
 *          needs to take special action when all of the endpoints have been
 *          configured.
 *
 ****************************************************************************/

static int nrf52_ep_configure(struct usbdev_ep_s *ep,
                              const struct usb_epdesc_s *desc, bool last)
{
  struct nrf52_ep_s *privep    = (struct nrf52_ep_s *)ep;
  uint16_t           maxpacket = 0;
  uint8_t            eptype    = 0;
  int                ret       = 0;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);
  DEBUGASSERT(desc->addr == ep->eplog);

  /* Initialize EP capabilities */

  maxpacket = GETUINT16(desc->mxpacketsize);
  eptype    = desc->attr & USB_EP_ATTR_XFERTYPE_MASK;

  /* Setup Endpoint Control Register */

  if (privep->isin)
    {
      ret = nrf52_epin_configure(privep, eptype, maxpacket);
    }
  else
    {
      ret = nrf52_epout_configure(privep, eptype, maxpacket);
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_ep0_configure
 *
 * Description:
 *   Reset Usb engine
 *
 ****************************************************************************/

static void nrf52_ep0_configure(struct nrf52_usbdev_s *priv)
{
  /* Enable EP0 IN and OUT */

  nrf52_epin_configure(&priv->epin[EP0],
                       USB_EP_ATTR_XFER_CONTROL,
                       USBDEV_EP0_MAXSIZE);
  nrf52_epout_configure(&priv->epout[EP0],
                        USB_EP_ATTR_XFER_CONTROL,
                        USBDEV_EP0_MAXSIZE);
}

/****************************************************************************
 * Name: nrf52_epout_disable
 *
 * Description:
 *   Disable an OUT endpoint will no longer be used
 *
 ****************************************************************************/

static void nrf52_epout_disable(struct nrf52_ep_s *privep)
{
  uint32_t   regval = 0;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Disable the endpoint */

  regval = nrf52_getreg(NRF52_USBD_EPOUTEN);
  regval &= ~USBD_EPOUTEN_OUT(privep->epphy);
  nrf52_putreg(regval, NRF52_USBD_EPOUTEN);

  /* Disable endpoint interrupts */

  regval = nrf52_getreg(NRF52_USBD_INTEN);
  regval &= ~USBD_INT_ENDEPOUT(privep->epphy);
  nrf52_putreg(regval, NRF52_USBD_INTEN);

  /* Cancel any queued write requests */

  nrf52_req_cancel(privep, -ESHUTDOWN);

  /* Free buffer */

  if (privep->rxbuff)
    {
      kmm_free(privep->rxbuff);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf52_epin_disable
 *
 * Description:
 *   Disable an IN endpoint when it will no longer be used
 *
 ****************************************************************************/

static void nrf52_epin_disable(struct nrf52_ep_s *privep)
{
  irqstate_t flags;
  uint32_t   regval = 0;

  flags = enter_critical_section();

  /* Disable the endpoint */

  regval = nrf52_getreg(NRF52_USBD_EPINEN);
  regval &= ~USBD_EPINEN_IN(privep->epphy);
  nrf52_putreg(regval, NRF52_USBD_EPINEN);

  /* Cancel any queued write requests */

  nrf52_req_cancel(privep, -ESHUTDOWN);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf52_ep_disable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 ****************************************************************************/

static int nrf52_ep_disable(struct usbdev_ep_s *ep)
{
  struct nrf52_ep_s *privep = (struct nrf52_ep_s *)ep;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPDISABLE, privep->epphy);

  /* Is this an IN or an OUT endpoint */

  if (privep->isin)
    {
      /* Disable the IN endpoint */

      nrf52_epin_disable(privep);
    }
  else
    {
      /* Disable the OUT endpoint */

      nrf52_epout_disable(privep);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_ep_allocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 ****************************************************************************/

static struct usbdev_req_s *nrf52_ep_allocreq(struct usbdev_ep_s *ep)
{
  struct nrf52_req_s *privreq = NULL;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  usbtrace(TRACE_EPALLOCREQ, ((struct nrf52_ep_s *)ep)->epphy);

  privreq = (struct nrf52_req_s *)kmm_malloc(sizeof(struct nrf52_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct nrf52_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: nrf52_ep_freereq
 *
 * Description:
 *   Free an I/O request
 *
 ****************************************************************************/

static void nrf52_ep_freereq(struct usbdev_ep_s *ep,
                             struct usbdev_req_s *req)
{
  struct nrf52_req_s *privreq = (struct nrf52_req_s *)req;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((struct nrf52_ep_s *)ep)->epphy);
  kmm_free(privreq);
}

#ifdef CONFIG_USBDEV_DMA
/****************************************************************************
 * Name: nrf52_ep_allocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 ****************************************************************************/

static void *nrf52_ep_allocbuffer(struct usbdev_ep_s *ep, unsigned bytes)
{
  usbtrace(TRACE_EPALLOCBUFFER, ((struct nrf52_ep_s *)ep)->epphy);
  return kmm_malloc(bytes);
}

/****************************************************************************
 * Name: nrf52_ep_freebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 ****************************************************************************/

static void nrf52_ep_freebuffer(struct usbdev_ep_s *ep, void *buf)
{
  usbtrace(TRACE_EPALLOCBUFFER, ((struct nrf52_ep_s *)ep)->epphy);
  kmm_free(buf);
}
#endif

/****************************************************************************
 * Name: nrf52_ep_submit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 ****************************************************************************/

static int nrf52_ep_submit(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct nrf52_req_s    *privreq = (struct nrf52_req_s *)req;
  struct nrf52_ep_s     *privep  = (struct nrf52_ep_s *)ep;
  struct nrf52_usbdev_s *priv    = NULL;
  irqstate_t             flags;
  int                    ret     = OK;

  /* Some sanity checking */

  DEBUGASSERT(privep && privep->dev);
  priv = (struct nrf52_usbdev_s *)privep->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      uinfo("req=%p callback=%p buf=%p ep=%p\n",
            req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->epphy);

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv->driver)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_NOTCONFIGURED),
               priv->usbdev.speed);
      return -ESHUTDOWN;
    }
#endif

  /* Handle the request from the class driver */

  req->result = -EINPROGRESS;
  req->xfrd   = 0;

  /* Disable Interrupts */

  flags = enter_critical_section();

  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Add the new request to the request queue for the endpoint. */

      if (nrf52_req_addlast(privep, privreq))
        {
          /* If a request was added to an IN endpoint, then attempt to send
           * the request data buffer now.
           */

          if (privep->isin)
            {
              usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);
              nrf52_epin_request(priv, privep);
            }

          /* If the request was added to an OUT endpoint, then attempt to
           * setup a read into the request data buffer now (this will, of
           * course, fail if there is already a read in place).
           */

          else
            {
              usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);

              /* Allow OUT trafic on this endpoint */

              nrf52_epout_allow(privep);
            }
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: nrf52_ep_cancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 ****************************************************************************/

static int nrf52_ep_cancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct nrf52_ep_s *privep = (struct nrf52_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, privep->epphy);

  flags = enter_critical_section();
  nrf52_req_cancel(privep, -ESHUTDOWN);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: nrf52_ep_setstall
 *
 * Description:
 *   Stall an endpoint
 *
 ****************************************************************************/

static int nrf52_ep_setstall(struct nrf52_ep_s *privep)
{
  uint32_t regval = 0;

  /* Is this an IN endpoint? */

  if (privep->isin == 1)
    {
      regval |= USBD_EPSTALL_IO_IN;
    }
  else
    {
      regval |= USBD_EPSTALL_IO_OUT;
    }

  /* Unstall a given EP */

  regval |= USBD_EPSTALL_EP(privep->epphy) | USBD_EPSTALL_IO_STALL;
  nrf52_putreg(regval, NRF52_USBD_EPSTALL);

  /* The endpoint is now stalled */

  privep->stalled = true;
  return OK;
}

/****************************************************************************
 * Name: nrf52_ep_clrstall
 *
 * Description:
 *   Resume a stalled endpoint
 *
 ****************************************************************************/

static int nrf52_ep_clrstall(struct nrf52_ep_s *privep)
{
  uint32_t regval = 0;

  /* Is this an IN endpoint? */

  if (privep->isin == 1)
    {
      regval |= USBD_EPSTALL_IO_IN;
    }
  else
    {
      regval |= USBD_EPSTALL_IO_OUT;
    }

  /* Unstall a given EP */

  regval |= USBD_EPSTALL_EP(privep->epphy) | USBD_EPSTALL_IO_UNSTALL;
  nrf52_putreg(regval, NRF52_USBD_EPSTALL);

  /* The endpoint is no longer stalled */

  privep->stalled = false;
  return OK;
}

/****************************************************************************
 * Name: nrf52_ep_stall
 *
 * Description:
 *   Stall or resume an endpoint
 *
 ****************************************************************************/

static int nrf52_ep_stall(struct usbdev_ep_s *ep, bool resume)
{
  struct nrf52_ep_s *privep = (struct nrf52_ep_s *)ep;
  irqstate_t flags;
  int ret = 0;

  /* Set or clear the stall condition as requested */

  flags = enter_critical_section();

  if (resume)
    {
      ret = nrf52_ep_clrstall(privep);
    }
  else
    {
      ret = nrf52_ep_setstall(privep);
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: nrf52_ep0_stall
 *
 * Description:
 *   Stall endpoint 0
 *
 ****************************************************************************/

static void nrf52_ep0_stall(struct nrf52_usbdev_s *priv)
{
  nrf52_putreg(1, NRF52_USBD_TASKS_EP0STALL);
  priv->stalled = true;
}

/****************************************************************************
 * Name: nrf52_ep_alloc
 *
 * Description:
 *   Allocate an endpoint matching the parameters.
 *
 * Input Parameters:
 *   eplog  - 7-bit logical endpoint number (direction bit ignored).
 *            Zero means that any endpoint matching the other requirements
 *            will suffice.
 *            The assigned endpoint can be found in the eplog field.
 *   in     - true: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.  One of {USB_EP_ATTR_XFER_ISOC,
 *            USB_EP_ATTR_XFER_BULK, USB_EP_ATTR_XFER_INT}
 *
 ****************************************************************************/

static struct usbdev_ep_s *
nrf52_ep_alloc(struct usbdev_s *dev, uint8_t eplog, bool in, uint8_t eptype)
{
  struct nrf52_usbdev_s *priv    = (struct nrf52_usbdev_s *)dev;
  irqstate_t             flags;
  uint8_t                epavail = 0;
  uint8_t                bit     = 0;
  int                    epphy   = 0;
  int                    epno    = 0;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)eplog);

  /* Ignore any direction bits in the logical address */

  epphy = USB_EPNO(eplog);

  /* Get the set of available endpoints depending on the direction */

  flags = enter_critical_section();
  epavail = priv->epavail[in];

  /* A physical address of 0 means that any endpoint will do */

  if (epphy > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the
       * requested 'logical' endpoint.  All of the other checks will
       * still be performed.
       *
       * First, verify that the logical endpoint is in the range supported
       * by by the hardware.
       */

      if (epphy >= NRF52_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BADEPNO), (uint16_t)epphy);
          return NULL;
        }

      /* Remove all of the candidate endpoints from the bitset except for the
       * this physical endpoint number.
       */

      epavail &= (1 << epphy);
    }

  /* Is there an available endpoint? */

  if (epavail)
    {
      /* Yes.. Select the lowest numbered endpoint in the set of available
       * endpoints.
       */

      for (epno = 1; epno < NRF52_NENDPOINTS; epno++)
        {
          bit = 1 << epno;
          if ((epavail & bit) != 0)
            {
              /* Mark the endpoint no longer available */

              priv->epavail[in] &= ~(1 << epno);

              /* And return the pointer to the standard endpoint structure */

              leave_critical_section(flags);
              return in ? &priv->epin[epno].ep : &priv->epout[epno].ep;
            }
        }

      /* We should not get here */
    }

  usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_NOEP), (uint16_t)eplog);
  leave_critical_section(flags);
  return NULL;
}

/****************************************************************************
 * Name: nrf52_ep_free
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 ****************************************************************************/

static void nrf52_ep_free(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct nrf52_usbdev_s *priv   = (struct nrf52_usbdev_s *)dev;
  struct nrf52_ep_s     *privep = (struct nrf52_ep_s *)ep;
  irqstate_t             flags;

  usbtrace(TRACE_DEVFREEEP, (uint16_t)privep->epphy);

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      flags = enter_critical_section();
      priv->epavail[privep->isin] |= (1 << privep->epphy);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: nrf52_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 ****************************************************************************/

static int nrf52_getframe(struct usbdev_s *dev)
{
  usbtrace(TRACE_DEVGETFRAME, 0);

  /* Return the last frame number of the last SOF detected by the hardware */

  return nrf52_getreg(NRF52_USBD_FRAMECNTR) & USBD_FRAMECNTR_MASK;
}

/****************************************************************************
 * Name: nrf52_wakeup
 *
 * Description:
 *   Exit suspend mode.
 *
 ****************************************************************************/

static int nrf52_wakeup(struct usbdev_s *dev)
{
  struct nrf52_usbdev_s *priv = (struct nrf52_usbdev_s *)dev;
  irqstate_t             flags;

  usbtrace(TRACE_DEVWAKEUP, 0);

  /* Is wakeup enabled? */

  flags = enter_critical_section();
  if (priv->wakeup)
    {
#ifdef CONFIG_USBDEV_LOWPOWER
      /* Force normal state */

      nrf52_putreg(0, NRF52_USBD_LOWPOWER);

      /* TODO: wait for event */
#endif

      /* Revisit: */
#if 0
      nrf52_putreg(USBD_DPDMVALUE_STATE_RESUME, NRF52_USBD_DPDMVALUE);
      nrf52_putreg(1, NRF52_USBD_TASKS_DPDMDRIVE);
#endif
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: nrf52_selfpowered
 *
 * Description:
 *   Sets/clears the device self-powered feature
 *
 ****************************************************************************/

static int nrf52_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct nrf52_usbdev_s *priv = (struct nrf52_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Name: nrf52_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 ****************************************************************************/

int nrf52_pullup(struct usbdev_s *dev, bool enable)
{
  if (enable)
    {
      nrf52_putreg(USBD_USBPULLUP_ENABLE, NRF52_USBD_USBPULLUP);
    }
  else
    {
      nrf52_putreg(USBD_USBPULLUP_DISABLE, NRF52_USBD_USBPULLUP);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_setaddress
 *
 * Description:
 *   Set the devices USB address
 *
 ****************************************************************************/

static void nrf52_setaddress(struct nrf52_usbdev_s *priv, uint16_t address)
{
  /* Are we now addressed?  (i.e., do we have a non-NULL device
   * address?)
   */

  if (address != 0)
    {
      priv->addressed = true;
    }
  else
    {
      priv->addressed = false;
    }
}

/****************************************************************************
 * Name: nrf52_swinitialize
 *
 * Description:
 *   Initialize all driver data structures.
 *
 ****************************************************************************/

static void nrf52_swinitialize(struct nrf52_usbdev_s *priv)
{
  struct nrf52_ep_s *privep;
  int i;

  /* Initialize the device state structure */

  memset(priv, 0, sizeof(struct nrf52_usbdev_s));

  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->epin[EP0].ep;

  priv->epavail[0] = NRF52_EP_AVAILABLE;
  priv->epavail[1] = NRF52_EP_AVAILABLE;

  /* Initialize the endpoint lists */

  for (i = 0; i < NRF52_NENDPOINTS; i++)
    {
      privep         = &priv->epin[i];
      privep->ep.ops = &g_epops;
      privep->dev    = priv;
      privep->isin   = 1;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      privep->epphy    = i;
      privep->ep.eplog = NRF52_EPPHYIN2LOG(i);

      /* Control until endpoint is activated */

      privep->eptype       = USB_EP_ATTR_XFER_CONTROL;
      privep->ep.maxpacket = USBDEV_EP0_MAXSIZE;
    }

  /* Initialize the endpoint lists */

  for (i = 0; i < NRF52_NENDPOINTS; i++)
    {
      privep         = &priv->epout[i];
      privep->ep.ops = &g_epops;
      privep->dev    = priv;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      privep->epphy    = i;
      privep->ep.eplog = NRF52_EPPHYOUT2LOG(i);

      /* Control until endpoint is activated */

      privep->eptype       = USB_EP_ATTR_XFER_CONTROL;
      privep->ep.maxpacket = USBDEV_EP0_MAXSIZE;
    }
}

/****************************************************************************
 * Name: nrf52_hwinitialize
 *
 * Description:
 *   Configure the USB core for operation.
 *
 ****************************************************************************/

static void nrf52_hwinitialize(struct nrf52_usbdev_s *priv)
{
  /* Wait for VBUS */

  /* TODO: connect to POWER USB events */

  while (getreg32(NRF52_POWER_EVENTS_USBDETECTED) == 0);

  /* Errata [187] USBD: USB cannot be enabled */

  *(volatile uint32_t *)0x4006ec00 = 0x00009375;
  *(volatile uint32_t *)0x4006ed14 = 0x00000003;
  *(volatile uint32_t *)0x4006ec00 = 0x00009375;

  /* Enable the USB controller */

  nrf52_putreg(USBD_ENABLE_ENABLE, NRF52_USBD_ENABLE);

  while ((nrf52_getreg(NRF52_USBD_EVENTCAUSE) & USBD_EVENTCAUSE_READY) == 0);
  nrf52_putreg(USBD_EVENTCAUSE_READY, NRF52_USBD_EVENTCAUSE);

  *(volatile uint32_t *)0x4006ec00 = 0x00009375;
  *(volatile uint32_t *)0x4006ed14 = 0x00000000;
  *(volatile uint32_t *)0x4006ec00 = 0x00009375;

  /* Disable all endpoints */

  nrf52_putreg(0, NRF52_USBD_EPOUTEN);
  nrf52_putreg(0, NRF52_USBD_EPINEN);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_usbinitialize
 *
 * Description:
 *   Initialize the USB driver
 *
 ****************************************************************************/

void arm_usbinitialize(void)
{
  struct nrf52_usbdev_s *priv = &g_usbdev;
  int                    ret  = 0;

  usbtrace(TRACE_DEVINIT, 0);

  /* Uninitialize the hardware so that we know that we are starting from a
   * known state.
   */

  arm_usbuninitialize();

  /* Initialie the driver data structure */

  nrf52_swinitialize(priv);

  /* Attach the USB interrupt handler */

  ret = irq_attach(NRF52_IRQ_USBD, nrf52_usbinterrupt, NULL);
  if (ret < 0)
    {
      uerr("ERROR: irq_attach failed: %d\n", ret);
      goto errout;
    }

  /* Initialize the USB core */

  nrf52_hwinitialize(priv);

  /* Disconnect device */

  nrf52_pullup(&priv->usbdev, false);

  /* Reset/Re-initialize the USB hardware */

  nrf52_usbreset(priv);

  /* Enable USB controller interrupts at the NVIC */

  up_enable_irq(NRF52_IRQ_USBD);
  return;

errout:
  arm_usbuninitialize();
}

/****************************************************************************
 * Name: arm_usbuninitialize
 *
 * Description:
 *   Initialize the USB driver
 *
 ****************************************************************************/

void arm_usbuninitialize(void)
{
  struct nrf52_usbdev_s *priv = &g_usbdev;
  irqstate_t             flags;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Disconnect device */

  flags = enter_critical_section();
  nrf52_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable and detach IRQs */

  up_disable_irq(NRF52_IRQ_USBD);
  irq_detach(NRF52_IRQ_USBD);

  /* Disable all interrupts */

  nrf52_putreg(0, NRF52_USBD_INTEN);

  /* Disable the USB controller */

  nrf52_putreg(USBD_ENABLE_DISABLE, NRF52_USBD_ENABLE);

  leave_critical_section(flags);
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
  struct nrf52_usbdev_s *priv = &g_usbdev;
  int                    ret  = 0;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_BINDFAILED), (uint16_t)-ret);
      priv->driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts */

      up_enable_irq(NRF52_IRQ_USBD);

      nrf52_pullup(&priv->usbdev, true);
      priv->usbdev.speed = USB_SPEED_FULL;
    }

  return ret;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver. If the USB device is connected to a
 *   USB host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *   returns.
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  struct nrf52_usbdev_s *priv = &g_usbdev;
  irqstate_t             flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(NRF52_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = enter_critical_section();
  nrf52_usbreset(priv);
  leave_critical_section(flags);

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts */

  flags = enter_critical_section();
  up_disable_irq(NRF52_IRQ_USBD);

  /* Disconnect device */

  nrf52_pullup(&priv->usbdev, false);

  /* Unhook the driver */

  priv->driver = NULL;
  leave_critical_section(flags);

  return OK;
}
