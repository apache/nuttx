/****************************************************************************
 * drivers/usbdev/cdcncm.c
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

/* References:
 *   [CDCNCM1.2] Universal Serial Bus - Communications Class - Subclass
 *               Specification for Ethernet Control Model Devices - Rev 1.2
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/cdcncm.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_BOARD_USBDEV_SERIALSTR
#  include <nuttx/board.h>
#endif

#include "cdcecm.h"

#ifdef CONFIG_NET_CDCNCM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK. NOTE: Use of the high priority work queue will
 * have a negative impact on interrupt handling latency and overall system
 * performance.  This should be avoided.
 */

#define ETHWORK LPWORK

/* CONFIG_CDCECM_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_CDCECM_NINTERFACES
#  define CONFIG_CDCECM_NINTERFACES 1
#endif

/* TX timeout = 1 minute */

#define CDCNCM_TXTIMEOUT             (60*CLK_TCK)
#define CDCNCM_DGRAM_COMBINE_PERIOD   1

#define NTB_DEFAULT_IN_SIZE           16384
#define NTB_OUT_SIZE                  16384
#define TX_MAX_NUM_DPE                32

/* NCM Transfer Block Parameter Structure */

#define CDC_NCM_NTB16_SUPPORTED      (1 << 0)
#define CDC_NCM_NTB32_SUPPORTED      (1 << 1)

#define FORMATS_SUPPORTED            (CDC_NCM_NTB16_SUPPORTED | \
                                      CDC_NCM_NTB32_SUPPORTED)

#define CDC_NCM_NTH16_SIGN            0x484D434E /* NCMH */
#define CDC_NCM_NTH32_SIGN            0x686D636E /* ncmh */
#define CDC_NCM_NDP16_NOCRC_SIGN      0x304D434E /* NCM0 */
#define CDC_NCM_NDP32_NOCRC_SIGN      0x306D636E /* ncm0 */

#define INIT_NDP16_OPTS {                               \
    .nthsign      = CDC_NCM_NTH16_SIGN,                 \
    .ndpsign      = CDC_NCM_NDP16_NOCRC_SIGN,           \
    .nthsize      = sizeof(struct cdc_ncm_nth16_s),     \
    .ndpsize      = sizeof(struct usb_cdc_ncm_ndp16_s), \
    .dpesize      = sizeof(struct usb_cdc_ncm_dpe16_s), \
    .ndpalign     = 4,                                  \
    .dgramitemlen = 2,                                  \
    .blocklen     = 2,                                  \
    .ndpindex     = 2,                                  \
    .reserved1    = 0,                                  \
    .reserved2    = 0,                                  \
    .nextndpindex = 2,                                  \
  }

#define INIT_NDP32_OPTS {                               \
    .nthsign      = CDC_NCM_NTH32_SIGN,                 \
    .ndpsign      = CDC_NCM_NDP32_NOCRC_SIGN,           \
    .nthsize      = sizeof(struct cdc_ncm_nth32_s),     \
    .ndpsize      = sizeof(struct usb_cdc_ncm_ndp32_s), \
    .dpesize      = sizeof(struct usb_cdc_ncm_dpe32_s), \
    .ndpalign     = 8,                                  \
    .dgramitemlen = 4,                                  \
    .blocklen     = 4,                                  \
    .ndpindex     = 4,                                  \
    .reserved1    = 2,                                  \
    .reserved2    = 4,                                  \
    .nextndpindex = 4,                                  \
  }

#define CDC_NCM_NCAP_ETH_FILTER (1 << 0)
#define NCAPS                   (CDC_NCM_NCAP_ETH_FILTER)

#define NCM_ALIGN_MASK(x, mask) (((x) + (mask)) & ~(mask))
#define NCM_ALIGN(x, a)         NCM_ALIGN_MASK((x), ((typeof(x))(a) - 1))

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum ncm_notify_state_e
{
  NCM_NOTIFY_NONE,    /* Don't notify */
  NCM_NOTIFY_CONNECT, /* Issue CONNECT next */
  NCM_NOTIFY_SPEED,   /* Issue SPEED_CHANGE next */
};

struct ndp_parser_opts_s
{
  uint32_t nthsign;          /* NCM Transfer Header signature */
  uint32_t ndpsign;          /* NCM Datagram Pointer signature */
  uint_fast8_t nthsize;      /* The length of NTH */
  uint_fast8_t ndpsize;      /* The length of NDP */
  uint_fast8_t dpesize;      /* The length of NDP Entry */
  uint_fast8_t ndpalign;     /* NDP alignment length */
  uint_fast8_t dgramitemlen; /* The length of index or length */
  uint_fast8_t blocklen;     /* The length of current NTB */
  uint_fast8_t ndpindex;     /* The offset of first NDP in current NTB */
  uint_fast8_t reserved1;    /* Reserved1 */
  uint_fast8_t reserved2;    /* Reserved2 */
  uint_fast8_t nextndpindex; /* The offset of next NDP in current NTB */
};

/* NTH: NCM Transfer Header
 * NDP: NCM Datagram Pointer
 * DPE: NCM Datagram Pointer Entry
 * +------------+  or   +------------+
 * |     NTH    |       |     NTH    |
 * +------------+       +------------+
 * |     NDP    |       |  Datagrams |
 * +------------+       +------------+
 * |  Datagrams |       |     NDP    |
 * +------------+       +------------+
 *
 * The layout of the NTB(NCM Transfer Block) structure in the NuttX system
 * is as follows:
 * +--------------------------+
 * |NTH :       nth sign      |
 * |            nth len       |
 * |            sequence      |
 * |            total len     |
 * |            ndp index     |----+
 * +--------------------------+    |
 * |NDP:        ndp sign      |<---+
 * |            ndp len       |
 * |            next ndp index|
 * |            Datagram index|----+
 * |            Datagram len  |    |
 * |            Datagram index|----|--+
 * |            Datagram len  |    |  |
 * |            Datagram index|----|--|--+
 * |            Datagram len  |    |  |  |
 * |            0             | Need to end with two zeros
 * |            0             | Need to end with two zeros
 * |            ... [32]      |    |  |  |
 * +--------------------------+    |  |  |
 * |Datagrams:  Datagram1     |<---+  |  |
 * |            pad           |       |  |
 * |            Datagram2     |<------+  |
 * |            pad           |          |
 * |            Datagram3     |<---------+
 * +--------------------------+
 */

begin_packed_struct struct cdc_ncm_nth16_s
{
  uint32_t sign;
  uint16_t headerlen;
  uint16_t seq;
  uint16_t blocklen;
  uint16_t ndpindex;
} end_packed_struct;

begin_packed_struct struct cdc_ncm_nth32_s
{
  uint32_t sign;
  uint16_t headerlen;
  uint16_t seq;
  uint32_t blocklen;
  uint32_t ndpindex;
} end_packed_struct;

/* 16-bit NCM Datagram Pointer Entry */

begin_packed_struct struct usb_cdc_ncm_dpe16_s
{
  uint16_t index;
  uint16_t len;
} end_packed_struct;

/* 16-bit NCM Datagram Pointer Table */

begin_packed_struct struct usb_cdc_ncm_ndp16_s
{
  uint32_t sign;
  uint16_t len;
  uint16_t nextndpindex;

  /* struct usb_cdc_ncm_dpe16_s dpe16[]; */
} end_packed_struct;

/* 32-bit NCM Datagram Pointer Entry */

begin_packed_struct struct usb_cdc_ncm_dpe32_s
{
  uint32_t index;
  uint32_t len;
} end_packed_struct;

/* 32-bit NCM Datagram Pointer Table */

begin_packed_struct struct usb_cdc_ncm_ndp32_s
{
  uint32_t sign;
  uint16_t len;
  uint16_t reserved1;
  uint32_t nextndpindex;
  uint32_t reserved2;

  /* struct usb_cdc_ncm_dpe32_s dpe32[]; */
} end_packed_struct;

begin_packed_struct struct usb_cdc_ncm_ntb_parameters_s
{
  uint16_t len;
  uint16_t ntbsupported;
  uint32_t ntbinmaxsize;
  uint16_t ndpindivisor;
  uint16_t ndpinpayloadremainder;
  uint16_t ndpinalignment;
  uint16_t padding;
  uint32_t ntboutmaxsize;
  uint16_t ndpoutdivisor;
  uint16_t ndpoutpayloadremainder;
  uint16_t ndpoutalignment;
  uint16_t ntboutmaxdatagrams;
} end_packed_struct;

/* The cdcncm_driver_s encapsulates all state information for a single
 * hardware interface
 */

struct cdcncm_driver_s
{
  /* USB CDC-ECM device */

  struct usbdevclass_driver_s   usbdev;      /* USB device class vtable */
  struct usbdev_devinfo_s       devinfo;
  FAR struct usbdev_req_s      *ctrlreq;     /* Allocated control request */
  FAR struct usbdev_req_s      *notifyreq;   /* Allocated norify request */
  FAR struct usbdev_ep_s       *epint;       /* Interrupt IN endpoint */
  FAR struct usbdev_ep_s       *epbulkin;    /* Bulk IN endpoint */
  FAR struct usbdev_ep_s       *epbulkout;   /* Bulk OUT endpoint */
  uint8_t                       config;      /* Selected configuration number */

  FAR struct usbdev_req_s      *rdreq;       /* Single read request */
  bool                          rxpending;   /* Packet available in rdreq */

  FAR struct usbdev_req_s      *wrreq;       /* Single write request */
  sem_t                         wrreq_idle;  /* Is the wrreq available? */
  bool                          txdone;      /* Did a write request complete? */
  enum ncm_notify_state_e       notify;      /* State of notify */
  FAR const struct ndp_parser_opts_s
                               *parseropts;  /* Options currently used to parse NTB */
  uint32_t                      ndpsign;     /* NDP signature */
  int                           dgramcount;  /* The current tx cache dgram count */
  FAR uint8_t                  *dgramaddr;   /* The next tx cache dgram address */

  /* Network device */

  bool                          bifup;       /* true:ifup false:ifdown */
  struct work_s                 irqwork;     /* For deferring interrupt work
                                              * to the work queue */
  struct work_s                 notifywork;  /* For deferring notify work
                                              * to the work queue */
  struct work_s                 delaywork;   /* For deferring tx work
                                              * to the work queue */

  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s     dev;         /* Interface understood by the
                                              * network */
  netpkt_queue_t                rx_queue;    /* RX packet queue */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Network Device ***********************************************************/

/* Interrupt handling */

static void cdcncm_receive(FAR struct cdcncm_driver_s *priv);
static void cdcncm_txdone(FAR struct cdcncm_driver_s *priv);

static void cdcncm_interrupt_work(FAR void *arg);

/* NuttX callback functions */

static int  cdcncm_ifup(FAR struct netdev_lowerhalf_s *dev);
static int  cdcncm_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int  cdcncm_send(struct netdev_lowerhalf_s *dev, netpkt_t *pkt);
static FAR netpkt_t *cdcncm_recv(FAR struct netdev_lowerhalf_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  cdcncm_addmac(FAR struct netdev_lowerhalf_s *dev,
                          FAR const uint8_t *mac);
#ifdef CONFIG_NET_MCASTGROUP
static int  cdcncm_rmmac(FAR struct netdev_lowerhalf_s *dev,
                         FAR const uint8_t *mac);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  cdcncm_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                         unsigned long arg);
#endif

/* USB Device Class Driver **************************************************/

/* USB Device Class methods */

static int  cdcncm_bind(FAR struct usbdevclass_driver_s *driver,
                        FAR struct usbdev_s *dev);

static void cdcncm_unbind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev);

static int  cdcncm_setup(FAR struct usbdevclass_driver_s *driver,
                         FAR struct usbdev_s *dev,
                         FAR const struct usb_ctrlreq_s *ctrl,
                         FAR uint8_t *dataout, size_t outlen);

static void cdcncm_disconnect(FAR struct usbdevclass_driver_s *driver,
                              FAR struct usbdev_s *dev);

/* USB Device Class helpers */

static void cdcncm_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req);
static void cdcncm_rdcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req);
static void cdcncm_wrcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req);

static void cdcncm_mkepdesc(int epidx, FAR struct usb_epdesc_s *epdesc,
                            FAR struct usbdev_devinfo_s *devinfo,
                            bool hispeed);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB Device Class Methods */

static const struct usbdevclass_driverops_s g_usbdevops =
{
  cdcncm_bind,
  cdcncm_unbind,
  cdcncm_setup,
  cdcncm_disconnect,
  NULL,
  NULL
};

#ifndef CONFIG_CDCNCM_COMPOSITE
static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,
  USB_DESC_TYPE_DEVICE,
  {
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  USB_CLASS_CDC,
  CDC_SUBCLASS_NCM,
  CDC_PROTO_NONE,
  CONFIG_CDCNCM_EP0MAXPACKET,
  {
    LSBYTE(CONFIG_CDCNCM_VENDORID),
    MSBYTE(CONFIG_CDCNCM_VENDORID)
  },
  {
    LSBYTE(CONFIG_CDCNCM_PRODUCTID),
    MSBYTE(CONFIG_CDCNCM_PRODUCTID)
  },
  {
    LSBYTE(CDCECM_VERSIONNO),
    MSBYTE(CDCECM_VERSIONNO)
  },
  CDCNCM_MANUFACTURERSTRID,
  CDCNCM_PRODUCTSTRID,
  CDCNCM_SERIALSTRID,
  CDCECM_NCONFIGS
};
#endif

static const struct ndp_parser_opts_s g_ndp16_opts = INIT_NDP16_OPTS;
static const struct ndp_parser_opts_s g_ndp32_opts = INIT_NDP32_OPTS;

static const struct usb_cdc_ncm_ntb_parameters_s g_ntbparameters =
{
  .len                    = sizeof(g_ntbparameters),
  .ntbsupported           = FORMATS_SUPPORTED,
  .ntbinmaxsize           = NTB_DEFAULT_IN_SIZE,
  .ndpindivisor           = 4,
  .ndpinpayloadremainder  = 0,
  .ndpinalignment         = 4,

  .ntboutmaxsize          = NTB_OUT_SIZE,
  .ndpoutdivisor          = 4,
  .ndpoutpayloadremainder = 0,
  .ndpoutalignment        = 4,
};

static const struct netdev_ops_s g_netops =
{
  cdcncm_ifup,   /* ifup */
  cdcncm_ifdown, /* ifdown */
  cdcncm_send,   /* transmit */
  cdcncm_recv,   /* receive */
#ifdef CONFIG_NET_MCASTGROUP
  cdcncm_addmac, /* addmac */
  cdcncm_rmmac,  /* rmmac */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  cdcncm_ioctl,  /* ioctl */
#endif
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcncm_get
 *
 * Description:
 *   Read size length data from address and increases the address by the
 *   corresponding size
 *
 * Input Parameters:
 *   address - Pointer to address
 *   size    - Size of data
 *
 * Returned Value:
 *   The read value
 *
 ****************************************************************************/

static inline uint32_t cdcncm_get(FAR uint8_t **address, size_t size)
{
  uint32_t value;

  switch (size)
  {
    case 2:
      value = GETUINT16(*address);
      break;
    case 4:
      value = GETUINT32(*address);
      break;
    default:
      nerr("Wrong size cdcncm_get %zu\n", size);
  }

  *address += size;
  return value;
}

/****************************************************************************
 * Name: cdcncm_put
 *
 * Description:
 *   Write size length data to address and increases the address by the
 *   corresponding size
 *
 * Input Parameters:
 *   address - Pointer to address
 *   size    - Size of data
 *   value   - Value of data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline
void cdcncm_put(FAR uint8_t **address, size_t size, uint32_t value)
{
  switch (size)
  {
    case 2:
      PUTUINT16(*address, value);
      break;
    case 4:
      PUTUINT32(*address, value);
      break;
    default:
      uerr("Wrong cdcncm_put\n");
  }

  *address += size;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcncm_transmit_format
 *
 * Description:
 *   Format the data to be transmitted to the host in the format specified by
 *   the NCM protocol (Network Control Model) and the NCM NTB (Network
 *   Transfer Block) format.
 *
 * Input Parameters:
 *   self - Reference to the driver state structure
 *   pkt  - Reference to the packet to be transmitted
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cdcncm_transmit_format(FAR struct cdcncm_driver_s *self,
                                   FAR netpkt_t *pkt)
{
  FAR const struct ndp_parser_opts_s *opts = self->parseropts;
  unsigned int dglen = netpkt_getdatalen(&self->dev, pkt);
  const int div = g_ntbparameters.ndpindivisor;
  const int rem = g_ntbparameters.ndpinpayloadremainder;
  const int dgramidxlen = 2 * opts->dgramitemlen;
  const int ndpalign = g_ntbparameters.ndpinalignment;
  FAR uint8_t *tmp;
  int ncblen;
  int ndpindex;

  ncblen   = opts->nthsize;
  ndpindex = NCM_ALIGN(ncblen, ndpalign);

  if (self->dgramcount == 0)
    {
      /* Fill NCB */

      tmp = self->wrreq->buf;
      memset(tmp, 0, ncblen);
      cdcncm_put(&tmp, 4, opts->nthsign);
      cdcncm_put(&tmp, 2, opts->nthsize);
      tmp += 2;              /* Skip seq */
      tmp += opts->blocklen; /* Skip block len */
      cdcncm_put(&tmp, opts->ndpindex, ndpindex);
      self->dgramaddr = self->wrreq->buf + ndpindex +
                        opts->ndpsize + (TX_MAX_NUM_DPE + 1) * dgramidxlen;
      self->dgramaddr = (FAR uint8_t *)NCM_ALIGN((uintptr_t)self->dgramaddr,
                                                 div) + rem;

      /* Fill NDP */

      tmp = self->wrreq->buf + ndpindex;
      cdcncm_put(&tmp, 4, self->ndpsign);
      tmp += 2 + opts->reserved1;
      cdcncm_put(&tmp, opts->nextndpindex, 0);
    }

  tmp = self->wrreq->buf + ndpindex + opts->ndpsize +
        self->dgramcount * dgramidxlen;
  cdcncm_put(&tmp, opts->dgramitemlen, self->dgramaddr - self->wrreq->buf);
  cdcncm_put(&tmp, opts->dgramitemlen, dglen);

  /* Fill IP packet */

  netpkt_copyout(&self->dev, self->dgramaddr, pkt, dglen, 0);

  self->dgramaddr += dglen;
  self->dgramaddr  = (FAR uint8_t *)NCM_ALIGN((uintptr_t)self->dgramaddr,
                                              div) + rem;

  self->dgramcount++;
}

/****************************************************************************
 * Name: cdcncm_transmit_work
 *
 * Description:
 *   Send NTB to the USB device for ethernet frame transmission
 *
 * Input Parameters:
 *   arg - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cdcncm_transmit_work(FAR void *arg)
{
  FAR struct cdcncm_driver_s *self = arg;
  FAR const struct ndp_parser_opts_s *opts = self->parseropts;
  FAR uint8_t *tmp;
  const int dgramidxlen = 2 * opts->dgramitemlen;
  const int ndpalign = g_ntbparameters.ndpinalignment;
  int ncblen;
  int ndpindex;
  int totallen;

  /* Wait until the USB device request for Ethernet frame transmissions
   * becomes available.
   */

  while (nxsem_wait(&self->wrreq_idle) != OK)
    {
    }

  ncblen   = opts->nthsize;
  ndpindex = NCM_ALIGN(ncblen, ndpalign);

  /* Fill NCB */

  tmp      = self->wrreq->buf + 8; /* Offset to block length */
  totallen = self->dgramaddr - self->wrreq->buf;
  cdcncm_put(&tmp, opts->blocklen, totallen);

  /* Fill NDP */

  tmp = self->wrreq->buf + ndpindex + 4; /* Offset to ndp length */
  cdcncm_put(&tmp, 2, opts->ndpsize + (self->dgramcount + 1) * dgramidxlen);

  tmp += opts->reserved1 + opts->nextndpindex + opts->reserved2 +
         self->dgramcount * dgramidxlen;
  self->dgramcount = 0;

  cdcncm_put(&tmp, opts->dgramitemlen, 0);
  cdcncm_put(&tmp, opts->dgramitemlen, 0);

  self->wrreq->len = totallen;

  EP_SUBMIT(self->epbulkin, self->wrreq);
}

/****************************************************************************
 * Name: cdcncm_packet_handler
 *
 * Description:
 *   Sends a single complete packet to the protocol stack
 *
 * Input Parameters:
 *   self - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int cdcncm_packet_handler(FAR struct cdcncm_driver_s *self,
                                 FAR uint8_t *dgram, uint32_t dglen)
{
  FAR netpkt_t *pkt = netpkt_alloc(&self->dev, NETPKT_RX);
  int ret = -ENOMEM;

  if (pkt == NULL)
    {
      return ret;
    }

  ret = netpkt_copyin(&self->dev, pkt, dgram, dglen, 0);
  if (ret < 0)
    {
      netpkt_free(&self->dev, pkt, NETPKT_RX);
      return ret;
    }

  ret = netpkt_tryadd_queue(pkt, &self->rx_queue);
  if (ret != 0)
    {
      netpkt_free(&self->dev, pkt, NETPKT_RX);
    }

  return ret;
}

/****************************************************************************
 * Name: cdcncm_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   self - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cdcncm_receive(FAR struct cdcncm_driver_s *self)
{
  FAR const struct ndp_parser_opts_s *opts = self->parseropts;
  FAR uint8_t *tmp = self->rdreq->buf;
  uint32_t ntbmax = g_ntbparameters.ntboutmaxsize;
  uint32_t blocklen;
  uint32_t ndplen;
  int ndpindex;
  int dgramcounter;

  /* Get signature */

  if (GETUINT32(tmp) != opts->nthsign)
    {
      uerr("Wrong NTH SIGN, skblen %d\n", self->rdreq->xfrd);
      return;
    }

  tmp += 4;

  /* Get header len */

  if (GETUINT16(tmp) != opts->nthsize)
    {
      uerr("Wrong NTB headersize\n");
      return;
    }

  tmp += 4; /* Skip header len and seq */

  blocklen = cdcncm_get(&tmp, opts->blocklen);

  /* Get block len */

  if (blocklen > ntbmax)
    {
      uerr("OUT size exceeded\n");
      return;
    }

  ndpindex = cdcncm_get(&tmp, opts->ndpindex);

  do
    {
      uint32_t index;
      uint32_t dglen;

      if (((ndpindex % 4) != 0) || (ndpindex < opts->nthsize) ||
          (ndpindex > (blocklen - opts->ndpsize)))
        {
          uerr("Bad index: %#X\n", ndpindex);
          return;
        }

      tmp = self->rdreq->buf + ndpindex;

      if (GETUINT32(tmp) != self->ndpsign)
        {
          uerr("Wrong NDP SIGN\n");
          return;
        }

      tmp   += 4;
      ndplen = cdcncm_get(&tmp, 2);

      if ((ndplen < opts->ndpsize + 2 * (opts->dgramitemlen * 2)) ||
          (ndplen % opts->ndpalign != 0))
        {
          uerr("Bad NDP length: %x\n", ndplen);
          return;
        }

      tmp         += opts->reserved1;
      ndpindex     = cdcncm_get(&tmp, opts->nextndpindex);
      tmp         += opts->reserved2;

      ndplen      -= opts->ndpsize;
      dgramcounter = 0;
      do
        {
          index = cdcncm_get(&tmp, opts->dgramitemlen);
          dglen = cdcncm_get(&tmp, opts->dgramitemlen);

          /* TODO: support CRC */

          /* Check if the packet is a valid size for the network buffer
           * configuration.
           */

          if (index == 0 || dglen == 0)
            {
              break;
            }

          dgramcounter++;

          /* Copy the data from the hardware to self->rx_queue. */

          cdcncm_packet_handler(self, self->rdreq->buf + index, dglen);

          ndplen -= 2 * (opts->dgramitemlen);
        }
      while (ndplen > 2 * (opts->dgramitemlen));
    }
  while (ndpindex);
}

/****************************************************************************
 * Name: cdcncm_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cdcncm_txdone(FAR struct cdcncm_driver_s *priv)
{
  /* In any event, poll the network for new TX data */

  netdev_lower_txdone(&priv->dev);
}

/****************************************************************************
 * Name: cdcncm_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs on a worker thread.
 *
 ****************************************************************************/

static void cdcncm_interrupt_work(FAR void *arg)
{
  FAR struct cdcncm_driver_s *self = (FAR struct cdcncm_driver_s *)arg;
  irqstate_t flags;

  /* Check if we received an incoming packet, if so, call cdcncm_receive() */

  if (self->rxpending)
    {
      cdcncm_receive(self);
      netdev_lower_rxready(&self->dev);

      flags = enter_critical_section();
      self->rxpending = false;
      EP_SUBMIT(self->epbulkout, self->rdreq);
      leave_critical_section(flags);
    }

  /* Check if a packet transmission just completed.  If so, call
   * cdcncm_txdone. This may disable further Tx interrupts if there
   * are no pending transmissions.
   */

  flags = enter_critical_section();
  if (self->txdone)
    {
      self->txdone = false;
      leave_critical_section(flags);

      cdcncm_txdone(self);
    }
  else
    {
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: cdcncm_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int cdcncm_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct cdcncm_driver_s *priv =
    container_of(dev, struct cdcncm_driver_s, dev);

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %u.%u.%u.%u\n",
        ip4_addr1(dev->netdev.d_ipaddr), ip4_addr2(dev->netdev.d_ipaddr),
        ip4_addr3(dev->netdev.d_ipaddr), ip4_addr4(dev->netdev.d_ipaddr));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->netdev.d_ipv6addr[0], dev->netdev.d_ipv6addr[1],
        dev->netdev.d_ipv6addr[2], dev->netdev.d_ipv6addr[3],
        dev->netdev.d_ipv6addr[4], dev->netdev.d_ipv6addr[5],
        dev->netdev.d_ipv6addr[6], dev->netdev.d_ipv6addr[7]);
#endif

  priv->bifup = true;
  return OK;
}

/****************************************************************************
 * Name: cdcncm_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int cdcncm_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct cdcncm_driver_s *priv =
    container_of(dev, struct cdcncm_driver_s, dev);
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the cdcncm_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->bifup = false;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: cdcncm_send
 *
 * Description:
 *   Transmit a packet through the USB interface
 *
 * Input Parameters:
 *   dev - Reference to the NuttX netdev lowerhalf driver structure
 *   pkt - The packet to be sent
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static int cdcncm_send(FAR struct netdev_lowerhalf_s *dev, FAR netpkt_t *pkt)
{
  FAR struct cdcncm_driver_s *self;

  self = container_of(dev, struct cdcncm_driver_s, dev);
  cdcncm_transmit_format(self, pkt);
  netpkt_free(dev, pkt, NETPKT_TX);

  if ((self->wrreq->buf + NTB_OUT_SIZE - self->dgramaddr <
       self->dev.netdev.d_pktsize) || self->dgramcount >= TX_MAX_NUM_DPE)
    {
      work_cancel(ETHWORK, &self->delaywork);
      cdcncm_transmit_work(self);
    }
  else
    {
      work_queue(ETHWORK, &self->delaywork, cdcncm_transmit_work, self,
                 MSEC2TICK(CDCNCM_DGRAM_COMBINE_PERIOD));
    }

  return OK;
}

/****************************************************************************
 * Name: cdcncm_recv
 *
 * Description:
 *   Receive a packet from the USB interface
 *
 * Input Parameters:
 *   dev - Reference to the NuttX netdev lowerhalf driver structure
 *
 * Returned Value:
 *   The received packet, or NULL if no packet is available
 *
 ****************************************************************************/

static FAR netpkt_t *cdcncm_recv(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct cdcncm_driver_s *self;
  FAR netpkt_t *pkt;

  self = container_of(dev, struct cdcncm_driver_s, dev);
  pkt  = netpkt_remove_queue(&self->rx_queue);
  return pkt;
}

/****************************************************************************
 * Name: cdcncm_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int cdcncm_addmac(FAR struct netdev_lowerhalf_s *dev,
                         FAR const uint8_t *mac)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: cdcncm_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int cdcncm_rmmac(FAR struct netdev_lowerhalf_s *dev,
                        FAR const uint8_t *mac)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: cdcncm_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int cdcncm_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                        unsigned long arg)
{
  return -ENOTTY;
}
#endif

/****************************************************************************
 * USB Device Class Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: cdcncm_ep0incomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void cdcncm_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                 FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      uerr("result: %hd, xfrd: %hu\n", req->result, req->xfrd);
    }
}

/****************************************************************************
 * Name: cdcncm_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.
 *
 ****************************************************************************/

static void cdcncm_rdcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
  FAR struct cdcncm_driver_s *self = (FAR struct cdcncm_driver_s *)ep->priv;

  uinfo("buf: %p, flags 0x%hhx, len %hu, xfrd %hu, result %hd\n",
        req->buf, req->flags, req->len, req->xfrd, req->result);

  switch (req->result)
    {
      case 0:  /* Normal completion */
        {
          DEBUGASSERT(!self->rxpending);
          self->rxpending = true;
          work_queue(ETHWORK, &self->irqwork,
                     cdcncm_interrupt_work, self, 0);
        }
        break;

      case -ESHUTDOWN:  /* Disconnection */
        break;

      default: /* Some other error occurred */
        {
          uerr("req->result: %hd\n", req->result);
          EP_SUBMIT(self->epbulkout, self->rdreq);
        }
        break;
    }
}

/****************************************************************************
 * Name: cdcncm_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void cdcncm_wrcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
  FAR struct cdcncm_driver_s *self = (FAR struct cdcncm_driver_s *)ep->priv;
  int rc;

  uinfo("buf: %p, flags 0x%hhx, len %hu, xfrd %hu, result %hd\n",
        req->buf, req->flags, req->len, req->xfrd, req->result);

  /* The single USB device write request is available for upcoming
   * transmissions again.
   */

  rc = nxsem_post(&self->wrreq_idle);

  if (rc != OK)
    {
      nerr("nxsem_post failed! rc: %d\n", rc);
    }

  /* Inform the network layer that an Ethernet frame was transmitted. */

  self->txdone = true;
  work_queue(ETHWORK, &self->irqwork, cdcncm_interrupt_work, self, 0);
}

/****************************************************************************
 * Name: cdcncm_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void cdcncm_resetconfig(FAR struct cdcncm_driver_s *self)
{
  /* Are we configured? */

  if (self->config != CDCECM_CONFIGID_NONE)
    {
      /* Yes.. but not anymore */

      self->config = CDCECM_CONFIGID_NONE;

      /* Inform the networking layer that the link is down */

      cdcncm_ifdown(&self->dev);

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

      EP_DISABLE(self->epint);
      EP_DISABLE(self->epbulkin);
      EP_DISABLE(self->epbulkout);
    }

  self->parseropts = &g_ndp16_opts;
  self->ndpsign    = self->parseropts->ndpsign;
  self->notify     = NCM_NOTIFY_NONE;
}

/****************************************************************************
 * Name: cdcncm_setconfig
 *
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

static int cdcncm_setconfig(FAR struct cdcncm_driver_s *self, uint8_t config)
{
  struct usb_epdesc_s epdesc;
  bool is_high_speed = self->usbdev.speed == USB_SPEED_HIGH;
  int ret;

  if (config == self->config)
    {
      return OK;
    }

  cdcncm_resetconfig(self);

  if (config == CDCECM_CONFIGID_NONE)
    {
      return OK;
    }

  if (config != CDCECM_CONFIGID)
    {
      return -EINVAL;
    }

  cdcncm_mkepdesc(CDCNCM_EP_INTIN_IDX, &epdesc, &self->devinfo, false);
  ret = EP_CONFIGURE(self->epint, &epdesc, false);

  if (ret < 0)
    {
      goto error;
    }

  self->epint->priv = self;
  cdcncm_mkepdesc(CDCNCM_EP_BULKIN_IDX,
                  &epdesc, &self->devinfo, is_high_speed);
  ret = EP_CONFIGURE(self->epbulkin, &epdesc, false);

  if (ret < 0)
    {
      goto error;
    }

  self->epbulkin->priv = self;

  cdcncm_mkepdesc(CDCNCM_EP_BULKOUT_IDX,
                  &epdesc, &self->devinfo, is_high_speed);
  ret = EP_CONFIGURE(self->epbulkout, &epdesc, true);

  if (ret < 0)
    {
      goto error;
    }

  self->epbulkout->priv = self;

  /* Queue read requests in the bulk OUT endpoint */

  DEBUGASSERT(!self->rxpending);

  self->rdreq->callback = cdcncm_rdcomplete,
  ret = EP_SUBMIT(self->epbulkout, self->rdreq);
  if (ret != OK)
    {
      uerr("EP_SUBMIT failed. ret %d\n", ret);
      goto error;
    }

  /* We are successfully configured */

  self->config = config;

  /* Set client's MAC address */

  memcpy(self->dev.netdev.d_mac.ether.ether_addr_octet,
         "\x00\xe0\xde\xad\xbe\xef", IFHWADDRLEN);

  /* Report link up to networking layer */

  if (cdcncm_ifup(&self->dev) == OK)
    {
      self->dev.netdev.d_flags |= IFF_UP;
    }

  return OK;

error:
  cdcncm_resetconfig(self);
  return ret;
}

/****************************************************************************
 * Name: ncm_notify
 *
 ****************************************************************************/

static int ncm_notify(FAR struct cdcncm_driver_s *self)
{
  FAR struct usb_ctrlreq_s *req =
                            (FAR struct usb_ctrlreq_s *)self->notifyreq->buf;
  int ret = 0;

  switch (self->notify)
    {
      case NCM_NOTIFY_NONE:
        return ret;

      case NCM_NOTIFY_CONNECT:

        /* Notifying the host of the NIC modification status */

        req->req      = NCM_NETWORK_CONNECTION;
        req->value[0] = LSBYTE(IFF_IS_RUNNING(self->dev.netdev.d_flags));
        req->value[1] = MSBYTE(IFF_IS_RUNNING(self->dev.netdev.d_flags));
        req->len[0]   = 0;
        req->len[1]   = 0;
        ret           = sizeof(*req);

        self->notify  = NCM_NOTIFY_NONE;
        break;

      case NCM_NOTIFY_SPEED:
        {
          FAR uint32_t *data;

          req->req      = NCM_SPEED_CHANGE;
          req->value[0] = LSBYTE(0);
          req->value[1] = MSBYTE(0);
          req->len[0]   = LSBYTE(8);
          req->len[1]   = MSBYTE(8);

          /* SPEED_CHANGE data is up/down speeds in bits/sec */

          data    = (FAR uint32_t *)(self->notifyreq->buf + sizeof(*req));
          data[0] = self->usbdev.speed == USB_SPEED_HIGH ?
                    CDCECM_HIGH_BITRATE : CDCECM_LOW_BITRATE;
          data[1] = data[0];
          ret     = sizeof(*req) + 8;

          self->notify = NCM_NOTIFY_CONNECT;
          break;
        }
      }

  req->type = 0xa1;
  req->index[0] = LSBYTE(self->devinfo.ifnobase);
  req->index[1] = MSBYTE(self->devinfo.ifnobase);

  return ret;
}

/****************************************************************************
 * Name: ncm_do_notify
 *
 ****************************************************************************/

static void ncm_do_notify(FAR void *arg)
{
  FAR struct cdcncm_driver_s *self = arg;
  int ret;

  while (self->notify != NCM_NOTIFY_NONE)
    {
      ret = ncm_notify(self);
      if (ret > 0)
        {
          FAR struct usbdev_req_s *notifyreq = self->notifyreq;

          notifyreq->len   = ret;
          notifyreq->flags = USBDEV_REQFLAGS_NULLPKT;

          EP_SUBMIT(self->epint, notifyreq);
        }
    }
}

/****************************************************************************
 * Name: cdcncm_setinterface
 *
 ****************************************************************************/

static int cdcncm_setinterface(FAR struct cdcncm_driver_s *self,
                               uint16_t interface, uint16_t altsetting)
{
  if (interface == self->devinfo.ifnobase + 1)
    {
      if (altsetting)
        {
          self->notify = NCM_NOTIFY_SPEED;
        }

      netdev_lower_carrier_on(&self->dev);
      work_queue(ETHWORK, &self->notifywork, ncm_do_notify, self,
                 MSEC2TICK(100));
    }
  else
    {
      uerr("invailid interface %d\n", interface);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: cdcncm_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

static int cdcncm_mkstrdesc(uint8_t id, FAR struct usb_strdesc_s *strdesc)
{
  FAR uint8_t *data = (FAR uint8_t *)(strdesc + 1);
  FAR const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_CDCNCM_COMPOSITE
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len  = 4;
        strdesc->type = USB_DESC_TYPE_STRING;
        data[0] = LSBYTE(CDCNCM_STR_LANGUAGE);
        data[1] = MSBYTE(CDCNCM_STR_LANGUAGE);
        return 4;
      }

    case CDCNCM_MANUFACTURERSTRID:
      str = CONFIG_CDCNCM_VENDORSTR;
      break;

    case CDCNCM_PRODUCTSTRID:
      str = CONFIG_CDCNCM_PRODUCTSTR;
      break;

    case CDCNCM_SERIALSTRID:
#ifdef CONFIG_BOARD_USBDEV_SERIALSTR
      str = board_usbdev_serialstr();
#else
      str = "0";
#endif
      break;

    case CDCNCM_CONFIGSTRID:
      str = "Default";
      break;
#endif

    case CDCECM_MACSTRID:
      str = "020000112233";
      break;

    default:
      uerr("Unknown string descriptor index: %d\n", id);
      return -EINVAL;
    }

  /* The string is utf16-le.  The poor man's utf-8 to utf16-le
   * conversion below will only handle 7-bit en-us ascii
   */

  len = strlen(str);
  if (len > (CDCECM_MAXSTRLEN / 2))
    {
      len = (CDCECM_MAXSTRLEN / 2);
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
 * Name: cdcncm_mkepdesc
 *
 * Description:
 *   Construct the endpoint descriptor
 *
 ****************************************************************************/

static void cdcncm_mkepdesc(int epidx, FAR struct usb_epdesc_s *epdesc,
                            FAR struct usbdev_devinfo_s *devinfo,
                            bool hispeed)
{
  uint16_t intin_mxpktsz   = CONFIG_CDCNCM_EPINTIN_FSSIZE;
  uint16_t bulkout_mxpktsz = CONFIG_CDCNCM_EPBULKOUT_FSSIZE;
  uint16_t bulkin_mxpktsz  = CONFIG_CDCNCM_EPBULKIN_FSSIZE;

#ifdef CONFIG_USBDEV_DUALSPEED
  if (hispeed)
    {
      intin_mxpktsz   = CONFIG_CDCNCM_EPINTIN_HSSIZE;
      bulkout_mxpktsz = CONFIG_CDCNCM_EPBULKOUT_HSSIZE;
      bulkin_mxpktsz  = CONFIG_CDCNCM_EPBULKIN_HSSIZE;
    }
#else
  UNUSED(hispeed);
#endif

  epdesc->len  = USB_SIZEOF_EPDESC;      /* Descriptor length */
  epdesc->type = USB_DESC_TYPE_ENDPOINT; /* Descriptor type */

  switch (epidx)
    {
      case CDCNCM_EP_INTIN_IDX: /* Interrupt IN endpoint */
        {
          epdesc->addr            = USB_DIR_IN |
                                    devinfo->epno[CDCNCM_EP_INTIN_IDX];
          epdesc->attr            = USB_EP_ATTR_XFER_INT;
          epdesc->mxpacketsize[0] = LSBYTE(intin_mxpktsz);
          epdesc->mxpacketsize[1] = MSBYTE(intin_mxpktsz);
          epdesc->interval        = 5;
        }
        break;

      case CDCNCM_EP_BULKIN_IDX:
        {
          epdesc->addr            = USB_DIR_IN |
                                    devinfo->epno[CDCNCM_EP_BULKIN_IDX];
          epdesc->attr            = USB_EP_ATTR_XFER_BULK;
          epdesc->mxpacketsize[0] = LSBYTE(bulkin_mxpktsz);
          epdesc->mxpacketsize[1] = MSBYTE(bulkin_mxpktsz);
          epdesc->interval        = 0;
        }
        break;

      case CDCNCM_EP_BULKOUT_IDX:
        {
          epdesc->addr            = USB_DIR_OUT |
                                    devinfo->epno[CDCNCM_EP_BULKOUT_IDX];
          epdesc->attr            = USB_EP_ATTR_XFER_BULK;
          epdesc->mxpacketsize[0] = LSBYTE(bulkout_mxpktsz);
          epdesc->mxpacketsize[1] = MSBYTE(bulkout_mxpktsz);
          epdesc->interval        = 0;
        }
        break;

      default:
        DEBUGPANIC();
    }
}

/****************************************************************************
 * Name: cdcncm_mkcfgdesc
 *
 * Description:
 *   Construct the config descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
static int16_t cdcncm_mkcfgdesc(FAR uint8_t *desc,
                                FAR struct usbdev_devinfo_s *devinfo,
                                uint8_t speed, uint8_t type)
#else
static int16_t cdcncm_mkcfgdesc(FAR uint8_t *desc,
                                FAR struct usbdev_devinfo_s *devinfo)
#endif
{
  FAR struct usb_cfgdesc_s *cfgdesc = NULL;
  int16_t len = 0;
  bool is_high_speed = false;

#ifdef CONFIG_USBDEV_DUALSPEED
  is_high_speed = (speed == USB_SPEED_HIGH);

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      is_high_speed = !is_high_speed;
    }
#endif

#ifndef CONFIG_CDCNCM_COMPOSITE
  if (desc)
    {
      cfgdesc = (FAR struct usb_cfgdesc_s *)desc;
      cfgdesc->len         = USB_SIZEOF_CFGDESC;
      cfgdesc->type        = USB_DESC_TYPE_CONFIG;
      cfgdesc->ninterfaces = CDCECM_NINTERFACES;
      cfgdesc->cfgvalue    = CDCECM_CONFIGID;
      cfgdesc->icfg        = devinfo->strbase + CDCNCM_CONFIGSTRID;
      cfgdesc->attr        = USB_CONFIG_ATTR_ONE | CDCNCM_SELFPOWERED |
                             CDCNCM_REMOTEWAKEUP;
      cfgdesc->mxpower     = (CONFIG_USBDEV_MAXPOWER + 1) / 2;

      desc += USB_SIZEOF_CFGDESC;
    }

  len += USB_SIZEOF_CFGDESC;

#elif defined(CONFIG_COMPOSITE_IAD)

  /* Interface association descriptor */

  if (desc)
    {
      FAR struct usb_iaddesc_s *iaddesc;

      iaddesc = (FAR struct usb_iaddesc_s *)desc;
      iaddesc->len       = USB_SIZEOF_IADDESC;                  /* Descriptor length */
      iaddesc->type      = USB_DESC_TYPE_INTERFACEASSOCIATION;  /* Descriptor type */
      iaddesc->firstif   = devinfo->ifnobase;                   /* Number of first interface of the function */
      iaddesc->nifs      = devinfo->ninterfaces;                /* Number of interfaces associated with the function */
      iaddesc->classid   = USB_CLASS_CDC;                       /* Class code */
      iaddesc->subclass  = CDC_SUBCLASS_NCM;                    /* Sub-class code */
      iaddesc->protocol  = CDC_PROTO_NONE;                      /* Protocol code */
      iaddesc->ifunction = 0;                                   /* Index to string identifying the function */

      desc += USB_SIZEOF_IADDESC;
    }

  len += USB_SIZEOF_IADDESC;
#endif

  /* Communications Class Interface */

  if (desc)
    {
      FAR struct usb_ifdesc_s *ifdesc;

      ifdesc = (FAR struct usb_ifdesc_s *)desc;
      ifdesc->len      = USB_SIZEOF_IFDESC;
      ifdesc->type     = USB_DESC_TYPE_INTERFACE;
      ifdesc->ifno     = devinfo->ifnobase;
      ifdesc->alt      = 0;
      ifdesc->neps     = 1;
      ifdesc->classid  = USB_CLASS_CDC;
      ifdesc->subclass = CDC_SUBCLASS_NCM;
      ifdesc->protocol = CDC_PROTO_NONE;
      ifdesc->iif      = 0;

      desc += USB_SIZEOF_IFDESC;
    }

  len += USB_SIZEOF_IFDESC;

  if (desc)
    {
      FAR struct cdc_hdr_funcdesc_s *hdrdesc;

      hdrdesc = (FAR struct cdc_hdr_funcdesc_s *)desc;
      hdrdesc->size    = SIZEOF_HDR_FUNCDESC;
      hdrdesc->type    = USB_DESC_TYPE_CSINTERFACE;
      hdrdesc->subtype = CDC_DSUBTYPE_HDR;
      hdrdesc->cdc[0]  = LSBYTE(0x0110);
      hdrdesc->cdc[1]  = MSBYTE(0x0110);

      desc += SIZEOF_HDR_FUNCDESC;
    }

  len += SIZEOF_HDR_FUNCDESC;

  if (desc)
    {
      FAR struct cdc_union_funcdesc_s *uniondesc;

      uniondesc = (FAR struct cdc_union_funcdesc_s *)desc;
      uniondesc->size     = SIZEOF_UNION_FUNCDESC(1);
      uniondesc->type     = USB_DESC_TYPE_CSINTERFACE;
      uniondesc->subtype  = CDC_DSUBTYPE_UNION;
      uniondesc->master   = devinfo->ifnobase;
      uniondesc->slave[0] = devinfo->ifnobase + 1;

      desc += SIZEOF_UNION_FUNCDESC(1);
    }

  len += SIZEOF_UNION_FUNCDESC(1);

  if (desc)
    {
      FAR struct cdc_ecm_funcdesc_s *ecmdesc;

      ecmdesc = (FAR struct cdc_ecm_funcdesc_s *)desc;
      ecmdesc->size       = SIZEOF_ECM_FUNCDESC;
      ecmdesc->type       = USB_DESC_TYPE_CSINTERFACE;
      ecmdesc->subtype    = CDC_DSUBTYPE_ECM;
      ecmdesc->mac        = devinfo->strbase + CDCECM_MACSTRID;
      ecmdesc->stats[0]   = 0;
      ecmdesc->stats[1]   = 0;
      ecmdesc->stats[2]   = 0;
      ecmdesc->stats[3]   = 0;
      ecmdesc->maxseg[0]  = LSBYTE(CONFIG_NET_ETH_PKTSIZE);
      ecmdesc->maxseg[1]  = MSBYTE(CONFIG_NET_ETH_PKTSIZE);
      ecmdesc->nmcflts[0] = LSBYTE(0);
      ecmdesc->nmcflts[1] = MSBYTE(0);
      ecmdesc->npwrflts   = 0;

      desc += SIZEOF_ECM_FUNCDESC;
    }

  len += SIZEOF_ECM_FUNCDESC;

  if (desc)
    {
      FAR struct cdc_ncm_funcdesc_s *ncmdesc;

      ncmdesc = (FAR struct cdc_ncm_funcdesc_s *)desc;
      ncmdesc->size       = SIZEOF_NCM_FUNCDESC;
      ncmdesc->type       = USB_DESC_TYPE_CSINTERFACE;
      ncmdesc->subtype    = CDC_DSUBTYPE_NCM;
      ncmdesc->version[0] = LSBYTE(CDCECM_VERSIONNO);
      ncmdesc->version[1] = MSBYTE(CDCECM_VERSIONNO);
      ncmdesc->netcaps    = NCAPS;

      desc += SIZEOF_NCM_FUNCDESC;
    }

  len += SIZEOF_NCM_FUNCDESC;

  if (desc)
    {
      FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)desc;

      cdcncm_mkepdesc(CDCNCM_EP_INTIN_IDX, epdesc, devinfo, false);
      desc += USB_SIZEOF_EPDESC;
    }

  len += USB_SIZEOF_EPDESC;

  /* Data Class Interface */

  if (desc)
    {
      FAR struct usb_ifdesc_s *ifdesc;

      ifdesc = (FAR struct usb_ifdesc_s *)desc;
      ifdesc->len      = USB_SIZEOF_IFDESC;
      ifdesc->type     = USB_DESC_TYPE_INTERFACE;
      ifdesc->ifno     = devinfo->ifnobase + 1;
      ifdesc->alt      = 0;
      ifdesc->neps     = 0;
      ifdesc->classid  = USB_CLASS_CDC_DATA;
      ifdesc->subclass = 0;
      ifdesc->protocol = CDC_DATA_PROTO_NCMNTB;
      ifdesc->iif      = 0;

      desc += USB_SIZEOF_IFDESC;
    }

  len += USB_SIZEOF_IFDESC;

  if (desc)
    {
      FAR struct usb_ifdesc_s *ifdesc;

      ifdesc = (FAR struct usb_ifdesc_s *)desc;
      ifdesc->len      = USB_SIZEOF_IFDESC;
      ifdesc->type     = USB_DESC_TYPE_INTERFACE;
      ifdesc->ifno     = devinfo->ifnobase + 1;
      ifdesc->alt      = 1;
      ifdesc->neps     = 2;
      ifdesc->classid  = USB_CLASS_CDC_DATA;
      ifdesc->subclass = 0;
      ifdesc->protocol = CDC_DATA_PROTO_NCMNTB;
      ifdesc->iif      = 0;

      desc += USB_SIZEOF_IFDESC;
    }

  len += USB_SIZEOF_IFDESC;

  if (desc)
    {
      FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)desc;

      cdcncm_mkepdesc(CDCNCM_EP_BULKIN_IDX, epdesc, devinfo, is_high_speed);
      desc += USB_SIZEOF_EPDESC;
    }

  len += USB_SIZEOF_EPDESC;

  if (desc)
    {
      FAR struct usb_epdesc_s *epdesc = (FAR struct usb_epdesc_s *)desc;

      cdcncm_mkepdesc(CDCNCM_EP_BULKOUT_IDX, epdesc, devinfo, is_high_speed);
      desc += USB_SIZEOF_EPDESC;
    }

  len += USB_SIZEOF_EPDESC;

  if (cfgdesc)
    {
      cfgdesc->totallen[0] = LSBYTE(len);
      cfgdesc->totallen[1] = MSBYTE(len);
    }

  DEBUGASSERT(len <= CDCECM_MXDESCLEN);
  return len;
}

/****************************************************************************
 * Name: cdcncm_getdescriptor
 *
 * Description:
 *   Copy the USB CDC-ECM Device USB Descriptor of a given Type and a given
 *   Index into the provided Descriptor Buffer.
 *
 * Input Parameter:
 *   drvr  - The USB Device Fuzzer Driver instance.
 *   type  - The Type of USB Descriptor requested.
 *   index - The Index of the USB Descriptor requested.
 *   desc  - The USB Descriptor is copied into this buffer, which must be at
 *           least CDCECM_MXDESCLEN bytes wide.
 *
 * Returned Value:
 *   The size in bytes of the requested USB Descriptor or a negated errno in
 *   case of failure.
 *
 ****************************************************************************/

static int cdcncm_getdescriptor(FAR struct cdcncm_driver_s *self,
                                uint8_t type, uint8_t index, FAR void *desc)
{
  switch (type)
    {
#ifndef CONFIG_CDCNCM_COMPOSITE
    case USB_DESC_TYPE_DEVICE:
      {
        memcpy(desc, &g_devdesc, sizeof(g_devdesc));
        return (int)sizeof(g_devdesc);
      }
      break;
#endif

#ifdef CONFIG_USBDEV_DUALSPEED
    case USB_DESC_TYPE_OTHERSPEEDCONFIG:
#endif /* CONFIG_USBDEV_DUALSPEED */
    case USB_DESC_TYPE_CONFIG:
      {
#ifdef CONFIG_USBDEV_DUALSPEED
        return cdcncm_mkcfgdesc((FAR uint8_t *)desc, &self->devinfo,
                                self->usbdev.speed, type);
#else
        return cdcncm_mkcfgdesc((FAR uint8_t *)desc, &self->devinfo);
#endif
      }
      break;

    case USB_DESC_TYPE_STRING:
      {
        return cdcncm_mkstrdesc(index, (FAR struct usb_strdesc_s *)desc);
      }
      break;

    default:
      uerr("Unsupported descriptor type: 0x%02hhx\n", type);
      break;
    }

  return -ENOTSUP;
}

/****************************************************************************
 * USB Device Class Methods
 ****************************************************************************/

/****************************************************************************
 * Name: cdcncm_bind
 *
 * Description:
 *   Invoked when the driver is bound to an USB device
 *
 ****************************************************************************/

static int cdcncm_bind(FAR struct usbdevclass_driver_s *driver,
                       FAR struct usbdev_s *dev)
{
  FAR struct cdcncm_driver_s *self = (FAR struct cdcncm_driver_s *)driver;
  int ret = OK;

  uinfo("\n");

#ifndef CONFIG_CDCNCM_COMPOSITE
  dev->ep0->priv = self;
#endif

  /* Preallocate control request */

  self->ctrlreq = usbdev_allocreq(dev->ep0, CDCECM_MXDESCLEN);

  if (self->ctrlreq == NULL)
    {
      ret = -ENOMEM;
      goto error;
    }

  self->ctrlreq->callback = cdcncm_ep0incomplete;

  self->epint     = DEV_ALLOCEP(dev,
                                USB_DIR_IN |
                                self->devinfo.epno[CDCNCM_EP_INTIN_IDX],
                                true, USB_EP_ATTR_XFER_INT);
  self->epbulkin  = DEV_ALLOCEP(dev,
                                USB_DIR_IN |
                                self->devinfo.epno[CDCNCM_EP_BULKIN_IDX],
                                true, USB_EP_ATTR_XFER_BULK);
  self->epbulkout = DEV_ALLOCEP(dev,
                                USB_DIR_OUT |
                                self->devinfo.epno[CDCNCM_EP_BULKOUT_IDX],
                                false, USB_EP_ATTR_XFER_BULK);

  if (!self->epint || !self->epbulkin || !self->epbulkout)
    {
      uerr("Failed to allocate endpoints!\n");
      ret = -ENODEV;
      goto error;
    }

  self->epint->priv     = self;
  self->epbulkin->priv  = self;
  self->epbulkout->priv = self;

  /* Pre-allocate notify requests. The buffer size is CDCECM_MXDESCLEN. */

  self->notifyreq = usbdev_allocreq(self->epint, CDCECM_MXDESCLEN);
  if (self->notifyreq == NULL)
    {
      uerr("Out of memory\n");
      ret = -ENOMEM;
      goto error;
    }

  self->notifyreq->callback = cdcncm_wrcomplete;

  /* Pre-allocate read requests. The buffer size is NTB_DEFAULT_IN_SIZE. */

  self->rdreq = usbdev_allocreq(self->epbulkout, NTB_DEFAULT_IN_SIZE);
  if (self->rdreq == NULL)
    {
      uerr("Out of memory\n");
      ret = -ENOMEM;
      goto error;
    }

  self->rdreq->callback = cdcncm_rdcomplete;

  /* Pre-allocate a single write request. Buffer size is NTB_OUT_SIZE */

  self->wrreq = usbdev_allocreq(self->epbulkin, NTB_OUT_SIZE);
  if (self->wrreq == NULL)
    {
      uerr("Out of memory\n");
      ret = -ENOMEM;
      goto error;
    }

  self->wrreq->callback = cdcncm_wrcomplete;

  /* The single write request just allocated is available now. */

  ret = nxsem_init(&self->wrreq_idle, 0, 1);

  if (ret != OK)
    {
      uerr("nxsem_init failed. ret: %d\n", ret);
      goto error;
    }

  self->txdone    = false;

#ifndef CONFIG_CDCNCM_COMPOSITE
#ifdef CONFIG_USBDEV_SELFPOWERED
  DEV_SETSELFPOWERED(dev);
#endif

  /* And pull-up the data line for the soft connect function (unless we are
   * part of a composite device)
   */

  DEV_CONNECT(dev);
#endif
  return OK;

error:
  uerr("cdcncm_bind failed! ret: %d\n", ret);
  cdcncm_unbind(driver, dev);
  return ret;
}

static void cdcncm_unbind(FAR struct usbdevclass_driver_s *driver,
                          FAR struct usbdev_s *dev)
{
  FAR struct cdcncm_driver_s *self = (FAR struct cdcncm_driver_s *)driver;

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !dev)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
    }
#endif

  /* Make sure that the endpoints have been unconfigured.  If
   * we were terminated gracefully, then the configuration should
   * already have been reset.  If not, then calling cdcacm_resetconfig
   * should cause the endpoints to immediately terminate all
   * transfers and return the requests to us (with result == -ESHUTDOWN)
   */

  cdcncm_resetconfig(self);
  up_mdelay(50);

  /* Free the interrupt IN endpoint */

  if (self->epint)
    {
      DEV_FREEEP(dev, self->epint);
      self->epint = NULL;
    }

  /* Free the pre-allocated control request */

  if (self->ctrlreq != NULL)
    {
      usbdev_freereq(dev->ep0, self->ctrlreq);
      self->ctrlreq = NULL;
    }

  /* Free pre-allocated read requests (which should all have
   * been returned to the free list at this time -- we don't check)
   */

  if (self->rdreq != NULL)
    {
      usbdev_freereq(self->epbulkout, self->rdreq);
      self->rdreq = NULL;
    }

  /* Free the bulk OUT endpoint */

  if (self->epbulkout)
    {
      DEV_FREEEP(dev, self->epbulkout);
      self->epbulkout = NULL;
    }

  /* Free write requests that are not in use (which should be all
   * of them)
   */

  if (self->wrreq != NULL)
    {
      usbdev_freereq(self->epbulkin, self->wrreq);
      self->wrreq = NULL;
    }

  /* Free the bulk IN endpoint */

  if (self->epbulkin)
    {
      DEV_FREEEP(dev, self->epbulkin);
      self->epbulkin = NULL;
    }

  /* Clear out all data in the rx_queue */

  netpkt_free_queue(&self->rx_queue);
}

static int cdcncm_setup(FAR struct usbdevclass_driver_s *driver,
                        FAR struct usbdev_s *dev,
                        FAR const struct usb_ctrlreq_s *ctrl,
                        FAR uint8_t *dataout,
                        size_t outlen)
{
  FAR struct cdcncm_driver_s *self = (FAR struct cdcncm_driver_s *)driver;
  uint16_t value = GETUINT16(ctrl->value);
  uint16_t index = GETUINT16(ctrl->index);
  uint16_t len = GETUINT16(ctrl->len);
  int ret = -EOPNOTSUPP;

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      switch (ctrl->req)
        {
          case USB_REQ_GETDESCRIPTOR:
            {
              uint8_t descindex = ctrl->value[0];
              uint8_t desctype  = ctrl->value[1];

              ret = cdcncm_getdescriptor(self, desctype, descindex,
                                         self->ctrlreq->buf);
            }
            break;

          case USB_REQ_SETCONFIGURATION:
            ret = cdcncm_setconfig(self, value);
            break;

          case USB_REQ_SETINTERFACE:
            ret = cdcncm_setinterface(self, index, value);
            break;

          default:
            uerr("Unsupported standard req: 0x%02hhx\n", ctrl->req);
            break;
        }
    }
  else if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS)
    {
      switch (ctrl->req)
        {
          case ECM_SET_PACKET_FILTER:

            /* SetEthernetPacketFilter is the only required CDCNCM subclass
             * specific request, but it is still ok to always operate in
             * promiscuous mode and rely on the host to do the filtering.
             * This is especially true for our case:
             * A simulated point-to-point connection.
             */

            uinfo("ECM_SET_PACKET_FILTER wValue: 0x%04hx, wIndex: 0x%04hx\n",
                  GETUINT16(ctrl->value), GETUINT16(ctrl->index));

            ret = OK;
            break;

          case NCM_GET_NTB_PARAMETERS:
            if (len >= sizeof(g_ntbparameters))
              {
                memcpy(self->ctrlreq->buf, &g_ntbparameters,
                       sizeof(g_ntbparameters));
                ret = sizeof(g_ntbparameters);
              }
            break;

          case NCM_SET_NTB_FORMAT:
            if (len != 0 || index != self->devinfo.ifnobase)
              break;
            switch (value)
              {
                case 0x0000:
                  self->parseropts = &g_ndp16_opts;
                  uinfo("NCM16 selected\n");
                  ret = 0;
                  break;
                case 0x0001:
                  self->parseropts = &g_ndp32_opts;
                  uinfo("NCM32 selected\n");
                  ret = 0;
                  break;
                default:
                  break;
              }

            break;

          case NCM_GET_NTB_INPUT_SIZE:
            uinfo("NCM_GET_NTB_INPUT_SIZE len %d\n", len);
            ret = 0;
            break;

          case NCM_SET_NTB_INPUT_SIZE:
            if (len == 4 && value == 0)
              {
                uinfo("NCM_SET_NTB_INPUT_SIZE len %d NTB input size %d\n",
                      len, *(int *)dataout);
                ret = 0;
              }
            break;

          default:
            uerr("Unsupported class req: 0x%02hhx\n", ctrl->req);
            break;
        }
    }
  else
    {
      uerr("Unsupported type: 0x%02hhx\n", ctrl->type);
    }

  if (ret >= 0)
    {
      FAR struct usbdev_req_s *ctrlreq = self->ctrlreq;

      ctrlreq->len   = MIN(len, ret);
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;

      ret = EP_SUBMIT(dev->ep0, ctrlreq);
      uinfo("EP_SUBMIT ret: %d\n", ret);

      if (ret < 0)
        {
          ctrlreq->result = OK;
          cdcncm_ep0incomplete(dev->ep0, ctrlreq);
        }
    }

  return ret;
}

static void cdcncm_disconnect(FAR struct usbdevclass_driver_s *driver,
                              FAR struct usbdev_s *dev)
{
  uinfo("\n");
}

/****************************************************************************
 * Name: cdcncm_classobject
 *
 * Description:
 *   Register USB CDC/ECM and return the class object.
 *
 * Returned Value:
 *   A pointer to the allocated class object (NULL on failure).
 *
 ****************************************************************************/

static int cdcncm_classobject(int minor,
                              FAR struct usbdev_devinfo_s *devinfo,
                              FAR struct usbdevclass_driver_s **classdev)
{
  FAR struct cdcncm_driver_s *self;
  int ret;

  /* Initialize the driver structure */

  self = kmm_zalloc(sizeof(struct cdcncm_driver_s));
  if (!self)
    {
      nerr("Out of memory!\n");
      return -ENOMEM;
    }

  /* Network device initialization */

  self->dev.ops              = &g_netops;
  self->dev.quota[NETPKT_TX] = CONFIG_CDCNCM_QUOTA_TX;
  self->dev.quota[NETPKT_RX] = CONFIG_CDCNCM_QUOTA_RX;

  /* USB device initialization */

#ifdef CONFIG_USBDEV_DUALSPEED
  self->usbdev.speed  = USB_SPEED_HIGH;
#else
  self->usbdev.speed  = USB_SPEED_FULL;
#endif
  self->usbdev.ops    = &g_usbdevops;

  memcpy(&self->devinfo, devinfo, sizeof(struct usbdev_devinfo_s));

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling cdcncm_ifdown().
   */

  cdcncm_ifdown(&self->dev);

  /* Read the MAC address from the hardware into
   * priv->dev.netdev.d_mac.ether.ether_addr_octet
   * Applies only if the Ethernet MAC has its own internal address.
   */

  memcpy(self->dev.netdev.d_mac.ether.ether_addr_octet,
         "\x00\xe0\xde\xad\xbe\xef", IFHWADDRLEN);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_lower_register(&self->dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      nerr("netdev_lower_register failed. ret: %d\n", ret);
    }

  *classdev = (FAR struct usbdevclass_driver_s *)self;
  return ret;
}

/****************************************************************************
 * Name: cdcncm_uninitialize
 *
 * Description:
 *   Un-initialize the USB CDC/ECM class driver.  This function is used
 *   internally by the USB composite driver to uninitialize the CDC/ECM
 *   driver.  This same interface is available (with an untyped input
 *   parameter) when the CDC/ECM driver is used standalone.
 *
 * Input Parameters:
 *   There is one parameter, it differs in typing depending upon whether the
 *   CDC/ECM driver is an internal part of a composite device, or a
 *   standalone USB driver:
 *
 *     classdev - The class object returned by cdcncm_classobject()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void cdcncm_uninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  FAR struct cdcncm_driver_s *self = (FAR struct cdcncm_driver_s *)classdev;
  int ret;

  /* Un-register the CDC/ECM netdev device */

  ret = netdev_lower_unregister(&self->dev);
  if (ret < 0)
    {
      nerr("ERROR: netdev_lower_unregister failed. ret: %d\n", ret);
    }

#ifndef CONFIG_CDCNCM_COMPOSITE
  usbdev_unregister(&self->usbdev);
#endif

  /* And free the driver structure */

  kmm_free(self);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcncm_initialize
 *
 * Description:
 *   Register CDC/ECM USB device interface. Register the corresponding
 *   network driver to NuttX and bring up the network.
 *
 * Input Parameters:
 *   minor - Device minor number.
 *   handle - An optional opaque reference to the CDC/ECM class object that
 *     may subsequently be used with cdcncm_uninitialize().
 *
 * Returned Value:
 *   Zero (OK) means that the driver was successfully registered.  On any
 *   failure, a negated errno value is returned.
 *
 ****************************************************************************/

#ifndef CONFIG_CDCNCM_COMPOSITE
int cdcncm_initialize(int minor, FAR void **handle)
{
  FAR struct usbdevclass_driver_s *drvr = NULL;
  struct usbdev_devinfo_s devinfo;
  int ret;

  memset(&devinfo, 0, sizeof(struct usbdev_devinfo_s));
  devinfo.ninterfaces                 = CDCECM_NINTERFACES;
  devinfo.nstrings                    = CDCECM_NSTRIDS;
  devinfo.nendpoints                  = CDCECM_NUM_EPS;
  devinfo.epno[CDCNCM_EP_INTIN_IDX]   = CONFIG_CDCNCM_EPINTIN;
  devinfo.epno[CDCNCM_EP_BULKIN_IDX]  = CONFIG_CDCNCM_EPBULKIN;
  devinfo.epno[CDCNCM_EP_BULKOUT_IDX] = CONFIG_CDCNCM_EPBULKOUT;

  ret = cdcncm_classobject(minor, &devinfo, &drvr);
  if (ret == OK)
    {
      ret = usbdev_register(drvr);
      if (ret < 0)
        {
          uinfo("usbdev_register failed. ret %d\n", ret);
        }
    }

  if (handle)
    {
      *handle = (FAR void *)drvr;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: cdcncm_get_composite_devdesc
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

#ifdef CONFIG_CDCNCM_COMPOSITE
void cdcncm_get_composite_devdesc(FAR struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  /* The callback functions for the CDC/ECM class.
   *
   * classobject() and uninitialize() must be provided by board-specific
   * logic
   */

  dev->mkconfdesc   = cdcncm_mkcfgdesc;
  dev->mkstrdesc    = cdcncm_mkstrdesc;
  dev->classobject  = cdcncm_classobject;
  dev->uninitialize = cdcncm_uninitialize;

  dev->nconfigs     = CDCECM_NCONFIGS; /* Number of configurations supported  */
  dev->configid     = CDCECM_CONFIGID; /* The only supported configuration ID */

  /* Let the construction function calculate the size of config descriptor */

#ifdef CONFIG_USBDEV_DUALSPEED
  dev->cfgdescsize  = cdcncm_mkcfgdesc(NULL, NULL, USB_SPEED_UNKNOWN, 0);
#else
  dev->cfgdescsize  = cdcncm_mkcfgdesc(NULL, NULL);
#endif

  /* Board-specific logic must provide the device minor */

  /* Interfaces.
   *
   * ifnobase must be provided by board-specific logic
   */

  dev->devinfo.ninterfaces = CDCECM_NINTERFACES; /* Number of interfaces in the configuration */

  /* Strings.
   *
   * strbase must be provided by board-specific logic
   */

  dev->devinfo.nstrings    = CDCECM_NSTRIDS + 1;     /* Number of Strings */

  /* Endpoints.
   *
   * Endpoint numbers must be provided by board-specific logic.
   */

  dev->devinfo.nendpoints  = CDCECM_NUM_EPS;
}
#endif /* CONFIG_CDCNCM_COMPOSITE */

#endif /* CONFIG_NET_CDCNCM */
