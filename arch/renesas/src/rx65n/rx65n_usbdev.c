/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_usbdev.c
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
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <stdint.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/serial/serial.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>
#include <arch/chip/types.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "rx65n_usbdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if defined(CONFIG_USBHOST) && defined(CONFIG_USBDEV)
#  error "Both USB Host & Device cannot be configured"
#endif

#ifndef  CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100  /* mA */
#endif

#define RX65N_NENDPOINTS      (8)
#define EP0                   (0)
#define EP1                   (1)
#define EP2                   (2)
#define EP3                   (3)
#define EP4                   (4)
#define EP5                   (5)
#define EP6                   (6)
#define EP7                   (7)

#define RX65N_ENDP_BIT(ep)    (1 << (ep))
#define RX65N_ENDP_ALLSET     0xff

#define RX65N_MAXPACKET_SHIFT (6)
#define RX65N_MAXPACKET_SIZE  (1 << (RX65N_MAXPACKET_SHIFT))
#define RX65N_MAXPACKET_MASK  (RX65N_MAXPACKET_SIZE-1)

#define RX65N_EP0MAXPACKET    RX65N_MAXPACKET_SIZE

#define REQRECIPIENT_MASK     (USB_REQ_TYPE_MASK | USB_REQ_RECIPIENT_MASK)

/* Debug ********************************************************************/

/* Trace error codes */

#define RX65N_TRACEERR_ALLOCFAIL         0x0001
#define RX65N_TRACEERR_ATTACHIRQ         0x0002
#define RX65N_TRACEERR_BINDFAILED        0x0003
#define RX65N_TRACEERR_DRIVER            0x0004
#define RX65N_TRACEERR_DRIVERREGISTERED  0x0005
#define RX65N_TRACEERR_EPREAD            0x0006
#define RX65N_TRACEERR_EWRITE            0x0007
#define RX65N_TRACEERR_INVALIDPARMS      0x0008
#define RX65N_TRACEERR_NOEP              0x0009
#define RX65N_TRACEERR_NOTCONFIGURED     0x000a
#define RX65N_TRACEERR_NULLPACKET        0x000b
#define RX65N_TRACEERR_NULLREQUEST       0x000c
#define RX65N_TRACEERR_REQABORTED        0x000d
#define RX65N_TRACEERR_STALLEDCLRFEATURE 0x000e
#define RX65N_TRACEERR_STALLEDISPATCH    0x000f
#define RX65N_TRACEERR_STALLEDGETST      0x0010
#define RX65N_TRACEERR_STALLEDGETSTEP    0x0011
#define RX65N_TRACEERR_STALLEDGETSTRECIP 0x0012
#define RX65N_TRACEERR_STALLEDREQUEST    0x0013
#define RX65N_TRACEERR_STALLEDSETFEATURE 0x0014
#define RX65N_TRACEERR_BADGETCONFIG      0x0015
#define RX65N_TRACEERR_IRQREGISTRATION   0x001a
#define RX65N_TRACEERR_DISPATCHSTALL     0x001b
#define RX65N_TRACEERR_INVALIDCTRLREQ    0x001c
#define RX65N_TRACEERR_BADEPTYPE         0x001d
#define RX65N_TRACEERR_BADEPNO           0x001e
#define RX65N_TRACEERR_EPRESERVE         0x001f
#define RX65N_TRACEERR_BADSETCONFIG      0x0020

/* Trace interrupt codes */

#define RX65N_TRACEINTID_CLEARFEATURE    0x0001
#define RX65N_TRACEINTID_CONTROL         0x0002
#define RX65N_TRACEINTID_DISPATCH        0x0003
#define RX65N_TRACEINTID_GETENDPOINT     0x0004
#define RX65N_TRACEINTID_GETIFDEV        0x0005
#define RX65N_TRACEINTID_GETSETDESC      0x0006
#define RX65N_TRACEINTID_GETSETIFCONFIG  0x0007
#define RX65N_TRACEINTID_GETSTATUS       0x0008
#define RX65N_TRACEINTID_RESUME          0x0009
#define RX65N_TRACEINTID_SETADDRESS      0x000a
#define RX65N_TRACEINTID_SETFEATURE      0x000b
#define RX65N_TRACEINTID_SUSPEND         0x000c
#define RX65N_TRACEINTID_SYNCHFRAME      0x000d
#define RX65N_TRACEINTID_TESTMODE        0x000e
#define RX65N_TRACEINTID_TXFIFOSTALL     0x000f
#define RX65N_TRACEINTID_GETCONFIG       0x0010
#define RX65N_TRACEINTID_EPINQEMPTY      0x0011
#define RX65N_TRACEINTID_EPOUTQEMPTY     0x0012
#define RX65N_TRACEINTID_NOSTDREQ        0x0013
#define RX65N_TRACEINTID_SETCONFIG       0x0014
#define RX65N_TRACEINTID_GETSETIF        0x0015

/* Hardware interface *******************************************************/

/* This driver supports the one interrupt IN, one bulk IN
 * and one bulk OUT endpoint.
 */

/* Hardware dependent sizes and numbers */

#define RX65N_BULKMAXPACKET     64          /* Bulk endpoint max packet */
#define RX65N_INTRMAXPACKET     64          /* Interrupt endpoint max packet */

/* Endpoint numbers */

#define RX65N_EP0                0          /* Control endpoint */
#define RX65N_EPBULKIN           1          /* Bulk EP for send to host */
#define RX65N_EPBULKOUT          2          /* Bulk EP for recv to host */
#define RX65N_EPINTRIN           3          /* Intr EP for host poll */

/* Request queue operations *************************************************/

#define rx65n_rqempty(ep)       ((ep)->head == NULL)
#define rx65n_rqpeek(ep)        ((ep)->head)

#  define LSB 0
#  define MSB 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A container for a request so that the request make be retained in a list */

struct rx65n_req_s
{
  struct usbdev_req_s    req;           /* Standard USB request */
  struct rx65n_req_s    *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct rx65n_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct rx65n_ep_s.
   */

  struct usbdev_ep_s     ep;            /* Standard endpoint structure */

  /* RX65N-specific fields */

  struct rx65n_usbdev_s *dev;           /* Reference to private driver data */
  struct rx65n_req_s    *head;          /* Request list for this endpoint */
  struct rx65n_req_s    *tail;
  uint8_t                epphy;         /* Physical EP address/index */
  uint8_t                stalled:1;     /* Endpoint is halted */
  uint8_t                in:1;          /* Endpoint is IN only */
  uint8_t                txbusy:1;      /* 1: TX endpoint FIFO full */
  uint8_t                halted:1;      /* Endpoint feature halted */
  uint8_t                txnullpkt:1;   /* Null packet needed at end of transfer */
};

/* This structure encapsulates the overall driver state */

struct rx65n_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct rx65n_usbdev_s.
   */

  struct usbdev_s        usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* RX65N-specific fields */

  uint8_t                stalled:1;     /* 1: Protocol stalled */
#ifdef CONFIG_USBDEV_SELFPOWERED
  uint8_t                selfpowered:1; /* 1: Device is self powered */
#endif
  uint8_t                epavail;       /* Bitset of available endpoints */
  uint8_t                paddrset:1;    /* 1: Peripheral addr has been set */
  uint8_t                attached:1;    /* 1: Host attached */
  uint8_t                rxpending:1;   /* 1: RX pending */
  uint8_t                paddr;         /* Peripheral address */
  uint8_t                ep0state;

  struct work_s    rx65n_interrupt_bhalf;
  struct usb_ctrlreq_s   ctrl;

  /* The endpoint list */

  struct rx65n_ep_s      eplist[RX65N_NENDPOINTS];
};

/* For maintaining tables of endpoint info */

struct rx65n_epinfo_s
{
  uint8_t                addr;          /* Logical endpoint address */
  uint8_t                attr;          /* Endpoint attributes */
  uint8_t                fifo;          /* FIFO mx pkt size + dual buffer bits */
#ifdef CONFIG_USBDEV_HIGHSPEED
  uint16_t               maxpacket;     /* Max packet size */
#else
  uint8_t                maxpacket;     /* Max packet size */
#endif
};

union wb_u
{
  uint16_t w;
  uint8_t  b[2];
};

uint16_t g_usb_pstd_req_type;          /* Type */
uint16_t g_usb_pstd_req_value;         /* Value */
uint16_t g_usb_pstd_req_index;         /* Index */
uint16_t g_usb_pstd_req_length;        /* Length */

uint32_t g_usb_pstd_data_cnt[USB_MAXPIPE_NUM + 1u];
uint8_t  *gp_usb_pstd_data[USB_MAXPIPE_NUM + 1u];
uint16_t g_usb_pstd_remote_wakeup = USB_FALSE;
uint16_t g_usb_pstd_test_mode_flag = USB_FALSE;         /* Test mode flag */
uint16_t g_usb_pstd_test_mode_select;

int bytesread = 0;

struct usb_utr
{
    uint16_t    keyword;        /* Root port / Device address / Pipe number */
    uint8_t     *p_tranadr;     /* Transfer data Start address */
    uint32_t    tranlen;        /* Transfer data length */
};

struct usb_utr *g_p_usb_pstd_pipe[USB_MAXPIPE_NUM + 1u];
struct usb_utr g_usb_pdata[USB_MAXPIPE_NUM + 1];
uint16_t g_usb_pstd_stall_pipe[USB_MAX_PIPE_NO + 1u];
uint8_t g_buffer[64] ;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations */

# define rx65n_getreg8(addr)      getreg8(addr)
# define rx65n_getreg16(addr)     getreg16(addr)
# define rx65n_getreg32(addr)     getreg32(addr)
# define rx65n_putreg8(val,addr)  putreg8(val,addr)
# define rx65n_putreg16(val,addr) putreg16(val,addr)
# define rx65n_putreg32(val,addr) putreg32(val,addr)

#define rx65n_usbdev_givesem(s) nxsem_post(s);

/* Request queue operations *************************************************/

FAR struct rx65n_req_s *rx65n_rqdequeue(FAR struct rx65n_ep_s
                                               *privep);
static void rx65n_rqenqueue(FAR struct rx65n_ep_s *privep, FAR struct
                            rx65n_req_s *req);

/* Low level data transfers and request operations */

static void rx65n_epwrite(uint8_t epno, struct rx65n_usbdev_s *priv,
                          struct rx65n_ep_s *privep,
                          uint8_t *buf, uint32_t nbytes);
static int  rx65n_epread(uint8_t epno, struct rx65n_usbdev_s *priv,
                         uint8_t *buf,
                         uint16_t nbytes);
static inline void rx65n_abortrequest(struct rx65n_ep_s *privep,
              struct rx65n_req_s *privreq, int16_t result);
static void rx65n_reqcomplete(struct rx65n_ep_s *privep, int16_t result);
static int  rx65n_wrrequest(uint8_t epno, struct rx65n_usbdev_s *priv,
                            struct rx65n_ep_s *privep);
static int  rx65n_rdrequest(uint8_t epno, struct rx65n_usbdev_s *priv,
                            struct rx65n_ep_s *privep);

/* Interrupt handling */

static void rx65n_dispatchrequest(struct rx65n_usbdev_s *priv);
void rx65n_ep0setup(struct rx65n_usbdev_s *priv);
static int  rx65n_usbinterrupt(int irq, FAR void *context, FAR void *arg);

/* Endpoint methods */

static int  rx65n_epconfigure(FAR struct usbdev_ep_s *ep,
              const struct usb_epdesc_s *desc, bool last);
static int  rx65n_epdisable(FAR struct usbdev_ep_s *ep);
static FAR struct usbdev_req_s *rx65n_epallocreq(FAR struct usbdev_ep_s *ep);
static void rx65n_epfreereq(FAR struct usbdev_ep_s *ep,
                            FAR struct usbdev_req_s *req);
static int  rx65n_epsubmit(FAR struct usbdev_ep_s *ep,
                           struct usbdev_req_s
                           *privreq);
static int  rx65n_epcancel(FAR struct usbdev_ep_s *ep,
                           struct usbdev_req_s
                           *privreq);
static void rx65n_cancelrequests(struct rx65n_ep_s *privep);

/* USB device controller methods */

static struct usbdev_ep_s *rx65n_allocep(struct usbdev_s *dev,
                                         uint8_t epno, bool in,
                                         uint8_t eptype);
static void rx65n_freeep(FAR struct usbdev_s *dev, FAR struct
                         usbdev_ep_s *ep);
static int rx65n_getframe(struct usbdev_s *dev);
static int rx65n_wakeup(struct usbdev_s *dev);
static int rx65n_selfpowered(struct usbdev_s *dev, bool selfpowered);
int rx65n_pullup(struct usbdev_s *dev, bool enable);

/* Interrupt Handler */

static int rx65n_usbinterrupt(int irq, FAR void *context, FAR void *arg);

/* USB Helper Functions */

void usb_pstd_attach_process (void);
uint16_t usb_pstd_chk_vbsts (void);
void hw_usb_pclear_dprpu(void);
static void hw_usb_clear_aclrm (uint16_t pipeno);
static void hw_usb_set_aclrm (uint16_t pipeno);
static void usb_cstd_do_aclrm (uint16_t pipe);
static void hw_usb_set_curpipe (uint16_t pipemode, uint16_t pipeno);
static void hw_usb_rmw_fifosel (uint16_t pipemode,
                         uint16_t data, uint16_t bitptn);
static void usb_cstd_chg_curpipe (uint16_t pipe,
                           uint16_t fifosel, uint16_t isel);
static void *hw_usb_get_fifosel_adr (uint16_t pipemode);
static uint16_t hw_usb_read_fifosel (uint16_t pipemode);
static void hw_usb_set_trclr (uint16_t pipeno);
static void hw_usb_clear_trenb (uint16_t pipeno);
static void hw_usb_clear_bempenb (uint16_t pipeno);
static void hw_usb_clear_nrdyenb (uint16_t pipeno);
static void hw_usb_clear_brdyenb (uint16_t pipeno);
static uint16_t hw_usb_read_pipectr (uint16_t pipeno);
static void hw_usb_clear_pid (uint16_t pipeno, uint16_t data);
static void usb_cstd_set_nak (uint16_t pipe);
void usb_pstd_forced_termination(uint16_t pipe, uint16_t status);
void usb_pstd_detach_process (void);
static void usb_cstd_clr_stall (uint16_t pipe);
static void hw_usb_write_pipesel (uint16_t data);
static void hw_usb_write_pipecfg (uint16_t data);
static void hw_usb_write_pipemaxp (uint16_t data);
static void hw_usb_write_pipeperi (uint16_t data);
static void hw_usb_set_sqclr (uint16_t pipeno);
static void hw_usb_clear_sts_brdy (uint16_t pipeno);
static void hw_usb_clear_status_nrdy (uint16_t pipeno);
static void hw_usb_clear_status_bemp (uint16_t pipeno);
void usb_pstd_bus_reset (void);
void usb_pstd_suspend_process (void);
uint16_t usb_cstd_port_speed (void);
static void hw_usb_write_dcpcfg (uint16_t data);
static uint16_t hw_usb_read_dvstctr (void);
static int hw_usb_read_syssts (void);
void usb_pstd_save_request(void);
static void hw_usb_set_pid (uint16_t pipeno, uint16_t data);
static void hw_usb_set_mbw (uint16_t pipemode, uint16_t data);
static uint16_t hw_usb_read_dcpmaxp (void);
static uint16_t hw_usb_read_pipemaxp (void);
static void hw_usb_write_dcpmxps (uint16_t data);
uint16_t usb_pstd_pipe2fport(uint16_t pipe);
static uint16_t usb_cstd_get_pid (uint16_t pipe);
static void usb_cstd_clr_transaction_counter (uint16_t trnreg);
static uint16_t hw_usb_read_pipecfg (void);
void usb_pstd_set_stall(uint16_t pipe);
static uint16_t hw_usb_read_frmnum (void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Endpoint methods */

static const struct usbdev_epops_s g_epops =
{
  .configure   = rx65n_epconfigure,
  .disable     = rx65n_epdisable,
  .allocreq    = rx65n_epallocreq,
  .freereq     = rx65n_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = rx65n_epallocbuffer,
  .freebuffer  = rx65n_epfreebuffer,
#endif
  .submit      = rx65n_epsubmit,
  .cancel      = rx65n_epcancel,
};

/* USB controller device methods */

static const struct usbdev_ops_s g_devops =
{
  .allocep     = rx65n_allocep,
  .freeep      = rx65n_freeep,
  .getframe    = rx65n_getframe,
  .wakeup      = rx65n_wakeup,
  .selfpowered = rx65n_selfpowered,
  .pullup      = rx65n_pullup,
};

enum rx65n_ep0state_e
{
  EP0STATE_WRREQUEST = 0,       /* Write request in progress */
  EP0STATE_RDREQUEST,           /* Read request in progress */
  EP0STATE_STALLED
};

/* There is only one, single, pre-allocated instance
 * of the driver structure
 */

struct rx65n_usbdev_s g_usbdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 ****************************************************************************/

FAR struct rx65n_req_s *rx65n_rqdequeue(FAR struct rx65n_ep_s *privep)
{
  struct rx65n_req_s *ret = privep->head;

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
 * Name: rx65n_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 ****************************************************************************/

static void rx65n_rqenqueue(FAR struct rx65n_ep_s *privep,
                            FAR struct rx65n_req_s *req)
{
  req->flink = NULL;
  if (!privep->head)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail        = req;
    }
}

/****************************************************************************
 * Name: hw_usb_get_fifoctr_adr
 *
 * Description:
 *   Get FIFOCTR Address
 *
 ****************************************************************************/

static void *hw_usb_get_fifoctr_adr (uint16_t pipemode)
{
  void *p_reg = NULL;

  switch (pipemode)
    {
      case USB_CUSE:
      p_reg = (void *)RX65N_USB_CFIFOCTR;
      break;

      case    USB_D0USE:
      p_reg = (void *)RX65N_USB_D0FIFOCTR;
      break;

      case    USB_D1USE:
      p_reg = (void *)RX65N_USB_D1FIFOCTR;
      break;

      default:
      break;
    }

  return p_reg;
}

/****************************************************************************
 * Name: hw_usb_set_mbw
 *
 * Description:
 *   Set MBW bit
 *
 ****************************************************************************/

static void hw_usb_set_mbw (uint16_t pipemode, uint16_t data)
{
  uint16_t *p_reg;

  p_reg = hw_usb_get_fifosel_adr(pipemode);

  (*p_reg) &= (~USB_MBW);
  if (0 != data)
    {
      (*p_reg) |= data;
    }
}

/****************************************************************************
 * Name: hw_usb_set_bclr
 *
 * Description:
 *   Set BCLR bit
 *
 ****************************************************************************/

static void hw_usb_set_bclr (uint16_t pipemode)
{
  uint16_t *p_reg;

  p_reg = (uint16_t *) hw_usb_get_fifoctr_adr(pipemode);

  *p_reg = USB_BCLR;
}

/****************************************************************************
 * Name: hw_usb_read_fifoctr
 *
 * Description:
 *   Read FIFOCTR Register
 *
 ****************************************************************************/

static uint16_t hw_usb_read_fifoctr (uint16_t pipemode)
{
  uint16_t *p_reg;

  p_reg = (uint16_t *) hw_usb_get_fifoctr_adr(pipemode);

  return *p_reg;
}

/****************************************************************************
 * Name: hw_usb_read_syscfg
 *
 * Description:
 *   Set Read SYSCFG Register
 *
 ****************************************************************************/

static uint16_t hw_usb_read_syscfg (void)
{
  return (rx65n_getreg16(RX65N_USB_SYSCFG));
}

/****************************************************************************
 * Name: usb_cstd_is_set_frdy
 *
 * Description:
 *   Set FRDY bit
 *
 ****************************************************************************/

static uint16_t usb_cstd_is_set_frdy (uint16_t pipe, uint16_t fifosel,
                               uint16_t isel)
{
  uint16_t buf;
  uint16_t i;

  /* Changes the FIFO port by the pipe. */

  usb_cstd_chg_curpipe(pipe, fifosel, isel);

  /* WAIT_LOOP */

  for (i = 0; i < 4; i++)
    {
      buf = hw_usb_read_fifoctr(fifosel);

      if (USB_FRDY == (uint16_t) (buf & USB_FRDY))
        {
          return (buf);
        }

      buf = hw_usb_read_syscfg();
      buf = hw_usb_read_syssts();
    }

  return (USB_ERROR);
}

/****************************************************************************
 * Name: hw_usb_read_dcpmaxp
 *
 * Description:
 *   Read DCPMAXP Register
 *
 ****************************************************************************/

static uint16_t hw_usb_read_dcpmaxp ()
{
  return rx65n_getreg16(RX65N_USB_DCPMAXP);
}

/****************************************************************************
 * Name: hw_usb_read_pipemaxp
 *
 * Description:
 *   Read PIPEMAXP Register
 *
 ****************************************************************************/

static uint16_t hw_usb_read_pipemaxp ()
{
  return rx65n_getreg16(RX65N_USB_PIPEMAXP);
}

/****************************************************************************
 * Name: usb_cstd_get_buf_size
 *
 * Description:
 *   Get buf size
 *
 ****************************************************************************/

static uint16_t usb_cstd_get_buf_size (uint16_t pipe)
{
  uint16_t size;
  uint16_t buf;

  if (USB_PIPE0 == pipe)
    {
      buf = hw_usb_read_dcpmaxp();

      /* Max Packet Size */

     size = (uint16_t) (buf & USB_MAXP);
    }
  else
    {
      /* Pipe select */

      hw_usb_write_pipesel(pipe);
      buf = hw_usb_read_pipemaxp();

      /* Max Packet Size */

      size = (uint16_t) (buf & USB_MXPS);
    }

  return size;
}

/****************************************************************************
 * Name: usb_cstd_get_maxpacket_size
 *
 * Description:
 *   Get maxpacket size
 *
 ****************************************************************************/

static uint16_t usb_cstd_get_maxpacket_size (uint16_t pipe)
{
  uint16_t size;
  uint16_t buf;

  if (USB_MAXPIPE_NUM < pipe)
    {
      return USB_NULL; /* Error */
    }

  if (USB_PIPE0 == pipe)
    {
      buf = hw_usb_read_dcpmaxp();
    }
  else
    {
      /* Pipe select */

      hw_usb_write_pipesel(pipe);
      buf = hw_usb_read_pipemaxp();
    }

  /* Max Packet Size */

  size = (uint16_t) (buf & USB_MXPS);

  return size;
}

/****************************************************************************
 * Name: hw_usb_write_fifo16
 *
 * Description:
 *   Write FIFO16
 *
 ****************************************************************************/

static void hw_usb_write_fifo16 (uint16_t pipemode, uint16_t data)
{
  switch (pipemode)
    {
      case USB_CUSE :
      USB0_CFIFO16 = data;
      break;
      case USB_D0USE :
      USB0_D0FIFO16 = data;
      break;
      case USB_D1USE :
      USB0_D1FIFO16 = data;
      break;
      default :
      break;
  }
}

/****************************************************************************
 * Name: hw_usb_write_fifo8
 *
 * Description:
 *   Write to FIFO8
 *
 ****************************************************************************/

static void hw_usb_write_fifo8 (uint16_t pipemode, uint8_t data)
{
  switch (pipemode)
    {
      case USB_CUSE :
      USB0_CFIFO8 = data;
      break;
      case USB_D0USE :
      USB0_D0FIFO8 = data;
      break;
      case USB_D1USE :
      USB0_D1FIFO8 = data;
      break;
      default :
      break;
    }
}

/****************************************************************************
 * Name: usb_pstd_write_fifo
 *
 * Description:
 *   Write to FIFO
 *
 ****************************************************************************/

uint8_t *usb_pstd_write_fifo(uint16_t count, uint16_t pipemode,
                             uint8_t *write_p)
{
  uint16_t even;

  /* WAIT_LOOP */

  for (even = (uint16_t)(count >> 1); (0 != even); --even)
    {
      /* 16bit access */

      hw_usb_write_fifo16(pipemode, *((uint16_t *)write_p));

      /* Renewal write pointer */

      write_p += sizeof(uint16_t);
    }

  if ((count & (uint16_t)0x0001u) != 0u)
    {
      /* 8bit access */

      /* count == odd */

      /* Change FIFO access width */

      hw_usb_set_mbw(pipemode, USB_MBW_8);

      /* FIFO write */

      hw_usb_write_fifo8(pipemode, *write_p);

      /* Return FIFO access width */

      hw_usb_set_mbw(pipemode, USB_MBW_16);

      /* Renewal write pointer */

      write_p++;
    }

  return write_p;
}

/****************************************************************************
 * Name: hw_usb_set_bval
 *
 * Description:
 *   Set BVAL
 *
 ****************************************************************************/

static void hw_usb_set_bval (uint16_t pipemode)
{
  uint16_t *p_reg;

  p_reg = (uint16_t *) hw_usb_get_fifoctr_adr(pipemode);

  (*p_reg) |= USB_BVAL;
}

/****************************************************************************
 * Name: usb_pstd_write_data
 *
 * Description:
 *   Write Data
 *
 ****************************************************************************/

uint16_t usb_pstd_write_data(uint16_t pipe, uint16_t pipemode)
{
  uint16_t size;
  uint16_t count;
  uint16_t buf;
  uint16_t mxps;
  uint16_t end_flag;

  if (USB_MAXPIPE_NUM < pipe)
    {
      return USB_ERROR; /* Error */
    }

  /* Changes FIFO port by the pipe. */

  if ((USB_CUSE == pipemode) && (USB_PIPE0 == pipe))
    {
      buf = usb_cstd_is_set_frdy(pipe, (uint16_t)USB_CUSE,
                                   (uint16_t)USB_ISEL);
    }
  else
    {
      buf = usb_cstd_is_set_frdy(pipe, (uint16_t)pipemode, USB_FALSE);
    }

  /* Check error */

  if (USB_ERROR == buf)
    {
      /* FIFO access error */

      buf = usb_cstd_is_set_frdy(pipe, (uint16_t)pipemode, USB_FALSE);
    }

  /* Data buffer size */

  size = usb_cstd_get_buf_size(pipe);

  /* Max Packet Size */

  mxps = usb_cstd_get_maxpacket_size(pipe);

  if (g_usb_pstd_data_cnt[pipe] <= (uint32_t)size)
    {
      count = (uint16_t)g_usb_pstd_data_cnt[pipe];

      /* Data count check */

      if (0 == count)
        {
          /* Null Packet is end of write */

          end_flag = USB_WRITESHRT;
        }

      else if (0 != (count % mxps))
        {
          /* Short Packet is end of write */

          end_flag = USB_WRITESHRT;
        }

      else
        {
          if (USB_PIPE0 == pipe)
            {
              /* Just Send Size */

              end_flag = USB_WRITING;
            }
          else
            {
              /* Write continues */

              end_flag = USB_WRITEEND;
            }
        }
    }
  else
    {
      /* Write continues */

      end_flag = USB_WRITING;
      count = size;
    }

  gp_usb_pstd_data[pipe] = usb_pstd_write_fifo(count, pipemode,
                                               gp_usb_pstd_data[pipe]);

  /* Check data count to remain */

  if (g_usb_pstd_data_cnt[pipe] < (uint32_t)size)
    {
      /* Clear data count */

      g_usb_pstd_data_cnt[pipe] = (uint32_t)0u;

      /* Read CFIFOCTR */

      buf = hw_usb_read_fifoctr(pipemode);

      /* Check BVAL */

      if (0u == (buf & USB_BVAL))
        {
          /* Short Packet */

          hw_usb_set_bval(pipemode);
        }
    }
  else
    {
      /* Total data count - count */

      g_usb_pstd_data_cnt[pipe] -= count;
    }

  /* End or Err or Continue */

  return end_flag;
}

/****************************************************************************
 * Name: hw_usb_set_nrdyenb
 *
 * Description:
 *   Set NRDYENB
 *
 ****************************************************************************/

static void hw_usb_set_nrdyenb (uint16_t pipeno)
{
  uint16_t regval;
  regval = rx65n_getreg16(RX65N_USB_NRDYENB);
  regval |= (1 << pipeno);
  rx65n_putreg16(regval, RX65N_USB_NRDYENB);
}

/****************************************************************************
 * Name: usb_cstd_nrdy_enable
 *
 * Description:
 *   NRDY Enable
 *
 ****************************************************************************/

static void usb_cstd_nrdy_enable (uint16_t pipe)
{
  if (USB_MAXPIPE_NUM < pipe)
    {
      return; /* Error */
    }

  /* Enable NRDY */

  hw_usb_set_nrdyenb(pipe);
}

/****************************************************************************
 * Name: usb_cstd_set_buf
 *
 * Description:
 *   Set Buf
 *
 ****************************************************************************/

static void usb_cstd_set_buf (uint16_t pipe)
{
  if (USB_MAXPIPE_NUM < pipe)
    {
      return; /* Error */
    }

  /* PIPE control reg set */

  hw_usb_set_pid(pipe, USB_PID_BUF);
}

/****************************************************************************
 * Name: hw_usb_set_bempenb
 *
 * Description:
 *   Set BEMPENB
 *
 ****************************************************************************/

static void hw_usb_set_bempenb (uint16_t pipeno)
{
  uint16_t regval;
  regval = rx65n_getreg16(RX65N_USB_BEMPENB);
  regval |= (1 << pipeno);
  rx65n_putreg16(regval, RX65N_USB_BEMPENB);
}

/****************************************************************************
 * Name: hw_usb_set_brdyenb
 *
 * Description:
 *   Set BRDYENB
 *
 ****************************************************************************/

static void hw_usb_set_brdyenb (uint16_t pipeno)
{
  uint16_t regval;
  regval = rx65n_getreg16(RX65N_USB_BRDYENB);
  regval |= (1 << pipeno);
  rx65n_putreg16(regval, RX65N_USB_BRDYENB);
}

/****************************************************************************
 * Name: usb_pstd_ctrl_write
 *
 * Description:
 *   CTRL Write
 *
 ****************************************************************************/

void usb_pstd_ctrl_write (uint32_t bsize, uint8_t *table)
{
  g_usb_pstd_data_cnt[USB_PIPE0] = bsize;
  gp_usb_pstd_data[USB_PIPE0] = table;

  usb_cstd_chg_curpipe((uint16_t) USB_PIPE0,
                      (uint16_t) USB_CUSE, USB_FALSE);

  /* Buffer clear */

  hw_usb_set_bclr(USB_CUSE);

  /* Interrupt enable */

  /* Enable ready interrupt */

  hw_usb_set_brdyenb((uint16_t) USB_PIPE0);

  /* Enable not ready interrupt */

  usb_cstd_nrdy_enable((uint16_t) USB_PIPE0);

  /* Set PID=BUF */

  usb_cstd_set_buf((uint16_t) USB_PIPE0);
}

/****************************************************************************
 * Name: usb_ctrl_read
 *
 * Description:
 *   CTRL Read
 *
 ****************************************************************************/

uint16_t usb_ctrl_read (uint32_t bsize, uint8_t *table)
{
  usb_pstd_ctrl_write(bsize, table);
  return OK;
}

/****************************************************************************
 * Name: usb_pstd_ctrl_read
 *
 * Description:
 *   CTRL Read invoked from class driver
 *
 ****************************************************************************/

uint16_t usb_pstd_ctrl_read (uint32_t bsize, uint8_t *table)
{
  uint16_t end_flag;
  g_usb_pstd_data_cnt[USB_PIPE0] = bsize;
  gp_usb_pstd_data[USB_PIPE0] = table;
  usb_cstd_chg_curpipe((uint16_t) USB_PIPE0, (uint16_t) USB_CUSE,
                       (uint16_t) USB_ISEL);

  /* Buffer clear */

  hw_usb_set_bclr(USB_CUSE);

  hw_usb_clear_status_bemp(USB_PIPE0);

  /* Peripheral Control sequence */

  end_flag = usb_pstd_write_data(USB_PIPE0, USB_CUSE);

  /* Peripheral control sequence */

  switch (end_flag)
    {
      /* End of data write */

      case USB_WRITESHRT :

      /* Enable not ready interrupt */

      usb_cstd_nrdy_enable((uint16_t) USB_PIPE0);

      /* Set PID=BUF */

      usb_cstd_set_buf((uint16_t) USB_PIPE0);
      break;

      /* End of data write (not null) */

      case USB_WRITEEND :

      /* Continue */

      /* Continue of data write */

      case USB_WRITING :

      /* Enable empty interrupt */

      hw_usb_set_bempenb((uint16_t) USB_PIPE0);

      /* Enable not ready interrupt */

      usb_cstd_nrdy_enable((uint16_t) USB_PIPE0);

      /* Set PID=BUF */

      usb_cstd_set_buf((uint16_t) USB_PIPE0);
      break;

      /* FIFO access error */

      case USB_ERROR :
      break;
      default :
      break;
    }

  return (end_flag); /* End or error or continue */
}

/****************************************************************************
 * Name: usb_pstd_buf_to_fifo
 *
 * Description:
 *   Buf to FIFO data write
 *
 ****************************************************************************/

void usb_pstd_buf_to_fifo(uint16_t pipe, uint16_t useport)
{
  uint16_t end_flag;

  if (USB_MAXPIPE_NUM < pipe)
    {
      return; /* Error */
    }

  /* Disable Ready Interrupt */

  hw_usb_clear_brdyenb(pipe);
  hw_usb_clear_bempenb(pipe);

  end_flag = usb_pstd_write_data(pipe, useport);

  /* Check FIFO access sequence */

  switch (end_flag)
    {
      case USB_WRITING:

          /* Continue of data write */

          /* Enable Ready Interrupt */

          hw_usb_set_brdyenb(pipe);

          /* Enable Not Ready Interrupt */

          usb_cstd_nrdy_enable(pipe);
        break;

        case USB_WRITEEND:

            /* End of data write */

            /* continue */

        case USB_WRITESHRT:

            /* End of data write */

            /* Enable Empty Interrupt */

            hw_usb_set_bempenb(pipe);

            /* Enable Not Ready Interrupt */

            usb_cstd_nrdy_enable(pipe);
        break;

        case USB_ERROR:

            /* FIFO access error */

            usb_pstd_forced_termination(pipe, (uint16_t)USB_DATA_ERR);
        break;

        default:
            usb_pstd_forced_termination(pipe, (uint16_t)USB_DATA_ERR);
        break;
    }
}

/****************************************************************************
 * Name: usb_pstd_data_end
 *
 * Description:
 *   Data End transfer
 *
 ****************************************************************************/

void usb_pstd_data_end(uint16_t pipe, uint16_t status)
{
  uint16_t useport;

  if (USB_MAXPIPE_NUM < pipe)
    {
      return; /* Error */
    }

  /* PID = NAK */

  /* Set NAK */

  usb_cstd_set_nak(pipe);

  /* Pipe number to FIFO port select */

  useport = USB_CUSE;

  /* Disable Interrupt */

  /* Disable Ready Interrupt */

  hw_usb_clear_brdyenb(pipe);

  /* Disable Not Ready Interrupt */

  hw_usb_clear_nrdyenb(pipe);

  /* Disable Empty Interrupt */

  hw_usb_clear_bempenb(pipe);

  /* Disable Transaction count */

  usb_cstd_clr_transaction_counter(pipe);

  /* Check use FIFO */

  switch (useport)
    {
      /* CFIFO use */

      case USB_CUSE:
      break;

      default:
      break;
    }
}

/****************************************************************************
 * Name: usb_pstd_send_start
 *
 * Description:
 *   Send start
 *
 ****************************************************************************/

void usb_pstd_send_start(uint16_t pipe, uint8_t *buf, uint32_t size)
{
  uint16_t useport;

  /* Select NAK */

  usb_cstd_set_nak(pipe);

  /* Set data count */

  g_usb_pstd_data_cnt[pipe] = size;

  /* Set data pointer */

  gp_usb_pstd_data[pipe] = buf;

  /* BEMP Status Clear */

  hw_usb_clear_status_bemp(pipe);

  /* BRDY Status Clear */

  hw_usb_clear_sts_brdy(pipe);

  /* Pipe number to FIFO port select */

  useport = USB_CUSE;

  usb_cstd_chg_curpipe(pipe, useport, USB_FALSE);

  /* Buffer to FIFO data write */

  usb_pstd_buf_to_fifo(pipe, useport);

  /* Set BUF */

  usb_cstd_set_buf(pipe);
}

/****************************************************************************
 * Name: hw_usb_read_pipecfg
 *
 * Description:
 *   Read Pipecfg
 *
 ****************************************************************************/

static uint16_t hw_usb_read_pipecfg ()
{
  return rx65n_getreg16(RX65N_USB_PIPECFG);
}

/****************************************************************************
 * Name: usb_cstd_get_pipe_dir
 *
 * Description:
 *   Get pipe direction
 *
 ****************************************************************************/

static uint16_t usb_cstd_get_pipe_dir (uint16_t pipe)
{
  uint16_t buf;

  if (USB_MAXPIPE_NUM < pipe)
    {
      return USB_NULL; /* Error */
    }

  /* Pipe select */

  hw_usb_write_pipesel(pipe);

  /* Read Pipe direction */

  buf = hw_usb_read_pipecfg();
  return (uint16_t) (buf & USB_DIRFIELD);
}

/****************************************************************************
 * Name: usb_cstd_get_pipe_type
 *
 * Description:
 *   Get pipe type
 *
 ****************************************************************************/

static uint16_t usb_cstd_get_pipe_type (uint16_t pipe)
{
  uint16_t buf;

  if (USB_MAXPIPE_NUM < pipe)
    {
      return USB_NULL; /* Error */
    }

  /* Pipe select */

  hw_usb_write_pipesel(pipe);

  /* Read Pipe direction */

  buf = hw_usb_read_pipecfg();
  return (uint16_t) (buf & USB_TYPFIELD);
}

/****************************************************************************
 * Name: usb_pstd_chk_configured
 *
 * Description:
 *   Check Configured
 *
 ****************************************************************************/

uint16_t usb_pstd_chk_configured(void)
{
  uint16_t buf;

  buf = rx65n_getreg16(RX65N_USB_INTSTS0);

  /* Device Status - Configured check */

  if (USB_DS_CNFG == (buf & USB_DVSQ))
    {
      /* Configured */

      return USB_TRUE;
    }
  else
    {
      /* not Configured */

      return USB_FALSE;
    }
}

/****************************************************************************
 * Name: hw_usb_set_trenb
 *
 * Description:
 *   Set TRENB
 *
 ****************************************************************************/

static void hw_usb_set_trenb (uint16_t pipeno)
{
  uint16_t *p_reg;

  p_reg = (uint16_t *)RX65N_USB_PIPE1TRE + ((pipeno - 1) * 2);

  (*p_reg) |= USB_TRENB;
}

/****************************************************************************
 * Name: hw_usb_write_pipetrn
 *
 * Description:
 *   Write to PIPETRN
 *
 ****************************************************************************/

static void hw_usb_write_pipetrn (uint16_t pipeno, uint16_t data)
{
  uint16_t *p_reg;

  p_reg = (uint16_t *)RX65N_USB_PIPE1TRN + ((pipeno - 1) * 2);

  *p_reg = data;
}

/****************************************************************************
 * Name: usb_cstd_set_transaction_counter
 *
 * Description:
 *   Set transaction counter
 *
 ****************************************************************************/

static void usb_cstd_set_transaction_counter (uint16_t trnreg,
                                              uint16_t trncnt)
{
    hw_usb_set_trclr(trnreg);
    hw_usb_write_pipetrn(trnreg, trncnt);
    hw_usb_set_trenb(trnreg);
}

/****************************************************************************
 * Name: usb_pstd_receive_start
 *
 * Description:
 *   Receive start
 *
 ****************************************************************************/

void usb_pstd_receive_start(uint16_t pipe)
{
  struct usb_utr *pp;
  uint32_t length;
  uint16_t mxps;
  uint16_t useport;

  /* Evacuation pointer */

  pp = g_p_usb_pstd_pipe[pipe];
  length = pp->tranlen;

  /* Select NAK */

  usb_cstd_set_nak(pipe);

  /* Set data pointer */

  gp_usb_pstd_data[pipe] = (uint8_t *)pp->p_tranadr;

  /* Pipe number to FIFO port select */

  useport = USB_CUSE;

  /* Check use FIFO access */

  switch (useport)
    {
      /* CFIFO use */

      case USB_CUSE:

      /* Changes the FIFO port by the pipe. */

      usb_cstd_chg_curpipe(pipe, useport, USB_FALSE);

      /* Max Packet Size */

      mxps = usb_cstd_get_maxpacket_size(pipe);
      if ((uint32_t)0u != length)
        {
           /* Data length check */

           if ((uint32_t)0u == (length % mxps))
             {
               /* Set Transaction counter */

               usb_cstd_set_transaction_counter(pipe, (uint16_t)(length /
                                                mxps));
             }
           else
            {
              /* Set Transaction counter */

              usb_cstd_set_transaction_counter(pipe, (uint16_t)((length
                                              / mxps) + (uint32_t)1u));
            }
        }

    /* Set BUF */

    usb_cstd_set_buf(pipe);

    /* Enable Ready Interrupt */

    hw_usb_set_brdyenb(pipe);

    /* Enable Not Ready Interrupt */

    usb_cstd_nrdy_enable(pipe);
    break;

    default:
    usb_pstd_forced_termination(pipe, (uint16_t)USB_DATA_ERR);
    break;
    }
}

/****************************************************************************
 * Name: usb_pstd_set_submitutr
 *
 * Description:
 *   Set Submit Request
 *
 ****************************************************************************/

void usb_pstd_set_submitutr (struct usb_utr *utrmsg)
{
  uint16_t pipenum;

  pipenum = utrmsg->keyword;
  g_p_usb_pstd_pipe[pipenum] = utrmsg;

  /* Check state (Configured) */

  if (USB_TRUE == usb_pstd_chk_configured())
    {
      if (USB_DIR_P_OUT == usb_cstd_get_pipe_dir(pipenum))
        {
          usb_pstd_receive_start(pipenum);    /* Out transfer */
        }
      else
        {
          /* In transfer */

          usb_pstd_send_start(pipenum, NULL, 0);
        }
    }
}

/****************************************************************************
 * Name: usb_pstd_transfer_start
 *
 * Description:
 *   Transfer start
 *
 ****************************************************************************/

uint8_t usb_pstd_transfer_start(struct usb_utr *ptr)
{
  usb_pstd_set_submitutr(ptr);
  return OK;
}

/****************************************************************************
 * Name: usb_data_write
 *
 * Description:
 *   Data write
 *
 ****************************************************************************/

long usb_data_write (uint8_t epno, uint8_t *buf, uint32_t size)
{
  uint8_t pipe;
  if (epno == BULK_IN_EPNUM)
    {
      pipe = BULK_IN_PIPE;
    }

  if (epno == INT_IN_EPNUM)
    {
      pipe = INT_IN_PIPE;
    }

  if (USB_TRUE == usb_pstd_chk_configured())
    {
      if (USB_DIR_P_OUT == usb_cstd_get_pipe_dir(pipe))
        {
          usb_pstd_receive_start(pipe);    /* Out transfer */
        }
      else
        {
          /* In transfer */

          usb_pstd_send_start(pipe, buf, size);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: usb_data_read
 *
 * Description:
 *   Data read
 *
 ****************************************************************************/

void usb_data_read(uint8_t *buf, uint32_t size)
{
  uint8_t pipe = BULK_OUT_PIPE;
  struct usb_utr *p_tran_data;
  memset((void *)&g_usb_pdata, 0, ((USB_MAXPIPE_NUM + 1) *
         sizeof(struct usb_utr)));
  p_tran_data = (struct usb_utr *)&g_usb_pdata[pipe];
  p_tran_data->p_tranadr = buf; /* Data address */
  p_tran_data->tranlen = size;  /* Data Size */
  p_tran_data->keyword = pipe;
  usb_pstd_transfer_start(p_tran_data);
}

/****************************************************************************
 * Name: rx65n_epwrite
 *
 * Description:
 *   Endpoint write (IN)
 *
 ****************************************************************************/

static void rx65n_epwrite(uint8_t epno, struct rx65n_usbdev_s *priv,
                          struct rx65n_ep_s *privep,
                          uint8_t *buf, uint32_t nbytes)
{
  uint8_t type;
  uint8_t g_buf[RX65N_USB_MAXP];
  type = MSBYTE(rx65n_getreg16(RX65N_USB_USBREQ));

  if (epno == 0)
    {
      if ((type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
        {
          if (type == ACM_GET_LINE_CODING)
            {
              usb_pstd_ctrl_read(nbytes, buf);
              usb_cstd_set_buf((uint16_t) USB_PIPE0);
              usb_data_read(g_buf, RX65N_USB_MAXP);
            }
      else
        {
          usb_pstd_ctrl_read(nbytes, buf);
          usb_cstd_set_buf((uint16_t) USB_PIPE0);
        }
        }
      else
        {
          usb_pstd_ctrl_read(nbytes, buf);
          usb_cstd_set_buf((uint16_t) USB_PIPE0);
        }
    }

  if (epno > 0)
    {
      usb_data_write(epno, buf, nbytes);
    }
}

/****************************************************************************
 * Name: rx65n_epread
 *
 * Description:
 *   Endpoint read (OUT)
 *
 ****************************************************************************/

static int rx65n_epread(uint8_t epno, struct rx65n_usbdev_s *priv,
                        uint8_t *buf, uint16_t nbytes)
{
  if (epno == EP0)
    {
      usb_ctrl_read(CDC_CLASS_DATA_LENGTH, buf);
    }
  else
    {
      usb_data_read(buf, nbytes);
      if (bytesread != 0)
        {
          return bytesread;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rx65n_abortrequest
 *
 * Description:
 *   Discard a request
 *
 ****************************************************************************/

static inline void rx65n_abortrequest(struct rx65n_ep_s *privep,
                                      struct rx65n_req_s *privreq,
                                      int16_t result)
{
  usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_REQABORTED),
          (uint16_t)USB_EPNO(privep->ep.eplog));

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/****************************************************************************
 * Name: rx65n_reqcomplete
 *
 * Description:
 *   Handle termination of a request.
 *
 ****************************************************************************/

static void rx65n_reqcomplete(struct rx65n_ep_s *privep, int16_t result)
{
  struct rx65n_req_s *privreq;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags = enter_critical_section();
  privreq = rx65n_rqdequeue(privep);
  leave_critical_section(flags);

  if (privreq)
    {
      /* If endpoint 0, temporarily reflect the state of protocol stalled
       * in the callback.
       */

      bool stalled = privep->stalled;
      if (USB_EPNO(privep->ep.eplog) == EP0)
        {
          privep->stalled = privep->dev->stalled;
        }

      /* Save the result in the request structure */

      privreq->req.result = result;

      /* Callback to the request completion handler */

      privreq->flink = NULL;
      privreq->req.callback(&privep->ep, &privreq->req);

      /* Restore the stalled indication */

      privep->stalled = stalled;
    }
}

/****************************************************************************
 * Name: rx65n_wrrequest
 *
 * Description:
 *   Send from the next queued write request
 *
 * Returned Value:
 *  0:not finished; 1:completed; <0:error
 *
 ****************************************************************************/

static int rx65n_wrrequest(uint8_t epno, struct rx65n_usbdev_s *priv,
                           struct rx65n_ep_s *privep)
{
  struct rx65n_req_s *privreq;
  uint8_t *buf = NULL;
  int nbytes;
  int bytesleft;

  /* We get here when an IN endpoint interrupt occurs.  So now we know that
   * there is no TX transfer in progress.
   */

  privep->txbusy = false;

  /* Check the request from the head of the endpoint request queue */

  privreq = rx65n_rqpeek(privep);
  if (!privreq)
    {
      /* There is no TX transfer in progress and no new pending TX
       * requests to send.
       */

      usbtrace(TRACE_INTDECODE(RX65N_TRACEINTID_EPINQEMPTY), 0);
      return -ENOENT;
    }

  epno = USB_EPNO(privep->ep.eplog);
  uinfo("epno=%d req=%p: len=%d xfrd=%d nullpkt=%d\n",
        epno, privreq, privreq->req.len, privreq->req.xfrd,
        privep->txnullpkt);

  /* Get the number of bytes left to be sent in the packet */

  for (; ; )
    {
      bytesleft         = privreq->req.len - privreq->req.xfrd;
      nbytes            = bytesleft;

      /* Either (1) we are committed to sending the null packet
       * (because txnullpkt == 1 && nbytes == 0), or
       * (2) we have not yet send the last packet (nbytes > 0).
       * In either case, it is appropriate to clearn txnullpkt now.
       */

      privep->txnullpkt = 0;

      /* If we are not sending a NULL packet, then clip the size
       * to maxpacket
       * and check if we need to send a following NULL packet.
       */

      if (nbytes > 0)
        {
          /* Either send the maxpacketsize or all of the
           * remaining data in request
           */

          if (nbytes >= privep->ep.maxpacket)
            {
              nbytes =  privep->ep.maxpacket;

              /* Handle the case where this packet is exactly the
               * maxpacketsize.  Do we need to send a zero-length packet
               * in this case?
               */

              if (bytesleft ==  privep->ep.maxpacket &&
               (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0)
                {
                  privep->txnullpkt = 1;
                }
            }
        }

      /* Send the packet (might be a null packet nbytes == 0) */

      buf = privreq->req.buf + privreq->req.xfrd;

      rx65n_epwrite(epno, priv, privep, buf, nbytes);

      /* Update for the next data IN interrupt */

      privreq->req.xfrd += nbytes;
      bytesleft          = privreq->req.len - privreq->req.xfrd;

      /* If all of the bytes were sent (including any final null packet)
       * then we are finished with the request buffer).
       */

      if (bytesleft == 0)
        {
          /* Return the write request to the class driver */

          usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               privreq->req.xfrd);
          privep->txnullpkt = 0;
          rx65n_reqcomplete(privep, OK);
          return OK;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rx65n_rdrequest
 *
 * Description:
 *   Receive to the next queued read request
 *
 ****************************************************************************/

static int rx65n_rdrequest(uint8_t epno, struct rx65n_usbdev_s *priv,
                           struct rx65n_ep_s *privep)
{
  struct rx65n_req_s *privreq;
  uint8_t *buf;
  int nbytesread;

  /* Check the request from the head of the endpoint request queue */

  privreq = rx65n_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(RX65N_TRACEINTID_EPOUTQEMPTY), 0);
      return OK;
    }

  uinfo("len=%d xfrd=%d nullpkt=%d\n",
        privreq->req.len, privreq->req.xfrd, privep->txnullpkt);

  usbtrace(TRACE_READ(privep->epphy), privreq->req.xfrd);

  /* Receive the next packet */

  buf        = privreq->req.buf + privreq->req.xfrd;
  nbytesread = rx65n_epread(epno, priv, buf, privep->ep.maxpacket);
  if (nbytesread < 0)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_EPREAD), nbytesread);
      return ERROR;
    }

  /* If the receive buffer is full or if the last packet was not full
   * then we are finished with the transfer.
   */

  privreq->req.xfrd += nbytesread;
  if (privreq->req.xfrd >= privreq->req.len ||
      nbytesread < privep->ep.maxpacket)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
      rx65n_reqcomplete(privep, OK);
    }

  return OK;
}

/****************************************************************************
 * Name: rx65n_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver
 *
 ****************************************************************************/

static void rx65n_dispatchrequest(struct rx65n_usbdev_s *priv)
{
  int ret;

  usbtrace(TRACE_INTDECODE(RX65N_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl,
                        NULL, 0);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_DISPATCHSTALL), 0);
          priv->stalled = 1;
        }
    }
}

/****************************************************************************
 * Name: usb_pstd_get_current_power
 *
 * Description:
 *   Get the power type
 *
 ****************************************************************************/

uint8_t usb_pstd_get_current_power (void)
{
  uint8_t currentpower;

#ifdef CONFIG_USBDEV_SELFPOWERED
  currentpower = USB_GS_SELFPOWERD;
#else
  currentpower = USB_GS_BUSPOWERD;
#endif

  return currentpower;
}

/****************************************************************************
 * Name: usb_pstd_epadr2pipe
 *
 * Description:
 *   Get the endpoint addr to pipe number
 *
 ****************************************************************************/

uint16_t usb_pstd_epadr2pipe(uint16_t dir_ep)
{
  uint16_t i;
  uint16_t direp;
  uint16_t tmp;

  /* Peripheral */

  /* Get PIPE Number from Endpoint address */

  direp = (uint16_t)(((dir_ep & 0x80) >> 3) | (dir_ep & 0x0f));

  /* EP table loop */

  /* WAIT_LOOP */

  for (i = USB_MIN_PIPE_NO; i < (USB_MAXPIPE_NUM +1); i++)
    {
      /* PIPECFG register will be set if PIPESEL register is set first */

      /* Check if PIPESEL register is set before reading PIPECFG */

      tmp = (rx65n_getreg16(RX65N_USB_PIPECFG)) &
            (USB_DIRFIELD | USB_EPNUMFIELD);

      /* EP table endpoint dir check */

      if (direp == tmp)
        {
          return i;
        }
    }

  return USB_ERROR;
}

/****************************************************************************
 * Name: hw_usb_read_dcpctr
 *
 * Description:
 *   Read DCPCTR register
 *
 ****************************************************************************/

uint16_t hw_usb_read_dcpctr(void)
{
  return rx65n_getreg16(RX65N_USB_DCPCTR);
}

/****************************************************************************
 * Name: usb_pstd_set_stall_pipe0
 *
 * Description:
 *   Set stall pipe
 *
 ****************************************************************************/

void usb_pstd_set_stall_pipe0(void)
{
  /* PIPE control reg set */

  hw_usb_set_pid(USB_PIPE0, USB_PID_STALL);
}

/****************************************************************************
 * Name: rx65n_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event
 *
 ****************************************************************************/

void rx65n_ep0setup(struct rx65n_usbdev_s *priv)
{
  struct rx65n_ep_s   *ep0     = &priv->eplist[EP0];
  struct rx65n_req_s  *privreq = rx65n_rqpeek(ep0);
  union wb_u           value;
  union wb_u           index;
  union wb_u           len;
  uint8_t              epno;
  uint8_t tbl[2];
  uint16_t ep;
  uint16_t buf;
  uint16_t pipe;

  /* Starting a control request? */

  if (priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      priv->usbdev.speed = USB_SPEED_FULL;
    }

  /* Terminate any pending requests */

  while (!rx65n_rqempty(ep0))
    {
      int16_t result = OK;
      if (privreq->req.xfrd != privreq->req.len)
        {
          result = -EPROTO;
        }

      usbtrace(TRACE_COMPLETE(ep0->epphy), privreq->req.xfrd);
      rx65n_reqcomplete(ep0, result);
    }

  /* Assume NOT stalled */

  ep0->stalled  = 0;
  priv->stalled = 0;

  /* And extract the little-endian 16-bit values to host order */

  priv->ctrl.type = LSBYTE(rx65n_getreg16(RX65N_USB_USBREQ));
  priv->ctrl.req  = MSBYTE(rx65n_getreg16(RX65N_USB_USBREQ));
  priv->ctrl.value[0] = LSBYTE(rx65n_getreg16(RX65N_USB_USBVAL));
  priv->ctrl.value[1] =  MSBYTE(rx65n_getreg16(RX65N_USB_USBVAL));
  priv->ctrl.index[0] = LSBYTE(rx65n_getreg16(RX65N_USB_USBINDX));
  priv->ctrl.index[1]  = MSBYTE(rx65n_getreg16(RX65N_USB_USBINDX));
  priv->ctrl.len[0] =  LSBYTE(rx65n_getreg16(RX65N_USB_USBLENG));
  priv->ctrl.len[1]  = MSBYTE(rx65n_getreg16(RX65N_USB_USBLENG));

  uinfo("SETUP: type=%02x req=%02x value[0]=%02x value[1] =%02x \
          index[0]=%02x index[1] =%02x len[0]=%02x len[1]=%02x\n",
          priv->ctrl.type, priv->ctrl.req, priv->ctrl.value[0],
          priv->ctrl.value[1], priv->ctrl.index[0],
          priv->ctrl.index[1], priv->ctrl.len[0],
          priv->ctrl.len[1]);

  /* And extract the little-endian 16-bit values to host order */

  value.w = GETUINT16(priv->ctrl.value);
  index.w = GETUINT16(priv->ctrl.index);
  len.w   = GETUINT16(priv->ctrl.len);

  uinfo("value=%x index=%x len=%x\n", value.w, index.w, len.w);

  /* Dispatch any non-standard requests */

  if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      usbtrace(TRACE_INTDECODE(RX65N_TRACEINTID_NOSTDREQ), priv->ctrl.type);

      /* Let the class implementation handle all non-standar requests */

      priv->ctrl.req = MSBYTE(rx65n_getreg16(RX65N_USB_USBREQ));
      priv->ctrl.type = LSBYTE(rx65n_getreg16(RX65N_USB_USBREQ));

      if (priv->ctrl.req == ACM_SET_LINE_CODING ||
          priv->ctrl.req == ACM_SET_CTRL_LINE_STATE)
        {
           priv->ep0state = EP0STATE_RDREQUEST;
           rx65n_dispatchrequest(priv);
        }

     priv->ep0state = EP0STATE_WRREQUEST;
     rx65n_dispatchrequest(priv);
    }

  /* Handle standard request.  Pick off the things of interest to the
   * USB device controller driver; pass what is left to the class driver
   */

  switch (priv->ctrl.req)
    {
      case USB_REQ_GETSTATUS:
        {
          /* type:  device-to-host; recipient = device, interface, endpoint
           * value: 0
           * index: zero interface endpoint
           * len:   2; data = status
           */

          if ((0 == value.w) && (2 == len.w))
            {
              tbl[0] = 0;
              tbl[1] = 0;

              /* Check request type */

              switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
                {
                case USB_REQ_RECIPIENT_ENDPOINT:
                  {
                    /* Endpoint number */

                    ep = (uint16_t) (index.w & USB_EPNUMFIELD);

                   /* Endpoint 0 */

                  if (0 == ep)
                    {
                      buf = hw_usb_read_dcpctr();
                      if ((uint16_t) 0 != (buf & USB_PID_STALL))
                        {
                          /* Halt set */

                          tbl[0] = USB_GS_HALT;
                        }

                      /* Control read start */

                      usb_pstd_ctrl_read((uint32_t) 2, tbl);
                    }

                  /* EP1 to max */

                  else if (ep <= USB_MAX_EP_NO)
                   {
                     if (USB_TRUE == usb_pstd_chk_configured())
                       {
                          pipe = usb_pstd_epadr2pipe(g_usb_pstd_req_index);
                          if (USB_ERROR == pipe)
                            {
                              /* Set pipe USB_PID_STALL */

                              usb_pstd_set_stall_pipe0();
                            }
                          else
                           {
                              buf = usb_cstd_get_pid(pipe);
                              if ((uint16_t) 0 != (buf & USB_PID_STALL))
                                {
                                  /* Halt set */

                                  tbl[0] = USB_GS_HALT;
                                }

                            /* Control read start */

                            usb_pstd_ctrl_read((uint32_t) 2, tbl);
                          }
                      }
                     else
                      {
                        /* Set pipe USB_PID_STALL */

                        usb_pstd_set_stall_pipe0();
                      }
                   }
                else
                {
                    /* Set pipe USB_PID_STALL */

                    usb_pstd_set_stall_pipe0();
                }
            break;

                case USB_REQ_RECIPIENT_DEVICE:
                  {
                    if (0 == index.w)
                      {
                        /* Self powered / Bus powered */

                       tbl[0] = usb_pstd_get_current_power();

                       /* Support remote wakeup ? */

                       if (USB_TRUE == g_usb_pstd_remote_wakeup)
                         {
                           tbl[0] |= USB_GS_REMOTEWAKEUP;
                         }

                       /* Control read start */

                       usb_pstd_ctrl_read((uint32_t) 2, tbl);
                      }
                    else
                     {
                      /* Request error */

                      usb_pstd_set_stall_pipe0();
                     }
                  }
                  break;

                  case USB_REQ_RECIPIENT_INTERFACE:
                    {
                      if (USB_TRUE == usb_pstd_chk_configured())
                      {
                        /* 2 is number of interfaces,
                         * as configured by class driver
                         */

                        if (g_usb_pstd_req_index < 2)

                          {
                            /* Control read start */

                            usb_pstd_ctrl_read((uint32_t) 2, tbl);
                          }
                        else
                          {
                            /* Request error (not exist interface) */

                            usb_pstd_set_stall_pipe0();
                          }
                      }
                    else
                     {
                       /* Request error */

                      usb_pstd_set_stall_pipe0();
                     }
                    }
                    break;

                  default:
                    {
                      usb_pstd_set_stall_pipe0();
                    }
                    break;
                  }
              }
          }
     }
     break;
        case USB_REQ_CLEARFEATURE:
          {
        /* type:  host-to-device; recipient = device, interface or endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   zero, data = none
         */

        usbtrace(TRACE_INTDECODE(RX65N_TRACEINTID_CLEARFEATURE),
                 priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
             USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* Let the class implementation handle all recipients
             * (except for the endpoint recipient)
             */

            rx65n_dispatchrequest(priv);
          }
        else
          {
            /* Endpoint recipient */

            epno = USB_EPNO(index.b[LSB]);
            if (USB_ENDPOINT_HALT == value.w)
                {
                    /* EP0 */

                    if (0 == epno)
                    {
                        /* Stall clear */

                        usb_cstd_clr_stall((uint16_t) USB_PIPE0);

                        /* Set pipe PID_BUF */

                        usb_cstd_set_buf((uint16_t) USB_PIPE0);
                    }

                   /* EP1 to max */

                    else if (epno <= USB_MAX_EP_NO)
                      {
                        pipe = usb_pstd_epadr2pipe(index.w);
                        if (USB_ERROR == pipe)
                          {
                            /* Request error */

                            usb_pstd_set_stall_pipe0();
                          }
                        else
                         {
                           if (USB_PID_BUF == usb_cstd_get_pid(pipe))
                              {
                                usb_cstd_set_nak(pipe);

                                /* SQCLR=1 */

                                hw_usb_set_sqclr(pipe);

                                /* Set pipe PID_BUF */

                                usb_cstd_set_buf(pipe);
                             }
                           else
                             {
                                usb_cstd_clr_stall(pipe);

                                /* SQCLR=1 */

                                hw_usb_set_sqclr(pipe);
                             }

                          /* Set pipe PID_BUF */

                          usb_cstd_set_buf((uint16_t) USB_PIPE0);
                         }
                      }
                     else
                    {
                        /* Request error */

                        usb_pstd_set_stall_pipe0();
                    }
                }
      }
          }
      break;

        case USB_REQ_SETFEATURE:
          {
        /* type:  host-to-device; recipient = device, interface, endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   0; data = none
         */

        usbtrace(TRACE_INTDECODE(RX65N_TRACEINTID_SETFEATURE),
                 priv->ctrl.type);
        if (((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE) && value.w == USB_FEATURE_TESTMODE)
          {
            /* Special case recipient=device test mode */

            uinfo("test mode: %d\n", index.w);
          }
        else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
                  USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* The class driver handles all recipients
             * except recipient=endpoint
             */

            rx65n_dispatchrequest(priv);
          }
        else
          {
            /* Handler recipient=endpoint */

            epno = USB_EPNO(index.b[LSB]);
            if (USB_ENDPOINT_HALT == value.w)
              {
                /* EP0 */

                if (0 == epno)
                  {
                    /* Set pipe PID_BUF */

                    usb_cstd_set_buf((uint16_t) USB_PIPE0);
                  }

                    /* EP1 to max */

                else if (epno <= USB_MAX_EP_NO)
                  {
                    pipe = usb_pstd_epadr2pipe(index.w);
                    if (USB_ERROR == pipe)
                      {
                        /* Request error */

                        usb_pstd_set_stall_pipe0();
                      }
                     else
                       {
                         /* Set pipe USB_PID_STALL */

                         usb_pstd_set_stall(pipe);

                         /* Set pipe PID_BUF */

                         usb_cstd_set_buf((uint16_t) USB_PIPE0);
                        }
                    }
                else
                  {
                     /* Request error */

                     usb_pstd_set_stall_pipe0();
                  }
                }
            else
              {
                /* Not specification */

                usb_pstd_set_stall_pipe0();
              }
      }
    }
  break;

        case USB_REQ_SETADDRESS:
          {
            /* type:  host-to-device; recipient = device
             * value: device address
             * index: 0
             * len:   0; data = none
             */

             if ((0 == index.w) && (0 == len.w))
               {
                 if (value.w <= 127)
                   {
                     /* Set pipe PID_BUF */

                    usb_cstd_set_buf((uint16_t) USB_PIPE0);
                   }
                 else
                   {
                     /* Not specification */

                     usb_pstd_set_stall_pipe0();
                   }
               }
             else
              {
                /* Not specification */

                usb_pstd_set_stall_pipe0();
               }
          }
      break;

        case USB_REQ_GETDESCRIPTOR:
        case USB_REQ_SETDESCRIPTOR:
          /* type:  host-to-device; recipient = device
           * value: descriptor type and index
           * index: 0 or language ID;
           * len:   descriptor len; data = descriptor
           */

          {
            uinfo("Get Descriptor Request Got from Host");
            usbtrace(TRACE_INTDECODE(RX65N_TRACEINTID_GETCONFIG),
                     priv->ctrl.type);
                /* The request seems valid... let the class
                 * implementation handle it
                 */

                rx65n_dispatchrequest(priv);
          }
            break;

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

          {
            usbtrace(TRACE_INTDECODE(RX65N_TRACEINTID_SETCONFIG), 0);
            if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
                 USB_REQ_RECIPIENT_DEVICE)
          {
            rx65n_dispatchrequest(priv);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_BADSETCONFIG), 0);
            priv->stalled = 1;
         }
          }
          break;

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
            /* Let the class implementation handle the request */

            usbtrace(TRACE_INTDECODE(RX65N_TRACEINTID_GETSETIF),
                     priv->ctrl.type);
            rx65n_dispatchrequest(priv);
          }
          break;

        case USB_REQ_SYNCHFRAME:

          /* type:  device-to-host; recipient = endpoint
           * value: 0
           * index: endpoint;
           * len:   2; data = frame number
           */

          {
            usbtrace(TRACE_INTDECODE(RX65N_TRACEINTID_SYNCHFRAME), 0);
          }
          break;

        default:
          {
            usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDCTRLREQ),
                      priv->ctrl.req);
             priv->ep0state = EP0STATE_STALLED;
          }
          break;
        }
}

/****************************************************************************
 * Name: usb_pstd_pipe2fport
 *
 * Description:
 *   Get Pipe Number to FIFO port address
 *
 ****************************************************************************/

uint16_t usb_pstd_pipe2fport(uint16_t pipe)
{
  uint16_t fifo_mode = USB_CUSE;

  if (USB_MAXPIPE_NUM < pipe)
    {
      return USB_NULL; /* Error */
    }

  return fifo_mode;
}

/****************************************************************************
 * Name: hw_usb_write_pipeperi
 *
 * Description:
 *   Write to PIPE PERI register
 *
 ****************************************************************************/

static void hw_usb_write_pipeperi (uint16_t data)
{
  rx65n_putreg16(data, RX65N_USB_PIPEPERI);
}

/****************************************************************************
 * Name: usb_cstd_pipe_init
 *
 * Description:
 *  Pipe initialization
 *
 ****************************************************************************/

static void usb_cstd_pipe_init (uint16_t pipe, uint16_t pipe_cfg,
                         uint16_t maxpacket)
{
  uint16_t pipe_peri = 0;

  /* Interrupt Disable */

  /* Ready         Int Disable */

  hw_usb_clear_brdyenb(pipe);

  /* NotReady      Int Disable */

  hw_usb_clear_nrdyenb(pipe);

  /* Empty/SizeErr Int Disable */

  hw_usb_clear_bempenb(pipe);

  /* PID=NAK & clear STALL */

  usb_cstd_clr_stall(pipe);

  /* PIPE Configuration */

  hw_usb_write_pipesel(pipe);

  hw_usb_write_pipecfg(pipe_cfg);

  hw_usb_write_pipemaxp(maxpacket);
  hw_usb_write_pipeperi(pipe_peri);

  /* FIFO buffer DATA-PID initialized */

  hw_usb_write_pipesel(USB_PIPE0);

  /* SQCLR */

  hw_usb_set_sqclr(pipe);

  /* ACLRM */

  usb_cstd_do_aclrm(pipe);

  /* Interrupt status clear */

  /* Ready         Int Clear */

  hw_usb_clear_sts_brdy(pipe);

  /* NotReady      Int Clear */

  hw_usb_clear_status_nrdy(pipe);

  /* Empty/SizeErr Int Clear */

  hw_usb_clear_status_bemp(pipe);
}

/****************************************************************************
 * Name: usb_pstd_set_pipe_reg
 *
 * Description:
 *  Set Pipe Register
 *
 ****************************************************************************/

void usb_pstd_set_pipe_reg (uint16_t pipe_no, uint16_t pipe_cfgint,
                            uint16_t pipe_cfgbulkin,
                            uint16_t pipe_cfgbulkout, uint16_t maxpacket)
{
  /* Search use pipe block */

  /* WAIT_LOOP */

  /* Initialization of registers associated with specified pipe. */

  if (pipe_no == BULK_IN_PIPE)
    {
      usb_cstd_pipe_init (pipe_no, pipe_cfgbulkin, maxpacket);
    }

  if (pipe_no == INT_IN_PIPE)
    {
      usb_cstd_pipe_init (pipe_no, pipe_cfgint, maxpacket);
    }

  if (pipe_no == BULK_OUT_PIPE)
    {
      usb_cstd_pipe_init (pipe_no, pipe_cfgbulkout, maxpacket);
    }
}

/****************************************************************************
 * Endpoint Methods
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_epconfigure
 *
 * Description:
 *   Configure endpoint, making it usable
 *
 * Input Parameters:
 *   ep   - the struct usbdev_ep_s instance obtained from allocep()
 *   desc - A struct usb_epdesc_s instance describing the endpoint
 *   last - true if this is the last endpoint to be configured.  Some
 *          hardware needs to take special action when all of the endpoints
 *          have been configured.
 *
 ****************************************************************************/

static int rx65n_epconfigure(FAR struct usbdev_ep_s *ep,
                             FAR const struct usb_epdesc_s *desc,
                             bool last)
{
  uint16_t maxpacket;
  uint8_t  epno;
  uint16_t pipe_no = 0 ;
  uint16_t pipe_cfg_int = 0;
  uint16_t pipe_cfg_bulkin = 0;
  uint16_t pipe_cfg_bulkout = 0;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !desc)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDPARMS), 0);
      printf("ERROR: ep=%p desc=%p\n");
      return -EINVAL;
    }
#endif

  /* Get the unadorned endpoint address */

  epno = USB_EPNO(desc->addr);

  usbtrace(TRACE_EPCONFIGURE, (uint16_t)epno);
  DEBUGASSERT(epno == USB_EPNO(ep->eplog));

  /* Set the requested type */

  switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK)
    {
      case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      if (USB_ISEPIN(desc->addr) && epno == INT_IN_EPNUM)
        {
          /* The full, logical EP number includes direction */

          ep->eplog = USB_EPIN(epno);
          pipe_no = INT_IN_PIPE;
          pipe_cfg_int = (uint16_t)(USB_TYPFIELD_INT | USB_DIR_P_IN | epno);
        }

      if (USB_ISEPOUT(desc->addr))
       {
         /* The full, logical EP number includes direction */

         ep->eplog = USB_EPOUT(epno);
         pipe_no = INT_OUT_PIPE;
         pipe_cfg_int = (uint16_t)(USB_TYPFIELD_INT | USB_DIR_P_OUT | epno);
       }
      break;

      case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      if (USB_ISEPIN(desc->addr) && epno == BULK_IN_EPNUM)
        {
          /* The full, logical EP number includes direction */

          ep->eplog = USB_EPIN(epno);
          pipe_no = BULK_IN_PIPE;
          pipe_cfg_bulkin    = (uint16_t)(USB_TYPFIELD_BULK | USB_CFG_DBLB |
                                         USB_DIR_P_IN | epno);
        }

      if (USB_ISEPOUT(desc->addr))
        {
          /* The full, logical EP number includes direction */

          ep->eplog = USB_EPOUT(epno);
          pipe_no = BULK_OUT_PIPE;
          pipe_cfg_bulkout    = (uint16_t)(USB_TYPFIELD_BULK | USB_CFG_DBLB |
                            USB_SHTNAKFIELD | USB_DIR_P_OUT | epno);
        }

      break;

      case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */

      /* Not Supported */

      break;

      case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint */
      hw_usb_write_dcpcfg(0);
      hw_usb_write_dcpmxps(USB_DCPMAXP);
      break;

      default:
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_BADEPTYPE),
              (uint16_t)desc->type);
      return -EINVAL;
  }

  maxpacket = GETUINT16(desc->mxpacketsize);
  hw_usb_write_pipemaxp(maxpacket);
  ep->maxpacket = maxpacket;

  hw_usb_write_dcpmxps(USB_DCPMAXP);

  usb_pstd_set_pipe_reg(pipe_no, pipe_cfg_int, pipe_cfg_bulkin,
                        pipe_cfg_bulkout, maxpacket);

  return OK;
}

/****************************************************************************
 * Name: rx65n_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 ****************************************************************************/

static int rx65n_epdisable(struct usbdev_ep_s *ep)
{
  struct rx65n_ep_s *privep = (struct rx65n_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDPARMS), 0);
      uerr("ERROR: ep=%p\n", ep);
      return -EINVAL;
    }
#endif

  /* Cancel any ongoing activity */

  flags = enter_critical_section();
  rx65n_cancelrequests(privep);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: rx65n_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 ****************************************************************************/

static FAR struct usbdev_req_s *rx65n_epallocreq(FAR struct usbdev_ep_s *ep)
{
  struct rx65n_req_s *privreq;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }

#endif
  usbtrace(TRACE_EPALLOCREQ, USB_EPNO(ep->eplog));

  privreq = (struct rx65n_req_s *)kmm_malloc(sizeof(struct rx65n_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct rx65n_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: rx65n_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 ****************************************************************************/

static void rx65n_epfreereq(FAR struct usbdev_ep_s *ep, FAR struct
                            usbdev_req_s *req)
{
  struct rx65n_req_s *privreq = (struct rx65n_req_s *)req;
#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDPARMS), 0);
      return;
    }

#endif
  usbtrace(TRACE_EPFREEREQ, USB_EPNO(ep->eplog));
  kmm_free(privreq);
}

/****************************************************************************
 * Name: rx65n_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 ****************************************************************************/

static int rx65n_epsubmit(FAR struct usbdev_ep_s *ep, FAR struct
                          usbdev_req_s *req)
{
  struct rx65n_req_s *privreq = (struct rx65n_req_s *)req;
  struct rx65n_ep_s *privep = (struct rx65n_ep_s *)ep;
  struct rx65n_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;
  int ret = OK;

#ifdef CONFIG_DEBUG_FEATURES
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDPARMS), 0);
      printf("ERROR: req=%p callback=%p buf=%p ep=%p\n",
           req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, USB_EPNO(ep->eplog));
  priv = privep->dev;

#ifdef CONFIG_DEBUG_FEATURES
  if (!priv->driver)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_NOTCONFIGURED),
               priv->usbdev.speed);
      printf("ERROR: driver=%p\n", priv->driver);
      return -ESHUTDOWN;
    }
#endif

  /* Handle the request from the class driver */

  epno        = USB_EPNO(ep->eplog);
  req->result = -EINPROGRESS;
  req->xfrd   = 0;
  flags       = enter_critical_section();

  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      rx65n_abortrequest(privep, privreq, -EBUSY);
      printf("ERROR: stalled\n");
      ret = -EBUSY;
    }

  /* Handle IN (device-to-host) requests.  NOTE:  If the class device is
   * using the bi-directional EP0, then we assume that they intend the EP0
   * IN functionality.
   */

  else if (epno == EP0)
    {
      /* Add the new request to the request queue for the IN endpoint */

      usbtrace(TRACE_INREQQUEUED(epno), req->len);
      rx65n_rqenqueue(privep, privreq);

      if (priv->ep0state == EP0STATE_RDREQUEST)
        {
          privreq->req.len = CDC_CLASS_DATA_LENGTH;
          rx65n_rdrequest(epno, priv, privep);
          return OK;
        }

      /* If the IN endpoint FIFO is available, then transfer the data now */

      if (!privep->txbusy)
        {
          ret = rx65n_wrrequest(epno, priv, privep);
          return OK;
        }
    }

  else if (USB_ISEPIN(ep->eplog) || epno == EP0)
    {
      /* Add the new request to the request queue for the IN endpoint */

      usbtrace(TRACE_INREQQUEUED(epno), req->len);
      rx65n_rqenqueue(privep, privreq);
      if (priv->ep0state == EP0STATE_RDREQUEST)
        {
          rx65n_rdrequest(epno, priv, privep);
          return OK;
        }

      /* If the IN endpoint FIFO is available, then transfer the data now */

      if (!privep->txbusy)
        {
          ret = rx65n_wrrequest(epno, priv, privep);
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      privep->txnullpkt = 0;
      rx65n_rqenqueue(privep, privreq);
      usbtrace(TRACE_OUTREQQUEUED(epno), req->len);

      /* This there a incoming data pending the availability of a request? */

      /* This there a incoming data pending the availability of a request? */

      if (priv->rxpending > 0)
        {
          ret = rx65n_rdrequest(epno, priv, privep);
          priv->rxpending--;
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: rx65n_cancelrequests
 ****************************************************************************/

static void rx65n_cancelrequests(struct rx65n_ep_s *privep)
{
  while (!rx65n_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               (rx65n_rqpeek(privep))->req.xfrd);
      rx65n_reqcomplete(privep, -ESHUTDOWN);
    }
}

/****************************************************************************
 * Name: rx65n_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 ****************************************************************************/

static int rx65n_epcancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct rx65n_ep_s *privep = (struct rx65n_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_USB
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

#endif
  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));

  flags = enter_critical_section();
  rx65n_cancelrequests(privep);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: rx65n_epreserve
 ****************************************************************************/

static inline struct rx65n_ep_s *
rx65n_epreserve(struct rx65n_usbdev_s *priv, uint8_t epset)
{
  struct rx65n_ep_s *privep = NULL;
  irqstate_t flags;
  int epndx = 0;

  flags = enter_critical_section();
  epset &= priv->epavail;
  if (epset)
    {
      /* Select the lowest bit in the set of matching, available endpoints
       * (skipping EP0)
       */

      for (epndx = 1; epndx < RX65N_NENDPOINTS; epndx++)
        {
          uint8_t bit = RX65N_ENDP_BIT(epndx);
          if ((epset & bit) != 0)
            {
              /* Mark the endpoint no longer available */

              priv->epavail &= ~bit;

              /* And return the pointer to the standard endpoint structure */

              privep = &priv->eplist[epndx];
              break;
            }
        }
    }

  leave_critical_section(flags);
  return privep;
}

/****************************************************************************
 * Device Methods
 ****************************************************************************/

/****************************************************************************
 * Name: rx65n_allocep
 *
 * Description:
 *   Allocate an endpoint matching the parameters
 *
 * Input Parameters:
 *   eplog  - 7-bit logical endpoint number (direction bit ignored).
 *   Zero means that any endpoint matching the other requirements
 *   will suffice.  The assigned endpoint can be found in
 *   the eplog field. in     - true: IN (device-to-host)
 *   endpoint requested eptype - Endpoint type.  One of
 *   {USB_EP_ATTR_XFER_ISOC, USB_EP_ATTR_XFER_BULK,
 *    USB_EP_ATTR_XFER_INT}
 *
 ****************************************************************************/

static FAR struct usbdev_ep_s *rx65n_allocep(FAR struct usbdev_s *dev,
                                             uint8_t epno, bool in,
                                             uint8_t eptype)
{
  struct rx65n_usbdev_s *priv = (struct rx65n_usbdev_s *)dev;
  struct rx65n_ep_s *privep = NULL;
  uint8_t epset = RX65N_ENDP_ALLSET;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)epno);
#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  /* Ignore any direction bits in the logical address */

  epno = USB_EPNO(epno);

  /* A logical address of 0 means that any endpoint will do */

  if (epno > 0)
    {
      /* Otherwise, we will return the endpoint structure only
       * for the requested 'logical' endpoint. All of the other
       * checks will still be performed.
       *
       * First, verify that the logical endpoint is in
       * the range supported by
       * by the hardware.
       */

      if (epno >= RX65N_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_BADEPNO), (uint16_t)epno);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset = RX65N_ENDP_BIT(epno);
    }

  /* Check if the selected endpoint number is available */

  privep = rx65n_epreserve(priv, epset);
  if (!privep)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_EPRESERVE), (uint16_t)epset);
      goto errout;
    }

  return &privep->ep;

errout:
  return NULL;
}

/****************************************************************************
 * Name: rx65n_epunreserve
 ****************************************************************************/

static inline void
rx65n_epunreserve(struct rx65n_usbdev_s *priv,
                    struct rx65n_ep_s *privep)
{
  irqstate_t flags = enter_critical_section();
  priv->epavail   |= RX65N_ENDP_BIT(USB_EPNO(privep->ep.eplog));
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rx65n_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 ****************************************************************************/

static void rx65n_freeep(FAR struct usbdev_s *dev, FAR struct
                         usbdev_ep_s *ep)
{
  struct rx65n_usbdev_s *priv;
  struct rx65n_ep_s *privep;

#ifdef CONFIG_DEBUG_USB
  if (!dev || !ep)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  priv   = (struct rx65n_usbdev_s *)dev;
  privep = (struct rx65n_ep_s *)ep;
  usbtrace(TRACE_DEVFREEEP, (uint16_t)USB_EPNO(ep->eplog));

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      rx65n_epunreserve(priv, privep);
    }
}

/****************************************************************************
 * Name: rx65n_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 ****************************************************************************/

static int rx65n_getframe(struct usbdev_s *dev)
{
  int regval;
  regval = (rx65n_getreg16(RX65N_USB_FRMNUM) & RX65N_USB_FRMNUM_VAL);
  return regval;
}

/****************************************************************************
 * Name: rx65n_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 ****************************************************************************/

static int rx65n_wakeup(struct usbdev_s *dev)
{
  uint16_t regval;
  regval = rx65n_getreg16 (RX65N_USB_INTENB0);
  regval |= RX65N_USB_INTENB0_RSME;
  rx65n_putreg16(regval, RX65N_USB_INTENB0);
  return OK;
}

/****************************************************************************
 * Name: rx65n_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature
 *
 ****************************************************************************/

static int rx65n_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
#ifdef CONFIG_USBDEV_SELFPOWERED
  struct rx65n_usbdev_s *priv = (struct rx65n_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;

  return OK;
#else
  /* Do Nothing for bus powered configuration */

  return OK;

#endif
}

/****************************************************************************
 * Name: rx65n_pullup
 *
 * Description:
 *    Software-controlled connect to/disconnect from USB host
 *
 ****************************************************************************/

int rx65n_pullup(struct usbdev_s *dev, bool enable)
{
  uint16_t regval;

  /* Disable DRPD bit in SYSCFG register */

  regval = rx65n_getreg16 (RX65N_USB_SYSCFG);
  regval &= ~(RX65N_USB_SYSCFG_DRPD);
  rx65n_putreg16(regval, RX65N_USB_SYSCFG);

  return OK;
}

/****************************************************************************
 * Name: usb_pstd_attach_process
 *
 * Description:
 *   Set the DPRPU bit
 *
 ****************************************************************************/

void usb_pstd_attach_process (void)
{
  uint16_t regval;

  regval = rx65n_getreg16 (RX65N_USB_SYSCFG);
  regval |= (RX65N_USB_SYSCFG_DPRPU);
  rx65n_putreg16(regval, RX65N_USB_SYSCFG);
}

/****************************************************************************
 * Name: hw_usb_pclear_dprpu
 *
 * Description:
 *   Clear the DPRPU bit
 *
 ****************************************************************************/

void hw_usb_pclear_dprpu(void)
{
  uint16_t regval;

  regval = rx65n_getreg16(RX65N_USB_SYSCFG);
  regval &= ~(RX65N_USB_SYSCFG_DPRPU);
  rx65n_putreg16(regval, RX65N_USB_SYSCFG);
}

/****************************************************************************
 * Name: hw_usb_clear_aclrm
 *
 * Description:
 *   Clear ACLRM bit
 *
 ****************************************************************************/

static void hw_usb_clear_aclrm (uint16_t pipeno)
{
  uint16_t *p_reg;

  p_reg = (uint16_t *)(RX65N_USB_PIPE1CTR) + (pipeno - 1);

  (*p_reg) &= (~USB_ACLRM);
}

/****************************************************************************
 * Name: hw_usb_set_aclrm
 *
 * Description:
 *   Set ACLRM bit
 *
 ****************************************************************************/

static void hw_usb_set_aclrm (uint16_t pipeno)
{
  uint16_t *p_reg;

       p_reg = (uint16_t *)(RX65N_USB_PIPE1CTR) + (pipeno - 1);

      (*p_reg) |= USB_ACLRM;
}

/****************************************************************************
 * Name: usb_cstd_do_aclrm
 *
 * Description:
 *   Control ACLRM bit
 *
 ****************************************************************************/

static void usb_cstd_do_aclrm (uint16_t pipe)
{
  if (USB_MAXPIPE_NUM < pipe)
    {
      return; /* Error */
    }

  /* Control ACLRM */

  hw_usb_set_aclrm(pipe);

  hw_usb_clear_aclrm(pipe);
}

/****************************************************************************
 * Name: hw_usb_set_curpipe
 *
 * Description:
 *   Set Current Pipe
 *
 ****************************************************************************/

static void hw_usb_set_curpipe (uint16_t pipemode, uint16_t pipeno)
{
  uint16_t *p_reg;
  uint16_t reg;

  p_reg = hw_usb_get_fifosel_adr(pipemode);
  reg = *p_reg;

  if ((USB_D0USE == pipemode) || (USB_D1USE == pipemode))
    {
      reg &= (~USB_DREQE);
    }

  reg &= (~USB_CURPIPE);
  *p_reg = reg;

  /* WAIT_LOOP */

  while (0 != ((*p_reg) & USB_CURPIPE))
    {
      /* Wait Clear CURPIPE */
    }

  reg |= pipeno;

  *p_reg = reg;
}

/****************************************************************************
 * Name: hw_usb_rmw_fifosel
 *
 * Description:
 *   Clear FIFOSEL bit
 *
 ****************************************************************************/

static void hw_usb_rmw_fifosel (uint16_t pipemode,
                         uint16_t data, uint16_t bitptn)
{
  uint16_t buf;
  uint16_t *p_reg;

  p_reg = (uint16_t *) hw_usb_get_fifosel_adr(pipemode);

  buf = *p_reg;
  buf &= (~bitptn);
  buf |= (data & bitptn);
  *p_reg = buf;
}

/****************************************************************************
 * Name: usb_cstd_chg_curpipe
 *
 * Description:
 *   Current Pipe Check
 *
 ****************************************************************************/

static void usb_cstd_chg_curpipe (uint16_t pipe,
                           uint16_t fifosel, uint16_t isel)
{
  uint16_t buf;

  /* Select FIFO */

  switch (fifosel)
  {
    /* CFIFO use */

    case USB_CUSE :

    /* ISEL=1, CURPIPE=0 */

    hw_usb_rmw_fifosel(USB_CUSE, ((USB_RCNT | isel) | pipe),
                      ((USB_RCNT | USB_ISEL) | USB_CURPIPE));
    break;

  /* D0FIFO use */

    case USB_D0USE :

    /* D1FIFO use */

    case USB_D1USE :

    /* DxFIFO pipe select */

    hw_usb_set_curpipe (fifosel, pipe);

    /* WAIT_LOOP */

    do
      {
        buf = hw_usb_read_fifosel (fifosel);
      }
    while ((uint16_t)(buf & USB_CURPIPE) != pipe);
    break;

    default :
    break;
  }
}

/****************************************************************************
 * Name: hw_usb_get_fifosel_adr
 *
 * Description:
 *   Get FIFOSEL register address
 *
 ****************************************************************************/

static void *hw_usb_get_fifosel_adr (uint16_t pipemode)
{
  void *p_reg = NULL;

      switch (pipemode)
        {
            case USB_CUSE:
              p_reg = (void *)RX65N_USB_CFIFOSEL;
              break;

            case    USB_D0USE:
              p_reg = (void *)RX65N_USB_D0FIFOSEL;
              break;

            case    USB_D1USE:
              p_reg = (void *)RX65N_USB_D1FIFOSEL;
              break;

            default:
            break;
    }

  return p_reg;
}

/****************************************************************************
 * Name: hw_usb_read_fifosel
 *
 * Description:
 *   Read FIFOSEL Register
 *
 ****************************************************************************/

static uint16_t hw_usb_read_fifosel (uint16_t pipemode)
{
  uint16_t *p_reg;

  p_reg = (uint16_t *) hw_usb_get_fifosel_adr(pipemode);

  return *p_reg;
}

/****************************************************************************
 * Name: hw_usb_set_trclr
 *
 * Description:
 *   Set Transaction Clear bit
 *
 ****************************************************************************/

static void hw_usb_set_trclr (uint16_t pipeno)
{
  uint16_t *p_reg;

     p_reg = (uint16_t *)(RX65N_USB_PIPE1CTR) + ((pipeno - 1) * 2);

    (*p_reg) |= USB_TRCLR;
}

/****************************************************************************
 * Name: hw_usb_clear_trenb
 *
 * Description:
 *   Clear Transaction Enable
 *
 ****************************************************************************/

static void hw_usb_clear_trenb (uint16_t pipeno)
{
  uint16_t *p_reg;

  p_reg = (uint16_t *)(RX65N_USB_PIPE1CTR) + ((pipeno - 1) * 2);
  (*p_reg) &= (~USB_TRENB);
}

/****************************************************************************
 * Name: usb_cstd_clr_transaction_counter
 *
 * Description:
 *   Clear Transaction Counter
 *
 ****************************************************************************/

static void usb_cstd_clr_transaction_counter (uint16_t trnreg)
{
    hw_usb_clear_trenb(trnreg);
    hw_usb_set_trclr(trnreg);
}

/****************************************************************************
 * Name: hw_usb_clear_bempenb
 *
 * Description:
 *   Clear BEMPENB
 *
 ****************************************************************************/

static void hw_usb_clear_bempenb (uint16_t pipeno)
{
  uint16_t regval;

  regval = rx65n_getreg16(RX65N_USB_BEMPENB);
  regval &= (~(1 << pipeno));
  rx65n_putreg16(regval, RX65N_USB_BEMPENB);
}

/****************************************************************************
 * Name: hw_usb_clear_nrdyenb
 *
 * Description:
 *   Clear NRDYENB
 *
 ****************************************************************************/

static void hw_usb_clear_nrdyenb (uint16_t pipeno)
{
  uint16_t regval;

    regval = rx65n_getreg16(RX65N_USB_NRDYENB);
    regval &= (~(1 << pipeno));
    rx65n_putreg16(regval, RX65N_USB_NRDYENB);
}

/****************************************************************************
 * Name: hw_usb_clear_brdyenb
 *
 * Description:
 *   Clear BRDYENB
 *
 ****************************************************************************/

static void hw_usb_clear_brdyenb (uint16_t pipeno)
{
    uint16_t regval;

    regval = rx65n_getreg16(RX65N_USB_BRDYENB);
    regval &= (~(1 << pipeno));
    rx65n_putreg16(regval, RX65N_USB_BRDYENB);
}

/****************************************************************************
 * Name: hw_usb_read_pipectr
 *
 * Description:
 *   Read PIPECTR
 *
 ****************************************************************************/

static uint16_t hw_usb_read_pipectr (uint16_t pipeno)
{
  uint16_t *p_reg;

      if (USB_PIPE0 == pipeno)
        {
          p_reg = (uint16_t *)RX65N_USB_DCPCTR;
        }
     else
        {
          p_reg = (uint16_t *)(RX65N_USB_PIPE1CTR) + (pipeno - 1);
        }

  return *p_reg;
}

/****************************************************************************
 * Name: hw_usb_clear_pid
 *
 * Description:
 *   Clear pid
 *
 ****************************************************************************/

static void hw_usb_clear_pid (uint16_t pipeno, uint16_t data)
{
  uint16_t *p_reg;

      if (USB_PIPE0 == pipeno)
        {
          p_reg = (uint16_t *)RX65N_USB_DCPCTR;
        }
      else
        {
           p_reg = (uint16_t *)(RX65N_USB_PIPE1CTR) + (pipeno - 1);
        }

  (*p_reg) &= (~data);
}

/****************************************************************************
 * Name: usb_cstd_set_nak
 *
 * Description:
 *   Set NAK
 *
 ****************************************************************************/

static void usb_cstd_set_nak (uint16_t pipe)
{
  uint16_t buf;
  uint16_t n;

  /* Set NAK */

  hw_usb_clear_pid(pipe, (uint16_t) USB_PID_BUF);

  /* The state of PBUSY continues while transmitting
   * the packet when it is a detach.
   */

  /* 1ms comes off when leaving because the packet
   * duration might not exceed 1ms.
   */

  /* Whether it is PBUSY release or
   * 1ms passage can be judged.
   */

  /* WAIT_LOOP */

  for (n = 0; n < 0xffff; ++n)
    {
      /* PIPE control reg read */

      buf = hw_usb_read_pipectr(pipe);
      if (0 == (uint16_t) (buf & USB_PBUSY))
        {
          n = 0xfffe;
        }
    }
}

/****************************************************************************
 * Name: usb_pstd_forced_termination
 *
 * Description:
 *   Clear the buffer data
 *
 ****************************************************************************/

void usb_pstd_forced_termination(uint16_t pipe, uint16_t status)
{
  uint16_t buf;

  /* PID = NAK */

  /* Set NAK */

  usb_cstd_set_nak(pipe);

  /* Disable Interrupt */

  /* Disable Ready Interrupt */

  hw_usb_clear_brdyenb(pipe);

  /* Disable Not Ready Interrupt */

  hw_usb_clear_nrdyenb(pipe);

  /* Disable Empty Interrupt */

  hw_usb_clear_bempenb(pipe);

  usb_cstd_clr_transaction_counter(pipe);

  /* Clear CFIFO-port */

  buf = hw_usb_read_fifosel(USB_CUSE);

  if ((buf & USB_CURPIPE) == pipe)
    {
      /* Changes the FIFO port by the pipe. */

      usb_cstd_chg_curpipe((uint16_t) USB_PIPE0,
                        (uint16_t) USB_CUSE, USB_FALSE);
    }

  /* Do Aclr */

  usb_cstd_do_aclrm(pipe);
}

/****************************************************************************
 * Name: usb_cstd_clr_stall
 *
 * Description:
 *   Clear Stall
 *
 ****************************************************************************/

static void usb_cstd_clr_stall (uint16_t pipe)
{
  if (USB_MAXPIPE_NUM < pipe)
    {
      return; /* Error */
    }

  /* Set NAK */

  usb_cstd_set_nak(pipe);

  /* Clear STALL */

  hw_usb_clear_pid(pipe, USB_PID_STALL);
}

/****************************************************************************
 * Name: hw_usb_write_pipesel
 *
 * Description:
 *   Write to PIPESEL register
 *
 ****************************************************************************/

static void hw_usb_write_pipesel (uint16_t data)
{
  rx65n_putreg16(data, RX65N_USB_PIPESEL);
}

/****************************************************************************
 * Name: hw_usb_write_pipecfg
 *
 * Description:
 *   Write to PIPECFG register
 *
 ****************************************************************************/

static void hw_usb_write_pipecfg (uint16_t data)
{
  rx65n_putreg16(data, RX65N_USB_PIPECFG);
}

/****************************************************************************
 * Name: hw_usb_write_pipemaxp
 *
 * Description:
 *   Write to PIPEMAXP register
 *
 ****************************************************************************/

static void hw_usb_write_pipemaxp (uint16_t data)
{
  rx65n_putreg16(data, RX65N_USB_PIPEMAXP);
}

/****************************************************************************
 * Name: hw_usb_write_pipeperi
 *
 * Description:
 *   Write to PIPEPERI register
 *
 ****************************************************************************/

/****************************************************************************
 * Name: hw_usb_set_sqclr
 *
 * Description:
 *   Write to SQCLR register
 *
 ****************************************************************************/

static void hw_usb_set_sqclr (uint16_t pipeno)
{
  uint16_t *p_reg;
  uint16_t regval;

  if (USB_PIPE0 == pipeno)
    {
      regval = rx65n_getreg16(RX65N_USB_DCPCTR);
      regval |= USB_SQCLR;
      rx65n_putreg16(regval, RX65N_USB_DCPCTR);
    }
  else
    {
      p_reg = ((uint16_t *)RX65N_USB_PIPE1CTR + (pipeno - 1));
      (*p_reg) |= USB_SQCLR;
    }
}

/****************************************************************************
 * Name: hw_usb_clear_sts_brdy
 *
 * Description:
 *   Clear BRDY status
 *
 ****************************************************************************/

static void hw_usb_clear_sts_brdy (uint16_t pipeno)
{
  rx65n_putreg16((~(1 << pipeno)) & RX65N_USB_PIPE_ALL,
                RX65N_USB_BRDYSTS);
}

/****************************************************************************
 * Name: hw_usb_clear_status_nrdy
 *
 * Description:
 *   Clear NRDY status
 *
 ****************************************************************************/

static void hw_usb_clear_status_nrdy (uint16_t pipeno)
{
  rx65n_putreg16((~(1 << pipeno)) & RX65N_USB_PIPE_ALL,
                RX65N_USB_NRDYSTS);
}

/****************************************************************************
 * Name: usb_cstd_clr_pipe_cnfg
 *
 * Description:
 *   Clear pipe configuration
 *
 ****************************************************************************/

static void usb_cstd_clr_pipe_cnfg (uint16_t pipe_no)
{
  /* PID=NAK & clear STALL */

  usb_cstd_clr_stall(pipe_no);

  /* Interrupt disable */

  /* Ready         Int Disable */

  hw_usb_clear_brdyenb(pipe_no);

  /* NotReady      Int Disable */

  hw_usb_clear_nrdyenb(pipe_no);

  /* Empty/SizeErr Int Disable */

  hw_usb_clear_bempenb(pipe_no);

  /* PIPE Configuration */

  usb_cstd_chg_curpipe((uint16_t) USB_PIPE0, (uint16_t) USB_CUSE, USB_FALSE);
  hw_usb_write_pipesel(pipe_no);
  hw_usb_write_pipecfg(0);

  hw_usb_write_pipemaxp(0);
  hw_usb_write_pipeperi(0);
  hw_usb_write_pipesel(0);

  /* FIFO buffer DATA-PID initialized */

  /* SQCLR */

  hw_usb_set_sqclr(pipe_no);

  /* ACLRM */

  usb_cstd_do_aclrm(pipe_no);
  usb_cstd_clr_transaction_counter(pipe_no);

  /* Interrupt status clear */

  /* Ready         Int Clear */

  hw_usb_clear_sts_brdy(pipe_no);

  /* NotReady      Int Clear */

  hw_usb_clear_status_nrdy(pipe_no);

  /* Empty/SizeErr Int Clear */

  hw_usb_clear_status_bemp(pipe_no);
}

/****************************************************************************
 * Name: hw_usb_clear_status_bemp
 *
 * Description:
 *   Clear BEMP status
 *
 ****************************************************************************/

static void hw_usb_clear_status_bemp (uint16_t pipeno)
{
  rx65n_putreg16((~(1 << pipeno)) & RX65N_USB_PIPE_ALL,
                RX65N_USB_BEMPSTS);
}

/****************************************************************************
 * Name: usb_pstd_detach_process
 *
 * Description:
 *   Set the DPRPU bit
 *
 ****************************************************************************/

void usb_pstd_detach_process (void)
{
  int i;

  /* Pull-up disable */

  hw_usb_pclear_dprpu();
  for (i = USB_MIN_PIPE_NO; i < (USB_MAXPIPE_NUM +1); i++)
    {
      usb_pstd_forced_termination(i, (uint16_t) USB_DATA_STOP);
      usb_cstd_clr_pipe_cnfg(i);
    }

  usb_pstd_forced_termination(i, (uint16_t) USB_DATA_STOP);
  usb_cstd_clr_pipe_cnfg(i);
}

/****************************************************************************
 * Name: usb_pstd_chk_vbsts
 *
 * Description:
 *   USB VBUS Status check
 *
 ****************************************************************************/

uint16_t usb_pstd_chk_vbsts (void)
{
  uint16_t buf1;
  uint16_t buf2;
  uint16_t buf3;
  uint16_t connect_info;

  /* VBUS chattering cut */

  /* WAIT_LOOP */

  do
    {
      buf1 = rx65n_getreg16(RX65N_USB_INTSTS0);
      up_udelay(10);
      buf2 = rx65n_getreg16(RX65N_USB_INTSTS0);
      up_udelay(10);
      buf3 = rx65n_getreg16(RX65N_USB_INTSTS0);
    }
  while (((buf1 & RX65N_USB_INTSTS0_VBSTS) !=
        (buf2 & RX65N_USB_INTSTS0_VBSTS)) ||
        ((buf2 & RX65N_USB_INTSTS0_VBSTS) !=
        (buf3 & RX65N_USB_INTSTS0_VBSTS)));

  if ((uint16_t) 0 != (buf1 & RX65N_USB_INTSTS0_VBSTS))
    {
      /* Attach */

       connect_info = USB_ATTACH;
    }

  else
    {
      /* Detach */

      connect_info = USB_DETACH;
    }

  return connect_info;
}

/****************************************************************************
 * Name: hw_usb_read_dvstctr
 *
 * Description:
 *   Read DVSTCTR register
 *
 ****************************************************************************/

static uint16_t hw_usb_read_dvstctr ()
{
  uint16_t regval;

  regval = rx65n_getreg16(RX65N_USB_DVSTCTR0);

  return regval;
}

/****************************************************************************
 * Name: usb_cstd_port_speed
 *
 * Description:
 *   Set Port Speed for USB device
 *
 ****************************************************************************/

uint16_t usb_cstd_port_speed ()
{
  uint16_t buf;
  uint16_t conn_inf;

  buf = hw_usb_read_dvstctr();

  /* Reset handshake status get */

  buf = (uint16_t) (buf & USB_RHST);

  switch (buf)
    {
      /* Get port speed */

      case USB_HSMODE :
      conn_inf = USB_HSCONNECT;
      break;
      case USB_FSMODE :
          conn_inf = USB_FSCONNECT;
      break;
      case USB_LSMODE :
          conn_inf = USB_LSCONNECT;
      break;
      case USB_HSPROC :
          conn_inf = USB_NOCONNECT;
      break;
      default :
          conn_inf = USB_NOCONNECT;
      break;
  }

  return (conn_inf);
}

/****************************************************************************
 * Name: hw_usb_write_dcpcfg
 *
 * Description:
 *   Write to DCPCFG register
 *
 ****************************************************************************/

static void hw_usb_write_dcpcfg (uint16_t data)
{
  rx65n_putreg16(data, RX65N_USB_DCPCFG);
}

/****************************************************************************
 * Name: hw_usb_write_dcpmxps
 *
 * Description:
 *   Write to DCPMAXP register
 *
 ****************************************************************************/

static void hw_usb_write_dcpmxps (uint16_t data)
{
  rx65n_putreg16(data, RX65N_USB_DCPMAXP);
}

/****************************************************************************
 * Name: usb_pstd_bus_reset
 *
 * Description:
 *   Bus Reset
 *
 ****************************************************************************/

void usb_pstd_bus_reset (void)
{
  struct rx65n_usbdev_s *priv = &g_usbdev;
  priv->usbdev.speed = USB_SPEED_FULL;

  /* DCP configuration register  (0x5C) */

  hw_usb_write_dcpcfg(0);
  hw_usb_write_dcpmxps(USB_DCPMAXP);
}

/****************************************************************************
 * Name: hw_usb_pset_enb_rsme
 *
 * Description:
 *   Enable Resume Interrupt
 *
 ****************************************************************************/

void hw_usb_pset_enb_rsme(void)
{
  uint16_t regval;

  regval = rx65n_getreg16(RX65N_USB_INTENB0);
  regval |= USB_RSME;
  rx65n_putreg16(regval, RX65N_USB_INTENB0);
}

/****************************************************************************
 * Name: hw_usb_read_syssts
 *
 * Description:
 *   Read System Status Register
 *
 ****************************************************************************/

static int hw_usb_read_syssts ()
{
  uint16_t regval;

  regval = rx65n_getreg16(RX65N_USB_SYSSTS0);

  return regval;
}

/****************************************************************************
 * Name: hw_usb_pclear_enb_rsme
 *
 * Description:
 *   Disable Resume Interrupt
 *
 ****************************************************************************/

void hw_usb_pclear_enb_rsme(void)
{
  uint16_t regval;

  regval = rx65n_getreg16(RX65N_USB_INTENB0);
  regval &= (~USB_RSME);
  rx65n_putreg16(regval, RX65N_USB_INTENB0);
}

/****************************************************************************
 * Name: hw_usb_pclear_sts_resm
 *
 * Description:
 *   Clear Resume state
 *
 ****************************************************************************/

void hw_usb_pclear_sts_resm(void)
{
  uint16_t regval;
  regval = (uint16_t)(~USB_RESM);
  rx65n_putreg16(regval, RX65N_USB_INTSTS0);
}

/****************************************************************************
 * Name: usb_pstd_suspend_process
 *
 * Description:
 *   Handle Suspend State of USB
 *
 ****************************************************************************/

void usb_pstd_suspend_process (void)
{
  /* Resume interrupt enable */

  hw_usb_pset_enb_rsme();
  hw_usb_pclear_sts_resm();

  /* RESM interrupt disable */

  hw_usb_pclear_enb_rsme();
}

/****************************************************************************
 * Name: hw_usb_pclear_sts_valid
 *
 * Description:
 *  Clear Valid Ststus bit
 *
 ****************************************************************************/

void hw_usb_pclear_sts_valid(void)
{
  uint16_t regval;

  regval = (uint16_t)~USB_VALID;
  rx65n_putreg16(regval, RX65N_USB_INTSTS0);
}

/****************************************************************************
 * Name: hw_usb_read_usbreq
 *
 * Description:
 *  Read USBREQ
 *
 ****************************************************************************/

uint16_t hw_usb_read_usbreq(void)
{
  uint16_t regval;

  regval = rx65n_getreg16(RX65N_USB_USBREQ);

  return regval;
}

/****************************************************************************
 * Name: hw_usb_read_usbval
 *
 * Description:
 *  Read USBVAL
 *
 ****************************************************************************/

uint16_t hw_usb_read_usbval(void)
{
  uint16_t regval;

  regval = rx65n_getreg16(RX65N_USB_USBVAL);

  return regval;
}

/****************************************************************************
 * Name: hw_usb_read_usbindx
 *
 * Description:
 *  Read USBINDX
 *
 ****************************************************************************/

uint16_t hw_usb_read_usbindx(void)
{
  uint16_t regval;

  regval = rx65n_getreg16(RX65N_USB_USBINDX);

  return regval;
}

/****************************************************************************
 * Name: hw_usb_read_usbleng
 *
 * Description:
 *  Read USBLENG
 *
 ****************************************************************************/

uint16_t hw_usb_read_usbleng(void)
{
  uint16_t regval;

  regval = rx65n_getreg16(RX65N_USB_USBLENG);

  return regval;
}

/****************************************************************************
 * Name: usb_pstd_save_request
 *
 * Description:
 *  Save the request
 *
 ****************************************************************************/

void usb_pstd_save_request(void)
{
  /* Valid clear */

  hw_usb_pclear_sts_valid();

  g_usb_pstd_req_type = hw_usb_read_usbreq();
  g_usb_pstd_req_value = hw_usb_read_usbval();
  g_usb_pstd_req_index = hw_usb_read_usbindx();
  g_usb_pstd_req_length = hw_usb_read_usbleng();
}

/****************************************************************************
 * Name: usb_pstd_set_stall
 *
 * Description:
 *  Set Stall status
 *
 ****************************************************************************/

void usb_pstd_set_stall(uint16_t pipe)
{
  /* PIPE control reg set */

  hw_usb_set_pid(pipe, USB_PID_STALL);
}

/****************************************************************************
 * Name: hw_usb_set_pid
 *
 * Description:
 *  Set PID
 *
 ****************************************************************************/

static void hw_usb_set_pid (uint16_t pipeno, uint16_t data)
{
  uint16_t *p_reg;

  if (USB_PIPE0 == pipeno)
    {
      p_reg = ((uint16_t *)(RX65N_USB_DCPCTR));
    }
  else
    {
      p_reg = ((uint16_t *)(RX65N_USB_PIPE1CTR)) + ((pipeno - 1));
    }

  (*p_reg) &= (~USB_PID);
  (*p_reg) |= data;
}

/****************************************************************************
 * Name: hw_usb_pset_ccpl
 *
 * Description:
 *  Set CCPL
 *
 ****************************************************************************/

void hw_usb_pset_ccpl(void)
{
  uint16_t regval;

  regval = rx65n_getreg16(RX65N_USB_DCPCTR);
  regval |= USB_CCPL;
  rx65n_putreg16(regval, RX65N_USB_DCPCTR);
}

/****************************************************************************
 * Name: usb_pstd_ctrl_end
 *
 * Description:
 *  CTRL End
 *
 ****************************************************************************/

void usb_pstd_ctrl_end (uint16_t status)
{
  /* Interrupt disable */

  /* BEMP0 disable */

  hw_usb_clear_bempenb((uint16_t) USB_PIPE0);

  /* BRDY0 disable */

  hw_usb_clear_brdyenb((uint16_t) USB_PIPE0);

  /* NRDY0 disable */

  hw_usb_clear_nrdyenb((uint16_t) USB_PIPE0);

  hw_usb_set_mbw(USB_CUSE, USB0_CFIFO_MBW);

  if ((USB_DATA_ERR == status) || (USB_DATA_OVR == status))
    {
      /* Request error */

      usb_pstd_set_stall_pipe0();
    }

  else if (USB_DATA_STOP == status)
    {
      /* Pipe stop */

      usb_cstd_set_nak((uint16_t) USB_PIPE0);
    }

  else
    {
      /* Set CCPL bit */

      hw_usb_pset_ccpl();
    }
}

/****************************************************************************
 * Name: usb_pstd_set_feature3
 *
 * Description:
 *  Handles the set feature request
 *
 ****************************************************************************/

static void usb_pstd_set_feature3 (void)
{
  uint16_t pipe;
  uint16_t ep;

  if (0 == g_usb_pstd_req_length)
    {
      /* check request type */

      switch ((g_usb_pstd_req_type & USB_BMREQUESTTYPERECIP))
        {
          case USB_DEVICE :
          switch (g_usb_pstd_req_value)
            {
              case USB_DEV_REMOTE_WAKEUP :
              if (0 == g_usb_pstd_req_index)
                {
                  /* Set pipe PID_BUF */

                  usb_cstd_set_buf((uint16_t) USB_PIPE0);
                }
              else
                {
                  /* Not specification */

                  usb_pstd_set_stall_pipe0();
                }
              break;
              case USB_TEST_MODE :
              if (USB_HSCONNECT == usb_cstd_port_speed())
                {
                  if ((g_usb_pstd_req_index < USB_TEST_RESERVED)
                     || (USB_TEST_VSTMODES <= g_usb_pstd_req_index))
                    {
                      g_usb_pstd_test_mode_flag = USB_TRUE;
                      g_usb_pstd_test_mode_select = g_usb_pstd_req_index;

                      /* Set pipe PID_BUF */

                      usb_cstd_set_buf((uint16_t) USB_PIPE0);
                    }
                  else
                    {
                      /* Not specification */

                      usb_pstd_set_stall_pipe0();
                            }
                    }
              else
                {
                  /* Not specification */

                  usb_pstd_set_stall_pipe0();
                }
              break;
              default :
              usb_pstd_set_stall_pipe0();
              break;
            }
            break;
            case USB_INTERFACE :

            /* Set pipe USB_PID_STALL */

            usb_pstd_set_stall_pipe0();
            break;
            case USB_ENDPOINT :

            /* Endpoint number */

            ep = (uint16_t) (g_usb_pstd_req_index & USB_EPNUMFIELD);
            if (USB_ENDPOINT_HALT == g_usb_pstd_req_value)
              {
                /* EP0 */

                if (0 == ep)
                  {
                    /* Set pipe PID_BUF */

                    usb_cstd_set_buf((uint16_t) USB_PIPE0);
                  }

                /* EP1 to max */

                else if (ep <= USB_MAX_EP_NO)
                  {
                    pipe = usb_pstd_epadr2pipe(g_usb_pstd_req_index);
                    if (USB_ERROR == pipe)
                      {
                        /* Request error */

                        usb_pstd_set_stall_pipe0();
                      }
                    else
                      {
                        /* Set pipe USB_PID_STALL */

                        usb_pstd_set_stall(pipe);

                        /* Set pipe PID_BUF */

                        usb_cstd_set_buf((uint16_t) USB_PIPE0);
                      }
                  }
            else
              {
                /* Request error */

                usb_pstd_set_stall_pipe0();
              }
         }
                else
                {
                    /* Not specification */

                    usb_pstd_set_stall_pipe0();
                }
            break;

            default :

                /* Request error */

                usb_pstd_set_stall_pipe0();
            break;
        }
    }
  else
    {
        /* Request error */

        usb_pstd_set_stall_pipe0();
    }
}

/****************************************************************************
 * Name: usb_pstd_clr_feature3
 *
 * Description:
 *  Handles the clear feature request
 *
 ****************************************************************************/

static void usb_pstd_clr_feature3 (void)
{
  uint16_t pipe;
  uint16_t ep;

  if (0 == g_usb_pstd_req_length)
    {
      /* check request type */

      switch ((g_usb_pstd_req_type & USB_BMREQUESTTYPERECIP))
        {
           case USB_DEVICE :
           if ((USB_DEV_REMOTE_WAKEUP == g_usb_pstd_req_value)
              && (0 == g_usb_pstd_req_index))
             {
               usb_cstd_set_buf((uint16_t) USB_PIPE0);
             }
           else
             {
               /* Not specification */

               usb_pstd_set_stall_pipe0();
             }
           break;
           case USB_INTERFACE :

            /* Request error */

            usb_pstd_set_stall_pipe0();
            break;

           case USB_ENDPOINT :

           /* Endpoint number */

           ep = (uint16_t) (g_usb_pstd_req_index & USB_EPNUMFIELD);
           if (USB_ENDPOINT_HALT == g_usb_pstd_req_value)
             {
               /* EP0 */

               if (0 == ep)
                 {
                   /* Stall clear */

                   usb_cstd_clr_stall((uint16_t) USB_PIPE0);

                   /* Set pipe PID_BUF */

                   usb_cstd_set_buf((uint16_t) USB_PIPE0);
                 }

               /* EP1 to max */

               else if (ep <= USB_MAX_EP_NO)
                 {
                   pipe = usb_pstd_epadr2pipe(g_usb_pstd_req_index);
                   if (USB_ERROR == pipe)
                     {
                       /* Request error */

                       usb_pstd_set_stall_pipe0();
                     }
                   else
                     {
                       if (USB_PID_BUF == usb_cstd_get_pid(pipe))
                         {
                           usb_cstd_set_nak(pipe);

                           /* SQCLR=1 */

                           hw_usb_set_sqclr(pipe);

                           /* Set pipe PID_BUF */

                           usb_cstd_set_buf(pipe);
                         }
                       else
                         {
                           usb_cstd_clr_stall(pipe);

                           /* SQCLR=1 */

                           hw_usb_set_sqclr(pipe);
                          }

                          /* Set pipe PID_BUF */

                          usb_cstd_set_buf((uint16_t) USB_PIPE0);
                          if (USB_TRUE == g_usb_pstd_stall_pipe[pipe])
                            {
                              g_usb_pstd_stall_pipe[pipe] = USB_FALSE;
                            }
                         }
                     }
               else
                 {
                   /* Request error */

                   usb_pstd_set_stall_pipe0();
                 }
             }
           else
             {
               /* Request error */

               usb_pstd_set_stall_pipe0();
             }
           break;
           default :
           usb_pstd_set_stall_pipe0();
           break;
        }
    }
  else
    {
      /* Not specification */

      usb_pstd_set_stall_pipe0();
    }
}

/****************************************************************************
 * Name: usb_pstd_set_address3
 *
 * Description:
 *  Handles the Set Address request
 *
 ****************************************************************************/

static void usb_pstd_set_address3 (void)
{
  if (USB_DEVICE == (g_usb_pstd_req_type & USB_BMREQUESTTYPERECIP))
    {
      if ((0 == g_usb_pstd_req_index) && (0 == g_usb_pstd_req_length))
        {
          if (g_usb_pstd_req_value <= 127)
            {
              /* Set pipe PID_BUF */

              usb_cstd_set_buf((uint16_t) USB_PIPE0);
            }
          else
            {
              /* Not specification */

              usb_pstd_set_stall_pipe0();
            }
        }
      else
        {
          /* Not specification */

          usb_pstd_set_stall_pipe0();
        }
    }
  else
    {
      /* Request error */

      usb_pstd_set_stall_pipe0();
    }
}

/****************************************************************************
 * Name: usb_pstd_stand_req3
 *
 * Description:
 *  Handle standard request
 *
 ****************************************************************************/

void usb_pstd_stand_req3 (void)
{
  uint16_t regval;
  struct rx65n_usbdev_s *priv = &g_usbdev;
  regval = MSBYTE(g_usb_pstd_req_type);

  switch (regval)
    {
      case USB_REQ_CLEARFEATURE :
      usb_pstd_clr_feature3();
      break;

      case USB_REQ_SETFEATURE :
      usb_pstd_set_feature3();
      break;

     case USB_REQ_SETADDRESS :
     usb_pstd_set_address3();
     break;

     case USB_REQ_SETCONFIGURATION :
     usb_cstd_set_buf((uint16_t) USB_PIPE0);      /* SetConfiguration3 */
     break;

     case USB_REQ_SETINTERFACE :
     /* Let the class implementation handle
      * as it requires access and updation of
      * Configuration descriptor
      */

     rx65n_dispatchrequest(priv);
     break;

    default :
    break;
    }

  usb_pstd_ctrl_end((uint16_t) USB_CTRL_END); /* Control transfer stop(end) */
}

/****************************************************************************
 * Name: usb_pstd_stand_req4
 *
 * Description:
 *  Handle standard request
 *
 ****************************************************************************/

void usb_pstd_stand_req4 (void)
{
  uint16_t regval;

  regval = MSBYTE(g_usb_pstd_req_type);
  switch (regval)
    {
      case USB_REQ_GETSTATUS :
      usb_cstd_set_buf((uint16_t) USB_PIPE0);            /* GetStatus4 */
      break;

      case USB_REQ_GETDESCRIPTOR :
      usb_cstd_set_buf((uint16_t) USB_PIPE0);            /* GetDescriptor4 */
      break;

      case USB_REQ_GETCONFIGURATION :
      usb_cstd_set_buf((uint16_t) USB_PIPE0);            /* GetConfiguration4 */
      break;

      case USB_REQ_GETINTERFACE :
      usb_cstd_set_buf((uint16_t) USB_PIPE0);            /* GetInterface4 */
      break;

      case USB_REQ_SYNCHFRAME :
      usb_cstd_set_buf((uint16_t) USB_PIPE0);            /* SynchFrame4 */
      break;

      default :
      break;
    }

  usb_pstd_ctrl_end((uint16_t) USB_CTRL_END);
}

/****************************************************************************
 * Name: usb_peri_class_request
 *
 * Description:
 *  Handle Class Request
 *
 ****************************************************************************/

void usb_peri_class_request(uint8_t type, uint16_t ctsq)
{
  if ((USB_CLASS == (type & USB_BMREQUESTTYPETYPE)) ||
      (USB_VENDOR == (type & USB_BMREQUESTTYPETYPE)))
    {
      switch (ctsq)
        {
           case USB_CS_IDST :
           case USB_CS_RDDS :
           case USB_CS_WRDS :

          /* Do Nothing */

           break;
           case USB_CS_WRND :
           usb_pstd_ctrl_end((uint16_t) USB_CTRL_END);  /* class request (control write nodata status stage) */
               break;

          case USB_CS_RDSS :
          usb_cstd_set_buf((uint16_t) USB_PIPE0);
          usb_pstd_ctrl_end((uint16_t) USB_CTRL_END);   /* class request (control read status stage) */
          break;

          case USB_CS_WRSS :
          usb_cstd_set_buf((uint16_t) USB_PIPE0);
          usb_pstd_ctrl_end((uint16_t) USB_CTRL_END);   /* class request (control write status stage) */
          break;

          case USB_CS_SQER :
          usb_pstd_ctrl_end((uint16_t) USB_DATA_ERR);     /* End control transfer. */
          break;

          default :
          usb_pstd_ctrl_end((uint16_t) USB_DATA_ERR);     /* End control transfer. */
          break;
        }
    }
}

/****************************************************************************
 * Name: hw_usb_read_fifo16
 *
 * Description:
 *  Read FIFO16
 *
 ****************************************************************************/

static uint16_t hw_usb_read_fifo16 (uint16_t pipemode)
{
  uint16_t data = 0;
  switch (pipemode)
    {
       case USB_CUSE:
       data = USB0.CFIFO.WORD;
       break;

       default:
       break;
    }

  return data;
}

/****************************************************************************
 * Name: usb_pstd_read_fifo
 *
 * Description:
 *  Read FIFO
 *
 ****************************************************************************/

uint8_t *usb_pstd_read_fifo(uint16_t count, uint16_t pipemode,
                            uint8_t *read_p)
{
  uint16_t even;
  uint32_t odd_byte_data_temp;

  /* WAIT_LOOP */

  for (even = (uint16_t)(count >> 1); (0 != even); --even)
    {
      /* 16bit FIFO access */

      *(uint16_t *)read_p = hw_usb_read_fifo16(pipemode);

      /* Renewal read pointer */

      read_p += sizeof(uint16_t);
    }

  if ((count & (uint16_t)0x0001) != 0)
    {
      /* 16bit FIFO access */

      odd_byte_data_temp = hw_usb_read_fifo16(pipemode);

      *read_p = LSBYTE(odd_byte_data_temp);

      /* Renewal read pointer */

      read_p += sizeof(uint8_t);
    }

  return read_p;
}

/****************************************************************************
 * Name: usb_pstd_read_data
 *
 * Description:
 *  Read data
 *
 ****************************************************************************/

uint16_t usb_pstd_read_data(uint16_t pipe, uint16_t pipemode)
{
  uint16_t count;
  uint16_t buf;
  uint16_t mxps;
  uint16_t dtln;
  uint16_t end_flag;

  if (USB_MAXPIPE_NUM < pipe)
    {
      return USB_ERROR; /* Error */
    }

  /* Changes FIFO port by the pipe. */

  buf = usb_cstd_is_set_frdy(pipe, (uint16_t)pipemode, USB_FALSE);

  if (USB_ERROR == buf)
    {
      /* FIFO access error */

      return (USB_ERROR);
    }

  dtln = (uint16_t)(buf & USB_DTLN);

  /* Max Packet Size */

  mxps = usb_cstd_get_maxpacket_size(pipe);

  g_usb_pstd_data_cnt[pipe] = RX65N_MAXPACKET_SIZE;

  if (g_usb_pstd_data_cnt[pipe] < dtln)
    {
      /* Buffer Over ? */

      end_flag = USB_READOVER;

      /* Set NAK */

      usb_cstd_set_nak(pipe);
      count = (uint16_t)g_usb_pstd_data_cnt[pipe];
      g_usb_pstd_data_cnt[pipe] = dtln;
    }
  else if (g_usb_pstd_data_cnt[pipe] == dtln)
    {
      /* Just Receive Size */

      count = dtln;
      if ((USB_PIPE0 == pipe) && (0 == (dtln % mxps)))
        {
          /* Just Receive Size */

          /* Peripheral Function */

          end_flag = USB_READING;
        }
      else
        {
          end_flag = USB_READEND;

          /* Set NAK */

          usb_cstd_set_nak(pipe);
        }
    }
  else
    {
      /* Continuous Receive data */

      count = dtln;
      end_flag = USB_READING;
      if (0 == count)
        {
          /* Null Packet receive */

          end_flag = USB_READSHRT;

          /* Select NAK */

          usb_cstd_set_nak(pipe);
        }

      if (0 != (count % mxps))
        {
          /* Null Packet receive */

          end_flag = USB_READSHRT;

          /* Select NAK */

          usb_cstd_set_nak(pipe);
        }
    }

  if (0 == dtln)
    {
      /* 0 length packet */

      /* Clear BVAL */

      hw_usb_set_bclr(pipemode);
    }
  else
    {
      gp_usb_pstd_data[2] = g_buffer;
      gp_usb_pstd_data[pipe] = usb_pstd_read_fifo(count, pipemode,
                                                  gp_usb_pstd_data[pipe]);
    }

  g_usb_pstd_data_cnt[pipe] -= count;

  /* End or Err or Continue */

  return (end_flag);
}

/****************************************************************************
 * Name: usb_pstd_fifo_to_buf
 *
 * Description:
 *  FIFO to BUF
 *
 ****************************************************************************/

void usb_pstd_fifo_to_buf(uint16_t pipe, uint16_t useport)
{
  uint16_t end_flag;

  end_flag = USB_ERROR;

  if (USB_MAXPIPE_NUM < pipe)
    {
      return; /* Error */
    }

  end_flag = usb_pstd_read_data(pipe, useport);

  /* Check FIFO access sequence */

  switch (end_flag)
    {
      case USB_READING:

      /* Continue of data read */

      break;

      case USB_READEND:

      /* End of data read */

      usb_pstd_data_end(pipe, (uint16_t)USB_DATA_OK);
      break;

      case USB_READSHRT:

      /* End of data read */

      usb_pstd_data_end(pipe, (uint16_t)USB_DATA_SHT);
      break;

      case USB_READOVER:

      /* Buffer over */

      usb_pstd_forced_termination(pipe, (uint16_t)USB_DATA_OVR);
      break;

      case USB_ERROR:

      /* FIFO access error */

      usb_pstd_forced_termination(pipe, (uint16_t)USB_DATA_ERR);
      break;

      default:
      usb_pstd_forced_termination(pipe, (uint16_t)USB_DATA_ERR);
      break;
   }
}

/****************************************************************************
 * Name: usb_pstd_brdy_pipe_process
 *
 * Description:
 *  Handle BRDY Pipe for Data
 *
 ****************************************************************************/

void usb_pstd_brdy_pipe_process(uint16_t bitsts)
{
  uint16_t useport;
  uint16_t i;

  /* WAIT_LOOP */

  for (i = USB_MIN_PIPE_NO; i <= USB_MAXPIPE_NUM; i++)
    {
      if (0u != (bitsts & USB_BITSET(i)))
        {
          /* Interrupt check */

          hw_usb_clear_sts_brdy (i);
          hw_usb_clear_status_bemp(i);

          if (USB_NULL != g_p_usb_pstd_pipe[i])
            {
              /* Pipe number to FIFO port select */

              useport = usb_pstd_pipe2fport(i);

              if (USB_CUSE == useport)
                {
                  if (USB_BUF2FIFO == usb_cstd_get_pipe_dir(i))
                    {
                      /* Buffer to FIFO data write */

                      usb_pstd_buf_to_fifo(i, useport);
                    }
                  else
                    {
                      /* FIFO to Buffer data read */

                     usb_pstd_fifo_to_buf(i, useport);
                    }
                }
            }
        }
    }
}

/****************************************************************************
 * Name: usb_pstd_brdy_pipe
 *
 * Description:
 *  Handle BRDY Interupt
 *
 ****************************************************************************/

void usb_pstd_brdy_pipe (uint16_t bitsts, struct rx65n_usbdev_s *priv,
                         struct rx65n_ep_s *privep, uint16_t epno)
{
  uint32_t nbytes;

  if (USB_BRDY0 == (rx65n_getreg16(RX65N_USB_BRDYSTS) & USB_BRDY0))
    {
      hw_usb_clear_sts_brdy (USB_PIPE0);
      g_usb_pstd_data_cnt[0] = CDC_CLASS_DATA_LENGTH;

      switch (usb_pstd_read_data(USB_PIPE0, USB_CUSE))
        {
          /* End of data read */

          case USB_READEND :

          /* Continue */

          /* End of data read */

          case USB_READSHRT :
          hw_usb_clear_brdyenb((uint16_t) USB_PIPE0);
          break;

          /* Continue of data read */

          case USB_READING :

          /* PID = BUF */

          usb_cstd_set_buf((uint16_t) USB_PIPE0);
          break;

          /* FIFO access error */

          case USB_READOVER :

          /* Clear BVAL */

          hw_usb_set_bclr(USB_CUSE);

          /* Control transfer stop(end) */

          usb_pstd_ctrl_end((uint16_t) USB_DATA_OVR);
          break;

          /* FIFO access error */

          case USB_ERROR :

          /* Control transfer stop(end) */

          usb_pstd_ctrl_end((uint16_t) USB_DATA_ERR);
          break;
          default :
          break;
        }
    }
    else
    {
        /* not PIPE0 */

        usb_pstd_brdy_pipe_process(bitsts);
        epno = BULK_IN_EPNUM;
        nbytes = (RX65N_USB_MAXP) - g_usb_pstd_data_cnt[BULK_OUT_PIPE];
        usb_data_write(epno, g_buffer, nbytes);
        rx65n_putreg16(0, RX65N_USB_BRDYSTS);
        privep->head->req.buf = g_buffer;

       /* EP3 is passed, because for read(),
        * the endpoint list should
        * always point to bulk out endpoint
        */

        privep =  &priv->eplist[EP3];
        usb_data_read(g_buffer, RX65N_MAXPACKET_SIZE);
        bytesread = nbytes;

        /* The rx65n_rdrequest() function is invoked here
         * to handle application specific read()
         * and invoke the reqcomplete() function
         * which further, unblocks the semaphore waiting
         * on read()
         * Failing to invoke this function will result,
         * in failiure of application specific read
         *
         */

        rx65n_rdrequest(epno, priv, privep);
        bytesread = 0;
    }
}

/****************************************************************************
 * Name: usb_cstd_get_pid
 *
 * Description:
 *  Get PID
 *
 ****************************************************************************/

static uint16_t usb_cstd_get_pid (uint16_t pipe)
{
  uint16_t buf;

  if (USB_MAXPIPE_NUM < pipe)
    {
      return USB_NULL; /* Error */
    }

  /* PIPE control reg read */

  buf = hw_usb_read_pipectr(pipe);
  return (uint16_t) (buf & USB_PID);
}

/****************************************************************************
 * Name: usb_pstd_bemp_pipe_process
 *
 * Description:
 *  BEMP Interrupt Handling for Pipe
 *
 ****************************************************************************/

void usb_pstd_bemp_pipe_process(uint16_t bitsts)
{
  uint16_t buf;
  uint16_t i;

  /* WAIT_LOOP */

  for (i = USB_MIN_PIPE_NO; i <= USB_PIPE5; i++)
    {
      if (0 != (bitsts & USB_BITSET(i)))
        {
          hw_usb_clear_status_bemp (i);

          /* Interrupt check */

          if ((USB_NULL != g_p_usb_pstd_pipe[i]))
            {
              buf = usb_cstd_get_pid(i);

              /* MAX packet size error ? */

              if (USB_PID_STALL == (buf & USB_PID_STALL))
                {
                   usb_pstd_forced_termination(i, (uint16_t)USB_DATA_STALL);
                }
              else
                {
                  if (USB_INBUFM != (hw_usb_read_pipectr(i) & USB_INBUFM))
                    {
                      usb_pstd_data_end(i, (uint16_t)USB_DATA_NONE);
                    }
                  else
                    {
                      /* set BEMP enable */

                      hw_usb_set_bempenb(i);
                    }
                }
            }
        }
    }

  /* WAIT_LOOP */

  for (i = USB_PIPE6; i <= USB_MAXPIPE_NUM; i++)
    {
      /* Interrupt check */

      if (0 != (bitsts & USB_BITSET(i)))
        {
          if (USB_NULL != g_p_usb_pstd_pipe[i])
            {
              buf = usb_cstd_get_pid(i);

              /* MAX packet size error ? */

              if (USB_PID_STALL == (buf & USB_PID_STALL))
                {
                  usb_pstd_forced_termination(i, (uint16_t)USB_DATA_STALL);
                }
              else
                {
                  /* End of data transfer */

                  usb_pstd_data_end(i, (uint16_t)USB_DATA_NONE);
                }
            }
        }
    }
}

/****************************************************************************
 * Name: usb_pstd_bemp_pipe
 *
 * Description:
 *  BEMP Interrupt Handling
 *
 ****************************************************************************/

void usb_pstd_bemp_pipe (uint16_t bitsts)
{
  if (USB_BEMP0 == (bitsts & USB_BEMP0))
    {
      switch (usb_pstd_write_data(USB_PIPE0, USB_CUSE))
        {
          /* End of data write (not null) */

          case USB_WRITEEND :

          /* Continue */

          /* End of data write */

          case USB_WRITESHRT :

          /* Enable empty interrupt */

          hw_usb_clear_bempenb((uint16_t) USB_PIPE0);
          break;

          /* Continue of data write */

          case USB_WRITING :

          /* PID = BUF */

          usb_cstd_set_buf((uint16_t) USB_PIPE0);
          break;

          /* FIFO access error */

          case USB_ERROR :

         /* Control transfer stop(end) */

         usb_pstd_ctrl_end((uint16_t) USB_DATA_ERR);
         break;

        default :
        break;
        }
    }
  else
    {
      /* BEMP interrupt */

      usb_pstd_bemp_pipe_process(bitsts);
    }
}

/****************************************************************************
 * Name: hw_usb_read_frmnum
 *
 * Description:
 *  Read Framenum
 *
 ****************************************************************************/

static uint16_t hw_usb_read_frmnum ()
{
  return rx65n_getreg16(RX65N_USB_FRMNUM);
}

/****************************************************************************
 * Name: usb_pstd_nrdy_pipe_process
 *
 * Description:
 *  NRDY Pipe Prrocess
 *
 ****************************************************************************/

void usb_pstd_nrdy_pipe_process(uint16_t bitsts)
{
  uint16_t buf;
  uint16_t i;

  /* WAIT_LOOP */

  for (i = USB_MIN_PIPE_NO; i <= USB_MAXPIPE_NUM; i++)
    {
      if (0 != (bitsts & USB_BITSET(i)))
        {
          /* Interrupt check */

          if (USB_NULL != g_p_usb_pstd_pipe[i])
            {
              if (USB_TYPFIELD_ISO == usb_cstd_get_pipe_type(i))
                {
                  /* Wait for About 60ns */

                  buf = hw_usb_read_frmnum();
                  if (USB_OVRN == (buf & USB_OVRN))
                    {
                      /* @1 */

                      /* End of data transfer */

                      usb_pstd_forced_termination(i, (uint16_t)USB_DATA_OVR);
                    }
                  else
                    {
                      /* @2 */

                      /* End of data transfer */

                      usb_pstd_forced_termination(i, (uint16_t)USB_DATA_ERR);
                    }
                }
          else
            {
              /* Non processing. */
            }
            }
        }
    }
}

/****************************************************************************
 * Name: rx65n_usbdev_bottomhalf
 *
 * Description:
 *  BRDY handler in bottom half
 *
 ****************************************************************************/

static void rx65n_usbdev_bottomhalf (void *arg)
{
  struct rx65n_usbdev_s *priv = &g_usbdev;
  uint32_t bottom_half_processing = (uint32_t)arg;
  struct rx65n_ep_s *privep ;

  if (bottom_half_processing == USB_INT_BRDY)
    {
      uint16_t regval;
      uint8_t epno;
      uint16_t brdysts;

      brdysts = USB0.BRDYSTS.WORD;
      regval = rx65n_getreg16(RX65N_USB_PIPECFG);
      regval &= USB_EPNUMFIELD;
      epno = regval;
      privep =  &priv->eplist[epno];
      usb_pstd_brdy_pipe(brdysts, priv, privep, epno);
    }
}

/****************************************************************************
 * Name: usb_pstd_nrdy_pipe
 *
 * Description:
 *  NRDY Interrupt handler
 *
 ****************************************************************************/

void usb_pstd_nrdy_pipe (uint16_t bitsts)
{
  /* Nrdy Pipe interrupt */

  usb_pstd_nrdy_pipe_process(bitsts);
}

/****************************************************************************
 * Name: rx65n_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 ****************************************************************************/

static int rx65n_usbinterrupt(int irq, FAR void *context, FAR void *arg)
{
  struct rx65n_usbdev_s *priv = &g_usbdev;

  uint8_t type;
  uint16_t ists0;
  uint16_t intenb0;
  uint16_t intsts0;
  uint16_t stginfo;
  uint16_t bempsts;
  uint16_t nrdysts;
  static uint32_t connected_times = 0;

  intenb0 = rx65n_getreg16(RX65N_USB_INTENB0);

  /* Read Interrupt Status and mask out interrupts that are not enabled. */

  intsts0 = ((rx65n_getreg16(RX65N_USB_INTSTS0) & intenb0));
  nrdysts = rx65n_getreg16(RX65N_USB_NRDYSTS);
  bempsts = rx65n_getreg16(RX65N_USB_BEMPSTS);

  /* Check VBUS */

  if ((intsts0 & RX65N_USB_INTSTS0_VBINT) == RX65N_USB_INTSTS0_VBINT)
    {
      rx65n_putreg16
       (((~RX65N_USB_INTSTS0_VBINT) & INTSTS0_BIT_VALUES_TO_DETECT),
        RX65N_USB_INTSTS0);

      if (USB_ATTACH == usb_pstd_chk_vbsts())
        {
          priv->attached = 1;
          connected_times ++;
          syslog (LOG_INFO, "NuttX: USB Device Connected. %d\n",
                  connected_times);
          uinfo("Device attached\n");
          usb_pstd_attach_process();    /* USB attach */
        }
      else
        {
          priv->attached = 0;
          syslog (LOG_INFO, "NuttX: USB Device Disconnected. %d\n",
                  connected_times);
          usb_pstd_detach_process();    /* USB detach */
          if (priv->driver)
            {
              CLASS_DISCONNECT(priv->driver, &priv->usbdev);
            }
        }
}

  intsts0 = (rx65n_getreg16(RX65N_USB_INTSTS0));
  if ((intsts0 & RX65N_USB_INTSTS0_DVST) == RX65N_USB_INTSTS0_DVST)
    {
      rx65n_putreg16
        (((~RX65N_USB_INTSTS0_DVST) & INTSTS0_BIT_VALUES_TO_DETECT),
        RX65N_USB_INTSTS0);

  switch ((uint16_t) (intsts0 & USB_DVSQ))
    {
      /* Power state  */

      case USB_DS_POWR :
      break;

      /* Default state  */

      case USB_DS_DFLT :
      uinfo("USB-reset int peri\n");
      usb_pstd_bus_reset();
      break;

      /* Address state  */

      case USB_DS_ADDS :
      break;

      /* Configured state  */

      case USB_DS_CNFG :
      uinfo("Device configuration int peri\n");
      break;

      /* Power suspend state */

      case USB_DS_SPD_POWR :

      /* Continue */

      /* Default suspend state */

      case USB_DS_SPD_DFLT :

      /* Continue */

      /* Address suspend state */

      case USB_DS_SPD_ADDR :

      /* Continue */

      /* Configured Suspend state */

      case USB_DS_SPD_CNFG :
      uinfo("SUSPEND int peri\n");
      usb_pstd_suspend_process();
      break;

      /* Error */

      default :
      break;
   }
   }

  intsts0 = (rx65n_getreg16(RX65N_USB_INTSTS0));
  if ((intsts0 & RX65N_USB_INTSTS0_CTRT) == RX65N_USB_INTSTS0_CTRT)
    {
      rx65n_putreg16
       (((~RX65N_USB_INTSTS0_CTRT) & INTSTS0_BIT_VALUES_TO_DETECT),
        RX65N_USB_INTSTS0);

  stginfo = (uint16_t) ((rx65n_getreg16(RX65N_USB_INTSTS0) & USB_CTSQ));

  if (((USB_CS_RDDS == stginfo) || (USB_CS_WRDS == stginfo))
      || (USB_CS_WRND == stginfo))
  {
    /* Save request register */

    usb_pstd_save_request();
  }

  if (USB_REQ_TYPE_STANDARD == (g_usb_pstd_req_type & USB_BMREQUESTTYPETYPE))
    {
      /* Switch on the control transfer stage (CTSQ). */

  switch (stginfo)
    {
      /* Idle or setup stage */

      uinfo("stginfo =%d\n", stginfo);
      case USB_CS_IDST :
      case USB_CS_RDDS :
      case USB_CS_WRDS :
      case USB_CS_WRSS :

      rx65n_ep0setup(priv);
      break;

      case USB_CS_RDSS :
      usb_pstd_stand_req4();
      break;

      case USB_CS_WRND :
      usb_pstd_stand_req3();
      break;

      /* Control sequence error */

      case USB_CS_SQER :
          usb_pstd_ctrl_end((uint16_t) USB_DATA_ERR);
      break;

      /* Illegal */

      default :
          usb_pstd_ctrl_end((uint16_t) USB_DATA_ERR);
      break;
    }
    }
    else
      {
        /* Vender Specific */

        type = LSBYTE(rx65n_getreg16(RX65N_USB_USBREQ));
        rx65n_ep0setup(priv);
        usb_peri_class_request(type, stginfo);
      }
    }

  ists0 = (uint16_t)(intsts0 & intenb0);

  /* BRDY Interrupt Check */

  if (USB_BRDY == (ists0 & USB_BRDY))
  {
    DEBUGVERIFY(work_queue(HPWORK, &g_usbdev.rx65n_interrupt_bhalf,
                rx65n_usbdev_bottomhalf,
            (void *)USB_INT_BRDY, 0));
  }

  /* BEMP Interrupt Check */

  if ((USB_BEMP == (ists0 & USB_BEMP)))
    {
      usb_pstd_bemp_pipe_process(bempsts);
    }

  /* NRDY Interrupt Check */

  if ((((intsts0 & intenb0) & RX65N_USB_INTSTS0_NRDY)) ==
      RX65N_USB_INTSTS0_NRDY)
    {
      usb_pstd_nrdy_pipe(nrdysts);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_usbinitialize
 *
 * Description:
 *   Initialize USB hardware
 *
 ****************************************************************************/

void up_usbinitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer
   */

  struct rx65n_usbdev_s *priv = &g_usbdev;
  uint32_t reg32;
  uint16_t regval;
  int epno;
  usbtrace(TRACE_DEVINIT, 0);

  /* Enable write to System registers */

  regval = rx65n_getreg16 (RX65N_PRCR_ADDR);

  rx65n_putreg16(RX65N_PRCR_VALUE, RX65N_PRCR_ADDR);
  regval = rx65n_getreg16 (RX65N_PRCR_ADDR);

  /* Clear bit 19 - so that USB module is released from stop state */

  reg32 = getreg32(RX65N_MSTPCRB_ADDR);
  reg32 &= (~RX65N_MSTPCRB_START_STOP_USB);
  putreg32(reg32, RX65N_MSTPCRB_ADDR);

  memset(priv, 0, sizeof(struct rx65n_usbdev_s));
  priv->usbdev.ops   = &g_devops;
  priv->usbdev.ep0   = &priv->eplist[EP0].ep;
  priv->epavail      = RX65N_ENDP_ALLSET & ~RX65N_ENDP_BIT(EP0);

  /* Initialize the endpoint list */

  for (epno = 0; epno < RX65N_NENDPOINTS; epno++)
    {
      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the (physical) endpoint number which is just the index to the
       * endpoint.
       */

      priv->eplist[epno].ep.ops    = &g_epops;
      priv->eplist[epno].dev       = priv;
      priv->eplist[epno].ep.eplog  = epno;

      /* We will use a fixed maxpacket size for all endpoints (perhaps
       * ISOC endpoints could have larger maxpacket???).  A smaller
       * packet size can be selected when the endpoint is configured.
       */

      priv->eplist[epno].ep.maxpacket = RX65N_MAXPACKET_SIZE;
    }

  /* Select a smaller endpoint size for EP0 */

#if RX65N_EP0MAXPACKET < RX65N_MAXPACKET_SIZE
  priv->eplist[EP0].ep.maxpacket = RX65N_EP0MAXPACKET;
#endif

  /* Set SCKE bit in SYSCFG Register */

  regval = rx65n_getreg16 (RX65N_USB_SYSCFG);
  regval |= (RX65N_USB_SYSCFG_SCKE);
  rx65n_putreg16(regval, RX65N_USB_SYSCFG);

  regval = rx65n_getreg16 (RX65N_USB_PHYSLEW);
  regval |= (RX65N_PHYSLEW_VALUE);
  rx65n_putreg16(regval, RX65N_USB_PHYSLEW);

  /* Set USBE bit in SYSCFG register */

  regval = rx65n_getreg16 (RX65N_USB_SYSCFG);
  regval |= (RX65N_USB_SYSCFG_USBE);
  rx65n_putreg16(regval, RX65N_USB_SYSCFG);

  /* Set MBW bit in CFIFOSEL register */

  regval = rx65n_getreg16 (RX65N_USB_CFIFOSEL);
  regval |= (RX65N_USB_CFIFOSEL_MBW_16);
  rx65n_putreg16(regval, RX65N_USB_CFIFOSEL);

  /* Set MBW bit in D0FIFOSEL register */

  regval = rx65n_getreg16 (RX65N_USB_D0FIFOSEL);
  regval |= (RX65N_USB_D0FIFOSEL_MBW_16);
  rx65n_putreg16(regval, RX65N_USB_D0FIFOSEL);

  /* Set MBW bit in D1FIFOSEL register */

  regval = rx65n_getreg16 (RX65N_USB_D1FIFOSEL);
  regval |= (RX65N_USB_D1FIFOSEL_MBW_16);
  rx65n_putreg16(regval, RX65N_USB_D1FIFOSEL);

  /* Set the Interrupt enable bits in INTENB0 register */

  regval = rx65n_getreg16 (RX65N_USB_INTENB0);
  regval |= (RX65N_USB_INTENB0_BEMPE | RX65N_USB_INTENB0_BRDYE |
             RX65N_USB_INTENB0_VBSE | RX65N_USB_INTENB0_DVSE |
             RX65N_USB_INTENB0_CTRE);
  rx65n_putreg16(regval, RX65N_USB_INTENB0);

  ICU.SLIBR185.BYTE = RX65N_USBI0_SOURCE;

  IPR(PERIB, INTB185) = RX65N_USBI0_PRIORITY;

  if (irq_attach(RX65N_INTB185_IRQ, rx65n_usbinterrupt, NULL) != 0)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_IRQREGISTRATION),
              (uint16_t)RX65N_INTB185_IRQ);
    }

  priv->usbdev.speed = USB_SPEED_FULL;

    syslog (LOG_INFO, "Debug:USB Device Initialized, Device connected:%s\n",
            priv->attached ? "YES" : "NO");
}

/****************************************************************************
 * Name: up_usbuninitialize
 * Description:
 *   Initialize the USB driver
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_usbuninitialize(void)
{
  struct rx65n_usbdev_s *priv = &g_usbdev;

  usbtrace(TRACE_DEVUNINIT, 0);
  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable and detach IRQs */

  IEN(PERIB, INTB185) = 0U;

  irq_detach(RX65N_INTB185_IRQ);
}

/****************************************************************************
 * Name: usbdevclass_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's
 *   bind() method will be
 *   called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(FAR struct usbdevclass_driver_s *driver)
{
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_USB
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  g_usbdev.driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_usbdev.driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts */

      IEN(PERIB, INTB185) = 1U;
    }

  return ret;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected
 *   to a USB host, it will first disconnect().  The driver is
 *   also requested to unbind() and clean
 *   up any device state, before this procedure finally returns.
 *
 ****************************************************************************/

int usbdev_unregister(FAR struct usbdevclass_driver_s *driver)
{
  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (driver != g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(RX65N_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_usbdev.usbdev);

  /* Disable IRQs */

  IEN(PERIB, INTB185) = 0U;

  /* Unhook the driver */

  g_usbdev.driver = NULL;
  return OK;
}
