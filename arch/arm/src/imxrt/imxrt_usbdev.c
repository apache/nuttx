/****************************************************************************
 * boards/arm/imxrt/imxrt1060-evk/src/imxrt_usbdev.c
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
 * IMXRT USB Device Driver
 *
 *   Authors: Thomas Axelsson <thomas.axelsson@actia.se>
 *            Simon Åström <simon.astrom@actia.se>
 *
 * Part of the NuttX OS and based, mostly, on the LPC43xx USB driver:
 *
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Which, in turn, was based on the LPC31xx USB driver:
 *
 *   Authors: David Hewson
 *            Gregory Nutt <gnutt@nuttx.org>
 *
 * Which, in turn, was based on the LPC2148 USB driver:
 *
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "hardware/imxrt_usbotg.h"
#include "hardware/imxrt_usbphy.h"
#include "hardware/rt106x/imxrt106x_ccm.h"
#include "imxrt_periphclks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef  CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100  /* mA */
#endif

/* Enable reading SOF from interrupt handler vs. simply reading on demand.
 * Probably a bad idea... Unless there is some issue with sampling the SOF
 * from hardware asynchronously.
 */

#ifdef CONFIG_IMXRT_USBDEV_FRAME_INTERRUPT
#  define USB_FRAME_INT USBDEV_USBINTR_SRE
#else
#  define USB_FRAME_INT 0
#endif

#ifdef CONFIG_DEBUG_FEATURES
#  define USB_ERROR_INT USBDEV_USBINTR_UEE
#else
#  define USB_ERROR_INT 0
#endif

/* Debug ********************************************************************/

/* Trace error codes */

#define IMXRT_TRACEERR_ALLOCFAIL            0x0001
#define IMXRT_TRACEERR_BADCLEARFEATURE      0x0002
#define IMXRT_TRACEERR_BADDEVGETSTATUS      0x0003
#define IMXRT_TRACEERR_BADEPNO              0x0004
#define IMXRT_TRACEERR_BADEPGETSTATUS       0x0005
#define IMXRT_TRACEERR_BADEPTYPE            0x0006
#define IMXRT_TRACEERR_BADGETCONFIG         0x0007
#define IMXRT_TRACEERR_BADGETSETDESC        0x0008
#define IMXRT_TRACEERR_BADGETSTATUS         0x0009
#define IMXRT_TRACEERR_BADSETADDRESS        0x000a
#define IMXRT_TRACEERR_BADSETCONFIG         0x000b
#define IMXRT_TRACEERR_BADSETFEATURE        0x000c
#define IMXRT_TRACEERR_BINDFAILED           0x000d
#define IMXRT_TRACEERR_DISPATCHSTALL        0x000e
#define IMXRT_TRACEERR_DRIVER               0x000f
#define IMXRT_TRACEERR_DRIVERREGISTERED     0x0010
#define IMXRT_TRACEERR_EP0SETUPSTALLED      0x0011
#define IMXRT_TRACEERR_EPINNULLPACKET       0x0012
#define IMXRT_TRACEERR_EPOUTNULLPACKET      0x0013
#define IMXRT_TRACEERR_INVALIDCTRLREQ       0x0014
#define IMXRT_TRACEERR_INVALIDPARMS         0x0015
#define IMXRT_TRACEERR_IRQREGISTRATION      0x0016
#define IMXRT_TRACEERR_NOEP                 0x0017
#define IMXRT_TRACEERR_NOTCONFIGURED        0x0018
#define IMXRT_TRACEERR_REQABORTED           0x0019

/* Trace interrupt codes */

#define IMXRT_TRACEINTID_USB                0x0001
#define IMXRT_TRACEINTID_CLEARFEATURE       0x0002
#define IMXRT_TRACEINTID_DEVGETSTATUS       0x0003
#define IMXRT_TRACEINTID_DEVRESET           0x0004
#define IMXRT_TRACEINTID_DISPATCH           0x0005
#define IMXRT_TRACEINTID_EP0COMPLETE        0x0006
#define IMXRT_TRACEINTID_EP0NAK             0x0007
#define IMXRT_TRACEINTID_EP0SETUP           0x0008
#define IMXRT_TRACEINTID_EPGETSTATUS        0x0009
#define IMXRT_TRACEINTID_EPIN               0x000a
#define IMXRT_TRACEINTID_EPINQEMPTY         0x000b
#define IMXRT_TRACEINTID_EP0INSETADDRESS    0x000c
#define IMXRT_TRACEINTID_EPOUT              0x000d
#define IMXRT_TRACEINTID_EPOUTQEMPTY        0x000e
#define IMXRT_TRACEINTID_EP0SETUPSETADDRESS 0x000f
#define IMXRT_TRACEINTID_FRAME              0x0010
#define IMXRT_TRACEINTID_GETCONFIG          0x0011
#define IMXRT_TRACEINTID_GETSETDESC         0x0012
#define IMXRT_TRACEINTID_GETSETIF           0x0013
#define IMXRT_TRACEINTID_GETSTATUS          0x0014
#define IMXRT_TRACEINTID_IFGETSTATUS        0x0015
#define IMXRT_TRACEINTID_SETCONFIG          0x0016
#define IMXRT_TRACEINTID_SETFEATURE         0x0017
#define IMXRT_TRACEINTID_SUSPENDED          0x0018
#define IMXRT_TRACEINTID_RESUMED            0x0019
#define IMXRT_TRACEINTID_SYNCHFRAME         0x001a

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(IMXRT_TRACEERR_ALLOCFAIL),
  TRACE_STR(IMXRT_TRACEERR_BADCLEARFEATURE),
  TRACE_STR(IMXRT_TRACEERR_BADDEVGETSTATUS),
  TRACE_STR(IMXRT_TRACEERR_BADEPNO),
  TRACE_STR(IMXRT_TRACEERR_BADEPGETSTATUS),
  TRACE_STR(IMXRT_TRACEERR_BADEPTYPE),
  TRACE_STR(IMXRT_TRACEERR_BADGETCONFIG),
  TRACE_STR(IMXRT_TRACEERR_BADGETSETDESC),
  TRACE_STR(IMXRT_TRACEERR_BADGETSTATUS),
  TRACE_STR(IMXRT_TRACEERR_BADSETADDRESS),
  TRACE_STR(IMXRT_TRACEERR_BADSETCONFIG),
  TRACE_STR(IMXRT_TRACEERR_BADSETFEATURE),
  TRACE_STR(IMXRT_TRACEERR_BINDFAILED),
  TRACE_STR(IMXRT_TRACEERR_DISPATCHSTALL),
  TRACE_STR(IMXRT_TRACEERR_DRIVER),
  TRACE_STR(IMXRT_TRACEERR_DRIVERREGISTERED),
  TRACE_STR(IMXRT_TRACEERR_EP0SETUPSTALLED),
  TRACE_STR(IMXRT_TRACEERR_EPINNULLPACKET),
  TRACE_STR(IMXRT_TRACEERR_EPOUTNULLPACKET),
  TRACE_STR(IMXRT_TRACEERR_INVALIDCTRLREQ),
  TRACE_STR(IMXRT_TRACEERR_INVALIDPARMS),
  TRACE_STR(IMXRT_TRACEERR_IRQREGISTRATION),
  TRACE_STR(IMXRT_TRACEERR_NOEP),
  TRACE_STR(IMXRT_TRACEERR_NOTCONFIGURED),
  TRACE_STR(IMXRT_TRACEERR_REQABORTED),
  TRACE_STR_END
};

const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(IMXRT_TRACEINTID_USB),
  TRACE_STR(IMXRT_TRACEINTID_CLEARFEATURE),
  TRACE_STR(IMXRT_TRACEINTID_DEVGETSTATUS),
  TRACE_STR(IMXRT_TRACEINTID_DEVRESET),
  TRACE_STR(IMXRT_TRACEINTID_DISPATCH),
  TRACE_STR(IMXRT_TRACEINTID_EP0COMPLETE),
  TRACE_STR(IMXRT_TRACEINTID_EP0NAK),
  TRACE_STR(IMXRT_TRACEINTID_EP0SETUP),
  TRACE_STR(IMXRT_TRACEINTID_EPGETSTATUS),
  TRACE_STR(IMXRT_TRACEINTID_EPIN),
  TRACE_STR(IMXRT_TRACEINTID_EPINQEMPTY),
  TRACE_STR(IMXRT_TRACEINTID_EP0INSETADDRESS),
  TRACE_STR(IMXRT_TRACEINTID_EPOUT),
  TRACE_STR(IMXRT_TRACEINTID_EPOUTQEMPTY),
  TRACE_STR(IMXRT_TRACEINTID_EP0SETUPSETADDRESS),
  TRACE_STR(IMXRT_TRACEINTID_FRAME),
  TRACE_STR(IMXRT_TRACEINTID_GETCONFIG),
  TRACE_STR(IMXRT_TRACEINTID_GETSETDESC),
  TRACE_STR(IMXRT_TRACEINTID_GETSETIF),
  TRACE_STR(IMXRT_TRACEINTID_GETSTATUS),
  TRACE_STR(IMXRT_TRACEINTID_IFGETSTATUS),
  TRACE_STR(IMXRT_TRACEINTID_SETCONFIG),
  TRACE_STR(IMXRT_TRACEINTID_SETFEATURE),
  TRACE_STR(IMXRT_TRACEINTID_SUSPENDED),
  TRACE_STR(IMXRT_TRACEINTID_RESUMED),
  TRACE_STR(IMXRT_TRACEINTID_SYNCHFRAME),
  TRACE_STR_END
};
#endif

/* Hardware interface *******************************************************/

/* This represents a Endpoint Transfer Descriptor - note these must be 32
 * byte aligned.
 */

struct imxrt_dtd_s
{
  volatile uint32_t       nextdesc;      /* Address of the next DMA descripto in RAM */
  volatile uint32_t       config;        /* Misc. bit encoded configuration information */
  uint32_t                buffer0;       /* Buffer start address */
  uint32_t                buffer1;       /* Buffer start address */
  uint32_t                buffer2;       /* Buffer start address */
  uint32_t                buffer3;       /* Buffer start address */
  uint32_t                buffer4;       /* Buffer start address */
  uint32_t                xfer_len;      /* Software only - transfer len that was queued */
};

/* DTD nextdesc field */

#define DTD_NEXTDESC_INVALID         (1 << 0)    /* Bit 0     : Next Descriptor Invalid. The "Terminate" bit. */

/* DTD config field */

#define DTD_CONFIG_LENGTH(n)         ((n) << 16) /* Bits 16-31 : Total bytes to transfer */
#define DTD_CONFIG_IOC               (1 << 15)   /* Bit 15     : Interrupt on Completion */
#define DTD_CONFIG_MULT_VARIABLE     (0 << 10)   /* Bits 10-11 : Number of packets executed per transacation descriptor (override) */
#define DTD_CONFIG_MULT_NUM(n)       ((n) << 10)
#define DTD_CONFIG_ACTIVE            (1 << 7)    /* Bit 7      : Status Active */
#define DTD_CONFIG_HALTED            (1 << 6)    /* Bit 6      : Status Halted */
#define DTD_CONFIG_BUFFER_ERROR      (1 << 5)    /* Bit 6      : Status Buffer Error */
#define DTD_CONFIG_TRANSACTION_ERROR (1 << 3)    /* Bit 3      : Status Transaction Error */

/* This represents a queue head  - not these must be aligned to a 2048 byte
 * boundary
 */

struct imxrt_dqh_s
{
  uint32_t                capability;  /* Endpoint capability */
  uint32_t                currdesc;    /* Current dTD pointer */
  struct imxrt_dtd_s      overlay;     /* DTD overlay */
  volatile uint32_t       setup[2];    /* Set-up buffer */
  uint32_t                gap[4];      /* align to 64 bytes */
};

/* DQH capability field */

#define DQH_CAPABILITY_MULT_VARIABLE (0 << 30)    /* Bits 30-31 : Number of packets executed per transaction descriptor */
#define DQH_CAPABILITY_MULT_NUM(n)   ((n) << 30)
#define DQH_CAPABILITY_ZLT           (1 << 29)    /* Bit 29     : Zero Length Termination Select */
#define DQH_CAPABILITY_MAX_PACKET(n) ((n) << 16)  /* Bits 16-29 : Maximum packet size of associated endpoint (<1024) */
#define DQH_CAPABILITY_IOS           (1 << 15)    /* Bit 15     : Interrupt on Setup */

/* Endpoints ****************************************************************/

/* Number of endpoints */

#define IMXRT_NLOGENDPOINTS          (8)          /* ep0-7 */
#define IMXRT_NPHYSENDPOINTS         (16)         /* x2 for IN and OUT */

/* Odd physical endpoint numbers are IN; even are OUT */

#define IMXRT_EPPHYIN(epphy)         (((epphy) & 1) != 0)
#define IMXRT_EPPHYOUT(epphy)        (((epphy) & 1) == 0)

#define IMXRT_EPPHYIN2LOG(epphy)     (((uint8_t)(epphy) >> 1)  |USB_DIR_IN)
#define IMXRT_EPPHYOUT2LOG(epphy)    (((uint8_t)(epphy) >> 1) | USB_DIR_OUT)

/* Endpoint 0 is special... */

#define IMXRT_EP0_OUT                (0)
#define IMXRT_EP0_IN                 (1)

/* Each endpoint has somewhat different characteristics */

#define IMXRT_EPALLSET               (0xffff)       /* All endpoints */
#define IMXRT_EPOUTSET               (0x5555)       /* Even phy endpoint numbers are OUT EPs */
#define IMXRT_EPINSET                (0xaaaa)       /* Odd endpoint numbers are IN EPs */
#define IMXRT_EPCTRLSET              (0x0003)       /* EP0 IN/OUT are control endpoints */
#define IMXRT_EPINTRSET              (0xfffc)       /* Interrupt endpoints */
#define IMXRT_EPBULKSET              (0xfffc)       /* Bulk endpoints */
#define IMXRT_EPISOCSET              (0xfffc)       /* Isochronous endpoints */

/* Maximum packet sizes for endpoints */

#define IMXRT_EP0MAXPACKET           (64)         /* EP0 max packet size (1-64) */
#define IMXRT_BULKMAXPACKET          (512)        /* Bulk endpoint max packet (8/16/32/64/512) */
#define IMXRT_INTRMAXPACKET          (1024)       /* Interrupt endpoint max packet (1 to 1024) */
#define IMXRT_ISOCMAXPACKET          (512)        /* Acutally 1..1023 */

/* Endpoint bit position in SETUPSTAT, PRIME, FLUSH, STAT, COMPLETE
 * registers
 */

#define IMXRT_ENDPTSHIFT(epphy)      (IMXRT_EPPHYIN(epphy) ? (16 + ((epphy) >> 1)) : ((epphy) >> 1))
#define IMXRT_ENDPTMASK(epphy)       (1 << IMXRT_ENDPTSHIFT(epphy))
#define IMXRT_ENDPTMASK_ALL          0x00ff00ff

/* Request queue operations *************************************************/

#define imxrt_rqempty(ep)            ((ep)->head == NULL)
#define imxrt_rqpeek(ep)             ((ep)->head)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A container for a request so that the request may be retained in a list */

struct imxrt_req_s
{
  struct usbdev_req_s  req;           /* Standard USB request */
  struct imxrt_req_s  *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct imxrt_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct imxrt_ep_s.
   */

  struct usbdev_ep_s      ep;          /* Standard endpoint structure */

  /* IMXRTXX-specific fields */

  struct imxrt_usbdev_s *dev;          /* Reference to private driver data */
  struct imxrt_req_s    *head;         /* Request list for this endpoint */
  struct imxrt_req_s    *tail;
  uint8_t                epphy;        /* Physical EP address */
  uint8_t                stalled:1;    /* 1: Endpoint is stalled */
};

/* This structure retains the state of the USB device controller */

struct imxrt_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct imxrt_usbdev_s.
   */

  struct usbdev_s         usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* IMXRTXX-specific fields */

  uint8_t                 ep0state;      /* State of certain EP0 operations */
  uint8_t                 ep0buf[64];    /* buffer for EP0 short transfers */
  uint8_t                 paddr;         /* Address assigned by SETADDRESS */
  uint8_t                 stalled:1;     /* 1: Protocol stalled */
  uint8_t                 selfpowered:1; /* 1: Device is self powered */
  uint8_t                 paddrset:1;    /* 1: Peripheral addr has been set */
  uint8_t                 attached:1;    /* 1: Host attached */
  uint8_t                 suspended:1;   /* 1: Suspended */
  uint32_t                softprio;      /* Bitset of high priority interrupts */
  uint32_t                epavail;       /* Bitset of available endpoints */
#ifdef CONFIG_IMXRT_USBDEV_FRAME_INTERRUPT
  uint32_t                sof;           /* Last start-of-frame */
#endif

  uint16_t                ep0buf_len;
  struct usb_ctrlreq_s    ep0ctrl;

  /* The endpoint list */

  struct imxrt_ep_s       eplist[IMXRT_NPHYSENDPOINTS];
};

#define EP0STATE_IDLE             0        /* Idle State, leave on receiving a setup packet or epsubmit */
#define EP0STATE_SETUP_OUT        1        /* Setup Packet received - SET/CLEAR */
#define EP0STATE_SETUP_IN         2        /* Setup Packet received - GET */
#define EP0STATE_SHORTREAD        3        /* Short read without a usb_request */
#define EP0STATE_SHORTWRITE       4        /* Short write without a usb_request */
#define EP0STATE_WAIT_NAK_OUT     5        /* Waiting for Host to illicit status phase (GET) */
#define EP0STATE_WAIT_NAK_IN      6        /* Waiting for Host to illicit status phase (SET/CLEAR) */
#define EP0STATE_WAIT_STATUS_OUT  7        /* Wait for status phase to complete */
#define EP0STATE_WAIT_STATUS_IN   8        /* Wait for status phase to complete */
#define EP0STATE_DATA_IN          9
#define EP0STATE_DATA_OUT         10

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_IMXRT_USBDEV_REGDEBUG
static uint32_t imxrt_getreg(uint32_t addr);
static void imxrt_putreg(uint32_t val, uint32_t addr);
#else
# define imxrt_getreg(addr)     getreg32(addr)
# define imxrt_putreg(val,addr) putreg32(val,addr)
#endif

static inline void imxrt_clrbits(uint32_t mask, uint32_t addr);
static inline void imxrt_setbits(uint32_t mask, uint32_t addr);
static inline void imxrt_chgbits(uint32_t mask, uint32_t val, uint32_t addr);

/* Request queue operations *************************************************/

static FAR struct imxrt_req_s *imxrt_rqdequeue(
    FAR struct imxrt_ep_s *privep);
static bool       imxrt_rqenqueue(FAR struct imxrt_ep_s *privep,
                    FAR struct imxrt_req_s *req);

/* Low level data transfers and request operations **************************/

static inline void imxrt_writedtd(struct imxrt_dtd_s *dtd,
                                  const uint8_t *data,
                                  uint32_t nbytes);
static inline void imxrt_queuedtd(uint8_t epphy, struct imxrt_dtd_s *dtd);
static inline void imxrt_ep0xfer(uint8_t epphy, uint8_t *data,
                                 uint32_t nbytes);
static void        imxrt_readsetup(uint8_t epphy,
                                   struct usb_ctrlreq_s *ctrl);

static inline void imxrt_set_address(struct imxrt_usbdev_s *priv,
                                     uint16_t address);

static void        imxrt_flushep(struct imxrt_ep_s *privep);

static int         imxrt_progressep(struct imxrt_ep_s *privep);
static void        imxrt_reqcomplete(struct imxrt_ep_s *privep,
                     struct imxrt_req_s *privreq, int16_t result);

static void        imxrt_cancelrequests(struct imxrt_ep_s *privep,
                                        int16_t status);

/* Interrupt handling *******************************************************/

static struct imxrt_ep_s *imxrt_epfindbyaddr(struct imxrt_usbdev_s *priv,
                     uint16_t eplog);
static void        imxrt_dispatchrequest(struct imxrt_usbdev_s *priv,
                     const struct usb_ctrlreq_s *ctrl);
static void        imxrt_ep0configure(struct imxrt_usbdev_s *priv);
static void        imxrt_usbreset(struct imxrt_usbdev_s *priv);

static inline void imxrt_ep0state(struct imxrt_usbdev_s *priv,
                                  uint16_t state);
static void        imxrt_ep0setup(struct imxrt_usbdev_s *priv);

static void        imxrt_ep0complete(struct imxrt_usbdev_s *priv,
                                     uint8_t epphy);
static void        imxrt_ep0nak(struct imxrt_usbdev_s *priv, uint8_t epphy);
static bool        imxrt_epcomplete(struct imxrt_usbdev_s *priv,
                                    uint8_t epphy);

static int         imxrt_usbinterrupt(int irq, FAR void *context,
                                      FAR void *arg);

/* Endpoint operations ******************************************************/

/* USB device controller operations *****************************************/

static int         imxrt_epconfigure(FAR struct usbdev_ep_s *ep,
                     const struct usb_epdesc_s *desc, bool last);
static int         imxrt_epdisable(FAR struct usbdev_ep_s *ep);
static FAR struct usbdev_req_s *imxrt_epallocreq(FAR struct usbdev_ep_s *ep);
static void        imxrt_epfreereq(FAR struct usbdev_ep_s *ep,
                     FAR struct usbdev_req_s *);
#ifdef CONFIG_USBDEV_DMA
static void       *imxrt_epallocbuffer(FAR struct usbdev_ep_s *ep,
                     uint16_t bytes);
static void        imxrt_epfreebuffer(FAR struct usbdev_ep_s *ep,
                     FAR void *buf);
#endif
static int         imxrt_epsubmit(FAR struct usbdev_ep_s *ep,
                     struct usbdev_req_s *req);
static int         imxrt_epcancel(FAR struct usbdev_ep_s *ep,
                     struct usbdev_req_s *req);
static int         imxrt_epstall(FAR struct usbdev_ep_s *ep, bool resume);

static FAR struct usbdev_ep_s *imxrt_allocep(FAR struct usbdev_s *dev,
                     uint8_t epno, bool in, uint8_t eptype);
static void        imxrt_freeep(FAR struct usbdev_s *dev,
                                FAR struct usbdev_ep_s *ep);
static int         imxrt_getframe(struct usbdev_s *dev);
static int         imxrt_wakeup(struct usbdev_s *dev);
static int         imxrt_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int         imxrt_pullup(struct usbdev_s *dev, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct imxrt_usbdev_s g_usbdev;

static struct imxrt_dqh_s g_qh[IMXRT_NPHYSENDPOINTS]
                               __attribute__((aligned(2048)));

static struct imxrt_dtd_s g_td[IMXRT_NPHYSENDPOINTS]
                               __attribute__((aligned(32)));

static const struct usbdev_epops_s g_epops =
{
  .configure   = imxrt_epconfigure,
  .disable     = imxrt_epdisable,
  .allocreq    = imxrt_epallocreq,
  .freereq     = imxrt_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = imxrt_epallocbuffer,
  .freebuffer  = imxrt_epfreebuffer,
#endif
  .submit      = imxrt_epsubmit,
  .cancel      = imxrt_epcancel,
  .stall       = imxrt_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = imxrt_allocep,
  .freeep      = imxrt_freeep,
  .getframe    = imxrt_getframe,
  .wakeup      = imxrt_wakeup,
  .selfpowered = imxrt_selfpowered,
  .pullup      = imxrt_pullup,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_getreg
 *
 * Description:
 *   Get the contents of an IMXRT3x register
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_USBDEV_REGDEBUG
static uint32_t imxrt_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              uinfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          uinfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  uinfo("%08x->%08x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: imxrt_putreg
 *
 * Description:
 *   Set the contents of an IMXRT3x register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_USBDEV_REGDEBUG
static void imxrt_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  uinfo("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: imxrt_clrbits
 *
 * Description:
 *   Clear bits in a register
 *
 ****************************************************************************/

static inline void imxrt_clrbits(uint32_t mask, uint32_t addr)
{
  uint32_t reg = imxrt_getreg(addr);
  reg &= ~mask;
  imxrt_putreg(reg, addr);
}

/****************************************************************************
 * Name: imxrt_setbits
 *
 * Description:
 *   Set bits in a register
 *
 ****************************************************************************/

static inline void imxrt_setbits(uint32_t mask, uint32_t addr)
{
  uint32_t reg = imxrt_getreg(addr);
  reg |= mask;
  imxrt_putreg(reg, addr);
}

/****************************************************************************
 * Name: imxrt_chgbits
 *
 * Description:
 *   Change bits in a register
 *
 ****************************************************************************/

static inline void imxrt_chgbits(uint32_t mask, uint32_t val, uint32_t addr)
{
  uint32_t reg = imxrt_getreg(addr);
  reg &= ~mask;
  reg |= val;
  imxrt_putreg(reg, addr);
}

/****************************************************************************
 * Name: imxrt_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 ****************************************************************************/

static FAR struct imxrt_req_s *imxrt_rqdequeue(FAR struct imxrt_ep_s *privep)
{
  FAR struct imxrt_req_s *ret = privep->head;

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
 * Name: imxrt_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 ****************************************************************************/

static bool imxrt_rqenqueue(FAR struct imxrt_ep_s *privep,
                            FAR struct imxrt_req_s *req)
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
 * Name: imxrt_writedtd
 *
 * Description:
 *   Initialise a DTD to transfer the data
 *
 ****************************************************************************/

static inline void imxrt_writedtd(struct imxrt_dtd_s *dtd,
                                  const uint8_t *data,
                                  uint32_t nbytes)
{
  dtd->nextdesc  = DTD_NEXTDESC_INVALID;
  dtd->config    = DTD_CONFIG_LENGTH(nbytes) | DTD_CONFIG_IOC |
      DTD_CONFIG_ACTIVE;
  dtd->buffer0   = ((uint32_t) data);
  dtd->buffer1   = (((uint32_t) data) + 0x1000) & 0xfffff000;
  dtd->buffer2   = (((uint32_t) data) + 0x2000) & 0xfffff000;
  dtd->buffer3   = (((uint32_t) data) + 0x3000) & 0xfffff000;
  dtd->buffer4   = (((uint32_t) data) + 0x4000) & 0xfffff000;
  dtd->xfer_len  = nbytes;

  up_flush_dcache((uintptr_t)dtd,
                  (uintptr_t)dtd + sizeof(struct imxrt_dtd_s));
  up_flush_dcache((uintptr_t)data,
                  (uintptr_t)data + nbytes);
}

/****************************************************************************
 * Name: imxrt_queuedtd
 *
 * Description:
 *   Add the DTD to the device list
 *
 * Assumptions:
 *   DTD is already flushed to RAM.
 *
 ****************************************************************************/

static void imxrt_queuedtd(uint8_t epphy, struct imxrt_dtd_s *dtd)
{
  struct imxrt_dqh_s *dqh = &g_qh[epphy];

  /* Queue the DTD onto the Endpoint
   * NOTE - this only works when no DTD is currently queued
   */

  dqh->overlay.nextdesc = (uint32_t) dtd;
  dqh->overlay.config  &= ~(DTD_CONFIG_ACTIVE | DTD_CONFIG_HALTED);

  up_flush_dcache((uintptr_t)dqh,
                  (uintptr_t)dqh + sizeof(struct imxrt_dqh_s));

  uint32_t bit = IMXRT_ENDPTMASK(epphy);

  imxrt_setbits (bit, IMXRT_USBDEV_ENDPTPRIME);

  while (imxrt_getreg (IMXRT_USBDEV_ENDPTPRIME) & bit)
    ;
}

/****************************************************************************
 * Name: imxrt_ep0xfer
 *
 * Description:
 *   Schedule a short transfer for Endpoint 0 (IN or OUT)
 *
 ****************************************************************************/

static inline void imxrt_ep0xfer(uint8_t epphy, uint8_t *buf,
                                 uint32_t nbytes)
{
  struct imxrt_dtd_s *dtd = &g_td[epphy];

  imxrt_writedtd(dtd, buf, nbytes);

  imxrt_queuedtd(epphy, dtd);
}

/****************************************************************************
 * Name: imxrt_readsetup
 *
 * Description:
 *   Read a Setup packet from the DTD.
 *
 ****************************************************************************/

static void imxrt_readsetup(uint8_t epphy, struct usb_ctrlreq_s *ctrl)
{
  struct imxrt_dqh_s *dqh = &g_qh[epphy];
  int i;

  do
    {
      /* Set the trip wire */

      imxrt_setbits(USBDEV_USBCMD_SUTW, IMXRT_USBDEV_USBCMD);

      up_invalidate_dcache((uintptr_t)dqh,
                           (uintptr_t)dqh + sizeof(struct imxrt_dqh_s));

      /* Copy the request... */

      for (i = 0; i < 8; i++)
        {
          ((uint8_t *) ctrl)[i] = ((uint8_t *) dqh->setup)[i];
        }
    }
  while (!(imxrt_getreg(IMXRT_USBDEV_USBCMD) & USBDEV_USBCMD_SUTW));

  /* Clear the trip wire */

  imxrt_clrbits(USBDEV_USBCMD_SUTW, IMXRT_USBDEV_USBCMD);

  /* Clear the Setup Interrupt */

  imxrt_putreg (IMXRT_ENDPTMASK(IMXRT_EP0_OUT), IMXRT_USBDEV_ENDPTSETUPSTAT);
}

/****************************************************************************
 * Name: imxrt_set_address
 *
 * Description:
 *   Set the devices USB address
 *
 ****************************************************************************/

static inline void imxrt_set_address(struct imxrt_usbdev_s *priv,
                                     uint16_t address)
{
  priv->paddr    = address;
  priv->paddrset = address != 0;

  imxrt_chgbits(USBDEV_DEVICEADDR_MASK,
                priv->paddr << USBDEV_DEVICEADDR_SHIFT,
                IMXRT_USBDEV_DEVICEADDR);
}

/****************************************************************************
 * Name: imxrt_flushep
 *
 * Description:
 *   Flush any primed descriptors from this ep
 *
 ****************************************************************************/

static void imxrt_flushep(struct imxrt_ep_s *privep)
{
  uint32_t mask = IMXRT_ENDPTMASK(privep->epphy);
  do
    {
      imxrt_putreg (mask, IMXRT_USBDEV_ENDPTFLUSH);
      while ((imxrt_getreg(IMXRT_USBDEV_ENDPTFLUSH) & mask) != 0)
      ;
    }
  while ((imxrt_getreg(IMXRT_USBDEV_ENDPTSTATUS) & mask) != 0);
}

/****************************************************************************
 * Name: imxrt_progressep
 *
 * Description:
 *   Progress the Endpoint by priming the first request into the queue head
 *
 ****************************************************************************/

static int imxrt_progressep(struct imxrt_ep_s *privep)
{
  struct imxrt_dtd_s *dtd = &g_td[privep->epphy];
  struct imxrt_req_s *privreq;

  /* Check the request from the head of the endpoint request queue */

  privreq = imxrt_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_EPINQEMPTY), 0);
      return OK;
    }

  /* Ignore any attempt to send a zero length packet */

  if (privreq->req.len == 0)
    {
      /* If the class driver is responding to a setup packet, then wait for
       * the host to illicit thr response
       */

      if (privep->epphy == IMXRT_EP0_IN &&
          privep->dev->ep0state == EP0STATE_SETUP_OUT)
        {
          imxrt_ep0state (privep->dev, EP0STATE_WAIT_NAK_IN);
        }
      else
        {
          if (IMXRT_EPPHYIN(privep->epphy))
            {
              usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_EPINNULLPACKET), 0);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_EPOUTNULLPACKET), 0);
            }
        }

      imxrt_reqcomplete(privep, imxrt_rqdequeue(privep), OK);
      return OK;
    }

  if (privep->epphy == IMXRT_EP0_IN)
    {
      imxrt_ep0state (privep->dev,  EP0STATE_DATA_IN);
    }
  else if (privep->epphy == IMXRT_EP0_OUT)
    {
      imxrt_ep0state (privep->dev, EP0STATE_DATA_OUT);
    }

  int bytesleft = privreq->req.len - privreq->req.xfrd;

  if (IMXRT_EPPHYIN(privep->epphy))
    {
      usbtrace(TRACE_WRITE(privep->epphy), privreq->req.xfrd);
    }
  else
    {
      usbtrace(TRACE_READ(privep->epphy), privreq->req.xfrd);
    }

  /* Initialise the DTD to transfer the next chunk */

  imxrt_writedtd (dtd, privreq->req.buf + privreq->req.xfrd, bytesleft);

  /* Then queue onto the DQH */

  imxrt_queuedtd(privep->epphy, dtd);

  return OK;
}

/****************************************************************************
 * Name: imxrt_reqcomplete
 *
 * Description:
 *   Handle termination of the request at the head of the endpoint request
 *   queue.
 *
 ****************************************************************************/

static void imxrt_reqcomplete(struct imxrt_ep_s *privep,
                              struct imxrt_req_s *privreq, int16_t result)
{
  /* If endpoint 0, temporarily reflect the state of protocol stalled
   * in the callback.
   */

  bool stalled = privep->stalled;
  if (privep->epphy == IMXRT_EP0_IN)
    privep->stalled = privep->dev->stalled;

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);

  /* Restore the stalled indication */

  privep->stalled = stalled;
}

/****************************************************************************
 * Name: imxrt_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 ****************************************************************************/

static void imxrt_cancelrequests(struct imxrt_ep_s *privep, int16_t status)
{
  if (!imxrt_rqempty(privep))
      imxrt_flushep(privep);

  while (!imxrt_rqempty(privep))
    {
      /* FIXME: the entry at the head should be sync'd with the DTD
       * FIXME: only report the error status if the transfer hasn't completed
       */

      usbtrace(TRACE_COMPLETE(privep->epphy),
               (imxrt_rqpeek(privep))->req.xfrd);
      imxrt_reqcomplete(privep, imxrt_rqdequeue(privep), status);
    }
}

/****************************************************************************
 * Name: imxrt_epfindbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 ****************************************************************************/

static struct imxrt_ep_s *imxrt_epfindbyaddr(struct imxrt_usbdev_s *priv,
                         uint16_t eplog)
{
  struct imxrt_ep_s *privep;
  int i;

  /* Endpoint zero is a special case */

  if (USB_EPNO(eplog) == 0)
    {
      return &priv->eplist[0];
    }

  /* Handle the remaining */

  for (i = 1; i < IMXRT_NPHYSENDPOINTS; i++)
    {
      privep = &priv->eplist[i];

      /* Same logical endpoint number? (includes direction bit) */

      if (eplog == privep->ep.eplog)
        {
          /* Return endpoint found */

          return privep;
        }
    }

  /* Return endpoint not found */

  return NULL;
}

/****************************************************************************
 * Name: imxrt_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically
 *   part of the USB interrupt handler.
 *
 ****************************************************************************/

static void imxrt_dispatchrequest(struct imxrt_usbdev_s *priv,
                                    const struct usb_ctrlreq_s *ctrl)
{
  int ret = -EIO;

  usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_DISPATCH), 0);
  if (priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl, priv->ep0buf,
                        priv->ep0buf_len);
    }

  if (ret < 0)
    {
      /* Stall on failure */

      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_DISPATCHSTALL), 0);
      priv->stalled = true;
    }
}

/****************************************************************************
 * Name: imxrt_ep0configure
 *
 * Description:
 *   Reset Usb engine
 *
 ****************************************************************************/

static void imxrt_ep0configure(struct imxrt_usbdev_s *priv)
{
  /* Enable ep0 IN and ep0 OUT */

  g_qh[IMXRT_EP0_OUT].capability =
      (DQH_CAPABILITY_MAX_PACKET(CONFIG_USBDEV_EP0_MAXSIZE) |
          DQH_CAPABILITY_IOS | DQH_CAPABILITY_ZLT);

  g_qh[IMXRT_EP0_IN].capability =
      (DQH_CAPABILITY_MAX_PACKET(CONFIG_USBDEV_EP0_MAXSIZE) |
          DQH_CAPABILITY_IOS | DQH_CAPABILITY_ZLT);

  g_qh[IMXRT_EP0_OUT].currdesc = DTD_NEXTDESC_INVALID;
  g_qh[IMXRT_EP0_IN].currdesc = DTD_NEXTDESC_INVALID;

  up_flush_dcache((uintptr_t)g_qh,
                  (uintptr_t)g_qh + (sizeof(struct imxrt_dqh_s) * 2));

  /* Enable EP0 */

  imxrt_setbits (USBDEV_ENDPTCTRL0_RXE | USBDEV_ENDPTCTRL0_TXE,
                 IMXRT_USBDEV_ENDPTCTRL0);
}

/****************************************************************************
 * Name: imxrt_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 ****************************************************************************/

static void imxrt_usbreset(struct imxrt_usbdev_s *priv)
{
  int epphy;

  /* Disable all endpoints. Control endpoint 0 is always enabled */

  imxrt_clrbits (USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE,
                 IMXRT_USBDEV_ENDPTCTRL1);
  imxrt_clrbits (USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE,
                 IMXRT_USBDEV_ENDPTCTRL2);
  imxrt_clrbits (USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE,
                 IMXRT_USBDEV_ENDPTCTRL3);
  imxrt_clrbits (USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE,
                 IMXRT_USBDEV_ENDPTCTRL4);
  imxrt_clrbits (USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE,
                 IMXRT_USBDEV_ENDPTCTRL5);

  /* Clear all pending interrupts */

  imxrt_putreg (imxrt_getreg(IMXRT_USBDEV_ENDPTNAK),
                IMXRT_USBDEV_ENDPTNAK);
  imxrt_putreg (imxrt_getreg(IMXRT_USBDEV_ENDPTSETUPSTAT),
                IMXRT_USBDEV_ENDPTSETUPSTAT);
  imxrt_putreg (imxrt_getreg(IMXRT_USBDEV_ENDPTCOMPLETE),
                IMXRT_USBDEV_ENDPTCOMPLETE);

  /* Wait for all prime operations to have completed and then flush all
   * DTDs
   */

  while (imxrt_getreg (IMXRT_USBDEV_ENDPTPRIME) != 0)
    ;
  imxrt_putreg (IMXRT_ENDPTMASK_ALL, IMXRT_USBDEV_ENDPTFLUSH);
  while (imxrt_getreg (IMXRT_USBDEV_ENDPTFLUSH))
    ;

  /* Reset endpoints */

  for (epphy = 0; epphy < IMXRT_NPHYSENDPOINTS; epphy++)
    {
      struct imxrt_ep_s *privep = &priv->eplist[epphy];

      imxrt_cancelrequests (privep, -ESHUTDOWN);

      /* Reset endpoint status */

      privep->stalled = false;
    }

  /* Tell the class driver that we are disconnected. The class
   * driver should then accept any new configurations.
   */

  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  /* Set the interrupt Threshold control interval to 0 */

  imxrt_chgbits(USBDEV_USBCMD_ITC_MASK, USBDEV_USBCMD_ITCIMME,
                IMXRT_USBDEV_USBCMD);

  /* Zero out the Endpoint queue heads */

  memset ((void *) g_qh, 0, sizeof (g_qh));
  memset ((void *) g_td, 0, sizeof (g_td));

  up_flush_dcache((uintptr_t)g_qh, (uintptr_t)g_qh + sizeof(g_qh));
  up_flush_dcache((uintptr_t)g_td, (uintptr_t)g_td + sizeof(g_td));

  /* Set USB address to 0 */

  imxrt_set_address (priv, 0);

  /* Initialise the Enpoint List Address */

  imxrt_putreg ((uint32_t)g_qh, IMXRT_USBDEV_ENDPOINTLIST);

  /* EndPoint 0 initialization */

  imxrt_ep0configure(priv);

  /* Enable Device interrupts */

  imxrt_putreg(USB_FRAME_INT | USB_ERROR_INT | USBDEV_USBINTR_NAKE |
               USBDEV_USBINTR_SLE | USBDEV_USBINTR_URE | USBDEV_USBINTR_PCE |
               USBDEV_USBINTR_UE, IMXRT_USBDEV_USBINTR);
}

/****************************************************************************
 * Name: imxrt_setstate
 *
 * Description:
 *   Sets the EP0 state and manages the NAK interrupts
 *
 ****************************************************************************/

static inline void imxrt_ep0state(struct imxrt_usbdev_s *priv,
                                  uint16_t state)
{
  priv->ep0state = state;

  switch (state)
    {
    case EP0STATE_WAIT_NAK_IN:
      imxrt_putreg (IMXRT_ENDPTMASK(IMXRT_EP0_IN), IMXRT_USBDEV_ENDPTNAKEN);
      break;

    case EP0STATE_WAIT_NAK_OUT:
      imxrt_putreg (IMXRT_ENDPTMASK(IMXRT_EP0_OUT), IMXRT_USBDEV_ENDPTNAKEN);
      break;

    default:
      imxrt_putreg(0, IMXRT_USBDEV_ENDPTNAKEN);
      break;
    }
}

/****************************************************************************
 * Name: imxrt_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event. This is logically part of the USB interrupt
 *   handler.  This event occurs when a setup packet is receive on EP0 OUT.
 *
 ****************************************************************************/

static inline void imxrt_ep0setup(struct imxrt_usbdev_s *priv)
{
  struct imxrt_ep_s *privep;
  struct usb_ctrlreq_s *ctrl;
  uint16_t value;
  uint16_t index;
  uint16_t len;

  ctrl = &priv->ep0ctrl;

  /* Terminate any pending requests - since all DTDs will have been retired
   * because of the setup packet.
   */

  imxrt_cancelrequests(&priv->eplist[IMXRT_EP0_OUT], -EPROTO);
  imxrt_cancelrequests(&priv->eplist[IMXRT_EP0_IN],  -EPROTO);

  /* Assume NOT stalled */

  priv->eplist[IMXRT_EP0_OUT].stalled = false;
  priv->eplist[IMXRT_EP0_IN].stalled = false;
  priv->stalled = false;

  /* Read EP0 setup data */

  imxrt_readsetup(IMXRT_EP0_OUT, ctrl);

  /* And extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  index = GETUINT16(ctrl->index);
  len   = GETUINT16(ctrl->len);

  priv->ep0buf_len = len;

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  /* Starting a control request - update state */

  if (ctrl->type & USB_REQ_DIR_IN)
    {
      imxrt_ep0state (priv, EP0STATE_SETUP_IN);
    }
  else
    {
      imxrt_ep0state (priv, EP0STATE_SETUP_OUT);

      if (len > 0)
        {
          imxrt_ep0state(priv, EP0STATE_SHORTREAD);
          imxrt_ep0xfer(IMXRT_EP0_OUT, priv->ep0buf, len);
          return;
        }
    }

  /* Dispatch any non-standard requests */

  if ((ctrl->type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      imxrt_dispatchrequest(priv, ctrl);
    }
  else
    {
      /* Handle standard request.  Pick off the things of interest to the USB
       * device controller driver; pass what is left to the class driver.
       */

      switch (ctrl->req)
        {
        case USB_REQ_GETSTATUS:
          {
            /* type:  device-to-host; recipient = device, interface, endpoint
             * value: 0
             * index: zero interface endpoint
             * len:   2; data = status
             */

            usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_GETSTATUS), 0);
            if (!priv->paddrset || len != 2 ||
                (ctrl->type & USB_REQ_DIR_IN) == 0 || value != 0)
              {
                priv->stalled = true;
              }
            else
              {
                switch (ctrl->type & USB_REQ_RECIPIENT_MASK)
                  {
                  case USB_REQ_RECIPIENT_ENDPOINT:
                    {
                      usbtrace(
                          TRACE_INTDECODE(IMXRT_TRACEINTID_EPGETSTATUS), 0);
                      privep = imxrt_epfindbyaddr(priv, index);
                      if (!privep)
                        {
                          usbtrace(
                              TRACE_DEVERROR(IMXRT_TRACEERR_BADEPGETSTATUS),
                              0);
                          priv->stalled = true;
                        }
                      else
                        {
                          if (privep->stalled)
                            {
                              priv->ep0buf[0] = 1; /* Stalled */
                            }
                          else
                            {
                              priv->ep0buf[0] = 0; /* Not stalled */
                            }

                          priv->ep0buf[1] = 0;

                          imxrt_ep0xfer (IMXRT_EP0_IN, priv->ep0buf, 2);
                          imxrt_ep0state (priv, EP0STATE_SHORTWRITE);
                        }
                    }
                    break;

                  case USB_REQ_RECIPIENT_DEVICE:
                    {
                      if (index == 0)
                        {
                          usbtrace(
                              TRACE_INTDECODE(IMXRT_TRACEINTID_DEVGETSTATUS),
                              0);

                          /* Features:  Remote Wakeup=YES; selfpowered=? */

                          priv->ep0buf[0] =
                              (priv->selfpowered <<
                                  USB_FEATURE_SELFPOWERED) |
                              (1 << USB_FEATURE_REMOTEWAKEUP);
                          priv->ep0buf[1] = 0;

                          imxrt_ep0xfer(IMXRT_EP0_IN, priv->ep0buf, 2);
                          imxrt_ep0state (priv, EP0STATE_SHORTWRITE);
                        }
                      else
                        {
                          usbtrace(
                              TRACE_DEVERROR(IMXRT_TRACEERR_BADDEVGETSTATUS),
                              0);
                          priv->stalled = true;
                        }
                    }
                    break;

                  case USB_REQ_RECIPIENT_INTERFACE:
                    {
                      usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_IFGETSTATUS),
                               0);
                      priv->ep0buf[0] = 0;
                      priv->ep0buf[1] = 0;

                      imxrt_ep0xfer(IMXRT_EP0_IN, priv->ep0buf, 2);
                      imxrt_ep0state (priv, EP0STATE_SHORTWRITE);
                    }
                    break;

                  default:
                    {
                      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_BADGETSTATUS),
                               0);
                      priv->stalled = true;
                    }
                    break;
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

          usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_CLEARFEATURE), 0);
          if ((ctrl->type & USB_REQ_RECIPIENT_MASK) !=
              USB_REQ_RECIPIENT_ENDPOINT)
            {
              imxrt_dispatchrequest(priv, ctrl);
            }
          else if (priv->paddrset != 0 &&
              value == USB_FEATURE_ENDPOINTHALT &&
              len == 0 && (privep = imxrt_epfindbyaddr(priv, index)) != NULL)
            {
              imxrt_epstall(&privep->ep, true);
              imxrt_ep0state (priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_BADCLEARFEATURE), 0);
              priv->stalled = true;
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

          usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_SETFEATURE), 0);
          if (((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE) && value == USB_FEATURE_TESTMODE)
            {
              uinfo("test mode: %d\n", index);
            }
          else if ((ctrl->type & USB_REQ_RECIPIENT_MASK) !=
              USB_REQ_RECIPIENT_ENDPOINT)
            {
              imxrt_dispatchrequest(priv, ctrl);
            }
          else if (priv->paddrset != 0 &&
              value == USB_FEATURE_ENDPOINTHALT &&
              len == 0 && (privep = imxrt_epfindbyaddr(priv, index)) != NULL)
            {
              imxrt_epstall(&privep->ep, false);
              imxrt_ep0state (priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_BADSETFEATURE), 0);
              priv->stalled = true;
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

          usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_EP0SETUPSETADDRESS),
                   value);
          if (((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE) &&
              index == 0 && len == 0 && value < 128)
            {
              /* Save the address.  We cannot actually change to the next
               * address until the completion of the status phase.
               */

              priv->paddr = ctrl->value[0];
              priv->paddrset = false;
              imxrt_ep0state (priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_BADSETADDRESS), 0);
              priv->stalled = true;
            }
        }
        break;

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
          usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_GETSETDESC), 0);
          if ((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE)
            {
              imxrt_dispatchrequest(priv, ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_BADGETSETDESC), 0);
              priv->stalled = true;
            }
        }
        break;

      case USB_REQ_GETCONFIGURATION:
        /* type:  device-to-host; recipient = device
         * value: 0;
         * index: 0;
         * len:   1; data = configuration value
         */

        {
          usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_GETCONFIG), 0);
          if (priv->paddrset &&
              ((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
                  USB_REQ_RECIPIENT_DEVICE) &&
                  value == 0 && index == 0 && len == 1)
            {
              imxrt_dispatchrequest(priv, ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_BADGETCONFIG), 0);
              priv->stalled = true;
            }
        }
        break;

      case USB_REQ_SETCONFIGURATION:
        /* type:  host-to-device; recipient = device
         * value: configuration value
         * index: 0;
         * len:   0; data = none
         */

        {
          usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_SETCONFIG), 0);
          if (((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE) && index == 0 && len == 0)
            {
              imxrt_dispatchrequest(priv, ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_BADSETCONFIG), 0);
              priv->stalled = true;
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
          usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_GETSETIF), 0);
          imxrt_dispatchrequest(priv, ctrl);
        }
        break;

      case USB_REQ_SYNCHFRAME:
        /* type:  device-to-host; recipient = endpoint
         * value: 0
         * index: endpoint;
         * len:   2; data = frame number
         */

        {
          usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_SYNCHFRAME), 0);
        }
        break;

      default:
        {
          usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_INVALIDCTRLREQ), 0);
          priv->stalled = true;
        }
        break;
      }
  }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_EP0SETUPSTALLED),
               priv->ep0state);
      imxrt_epstall(&priv->eplist[IMXRT_EP0_IN].ep, false);
      imxrt_epstall(&priv->eplist[IMXRT_EP0_OUT].ep, false);
    }
}

/****************************************************************************
 * Name: imxrt_ep0complete
 *
 * Description:
 *   Transfer complete handler for Endpoint 0
 *
 ****************************************************************************/

static void imxrt_ep0complete(struct imxrt_usbdev_s *priv, uint8_t epphy)
{
  struct imxrt_ep_s *privep = &priv->eplist[epphy];

  usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_EP0COMPLETE),
           (uint16_t)priv->ep0state);

  switch (priv->ep0state)
    {
    case EP0STATE_DATA_IN:
      if (imxrt_rqempty(privep))
        {
          return;
        }

      if (imxrt_epcomplete (priv, epphy))
        {
          imxrt_ep0state (priv, EP0STATE_WAIT_NAK_OUT);
        }
      break;

    case EP0STATE_DATA_OUT:
      if (imxrt_rqempty(privep))
        {
          return;
        }

      if (imxrt_epcomplete (priv, epphy))
        {
          imxrt_ep0state (priv, EP0STATE_WAIT_NAK_IN);
        }
      break;

    case EP0STATE_SHORTREAD:

      /* Make sure we have updated data after the DMA transfer.
       * This invalidation matches the flush in writedtd().
       */

      up_invalidate_dcache((uintptr_t)priv->ep0buf,
                           (uintptr_t)priv->ep0buf + sizeof(priv->ep0buf));

      imxrt_dispatchrequest(priv, &priv->ep0ctrl);
      imxrt_ep0state (priv, EP0STATE_WAIT_NAK_IN);
      break;

    case EP0STATE_SHORTWRITE:
      imxrt_ep0state (priv, EP0STATE_WAIT_NAK_OUT);
      break;

    case EP0STATE_WAIT_STATUS_IN:
      imxrt_ep0state (priv, EP0STATE_IDLE);

      /* If we've received a SETADDRESS packet, then we set the address
       * now that the status phase has completed
       */

      if (! priv->paddrset && priv->paddr != 0)
        {
          usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_EP0INSETADDRESS),
                   (uint16_t)priv->paddr);
          imxrt_set_address (priv, priv->paddr);
        }

      break;

    case EP0STATE_WAIT_STATUS_OUT:
      imxrt_ep0state (priv, EP0STATE_IDLE);
      break;

    default:
#ifdef CONFIG_DEBUG_FEATURES
      DEBUGASSERT(priv->ep0state != EP0STATE_DATA_IN &&
          priv->ep0state != EP0STATE_DATA_OUT        &&
          priv->ep0state != EP0STATE_SHORTWRITE      &&
          priv->ep0state != EP0STATE_WAIT_STATUS_IN  &&
          priv->ep0state != EP0STATE_WAIT_STATUS_OUT);
#endif
      priv->stalled = true;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_EP0SETUPSTALLED),
               priv->ep0state);
      imxrt_epstall(&priv->eplist[IMXRT_EP0_IN].ep, false);
      imxrt_epstall(&priv->eplist[IMXRT_EP0_OUT].ep, false);
    }
}

/****************************************************************************
 * Name: imxrt_ep0nak
 *
 * Description:
 *   Handle a NAK interrupt on EP0
 *
 ****************************************************************************/

static void imxrt_ep0nak(struct imxrt_usbdev_s *priv, uint8_t epphy)
{
  usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_EP0NAK),
           (uint16_t)priv->ep0state);

  switch (priv->ep0state)
    {
    case EP0STATE_WAIT_NAK_IN:
      imxrt_ep0xfer (IMXRT_EP0_IN, NULL, 0);
      imxrt_ep0state (priv, EP0STATE_WAIT_STATUS_IN);
      break;

    case EP0STATE_WAIT_NAK_OUT:
      imxrt_ep0xfer (IMXRT_EP0_OUT, NULL, 0);
      imxrt_ep0state (priv, EP0STATE_WAIT_STATUS_OUT);
      break;

    default:
#ifdef CONFIG_DEBUG_FEATURES
      DEBUGASSERT(priv->ep0state != EP0STATE_WAIT_NAK_IN &&
                  priv->ep0state != EP0STATE_WAIT_NAK_OUT);
#endif
      priv->stalled = true;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_EP0SETUPSTALLED),
               priv->ep0state);
      imxrt_epstall(&priv->eplist[IMXRT_EP0_IN].ep, false);
      imxrt_epstall(&priv->eplist[IMXRT_EP0_OUT].ep, false);
    }
}

/****************************************************************************
 * Name: imxrt_epcomplete
 *
 * Description:
 *   Transfer complete handler for Endpoints other than 0
 *   returns whether the request at the head has completed
 *
 ****************************************************************************/

bool imxrt_epcomplete(struct imxrt_usbdev_s *priv, uint8_t epphy)
{
  struct imxrt_ep_s  *privep  = &priv->eplist[epphy];
  struct imxrt_req_s *privreq = privep->head;
  struct imxrt_dtd_s *dtd     = &g_td[epphy];

  if (privreq == NULL)        /* This shouldn't really happen */
    {
      if (IMXRT_EPPHYOUT(privep->epphy))
        {
          usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_EPINQEMPTY), 0);
        }
      else
        {
          usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_EPOUTQEMPTY), 0);
        }

      return true;
    }

  /* Make sure we have updated data after the DMA transfer.
   * This invalidation matches the flush in writedtd().
   */

  up_invalidate_dcache((uintptr_t)dtd,
                       (uintptr_t)dtd + sizeof(struct imxrt_dtd_s));
  up_invalidate_dcache((uintptr_t)dtd->buffer0,
                       (uintptr_t)dtd->buffer0 + dtd->xfer_len);

  int xfrd = dtd->xfer_len - (dtd->config >> 16);

  privreq->req.xfrd += xfrd;

  bool complete = true;
  if (IMXRT_EPPHYOUT(privep->epphy))
    {
      /* read(OUT) completes when request filled, or a short transfer is
       * received
       */

      usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_EPIN), complete);
    }
  else
    {
      /* write(IN) completes when request finished, unless we need to
       * terminate with a ZLP
       */

      bool need_zlp = (xfrd == privep->ep.maxpacket) &&
          ((privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0);

      complete = (privreq->req.xfrd >= privreq->req.len && !need_zlp);

      usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_EPOUT), complete);
    }

  /* If the transfer is complete, then dequeue and progress any further
   * queued requests
   */

  if (complete)
    {
      privreq = imxrt_rqdequeue (privep);
    }

  if (!imxrt_rqempty(privep))
    {
      imxrt_progressep(privep);
    }

  /* Now it's safe to call the completion callback as it may well submit a
   * new request
   */

  if (complete)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
      imxrt_reqcomplete(privep, privreq, OK);
    }

  return complete;
}

/****************************************************************************
 * Name: imxrt_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 ****************************************************************************/

static int imxrt_usbinterrupt(int irq, FAR void *context, FAR void *arg)
{
  struct imxrt_usbdev_s *priv = &g_usbdev;
  uint32_t disr;
  uint32_t portsc1;
  uint32_t n;

  usbtrace(TRACE_INTENTRY(IMXRT_TRACEINTID_USB), 0);

  /* Read the interrupts and then clear them */

  disr = imxrt_getreg(IMXRT_USBDEV_USBSTS);
  imxrt_putreg(disr, IMXRT_USBDEV_USBSTS);

  if (disr & USBDEV_USBSTS_URI)
    {
      usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_DEVRESET), 0);

      imxrt_usbreset(priv);

      usbtrace(TRACE_INTEXIT(IMXRT_TRACEINTID_USB), 0);
      return OK;
    }

  /* When the device controller enters a suspend state from an active state,
   * the SLI bit will be set to a one.
   */

  if (!priv->suspended && (disr & USBDEV_USBSTS_SLI) != 0)
    {
      usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_SUSPENDED), 0);

      /* Inform the Class driver of the suspend event */

      priv->suspended = 1;
      if (priv->driver)
        {
          CLASS_SUSPEND(priv->driver, &priv->usbdev);
        }

      /* TODO: Perform power management operations here. */
    }

  /* The device controller clears the SLI bit upon exiting from a suspend
   * state. This bit can also be cleared by software writing a one to it.
   */

  else if (priv->suspended && (disr & USBDEV_USBSTS_SLI) == 0)
    {
      usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_RESUMED), 0);

      /* Inform the Class driver of the resume event */

      priv->suspended = 0;
      if (priv->driver)
        {
          CLASS_RESUME(priv->driver, &priv->usbdev);
        }

      /* TODO: Perform power management operations here. */
    }

  if (disr & USBDEV_USBSTS_PCI)
    {
      portsc1 = imxrt_getreg(IMXRT_USBDEV_PORTSC1);

      if (portsc1 & USBDEV_PRTSC1_HSP)
        priv->usbdev.speed = USB_SPEED_HIGH;
      else
        priv->usbdev.speed = USB_SPEED_FULL;

      if (portsc1 & USBDEV_PRTSC1_FPR)
        {
          /* FIXME: this occurs because of a J-to-K transition detected
           *         while the port is in SUSPEND state - presumambly this
           *         is where the host is resuming the device?
           *
           *  - but do we need to "ack" the interrupt
           */
        }
    }

#ifdef CONFIG_IMXRT_USBDEV_FRAME_INTERRUPT
  if (disr & USBDEV_USBSTS_SRI)
    {
      usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_FRAME), 0);

      uint32_t frindex = imxrt_getreg(IMXRT_USBDEV_FRINDEX);
      uint16_t frame_num =
          (frindex & USBDEV_FRINDEX_LFN_MASK) >> USBDEV_FRINDEX_LFN_SHIFT;

      priv->sof = frame_num;
    }
#endif

  if (disr & USBDEV_USBSTS_UEI)
    {
      /* FIXME: these occur when a transfer results in an error condition
       *        it is set alongside USBINT if the DTD also had its IOC
       *        bit set.
       */
    }

  if (disr & USBDEV_USBSTS_UI)
    {
      /* Handle completion interrupts */

      uint32_t mask = imxrt_getreg (IMXRT_USBDEV_ENDPTCOMPLETE);

      if (mask)
        {
          /* Clear any NAK interrupt and completion interrupts */

          imxrt_putreg (mask, IMXRT_USBDEV_ENDPTNAK);
          imxrt_putreg (mask, IMXRT_USBDEV_ENDPTCOMPLETE);

          if (mask & IMXRT_ENDPTMASK(0))
            {
              imxrt_ep0complete(priv, 0);
            }

          if (mask & IMXRT_ENDPTMASK(1))
            {
              imxrt_ep0complete(priv, 1);
            }

          for (n = 1; n < IMXRT_NLOGENDPOINTS; n++)
            {
              if (mask & IMXRT_ENDPTMASK((n << 1)))
                {
                  imxrt_epcomplete (priv, (n << 1));
                }

              if (mask & IMXRT_ENDPTMASK((n << 1)+1))
                {
                  imxrt_epcomplete(priv, (n << 1)+1);
                }
            }
        }

      /* Handle setup interrupts */

      uint32_t setupstat = imxrt_getreg(IMXRT_USBDEV_ENDPTSETUPSTAT);
      if (setupstat)
        {
          /* Clear the endpoint complete CTRL OUT and IN when a Setup is
           * received
           */

          imxrt_putreg(IMXRT_ENDPTMASK(IMXRT_EP0_IN) |
                       IMXRT_ENDPTMASK(IMXRT_EP0_OUT),
                       IMXRT_USBDEV_ENDPTCOMPLETE);

          if (setupstat & IMXRT_ENDPTMASK(IMXRT_EP0_OUT))
            {
              usbtrace(TRACE_INTDECODE(IMXRT_TRACEINTID_EP0SETUP),
                       setupstat);
              imxrt_ep0setup(priv);
            }
        }
    }

  if (disr & USBDEV_USBSTS_NAKI)
    {
      uint32_t pending = imxrt_getreg(IMXRT_USBDEV_ENDPTNAK) &
          imxrt_getreg(IMXRT_USBDEV_ENDPTNAKEN);
      if (pending)
        {
          /* We shouldn't see NAK interrupts except on Endpoint 0 */

          if (pending & IMXRT_ENDPTMASK(0))
            {
              imxrt_ep0nak(priv, 0);
            }

          if (pending & IMXRT_ENDPTMASK(1))
            {
              imxrt_ep0nak(priv, 1);
            }
        }

      /* Clear the interrupts */

      imxrt_putreg(pending, IMXRT_USBDEV_ENDPTNAK);
    }

  usbtrace(TRACE_INTEXIT(IMXRT_TRACEINTID_USB), 0);
  return OK;
}

/****************************************************************************
 * Endpoint operations
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_epconfigure
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

static int imxrt_epconfigure(FAR struct usbdev_ep_s *ep,
                               FAR const struct usb_epdesc_s *desc,
                               bool last)
{
  FAR struct imxrt_ep_s *privep = (FAR struct imxrt_ep_s *)ep;
  struct imxrt_dqh_s *dqh = &g_qh[privep->epphy];

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);
  DEBUGASSERT(desc->addr == ep->eplog);

  /* Initialise EP capabilities */

  uint16_t maxsize = GETUINT16(desc->mxpacketsize);
  if ((desc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_ISOC)
    {
      dqh->capability = (DQH_CAPABILITY_MAX_PACKET(maxsize) |
                    DQH_CAPABILITY_IOS |
                    DQH_CAPABILITY_ZLT);
    }
  else
    {
      dqh->capability = (DQH_CAPABILITY_MAX_PACKET(maxsize) |
                    DQH_CAPABILITY_ZLT);
    }

  up_flush_dcache((uintptr_t)dqh,
                  (uintptr_t)dqh + sizeof(struct imxrt_dqh_s));

  /* Setup Endpoint Control Register */

  if (IMXRT_EPPHYIN(privep->epphy))
    {
      /* Reset the data toggles */

      uint32_t cfg = USBDEV_ENDPTCTRL_TXR;

      /* Set the endpoint type */

      switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK)
        {
          case USB_EP_ATTR_XFER_CONTROL:
            cfg |= USBDEV_ENDPTCTRL_TXT_CTRL; break;
          case USB_EP_ATTR_XFER_ISOC:
            cfg |= USBDEV_ENDPTCTRL_TXT_ISOC; break;
          case USB_EP_ATTR_XFER_BULK:
            cfg |= USBDEV_ENDPTCTRL_TXT_BULK; break;
          case USB_EP_ATTR_XFER_INT:
            cfg |= USBDEV_ENDPTCTRL_TXT_INTR; break;
        }

      imxrt_chgbits (0xffff0000, cfg,
                     IMXRT_USBDEV_ENDPTCTRL(privep->epphy >> 1));
    }
  else
    {
      /* Reset the data toggles */

      uint32_t cfg = USBDEV_ENDPTCTRL_RXR;

      /* Set the endpoint type */

      switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK)
        {
          case USB_EP_ATTR_XFER_CONTROL:
            cfg |= USBDEV_ENDPTCTRL_RXT_CTRL; break;
          case USB_EP_ATTR_XFER_ISOC:
            cfg |= USBDEV_ENDPTCTRL_RXT_ISOC; break;
          case USB_EP_ATTR_XFER_BULK:
            cfg |= USBDEV_ENDPTCTRL_RXT_BULK; break;
          case USB_EP_ATTR_XFER_INT:
            cfg |= USBDEV_ENDPTCTRL_RXT_INTR; break;
        }

      imxrt_chgbits (0x0000ffff, cfg,
                     IMXRT_USBDEV_ENDPTCTRL(privep->epphy >> 1));
    }

  /* Reset endpoint status */

  privep->stalled = false;

  /* Enable the endpoint */

  if (IMXRT_EPPHYIN(privep->epphy))
    {
      imxrt_setbits (USBDEV_ENDPTCTRL_TXE,
                     IMXRT_USBDEV_ENDPTCTRL(privep->epphy >> 1));
    }
  else
    {
      imxrt_setbits (USBDEV_ENDPTCTRL_RXE,
                     IMXRT_USBDEV_ENDPTCTRL(privep->epphy >> 1));
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 ****************************************************************************/

static int imxrt_epdisable(FAR struct usbdev_ep_s *ep)
{
  FAR struct imxrt_ep_s *privep = (FAR struct imxrt_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPDISABLE, privep->epphy);

  flags = enter_critical_section();

  /* Disable Endpoint */

  if (IMXRT_EPPHYIN(privep->epphy))
    {
      imxrt_clrbits (USBDEV_ENDPTCTRL_TXE,
                     IMXRT_USBDEV_ENDPTCTRL(privep->epphy >> 1));
    }
  else
    {
      imxrt_clrbits (USBDEV_ENDPTCTRL_RXE,
                     IMXRT_USBDEV_ENDPTCTRL(privep->epphy >> 1));
    }

  privep->stalled = true;

  /* Cancel any ongoing activity */

  imxrt_cancelrequests(privep, -ESHUTDOWN);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: imxrt_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 ****************************************************************************/

static FAR struct usbdev_req_s *imxrt_epallocreq(FAR struct usbdev_ep_s *ep)
{
  FAR struct imxrt_req_s *privreq;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  usbtrace(TRACE_EPALLOCREQ, ((FAR struct imxrt_ep_s *)ep)->epphy);

  privreq = (FAR struct imxrt_req_s *)kmm_malloc(sizeof(struct imxrt_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct imxrt_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: imxrt_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 ****************************************************************************/

static void imxrt_epfreereq(FAR struct usbdev_ep_s *ep,
                            FAR struct usbdev_req_s *req)
{
  FAR struct imxrt_req_s *privreq = (FAR struct imxrt_req_s *)req;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((FAR struct imxrt_ep_s *)ep)->epphy);
  kmm_free(privreq);
}

/****************************************************************************
 * Name: imxrt_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *imxrt_epallocbuffer(FAR struct usbdev_ep_s *ep, uint16_t bytes)
{
  /* The USB peripheral DMA is very forgiving, as the dTD allows the buffer
   * to start at any address. Hence, no need for alignment.
   */

  FAR struct imxrt_ep_s *privep = (FAR struct imxrt_ep_s *)ep;

  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);

#ifdef CONFIG_USBDEV_DMAMEMORY
  return usbdev_dma_alloc(bytes);
#else
  return kmm_malloc(bytes);
#endif
}
#endif

/****************************************************************************
 * Name: imxrt_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void imxrt_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf)
{
  FAR struct imxrt_ep_s *privep = (FAR struct imxrt_ep_s *)ep;

  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);

#ifdef CONFIG_USBDEV_DMAMEMORY
  usbdev_dma_free(buf);
#else
  kmm_free(buf);
#endif
}
#endif

/****************************************************************************
 * Name: imxrt_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 ****************************************************************************/

static int imxrt_epsubmit(FAR struct usbdev_ep_s *ep,
                          FAR struct usbdev_req_s *req)
{
  FAR struct imxrt_req_s *privreq = (FAR struct imxrt_req_s *)req;
  FAR struct imxrt_ep_s *privep = (FAR struct imxrt_ep_s *)ep;
  FAR struct imxrt_usbdev_s *priv;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG_FEATURES
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_INVALIDPARMS), 0);
      uinfo("req=%p callback=%p buf=%p ep=%p\n", req,
            req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

  if (!priv->driver || priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_NOTCONFIGURED),
               priv->usbdev.speed);
      return -ESHUTDOWN;
    }

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
      /* Add the new request to the request queue for the endpoint */

      if (IMXRT_EPPHYIN(privep->epphy))
        {
          usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);
        }
      else
        {
          usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);
        }

      if (imxrt_rqenqueue(privep, privreq))
        {
          imxrt_progressep(privep);
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: imxrt_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 ****************************************************************************/

static int imxrt_epcancel(FAR struct usbdev_ep_s *ep,
                          FAR struct usbdev_req_s *req)
{
  FAR struct imxrt_ep_s *privep = (FAR struct imxrt_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, privep->epphy);

  flags = enter_critical_section();

  /* FIXME: if the request is the first, then we need to flush the EP
   *         otherwise just remove it from the list
   *
   *  but ... all other implementations cancel all requests ...
   */

  imxrt_cancelrequests(privep, -ESHUTDOWN);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: imxrt_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 ****************************************************************************/

static int imxrt_epstall(FAR struct usbdev_ep_s *ep, bool resume)
{
  FAR struct imxrt_ep_s *privep = (FAR struct imxrt_ep_s *)ep;
  irqstate_t flags;

  /* STALL or RESUME the endpoint */

  flags = enter_critical_section();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, privep->epphy);

  uint32_t addr    = IMXRT_USBDEV_ENDPTCTRL(privep->epphy >> 1);
  uint32_t ctrl_xs = IMXRT_EPPHYIN(privep->epphy) ?
      USBDEV_ENDPTCTRL_TXS : USBDEV_ENDPTCTRL_RXS;
  uint32_t ctrl_xr = IMXRT_EPPHYIN(privep->epphy) ?
      USBDEV_ENDPTCTRL_TXR : USBDEV_ENDPTCTRL_RXR;

  if (resume)
    {
      privep->stalled = false;

      /* Clear stall and reset the data toggle */

      imxrt_chgbits (ctrl_xs | ctrl_xr, ctrl_xr, addr);
    }
  else
    {
      privep->stalled = true;

      imxrt_setbits (ctrl_xs, addr);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Device operations
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_allocep
 *
 * Description:
 *   Allocate an endpoint matching the parameters.
 *
 * Input Parameters:
 *   eplog  - 7-bit logical endpoint number (direction bit ignored).  Zero
 *            means that any endpoint matching the other requirements will
 *            suffice. The assigned endpoint can be found in the eplog field.
 *   in     - true: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.  One of {USB_EP_ATTR_XFER_ISOC,
 *            USB_EP_ATTR_XFER_BULK, USB_EP_ATTR_XFER_INT}
 *
 ****************************************************************************/

static FAR struct usbdev_ep_s *imxrt_allocep(FAR struct usbdev_s *dev,
                                             uint8_t eplog,
                                             bool in, uint8_t eptype)
{
  FAR struct imxrt_usbdev_s *priv = (FAR struct imxrt_usbdev_s *)dev;
  uint32_t epset = IMXRT_EPALLSET & ~IMXRT_EPCTRLSET;
  irqstate_t flags;
  int epndx = 0;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)eplog);

  /* Ignore any direction bits in the logical address */

  eplog = USB_EPNO(eplog);

  /* A logical address of 0 means that any endpoint will do */

  if (eplog > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the
       * requested 'logical' endpoint.  All of the other checks will still be
       * performed.
       *
       * First, verify that the logical endpoint is in the range supported by
       * by the hardware.
       */

      if (eplog >= IMXRT_NLOGENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_BADEPNO), (uint16_t)eplog);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset &= 3 << (eplog << 1);
    }

  /* Get the subset matching the requested direction */

  if (in)
    {
      epset &= IMXRT_EPINSET;
    }
  else
    {
      epset &= IMXRT_EPOUTSET;
    }

  /* Get the subset matching the requested type */

  switch (eptype)
    {
    case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      epset &= IMXRT_EPINTRSET;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      epset &= IMXRT_EPBULKSET;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
      epset &= IMXRT_EPISOCSET;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint -- not a valid choice */
    default:
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_BADEPTYPE), (uint16_t)eptype);
      return NULL;
    }

  /* Is the resulting endpoint supported by the IMXRT3x? */

  if (epset)
    {
      /* Yes.. now see if any of the request endpoints are available */

      flags = enter_critical_section();
      epset &= priv->epavail;
      if (epset)
        {
          /* Select the lowest bit in the set of matching, available
           * endpoints
           */

          for (epndx = 2; epndx < IMXRT_NPHYSENDPOINTS; epndx++)
            {
              uint32_t bit = 1 << epndx;
              if ((epset & bit) != 0)
                {
                  /* Mark endpoint no longer available */

                  priv->epavail &= ~bit;
                  leave_critical_section(flags);

                  /* And return the pointer to the standard endpoint
                   * structure
                   */

                  return &priv->eplist[epndx].ep;
                }
            }

          /* Shouldn't get here */
        }

      leave_critical_section(flags);
    }

  usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_NOEP), (uint16_t)eplog);
  return NULL;
}

/****************************************************************************
 * Name: imxrt_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 ****************************************************************************/

static void imxrt_freeep(FAR struct usbdev_s *dev,
                         FAR struct usbdev_ep_s *ep)
{
  FAR struct imxrt_usbdev_s *priv = (FAR struct imxrt_usbdev_s *)dev;
  FAR struct imxrt_ep_s *privep = (FAR struct imxrt_ep_s *)ep;
  irqstate_t flags;

  usbtrace(TRACE_DEVFREEEP, (uint16_t)privep->epphy);

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      flags = enter_critical_section();
      priv->epavail |= (1 << privep->epphy);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: imxrt_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 ****************************************************************************/

static int imxrt_getframe(struct usbdev_s *dev)
{
#ifdef CONFIG_IMXRT_USBDEV_FRAME_INTERRUPT
  FAR struct imxrt_usbdev_s *priv = (FAR struct imxrt_usbdev_s *)dev;

  /* Return last valid value of SOF read by the interrupt handler */

  usbtrace(TRACE_DEVGETFRAME, (uint16_t)priv->sof);
  return priv->sof;
#else
  uint32_t frindex = imxrt_getreg(IMXRT_USBDEV_FRINDEX);
  uint16_t frame_num =
      (frindex & USBDEV_FRINDEX_LFN_MASK) >> USBDEV_FRINDEX_LFN_SHIFT;

  /* Return the last frame number detected by the hardware */

  usbtrace(TRACE_DEVGETFRAME, frame_num);

  return (int)(frame_num);
#endif
}

/****************************************************************************
 * Name: imxrt_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 ****************************************************************************/

static int imxrt_wakeup(struct usbdev_s *dev)
{
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, 0);

  flags = enter_critical_section();
  imxrt_setbits(USBDEV_PRTSC1_FPR, IMXRT_USBDEV_PORTSC1);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: imxrt_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature
 *
 ****************************************************************************/

static int imxrt_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  FAR struct imxrt_usbdev_s *priv = (FAR struct imxrt_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Name: imxrt_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 ****************************************************************************/

static int imxrt_pullup(struct usbdev_s *dev, bool enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  irqstate_t flags = enter_critical_section();
  if (enable)
    {
      imxrt_setbits (USBDEV_USBCMD_RS, IMXRT_USBDEV_USBCMD);

#ifdef CONFIG_IMXRT_USB0DEV_NOVBUS
      /* Create a 'false' power event on the USB port so the MAC connects */

      imxrt_clrbits (USBOTG_OTGSC_VD, IMXRT_USBOTG_OTGSC);
      imxrt_setbits (USBOTG_OTGSC_VC, IMXRT_USBOTG_OTGSC);
#endif
    }
  else
    {
      imxrt_clrbits (USBDEV_USBCMD_RS, IMXRT_USBDEV_USBCMD);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_usbinitialize
 *
 * Description:
 *   Initialize USB hardware.
 *
 * Assumptions:
 * - This function is called very early in the initialization sequence
 * - PLL  initialization is not performed here but should been in
 *   the low-level boot logic: USB1 PLL must be configured for operation
 *   at 480MHz
 *
 ****************************************************************************/

void arm_usbinitialize(void)
{
  struct imxrt_usbdev_s *priv = &g_usbdev;
  int i;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Initialize the device state structure */

  memset(priv, 0, sizeof(struct imxrt_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[IMXRT_EP0_IN].ep;
  priv->epavail    = IMXRT_EPALLSET & ~IMXRT_EPCTRLSET;

  /* Initialize the endpoint list */

  for (i = 0; i < IMXRT_NPHYSENDPOINTS; i++)
    {
      uint32_t bit = 1 << i;

      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the physical endpoint number (which is just the index to the
       * endpoint).
       */

      priv->eplist[i].ep.ops       = &g_epops;
      priv->eplist[i].dev          = priv;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      priv->eplist[i].epphy        = i;
      if (IMXRT_EPPHYIN(i))
        {
          priv->eplist[i].ep.eplog = IMXRT_EPPHYIN2LOG(i);
        }
      else
        {
          priv->eplist[i].ep.eplog = IMXRT_EPPHYOUT2LOG(i);
        }

      /* The maximum packet size may depend on the type of endpoint */

      if ((IMXRT_EPCTRLSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = IMXRT_EP0MAXPACKET;
        }
      else if ((IMXRT_EPINTRSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = IMXRT_INTRMAXPACKET;
        }
      else if ((IMXRT_EPBULKSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = IMXRT_BULKMAXPACKET;
        }
      else /* if ((IMXRT_EPISOCSET & bit) != 0) */
        {
          priv->eplist[i].ep.maxpacket = IMXRT_ISOCMAXPACKET;
        }
    }

  /* Clock run */

  imxrt_clockall_usboh3();

  /* Disable USB interrupts */

  imxrt_putreg(0, IMXRT_USBDEV_USBINTR);

  /* Soft reset PHY and enable clock */

  putreg32(USBPHY_CTRL_SFTRST | USBPHY_CTRL_CLKGATE, IMXRT_USBPHY1_CTRL_CLR);

  /* Disconnect device */

  imxrt_pullup(&priv->usbdev, false);

  /* Reset the controller */

  imxrt_setbits (USBDEV_USBCMD_RST, IMXRT_USBDEV_USBCMD);
  while (imxrt_getreg (IMXRT_USBDEV_USBCMD) & USBDEV_USBCMD_RST)
      ;

  /* Power up the PHY (turn off power disable) - USBPHYx_PWDn
   * Manual: The USB PHY Power-Down Register provides overall control of the
   * PHY power state. Before programming this register, the PHY clocks must
   * be enabled in registers USBPHYx_CTRLn and
   * CCM_ANALOG_USBPHYx_PLL_480_CTRLn.
   */

  imxrt_putreg(0, IMXRT_USBPHY1_PWD);

  /* Program the controller to be the USB device controller */

  imxrt_putreg (USBDEV_USBMODE_SDIS | USBDEV_USBMODE_SLOM |
                USBDEV_USBMODE_CM_DEVICE, IMXRT_USBDEV_USBMODE);

  /* Attach USB controller interrupt handler */

  irq_attach(IMXRT_IRQ_USBOTG1, imxrt_usbinterrupt, NULL);
  up_enable_irq(IMXRT_IRQ_USBOTG1);

  leave_critical_section(flags);

  /* Reset/Re-initialize the USB hardware */

  imxrt_usbreset(priv);

  return;
}

/****************************************************************************
 * Name: arm_usbuninitialize
 ****************************************************************************/

void arm_usbuninitialize(void)
{
  struct imxrt_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  flags = enter_critical_section();

  /* Disconnect device */

  imxrt_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable and detach IRQs */

  up_disable_irq(IMXRT_IRQ_USBOTG1);
  irq_detach(IMXRT_IRQ_USBOTG1);

  /* Reset the controller */

  imxrt_setbits (USBDEV_USBCMD_RST, IMXRT_USBDEV_USBCMD);
  while (imxrt_getreg (IMXRT_USBDEV_USBCMD) & USBDEV_USBCMD_RST)
      ;

  /* Turn off USB power and clocking */

  /* Power down the PHY */

  imxrt_putreg(0xffffffff, IMXRT_USBPHY1_PWD);

  /* Stop clock
   * NOTE: This will interfere with USB OTG 2 and should probably be removed
   * if Device or Host code is expanded to support both OTG Cores.
   */

  imxrt_clockoff_usboh3();

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
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  g_usbdev.driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_usbdev.driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts */

      up_enable_irq(IMXRT_IRQ_USBOTG1);
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
  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG_FEATURES
  if (driver != g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(IMXRT_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_usbdev.usbdev);

  /* Disable USB controller interrupts */

  up_disable_irq(IMXRT_IRQ_USBOTG1);

  /* Unhook the driver */

  g_usbdev.driver = NULL;
  return OK;
}

