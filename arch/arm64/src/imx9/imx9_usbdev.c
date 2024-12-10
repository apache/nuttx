/****************************************************************************
 * arch/arm64/src/imx9/imx9_usbdev.c
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
#include <nuttx/spinlock.h>
#include <arch/barriers.h>
#include <arch/board/board.h>

#include <imx9_usbdev.h>
#include "chip.h"
#include "arm64_internal.h"
#include "imx9_ccm.h"
#include "hardware/imx9_memorymap.h"
#include "hardware/imx9_usbotg.h"
#include "hardware/imx9_ccm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(ARMV8A_DCACHE_LINESIZE) || ARMV8A_DCACHE_LINESIZE == 0
#  undef ARMV8A_DCACHE_LINESIZE
#  define ARMV8A_DCACHE_LINESIZE 64
#endif

#define DCACHE_LINEMASK (ARMV8A_DCACHE_LINESIZE - 1)
#define DCACHE_ALIGN_UP(a)   (((a) + DCACHE_LINEMASK) & ~DCACHE_LINEMASK)
#if !defined(CONFIG_ARM64_DCACHE_DISABLE)
#  define cache_aligned_alloc(s) kmm_memalign(ARMV8A_DCACHE_LINESIZE,(s))
#  define CACHE_ALIGNED_DATA     aligned_data(ARMV8A_DCACHE_LINESIZE)
#else
#  define cache_aligned_alloc    kmm_malloc
#  define CACHE_ALIGNED_DATA
#endif

/* Check that both start and length are cache-aligned */

#define IS_CACHE_ALIGNED(x,y) (((uintptr_t)(x) & DCACHE_LINEMASK) == 0 &&        \
                               ((y) & DCACHE_LINEMASK) == 0)

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

#ifdef CONFIG_IMX9_USB_FRAME_INTERRUPT
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

#define IMX9_TRACEERR_ALLOCFAIL            0x0001
#define IMX9_TRACEERR_BADCLEARFEATURE      0x0002
#define IMX9_TRACEERR_BADDEVGETSTATUS      0x0003
#define IMX9_TRACEERR_BADEPNO              0x0004
#define IMX9_TRACEERR_BADEPGETSTATUS       0x0005
#define IMX9_TRACEERR_BADEPTYPE            0x0006
#define IMX9_TRACEERR_BADGETCONFIG         0x0007
#define IMX9_TRACEERR_BADGETSETDESC        0x0008
#define IMX9_TRACEERR_BADGETSTATUS         0x0009
#define IMX9_TRACEERR_BADSETADDRESS        0x000a
#define IMX9_TRACEERR_BADSETCONFIG         0x000b
#define IMX9_TRACEERR_BADSETFEATURE        0x000c
#define IMX9_TRACEERR_BINDFAILED           0x000d
#define IMX9_TRACEERR_DISPATCHSTALL        0x000e
#define IMX9_TRACEERR_DRIVER               0x000f
#define IMX9_TRACEERR_DRIVERREGISTERED     0x0010
#define IMX9_TRACEERR_EP0SETUPSTALLED      0x0011
#define IMX9_TRACEERR_EPINNULLPACKET       0x0012
#define IMX9_TRACEERR_EPOUTNULLPACKET      0x0013
#define IMX9_TRACEERR_INVALIDCTRLREQ       0x0014
#define IMX9_TRACEERR_INVALIDPARMS         0x0015
#define IMX9_TRACEERR_IRQREGISTRATION      0x0016
#define IMX9_TRACEERR_NOEP                 0x0017
#define IMX9_TRACEERR_NOTCONFIGURED        0x0018
#define IMX9_TRACEERR_REQABORTED           0x0019

/* Trace interrupt codes */

#define IMX9_TRACEINTID_USB                0x0001
#define IMX9_TRACEINTID_CLEARFEATURE       0x0002
#define IMX9_TRACEINTID_DEVGETSTATUS       0x0003
#define IMX9_TRACEINTID_DEVRESET           0x0004
#define IMX9_TRACEINTID_DISPATCH           0x0005
#define IMX9_TRACEINTID_EP0COMPLETE        0x0006
#define IMX9_TRACEINTID_EP0NAK             0x0007
#define IMX9_TRACEINTID_EP0SETUP           0x0008
#define IMX9_TRACEINTID_EPGETSTATUS        0x0009
#define IMX9_TRACEINTID_EPIN               0x000a
#define IMX9_TRACEINTID_EPINQEMPTY         0x000b
#define IMX9_TRACEINTID_EP0INSETADDRESS    0x000c
#define IMX9_TRACEINTID_EPOUT              0x000d
#define IMX9_TRACEINTID_EPOUTQEMPTY        0x000e
#define IMX9_TRACEINTID_EP0SETUPSETADDRESS 0x000f
#define IMX9_TRACEINTID_FRAME              0x0010
#define IMX9_TRACEINTID_GETCONFIG          0x0011
#define IMX9_TRACEINTID_GETSETDESC         0x0012
#define IMX9_TRACEINTID_GETSETIF           0x0013
#define IMX9_TRACEINTID_GETSTATUS          0x0014
#define IMX9_TRACEINTID_IFGETSTATUS        0x0015
#define IMX9_TRACEINTID_SETCONFIG          0x0016
#define IMX9_TRACEINTID_SETFEATURE         0x0017
#define IMX9_TRACEINTID_SUSPENDED          0x0018
#define IMX9_TRACEINTID_RESUMED            0x0019
#define IMX9_TRACEINTID_SYNCHFRAME         0x001a

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(IMX9_TRACEERR_ALLOCFAIL),
  TRACE_STR(IMX9_TRACEERR_BADCLEARFEATURE),
  TRACE_STR(IMX9_TRACEERR_BADDEVGETSTATUS),
  TRACE_STR(IMX9_TRACEERR_BADEPNO),
  TRACE_STR(IMX9_TRACEERR_BADEPGETSTATUS),
  TRACE_STR(IMX9_TRACEERR_BADEPTYPE),
  TRACE_STR(IMX9_TRACEERR_BADGETCONFIG),
  TRACE_STR(IMX9_TRACEERR_BADGETSETDESC),
  TRACE_STR(IMX9_TRACEERR_BADGETSTATUS),
  TRACE_STR(IMX9_TRACEERR_BADSETADDRESS),
  TRACE_STR(IMX9_TRACEERR_BADSETCONFIG),
  TRACE_STR(IMX9_TRACEERR_BADSETFEATURE),
  TRACE_STR(IMX9_TRACEERR_BINDFAILED),
  TRACE_STR(IMX9_TRACEERR_DISPATCHSTALL),
  TRACE_STR(IMX9_TRACEERR_DRIVER),
  TRACE_STR(IMX9_TRACEERR_DRIVERREGISTERED),
  TRACE_STR(IMX9_TRACEERR_EP0SETUPSTALLED),
  TRACE_STR(IMX9_TRACEERR_EPINNULLPACKET),
  TRACE_STR(IMX9_TRACEERR_EPOUTNULLPACKET),
  TRACE_STR(IMX9_TRACEERR_INVALIDCTRLREQ),
  TRACE_STR(IMX9_TRACEERR_INVALIDPARMS),
  TRACE_STR(IMX9_TRACEERR_IRQREGISTRATION),
  TRACE_STR(IMX9_TRACEERR_NOEP),
  TRACE_STR(IMX9_TRACEERR_NOTCONFIGURED),
  TRACE_STR(IMX9_TRACEERR_REQABORTED),
  TRACE_STR_END
};

const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(IMX9_TRACEINTID_USB),
  TRACE_STR(IMX9_TRACEINTID_CLEARFEATURE),
  TRACE_STR(IMX9_TRACEINTID_DEVGETSTATUS),
  TRACE_STR(IMX9_TRACEINTID_DEVRESET),
  TRACE_STR(IMX9_TRACEINTID_DISPATCH),
  TRACE_STR(IMX9_TRACEINTID_EP0COMPLETE),
  TRACE_STR(IMX9_TRACEINTID_EP0NAK),
  TRACE_STR(IMX9_TRACEINTID_EP0SETUP),
  TRACE_STR(IMX9_TRACEINTID_EPGETSTATUS),
  TRACE_STR(IMX9_TRACEINTID_EPIN),
  TRACE_STR(IMX9_TRACEINTID_EPINQEMPTY),
  TRACE_STR(IMX9_TRACEINTID_EP0INSETADDRESS),
  TRACE_STR(IMX9_TRACEINTID_EPOUT),
  TRACE_STR(IMX9_TRACEINTID_EPOUTQEMPTY),
  TRACE_STR(IMX9_TRACEINTID_EP0SETUPSETADDRESS),
  TRACE_STR(IMX9_TRACEINTID_FRAME),
  TRACE_STR(IMX9_TRACEINTID_GETCONFIG),
  TRACE_STR(IMX9_TRACEINTID_GETSETDESC),
  TRACE_STR(IMX9_TRACEINTID_GETSETIF),
  TRACE_STR(IMX9_TRACEINTID_GETSTATUS),
  TRACE_STR(IMX9_TRACEINTID_IFGETSTATUS),
  TRACE_STR(IMX9_TRACEINTID_SETCONFIG),
  TRACE_STR(IMX9_TRACEINTID_SETFEATURE),
  TRACE_STR(IMX9_TRACEINTID_SUSPENDED),
  TRACE_STR(IMX9_TRACEINTID_RESUMED),
  TRACE_STR(IMX9_TRACEINTID_SYNCHFRAME),
  TRACE_STR_END
};
#endif

/* Length of allocated EP0 buffer (DCACHE aligned) */

#define EP0_MAXBUFLEN 64

/* Hardware interface *******************************************************/

/* This represents a Endpoint Transfer Descriptor dQH overlay (32 bytes) */

#define IMX9_DTD_S                                                                          \
  volatile uint32_t       nextdesc;      /* Address of the next DMA descripto in RAM */     \
  volatile uint32_t       config;        /* Misc. bit encoded configuration information */  \
  uint32_t                buffer0;       /* Buffer start address */                         \
  uint32_t                buffer1;       /* Buffer start address */                         \
  uint32_t                buffer2;       /* Buffer start address */                         \
  uint32_t                buffer3;       /* Buffer start address */                         \
  uint32_t                buffer4;       /* Buffer start address */                         \
  uint32_t                xfer_len;      /* Software only - transfer len that was queued */ \

struct imx9_dtd_ovl_s
{
  IMX9_DTD_S
};

/* This represents a Endpoint Transfer Descriptor - cache line aligned */

struct imx9_dtd_s
{
  IMX9_DTD_S
} CACHE_ALIGNED_DATA;

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

/* This represents a queue head */

struct imx9_dqh_s
{
  uint32_t                capability; /* Endpoint capability */
  uint32_t                currdesc;   /* Current dTD pointer */
  struct imx9_dtd_ovl_s   overlay;    /* DTD overlay */
  volatile uint32_t       setup[2];   /* Set-up buffer */
} CACHE_ALIGNED_DATA;

/* DQH capability field */

#define DQH_CAPABILITY_MULT_VARIABLE (0 << 30)    /* Bits 30-31 : Number of packets executed per transaction descriptor */
#define DQH_CAPABILITY_MULT_NUM(n)   ((n) << 30)
#define DQH_CAPABILITY_ZLT           (1 << 29)    /* Bit 29     : Zero Length Termination Select */
#define DQH_CAPABILITY_MAX_PACKET(n) ((n) << 16)  /* Bits 16-29 : Maximum packet size of associated endpoint (<1024) */
#define DQH_CAPABILITY_IOS           (1 << 15)    /* Bit 15     : Interrupt on Setup */

/* Endpoints ****************************************************************/

/* Number of endpoints */

#define IMX9_NLOGENDPOINTS          (8)          /* ep0-7 */
#define IMX9_NPHYSENDPOINTS         (16)         /* x2 for IN and OUT */

/* Odd physical endpoint numbers are IN; even are OUT */

#define IMX9_EPPHYIN(epphy)         (((epphy) & 1) != 0)
#define IMX9_EPPHYOUT(epphy)        (((epphy) & 1) == 0)

#define IMX9_EPPHYIN2LOG(epphy)     (((uint8_t)(epphy) >> 1)  |USB_DIR_IN)
#define IMX9_EPPHYOUT2LOG(epphy)    (((uint8_t)(epphy) >> 1) | USB_DIR_OUT)

/* Endpoint 0 is special... */

#define IMX9_EP0_OUT                (0)
#define IMX9_EP0_IN                 (1)

/* Each endpoint has somewhat different characteristics */

#define IMX9_EPALLSET               (0xffff)       /* All endpoints */
#define IMX9_EPOUTSET               (0x5555)       /* Even phy endpoint numbers are OUT EPs */
#define IMX9_EPINSET                (0xaaaa)       /* Odd endpoint numbers are IN EPs */
#define IMX9_EPCTRLSET              (0x0003)       /* EP0 IN/OUT are control endpoints */
#define IMX9_EPINTRSET              (0x000c)       /* Interrupt endpoints */
#define IMX9_EPBULKSET              (0x0ff0)       /* Bulk endpoints */
#define IMX9_EPISOCSET              (0xf000)       /* Isochronous endpoints */

/* Maximum packet sizes for endpoints */

#define IMX9_EP0MAXPACKET           (64)         /* EP0 max packet size (1-64) */
#define IMX9_BULKMAXPACKET          (512)        /* Bulk endpoint max packet (8/16/32/64/512) */
#define IMX9_INTRMAXPACKET          (1024)       /* Interrupt endpoint max packet (1 to 1024) */
#define IMX9_ISOCMAXPACKET          (512)        /* Acutally 1..1023 */

/* Endpoint bit position in SETUPSTAT, PRIME, FLUSH, STAT, COMPLETE
 * registers
 */

#define IMX9_ENDPTSHIFT(epphy)      (IMX9_EPPHYIN(epphy) ? (16 + ((epphy) >> 1)) : ((epphy) >> 1))
#define IMX9_ENDPTMASK(epphy)       (1 << IMX9_ENDPTSHIFT(epphy))
#define IMX9_ENDPTMASK_ALL          0x00ff00ff

/* Request queue operations *************************************************/

#define imx9_rqempty(ep)            ((ep)->head == NULL)
#define imx9_rqpeek(ep)             ((ep)->head)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A container for a request so that the request may be retained in a list */

struct imx9_req_s
{
  struct usbdev_req_s req;         /* Standard USB request */
  struct imx9_req_s *flink;        /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct imx9_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct imx9_ep_s.
   */

  struct usbdev_ep_s ep;           /* Standard endpoint structure */
  spinlock_t         spinlock;     /* Spinlock */

  /* IMX9XX-specific fields */

  struct imx9_usb_s *dev;          /* Reference to private driver data */
  struct imx9_req_s *head;         /* Request list for this endpoint */
  struct imx9_req_s *tail;
  uint8_t epphy;                   /* Physical EP address */
  uint8_t stalled:1;               /* 1: Endpoint is stalled */
};

/* Structure for ep0 short transfers */

struct imx9_ep0_s
{
  uint8_t * const         buf;     /* buffer for EP0 short transfers */
  uint16_t                buf_len; /* buffer length */
  struct usb_ctrlreq_s    ctrl;    /* structure for EP0 short transfers */
  uint8_t                 state;   /* state of certain EP0 operations */
};

/* This structure retains the state of the USB device controller */

struct imx9_usb_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct imx9_usb_s.
   */

  struct usbdev_s              usbdev;
  spinlock_t                   spinlock;      /* Spinlock */

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  const int                    id;            /* Id of the usb controller */
  const uintptr_t              base;          /* Base address of the controller */
  uint8_t                      paddr;         /* Address assigned by SETADDRESS */
  uint8_t                      stalled:1;     /* 1: Protocol stalled */
  uint8_t                      selfpowered:1; /* 1: Device is self powered */
  uint8_t                      paddrset:1;    /* 1: Peripheral addr has been set */
  uint8_t                      attached:1;    /* 1: Host attached */
  uint8_t                      suspended:1;   /* 1: Suspended */
  uint32_t                     softprio;      /* Bitset of high priority interrupts */
  uint32_t                     epavail;       /* Bitset of available endpoints */
#ifdef CONFIG_IMX9_USB_FRAME_INTERRUPT
  uint32_t                     sof;           /* Last start-of-frame */
#endif

  struct imx9_ep0_s            ep0;           /* ep0 */

  /* The endpoint list */

  struct imx9_ep_s             eplist[IMX9_NPHYSENDPOINTS];
  struct imx9_dqh_s * const    qh;
  struct imx9_dtd_s * const    td;
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

#ifdef CONFIG_IMX9_USB_REGDEBUG
static uint32_t imx9_getreg(struct imx9_usb_s *priv, off_t offset);
static void imx9_putreg(struct imx9_usb_s *priv, off_t offset, uint32_t val);
#else
#  define imx9_getreg(priv, offset)      getreg32(priv->base + offset)
#  define imx9_putreg(priv, offset, val) putreg32(val,priv->base + offset)
#endif

static inline void imx9_modifyreg(struct imx9_usb_s *priv,
                                  off_t offset,
                                  uint32_t clear,
                                  uint32_t set);

/* Request queue operations *************************************************/

static struct imx9_req_s *imx9_rqdequeue(
    struct imx9_ep_s *privep);
static bool       imx9_rqenqueue(struct imx9_ep_s *privep,
                    struct imx9_req_s *req);

/* Low level data transfers and request operations **************************/

static inline void imx9_writedtd(struct imx9_dtd_s *dtd,
                                 const uint8_t *data,
                                 uint32_t nbytes);
static inline void imx9_queuedtd(struct imx9_usb_s *priv, uint8_t epphy,
                                 struct imx9_dtd_s *dtd);
static inline void imx9_ep0xfer(struct imx9_usb_s *priv, uint8_t epphy,
                                uint8_t *data,
                                uint32_t nbytes);
static void imx9_readsetup(struct imx9_usb_s *priv, uint8_t epphy,
                           struct usb_ctrlreq_s *ctrl);
static inline void imx9_set_address(struct imx9_usb_s *priv,
                                    uint16_t address);

static void imx9_flushep(struct imx9_ep_s *privep);

static int imx9_progressep(struct imx9_ep_s *privep);
static void imx9_reqcomplete(struct imx9_ep_s *privep,
                             struct imx9_req_s *privreq, int16_t result);

static void imx9_cancelrequests(struct imx9_ep_s *privep,
                                int16_t status);

/* Interrupt handling *******************************************************/

static struct imx9_ep_s *imx9_epfindbyaddr(struct imx9_usb_s *priv,
                     uint16_t eplog);
static void imx9_dispatchrequest(struct imx9_usb_s *priv,
                                 const struct usb_ctrlreq_s *ctrl);
static void imx9_ep0configure(struct imx9_usb_s *priv);
static void imx9_usbreset(struct imx9_usb_s *priv);
static inline void imx9_ep0state(struct imx9_usb_s *priv,
                                 uint16_t state);
static void imx9_ep0setup(struct imx9_usb_s *priv);
static void imx9_ep0complete(struct imx9_usb_s *priv,
                             uint8_t epphy);
static void imx9_ep0nak(struct imx9_usb_s *priv, uint8_t epphy);
static bool imx9_epcomplete(struct imx9_usb_s *priv,
                            uint8_t epphy);
static int imx9_usbinterrupt(int irq, void *context,
                             void *arg);

/* Endpoint operations ******************************************************/

/* USB device controller operations *****************************************/

static int         imx9_epconfigure(struct usbdev_ep_s *ep,
                     const struct usb_epdesc_s *desc, bool last);
static int         imx9_epdisable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *imx9_epallocreq(struct usbdev_ep_s *ep);
static void        imx9_epfreereq(struct usbdev_ep_s *ep,
                     struct usbdev_req_s *);
#ifdef CONFIG_USBDEV_DMA
static void       *imx9_epallocbuffer(struct usbdev_ep_s *ep,
                     uint16_t bytes);
static void        imx9_epfreebuffer(struct usbdev_ep_s *ep,
                     void *buf);
#endif
static int         imx9_epsubmit(struct usbdev_ep_s *ep,
                     struct usbdev_req_s *req);
static int         imx9_epcancel(struct usbdev_ep_s *ep,
                     struct usbdev_req_s *req);
static int         imx9_epstall(struct usbdev_ep_s *ep, bool resume);

static struct usbdev_ep_s *imx9_allocep(struct usbdev_s *dev,
                     uint8_t epno, bool in, uint8_t eptype);
static void        imx9_freeep(struct usbdev_s *dev,
                                struct usbdev_ep_s *ep);
static int         imx9_getframe(struct usbdev_s *dev);
static int         imx9_wakeup(struct usbdev_s *dev);
static int         imx9_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int         imx9_pullup(struct usbdev_s *dev, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Note: dqh lists must be aligned to a 2048 byte
 * boundary, since ENDPTLISTADDR register last 10 bits are tied to 0
 */

#ifdef CONFIG_IMX9_USBDEV_USBC1
static uint8_t g_usb0_ep0buf[EP0_MAXBUFLEN] CACHE_ALIGNED_DATA;
static struct imx9_dqh_s g_usb0_qh[IMX9_NPHYSENDPOINTS] aligned_data(2048);
static struct imx9_dtd_s g_usb0_td[IMX9_NPHYSENDPOINTS];
#endif

#ifdef CONFIG_IMX9_USBDEV_USBC2
static uint8_t g_usb1_ep0buf[EP0_MAXBUFLEN] CACHE_ALIGNED_DATA;
static struct imx9_dqh_s g_usb1_qh[IMX9_NPHYSENDPOINTS] aligned_data(2048);
static struct imx9_dtd_s g_usb1_td[IMX9_NPHYSENDPOINTS];
#endif

static struct imx9_usb_s g_usbdev[] =
{
#ifdef CONFIG_IMX9_USBDEV_USBC1
  {
    .id = 0,
    .base = IMX9_USB_OTG1_BASE,
    .ep0.buf = g_usb0_ep0buf,
    .qh = g_usb0_qh,
    .td = g_usb0_td,
  },
#endif

#ifdef CONFIG_IMX9_USBDEV_USBC2
  {
    .id = 1,
    .base = IMX9_USB_OTG2_BASE,
    .ep0.buf = g_usb1_ep0buf,
    .qh = g_usb1_qh,
    .td = g_usb1_td,
  },
#endif
};

static const int n_usbdevs =  sizeof(g_usbdev) / sizeof(g_usbdev[0]);

static const struct usbdev_epops_s g_epops =
{
  .configure   = imx9_epconfigure,
  .disable     = imx9_epdisable,
  .allocreq    = imx9_epallocreq,
  .freereq     = imx9_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = imx9_epallocbuffer,
  .freebuffer  = imx9_epfreebuffer,
#endif
  .submit      = imx9_epsubmit,
  .cancel      = imx9_epcancel,
  .stall       = imx9_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = imx9_allocep,
  .freeep      = imx9_freeep,
  .getframe    = imx9_getframe,
  .wakeup      = imx9_wakeup,
  .selfpowered = imx9_selfpowered,
  .pullup      = imx9_pullup,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_getreg
 *
 * Description:
 *   Get the contents of an IMX93x register
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_USB_REGDEBUG
static uint32_t imx9_getreg(struct imx9_usb_s *priv, off_t offset)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(priv->base + offset);

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

  uinfo("%08x->%08x\n", priv->base + offset, val);
  return val;
}
#endif

/****************************************************************************
 * Name: imx9_putreg
 *
 * Description:
 *   Set the contents of an IMX93x register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_IMX9_USB_REGDEBUG
static void imx9_putreg(struct imx9_usb_s *priv, off_t offset, uint32_t val)
{
  /* Show the register value being written */

  uinfo("%08x<-%08x\n", priv->base + offset, val);

  /* Write the value */

  putreg32(val, priv->base + offset);
}
#endif

/****************************************************************************
 * Name: imx9_modifyreg
 *
 * Description:
 *   Change bits in a register
 *
 ****************************************************************************/

static inline void imx9_modifyreg(struct imx9_usb_s *priv, off_t offset,
                                  uint32_t clear, uint32_t set)
{
  modifyreg32(priv->base + offset, clear, set);
}

/****************************************************************************
 * Name: imx9_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 ****************************************************************************/

static struct imx9_req_s *imx9_rqdequeue(struct imx9_ep_s *privep)
{
  struct imx9_req_s *ret = privep->head;

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
 * Name: imx9_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 ****************************************************************************/

static bool imx9_rqenqueue(struct imx9_ep_s *privep,
                            struct imx9_req_s *req)
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
 * Name: imx9_writedtd
 *
 * Description:
 *   Initialise a DTD to transfer the data
 *
 ****************************************************************************/

static inline void imx9_writedtd(struct imx9_dtd_s *dtd,
                                  const uint8_t *data,
                                  uint32_t nbytes)
{
  DEBUGASSERT(IS_CACHE_ALIGNED(dtd, sizeof(*dtd)));

  dtd->nextdesc  = DTD_NEXTDESC_INVALID;
  dtd->config    = DTD_CONFIG_LENGTH(nbytes) | DTD_CONFIG_IOC |
      DTD_CONFIG_ACTIVE;
  dtd->buffer0   = (uint32_t)((uintptr_t) data);
  dtd->buffer1   = (uint32_t)(((uintptr_t) data) + 0x1000) & 0xfffff000;
  dtd->buffer2   = (uint32_t)(((uintptr_t) data) + 0x2000) & 0xfffff000;
  dtd->buffer3   = (uint32_t)(((uintptr_t) data) + 0x3000) & 0xfffff000;
  dtd->buffer4   = (uint32_t)(((uintptr_t) data) + 0x4000) & 0xfffff000;
  dtd->xfer_len  = nbytes;

  up_clean_dcache((uintptr_t)dtd,
                  (uintptr_t)dtd + sizeof(struct imx9_dtd_s));

  /* the pointer is NULL for EP0 NAK IN/OUT */

  if (data != NULL)
    {
      DEBUGASSERT(IS_CACHE_ALIGNED(data, DCACHE_ALIGN_UP(nbytes)));
      DEBUGASSERT(nbytes <= 0x5000);

      /* Data needs to be clean before TX, clean or invalid before RX */

      up_clean_dcache((uintptr_t)data, (uintptr_t)data + nbytes);
    }
}

/****************************************************************************
 * Name: imx9_queuedtd
 *
 * Description:
 *   Add the DTD to the device list
 *
 * Assumptions:
 *   DTD is already flushed to RAM.
 *
 ****************************************************************************/

static void imx9_queuedtd(struct imx9_usb_s *priv, uint8_t epphy,
                          struct imx9_dtd_s *dtd)
{
  struct imx9_dqh_s *dqh = &priv->qh[epphy];

  /* Queue the DTD onto the Endpoint
   * NOTE - this only works when no DTD is currently queued
   */

  dqh->overlay.nextdesc = (uint32_t)(uintptr_t)dtd;
  dqh->overlay.config  &= ~(DTD_CONFIG_ACTIVE | DTD_CONFIG_HALTED);

  up_clean_dcache((uintptr_t)dqh,
                  (uintptr_t)dqh + sizeof(struct imx9_dqh_s));

  uint32_t bit = IMX9_ENDPTMASK(epphy);

  imx9_modifyreg(priv, IMX9_USBDEV_ENDPTPRIME_OFFSET, 0, bit);

  while (imx9_getreg(priv, IMX9_USBDEV_ENDPTPRIME_OFFSET) & bit);
}

/****************************************************************************
 * Name: imx9_ep0xfer
 *
 * Description:
 *   Schedule a short transfer for Endpoint 0 (IN or OUT)
 *
 ****************************************************************************/

static inline void imx9_ep0xfer(struct imx9_usb_s *priv, uint8_t epphy,
                                uint8_t *buf, uint32_t nbytes)
{
  struct imx9_dtd_s *dtd = &priv->td[epphy];

  imx9_writedtd(dtd, buf, nbytes);

  imx9_queuedtd(priv, epphy, dtd);
}

/****************************************************************************
 * Name: imx9_readsetup
 *
 * Description:
 *   Read a Setup packet from the DTD.
 *
 ****************************************************************************/

static void imx9_readsetup(struct imx9_usb_s *priv, uint8_t epphy,
                           struct usb_ctrlreq_s *ctrl)
{
  struct imx9_dqh_s *dqh = &priv->qh[epphy];
  int i;

  /* Set the trip wire */

  imx9_modifyreg(priv, IMX9_USBDEV_USBCMD_OFFSET, 0, USBDEV_USBCMD_SUTW);
  UP_DSB();

  DEBUGASSERT(IS_CACHE_ALIGNED(dqh, sizeof(struct imx9_dqh_s)));
  up_invalidate_dcache((uintptr_t)dqh,
                       (uintptr_t)dqh + sizeof(struct imx9_dqh_s));

  /* Copy the request... */

  for (i = 0; i < 8; i++)
    {
      ((uint8_t *) ctrl)[i] = ((uint8_t *) dqh->setup)[i];
    }

  while (!(imx9_getreg(priv,
                       IMX9_USBDEV_USBCMD_OFFSET) & USBDEV_USBCMD_SUTW));

  /* Clear the trip wire */

  imx9_modifyreg(priv, IMX9_USBDEV_USBCMD_OFFSET, USBDEV_USBCMD_SUTW, 0);

  /* Clear the Setup Interrupt */

  imx9_putreg(priv, IMX9_USBDEV_ENDPTSETUPSTAT_OFFSET,
              IMX9_ENDPTMASK(IMX9_EP0_OUT));
  UP_DSB();
}

/****************************************************************************
 * Name: imx9_set_address
 *
 * Description:
 *   Set the devices USB address
 *
 ****************************************************************************/

static inline void imx9_set_address(struct imx9_usb_s *priv,
                                    uint16_t address)
{
  priv->paddr    = address;
  priv->paddrset = address != 0;

  imx9_modifyreg(priv, IMX9_USBDEV_DEVICEADDR_OFFSET, USBDEV_DEVICEADDR_MASK,
                 priv->paddr << USBDEV_DEVICEADDR_SHIFT);
}

/****************************************************************************
 * Name: imx9_flushep
 *
 * Description:
 *   Flush any primed descriptors from this ep
 *
 ****************************************************************************/

static void imx9_flushep(struct imx9_ep_s *privep)
{
  uint32_t mask = IMX9_ENDPTMASK(privep->epphy);
  struct imx9_usb_s *priv = privep->dev;

  do
    {
      imx9_putreg(priv, IMX9_USBDEV_ENDPTFLUSH_OFFSET, mask);
      while ((imx9_getreg(priv, IMX9_USBDEV_ENDPTFLUSH_OFFSET) & mask) != 0);
    }
  while ((imx9_getreg(priv, IMX9_USBDEV_ENDPTSTATUS_OFFSET) & mask) != 0);
}

/****************************************************************************
 * Name: imx9_progressep
 *
 * Description:
 *   Progress the Endpoint by priming the first request into the queue head
 *
 ****************************************************************************/

static int imx9_progressep(struct imx9_ep_s *privep)
{
  struct imx9_usb_s *priv = privep->dev;
  struct imx9_dtd_s *dtd = &priv->td[privep->epphy];
  struct imx9_req_s *privreq;

  /* Check the request from the head of the endpoint request queue */

  privreq = imx9_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_EPINQEMPTY), 0);
      return OK;
    }

  /* Ignore any attempt to send a zero length packet */

  if (privreq->req.len == 0)
    {
      /* If the class driver is responding to a setup packet, then wait for
       * the host to illicit the response
       */

      if (privep->epphy == IMX9_EP0_IN &&
          privep->dev->ep0.state == EP0STATE_SETUP_OUT)
        {
          imx9_ep0state(priv, EP0STATE_WAIT_NAK_IN);
        }
      else
        {
          if (IMX9_EPPHYIN(privep->epphy))
            {
              usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_EPINNULLPACKET), 0);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_EPOUTNULLPACKET), 0);
            }
        }

      imx9_reqcomplete(privep, imx9_rqdequeue(privep), OK);
      return OK;
    }

  if (privep->epphy == IMX9_EP0_IN)
    {
      imx9_ep0state(priv,  EP0STATE_DATA_IN);
    }
  else if (privep->epphy == IMX9_EP0_OUT)
    {
      imx9_ep0state(priv, EP0STATE_DATA_OUT);
    }

  int bytesleft = privreq->req.len - privreq->req.xfrd;

  if (IMX9_EPPHYIN(privep->epphy))
    {
      usbtrace(TRACE_WRITE(privep->epphy), privreq->req.xfrd);
    }
  else
    {
      usbtrace(TRACE_READ(privep->epphy), privreq->req.xfrd);
    }

  /* Initialise the DTD to transfer the next chunk */

  imx9_writedtd(dtd, privreq->req.buf + privreq->req.xfrd, bytesleft);

  /* Then queue onto the DQH */

  imx9_queuedtd(priv, privep->epphy, dtd);

  return OK;
}

/****************************************************************************
 * Name: imx9_reqcomplete
 *
 * Description:
 *   Handle termination of the request at the head of the endpoint request
 *   queue.
 *
 ****************************************************************************/

static void imx9_reqcomplete(struct imx9_ep_s *privep,
                              struct imx9_req_s *privreq, int16_t result)
{
  /* If endpoint 0, temporarily reflect the state of protocol stalled
   * in the callback.
   */

  bool stalled = privep->stalled;
  if (privep->epphy == IMX9_EP0_IN)
    privep->stalled = privep->dev->stalled;

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);

  /* Restore the stalled indication */

  privep->stalled = stalled;
}

/****************************************************************************
 * Name: imx9_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 ****************************************************************************/

static void imx9_cancelrequests(struct imx9_ep_s *privep, int16_t status)
{
  if (!imx9_rqempty(privep))
      imx9_flushep(privep);

  while (!imx9_rqempty(privep))
    {
      /* FIXME: the entry at the head should be sync'd with the DTD
       * FIXME: only report the error status if the transfer hasn't completed
       */

      usbtrace(TRACE_COMPLETE(privep->epphy),
               (imx9_rqpeek(privep))->req.xfrd);
      imx9_reqcomplete(privep, imx9_rqdequeue(privep), status);
    }
}

/****************************************************************************
 * Name: imx9_epfindbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 ****************************************************************************/

static struct imx9_ep_s *imx9_epfindbyaddr(struct imx9_usb_s *priv,
                         uint16_t eplog)
{
  struct imx9_ep_s *privep;
  int i;

  /* Endpoint zero is a special case */

  if (USB_EPNO(eplog) == 0)
    {
      return &priv->eplist[0];
    }

  /* Handle the remaining */

  for (i = 1; i < IMX9_NPHYSENDPOINTS; i++)
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
 * Name: imx9_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically
 *   part of the USB interrupt handler.
 *
 ****************************************************************************/

static void imx9_dispatchrequest(struct imx9_usb_s *priv,
                                    const struct usb_ctrlreq_s *ctrl)
{
  int ret = -EIO;

  usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_DISPATCH), 0);
  if (priv->driver)
    {
      /* Invalidate buffer data cache */

      DEBUGASSERT(IS_CACHE_ALIGNED(priv->ep0.buf, EP0_MAXBUFLEN));
      up_invalidate_dcache((uintptr_t)priv->ep0.buf,
                           (uintptr_t)priv->ep0.buf + EP0_MAXBUFLEN);

      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl, priv->ep0.buf,
                        priv->ep0.buf_len);
    }

  if (ret < 0)
    {
      /* Stall on failure */

      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_DISPATCHSTALL), 0);
      priv->stalled = true;
    }
}

/****************************************************************************
 * Name: imx9_ep0configure
 *
 * Description:
 *   Reset Usb engine
 *
 ****************************************************************************/

static void imx9_ep0configure(struct imx9_usb_s *priv)
{
  /* Enable ep0 IN and ep0 OUT */

  priv->qh[IMX9_EP0_OUT].capability =
    (DQH_CAPABILITY_MAX_PACKET(CONFIG_USBDEV_EP0_MAXSIZE) |
     DQH_CAPABILITY_IOS | DQH_CAPABILITY_ZLT);

  priv->qh[IMX9_EP0_IN].capability =
    (DQH_CAPABILITY_MAX_PACKET(CONFIG_USBDEV_EP0_MAXSIZE) |
     DQH_CAPABILITY_IOS | DQH_CAPABILITY_ZLT);

  priv->qh[IMX9_EP0_OUT].currdesc = DTD_NEXTDESC_INVALID;
  priv->qh[IMX9_EP0_IN].currdesc = DTD_NEXTDESC_INVALID;

  up_clean_dcache((uintptr_t)priv->qh,
                  (uintptr_t)priv->qh + (sizeof(struct imx9_dqh_s)));

  /* Enable EP0 */

  imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL0_OFFSET, 0,
                 USBDEV_ENDPTCTRL0_RXE | USBDEV_ENDPTCTRL0_TXE);
}

/****************************************************************************
 * Name: imx9_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 ****************************************************************************/

static void imx9_usbreset(struct imx9_usb_s *priv)
{
  int epphy;

  /* Disable all endpoints. Control endpoint 0 is always enabled */

  imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL1_OFFSET,
                 USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE, 0);
  imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL2_OFFSET,
                 USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE, 0);
  imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL3_OFFSET,
                 USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE, 0);
  imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL4_OFFSET,
                 USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE, 0);
  imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL5_OFFSET,
                 USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE, 0);

  /* Clear all pending interrupts */

  imx9_putreg(priv, IMX9_USBDEV_ENDPTNAK_OFFSET,
              imx9_getreg(priv, IMX9_USBDEV_ENDPTNAK_OFFSET));

  imx9_putreg(priv, IMX9_USBDEV_ENDPTSETUPSTAT_OFFSET,
              imx9_getreg(priv, IMX9_USBDEV_ENDPTSETUPSTAT_OFFSET));

  imx9_putreg(priv, IMX9_USBDEV_ENDPTCOMPLETE_OFFSET,
              imx9_getreg(priv, IMX9_USBDEV_ENDPTCOMPLETE_OFFSET));

  /* Wait for all prime operations to have completed and then flush all
   * DTDs
   */

  while (imx9_getreg(priv, IMX9_USBDEV_ENDPTPRIME_OFFSET) != 0);

  imx9_putreg(priv, IMX9_USBDEV_ENDPTFLUSH_OFFSET, IMX9_ENDPTMASK_ALL);

  while (imx9_getreg(priv, IMX9_USBDEV_ENDPTFLUSH_OFFSET));

  /* Reset endpoints */

  for (epphy = 0; epphy < IMX9_NPHYSENDPOINTS; epphy++)
    {
      struct imx9_ep_s *privep = &priv->eplist[epphy];

      imx9_cancelrequests(privep, -ESHUTDOWN);

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

  imx9_modifyreg(priv, IMX9_USBDEV_USBCMD_OFFSET, USBDEV_USBCMD_ITC_MASK,
                 USBDEV_USBCMD_ITCIMME);

  /* Zero out the Endpoint queue heads */

  memset ((void *)priv->qh, 0, sizeof(priv->qh));
  memset ((void *)priv->td, 0, sizeof(priv->td));

  up_clean_dcache((uintptr_t)priv->qh,
                  (uintptr_t)priv->qh + sizeof(priv->qh));
  up_clean_dcache((uintptr_t)priv->td,
                  (uintptr_t)priv->td + sizeof(priv->td));

  /* Set USB address to 0 */

  imx9_set_address(priv, 0);

  /* Initialise the Enpoint List Address */

  imx9_putreg(priv, IMX9_USBDEV_ENDPOINTLIST_OFFSET,
              (uint32_t)(uintptr_t)priv->qh);

  /* EndPoint 0 initialization */

  imx9_ep0configure(priv);

  /* Enable Device interrupts */

  imx9_putreg(priv, IMX9_USBDEV_USBINTR_OFFSET,
              USB_FRAME_INT | USB_ERROR_INT | USBDEV_USBINTR_NAKE |
              USBDEV_USBINTR_SLE | USBDEV_USBINTR_URE | USBDEV_USBINTR_PCE |
              USBDEV_USBINTR_UE);
}

/****************************************************************************
 * Name: imx9_setstate
 *
 * Description:
 *   Sets the EP0 state and manages the NAK interrupts
 *
 ****************************************************************************/

static inline void imx9_ep0state(struct imx9_usb_s *priv,
                                  uint16_t state)
{
  priv->ep0.state = state;

  switch (state)
    {
    case EP0STATE_WAIT_NAK_IN:
      imx9_putreg(priv,  IMX9_USBDEV_ENDPTNAKEN_OFFSET,
                  IMX9_ENDPTMASK(IMX9_EP0_IN));
      break;

    case EP0STATE_WAIT_NAK_OUT:
      imx9_putreg(priv, IMX9_USBDEV_ENDPTNAKEN_OFFSET,
                  IMX9_ENDPTMASK(IMX9_EP0_OUT));
      break;

    default:
      imx9_putreg(priv, IMX9_USBDEV_ENDPTNAKEN_OFFSET, 0);
      break;
    }

  UP_DSB();
}

/****************************************************************************
 * Name: imx9_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event. This is logically part of the USB interrupt
 *   handler.  This event occurs when a setup packet is receive on EP0 OUT.
 *
 ****************************************************************************/

static inline void imx9_ep0setup(struct imx9_usb_s *priv)
{
  struct imx9_ep_s *privep;
  struct usb_ctrlreq_s *ctrl;
  uint16_t value;
  uint16_t index;
  uint16_t len;

  ctrl = &priv->ep0.ctrl;

  /* Terminate any pending requests - since all DTDs will have been retired
   * because of the setup packet.
   */

  imx9_cancelrequests(&priv->eplist[IMX9_EP0_OUT], -EPROTO);
  imx9_cancelrequests(&priv->eplist[IMX9_EP0_IN],  -EPROTO);

  /* Assume NOT stalled */

  priv->eplist[IMX9_EP0_OUT].stalled = false;
  priv->eplist[IMX9_EP0_IN].stalled = false;
  priv->stalled = false;

  /* Read EP0 setup data */

  imx9_readsetup(priv, IMX9_EP0_OUT, ctrl);

  /* And extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  index = GETUINT16(ctrl->index);
  len   = GETUINT16(ctrl->len);

  /* Limit the ep0 length to maximum */

  priv->ep0.buf_len = len <= EP0_MAXBUFLEN ? len : EP0_MAXBUFLEN;

  uinfo("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  /* Starting a control request - update state */

  if (ctrl->type & USB_REQ_DIR_IN)
    {
      imx9_ep0state(priv, EP0STATE_SETUP_IN);
    }
  else
    {
      imx9_ep0state(priv, EP0STATE_SETUP_OUT);

      if (len > 0)
        {
          imx9_ep0state(priv, EP0STATE_SHORTREAD);
          imx9_ep0xfer(priv, IMX9_EP0_OUT, priv->ep0.buf, len);
          return;
        }
    }

  /* Dispatch any non-standard requests */

  if ((ctrl->type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      imx9_dispatchrequest(priv, ctrl);
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

            usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_GETSTATUS), 0);
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
                          TRACE_INTDECODE(IMX9_TRACEINTID_EPGETSTATUS), 0);
                      privep = imx9_epfindbyaddr(priv, index);
                      if (!privep)
                        {
                          usbtrace(
                              TRACE_DEVERROR(IMX9_TRACEERR_BADEPGETSTATUS),
                              0);
                          priv->stalled = true;
                        }
                      else
                        {
                          if (privep->stalled)
                            {
                              priv->ep0.buf[0] = 1; /* Stalled */
                            }
                          else
                            {
                              priv->ep0.buf[0] = 0; /* Not stalled */
                            }

                          priv->ep0.buf[1] = 0;

                          imx9_ep0xfer(priv, IMX9_EP0_IN, priv->ep0.buf, 2);
                          imx9_ep0state(priv, EP0STATE_SHORTWRITE);
                        }
                    }
                    break;

                  case USB_REQ_RECIPIENT_DEVICE:
                    {
                      if (index == 0)
                        {
                          usbtrace(
                              TRACE_INTDECODE(IMX9_TRACEINTID_DEVGETSTATUS),
                              0);

                          /* Features:  Remote Wakeup=YES; selfpowered=? */

                          priv->ep0.buf[0] =
                            (priv->selfpowered << USB_FEATURE_SELFPOWERED) |
                            (1 << USB_FEATURE_REMOTEWAKEUP);
                          priv->ep0.buf[1] = 0;

                          imx9_ep0xfer(priv, IMX9_EP0_IN, priv->ep0.buf, 2);
                          imx9_ep0state(priv, EP0STATE_SHORTWRITE);
                        }
                      else
                        {
                          usbtrace(
                              TRACE_DEVERROR(IMX9_TRACEERR_BADDEVGETSTATUS),
                              0);
                          priv->stalled = true;
                        }
                    }
                    break;

                  case USB_REQ_RECIPIENT_INTERFACE:
                    {
                      usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_IFGETSTATUS),
                               0);
                      priv->ep0.buf[0] = 0;
                      priv->ep0.buf[1] = 0;

                      imx9_ep0xfer(priv, IMX9_EP0_IN, priv->ep0.buf, 2);
                      imx9_ep0state(priv, EP0STATE_SHORTWRITE);
                    }
                    break;

                  default:
                    {
                      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_BADGETSTATUS),
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

          usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_CLEARFEATURE), 0);
          if ((ctrl->type & USB_REQ_RECIPIENT_MASK) !=
              USB_REQ_RECIPIENT_ENDPOINT)
            {
              imx9_dispatchrequest(priv, ctrl);
            }
          else if (priv->paddrset != 0 &&
              value == USB_FEATURE_ENDPOINTHALT &&
              len == 0 && (privep = imx9_epfindbyaddr(priv, index)) != NULL)
            {
              imx9_epstall(&privep->ep, true);
              imx9_ep0state(priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_BADCLEARFEATURE), 0);
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

          usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_SETFEATURE), 0);
          if (((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE) && value == USB_FEATURE_TESTMODE)
            {
              uinfo("test mode: %d\n", index);
            }
          else if ((ctrl->type & USB_REQ_RECIPIENT_MASK) !=
              USB_REQ_RECIPIENT_ENDPOINT)
            {
              imx9_dispatchrequest(priv, ctrl);
            }
          else if (priv->paddrset != 0 &&
              value == USB_FEATURE_ENDPOINTHALT &&
              len == 0 && (privep = imx9_epfindbyaddr(priv, index)) != NULL)
            {
              imx9_epstall(&privep->ep, false);
              imx9_ep0state(priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_BADSETFEATURE), 0);
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

          usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_EP0SETUPSETADDRESS),
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
              imx9_ep0state(priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_BADSETADDRESS), 0);
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
          usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_GETSETDESC), 0);
          if ((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE)
            {
              imx9_dispatchrequest(priv, ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_BADGETSETDESC), 0);
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
          usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_GETCONFIG), 0);
          if (priv->paddrset &&
              ((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
                  USB_REQ_RECIPIENT_DEVICE) &&
                  value == 0 && index == 0 && len == 1)
            {
              imx9_dispatchrequest(priv, ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_BADGETCONFIG), 0);
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
          usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_SETCONFIG), 0);
          if (((ctrl->type & USB_REQ_RECIPIENT_MASK) ==
              USB_REQ_RECIPIENT_DEVICE) && index == 0 && len == 0)
            {
              imx9_dispatchrequest(priv, ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_BADSETCONFIG), 0);
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
          usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_GETSETIF), 0);
          imx9_dispatchrequest(priv, ctrl);
        }
        break;

      case USB_REQ_SYNCHFRAME:
        /* type:  device-to-host; recipient = endpoint
         * value: 0
         * index: endpoint;
         * len:   2; data = frame number
         */

        {
          usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_SYNCHFRAME), 0);
        }
        break;

      default:
        {
          usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_INVALIDCTRLREQ), 0);
          priv->stalled = true;
        }
        break;
      }
  }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_EP0SETUPSTALLED),
               priv->ep0.state);
      imx9_epstall(&priv->eplist[IMX9_EP0_IN].ep, false);
      imx9_epstall(&priv->eplist[IMX9_EP0_OUT].ep, false);
    }
}

/****************************************************************************
 * Name: imx9_ep0complete
 *
 * Description:
 *   Transfer complete handler for Endpoint 0
 *
 ****************************************************************************/

static void imx9_ep0complete(struct imx9_usb_s *priv, uint8_t epphy)
{
  struct imx9_ep_s *privep = &priv->eplist[epphy];

  usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_EP0COMPLETE),
           (uint16_t)priv->ep0.state);

  switch (priv->ep0.state)
    {
    case EP0STATE_DATA_IN:
      if (imx9_rqempty(privep))
        {
          return;
        }

      if (imx9_epcomplete(priv, epphy))
        {
          imx9_ep0state(priv, EP0STATE_WAIT_NAK_OUT);
        }
      break;

    case EP0STATE_DATA_OUT:
      if (imx9_rqempty(privep))
        {
          return;
        }

      if (imx9_epcomplete(priv, epphy))
        {
          imx9_ep0state(priv, EP0STATE_WAIT_NAK_IN);
        }
      break;

    case EP0STATE_SHORTREAD:
      imx9_dispatchrequest(priv, &priv->ep0.ctrl);
      imx9_ep0state(priv, EP0STATE_WAIT_NAK_IN);
      break;

    case EP0STATE_SHORTWRITE:
      imx9_ep0state(priv, EP0STATE_WAIT_NAK_OUT);
      break;

    case EP0STATE_WAIT_STATUS_IN:
      imx9_ep0state(priv, EP0STATE_IDLE);

      /* If we've received a SETADDRESS packet, then we set the address
       * now that the status phase has completed
       */

      if (!priv->paddrset && priv->paddr != 0)
        {
          usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_EP0INSETADDRESS),
                   (uint16_t)priv->paddr);
          imx9_set_address(priv, priv->paddr);
        }

      break;

    case EP0STATE_WAIT_STATUS_OUT:
      imx9_ep0state(priv, EP0STATE_IDLE);
      break;

    default:
#ifdef CONFIG_DEBUG_FEATURES
      DEBUGASSERT(priv->ep0.state != EP0STATE_DATA_IN &&
          priv->ep0.state != EP0STATE_DATA_OUT        &&
          priv->ep0.state != EP0STATE_SHORTWRITE      &&
          priv->ep0.state != EP0STATE_WAIT_STATUS_IN  &&
          priv->ep0.state != EP0STATE_WAIT_STATUS_OUT);
#endif
      priv->stalled = true;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_EP0SETUPSTALLED),
               priv->ep0.state);
      imx9_epstall(&priv->eplist[IMX9_EP0_IN].ep, false);
      imx9_epstall(&priv->eplist[IMX9_EP0_OUT].ep, false);
    }
}

/****************************************************************************
 * Name: imx9_ep0nak
 *
 * Description:
 *   Handle a NAK interrupt on EP0
 *
 ****************************************************************************/

static void imx9_ep0nak(struct imx9_usb_s *priv, uint8_t epphy)
{
  usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_EP0NAK),
           (uint16_t)priv->ep0.state);

  switch (priv->ep0.state)
    {
    case EP0STATE_WAIT_NAK_IN:
      imx9_ep0xfer(priv, IMX9_EP0_IN, NULL, 0);
      imx9_ep0state(priv, EP0STATE_WAIT_STATUS_IN);
      break;

    case EP0STATE_WAIT_NAK_OUT:
      imx9_ep0xfer(priv, IMX9_EP0_OUT, NULL, 0);
      imx9_ep0state(priv, EP0STATE_WAIT_STATUS_OUT);
      break;

    default:
#ifdef CONFIG_DEBUG_FEATURES
      DEBUGASSERT(priv->ep0.state != EP0STATE_WAIT_NAK_IN &&
                  priv->ep0.state != EP0STATE_WAIT_NAK_OUT);
#endif
      priv->stalled = true;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_EP0SETUPSTALLED),
               priv->ep0.state);
      imx9_epstall(&priv->eplist[IMX9_EP0_IN].ep, false);
      imx9_epstall(&priv->eplist[IMX9_EP0_OUT].ep, false);
    }
}

/****************************************************************************
 * Name: imx9_epcomplete
 *
 * Description:
 *   Transfer complete handler for Endpoints other than 0
 *   returns whether the request at the head has completed
 *
 ****************************************************************************/

bool imx9_epcomplete(struct imx9_usb_s *priv, uint8_t epphy)
{
  struct imx9_ep_s  *privep  = &priv->eplist[epphy];
  struct imx9_req_s *privreq = privep->head;
  struct imx9_dtd_s *dtd     = &priv->td[epphy];

  if (privreq == NULL)        /* This shouldn't really happen */
    {
      if (IMX9_EPPHYOUT(privep->epphy))
        {
          usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_EPINQEMPTY), 0);
        }
      else
        {
          usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_EPOUTQEMPTY), 0);
        }

      return true;
    }

  /* Make sure we have updated data after the DMA transfer. */

  DEBUGASSERT(IS_CACHE_ALIGNED(dtd, sizeof(struct imx9_dtd_s)));
  up_invalidate_dcache((uintptr_t)dtd,
                       (uintptr_t)dtd + sizeof(struct imx9_dtd_s));

  int xfrd = dtd->xfer_len - (dtd->config >> 16);

  privreq->req.xfrd += xfrd;

  bool complete = true;
  if (IMX9_EPPHYOUT(privep->epphy))
    {
      /* read(OUT) completes when request filled, or a short transfer is
       * received
       */

      usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_EPIN), complete);

      /* Invalidate rx buffer cache */

      DEBUGASSERT(IS_CACHE_ALIGNED(privreq->req.buf,
                                   DCACHE_ALIGN_UP(privreq->req.xfrd)));
      up_invalidate_dcache((uintptr_t)privreq->req.buf,
                           (uintptr_t)privreq->req.buf +
                           DCACHE_ALIGN_UP(privreq->req.xfrd));
    }
  else
    {
      /* write(IN) completes when request finished, unless we need to
       * terminate with a ZLP
       */

      bool need_zlp = (xfrd == privep->ep.maxpacket) &&
          ((privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0);

      complete = (privreq->req.xfrd >= privreq->req.len && !need_zlp);

      usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_EPOUT), complete);
    }

  /* If the transfer is complete, then dequeue and progress any further
   * queued requests
   */

  if (complete)
    {
      privreq = imx9_rqdequeue(privep);
    }

  if (!imx9_rqempty(privep))
    {
      imx9_progressep(privep);
    }

  /* Now it's safe to call the completion callback as it may well submit a
   * new request
   */

  if (complete)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
      imx9_reqcomplete(privep, privreq, OK);
    }

  return complete;
}

/****************************************************************************
 * Name: imx9_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 ****************************************************************************/

static int imx9_usbinterrupt(int irq, void *context, void *arg)
{
  struct imx9_usb_s *priv = (struct imx9_usb_s *)arg;
  uint32_t disr;
  uint32_t portsc1;
  uint32_t n;

  usbtrace(TRACE_INTENTRY(IMX9_TRACEINTID_USB), 0);

  /* Read the interrupts and then clear them */

  disr = imx9_getreg(priv, IMX9_USBDEV_USBSTS_OFFSET);
  imx9_putreg(priv, IMX9_USBDEV_USBSTS_OFFSET, disr);

  if (disr & USBDEV_USBSTS_URI)
    {
      usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_DEVRESET), 0);

      imx9_usbreset(priv);

      usbtrace(TRACE_INTEXIT(IMX9_TRACEINTID_USB), 0);
      return OK;
    }

  /* When the device controller enters a suspend state from an active state,
   * the SLI bit will be set to a one.
   */

  if (!priv->suspended && (disr & USBDEV_USBSTS_SLI) != 0)
    {
      usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_SUSPENDED), 0);

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
      usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_RESUMED), 0);

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
      portsc1 = imx9_getreg(priv, IMX9_USBDEV_PORTSC1_OFFSET);

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

#ifdef CONFIG_IMX9_USB_FRAME_INTERRUPT
  if (disr & USBDEV_USBSTS_SRI)
    {
      usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_FRAME), 0);

      uint32_t frindex = imx9_getreg(IMX9_USB_FRINDEX);
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

      uint32_t mask = imx9_getreg(priv, IMX9_USBDEV_ENDPTCOMPLETE_OFFSET);

      if (mask)
        {
          /* Clear any NAK interrupt and completion interrupts */

          imx9_putreg(priv, IMX9_USBDEV_ENDPTNAK_OFFSET, mask);
          imx9_putreg(priv, IMX9_USBDEV_ENDPTCOMPLETE_OFFSET, mask);

          if (mask & IMX9_ENDPTMASK(0))
            {
              imx9_ep0complete(priv, 0);
            }

          if (mask & IMX9_ENDPTMASK(1))
            {
              imx9_ep0complete(priv, 1);
            }

          for (n = 1; n < IMX9_NLOGENDPOINTS; n++)
            {
              if (mask & IMX9_ENDPTMASK((n << 1)))
                {
                  imx9_epcomplete(priv, (n << 1));
                }

              if (mask & IMX9_ENDPTMASK((n << 1)+1))
                {
                  imx9_epcomplete(priv, (n << 1)+1);
                }
            }
        }

      /* Handle setup interrupts */

      uint32_t setupstat = imx9_getreg(priv,
                                       IMX9_USBDEV_ENDPTSETUPSTAT_OFFSET);
      if (setupstat)
        {
          /* Clear the endpoint complete CTRL OUT and IN when a Setup is
           * received
           */

          imx9_putreg(priv, IMX9_USBDEV_ENDPTCOMPLETE_OFFSET,
                      IMX9_ENDPTMASK(IMX9_EP0_IN) |
                      IMX9_ENDPTMASK(IMX9_EP0_OUT));

          if (setupstat & IMX9_ENDPTMASK(IMX9_EP0_OUT))
            {
              usbtrace(TRACE_INTDECODE(IMX9_TRACEINTID_EP0SETUP),
                       setupstat);
              imx9_ep0setup(priv);
            }
        }
    }

  if (disr & USBDEV_USBSTS_NAKI)
    {
      uint32_t pending = imx9_getreg(priv, IMX9_USBDEV_ENDPTNAK_OFFSET) &
        imx9_getreg(priv, IMX9_USBDEV_ENDPTNAKEN_OFFSET);

      if (pending)
        {
          /* We shouldn't see NAK interrupts except on Endpoint 0 */

          if (pending & IMX9_ENDPTMASK(0))
            {
              imx9_ep0nak(priv, 0);
            }

          if (pending & IMX9_ENDPTMASK(1))
            {
              imx9_ep0nak(priv, 1);
            }
        }

      /* Clear the interrupts */

      imx9_putreg(priv, IMX9_USBDEV_ENDPTNAK_OFFSET, pending);
    }

  usbtrace(TRACE_INTEXIT(IMX9_TRACEINTID_USB), 0);
  return OK;
}

/****************************************************************************
 * Endpoint operations
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_epconfigure
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

static int imx9_epconfigure(struct usbdev_ep_s *ep,
                             const struct usb_epdesc_s *desc,
                             bool last)
{
  struct imx9_ep_s *privep = (struct imx9_ep_s *)ep;
  struct imx9_usb_s *priv = privep->dev;
  struct imx9_dqh_s *dqh = &priv->qh[privep->epphy];

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

  up_clean_dcache((uintptr_t)dqh,
                  (uintptr_t)dqh + sizeof(struct imx9_dqh_s));

  /* Setup Endpoint Control Register */

  if (IMX9_EPPHYIN(privep->epphy))
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

      imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL_OFFSET(privep->epphy >> 1),
                     0xffff0000, cfg);
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

      imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL_OFFSET(privep->epphy >> 1),
                     0xffff0000, cfg);
    }

  /* Reset endpoint status */

  privep->stalled = false;

  /* Enable the endpoint */

  if (IMX9_EPPHYIN(privep->epphy))
    {
      imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL_OFFSET(privep->epphy >> 1),
                     0, USBDEV_ENDPTCTRL_TXE);
    }
  else
    {
      imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL_OFFSET(privep->epphy >> 1),
                     0, USBDEV_ENDPTCTRL_RXE);
    }

  return OK;
}

/****************************************************************************
 * Name: imx9_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 ****************************************************************************/

static int imx9_epdisable(struct usbdev_ep_s *ep)
{
  struct imx9_ep_s *privep = (struct imx9_ep_s *)ep;
  struct imx9_usb_s *priv = privep->dev;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPDISABLE, privep->epphy);

  flags = spin_lock_irqsave(&privep->spinlock);

  /* Disable Endpoint */

  if (IMX9_EPPHYIN(privep->epphy))
    {
      imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL_OFFSET(privep->epphy >> 1),
                     USBDEV_ENDPTCTRL_TXE, 0);
    }
  else
    {
      imx9_modifyreg(priv, IMX9_USBDEV_ENDPTCTRL_OFFSET(privep->epphy >> 1),
                     USBDEV_ENDPTCTRL_RXE, 0);
    }

  privep->stalled = true;

  /* Cancel any ongoing activity */

  imx9_cancelrequests(privep, -ESHUTDOWN);

  spin_unlock_irqrestore(&privep->spinlock, flags);
  return OK;
}

/****************************************************************************
 * Name: imx9_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 ****************************************************************************/

static struct usbdev_req_s *imx9_epallocreq(struct usbdev_ep_s *ep)
{
  struct imx9_req_s *privreq;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  usbtrace(TRACE_EPALLOCREQ, ((struct imx9_ep_s *)ep)->epphy);

  privreq = kmm_malloc(sizeof(struct imx9_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct imx9_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: imx9_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 ****************************************************************************/

static void imx9_epfreereq(struct usbdev_ep_s *ep,
                            struct usbdev_req_s *req)
{
  struct imx9_req_s *privreq = (struct imx9_req_s *)req;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((struct imx9_ep_s *)ep)->epphy);
  kmm_free(privreq);
}

/****************************************************************************
 * Name: imx9_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *imx9_epallocbuffer(struct usbdev_ep_s *ep, uint16_t bytes)
{
  /* The USB peripheral DMA is very forgiving, as the dTD allows the buffer
   * to start at any address. Hence, no need for alignment.
   */

  struct imx9_ep_s *privep = (struct imx9_ep_s *)ep;
  UNUSED(privep);

  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);
#ifdef CONFIG_USBDEV_DMAMEMORY
  return usbdev_dma_alloc(bytes);
#else
  return cache_aligned_alloc(bytes);
#endif
}
#endif

/****************************************************************************
 * Name: imx9_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void imx9_epfreebuffer(struct usbdev_ep_s *ep, void *buf)
{
  struct imx9_ep_s *privep = (struct imx9_ep_s *)ep;
  UNUSED(privep);

  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);

#ifdef CONFIG_USBDEV_DMAMEMORY
  usbdev_dma_free(buf);
#else
  kmm_free(buf);
#endif
}
#endif

/****************************************************************************
 * Name: imx9_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 ****************************************************************************/

static int imx9_epsubmit(struct usbdev_ep_s *ep,
                          struct usbdev_req_s *req)
{
  struct imx9_req_s *privreq = (struct imx9_req_s *)req;
  struct imx9_ep_s *privep = (struct imx9_ep_s *)ep;
  struct imx9_usb_s *priv;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG_FEATURES
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_INVALIDPARMS), 0);
      uinfo("req=%p callback=%p buf=%p ep=%p\n", req,
            req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

  if (!priv->driver || priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_NOTCONFIGURED),
               priv->usbdev.speed);
      return -ESHUTDOWN;
    }

  /* Handle the request from the class driver */

  req->result = -EINPROGRESS;
  req->xfrd   = 0;

  /* Disable Interrupts */

  flags = spin_lock_irqsave(&privep->spinlock);

  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      ret = -EBUSY;
    }
  else
    {
      /* Add the new request to the request queue for the endpoint */

      if (IMX9_EPPHYIN(privep->epphy))
        {
          usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);
        }
      else
        {
          usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);
        }

      if (imx9_rqenqueue(privep, privreq))
        {
          imx9_progressep(privep);
        }
    }

  spin_unlock_irqrestore(&privep->spinlock, flags);
  return ret;
}

/****************************************************************************
 * Name: imx9_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 ****************************************************************************/

static int imx9_epcancel(struct usbdev_ep_s *ep,
                          struct usbdev_req_s *req)
{
  struct imx9_ep_s *privep = (struct imx9_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_FEATURES
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, privep->epphy);

  flags = spin_lock_irqsave(&privep->spinlock);

  /* FIXME: if the request is the first, then we need to flush the EP
   *         otherwise just remove it from the list
   *
   *  but ... all other implementations cancel all requests ...
   */

  imx9_cancelrequests(privep, -ESHUTDOWN);
  spin_unlock_irqrestore(&privep->spinlock, flags);
  return OK;
}

/****************************************************************************
 * Name: imx9_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 ****************************************************************************/

static int imx9_epstall(struct usbdev_ep_s *ep, bool resume)
{
  struct imx9_ep_s *privep = (struct imx9_ep_s *)ep;
  struct imx9_usb_s *priv = privep->dev;
  irqstate_t flags;

  /* STALL or RESUME the endpoint */

  flags = spin_lock_irqsave(&privep->spinlock);
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, privep->epphy);

  uint32_t offs    = IMX9_USBDEV_ENDPTCTRL_OFFSET(privep->epphy >> 1);
  uint32_t ctrl_xs = IMX9_EPPHYIN(privep->epphy) ?
      USBDEV_ENDPTCTRL_TXS : USBDEV_ENDPTCTRL_RXS;
  uint32_t ctrl_xr = IMX9_EPPHYIN(privep->epphy) ?
      USBDEV_ENDPTCTRL_TXR : USBDEV_ENDPTCTRL_RXR;

  if (resume)
    {
      privep->stalled = false;

      /* Clear stall and reset the data toggle */

      imx9_modifyreg(priv, offs, ctrl_xs | ctrl_xr, ctrl_xr);
    }
  else
    {
      privep->stalled = true;

      imx9_modifyreg(priv, offs, 0, ctrl_xs);
    }

  spin_unlock_irqrestore(&privep->spinlock, flags);
  return OK;
}

/****************************************************************************
 * Device operations
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_allocep
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

static struct usbdev_ep_s *imx9_allocep(struct usbdev_s *dev,
                                         uint8_t eplog,
                                         bool in, uint8_t eptype)
{
  struct imx9_usb_s *priv = (struct imx9_usb_s *)dev;
  uint32_t epset = IMX9_EPALLSET & ~IMX9_EPCTRLSET;
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

      if (eplog >= IMX9_NLOGENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_BADEPNO), (uint16_t)eplog);
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
      epset &= IMX9_EPINSET;
    }
  else
    {
      epset &= IMX9_EPOUTSET;
    }

  /* Get the subset matching the requested type */

  switch (eptype)
    {
    case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      epset &= IMX9_EPINTRSET;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      epset &= IMX9_EPBULKSET;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
      epset &= IMX9_EPISOCSET;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint -- not a valid choice */
    default:
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_BADEPTYPE), (uint16_t)eptype);
      return NULL;
    }

  /* Is the resulting endpoint supported by the IMX9? */

  if (epset)
    {
      /* Yes.. now see if any of the request endpoints are available */

      flags = spin_lock_irqsave(&priv->spinlock);
      epset &= priv->epavail;
      if (epset)
        {
          /* Select the lowest bit in the set of matching, available
           * endpoints
           */

          for (epndx = 2; epndx < IMX9_NPHYSENDPOINTS; epndx++)
            {
              uint32_t bit = 1 << epndx;
              if ((epset & bit) != 0)
                {
                  /* Mark endpoint no longer available */

                  priv->epavail &= ~bit;
                  spin_unlock_irqrestore(&priv->spinlock, flags);

                  /* And return the pointer to the standard endpoint
                   * structure
                   */

                  return &priv->eplist[epndx].ep;
                }
            }

          /* Shouldn't get here */
        }

      spin_unlock_irqrestore(&priv->spinlock, flags);
    }

  usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_NOEP), (uint16_t)eplog);
  return NULL;
}

/****************************************************************************
 * Name: imx9_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 ****************************************************************************/

static void imx9_freeep(struct usbdev_s *dev,
                         struct usbdev_ep_s *ep)
{
  struct imx9_usb_s *priv = (struct imx9_usb_s *)dev;
  struct imx9_ep_s *privep = (struct imx9_ep_s *)ep;
  irqstate_t flags_1;
  irqstate_t flags_2;

  usbtrace(TRACE_DEVFREEEP, (uint16_t)privep->epphy);

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      flags_1 = spin_lock_irqsave(&priv->spinlock);
      flags_2 = spin_lock_irqsave(&privep->spinlock);
      priv->epavail |= (1 << privep->epphy);
      spin_unlock_irqrestore(&privep->spinlock, flags_2);
      spin_unlock_irqrestore(&priv->spinlock, flags_1);
    }
}

/****************************************************************************
 * Name: imx9_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 ****************************************************************************/

static int imx9_getframe(struct usbdev_s *dev)
{
  struct imx9_usb_s *priv = (struct imx9_usb_s *)dev;

#ifdef CONFIG_IMX9_USB_FRAME_INTERRUPT
  /* Return last valid value of SOF read by the interrupt handler */

  usbtrace(TRACE_DEVGETFRAME, (uint16_t)priv->sof);
  return priv->sof;
#else
  uint32_t frindex = imx9_getreg(priv, IMX9_USBDEV_FRINDEX_OFFSET);
  uint16_t frame_num =
      (frindex & USBDEV_FRINDEX_LFN_MASK) >> USBDEV_FRINDEX_LFN_SHIFT;

  /* Return the last frame number detected by the hardware */

  usbtrace(TRACE_DEVGETFRAME, frame_num);

  return (int)(frame_num);
#endif
}

/****************************************************************************
 * Name: imx9_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 ****************************************************************************/

static int imx9_wakeup(struct usbdev_s *dev)
{
  irqstate_t flags;
  struct imx9_usb_s *priv = (struct imx9_usb_s *)dev;

  usbtrace(TRACE_DEVWAKEUP, 0);

  flags = spin_lock_irqsave(&priv->spinlock);
  imx9_modifyreg(priv, IMX9_USBDEV_PORTSC1_OFFSET, 0, USBDEV_PRTSC1_FPR);
  spin_unlock_irqrestore(&priv->spinlock, flags);
  return OK;
}

/****************************************************************************
 * Name: imx9_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature
 *
 ****************************************************************************/

static int imx9_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct imx9_usb_s *priv = (struct imx9_usb_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_FEATURES
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Name: imx9_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 ****************************************************************************/

static int imx9_pullup(struct usbdev_s *dev, bool enable)
{
  struct imx9_usb_s *priv = (struct imx9_usb_s *)dev;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  irqstate_t flags = spin_lock_irqsave(&priv->spinlock);
  if (enable)
    {
      imx9_modifyreg(priv, IMX9_USBDEV_USBCMD_OFFSET, 0, USBDEV_USBCMD_RS);

#ifdef CONFIG_IMX9_USB0DEV_NOVBUS
      /* Create a 'false' power event on the USB port so the MAC connects */

      imx9_modifyreg(priv, IMX9_USBOTG_OTGSC_OFFSET, USBOTG_OTGSC_VD, 0);
      imx9_modifyreg(priv, IMX9_USBOTG_OTGSC_OFFSET, 0, USBOTG_OTGSC_VC);
#endif
    }
  else
    {
      imx9_modifyreg(priv, IMX9_USBDEV_USBCMD_OFFSET, USBDEV_USBCMD_RS, 0);
    }

  spin_unlock_irqrestore(&priv->spinlock, flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_vbus_detect
 *
 * Description:
 *   Read the VBUS state from the USB OTG controller. This can be used
 *   to poll the VBUS state
 *
 * Input Parameters:
 *   id: IMX9_USBC1 or IMX9_USBC2
 *
 * Returned Value:
 *   true if VBUS is valid; false otherwise
 *
 ****************************************************************************/

bool imx9_vbus_detect(imx9_usb_id_t id)
{
  int i;
  struct imx9_usb_s *priv;
  uint32_t otgsc = 0;

  /* Find the correct device to which the driver is bound to */

  for (i = 0; i < n_usbdevs; i++)
    {
      if (id == g_usbdev[i].id)
        {
          break;
        }
    }

  if (i < n_usbdevs)
    {
      priv = &g_usbdev[i];
      otgsc = imx9_getreg(priv, IMX9_USBOTG_OTGSC_OFFSET);
    }

  return (otgsc & USBOTG_OTGSC_AVV) != 0;
}

/****************************************************************************
 * Name: arm64_usbinitialize
 *
 * Description:
 *   Initialize USB hardware.
 *
 * Assumptions:
 * - This function is called very early in the initialization sequence
 *
 ****************************************************************************/

void arm64_usbinitialize(void)
{
  /* For now, this driver supports just one usb device, either
   * USBC1 or USBC2. The configured one is in g_usbdev[0].
   */

  struct imx9_usb_s *priv = &g_usbdev[0];
  int i;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->spinlock);

  /* Initialize the device state structure */

  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[IMX9_EP0_IN].ep;
  priv->epavail    = IMX9_EPALLSET & ~IMX9_EPCTRLSET;

  /* Initialize the endpoint list */

  for (i = 0; i < IMX9_NPHYSENDPOINTS; i++)
    {
      uint32_t bit = 1 << i;

      /* Set endpoint operations, reference to driver structure and
       * the physical endpoint number (which is just the index to the
       * endpoint).
       */

      priv->eplist[i].ep.ops       = &g_epops;
      priv->eplist[i].dev          = priv;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      priv->eplist[i].epphy        = i;
      if (IMX9_EPPHYIN(i))
        {
          priv->eplist[i].ep.eplog = IMX9_EPPHYIN2LOG(i);
        }
      else
        {
          priv->eplist[i].ep.eplog = IMX9_EPPHYOUT2LOG(i);
        }

      /* The maximum packet size may depend on the type of endpoint */

      if ((IMX9_EPCTRLSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = IMX9_EP0MAXPACKET;
        }
      else if ((IMX9_EPINTRSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = IMX9_INTRMAXPACKET;
        }
      else if ((IMX9_EPBULKSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = IMX9_BULKMAXPACKET;
        }
      else /* if ((IMX9_EPISOCSET & bit) != 0) */
        {
          priv->eplist[i].ep.maxpacket = IMX9_ISOCMAXPACKET;
        }
    }

  /* Clock gate on */

  imx9_ccm_gate_on(CCM_LPCG_USB_CONTROLLER, true);

  /* Disable USB interrupts */

  imx9_putreg(priv, IMX9_USBDEV_USBINTR_OFFSET, 0);

  /* Soft reset PHY and enable clock - not needed for on-chip USB2 phy */

  /* Disconnect device */

  imx9_pullup(&priv->usbdev, false);

  /* Reset the controller */

  imx9_modifyreg(priv, IMX9_USBDEV_USBCMD_OFFSET, 0, USBDEV_USBCMD_RST);
  while (imx9_getreg(priv, IMX9_USBDEV_USBCMD_OFFSET) & USBDEV_USBCMD_RST);

  /* Power up the PHY - not needed for on-chip USB2 phy */

  /* Program the controller to be the USB device controller */

  imx9_putreg(priv, IMX9_USBDEV_USBMODE_OFFSET,
              USBDEV_USBMODE_SDIS | USBDEV_USBMODE_SLOM |
              USBDEV_USBMODE_CM_DEVICE);

  /* Attach USB controller interrupt handler */

  irq_attach(IMX9_IRQ_USB1 + priv->id, imx9_usbinterrupt, priv);
  up_enable_irq(IMX9_IRQ_USB1 + priv->id);

  spin_unlock_irqrestore(&priv->spinlock, flags);

  /* Reset/Re-initialize the USB hardware */

  imx9_usbreset(priv);
}

/****************************************************************************
 * Name: arm_usbuninitialize
 ****************************************************************************/

void arm64_usbuninitialize(void)
{
  struct imx9_usb_s *priv = &g_usbdev[0];
  irqstate_t flags;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  flags = spin_lock_irqsave(&priv->spinlock);

  /* Disconnect device */

  imx9_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable and detach IRQs */

  up_disable_irq(IMX9_IRQ_USB1 + priv->id);
  irq_detach(IMX9_IRQ_USB1 + priv->id);

  /* Reset the controller */

  imx9_modifyreg(priv, IMX9_USBDEV_USBCMD_OFFSET, 0, USBDEV_USBCMD_RST);
  while (imx9_getreg(priv, IMX9_USBDEV_USBCMD_OFFSET) & USBDEV_USBCMD_RST);

  /* Turn off USB power and clocking */

  /* Power down the PHY */

  /* Clock gate off - NOTE: this turns off the clock from both controllers.
   * Add reference counting if expanding this to support several ones.
   */

  imx9_ccm_gate_on(CCM_LPCG_USB_CONTROLLER, false);

  spin_unlock_irqrestore(&priv->spinlock, flags);
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
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev[0].driver)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  g_usbdev[0].driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev[0].usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(IMX9_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_usbdev[0].driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts */

      up_enable_irq(IMX9_IRQ_USB1 + g_usbdev[0].id);
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
  int i;

  usbtrace(TRACE_DEVUNREGISTER, 0);

  /* Find the correct device to which the driver is bound to */

  for (i = 0; i < n_usbdevs; i++)
    {
      if (driver == g_usbdev[i].driver)
        {
          break;
        }
    }

  if (i == n_usbdevs)
    {
      return -EINVAL;
    }

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_usbdev[i].usbdev);

  /* Disable USB controller interrupts */

  up_disable_irq(IMX9_IRQ_USB1 + g_usbdev[i].id);

  /* Unhook the driver */

  g_usbdev[i].driver = NULL;

  return OK;
}

