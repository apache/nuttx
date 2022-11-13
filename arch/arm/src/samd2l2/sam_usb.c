/****************************************************************************
 * arch/arm/src/samd2l2/sam_usb.c
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
 * USB core features:
 *
 * - Compatible with the USB 2.1 specification
 * - USB Embedded Host and Device mode
 * - Supports full (12Mbit/s) and low (1.5Mbit/s) speed communication
 * - Supports Link Power Management (LPM-L1) protocol
 * - On-chip transceivers with built-in pull-ups and pull-downs
 * - On-Chip USB serial resistors
 * - 1kHz SOF clock available on external pin
 *
 *   Device mode
 *   - Supports 8 IN endpoints and 8 OUT endpoints
 *   – No endpoint size limitations
 *   – Built-in DMA with multi-packet and dual bank for all endpoints
 *   – Supports feedback endpoint
 *   – Supports crystal less clock
 *
 *   Host mode
 *   - Supports 8 physical pipes
 *   – No pipe size limitations
 *   – Supports multiplexed virtual pipe on one physical pipe to allow an
 *     unlimited USB tree
 *   – Built-in DMA with multi-packet support and dual bank for all pipes
 *   – Supports feedback endpoint
 *   – Supports the USB 2.0 Phase-locked SOFs feature
 *
 ****************************************************************************/

/****************************************************************************
 * WIP NOTES:
 *
 * DS 38.1 (859)
 * To maximize throughput, an endpoint can be configured for ping-pong
 * operation.
 * When this is done the input and output endpoint with the same address are
 * used in the same direction. The CPU or DMA Controller can then read/write
 * one data buffer while the USB module writes/reads from the other buffer.
 * This gives double buffered communication.
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
#include <nuttx/irq.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "sam_gclk.h"
#include "chip.h"
#include "sam_port.h"
#include "sam_pinmap.h"
#include "sam_usb.h"
#include "sam_fuses.h"
#include "sam_periphclks.h"

#if defined(CONFIG_USBHOST) && defined(CONFIG_SAMD2L2_USB)
#  error USBHOST mode not yet implemented!
#endif

#if defined(CONFIG_USBDEV) && defined(CONFIG_SAMD2L2_USB)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG_USB
#  undef CONFIG_SAMD2L2_USB_REGDEBUG
#endif

/* Driver Definitions *******************************************************/

#define EP0                 (0)
#define SAM_EPSET_ALL       (0xff)    /* All endpoints */
#define SAM_EPSET_NOTEP0    (0xfe)    /* All endpoints except EP0 */
#define SAM_EP_BIT(ep)      (1 << (ep))
#define SAM_EP0_MAXPACKET   (CONFIG_USBDEV_EP0_MAXSIZE) /* EP0 Max. packet size */
#define SAM_MAX_MULTIPACKET_SIZE  (0x3fff)

/* Request queue operations *************************************************/

#define sam_rqempty(q)      ((q)->head == NULL)
#define sam_rqpeek(q)       ((q)->head)

/* USB trace ****************************************************************/

/* Trace error codes */

#define SAM_TRACEERR_ALLOCFAIL            0x0001
#define SAM_TRACEERR_BADCLEARFEATURE      0x0002
#define SAM_TRACEERR_BADDEVGETSTATUS      0x0003
#define SAM_TRACEERR_BADEPGETSTATUS       0x0004
#define SAM_TRACEERR_BADEOBSTATE          0x0005
#define SAM_TRACEERR_BADEPNO              0x0006
#define SAM_TRACEERR_BADEPTYPE            0x0007
#define SAM_TRACEERR_BADGETCONFIG         0x0008
#define SAM_TRACEERR_BADGETSETDESC        0x0009
#define SAM_TRACEERR_BADGETSTATUS         0x000a
#define SAM_TRACEERR_BADSETADDRESS        0x000b
#define SAM_TRACEERR_BADSETCONFIG         0x000c
#define SAM_TRACEERR_BADSETFEATURE        0x000d
#define SAM_TRACEERR_BINDFAILED           0x000e
#define SAM_TRACEERR_DISPATCHSTALL        0x000f
#define SAM_TRACEERR_DRIVER               0x0010
#define SAM_TRACEERR_DRIVERREGISTERED     0x0011
#define SAM_TRACEERR_EP0SETUPOUTSIZE      0x0012
#define SAM_TRACEERR_EP0SETUPSTALLED      0x0013
#define SAM_TRACEERR_EPOUTNULLPACKET      0x0014
#define SAM_TRACEERR_EPRESERVE            0x0015
#define SAM_TRACEERR_INVALIDCTRLREQ       0x0016
#define SAM_TRACEERR_INVALIDPARMS         0x0017
#define SAM_TRACEERR_IRQREGISTRATION      0x0018
#define SAM_TRACEERR_NOTCONFIGURED        0x0019
#define SAM_TRACEERR_REQABORTED           0x001a
#define SAM_TRACEERR_RXDATABKERR          0x001b
#define SAM_TRACEERR_TXCOMPERR            0x001c
#define SAM_TRACEERR_UNSUPPEPTYPE         0x001d

/* Trace interrupt codes */

#define SAM_TRACEINTID_INTERRUPT          0x0001
#define SAM_TRACEINTID_PENDING            0x0002
#define SAM_TRACEINTID_PENDING_EP         0x0003
#define SAM_TRACEINTID_SUSPEND            0x0004
#define SAM_TRACEINTID_SOF                0x0005
#define SAM_TRACEINTID_EORST              0x0006
#define SAM_TRACEINTID_WAKEUP             0x0007
#define SAM_TRACEINTID_EORSM              0x0008
#define SAM_TRACEINTID_UPRSM              0x0009
#define SAM_TRACEINTID_RAMACER            0x000a
#define SAM_TRACEINTID_LPMNYET            0x000b
#define SAM_TRACEINTID_LPMSUSP            0x000c
#define SAM_TRACEINTID_EPNO               0x000d
#define SAM_TRACEINTID_EPINTFLAGS         0x000e
#define SAM_TRACEINTID_EPTRCPT0           0x000f
#define SAM_TRACEINTID_EPTRCPT1           0x0010
#define SAM_TRACEINTID_EPTRFAIL0          0x0011
#define SAM_TRACEINTID_EPTRFAIL1          0x0012
#define SAM_TRACEINTID_EPRXSTP            0x0013
#define SAM_TRACEINTID_EPSTALL0           0x0014
#define SAM_TRACEINTID_EPSTALL1           0x0015
#define SAM_TRACEINTID_EPINQEMPTY         0x0016
#define SAM_TRACEINTID_EPOUTQEMPTY        0x0017
#define SAM_TRACEINTID_EP0SETUPOUT        0x0018
#define SAM_TRACEINTID_EP0SETUPIN         0x0019
#define SAM_TRACEINTID_EP0SETUPSETADDRESS 0x001a
#define SAM_TRACEINTID_NOSTDREQ           0x001b
#define SAM_TRACEINTID_GETSTATUS          0x001c
#define SAM_TRACEINTID_DEVGETSTATUS       0x001d
#define SAM_TRACEINTID_IFGETSTATUS        0x001e
#define SAM_TRACEINTID_CLEARFEATURE       0x001f
#define SAM_TRACEINTID_SETFEATURE         0x0020
#define SAM_TRACEINTID_GETSETDESC         0x0021
#define SAM_TRACEINTID_GETCONFIG          0x0022
#define SAM_TRACEINTID_SETCONFIG          0x0023
#define SAM_TRACEINTID_GETSETIF           0x0024
#define SAM_TRACEINTID_SYNCHFRAME         0x0025
#define SAM_TRACEINTID_DISPATCH           0x0026
#define SAM_TRACEINTID_ADDRESSED          0x0027
#define SAM_TRACEINTID_EPCONF             0x0028
#define SAM_TRACEINTID_EPINTEN            0x0029
#define SAM_TRACEINTID_EP0WRSTATUS        0x002a
#define SAM_TRACEINTID_EPTRCPT0_LEN       0x002b

/* Ever-present MIN and MAX macros */

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/* Byte ordering in host-based values */

#ifdef CONFIG_ENDIAN_BIG
#  define LSB 1
#  define MSB 0
#else
#  define LSB 0
#  define MSB 1
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* State of an endpoint */

enum sam_epstate_e
{
  /* --- All Endpoints --- */

  USB_EPSTATE_DISABLED = 0, /* Endpoint is disabled */
  USB_EPSTATE_STALLED,      /* Endpoint is stalled */
  USB_EPSTATE_IDLE,         /* Endpoint is idle (i.e. ready for
                             * transmission) */
  USB_EPSTATE_SENDING,      /* Endpoint is sending data */
  USB_EPSTATE_RXSTOPPED,    /* OUT endpoint is stopped waiting for a read
                             * request */

  /* --- Endpoint 0 Only --- */

  USB_EPSTATE_EP0DATAOUT,   /* Endpoint 0 is receiving SETUP OUT data */
  USB_EPSTATE_EP0STATUSIN,  /* Endpoint 0 is sending SETUP status */
  USB_EPSTATE_EP0ADDRESS    /* Address change is pending completion of
                             * status */
};

/* The overall state of the device */

enum sam_devstate_e
{
  USB_DEVSTATE_SUSPENDED = 0, /* The device is currently suspended */
  USB_DEVSTATE_POWERED,       /* Host is providing +5V through the USB cable */
  USB_DEVSTATE_DEFAULT,       /* Device has been reset */
  USB_DEVSTATE_ADDRESSED,     /* The device has been given an address on the bus */
  USB_DEVSTATE_CONFIGURED     /* A valid configuration has been selected. */
};

/* The result of EP0 SETUP processing */

enum sam_ep0setup_e
{
  USB_EP0SETUP_SUCCESS = 0,   /* The SETUP was handle without incident */
  USB_EP0SETUP_DISPATCHED,    /* The SETUP was forwarded to the class driver */
  USB_EP0SETUP_ADDRESS,       /* A new device address is pending */
  USB_EP0SETUP_STALL          /* An error occurred */
};

union wb_u
{
  uint16_t w;
  uint8_t  b[2];
};

/* A container for a request so that the request may be retained in a list */

struct sam_req_s
{
  struct usbdev_req_s  req;          /* Standard USB request */
  struct sam_req_s    *flink;        /* Supports a singly linked list */
  uint16_t             inflight;     /* Number of TX bytes tansmitting or
                                      * number of RX bytes we are waiting */
};

/* The head of a queue of requests */

struct sam_rqhead_s
{
  struct sam_req_s    *head;         /* Requests are added to the head of the list */
  struct sam_req_s    *tail;         /* Requests are removed from the tail of the list */
};

/* This is the internal representation of an endpoint */

struct sam_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct sam_ep_s.
   */

  struct usbdev_ep_s   ep;           /* Standard endpoint structure */

  /* SAMD2L2-specific fields */

  struct sam_usbdev_s *dev;          /* Reference to private driver data */
  struct sam_rqhead_s  reqq;         /* Read/write request queue */
  struct sam_rqhead_s  pendq;        /* Write requests pending stall sent */
  struct usbdev_epdesc_s *descb[2];  /* Pointers to this endpoints descriptors */
  volatile uint8_t     epstate;      /* State of the endpoint (see enum sam_epstate_e) */
  uint8_t              stalled:1;    /* true: Endpoint is stalled */
  uint8_t              pending:1;    /* true: IN Endpoint stall is pending */
  uint8_t              halted:1;     /* true: Endpoint feature halted */
  uint8_t              zlpsent:1;    /* Zero length packet has been sent */
  uint8_t              txbusy:1;     /* Write request queue is busy (recursion avoidance kludge) */
  uint8_t              rxactive:1;   /* read request is active (for top of queue) */
};

struct sam_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct sam_usbdev_s.
   */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* USB-specific fields */

  struct usb_ctrlreq_s ctrl;          /* Last EP0 request */
  uint8_t              devstate;      /* State of the device (see enum
                                       * sam_devstate_e) */
  uint8_t              prevstate;     /* Previous state of the device
                                       * before SUSPEND */
  uint8_t              devaddr;       /* Assigned device address */
  uint8_t              selfpowered:1; /* 1: Device is self powered */
  uint16_t             epavail;       /* Bitset of available endpoints */

  /* The endpoint list */

  struct sam_ep_s eplist[SAM_USB_NENDPOINTS];

  /* Endpoint descriptors 2 banks for each endpoint */

  struct usbdev_epdesc_s ep_descriptors[SAM_USB_NENDPOINTS * 2];

  /* EP0 data buffer.  For data that is included in an EP0 SETUP OUT
   * transaction.  In this case, no request is in place from the class
   * driver and the incoming data is caught in this buffer.  The size
   * of valid data in the buffer is given by ctrlreg.len[].  For the
   * case of EP0 SETUP IN transaction, the normal request mechanism is
   * used and the class driver provides the buffering.
   */

  uint8_t ep0out[SAM_EP0_MAXPACKET];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_SAMD2L2_USB_REGDEBUG
static void   sam_printreg(uintptr_t regaddr, uint32_t regval, bool iswrite);
static void   sam_checkreg(uintptr_t regaddr, uint32_t regval, bool iswrite);
static void   sam_putreg32(uint32_t regval, uintptr_t regaddr);
static uint32_t sam_getreg16(uintptr_t regaddr);
static void   sam_putreg16(uint16_t regval, uintptr_t regaddr);
static uint32_t sam_getreg8(uintptr_t regaddr);
static void   sam_putreg8(uint8_t regval, uintptr_t regaddr);
static void   sam_dumpep(struct sam_usbdev_s *priv, uint8_t epno);
#else
static inline void sam_putreg32(uint32_t regval, uintptr_t regaddr);
static inline uint32_t sam_getreg16(uintptr_t regaddr);
static inline void sam_putreg16(uint16_t regval, uintptr_t regaddr);
static inline uint32_t sam_getreg8(uintptr_t regaddr);
static inline void sam_putreg8(uint8_t regval, uintptr_t regaddr);
# define sam_dumpep(priv,epno)
#endif

/* Suspend/Resume Helpers ***************************************************/

#if 0 /* Not used */
static void   sam_suspend(struct sam_usbdev_s *priv);
#endif
static void   sam_resume(struct sam_usbdev_s *priv);

/* Request Helpers **********************************************************/

static struct sam_req_s *
              sam_req_dequeue(struct sam_rqhead_s *queue);
static void   sam_req_enqueue(struct sam_rqhead_s *queue,
                struct sam_req_s *req);
static void   sam_req_complete(struct sam_ep_s *privep, int16_t result);
static void   sam_req_wrsetup(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, struct sam_req_s *privreq);
static int    sam_req_write(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep);
static int    sam_req_read(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, uint16_t recvsize);
static void   sam_req_cancel(struct sam_ep_s *privep, int16_t status);

/* Interrupt level processing ***********************************************/

static void   sam_ep0_ctrlread(struct sam_usbdev_s *priv);
static void   sam_ep0_wrstatus(struct sam_usbdev_s *priv,
                const uint8_t *buffer, size_t buflen);
static void   sam_ep0_dispatch(struct sam_usbdev_s *priv);
static void   sam_setdevaddr(struct sam_usbdev_s *priv, uint8_t value);
static void   sam_ep0_setup(struct sam_usbdev_s *priv);
static void   sam_ep_interrupt(struct sam_usbdev_s *priv, int epno);
static int    sam_usb_interrupt(int irq, void *context, void *arg);

/* Endpoint helpers *********************************************************/

static void   sam_ep_reset(struct sam_usbdev_s *priv, uint8_t epno);
static void   sam_epset_reset(struct sam_usbdev_s *priv, uint16_t epset);
static int    sam_ep_stall(struct sam_ep_s *privep);
static int    sam_ep_resume(struct sam_ep_s *privep);
static inline struct sam_ep_s *
              sam_ep_reserve(struct sam_usbdev_s *priv, uint8_t epset);
static inline void
              sam_ep_unreserve(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep);
static int    sam_ep_configure_internal(struct sam_ep_s *privep,
                const struct usb_epdesc_s *desc);

/* Endpoint operations ******************************************************/

static int    sam_ep_configure(struct usbdev_ep_s *ep,
                const struct usb_epdesc_s *desc, bool last);
static int    sam_ep_disable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *
              sam_ep_allocreq(struct usbdev_ep_s *ep);
static void   sam_ep_freereq(struct usbdev_ep_s *ep,
                struct usbdev_req_s *);
static int    sam_ep_submit(struct usbdev_ep_s *ep,
                struct usbdev_req_s *req);
static int    sam_ep_cancel(struct usbdev_ep_s *ep,
                struct usbdev_req_s *req);
static int    sam_ep_stallresume(struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations *****************************************/

static struct usbdev_ep_s *
              sam_allocep(struct usbdev_s *dev, uint8_t epno, bool in,
                uint8_t eptype);
static void   sam_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int    sam_getframe(struct usbdev_s *dev);
static int    sam_wakeup(struct usbdev_s *dev);
static int    sam_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int    sam_pullup(struct usbdev_s *dev,  bool enable);

/* Initialization/Reset *****************************************************/

static void   sam_reset(struct sam_usbdev_s *priv);
static void   sam_enableclks(void);
static void   sam_disableclks(void);
static void   sam_hw_setup(struct sam_usbdev_s *priv);
static void   sam_sw_setup(struct sam_usbdev_s *priv);
static void   sam_hw_shutdown(struct sam_usbdev_s *priv);
static void   sam_sw_shutdown(struct sam_usbdev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct sam_usbdev_s g_usbd;

static const struct usbdev_epops_s g_epops =
{
  .configure     = sam_ep_configure,
  .disable       = sam_ep_disable,
  .allocreq      = sam_ep_allocreq,
  .freereq       = sam_ep_freereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer   = sam_ep_allocbuffer,
  .freebuffer    = sam_ep_freebuffer,
#endif
  .submit        = sam_ep_submit,
  .cancel        = sam_ep_cancel,
  .stall         = sam_ep_stallresume,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep       = sam_allocep,
  .freeep        = sam_freeep,
  .getframe      = sam_getframe,
  .wakeup        = sam_wakeup,
  .selfpowered   = sam_selfpowered,
  .pullup        = sam_pullup,
};

/* This describes endpoint 0 */

static const struct usb_epdesc_s g_ep0desc =
{
  .len           = USB_SIZEOF_EPDESC,
  .type          = USB_DESC_TYPE_ENDPOINT,
  .addr          = EP0,
  .attr          = USB_EP_ATTR_XFER_CONTROL,
  .mxpacketsize  =
  {
    64, 0
  },

  .interval      = 0
};

/* Device error strings that may be enabled for more desciptive USB trace
 * output.
 */

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_deverror[] =
{
  TRACE_STR(SAM_TRACEERR_ALLOCFAIL),
  TRACE_STR(SAM_TRACEERR_BADCLEARFEATURE),
  TRACE_STR(SAM_TRACEERR_BADDEVGETSTATUS),
  TRACE_STR(SAM_TRACEERR_BADEPGETSTATUS),
  TRACE_STR(SAM_TRACEERR_BADEOBSTATE),
  TRACE_STR(SAM_TRACEERR_BADEPNO),
  TRACE_STR(SAM_TRACEERR_BADEPTYPE),
  TRACE_STR(SAM_TRACEERR_BADGETCONFIG),
  TRACE_STR(SAM_TRACEERR_BADGETSETDESC),
  TRACE_STR(SAM_TRACEERR_BADGETSTATUS),
  TRACE_STR(SAM_TRACEERR_BADSETADDRESS),
  TRACE_STR(SAM_TRACEERR_BADSETCONFIG),
  TRACE_STR(SAM_TRACEERR_BADSETFEATURE),
  TRACE_STR(SAM_TRACEERR_BINDFAILED),
  TRACE_STR(SAM_TRACEERR_DISPATCHSTALL),
  TRACE_STR(SAM_TRACEERR_DRIVER),
  TRACE_STR(SAM_TRACEERR_DRIVERREGISTERED),
  TRACE_STR(SAM_TRACEERR_EP0SETUPOUTSIZE),
  TRACE_STR(SAM_TRACEERR_EP0SETUPSTALLED),
  TRACE_STR(SAM_TRACEERR_EPOUTNULLPACKET),
  TRACE_STR(SAM_TRACEERR_EPRESERVE),
  TRACE_STR(SAM_TRACEERR_INVALIDCTRLREQ),
  TRACE_STR(SAM_TRACEERR_INVALIDPARMS),
  TRACE_STR(SAM_TRACEERR_IRQREGISTRATION),
  TRACE_STR(SAM_TRACEERR_NOTCONFIGURED),
  TRACE_STR(SAM_TRACEERR_REQABORTED),
  TRACE_STR(SAM_TRACEERR_RXDATABKERR),
  TRACE_STR(SAM_TRACEERR_TXCOMPERR),
  TRACE_STR(SAM_TRACEERR_UNSUPPEPTYPE),
  TRACE_STR_END
};
#endif

/* Interrupt event strings that may be enabled for more desciptive USB trace
 * output.
 */

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(SAM_TRACEINTID_INTERRUPT),
  TRACE_STR(SAM_TRACEINTID_PENDING),
  TRACE_STR(SAM_TRACEINTID_PENDING_EP),
  TRACE_STR(SAM_TRACEINTID_SUSPEND),
  TRACE_STR(SAM_TRACEINTID_SOF),
  TRACE_STR(SAM_TRACEINTID_EORST),
  TRACE_STR(SAM_TRACEINTID_WAKEUP),
  TRACE_STR(SAM_TRACEINTID_EORSM),
  TRACE_STR(SAM_TRACEINTID_UPRSM),
  TRACE_STR(SAM_TRACEINTID_RAMACER),
  TRACE_STR(SAM_TRACEINTID_LPMNYET),
  TRACE_STR(SAM_TRACEINTID_LPMSUSP),

  TRACE_STR(SAM_TRACEINTID_EPNO),
  TRACE_STR(SAM_TRACEINTID_EPINTFLAGS),
  TRACE_STR(SAM_TRACEINTID_EPTRCPT0),
  TRACE_STR(SAM_TRACEINTID_EPTRCPT1),
  TRACE_STR(SAM_TRACEINTID_EPTRFAIL0),
  TRACE_STR(SAM_TRACEINTID_EPTRFAIL1),
  TRACE_STR(SAM_TRACEINTID_EPRXSTP),
  TRACE_STR(SAM_TRACEINTID_EPSTALL0),
  TRACE_STR(SAM_TRACEINTID_EPSTALL1),

  TRACE_STR(SAM_TRACEINTID_EPINQEMPTY),
  TRACE_STR(SAM_TRACEINTID_EPOUTQEMPTY),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPOUT),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPIN),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPSETADDRESS),
  TRACE_STR(SAM_TRACEINTID_NOSTDREQ),
  TRACE_STR(SAM_TRACEINTID_GETSTATUS),
  TRACE_STR(SAM_TRACEINTID_DEVGETSTATUS),
  TRACE_STR(SAM_TRACEINTID_IFGETSTATUS),
  TRACE_STR(SAM_TRACEINTID_CLEARFEATURE),
  TRACE_STR(SAM_TRACEINTID_SETFEATURE),
  TRACE_STR(SAM_TRACEINTID_GETSETDESC),
  TRACE_STR(SAM_TRACEINTID_GETCONFIG),
  TRACE_STR(SAM_TRACEINTID_SETCONFIG),
  TRACE_STR(SAM_TRACEINTID_GETSETIF),
  TRACE_STR(SAM_TRACEINTID_SYNCHFRAME),

  TRACE_STR(SAM_TRACEINTID_DISPATCH),
  TRACE_STR(SAM_TRACEINTID_ADDRESSED),
  TRACE_STR(SAM_TRACEINTID_EPCONF),
  TRACE_STR(SAM_TRACEINTID_EPINTEN),
  TRACE_STR(SAM_TRACEINTID_EP0WRSTATUS),
  TRACE_STR(SAM_TRACEINTID_EPTRCPT0_LEN),

  TRACE_STR_END
};
#endif

/****************************************************************************
 * Private Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_printreg
 *
 * Description:
 *   Print the SAMD2L2 USB register access
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD2L2_USB_REGDEBUG
static void sam_printreg(uintptr_t regaddr, uint32_t regval, bool iswrite)
{
  uinfo("%p%s%08x\n", regaddr, iswrite ? "<-" : "->", regval);
}
#endif

/****************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if it is time to output debug information for accesses to a
 *   SAMD2L2 USB registers
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD2L2_USB_REGDEBUG
static void sam_checkreg(uintptr_t regaddr, uint32_t regval, bool iswrite)
{
  static uintptr_t prevaddr  = 0;
  static uint32_t  preval    = 0;
  static uint32_t  count     = 0;
  static bool      prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register
   * last time?  Are we polling the register?  If so, suppress the output.
   */

  if (regaddr == prevaddr && regval == preval && prevwrite == iswrite)
    {
      /* Yes.. Just increment the count */

      count++;
    }
  else
    {
      /* No this is a new address or value or operation. Were there any
       * duplicate accesses before this one?
       */

      if (count > 0)
        {
          /* Yes.. Just one? */

          if (count == 1)
            {
              /* Yes.. Just one */

              sam_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              uinfo("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = regaddr;
      preval    = regval;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new register access */

      sam_printreg(regaddr, regval, iswrite);
    }
}
#endif

/****************************************************************************
 * Name: sam_putreg32
 *
 * Description:
 *   Set the contents of an 32-bit SAMD2L2 USB register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD2L2_USB_REGDEBUG
static void sam_putreg32(uint32_t regval, uintptr_t regaddr)
{
  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, true);

  /* Write the value */

  putreg32(regval, regaddr);
}
#else
static inline void sam_putreg32(uint32_t regval, uintptr_t regaddr)
{
  putreg32(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: sam_getreg16
 *
 * Description:
 *   Get the contents of an 16-bit SAMD2L2 USB register
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD2L2_USB_REGDEBUG
static uint32_t sam_getreg16(uintptr_t regaddr)
{
  /* Read the value from the register */

  uint32_t regval = getreg16(regaddr);

  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, false);
  return regval;
}
#else
static inline uint32_t sam_getreg16(uintptr_t regaddr)
{
  return getreg16(regaddr);
}
#endif

/****************************************************************************
 * Name: sam_putreg16
 *
 * Description:
 *   Set the contents of an 16-bit SAMD2L2 USB register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD2L2_USB_REGDEBUG
static void sam_putreg16(uint16_t regval, uintptr_t regaddr)
{
  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, true);

  /* Write the value */

  putreg16(regval, regaddr);
}
#else
static inline void sam_putreg16(uint16_t regval, uintptr_t regaddr)
{
  putreg16(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: sam_getreg8
 *
 * Description:
 *   Get the contents of an 8-bit SAMD2L2 USB register
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD2L2_USB_REGDEBUG
static uint32_t sam_getreg8(uintptr_t regaddr)
{
  /* Read the value from the register */

  uint32_t regval = getreg8(regaddr);

  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, false);
  return regval;
}
#else
static inline uint32_t sam_getreg8(uintptr_t regaddr)
{
  return getreg8(regaddr);
}
#endif

/****************************************************************************
 * Name: sam_putreg8
 *
 * Description:
 *   Set the contents of an 8-bit SAMD2L2 USB register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_SAMD2L2_USB_REGDEBUG
static void sam_putreg8(uint8_t regval, uintptr_t regaddr)
{
  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, true);

  /* Write the value */

  putreg8(regval, regaddr);
}
#else
static inline void sam_putreg8(uint8_t regval, uintptr_t regaddr)
{
  putreg8(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: sam_dumpep
 ****************************************************************************/

#if defined(CONFIG_SAMD2L2_USB_REGDEBUG) && defined(CONFIG_DEBUG_USB)
static void sam_dumpep(struct sam_usbdev_s *priv, uint8_t epno)
{
  /* Global Registers */

  uinfo("Global Registers:\n");
  uinfo("       CTRLB:    %04x\n",
        sam_getreg16(SAM_USBDEV_CTRLB));
  uinfo("        FNUM:    %04x\n",
        sam_getreg16(SAM_USBDEV_FNUM));
  uinfo("        DADD:    %02x\n",
        sam_getreg8(SAM_USBDEV_DADD));
  uinfo("       INTEN:    %04x\n",
        sam_getreg16(SAM_USBDEV_INTENSET));
  uinfo("      STATUS:    %02x\n",
        sam_getreg8(SAM_USBDEV_STATUS));
  uinfo("     INTFLAG:    %04x\n",
        sam_getreg16(SAM_USBDEV_INTFLAG));
  uinfo("   EPCFG[%d]:    %02x\n",
        epno, sam_getreg8(SAM_USBDEV_EPCFG(epno)));
  uinfo("EPSTATUS[%d]:    %02x\n",
        epno, sam_getreg8(SAM_USBDEV_EPSTATUS(epno)));
}
#endif

/****************************************************************************
 * Name: sam_req_dequeue
 ****************************************************************************/

static struct sam_req_s *sam_req_dequeue(struct sam_rqhead_s *queue)
{
  struct sam_req_s *ret = queue->head;

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
 * Name: sam_req_enqueue
 ****************************************************************************/

static void sam_req_enqueue(struct sam_rqhead_s *queue,
                            struct sam_req_s *req)
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
 * Name: sam_req_complete
 ****************************************************************************/

static void sam_req_complete(struct sam_ep_s *privep, int16_t result)
{
  struct sam_req_s *privreq;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags = enter_critical_section();
  privreq = sam_req_dequeue(&privep->reqq);
  leave_critical_section(flags);

  if (privreq)
    {
      /* Save the result in the request structure */

      privreq->req.result = result;

      /* Callback to the request completion handler */

      privreq->flink = NULL;
      privreq->req.callback(&privep->ep, &privreq->req);

      /* mark endpoint ready to next transmission */

      privep->epstate = USB_EPSTATE_IDLE;
      privep->zlpsent = false;
    }
}

/****************************************************************************
 * Name: sam_req_wrsetup
 *
 * Description:
 *   Process the next queued write request.
 *
 ****************************************************************************/

static void sam_req_wrsetup(struct sam_usbdev_s *priv,
                            struct sam_ep_s *privep,
                            struct sam_req_s *privreq)
{
  const uint8_t *buf;
  uint8_t epno;
  int nbytes;
  uint32_t packetsize;

  /* Get the unadorned endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* Get the number of bytes remaining to be sent. */

  DEBUGASSERT(privreq->req.xfrd < privreq->req.len);
  nbytes = privreq->req.len - privreq->req.xfrd;

  /* Either send the maxpacketsize(multi) or all of the remaining data in
   * the request.
   */

  if (nbytes >= SAM_MAX_MULTIPACKET_SIZE)
    {
      nbytes = SAM_MAX_MULTIPACKET_SIZE;
    }

  /* This is the new number of bytes "in-flight" */

  privreq->inflight = nbytes;
  usbtrace(TRACE_WRITE(USB_EPNO(privep->ep.eplog)), nbytes);

  /* The new buffer pointer is the start of the buffer plus the number of
   * bytes successfully transferred plus the number of bytes previously
   * "in-flight".
   */

  buf = privreq->req.buf + privreq->req.xfrd;

  /* setup TX transfer using ep configured maxpacket size */

  priv->eplist[epno].descb[1]->addr = (uint32_t) buf;
  packetsize = priv->eplist[epno].descb[1]->pktsize;
  packetsize &= ~USBDEV_PKTSIZE_BCNT_MASK;
  packetsize &= ~USBDEV_PKTSIZE_MPKTSIZE_MASK;
  packetsize |= USBDEV_PKTSIZE_BCNT(nbytes);

  /* also set automatic ZLP sending if requested on req */

  if (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT)
    {
      packetsize |= USBDEV_PKTSIZE_AUTOZLP;
    }

  priv->eplist[epno].descb[1]->pktsize = packetsize;

  /* Indicate that we are in the sending state
   * This indication will be need in interrupt processing (TRCPT1)
   * in order to properly terminate the request.
   */

  privep->epstate = USB_EPSTATE_SENDING;

  /* Set BK1RDY to notify the USB hardware that TX data is ready on
   * descriptor bank1.  We will be notified that the descriptor has been
   * transmitted by the USB device when TRCPT1 in the endpoint's EPINTFLAG
   * register has been set.
   */

  sam_putreg8(USBDEV_EPSTATUS_BK1RDY, SAM_USBDEV_EPSTATUSSET(epno));
}

/****************************************************************************
 * Name: sam_req_write
 *
 * Description:
 *   Process the next queued write request. This function is called in one
 *   of three contexts:
 *     (1) When the endpoint is IDLE and a new write request is submitted
 *         (with interrupts disabled),
 *     (2) from TRCPT1 interrupt handling when the current Tx transfer
 *         completes
 *     (3) when resuming a stalled IN or control endpoint.
 *
 *   Calling rules:
 *
 *     The transfer state must IDLE
 *
 *     When a request is queued, the request 'len' is the number of bytes
 *     to transfer and 'xfrd' and 'inflight' must be zero.
 *
 *     When this function starts a transfer it will update the request
 *     'inflight' field to indicate the size of the transfer.
 *
 *     When the transfer completes, the 'inflight' field must hold the
 *     number of bytes that have completed the transfer.  This function will
 *     update 'xfrd' with the new size of the transfer.
 *
 ****************************************************************************/

static int sam_req_write(struct sam_usbdev_s *priv, struct sam_ep_s *privep)
{
  struct sam_req_s *privreq;
  uint8_t epno;
  int bytesleft;

  /* Get the unadorned endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* We get here when an IN endpoint interrupt occurs.  So now we know that
   * there is no TX transfer in progress (epstate should be IDLE).
   */

  DEBUGASSERT(privep->epstate == USB_EPSTATE_IDLE);
  while (privep->epstate == USB_EPSTATE_IDLE)
    {
      /* Check the request from the head of the endpoint request queue */

      privreq = sam_rqpeek(&privep->reqq);
      if (!privreq)
        {
          /* There is no TX transfer in progress and no new pending TX
           * requests to send.
           */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPINQEMPTY), epno);

          /* Was there a pending endpoint stall? */

          if (privep->pending)
            {
              /* Yes... stall the endpoint now */

              sam_ep_stall(privep);
            }

          return -ENOENT;
        }

      uinfo("epno=%d req=%p: len=%d xfrd=%d inflight=%d\n",
            epno, privreq, privreq->req.len, privreq->req.xfrd,
            privreq->inflight);

      /* Handle any bytes in flight. */

      privreq->req.xfrd += privreq->inflight;
      privreq->inflight  = 0;

      /* Get the number of bytes left to be sent in the packet */

      bytesleft = privreq->req.len - privreq->req.xfrd;
      if (bytesleft > 0)
        {
          /* Perform the write operation.  epstate will become SENDING. */

          sam_req_wrsetup(priv, privep, privreq);
        }

      /* No data to send...
       * This can happen on one of two ways:
       * (1) The last packet sent was the final packet of a transfer.
       *     Or
       * (2) called with a request packet that has len == 0
       *
       * len == 0 means that it is requested to send a zero length packet
       * required by protocol
       */

      else if ((privreq->req.len == 0) && !privep->zlpsent)
        {
          /* If we get here, we requested to send the zero length packet now.
           */

          privep->epstate   = USB_EPSTATE_SENDING;
          privep->zlpsent   = true;
          privreq->inflight = 0;

          usbtrace(TRACE_WRITE(epno), 0);

          /* setup 0 length TX transfer */

          priv->eplist[0].descb[1]->addr     = (uint32_t) &priv->ep0out[0];
          priv->eplist[0].descb[1]->pktsize &= ~USBDEV_PKTSIZE_BCNT_MASK;
          priv->eplist[0].descb[1]->pktsize &= ~USBDEV_PKTSIZE_MPKTSIZE_MASK;
          priv->eplist[0].descb[1]->pktsize |= USBDEV_PKTSIZE_BCNT(0);
          sam_putreg8(USBDEV_EPSTATUS_BK1RDY, SAM_USBDEV_EPSTATUSSET(epno));
        }

      /* If all of the bytes were sent (including any final zero length
       * packet) then we are finished with the request buffer and we can
       * return the request buffer to the class driver.  The state will
       * remain IDLE only if nothing else was put in flight.
       *
       * Note that we will then loop to check to check the next queued
       * write request.
       */

      if (privep->epstate == USB_EPSTATE_IDLE)
        {
          /* Return the write request to the class driver.  Set the txbusy
           * bit to prevent being called recursively from any new submission
           * generated by returning the write request.
           */

          usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
          DEBUGASSERT(privreq->req.len == privreq->req.xfrd);

          privep->txbusy = true;
          sam_req_complete(privep, OK);
          privep->txbusy = false;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sam_req_read
 *
 * Description:
 *   Complete the last read request. full or partial.
 *   The USB core has transferred the data to user request buffer.
 *   return the completed read request to the class
 *   implementation, and try to start the next queued read request.
 *
 *   REVISIT:
 *   This function is called in one of two contexts:
 *   The normal case is
 *   (1) When the endpoint is IDLE and a new read request is submitted
 *         (with interrupts disabled),
 *   (2) from interrupt handling when the current RX transfer completes.
 *       But there is also a special case
 *   (3) when the OUT endpoint is stopped because there are no
 *       available read requests.
 *
 *   Calling rules:
 *
 *     The transfer state must IDLE
 *
 *     When a request is queued, the request 'len' is size of the request
 *     buffer.  Any OUT request can be received that will fit in this
 *     buffer.  'xfrd' and 'inflight' in the request must be zero
 *     If sam_req_read() is called to start a new transfer, the recvsize
 *     parameter must be zero.
 *
 *     When the transfer completes, the 'recvsize' is the number of bytes
 *     ready on req->buffer.
 *
 ****************************************************************************/

static int sam_req_read(struct sam_usbdev_s *priv, struct sam_ep_s *privep,
                        uint16_t recvsize)
{
  struct sam_req_s *privreq;
  int epno;

  DEBUGASSERT(priv && privep && privep->epstate == USB_EPSTATE_IDLE);

  /* Check the request from the head of the endpoint request queue */

  epno = USB_EPNO(privep->ep.eplog);
  do
    {
      /* Peek at the next read request in the request queue */

      privreq = sam_rqpeek(&privep->reqq);
      if (!privreq)
        {
          /* No read request to receive data */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPOUTQEMPTY), epno);

          /* When no read requests are pending no EP descriptors are set to
           * ready. HW sends NAK to host if it tries to send something.
           */

          privep->epstate = USB_EPSTATE_RXSTOPPED;
          return -ENOENT;
        }

      uinfo("EP%d: req.len=%d xfrd=%d recvsize=%d\n",
            epno, privreq->req.len, privreq->req.xfrd, recvsize);

      /* Ignore any attempt to receive a zero length packet */

      if (privreq->req.len == 0)
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EPOUTNULLPACKET), 0);
          sam_req_complete(privep, OK);
          privreq = NULL;
        }

      /* complete read request with available data */

      if ((privreq->inflight) && (recvsize != 0))
        {
          usbtrace(TRACE_READ(USB_EPNO(privep->ep.eplog)), recvsize);

          /* Update the total number of bytes transferred */

          privreq->req.xfrd += recvsize;
          privreq->inflight  = 0;
          usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
          sam_req_complete(privep, OK);

          /* need to set recvsize to zero.  When calling sam_req_complete()
           * class driver could call submit() again and we have new request
           * ready on next while() loop.
           */

          privep->rxactive = false;
          recvsize = 0;
          privreq = NULL;
        }
    }
  while (privreq == NULL);

  DEBUGASSERT(recvsize == 0);

  /* activate new read request from queue */

  privep->rxactive  = true;
  privreq->req.xfrd = 0;
  privreq->inflight = privreq->req.len;
  priv->eplist[epno].descb[0]->addr = (uint32_t) privreq->req.buf;
  sam_putreg8(USBDEV_EPSTATUS_BK0RDY, SAM_USBDEV_EPSTATUSCLR(epno));

  return OK;
}

/****************************************************************************
 * Name: sam_req_cancel
 ****************************************************************************/

static void sam_req_cancel(struct sam_ep_s *privep, int16_t result)
{
  /* Complete every queued request with the specified status */

  while (!sam_rqempty(&privep->reqq))
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               (sam_rqpeek(&privep->reqq))->req.xfrd);
      sam_req_complete(privep, result);
    }
}

/****************************************************************************
 * Name: sam_ep_configure_internal
 *
 * Description:
 *   This is the internal implementation of the endpoint configuration logic
 *   and implements the endpoint configuration method of the usbdev_ep_s
 *   interface.  As an internal interface, it will be used to configure
 *   endpoint 0 which is not available to the class implementation.
 *
 ****************************************************************************/

static int sam_ep_configure_internal(struct sam_ep_s *privep,
                                     const struct usb_epdesc_s *desc)
{
  uint8_t epconf;
  uint16_t maxpacket;
  uint8_t epno;
  uint8_t eptype;
  uint8_t intflags;
  uint32_t ephwsize;
  bool dirin;

  DEBUGASSERT(privep && privep->dev && desc);

  uinfo("len: %02x type: %02x addr: %02x attr: %02x "
        "maxpacketsize: %02x %02x interval: %02x\n",
        desc->len, desc->type, desc->addr, desc->attr,
        desc->mxpacketsize[0],  desc->mxpacketsize[1],
        desc->interval);

  /* Decode the endpoint descriptor */

  epno      = USB_EPNO(desc->addr);
  dirin     = (desc->addr & USB_DIR_MASK) == USB_REQ_DIR_IN;
  eptype    = (desc->attr & USB_EP_ATTR_XFERTYPE_MASK);
  maxpacket = GETUINT16(desc->mxpacketsize);

  if (maxpacket <= 8)
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_8B;
    }
  else if (maxpacket <= 16)
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_16B;
    }
  else if (maxpacket <= 32)
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_32B;
    }
  else if (maxpacket <= 64)
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_64B;
    }
  else if ((maxpacket <= 128) && (eptype == USB_EP_ATTR_XFER_ISOC))
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_128B;
    }
  else if ((maxpacket <= 256) && (eptype == USB_EP_ATTR_XFER_ISOC))
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_256B;
    }
  else if ((maxpacket <= 512) && (eptype == USB_EP_ATTR_XFER_ISOC))
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_512B;
    }
  else if ((maxpacket <= 1023) && (eptype == USB_EP_ATTR_XFER_ISOC))
    {
      ephwsize = USBDEV_PKTSIZE_SIZE_1023B;
    }
  else
    {
      return -EINVAL;
    }

  /* update endpoint descriptors to correct size */

  privep->descb[0]->pktsize = ephwsize;
  privep->descb[1]->pktsize = ephwsize;

  /* Initialize the endpoint structure */

  privep->ep.eplog     = desc->addr;  /* Includes direction */
  privep->ep.maxpacket = maxpacket;
  privep->epstate      = USB_EPSTATE_IDLE;

  /* get current config IN and OUT */

  epconf = 0x00;
  sam_putreg8(0x00, SAM_USBDEV_EPCFG(epno));

  if (dirin)
    {
      /* Disable bank1 (IN) */

      intflags = USBDEV_EPINT_TRCPT1 | USBDEV_EPINT_STALL1;
    }
  else
    {
      /* Disable bank0 (OUT) */

      intflags = USBDEV_EPINT_TRCPT0 | USBDEV_EPINT_STALL0;
    }

  /* write back disabled config */

  sam_putreg8(0x7e, SAM_USBDEV_EPINTENCLR(epno));
  sam_putreg8(0x7e, SAM_USBDEV_EPINTFLAG(epno));

  /* Re-configure and enable the endpoint */

  switch (eptype)
    {
    case USB_EP_ATTR_XFER_CONTROL:
        {
          epconf = USBDEV_EPCCFG_EPTYPE0_CTRLOUT |
                   USBDEV_EPCCFG_EPTYPE1_CTRLIN;

          /* Also enable IN interrupts */

          intflags =  USBDEV_EPINT_TRCPT0 | USBDEV_EPINT_STALL0;
          intflags |= USBDEV_EPINT_TRCPT1 | USBDEV_EPINT_STALL1;
          intflags |= USBDEV_EPINT_RXSTP;
          sam_putreg8(USBDEV_EPSTATUS_BK0RDY, SAM_USBDEV_EPSTATUSSET(0));
          sam_putreg8(USBDEV_EPSTATUS_BK1RDY, SAM_USBDEV_EPSTATUSCLR(0));
        }
      break;

#ifdef CONFIG_USBDEV_ISOCHRONOUS
    case USB_EP_ATTR_XFER_ISOC:
      if (dirin)
        {
          epconf |= USBDEV_EPCCFG_EPTYPE1_ISOCIN;
        }
      else
        {
          epconf |= USBDEV_EPCCFG_EPTYPE0_ISOCOUT;
        }
      break;
#endif

    case USB_EP_ATTR_XFER_BULK:
      if (dirin)
        {
          epconf |= USBDEV_EPCCFG_EPTYPE1_BULKIN;
        }
      else
        {
          epconf |= USBDEV_EPCCFG_EPTYPE0_BULKOUT;
        }
      break;

    case USB_EP_ATTR_XFER_INT:
      if (dirin)
        {
          epconf |= USBDEV_EPCCFG_EPTYPE1_INTIN;
        }
      else
        {
          epconf |= USBDEV_EPCCFG_EPTYPE0_INTOUT;
        }
      break;

    default:
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPTYPE),
               eptype >> USB_EP_ATTR_XFERTYPE_SHIFT);
      return -EINVAL;
    }

  sam_putreg8(epconf, SAM_USBDEV_EPCFG(epno));

  /* Enable endpoint interrupts */

  sam_putreg8(intflags, SAM_USBDEV_EPINTENSET(epno));
  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPCONF),  epno << 8 | epconf);
  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPINTEN), epno << 8 | intflags);

  sam_dumpep(privep->dev, epno);
  return OK;
}

/****************************************************************************
 * Name: sam_ep_reserve
 *
 * Description:
 *   Find and un-reserved endpoint number and reserve it for the caller.
 *
 ****************************************************************************/

static inline struct sam_ep_s *
sam_ep_reserve(struct sam_usbdev_s *priv, uint8_t epset)
{
  struct sam_ep_s *privep = NULL;
  irqstate_t flags;
  int epndx = 0;

  flags  = enter_critical_section();
  epset &= priv->epavail;
  if (epset)
    {
      /* Select the lowest bit in the set of matching, available endpoints
       * (skipping EP0)
       */

      for (epndx = 1; epndx < SAM_USB_NENDPOINTS; epndx++)
        {
          uint8_t bit = SAM_EP_BIT(epndx);
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
 * Name: sam_ep_unreserve
 *
 * Description:
 *   The endpoint is no long in-used.  It will be unreserved and can be
 *   re-used if needed.
 *
 ****************************************************************************/

static inline void
sam_ep_unreserve(struct sam_usbdev_s *priv, struct sam_ep_s *privep)
{
  irqstate_t flags = enter_critical_section();
  priv->epavail   |= SAM_EP_BIT(USB_EPNO(privep->ep.eplog));
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_ep_configure
 *
 * Description:
 *   This is the endpoint configuration method of the usbdev_ep_s interface.
 *
 ****************************************************************************/

static int sam_ep_configure(struct usbdev_ep_s *ep,
                            const struct usb_epdesc_s *desc,
                            bool last)
{
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
  int ret;

  /* Verify parameters.  Endpoint 0 is not available at this interface */

#if defined(CONFIG_DEBUG_USB) || defined(CONFIG_USBDEV_TRACE)
  uint8_t epno = USB_EPNO(desc->addr);
  usbtrace(TRACE_EPCONFIGURE, (uint16_t)epno);

  DEBUGASSERT(ep && desc && epno > 0 && epno < SAM_USB_NENDPOINTS);
  DEBUGASSERT(epno == USB_EPNO(ep->eplog));
#endif

  /* This logic is implemented in sam_ep_configure_internal */

  ret = sam_ep_configure_internal(privep, desc);

  /* If this was the last endpoint, then the class driver is fully
   * configured.
   */

  if (ret == OK && last)
    {
      struct sam_usbdev_s *priv = privep->dev;

      /* Go to the configured state (we should have been in the addressed
       * state)
       */

      DEBUGASSERT(priv && priv->devstate == USB_DEVSTATE_ADDRESSED);
      priv->devstate = USB_DEVSTATE_CONFIGURED;
    }

  return ret;
}

/****************************************************************************
 * Name: sam_ep_disable
 *
 * Description:
 *   This is the disable() method of the USB device endpoint structure.
 *
 ****************************************************************************/

static int sam_ep_disable(struct usbdev_ep_s *ep)
{
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
  struct sam_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;

#ifdef CONFIG_DEBUG_USB
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      uerr("ERROR: ep=%p\n", ep);
      return -EINVAL;
    }
#endif

  epno = USB_EPNO(ep->eplog);
  usbtrace(TRACE_EPDISABLE, epno);

  /* Reset the endpoint and cancel any ongoing activity */

  flags = enter_critical_section();
  priv  = privep->dev;
  sam_ep_reset(priv, epno);

  /* Revert to the addressed-but-not-configured state */

  sam_setdevaddr(priv, priv->devaddr);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_ep_allocreq
 *
 * Description:
 *   This is the allocreq() method of the USB device endpoint structure.
 *
 ****************************************************************************/

static struct usbdev_req_s *sam_ep_allocreq(struct usbdev_ep_s *ep)
{
  struct sam_req_s *privreq;

#ifdef CONFIG_DEBUG_USB
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  usbtrace(TRACE_EPALLOCREQ, USB_EPNO(ep->eplog));

  privreq = (struct sam_req_s *)kmm_malloc(sizeof(struct sam_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct sam_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: sam_ep_freereq
 *
 * Description:
 *   This is the freereq() method of the USB device endpoint structure.
 *
 ****************************************************************************/

static void sam_ep_freereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct sam_req_s *privreq = (struct sam_req_s *)req;

#ifdef CONFIG_DEBUG_USB
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif
  usbtrace(TRACE_EPFREEREQ, USB_EPNO(ep->eplog));

  kmm_free(privreq);
}

#if 0 /* FIXME, not required, check... */

/****************************************************************************
 * Name: sam_ep_allocbuffer
 *
 * Description:
 *   This is the allocbuffer() method of the USB device endpoint structure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *sam_ep_allocbuffer(struct usbdev_ep_s *ep, uint16_t nbytes)
{
  /* There is not special buffer allocation requirement */

  return kumm_malloc(nbytes);
}
#endif

/****************************************************************************
 * Name: sam_ep_freebuffer
 *
 * Description:
 *   This is the freebuffer() method of the USB device endpoint structure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void sam_ep_freebuffer(struct usbdev_ep_s *ep, void *buf)
{
  /* There is not special buffer allocation requirement */

  kumm_free(buf);
}
#endif

#endif

/****************************************************************************
 * Name: sam_ep_submit
 *
 * Description:
 *   This is the submit() method of the USB device endpoint structure.
 *
 ****************************************************************************/

static int sam_ep_submit(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct sam_req_s *privreq = (struct sam_req_s *)req;
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
  struct sam_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;
  int ret = OK;

#ifdef CONFIG_DEBUG_USB
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      uerr("ERROR: req=%p callback=%p buf=%p ep=%p\n", req, req->callback,
           req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, USB_EPNO(ep->eplog));
  priv = privep->dev;

#ifdef CONFIG_DEBUG_USB
  if (!priv->driver)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_NOTCONFIGURED),
               priv->usbdev.speed);
      uerr("ERROR: driver=%p\n", priv->driver);
      return -ESHUTDOWN;
    }
#endif

  /* Handle the request from the class driver */

  epno              = USB_EPNO(ep->eplog);
  req->result       = -EINPROGRESS;
  req->xfrd         = 0;
  privreq->inflight = 0;
  flags             = enter_critical_section();

  /* Handle IN (device-to-host) requests.  NOTE:  If the class device is
   * using the bi-directional EP0, then we assume that they intend the EP0
   * IN functionality (EP0 SETUP OUT data receipt does not use requests).
   */

  if (USB_ISEPIN(ep->eplog) || epno == EP0)
    {
      /* Check if the endpoint is stalled (or there is a stall pending) */

      if (privep->stalled || privep->pending)
        {
          /* Yes.. in this case, save the new they will get in a special
           * "pending" they will get queue until the stall is cleared.
           */

          uinfo("Pending stall clear\n");
          sam_req_enqueue(&privep->pendq, privreq);
          usbtrace(TRACE_INREQQUEUED(epno), req->len);
          ret = OK;
        }

      else
        {
          /* Add the new request to the request queue for the IN endpoint */

          sam_req_enqueue(&privep->reqq, privreq);
          usbtrace(TRACE_INREQQUEUED(epno), req->len);

          /* If the IN endpoint is IDLE and there is not write queue
           * processing in progress, then transfer the data now.
           */

          if (privep->epstate == USB_EPSTATE_IDLE && !privep->txbusy)
            {
              ret = sam_req_write(priv, privep);
            }
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      sam_req_enqueue(&privep->reqq, privreq);
      usbtrace(TRACE_OUTREQQUEUED(epno), req->len);

      /* Check if we have stopped RX receipt due to lack of read
       * requests.  In that case we are not receiving anything from host.
       * and HW sends NAK to host. see sam_req_read()
       * so this "state" is actually not required (at least yet)
       */

      if (privep->epstate == USB_EPSTATE_RXSTOPPED)
        {
          privep->epstate = USB_EPSTATE_IDLE;
        }

      /* start new read if no active yet */

      if (!privep->rxactive)
        {
          ret = sam_req_read(priv, privep, 0);
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: sam_ep_cancel
 ****************************************************************************/

static int sam_ep_cancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG_USB
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));

  flags = enter_critical_section();
  sam_req_cancel(privep, -EAGAIN);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_ep_stallresume
 ****************************************************************************/

static int sam_ep_stallresume(struct usbdev_ep_s *ep, bool resume)
{
  struct sam_ep_s *privep;
  uint8_t epno;
  irqstate_t flags;
  int ret;

#ifdef CONFIG_DEBUG_USB
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Handle the resume condition */

  privep = (struct sam_ep_s *)ep;
  if (resume)
    {
      ret = sam_ep_resume(privep);
    }

  /* Handle the stall condition */

  else
    {
      /* If this is an IN endpoint (and not EP0) and if there are queued
       * write requests, then we cannot stall now.  Perhaps this is a
       * protocol stall.  In that case, we will need to drain the write
       * requests before sending the stall.
       */

      flags = enter_critical_section();
      epno = USB_EPNO(ep->eplog);
      if (epno != 0 && USB_ISEPIN(ep->eplog))
        {
          /* Are there any unfinished write requests in the request queue? */

          if (!sam_rqempty(&privep->reqq))
            {
              /* Just set a flag to indicate that the endpoint must be
               * stalled on the next TRCPTx interrupt when the request
               * queue becomes empty.
               */

              privep->pending = true;
              leave_critical_section(flags);
              return OK;
            }
        }

      /* Not an IN endpoint, endpoint 0, or no pending write requests.
       * Stall the endpoint now.
       */

      ret = sam_ep_stall(privep);
      leave_critical_section(flags);
    }

  return ret;
}

/****************************************************************************
 * Name: sam_allocep
 *
 * Description:
 *   This is the allocep() method of the USB device driver interface
 *
 ****************************************************************************/

static struct usbdev_ep_s *sam_allocep(struct usbdev_s *dev, uint8_t epno,
                                         bool in, uint8_t eptype)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;
  struct sam_ep_s *privep = NULL;
  uint16_t epset = SAM_EPSET_NOTEP0;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)epno);
#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  /* Ignore any direction bits in the logical address */

  epno = USB_EPNO(epno);

  /* A logical address of 0 means that any endpoint will do */

  if (epno > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the
       * requested 'logical' endpoint.  All of the other checks will still
       * be performed.
       *
       * First, verify that the logical endpoint is in the range supported by
       * by the hardware.
       */

      if (epno >= SAM_USB_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPNO), (uint16_t)epno);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset = SAM_EP_BIT(epno);
    }

  /* Check if the selected endpoint number is available */

  privep = sam_ep_reserve(priv, epset);
  if (!privep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EPRESERVE), (uint16_t)epset);
      return NULL;
    }

  return &privep->ep;
}

/****************************************************************************
 * Name: sam_freeep
 *
 * Description:
 *   This is the freeep() method of the USB device driver interface
 *
 ****************************************************************************/

static void sam_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct sam_usbdev_s *priv;
  struct sam_ep_s *privep;

#ifdef CONFIG_DEBUG_USB
  if (!dev || !ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif
  priv   = (struct sam_usbdev_s *)dev;
  privep = (struct sam_ep_s *)ep;
  usbtrace(TRACE_DEVFREEEP, (uint16_t)USB_EPNO(ep->eplog));

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      sam_ep_unreserve(priv, privep);
    }
}

/****************************************************************************
 * Name: sam_getframe
 *
 * Description:
 *   This is the getframe() method of the USB device driver interface
 *
 ****************************************************************************/

static int sam_getframe(struct usbdev_s *dev)
{
  uint32_t regval;
  uint16_t frameno;

#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Return the last frame number detected by the hardware */

  regval  = sam_getreg16(SAM_USBDEV_FNUM);
  frameno = (regval & USBDEV_FNUM_MASK) >> USBDEV_FNUM_SHIFT;

  usbtrace(TRACE_DEVGETFRAME, frameno);
  return frameno;
}

/****************************************************************************
 * Name: sam_wakeup
 *
 * Description:
 *   This is the wakeup() method of the USB device driver interface
 *
 ****************************************************************************/

static int sam_wakeup(struct usbdev_s *dev)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;
  irqstate_t flags;
  uint16_t regval;

  usbtrace(TRACE_DEVWAKEUP, 0);
#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Resume normal operation */

  flags = enter_critical_section();
  sam_resume(priv);

  /* Activate a remote wakeup. (aka upstream resume)
   * Setting the Remote Wakeup bit in CTRLB.UPRSM starts the
   * Remote Wake Up procedure.
   *
   * This will automatically be done by the controller after 5 ms of
   * inactivity on the USB bus.
   *
   * When the controller sends the Upstream Resume INTFLAG.WAKEUP is set
   * and INTFLAG.SUSPEND is cleared.
   * The CTRLB.UPRSM is cleared at the end of the transmitting Upstream
   * Resume.
   */

  regval  = sam_getreg16(SAM_USBDEV_CTRLB);
  regval |= USBDEV_CTRLB_UPRSM;
  sam_putreg16(regval, SAM_USBDEV_CTRLB);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_selfpowered
 *
 * Description:
 *   This is the selfpowered() method of the USB device driver interface
 *
 ****************************************************************************/

static int sam_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG_USB
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Name: sam_suspend
 ****************************************************************************/

#if 0 /* Not used */
static void sam_suspend(struct sam_usbdev_s *priv)
{
  /* Don't do anything if the device is already suspended */

  if (priv->devstate != USB_DEVSTATE_SUSPENDED)
    {
      /* Notify the class driver of the suspend event */

      if (priv->driver)
        {
          CLASS_SUSPEND(priv->driver, &priv->usbdev);
        }

      /* Switch to the Suspended state */

      priv->prevstate = priv->devstate;
      priv->devstate  = USB_DEVSTATE_SUSPENDED;

      /* Disable clocking to the USB peripheral */

      sam_disableclks();

      /* Let the board-specific logic know that we have entered the
       * suspend state.  This may trigger additional reduced power
       * consumption measures.
       */

      sam_usb_suspend((struct usbdev_s *)priv, false);
    }
}
#endif

/****************************************************************************
 * Name: sam_resume
 ****************************************************************************/

static void sam_resume(struct sam_usbdev_s *priv)
{
  /* This function is called when either (1) a WAKEUP interrupt is received
   * from the host PC, or (2) the class device implementation calls the
   * wakeup() method.
   */

  /* Don't do anything if the device was not suspended */

  if (priv->devstate == USB_DEVSTATE_SUSPENDED)
    {
      /* Revert to the previous state */

      priv->devstate = priv->prevstate;

      /* Restore clocking to the USB peripheral */

      sam_enableclks();

      /* Restore full power -- whatever that means for this particular
       * board
       */

      sam_usb_suspend((struct usbdev_s *)priv, true);

      /* Notify the class driver of the resume event */

      if (priv->driver)
        {
          CLASS_RESUME(priv->driver, &priv->usbdev);
        }
    }
}

/****************************************************************************
 * Name: sam_reset
 ****************************************************************************/

static void sam_reset(struct sam_usbdev_s *priv)
{
  uint16_t regval;
  uint8_t epno;

  /* Make sure that clocking is enabled to the USB peripheral. */

  sam_enableclks();

  /* Tell the class driver that we are disconnected.  The class driver
   * should then accept any new configurations.
   */

  CLASS_DISCONNECT(priv->driver, &priv->usbdev);

  /* The device enters the Default state (un-addressed and un-configured) */

  priv->devaddr   = 0;
  sam_setdevaddr(priv, 0);

  priv->devstate  = USB_DEVSTATE_DEFAULT;

  /* Reset and disable all endpoints. Then re-configure EP0 */

  sam_epset_reset(priv, SAM_EPSET_ALL);
  sam_ep_configure_internal(&priv->eplist[EP0], &g_ep0desc);

  /* set EP0 waiting for SETUP */

  sam_ep0_ctrlread(priv);

  /* Reset endpoint data structures */

  for (epno = 0; epno < SAM_USB_NENDPOINTS; epno++)
    {
      struct sam_ep_s *privep = &priv->eplist[epno];

      /* Cancel any queued requests.  Since they are cancelled
       * with status -ESHUTDOWN, then will not be requeued
       * until the configuration is reset.  NOTE:  This should
       * not be necessary... the CLASS_DISCONNECT above should
       * result in the class implementation calling sam_ep_disable
       * for each of its configured endpoints.
       */

      sam_req_cancel(privep, -ESHUTDOWN);

      /* Reset endpoint status */

      privep->stalled   = false;
      privep->pending   = false;
      privep->halted    = false;
      privep->zlpsent   = false;
      privep->txbusy    = false;
      privep->rxactive  = false;
    }

  /* Re-configure the USB controller in its initial, unconnected state */

  priv->usbdev.speed = USB_SPEED_FULL;

  /* Clear all pending interrupt status */

  regval = USBDEV_INT_SUSPEND | USBDEV_INT_SOF     | USBDEV_INT_EORST |
           USBDEV_INT_WAKEUP  | USBDEV_INT_EORSM   | USBDEV_INT_UPRSM |
           USBDEV_INT_RAMACER | USBDEV_INT_LPMNYET | USBDEV_INT_LPMSUSP;

  sam_putreg16(regval, SAM_USBDEV_INTFLAG);

  /* Enable normal operational interrupts
   * endpoint 0 is enabled on sam_ep_configure_internal()
   */

  regval = USBDEV_INT_SOF | USBDEV_INT_WAKEUP | USBDEV_INT_SUSPEND;
  sam_putreg16(regval, SAM_USBDEV_INTENSET);

  sam_dumpep(priv, EP0);
}

/****************************************************************************
 * Name: sam_ep0_wrstatus
 *
 * Description:
 *   write ep0 status reply back to host
 *
 ****************************************************************************/

static void sam_ep0_wrstatus(struct sam_usbdev_s *priv,
                             const uint8_t *buffer, size_t buflen)
{
  uint32_t packetsize;

  /* we need to make copy of data as source is in stack
   * reusing the static ep0 setup buffer
   */

  DEBUGASSERT(buflen < SAM_EP0_MAXPACKET);
  memcpy(&priv->ep0out[0], buffer, buflen);

  /* set read for next setup OUT */

  sam_ep0_ctrlread(priv);

  /* setup TX transfer */

  priv->eplist[0].descb[1]->addr = (uint32_t) &priv->ep0out[0];
  packetsize = priv->eplist[0].descb[1]->pktsize;
  packetsize &= ~USBDEV_PKTSIZE_BCNT_MASK;
  packetsize &= ~USBDEV_PKTSIZE_MPKTSIZE_MASK;
  packetsize |= USBDEV_PKTSIZE_BCNT(buflen);
  priv->eplist[0].descb[1]->pktsize = packetsize;

  /* Set BK1RDY to notify the USB hardware that TX data is ready on
   * descriptor bank1.  We will be notified that the descriptor has been
   * transmitted by the USB device when TRCPT1 in the endpoint's EPINTFLAG
   * register has been set.
   */

  sam_putreg8(USBDEV_EPSTATUS_BK1RDY, SAM_USBDEV_EPSTATUSSET(0));

  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0WRSTATUS), buflen);
}

/****************************************************************************
 * Name: sam_ep0_dispatch
 ****************************************************************************/

static void sam_ep0_dispatch(struct sam_usbdev_s *priv)
{
  uint8_t *dataout;
  size_t outlen;
  int ret;

  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      /* Assume IN SETUP (or OUT SETUP with no data) */

      dataout = NULL;
      outlen  = 0;

      /* Was this an OUT SETUP command? */

      if (USB_REQ_ISOUT(priv->ctrl.type))
        {
          uint16_t tmplen = GETUINT16(priv->ctrl.len);
          if (tmplen > 0)
            {
              dataout = priv->ep0out;
              outlen  = tmplen;
            }
        }

      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl,
                        dataout, outlen);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_DISPATCHSTALL), 0);
          sam_ep_stall(&priv->eplist[EP0]);
        }
    }
}

/****************************************************************************
 * Name: sam_setdevaddr
 *
 * Description:
 *   This function is called after the completion of the STATUS phase to
 *   instantiate the device address that was received during the SETUP
 *   phase.  This enters the ADDRESSED state from either the DEFAULT or the
 *   CONFIGURED states.
 *
 *   If called with address == 0, then function will revert to the DEFAULT,
 *   un-configured and un-addressed state.
 *
 ****************************************************************************/

static void sam_setdevaddr(struct sam_usbdev_s *priv, uint8_t address)
{
  DEBUGASSERT(address <= 0x7f);
  if (address)
    {
      /* Enable the address */

      address |= USBDEV_DADD_ADDEN;
      sam_putreg8(address, SAM_USBDEV_DADD);

      /* Go to the addressed but not configured state */

      priv->devstate = USB_DEVSTATE_ADDRESSED;
    }
  else
    {
      /* Set address to zero. clear ADDEN bit */

      sam_putreg8(0x00, SAM_USBDEV_DADD);

      /* Revert to the un-addressed, default state */

      priv->devstate = USB_DEVSTATE_DEFAULT;
    }
}

/****************************************************************************
 * Name: sam_ep0_setup
 *
 * Description:
 *   This function is called after the receiving of the SETUP packet
 *   data is ready on usb_ctrlreq_s struct
 *
 ****************************************************************************/

static void sam_ep0_setup(struct sam_usbdev_s *priv)
{
  struct sam_ep_s     *ep0 = &priv->eplist[EP0];
  struct sam_ep_s     *privep;
  union wb_u           value;
  union wb_u           index;
  union wb_u           len;
  union wb_u           response;
  enum sam_ep0setup_e  ep0result;
  uint8_t              epno;
  int                  nbytes = 0; /* Assume zero-length packet */
  int                  ret;

  /* Terminate any pending requests */

  sam_req_cancel(ep0, -EPROTO);

  /* Assume NOT stalled; no TX in progress */

  ep0->stalled  = false;
  ep0->pending  = false;
  ep0->epstate  = USB_EPSTATE_IDLE;

  /* And extract the little-endian 16-bit values to host order */

  value.w = GETUINT16(priv->ctrl.value);
  index.w = GETUINT16(priv->ctrl.index);
  len.w   = GETUINT16(priv->ctrl.len);

  uinfo("SETUP: type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        priv->ctrl.type, priv->ctrl.req, value.w, index.w, len.w);

  /* Dispatch any non-standard requests */

  if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_NOSTDREQ), priv->ctrl.type);

      /* Let the class implementation handle all non-standard requests */

      sam_ep0_dispatch(priv);
      return;
    }

  /* Handle standard request.  Pick off the things of interest to the
   * USB device controller driver; pass what is left to the class driver
   */

  ep0result = USB_EP0SETUP_SUCCESS;
  switch (priv->ctrl.req)
    {
    case USB_REQ_GETSTATUS:
      {
        /* type:  device-to-host; recipient = device, interface, endpoint
         * value: 0
         * index: zero interface endpoint
         * len:   2; data = status
         */

        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETSTATUS), priv->ctrl.type);
        if (len.w != 2 || (priv->ctrl.type & USB_REQ_DIR_IN) == 0 ||
            index.b[MSB] != 0 || value.w != 0)
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPGETSTATUS), 0);
            ep0result = USB_EP0SETUP_STALL;
          }
        else
          {
            switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
               case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  epno = USB_EPNO(index.b[LSB]);
                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETSTATUS), epno);
                  if (epno >= SAM_USB_NENDPOINTS)
                    {
                      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPGETSTATUS),
                               epno);
                      ep0result = USB_EP0SETUP_STALL;
                    }
                  else
                    {
                      privep     = &priv->eplist[epno];
                      response.w = 0; /* Not stalled */
                      nbytes     = 2; /* Response size: 2 bytes */

                      if (privep->stalled)
                        {
                          /* Endpoint stalled */

                          response.b[LSB] = 1; /* Stalled */
                        }
                    }
                }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
                {
                 if (index.w == 0)
                    {
                      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DEVGETSTATUS),
                               0);

                      /* Features:  Remote Wakeup=YES; selfpowered=? */

                      response.w      = 0;
                      response.b[LSB] = (priv->selfpowered <<
                                        USB_FEATURE_SELFPOWERED) |
                                        (1 << USB_FEATURE_REMOTEWAKEUP);
                      nbytes          = 2; /* Response size: 2 bytes */
                    }
                  else
                    {
                      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADDEVGETSTATUS),
                               0);
                      ep0result = USB_EP0SETUP_STALL;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_IFGETSTATUS), 0);
                  response.w = 0;
                  nbytes     = 2; /* Response size: 2 bytes */
                }
                break;

              default:
                {
                  usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETSTATUS), 0);
                  ep0result = USB_EP0SETUP_STALL;
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

        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_CLEARFEATURE),
                 priv->ctrl.type);

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
            USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* Let the class implementation handle all recipients (except
             * for the endpoint recipient)
             */

            sam_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            /* Endpoint recipient */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < SAM_USB_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = false;

                ret = sam_ep_resume(privep);
                if (ret < 0)
                  {
                    ep0result = USB_EP0SETUP_STALL;
                  }
              }
            else
              {
                usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADCLEARFEATURE), 0);
                ep0result = USB_EP0SETUP_STALL;
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

        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SETFEATURE),
                 priv->ctrl.type);
        if (((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE) &&
            value.w == USB_FEATURE_TESTMODE)
          {
            /* Special case recipient=device test mode */

            uinfo("test mode: %d\n", index.w);
          }
        else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
                 USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* The class driver handles all recipients except
             * recipient=endpoint.
             */

            sam_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            /* Handler recipient=endpoint */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < SAM_USB_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = true;

                ret = sam_ep_stall(privep);
                if (ret < 0)
                  {
                    ep0result = USB_EP0SETUP_STALL;
                  }
              }
            else
              {
                usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETFEATURE), 0);
                ep0result = USB_EP0SETUP_STALL;
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

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) !=
            USB_REQ_RECIPIENT_DEVICE ||
            index.w != 0 || len.w != 0 || value.w > 127)
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETADDRESS), 0);
            ep0result = USB_EP0SETUP_STALL;
          }
        else
          {
            /* Note that setting of the device address will be deferred.
             * A zero-length packet will be sent and the device address will
             * be set when the zero-length packet transfer completes.
             */

            usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPSETADDRESS),
                     value.w);

            priv->devaddr = value.w;
            ep0result     = USB_EP0SETUP_ADDRESS;
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
        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETSETDESC),
                 priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE)
          {
            /* The request seems valid... let the class implementation
             * handle it.
             */

            sam_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETSETDESC), 0);
            ep0result = USB_EP0SETUP_STALL;
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
        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETCONFIG),
                 priv->ctrl.type);

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE &&
            value.w == 0 && index.w == 0 && len.w == 1)
          {
            /* The request seems valid... let the class implementation
             * handle it.
             */

            sam_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETCONFIG), 0);
            ep0result = USB_EP0SETUP_STALL;
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
        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SETCONFIG),
                 priv->ctrl.type);

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) ==
            USB_REQ_RECIPIENT_DEVICE &&
            index.w == 0 && len.w == 0)
          {
            /* The request seems valid... let the class implementation
             * handle it.  If the class implementation accepts it new
             * configuration, it will call sam_ep_configure() to configure
             * the endpoints.
             */

            sam_ep0_dispatch(priv);
            ep0result = USB_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETCONFIG), 0);
            ep0result = USB_EP0SETUP_STALL;
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

        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETSETIF), priv->ctrl.type);
        sam_ep0_dispatch(priv);
        ep0result = USB_EP0SETUP_DISPATCHED;
      }
      break;

    case USB_REQ_SYNCHFRAME:
      /* type:  device-to-host; recipient = endpoint
       * value: 0
       * index: endpoint;
       * len:   2; data = frame number
       */

      {
        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SYNCHFRAME), 0);
      }
      break;

    default:
      {
        usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDCTRLREQ),
                 priv->ctrl.req);
        ep0result = USB_EP0SETUP_STALL;
      }
      break;
    }

  /* Restrict the data length to the length requested in the setup packet */

  if (nbytes > len.w)
    {
      nbytes = len.w;
    }

  /* At this point, the request has been handled and there are three
   * (or four) possible outcomes:
   *
   * 1a. ep0result == USB_EP0SETUP_SUCCESS
   *
   *    The setup request was successfully handled above and a response
   *    packet must be sent (may be a zero length packet).
   *
   * 1b. ep0result == USB_EP0SETUP_ADDRESS
   *
   *    A special case is the case where epstate=USB_EPSTATE_EP0ADDRESS.
   *    This means that the above processing generated an additional state
   *    where we need to wait until we complete the status phase before
   *    applying the new device address.
   *
   * 2. ep0result == USB_EP0SETUP_DISPATCHED;
   *
   *    The request was forwarded to the class implementation.  In case,
   *    EP0 IN data may have already been sent and the EP0 IN response
   *    has already been queued?  Or perhaps the endpoint has already
   *    been stalled?  This is all under the control of the class driver.
   *
   *    NOTE that for the case of non-standard SETUP requested, those
   *    requests were forwarded to the class driver and we don't even get
   *    to this logic.
   *
   * 3. ep0result == USB_EP0SETUP_STALL;
   *
   *    An error was detected in either the above logic or by the class
   *    implementation logic.
   */

  switch (ep0result)
    {
      case USB_EP0SETUP_SUCCESS:
        {
          /* Send the response (might be a zero-length packet) */

          ep0->epstate = USB_EPSTATE_EP0STATUSIN;
          sam_ep0_wrstatus(priv, response.b, nbytes);
        }
        break;

      case USB_EP0SETUP_ADDRESS:
        {
          /* Send the response (might be a zero-length packet) */

          ep0->epstate = USB_EPSTATE_EP0ADDRESS;
          sam_ep0_wrstatus(priv, response.b, nbytes);
        }
        break;

      case USB_EP0SETUP_STALL:
        {
          /* Stall EP0 */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EP0SETUPSTALLED),
                   priv->ctrl.req);

          sam_ep_stall(&priv->eplist[EP0]);
        }
        break;

      case USB_EP0SETUP_DISPATCHED:
      default:
        break;
    }
}

/****************************************************************************
 * Name: sam_ctrla_write
 *
 * Description:
 *   writes value to CTRLA register some bits needs write-synchronisation
 *
 ****************************************************************************/

static void sam_ctrla_write(uint8_t value)
{
  sam_putreg8(value, SAM_USB_CTRLA);

  if (value & USB_CTRLA_SWRST)
    {
      /* Due to synchronization there is a delay from writing CTRLA.SWRST
       * until the reset is complete. CTRLA.SWRST and SYNCBUSY.SWRST will
       * both be cleared when the reset is complete.
       */

      while ((sam_getreg8(SAM_USB_CTRLA) & USB_CTRLA_SWRST) &&
             (sam_getreg8(SAM_USB_SYNCBUSY) & USB_SYNCBUSY_SWRST))
        ;

      return;
    }

  if (value & USB_CTRLA_ENABLE)
    {
      /* Due to synchronization there is delay from writing CTRLA.ENABLE
       * until the peripheral is enabled/disabled.
       * SYNCBUSY.ENABLE will be cleared when the operation is complete.
       */

      while ((sam_getreg8(SAM_USB_SYNCBUSY) & USB_SYNCBUSY_ENABLE))
        ;
    }
}

/****************************************************************************
 * Name: sam_ep_trcpt_interrupt
 *
 * Description:
 *   Transmit completed on Bank 0/1
 *   Normal:
 *     OUT data transmit has been completed bank=0
 *   Ping-Pong:
 *     TODO:
 *
 ****************************************************************************/

static void sam_ep_trcpt_interrupt(struct sam_usbdev_s *priv,
                                   struct sam_ep_s *privep,
                                   uint32_t flags, int bank)
{
  uint32_t eptype;
  uint16_t pktsize;
  uint8_t epno;

  /* Get the endpoint type */

  epno   = USB_EPNO(privep->ep.eplog);
  eptype = sam_getreg8(SAM_USBDEV_EPCFG(epno)) & USBDEV_EPCFG_EPTYPE0_MASK;

  /* Are we receiving data for a read request?  EP0 does not receive data
   * using read requests.
   */

  /* Get the size of the packet that we just received */

  pktsize = privep->descb[bank]->pktsize & USBDEV_PKTSIZE_BCNT_MASK;
  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPTRCPT0_LEN), (uint16_t)pktsize);

  if (privep->epstate == USB_EPSTATE_IDLE && epno != 0)
    {
      /* continue processing the read request. */

      sam_req_read(priv, privep, pktsize);
    }

  /* Did we just receive the data associated with an OUT SETUP command? */

  else if (privep->epstate == USB_EPSTATE_EP0DATAOUT)
    {
      uint16_t len;

      DEBUGASSERT(epno == EP0 && bank == 0);

      /* Yes.. back to the IDLE state */

      privep->epstate = USB_EPSTATE_IDLE;

      /* Get the size that we expected to receive */

      len = GETUINT16(priv->ctrl.len);
      if (len == pktsize)
        {
          /* And handle the EP0 SETUP now. */

          sam_ep0_setup(priv);
        }
      else
        {
          /* Then stall. */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EP0SETUPOUTSIZE), pktsize);

          sam_ep_stall(privep);
        }
    }

  /* Check for a EP0 STATUS packet returned by the host at the end of a
   * SETUP status phase
   */

  else if ((eptype == USBDEV_EPCCFG_EPTYPE0_CTRLOUT) && pktsize == 0)
    {
      DEBUGASSERT(epno == EP0 && bank == 0);
    }

  /* Otherwise there is a problem.  Complain an clear the interrupt */

  else
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_RXDATABKERR), privep->epstate);
    }
}

/****************************************************************************
 * Name: sam_ep0_ctrlread
 *
 * Description:
 *   setup 8-byte read req for ep0-ctrl setup phase.
 *   data is received on priv->ep0out buffer.  This is notified by
 *   endpoint TRCPT0 interrupt
 *
 ****************************************************************************/

static void sam_ep0_ctrlread(struct sam_usbdev_s *priv)
{
  priv->eplist[0].descb[0]->addr    = (uint32_t) &priv->ep0out[0];
  priv->eplist[0].descb[0]->pktsize = USBDEV_PKTSIZE_MPKTSIZE(8) |
                                      USBDEV_PKTSIZE_SIZE_64B;
  sam_putreg8(USBDEV_EPSTATUS_BK0RDY, SAM_USBDEV_EPSTATUSCLR(0));
}

/****************************************************************************
 * Name: sam_ep_interrupt
 *
 * Description:
 *   Handle the USB endpoint interrupt
 *
 ****************************************************************************/

static void sam_ep_interrupt(struct sam_usbdev_s *priv, int epno)
{
  struct sam_ep_s *privep;
  uint16_t flags;

  DEBUGASSERT((unsigned)epno < SAM_USB_NENDPOINTS);

  /* Get the endpoint structure */

  privep = &priv->eplist[epno];

  /* Get the endpoint irq */

  flags = sam_getreg8(SAM_USBDEV_EPINTFLAG(epno));
  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPINTFLAGS), flags);

  /* TRCPT1: IN packet sent and acknowledged by the host */

  if ((flags & USBDEV_EPINT_TRCPT1) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPTRCPT1), flags);

      /* Clear the TRCPT1 interrupt */

      sam_putreg8(USBDEV_EPINT_TRCPT1, SAM_USBDEV_EPINTFLAG(epno));

      /* Sending state.  This is the completion of a "normal" write request
       * transfer.  In this case, we need to resume request processing in
       * order to send the next outgoing packet.
       */

      if (privep->epstate == USB_EPSTATE_SENDING ||
          privep->epstate == USB_EPSTATE_EP0STATUSIN)
        {
          /* Continue/resume processing the write requests */

          privep->epstate = USB_EPSTATE_IDLE;
          sam_req_write(priv, privep);
        }

      /* Setting of the device address is a special case.  The address was
       * obtained when a preceding SETADDRESS SETUP command was processed.
       * But the address is not set until the final SETUP status phase
       * completes.  This interrupt indicates the completion of that status
       * phase and now we set the address.
       */

      else if (privep->epstate == USB_EPSTATE_EP0ADDRESS)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_ADDRESSED), priv->devaddr);
          DEBUGASSERT(epno == EP0);

          /* Set the device address */

          privep->epstate = USB_EPSTATE_IDLE;
          sam_setdevaddr(priv, priv->devaddr);
        }
      else
        {
          /* Unexpected TRCPT1 interrupt */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_TXCOMPERR), privep->epstate);
        }
    }

  /* OUT packet received  */

  if ((flags & USBDEV_EPINT_TRCPT0) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPTRCPT0), flags);

      /* Clear the TRCPT0 interrupt. */

      sam_putreg8(USBDEV_EPINT_TRCPT0, SAM_USBDEV_EPINTFLAG(epno));

      sam_ep_trcpt_interrupt(priv, privep, flags, 0);
    }

  /* Endpoint stall */

  if ((flags & USBDEV_EPINT_STALL0) != 0)
    {
      sam_putreg8(USBDEV_EPINT_STALL0, SAM_USBDEV_EPINTFLAG(epno));
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPSTALL0), flags);

      /* If EP is not halted, clear STALL */

      if (privep->epstate != USB_EPSTATE_STALLED)
        {
          sam_putreg8(USBDEV_EPSTATUS_STALLRQ0,
                      SAM_USBDEV_EPSTATUSCLR(epno));
        }
    }

  if ((flags & USBDEV_EPINT_STALL1) != 0)
    {
      sam_putreg8(USBDEV_EPINT_STALL1, SAM_USBDEV_EPINTFLAG(epno));
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPSTALL1), flags);

      /* If EP is not halted, clear STALL */

      if (privep->epstate != USB_EPSTATE_STALLED)
        {
          sam_putreg8(USBDEV_EPSTATUS_STALLRQ1,
                      SAM_USBDEV_EPSTATUSCLR(epno));
        }
    }

  /* Transmit FAIL! */

  if ((flags & USBDEV_EPINT_TRFAIL0) != 0)
    {
      sam_putreg8(USBDEV_EPINT_TRFAIL0, SAM_USBDEV_EPINTFLAG(epno));
      privep->descb[0]->stausbk &= ~USBDEV_STATUSBK_ERRORFLOW;
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPTRFAIL0), flags);
    }
  if ((flags & USBDEV_EPINT_TRFAIL1) != 0)
    {
      sam_putreg8(USBDEV_EPINT_TRFAIL1, SAM_USBDEV_EPINTFLAG(epno));
      privep->descb[1]->stausbk &= ~USBDEV_STATUSBK_ERRORFLOW;
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPTRFAIL1), flags);
    }

  /* SETUP packet received */

  if ((flags & USBDEV_EPINT_RXSTP) != 0)
    {
      uint16_t len;

      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPRXSTP), flags);

      /* If a write request transfer was pending, complete it. */

      if (privep->epstate == USB_EPSTATE_SENDING)
        {
          sam_req_complete(privep, -EPROTO);
        }

      /* SETUP data is ready in the ep0out buffer. */

      memcpy((uint8_t *)&priv->ctrl, (uint8_t *)&priv->ep0out[0],
             USB_SIZEOF_CTRLREQ);

      /* Check for a SETUP IN transaction with data. */

      len = GETUINT16(priv->ctrl.len);
      if (USB_REQ_ISOUT(priv->ctrl.type) && len > 0)
        {
          /* Yes.. then we have to wait for the OUT data phase to complete
           * before processing the SETUP command.
           */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPOUT),
                   priv->ctrl.req);

          privep->epstate = USB_EPSTATE_EP0DATAOUT;

          /* Clear the RXSTP indication. */

          sam_putreg8(USBDEV_EPINT_RXSTP, SAM_USBDEV_EPINTFLAG(epno));
        }
      else
        {
          /* This is an SETUP IN command (or a SETUP IN with no data). */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPIN), len);
          privep->epstate = USB_EPSTATE_IDLE;

          /* Clear the RXSTP indication. */

          sam_putreg8(USBDEV_EPINT_RXSTP, SAM_USBDEV_EPINTFLAG(epno));

          /* Handle the SETUP OUT command now */

          sam_ep0_setup(priv);
        }

      /* ready for next setup data */

      sam_ep0_ctrlread(priv);
    }
}

/****************************************************************************
 * Name: sam_usb_interrupt
 *
 * Description:
 *   Handle the USB interrupt.
 *   Device Mode only TODO: Host
 *
 ****************************************************************************/

static int sam_usb_interrupt(int irq, void *context, void *arg)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)arg;
  uint16_t isr;
  uint16_t pending;
  uint16_t regval;
  uint16_t pendingep;
  int i;

  /* Get the set of pending device interrupts */

  isr     = sam_getreg16(SAM_USBDEV_INTFLAG);
  regval  = sam_getreg16(SAM_USBDEV_INTENSET);
  pending = isr & regval;

  /* Get the set of pending endpoint interrupts */

  pendingep = sam_getreg16(SAM_USBDEV_EPINTSMRY);

  /* Handle all pending USB interrupts */

  /* Serve Endpoint Interrupts first */

  if (pendingep)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_PENDING_EP), pendingep);

      for (i = 0; i < SAM_USB_NENDPOINTS; i++)
        {
          if ((pendingep & USBDEV_EPINTSMRY_EPINT(i)))
            {
              usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPNO), (uint16_t)i);
              sam_ep_interrupt(priv, i);
            }
        }
    }

  /* Suspend, treated last */

  if (pending == USBDEV_INT_SUSPEND)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SUSPEND), pending);

      /* Enable wakeup interrupts */

      sam_putreg16(USBDEV_INT_SUSPEND, SAM_USBDEV_INTENCLR);
      sam_putreg16(USBDEV_INT_UPRSM | USBDEV_INT_WAKEUP | USBDEV_INT_EORSM,
                  SAM_USBDEV_INTENSET);

      /* Clear the pending suspend (and any wakeup) interrupts */

      sam_putreg16(USBDEV_INT_SUSPEND | USBDEV_INT_WAKEUP,
                   SAM_USBDEV_INTFLAG);

      /* Perform board-specific suspend operations.
       *
       * The USB device peripheral clocks can be switched off.
       * Resume event is asynchronously detected. MCK and USBCK can be
       * switched off in the Power Management controller and
       * Other board-specific operations could also be performed.
       */
    }

  /* SOF interrupt */

  else if ((pending & USBDEV_INT_SOF) != 0)
    {
      /* Clear the pending SOF interrupt */

      sam_putreg16(SAM_TRACEINTID_SOF, SAM_USBDEV_INTFLAG);

      /* TODO: do we need check frame errors FNUM.FNCERR */
    }

  /* Resume or wakeup.  REVISIT:  Treat the same? */

  else if ((pending & (USBDEV_INT_WAKEUP | USBDEV_INT_EORSM)) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_WAKEUP), (uint16_t)pending);
      sam_resume(priv);

      /* Clear the pending wakeup, resume, (and any suspend) interrupts */

      sam_putreg16(USBDEV_INT_WAKEUP | USBDEV_INT_EORSM |
                   USBDEV_INT_SUSPEND, SAM_USBDEV_INTFLAG);

      /* Disable wakeup and endofresume Enable suspend interrupt */

      sam_putreg16(USBDEV_INT_WAKEUP | USBDEV_INT_EORSM,
                   SAM_USBDEV_INTENCLR);
      sam_putreg16(USBDEV_INT_SUSPEND, SAM_USBDEV_INTENSET);
    }

  /* End of Reset. Set by hardware when an End Of Reset has been
   * detected by the USB controller.
   */

  if ((pending & USBDEV_INT_EORST) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EORST), pending);

      /* Clear the end-of-reset interrupt */

      sam_putreg16(USBDEV_INT_EORST, SAM_USBDEV_INTFLAG);

      /* Handle the reset */

      sam_reset(priv);

      /* REVISIT: Set the device speed Why here ?? */

      priv->usbdev.speed = USB_SPEED_FULL;
    }

#if 0
  /* for DEBUG help: check for pending unhandled irq's */

  isr = sam_getreg16(SAM_USBDEV_INTFLAG);
  if (isr)
    {
      uwarn("WARNING: Unhandled:0x%X\n", isr);
    }

  pendingep = sam_getreg16(SAM_USBDEV_EPINTSMRY);
  if (pendingep)
    {
      uwarn("WARNING: Unhandled_EP:0x%X\n", pendingep);
    }
#endif

  return OK;
}

void arm_usbuninitialize(void)
{
  uinfo("arm_usbuninitialize()\n");
}

void arm_usbinitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_usbd;

  usbtrace(TRACE_DEVINIT, 0);

  /* Software initialization */

  sam_sw_setup(priv);

  /* Power up and initialize USB controller.  Interrupt from the USB
   * controller is initialized here, but will not be enabled at the AIC
   * until the class driver is installed.
   */

  sam_hw_setup(priv);

  /* Attach USB controller interrupt handlers.  The hardware will not be
   * initialized and interrupts will not be enabled until the class device
   * driver is bound.  Getting the IRQs here only makes sure that we have
   * them when we need them later.
   */

  if (irq_attach(SAM_IRQ_USB, sam_usb_interrupt, priv) != 0)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
               (uint16_t)SAM_IRQ_USB);
      goto errout;
    }

  uinfo("OK\n");
  return;

errout:
  arm_usbuninitialize();
}

/****************************************************************************
 * Name: sam_ep_reset
 *
 * Description:
 *   Reset and disable one endpoints.
 *
 ****************************************************************************/

static void sam_ep_reset(struct sam_usbdev_s *priv, uint8_t epno)
{
  struct sam_ep_s *privep = &priv->eplist[epno];

  /* Disable endpoint interrupts */

  sam_putreg8(0x7e, SAM_USBDEV_EPINTENCLR(epno));
  sam_putreg8(0x7e, SAM_USBDEV_EPINTFLAG(epno));

  /* Cancel any queued requests.  Since they are cancelled with status
   * -ESHUTDOWN, then will not be requeued until the configuration is reset.
   * NOTE:  This should not be necessary... the CLASS_DISCONNECT above
   * should result in the class implementation calling sam_ep_disable
   * for each of its configured endpoints.
   */

  sam_req_cancel(privep, -ESHUTDOWN);

  /* Reset endpoint status */

  privep->epstate   = USB_EPSTATE_DISABLED;
  privep->stalled   = false;
  privep->pending   = false;
  privep->halted    = false;
  privep->zlpsent   = false;
  privep->txbusy    = false;
  privep->rxactive  = false;
}

/****************************************************************************
 * Name: sam_epset_reset
 *
 * Description:
 *   Reset and disable a set of endpoints.
 *
 ****************************************************************************/

static void sam_epset_reset(struct sam_usbdev_s *priv, uint16_t epset)
{
  uint32_t bit;
  int epno;

  /* Reset each endpoint in the set */

  for (epno = 0, bit = 1, epset &= SAM_EPSET_ALL;
       epno < SAM_USB_NENDPOINTS && epset != 0;
       epno++, bit <<= 1)
    {
      /* Is this endpoint in the set? */

      if ((epset & bit) != 0)
        {
           /* Yes.. reset and disable it */

           sam_ep_reset(priv, epno);
           epset &= ~bit;
        }
    }
}

/****************************************************************************
 * Name: sam_ep_stall
 ****************************************************************************/

static int sam_ep_stall(struct sam_ep_s *privep)
{
  irqstate_t flags;
  uint8_t epno;

  /* Check that endpoint is in Idle state */

  DEBUGASSERT(/* privep->epstate == UDP_EPSTATE_IDLE && */ privep->dev);

  /* Check that endpoint is enabled and not already in Halt state */

  flags = enter_critical_section();
  if ((privep->epstate != USB_EPSTATE_DISABLED) &&
      (privep->epstate != USB_EPSTATE_STALLED))
    {
      epno = USB_EPNO(privep->ep.eplog);
      usbtrace(TRACE_EPSTALL, epno);

      /* If this is an IN endpoint (or endpoint 0), then cancel any
       * write requests in progress.
       */

      if (epno == 0 || USB_ISEPIN(privep->ep.eplog))
        {
          sam_req_cancel(privep, -EPERM);
        }

      /* Put endpoint into stalled state */

      privep->epstate = USB_EPSTATE_STALLED;
      privep->stalled = true;
      privep->pending = false;

      sam_putreg8(USBDEV_EPSTATUS_STALLRQ0 | USBDEV_EPSTATUS_STALLRQ1,
                  SAM_USBDEV_EPSTATUSSET(epno));
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_ep_resume
 ****************************************************************************/

static int sam_ep_resume(struct sam_ep_s *privep)
{
  struct sam_usbdev_s *priv;
  struct sam_req_s *req;
  irqstate_t flags;
  uint8_t epno;

  /* Check that endpoint is in Idle state */

  DEBUGASSERT(/* privep->epstate == UDP_EPSTATE_IDLE && */ privep->dev);

  flags = enter_critical_section();

  /* Check if the endpoint is stalled */

  if (privep->epstate == USB_EPSTATE_STALLED)
    {
      epno = USB_EPNO(privep->ep.eplog);
      usbtrace(TRACE_EPRESUME, epno);

      priv = (struct sam_usbdev_s *)privep->dev;

      /* Return endpoint to Idle state */

      privep->stalled = false;
      privep->pending = false;
      privep->epstate = USB_EPSTATE_IDLE;

      /* Clear STALLRQx request and reset data toggle */

      if (USB_ISEPIN(privep->ep.eplog))
        {
          sam_putreg8(USBDEV_EPSTATUS_STALLRQ1,
                      SAM_USBDEV_EPSTATUSCLR(epno));
          sam_putreg8(USBDEV_EPSTATUS_DTGLIN,
                      SAM_USBDEV_EPSTATUSCLR(epno));
          sam_putreg8(USBDEV_EPINT_STALL1,
                      SAM_USBDEV_EPINTFLAG(epno));
        }
      else
        {
          sam_putreg8(USBDEV_EPSTATUS_STALLRQ0,
                      SAM_USBDEV_EPSTATUSCLR(epno));
          sam_putreg8(USBDEV_EPSTATUS_DTGLOUT,
                      SAM_USBDEV_EPSTATUSCLR(epno));
          sam_putreg8(USBDEV_EPINT_STALL0,
                      SAM_USBDEV_EPINTFLAG(epno));
        }

      /* Copy any requests in the pending request queue to the working
       * request queue.
       */

      while ((req = sam_req_dequeue(&privep->pendq)) != NULL)
        {
          sam_req_enqueue(&privep->reqq, req);
        }

      /* Resuming any blocked data transfers on the endpoint */

      if (epno == 0 || USB_ISEPIN(privep->ep.eplog))
        {
          /* IN endpoint (or EP0).  Restart any queued write requests */

          sam_req_write(priv, privep);
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_pullup
 *
 * Description:
 *   This is the pullup() method of the USB device driver interface
 *
 ****************************************************************************/

static int sam_pullup(struct usbdev_s *dev, bool enable)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;
  uint32_t regval;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  /* Enable/disable the USB pull-up resistor */

  regval = sam_getreg8(SAM_USBDEV_CTRLB);

  if (enable)
    {
      /* Connect the 1.5 KOhm integrated pull-up on USB */

      regval &= ~USBDEV_CTRLB_DETACH;
    }
  else
    {
      /* Disconnect the 1.5 KOhm integrated pull-up on USB */

      regval |= USBDEV_CTRLB_DETACH;

      /* Device returns to the Powered state */

      if (priv->devstate > USB_DEVSTATE_POWERED)
        {
          priv->devstate = USB_DEVSTATE_POWERED;
        }
    }

  sam_putreg8((uint8_t)regval, SAM_USBDEV_CTRLB);

  return OK;
}

/****************************************************************************
 * Name: sam_enableclks
 ****************************************************************************/

static void sam_enableclks(void)
{
  sam_usb_enableperiph();

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)
  uint16_t regval;

  /* Enable GCLK specified by BOARD_USB_GCLKGEN */

  regval = GCLK_CLKCTRL_ID_USB;
  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* Wait for clock to become disabled */

  while ((getreg16(SAM_GCLK_CLKCTRL) & GCLK_CLKCTRL_CLKEN) != 0);

  /* Select the USB source clock generator */

  regval |= (uint16_t)BOARD_USB_GCLKGEN << GCLK_CLKCTRL_GEN_SHIFT;

  /* Write the new configuration */

  putreg16(regval, SAM_GCLK_CLKCTRL);

  regval |= GCLK_CLKCTRL_CLKEN;
  putreg16(regval, SAM_GCLK_CLKCTRL);

  while ((getreg16(SAM_GCLK_CLKCTRL) & GCLK_CLKCTRL_CLKEN) == 0);

#elif defined(CONFIG_ARCH_FAMILY_SAML21)
  sam_gclk_chan_enable(GCLK_CHAN_USB, BOARD_USB_GCLKGEN);

#endif
}

/****************************************************************************
 * Name: sam_disableclks
 ****************************************************************************/

static void sam_disableclks(void)
{
#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)

#elif defined(CONFIG_ARCH_FAMILY_SAML21)
  sam_gclk_chan_disable(GCLK_CHAN_USB);
#endif
  sam_usb_disableperiph();
}

/****************************************************************************
 * Name: sam_hw_setup
 ****************************************************************************/

static void sam_hw_setup(struct sam_usbdev_s *priv)
{
  int i;
  uint16_t regval;
  uint32_t padcalib;

  uint8_t calib_transn;
  uint8_t calib_transp;
  uint8_t calib_trim;

  /* To use the USB, the programmer must first configure the USB clock
   * input,
   */

  sam_enableclks();

  /* full reset USB */

  sam_ctrla_write(USB_CTRLA_SWRST);

  /* Load USB factory calibration values from NVRAM */

  calib_transn = getreg32(SYSCTRL_FUSES_USBTRANSN_ADDR) &
                          SYSCTRL_FUSES_USBTRANSN_MASK >>
                          SYSCTRL_FUSES_USBTRANSN_SHIFT;

  calib_transp = getreg32(SYSCTRL_FUSES_USBTRANSP_ADDR) &
                          SYSCTRL_FUSES_USBTRANSP_MASK >>
                          SYSCTRL_FUSES_USBTRANSP_SHIFT;

  calib_trim = getreg32(SYSCTRL_FUSES_USBTRIM_ADDR) &
                        SYSCTRL_FUSES_USBTRIM_MASK >>
                        SYSCTRL_FUSES_USBTRIM_SHIFT;

  padcalib = USB_PADCAL_TRANSP(calib_transp) |
             USB_PADCAL_TRANSN(calib_transn) |
             USB_PADCAL_TRIM(calib_trim);

  sam_putreg32(padcalib, SAM_USB_PADCAL);

  /* set config
   * NREPLY = Any transaction to endpoint 0 will be ignored except SETUP
   *          cleared by hardware when receiving a SETUP packet.
   * DETACH = The internal device pull-ups are disabled
   */

  regval = USBDEV_CTRLB_NREPLY | USBDEV_CTRLB_DETACH;

  /* do we need config to set LOW_SPEED mode? */

  regval |= USBDEV_CTRLB_SPDCONF_FULL;

  sam_putreg16(regval, SAM_USBDEV_CTRLB);

  /* Enable USB core */

#ifdef CONFIG_USBDEV
  sam_ctrla_write(USB_CTRLA_ENABLE | USB_CTRLA_MODE_DEVICE);
#endif
#ifdef CONFIG_USBHOST
  sam_ctrla_write(USB_CTRLA_ENABLE | USB_CTRLA_MODE_HOST);
#endif

  /* Set up the USB DP/DM pins */

  sam_configport(PORT_USB_DP);
  sam_configport(PORT_USB_DM);

  /* Reset and disable endpoints */

  sam_epset_reset(priv, SAM_EPSET_ALL);

  /* Initialize Endpoints */

  for (i = 0; i < SAM_USB_NENDPOINTS; i++)
    {
      /* Reset endpoint configuration */

      sam_putreg8(0, SAM_USBDEV_EPCFG(i));
    }

  /* Init descriptor base address */

  sam_putreg32((uint32_t)&priv->ep_descriptors, SAM_USB_DESCADD);

  /* clear all previous descriptor data so no accidental
   * DMA transfers could happen.
   */

  memset((uint8_t *)(&priv->ep_descriptors[0]), 0,
         sizeof(priv->ep_descriptors));

  /* Disable all interrupts */

  sam_putreg16(USBDEV_INT_SUSPEND | USBDEV_INT_SOF     | USBDEV_INT_EORST |
               USBDEV_INT_WAKEUP  | USBDEV_INT_EORSM   | USBDEV_INT_UPRSM |
               USBDEV_INT_RAMACER | USBDEV_INT_LPMNYET | USBDEV_INT_LPMSUSP,
               SAM_USBDEV_INTENCLR);

  sam_pullup(&priv->usbdev, false);
}

/****************************************************************************
 * Name: sam_hw_shutdown
 ****************************************************************************/

static void sam_hw_shutdown(struct sam_usbdev_s *priv)
{
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable all interrupts */

  sam_putreg16(USBDEV_INT_SUSPEND | USBDEV_INT_SOF     | USBDEV_INT_EORST |
               USBDEV_INT_WAKEUP  | USBDEV_INT_EORSM   | USBDEV_INT_UPRSM |
               USBDEV_INT_RAMACER | USBDEV_INT_LPMNYET | USBDEV_INT_LPMSUSP,
               SAM_USBDEV_INTENCLR);

  /* Clear all pending interrupt status */

  sam_putreg16(USBDEV_INT_SUSPEND | USBDEV_INT_SOF     | USBDEV_INT_EORST |
               USBDEV_INT_WAKEUP  | USBDEV_INT_EORSM   | USBDEV_INT_UPRSM |
               USBDEV_INT_RAMACER | USBDEV_INT_LPMNYET | USBDEV_INT_LPMSUSP,
               SAM_USBDEV_INTFLAG);

  /* Disconnect the device / disable the pull-up */

  sam_pullup(&priv->usbdev, false);

  /* Disable clocking to the UDP peripheral */

  sam_disableclks();
}

/****************************************************************************
 * Name: sam_sw_setup
 ****************************************************************************/

static void sam_sw_setup(struct sam_usbdev_s *priv)
{
  int epno;

  /* Initialize the device state structure.  NOTE: many fields
   * have the initial value of zero and, hence, are not explicitly
   * initialized here.
   */

  memset(priv, 0, sizeof(struct sam_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[EP0].ep;
  priv->epavail    = SAM_EPSET_ALL & ~SAM_EP_BIT(EP0);
  priv->devstate   = USB_DEVSTATE_SUSPENDED;
  priv->prevstate  = USB_DEVSTATE_POWERED;

  /* Initialize the endpoint list */

  for (epno = 0; epno < SAM_USB_NENDPOINTS; epno++)
    {
      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the (physical) endpoint number which is just the index to the
       * endpoint.
       */

      priv->eplist[epno].ep.ops    = &g_epops;
      priv->eplist[epno].dev       = priv;
      priv->eplist[epno].ep.eplog  = epno;

      /* We will use a maxpacket size for supported for each endpoint */

      priv->eplist[epno].ep.maxpacket = SAM_USB_MAXPACKETSIZE(epno);

      /* set descriptor addresses */

      priv->eplist[epno].descb[0] = &priv->ep_descriptors[(epno << 1)];
      priv->eplist[epno].descb[1] = &priv->ep_descriptors[(epno << 1) + 1];
    }

  /* Select a smaller endpoint size for EP0 */

#if SAM_EP0_MAXPACKET < 64 /* SAM_USB_MAXPACKETSIZE(0)? */
  priv->eplist[EP0].ep.maxpacket = SAM_EP0_MAXPACKET;
#endif
}

/****************************************************************************
 * Name: sam_sw_shutdown
 ****************************************************************************/

static void sam_sw_shutdown(struct sam_usbdev_s *priv)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_usbd;
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG_USB
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BINDFAILED), (uint16_t)-ret);
      priv->driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts at the NVIC. */

      sam_hw_setup(priv);
      up_enable_irq(SAM_IRQ_USB);

      /* Enable EORST irq */

      sam_putreg16(USBDEV_INT_EORST, SAM_USBDEV_INTENSET);

      /* Enable pull-up to connect the device.  The host should enumerate us
       * some time after this.  The next thing we expect is the EORST
       * interrupt.
       */

      sam_pullup(&priv->usbdev, true);
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
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_usbd;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG_USB
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = enter_critical_section();

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts (but keep them attached) */

  up_disable_irq(SAM_IRQ_USB);

  /* Put the hardware in an inactive state.  Then bring the hardware back up
   * in the initial state.  This is essentially the same state as we were
   * in when arm_usbinitialize() was first called.
   */

  sam_hw_shutdown(priv);
  sam_sw_shutdown(priv);

  sam_sw_setup(priv);
  sam_hw_setup(priv);

  /* Unhook the driver */

  priv->driver = NULL;
  leave_critical_section(flags);
  return OK;
}

#endif /* CONFIG_USBDEV && CONFIG_SAMD2L2_USB */
