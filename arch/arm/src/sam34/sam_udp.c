/****************************************************************************
 * arch/arm/src/sam34/sam_udp.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.orgr>
 *
 * This driver derives in a small part from the SAMA5D3 UDP driver:
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.orgr>
 *
 * Atmel sample code was used as a reference (only) in the SAMA5D3 driver
 * development.  The Atmel sample code has a BSD compatible license that
 * requires this copyright notice:
 *
 *   Copyright (c) 2009, Atmel Corporation
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
 * 3. Neither the name NuttX, Atmel, nor the names of its contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
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

#include <arch/irq.h>

#include "up_arch.h"
#include "up_internal.h"

#include "sam_periphclks.h"
#include "chip/sam_udp.h"
#include "sam_udp.h"

#if defined(CONFIG_USBDEV) && defined(CONFIG_SAM34_UDP)

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

#ifndef CONFIG_DEBUG
#  undef CONFIG_SAM34_UDP_REGDEBUG
#endif

/* Driver Definitions *******************************************************/
/* Initial interrupt mask: Reset + Suspend + Correct Transfer */

#define SAM_CNTR_SETUP      (USB_CNTR_RESETM|USB_CNTR_SUSPM|USB_CNTR_CTRM)

/* Endpoint definitions (Assuming 8 endpoints) */

#define EP0                 (0)
#define SAM_EPSET_ALL       (0xff)    /* All endpoints */
#define SAM_EPSET_NOTEP0    (0xfe)    /* All endpoints except EP0 */
#define SAM_EP_BIT(ep)      (1 << (ep))
#define SAM_EP0_MAXPACKET   (64)      /* EP0 Max. packet size */

/* Bitmap for all status bits in CSR that are not effected by a value 1 */

#define CSR_NOEFFECT_BITS   (UDPEP_CSR_RXDATABK0 | UDPEP_CSR_RXDATABK1 | \
                             UDPEP_CSR_STALLSENT | UDPEP_CSR_RXSETUP | \
                             UDPEP_CSR_TXCOMP)

#define nop()               __asm__ __volatile__ ( "nop" )

/* USB-related masks */

#define REQRECIPIENT_MASK   (USB_REQ_TYPE_MASK | USB_REQ_RECIPIENT_MASK)

/* Endpoint register masks (handling toggle fields) */

#define EPR_NOTOG_MASK      (USB_EPR_CTR_RX  | USB_EPR_SETUP  | USB_EPR_EPTYPE_MASK |\
                             USB_EPR_EP_KIND | USB_EPR_CTR_TX | USB_EPR_EA_MASK)
#define EPR_TXDTOG_MASK     (USB_EPR_STATTX_MASK | EPR_NOTOG_MASK)
#define EPR_RXDTOG_MASK     (USB_EPR_STATRX_MASK | EPR_NOTOG_MASK)

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

#define SAM_TRACEINTID_ADDRESSED          0x0001
#define SAM_TRACEINTID_CLEARFEATURE       0x0002
#define SAM_TRACEINTID_RXSUSP             0x0003
#define SAM_TRACEINTID_DEVGETSTATUS       0x0004
#define SAM_TRACEINTID_DISPATCH           0x0005
#define SAM_TRACEINTID_ENDBUSRES          0x0006
#define SAM_TRACEINTID_EP                 0x0007
#define SAM_TRACEINTID_EP0SETUPIN         0x0008
#define SAM_TRACEINTID_EP0SETUPOUT        0x0009
#define SAM_TRACEINTID_EP0SETUPSETADDRESS 0x000a
#define SAM_TRACEINTID_EPGETSTATUS        0x000b
#define SAM_TRACEINTID_EPINQEMPTY         0x000c
#define SAM_TRACEINTID_EPOUTQEMPTY        0x000d
#define SAM_TRACEINTID_GETCONFIG          0x000e
#define SAM_TRACEINTID_GETSETDESC         0x000f
#define SAM_TRACEINTID_GETSETIF           0x0010
#define SAM_TRACEINTID_GETSTATUS          0x0011
#define SAM_TRACEINTID_IFGETSTATUS        0x0012
#define SAM_TRACEINTID_INTERRUPT          0x0013
#define SAM_TRACEINTID_SOF                0x0014
#define SAM_TRACEINTID_NOSTDREQ           0x0015
#define SAM_TRACEINTID_PENDING            0x0016
#define SAM_TRACEINTID_RXDATABK0          0x0017
#define SAM_TRACEINTID_RXDATABK1          0x0018
#define SAM_TRACEINTID_RXSETUP            0x0019
#define SAM_TRACEINTID_SETCONFIG          0x001a
#define SAM_TRACEINTID_SETFEATURE         0x001b
#define SAM_TRACEINTID_STALLSNT           0x001c
#define SAM_TRACEINTID_SYNCHFRAME         0x001d
#define SAM_TRACEINTID_TXCOMP             0x001e
#define SAM_TRACEINTID_UPSTRRES           0x001f
#define SAM_TRACEINTID_WAKEUP             0x0020

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
  UDP_EPSTATE_DISABLED = 0, /* Endpoint is disabled */
  UDP_EPSTATE_STALLED,      /* Endpoint is stalled */
  UDP_EPSTATE_IDLE,         /* Endpoint is idle (i.e. ready for transmission) */
  UDP_EPSTATE_SENDING,      /* Endpoint is sending data */
  UDP_EPSTATE_RXSTOPPED,    /* OUT endpoint is stopped waiting for a read request */
                            /* --- Endpoint 0 Only --- */
  UDP_EPSTATE_EP0DATAOUT,   /* Endpoint 0 is receiving SETUP OUT data */
  UDP_EPSTATE_EP0STATUSIN,  /* Endpoint 0 is sending SETUP status */
  UDP_EPSTATE_EP0ADDRESS    /* Address change is pending completion of status */
};

/* The overall state of the device */

enum sam_devstate_e
{
  UDP_DEVSTATE_SUSPENDED = 0, /* The device is currently suspended */
  UDP_DEVSTATE_POWERED,       /* Host is providing +5V through the USB cable */
  UDP_DEVSTATE_DEFAULT,       /* Device has been reset */
  UDP_DEVSTATE_ADDRESSED,     /* The device has been given an address on the bus */
  UDP_DEVSTATE_CONFIGURED     /* A valid configuration has been selected. */
};

/* The result of EP0 SETUP processing */

enum sam_ep0setup_e
{
  UDP_EP0SETUP_SUCCESS = 0,   /* The SETUP was handle without incident */
  UDP_EP0SETUP_DISPATCHED,    /* The SETUP was forwarded to the class driver */
  UDP_EP0SETUP_ADDRESS,       /* A new device address is pending */
  UDP_EP0SETUP_STALL          /* An error occurred */
};

union wb_u
{
  uint16_t w;
  uint8_t  b[2];
};

/* A container for a request so that the request make be retained in a list */

struct sam_req_s
{
  struct usbdev_req_s  req;          /* Standard USB request */
  struct sam_req_s    *flink;        /* Supports a singly linked list */
  uint16_t             inflight;     /* Number of TX bytes written to FIFO */
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

  /* SAM34-specific fields */

  struct sam_usbdev_s *dev;          /* Reference to private driver data */
  struct sam_rqhead_s  reqq;         /* Read/write request queue */
  struct sam_rqhead_s  pendq;        /* Write requests pending stall sent */
  volatile uint8_t     epstate;      /* State of the endpoint (see enum sam_epstate_e) */
  uint8_t              stalled:1;    /* true: Endpoint is stalled */
  uint8_t              pending:1;    /* true: IN Endpoint stall is pending */
  uint8_t              halted:1;     /* true: Endpoint feature halted */
  uint8_t              zlpneeded:1;  /* Zero length packet needed at end of transfer */
  uint8_t              zlpsent:1;    /* Zero length packet has been sent */
  uint8_t              txbusy:1;     /* Write request queue is busy (recursion avoidance kludge) */
};

struct sam_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to structsam_usbdev_s.
   */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* UDP-specific fields */

  struct usb_ctrlreq_s     ctrl;          /* Last EP0 request */
  uint8_t                  devstate;      /* State of the device (see enum sam_devstate_e) */
  uint8_t                  prevstate;     /* Previous state of the device before SUSPEND */
  uint8_t                  devaddr;       /* Assigned device address */
  uint8_t                  selfpowered:1; /* 1: Device is self powered */
  uint16_t                 epavail;       /* Bitset of available endpoints */

  /* The endpoint list */

  struct sam_ep_s          eplist[SAM_UDP_NENDPOINTS];

  /* EP0 data buffer.  For data that is included in an EP0 SETUP OUT
   * transaction.  In this case, no request is in place from the class
   * driver and the incoming data is caught in this buffer.  The size
   * of valid dat in the buffer is given by ctrlreg.len[].  For the
   * case of EP0 SETUP IN transaction, the normal request mechanism is
   * used and the class driver provides the buffering.
   */

  uint8_t                  ep0out[SAM_EP0_MAXPACKET];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_SAM34_UDP_REGDEBUG
static void   sam_printreg(uintptr_t regaddr, uint32_t regval, bool iswrite);
static void   sam_checkreg(uintptr_t regaddr, uint32_t regval, bool iswrite);
static uint32_t sam_getreg(uintptr_t regaddr);
static void   sam_putreg(uint32_t regval, uintptr_t regaddr);
static void   sam_dumpep(struct sam_usbdev_s *priv, uint8_t epno);
#else
static inline uint32_t sam_getreg(uintptr_t regaddr);
static inline void sam_putreg(uint32_t regval, uintptr_t regaddr);
# define sam_dumpep(priv,epno)
#endif

static void   sam_csr_setbits(uint8_t epno, uint32_t setbits);
static void   sam_csr_clrbits(uint8_t epno, uint32_t clrbits);

/* Suspend/Resume Helpers ***************************************************/

static void   sam_suspend(struct sam_usbdev_s *priv);
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
                struct sam_ep_s *privep, uint16_t recvsize,
                int bank);
static void   sam_req_cancel(struct sam_ep_s *privep, int16_t status);

/* Interrupt level processing ***********************************************/

static void   sam_ep0_read(uint8_t *buffer, size_t buflen);
static void   sam_ep0_wrstatus(const uint8_t *buffer, size_t buflen);
static void   sam_ep0_dispatch(struct sam_usbdev_s *priv);
static void   sam_setdevaddr(struct sam_usbdev_s *priv, uint8_t value);
static void   sam_ep0_setup(struct sam_usbdev_s *priv);
static void   sam_ep_bankinterrupt(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, uint32_t csr, int bank);
static void   sam_ep_interrupt(struct sam_usbdev_s *priv, int epno);
static int    sam_udp_interrupt(int irq, void *context);

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
static inline bool
              sam_ep_reserved(struct sam_usbdev_s *priv, int epno);
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
#ifdef CONFIG_USBDEV_DMA
static void  *sam_ep_allocbuffer(struct usbdev_ep_s *ep, uint16_t nbytes);
static void   sam_ep_freebuffer(struct usbdev_ep_s *ep, void *buf);
#endif
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
static int    sam_pullup(FAR struct usbdev_s *dev,  bool enable);

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

static struct sam_usbdev_s g_udp;

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
  .mxpacketsize  = {64, 0},
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
  TRACE_STR(SAM_TRACEINTID_ADDRESSED),
  TRACE_STR(SAM_TRACEINTID_CLEARFEATURE),
  TRACE_STR(SAM_TRACEINTID_RXSUSP),
  TRACE_STR(SAM_TRACEINTID_DEVGETSTATUS),
  TRACE_STR(SAM_TRACEINTID_DISPATCH),
  TRACE_STR(SAM_TRACEINTID_ENDBUSRES),
  TRACE_STR(SAM_TRACEINTID_EP),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPIN),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPOUT),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPSETADDRESS),
  TRACE_STR(SAM_TRACEINTID_EPGETSTATUS),
  TRACE_STR(SAM_TRACEINTID_EPINQEMPTY),
  TRACE_STR(SAM_TRACEINTID_EPOUTQEMPTY),
  TRACE_STR(SAM_TRACEINTID_GETCONFIG),
  TRACE_STR(SAM_TRACEINTID_GETSETDESC),
  TRACE_STR(SAM_TRACEINTID_GETSETIF),
  TRACE_STR(SAM_TRACEINTID_GETSTATUS),
  TRACE_STR(SAM_TRACEINTID_IFGETSTATUS),
  TRACE_STR(SAM_TRACEINTID_INTERRUPT),
  TRACE_STR(SAM_TRACEINTID_SOF),
  TRACE_STR(SAM_TRACEINTID_NOSTDREQ),
  TRACE_STR(SAM_TRACEINTID_PENDING),
  TRACE_STR(SAM_TRACEINTID_RXDATABK0),
  TRACE_STR(SAM_TRACEINTID_RXDATABK1),
  TRACE_STR(SAM_TRACEINTID_RXSETUP),
  TRACE_STR(SAM_TRACEINTID_SETCONFIG),
  TRACE_STR(SAM_TRACEINTID_SETFEATURE),
  TRACE_STR(SAM_TRACEINTID_STALLSNT),
  TRACE_STR(SAM_TRACEINTID_SYNCHFRAME),
  TRACE_STR(SAM_TRACEINTID_TXCOMP),
  TRACE_STR(SAM_TRACEINTID_UPSTRRES),
  TRACE_STR(SAM_TRACEINTID_WAKEUP),
  TRACE_STR_END
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Private Functions
 ****************************************************************************/

/****************************************************************************
 * Register Operations
 ****************************************************************************/
/*******************************************************************************
 * Name: sam_printreg
 *
 * Description:
 *   Print the contents of a SAM34 UDP registers
 *
 *******************************************************************************/

#ifdef CONFIG_SAM34_UDP_REGDEBUG
static void sam_printreg(uintptr_t regaddr, uint32_t regval, bool iswrite)
{
  lldbg("%p%s%08x\n", regaddr, iswrite ? "<-" : "->", regval);
}
#endif

/*******************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if it is time to output debug information for accesses to a SAM34
 *   UDP registers
 *
 *******************************************************************************/

#ifdef CONFIG_SAM34_UDP_REGDEBUG
static void sam_checkreg(uintptr_t regaddr, uint32_t regval, bool iswrite)
{
  static uintptr_t prevaddr  = 0;
  static uint32_t  preval    = 0;
  static uint32_t  count     = 0;
  static bool      prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register last time?
   * Are we polling the register?  If so, suppress the output.
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

              lldbg("[repeats %d more times]\n", count);
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

/*******************************************************************************
 * Name: sam_getreg
 *
 * Description:
 *   Get the contents of an SAM34 register
 *
 *******************************************************************************/

#ifdef CONFIG_SAM34_UDP_REGDEBUG
static uint32_t sam_getreg(uintptr_t regaddr)
{
  /* Read the value from the register */

  uint32_t regval = getreg32(regaddr);

  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, false);
  return regval;
}
#else
static inline uint32_t sam_getreg(uintptr_t regaddr)
{
  return getreg32(regaddr);
}
#endif

/*******************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *   Set the contents of an SAM34 register to a value
 *
 *******************************************************************************/

#ifdef CONFIG_SAM34_UDP_REGDEBUG
static void sam_putreg(uint32_t regval, uintptr_t regaddr)
{
  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, true);

  /* Write the value */

  putreg32(regval, regaddr);
}
#else
static inline void sam_putreg(uint32_t regval, uint32_t regaddr)
{
  putreg32(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: sam_dumpep
 ****************************************************************************/

#if defined(CONFIG_SAM34_UDP_REGDEBUG) && defined(CONFIG_DEBUG)
static void sam_dumpep(struct sam_usbdev_s *priv, uint8_t epno)
{
  /* Global Registers */

  lldbg("Global Registers:\n");
  lldbg(" FRMNUM:    %08x\n", sam_getreg(SAM_UDP_FRMNUM));
  lldbg("GLBSTAT:    %08x\n", sam_getreg(SAM_UDP_GLBSTAT));
  lldbg("  FADDR:    %08x\n", sam_getreg(SAM_UDP_FADDR));
  lldbg("    IMR:    %08x\n", sam_getreg(SAM_UDP_IMR));
  lldbg("    ISR:    %08x\n", sam_getreg(SAM_UDP_ISR));
  lldbg("  RSTEP:    %08x\n", sam_getreg(SAM_UDP_RSTEP));
  lldbg("   TXVC:    %08x\n", sam_getreg(SAM_UDP_TXVC));
  lldbg(" CSR[%d]:    %08x\n", epno, sam_getreg(SAM_UDPEP_CSR(epno)));
}
#endif

/****************************************************************************
 * Request Helpers
 ****************************************************************************/
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

static void sam_req_enqueue(struct sam_rqhead_s *queue, struct sam_req_s *req)
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

  flags = irqsave();
  privreq = sam_req_dequeue(&privep->reqq);
  irqrestore(flags);

  if (privreq)
    {
      /* Save the result in the request structure */

      privreq->req.result = result;

      /* Callback to the request completion handler */

      privreq->flink = NULL;
      privreq->req.callback(&privep->ep, &privreq->req);

      /* Reset the endpoint state and restore the stalled indication */

      privep->epstate   = UDP_EPSTATE_IDLE;
      privep->zlpneeded = false;
      privep->zlpsent   = false;
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
  volatile uint32_t *fifo;
  uint8_t epno;
  int nbytes;

  /* Get the unadorned endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* Write access to the FIFO is not possible if TXDRY is set */

  DEBUGASSERT((sam_getreg(SAM_UDPEP_CSR(epno)) & UDPEP_CSR_TXPKTRDY) == 0);

  /* Get the number of bytes remaining to be sent. */

  DEBUGASSERT(privreq->req.xfrd < privreq->req.len);
  nbytes = privreq->req.len - privreq->req.xfrd;

  /* Either send the maxpacketsize or all of the remaining data in
   * the request.
   */

  if (nbytes >= privep->ep.maxpacket)
    {
      nbytes = privep->ep.maxpacket;
    }

  /* This is the new number of bytes "in-flight" */

  privreq->inflight = nbytes;
  usbtrace(TRACE_WRITE(USB_EPNO(privep->ep.eplog)), nbytes);

  /* The new buffer pointer is the start of the buffer plus the number of
   * bytes successfully transferred plus the number of bytes previously
   * "in-flight".
   */

  buf = privreq->req.buf + privreq->req.xfrd;

  /* Write packet in the FIFO buffer */

  fifo = (volatile uint32_t *)SAM_UDPEP_FDR(epno);
  for (; nbytes; nbytes--)
    {
      *fifo = (uint32_t)(*buf++);
    }

  /* Indicate that there we are in the sending state (even if this is a
   * zero-length packet) .  This indication will be need in interrupt
   * processing in order to properly terminate the request.
   */

  privep->epstate = UDP_EPSTATE_SENDING;

  /* Set TXPKTRDY to notify the USB hardware that there is TX data in the
   * endpoint FIFO.  We will be notified that the endpoint’s FIFO has been
   * released by the USB device when TXCOMP in the endpoint’s UDPEP_CSRx
   * register has been set.
   */

  sam_csr_setbits(epno, UDPEP_CSR_TXPKTRDY);
}

/****************************************************************************
 * Name: sam_req_write
 *
 * Description:
 *   Process the next queued write request.  This function is called in one
 *   of three contexts:  (1) When the endpoint is IDLE and a new write request
 *   is submitted (with interrupts disabled), (2) from TXCOMP interrupt
 *   handling when the current FIFO Tx transfer completes, or (3) when resuming
 *   a stalled IN or control endpoint.
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

  DEBUGASSERT(privep->epstate == UDP_EPSTATE_IDLE);
  while (privep->epstate == UDP_EPSTATE_IDLE)
    {
      /* Check the request from the head of the endpoint request queue */

      privreq = sam_rqpeek(&privep->reqq);
      if (!privreq)
        {
          /* There is no TX transfer in progress and no new pending TX
           * requests to send.
           */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPINQEMPTY), 0);

          /* Was there a pending endpoint stall? */

          if (privep->pending)
            {
              /* Yes... stall the endpoint now */

              (void)sam_ep_stall(privep);
            }

          return -ENOENT;
        }

      ullvdbg("epno=%d req=%p: len=%d xfrd=%d inflight=%d zlpneeded=%d\n",
              epno, privreq, privreq->req.len, privreq->req.xfrd,
              privreq->inflight, privep->zlpneeded);

      /* Handle any bytes in flight. */

      privreq->req.xfrd += privreq->inflight;
      privreq->inflight  = 0;

      /* Get the number of bytes left to be sent in the packet */

      bytesleft = privreq->req.len - privreq->req.xfrd;
      if (bytesleft > 0)
        {
          /* If the size is exactly a full packet, then note if we need to
           * send a zero length packet next.
           */

          if (bytesleft == privep->ep.maxpacket &&
             (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0)
            {
              /* Next time we get here, bytesleft will be zero and zlpneeded
               * will be set.
               */

              privep->zlpneeded = true;
            }
          else
            {
              /* No zero packet is forthcoming (maybe later) */

              privep->zlpneeded = false;
            }

          /* Perform the write operation.  epstate will become SENDING. */

          sam_req_wrsetup(priv, privep, privreq);
        }

      /* No data to send... This can happen on one of two ways:
       * (1) The last packet sent was the final packet of a transfer.
       * If it was also exactly maxpacketsize and the protocol expects
       * a zero length packet to follow then privep->zlpneeded will be
       * set.  Or (2) we called with a request packet that has
       * len == 0 (privep->zlpneeded will not be set).  Either case
       * means that it is time to send a zero length packet and complete
       * this transfer.
       */

      else if ((privreq->req.len == 0 || privep->zlpneeded) && !privep->zlpsent)
        {
          /* If we get here, then we sent the last of the data on the
           * previous pass and we need to send the zero length packet now.
           *
           * A Zero Length Packet can be sent by setting just the TXPTKRDY flag
           * in the UDP_EPTSETSTAx register
           */

          privep->epstate   = UDP_EPSTATE_SENDING;
          privep->zlpneeded = false;
          privep->zlpsent   = true;
          privreq->inflight = 0;

          /* Set TXPKTRDY to notify the USB hardware that there is (null)
           * TX packet available.  We will be notified that the endpoint’s
           * FIFO has been released by the USB device when TXCOMP in the
           * endpoint’s UDPEP_CSRx register has been set.
           */

          usbtrace(TRACE_WRITE(epno), 0);
          sam_csr_setbits(epno, UDPEP_CSR_TXPKTRDY);
        }

      /* If all of the bytes were sent (including any final zero length
       * packet) then we are finished with the request buffer and we can
       * return the request buffer to the class driver.  The state will
       * remain IDLE only if nothing else was put in flight.
       *
       * Note that we will then loop to check to check the next queued
       * write request.
       */

      if (privep->epstate == UDP_EPSTATE_IDLE)
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
 *   Complete the last read request by transferring the data from the RX FIFO
 *   to the request buffer, return the completed read request to the class
 *   implementation, and try to start the next queued read request.
 *
 *   This function is called in one of two contexts:  The normal case is (1)
 *   from interrupt handling when the current RX FIFO transfer completes.
 *   But there is also a special case (2) when the OUT endpoint is stopped
 *   because there are no available read requests.
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
 *     waiting in the FIFO to be read.
 *
 *     bank indicates the bit in the CSR register that must be cleared
 *     after the data has been read from the RX FIFO
 *
 ****************************************************************************/

static int sam_req_read(struct sam_usbdev_s *priv, struct sam_ep_s *privep,
                        uint16_t recvsize, int bank)
{
  struct sam_req_s *privreq;
  volatile const uint32_t *fifo;
  uint8_t *dest;
  int remaining;
  int readlen;
  int epno;

  DEBUGASSERT(priv && privep && privep->epstate == UDP_EPSTATE_IDLE);

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

          /* Disable further interrupts from this endpoint.  The RXDATABK0/1
           * interrupt will pend until either another read request is received
           * from the class driver or until the endpoint is reset because of
           * no response.  Set a flag so that we know that we are in this
           * perverse state and can re-enable endpoint interrupts when the
           * next read request is received.
           */

          sam_putreg(UDP_INT_EP(epno), SAM_UDP_IDR);
          privep->epstate = UDP_EPSTATE_RXSTOPPED;
          return -ENOENT;
        }

      ullvdbg("EP%d: len=%d xfrd=%d\n",
              epno, privreq->req.len, privreq->req.xfrd);

      /* Ignore any attempt to receive a zero length packet */

      if (privreq->req.len == 0)
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EPOUTNULLPACKET), 0);
          sam_req_complete(privep, OK);
          privreq = NULL;
        }
    }
  while (privreq == NULL);

  usbtrace(TRACE_READ(USB_EPNO(privep->ep.eplog)), recvsize);

  /* Get the number of bytes that can be received.  This is the size
   * of the user-provided request buffer, minus the number of bytes
   * already transferred to the user-buffer.
   */

  remaining = privreq->req.len - privreq->req.xfrd;

  /* Read the smaller of the number of bytes available in FIFO and the
   * size remaining in the request buffer provided by the caller.
   */

  readlen  = MIN(remaining, recvsize);
  recvsize = 0;

  /* Get the source and destination transfer addresses */

  fifo = (volatile const uint32_t *)SAM_UDPEP_FDR(epno);
  dest = privreq->req.buf + privreq->req.xfrd;

  /* Update the total number of bytes transferred */

  privreq->req.xfrd += readlen;
  privreq->inflight  = 0;

  /* Retrieve packet from the endpoint FIFO */

  for (; readlen > 0; readlen--)
    {
      *dest++ = (uint8_t)(*fifo);
    }

  /* We get here when an RXDATABK0/1 interrupt occurs.  That interrupt
   * cannot be cleared until all of the data has been taken from the RX
   * FIFO.  But we can
   */

  sam_csr_clrbits(epno, bank ? UDPEP_CSR_RXDATABK1 : UDPEP_CSR_RXDATABK0);

  /* Complete the transfer immediately and give the data to the class
   * driver.  The idea is that we will let the receiving be in-charge of
   * re-assembling data fragments.
   */

  usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
  sam_req_complete(privep, OK);
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
 * Interrupt Level Processing
 ****************************************************************************/
/****************************************************************************
 * Name: sam_ep0_read
 *
 * Description:
 *   Read a general USB request from the UDP FIFO
 *
 ****************************************************************************/

static void sam_ep0_read(uint8_t *buffer, size_t buflen)
{
  volatile const uint32_t *fifo;

  usbtrace(TRACE_READ(EP0), buflen);

  /* Retrieve packet from the FIFO */

  fifo = (volatile const uint32_t *)SAM_UDPEP_FDR(EP0);
  for (; buflen > 0; buflen--)
    {
      *buffer++ = (uint8_t)*fifo;
    }
}

/****************************************************************************
 * Name: sam_ep0_wrstatus
 *
 * Description:
 *   Process the next queued write request.
 *
 ****************************************************************************/

static void sam_ep0_wrstatus(const uint8_t *buffer, size_t buflen)
{
  volatile uint32_t *fifo;

  /* Write packet in the FIFO buffer */

  fifo = (volatile uint32_t *)SAM_UDPEP_FDR(EP0);
  for (; buflen > 0; buflen--)
    {
      *fifo = (uint32_t)(*buffer++);
    }

  /* Set TXPKTRDY to notify the USB hardware that there is TX data in the
   * endpoint FIFO.  We will be notified that the endpoint’s FIFO has been
   * released by the USB device when TXCOMP in the endpoint’s UDPEP_CSRx
   * register has been set.
   */

  sam_csr_setbits(EP0, UDPEP_CSR_TXPKTRDY);
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
          (void)sam_ep_stall(&priv->eplist[EP0]);
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
  uint32_t regval;

  DEBUGASSERT(address <= 0x7f);
  if (address)
    {
      /* Enable the address */

      regval = UDP_FADDR(address) | UDP_FADDR_FEN;
      sam_putreg(regval, SAM_UDP_FADDR);

      /* Go to the addressed but not configured state */

      regval  = sam_getreg(SAM_UDP_GLBSTAT);
      regval |= UDP_GLBSTAT_FADDEN;
      regval &= ~UDP_GLBSTAT_CONFG;
      sam_putreg(regval, SAM_UDP_GLBSTAT);

      priv->devstate = UDP_DEVSTATE_ADDRESSED;
    }
  else
    {
      /* Set address to zero.  The FEN bit still must be set in order to
       * receive or send data packets from or to the host.
       */

      sam_putreg(UDP_FADDR_FEN, SAM_UDP_FADDR);

      /* Make sure that we are not in either the configured or addressed
       * states
       */

      regval  = sam_getreg(SAM_UDP_GLBSTAT);
      regval &= ~(UDP_GLBSTAT_FADDEN | UDP_GLBSTAT_CONFG);
      sam_putreg(regval, SAM_UDP_GLBSTAT);

      /* Revert to the un-addressed, default state */

      priv->devstate = UDP_DEVSTATE_DEFAULT;
    }
}

/****************************************************************************
 * Name: sam_ep0_setup
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
  ep0->epstate  = UDP_EPSTATE_IDLE;

  /* And extract the little-endian 16-bit values to host order */

  value.w = GETUINT16(priv->ctrl.value);
  index.w = GETUINT16(priv->ctrl.index);
  len.w   = GETUINT16(priv->ctrl.len);

  ullvdbg("SETUP: type=%02x req=%02x value=%04x index=%04x len=%04x\n",
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

  ep0result = UDP_EP0SETUP_SUCCESS;
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
            ep0result = UDP_EP0SETUP_STALL;
          }
        else
          {
            switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
               case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  epno = USB_EPNO(index.b[LSB]);
                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPGETSTATUS), epno);
                  if (epno >= SAM_UDP_NENDPOINTS)
                    {
                      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPGETSTATUS), epno);
                      ep0result = UDP_EP0SETUP_STALL;
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
                      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DEVGETSTATUS), 0);

                      /* Features:  Remote Wakeup=YES; selfpowered=? */

                      response.w      = 0;
                      response.b[LSB] = (priv->selfpowered << USB_FEATURE_SELFPOWERED) |
                                        (1 << USB_FEATURE_REMOTEWAKEUP);
                      nbytes          = 2; /* Response size: 2 bytes */
                    }
                  else
                    {
                      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADDEVGETSTATUS), 0);
                      ep0result = UDP_EP0SETUP_STALL;
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
                  ep0result = UDP_EP0SETUP_STALL;
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

        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_CLEARFEATURE), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* Let the class implementation handle all recipients (except for the
             * endpoint recipient)
             */

            sam_ep0_dispatch(priv);
            ep0result = UDP_EP0SETUP_DISPATCHED;
          }
        else
          {
            /* Endpoint recipient */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < SAM_UDP_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = false;

                ret = sam_ep_resume(privep);
                if (ret < 0)
                  {
                    ep0result = UDP_EP0SETUP_STALL;
                  }
              }
            else
              {
                usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADCLEARFEATURE), 0);
                ep0result = UDP_EP0SETUP_STALL;
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

        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SETFEATURE), priv->ctrl.type);
        if (((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE) &&
            value.w == USB_FEATURE_TESTMODE)
          {
            /* Special case recipient=device test mode */

            ullvdbg("test mode: %d\n", index.w);
          }
        else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* The class driver handles all recipients except recipient=endpoint */

            sam_ep0_dispatch(priv);
            ep0result = UDP_EP0SETUP_DISPATCHED;
          }
        else
          {
            /* Handler recipient=endpoint */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < SAM_UDP_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = true;

                ret = sam_ep_stall(privep);
                if (ret < 0)
                  {
                    ep0result = UDP_EP0SETUP_STALL;
                  }
              }
            else
              {
                usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETFEATURE), 0);
                ep0result = UDP_EP0SETUP_STALL;
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

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_DEVICE ||
            index.w != 0 || len.w != 0 || value.w > 127)
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETADDRESS), 0);
            ep0result = UDP_EP0SETUP_STALL;
          }
        else
          {
            /* Note that setting of the device address will be deferred.  A
             * zero-length packet will be sent and the device address will
             * be set when the zero-length packet transfer completes.
             */

            usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPSETADDRESS), value.w);
            priv->devaddr = value.w;
            ep0result     = UDP_EP0SETUP_ADDRESS;
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
        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETSETDESC), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
          {
            /* The request seems valid... let the class implementation handle it */

            sam_ep0_dispatch(priv);
            ep0result = UDP_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETSETDESC), 0);
            ep0result = UDP_EP0SETUP_STALL;
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
        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_GETCONFIG), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            value.w == 0 && index.w == 0 && len.w == 1)
          {
            /* The request seems valid... let the class implementation handle it */

            sam_ep0_dispatch(priv);
            ep0result = UDP_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETCONFIG), 0);
            ep0result = UDP_EP0SETUP_STALL;
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
        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SETCONFIG), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            index.w == 0 && len.w == 0)
          {
             /* The request seems valid... let the class implementation handle it.
              * If the class implementation accepts it new configuration, it will
              * call sam_ep_configure() to configure the endpoints.
              */

             sam_ep0_dispatch(priv);
            ep0result = UDP_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETCONFIG), 0);
            ep0result = UDP_EP0SETUP_STALL;
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
        ep0result = UDP_EP0SETUP_DISPATCHED;
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
        usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDCTRLREQ), priv->ctrl.req);
        ep0result = UDP_EP0SETUP_STALL;
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
   * 1a. ep0result == UDP_EP0SETUP_SUCCESS
   *
   *    The setup request was successfully handled above and a response
   *    packet must be sent (may be a zero length packet).
   *
   * 1b. ep0result == UDP_EP0SETUP_ADDRESS
   *
   *    A special case is the case where epstate=UDP_EPSTATE_EP0ADDRESS.
   *    This means that the above processing generated an additional state
   *    where we need to wait until we complete the status phase before
   *    applying the new device address.
   *
   * 2. ep0result == UDP_EP0SETUP_DISPATCHED;
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
   * 3. ep0result == UDP_EP0SETUP_STALL;
   *
   *    An error was detected in either the above logic or by the class
   *    implementation logic.
   */

  switch (ep0result)
    {
      case UDP_EP0SETUP_SUCCESS:
        {
          /* Send the response (might be a zero-length packet) */

          ep0->epstate = UDP_EPSTATE_EP0STATUSIN;
          sam_ep0_wrstatus(response.b, nbytes);
        }
        break;

      case UDP_EP0SETUP_ADDRESS:
        {
          /* Send the response (might be a zero-length packet) */

          ep0->epstate = UDP_EPSTATE_EP0ADDRESS;
          sam_ep0_wrstatus(response.b, nbytes);
        }
        break;

      case UDP_EP0SETUP_STALL:
        {
          /* Stall EP0 */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EP0SETUPSTALLED),
                   priv->ctrl.req);

          (void)sam_ep_stall(&priv->eplist[EP0]);
        }
        break;

      case UDP_EP0SETUP_DISPATCHED:
      default:
        break;
    }
}

/****************************************************************************
 * Name: sam_ep_bankinterrupt
 *
 * Description:
 *   OUT data has been received on either bank 0 or bank 1
 *
 ****************************************************************************/

static void sam_ep_bankinterrupt(struct sam_usbdev_s *priv,
                                 struct sam_ep_s *privep,
                                 uint32_t csr, int bank)
{
  uint32_t eptype;
  uint16_t pktsize;
  uint8_t epno;

  /* Get the endpoint type */

  eptype = csr & UDPEP_CSR_EPTYPE_MASK;
  epno   = USB_EPNO(privep->ep.eplog);

  /* Are we receiving data for a read request?  EP0 does not receive data
   * using read requests.
   */

  if (privep->epstate == UDP_EPSTATE_IDLE && epno != 0)
    {
      /* Yes, get the size of the packet that we just received */

      pktsize = (uint16_t)
        ((csr & UDPEP_CSR_RXBYTECNT_MASK) >> UDPEP_CSR_RXBYTECNT_SHIFT);

      /* And continue processing the read request.  sam_req_read will
       * clear the  RXDATABK1 interrupt once that data has been
       * transferred from the FIFO.
       */

      privep->epstate = UDP_EPSTATE_IDLE;
      (void)sam_req_read(priv, privep, pktsize, bank);
    }

  /* Did we just receive the data associated with an OUT SETUP command? */

  else if (privep->epstate == UDP_EPSTATE_EP0DATAOUT)
    {
      uint16_t len;

      DEBUGASSERT(epno == EP0 && bank == 0);

      /* Yes.. back to the IDLE state */

      privep->epstate = UDP_EPSTATE_IDLE;

      /* Get the size of the packet that we just received */

      pktsize = (uint16_t)
        ((csr & UDPEP_CSR_RXBYTECNT_MASK) >> UDPEP_CSR_RXBYTECNT_SHIFT);

      /* Get the size that we expected to receive */

      len = GETUINT16(priv->ctrl.len);
      if (len == pktsize)
        {
          /* Copy the OUT data from the EP0 FIFO into a special EP0 buffer. */

          sam_ep0_read(priv->ep0out, len);

          /* Clear the RX Data Bank 0 interrupt (should not be bank 1!). */

          sam_csr_clrbits(EP0, UDPEP_CSR_RXDATABK0);

          /* And handle the EP0 SETUP now. */

          sam_ep0_setup(priv);
        }
      else
        {
          /* Clear the RX Data Bank 0 interrupt (should not be bank 1!).
           * Then stall.
           */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EP0SETUPOUTSIZE), pktsize);
          sam_csr_clrbits(EP0, UDPEP_CSR_RXDATABK0);
          (void)sam_ep_stall(privep);
        }
    }

  /* Check for a EP0 STATUS packet returned by the host at the end of a
   * SETUP status phase
   */

  else if (eptype == UDPEP_CSR_EPTYPE_CTRL &&
          (csr & UDPEP_CSR_RXBYTECNT_MASK) == 0)
    {
      DEBUGASSERT(epno == EP0 && bank == 0);

      /* Clear the RX Data Bank 0 interrupt */

      sam_csr_clrbits(EP0, UDPEP_CSR_RXDATABK0);
    }

  /* Otherwise there is a problem.  Complain an clear the interrupt */

  else
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_RXDATABKERR), privep->epstate);
      sam_csr_clrbits(epno, bank ? UDPEP_CSR_RXDATABK1 : UDPEP_CSR_RXDATABK0);
    }
}

/****************************************************************************
 * Name: sam_ep_interrupt
 *
 * Description:
 *   Handle the UDP endpoint interrupt
 *
 ****************************************************************************/

static void sam_ep_interrupt(struct sam_usbdev_s *priv, int epno)
{
  struct sam_ep_s *privep;
  uintptr_t regaddr;
  uint32_t csr;

  DEBUGASSERT((unsigned)epno < SAM_UDP_NENDPOINTS);

  /* Get the endpoint structure */

  privep = &priv->eplist[epno];

  /* Get the endpoint status */

  regaddr = SAM_UDPEP_CSR(epno);
  csr = sam_getreg(regaddr);

  /* TXCOMP: IN packet sent and acknowledged by the host */

  if ((csr & UDPEP_CSR_TXCOMP) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_TXCOMP), (uint16_t)csr);

      /* Clear the TXCOMP interrupt */

      sam_csr_clrbits(epno, UDPEP_CSR_TXCOMP);

      /* Sending state.  This is the completion of a "normal" write request
       * transfer.  In this case, we need to resume request processing in
       * order to send the next outgoing packet.
       */

      if (privep->epstate == UDP_EPSTATE_SENDING ||
          privep->epstate == UDP_EPSTATE_EP0STATUSIN)
        {
          /* Continue/resume processing the write requests */

          privep->epstate = UDP_EPSTATE_IDLE;
          (void)sam_req_write(priv, privep);
        }

      /* Setting of the device address is a special case.  The address was
       * obtained when a preceding SETADDRESS SETUP command was processed.
       * But the address is not set until the final SETUP status phase
       * completes.  This interrupt indicates the completion of that status
       * phase and now we set the address.
       */

      else if (privep->epstate == UDP_EPSTATE_EP0ADDRESS)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_ADDRESSED), priv->devaddr);
          DEBUGASSERT(epno == EP0);

          /* Set the device address */

          privep->epstate = UDP_EPSTATE_IDLE;
          sam_setdevaddr(priv, priv->devaddr);
        }
      else
        {
          /* Unexpected TXCOMP interrupt */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_TXCOMPERR), privep->epstate);
        }
    }

  /* OUT packet received in data bank 0 */

  if ((csr & UDPEP_CSR_RXDATABK0) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_RXDATABK0), (uint16_t)csr);

      /* Handle data received on Bank 0.  sam_ep_bankinterrupt will
       * clear the  RXDATABK0 interrupt once that data has been
       * transferred from the FIFO.
       */

      sam_ep_bankinterrupt(priv, privep, csr, 0);
    }

  /* OUT packet received in data bank 1 */

  else if ((csr & UDPEP_CSR_RXDATABK1) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_RXDATABK1), (uint16_t)csr);
      DEBUGASSERT(SAM_UDP_NBANKS(epno) > 1);

      /* Handle data received on Bank 1.  sam_ep_bankinterrupt will
       * clear the  RXDATABK1 interrupt once that data has been
       * transferred from the FIFO.
       */

      sam_ep_bankinterrupt(priv, privep, csr, 1);
    }

  /* STALL sent */

  if ((csr & UDPEP_CSR_STALLSENT) != 0)
    {
#ifdef CONFIG_USBDEV_ISOCHRONOUS
      uint32_t eptype;
#endif

      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_STALLSNT), (uint16_t)csr);

      /* Clear the STALLSENT interrupt */

      sam_csr_clrbits(epno, UDPEP_CSR_STALLSENT);

#ifdef CONFIG_USBDEV_ISOCHRONOUS
      /* Get the endpoint type */

      eptype = csr & UDPEP_CSR_EPTYPE_MASK;

     /* ISO error */

      if (eptype == UDPEP_CSR_EPTYPE_ISOIN || eptype == UDPEP_CSR_EPTYPE_ISOOUT)
        {
          privep->epstate = UDP_EPSTATE_IDLE;
          sam_req_complete(privep, -EIO);
        }
      else
#endif

      /* If EP is not halted, clear STALL */

      if (privep->epstate != UDP_EPSTATE_STALLED)
        {
          sam_csr_clrbits(epno, UDPEP_CSR_FORCESTALL);
        }
    }

  /* SETUP packet received */

  if ((csr & UDPEP_CSR_RXSETUP) != 0)
    {
      uint16_t len;

      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_RXSETUP), (uint16_t)csr);

      /* If a write request transfer was pending, complete it. */

      if (privep->epstate == UDP_EPSTATE_SENDING)
        {
          sam_req_complete(privep, -EPROTO);
        }

      /* Copy SETUP data from the EP0 FIFO into the driver structure. */

      sam_ep0_read((uint8_t *)&priv->ctrl, USB_SIZEOF_CTRLREQ);

      /* Check for a SETUP IN transaction with data. */

      len = GETUINT16(priv->ctrl.len);
      if (USB_REQ_ISOUT(priv->ctrl.type) && len > 0)
        {
          /* Yes.. then we have to wait for the OUT data phase to complete
           * before processing the SETUP command.
           */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPOUT), priv->ctrl.req);
          privep->epstate = UDP_EPSTATE_EP0DATAOUT;

          /* Clear the CSR:DIR bit to support the host-to-device data OUT
           * data transfer.  This bit must be cleared before CSR:RXSETUP is
           * cleared at the end of the SETUP stage.
           *
           * NOTE: Clearing this bit seems to be unnecessary.  I think it must
           * be cleared when RXSETUP is set.
           */

          sam_csr_clrbits(epno, UDPEP_CSR_DIR);

          /* Clear the RXSETUP indication. RXSETUP cannot be cleared before the
           * SETUP packet has been read in from the FIFO.  Otherwise, the USB
           * device would accept the next Data OUT transfer and overwrite the
           * SETUP packet in the FIFO.
           */

          sam_csr_clrbits(epno, UDPEP_CSR_RXSETUP);
        }
      else
        {
          /* This is an SETUP IN command (or a SETUP IN with no data). */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPIN), len);
          privep->epstate = UDP_EPSTATE_IDLE;

          /* Set the CSR:DIR bit to support the device-to-host data IN
           * data transfer.  This bit must be set before CSR:RXSETUP is
           * cleared at the end of the SETUP stage.
           */

          sam_csr_setbits(epno, UDPEP_CSR_DIR);

          /* Clear the RXSETUP indication. */

          sam_csr_clrbits(epno, UDPEP_CSR_RXSETUP);

          /* Handle the SETUP OUT command now */

          sam_ep0_setup(priv);
        }
    }
}

/****************************************************************************
 * Name: sam_udp_interrupt
 *
 * Description:
 *   Handle the UDP interrupt
 *
 ****************************************************************************/

static int sam_udp_interrupt(int irq, void *context)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple UDP controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_udp;
  uint32_t isr;
  uint32_t pending;
  uint32_t regval;
  int i;

  /* Get the set of pending interrupts */

  isr     = sam_getreg(SAM_UDP_ISR);
  usbtrace(TRACE_INTENTRY(SAM_TRACEINTID_INTERRUPT), isr);

  regval  = sam_getreg(SAM_UDP_IMR);
  pending = isr & regval;

  /* Handle all pending UDP interrupts (and new interrupts that become
   * pending)
   */

  while (pending)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_PENDING), (uint16_t)pending);

      /* Suspend, treated last */

      if (pending == UDP_INT_RXSUSP)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_RXSUSP),
                  (uint16_t)pending);

          /* Enable wakeup interrupts */

          sam_putreg(UDP_INT_RXSUSP, SAM_UDP_IDR);
          sam_putreg(UDP_INT_WAKEUP | UDP_INT_RXRSM, SAM_UDP_IER);

          /* Clear the pending suspend (and any wakeup) interrupts */

          sam_putreg(UDP_INT_RXSUSP | UDP_INT_WAKEUP, SAM_UDP_ICR);

          /* Perform board-specific suspend operations.  The USB device
           * peripheral clocks can be switched off. Resume event is
           * asynchronously detected. MCK and UDPCK can be switched off in
           * the Power Management controller and the USB transceiver can
           * be disabled by setting the TXVDIS field in the UDP_TXVC
           * register.  Other board-specific operations could also be
           * performed.
           */

          sam_suspend(priv);
        }

      /* SOF interrupt*/

      else if ((pending & UDP_INT_SOF) != 0)
        {
          /* Clear the pending SOF interrupt */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SOF),
                  (uint16_t)pending);
          sam_putreg(UDP_INT_SOF, SAM_UDP_ICR);
        }

      /* Resume or wakeup.  REVISIT:  Treat the same? */

      else if ((pending & (UDP_INT_WAKEUP | UDP_INT_RXRSM)) != 0)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_WAKEUP),
                  (uint16_t)pending);
          sam_resume(priv);

          /* Clear the pending wakeup, resume, (and any suspend) interrupts */

          sam_putreg(UDP_INT_WAKEUP | UDP_INT_RXRSM | UDP_INT_RXSUSP,
                     SAM_UDP_ICR);

          /* Enable suspend interrupts */

          sam_putreg(UDP_INT_WAKEUP | UDP_INT_RXRSM, SAM_UDP_IDR);
          sam_putreg(UDP_INT_RXSUSP, SAM_UDP_IER);
        }

      /* End of Reset. Set by hardware when an End Of Reset has been
       * detected by the UDP controller. Automatically enabled after USB
       * reset.
       *
       * "After its connection to a USB host, the USB device waits for an
       *  end-of-bus reset. The unmaskable flag ENDBUSRES is set in the
       *  register UDP_ISR and an interrupt is triggered.  Once the
       *  ENDBUSRES interrupt has been triggered, the device enters Default
       *  State. In this state, the UDP software must:
       *
       *  - "Enable the default endpoint, setting the EPEDS flag in the
       *     UDPEP_CSR[0] register and, optionally, enabling the interrupt
       *     for endpoint 0 by writing 1 to the UDP_IER register. The
       *     enumeration then begins by a control transfer.
       *  - "Configure the interrupt mask register which has been reset by
       *     the USB reset detection
       *  - "Enable the transceiver clearing the TXVDIS flag in the UDP_TXVC
       *     register.
       *
       * In this state UDPCK and MCK must be enabled.
       *
       * Warning: Each time an ENDBUSRES interrupt is triggered, the Interrupt
       * Mask Register and UDPEP_CSR registers have been reset.
       */

      if ((pending & UDP_ISR_ENDBUSRES) != 0)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_ENDBUSRES),
                  (uint16_t)pending);

          /* Clear the end-of-reset interrupt */

          sam_putreg(UDP_ISR_ENDBUSRES, SAM_UDP_ICR);

          /* Handle the reset */

          sam_reset(priv);

          /* Set the device speed */

          priv->usbdev.speed = USB_SPEED_FULL;
        }

      /* Endpoint Interrupts */

      else if ((pending & UDP_INT_EP_MASK) != 0)
        {
          for (i = 0; i < SAM_UDP_NENDPOINTS; i++)
            {
              if ((pending & UDP_INT_EP(i)) != 0)
                {
                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP), (uint16_t)i);
                  sam_ep_interrupt(priv, i);
                }
            }
        }

      /* Re-sample the set of pending interrupts */

      isr     = sam_getreg(SAM_UDP_ISR);
      regval  = sam_getreg(SAM_UDP_IMR);
      pending = isr & regval;
    }

  usbtrace(TRACE_INTEXIT(SAM_TRACEINTID_INTERRUPT), isr);
  return OK;
}

/****************************************************************************
 * Name: sam_suspend
 *
 * Description:
 *   Sets the specified bit(s) in the UDPEP_CSR register.
 *
 ****************************************************************************/

static void sam_csr_setbits(uint8_t epno, uint32_t setbits)
{
  uintptr_t regaddr;
  uint32_t regval;
  int count;

  /* Set the specified bits */

  regaddr = SAM_UDPEP_CSR(epno);
  regval  = sam_getreg(regaddr);
  regval |= CSR_NOEFFECT_BITS;
  regval |= setbits;
  sam_putreg(regval, regaddr);

  /* Followed by 15 nops (plus loop overhead).  After any bit is changed in
   * the CSR, a wait of 1 UDPCK clock cycle and 1 peripheral clock cycle is
   * required. However, RX_DATA_BK0, TXPKTRDY, RX_DATA_BK1 require wait
   * times of 3 UDPCK clock cycles and 5 peripheral clock cycles before
   * accessing DPR.
   */

  for (count = 0; count < 15; count++ )
    {
      nop();
    }
}

/****************************************************************************
 * Name: sam_suspend
 *
 * Description:
 *   Clears the specified bit(s) in the UDPEP_CSR register.
 *
 ****************************************************************************/

static void sam_csr_clrbits(uint8_t epno, uint32_t clrbits)
{
  uintptr_t regaddr;
  uint32_t regval;
  int count;

  /* Clear the specified bits */

  regaddr = SAM_UDPEP_CSR(epno);
  regval  = sam_getreg(regaddr);
  regval |= CSR_NOEFFECT_BITS;
  regval &= ~clrbits;
  sam_putreg(regval, regaddr);

  /* Followed by 15 nops (plus loop overhead).  After any bit is changed in
   * the CSR, a wait of 1 UDPCK clock cycle and 1 peripheral clock cycle is
   * required. However, RX_DATA_BK0, TXPKTRDY, RX_DATA_BK1 require wait
   * times of 3 UDPCK clock cycles and 5 peripheral clock cycles before
   * accessing DPR.
   */

  for (count = 0; count < 15; count++ )
    {
      nop();
    }
}

/****************************************************************************
 * Suspend/Resume Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: sam_suspend
 ****************************************************************************/

static void sam_suspend(struct sam_usbdev_s *priv)
{
  /* Don't do anything if the device is already suspended */

  if (priv->devstate != UDP_DEVSTATE_SUSPENDED)
    {
      /* Notify the class driver of the suspend event */

      if (priv->driver)
        {
          CLASS_SUSPEND(priv->driver, &priv->usbdev);
        }

      /* Switch to the Suspended state */

      priv->prevstate = priv->devstate;
      priv->devstate  = UDP_DEVSTATE_SUSPENDED;

      /* Disable clocking to the UDP peripheral */

      sam_disableclks();

      /* Let the board-specific logic know that we have entered the
       * suspend state.  This may trigger additional reduced power
       * consumption measures.
       */

      sam_udp_suspend((struct usbdev_s *)priv, false);
    }
}

/****************************************************************************
 * Name: sam_resume
 ****************************************************************************/

static void sam_resume(struct sam_usbdev_s *priv)
{
  /* This function is called when either (1) a WKUP interrupt is received from
   * the host PC, or (2) the class device implementation calls the wakeup()
   * method.
   */

  /* Don't do anything if the device was not suspended */

  if (priv->devstate == UDP_DEVSTATE_SUSPENDED)
    {
      /* Revert to the previous state */

      priv->devstate = priv->prevstate;

      /* Restore clocking to the UDP peripheral */

      sam_enableclks();

      /* Restore full power -- whatever that means for this particular board */

      sam_udp_suspend((struct usbdev_s *)priv, true);

      /* Notify the class driver of the resume event */

      if (priv->driver)
        {
          CLASS_RESUME(priv->driver, &priv->usbdev);
        }
    }
}

/****************************************************************************
 * Endpoint Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam_ep_reset
 *
 * Description
 *   Reset and disable one endpoints.
 *
 ****************************************************************************/

static void sam_ep_reset(struct sam_usbdev_s *priv, uint8_t epno)
{
  struct sam_ep_s *privep = &priv->eplist[epno];

  /* Disable endpoint interrupt */

  sam_putreg(UDP_INT_EP(epno), SAM_UDP_IDR);

  /* Cancel any queued requests.  Since they are cancelled with status
   * -ESHUTDOWN, then will not be requeued until the configuration is reset.
   * NOTE:  This should not be necessary... the CLASS_DISCONNECT above
   * should result in the class implementation calling sam_ep_disable
   * for each of its configured endpoints.
   */

  sam_req_cancel(privep, -ESHUTDOWN);

  /* Reset the endpoint FIFO */

  sam_putreg(UDP_RSTEP(epno), SAM_UDP_RSTEP);
  sam_putreg(0, SAM_UDP_RSTEP);

  /* Reset endpoint status */

  privep->epstate   = UDP_EPSTATE_DISABLED;
  privep->stalled   = false;
  privep->pending   = false;
  privep->halted    = false;
  privep->zlpneeded = false;
  privep->zlpsent   = false;
  privep->txbusy    = false;
}

/****************************************************************************
 * Name: sam_epset_reset
 *
 * Description
 *   Reset and disable a set of endpoints.
 *
 ****************************************************************************/

static void sam_epset_reset(struct sam_usbdev_s *priv, uint16_t epset)
{
  uint32_t bit;
  int epno;

  /* Reset each endpoint in the set */

  for (epno = 0, bit = 1, epset &= SAM_EPSET_ALL;
       epno < SAM_UDP_NENDPOINTS && epset != 0;
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

  flags = irqsave();
  if ((privep->epstate != UDP_EPSTATE_DISABLED) &&
      (privep->epstate != UDP_EPSTATE_STALLED))
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

      privep->epstate = UDP_EPSTATE_STALLED;
      privep->stalled = true;
      privep->pending = false;

      sam_csr_setbits(epno, UDPEP_CSR_FORCESTALL);
    }

  irqrestore(flags);
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

  flags = irqsave();

  /* Check if the endpoint is stalled */

  if (privep->epstate == UDP_EPSTATE_STALLED)
    {
      epno = USB_EPNO(privep->ep.eplog);
      usbtrace(TRACE_EPRESUME, epno);

      priv = (struct sam_usbdev_s *)privep->dev;

      /* Return endpoint to Idle state */

      privep->stalled = false;
      privep->pending = false;
      privep->epstate = UDP_EPSTATE_IDLE;

      /* Clear FORCESTALL request */

      sam_csr_clrbits(epno, UDPEP_CSR_FORCESTALL);

      /* Reset the endpoint FIFO */

      sam_putreg(UDP_RSTEP(epno), SAM_UDP_RSTEP);
      sam_putreg(0, SAM_UDP_RSTEP);

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

          (void)sam_req_write(priv, privep);
        }
    }

  irqrestore(flags);
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

  flags  = irqsave();
  epset &= priv->epavail;
  if (epset)
    {
      /* Select the lowest bit in the set of matching, available endpoints
       * (skipping EP0)
       */

      for (epndx = 1; epndx < SAM_UDP_NENDPOINTS; epndx++)
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

  irqrestore(flags);
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
  irqstate_t flags = irqsave();
  priv->epavail   |= SAM_EP_BIT(USB_EPNO(privep->ep.eplog));
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_ep_reserved
 *
 * Description:
 *   Check if the endpoint has already been allocated.
 *
 ****************************************************************************/

static inline bool
sam_ep_reserved(struct sam_usbdev_s *priv, int epno)
{
  return ((priv->epavail & SAM_EP_BIT(epno)) == 0);
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
  uintptr_t csr;
  uint32_t regval;
  uint16_t maxpacket;
  uint8_t epno;
  uint8_t eptype;
  bool dirin;

  DEBUGASSERT(privep && privep->dev && desc);

  uvdbg("len: %02x type: %02x addr: %02x attr: %02x "
        "maxpacketsize: %02x %02x interval: %02x\n",
        desc->len, desc->type, desc->addr, desc->attr,
        desc->mxpacketsize[0],  desc->mxpacketsize[1],
        desc->interval);

  /* Decode the endpoint descriptor */

  epno      = USB_EPNO(desc->addr);
  dirin     = (desc->addr & USB_DIR_MASK) == USB_REQ_DIR_IN;
  eptype    = (desc->attr & USB_EP_ATTR_XFERTYPE_MASK);
  maxpacket = GETUINT16(desc->mxpacketsize);

  DEBUGASSERT(maxpacket <= SAM_UDP_MAXPACKETSIZE(epno));

  /* Initialize the endpoint structure */

  privep->ep.eplog     = desc->addr;              /* Includes direction */
  privep->ep.maxpacket = maxpacket;
  privep->epstate      = UDP_EPSTATE_IDLE;

  /* Initialize the endpoint hardware */
  /* Disable the endpoint */

  csr = SAM_UDPEP_CSR(epno);
  sam_putreg(0, csr);

  /* Reset the endpoint FIFO */

  sam_putreg(UDP_RSTEP(epno), SAM_UDP_RSTEP);
  sam_putreg(0, SAM_UDP_RSTEP);

  /* Disable endpoint interrupts now */

  sam_putreg(UDP_INT_EP(epno), SAM_UDP_IDR);

  /* Configure and enable the endpoint */

  regval = UDPEP_CSR_EPEDS;
  switch (eptype)
    {
    case USB_EP_ATTR_XFER_CONTROL:
      if (!SAM_UDP_CONTROL(epno))
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_UNSUPPEPTYPE),
                   eptype >> USB_EP_ATTR_XFERTYPE_SHIFT);
          return -ENOSYS;
        }
      else
        {
          regval |= UDPEP_CSR_EPTYPE_CTRL;
        }
      break;

#ifdef CONFIG_USBDEV_ISOCHRONOUS
    case USB_EP_ATTR_XFER_ISOC:
      if (!SAM_UDP_ISOCHRONOUS(epno))
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_UNSUPPEPTYPE),
                   eptype >> USB_EP_ATTR_XFERTYPE_SHIFT);
          return -ENOSYS;
        }
      else if (dirin)
        {
          regval |= UDPEP_CSR_EPTYPE_ISOIN;
        }
      else
        {
          regval |= UDPEP_CSR_EPTYPE_ISOOUT;
        }
      break;
#endif

    case USB_EP_ATTR_XFER_BULK:
      if (!SAM_UDP_BULK(epno))
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_UNSUPPEPTYPE),
                   eptype >> USB_EP_ATTR_XFERTYPE_SHIFT);
          return -ENOSYS;
        }
      else if (dirin)
        {
          regval |= UDPEP_CSR_EPTYPE_BULKIN;
        }
      else
        {
          regval |= UDPEP_CSR_EPTYPE_BULKOUT;
        }
      break;

    case USB_EP_ATTR_XFER_INT:
      if (!SAM_UDP_INTERRUPT(epno))
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_UNSUPPEPTYPE),
                   eptype >> USB_EP_ATTR_XFERTYPE_SHIFT);
          return -ENOSYS;
        }
      else if (dirin)
        {
          regval |= UDPEP_CSR_EPTYPE_INTIN;
        }
      else
        {
          regval |= UDPEP_CSR_EPTYPE_INTOUT;
        }
      break;

    default:
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPTYPE),
               eptype >> USB_EP_ATTR_XFERTYPE_SHIFT);
      return -EINVAL;
    }

  sam_putreg(regval, csr);

  /* Enable endpoint interrupts */

  sam_putreg(UDP_INT_EP(epno), SAM_UDP_IER);
  sam_dumpep(privep->dev, epno);
  return OK;
}

/****************************************************************************
 * Endpoint operations
 ****************************************************************************/
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

#if defined(CONFIG_DEBUG) || defined(CONFIG_USBDEV_TRACE)
  uint8_t epno = USB_EPNO(desc->addr);
  usbtrace(TRACE_EPCONFIGURE, (uint16_t)epno);

  DEBUGASSERT(ep && desc && epno > 0 && epno < SAM_UDP_NENDPOINTS);
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
      uint32_t regval;

      /* Go to the configured state (we should have been in the addressed
       * state)
       */

      DEBUGASSERT(priv && priv->devstate == UDP_DEVSTATE_ADDRESSED);
      priv->devstate = UDP_DEVSTATE_CONFIGURED;

      regval  = sam_getreg(SAM_UDP_GLBSTAT);
      regval |= UDP_GLBSTAT_CONFG;
      sam_putreg(regval, SAM_UDP_GLBSTAT);
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

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      ulldbg("ERROR: ep=%p\n", ep);
      return -EINVAL;
    }
#endif

  epno = USB_EPNO(ep->eplog);
  usbtrace(TRACE_EPDISABLE, epno);

  /* Reset the endpoint and cancel any ongoing activity */

  flags = irqsave();
  priv  = privep->dev;
  sam_ep_reset(priv, epno);

  /* Revert to the addressed-but-not-configured state */

  sam_setdevaddr(priv, priv->devaddr);
  irqrestore(flags);
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

#ifdef CONFIG_DEBUG
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
  struct sam_req_s *privreq = (struct sam_req_s*)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif
  usbtrace(TRACE_EPFREEREQ, USB_EPNO(ep->eplog));

  kmm_free(privreq);
}

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

#ifdef CONFIG_DEBUG
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      ulldbg("ERROR: req=%p callback=%p buf=%p ep=%p\n", req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, USB_EPNO(ep->eplog));
  priv = privep->dev;

#ifdef CONFIG_DEBUG
  if (!priv->driver)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_NOTCONFIGURED), priv->usbdev.speed);
      ulldbg("ERROR: driver=%p\n", priv->driver);
      return -ESHUTDOWN;
    }
#endif

  /* Handle the request from the class driver */

  epno              = USB_EPNO(ep->eplog);
  req->result       = -EINPROGRESS;
  req->xfrd         = 0;
  privreq->inflight = 0;
  flags             = irqsave();

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

          ulldbg("Pending stall clear\n");
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

          if (privep->epstate == UDP_EPSTATE_IDLE && !privep->txbusy)
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
       * read requests.  If that is the case for this endpoint, then
       * re-enable endpoint interrupts now.
       */

      if (privep->epstate == UDP_EPSTATE_RXSTOPPED)
        {
          /* Un-stop the OUT endpoint be re-enabling endpoint interrupts.
           * There should be a pending RXDATABK0/1 interrupt or, if a long
           * time has elapsed since the endpoint was stopped, an ENDBUSRES
           * interrupt.
           */

          privep->epstate = UDP_EPSTATE_IDLE;
          sam_putreg(UDP_INT_EP(epno), SAM_UDP_IER);
        }
    }

  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Name: sam_ep_cancel
 ****************************************************************************/

static int sam_ep_cancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));

  flags = irqsave();
  sam_req_cancel(privep, -EAGAIN);
  irqrestore(flags);
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

#ifdef CONFIG_DEBUG
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

      flags = irqsave();
      epno = USB_EPNO(ep->eplog);
      if (epno != 0 && USB_ISEPIN(ep->eplog))
        {
          /* Are there any unfinished write requests in the request queue? */

          if (!sam_rqempty(&privep->reqq))
            {
              /* Just set a flag to indicate that the endpoint must be
               * stalled on the next TXCOMP interrupt when the request
               * queue becomes empty.
               */

              privep->pending = true;
              irqrestore(flags);
              return OK;
            }
        }

      /* Not an IN endpoint, endpoint 0, or no pending write requests.
       * Stall the endpoint now.
       */

      ret = sam_ep_stall(privep);
      irqrestore(flags);
    }

  return ret;
}

/****************************************************************************
 * Device Controller Operations
 ****************************************************************************/
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
#ifdef CONFIG_DEBUG
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
      /* Otherwise, we will return the endpoint structure only for the requested
       * 'logical' endpoint.  All of the other checks will still be performed.
       *
       * First, verify that the logical endpoint is in the range supported by
       * by the hardware.
       */

      if (epno >= SAM_UDP_NENDPOINTS)
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

#ifdef CONFIG_DEBUG
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

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Return the last frame number detected by the hardware */

  regval  = sam_getreg(SAM_UDP_FRMNUM);
  frameno = (regval & UDP_FRMNUM_MASK) >> UDP_FRMNUM_SHIFT;

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
  uint32_t regval;

  usbtrace(TRACE_DEVWAKEUP, 0);
#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Resume normal operation */

  flags = irqsave();
  sam_resume(priv);

  /* Activate a remote wakeup.  Setting the Enable Send Resume (ESR) bit
   * starts the Remote Wake Up procedure if this bit value was 0 and if
   * RMWUPE is enabled.
   *
   * "In Suspend state it is possible to wake up the host sending an external
   *  resume.
   *
   *  - "The device must wait at least 5 ms after being entered in suspend
   *     before sending an external resume.
   *  - "The device has 10 ms from the moment it starts to drain current and
   *     it forces a K state to resume the host.
   *  - "The device must force a K state from 1 to 15 ms to resume the host
   *
   * "Before sending a K state to the host, MCK, UDPCK and the transceiver
   *  must be enabled. Then to enable the remote wake-up feature, the RMWUPE bit
   *  in the UDP_GLB_STAT register must be enabled. To force the K state on the
   *  line, a transition of the ESR bit from 0 to 1 has to be done in the
   *  UDP_GLB_STAT register. This transition must be accomplished by first
   *  writing a 0 in the ESR bit and then writing a 1.
   *
   * " The K state is automatically generated and released according to the USB
   *   2.0 specification."
   */

  /* Make sure that the ESR bit is zero */

  regval  = sam_getreg(SAM_UDP_GLBSTAT);
  regval |= UDP_GLBSTAT_RMWUPE; /* Should already be set */
  regval &= ~UDP_GLBSTAT_ESR;
  sam_putreg(regval, SAM_UDP_GLBSTAT);

  /* Wait 5msec in case we just entered the resume state */

  usleep(5*1000);

  /* Set the ESR bit to send the remote resume */

  regval  = sam_getreg(SAM_UDP_GLBSTAT);
  regval |= UDP_GLBSTAT_ESR;
  sam_putreg(regval, SAM_UDP_GLBSTAT);
  irqrestore(flags);
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

#ifdef CONFIG_DEBUG
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
 * Name: sam_pullup
 *
 * Description:
 *   This is the pullup() method of the USB device driver interface
 *
 ****************************************************************************/

static int sam_pullup(FAR struct usbdev_s *dev, bool enable)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;
  uint32_t regval;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  /* Enable/disable the UDP pull-up resistor */

  regval = sam_getreg(SAM_UDP_TXVC);

  if (enable)
    {
      /* Connect the 1.5 KOhm integrated pull-up on DDP */

      regval |= UDP_TXVC_PUON;
    }
  else
    {
      /* Disconnect the 1.5 KOhm integrated pull-up on DDP */

      regval &= ~UDP_TXVC_PUON;

      /* Device returns to the Powered state */

      if (priv->devstate > UDP_DEVSTATE_POWERED)
        {
          priv->devstate = UDP_DEVSTATE_POWERED;
        }
    }

  sam_putreg(regval, SAM_UDP_TXVC);
  return OK;
}

/****************************************************************************
 * Initialization/Reset
 ****************************************************************************/

/****************************************************************************
 * Name: sam_reset
 ****************************************************************************/

static void sam_reset(struct sam_usbdev_s *priv)
{
  uint32_t regval;
  uint8_t epno;

  /* Make sure that clocking is enabled to the UDP peripheral. */

  sam_enableclks();

  /* Tell the class driver that we are disconnected.  The class driver
   * should then accept any new configurations.
   */

  CLASS_DISCONNECT(priv->driver, &priv->usbdev);

  /* The device enters the Default state (un-addressed and un-configured) */

  priv->devaddr   = 0;
  sam_setdevaddr(priv, 0);

  priv->devstate  = UDP_DEVSTATE_DEFAULT;

  /* Reset and disable all endpoints other.  Then re-configure EP0 */

  sam_epset_reset(priv, SAM_EPSET_ALL);
  sam_ep_configure_internal(&priv->eplist[EP0], &g_ep0desc);

  /* Reset endpoint data structures */

  for (epno = 0; epno < SAM_UDP_NENDPOINTS; epno++)
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
      privep->zlpneeded = false;
      privep->zlpsent   = false;
      privep->txbusy    = false;
    }

  /* Re-configure the USB controller in its initial, unconnected state */

  priv->usbdev.speed = USB_SPEED_FULL;

  /* Clear all pending interrupt status */

  regval = UDP_INT_WAKEUP | UDP_ISR_ENDBUSRES | UDP_INT_SOF | UDP_INT_RXSUSP;
  sam_putreg(regval, SAM_UDP_ICR);

  /* Enable normal operational interrupts (including endpoint 0) */

  regval = UDP_INT_WAKEUP | UDP_INT_RXSUSP | UDP_INT_EP0;
  sam_putreg(regval, SAM_UDP_IER);

  sam_dumpep(priv, EP0);
}

/****************************************************************************
 * Name: sam_enableclks
 ****************************************************************************/

static void sam_enableclks(void)
{
  uint32_t regval;

  /* To use the UDP, the programmer must first configuration the USB clock
   * input, in the PMC_UCKR register if the MCU has a UPLL or in the PMC_USB
   * register if PLLA or PLLB is divided down to generated the clock.  In
   * either case, the UDP Clock must in the PMC_PCER register.
   */

  /* Set the UDP bit in the PMC_PCER1 register in order to enable the MCK
   * clock to the UDP peripheral.  This is necessary in order to access
   * the UDP registers.
   */

  sam_udp_enableclk();

  /* Set the UDP bit in the SCER register to enable the USB clock output
   * to the UDP peripheral.
   */

  regval  = getreg32(SAM_PMC_SCER);
  regval |= PMC_UDP;
  putreg32(regval, SAM_PMC_SCER);

  /* Make sure that the UDP transceiver is not disabled */

  regval  = getreg32(SAM_UDP_TXVC);
  regval &= ~UDP_TXVC_TXVDIS;
  putreg32(regval, SAM_UDP_TXVC);
}

/****************************************************************************
 * Name: sam_disableclks
 ****************************************************************************/

static void sam_disableclks(void)
{
  uint32_t regval;

  /* Disable the UDP transceiver */

  regval  = getreg32(SAM_UDP_TXVC);
  regval |= UDP_TXVC_TXVDIS;
  putreg32(regval, SAM_UDP_TXVC);

  /* Clear the UDP bit in the SCER register to disable the USB clock output */

  regval  = getreg32(SAM_PMC_SCER);
  regval &= ~PMC_UDP;
  putreg32(regval, SAM_PMC_SCER);

  /* Clear the UDP bit in the PMC_PCER1 register in order to disable the MCK
   * clock to the UDP peripheral.  We can no longer access UDP registers.
   */

  sam_udp_disableclk();
}

/****************************************************************************
 * Name: sam_hw_setup
 ****************************************************************************/

static void sam_hw_setup(struct sam_usbdev_s *priv)
{
  uint32_t regval;
  int i;

  /* To use the UDP, the programmer must first configuration the USB clock
   * input, in the PMC_UCKR register if the MCU has a UPLL or in the PMC_USB
   * register if PLLA or PLLB is divided down to generated the clock.  In
   * either case, the UDP Clock must in the PMC_PCER register.
   */

  sam_enableclks();

  /* Configure PIO pins -- nothing needs to be done.
   *
   * By default, the USB function is activated and pins DDP and DDM are used
   * for USB.  The USB device interface also has an interrupt line for VBUS
   * sensing that is connected to the Interrupt Controller.
   */

  /* Reset and disable endpoints */

  sam_epset_reset(priv, SAM_EPSET_ALL);

  /* Initialize Endpoints */

  for (i = 0; i < SAM_UDP_NENDPOINTS; i++)
    {
      /* Reset endpoint configuration */

      sam_putreg(0, SAM_UDPEP_CSR(i));
    }

  /* Enable the remote wakeup feature */

  regval  = sam_getreg(SAM_UDP_GLBSTAT);
  regval |= UDP_GLBSTAT_RMWUPE;
  sam_putreg(regval, SAM_UDP_GLBSTAT);

  /* Disable all interrupts */

  sam_putreg(UDP_INT_ALL, SAM_UDP_IDR);

  /* Disable the 1.5 KOhm integrated pull-up on DDP and make sure that the UDP
   * transceiver is not disabled
   */

  sam_putreg(0, SAM_UDP_TXVC);
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
  priv->devstate   = UDP_DEVSTATE_SUSPENDED;
  priv->prevstate  = UDP_DEVSTATE_POWERED;

  /* Initialize the endpoint list */

  for (epno = 0; epno < SAM_UDP_NENDPOINTS; epno++)
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

      priv->eplist[epno].ep.maxpacket = SAM_UDP_MAXPACKETSIZE(epno);
    }

  /* Select a smaller endpoint size for EP0 */

#if SAM_EP0MAXPACKET < SAM_MAXPACKET_SIZE
  priv->eplist[EP0].ep.maxpacket = SAM_EP0MAXPACKET;
#endif
}

/****************************************************************************
 * Name: sam_hw_shutdown
 ****************************************************************************/

static void sam_hw_shutdown(struct sam_usbdev_s *priv)
{
  uint32_t regval;

  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable all interrupts */

  sam_putreg(UDP_INT_ALL, SAM_UDP_IDR);

  /* Clear all pending interrupt status */

  regval = UDP_INT_WAKEUP | UDP_ISR_ENDBUSRES | UDP_INT_SOF | UDP_INT_RXSUSP;
  sam_putreg(regval, SAM_UDP_ICR);

  /* Disconnect the device / disable the pull-up */

  sam_pullup(&priv->usbdev, false);

  /* Disable clocking to the UDP peripheral */

  sam_disableclks();
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
 * Name: up_usbinitialize
 * Description:
 *   Initialize the USB driver
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_usbinitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_udp;

  usbtrace(TRACE_DEVINIT, 0);

  /* Software initialization */

  sam_sw_setup(priv);

  /* Power up and initialize USB controller.  Interrupts from the UDP
   * controller are initialized here, but will not be enabled at the AIC
   * until the class driver is installed.
   */

  sam_hw_setup(priv);

  /* Attach USB controller interrupt handlers.  The hardware will not be
   * initialized and interrupts will not be enabled until the class device
   * driver is bound.  Getting the IRQs here only makes sure that we have
   * them when we need them later.
   */

  if (irq_attach(SAM_IRQ_UDP, sam_udp_interrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
               (uint16_t)SAM_IRQ_UDP);
      goto errout;
    }

  return;

errout:
  up_usbuninitialize();
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
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_udp;
  irqstate_t flags;

  flags = irqsave();
  usbtrace(TRACE_DEVUNINIT, 0);

  /* Disable and detach the UDP IRQ */

  up_disable_irq(SAM_IRQ_UDP);
  irq_detach(SAM_IRQ_UDP);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Put the hardware in an inactive state */

  sam_hw_shutdown(priv);
  sam_sw_shutdown(priv);
  irqrestore(flags);
}

/****************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method will be
 *   called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_udp;
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG
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
      /* Enable USB controller interrupts at the AIC.
       *
       * NOTE that interrupts and clocking are left disabled in the UDP
       * peripheral.  The ENDBUSRES interrupt will automatically be enabled
       * when the bus reset occurs.  The normal operating configuration will
       * be established at that time.
       */

      up_enable_irq(SAM_IRQ_UDP);

      /* Enable pull-up to connect the device.  The host should enumerate us
       * some time after this.  The next thing we expect is the ENDBUSRES
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

  struct sam_usbdev_s *priv = &g_udp;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = irqsave();

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts (but keep them attached) */

  up_disable_irq(SAM_IRQ_UDP);

  /* Put the hardware in an inactive state.  Then bring the hardware back up
   * in the initial state.  This is essentially the same state as we were
   * in when up_usbinitialize() was first called.
   */

  sam_hw_shutdown(priv);
  sam_sw_shutdown(priv);

  sam_sw_setup(priv);
  sam_hw_setup(priv);

  /* Unhook the driver */

  priv->driver = NULL;
  irqrestore(flags);
  return OK;
}

#endif /* CONFIG_USBDEV && CONFIG_SAM34_UDP */
