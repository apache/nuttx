/****************************************************************************
 * arch/arm/src/samv7/sam_usbdevhs.c
 *
 *   Copyright (C) 2015-2016, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.orgr>
 *
 * This code derives from the UDPHS device controller driver for the SAMA5D3.
 * That code, in turn, includes some reference logic extracted from the
 * SAMA5D3 sample code.  That Atmel sample code has a BSD compatible license
 * that requires this copyright notice:
 *
 *   Copyright (c) 2009, Atmel Corporation
 *
 * Additional updates for the SAMV7 was taken from Atmel sample code for the
 * SAMV71:
 *
 *   Copyright (c) 2014, Atmel Corporation
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "barriers.h"

#include "chip.h"
#include "sam_periphclks.h"
#include "hardware/sam_usbhs.h"
#include "sam_clockconfig.h"
#include "sam_usbdev.h"

#if defined(CONFIG_USBDEV) && defined(CONFIG_SAMV7_USBDEVHS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef CONFIG_DEBUG_USB_INFO
# undef CONFIG_SAMV7_USBHS_REGDEBUG
#endif

/* Number of DMA transfer descriptors.  Default: 8 */

#ifndef CONFIG_SAMV7_USBDEVHS_NDTDS
#  define CONFIG_SAMV7_USBDEVHS_NDTDS 8
#endif

#ifndef CONFIG_USBDEV_DMA
#  warning Currently CONFIG_USBDEV_DMA must be set to make all endpoints working
#endif

#if defined(CONFIG_USBDEV_DUALSPEED) && defined(CONFIG_SAMV7_USBDEVHS_LOWPOWER)
#  error CONFIG_USBDEV_DUALSPEED must not be defined with full-speed only support
#endif

#if !defined(CONFIG_USBDEV_DUALSPEED) && !defined(CONFIG_SAMV7_USBDEVHS_LOWPOWER)
#  warning CONFIG_USBDEV_DUALSPEED should be defined for high speed support
#endif

/* Not yet supported */

#undef CONFIG_SAMV7_USBDEVHS_SCATTERGATHER

/* Driver Definitions *******************************************************/

/* Initial interrupt mask: Reset + Suspend + Correct Transfer */

#define SAM_CNTR_SETUP     (USB_CNTR_RESETM|USB_CNTR_SUSPM|USB_CNTR_CTRM)

/* Endpoint definitions */

#define EP0                 (0)
#define SAM_EPSET_ALL       (0xffff)  /* All endpoints */
#define SAM_EPSET_NOTEP0    (0xfffe)  /* All endpoints except EP0 */
#define SAM_EP_BIT(ep)      (1 << (ep))
#define SAM_EP0_MAXPACKET   (64)      /* EP0 Max. packet size */

#ifdef CONFIG_SAMV7_USBHS_EP7DMA_WAR
/* Normally EP1..7 should support DMA (0x1fe), but according an ERRATA in
 * "Atmel-11296D-ATARM-SAM E70-Datasheet_19-Jan-16" only the EP1..6 support
 * the DMA transfer (0x7e)
 */

#  define SAM_EPSET_DMA     (0x007e)  /* All endpoints that support DMA transfers */
#else
#  define SAM_EPSET_DMA     (0x01fe)  /* All endpoints that support DMA transfers */
#endif

/* DMA FIFO */

#define DMA_MAX_FIFO_SIZE   (65536/1) /* Max size of the FMA FIFO */
#define EPT_FIFO_SIZE       16384     /* FIFO space size in units of 32-bit words */

/* USB-related masks */

#define REQRECIPIENT_MASK     (USB_REQ_TYPE_MASK | USB_REQ_RECIPIENT_MASK)

/* Endpoint register masks (handling toggle fields) */

#define EPR_NOTOG_MASK        (USB_EPR_CTR_RX  | USB_EPR_SETUP  | USB_EPR_EPTYPE_MASK |\
                               USB_EPR_EP_KIND | USB_EPR_CTR_TX | USB_EPR_EA_MASK)
#define EPR_TXDTOG_MASK       (USB_EPR_STATTX_MASK | EPR_NOTOG_MASK)
#define EPR_RXDTOG_MASK       (USB_EPR_STATRX_MASK | EPR_NOTOG_MASK)

/* Cache-related */

#define MEMORY_SYNC()          do { ARM_DSB();ARM_ISB(); } while (0)

/* Request queue operations *************************************************/

#define sam_rqempty(q)        ((q)->head == NULL)
#define sam_rqpeek(q)         ((q)->head)

/* Buffer Alignment *********************************************************
 *
 * DMA buffers be aligned the 8-byte (2 word boundaries).  However, if the
 * data cache is enabled the a higher level of alignment is required.  That
 * is because the data will need to be invalidated and that cache
 * invalidation will occur in multiples of full cache lines.
 */

#ifdef CONFIG_ARMV7M_DCACHE
/* Align to the cache line size which we assume is >= 8 */

#  define USBHS_ALIGN         ARMV7M_DCACHE_LINESIZE
#  define USBHS_ALIGN_MASK    (USBHS_ALIGN-1)
#  define USBHS_ALIGN_DOWN(n) ((n) & ~USBHS_ALIGN_MASK)
#  define USBHS_ALIGN_UP(n)   (((n) + USBHS_ALIGN_MASK) & ~USBHS_ALIGN_MASK)

#  ifndef CONFIG_ARMV7M_DCACHE_WRITETHROUGH
#    warning !!! This driver will not work without CONFIG_ARMV7M_DCACHE_WRITETHROUGH=y!!!
#  endif

#else
/* Use the minimum alignment requirement */

#  define USBHS_ALIGN         8
#  define USBHS_ALIGN_MASK    7
#  define USBHS_ALIGN_DOWN(n) ((n) & ~7)
#  define USBHS_ALIGN_UP(n)   (((n) + 7) & ~7)

#endif

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
#define SAM_TRACEERR_DMAERR               0x0010
#define SAM_TRACEERR_DRIVER               0x0011
#define SAM_TRACEERR_DRIVERREGISTERED     0x0012
#define SAM_TRACERR_REMAINING             0x0013
#define SAM_TRACEERR_EP0SETUPOUTSIZE      0x0014
#define SAM_TRACEERR_EP0SETUPSTALLED      0x0015
#define SAM_TRACEERR_EPOUTNULLPACKET      0x0016
#define SAM_TRACEERR_EPRESERVE            0x0017
#define SAM_TRACEERR_NCFGOK               0x0018
#define SAM_TRACEERR_INVALIDCTRLREQ       0x0019
#define SAM_TRACEERR_IRQREGISTRATION      0x001a
#define SAM_TRACEERR_NOTCONFIGURED        0x001b
#define SAM_TRACEERR_REQABORTED           0x001c
#define SAM_TRACEERR_TXINERR              0x001d

/* Trace interrupt codes */

#define SAM_TRACEINTID_ADDRESSED          0x0001
#define SAM_TRACEINTID_CLEARFEATURE       0x0002
#define SAM_TRACEINTID_INTSUSPD           0x0003
#define SAM_TRACEINTID_DEVGETSTATUS       0x0004
#define SAM_TRACEINTID_DISPATCH           0x0005
#define SAM_TRACEINTID_DMA                0x0006
#define SAM_TRACEINTID_DMAEOB             0x0007
#define SAM_TRACEINTID_DMAEOC             0x0008
#define SAM_TRACEINTID_ENDRESET           0x0009
#define SAM_TRACEINTID_EP                 0x000a
#define SAM_TRACEINTID_EP0SETUPIN         0x000b
#define SAM_TRACEINTID_EP0SETUPOUT        0x000c
#define SAM_TRACEINTID_EP0SETUPSETADDRESS 0x000d
#define SAM_TRACEINTID_EPDMAINT           0x000e
#define SAM_TRACEINTID_EPGETSTATUS        0x000f
#define SAM_TRACEINTID_EPINQEMPTY         0x0010
#define SAM_TRACEINTID_EPINT              0x0011
#define SAM_TRACEINTID_EPOUTQEMPTY        0x0012
#define SAM_TRACEINTID_GETCONFIG          0x0013
#define SAM_TRACEINTID_GETSETDESC         0x0014
#define SAM_TRACEINTID_GETSETIF           0x0015
#define SAM_TRACEINTID_GETSTATUS          0x0016
#define SAM_TRACEINTID_IFGETSTATUS        0x0017
#define SAM_TRACEINTID_INTERRUPT          0x0018
#define SAM_TRACEINTID_INTSOF             0x0019
#define SAM_TRACEINTID_INTMSOF            0x001a
#define SAM_TRACEINTID_NOSTDREQ           0x001b
#define SAM_TRACEINTID_PENDING            0x001c
#define SAM_TRACEINTID_RXRDY              0x001d
#define SAM_TRACEINTID_RXSETUP            0x001e
#define SAM_TRACEINTID_SETCONFIG          0x001f
#define SAM_TRACEINTID_SETFEATURE         0x0020
#define SAM_TRACEINTID_SPEED              0x0021
#define SAM_TRACEINTID_STALLSNT           0x0022
#define SAM_TRACEINTID_SYNCHFRAME         0x0023
#define SAM_TRACEINTID_TXINI              0x0024
#define SAM_TRACEINTID_UPSTRRES           0x0025
#define SAM_TRACEINTID_WAKEUP             0x0026

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
 *
 * Private Type Definitions
 ****************************************************************************/

/* State of an endpoint */

enum sam_epstate_e
{
                              /* --- All Endpoints --- */

  USBHS_EPSTATE_DISABLED = 0, /* Endpoint is disabled */
  USBHS_EPSTATE_STALLED,      /* Endpoint is stalled */
  USBHS_EPSTATE_IDLE,         /* Endpoint is idle (i.e. ready for transmission) */
  USBHS_EPSTATE_SENDING,      /* Endpoint is sending data */
  USBHS_EPSTATE_SENDING_DMA,  /* Endpoint is sending data via DMA */
  USBHS_EPSTATE_NBUSYBK,      /* Endpoint DMA complete, waiting for NBUSYBK==0 */
  USBHS_EPSTATE_RECEIVING,    /* Endpoint is receiving data */
                              /* --- Endpoint 0 Only --- */
  USBHS_EPSTATE_EP0DATAOUT,   /* Endpoint 0 is receiving SETUP OUT data */
  USBHS_EPSTATE_EP0STATUSIN,  /* Endpoint 0 is sending SETUP status */
  USBHS_EPSTATE_EP0ADDRESS    /* Address change is pending completion of status */
};

/* The overall state of the device */

enum sam_devstate_e
{
  USBHS_DEVSTATE_SUSPENDED = 0, /* The device is currently suspended */
  USBHS_DEVSTATE_POWERED,       /* Host is providing +5V through the USB cable */
  USBHS_DEVSTATE_DEFAULT,       /* Device has been reset */
  USBHS_DEVSTATE_ADDRESSED,     /* The device has been given an address on the bus */
  USBHS_DEVSTATE_CONFIGURED     /* A valid configuration has been selected. */
};

/* The result of EP0 SETUP processing */

enum sam_ep0setup_e
{
  USBHS_EP0SETUP_SUCCESS = 0,   /* The SETUP was handle without incident */
  USBHS_EP0SETUP_DISPATCHED,    /* The SETUP was forwarded to the class driver */
  USBHS_EP0SETUP_ADDRESS,       /* A new device address is pending */
  USBHS_EP0SETUP_STALL          /* An error occurred */
};

/* DMA transfer descriptor */

#ifdef CONFIG_SAMV7_USBDEVHS_SCATTERGATHER
struct sam_dtd_s
{
  struct usbhs_dtd_s hw;     /* These are the fields as seen by the hardware */
  uint32_t pad;              /* Pad to 16 bytes to support arrays of descriptors */
};
#define SIZEOF_SAM_DTD_S 16
#endif

/* The following is used to manage lists of free DMA transfer descriptors */

struct sam_list_s
{
  struct sam_list_s *flink;  /* Link to next entry in the list */
                             /* Variable length entry data follows */
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

  /* SAMV7-specific fields */

  struct sam_usbdev_s *dev;          /* Reference to private driver data */
  struct sam_rqhead_s  reqq;         /* Read/write request queue */
#ifdef CONFIG_SAMV7_USBDEVHS_SCATTERGATHER
  struct sam_dtd_s    *dtdll;        /* Head of the DMA transfer descriptor list */
#endif
  volatile uint8_t     epstate;      /* State of the endpoint (see enum sam_epstate_e) */
  volatile uint8_t     bank;         /* Current reception bank (0 or 1) */
  uint8_t              stalled:1;    /* true: Endpoint is stalled */
  uint8_t              halted:1;     /* true: Endpoint feature halted */
  uint8_t              zlpneeded:1;  /* Zero length packet needed at end of transfer */
  uint8_t              zlpsent:1;    /* Zero length packet has been sent */
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

  /* USBHS-specific fields */

  struct usb_ctrlreq_s     ctrl;          /* Last EP0 request */
  uint8_t                  devstate;      /* State of the device (see enum sam_devstate_e) */
  uint8_t                  prevstate;     /* Previous state of the device before SUSPEND */
  uint8_t                  selfpowered:1; /* 1: Device is self powered */
  uint16_t                 epavail;       /* Bitset of available endpoints */

  /* DMA Transfer descriptors */

#ifdef CONFIG_SAMV7_USBDEVHS_SCATTERGATHER
  struct sam_dtd_s        *tdfree;        /* A list of free transfer descriptors */
#ifndef CONFIG_SAMV7_USBDEVHS_PREALLOCATE
  struct sam_dtd_s        *tdpool;        /* Pool of allocated DMA transfer descriptors */
#endif
#endif

  /* The endpoint list */

  struct sam_ep_s          eplist[SAM_USBHS_NENDPOINTS];

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

#ifdef CONFIG_SAMV7_USBHS_REGDEBUG
static void   sam_printreg(uintptr_t regaddr, uint32_t regval, bool iswrite);
static void   sam_checkreg(uintptr_t regaddr, uint32_t regval, bool iswrite);
static uint32_t sam_getreg(uintptr_t regaddr);
static void   sam_putreg(uint32_t regval, uintptr_t regaddr);
static void   sam_dumpep(struct sam_usbdev_s *priv, int epno);
#else
static inline uint32_t sam_getreg(uintptr_t regaddr);
static inline void sam_putreg(uint32_t regval, uintptr_t regaddr);
# define sam_dumpep(priv,epno)
#endif

/* Suspend/Resume Helpers ***************************************************/

static void   sam_suspend(struct sam_usbdev_s *priv);
static void   sam_resume(struct sam_usbdev_s *priv);

/* DMA Transfer Helpers *****************************************************/

#ifdef CONFIG_SAMV7_USBDEVHS_SCATTERGATHER
static struct sam_dtd_s *sam_dtd_alloc(struct sam_usbdev_s *priv);
static void   sam_dtd_free(struct sam_usbdev_s *priv, struct sam_dtd_s *dtd);
#endif
static void   sam_dma_single(uint8_t epno, struct sam_req_s *privreq,
                uint32_t dmacontrol);
static void   sam_dma_wrsetup(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, struct sam_req_s *privreq);
static void   sam_dma_rdsetup(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, struct sam_req_s *privreq);

/* Request Helpers **********************************************************/

static struct sam_req_s *
              sam_req_dequeue(struct sam_rqhead_s *queue);
static void   sam_req_enqueue(struct sam_rqhead_s *queue,
                struct sam_req_s *req);
static inline void
              sam_req_abort(struct sam_ep_s *privep,
                struct sam_req_s *privreq, int16_t result);
static void   sam_req_complete(struct sam_ep_s *privep, int16_t result);
static void   sam_ep_fifocon(unsigned int epno);
static void   sam_req_wrsetup(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, struct sam_req_s *privreq);
static int    sam_req_write(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep);
static void   sam_req_rddone(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, struct sam_req_s *privreq,
                uint16_t recvsize);
static void   sam_req_rdenable(uint8_t epno);
static void   sam_req_rddisable(uint8_t epno);
static int    sam_req_read(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, uint16_t recvsize);
static void   sam_req_cancel(struct sam_ep_s *privep, int16_t status);

/* Interrupt level processing ***********************************************/

static void   sam_ep0_read(uint8_t *buffer, size_t buflen);
static void   sam_ctrlep_write(struct sam_ep_s *privep,
                const uint8_t *buffer, size_t buflen);
static void   sam_ep_write(struct sam_ep_s *privep, const uint8_t *buffer,
                size_t buflen);
static void   sam_ep0_dispatch(struct sam_usbdev_s *priv);
static void   sam_ep0_setup(struct sam_usbdev_s *priv);
#ifdef CONFIG_USBDEV_DMA
static void   sam_dma_interrupt(struct sam_usbdev_s *priv, int epno);
#endif
static void   sam_ep_interrupt(struct sam_usbdev_s *priv, int epno);
static int    sam_usbhs_interrupt(int irq, void *context, void *arg);

/* Endpoint helpers *********************************************************/

static void   sam_ep_reset(struct sam_usbdev_s *priv, uint8_t epno);
static void   sam_epset_reset(struct sam_usbdev_s *priv, uint16_t epset);
static inline struct sam_ep_s *
              sam_ep_reserve(struct sam_usbdev_s *priv, uint16_t epset);
static inline void
              sam_ep_unreserve(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep);
static int    sam_ep_configure_internal(struct sam_ep_s *privep,
                const struct usb_epdesc_s *desc);
static inline int
              sam_ep0_configure(struct sam_usbdev_s *priv);

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
static int    sam_ep_stall(struct usbdev_ep_s *ep, bool resume);

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

static struct sam_usbdev_s g_usbhs;

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
  .stall         = sam_ep_stall,
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
                  {64, 0},
  .interval      = 0
};

#ifdef CONFIG_SAMV7_USBDEVHS_SCATTERGATHER
#ifdef CONFIG_SAMV7_USBDEVHS_PREALLOCATE
/* This is a properly aligned pool of preallocated DMA transfer descriptors */

static struct sam_dtd_s g_dtdpool[CONFIG_SAMV7_USBDEVHS_NDTDS]
                        aligned_data(16);
#endif
#endif

/* Device error strings that may be enabled for more descriptive USB trace
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
  TRACE_STR(SAM_TRACEERR_DMAERR),
  TRACE_STR(SAM_TRACEERR_DRIVER),
  TRACE_STR(SAM_TRACEERR_DRIVERREGISTERED),
  TRACE_STR(SAM_TRACERR_REMAINING),
  TRACE_STR(SAM_TRACEERR_EP0SETUPOUTSIZE),
  TRACE_STR(SAM_TRACEERR_EP0SETUPSTALLED),
  TRACE_STR(SAM_TRACEERR_EPOUTNULLPACKET),
  TRACE_STR(SAM_TRACEERR_EPRESERVE),
  TRACE_STR(SAM_TRACEERR_NCFGOK),
  TRACE_STR(SAM_TRACEERR_INVALIDCTRLREQ),
  TRACE_STR(SAM_TRACEERR_IRQREGISTRATION),
  TRACE_STR(SAM_TRACEERR_NOTCONFIGURED),
  TRACE_STR(SAM_TRACEERR_REQABORTED),
  TRACE_STR(SAM_TRACEERR_TXINERR),
  TRACE_STR_END
};
#endif

/* Interrupt event strings that may be enabled for more descriptive USB trace
 * output.
 */

#ifdef CONFIG_USBDEV_TRACE_STRINGS
const struct trace_msg_t g_usb_trace_strings_intdecode[] =
{
  TRACE_STR(SAM_TRACEINTID_ADDRESSED),
  TRACE_STR(SAM_TRACEINTID_CLEARFEATURE),
  TRACE_STR(SAM_TRACEINTID_INTSUSPD),
  TRACE_STR(SAM_TRACEINTID_DEVGETSTATUS),
  TRACE_STR(SAM_TRACEINTID_DISPATCH),
  TRACE_STR(SAM_TRACEINTID_DMA),
  TRACE_STR(SAM_TRACEINTID_DMAEOB),
  TRACE_STR(SAM_TRACEINTID_DMAEOC),
  TRACE_STR(SAM_TRACEINTID_ENDRESET),
  TRACE_STR(SAM_TRACEINTID_EP),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPIN),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPOUT),
  TRACE_STR(SAM_TRACEINTID_EP0SETUPSETADDRESS),
  TRACE_STR(SAM_TRACEINTID_EPDMAINT),
  TRACE_STR(SAM_TRACEINTID_EPGETSTATUS),
  TRACE_STR(SAM_TRACEINTID_EPINQEMPTY),
  TRACE_STR(SAM_TRACEINTID_EPINT),
  TRACE_STR(SAM_TRACEINTID_EPOUTQEMPTY),
  TRACE_STR(SAM_TRACEINTID_GETCONFIG),
  TRACE_STR(SAM_TRACEINTID_GETSETDESC),
  TRACE_STR(SAM_TRACEINTID_GETSETIF),
  TRACE_STR(SAM_TRACEINTID_GETSTATUS),
  TRACE_STR(SAM_TRACEINTID_IFGETSTATUS),
  TRACE_STR(SAM_TRACEINTID_INTERRUPT),
  TRACE_STR(SAM_TRACEINTID_INTSOF),
  TRACE_STR(SAM_TRACEINTID_INTMSOF),
  TRACE_STR(SAM_TRACEINTID_NOSTDREQ),
  TRACE_STR(SAM_TRACEINTID_PENDING),
  TRACE_STR(SAM_TRACEINTID_RXRDY),
  TRACE_STR(SAM_TRACEINTID_RXSETUP),
  TRACE_STR(SAM_TRACEINTID_SETCONFIG),
  TRACE_STR(SAM_TRACEINTID_SETFEATURE),
  TRACE_STR(SAM_TRACEINTID_SPEED),
  TRACE_STR(SAM_TRACEINTID_STALLSNT),
  TRACE_STR(SAM_TRACEINTID_SYNCHFRAME),
  TRACE_STR(SAM_TRACEINTID_TXINI),
  TRACE_STR(SAM_TRACEINTID_UPSTRRES),
  TRACE_STR(SAM_TRACEINTID_WAKEUP),
  TRACE_STR_END
};
#endif

/****************************************************************************
 * Private Private Functions
 ****************************************************************************/

/****************************************************************************
 * Register Operations
 ****************************************************************************/

/****************************************************************************
 * Name: sam_printreg
 *
 * Description:
 *   Print the contents of a SAMV7 USBHS register
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_USBHS_REGDEBUG
static void sam_printreg(uintptr_t regaddr, uint32_t regval, bool iswrite)
{
  uinfo("%p%s%08x\n", regaddr, iswrite ? "<-" : "->", regval);
}
#endif

/****************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if it is time to output debug information for accesses to a SAMV7
 *   USBHS register
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_USBHS_REGDEBUG
static void sam_checkreg(uintptr_t regaddr, uint32_t regval, bool iswrite)
{
  static uintptr_t prevaddr  = 0;
  static uint32_t  preval    = 0;
  static uint32_t  count     = 0;
  static bool      prevwrite = false;

  /* Is this the same value that we read from/wrote
   * to the same register last time?
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
 * Name: sam_getreg
 *
 * Description:
 *   Get the contents of an SAMV7 register
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_USBHS_REGDEBUG
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

/****************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *   Set the contents of an SAMV7 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_USBHS_REGDEBUG
static void sam_putreg(uint32_t regval, uintptr_t regaddr)
{
  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, true);

  /* Write the value */

  putreg32(regval, regaddr);
}
#else
static inline void sam_putreg(uint32_t regval, uintptr_t regaddr)
{
  putreg32(regval, regaddr);
}
#endif

/****************************************************************************
 * Name: sam_dumpep
 ****************************************************************************/

#ifdef CONFIG_SAMV7_USBHS_REGDEBUG
static void sam_dumpep(struct sam_usbdev_s *priv, int epno)
{
  /* Global Registers */

  uinfo("Global Register:\n");
  uinfo("  CTRL:    %08x\n", sam_getreg(SAM_USBHS_DEVCTRL));
  uinfo("  ISR:     %08x\n", sam_getreg(SAM_USBHS_DEVISR));
  uinfo("  IMR:     %08x\n", sam_getreg(SAM_USBHS_DEVIMR));
  uinfo("  EPT:     %08x\n", sam_getreg(SAM_USBHS_DEVEPT));
  uinfo("  FNUM:    %08x\n", sam_getreg(SAM_USBHS_DEVFNUM));

  /* Endpoint registers */

  uinfo("Endpoint %d Register:\n", epno);
  uinfo("  CFG:     %08x\n", sam_getreg(SAM_USBHS_DEVEPTCFG(epno)));
  uinfo("  ISR:     %08x\n", sam_getreg(SAM_USBHS_DEVEPTISR(epno)));
  uinfo("  IMR:     %08x\n", sam_getreg(SAM_USBHS_DEVEPTIMR(epno)));

  uinfo("DMA %d Register:\n", epno);
  if ((SAM_EPSET_DMA & SAM_EP_BIT(epno)) != 0)
    {
      uinfo("  NXTDSC:  %08x\n", sam_getreg(SAM_USBHS_DEVDMANXTDSC(epno)));
      uinfo("  ADDRESS: %08x\n", sam_getreg(SAM_USBHS_DEVDMAADDR(epno)));
      uinfo("  CONTROL: %08x\n", sam_getreg(SAM_USBHS_DEVDMACTRL(epno)));
      uinfo("  STATUS:  %08x\n", sam_getreg(SAM_USBHS_DEVDMASTA(epno)));
    }
  else
    {
      uinfo("  None\n");
    }
}
#endif

/****************************************************************************
 * DMA
 ****************************************************************************/

/****************************************************************************
 * Name: sam_dtd_alloc
 *
 * Description:
 *   Allocate a DMA transfer descriptor by removing it from the free list
 *
 * Assumption:  Caller holds the lock
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_USBDEVHS_SCATTERGATHER
static struct sam_dtd_s *sam_dtd_alloc(struct sam_usbdev_s *priv)
{
  struct sam_dtd_s *dtd;

  /* Remove the DMA transfer descriptor from the freelist */

  dtd = (struct sam_dtd_s *)g_usbhs.dtdfree;
  if (dtd)
    {
      g_usbhs.dtdfree = ((struct sam_list_s *)dtd)->flink;
      memset(dtd, 0, sizeof(struct sam_dtd_s));
    }

  return dtd;
}
#endif

/****************************************************************************
 * Name: sam_dtd_free
 *
 * Description:
 *   Free a DMA transfer descriptor by returning it to the free list
 *
 * Assumption:  Caller holds the lock
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_USBDEVHS_SCATTERGATHER
static void sam_dtd_free(struct sam_usbdev_s *priv, struct sam_dtd_s *dtd)
{
  struct sam_list_s *entry = (struct sam_list_s *)dtd;

  /* Put the dtd structure back into the free list */

  entry->flink  = g_usbhs.dtdfree;
  g_usbhs.dtdfree = entry;
}
#endif

/****************************************************************************
 * Name: sam_dma_single
 *
 * Description:
 *   Setup a start a single buffer DMA.
 *
 * Assumption:  Called as part of USBHS interrupt handling
 *
 ****************************************************************************/

static void sam_dma_single(uint8_t epno, struct sam_req_s *privreq,
                           uint32_t dmacontrol)
{
  uintptr_t buffer;

  /* Not all endpoints support DMA */

  DEBUGASSERT((SAM_EPSET_DMA & SAM_EP_BIT(epno)) != 0);

  /* Flush the contents of the DMA buffer to RAM */

  buffer = (uintptr_t)&privreq->req.buf[privreq->req.xfrd];
  up_clean_dcache(buffer, buffer + privreq->inflight);

  /* Set up the DMA */

  sam_putreg((uint32_t)buffer, SAM_USBHS_DEVDMAADDR(epno));

  /* Clear any pending interrupts then enable the DMA interrupt */

  sam_getreg(SAM_USBHS_DEVDMASTA(epno));
  sam_putreg(USBHS_DEVINT_DMA(epno), SAM_USBHS_DEVIER);

  /* Setup and enable the DMA */

  sam_putreg(0, SAM_USBHS_DEVDMACTRL(epno));

  dmacontrol |= USBHS_DEVDMACTRL_BUFLEN(privreq->inflight);
  sam_putreg(dmacontrol, SAM_USBHS_DEVDMACTRL(epno));
}

/****************************************************************************
 * Name: sam_dma_wrsetup
 *
 * Description:
 *   Process the next queued write request for an endpoint that supports DMA.
 *
 ****************************************************************************/

static void sam_dma_wrsetup(struct sam_usbdev_s *priv,
                          struct sam_ep_s *privep, struct sam_req_s *privreq)
{
  int remaining;
  int epno;

  /* Switch to the sending state */

  privep->epstate   = USBHS_EPSTATE_SENDING_DMA;
  privreq->inflight = 0;

  /* Get the endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* How many bytes remain to be transferred in the request? */

  remaining = (int)privreq->req.len - (int)privreq->req.xfrd;
  DEBUGASSERT(remaining >= 0 && remaining <= (int)privreq->req.len);

  /* If there are no bytes to send, then send a zero length packet */

  if (remaining > 0)
    {
      /* Clip the transfer to the size of the DMA FIFO */

#if USBDEV_MAXREQUEUST > DMA_MAX_FIFO_SIZE
      if (remaining > DMA_MAX_FIFO_SIZE)
        {
          privreq->inflight = DMA_MAX_FIFO_SIZE;
          privep->zlpneeded = false;
        }
      else
#endif
        {
          privreq->inflight = remaining;

          /* If the size is an exact multiple of full packets, then note if
           * we need to send a zero length packet next.
           */

          privep->zlpneeded =
            ((privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0 &&
             (remaining % privep->ep.maxpacket) == 0);
        }

      /* And perform the single DMA transfer.
       *
       * 32.6.10.6 Bulk IN or Interrupt IN: Sending a Buffer Using DMA
       * - END_B_EN: The endpoint can validate the packet (according to the
       *   values programmed in the AUTO_VALID and SHRT_PCKT fields of
       *   USBHS_EPTCTLx.) ...
       * - END_BUFFIT: generate an interrupt when the BUFF_COUNT in
       *    USBHS_DMASTATUSx reaches 0.
       * - CHANN_ENB: Run and stop at end of buffer
       */

      sam_dma_single(epno, privreq,
                     USBHS_DEVDMACTRL_ENDBEN | USBHS_DEVDMACTRL_ENDBUFFIT |
                     USBHS_DEVDMACTRL_CHANNENB);
    }

  /* Enable the endpoint interrupt */

  sam_putreg(USBHS_DEVINT_PEP(epno), SAM_USBHS_DEVIER);
}

/****************************************************************************
 * Name: sam_dma_rdsetup
 *
 * Description:
 *   Process the next queued read request for an endpoint that supports DMA.
 *
 ****************************************************************************/

static void sam_dma_rdsetup(struct sam_usbdev_s *priv,
                            struct sam_ep_s *privep,
                            struct sam_req_s *privreq)
{
  uint32_t regval;
  int remaining;
  int epno;

  /* Get the endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* How many more bytes can we append to the request buffer? */

  remaining = (int)privreq->req.len - (int)privreq->req.xfrd;
  DEBUGASSERT(remaining > 0 && remaining <= (int)privreq->req.len &&
              privep->epstate == USBHS_EPSTATE_RECEIVING);

  /* Clip the DMA transfer size to the size available in the user buffer */

#if USBDEV_MAXREQUEUST > DMA_MAX_FIFO_SIZE
  if (remaining > DMA_MAX_FIFO_SIZE)
    {
     privreq->inflight = DMA_MAX_FIFO_SIZE;
    }
  else
#endif
    {
      privreq->inflight = remaining;
    }

  /* And perform the single DMA transfer.
   *
   * 32.6.10.12 Bulk OUT or Interrupt OUT: Sending a Buffer Using DMA
   * - END_B_EN: Can be used for OUT packet truncation (discarding of
   *   unbuffered packet data) at the end of DMA buffer.
   * - END_BUFFIT: Generate an interrupt when BUFF_COUNT in the
   *   USBHS_DMASTATUSx register reaches 0.
   * - END_TR_EN: End of transfer enable, the USBHS device can put an
   *   end to the current DMA transfer, in case of a short packet.
   * - END_TR_IT: End of transfer interrupt enable, an interrupt is sent
   *   after the last USB packet has been transferred by the DMA, if the
   *   USB transfer ended with a short packet. (Beneficial when the
   *   receive size is unknown.)
   * - CHANN_ENB: Run and stop at end of buffer.
   */

  regval = USBHS_DEVDMACTRL_ENDBEN | USBHS_DEVDMACTRL_ENDBUFFIT |
           USBHS_DEVDMACTRL_ENDTREN | USBHS_DEVDMACTRL_ENDTRIT |
           USBHS_DEVDMACTRL_CHANNENB;

  sam_dma_single(epno, privreq, regval);
}

/****************************************************************************
 *
 * Request Helpers
 *
 ****************************************************************************/

/****************************************************************************
 *
 * Name: sam_req_dequeue
 *
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
 *
 * Name: sam_req_enqueue
 *
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
 *
 * Name: sam_req_abort
 *
 ****************************************************************************/

static inline void
sam_req_abort(struct sam_ep_s *privep, struct sam_req_s *privreq,
              int16_t result)
{
  usbtrace(TRACE_DEVERROR(SAM_TRACEERR_REQABORTED),
           (uint16_t)USB_EPNO(privep->ep.eplog));

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/****************************************************************************
 *
 * Name: sam_req_complete
 *
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

      /* Reset the endpoint state and restore the stalled indication.
       *
       * At least the USB class CDC/ACM calls the function
       * sam_ep_submit within the callback.
       * This function uses sam_req_write or
       * sam_req_read to process the request,
       * both functions can change the state.
       * Therefore it is very important to set
       * the state to USBHS_EPSTATE_IDLE
       * before the callback is called.
       */

      privep->epstate   = USBHS_EPSTATE_IDLE;
      privep->zlpneeded = false;
      privep->zlpsent   = false;

      /* Callback to the request completion handler */

      privreq->flink = NULL;
      privreq->req.callback(&privep->ep, &privreq->req);
    }
}

/****************************************************************************
 *
 * Name: sam_ep_fifocon
 *
 * Description:
 *   IN data has been loaded in the endpoint FIFO.  Manage the endpoint to
 *   (1) initiate sending of the data and (2) receive the TXIN interrupt
 *   when the transfer completes.
 *
 ****************************************************************************/

static void sam_ep_fifocon(unsigned int epno)
{
  /* Clear FIFOCON to indicate that the packet is ready to send (this works
   * even for zero length packets).  We will get an TXIN interrupt with
   * FIFCON=1 when the transfer completes.  Then we are able to send the
   * next packet.
   */

  sam_putreg(USBHS_DEVEPTINT_FIFOCONI, SAM_USBHS_DEVEPTIDR(epno));

  /* Clear the NAK IN bit to stop NAKing IN tokens from the host.  We now
   * have data ready to go.
   *
   * REVISIT: I don't think the USBHS_DEVEPTINT_NAKINI is necessary.
   */

  sam_putreg(USBHS_DEVEPTINT_NAKINI, SAM_USBHS_DEVEPTICR(epno));

  /* Enable the TXIN interrupt on the endpoint */

  sam_putreg(USBHS_DEVEPTINT_TXINI, SAM_USBHS_DEVEPTIER(epno));
}

/****************************************************************************
 *
 * Name: sam_req_wrsetup
 *
 * Description:
 *   Process the next queued write request for an endpoint that does not
 *   support DMA.
 *
 ****************************************************************************/

static void sam_req_wrsetup(struct sam_usbdev_s *priv,
                            struct sam_ep_s *privep,
                            struct sam_req_s *privreq)
{
  uint32_t regval;
  const uint8_t *buf;
  uint8_t epno;
  unsigned int eptype;
  int nbytes;

  /* Get the unadorned endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* Get the number of bytes remaining to be sent. */

  DEBUGASSERT(privreq->req.xfrd < privreq->req.len);
  nbytes = privreq->req.len - privreq->req.xfrd;

  /* Either send the maxpacketsize or all of the remaining data in
   * the request.
   */

  privep->zlpneeded = false;
  if (nbytes > privep->ep.maxpacket)
    {
      nbytes = privep->ep.maxpacket;
    }
  else if (nbytes == privep->ep.maxpacket)
    {
      /* If the size is exactly a full packet, then note if we need to
       * send a zero length packet next.
       */

      privep->zlpneeded =
        ((privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0);
    }

  /* This is the new number of bytes "in-flight" */

  privreq->inflight = nbytes;
  usbtrace(TRACE_WRITE(USB_EPNO(privep->ep.eplog)), nbytes);

  /* The new buffer pointer is the started of the buffer plus the number
   * of bytes successfully transferred plus the number of bytes previously
   * "in-flight".
   */

  buf = privreq->req.buf + privreq->req.xfrd;

  /* How we send packets differs for control endpoints */

  regval = sam_getreg(SAM_USBHS_DEVEPTCFG(epno));
  eptype = regval & USBHS_DEVEPTCFG_EPTYPE_MASK;

  if (eptype == USBHS_DEVEPTCFG_EPTYPE_CTRL)
    {
      sam_ctrlep_write(privep, buf, nbytes);
    }
  else
    {
      sam_ep_write(privep, buf, nbytes);
    }
}

/****************************************************************************
 *
 * Name: sam_req_write
 *
 * Description:
 *   Process the next queued write request.  This function is called in one
 *   of three contexts:
 *   (1) When the endpoint is IDLE and a new write request
 *   is submitted (with interrupts disabled),
 *   (2) from interrupt handling when the current
 *   transfer completes (either DMA or FIFO), or
 *   (3) when resuming a stalled IN or control endpoint.
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
  uint32_t regval;
  uint32_t eptype;
  uint8_t epno;
  int bytesleft;

  /* Get the unadorned endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* We get here when an IN endpoint interrupt occurs.  So now we know that
   * there is no TX transfer in progress.
   */

  while (privep->epstate == USBHS_EPSTATE_IDLE)
    {
      /* Check the request from the head of the endpoint request queue */

      privreq = sam_rqpeek(&privep->reqq);
      if (!privreq)
        {
          /* There is no TX transfer in progress and no new pending TX
           * requests to send.
           */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPINQEMPTY), 0);

          /* Get the endpoint type */

          regval = sam_getreg(SAM_USBHS_DEVEPTCFG(epno));
          eptype = regval & USBHS_DEVEPTCFG_EPTYPE_MASK;

          /* Disable interrupts on non-control endpoints */

          if (eptype != USBHS_DEVEPTCFG_EPTYPE_CTRL)
            {
              sam_putreg(USBHS_DEVINT_PEP(epno), SAM_USBHS_DEVIDR);
            }

          /* Clear and Disable the TXIN interrupt */

          sam_putreg(USBHS_DEVEPTINT_TXINI, SAM_USBHS_DEVEPTICR(epno));
          sam_putreg(USBHS_DEVEPTINT_TXINI, SAM_USBHS_DEVEPTIDR(epno));
          return -ENOENT;
        }

      uinfo("epno=%d req=%p: len=%d xfrd=%d inflight=%d zlpneeded=%d\n",
            epno, privreq, privreq->req.len, privreq->req.xfrd,
            privreq->inflight, privep->zlpneeded);

      /* Handle any bytes in flight. */

      privreq->req.xfrd += privreq->inflight;
      privreq->inflight  = 0;

      /* Get the number of bytes left to be sent in the packet */

      bytesleft = privreq->req.len - privreq->req.xfrd;
      if (bytesleft > 0)
        {
          /* The way that we handle the transfer is going to depend on
           * whether or not this endpoint supports DMA.  In either case
           * the endpoint state will transition to SENDING.
           */

          if ((SAM_EPSET_DMA & SAM_EP_BIT(epno)) != 0)
            {
              sam_dma_wrsetup(priv, privep, privreq);
            }
          else
            {
              sam_req_wrsetup(priv, privep, privreq);
            }
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

      else if ((privreq->req.len == 0 ||
              privep->zlpneeded) & !privep->zlpsent)
        {
          /* If we get here, then we sent the last of the data on the
           * previous pass and we need to send the zero length packet now.
           *
           * A Zero Length Packet can be sent by clearing just the FIFOCON
           * flag in the USBHS_DEVTEPTIDRx register
           */

          privep->epstate   = USBHS_EPSTATE_SENDING;
          privep->zlpneeded = false;
          privep->zlpsent   = true;
          privreq->inflight = 0;

          /* Initiate the zero length transfer and configure to receive the
           * transfer complete interrupt.
           */

          sam_ep_fifocon(epno);
        }

      /* If all of the bytes were sent (including any final zero length
       * packet) then we are finished with the request buffer and we can
       * return the request buffer to the class driver.  The state will
       * remain IDLE only if nothing else was put in flight.
       *
       * Note that we will then loop to check to check the next queued
       * write request.
       */

      if (privep->epstate == USBHS_EPSTATE_IDLE)
        {
          /* Return the write request to the class driver */

          usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
                   privreq->req.xfrd);

          DEBUGASSERT(privreq->req.len == privreq->req.xfrd);
          sam_req_complete(privep, OK);
        }
    }

  return OK;
}

/****************************************************************************
 *
 * Name: sam_req_rddone
 *
 * Description:
 *   The last non-DMA OUT transfer has completed.  Read 'recvsize' byts from
 *   the FIFO into the read request buffer.
 *
 ****************************************************************************/

static void sam_req_rddone(struct sam_usbdev_s *priv,
                           struct sam_ep_s *privep,
                           struct sam_req_s *privreq, uint16_t recvsize)
{
  const uint8_t *fifo;
  uint8_t *dest;
  int remaining;
  int readlen;
  int epno;

  /* Get the number of bytes that can be received.  This is the size of the
   * user-provided request buffer, minus the number of bytes already
   * transferred to the user-buffer.
   */

  remaining = privreq->req.len - privreq->req.xfrd;

  /* Read the smaller of the number of bytes available in FIFO and the
   * size remaining in the request buffer provided by the caller.
   */

  readlen = MIN(remaining, recvsize);
  privreq->req.xfrd += readlen;

  /* Get the source and destination transfer addresses */

  epno = USB_EPNO(privep->ep.eplog);
  fifo = (const uint8_t *)
    ((uint32_t *)SAM_USBHSRAM_BASE + (EPT_FIFO_SIZE * epno));
  dest = privreq->req.buf + privreq->req.xfrd;

  /* Retrieve packet from the FIFO */

  for (; readlen > 0; readlen--)
    {
      *dest++ = *fifo++;
    }

  MEMORY_SYNC();
}

/****************************************************************************
 *
 * Name: sam_req_rdenable
 *
 * Description:
 *   Make sure that the endpoint RXRDY_TXTK interrupt is enabled in order
 *   to receive the next incoming packet
 *
 ****************************************************************************/

static void sam_req_rdenable(uint8_t epno)
{
  sam_putreg(USBHS_DEVINT_PEP(epno), SAM_USBHS_DEVIER);
  sam_putreg(USBHS_DEVEPTINT_RXOUTI, SAM_USBHS_DEVEPTIER(epno));
}

/****************************************************************************
 *
 * Name: sam_req_rddisable
 *
 * Description:
 *   Disable endpoint interrupts
 *
 ****************************************************************************/

static void sam_req_rddisable(uint8_t epno)
{
  sam_putreg(USBHS_DEVINT_PEP(epno), SAM_USBHS_DEVIDR);
  sam_putreg(USBHS_DEVEPTINT_RXOUTI, SAM_USBHS_DEVEPTIDR(epno));
}

/****************************************************************************
 *
 * Name: sam_req_read
 *
 * Description:
 *   Complete the last read request, return the read request to the class
 *   implementation, and try to started the next queued read request.
 *
 *   This function is called in one of three contexts:  (1) When the endpoint
 *   is IDLE and a new read request is submitted (with interrupts disabled),
 *   (2) from interrupt handling when the current transfer completes (either
 *   DMA or FIFO), or (3) when resuming a stalled OUT or control endpoint.
 *
 *   There is a fundamental difference between receiving packets via DMA and
 *   via the FIFO:
 *
 *   - When receiving data via DMA, then data has already been transferred
 *     and this function is called on the terminating event.  The transfer
 *     is complete and we just need to check for end of request events and
 *     if we need to setup the transfer for the next request.
 *   - When receiving via the FIFO, the transfer is not complete.  The
 *     data is in the FIFO and must be transferred from the FIFO to the
 *     request buffer.  No setup is needed for the next transfer other than
 *     assuring that the endpoint RXRDY_TXTK interrupt is enabled.
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
 *     When this function starts a DMA transfer it will update the request
 *     'inflight' field to hold the maximum size of the transfer; but
 *     'inflight' is not used with FIFO transfers.
 *
 *     When the transfer completes, the 'recvsize' parameter must be the
 *     size of the transfer that just completed.   For the case of DMA,
 *     that is the size of the DMA transfer that has just been written to
 *     memory; for the FIFO transfer, recvsize is the number of bytes
 *     waiting in the FIFO to be read.
 *
 ****************************************************************************/

static int sam_req_read(struct sam_usbdev_s *priv, struct sam_ep_s *privep,
                        uint16_t recvsize)
{
  struct sam_req_s *privreq;
  uint32_t regval;
  uint32_t eptype;
  uint8_t epno;

  DEBUGASSERT(priv && privep && privep->epstate == USBHS_EPSTATE_IDLE);

  /* Loop in case we need to handle multiple read requests */

  while (privep->epstate == USBHS_EPSTATE_IDLE)
    {
      /* Check the request from the head of the endpoint request queue */

      epno    = USB_EPNO(privep->ep.eplog);
      privreq = sam_rqpeek(&privep->reqq);
      if (!privreq)
        {
          /* No packet to receive data */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPOUTQEMPTY), epno);
          return -ENOENT;
        }

      uinfo("EP%d: len=%d xfrd=%d\n",
            epno, privreq->req.len, privreq->req.xfrd);

      /* Ignore any attempt to receive a zero length packet */

      if (privreq->req.len == 0)
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EPOUTNULLPACKET), 0);
          sam_req_complete(privep, OK);
          recvsize = 0;
          continue;
        }

      usbtrace(TRACE_READ(USB_EPNO(privep->ep.eplog)), recvsize);

      /* Update the number of bytes transferred with the received size */

      privreq->req.xfrd += recvsize;
      privreq->inflight  = 0;

      /* If this was not a DMA transfer,
       * read the incoming data from the FIFO
       */

      if ((SAM_EPSET_DMA & SAM_EP_BIT(epno)) == 0)
        {
          sam_req_rddone(priv, privep, privreq, recvsize);
        }

      /* In case we go through the loop again */

      recvsize = 0;

      /* If nothing has yet be transferred into the read request, then
       * indicate that we are in the RECEIVING state and, if the endpoint
       * supports DMA, setup the receive DMA.
       */

      if (privreq->req.xfrd == 0)
        {
          /* Set the RECEIVING state */

          privep->epstate = USBHS_EPSTATE_RECEIVING;

          /* If the endpoint supports DMA, set up the DMA now */

          if ((SAM_EPSET_DMA & SAM_EP_BIT(epno)) != 0)
            {
              /* Set up the next DMA.  We will come through this logic path
               * again with xrfd != 0 when the DMA completes.
               */

              sam_dma_rdsetup(priv, privep, privreq);
            }

          /* No DMA for this endpoint and we have an available, empty read
           * request.  We need to wait for data to become available.
           */

          else
            {
              /* Enable endpoint RXRDY_TXTK interrupts */

              sam_req_rdenable(epno);
            }
        }

      /* We will not try to accumulate packet data here.  If anything
       * has been received, we will complete the transfer immediately and
       * give the data to the class driver.  The idea is that we will let the
       * receiving be in-charge if incoming buffer.
       */

      else
        {
          /* Return the read request to the class driver. */

          usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
          sam_putreg(USBHS_DEVEPTINT_RXOUTI, SAM_USBHS_DEVEPTIDR(epno));

          /* Get the endpoint type */

          regval = sam_getreg(SAM_USBHS_DEVEPTCFG(epno));
          eptype = regval & USBHS_DEVEPTCFG_EPTYPE_MASK;

          /* Disable endpoint interrupts if not the control endpoint */

          if (eptype != USBHS_DEVEPTCFG_EPTYPE_CTRL)
            {
              sam_req_rddisable(epno);
            }

          /* And complete the request */

          privep->epstate = USBHS_EPSTATE_IDLE;
          sam_req_complete(privep, OK);
        }
    }

  return OK;
}

/****************************************************************************
 *
 * Name: sam_req_cancel
 *
 ****************************************************************************/

static void sam_req_cancel(struct sam_ep_s *privep, int16_t result)
{
  uint32_t regval;
  uint8_t epno;

  /* Disable endpoint interrupts if not endpoint 0 */

  epno = USB_EPNO(privep->ep.eplog);
  if (epno != 0)
    {
      regval = USBHS_DEVINT_DMA(epno) | USBHS_DEVINT_PEP(epno);
      sam_putreg(regval, SAM_USBHS_DEVIDR);
    }

  /* Then complete every queued request with the specified status */

  while (!sam_rqempty(&privep->reqq))
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               (sam_rqpeek(&privep->reqq))->req.xfrd);
      sam_req_complete(privep, result);
    }
}

/****************************************************************************
 *
 * Interrupt Level Processing
 *
 ****************************************************************************/

/****************************************************************************
 *
 * Name: sam_ep0_read
 *
 * Description:
 *   Read a general USB request from the USBHS FIFO
 *
 ****************************************************************************/

static void sam_ep0_read(uint8_t *buffer, size_t buflen)
{
  volatile const uint8_t *fifo;

  /* Retrieve packet from the FIFO */

  fifo = (volatile const uint8_t *)SAM_USBHSRAM_BASE;
  for (; buflen > 0; buflen--)
    {
      *buffer++ = *fifo++;
    }

  MEMORY_SYNC();
}

/****************************************************************************
 *
 * Name: sam_ctrlep_write
 *
 * Description:
 *   Process the next queued write request for a control endpoint.
 *
 ****************************************************************************/

static void sam_ctrlep_write(struct sam_ep_s *privep, const uint8_t *buffer,
                             size_t buflen)
{
  volatile uint8_t *fifo;
  unsigned int epno;

  /* Get the endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* Write packet in the FIFO buffer */

  fifo = (uint8_t *)
    ((uint32_t *)SAM_USBHSRAM_BASE + (EPT_FIFO_SIZE * epno));

  for (; buflen > 0; buflen--)
    {
      *fifo++ = *buffer++;
    }

  MEMORY_SYNC();

  /* Indicate that there is data in the TX packet memory.  This will
   * be cleared when the next NAKIN interrupt is received.
   */

  privep->epstate = USBHS_EPSTATE_SENDING;

  /* The FIFO Control (USBHS_DEVEPTIMRx.FIFOCON) bit and the Read/Write
   * Allowed (USBHS_DEVEPTISRx.RWALL) bit are irrelevant for control
   * endpoints. The user never uses them on these endpoints.
   */

  /* USBHS_DEVEPTISRx.TXINI is cleared by software (by writing a one to
   * the Transmitted IN Data Interrupt Clear bit (USBHS_DEVEPTIDRx.TXINIC)
   * to acknowledge the interrupt, which has no effect on the endpoint
   * FIFO. This acknowledges the interrupt and sends the packet.
   */

  sam_putreg(USBHS_DEVEPTINT_TXINI, SAM_USBHS_DEVEPTIDR(epno));

  /* Clear the NAKIN bit to stop NAKing IN tokens from the host.  We now
   * have data ready to go.
   */

  sam_putreg((USBHS_DEVEPTINT_NAKINI | USBHS_DEVEPTINT_TXINI),
             SAM_USBHS_DEVEPTICR(epno));

  /* Enable the TXIN interrupt on the endpoint */

  sam_putreg(USBHS_DEVEPTINT_TXINI, SAM_USBHS_DEVEPTIER(epno));
}

/****************************************************************************
 *
 * Name: sam_ep_write
 *
 * Description:
 *   Process the next queued write request for a control endpoint.
 *
 ****************************************************************************/

static void sam_ep_write(struct sam_ep_s *privep, const uint8_t *buffer,
                         size_t buflen)
{
  volatile uint8_t *fifo;
  unsigned int epno;

  /* Get the endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* Write packet in the FIFO buffer */

  fifo = (uint8_t *)
    ((uint32_t *)SAM_USBHSRAM_BASE + (EPT_FIFO_SIZE * epno));

  for (; buflen; buflen--)
    {
      *fifo++ = *buffer++;
    }

  MEMORY_SYNC();

  /* Indicate that there is data in the TX packet memory.  This will
   * be cleared when the next data out interrupt is received.
   */

  privep->epstate = USBHS_EPSTATE_SENDING;

  /* Initiate the transfer and configure to receive the transfer complete
   * interrupt.
   */

  sam_ep_fifocon(epno);
}

/****************************************************************************
 *
 * Name: sam_ep0_dispatch
 *
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
          sam_ep_stall(&priv->eplist[EP0].ep, false);
        }
    }
}

/****************************************************************************
 *
 * Name: sam_ep0_setup
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
  uint32_t             regval;
  uint8_t              epno;
  int                  nbytes = 0; /* Assume zero-length packet */
  int                  ret;

  /* Terminate any pending requests */

  sam_req_cancel(ep0, -EPROTO);

  /* Assume NOT stalled; no TX in progress */

  ep0->stalled  = 0;
  ep0->epstate  = USBHS_EPSTATE_IDLE;

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

      /* Let the class implementation handle all non-standar requests */

      sam_ep0_dispatch(priv);
      return;
    }

  /* Handle standard request.  Pick off the things of interest to the
   * USB device controller driver; pass what is left to the class driver
   */

  ep0result = USBHS_EP0SETUP_SUCCESS;
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
            ep0result = USBHS_EP0SETUP_STALL;
          }
        else
          {
            switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
               case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  epno = USB_EPNO(index.b[LSB]);
                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPGETSTATUS),
                          epno);
                  if (epno >= SAM_USBHS_NENDPOINTS)
                    {
                      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPGETSTATUS),
                              epno);
                      ep0result = USBHS_EP0SETUP_STALL;
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
                      ep0result = USBHS_EP0SETUP_STALL;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_IFGETSTATUS),
                          0);
                  response.w = 0;
                  nbytes     = 2; /* Response size: 2 bytes */
                }
                break;

              default:
                {
                  usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETSTATUS),
                          0);
                  ep0result = USBHS_EP0SETUP_STALL;
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
            /* Let the class implementation handle all
             * recipients (except for the endpoint recipient)
             */

            sam_ep0_dispatch(priv);
            ep0result = USBHS_EP0SETUP_DISPATCHED;
          }
        else
          {
            /* Endpoint recipient */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < SAM_USBHS_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = 0;

                ret = sam_ep_stall(&privep->ep, true);
                if (ret < 0)
                  {
                    ep0result = USBHS_EP0SETUP_STALL;
                  }
              }
            else
              {
                usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADCLEARFEATURE), 0);
                ep0result = USBHS_EP0SETUP_STALL;
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
            /* The class driver handles all
            * recipients except recipient=endpoint
            */

            sam_ep0_dispatch(priv);
            ep0result = USBHS_EP0SETUP_DISPATCHED;
          }
        else
          {
            /* Handler recipient=endpoint */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < SAM_USBHS_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = 1;

                ret = sam_ep_stall(&privep->ep, false);
                if (ret < 0)
                  {
                    ep0result = USBHS_EP0SETUP_STALL;
                  }
              }
            else
              {
                usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETFEATURE), 0);
                ep0result = USBHS_EP0SETUP_STALL;
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
            ep0result = USBHS_EP0SETUP_STALL;
          }
        else
          {
            /* Note that setting of the device address will be deferred.  A
             * zero-length packet will be sent and the device address will
             * be set when the zero-length packet transfer completes.
             */

            usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPSETADDRESS),
                    value.w);

            /* Write this address to the USB Address
             * (USBHS_DEVCTRL.UADD) field
             * but do not yet enable (USBHS_DEVCTRL.ADDEN)
             * so the actual address is still 0.
             *
             * USBHS_DEVCTRL.UADD and USBHS_DEVCTRL.ADDEN
             * must not be written all at once.
             */

            regval  = sam_getreg(SAM_USBHS_DEVCTRL);
            regval &= ~(USBHS_DEVCTRL_UADD_MASK | USBHS_DEVCTRL_ADDEN);
            regval |= USBHS_DEVCTRL_UADD(value.w);
            sam_putreg(regval, SAM_USBHS_DEVCTRL);

            ep0result = USBHS_EP0SETUP_ADDRESS;
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
            /* The request seems valid...
             * let the class implementation handle it
             */

            sam_ep0_dispatch(priv);
            ep0result = USBHS_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETSETDESC), 0);
            ep0result = USBHS_EP0SETUP_STALL;
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
            /* The request seems valid...
             * let the class implementation handle it
             */

            sam_ep0_dispatch(priv);
            ep0result = USBHS_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETCONFIG), 0);
            ep0result = USBHS_EP0SETUP_STALL;
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
            /* The request seems valid...
             * let the class implementation handle it.
             * If the class implementation accespts
             * it new configuration, it will
             * call sam_ep_configure() to configure the endpoints.
             */

            sam_ep0_dispatch(priv);
            ep0result = USBHS_EP0SETUP_DISPATCHED;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETCONFIG), 0);
            ep0result = USBHS_EP0SETUP_STALL;
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
        ep0result = USBHS_EP0SETUP_DISPATCHED;
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
        ep0result = USBHS_EP0SETUP_STALL;
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
   * 1a. ep0result == USBHS_EP0SETUP_SUCCESS
   *
   *    The setup request was successfully handled above and a response
   *    packet must be sent (may be a zero length packet).
   *
   * 1b. ep0result == USBHS_EP0SETUP_ADDRESS
   *
   *    A special case is the case where epstate=USBHS_EPSTATE_EP0ADDRESS.
   *    This means that the above processing generated an additional state
   *    where we need to wait until we complete the status phase before
   *    applying the new device address.
   *
   * 2. ep0result == USBHS_EP0SETUP_DISPATCHED;
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
   * 3. ep0result == USBHS_EP0SETUP_STALL;
   *
   *    An error was detected in either the above logic or by the class
   *    implementation logic.
   */

  switch (ep0result)
    {
      case USBHS_EP0SETUP_SUCCESS:
        {
          /* Send the response (might be a zero-length packet) */

          sam_ctrlep_write(ep0, response.b, nbytes);
          ep0->epstate = USBHS_EPSTATE_EP0STATUSIN;
        }
        break;

      case USBHS_EP0SETUP_ADDRESS:
        {
          /* Send the response (might be a zero-length packet) */

          sam_ctrlep_write(ep0, response.b, nbytes);
          ep0->epstate = USBHS_EPSTATE_EP0ADDRESS;
        }
        break;

      case USBHS_EP0SETUP_STALL:
        {
          /* Stall EP0 */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EP0SETUPSTALLED),
                   priv->ctrl.req);

          sam_ep_stall(&priv->eplist[EP0].ep, false);
        }
        break;

      case USBHS_EP0SETUP_DISPATCHED:
      default:
        break;
    }
}

/****************************************************************************
 *
 * Name: sam_dma_interrupt
 *
 * Description:
 *   Handle the USBHS DMA interrupt
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void sam_dma_interrupt(struct sam_usbdev_s *priv, int epno)
{
  struct sam_ep_s  *privep;
  struct sam_req_s *privreq;
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t dmastatus;
  uint8_t *buf;
  int bufcnt;
  int xfrsize;

  /* Not all endpoints support DMA */

  DEBUGASSERT((unsigned)epno < SAM_USBHS_NENDPOINTS &&
              (SAM_EPSET_DMA & SAM_EP_BIT(epno)) != 0);

  /* Get the endpoint structure */

  privep = &priv->eplist[epno];

  /* Get the request from the head of the endpoint request queue */

  privreq = sam_rqpeek(&privep->reqq);
  DEBUGASSERT(privreq);

  /* Disable DMA interrupt to avoid receiving 2 (B_EN and TR_EN) */

  regaddr = SAM_USBHS_DEVDMACTRL(epno);
  regval  = sam_getreg(regaddr);
  regval &= ~(USBHS_DEVDMACTRL_ENDTREN | USBHS_DEVDMACTRL_ENDBEN);
  sam_putreg(regval, regaddr);

  /* Get the result of the DMA operation */

  dmastatus = sam_getreg(SAM_USBHS_DEVDMASTA(epno));
  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPDMAINT), dmastatus);

  /* Disable DMA interrupt to avoid receiving 2 (B_EN and TR_EN) */

  regaddr = SAM_USBHS_DEVDMACTRL(epno);
  regval  = sam_getreg(regaddr);
  regval &= ~(USBHS_DEVDMACTRL_ENDTREN | USBHS_DEVDMACTRL_ENDBEN);
  sam_putreg(regval, regaddr);

  /* Check for end of the buffer.  Set by hardware when the BUFF_COUNT
   * downcount reaches zero.  This could be either an IN or OUT transfer.
   */

  if ((dmastatus & USBHS_DEVDMASTA_ENDBUFFST) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DMAEOB), (uint16_t)dmastatus);

      /* BUFF_COUNT holds the number of untransmitted bytes. BUFF_COUNT is
       * equal to zero in case of good transfer. BUFF_COUNT was set to
       * the 'inflight' count when the DMA started and the BUFF_COUNT has
       * now decremented to zero
       */

      /* This is just debug logic that only does any if USB debug or tracing
       * are enabled.  This just verifies that BUFF_COUNT is zero.
       */

      bufcnt = (dmastatus & USBHS_DEVDMASTA_BUFCNT_MASK)
                >> USBHS_DEVDMASTA_BUFCNT_SHIFT;

      if (bufcnt != 0)
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACERR_REMAINING), bufcnt);
        }

      /* Were we sending?  Or receiving? */

      if (privep->epstate == USBHS_EPSTATE_SENDING ||
          privep->epstate == USBHS_EPSTATE_SENDING_DMA)
        {
          uint32_t nbusybk;
          uint32_t byct;

          /* This is an IN endpoint.  Continuing processing the write
           * request.
           */

          DEBUGASSERT(USB_ISEPIN(privep->ep.eplog));

          /* Assume that will wait for the NBUSYBKS interrupt.  Enable it
           * now.  We do this PRIOR to sampling the BYCT and BUSBK
           * fields to avoid the race condition that would occur if
           * the interrupt were enabled afterward.
           */

          sam_putreg(USBHS_DEVEPTINT_NBUSYBKI, SAM_USBHS_DEVEPTIER(epno));

          /* Have all of the bytes in the FIFO been transmitted to the
           * host?
           *
           * BYCT == 0    Means that all of the data has been transferred
           *              out of the FIFO.
           *              Warning: This field may be updated one clock cycle
           *              after the RWALL bit changes, so the user should not
           *              poll this field as an interrupt bit.
           * NBUSYBK == 0 Indicates that all banks that have been sent to
           *              the host.
           */

          regval  = sam_getreg(SAM_USBHS_DEVEPTISR(epno));
          byct    = (regval & USBHS_DEVEPTISR_BYCT_MASK) >>
                    USBHS_DEVEPTISR_BYCT_SHIFT;
          nbusybk = (regval & USBHS_DEVEPTISR_NBUSYBK_MASK) >>
                    USBHS_DEVEPTISR_NBUSYBK_SHIFT;

          if (byct > 0 || nbusybk > 0)
            {
              /* Not all of the data has been sent to the host.
               * A NBUSYBKE interrupt will be generated later.
               * It has already been enabled.
               * Now wait for the transfer to complete.
               */

              privep->epstate = USBHS_EPSTATE_NBUSYBK;
            }
          else
            {
              /* All bytes have been sent to the host.  We must call
               * sam_req_write() now in the IDLE state with the number of
               * bytes transferred in 'inflight'.  There must not be a
               * pending TXIN interrupt when sam_req_write() is called.
               */

              sam_putreg(USBHS_DEVEPTINT_NBUSYBKI,
                        SAM_USBHS_DEVEPTIDR(epno));
              sam_putreg(USBHS_DEVEPTINT_TXINI, SAM_USBHS_DEVEPTICR(epno));

              privep->epstate = USBHS_EPSTATE_IDLE;
              sam_req_write(priv, privep);
            }
        }
      else if (privep->epstate == USBHS_EPSTATE_RECEIVING)
        {
          /* privreg->inflight holds the total transfer size */

          xfrsize           = privreq->inflight;
          privreq->inflight = 0;

          /* This is an OUT endpoint.  Invalidate the data cache for
           * region that just completed DMA. This will force the
           * buffer data to be reloaded from RAM when it is accessed.
           *
           * REVISIT: If the buffer is not aligned to the cacheline
           * size the cached data might be lost at the boundaries.
           */

          DEBUGASSERT(USB_ISEPOUT(privep->ep.eplog));
          buf = &privreq->req.buf[privreq->req.xfrd];
          up_invalidate_dcache((uintptr_t)buf, (uintptr_t)buf + xfrsize);

          /* Complete this transfer, return the request to the class
           * implementation, and try to start the next, queue read request.
           * We must call sam_req_read in the IDLE state, 'inflight' is
           * ignored (should be zero) and the transfer size is passed as
           * an argument to sam_req_read().
           */

          privep->epstate = USBHS_EPSTATE_IDLE;
          sam_req_read(priv, privep, xfrsize);
        }
      else
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEOBSTATE), bufcnt);
        }
    }

  /* Check for end of channel transfer. END_TR_ST is set by hardware when
   * the last packet transfer is complete if END_TR_EN is set in the
   * DMACONTROL register.  The request is complete.
   *
   * "Used for OUT transfers only.
   *
   *    "0 = USB end of transfer is ignored.
   *    "1 = USBHS device can put an end to the current buffer transfer.
   *
   * "When set, a BULK or INTERRUPT short packet or the last packet of
   *  an ISOCHRONOUS (micro) frame (DATAX) will close the current buffer
   *  and the USBHS_DMASTATUSx register END_TR_ST flag will be raised."
   */

  else if ((dmastatus & USBHS_DEVDMASTA_ENDTRST) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DMAEOC), (uint16_t)dmastatus);
      DEBUGASSERT(privep->epstate == USBHS_EPSTATE_RECEIVING &&
                  USB_ISEPOUT(privep->ep.eplog));

      /* Get the number of bytes transferred from the DMA status.
       *
       * BUFF_COUNT holds the number of untransmitted bytes. In this case,
       * BUFF_COUNT should not be zero.  BUFF_COUNT was set to the
       * 'inflight' count when the DMA started so the difference will
       * give us the actual size of the transfer.
       */

      bufcnt             = ((dmastatus & USBHS_DEVDMASTA_BUFCNT_MASK)
                           >> USBHS_DEVDMASTA_BUFCNT_SHIFT);
      xfrsize            = privreq->inflight - bufcnt;
      privreq->inflight  = 0;

      /* Invalidate the data cache for region that just completed DMA.
       * This will force the buffer data to be reloaded from RAM.
       *
       * REVISIT: If the buffer is not aligned to the cacheline size the
       * cached data might be lost at the boundaries.
       */

      buf = &privreq->req.buf[privreq->req.xfrd];
      up_invalidate_dcache((uintptr_t)buf, (uintptr_t)buf + xfrsize);

      /* Complete this transfer, return the request to the class
       * implementation, and try to start the next, queue read request.
       */

      privep->epstate = USBHS_EPSTATE_IDLE;
      sam_req_read(priv, privep, xfrsize);
    }
  else
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_DMAERR), (uint16_t)dmastatus);

      /* Return the request buffer to the class implementation with the I/O
       * error indication.
       */

      sam_req_complete(privep, -EIO);
    }
}
#endif

/****************************************************************************
 *
 * Name: sam_ep_interrupt
 *
 * Description:
 *   Handle the USBHS endpoint interrupt
 *
 ****************************************************************************/

static void sam_ep_interrupt(struct sam_usbdev_s *priv, int epno)
{
  struct sam_ep_s *privep;
  uint32_t eptisr;
  uint32_t eptype;
  uint32_t regval;
  uint16_t pktsize;

  DEBUGASSERT((unsigned)epno < SAM_USBHS_NENDPOINTS);

  /* Get the endpoint structure */

  privep = &priv->eplist[epno];

  /* Get the endpoint status */

  eptisr = sam_getreg(SAM_USBHS_DEVEPTISR(epno));
  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPINT), eptisr);

  /* Get the endpoint type */

  regval = sam_getreg(SAM_USBHS_DEVEPTCFG(epno));
  eptype = regval & USBHS_DEVEPTCFG_EPTYPE_MASK;

#ifdef CONFIG_USBDEV_DMA
  /* Check for IN DMA packet sent and NBUSYBKS transitioning to zero.  This
   * is the USBHS_DEVEPTINT_NBUSYBKI interrupt.  There is no indication of
   * the pending USBHS_DEVEPTINT_NBUSYBKI in the status register.  We depend
   * on (a) NBUSYBKS == 0, and (c) state == USBHS_EPSTATE_NBUSYBK.
   */

  if ((eptisr & USBHS_DEVEPTISR_NBUSYBK_MASK) == 0 &&
      privep->epstate == USBHS_EPSTATE_NBUSYBK)
    {
      /* Clear the pending NBUSYBKS (and TXIN) interrupts */

      sam_putreg(USBHS_DEVEPTINT_TXINI | USBHS_DEVEPTINT_NBUSYBKI,
                 SAM_USBHS_DEVEPTICR(epno));

      /* Disable further NBUSYBKS interrupts */

      sam_putreg(USBHS_DEVEPTINT_NBUSYBKI, SAM_USBHS_DEVEPTIDR(epno));

      /* Continue/resume processing the write requests. */

      privep->epstate = USBHS_EPSTATE_IDLE;
      sam_req_write(priv, privep);
    }
  else
#endif

  /* IN packet sent */

  if ((eptisr & USBHS_DEVEPTINT_TXINI) != 0 &&
      (sam_getreg(SAM_USBHS_DEVEPTIMR(epno)) & USBHS_DEVEPTINT_TXINI) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_TXINI), (uint16_t)eptisr);

      /* Sending state.  This is the completion of a "normal" write request
       * transfer.  In this case, we need to resume request processing in
       * order to send the next outgoing packet.
       */

      if (privep->epstate == USBHS_EPSTATE_SENDING ||
          privep->epstate == USBHS_EPSTATE_EP0STATUSIN)
        {
          /* A control endpoint clears the TXIN bit after new data is written
           * to the fifo inside the function "sam_ctrlep_write"
           * (sam_req_write -> sam_req_wrsetup -> sam_ctrlep_write).
           * All other Endpoints needs to clear the bit here.
           *
           * REVISIT: normally all other endpoints also have to reset the bit
           *          at a later point.
           *
           * Continue/resume processing the write requests.
           */

           if (eptype != USBHS_DEVEPTCFG_EPTYPE_CTRL)
          {
               sam_putreg(USBHS_DEVEPTINT_TXINI, SAM_USBHS_DEVEPTICR(epno));
          }

          privep->epstate = USBHS_EPSTATE_IDLE;
          sam_req_write(priv, privep);
        }

      /* Setting of the device address is a special case.  The address was
       * obtained when a preceding SETADDRESS SETUP command was processed.
       * But the address is not set until the final SETUP status phase
       * completes.  This interrupt indicates the completion of that status
       * phase and now we set the address.
       */

      else if (privep->epstate == USBHS_EPSTATE_EP0ADDRESS)
        {
          DEBUGASSERT(epno == EP0);

          /* Enable the address previously set in the SETUP processing.
           * USBHS_DEVCTRL.UADD and USBHS_DEVCTRL.ADDEN must not be written
           * all at once.
           */

          regval  = sam_getreg(SAM_USBHS_DEVCTRL);
          regval |= USBHS_DEVCTRL_ADDEN;
          sam_putreg(regval, SAM_USBHS_DEVCTRL);

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_ADDRESSED),
                   (regval & USBHS_DEVCTRL_UADD_MASK) <<
                   USBHS_DEVCTRL_UADD_SHIFT);

          /* Go to the addressed state.  EP0 is now IDLE. */

          priv->devstate  = USBHS_DEVSTATE_ADDRESSED;
          privep->epstate = USBHS_EPSTATE_IDLE;

          /* Acknowledge then disable the further TXIN interrupts on EP0. */

          sam_putreg(USBHS_DEVEPTINT_TXINI, SAM_USBHS_DEVEPTICR(epno));
          sam_putreg(USBHS_DEVEPTINT_TXINI, SAM_USBHS_DEVEPTIDR(epno));
        }
      else
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_TXINERR), privep->epstate);
          sam_putreg(USBHS_DEVEPTINT_TXINI, SAM_USBHS_DEVEPTICR(epno));
          sam_putreg(USBHS_DEVEPTINT_TXINI, SAM_USBHS_DEVEPTIDR(epno));
        }
    }

  /* OUT packet received */

  if ((eptisr & USBHS_DEVEPTINT_RXOUTI) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_RXRDY), (uint16_t)eptisr);

      /* Are we receiving data for a read request? */

      if (privep->epstate == USBHS_EPSTATE_RECEIVING)
        {
          /* Yes, get the size of the packet that we just received */

          pktsize = (uint16_t)
            ((eptisr & USBHS_DEVEPTISR_BYCT_MASK) >>
            USBHS_DEVEPTISR_BYCT_SHIFT);

          /* And continue processing the read request, clearing RXOUT in
           * order to receive more data.
           */

          privep->epstate = USBHS_EPSTATE_IDLE;
          sam_req_read(priv, privep, pktsize);
          sam_putreg(USBHS_DEVEPTINT_RXOUTI, SAM_USBHS_DEVEPTICR(epno));
        }

      /* Did we just receive the data associated with an OUT SETUP command? */

      else if (privep->epstate == USBHS_EPSTATE_EP0DATAOUT)
        {
          uint16_t len;

          /* Yes.. back to the IDLE state */

          privep->epstate = USBHS_EPSTATE_IDLE;

          /* Get the size of the packet that we just received */

          pktsize = (uint16_t)
            ((eptisr & USBHS_DEVEPTISR_BYCT_MASK) >>
            USBHS_DEVEPTISR_BYCT_SHIFT);

          /* Get the size that we expected to receive */

          len = GETUINT16(priv->ctrl.len);
          if (len == pktsize)
            {
              /* Copy the OUT data from the EP0 FIFO
               * into a special EP0 buffer
               * and clear RXOUT in order to receive more data.
               */

              sam_ep0_read(priv->ep0out, len);
              sam_putreg(USBHS_DEVEPTINT_RXOUTI, SAM_USBHS_DEVEPTICR(epno));

              /* And handle the EP0 SETUP now. */

              sam_ep0_setup(priv);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EP0SETUPOUTSIZE),
                      pktsize);

              /* STALL and discard received data. */

              sam_ep_stall(&privep->ep, false);
              sam_putreg(USBHS_DEVEPTINT_STALLRQI,
                        SAM_USBHS_DEVEPTICR(epno));
            }
        }
      else
        {
          /* Check if ACK received on a Control EP */

          if (eptype == USBHS_DEVEPTCFG_EPTYPE_CTRL &&
              (eptisr & USBHS_DEVEPTISR_BYCT_MASK) == 0)
            {
            }

          /* Data has been STALLed */

          else if ((eptisr & USBHS_DEVEPTINT_STALLEDI) != 0)
            {
            }

          /* NAK the data */

          else
            {
              sam_putreg(USBHS_DEVINT_PEP(epno), SAM_USBHS_DEVIDR);
            }

          /* Acknowledge the interrupt. */

          sam_putreg(USBHS_DEVEPTINT_RXOUTI, SAM_USBHS_DEVEPTICR(epno));
        }
    }

  /* STALL sent */

  if ((eptisr & USBHS_DEVEPTINT_STALLEDI) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_STALLSNT), (uint16_t)eptisr);

      /* Acknowledge */

      sam_putreg(USBHS_DEVEPTINT_STALLEDI, SAM_USBHS_DEVEPTICR(epno));

      /* ISO error */

      if (eptype == USBHS_DEVEPTCFG_EPTYPE_ISO)
        {
          privep->epstate = USBHS_EPSTATE_IDLE;
          sam_req_complete(privep, -EIO);
        }

      /* If EP is not halted, clear STALL */

      else if (privep->epstate != USBHS_EPSTATE_STALLED)
        {
          sam_putreg(USBHS_DEVEPTINT_STALLEDI, SAM_USBHS_DEVEPTIDR(epno));
        }
    }

  /* SETUP packet received */

  if ((eptisr & USBHS_DEVEPTINT_RXSTPI) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_RXSETUP), (uint16_t)eptisr);

      /* If a request transfer was pending, complete it.
       * Handle the case where during the status phase
       * of a control write transfer, the host receives
       * the device ZLP and ack it, but the ack is
       * not received by the device
       */

      if (privep->epstate == USBHS_EPSTATE_RECEIVING ||
          privep->epstate == USBHS_EPSTATE_SENDING ||
          privep->epstate == USBHS_EPSTATE_SENDING_DMA)
        {
          sam_req_complete(privep, -EPROTO);
        }

      /* ISO Err Flow */

      if (eptype == USBHS_DEVEPTCFG_EPTYPE_ISO)
        {
          /* Acknowledge setup packet */

          sam_putreg(USBHS_DEVEPTINT_RXSTPI, SAM_USBHS_DEVEPTICR(epno));
        }
      else
        {
          uint16_t len;

          /* Copy setup data from the EP0 FIFO into the driver structure. */

          sam_ep0_read((uint8_t *)&priv->ctrl, USB_SIZEOF_CTRLREQ);

          /* Acknowledge setup packet */

          sam_putreg(USBHS_DEVEPTINT_RXSTPI, SAM_USBHS_DEVEPTICR(epno));

          /* Check for a SETUP IN transaction */

          len = GETUINT16(priv->ctrl.len);
          if (USB_REQ_ISOUT(priv->ctrl.type) && len > 0)
            {
              /* Yes.. then we have to wait for the OUT data phase to
               * complete before processing the SETUP command.
               */

              usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPOUT),
                      priv->ctrl.req);
              privep->epstate = USBHS_EPSTATE_EP0DATAOUT;
            }
          else
            {
              /* This is an SETUP IN command (or a SETUP IN with no data).
               * Handle the EP0 SETUP now.
               */

              usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPIN), len);
              privep->epstate = USBHS_EPSTATE_IDLE;
              sam_ep0_setup(priv);
            }
        }
    }
}

/****************************************************************************
 *
 * Name: sam_usbhs_interrupt
 *
 * Description:
 *   Handle the USBHS interrupt
 *
 ****************************************************************************/

static int sam_usbhs_interrupt(int irq, void *context, void *arg)
{
  /* For now there is only one USB controller,
   * but we will always refer to it using a pointer
   * to make any future ports to multiple
   * USBHS controllers easier.
   */

  struct sam_usbdev_s *priv = &g_usbhs;
  uint32_t devisr;
  uint32_t pending;
  uint32_t regval;
  int i;

  /* Get the set of pending interrupts */

  devisr  = sam_getreg(SAM_USBHS_DEVISR);
  usbtrace(TRACE_INTENTRY(SAM_TRACEINTID_INTERRUPT), devisr);

  regval  = sam_getreg(SAM_USBHS_DEVIMR);
  pending = devisr & regval;

  /* Handle all pending USBHS interrupts (and new interrupts that become
   * pending)
   */

  while (pending)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_PENDING), (uint16_t)pending);

      /* Suspend, treated last */

      if (pending == USBHS_DEVINT_SUSPD)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_INTSUSPD),
                  (uint16_t)pending);

          /* Un-freeze the clock */

          regval  = sam_getreg(SAM_USBHS_CTRL);
          regval &= ~USBHS_CTRL_FRZCLK;
          sam_putreg(regval, SAM_USBHS_CTRL);

          /* Disable suspend interrupts */

          sam_putreg(USBHS_DEVINT_SUSPD, SAM_USBHS_DEVIDR);

          /* Enable wakeup interrupts */

          sam_putreg(USBHS_DEVINT_WAKEUP | USBHS_DEVINT_EORSM,
                    SAM_USBHS_DEVIER);

          /* Acknowledge the suspend interrupt */

          sam_putreg(USBHS_DEVINT_SUSPD | USBHS_DEVINT_WAKEUP,
                    SAM_USBHS_DEVICR);

          /* Inform board logic that USB is suspended */

          sam_suspend(priv);

          /* Re-freeze the clock */

          regval |= USBHS_CTRL_FRZCLK;
          sam_putreg(regval, SAM_USBHS_CTRL);
        }

      /* SOF interrupt */

      else if ((pending & USBHS_DEVINT_SOF) != 0)
        {
          /* Acknowledge interrupt */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_INTSOF),
                  (uint16_t)pending);
          sam_putreg(USBHS_DEVINT_SOF, SAM_USBHS_DEVICR);
        }

      /* MSOF interrupt */

      else if ((pending & USBHS_DEVINT_MSOF) != 0)
        {
          /* Acknowledge interrupt */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_INTMSOF),
                  (uint16_t)pending);
          sam_putreg(USBHS_DEVINT_MSOF, SAM_USBHS_DEVICR);
        }

      /* Resume */

      else if ((pending & USBHS_DEVINT_WAKEUP) != 0 ||
               (pending & USBHS_DEVINT_EORSM) != 0)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_WAKEUP),
                  (uint16_t)pending);
          sam_resume(priv);

          /* Un-freeze the clock */

          regval  = sam_getreg(SAM_USBHS_CTRL);
          regval &= ~USBHS_CTRL_FRZCLK;
          sam_putreg(regval, SAM_USBHS_CTRL);

          /* Acknowledge interrupt */

          sam_putreg(USBHS_DEVINT_WAKEUP | USBHS_DEVINT_EORSM |
                    USBHS_DEVINT_SUSPD, SAM_USBHS_DEVICR);

          /* Disable wakeup interrupts */

          sam_putreg(USBHS_DEVINT_WAKEUP, SAM_USBHS_DEVIDR);

          /* Enable suspend interrupts and clear */

          sam_putreg(USBHS_DEVINT_EORSM | USBHS_DEVINT_SUSPD,
                    SAM_USBHS_DEVIER);
        }

      /* End of Reset.
       *
       * The USB bus reset is managed by hardware.
       * It is initiated by a connected host.
       * When a USB reset is detected on the USB line, the following
       * operations are performed by the controller:
       *
       *   - All endpoints are disabled, except the default control endpoint.
       *   - The default control endpoint is reset
       *   - The data toggle sequence of the default control
       *     endpoint is cleared.
       *   - At the end of the reset process,
       *     the End of Reset (USBHS_DEVISR.EORST) bit is set.
       *   - During a reset, the USBHS automatically switches
       *     to High-speed modeif the host is High-speed-capable
       *     (the reset is called High-speed reset).
       *     The user should observe the USBHS_SR.SPEED field to know
       *     the speed running at the end of the reset
       *     (USBHS_DEVISR.EORST = 1).
       */

      else if ((pending & USBHS_DEVINT_EORST) != 0)
        {
          /* Sample the USBHS SR register at the time of the EORST event. */

          regval = sam_getreg(SAM_USBHS_SR);
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_ENDRESET), regval);

          /* Acknowledge the interrupt, clear pending wakeup and suspend
           * status as we..
           */

          sam_putreg(USBHS_DEVINT_WAKEUP | USBHS_DEVINT_SUSPD |
                     USBHS_DEVINT_EORST,
                     SAM_USBHS_DEVICR);

          /* Enable suspend interrupts */

          sam_putreg(USBHS_DEVINT_SUSPD, SAM_USBHS_DEVIER);

          /* Handle the reset (will select full speed mode) */

          sam_reset(priv);

          /* Get the correct speed mode reported by the hardware */

          switch (regval & USBHS_SR_SPEED_MASK)
            {
              default:
              case USBHS_SR_SPEED_FULL:
                priv->usbdev.speed = USB_SPEED_FULL;
                break;

              case USBHS_SR_SPEED_HIGH:
                priv->usbdev.speed = USB_SPEED_HIGH;
                break;

              case USBHS_SR_SPEED_LOW:
                priv->usbdev.speed = USB_SPEED_LOW;
                break;
            }

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_SPEED),
                   priv->usbdev.speed);
        }

      /* Upstream resume */

      else if ((pending & USBHS_DEVINT_UPRSM) != 0)
        {
          /* Acknowledge interrupt */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_UPSTRRES),
                  (uint16_t)pending);
          sam_putreg(USBHS_DEVINT_UPRSM, SAM_USBHS_DEVICR);
        }

#ifdef CONFIG_USBDEV_DMA
      /* Endpoint DMA interrupts */

      else if ((pending & USBHS_DEVINT_DMA_MASK) != 0)
        {
          /* Each endpoint DMA ineterrupt is cleared when the
           * USBHS_DEVDMASTATUSx interrupt source is cleared.
           */

          /* Process each pending endpoint DMA interrupt */

          for (i = 1; i <= SAM_USBHS_NDMACHANNELS; i++)
            {
              /* Is there a DMA interrupt pending for endpoint i? */

              if ((pending & USBHS_DEVINT_DMA(i)) != 0)
                {
                  /* Yes.. process the endpoint i DMA interrupt */

                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DMA), (uint16_t)i);
                  sam_dma_interrupt(priv, i);
                }
            }
        }
#endif

      /* Endpoint Interrupts */

      else if ((pending & USBHS_DEVINT_PEP_MASK) != 0)
        {
          /* Each endpoint interrupt is cleared when the interrupt source
           * is serviced.
           */

          /* Process each pending endpoint interrupt */

          for (i = 0; i < SAM_USBHS_NENDPOINTS; i++)
            {
              /* Is there an interrupt pending for endpoint i? */

              if ((pending & USBHS_DEVINT_PEP(i)) != 0)
                {
                  /* Yes.. process the endpoint i interrupt */

                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP), (uint16_t)i);
                  sam_ep_interrupt(priv, i);
                }
            }
        }

      /* Re-sample the set of pending interrupts */

      devisr  = sam_getreg(SAM_USBHS_DEVISR);
      regval  = sam_getreg(SAM_USBHS_DEVIMR);
      pending = devisr & regval;
    }

  usbtrace(TRACE_INTEXIT(SAM_TRACEINTID_INTERRUPT), devisr);
  MEMORY_SYNC();
  return OK;
}

/****************************************************************************
 *
 * Suspend/Resume Helpers
 *
 ****************************************************************************/

/****************************************************************************
 *
 * Name: sam_suspend
 *
 ****************************************************************************/

static void sam_suspend(struct sam_usbdev_s *priv)
{
  /* Don't do anything if the device is already suspended */

  if (priv->devstate != USBHS_DEVSTATE_SUSPENDED)
    {
      /* Notify the class driver of the suspend event */

      if (priv->driver)
        {
          CLASS_SUSPEND(priv->driver, &priv->usbdev);
        }

      /* Switch to the Suspended state */

      priv->prevstate = priv->devstate;
      priv->devstate  = USBHS_DEVSTATE_SUSPENDED;

      /* Let the board-specific logic know that we have entered the
       * suspend state.  This may trigger additional reduced power
       * consumption measures.
       */

      sam_usbsuspend((struct usbdev_s *)priv, false);
    }
}

/****************************************************************************
 *
 * Name: sam_resume
 *
 ****************************************************************************/

static void sam_resume(struct sam_usbdev_s *priv)
{
  uint32_t regval;

  /* This function is called when either
   * (1) a WKUP interrupt is received from
   * the host PC, or
   * (2) the class device implementation calls the wakeup()
   * method.
   */

  /* Don't do anything if the device was not suspended */

  if (priv->devstate == USBHS_DEVSTATE_SUSPENDED)
    {
      /* Un-freeze clocking */

      regval  = sam_getreg(SAM_USBHS_CTRL);
      regval &= ~USBHS_CTRL_FRZCLK;
      sam_putreg(regval, SAM_USBHS_CTRL);

      /* Revert to the previous state */

      priv->devstate = priv->prevstate;

      /* Restore full power --
       * whatever that means forSC this particular board
       */

      /* Restore full power --
       * whatever that means for this particular board
       */

      sam_usbsuspend((struct usbdev_s *)priv, true);

      /* Notify the class driver of the resume event */

      if (priv->driver)
        {
          CLASS_RESUME(priv->driver, &priv->usbdev);
        }
    }
}

/****************************************************************************
 *
 * Endpoint Helpers
 *
 ****************************************************************************/

/****************************************************************************
 *
 * Name: sam_ep_reset
 *
 * Description:
 *   Reset and disable a set of endpoints.
 *
 ****************************************************************************/

static void sam_ep_reset(struct sam_usbdev_s *priv, uint8_t epno)
{
  struct sam_ep_s *privep = &priv->eplist[epno];
  uint32_t regval;

  /* Disable endpoint interrupt */

  sam_putreg(USBHS_DEVINT_PEP(epno), SAM_USBHS_DEVIDR);

  /* Clear all pending interrupts */

  sam_putreg(USBHS_DEVEPTICR_ALLINTS, SAM_USBHS_DEVEPTICR(epno));

  /* Set DMA control register to a defined state */

  if ((SAM_EPSET_DMA & SAM_EP_BIT(epno)) != 0)
    {
      sam_putreg(0, SAM_USBHS_DEVDMACTRL(epno));
    }

  /* Cancel any queued requests.  Since they are cancelled with status
   * -ESHUTDOWN, then will not be requeued until the configuration is reset.
   * NOTE:  This should not be necessary... the CLASS_DISCONNECT above
   * should result in the class implementation calling sam_ep_disable
   * for each of its configured endpoints.
   */

  sam_req_cancel(privep, -ESHUTDOWN);

  /* Put the endpoint in reset */

  regval = sam_getreg(SAM_USBHS_DEVEPT);
  regval |= USBHS_DEVEPT_EPRST(epno);
  sam_putreg(regval, SAM_USBHS_DEVEPT);

  /* Reset endpoint status */

  privep->epstate   = USBHS_EPSTATE_DISABLED;
  privep->stalled   = false;
  privep->halted    = false;
  privep->zlpneeded = false;
  privep->zlpsent   = false;
  privep->bank      = 0;

  /* Take the endpoint out of reset */

  regval &= ~USBHS_DEVEPT_EPRST(epno);
  sam_putreg(regval, SAM_USBHS_DEVEPT);
}

/****************************************************************************
 *
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
       epno < SAM_USBHS_NENDPOINTS && epset != 0;
       epno++, bit <<= 1)
    {
      /* Is this endpoint in the set? */

      if ((epset & bit) != 0)
        {
           /* Yes.. reset it */

           sam_ep_reset(priv, epno);
           epset &= ~bit;
        }
    }
}

/****************************************************************************
 *
 * Name: sam_ep_reserve
 *
 * Description:
 *   Find an un-reserved endpoint number and reserve it for the caller.
 *
 ****************************************************************************/

static inline struct sam_ep_s *
sam_ep_reserve(struct sam_usbdev_s *priv, uint16_t epset)
{
  struct sam_ep_s *privep = NULL;
  irqstate_t flags;
  int epndx = 0;

  flags = enter_critical_section();
  epset &= priv->epavail;
  if (epset)
    {
      /* Select the lowest bit in the set of matching, available endpoints
       * (skipping EP0)
       */

      for (epndx = 1; epndx < SAM_USBHS_NENDPOINTS; epndx++)
        {
          uint16_t bit = SAM_EP_BIT(epndx);
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
 *
 * Name: sam_ep_unreserve
 *
 * Description:
 *   The endpoint is no long in-used.  It will be un-reserved and can be
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
 *
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
  struct sam_usbdev_s *priv;
  uintptr_t regaddr;
  uint32_t regval;
  uint16_t maxpacket;
  uint8_t epno;
  uint8_t eptype;
  uint8_t nbtrans;
  bool dirin;

  uinfo("len: %02x type: %02x addr: %02x attr: %02x "
        "maxpacketsize: %02x %02x interval: %02x\n",
        desc->len, desc->type, desc->addr, desc->attr,
        desc->mxpacketsize[0],  desc->mxpacketsize[1],
        desc->interval);

  /* Decode the endpoint descriptor */

  epno      = USB_EPNO(desc->addr);
  dirin     = (desc->addr & USB_DIR_MASK) == USB_REQ_DIR_IN;
  eptype    = (desc->attr & USB_EP_ATTR_XFERTYPE_MASK) >>
              USB_EP_ATTR_XFERTYPE_SHIFT;
  maxpacket = GETUINT16(desc->mxpacketsize);
  nbtrans   = 1;

  /* Initialize the endpoint structure */

  priv             = privep->dev;
  privep->ep.eplog = desc->addr;              /* Includes direction */
  privep->epstate  = USBHS_EPSTATE_IDLE;
  privep->bank     = SAM_USBHS_NBANKS(epno);

  /* Special case maxpacket handling for high-speed endpoints */

  if (priv->usbdev.speed == USB_SPEED_HIGH)
    {
      /* HS Interval, 125us */

      /* MPS: Bits 12:11 specify NB_TRANS, as USB 2.0 Spec. */

      nbtrans = ((maxpacket >> 11) & 3);
      if (nbtrans == 3)
        {
          nbtrans = 1;
        }
      else
        {
          nbtrans++;
        }

      /* nbtrans = 0: Reserved to endpoint that does not have the high-
       *              bandwidth isochronous capability.
       * nbtrans = 1: Default value: one transaction per microframe.
       * nbtrans = 2: Two transactions per microframe. This endpoint
       *              should be configured as double-bank.
       * nbtrans = 3  Three transactions per microframe. This endpoint
       *              should be configured as triple-bank
       */

      if (privep->bank < nbtrans)
        {
          nbtrans = privep->bank;
        }

      /* Mask, bit 10..0 is the max packet size */

       maxpacket &= 0x7ff;
    }

  privep->ep.maxpacket = maxpacket;

  /* Initialize the endpoint hardware */

  /* Disable the endpoint */

  regval  = sam_getreg(SAM_USBHS_DEVEPT);
  regval &= ~USBHS_DEVEPT_EPEN(epno);
  sam_putreg(regval, SAM_USBHS_DEVEPT);

  sam_putreg(USBHS_DEVEPTIDR_ALLINTS, SAM_USBHS_DEVEPTIDR(epno));

  /* Clear toggle and stall indications */

  sam_putreg(USBHS_DEVEPTISR_DTSEQ_MASK | USBHS_DEVEPTINT_STALLEDI,
             SAM_USBHS_DEVEPTICR(epno));

  /* Reset the endpoint */

  regval  = sam_getreg(SAM_USBHS_DEVEPT);
  regval |= USBHS_DEVEPT_EPRST(epno);
  sam_putreg(regval, SAM_USBHS_DEVEPT);

  regval &= ~USBHS_DEVEPT_EPRST(epno);
  sam_putreg(regval, SAM_USBHS_DEVEPT);

  /* If this is EP0, disable interrupts now */

  if (eptype == USB_EP_ATTR_XFER_CONTROL)
    {
      sam_putreg(USBHS_DEVINT_PEP(epno), SAM_USBHS_DEVIDR);
    }

  /* Enable the endpoint */

  regval  = sam_getreg(SAM_USBHS_DEVEPT);
  regval |= USBHS_DEVEPT_EPEN(epno);
  sam_putreg(regval, SAM_USBHS_DEVEPT);

  /* Configure the endpoint */

  regval = USBHS_DEVEPTCFG_ALLOC |
           USBHS_DEVEPTCFG_EPDIR(dirin) |
           USBHS_DEVEPTCFG_EPTYPE(eptype) |
           USBHS_DEVEPTCFG_EPBK(privep->bank) |
           USBHS_DEVEPTCFG_NBTRANS(nbtrans);

  if (maxpacket <= 8)
    {
      regval |= USBHS_DEVEPTCFG_EPSIZE_8;
    }
  else if (maxpacket <= 16)
    {
      regval |= USBHS_DEVEPTCFG_EPSIZE_16;
    }
  else if (maxpacket <= 32)
    {
      regval |= USBHS_DEVEPTCFG_EPSIZE_32;
    }
  else if (maxpacket <= 64)
    {
      regval |= USBHS_DEVEPTCFG_EPSIZE_64;
    }
  else if (maxpacket <= 128)
    {
      regval |= USBHS_DEVEPTCFG_EPSIZE_128;
    }
  else if (maxpacket <= 256)
    {
      regval |= USBHS_DEVEPTCFG_EPSIZE_256;
    }
  else if (maxpacket <= 512)
    {
      regval |= USBHS_DEVEPTCFG_EPSIZE_512;
    }
  else if (maxpacket <= 1024)
    {
      regval |= USBHS_DEVEPTCFG_EPSIZE_1024;
    }
  else
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPTYPE), eptype);
      DEBUGPANIC();
      regval |= USBHS_DEVEPTCFG_EPSIZE_8;
    }

  regaddr = SAM_USBHS_DEVEPTCFG(epno);
  sam_putreg(regval, regaddr);

  /* Verify that the CFGOK flag is set. This flag is set if the
   * endpoint size and the number of banks are correct compared to
   * the FIFO maximum capacity and the maximum number of allowed banks.
   */

  regaddr = SAM_USBHS_DEVEPTISR(epno);
  if ((sam_getreg(regaddr) & USBHS_DEVEPTISR_CFGOK) == 0)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_NCFGOK), epno);
      return -EINVAL;
    }

  /* Enable the endpoint.  The way that the endpoint is enabled depends of
   * if the endpoint supports DMA transfers or not.
   */

  if ((SAM_EPSET_DMA & SAM_EP_BIT(epno)) != 0)
    {
      /* Select AUTO_VALID so that the hardware will manage FIFCON. */

      regaddr = SAM_USBHS_DEVEPTCFG(epno);
      regval |= USBHS_DEVEPTCFG_AUTOSW;
      sam_putreg(regval, SAM_USBHS_DEVEPTCFG(epno));
    }
  else
    {
      /* No DMA... Software will manage FIFOCON. */

      regval = USBHS_DEVEPTINT_KILLBKI | USBHS_DEVEPTINT_RXSTPI;
      sam_putreg(regval, SAM_USBHS_DEVEPTIER(epno));
    }

  /* If this is EP0, enable interrupts now */

  if (eptype == USB_EP_ATTR_XFER_CONTROL)
    {
      /* Enable the endpoint 0 RX interrupts */

      regaddr = SAM_USBHS_DEVEPTIER(epno);
      regval  = USBHS_DEVEPTINT_RXOUTI | USBHS_DEVEPTINT_RXSTPI;
      sam_putreg(regval, regaddr);

      /* Enable endpoint 0 general interrupts */

      sam_putreg(USBHS_DEVINT_PEP(epno), SAM_USBHS_DEVIER);
    }
#ifdef CONFIG_USBDEV_DMA
  else
    {
      /* Enable automatic bank switching */

      regaddr = SAM_USBHS_DEVEPTIER(epno);
      regval  = sam_getreg(regaddr);
      regval |= USBHS_DEVEPTCFG_AUTOSW;
      sam_putreg(regval, regaddr);
    }
#endif

  sam_dumpep(priv, epno);
  return OK;
}

/****************************************************************************
 *
 * Name: sam_ep0_configure
 *
 * Description:
 *  Configure EP0 for normal operation.
 *
 ****************************************************************************/

static inline int sam_ep0_configure(struct sam_usbdev_s *priv)
{
  return sam_ep_configure_internal(&priv->eplist[EP0], &g_ep0desc);
}

/****************************************************************************
 *
 * Endpoint operations
 *
 ****************************************************************************/

/****************************************************************************
 *
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
  struct sam_usbdev_s *priv;
  int ret;

  /* Verify parameters.  Endpoint 0 is not available at this interface */

#if defined(CONFIG_DEBUG_FEATURES) || defined(CONFIG_USBDEV_TRACE)
  uint8_t epno = USB_EPNO(desc->addr);
  usbtrace(TRACE_EPCONFIGURE, (uint16_t)epno);

  DEBUGASSERT(ep && desc && epno > 0 && epno < SAM_USBHS_NENDPOINTS);
  DEBUGASSERT(epno == USB_EPNO(ep->eplog));
#endif

  /* This logic is implemented in sam_ep_configure_internal */

  ret = sam_ep_configure_internal(privep, desc);
  if (ret == OK && last)
    {
      /* If this was the last endpoint, then the class driver is fully
       * configured.
       */

      priv           = privep->dev;
      priv->devstate = USBHS_DEVSTATE_CONFIGURED;
    }

  return ret;
}

/****************************************************************************
 *
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

  DEBUGASSERT(ep != NULL);

  epno = USB_EPNO(ep->eplog);
  usbtrace(TRACE_EPDISABLE, epno);

  /* Reset the endpoint and cancel any ongoing activity */

  flags = enter_critical_section();
  priv  = privep->dev;
  sam_ep_reset(priv, epno);

  /* Revert to the addressed-but-not-configured state */

  priv->devstate = USBHS_DEVSTATE_ADDRESSED;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 *
 * Name: sam_ep_allocreq
 *
 * Description:
 *   This is the allocreq() method of the USB device endpoint structure.
 *
 ****************************************************************************/

static struct usbdev_req_s *sam_ep_allocreq(struct usbdev_ep_s *ep)
{
  struct sam_req_s *privreq;

  DEBUGASSERT(ep != NULL);
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
 *
 * Name: sam_ep_freereq
 *
 * Description:
 *   This is the freereq() method of the USB device endpoint structure.
 *
 ****************************************************************************/

static void sam_ep_freereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct sam_req_s *privreq = (struct sam_req_s *)req;

  DEBUGASSERT(ep != NULL && req != NULL);
  usbtrace(TRACE_EPFREEREQ, USB_EPNO(ep->eplog));

  kmm_free(privreq);
}

/****************************************************************************
 *
 * Name: sam_ep_allocbuffer
 *
 * Description:
 *   This is the allocbuffer() method of the USB device endpoint structure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *sam_ep_allocbuffer(struct usbdev_ep_s *ep, uint16_t nbytes)
{
  /* Allocate properly aligned memory */

  return kmm_memalign(USBHS_ALIGN, USBHS_ALIGN_UP(nbytes));
}
#endif

/****************************************************************************
 *
 * Name: sam_ep_freebuffer
 *
 * Description:
 *   This is the freebuffer() method of the USB device endpoint structure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void sam_ep_freebuffer(struct usbdev_ep_s *ep, void *buf)
{
  /* There is no special buffer requirement to free aligned DMA buffers */

  kumm_free(buf);
}
#endif

/****************************************************************************
 *
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

  DEBUGASSERT(ep != NULL && req != NULL && req->callback !=
              NULL && req->buf != NULL);
  usbtrace(TRACE_EPSUBMIT, USB_EPNO(ep->eplog));
  priv = privep->dev;
  DEBUGASSERT(priv->driver != NULL);

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
      /* If the endpoint is stalled, then fail any attempts to write
       * through the endpoint.
       */

      if (privep->stalled)
        {
          sam_req_abort(privep, privreq, -EBUSY);
          uerr("ERROR: stalled\n");
          ret = -EPERM;
        }
      else
        {
          /* Add the new request to the request queue for the IN endpoint */

          sam_req_enqueue(&privep->reqq, privreq);
          usbtrace(TRACE_INREQQUEUED(epno), req->len);

          /* If the IN endpoint is IDLE, then transfer the data now */

          if (privep->epstate == USBHS_EPSTATE_IDLE)
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

      /* If the OUT endpoint IDLE, then setup the read */

      if (privep->epstate == USBHS_EPSTATE_IDLE)
        {
          ret = sam_req_read(priv, privep, 0);
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 *
 * Name: sam_ep_cancel
 *
 ****************************************************************************/

static int sam_ep_cancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
  irqstate_t flags;

  DEBUGASSERT(ep != NULL && req != NULL);
  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));

  flags = enter_critical_section();
  sam_req_cancel(privep, -EAGAIN);
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 *
 * Name: sam_ep_stall
 *
 ****************************************************************************/

static int sam_ep_stall(struct usbdev_ep_s *ep, bool resume)
{
  struct sam_ep_s *privep;
  struct sam_usbdev_s *priv;
  uint8_t epno = USB_EPNO(ep->eplog);
  uintptr_t regaddr;
  uint32_t regval;
  irqstate_t flags;

  DEBUGASSERT(ep != NULL);

  /* Check that endpoint is in Idle state */

  privep = (struct sam_ep_s *)ep;
  DEBUGASSERT(/* privep->epstate == USBHS_EPSTATE_IDLE && */ privep->dev);

  priv = (struct sam_usbdev_s *)privep->dev;
  epno = USB_EPNO(ep->eplog);

  /* STALL or RESUME the endpoint */

  flags = enter_critical_section();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, USB_EPNO(ep->eplog));

  /* Handle the resume condition */

  if (resume)
    {
      /* Check if the endpoint is halted */

      if (privep->epstate == USBHS_EPSTATE_STALLED)
        {
          usbtrace(TRACE_EPRESUME, epno);
          privep->stalled = false;

          /* Return endpoint to Idle state */

          privep->epstate = USBHS_EPSTATE_IDLE;

          /* Clear FORCESTALL request
           * REVISIT:  Data sheet says to reset toggle to DATA0 only on OUT
           * endpoints.
           */

          /* Clear FORCESTALL flag */

          sam_putreg(USBHS_DEVEPTINT_STALLEDI, SAM_USBHS_DEVEPTIDR(epno));

          regaddr = SAM_USBHS_DEVEPTCFG(epno);
          regval  = sam_getreg(regaddr);
          regval |= USBHS_DEVEPTCFG_AUTOSW;
          sam_putreg(regval, regaddr);

          /* Reset the endpoint */

          regval  = sam_getreg(SAM_USBHS_DEVEPT);
          regval |= USBHS_DEVEPT_EPRST(epno);
          sam_putreg(regval, SAM_USBHS_DEVEPT);

          regval &= ~USBHS_DEVEPT_EPRST(epno);
          sam_putreg(regval, SAM_USBHS_DEVEPT);

          /* Resuming any blocked data transfers on the endpoint */

          if (epno == 0 || USB_ISEPIN(ep->eplog))
            {
              /* IN endpoint (or EP0).  Restart any queued write requests */

              sam_req_write(priv, privep);
            }
          else
            {
              /* OUT endpoint.  Restart any queued read requests. */

              sam_req_read(priv, privep, 0);
            }
        }
    }

  /* Handle the stall condition */

  else
    {
      /* Check that endpoint is enabled and not already in Halt state */

      if ((privep->epstate != USBHS_EPSTATE_DISABLED) &&
          (privep->epstate != USBHS_EPSTATE_STALLED))
        {
          usbtrace(TRACE_EPSTALL, epno);

          /* If this is an IN endpoint (or endpoint 0), then cancel all
           * of the pending write requests.
           */

          if (epno == 0 || USB_ISEPIN(ep->eplog))
            {
              sam_req_cancel(privep, -EPERM);
            }

          /* Otherwise, it is an OUT endpoint.  Complete any read request
           * currently in progress (it will get requeued immediately).
           */

          else if (privep->epstate == USBHS_EPSTATE_RECEIVING)
            {
              sam_req_complete(privep, -EPERM);
            }

          /* Put endpoint into stalled state */

          privep->epstate = USBHS_EPSTATE_STALLED;
          privep->stalled = true;

          regaddr = SAM_USBHS_DEVEPTCFG(epno);
          regval  = sam_getreg(regaddr);
          regval &= ~USBHS_DEVEPTCFG_AUTOSW;
          sam_putreg(regval, regaddr);

          sam_putreg(USBHS_DEVEPTINT_STALLRQI, SAM_USBHS_DEVEPTIER(epno));

          /* Disable endpoint/DMA interrupts.  They not be re-enabled until
           * the stall is cleared and the next transfer is started.  It
           * would, of course, be a bad idea to do this on EP0 since it is
           * a SETUP request that is going to clear the STALL.
           */

          if (epno != 0)
            {
              regval  = sam_getreg(SAM_USBHS_DEVIDR);
              regval |=  USBHS_DEVINT_PEP(epno);

#ifdef CONFIG_USBDEV_DMA
              if (SAM_USBHS_DMA(epno))
                {
                  /* Disable the endpoint DMA interrupt */

                  regval |= USBHS_DEVINT_DMA(epno);
                }
#endif
              sam_putreg(regval, SAM_USBHS_DEVIDR);
            }
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 *
 * Device Controller Operations
 *
 ****************************************************************************/

/****************************************************************************
 *
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
  DEBUGASSERT(dev != NULL);

  /* Ignore any direction bits in the logical address */

  epno = USB_EPNO(epno);

  /* A logical address of 0 means that any endpoint will do */

  if (epno > 0)
    {
      /* Otherwise, we will return the endpoint
       * structure only for the requested 'logical' endpoint.
       * All of the other checks will still be performed.
       *
       * First, verify that the logical endpoint is in the
       * range supported by the hardware.
       */

      if (epno >= SAM_USBHS_NENDPOINTS)
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
 *
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

  DEBUGASSERT(dev != NULL && ep != NULL);

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
 *
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

  DEBUGASSERT(dev != NULL);

  /* Return the last frame number detected by the hardware */

  regval  = sam_getreg(SAM_USBHS_DEVFNUM);
  frameno = (regval & USBHS_DEVFNUM_FNUM_MASK) >> USBHS_DEVFNUM_FNUM_SHIFT;

  usbtrace(TRACE_DEVGETFRAME, frameno);
  return frameno;
}

/****************************************************************************
 *
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
  DEBUGASSERT(dev != NULL);

  /* Resume normal operation */

  flags = enter_critical_section();
  sam_resume(priv);

  /* Activate a remote wakeup.
   * Setting this bit forces an external interrupt
   * on the USBHS controller for Remote Wake UP purposes.
   * An Upstream Resume is sent only after the
   * USBHS bus has been in SUSPEND state for at least 5 ms.
   */

  regval  = sam_getreg(SAM_USBHS_DEVCTRL);
  regval |= USBHS_DEVCTRL_RMWKUP;
  sam_putreg(regval, SAM_USBHS_DEVCTRL);
  leave_critical_section(flags);

  /* This bit is automatically cleared by hardware at the end of the Upstream
   * Resume
   */

  while ((sam_getreg(SAM_USBHS_DEVCTRL) & USBHS_DEVCTRL_RMWKUP) != 0);
  return OK;
}

/****************************************************************************
 *
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
  DEBUGASSERT(dev != NULL);

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 *
 * Name: sam_pullup
 *
 * Description:
 *   This is the pullup() method of the USB device driver interface
 *
 ****************************************************************************/

static int sam_pullup(struct usbdev_s *dev, bool enable)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  flags = enter_critical_section();
  if (enable)
    {
      /* Un-freeze clocking.
       *
       * When the clock is frozen, on certain bits in the USBCH_CTRL
       * register can be modified (FRZCLK, USBE, and LS).  In addition,
       * only the asynchronous interrupt sources can trigger the USB
       * interrupt:
       *
       *  - Wake-up Interrupt (USBHS_DEVISR.WAKEUP)
       *  - Host Wake-up Interrupt (USBHS_HSTISR.HWUPI)
       */

      regval  = sam_getreg(SAM_USBHS_CTRL);
      regval &= ~USBHS_CTRL_FRZCLK;
      sam_putreg(regval, SAM_USBHS_CTRL);

      /* DETACH=0: USBHS is attached.  ARMs the USBHS to pull up the DP line
       * when the USBHS is no longer suspended.
       */

      regval = sam_getreg(SAM_USBHS_DEVCTRL);
      regval &= ~USBHS_DEVCTRL_DETACH;
      sam_putreg(regval, SAM_USBHS_DEVCTRL);

      priv->usbdev.speed = USB_SPEED_FULL;

      /* There are several possibilities:
       *
       * 1. The device may not be plugged into a host.
       *    In that case, the hardware will be in a suspended state.
       *    When an idle USB bus state has been detected for 3 ms,
       *    the controller sets the Suspend
       *    (USBHS_DEVISR.SUSP) interrupt bit.
       * 2. We may have been suspended but a WAKEUP
       *    event has already occurred.
       *    The USBHS_DEVISR.WAKEUP interrupt bit is
       *    set when a non-idle event is detected,
       *    it can occur whether the controller is in the Suspend
       *    mode or not.
       *    The USBHS_DEVISR.SUSP and USBHS_DEVISR.WAKEUP interrupts
       *    are thus independent, except that one bit is cleared
       *    when the other is set.
       * 3. Because we have already enabled the pull-up,
       *    that event may have already have been reset by the host.
       */

      regval = sam_getreg(SAM_USBHS_DEVISR);
      if ((regval & USBHS_DEVINT_SUSPD) == 0)
        {
          /* If the USBHS detects activity on the BUS, then it will not be
           * suspended.  In that case, next event that we expect to see is a
           * reset from the connected host.  When a USB reset is detected on
           * the USB line, the following operations are performed by the
           * controller:
           *
           *   - All endpoints are disabled, except the default control
           *     endpoint.
           *   - The default control endpoint is reset
           *   - The data toggle sequence of the default control endpoint is
           *     cleared.
           *   - At the end of the reset process, the End of Reset
           *     (USBHS_DEVISR.EORST) bit is set.
           *   - During a reset, the USBHS automatically switches to High-
           *     speed mode if the host is High-speed-capable (the reset is
           *     called High-speed reset). The user should observe the
           *     USBHS_SR.SPEED field to know the speed running at the end
           *     of the reset (USBHS_DEVISR.EORST = 1).
           *
           * The class implementation should not call this method with
           * enable == true until is is fully initialized and ready to
           * accept connections.
           */

          /* Enable expected interrupts */

          sam_putreg(USBHS_DEVINT_EORST | USBHS_DEVINT_EORSM |
                     USBHS_DEVINT_SUSPD,
                     SAM_USBHS_DEVIER);

          /* Leave the clock unfrozen */
        }
      else
        {
          /* If there is no host connected (no bus activity), then we
           * might get a SUSPend interrupt instead of a End of Reset.  In
           * the case, we would like to keep the clock frozen until the
           * host is connected.
           *
           * The strategy here was taken from the SAMV7 sample code:   It
           * will force a SUSPend event.  Then disable clocking.  We will
           * take the SUSPend interrupt (because it is already pending),
           * but after the clock is frozen, only a WAKEUP interrupt can be
           * received.
           */

          /* Enable wakeup interrupts */

          sam_putreg(USBHS_DEVINT_WAKEUP | USBHS_DEVINT_EORSM,
                     SAM_USBHS_DEVIER);

          /* Enable expected interrupts */

          sam_putreg(USBHS_DEVINT_EORST | USBHS_DEVINT_WAKEUP |
                     USBHS_DEVINT_SUSPD,
                     SAM_USBHS_DEVIER);

          /* Clear pending interrupts */

          sam_putreg(USBHS_DEVINT_EORST | USBHS_DEVINT_SUSPD,
                     SAM_USBHS_DEVICR);

          /* Force the first suspend event */

          sam_putreg(USBHS_DEVINT_SUSPD, SAM_USBHS_DEVIFR);
          sam_putreg(USBHS_DEVINT_WAKEUP, SAM_USBHS_DEVICR);

          /* Refreeze the clock and wait for the wakeup event */

          regval  = sam_getreg(SAM_USBHS_CTRL);
          regval |= USBHS_CTRL_FRZCLK;
          sam_putreg(regval, SAM_USBHS_CTRL);
        }
    }
  else
    {
      /* DETACH=1: USBHS is detached, UTMI transceiver is suspended. */

      regval  = sam_getreg(SAM_USBHS_DEVCTRL);
      regval |= USBHS_DEVCTRL_DETACH;
      sam_putreg(regval, SAM_USBHS_DEVCTRL);

      /* Freeze clocking */

      regval  = sam_getreg(SAM_USBHS_CTRL);
      regval |= USBHS_CTRL_FRZCLK;
      sam_putreg(regval, SAM_USBHS_CTRL);

      /* Device returns to the Powered state */

      if (priv->devstate > USBHS_DEVSTATE_POWERED)
        {
          priv->devstate = USBHS_DEVSTATE_POWERED;
        }

      if (priv->prevstate > USBHS_DEVSTATE_POWERED)
        {
          priv->prevstate = USBHS_DEVSTATE_POWERED;
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 *
 * Initialization/Reset
 *
 ****************************************************************************/

/****************************************************************************
 *
 * Name: sam_reset
 *
 ****************************************************************************/

static void sam_reset(struct sam_usbdev_s *priv)
{
  uint32_t regval;
  uint8_t epno;

  /* Unfreeze clocking to the USBHS peripheral. */

  regval  = sam_getreg(SAM_USBHS_CTRL);
  regval &= ~USBHS_CTRL_FRZCLK;
  sam_putreg(regval, SAM_USBHS_CTRL);

  /* Tell the class driver that we are disconnected.  The class driver
   * should then accept any new configurations.
   */

  CLASS_DISCONNECT(priv->driver, &priv->usbdev);

  /* The device enters the Default state */

  /* Disable the device address */

  regval  = sam_getreg(SAM_USBHS_DEVCTRL);
  regval &= ~(USBHS_DEVCTRL_UADD_MASK | USBHS_DEVCTRL_ADDEN);
  sam_putreg(regval, SAM_USBHS_DEVCTRL);

  /* Revert to the un-addressed, default state */

  priv->devstate  = USBHS_DEVSTATE_DEFAULT;

  /* Reset and disable all endpoints.  Then re-configure EP0 */

  sam_epset_reset(priv, SAM_EPSET_ALL);
  sam_ep0_configure(priv);

  /* Reset endpoint data structures */

  for (epno = 0; epno < SAM_USBHS_NENDPOINTS; epno++)
    {
      struct sam_ep_s *privep = &priv->eplist[epno];

      /* Cancel any queued requests.  Since they are cancelled
       * with status -ESHUTDOWN, then will not be re-queued
       * until the configuration is reset.  NOTE:  This should
       * not be necessary... the CLASS_DISCONNECT above should
       * result in the class implementation calling sam_ep_disable
       * for each of its configured endpoints.
       */

      sam_req_cancel(privep, -ESHUTDOWN);

      /* Reset endpoint status */

      privep->stalled   = false;
      privep->halted    = false;
      privep->zlpneeded = false;
      privep->zlpsent   = false;
    }

  /* Re-configure the USB controller in its initial, unconnected state */

  priv->usbdev.speed = USB_SPEED_FULL;

  /* Enable normal operational interrupts (including endpoint 0) */

  regval = USBHS_DEVINT_EORST | USBHS_DEVINT_WAKEUP | USBHS_DEVINT_SUSPD |
           USBHS_DEVINT_PEP0;
  sam_putreg(regval, SAM_USBHS_DEVIER);

  sam_dumpep(priv, EP0);
}

/****************************************************************************
 *
 * Name: sam_hw_setup
 *
 ****************************************************************************/

static void sam_hw_setup(struct sam_usbdev_s *priv)
{
  uint32_t regval;
  int i;

  /* Disable USB hardware.  Will perform a reset of most resisters. */

  regval  = sam_getreg(SAM_USBHS_CTRL);
  regval &= ~USBHS_CTRL_USBE;
  sam_putreg(regval, SAM_USBHS_CTRL);

  /* Configure USBHS pins.  Nothing needs to be done:  HDM and HDP are the
   * primary pin functions and there are no alternatives.
   */

  /* Enable clocking to the USBHS peripheral.
   *
   * The clock for the USBHS bus interface is generated by the Power
   * Management Controller (PMC).  Before enabling the USB clock in the
   * PMC, the USBHS must be enabled (by writing a one to the
   * USBHS_CTRL.USBE bit and a zero to the USBHS_CTRL.FRZCLK bit).
   *
   * The USBHS can work in two modes:
   *
   * - Normal mode (SPDCONF = 0) where High speed, Full speed and Low
   *   speed are available.
   * - Low-power mode (SPDCONF = 1) where Full speed and Low speed are
   *   available.
   *
   * Only the normal mode is supported by this driver.  For Normal mode:
   *
   * 1. Enable the USBHS peripheral clock (PMC_PCER).
   * 2. Enable the USBHS (UIMOD = 1, USBE = 1, FRZCLK = 0).
   * 3. Enable the UPLL 480 MHz.
   * 4. Wait for the UPLL 480 MHz to be considered as locked by the PMC.
   *
   * Steps 3 and 4 are performed in sam_usbclock.c.
   */

  /* Enable the USBHS peripheral clock (PMC_PCER) */

  sam_usbhs_enableclk();

  /* Enable USBHS peripheral (USBE = 1) in device mode (UIMOD = 1) and
   * unfreeze clocking (FRZCLK = 0)
   */

  regval |= USBHS_CTRL_UIMOD_DEVICE;
  sam_putreg(regval, SAM_USBHS_CTRL);

  regval |= USBHS_CTRL_USBE;
  sam_putreg(regval, SAM_USBHS_CTRL);

  regval &= ~USBHS_CTRL_UIDE;
  sam_putreg(regval, SAM_USBHS_CTRL);

  regval &= ~USBHS_CTRL_FRZCLK;
  sam_putreg(regval, SAM_USBHS_CTRL);

  /* Enable the UPLL */

  sam_usbclock();

  /* Select High Speed or force Full Speed */

  regval  = sam_getreg(SAM_USBHS_DEVCTRL);
  regval &= ~USBHS_DEVCTRL_SPDCONF_MASK;
#ifdef CONFIG_SAMV7_USBDEVHS_LOWPOWER
  regval |= USBHS_DEVCTRL_SPDCONF_LOWPOWER;
#else
  regval |= USBHS_DEVCTRL_SPDCONF_NORMAL;
#endif
  sam_putreg(regval, SAM_USBHS_DEVCTRL);

  /* Wait for UTMI clocking to be usable */

  while ((sam_getreg(SAM_USBHS_SR) & USBHS_SR_CLKUSABLE) == 0);

  /* Make sure that we are not in Forced Low-Speed mode */

  regval  = sam_getreg(SAM_USBHS_DEVCTRL);
  regval &= ~USBHS_DEVCTRL_LS;
  sam_putreg(regval, SAM_USBHS_DEVCTRL);

  /* Reset and disable all endpoints, re-initializing endpoint 0. */

  sam_epset_reset(priv, SAM_EPSET_ALL);
  sam_ep0_configure(priv);

  /* Disconnect the device */

  regval  = sam_getreg(SAM_USBHS_DEVCTRL);
  regval |= USBHS_DEVCTRL_DETACH;
  sam_putreg(regval, SAM_USBHS_DEVCTRL);

  /* Initialize DMA channels */

  for (i = 1; i <= SAM_USBHS_NDMACHANNELS; i++)
    {
      /* Stop any DMA transfer */

      sam_putreg(0, SAM_USBHS_DEVDMACTRL(i));
    }

  /* Disable all endpoint interrupts.  Disable all endpoints */

  sam_putreg(USBHS_DEVEPT_ALLEPEN, SAM_USBHS_DEVIDR);
  sam_putreg(0, SAM_USBHS_DEVEPT);

  /* Disable each endpoint interrupt */

  for (i = 0; i < SAM_USBHS_NENDPOINTS; i++)
    {
      /* Disable endpoint interrupts */

      sam_putreg(USBHS_DEVEPTIDR_ALLINTS, SAM_USBHS_DEVEPTIDR(i));

      /* Clear endpoint status */

      sam_putreg(USBHS_DEVEPTICR_ALLINTS, SAM_USBHS_DEVEPTICR(i));
    }

  /* Disable all interrupts */

  sam_putreg(USBHS_DEVINT_ALL, SAM_USBHS_DEVIDR);

  /* Initialization complete... Freeze the clock */

  regval  = sam_getreg(SAM_USBHS_CTRL);
  regval |= USBHS_CTRL_FRZCLK;
  sam_putreg(regval, SAM_USBHS_CTRL);
}

/****************************************************************************
 *
 * Name: sam_sw_setup
 *
 ****************************************************************************/

static void sam_sw_setup(struct sam_usbdev_s *priv)
{
  int epno;

#ifdef CONFIG_SAMV7_USBDEVHS_SCATTERGATHER
#ifndef CONFIG_SAMV7_USBDEVHS_PREALLOCATE
  int i;

  /* Allocate a pool of free DMA transfer descriptors */

  priv->dtdpool = (struct sam_dtd_s *)
    kmm_memalign(16, CONFIG_SAMV7_USBDEVHS_NDTDS * sizeof(struct sam_dtd_s));
  if (!priv->dtdpool)
  {
      uerr("ERROR: Failed to allocate the DMA transfer descriptor pool\n");
      return NULL;
  }

  /* Initialize the list of free DMA transfer descriptors */

  for (i = 0; i < CONFIG_SAMV7_USBDEVHS_NDTDS; i++)
    {
      /* Put the transfer descriptor in a free list */

      sam_td_free(&priv->dtdpool[i]);
    }

#else
  /* Initialize the list of free DMA transfer descriptors */

  DEBUGASSERT(((uintptr_t)&g_dtdpool & 15) == 0);
  for (i = 0; i < CONFIG_SAMV7_USBDEVHS_NDTDS; i++)
    {
      /* Put the transfer descriptor in a free list */

      sam_td_free(&g_dtdpool[i]);
    }

#endif /* CONFIG_SAMV7_USBDEVHS_PREALLOCATE */
#endif /* CONFIG_SAMV7_USBDEVHS_SCATTERGATHER */

  /* Initialize the device state structure.  NOTE: many fields
   * have the initial value of zero and, hence, are not explicitly
   * initialized here.
   */

  memset(priv, 0, sizeof(struct sam_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[EP0].ep;
  priv->epavail    = SAM_EPSET_ALL & ~SAM_EP_BIT(EP0);
  priv->devstate   = USBHS_DEVSTATE_SUSPENDED;
  priv->prevstate  = USBHS_DEVSTATE_POWERED;

  /* Initialize the endpoint list */

  for (epno = 0; epno < SAM_USBHS_NENDPOINTS; epno++)
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

      priv->eplist[epno].ep.maxpacket = SAM_USBHS_MAXPACKETSIZE(epno);
    }

#if CONFIG_USBDEV_EP0_MAXSIZE < SAM_EP0_MAXPACKET
  /* Select a smaller endpoint size for EP0 if so configured */

  priv->eplist[EP0].ep.maxpacket = CONFIG_USBDEV_EP0_MAXSIZE;
#endif
}

/****************************************************************************
 *
 * Name: sam_hw_shutdown
 *
 ****************************************************************************/

static void sam_hw_shutdown(struct sam_usbdev_s *priv)
{
  uint32_t regval;

  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable all interrupts */

  sam_putreg(USBHS_DEVINT_ALL, SAM_USBHS_DEVIDR);

  /* Clear all pending interrupt status */

  sam_putreg(USBHS_DEVINT_ALL, SAM_USBHS_DEVICR);

  /* DETACH=1: USBHS is detached, UTMI transceiver is suspended. */

  regval  = sam_getreg(SAM_USBHS_DEVCTRL);
  regval |= USBHS_DEVCTRL_DETACH;
  sam_putreg(regval, SAM_USBHS_DEVCTRL);

  /* Freeze clocking */

  regval  = sam_getreg(SAM_USBHS_CTRL);
  regval |= USBHS_CTRL_FRZCLK;
  sam_putreg(regval, SAM_USBHS_CTRL);

  /* Disable USB hardware */

  regval &= ~USBHS_CTRL_USBE;
  sam_putreg(regval, SAM_USBHS_CTRL);

  /* Disable clocking to the USBHS peripheral */

  sam_usbhs_disableclk();
}

/****************************************************************************
 *
 * Name: sam_sw_shutdown
 *
 ****************************************************************************/

static void sam_sw_shutdown(struct sam_usbdev_s *priv)
{
}

/****************************************************************************
 *
 * Public Functions
 *
 ****************************************************************************/

/****************************************************************************
 *
 * Name: arm_usbinitialize
 * Description:
 *   Initialize the USB driver
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_usbinitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_usbhs;

  usbtrace(TRACE_DEVINIT, 0);

  /* Software initialization */

  sam_sw_setup(priv);

  /* Power up and initialize USB controller.  Interrupts from the USBHS
   * controller are initialized here, but will not be enabled at the NVIC
   * until the class driver is installed.
   */

  sam_hw_setup(priv);

  /* Attach USB controller interrupt handlers.  The hardware will not be
   * initialized and interrupts will not be enabled until the class device
   * driver is bound.  Getting the IRQs here only makes sure that we have
   * them when we need them later.
   */

  if (irq_attach(SAM_IRQ_USBHS, sam_usbhs_interrupt, NULL) != 0)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
               (uint16_t)SAM_IRQ_USBHS);
      goto errout;
    }

  /* Enable USB controller interrupts at the NVIC.  Interrupts are still
   * disabled at the USBHS.
   */

  up_enable_irq(SAM_IRQ_USBHS);
  return;

errout:
  arm_usbuninitialize();
}

/****************************************************************************
 *
 * Name: arm_usbuninitialize
 * Description:
 *   Initialize the USB driver
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_usbuninitialize(void)
{
  struct sam_usbdev_s *priv = &g_usbhs;
  irqstate_t flags;

  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  flags = enter_critical_section();
  usbtrace(TRACE_DEVUNINIT, 0);

  /* Disable and detach the USBHS IRQ */

  up_disable_irq(SAM_IRQ_USBHS);
  irq_detach(SAM_IRQ_USBHS);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Put the hardware in an inactive state */

  sam_hw_shutdown(priv);
  sam_sw_shutdown(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 *
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver.
 *   The class driver's bind() method will be
 *   called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_usbhs;
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);
  DEBUGASSERT(driver != NULL &&
              driver->ops->bind != NULL &&
              driver->ops->unbind != NULL &&
              driver->ops->disconnect != NULL &&
              driver->ops->setup != NULL);
  DEBUGASSERT(priv->driver == NULL);

  /* First hook up the driver */

  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret != OK)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BINDFAILED), (uint16_t)-ret);
      priv->driver = NULL;
    }

  return ret;
}

/****************************************************************************
 *
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

  struct sam_usbdev_s *priv = &g_usbhs;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);
  DEBUGASSERT(driver == priv->driver);

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = enter_critical_section();

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts (but keep them attached) */

  up_disable_irq(SAM_IRQ_USBHS);

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

#endif /* CONFIG_USBDEV && CONFIG_SAMV7_USBDEVHS */
