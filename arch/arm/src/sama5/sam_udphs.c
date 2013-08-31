/****************************************************************************
 * arch/arm/src/sama5/sam_udphs.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.orgr>
 *
 * References:
 *   SAMA5D3 Series Data Sheet
 *   Atmel NoOS sample code.
 *
 * The Atmel sample code has a BSD compatibile license that requires this
 * copyright notice:
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
#include "sam_periphclks.h"
#include "sam_syscfg.h"
#include "sam_gpio.h"
#include "sam_usbdev.h"

#if defined(CONFIG_USBDEV) && defined(CONFIG_SAM_USB)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef CONFIG_USB_PRI
#  define CONFIG_USB_PRI NVIC_SYSH_PRIORITY_DEFAULT
#endif

/* Number of DMA transfer descriptors.  Default: 8 */

#ifndef CONFIG_SAMA5_UDPHS_NTDS
#  define CONFIG_SAMA5_UDPHS_NTDS 8
#endif

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_SAMA5_UDPHS_REGDEBUG
#endif

/* Not yet supported */

#undef CONFIG_SAMA5_UDPHS_SCATTERGATHER

/* Driver Definitions *******************************************************/
/* Initial interrupt mask: Reset + Suspend + Correct Transfer */

#define SAM_CNTR_SETUP     (USB_CNTR_RESETM|USB_CNTR_SUSPM|USB_CNTR_CTRM)

/* Endpoint definitions */

#define EP0                 (0)
#define SAM_EPSET_ALL       (0xffff)  /* All endpoints */
#define SAM_EPSET_NOTEP0    (0xfffe)  /* All endpoints except EP0 */
#define SAM_EPSET_DMA       (0x00fe)  /* All endpoints that support DMA transfers */
#define SAM_EP_BIT(ep)      (1 << (ep))

/* DMA FIFO */

#define DMA_MAX_FIFO_SIZE   (65536/1) /* Max size of the FMA FIFO */
#define EPT_VIRTUAL_SIZE    16384     /* FIFO space size in units of 32-bit words */

/* Packet sizes.  We use a fixed 64 max packet size for all endpoint types */

#define SAM_MAXPACKET_SHIFT (6)
#define SAM_MAXPACKET_SIZE  (1 << (SAM_MAXPACKET_SHIFT))
#define SAM_MAXPACKET_MASK  (SAM_MAXPACKET_SIZE-1)

#define SAM_EP0MAXPACKET    SAM_MAXPACKET_SIZE

/* USB-related masks */

#define REQRECIPIENT_MASK     (USB_REQ_TYPE_MASK | USB_REQ_RECIPIENT_MASK)

/* Endpoint register masks (handling toggle fields) */

#define EPR_NOTOG_MASK        (USB_EPR_CTR_RX  | USB_EPR_SETUP  | USB_EPR_EPTYPE_MASK |\
                               USB_EPR_EP_KIND | USB_EPR_CTR_TX | USB_EPR_EA_MASK)
#define EPR_TXDTOG_MASK       (USB_EPR_STATTX_MASK | EPR_NOTOG_MASK)
#define EPR_RXDTOG_MASK       (USB_EPR_STATRX_MASK | EPR_NOTOG_MASK)

/* Request queue operations *************************************************/

#define sam_rqempty(ep)     ((ep)->head == NULL)
#define sam_rqpeek(ep)      ((ep)->head)

/* USB trace ****************************************************************/
/* Trace error codes */

#define SAM_TRACEERR_ALLOCFAIL            0x0001
#define SAM_TRACEERR_BADCLEARFEATURE      0x0002
#define SAM_TRACEERR_BADDEVGETSTATUS      0x0003
#define SAM_TRACEERR_BADEPGETSTATUS       0x0004
#define SAM_TRACEERR_BADEPNO              0x0005
#define SAM_TRACEERR_BADEPTYPE            0x0006
#define SAM_TRACEERR_BADGETCONFIG         0x0007
#define SAM_TRACEERR_BADGETSETDESC        0x0008
#define SAM_TRACEERR_BADGETSTATUS         0x0009
#define SAM_TRACEERR_BADSETADDRESS        0x000a
#define SAM_TRACEERR_BADSETCONFIG         0x000b
#define SAM_TRACEERR_BADSETFEATURE        0x000c
#define SAM_TRACEERR_BINDFAILED           0x000d
#define SAM_TRACEERR_DISPATCHSTALL        0x000e
#define SAM_TRACEERR_DRIVER               0x000f
#define SAM_TRACEERR_DRIVERREGISTERED     0x0010
#define SAM_TRACEERR_EP0BADCTR            0x0011
#define SAM_TRACEERR_EP0SETUPSTALLED      0x0012
#define SAM_TRACEERR_EPBUFFER             0x0013
#define SAM_TRACEERR_EPDISABLED           0x0014
#define SAM_TRACEERR_EPOUTNULLPACKET      0x0015
#define SAM_TRACEERR_EPRESERVE            0x0016
#define SAM_TRACEERR_INVALIDCTRLREQ       0x0017
#define SAM_TRACEERR_INVALIDPARMS         0x0018
#define SAM_TRACEERR_IRQREGISTRATION      0x0019
#define SAM_TRACEERR_NOTCONFIGURED        0x001a
#define SAM_TRACEERR_REQABORTED           0x001b

/* Trace interrupt codes */

#define SAM_TRACEINTID_CLEARFEATURE       0x0001
#define SAM_TRACEINTID_DEVGETSTATUS       0x0002
#define SAM_TRACEINTID_DISPATCH           0x0003
#define SAM_TRACEINTID_EP0IN              0x0004
#define SAM_TRACEINTID_EP0INDONE          0x0005
#define SAM_TRACEINTID_EP0OUTDONE         0x0006
#define SAM_TRACEINTID_EP0SETUPDONE       0x0007
#define SAM_TRACEINTID_EP0SETUPSETADDRESS 0x0008
#define SAM_TRACEINTID_EPGETSTATUS        0x0009
#define SAM_TRACEINTID_EPINDONE           0x000a
#define SAM_TRACEINTID_EPINQEMPTY         0x000b
#define SAM_TRACEINTID_EPOUTDONE          0x000c
#define SAM_TRACEINTID_EPOUTPENDING       0x000d
#define SAM_TRACEINTID_EPOUTQEMPTY        0x000e
#define SAM_TRACEINTID_ESOF               0x000f
#define SAM_TRACEINTID_GETCONFIG          0x0010
#define SAM_TRACEINTID_GETSETDESC         0x0011
#define SAM_TRACEINTID_GETSETIF           0x0012
#define SAM_TRACEINTID_GETSTATUS          0x0013
#define SAM_TRACEINTID_INTERRUPT          0x0014
#define SAM_TRACEINTID_IFGETSTATUS        0x0015
#define SAM_TRACEINTID_LPCTR              0x0016
#define SAM_TRACEINTID_NOSTDREQ           0x0017
#define SAM_TRACEINTID_RESET              0x0018
#define SAM_TRACEINTID_SETCONFIG          0x0019
#define SAM_TRACEINTID_SETFEATURE         0x001a
#define SAM_TRACEINTID_SUSP               0x001b
#define SAM_TRACEINTID_SYNCHFRAME         0x001c
#define SAM_TRACEINTID_WKUP               0x001d

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
  UDPHS_EPSTATE_DISABLED = 0, /* Endpoint is disabled */
  UDPHS_EPSTATE_HALTED,       /* Endpoint is halted (i.e. STALLs every request) */
  UDPHS_EPSTATE_IDLE,         /* Endpoint is idle (i.e. ready for transmission) */
  UDPHS_EPSTATE_SENDING,      /* Endpoint is sending data */
  UDPHS_EPSTATE_RECEIVING     /* Endpoint is receiving data */
};

enum sam_devstate_s
{
  UDPHS_DEVSTATE_SUSPENDED = 0, /* The device is currently suspended */
  UDPHS_DEVSTATE_ATTACHED,      /* USB cable is plugged into the device */
  UDPHS_DEVSTATE_POWERED,       /* Host is providing +5V through the USB cable */
  UDPHS_DEVSTATE_DEFAULT,       /* Device has been reset */
  UDPHS_DEVSTATE_ADDRESS,       /* The device has been given an address on the bus */
  UDPHS_DEVSTATE_CONFIGURED     /* A valid configuration has been selected. */
};

/* DMA transfer descriptor */

#ifdef CONFIG_SAMA5_UDPHS_SCATTERGATHER
struct sam_dtd_s
{
  struct udphs_dtd_s hw;     /* These are the fields as seen by the hardware */
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

/* The various states of a control pipe */

enum sam_ep0state_e
{
  EP0STATE_IDLE = 0,        /* No request in progress */
  EP0STATE_RDREQUEST,       /* Read request in progress */
  EP0STATE_WRREQUEST,       /* Write request in progress */
  EP0STATE_STALLED          /* We are stalled */
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

/* This is the internal representation of an endpoint */

struct sam_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct sam_ep_s.
   */

  struct usbdev_ep_s   ep;           /* Standard endpoint structure */

  /* SAMA5-specific fields */

  struct sam_usbdev_s *dev;          /* Reference to private driver data */
  struct sam_req_s    *head;         /* Request list for this endpoint */
  struct sam_req_s    *tail;
#ifdef CONFIG_SAMA5_UDPHS_SCATTERGATHER
  struct sam_dtd_s    *dtdll;        /* Head of the DMA transfer descriptor list */
#endif
  volatile uint8_t     epstate;      /* State of the endpoint (see enum sam_epstate_e) */
  volatile uint8_t     bank;         /* Current reception bank (0 or 1) */
  uint8_t              stalled:1;    /* true: Endpoint is stalled */
  uint8_t              halted:1;     /* true: Endpoint feature halted */
  uint8_t              txbusy:1;     /* true: TX endpoint FIFO full */
  uint8_t              txnullpkt:1;  /* Null packet needed at end of transfer */
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

  /* UDPHS-specific fields */

  struct usb_ctrlreq_s     ctrl;          /* Last EP0 request */
  uint8_t                  devstate;      /* State of the device (see enum sam_devstate_e) */
  uint8_t                  prevstate;     /* Previous state of the device */
  uint8_t                  ep0state;      /* State of EP0 (see enum sam_ep0state_e) */
  uint8_t                  nesofs;        /* ESOF counter (for resume support) */
  uint8_t                  rxpending:1;   /* 1: OUT data in PMA, but no read requests */
  uint8_t                  selfpowered:1; /* 1: Device is self powered */
  uint8_t                  epavail;       /* Bitset of available endpoints */
  uint8_t                  bufavail;      /* Bitset of available buffers */
  uint16_t                 imask;         /* Current interrupt mask */

  /* DMA Transfer descriptors */

#ifdef CONFIG_SAMA5_UDPHS_SCATTERGATHER
  struct sam_dtd_s        *tdfree;        /* A list of free transfer descriptors */
#ifndef CONFIG_SAMA5_UDPHS_PREALLOCATE
  struct sam_dtd_s        *tdpool;        /* Pool of allocated DMA transfer descriptors */
#endif
#endif

  /* The endpoint list */

  struct sam_ep_s          eplist[SAM_UDPHS_NENDPOINTS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_SAMA5_UDPHS_REGDEBUG
static void sam_printreg(uintptr_t regaddr, uint32_t regval, bool iswrite);
static void sam_checkreg(uintptr_t regaddr, uint32_t regval, bool iswrite);
static uint32_t sam_getreg(uintptr_t regaddr);
static void sam_putreg(uint32_t regval, uintptr_t regaddr);
static void sam_dumpep(struct sam_usbdev_s *priv, int epno);
#else
static inline uint32_t sam_getreg(uintptr_t regaddr);
static inline void sam_putreg(uint32_t regval, uintptr_t regaddr);
# define sam_dumpep(priv,epno)
#endif

/* Low-Level Helpers ********************************************************/

/* Suspend/Resume Helpers ***************************************************/

static void   sam_suspend(struct sam_usbdev_s *priv);
static void   sam_resume(struct sam_usbdev_s *priv);

/* DMA/Request Helpers ******************************************************/

#ifdef CONFIG_SAMA5_UDPHS_SCATTERGATHER
static struct sam_dtd_s *sam_dtd_alloc(struct sam_usbdev_s *priv);
static void   sam_dtd_free(struct sam_usbdev_s *priv, struct sam_dtd_s *dtd);
#endif
static void   sam_dma_single(uint8_t epno, struct sam_req_s *privreq,
                uint32_t dmacontrol);
static int    sam_req_wrdma(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, struct sam_req_s *privreq)

/* Request Helpers **********************************************************/

static struct sam_req_s *
              sam_req_dequeue(struct sam_ep_s *privep);
static void   sam_req_enqueue(struct sam_ep_s *privep,
                struct sam_req_s *req);
static inline void
              sam_req_abort(struct sam_ep_s *privep,
                struct sam_req_s *privreq, int16_t result);
static void   sam_req_complete(struct sam_ep_s *privep, int16_t result);
static void   sam_fifo_write(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, const uint8_t *data, size_t nbytes);
static int    sam_req_wrnodma(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep, struct sam_req_s *privreq)
static int    sam_req_write(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep);
static int    sam_req_read(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep);
static void   sam_req_cancel(struct sam_ep_s *privep);

/* Interrupt level processing ***********************************************/

static void   sam_ep0_read(struct usb_ctrlreq_s *req);
static void   sam_ep0_dispatch(struct sam_usbdev_s *priv);
static void   sam_ep_done(struct sam_usbdev_s *priv, uint8_t epno);
static void   sam_setdevaddr(struct sam_usbdev_s *priv, uint8_t value);
static void   sam_ep0_setup(struct sam_usbdev_s *priv);
static void   sam_ep0_out(struct sam_usbdev_s *priv);
static void   sam_ep0_in(struct sam_usbdev_s *priv);
static inline void
              sam_ep0_done(struct sam_usbdev_s *priv, uint16_t intsta);
static void   sam_lptransfer(struct sam_usbdev_s *priv);
static void   sam_dma_interrupt(struct sam_usbdev_s *priv, int chan);
static void   sam_ep_interrupt(struct sam_usbdev_s *priv, int epno);
static int    sam_udphs_interrupt(int irq, void *context);

/* Endpoint helpers *********************************************************/

static void   sam_epset_reset(struct sam_usbdev_s *priv, uint16_t epset);
static inline struct sam_ep_s *
              sam_ep_reserve(struct sam_usbdev_s *priv, uint8_t epset);
static inline void
              sam_ep_unreserve(struct sam_usbdev_s *priv,
                struct sam_ep_s *privep);
static inline bool
              sam_ep_reserved(struct sam_usbdev_s *priv, int epno);

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
static int    sam_ep_stall(struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations *****************************************/

static struct usbdev_ep_s *
              sam_allocep(struct usbdev_s *dev, uint8_t epno, bool in,
                uint8_t eptype);
static void   sam_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int    sam_getframe(struct usbdev_s *dev);
static int    sam_wakeup(struct usbdev_s *dev);
static int    sam_selfpowered(struct usbdev_s *dev, bool selfpowered);

/* Initialization/Reset *****************************************************/

static void   sam_reset(struct sam_usbdev_s *priv);
static void   sam_hw_reset(struct sam_usbdev_s *priv);
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

static struct sam_usbdev_s g_usbdev;

static const struct usbdev_epops_s g_epops =
{
  .configure     = sam_ep_configure,
  .disable       = sam_ep_disable,
  .allocreq      = sam_ep_allocreq,
  .freereq       = sam_ep_freereq,
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
  .pullup        = sam_usbpullup,
};

/* This describes endpoint 0 */

static const struct usb_epdesc_s g_ep0desc =
{
  .len           = USB_SIZEOF_EPDESC,
  .type          = USB_DESC_TYPE_ENDPOINT,
  .addr          = EP0,
  .attr          = USB_EP_ATTR_XFER_CONTROL,
  .maxpacketsize = {64, 0},
  .interval      = 0
};

#ifdef CONFIG_SAMA5_UDPHS_SCATTERGATHER
#ifdef CONFIG_SAMA5_UDPHS_PREALLOCATE
/* This is a properly aligned pool of preallocated DMA transfer desciptors */

static struct sam_dtd_s g_dtdpool[CONFIG_SAMA5_UDPHS_NTDS]
                        __attribute__ ((aligned(16)));
#endif
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
 *   Print the contents of a SAMA5 EHCI register
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_UDPHS_REGDEBUG
static void sam_printreg(uint32_t regaddr, uint32_t regval,
                          bool iswrite)
{
  lldbg("%p%s%08x\n", regaddr, iswrite ? "<-" : "->", regval);
}
#endif

/*******************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if it is time to output debug information for accesses to a SAMA5
 *   EHCI register
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_UDPHS_REGDEBUG
static void sam_checkreg(uint32_t regaddr, uint32_t regval, bool iswrite)
{
  static uint32_t *prevaddr = NULL;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

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

      prevaddr  = (uint32_t *)regaddr;
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
 *   Get the contents of an SAMA5 register
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_UDPHS_REGDEBUG
static uint32_t sam_getreg(uint32_t regaddr)
{
  /* Read the value from the register */

  uint32_t regval = *regaddr;

  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, false);
  return regval;
}
#else
static inline uint32_t sam_getreg(uint32_t regaddr)
{
  return *regaddr;
}
#endif

/*******************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *   Set the contents of an SAMA5 register to a value
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_UDPHS_REGDEBUG
static void sam_putreg(uint32_t regval, uint32_t regaddr)
{
  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, true);

  /* Write the value */

  *regaddr = regval;
}
#else
static inline void sam_putreg(uint32_t regval, uint32_t regaddr)
{
  *regaval = regval;
}
#endif

/****************************************************************************
 * Name: sam_dumpep
 ****************************************************************************/

#if defined(CONFIG_SAMA5_UDPHS_REGDEBUG) && defined(CONFIG_DEBUG)
static void sam_dumpep(struct sam_usbdev_s *priv, int epno)
{
  uintptr_t addr;

  /* Common registers */

  lldbg("CNTR:   %04x\n", getreg16(SAM_USB_CNTR));
  lldbg("ISTR:   %04x\n", getreg16(SAM_UDPHS_INTSTA));
  lldbg("FNR:    %04x\n", getreg16(SAM_USB_FNR));
  lldbg("DADDR:  %04x\n", getreg16(SAM_USB_DADDR));
  lldbg("BTABLE: %04x\n", getreg16(SAM_USB_BTABLE));

  /* Endpoint register */

  addr = SAM_USB_EPR(epno);
  lldbg("EPR%d:   [%08x] %04x\n", epno, addr, getreg16(addr));

  /* Endpoint descriptor */

  addr = SAM_USB_BTABLE_ADDR(epno, 0);
  lldbg("DESC:   %08x\n", addr);

  /* Endpoint buffer descriptor */

  addr = SAM_USB_ADDR_TX(epno);
  lldbg("  TX ADDR:  [%08x] %04x\n",  addr, getreg16(addr));

  addr = SAM_USB_COUNT_TX(epno);
  lldbg("     COUNT: [%08x] %04x\n",  addr, getreg16(addr));

  addr = SAM_USB_ADDR_RX(epno);
  lldbg("  RX ADDR:  [%08x] %04x\n",  addr, getreg16(addr));

  addr = SAM_USB_COUNT_RX(epno);
  lldbg("     COUNT: [%08x] %04x\n",  addr, getreg16(addr));
}
#endif

/****************************************************************************
 * Low-Level Helpers
 ****************************************************************************/

/****************************************************************************
 * DMA
 ****************************************************************************/
/****************************************************************************
 * Name: sam_dtd_alloc
 *
 * Description:
 *   Allocate a DMA transfer descriptor by removing it from the free list
 *
 * Assumption:  Caller holds the exclsem
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_UDPHS_SCATTERGATHER
static struct sam_dtd_s *sam_dtd_alloc(struct sam_usbdev_s *priv)
{
  struct sam_dtd_s *dtd;

  /* Remove the DMA transfer descriptor from the freelist */

  dtd = (struct sam_dtd_s *)g_udphs.dtdfree;
  if (dtd)
    {
      g_udphs.dtdfree = ((struct sam_list_s *)dtd)->flink;
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
 * Assumption:  Caller holds the exclsem
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_UDPHS_SCATTERGATHER
static void sam_dtd_free(struct sam_usbdev_s *priv, struct sam_dtd_s *dtd)
{
  struct sam_list_s *entry = (struct sam_list_s *)dtd;

  /* Put the dtd structure back into the free list */

  entry->flink  = g_udphs.dtdfree;
  g_udphs.dtdfree = entry;
}
#endif

/****************************************************************************
 * Name: sam_dma_single
 *
 * Description:
 *   Setup a start a single buffer DMA.
 *
 * Assumption:  Called as part of UDPHS interrupt handling
 *
 ****************************************************************************/

static void sam_dma_single(uint8_t epno, struct sam_req_s *privreq,
                           uint32_t dmacontrol)
{
  uintptr_t buffer;
  uintptr_t physaddr;

  /* Not all endpoints support DMA */

  DEBUGASSERT((SAM_EPSET_DMA & SAM_EP_BIT(epno)) != 0);

  /* Flush the contents of the DMA buffer to RAM */

  buffer = (uintptr_t)&privreq->req.buf[privreg->req.xfrd];
  cp15_clean_dcache(buffer, buffer + privreq->inflight);

  /* Set up the DMA */

  physaddr = sam_phyramaddr(buffer);
  sam_putreg(physaddr, SAM_UDPHS_DMAADDRESS(epno));

  /* Clear any pending interrupts then enable the DMA interrupt */

  (void)sam_getreg(SAM_UDPHS_DMASTATUS(epno));
  regval  = sam_getreg(SAM_UDPHS_IEN);
  regval |= UDPHS_INT_DMA(epno)
  sam_putreg(regval, SAM_UDPHS_IEN);

  /* Setup and enable the DMA */

  sam_putreg(0, SAM_UDPHS_DMACONTROL(epno));

  dmacontrol |= UDPHS_DMACONTROL_BUFLEN(privreq->inflight);
  sam_putreg(dmacontrol, SAM_UDPHS_DMACONTROL(epno));
}

/****************************************************************************
 * Name: sam_req_wrdma
 *
 * Description:
 *   Process the next queued write request for an endpoint that supports DMA.
 *
 ****************************************************************************/

static int sam_req_wrdma(struct sam_usbdev_s *priv, struct sam_ep_s *privep,
                         struct sam_req_s *privreq)
{
  uint32_t regval;
  int epno;

  /* The endpoint must be IDLE and ready to begin the next transfer */

  if (privep->epstate != UDPHS_EPSTATE_IDLE)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EPINBUSY), privep->epstate);
      return -EBUSY;
    }

  /* Switch to the sending state */

  privep->epstate   = UDPHS_EPSTATE_SENDING;
  privep->txnullpkt = 0;
  privreq->inflight = 0;
  privreq->req.xfrd = 0;

  /* Get the endpoint number */

  epno = USB_EPNO(privep->ep.eplog);

  /* Either (1) we are committed to sending the null packet (because
   * txnullpkt == 1 && nbytes == 0), or (2) we havenot yet sent the last
   * packet (nbytes > 0). In either case, it is appropriate to clear
   * txnullpkt now.
   */

  privep->txnullpkt = 0;

  /* How many bytes remain to be transferred in the request? */

  remaining = privreq->req.len - privreq->req.xfrd - privreq->inflight;

  /* If there are no bytes to send, then send a null packet */

  if (remaining > 0)
    {
      /* Clip the transfer to the size of the DMA FIFO */

      if (remaining > DMA_MAX_FIFO_SIZE)
        {
          privreq->inflight = DMA_MAX_FIFO_SIZE;
        }
      else
        {
          privreq->inflight = remaining;
        }

      /* Single transfer */

      sam_dma_single(epno, privreq, UDPHS_DMACONTROL_ENDBEN
                      | UDPHS_DMACONTROL_ENDBUFFIT
                      | UDPHS_DMACONTROL_CHANNENB);
      return OK;
    }

  /* Enable the endpoint interrupt */

  regval  = sam_getreg(SAM_UDPHS_IEN);
  regval |= UDPHS_INT_EPT(epno);
  sam_putreg(regval, SAM_UDPHS_IEN);

  sam_putreg(UDPHS_EPTCTL_TXRDY, SAM_UDPHS_EPTCTLENB);
  return OK;
}

/****************************************************************************
 * Request Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: sam_req_dequeue
 ****************************************************************************/

static struct sam_req_s *sam_req_dequeue(struct sam_ep_s *privep)
{
  struct sam_req_s *ret = privep->head;

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
 * Name: sam_req_enqueue
 ****************************************************************************/

static void sam_req_enqueue(struct sam_ep_s *privep, struct sam_req_s *req)
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
 * Name: sam_req_abort
 ****************************************************************************/

static inline void
sam_req_abort(struct sam_ep_s *privep, struct sam_req_s *privreq, int16_t result)
{
  usbtrace(TRACE_DEVERROR(SAM_TRACEERR_REQABORTED), (uint16_t)USB_EPNO(privep->ep.eplog));

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
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
  privreq = sam_req_dequeue(privep);
  irqrestore(flags);

  if (privreq)
    {
      DEBUGASSERT((privep->epstate == UDPHS_EPSTATE_RECEIVING) ||
                  (privep->epstate == UDPHS_EPSTATE_SENDING))

      /* If endpoint 0, temporarily reflect the state of protocol stalled
       * in the callback.
       */

      bool stalled = privep->stalled;
      if (USB_EPNO(privep->ep.eplog) == EP0)
        {
          privep->stalled = (privep->dev->ep0state == EP0STATE_STALLED);
        }

      /* Save the result in the request structure */

      privreq->req.result = result;

      /* Callback to the request completion handler */

      privreq->flink = NULL;
      privreq->req.callback(&privep->ep, &privreq->req);

      /* Reset the endpoint state and restore the stalled indication */

      privep->epstate   = UDPHS_EPSTATE_IDLE;
      privep->stalled   = stalled;
      privep->txnullpkt = 0;
      privep->inflight  = 0;
    }
}

/****************************************************************************
 * Name: sam_req_wrnodma
 *
 * Description:
 *   Process the next queued write request for an endpoint that does not
 *   support DMA.
 *
 ****************************************************************************/

static int sam_req_wrnodma(struct sam_usbdev_s *priv, struct sam_ep_s *privep,
                           struct sam_req_s *privreq)
{
  uint8_t *buf;
  uint8_t *fifo;
  uint8_t epno;
  int committed;
  int nbytes;
  int bytesleft;

  epno = USB_EPNO(privep->ep.eplog);

  /* Get the number of bytes to send.  The totals bytes remaining to be sent
   * is the the total size of the buffer, minus the number of bytes
   * successfully transferred, minus the number of bytes in-flight.
   */

  committed = privreg.req.xfrd + privreq.inflight
  bytesleft = privreq->req.len - committed;

  /* Clip the requested transfer size to the number of bytes actually
   * available
   */

  nbytes = bytesleft;
  if (nbytes > bytesleft)
    {
      nbytes = bytesleft;
    }

  /* Either (1) we are committed to sending the null packet (because txnullpkt == 1
   * && nbytes == 0), or (2) we havenot yet sent the last packet (nbytes > 0).
   * In either case, it is appropriate to clear txnullpkt now.
   */

  privep->txnullpkt = 0;

  /* If we are not sending a NULL packet, then clip the size to maxpacket
   * and check if we need to send a following NULL packet.
   */

  nbytes = bytesleft;
  if (nbytes > 0)
    {
      /* Either send the maxpacketsize or all of the remaining data in
       * the request.
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

      /* This is the new number of bytes "in-flight" */

      privreq->inflight += nbytes;
      usbtrace(TRACE_WRITE(USB_EPNO(privep->ep.eplog)), nbytes);

      /* The new buffer pointer is the started of the buffer plus the number
       * of bytes successfully transfered plus the number of bytes previously
       * "in-flight".
       */

      buf = privreq->req.buf + committed;

      /* Write packet in the FIFO buffer */

      fifo = (uint8_t *)
        ((uint32_t *)SAM_UDPHSRAM_VSECTION + (EPT_VIRTUAL_SIZE * epno));

      for (; nbytes; nbytes--)
        {
          *fifo++ = *buf++;
        }

        /* Indicate that there is data in the TX packet memory.  This will
         * be cleared when the next data out interrupt is received.
         */

        privep->txbusy = true;
    }

  return OK;
}

/****************************************************************************
 * Name: sam_req_write
 *
 * Description:
 *   Process the next queued write request
 *
 ****************************************************************************/

static int sam_req_write(struct sam_usbdev_s *priv, struct sam_ep_s *privep)
{
  struct sam_req_s *privreq;
  uint8_t epno;

  /* We get here when an IN endpoint interrupt occurs.  So now we know that
   * there is no TX transfer in progress.
   */

  privep->txbusy = false;

  /* Check the request from the head of the endpoint request queue */

  privreq = sam_rqpeek(privep);
  if (!privreq)
    {
      /* There is no TX transfer in progress and no new pending TX
       * requests to send.
       */

      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPINQEMPTY), 0);
      return -ENOENT;
    }

  epno = USB_EPNO(privep->ep.eplog);
  ullvdbg("epno=%d req=%p: len=%d xfrd=%d nullpkt=%d\n",
          epno, privreq, privreq->req.len, privreq->req.xfrd, privep->txnullpkt);

  /* The way that we handle the transfer is going to depend on whether
   * or not this endpoint supports DMA.
   */

  if ((SAM_EPSET_DMA & SAM_EP_BIT(epno)) != 0)
    {
      ret = sam_req_wrdma(priv, privep, privreq);
    }
  else
    {
      ret = sam_req_wrnodma(priv, privep, privreq);
    }

  if (ret < 0)
    {
      return ret;
    }

  /* If all of the bytes were sent (including any final null packet)
   * then we are finished with the request buffer).
   */

  if (privreq->req.len == privreq->req.xfrd && !privep->txnullpkt)
    {
      /* Return the write request to the class driver */

      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);
      privep->txnullpkt = 0;
      sam_req_complete(privep, OK);
    }

  return OK;
}

/****************************************************************************
 * Name: sam_req_read
 *
 * Description:
 *   Process the next queued read request
 *
 ****************************************************************************/

static int sam_req_read(struct sam_usbdev_s *priv, struct sam_ep_s *privep)
{
  struct sam_req_s *privreq;
  uint8_t *dest;
  uint8_t *fifo;
  uint8_t epno;
  int available;
  int remaining;
  int readlen;

  /* Check the request from the head of the endpoint request queue */

  epno    = USB_EPNO(privep->ep.eplog);
  privreq = sam_rqpeek(privep);
  if (!privreq)
    {
      /* Incoming data available in PMA, but no packet to receive the data.
       * Mark that the RX data is pending and hope that a packet is returned
       * soon.
       */

      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPOUTQEMPTY), epno);
      return -ENOENT;
    }

  ullvdbg("EP%d: len=%d xfrd=%d\n", epno, privreq->req.len, privreq->req.xfrd);

  /* Ignore any attempt to receive a zero length packet */

  if (privreq->req.len == 0)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EPOUTNULLPACKET), 0);
      sam_req_complete(privep, OK);
      return OK;
    }

  usbtrace(TRACE_READ(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);

  /* Get the number of bytes that can be received.  This is the size of the
   * user-provided request buffer, minus the number of bytes already
   * transferred to the user-buffer.
   */

  remaining = privreq->req.len - privreg.req.xfrd;

  /* Get the number of bytes to read from FIFO memory */
#missing logic

  /* Read the smaller of the number of bytes available in FIFO and the
   * size remaining in the request buffer provided by the caller.
   */

  readlen = MIN(remaining, available);

 /* Get the source and destination transfer addresses */

  fifo = (uint8_t *)
         ((uint32_t *)SAM_UDPHSRAM_VSECTION + (EPT_VIRTUAL_SIZE * epno));
  dest = privreq->req.buf + privreq->req.xfrd;

  /* Retrieve packet from the FIFO */

  for (; readlen > 0; readlen--)
    {
      *dest++ = *fifo++;
    }

  /* If the receive buffer is full or this is a partial packet,
   * then we are finished with the request buffer).
   */

  privreq->req.xfrd += readlen;
  if (available < privep->ep.maxpacket ||
      privreq->req.xfrd >= privreq->req.len)
    {
      /* Return the read request to the class driver. */

      usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
      sam_req_complete(privep, OK);
    }

  return OK;
}

/****************************************************************************
 * Name: sam_req_cancel
 ****************************************************************************/

static void sam_req_cancel(struct sam_ep_s *privep)
{
  while (!sam_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               (sam_rqpeek(privep))->req.xfrd);
      sam_req_complete(privep, -ESHUTDOWN);
    }
}

/****************************************************************************
 * Interrupt Level Processing
 ****************************************************************************/
/****************************************************************************
 * Name: sam_ep0_read
 *
 * Description:
 *   Read a general USB request from the UDPHS FIFO
 *
 ****************************************************************************/

static void sam_ep0_read(struct usb_ctrlreq_s *req)
{
  uint32_t *buffer = (uint32_t *)req;
  volatile uint32_t *fifo;

  fifo = (volatile uint32_t *)SAM_UDPHSRAM_VSECTION;
  *buffer++ = *fifo;
  *buffer = *fifo;
}

/****************************************************************************
 * Name: sam_ep0_dispatch
 ****************************************************************************/

static void sam_req_dispatch(struct sam_usbdev_s *priv)
{
  int ret;

  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl, NULL, 0);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_DISPATCHSTALL), 0);
          priv->ep0state = EP0STATE_STALLED;
        }
    }
}

/****************************************************************************
 * Name: sam_ep_done
 ****************************************************************************/

static void sam_ep_done(struct sam_usbdev_s *priv, uint8_t epno)
{
  struct sam_ep_s *privep;
  uint32_t epr;

  /* Decode and service non control endpoints interrupt */

  epr    = sam_getreg(SAM_USB_EPR(epno));
  privep = &priv->eplist[epno];

  /* OUT: host-to-device
   * CTR_RX is set by the hardware when an OUT/SETUP transaction
   * successfully completed on this endpoint.
   */

  if ((epr & USB_EPR_CTR_RX) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPOUTDONE), epr);

      /* Handle read requests.  First check if a read request is available to
       * accept the host data.
       */

      if (!sam_rqempty(privep))
        {
          /* Read host data into the current read request */

          (void)sam_ep0_read(priv, privep);

          /* Enable further transactions on the endpoint */
#warning Missing logic
        }

      /* NAK further OUT packets if there there no more read requests */

      if (sam_rqempty(privep))
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPOUTPENDING), (uint16_t)epno);

          /* Mark the RX processing as pending and NAK any OUT actions
           * on this endpoint.
           */
#warning Missing logic

          priv->rxpending = true;
        }

      /* Clear the interrupt status and set the new RX status */
#warning Missing logic
    }

  /* IN: device-to-host
   * CTR_TX is set when an IN transaction successfully completes on
   * an endpoint
   */

  else if ((epr & USB_EPR_CTR_TX) != 0)
    {
      /* Clear interrupt status */

      sam_clrepctrtx(epno);
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPINDONE), epr);

      /* Handle write requests */

      sam_req_write(priv, privep);
    }
}

/****************************************************************************
 * Name: sam_setdevaddr
 ****************************************************************************/

static void sam_setdevaddr(struct sam_usbdev_s *priv, uint8_t value)
{
  int epno;

  /* Set address in every allocated endpoint */

  for (epno = 0; epno < SAM_UDPHS_NENDPOINTS; epno++)
    {
      if (sam_ep_reserved(priv, epno))
        {
          sam_setepaddress((uint8_t)epno, (uint8_t)epno);
        }
    }

  /* Set the device address and enable function */

  sam_putreg(value|USB_DADDR_EF, SAM_USB_DADDR);
}

/****************************************************************************
 * Name: sam_ep0_setup
 ****************************************************************************/

static void sam_ep0_setup(struct sam_usbdev_s *priv)
{
  struct sam_ep_s   *ep0     = &priv->eplist[EP0];
  struct sam_req_s  *privreq = sam_rqpeek(ep0);
  struct sam_ep_s   *privep;
  union wb_u           value;
  union wb_u           index;
  union wb_u           len;
  union wb_u           response;
  bool                 handled = false;
  uint8_t              epno;
  int                  nbytes = 0; /* Assume zero-length packet */
  int                  ret;

  /* Terminate any pending requests (doesn't work if the pending request
   * was a zero-length transfer!)
   */

  while (!sam_rqempty(ep0))
    {
      int16_t result = OK;
      if (privreq->req.xfrd != privreq->req.len)
        {
          result = -EPROTO;
        }

      usbtrace(TRACE_COMPLETE(ep0->ep.eplog), privreq->req.xfrd);
      sam_req_complete(ep0, result);
    }

  /* Assume NOT stalled; no TX in progress */

  ep0->stalled  = 0;
  ep0->txbusy   = 0;

  /* And extract the little-endian 16-bit values to host order */

  value.w = GETUINT16(priv->ctrl.value);
  index.w = GETUINT16(priv->ctrl.index);
  len.w   = GETUINT16(priv->ctrl.len);

  ullvdbg("SETUP: type=%02x req=%02x value=%04x index=%04x len=%04x\n",
          priv->ctrl.type, priv->ctrl.req, value.w, index.w, len.w);

  priv->ep0state = EP0STATE_IDLE;

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
            priv->ep0state = EP0STATE_STALLED;
          }
        else
          {
            switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
               case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  epno = USB_EPNO(index.b[LSB]);
                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EPGETSTATUS), epno);
                  if (epno >= SAM_UDPHS_NENDPOINTS)
                    {
                      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPGETSTATUS), epno);
                      priv->ep0state = EP0STATE_STALLED;
                    }
                  else
                    {
                      privep     = &priv->eplist[epno];
                      response.w = 0; /* Not stalled */
                      nbytes     = 2; /* Response size: 2 bytes */

                      if (USB_ISEPIN(index.b[LSB]))
                        {
                          /* IN endpoint */

                          if (sam_eptxstalled(epno))
                            {
                              /* IN Endpoint stalled */

                              response.b[LSB] = 1; /* Stalled */
                            }
                          }
                      else
                        {
                          /* OUT endpoint */

                          if (sam_eprxstalled(epno))
                            {
                              /* OUT Endpoint stalled */

                              response.b[LSB] = 1; /* Stalled */
                            }
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
                      priv->ep0state = EP0STATE_STALLED;
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
                  priv->ep0state = EP0STATE_STALLED;
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
            handled = true;
          }
        else
          {
            /* Endpoint recipient */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < SAM_UDPHS_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = 0;
                ret            = sam_ep_stall(&privep->ep, true);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADCLEARFEATURE), 0);
                priv->ep0state = EP0STATE_STALLED;
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
            handled = true;
          }
        else
          {
            /* Handler recipient=endpoint */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < SAM_UDPHS_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = 1;
                ret            = sam_ep_stall(&privep->ep, false);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETFEATURE), 0);
                priv->ep0state = EP0STATE_STALLED;
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

        usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPSETADDRESS), value.w);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_DEVICE ||
            index.w != 0 || len.w != 0 || value.w > 127)
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETADDRESS), 0);
            priv->ep0state = EP0STATE_STALLED;
          }

        /* Note that setting of the device address will be deferred.  A zero-length
         * packet will be sent and the device address will be set when the zero-
         * length packet transfer completes.
         */
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
            handled = true;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETSETDESC), 0);
            priv->ep0state = EP0STATE_STALLED;
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
            handled = true;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADGETCONFIG), 0);
            priv->ep0state = EP0STATE_STALLED;
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
             /* The request seems valid... let the class implementation handle it */

             sam_ep0_dispatch(priv);
             handled = true;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADSETCONFIG), 0);
            priv->ep0state = EP0STATE_STALLED;
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
        handled = true;
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
        priv->ep0state = EP0STATE_STALLED;
      }
      break;
    }

  /* At this point, the request has been handled and there are three possible
   * outcomes:
   *
   * 1. The setup request was successfully handled above and a response packet
   *    must be sent (may be a zero length packet).
   * 2. The request was successfully handled by the class implementation.  In
   *    case, the EP0 IN response has already been queued and the local variable
   *    'handled' will be set to true and ep0state != EP0STATE_STALLED;
   * 3. An error was detected in either the above logic or by the class implementation
   *    logic.  In either case, priv->state will be set EP0STATE_STALLED
   *    to indicate this case.
   *
   * NOTE: Non-standard requests are a special case.  They are handled by the
   * class implementation and this function returned early above, skipping this
   * logic altogether.
   */

  if (priv->ep0state != EP0STATE_STALLED && !handled)
    {
      /* We will response.  First, restrict the data length to the length
       * requested in the setup packet
       */

      if (nbytes > len.w)
        {
          nbytes = len.w;
        }

      /* Send the response (might be a zero-length packet) */

      sam_fifo_write(priv, ep0, response.b, nbytes);
      priv->ep0state = EP0STATE_IDLE;
    }
}

/****************************************************************************
 * Name: sam_ep0_in
 ****************************************************************************/

static void sam_ep0_in(struct sam_usbdev_s *priv)
{
  int ret;

  /* There is no longer anything in the EP0 TX packet memory */

  priv->eplist[EP0].txbusy = false;

  /* Are we processing the completion of one packet of an outgoing request
   * from the class driver?
   */

  if (priv->ep0state == EP0STATE_WRREQUEST)
    {
      ret = sam_req_write(priv, &priv->eplist[EP0]);
      priv->ep0state = ((ret == OK) ? EP0STATE_WRREQUEST : EP0STATE_IDLE);
    }

  /* No.. Are we processing the completion of a status response? */

  else if (priv->ep0state == EP0STATE_IDLE)
    {
      /* Look at the saved SETUP command.  Was it a SET ADDRESS request?
       * If so, then now is the time to set the address.
       */

      if (priv->ctrl.req == USB_REQ_SETADDRESS &&
          (priv->ctrl.type & REQRECIPIENT_MASK) == (USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE))
        {
          union wb_u value;
          value.w = GETUINT16(priv->ctrl.value);
          sam_setdevaddr(priv, value.b[LSB]);
        }
    }
  else
    {
      priv->ep0state = EP0STATE_STALLED;
    }
}

/****************************************************************************
 * Name: sam_ep0_out
 ****************************************************************************/

static void sam_ep0_out(struct sam_usbdev_s *priv)
{
  int ret;

  struct sam_ep_s *privep = &priv->eplist[EP0];
  switch (priv->ep0state)
    {
      case EP0STATE_RDREQUEST:  /* Write request in progress */
      case EP0STATE_IDLE:       /* No transfer in progress */
        ret = sam_req_read(priv, privep);
        priv->ep0state = ((ret == OK) ? EP0STATE_RDREQUEST : EP0STATE_IDLE);
        break;

      default:
        /* Unexpected state OR host aborted the OUT transfer before it
         * completed, STALL the endpoint in either case
         */

        priv->ep0state = EP0STATE_STALLED;
        break;
    }
}

/****************************************************************************
 * Name: sam_ep0_done
 ****************************************************************************/

static inline void sam_ep0_done(struct sam_usbdev_s *priv, uint32_t intsta)
{
  uint32_t epr;

  /* Setup to NAK further requests on the endpoint */
#warning Missing logic

  /* Check the direction bit to determine if this the completion of an EP0
   * packet sent to or received from the host PC.
   */

  if ((intsta & USB_ISTR_DIR) == 0)
    {
      /* EP0 IN: device-to-host (DIR=0) */

      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0IN), intsta);
      sam_clrepctrtx(EP0);
      sam_ep0_in(priv);
    }
  else
    {
      /* EP0 OUT: host-to-device (DIR=1) */

      epr = sam_getreg(SAM_USB_EPR(EP0));

      /* CTR_TX is set when an IN transaction successfully
       * completes on an endpoint
       */

      if ((epr & USB_EPR_CTR_TX) != 0)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0INDONE), epr);
          sam_clrepctrtx(EP0);
          sam_ep0_in(priv);
        }

      /* SETUP is set by the hardware when the last completed
       * transaction was a control endpoint SETUP
       */

      else if ((epr & USB_EPR_SETUP) != 0)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0SETUPDONE), epr);
          sam_clrepctrrx(EP0);
          sam_ep0_setup(priv);
        }

      /* Set by the hardware when an OUT/SETUP transaction successfully
       * completed on this endpoint.
       */

      else if ((epr & USB_EPR_CTR_RX) != 0)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP0OUTDONE), epr);
          sam_clrepctrrx(EP0);
          sam_ep0_out(priv);
        }

      /* None of the above */

      else
        {
          usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EP0BADCTR), epr);
          return; /* Does this ever happen? */
        }
    }

  /* Make sure that the EP0 packet size is still OK (superstitious?) */

  sam_seteprxcount(EP0, SAM_EP0MAXPACKET);

  /* Handle the STALL condition: */

  if (priv->ep0state == EP0STATE_STALLED)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
    }
}

/****************************************************************************
 * Name: sam_lptransfer
 ****************************************************************************/

static void sam_lptransfer(struct sam_usbdev_s *priv)
{
  uint8_t  epno;
  uint32_t intsta;

  /* Stay in loop while LP interrupts are pending */

  while (((intsta = sam_getreg(SAM_UDPHS_INTSTA)) & USB_ISTR_CTR) != 0)
    {
      sam_putreg((uint32_t)~USB_ISTR_CTR, SAM_UDPHS_INTSTA);

      /* Extract highest priority endpoint number */

      epno = (uint8_t)(intsta & USB_ISTR_EPID_MASK);

      /* Handle EP0 completion events */

      if (epno == 0)
        {
          sam_ep0_done(priv, intsta);
        }

      /* Handle other endpoint completion events */

      else
        {
          sam_ep_done(priv, epno);
        }
    }
}

/****************************************************************************
 * Name: sam_dma_interrupt
 *
 * Description:
 *   Handle the UDPHS DMA interrupt
 *
 ****************************************************************************/

static void sam_dma_interrupt(struct sam_usbdev_s *priv, int epno)
{
  struct sam_ep_s  *privep;
  struct sam_req_s *privreq;
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t dmastatus;
  uint8_t *buf;
  int bufcnt;
  int xfrd;
  int16_t result = OK;

  /* Not all endpoints support DMA */

  DEBUGASSERT((unsigned)epno < SAM_UDPHS_NENDPOINTS &&
              (SAM_EPSET_DMA & SAM_EP_BIT(epno)) != 0);

  /* Get the endpoint structure */

  privep = &priv->eplist[epno];

  /* Get the request from the head of the endpoint request queue */

  privreq = sam_rqpeek(privep);
  DEBUGASSERT(privreg);

  /* Invalidate the data cache for region that just completed DMA.
   * This will force the buffer data to be reloaded from RAM.
   */

  buf = &privreq->req.buf[privreq->req.xfrd];
  cp15_invalidate_dcache((uintptr_t)buf, (uintptr_t)buf + privreg->inflight);

  /* Get the result of the DMA operation */

  dmastatus = pUdp->UDPHS_DMA[epno].UDPHS_DMASTATUS;
  uvdbg("DMA%d DMASTATUS: %08x\n", epno, dmastatus);

  /* Disable DMA interrupt to avoid receiving 2 (B_EN and TR_EN) */

  regaddr = SAM_UDPHS_DMACONTROL(epno)
  regval  = sam_getreg(regaddr);
  regval &= ~(UDPHS_DMACONTROL_ENDTREN | UDPHS_DMACONTROL_ENDBEN);
  sam_putreg(regval, regaddr);

  /* Check for end of buffer buffer.  Set by hardware when the
   * BUFF_COUNT downcount reach zero.
   */

  if ((dmastatus & UDPHS_DMASTATUS_ENDBFST) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DMAEOB), (uint16_t)dmastatus);

      /* BUFF_COUNT holds the number of untransmitted bytes. BUFF_COUNT is
       * equal to zero in case of good transfer
       */

      bufcnt = (dmastatus & UDPHS_DMASTATUS_BUFCNT_MASK)
                >> UDPHS_DMASTATUS_BUFCNT_SHIFT;

      xfrd               = privreq->inflight - bufcnt;
      privreq->req.xfrd += xfrd;
      privreq->inflight  = bufcnt;

      /* Is there more data to send? */

      bufcnt = privreq->req.len - privreq->req.xfrd - privreq->inflight;
      if (bufcnt > 0)
        {
          /* Yes, clip to the size of the DMA FIFO */

          if (bufcnt > DMA_MAX_FIFO_SIZE)
            {
              privreq->inflight = DMA_MAX_FIFO_SIZE;
            }
          else
            {
              privreq->inflight = bufcnt;
            }

          /* And perform the DMA transfer */

          regval = UDPHS_DMACONTROL_ENDTREN | UDPHS_DMACONTROL_ENDTRIT |
                   UDPHS_DMACONTROL_ENDBEN | UDPHS_DMACONTROL_ENDBUFFIT |
                   UDPHS_DMACONTROL_CHANNENB;
          sam_dma_single(epno, privreq, regval);
        }
    }

  /* Check for end of channel transfer. Set by hardware when the last
   * packet transfer is complete
   */

  else if ((dmastatus & UDPHS_DMASTATUS_ENDTRST) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DMAEOC), (uint16_t)dmastatus);

      /* Get the number of btyes transferred from the DMA status */

      bufcnt = ((dmastatus & UDPHS_DMASTATUS_BUFCNT_MASK)
                >> UDPHS_DMASTATUS_BUFCNT_SHIFT);

      xfrd               = privreq->inflight - bufcnt;
      privreq->req.xfrd += xfrd;
      privreq->inflight -= bufcnt;
    }
  else
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DMAERR), (uint16_t)dmastatus);
      result = -EIO;
    }

  /* Check if we are finished with this requrest */

  if (privreq->req.len == privreq->req.xfrd)
    {
      /* Return the request buffer to the class implementation */

      sam_req_complete(privep, result);

      /* Try to start the transfer for the next request */
#warning Missing logic
    }
}

/****************************************************************************
 * Name: sam_ep_interrupt
 *
 * Description:
 *   Handle the UDPHS endpoint interrupt
 *
 ****************************************************************************/

static void sam_ep_interrupt(struct sam_usbdev_s *priv, int epno)
{
  struct sam_ep_s *privep;
  struct sam_req_s *privreq;
  struct usb_ctrlreq_s *req;
  uint32_t reqbuf[2];
  uint32_t eptsta;
  uint32_t eptype;
  uint32_t regval;
  uint16_t pktsize;

  DEBUGASSERT((unsigned)epno < SAM_UDPHS_NENDPOINTS);

  /* Get the endpoint structure */

  privep = &priv->eplist[epno];

  /* Get the request from the head of the endpoint request queue */

  privreq = sam_rqpeek(privep);
  DEBUGASSERT(privreg);

  /* Get the endpoint status */

  eptsta = sam_getreg(SAM_UDPHS_EPTSTA(epno));

  /* Get the endpoint type */

  regval =  sam_getreg(SAM_UDPHS_EPTCFG(epno))
  eptype = regval & UDPHS_EPTCFG_TYPE_MASK;

  req    = (struct usb_ctrlreq_s *)reqbuf;

  /* IN packet sent */

  if ((sam_getreg(SAM_UDPHS_EPTCTL(epno)) & UDPHS_EPTCTL_TXRDY) != 0 &&
      (eptsta & UDPHS_EPTSTA_TXRDY) == 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_TXRDY), (uint16_t)eptsta);

      /* Sending state */

      if (privep->epstate == UDPHS_EPSTATE_SENDING)
        {
          ssize_t remaining;

          /* Were their bytes in flight? */

          if (privreq->inflight)
            {
              privreq->req.xfrd += privreq->inflight;
              privreq->inflight = 0;
            }

          /* Check for a zero-length data transfer */

          if (privreq->req.len == 0 &&
             (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0)
            {
              privep->txnullpkt = 1;
            }

          /* Check for the end of transfer */

          if (privreq->req.xfrd == privreq->req.len ||
              privep->txnullpkt != 0)
            {
              privep->txnullpkt = 1;

              /* Send next packet */

              (void)sam_req_wrdma(priv, privep);
              sam_putreg(UDPHS_EPTSETSTA_TXRDY, SAM_UDPHS_EPTSETSTA(ep));
            }
          else
            {
              /* Disable interrupt on none-control EP */

              if (eptype != UDPHS_EPTCFG_TYPE_CTRL8)
                {
                  regval  = sam_getreg(SAM_UDPHS_IEN);
                  regval &= ~UDPHS_INT_EPT(epno);
                  sam_putreg(regval, SAM_UDPHS_IEN);
                }

              sam_putreg(UDPHS_EPTCTL_TXRDY, SAM_UDPHS_EPTCTLDIS(epno));
              sam_req_complete(privep, OK);
              privep->txnullpkt = 0;
            }
        }
      else
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_TXRDYERR), privep->txnullpkt);
        }
    }

  /* OUT packet received */

  if ((eptsta & UDPHS_EPTSTA_RXRDYTXKL) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_RXRDY), (uint16_t)eptsta);

      /* NOT in receiving state */

      if (privep->epstate != UDPHS_EPSTATE_RECEIVING)
        {
          /* Check if ACK received on a Control EP */

          if (eptype == UDPHS_EPTCFG_TYPE_CTRL8 &&
              (eptsta & UDPHS_EPTSTA_BYTECNT_MASK) == 0)
            {
              sam_putreg(UDPHS_EPTSTA_RXRDYTXKL, SAM_UDPHS_EPTCLRSTA(ep));
              sam_req_complete(privep, OK);
            }

          /* Data has been STALLed */

          else if ((epsta & UDPHS_EPTSTA_FRCESTALL) != 0)
            {
              sam_putreg(UDPHS_EPTSTA_RXRDYTXKL, SAM_UDPHS_EPTCLRSTA(ep));
            }

          /* NAK the data */

          else
            {
              regval  = sam_getreg(SAM_UDPHS_IEN);
              regval &= ~UDPHS_INT_EPT(epno);
              sam_putreg(regval, SAM_UDPHS_IEN);
            }
        }

      /* In read state */

      else
        {
          pktsize = (uint16_t)
            ((eptsta & UDPHS_EPTSTA_BYTECNT_MASK) >> UDPHS_EPTSTA_BYTECNT_SHIFT);

          sam_ep0_read(priv, privep);
          sam_putreg(UDPHS_EPTSTA_RXRDYTXKL, SAM_UDPHS_EPTCLRSTA(ep));

          /* Check if transfer is finished */

          if (privreq->req.xfrd == privreq->req.len ||
              pktsize < privep->ep.maxpacket)
            {
              sam_putreg(UDPHS_EPTCTL_RXRDYTXKL, SAM_UDPHS_EPTCTLDIS(epno));

              /* Disable interrupt if not control EP */

              if (UDPHS_EPTCFG_TYPE_CTRL8 != eptype)
                {
                  regval  = sam_getreg(SAM_UDPHS_IEN);
                  regval &= ~UDPHS_INT_EPT(epno);
                  sam_putreg(regval, SAM_UDPHS_IEN);
                }

              sam_req_complete(privep, OK);
            }
        }
    }

  /* STALL sent */

  if ((eptsta & UDPHS_EPTSTA_STALLSNT) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_STALLSNT), (uint16_t)eptsta);

      /* Acknowledge */

      sam_putreg(UDPHS_EPTSTA_STALLSNT, SAM_UDPHS_EPTCLRSTA(ep));

      /* ISO error */

      if (eptype == UDPHS_EPTCFG_EPT_TYPE_ISO)
        {
          sam_req_complete(privep, -EIO);
        }

      /* If EP is not halted, clear STALL */

      else
        {
          if (privep->epstate != UDPHS_EPSTATE_HALTED)
            {
              sam_putreg(UDPHS_EPTSTA_FRCESTALL, SAM_UDPHS_EPTCLRSTA(ep));
            }
        }
    }

  /* SETUP packet received */

  if ((eptsta & UDPHS_EPTSTA_RXSETUP) != 0)
    {
      usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_RXSETUP), (uint16_t)eptsta);

      /* If a transfer was pending, complete it Handles the case where during
       * the status phase of a control write transfer, the host receives the
       * device ZLP and ack it, but the ack is not received by the device
       */

      if (privep->epstate == UDPHS_EPSTATE_RECEIVING ||
          privep->epstate == UDPHS_EPSTATE_SENDING)
        {
          sam_req_complete(privep, OK);
        }

      /* ISO Err Flow */

      if (eptype == UDPHS_EPTCFG_EPT_TYPE_ISO)
        {
          /* Acknowledge setup packet */

          sam_putreg(UDPHS_EPTSTA_RXSETUP, SAM_UDPHS_EPTCLRSTA(ep));
        }
      else
        {
          /* Copy setup */

          sam_ep0_read(priv->ctrl);

          /* Acknowledge setup packet */

          sam_putreg(UDPHS_EPTSTA_RXSETUP, SAM_UDPHS_EPTCLRSTA(ep));

          /* Handle the EP0 SETUP command */

          sam_ep0_setup(priv);
        }
    }
}

/****************************************************************************
 * Name: sam_udphs_interrupt
 *
 * Description:
 *   Handle the UDPHS interrupt
 *
 ****************************************************************************/

static int sam_udphs_interrupt(int irq, void *context)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple UDPHS controllers
   * easier.
   */

  struct sam_usbdev_s *priv = &g_usbdev;
  uint32_t intsta;
  uint32_t ien;
  uint32_t pending;
  int i;

  /* Get the set of pending interrupts */

  intsta = sam_getreg(SAM_UDPHS_INTSTA);
  usbtrace(TRACE_INTENTRY(SAM_TRACEINTID_INTERRUPT), intsta);

  inten   = sam_getreg(SAM_UDPHS_IEN);
  pending = insta & inten;

  /* Handle all pending UDPHS interrupts (and new interrupts that become
   * pending)
   */

  while (pending)
    {
      usbtrace(TRACE_INTENTRY(SAM_TRACEINTID_INTERRUPT), intsta);

      /* Suspend, treated last */

      if ((pending == UDPHS_INT_DETSUSPD) != 0)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DETSUSPD), (uint16_t)pending);

          /* Enable wakeup interrupts */

          regval  = inten;
          regval &= ~UDPHS_INT_DETSUSPD;
          regval |= (UDPHS_INT_WAKEUP | UDPHS_INT_ENDOFRSM);
          sam_putreg(regval, SAM_UDPHS_IEN);

          /* Acknowledge interrupt */

          sam_putreg(UDPHS_INT_DETSUSPD | UDPHS_INT_WAKEUP, SAM_UDPHS_CLRINT);
          sam_suspend(priv);
        }

      /* SOF interrupt*/

      else if ((pending & UDPHS_INT_INTSOF) != 0)
        {
          /* Acknowledge interrupt */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_INTSOF), (uint16_t)pending);
          sam_putreg(UDPHS_INT_INTSOF, SAM_UDPHS_CLRINT);
        }

      /* Resume */

      else if ((pending & UDPHS_INT_WAKEUP) != 0 ||
               (pending & UDPHS_INT_ENDOFRSM) != 0)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_WAKEUP), (uint16_t)pending);
          sam_resume(priv);

          /* Acknowledge interrupt */

          sam_putreg(UDPHS_INT_WAKEUP | UDPHS_INT_ENDOFRSM | UDPHS_INT_DETSUSPD,
                     SAM_UDPHS_CLRINT);

          /* Enable suspend interrupts */

          inten &= ~UDPHS_INT_WAKEUP;
          inten |= (UDPHS_INT_ENDOFRSM | UDPHS_INT_DETSUSPD);
          sam_putreg(inten, SAM_UDPHS_IEN);
        }

      /* Bus reset */

      if ((pending & UDPHS_INT_ENDRESET) != 0)
        {
          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_ENDRESET), (uint16_t)pending);

          /* Clear and enable the suspend interrupt */

          sam_putreg(UDPHS_INT_WAKEUP | UDPHS_INT_DETSUSPD, SAM_UDPHS_CLRINT);

          inten |= UDPHS_INT_DETSUSPD;
          sam_putreg(inten, SAM_UDPHS_IEN);

          /* Handle the reset */

          sam_reset(priv);

          /* Acknowledge the interrupt */

          sam_putreg(UDPHS_INT_ENDRESET, SAM_UDPHS_CLRINT);
        }

      /* Upstream resume */

      else if ((pending & UDPHS_INT_UPSTRRES) != 0)
        {
          /* Acknowledge interrupt */

          usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_UPSTRRES), (uint16_t)pending);
          sam_putreg(UDPHS_INT_UPSTRRES, SAM_UDPHS_CLRINT);
        }

      /* DMA interrupts */

      if ((pending & UDPHS_INT_DMA_MASK) != 0)
        {
          for (i = 1; i <= SAM_UDPHS_NDMACHANNELS; i++)
            {
              if ((pending & UDPHS_INT_DMA(i)) != 0)
                {
                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_DMA), (uint16_t)i);
                  sam_dma_interrupt(priv, i);
                }
            }
        }

      /* Endpoint Interrupts */

      if ((pending & UDPHS_INT_EPT_MASK) != 0)
        {
          for (i = 1; i <= SAM_UDPHS_NENDPOINTS; i++)
            {
              if ((pending & UDPHS_INT_EPT(i)) != 0)
                {
                  usbtrace(TRACE_INTDECODE(SAM_TRACEINTID_EP), (uint16_t)i);
                  sam_ep_interrupt(priv, i);
                }
            }
        }

      /* Re-sample the set of pending interrupts */

      intsta  = sam_getreg(SAM_UDPHS_INTSTA);
      inten   = sam_getreg(SAM_UDPHS_IEN);
      pending = insta & inten;
    }

  usbtrace(TRACE_INTEXIT(SAM_TRACEINTID_INTERRUPT), insta);
  return OK;
}

/****************************************************************************
 * Name: sam_setimask
 ****************************************************************************/

static void
sam_setimask(struct sam_usbdev_s *priv, uint16_t setbits, uint16_t clrbits)
{
  uint32_t regval;

  /* Adjust the interrupt mask bits in the shadow copy first */

  priv->imask &= ~clrbits;
  priv->imask |= setbits;

  /* Then make the interrupt mask bits in the CNTR register match the shadow
   * register (Hmmm... who is shadowing whom?)
   */

  regval  = sam_getreg(SAM_USB_CNTR);
  regval &= ~USB_CNTR_ALLINTS;
  regval |= priv->imask;
  sam_putreg(regval, SAM_USB_CNTR);
}

/****************************************************************************
 * Suspend/Resume Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: sam_suspend
 ****************************************************************************/

static void sam_suspend(struct sam_usbdev_s *priv)
{
  uint32_t regval;

  /* Don't do anything if the device is already suspended */

  if (priv->devstate != UDPHS_DEVSTATE_SUSPENDED)
    {
      /* Notify the class driver of the suspend event */

      if (priv->driver)
        {
          CLASS_SUSPEND(priv->driver, &priv->usbdev);
        }

      /* Switch to the Suspended state */

      priv->prevstate = priv->devstate;
      priv->devstate  = UDPHS_DEVSTATE_SUSPENDED;

      /* The Atmel sample code disables USB clocking here (via the PMC
       * CKGR_UCKR).  However, we cannot really do that here because that
       * clocking is also needed by the UHPHS host.
       */
#warning REVISIT

      /* Disable clocking to the UDPHS peripheral */

      sam_udphs_disableclk();

      /* Let the board-specific logic know that we have entered the
       * suspend state.  This may trigger additional reduced power
       * consumuption measures.
       */

      sam_usbsuspend((struct usbdev_s *)priv, false);
    }
}

/****************************************************************************
 * Name: sam_resume
 ****************************************************************************/

static void sam_resume(struct sam_usbdev_s *priv)
{
  uint32_t regval;

  /* This function is called when either (1) a WKUP interrupt is received from
   * the host PC, or (2) the class device implementation calls the wakeup()
   * method.
   */

  /* Don't do anything if the device was not suspended */

  if (priv->devstate == UDPHS_DEVSTATE_SUSPENDED)
    {
      /* Enable clocking to the UDPHS peripheral. */

      sam_udphs_enableclk();

      /* In the Atmel example code, they also enable USB clocking
       * at this point (via the BIAS in the CKGR_UCKR register).  In this
       * implementation, that should not be necessary here because we
       * never disable BIAS to begin with.
       */
#warning REVISIT

      priv->devstate = priv->prevstate;

      /* Restore full power -- whatever that means for this particular board */

      sam_usbsuspend((struct usbdev_s *)priv, true);

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
 * Name: sam_epset_reset
 ****************************************************************************/

void sam_epset_reset(struct sam_usbdev_s *priv, uint16_t epset)
{
  struct sam_ep_s *privep;
  uint32_t bit;
  int epno;

  /* Reset each endpoint in the set */

  for (epno = 0, bit = 1; epno < SAM_UDPHS_NENDPOINTS; epno++)
    {
      if ((epset & bit) != 0)
        {
          privep = &priv->eplist[epno];

          /* Disable endpoint interrupt */

          regval = sam_getreg(SAM_UDPHS_IEN);
          regval &= ~UDPHS_INT_EPT(epno);
          sam_putreg(regval, SAM_UDPHS_IEN);

          /* Cancel any queued requests.  Since they are canceled
           * with status -ESHUTDOWN, then will not be requeued
           * until the configuration is reset.  NOTE:  This should
           * not be necessary... the CLASS_DISCONNECT above should
           * result in the class implementation calling sam_ep_disable
           * for each of its configured endpoints.
           */

          sam_req_cancel(privep);

          /* Reset endpoint */

          sam_putreg(UDPHS_EPTRST(epno), SAM_UDPHS_EPTRST);

          /* Reset endpoint status */

          privep->epstate   = UDPHS_EPSTATE_DISABLED;
          privep->stalled   = false;
          privep->halted    = false;
          privep->txbusy    = false;
          privep->txnullpkt = false;
          privep->bank      = 0;
        }

      bit <<= 1;
    }
}

/****************************************************************************
 * Name: sam_ep_reserve
 ****************************************************************************/

static inline struct sam_ep_s *
sam_ep_reserve(struct sam_usbdev_s *priv, uint8_t epset)
{
  struct sam_ep_s *privep = NULL;
  irqstate_t flags;
  int epndx = 0;

  flags = irqsave();
  epset &= priv->epavail;
  if (epset)
    {
      /* Select the lowest bit in the set of matching, available endpoints
       * (skipping EP0)
       */

      for (epndx = 1; epndx < SAM_UDPHS_NENDPOINTS; epndx++)
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
 ****************************************************************************/

static inline bool
sam_ep_reserved(struct sam_usbdev_s *priv, int epno)
{
  return ((priv->epavail & SAM_EP_BIT(epno)) == 0);
}

/****************************************************************************
 * Endpoint operations
 ****************************************************************************/
/****************************************************************************
 * Name: sam_ep_configure
 ****************************************************************************/

static int sam_ep_configure(struct usbdev_ep_s *ep,
                            const struct usb_epdesc_s *desc,
                            bool last)
{
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
  uint16_t pma;
  uint16_t setting;
  uint16_t maxpacket;
  uint8_t  epno;

#ifdef CONFIG_DEBUG
  if (!ep || !desc)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      ulldbg("ERROR: ep=%p desc=%p\n");
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
      setting = USB_EPR_EPTYPE_INTERRUPT;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      setting = USB_EPR_EPTYPE_BULK;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
#warning "REVISIT: Need to review isochronous EP setup"
      setting = USB_EPR_EPTYPE_ISOC;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint */
      setting = USB_EPR_EPTYPE_CONTROL;
      break;

    default:
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_BADEPTYPE), (uint16_t)desc->type);
      return -EINVAL;
    }

  sam_seteptype(epno, setting);

  /* Get the maxpacket size of the endpoint. */

  maxpacket = GETUINT16(desc->mxpacketsize);
  DEBUGASSERT(maxpacket <= SAM_MAXPACKET_SIZE);
  ep->maxpacket = maxpacket;

  /* Get the subset matching the requested direction */

  if (USB_ISEPIN(desc->addr))
    {
      /* The full, logical EP number includes direction */

      ep->eplog = USB_EPIN(epno);
#warning Missing logic
    }
  else
    {
      /* The full, logical EP number includes direction */

      ep->eplog = USB_EPOUT(epno);
#warning Missing logic
    }

   sam_dumpep(priv, epno);
   return OK;
}

/****************************************************************************
 * Name: sam_ep_disable
 ****************************************************************************/

static int sam_ep_disable(struct usbdev_ep_s *ep)
{
  struct sam_ep_s *privep = (struct sam_ep_s *)ep;
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

  /* Cancel any ongoing activity */

  flags = irqsave();
  sam_req_cancel(privep);

  /* Disable TX; disable RX */
#warning Missing logic

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_ep_allocreq
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

  privreq = (struct sam_req_s *)kmalloc(sizeof(struct sam_req_s));
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

  kfree(privreq);
}

/****************************************************************************
 * Name: sam_ep_submit
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

  epno        = USB_EPNO(ep->eplog);
  req->result = -EINPROGRESS;
  req->xfrd   = 0;
  flags       = irqsave();

  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      sam_req_abort(privep, privreq, -EBUSY);
      ulldbg("ERROR: stalled\n");
      ret = -EBUSY;
    }

  /* Handle IN (device-to-host) requests.  NOTE:  If the class device is
   * using the bi-directional EP0, then we assume that they intend the EP0
   * IN functionality.
   */

  else if (USB_ISEPIN(ep->eplog) || epno == EP0)
    {
      /* Add the new request to the request queue for the IN endpoint */

      sam_req_enqueue(privep, privreq);
      usbtrace(TRACE_INREQQUEUED(epno), req->len);

      /* If the IN endpoint FIFO is available, then transfer the data now */

      if (!privep->txbusy)
        {
          ret = sam_req_write(priv, privep);
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      privep->txnullpkt = 0;
      sam_req_enqueue(privep, privreq);
      usbtrace(TRACE_OUTREQQUEUED(epno), req->len);

      /* This there a incoming data pending the availability of a request? */

      if (priv->rxpending)
        {
          /* NAK any OUT request addressed to the endpoint */
#warning Missing logic

          /* Data is no longer pending */

          priv->rxpending = false;
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
  struct sam_usbdev_s *priv;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));
  priv = privep->dev;

  flags = irqsave();
  sam_req_cancel(privep);
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_ep_stall
 ****************************************************************************/

static int sam_ep_stall(struct usbdev_ep_s *ep, bool resume)
{
  struct sam_ep_s *privep;
  struct sam_usbdev_s *priv;
  uint8_t epno = USB_EPNO(ep->eplog);
  uint16_t status;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  privep = (struct sam_ep_s *)ep;
  priv   = (struct sam_usbdev_s *)privep->dev;
  epno   = USB_EPNO(ep->eplog);

  /* STALL or RESUME the endpoint */

  flags = irqsave();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, USB_EPNO(ep->eplog));

  /* Get status of the endpoint; stall the request if the endpoint is
   * disabled
   */
#warning Missing logic


  if (status == 0)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_EPDISABLED), 0);

      if (epno == 0)
        {
          priv->ep0state = EP0STATE_STALLED;
        }

      return -ENODEV;
    }

  /* Handle the resume condition */

  if (resume)
    {
      /* Resuming a stalled endpoint */

      usbtrace(TRACE_EPRESUME, epno);
      privep->stalled = false;

      if (USB_ISEPIN(ep->eplog))
        {
          /* IN endpoint */

          if (sam_eptxstalled(epno))
            {
              sam_clrtxdtog(epno);

              /* Restart any queued write requests */

              (void)sam_req_write(priv, privep);
            }
        }
      else
        {
          /* OUT endpoint */

          if (sam_eprxstalled(epno))
            {
              if (epno == EP0)
                {
                  /* After clear the STALL, enable the default endpoint receiver */

                  sam_seteprxcount(epno, ep->maxpacket);
                }
              else
                {
                  sam_clrrxdtog(epno);
                }
            }
        }
    }

  /* Handle the stall condition */

  else
    {
      usbtrace(TRACE_EPSTALL, epno);
      privep->stalled = true;
#warning Missing logic
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Device Controller Operations
 ****************************************************************************/
/****************************************************************************
 * Name: sam_allocep
 ****************************************************************************/

static struct usbdev_ep_s *sam_allocep(struct usbdev_s *dev, uint8_t epno,
                                         bool in, uint8_t eptype)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;
  struct sam_ep_s *privep = NULL;
  uint8_t epset = SAM_EPSET_ALL;

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

      if (epno >= SAM_UDPHS_NENDPOINTS)
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
      goto errout;
    }
  epno = USB_EPNO(privep->ep.eplog);

  /* Allocate a PMA buffer for this endpoint */
#warning Missing logic

  return &privep->ep;

errout_with_ep:
  sam_ep_unreserve(priv, privep);
errout:
  return NULL;
}

/****************************************************************************
 * Name: sam_freeep
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
      /* Free the PMA buffer assigned to this endpoint */
#warning Missing logic

      /* Mark the endpoint as available */

      sam_ep_unreserve(priv, privep);
    }
}

/****************************************************************************
 * Name: sam_getframe
 ****************************************************************************/

static int sam_getframe(struct usbdev_s *dev)
{
  uint16_t fnr;

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Return the last frame number detected by the hardware */

  fnr = sam_getreg(SAM_USB_FNR);
  usbtrace(TRACE_DEVGETFRAME, fnr);
  return (fnr & USB_FNR_FN_MASK);
}

/****************************************************************************
 * Name: sam_wakeup
 ****************************************************************************/

static int sam_wakeup(struct usbdev_s *dev)
{
  struct sam_usbdev_s *priv = (struct sam_usbdev_s *)dev;
  irqstate_t flags;

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
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: sam_selfpowered
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
 * Initialization/Reset
 ****************************************************************************/

/****************************************************************************
 * Name: sam_reset
 ****************************************************************************/

static void sam_reset(struct sam_usbdev_s *priv)
{
  /* Make sure that clocking is eanbled to the UDPHS peripheral. */

  sam_udphs_enableclk();

  /* In the Atmel example code, they also enable USB clocking
   * at this point (via the BIAS in the CKGR_UCKR register).  In this
   * implementation, that should not be necessary here because we
   * never disable BIAS to begin with.
   */
#warning REVISIT

  /* Tell the class driver that we are disconnected.  The class driver
   * should then accept any new configurations.
   */

  CLASS_DISCONNECT(priv->driver, &priv->usbdev);

  /* The device enters the Default state */

  priv->devstate  = UDPHS_DEVSTATE_DEFAULT;
  priv->ep0state  = EP0STATE_IDLE;
  priv->rxpending = false;

  /* Reset and disable all endpoints other.  Then re-configure EP0 */

  sam_epset_reset(priv, SAM_EPSET_ALL);
  sam_ep_configure((struct usbdev_ep_s *ep)&priv->eplist[EP0],
                   &g_ep0desc, false);
  /* Reset endpoints */

  for (epno = 0; epno < SAM_UDPHS_NENDPOINTS; epno++)
    {
      struct sam_ep_s *privep = &priv->eplist[epno];

      /* Cancel any queued requests.  Since they are canceled
       * with status -ESHUTDOWN, then will not be requeued
       * until the configuration is reset.  NOTE:  This should
       * not be necessary... the CLASS_DISCONNECT above should
       * result in the class implementation calling sam_ep_disable
       * for each of its configured endpoints.
       */

      sam_req_cancel(privep);

      /* Reset endpoint status */

      privep->stalled   = false;
      privep->halted    = false;
      privep->txbusy    = false;
      privep->txnullpkt = false;
    }

  /* Re-configure the USB controller in its initial, unconnected state */

  sam_hw_reset(priv);
  priv->usbdev.speed = USB_SPEED_FULL;
}

/****************************************************************************
 * Name: sam_hw_reset
 ****************************************************************************/

static void sam_hw_reset(struct sam_usbdev_s *priv)
{
  /* Put the USB controller into reset, clear all interrupt enables */

  sam_putreg(USB_CNTR_FRES, SAM_USB_CNTR);

  /* Disable interrupts (and perhaps take the USB controller out of reset) */

  priv->imask = 0;
  sam_putreg(priv->imask, SAM_USB_CNTR);

  /* Set the SAM BTABLE address */
# warning Missing logic

  /* Initialize EP0 */
#warning Missing logic

  /* Set the device to respond on default address */

  sam_setdevaddr(priv, 0);

  /* Clear any pending interrupts */

  sam_putreg(0, SAM_UDPHS_INTSTA);

  /* Enable interrupts at the USB controller */

  sam_setimask(priv, SAM_CNTR_SETUP, (USB_CNTR_ALLINTS & ~SAM_CNTR_SETUP));
  sam_dumpep(priv, EP0);
}

/****************************************************************************
 * Name: sam_hw_setup
 ****************************************************************************/

static void sam_hw_setup(struct sam_usbdev_s *priv)
{
  uint32_t regval;
  int i;

  /* Paragraph 32.5.1, "Power Management".  The UDPHS is not continuously
   * clocked.  For using the UDPHS, the programmer must first enable the
   * UDPHS Clock in the Power Management Controller (PMC_PCER register).
   * Then enable the PLL (PMC_UCKR register). Finally, enable BIAS in
   * PMC_UCKR register. However, if the application does not require UDPHS
   * operations, the UDPHS clock can be stopped when not needed and
   * restarted later.
   *
   * Here, we set only the PCER.  PLL configuration was performed in
   * sam_clockconfig() earlier in the boot sequence.
   */

  sam_udphs_enableclk();

  /* Reset and disable endpoints */

  sam_epset_reset(priv, SAM_EPSET_ALL);

  /* Configure the pull-up on D+ and disconnect it */

  regval  = sam_getreg(SAM_UDPHS_CTRL);
  regval |= UDPHS_CTRL_DETACH;
  sam_putreg(regval, SAM_UDPHS_CTRL);

  regval |= UDPHS_CTRL_PULLDDIS;
  sam_putreg(regval, SAM_UDPHS_CTRL);

  /* Reset the UDPHS block
   *
   * Paragraph 33.5.1.  "One transceiver is shared with the USB High Speed
   *   Device (port A). The selection between Host Port A and USB Device is
   *   controlled by the UDPHS enable bit (EN_UDPHS) located in the UDPHS_CTRL
   *   control register.
   *
   *  "In the case the port A is driven by the USB High Speed Device, the ...
   *   transceiver is automatically selected for Device operation once the
   *   USB High Speed Device is enabled."
   */

  regval &= ~UDPHS_CTRL_ENUDPHS;
  sam_putreg(regval, SAM_UDPHS_CTRL);

  regval |= UDPHS_CTRL_ENUDPHS;
  sam_putreg(regval, SAM_UDPHS_CTRL);

  /* REVISIT: Per recommendations and sample code, USB clocking (as
   * configured in the PMC CKGR_UCKR) is set up after reseting the UDHPS.
   * However, that initialation has already been done in sam_clockconfig().
   * Also, that clocking is shared with the UHPHS USB host logic; the
   * device logica cannot autonomously control USB clocking.
   */

  /* Initialize DMA channels */

  for (i = 1; i < SAM_UDPHS_NDMACHANNELS; i++)
    {
      /* Stop any DMA transfer */

      sam_putreg(0, SAM_UDPHS_DMACONTROL(i));

      /* Reset DMA channel (Buffer count and Control field) */

      sam_putreg(UDPHS_DMACONTROL_LDNXTDSC, SAM_UDPHS_DMACONTROL(i));

      /* Reset DMA channel */

      sam_putreg(0, SAM_UDPHS_DMACONTROL(i));

      /* Clear DMA channel status (read to clear) */

      regval = sam_getreg(SAM_UDPHS_DMASTATUS(i));
      sam_putreg(regval, SAM_UDPHS_DMACONTROL(i));
    }

  /* Initialize DMA channels */

  for (i = 1; i < SAM_UDPHS_NENDPOINTS; i++)
    {
      /* Disable endpoint */

      regval = UDPHS_EPTCTL_SHRTPCKT | UDPHS_EPTCTL_BUSYBANK |
               UDPHS_EPTCTL_NAKOUT | UDPHS_EPTCTL_NAKI |
               UDPHS_EPTCTL_STALLSNT | UDPHS_EPTCTL_STALLSNT |
               UDPHS_EPTCTL_TXRDY | UDPHS_EPTCTL_TXCOMPLT |
               UDPHS_EPTCTL_RXRDYTXKL | UDPHS_EPTCTL_ERROVFLW |
               UDPHS_EPTCTL_MDATARX | UDPHS_EPTCTL_DATAXRX |
               UDPHS_EPTCTL_NYETDIS | UDPHS_EPTCTL_INTDISDMA |
               UDPHS_EPTCTL_AUTOVALID | UDPHS_EPTCTL_EPTENABL;
      sam_putreg(regval, SAM_UDPHS_EPTCTLDIS(i));

      /* Clear endpoint status */

      regval = UDPHS_EPTSTA_TOGGLESQ_MASK | UDPHS_EPTSTA_FRCESTALL |
               UDPHS_EPTSTA_RXRDYTXKL | UDPHS_EPTSTA_TXCOMPLT |
               UDPHS_EPTSTA_RXSETUP | UDPHS_EPTSTA_STALLSNT |
               UDPHS_EPTSTA_NAKIN | UDPHS_EPTSTA_NAKOUT;
      sam_putreg(regval, SAM_UDPHS_EPTCLRSTA(i));

      /* Reset endpoint configuration */

      sam_putreg(0, SAM_UDPHS_EPTCTLENB(i));
    }

  /* Normal mode (full speed not forced) */

  sam_putreg(0, SAM_UDPHS_TST);

  /* Disable all interrupts */

  sam_putreg(0, SAM_UDPHS_IEN);

  /* Clear all pending interrupt status */

  regval = UDPHS_INT_UPSTRRES | UDPHS_INT_ENDOFRSM | UDPHS_INT_WAKEUP |
           UDPHS_INT_ENDRESET | UDPHS_INT_INTSOF | UDPHS_INT_MICROSOF |
           UDPHS_INT_DETSUSPD;
  sam_putreg(regval, SAM_UDPHS_CLRINT);

  /* Enable interrupts */

  regval = UDPHS_IEN_ENDOFRSM | UDPHS_IEN_WAKE_UP | UDPHS_IEN_DET_SUSPD;
  sam_putreg(regval, SAM_UDPHS_IEN);

  /* The Atmel sample code disables USB clocking here (via the PMC
   * CKGR_UCKR).  However, we cannot really do that here because that
   * clocking is also needed by the UHPHS host.
   */
}

/****************************************************************************
 * Name: sam_sw_setup
 ****************************************************************************/

static void sam_sw_setup(struct sam_usbdev_s *priv)
{
  int epno;
  int i;

#ifdef CONFIG_SAMA5_UDPHS_SCATTERGATHER
#ifndef CONFIG_SAMA5_EHCI_PREALLOCATE
  /* Allocate a pool of free DMA transfer descriptors */

  priv->dtdpool = (struct sam_dtd_s *)
    kmemalign(16, CONFIG_SAMA5_UDPHS_NDTDS * sizeof(struct sam_dtd_s));
  if (!priv->dtdpool)
     {
      udbg("ERROR: Failed to allocate the DMA transfer descriptor pool\n");
      return NULL;
    }

  /* Initialize the list of free DMA transfer descriptors */

  for (i = 0; i < CONFIG_SAMA5_UDPHS_NDTDS; i++)
    {
      /* Put the transfer descriptor in a free list */

      sam_td_free(&priv->dtdpool[i]);
    }

#else
  /* Initialize the list of free DMA transfer descriptors */

  DEBUGASSERT(((uintptr_t)&g_dtdpool & 15) == 0);
  for (i = 0; i < CONFIG_SAMA5_UDPHS_NDTDS; i++)
    {
      /* Put the transfer descriptor in a free list */

      sam_td_free(&g_dtdpool[i]);
    }

#endif /* CONFIG_SAMA5_EHCI_PREALLOCATE */
#endif /* CONFIG_SAMA5_UDPHS_SCATTERGATHER */

  /* Initialize the device state structure.  NOTE: many fields
   * have the initial value of zero and, hence, are not explicitly
   * initialized here.
   */

  memset(priv, 0, sizeof(struct sam_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[EP0].ep;
  priv->epavail    = SAM_EPSET_ALL & ~SAM_EP_BIT(EP0);
  priv->bufavail   = SAM_BUFFER_ALLSET & ~SAM_BUFFER_EP0;
  priv->devstate   = UDPHS_DEVSTATE_SUSPENDED;
  priv->prevstate  = UDPHS_DEVSTATE_POWERED;

  /* Initialize the endpoint list */

  for (epno = 0; epno < SAM_UDPHS_NENDPOINTS; epno++)
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

      priv->eplist[epno].ep.maxpacket = SAM_MAXPACKET_SIZE;
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
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable all interrupts and force the USB controller into reset */

  sam_putreg(USB_CNTR_FRES, SAM_USB_CNTR);

  /* Clear any pending interrupts */

  sam_putreg(0, SAM_UDPHS_INTSTA);

  /* Disconnect the device / disable the pull-up */

  sam_usbpullup(&priv->usbdev, false);

  /* Power down the USB controller */

  sam_putreg(USB_CNTR_FRES|USB_CNTR_PDWN, SAM_USB_CNTR);
}

/****************************************************************************
 * Name: sam_sw_shutdown
 ****************************************************************************/

static void sam_sw_shutdown(struct sam_usbdev_s *priv)
{
#warning Missing logic
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

  struct sam_usbdev_s *priv = &g_usbdev;

  usbtrace(TRACE_DEVINIT, 0);

  /* Software initialization */

  sam_sw_setup(priv);

  /* Power up and initialize USB controller, but leave it in the reset
   * state.  Interrupts from the UDPHS controller are initialized here,
   * but will not be enabled at the AIC until the class driver is installed.
   */

  sam_hw_setup(priv);

  /* Attach USB controller interrupt handlers.  The hardware will not be
   * initialized and interrupts will not be enabled until the class device
   * driver is bound.  Getting the IRQs here only makes sure that we have
   * them when we need them later.
   */

  if (irq_attach(SAM_IRQ_UHPHS, sam_udphs_interrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(SAM_TRACEERR_IRQREGISTRATION),
               (uint16_t)SAM_IRQ_UDPHS);
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

  struct sam_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  flags = irqsave();
  usbtrace(TRACE_DEVUNINIT, 0);

  /* Disable and detach the UDPHS IRQ */

  up_disable_irq(SAM_IRQ_UDPHS);
  irq_detach(SAM_IRQ_UDPHS);

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

  struct sam_usbdev_s *priv = &g_usbdev;
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
      /* Setup the USB controller -- enabling interrupts at the USB controller */

      sam_hw_reset(priv);

      /* Enable USB controller interrupts at the NVIC */

      up_enable_irq(SAM_IRQ_UDPHS);

      /* Enable pull-up to connect the device.  The host should enumerate us
       * some time after this
       */

      sam_usbpullup(&priv->usbdev, true);
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

  struct sam_usbdev_s *priv = &g_usbdev;
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
  sam_reset(priv);

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts (but keep them attached) */

  up_disable_irq(SAM_IRQ_UDPHS);

  /* Put the hardware in an inactive state.  Then bring the hardware back up
   * in the reset state (this is probably not necessary, the sam_reset()
   * call above was probably sufficient).
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

#endif /* CONFIG_USBDEV && CONFIG_SAM_USB */
