/*******************************************************************************
 * arch/arm/src/lpc214x/lpc214x_usbdev.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/usb.h>
#include <nuttx/usbdev.h>
#include <nuttx/usbdev_trace.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "lpc214x_usbdev.h"
#include "lpc214x_power.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/* Configuration ***************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef  CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100  /* mA */
#endif

#define USB_SLOW_INT USBDEV_DEVINT_EPSLOW
#define USB_DEVSTATUS_INT USBDEV_DEVINT_DEVSTAT

#ifdef CONFIG_LPC214X_USBDEV_EPFAST_INTERRUPT
#  define USB_FAST_INT USBDEV_DEVINT_EPFAST
#else
#  define USB_FAST_INT 0
#endif

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#undef CONFIG_LPC214X_USBDEV_REGDEBUG

/* Enable reading SOF from interrupt handler vs. simply reading on demand.  Probably
 * a bad idea... Unless there is some issue with sampling the SOF from hardware
 * asynchronously.
 */

#ifdef CONFIG_LPC214X_USBDEV_FRAME_INTERRUPT
#  define USB_FRAME_INT USBDEV_DEVINT_FRAME
#else
#  define USB_FRAME_INT 0
#endif

#ifdef CONFIG_DEBUG
#  define USB_ERROR_INT USBDEV_DEVINT_EPRINT
#else
#  define USB_ERROR_INT 0
#endif

/* Number of DMA descriptors */

#ifdef CONFIG_LPC214X_USBDEV_DMA
# error DMA SUPPORT NOT YET FULLY IMPLEMENTED
#  ifndef CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS
#    define CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS 8
#  elif CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS > 30
#    define CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS 30
#  endif
#endif

/* Debug ***********************************************************************/

#ifndef CONFIG_USBDEV_TRACE
#  undef usbtrace
#  define usbtrace(a,b) ulldbg("%04x:%04x\n", a, b)
#endif

/* Trace error codes */

#define LPC214X_TRACEERR_ALLOCFAIL        0x0001
#define LPC214X_TRACEERR_BADEPNO          0x0002
#define LPC214X_TRACEERR_BADEPTYPE        0x0003
#define LPC214X_TRACEERR_BADREQUEST       0x0004
#define LPC214X_TRACEERR_BINDFAILED       0x0005
#define LPC214X_TRACEERR_DMABUSY          0x0006
#define LPC214X_TRACEERR_DRIVER           0x0007
#define LPC214X_TRACEERR_DRIVERREGISTERED 0x0008
#define LPC214X_TRACEERR_EPREAD           0x0009
#define LPC214X_TRACEERR_INVALIDPARMS     0x000a
#define LPC214X_TRACEERR_IRQREGISTRATION  0x000b
#define LPC214X_TRACEERR_NODMADESC        0x000c
#define LPC214X_TRACEERR_NOEP             0x000d
#define LPC214X_TRACEERR_NOTCONFIGURED    0x000e
#define LPC214X_TRACEERR_NULLPACKET       0x000f
#define LPC214X_TRACEERR_NULLREQUEST      0x0010
#define LPC214X_TRACEERR_STALLED          0x0011

/* Trace interrupt codes */

#define LPC214X_TRACEINTID_USB            0x0001
#define LPC214X_TRACEINTID_CLEARFEATURE   0x0002
#define LPC214X_TRACEINTID_CONNECTCHG     0x0003
#define LPC214X_TRACEINTID_CONNECTED      0x0004
#define LPC214X_TRACEINTID_DEVGETSTATUS   0x0005
#define LPC214X_TRACEINTID_DEVRESET       0x0006
#define LPC214X_TRACEINTID_DEVSTAT        0x0007
#define LPC214X_TRACEINTID_DISCONNECTED   0x0008
#define LPC214X_TRACEINTID_DISPATCH       0x0009
#define LPC214X_TRACEINTID_EP0IN          0x000a
#define LPC214X_TRACEINTID_EP0OUT         0x000b
#define LPC214X_TRACEINTID_EP0SETUP       0x000c
#define LPC214X_TRACEINTID_EPDMA          0x000d
#define LPC214X_TRACEINTID_EPFAST         0x000e
#define LPC214X_TRACEINTID_EPGETSTATUS    0x000f
#define LPC214X_TRACEINTID_EPIN           0x0010
#define LPC214X_TRACEINTID_EPOUT          0x0011
#define LPC214X_TRACEINTID_EPRINT         0x0012
#define LPC214X_TRACEINTID_EPSLOW         0x0013
#define LPC214X_TRACEINTID_FRAME          0x0014
#define LPC214X_TRACEINTID_IFGETSTATUS    0x0015
#define LPC214X_TRACEINTID_SETADDRESS     0x0017
#define LPC214X_TRACEINTID_SETFEATURE     0x0018
#define LPC214X_TRACEINTID_SUSPENDCHG     0x0019

/* Hardware interface **********************************************************/

/* Macros for testing the device status response */

#define DEVSTATUS_CONNECT(s)    (((s)&USBDEV_DEVSTATUS_CONNECT)!=0)
#define DEVSTATUS_CONNCHG(s)    (((s)&USBDEV_DEVSTATUS_CONNCHG)!=0)
#define DEVSTATUS_SUSPEND(s)    (((s)&USBDEV_DEVSTATUS_SUSPEND)!=0)
#define DEVSTATUS_SUSPCHG(s)    (((s)&USBDEV_DEVSTATUS_SUSPCHG)!=0)
#define DEVSTATUS_RESET(s)      (((s)&USBDEV_DEVSTATUS_RESET)!=0)

/* If this bit is set in the lpc214x_epread response, it means that the
 * recevied packet was overwritten by a later setup packet (ep0 only).
 */

#define LPC214X_READOVERRUN_BIT (0x80000000)
#define LPC214X_READOVERRUN(s)  (((s) & LPC214X_READOVERRUN_BIT) != 0)

/* USB RAM  ********************************************************************
 *
 * UBS_UDCA is is list of 32 pointers to DMA desciptors located at the
 * beginning of USB RAM.  Each pointer points to a DMA descriptor with
 * assocated DMA buffer.
 */

#define USB_UDCA           (uint32*)LPC214X_USBDEV_RAMBASE)
#define USB_USCASIZE       (LPC214X_NPHYSENDPOINTS*sizeof(uint32))

/* Each descriptor must be aligned to a 128 address boundary */

#define USB_DDALIGNDOWN(a) ((a)&~0x7f)
#define USB_DDALIGNUP(a)   USB_DDALIGNDOWN((a)+0x7f)

#define USB_DDSIZE USB_DDALIGNDOWN((LPC214X_USBDEV_RAMSIZE-USB_USCASIZE)/CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS)
#define USB_DDESC  ((struct lpc214x_dmadesc_s*)(LPC214X_USBDEV_RAMBASE+USB_USCASIZE))

#ifdef CONFIG_USBDEV_ISOCHRONOUS
#  define USB_DDESCSIZE (5*sizeof(uint32))
#else
#  define USB_DDESCSIZE (4*sizeof(uint32))
#endif

/* Endpoints ******************************************************************/

/* Number of endpoints */

#define LPC214X_NLOGENDPOINTS        (16)          /* ep0-15 */
#define LPC214X_NPHYSENDPOINTS       (32)          /* x2 for IN and OUT */

/* Mapping to more traditional endpoint numbers */

#define LPC214X_EP_LOG2PHYOUT(ep)    ((ep)&0x0f)<<1))
#define LPC214X_EP_LOG2PHYIN(ep)     (LPC214X_EP_OUT(ep)|0x01)
#define LPC214X_EP_LOG2PHY(ep)       ((((ep)&0x0f)<<1)|(((ep)&0x80)>>7))

/* Each endpoint has somewhat different characteristics */

#define LPC214X_EPALLSET             (0xffffffff)  /* All endpoints */
#define LPC214X_EPOUTSET             (0x55555555)  /* Even phy endpoint numbers are OUT EPs */
#define LPC214X_EPINSET              (0xaaaaaaaa)  /* Odd endpoint numbers are IN EPs */
#define LPC214X_EPCTRLSET            (0x00000003)  /* EP0 IN/OUT are control endpoints */
#define LPC214X_EPINTRSET            (0x0c30c30c)  /* Interrupt endpoints */
#define LPC214X_EPBULKSET            (0xf0c30c30)  /* Bulk endpoints */
#define LPC214X_EPISOCSET            (0x030c30c0)  /* Isochronous endpoints */

#define LPC214X_EP0MAXPACKET         (64)          /* EP0 max packet size */
#define LPC214X_BULKMAXPACKET        (64)          /* Bulk endpoint max packet */
#define LPC214X_INTRMAXPACKET        (64)          /* Interrupt endpoint max packet */

/* EP0 status */

#define LPC214X_EP0IDLE              (0)           /* Nothing in progress */
#define LPC214X_EP0STATUSIN          (1)           /* Status sent */
#define LPC214X_EP0STATUSOUT         (2)           /* Status received */
#define LPC214X_EP0SHORTWRITE        (3)           /* Short data sent with no request */
#define LPC214X_EP0SHORTWRSENT       (4)           /* Short data write complete */
#define LPC214X_EP0SETADDRESS        (5)           /* Set address received */

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/* A container for a request so that the request make be retained in a list */

struct lpc214x_req_s
{
  struct usbdev_req_s    req;           /* Standard USB request */
  sq_entry_t             sqe;           /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct lpc214x_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct lpc214x_ep_s.
   */

  struct usbdev_ep_s      ep;            /* Standard endpoint structure */

  /* LPC214X-specific fields */

  struct lpc214x_usbdev_s *dev;          /* Reference to private driver data */
  sq_queue_t              reqlist;       /* Request list for this endpoint */
  ubyte                   eplog;         /* Logical EP address from descriptor */
  ubyte                   epphy;         /* Physical EP address */
  ubyte                   stalled:1;     /* Endpoint is halted */
  ubyte                   in:1;          /* Endpoint is IN only */
  ubyte                   halted:1;      /* Endport feature halted */
};

/* This represents a DMA descriptor */

#ifdef CONFIG_LPC214X_USBDEV_DMA
struct lpc214x_dmadesc_s
{
  uint32                  nextdesc;      /* Address of the next DMA descripto in RAM */
  uint32                  config;        /* Misc. bit encoded configuration information */
  uint32                  start;         /* DMA start address */
  uint32                  status;        /* Misc. bit encoded status inforamation */
#ifdef CONFIG_USBDEV_ISOCHRONOUS
  uint32                  size;          /* Isochronous packet size address */
#endif
  ubyte                   buffer[USB_DDSIZE-USB_DDESCSIZE];
};
#endif

/* This structure retains the state of the USB device controller */

struct lpc214x_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to structlpc214x_usbdev_s.
   */

  struct usbdev_s         usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* LPC214X-specific fields */

  ubyte                   devstatus;     /* Last response to device status command */
  ubyte                   ep0state;      /* State of certain EP0 operations */
  ubyte                   stalled:1;     /* 1: Protocol stalled */
  ubyte                   selfpowered:1; /* 1: Device is self powered */
  ubyte                   paddrset:1;    /* 1: Peripheral addr has been set */
  ubyte                   attached:1;    /* 1: Host attached */
  ubyte                   rxpending:1;   /* 1: RX pending */
  uint32                  epavail;       /* Available endpoints */
  uint32                  softprio;      /* Bitset of high priority interrupts */
  uint32                  wravail;       /* Bitset of available endpoints */
#ifdef CONFIG_LPC214X_USBDEV_FRAME_INTERRUPT
  uint32                  sof;           /* Last start-of-frame */
#endif

  /* Allocated DMA descriptor */

#ifdef CONFIG_LPC214X_USBDEV_DMA
  struct lpc214x_dmadesc_s *dmadesc;
#endif

  /* The endpoint list */

  struct lpc214x_ep_s     eplist[LPC214X_NPHYSENDPOINTS];
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations */

#if defined(CONFIG_LPC214X_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32 lpc214x_getreg(uint32 addr);
static void lpc214x_putreg(uint32 val, uint32 addr);
#else
# define lpc214x_getreg(addr)     getreg32(addr)
# define lpc214x_putreg(val,addr) putreg32(val,addr)
#endif

/* Command operations */

static uint32 lpc214x_usbcmd(uint16 cmd, ubyte data);

/* Low level data transfers and request operations */

static void lpc214x_epwrite(ubyte epphy, const ubyte *data, uint32 nbytes);
static int  lpc214x_epread(ubyte epphy, ubyte *data, uint32 nbytes);
static void lpc214x_reqcomplete(struct lpc214x_ep_s *privep, sint16 result);
static int  lpc214x_wrrequest(struct lpc214x_ep_s *privep);
static int  lpc214x_rdrequest(struct lpc214x_ep_s *privep);
static void lpc214x_cancelrequests(struct lpc214x_ep_s *privep);

/* Interrupt handling */

static void lpc214x_eprealize(struct lpc214x_ep_s *privep, boolean prio, uint32 packetsize);
static ubyte lpc214x_epclrinterrupt(ubyte epphy);
static inline void lpc214x_ep0configure(struct lpc214x_usbdev_s *priv);
#ifdef CONFIG_LPC214X_USBDEV_DMA
static inline void lpc214x_dmareset(uint32 enable);
#endif
static void lpc214x_usbreset(struct lpc214x_usbdev_s *priv);
static void lpc214x_dispatchrequest(struct lpc214x_usbdev_s *priv,
              const struct usb_ctrlreq_s *ctrl);
static inline void lpc214x_ep0setup(struct lpc214x_usbdev_s *priv);
static inline void lpc214x_ep0dataoutinterrupt(struct lpc214x_usbdev_s *priv);
static inline void lpc214x_ep0dataininterrupt(struct lpc214x_usbdev_s *priv);
static int lpc214x_usbinterrupt(int irq, FAR void *context);

#ifdef CONFIG_LPC214X_USBDEV_DMA
static int  lpc214x_dmasetup(struct lpc214x_usbdev_s *priv, ubyte epphy,
              uint32 epmaxsize, uint32 nbytes, uint32 *isocpacket,
              boolean isochronous);
static void lpc214x_dmarestart(ubyte epphy, uint32 descndx);
static void lpc214x_dmadisable(ubyte epphy);
#endif /* CONFIG_LPC214X_USBDEV_DMA */

/* Endpoint operations */

static int  lpc214x_epconfigure(FAR struct usbdev_ep_s *ep,
              const struct usb_epdesc_s *desc);
static int  lpc214x_epdisable(FAR struct usbdev_ep_s *ep);
static FAR struct usbdev_req_s *lpc214x_epallocreq(FAR struct usbdev_ep_s *ep);
static void lpc214x_epfreereq(FAR struct usbdev_ep_s *ep,
              FAR struct usbdev_req_s *);
#ifdef CONFIG_LPC214X_USBDEV_DMA
static FAR void *lpc214x_epallocbuffer(FAR struct usbdev_ep_s *ep,
              uint16 nbytes);
static void lpc214x_epfreebuffer(FAR struct usbdev_ep_s *ep, void *buf);
#endif
static int  lpc214x_epsubmit(FAR struct usbdev_ep_s *ep,
              struct usbdev_req_s *req);
static int  lpc214x_epcancel(FAR struct usbdev_ep_s *ep,
              struct usbdev_req_s *req);
static int  lpc214x_epstall(FAR struct usbdev_ep_s *ep, boolean resume);

/* USB device controller operations */

static FAR struct usbdev_ep_s *lcp214x_allocep(FAR struct usbdev_s *dev,
              ubyte epno, boolean in, ubyte eptype);
static void lpc214x_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep);
static int  lpc214x_getframe(struct usbdev_s *dev);
static int  lpc214x_wakeup(struct usbdev_s *dev);
static int  lpc214x_selfpowered(struct usbdev_s *dev, boolean selfpowered);
static int  lpc214x_pullup(struct usbdev_s *dev, boolean enable);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct lpc214x_usbdev_s g_usbdev;

static const struct usbdev_epops_s g_epops =
{
  .configure   = lpc214x_epconfigure,
  .disable     = lpc214x_epdisable,
  .allocreq    = lpc214x_epallocreq,
  .freereq     = lpc214x_epfreereq,
#ifdef CONFIG_LPC214X_USBDEV_DMA
  .allocbuffer = lpc214x_epallocbuffer,
  .freebuffer  = lpc214x_epfreebuffer,
#endif
  .submit      = lpc214x_epsubmit,
  .cancel      = lpc214x_epcancel,
  .stall       = lpc214x_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = lcp214x_allocep,
  .freeep      = lpc214x_freeep,
  .getframe    = lpc214x_getframe,
  .wakeup      = lpc214x_wakeup,
  .selfpowered = lpc214x_selfpowered,
  .pullup      = lpc214x_pullup,
};

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc214x_getreg
 *
 * Description:
 *   Get the contents of an LPC214x register
 *
 *******************************************************************************/

#if defined(CONFIG_LPC214X_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32 lpc214x_getreg(uint32 addr)
{
  uint32 val = getreg32(addr);
  lldbg("%04x->%04x\n", addr, val);
  return val;
}
#endif

/*******************************************************************************
 * Name: lpc214x_putreg
 *
 * Description:
 *   Set the contents of an LPC214x register to a value
 *
 *******************************************************************************/

#if defined(CONFIG_LPC214X_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void lpc214x_putreg(uint32 val, uint32 addr)
{
  lldbg("%04x<-%04x\n", addr, val);
  putreg32(val, addr);
}
#endif

/*******************************************************************************
 * Name: lpc214x_usbcmd
 *
 * Description:
 *   Transmit commands to the USB engine
 *
 *******************************************************************************/

static uint32 lpc214x_usbcmd(uint16 cmd, ubyte data)
{
  irqstate_t flags;
  uint32 tmp = 0;

  /* Disable interrupt and clear CDFULL and CCEMPTY interrupt status */

  flags = irqsave();
  lpc214x_putreg(USBDEV_DEVINT_CDFULL|USBDEV_DEVINT_CCEMTY, LPC214X_USBDEV_DEVINTCLR);

  /* Load commonad in USB engine */

  lpc214x_putreg(((cmd & 0xff) << 16) + CMD_USB_CMDWR, LPC214X_USBDEV_CMDCODE);

  /* Wait until command is accepted */

  while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CCEMTY) == 0);

  /* Clear command register empty (CCEMPTY) interrupt */

  lpc214x_putreg(USBDEV_DEVINT_CCEMTY, LPC214X_USBDEV_DEVINTCLR);

  /* determinate next phase of the command */

  switch (cmd)
    {
    case CMD_USB_DEV_SETADDRESS:
    case CMD_USB_DEV_CONFIG:
    case CMD_USB_DEV_SETMODE:
    case CMD_USB_DEV_SETSTATUS:
      lpc214x_putreg((data << 16) + CMD_USB_DATAWR, LPC214X_USBDEV_CMDCODE);
      while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CCEMTY) == 0);
      break;

    case CMD_USB_DEV_READFRAMENO:
    case CMD_USB_DEV_READTESTREG:
      lpc214x_putreg((cmd << 16) + CMD_USB_DATARD, LPC214X_USBDEV_CMDCODE);
      while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CDFULL) == 0);
      lpc214x_putreg(USBDEV_DEVINT_CDFULL, LPC214X_USBDEV_DEVINTCLR);
      tmp = lpc214x_getreg(LPC214X_USBDEV_CMDDATA);
      lpc214x_putreg((cmd << 16) + CMD_USB_DATARD, LPC214X_USBDEV_CMDCODE);
      while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CDFULL) == 0);
      tmp |= lpc214x_getreg(LPC214X_USBDEV_CMDDATA) << 8;
      break;

    case CMD_USB_DEV_GETSTATUS:
    case CMD_USB_DEV_GETERRORCODE:
    case CMD_USB_DEV_READERRORSTATUS:
    case CMD_USB_EP_CLRBUFFER:
      lpc214x_putreg((cmd << 16) + CMD_USB_DATARD, LPC214X_USBDEV_CMDCODE);
      while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CDFULL) == 0);
      tmp = lpc214x_getreg(LPC214X_USBDEV_CMDDATA);
      break;

    default:
      switch (cmd & 0x1e0)
        {
        case CMD_USB_EP_SELECT:
        case CMD_USB_EP_SELECTCLEAR:
          lpc214x_putreg((cmd << 16) + CMD_USB_DATARD, LPC214X_USBDEV_CMDCODE);
          while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CDFULL) == 0);
          tmp = lpc214x_getreg(LPC214X_USBDEV_CMDDATA);
          break;

        case CMD_USB_EP_SETSTATUS:
          lpc214x_putreg((data << 16) + CMD_USB_DATAWR, LPC214X_USBDEV_CMDCODE);
          while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CCEMTY) == 0);
          break;
        }
      break;
    }

  /* Restore the interrupt flags */

  irqrestore(flags);
  return tmp;
}

/*******************************************************************************
 * Name: lpc214x_epwrite
 *
 * Description:
 *   Endpoint write (IN)
 *
 *******************************************************************************/

static void lpc214x_epwrite(ubyte epphy, const ubyte *data, uint32 nbytes)
{
  /* Set the write enable bit for this physical EP address */

  lpc214x_putreg(((epphy << 1) & LPC214X_USBCTRL_EPMASK) | LPC214X_USBCTRL_WREN,
                 LPC214X_USBDEV_CTRL);

  /* Set the transmit packet length (nbytes must be less than 2048) */

  putreg32(nbytes, LPC214X_USBDEV_TXPLEN);

  /* Transfer the packet data */

  do
    {
      /* Zero length packets are a special case */

      if (nbytes)
        {
          lpc214x_putreg(*data++, LPC214X_USBDEV_TXDATA);
        }
      else
        {
          /* Zero length packet */

          lpc214x_putreg(0, LPC214X_USBDEV_TXDATA);
        }
    }
  while ((lpc214x_getreg(LPC214X_USBDEV_CTRL) & LPC214X_USBCTRL_WREN) != 0);

  /* Done */

  lpc214x_putreg(0, LPC214X_USBDEV_CTRL);
  (void)lpc214x_usbcmd(CMD_USB_EP_SELECT | epphy, 0);
  (void)lpc214x_usbcmd(CMD_USB_EP_VALIDATEBUFFER, 0);
}

/*******************************************************************************
 * Name: lpc214x_epread
 *
 * Description:
 *   Endpoint read (OUT)
 *
 *******************************************************************************/

static int lpc214x_epread(ubyte epphy, ubyte *data, uint32 nbytes)
{
  uint32 pktlen;
  uint32 result;
  uint32 value;

  /* Set the read enable bit for this physical EP address */

  lpc214x_putreg(((epphy << 1) & LPC214X_USBCTRL_EPMASK) | LPC214X_USBCTRL_RDEN,
                 LPC214X_USBDEV_CTRL);

  /* Wait for packet buffer ready for reading */

  while ((lpc214x_getreg(LPC214X_USBDEV_RXPLEN) & USBDEV_RXPLEN_PKTRDY) == 0);

  /* Get the about of data to be read */

  pktlen = lpc214x_getreg(LPC214X_USBDEV_RXPLEN) & USBDEV_RXPLEN_PKTLENGTH;

  /* Read data from input buffer while read data is valid (DV) */

  while ((lpc214x_getreg(LPC214X_USBDEV_RXPLEN) & USBDEV_RXPLEN_DV) != 0)
    {
      value = lpc214x_getreg(LPC214X_USBDEV_RXDATA);
      if (data)
        {
          *data++ = value;
        }
    }

  /* Done */

  lpc214x_putreg(0, LPC214X_USBDEV_CTRL);
  (void)lpc214x_usbcmd(CMD_USB_EP_SELECT | epphy, 0);
  result = lpc214x_usbcmd(CMD_USB_EP_CLRBUFFER, 0);

  /* The packet overrun bit in the clear buffer response is applicable only
   * on EP0 transfers.  If set it means that the recevied packet was overwritten
   * by a later setup packet.
   */

  if (epphy == LPC214X_EP0_OUT && (result & CMD_USB_CLRBUFFER_PO) != 0)
    {
      /* Pass this information in bit 31 */

      pktlen |= LPC214X_READOVERRUN_BIT;
    }
  return pktlen;
}

/*******************************************************************************
 * Name: lpc214x_reqcomplete
 *
 * Description:
 *   Handle termination of a request.
 *
 *******************************************************************************/

static void lpc214x_reqcomplete(struct lpc214x_ep_s *privep, sint16 result)
{
  struct lpc214x_req_s *privreq;
  int stalled = privep->stalled;
  irqstate_t flags;

  /* Remove the complete request at the head of the endpoint request list */

  flags = irqsave();
  privreq = (struct lpc214x_req_s *)sq_remfirst(&privep->reqlist);
  irqrestore(flags);

  if (privreq)
    {
      /* If endpoint 0, temporarily reflect the state of protocol stalled
       * in the callback.
       */

      if (privep->epphy == 0)
        {
          if (privep->dev->stalled)
            {
              privep->stalled = 1;
            }
        }

      /* Save the result in the request structure */

      privreq->req.result = result;

      /* Callback to the request completion handler */

      privreq->sqe.flink = NULL;
      privreq->req.callback(&privep->ep, &privreq->req);

      /* Restore the stalled indication */

      privep->stalled = stalled;
    }
}

/*******************************************************************************
 * Name: lpc214x_wrrequest
 *
 * Description:
 *   Send from the next queued write request
 *
 * Returned Value:
 *  0:not finished; 1:completed; <0:error
 *
 *******************************************************************************/

static int lpc214x_wrrequest(struct lpc214x_ep_s *privep)
{
  struct lpc214x_req_s *privreq;
  ubyte *buf;
  int nbytes;
  int bytesleft;

  /* Check the request from the head of the endpoint request queue */

  privreq = (struct lpc214x_req_s *)sq_peek(&privep->reqlist);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_NULLREQUEST), 0);
      return OK;
    }

  bytesleft = privreq->req.len;
  for (;;)
    {
      /* Send the next packet if (1) there are more bytes to be sent, or
       * (2) the last packet was exactly maxpacketsize.
       */

      usbtrace(TRACE_WRITE(privep->epphy), privreq->req.xfrd);
      if (privreq->req.len >= privreq->req.xfrd)
        {
          /* Try to send maxpacketsize -- unless we don't have that many
           * bytes to send.
           */

          nbytes = privep->ep.maxpacket;
          if (nbytes > bytesleft)
            {
              nbytes = bytesleft;
            }

          /* Send the largest number of bytes that we can in this packet */

          buf = privreq->req.buf + privreq->req.xfrd;
          lpc214x_epwrite(privep->epphy, buf, nbytes);

          /* Update for the next time through the loop */

          privreq->req.xfrd += nbytes;
          bytesleft          = privreq->req.len - privreq->req.xfrd;
        }

      /* If all of the bytes were sent (and the last packet was not full)
       * then we are finished with the transfer)
       */

      if (privreq->req.len < privreq->req.xfrd)
        {
          usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
          lpc214x_reqcomplete(privep, OK);
          return OK;
        }
    }

  return OK; /* Won't get here */
}

/*******************************************************************************
 * Name: lpc214x_rdrequest
 *
 * Description:
 *   Receive to the next queued read request
 *
 *******************************************************************************/

static int lpc214x_rdrequest(struct lpc214x_ep_s *privep)
{
  struct lpc214x_req_s *privreq;
  ubyte *buf;
  int nbytesread;

  /* Check the request from the head of the endpoint request queue */

  privreq = (struct lpc214x_req_s *)sq_peek(&privep->reqlist);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_NULLREQUEST), 0);
      return OK;
    }

  usbtrace(TRACE_READ(privep->epphy), privreq->req.xfrd);
  for (;;)
    {
      /* Receive the next packet if (1) there are more bytes to be receive, or
       * (2) the last packet was exactly maxpacketsize.
       */

      buf        = privreq->req.buf + privreq->req.xfrd;
      nbytesread = lpc214x_epread(privep->epphy, buf, privep->ep.maxpacket);
      if (nbytesread < 0)
        {
          usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_EPREAD), nbytesread);
          return ERROR;
        }

      /* If the receive buffer is full or if the last packet was not full
       * then we are finished with the transfer.
       */

      privreq->req.xfrd += nbytesread;
      if (privreq->req.len < privreq->req.xfrd || nbytesread < privep->ep.maxpacket)
        {
          usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
          lpc214x_reqcomplete(privep, OK);
          return OK;
        }
    }

  return OK; /* Won't get here */
}

/*******************************************************************************
 * Name: lpc214x_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 *******************************************************************************/

static void lpc214x_cancelrequests(struct lpc214x_ep_s *privep)
{
  while (!sq_empty(&privep->reqlist))
    {
      usbtrace(TRACE_COMPLETE(privep->epphy),
               ((struct lpc214x_req_s *)sq_peek(&privep->reqlist))->req.xfrd);
      lpc214x_reqcomplete(privep, -ESHUTDOWN);
    }
}

/*******************************************************************************
 * Name: lpc214x_eprealize
 *
 * Description:
 *   Enable or disable an endpoint
 *
 *******************************************************************************/

static void lpc214x_eprealize(struct lpc214x_ep_s *privep, boolean prio, uint32 packetsize)
{
  struct lpc214x_usbdev_s *priv = privep->dev;
  uint32 mask;
  uint32 reg;

  /* Initialize endpoint software priority */

  mask = 1 << privep->epphy;
  if (prio)
    {
      priv->softprio = priv->softprio | mask;
    }
  else
    {
      priv->softprio = priv->softprio & ~mask;
    }

  /* Clear realize interrupt bit */

  lpc214x_putreg(USBDEV_DEVINT_EPRLZED, LPC214X_USBDEV_DEVINTCLR);

  /* Realize the endpoint */

  reg  = lpc214x_getreg(LPC214X_USBDEV_REEP);
  reg |= (1 << privep->epphy);
  lpc214x_putreg(reg, LPC214X_USBDEV_REEP);

  /* Set endpoint maximum packet size */

  lpc214x_putreg(privep->epphy, LPC214X_USBDEV_EPIND);
  lpc214x_putreg(packetsize, LPC214X_USBDEV_MAXPSIZE);

  /* Wait for Realize complete */

  while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_EPRLZED) == 0);

  /* Clear realize interrupt bit */

  lpc214x_putreg(USBDEV_DEVINT_EPRLZED,LPC214X_USBDEV_DEVINTCLR);
}

/*******************************************************************************
 * Name: lpc214x_epclrinterrupt
 *
 * Description:
 *   Clear the EP interrupt flag and return the current EP status
 *
 *******************************************************************************/

static ubyte lpc214x_epclrinterrupt(ubyte epphy)
{
  /* Clear the endpoint interrupt */

  lpc214x_putreg(1 << epphy, LPC214X_USBDEV_EPINTCLR);

  /* Wait for data in the command data register */

  while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CDFULL) == 0);

  /* Return the value of the command data register */

  return lpc214x_getreg(LPC214X_USBDEV_CMDDATA);
}

/*******************************************************************************
 * Name: lpc214x_ep0configure
 *
 * Description:
 *   Configure endpoint 0
 *
 *******************************************************************************/

static inline void lpc214x_ep0configure(struct lpc214x_usbdev_s *priv)
{
  uint32 inten;

  /* EndPoint 0 initialization */

  lpc214x_eprealize(&priv->eplist[LPC214X_CTRLEP_OUT], 0, CONFIG_USBDEV_EP0_MAXSIZE);
  lpc214x_eprealize(&priv->eplist[LPC214X_CTRLEP_IN], 1, CONFIG_USBDEV_EP0_MAXSIZE);

  /* Enable EP0 interrupts (not DMA) */

  inten = lpc214x_getreg(LPC214X_USBDEV_EPINTEN);
  inten |= 3; /* EP0 Rx and Tx */
  lpc214x_putreg(inten, LPC214X_USBDEV_EPINTEN);
}

/*******************************************************************************
 * Name: lpc214x_dmareset
 *
 * Description: Reset USB DMA
 *
 *******************************************************************************/

#ifdef CONFIG_LPC214X_USBDEV_DMA
static inline void lpc214x_dmareset(uint32 enable)
{
  int i;

  /* Disable All DMA interrupts */

  lpc214x_putreg(0, LPC214X_USBDEV_DMAINTEN);

  /* DMA Disable */

  lpc214x_putreg(0xffffffff, LPC214X_USBDEV_EPDMADIS);

  /* DMA Request clear */

  putreq32(0xffffffff, LPC214X_USBDEV_DMARCLR);

  /* End of Transfer Interrupt Clear */

  putreq32(0xffffffff, LPC214X_USBDEV_EOTINTCLR);

  /* New DD Request Interrupt Clear */

  putreq32(0xffffffff, LPC214X_USBDEV_NDDRINTCLR);

  /* System Error Interrupt Clear */

  putreq32(0xffffffff, LPC214X_USBDEV_SYSERRINTCLR);

  /* Nullify all pointers in the UDCA */

  for (i = 0; i < LPC214X_NPHYSENDPOINTS; ++i)
    {
      USB_UDCA[i] = NULL;
    }

  /* Set USB UDCA Head register */

  lpc214x_putreg((uint32)USB_UDCA, LPC214X_USBDEV_UDCAH);

  /* Invalidate all DMA descriptors */

  for (i = 0; i < CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS; ++i)
    {
      memset(&USB_DDESC[i], 0, USB_DDESCSIZE);
    }

  /* Enable DMA interrupts */

  lpc214x_putreg(enable, LPC214X_USBDEV_DMAINTEN);
}
#endif

/*******************************************************************************
 * Name: lpc214x_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 *******************************************************************************/

static void lpc214x_usbreset(struct lpc214x_usbdev_s *priv)
{
  /* Disable all endpoint interrupts */

  lpc214x_putreg(0, LPC214X_USBDEV_EPINTEN);

  /* Frame is Hp interrupt */

  lpc214x_putreg(1, LPC214X_USBDEV_DEVINTPRI);

  /* Clear all pending interrupts */

  lpc214x_putreg(0xffffffff, LPC214X_USBDEV_EPINTCLR);
  lpc214x_putreg(0xffffffff, LPC214X_USBDEV_DEVINTCLR);

  /* Periperhal address is needed */

  priv->paddrset = 0;

  /* Endpoints not yet configured */

  lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 0);

  /* EndPoint 0 initialization */

  lpc214x_ep0configure(priv);

#ifdef CONFIG_LPC214X_USBDEV_DMA
  /* Enable End_of_Transfer_Interrupt and System_Error_Interrupt USB DMA
   * interrupts
   */

  lpc214x_dmareset(CONFIG_LPC214X_USBDEV_DMAINTMASK);

#endif

  /* Enable Device interrupts */

  lpc214x_putreg(USB_SLOW_INT|USB_DEVSTATUS_INT|USB_FAST_INT|USB_FRAME_INT|USB_ERROR_INT,
                 LPC214X_USBDEV_DEVINTEN);
}

/*******************************************************************************
 * Name: lpc214x_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically part
 *   of the USB interrupt handler.
 *
 *******************************************************************************/

static void lpc214x_dispatchrequest(struct lpc214x_usbdev_s *priv,
                                    const struct usb_ctrlreq_s *ctrl)
{
  int ret;

  usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl);
      if (ret < 0)
        {
          /* Stall on failure */

          priv->stalled = 1;
        }
      else
        {
          lpc214x_epwrite(LPC214X_EP0_IN, NULL, 0);
          priv->ep0state = LPC214X_EP0SHORTWRITE;
        }
    }
}

/*******************************************************************************
 * Name: lpc214x_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event. This is logically part of the USB interrupt
 *   handler.
 *
 *******************************************************************************/

static inline void lpc214x_ep0setup(struct lpc214x_usbdev_s *priv)
{
  struct lpc214x_ep_s *ep0 = &priv->eplist[LPC214X_EP0_OUT];
  struct lpc214x_req_s *privreq = (struct lpc214x_req_s *)sq_peek(&ep0->reqlist);
  struct usb_ctrlreq_s ctrl;
  uint16 index;
  ubyte  epphy;
  ubyte  response[2];
  int    ret;

  /* Starting a control request? */

  if (priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      priv->usbdev.speed = USB_SPEED_FULL;
      lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 1);
    }

  /* Terminate any pending requests */

  while (!sq_empty(&ep0->reqlist))
    {
      sint16 result = OK;
      if (privreq->req.xfrd != privreq->req.len)
        {
          result = -EPROTO;
        }

      usbtrace(TRACE_COMPLETE(ep0->epphy), privreq->req.xfrd);
      lpc214x_reqcomplete(ep0, result);
    }

  /* Assume NOT stalled */

  ep0->stalled  = 0;
  priv->stalled = 0;

  /* Read EP0 data */

  ret = lpc214x_epread(LPC214X_EP0_OUT, (ubyte*)&ctrl, sizeof(struct usb_ctrlreq_s));
  if (ret <= 0)
    {
      return;
    }

  /* Dispatch any non-standard requests */

  ep0->in = (ctrl.type & USB_DIR_IN) != 0;
  if ((ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      lpc214x_dispatchrequest(priv, &ctrl);
      return;
    }

  /* Handle standard request.  Pick off the things of interest to the
   * USB device controller driver; pass what is left to the class driver
   */

  switch (ctrl.req)
    {
    case USB_REQ_GETSTATUS:
      {
        /* Length must be 2, from device, and value == 0 */

        if (!priv->paddrset || GETUINT16(ctrl.len) != 2 ||
            (ctrl.type & USB_REQ_DIR_IN) == 0 || GETUINT16(ctrl.value) != 0)
          {
            priv->stalled = 1;
          }
        else
          {
            switch (ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
              case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPGETSTATUS), 0);
                  epphy = USB_EPNO(GETUINT16(ctrl.index)) << 1;
                  if (epphy < LPC214X_NPHYSENDPOINTS)
                    {
                       if ((lpc214x_usbcmd(CMD_USB_EP_SELECT|epphy, 0) & CMD_USB_EPSELECT_ST) != 0)
                         {
                           response[0] = 1; /* Stalled */
                         }
                       else
                         {
                           response[0] = 0; /* Not stalled */
                         }
                      response[1] = 0;
                      lpc214x_epwrite(LPC214X_EP0_IN, response, 2);
                      priv->ep0state = LPC214X_EP0SHORTWRITE;
                    }
                  else
                    {
                      priv->stalled = 1;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
                {
                  index = GETUINT16(ctrl.index);
                  if (index == 0)
                    {
                      usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_DEVGETSTATUS), 0);

                      /* Features:  Remote Wakeup=YES; selfpowered=? */

                      response[0] = (priv->selfpowered << USB_FEATURE_SELFPOWERED) |
                                    (1 << USB_FEATURE_REMOTEWAKEUP);
                      response[1] = 0;
                      lpc214x_epwrite(LPC214X_EP0_IN, response, 2);
                      priv->ep0state = LPC214X_EP0SHORTWRITE;
                    }
                  else
                    {
                      priv->stalled = 1;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_IFGETSTATUS), 0);
                  response[0] = 0;
                  response[1] = 0;
                  lpc214x_epwrite(LPC214X_EP0_IN, response, 2);
                  priv->ep0state = LPC214X_EP0SHORTWRITE;
                }
                break;

              default:
                {
                  priv->stalled = 1;
                }
                break;
              }
          }
      }
      break;

    case USB_REQ_CLEARFEATURE:
      {
        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_CLEARFEATURE), 0);
        if (ctrl.type != USB_REQ_RECIPIENT_ENDPOINT)
          {
            lpc214x_dispatchrequest(priv, &ctrl);
          }
        else if (priv->paddrset && GETUINT16(ctrl.value) == USB_ENDPOINT_HALT &&
                 GETUINT16(ctrl.index) < LPC214X_NLOGENDPOINTS && GETUINT16(ctrl.len) == 0)
          {
            ubyte epphys = LPC214X_EP_LOG2PHY(GETUINT16(ctrl.index));
            priv->eplist[epphys].halted = 0;
            lpc214x_epwrite(LPC214X_EP0_IN, NULL, 0);
            priv->ep0state = LPC214X_EP0STATUSIN;
          }
        else
          {
              priv->stalled = 1;
          }
      }
      break;

    case USB_REQ_SETFEATURE:
      {
        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_SETFEATURE), 0);
        if (ctrl.type == USB_REQ_RECIPIENT_DEVICE &&
            GETUINT16(ctrl.value) == USB_FEATURE_TESTMODE)
          {
            uvdbg("test mode: %d\n", GETUINT16(ctrl.index));
          }
        else if (ctrl.type != USB_REQ_RECIPIENT_ENDPOINT)
          {
           lpc214x_dispatchrequest(priv, &ctrl);
          }
        else if (priv->paddrset && GETUINT16(ctrl.value) == USB_ENDPOINT_HALT &&
                 GETUINT16(ctrl.index) < LPC214X_NLOGENDPOINTS && GETUINT16(ctrl.len) == 0)
          {
            ubyte epphys = LPC214X_EP_LOG2PHY(GETUINT16(ctrl.index));
            priv->eplist[epphys].halted = 1;
            lpc214x_epwrite(LPC214X_EP0_IN, NULL, 0);
            priv->ep0state = LPC214X_EP0STATUSIN;
          }
        else
          {
             priv->stalled = 1;
          }
      }
      break;

    case USB_REQ_SETADDRESS:
      {
        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_SETADDRESS), 0);
        if ((ctrl.type == USB_REQ_RECIPIENT_DEVICE) &&
            (GETUINT16(ctrl.index)  == 0) && (GETUINT16(ctrl.len) == 0) &&
            (GETUINT16(ctrl.value) < 128))
          {
            lpc214x_epwrite(LPC214X_EP0_IN, NULL, 0);
            priv->eplist[LPC214X_EP0_OUT].eplog = ctrl.value[0];
            priv->eplist[LPC214X_EP0_IN].eplog  = ctrl.value[0];
            priv->ep0state = LPC214X_EP0SETADDRESS;
          }
        else
          {
            priv->stalled = 1;
          }
      }
      break;

    case USB_REQ_GETDESCRIPTOR:
    case USB_REQ_SETDESCRIPTOR:
    case USB_REQ_GETCONFIGURATION:
      {
        if (priv->paddrset && (GETUINT16(ctrl.value) == 0) &&
            (GETUINT16(ctrl.index) == 0) && (GETUINT16(ctrl.len) == 1))
          {
            lpc214x_dispatchrequest(priv, &ctrl);
          }
        else
          {
            priv->stalled = 1;
          }
      }
      break;


    case USB_REQ_SETCONFIGURATION:
    case USB_REQ_GETINTERFACE:
    case USB_REQ_SETINTERFACE:
      {
       lpc214x_dispatchrequest(priv, &ctrl);
      }
      break;

    case USB_REQ_SYNCHFRAME:
      break;

    default:
      priv->stalled = 1;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_STALLED), priv->ep0state);
      lpc214x_epstall(&ep0->ep, FALSE);
      lpc214x_epstall(&ep0->ep, FALSE);
    }
}

/*******************************************************************************
 * Name: lpc214x_ep0dataoutinterrupt
 *
 * Description:
 *   USB Ctrl EP Data OUT Event. This is logically part of the USB interrupt
 *   handler.
 *
 *******************************************************************************/

static inline void lpc214x_ep0dataoutinterrupt(struct lpc214x_usbdev_s *priv)
{
  struct lpc214x_ep_s *ep0;
  uint32 pktlen;

  /* Copy new setup packet int setup buffer */

  switch (priv->ep0state)
    {
    case LPC214X_EP0SHORTWRITE:
      {
        priv->ep0state = LPC214X_EP0STATUSOUT;
        pktlen = lpc214x_epread(LPC214X_EP0_OUT, NULL, CONFIG_USBDEV_EP0_MAXSIZE);
        if (LPC214X_READOVERRUN(pktlen))
          {
            lpc214x_ep0setup(priv);
          }
      }
      break;

    case LPC214X_EP0SHORTWRSENT:
      {
        priv->ep0state = LPC214X_EP0IDLE;
        pktlen = lpc214x_epread(LPC214X_EP0_OUT, NULL, CONFIG_USBDEV_EP0_MAXSIZE);
        if (LPC214X_READOVERRUN(pktlen))
          {
            lpc214x_ep0setup(priv);
          }
      }
      break;

    case LPC214X_EP0IDLE:
      {
        ep0 = &priv->eplist[LPC214X_EP0_OUT];
        if (!sq_empty(&ep0->reqlist))
          {
            lpc214x_wrrequest(ep0);
          }
      }
      break;

    default:
      priv->stalled = 1;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_STALLED), priv->ep0state);
      ep0 = &priv->eplist[LPC214X_EP0_OUT];
      lpc214x_epstall(&ep0->ep, FALSE);
      lpc214x_epstall(&ep0->ep, FALSE);
    }
  return;
}

/*******************************************************************************
 * Name: lpc214x_ep0dataininterrupt
 *
 * Description:
 *   USB Ctrl EP Data IN Event. This is logically part of the USB interrupt
 *   handler.
 *
 *******************************************************************************/

static inline void lpc214x_ep0dataininterrupt(struct lpc214x_usbdev_s *priv)
{
  struct lpc214x_ep_s *ep0;

  switch (priv->ep0state)
    {
    case LPC214X_EP0STATUSOUT:
    case LPC214X_EP0STATUSIN:
      priv->ep0state = LPC214X_EP0IDLE;
      break;

    case LPC214X_EP0SHORTWRITE:
      priv->ep0state = LPC214X_EP0SHORTWRSENT;
      break;

    case LPC214X_EP0SETADDRESS:
      {
        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_SETADDRESS), 0);

        /* Set EP0 logical address */

        ubyte eplog = priv->eplist[LPC214X_EP0_OUT].eplog;
        lpc214x_usbcmd(CMD_USB_DEV_SETADDRESS, CMD_USB_SETADDRESS_DEVEN | eplog);
        lpc214x_usbcmd(CMD_USB_DEV_SETADDRESS, CMD_USB_SETADDRESS_DEVEN | eplog);

        /* Not yet fully configured */

        lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 0);
        if (eplog)
          {
            priv->paddrset = 1;
            priv->ep0state = LPC214X_EP0IDLE;
          }
      }
      break;

    default:
      priv->stalled = 1;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_STALLED), priv->ep0state);
      ep0 = &priv->eplist[LPC214X_EP0_IN];
      lpc214x_epstall(&ep0->ep, FALSE);
      lpc214x_epstall(&ep0->ep, FALSE);
    }
}

/*******************************************************************************
 * Name: lpc214x_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 *******************************************************************************/

static int lpc214x_usbinterrupt(int irq, FAR void *context)
{
  struct lpc214x_usbdev_s *priv = &g_usbdev;
  struct lpc214x_ep_s *privep ;

  uint32 devintstatus;  /* Sampled state of the device interrupt status register */
  uint32 epintstatus;   /* Sampled state of the endpoint interrupt status register */
#ifdef CONFIG_LPC214X_USBDEV_DMA
  uint32 dmaintstatus;  /* Sampled state of dma interrupt status register */
#endif
  uint32 softprio;      /* Current priority interrupt bitset */
  uint32 pending;       /* Pending subset of priority interrupt bitset */
  ubyte  epphy;         /* Physical endpoint number being processed */
  int    i;

  usbtrace(TRACE_INTENTRY(LPC214X_TRACEINTID_USB), 0);

  /* Read the device interrupt status register */

  devintstatus = lpc214x_getreg(LPC214X_USBDEV_DEVINTST);

#ifdef CONFIG_LPC214X_USBDEV_DMA
  /* Check for low priority and high priority (non-DMA) interrupts */

  if ((lpc214x_getreg(LPC214X_USBDEV_INTST) & (USBDEV_INTST_REQLP|USBDEV_INTST_REQHP)) != 0)
    {
#endif
#ifdef CONFIG_LPC214X_USBDEV_EPFAST_INTERRUPT
      /* Fast EP interrupt */

      if ((devintstatus & USBDEV_DEVINT_EPFAST) != 0)
        {
          /* Clear Fast EP interrupt */

         lpc214x_putreg(USBDEV_DEVINT_EPFAST, LPC214X_USBDEV_DEVINTCLR);
         usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPFAST), 0);

         /* Do what? */
        }

#endif

#if CONFIG_DEBUG
      /* USB engine error interrupt */

      if ((devintstatus & USBDEV_DEVINT_EPRINT))
        {
          ubyte errcode;

          /* Clear the error interrupt */

          lpc214x_putreg(USBDEV_DEVINT_EPRINT, LPC214X_USBDEV_DEVINTCLR);

          /* And show what error occurred */

          errcode  = (ubyte)lpc214x_usbcmd(CMD_USB_DEV_READERRORSTATUS, 0) & 0x0f;
          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPRINT), (uint16)errcode);
        }
#endif

#ifdef CONFIG_LPC214X_USBDEV_FRAME_INTERRUPT
      /* Frame interrupt */

      if ((devintstatus & USBDEV_DEVINT_FRAME) != 0)
        {
          /* Clear the frame interrupt */

          lpc214x_putreg(USBDEV_DEVINT_FRAME, LPC214X_USBDEV_DEVINTCLR);
          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_FRAME), 0);

          /* Then read the start of frame value */

          priv->sof = (uint16)lpc214x_usbcmd(CMD_USB_DEV_READFRAMENO, 0);
        }
#endif

      /* Device Status interrupt */

      if ((devintstatus & USBDEV_DEVINT_DEVSTAT) != 0)
        {
          /* Clear Device status interrupt */

          lpc214x_putreg(USBDEV_DEVINT_DEVSTAT, LPC214X_USBDEV_DEVINTCLR);

          /* Get device status */

          g_usbdev.devstatus = lpc214x_usbcmd(CMD_USB_DEV_GETSTATUS, 0);
          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_DEVSTAT), (uint16)g_usbdev.devstatus);

          /* Device connection status */

          if (DEVSTATUS_CONNCHG(g_usbdev.devstatus))
            {
              usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_CONNECTCHG), 0);
              if (DEVSTATUS_CONNECT(g_usbdev.devstatus))
                {
                   /* Host is connected */

                   if (!priv->attached)
                     {
                       /* We have a transition from unattached to attached */

                       usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_CONNECTED), 0);
                       priv->usbdev.speed = USB_SPEED_UNKNOWN;
                       lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 0);
                       priv->attached     = 1;
                    }
                 }

               /* Otherwise the host is not attached */

               else if (priv->attached)
                 {
                   usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_DISCONNECTED), 0);
                   priv->usbdev.speed = USB_SPEED_UNKNOWN;
                   lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 0);
                   priv->attached = 0;
                   priv->paddrset = 0;
                 }
            }

          /* Device suspend status */

          if (DEVSTATUS_SUSPCHG(g_usbdev.devstatus))
            {
              usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_SUSPENDCHG), 0);
            }

          /* Device reset */

          if (DEVSTATUS_RESET(g_usbdev.devstatus))
            {
              usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_DEVRESET), 0);
              lpc214x_usbreset(priv);
            }
        }

      /* Slow EP interrupt */

      if ((devintstatus & USBDEV_DEVINT_EPSLOW) != 0)
        {
          /* Clear Slow EP interrupt */

          lpc214x_putreg(USBDEV_DEVINT_EPSLOW, LPC214X_USBDEV_DEVINTCLR);
          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPSLOW), 0);

          do
            {
              /* Read the endpoint interrupt status register */

              epintstatus = lpc214x_getreg(LPC214X_USBDEV_EPINTST);

              /* Loop twice:  Process software high priority interrupts
               * on the first pass and low priority interrupts on the
               * second.
               */

              softprio = priv->softprio;
              for (i = 0; i < 2; i++, softprio = ~softprio)
                {
                  /* On the first time through the loop, pending will be
                   * the bitset of high priority pending interrupts; on the
                   * second time throught it will be the bitset of low
                   * priority interrupts.
                   */

                  pending = epintstatus & softprio;

                  /* EP0 OUT interrupt indicated by bit0 == 1 */

                  if ((pending & 1) != 0)
                    {
                      /* Clear the endpoint interrupt */

                      uint32 result = lpc214x_epclrinterrupt(LPC214X_CTRLEP_OUT);
                      if (result & USBDEV_EPSETUPPACKET)
                        {
                          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EP0SETUP), (uint16)result);
                          lpc214x_ep0setup(priv);
                        }
                      else
                        {
                          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EP0OUT), priv->ep0state);
                          lpc214x_ep0dataoutinterrupt(priv);
                        }
                      break;
                    }

                  /* EP0 IN interrupt indicated by bit1 == 1 */

                  if ((pending & 2) != 0)
                    {
                      /* Clear the endpoint interrupt */

                      usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EP0IN), priv->ep0state);
                      (void)lpc214x_epclrinterrupt(LPC214X_CTRLEP_IN);
                      lpc214x_ep0dataininterrupt(priv);
                    }
                  pending >>= 2;

                  /* All other endpoints EP 1-31 */

                  for (epphy = 2; pending; epphy++, pending >>= 1)
                    {
                      /* Is the endpoint interrupt pending? */

                      if ((pending & 1) != 0)
                        {
                           /* Yes.. clear the endpoint interrupt */

                          (void)lpc214x_epclrinterrupt(epphy);

                          /* Get the endpoint sructure corresponding to the physical
                           * endpoint number.
                           */

                          privep =  &priv->eplist[epphy];

                          /* Check for complete on IN or OUT endpoint.  Odd physical
                           * endpoint addresses are IN endpoints.
                           */

                          if ((epphy & 1) != 0)
                            {
                              /* IN: device-to-host */

                              usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPOUT), (uint16)epphy);
                              if (priv->usbdev.speed == USB_SPEED_UNKNOWN)
                                {
                                  priv->usbdev.speed = USB_SPEED_FULL;
                                  lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 1);
                                }

                              /* Write host data from the current write request */

                              if (!sq_empty(&privep->reqlist))
                                {
                                  lpc214x_wrrequest(privep);
                                }
                           }
                          else
                            {
                              /* OUT: host-to-device */

                              usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPIN), (uint16)epphy);

                              /* Read host data into the current read request */

                             if (!sq_empty(&privep->reqlist))
                                 {
                                  lpc214x_rdrequest(privep);
                                }
                              else
                                {
                                  uvdbg("Pending interrupt\n");
                                  priv->rxpending = 1;
                                }
                            }
                        }
                    }
                }
            }
          while (epintstatus);
        }
#ifdef CONFIG_LPC214X_USBDEV_DMA
    }

  /* Check for DMA interrupts */

  if ((lpc214x_getreg(LPC214X_USBDEV_INTST) & USBDEV_INTST_REQDMA) != 0)
    {
      /* First Software High priority and then low priority */

      uint32 tmp;

      /* Collect the DMA interrupt sources */

      dmaintstatus = 0;
      tmp = lpc214x_getreg(LPC214X_USBDEV_EOTINTST);
      if (lpc214x_getreg(LPC214X_USBDEV_DMAINTEN) & 1)
        {
          dmaintstatus |= tmp;
        }
      lpc214x_putreg(tmp, LPC214X_USBDEV_EOTINTCLR);

      tmp = lpc214x_getreg(LPC214X_USBDEV_NDDRINTST);
      if (lpc214x_getreg(LPC214X_USBDEV_DMAINTEN) & 2)
        {
          dmaintstatus |= tmp;
        }
      lpc214x_putreg(tmp, LPC214X_USBDEV_NDDRINTCLR);

      tmp = lpc214x_getreg(LPC214X_USBDEV_SYSERRINTST);
      if (lpc214x_getreg(LPC214X_USBDEV_DMAINTEN) & 4)
        {
          dmaintstatus |= tmp;
        }
      lpc214x_putreg(tmp, LPC214X_USBDEV_SYSERRINTCLR);

      /* Loop twice:  Process software high priority interrupts on the
       * first pass and low priority interrupts on the second.
       */

      softprio = priv->softprio;
      for (i = 0; i < 2; i++, softprio = ~softprio)
        {
          /* On the first time through the loop, pending will be
           * the bitset of high priority pending interrupts; on the
           * second time throught it will be the bitset of low
           * priority interrupts. Note that EP0 IN and OUT are
           * omitted.
           */

          pending = (dmaintstatus & softprio) >> 2;
          for (epphy = 2; pending; epphy++, pending >>= 1)
            {
              if ((pending & 1) != 0)
                {
                  usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPDMA), (uint16)epphy);
#warning DO WHAT?
                }
            }
        }
    }
#endif
  usbtrace(TRACE_INTEXIT(LPC214X_TRACEINTID_USB), 0);
  return OK;
}

/*******************************************************************************
 * Name: lpc214x_dmasetup
 *
 * Description:
 *   Setup for DMA Transfer
 *
 *******************************************************************************/

#ifdef CONFIG_LPC214X_USBDEV_DMA
static int lpc214x_dmasetup(struct lpc214x_usbdev_s *priv, ubyte epphy,
                            uint32 epmaxsize, uint32 nbytes, uint32 *isocpacket,
                            boolean isochronous);
{
  struct lpc214x_dmadesc_s *dmadesc = priv;
  uint32 reg;

#ifdef CONFIG_DEBUG
  if (!priv || epphy < 2)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Check if a DMA descriptor has been assigned.  If not, than that indicates
   * that we will have to do parallel I/O
   */

  if (!dmadesc)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_NODMADESC), 0);
      return -EBUSY;
    }

  /* Verify that the DMA descriptor is available */

  if ((dmadesc->status & USB_DMADESC_STATUSMASK) == USB_DMADESC_BEINGSERVICED)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_DMABUSY), 0);
      return -EBUSY; /* Shouldn't happen */
    }

  /* Init DMA Descriptor */

  dmadesc->nexdesc = 0;
  dmadesc->config  = USB_DMADESC_MODENORMAL |
                     ((epmaxsize << USB_DMADESC_PKTSIZESHIFT) & USB_DMADESC_PKTSIZEMASK) |
                     ((nbytes << USB_DMADESC_BULENSHIFT) & USB_DMADESC_BUFLENMASK);

#ifdef CONFIG_USBDEV_ISOCHRONOUS
  if (isochronous)
    {
      dmadesc->config |= USB_DMADESC_ISCOEP;
    }
#endif

  dmadesc->start = (uint32)&dmadesc->buffer;
  dmadesc->status = 0;

#ifdef CONFIG_USBDEV_ISOCHRONOUS
  dmadesc->size = (uint32)packet;
#endif

  /* Enable DMA tranfer for this endpoint */

  putreq32(1 << epphy, LPC214X_USBDEV_EPDMAEN);

  /* Check state of IN/OUT Ep buffer */

  reg = lpc214x_usbcmd(CMD_USB_EP_SELECT | epphy, 0);

  if (((epphy & 1) != 0 &&  (reg & 0x60) == 0) ||
      ((epphy & 1) == 0 &&  (reg & 0x60) == 0x60))
    {
      /* DMA should be "being serviced" */

      if ((dmadesc->status & USB_DMADESC_STATUSMASK) != USB_DMADESC_BEINGSERVICED))
        {
          /* Re-trigger the DMA Transfer */

          putreq21(1 << epphy, LPC214X_USBDEV_DMARCLR);
          putreq32(1 << epphy, LPC214X_USBDEV_EPDMAEN);
        }
    }
  return OK;
}
#endif /* CONFIG_LPC214X_USBDEV_DMA */

/*******************************************************************************
 * Name: lpc214x_dmarestart
 *
 * Description:
 *   Restart DMA Transfer
 *
 *******************************************************************************/

#ifdef CONFIG_LPC214X_USBDEV_DMA
static void lpc214x_dmarestart(ubyte epphy, uint32 descndx)
{
  uint32 reg;

  /* Clear DMA descriptor status */

  USB_DmaDesc[descndx].status = 0;

  /* Enable DMA transfer on the endpoint */

  lpc214x_putreg(1 << epph, LPC214X_USBDEV_EPDMAEN);

  /* Check the state of IN/OUT EP buffer */

  uint32 reg = lpc214x_usbcmd(CMD_USB_EP_SELECT | epphy, 0);
  if (((epphy & 1) != 0 &&  (reg & 0x60) == 0) ||
      ((epphy & 1) == 0 &&  (reg & 0x60) == 0x60))
    {
      /* Re-trigger the DMA Transfer */

      putreq21(1 << epphy, LPC214X_USBDEV_DMARCLR);
      putreq32(1 << epphy, LPC214X_USBDEV_EPDMAEN);
    }
}
#endif /* CONFIG_LPC214X_USBDEV_DMA */

/*******************************************************************************
 * Name: lpc214x_dmadisable
 *
 * Description:
 *   Disable DMA transfer for the EP
 *
 *******************************************************************************/

#ifdef CONFIG_LPC214X_USBDEV_DMA
static void lpc214x_dmadisable(ubyte epphy)
{
  EPDMADIS = 1 << epphy;
}
#endif /* CONFIG_LPC214X_USBDEV_DMA */

/*******************************************************************************
 * Endpoint operations
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc214x_epconfigure
 *
 * Description:
 *   Configure endpoint, making it usable
 *
 *******************************************************************************/

static int lpc214x_epconfigure(FAR struct usbdev_ep_s *ep,
                               FAR const struct usb_epdesc_s *desc)
{
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  uint32 inten;
  int eplog;
  int epphy;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);

  eplog = desc->addr;
  epphy = LPC214X_EP_LOG2PHY(eplog);

  /* Realize the endpoint */

  lpc214x_eprealize(privep, 1, GETUINT16(desc->mxpacketsize));

  /* Enable and reset EP -- twice */

  lpc214x_usbcmd(CMD_USB_EP_SETSTATUS | epphy, 0);
  lpc214x_usbcmd(CMD_USB_EP_SETSTATUS | epphy, 0);

#ifdef CONFIG_LPC214X_USBDEV_DMA
  /* Enable DMA Ep interrupt (WO) */

   lpc214x_putreg(1 << epphy, LPC214X_USBDEV_EPDMAEN);
#else
  /* Enable Ep interrupt (R/W) */

   inten = lpc214x_getreg(LPC214X_USBDEV_EPINTEN);
   inten |= (1 << epphy);
   lpc214x_putreg(inten, LPC214X_USBDEV_EPINTEN);
#endif
   return OK;
}

/*******************************************************************************
 * Name: lpc214x_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 *******************************************************************************/

static int lpc214x_epdisable(FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  irqstate_t flags;
  uint32 mask = (1 << privep->epphy);
  uint32 reg;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPDISABLE, privep->epphy);

  /* Cancel any ongoing activity */

  flags = irqsave();
  lpc214x_cancelrequests(privep);

  /* Disable endpoint and interrupt */

  reg  = lpc214x_getreg(LPC214X_USBDEV_REEP);
  reg &= ~mask;
  lpc214x_putreg(reg, LPC214X_USBDEV_REEP);

  lpc214x_putreg(mask, LPC214X_USBDEV_EPDMADIS);

  reg  = lpc214x_getreg(LPC214X_USBDEV_EPINTEN);
  reg &= ~mask;
  lpc214x_putreg(reg, LPC214X_USBDEV_EPINTEN);

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc214x_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 *******************************************************************************/

static FAR struct usbdev_req_s *lpc214x_epallocreq(FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc214x_req_s *privreq;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif
  usbtrace(TRACE_EPALLOCREQ, ((FAR struct lpc214x_ep_s *)ep)->epphy);

  privreq = (FAR struct lpc214x_req_s *)malloc(sizeof(struct lpc214x_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct lpc214x_req_s));
  return &privreq->req;
}

/*******************************************************************************
 * Name: lpc214x_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 *******************************************************************************/

static void lpc214x_epfreereq(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc214x_req_s *privreq = (FAR struct lpc214x_req_s *)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif
  usbtrace(TRACE_EPFREEREQ, ((FAR struct lpc214x_ep_s *)ep)->epphy);

  free(privreq);
}

/*******************************************************************************
 * Name: lpc214x_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_LPC214X_USBDEV_DMA
static FAR void *lpc214x_epallocbuffer(FAR struct usbdev_ep_s *ep, uint16 nbytes)
{
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  int descndx;

  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);

  /* Find a free  DMA description */

#error "LOGIC INCOMPLETE"

  /* Set UDCA to the allocated DMA descriptor for this endpoint */

  USB_UDCA[privep->epphy] = &USB_DDESC[descndx];
  return  &USB_DDESC[descndx]
}
#endif

/*******************************************************************************
 * Name: lpc214x_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_LPC214X_USBDEV_DMA
static void lpc214x_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf)
{
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;

  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);

  /* Indicate that there is no DMA descriptor associated with this endpoint  */

  USB_UDCA[privep->epphy] = NULL;

  /* Mark the DMA descriptor as free for re-allocation */

#error "LOGIC INCOMPLETE"
}
#endif

/*******************************************************************************
 * Name: lpc214x_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 *******************************************************************************/

static int lpc214x_epsubmit(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc214x_req_s *privreq = (FAR struct lpc214x_req_s *)req;
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  FAR struct lpc214x_usbdev_s *priv;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

  if (!priv->driver || priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_NOTCONFIGURED), 0);
      return -ESHUTDOWN;
    }

  req->result = -EINPROGRESS;
  req->xfrd = 0;

  /* Check for NULL packet */

  flags = irqsave();
  if (req->len == 0 && (privep->in || privep->epphy == 3))
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_NULLPACKET), 0);
      sq_addlast(&privreq->sqe, &privep->reqlist);
      goto success_notransfer;
    }

  /* Has everything been sent? Or are we stalled? */

  if (!sq_empty(&privep->reqlist) && !privep->stalled)
    {
      /* Handle zero-length transfers on EP0 */

      if (privep->epphy == 0 && req->len == 0)
        {
          /* Nothing to transfer -- exit success, zero-bytes transferred */

          usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
          lpc214x_reqcomplete(privep, OK);
          goto success_notransfer;
        }

      /* Handle IN requests */

      if ((privep->in) || privep->epphy == 3)
        {
          ret = lpc214x_wrrequest(privep);
        }

      /* Handle pending OUT requests */

      else if (priv->rxpending)
        {
          ret = lpc214x_rdrequest(privep);
          priv->rxpending = 0;
        }
      else
        {
          usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADREQUEST), 0);
          ret = ERROR;
          goto errout;
        }
    }

  /* Add to endpoint's request queue */

  if (ret >= 0)
    {
      usbtrace(TRACE_REQQUEUED(privep->epphy), privreq->req.len);
      sq_addlast(&privreq->sqe, &privep->reqlist);
    }

success_notransfer:
errout:
  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: lpc214x_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 *******************************************************************************/

static int lpc214x_epcancel(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  FAR struct lpc214x_usbdev_s *priv;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPCANCEL, privep->epphy);
  priv = privep->dev;

  flags = irqsave();
  lpc214x_cancelrequests(privep);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc214x_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 *******************************************************************************/

static int lpc214x_epstall(FAR struct usbdev_ep_s *ep, boolean resume)
{
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, privep->epphy);
  lpc214x_usbcmd(CMD_USB_EP_SETSTATUS | privep->epphy, (resume ? 0 : USBDEV_EPSTALL));
  return OK;
}

/*******************************************************************************
 * Device operations
 *******************************************************************************/

/*******************************************************************************
 * Name: lcp214x_allocep
 *
 * Description:
 *   Allocate an endpoint matching the parameters.
 *
 * Input Parameters:
 *   epphy  - 7-bit physical endpoint number (without direction bit).  Zero means
 *            that any endpoint matching the other requirements will suffice.
 *   in     - TRUE: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.  One of {USB_EP_ATTR_XFER_ISOC, USB_EP_ATTR_XFER_BULK,
 *            USB_EP_ATTR_XFER_INT}
 *
 *******************************************************************************/

static FAR struct usbdev_ep_s *lcp214x_allocep(FAR struct usbdev_s *dev, ubyte epphy,
                                               boolean in, ubyte eptype)
{
  FAR struct lpc214x_usbdev_s *priv = (FAR struct lpc214x_usbdev_s *)dev;
  uint32 epset = LPC214X_EPALLSET;
  irqstate_t flags;
  int epndx = 0;

  usbtrace(TRACE_DEVALLOCEP, 0);

  /* epphy=0 means that any endpoint will do */

  if (epphy > 0)
    {
      if (epphy >= LPC214X_NLOGENDPOINTS)
        {
         usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADEPNO), 0);
         return NULL;
        }

      epset &= 3 << epphy;
    }

  /* Get the subset matching the requested direction */

  if (in)
    {
      epset &= LPC214X_EPINSET;
    }
  else
    {
      epset &= LPC214X_EPOUTSET;
    }

  /* Get the subset matching the requested type */

  switch (eptype)
    {
    case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      epset &= LPC214X_EPINTRSET;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      epset &= LPC214X_EPBULKSET;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
      epset &= LPC214X_EPBULKSET;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint -- not a valid choice */
    default:
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADEPTYPE), 0);
      return NULL;
    }

  /* Is the resulting endpoint supported by the LPC214x? */

  if (epset)
    {
      /* Yes.. now see if any of the request endpoints are available */

      flags = irqsave();
      epset &= priv->epavail;
      if (epset)
        {
          /* Select the lowest bit in the set of matching, available endpoints */

          for (epndx = 2; epndx < LPC214X_NPHYSENDPOINTS; epndx++)
            {
              uint32 bit = 1 << epndx;
              if ((epset & bit) == 0)
                {
                  /* Mark the endpoint no longer available */

                  priv->wravail &= ~bit;
                  irqrestore(flags);

                  /* And return the pointer to the standard endpoint structure */

                  return &priv->eplist[epndx].ep;
                }
            }
          /* Shouldn't get here */
        }
      irqrestore(flags);
    }

  usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_NOEP), 0);
  return NULL;
}

/*******************************************************************************
 * Name: lpc214x_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 *******************************************************************************/

static void lpc214x_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc214x_usbdev_s *priv = (FAR struct lpc214x_usbdev_s *)dev;
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  irqstate_t flags;

  usbtrace(TRACE_DEVFREEEP, (uint16)privep->epphy);

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      flags = irqsave();
      priv->wravail &= ~(1 << privep->epphy);
      irqrestore(flags);
    }
}

/*******************************************************************************
 * Name: lpc214x_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 *******************************************************************************/

static int lpc214x_getframe(struct usbdev_s *dev)
{
#ifdef CONFIG_LPC214X_USBDEV_FRAME_INTERRUPT
  FAR struct lpc214x_usbdev_s *priv = (FAR struct lpc214x_usbdev_s *)dev;

  /* Return last valid value of SOF read by the interrupt handler */

  usbtrace(TRACE_DEVGETFRAME, (uint16)priv->sof);
  return priv->sof;
#else
  /* Return the last frame number detected by the hardware */

  usbtrace(TRACE_DEVGETFRAME, 0);
  return (int)lpc214x_usbcmd(CMD_USB_DEV_READFRAMENO, 0);
#endif
}

/*******************************************************************************
 * Name: lpc214x_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 *******************************************************************************/

static int lpc214x_wakeup(struct usbdev_s *dev)
{
  ubyte arg = USBDEV_DEVSTATUS_SUSPEND;
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, (uint16)g_usbdev.devstatus);

  flags = irqsave();
  if (DEVSTATUS_CONNECT(g_usbdev.devstatus))
    {
      arg |= USBDEV_DEVSTATUS_CONNECT;
    }

  lpc214x_usbcmd(CMD_USB_DEV_SETSTATUS, arg);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc214x_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature 
 *
 *******************************************************************************/

static int lpc214x_selfpowered(struct usbdev_s *dev, boolean selfpowered)
{
  FAR struct lpc214x_usbdev_s *priv = (FAR struct lpc214x_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16)selfpowered);

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/*******************************************************************************
 * Name: lpc214x_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 *******************************************************************************/

static int lpc214x_pullup(struct usbdev_s *dev, boolean enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16)enable);

  /* Prohibit interruption during connection command */

  irqstate_t flags = irqsave();

  /* The USBDEV_DEVSTATUS_CONNECT bit in the CMD_USB_DEV_SETSTATUS command
   * controls the LPC214x SoftConnect_N output pin that is used for SoftConnect.
   */

  lpc214x_usbcmd(CMD_USB_DEV_SETSTATUS, (enable ? USBDEV_DEVSTATUS_CONNECT : 0));
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: up_usbinitialize
 *
 * Description:
 *   Initialize USB hardware.
 *
 * Assumptions:
 * - This function is called very early in the initialization sequence
 * - PLL initialization is not performed here but should been in the low-level
 *   boot logic.  For the USB engine, FUSB=FOSC*PLL_M.  The USB device controller
 *   clock is a 48MHz clock.
 *
 *******************************************************************************/

void up_usbinitialize(void)
{
  struct lpc214x_usbdev_s *priv = &g_usbdev;
  uint32 reg;

  usbtrace(TRACE_DEVINIT, 0);

  /* Initialize the device state structure */

  memset(priv, 0, sizeof(struct lpc214x_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->wravail    = LPC214X_EPALLSET;

  /* Turn on USB power and clocking */

  reg = lpc214x_getreg(LPC214X_PCON_PCONP);
  reg |= LPC214X_PCONP_PCUSB;
  lpc214x_putreg(reg, LPC214X_PCON_PCONP);

  /* Enable Vbus sense and Connect */

  reg = lpc214x_getreg(LPC214X_PINSEL1);
  reg = (reg & LPC214X_USBDEV_PINMASK) | LPC214X_USBDEV_PINSEL;
  lpc214x_putreg(reg, LPC214X_PINSEL1);

  /* Attach USB controller  interrupt handler */

  if (irq_attach(LPC214X_USB_IRQ, lpc214x_usbinterrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_IRQREGISTRATION), 0);
      goto errout;
    }

  /* Enable USB inerrupts */

  up_enable_irq(LPC214X_USB_IRQ);

  /* Disconnect device */

  lpc214x_pullup(&priv->usbdev, FALSE);

  /* Enable EP0 for OUT (host-to-device) */

  lpc214x_usbcmd(CMD_USB_DEV_SETADDRESS, CMD_USB_SETADDRESS_DEVEN|LPC214X_EP0_OUT);
  lpc214x_usbcmd(CMD_USB_DEV_SETADDRESS, CMD_USB_SETADDRESS_DEVEN|LPC214X_EP0_OUT);

  /* Reset/Re-initialize the USB hardware */

  lpc214x_usbreset(priv);

  /* Init Device state structure */

  priv->devstatus = lpc214x_usbcmd(CMD_USB_DEV_GETSTATUS, 0);
  return;

errout:
  up_usbuninitialize();
}

/*******************************************************************************
 * Name: up_usbuninitialize
 *******************************************************************************/

void up_usbuninitialize(void)
{
  struct lpc214x_usbdev_s *priv = &g_usbdev;
  uint32 reg;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNINIT, 0);
  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Disconnect device */

  flags = irqsave();
  lpc214x_pullup(&priv->usbdev, FALSE);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;
  lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 0);

  /* Disable and detach IRQs */

  up_disable_irq(LPC214X_USB_IRQ);
  irq_detach(LPC214X_USB_IRQ);

  /* Turn off USB power and clocking */

  reg = lpc214x_getreg(LPC214X_PCON_PCONP);
  reg &= ~LPC214X_PCONP_PCUSB;
  lpc214x_putreg(reg, LPC214X_PCON_PCONP);
  irqrestore(flags);
}

/*******************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method will be
 *   called to bind it to a USB device driver.
 *
 *******************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  g_usbdev.driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BINDFAILED), (uint16)-ret);
      g_usbdev.driver = NULL;
    }
  return ret;
}

/*******************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected to a USB host,
 *   it will first disconnect().  The driver is also requested to unbind() and clean
 *   up any device state, before this procedure finally returns.
 *
 *******************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (driver != g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_usbdev.usbdev);

  /* Unhook the driver */

  g_usbdev.driver = NULL;
  return OK;
}
