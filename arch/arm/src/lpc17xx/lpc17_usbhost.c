/*******************************************************************************
 * arch/arm/src/lpc17xx/lpc17_usbhost.c
 *
 *   Copyright (C) 2010-2012, 2014-2015 Gregory Nutt. All rights reserved.
 *   Authors: Rafael Noronha <rafael@pdsolucoes.com.br>
 *            Gregory Nutt <gnutt@nuttx.org>
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/ohci.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>

#include <arch/irq.h>

#include <arch/board/board.h> /* May redefine GPIO settings */

#include "up_arch.h"
#include "up_internal.h"

#include "chip.h"
#include "chip/lpc17_usb.h"
#include "chip/lpc17_syscon.h"
#include "lpc17_gpio.h"
#include "lpc17_ohciram.h"

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/

/* Configuration ***************************************************************/

/* All I/O buffers must lie in AHB SRAM because of the OHCI DMA. It might be
 * okay if no I/O buffers are used *IF* the application can guarantee that all
 * end-user I/O buffers reside in AHB SRAM.
 */

#if LPC17_IOBUFFERS < 1
#  warning "No IO buffers allocated"
#endif

#ifndef CONFIG_LPC17_USBHOST_NPREALLOC
#  define CONFIG_LPC17_USBHOST_NPREALLOC 8
#endif

/* OHCI Setup ******************************************************************/
/* Frame Interval / Periodic Start */

#define BITS_PER_FRAME          12000
#define FI                     (BITS_PER_FRAME-1)
#define FSMPS                  ((6 * (FI - 210)) / 7)
#define DEFAULT_FMINTERVAL     ((FSMPS << OHCI_FMINT_FSMPS_SHIFT) | FI)
#define DEFAULT_PERSTART       (((9 * BITS_PER_FRAME) / 10) - 1)

/* CLKCTRL enable bits */

#define LPC17_CLKCTRL_ENABLES   (USBOTG_CLK_HOSTCLK|USBOTG_CLK_PORTSELCLK|USBOTG_CLK_AHBCLK)

/* Interrupt enable bits */

#ifdef CONFIG_DEBUG_USB
#  define LPC17_DEBUG_INTS      (OHCI_INT_SO|OHCI_INT_RD|OHCI_INT_UE|OHCI_INT_OC)
#else
#  define LPC17_DEBUG_INTS      0
#endif

#define LPC17_NORMAL_INTS       (OHCI_INT_WDH|OHCI_INT_RHSC)
#define LPC17_ALL_INTS          (LPC17_NORMAL_INTS|LPC17_DEBUG_INTS)

/* Dump GPIO registers */

#if defined(CONFIG_LPC17_USBHOST_REGDEBUG) && defined(CONFIG_DEBUG_GPIO)
#  define usbhost_dumpgpio() \
   do { \
     lpc17_dumpgpio(GPIO_USB_DP, "D+ P0.29; D- P0.30"); \
     lpc17_dumpgpio(GPIO_USB_UPLED, "LED P1:18; PPWR P1:19 PWRD P1:22 PVRCR P1:27"); \
   } while (0);
#else
#  define usbhost_dumpgpio()
#endif

/* USB Host Memory *************************************************************/

/* Helper definitions */

#define HCCA        ((struct ohci_hcca_s *)LPC17_HCCA_BASE)
#define TDTAIL      ((struct lpc17_gtd_s *)LPC17_TDTAIL_ADDR)
#define EDCTRL      ((struct lpc17_ed_s *)LPC17_EDCTRL_ADDR)

/* Periodic intervals 2, 4, 8, 16,and 32 supported */

#define MIN_PERINTERVAL 2
#define MAX_PERINTERVAL 32

/* Descriptors *****************************************************************/

/* TD delay interrupt value */

#define TD_DELAY(n) (uint32_t)((n) << GTD_STATUS_DI_SHIFT)

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/* This structure retains the state of the USB host controller */

struct lpc17_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to structlpc17_usbhost_s.
   */

  struct usbhost_driver_s drvr;

  /* This is the hub port description understood by class drivers */

  struct usbhost_roothubport_s rhport;

  /* Driver status */

  volatile bool    change;      /* Connection change */
  volatile bool    connected;   /* Connected to device */
  volatile bool    pscwait;     /* TRUE: Thread is waiting for a port status change */

#ifndef CONFIG_USBHOST_INT_DISABLE
  uint8_t          ininterval;  /* Minimum periodic IN EP polling interval: 2, 4, 6, 16, or 32 */
  uint8_t          outinterval; /* Minimum periodic IN EP polling interval: 2, 4, 6, 16, or 32 */
#endif

  sem_t            exclsem;     /* Support mutually exclusive access */
  sem_t            pscsem;      /* Semaphore to wait Writeback Done Head event */

#ifdef CONFIG_USBHOST_HUB
  /* Used to pass external hub port events */

  volatile struct usbhost_hubport_s *hport;
#endif
 };

/* This structure describes one asynchronous transfer */

struct lpc17_xfrinfo_s
{
  volatile bool wdhwait;      /* Thread is waiting for WDH interrupt */
  volatile uint8_t tdstatus;  /* TD control status bits from last Writeback Done Head event */
  uint8_t *buffer;            /* Transfer buffer start */
  uint16_t buflen;            /* Buffer length */
  uint16_t xfrd;              /* Number of bytes transferred */

#ifdef CONFIG_USBHOST_ASYNCH
#if LPC17_IOBUFFERS > 0
  /* Remember the allocated DMA buffer address so that it can be freed when
   * the transfer completes.
   */

  uint8_t *alloc;             /* Allocated buffer */
#endif

  /* Retain the callback information for the asynchronous transfer
   * completion.
   */

  usbhost_asynch_t callback;  /* Transfer complete callback */
  void *arg;                  /* Argument that accompanies the callback */
#endif
};

/* The OCHI expects the size of an endpoint descriptor to be 16 bytes.
 * However, the size allocated for an endpoint descriptor is 32 bytes in
 * lpc17_ohciram.h.  This extra 16-bytes is used by the OHCI host driver in
 * order to maintain additional endpoint-specific data.
 */

struct lpc17_ed_s
{
  /* Hardware specific fields */

  struct ohci_ed_s hw;        /* 0-15 */

  /* Software specific fields */

  uint8_t          xfrtype;   /* 16: Transfer type.  See SB_EP_ATTR_XFER_* in usb.h */
  uint8_t          interval;  /* 17: Periodic EP polling interval: 2, 4, 6, 16, or 32 */
  sem_t            wdhsem;    /* 18: Semaphore used to wait for Writeback Done Head event */
                              /*    Unused bytes may follow, depending on the size of sem_t */
  /* Pointer to structure that manages asynchronous transfers on this pipe */

  struct lpc17_xfrinfo_s *xfrinfo;
};

/* The OCHI expects the size of an transfer descriptor to be 16 bytes.
 * However, the size allocated for an endpoint descriptor is 32 bytes in
 * lpc17_ohciram.h.  This extra 16-bytes is used by the OHCI host driver in
 * order to maintain additional endpoint-specific data.
 */

struct lpc17_gtd_s
{
  /* Hardware specific fields */

  struct ohci_gtd_s hw;

  /* Software specific fields */

  struct lpc17_ed_s *ed;      /* Pointer to parent ED */
  uint8_t           pad[12];
};

/* The following is used to manage lists of free EDs, TDs, and TD buffers */

struct lpc17_list_s
{
  struct lpc17_list_s *flink; /* Link to next buffer in the list */
                              /* Variable length buffer data follows */
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations ********************************************************/

#ifdef CONFIG_LPC17_USBHOST_REGDEBUG
static void lpc17_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void lpc17_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t lpc17_getreg(uint32_t addr);
static void lpc17_putreg(uint32_t val, uint32_t addr);
#else
# define lpc17_getreg(addr)     getreg32(addr)
# define lpc17_putreg(val,addr) putreg32(val,addr)
#endif

/* Semaphores ******************************************************************/

static void lpc17_takesem(sem_t *sem);
#define lpc17_givesem(s) sem_post(s);

/* Byte stream access helper functions *****************************************/

static inline uint16_t lpc17_getle16(const uint8_t *val);
#if 0 /* Not used */
static void lpc17_putle16(uint8_t *dest, uint16_t val);
#endif

/* OHCI memory pool helper functions *******************************************/

static inline void lpc17_edfree(struct lpc17_ed_s *ed);
static  struct lpc17_gtd_s *lpc17_tdalloc(void);
static void lpc17_tdfree(struct lpc17_gtd_s *buffer);
static uint8_t *lpc17_tballoc(void);
static void lpc17_tbfree(uint8_t *buffer);
#if LPC17_IOBUFFERS > 0
static uint8_t *lpc17_allocio(void);
static void lpc17_freeio(uint8_t *buffer);
#endif
static struct lpc17_xfrinfo_s *lpc17_alloc_xfrinfo(void);
static void lpc17_free_xfrinfo(struct lpc17_xfrinfo_s *xfrinfo);

/* ED list helper functions ****************************************************/

static inline int lpc17_addctrled(struct lpc17_usbhost_s *priv,
                                  struct lpc17_ed_s *ed);
static inline int lpc17_remctrled(struct lpc17_usbhost_s *priv,
                                  struct lpc17_ed_s *ed);

static inline int lpc17_addbulked(struct lpc17_usbhost_s *priv,
                                  struct lpc17_ed_s *ed);
static inline int lpc17_rembulked(struct lpc17_usbhost_s *priv,
                                  struct lpc17_ed_s *ed);

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static unsigned int lpc17_getinterval(uint8_t interval);
static void lpc17_setinttab(uint32_t value, unsigned int interval, unsigned int offset);
#endif

static inline int lpc17_addinted(struct lpc17_usbhost_s *priv,
                                 const struct usbhost_epdesc_s *epdesc,
                                 struct lpc17_ed_s *ed);
static inline int lpc17_reminted(struct lpc17_usbhost_s *priv,
                                 struct lpc17_ed_s *ed);

static inline int lpc17_addisoced(struct lpc17_usbhost_s *priv,
                                  const struct usbhost_epdesc_s *epdesc,
                                  struct lpc17_ed_s *ed);
static inline int lpc17_remisoced(struct lpc17_usbhost_s *priv,
                                  struct lpc17_ed_s *ed);

/* Descriptor helper functions *************************************************/

static int lpc17_enqueuetd(struct lpc17_usbhost_s *priv,
                           struct lpc17_ed_s *ed, uint32_t dirpid,
                           uint32_t toggle, volatile uint8_t *buffer,
                           size_t buflen);
static int lpc17_ctrltd(struct lpc17_usbhost_s *priv, struct lpc17_ed_s *ed,
                        uint32_t dirpid, uint8_t *buffer, size_t buflen);

/* Interrupt handling **********************************************************/

static int lpc17_usbinterrupt(int irq, void *context);

/* USB host controller operations **********************************************/

static int lpc17_wait(struct usbhost_connection_s *conn,
                      struct usbhost_hubport_s **hport);
static int lpc17_rh_enumerate(struct usbhost_connection_s *conn,
                              struct usbhost_hubport_s *hport);
static int lpc17_enumerate(struct usbhost_connection_s *conn,
                           struct usbhost_hubport_s *hport);

static int lpc17_ep0configure(struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep0, uint8_t funcaddr, uint8_t speed,
                              uint16_t maxpacketsize);
static int lpc17_epalloc(struct usbhost_driver_s *drvr,
                         const struct usbhost_epdesc_s *epdesc, usbhost_ep_t *ep);
static int lpc17_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int lpc17_alloc(struct usbhost_driver_s *drvr,
                       uint8_t **buffer, size_t *maxlen);
static int lpc17_free(struct usbhost_driver_s *drvr, uint8_t *buffer);
static int lpc17_ioalloc(struct usbhost_driver_s *drvr,
                         uint8_t **buffer, size_t buflen);
static int lpc17_iofree(struct usbhost_driver_s *drvr, uint8_t *buffer);
static int lpc17_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        const struct usb_ctrlreq_s *req,
                        uint8_t *buffer);
static int lpc17_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                         const struct usb_ctrlreq_s *req,
                         const uint8_t *buffer);
static int lpc17_transfer_common(struct lpc17_usbhost_s *priv,
                                 struct lpc17_ed_s *ed, uint8_t *buffer,
                                 size_t buflen);
#if LPC17_IOBUFFERS > 0
static int lpc17_dma_alloc(struct lpc17_usbhost_s *priv,
                           struct lpc17_ed_s *ed, uint8_t *userbuffer,
                           size_t buflen, uint8_t **alloc);
static void lpc17_dma_free(struct lpc17_usbhost_s *priv,
                           struct lpc17_ed_s *ed, uint8_t *userbuffer,
                           size_t buflen, uint8_t *alloc);
#endif
static ssize_t lpc17_transfer(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                              uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void lpc17_asynch_completion(struct lpc17_usbhost_s *priv,
                                    struct lpc17_ed_s *ed);
static int lpc17_asynch(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                        FAR uint8_t *buffer, size_t buflen,
                        usbhost_asynch_t callback, FAR void *arg);
#endif
static int lpc17_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int lpc17_connect(FAR struct usbhost_driver_s *drvr,
                         FAR struct usbhost_hubport_s *hport,
                         bool connected);
#endif
static void lpc17_disconnect(struct usbhost_driver_s *drvr,
                             struct usbhost_hubport_s *hport);

/* Initialization **************************************************************/

static inline void lpc17_ep0init(struct lpc17_usbhost_s *priv);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* In this driver implementation, support is provided for only a single a single
 * USB device.  All status information can be simply retained in a single global
 * instance.
 */

static struct lpc17_usbhost_s g_usbhost;

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_usbconn =
{
  .wait             = lpc17_wait,
  .enumerate        = lpc17_enumerate,
};

/* This is a free list of EDs and TD buffers */

static struct lpc17_list_s *g_edfree; /* List of unused EDs */
static struct lpc17_list_s *g_tdfree; /* List of unused TDs */
static struct lpc17_list_s *g_tbfree; /* List of unused transfer buffers */
#if LPC17_IOBUFFERS > 0
static struct lpc17_list_s *g_iofree; /* List of unused I/O buffers */
#endif

/* Pool and freelist of transfer structures */

static struct lpc17_list_s *g_xfrfree;
static struct lpc17_xfrinfo_s g_xfrbuffers[CONFIG_LPC17_USBHOST_NPREALLOC];

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc17_printreg
 *
 * Description:
 *   Print the contents of an LPC17xx register operation
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBHOST_REGDEBUG
static void lpc17_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  lldbg("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/*******************************************************************************
 * Name: lpc17_checkreg
 *
 * Description:
 *   Get the contents of an LPC17xx register
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBHOST_REGDEBUG
static void lpc17_checkreg(uint32_t addr, uint32_t val, bool iswrite)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register last time?
   * Are we polling the register?  If so, suppress the output.
   */

  if (addr == prevaddr && val == preval && prevwrite == iswrite)
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

              lpc17_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              lldbg("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = addr;
      preval    = val;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new regisgter access */

      lpc17_printreg(addr, val, iswrite);
    }
}
#endif

/*******************************************************************************
 * Name: lpc17_getreg
 *
 * Description:
 *   Get the contents of an LPC17xx register
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBHOST_REGDEBUG
static uint32_t lpc17_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  lpc17_checkreg(addr, val, false);
  return val;
}
#endif

/*******************************************************************************
 * Name: lpc17_putreg
 *
 * Description:
 *   Set the contents of an LPC17xx register to a value
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBHOST_REGDEBUG
static void lpc17_putreg(uint32_t val, uint32_t addr)
{
  /* Check if we need to print this value */

  lpc17_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: lpc17_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 *******************************************************************************/

static void lpc17_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: lpc17_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 *******************************************************************************/

static inline uint16_t lpc17_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: lpc17_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 *******************************************************************************/

#if 0 /* Not used */
static void lpc17_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}
#endif

/*******************************************************************************
 * Name: lpc17_edfree
 *
 * Description:
 *   Return an endpoint descriptor to the free list
 *
 *******************************************************************************/

static inline void lpc17_edfree(struct lpc17_ed_s *ed)
{
  struct lpc17_list_s *entry = (struct lpc17_list_s *)ed;

  /* Put the ED back into the free list */

  entry->flink = g_edfree;
  g_edfree     = entry;
}

/*******************************************************************************
 * Name: lpc17_tdalloc
 *
 * Description:
 *   Allocate an transfer descriptor from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protected from conconcurrent access to the TD pool by the interrupt
 *     handler
 *   - Protection from re-entrance must be assured by the caller
 *
 *******************************************************************************/

static struct lpc17_gtd_s *lpc17_tdalloc(void)
{
  struct lpc17_gtd_s *ret;
  irqstate_t flags;

  /* Disable interrupts momentarily so that lpc17_tdfree is not called from the
   * interrupt handler.
   */

  flags = irqsave();
  ret   = (struct lpc17_gtd_s *)g_tdfree;
  if (ret)
    {
      g_tdfree = ((struct lpc17_list_s*)ret)->flink;
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: lpc17_tdfree
 *
 * Description:
 *   Return an transfer descriptor to the free list
 *
 * Assumptions:
 *   - Only called from the WDH interrupt handler (and during initialization).
 *   - Interrupts are disabled in any case.
 *
 *******************************************************************************/

static void lpc17_tdfree(struct lpc17_gtd_s *td)
{
  struct lpc17_list_s *tdfree = (struct lpc17_list_s *)td;

  /* This should not happen but just to be safe, don't free the common, pre-
   * allocated tail TD.
   */

 if (tdfree != NULL && td != TDTAIL)
    {
      tdfree->flink = g_tdfree;
      g_tdfree      = tdfree;
    }
}

/*******************************************************************************
 * Name: lpc17_tballoc
 *
 * Description:
 *   Allocate an request/descriptor transfer buffer from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protection from re-entrance must be assured by the caller
 *
 *******************************************************************************/

static uint8_t *lpc17_tballoc(void)
{
  uint8_t *ret = (uint8_t *)g_tbfree;
  if (ret)
    {
      g_tbfree = ((struct lpc17_list_s*)ret)->flink;
    }
  return ret;
}

/*******************************************************************************
 * Name: lpc17_tbfree
 *
 * Description:
 *   Return an request/descriptor transfer buffer to the free list
 *
 *******************************************************************************/

static void lpc17_tbfree(uint8_t *buffer)
{
  struct lpc17_list_s *tbfree = (struct lpc17_list_s *)buffer;

  if (tbfree)
    {
      tbfree->flink              = g_tbfree;
      g_tbfree                   = tbfree;
    }
}

/*******************************************************************************
 * Name: lpc17_allocio
 *
 * Description:
 *   Allocate an IO buffer from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protection from re-entrance must be assured by the caller
 *
 *******************************************************************************/

#if LPC17_IOBUFFERS > 0
static uint8_t *lpc17_allocio(void)
{
  uint8_t *ret;
  irqstate_t flags;

  /* lpc17_freeio() may be called from the interrupt level */

  flags = irqsave();
  ret = (uint8_t *)g_iofree;
  if (ret)
    {
      g_iofree = ((struct lpc17_list_s*)ret)->flink;
    }

  irqrestore(flags);
  return ret;
}
#endif

/*******************************************************************************
 * Name: lpc17_freeio
 *
 * Description:
 *   Return an TD buffer to the free list
 *
 *******************************************************************************/

#if LPC17_IOBUFFERS > 0
static void lpc17_freeio(uint8_t *buffer)
{
  struct lpc17_list_s *iofree;
  irqstate_t flags;

  /* Could be called from the interrupt level */

  flags         = irqsave();
  iofree        = (struct lpc17_list_s *)buffer;
  iofree->flink = g_iofree;
  g_iofree      = iofree;
  irqrestore(flags);
}
#endif

/*******************************************************************************
 * Name: lpc17_alloc_xfrinfo
 *
 * Description:
 *   Allocate an asynchronous data structure from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protection from re-entrance must be assured by the caller
 *
 *******************************************************************************/

static struct lpc17_xfrinfo_s *lpc17_alloc_xfrinfo(void)
{
  struct lpc17_xfrinfo_s *ret;
  irqstate_t flags;

  /* lpc17_free_xfrinfo() may be called from the interrupt level */

  flags = irqsave();
  ret = (struct lpc17_xfrinfo_s *)g_xfrfree;
  if (ret)
    {
      g_xfrfree = ((struct lpc17_list_s*)ret)->flink;
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: lpc17_freeio
 *
 * Description:
 *   Return an TD buffer to the free list
 *
 *******************************************************************************/

static void lpc17_free_xfrinfo(struct lpc17_xfrinfo_s *xfrinfo)
{
  struct lpc17_list_s *node;
  irqstate_t flags;

  /* Could be called from the interrupt level */

  flags        = irqsave();
  node         = (struct lpc17_list_s *)xfrinfo;
  node->flink  = g_xfrfree;
  g_xfrfree    = node;
  irqrestore(flags);
}

/*******************************************************************************
 * Name: lpc17_addctrled
 *
 * Description:
 *   Helper function to add an ED to the control list.
 *
 *******************************************************************************/

static inline int lpc17_addctrled(struct lpc17_usbhost_s *priv,
                                  struct lpc17_ed_s *ed)
{
  irqstate_t flags;
  uint32_t regval;

  /* Disable control list processing while we modify the list */

  flags   = irqsave();
  regval = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_CLE;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);

  /* Add the new bulk ED to the head of the bulk list */

  ed->hw.nexted = lpc17_getreg(LPC17_USBHOST_CTRLHEADED);
  lpc17_putreg((uint32_t)ed, LPC17_USBHOST_CTRLHEADED);

  /* Re-enable control list processing. */

  lpc17_putreg(0, LPC17_USBHOST_CTRLED);

  regval = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval |= OHCI_CTRL_CLE;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc17_remctrled
 *
 * Description:
 *   Helper function remove an ED from the control list.
 *
 *******************************************************************************/

static inline int lpc17_remctrled(struct lpc17_usbhost_s *priv,
                                  struct lpc17_ed_s *ed)
{
  struct lpc17_ed_s *curr;
  struct lpc17_ed_s *prev;
  struct lpc17_ed_s *head;
  irqstate_t flags;
  uint32_t regval;

  /* Disable control list processing while we modify the list */

  flags   = irqsave();
  regval = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_CLE;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);

  /* Find the ED in the control list. */

  head = (struct lpc17_ed_s *)lpc17_getreg(LPC17_USBHOST_CTRLHEADED);
  for (prev = NULL, curr = head;
       curr && curr != ed;
       prev = curr, curr = (struct lpc17_ed_s *)curr->hw.nexted);

  /* It would be a bug if we do not find the ED in the control list. */

  DEBUGASSERT(curr != NULL);

  /* Remove the ED from the control list */

  if (curr != NULL)
    {
      /* Is this ED the first on in the control list? */

      if (prev == NULL)
        {
          /* Yes... set the head of the control list to skip over this ED */

          head = (struct lpc17_ed_s *)ed->hw.nexted;
          lpc17_putreg((uint32_t)head, LPC17_USBHOST_CTRLHEADED);
        }
      else
        {
          /* No.. set the forward link of the previous ED in the list
           * skip over this ED.
           */

          prev->hw.nexted = ed->hw.nexted;
        }

      /* Just in case the hardware happens to be processing this ed now...
       * it should go back to the control list head.
       */

      ed->hw.nexted = 0;
    }

  /* Re-enable control list processing if the control list is still non-empty
   * after removing the ED node.
   */

  lpc17_putreg(0, LPC17_USBHOST_CTRLED);
  if (lpc17_getreg(LPC17_USBHOST_CTRLHEADED) != 0)
    {
      /* If the control list is now empty, then disable it */

      regval  = lpc17_getreg(LPC17_USBHOST_CTRL);
      regval &= ~OHCI_CTRL_CLE;
      lpc17_putreg(regval, LPC17_USBHOST_CTRL);
    }

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc17_addbulked
 *
 * Description:
 *   Helper function to add an ED to the bulk list.
 *
 *******************************************************************************/

static inline int lpc17_addbulked(struct lpc17_usbhost_s *priv,
                                  struct lpc17_ed_s *ed)
{
#ifndef CONFIG_USBHOST_BULK_DISABLE
  irqstate_t flags;
  uint32_t regval;

  /* Disable bulk list processing while we modify the list */

  flags   = irqsave();
  regval  = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_BLE;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);

  /* Add the new bulk ED to the head of the bulk list */

  ed->hw.nexted = lpc17_getreg(LPC17_USBHOST_BULKHEADED);
  lpc17_putreg((uint32_t)ed, LPC17_USBHOST_BULKHEADED);

  /* Re-enable bulk list processing. */

  lpc17_putreg(0, LPC17_USBHOST_BULKED);

  regval  = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval |= OHCI_CTRL_BLE;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);

  irqrestore(flags);
  return OK;
#else
  return -ENOSYS;
#endif
}

/*******************************************************************************
 * Name: lpc17_rembulked
 *
 * Description:
 *   Helper function remove an ED from the bulk list.
 *
 *******************************************************************************/

static inline int lpc17_rembulked(struct lpc17_usbhost_s *priv,
                                  struct lpc17_ed_s *ed)
{
#ifndef CONFIG_USBHOST_BULK_DISABLE
  struct lpc17_ed_s *curr;
  struct lpc17_ed_s *prev;
  struct lpc17_ed_s *head;
  irqstate_t flags;
  uint32_t regval;

  /* Disable bulk list processing while we modify the list */

  flags   = irqsave();
  regval  = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_BLE;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);

  /* Find the ED in the bulk list. */

  head = (struct lpc17_ed_s *)lpc17_getreg(LPC17_USBHOST_BULKHEADED);
  for (prev = NULL, curr = head;
       curr && curr != ed;
       prev = curr, curr = (struct lpc17_ed_s *)curr->hw.nexted);

  /* It would be a bug if we do not find the ED in the bulk list. */

  DEBUGASSERT(curr != NULL);

  /* Remove the ED from the bulk list */

  if (curr != NULL)
    {
      /* Is this ED the first on in the bulk list? */

      if (prev == NULL)
        {
          /* Yes... set the head of the bulk list to skip over this ED */

          head = (struct lpc17_ed_s *)ed->hw.nexted;
          lpc17_putreg((uint32_t)head, LPC17_USBHOST_BULKHEADED);
        }
      else
        {
          /* No.. set the forward link of the previous ED in the list
           * skip over this ED.
           */

          prev->hw.nexted = ed->hw.nexted;
        }
    }

  /* Re-enable bulk list processing if the bulk list is still non-empty
   * after removing the ED node.
   */

  lpc17_putreg(0, LPC17_USBHOST_BULKED);
  if (lpc17_getreg(LPC17_USBHOST_BULKHEADED) != 0)
    {
      /* If the bulk list is now empty, then disable it */

      regval  = lpc17_getreg(LPC17_USBHOST_CTRL);
      regval |= OHCI_CTRL_BLE;
      lpc17_putreg(regval, LPC17_USBHOST_CTRL);
    }

  irqrestore(flags);
  return OK;
#else
  return -ENOSYS;
#endif
}

/*******************************************************************************
 * Name: lpc17_getinterval
 *
 * Description:
 *   Convert the endpoint polling interval into a HCCA table increment
 *
 *******************************************************************************/

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static unsigned int lpc17_getinterval(uint8_t interval)
{
  /* The bInterval field of the endpoint descriptor contains the polling interval
   * for interrupt and isochronous endpoints. For other types of endpoint, this
   * value should be ignored. bInterval is provided in units of 1MS frames.
   */

  if (interval < 3)
    {
      return 2;
    }
  else if (interval < 7)
    {
      return 4;
    }
  else if (interval < 15)
    {
      return 8;
    }
  else if (interval < 31)
    {
      return 16;
    }
  else
    {
      return 32;
    }
}
#endif

/*******************************************************************************
 * Name: lpc17_setinttab
 *
 * Description:
 *   Set the interrupt table to the selected value using the provided interval
 *   and offset.
 *
 *******************************************************************************/

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static void lpc17_setinttab(uint32_t value, unsigned int interval, unsigned int offset)
{
  unsigned int i;
  for (i = offset; i < HCCA_INTTBL_WSIZE; i += interval)
    {
      HCCA->inttbl[i] = value;
    }
}
#endif

/*******************************************************************************
 * Name: lpc17_addinted
 *
 * Description:
 *   Helper function to add an ED to the HCCA interrupt table.
 *
 *   To avoid reshuffling the table so much and to keep life simple in general,
 *    the following rules are applied:
 *
 *     1. IN EDs get the even entries, OUT EDs get the odd entries.
 *     2. Add IN/OUT EDs are scheduled together at the minimum interval of all
 *        IN/OUT EDs.
 *
 *   This has the following consequences:
 *
 *     1. The minimum support polling rate is 2MS, and
 *     2. Some devices may get polled at a much higher rate than they request.
 *
 *******************************************************************************/

static inline int lpc17_addinted(struct lpc17_usbhost_s *priv,
                                 const struct usbhost_epdesc_s *epdesc,
                                 struct lpc17_ed_s *ed)
{
#ifndef CONFIG_USBHOST_INT_DISABLE
  unsigned int interval;
  unsigned int offset;
  uint32_t head;
  uint32_t regval;

  /* Disable periodic list processing.  Does this take effect immediately?  Or
   * at the next SOF... need to check.
   */

  regval  = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_PLE;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);

  /* Get the quantized interval value associated with this ED and save it
   * in the ED.
   */

  interval     = lpc17_getinterval(epdesc->interval);
  ed->interval = interval;
  uvdbg("interval: %d->%d\n", epdesc->interval, interval);

  /* Get the offset associated with the ED direction. IN EDs get the even
   * entries, OUT EDs get the odd entries.
   *
   * Get the new, minimum interval. Add IN/OUT EDs are scheduled together
   * at the minimum interval of all IN/OUT EDs.
   */

  if (epdesc->in)
    {
      offset = 0;
      if (priv->ininterval > interval)
        {
          priv->ininterval = interval;
        }
      else
        {
          interval = priv->ininterval;
        }
    }
  else
    {
      offset = 1;
      if (priv->outinterval > interval)
        {
          priv->outinterval = interval;
        }
      else
        {
          interval = priv->outinterval;
        }
    }
  uvdbg("min interval: %d offset: %d\n", interval, offset);

  /* Get the head of the first of the duplicated entries.  The first offset
   * entry is always guaranteed to contain the common ED list head.
   */

  head = HCCA->inttbl[offset];

  /* Clear all current entries in the interrupt table for this direction */

  lpc17_setinttab(0, 2, offset);

  /* Add the new ED before the old head of the periodic ED list and set the
   * new ED as the head ED in all of the appropriate entries of the HCCA
   * interrupt table.
   */

  ed->hw.nexted = head;
  lpc17_setinttab((uint32_t)ed, interval, offset);
  uvdbg("head: %08x next: %08x\n", ed, head);

  /* Re-enabled periodic list processing */

  regval  = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval |= OHCI_CTRL_PLE;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);
  return OK;
#else
  return -ENOSYS;
#endif
}

/*******************************************************************************
 * Name: lpc17_reminted
 *
 * Description:
 *   Helper function to remove an ED from the HCCA interrupt table.
 *
 *   To avoid reshuffling the table so much and to keep life simple in general,
 *    the following rules are applied:
 *
 *     1. IN EDs get the even entries, OUT EDs get the odd entries.
 *     2. Add IN/OUT EDs are scheduled together at the minimum interval of all
 *        IN/OUT EDs.
 *
 *   This has the following consequences:
 *
 *     1. The minimum support polling rate is 2MS, and
 *     2. Some devices may get polled at a much higher rate than they request.
 *
 *******************************************************************************/

static inline int lpc17_reminted(struct lpc17_usbhost_s *priv,
                                 struct lpc17_ed_s *ed)
{
#ifndef CONFIG_USBHOST_INT_DISABLE
  struct lpc17_ed_s *head;
  struct lpc17_ed_s *curr;
  struct lpc17_ed_s *prev;
  unsigned int       interval;
  unsigned int       offset;
  uint32_t           regval;

  /* Disable periodic list processing.  Does this take effect immediately?  Or
   * at the next SOF... need to check.
   */

  regval  = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_PLE;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);

  /* Get the offset associated with the ED direction. IN EDs get the even
   * entries, OUT EDs get the odd entries.
   */

  if ((ed->hw.ctrl & ED_CONTROL_D_MASK) == ED_CONTROL_D_IN)
    {
      offset = 0;
    }
  else
    {
      offset = 1;
    }

  /* Get the head of the first of the duplicated entries.  The first offset
   * entry is always guaranteed to contain the common ED list head.
   */

  head = (struct lpc17_ed_s *)HCCA->inttbl[offset];
  uvdbg("ed: %08x head: %08x next: %08x offset: %d\n",
        ed, head, head ? head->hw.nexted : 0, offset);

  /* Find the ED to be removed in the ED list */

  for (curr = head, prev = NULL;
       curr && curr != ed;
       prev = curr, curr = (struct lpc17_ed_s *)curr->hw.nexted);

  /* Hmmm.. It would be a bug if we do not find the ED in the bulk list. */

  DEBUGASSERT(curr != NULL);
  if (curr != NULL)
    {
      /* Clear all current entries in the interrupt table for this direction */

      lpc17_setinttab(0, 2, offset);

      /* Remove the ED from the list..  Is this ED the first on in the list? */

      if (prev == NULL)
        {
          /* Yes... set the head of the bulk list to skip over this ED */

          head = (struct lpc17_ed_s *)ed->hw.nexted;
        }
      else
        {
          /* No.. set the forward link of the previous ED in the list
           * skip over this ED.
           */

          prev->hw.nexted = ed->hw.nexted;
        }

        uvdbg("ed: %08x head: %08x next: %08x\n",
              ed, head, head ? head->hw.nexted : 0);

      /* Calculate the new minimum interval for this list */

      interval = MAX_PERINTERVAL;
      for (curr = head; curr; curr = (struct lpc17_ed_s *)curr->hw.nexted)
        {
          if (curr->interval < interval)
            {
              interval = curr->interval;
            }
        }

      uvdbg("min interval: %d offset: %d\n", interval, offset);

      /* Save the new minimum interval */

      if ((ed->hw.ctrl & ED_CONTROL_D_MASK) == ED_CONTROL_D_IN)
        {
          priv->ininterval  = interval;
        }
      else
        {
          priv->outinterval = interval;
        }

      /* Set the head ED in all of the appropriate entries of the HCCA interrupt
       * table (head might be NULL).
       */

      lpc17_setinttab((uint32_t)head, interval, offset);
    }

  /* Re-enabled periodic list processing */

  if (head != NULL)
    {
      regval  = lpc17_getreg(LPC17_USBHOST_CTRL);
      regval |= OHCI_CTRL_PLE;
      lpc17_putreg(regval, LPC17_USBHOST_CTRL);
    }

  return OK;
#else
  return -ENOSYS;
#endif
}

/*******************************************************************************
 * Name: lpc17_addisoced
 *
 * Description:
 *   Helper functions to add an ED to the periodic table.
 *
 *******************************************************************************/

static inline int lpc17_addisoced(struct lpc17_usbhost_s *priv,
                                  const struct usbhost_epdesc_s *epdesc,
                                  struct lpc17_ed_s *ed)
{
#ifndef CONFIG_USBHOST_ISOC_DISABLE
#  warning "Isochronous endpoints not yet supported"
#endif
  return -ENOSYS;
}

/*******************************************************************************
 * Name: lpc17_remisoced
 *
 * Description:
 *   Helper functions to remove an ED from the periodic table.
 *
 *******************************************************************************/

static inline int lpc17_remisoced(struct lpc17_usbhost_s *priv,
                                  struct lpc17_ed_s *ed)
{
#ifndef CONFIG_USBHOST_ISOC_DISABLE
#  warning "Isochronous endpoints not yet supported"
#endif
  return -ENOSYS;
}

/*******************************************************************************
 * Name: lpc17_enqueuetd
 *
 * Description:
 *   Enqueue a transfer descriptor.  Notice that this function only supports
 *   queue on TD per ED.
 *
 *******************************************************************************/

static int lpc17_enqueuetd(struct lpc17_usbhost_s *priv,
                           struct lpc17_ed_s *ed, uint32_t dirpid,
                           uint32_t toggle, volatile uint8_t *buffer, size_t buflen)
{
  struct lpc17_gtd_s *td;
  int ret = -ENOMEM;

  /* Allocate a TD from the free list */

  td = lpc17_tdalloc();
  if (td != NULL)
    {
      /* Initialize the allocated TD and link it before the common tail TD. */

      td->hw.ctrl         = (GTD_STATUS_R | dirpid | TD_DELAY(0) | toggle | GTD_STATUS_CC_MASK);
      TDTAIL->hw.ctrl     = 0;
      td->hw.cbp          = (uint32_t)buffer;
      TDTAIL->hw.cbp      = 0;
      td->hw.nexttd       = (uint32_t)TDTAIL;
      TDTAIL->hw.nexttd   = 0;
      td->hw.be           = (uint32_t)(buffer + (buflen - 1));
      TDTAIL->hw.be       = 0;

      /* Configure driver-only fields in the extended TD structure */

      td->ed              = ed;

      /* Link the td to the head of the ED's TD list */

      ed->hw.headp        = (uint32_t)td | ((ed->hw.headp) & ED_HEADP_C);
      ed->hw.tailp        = (uint32_t)TDTAIL;

      ret                 = OK;
    }

  return ret;
}

/*******************************************************************************
 * Name: lpc17_wdhwait
 *
 * Description:
 *   Set the request for the Writeback Done Head event well BEFORE enabling the
 *   transfer (as soon as we are absolutely committed to the to avoid transfer).
 *   We do this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 *******************************************************************************/

static int lpc17_wdhwait(struct lpc17_usbhost_s *priv, struct lpc17_ed_s *ed)
{
  struct lpc17_xfrinfo_s *xfrinfo;
  irqstate_t flags = irqsave();
  int        ret   = -ENODEV;

  DEBUGASSERT(ed && ed->xfrinfo);
  xfrinfo = ed->xfrinfo;

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set wdhwait to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer completed.
       */

      xfrinfo->wdhwait = true;
      ret = OK;
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: lpc17_ctrltd
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  This function
 *   will enqueue the request and wait for it to complete.  Only one transfer
 *   may be queued; Neither these methods nor the transfer() method can be
 *   called again until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 *******************************************************************************/

static int lpc17_ctrltd(struct lpc17_usbhost_s *priv, struct lpc17_ed_s *ed,
                        uint32_t dirpid, uint8_t *buffer, size_t buflen)
{
  struct lpc17_xfrinfo_s *xfrinfo;
  uint32_t toggle;
  uint32_t regval;
  int ret;

  /* Allocate a structure to retain the information needed when the transfer
   * completes.
   */

  DEBUGASSERT(ed->xfrinfo == NULL);

  xfrinfo = lpc17_alloc_xfrinfo();
  if (xfrinfo == NULL)
    {
      udbg("ERROR: lpc17_alloc_xfrinfo failed\n");
      return -ENOMEM;
    }

  /* Initialize the transfer structure */

  memset(xfrinfo, 0, sizeof(struct lpc17_xfrinfo_s));
  xfrinfo->buffer = buffer;
  xfrinfo->buflen = buflen;

  ed->xfrinfo = xfrinfo;

  /* Set the request for the Writeback Done Head event well BEFORE enabling the
   * transfer.
   */

  ret = lpc17_wdhwait(priv, ed);
  if (ret < 0)
    {
      udbg("ERROR: Device disconnected\n");
      goto errout_with_xfrinfo;
    }

  /* Configure the toggle field in the TD */

  if (dirpid == GTD_STATUS_DP_SETUP)
    {
      toggle = GTD_STATUS_T_DATA0;
    }
  else
    {
      toggle = GTD_STATUS_T_DATA1;
    }

  /* Then enqueue the transfer */

  xfrinfo->tdstatus = TD_CC_NOERROR;
  ret = lpc17_enqueuetd(priv, ed, dirpid, toggle, buffer, buflen);
  if (ret == OK)
    {
      /* Set ControlListFilled.  This bit is used to indicate whether there are
       * TDs on the Control list.
       */

      regval = lpc17_getreg(LPC17_USBHOST_CMDST);
      regval |= OHCI_CMDST_CLF;
      lpc17_putreg(regval, LPC17_USBHOST_CMDST);

      /* Wait for the Writeback Done Head interrupt */

      lpc17_takesem(&ed->wdhsem);

      /* Check the TD completion status bits */

      if (xfrinfo->tdstatus == TD_CC_NOERROR)
        {
          ret = OK;
        }
      else
        {
          udbg("ERROR: Bad TD completion status: %d\n", xfrinfo->tdstatus);
          ret = xfrinfo->tdstatus == TD_CC_STALL ? -EPERM : -EIO;
        }
    }

  /* Make sure that there is no outstanding request on this endpoint */

errout_with_xfrinfo:
  lpc17_free_xfrinfo(xfrinfo);
  ed->xfrinfo = NULL;
  return ret;
}

/*******************************************************************************
 * Name: lpc17_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 *******************************************************************************/

static int lpc17_usbinterrupt(int irq, void *context)
{
  struct lpc17_usbhost_s *priv = &g_usbhost;
  struct lpc17_ed_s *ed;
  struct lpc17_xfrinfo_s *xfrinfo;
  uintptr_t tmp;
  uint32_t intst;
  uint32_t pending;
  uint32_t regval;

  /* Read Interrupt Status and mask out interrupts that are not enabled. */

  intst  = lpc17_getreg(LPC17_USBHOST_INTST);
  regval = lpc17_getreg(LPC17_USBHOST_INTEN);
  ullvdbg("INST: %08x INTEN: %08x\n", intst, regval);

  pending = intst & regval;
  if (pending != 0)
    {
      /* Root hub status change interrupt */

      if ((pending & OHCI_INT_RHSC) != 0)
        {
          uint32_t rhportst1 = lpc17_getreg(LPC17_USBHOST_RHPORTST1);
          ullvdbg("Root Hub Status Change, RHPORTST1: %08x\n", rhportst1);

          if ((rhportst1 & OHCI_RHPORTST_CSC) != 0)
            {
              uint32_t rhstatus = lpc17_getreg(LPC17_USBHOST_RHSTATUS);
              ullvdbg("Connect Status Change, RHSTATUS: %08x\n", rhstatus);

              /* If DRWE is set, Connect Status Change indicates a remote wake-up event */

              if (rhstatus & OHCI_RHSTATUS_DRWE)
                {
                  ullvdbg("DRWE: Remote wake-up\n");
                }

              /* Otherwise... Not a remote wake-up event */

              else
                {
                  /* Check current connect status */

                  if ((rhportst1 & OHCI_RHPORTST_CCS) != 0)
                    {
                      /* Connected ... Did we just become connected? */

                      if (!priv->connected)
                        {
                          /* Yes.. connected. */

                          ullvdbg("Connected\n");
                          priv->connected = true;
                          priv->change    = true;

                          /* Notify any waiters */

                          if (priv->pscwait)
                            {
                              lpc17_givesem(&priv->pscsem);
                              priv->pscwait = false;
                            }
                        }
                      else
                        {
                          ulldbg("Spurious status change (connected)\n");
                        }

                      /* The LSDA (Low speed device attached) bit is valid
                       * when CCS == 1.
                       */

                      if ((rhportst1 & OHCI_RHPORTST_LSDA) != 0)
                        {
                          priv->rhport.hport.speed = USB_SPEED_LOW;
                        }
                      else
                        {
                          priv->rhport.hport.speed = USB_SPEED_FULL;
                        }

                      ullvdbg("Speed:%d\n", priv->rhport.hport.speed);
                    }

                  /* Check if we are now disconnected */

                  else if (priv->connected)
                    {
                      /* Yes.. disconnect the device */

                      ullvdbg("Disconnected\n");
                      priv->connected = false;
                      priv->change    = true;

                      /* Set the port speed to the default (FULL).  We cannot
                       * yet free the function address.  That has to be done
                       * by the class when responds to the disconnection.
                       */

                      priv->rhport.hport.speed = USB_SPEED_FULL;

                      /* Are we bound to a class instance? */

                      if (priv->rhport.hport.devclass)
                        {
                          /* Yes.. Disconnect the class */

                          CLASS_DISCONNECTED(priv->rhport.hport.devclass);
                          priv->rhport.hport.devclass = NULL;
                        }

                      /* Notify any waiters for the Root Hub Status change event */

                      if (priv->pscwait)
                        {
                          lpc17_givesem(&priv->pscsem);
                          priv->pscwait = false;
                        }
                    }
                  else
                    {
                       ulldbg("Spurious status change (disconnected)\n");
                    }
                }

              /* Clear the status change interrupt */

              lpc17_putreg(OHCI_RHPORTST_CSC, LPC17_USBHOST_RHPORTST1);
            }

          /* Check for port reset status change */

          if ((rhportst1 & OHCI_RHPORTST_PRSC) != 0)
            {
              /* Release the RH port from reset */

              lpc17_putreg(OHCI_RHPORTST_PRSC, LPC17_USBHOST_RHPORTST1);
            }
        }

      /* Writeback Done Head interrupt */

      if ((pending & OHCI_INT_WDH) != 0)
        {
          struct lpc17_gtd_s *td;
          struct lpc17_gtd_s *next;

          /* The host controller just wrote the list of finished TDs into the HCCA
           * done head.  This may include multiple packets that were transferred
           * in the preceding frame.
           *
           * Remove the TD(s) from the Writeback Done Head in the HCCA and return
           * them to the free list.  Note that this is safe because the hardware
           * will not modify the writeback done head again until the WDH bit is
           * cleared in the interrupt status register.
           */

          td = (struct lpc17_gtd_s *)(HCCA->donehead & HCCA_DONEHEAD_MASK);
          HCCA->donehead = 0;

          /* Process each TD in the write done list */

          for (; td; td = next)
            {
              /* REVISIT: I have encountered bad TDs in the done list linked
               * after at least one good TD.  This is some consequence of how
               * transfers are being cancelled.  But for now, I have only
               * this work-around.
               */

              if ((uintptr_t)td < LPC17_TDFREE_BASE ||
                  (uintptr_t)td >= (LPC17_TDFREE_BASE + LPC17_TD_SIZE*CONFIG_USBHOST_NTDS))
                {
                  break;
                }

              /* Get the ED in which this TD was enqueued */

              ed      = td->ed;
              DEBUGASSERT(ed != NULL);

              /* If there is a transfer in progress, then the xfrinfo pointer will be
               * non-NULL.  But it appears that a NULL pointer may be received with a
               * spurious interrupt such as may occur after a transfer is cancelled.
               */

              xfrinfo = ed->xfrinfo;
              if (xfrinfo)
                {
                  /* Save the condition code from the (single) TD status/control
                   * word.
                   */

                  xfrinfo->tdstatus = (td->hw.ctrl & GTD_STATUS_CC_MASK) >> GTD_STATUS_CC_SHIFT;

#ifdef CONFIG_DEBUG_USB
                  if (xfrinfo->tdstatus != TD_CC_NOERROR)
                    {
                      /* The transfer failed for some reason... dump some diagnostic info. */

                      ulldbg("ERROR: ED xfrtype:%d TD CTRL:%08x/CC:%d RHPORTST1:%08x\n",
                             ed->xfrtype, td->hw.ctrl, xfrinfo->tdstatus,
                             lpc17_getreg(LPC17_USBHOST_RHPORTST1));
                    }
#endif

                  /* Determine the number of bytes actually transfer by
                   * subtracting the buffer start address from the CBP.  A
                   * value of zero means that all bytes were transferred.
                   */

                  tmp = (uintptr_t)td->hw.cbp;
                  if (tmp == 0)
                    {
                      /* Set the (fake) CBP to the end of the buffer + 1 */

                      tmp = xfrinfo->buflen;
                    }
                  else
                    {
                      DEBUGASSERT(tmp >= (uintptr_t)xfrinfo->buffer);

                      /* Determine the size of the transfer by subtracting
                       * the current buffer pointer (CBP) from the initial
                       * buffer pointer (on packet receipt only).
                       */

                      tmp -= (uintptr_t)xfrinfo->buffer;
                      DEBUGASSERT(tmp < UINT16_MAX);
                    }

                  xfrinfo->xfrd = (uint16_t)tmp;

                  /* Return the TD to the free list */

                  next = (struct lpc17_gtd_s *)td->hw.nexttd;
                  lpc17_tdfree(td);

                  if (xfrinfo->wdhwait)
                    {
                      /* Wake up the thread waiting for the WDH event */

                      lpc17_givesem(&ed->wdhsem);
                      xfrinfo->wdhwait = false;
                    }

#ifdef CONFIG_USBHOST_ASYNCH
                  /* Perform any pending callbacks for the case of
                   * asynchronous transfers.
                   */

                  else if (xfrinfo->callback)
                    {
                      DEBUGASSERT(xfrinfo->wdhwait == false);
                      lpc17_asynch_completion(priv, ed);
                    }
#endif
                }
            }
        }

#ifdef CONFIG_DEBUG_USB
      if ((pending & LPC17_DEBUG_INTS) != 0)
        {
          ulldbg("ERROR: Unhandled interrupts INTST:%08x\n", intst);
        }
#endif

      /* Clear interrupt status register */

      lpc17_putreg(intst, LPC17_USBHOST_INTST);
    }

  return OK;
}

/*******************************************************************************
 * USB Host Controller Operations
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc17_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from the call to
 *      the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected the
 *      connection related event.
 *
 * Returned Values:
 *   Zero (OK) is returned on success when a device in connected or
 *   disconnected. This function will not return until either (1) a device is
 *   connected or disconnect to/from any hub port or until (2) some failure
 *   occurs.  On a failure, a negated errno value is returned indicating the
 *   nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int lpc17_wait(struct usbhost_connection_s *conn,
                      struct usbhost_hubport_s **hport)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)&g_usbhost;
  struct usbhost_hubport_s *connport;
  irqstate_t flags;

  flags = irqsave();
  for (;;)
    {
      /* Is there a change in the connection state of the single root hub
       * port?
       */

      if (priv->change)
        {
          connport = &priv->rhport.hport;
          priv->change = false;

          /* Yes.. check for false alarms */

          if (priv->connected != connport->connected)
            {
              /* Not a false alarm.. Remember the new state */

              connport->connected = priv->connected;

              /* And return the root hub port */

              *hport = connport;
              irqrestore(flags);

              udbg("RHport Connected: %s\n",
                   connport->connected ? "YES" : "NO");

              return OK;
            }
        }

#ifdef CONFIG_USBHOST_HUB
      /* Is a device connected to an external hub? */

      if (priv->hport)
        {
          /* Yes.. return the external hub port */

          connport = (struct usbhost_hubport_s *)priv->hport;
          priv->hport = NULL;

          *hport = connport;
          irqrestore(flags);

          udbg("Hub port Connected: %s\n", connport->connected ? "YES" : "NO");
          return OK;
        }
#endif

      /* Wait for the next connection event */

      priv->pscwait = true;
      lpc17_takesem(&priv->pscsem);
    }
}

/*******************************************************************************
 * Name: lpc17_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the connect() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from
 *      the call to the USB driver initialization logic.
 *   hport - The descriptor of the hub port that has the newly connected
 *      device.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 *******************************************************************************/

static int lpc17_rh_enumerate(struct usbhost_connection_s *conn,
                              struct usbhost_hubport_s *hport)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)&g_usbhost;
  DEBUGASSERT(conn != NULL && hport != NULL && hport->port == 0);

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!priv->connected)
    {
      /* No, return an error */

      udbg("Not connected\n");
      return -ENODEV;
    }

  /* USB 2.0 spec says at least 50ms delay before port reset */

  (void)usleep(100*1000);

  /* Put RH port 1 in reset (the LPC176x supports only a single downstream port) */

  lpc17_putreg(OHCI_RHPORTST_PRS, LPC17_USBHOST_RHPORTST1);

  /* Wait for the port reset to complete */

  while ((lpc17_getreg(LPC17_USBHOST_RHPORTST1) & OHCI_RHPORTST_PRS) != 0);

  /* Release RH port 1 from reset and wait a bit */

  lpc17_putreg(OHCI_RHPORTST_PRSC, LPC17_USBHOST_RHPORTST1);
  (void)usleep(200*1000);
  return OK;
}

static int lpc17_enumerate(FAR struct usbhost_connection_s *conn,
                           FAR struct usbhost_hubport_s *hport)
{
  int ret;

  DEBUGASSERT(hport);

  /* If this is a connection on the root hub, then we need to go to
   * little more effort to get the device speed.  If it is a connection
   * on an external hub, then we already have that information.
   */

#ifdef CONFIG_USBHOST_HUB
  if (ROOTHUB(hport))
#endif
    {
      ret = lpc17_rh_enumerate(conn, hport);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Then let the common usbhost_enumerate do the real enumeration. */

  uvdbg("Enumerate the device\n");
  ret = usbhost_enumerate(hport, &hport->devclass);
  if (ret < 0)
    {
      udbg("ERROR: Enumeration failed: %d\n", ret);
    }

  return ret;
}

/************************************************************************************
 * Name: lpc17_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support
 *   an external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep0 - The (opaque) EP0 endpoint instance
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *   speed - The speed of the port USB_SPEED_LOW, _FULL, or _HIGH
 *   mps (maxpacketsize) - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int lpc17_ep0configure(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                              uint8_t funcaddr, uint8_t speed, uint16_t maxpacketsize)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  struct lpc17_ed_s      *ed;
  uint32_t hwctrl;

  DEBUGASSERT(drvr != NULL && ep0 != NULL && funcaddr < 128 && maxpacketsize < 2048);
  ed = (struct lpc17_ed_s *)ep0;

  /* We must have exclusive access to EP0 and the control list */

  lpc17_takesem(&priv->exclsem);

  /* Set the EP0 ED control word */

  hwctrl = (uint32_t)funcaddr << ED_CONTROL_FA_SHIFT |
           (uint32_t)ED_CONTROL_D_TD1 |
           (uint32_t)maxpacketsize << ED_CONTROL_MPS_SHIFT;

  if (speed == USB_SPEED_LOW)
    {
      hwctrl |= ED_CONTROL_S;
    }

  ed->hw.ctrl = hwctrl;

  lpc17_givesem(&priv->exclsem);

  uvdbg("EP0 CTRL:%08x\n", ed->hw.ctrl);
  return OK;
}

/************************************************************************************
 * Name: lpc17_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int lpc17_epalloc(struct usbhost_driver_s *drvr,
                         const struct usbhost_epdesc_s *epdesc, usbhost_ep_t *ep)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  struct usbhost_hubport_s *hport;
  struct lpc17_ed_s *ed;
  int ret  = -ENOMEM;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(priv && epdesc && ep && priv->connected);

  /* We must have exclusive access to the ED pool, the bulk list, the periodic list
   * and the interrupt table.
   */

  lpc17_takesem(&priv->exclsem);

  /* Take the next ED from the beginning of the free list */

  ed = (struct lpc17_ed_s *)g_edfree;
  if (ed)
    {
      /* Remove the ED from the freelist */

      g_edfree = ((struct lpc17_list_s*)ed)->flink;

      /* Configure the endpoint descriptor. */

      memset((void*)ed, 0, sizeof(struct lpc17_ed_s));

      hport = epdesc->hport;
      ed->hw.ctrl = (uint32_t)(hport->funcaddr)      << ED_CONTROL_FA_SHIFT |
                    (uint32_t)(epdesc->addr)         << ED_CONTROL_EN_SHIFT |
                    (uint32_t)(epdesc->mxpacketsize) << ED_CONTROL_MPS_SHIFT;

      /* Get the direction of the endpoint.  For control endpoints, the
       * direction is in the TD.
       */

      if (epdesc->xfrtype == USB_EP_ATTR_XFER_CONTROL)
        {
          ed->hw.ctrl |= ED_CONTROL_D_TD1;
        }
      else if (epdesc->in)
        {
          ed->hw.ctrl |= ED_CONTROL_D_IN;
        }
      else
        {
          ed->hw.ctrl |= ED_CONTROL_D_OUT;
        }

      /* Check for a low-speed device */

      if (hport->speed == USB_SPEED_LOW)
        {
          ed->hw.ctrl |= ED_CONTROL_S;
        }

      /* Set the transfer type */

      ed->xfrtype = epdesc->xfrtype;

      /* Special Case isochronous transfer types */

#if 0 /* Isochronous transfers not yet supported */
      if (ed->xfrtype == USB_EP_ATTR_XFER_ISOC)
        {
          ed->hw.ctrl |= ED_CONTROL_F;
        }
#endif
      uvdbg("EP%d CTRL:%08x\n", epdesc->addr, ed->hw.ctrl);

      /* Initialize the semaphore that is used to wait for the endpoint
       * WDH event.
       */

      sem_init(&ed->wdhsem, 0, 0);

      /* Link the common tail TD to the ED's TD list */

      ed->hw.headp = (uint32_t)TDTAIL;
      ed->hw.tailp = (uint32_t)TDTAIL;

      /* Now add the endpoint descriptor to the appropriate list */

      switch (ed->xfrtype)
        {
        case USB_EP_ATTR_XFER_CONTROL:
          ret = lpc17_addctrled(priv, ed);
          break;

        case USB_EP_ATTR_XFER_BULK:
          ret = lpc17_addbulked(priv, ed);
          break;

        case USB_EP_ATTR_XFER_INT:
          ret = lpc17_addinted(priv, epdesc, ed);
          break;

        case USB_EP_ATTR_XFER_ISOC:
          ret = lpc17_addisoced(priv, epdesc, ed);
          break;

        default:
          ret = -EINVAL;
          break;
        }

      /* Was the ED successfully added? */

      if (ret < 0)
        {
          /* No.. destroy it and report the error */

          udbg("ERROR: Failed to queue ED for transfer type: %d\n", ed->xfrtype);
          sem_destroy(&ed->wdhsem);
          lpc17_edfree(ed);
        }
      else
        {
          /* Yes.. return an opaque reference to the ED */

          *ep = (usbhost_ep_t)ed;
        }
    }

  lpc17_givesem(&priv->exclsem);
  return ret;
}

/************************************************************************************
 * Name: lpc17_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The endpint to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int lpc17_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  struct lpc17_ed_s      *ed   = (struct lpc17_ed_s *)ep;
  int                     ret;

  /* There should not be any pending, real TDs linked to this ED */

  DEBUGASSERT(ed && (ed->hw.headp & ED_HEADP_ADDR_MASK) == LPC17_TDTAIL_ADDR);

  /* We must have exclusive access to the ED pool, the bulk list, the periodic list
   * and the interrupt table.
   */

  lpc17_takesem(&priv->exclsem);

  /* Remove the ED to the correct list depending on the trasfer type */

  switch (ed->xfrtype)
    {
    case USB_EP_ATTR_XFER_CONTROL:
      ret = lpc17_remctrled(priv, ed);
      break;

    case USB_EP_ATTR_XFER_BULK:
      ret = lpc17_rembulked(priv, ed);
      break;

    case USB_EP_ATTR_XFER_INT:
      ret = lpc17_reminted(priv, ed);
      break;

    case USB_EP_ATTR_XFER_ISOC:
      ret = lpc17_remisoced(priv, ed);
      break;

    default:
      ret = -EINVAL;
      break;
    }

  /* Destroy the semaphore */

  sem_destroy(&ed->wdhsem);

  /* Put the ED back into the free list */

  lpc17_edfree(ed);
  lpc17_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: lpc17_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_malloc.
 *
 *   This interface was optimized under a particular assumption.  It was assumed
 *   that the driver maintains a pool of small, pre-allocated buffers for descriptor
 *   traffic.  NOTE that size is not an input, but an output:  The size of the
 *   pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in which to
 *     return the maximum size of the allocated buffer memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int lpc17_alloc(struct usbhost_driver_s *drvr,
                       uint8_t **buffer, size_t *maxlen)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  DEBUGASSERT(priv && buffer && maxlen);
  int ret = -ENOMEM;

  /* We must have exclusive access to the transfer buffer pool */

  lpc17_takesem(&priv->exclsem);

  *buffer = lpc17_tballoc();
  if (*buffer)
    {
      *maxlen = CONFIG_USBHOST_TDBUFSIZE;
      ret = OK;
    }

  lpc17_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: lpc17_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to free that
 *   request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int lpc17_free(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  DEBUGASSERT(buffer);

  /* We must have exclusive access to the transfer buffer pool */

  lpc17_takesem(&priv->exclsem);
  lpc17_tbfree(buffer);
  lpc17_givesem(&priv->exclsem);
  return OK;
}

/************************************************************************************
 * Name: lpc17_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kmm_malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are variable-sized.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int lpc17_ioalloc(struct usbhost_driver_s *drvr,
                         uint8_t **buffer, size_t buflen)
{
  DEBUGASSERT(drvr && buffer);

#if LPC17_IOBUFFERS > 0
  if (buflen <= CONFIG_USBHOST_IOBUFSIZE)
    {
      uint8_t *alloc = lpc17_allocio();
      if (alloc)
        {
          *buffer = alloc;
          return OK;
        }
    }

  return -ENOMEM;
#else
  return -ENOSYS;
#endif
}

/************************************************************************************
 * Name: lpc17_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed more
 *   efficiently.  This method provides a mechanism to free that IO buffer
 *   memory.  If the underlying hardware does not support such "special" memory,
 *   this functions may simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int lpc17_iofree(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

#if LPC17_IOBUFFERS > 0
  lpc17_freeio(buffer);
  return OK;
#else
  return -ENOSYS;
#endif
}

/*******************************************************************************
 * Name: lpc17_ctrlin and lpc17_ctrlout
 *
 * Description:
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one transfer may be
 *   queued; Neither these methods nor the transfer() method can be called again
 *   until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep0 - The control endpoint to send/receive the control request.
 *   req - Describes the request to be sent.  This request must lie in memory
 *      created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same allocated
 *   memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int lpc17_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        const struct usb_ctrlreq_s *req,
                        uint8_t *buffer)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  struct lpc17_ed_s *ed = (struct lpc17_ed_s *)ep0;
  uint16_t len;
  int  ret;

  DEBUGASSERT(priv != NULL && ed != NULL && req!= NULL);

  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* We must have exclusive access to EP0 and the control list */

  lpc17_takesem(&priv->exclsem);

  len = lpc17_getle16(req->len);
  ret = lpc17_ctrltd(priv, ed, GTD_STATUS_DP_SETUP, (uint8_t*)req, USB_SIZEOF_CTRLREQ);
  if (ret == OK)
    {
      if (len)
        {
          ret = lpc17_ctrltd(priv, ed, GTD_STATUS_DP_IN, buffer, len);
        }

      if (ret == OK)
        {
          ret = lpc17_ctrltd(priv, ed, GTD_STATUS_DP_OUT, NULL, 0);
        }
    }

  lpc17_givesem(&priv->exclsem);
  return ret;
}

static int lpc17_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                         const struct usb_ctrlreq_s *req,
                         const uint8_t *buffer)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  struct lpc17_ed_s *ed = (struct lpc17_ed_s *)ep0;
  uint16_t len;
  int  ret;

  DEBUGASSERT(priv != NULL && ed != NULL && req!= NULL);

  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* We must have exclusive access to EP0 and the control list */

  lpc17_takesem(&priv->exclsem);

  len = lpc17_getle16(req->len);
  ret = lpc17_ctrltd(priv, ed, GTD_STATUS_DP_SETUP, (uint8_t*)req, USB_SIZEOF_CTRLREQ);
  if (ret == OK)
    {
      if (len)
        {
          ret = lpc17_ctrltd(priv, ed, GTD_STATUS_DP_OUT, (uint8_t*)buffer, len);
        }

      if (ret == OK)
        {
          ret = lpc17_ctrltd(priv, ed, GTD_STATUS_DP_IN, NULL, 0);
        }
    }

  lpc17_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: lpc17_transfer_common
 *
 * Description:
 *   Initiate a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately
 *
 * Input Parameters:
 *   priv - Internal driver state structure.
 *   ed - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int lpc17_transfer_common(struct lpc17_usbhost_s *priv,
                                 struct lpc17_ed_s *ed, uint8_t *buffer,
                                 size_t buflen)
{
  struct lpc17_xfrinfo_s *xfrinfo;
  uint32_t dirpid;
  uint32_t regval;
  bool in;
  int ret;

  xfrinfo = ed->xfrinfo;
  in      = (ed->hw.ctrl & ED_CONTROL_D_MASK) == ED_CONTROL_D_IN;

  uvdbg("EP%u %s toggle:%u maxpacket:%u buflen:%lu\n",
        (ed->hw.ctrl  & ED_CONTROL_EN_MASK) >> ED_CONTROL_EN_SHIFT,
        in ? "IN" : "OUT",
        (ed->hw.headp & ED_HEADP_C) != 0 ? 1 : 0,
        (ed->hw.ctrl  & ED_CONTROL_MPS_MASK) >> ED_CONTROL_MPS_SHIFT,
        (unsigned long)buflen);

  /* Get the direction of the endpoint */

  if (in)
    {
      dirpid = GTD_STATUS_DP_IN;
    }
  else
    {
      dirpid = GTD_STATUS_DP_OUT;
    }

  /* Then enqueue the transfer */

  xfrinfo->tdstatus = TD_CC_NOERROR;
  ret = lpc17_enqueuetd(priv, ed, dirpid, GTD_STATUS_T_TOGGLE, buffer, buflen);
  if (ret == OK)
    {
      /* BulkListFilled. This bit is used to indicate whether there are any
       * TDs on the Bulk list.
       */

      if (ed->xfrtype == USB_EP_ATTR_XFER_BULK)
        {
          regval  = lpc17_getreg(LPC17_USBHOST_CMDST);
          regval |= OHCI_CMDST_BLF;
          lpc17_putreg(regval, LPC17_USBHOST_CMDST);
        }
    }

  return ret;
}

/*******************************************************************************
 * Name: lpc17_dma_alloc
 *
 * Description:
 *   Allocate DMA memory to perform a transfer, copying user data as necessary
 *
 * Input Parameters:
 *   priv - Internal driver state structure.
 *   ed - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   userbuffer - The user buffer containing the data to be sent (OUT endpoint)
 *      or received (IN endpoint).
 *   buflen - The length of the data to be sent or received.
 *   alloc - The location to return the allocated DMA buffer.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

#if LPC17_IOBUFFERS > 0
static int lpc17_dma_alloc(struct lpc17_usbhost_s *priv,
                           struct lpc17_ed_s *ed, uint8_t *userbuffer,
                           size_t buflen, uint8_t **alloc)
{
  uint8_t *newbuffer;

  if ((uintptr_t)userbuffer < LPC17_SRAM_BANK0 ||
      (uintptr_t)userbuffer >= (LPC17_SRAM_BANK0 + LPC17_BANK0_SIZE + LPC17_BANK1_SIZE))
    {
      /* Will the transfer fit in an IO buffer? */

      if (buflen > CONFIG_USBHOST_IOBUFSIZE)
        {
          uvdbg("buflen (%d) > IO buffer size (%d)\n",
                 buflen, CONFIG_USBHOST_IOBUFSIZE);
          return -ENOMEM;
        }

      /* Allocate an IO buffer in AHB SRAM */

      newbuffer = lpc17_allocio();
      if (!newbuffer)
        {
          uvdbg("IO buffer allocation failed\n");
          return -ENOMEM;
        }

      /* If this is an OUT transaction, copy the user data into the AHB
       * SRAM IO buffer.  Sad... so inefficient.  But without exposing
       * the AHB SRAM to the final, end-user client I don't know of any
       * way around this copy.
       */

      if ((ed->hw.ctrl & ED_CONTROL_D_MASK) != ED_CONTROL_D_IN)
        {
          memcpy(newbuffer, userbuffer, buflen);
        }

      /* Return the allocated buffer */

      *alloc = newbuffer;
    }

  return OK;
}

/*******************************************************************************
 * Name: lpc17_dma_free
 *
 * Description:
 *   Free allocated DMA memory.
 *
 * Input Parameters:
 *   priv - Internal driver state structure.
 *   ed - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   userbuffer - The user buffer containing the data to be sent (OUT endpoint)
 *      or received (IN endpoint).
 *   buflen - The length of the data to be sent or received.
 *   alloc - The allocated DMA buffer to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static void lpc17_dma_free(struct lpc17_usbhost_s *priv,
                           struct lpc17_ed_s *ed, uint8_t *userbuffer,
                           size_t buflen, uint8_t *newbuffer)
{
  irqstate_t flags;

  /* Could be called from the interrupt level */

  flags = irqsave();
  if (userbuffer && newbuffer)
    {
      /* If this is an IN transaction, get the user data from the AHB
       * SRAM IO buffer.  Sad... so inefficient.  But without exposing
       * the AHB SRAM to the final, end-user client I don't know of any
       * way around this copy.
       */

      if ((ed->hw.ctrl & ED_CONTROL_D_MASK) == ED_CONTROL_D_IN)
        {
          memcpy(userbuffer, newbuffer, buflen);
        }

      /* Then free the temporary I/O buffer */

      lpc17_freeio(newbuffer);
    }

  irqrestore(flags);
}
#endif

/*******************************************************************************
 * Name: lpc17_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request, blocking until the transfer completes. Only
 *   one transfer may be  queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Values:
 *   On success, a non-negative value is returned that indicates the number
 *   of bytes successfully transferred.  On a failure, a negated errno value is
 *   returned that indicates the nature of the failure:
 *
 *     EAGAIN - If devices NAKs the transfer (or NYET or other error where
 *              it may be appropriate to restart the entire transaction).
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Overrun errors
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static ssize_t lpc17_transfer(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                              uint8_t *buffer, size_t buflen)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  struct lpc17_ed_s *ed = (struct lpc17_ed_s *)ep;
  struct lpc17_xfrinfo_s *xfrinfo;
#if LPC17_IOBUFFERS > 0
  uint8_t *alloc = NULL;
  uint8_t *userbuffer = NULL;
#endif
  ssize_t nbytes;
  int ret;

  DEBUGASSERT(priv && ed && buffer && buflen > 0);

  /* We must have exclusive access to the endpoint, the TD pool, the I/O buffer
   * pool, the bulk and interrupt lists, and the HCCA interrupt table.
   */

  lpc17_takesem(&priv->exclsem);

  /* Allocate a structure to retain the information needed when the transfer
   * completes.
   */

  DEBUGASSERT(ed->xfrinfo == NULL);

  xfrinfo = lpc17_alloc_xfrinfo();
  if (xfrinfo == NULL)
    {
      udbg("ERROR: lpc17_alloc_xfrinfo failed\n");
      nbytes = -ENOMEM;
      goto errout_with_sem;
    }

  /* Initialize the transfer structure */

  memset(xfrinfo, 0, sizeof(struct lpc17_xfrinfo_s));
  xfrinfo->buffer = buffer;
  xfrinfo->buflen = buflen;

  ed->xfrinfo = xfrinfo;

#if LPC17_IOBUFFERS > 0
  /* Allocate an IO buffer if the user buffer does not lie in AHB SRAM */

  ret = lpc17_dma_alloc(priv, ed, buffer, buflen, &alloc);
  if (ret < 0)
    {
      udbg("ERROR: lpc17_dma_alloc failed: %d\n", ret);
      nbytes = (ssize_t)ret;
      goto errout_with_xfrinfo;
    }

  /* If a buffer was allocated, then use it instead of the callers buffer */

  if (alloc)
    {
      userbuffer = buffer;
      buffer  = alloc;
    }
#endif

  /* Set the request for the Writeback Done Head event well BEFORE enabling the
   * transfer.
   */

  ret = lpc17_wdhwait(priv, ed);
  if (ret < 0)
    {
      udbg("ERROR: Device disconnected\n");
      nbytes = (ssize_t)ret;
      goto errout_with_buffers;
    }

  /* Set up the transfer */

  ret = lpc17_transfer_common(priv, ed, buffer, buflen);
  if (ret < 0)
    {
      udbg("ERROR: lpc17_transfer_common failed: %d\n", ret);
      nbytes = (ssize_t)ret;
      goto errout_with_wdhwait;
    }

  /* Wait for the Writeback Done Head interrupt */

  lpc17_takesem(&ed->wdhsem);

  /* Check the TD completion status bits */

  if (xfrinfo->tdstatus == TD_CC_NOERROR)
    {
      /* Return the number of bytes successfully transferred */

      nbytes = xfrinfo->xfrd;
      DEBUGASSERT(nbytes >=0 && nbytes <= buflen);
    }
  else
    {
      /* Map the bad completion status to something that a class driver
       * might understand.
       */

      udbg("ERROR: Bad TD completion status: %d\n", xfrinfo->tdstatus);

      switch (xfrinfo->tdstatus)
        {
        case TD_CC_STALL:
          nbytes = -EPERM;
          break;

        case TD_CC_USER:
          nbytes = -ESHUTDOWN;
          break;

        default:
          nbytes = -EIO;
          break;
        }
     }

errout_with_wdhwait:
  /* Make sure that there is no outstanding request on this endpoint */

  xfrinfo->wdhwait = false;

errout_with_buffers:
#if LPC17_IOBUFFERS > 0
  /* Free any temporary IO buffers */

  lpc17_dma_free(priv, ed, userbuffer, buflen, alloc);
#endif

errout_with_xfrinfo:
  /* Make sure that there is no outstanding request on this endpoint */

  lpc17_free_xfrinfo(xfrinfo);
  ed->xfrinfo = NULL;

errout_with_sem:
  lpc17_givesem(&priv->exclsem);
  return nbytes;
}

/*******************************************************************************
 * Name: lpc17_asynch_completion
 *
 * Description:
 *   This function is called at the interrupt level when an asynchronous
 *   transfer completes.  It performs the pending callback.
 *
 * Input Parameters:
 *   priv - Internal driver state structure.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which the
 *      transfer was performed.
 *
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   - Called from the interrupt level
 *
 *******************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void lpc17_asynch_completion(struct lpc17_usbhost_s *priv,
                                    struct lpc17_ed_s *ed)
{
  struct lpc17_xfrinfo_s *xfrinfo;
  usbhost_asynch_t callback;
  void *arg;
  ssize_t nbytes;

  DEBUGASSERT(ed != NULL && ed->xfrinfo != NULL);
  xfrinfo = ed->xfrinfo;

  DEBUGASSERT(xfrinfo->wdhwait == false &&  xfrinfo->callback != NULL &&
              xfrinfo->buffer != NULL && xfrinfo->buflen > 0);

  /* Check the TD completion status bits */

  if (xfrinfo->tdstatus == TD_CC_NOERROR)
    {
      /* Provide the number of bytes successfully transferred */

      nbytes = xfrinfo->xfrd;
    }
  else
    {
      /* Map the bad completion status to something that a class driver
       * might understand.
       */

      udbg("ERROR: Bad TD completion status: %d\n", xfrinfo->tdstatus);

      switch (xfrinfo->tdstatus)
        {
        case TD_CC_STALL:
          nbytes = -EPERM;
          break;

        case TD_CC_USER:
          nbytes = -ESHUTDOWN;
          break;

        default:
          nbytes = -EIO;
          break;
        }
     }

#if LPC17_IOBUFFERS > 0
  /* Free any temporary IO buffers */

  lpc17_dma_free(priv, ed, xfrinfo->buffer, xfrinfo->buflen, xfrinfo->alloc);
#endif

  /* Extract the callback information before freeing the buffer */

  callback = xfrinfo->callback;
  arg = xfrinfo->arg;

  /* Make sure that there is no outstanding request on this endpoint */

  lpc17_free_xfrinfo(xfrinfo);
  ed->xfrinfo  = NULL;

  /* Then perform the callback */

  callback(arg, nbytes);
}
#endif

/*******************************************************************************
 * Name: lpc17_asynch
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  When the transfer
 *   completes, the the callback will be invoked with the provided transfer.
 *   This method is useful for receiving interrupt transfers which may come
 *   infrequently.
 *
 *   Only one transfer may be queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until the transfer completes.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *   callback - This function will be called when the transfer completes.
 *   arg - The arbitrary parameter that will be passed to the callback function
 *     when the transfer completes.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int lpc17_asynch(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                        uint8_t *buffer, size_t buflen,
                        usbhost_asynch_t callback, void *arg)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  struct lpc17_ed_s *ed = (struct lpc17_ed_s *)ep;
  struct lpc17_xfrinfo_s *xfrinfo;
  int ret;

  DEBUGASSERT(priv && ed && ed->xfrinfo == NULL && buffer && buflen > 0 && callback);

  /* We must have exclusive access to the endpoint, the TD pool, the I/O buffer
   * pool, the bulk and interrupt lists, and the HCCA interrupt table.
   */

  lpc17_takesem(&priv->exclsem);

  /* Allocate a structure to retain the information needed when the asynchronous
   * transfer completes.
   */

  DEBUGASSERT(ed->xfrinfo == NULL);

  xfrinfo = lpc17_alloc_xfrinfo();
  if (xfrinfo == NULL)
    {
      udbg("ERROR: lpc17_alloc_xfrinfo failed\n");
      ret = -ENOMEM;
      goto errout_with_sem;
    }

  /* Initialize the transfer structure */

  memset(xfrinfo, 0, sizeof(struct lpc17_xfrinfo_s));
  xfrinfo->buffer   = buffer;
  xfrinfo->buflen   = buflen;
  xfrinfo->callback = callback;
  xfrinfo->arg      = arg;

  ed->xfrinfo       = xfrinfo;

  #if LPC17_IOBUFFERS > 0
  /* Allocate an IO buffer if the user buffer does not lie in AHB SRAM */

  ret = lpc17_dma_alloc(priv, ed, buffer, buflen, &xfrinfo->alloc);
  if (ret < 0)
    {
      udbg("ERROR: lpc17_dma_alloc failed: %d\n", ret);
      goto errout_with_sem;
    }

  /* If a buffer was allocated, then use it instead of the callers buffer */

  if (xfrinfo->alloc)
    {
      buffer  = xfrinfo->alloc;
    }
#endif

  /* Set up the transfer */

  ret = lpc17_transfer_common(priv, ed, buffer, buflen);
  if (ret < 0)
    {
      udbg("ERROR: lpc17_transfer_common failed: %d\n", ret);
      goto errout_with_asynch;
    }

  /* And return now.  The callback will be invoked when the transfer
   * completes.
   */

  lpc17_givesem(&priv->exclsem);
  return OK;

errout_with_asynch:
#if LPC17_IOBUFFERS > 0
  /* Free any temporary IO buffers */

  lpc17_dma_free(priv, ed, buffer, buflen, xfrinfo->alloc);
#endif

  /* Free the transfer structure */

  lpc17_free_xfrinfo(xfrinfo);
  ed->xfrinfo = NULL;

errout_with_sem:
  lpc17_givesem(&priv->exclsem);
  return ret;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/************************************************************************************
 * Name: lpc17_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Cancelled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which an
 *      asynchronous transfer should be transferred.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ************************************************************************************/

static int lpc17_cancel(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  struct lpc17_ed_s *ed = (struct lpc17_ed_s *)ep;
  struct lpc17_gtd_s *td;
  struct lpc17_gtd_s *next;
  struct lpc17_xfrinfo_s *xfrinfo;
  uint32_t ctrl;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && ed != NULL);

  /* These first steps must be atomic as possible */

  flags = irqsave();

  /* It is possible there there is no transfer to be in progress */

  xfrinfo = ed->xfrinfo;
  if (xfrinfo)
    {
      /* It might be possible for no transfer to be in progress (callback == NULL
       * and wdhwait == false)
       */

#ifdef CONFIG_USBHOST_ASYNCH
      if (xfrinfo->callback || xfrinfo->wdhwait)
#else
      if (xfrinfo->wdhwait)
#endif
        {
          /* Control endpoints should not come through this path and
           * isochronous endpoints are not yet implemented.  So we only have
           * to distinguish bulk and interrupt endpoints.
           */

          if (ed->xfrtype == USB_EP_ATTR_XFER_BULK)
            {
              /* Disable bulk list processing while we modify the list */

              ctrl  = lpc17_getreg(LPC17_USBHOST_CTRL);
              lpc17_putreg(ctrl & ~OHCI_CTRL_BLE, LPC17_USBHOST_CTRL);

              /* Remove the TDs attached to the ED, keeping the ED in the list */

              td           = (struct lpc17_gtd_s *)(ed->hw.headp & ED_HEADP_ADDR_MASK);
              ed->hw.headp = LPC17_TDTAIL_ADDR;
              ed->xfrinfo  = NULL;

              /* Re-enable bulk list processing, if it was enabled before */

              lpc17_putreg(0, LPC17_USBHOST_BULKED);
              lpc17_putreg(ctrl, LPC17_USBHOST_CTRL);
            }
          else
            {
              /* Remove the TDs attached to the ED, keeping the Ed in the list */

              td           = (struct lpc17_gtd_s *)(ed->hw.headp & ED_HEADP_ADDR_MASK);
              ed->hw.headp = LPC17_TDTAIL_ADDR;
              ed->xfrinfo  = NULL;
            }

          /* Free all transfer descriptors that were connected to the ED.  In
           * some race conditions with the hardware, this might be none.
           */

          while (td != (struct lpc17_gtd_s *)LPC17_TDTAIL_ADDR)
            {
              next = (struct lpc17_gtd_s *)td->hw.nexttd;
              lpc17_tdfree(td);
              td = next;
            }

          xfrinfo->tdstatus = TD_CC_USER;

          /* If there is a thread waiting for the transfer to complete, then
           * wake up the thread.
           */

          if (xfrinfo->wdhwait)
            {
#ifdef CONFIG_USBHOST_ASYNCH
              /* Yes.. there should not also be a callback scheduled */

              DEBUGASSERT(xfrinfo->callback == NULL);
#endif

              /* Wake up the waiting thread */

              lpc17_givesem(&ed->wdhsem);
              xfrinfo->wdhwait = false;

               /* And free the transfer structure */

              lpc17_free_xfrinfo(xfrinfo);
              ed->xfrinfo = NULL;
            }
#ifdef CONFIG_USBHOST_ASYNCH
          else
            {
              /* Otherwise, perform the callback and free the transfer structure */

              lpc17_asynch_completion(priv, ed);
            }
#endif
        }
      else
        {
           /* Just free the transfer structure */

          lpc17_free_xfrinfo(xfrinfo);
          ed->xfrinfo = NULL;
        }
    }

  /* Determine the return value */

  irqrestore(flags);
  return OK;
}

/************************************************************************************
 * Name: lpc17_connect
 *
 * Description:
 *   New connections may be detected by an attached hub.  This method is the
 *   mechanism that is used by the hub class to introduce a new connection
 *   and port description to the system.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   hport - The descriptor of the hub port that detected the connection
 *      related event
 *   connected - True: device connected; false: device disconnected
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ************************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int lpc17_connect(FAR struct usbhost_driver_s *drvr,
                         FAR struct usbhost_hubport_s *hport,
                         bool connected)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  DEBUGASSERT(priv != NULL && hport != NULL);
  irqstate_t flags;

  /* Set the connected/disconnected flag */

  hport->connected = connected;
  ullvdbg("Hub port %d connected: %s\n", hport->port, connected ? "YES" : "NO");

  /* Report the connection event */

  flags = irqsave();
  priv->hport = hport;
  if (priv->pscwait)
    {
      priv->pscwait = false;
      lpc17_givesem(&priv->pscsem);
    }

  irqrestore(flags);
  return OK;
}
#endif

/*******************************************************************************
 * Name: lpc17_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and driver has been disconnected.
 *   The USB host driver should discard the handle to the class instance (it is
 *   stale) and not attempt any further interaction with the class driver instance
 *   (until a new instance is received from the create() method).  The driver
 *   should not called the class' disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   hport - The port from which the device is being disconnected.  Might be a port
 *      on a hub.
 *
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static void lpc17_disconnect(struct usbhost_driver_s *drvr,
                             struct usbhost_hubport_s *hport)
{
  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;
}

/*******************************************************************************
 * Initialization
 *******************************************************************************/
/*******************************************************************************
 * Name: lpc17_ep0init
 *
 * Description:
 *   Initialize ED for EP0, add it to the control ED list, and enable control
 *   transfers.
 *
 * Input Parameters:
 *   priv - private driver state instance.
 *
 * Returned Values:
 *   None
 *
 *******************************************************************************/

static inline void lpc17_ep0init(struct lpc17_usbhost_s *priv)
{
  /* Initialize the common tail TD. */

  memset(TDTAIL, 0, sizeof(struct lpc17_gtd_s));
  TDTAIL->ed              = EDCTRL;

  /* Link the common tail TD to the ED's TD list */

  memset(EDCTRL, 0, sizeof(struct lpc17_ed_s));
  EDCTRL->hw.headp = (uint32_t)TDTAIL;
  EDCTRL->hw.tailp = (uint32_t)TDTAIL;
  EDCTRL->xfrtype  = USB_EP_ATTR_XFER_CONTROL;

  /* Set the head of the control list to the NULL (for now). */

  lpc17_putreg(0, LPC17_USBHOST_CTRLHEADED);

  /* Then add EP0 to the empty Control List */

  lpc17_addctrled(priv, EDCTRL);
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc17_usbhost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being initialized.  Normally, this
 *     is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 *******************************************************************************/

struct usbhost_connection_s *lpc17_usbhost_initialize(int controller)
{
  struct lpc17_usbhost_s *priv = &g_usbhost;
  struct usbhost_driver_s *drvr;
  struct usbhost_hubport_s *hport;
  struct lpc17_xfrinfo_s *xfrinfo;
  uint32_t regval;
  uint8_t *buffer;
  irqstate_t flags;
  int i;

  /* Sanity checks.  NOTE: If certain OS features are enabled, it may be
   * necessary to increase the size of LPC17_ED/TD_SIZE in lpc17_ohciram.h
   */

  DEBUGASSERT(controller == 0);
  DEBUGASSERT(sizeof(struct lpc17_ed_s)  <= LPC17_ED_SIZE);
  DEBUGASSERT(sizeof(struct lpc17_gtd_s) <= LPC17_TD_SIZE);

  /* Initialize the state data structure */
  /* Initialize the device operations */

  drvr                 = &priv->drvr;
  drvr->ep0configure   = lpc17_ep0configure;
  drvr->epalloc        = lpc17_epalloc;
  drvr->epfree         = lpc17_epfree;
  drvr->alloc          = lpc17_alloc;
  drvr->free           = lpc17_free;
  drvr->ioalloc        = lpc17_ioalloc;
  drvr->iofree         = lpc17_iofree;
  drvr->ctrlin         = lpc17_ctrlin;
  drvr->ctrlout        = lpc17_ctrlout;
  drvr->transfer       = lpc17_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
  drvr->asynch         = lpc17_asynch;
#endif
  drvr->cancel         = lpc17_cancel;
#ifdef CONFIG_USBHOST_HUB
  drvr->connect        = lpc17_connect;
#endif
  drvr->disconnect     = lpc17_disconnect;

  /* Initialize the public port representation */

  hport                = &priv->rhport.hport;
  hport->drvr          = drvr;
#ifdef CONFIG_USBHOST_HUB
  hport->parent        = NULL;
#endif
  hport->ep0           = EDCTRL;
  hport->speed         = USB_SPEED_FULL;
  hport->funcaddr      = 0;

  /* Initialize function address generation logic */

  usbhost_devaddr_initialize(&priv->rhport);

  /* Initialize semaphores */

  sem_init(&priv->pscsem,  0, 0);
  sem_init(&priv->exclsem, 0, 1);

#ifndef CONFIG_USBHOST_INT_DISABLE
  priv->ininterval  = MAX_PERINTERVAL;
  priv->outinterval = MAX_PERINTERVAL;
#endif

  /* Enable power by setting PCUSB in the PCONP register.  Disable interrupts
   * because this register may be shared with other drivers.
   */

  flags   = irqsave();
  regval  = lpc17_getreg(LPC17_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUSB;
  lpc17_putreg(regval, LPC17_SYSCON_PCONP);
  irqrestore(flags);

  /* Enable clocking on USB (USB PLL clocking was initialized in very low-
   * evel clock setup logic (see lpc17_clockconfig.c)).  We do still need
   * to set up USBOTG CLKCTRL to enable clocking.
   *
   * NOTE: The PORTSEL clock needs to be enabled only when accessing OTGSTCTRL
   */

  lpc17_putreg(LPC17_CLKCTRL_ENABLES, LPC17_USBOTG_CLKCTRL);

  /* Then wait for the clocks to be reported as "ON" */

  do
    {
      regval = lpc17_getreg(LPC17_USBOTG_CLKST);
    }
  while ((regval & LPC17_CLKCTRL_ENABLES) != LPC17_CLKCTRL_ENABLES);

  /* Set the OTG status and control register.  Bits 0:1 apparently mean:
   *
   *   00: U1=device, U2=host
   *   01: U1=host, U2=host
   *   10: reserved
   *   11: U1=host, U2=device
   *
   * We need only select U1=host (Bit 0=1, Bit 1 is not used on LPC176x);
   * NOTE: The PORTSEL clock needs to be enabled when accessing OTGSTCTRL
   */

  lpc17_putreg(1, LPC17_USBOTG_STCTRL);

  /* Now we can turn off the PORTSEL clock */

  lpc17_putreg((LPC17_CLKCTRL_ENABLES & ~USBOTG_CLK_PORTSELCLK), LPC17_USBOTG_CLKCTRL);

  /* Configure I/O pins */

  usbhost_dumpgpio();
  lpc17_configgpio(GPIO_USB_DP);      /* Positive differential data */
  lpc17_configgpio(GPIO_USB_DM);      /* Negative differential data */
  lpc17_configgpio(GPIO_USB_UPLED);   /* GoodLink LED control signal */
  lpc17_configgpio(GPIO_USB_PPWR);    /* Port Power enable signal for USB port */
  lpc17_configgpio(GPIO_USB_PWRD);    /* Power Status for USB port (host power switch) */
  lpc17_configgpio(GPIO_USB_OVRCR);   /* USB port Over-Current status */
  usbhost_dumpgpio();

  udbg("Initializing Host Stack\n");

  /* Show AHB SRAM memory map */

#if 0 /* Useful if you have doubts about the layout */
  uvdbg("AHB SRAM:\n");
  uvdbg("  HCCA:   %08x %d\n", LPC17_HCCA_BASE,   LPC17_HCCA_SIZE);
  uvdbg("  TDTAIL: %08x %d\n", LPC17_TDTAIL_ADDR, LPC17_TD_SIZE);
  uvdbg("  EDCTRL: %08x %d\n", LPC17_EDCTRL_ADDR, LPC17_ED_SIZE);
  uvdbg("  EDFREE: %08x %d\n", LPC17_EDFREE_BASE, LPC17_ED_SIZE);
  uvdbg("  TDFREE: %08x %d\n", LPC17_TDFREE_BASE, LPC17_EDFREE_SIZE);
  uvdbg("  TBFREE: %08x %d\n", LPC17_TBFREE_BASE, LPC17_TBFREE_SIZE);
  uvdbg("  IOFREE: %08x %d\n", LPC17_IOFREE_BASE, LPC17_IOBUFFERS * CONFIG_USBHOST_IOBUFSIZE);
#endif

  /* Initialize all the TDs, EDs and HCCA to 0 */

  memset((void*)HCCA,   0, sizeof(struct ohci_hcca_s));
  memset((void*)TDTAIL, 0, sizeof(struct ohci_gtd_s));
  memset((void*)EDCTRL, 0, sizeof(struct lpc17_ed_s));
  sem_init(&EDCTRL->wdhsem, 0, 0);

  /* Initialize user-configurable EDs */

  buffer = (uint8_t *)LPC17_EDFREE_BASE;
  for (i = 0; i < CONFIG_USBHOST_NEDS; i++)
    {
      /* Put the ED in a free list */

      lpc17_edfree((struct lpc17_ed_s *)buffer);
      buffer += LPC17_ED_SIZE;
    }

  /* Initialize user-configurable TDs */

  buffer = (uint8_t *)LPC17_TDFREE_BASE;
  for (i = 0; i < CONFIG_USBHOST_NTDS; i++)
    {
      /* Put the TD in a free list */

      lpc17_tdfree((struct lpc17_gtd_s *)buffer);
      buffer += LPC17_TD_SIZE;
    }

  /* Initialize user-configurable request/descriptor transfer buffers */

  buffer = (uint8_t *)LPC17_TBFREE_BASE;
  for (i = 0; i < CONFIG_USBHOST_TDBUFFERS; i++)
    {
      /* Put the TD buffer in a free list */

      lpc17_tbfree(buffer);
      buffer += CONFIG_USBHOST_TDBUFSIZE;
    }

#if LPC17_IOBUFFERS > 0
  /* Initialize user-configurable IO buffers */

  buffer = (uint8_t *)LPC17_IOFREE_BASE;
  for (i = 0; i < LPC17_IOBUFFERS; i++)
    {
      /* Put the IO buffer in a free list */

      lpc17_freeio(buffer);
      buffer += CONFIG_USBHOST_IOBUFSIZE;
    }
#endif

  /* Initialize transfer structures */

  for (i = 0, xfrinfo = g_xfrbuffers;
       i < CONFIG_LPC17_USBHOST_NPREALLOC;
       i++, xfrinfo++)
    {
      /* Put the transfer structure in a free list */

      lpc17_free_xfrinfo(xfrinfo);
    }

  /* Wait 50MS then perform hardware reset */

  up_mdelay(50);

  lpc17_putreg(0, LPC17_USBHOST_CTRL);        /* Hardware reset */
  lpc17_putreg(0, LPC17_USBHOST_CTRLHEADED);  /* Initialize control list head to Zero */
  lpc17_putreg(0, LPC17_USBHOST_BULKHEADED);  /* Initialize bulk list head to Zero */

  /* Software reset */

  lpc17_putreg(OHCI_CMDST_HCR, LPC17_USBHOST_CMDST);

  /* Write Fm interval (FI), largest data packet counter (FSMPS), and
   * periodic start.
   */

  lpc17_putreg(DEFAULT_FMINTERVAL, LPC17_USBHOST_FMINT);
  lpc17_putreg(DEFAULT_PERSTART, LPC17_USBHOST_PERSTART);

  /* Put HC in operational state */

  regval  = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_HCFS_MASK;
  regval |= OHCI_CTRL_HCFS_OPER;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);

  /* Set global power in HcRhStatus */

  lpc17_putreg(OHCI_RHSTATUS_SGP, LPC17_USBHOST_RHSTATUS);

  /* Set HCCA base address */

  lpc17_putreg((uint32_t)HCCA, LPC17_USBHOST_HCCA);

  /* Set up the root hub port EP0 */

  lpc17_ep0init(priv);

  /* Clear pending interrupts */

  regval = lpc17_getreg(LPC17_USBHOST_INTST);
  lpc17_putreg(regval, LPC17_USBHOST_INTST);

  /* Enable OHCI interrupts */

  lpc17_putreg((LPC17_ALL_INTS|OHCI_INT_MIE), LPC17_USBHOST_INTEN);

  /* Attach USB host controller interrupt handler */

  if (irq_attach(LPC17_IRQ_USB, lpc17_usbinterrupt) != 0)
    {
      udbg("Failed to attach IRQ\n");
      return NULL;
    }

  /* Enable USB interrupts at the SYCON controller.  Disable interrupts
   * because this register may be shared with other drivers.
   */

  flags   = irqsave();
  regval  = lpc17_getreg(LPC17_SYSCON_USBINTST);
  regval |= SYSCON_USBINTST_ENINTS;
  lpc17_putreg(regval, LPC17_SYSCON_USBINTST);
  irqrestore(flags);

  /* If there is a USB device in the slot at power up, then we will not
   * get the status change interrupt to signal us that the device is
   * connected.  We need to set the initial connected state accordingly.
   */

  regval          = lpc17_getreg(LPC17_USBHOST_RHPORTST1);
  priv->connected = ((regval & OHCI_RHPORTST_CCS) != 0);

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(LPC17_IRQ_USB); /* enable USB interrupt */
  udbg("USB host Initialized, Device connected:%s\n",
       priv->connected ? "YES" : "NO");

  return &g_usbconn;
}
