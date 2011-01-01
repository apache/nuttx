/*******************************************************************************
 * arch/arm/src/lpc17xx/lpc17_usbhost.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Authors: Rafael Noronha <rafael@pdsolucoes.com.br>
 *            Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/ohci.h>
#include <nuttx/usb/usbhost.h>

#include <arch/irq.h>

#include "lpc17_internal.h"   /* Includes default GPIO settings */
#include <arch/board/board.h> /* May redefine GPIO settings */

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#include "lpc17_usb.h"
#include "lpc17_syscon.h"
#include "lpc17_ohciram.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/* I think it is the case that all I/O buffers must lie in AHB SRAM because of
 * the OHCI DMA.  But this definition has here so that I can experiment later
 * to see if this really required.
 */

#define CONFIG_UBHOST_AHBIOBUFFERS 1

#if defined(CONFIG_UBHOST_AHBIOBUFFERS) && LPC17_IOBUFFERS < 1
#  error "No IO buffers allocated"
#endif

/* Frame Interval / Periodic Start */

#define  FI                     (12000-1) /* 12000 bits per frame (-1) */
#define  FSMPS                  ((6 * (FI - 210)) / 7)
#define  DEFAULT_FMINTERVAL     ((FSMPS << OHCI_FMINT_FSMPS_SHIFT) | FI)
#define  DEFAULT_PERSTART       ((9 * FI) / 10)

/* CLKCTRL enable bits */

#define LPC17_CLKCTRL_ENABLES (USBOTG_CLK_HOSTCLK|USBOTG_CLK_PORTSELCLK|USBOTG_CLK_AHBCLK)

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

#define HCCA        ((volatile struct ohci_hcca_s *)LPC17_HCCA_BASE)
#define TDHEAD      ((volatile struct ohci_gtd_s *)LPC17_TDHEAD_ADDR)
#define TDTAIL      ((volatile struct ohci_gtd_s *)LPC17_TDTAIL_ADDR)
#define EDCTRL      ((volatile struct ohci_ed_s *)LPC17_EDCTRL_ADDR)

#define EDFREE      ((struct ohci_ed_s *)LPC17_EDFREE_BASE)
#define TDFREE      ((uint8_t *)LPC17_TDFREE_BASE)
#define IOFREE      ((uint8_t *)LPC17_IOFREE_BASE)

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

  /* The bound device class driver */

  struct usbhost_class_s *class;

  /* Driver status */

  volatile uint8_t tdstatus;  /* TD control status bits from last Writeback Done Head event */
  volatile bool    connected; /* Connected to device */
  sem_t            rhssem;    /* Semaphore  to wait Root Hub Status change */
  sem_t            wdhsem;    /* Semaphore  used to wait for Writeback Done Head event */
};

/* The following are used to manage lists of free EDs and TD buffers*/

struct lpc17_edlist_s
{
  struct lpc17_edlist_s *flink;        /* Link to next ED in the list */
  uint32_t               pad[3];       /* To make the same size as struct ohci_ed_s */
};

struct lpc17_buflist_s
{
  struct lpc17_buflist_s *flink;       /* Link to next buffer in the list */
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
static void lpc17_putle16(uint8_t *dest, uint16_t val);

/* Descriptor helper functions *************************************************/

static struct ohci_ed_s *lpc17_edalloc(struct lpc17_usbhost_s *priv);
static void lpc17_edfree(struct lpc17_usbhost_s *priv, struct ohci_ed_s *ed);
static uint8_t *lpc17_tdalloc(struct lpc17_usbhost_s *priv);
static void lpc17_tdfree(struct lpc17_usbhost_s *priv, uint8_t *buffer);
#ifdef CONFIG_UBHOST_AHBIOBUFFERS
static uint8_t *lpc17_ioalloc(struct lpc17_usbhost_s *priv);
static void lpc17_iofree(struct lpc17_usbhost_s *priv, uint8_t *buffer);
#endif
static void lpc17_enqueuetd(volatile struct ohci_ed_s *ed, uint32_t dirpid,
                            uint32_t toggle, volatile uint8_t *buffer,
                            size_t buflen);
static int lpc17_ctrltd(struct lpc17_usbhost_s *priv, uint32_t dirpid,
                        uint8_t *buffer, size_t buflen);

/* Interrupt handling **********************************************************/

static int lpc17_usbinterrupt(int irq, FAR void *context);

/* USB host controller operations **********************************************/

static int lpc17_wait(FAR struct usbhost_driver_s *drvr, bool connected);
static int lpc17_enumerate(FAR struct usbhost_driver_s *drvr);
static int lpc17_ep0configure(FAR struct usbhost_driver_s *drvr, uint8_t funcaddr,
                              uint16_t maxpacketsize);
static int lpc17_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen);
static int lpc17_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int lpc17_ctrlin(FAR struct usbhost_driver_s *drvr,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer);
static int lpc17_ctrlout(FAR struct usbhost_driver_s *drvr,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer);
static int lpc17_transfer(FAR struct usbhost_driver_s *drvr,
                          FAR struct usbhost_epdesc_s *ed,
                          FAR uint8_t *buffer, size_t buflen);
static void lpc17_disconnect(FAR struct usbhost_driver_s *drvr);
  
/* Initializaion ***************************************************************/

static void lpc17_tdinit(volatile struct ohci_gtd_s *td);
static void lpc17_edinit(volatile struct ohci_ed_s *ed);
static void lpc17_hccainit(volatile struct ohci_hcca_s *hcca);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* In this driver implementation, support is provided for only a single a single
 * USB device.  All status information can be simply retained in a single global
 * instance.
 */

static struct lpc17_usbhost_s g_usbhost =
{
  .drvr             =
    {
      .wait         = lpc17_wait,
      .enumerate    = lpc17_enumerate,
      .ep0configure = lpc17_ep0configure,
      .alloc        = lpc17_alloc,
      .free         = lpc17_free,
      .ctrlin       = lpc17_ctrlin,
      .ctrlout      = lpc17_ctrlout,
      .transfer     = lpc17_transfer,
      .disconnect   = lpc17_disconnect,
    },
  .class            = NULL,
};

/* This is a free list of EDs and TD buffers */

static struct lpc17_edlist_s  *g_edfree;
static struct lpc17_buflist_s *g_tdfree;
#ifdef CONFIG_UBHOST_AHBIOBUFFERS
static struct lpc17_buflist_s *g_iofree;
#endif

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

static void lpc17_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/*******************************************************************************
 * Name: lpc17_edalloc
 *
 * Description:
 *   Allocate an ED from the free list
 *
 *******************************************************************************/

static struct ohci_ed_s *lpc17_edalloc(struct lpc17_usbhost_s *priv)
{
  struct ohci_ed_s *ret = (struct ohci_ed_s *)g_edfree;
  if (ret)
    {
      g_edfree = ((struct lpc17_edlist_s*)ret)->flink;
    }
  return ret;
}

/*******************************************************************************
 * Name: lpc17_edfree
 *
 * Description:
 *   Return an ED to the free list
 *
 *******************************************************************************/

static void lpc17_edfree(struct lpc17_usbhost_s *priv, struct ohci_ed_s *ed)
{
  struct lpc17_edlist_s *edfree = (struct lpc17_edlist_s *)ed;
  edfree->flink                 = g_edfree;
  g_edfree                      = edfree;
}

/*******************************************************************************
 * Name: lpc17_tdalloc
 *
 * Description:
 *   Allocate an TD buffer from the free list
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static uint8_t *lpc17_tdalloc(struct lpc17_usbhost_s *priv)
{
  uint8_t *ret = (uint8_t *)g_tdfree;
  if (ret)
    {
      g_tdfree = ((struct lpc17_buflist_s*)ret)->flink;
    }
  return ret;
}

/*******************************************************************************
 * Name: lpc17_tdfree
 *
 * Description:
 *   Return an TD buffer to the free list
 *
 *******************************************************************************/

static void lpc17_tdfree(struct lpc17_usbhost_s *priv, uint8_t *buffer)
{
  struct lpc17_buflist_s *tdfree = (struct lpc17_buflist_s *)buffer;

  if (tdfree)
    {
      tdfree->flink              = g_tdfree;
      g_tdfree                   = tdfree;
    }
}

/*******************************************************************************
 * Name: lpc17_ioalloc
 *
 * Description:
 *   Allocate an IO buffer from the free list
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

#ifdef CONFIG_UBHOST_AHBIOBUFFERS
static uint8_t *lpc17_ioalloc(struct lpc17_usbhost_s *priv)
{
  uint8_t *ret = (uint8_t *)g_iofree;
  if (ret)
    {
      g_iofree = ((struct lpc17_buflist_s*)ret)->flink;
    }
  return ret;
}
#endif

/*******************************************************************************
 * Name: lpc17_iofree
 *
 * Description:
 *   Return an TD buffer to the free list
 *
 *******************************************************************************/

#ifdef CONFIG_UBHOST_AHBIOBUFFERS
static void lpc17_iofree(struct lpc17_usbhost_s *priv, uint8_t *buffer)
{
  struct lpc17_buflist_s *iofree = (struct lpc17_buflist_s *)buffer;
  iofree->flink                  = g_iofree;
  g_iofree                       = iofree;
}
#endif

/*******************************************************************************
 * Name: lpc17_enqueuetd
 *
 * Description:
 *   Enqueue a transfer descriptor
 *
 *******************************************************************************/

static void lpc17_enqueuetd(volatile struct ohci_ed_s *ed, uint32_t dirpid,
                            uint32_t toggle, volatile uint8_t *buffer, size_t buflen)
{
  TDHEAD->ctrl    = (GTD_STATUS_R | dirpid | TD_DELAY(0) | toggle | GTD_STATUS_CC_MASK);
  TDTAIL->ctrl    = 0;
  TDHEAD->cbp     = (uint32_t)buffer;
  TDTAIL->cbp     = 0;
  TDHEAD->nexttd  = (uint32_t)TDTAIL;
  TDTAIL->nexttd  = 0;
  TDHEAD->be      = (uint32_t)(buffer + (buflen - 1));
  TDTAIL->be      = 0;

  ed->headp       = (uint32_t)TDHEAD | ((ed->headp) & ED_HEADP_C);
  ed->tailp       = (uint32_t)TDTAIL;
  ed->nexted      = 0;
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

static int lpc17_ctrltd(struct lpc17_usbhost_s *priv, uint32_t dirpid,
                        uint8_t *buffer, size_t buflen)
{
  uint32_t toggle;
  uint32_t regval;

  if (dirpid == GTD_STATUS_DP_SETUP)
    {
      toggle = GTD_STATUS_T_DATA0;
    }
  else
    {
      toggle = GTD_STATUS_T_DATA1;
    }

  /* Then enqueue the transfer */

  priv->tdstatus = 0;
  lpc17_enqueuetd(EDCTRL, dirpid, toggle, buffer, buflen);

  /* Set the head of the control list to the EP0 EDCTRL (this would have to
   * change if we want more than on control EP queued at a time).
   */

  lpc17_putreg(LPC17_EDCTRL_ADDR, LPC17_USBHOST_CTRLHEADED);

  /* Set ControlListFilled.  This bit is used to indicate whether there are
   * TDs on the Control list.
   */

  regval = lpc17_getreg(LPC17_USBHOST_CMDST);
  regval |= OHCI_CMDST_CLF;
  lpc17_putreg(regval, LPC17_USBHOST_CMDST);

  /* ControlListEnable.  This bit is set to enable the processing of the
   * Control list.  Note: once enabled, it remains enabled and we may even
   * complete list processing before we get the bit set.  We really
   * should never modify the control list while CLE is set.
   */

  regval = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval |= OHCI_CTRL_CLE;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);

  /* Wait for the Writeback Done Head interrupt */

  lpc17_takesem(&priv->wdhsem);

  /* Check the TDHEAD completion status bits */

  if (priv->tdstatus == 0)
    {
      return OK;
    }
  else 
    {
      uvdbg("Bad TD completion status: %d\n", priv->tdstatus);
      return -EIO;
    }
}

/*******************************************************************************
 * Name: lpc17_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 *******************************************************************************/

static int lpc17_usbinterrupt(int irq, FAR void *context)
{
  struct lpc17_usbhost_s *priv = &g_usbhost;
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
                  /* Check if we are now connected */

                  if ((rhportst1 & OHCI_RHPORTST_CCS) != 0)
                    {
                      if (!priv->connected)
                        {
                          /* Yes.. connected. */

                          ullvdbg("Connected\n");
                          priv->tdstatus = 0;

                          /* Notify any waiters */

                          priv->connected = true;
                          lpc17_givesem(&priv->rhssem);
                        }
                      else
                        {
                          ulldbg("Spurious status change (connected)\n");
                        }
                    }

                  /* Check if we are now disconnected */
 
                  else if (priv->connected)
                    {
                      /* Yes.. disable interrupts and disconnect the device */

                      ullvdbg("Disconnected\n");
                      priv->connected = false;

                      /* Are we bound to a class instance? */

                      if (priv->class)
                        {
                          /* Yes.. Disconnect the class */

                          CLASS_DISCONNECTED(priv->class);
                        }

                      /* Notify any waiters */

                      lpc17_givesem(&priv->rhssem);
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
          /* The host controller just wrote the list of finished TDs into the HCCA
           * done head.  Here we assume that only a single packet is "in flight"
           * at any given time and we use only the TD at TDHEAD.  So not much needs
           * to be done here.  In a more complex implementation we would need to
           * recover these completed TDs in some meaningful way.
           */

          /* Since there is only one TD, we can disable further TD processing. */

          regval = lpc17_getreg(LPC17_USBHOST_CTRL);
          regval &= ~(OHCI_CTRL_PLE|OHCI_CTRL_IE|OHCI_CTRL_CLE|OHCI_CTRL_BLE);
          lpc17_putreg(regval, LPC17_USBHOST_CTRL);

          /* Get the condition code from the (single) TDHEAD TD status/control word */

          priv->tdstatus = (TDHEAD->ctrl & GTD_STATUS_CC_MASK) >> GTD_STATUS_CC_SHIFT;

#ifdef CONFIG_DEBUG_USB
          if (priv->tdstatus != 0)
            {
              /* The transfer failed for some reason... dump some diagnostic info. */

              ulldbg("ERROR: TD CTRL:%08x/CC:%d RHPORTST1:%08x\n",
                     TDHEAD->ctrl, priv->tdstatus,
                     lpc17_getreg(LPC17_USBHOST_RHPORTST1));
           }
#endif

          /* And wake up the thread waiting for the WDH event */

          DEBUGASSERT(priv->wdhsem.semcount <= 0);
          lpc17_givesem(&priv->wdhsem);
        }

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
 *   Wait for a device to be connected or disconneced.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   connected - TRUE: Wait for device to be connected; FALSE: wait for device
 *      to be disconnected
 *
 * Returned Values:
 *   Zero (OK) is returned when a device in connected. This function will not
 *   return until either (1) a device is connected or (2) some failure occurs.
 *   On a failure, a negated errno value is returned indicating the nature of
 *   the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int lpc17_wait(FAR struct usbhost_driver_s *drvr, bool connected)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;

  /* Are we already connected? */

  while (priv->connected == connected)
    {
      /* No... wait for the connection/disconnection */

      lpc17_takesem(&priv->rhssem);
    }

  return OK;
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
 *   to get a class instance, and finally (5) call the configdesc() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int lpc17_enumerate(FAR struct usbhost_driver_s *drvr)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;

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

  up_mdelay(100);

  /* Put RH port 1 in reset (the LPC176x supports only a single downstream port) */

  lpc17_putreg(OHCI_RHPORTST_PRS, LPC17_USBHOST_RHPORTST1);

  /* Wait for the port reset to complete */

  while ((lpc17_getreg(LPC17_USBHOST_RHPORTST1) & OHCI_RHPORTST_PRS) != 0);

  /* Release RH port 1 from reset and wait a bit */

  lpc17_putreg(OHCI_RHPORTST_PRSC, LPC17_USBHOST_RHPORTST1);
  up_mdelay(200);

  /* Let the common usbhost_enumerate do all of the real work */

  uvdbg("Enumerate the device\n");
  return usbhost_enumerate(drvr, &priv->class);
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
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *   maxpacketsize - The maximum number of bytes that can be sent to or
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

static int lpc17_ep0configure(FAR struct usbhost_driver_s *drvr, uint8_t funcaddr,
                              uint16_t maxpacketsize)
{
  DEBUGASSERT(drvr && funcaddr < 128 && maxpacketsize < 2048);
  EDCTRL->ctrl = (uint32_t)funcaddr << ED_CONTROL_FA_SHIFT | 
                 (uint32_t)maxpacketsize << ED_CONTROL_MPS_SHIFT;
  return OK;
}

/*******************************************************************************
 * Name: lpc17_alloc
 *
 * Description:
 *   Some hardware supports special memory in which transfer descriptors can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the transfer descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to malloc.
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

static int lpc17_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  DEBUGASSERT(priv && buffer && maxlen);

  *buffer = lpc17_tdalloc(priv);
  if (*buffer)
    {
      *maxlen = CONFIG_USBHOST_TDBUFSIZE;
      return OK;
    }
  return -ENOMEM;
}

/*******************************************************************************
 * Name: lpc17_free
 *
 * Description:
 *   Some hardware supports special memory in which transfer descriptors can
 *   be accessed more efficiently.  This method provides a mechanism to free that
 *   transfer descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to free().
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
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int lpc17_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  DEBUGASSERT(priv && buffer);
  lpc17_tdfree(priv, buffer);
  return OK;
}

/*******************************************************************************
 * Name: lpc17_ctrlin and lpc17_ctrlout
 *
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
 *   req - Describes the request to be sent.  This request must lie in memory
 *      created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using DRVR_ALLOC
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same allocated
 *   memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int lpc17_ctrlin(FAR struct usbhost_driver_s *drvr,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  uint16_t len;
  int  ret;

  DEBUGASSERT(drvr && req);
  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  len = lpc17_getle16(req->len);
  ret = lpc17_ctrltd(priv, GTD_STATUS_DP_SETUP, (uint8_t*)req, USB_SIZEOF_CTRLREQ);
  if (ret == OK)
    {
      if (len)
        {
          ret = lpc17_ctrltd(priv, GTD_STATUS_DP_IN, buffer, len);
        }

      if (ret == OK)
        {
          ret = lpc17_ctrltd(priv, GTD_STATUS_DP_OUT, NULL, 0);
        }
    }
  return ret;
}

static int lpc17_ctrlout(FAR struct usbhost_driver_s *drvr,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  uint16_t len;
  int  ret;

  DEBUGASSERT(drvr && req);
  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  len = lpc17_getle16(req->len);
  ret = lpc17_ctrltd(priv, GTD_STATUS_DP_SETUP, (uint8_t*)req, USB_SIZEOF_CTRLREQ);
  if (ret == OK)
    {
      if (len)
        {
          ret = lpc17_ctrltd(priv, GTD_STATUS_DP_OUT, (uint8_t*)buffer, len);
        }

      if (ret == OK)
        {
          ret = lpc17_ctrltd(priv, GTD_STATUS_DP_IN, NULL, 0);
        }
    }
  return ret;
}

/*******************************************************************************
 * Name: lpc17_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  Only one transfer may be
 *   queued; Neither this method nor the ctrlin or ctrlout methods can be called
 *   again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ed - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int lpc17_transfer(FAR struct usbhost_driver_s *drvr,
                          FAR struct usbhost_epdesc_s *ep,
                          FAR uint8_t *buffer, size_t buflen)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  struct ohci_ed_s *ed = NULL;
  uint32_t dirpid;
  uint32_t regval;
#ifdef CONFIG_UBHOST_AHBIOBUFFERS
  uint8_t *origbuf = NULL;
#endif
  int ret = -ENOMEM;

  DEBUGASSERT(drvr && ep && buffer && buflen > 0);
  uvdbg("EP%d %s maxpacket:%d buflen:%d\n",
        ep->addr, ep->in ? "IN" : "OUT", ep->mxpacketsize, buflen);

  /* Allocate an IO buffer if the user buffer does not lie in AHB SRAM */

#ifdef CONFIG_UBHOST_AHBIOBUFFERS
  if ((uintptr_t)buffer < LPC17_SRAM_BANK0 ||
      (uintptr_t)buffer >= (LPC17_SRAM_BANK0 + LPC17_BANK0_SIZE + LPC17_BANK1_SIZE))
    {
      /* Will the transfer fit in an IO buffer? */

      if (buflen > CONFIG_USBHOST_IOBUFSIZE)
        {
          uvdbg("buflen (%d) > IO buffer size (%d)\n",
                 buflen, CONFIG_USBHOST_IOBUFSIZE);
          goto errout;
        }

      /* Allocate an IO buffer in AHB SRAM */

      origbuf = buffer;
      buffer  = lpc17_ioalloc(priv);
      if (!buffer)
        {
          uvdbg("IO buffer allocation failed\n");
          goto errout;
        }

      /* Copy the user data into the AHB SRAM IO buffer.  Sad... so
       * inefficient.  But without exposing the AHB SRAM to the final,
       * end-user client I don't know of any way around this copy.
       */

      memcpy(buffer, origbuf, buflen);
    }
#endif

  /* Allocate an ED */

  ed = lpc17_edalloc(priv);
  if (!ed)
    {
      udbg("ED allocation failed\n");
      goto errout;
    }

  /* Format the endpoint descriptor */
 
  lpc17_edinit(ed);
  ed->ctrl = (uint32_t)(ep->funcaddr)     << ED_CONTROL_FA_SHIFT | 
             (uint32_t)(ep->addr)         << ED_CONTROL_EN_SHIFT |
             (uint32_t)(ep->mxpacketsize) << ED_CONTROL_MPS_SHIFT;

  /* Get the direction of the endpoint */

  if (ep->in)
    {
      ed->ctrl |= ED_CONTROL_D_IN;
      dirpid    = GTD_STATUS_DP_IN;
    }
  else
    {
      ed->ctrl |= ED_CONTROL_D_OUT;
      dirpid    = GTD_STATUS_DP_OUT;
    }

  /* Then enqueue the transfer */

  priv->tdstatus = 0;
  lpc17_enqueuetd(ed, dirpid, GTD_STATUS_T_TOGGLE, buffer, buflen);

  /* Set the head of the bulk list to the EP descriptor (this would have to
   * change if we want more than on bulk EP queued at a time).
   */

  lpc17_putreg((uint32_t)ed, LPC17_USBHOST_BULKHEADED);

  /* BulkListFilled. This bit is used to indicate whether there are any
   * TDs on the Bulk list.
   */
 
  regval  = lpc17_getreg(LPC17_USBHOST_CMDST);
  regval |= OHCI_CMDST_BLF;
  lpc17_putreg(regval, LPC17_USBHOST_CMDST);

  /* BulkListEnable. This bit is set to enable the processing of the Bulk
   * list.  Note: once enabled, it remains enabled and we may even
   * complete list processing before we get the bit set.  We really
   * should never modify the bulk list while BLE is set.
   */

  regval  = lpc17_getreg(LPC17_USBHOST_CTRL);
  regval |= OHCI_CTRL_BLE;
  lpc17_putreg(regval, LPC17_USBHOST_CTRL);

  /* Wait for the Writeback Done Head interrupt */

  lpc17_takesem(&priv->wdhsem);

  /* Check the TDHEAD completion status bits */

  if (priv->tdstatus == 0)
    {
      ret = OK;
    }
  else 
    {
      uvdbg("Bad TD completion status: %d\n", priv->tdstatus);
      ret = -EIO;
    }

errout:
  /* Free any temporary IO buffers */

#ifdef CONFIG_UBHOST_AHBIOBUFFERS
  if (buffer && origbuf)
    {
      lpc17_iofree(priv, buffer);
    }
#endif

  /* Free the endpoint descriptor */

  if (ed)
    {
      lpc17_edfree(priv, ed);
    }

  return ret;
}

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
 *
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static void lpc17_disconnect(FAR struct usbhost_driver_s *drvr)
{
  struct lpc17_usbhost_s *priv = (struct lpc17_usbhost_s *)drvr;
  priv->class = NULL;
}
  
/*******************************************************************************
 * Initialization
 *******************************************************************************/

static void lpc17_tdinit(volatile struct ohci_gtd_s *td)
{
  td->ctrl    = 0;
  td->cbp     = 0;
  td->nexttd  = 0;
  td->be      = 0;
}

static void lpc17_edinit(volatile struct ohci_ed_s *ed)
{
  ed->ctrl    = 0;
  ed->tailp   = 0;
  ed->headp   = 0;
  ed->nexted  = 0;
}

static void lpc17_hccainit(volatile struct ohci_hcca_s *hcca)
{
  int  i;

  for (i = 0; i < 32; i++)
    {
      hcca->inttbl[i] = 0;
    }

  hcca->fmno      = 0;
  hcca->donehead  = 0;
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: usbhost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being intialized.  Normally, this
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

FAR struct usbhost_driver_s *usbhost_initialize(int controller)
{
  struct lpc17_usbhost_s *priv = &g_usbhost;
  uint32_t regval;
  uint8_t *buffer;
  irqstate_t flags;
  int i;

  DEBUGASSERT(controller == 0);

  /* Initialize the state data structure */

  sem_init(&priv->rhssem, 0, 0);
  sem_init(&priv->wdhsem, 0, 0);

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

  /* Initialize all the TDs, EDs and HCCA to 0 */

  lpc17_hccainit(HCCA);
  lpc17_tdinit(TDHEAD);
  lpc17_tdinit(TDTAIL);
  lpc17_edinit(EDCTRL);

  /* Initialize user-configurable EDs */

  for (i = 0; i < CONFIG_USBHOST_NEDS; i++)
    {
      /* Put the ED in a free list */

      lpc17_edfree(priv, &EDFREE[i]);
    }

  /* Initialize user-configurable TD buffers */

  buffer = TDFREE;
  for (i = 0; i < CONFIG_USBHOST_NEDS; i++)
    {
      /* Put the TD buffer in a free list */

      lpc17_tdfree(priv, buffer);
      buffer += CONFIG_USBHOST_TDBUFSIZE;
    }

#ifdef CONFIG_UBHOST_AHBIOBUFFERS
  /* Initialize user-configurable IO buffers */

  buffer = IOFREE;
  for (i = 0; i < LPC17_IOBUFFERS; i++)
    {
      /* Put the IO buffer in a free list */

      lpc17_iofree(priv, buffer);
      buffer += CONFIG_USBHOST_IOBUFSIZE;
    }
#endif

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

  /* Clear pending interrupts */

  regval = lpc17_getreg(LPC17_USBHOST_INTST);
  lpc17_putreg(regval, LPC17_USBHOST_INTST);

  /* Enable OHCI interrupts */
 
  lpc17_putreg((OHCI_INT_MIE | OHCI_INT_WDH | OHCI_INT_RHSC), LPC17_USBHOST_INTEN);

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

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(LPC17_IRQ_USB); /* enable USB interrupt */
  udbg("USB host Initialized\n");

  /* If there is a USB device in the slot at power up, then we will not
   * get the status change interrupt to signal us that the device is
   * connected.  We need to set the initial connected state accordingly.
   */

  regval          = lpc17_getreg(LPC17_USBHOST_RHPORTST1);
  priv->connected = ((regval & OHCI_RHPORTST_CCS) != 0);

  return &priv->drvr;
}
