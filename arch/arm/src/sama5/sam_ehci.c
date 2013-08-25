/*******************************************************************************
 * arch/arm/src/sama5/sam_ehci.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/ehci.h>

#include "up_arch.h"
#include "cache.h"

#include "sam_periphclks.h"
#include "sam_memories.h"
#include "sam_usbhost.h"
#include "chip/sam_sfr.h"
#include "chip/sam_ehci.h"

#ifdef CONFIG_SAMA5_EHCI

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************/
/* Configuration ***************************************************************/
/* Pre-requisites */

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

/* Configurable number of Queue Head (QH) structures.  The default is one per
 * Root hub port plus one for EP0.
 */

#ifndef CONFIG_SAMA5_EHCI_NQHS
#  define CONFIG_SAMA5_EHCI_NQHS (SAM_EHCI_NRHPORT + 1)
#endif

/* Configurable number of Queue Element Transfer Descriptor (qTDs).  The default
 * is one per root hub plus three from EP0.
 */

#ifndef CONFIG_SAMA5_EHCI_NQTDS
#  define CONFIG_SAMA5_EHCI_NQTDS (SAM_EHCI_NRHPORT + 3)
#endif

/* Configurable size of a request/descriptor buffers */

#ifndef CONFIG_SAMA5_EHCI_BUFSIZE
#  define CONFIG_SAMA5_EHCI_BUFSIZE 128
#endif

/* Debug options */

#ifndef CONFIG_DEBUG
#  undef CONFIG_SAMA5_EHCI_REGDEBUG
#endif

/* Periodic transfers will be supported later */

#undef CONFIG_USBHOST_INT_DISABLE
#define CONFIG_USBHOST_INT_DISABLE 1

#undef CONFIG_USBHOST_ISOC_DISABLE
#define CONFIG_USBHOST_ISOC_DISABLE 1

/* If UDPHS is enabled, then don't use port A */

#ifdef CONFIG_SAMA5_UDPHS
#  undef CONFIG_SAMA5_UHPHS_RHPORT1
#endif

/* For now, suppress use of PORTA in any event.  I use that for SAM-BA and
 * would prefer that the board not try to drive VBUS on that port!
 */

#undef CONFIG_SAMA5_UHPHS_RHPORT1

/* Driver-private Definitions **************************************************/

/* This is the set of interrupts handled by this driver */

#define EHCI_HANDLED_INTS (EHCI_INT_USBINT | EHCI_INT_USBERRINT | \
                           EHCI_INT_PORTSC |  EHCI_INT_SYSERROR | \
                           EHCI_INT_AAINT)

/* The periodic frame list is a 4K-page aligned array of Frame List Link
 * pointers. The length of the frame list may be programmable. The programmability
 * of the periodic frame list is exported to system software via the HCCPARAMS
 * register. If non-programmable, the length is 1024 elements. If programmable,
 * the length can be selected by system software as one of 256, 512, or 1024
 * elements.
 */

#define FRAME_LIST_SIZE 1024

/*******************************************************************************
 * Private Types
 *******************************************************************************/
/* Internal representation of the EHCI Queue Head (QH) */

struct sam_epinfo_s;
struct sam_qh_s
{
  /* Fields visible to hardware */

  struct ehci_qh_s hw;         /* Hardware representation of the queue head */

  /* Internal fields used by the EHCI driver */

  struct sam_epinfo_s *epinfo; /* Endpoint used for the transfer */
  uint32_t fqp;                /* First qTD in the list (physical address) */
  uint8_t pad[8];              /* Padding to assure 32-byte alignment */
};

/* Internal representation of the EHCI Queue Element Transfer Descriptor (qTD) */

struct sam_qtd_s
{
  /* Fields visible to hardware */

  struct ehci_qtd_s hw;        /* Hardware representation of the queue head */

  /* Internal fields used by the EHCI driver */
};

/* The following is used to manage lists of free QHs and qTDs */

struct sam_list_s
{
  struct sam_list_s *flink;    /* Link to next entry in the list */
                               /* Variable length entry data follows */
};

/* List traversal callout functions */

typedef int (*foreach_qh_t)(struct sam_qh_s *qh, uint32_t **bp, void *arg);
typedef int (*foreach_qtd_t)(struct sam_qtd_s *qtd, uint32_t **bp, void *arg);

/* This structure describes one endpoint. */

struct sam_epinfo_s
{
  uint8_t epno:7;              /* Endpoint number */
  uint8_t dirin:1;             /* 1:IN endpoint 0:OUT endpoint */
  uint8_t devaddr:7;           /* Device address */
  uint8_t toggle:1;            /* Next data toggle */
#ifndef CONFIG_USBHOST_INT_DISABLE
  uint8_t interval;            /* Polling interval */
#endif
  uint8_t status;              /* Retained token status bits (for debug purposes) */
  volatile bool iocwait;       /* TRUE: Thread is waiting for transfer completion */
  uint16_t maxpacket:11;       /* Maximum packet size */
  uint16_t xfrtype:2;          /* See USB_EP_ATTR_XFER_* definitions in usb.h */
  uint16_t speed:2;            /* See USB_*_SPEED definitions in ehci.h */
  int result;                  /* The result of the transfer */
  uint32_t xfrd;               /* On completion, will hold the number of bytes transferred */
  sem_t iocsem;                /* Semaphore used to wait for transfer completion */
};

/* This structure retains the state of one root hub port */

struct sam_rhport_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to struct sam_rhport_s.
   */

  struct usbhost_driver_s drvr;

  /* Root hub port status */

  volatile bool connected;     /* Connected to device */
  volatile bool lowspeed;      /* Low speed device attached */
  uint8_t rhpndx;              /* Root hub port index */
  struct sam_epinfo_s ep0;     /* EP0 endpoint info */

  /* The bound device class driver */

  struct usbhost_class_s *class;
};

/* This structure retains the overall state of the USB host controller */

struct sam_ehci_s
{
  volatile bool pscwait;       /* TRUE: Thread is waiting for port status change event */
  sem_t exclsem;               /* Support mutually exclusive access */
  sem_t pscsem;                /* Semaphore to wait for port status change events */

  struct sam_epinfo_s ep0;     /* Endpoint 0 */
  struct sam_list_s *qhfree;   /* List of free Queue Head (QH) structures */
  struct sam_list_s *qtdfree;  /* List of free Queue Element Transfer Descriptor (qTD) */
  struct work_s work;          /* Supports interrupt bottom half */

  /* Root hub ports */

  struct sam_rhport_s rhport[SAM_EHCI_NRHPORT];
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations ********************************************************/

static uint16_t sam_read16(const uint8_t *addr);
static uint32_t sam_read32(const uint8_t *addr);
#if 0 /* Not used */
static void sam_write16(uint16_t memval, uint8_t *addr);
static void sam_write32(uint32_t memval, uint8_t *addr);
#endif

#ifdef CONFIG_ENDIAN_BIG
static uint16_t sam_swap16(uint16_t value);
static uint32_t sam_swap32(uint32_t value);
#else
#  define sam_swap16(value) (value)
#  define sam_swap32(value) (value)
#endif

#ifdef CONFIG_SAMA5_EHCI_REGDEBUG
static void sam_printreg(volatile uint32_t *regaddr, uint32_t regval,
         bool iswrite);
static void sam_checkreg(volatile uint32_t *regaddr, uint32_t regval,
         bool iswrite);
static uint32_t sam_getreg(volatile uint32_t *regaddr);
static void sam_putreg(uint32_t regval, volatile uint32_t *regaddr);
#else
static inline uint32_t sam_getreg(volatile uint32_t *regaddr);
static inline void sam_putreg(uint32_t regval, volatile uint32_t *regaddr);
#endif
static int ehci_wait_usbsts(uint32_t maskbits, uint32_t donebits,
         unsigned int delay);

/* Semaphores ******************************************************************/

static void sam_takesem(sem_t *sem);
#define sam_givesem(s) sem_post(s);

/* Allocators ******************************************************************/

static struct sam_qh_s *sam_qh_alloc(void);
static void sam_qh_free(struct sam_qh_s *qh);
static struct sam_qtd_s *sam_qtd_alloc(void);
static void sam_qtd_free(struct sam_qtd_s *qtd);

/* List Management *************************************************************/

static int sam_qh_foreach(struct sam_qh_s *qh, uint32_t **bp,
         foreach_qh_t handler, void *arg);
static int sam_qh_forall(foreach_qh_t handler, void *arg);
static int sam_qtd_foreach(struct sam_qh_s *qh, foreach_qtd_t handler,
         void *arg);
static int sam_qtd_discard(struct sam_qtd_s *qtd, uint32_t **bp, void *arg);
static int sam_qh_discard(struct sam_qh_s *qh);

/* Cache Operations ************************************************************/

#if 0 /* Not used */
static int sam_qtd_invalidate(struct sam_qtd_s *qtd, uint32_t **bp, void *arg);
static int sam_qh_invalidate(struct sam_qh_s *qh);
#endif
static int sam_qtd_flush(struct sam_qtd_s *qtd, uint32_t **bp, void *arg);
static int sam_qh_flush(struct sam_qh_s *qh);

/* Endpoint Transfer Handling **************************************************/

#ifdef CONFIG_SAMA5_EHCI_REGDEBUG
static void sam_qtd_print(struct sam_qtd_s *qtd);
static void sam_qh_print(struct sam_qh_s *qh);
static int sam_qtd_dump(struct sam_qtd_s *qtd, uint32_t **bp, void *arg);
static int sam_qh_dump(struct sam_qh_s *qh, uint32_t **bp, void *arg);
#if 0 /* not used */
static int sam_qh_dumpall(void);
#endif
#else
#  define sam_qtd_print(qtd)
#  define sam_qh_print(qh)
#  define sam_qtd_dump(qtd, bp, arg) OK
#  define sam_qh_dump(qh, bp, arg)   OK
#  define sam_qh_dumpall()           OK
#endif

static int sam_ioc_setup(struct sam_rhport_s *rhport, struct sam_epinfo_s *epinfo);
static int sam_ioc_wait(struct sam_epinfo_s *epinfo);
static void sam_qh_enqueue(struct sam_qh_s *qh);
static struct sam_qh_s *sam_qh_create(struct sam_rhport_s *rhport,
         struct sam_epinfo_s *epinfo);
static int sam_qtd_addbpl(struct sam_qtd_s *qtd, const void *buffer, size_t buflen);
static struct sam_qtd_s *sam_qtd_setupphase(struct sam_epinfo_s *epinfo,
         const struct usb_ctrlreq_s *req);
static struct sam_qtd_s *sam_qtd_dataphase(struct sam_epinfo_s *epinfo,
         void *buffer, int buflen, uint32_t tokenbits);
static struct sam_qtd_s *sam_qtd_statusphase(uint32_t tokenbits);
static ssize_t sam_async_transfer(struct sam_rhport_s *rhport,
         struct sam_epinfo_s *epinfo, const struct usb_ctrlreq_s *req,
         uint8_t *buffer, size_t buflen);

/* Interrupt Handling **********************************************************/

static int sam_qtd_ioccheck(struct sam_qtd_s *qtd, uint32_t **bp, void *arg);
static int sam_qh_ioccheck(struct sam_qh_s *qh, uint32_t **bp, void *arg);
static inline void sam_ioc_bottomhalf(void);
static inline void sam_portsc_bottomhalf(void);
static inline void sam_syserr_bottomhalf(void);
static inline void sam_async_advance_bottomhalf(void);
static void sam_ehci_bottomhalf(FAR void *arg);
static int sam_ehci_tophalf(int irq, FAR void *context);

/* USB Host Controller Operations **********************************************/

static int sam_wait(FAR struct usbhost_connection_s *conn,
         FAR const bool *connected);
static int sam_enumerate(FAR struct usbhost_connection_s *conn, int rhpndx);

static int sam_ep0configure(FAR struct usbhost_driver_s *drvr, uint8_t funcaddr,
         uint16_t maxpacketsize);
static int sam_epalloc(FAR struct usbhost_driver_s *drvr,
         const FAR struct usbhost_epdesc_s *epdesc, usbhost_ep_t *ep);
static int sam_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int sam_alloc(FAR struct usbhost_driver_s *drvr,
         FAR uint8_t **buffer, FAR size_t *maxlen);
static int sam_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int sam_ioalloc(FAR struct usbhost_driver_s *drvr,
         FAR uint8_t **buffer, size_t buflen);
static int sam_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int sam_ctrlin(FAR struct usbhost_driver_s *drvr,
         FAR const struct usb_ctrlreq_s *req, FAR uint8_t *buffer);
static int sam_ctrlout(FAR struct usbhost_driver_s *drvr,
         FAR const struct usb_ctrlreq_s *req, FAR const uint8_t *buffer);
static int sam_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
         FAR uint8_t *buffer, size_t buflen);
static void sam_disconnect(FAR struct usbhost_driver_s *drvr);

/* Initialization **************************************************************/

static int sam_reset(void);

/*******************************************************************************
 * Private Data
 *******************************************************************************/
/* In this driver implementation, support is provided for only a single a single
 * USB device.  All status information can be simply retained in a single global
 * instance.
 */

static struct sam_ehci_s g_ehci;

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_ehciconn;

/* The head of the asynchronous queue */

static struct sam_qh_s g_asynchead __attribute__ ((aligned(32)));

#ifndef CONFIG_USBHOST_INT_DISABLE
/* The head of the periodic queue */

static struct sam_qh_s g_perhead   __attribute__ ((aligned(32)));

/* The frame list */

static uint32_t g_framelist[FRAME_LIST_SIZE] __attribute__ ((aligned(4096)));
#endif

/* Pools of pre-allocated data structures.  These will all be linked into the
 * free lists within g_ehci.  These must all be aligned to 32-byte boundaries
 */

/* Queue Head (QH) pool */

static struct sam_qh_s g_qhpool[CONFIG_SAMA5_EHCI_NQHS]
                       __attribute__ ((aligned(32)));

/* Queue Element Transfer Descriptor (qTD) pool */

static struct sam_qtd_s g_qtdpool[CONFIG_SAMA5_EHCI_NQTDS]
                        __attribute__ ((aligned(32)));

/*******************************************************************************
 * Private Functions
 *******************************************************************************/
/*******************************************************************************
 * Register Operations
 *******************************************************************************/
/*******************************************************************************
 * Name: sam_read16
 *
 * Description:
 *   Read 16-bit little endian data
 *
 *******************************************************************************/

static uint16_t sam_read16(const uint8_t *addr)
{
#ifdef CONFIG_ENDIAN_BIG
  return (uint16_t)addr[0] << 8 | (uint16_t)addr[1];
#else
  return (uint16_t)addr[1] << 8 | (uint16_t)addr[0];
#endif
}

/*******************************************************************************
 * Name: sam_read32
 *
 * Description:
 *   Read 32-bit little endian data
 *
 *******************************************************************************/

static inline uint32_t sam_read32(const uint8_t *addr)
{
#ifdef CONFIG_ENDIAN_BIG
  return (uint32_t)sam_read16(&addr[0]) << 16 |
         (uint32_t)sam_read16(&addr[2]);
#else
  return (uint32_t)sam_read16(&addr[2]) << 16 |
         (uint32_t)sam_read16(&addr[0]);
#endif
}

/*******************************************************************************
 * Name: sam_write16
 *
 * Description:
 *   Write 16-bit little endian data
 *
 *******************************************************************************/

#if 0 /* Not used */
static void sam_write16(uint16_t memval, uint8_t *addr)
{
#ifdef CONFIG_ENDIAN_BIG
  addr[0] = memval & 0xff;
  addr[1] = memval >> 8;
#else
  addr[0] = memval >> 8;
  addr[1] = memval & 0xff;
#endif
}
#endif

/*******************************************************************************
 * Name: sam_write32
 *
 * Description:
 *   Write 32-bit little endian data
 *
 *******************************************************************************/

#if 0 /* Not used */
static void sam_write32(uint32_t memval, uint8_t *addr)
{
#ifdef CONFIG_ENDIAN_BIG
  sam_write16(memval >> 16, &addr[0]);
  sam_write16(memval & 0xffff, &addr[2]);
#else
  sam_write16(memval & 0xffff, &addr[0]);
  sam_write16(memval >> 16, &addr[2]);
#endif
}
#endif

/*******************************************************************************
 * Name: sam_swap16
 *
 * Description:
 *   Swap bytes on a 16-bit value
 *
 *******************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
static uint16_t sam_swap16(uint16_t value)
{
  return ((value >> 8) & 0xff) | ((value & 0xff) << 8);
}
#endif

/*******************************************************************************
 * Name: sam_swap32
 *
 * Description:
 *   Swap bytes on a 32-bit value
 *
 *******************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
static uint32_t sam_swap32(uint32_t value)
{
  return (uint32_t)sam_swap16((uint16_t)((value >> 16) & 0xffff)) |
         (uint32_t)sam_swap16((uint16_t)(value & 0xffff)) << 16;
}
#endif

/*******************************************************************************
 * Name: sam_printreg
 *
 * Description:
 *   Print the contents of a SAMA5 EHCI register
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_EHCI_REGDEBUG
static void sam_printreg(volatile uint32_t *regaddr, uint32_t regval,
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

#ifdef CONFIG_SAMA5_EHCI_REGDEBUG
static void sam_checkreg(volatile uint32_t *regaddr, uint32_t regval, bool iswrite)
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

#ifdef CONFIG_SAMA5_EHCI_REGDEBUG
static uint32_t sam_getreg(volatile uint32_t *regaddr)
{
  /* Read the value from the register */

  uint32_t regval = *regaddr;

  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, false);
  return regval;
}
#else
static inline uint32_t sam_getreg(volatile uint32_t *regaddr)
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

#ifdef CONFIG_SAMA5_EHCI_REGDEBUG
static void sam_putreg(uint32_t regval, volatile uint32_t *regaddr)
{
  /* Check if we need to print this value */

  sam_checkreg(regaddr, regval, true);

  /* Write the value */

  *regaddr = regval;
}
#else
static inline void sam_putreg(uint32_t regval, volatile uint32_t *regaddr)
{
  *regaval = regval;
}
#endif

/*******************************************************************************
 * Name: ehci_wait_usbsts
 *
 * Description:
 *   Wait for either (1) a field in the USBSTS register to take a specific
 *   value, (2) for a timeout to occur, or (3) a error to occur.  Return
 *   a value to indicate which terminated the wait.
 *
 *******************************************************************************/

static int ehci_wait_usbsts(uint32_t maskbits, uint32_t donebits,
                            unsigned int delay)
{
  uint32_t regval;
  unsigned int timeout;

  timeout = 0;
  do
    {
      /* Wait 5usec before trying again */

      up_udelay(5);
      timeout += 5;

      /* Read the USBSTS register and check for a system error */

      regval = sam_getreg(&HCOR->usbsts);
      if ((regval & EHCI_INT_SYSERROR) != 0)
        {
          udbg("ERROR: System error: %08x\n", regval);
          return -EIO;
        }

      /* Mask out the bits of interest */

      regval &= maskbits;

      /* Loop until the masked bits take the specified value or until a
       * timeout occurs.
       */
    }
  while (regval != donebits && timeout < delay);

  /* We got here because either the waited for condition or a timeout
   * occurred.  Return a value to indicate which.
   */

  return (regval == donebits) ? OK : -ETIMEDOUT;
}

/*******************************************************************************
 * Semaphores
 *******************************************************************************/
/*******************************************************************************
 * Name: sam_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 *******************************************************************************/

static void sam_takesem(sem_t *sem)
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

/*******************************************************************************
 * Allocators
 *******************************************************************************/
/*******************************************************************************
 * Name: sam_qh_alloc
 *
 * Description:
 *   Allocate a Queue Head (QH) structure by removing it from the free list
 *
 * Assumption:  Caller holds the exclsem
 *
 *******************************************************************************/

static struct sam_qh_s *sam_qh_alloc(void)
{
  struct sam_qh_s *qh;

  /* Remove the QH structure from the freelist */

  qh = (struct sam_qh_s *)g_ehci.qhfree;
  if (qh)
    {
      g_ehci.qhfree = ((struct sam_list_s *)qh)->flink;
      memset(qh, 0, sizeof(struct sam_qh_s));
    }

  return qh;
}

/*******************************************************************************
 * Name: sam_qh_free
 *
 * Description:
 *   Free a Queue Head (QH) structure by returning it to the free list
 *
 * Assumption:  Caller holds the exclsem
 *
 *******************************************************************************/

static void sam_qh_free(struct sam_qh_s *qh)
{
  struct sam_list_s *entry = (struct sam_list_s *)qh;

  /* Put the QH structure back into the free list */

  entry->flink  = g_ehci.qhfree;
  g_ehci.qhfree = entry;
}

/*******************************************************************************
 * Name: sam_qtd_alloc
 *
 * Description:
 *   Allocate a Queue Element Transfer Descriptor (qTD) by removing it from the
 *   free list
 *
 * Assumption:  Caller holds the exclsem
 *
 *******************************************************************************/

static struct sam_qtd_s *sam_qtd_alloc(void)
{
  struct sam_qtd_s *qtd;

  /* Remove the qTD from the freelist */

  qtd = (struct sam_qtd_s *)g_ehci.qtdfree;
  if (qtd)
    {
      g_ehci.qtdfree = ((struct sam_list_s *)qtd)->flink;
      memset(qtd, 0, sizeof(struct sam_list_s));
    }

  return qtd;
}

/*******************************************************************************
 * Name: sam_qtd_free
 *
 * Description:
 *   Free a Queue Element Transfer Descriptor (qTD) by returning it to the free
 *   list
 *
 * Assumption:  Caller holds the exclsem
 *
 *******************************************************************************/

static void sam_qtd_free(struct sam_qtd_s *qtd)
{
  struct sam_list_s *entry = (struct sam_list_s *)qtd;

  /* Put the qTD back into the free list */

  entry->flink   = g_ehci.qtdfree;
  g_ehci.qtdfree = entry;
}

/*******************************************************************************
 * List Management
 *******************************************************************************/

/*******************************************************************************
 * Name: sam_qh_foreach
 *
 * Description:
 *   Give the first entry in a list of Queue Head (QH) structures, call the
 *   handler for each QH structure in the list (including the one at the head
 *   of the list).
 *
 *******************************************************************************/

static int sam_qh_foreach(struct sam_qh_s *qh, uint32_t **bp, foreach_qh_t handler,
                          void *arg)
{
  struct sam_qh_s *next;
  uintptr_t physaddr;
  int ret;

  DEBUGASSERT(qh && handler);
  while (qh)
    {
      /* Is this the end of the list?  Check the horizontal link pointer (HLP)
       * terminate (T) bit.  If T==1, then the HLP address is not valid.
       */

      physaddr = sam_swap32(qh->hw.hlp);
      if ((physaddr & QH_HLP_T) != 0)
        {
          /* Set the next pointer to NULL.  This will terminate the loop. */

          next = NULL;
        }

      /* Is the next QH the asynchronous list head which will always be at
       * the end of the asynchronous queue?
       */

      else if (sam_virtramaddr(physaddr & QH_HLP_MASK) == (uintptr_t)&g_asynchead)
        {
          /* That will also terminate the loop */

          next = NULL;
        }

      /* Otherwise, there is a QH structure after this one that describes
       * another transaction.
       */

      else
        {
          physaddr = sam_swap32(qh->hw.hlp) & QH_HLP_MASK;
          next     = (struct sam_qh_s *)sam_virtramaddr(physaddr);
        }

      /* Perform the user action on this entry.  The action might result in
       * unlinking the entry!  But that is okay because we already have the
       * next QH pointer.
       *
       * Notice that we do not manage the back pointer (bp).  If the callout
       * uses it, it must update it as necessary.
       */

      ret = handler(qh, bp, arg);

      /* If the handler returns any non-zero value, then terminate the traversal
       * early.
       */

      if (ret != 0)
        {
          return ret;
        }

      /* Set up to visit the next entry */

      qh = next;
    }

  return OK;
}

/*******************************************************************************
 * Name: sam_qh_forall
 *
 * Description:
 *   Setup and call sam_qh_foreach to that every element of the asynchronous
 *   queue is examined.
 *
 * Assumption:  The caller holds the EHCI exclsem
 *
 *******************************************************************************/

static int sam_qh_forall(foreach_qh_t handler, void *arg)
{
  struct sam_qh_s *qh;
  uint32_t *bp;

  /* Set the back pointer to the forward qTD pointer of the asynchronous
   * queue head.
   */

  bp = (uint32_t *)&g_asynchead.hw.hlp;
  qh = (struct sam_qh_s *)sam_virtramaddr(sam_swap32(*bp) & QH_HLP_MASK);

  /* Then traverse and operate on every QH and qTD in the list */

  return sam_qh_foreach(qh, &bp, handler, arg);
}

/*******************************************************************************
 * Name: sam_qtd_foreach
 *
 * Description:
 *   Give a Queue Head (QH) instance, call the handler for each qTD structure
 *   in the queue.
 *
 *******************************************************************************/

static int sam_qtd_foreach(struct sam_qh_s *qh, foreach_qtd_t handler, void *arg)
{
  struct sam_qtd_s *qtd;
  struct sam_qtd_s *next;
  uintptr_t physaddr;
  uint32_t *bp;
  int ret;

  DEBUGASSERT(qh && handler);

  /* Handle the special case where the queue is empty */

  bp       = &qh->fqp;                 /* Start of qTDs in original list */
  physaddr = sam_swap32(*bp);          /* Physical address of first qTD in CPU order */

  if ((physaddr & QTD_NQP_T) != 0)
    {
      return 0;
    }

  /* Start with the first qTD in the list */

  qtd  = (struct sam_qtd_s *)sam_virtramaddr(physaddr);
  next = NULL;

  /* And loop until we encounter the end of the qTD list */

  while (qtd)
    {
      /* Is this the end of the list?  Check the next qTD pointer (NQP)
       * terminate (T) bit.  If T==1, then the NQP address is not valid.
       */

      if ((sam_swap32(qtd->hw.nqp) & QTD_NQP_T) != 0)
        {
          /* Set the next pointer to NULL.  This will terminate the loop. */

          next = NULL;
        }
      else
        {
          physaddr = sam_swap32(qtd->hw.nqp) & QTD_NQP_NTEP_MASK;
          next     = (struct sam_qtd_s *)sam_virtramaddr(physaddr);
        }

      /* Perform the user action on this entry.  The action might result in
       * unlinking the entry!  But that is okay because we already have the
       * next qTD pointer.
       *
       * Notice that we do not manage the back pointer (bp).  If the callout
       * uses it, it must update it as necessary.
       */

      ret = handler(qtd, &bp, arg);

      /* If the handler returns any non-zero value, then terminate the traversal
       * early.
       */

      if (ret != 0)
        {
          return ret;
        }

      /* Set up to visit the next entry */

      qtd = next;
    }

  return OK;
}

/*******************************************************************************
 * Name: sam_qtd_discard
 *
 * Description:
 *   This is a sam_qtd_foreach callback.  It simply unlinks the QTD, updates
 *   the back pointer, and frees the QTD structure.
 *
 *******************************************************************************/

static int sam_qtd_discard(struct sam_qtd_s *qtd, uint32_t **bp, void *arg)
{
  DEBUGASSERT(qtd && bp && *bp);

  /* Remove the qTD from the list by updating the forward pointer to skip
   * around this qTD.  We do not change that pointer because are repeatedly
   * removing the aTD at the head of the QH list.
   */

  **bp = qtd->hw.nqp;

  /* Then free the qTD */

  sam_qtd_free(qtd);
  return OK;
}

/*******************************************************************************
 * Name: sam_qh_discard
 *
 * Description:
 *   Free the Queue Head (QH) and all qTD's attached to the QH.
 *
 * Assumptions:
 *   The QH structure itself has already been unlinked from whatever list it
 *   may have been in.
 *
 *******************************************************************************/

static int sam_qh_discard(struct sam_qh_s *qh)
{
  int ret;

  DEBUGASSERT(qh);

  /* Free all of the qTD's attached to the QH */

  ret = sam_qtd_foreach(qh, sam_qtd_discard, NULL);
  if (ret < 0)
    {
      udbg("ERROR: sam_qtd_foreach failed: %d\n", ret);
    }

  /* Then free the QH itself */

  sam_qh_free(qh);
  return ret;
}

/*******************************************************************************
 * Cache Operations
 *******************************************************************************/

/*******************************************************************************
 * Name: sam_qtd_invalidate
 *
 * Description:
 *   This is a callback from sam_qtd_foreach.  It simply invalidates D-cache for
 *   address range of the qTD entry.
 *
 *******************************************************************************/

#if 0 /* Not used */
static int sam_qtd_invalidate(struct sam_qtd_s *qtd, uint32_t **bp, void *arg)
{
  /* Invalidate the D-Cache, i.e., force reloading of the D-Cache from memory
   * memory over the specified address range.
   */

  cp15_invalidate_dcache((uintptr_t)&qtd->hw,
                         (uintptr_t)&qtd->hw + sizeof(struct ehci_qtd_s));
  return OK;
}
#endif

/*******************************************************************************
 * Name: sam_qh_invalidate
 *
 * Description:
 *   Invalidate the Queue Head and all qTD entries in the queue.
 *
 *******************************************************************************/

#if 0 /* Not used */
static int sam_qh_invalidate(struct sam_qh_s *qh)
{
  /* Invalidate the QH first so that we reload the qTD list head */

  cp15_invalidate_dcache((uintptr_t)&qh->hw,
                         (uintptr_t)&qh->hw + sizeof(struct ehci_qh_s));

  /* Then invalidate all of the qTD entries in the queue */

  return sam_qtd_foreach(qh, sam_qtd_invalidate, NULL);
}
#endif

/*******************************************************************************
 * Name: sam_qtd_flush
 *
 * Description:
 *   This is a callback from sam_qtd_foreach.  It simply flushes D-cache for
 *   address range of the qTD entry.
 *
 *******************************************************************************/

static int sam_qtd_flush(struct sam_qtd_s *qtd, uint32_t **bp, void *arg)
{
  /* Flush the D-Cache, i.e., make the contents of the memory match the contents
   * of the D-Cache in the specified address range.
   */

  cp15_coherent_dcache((uintptr_t)&qtd->hw,
                       (uintptr_t)&qtd->hw + sizeof(struct ehci_qtd_s));
  return OK;
}

/*******************************************************************************
 * Name: sam_qh_flush
 *
 * Description:
 *   Invalidate the Queue Head and all qTD entries in the queue.
 *
 *******************************************************************************/

static int sam_qh_flush(struct sam_qh_s *qh)
{
  /* Flush the QH first */

  cp15_coherent_dcache((uintptr_t)&qh->hw,
                       (uintptr_t)&qh->hw + sizeof(struct ehci_qh_s));

  /* Then flush all of the qTD entries in the queue */

  return sam_qtd_foreach(qh, sam_qtd_flush, NULL);
}

/*******************************************************************************
 * Endpoint Transfer Handling
 *******************************************************************************/

/*******************************************************************************
 * Name: sam_qtd_print
 *
 * Description:
 *   Print the context of one qTD
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_EHCI_REGDEBUG
static void sam_qtd_print(struct sam_qtd_s *qtd)
{
  udbg("  QTD[%p]:\n", qtd);
  udbg("    hw:\n");
  udbg("      nqp: %08x alt: %08x token: %08x\n",
       qtd->hw.nqp, qtd->hw.alt, qtd->hw.token);
  udbg("      bpl: %08x %08x %08x %08x %08x\n",
       qtd->hw.bpl[0], qtd->hw.bpl[1], qtd->hw.bpl[2],
       qtd->hw.bpl[3], qtd->hw.bpl[4]);
}
#endif

/*******************************************************************************
 * Name: sam_qh_print
 *
 * Description:
 *   Print the context of one QH
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_EHCI_REGDEBUG
static void sam_qh_print(struct sam_qh_s *qh)
{
  struct sam_epinfo_s *epinfo;
  struct ehci_overlay_s *overlay;

  udbg("QH[%p]:\n", qh);
  udbg("  hw:\n");
  udbg("    hlp: %08x epchar: %08x epcaps: %08x cqp: %08x\n",
       qh->hw.hlp, qh->hw.epchar, qh->hw.epcaps, qh->hw.cqp);

  overlay = &qh->hw.overlay;
  udbg("  overlay:\n");
  udbg("    nqp: %08x alt: %08x token: %08x\n",
       overlay->nqp, overlay->alt, overlay->token);
  udbg("    bpl: %08x %08x %08x %08x %08x\n",
       overlay->bpl[0], overlay->bpl[1], overlay->bpl[2],
       overlay->bpl[3], overlay->bpl[4]);

  udbg("  fqp:\n", qh->fqp);

  epinfo = qh->epinfo;
  udbg("  epinfo[%p]:\n", epinfo);
  if (epinfo)
    {
      udbg("    EP%d DIR=%s FA=%08x TYPE=%d MaxPacket=%d\n",
           epinfo->epno, epinfo->dirin ? "IN" : "OUT", epinfo->devaddr,
           epinfo->xfrtype, epinfo->maxpacket);
      udbg("    Toggle=%d iocwait=%d speed=%d result=%d\n",
           epinfo->toggle, epinfo->iocwait, epinfo->speed, epinfo->result);
    }
}
#endif

/*******************************************************************************
 * Name: sam_qtd_dump
 *
 * Description:
 *   This is a sam_qtd_foreach callout function.  It dumps the context of one
 *   qTD
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_EHCI_REGDEBUG
static int sam_qtd_dump(struct sam_qtd_s *qtd, uint32_t **bp, void *arg)
{
  sam_qtd_print(qtd);
  return OK;
}
#endif

/*******************************************************************************
 * Name: sam_qh_dump
 *
 * Description:
 *   This is a sam_qh_foreach callout function.  It dumps a QH structure and
 *   all of the qTD structures linked to the QH.
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_EHCI_REGDEBUG
static int sam_qh_dump(struct sam_qh_s *qh, uint32_t **bp, void *arg)
{
  sam_qh_print(qh);
  return sam_qtd_foreach(qh, sam_qtd_dump, NULL);
}
#endif

/*******************************************************************************
 * Name: sam_qh_dumpall
 *
 * Description:
 *   Set the request for the IOC event well BEFORE enabling the transfer (as
 *   soon as we are absolutely committed to the to avoid transfer).  We do this
 *   to minimize race conditions.  This logic would have to be expanded if we
 *   want to have more than one packet in flight at a time!
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_EHCI_REGDEBUG
#if 0 /* not used */
static int sam_qh_dumpall(void)
{
  return sam_qh_forall(sam_qh_dump, NULL);
}
#endif
#endif

/*******************************************************************************
 * Name: sam_ioc_setup
 *
 * Description:
 *   Set the request for the IOC event well BEFORE enabling the transfer (as
 *   soon as we are absolutely committed to the to avoid transfer).  We do this
 *   to minimize race conditions.  This logic would have to be expanded if we
 *   want to have more than one packet in flight at a time!
 *
 * Assumption:  The caller holds tex EHCI exclsem
 *
 *******************************************************************************/

static int sam_ioc_setup(struct sam_rhport_s *rhport, struct sam_epinfo_s *epinfo)
{
  irqstate_t flags;
  int ret = -ENODEV;

  DEBUGASSERT(rhport && epinfo && !epinfo->iocwait);

  /* Is the device still connected? */

  flags = irqsave();
  if (rhport->connected)
    {
      /* Then set wdhwait to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer
       * completed.
       */

      epinfo->iocwait = true;   /* We want to be awakened by IOC interrupt */
      epinfo->status  = 0;      /* No status yet */
      epinfo->xfrd    = 0;      /* Nothing transferred yet */
      epinfo->result  = -EBUSY; /* Transfer in progress */
      ret             = OK;     /* We are good to go */
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: sam_ioc_wait
 *
 * Description:
 *   Wait for the IOC event.
 *
 * Assumption:  The caller does *NOT* hold the EHCI exclsem.  That would cause
 * a deadlock when the bottom-half, worker thread needs to take the semaphore.
 *
 *******************************************************************************/

static int sam_ioc_wait(struct sam_epinfo_s *epinfo)
{
  /* Wait for the IOC event.  Loop to handle any false alarm semaphore counts. */

  while (epinfo->iocwait)
    {
      sam_takesem(&epinfo->iocsem);
    }

  return epinfo->result;
}

/*******************************************************************************
 * Name: sam_qh_enqueue
 *
 * Description:
 *   Add a new, ready-to-go QH w/attached qTDs to the asynchonous queue.
 *
 * Assumptions:  The caller holds the EHCI exclsem
 *
 *******************************************************************************/

static void sam_qh_enqueue(struct sam_qh_s *qh)
{
  uintptr_t physaddr;

  /* Set the internal fqp field.  When we transverse the the QH list later,
   * we need to know the correct place to start because the overlay may no
   * longer point to the first qTD entry.
   */

  qh->fqp = qh->hw.overlay.nqp;
  (void)sam_qh_dump(qh, NULL, NULL);

  /* Add the new QH to the head of the asynchronous queue list.
   *
   * First, attach the old head as the new QH HLP and flush the new QH and its
   * attached qTDs to RAM.
   */

  qh->hw.hlp = g_asynchead.hw.hlp;
  sam_qh_flush(qh);

  /* Then set the new QH as the first QH in the asychronous queue and flush the
   * modified head to RAM.
   */

  physaddr = (uintptr_t)sam_physramaddr((uintptr_t)qh);
  g_asynchead.hw.hlp = sam_swap32(physaddr | QH_HLP_TYP_QH);
  cp15_coherent_dcache((uintptr_t)&g_asynchead.hw,
                       (uintptr_t)&g_asynchead.hw + sizeof(struct ehci_qh_s));
}

/*******************************************************************************
 * Name: sam_qh_create
 *
 * Description:
 *   Create a new Queue Head (QH)
 *
 *******************************************************************************/

static struct sam_qh_s *sam_qh_create(struct sam_rhport_s *rhport,
                                      struct sam_epinfo_s *epinfo)
{
  struct sam_qh_s *qh;
  uint32_t regval;

  /* Allocate a new queue head structure */

  qh = sam_qh_alloc();
  if (qh == NULL)
    {
      udbg("ERROR: Failed to allocate a QH\n");
      return NULL;
    }

  /* Save the endpoint information with the QH itself */

  qh->epinfo = epinfo;

  /* Write QH endpoint characteristics:
   *
   * FIELD    DESCRIPTION                     VALUE/SOURCE
   * -------- ------------------------------- --------------------
   * DEVADDR  Device address                  Endpoint structure
   * I        Inactivate on Next Transaction  0
   * ENDPT    Endpoint number                 Endpoint structure
   * EPS      Endpoint speed                  Endpoint structure
   * DTC      Data toggle control             1
   * MAXPKT   Max packet size                 Endpoint structure
   * C        Control endpoint                Calculated
   * RL       NAK count reloaded              8
   */

  regval = ((uint32_t)epinfo->devaddr   << QH_EPCHAR_DEVADDR_SHIFT) |
           ((uint32_t)epinfo->epno      << QH_EPCHAR_ENDPT_SHIFT) |
           ((uint32_t)epinfo->speed     << QH_EPCHAR_EPS_SHIFT) |
           QH_EPCHAR_DTC |
           ((uint32_t)epinfo->maxpacket << QH_EPCHAR_MAXPKT_SHIFT) |
           ((uint32_t)8 << QH_EPCHAR_RL_SHIFT);

  /* Paragraph 3.6.3: "Control Endpoint Flag (C). If the QH.EPS field
   * indicates the endpoint is not a high-speed device, and the endpoint
   * is an control endpoint, then software must set this bit to a one.
   * Otherwise it should always set this bit to a zero."
   */

  if (epinfo->speed   != EHCI_HIGH_SPEED &&
      epinfo->xfrtype == USB_EP_ATTR_XFER_CONTROL)
    {
      regval |= QH_EPCHAR_C;
    }

  /* Save the endpoint characteristics word with the correct byte order */

  qh->hw.epchar = sam_swap32(regval);

  /* Write QH endpoint capabilities
   *
   * FIELD    DESCRIPTION                     VALUE/SOURCE
   * -------- ------------------------------- --------------------
   * SSMASK   Interrupt Schedule Mask         0
   * SCMASK   Split Completion Mask           0
   * HUBADDR  Hub Address                     Always 0 for now
   * PORT     Port number                     RH port index + 1
   * MULT     High band width multiplier      1
   *
   * REVISIT:  Future HUB support will require the HUB port number
   * and HUB device address to be included here.
   */

  regval = ((uint32_t)0                      << QH_EPCAPS_HUBADDR_SHIFT) |
           ((uint32_t)(rhport->rhpndx + 1)   << QH_EPCAPS_PORT_SHIFT) |
           ((uint32_t)1                      << QH_EPCAPS_MULT_SHIFT);

  qh->hw.epcaps = sam_swap32(regval);

  /* Mark this as the end of this list.  This will be overwritten if/when the
   * next qTD is added to the queue.
   */

  qh->hw.hlp         = sam_swap32(QH_HLP_T);
  qh->hw.overlay.nqp = sam_swap32(QH_NQP_T);
  qh->hw.overlay.alt = sam_swap32(QH_AQP_T);
  return qh;
}

/*******************************************************************************
 * Name: sam_qtd_addbpl
 *
 * Description:
 *   Add a buffer pointer list to a qTD.
 *
 *******************************************************************************/

static int sam_qtd_addbpl(struct sam_qtd_s *qtd, const void *buffer, size_t buflen)
{
  uint32_t physaddr;
  uint32_t nbytes;
  uint32_t next;
  int ndx;

  physaddr = (uint32_t)sam_physramaddr((uintptr_t)buffer);

  for (ndx = 0; ndx < 5; ndx++)
    {
      /* Write the physical address of the buffer into the qTD buffer pointer
       * list.
       */

      qtd->hw.bpl[ndx] = sam_swap32(physaddr);

      /* Get the next buffer pointer (in the case where we will have to transfer
       * more then one chunk).  This buffer must be aligned to a 4KB address
       * boundary.
       */

      next = (physaddr + 4096) & ~4095;

      /* How many bytes were included in the last buffer?  Was the the whole
       * thing?
       */

      nbytes = next - physaddr;
      if (nbytes >= buflen)
        {
          /* Yes... it was the whole thing.  Break out of the loop early. */

          break;
        }

      /* Adjust the buffer length and physical address for the next time
       * through the loop.
       */

      buflen  -= nbytes;
      physaddr = next;
    }

  /* Handle the case of a huge buffer > 4*4KB = 16KB */

  if (ndx >= 5)
    {
      udbg("ERROR:  Buffer too big.  Remaining %d\n", buflen);
      return -EFBIG;
    }

  return OK;
}

/*******************************************************************************
 * Name: sam_qtd_setupphase
 *
 * Description:
 *   Create a SETUP phase request qTD.
 *
 *******************************************************************************/

static struct sam_qtd_s *sam_qtd_setupphase(struct sam_epinfo_s *epinfo,
                                            const struct usb_ctrlreq_s *req)
{
  struct sam_qtd_s *qtd;
  uint32_t regval;
  int ret;

  /* Allocate a new Queue Element Transfer Descriptor (qTD) */

  qtd = sam_qtd_alloc();
  if (qtd == NULL)
    {
      udbg("ERROR: Failed to allocate request qTD");
      return NULL;
    }

  /* Mark this as the end of the list (this will be overwritten if another
   * qTD is added after this one.
   */

  qtd->hw.nqp = sam_swap32(QTD_NQP_T);
  qtd->hw.alt = sam_swap32(QTD_AQP_T);

  /* Write qTD token:
   *
   * FIELD    DESCRIPTION                     VALUE/SOURCE
   * -------- ------------------------------- --------------------
   * STATUS   Status                          QTD_TOKEN_ACTIVE
   * PID      PID Code                        QTD_TOKEN_PID_SETUP
   * CERR     Error Counter                   3
   * CPAGE    Current Page                    0
   * IOC      Interrupt on complete           0
   * NBYTES   Total Bytes to Transfer         USB_SIZEOF_CTRLREQ
   * TOGGLE   Data Toggle                     0
   */

  regval = QTD_TOKEN_ACTIVE | QTD_TOKEN_PID_SETUP |
           ((uint32_t)3                  << QTD_TOKEN_CERR_SHIFT) |
           ((uint32_t)USB_SIZEOF_CTRLREQ << QTD_TOKEN_NBYTES_SHIFT);

  qtd->hw.token = sam_swap32(regval);

  /* Add the buffer data */

  ret = sam_qtd_addbpl(qtd, req, USB_SIZEOF_CTRLREQ);
  if (ret < 0)
    {
      udbg("ERROR: sam_qtd_addbpl failed: %d\n", ret);
      sam_qtd_free(qtd);
      return NULL;
    }

  /* Add the data transfer size to the count in the epinfo structure */

  epinfo->xfrd += USB_SIZEOF_CTRLREQ;

  return qtd;
}

/*******************************************************************************
 * Name: sam_qtd_dataphase
 *
 * Description:
 *   Create a data transfer or SET data phase qTD.
 *
 *******************************************************************************/

static struct sam_qtd_s *sam_qtd_dataphase(struct sam_epinfo_s *epinfo,
                                           void *buffer, int buflen,
                                           uint32_t tokenbits)
{
  struct sam_qtd_s *qtd;
  uint32_t regval;
  int ret;

  /* Allocate a new Queue Element Transfer Descriptor (qTD) */

  qtd = sam_qtd_alloc();
  if (qtd == NULL)
    {
      udbg("ERROR: Failed to allocate data buffer qTD");
      return NULL;
    }

  /* Mark this as the end of the list (this will be overwritten if another
   * qTD is added after this one.
   */

  qtd->hw.nqp = sam_swap32(QTD_NQP_T);
  qtd->hw.alt = sam_swap32(QTD_AQP_T);

  /* Write qTD token:
   *
   * FIELD    DESCRIPTION                     VALUE/SOURCE
   * -------- ------------------------------- --------------------
   * STATUS   Status                          QTD_TOKEN_ACTIVE
   * PID      PID Code                        Contained in tokenbits
   * CERR     Error Counter                   3
   * CPAGE    Current Page                    0
   * IOC      Interrupt on complete           Contained in tokenbits
   * NBYTES   Total Bytes to Transfer         buflen
   * TOGGLE   Data Toggle                     Contained in tokenbits
   */

  regval = tokenbits | QTD_TOKEN_ACTIVE |
           ((uint32_t)3       << QTD_TOKEN_CERR_SHIFT) |
           ((uint32_t)buflen  << QTD_TOKEN_NBYTES_SHIFT);

  qtd->hw.token = sam_swap32(regval);

  /* Add the buffer information to the bufffer pointer list */

  ret = sam_qtd_addbpl(qtd, buffer, buflen);
  if (ret < 0)
    {
      udbg("ERROR: sam_qtd_addbpl failed: %d\n", ret);
      sam_qtd_free(qtd);
      return NULL;
    }

  /* Add the data transfer size to the count in the epinfo structure */

  epinfo->xfrd += buflen;

  return qtd;
}

/*******************************************************************************
 * Name: sam_qtd_statusphase
 *
 * Description:
 *   Create a STATUS phase request qTD.
 *
 *******************************************************************************/

static struct sam_qtd_s *sam_qtd_statusphase(uint32_t tokenbits)
{
  struct sam_qtd_s *qtd;
  uint32_t regval;

  /* Allocate a new Queue Element Transfer Descriptor (qTD) */

  qtd = sam_qtd_alloc();
  if (qtd == NULL)
    {
      udbg("ERROR: Failed to allocate request qTD");
      return NULL;
    }

  /* Mark this as the end of the list (this will be overwritten if another
   * qTD is added after this one.
   */

  qtd->hw.nqp = sam_swap32(QTD_NQP_T);
  qtd->hw.alt = sam_swap32(QTD_AQP_T);

  /* Write qTD token:
   *
   * FIELD    DESCRIPTION                     VALUE/SOURCE
   * -------- ------------------------------- --------------------
   * STATUS   Status                          QTD_TOKEN_ACTIVE
   * PID      PID Code                        Contained in tokenbits
   * CERR     Error Counter                   3
   * CPAGE    Current Page                    0
   * IOC      Interrupt on complete           QTD_TOKEN_IOC
   * NBYTES   Total Bytes to Transfer         0
   * TOGGLE   Data Toggle                     Contained in tokenbits
   */

  regval = tokenbits | QTD_TOKEN_ACTIVE | QTD_TOKEN_IOC |
           ((uint32_t)3 << QTD_TOKEN_CERR_SHIFT);

  qtd->hw.token = sam_swap32(regval);
  return qtd;
}

/*******************************************************************************
 * Name: sam_async_transfer
 *
 * Description:
 *   Process a IN or OUT request on any asynchronous endpoint (bulk or control).
 *   This function will enqueue the request and wait for it to complete.  Bulk
 *   data transfers differ in that req == NULL and there are not SETUP or STATUS
 *   phases.
 *
 *   This is a blocking function; it will not return until the control transfer
 *   has completed.
 *
 * Assumption:  The caller holds the EHCI exclsem.  The caller must be aware
 *   that the EHCI exclsem will released while waiting for the transfer to
 *   complete, but will be re-aquired when before returning.  The state of
 *   EHCI resources could be very different upon return.
 *
 * Returned value:
 *   On success, this function returns the number of bytes actually transferred.
 *   For control transfers, this size includes the size of the control request
 *   plus the size of the data (which could be short); For bulk transfers, this
 *   will be the number of data bytes transfers (which could be short).
 *
 *******************************************************************************/

static ssize_t sam_async_transfer(struct sam_rhport_s *rhport,
                                  struct sam_epinfo_s *epinfo,
                                  const struct usb_ctrlreq_s *req,
                                  uint8_t *buffer, size_t buflen)
{
  struct sam_qh_s *qh;
  struct sam_qtd_s *qtd;
  uintptr_t physaddr;
  uint32_t *flink;
  uint32_t toggle;
  uint32_t datapid;
  int ret;

  uvdbg("RHport%d EP%d: buffer=%p, buflen=%d, req=%p\n",
        rhport->rhpndx+1, epinfo->epno, buffer, buflen, req);

  DEBUGASSERT(rhport && epinfo);

  if (req != NULL)
    {
      uvdbg("req=%02x type=%02x value=%04x index=%04x\n",
            req->req, req->type, sam_read16(req->value),
            sam_read16(req->index));
    }

  /* A buffer may or may be supplied with an EP0 SETUP transfer.  A buffer will
   * always be present for normal endpoint data transfers.
   */

  DEBUGASSERT(req || (buffer && buflen > 0));

  /* Set the request for the IOC event well BEFORE enabling the transfer. */

  ret = sam_ioc_setup(rhport, epinfo);
  if (ret != OK)
    {
      udbg("ERROR: Device disconnected\n");
      return ret;
    }

  /* Get the data token direction */

  datapid = QTD_TOKEN_PID_OUT;
  if (req)
    {
      if ((req->req & USB_REQ_DIR_MASK) == USB_REQ_DIR_IN)
        {
          datapid = QTD_TOKEN_PID_IN;
        }
    }
  else if (epinfo->dirin)
    {
      datapid = QTD_TOKEN_PID_IN;
    }

  /* Create and initialize a Queue Head (QH) structure for this transfer */

  qh = sam_qh_create(rhport, epinfo);
  if (qh == NULL)
    {
      udbg("ERROR: sam_qh_create failed\n");
      ret = -ENOMEM;
      goto errout_with_iocwait;
    }

  /* Initialize the QH link and get the next data toggle (not used for SETUP
   * transfers)
   */

  flink  = &qh->hw.overlay.nqp;
  toggle = (uint32_t)epinfo->toggle << QTD_TOKEN_TOGGLE_SHIFT;
  ret    = -EIO;

  /* Is the an EP0 SETUP request?  If req will be non-NULL */

  if (req != NULL)
    {
      /* Allocate a new Queue Element Transfer Descriptor (qTD) for the SETUP
       * phase of the request sequence.
       */

      qtd = sam_qtd_setupphase(epinfo, req);
      if (qtd == NULL)
        {
          udbg("ERROR: sam_qtd_setupphase failed\n");
          goto errout_with_qh;
        }

      /* Link the new qTD to the QH head. */

      physaddr = sam_physramaddr((uintptr_t)qtd);
      *flink = sam_swap32(physaddr);

      /* Get the new forward link pointer and data toggle */

      flink  = &qtd->hw.nqp;
      toggle = 0;
    }

  /* A buffer may or may be supplied with an EP0 SETUP transfer.  A buffer will
   * always be present for normal endpoint data transfers.
   */

  if (buffer && buflen > 0)
    {
      /* Extra TOKEN bits include the data toggle, the data PID, and if there
       * is no request, and indication to interrupt at the end of this
       * transfer.
       */

      uint32_t tokenbits = toggle | datapid;

      /* If this is not an EP0 SETUP request, then nothing follows the data and
       * we want the IOC interrupt when the data transfer completes.
       */

      if (!req)
        {
          tokenbits |= QTD_TOKEN_IOC;
        }

      /* Allocate a new Queue Element Transfer Descriptor (qTD) for the data
       * buffer.
       */

      qtd = sam_qtd_dataphase(epinfo, buffer, buflen, tokenbits);
      if (qtd == NULL)
        {
          udbg("ERROR: sam_qtd_dataphase failed\n");
          goto errout_with_qh;
        }

      /* Link the new qTD to either QH head of the SETUP qTD. */

      physaddr = sam_physramaddr((uintptr_t)qtd);
      *flink = sam_swap32(physaddr);

      /* Set the forward link pointer to this new qTD */

      flink = &qtd->hw.nqp;
    }

  if (req != NULL)
    {
      /* Extra TOKEN bits include the data toggle and the data PID. */

      uint32_t tokenbits = toggle | datapid;

      /* Allocate a new Queue Element Transfer Descriptor (qTD) for the status */

      qtd = sam_qtd_statusphase(tokenbits);
      if (qtd == NULL)
        {
          udbg("ERROR: sam_qtd_statusphase failed\n");
          goto errout_with_qh;
        }

      /* Link the new qTD to either the SETUP or data qTD. */

      physaddr = sam_physramaddr((uintptr_t)qtd);
      *flink = sam_swap32(physaddr);
    }

  /* Add the new QH to the head of the asynchronous queue list */

  sam_qh_enqueue(qh);

  /* Release the EHCI semaphore while we wait.  Other threads need the
   * opportunity to access the EHCI resources while we wait.
   *
   * REVISIT:  Is this safe?  NO.  This is a bug and needs rethinking.
   * We need to lock all of the port-resources (not EHCI common) until
   * the transfer is complete.  But we can't use the common ECHI exclsem
   * or we will deadlock while waiting (because the working thread that
   * wakes this thread up needs the exclsem).
   */
#warning REVISIT
  sam_givesem(&g_ehci.exclsem);

  /* Wait for the IOC completion event */

  ret = sam_ioc_wait(epinfo);

  /* Re-aquire the ECHI semaphore.  The caller expects to be holding
   * this upon return.
   */

  sam_takesem(&g_ehci.exclsem);

  /* Did sam_ioc_wait() report an error? */

  if (ret < 0)
    {
      udbg("ERROR: Transfer failed\n");
      goto errout_with_iocwait;
    }

  /* Transfer completed successfully.  Return the number of bytes transferred */

  return epinfo->xfrd;

  /* Clean-up after an error */

errout_with_qh:
  sam_qh_discard(qh);
errout_with_iocwait:
  epinfo->iocwait = false;
  return (ssize_t)ret;
}

/*******************************************************************************
 * EHCI Interrupt Handling
 *******************************************************************************/

/*******************************************************************************
 * Name: sam_qh_ioccheck
 *
 * Description:
 *   This function is a sam_qtd_foreach() callback function.  It services one
 *   qTD in the asynchronous queue.  It removes all of the qTD structures that
 *   are no longer active.
 *
 *******************************************************************************/

static int sam_qtd_ioccheck(struct sam_qtd_s *qtd, uint32_t **bp, void *arg)
{
  struct sam_epinfo_s *epinfo = (struct sam_epinfo_s *)arg;
  DEBUGASSERT(qtd && epinfo);

  /* Make sure we reload the QH from memory */

  cp15_invalidate_dcache((uintptr_t)&qtd->hw,
                         (uintptr_t)&qtd->hw + sizeof(struct ehci_qtd_s));
  sam_qtd_print(qtd);

  /* Remove the qTD from the list
   *
   * NOTE that we don't check if the qTD is active nor do we check if there
   * are any errors reported in the qTD.  If the transfer halted due to
   * an error, then qTDs in the list after the error qTD will still appear
   * to be active.
   */

  **bp = qtd->hw.nqp;

  /* Subtract the number of bytes left untransferred.  The epinfo->xfrd
   * field is initialized to the the total number of bytes to be transferred
   * (all qTDs in the list).  We subtract out the number of untransferred
   * bytes on each transfer and the final result will be the number of bytes
   * actually transferred.
   */

  epinfo->xfrd -= (sam_swap32(qtd->hw.token) & QTD_TOKEN_NBYTES_MASK) >>
    QTD_TOKEN_NBYTES_SHIFT;

  /* Release this QH by returning it to the free list */

  sam_qtd_free(qtd);
  return OK;
}

/*******************************************************************************
 * Name: sam_qh_ioccheck
 *
 * Description:
 *   This function is a sam_qh_foreach() callback function.  It services one
 *   QH in the asynchronous queue.  It check all attached qTD structures and
 *   remove all of the structures that are no longer active.  if all of the
 *   qTD structures are removed, then QH itself will also be removed.
 *
 *******************************************************************************/

static int sam_qh_ioccheck(struct sam_qh_s *qh, uint32_t **bp, void *arg)
{
  struct sam_epinfo_s *epinfo;
  uint32_t token;
  int ret;

  DEBUGASSERT(qh && bp);

  /* Make sure we reload the QH from memory */

  cp15_invalidate_dcache((uintptr_t)&qh->hw,
                         (uintptr_t)&qh->hw + sizeof(struct ehci_qh_s));
  sam_qh_print(qh);

  /* Get the endpoint info pointer from the extended QH data.  Only the
   * g_asynchead QH can have a NULL epinfo field.
   */

  epinfo = qh->epinfo;
  DEBUGASSERT(epinfo);

  /* Paragraph 3.6.3: "The nine DWords in [the Transfer Overlay] area represent
   * a transaction working space for the host controller. The general
   * operational model is that the host controller can detect whether the
   * overlay area contains a description of an active transfer. If it does
   * not contain an active transfer, then it follows the Queue Head Horizontal
   * Link Pointer to the next queue head. The host controller will never follow
   * the Next Transfer Queue Element or Alternate Queue Element pointers unless
   * it is actively attempting to advance the queue ..."
   */

  /* Is the qTD still active? */

  token = sam_swap32(qh->hw.overlay.token);
  uvdbg("EP%d TOKEN=%08x\n", epinfo->epno, token);

  if ((token & QH_TOKEN_ACTIVE) != 0)
    {
      /* Yes... we cannot process the QH while it is still active.  Return
       * zero to visit the next QH in the list.
       */

      return OK;
    }

  /* Remove all active, attached qTD structures from the inactive QH */

  ret = sam_qtd_foreach(qh, sam_qtd_ioccheck, (void *)qh->epinfo);
  if (ret < 0)
    {
      udbg("ERROR: sam_qh_forall failed: %d\n", ret);
    }

  /* If there is no longer anything attached to the QH, then remove it from
   * the asynchronous queue.
   */

  if ((sam_swap32(qh->fqp) & QTD_NQP_T) != 0)
    {
      /* Set the forward link of the previous QH to point to the next
       * QH in the list.
       */

      **bp = qh->hw.hlp;
      cp15_coherent_dcache((uintptr_t)*bp, (uintptr_t)*bp + sizeof(uint32_t));

      /* Check for errors, update the data toggle */

      if ((token & QH_TOKEN_ERRORS) == 0)
        {
          /* No errors.. Save the last data toggle value */

          epinfo->toggle = (token >> QTD_TOKEN_TOGGLE_SHIFT) & 1;

          /* Report success */

          epinfo->status  = 0;
          epinfo->result  = OK;
        }
      else
        {
          /* An error occurred */

          udbg("ERROR: EP%d TOKEN=%08x", epinfo->epno, token);
          epinfo->status = (token & QH_TOKEN_STATUS_MASK) >> QH_TOKEN_STATUS_SHIFT;
          epinfo->result = -EIO;
        }

      /* Is there a thread waiting for this transfer to complete? */

      if (epinfo->iocwait)
        {
          /* Yes... wake it up */

          sam_givesem(&epinfo->iocsem);
          epinfo->iocwait = 0;
        }

      /* Then release this QH by returning it to the free list */

      sam_qh_free(qh);
    }
  else
    {
      /* Otherwise, the horizontal link pointer of this QH will become the next back pointer.
       */

      *bp = &qh->hw.hlp;
    }

  return OK;
}

/*******************************************************************************
 * Name: sam_ioc_bottomhalf
 *
 * Description:
 *   EHCI USB Interrupt (USBINT) "Bottom Half" interrupt handler
 *
 *  "The Host Controller sets this bit to 1 on the  completion of a USB
 *   transaction, which results in the retirement of a Transfer Descriptor that
 *   had its IOC bit set.
 *
 *  "The Host Controller also sets this bit to 1 when a short packet is detected
 *   (actual number of bytes received was less than the expected number of
 *   bytes)."
 *
 * Assumptions:  The caller holds the EHCI exclsem
 *
 *******************************************************************************/

static inline void sam_ioc_bottomhalf(void)
{
  int ret;

  /* Make sure that the head of the asynchronous queue is invalidated */

  cp15_invalidate_dcache((uintptr_t)&g_asynchead.hw,
                         (uintptr_t)&g_asynchead.hw + sizeof(struct ehci_qh_s));

  /* Traverse all elements in the asynchronous queue */

  ret = sam_qh_forall(sam_qh_ioccheck, NULL);
  if (ret < 0)
    {
      udbg("ERROR: sam_qh_forall failed: %d\n", ret);
    }
}

/*******************************************************************************
 * Name: sam_portsc_bottomhalf
 *
 * Description:
 *   EHCI Port Change Detect "Bottom Half" interrupt handler
 *
 *  "The Host Controller sets this bit to a one when any port for which the Port
 *   Owner bit is set to zero ... has a change bit transition from a zero to a
 *   one or a Force Port Resume bit transition from a zero to a one as a result
 *   of a J-K transition detected on a suspended port.  This bit will also be set
 *   as a result of the Connect Status Change being set to a one after system
 *   software has relinquished ownership of a connected port by writing a one
 *   to a port's Port Owner bit...
 *
 *  "This bit is allowed to be maintained in the Auxiliary power well.
 *   Alternatively, it is also acceptable that on a D3 to D0 transition of the
 *   EHCI HC device, this bit is loaded with the OR of all of the PORTSC change
 *   bits (including: Force port resume, over-current change, enable/disable
 *   change and connect status change)."
 *
 *******************************************************************************/

static inline void sam_portsc_bottomhalf(void)
{
  struct sam_rhport_s *rhport;
  uint32_t portsc;
  int rhpndx;

  /* Handle root hub status change on each root port */

  for (rhpndx = 0; rhpndx < SAM_EHCI_NRHPORT; rhpndx++)
    {
      rhport = &g_ehci.rhport[rhpndx];
      portsc = sam_getreg(&HCOR->portsc[rhpndx]);

      uvdbg("PORTSC%d: %08x\n", rhpndx + 1, portsc);

      /* Handle port connection status change (CSC) events */

      if ((portsc & EHCI_PORTSC_CSC) != 0)
        {
          uvdbg("Connect Status Change\n");

          /* Check current connect status */

          if ((portsc & EHCI_PORTSC_CCS) != 0)
            {
              /* Connected ... Did we just become connected? */

              if (!rhport->connected)
                {
                  /* Yes.. connected. */

                  rhport->connected = true;

                  uvdbg("RHPort%d connected, pscwait: %d\n",
                        rhpndx + 1, g_ehci.pscwait);

                  /* Notify any waiters */

                  if (g_ehci.pscwait)
                    {
                      sam_givesem(&g_ehci.pscsem);
                      g_ehci.pscwait = false;
                    }
                }
              else
                {
                  uvdbg("Already connected\n");
                }
            }
          else
            {
              /* Disconnected... Did we just become disconnected? */

              if (rhport->connected)
                {
                  /* Yes.. disconnect the device */

                  uvdbg("RHport%d disconnected\n", rhpndx+1);
                  rhport->connected = false;
                  rhport->lowspeed  = false;

                  /* Are we bound to a class instance? */

                  if (rhport->class)
                    {
                      /* Yes.. Disconnect the class */

                      CLASS_DISCONNECTED(rhport->class);
                      rhport->class = NULL;
                    }

                  /* Notify any waiters for the Root Hub Status change
                   * event.
                   */

                  if (g_ehci.pscwait)
                    {
                      sam_givesem(&g_ehci.pscsem);
                      g_ehci.pscwait = false;
                    }
                }
              else
                {
                   uvdbg("Already disconnected\n");
                }
            }
        }

      /* Clear all pending port interrupt sources by writing a '1' to the
       * corresponding bit in the PORTSC register.
       */

      sam_putreg(portsc & EHCI_PORTSC_ALLINTS, &HCOR->portsc[rhpndx]);
    }
}

/*******************************************************************************
 * Name: sam_syserr_bottomhalf
 *
 * Description:
 *   EHCI Host System Error "Bottom Half" interrupt handler
 *
 *  "The Host Controller sets this bit to 1 when a serious error occurs during a
 *   host system access involving the Host Controller module. ... When this
 *   error occurs, the Host Controller clears the Run/Stop bit in the Command
 *   register to prevent further execution of the scheduled TDs."
 *
 *******************************************************************************/

static inline void sam_syserr_bottomhalf(void)
{
  udbg("Host System Error Interrupt\n");
  PANIC();
}

/*******************************************************************************
 * Name: sam_async_advance_bottomhalf
 *
 * Description:
 *   EHCI Async Advance "Bottom Half" interrupt handler
 *
 *  "System software can force the host controller to issue an interrupt the
 *   next time the host controller advances the asynchronous schedule by writing
 *   a one to the Interrupt on Async Advance Doorbell bit in the USBCMD
 *   register. This status bit indicates the assertion of that interrupt
 *   source."
 *
 *******************************************************************************/

static inline void sam_async_advance_bottomhalf(void)
{
  uvdbg("Async Advance Interrupt\n");

  /* REVISIT: Could remove all tagged QH entries here */
}

/*******************************************************************************
 * Name: sam_ehci_bottomhalf
 *
 * Description:
 *   EHCI "Bottom Half" interrupt handler
 *
 *******************************************************************************/

static void sam_ehci_bottomhalf(FAR void *arg)
{
  uint32_t pending = (uint32_t)arg;

  /* We need to have exclusive access to the EHCI data structures.  Waiting here
   * is not a good thing to do on the worker thread, but there is no real option
   * (other than to reschedule and delay).
   */

  sam_takesem(&g_ehci.exclsem);

  /* Handle all unmasked interrupt sources */
  /* USB Interrupt (USBINT)
   *
   *  "The Host Controller sets this bit to 1 on the completion of a USB
   *   transaction, which results in the retirement of a Transfer Descriptor
   *   that had its IOC bit set.
   *
   *  "The Host Controller also sets this bit to 1 when a short packet is
   *   detected (actual number of bytes received was less than the expected
   *   number of bytes)."
   *
   * USB Error Interrupt (USBERRINT)
   *
   *  "The Host Controller sets this bit to 1 when completion of a USB
   *   transaction results in an error condition (e.g., error counter
   *   underflow). If the TD on which the error interrupt occurred also
   *   had its IOC bit set, both this bit and USBINT bit are set. ..."
   *
   * We do the same thing in either case:  Traverse the asynchonous queue
   * and remove all of the transfers that are no longer active.
   */

  if ((pending & (EHCI_INT_USBINT | EHCI_INT_USBERRINT)) != 0)
    {
      if ((pending & EHCI_INT_USBERRINT) != 0)
        {
          udbg("USB Error Interrupt (USBERRINT) Interrupt\n");
        }
      else
        {
          uvdbg("USB Interrupt (USBINT) Interrupt\n");
        }

      sam_ioc_bottomhalf();
    }

  /* Port Change Detect
   *
   *  "The Host Controller sets this bit to a one when any port for which
   *   the Port Owner bit is set to zero ... has a change bit transition
   *   from a zero to a one or a Force Port Resume bit transition from a zero
   *   to a one as a result of a J-K transition detected on a suspended port.
   *   This bit will also be set as a result of the Connect Status Change
   *   being set to a one after system software has relinquished ownership
   *    of a connected port by writing a one to a port's Port Owner bit...
   *
   *  "This bit is allowed to be maintained in the Auxiliary power well.
   *   Alternatively, it is also acceptable that on a D3 to D0 transition
   *   of the EHCI HC device, this bit is loaded with the OR of all of the
   *   PORTSC change bits (including: Force port resume, over-current change,
   *   enable/disable change and connect status change)."
   */

  if ((pending & EHCI_INT_PORTSC) != 0)
    {
      sam_portsc_bottomhalf();
    }

  /* Frame List Rollover
   *
   *  "The Host Controller sets this bit to a one when the Frame List Index ...
   *   rolls over from its maximum value to zero. The exact value at which
   *   the rollover occurs depends on the frame list size. For example, if
   *   the frame list size (as programmed in the Frame List Size field of the
   *   USBCMD register) is 1024, the Frame Index Register rolls over every
   *   time FRINDEX[13] toggles. Similarly, if the size is 512, the Host
   *   Controller sets this bit to a one every time FRINDEX[12] toggles."
   */

#if 0 /* Not used */
  if ((pending & EHCI_INT_FLROLL) != 0)
    {
      sam_flroll_bottomhalf();
    }
#endif

  /* Host System Error
   *
   *  "The Host Controller sets this bit to 1 when a serious error occurs
   *   during a host system access involving the Host Controller module. ...
   *   When this error occurs, the Host Controller clears the Run/Stop bit
   *   in the Command register to prevent further execution of the scheduled
   *   TDs."
   */

  if ((pending & EHCI_INT_SYSERROR) != 0)
    {
      sam_syserr_bottomhalf();
    }

  /* Interrupt on Async Advance
   *
   *  "System software can force the host controller to issue an interrupt
   *   the next time the host controller advances the asynchronous schedule
   *   by writing a one to the Interrupt on Async Advance Doorbell bit in
   *   the USBCMD register. This status bit indicates the assertion of that
   *   interrupt source."
   */

  if ((pending & EHCI_INT_AAINT) != 0)
    {
      sam_async_advance_bottomhalf();
    }

  /* We are done with the EHCI structures */

  sam_givesem(&g_ehci.exclsem);

  /* Re-enable relevant EHCI interrupts.  Interrupts should still be enabled
   * at the level of the AIC.
   */

  sam_putreg(EHCI_HANDLED_INTS, &HCOR->usbintr);
}

/*******************************************************************************
 * Name: sam_ehci_tophalf
 *
 * Description:
 *   EHCI "Top Half" interrupt handler
 *
 *******************************************************************************/

static int sam_ehci_tophalf(int irq, FAR void *context)
{
  uint32_t usbsts;
  uint32_t pending;
  uint32_t regval;

  /* Read Interrupt Status and mask out interrupts that are not enabled. */

  usbsts = sam_getreg(&HCOR->usbsts);
  regval = sam_getreg(&HCOR->usbintr);
  ullvdbg("USBSTS: %08x USBINTR: %08x\n", usbsts, regval);

  /* Handle all unmasked interrupt sources */

  pending = usbsts & regval;
  if (pending != 0)
    {
      /* Schedule interrupt handling work for the high priority worker thread
       * so that we are not pressed for time and so that we can interrupt with
       * other USB threads gracefully.
       *
       * The worker should be available now because we implement a handshake
       * by controlling the EHCI interrupts.
       */

      DEBUGASSERT(work_available(&g_ehci.work));
      DEBUGVERIFY(work_queue(HPWORK, &g_ehci.work, sam_ehci_bottomhalf,
                            (FAR void *)pending, 0));

      /* Disable further EHCI interrupts so that we do not overrun the work
       * queue.
       */

      sam_putreg(0, &HCOR->usbintr);

      /* Clear all pending status bits by writing the value of the pending
       * interrupt bits back to the status register.
       */

      sam_putreg(usbsts & ECHI_INT_ALLINTS, &HCOR->usbsts);
    }

  return OK;
}

/*******************************************************************************
 * Name: sam_uhphs_interrupt
 *
 * Description:
 *   Common UHPHS interrupt handler.  When both OHCI and EHCI are enabled, EHCI
 *   owns the interrupt and provides the interrupting event to both the OHCI and
 *   EHCI controllers.
 *
 *******************************************************************************/

#ifdef CONFIG_SAMA5_OHCI
static int sam_uhphs_interrupt(int irq, FAR void *context)
{
   int ohci;
   int ehci;

   /* Provide the interrupting event to both the EHCI and OHCI top half */
   ohci = sam_ohci_tophalf(irq, context);
   ehci = sam_ehci_tophalf(irq, context);

   /* Return OK only if both handlers returned OK */

   return ohci == OK ? ehci : ohci;
}
#endif

/*******************************************************************************
 * USB Host Controller Operations
 *******************************************************************************/
/*******************************************************************************
 * Name: sam_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a root hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from the call to
 *      the USB driver initialization logic.
 *   connected - A pointer to an array of 3 boolean values corresponding to
 *      root hubs 1, 2, and 3.  For each boolean value: TRUE: Wait for a device
 *      to be connected on the root hub; FALSE: wait for device to be
 *      disconnected from the root hub.
 *
 * Returned Values:
 *   And index [0, 1, or 2} corresponding to the root hub port number {1, 2,
 *   or 3} is returned when a device is connected or disconnected. This
 *   function will not return until either (1) a device is connected or
 *   disconnected to/from any root hub port or until (2) some failure occurs.
 *   On a failure, a negated errno value is returned indicating the nature of
 *   the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int sam_wait(FAR struct usbhost_connection_s *conn,
                    FAR const bool *connected)
{
  irqstate_t flags;
  int rhpndx;

  /* Loop until a change in the connection state changes on one of the root hub
   * ports or until an error occurs.
   */

  flags = irqsave();
  for (;;)
    {
      /* Check for a change in the connection state on any root hub port */

      for (rhpndx = 0; rhpndx < SAM_EHCI_NRHPORT; rhpndx++)
        {
          /* Has the connection state changed on the RH port? */

          if (g_ehci.rhport[rhpndx].connected != connected[rhpndx])
            {
              /* Yes.. Return the RH port number */

              irqrestore(flags);

              uvdbg("RHPort%d connected: %s\n",
                    rhpndx + 1, g_ehci.rhport[rhpndx].connected ? "YES" : "NO");

              return rhpndx;
            }
        }

      /* No changes on any port. Wait for a connection/disconnection event
       * and check again
       */

      g_ehci.pscwait = true;
      sam_takesem(&g_ehci.pscsem);
    }
}

/*******************************************************************************
 * Name: sam_enumerate
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
 *   conn - The USB host connection instance obtained as a parameter from the call to
 *      the USB driver initialization logic.
 *   rphndx - Root hub port index.  0-(n-1) corresponds to root hub port 1-n.
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

static int sam_enumerate(FAR struct usbhost_connection_s *conn, int rhpndx)
{
  struct sam_rhport_s *rhport;
  volatile uint32_t *regaddr;
  uint32_t regval;

  DEBUGASSERT(rhpndx >= 0 && rhpndx < SAM_EHCI_NRHPORT);
  rhport = &g_ehci.rhport[rhpndx];

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!rhport->connected)
    {
      /* No, return an error */

      uvdbg("Not connected\n");
      return -ENODEV;
    }

  /* USB 2.0 spec says at least 50ms delay before port reset.
   * REVISIT:  I think this is wrong.  It needs to hold the port in
   * reset for 50Msec, not wait 50Msec before resetting.
   */

  usleep(100*1000);

  /* Paragraph 2.3.9:
   *
   *  "Line Status ... These bits reflect the current logical levels of the
   *   D+ (bit 11) and D- (bit 10) signal lines. These bits are used for
   *   detection of low-speed USB devices prior to the port reset and enable
   *   sequence. This field is valid only when the port enable bit is zero
   *   and the current connect status bit is set to a one."
   *
   *   Bits[11:10] USB State Interpretation
   *   ----------- --------- --------------
   *   00b         SE0       Not Low-speed device, perform EHCI reset
   *   10b         J-state   Not Low-speed device, perform EHCI reset
   *   01b         K-state   Low-speed device, release ownership of port
   */

  regval = sam_getreg(&HCOR->portsc[rhpndx]);
  if ((regval & EHCI_PORTSC_LSTATUS_MASK) == EHCI_PORTSC_LSTATUS_KSTATE)
    {
      /* Paragraph 2.3.9:
       *
       *   "Port Owner ... This bit unconditionally goes to a 0b when the
       *    Configured bit in the CONFIGFLAG register makes a 0b to 1b
       *    transition. This bit unconditionally goes to 1b whenever the
       *    Configured bit is zero.
       *
       *   "System software uses this field to release ownership of the
       *    port to a selected host controller (in the event that the
       *    attached device is not a high-speed device). Software writes
       *    a one to this bit when the attached device is not a high-speed
       *    device. A one in this bit means that a companion host
       *    controller owns and controls the port. ....
       *
       * Paragraph 4.2:
       *
       *   "When a port is routed to a companion HC, it remains under the
       *    control of the companion HC until the device is disconnected
       *    from the root por ... When a disconnect occurs, the disconnect
       *    event is detected by both the companion HC port control and the
       *    EHCI port ownership control. On the event, the port ownership
       *    is returned immediately to the EHCI controller. The companion
       *    HC stack detects the disconnect and acknowledges as it would
       *    in an ordinary standalone implementation. Subsequent connects
       *    will be detected by the EHCI port register and the process will
       *    repeat."
       */

      rhport->ep0.speed = EHCI_LOW_SPEED;
      regval |= EHCI_PORTSC_OWNER;
      sam_putreg(regval, &HCOR->portsc[rhpndx]);

#ifndef CONFIG_SAMA5_OHCI
      /* Give the port to the OHCI controller. Zero is the reset value for
       * all ports; one makes the corresponding port available to OHCI.
       */

      regval  = getreg32(SAM_SFR_OHCIICR);
      regval |= SFR_OHCIICR_RES(rhpndx);
      putreg32(regval, SAM_SFR_OHCIICR);
#endif

      /* And return a failure */

      return -EPERM;
    }
  else
    {
      /* Assume full-speed for now */

      rhport->ep0.speed = EHCI_FULL_SPEED;
    }

  /* Put the root hub port in reset.
   *
   * Paragraph 2.3.9:
   *
   *  "The HCHalted bit in the USBSTS register should be a zero before
   *   software attempts to use [the Port Reset] bit. The host controller
   *   may hold Port Reset asserted to a one when the HCHalted bit is a one.
   */

  DEBUGASSERT((sam_getreg(&HCOR->usbsts) & EHCI_USBSTS_HALTED) == 0);

  /* paragraph 2.3.9:
   *
   *  "When software writes a one to [the Port Reset] bit (from a zero), the
   *   bus reset sequence as defined in the USB Specification Revision 2.0 is
   *   started.  Software writes a zero to this bit to terminate the bus reset
   *   sequence.  Software must keep this bit at a one long enough to ensure
   *   the reset sequence, as specified in the USB Specification Revision 2.0,
   *   completes. Note: when software writes this bit to a one, it must also
   *   write a zero to the Port Enable bit."
   */

  regaddr = &HCOR->portsc[rhport->rhpndx];
  regval  = sam_getreg(regaddr);
  regval &= ~EHCI_PORTSC_PE;
  regval |= EHCI_PORTSC_RESET;
  sam_putreg(regval, regaddr);

  /* USB 2.0 "Root hubs must provide an aggregate reset period of at least
   * 50 ms."
   */

  usleep(50*1000);

  regval  = sam_getreg(regaddr);
  regval &= ~EHCI_PORTSC_RESET;
  sam_putreg(regval, regaddr);

  /* Wait for the port reset to complete
   *
   * Paragraph 2.3.9:
   *
   *  "Note that when software writes a zero to this bit there may be a
   *   delay before the bit status changes to a zero. The bit status will
   *   not read as a zero until after the reset has completed. If the port
   *   is in high-speed mode after reset is complete, the host controller
   *   will automatically enable this port (e.g. set the Port Enable bit
   *   to a one). A host controller must terminate the reset and stabilize
   *   the state of the port within 2 milliseconds of software transitioning
   *   this bit from a one to a zero ..."
   */

  while ((sam_getreg(regaddr) & EHCI_PORTSC_RESET) != 0);
  usleep(200*1000);

  /* Paragraph 4.2.2:
   *
   *  "... The reset process is actually complete when software reads a zero
   *   in the PortReset bit. The EHCI Driver checks the PortEnable bit in the
   *   PORTSC register. If set to a one, the connected device is a high-speed
   *   device and EHCI Driver (root hub emulator) issues a change report to the
   *   hub driver and the hub driver continues to enumerate the attached device."
   *
   *  "At the time the EHCI Driver receives the port reset and enable request
   *   the LineStatus bits might indicate a low-speed device. Additionally,
   *   when the port reset process is complete, the PortEnable field may
   *   indicate that a full-speed device is attached. In either case the EHCI
   *   driver sets the PortOwner bit in the PORTSC register to a one to
   *   release port ownership to a companion host controller."
   */
#warning REVISIT

  regval = sam_getreg(&HCOR->portsc[rhpndx]);
  if ((regval & EHCI_PORTSC_PE) != 0)
    {
      /* High speed device */

      rhport->ep0.speed = EHCI_HIGH_SPEED;
    }
  else
    {
      /* Low- or Full- speed device.  Set the port ownership bit.
       *
       * Paragraph 4.2:
       *
       *   "When a port is routed to a companion HC, it remains under the
       *    control of the companion HC until the device is disconnected
       *    from the root por ... When a disconnect occurs, the disconnect
       *    event is detected by both the companion HC port control and the
       *    EHCI port ownership control. On the event, the port ownership
       *    is returned immediately to the EHCI controller. The companion
       *    HC stack detects the disconnect and acknowledges as it would
       *    in an ordinary standalone implementation. Subsequent connects
       *    will be detected by the EHCI port register and the process will
       *    repeat."
       */

      regval |= EHCI_PORTSC_OWNER;
      sam_putreg(regval, &HCOR->portsc[rhpndx]);

#ifndef CONFIG_SAMA5_OHCI
      /* Give the port to the OHCI controller. Zero is the reset value for
       * all ports; one makes the corresponding port available to OHCI.
       */

      regval  = getreg32(SAM_SFR_OHCIICR);
      regval |= SFR_OHCIICR_RES(rhpndx);
      putreg32(regval, SAM_SFR_OHCIICR);
#endif

      /* And return a failure */

      return -EPERM;
    }

  /* Let the common usbhost_enumerate do all of the real work.  Note that the
   * FunctionAddress (USB address) is set to the root hub port number + 1
   * for now.
   *
   * REVISIT:  Hub support will require better device address assignment.
   * See include/nuttx/usb/usbhost_devaddr.h.
   */

  uvdbg("Enumerate the device\n");
  return usbhost_enumerate(&g_ehci.rhport[rhpndx].drvr, rhpndx+1, &rhport->class);
}

/************************************************************************************
 * Name: sam_ep0configure
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
 *     controls.  A funcaddr of zero will be received if no address is yet assigned
 *     to the device.
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

static int sam_ep0configure(FAR struct usbhost_driver_s *drvr, uint8_t funcaddr,
                            uint16_t maxpacketsize)
{
  struct sam_rhport_s *rhport = (struct sam_rhport_s *)drvr;
  struct sam_epinfo_s *epinfo;

  DEBUGASSERT(rhport &&
              funcaddr >= 0 && funcaddr <= SAM_EHCI_NRHPORT &&
              maxpacketsize < 2048);

  epinfo = &rhport->ep0;

  /* We must have exclusive access to the EHCI data structures. */

  sam_takesem(&g_ehci.exclsem);

  /* Remember the new device address and max packet size */

  epinfo->devaddr   = funcaddr;
  epinfo->maxpacket = maxpacketsize;

  sam_givesem(&g_ehci.exclsem);
  return OK;
}

/************************************************************************************
 * Name: sam_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint desciptor.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int sam_epalloc(FAR struct usbhost_driver_s *drvr,
                       const FAR struct usbhost_epdesc_s *epdesc, usbhost_ep_t *ep)
{
  struct sam_rhport_s *rhport = (struct sam_rhport_s *)drvr;
  struct sam_epinfo_s *epinfo;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(drvr && epdesc && ep);
  uvdbg("EP%d DIR=%s FA=%08x TYPE=%d Interval=%d MaxPacket=%d\n",
        epdesc->addr, epdesc->in ? "IN" : "OUT", epdesc->funcaddr,
        epdesc->xfrtype, epdesc->interval, epdesc->mxpacketsize);

  /* Allocate a endpoint information structure */

  epinfo = (struct sam_epinfo_s *)kzalloc(sizeof(struct sam_epinfo_s));
  if (!epinfo)
    {
      udbg("ERROR: Failed to allocate EP info structure\n");
      return -ENOMEM;
    }

  /* Initialize the endpoint container (which is really just another form of
   * 'struct usbhost_epdesc_s', packed differently and with additional
   * information.  A cleaner design might just embed struct usbhost_epdesc_s
   * inside of struct sam_epinfo_s and just memcpy here.
   */

  epinfo->epno      = epdesc->addr;
  epinfo->dirin     = epdesc->in;
  epinfo->devaddr   = epdesc->funcaddr;
#ifndef CONFIG_USBHOST_INT_DISABLE
  epinfo->interval  = epdesc->interval;
#endif
  epinfo->maxpacket = epdesc->mxpacketsize;
  epinfo->xfrtype   = epdesc->xfrtype;
  epinfo->speed     = rhport->ep0.speed;
  sem_init(&epinfo->iocsem, 0, 0);

  /* Success.. return an opaque reference to the endpoint information structure
   * instance
   */

  *ep = (usbhost_ep_t)epinfo;
  return OK;
}

/************************************************************************************
 * Name: sam_epfree
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

static int sam_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct sam_epinfo_s *epinfo = (struct sam_epinfo_s *)ep;

  /* There should not be any pending, transfers */

  DEBUGASSERT(drvr && epinfo && epinfo->iocwait == 0);

  /* Free the container */

  kfree(epinfo);
  return OK;
}

/*******************************************************************************
 * Name: sam_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data
 *   can be accessed more efficiently.  This method provides a mechanism to
 *   allocate the request/descriptor memory.  If the underlying hardware does
 *   not support such "special" memory, this functions may simply map to kmalloc.
 *
 *   This interface was optimized under a particular assumption.  It was
 *   assumed that the driver maintains a pool of small, pre-allocated buffers
 *   for descriptor traffic.  NOTE that size is not an input, but an output:
 *   The size of the pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call
 *      to the class create() method.
 *   buffer - The address of a memory location provided by the caller in which
 *      to return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in which
 *      to return the maximum size of the allocated buffer memory.
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

static int sam_alloc(FAR struct usbhost_driver_s *drvr,
                     FAR uint8_t **buffer, FAR size_t *maxlen)
{
  int ret = -ENOMEM;
  DEBUGASSERT(drvr && buffer && maxlen);

  /* There is no special requirements for transfer/descriptor buffers. */

  *buffer = (FAR uint8_t *)kmalloc(CONFIG_SAMA5_EHCI_BUFSIZE);
  if (*buffer)
    {
      *maxlen = CONFIG_SAMA5_EHCI_BUFSIZE;
      ret = OK;
    }

  return ret;
}

/*******************************************************************************
 * Name: sam_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data
 *   can be accessed more efficiently.  This method provides a mechanism to
 *   free that request/descriptor memory.  If the underlying hardware does not
 *   support such "special" memory, this functions may simply map to kfree().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call
 *      to the class create() method.
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

static int sam_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

  /* No special action is require to free the transfer/descriptor buffer memory */

  kfree(buffer);
  return OK;
}

/************************************************************************************
 * Name: sam_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to kumalloc.
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

static int sam_ioalloc(FAR struct usbhost_driver_s *drvr, FAR uint8_t **buffer,
                       size_t buflen)
{
  DEBUGASSERT(drvr && buffer && buflen > 0);

  /* The only special requirements for I/O buffers are they might need to be user
   * accessible (depending on how the class driver implements its buffering).
   */

  *buffer = (FAR uint8_t *)kumalloc(buflen);
  return *buffer ? OK : -ENOMEM;
}

/************************************************************************************
 * Name: sam_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed more
 *   efficiently.  This method provides a mechanism to free that IO buffer
 *   memory.  If the underlying hardware does not support such "special" memory,
 *   this functions may simply map to kufree().
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

static int sam_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

  /* No special action is require to free the I/O buffer memory */

  kufree(buffer);
  return OK;
}

/*******************************************************************************
 * Name: sam_ctrlin and sam_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one transfer may
 *   be queued; Neither these methods nor the transfer() method can be called
 *   again until the control transfer functions returns.
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
 *     in the request description. buffer must have been allocated using
 *     DRVR_ALLOC
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

static int sam_ctrlin(FAR struct usbhost_driver_s *drvr,
                      FAR const struct usb_ctrlreq_s *req,
                      FAR uint8_t *buffer)
{
  struct sam_rhport_s *rhport = (struct sam_rhport_s *)drvr;
  uint16_t len;
  ssize_t nbytes;

  DEBUGASSERT(rhport && req);

  len = sam_read16(req->len);
  uvdbg("RHPort%d type: %02x req: %02x value: %02x%02x index: %02x%02x len: %04x\n",
        rhport->rhpndx + 1, req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], len);

  /* We must have exclusive access to the EHCI hardware and data structures. */

  sam_takesem(&g_ehci.exclsem);

  /* Now perform the transfer */

  nbytes = sam_async_transfer(rhport, &rhport->ep0, req, buffer, len);
  sam_givesem(&g_ehci.exclsem);
  return nbytes >=0 ? OK : (int)nbytes;
}

static int sam_ctrlout(FAR struct usbhost_driver_s *drvr,
                       FAR const struct usb_ctrlreq_s *req,
                       FAR const uint8_t *buffer)
{
  /* sam_ctrlin can handle both directions.  We just need to work around the
   * differences in the function signatures.
   */

  return sam_ctrlin(drvr, req, (uint8_t *)buffer);
}

/*******************************************************************************
 * Name: sam_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  Only one transfer may be
 *   queued;.
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
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     EAGAIN - If devices NAKs the transfer (or NYET or other error where
 *              it may be appropriate to restart the entire transaction).
 *     EPERM  - If the endpoint stalls
 *     EIO    - On a TX or data toggle error
 *     EPIPE  - Overrun errors
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int sam_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                        FAR uint8_t *buffer, size_t buflen)
{
  struct sam_rhport_s *rhport = (struct sam_rhport_s *)drvr;
  struct sam_epinfo_s *epinfo = (struct sam_epinfo_s *)ep;
  ssize_t nbytes;

  DEBUGASSERT(rhport && epinfo && buffer && buflen > 0);

  /* We must have exclusive access to the EHCI hardware and data structures. */

  sam_takesem(&g_ehci.exclsem);

  /* Perform the transfer */

  switch (epinfo->xfrtype)
    {
      case USB_EP_ATTR_XFER_BULK:
        nbytes = sam_async_transfer(rhport, epinfo, NULL, buffer, buflen);
        break;

#ifndef CONFIG_USBHOST_INT_DISABLE
      case USB_EP_ATTR_XFER_INT:
# warning "Interrupt endpoint support not emplemented"
#endif
#ifndef CONFIG_USBHOST_ISOC_DISABLE
      case USB_EP_ATTR_XFER_ISOC:
# warning "Isochronous endpoint support not emplemented"
#endif
      case USB_EP_ATTR_XFER_CONTROL:
      default:
        udbg("ERROR: Support for transfer type %d not implemented\n");
        nbytes = -ENOSYS;
        break;
    }

  sam_givesem(&g_ehci.exclsem);
  return nbytes >=0 ? OK : (int)nbytes;
}

/*******************************************************************************
 * Name: sam_disconnect
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
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static void sam_disconnect(FAR struct usbhost_driver_s *drvr)
{
  struct sam_rhport_s *rhport = (struct sam_rhport_s *)drvr;
  DEBUGASSERT(rhport);

  /* Unbind the class */
  /* REVISIT:  Is there more that needs to be done? */

  rhport->class = NULL;
}

/*******************************************************************************
 * Initialization
 *******************************************************************************/
/*******************************************************************************
 * Name: sam_reset
 *
 * Description:
 *   Set the HCRESET bit in the USBCMD register to reset the EHCI hardware.
 *
 *   Table 2-9. USBCMD – USB Command Register Bit Definitions
 *
 *    "Host Controller Reset (HCRESET) ... This control bit is used by software
 *     to reset the host controller. The effects of this on Root Hub registers
 *     are similar to a Chip Hardware Reset.
 *
 *    "When software writes a one to this bit, the Host Controller resets its
 *     internal pipelines, timers, counters, state machines, etc. to their
 *     initial value. Any transaction currently in progress on USB is
 *     immediately terminated. A USB reset is not driven on downstream
 *     ports.
 *
 *    "PCI Configuration registers are not affected by this reset. All
 *     operational registers, including port registers and port state machines
 *     are set to their initial values. Port ownership reverts to the companion
 *     host controller(s)... Software must reinitialize the host controller ...
 *     in order to return the host controller to an operational state.
 *
 *    "This bit is set to zero by the Host Controller when the reset process is
 *     complete. Software cannot terminate the reset process early by writing a
 *     zero to this register. Software should not set this bit to a one when
 *     the HCHalted bit in the USBSTS register is a zero. Attempting to reset
 *     an actively running host controller will result in undefined behavior."
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   on failure.
 *
 * Assumptions:
 * - Called during the initializaation of the EHCI.
 *
 *******************************************************************************/

static int sam_reset(void)
{
  uint32_t regval;
  unsigned int timeout;

  /* "... Software should not set [HCRESET] to a one when the HCHalted bit in
   *  the USBSTS register is a zero. Attempting to reset an actively running
   *   host controller will result in undefined behavior."
   */

  sam_putreg(0, &HCOR->usbcmd);
  timeout = 0;
  do
    {
      /* Wait one microsecond and update the timeout counter */

      up_udelay(1);
      timeout++;

      /* Get the current valud of the USBSTS register.  This loop will terminate
       * when either the timeout exceeds one millisecond or when the HCHalted
       * bit is no longer set in the USBSTS register.
       */

      regval = sam_getreg(&HCOR->usbsts);
    }
  while (((regval & EHCI_USBSTS_HALTED) == 0) && (timeout < 1000));

  /* Is the EHCI still running?  Did we timeout? */

  if ((regval & EHCI_USBSTS_HALTED) == 0)
    {
      udbg("ERROR: Timed out waiting for HCHalted. USBSTS: %08x", regval);
      return -ETIMEDOUT;
    }

  /* Now we can set the HCReset bit in the USBCMD register to initiate the reset */

  regval  = sam_getreg(&HCOR->usbcmd);
  regval |= EHCI_USBCMD_HCRESET;
  sam_putreg(regval, &HCOR->usbcmd);

  /* Wait for the HCReset bit to become clear */

  do
    {
      /* Wait five microsecondw and update the timeout counter */

      up_udelay(5);
      timeout += 5;

      /* Get the current valud of the USBCMD register.  This loop will terminate
       * when either the timeout exceeds one second or when the HCReset
       * bit is no longer set in the USBSTS register.
       */

      regval = sam_getreg(&HCOR->usbcmd);
    }
  while (((regval & EHCI_USBCMD_HCRESET) != 0) && (timeout < 1000000));

  /* Return either success or a timeout */

  return (regval & EHCI_USBCMD_HCRESET) != 0 ? -ETIMEDOUT : OK;
}

/*******************************************************************************
 * Global Functions
 *******************************************************************************/
/*******************************************************************************
 * Name: sam_ehci_initialize
 *
 * Description:
 *   Initialize USB EHCI host controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than one EHCI interface, then
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

FAR struct usbhost_connection_s *sam_ehci_initialize(int controller)
{
  irqstate_t flags;
  uint32_t regval;
#ifdef CONFIG_DEBUG_USB
  uint16_t regval16;
  unsigned int nports;
#endif
  uintptr_t physaddr;
  int ret;
  int i;

  /* Sanity checks */

  DEBUGASSERT(controller == 0);
  DEBUGASSERT(((uintptr_t)&g_asynchead & 0x1f) == 0);
  DEBUGASSERT(((uintptr_t)&g_qhpool & 0x1f) == 0);
  DEBUGASSERT(((uintptr_t)&g_qtdpool & 0x1f) == 0);
  DEBUGASSERT((sizeof(struct sam_qh_s) & 0x1f) == 0);
  DEBUGASSERT((sizeof(struct sam_qtd_s) & 0x1f) == 0);

#ifndef CONFIG_USBHOST_INT_DISABLE
  DEBUGASSERT(((uintptr_t)&g_perhead & 0x1f) == 0);
  DEBUGASSERT(((uintptr_t)g_framelist & 0xfff) == 0);
#endif

  /* SAMA5 Configuration *******************************************************/
  /* For High-speed operations, the user has to perform the following:
   *
   *   1) Enable UHP peripheral clock, bit (1 << AT91C_ID_UHPHS) in
   *      PMC_PCER register.
   *   2) Write CKGR_PLLCOUNT field in PMC_UCKR register.
   *   3) Enable UPLL, bit AT91C_CKGR_UPLLEN in PMC_UCKR register.
   *   4) Wait until UTMI_PLL is locked. LOCKU bit in PMC_SR register
   *   5) Enable BIAS, bit AT91C_CKGR_BIASEN in PMC_UCKR register.
   *   6) Select UPLLCK as Input clock of OHCI part, USBS bit in PMC_USB
   *      register.
   *   7) Program the OHCI clocks (UHP48M and UHP12M) with USBDIV field in
   *      PMC_USB register. USBDIV must be 9 (division by 10) if UPLLCK is
   *      selected.
   *   8) Enable OHCI clocks, UHP bit in PMC_SCER register.
   *
   * Steps 1 and 8 are performed here.  Steps 2 through 7 were are performed
   * by sam_clockconfig() earlier in the boot sequence.
   */

  /* Enable UHP peripheral clocking */

  flags = irqsave();
  sam_uhphs_enableclk();

  /* Enable OHCI clocks */

  regval = sam_getreg((volatile uint32_t *)SAM_PMC_SCER);
  regval |= PMC_UHP;
  sam_putreg(regval, (volatile uint32_t *)SAM_PMC_SCER);

  /* "One transceiver is shared with the USB High Speed Device (port A). The
   *  selection between Host Port A and USB Device is controlled by the UDPHS
   *  enable bit (EN_UDPHS) located in the UDPHS_CTRL control register."
   *
   * Make all three ports usable for EHCI unless the high speed device is
   * enabled; then let the device manage port zero.  Zero is the reset
   * value for all ports; one makes the corresponding port available to OHCI.
   */

  regval  = getreg32(SAM_SFR_OHCIICR);
#ifdef CONFIG_SAMA5_UHPHS_RHPORT1
  regval &= ~SFR_OHCIICR_RES0;
#endif
#ifdef CONFIG_SAMA5_UHPHS_RHPORT2
  regval &= ~SFR_OHCIICR_RES1;
#endif
#ifdef CONFIG_SAMA5_UHPHS_RHPORT3
  regval &= ~SFR_OHCIICR_RES2;
#endif
  putreg32(regval, SAM_SFR_OHCIICR);
  irqrestore(flags);

  /* Note that no pin configuration is required.  All USB HS pins have
   * dedicated function
   */

  /* Software Configuration ****************************************************/

  uvdbg("Initializing EHCI Stack\n");

  /* Initialize the EHCI state data structure */

  sem_init(&g_ehci.exclsem, 0, 1);
  sem_init(&g_ehci.pscsem,  0, 0);

  /* Initialize EP0 */

  sem_init(&g_ehci.ep0.iocsem, 0, 1);

  /* Initialize the root hub port structures */

  for (i = 0; i < SAM_EHCI_NRHPORT; i++)
    {
      struct sam_rhport_s *rhport = &g_ehci.rhport[i];
      rhport->rhpndx              = i;

      /* Initialize the device operations */

      rhport->drvr.ep0configure   = sam_ep0configure;
      rhport->drvr.epalloc        = sam_epalloc;
      rhport->drvr.epfree         = sam_epfree;
      rhport->drvr.alloc          = sam_alloc;
      rhport->drvr.free           = sam_free;
      rhport->drvr.ioalloc        = sam_ioalloc;
      rhport->drvr.iofree         = sam_iofree;
      rhport->drvr.ctrlin         = sam_ctrlin;
      rhport->drvr.ctrlout        = sam_ctrlout;
      rhport->drvr.transfer       = sam_transfer;
      rhport->drvr.disconnect     = sam_disconnect;

      /* Initialize EP0 */

      rhport->ep0.xfrtype         = USB_EP_ATTR_XFER_CONTROL;
      rhport->ep0.speed           = EHCI_FULL_SPEED;
      rhport->ep0.maxpacket       = 8;
      sem_init(&rhport->ep0.iocsem, 0, 0);
    }

  /* Initialize the list of free Queue Head (QH) structures */

  for (i = 0; i < CONFIG_SAMA5_EHCI_NQHS; i++)
    {
      /* Put the QH structure in a free list */

      sam_qh_free(&g_qhpool[i]);
    }

  /* Initialize the list of free Queue Head (QH) structures */

  for (i = 0; i < CONFIG_SAMA5_EHCI_NQTDS; i++)
    {
      /* Put the TD in a free list */

      sam_qtd_free(&g_qtdpool[i]);
    }

  /* EHCI Hardware Configuration ***********************************************/
  /* Host Controller Initialization. Paragraph 4.1 */
  /* Reset the EHCI hardware */

  ret = sam_reset();
  if (ret < 0)
    {
      udbg("ERROR: sam_reset failed: %d\n", ret);
      return NULL;
    }

  /* "In order to initialize the host controller, software should perform the
   *  following steps:
   *
   *  • "Program the CTRLDSSEGMENT register with 4-Gigabyte segment where all
   *     of the interface data structures are allocated. [64-bit mode]
   *  • "Write the appropriate value to the USBINTR register to enable the
   *     appropriate interrupts.
   *  • "Write the base address of the Periodic Frame List to the PERIODICLIST
   *     BASE register. If there are no work items in the periodic schedule,
   *     all elements of the Periodic Frame List should have their T-Bits set
   *     to a one.
   *  • "Write the USBCMD register to set the desired interrupt threshold,
   *     frame list size (if applicable) and turn the host controller ON via
   *     setting the Run/Stop bit.
   *  •  Write a 1 to CONFIGFLAG register to route all ports to the EHCI controller
   *     ...
   *
   * "At this point, the host controller is up and running and the port registers
   *  will begin reporting device connects, etc. System software can enumerate a
   *  port through the reset process (where the port is in the enabled state). At
   *  this point, the port is active with SOFs occurring down the enabled por
   *  enabled Highspeed ports, but the schedules have not yet been enabled. The
   *  EHCI Host controller will not transmit SOFs to enabled Full- or Low-speed
   *  ports.
   */

  /* Disable all interrupts */

  sam_putreg(0, &HCOR->usbintr);

  /* Clear pending interrupts.  Bits in the USBSTS register are cleared by
   * writing a '1' to the corresponding bit.
   */

  sam_putreg(ECHI_INT_ALLINTS, &HCOR->usbsts);

#ifdef CONFIG_DEBUG
  /* Show the ECHI version */

  regval16 = sam_swap16(HCCR->hciversion);
  uvdbg("HCIVERSION %x.%02x\n", regval16 >> 8, regval16 & 0xff);

  /* Verify the the correct number of ports is reported */

  regval = sam_getreg(&HCCR->hcsparams);
  nports = (regval & EHCI_HCSPARAMS_NPORTS_MASK) >> EHCI_HCSPARAMS_NPORTS_SHIFT;

  uvdbg("HCSPARAMS=%08x nports=%d\n", regval, nports);
  DEBUGASSERT(nports == SAM_EHCI_NRHPORT);

  /* Show the HCCPARAMS register */

  regval = sam_getreg(&HCCR->hccparams);
  uvdbg("HCCPARAMS=%08x\n", regval);
#endif

  /* Initialize the head of the asynchronous queue/reclamation list.
   *
   * "In order to communicate with devices via the asynchronous schedule,
   *  system software must write the ASYNDLISTADDR register with the address
   *  of a control or bulk queue head. Software must then enable the
   *  asynchronous schedule by writing a one to the Asynchronous Schedule
   *  Enable bit in the USBCMD register. In order to communicate with devices
   *  via the periodic schedule, system software must enable the periodic
   *  schedule by writing a one to the Periodic Schedule Enable bit in the
   *  USBCMD register. Note that the schedules can be turned on before the
   *  first port is reset (and enabled)."
   */

  memset(&g_asynchead, 0, sizeof(struct sam_qh_s));
  physaddr                     = sam_physramaddr((uintptr_t)&g_asynchead);
  g_asynchead.hw.hlp           = sam_swap32(physaddr | QH_HLP_TYP_QH);
  g_asynchead.hw.epchar        = sam_swap32(QH_EPCHAR_H | QH_EPCHAR_EPS_FULL);
  g_asynchead.hw.overlay.nqp   = sam_swap32(QH_NQP_T);
  g_asynchead.hw.overlay.alt   = sam_swap32(QH_NQP_T);
  g_asynchead.hw.overlay.token = sam_swap32(QH_TOKEN_HALTED);
  g_asynchead.fqp              = sam_swap32(QTD_NQP_T);

  cp15_coherent_dcache((uintptr_t)&g_asynchead.hw,
                       (uintptr_t)&g_asynchead.hw + sizeof(struct ehci_qh_s));

  /* Set the Current Asynchronous List Address. */

  sam_putreg(sam_swap32(physaddr), &HCOR->asynclistaddr);

#ifndef CONFIG_USBHOST_INT_DISABLE
  /* Initialize the head of the periodic list */

  memset(&g_perhead, 0, sizeof(struct sam_qh_s));
  g_perhead.hw.hlp           = sam_swap32(QH_HLP_T);
  g_perhead.hw.overlay.nqp   = sam_swap32(QH_NQP_T);
  g_perhead.hw.overlay.alt   = sam_swap32(QH_NQP_T);
  g_perhead.hw.overlay.token = sam_swap32(QH_TOKEN_HALTED);
  g_perhead.hw.epcaps        = sam_swap32(QH_EPCAPS_SSMASK(1));

  /* Attach the periodic QH to Period Frame List */

  physaddr = sam_physramaddr((uintptr_t)&g_perhead);
  for (i = 0; i < FRAME_LIST_SIZE; i++)
    {
      g_framelist[i] = sam_swap32(physaddr) | PFL_TYP_QH;
    }

  /* Set the Periodic Frame List Base Address. */

  cp15_coherent_dcache((uintptr_t)&g_perhead.hw,
                       (uintptr_t)&g_perhead.hw + sizeof(struct ehci_qh_s));
  cp15_coherent_dcache((uintptr_t)g_framelist,
                       (uintptr_t)g_framelist + FRAME_LIST_SIZE * sizeof(uint32_t));

  sam_putreg(sam_swap32(physaddr), &HCOR->periodiclistbase);
#endif

  /* Enable the asynchronous schedule and, possibly set the frame list size */

  regval  = sam_getreg(&HCOR->usbcmd);
  regval &= ~(EHCI_USBCMD_HCRESET | EHCI_USBCMD_FLSIZE_MASK |
              EHCI_USBCMD_FLSIZE_MASK | EHCI_USBCMD_PSEN |
              EHCI_USBCMD_IAADB | EHCI_USBCMD_LRESET);
  regval |= EHCI_USBCMD_ASEN;

#ifndef CONFIG_USBHOST_INT_DISABLE
#  if FRAME_LIST_SIZE == 1024
  regval |= EHCI_USBCMD_FLSIZE_1024;
#  elif FRAME_LIST_SIZE == 512
  regval |= EHCI_USBCMD_FLSIZE_512;
#  elif FRAME_LIST_SIZE == 512
  regval |= EHCI_USBCMD_FLSIZE_256;
#  else
#    error Unsupported frame size list size
#  endif
#endif

  sam_putreg(regval, &HCOR->usbcmd);

  /* Start the host controller by setting the RUN bit in the USBCMD regsiter. */

  regval  = sam_getreg(&HCOR->usbcmd);
  regval |= EHCI_USBCMD_RUN;
  sam_putreg(regval, &HCOR->usbcmd);

  /* Route all ports to this host controller by setting the CONFIG flag. */

  regval  = sam_getreg(&HCOR->configflag);
  regval |= EHCI_CONFIGFLAG;
  sam_putreg(regval, &HCOR->configflag);

  /* Wait for the ECHI to run (i.e., not longer report halted) */

  ret = ehci_wait_usbsts(EHCI_USBSTS_HALTED, 0, 100*1000);
  if (ret < 0)
    {
      udbg("ERROR: EHCI Failed to run: USBSTS=%08x\n",
           sam_getreg(&HCOR->usbsts));

      return NULL;
    }

  /* Interrupt Configuration ***************************************************/
  /* Attach USB host controller interrupt handler.  If OHCI is also enabled,
   * then we have to use a common UHPHS interrupt handler.
   */

#ifdef CONFIG_SAMA5_OHCI
  ret = irq_attach(SAM_IRQ_UHPHS, sam_uhphs_interrupt);
#else
  ret = irq_attach(SAM_IRQ_UHPHS, sam_ehci_tophalf);
#endif
  if (ret != 0)
    {
      udbg("ERROR: Failed to attach IRQ\n");
      return NULL;
    }

  /* Enable EHCI interrupts.  Interrupts are still disabled at the level of
   * the AIC.
   */

  sam_putreg(EHCI_HANDLED_INTS, &HCOR->usbintr);

  /* Drive Vbus +5V (the smoke test)
   *
   * REVISIT:
   * - Should be done elsewhere in OTG mode.
   * - Can we postpone enabling VBUS to save power?  I think it can be
   *   done in sam_enumerate() and can probably be disabled when the
   *   port is disconnected.
   * - Some EHCI implementations require setting the power bit in the
   *   PORTSC register to enable power.
   */

#ifdef CONFIG_SAMA5_UHPHS_RHPORT1
  sam_usbhost_vbusdrive(SAM_RHPORT1, true);
#endif
#ifdef CONFIG_SAMA5_UHPHS_RHPORT2
  sam_usbhost_vbusdrive(SAM_RHPORT2, true);
#endif
#ifdef CONFIG_SAMA5_UHPHS_RHPORT3
  sam_usbhost_vbusdrive(SAM_RHPORT3, true);
#endif
  up_mdelay(50);

  /* If there is a USB device in the slot at power up, then we will not
   * get the status change interrupt to signal us that the device is
   * connected.  We need to set the initial connected state accordingly.
   */

  for (i = 0; i < SAM_EHCI_NRHPORT; i++)
    {
      g_ehci.rhport[i].connected =
        ((sam_getreg(&HCOR->portsc[i]) & EHCI_PORTSC_CCS) != 0);
    }

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(SAM_IRQ_UHPHS); /* enable USB interrupt */
  uvdbg("USB EHCI Initialized\n");

  /* Initialize and return the connection interface */

  g_ehciconn.wait      = sam_wait;
  g_ehciconn.enumerate = sam_enumerate;
  return &g_ehciconn;
}

#endif /* CONFIG_SAMA5_EHCI */
