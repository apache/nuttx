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
#define CONFIG_USBHOST_INT_DISABLE 1

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
  uint32_t pad[12];            /* Padding to assure 32-byte alignment */
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
  uint8_t xfrtype:2;           /* See USB_EP_ATTR_XFER_* definitions in usb.h */
  uint8_t speed:2;             /* See USB_*_SPEED definitions in ehci.h */
  uint8_t status;              /* Retained token status bits (for debug purposes) */
  volatile bool iocwait;       /* TRUE: Thread is waiting for transfer completion */
  uint16_t maxpacket;          /* Maximum packet size */
  int result;                  /* The result of the transfer */
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
static void sam_write16(uint16_t memval, uint8_t *addr);
static void sam_write32(uint32_t memval, uint8_t *addr);

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

static int sam_qtd_invalidate(struct sam_qtd_s *qtd, uint32_t **bp, void *arg);
static int sam_qh_invalidate(struct sam_qh_s *qh);
static int sam_qtd_flush(struct sam_qtd_s *qtd, uint32_t **bp, void *arg);
static int sam_qh_flush(struct sam_qh_s *qh);

/* Endpoint Transfer Handling **************************************************/

static int sam_ioc_setup(struct sam_rhport_s *rhport, struct sam_epinfo_s *epinfo);
static int sam_ioc_wait(struct sam_epinfo_s *epinfo);
static void sam_qh_enqueue(struct sam_qh_s *qh);
static int sam_async_transfer(struct sam_rhport_s *rhport,
         struct sam_epinfo_s *epinfo, const struct usb_ctrlreq_s *req,
         uint8_t *buffer, size_t buflen);

/* Interrupt Handling **********************************************************/

static int sam_qtd_ioccheck(struct sam_qtd_s *qtd, uint32_t **bp, void *arg);
static int sam_qh_ioccheck(struct sam_qh_s *qh, uint32_t **bp, void *arg);
static inline void sam_ioc_bottomhalf(void);
static inline void sam_err_bottomhalf(void);
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

/*******************************************************************************
 * Name: sam_write32
 *
 * Description:
 *   Write 32-bit little endian data
 *
 *******************************************************************************/

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
          udbg("ERROR: System error: 0x%08X", regval);
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

      if ((sam_swap32(qh->hw.hlp) & QH_HLP_T) != 0)
        {
          /* Set the next pointer to NULL.  This will terminate the loop. */

          next = NULL;
        }
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
 *******************************************************************************/

static int sam_qh_forall(foreach_qh_t handler, void *arg)
{
  struct sam_qh_s *qh;
  uint32_t *bp;
  int ret;

  /* Preemption is disabled to prevent concurrent modification of the queue
   * head by the other threads.
   */

  bp = (uint32_t *)&qh->hw.hlp;

  sched_lock();
  qh = (struct sam_qh_s *)sam_virtramaddr(sam_swap32(*bp) & QH_HLP_MASK);
  sam_qh_foreach(qh, &bp, handler, arg);
  sched_unlock();

  return ret;
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

  bp = &qh->hw.overlay.nqp;
  if ((*bp & QH_NQP_T) != 0)
    {
      return 0;
    }

  /* Start with the first qTD in the queue */

  physaddr = sam_swap32(*bp);
  qtd      = (struct sam_qtd_s *)sam_virtramaddr(physaddr);
  next     = NULL;

  /* Now loop until we encounter the end of the qTD list */

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

static int sam_qtd_invalidate(struct sam_qtd_s *qtd, uint32_t **bp, void *arg)
{
  /* Invalidate the D-Cache, i.e., force reloading of the D-Cache from memory
   * memory over the specified address range.
   */

  cp15_invalidate_dcache((uintptr_t)&qtd->hw,
                         (uintptr_t)&qtd->hw + sizeof(struct ehci_qtd_s));
  return OK;
}

/*******************************************************************************
 * Name: sam_qh_invalidate
 *
 * Description:
 *   Invalidate the Queue Head and all qTD entries in the queue.
 *
 *******************************************************************************/

static int sam_qh_invalidate(struct sam_qh_s *qh)
{
  /* Invalidate the QH first so that we reload the qTD list head */

  cp15_invalidate_dcache((uintptr_t)&qh->hw,
                         (uintptr_t)&qh->hw + sizeof(struct ehci_qh_s));

  /* Then invalidate all of the qTD entries in the queue */

  return sam_qtd_foreach(qh, NULL, NULL);
}

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

  cp15_invalidate_dcache((uintptr_t)&qh->hw,
                         (uintptr_t)&qh->hw + sizeof(struct ehci_qh_s));

  /* Then flush all of the qTD entries in the queue */

  return sam_qtd_foreach(qh, NULL, NULL);
}

/*******************************************************************************
 * Endpoint Transfer Handling
 *******************************************************************************/

/*******************************************************************************
 * Name: sam_ioc_setup
 *
 * Description:
 *   Set the request for the IOC event well BEFORE enabling the transfer (as
 *   soon as we are absolutely committed to the to avoid transfer).  We do this
 *   to minimize race conditions.  This logic would have to be expanded if we
 *   want to have more than one packet in flight at a time!
 *
 *******************************************************************************/

static int sam_ioc_setup(struct sam_rhport_s *rhport, struct sam_epinfo_s *epinfo)
{
  irqstate_t flags;
  int ret = -ENODEV;

  DEBUGASSERT(rhport && epinfo);

  /* Is the device still connected? */

  flags = irqsave();
  if (rhport->connected)
    {
      /* Then set wdhwait to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer
       * completed.
       */

      epinfo->iocwait = true;  /* We want to be awakened by IOC interrupt */
      epinfo->status  = 0;     /* No status yet */
      epinfo->result  = -EBUSY; /* Transfer in progress */
      ret             = OK;    /* We are good to go */
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
 *******************************************************************************/

static void sam_qh_enqueue(struct sam_qh_s *qh)
{
  uintptr_t physaddr;

  /* Add the new QH to the head of the asynchronous queue list.  Preemption
   * is disabled momentarily to prevent concurrent modification of the queue
   * head by the worker thread.
   */

  physaddr = (uintptr_t)sam_physramaddr((uintptr_t)qh);
  sched_lock();

  /* Attach the old head as the new QH HLP and flush the new QH and its attached
   * qTDs to RAM.
   */

  qh->hw.hlp = g_asynchead.hw.hlp;
  sam_qh_flush(qh);

  /* Set the new QH as the first QH in the asychronous queue and flush the
   * modified head to RAM.
   */

  g_asynchead.hw.hlp = sam_swap32(physaddr | QH_HLP_TYP_QH);
  cp15_coherent_dcache((uintptr_t)&g_asynchead,
                       (uintptr_t)&g_asynchead + sizeof(struct ehci_qh_s));
  sched_unlock();
}

/*******************************************************************************
 * Name: sam_async_transfer
 *
 * Description:
 *   Process a IN or OUT request on any asynchronous endpoint (bulk or control).
 *   This function will enqueue the request and wait for it to complete.
 *
 *   This is a blocking function; it will not return until the control transfer
 *   has completed.
 *
 *******************************************************************************/

static int sam_async_transfer(struct sam_rhport_s *rhport,
                              struct sam_epinfo_s *epinfo,
                              const struct usb_ctrlreq_s *req,
                              uint8_t *buffer, size_t buflen)
{
  int ret;

  uvdbg("RHport%d EP%d: buffer=%p, buflen=%d, req=%p\n",
        rhport->rhpndx+1, epinfo->epno, buffer, buflen, req);

  if (req != NULL)
    {
      uvdbg("req=%02x type=%02x value=%04x index=%04x\n",
            req->req, req->type, sam_read16(req->value),
            sam_read16(req->index));
    }

  /* Set the request for the IOC event well BEFORE enabling the transfer. */

  ret = sam_ioc_setup(rhport, epinfo);
  if (ret != OK)
    {
      udbg("ERROR: Device disconnected\n");
      return ret;
    }

#warning "Missing logic"

  /* Wait for the IOC completion event */

  ret = sam_ioc_wait(epinfo);
  if (ret < 0)
    {
      udbg("ERROR: Transfer failed\n");
      goto errout_with_iocwait;
    }

  /* Transfer completed successfully */

  return OK;

errout_with_iocwait:
  epinfo->iocwait = false;
  return ret;
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
  DEBUGASSERT(qtd);

  /* Make sure we reload the QH from memory */

  cp15_invalidate_dcache((uintptr_t)&qtd->hw,
                         (uintptr_t)&qtd->hw + sizeof(struct ehci_qtd_s));

  /* Remove the qTD from the list */

  **bp = qtd->hw.nqp;

  /* NOTE that we don't check if the qTD is active nor do we check if there
   * are any errors reported in the qTD.  If the transfer halted due to
   * an error, then I am not sure if we can believe this information anyway.
   * The only sure place to check for errors in in the QH overlay.
   */

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

  /* Get the endpoint info pointer from the extended QH data */

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

  ret = sam_qtd_foreach(qh, sam_qtd_ioccheck, NULL);
  if (ret < 0)
    {
      udbg("ERROR: sam_qh_forall failed: %d\n", ret);
    }

  /* If there is no longer anything attached to the QH, then remove it from
   * the asynchronous queue.
   */

  if ((sam_swap32(qh->hw.overlay.nqp) & QH_NQP_T) != 0)
    {
      /* Set the forward link of the previous QH to point to the next
       * QH in the list.
       */

      **bp = qh->hw.overlay.nqp;

      /* Check for errors, update the data toggle */

      if ((token & QH_TOKEN_ERRORS) == 0)
        {
          /* No errors.. Save the last data toggle value */

          epinfo->toggle = ((token & QTD_TOKEN_TOGGLE) != 0);

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

      *bp = &qh->hw.overlay.nqp;
    }
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

  uvdbg("USB Interrupt (USBINT) Interrupt\n");

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
 * Name: sam_err_bottomhalf
 *
 * Description:
 *   EHCI USB Error Interrupt (USBERRINT) "Bottom Half" interrupt handler
 *
 *  "The Host Controller sets this bit to 1 when completion of a USB transaction
 *   results in an error condition (e.g., error counter underflow). If the TD on
 *   which the error interrupt occurred also had its IOC bit set, both this bit
 *   and USBINT bit are set. ..."
 *
 *******************************************************************************/

static inline void sam_err_bottomhalf(void)
{
  udbg("USB Error Interrupt (USBERRINT) Interrupt\n");

  /* Remove all queued transfers */
#warning Missing logic
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
                  udbg("Already connected\n");
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
                   udbg("Already disconnected\n");
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
  udbg("Async Advance Interrupt\n");

  /* Remove all tagged QH entries */
#warning Missing logic
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
   */

  if ((pending & EHCI_INT_USBINT) != 0)
    {
      sam_ioc_bottomhalf();
    }

  /* USB Error Interrupt (USBERRINT)
   *
   *  "The Host Controller sets this bit to 1 when completion of a USB
   *   transaction results in an error condition (e.g., error counter
   *   underflow). If the TD on which the error interrupt occurred also
   *   had its IOC bit set, both this bit and USBINT bit are set. ..."
   */

  if ((pending & EHCI_INT_USBERRINT) != 0)
    {
      sam_err_bottomhalf();
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

              udbg("RHPort%d connected: %s\n",
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

  DEBUGASSERT(rhpndx >= 0 && rhpndx < SAM_EHCI_NRHPORT);
  rhport = &g_ehci.rhport[rhpndx];

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!rhport->connected)
    {
      /* No, return an error */

      udbg("Not connected\n");
      return -ENODEV;
    }

  /* Add EP0 to the control list */
#warning Missing logic

  /* USB 2.0 spec says at least 50ms delay before port reset */

  up_mdelay(100);

  /* Put the root hub port in reset (the SAMA5 supports three downstream ports) */
#warning Missing logic

  /* Wait for the port reset to complete */
#warning Missing logic

  /* Release RH port 1 from reset and wait a bit */
#warning Missing logic

  up_mdelay(200);

  /* Let the common usbhost_enumerate do all of the real work.  Note that the
   * FunctionAddress (USB address) is set to the root hub port number for now.
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

  DEBUGASSERT(rhport &&
              funcaddr >= 0 && funcaddr <= SAM_EHCI_NRHPORT &&
              maxpacketsize < 2048);

  /* We must have exclusive access to the EHCI data structures. */

  sam_takesem(&g_ehci.exclsem);

#warning Missing logic

  sam_givesem(&g_ehci.exclsem);
  return -ENOSYS;
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
  int ret  = -ENOMEM;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(rhport && epdesc && ep && rhport->connected);

  /* Allocate a container for the endpoint data */

  epinfo = (struct sam_epinfo_s *)kzalloc(sizeof(struct sam_epinfo_s));
  if (!epinfo)
    {
      udbg("ERROR: Failed to allocate EP info structure\n");
      goto errout;
    }

  /* Initialize the endpoint container */

  sem_init(&epinfo->iocsem, 0, 0);

  /* We must have exclusive access to the EHCI data structures. */

  sam_takesem(&g_ehci.exclsem);

#warning Missing logic

  /* Success.. return an opaque reference to the endpoint list container */

  *ep = (usbhost_ep_t)epinfo;
  sam_givesem(&g_ehci.exclsem);
  return OK;

errout_with_semaphore:
  sam_givesem(&g_ehci.exclsem);
  kfree(epinfo);
errout:
  return ret;
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
  struct sam_rhport_s *rhport = (struct sam_rhport_s *)drvr;
  struct sam_epinfo_s *epinfo = (struct sam_epinfo_s *)ep;
  int ret;

  DEBUGASSERT(rhport && epinfo);

  /* There should not be any pending, transfers */
#warning Missing logic

  /* We must have exclusive access to the EHCI data structures. */

  sam_takesem(&g_ehci.exclsem);

#warning Missing logic
  ret = -ENOSYS;

  /* And free the container */

  sem_destroy(&epinfo->iocsem);
  kfree(epinfo);
  sam_givesem(&g_ehci.exclsem);
  return ret;
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
  int  ret;

  DEBUGASSERT(rhport && req);

  len = sam_read16(req->len);
  uvdbg("RHPort%d type: %02x req: %02x value: %02x%02x index: %02x%02x len: %04x\n",
        rhport->rhpndx + 1, req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], len);

  /* We must have exclusive access to the EHCI hardware and data structures. */

  sam_takesem(&g_ehci.exclsem);

  /* Now perform the transfer */

  ret = sam_async_transfer(rhport, &rhport->ep0, req, buffer, len);
  sam_givesem(&g_ehci.exclsem);
  return ret;
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
  int ret;

  DEBUGASSERT(rhport && epinfo && buffer && buflen > 0);
#warning Missing logic

  /* We must have exclusive access to the EHCI hardware and data structures. */

  sam_takesem(&g_ehci.exclsem);

  /* Perform the transfer */

  switch (epinfo->xfrtype)
    {
      case USB_EP_ATTR_XFER_BULK:
        ret = sam_async_transfer(rhport, epinfo, NULL, buffer, buflen);
        break;

      default:
      case USB_EP_ATTR_XFER_CONTROL:
      case USB_EP_ATTR_XFER_ISOC:
      case USB_EP_ATTR_XFER_INT:
        udbg("ERROR: Support for transfer type %d not implemented\n");
        ret = -ENOSYS;
        break;
    }

  sam_givesem(&g_ehci.exclsem);
  return ret;
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

  /* Remove the disconnected port */
#warning Missing logic

  /* Unbind the class */

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
      udbg("ERROR: Timed out waiting for HCHalted.  USBSTS: %08X", regval);
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
  irqrestore(flags);

  /* Note that no pin pinconfiguration is required.  All USB HS pins have
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
  uvdbg("HCIVERSIONI %x.%02x", regval16 >> 8, regval16 & 0xff);

  /* Verify the the correct number of ports is reported */

  regval = sam_getreg(&HCCR->hcsparams);
  nports = (regval & EHCI_HCSPARAMS_NPORTS_MASK) >> EHCI_HCSPARAMS_NPORTS_SHIFT;

  uvdbg("HCSPARAMS=%08x nports=%d", regval, nports);
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
  /* Attach USB host controller interrupt handler */

  if (irq_attach(SAM_IRQ_UHPHS, sam_ehci_tophalf) != 0)
    {
      udbg("ERROR: Failed to attach IRQ\n");
      return NULL;
    }

  /* Enable EHCI interrupts.  Interrupts are still disabled at the level of
   * the AIC.
   */

  sam_putreg(EHCI_HANDLED_INTS, &HCOR->usbintr);

  /* Drive Vbus +5V (the smoke test).  Should be done elsewhere in OTG
   * mode.
   */

  sam_usbhost_vbusdrive(SAM_EHCI_IFACE, true);
  up_mdelay(50);

  /* If there is a USB device in the slot at power up, then we will not
   * get the status change interrupt to signal us that the device is
   * connected.  We need to set the initial connected state accordingly.
   */

  for (i = 0; i < SAM_EHCI_NRHPORT; i++)
    {
#warning Missing logic
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
