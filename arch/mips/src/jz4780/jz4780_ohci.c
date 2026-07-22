/****************************************************************************
 * arch/mips/src/jz4780/jz4780_ohci.c
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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/ohci.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/usbhost_trace.h>

#include <nuttx/irq.h>

#include <arch/board/board.h> /* May redefine PIO settings */

#include "mips_internal.h"
#include "chip.h"
#include "jz_usbhost.h"
#include "jz4780_ohci.h"

#ifdef CONFIG_JZ4780_OHCI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Pre-requisites */

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

/* Configurable number of user endpoint descriptors (EDs).  This number
 * excludes the control endpoint that is always allocated.
 */

#ifndef CONFIG_JZ4780_OHCI_NEDS
#  define CONFIG_JZ4780_OHCI_NEDS 2
#endif

/* Configurable number of user transfer descriptors (TDs).  */

#ifndef CONFIG_JZ4780_OHCI_NTDS
#  define CONFIG_JZ4780_OHCI_NTDS 3
#endif

#if CONFIG_JZ4780_OHCI_NTDS < 2
#  error Insufficient number of transfer descriptors (CONFIG_JZ4780_OHCI_NTDS < 2)
#endif

/* Minimum alignment for DMA access is 16 bytes, but it is safer to align to
 * the cache line size.
 */

#define JZ4780_DMA_ALIGN JZ4780_DCACHE_LINESIZE

/* Configurable number of request/descriptor buffers (TDBUFFER) */

#ifndef CONFIG_JZ4780_OHCI_TDBUFFERS
#  define CONFIG_JZ4780_OHCI_TDBUFFERS 2
#endif

#if CONFIG_JZ4780_OHCI_TDBUFFERS < 2
#  error At least two TD buffers are required (CONFIG_JZ4780_OHCI_TDBUFFERS < 2)
#endif

/* Configurable size of one TD buffer */

#if CONFIG_JZ4780_OHCI_TDBUFFERS > 0 && !defined(CONFIG_JZ4780_OHCI_TDBUFSIZE)
#  define CONFIG_JZ4780_OHCI_TDBUFSIZE 128
#endif

#if (CONFIG_JZ4780_OHCI_TDBUFSIZE & 3) != 0
#  error "TD buffer size must be an even number of 32-bit words"
#endif

/* Total buffer size */

#define JZ_BUFALLOC (CONFIG_JZ4780_OHCI_TDBUFFERS * CONFIG_JZ4780_OHCI_TDBUFSIZE)

/* Debug */

#ifndef CONFIG_DEBUG_USB_INFO
#  undef CONFIG_JZ4780_OHCI_REGDEBUG
#endif

/* OHCI Setup ***************************************************************/

/* Frame Interval / Periodic Start.
 *
 * At 12Mbps, there are 12000 bit time in each 1Msec frame.
 */

#define BITS_PER_FRAME          12000
#define FI                     (BITS_PER_FRAME-1)
#define FSMPS                  ((6 * (FI - 210)) / 7)
#define DEFAULT_FMINTERVAL     ((FSMPS << OHCI_FMINT_FSMPS_SHIFT) | FI)
#define DEFAULT_PERSTART       (((9 * BITS_PER_FRAME) / 10) - 1)

/* CLKCTRL enable bits */

#define JZ_CLKCTRL_ENABLES   (USBOTG_CLK_HOSTCLK|USBOTG_CLK_PORTSELCLK|USBOTG_CLK_AHBCLK)

/* Interrupt enable bits */

#ifdef CONFIG_DEBUG_USB
#  define JZ_DEBUG_INTS      (OHCI_INT_SO|OHCI_INT_RD|OHCI_INT_UE|OHCI_INT_OC)
#else
#  define JZ_DEBUG_INTS      0
#endif

#define JZ_NORMAL_INTS       (OHCI_INT_WDH|OHCI_INT_RHSC)
#define JZ_ALL_INTS          (JZ_NORMAL_INTS|JZ_DEBUG_INTS)

/* Periodic Intervals *******************************************************/

/* Periodic intervals 2, 4, 8, 16,and 32 supported */

#define MIN_PERINTERVAL 2
#define MAX_PERINTERVAL 32

/* Descriptors **************************************************************/

/* Actual number of allocated EDs and TDs will include one for the control ED
 * and one for the tail ED for each RHPort:
 */

#define JZ4780_OHCI_NEDS       (CONFIG_JZ4780_OHCI_NEDS + JZ_OHCI_NRHPORT)
#define JZ4780_OHCI_NTDS       (CONFIG_JZ4780_OHCI_NTDS + JZ_OHCI_NRHPORT)

/* TD delay interrupt value */

#define TD_DELAY(n)           (uint32_t)((n) << GTD_STATUS_DI_SHIFT)

/* Port numbers */

#define HPNDX(hp)             ((hp)->port)
#define HPORT(hp)             (HPNDX(hp)+1)
#define RHPNDX(rh)            ((rh)->hport.hport.port)
#define RHPORT(rh)            (RHPNDX(rh)+1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure contains one endpoint list.  The main reason for the
 * existence of this structure is to contain the sem_t value associated with
 * the ED.  It doesn't work well within the ED itself because then the
 * semaphore counter is subject to DMA cache operations (invalidating a
 * modified semaphore count is fatal!).
 */

struct jz_eplist_s
{
  volatile bool     wdhwait;   /* TRUE: Thread is waiting for WDH interrupt */
  sem_t             wdhsem;    /* Semaphore used to wait for Writeback Done Head event */
#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t  callback;  /* Transfer complete callback */
  void             *arg;       /* Argument that accompanies the callback */
#endif
  uint8_t          *buffer;    /* Buffer being transferred */
  uint16_t          buflen;    /* Length of the buffer */
  uint16_t          xfrd;      /* Number of bytes completed in the last transfer */
  struct jz_ed_s  *ed;         /* Endpoint descriptor (ED) */
  struct jz_gtd_s *tail;       /* Tail transfer descriptor (TD) */
};

/* This structure retains the state of one root hub port */

struct jz_rhport_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to struct jz_rhport_s.
   */

  struct usbhost_driver_s drvr;

  /* Root hub port status */

  volatile bool connected;     /* Connected to device */
  uint8_t rhpndx;              /* Root hub port index */
  bool ep0init;                /* True:  EP0 initialized */

  struct jz_eplist_s ep0;      /* EP0 endpoint list */

  /* This is the hub port description understood by class drivers */

  struct usbhost_roothubport_s hport;
};

/* This structure retains the overall state of the USB host controller */

struct jz_ohci_s
{
  volatile bool pscwait;       /* TRUE: Thread is waiting for Root Hub Status change */

#ifndef CONFIG_USBHOST_INT_DISABLE
  uint8_t ininterval;          /* Minimum periodic IN EP polling interval: 2, 4, 6, 16, or 32 */
  uint8_t outinterval;         /* Minimum periodic OUT EP polling interval: 2, 4, 6, 16, or 32 */
#endif

  rmutex_t lock;               /* Support mutually exclusive access */
  sem_t pscsem;                /* Semaphore to wait for Root Hub Status Change event */
  struct work_s work;          /* Supports interrupt bottom half */

#ifdef CONFIG_USBHOST_HUB
  /* Used to pass external hub port events */

  volatile struct usbhost_hubport_s *hport;
#endif

  struct usbhost_devaddr_s devgen;  /* Address generation data */

  /* Root hub ports */

  struct jz_rhport_s rhport[JZ_OHCI_NRHPORT];
};

/* The OHCI expects the size of an endpoint descriptor to be 16 bytes.
 * However, the size allocated for an endpoint descriptor is 32 bytes.  This
 * is necessary first because the Cortex-A5 cache line size is 32 bytes and
 * this is the smallest amount of memory that we can perform cache
 * operations on.  The  16-bytes is also used by the OHCI host driver in
 * order to maintain additional endpoint-specific data.
 */

struct jz_ed_s
{
  /* Hardware specific fields */

  struct ohci_ed_s hw;         /* 0-15 */

  /* Software specific fields */

  struct jz_eplist_s *eplist;  /* 16-19: List structure associated with the ED */
  uint8_t          xfrtype;    /* 20: Transfer type.  See SB_EP_ATTR_XFER_* in usb.h */
  uint8_t          interval;   /* 21: Periodic EP polling interval: 2, 4, 6, 16, or 32 */
  volatile uint8_t tdstatus;   /* 22: TD control status bits from last Writeback Done Head event */
  uint8_t          pad[9];     /* 23-31: Pad to 32-bytes */
};

#define SIZEOF_JZ_ED_S 32

/* The OHCI expects the size of an transfer descriptor to be 16 bytes.
 * However, the size allocated for an endpoint descriptor is 32 bytes in
 * RAM.  This extra 16-bytes is used by the OHCI host driver in order to
 * maintain additional endpoint-specific data.
 */

struct jz_gtd_s
{
  /* Hardware specific fields */

  struct ohci_gtd_s hw;        /* 0-15 */

  /* Software specific fields */

  struct jz_ed_s *ed;          /* 16-19: Pointer to parent ED */
  uint8_t          pad[12];    /* 20-31: Pad to 32 bytes */
};

#define SIZEOF_JZ_TD_S 32

/* The following is used to manage lists of free EDs, TDs, and TD buffers */

struct jz_list_s
{
  struct jz_list_s *flink;     /* Link to next buffer in the list */
                               /* Variable length buffer data follows */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_JZ4780_OHCI_REGDEBUG
static void jz_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void jz_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t jz_getreg(uint32_t addr);
static void jz_putreg(uint32_t val, uint32_t addr);
#else
#  define jz_getreg(addr)     getreg32(addr)
#  define jz_putreg(val,addr) putreg32(val,addr)
#endif

/* Byte stream access helper functions **************************************/

static inline uint16_t jz_getle16(const uint8_t *val);
#if 0 /* Not used */
static void jz_putle16(uint8_t *dest, uint16_t val);
#endif

/* OHCI memory pool helper functions ****************************************/

static struct jz_ed_s *jz_edalloc(void);
static void jz_edfree(struct jz_ed_s *ed);
static struct jz_gtd_s *jz_tdalloc(void);
static void jz_tdfree(struct jz_gtd_s *buffer);
static uint8_t *jz_tballoc(void);
static void jz_tbfree(uint8_t *buffer);

/* ED list helper functions *************************************************/

static int jz_addctrled(struct jz_ed_s *ed);
static inline int jz_remctrled(struct jz_ed_s *ed);

static inline int jz_addbulked(struct jz_ed_s *ed);
static inline int jz_rembulked(struct jz_ed_s *ed);

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static unsigned int jz_getinterval(uint8_t interval);
static void jz_setinttab(uint32_t value, unsigned int interval,
                          unsigned int offset);
#endif

static inline int jz_addinted(const struct usbhost_epdesc_s *epdesc,
                               struct jz_ed_s *ed);
static inline int jz_reminted(struct jz_ed_s *ed);

static inline int jz_addisoced(const struct usbhost_epdesc_s *epdesc,
                                struct jz_ed_s *ed);
static inline int jz_remisoced(struct jz_ed_s *ed);

/* Descriptor helper functions **********************************************/

static int jz_enqueuetd(struct jz_rhport_s *rhport,
                         struct jz_eplist_s *eplist, struct jz_ed_s *ed,
                         uint32_t dirpid, uint32_t toggle,
                         volatile uint8_t *buffer, size_t buflen);
static int  jz_ep0enqueue(struct jz_rhport_s *rhport);
static void jz_ep0dequeue(struct jz_eplist_s *ep0);
static int  jz_wdhwait(struct jz_rhport_s *rhport, struct jz_ed_s *ed,
                        uint8_t *buffer, uint16_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int  jz_wdhasynch(struct jz_rhport_s *rhport, struct jz_ed_s *ed,
                          usbhost_asynch_t callback, void *arg,
                          uint8_t *buffer, uint16_t buflen);
#endif
static int  jz_ctrltd(struct jz_rhport_s *rhport, struct jz_eplist_s *ep0,
                       uint32_t dirpid, uint8_t *buffer, size_t buflen);

/* Interrupt handling *******************************************************/

static void jz_rhsc_bottomhalf(void);
static void jz_wdh_bottomhalf(void);
static void jz_ohci_bottomhalf(void *arg);
static int jz_ohci_tophalf(int irq, void *context, void *arg);

/* USB host controller operations *******************************************/

static int jz_wait(struct usbhost_connection_s *conn,
                    struct usbhost_hubport_s **hport);
static int jz_rh_enumerate(struct usbhost_connection_s *conn,
                            struct usbhost_hubport_s *hport);
static int jz_enumerate(struct usbhost_connection_s *conn,
                         struct usbhost_hubport_s *hport);

static int jz_ep0configure(struct usbhost_driver_s *drvr,
                            usbhost_ep_t ep0, uint8_t funcaddr,
                            uint8_t speed, uint16_t maxpacketsize);
static int jz_epalloc(struct usbhost_driver_s *drvr,
                       const struct usbhost_epdesc_s *epdesc,
                       usbhost_ep_t *ep);
static int jz_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int jz_alloc(struct usbhost_driver_s *drvr,
                     uint8_t **buffer, size_t *maxlen);
static int jz_free(struct usbhost_driver_s *drvr, uint8_t *buffer);
static int jz_ioalloc(struct usbhost_driver_s *drvr,
                       uint8_t **buffer, size_t buflen);
static int jz_iofree(struct usbhost_driver_s *drvr, uint8_t *buffer);
static int jz_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                      const struct usb_ctrlreq_s *req,
                      uint8_t *buffer);
static int jz_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                       const struct usb_ctrlreq_s *req,
                       const uint8_t *buffer);
static int jz_transfer_common(struct jz_rhport_s *rhport,
                               struct jz_eplist_s *eplist,
                               uint8_t *buffer, size_t buflen);
static ssize_t jz_transfer(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                            uint8_t *buffer, size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static void jz_asynch_completion(struct jz_eplist_s *eplist);
static int jz_asynch(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                      uint8_t *buffer, size_t buflen,
                      usbhost_asynch_t callback, void *arg);
#endif
static int jz_cancel(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int jz_connect(struct usbhost_driver_s *drvr,
                       struct usbhost_hubport_s *hport,
                       bool connected);
#endif
static void jz_disconnect(struct usbhost_driver_s *drvr,
                           struct usbhost_hubport_s *hport);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* In this driver implementation, support is provided for only a single a
 * single USB device.  All status information can be simply retained in a
 * single global instance.
 */

static struct jz_ohci_s g_ohci =
{
  .lock = NXRMUTEX_INITIALIZER,
  .pscsem = SEM_INITIALIZER(0),
};

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_ohciconn =
{
  .wait = jz_wait,
  .enumerate = jz_enumerate,
};

/* This is a free list of EDs and TD buffers */

static struct jz_list_s *g_edfree; /* List of unused EDs */
static struct jz_list_s *g_tdfree; /* List of unused TDs */
static struct jz_list_s *g_tbfree; /* List of unused transfer buffers */

/* Allocated descriptor memory. These must all be properly aligned
 * and must be positioned in a DMA-able memory region.
 */

/* This must be aligned to a 256-byte boundary */

static struct ohci_hcca_s g_hcca
                          aligned_data(256);

/* Pools of free descriptors and buffers.  These will all be linked
 * into the free lists declared above.  These must be aligned to 8-byte
 * boundaries (we do 16-byte alignment).
 */

static struct jz_ed_s    g_edalloc[JZ4780_OHCI_NEDS]
                          aligned_data(JZ4780_DMA_ALIGN);
static struct jz_gtd_s   g_tdalloc[JZ4780_OHCI_NTDS]
                          aligned_data(JZ4780_DMA_ALIGN);
static uint8_t            g_bufalloc[JZ_BUFALLOC]
                          aligned_data(JZ4780_DMA_ALIGN);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jz_printreg
 *
 * Description:
 *   Print the contents of an JZ4780 OHCI register operation
 *
 ****************************************************************************/

#ifdef CONFIG_JZ4780_OHCI_REGDEBUG
static void jz_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  uinfo("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/****************************************************************************
 * Name: jz_checkreg
 *
 * Description:
 *   Get the contents of an JZ4780 OHCI register
 *
 ****************************************************************************/

#ifdef CONFIG_JZ4780_OHCI_REGDEBUG
static void jz_checkreg(uint32_t addr, uint32_t val, bool iswrite)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register
   * last time?  Are we polling the register?  If so, suppress the output.
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

              jz_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              uinfo("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = addr;
      preval    = val;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new register access */

      jz_printreg(addr, val, iswrite);
    }
}
#endif

/****************************************************************************
 * Name: jz_getreg
 *
 * Description:
 *   Get the contents of an JZ4780 register
 *
 ****************************************************************************/

#ifdef CONFIG_JZ4780_OHCI_REGDEBUG
static uint32_t jz_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  jz_checkreg(addr, val, false);
  return val;
}
#endif

/****************************************************************************
 * Name: jz_putreg
 *
 * Description:
 *   Set the contents of an JZ4780 register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_JZ4780_OHCI_REGDEBUG
static void jz_putreg(uint32_t val, uint32_t addr)
{
  /* Check if we need to print this value */

  jz_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: jz_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

static inline uint16_t jz_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: jz_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 ****************************************************************************/

#if 0 /* Not used */
static void jz_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}
#endif

/****************************************************************************
 * Name: jz_edalloc
 *
 * Description:
 *   Allocate an endpoint descriptor by removing it from the free list
 *
 ****************************************************************************/

static struct jz_ed_s *jz_edalloc(void)
{
  struct jz_ed_s *ed;

  /* Remove the ED from the freelist */

  ed = (struct jz_ed_s *)g_edfree;
  if (ed)
    {
      g_edfree = ((struct jz_list_s *)ed)->flink;
    }

  return ed;
}

/****************************************************************************
 * Name: jz_edfree
 *
 * Description:
 *   Free an endpoint descriptor by returning to the free list
 *
 ****************************************************************************/

static inline void jz_edfree(struct jz_ed_s *ed)
{
  struct jz_list_s *entry = (struct jz_list_s *)ed;

  /* Put the ED back into the free list */

  entry->flink = g_edfree;
  g_edfree     = entry;
}

/****************************************************************************
 * Name: jz_tdalloc
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
 ****************************************************************************/

static struct jz_gtd_s *jz_tdalloc(void)
{
  struct jz_gtd_s *ret;
  irqstate_t flags;

  /* Disable interrupts momentarily so that jz_tdfree is not called from the
   * interrupt handler.
   */

  flags = enter_critical_section();
  ret   = (struct jz_gtd_s *)g_tdfree;
  if (ret)
    {
      g_tdfree = ((struct jz_list_s *)ret)->flink;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: jz_tdfree
 *
 * Description:
 *   Free a transfer descriptor by returning it to the free list
 *
 * Assumptions:
 *   - Only called from the WDH interrupt handler (and during
 *     initialization).
 *   - Interrupts are disabled in any case.
 *
 ****************************************************************************/

static void jz_tdfree(struct jz_gtd_s *td)
{
  struct jz_list_s *tdfree = (struct jz_list_s *)td;
  DEBUGASSERT(td);

  tdfree->flink = g_tdfree;
  g_tdfree      = tdfree;
}

/****************************************************************************
 * Name: jz_tballoc
 *
 * Description:
 *   Allocate an request/descriptor transfer buffer from the free list
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *   - Protection from re-entrance must be assured by the caller
 *
 ****************************************************************************/

static uint8_t *jz_tballoc(void)
{
  uint8_t *ret = (uint8_t *)g_tbfree;
  if (ret)
    {
      g_tbfree = ((struct jz_list_s *)ret)->flink;
    }

  return ret;
}

/****************************************************************************
 * Name: jz_tbfree
 *
 * Description:
 *   Return an request/descriptor transfer buffer to the free list
 *
 ****************************************************************************/

static void jz_tbfree(uint8_t *buffer)
{
  struct jz_list_s *tbfree = (struct jz_list_s *)buffer;

  if (tbfree)
    {
      tbfree->flink = g_tbfree;
      g_tbfree      = tbfree;
    }
}

/****************************************************************************
 * Name: jz_addctrled
 *
 * Description:
 *   Helper function to add an ED to the control list.
 *
 ****************************************************************************/

static int jz_addctrled(struct jz_ed_s *ed)
{
  irqstate_t flags;
  uint32_t regval;
  uintptr_t physed;

  /* Disable control list processing while we modify the list */

  flags   = enter_critical_section();
  regval  = jz_getreg(JZ_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_CLE;
  jz_putreg(regval, JZ_USBHOST_CTRL);

  /* Add the new control ED to the head of the control list */

  ed->hw.nexted = jz_getreg(JZ_USBHOST_CTRLHEADED);
  up_clean_dcache((uintptr_t)ed, (uintptr_t)ed + sizeof(struct ohci_ed_s));

  physed = jz_physramaddr((uintptr_t)ed);
  jz_putreg((uint32_t)physed, JZ_USBHOST_CTRLHEADED);

  /* Re-enable control list processing. */

  jz_putreg(0, JZ_USBHOST_CTRLED);

  regval  = jz_getreg(JZ_USBHOST_CTRL);
  regval |= OHCI_CTRL_CLE;
  jz_putreg(regval, JZ_USBHOST_CTRL);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: jz_remctrled
 *
 * Description:
 *   Helper function remove an ED from the control list.
 *
 ****************************************************************************/

static inline int jz_remctrled(struct jz_ed_s *ed)
{
  struct jz_ed_s *curr;
  struct jz_ed_s *prev;
  irqstate_t flags;
  uintptr_t physed;
  uint32_t regval;

  /* Disable control list processing while we modify the list */

  flags = enter_critical_section();
  regval  = jz_getreg(JZ_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_CLE;
  jz_putreg(regval, JZ_USBHOST_CTRL);

  /* Find the ED in the control list. */

  physed = jz_getreg(JZ_USBHOST_CTRLHEADED);
  for (curr = (struct jz_ed_s *)jz_virtramaddr(physed), prev = NULL;
       curr && curr != ed;
       prev = curr, curr = (struct jz_ed_s *)curr->hw.nexted);

  /* Hmmm.. It would be a bug if we do not find the ED in the control list. */

  DEBUGASSERT(curr != NULL);

  /* Remove the ED from the control list */

  if (curr != NULL)
    {
      /* Is this ED the first on in the control list? */

      if (prev == NULL)
        {
          /* Yes... set the head of the control list to skip over this ED */

          jz_putreg(ed->hw.nexted, JZ_USBHOST_CTRLHEADED);
        }
      else
        {
          /* No.. set the forward link of the previous ED in the list
           * skip over this ED.
           */

          prev->hw.nexted = ed->hw.nexted;
          up_clean_dcache((uintptr_t)prev,
                          (uintptr_t)prev + sizeof(struct jz_ed_s));
        }
    }

  /* Re-enable control list processing if the control list is still non-empty
   * after removing the ED node.
   */

  jz_putreg(0, JZ_USBHOST_CTRLED);
  if (jz_getreg(JZ_USBHOST_CTRLHEADED) != 0)
    {
      /* If the control list is now empty, then disable it */

      regval  = jz_getreg(JZ_USBHOST_CTRL);
      regval |= OHCI_CTRL_CLE;
      jz_putreg(regval, JZ_USBHOST_CTRL);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: jz_addbulked
 *
 * Description:
 *   Helper function to add an ED to the bulk list.
 *
 ****************************************************************************/

static inline int jz_addbulked(struct jz_ed_s *ed)
{
#ifndef CONFIG_USBHOST_BULK_DISABLE
  irqstate_t flags;
  uint32_t regval;
  uintptr_t physed;

  /* Disable bulk list processing while we modify the list */

  flags   = enter_critical_section();
  regval  = jz_getreg(JZ_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_BLE;
  jz_putreg(regval, JZ_USBHOST_CTRL);

  /* Add the new bulk ED to the head of the bulk list */

  ed->hw.nexted = jz_getreg(JZ_USBHOST_BULKHEADED);
  up_clean_dcache((uintptr_t)ed, (uintptr_t)ed + sizeof(struct ohci_ed_s));

  physed = jz_physramaddr((uintptr_t)ed);
  jz_putreg((uint32_t)physed, JZ_USBHOST_BULKHEADED);

  /* Re-enable bulk list processing. */

  jz_putreg(0, JZ_USBHOST_BULKED);

  regval  = jz_getreg(JZ_USBHOST_CTRL);
  regval |= OHCI_CTRL_BLE;
  jz_putreg(regval, JZ_USBHOST_CTRL);

  leave_critical_section(flags);
  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: jz_rembulked
 *
 * Description:
 *   Helper function remove an ED from the bulk list.
 *
 ****************************************************************************/

static inline int jz_rembulked(struct jz_ed_s *ed)
{
#ifndef CONFIG_USBHOST_BULK_DISABLE
  struct jz_ed_s *curr;
  struct jz_ed_s *prev;
  irqstate_t flags;
  uintptr_t physed;
  uint32_t regval;

  /* Disable bulk list processing while we modify the list */

  flags = enter_critical_section();
  regval  = jz_getreg(JZ_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_BLE;
  jz_putreg(regval, JZ_USBHOST_CTRL);

  /* Find the ED in the bulk list. */

  physed = jz_getreg(JZ_USBHOST_BULKHEADED);
  for (curr = (struct jz_ed_s *)jz_virtramaddr(physed), prev = NULL;
       curr && curr != ed;
       prev = curr, curr = (struct jz_ed_s *)curr->hw.nexted);

  /* Hmmm.. It would be a bug if we do not find the ED in the bulk list. */

  DEBUGASSERT(curr != NULL);

  /* Remove the ED from the bulk list */

  if (curr != NULL)
    {
      /* Is this ED the first on in the bulk list? */

      if (prev == NULL)
        {
          /* Yes... set the head of the bulk list to skip over this ED */

          jz_putreg(ed->hw.nexted, JZ_USBHOST_BULKHEADED);
        }
      else
        {
          /* No.. set the forward link of the previous ED in the list
           * skip over this ED.
           */

          prev->hw.nexted = ed->hw.nexted;
          up_clean_dcache((uintptr_t)prev,
                          (uintptr_t)prev + sizeof(struct jz_ed_s));
        }
    }

  /* Re-enable bulk list processing if the bulk list is still non-empty
   * after removing the ED node.
   */

  jz_putreg(0, JZ_USBHOST_BULKED);
  if (jz_getreg(JZ_USBHOST_BULKHEADED) != 0)
    {
      /* If the bulk list is now empty, then disable it */

      regval  = jz_getreg(JZ_USBHOST_CTRL);
      regval |= OHCI_CTRL_BLE;
      jz_putreg(regval, JZ_USBHOST_CTRL);
    }

  leave_critical_section(flags);
  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: jz_getinterval
 *
 * Description:
 *   Convert the endpoint polling interval into a HCCA table increment
 *
 ****************************************************************************/

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static unsigned int jz_getinterval(uint8_t interval)
{
  /* The bInterval field of the endpoint descriptor contains the polling
   * interval for interrupt and isochronous endpoints. For other types of
   * endpoint, this value should be ignored. bInterval is provided in units
   * of 1MS frames.
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

/****************************************************************************
 * Name: jz_setinttab
 *
 * Description:
 *   Set the interrupt table to the selected value using the provided
 *   interval and offset.
 *
 ****************************************************************************/

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static void jz_setinttab(uint32_t value, unsigned int interval,
                          unsigned int offset)
{
  uintptr_t inttbl;
  unsigned int i;

  for (i = offset; i < HCCA_INTTBL_WSIZE; i += interval)
    {
      /* Modify the table value */

      g_hcca.inttbl[i] = value;
    }

  /* Make sure that the modified table value is flushed to RAM */

  inttbl = (uintptr_t)g_hcca.inttbl;
  up_clean_dcache(inttbl, inttbl + sizeof(uint32_t)*HCCA_INTTBL_WSIZE);
}
#endif

/****************************************************************************
 * Name: jz_addinted
 *
 * Description:
 *   Helper function to add an ED to the HCCA interrupt table.
 *
 *   To avoid reshuffling the table so much and to keep life simple in
 *   general, the following rules are applied:
 *
 *     1. IN EDs get the even entries, OUT EDs get the odd entries.
 *     2. Add IN/OUT EDs are scheduled together at the minimum interval of
 *        all IN/OUT EDs.
 *
 *   This has the following consequences:
 *
 *     1. The minimum support polling rate is 2MS, and
 *     2. Some devices may get polled at a much higher rate than they
 *         request.
 *
 ****************************************************************************/

static inline int jz_addinted(const struct usbhost_epdesc_s *epdesc,
                               struct jz_ed_s *ed)
{
#ifndef CONFIG_USBHOST_INT_DISABLE
  irqstate_t flags;
  unsigned int interval;
  unsigned int offset;
  uintptr_t physed;
  uintptr_t physhead;
  uint32_t regval;

  /* Disable periodic list processing.  Does this take effect immediately?
   * Or at the next SOF... need to check.
   */

  flags   = enter_critical_section();
  regval  = jz_getreg(JZ_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_PLE;
  jz_putreg(regval, JZ_USBHOST_CTRL);

  /* Get the quanitized interval value associated with this ED and save it
   * in the ED.
   */

  interval     = jz_getinterval(epdesc->interval);
  ed->interval = interval;
  usbhost_vtrace2(OHCI_VTRACE2_INTERVAL, epdesc->interval, interval);

  /* Get the offset associated with the ED direction. IN EDs get the even
   * entries, OUT EDs get the odd entries.
   *
   * Get the new, minimum interval. Add IN/OUT EDs are scheduled together
   * at the minimum interval of all IN/OUT EDs.
   */

  if (epdesc->in)
    {
      offset = 0;
      if (g_ohci.ininterval > interval)
        {
          g_ohci.ininterval = interval;
        }
      else
        {
          interval = g_ohci.ininterval;
        }
    }
  else
    {
      offset = 1;
      if (g_ohci.outinterval > interval)
        {
          g_ohci.outinterval = interval;
        }
      else
        {
          interval = g_ohci.outinterval;
        }
    }

  usbhost_vtrace2(OHCI_VTRACE2_MININTERVAL, interval, offset);

  /* Get the (physical) head of the first of the duplicated entries.  The
   * first offset entry is always guaranteed to contain the common ED list
   * head.
   */

  physhead = g_hcca.inttbl[offset];

  /* Clear all current entries in the interrupt table for this direction */

  jz_setinttab(0, 2, offset);

  /* Add the new ED before the old head of the periodic ED list and set the
   * new ED as the head ED in all of the appropriate entries of the HCCA
   * interrupt table.
   */

  ed->hw.nexted = physhead;
  up_clean_dcache((uintptr_t)ed, (uintptr_t)ed + sizeof(struct ohci_ed_s));

  physed =  jz_physramaddr((uintptr_t)ed);
  jz_setinttab((uint32_t)physed, interval, offset);

  usbhost_vtrace1(OHCI_VTRACE1_PHYSED, physed);

  /* Re-enable periodic list processing */

  regval  = jz_getreg(JZ_USBHOST_CTRL);
  regval |= OHCI_CTRL_PLE;
  jz_putreg(regval, JZ_USBHOST_CTRL);

  leave_critical_section(flags);
  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: jz_reminted
 *
 * Description:
 *   Helper function to remove an ED from the HCCA interrupt table.
 *
 *   To avoid reshuffling the table so much and to keep life simple in
 *   general, the following rules are applied:
 *
 *     1. IN EDs get the even entries, OUT EDs get the odd entries.
 *     2. Add IN/OUT EDs are scheduled together at the minimum interval of
 *        all IN/OUT EDs.
 *
 *   This has the following consequences:
 *
 *     1. The minimum support polling rate is 2MS, and
 *     2. Some devices may get polled at a much higher rate than they
 *        request.
 *
 ****************************************************************************/

static inline int jz_reminted(struct jz_ed_s *ed)
{
#ifndef CONFIG_USBHOST_INT_DISABLE
  struct jz_ed_s *head;
  struct jz_ed_s *curr;
  struct jz_ed_s *prev;
  irqstate_t flags;
  uintptr_t physhead;
  unsigned int interval;
  unsigned int offset;
  uint32_t regval;

  /* Disable periodic list processing.  Does this take effect immediately?
   * Or at the next SOF... need to check.
   */

  flags   = enter_critical_section();
  regval  = jz_getreg(JZ_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_PLE;
  jz_putreg(regval, JZ_USBHOST_CTRL);

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

  physhead = g_hcca.inttbl[offset];
  head     = (struct jz_ed_s *)jz_virtramaddr((uintptr_t)physhead);

#ifdef CONFIG_USBHOST_TRACE
  usbhost_vtrace1(OHCI_VTRACE1_VIRTED, (uintptr_t)ed);
#else
  uinfo("ed: %p head: %08" PRIxPTR " next: %08" PRIx32 " offset: %d\n",
        ed, (uintptr_t)physhead, head ? head->hw.nexted : 0, offset);
#endif

  /* Find the ED to be removed in the ED list */

  for (curr = head, prev = NULL;
       curr && curr != ed;
       prev = curr, curr = (struct jz_ed_s *)curr->hw.nexted);

  /* Hmmm.. It would be a bug if we do not find the ED in the bulk list. */

  DEBUGASSERT(curr != NULL);
  if (curr != NULL)
    {
      /* Clear all current entries in the interrupt table for this
       * direction
       */

      jz_setinttab(0, 2, offset);

      /* Remove the ED from the list..  Is this ED the first on in the
       * list?
       */

      if (prev == NULL)
        {
          /* Yes... set the head of the bulk list to skip over this ED */

          physhead = ed->hw.nexted;
          head     = (struct jz_ed_s *)jz_virtramaddr((uintptr_t)physhead);
        }
      else
        {
          /* No.. set the forward link of the previous ED in the list
           * skip over this ED.
           */

          prev->hw.nexted = ed->hw.nexted;
          up_clean_dcache((uintptr_t)prev,
                          (uintptr_t)prev + sizeof(struct ohci_ed_s));
        }

#ifdef CONFIG_USBHOST_TRACE
      usbhost_vtrace1(OHCI_VTRACE1_VIRTED, (uintptr_t)ed);
#else
      uinfo("ed: %p head: %08" PRIxPTR " next: %08" PRIx32 "\n",
            ed, physhead, head ? head->hw.nexted : 0);
#endif

      /* Calculate the new minimum interval for this list */

      interval = MAX_PERINTERVAL;
      for (curr = head; curr; curr = (struct jz_ed_s *)curr->hw.nexted)
        {
          if (curr->interval < interval)
            {
              interval = curr->interval;
            }
        }

      usbhost_vtrace2(OHCI_VTRACE2_MININTERVAL, interval, offset);

      /* Save the new minimum interval */

      if ((ed->hw.ctrl & ED_CONTROL_D_MASK) == ED_CONTROL_D_IN)
        {
          g_ohci.ininterval  = interval;
        }
      else
        {
          g_ohci.outinterval = interval;
        }

      /* Set the head ED in all of the appropriate entries of the HCCA
       * interrupt table (head might be NULL).
       */

      jz_setinttab((uint32_t)physhead, interval, offset);
    }

  /* Re-enabled periodic list processing */

  if (head != NULL)
    {
      regval  = jz_getreg(JZ_USBHOST_CTRL);
      regval |= OHCI_CTRL_PLE;
      jz_putreg(regval, JZ_USBHOST_CTRL);
    }

  leave_critical_section(flags);
  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: jz_addisoced
 *
 * Description:
 *   Helper functions to add an ED to the periodic table.
 *
 ****************************************************************************/

static inline int jz_addisoced(const struct usbhost_epdesc_s *epdesc,
                                struct jz_ed_s *ed)
{
#ifndef CONFIG_USBHOST_ISOC_DISABLE
#  warning "Isochronous endpoints not yet supported"
#endif
  return -ENOSYS;
}

/****************************************************************************
 * Name: jz_remisoced
 *
 * Description:
 *   Helper functions to remove an ED from the periodic table.
 *
 ****************************************************************************/

static inline int jz_remisoced(struct jz_ed_s *ed)
{
#ifndef CONFIG_USBHOST_ISOC_DISABLE
#  warning "Isochronous endpoints not yet supported"
#endif
  return -ENOSYS;
}

/****************************************************************************
 * Name: jz_enqueuetd
 *
 * Description:
 *   Enqueue a transfer descriptor.  Notice that this function only supports
 *   queueing one TD per ED.
 *
 ****************************************************************************/

static int jz_enqueuetd(struct jz_rhport_s *rhport,
                         struct jz_eplist_s *eplist, struct jz_ed_s *ed,
                         uint32_t dirpid, uint32_t toggle,
                         volatile uint8_t *buffer, size_t buflen)
{
  struct jz_gtd_s *td;
  struct jz_gtd_s *tdtail;
  uintptr_t phytd;
  uintptr_t phytail;
  uintptr_t phybuf;
  int ret = -ENOMEM;

  /* Allocate a TD from the free list */

  td = jz_tdalloc();
  if (td != NULL)
    {
      /* Skip processing of this ED while we modify the TD list. */

      ed->hw.ctrl      |= ED_CONTROL_K;
      up_clean_dcache((uintptr_t)ed,
                      (uintptr_t)ed + sizeof(struct ohci_ed_s));

      /* Get the tail ED for this hub port */

      tdtail            = eplist->tail;

      /* Get physical addresses to support the DMA */

      phytd             = jz_physramaddr((uintptr_t)td);
      phytail           = jz_physramaddr((uintptr_t)tdtail);
      phybuf            = jz_physramaddr((uintptr_t)buffer);

      /* Initialize the allocated TD and link it before the common tail TD. */

      td->hw.ctrl       = (GTD_STATUS_R | dirpid | TD_DELAY(0) |
                           toggle | GTD_STATUS_CC_MASK);
      tdtail->hw.ctrl   = 0;
      td->hw.cbp        = (uint32_t)phybuf;
      tdtail->hw.cbp    = 0;
      td->hw.nexttd     = (uint32_t)phytail;
      tdtail->hw.nexttd = 0;
      td->hw.be         = (uint32_t)(phybuf + (buflen - 1));
      tdtail->hw.be     = 0;

      /* Configure driver-only fields in the extended TD structure */

      td->ed            = ed;

      /* Link the td to the head of the ED's TD list */

      ed->hw.headp      = (uint32_t)phytd | ((ed->hw.headp) & ED_HEADP_C);
      ed->hw.tailp      = (uint32_t)phytail;

      /* Flush the buffer (if there is one), the new TD, and the modified ED
       * to RAM.
       */

      if (buffer && buflen > 0)
        {
          up_clean_dcache((uintptr_t)buffer,
                          (uintptr_t)buffer + buflen);
        }

      up_clean_dcache((uintptr_t)tdtail,
                      (uintptr_t)tdtail + sizeof(struct ohci_gtd_s));
      up_clean_dcache((uintptr_t)td,
                      (uintptr_t)td + sizeof(struct ohci_gtd_s));

      /* Resume processing of this ED */

      ed->hw.ctrl      &= ~ED_CONTROL_K;
      up_clean_dcache((uintptr_t)ed,
                      (uintptr_t)ed + sizeof(struct ohci_ed_s));
      ret               = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: jz_ep0enqueue
 *
 * Description:
 *   Initialize ED for a root hub EP0, add it to the control ED list, and
 *   enable control transfers.
 *
 * Input Parameters:
 *   rhpndx - Root hub port index.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int jz_ep0enqueue(struct jz_rhport_s *rhport)
{
  struct jz_ed_s *edctrl;
  struct jz_gtd_s *tdtail;
  irqstate_t flags;
  uintptr_t physaddr;
  int ret;

  DEBUGASSERT(rhport && !rhport->ep0init && rhport->ep0.ed == NULL &&
              rhport->ep0.tail == NULL);

  /* Allocate a control ED and a tail TD */

  flags  = enter_critical_section();
  edctrl = jz_edalloc();
  if (!edctrl)
    {
      leave_critical_section(flags);
      return -ENOMEM;
    }

  tdtail = jz_tdalloc();
  if (!tdtail)
    {
      jz_edfree(edctrl);
      leave_critical_section(flags);
      return -ENOMEM;
    }

  rhport->ep0.ed   = edctrl;
  rhport->ep0.tail = tdtail;

  /* Initialize the common tail TD for this port */

  memset(tdtail, 0, sizeof(struct jz_gtd_s));
  tdtail->ed       = edctrl;

  /* Initialize the control endpoint for this port.
   * Set up some default values (like max packetsize = 8).
   * NOTE that the SKIP bit is set until the first real TD is added.
   */

  memset(edctrl, 0, sizeof(struct jz_ed_s));
  jz_ep0configure(&rhport->drvr, &rhport->ep0, 0,
                   rhport->hport.hport.speed, 8);

  edctrl->hw.ctrl  |= ED_CONTROL_K;
  edctrl->eplist    = &rhport->ep0;
  edctrl->xfrtype   = USB_EP_ATTR_XFER_CONTROL;

  /* Link the common tail TD to the ED's TD list */

  physaddr          = jz_physramaddr((uintptr_t)tdtail);
  edctrl->hw.headp  = (uint32_t)physaddr;
  edctrl->hw.tailp  = (uint32_t)physaddr;

  /* Flush the affected control ED and tail TD to RAM */

  up_clean_dcache((uintptr_t)edctrl,
                  (uintptr_t)edctrl + sizeof(struct ohci_ed_s));
  up_clean_dcache((uintptr_t)tdtail,
                  (uintptr_t)tdtail + sizeof(struct ohci_gtd_s));

  /* Add the ED to the control list */

  ret = jz_addctrled(edctrl);
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: jz_ep0dequeue
 *
 * Description:
 *   Remove the ED for EP0 from the control ED list and possibly disable
 *   control list processing.
 *
 * Input Parameters:
 *   ep0 - The control endpoint to be released.  May be the control endpoint
 *     for an attached hub.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void jz_ep0dequeue(struct jz_eplist_s *ep0)
{
  struct jz_ed_s *edctrl;
  struct jz_ed_s *curred;
  struct jz_ed_s *preved;
  struct jz_gtd_s *tdtail;
  struct jz_gtd_s *currtd;
  irqstate_t flags;
  uintptr_t physcurr;
  uint32_t regval;

  DEBUGASSERT(ep0->ed != NULL && ep0->tail != NULL);

  /* ControlListEnable.  This bit is cleared to disable the processing of the
   * Control list.  We should never modify the control list while CLE is set.
   */

  flags   = enter_critical_section();
  regval  = jz_getreg(JZ_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_CLE;
  jz_putreg(regval, JZ_USBHOST_CTRL);

  /* Search the control list to find the entry to be removed (and its
   * predecessor).
   */

  edctrl   = ep0->ed;
  physcurr = jz_getreg(JZ_USBHOST_CTRLHEADED);

  for (curred = (struct jz_ed_s *)jz_virtramaddr(physcurr),
       preved = NULL;
       curred && curred != edctrl;
       preved = curred,
       curred = (struct jz_ed_s *)jz_virtramaddr(physcurr))
    {
      physcurr = curred->hw.nexted;
    }

  DEBUGASSERT(curred);

  /* Remove the ED from the control list */

  if (preved)
    {
      /* Unlink the ED from the previous ED in the list */

      preved->hw.nexted = edctrl->hw.nexted;

      /* Flush the modified ED to RAM */

      up_clean_dcache((uintptr_t)preved,
                      (uintptr_t)preved + sizeof(struct ohci_ed_s));
    }
  else
    {
      /* Set the new control list head ED */

      jz_putreg(edctrl->hw.nexted, JZ_USBHOST_CTRLHEADED);

      /* If the control list head is still non-NULL, then (re-)enable
       * processing of the Control list.
       */

      if (edctrl->hw.nexted != 0)
        {
          regval = jz_getreg(JZ_USBHOST_CTRL);
          regval |= OHCI_CTRL_CLE;
          jz_putreg(regval, JZ_USBHOST_CTRL);
        }
    }

  leave_critical_section(flags);

  /* Release any TDs that may still be attached to the ED. */

  tdtail   = ep0->tail;
  physcurr = edctrl->hw.headp;

  for (currtd = (struct jz_gtd_s *)jz_virtramaddr(physcurr);
       currtd && currtd != tdtail;
       currtd = (struct jz_gtd_s *)jz_virtramaddr(physcurr))
    {
      physcurr = currtd->hw.nexttd;
      jz_tdfree(currtd);
    }

  /* Free the tail TD and the control ED allocated for this port */

  jz_tdfree(tdtail);
  jz_edfree(edctrl);

  ep0->ed   = NULL;
  ep0->tail = NULL;
}

/****************************************************************************
 * Name: jz_wdhwait
 *
 * Description:
 *   Set the request for the Writeback Done Head event well BEFORE enabling
 *   the transfer (as soon as we are absolutely committed to perform the
 *   transfer).  We do this to minimize race conditions.  This logic would
 *   have to be expanded if we want to have more than one packet in flight
 *   at a time!
 *
 ****************************************************************************/

static int jz_wdhwait(struct jz_rhport_s *rhport, struct jz_ed_s *ed,
                       uint8_t *buffer, uint16_t buflen)
{
  struct jz_eplist_s *eplist;
  irqstate_t flags = enter_critical_section();
  int ret = -ENODEV;

  /* Is the device still connected? */

  if (rhport->connected)
    {
      /* Yes.. Get the endpoint list associated with the ED */

      eplist = ed->eplist;
      DEBUGASSERT(eplist);

      /* Then set wdhwait to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer
       * completed.
       */

      eplist->wdhwait  = true;
#ifdef CONFIG_USBHOST_ASYNCH
      eplist->callback = NULL;
      eplist->arg      = NULL;
#endif
      eplist->buffer   = buffer;
      eplist->buflen   = buflen;
      ret              = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: jz_wdhasynch
 *
 * Description:
 *   Set the request for the Writeback Done Head callback well BEFORE
 *   enabling the transfer (as soon as we are absolutely committed to
 *   perform the transfer).  We do this to minimize race conditions.  This
 *   logic would have to be expanded if we want to have more than one packet
 *   in flight at a time!
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int jz_wdhasynch(struct jz_rhport_s *rhport, struct jz_ed_s *ed,
                         usbhost_asynch_t callback, void *arg,
                         uint8_t *buffer, uint16_t buflen)
{
  struct jz_eplist_s *eplist;
  irqstate_t flags = enter_critical_section();
  int ret = -ENODEV;

  /* Is the device still connected? */

  if (rhport->connected)
    {
      /* Yes.. Get the endpoint list associated with the ED */

      eplist = ed->eplist;
      DEBUGASSERT(eplist);

      /* Then set wdhwait to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer
       * completed.
       */

      eplist->wdhwait  = false;
      eplist->callback = callback;
      eplist->arg      = arg;
      eplist->buffer   = buffer;
      eplist->buflen   = buflen;
      ret              = OK;
    }

  leave_critical_section(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: jz_ctrltd
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
 ****************************************************************************/

static int jz_ctrltd(struct jz_rhport_s *rhport,
                      struct jz_eplist_s *eplist, uint32_t dirpid,
                      uint8_t *buffer, size_t buflen)
{
  struct jz_ed_s *edctrl;
  uint32_t toggle;
  uint32_t regval;
  int ret;
  int ret2;

  /* Set the request for the Writeback Done Head event well BEFORE enabling
   * the transfer.
   */

  edctrl = eplist->ed;

  ret = jz_wdhwait(rhport, edctrl, buffer, buflen);
  if (ret != OK)
    {
      usbhost_trace1(OHCI_TRACE1_DEVDISCONN, RHPORT(rhport));
      return ret;
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

  edctrl->tdstatus = TD_CC_NOERROR;
  ret = jz_enqueuetd(rhport, eplist, edctrl, dirpid, toggle, buffer,
                      buflen);
  if (ret == OK)
    {
      /* Set ControlListFilled.  This bit is used to indicate whether there
       * are TDs on the Control list.
       */

      regval = jz_getreg(JZ_USBHOST_CMDST);
      regval |= OHCI_CMDST_CLF;
      jz_putreg(regval, JZ_USBHOST_CMDST);

      /* Release the OHCI semaphore while we wait.  Other threads need the
       * opportunity to access the OHCI resources while we wait.
       *
       * REVISIT:  Is this safe?  NO.  This is a bug and needs rethinking.
       * We need to lock all of the port-resources (not OHCI common) until
       * the transfer is complete.  But we can't use the common OHCI lock
       * or we will deadlock while waiting (because the working thread that
       * wakes this thread up needs the lock).
       */
#warning REVISIT
      nxrmutex_unlock(&g_ohci.lock);

      /* Wait for the Writeback Done Head interrupt.  Loop to handle any
       * false alarm semaphore counts.
       */

      while (eplist->wdhwait && ret >= 0)
        {
          ret = nxsem_wait_uninterruptible(&eplist->wdhsem);
        }

      /* Re-acquire the OHCI semaphore.  The caller expects to be holding
       * this upon return.
       */

      ret2 = nxrmutex_lock(&g_ohci.lock);
      if (ret2 < 0)
        {
          ret = ret2;
        }

      if (ret < 0)
        {
          /* The thread was canceled */
        }

      /* Check the TD completion status bits */

      else if (edctrl->tdstatus == TD_CC_NOERROR)
        {
          ret = OK;
        }
      else
        {
          usbhost_trace2(OHCI_TRACE2_BADTDSTATUS, RHPORT(rhport),
                         edctrl->tdstatus);
          ret = edctrl->tdstatus == TD_CC_STALL ? -EPERM : -EIO;
        }
    }

  /* Make sure that there is no outstanding request on this endpoint */

  eplist->wdhwait = false;
  return ret;
}

/****************************************************************************
 * Name: jz_rhsc_bottomhalf
 *
 * Description:
 *   OHCI root hub status change interrupt handler
 *
 ****************************************************************************/

static void jz_rhsc_bottomhalf(void)
{
  struct jz_rhport_s *rhport;
  uint32_t regaddr;
  uint32_t rhportst;
  int rhpndx;

  /* Handle root hub status change on each root port */

  for (rhpndx = 0; rhpndx < JZ_OHCI_NRHPORT; rhpndx++)
    {
      rhport   = &g_ohci.rhport[rhpndx];

      regaddr  = JZ_USBHOST_RHPORTST(rhpndx + 1);
      rhportst = jz_getreg(regaddr);

      usbhost_vtrace2(OHCI_VTRACE2_RHPORTST, rhpndx + 1, (uint16_t)rhportst);

      if ((rhportst & OHCI_RHPORTST_CSC) != 0)
        {
          uint32_t rhstatus = jz_getreg(JZ_USBHOST_RHSTATUS);
          usbhost_vtrace1(OHCI_VTRACE1_CSC, rhstatus);

          /* If DRWE is set, Connect Status Change indicates a remote
           * wake-up event
           */

          if (rhstatus & OHCI_RHSTATUS_DRWE)
            {
              usbhost_vtrace1(OHCI_VTRACE1_DRWE, rhstatus);
            }

          /* Otherwise... Not a remote wake-up event */

          else
            {
              /* Check current connect status */

              if ((rhportst & OHCI_RHPORTST_CCS) != 0)
                {
                  /* Connected ... Did we just become connected? */

                  if (!rhport->connected)
                    {
                      /* Yes.. connected. */

                      rhport->connected = true;

                      usbhost_vtrace2(OHCI_VTRACE2_CONNECTED,
                                      rhpndx + 1, g_ohci.pscwait);

                      /* Notify any waiters */

                      if (g_ohci.pscwait)
                        {
                          nxsem_post(&g_ohci.pscsem);
                          g_ohci.pscwait = false;
                        }
                    }
                  else
                    {
                      usbhost_vtrace1(OHCI_VTRACE1_ALREADYCONN, rhportst);
                    }

                  /* The LSDA (Low speed device attached) bit is valid
                   * when CCS == 1.
                   */

                  if ((rhportst & OHCI_RHPORTST_LSDA) != 0)
                    {
                      rhport->hport.hport.speed = USB_SPEED_LOW;
                    }
                  else
                    {
                      rhport->hport.hport.speed = USB_SPEED_FULL;
                    }

                  usbhost_vtrace1(OHCI_VTRACE1_SPEED,
                                  rhport->hport.hport.speed);
                }

              /* Check if we are now disconnected */

              else if (rhport->connected)
                {
                  /* Yes.. disconnect the device */

                  usbhost_vtrace2(OHCI_VTRACE2_DISCONNECTED,
                                  rhpndx + 1, g_ohci.pscwait);

                  rhport->connected = false;

                  /* Set the port speed to the default (FULL).  We cannot
                   * yet free the function address.  That has to be done
                   * by the class when responds to the disconnection.
                   */

                  rhport->hport.hport.speed = USB_SPEED_FULL;

                  /* Are we bound to a class instance? */

                  if (rhport->hport.hport.devclass)
                    {
                      /* Yes.. Disconnect the class */

                      CLASS_DISCONNECTED(rhport->hport.hport.devclass);
                      rhport->hport.hport.devclass = NULL;
                    }

                  /* Notify any waiters for the Root Hub Status change
                   * event.
                   */

                  if (g_ohci.pscwait)
                    {
                      nxsem_post(&g_ohci.pscsem);
                      g_ohci.pscwait = false;
                    }
                }
              else
                {
                  usbhost_vtrace1(OHCI_VTRACE1_ALREADYDISCONN, rhportst);
                }
            }

          /* Clear the status change interrupt */

          jz_putreg(OHCI_RHPORTST_CSC, regaddr);
        }

      /* Check for port reset status change */

      if ((rhportst & OHCI_RHPORTST_PRSC) != 0)
        {
          /* Release the RH port from reset */

          jz_putreg(OHCI_RHPORTST_PRSC, regaddr);
        }
    }
}

/****************************************************************************
 * Name: jz_wdh_bottomhalf
 *
 * Description:
 *   OHCI write done head interrupt handler
 *
 ****************************************************************************/

static void jz_wdh_bottomhalf(void)
{
  struct jz_eplist_s *eplist;
  struct jz_gtd_s *td;
  struct jz_gtd_s *next;
  struct jz_ed_s *ed;
  uintptr_t paddr;
  uintptr_t tmp;

  /* The host controller just wrote the finished TDs into the HCCA done head.
   * This may include multiple packets that were transferred in the preceding
   * frame.
   *
   * Remove the TD from the Writeback Done Head in the HCCA and return
   * it to the free list.  Note that this is safe because the hardware
   * will not modify the writeback done head again until the WDH bit is
   * cleared in the interrupt status register.
   */

  /* Invalidate D-cache to force re-reading of the Done Head */

#if 0 /* Apparently insufficient */
  up_invalidate_dcache((uintptr_t)&g_hcca.donehead,
                       (uintptr_t)&g_hcca.donehead + sizeof(uint32_t));
#else
  up_invalidate_dcache((uintptr_t)&g_hcca,
                       (uintptr_t)&g_hcca + sizeof(struct ohci_hcca_s));
#endif

  /* Now read the done head. */

  td = (struct jz_gtd_s *)
    jz_virtramaddr(g_hcca.donehead & HCCA_DONEHEAD_MASK);
  g_hcca.donehead = 0;

  /* Process each TD in the write done list */

  for (; td; td = next)
    {
      /* Invalidate the just-finished TD from D-cache to force it to be
       * reloaded from memory.
       */

      up_invalidate_dcache((uintptr_t)td,
                           (uintptr_t)td + sizeof(struct ohci_gtd_s));

      /* Get the ED in which this TD was enqueued */

      ed = td->ed;
      DEBUGASSERT(ed != NULL);

      /* Get the endpoint list that contains the ED */

      eplist = ed->eplist;
      DEBUGASSERT(eplist != NULL);

      /* Do nothing if there is no transfer in progress.  This situation may
       * occur after cancellation of a transfer.
       */

#ifdef CONFIG_USBHOST_ASYNCH
      if (eplist->wdhwait || eplist->callback)
#else
      if (eplist->wdhwait)
#endif
        {
          /* Invalidate the control ED so that it two will be re-read from
           * memory.
           */

          up_invalidate_dcache((uintptr_t)ed,
                               (uintptr_t)ed + sizeof(struct ohci_ed_s));

          /* Save the condition code from the (single) TD status/control
           * word.
           */

          ed->tdstatus = (td->hw.ctrl & GTD_STATUS_CC_MASK) >>
                         GTD_STATUS_CC_SHIFT;

#ifdef HAVE_USBHOST_TRACE
          if (ed->tdstatus != TD_CC_NOERROR)
            {
              /* The transfer failed for some reason... dump some diagnostic
               * info.
               */

              usbhost_trace2(OHCI_TRACE2_WHDTDSTATUS, ed->tdstatus,
                             ed->xfrtype);
            }
#endif

          /* Determine the number of bytes actually transferred.
           * A CBP value of zero means that all bytes were transferred.
           */

          tmp = (uintptr_t)td->hw.cbp;
          if (tmp == 0)
            {
              /* All bytes have been transferred */

              tmp = eplist->buflen;
            }
          else
            {
              paddr = jz_physramaddr((uintptr_t)eplist->buffer);
              DEBUGASSERT(tmp >= paddr);

              /* Determine the size of the transfer by subtracting the
               * current buffer pointer (CBP) from the initial buffer
               * pointer (on packet receipt only).
               */

              tmp -= paddr;
              DEBUGASSERT(tmp < UINT16_MAX);
            }

          eplist->xfrd = (uint16_t)tmp;
        }

      /* Return the TD to the free list */

      next = (struct jz_gtd_s *)jz_virtramaddr(td->hw.nexttd);
      jz_tdfree(td);

      /* And wake up the thread waiting for the WDH event */

      if (eplist->wdhwait)
        {
          nxsem_post(&eplist->wdhsem);
          eplist->wdhwait = false;
        }

#ifdef CONFIG_USBHOST_ASYNCH
      /* No thread waiting.  Is there a callback scheduled? */

      else if (eplist->callback)
        {
          /* Yes.. perform the callback */

          jz_asynch_completion(eplist);
        }
#endif
    }
}

/****************************************************************************
 * Name: jz_ohci_bottomhalf
 *
 * Description:
 *   OHCI interrupt bottom half.  This function runs on the high priority
 *   worker thread and was scheduled when the last interrupt occurred.  The
 *   set of pending interrupts is provided as the argument.  OHCI interrupts
 *   were disabled when this function is scheduled so no further interrupts
 *   can occur until this work re-enables OHCI interrupts
 *
 ****************************************************************************/

static void jz_ohci_bottomhalf(void *arg)
{
  uint32_t pending = (uint32_t)arg;

  /* We need to have exclusive access to the OHCI data structures.  Waiting
   * here is not a good thing to do on the worker thread, but there is no
   * real option (other than to reschedule and delay).
   */

  nxrmutex_lock(&g_ohci.lock);

  /* Root hub status change interrupt */

  if ((pending & OHCI_INT_RHSC) != 0)
    {
      /* Handle root hub status change on each root port */

      usbhost_vtrace1(OHCI_VTRACE1_RHSC, pending);
      jz_rhsc_bottomhalf();
    }

  /* Writeback Done Head interrupt */

  if ((pending & OHCI_INT_WDH) != 0)
    {
      /* The host controller just wrote the list of finished TDs into the
       * HCCA done head.  This may include multiple packets that were
       * transferred in the preceding frame.
       */

      usbhost_vtrace1(OHCI_VTRACE1_WDHINTR, pending);
      jz_wdh_bottomhalf();
    }

#ifdef CONFIG_DEBUG_USB
  if ((pending & JZ_DEBUG_INTS) != 0)
    {
      if ((pending & OHCI_INT_UE) != 0)
        {
          /* An unrecoverable error occurred.  Unrecoverable errors
           * are usually the consequence of bad descriptor contents
           * or DMA errors.
           *
           * Treat this like a normal write done head interrupt.  We
           * just want to see if there is any status information written
           * to the descriptors (and the normal write done head
           * interrupt will not be occurring).
           */

          usbhost_trace1(OHCI_TRACE1_INTRUNRECOVERABLE, pending);
          jz_wdh_bottomhalf();
        }
      else
        {
          usbhost_trace1(OHCI_TRACE1_INTRUNHANDLED, pending);
        }
    }
#endif

  /* Now re-enable interrupts */

  jz_putreg(OHCI_INT_MIE, JZ_USBHOST_INTEN);
  nxrmutex_unlock(&g_ohci.lock);
}

/****************************************************************************
 * Name: jz_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from
 *     the call to the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected
 *     the connection related event.
 *
 * Returned Value:
 *   Zero (OK) is returned on success when a device is connected or
 *   disconnected. This function will not return until either (1) a device
 *   is connected or disconnect to/from any hub port or until (2) some
 *   failure occurs.  On a failure, a negated errno value is returned
 *   indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int jz_wait(struct usbhost_connection_s *conn,
                    struct usbhost_hubport_s **hport)
{
  irqstate_t flags;
  int rhpndx;
  int ret;

  /* Loop until a change in the connection state changes on one of the root
   * hub ports or until an error occurs.
   */

  flags = enter_critical_section();
  for (; ; )
    {
      /* Check for a change in the connection state on any root hub port */

      for (rhpndx = 0; rhpndx < JZ_OHCI_NRHPORT; rhpndx++)
        {
          struct jz_rhport_s *rhport = &g_ohci.rhport[rhpndx];
          struct usbhost_hubport_s *connport;

          /* Has the connection state changed on the RH port? */

          connport = &rhport->hport.hport;
          if (rhport->connected != connport->connected)
            {
              /* Yes.. Return the RH port number to inform the caller which
               * port has the connection change.
               */

              leave_critical_section(flags);
              usbhost_vtrace2(OHCI_VTRACE2_WAKEUP,
                              rhpndx + 1, g_ohci.rhport[rhpndx].connected);

              connport->connected  = rhport->connected;
              *hport = connport;
              return OK;
            }
        }

#ifdef CONFIG_USBHOST_HUB
      /* Is a device connected to an external hub? */

      if (g_ohci.hport)
        {
          struct usbhost_hubport_s *connport;

          /* Yes.. return the external hub port */

          connport = (struct usbhost_hubport_s *)g_ohci.hport;
          g_ohci.hport = NULL;

          *hport = connport;
          leave_critical_section(flags);

          usbhost_vtrace2(OHCI_VTRACE2_HUBWAKEUP,
                          HPORT(connport), connport->connected);
          return OK;
        }
#endif

      /* No changes on any port. Wait for a connection/disconnection event
       * and check again
       */

      g_ohci.pscwait = true;

      ret = nxsem_wait_uninterruptible(&g_ohci.pscsem);
      if (ret < 0)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: jz_enumerate
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
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int jz_rh_enumerate(struct usbhost_connection_s *conn,
                            struct usbhost_hubport_s *hport)
{
  struct jz_rhport_s *rhport;
  uint32_t regaddr;
  int rhpndx;
  int ret;

  DEBUGASSERT(conn != NULL && hport != NULL);
  rhpndx = hport->port;

  DEBUGASSERT(rhpndx >= 0 && rhpndx < JZ_OHCI_NRHPORT);
  rhport = &g_ohci.rhport[rhpndx];

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!rhport->connected)
    {
      /* No, return an error */

      usbhost_vtrace1(OHCI_VTRACE1_ENUMDISCONN, RHPORT(rhport));
      return -ENODEV;
    }

  /* Add root hub EP0 to the control list */

  if (!rhport->ep0init)
    {
      ret = jz_ep0enqueue(rhport);
      if (ret < 0)
        {
          usbhost_trace2(OHCI_TRACE2_EP0ENQUEUE_FAILED, RHPORT(rhport),
                         -ret);
          return ret;
        }

      /* Successfully initialized */

      rhport->ep0init = true;
    }

  /* USB 2.0 spec says at least 50ms delay before port reset */

  up_mdelay(100);

  /* Put the root hub port in reset (the JZ4780 supports three downstream
   * ports)
   */

  regaddr = JZ_USBHOST_RHPORTST(rhpndx + 1);
  jz_putreg(OHCI_RHPORTST_PRS, regaddr);

  /* Wait for the port reset to complete */

  while ((jz_getreg(regaddr) & OHCI_RHPORTST_PRS) != 0);

  /* Release RH port 1 from reset and wait a bit */

  jz_putreg(OHCI_RHPORTST_PRSC, regaddr);
  up_mdelay(200);
  return OK;
}

static int jz_enumerate(struct usbhost_connection_s *conn,
                         struct usbhost_hubport_s *hport)
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
      ret = jz_rh_enumerate(conn, hport);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Then let the common usbhost_enumerate do the real enumeration. */

  usbhost_vtrace1(OHCI_VTRACE1_CLASSENUM, HPORT(hport));
  ret = usbhost_enumerate(hport, &hport->devclass);
  if (ret < 0)
    {
      usbhost_trace2(OHCI_TRACE2_CLASSENUM_FAILED, HPORT(hport), -ret);
    }

  return ret;
}

/****************************************************************************
 * Name: jz_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support
 *   an external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   funcaddr - The USB address of the function containing the endpoint that
 *     EP0 controls.  A funcaddr of zero will be received if no address is
 *     yet assigned to the device.
 *   speed - The speed of the port USB_SPEED_LOW, _FULL, or _HIGH
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int jz_ep0configure(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                              uint8_t funcaddr, uint8_t speed,
                              uint16_t maxpacketsize)
{
  struct jz_rhport_s *rhport = (struct jz_rhport_s *)drvr;
  struct jz_eplist_s *ep0list = (struct jz_eplist_s *)ep0;
  struct jz_ed_s *edctrl;
  uint32_t hwctrl;
  int ret;

  usbhost_vtrace2(OHCI_VTRACE2_EP0CONFIG, speed, funcaddr);
  DEBUGASSERT(rhport && maxpacketsize < 2048);

  /* Expect the device to be unplugged during enumeration */

  if (!ep0list || !ep0list->ed)
    {
      _err("Device was probably removed\n");
      return -ENOMEM;
    }

  edctrl = ep0list->ed;

  /* We must have exclusive access to EP0 and the control list */

  ret = nxrmutex_lock(&g_ohci.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the EP0 ED control word (preserving only speed) */

  hwctrl = (uint32_t)funcaddr << ED_CONTROL_FA_SHIFT |
           (uint32_t)ED_CONTROL_D_TD1 |
           (uint32_t)maxpacketsize << ED_CONTROL_MPS_SHIFT;

  if (speed == USB_SPEED_LOW)
    {
      hwctrl |= ED_CONTROL_S;
    }

  edctrl->hw.ctrl = hwctrl;

  /* Flush the modified control ED to RAM */

  up_clean_dcache((uintptr_t)edctrl,
                  (uintptr_t)edctrl + sizeof(struct ohci_ed_s));
  nxrmutex_unlock(&g_ohci.lock);

  usbhost_vtrace2(OHCI_VTRACE2_EP0CTRLED, RHPORT(rhport),
                  (uint16_t)edctrl->hw.ctrl);
  UNUSED(rhport);
  return OK;
}

/****************************************************************************
 * Name: jz_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint descriptor.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int jz_epalloc(struct usbhost_driver_s *drvr,
                       const struct usbhost_epdesc_s *epdesc,
                       usbhost_ep_t *ep)
{
  struct jz_rhport_s *rhport = (struct jz_rhport_s *)drvr;
  struct jz_eplist_s *eplist;
  struct usbhost_hubport_s *hport;
  struct jz_ed_s *ed;
  struct jz_gtd_s *td;
  uintptr_t physaddr;
  int ret  = -ENOMEM;

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(rhport != NULL && epdesc != NULL && epdesc->hport != NULL);
  DEBUGASSERT(ep != NULL && rhport->connected);
  UNUSED(rhport);

  hport = epdesc->hport;

  /* Allocate a container for the endpoint data */

  eplist = kmm_zalloc(sizeof(struct jz_eplist_s));
  if (!eplist)
    {
      usbhost_trace1(OHCI_TRACE1_EPLISTALLOC_FAILED, 0);
      goto errout;
    }

  /* Initialize the endpoint container */

  nxsem_init(&eplist->wdhsem, 0, 0);

  /* We must have exclusive access to the ED pool, the bulk list, the
   * periodic list, and the interrupt table.
   */

  ret = nxrmutex_lock(&g_ohci.lock);
  if (ret < 0)
    {
      goto errout_with_eplist;
    }

  /* Allocate an ED and a tail TD for the new endpoint */

  ed = jz_edalloc();
  if (!ed)
    {
      usbhost_trace1(OHCI_TRACE1_EDALLOC_FAILED, 0);
      goto errout_with_lock;
    }

  td = jz_tdalloc();
  if (!td)
    {
      usbhost_trace1(OHCI_TRACE1_TDALLOC_FAILED, 0);
      goto errout_with_ed;
    }

  /* Save the descriptors in the endpoint container */

  eplist->ed   = ed;
  eplist->tail = td;

  /* Configure the endpoint descriptor. */

  memset((void *)ed, 0, sizeof(struct jz_ed_s));

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

  ed->eplist = eplist;
  usbhost_vtrace2(OHCI_VTRACE2_EPALLOC, epdesc->addr, (uint16_t)ed->hw.ctrl);

  /* Configure the tail descriptor. */

  memset(td, 0, sizeof(struct jz_gtd_s));
  td->ed = ed;

  /* Link the tail TD to the ED's TD list */

  physaddr     = (uintptr_t)jz_physramaddr((uintptr_t)td);
  ed->hw.headp = physaddr;
  ed->hw.tailp = physaddr;

  /* Make sure these settings are flushed to RAM */

  up_clean_dcache((uintptr_t)ed,
                  (uintptr_t)ed + sizeof(struct ohci_ed_s));
  up_clean_dcache((uintptr_t)td,
                  (uintptr_t)td + sizeof(struct ohci_gtd_s));

  /* Now add the endpoint descriptor to the appropriate list */

  switch (ed->xfrtype)
    {
    case USB_EP_ATTR_XFER_CONTROL:
      ret = jz_addctrled(ed);
      break;

    case USB_EP_ATTR_XFER_BULK:
      ret = jz_addbulked(ed);
      break;

    case USB_EP_ATTR_XFER_INT:
      ret = jz_addinted(epdesc, ed);
      break;

    case USB_EP_ATTR_XFER_ISOC:
      ret = jz_addisoced(epdesc, ed);
      break;

    default:
      ret = -EINVAL;
      break;
    }

  /* Was the ED successfully added? */

  if (ret != OK)
    {
      /* No.. destroy it and report the error */

      usbhost_trace2(OHCI_TRACE2_EDENQUEUE_FAILED, ed->xfrtype, -ret);
      goto errout_with_td;
    }

  /* Success.. return an opaque reference to the endpoint list container */

  *ep = (usbhost_ep_t)eplist;
  nxrmutex_unlock(&g_ohci.lock);
  return OK;

errout_with_td:
  jz_tdfree(td);
errout_with_ed:
  jz_edfree(ed);
errout_with_lock:
  nxrmutex_unlock(&g_ohci.lock);
errout_with_eplist:
  kmm_free(eplist);
errout:
  return ret;
}

/****************************************************************************
 * Name: jz_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The endpoint to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int jz_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct jz_rhport_s *rhport = (struct jz_rhport_s *)drvr;
  struct jz_eplist_s *eplist = (struct jz_eplist_s *)ep;
  struct jz_ed_s *ed;
  int ret;
  int ret2;

  DEBUGASSERT(rhport != NULL && eplist != NULL &&
              eplist->ed != NULL && eplist->tail != NULL);

  /* There should not be any pending, real TDs linked to this ED */

  ed = eplist->ed;
  DEBUGASSERT((ed->hw.headp & ED_HEADP_ADDR_MASK) == ed->hw.tailp);

  /* We must have exclusive access to the ED pool, the bulk list, the
   * periodic list and the interrupt table.
   */

  ret2 = nxrmutex_lock(&g_ohci.lock);

  /* Remove the ED to the correct list depending on the transfer type */

  switch (ed->xfrtype)
    {
    case USB_EP_ATTR_XFER_CONTROL:
      ret = jz_remctrled(eplist->ed);
      break;

    case USB_EP_ATTR_XFER_BULK:
      ret = jz_rembulked(eplist->ed);
      break;

    case USB_EP_ATTR_XFER_INT:
      ret = jz_reminted(eplist->ed);
      break;

    case USB_EP_ATTR_XFER_ISOC:
      ret = jz_remisoced(eplist->ed);
      break;

    default:
      ret = -EINVAL;
      break;
    }

  /* Put the ED and tail TDs back into the free list */

  jz_edfree(eplist->ed);
  jz_tdfree(eplist->tail);

  /* And free the container */

  nxsem_destroy(&eplist->wdhsem);
  kmm_free(eplist);
  nxrmutex_unlock(&g_ohci.lock);
  return ret < 0 ? ret : ret2;
}

/****************************************************************************
 * Name: jz_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to allocate the request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_malloc.
 *
 *   This interface was optimized under a particular assumption.  It was
 *   assumed that the driver maintains a pool of small, pre-allocated
 *   buffers for descriptor traffic.  NOTE that size is not an input, but
 *   an output:  The size of the pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *     which to return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in
 *     which to return the maximum size of the allocated buffer memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int jz_alloc(struct usbhost_driver_s *drvr,
                     uint8_t **buffer, size_t *maxlen)
{
  int ret;

  DEBUGASSERT(drvr && buffer && maxlen);

  /* We must have exclusive access to the transfer buffer pool */

  ret = nxrmutex_lock(&g_ohci.lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = -ENOMEM;

  *buffer = jz_tballoc();
  if (*buffer)
    {
      *maxlen = CONFIG_JZ4780_OHCI_TDBUFSIZE;
      ret = OK;
    }

  nxrmutex_unlock(&g_ohci.lock);
  return ret;
}

/****************************************************************************
 * Name: jz_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to free that request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int jz_free(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  int ret;

  DEBUGASSERT(drvr && buffer);

  /* We must have exclusive access to the transfer buffer pool */

  ret = nxrmutex_lock(&g_ohci.lock);
  jz_tbfree(buffer);
  nxrmutex_unlock(&g_ohci.lock);
  return ret;
}

/****************************************************************************
 * Name: jz_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to
 *   allocate the request/descriptor memory.  If the underlying hardware
 *   does not support such "special" memory, this functions may simply map
 *   to kumm_malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are
 *   variable-sized.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *     which to return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int jz_ioalloc(struct usbhost_driver_s *drvr, uint8_t **buffer,
                       size_t buflen)
{
  DEBUGASSERT(drvr && buffer);

  /* kumm_malloc() should return user accessible, DMA-able memory */

  *buffer = kumm_malloc(buflen);
  return *buffer ? OK : -ENOMEM;
}

/****************************************************************************
 * Name: jz_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed
 *   more efficiently.  This method provides a mechanism to free that IO
 *   buffer memory.  If the underlying hardware does not support such
 *   "special" memory, this functions may simply map to kumm_free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int jz_iofree(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

  /* kumm_free is all that is required */

  kumm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: jz_ctrlin and jz_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one
 *   transfer may be queued; Neither these methods nor the transfer() method
 *   can be called again until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep0 - The control endpoint to send/receive the control request.
 *   req - Describes the request to be sent.  This request must lie in
 *     memory created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using
 *     DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same
 *   allocated memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int jz_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                      const struct usb_ctrlreq_s *req,
                      uint8_t *buffer)
{
  struct jz_rhport_s *rhport = (struct jz_rhport_s *)drvr;
  struct jz_eplist_s *eplist = (struct jz_eplist_s *)ep0;
  uint16_t len;
  int ret;

  DEBUGASSERT(rhport != NULL && eplist != NULL && req != NULL);

#ifdef CONFIG_USBHOST_TRACE
  usbhost_vtrace2(OHCI_VTRACE2_CTRLIN, RHPORT(rhport), req->req);
#else
  uinfo("RHPort%d type: %02x req: %02x value: %02x%02x index: %02x%02x "
        "len: %02x%02x\n",
        RHPORT(rhport), req->type, req->req, req->value[1],
        req->value[0], req->index[1], req->index[0], req->len[1],
        req->len[0]);
#endif

  /* We must have exclusive access to EP0 and the control list */

  ret = nxrmutex_lock(&g_ohci.lock);
  if (ret < 0)
    {
      return ret;
    }

  len = jz_getle16(req->len);
  ret = jz_ctrltd(rhport, eplist, GTD_STATUS_DP_SETUP, (uint8_t *)req,
                   USB_SIZEOF_CTRLREQ);
  if (ret == OK)
    {
      if (len)
        {
          ret = jz_ctrltd(rhport, eplist, GTD_STATUS_DP_IN, buffer, len);
        }

      if (ret == OK)
        {
          ret = jz_ctrltd(rhport, eplist, GTD_STATUS_DP_OUT, NULL, 0);
        }
    }

  /* On an IN transaction, we need to invalidate the buffer contents to force
   * it to be reloaded from RAM after the DMA.
   */

  nxrmutex_unlock(&g_ohci.lock);
  up_invalidate_dcache((uintptr_t)buffer, (uintptr_t)buffer + len);
  return ret;
}

static int jz_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                       const struct usb_ctrlreq_s *req,
                       const uint8_t *buffer)
{
  struct jz_rhport_s *rhport = (struct jz_rhport_s *)drvr;
  struct jz_eplist_s *eplist = (struct jz_eplist_s *)ep0;
  uint16_t len;
  int ret;

  DEBUGASSERT(rhport != NULL && eplist != NULL && req != NULL);

#ifdef CONFIG_USBHOST_TRACE
  usbhost_vtrace2(OHCI_VTRACE2_CTRLOUT, RHPORT(rhport), req->req);
#else
  uinfo("RHPort%d type: %02x req: %02x value: %02x%02x index: %02x%02x "
        "len: %02x%02x\n",
        RHPORT(rhport), req->type, req->req, req->value[1],
        req->value[0], req->index[1], req->index[0], req->len[1],
        req->len[0]);
#endif

  /* We must have exclusive access to EP0 and the control list */

  ret = nxrmutex_lock(&g_ohci.lock);
  if (ret < 0)
    {
      return ret;
    }

  len = jz_getle16(req->len);
  ret = jz_ctrltd(rhport, eplist, GTD_STATUS_DP_SETUP, (uint8_t *)req,
                   USB_SIZEOF_CTRLREQ);
  if (ret == OK)
    {
      if (len)
        {
          ret = jz_ctrltd(rhport, eplist, GTD_STATUS_DP_OUT,
                           (uint8_t *)buffer, len);
        }

      if (ret == OK)
        {
          ret = jz_ctrltd(rhport, eplist, GTD_STATUS_DP_IN, NULL, 0);
        }
    }

  nxrmutex_unlock(&g_ohci.lock);
  return ret;
}

/****************************************************************************
 * Name: jz_transfer_common
 *
 * Description:
 *   Initiate a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately
 *
 * Input Parameters:
 *   rhport - Internal driver root hub port state structure.
 *   eplist - The internal representation of the device endpoint on which
 *     to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *     received (IN endpoint).  buffer must have been allocated using
 *     DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int jz_transfer_common(struct jz_rhport_s *rhport,
                               struct jz_eplist_s *eplist,
                               uint8_t *buffer, size_t buflen)
{
  struct jz_ed_s *ed;
  uint32_t dirpid;
  uint32_t regval;
  bool in;
  int ret;

  ed = eplist->ed;
  in = (ed->hw.ctrl & ED_CONTROL_D_MASK) == ED_CONTROL_D_IN;

#ifdef CONFIG_USBHOST_TRACE
  usbhost_vtrace2(OHCI_VTRACE2_TRANSFER,
                  (ed->hw.ctrl  & ED_CONTROL_EN_MASK) >> ED_CONTROL_EN_SHIFT,
                  (uint16_t)buflen);
#else
  uinfo("EP%" PRId32 " %s toggle: %d maxpacket: %" PRId32 " buflen: %zd\n",
        (ed->hw.ctrl  & ED_CONTROL_EN_MASK) >> ED_CONTROL_EN_SHIFT,
        in ? "IN" : "OUT",
        (ed->hw.headp & ED_HEADP_C) != 0 ? 1 : 0,
        (ed->hw.ctrl  & ED_CONTROL_MPS_MASK) >> ED_CONTROL_MPS_SHIFT,
        buflen);
#endif

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

  ed->tdstatus = TD_CC_NOERROR;
  ret = jz_enqueuetd(rhport, eplist, ed, dirpid, GTD_STATUS_T_TOGGLE,
                      buffer, buflen);
  if (ret == OK)
    {
      /* BulkListFilled. This bit is used to indicate whether there are any
       * TDs on the Bulk list.
       */

      if (ed->xfrtype == USB_EP_ATTR_XFER_BULK)
        {
          regval  = jz_getreg(JZ_USBHOST_CMDST);
          regval |= OHCI_CMDST_BLF;
          jz_putreg(regval, JZ_USBHOST_CMDST);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: jz_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request, blocking until the transfer completes.
 *   Only one transfer may be  queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on
 *     which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *     received (IN endpoint).  buffer must have been allocated using
 *     DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure:
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
 ****************************************************************************/

static ssize_t jz_transfer(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                            uint8_t *buffer, size_t buflen)
{
  struct jz_rhport_s *rhport = (struct jz_rhport_s *)drvr;
  struct jz_eplist_s *eplist = (struct jz_eplist_s *)ep;
  struct jz_ed_s *ed;
  ssize_t nbytes;
  bool in;
  int ret;
  int ret2;

  DEBUGASSERT(rhport && eplist && eplist->ed && eplist->tail &&
              buffer && buflen > 0);

  ed = eplist->ed;
  in = (ed->hw.ctrl & ED_CONTROL_D_MASK) == ED_CONTROL_D_IN;

  /* We must have exclusive access to the endpoint, the TD pool, the I/O
   * buffer pool, the bulk and interrupt lists, and the HCCA interrupt
   * table.
   */

  ret = nxrmutex_lock(&g_ohci.lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Set the request for the Writeback Done Head event well BEFORE enabling
   * the transfer.
   */

  ret = jz_wdhwait(rhport, ed, buffer, buflen);
  if (ret != OK)
    {
      usbhost_trace1(OHCI_TRACE1_DEVDISCONN, RHPORT(rhport));
      goto errout;
    }

  /* Set up the transfer */

  ret = jz_transfer_common(rhport, eplist, buffer, buflen);
  if (ret < 0)
    {
      uerr("ERROR: jz_transfer_common failed: %d\n", ret);
      goto errout;
    }

  /* Release the OHCI semaphore while we wait.  Other threads need the
   * opportunity to access the OHCI resources while we wait.
   *
   * REVISIT:  Is this safe?  NO.  This is a bug and needs rethinking.
   * We need to lock all of the port-resources (not OHCI common) until
   * the transfer is complete.  But we can't use the common OHCI lock
   * or we will deadlock while waiting (because the working thread that
   * wakes this thread up needs the lock).
   */

#warning REVISIT
  nxrmutex_unlock(&g_ohci.lock);

  /* Wait for the Writeback Done Head interrupt.  Loop to handle any false
   * alarm semaphore counts.
   */

  while (eplist->wdhwait && ret >= 0)
    {
      ret = nxsem_wait_uninterruptible(&eplist->wdhsem);
    }

  /* Re-acquire the OHCI semaphore.  The caller expects to be holding
   * this upon return.
   */

  ret2 = nxrmutex_lock(&g_ohci.lock);
  if (ret2 < 0)
    {
      ret = ret2;
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Invalidate the D cache to force the ED to be reloaded from RAM */

  up_invalidate_dcache((uintptr_t)ed,
                       (uintptr_t)ed + sizeof(struct ohci_ed_s));

  /* Check the TD completion status bits */

  if (ed->tdstatus == TD_CC_NOERROR)
    {
      /* On an IN transaction, we also need to invalidate the buffer
       * contents to force it to be reloaded from RAM.
       */

      if (in)
        {
          up_invalidate_dcache((uintptr_t)buffer,
                               (uintptr_t)buffer + buflen);
        }

      nbytes = eplist->xfrd;
      DEBUGASSERT(nbytes >= 0 && nbytes <= buflen);

      nxrmutex_unlock(&g_ohci.lock);
      return nbytes;
    }

  /* A transfer error occurred */

  usbhost_trace2(OHCI_TRACE2_BADTDSTATUS, RHPORT(rhport), ed->tdstatus);
  switch (ed->tdstatus)
    {
    case TD_CC_STALL:
      ret = -EPERM;
      break;

    case TD_CC_USER:
      ret = -ESHUTDOWN;
      break;

    default:
      ret = -EIO;
      break;
    }

errout:

  /* Make sure that there is no outstanding request on this endpoint */

  eplist->wdhwait = false;
  nxrmutex_unlock(&g_ohci.lock);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: jz_asynch_completion
 *
 * Description:
 *   This function is called at the interrupt level when an asynchronous
 *   transfer completes.  It performs the pending callback.
 *
 * Input Parameters:
 *   rhport - Internal driver root hub port state structure.
 *   eplist - The internal representation of the device endpoint on which
 *     to perform the transfer.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Called from the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void jz_asynch_completion(struct jz_eplist_s *eplist)
{
  struct jz_ed_s *ed;
  usbhost_asynch_t callback;
  void *arg;
  ssize_t nbytes;

  DEBUGASSERT(eplist->ed && eplist->tail && eplist->callback != NULL &&
              eplist->buffer != NULL && eplist->buflen > 0);
  ed = eplist->ed;

  /* Invalidate the D cache to force the ED to be reloaded from RAM */

  up_invalidate_dcache((uintptr_t)ed,
                       (uintptr_t)ed + sizeof(struct ohci_ed_s));

  /* Check the TD completion status bits */

  if (ed->tdstatus == TD_CC_NOERROR)
    {
      /* On an IN transaction, we also need to invalidate the buffer
       * contents to force it to be reloaded from RAM.
       */

      if ((ed->hw.ctrl & ED_CONTROL_D_MASK) == ED_CONTROL_D_IN)
        {
          uintptr_t buffaddr = (uintptr_t)eplist->buffer;
          up_invalidate_dcache(buffaddr, buffaddr + eplist->buflen);
        }

      nbytes = eplist->xfrd;
      DEBUGASSERT(nbytes >= 0 && nbytes <= eplist->buflen);
    }
  else
    {
      /* Map the bad completion status to something that a class driver
       * might understand.
       */

      usbhost_trace1(OHCI_TRACE1_BADTDSTATUS, ed->tdstatus);

      switch (ed->tdstatus)
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

  /* Extract the callback information before freeing the buffer */

  callback = eplist->callback;
  arg      = eplist->arg;

  /* Clear any pending transfer indicators */

  eplist->wdhwait  = false;
  eplist->callback = NULL;
  eplist->arg      = NULL;
  eplist->buffer   = NULL;
  eplist->buflen   = 0;

  /* Then perform the callback */

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: jz_asynch
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  When the transfer
 *   completes, the callback will be invoked with the provided transfer.
 *   This method is useful for receiving interrupt transfers which may come
 *   infrequently.
 *
 *   Only one transfer may be queued; Neither this method nor the ctrlin or
 *   ctrlout methods can be called again until the transfer completes.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on
 *     which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *     received (IN endpoint).  buffer must have been allocated using
 *     DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *   callback - This function will be called when the transfer completes.
 *   arg - The arbitrary parameter that will be passed to the callback
 *     function when the transfer completes.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int jz_asynch(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                      uint8_t *buffer, size_t buflen,
                      usbhost_asynch_t callback, void *arg)
{
  struct jz_rhport_s *rhport = (struct jz_rhport_s *)drvr;
  struct jz_eplist_s *eplist = (struct jz_eplist_s *)ep;
  struct jz_ed_s *ed;
  int ret;

  DEBUGASSERT(rhport && eplist && eplist->ed && eplist->tail &&
              buffer && buflen > 0 && buflen <= UINT16_MAX);
  ed = eplist->ed;

  /* We must have exclusive access to the endpoint, the TD pool, the I/O
   * buffer pool, the bulk and interrupt lists, and the HCCA interrupt
   * table.
   */

  ret = nxrmutex_lock(&g_ohci.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the request for the Writeback Done Head callback well BEFORE
   * enabling the transfer.
   */

  ret = jz_wdhasynch(rhport, ed, callback, arg, buffer, buflen);
  if (ret != OK)
    {
      usbhost_trace1(OHCI_TRACE1_DEVDISCONN, RHPORT(rhport));
      goto errout;
    }

  /* Set up the transfer */

  ret = jz_transfer_common(rhport, eplist, buffer, buflen);
  if (ret < 0)
    {
      uerr("ERROR: jz_transfer_common failed: %d\n", ret);
      goto errout;
    }

  /* Then just return.  The callback will be performed asynchronously
   * when the transfer completes.
   */

  nxrmutex_unlock(&g_ohci.lock);
  return OK;

errout:

  /* Make sure that there is no outstanding request on this endpoint */

  eplist->callback = NULL;
  eplist->arg      = NULL;
  nxrmutex_unlock(&g_ohci.lock);
  return ret;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/****************************************************************************
 * Name: jz_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Cancelled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which
 *     an asynchronous transfer should be transferred.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 ****************************************************************************/

static int jz_cancel(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct jz_eplist_s *eplist = (struct jz_eplist_s *)ep;
  struct jz_ed_s *ed;
  struct jz_gtd_s *td;
  struct jz_gtd_s *next;
  irqstate_t flags;
  uintptr_t paddr;
  uint32_t ctrl;

  DEBUGASSERT(eplist && eplist->ed && eplist->tail);
  ed = eplist->ed;

  /* These first steps must be atomic as possible */

  flags  = enter_critical_section();

  /* It might be possible for no transfer to be in progress (callback == NULL
   * and wdhwait == false)
   */

#ifdef CONFIG_USBHOST_ASYNCH
  if (eplist->callback || eplist->wdhwait)
#else
  if (eplist->wdhwait)
#endif
    {
      /* Control endpoints should not come through this path and
       * isochronous endpoints are not yet implemented.  So we only have
       * to distinguish bulk and interrupt endpoints.
       */

      if (ed->xfrtype == USB_EP_ATTR_XFER_BULK)
        {
          /* Disable bulk list processing while we modify the list */

          ctrl  = jz_getreg(JZ_USBHOST_CTRL);
          jz_putreg(ctrl & ~OHCI_CTRL_BLE, JZ_USBHOST_CTRL);

          /* Remove the TDs attached to the ED, keeping the ED in the list */

          paddr        = ed->hw.headp & ED_HEADP_ADDR_MASK;
          td           = (struct jz_gtd_s *)jz_virtramaddr(paddr);

          paddr        = jz_physramaddr((uintptr_t)eplist->tail);
          ed->hw.headp = paddr;

          up_clean_dcache((uintptr_t)ed,
                          (uintptr_t)ed + sizeof(struct ohci_ed_s));

          /* Re-enable bulk list processing, if it was enabled before */

          jz_putreg(0, JZ_USBHOST_BULKED);
          jz_putreg(ctrl, JZ_USBHOST_CTRL);
        }
      else
        {
          /* Remove the TDs attached to the ED, keeping the Ed in the list */

          paddr        = ed->hw.headp & ED_HEADP_ADDR_MASK;
          td           = (struct jz_gtd_s *)jz_virtramaddr(paddr);

          paddr        = jz_physramaddr((uintptr_t)eplist->tail);
          ed->hw.headp = paddr;

          up_clean_dcache((uintptr_t)ed,
                          (uintptr_t)ed + sizeof(struct ohci_ed_s));
        }

      /* Free all transfer descriptors that were connected to the ED.  In
       * some race conditions with the hardware, this might be none.
       */

      while (td != (struct jz_gtd_s *)eplist->tail)
        {
          paddr = (uintptr_t)td->hw.nexttd;
          next  = (struct jz_gtd_s *)jz_virtramaddr(paddr);
          jz_tdfree(td);
          td    = next;
        }

      ed->tdstatus = TD_CC_USER;

      /* If there is a thread waiting for the transfer to complete, then
       * wake up the thread.
       */

      if (eplist->wdhwait)
        {
#ifdef CONFIG_USBHOST_ASYNCH
          /* Yes.. there should not also be a callback scheduled */

          DEBUGASSERT(eplist->callback == NULL);
#endif

          /* Wake up the waiting thread */

          nxsem_post(&eplist->wdhsem);
          eplist->wdhwait = false;
        }
#ifdef CONFIG_USBHOST_ASYNCH
      else
        {
          /* Otherwise, perform the callback */

          jz_asynch_completion(eplist);
        }
#endif
    }

  /* Reset any pending activity indications */

  eplist->wdhwait  = false;
#ifdef CONFIG_USBHOST_ASYNCH
  eplist->callback = NULL;
  eplist->arg      = NULL;
#endif
  eplist->buffer   = NULL;
  eplist->buflen   = 0;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: jz_connect
 *
 * Description:
 *   New connections may be detected by an attached hub.  This method is the
 *   mechanism that is used by the hub class to introduce a new connection
 *   and port description to the system.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   hport - The descriptor of the hub port that detected the connection
 *     related event
 *   connected - True: device connected; false: device disconnected
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int jz_connect(struct usbhost_driver_s *drvr,
                       struct usbhost_hubport_s *hport,
                       bool connected)
{
  irqstate_t flags;

  /* Set the connected/disconnected flag */

  hport->connected = connected;
  uinfo("Hub port %d connected: %s\n",
        hport->port, connected ? "YES" : "NO");

  /* Report the connection event */

  flags = enter_critical_section();
  DEBUGASSERT(g_ohci.hport == NULL); /* REVISIT */

  g_ohci.hport = hport;
  if (g_ohci.pscwait)
    {
      g_ohci.pscwait = false;
      nxsem_post(&g_ohci.pscsem);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: jz_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and driver has been
 *   disconnected.  The USB host driver should discard the handle to the
 *   class instance (it is stale) and not attempt any further interaction
 *   with the class driver instance (until a new instance is received from
 *   the create() method).  The driver should not called the class'
 *   disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *     call to the class create() method.
 *   hport - The port from which the device is being disconnected.  Might be
 *     a port on a hub.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static void jz_disconnect(struct usbhost_driver_s *drvr,
                           struct usbhost_hubport_s *hport)
{
  struct jz_rhport_s *rhport = (struct jz_rhport_s *)drvr;
  struct jz_eplist_s *ep0;

  DEBUGASSERT(rhport != NULL && hport != NULL && hport->ep0);
  ep0 = (struct jz_eplist_s *)hport->ep0;

  /* Did we just dequeue EP0 from a root hub port? */

  if (ROOTHUB(hport))
    {
      /* Remove the disconnected port from the control list */

      jz_ep0dequeue(ep0);
      rhport->ep0init = false;
    }

  /* Unbind the class from the port */

  hport->devclass = NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jz_ohci_initialize
 *
 * Description:
 *   Initialize USB OHCI host controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than one OHCI interface, then
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
 ****************************************************************************/

struct usbhost_connection_s *jz_ohci_initialize(int controller)
{
  struct usbhost_hubport_s *hport;
  uintptr_t physaddr;
  uint32_t regval;
  uint8_t *buffer;
  irqstate_t flags;
  int i;

  /* One time sanity checks */

  DEBUGASSERT(controller == 0);
  DEBUGASSERT(sizeof(struct jz_ed_s)  == SIZEOF_JZ_ED_S);
  DEBUGASSERT(sizeof(struct jz_gtd_s) == SIZEOF_JZ_TD_S);

#ifndef CONFIG_USBHOST_INT_DISABLE
  g_ohci.ininterval  = MAX_PERINTERVAL;
  g_ohci.outinterval = MAX_PERINTERVAL;
#endif

  /* Enable UHP peripheral clocking */

  flags   = enter_critical_section();

  leave_critical_section(flags);

  /* Note that no pin configuration is required.  All USB HS pins have
   * dedicated function
   */

  usbhost_vtrace1(OHCI_VTRACE1_INITIALIZING, 0);

  /* Initialize all the HCCA to 0 */

  memset((void *)&g_hcca, 0, sizeof(struct ohci_hcca_s));

  up_clean_dcache((uint32_t)&g_hcca,
                  (uint32_t)&g_hcca + sizeof(struct ohci_hcca_s));

  /* Initialize user-configurable EDs */

  for (i = 0; i < JZ4780_OHCI_NEDS; i++)
    {
      /* Put the ED in a free list */

      jz_edfree(&g_edalloc[i]);
    }

  /* Initialize user-configurable TDs */

  for (i = 0; i < JZ4780_OHCI_NTDS; i++)
    {
      /* Put the TD in a free list */

      jz_tdfree(&g_tdalloc[i]);
    }

  /* Initialize user-configurable request/descriptor transfer buffers */

  buffer = g_bufalloc;
  for (i = 0; i < CONFIG_JZ4780_OHCI_TDBUFFERS; i++)
    {
      /* Put the TD buffer in a free list */

      jz_tbfree(buffer);
      buffer += CONFIG_JZ4780_OHCI_TDBUFSIZE;
    }

  /* Initialize function address generation logic */

  usbhost_devaddr_initialize(&g_ohci.devgen);

  /* Initialize the root hub port structures */

  for (i = 0; i < JZ_OHCI_NRHPORT; i++)
    {
      struct jz_rhport_s *rhport = &g_ohci.rhport[i];

      /* Initialize the device operations */

      rhport->drvr.ep0configure   = jz_ep0configure;
      rhport->drvr.epalloc        = jz_epalloc;
      rhport->drvr.epfree         = jz_epfree;
      rhport->drvr.alloc          = jz_alloc;
      rhport->drvr.free           = jz_free;
      rhport->drvr.ioalloc        = jz_ioalloc;
      rhport->drvr.iofree         = jz_iofree;
      rhport->drvr.ctrlin         = jz_ctrlin;
      rhport->drvr.ctrlout        = jz_ctrlout;
      rhport->drvr.transfer       = jz_transfer;
#ifdef CONFIG_USBHOST_ASYNCH
      rhport->drvr.asynch         = jz_asynch;
#endif
      rhport->drvr.cancel         = jz_cancel;
#ifdef CONFIG_USBHOST_HUB
      rhport->drvr.connect        = jz_connect;
#endif
      rhport->drvr.disconnect     = jz_disconnect;
      rhport->hport.pdevgen       = &g_ohci.devgen;

      /* Initialize the public port representation */

      hport                       = &rhport->hport.hport;
      hport->drvr                 = &rhport->drvr;
#ifdef CONFIG_USBHOST_HUB
      hport->parent               = NULL;
#endif
      hport->ep0                  = &rhport->ep0;
      hport->port                 = i;
      hport->speed                = USB_SPEED_FULL;
      hport->funcaddr             = 0;
    }

  /* Wait 50MS then perform hardware reset */

  up_mdelay(50);

  jz_putreg(0, JZ_USBHOST_CTRL);        /* Hardware reset */
  jz_putreg(0, JZ_USBHOST_CTRLHEADED);  /* Initialize control list head to Zero */
  jz_putreg(0, JZ_USBHOST_BULKHEADED);  /* Initialize bulk list head to Zero */

  /* Software reset */

  jz_putreg(OHCI_CMDST_HCR, JZ_USBHOST_CMDST);

  /* Write Fm interval (FI), largest data packet counter (FSMPS), and
   * periodic start.
   */

  jz_putreg(DEFAULT_FMINTERVAL, JZ_USBHOST_FMINT);
  jz_putreg(DEFAULT_PERSTART, JZ_USBHOST_PERSTART);

  /* Put HC in operational state */

  regval  = jz_getreg(JZ_USBHOST_CTRL);
  regval &= ~OHCI_CTRL_HCFS_MASK;
  regval |= OHCI_CTRL_HCFS_OPER;
  jz_putreg(regval, JZ_USBHOST_CTRL);

  /* Set global power in HcRhStatus */

  jz_putreg(OHCI_RHSTATUS_SGP, JZ_USBHOST_RHSTATUS);

  /* Set HCCA base address */

  physaddr = jz_physramaddr((uintptr_t)&g_hcca);
  jz_putreg(physaddr, JZ_USBHOST_HCCA);

  /* Clear pending interrupts */

  regval = jz_getreg(JZ_USBHOST_INTST);
  jz_putreg(regval, JZ_USBHOST_INTST);

  /* Enable OHCI interrupts */

  jz_putreg((JZ_ALL_INTS | OHCI_INT_MIE), JZ_USBHOST_INTEN);

  /* Attach USB host controller interrupt handler.  If EHCI is enabled,
   * then it will manage the shared interrupt.
   */

  if (irq_attach(JZ4780_IRQ_OHCI, jz_ohci_tophalf, NULL) != 0)
    {
      usbhost_trace1(OHCI_TRACE1_IRQATTACH, JZ4780_IRQ_OHCI);
      return NULL;
    }

  /* Drive Vbus +5V (the smoke test). */

  up_mdelay(50);

  /* If there is a USB device in the slot at power up, then we will not
   * get the status change interrupt to signal us that the device is
   * connected.  We need to set the initial connected state accordingly.
   */

  for (i = 0; i < JZ_OHCI_NRHPORT; i++)
    {
      regval                     = jz_getreg(JZ_USBHOST_RHPORTST(i));
      g_ohci.rhport[i].connected = ((regval & OHCI_RHPORTST_CCS) != 0);

      usbhost_vtrace2(OHCI_VTRACE2_INITCONNECTED,
                      i + 1, g_ohci.rhport[i].connected);
    }

  up_enable_irq(JZ4780_IRQ_OHCI); /* enable USB interrupt */

  usbhost_vtrace1(OHCI_VTRACE1_INITIALIZED, 0);

  return &g_ohciconn;
}

/****************************************************************************
 * Name: jz_ohci_tophalf
 *
 * Description:
 *   OHCI "Top Half" interrupt handler.
 *
 ****************************************************************************/

static int jz_ohci_tophalf(int irq, void *context, void *arg)
{
  uint32_t intst;
  uint32_t inten;
  uint32_t pending;

  /* Read Interrupt Status and mask out interrupts that are not enabled. */

  intst = jz_getreg(JZ_USBHOST_INTST);
  inten = jz_getreg(JZ_USBHOST_INTEN);
  usbhost_vtrace1(OHCI_VTRACE1_INTRPENDING, intst & inten);

    {
      /* Mask out the interrupts that are not enabled */

      pending = intst & inten;
      if (pending != 0)
        {
          /* Schedule interrupt handling work for the high priority worker
           * thread so that we are not pressed for time and so that we can
           * interrupt with other USB threads gracefully.
           *
           * The worker should be available now because we implement a
           * handshake by controlling the OHCI interrupts.
           */

          DEBUGASSERT(work_available(&g_ohci.work));
          DEBUGVERIFY(work_queue(HPWORK, &g_ohci.work, jz_ohci_bottomhalf,
                                 (void *)pending, 0));

          /* Disable further OHCI interrupts so that we do not overrun the
           * work queue.
           */

          jz_putreg(OHCI_INT_MIE, JZ_USBHOST_INTDIS);

          /* Clear all pending status bits by writing the value of the
           * pending interrupt bits back to the status register.
           */

          jz_putreg(intst, JZ_USBHOST_INTST);
        }
    }

  return OK;
}

#endif /* CONFIG_JZ4780_OHCI */
