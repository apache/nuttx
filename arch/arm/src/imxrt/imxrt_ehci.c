/****************************************************************************
 * arch/arm/src/imxrt/imxrt_ehci.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/ehci.h>
#include <nuttx/usb/usbhost_devaddr.h>
#include <nuttx/usb/usbhost_trace.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/imxrt_usbotg.h"
#include "imxrt_periphclks.h"

#if defined(CONFIG_IMXRT_USBOTG) && defined(CONFIG_USBHOST)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Pre-requisites */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#elif !defined(CONFIG_SCHED_HPWORK)
#  error Hi-priority work queue support is required (CONFIG_SCHED_HPWORK)
#endif

/* Configurable number of Queue Head (QH) structures.  The default is one per
 * Root hub port plus one for EP0.
 */

#ifndef CONFIG_IMXRT_EHCI_NQHS
#  define CONFIG_IMXRT_EHCI_NQHS (IMXRT_EHCI_NRHPORT + 1)
#endif

/* Configurable number of Queue Element Transfer Descriptor (qTDs).  The
 * default is one per root hub plus three from EP0.
 */

#ifndef CONFIG_IMXRT_EHCI_NQTDS
#  define CONFIG_IMXRT_EHCI_NQTDS (IMXRT_EHCI_NRHPORT + 3)
#endif

/* Buffers must be aligned to the cache line size */

#define DCACHE_LINEMASK (ARMV7M_DCACHE_LINESIZE -1)

/* Configurable size of a request/descriptor buffers */

#ifndef CONFIG_IMXRT_EHCI_BUFSIZE
#  define CONFIG_IMXRT_EHCI_BUFSIZE 128
#endif

#define IMXRT_EHCI_BUFSIZE \
  ((CONFIG_IMXRT_EHCI_BUFSIZE + DCACHE_LINEMASK) & ~DCACHE_LINEMASK)

/* Debug options */

#ifndef CONFIG_DEBUG_USB_INFO
#  undef CONFIG_IMXRT_EHCI_REGDEBUG
#endif

/* Isochronous transfers are not currently supported */

#undef CONFIG_USBHOST_ISOC_DISABLE
#define CONFIG_USBHOST_ISOC_DISABLE 1

/* Registers ****************************************************************
 * Traditionally, NuttX specifies register locations using individual
 * register offsets from a base address.  That tradition is broken here and,
 * instead, register blocks are represented as structures.  This is done here
 * because, in principle, EHCI operational register address may not be known
 * at compile time; the operational registers lie at an offset specified in
 * the 'caplength' byte of the Host Controller Capability Registers.
 *
 * However, for the case of the IMXRT EHCI, we know apriori that locations
 * of these register blocks.
 */

/* Host Controller Capability Registers */

#define HCCR ((struct ehci_hccr_s *)IMXRT_USBOTG_HCCR_BASE)

/* Host Controller Operational Registers */

#define HCOR ((volatile struct ehci_hcor_s *)IMXRT_USBOTG_HCOR_BASE)

/* Interrupts ***************************************************************
 * This is the set of interrupts handled by this driver.
 */

#define EHCI_HANDLED_INTS (EHCI_INT_USBINT | EHCI_INT_USBERRINT | \
                           EHCI_INT_PORTSC |  EHCI_INT_SYSERROR | \
                           EHCI_INT_AAINT)

/* The periodic frame list is a 4K-page aligned array of Frame List Link
 * pointers. The length of the frame list may be programmable.  The
 * programmability of the periodic frame list is exported to system software
 * via the HCCPARAMS register. If non-programmable, the length is 1024
 * elements. If programmable, the length can be selected by system software
 * as one of 256, 512, or 1024 elements.
 */

#define FRAME_LIST_SIZE 1024

/* DMA **********************************************************************/

/* For now, we are assuming an identity mapping between physical and virtual
 * address spaces.
 */

#define imxrt_physramaddr(a) (a)
#define imxrt_virtramaddr(a) (a)

/* USB trace ****************************************************************/

#ifdef HAVE_USBHOST_TRACE
#  define TR_FMT1 false
#  define TR_FMT2 true

#  define TRENTRY(id,fmt1,string) {string}

#  define TRACE1_FIRST     ((int)__TRACE1_BASEVALUE + 1)
#  define TRACE1_INDEX(id) ((int)(id) - TRACE1_FIRST)
#  define TRACE1_NSTRINGS  TRACE1_INDEX(__TRACE1_NSTRINGS)

#  define TRACE2_FIRST     ((int)__TRACE1_NSTRINGS + 1)
#  define TRACE2_INDEX(id) ((int)(id) - TRACE2_FIRST)
#  define TRACE2_NSTRINGS  TRACE2_INDEX(__TRACE2_NSTRINGS)
#endif

/* Port numbers */

#define RHPNDX(rh)            ((rh)->hport.hport.port)
#define RHPORT(rh)            (RHPNDX(rh)+1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Internal representation of the EHCI Queue Head (QH) */

struct imxrt_epinfo_s;
struct imxrt_qh_s
{
  /* Fields visible to hardware */

  struct ehci_qh_s hw;           /* Hardware representation of the queue head */

  /* Internal fields used by the EHCI driver */

  struct imxrt_epinfo_s *epinfo; /* Endpoint used for the transfer */
  uint32_t fqp;                  /* First qTD in the list (physical address) */
  uint8_t pad[8];                /* Padding to assure 32-byte alignment */
};

/* Internal representation of the EHCI Queue Element Transfer Descriptor
 * (qTD)
 */

struct imxrt_qtd_s
{
  /* Fields visible to hardware */

  struct ehci_qtd_s hw;          /* Hardware representation of the queue head */

  /* Internal fields used by the EHCI driver */
};

/* The following is used to manage lists of free QHs and qTDs */

struct imxrt_list_s
{
  struct imxrt_list_s *flink;    /* Link to next entry in the list
                                  * Variable length entry data follows
                                  */
};

/* List traversal call-out functions */

typedef int (*foreach_qh_t)(struct imxrt_qh_s *qh, uint32_t **bp,
                            void *arg);
typedef int (*foreach_qtd_t)(struct imxrt_qtd_s *qtd, uint32_t **bp,
                             void *arg);

/* This structure describes one endpoint. */

struct imxrt_epinfo_s
{
  uint8_t epno:7;                /* Endpoint number */
  uint8_t dirin:1;               /* 1:IN endpoint 0:OUT endpoint */
  uint8_t devaddr:7;             /* Device address */
  uint8_t toggle:1;              /* Next data toggle */
#ifndef CONFIG_USBHOST_INT_DISABLE
  uint8_t interval;              /* Polling interval */
#endif
  uint8_t status;                /* Retained token status bits (for debug purposes) */
  volatile bool iocwait;         /* TRUE: Thread is waiting for transfer completion */
  uint16_t maxpacket:11;         /* Maximum packet size */
  uint16_t xfrtype:2;            /* See USB_EP_ATTR_XFER_* definitions in usb.h */
  uint16_t speed:2;              /* See USB_*_SPEED definitions in ehci.h */
  int result;                    /* The result of the transfer */
  uint32_t xfrd;                 /* On completion, will hold the number of bytes transferred */
  sem_t iocsem;                  /* Semaphore used to wait for transfer completion */
#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t callback;     /* Transfer complete callback */
  void *arg;                     /* Argument that accompanies the callback */
#endif
};

/* This structure retains the state of one root hub port */

struct imxrt_rhport_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to struct imxrt_rhport_s.
   */

  struct usbhost_driver_s drvr;

  /* Root hub port status */

  volatile bool connected;     /* Connected to device */
  volatile bool lowspeed;      /* Low speed device attached */
  struct imxrt_epinfo_s ep0;   /* EP0 endpoint info */

  /* This is the hub port description understood by class drivers */

  struct usbhost_roothubport_s hport;
};

/* This structure retains the overall state of the USB host controller */

struct imxrt_ehci_s
{
  volatile bool pscwait;        /* TRUE: Thread is waiting for port status change event */

  mutex_t lock;                 /* Support mutually exclusive access */
  sem_t pscsem;                 /* Semaphore to wait for port status change events */

  struct imxrt_epinfo_s ep0;    /* Endpoint 0 */
  struct imxrt_list_s *qhfree;  /* List of free Queue Head (QH) structures */
  struct imxrt_list_s *qtdfree; /* List of free Queue Element Transfer Descriptor (qTD) */
  struct work_s work;           /* Supports interrupt bottom half */

#ifdef CONFIG_USBHOST_HUB
  /* Used to pass external hub port events */

  volatile struct usbhost_hubport_s *hport;
#endif

  /* Root hub ports */

  struct imxrt_rhport_s rhport[IMXRT_EHCI_NRHPORT];
};

#ifdef HAVE_USBHOST_TRACE
/* USB trace codes */

enum usbhost_trace1codes_e
{
  __TRACE1_BASEVALUE = 0,           /* This will force the first value to be 1 */

  EHCI_TRACE1_SYSTEMERROR,          /* EHCI ERROR: System error */
  EHCI_TRACE1_QTDFOREACH_FAILED,    /* EHCI ERROR: imxrt_qtd_foreach failed */
  EHCI_TRACE1_QHALLOC_FAILED,       /* EHCI ERROR: Failed to allocate a QH */
  EHCI_TRACE1_BUFTOOBIG,            /* EHCI ERROR: Buffer too big */
  EHCI_TRACE1_REQQTDALLOC_FAILED,   /* EHCI ERROR: Failed to allocate request qTD */
  EHCI_TRACE1_ADDBPL_FAILED,        /* EHCI ERROR: imxrt_qtd_addbpl failed */
  EHCI_TRACE1_DATAQTDALLOC_FAILED,  /* EHCI ERROR: Failed to allocate data buffer qTD */
  EHCI_TRACE1_DEVDISCONNECTED,      /* EHCI ERROR: Device disconnected */
  EHCI_TRACE1_QHCREATE_FAILED,      /* EHCI ERROR: imxrt_qh_create failed */
  EHCI_TRACE1_QTDSETUP_FAILED,      /* EHCI ERROR: imxrt_qtd_setupphase failed */

  EHCI_TRACE1_QTDDATA_FAILED,       /* EHCI ERROR: imxrt_qtd_dataphase failed */
  EHCI_TRACE1_QTDSTATUS_FAILED,     /* EHCI ERROR: imxrt_qtd_statusphase failed */
  EHCI_TRACE1_TRANSFER_FAILED,      /* EHCI ERROR: Transfer failed */
  EHCI_TRACE1_QHFOREACH_FAILED,     /* EHCI ERROR: imxrt_qh_foreach failed: */
  EHCI_TRACE1_SYSERR_INTR,          /* EHCI: Host System Error Interrupt */
  EHCI_TRACE1_USBERR_INTR,          /* EHCI: USB Error Interrupt (USBERRINT) Interrupt */
  EHCI_TRACE1_EPALLOC_FAILED,       /* EHCI ERROR: Failed to allocate EP info structure */
  EHCI_TRACE1_BADXFRTYPE,           /* EHCI ERROR: Support for transfer type not implemented */
  EHCI_TRACE1_HCHALTED_TIMEOUT,     /* EHCI ERROR: Timed out waiting for HCHalted */
  EHCI_TRACE1_QHPOOLALLOC_FAILED,   /* EHCI ERROR: Failed to allocate the QH pool */

  EHCI_TRACE1_QTDPOOLALLOC_FAILED,  /* EHCI ERROR: Failed to allocate the qTD pool */
  EHCI_TRACE1_PERFLALLOC_FAILED,    /* EHCI ERROR: Failed to allocate the periodic frame list */
  EHCI_TRACE1_RESET_FAILED,         /* EHCI ERROR: imxrt_reset failed */
  EHCI_TRACE1_RUN_FAILED,           /* EHCI ERROR: EHCI Failed to run */
  EHCI_TRACE1_IRQATTACH_FAILED,     /* EHCI ERROR: Failed to attach IRQ */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  EHCI_VTRACE1_PORTSC_CSC,          /* EHCI Connect Status Change */
  EHCI_VTRACE1_PORTSC_CONNALREADY,  /* EHCI Already connected */
  EHCI_VTRACE1_PORTSC_DISCALREADY,  /* EHCI Already disconnected */
  EHCI_VTRACE1_TOPHALF,             /* EHCI Interrupt top half */
  EHCI_VTRACE1_AAINTR,              /* EHCI Async Advance Interrupt */

  EHCI_VTRACE1_CLASSENUM,           /* EHCI Hub port CLASS enumeration */
  EHCI_VTRACE1_USBINTR,             /* EHCI USB Interrupt (USBINT) Interrupt */
  EHCI_VTRACE1_ENUM_DISCONN,        /* EHCI Enumeration not connected */
  EHCI_VTRACE1_INITIALIZING,        /* EHCI Initializing EHCI Stack */
  EHCI_VTRACE1_HCCPARAMS,           /* EHCI HCCPARAMS */
  EHCI_VTRACE1_INIITIALIZED,        /* EHCI USB EHCI Initialized */
#endif

  __TRACE1_NSTRINGS,                /* Separates the format 1 from the format 2 strings */

  EHCI_TRACE2_EPSTALLED,            /* EHCI EP Stalled */
  EHCI_TRACE2_EPIOERROR,            /* EHCI ERROR: EP TOKEN */
  EHCI_TRACE2_CLASSENUM_FAILED,     /* EHCI usbhost_enumerate() failed */

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  EHCI_VTRACE2_ASYNCXFR,            /* EHCI Async transfer */
  EHCI_VTRACE2_INTRXFR,             /* EHCI Interrupt Transfer */
  EHCI_VTRACE2_IOCCHECK,            /* EHCI IOC */
  EHCI_VTRACE2_PORTSC,              /* EHCI PORTSC */
  EHCI_VTRACE2_PORTSC_CONNECTED,    /* EHCI RHPort connected */
  EHCI_VTRACE2_PORTSC_DISCONND,     /* EHCI RHport disconnected */
  EHCI_VTRACE2_MONWAKEUP,           /* EHCI RHPort connected wakeup */

  EHCI_VTRACE2_EPALLOC,             /* EHCI EPALLOC */
  EHCI_VTRACE2_CTRLINOUT,           /* EHCI CTRLIN/OUT */
  EHCI_VTRACE2_HCIVERSION,          /* EHCI HCIVERSION */
  EHCI_VTRACE2_HCSPARAMS,           /* EHCI HCSPARAMS */
#endif

  __TRACE2_NSTRINGS                 /* Total number of enumeration values */
};

/* USB trace data structure */

struct imxrt_ehci_trace_s
{
#if 0
  uint16_t id;
  bool fmt2;
#endif
  const char *string;
};

#endif /* HAVE_USBHOST_TRACE */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

static uint16_t imxrt_read16(const uint8_t *addr);
static uint32_t imxrt_read32(const uint8_t *addr);
#if 0 /* Not used */
static void imxrt_write16(uint16_t memval, uint8_t *addr);
static void imxrt_write32(uint32_t memval, uint8_t *addr);
#endif

#ifdef CONFIG_ENDIAN_BIG
static uint16_t imxrt_swap16(uint16_t value);
static uint32_t imxrt_swap32(uint32_t value);
#else
#  define imxrt_swap16(value) (value)
#  define imxrt_swap32(value) (value)
#endif

#ifdef CONFIG_IMXRT_EHCI_REGDEBUG
static void imxrt_printreg(volatile uint32_t *regaddr, uint32_t regval,
         bool iswrite);
static void imxrt_checkreg(volatile uint32_t *regaddr, uint32_t regval,
         bool iswrite);
static uint32_t imxrt_getreg(volatile uint32_t *regaddr);
static void imxrt_putreg(uint32_t regval, volatile uint32_t *regaddr);
#else
static inline uint32_t imxrt_getreg(volatile uint32_t *regaddr);
static inline void imxrt_putreg(uint32_t regval, volatile uint32_t *regaddr);
#endif
static int ehci_wait_usbsts(uint32_t maskbits, uint32_t donebits,
         unsigned int delay);

/* Allocators ***************************************************************/

static struct imxrt_qh_s *imxrt_qh_alloc(void);
static void imxrt_qh_free(struct imxrt_qh_s *qh);
static struct imxrt_qtd_s *imxrt_qtd_alloc(void);
static void imxrt_qtd_free(struct imxrt_qtd_s *qtd);

/* List Management **********************************************************/

static int imxrt_qh_foreach(struct imxrt_qh_s *qh, uint32_t **bp,
         foreach_qh_t handler, void *arg);
static int imxrt_qtd_foreach(struct imxrt_qh_s *qh, foreach_qtd_t handler,
         void *arg);
static int imxrt_qtd_discard(struct imxrt_qtd_s *qtd, uint32_t **bp,
         void *arg);
static int imxrt_qh_discard(struct imxrt_qh_s *qh);

/* Cache Operations *********************************************************/

#if 0 /* Not used */
static int imxrt_qtd_invalidate(struct imxrt_qtd_s *qtd, uint32_t **bp,
          void *arg);
static int imxrt_qh_invalidate(struct imxrt_qh_s *qh);
#endif
static int imxrt_qtd_flush(struct imxrt_qtd_s *qtd, uint32_t **bp,
                           void *arg);
static int imxrt_qh_flush(struct imxrt_qh_s *qh);

/* Endpoint Transfer Handling ***********************************************/

#ifdef CONFIG_IMXRT_EHCI_REGDEBUG
static void imxrt_qtd_print(struct imxrt_qtd_s *qtd);
static void imxrt_qh_print(struct imxrt_qh_s *qh);
static int imxrt_qtd_dump(struct imxrt_qtd_s *qtd, uint32_t **bp, void *arg);
static int imxrt_qh_dump(struct imxrt_qh_s *qh, uint32_t **bp, void *arg);
#else
#  define imxrt_qtd_print(qtd)
#  define imxrt_qh_print(qh)
#  define imxrt_qtd_dump(qtd, bp, arg) OK
#  define imxrt_qh_dump(qh, bp, arg)   OK
#endif

static inline uint8_t imxrt_ehci_speed(uint8_t usbspeed);
static int imxrt_ioc_setup(struct imxrt_rhport_s *rhport,
         struct imxrt_epinfo_s *epinfo);
static int imxrt_ioc_wait(struct imxrt_epinfo_s *epinfo);
static void imxrt_qh_enqueue(struct imxrt_qh_s *qhead,
         struct imxrt_qh_s *qh);
static struct imxrt_qh_s *imxrt_qh_create(struct imxrt_rhport_s *rhport,
         struct imxrt_epinfo_s *epinfo);
static int imxrt_qtd_addbpl(struct imxrt_qtd_s *qtd, const void *buffer,
         size_t buflen);
static struct imxrt_qtd_s *imxrt_qtd_setupphase(
         struct imxrt_epinfo_s *epinfo, const struct usb_ctrlreq_s *req);
static struct imxrt_qtd_s *imxrt_qtd_dataphase(struct imxrt_epinfo_s *epinfo,
         void *buffer, int buflen, uint32_t tokenbits);
static struct imxrt_qtd_s *imxrt_qtd_statusphase(uint32_t tokenbits);
static ssize_t imxrtimxrt_virtramaddr_async_setup(
         struct imxrt_rhport_s *rhport, struct imxrt_epinfo_s *epinfo,
         const struct usb_ctrlreq_s *req, uint8_t *buffer, size_t buflen);
#ifndef CONFIG_USBHOST_INT_DISABLE
static int imxrt_intr_setup(struct imxrt_rhport_s *rhport,
         struct imxrt_epinfo_s *epinfo, uint8_t *buffer, size_t buflen);
#endif
static ssize_t imxrt_transfer_wait(struct imxrt_epinfo_s *epinfo);
#ifdef CONFIG_USBHOST_ASYNCH
static inline int imxrt_ioc_async_setup(struct imxrt_rhport_s *rhport,
         struct imxrt_epinfo_s *epinfo, usbhost_asynch_t callback,
         void *arg);
static void imxrt_asynch_completion(struct imxrt_epinfo_s *epinfo);
#endif

/* Interrupt Handling *******************************************************/

static int imxrt_qtd_ioccheck(struct imxrt_qtd_s *qtd, uint32_t **bp,
         void *arg);
static int imxrt_qh_ioccheck(struct imxrt_qh_s *qh, uint32_t **bp,
         void *arg);
#ifdef CONFIG_USBHOST_ASYNCH
static int imxrt_qtd_cancel(struct imxrt_qtd_s *qtd, uint32_t **bp,
         void *arg);
static int imxrt_qh_cancel(struct imxrt_qh_s *qh, uint32_t **bp, void *arg);
#endif
static inline void imxrt_ioc_bottomhalf(void);
static inline void imxrt_portsc_bottomhalf(void);
static inline void imxrt_syserr_bottomhalf(void);
static inline void imxrt_async_advance_bottomhalf(void);
static void imxrt_ehci_bottomhalf(void *arg);
static int imxrt_ehci_interrupt(int irq, void *context, void *arg);

/* USB Host Controller Operations *******************************************/

static int imxrt_wait(struct usbhost_connection_s *conn,
                      struct usbhost_hubport_s **hport);
static int imxrt_rh_enumerate(struct usbhost_connection_s *conn,
                              struct usbhost_hubport_s *hport);
static int imxrt_enumerate(struct usbhost_connection_s *conn,
                           struct usbhost_hubport_s *hport);

static int imxrt_ep0configure(struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep0, uint8_t funcaddr,
                              uint8_t speed, uint16_t maxpacketsize);
static int imxrt_epalloc(struct usbhost_driver_s *drvr,
                         const struct usbhost_epdesc_s *epdesc,
                         usbhost_ep_t *ep);
static int imxrt_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int imxrt_alloc(struct usbhost_driver_s *drvr,
                       uint8_t **buffer, size_t *maxlen);
static int imxrt_free(struct usbhost_driver_s *drvr,
                      uint8_t *buffer);
static int imxrt_ioalloc(struct usbhost_driver_s *drvr,
                         uint8_t **buffer, size_t buflen);
static int imxrt_iofree(struct usbhost_driver_s *drvr,
                        uint8_t *buffer);
static int imxrt_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        const struct usb_ctrlreq_s *req,
                        uint8_t *buffer);
static int imxrt_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                         const struct usb_ctrlreq_s *req,
                         const uint8_t *buffer);
static ssize_t imxrt_transfer(struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep, uint8_t *buffer,
                              size_t buflen);
#ifdef CONFIG_USBHOST_ASYNCH
static int imxrt_asynch(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                        uint8_t *buffer, size_t buflen,
                        usbhost_asynch_t callback, void *arg);
#endif
static int imxrt_cancel(struct usbhost_driver_s *drvr, usbhost_ep_t ep);
#ifdef CONFIG_USBHOST_HUB
static int imxrt_connect(struct usbhost_driver_s *drvr,
                         struct usbhost_hubport_s *hport,
                         bool connected);
#endif
static void imxrt_disconnect(struct usbhost_driver_s *drvr,
                             struct usbhost_hubport_s *hport);

/* Initialization ***********************************************************/

static int imxrt_reset(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* In this driver implementation, support is provided for only a single a
 * single USB device.  All status information can be simply retained in a
 * single global instance.
 */

static struct imxrt_ehci_s g_ehci =
{
  .lock = NXMUTEX_INITIALIZER,
  .pscsem = SEM_INITIALIZER(0),
  .ep0.iocsem = SEM_INITIALIZER(1),
};

/* This is the connection/enumeration interface */

static struct usbhost_connection_s g_ehciconn;

/* Maps USB chapter 9 speed to EHCI speed */

static const uint8_t g_ehci_speed[4] =
{
  0, EHCI_LOW_SPEED, EHCI_FULL_SPEED, EHCI_HIGH_SPEED
};

/* The head of the asynchronous queue */

static struct imxrt_qh_s g_asynchead aligned_data(32);

#ifndef CONFIG_USBHOST_INT_DISABLE
/* The head of the periodic queue */

static struct imxrt_qh_s g_intrhead   aligned_data(32);

/* The frame list */

#ifdef CONFIG_IMXRT_EHCI_PREALLOCATE
static uint32_t g_framelist[FRAME_LIST_SIZE] aligned_data(4096);
#else
static uint32_t *g_framelist;
#endif
#endif /* CONFIG_USBHOST_INT_DISABLE */

#ifdef CONFIG_IMXRT_EHCI_PREALLOCATE
/* Pools of pre-allocated data structures.  These will all be linked into the
 * free lists within g_ehci.  These must all be aligned to 32-byte boundaries
 */

/* Queue Head (QH) pool */

static struct imxrt_qh_s g_qhpool[CONFIG_IMXRT_EHCI_NQHS]
                       aligned_data(32);

/* Queue Element Transfer Descriptor (qTD) pool */

static struct imxrt_qtd_s g_qtdpool[CONFIG_IMXRT_EHCI_NQTDS]
                        aligned_data(32);

#else
/* Pools of dynamically data structures.  These will all be linked into the
 * free lists within g_ehci.  These must all be aligned to 32-byte boundaries
 */

/* Queue Head (QH) pool */

static struct imxrt_qh_s *g_qhpool;

/* Queue Element Transfer Descriptor (qTD) pool */

static struct imxrt_qtd_s *g_qtdpool;

#endif

#ifdef HAVE_USBHOST_TRACE
/* USB trace strings */

static const struct imxrt_ehci_trace_s g_trace1[TRACE1_NSTRINGS] =
{
  TRENTRY(EHCI_TRACE1_SYSTEMERROR, TR_FMT1,
          "EHCI ERROR: System error: %06x\n"),
  TRENTRY(EHCI_TRACE1_QTDFOREACH_FAILED, TR_FMT1,
          "EHCI ERROR: imxrt_qtd_foreach failed: %d\n"),
  TRENTRY(EHCI_TRACE1_QHALLOC_FAILED, TR_FMT1,
          "EHCI ERROR: Failed to allocate a QH\n"),
  TRENTRY(EHCI_TRACE1_BUFTOOBIG, TR_FMT1,
          "EHCI ERROR: Buffer too big. Remaining %d\n"),
  TRENTRY(EHCI_TRACE1_REQQTDALLOC_FAILED, TR_FMT1,
          "EHCI ERROR: Failed to allocate request qTD"),
  TRENTRY(EHCI_TRACE1_ADDBPL_FAILED, TR_FMT1,
          "EHCI ERROR: imxrt_qtd_addbpl failed: %d\n"),
  TRENTRY(EHCI_TRACE1_DATAQTDALLOC_FAILED, TR_FMT1,
          "EHCI ERROR: Failed to allocate data buffer qTD, 0"),
  TRENTRY(EHCI_TRACE1_DEVDISCONNECTED, TR_FMT1,
          "EHCI ERROR: Device disconnected %d\n"),
  TRENTRY(EHCI_TRACE1_QHCREATE_FAILED, TR_FMT1,
          "EHCI ERROR: imxrt_qh_create failed\n"),
  TRENTRY(EHCI_TRACE1_QTDSETUP_FAILED, TR_FMT1,
          "EHCI ERROR: imxrt_qtd_setupphase failed\n"),

  TRENTRY(EHCI_TRACE1_QTDDATA_FAILED, TR_FMT1,
          "EHCI ERROR: imxrt_qtd_dataphase failed\n"),
  TRENTRY(EHCI_TRACE1_QTDSTATUS_FAILED, TR_FMT1,
          "EHCI ERROR: imxrt_qtd_statusphase failed\n"),
  TRENTRY(EHCI_TRACE1_TRANSFER_FAILED, TR_FMT1,
          "EHCI ERROR: Transfer failed %d\n"),
  TRENTRY(EHCI_TRACE1_QHFOREACH_FAILED, TR_FMT1,
          "EHCI ERROR: imxrt_qh_foreach failed: %d\n"),
  TRENTRY(EHCI_TRACE1_SYSERR_INTR, TR_FMT1,
          "EHCI: Host System Error Interrupt\n"),
  TRENTRY(EHCI_TRACE1_USBERR_INTR, TR_FMT1,
          "EHCI: USB Error Interrupt (USBERRINT) Interrupt: %06x\n"),
  TRENTRY(EHCI_TRACE1_EPALLOC_FAILED, TR_FMT1,
          "EHCI ERROR: Failed to allocate EP info structure\n"),
  TRENTRY(EHCI_TRACE1_BADXFRTYPE, TR_FMT1,
          "EHCI ERROR: Support for transfer type %d not implemented\n"),
  TRENTRY(EHCI_TRACE1_HCHALTED_TIMEOUT, TR_FMT1,
          "EHCI ERROR: Timed out waiting for HCHalted. USBSTS: %06x\n"),
  TRENTRY(EHCI_TRACE1_QHPOOLALLOC_FAILED, TR_FMT1,
          "EHCI ERROR: Failed to allocate the QH pool\n"),

  TRENTRY(EHCI_TRACE1_QTDPOOLALLOC_FAILED, TR_FMT1,
          "EHCI ERROR: Failed to allocate the qTD pool\n"),
  TRENTRY(EHCI_TRACE1_PERFLALLOC_FAILED, TR_FMT1,
          "EHCI ERROR: Failed to allocate the periodic frame list\n"),
  TRENTRY(EHCI_TRACE1_RESET_FAILED, TR_FMT1,
          "EHCI ERROR: imxrt_reset failed: %d\n"),
  TRENTRY(EHCI_TRACE1_RUN_FAILED, TR_FMT1,
          "EHCI ERROR: EHCI Failed to run: USBSTS=%06x\n"),
  TRENTRY(EHCI_TRACE1_IRQATTACH_FAILED, TR_FMT1,
          "EHCI ERROR: Failed to attach IRQ%d\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(EHCI_VTRACE1_PORTSC_CSC, TR_FMT1,
          "EHCI Connect Status Change: %06x\n"),
  TRENTRY(EHCI_VTRACE1_PORTSC_CONNALREADY, TR_FMT1,
          "EHCI Already connected: %06x\n"),
  TRENTRY(EHCI_VTRACE1_PORTSC_DISCALREADY, TR_FMT1,
          "EHCI Already disconnected: %06x\n"),
  TRENTRY(EHCI_VTRACE1_TOPHALF, TR_FMT1,
          "EHCI Interrupt: %06x\n"),
  TRENTRY(EHCI_VTRACE1_AAINTR, TR_FMT1,
          "EHCI Async Advance Interrupt\n"),

  TRENTRY(EHCI_VTRACE1_CLASSENUM, TR_FMT1,
          "EHCI Hub port %d: Enumerate the device\n"),
  TRENTRY(EHCI_VTRACE1_USBINTR, TR_FMT1,
          "EHCI USB Interrupt (USBINT) Interrupt: %06x\n"),
  TRENTRY(EHCI_VTRACE1_ENUM_DISCONN, TR_FMT1,
          "EHCI Enumeration not connected\n"),
  TRENTRY(EHCI_VTRACE1_INITIALIZING, TR_FMT1,
          "EHCI Initializing EHCI Stack\n"),
  TRENTRY(EHCI_VTRACE1_HCCPARAMS, TR_FMT1,
          "EHCI HCCPARAMS=%06x\n"),
  TRENTRY(EHCI_VTRACE1_INIITIALIZED, TR_FMT1,
          "EHCI USB EHCI Initialized\n"),
#endif
};

static const struct imxrt_ehci_trace_s g_trace2[TRACE2_NSTRINGS] =
{
  TRENTRY(EHCI_TRACE2_EPSTALLED, TR_FMT2,
          "EHCI EP%d Stalled: TOKEN=%04x\n"),
  TRENTRY(EHCI_TRACE2_EPIOERROR, TR_FMT2,
          "EHCI ERROR: EP%d TOKEN=%04x\n"),
  TRENTRY(EHCI_TRACE2_CLASSENUM_FAILED, TR_FMT2,
          "EHCI Hub port %d usbhost_enumerate() failed: %d\n"),

#ifdef HAVE_USBHOST_TRACE_VERBOSE
  TRENTRY(EHCI_VTRACE2_ASYNCXFR, TR_FMT2,
          "EHCI Async transfer EP%d buflen=%d\n"),
  TRENTRY(EHCI_VTRACE2_INTRXFR, TR_FMT2,
          "EHCI Intr Transfer EP%d buflen=%d\n"),
  TRENTRY(EHCI_VTRACE2_IOCCHECK, TR_FMT2,
          "EHCI IOC EP%d TOKEN=%04x\n"),
  TRENTRY(EHCI_VTRACE2_PORTSC, TR_FMT2,
          "EHCI PORTSC%d: %04x\n"),
  TRENTRY(EHCI_VTRACE2_PORTSC_CONNECTED, TR_FMT2,
          "EHCI RHPort%d connected, pscwait: %d\n"),
  TRENTRY(EHCI_VTRACE2_PORTSC_DISCONND, TR_FMT2,
          "EHCI RHport%d disconnected, pscwait: %d\n"),
  TRENTRY(EHCI_VTRACE2_MONWAKEUP, TR_FMT2,
          "EHCI RHPort%d connected: %d\n"),

  TRENTRY(EHCI_VTRACE2_EPALLOC, TR_FMT2,
          "EHCI EPALLOC: EP%d TYPE=%d\n"),
  TRENTRY(EHCI_VTRACE2_CTRLINOUT, TR_FMT2,
          "EHCI CTRLIN/OUT: RHPort%d req: %02x\n"),
  TRENTRY(EHCI_VTRACE2_HCIVERSION, TR_FMT2,
          "EHCI HCIVERSION %x.%02x\n"),
  TRENTRY(EHCI_VTRACE2_HCSPARAMS, TR_FMT2,
          "EHCI nports=%d, HCSPARAMS=%04x\n"),
#endif
};
#endif /* HAVE_USBHOST_TRACE */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_read16
 *
 * Description:
 *   Read 16-bit little endian data
 *
 ****************************************************************************/

static uint16_t imxrt_read16(const uint8_t *addr)
{
#ifdef CONFIG_ENDIAN_BIG
  return (uint16_t)addr[0] << 8 | (uint16_t)addr[1];
#else
  return (uint16_t)addr[1] << 8 | (uint16_t)addr[0];
#endif
}

/****************************************************************************
 * Name: imxrt_read32
 *
 * Description:
 *   Read 32-bit little endian data
 *
 ****************************************************************************/

static inline uint32_t imxrt_read32(const uint8_t *addr)
{
#ifdef CONFIG_ENDIAN_BIG
  return (uint32_t)imxrt_read16(&addr[0]) << 16 |
         (uint32_t)imxrt_read16(&addr[2]);
#else
  return (uint32_t)imxrt_read16(&addr[2]) << 16 |
         (uint32_t)imxrt_read16(&addr[0]);
#endif
}

/****************************************************************************
 * Name: imxrt_write16
 *
 * Description:
 *   Write 16-bit little endian data
 *
 ****************************************************************************/

#if 0 /* Not used */
static void imxrt_write16(uint16_t memval, uint8_t *addr)
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

/****************************************************************************
 * Name: imxrt_write32
 *
 * Description:
 *   Write 32-bit little endian data
 *
 ****************************************************************************/

#if 0 /* Not used */
static void imxrt_write32(uint32_t memval, uint8_t *addr)
{
#ifdef CONFIG_ENDIAN_BIG
  imxrt_write16(memval >> 16, &addr[0]);
  imxrt_write16(memval & 0xffff, &addr[2]);
#else
  imxrt_write16(memval & 0xffff, &addr[0]);
  imxrt_write16(memval >> 16, &addr[2]);
#endif
}
#endif

/****************************************************************************
 * Name: imxrt_swap16
 *
 * Description:
 *   Swap bytes on a 16-bit value
 *
 ****************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
static uint16_t imxrt_swap16(uint16_t value)
{
  return ((value >> 8) & 0xff) | ((value & 0xff) << 8);
}
#endif

/****************************************************************************
 * Name: imxrt_swap32
 *
 * Description:
 *   Swap bytes on a 32-bit value
 *
 ****************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
static uint32_t imxrt_swap32(uint32_t value)
{
  return (uint32_t)imxrt_swap16((uint16_t)((value >> 16) & 0xffff)) |
         (uint32_t)imxrt_swap16((uint16_t)(value & 0xffff)) << 16;
}
#endif

/****************************************************************************
 * Name: imxrt_printreg
 *
 * Description:
 *   Print the contents of a IMXRT EHCI register
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_EHCI_REGDEBUG
static void imxrt_printreg(volatile uint32_t *regaddr, uint32_t regval,
                           bool iswrite)
{
  uinfo("%08x%s%08x\n", (uintptr_t)regaddr, iswrite ? "<-" : "->", regval);
}
#endif

/****************************************************************************
 * Name: imxrt_checkreg
 *
 * Description:
 *   Check if it is time to output debug information for accesses to a IMXRT
 *   EHCI register
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_EHCI_REGDEBUG
static void imxrt_checkreg(volatile uint32_t *regaddr, uint32_t regval,
                           bool iswrite)
{
  static uint32_t *prevaddr = NULL;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register last
   * time?  Are we polling the register?  If so, suppress the output.
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

              imxrt_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              uinfo("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = (uint32_t *)regaddr;
      preval    = regval;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new register access */

      imxrt_printreg(regaddr, regval, iswrite);
    }
}
#endif

/****************************************************************************
 * Name: imxrt_getreg
 *
 * Description:
 *   Get the contents of an IMXRT register
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_EHCI_REGDEBUG
static uint32_t imxrt_getreg(volatile uint32_t *regaddr)
{
  /* Read the value from the register */

  uint32_t regval = *regaddr;

  /* Check if we need to print this value */

  imxrt_checkreg(regaddr, regval, false);
  return regval;
}
#else
static inline uint32_t imxrt_getreg(volatile uint32_t *regaddr)
{
  return *regaddr;
}
#endif

/****************************************************************************
 * Name: imxrt_putreg
 *
 * Description:
 *   Set the contents of an IMXRT register to a value
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_EHCI_REGDEBUG
static void imxrt_putreg(uint32_t regval, volatile uint32_t *regaddr)
{
  /* Check if we need to print this value */

  imxrt_checkreg(regaddr, regval, true);

  /* Write the value */

  *regaddr = regval;
}
#else
static inline void imxrt_putreg(uint32_t regval, volatile uint32_t *regaddr)
{
  *regaddr = regval;
}
#endif

/****************************************************************************
 * Name: ehci_wait_usbsts
 *
 * Description:
 *   Wait for either (1) a field in the USBSTS register to take a specific
 *   value, (2) for a timeout to occur, or (3) a error to occur.  Return
 *   a value to indicate which terminated the wait.
 *
 ****************************************************************************/

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

      regval = imxrt_getreg(&HCOR->usbsts);
      if ((regval & EHCI_INT_SYSERROR) != 0)
        {
          usbhost_trace1(EHCI_TRACE1_SYSTEMERROR, regval);
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

/****************************************************************************
 * Name: imxrt_qh_alloc
 *
 * Description:
 *   Allocate a Queue Head (QH) structure by removing it from the free list
 *
 * Assumption:  Caller holds the lock
 *
 ****************************************************************************/

static struct imxrt_qh_s *imxrt_qh_alloc(void)
{
  struct imxrt_qh_s *qh;

  /* Remove the QH structure from the freelist */

  qh = (struct imxrt_qh_s *)g_ehci.qhfree;
  if (qh)
    {
      g_ehci.qhfree = ((struct imxrt_list_s *)qh)->flink;
      memset(qh, 0, sizeof(struct imxrt_qh_s));
    }

  return qh;
}

/****************************************************************************
 * Name: imxrt_qh_free
 *
 * Description:
 *   Free a Queue Head (QH) structure by returning it to the free list
 *
 * Assumption:  Caller holds the lock
 *
 ****************************************************************************/

static void imxrt_qh_free(struct imxrt_qh_s *qh)
{
  struct imxrt_list_s *entry = (struct imxrt_list_s *)qh;

  /* Put the QH structure back into the free list */

  entry->flink  = g_ehci.qhfree;
  g_ehci.qhfree = entry;
}

/****************************************************************************
 * Name: imxrt_qtd_alloc
 *
 * Description:
 *   Allocate a Queue Element Transfer Descriptor (qTD) by removing it from
 *   the free list
 *
 * Assumption:  Caller holds the lock
 *
 ****************************************************************************/

static struct imxrt_qtd_s *imxrt_qtd_alloc(void)
{
  struct imxrt_qtd_s *qtd;

  /* Remove the qTD from the freelist */

  qtd = (struct imxrt_qtd_s *)g_ehci.qtdfree;
  if (qtd)
    {
      g_ehci.qtdfree = ((struct imxrt_list_s *)qtd)->flink;
      memset(qtd, 0, sizeof(struct imxrt_qtd_s));
    }

  return qtd;
}

/****************************************************************************
 * Name: imxrt_qtd_free
 *
 * Description:
 *   Free a Queue Element Transfer Descriptor (qTD) by returning it to the
 *   free list
 *
 * Assumption:
 *   Caller holds the lock
 *
 ****************************************************************************/

static void imxrt_qtd_free(struct imxrt_qtd_s *qtd)
{
  struct imxrt_list_s *entry = (struct imxrt_list_s *)qtd;

  /* Put the qTD back into the free list */

  entry->flink   = g_ehci.qtdfree;
  g_ehci.qtdfree = entry;
}

/****************************************************************************
 * Name: imxrt_qh_foreach
 *
 * Description:
 *   Give the first entry in a list of Queue Head (QH) structures, call the
 *   handler for each QH structure in the list (including the one at the head
 *   of the list).
 *
 ****************************************************************************/

static int imxrt_qh_foreach(struct imxrt_qh_s *qh, uint32_t **bp,
                            foreach_qh_t handler, void *arg)
{
  struct imxrt_qh_s *next;
  uintptr_t physaddr;
  int ret;

  DEBUGASSERT(qh && handler);
  while (qh)
    {
      /* Is this the end of the list?  Check the horizontal link pointer
       * (HLP) terminate (T) bit.  If T==1, then the HLP address is not
       * valid.
       */

      physaddr = imxrt_swap32(qh->hw.hlp);
      if ((physaddr & QH_HLP_T) != 0)
        {
          /* Set the next pointer to NULL.  This will terminate the loop. */

          next = NULL;
        }

      /* Is the next QH the asynchronous list head which will always be at
       * the end of the asynchronous queue?
       */

      else if (imxrt_virtramaddr(physaddr & QH_HLP_MASK) ==
               (uintptr_t)&g_asynchead)
        {
          /* That will also terminate the loop */

          next = NULL;
        }

      /* Otherwise, there is a QH structure after this one that describes
       * another transaction.
       */

      else
        {
          physaddr = imxrt_swap32(qh->hw.hlp) & QH_HLP_MASK;
          next     = (struct imxrt_qh_s *)imxrt_virtramaddr(physaddr);
        }

      /* Perform the user action on this entry.  The action might result in
       * unlinking the entry!  But that is okay because we already have the
       * next QH pointer.
       *
       * Notice that we do not manage the back pointer (bp).  If the call-
       * out uses it, it must update it as necessary.
       */

      ret = handler(qh, bp, arg);

      /* If the handler returns any non-zero value, then terminate the
       * traversal early.
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

/****************************************************************************
 * Name: imxrt_qtd_foreach
 *
 * Description:
 *   Give a Queue Head (QH) instance, call the handler for each qTD structure
 *   in the queue.
 *
 ****************************************************************************/

static int imxrt_qtd_foreach(struct imxrt_qh_s *qh, foreach_qtd_t handler,
                             void *arg)
{
  struct imxrt_qtd_s *qtd;
  struct imxrt_qtd_s *next;
  uintptr_t physaddr;
  uint32_t *bp;
  int ret;

  DEBUGASSERT(qh && handler);

  /* Handle the special case where the queue is empty */

  bp       = &qh->fqp;          /* Start of qTDs in original list */
  physaddr = imxrt_swap32(*bp); /* Physical address of first qTD in CPU order */

  if ((physaddr & QTD_NQP_T) != 0)
    {
      return 0;
    }

  /* Start with the first qTD in the list */

  qtd  = (struct imxrt_qtd_s *)imxrt_virtramaddr(physaddr);
  next = NULL;

  /* And loop until we encounter the end of the qTD list */

  while (qtd)
    {
      /* Is this the end of the list?  Check the next qTD pointer (NQP)
       * terminate (T) bit.  If T==1, then the NQP address is not valid.
       */

      if ((imxrt_swap32(qtd->hw.nqp) & QTD_NQP_T) != 0)
        {
          /* Set the next pointer to NULL.  This will terminate the loop. */

          next = NULL;
        }
      else
        {
          physaddr = imxrt_swap32(qtd->hw.nqp) & QTD_NQP_NTEP_MASK;
          next     = (struct imxrt_qtd_s *)imxrt_virtramaddr(physaddr);
        }

      /* Perform the user action on this entry.  The action might result in
       * unlinking the entry!  But that is okay because we already have the
       * next qTD pointer.
       *
       * Notice that we do not manage the back pointer (bp).  If the call-out
       * uses it, it must update it as necessary.
       */

      ret = handler(qtd, &bp, arg);

      /* If the handler returns any non-zero value, then terminate the
       * traversal early.
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

/****************************************************************************
 * Name: imxrt_qtd_discard
 *
 * Description:
 *   This is a imxrt_qtd_foreach callback.  It simply unlinks the QTD,
 *   updates the back pointer, and frees the QTD structure.
 *
 ****************************************************************************/

static int imxrt_qtd_discard(struct imxrt_qtd_s *qtd, uint32_t **bp,
                             void *arg)
{
  DEBUGASSERT(qtd && bp && *bp);

  /* Remove the qTD from the list by updating the forward pointer to skip
   * around this qTD.  We do not change that pointer because are repeatedly
   * removing the aTD at the head of the QH list.
   */

  **bp = qtd->hw.nqp;

  /* Then free the qTD */

  imxrt_qtd_free(qtd);
  return OK;
}

/****************************************************************************
 * Name: imxrt_qh_discard
 *
 * Description:
 *   Free the Queue Head (QH) and all qTD's attached to the QH.
 *
 * Assumptions:
 *   The QH structure itself has already been unlinked from whatever list it
 *   may have been in.
 *
 ****************************************************************************/

static int imxrt_qh_discard(struct imxrt_qh_s *qh)
{
  int ret;

  DEBUGASSERT(qh);

  /* Free all of the qTD's attached to the QH */

  ret = imxrt_qtd_foreach(qh, imxrt_qtd_discard, NULL);
  if (ret < 0)
    {
      usbhost_trace1(EHCI_TRACE1_QTDFOREACH_FAILED, -ret);
    }

  /* Then free the QH itself */

  imxrt_qh_free(qh);
  return ret;
}

/****************************************************************************
 * Name: imxrt_qtd_invalidate
 *
 * Description:
 *   This is a callback from imxrt_qtd_foreach.  It simply invalidates D-
 *   cache for address range of the qTD entry.
 *
 ****************************************************************************/

#if 0 /* Not used */
static int imxrt_qtd_invalidate(struct imxrt_qtd_s *qtd, uint32_t **bp,
                                void *arg)
{
  /* Invalidate the D-Cache, i.e., force reloading of the D-Cache from memory
   * memory over the specified address range.
   */

  up_invalidate_dcache((uintptr_t)&qtd->hw,
                       (uintptr_t)&qtd->hw + sizeof(struct ehci_qtd_s));
  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_qh_invalidate
 *
 * Description:
 *   Invalidate the Queue Head and all qTD entries in the queue.
 *
 ****************************************************************************/

#if 0 /* Not used */
static int imxrt_qh_invalidate(struct imxrt_qh_s *qh)
{
  /* Invalidate the QH first so that we reload the qTD list head */

  up_invalidate_dcache((uintptr_t)&qh->hw,
                       (uintptr_t)&qh->hw + sizeof(struct ehci_qh_s));

  /* Then invalidate all of the qTD entries in the queue */

  return imxrt_qtd_foreach(qh, imxrt_qtd_invalidate, NULL);
}
#endif

/****************************************************************************
 * Name: imxrt_qtd_flush
 *
 * Description:
 *   This is a callback from imxrt_qtd_foreach.  It simply flushes D-cache
 *   for address range of the qTD entry.
 *
 ****************************************************************************/

static int imxrt_qtd_flush(struct imxrt_qtd_s *qtd, uint32_t **bp, void *arg)
{
  /* Flush the D-Cache, i.e., make the contents of the memory match the
   * contents of the D-Cache in the specified address range and invalidate
   * the D-Cache to force re-loading of the data from memory when next
   * accessed.
   */

  up_flush_dcache((uintptr_t)&qtd->hw,
                  (uintptr_t)&qtd->hw + sizeof(struct ehci_qtd_s));
  return OK;
}

/****************************************************************************
 * Name: imxrt_qh_flush
 *
 * Description:
 *   Invalidate the Queue Head and all qTD entries in the queue.
 *
 ****************************************************************************/

static int imxrt_qh_flush(struct imxrt_qh_s *qh)
{
  /* Flush the QH first.  This will write the contents of the D-cache to RAM
   * and invalidate the contents of the D-cache so that the next access will
   * be reloaded from D-Cache.
   */

  up_flush_dcache((uintptr_t)&qh->hw,
                  (uintptr_t)&qh->hw + sizeof(struct ehci_qh_s));

  /* Then flush all of the qTD entries in the queue */

  return imxrt_qtd_foreach(qh, imxrt_qtd_flush, NULL);
}

/****************************************************************************
 * Name: imxrt_qtd_print
 *
 * Description:
 *   Print the context of one qTD
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_EHCI_REGDEBUG
static void imxrt_qtd_print(struct imxrt_qtd_s *qtd)
{
  uinfo("  QTD[%p]:\n", qtd);
  uinfo("    hw:\n");
  uinfo("      nqp: %08x alt: %08x token: %08x\n",
        qtd->hw.nqp, qtd->hw.alt, qtd->hw.token);
  uinfo("      bpl: %08x %08x %08x %08x %08x\n",
        qtd->hw.bpl[0], qtd->hw.bpl[1], qtd->hw.bpl[2],
        qtd->hw.bpl[3], qtd->hw.bpl[4]);
}
#endif

/****************************************************************************
 * Name: imxrt_qh_print
 *
 * Description:
 *   Print the context of one QH
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_EHCI_REGDEBUG
static void imxrt_qh_print(struct imxrt_qh_s *qh)
{
  struct imxrt_epinfo_s *epinfo;
  struct ehci_overlay_s *overlay;

  uinfo("QH[%p]:\n", qh);
  uinfo("  hw:\n");
  uinfo("    hlp: %08x epchar: %08x epcaps: %08x cqp: %08x\n",
        qh->hw.hlp, qh->hw.epchar, qh->hw.epcaps, qh->hw.cqp);

  overlay = &qh->hw.overlay;
  uinfo("  overlay:\n");
  uinfo("    nqp: %08x alt: %08x token: %08x\n",
        overlay->nqp, overlay->alt, overlay->token);
  uinfo("    bpl: %08x %08x %08x %08x %08x\n",
        overlay->bpl[0], overlay->bpl[1], overlay->bpl[2],
        overlay->bpl[3], overlay->bpl[4]);

  uinfo("  fqp:\n", qh->fqp);

  epinfo = qh->epinfo;
  uinfo("  epinfo[%p]:\n", epinfo);
  if (epinfo)
    {
      uinfo("    EP%d DIR=%s FA=%08x TYPE=%d MaxPacket=%d\n",
            epinfo->epno, epinfo->dirin ? "IN" : "OUT", epinfo->devaddr,
            epinfo->xfrtype, epinfo->maxpacket);
      uinfo("    Toggle=%d iocwait=%d speed=%d result=%d\n",
            epinfo->toggle, epinfo->iocwait, epinfo->speed, epinfo->result);
    }
}
#endif

/****************************************************************************
 * Name: imxrt_qtd_dump
 *
 * Description:
 *   This is a imxrt_qtd_foreach callout function.  It dumps the context of
 *   one qTD
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_EHCI_REGDEBUG
static int imxrt_qtd_dump(struct imxrt_qtd_s *qtd, uint32_t **bp, void *arg)
{
  imxrt_qtd_print(qtd);
  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_qh_dump
 *
 * Description:
 *   This is a imxrt_qh_foreach call-out function.  It dumps a QH structure
 *   and all of the qTD structures linked to the QH.
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_EHCI_REGDEBUG
static int imxrt_qh_dump(struct imxrt_qh_s *qh, uint32_t **bp, void *arg)
{
  imxrt_qh_print(qh);
  return imxrt_qtd_foreach(qh, imxrt_qtd_dump, NULL);
}
#endif

/****************************************************************************
 * Name: imxrt_ehci_speed
 *
 * Description:
 *  Map a speed enumeration value per Chapter 9 of the USB specification to
 *  the speed enumeration required in the EHCI queue head.
 *
 ****************************************************************************/

static inline uint8_t imxrt_ehci_speed(uint8_t usbspeed)
{
  DEBUGASSERT(usbspeed >= USB_SPEED_LOW && usbspeed <= USB_SPEED_HIGH);
  return g_ehci_speed[usbspeed];
}

/****************************************************************************
 * Name: imxrt_ioc_setup
 *
 * Description:
 *   Set the request for the IOC event well BEFORE enabling the transfer (as
 *   soon as we are absolutely committed to the to avoid transfer).  We do
 *   this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 * Assumption:  The caller holds the EHCI lock
 *
 ****************************************************************************/

static int imxrt_ioc_setup(struct imxrt_rhport_s *rhport,
                           struct imxrt_epinfo_s *epinfo)
{
  irqstate_t flags;
  int ret = -ENODEV;

  DEBUGASSERT(rhport && epinfo && !epinfo->iocwait);
#ifdef CONFIG_USBHOST_ASYNCH
  DEBUGASSERT(epinfo->callback == NULL);
#endif

  /* Is the device still connected? */

  flags = enter_critical_section();
  if (rhport->connected)
    {
      /* Then set iocwait to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer
       * completed.
       */

      epinfo->iocwait  = true;   /* We want to be awakened by IOC interrupt */
      epinfo->status   = 0;      /* No status yet */
      epinfo->xfrd     = 0;      /* Nothing transferred yet */
      epinfo->result   = -EBUSY; /* Transfer in progress */
#ifdef CONFIG_USBHOST_ASYNCH
      epinfo->callback = NULL;   /* No asynchronous callback */
      epinfo->arg      = NULL;
#endif
      ret              = OK;     /* We are good to go */
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: imxrt_ioc_wait
 *
 * Description:
 *   Wait for the IOC event.
 *
 * Assumption:  The caller does *NOT* hold the EHCI lock.  That would
 * cause a deadlock when the bottom-half, worker thread needs to take the
 * semaphore.
 *
 ****************************************************************************/

static int imxrt_ioc_wait(struct imxrt_epinfo_s *epinfo)
{
  int ret = OK;

  /* Wait for the IOC event.  Loop to handle any false alarm semaphore
   * counts.  Return an error if the task is canceled.
   */

  while (epinfo->iocwait)
    {
      ret = nxsem_wait_uninterruptible(&epinfo->iocsem);
      if (ret < 0)
        {
          break;
        }
    }

  return ret < 0 ? ret : epinfo->result;
}

/****************************************************************************
 * Name: imxrt_qh_enqueue
 *
 * Description:
 *   Add a new, ready-to-go QH w/attached qTDs to the asynchronous queue.
 *
 * Assumptions:  The caller holds the EHCI lock
 *
 ****************************************************************************/

static void imxrt_qh_enqueue(struct imxrt_qh_s *qhead, struct imxrt_qh_s *qh)
{
  uintptr_t physaddr;

  /* Set the internal fqp field.  When we transverse the QH list later,
   * we need to know the correct place to start because the overlay may no
   * longer point to the first qTD entry.
   */

  qh->fqp = qh->hw.overlay.nqp;
  imxrt_qh_dump(qh, NULL, NULL);

  /* Add the new QH to the head of the asynchronous queue list.
   *
   * First, attach the old head as the new QH HLP and flush the new QH and
   * its attached qTDs to RAM.
   */

  qh->hw.hlp = qhead->hw.hlp;
  imxrt_qh_flush(qh);

  /* Then set the new QH as the first QH in the asynchronous queue and flush
   * the modified head to RAM.
   */

  physaddr = (uintptr_t)imxrt_physramaddr((uintptr_t)qh);
  qhead->hw.hlp = imxrt_swap32(physaddr | QH_HLP_TYP_QH);

  up_flush_dcache((uintptr_t)&qhead->hw,
                  (uintptr_t)&qhead->hw + sizeof(struct ehci_qh_s));
}

/****************************************************************************
 * Name: imxrt_qh_create
 *
 * Description:
 *   Create a new Queue Head (QH)
 *
 ****************************************************************************/

static struct imxrt_qh_s *imxrt_qh_create(struct imxrt_rhport_s *rhport,
                                          struct imxrt_epinfo_s *epinfo)
{
  struct imxrt_qh_s *qh;
  uint32_t rhpndx;
  uint32_t regval;
  uint8_t hubaddr;
  uint8_t hubport;

  /* Allocate a new queue head structure */

  qh = imxrt_qh_alloc();
  if (qh == NULL)
    {
      usbhost_trace1(EHCI_TRACE1_QHALLOC_FAILED, 0);
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

  regval = ((uint32_t)epinfo->devaddr << QH_EPCHAR_DEVADDR_SHIFT) |
           ((uint32_t)epinfo->epno << QH_EPCHAR_ENDPT_SHIFT) |
           ((uint32_t)imxrt_ehci_speed(epinfo->speed) <<
            QH_EPCHAR_EPS_SHIFT) |
           QH_EPCHAR_DTC |
           ((uint32_t)epinfo->maxpacket << QH_EPCHAR_MAXPKT_SHIFT) |
           ((uint32_t)8 << QH_EPCHAR_RL_SHIFT);

  /* Paragraph 3.6.3: "Control Endpoint Flag (C). If the QH.EPS field
   * indicates the endpoint is not a high-speed device, and the endpoint
   * is an control endpoint, then software must set this bit to a one.
   * Otherwise it should always set this bit to a zero."
   */

  if (epinfo->speed   != USB_SPEED_HIGH &&
      epinfo->xfrtype == USB_EP_ATTR_XFER_CONTROL)
    {
      regval |= QH_EPCHAR_C;
    }

  /* Save the endpoint characteristics word with the correct byte order */

  qh->hw.epchar = imxrt_swap32(regval);

  /* Write QH endpoint capabilities
   *
   * FIELD    DESCRIPTION                     VALUE/SOURCE
   * -------- ------------------------------- --------------------
   * SSMASK   Interrupt Schedule Mask         Depends on epinfo->xfrtype
   * SCMASK   Split Completion Mask           0
   * HUBADDR  Hub Address                     Always 0 for now
   * PORT     Port number                     RH port index + 1
   * MULT     High band width multiplier      1
   */

  rhpndx  = RHPNDX(rhport);

#ifdef CONFIG_USBHOST_HUB
  /* REVISIT:  Future HUB support will require the HUB port number
   * and HUB device address to be included here:
   *
   * - The HUB device address is the USB device address of the USB 2.0 Hub
   *   below which a full- or low-speed device is attached.
   * - The HUB port number is the port number on the above USB 2.0 Hub
   *
   * These fields are used in the split-transaction protocol.  The kludge
   * below should work for hubs connected directly to a root hub port,
   * but would not work for devices connected to downstream hubs.
   */

#warning Missing logic
  hubaddr = rhport->ep0.devaddr;
  hubport = rhpndx + 1;
#else
  hubaddr = rhport->ep0.devaddr;
  hubport = rhpndx + 1;
#endif

  regval  = ((uint32_t)hubaddr << QH_EPCAPS_HUBADDR_SHIFT) |
            ((uint32_t)hubport << QH_EPCAPS_PORT_SHIFT) |
            ((uint32_t)1       << QH_EPCAPS_MULT_SHIFT);

#ifndef CONFIG_USBHOST_INT_DISABLE
  if (epinfo->xfrtype == USB_EP_ATTR_XFER_INT)
    {
      /* Here, the S-Mask field in the queue head is set to 1, indicating
       * that the transaction for the endpoint should be executed on the bus
       * during micro-frame 0 of the frame.
       *
       * REVISIT: The polling interval should be controlled by the which
       * entry is the framelist holds the QH pointer for a given micro-frame
       * and the QH pointer should be replicated for different polling rates.
       * This implementation currently just sets all frame_list entry to
       * all the same interrupt queue.  That should work but will not give
       * any control over polling rates.
       */
#warning REVISIT

      regval |= ((uint32_t)1               << QH_EPCAPS_SSMASK_SHIFT);
    }
#endif

  qh->hw.epcaps = imxrt_swap32(regval);

  /* Mark this as the end of this list.  This will be overwritten if/when the
   * next qTD is added to the queue.
   */

  qh->hw.hlp         = imxrt_swap32(QH_HLP_T);
  qh->hw.overlay.nqp = imxrt_swap32(QH_NQP_T);
  qh->hw.overlay.alt = imxrt_swap32(QH_AQP_T);
  return qh;
}

/****************************************************************************
 * Name: imxrt_qtd_addbpl
 *
 * Description:
 *   Add a buffer pointer list to a qTD.
 *
 ****************************************************************************/

static int imxrt_qtd_addbpl(struct imxrt_qtd_s *qtd, const void *buffer,
                            size_t buflen)
{
  uint32_t physaddr;
  uint32_t nbytes;
  uint32_t next;
  int ndx;

  /* Flush the contents of the data buffer to RAM so that the correct
   * contents will be accessed for an OUT DMA.
   */

  up_flush_dcache((uintptr_t)buffer, (uintptr_t)buffer + buflen);

  /* Loop, adding the aligned physical addresses of the buffer to the buffer
   * page list.  Only the first entry need not be aligned (because only the
   * first entry has the offset field). The subsequent entries must begin on
   * 4KB address boundaries.
   */

  physaddr = (uint32_t)imxrt_physramaddr((uintptr_t)buffer);

  for (ndx = 0; ndx < 5; ndx++)
    {
      /* Write the physical address of the buffer into the qTD buffer pointer
       * list.
       */

      qtd->hw.bpl[ndx] = imxrt_swap32(physaddr);

      /* Get the next buffer pointer (in the case where we will have to
       * transfer more then one chunk).  This buffer must be aligned to a
       * 4KB address boundary.
       */

      next = (physaddr + 4096) & ~4095;

      /* How many bytes were included in the last buffer?  Was it the whole
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
      usbhost_trace1(EHCI_TRACE1_BUFTOOBIG, buflen);
      return -EFBIG;
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_qtd_setupphase
 *
 * Description:
 *   Create a SETUP phase request qTD.
 *
 ****************************************************************************/

static struct imxrt_qtd_s *
  imxrt_qtd_setupphase(struct imxrt_epinfo_s *epinfo,
                       const struct usb_ctrlreq_s *req)
{
  struct imxrt_qtd_s *qtd;
  uint32_t regval;
  int ret;

  /* Allocate a new Queue Element Transfer Descriptor (qTD) */

  qtd = imxrt_qtd_alloc();
  if (qtd == NULL)
    {
      usbhost_trace1(EHCI_TRACE1_REQQTDALLOC_FAILED, 0);
      return NULL;
    }

  /* Mark this as the end of the list (this will be overwritten if another
   * qTD is added after this one).
   */

  qtd->hw.nqp = imxrt_swap32(QTD_NQP_T);
  qtd->hw.alt = imxrt_swap32(QTD_AQP_T);

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

  qtd->hw.token = imxrt_swap32(regval);

  /* Add the buffer data */

  ret = imxrt_qtd_addbpl(qtd, req, USB_SIZEOF_CTRLREQ);
  if (ret < 0)
    {
      usbhost_trace1(EHCI_TRACE1_ADDBPL_FAILED, -ret);
      imxrt_qtd_free(qtd);
      return NULL;
    }

  /* Add the data transfer size to the count in the epinfo structure */

  epinfo->xfrd += USB_SIZEOF_CTRLREQ;

  return qtd;
}

/****************************************************************************
 * Name: imxrt_qtd_dataphase
 *
 * Description:
 *   Create a data transfer or SET data phase qTD.
 *
 ****************************************************************************/

static struct imxrt_qtd_s *imxrt_qtd_dataphase(struct imxrt_epinfo_s *epinfo,
                                               void *buffer, int buflen,
                                               uint32_t tokenbits)
{
  struct imxrt_qtd_s *qtd;
  uint32_t regval;
  int ret;

  /* Allocate a new Queue Element Transfer Descriptor (qTD) */

  qtd = imxrt_qtd_alloc();
  if (qtd == NULL)
    {
      usbhost_trace1(EHCI_TRACE1_DATAQTDALLOC_FAILED, 0);
      return NULL;
    }

  /* Mark this as the end of the list (this will be overwritten if another
   * qTD is added after this one).
   */

  qtd->hw.nqp = imxrt_swap32(QTD_NQP_T);
  qtd->hw.alt = imxrt_swap32(QTD_AQP_T);

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

  qtd->hw.token = imxrt_swap32(regval);

  /* Add the buffer information to the buffer pointer list */

  ret = imxrt_qtd_addbpl(qtd, buffer, buflen);
  if (ret < 0)
    {
      usbhost_trace1(EHCI_TRACE1_ADDBPL_FAILED, -ret);
      imxrt_qtd_free(qtd);
      return NULL;
    }

  /* Add the data transfer size to the count in the epinfo structure */

  epinfo->xfrd += buflen;

  return qtd;
}

/****************************************************************************
 * Name: imxrt_qtd_statusphase
 *
 * Description:
 *   Create a STATUS phase request qTD.
 *
 ****************************************************************************/

static struct imxrt_qtd_s *imxrt_qtd_statusphase(uint32_t tokenbits)
{
  struct imxrt_qtd_s *qtd;
  uint32_t regval;

  /* Allocate a new Queue Element Transfer Descriptor (qTD) */

  qtd = imxrt_qtd_alloc();
  if (qtd == NULL)
    {
      usbhost_trace1(EHCI_TRACE1_REQQTDALLOC_FAILED, 0);
      return NULL;
    }

  /* Mark this as the end of the list (this will be overwritten if another
   * qTD is added after this one).
   */

  qtd->hw.nqp = imxrt_swap32(QTD_NQP_T);
  qtd->hw.alt = imxrt_swap32(QTD_AQP_T);

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

  qtd->hw.token = imxrt_swap32(regval);
  return qtd;
}

/****************************************************************************
 * Name: imxrt_async_setup
 *
 * Description:
 *   Process a IN or OUT request on any asynchronous endpoint (bulk or
 *   control).  This function will enqueue the request and wait for it to
 *   complete.  Bulk data transfers differ in that req == NULL and there are
 *   not SETUP or STATUS phases.
 *
 *   This is a blocking function; it will not return until the control
 *   transfer has completed.
 *
 * Assumption:  The caller holds the EHCI lock.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

static int imxrt_async_setup(struct imxrt_rhport_s *rhport,
                             struct imxrt_epinfo_s *epinfo,
                             const struct usb_ctrlreq_s *req,
                             uint8_t *buffer, size_t buflen)
{
  struct imxrt_qh_s *qh;
  struct imxrt_qtd_s *qtd;
  uintptr_t physaddr;
  uint32_t *flink;
  uint32_t *alt;
  uint32_t toggle;
  bool dirin = false;
  int ret;

  /* Terse output only if we are tracing */

#ifdef CONFIG_USBHOST_TRACE
  usbhost_vtrace2(EHCI_VTRACE2_ASYNCXFR, epinfo->epno, buflen);
#else
  uinfo("RHport%d EP%d: buffer=%p, buflen=%d, req=%p\n",
        RHPORT(rhport), epinfo->epno, buffer, buflen, req);
#endif

  DEBUGASSERT(rhport && epinfo);

  /* A buffer may or may be supplied with an EP0 SETUP transfer.  A buffer
   * will always be present for normal endpoint data transfers.
   */

  DEBUGASSERT(req || (buffer && buflen > 0));

  /* Create and initialize a Queue Head (QH) structure for this transfer */

  qh = imxrt_qh_create(rhport, epinfo);
  if (qh == NULL)
    {
      usbhost_trace1(EHCI_TRACE1_QHCREATE_FAILED, 0);
      return -ENOMEM;
    }

  /* Initialize the QH link and get the next data toggle (not used for SETUP
   * transfers)
   */

  flink  = &qh->hw.overlay.nqp;
  toggle = (uint32_t)epinfo->toggle << QTD_TOKEN_TOGGLE_SHIFT;
  ret    = -EIO;

  /* Is there an EP0 SETUP request?  If so, req will be non-NULL and we will
   * queue two or three qTDs:
   *
   *   1) One for the SETUP phase,
   *   2) One for the DATA phase (if there is data), and
   *   3) One for the STATUS phase.
   *
   * If this is not an EP0 SETUP request, then only a data transfer will be
   * enqueued.
   */

  if (req != NULL)
    {
      /* Allocate a new Queue Element Transfer Descriptor (qTD) for the SETUP
       * phase of the request sequence.
       */

      qtd = imxrt_qtd_setupphase(epinfo, req);
      if (qtd == NULL)
        {
          usbhost_trace1(EHCI_TRACE1_QTDSETUP_FAILED, 0);
          goto errout_with_qh;
        }

      /* Link the new qTD to the QH head. */

      physaddr = imxrt_physramaddr((uintptr_t)qtd);
      *flink = imxrt_swap32(physaddr);

      /* Get the new forward link pointer and data toggle */

      flink  = &qtd->hw.nqp;
      toggle = QTD_TOKEN_TOGGLE;
    }

  /* A buffer may or may be supplied with an EP0 SETUP transfer.  A buffer
   * will always be present for normal endpoint data transfers.
   */

  alt = NULL;
  if (buffer != NULL && buflen > 0)
    {
      uint32_t tokenbits;

      /* Extra TOKEN bits include the data toggle, the data PID, and if
       * there is no request, an indication to interrupt at the end of this
       * transfer.
       */

      tokenbits = toggle;

      /* Get the data token direction.
       *
       * If this is a SETUP request, use the direction contained in the
       * request.  The IOC bit is not set.
       */

      if (req)
        {
          if ((req->type & USB_REQ_DIR_MASK) == USB_REQ_DIR_IN)
            {
              tokenbits |= QTD_TOKEN_PID_IN;
              dirin      = true;
            }
          else
            {
              tokenbits |= QTD_TOKEN_PID_OUT;
              dirin      = false;
            }
        }

      /* Otherwise, the endpoint is uni-directional.  Get the direction from
       * the epinfo structure.  Since this is not an EP0 SETUP request,
       * nothing follows the data and we want the IOC interrupt when the
       * data transfer completes.
       */

      else if (epinfo->dirin)
        {
          tokenbits |= (QTD_TOKEN_PID_IN | QTD_TOKEN_IOC);
          dirin      = true;
        }
      else
        {
          tokenbits |= (QTD_TOKEN_PID_OUT | QTD_TOKEN_IOC);
          dirin      = false;
        }

      /* Allocate a new Queue Element Transfer Descriptor (qTD) for the data
       * buffer.
       */

      qtd = imxrt_qtd_dataphase(epinfo, buffer, buflen, tokenbits);
      if (qtd == NULL)
        {
          usbhost_trace1(EHCI_TRACE1_QTDDATA_FAILED, 0);
          goto errout_with_qh;
        }

      /* Link the new qTD to either QH head of the SETUP qTD. */

      physaddr = imxrt_physramaddr((uintptr_t)qtd);
      *flink = imxrt_swap32(physaddr);

      /* Set the forward link pointer to this new qTD */

      flink = &qtd->hw.nqp;

      /* If this was an IN transfer, then setup a pointer alternate link.
       * The EHCI hardware will use this link if a short packet is received.
       */

      if (dirin)
        {
          alt = &qtd->hw.alt;
        }
    }

  /* If this is an EP0 SETUP request, then enqueue one more qTD for the
   * STATUS phase transfer.
   */

  if (req != NULL)
    {
      /* Extra TOKEN bits include the data toggle and the correct data PID. */

      uint32_t tokenbits = toggle;

      /* The status phase direction is the opposite of the data phase.  If
       * this is an IN request, then we received the buffer and we will send
       * the zero length packet handshake.
       */

      if ((req->type & USB_REQ_DIR_MASK) == USB_REQ_DIR_IN)
        {
          tokenbits |= QTD_TOKEN_PID_OUT;
        }

      /* Otherwise, this in an OUT request.  We send the buffer and we expect
       * to receive the NULL packet handshake.
       */

      else
        {
          tokenbits |= QTD_TOKEN_PID_IN;
        }

      /* Allocate a new Queue Element Transfer Descriptor (qTD)
       * for the status
       */

      qtd = imxrt_qtd_statusphase(tokenbits);
      if (qtd == NULL)
        {
          usbhost_trace1(EHCI_TRACE1_QTDSTATUS_FAILED, 0);
          goto errout_with_qh;
        }

      /* Link the new qTD to either the SETUP or data qTD. */

      physaddr = imxrt_physramaddr((uintptr_t)qtd);
      *flink = imxrt_swap32(physaddr);

      /* In an IN data qTD was also enqueued, then linked the data qTD's
       * alternate pointer to this STATUS phase qTD in order to handle short
       * transfers.
       */

      if (alt)
        {
          *alt = imxrt_swap32(physaddr);
        }
    }

  /* Add the new QH to the head of the asynchronous queue list */

  imxrt_qh_enqueue(&g_asynchead, qh);
  return OK;

  /* Clean-up after an error */

errout_with_qh:
  imxrt_qh_discard(qh);
  return ret;
}

/****************************************************************************
 * Name: imxrt_intr_setup
 *
 * Description:
 *   Process a IN or OUT request on any interrupt endpoint by inserting a qTD
 *   into the periodic frame list.
 *
 *  Paragraph 4.10.7 "Adding Interrupt Queue Heads to the Periodic Schedule"
 *    "The link path(s) from the periodic frame list to a queue head
 *     establishes in which frames a transaction can be executed for the
 *     queue head. Queue heads are linked into the periodic schedule so they
 *     are polled at the appropriate rate. System software sets a bit in a
 *     queue head's S-Mask to indicate which micro-frame with-in a 1
 *     millisecond period a transaction should be executed for the queue
 *     head. Software must ensure that all queue heads in the periodic
 *     schedule have S-Mask set to a non-zero value. An S-mask with a zero
 *     value in the context of the periodic schedule yields undefined
 *     results.
 *
 *    "If the desired poll rate is greater than one frame, system software
 *     can use a combination of queue head linking and S-Mask values to
 *     spread interrupts of equal poll rates through the schedule so that the
 *     periodic bandwidth is allocated and managed in the most efficient
 *     manner possible."
 *
 *  Paragraph 4.6 "Periodic Schedule"
 *
 *    "The periodic schedule is used to manage all isochronous and interrupt
 *     transfer streams. The base of the periodic schedule is the periodic
 *     frame list. Software links schedule data structures to the periodic
 *     frame list to produce a graph of scheduled data structures. The graph
 *     represents an appropriate sequence of transactions on the USB. ...
 *     isochronous transfers (using iTDs and siTDs) with a period of one are
 *     linked directly to the periodic frame list. Interrupt transfers (are
 *     managed with queue heads) and isochronous streams with periods other
 *     than one are linked following the period-one iTD/siTDs. Interrupt
 *     queue heads are linked into the frame list ordered by poll rate.
 *     Longer poll rates are linked first (e.g. closest to the periodic
 *     frame list), followed by shorter poll rates, with queue heads with a
 *     poll rate of one, on the very end."
 *
 * Assumption:  The caller holds the EHCI lock.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

#ifndef CONFIG_USBHOST_INT_DISABLE
static int imxrt_intr_setup(struct imxrt_rhport_s *rhport,
                            struct imxrt_epinfo_s *epinfo,
                            uint8_t *buffer, size_t buflen)
{
  struct imxrt_qh_s *qh;
  struct imxrt_qtd_s *qtd;
  uintptr_t physaddr;
  uint32_t tokenbits;
  uint32_t regval;
  int ret;

  /* Terse output only if we are tracing */

#ifdef CONFIG_USBHOST_TRACE
  usbhost_vtrace2(EHCI_VTRACE2_INTRXFR, epinfo->epno, buflen);
#else
  uinfo("RHport%d EP%d: buffer=%p, buflen=%d\n",
        RHPORT(rhport), epinfo->epno, buffer, buflen);
#endif

  DEBUGASSERT(rhport && epinfo && buffer && buflen > 0);

  /* Create and initialize a Queue Head (QH) structure for this transfer */

  qh = imxrt_qh_create(rhport, epinfo);
  if (qh == NULL)
    {
      usbhost_trace1(EHCI_TRACE1_QHCREATE_FAILED, 0);
      return -ENOMEM;
    }

  /* Extra TOKEN bits include the data toggle, the data PID, and an
   * indication to interrupt at the end of this transfer.
   */

  tokenbits = (uint32_t)epinfo->toggle << QTD_TOKEN_TOGGLE_SHIFT;

  /* Get the data token direction. */

  if (epinfo->dirin)
    {
      tokenbits |= (QTD_TOKEN_PID_IN | QTD_TOKEN_IOC);
    }
  else
    {
      tokenbits |= (QTD_TOKEN_PID_OUT | QTD_TOKEN_IOC);
    }

  /* Allocate a new Queue Element Transfer Descriptor (qTD) for the data
   * buffer.
   */

  qtd = imxrt_qtd_dataphase(epinfo, buffer, buflen, tokenbits);
  if (qtd == NULL)
    {
      usbhost_trace1(EHCI_TRACE1_QTDDATA_FAILED, 0);
      ret = -ENOMEM;
      goto errout_with_qh;
    }

  /* Link the new qTD to the QH. */

  physaddr = imxrt_physramaddr((uintptr_t)qtd);
  qh->hw.overlay.nqp = imxrt_swap32(physaddr);

  /* Disable the periodic schedule */

  regval  = imxrt_getreg(&HCOR->usbcmd);
  regval &= ~EHCI_USBCMD_PSEN;
  imxrt_putreg(regval, &HCOR->usbcmd);

  /* Add the new QH to the head of the interrupt transfer list */

  imxrt_qh_enqueue(&g_intrhead, qh);

  /* Re-enable the periodic schedule */

  regval |= EHCI_USBCMD_PSEN;
  imxrt_putreg(regval, &HCOR->usbcmd);
  return OK;

  /* Clean-up after an error */

errout_with_qh:
  imxrt_qh_discard(qh);
  return ret;
}
#endif /* CONFIG_USBHOST_INT_DISABLE */

/****************************************************************************
 * Name: imxrt_transfer_wait
 *
 * Description:
 *   Wait for an IN or OUT transfer to complete.
 *
 * Assumption:  The caller holds the EHCI lock.  The caller must be aware
 *   that the EHCI lock will released while waiting for the transfer to
 *   complete, but will be re-acquired when before returning.  The state of
 *   EHCI resources could be very different upon return.
 *
 * Returned Value:
 *   On success, this function returns the number of bytes actually
 *   transferred.  For control transfers, this size includes the size of the
 *   control request plus the size of the data (which could be short); for
 *   bulk transfers, this will be the number of data bytes transfers (which
 *   could be short).
 *
 ****************************************************************************/

static ssize_t imxrt_transfer_wait(struct imxrt_epinfo_s *epinfo)
{
  int ret;
  int ret2;

  /* Release the EHCI lock while we wait.  Other threads need the
   * opportunity to access the EHCI resources while we wait.
   *
   * REVISIT:  Is this safe?  NO.  This is a bug and needs rethinking.
   * We need to lock all of the port-resources (not EHCI common) until
   * the transfer is complete.  But we can't use the common EHCI lock
   * or we will deadlock while waiting (because the working thread that
   * wakes this thread up needs the lock).
   */

  /* REVISIT */

  nxmutex_unlock(&g_ehci.lock);

  /* Wait for the IOC completion event */

  ret = imxrt_ioc_wait(epinfo);

  /* Re-acquire the EHCI lock.  The caller expects to be holding
   * this upon return.
   */

  ret2 = nxmutex_lock(&g_ehci.lock);
  if (ret >= 0 && ret2 < 0)
    {
      ret = ret2;
    }

#if 0 /* Does not seem to be needed */
  /* Was there a data buffer?  Was this an OUT transfer? */

  if (buffer != NULL && buflen > 0 && !dirin)
    {
      /* We have received data from the host -- unless there was an error.
       * in any event, we will invalidate the data buffer so that we will
       * reload any new data freshly DMAed into the user buffer.
       *
       * NOTE: This might be un-necessary.  We cleaned and invalidated the
       * D-Cache prior to starting the DMA so the D-Cache should still be
       * invalid in this memory region.
       */

      up_invalidate_dcache((uintptr_t)buffer, (uintptr_t)buffer + buflen);
    }
#endif

  /* Did imxrt_ioc_wait() or nxmutex_lock() report an error? */

  if (ret < 0)
    {
      usbhost_trace1(EHCI_TRACE1_TRANSFER_FAILED, -ret);
      epinfo->iocwait = false;
      return (ssize_t)ret;
    }

  /* Transfer completed successfully.  Return the number of bytes
   * transferred.
   */

  return epinfo->xfrd;
}

/****************************************************************************
 * Name: imxrt_ioc_async_setup
 *
 * Description:
 *   Setup to receive an asynchronous notification when a transfer completes.
 *
 * Input Parameters:
 *   epinfo - The IN or OUT endpoint descriptor for the device endpoint on
 *      which the transfer will be performed.
 *   callback - The function to be called when the completes
 *   arg - An arbitrary argument that will be provided with the callback.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Called from the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static inline int imxrt_ioc_async_setup(struct imxrt_rhport_s *rhport,
                                        struct imxrt_epinfo_s *epinfo,
                                        usbhost_asynch_t callback,
                                        void *arg)
{
  irqstate_t flags;
  int ret = -ENODEV;

  DEBUGASSERT(rhport && epinfo && !epinfo->iocwait &&
              callback != NULL && epinfo->callback == NULL);

  /* Is the device still connected? */

  flags = enter_critical_section();
  if (rhport->connected)
    {
      /* Then save callback information to used when either (1) the
       * device is disconnected, or (2) the transfer completes.
       */

      epinfo->iocwait  = false;    /* No synchronous wakeup */
      epinfo->status   = 0;        /* No status yet */
      epinfo->xfrd     = 0;        /* Nothing transferred yet */
      epinfo->result   = -EBUSY;   /* Transfer in progress */
      epinfo->callback = callback; /* Asynchronous callback */
      epinfo->arg      = arg;      /* Argument that accompanies the callback */
      ret              = OK;       /* We are good to go */
    }

  leave_critical_section(flags);
  return ret;
}
#endif

/****************************************************************************
 * Name: imxrt_asynch_completion
 *
 * Description:
 *   This function is called at the interrupt level when an asynchronous
 *   transfer completes.  It performs the pending callback.
 *
 * Input Parameters:
 *   epinfo - The IN or OUT endpoint descriptor for the device endpoint on
 *      which the transfer was performed.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Called from the interrupt level
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static void imxrt_asynch_completion(struct imxrt_epinfo_s *epinfo)
{
  usbhost_asynch_t callback;
  ssize_t nbytes;
  void *arg;
  int result;

  DEBUGASSERT(epinfo != NULL && epinfo->iocwait == false &&
              epinfo->callback != NULL);

  /* Extract and reset the callback info */

  callback         = epinfo->callback;
  arg              = epinfo->arg;
  result           = epinfo->result;
  nbytes           = epinfo->xfrd;

  epinfo->callback = NULL;
  epinfo->arg      = NULL;
  epinfo->result   = OK;
  epinfo->iocwait  = false;

  /* Then perform the callback.  Provide the number of bytes successfully
   * transferred or the negated errno value in the event of a failure.
   */

  if (result < 0)
    {
      nbytes = (ssize_t)result;
    }

  callback(arg, nbytes);
}
#endif

/****************************************************************************
 * Name: imxrt_qtd_ioccheck
 *
 * Description:
 *   This function is a imxrt_qtd_foreach() callback function.  It services
 *   one qTD in the asynchronous queue.  It removes all of the qTD
 *   structures that are no longer active.
 *
 ****************************************************************************/

static int imxrt_qtd_ioccheck(struct imxrt_qtd_s *qtd, uint32_t **bp,
                              void *arg)
{
  struct imxrt_epinfo_s *epinfo = (struct imxrt_epinfo_s *)arg;
  DEBUGASSERT(qtd && epinfo);

  /* Make sure we reload the QH from memory */

  up_invalidate_dcache((uintptr_t)&qtd->hw,
                       (uintptr_t)&qtd->hw + sizeof(struct ehci_qtd_s));
  imxrt_qtd_print(qtd);

  /* Remove the qTD from the list
   *
   * NOTE that we don't check if the qTD is active nor do we check if there
   * are any errors reported in the qTD.  If the transfer halted due to
   * an error, then qTDs in the list after the error qTD will still appear
   * to be active.
   */

  **bp = qtd->hw.nqp;

  /* Subtract the number of bytes left un-transferred.  The epinfo->xfrd
   * field is initialized to the total number of bytes to be transferred
   * (all qTDs in the list).  We subtract out the number of un-transferred
   * bytes on each transfer and the final result will be the number of bytes
   * actually transferred.
   */

  epinfo->xfrd -= (imxrt_swap32(qtd->hw.token) & QTD_TOKEN_NBYTES_MASK) >>
    QTD_TOKEN_NBYTES_SHIFT;

  /* Release this QH by returning it to the free list */

  imxrt_qtd_free(qtd);
  return OK;
}

/****************************************************************************
 * Name: imxrt_qh_ioccheck
 *
 * Description:
 *   This function is a imxrt_qh_foreach() callback function.  It services
 *   one QH in the asynchronous queue.  It check all attached qTD structures
 *   and remove all of the structures that are no longer active.  if all of
 *   the qTD structures are removed, then QH itself will also be removed.
 *
 ****************************************************************************/

static int imxrt_qh_ioccheck(struct imxrt_qh_s *qh, uint32_t **bp, void *arg)
{
  struct imxrt_epinfo_s *epinfo;
  uint32_t token;
  int ret;

  DEBUGASSERT(qh && bp);

  /* Make sure we reload the QH from memory */

  up_invalidate_dcache((uintptr_t)&qh->hw,
                       (uintptr_t)&qh->hw + sizeof(struct ehci_qh_s));
  imxrt_qh_print(qh);

  /* Get the endpoint info pointer from the extended QH data.  Only the
   * g_asynchead QH can have a NULL epinfo field.
   */

  epinfo = qh->epinfo;
  DEBUGASSERT(epinfo);

  /* Paragraph 3.6.3:  "The nine DWords in [the Transfer Overlay] area
   * represent a transaction working space for the host controller.  The
   * general operational model is that the host controller can detect
   * whether the overlay area contains a description of an active transfer.
   * If it does not contain an active transfer, then it follows the Queue
   * Head Horizontal Link Pointer to the next queue head.  The host
   * controller will never follow the Next Transfer Queue Element or
   * Alternate Queue Element pointers unless it is actively attempting to
   * advance the queue ..."
   */

  /* Is the qTD still active? */

  token = imxrt_swap32(qh->hw.overlay.token);
  usbhost_vtrace2(EHCI_VTRACE2_IOCCHECK, epinfo->epno, token);

  if ((token & QH_TOKEN_ACTIVE) != 0)
    {
      /* Yes... we cannot process the QH while it is still active.  Return
       * zero to visit the next QH in the list.
       */

      *bp = &qh->hw.hlp;
      return OK;
    }

  /* Remove all active, attached qTD structures from the inactive QH */

  ret = imxrt_qtd_foreach(qh, imxrt_qtd_ioccheck, (void *)qh->epinfo);
  if (ret < 0)
    {
      usbhost_trace1(EHCI_TRACE1_QTDFOREACH_FAILED, -ret);
    }

  /* If there is no longer anything attached to the QH, then remove it from
   * the asynchronous queue.
   */

  if ((imxrt_swap32(qh->fqp) & QTD_NQP_T) != 0)
    {
      /* Set the forward link of the previous QH to point to the next
       * QH in the list.
       */

      **bp = qh->hw.hlp;
      up_flush_dcache((uintptr_t)*bp, (uintptr_t)*bp + sizeof(uint32_t));

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

          epinfo->status = (token & QH_TOKEN_STATUS_MASK) >>
                           QH_TOKEN_STATUS_SHIFT;

          /* The HALT condition is set on a variety of conditions:  babble,
           * error counter countdown to zero, or a STALL.  If we can rule
           * out babble (babble bit not set) and if the error counter is
           * non-zero, then we can assume a STALL. In this case, we return
           * -PERM to inform the class driver of the stall condition.
           */

          if ((token & (QH_TOKEN_BABBLE | QH_TOKEN_HALTED)) ==
               QH_TOKEN_HALTED &&
              (token & QH_TOKEN_CERR_MASK) != 0)
            {
              /* It is a stall,  Note that the data toggle is reset
               * after the stall.
               */

              usbhost_trace2(EHCI_TRACE2_EPSTALLED, epinfo->epno, token);
              epinfo->result = -EPERM;
              epinfo->toggle = 0;
            }
          else
            {
              /* Otherwise, it is some kind of data transfer error */

              usbhost_trace2(EHCI_TRACE2_EPIOERROR, epinfo->epno, token);
              epinfo->result = -EIO;
            }
        }

      /* Is there a thread waiting for this transfer to complete? */

      if (epinfo->iocwait)
        {
          /* Yes... wake it up */

          epinfo->iocwait = false;
          nxsem_post(&epinfo->iocsem);
        }

#ifdef CONFIG_USBHOST_ASYNCH
      /* No.. Is there a pending asynchronous transfer? */

      else if (epinfo->callback != NULL)
        {
          /* Yes.. perform the callback */

          imxrt_asynch_completion(epinfo);
        }
#endif

      /* Then release this QH by returning it to the free list */

      imxrt_qh_free(qh);
    }
  else
    {
      /* Otherwise, the horizontal link pointer of this QH will become the
       * next back pointer.
       */

      *bp = &qh->hw.hlp;
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_qtd_cancel
 *
 * Description:
 *   This function is a imxrt_qtd_foreach() callback function.  It removes
 *   each qTD attached to a QH.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int imxrt_qtd_cancel(struct imxrt_qtd_s *qtd, uint32_t **bp,
                            void *arg)
{
  DEBUGASSERT(qtd != NULL && bp != NULL);

  /* Make sure we reload the QH from memory */

  up_invalidate_dcache((uintptr_t)&qtd->hw,
                       (uintptr_t)&qtd->hw + sizeof(struct ehci_qtd_s));
  imxrt_qtd_print(qtd);

  /* Remove the qTD from the list
   *
   * NOTE that we don't check if the qTD is active nor do we check if there
   * are any errors reported in the qTD.  If the transfer halted due to
   * an error, then qTDs in the list after the error qTD will still appear
   * to be active.
   *
   * REVISIT: There is a race condition here that needs to be resolved.
   */

  **bp = qtd->hw.nqp;

  /* Release this QH by returning it to the free list */

  imxrt_qtd_free(qtd);
  return OK;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/****************************************************************************
 * Name: imxrt_qh_cancel
 *
 * Description:
 *   This function is a imxrt_qh_foreach() callback function.  It cancels
 *   one QH in the asynchronous queue.  It will remove all attached qTD
 *   structures and remove all of the structures that are no longer active.
 *   Then QH itself will also be removed.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int imxrt_qh_cancel(struct imxrt_qh_s *qh, uint32_t **bp, void *arg)
{
  struct imxrt_epinfo_s *epinfo = (struct imxrt_epinfo_s *)arg;
  uint32_t regval;
  int ret;

  DEBUGASSERT(qh != NULL && bp != NULL && epinfo != NULL);

  /* Make sure we reload the QH from memory */

  up_invalidate_dcache((uintptr_t)&qh->hw,
                       (uintptr_t)&qh->hw + sizeof(struct ehci_qh_s));
  imxrt_qh_print(qh);

  /* Check if this is the QH that we are looking for */

  if (qh->epinfo == epinfo)
    {
      /* No... keep looking */

      return OK;
    }

  /* Disable both the asynchronous and period schedules */

  regval = imxrt_getreg(&HCOR->usbcmd);
  imxrt_putreg(regval & ~(EHCI_USBCMD_ASEN | EHCI_USBCMD_PSEN),
               &HCOR->usbcmd);

  /* Remove the QH from the list
   *
   * NOTE that we don't check if the qTD is active nor do we check if there
   * are any errors reported in the qTD.  If the transfer halted due to
   * an error, then qTDs in the list after the error qTD will still appear
   * to be active.
   *
   * REVISIT: There is a race condition here that needs to be resolved.
   */

  **bp = qh->hw.hlp;
  up_flush_dcache((uintptr_t)*bp, (uintptr_t)*bp + sizeof(uint32_t));

  /* Re-enable the schedules (if they were enabled before. */

  imxrt_putreg(regval, &HCOR->usbcmd);

  /* Remove all active, attached qTD structures from the removed QH */

  ret = imxrt_qtd_foreach(qh, imxrt_qtd_cancel, NULL);
  if (ret < 0)
    {
      usbhost_trace1(EHCI_TRACE1_QTDFOREACH_FAILED, -ret);
    }

  /* Then release this QH by returning it to the free list.  Return 1
   * to stop the traverse without an error.
   */

  imxrt_qh_free(qh);
  return 1;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/****************************************************************************
 * Name: imxrt_ioc_bottomhalf
 *
 * Description:
 *   EHCI USB Interrupt (USBINT) "Bottom Half" interrupt handler
 *
 *  "The Host Controller sets this bit to 1 on the completion of a USB
 *   transaction, which results in the retirement of a Transfer Descriptor
 *   that had its IOC bit set.
 *
 *  "The Host Controller also sets this bit to 1 when a short packet is
 *   detected (actual number of bytes received was less than the expected
 *   number of bytes)."
 *
 * Assumptions:  The caller holds the EHCI lock
 *
 ****************************************************************************/

static inline void imxrt_ioc_bottomhalf(void)
{
  struct imxrt_qh_s *qh;
  uint32_t *bp;
  int ret;

  /* Check the Asynchronous Queue
   * Make sure that the head of the asynchronous queue is invalidated.
   */

  up_invalidate_dcache((uintptr_t)&g_asynchead.hw,
                       (uintptr_t)&g_asynchead.hw +
                       sizeof(struct ehci_qh_s));

  /* Set the back pointer to the forward QH pointer of the asynchronous
   * queue head.
   */

  bp = (uint32_t *)&g_asynchead.hw.hlp;
  qh = (struct imxrt_qh_s *)
       imxrt_virtramaddr(imxrt_swap32(*bp) & QH_HLP_MASK);

  /* If the asynchronous queue is empty, then the forward point in the
   * asynchronous queue head will point back to the queue head.
   */

  if (qh && qh != &g_asynchead)
    {
      /* Then traverse and operate on every QH and qTD in the asynchronous
       * queue
       */

      ret = imxrt_qh_foreach(qh, &bp, imxrt_qh_ioccheck, NULL);
      if (ret < 0)
        {
          usbhost_trace1(EHCI_TRACE1_QHFOREACH_FAILED, -ret);
        }
    }

#ifndef CONFIG_USBHOST_INT_DISABLE

  /* Check the Interrupt Queue
   * Make sure that the head of the interrupt queue is invalidated.
   */

  up_invalidate_dcache((uintptr_t)&g_intrhead.hw,
                       (uintptr_t)&g_intrhead.hw + sizeof(struct ehci_qh_s));

  /* Set the back pointer to the forward qTD pointer of the asynchronous
   * queue head.
   */

  bp = (uint32_t *)&g_intrhead.hw.hlp;
  qh = (struct imxrt_qh_s *)
       imxrt_virtramaddr(imxrt_swap32(*bp) & QH_HLP_MASK);
  if (qh)
    {
      /* Then traverse and operate on every QH and qTD in the asynchronous
       * queue.
       */

      ret = imxrt_qh_foreach(qh, &bp, imxrt_qh_ioccheck, NULL);
      if (ret < 0)
        {
          usbhost_trace1(EHCI_TRACE1_QHFOREACH_FAILED, -ret);
        }
    }
#endif
}

/****************************************************************************
 * Name: imxrt_portsc_bottomhalf
 *
 * Description:
 *   EHCI Port Change Detect "Bottom Half" interrupt handler
 *
 *  "The Host Controller sets this bit to a one when any port for which the
 *   Port Owner bit is set to zero ... has a change bit transition from a
 *   zero to a one or a Force Port Resume bit transition from a zero to a
 *   one as a result of a J-K transition detected on a suspended port.
 *   This bit will also be set as a result of the Connect Status Change
 *   being set to a one after system software has relinquished ownership of
 *   a connected port by writing a one to a port's Port Owner bit...
 *
 *  "This bit is allowed to be maintained in the Auxiliary power well.
 *   Alternatively, it is also acceptable that on a D3 to D0 transition of
 *   the EHCI HC device, this bit is loaded with the OR of all of the PORTSC
 *   change bits (including: Force port resume, over-current change,
 *   enable/disable change and connect status change)."
 *
 ****************************************************************************/

static inline void imxrt_portsc_bottomhalf(void)
{
  struct imxrt_rhport_s *rhport;
  struct usbhost_hubport_s *hport;
  uint32_t portsc;
  int rhpndx;

  /* Handle root hub status change on each root port */

  for (rhpndx = 0; rhpndx < IMXRT_EHCI_NRHPORT; rhpndx++)
    {
      rhport = &g_ehci.rhport[rhpndx];
      portsc = imxrt_getreg(&HCOR->portsc[rhpndx]);

      usbhost_vtrace2(EHCI_VTRACE2_PORTSC, rhpndx + 1, portsc);

      /* Handle port connection status change (CSC) events */

      if ((portsc & EHCI_PORTSC_CSC) != 0)
        {
          usbhost_vtrace1(EHCI_VTRACE1_PORTSC_CSC, portsc);

          /* Check current connect status */

          if ((portsc & EHCI_PORTSC_CCS) != 0)
            {
              /* Connected ... Did we just become connected? */

              if (!rhport->connected)
                {
                  /* Yes.. connected. */

                  rhport->connected = true;

                  usbhost_vtrace2(EHCI_VTRACE2_PORTSC_CONNECTED,
                                  rhpndx + 1, g_ehci.pscwait);

                  /* Notify any waiters */

                  if (g_ehci.pscwait)
                    {
                      nxsem_post(&g_ehci.pscsem);
                      g_ehci.pscwait = false;
                    }
                }
              else
                {
                  usbhost_vtrace1(EHCI_VTRACE1_PORTSC_CONNALREADY, portsc);
                }
            }
          else
            {
              /* Disconnected... Did we just become disconnected? */

              if (rhport->connected)
                {
                  /* Yes.. disconnect the device */

                  usbhost_vtrace2(EHCI_VTRACE2_PORTSC_DISCONND,
                                  rhpndx + 1, g_ehci.pscwait);

                  rhport->connected = false;
                  rhport->lowspeed  = false;

                  /* Are we bound to a class instance? */

                  hport = &rhport->hport.hport;
                  if (hport->devclass)
                    {
                      /* Yes.. Disconnect the class */

                      CLASS_DISCONNECTED(hport->devclass);
                      hport->devclass = NULL;
                    }

                  /* Notify any waiters for the Root Hub Status change
                   * event.
                   */

                  if (g_ehci.pscwait)
                    {
                      nxsem_post(&g_ehci.pscsem);
                      g_ehci.pscwait = false;
                    }
                }
              else
                {
                   usbhost_vtrace1(EHCI_VTRACE1_PORTSC_DISCALREADY, portsc);
                }
            }
        }

      /* Clear all pending port interrupt sources by writing a '1' to the
       * corresponding bit in the PORTSC register.  In addition, we need
       * to preserve the values of all R/W bits (RO bits don't matter)
       */

      imxrt_putreg(portsc, &HCOR->portsc[rhpndx]);
    }
}

/****************************************************************************
 * Name: imxrt_syserr_bottomhalf
 *
 * Description:
 *   EHCI Host System Error "Bottom Half" interrupt handler
 *
 *  "The Host Controller sets this bit to 1 when a serious error occurs
 *   during a host system access involving the Host Controller module. ...
 *   When this error occurs, the Host Controller clears the Run/Stop bit in
 *   the Command register to prevent further execution of the scheduled TDs."
 *
 ****************************************************************************/

static inline void imxrt_syserr_bottomhalf(void)
{
  usbhost_trace1(EHCI_TRACE1_SYSERR_INTR, 0);
  DEBUGPANIC();
}

/****************************************************************************
 * Name: imxrt_async_advance_bottomhalf
 *
 * Description:
 *   EHCI Async Advance "Bottom Half" interrupt handler
 *
 *  "System software can force the host controller to issue an interrupt the
 *   next time the host controller advances the asynchronous schedule by
 *   writing a one to the Interrupt on Async Advance Doorbell bit in the
 *   USBCMD register. This status bit indicates the assertion of that
 *   interrupt source."
 *
 ****************************************************************************/

static inline void imxrt_async_advance_bottomhalf(void)
{
  usbhost_vtrace1(EHCI_VTRACE1_AAINTR, 0);

  /* REVISIT: Could remove all tagged QH entries here */
}

/****************************************************************************
 * Name: imxrt_ehci_bottomhalf
 *
 * Description:
 *   EHCI "Bottom Half" interrupt handler.  Runs on a work queue thread.
 *
 ****************************************************************************/

static void imxrt_ehci_bottomhalf(void *arg)
{
  uint32_t pending = (uint32_t)arg;

  /* We need to have exclusive access to the EHCI data structures.  Waiting
   * here is not a good thing to do on the worker thread, but there is no
   * real option (other than to reschedule and delay).
   */

  nxmutex_lock(&g_ehci.lock);

  /* Handle all unmasked interrupt sources
   * USB Interrupt (USBINT)
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
   * We do the same thing in either case:  Traverse the asynchronous queue
   * and remove all of the transfers that are no longer active.
   */

  if ((pending & (EHCI_INT_USBINT | EHCI_INT_USBERRINT)) != 0)
    {
      if ((pending & EHCI_INT_USBERRINT) != 0)
        {
          usbhost_trace1(EHCI_TRACE1_USBERR_INTR, pending);
        }
      else
        {
          usbhost_vtrace1(EHCI_VTRACE1_USBINTR, pending);
        }

      imxrt_ioc_bottomhalf();
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
      imxrt_portsc_bottomhalf();
    }

  /* Frame List Rollover
   *
   *  "The Host Controller sets this bit to a one when the Frame List Index
   *   ... rolls over from its maximum value to zero. The exact value at
   *   which the rollover occurs depends on the frame list size. For example,
   *   if the frame list size (as programmed in the Frame List Size field of
   *   the USBCMD register) is 1024, the Frame Index Register rolls over
   *   every time FRINDEX[13] toggles. Similarly, if the size is 512, the
   *   Host Controller sets this bit to a one every time FRINDEX[12]
   *   toggles."
   */

#if 0 /* Not used */
  if ((pending & EHCI_INT_FLROLL) != 0)
    {
      imxrt_flroll_bottomhalf();
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
      uerr("Syserror\n");
      imxrt_syserr_bottomhalf();
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
      uerr("Async Advance\n");
      imxrt_async_advance_bottomhalf();
    }

  /* We are done with the EHCI structures */

  nxmutex_unlock(&g_ehci.lock);

  /* Re-enable relevant EHCI interrupts.  Interrupts should still be enabled
   * at the level of the interrupt controller.
   */

  imxrt_putreg(EHCI_HANDLED_INTS, &HCOR->usbintr);
}

/****************************************************************************
 * Name: imxrt_ehci_interrupt
 *
 * Description:
 *   EHCI "Top Half" interrupt handler
 *
 ****************************************************************************/

static int imxrt_ehci_interrupt(int irq, void *context, void *arg)
{
  uint32_t usbsts;
  uint32_t pending;
  uint32_t regval;

  /* Read Interrupt Status and mask out interrupts that are not enabled. */

  usbsts = imxrt_getreg(&HCOR->usbsts);
  regval = imxrt_getreg(&HCOR->usbintr);

#ifdef CONFIG_USBHOST_TRACE
  usbhost_vtrace1(EHCI_VTRACE1_TOPHALF, usbsts & regval);
#else
  uinfo("USBSTS: %08x USBINTR: %08x\n", usbsts, regval);
#endif

  /* Handle all unmasked interrupt sources */

  pending = usbsts & regval;
  if (pending != 0)
    {
      /* Schedule interrupt handling work for the high priority worker
       * thread so that we are not pressed for time and so that we can
       * interrupt with other USB threads gracefully.
       *
       * The worker should be available now because we implement a handshake
       * by controlling the EHCI interrupts.
       */

      DEBUGASSERT(work_available(&g_ehci.work));
      DEBUGVERIFY(work_queue(HPWORK, &g_ehci.work, imxrt_ehci_bottomhalf,
                            (void *)pending, 0));

      /* Disable further EHCI interrupts so that we do not overrun the work
       * queue.
       */

      imxrt_putreg(0, &HCOR->usbintr);

      /* Clear all pending status bits by writing the value of the pending
       * interrupt bits back to the status register.
       */

      imxrt_putreg(usbsts & EHCI_INT_ALLINTS, &HCOR->usbsts);
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_wait
 *
 * Description:
 *   Wait for a device to be connected or disconnected to/from a hub port.
 *
 * Input Parameters:
 *   conn - The USB host connection instance obtained as a parameter from the
 *      call to the USB driver initialization logic.
 *   hport - The location to return the hub port descriptor that detected the
 *      connection related event.
 *
 * Returned Value:
 *   Zero (OK) is returned on success when a device is connected or
 *   disconnected. This function will not return until either (1) a device is
 *   connected or disconnect to/from any hub port or until (2) some failure
 *   occurs.  On a failure, a negated errno value is returned indicating the
 *   nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int imxrt_wait(struct usbhost_connection_s *conn,
                      struct usbhost_hubport_s **hport)
{
  irqstate_t flags;
  int rhpndx;
  int ret;

  /* Loop until the connection state changes on one of the root hub ports or
   * until an error occurs.
   */

  flags = enter_critical_section();
  for (; ; )
    {
      /* Check for a change in the connection state on any root hub port */

      for (rhpndx = 0; rhpndx < IMXRT_EHCI_NRHPORT; rhpndx++)
        {
          struct imxrt_rhport_s *rhport;
          struct usbhost_hubport_s *connport;

          /* Has the connection state changed on the RH port? */

          rhport   = &g_ehci.rhport[rhpndx];
          connport = &rhport->hport.hport;
          if (rhport->connected != connport->connected)
            {
              /* Yes.. Return the RH port to inform the caller which
               * port has the connection change.
               */

              connport->connected = rhport->connected;
              *hport = connport;
              leave_critical_section(flags);

              usbhost_vtrace2(EHCI_VTRACE2_MONWAKEUP,
                              rhpndx + 1, rhport->connected);
              return OK;
            }
        }

#ifdef CONFIG_USBHOST_HUB
      /* Is a device connected to an external hub? */

      if (g_ehci.hport)
        {
          volatile struct usbhost_hubport_s *connport;

          /* Yes.. return the external hub port */

          connport = g_ehci.hport;
          g_ehci.hport = NULL;

          *hport = (struct usbhost_hubport_s *)connport;
          leave_critical_section(flags);

          usbhost_vtrace2(EHCI_VTRACE2_MONWAKEUP,
                          connport->port + 1, connport->connected);
          return OK;
        }
#endif

      /* No changes on any port. Wait for a connection/disconnection event
       * and check again
       */

      g_ehci.pscwait = true;
      ret = nxsem_wait_uninterruptible(&g_ehci.pscsem);
      if (ret < 0)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: imxrt_enumerate
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
 *   conn  - The USB host connection instance obtained as a parameter from
 *           the call to the USB driver initialization logic.
 *   hport - The descriptor of the hub port that has the newly connected
 *           device.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int imxrt_rh_enumerate(struct usbhost_connection_s *conn,
                              struct usbhost_hubport_s *hport)
{
  struct imxrt_rhport_s *rhport;
  volatile uint32_t *regaddr;
  uint32_t regval;
  int rhpndx;

  DEBUGASSERT(conn != NULL && hport != NULL);
  rhpndx = hport->port;

  DEBUGASSERT(rhpndx >= 0 && rhpndx < IMXRT_EHCI_NRHPORT);
  rhport = &g_ehci.rhport[rhpndx];

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!rhport->connected)
    {
      /* No, return an error */

      usbhost_vtrace1(EHCI_VTRACE1_ENUM_DISCONN, 0);
      return -ENODEV;
    }

  /* USB 2.0 spec says at least 50ms delay before port reset.
   * REVISIT:  I think this is wrong.  It needs to hold the port in
   * reset for 50Msec, not wait 50Msec before resetting.
   */

  nxsig_usleep(100 * 1000);

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
   *
   * NOTE: Low-speed devices could be detected by examining the PORTSC PSPD
   * field after resetting the device.  The more conventional way here,
   * however, also appears to work.
   */

  regval = imxrt_getreg(&HCOR->portsc[rhpndx]);
  if ((regval & EHCI_PORTSC_LSTATUS_MASK) == EHCI_PORTSC_LSTATUS_KSTATE)
    {
      /* EHCI Paragraph 2.3.9:
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
       * EHCI Paragraph 4.2:
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

      hport->speed = USB_SPEED_LOW;
    }
  else
    {
      /* Assume full-speed for now */

      hport->speed = USB_SPEED_FULL;
    }

  /* Put the root hub port in reset.
   *
   * EHCI Paragraph 2.3.9:
   *
   *  "The HCHalted bit in the USBSTS register should be a zero before
   *   software attempts to use [the Port Reset] bit. The host controller
   *   may hold Port Reset asserted to a one when the HCHalted bit is a one.
   */

  DEBUGASSERT((imxrt_getreg(&HCOR->usbsts) & EHCI_USBSTS_HALTED) == 0);

  /* EHCI paragraph 2.3.9:
   *
   *  "When software writes a one to [the Port Reset] bit (from a zero), the
   *   bus reset sequence as defined in the USB Specification Revision 2.0
   *   is started.  Software writes a zero to this bit to terminate the bus
   *   reset sequence.  Software must keep this bit at a one long enough to
   *   ensure the reset sequence, as specified in the USB Specification
   *   Revision 2.0, completes. Note: when software writes this bit to a
   *   one, it must also write a zero to the Port Enable bit."
   */

  regaddr = &HCOR->portsc[RHPNDX(rhport)];
  regval  = imxrt_getreg(regaddr);
  regval &= ~EHCI_PORTSC_PE;
  regval |= EHCI_PORTSC_RESET;
  imxrt_putreg(regval, regaddr);

  /* USB 2.0 "Root hubs must provide an aggregate reset period of at least
   * 50 ms."
   */

  nxsig_usleep(50 * 1000);

  regval  = imxrt_getreg(regaddr);
  regval &= ~EHCI_PORTSC_RESET;
  imxrt_putreg(regval, regaddr);

  /* Wait for the port reset to complete
   *
   * EHCI Paragraph 2.3.9:
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

  while ((imxrt_getreg(regaddr) & EHCI_PORTSC_RESET) != 0);
  nxsig_usleep(200 * 1000);

  /* EHCI Paragraph 4.2.2:
   *
   *  "... The reset process is actually complete when software reads a zero
   *   in the PortReset bit. The EHCI Driver checks the PortEnable bit in
   *   the PORTSC register. If set to a one, the connected device is a high-
   *   speed device and EHCI Driver (root hub emulator) issues a change
   *   report to the hub driver and the hub driver continues to enumerate
   *   the attached device."
   *
   *  "At the time the EHCI Driver receives the port reset and enable request
   *   the LineStatus bits might indicate a low-speed device. Additionally,
   *   when the port reset process is complete, the PortEnable field may
   *   indicate that a full-speed device is attached. In either case the EHCI
   *   driver sets the PortOwner bit in the PORTSC register to a one to
   *   release port ownership to a companion host controller."
   *
   * LPC31xx User Manual Paragraph 6.1.3:
   *
   *  "In a standard EHCI controller design, the EHCI host controller driver
   *   detects a Full speed (FS) or Low speed (LS) device by noting if the
   *   port enable bit is set after the port reset operation. The port enable
   *   will only be set in a standard EHCI controller implementation after
   *   the port reset operation and when the host and device negotiate a
   *   High-Speed connection (i.e. Chirp completes successfully). Since this
   *   controller has an embedded Transaction Translator, the port enable
   *   will always be set after the port reset operation regardless of the
   *   result of the host device chirp result and the resulting port speed
   *   will be indicated by the PSPD field in PORTSC1.
   */

  regval = imxrt_getreg(&HCOR->portsc[rhpndx]);

  if ((regval & USBDEV_PRTSC1_PSPD_MASK) == USBDEV_PRTSC1_PSPD_HS)
    {
      /* High speed device */

      hport->speed = USB_SPEED_HIGH;
    }
  else if ((regval & USBDEV_PRTSC1_PSPD_MASK) == USBDEV_PRTSC1_PSPD_FS)
    {
      /* Low- or Full- speed device.  Set the port ownership bit.
       *
       * EHCI Paragraph 4.2:
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

      DEBUGASSERT(hport->speed == USB_SPEED_FULL);
    }

  /* Otherwise it must be a low speed device */

  else
    {
      DEBUGASSERT(hport->speed == USB_SPEED_LOW);
      DEBUGASSERT((regval & USBDEV_PRTSC1_PSPD_MASK) ==
                  USBDEV_PRTSC1_PSPD_LS);
    }

  return OK;
}

static int imxrt_enumerate(struct usbhost_connection_s *conn,
                           struct usbhost_hubport_s *hport)
{
  int ret;

  /* If this is a connection on the root hub, then we need to go to
   * little more effort to get the device speed.  If it is a connection
   * on an external hub, then we already have that information.
   */

  DEBUGASSERT(hport);
#ifdef CONFIG_USBHOST_HUB
  if (ROOTHUB(hport))
#endif
    {
      ret = imxrt_rh_enumerate(conn, hport);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Then let the common usbhost_enumerate do the real enumeration. */

  usbhost_vtrace1(EHCI_VTRACE1_CLASSENUM, hport->port);
  ret = usbhost_enumerate(hport, &hport->devclass);
  if (ret < 0)
    {
      /* Failed to enumerate */

      usbhost_trace2(EHCI_TRACE2_CLASSENUM_FAILED, hport->port + 1, -ret);

      /* If this is a root hub port, then marking the hub port not connected
       * will cause imxrt_wait() to return and we will try the connection
       * again.
       */

      hport->connected = false;
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_ep0configure
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

static int imxrt_ep0configure(struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep0, uint8_t funcaddr,
                              uint8_t speed, uint16_t maxpacketsize)
{
  struct imxrt_epinfo_s *epinfo = (struct imxrt_epinfo_s *)ep0;
  int ret;

  DEBUGASSERT(drvr != NULL && epinfo != NULL && maxpacketsize < 2048);

  /* We must have exclusive access to the EHCI data structures. */

  ret = nxmutex_lock(&g_ehci.lock);
  if (ret >= 0)
    {
      /* Remember the new device address and max packet size */

      epinfo->devaddr   = funcaddr;
      epinfo->speed     = speed;
      epinfo->maxpacket = maxpacketsize;

      nxmutex_unlock(&g_ehci.lock);
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_epalloc
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

static int imxrt_epalloc(struct usbhost_driver_s *drvr,
                         const struct usbhost_epdesc_s *epdesc,
                         usbhost_ep_t *ep)
{
  struct imxrt_epinfo_s *epinfo;
  struct usbhost_hubport_s *hport;

  /* Sanity check.  NOTE that this method should only be called if a device
   * is connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(drvr != 0 && epdesc != NULL && epdesc->hport != NULL &&
              ep != NULL);
  hport = epdesc->hport;

  /* Terse output only if we are tracing */

#ifdef CONFIG_USBHOST_TRACE
  usbhost_vtrace2(EHCI_VTRACE2_EPALLOC, epdesc->addr, epdesc->xfrtype);
#else
  uinfo("EP%d DIR=%s FA=%08x TYPE=%d Interval=%d MaxPacket=%d\n",
        epdesc->addr, epdesc->in ? "IN" : "OUT", hport->funcaddr,
        epdesc->xfrtype, epdesc->interval, epdesc->mxpacketsize);
#endif

  /* Allocate a endpoint information structure */

  epinfo = (struct imxrt_epinfo_s *)
    kmm_zalloc(sizeof(struct imxrt_epinfo_s));
  if (!epinfo)
    {
      usbhost_trace1(EHCI_TRACE1_EPALLOC_FAILED, 0);
      return -ENOMEM;
    }

  /* Initialize the endpoint container (which is really just another form of
   * 'struct usbhost_epdesc_s', packed differently and with additional
   * information.  A cleaner design might just embed struct usbhost_epdesc_s
   * inside of struct imxrt_epinfo_s and just memcpy() here.
   */

  epinfo->epno      = epdesc->addr;
  epinfo->dirin     = epdesc->in;
  epinfo->devaddr   = hport->funcaddr;
#ifndef CONFIG_USBHOST_INT_DISABLE
  epinfo->interval  = epdesc->interval;
#endif
  epinfo->maxpacket = epdesc->mxpacketsize;
  epinfo->xfrtype   = epdesc->xfrtype;
  epinfo->speed     = hport->speed;

  nxsem_init(&epinfo->iocsem, 0, 0);

  /* Success.. return an opaque reference to the endpoint information
   * structure instance
   */

  *ep = (usbhost_ep_t)epinfo;
  return OK;
}

/****************************************************************************
 * Name: imxrt_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *           call to the class create() method.
 *   ep   - The endpint to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int imxrt_epfree(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct imxrt_epinfo_s *epinfo = (struct imxrt_epinfo_s *)ep;

  /* There should not be any pending, transfers */

  DEBUGASSERT(drvr && epinfo && epinfo->iocwait == 0);

  /* Free the container */

  kmm_free(epinfo);
  return OK;
}

/****************************************************************************
 * Name: imxrt_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to allocate the request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_malloc().
 *
 *   This interface was optimized under a particular assumption.  It was
 *   assumed that the driver maintains a pool of small, pre-allocated buffers
 *   for descriptor traffic.  NOTE that size is not an input, but an output:
 *   The size of the pre-allocated buffer is returned.
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
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int imxrt_alloc(struct usbhost_driver_s *drvr,
                       uint8_t **buffer, size_t *maxlen)
{
  int ret = -ENOMEM;
  DEBUGASSERT(drvr && buffer && maxlen);

  /* The only special requirements for transfer/descriptor buffers are that
   * (1) they be aligned to a cache line boundary and (2) they are a
   * multiple of the cache line size in length.
   */

  *buffer = (uint8_t *)kmm_memalign(ARMV7M_DCACHE_LINESIZE,
                                    IMXRT_EHCI_BUFSIZE);
  if (*buffer)
    {
      *maxlen = IMXRT_EHCI_BUFSIZE;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: imxrt_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor
 *   data can be accessed more efficiently.  This method provides a
 *   mechanism to free that request/descriptor memory.  If the underlying
 *   hardware does not support such "special" memory, this functions may
 *   simply map to kmm_free().
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int imxrt_free(struct usbhost_driver_s *drvr, uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

  /* No special action is require to free the transfer/descriptor buffer
   * memory
   */

  kmm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: imxrt_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to
 *   allocate the request/descriptor memory.  If the underlying hardware
 *   does not support such "special" memory, this functions may simply map
 *   to kumm_malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are variable-
 *   sized.
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of a memory location provided by the caller in
 *            which to return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int imxrt_ioalloc(struct usbhost_driver_s *drvr,
                         uint8_t **buffer, size_t buflen)
{
  DEBUGASSERT(drvr && buffer && buflen > 0);

  /* The only special requirements for I/O buffers are that (1) they be
   * aligned to a cache line boundary, (2) they are a multiple of the cache
   * line size in length, and (3) they might need to be user accessible
   * (depending on how the class driver implements its buffering).
   */

  buflen  = (buflen + DCACHE_LINEMASK) & ~DCACHE_LINEMASK;
  *buffer = (uint8_t *)kumm_memalign(ARMV7M_DCACHE_LINESIZE, buflen);
  return *buffer ? OK : -ENOMEM;
}

/****************************************************************************
 * Name: imxrt_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed
 *   more efficiently.  This method provides a mechanism to free that IO
 *   buffer memory.  If the underlying hardware does not support such
 *   "special" memory, this functions may simply map to kumm_free().
 *
 * Input Parameters:
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ****************************************************************************/

static int imxrt_iofree(struct usbhost_driver_s *drvr,
                        uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);

  /* No special action is require to free the I/O buffer memory */

  kumm_free(buffer);
  return OK;
}

/****************************************************************************
 * Name: imxrt_ctrlin and imxrt_ctrlout
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
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   ep0    - The control endpoint to send/receive the control request.
 *   req    - Describes the request to be sent.  This request must lie in
 *            memory created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *            responses.  This buffer must be large enough to hold the
 *            length value in the request description. buffer must have been
 *            allocated using DRVR_ALLOC.
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same
 *   allocated memory.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

static int imxrt_ctrlin(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                        const struct usb_ctrlreq_s *req,
                        uint8_t *buffer)
{
  struct imxrt_rhport_s *rhport = (struct imxrt_rhport_s *)drvr;
  struct imxrt_epinfo_s *ep0info = (struct imxrt_epinfo_s *)ep0;
  uint16_t len;
  ssize_t nbytes;
  int ret;

  DEBUGASSERT(rhport != NULL && ep0info != NULL && req != NULL);

  len = imxrt_read16(req->len);

  /* Terse output only if we are tracing */

#ifdef CONFIG_USBHOST_TRACE
  usbhost_vtrace2(EHCI_VTRACE2_CTRLINOUT, RHPORT(rhport), req->req);
#else
  uinfo("RHPort%d type: %02x req: %02x value: %02x%02x index: %02x%02x "
        "len: %04x\n",
        RHPORT(rhport), req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], len);
#endif

  /* We must have exclusive access to the EHCI hardware and data
   * structures.
   */

  ret = nxmutex_lock(&g_ehci.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the request for the IOC event well BEFORE initiating the transfer. */

  ret = imxrt_ioc_setup(rhport, ep0info);
  if (ret != OK)
    {
      usbhost_trace1(EHCI_TRACE1_DEVDISCONNECTED, -ret);
      goto errout_with_lock;
    }

  /* Now initiate the transfer */

  ret = imxrt_async_setup(rhport, ep0info, req, buffer, len);
  if (ret < 0)
    {
      uerr("ERROR: imxrt_async_setup failed: %d\n", ret);
      goto errout_with_iocwait;
    }

  /* And wait for the transfer to complete */

  nbytes = imxrt_transfer_wait(ep0info);
  nxmutex_unlock(&g_ehci.lock);
  return nbytes >= 0 ? OK : (int)nbytes;

errout_with_iocwait:
  ep0info->iocwait = false;
errout_with_lock:
  nxmutex_unlock(&g_ehci.lock);
  return ret;
}

static int imxrt_ctrlout(struct usbhost_driver_s *drvr, usbhost_ep_t ep0,
                         const struct usb_ctrlreq_s *req,
                         const uint8_t *buffer)
{
  /* imxrt_ctrlin can handle both directions.  We just need to work around
   * the differences in the function signatures.
   */

  return imxrt_ctrlin(drvr, ep0, req, (uint8_t *)buffer);
}

/****************************************************************************
 * Name: imxrt_transfer
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
 *   drvr   - The USB host driver instance obtained as a parameter from the
 *            call to the class create() method.
 *   ep     - The IN or OUT endpoint descriptor for the device endpoint on
 *            which to perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or
 *            received (IN endpoint).  buffer must have been allocated using
 *            DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Value:
 *   On success, a non-negative value is returned that indicates the number
 *   of bytes successfully transferred.  On a failure, a negated errno value
 *   is returned that indicates the nature of the failure:
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

static ssize_t imxrt_transfer(struct usbhost_driver_s *drvr,
                              usbhost_ep_t ep, uint8_t *buffer,
                              size_t buflen)
{
  struct imxrt_rhport_s *rhport = (struct imxrt_rhport_s *)drvr;
  struct imxrt_epinfo_s *epinfo = (struct imxrt_epinfo_s *)ep;
  ssize_t nbytes;
  int ret;

  DEBUGASSERT(rhport && epinfo && buffer && buflen > 0);

  /* We must have exclusive access to the EHCI hardware and data
   * structures.
   */

  ret = nxmutex_lock(&g_ehci.lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Set the request for the IOC event well BEFORE initiating the
   * transfer.
   */

  ret = imxrt_ioc_setup(rhport, epinfo);
  if (ret != OK)
    {
      usbhost_trace1(EHCI_TRACE1_DEVDISCONNECTED, -ret);
      goto errout_with_lock;
    }

  /* Initiate the transfer */

  switch (epinfo->xfrtype)
    {
      case USB_EP_ATTR_XFER_BULK:
        ret = imxrt_async_setup(rhport, epinfo, NULL, buffer, buflen);
        break;

#ifndef CONFIG_USBHOST_INT_DISABLE
      case USB_EP_ATTR_XFER_INT:
        ret = imxrt_intr_setup(rhport, epinfo, buffer, buflen);
        break;
#endif

#ifndef CONFIG_USBHOST_ISOC_DISABLE
      case USB_EP_ATTR_XFER_ISOC:
# warning "Isochronous endpoint support not emplemented"
#endif
      case USB_EP_ATTR_XFER_CONTROL:
      default:
        usbhost_trace1(EHCI_TRACE1_BADXFRTYPE, epinfo->xfrtype);
        ret = -ENOSYS;
        break;
    }

  /* Check for errors in the setup of the transfer */

  if (ret < 0)
    {
      goto errout_with_iocwait;
    }

  /* Then wait for the transfer to complete */

  nbytes = imxrt_transfer_wait(epinfo);
  nxmutex_unlock(&g_ehci.lock);
  return nbytes;

errout_with_iocwait:
  epinfo->iocwait = false;
errout_with_lock:
  uerr("!!!\n");
  nxmutex_unlock(&g_ehci.lock);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: imxrt_asynch
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
 *   drvr     - The USB host driver instance obtained as a parameter from
 *              the call to the class create() method.
 *   ep       - The IN or OUT endpoint descriptor for the device endpoint on
 *              which to perform the transfer.
 *   buffer   - A buffer containing the data to be sent (OUT endpoint) or
 *              received (IN endpoint).  buffer must have been allocated
 *              using DRVR_ALLOC
 *   buflen   - The length of the data to be sent or received.
 *   callback - This function will be called when the transfer completes.
 *   arg      - The arbitrary parameter that will be passed to the callback
 *              function when the transfer completes.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_ASYNCH
static int imxrt_asynch(struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                        uint8_t *buffer, size_t buflen,
                        usbhost_asynch_t callback, void *arg)
{
  struct imxrt_rhport_s *rhport = (struct imxrt_rhport_s *)drvr;
  struct imxrt_epinfo_s *epinfo = (struct imxrt_epinfo_s *)ep;
  int ret;

  DEBUGASSERT(rhport && epinfo && buffer && buflen > 0);

  /* We must have exclusive access to the EHCI hardware and data
   * structures.
   */

  ret = nxmutex_lock(&g_ehci.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the request for the callback well BEFORE initiating the transfer. */

  ret = imxrt_ioc_async_setup(rhport, epinfo, callback, arg);
  if (ret != OK)
    {
      usbhost_trace1(EHCI_TRACE1_DEVDISCONNECTED, -ret);
      goto errout_with_lock;
    }

  /* Initiate the transfer */

  switch (epinfo->xfrtype)
    {
      case USB_EP_ATTR_XFER_BULK:
        ret = imxrt_async_setup(rhport, epinfo, NULL, buffer, buflen);
        break;

#ifndef CONFIG_USBHOST_INT_DISABLE
      case USB_EP_ATTR_XFER_INT:
        ret = imxrt_intr_setup(rhport, epinfo, buffer, buflen);
        break;
#endif

#ifndef CONFIG_USBHOST_ISOC_DISABLE
      case USB_EP_ATTR_XFER_ISOC:
# warning "Isochronous endpoint support not emplemented"
#endif
      case USB_EP_ATTR_XFER_CONTROL:
      default:
        usbhost_trace1(EHCI_TRACE1_BADXFRTYPE, epinfo->xfrtype);
        ret = -ENOSYS;
        break;
    }

  /* Check for errors in the setup of the transfer */

  if (ret < 0)
    {
      goto errout_with_callback;
    }

  /* The transfer is in progress */

  nxmutex_unlock(&g_ehci.lock);
  return OK;

errout_with_callback:
  epinfo->callback = NULL;
  epinfo->arg      = NULL;
errout_with_lock:
  nxmutex_unlock(&g_ehci.lock);
  return ret;
}
#endif /* CONFIG_USBHOST_ASYNCH */

/****************************************************************************
 * Name: imxrt_cancel
 *
 * Description:
 *   Cancel a pending transfer on an endpoint.  Canceled synchronous or
 *   asynchronous transfer will complete normally with the error -ESHUTDOWN.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the
 *          call to the class create() method.
 *   ep   - The IN or OUT endpoint descriptor for the device endpoint on
 *          which an asynchronous transfer should be transferred.
 *
 * Returned Value:
 *   On success, zero (OK) is returned. On a failure, a negated errno value
 *   is returned indicating the nature of the failure
 *
 ****************************************************************************/

static int imxrt_cancel(struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct imxrt_epinfo_s *epinfo = (struct imxrt_epinfo_s *)ep;
  struct imxrt_qh_s *qh;
#ifdef CONFIG_USBHOST_ASYNCH
  usbhost_asynch_t callback;
  void *arg;
#endif
  uint32_t *bp;
  irqstate_t flags;
  bool iocwait;
  int ret;

  DEBUGASSERT(epinfo);

  /* We must have exclusive access to the EHCI hardware and data structures.
   * This will prevent servicing any transfer completion events while we
   * perform the the cancellation, but will not prevent DMA-related race
   * conditions.
   *
   * REVISIT: This won't work.  This function must be callable from the
   * interrupt level.
   */

  ret = nxmutex_lock(&g_ehci.lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Sample and reset all transfer termination information.  This will
   * prevent any callbacks from occurring while are performing the
   * cancellation.  The transfer may still be in progress, however, so this
   * does not eliminate other DMA-related race conditions.
   */

  flags = enter_critical_section();
#ifdef CONFIG_USBHOST_ASYNCH
  callback         = epinfo->callback;
  arg              = epinfo->arg;
#endif
  iocwait          = epinfo->iocwait;

#ifdef CONFIG_USBHOST_ASYNCH
  epinfo->callback = NULL;
  epinfo->arg      = NULL;
#endif
  epinfo->iocwait  = false;

  /* This will prevent any callbacks from occurring while are performing
   * the cancellation.  The transfer may still be in progress, however, so
   * this does not eliminate other DMA-related race conditions.
   */

  epinfo->callback = NULL;
  epinfo->arg      = NULL;
  leave_critical_section(flags);

  /* Bail if there is no transfer in progress for this endpoint */

#ifdef CONFIG_USBHOST_ASYNCH
  if (callback == NULL && !iocwait)
#else
  if (!iocwait)
#endif
    {
      ret = OK;
      goto errout_with_lock;
    }

  /* Handle the cancellation according to the type of the transfer */

  switch (epinfo->xfrtype)
    {
      case USB_EP_ATTR_XFER_CONTROL:
      case USB_EP_ATTR_XFER_BULK:
        {
          /* Get the horizontal pointer from the head of the asynchronous
           * queue.
           */

          bp = (uint32_t *)&g_asynchead.hw.hlp;
          qh = (struct imxrt_qh_s *)
               imxrt_virtramaddr(imxrt_swap32(*bp) & QH_HLP_MASK);

          /* If the asynchronous queue is empty, then the forward point in
           * the asynchronous queue head will point back to the queue
           * head.
           */

          if (qh && qh != &g_asynchead)
            {
              /* Claim that we successfully cancelled the transfer */

              ret = OK;
              goto exit_terminate;
            }
        }
        break;

#ifndef CONFIG_USBHOST_INT_DISABLE
      case USB_EP_ATTR_XFER_INT:
        {
          /* Get the horizontal pointer from the head of the interrupt
           * queue.
           */

          bp = (uint32_t *)&g_intrhead.hw.hlp;
          qh = (struct imxrt_qh_s *)
               imxrt_virtramaddr(imxrt_swap32(*bp) & QH_HLP_MASK);
          if (qh)
            {
              /* if the queue is empty, then just claim that we successfully
               * canceled the transfer.
               */

              ret = OK;
              goto exit_terminate;
            }
        }
        break;
#endif

#ifndef CONFIG_USBHOST_ISOC_DISABLE
      case USB_EP_ATTR_XFER_ISOC:
# warning "Isochronous endpoint support not emplemented"
#endif
      default:
        usbhost_trace1(EHCI_TRACE1_BADXFRTYPE, epinfo->xfrtype);
        ret = -ENOSYS;
        goto errout_with_lock;
    }

  /* Find and remove the QH.  There are four possibilities:
   *
   * 1)  The transfer has already completed and the QH is no longer in the
   *     list.  In this case, sam_hq_foreach will return zero
   * 2a) The transfer is not active and still pending.  It was removed from
   *     the list and sam_hq_foreach will return one.
   * 2b) The is active but not yet complete.  This is currently handled the
   *     same as 2a).  REVISIT: This needs to be fixed.
   * 3)  Some bad happened and sam_hq_foreach returned an error code < 0.
   */

  ret = imxrt_qh_foreach(qh, &bp, imxrt_qh_cancel, epinfo);
  if (ret < 0)
    {
      usbhost_trace1(EHCI_TRACE1_QTDFOREACH_FAILED, -ret);
    }

  /* Was there a pending synchronous transfer? */

exit_terminate:
  epinfo->result = -ESHUTDOWN;
#ifdef CONFIG_USBHOST_ASYNCH
  if (iocwait)
    {
      /* Yes... wake it up */

      DEBUGASSERT(callback == NULL);
      nxsem_post(&epinfo->iocsem);
    }

  /* No.. Is there a pending asynchronous transfer? */

  else /* if (callback != NULL) */
    {
      /* Yes.. perform the callback */

      callback(arg, -ESHUTDOWN);
    }

#else
  /* Wake up the waiting thread */

  nxsem_post(&epinfo->iocsem);
#endif

errout_with_lock:
  nxmutex_unlock(&g_ehci.lock);
  return ret;
}

/****************************************************************************
 * Name: imxrt_connect
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
 *   is returned indicating the nature of the failure
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST_HUB
static int imxrt_connect(struct usbhost_driver_s *drvr,
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
  DEBUGASSERT(g_ehci.hport == NULL); /* REVISIT */

  g_ehci.hport = hport;
  if (g_ehci.pscwait)
    {
      g_ehci.pscwait = false;
      nxsem_post(&g_ehci.pscsem);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: imxrt_disconnect
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

static void imxrt_disconnect(struct usbhost_driver_s *drvr,
                             struct usbhost_hubport_s *hport)
{
  DEBUGASSERT(hport != NULL);
  hport->devclass = NULL;
}

/****************************************************************************
 * Name: imxrt_reset
 *
 * Description:
 *   Set the HCRESET bit in the USBCMD register to reset the EHCI hardware.
 *
 *   Table 2-9. USBCMD - USB Command Register Bit Definitions
 *
 *    "Host Controller Reset (HCRESET) ... This control bit is used by
 *     software to reset the host controller. The effects of this on Root
 *     Hub registers are similar to a Chip Hardware Reset.
 *
 *    "When software writes a one to this bit, the Host Controller resets its
 *     internal pipelines, timers, counters, state machines, etc. to their
 *     initial value. Any transaction currently in progress on USB is
 *     immediately terminated. A USB reset is not driven on downstream
 *     ports.
 *
 *    "PCI Configuration registers are not affected by this reset. All
 *     operational registers, including port registers and port state
 *     machines are set to their initial values. Port ownership reverts
 *     to the companion host controller(s)... Software must reinitialize
 *     the host controller ... in order to return the host controller to
 *     an operational state.
 *
 *    "This bit is set to zero by the Host Controller when the reset process
 *     is complete. Software cannot terminate the reset process early by
 *     writing a zero to this register. Software should not set this bit to
 *     a one when the HCHalted bit in the USBSTS register is a zero.
 *     Attempting to reset an actively running host controller will result
 *     in undefined behavior."
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   on failure.
 *
 * Assumptions:
 * - Called during the initialization of the EHCI.
 *
 ****************************************************************************/

static int imxrt_reset(void)
{
  uint32_t regval;
  unsigned int timeout;

  /* Make sure that the EHCI is halted:  "When [the Run/Stop] bit is set to
   * 0, the Host Controller completes the current transaction on the USB and
   * then halts. The HC Halted bit in the status register indicates when the
   * Host Controller has finished the transaction and has entered the
   * stopped state..."
   */

  imxrt_putreg(0, &HCOR->usbcmd);

  /* "... Software should not set [HCRESET] to a one when the HCHalted bit in
   *  the USBSTS register is a zero. Attempting to reset an actively running
   *  host controller will result in undefined behavior."
   */

  timeout = 0;
  do
    {
      /* Wait one microsecond and update the timeout counter */

      up_udelay(1);
      timeout++;

      /* Get the current value of the USBSTS register.  This loop will
       * terminate when either the timeout exceeds one millisecond or when
       * the HCHalted bit is no longer set in the USBSTS register.
       */

      regval = imxrt_getreg(&HCOR->usbsts);
    }
  while (((regval & EHCI_USBSTS_HALTED) == 0) && (timeout < 1000));

  /* Is the EHCI still running?  Did we timeout? */

  if ((regval & EHCI_USBSTS_HALTED) == 0)
    {
      usbhost_trace1(EHCI_TRACE1_HCHALTED_TIMEOUT, regval);
      return -ETIMEDOUT;
    }

  /* Now we can set the HCReset bit in the USBCMD register to initiate the
   * reset
   */

  regval  = imxrt_getreg(&HCOR->usbcmd);
  regval |= EHCI_USBCMD_HCRESET;
  imxrt_putreg(regval, &HCOR->usbcmd);

  /* Wait for the HCReset bit to become clear */

  do
    {
      /* Wait five microsecondw and update the timeout counter */

      up_udelay(5);
      timeout += 5;

      /* Get the current value of the USBCMD register.  This loop will
       * terminate when either the timeout exceeds one second or when the
       * HCReset bit is no longer set in the USBSTS register.
       */

      regval = imxrt_getreg(&HCOR->usbcmd);
    }
  while (((regval & EHCI_USBCMD_HCRESET) != 0) && (timeout < 1000000));

  /* Return either success or a timeout */

  return (regval & EHCI_USBCMD_HCRESET) != 0 ? -ETIMEDOUT : OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_ehci_initialize
 *
 * Description:
 *   Initialize USB EHCI host controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than one EHCI interface, then
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

struct usbhost_connection_s *imxrt_ehci_initialize(int controller)
{
  struct usbhost_hubport_s *hport;
  uint32_t regval;
#  if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_INFO)
  uint16_t regval16;
  unsigned int nports;
#  endif
  uintptr_t physaddr;
  int ret;
  int i;

  /* Sanity checks */

  DEBUGASSERT(controller == 0);
  DEBUGASSERT(((uintptr_t) & g_asynchead & 0x1f) == 0);
  DEBUGASSERT((sizeof(struct imxrt_qh_s) & 0x1f) == 0);
  DEBUGASSERT((sizeof(struct imxrt_qtd_s) & 0x1f) == 0);

#  ifdef CONFIG_IMXRT_EHCI_PREALLOCATE
  DEBUGASSERT(((uintptr_t) & g_qhpool & 0x1f) == 0);
  DEBUGASSERT(((uintptr_t) & g_qtdpool & 0x1f) == 0);
#  endif

#  ifndef CONFIG_USBHOST_INT_DISABLE
  DEBUGASSERT(((uintptr_t) & g_intrhead & 0x1f) == 0);
#    ifdef CONFIG_IMXRT_EHCI_PREALLOCATE
  DEBUGASSERT(((uintptr_t) g_framelist & 0xfff) == 0);
#    endif
#  endif                               /* CONFIG_USBHOST_INT_DISABLE */

  /* Software Configuration *************************************************/

  usbhost_vtrace1(EHCI_VTRACE1_INITIALIZING, 0);

  /* Initialize the root hub port structures */

  for (i = 0; i < IMXRT_EHCI_NRHPORT; i++)
    {
      struct imxrt_rhport_s *rhport = &g_ehci.rhport[i];

      /* Initialize the device operations */

      rhport->drvr.ep0configure = imxrt_ep0configure;
      rhport->drvr.epalloc = imxrt_epalloc;
      rhport->drvr.epfree = imxrt_epfree;
      rhport->drvr.alloc = imxrt_alloc;
      rhport->drvr.free = imxrt_free;
      rhport->drvr.ioalloc = imxrt_ioalloc;
      rhport->drvr.iofree = imxrt_iofree;
      rhport->drvr.ctrlin = imxrt_ctrlin;
      rhport->drvr.ctrlout = imxrt_ctrlout;
      rhport->drvr.transfer = imxrt_transfer;
#  ifdef CONFIG_USBHOST_ASYNCH
      rhport->drvr.asynch = imxrt_asynch;
#  endif
      rhport->drvr.cancel = imxrt_cancel;
#  ifdef CONFIG_USBHOST_HUB
      rhport->drvr.connect = imxrt_connect;
#  endif
      rhport->drvr.disconnect = imxrt_disconnect;

      /* Initialize EP0 */

      rhport->ep0.xfrtype = USB_EP_ATTR_XFER_CONTROL;
      rhport->ep0.speed = USB_SPEED_FULL;
      rhport->ep0.maxpacket = 8;
      nxsem_init(&rhport->ep0.iocsem, 0, 0);

      /* Initialize the public port representation */

      hport = &rhport->hport.hport;
      hport->drvr = &rhport->drvr;
#  ifdef CONFIG_USBHOST_HUB
      hport->parent = NULL;
#  endif
      hport->ep0 = &rhport->ep0;
      hport->port = i;
      hport->speed = USB_SPEED_FULL;

      /* Initialize function address generation logic */

      usbhost_devaddr_initialize(&rhport->hport);
    }

#  ifndef CONFIG_IMXRT_EHCI_PREALLOCATE
  /* Allocate a pool of free Queue Head (QH) structures */

  g_qhpool =
    (struct imxrt_qh_s *)kmm_memalign(32,
                                      CONFIG_IMXRT_EHCI_NQHS *
                                      sizeof(struct imxrt_qh_s));
  if (!g_qhpool)
    {
      usbhost_trace1(EHCI_TRACE1_QHPOOLALLOC_FAILED, 0);
      return NULL;
    }
#  endif

  /* Initialize the list of free Queue Head (QH) structures */

  for (i = 0; i < CONFIG_IMXRT_EHCI_NQHS; i++)
    {
      /* Put the QH structure in a free list */

      imxrt_qh_free(&g_qhpool[i]);
    }

#  ifndef CONFIG_IMXRT_EHCI_PREALLOCATE
  /* Allocate a pool of free Transfer Descriptor (qTD) structures */

  g_qtdpool =
    (struct imxrt_qtd_s *)kmm_memalign(32,
                                       CONFIG_IMXRT_EHCI_NQTDS *
                                       sizeof(struct imxrt_qtd_s));
  if (!g_qtdpool)
    {
      usbhost_trace1(EHCI_TRACE1_QTDPOOLALLOC_FAILED, 0);
      kmm_free(g_qhpool);
      return NULL;
    }
#  endif

#  if !defined(CONFIG_IMXRT_EHCI_PREALLOCATE) && !defined(CONFIG_USBHOST_INT_DISABLE)
  /* Allocate the periodic framelist */

  g_framelist = (uint32_t *)
    kmm_memalign(4096, FRAME_LIST_SIZE * sizeof(uint32_t));
  if (!g_framelist)
    {
      usbhost_trace1(EHCI_TRACE1_PERFLALLOC_FAILED, 0);
      kmm_free(g_qhpool);
      kmm_free(g_qtdpool);
      return NULL;
    }
#  endif

  /* Initialize the list of free Transfer Descriptor (qTD) structures */

  for (i = 0; i < CONFIG_IMXRT_EHCI_NQTDS; i++)
    {
      /* Put the TD in a free list */

      imxrt_qtd_free(&g_qtdpool[i]);
    }

  /* EHCI Hardware Configuration ********************************************/

  imxrt_clockall_usboh3();

  /* Reset the controller from the OTG peripheral */

  putreg32(USBDEV_USBCMD_RST, IMXRT_USBDEV_USBCMD);
  while ((getreg32(IMXRT_USBDEV_USBCMD) & USBDEV_USBCMD_RST) != 0);

  /* Program the controller to be the USB host controller Fixed selections:
   * CM = Host mode ES = 0, Little endian mode.  SLOM Not used in host mode.
   * VBPS = 1, off-chip power source Configurable selections: SDIS = 1,
   * Stream disable mode.  Eliminates overruns/underruns at the expense of
   * some performance.
   */

#  ifdef CONFIG_IMXRT_EHCI_SDIS
  putreg32(USBHOST_USBMODE_CM_HOST | USBHOST_USBMODE_SDIS |
           USBHOST_USBMODE_VBPS, IMXRT_USBDEV_USBMODE);
#  else
  putreg32(USBHOST_USBMODE_CM_HOST | USBHOST_USBMODE_VBPS,
           IMXRT_USBDEV_USBMODE);
#  endif

  /* Reset the EHCI hardware */

  ret = imxrt_reset();
  if (ret < 0)
    {
      usbhost_trace1(EHCI_TRACE1_RESET_FAILED, -ret);
      return NULL;
    }

  /* Re-program the USB host controller.  As implemented, imxrt_reset()
   * requires the host mode setup in order to work.  However, we lose the
   * host configuration in the reset.
   */

#  ifdef CONFIG_IMXRT_EHCI_SDIS
  putreg32(USBHOST_USBMODE_CM_HOST | USBHOST_USBMODE_SDIS |
           USBHOST_USBMODE_VBPS, IMXRT_USBDEV_USBMODE);
#  else
  putreg32(USBHOST_USBMODE_CM_HOST | USBHOST_USBMODE_VBPS,
           IMXRT_USBDEV_USBMODE);
#  endif

  /* Disable all interrupts */

  imxrt_putreg(0, &HCOR->usbintr);

  /* Clear pending interrupts.  Bits in the USBSTS register are cleared by
   * writing a '1' to the corresponding bit.
   */

  imxrt_putreg(EHCI_INT_ALLINTS, &HCOR->usbsts);

#  if defined(CONFIG_DEBUG_USB) && defined(CONFIG_DEBUG_INFO)
  /* Show the EHCI version */

  regval16 = imxrt_swap16(HCCR->hciversion);
  usbhost_vtrace2(EHCI_VTRACE2_HCIVERSION, regval16 >> 8, regval16 & 0xff);

  /* Verify that the correct number of ports is reported */

  regval = imxrt_getreg(&HCCR->hcsparams);
  nports = (regval & EHCI_HCSPARAMS_NPORTS_MASK) >>
            EHCI_HCSPARAMS_NPORTS_SHIFT;

  usbhost_vtrace2(EHCI_VTRACE2_HCSPARAMS, nports, regval);
  DEBUGASSERT(nports == IMXRT_EHCI_NRHPORT);

  /* Show the HCCPARAMS register */

  regval = imxrt_getreg(&HCCR->hccparams);
  usbhost_vtrace1(EHCI_VTRACE1_HCCPARAMS, regval);
#  endif

  /* Initialize the head of the asynchronous queue/reclamation list. "In
   * order to communicate with devices via the asynchronous schedule, system
   * software must write the ASYNDLISTADDR register with the address of a
   * control or bulk queue head. Software must then enable the asynchronous
   * schedule by writing a one to the Asynchronous Schedule Enable bit in
   * the USBCMD register. In order to communicate with devices via the
   * periodic schedule, system software must enable the periodic schedule by
   * writing a one to the Periodic Schedule Enable bit in the USBCMD
   * register. Note that the schedules can be turned on before the first
   * port is reset (and enabled)."
   */

  memset(&g_asynchead, 0, sizeof(struct imxrt_qh_s));
  physaddr = imxrt_physramaddr((uintptr_t) & g_asynchead);
  g_asynchead.hw.hlp = imxrt_swap32(physaddr | QH_HLP_TYP_QH);
  g_asynchead.hw.epchar = imxrt_swap32(QH_EPCHAR_H | QH_EPCHAR_EPS_FULL);
  g_asynchead.hw.overlay.nqp = imxrt_swap32(QH_NQP_T);
  g_asynchead.hw.overlay.alt = imxrt_swap32(QH_NQP_T);
  g_asynchead.hw.overlay.token = imxrt_swap32(QH_TOKEN_HALTED);
  g_asynchead.fqp = imxrt_swap32(QTD_NQP_T);

  /* Set the Current Asynchronous List Address. */

  up_flush_dcache((uintptr_t)&g_asynchead.hw,
    (uintptr_t)&g_asynchead.hw + sizeof(struct ehci_qh_s));

  imxrt_putreg(imxrt_swap32(physaddr), &HCOR->asynclistaddr);

#  ifndef CONFIG_USBHOST_INT_DISABLE

  /* Initialize the head of the periodic list.  Since Isochronous endpoints
   * are not not yet supported, each element of the frame list is initialized
   * to point to the Interrupt Queue Head (g_intrhead).
   */

  memset(&g_intrhead, 0, sizeof(struct imxrt_qh_s));
  g_intrhead.hw.hlp = imxrt_swap32(QH_HLP_T);
  g_intrhead.hw.overlay.nqp = imxrt_swap32(QH_NQP_T);
  g_intrhead.hw.overlay.alt = imxrt_swap32(QH_NQP_T);
  g_intrhead.hw.overlay.token = imxrt_swap32(QH_TOKEN_HALTED);
  g_intrhead.hw.epcaps = imxrt_swap32(QH_EPCAPS_SSMASK(1));

  /* Attach the periodic QH to Period Frame List */

  physaddr = imxrt_physramaddr((uintptr_t) & g_intrhead);
  for (i = 0; i < FRAME_LIST_SIZE; i++)
    {
      g_framelist[i] = imxrt_swap32(physaddr) | PFL_TYP_QH;
    }

  /* Set the Periodic Frame List Base Address. */

  physaddr = imxrt_physramaddr((uintptr_t) g_framelist);
  imxrt_putreg(imxrt_swap32(physaddr), &HCOR->periodiclistbase);
#  endif

  /* Enable the asynchronous schedule and, possibly enable the periodic
   * schedule and set the frame list size.
   */

  regval = imxrt_getreg(&HCOR->usbcmd);
  regval &= ~(EHCI_USBCMD_HCRESET | EHCI_USBCMD_FLSIZE_MASK |
              EHCI_USBCMD_FLSIZE_MASK | EHCI_USBCMD_PSEN |
              EHCI_USBCMD_IAADB | EHCI_USBCMD_LRESET);
  regval |= EHCI_USBCMD_ASEN;

#  ifndef CONFIG_USBHOST_INT_DISABLE
  regval |= EHCI_USBCMD_PSEN;
#    if FRAME_LIST_SIZE == 1024
  regval |= EHCI_USBCMD_FLSIZE_1024;
#    elif FRAME_LIST_SIZE == 512
  regval |= EHCI_USBCMD_FLSIZE_512;
#    elif FRAME_LIST_SIZE == 512
  regval |= EHCI_USBCMD_FLSIZE_256;
#    else
#      error Unsupported frame size list size
#    endif
#  endif

  imxrt_putreg(regval, &HCOR->usbcmd);

  /* Start the host controller by setting the RUN bit in the USBCMD
   * register.
   */

  regval = imxrt_getreg(&HCOR->usbcmd);
  regval |= EHCI_USBCMD_RUN;
  imxrt_putreg(regval, &HCOR->usbcmd);

  /* Route all ports to this host controller by setting the CONFIG flag. */

  regval = imxrt_getreg(&HCOR->configflag);
  regval |= EHCI_CONFIGFLAG;
  imxrt_putreg(regval, &HCOR->configflag);

  /* Wait for the EHCI to run (i.e., no longer report halted) */

  ret = ehci_wait_usbsts(EHCI_USBSTS_HALTED, 0, 100 * 1000);
  if (ret < 0)
    {
      usbhost_trace1(EHCI_TRACE1_RUN_FAILED, imxrt_getreg(&HCOR->usbsts));
      return NULL;
    }

  /* Interrupt Configuration ************************************************/

  ret = irq_attach(IMXRT_IRQ_USBOTG1, imxrt_ehci_interrupt, NULL);
  if (ret != 0)
    {
      usbhost_trace1(EHCI_TRACE1_IRQATTACH_FAILED, IMXRT_IRQ_USBOTG1);
      return NULL;
    }

  /* Enable EHCI interrupts.  Interrupts are still disabled at the level of
   * the interrupt controller.
   */

  imxrt_putreg(EHCI_HANDLED_INTS, &HCOR->usbintr);

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(IMXRT_IRQ_USBOTG1);

  /* Drive Vbus +5V (the smoke test) */

  for (i = 0; i < IMXRT_EHCI_NRHPORT; i++)
    {
      /* Enable VBUS power for the port */

      imxrt_usbhost_vbusdrive(i, true);
      up_mdelay(25);
    }

  /* If there is a USB device in the slot at power up, then we will not get
   * the status change interrupt to signal us that the device is connected.
   * We need to set the initial connected state accordingly.
   */

  for (i = 0; i < IMXRT_EHCI_NRHPORT; i++)
    {
      g_ehci.rhport[i].connected =
        ((imxrt_getreg(&HCOR->portsc[i]) & EHCI_PORTSC_CCS) != 0);
    }

  usbhost_vtrace1(EHCI_VTRACE1_INIITIALIZED, 0);

  /* Initialize and return the connection interface */

  g_ehciconn.wait = imxrt_wait;
  g_ehciconn.enumerate = imxrt_enumerate;
  return &g_ehciconn;
}

/****************************************************************************
 * Name: usbhost_trformat1 and usbhost_trformat2
 *
 * Description:
 *   This interface must be provided by platform specific logic that knows
 *   the HCDs encoding of USB trace data.
 *
 *   Given an 9-bit index, return a format string suitable for use with, say,
 *   printf.  The returned format is expected to handle two unsigned integer
 *   values.
 *
 ****************************************************************************/

#ifdef HAVE_USBHOST_TRACE
const char *usbhost_trformat1(uint16_t id)
{
  int ndx = TRACE1_INDEX(id);

  if (ndx < TRACE1_NSTRINGS)
    {
      return g_trace1[ndx].string;
    }

  return NULL;
}

const char *usbhost_trformat2(uint16_t id)
{
  int ndx = TRACE2_INDEX(id);

  if (ndx < TRACE2_NSTRINGS)
    {
      return g_trace2[ndx].string;
    }

  return NULL;
}
#endif /* HAVE_USBHOST_TRACE */

#endif /* CONFIG_IMXRT_USBOTG && CONFIG_USBHOST */
