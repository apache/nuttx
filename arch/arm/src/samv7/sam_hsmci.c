/****************************************************************************
 * arch/arm/src/samv7/sam_hsmci.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "sam_gpio.h"
#include "sam_xdmac.h"
#include "sam_periphclks.h"
#include "sam_hsmci.h"
#include "hardware/sam_xdmac.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_hsmci.h"
#include "hardware/sam_pinmap.h"

#ifdef CONFIG_SAMV7_HSMCI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SAMV7_HSMCI_DMA
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#else
#  ifndef CONFIG_SAMV7_XDMAC
#    error "CONFIG_SAMV7_HSMCI_DMA support requires CONFIG_SAMV7_XDMAC"
#  endif
#  ifndef CONFIG_SDIO_DMA
#    error CONFIG_SDIO_DMA must be defined with CONFIG_SAMV7_HSMCI_DMA
#  endif
#endif

#ifndef CONFIG_DEBUG_MEMCARD_INFO
#  undef CONFIG_SAMV7_HSMCI_REGDEBUG
#endif

/* System Bus Interfaces */

#if defined(CONFIG_ARCH_CHIP_SAMV71) || defined(CONFIG_ARCH_CHIP_SAME70)
#  define HSMCI_SYSBUS_IF  DMACH_FLAG_PERIPHAHB_AHB_IF1
#  define MEMORY_SYSBUS_IF DMACH_FLAG_MEMAHB_AHB_IF0
#else
#  error Missing bus interface definitions
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifndef CONFIG_SDIO_BLOCKSETUP
#  error "This driver requires CONFIG_SDIO_BLOCKSETUP"
#endif

#if !defined(CONFIG_DEBUG_FS) || !defined(CONFIG_DEBUG_INFO)
#  undef CONFIG_SAMV7_HSMCI_CMDDEBUG
#  undef CONFIG_SAMV7_HSMCI_XFRDEBUG
#endif

#ifdef CONFIG_SAMV7_HSMCI_RDPROOF
#  ifdef CONFIG_SAMV7_HSMCI_WRPROOF
#    define HSMCU_PROOF_BITS (HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF)
#  else
#    define HSMCU_PROOF_BITS HSMCI_MR_RDPROOF
#  endif
#else
#  ifdef CONFIG_SAMV7_HSMCI_WRPROOF
#    define HSMCU_PROOF_BITS HSMCI_MR_WRPROOF
#  else
#    define HSMCU_PROOF_BITS (0)
#  endif
#endif

/* There is some unresolved issue with the SAMV7 DMA.  TX DMA is currently
 * disabled.
 */

#undef  HSCMI_NORXDMA              /* Define to disable RX DMA */
#undef  HSCMI_NOTXDMA              /* Define to disable TX DMA */

/* Timing : 100mS short timeout, 2 seconds for long one */

#define HSMCI_CMDTIMEOUT         MSEC2TICK(100)
#define HSMCI_LONGTIMEOUT        MSEC2TICK(2000)

/* Big DTIMER setting */

#define HSMCI_DTIMER_DATATIMEOUT (0x000fffff)

/* DMA configuration flags */

#define HSMCI_DMA_CHKSIZE HSMCI_DMA_CHKSIZE_1

#define DMA_FLAGS(pid) \
  (DMACH_FLAG_PERIPHPID(pid) | HSMCI_SYSBUS_IF | \
   DMACH_FLAG_PERIPHH2SEL | DMACH_FLAG_PERIPHISPERIPH |  \
   DMACH_FLAG_PERIPHWIDTH_32BITS | DMACH_FLAG_PERIPHCHUNKSIZE_1 | \
   DMACH_FLAG_MEMPID_MAX | MEMORY_SYSBUS_IF | \
   DMACH_FLAG_MEMWIDTH_32BITS | DMACH_FLAG_MEMINCREMENT | \
   DMACH_FLAG_MEMCHUNKSIZE_4 | DMACH_FLAG_MEMBURST_1)

/* Status errors:
 *
 *   HSMCI_INT_UNRE          Data transmit underrun
 *   HSMCI_INT_OVRE          Data receive overrun
 *   HSMCI_INT_BLKOVRE       DMA receive block overrun error
 *   HSMCI_INT_CSTOE         Completion signal time-out error
 *                           (see HSMCI_CSTOR)
 *   HSMCI_INT_DTOE          Data time-out error (see HSMCI_DTOR)
 *   HSMCI_INT_DCRCE         Data CRC Error
 *   HSMCI_INT_RTOE          Response Time-out
 *   HSMCI_INT_RENDE         Response End Bit Error
 *   HSMCI_INT_RCRCE         Response CRC Error
 *   HSMCI_INT_RDIRE         Response Direction Error
 *   HSMCI_INT_RINDE         Response Index Error
 */

#define HSMCI_STATUS_ERRORS \
  (HSMCI_INT_UNRE  | HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | \
   HSMCI_INT_DTOE  | HSMCI_INT_DCRCE | HSMCI_INT_RTOE    | HSMCI_INT_RENDE | \
   HSMCI_INT_RCRCE | HSMCI_INT_RDIRE | HSMCI_INT_RINDE)

/* Response errors:
 *
 *   HSMCI_INT_CSTOE         Completion signal time-out error
 *                           (see HSMCI_CSTOR)
 *   HSMCI_INT_RTOE          Response Time-out
 *   HSMCI_INT_RENDE         Response End Bit Error
 *   HSMCI_INT_RCRCE         Response CRC Error
 *   HSMCI_INT_RDIRE         Response Direction Error
 *   HSMCI_INT_RINDE         Response Index Error
 */

#define HSMCI_RESPONSE_ERRORS \
  (HSMCI_INT_CSTOE | HSMCI_INT_RTOE  | HSMCI_INT_RENDE   | HSMCI_INT_RCRCE | \
   HSMCI_INT_RDIRE | HSMCI_INT_RINDE)
#define HSMCI_RESPONSE_NOCRC_ERRORS \
  (HSMCI_INT_CSTOE | HSMCI_INT_RTOE  | HSMCI_INT_RENDE   | HSMCI_INT_RDIRE | \
   HSMCI_INT_RINDE)
#define HSMCI_RESPONSE_TIMEOUT_ERRORS \
  (HSMCI_INT_CSTOE | HSMCI_INT_RTOE)

/* Data transfer errors:
 *
 *   HSMCI_INT_UNRE          Data transmit underrun
 *   HSMCI_INT_OVRE          Data receive overrun
 *   HSMCI_INT_BLKOVRE       DMA receive block overrun error
 *   HSMCI_INT_CSTOE         Completion signal time-out error
 *                           (see HSMCI_CSTOR)
 *   HSMCI_INT_DTOE          Data time-out error (see HSMCI_DTOR)
 *   HSMCI_INT_DCRCE         Data CRC Error
 */

#define HSMCI_DATA_ERRORS \
  (HSMCI_INT_UNRE  | HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | \
   HSMCI_INT_DTOE  | HSMCI_INT_DCRCE)

#define HSMCI_DATA_TIMEOUT_ERRORS \
  (HSMCI_INT_CSTOE | HSMCI_INT_DTOE)

#define HSMCI_DATA_RECV_ERRORS \
  (HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | HSMCI_INT_DTOE | \
   HSMCI_INT_DCRCE)

#define HSMCI_DATA_SEND_ERRORS \
  (HSMCI_INT_UNRE  | HSMCI_INT_CSTOE | HSMCI_INT_DTOE    | HSMCI_INT_DCRCE)

/* Data transfer status and interrupt mask bits.
 *
 * The XFRDONE flag in the HSMCI_SR indicates exactly when the read or
 * write sequence is finished.
 *
 *   0: A transfer is in progress.
 *   1: Command register is ready to operate and the data bus is in the
 *      idle state.
 *
 * DMADONE: DMA Transfer done
 *
 *   0: DMA buffer transfer has not completed since the last read of
 *      HSMCI_SR register.
 *   1: DMA buffer transfer has completed.
 */

#define HSMCI_RECV_INTS \
  (HSMCI_DATA_RECV_ERRORS | HSMCI_INT_RXRDY)
#define HSMCI_DMARECV_INTS \
  (HSMCI_DATA_RECV_ERRORS | HSMCI_INT_XFRDONE /* | HSMCI_INT_DMADONE */)
#define HSMCI_DMASEND_INTS \
  (HSMCI_DATA_SEND_ERRORS | HSMCI_INT_XFRDONE /* | HSMCI_INT_DMADONE */)

/* Event waiting interrupt mask bits.
 *
 * CMDRDY (Command Ready):
 *
 *   0: A command is in progress
 *   1: The last command has been sent.  The CMDRDY flag is released 8 bits
 *      after the end of the card response. Cleared when writing in the
 *      HSMCI_CMDR
 */

#define HSMCI_CMDRESP_INTS \
  (HSMCI_RESPONSE_ERRORS | HSMCI_INT_CMDRDY)
#define HSMCI_CMDRESP_NOCRC_INTS \
  (HSMCI_RESPONSE_NOCRC_ERRORS | HSMCI_INT_CMDRDY)

/* Register logging support */

#ifdef CONFIG_SAMV7_HSMCI_XFRDEBUG
#  ifdef CONFIG_DEBUG_DMA
#    define SAMPLENDX_BEFORE_SETUP  0
#    define SAMPLENDX_BEFORE_ENABLE 1
#    define SAMPLENDX_AFTER_SETUP   2
#    define SAMPLENDX_END_TRANSFER  3
#    define SAMPLENDX_DMA_CALLBACK  4
#    define SAMPLENDX_TIMEOUT       5
#    define DEBUG_NDMASAMPLES       6
#  else
#    define SAMPLENDX_BEFORE_SETUP  0
#    define SAMPLENDX_AFTER_SETUP   1
#    define SAMPLENDX_END_TRANSFER  2
#    define SAMPLENDX_TIMEOUT       3
#    define DEBUG_NDMASAMPLES       4
#  endif
#endif

#ifdef CONFIG_SAMV7_HSMCI_CMDDEBUG
#  define SAMPLENDX_AFTER_CMDR      0
#  define SAMPLENDX_AT_WAKEUP       1
#  define DEBUG_NCMDSAMPLES         2
#endif

/* Some semi-standard definitions */

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Register logging support */

#if defined(CONFIG_SAMV7_HSMCI_XFRDEBUG) || defined(CONFIG_SAMV7_HSMCI_CMDDEBUG)
struct sam_hsmciregs_s
{
  uint32_t mr;    /* Mode Register */
  uint32_t dtor;  /* Data Timeout Register */
  uint32_t sdcr;  /* SD/SDIO Card Register */
  uint32_t argr;  /* Argument Register */
  uint32_t blkr;  /* Block Register */
  uint32_t cstor; /* Completion Signal Timeout Register */
#if 0 /* Reading these can cause loss of response data */
  uint32_t rsp0;  /* Response Register 0 */
  uint32_t rsp1;  /* Response Register 1 */
  uint32_t rsp2;  /* Response Register 2 */
  uint32_t rsp3;  /* Response Register 3 */
#endif
  uint32_t sr;    /* Status Register */
  uint32_t imr;   /* Interrupt Mask Register */
  uint32_t dma;   /* DMA Configuration Register */
  uint32_t cfg;   /* Configuration Register */
  uint32_t wpmr;  /* Write Protection Mode Register */
  uint32_t wpsr;  /* Write Protection Status Register */
};
#endif

#ifdef CONFIG_SAMV7_HSMCI_XFRDEBUG
struct sam_xfrregs_s
{
  struct sam_hsmciregs_s hsmci;
#ifdef CONFIG_DEBUG_DMA
  struct sam_dmaregs_s  dma;
#endif
};
#endif

/* This structure defines the state of the SAMV7 HSMCI interface */

struct sam_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  /* SAMV7-specific extensions */

  /* Event support */

  sem_t              waitsem;         /* Implements event waiting */
  sdio_eventset_t    waitevents;      /* Set of events to be waited for */
  uint32_t           base;            /* HSMCI register base address */
  uint32_t           waitmask;        /* Interrupt enables for event waiting */
  uint32_t           cmdrmask;        /* Interrupt enables for this
                                       * particular cmd/response */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  struct wdog_s      waitwdog;        /* Watchdog that handles event timeouts */
  uint8_t            hsmci;           /* HSMCI (0, 1, or 2) */
  volatile bool      dmabusy;         /* TRUE: DMA transfer is in progress */
  volatile bool      xfrbusy;         /* TRUE: Transfer is in progress */
  volatile bool      txbusy;          /* TRUE: TX transfer is in progress
                                       * (for delay calculation) */

  /* Callback support */

  sdio_statset_t     cdstatus;        /* Card status */
  sdio_eventset_t    cbevents;        /* Set of events to be cause callbacks */
  worker_t           callback;        /* Registered callback function */
  void              *cbarg;           /* Registered callback argument */
  struct work_s      cbwork;          /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t           xfrmask;         /* Interrupt enables for data transfer */

  /* Interrupt mode data transfer support */

  uint32_t          *buffer;          /* Address of current R/W buffer */
  ssize_t            remaining;       /* Number of bytes remaining in the
                                       * transfer */

  /* DMA data transfer support */

  bool               widebus;         /* Required for DMA support */
  DMA_HANDLE         dma;             /* Handle for DMA channel */

  /* Debug stuff */

#ifdef CONFIG_SAMV7_HSMCI_REGDEBUG
  bool               wrlast;          /* Last was a write */
  uint32_t           addrlast;        /* Last address */
  uint32_t           vallast;         /* Last value */
  int                ntimes;          /* Number of times */
#endif

  /* Register logging support */

#if defined(CONFIG_SAMV7_HSMCI_CMDDEBUG) && defined(CONFIG_SAMV7_HSMCI_XFRDEBUG)
  bool               xfrinitialized;
  bool               cmdinitialized;
#endif
#ifdef CONFIG_SAMV7_HSMCI_XFRDEBUG
  uint8_t            smplset;
  struct sam_xfrregs_s xfrsamples[DEBUG_NDMASAMPLES];
#endif
#ifdef CONFIG_SAMV7_HSMCI_CMDDEBUG
  struct sam_hsmciregs_s cmdsamples[DEBUG_NCMDSAMPLES];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

#ifdef CONFIG_SAMV7_HSMCI_REGDEBUG
static bool sam_checkreg(struct sam_dev_s *priv, bool wr,
              uint32_t value, uint32_t address);
#else
# define    sam_checkreg(priv,wr,value,address) (false)
#endif

static inline uint32_t sam_getreg(struct sam_dev_s *priv,
                                  unsigned int offset);
static inline void sam_putreg(struct sam_dev_s *priv, uint32_t value,
                              unsigned int offset);

static inline void sam_configwaitints(struct sam_dev_s *priv,
                                      uint32_t waitmask,
                                      sdio_eventset_t waitevents);
static void sam_disablewaitints(struct sam_dev_s *priv,
                                sdio_eventset_t wkupevent);
static inline void sam_configxfrints(struct sam_dev_s *priv,
                                     uint32_t xfrmask);
static void sam_disablexfrints(struct sam_dev_s *priv);
static inline void sam_enableints(struct sam_dev_s *priv);

static inline void sam_disable(struct sam_dev_s *priv);
static inline void sam_enable(struct sam_dev_s *priv);

/* Register Sampling ********************************************************/

#if defined(CONFIG_SAMV7_HSMCI_XFRDEBUG) || defined(CONFIG_SAMV7_HSMCI_CMDDEBUG)
static void sam_hsmcisample(struct sam_dev_s *priv,
              struct sam_hsmciregs_s *regs);
static void sam_hsmcidump(struct sam_dev_s *priv,
              struct sam_hsmciregs_s *regs, const char *msg);
#endif

#ifdef CONFIG_SAMV7_HSMCI_XFRDEBUG
static void sam_xfrsampleinit(struct sam_dev_s *priv);
static void sam_xfrsample(struct sam_dev_s *priv, int index);
static void sam_xfrdumpone(struct sam_dev_s *priv, int index,
              const char *msg);
static void sam_xfrdump(struct sam_dev_s *priv);
#else
#  define   sam_xfrsampleinit(priv)
#  define   sam_xfrsample(priv,index)
#  define   sam_xfrdump(priv)
#endif

#ifdef CONFIG_SAMV7_HSMCI_CMDDEBUG
static void sam_cmdsampleinit(struct sam_dev_s *priv);
static inline void sam_cmdsample1(struct sam_dev_s *priv, int index3);
static inline void sam_cmdsample2(struct sam_dev_s *priv, int index,
              uint32_t sr);
static void sam_cmddump(struct sam_dev_s *priv);
#else
#  define   sam_cmdsampleinit(priv)
#  define   sam_cmdsample1(priv,index)
#  define   sam_cmdsample2(priv,index,sr)
#  define   sam_cmddump(priv)
#endif

/* DMA Helpers **************************************************************/

static void sam_dmacallback(DMA_HANDLE handle, void *arg, int result);
static inline uintptr_t hsmci_regaddr(struct sam_dev_s *priv,
                                      unsigned int offset);

/* Data Transfer Helpers ****************************************************/

static void sam_eventtimeout(wdparm_t arg);
static void sam_endwait(struct sam_dev_s *priv, sdio_eventset_t wkupevent);
static void sam_endtransfer(struct sam_dev_s *priv,
                            sdio_eventset_t wkupevent);
static void sam_notransfer(struct sam_dev_s *priv);

/* Interrupt Handling *******************************************************/

static int  sam_hsmci_interrupt(int irq, void *context, void *arg);

/* SDIO interface methods ***************************************************/

/* Initialization/setup */

static void sam_reset(struct sdio_dev_s *dev);
static sdio_capset_t sam_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t sam_status(struct sdio_dev_s *dev);
static void sam_widebus(struct sdio_dev_s *dev, bool enable);
static void sam_clock(struct sdio_dev_s *dev,
                      enum sdio_clock_e rate);
static int  sam_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  sam_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                        uint32_t arg);
static void sam_blocksetup(struct sdio_dev_s *dev, unsigned int blocklen,
                           unsigned int nblocks);
static int  sam_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                          size_t nbytes);
static int  sam_sendsetup(struct sdio_dev_s *dev,
                          const uint8_t *buffer, size_t nbytes);
static int  sam_cancel(struct sdio_dev_s *dev);
static int  sam_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  sam_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                          uint32_t *rshort);
static int  sam_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                         uint32_t rlong[4]);
static int  sam_recvnotimpl(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t *rnotimpl);

/* EVENT handler */

static void sam_waitenable(struct sdio_dev_s *dev,
                           sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t sam_eventwait(struct sdio_dev_s *dev);
static void sam_callbackenable(struct sdio_dev_s *dev,
                               sdio_eventset_t eventset);
static int  sam_registercallback(struct sdio_dev_s *dev,
                                 worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_SAMV7_HSMCI_DMA
#ifndef HSCMI_NORXDMA
static int  sam_dmarecvsetup(struct sdio_dev_s *dev,
                             uint8_t *buffer, size_t buflen);
#endif
#ifndef HSCMI_NOTXDMA
static int  sam_dmasendsetup(struct sdio_dev_s *dev,
                             const uint8_t *buffer, size_t buflen);
#endif
#endif

/* Initialization/uninitialization/reset ************************************/

static void sam_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pre-allocate memory for each HSMCI device */

#ifdef CONFIG_SAMV7_HSMCI0
static struct sam_dev_s g_hsmci0 =
{
  .dev =
  {
    .reset            = sam_reset,
    .capabilities     = sam_capabilities,
    .status           = sam_status,
    .widebus          = sam_widebus,
    .clock            = sam_clock,
    .attach           = sam_attach,
    .sendcmd          = sam_sendcmd,
    .blocksetup       = sam_blocksetup,
    .recvsetup        = sam_recvsetup,
    .sendsetup        = sam_sendsetup,
    .cancel           = sam_cancel,
    .waitresponse     = sam_waitresponse,
    .recv_r1          = sam_recvshort,
    .recv_r2          = sam_recvlong,
    .recv_r3          = sam_recvshort,
    .recv_r4          = sam_recvnotimpl,
    .recv_r5          = sam_recvnotimpl,
    .recv_r6          = sam_recvshort,
    .recv_r7          = sam_recvshort,
    .waitenable       = sam_waitenable,
    .eventwait        = sam_eventwait,
    .callbackenable   = sam_callbackenable,
    .registercallback = sam_registercallback,
#ifdef CONFIG_SDIO_DMA
#ifndef HSCMI_NORXDMA
    .dmarecvsetup     = sam_dmarecvsetup,
#else
    .dmarecvsetup     = sam_recvsetup,
#endif
#ifndef HSCMI_NOTXDMA
    .dmasendsetup     = sam_dmasendsetup,
#else
    .dmasendsetup     = sam_sendsetup,
#endif
#endif
  },
  .waitsem            = SEM_INITIALIZER(0),
  .base               = SAM_HSMCI0_BASE,
  .hsmci              = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_checkreg
 *
 * Description:
 *   Check if the current register access is a duplicate of the preceding.
 *
 * Input Parameters:
 *   value   - The value to be written
 *   address - The address of the register to write to
 *
 * Returned Value:
 *   true:  This is the first register access of this type.
 *   flase: This is the same as the preceding register access.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_HSMCI_REGDEBUG
static bool sam_checkreg(struct sam_dev_s *priv, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == priv->wrlast &&   /* Same kind of access? */
      value   == priv->vallast &&  /* Same value? */
      address == priv->addrlast)   /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      priv->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (priv->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          mcinfo("...[Repeats %d times]...\n", priv->ntimes);
        }

      /* Save information about the new access */

      priv->wrlast   = wr;
      priv->vallast  = value;
      priv->addrlast = address;
      priv->ntimes   = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: sam_getreg
 *
 * Description:
 *  Read an HSMCI register
 *
 ****************************************************************************/

static inline uint32_t sam_getreg(struct sam_dev_s *priv,
                                  unsigned int offset)
{
  uint32_t address = priv->base + offset;
  uint32_t value = getreg32(address);

#ifdef CONFIG_SAMV7_HSMCI_REGDEBUG
  if (sam_checkreg(priv, false, value, address))
    {
      mcinfo("%08x->%08x\n", address, value);
    }
#endif

  return value;
}

/****************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *  Write a value to an HSMCI register
 *
 ****************************************************************************/

static inline void sam_putreg(struct sam_dev_s *priv, uint32_t value,
                              unsigned int offset)
{
  uint32_t address = priv->base + offset;

#ifdef CONFIG_SAMV7_HSMCI_REGDEBUG
  if (sam_checkreg(priv, true, value, address))
    {
      mcinfo("%08x<-%08x\n", address, value);
    }
#endif

  putreg32(value, address);
}

/****************************************************************************
 * Name: sam_configwaitints
 *
 * Description:
 *   Configure HSMCI interrupts needed to support the wait function.  Wait
 *   interrupts are configured here, but not enabled until
 *   sam_enableints() is called.  Why?  Because the XFRDONE interrupt
 *   is always pending until start the data transfer.
 *
 * Input Parameters:
 *   priv       - A reference to the HSMCI device state structure
 *   waitmask   - The set of bits in the HSMCI MASK register to set
 *   waitevents - Waited for events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam_configwaitints(struct sam_dev_s *priv,
                                      uint32_t waitmask,
                                      sdio_eventset_t waitevents)
{
  irqstate_t flags;

  /* Save all of the data in one, atomic operation. */

  flags = enter_critical_section();
  priv->waitevents = waitevents;
  priv->wkupevent  = 0;
  priv->waitmask   = waitmask;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_disablewaitints
 *
 * Description:
 *   Disable HSMCI interrupts and save wakeup event.  Called
 *
 * Input Parameters:
 *   priv       - A reference to the HSMCI device state structure
 *   wkupevent  - Wake-up event(s)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_disablewaitints(struct sam_dev_s *priv,
                                sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags            = enter_critical_section();
  priv->waitevents = 0;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = 0;
  sam_putreg(priv, ~priv->xfrmask, SAM_HSMCI_IDR_OFFSET);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_configxfrints
 *
 * Description:
 *   Configure HSMCI interrupts needed to support the data transfer.  Data
 *   transfer interrupts are configured here, but not enabled until
 *   sam_enableints() is called.  Why?  Because the XFRDONE interrupt
 *   is always pending until start the data transfer.
 *
 * Input Parameters:
 *   priv    - A reference to the HSMCI device state structure
 *   xfrmask - The set of bits in the HSMCI MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam_configxfrints(struct sam_dev_s *priv,
                                     uint32_t xfrmask)
{
  priv->xfrmask = xfrmask;
}

/****************************************************************************
 * Name: sam_disablexfrints
 *
 * Description:
 *   Disable HSMCI interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the HSMCI device state structure
 *   xfrmask - The set of bits in the HSMCI MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_disablexfrints(struct sam_dev_s *priv)
{
  irqstate_t flags = enter_critical_section();
  priv->xfrmask = 0;
  sam_putreg(priv, ~priv->waitmask, SAM_HSMCI_IDR_OFFSET);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_enableints
 *
 * Description:
 *   Enable the previously configured HSMCI interrupts needed to support the
 *   wait and transfer functions.
 *
 * Input Parameters:
 *   priv - A reference to the HSMCI device state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam_enableints(struct sam_dev_s *priv)
{
  /* Enable all interrupts associated with the waited-for event */

  sam_putreg(priv, priv->xfrmask | priv->waitmask, SAM_HSMCI_IER_OFFSET);
}

/****************************************************************************
 * Name: sam_disable
 *
 * Description:
 *   Disable the HSMCI
 *
 ****************************************************************************/

static inline void sam_disable(struct sam_dev_s *priv)
{
  /* Disable the MCI */

  sam_putreg(priv, HSMCI_CR_MCIDIS, SAM_HSMCI_CR_OFFSET);

  /* Disable all the interrupts */

  sam_putreg(priv, 0xffffffff, SAM_HSMCI_IDR_OFFSET);
}

/****************************************************************************
 * Name: sam_enable
 *
 * Description:
 *   Enable the HSMCI
 *
 ****************************************************************************/

static inline void sam_enable(struct sam_dev_s *priv)
{
  /* Enable the MCI and the Power Saving */

  sam_putreg(priv, HSMCI_CR_MCIEN, SAM_HSMCI_CR_OFFSET);
}

/****************************************************************************
 * Name: sam_hsmcisample
 *
 * Description:
 *   Sample HSMCI registers
 *
 ****************************************************************************/

#if defined(CONFIG_SAMV7_HSMCI_XFRDEBUG) || defined(CONFIG_SAMV7_HSMCI_CMDDEBUG)
static void sam_hsmcisample(struct sam_dev_s *priv,
                            struct sam_hsmciregs_s *regs)
{
  regs->mr    = sam_getreg(priv, SAM_HSMCI_MR_OFFSET);
  regs->dtor  = sam_getreg(priv, SAM_HSMCI_DTOR_OFFSET);
  regs->sdcr  = sam_getreg(priv, SAM_HSMCI_SDCR_OFFSET);
  regs->argr  = sam_getreg(priv, SAM_HSMCI_ARGR_OFFSET);
  regs->blkr  = sam_getreg(priv, SAM_HSMCI_BLKR_OFFSET);
  regs->cstor = sam_getreg(priv, SAM_HSMCI_CSTOR_OFFSET);
#if 0 /* Reading these can cause loss of response data */
  regs->rsp0  = sam_getreg(priv, SAM_HSMCI_RSPR0_OFFSET);
  regs->rsp1  = sam_getreg(priv, SAM_HSMCI_RSPR1_OFFSET);
  regs->rsp2  = sam_getreg(priv, SAM_HSMCI_RSPR2_OFFSET);
  regs->rsp3  = sam_getreg(priv, SAM_HSMCI_RSPR3_OFFSET);
#endif
  regs->sr    = sam_getreg(priv, SAM_HSMCI_SR_OFFSET);
  regs->imr   = sam_getreg(priv, SAM_HSMCI_IMR_OFFSET);
  regs->dma   = sam_getreg(priv, SAM_HSMCI_DMA_OFFSET);
  regs->cfg   = sam_getreg(priv, SAM_HSMCI_CFG_OFFSET);
  regs->wpmr  = sam_getreg(priv, SAM_HSMCI_WPMR_OFFSET);
  regs->wpsr  = sam_getreg(priv, SAM_HSMCI_WPSR_OFFSET);
}
#endif

/****************************************************************************
 * Name: sam_hsmcidump
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#if defined(CONFIG_SAMV7_HSMCI_XFRDEBUG) || defined(CONFIG_SAMV7_HSMCI_CMDDEBUG)
static void sam_hsmcidump(struct sam_dev_s *priv,
                          struct sam_hsmciregs_s *regs, const char *msg)
{
  mcinfo("HSMCI Registers: %s\n", msg);
  mcinfo("      MR[%08x]: %08x\n",
         priv->base + SAM_HSMCI_MR_OFFSET, regs->mr);
  mcinfo("    DTOR[%08x]: %08x\n",
         priv->base + SAM_HSMCI_DTOR_OFFSET, regs->dtor);
  mcinfo("    SDCR[%08x]: %08x\n",
         priv->base + SAM_HSMCI_SDCR_OFFSET, regs->sdcr);
  mcinfo("    ARGR[%08x]: %08x\n",
         priv->base + SAM_HSMCI_ARGR_OFFSET, regs->argr);
  mcinfo("    BLKR[%08x]: %08x\n",
         priv->base + SAM_HSMCI_BLKR_OFFSET, regs->blkr);
  mcinfo("   CSTOR[%08x]: %08x\n",
         priv->base + SAM_HSMCI_CSTOR_OFFSET, regs->cstor);
#if 0 /* Reading these can cause loss of response data */
  mcinfo("   RSPR0[%08x]: %08x\n",
         priv->base + SAM_HSMCI_RSPR0_OFFSET, regs->rsp0);
  mcinfo("   RSPR1[%08x]: %08x\n",
         priv->base + SAM_HSMCI_RSPR1_OFFSET, regs->rsp1);
  mcinfo("   RSPR2[%08x]: %08x\n",
         priv->base + SAM_HSMCI_RSPR2_OFFSET, regs->rsp2);
  mcinfo("   RSPR3[%08x]: %08x\n",
         priv->base + SAM_HSMCI_RSPR3_OFFSET, regs->rsp3);
#endif
  mcinfo("      SR[%08x]: %08x\n",
         priv->base + SAM_HSMCI_SR_OFFSET, regs->sr);
  mcinfo("     IMR[%08x]: %08x\n",
         priv->base + SAM_HSMCI_IMR_OFFSET, regs->imr);
  mcinfo("     DMA[%08x]: %08x\n",
         priv->base + SAM_HSMCI_DMA_OFFSET, regs->dma);
  mcinfo("     CFG[%08x]: %08x\n",
         priv->base + SAM_HSMCI_CFG_OFFSET, regs->cfg);
  mcinfo("    WPMR[%08x]: %08x\n",
         priv->base + SAM_HSMCI_WPMR_OFFSET, regs->wpmr);
  mcinfo("    WPSR[%08x]: %08x\n",
         priv->base + SAM_HSMCI_WPSR_OFFSET, regs->wpsr);
}
#endif

/****************************************************************************
 * Name: sam_xfrsample
 *
 * Description:
 *   Sample HSMCI/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_HSMCI_XFRDEBUG
static void sam_xfrsample(struct sam_dev_s *priv, int index)
{
  /* On a multiple block transfer, only sample on the first block */

  if ((priv->smplset & (1 << index)) == 0)
    {
      struct sam_xfrregs_s *regs = &priv->xfrsamples[index];

#ifdef CONFIG_DEBUG_DMA
      sam_dmasample(priv->dma, &regs->dma);
#endif
      sam_hsmcisample(priv, &regs->hsmci);
      priv->smplset |= (1 << index);
    }
}
#endif

/****************************************************************************
 * Name: sam_xfrsampleinit
 *
 * Description:
 *   Setup prior to collecting transfer samples
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_HSMCI_XFRDEBUG
static void sam_xfrsampleinit(struct sam_dev_s *priv)
{
  priv->smplset = 0;
  memset(priv->xfrsamples, 0xff,
         DEBUG_NDMASAMPLES * sizeof(struct sam_xfrregs_s));

#ifdef CONFIG_SAMV7_HSMCI_CMDDEBUG
  priv->xfrinitialized = true;
#endif
}
#endif

/****************************************************************************
 * Name: sam_xfrdumpone
 *
 * Description:
 *   Dump one transfer register sample
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_HSMCI_XFRDEBUG
static void sam_xfrdumpone(struct sam_dev_s *priv, int index,
                           const char *msg)
{
  if ((priv->smplset & (1 << index)) != 0)
    {
      struct sam_xfrregs_s *regs = &priv->xfrsamples[index];

#ifdef CONFIG_DEBUG_DMA
      sam_dmadump(priv->dma, &regs->dma, msg);
#endif
      sam_hsmcidump(priv, &regs->hsmci, msg);
    }
  else
    {
      mcerr("ERROR: %s: Not collected\n", msg);
    }
}
#endif

/****************************************************************************
 * Name: sam_xfrdump
 *
 * Description:
 *   Dump all transfer-related, sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_HSMCI_XFRDEBUG
static void  sam_xfrdump(struct sam_dev_s *priv)
{
#ifdef CONFIG_SAMV7_HSMCI_CMDDEBUG
  if (priv->xfrinitialized)
#endif
    {
      sam_xfrdumpone(priv, SAMPLENDX_BEFORE_SETUP, "Before setup");
#ifdef CONFIG_DEBUG_DMA
      sam_xfrdumpone(priv, SAMPLENDX_BEFORE_ENABLE, "Before DMA enable");
#endif
      sam_xfrdumpone(priv, SAMPLENDX_AFTER_SETUP, "After setup");
      sam_xfrdumpone(priv, SAMPLENDX_END_TRANSFER, "End of transfer");
#ifdef CONFIG_DEBUG_DMA
      sam_xfrdumpone(priv, SAMPLENDX_DMA_CALLBACK, "DMA Callback");
#endif
      sam_xfrdumpone(priv, SAMPLENDX_TIMEOUT, "Timeout");

      priv->smplset        = 0;
#ifdef CONFIG_SAMV7_HSMCI_CMDDEBUG
      priv->xfrinitialized = false;
#endif
    }
}
#endif

/****************************************************************************
 * Name: sam_cmdsampleinit
 *
 * Description:
 *   Setup prior to collecting command/response samples
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_HSMCI_CMDDEBUG
static void sam_cmdsampleinit(struct sam_dev_s *priv)
{
  memset(priv->cmdsamples, 0xff,
         DEBUG_NCMDSAMPLES * sizeof(struct sam_hsmciregs_s));

#ifdef CONFIG_SAMV7_HSMCI_XFRDEBUG
  priv->cmdinitialized = true;
#endif
}
#endif

/****************************************************************************
 * Name: sam_cmdsample1 & 2
 *
 * Description:
 *   Sample command/response registers
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_HSMCI_CMDDEBUG
static inline void sam_cmdsample1(struct sam_dev_s *priv, int index)
{
  sam_hsmcisample(priv, &priv->cmdsamples[index]);
}

static inline void sam_cmdsample2(struct sam_dev_s *priv, int index,
                                  uint32_t sr)
{
  sam_hsmcisample(priv, &priv->cmdsamples[index]);
  priv->cmdsamples[index].sr = sr;
}
#endif

/****************************************************************************
 * Name: sam_cmddump
 *
 * Description:
 *   Dump all command/response register data
 *
 ****************************************************************************/

#ifdef CONFIG_SAMV7_HSMCI_CMDDEBUG
static void sam_cmddump(struct sam_dev_s *priv)
{
#ifdef CONFIG_SAMV7_HSMCI_XFRDEBUG
  if (priv->cmdinitialized)
#endif
    {
      sam_hsmcidump(priv, &priv->cmdsamples[SAMPLENDX_AFTER_CMDR],
                    "After command setup");
      sam_hsmcidump(priv, &priv->cmdsamples[SAMPLENDX_AT_WAKEUP],
                    "After wakeup");
#ifdef CONFIG_SAMV7_HSMCI_XFRDEBUG
      priv->cmdinitialized = false;
#endif
    }
}
#endif

/****************************************************************************
 * Name: sam_dmacallback
 *
 * Description:
 *   Called when HSMCI DMA completes
 *
 ****************************************************************************/

static void sam_dmacallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg;
  sdio_eventset_t wkupevent;

  /* Is DMA still active?  We can get this callback when sam_dmastop() is
   * called too.
   */

  if (priv->dmabusy)
    {
      /* Mark the DMA not busy and sample DMA registers */

      priv->dmabusy = false;
      sam_xfrsample((struct sam_dev_s *)arg, SAMPLENDX_DMA_CALLBACK);

      /* Disable the DMA handshaking */

      sam_putreg(priv, 0, SAM_HSMCI_DMA_OFFSET);

      /* Terminate the transfer with an I/O error in the event of a DMA
       * failure.
       */

      if (result < 0)
        {
          wkupevent = (result == -ETIMEDOUT ?
                       SDIOWAIT_TIMEOUT : SDIOWAIT_ERROR);

          mcerr("ERROR: DMA failed: result=%d wkupevent=%04x\n",
                result, wkupevent);

          /* sam_endtransfer will terminate the transfer and wait up the
           * waiting client in this case.
           */

          sam_endtransfer(priv, wkupevent);
        }

      /* The DMA completed without error.  Wake-up the waiting client if
       * (1) both the HSMCI and DMA completion events, and (2) There is a
       * client waiting for this event.
       *
       * If the HSMCI transfer event has already completed, it must have
       * completed successfully (because the DMA was not cancelled).
       * sam_endtransfer() should have already received the
       * SDIOWAIT_TRANSFERDONE event, but this event would not yet have been
       * recorded.  We need to post the SDIOWAIT_TRANSFERDONE again in this
       * case here.
       *
       * The timeout will remain active until sam_endwait() is eventually
       * called so we should not have any concern about hangs if the HSMCI
       * transfer never completed.
       */

      else if (!priv->xfrbusy &&
               (priv->waitevents & SDIOWAIT_TRANSFERDONE) != 0)
        {
          /* Okay.. wake up any waiting threads */

          sam_endwait(priv, SDIOWAIT_TRANSFERDONE);
        }
    }
}

/****************************************************************************
 * Name: hsmci_regaddr
 *
 * Description:
 *   Return the physical address of an HSMCI register
 *
 ****************************************************************************/

static inline uintptr_t hsmci_regaddr(struct sam_dev_s *priv,
                                      unsigned int offset)
{
  return priv->base + offset;
}

/****************************************************************************
 * Name: sam_eventtimeout
 *
 * Description:
 *   The watchdog timeout setup when the event wait start has expired without
 *   any other waited-for event occurring.
 *
 * Input Parameters:
 *   arg    - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sam_eventtimeout(wdparm_t arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg;

  DEBUGASSERT(priv != NULL);
  sam_xfrsample(priv, SAMPLENDX_TIMEOUT);

  /* Make sure that any hung DMA is stopped.  dmabusy == false is the cue
   * so the DMA callback is ignored.
   */

  priv->dmabusy = false;
  sam_dmastop(priv->dma);

  /* Disable the DMA handshaking */

  sam_putreg(priv, 0, SAM_HSMCI_DMA_OFFSET);

  /* Make sure that any hung HSMCI transfer is stopped */

  sam_disablexfrints(priv);
  sam_notransfer(priv);

  /* Is a data timeout complete event expected? (should always be the case) */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

      sam_endwait(priv, SDIOWAIT_TIMEOUT);
      mcerr("ERROR: Timeout\n");
    }
}

/****************************************************************************
 * Name: sam_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the HSMCI device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sam_endwait(struct sam_dev_s *priv, sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts and save wakeup event */

  sam_disablewaitints(priv, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: sam_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the HSMCI interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv   - An instance of the HSMCI device interface
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sam_endtransfer(struct sam_dev_s *priv,
                            sdio_eventset_t wkupevent)
{
  /* Disable all transfer related interrupts */

  sam_disablexfrints(priv);

  /* No data transfer */

  sam_notransfer(priv);

  /* DMA debug instrumentation */

  sam_xfrsample(priv, SAMPLENDX_END_TRANSFER);

  /* Make sure that the DMA is stopped (it will be stopped automatically
   * on normal transfers, but not necessarily when the transfer terminates
   * on an error condition).
   */

  if ((wkupevent & (SDIOWAIT_TIMEOUT | SDIOWAIT_ERROR)) != 0)
    {
      /* dmabusy == false gives the DMA callback handler a clue about what
       * is going on.
       */

      priv->dmabusy = false;
      sam_dmastop(priv->dma);

      /* Disable the DMA handshaking */

      sam_putreg(priv, 0, SAM_HSMCI_DMA_OFFSET);
    }

  /* The transfer is complete.  Wake-up the waiting client if (1) both the
   * HSMCI and DMA completion events, and (2) There is a client waiting for
   * this event.
   *
   * The timeout will remain active until sam_endwait() is eventually called
   * so we should not have any concern about hangs if the DMA never
   * completes.
   */

  if (!priv->dmabusy && (priv->waitevents & wkupevent) != 0)
    {
      /* Okay.. wake up any waiting threads */

      sam_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: sam_notransfer
 *
 * Description:
 *   Setup for no transfer.  This is called both before beginning a new
 *   transfer and when a transfer completes.  In the first case, this is the
 *   default setup that is overridden by sam_dmarecvsetup or sam_dmasendsetup
 *
 * Input Parameters:
 *   priv   - An instance of the HSMCI device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_notransfer(struct sam_dev_s *priv)
{
  uint32_t regval;

  /* Make read/write proof (or not).  This is a legacy behavior: This really
   * just needs be be done once at initialization time.
   */

  regval  = sam_getreg(priv, SAM_HSMCI_MR_OFFSET);
  regval &= ~(HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF);
  sam_putreg(priv, regval, SAM_HSMCI_MR_OFFSET);

  /* Clear the block size and count */

  sam_putreg(priv, 0, SAM_HSMCI_BLKR_OFFSET);

  /* Clear transfer flags (DMA could still be active in a corner case) */

  priv->xfrbusy = false;
  priv->txbusy  = false;
}

/****************************************************************************
 * Name: sam_hsmci_interrupt
 *
 * Description:
 *   HSMCI interrupt handler
 *
 * Input Parameters:
 *   Standard interrupt handler arguments.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sam_hsmci_interrupt(int irq, void *context, void *arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg;
  uint32_t sr;
  uint32_t enabled;
  uint32_t pending;

  DEBUGASSERT(priv != NULL);

  /* Loop while there are pending interrupts. */

  for (; ; )
    {
      /* Check the HSMCI status register.  Mask out all bits that don't
       * correspond to enabled interrupts.  (This depends on the fact that
       * bits are ordered the same in both the SR and IMR registers).  If
       * there are non-zero bits remaining, then we have work to do here.
       */

      sr      = sam_getreg(priv, SAM_HSMCI_SR_OFFSET);
      enabled = sr & sam_getreg(priv, SAM_HSMCI_IMR_OFFSET);

      if (enabled == 0)
        {
          break;
        }

      /* Handle in progress, interrupt driven data transfers ****************/

      /* Do any of these interrupts signal a data transfer event? */

      pending = enabled & priv->xfrmask;
      if (pending != 0)
        {
          /* Yes.. Did the transfer complete with an error? */

          if ((pending & HSMCI_DATA_ERRORS) != 0)
            {
              /* Yes.. Was it some kind of timeout error? */

              mcerr("ERROR: enabled: %08" PRIx32 " pending: %08" PRIx32 "\n",
                    enabled, pending);

              if ((pending & HSMCI_DATA_TIMEOUT_ERRORS) != 0)
                {
                  /* Yes.. Terminate with a timeout. */

                  sam_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
                }
              else
                {
                  /* No..  Terminate with an I/O error. */

                  sam_endtransfer(priv,
                                  SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
                }
            }

          /* No, If RXRDY is enabled, then we are doing a non-DMA receive.
           * We need to transfer word(s) from the RDR register to the user
           * buffer.
           */

          else if ((pending & HSMCI_INT_RXRDY) != 0)
            {
#ifdef CONFIG_SAMV7_HSMCI_UNALIGNED
              uint32_t value;

              /* Interrupt mode data transfer support */

              DEBUGASSERT(!priv->dmabusy && priv->xfrbusy && !priv->txbusy);
              DEBUGASSERT(priv->buffer && priv->remaining > 0);

              /* Is the receiving buffer aligned? */

              value = sam_getreg(priv, SAM_HSMCI_RDR_OFFSET);

              if (((uintptr_t)priv->buffer & 3) == 0 &&
                  priv->remaining >= sizeof(uint32_t))
                {
                  /* Yes.. transfer 32-bits at a time */

                  *priv->buffer++  = value;
                  priv->remaining -= sizeof(uint32_t);
                }
              else
                {
                  /* No.. transfer 8-bits at a time (little endian source) */

                  uint8_t *dest8 = (uint8_t *)priv->buffer;

                  /* Unpack little endian; write in native byte order */

                  *dest8++ = (value & 0xff);
                  if (--priv->remaining > 0)
                    {
                      *dest8++ = ((value >> 8) & 0xff);

                      if (--priv->remaining > 0)
                        {
                          *dest8++ = ((value >> 16) & 0xff);

                          if (--priv->remaining > 0)
                            {
                              *dest8++ = ((value >> 24) & 0xff);
                            }
                        }
                    }

                  priv->buffer = (uint32_t *)dest8;
                }
#else
              /* Interrupt mode data transfer support */

              DEBUGASSERT(!priv->dmabusy && priv->xfrbusy && !priv->txbusy);
              DEBUGASSERT(priv->buffer && priv->remaining > 0);

              /* Transfer 32-bit aligned data */

              *priv->buffer++  = sam_getreg(priv, SAM_HSMCI_RDR_OFFSET);
              priv->remaining -= sizeof(uint32_t);
#endif
              /* Are we finished? */

              if (priv->remaining <= 0)
                {
                  /* Yes.. End the transfer */

                  priv->buffer    = NULL;
                  priv->remaining = 0;
                  sam_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
                }
            }

          /* Otherwise it must be a DMA transfer that completed
           * successfully.
           */

          else
            {
              /* If buffer is not NULL that means that RX DMA is finished.
               * We need to invalidate RX buffer
               */

              if (priv->buffer != NULL)
                {
                  DEBUGASSERT(priv->remaining > 0);

                  up_invalidate_dcache((uintptr_t)priv->buffer,
                                       (uintptr_t)priv->buffer +
                                       priv->remaining);
                  priv->buffer    = NULL;
                  priv->remaining = 0;
                }

              /* End the transfer */

              sam_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
            }
        }

      /* Handle wait events *************************************************/

      /* Do any of these interrupts signal wakeup event? */

      pending = enabled & priv->waitmask;
      if (pending != 0)
        {
          sdio_eventset_t wkupevent = 0;

          /* Is this a Command-Response sequence completion event? */

          if ((pending & priv->cmdrmask) != 0)
            {
              sam_cmdsample2(priv, SAMPLENDX_AT_WAKEUP, sr);

              /* Yes.. Did the Command-Response sequence end with an error? */

              if ((pending & HSMCI_RESPONSE_ERRORS) != 0)
                {
                  /* Yes.. Was the error some kind of timeout? */

                  mcerr("ERROR: events: %08" PRIx32 " SR: %08" PRIx32 "\n",
                         priv->cmdrmask, enabled);

                  if ((pending & HSMCI_RESPONSE_TIMEOUT_ERRORS) != 0)
                    {
                      /* Yes.. signal a timeout error */

                      wkupevent = SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE |
                                  SDIOWAIT_TIMEOUT;
                    }
                  else
                    {
                      /* No.. signal some generic I/O error */

                      wkupevent = SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE |
                                  SDIOWAIT_ERROR;
                    }
                }
              else
                {
                  /* The Command-Response sequence ended with no error */

                  wkupevent = SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE;
                }

              /* Yes.. Is there a thread waiting for this event set? */

              wkupevent &= priv->waitevents;
              if (wkupevent != 0)
                {
                  /* Yes.. wake the thread up */

                  sam_endwait(priv, wkupevent);
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sam_reset
 *
 * Description:
 *   Reset the HSMCI controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_reset(struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  irqstate_t flags;

  /* Reset the MCI */

  flags = enter_critical_section();
  sam_putreg(priv, HSMCI_CR_SWRST, SAM_HSMCI_CR_OFFSET);

  /* Disable the MCI */

  sam_putreg(priv, HSMCI_CR_MCIDIS | HSMCI_CR_PWSDIS, SAM_HSMCI_CR_OFFSET);

  /* Disable all the interrupts */

  sam_putreg(priv, 0xffffffff, SAM_HSMCI_IDR_OFFSET);

  /* Set the Data Timeout Register */

  sam_putreg(priv, HSMCI_DTOR_DTOCYC_MAX | HSMCI_DTOR_DTOMUL_MAX,
             SAM_HSMCI_DTOR_OFFSET);

  /* Set the Mode Register for ID mode frequency (probably 400KHz) */

  sam_clock(dev, CLOCK_IDMODE);

  /* Set the SDCard Register */

  sam_putreg(priv, HSMCI_SDCR_SDCSEL_SLOTA | HSMCI_SDCR_SDCBUS_4BIT,
             SAM_HSMCI_SDCR_OFFSET);

  /* Enable the MCI controller */

  sam_putreg(priv, HSMCI_CR_MCIEN, SAM_HSMCI_CR_OFFSET);

  /* Disable the DMA interface */

  sam_putreg(priv, 0, SAM_HSMCI_DMA_OFFSET);

  /* Configure MCI */

  sam_putreg(priv, HSMCI_CFG_FIFOMODE, SAM_HSMCI_CFG_OFFSET);

  /* No data transfer */

  sam_notransfer(priv);

  /* Reset data */

  priv->waitevents = 0;       /* Set of events to be waited for */
  priv->waitmask   = 0;       /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;       /* The event that caused the wakeup */
  priv->dmabusy    = false;   /* No DMA in progress */
  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->xfrmask    = 0;       /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  priv->widebus    = false;   /* Required for DMA support */
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_capabilities
 *
 * Description:
 *   Get capabilities (and limitations) of the SDIO driver (optional)
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see SDIO_CAPS_* defines)
 *
 ****************************************************************************/

static sdio_capset_t sam_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

#ifdef CONFIG_SAMV7_HSMCI_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif
  caps |= SDIO_CAPS_DMABEFOREWRITE;

  return caps;
}

/****************************************************************************
 * Name: sam_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see sam_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t sam_status(struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: sam_widebus
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   wide - true: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_widebus(struct sdio_dev_s *dev, bool wide)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regval;

  /* Set 1-bit or 4-bit bus by configuring the SDCBUS field of the SDCR
   * register.
   */

  regval  = sam_getreg(priv, SAM_HSMCI_SDCR_OFFSET);
  regval &= ~HSMCI_SDCR_SDCBUS_MASK;
  regval |= wide ? HSMCI_SDCR_SDCBUS_4BIT : HSMCI_SDCR_SDCBUS_1BIT;
  sam_putreg(priv, regval, SAM_HSMCI_SDCR_OFFSET);

  /* Remember the setting */

  priv->widebus = wide;
}

/****************************************************************************
 * Name: sam_clock
 *
 * Description:
 *   Enable/disable SDIO clocking
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regval;
  bool enable = true;

  /* Fetch the current mode register and mask out the clkdiv+clockodd (and
   * pwsdiv).
   */

  regval = sam_getreg(priv, SAM_HSMCI_MR_OFFSET);
  regval &= ~(HSMCI_MR_CLKDIV_MASK | HSMCI_MR_PWSDIV_MASK | HSMCI_MR_CLKODD);

  /* These clock devisor values that must be defined in the board-specific
   * board.h header file: HSMCI_INIT_CLKDIV, HSMCI_MMCXFR_CLKDIV,
   * HSMCI_SDXFR_CLKDIV, and HSMCI_SDWIDEXFR_CLKDIV.
   */

  switch (rate)
    {
    default:
    case CLOCK_SDIO_DISABLED:     /* Clock is disabled */
      regval |= HSMCI_INIT_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      enable = false;
      return;

    case CLOCK_IDMODE:            /* Initial ID mode clocking (<400KHz) */
      regval |= HSMCI_INIT_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;

    case CLOCK_MMC_TRANSFER:      /* MMC normal operation clocking */
      regval |= HSMCI_MMCXFR_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;

    case CLOCK_SD_TRANSFER_1BIT:  /* SD normal operation clocking (narrow 1-bit mode) */
      regval |= HSMCI_SDXFR_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;

    case CLOCK_SD_TRANSFER_4BIT:  /* SD normal operation clocking (wide 4-bit mode) */
      regval |= HSMCI_SDWIDEXFR_CLKDIV | HSMCI_MR_PWSDIV_MAX;
      break;
    };

  /* Set the new clock  diver and make sure that the clock is enabled or
   * disabled, whichever the case.
   */

  sam_putreg(priv, regval, SAM_HSMCI_MR_OFFSET);
  if (enable)
    {
      sam_enable(priv);
    }
  else
    {
      sam_disable(priv);
    }
}

/****************************************************************************
 * Name: sam_attach
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int sam_attach(struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  int irq;
  int ret;

  /* Select the handler and IRQ */

#ifdef CONFIG_SAMV7_HSMCI0
  if (priv->hsmci == 0)
    {
      irq = SAM_IRQ_HSMCI0;
    }
  else
#endif
    {
      DEBUGPANIC();
      return -EINVAL; /* Shouldn't happen */
    }

  /* Attach the HSMCI interrupt handler */

  ret = irq_attach(irq, sam_hsmci_interrupt, priv);
  if (ret == OK)
    {
      /* Disable all interrupts at the HSMCI controller and clear (most)
       * static interrupt flags by reading the status register.
       */

      sam_putreg(priv, 0xffffffff, SAM_HSMCI_IDR_OFFSET);
      sam_getreg(priv, SAM_HSMCI_SR_OFFSET);

      /* Enable HSMCI interrupts at the NVIC.  They can now be enabled at
       * the HSMCI controller as needed.
       */

      up_enable_irq(irq);
    }

  return ret;
}

/****************************************************************************
 * Name: sam_sendcmd
 *
 * Description:
 *   Send the SDIO command
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command to send (32-bits, encoded)
 *   arg  - 32-bit argument required with some commands
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sam_sendcmd(struct sdio_dev_s *dev,
                       uint32_t cmd, uint32_t arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regval;
  uint32_t cmdidx;

  sam_cmdsampleinit(priv);

  /* Set the HSMCI Argument value */

  sam_putreg(priv, arg, SAM_HSMCI_ARGR_OFFSET);

  /* Construct the command valid, starting with the command index */

  cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval  = cmdidx << HSMCI_CMDR_CMDNB_SHIFT;

  /* 'OR' in response related bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    /* No response */

    case MMCSD_NO_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= HSMCI_CMDR_RSPTYP_NONE;

      break;

    /* 48-bit response with CRC */

    case MMCSD_R1_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= HSMCI_CMDR_RSPTYP_48BIT | HSMCI_CMDR_MAXLAT;
      break;

    case MMCSD_R1B_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= HSMCI_CMDR_RSPTYP_R1B | HSMCI_CMDR_MAXLAT;
      break;

    /* 48-bit response without CRC */

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_NOCRC_INTS;
      regval |= HSMCI_CMDR_RSPTYP_48BIT | HSMCI_CMDR_MAXLAT;
      break;

    /* 136-bit response with CRC */

    case MMCSD_R2_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= HSMCI_CMDR_RSPTYP_136BIT | HSMCI_CMDR_MAXLAT;
      break;
    }

  /* 'OR' in data transfer related bits */

  switch (cmd & MMCSD_DATAXFR_MASK)
    {
#if 0 /* No MMC support */
    case MMCSD_RDSTREAM: /* MMC Read stream */
      regval |= HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRTYP_STREAM |
                HSMCI_CMDR_TRDIR_READ;
      break;

    case MMCSD_WRSTREAM: /* MMC Write stream */
      regval |= HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRTYP_STREAM |
                HSMCI_CMDR_TRDIR_WRITE;
      break;
#endif

    case MMCSD_RDDATAXFR: /* Read block transfer */
      regval |= HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRDIR_READ;
      regval |= (cmd & MMCSD_MULTIBLOCK) ?
                HSMCI_CMDR_TRTYP_MULTIPLE : HSMCI_CMDR_TRTYP_SINGLE;
      break;

    case MMCSD_WRDATAXFR: /* Write block transfer */
      regval |= HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRDIR_WRITE;
      regval |= (cmd & MMCSD_MULTIBLOCK) ?
                HSMCI_CMDR_TRTYP_MULTIPLE : HSMCI_CMDR_TRTYP_SINGLE;
      break;

    case MMCSD_NODATAXFR:
    default:
      if ((cmd & MMCSD_STOPXFR) != 0)
        {
          regval |= HSMCI_CMDR_TRCMD_STOP;
        }
      break;
    }

  /* 'OR' in Open Drain option */

#if 0 /* No MMC support */
  if ((cmd & MMCSD_OPENDRAIN) != 0)
    {
      regval |= HSMCI_CMDR_OPDCMD;
    }
#endif

  /* Write the fully decorated command to CMDR */

  mcinfo("cmd: %08" PRIx32 " arg: %08" PRIx32 " regval: %08" PRIx32 "\n",
         cmd, arg, regval);
  sam_putreg(priv, regval, SAM_HSMCI_CMDR_OFFSET);
  sam_cmdsample1(priv, SAMPLENDX_AFTER_CMDR);
  return OK;
}

/****************************************************************************
 * Name: sam_blocksetup
 *
 * Description:
 *   Some hardware needs to be informed of the selected blocksize.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   blocklen - The selected block size.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_blocksetup(struct sdio_dev_s *dev, unsigned int blocklen,
                           unsigned int nblocks)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regval;

  DEBUGASSERT(dev != NULL && nblocks > 0 && nblocks < 65535);
  DEBUGASSERT(blocklen < 65535 && (blocklen & 3) == 0);

  /* Make read/write proof (or not).  This is a legacy behavior: This really
   * just needs be be done once at initialization time.
   */

  regval = sam_getreg(priv, SAM_HSMCI_MR_OFFSET);
  regval &= ~(HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF);
  regval |= HSMCU_PROOF_BITS;
  sam_putreg(priv, regval, SAM_HSMCI_MR_OFFSET);

  /* Set the block size and count */

  regval = (blocklen << HSMCI_BLKR_BLKLEN_SHIFT) |
           (nblocks  << HSMCI_BLKR_BCNT_SHIFT);
  sam_putreg(priv, regval, SAM_HSMCI_BLKR_OFFSET);
}

/****************************************************************************
 * Name: sam_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.  This would be called for SD memory just BEFORE sending
 *   CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally,
 *   SDIO_WAITEVENT will be called to receive the indication that the
 *   transfer is complete.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer in which to receive the data
 *   buflen - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int sam_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                         size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

#ifndef CONFIG_SAMV7_HSMCI_UNALIGNED
  /* Default behavior is to transfer 32-bit values only */

  if (((uintptr_t)buffer & 3) != 0 || (buflen & 3) != 0)
    {
      /* Return the -EFAULT error. */

      return -EFAULT;
    }
#endif

  /* Initialize register sampling */

  sam_xfrsampleinit(priv);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Disable DMA handshaking */

  sam_putreg(priv, 0, SAM_HSMCI_DMA_OFFSET);

  /* Setup of the transfer configuration */

  priv->dmabusy = false;
  priv->xfrbusy = true;
  priv->txbusy  = false;

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;

  /* And enable interrupts */

  sam_configxfrints(priv, HSMCI_RECV_INTS);
  sam_xfrsample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: sam_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.  This
 *   method will do whatever controller setup is necessary.  This would be
 *   called for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK), ... and before SDIO_SENDDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer containing the data to send
 *   buflen - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int sam_sendsetup(struct sdio_dev_s *dev,
                         const uint8_t *buffer, size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  unsigned int remaining;
  const uint32_t *src;
  uint32_t sr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

#ifndef CONFIG_SAMV7_HSMCI_UNALIGNED
  /* Default behavior is to transfer 32-bit values only */

  if (((uintptr_t)buffer & 3) != 0 || (buflen & 3) != 0)
    {
      /* Return the -EFAULT error. */

      return -EFAULT;
    }
#endif

  /* Disable DMA handshaking */

  sam_putreg(priv, 0, SAM_HSMCI_DMA_OFFSET);
  sam_configxfrints(priv, HSMCI_DMASEND_INTS);

  priv->dmabusy = false;
  priv->xfrbusy = true;
  priv->txbusy  = true;

  /* Nullify register sampling */

  sam_xfrsampleinit(priv);

  /* Copy each word to the TX FIFO
   *
   * It is necessary to disable pre-emption and interrupts around this loop
   * in order to avoid a TX data underrun.
   */

  sched_lock();
  flags = enter_critical_section();

  src       = (const uint32_t *)buffer;
  remaining = buflen;

  while (remaining > 0)
    {
      /* Check the HSMCI status */

      sr = sam_getreg(priv, SAM_HSMCI_SR_OFFSET);
      if ((sr & HSMCI_DATA_SEND_ERRORS) != 0)
        {
          /* Some fatal error has occurred */

          mcerr("ERROR: sr %08" PRIx32 "\n", sr);
          leave_critical_section(flags);
          sched_unlock();
          return -EIO;
        }
      else if ((sr & HSMCI_INT_TXRDY) != 0)
        {
          /* TXRDY -- transfer another word */

#ifdef CONFIG_SAMV7_HSMCI_UNALIGNED
          /* First check:  Do we we have 32-bit address alignment?  Are we
           * transferring a full 32-bits?
           */

          if (((uintptr_t)priv->buffer & 3) == 0 &&
              priv->remaining >= sizeof(uint32_t))
            {
              /* Yes.. transfer 32-bits at a time */

              sam_putreg(priv, *src++, SAM_HSMCI_TDR_OFFSET);
              remaining -= sizeof(uint32_t);
            }
          else
            {
              /* No.. transfer 8-bits at a time (little endian destination) */

              const uint8_t *src8 = (const uint8_t *)src;
              uint32_t value;

              /* Read data in native byte order, pack little endian */

              value = (uint32_t)*src8++;
              if (--remaining > 0)
                {
                  value |= ((uint32_t)*src8++ << 8);

                  if (--remaining > 0)
                    {
                      value |= ((uint32_t)*src8++ << 16);

                      if (--remaining > 0)
                        {
                          value |= ((uint32_t)*src8++ << 24);
                        }
                    }
                }

              src = (const uint32_t *)src8;
              sam_putreg(priv, value, SAM_HSMCI_TDR_OFFSET);
            }
#else
          /* Transfer 32-bit aligned data */

          sam_putreg(priv, *src++, SAM_HSMCI_TDR_OFFSET);
          remaining -= sizeof(uint32_t);
#endif
        }
    }

  leave_critical_section(flags);
  sched_unlock();
  return OK;
}

/****************************************************************************
 * Name: sam_cancel
 *
 * Description:
 *   Cancel the data transfer setup of SDIO_RECVSETUP, SDIO_SENDSETUP,
 *   SDIO_DMARECVSETUP or SDIO_DMASENDSETUP.  This must be called to cancel
 *   the data transfer setup if, for some reason, you cannot perform the
 *   transfer.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int sam_cancel(struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  /* Disable all transfer- and event- related interrupts */

  sam_disablexfrints(priv);
  sam_disablewaitints(priv, 0);

  /* No data transfer */

  sam_notransfer(priv);

  /* Clearing (most) pending interrupt status by reading the status
   * register.
   */

  sam_getreg(priv, SAM_HSMCI_SR_OFFSET);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Make sure that the DMA is stopped (it will be stopped automatically
   * on normal transfers, but not necessarily when the transfer terminates
   * on an error condition.
   *
   * dmabusy == false let's the DMA callback know what is happening.
   */

  priv->dmabusy = false;
  sam_dmastop(priv->dma);

  /* Disable the DMA handshaking */

  sam_putreg(priv, 0, SAM_HSMCI_DMA_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: sam_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.
 *
 * Input Parameters:
 *   dev  - An instance of the SDIO device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int sam_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t sr;
  uint32_t pending;
  clock_t  watchtime;
  int32_t  timeout;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      timeout = HSMCI_LONGTIMEOUT;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
      return -ENOSYS;

    case MMCSD_NO_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      timeout = HSMCI_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the response (or timeout) */

  watchtime = clock_systime_ticks();
  for (; ; )
    {
      /* Did a Command-Response sequence termination event occur? */

      sr      = sam_getreg(priv, SAM_HSMCI_SR_OFFSET);
      pending = sr & priv->cmdrmask;

      if (pending != 0)
        {
          sam_cmdsample2(priv, SAMPLENDX_AT_WAKEUP, sr);
          sam_cmddump(priv);

          /* Yes.. Did the Command-Response sequence end with an error? */

          if ((pending & HSMCI_RESPONSE_ERRORS) != 0)
            {
              /* Yes.. Was the error some kind of timeout? */

              mcerr("ERROR: cmd: %08" PRIx32 " events: %08" PRIx32
                    " SR: %08" PRIx32 "\n",
                    cmd, priv->cmdrmask, sr);

              if ((pending & HSMCI_RESPONSE_TIMEOUT_ERRORS) != 0)
                {
                  /* Yes.. return a timeout error */

                  priv->wkupevent = SDIOWAIT_CMDDONE |
                                    SDIOWAIT_RESPONSEDONE |
                                    SDIOWAIT_TIMEOUT;
                  return -ETIMEDOUT;
                }
              else
                {
                  /* No.. return some generic I/O error */

                  priv->wkupevent = SDIOWAIT_CMDDONE |
                                    SDIOWAIT_RESPONSEDONE |
                                    SDIOWAIT_ERROR;
                  return -EIO;
                }
            }
          else
            {
              /* The Command-Response sequence ended with no error */

              priv->wkupevent = SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE;
              return OK;
            }
       }
      else if (clock_systime_ticks() - watchtime > timeout)
        {
          mcerr("ERROR: Timeout cmd: %08" PRIx32 " events: %08" PRIx32
                " SR: %08" PRIx32 "\n",
                cmd, priv->cmdrmask, sr);

          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return -ETIMEDOUT;
        }
    }
}

/****************************************************************************
 * Name: sam_recv*
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *   Rx  - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a failure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intact and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int sam_recvshort(struct sdio_dev_s *dev,
                         uint32_t cmd, uint32_t *rshort)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  int ret = OK;

  /* These responses could have CRC errors:
   *
   * R1  Command response (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit card status
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   *
   * R1b Identical to R1 with the additional busy signaling via the data
   *     line.
   *
   * R6  Published RCA Response (48-bit, SD card only)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Command index (0-63)
   *     39:8      bit31  - bit0   32-bit Argument Field, consisting of:
   *                               [31:16] New published RCA of card
   *                               [15:0]  Card status bits {23,22,19,12:0}
   *     7:1       bit6   - bit0   CRC7
   *     0         1               End bit
   *
   * But there is no parity on the R3 response and parity errors should
   * be ignored.
   *
   * R3  OCR (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Reserved
   *     39:8      bit31  - bit0   32-bit OCR register
   *     7:1       bit6   - bit0   Reserved
   *     0         1               End bit
   */

#ifdef CONFIG_DEBUG_FEATURES
  if (!rshort)
    {
      mcerr("ERROR: rshort=NULL\n");
      ret = -EINVAL;
    }

  /* Check that this is the correct response to this command */

  else if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08" PRIx32 "\n", cmd);
      ret = -EINVAL;
    }
  else
#endif

  /* Check for timeout errors */

  if ((priv->wkupevent & SDIOWAIT_TIMEOUT) != 0)
    {
      ret = -EINVAL;
    }

  /* Check for other errors */

  else if ((priv->wkupevent & SDIOWAIT_ERROR) != 0)
    {
      ret = -EIO;
    }

  /* Return the R1/R6 response */

  else if (rshort)
    {
      *rshort = sam_getreg(priv, SAM_HSMCI_RSPR0_OFFSET);
    }

  priv->wkupevent = 0;
  return ret;
}

static int sam_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                        uint32_t rlong[4])
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  int ret = OK;

  /* R2  CID, CSD register (136-bit)
   *     135       0               Start bit
   *     134       0               Transmission bit (0=from card)
   *     133:128   bit5   - bit0   Reserved
   *     127:1     bit127 - bit1   127-bit CID or CSD register
   *                               (including internal CRC)
   *     0         1               End bit
   */

#ifdef CONFIG_DEBUG_FEATURES
  /* Check that R1 is the correct response to this command */

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08" PRIx32 "\n", cmd);
      ret = -EINVAL;
    }
  else
#endif

  /* Check for timeout errors */

  if ((priv->wkupevent & SDIOWAIT_TIMEOUT) != 0)
    {
      mcerr("ERROR: timeout\n");
      ret = -EINVAL;
    }

  /* Check for other errors */

  else if ((priv->wkupevent & SDIOWAIT_ERROR) != 0)
    {
      mcerr("ERROR: Other error\n");
      ret = -EIO;
    }

  /* Return the long response */

  else if (rlong)
    {
      rlong[0] = sam_getreg(priv, SAM_HSMCI_RSPR0_OFFSET);
      rlong[1] = sam_getreg(priv, SAM_HSMCI_RSPR1_OFFSET);
      rlong[2] = sam_getreg(priv, SAM_HSMCI_RSPR2_OFFSET);
      rlong[3] = sam_getreg(priv, SAM_HSMCI_RSPR3_OFFSET);
    }

  priv->wkupevent = 0;
  return ret;
}

/* MMC responses not supported */

static int sam_recvnotimpl(struct sdio_dev_s *dev,
                           uint32_t cmd, uint32_t *rnotimpl)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  priv->wkupevent = 0;
  return -ENOSYS;
}

/****************************************************************************
 * Name: sam_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling either calling SDIO_DMARECVSETUP,
 *   SDIO_DMASENDSETUP, or or SDIO_WAITEVENT.  This is the recommended
 *   ordering:
 *
 *     SDIO_WAITENABLE:    Discard any pending interrupts, enable event(s)
 *                         of interest
 *     SDIO_DMARECVSETUP/
 *     SDIO_DMASENDSETUP:  Setup the logic that will trigger the event the
 *                         event(s) of interest
 *     SDIO_WAITEVENT:     Wait for the event of interest (which might
 *                         already have occurred)
 *
 *   This sequence should eliminate race conditions between the command/
 *   transfer setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDIO_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDIO_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_waitenable(struct sdio_dev_s *dev,
                           sdio_eventset_t eventset, uint32_t timeout)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t waitmask;

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  sam_disablewaitints(priv, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitmask = 0;
  if ((eventset & (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE)) != 0)
    {
      waitmask |= priv->cmdrmask;
    }

  /* Clear (most) pending interrupts by reading the status register.
   * No interrupts should be lost (assuming that interrupts were enabled
   * before sam_waitenable() was called).  Any interrupts that become
   * pending after this point must be valid event indications.
   */

  sam_getreg(priv, SAM_HSMCI_SR_OFFSET);

  /* Wait interrupts are configured here, but not enabled until
   * sam_eventwait() is called.  Why?  Because the XFRDONE interrupt is
   * always pending until start the data transfer.
   */

  sam_configwaitints(priv, waitmask, eventset);

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;
      int ret;

      /* Yes.. Handle a cornercase */

      if (!timeout)
        {
          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return;
        }

      /* Start the watchdog timer */

      delay = MSEC2TICK(timeout);
      ret   = wd_start(&priv->waitwdog, delay,
                       sam_eventtimeout, (wdparm_t)priv);
      if (ret < 0)
        {
           mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: sam_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when sam_eventwait
 *   returns.  SDIO_WAITEVENTS must be called again before sam_eventwait
 *   can be used again.
 *
 * Input Parameters:
 *   dev     - An instance of the SDIO device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means immediate
 *             timeout with no wait.  The timeout value is ignored if
 *             SDIOWAIT_TIMEOUT is not included in the waited-for eventset.
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  Should always
 *   be non-zero.  All events are disabled after the wait concludes.
 *
 ****************************************************************************/

static sdio_eventset_t sam_eventwait(struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  int ret;

#if 0 /* REVISIT */
  /* Since interrupts not been enabled to this point, any relevant events
   * are pending and should not yet have occurred.
   * REVISIT: Not true.  DMA interrupts are enabled.
   */

  DEBUGASSERT(priv->waitevents != 0 && priv->wkupevent == 0);
#endif

  /* Now enable event-related interrupts. If the events are pending, they
   * may happen immediately here before entering the loop.
   */

  sam_enableints(priv);

  /* Loop until the event (or the timeout occurs). Race conditions are
   * avoided by calling sam_waitenable prior to triggering the logic that
   * will cause the wait to terminate.  Under certain race conditions, the
   * waited-for may have already occurred before this function was called!
   */

  for (; ; )
    {
      /* Wait for an event in event set to occur.  If this the event has
       * already occurred, then the semaphore will already have been
       * incremented and there will be no wait.
       */

      ret = nxsem_wait_uninterruptible(&priv->waitsem);
      if (ret < 0)
        {
          /* Task canceled.  Cancel the wdog (assuming it was started),
           * disable all event, and return an SDIO error.
           */

          wd_cancel(&priv->waitwdog);
          sam_disablexfrints(priv);
          sam_disablewaitints(priv, SDIOWAIT_ERROR);
          return SDIOWAIT_ERROR;
        }

      wkupevent = priv->wkupevent;

      /* Check if the event has occurred.  When the event has occurred, then
       * evenset will be set to 0 and wkupevent will be set to a nonzero
       * value.  When wkupevent becomes non-zero, further interrupts will
       * have already been disabled.
       */

      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          break;
        }
    }

  sam_cmddump(priv);
  sam_xfrdump(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: sam_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in sam_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this methods.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_callbackenable(struct sdio_dev_s *dev,
                               sdio_eventset_t eventset)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  sam_callback(priv);
}

/****************************************************************************
 * Name: sam_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDIO_CALLBACKENABLE.
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

static int sam_registercallback(struct sdio_dev_s *dev,
                                worker_t callback, void *arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: sam_dmarecvsetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifndef HSCMI_NORXDMA
static int sam_dmarecvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                            size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regaddr;
  uint32_t memaddr;
  uint32_t regval;
  unsigned int blocksize;
  unsigned int nblocks;
  unsigned int offset;
  unsigned int i;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* 32-bit buffer alignment is required for DMA transfers */

  if (((uintptr_t)buffer & 3) != 0 || (buflen & 3) != 0)
    {
#ifdef CONFIG_SAMV7_HSMCI_UNALIGNED
      /* Fall back and do an unaligned, non-DMA transfer */

      return sam_recvsetup(dev, buffer, buflen);
#else
      /* Return the -EFAULT error. */

      return -EFAULT;
#endif
    }

  /* How many blocks?  That should have been saved by the sam_blocksetup()
   * method earlier.
   */

  regval    = sam_getreg(priv, SAM_HSMCI_BLKR_OFFSET);
  nblocks   = ((regval &  HSMCI_BLKR_BCNT_MASK) >>
               HSMCI_BLKR_BCNT_SHIFT);
  blocksize = ((regval &  HSMCI_BLKR_BLKLEN_MASK) >>
               HSMCI_BLKR_BLKLEN_SHIFT);

  DEBUGASSERT(nblocks > 0 && blocksize > 0 && (blocksize & 3) == 0);

  /* Physical address of the HSCMI source register, either the RDR (for
   * single transfers) or the first FIFO register, and the physical address
   * of the buffer in RAM.
   */

  offset  = (nblocks == 1 ? SAM_HSMCI_RDR_OFFSET : SAM_HSMCI_FIFO_OFFSET);
  regaddr = hsmci_regaddr(priv, offset);
  memaddr = (uintptr_t)buffer;

  /* Setup register sampling (only works for the case of nblocks == 1) */

  sam_xfrsampleinit(priv);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Set DMA for each block */

  for (i = 0; i < nblocks; i++)
    {
      /* Configure the RX DMA */

      sam_dmarxsetup(priv->dma, regaddr, memaddr, buflen);

      /* Update addresses for the next block */

      regaddr += sizeof(uint32_t);
      memaddr += blocksize;
    }

  /* Enable DMA handshaking */

  sam_putreg(priv,
             HSMCI_DMA_DMAEN | HSMCI_DMA_CHKSIZE, SAM_HSMCI_DMA_OFFSET);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_ENABLE);

  /* Start the DMA */

  priv->dmabusy   = true;
  priv->xfrbusy   = true;
  priv->txbusy    = false;
  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  sam_dmastart(priv->dma, sam_dmacallback, priv);

  /* Configure transfer-related interrupts.  Transfer interrupts are not
   * enabled until after the transfer is started with an SD command (i.e.,
   * at the beginning of sam_eventwait().
   */

  sam_xfrsample(priv, SAMPLENDX_AFTER_SETUP);
  sam_configxfrints(priv, HSMCI_DMARECV_INTS);
  return OK;
}
#endif

/****************************************************************************
 * Name: sam_dmasendsetup
 *
 * Description:
 *   Setup to perform a write DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For write transfers, this may mean
 *   flushing the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA into
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifndef HSCMI_NOTXDMA
static int sam_dmasendsetup(struct sdio_dev_s *dev,
                            const uint8_t *buffer, size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regaddr;
  uint32_t memaddr;
  uint32_t regval;
  unsigned int blocksize;
  unsigned int nblocks;
  unsigned int offset;
  unsigned int i;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* 32-bit buffer alignment is required for DMA transfers */

  if (((uintptr_t)buffer & 3) != 0 || (buflen & 3) != 0)
    {
#ifdef CONFIG_SAMV7_HSMCI_UNALIGNED
      /* Fall back and do an unaligned, non-DMA transfer */

      return sam_sendsetup(dev, buffer, buflen);
#else
      /* Return the -EFAULT error. */

      return -EFAULT;
#endif
    }

  /* How many blocks?  That should have been saved by the sam_blocksetup()
   * method earlier.
   */

  regval    = sam_getreg(priv, SAM_HSMCI_BLKR_OFFSET);
  nblocks   = ((regval &  HSMCI_BLKR_BCNT_MASK) >>
               HSMCI_BLKR_BCNT_SHIFT);
  blocksize = ((regval &  HSMCI_BLKR_BLKLEN_MASK) >>
               HSMCI_BLKR_BLKLEN_SHIFT);

  DEBUGASSERT(nblocks > 0 && blocksize > 0 && (blocksize & 3) == 0);

  /* Physical address of the HSCMI source register, either the TDR (for
   * single transfers) or the first FIFO register, and the physical address
   * of the buffer in RAM.
   */

  offset  = (nblocks == 1 ? SAM_HSMCI_TDR_OFFSET : SAM_HSMCI_FIFO_OFFSET);
  regaddr = hsmci_regaddr(priv, offset);
  memaddr = (uintptr_t)buffer;

  /* Setup register sampling (only works for the case of nblocks == 1) */

  sam_xfrsampleinit(priv);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Set DMA for each block */

  for (i = 0; i < nblocks; i++)
    {
      /* Configure the TX DMA */

      sam_dmatxsetup(priv->dma, regaddr, memaddr, buflen);

      /* Update addresses for the next block */

      regaddr += sizeof(uint32_t);
      memaddr += blocksize;
    }

  /* Enable DMA handshaking */

  sam_putreg(priv,
             HSMCI_DMA_DMAEN | HSMCI_DMA_CHKSIZE, SAM_HSMCI_DMA_OFFSET);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_ENABLE);

  /* Start the DMA */

  priv->dmabusy = true;
  priv->xfrbusy = true;
  priv->txbusy  = true;
  sam_dmastart(priv->dma, sam_dmacallback, priv);

  /* Configure transfer-related interrupts.  Transfer interrupts are not
   * enabled until after the transfer is started with an SD command (i.e.,
   * at the beginning of sam_eventwait().
   */

  sam_xfrsample(priv, SAMPLENDX_AFTER_SETUP);
  sam_configxfrints(priv, HSMCI_DMASEND_INTS);
  return OK;
}
#endif

/****************************************************************************
 * Name: sam_callback
 *
 * Description:
 *   Perform callback.
 *
 * Assumptions:
 *   This function does not execute in the context of an interrupt handler.
 *   It may be invoked on any user thread or scheduled on the work thread
 *   from an interrupt handler.
 *
 ****************************************************************************/

static void sam_callback(void *arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg;
  irqstate_t flags;
  int ret;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  mcinfo("Callback %p(%p) cbevents: %02x cdstatus: %02x\n",
        priv->callback, priv->cbarg, priv->cbevents, priv->cdstatus);

  flags = enter_critical_section();
  if (priv->callback)
    {
      /* Yes.. Check for enabled callback events */

      if ((priv->cdstatus & SDIO_STATUS_PRESENT) != 0)
        {
          /* Media is present.  Is the media inserted event enabled? */

          if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
            {
              /* No... return without performing the callback */

              leave_critical_section(flags);
              return;
            }
        }
      else
        {
          /* Media is not present.  Is the media eject event enabled? */

          if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
            {
              /* No... return without performing the callback */

              leave_critical_section(flags);
              return;
            }
        }

      /* Perform the callback, disabling further callbacks.  Of course, the
       * the callback can (and probably should) re-enable callbacks.
       */

      priv->cbevents = 0;

      /* This function is called either from (1) the context of the calling
       * thread or from the context of (2) card detection logic.  The
       * caller may or may not have interrupts disabled (we have them
       * disabled here!).
       *
       * So to minimize the possibility of recursive behavior and to assure
       * that callback is always performed outside of the interrupt handling
       * context and with interrupts enabled, the callback is always
       * performed on the lower priority work thread.
       */

      /* First cancel any existing work */

      ret = work_cancel(LPWORK, &priv->cbwork);
      if (ret < 0)
        {
          /* NOTE: Currently, work_cancel only returns success */

          mcerr("ERROR: Failed to cancel work: %d\n", ret);
        }

      mcinfo("Queuing callback to %p(%p)\n", priv->callback, priv->cbarg);
      ret = work_queue(LPWORK, &priv->cbwork, priv->callback,
                       priv->cbarg, 0);
      if (ret < 0)
        {
          /* NOTE: Currently, work_queue only returns success */

          mcerr("ERROR: Failed to schedule work: %d\n", ret);
        }
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SD for operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Value:
 *   A reference to an SDIO interface structure.  NULL is returned on
 *   failures.
 *
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno)
{
  struct sam_dev_s *priv;
  uint32_t pid;

  /* Select the slot and perform slot-specific initialization.  The
   * semantics here are bad.  There are three HSMCI peripherals that we
   * will treat as "slots."  In principle they could each peripheral could
   * support 4 slots, A-D.  However, selection of slots B, C, and D is
   * listed as "reserved" in the HSMCI register definitions.  So, at least
   * for now, an* HSMCI peripheral does correspond to a slot.
   */

  mcinfo("slotno: %d\n", slotno);

#ifdef CONFIG_SAMV7_HSMCI0
  if (slotno == 0)
    {
      /* Select HSMCI0 */

      priv = &g_hsmci0;

      /* Configure PIOs for 4-bit, wide-bus operation.  NOTE: (1) the chip
       * is capable of 8-bit wide bus operation but D4-D7 are not configured,
       * (2) any card detection PIOs must be set up in board-specific logic.
       *
       * REVISIT: What about Slot B?
       */

      sam_configgpio(GPIO_MCI0_DA0);   /* Data 0 of Slot A */
      sam_configgpio(GPIO_MCI0_DA1);   /* Data 1 of Slot A */
      sam_configgpio(GPIO_MCI0_DA2);   /* Data 2 of Slot A */
      sam_configgpio(GPIO_MCI0_DA3);   /* Data 3 of Slot A */
      sam_configgpio(GPIO_MCI0_CK);    /* Common SD clock */
      sam_configgpio(GPIO_MCI0_CDA);   /* Command/Response of Slot A */

      /* Enable the HSMCI0 peripheral clock.  This really should be done in
       * sam_enable (as well as disabling peripheral clocks in sam_disable().
       */

      sam_hsmci0_enableclk();

      /* For DMA channel selection */

      pid = SAM_PID_HSMCI0;
    }
  else
#endif
    {
      DEBUGPANIC();
      return NULL;
    }

  mcinfo("priv: %p base: %08" PRIx32 " hsmci: %d pid: %" PRId32 "\n",
         priv, priv->base, priv->hsmci, pid);

  /* Allocate a DMA channel */

  priv->dma = sam_dmachannel(0, DMA_FLAGS(pid));
  DEBUGASSERT(priv->dma);

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  sam_reset(&priv->dev);
  return &priv->dev;
}

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- possibly from an interrupt handler --
 *   in order to signal to the driver that a card has been inserted or
 *   removed from the slot
 *
 * Input Parameters:
 *   dev        - An instance of the SDIO driver device state structure.
 *   cardinslot - true is a card has been detected in the slot; false if a
 *                card has been removed from the slot.  Only transitions
 *                (inserted->removed or removed->inserted should be reported)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   May be called from an interrupt handler.
 *
 ****************************************************************************/

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  sdio_statset_t cdstatus;
  irqstate_t flags;

  /* Update card status.  Interrupts are disabled here because if we are
   * not called from an interrupt handler, then the following steps must
   * still be atomic.
   */

  flags = enter_critical_section();
  cdstatus = priv->cdstatus;
  if (cardinslot)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }

  mcinfo("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

  /* Perform any requested callback if the status has changed */

  if (cdstatus != priv->cdstatus)
    {
      sam_callback(priv);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report if the card in the slot is
 *   mechanically write protected.
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO driver device state structure.
 *   wrprotect - true is a card is write protected.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  irqstate_t flags;

  /* Update card status */

  flags = enter_critical_section();
  if (wrprotect)
    {
      priv->cdstatus |= SDIO_STATUS_WRPROTECTED;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_WRPROTECTED;
    }

  mcinfo("cdstatus: %02x\n", priv->cdstatus);
  leave_critical_section(flags);
}

#endif /* CONFIG_SAMV7_HSMCI */
