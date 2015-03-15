/****************************************************************************
 * arch/arm/src/sama5/sam_hsmci.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/mmcsd.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "sam_pio.h"
#include "sam_dmac.h"
#include "sam_periphclks.h"
#include "sam_memories.h"
#include "sam_hsmci.h"
#include "chip/sam_dmac.h"
#include "chip/sam_pmc.h"
#include "chip/sam_hsmci.h"
#include "chip/sam_pinmap.h"

#if defined(CONFIG_SAMA5_HSMCI0) || defined(CONFIG_SAMA5_HSMCI1) || \
    defined(CONFIG_SAMA5_HSMCI2)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if defined(ATSAMA5D3)
  /* The SAMA5D3 has three HSMCI blocks: HSMCI0-2.  HSMCI0 requires DMAC0
   * support, HSMCI1-2 require DMAC1 support.
   */

#  define HSMCI0_DMAC 0
#  define HSMCI1_DMAC 1
#  define HSMCI2_DMAC 1

#  if defined(CONFIG_SAMA5_HSMCI0) && !defined(CONFIG_SAMA5_DMAC0)
#    error "HSMCI0 support requires CONFIG_SAMA5_DMAC0"
#  endif

#  if defined(CONFIG_SAMA5_HSMCI1) && !defined(CONFIG_SAMA5_DMAC1)
#    error "HSMCI1 support requires CONFIG_SAMA5_DMAC1"
#  endif

#  if defined(CONFIG_SAMA5_HSMCI2) && !defined(CONFIG_SAMA5_DMAC1)
#    error "HSMCI2 support requires CONFIG_SAMA5_DMAC1"
#  endif

  /* System Bus Interfaces */

#  define HSMCI_SYSBUS_IF  DMACH_FLAG_PERIPHAHB_AHB_IF2
#  define MEMORY_SYSBUS_IF DMACH_FLAG_MEMAHB_AHB_IF0

#elif defined(ATSAMA5D4)
  /* The SAMA5D3 has two HSMCI blocks: HSMCI0-1.  They can be driven
   * either by XDMAC0 (secure) or XDMAC1 (unsecure).
   */

#  if !defined(CONFIG_SAMA5_XDMAC0) && !defined(CONFIG_SAMA5_XDMAC1)
#    error HSMCI0/1 require CONFIG_SAMA5_XDMAC0 and/or CONFIG_SAMA5_XDMAC1
#  endif

  /* If both XDMAC blocks are enabled, then we have to select which
   * used by each HSMCI block.
   */

#  if defined(CONFIG_SAMA5_HSMCI0)
#    if defined(CONFIG_SAMA5_HSMCI0_XDMAC0) && defined(CONFIG_SAMA5_XDMAC0)
#      define HSMCI0_DMAC 0
#    elif defined(CONFIG_SAMA5_HSMCI0_XDMAC1) && defined(CONFIG_SAMA5_XDMAC1)
#      define HSMCI0_DMAC 1
#    else
#      error No valid DMA configuration for HSMCI0
#    endif
#  endif

#  if defined(CONFIG_SAMA5_HSMCI1)
#    if defined(CONFIG_SAMA5_HSMCI1_XDMAC0) && defined(CONFIG_SAMA5_XDMAC0)
#      define HSMCI1_DMAC 0
#    elif defined(CONFIG_SAMA5_HSMCI0_XDMAC1) && defined(CONFIG_SAMA5_XDMAC1)
#      define HSMCI1_DMAC 1
#    else
#      error No valid DMA configuration for HSMCI1
#    endif
#  endif

  /* System Bus Interfaces
   *
   * HSMCI0 is on H32MX, APB1; HSMCI1 is on H32MX, APB0.  Both are
   * accessible on MATRIX IF1.
   *
   * Memory is available on either port 5 (IF0 for both XDMAC0 and 1) or
   * port 6 (IF1 for both XDMAC0 and 1).
   */

#  define HSMCI_SYSBUS_IF  DMACH_FLAG_PERIPHAHB_AHB_IF1
#  define MEMORY_SYSBUS_IF DMACH_FLAG_MEMAHB_AHB_IF0

#else
#  error Unrecognized SAMA5 architecture
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifndef CONFIG_SDIO_BLOCKSETUP
#  error "This driver requires CONFIG_SDIO_BLOCKSETUP"
#endif

#if !defined(CONFIG_DEBUG_FS) || !defined(CONFIG_DEBUG_VERBOSE)
#  undef CONFIG_SAMA5_HSMCI_CMDDEBUG
#  undef CONFIG_SAMA5_HSMCI_XFRDEBUG
#endif

#ifdef CONFIG_SAMA5_HSMCI_RDPROOF
#  ifdef CONFIG_SAMA5_HSMCI_WRPROOF
#    define HSMCU_PROOF_BITS (HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF)
#  else
#    define HSMCU_PROOF_BITS HSMCI_MR_RDPROOF
#  endif
#else
#  ifdef CONFIG_SAMA5_HSMCI_WRPROOF
#    define HSMCU_PROOF_BITS HSMCI_MR_WRPROOF
#  else
#    define HSMCU_PROOF_BITS (0)
#  endif
#endif

/* There is some unresolved issue with the SAMA5D3 DMA.  TX DMA is currently
 * disabled.
 */

#undef  HSCMI_NORXDMA            /* Define to disable RX DMA */
#undef  HSCMI_NOTXDMA            /* Define to disable TX DMA */

#ifdef ATSAMA5D3
#  define HSCMI_NOTXDMA 1        /* Disabled */
#endif

/* Timing */

#define HSMCI_CMDTIMEOUT         (100000)
#define HSMCI_LONGTIMEOUT        (0x7fffffff)

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
 *   HSMCI_INT_CSTOE         Completion signal time-out error (see HSMCI_CSTOR)
 *   HSMCI_INT_DTOE          Data time-out error (see HSMCI_DTOR)
 *   HSMCI_INT_DCRCE         Data CRC Error
 *   HSMCI_INT_RTOE          Response Time-out
 *   HSMCI_INT_RENDE         Response End Bit Error
 *   HSMCI_INT_RCRCE         Response CRC Error
 *   HSMCI_INT_RDIRE         Response Direction Error
 *   HSMCI_INT_RINDE         Response Index Error
 */

#define HSMCI_STATUS_ERRORS \
  ( HSMCI_INT_UNRE  | HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | \
    HSMCI_INT_DTOE  | HSMCI_INT_DCRCE | HSMCI_INT_RTOE    | HSMCI_INT_RENDE | \
    HSMCI_INT_RCRCE | HSMCI_INT_RDIRE | HSMCI_INT_RINDE )

/* Response errors:
 *
 *   HSMCI_INT_CSTOE         Completion signal time-out error (see HSMCI_CSTOR)
 *   HSMCI_INT_RTOE          Response Time-out
 *   HSMCI_INT_RENDE         Response End Bit Error
 *   HSMCI_INT_RCRCE         Response CRC Error
 *   HSMCI_INT_RDIRE         Response Direction Error
 *   HSMCI_INT_RINDE         Response Index Error
 */

#define HSMCI_RESPONSE_ERRORS \
  ( HSMCI_INT_CSTOE | HSMCI_INT_RTOE  | HSMCI_INT_RENDE   | HSMCI_INT_RCRCE | \
    HSMCI_INT_RDIRE | HSMCI_INT_RINDE )
#define HSMCI_RESPONSE_NOCRC_ERRORS \
  ( HSMCI_INT_CSTOE | HSMCI_INT_RTOE  | HSMCI_INT_RENDE   | HSMCI_INT_RDIRE | \
    HSMCI_INT_RINDE )
#define HSMCI_RESPONSE_TIMEOUT_ERRORS \
  ( HSMCI_INT_CSTOE | HSMCI_INT_RTOE )

/* Data transfer errors:
 *
 *   HSMCI_INT_UNRE          Data transmit underrun
 *   HSMCI_INT_OVRE          Data receive overrun
 *   HSMCI_INT_BLKOVRE       DMA receive block overrun error
 *   HSMCI_INT_CSTOE         Completion signal time-out error (see HSMCI_CSTOR)
 *   HSMCI_INT_DTOE          Data time-out error (see HSMCI_DTOR)
 *   HSMCI_INT_DCRCE         Data CRC Error
 */

#define HSMCI_DATA_ERRORS \
  ( HSMCI_INT_UNRE  | HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | \
    HSMCI_INT_DTOE  | HSMCI_INT_DCRCE )

#define HSMCI_DATA_TIMEOUT_ERRORS \
  ( HSMCI_INT_CSTOE | HSMCI_INT_DTOE )

#define HSMCI_DATA_RECV_ERRORS \
  ( HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | HSMCI_INT_DTOE | \
    HSMCI_INT_DCRCE )

#define HSMCI_DATA_DMASEND_ERRORS \
  ( HSMCI_INT_UNRE  | HSMCI_INT_CSTOE | HSMCI_INT_DTOE    | HSMCI_INT_DCRCE )

/* Data transfer status and interrupt mask bits.
 *
 * The XFRDONE flag in the HSMCI_SR indicates exactly when the read or
 * write sequence is finished.
 *
 *   0: A transfer is in progress.
 *   1: Command register is ready to operate and the data bus is in the idle state.
 *
 * DMADONE: DMA Transfer done
 *
 *   0: DMA buffer transfer has not completed since the last read of HSMCI_SR register.
 *   1: DMA buffer transfer has completed.
 */

#define HSMCI_RECV_INTS \
  ( HSMCI_DATA_RECV_ERRORS | HSMCI_INT_RXRDY)
#define HSMCI_DMARECV_INTS \
  ( HSMCI_DATA_RECV_ERRORS | HSMCI_INT_XFRDONE /* | HSMCI_INT_DMADONE */ )
#define HSMCI_DMASEND_INTS \
  ( HSMCI_DATA_DMASEND_ERRORS | HSMCI_INT_XFRDONE /* | HSMCI_INT_DMADONE */ )

/* Event waiting interrupt mask bits.
 *
 * CMDRDY (Command Ready):
 *
 *   0: A command is in progress
 *   1: The last command has been sent.  The CMDRDY flag is released 8 bits
 *     after the end of the card response. Cleared when writing in the HSMCI_CMDR
 */

#define HSMCI_CMDRESP_INTS \
  ( HSMCI_RESPONSE_ERRORS | HSMCI_INT_CMDRDY )
#define HSMCI_CMDRESP_NOCRC_INTS \
  ( HSMCI_RESPONSE_NOCRC_ERRORS | HSMCI_INT_CMDRDY )

/* Register logging support */

#ifdef CONFIG_SAMA5_HSMCI_XFRDEBUG
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

#ifdef CONFIG_SAMA5_HSMCI_CMDDEBUG
#  define SAMPLENDX_AFTER_CMDR      0
#  define SAMPLENDX_AT_WAKEUP       1
#  define DEBUG_NCMDSAMPLES         2
#endif

/* Some semi-standard definitions */

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* Register logging support */

#if defined(CONFIG_SAMA5_HSMCI_XFRDEBUG) || defined(CONFIG_SAMA5_HSMCI_CMDDEBUG)
struct sam_hsmciregs_s
{
  uint32_t mr;    /* Mode Register */
  uint32_t dtor;  /* Data Timeout Register */
  uint32_t sdcr;  /* SD/SDIO Card Register */
  uint32_t argr;  /* Argument Register */
  uint32_t blkr;  /* Block Register */
  uint32_t cstor; /* Completion Signal Timeout Register */
  uint32_t rsp0;  /* Response Register 0 */
  uint32_t rsp1;  /* Response Register 1 */
  uint32_t rsp2;  /* Response Register 2 */
  uint32_t rsp3;  /* Response Register 3 */
  uint32_t sr;    /* Status Register */
  uint32_t imr;   /* Interrupt Mask Register */
  uint32_t dma;   /* DMA Configuration Register */
  uint32_t cfg;   /* Configuration Register */
  uint32_t wpmr;  /* Write Protection Mode Register */
  uint32_t wpsr;  /* Write Protection Status Register */
};
#endif

#ifdef CONFIG_SAMA5_HSMCI_XFRDEBUG
struct sam_xfrregs_s
{
  struct sam_hsmciregs_s hsmci;
#ifdef CONFIG_DEBUG_DMA
  struct sam_dmaregs_s  dma;
#endif
};
#endif

/* This structure defines the state of the SAMA5 HSMCI interface */

struct sam_dev_s
{
  struct sdio_dev_s  dev;        /* Standard, base SDIO interface */

  /* SAMA5-specific extensions */
  /* Event support */

  sem_t              waitsem;    /* Implements event waiting */
  sdio_eventset_t    waitevents; /* Set of events to be waited for */
  uint32_t           base;       /* HSMCI register base address */
  uint32_t           waitmask;   /* Interrupt enables for event waiting */
  uint32_t           cmdrmask;   /* Interrupt enables for this particular cmd/response */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  WDOG_ID            waitwdog;   /* Watchdog that handles event timeouts */
  uint8_t            hsmci;      /* HSMCI (0, 1, or 2) */
  volatile bool      dmabusy;    /* TRUE: DMA transfer is in progress */
  volatile bool      xfrbusy;    /* TRUE: Transfer is in progress */
  volatile bool      txbusy;     /* TRUE: TX transfer is in progress (for delay calculation) */

  /* Callback support */

  uint8_t            cdstatus;   /* Card status */
  sdio_eventset_t    cbevents;   /* Set of events to be cause callbacks */
  worker_t           callback;   /* Registered callback function */
  void              *cbarg;      /* Registered callback argument */
  struct work_s      cbwork;     /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t           xfrmask;    /* Interrupt enables for data transfer */

  /* Interrupt mode data transfer support */

  uint32_t          *buffer;     /* Address of current R/W buffer */
  ssize_t            remaining;  /* Number of bytes remaining in the transfer */

  /* DMA data transfer support */

  bool               widebus;    /* Required for DMA support */
  DMA_HANDLE         dma;        /* Handle for DMA channel */

  /* Debug stuff */

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
   bool              wrlast;     /* Last was a write */
   uint32_t          addrlast;   /* Last address */
   uint32_t          vallast;    /* Last value */
   int               ntimes;     /* Number of times */
#endif

  /* Register logging support */

#if defined(CONFIG_SAMA5_HSMCI_CMDDEBUG) && defined(CONFIG_SAMA5_HSMCI_XFRDEBUG)
   bool              xfrinitialized;
   bool              cmdinitialized;
#endif
#ifdef CONFIG_SAMA5_HSMCI_XFRDEBUG
  uint8_t            smplset;
  struct sam_xfrregs_s xfrsamples[DEBUG_NDMASAMPLES];
#endif
#ifdef CONFIG_SAMA5_HSMCI_CMDDEBUG
  struct sam_hsmciregs_s cmdsamples[DEBUG_NCMDSAMPLES];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void sam_takesem(struct sam_dev_s *priv);
#define     sam_givesem(priv) (sem_post(&priv->waitsem))

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
static bool sam_checkreg(struct sam_dev_s *priv, bool wr,
              uint32_t value, uint32_t address);
#else
# define    sam_checkreg(priv,wr,value,address) (false)
#endif

static inline uint32_t sam_getreg(struct sam_dev_s *priv,
                  unsigned int offset);
static inline void sam_putreg(struct sam_dev_s *priv, uint32_t value,
                  unsigned int offset);

static inline void sam_configwaitints(struct sam_dev_s *priv, uint32_t waitmask,
              sdio_eventset_t waitevents);
static void sam_disablewaitints(struct sam_dev_s *priv, sdio_eventset_t wkupevent);
static inline void sam_configxfrints(struct sam_dev_s *priv, uint32_t xfrmask);
static void sam_disablexfrints(struct sam_dev_s *priv);
static inline void sam_enableints(struct sam_dev_s *priv);

static inline void sam_disable(struct sam_dev_s *priv);
static inline void sam_enable(struct sam_dev_s *priv);

/* Register Sampling ********************************************************/

#if defined(CONFIG_SAMA5_HSMCI_XFRDEBUG) || defined(CONFIG_SAMA5_HSMCI_CMDDEBUG)
static void sam_hsmcisample(struct sam_dev_s *priv,
              struct sam_hsmciregs_s *regs);
static void sam_hsmcidump(struct sam_dev_s *priv,
              struct sam_hsmciregs_s *regs, const char *msg);
#endif

#ifdef CONFIG_SAMA5_HSMCI_XFRDEBUG
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

#ifdef CONFIG_SAMA5_HSMCI_CMDDEBUG
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
static inline uintptr_t hsmci_physregaddr(struct sam_dev_s *priv,
              unsigned int offset);

/* Data Transfer Helpers ****************************************************/

static void sam_eventtimeout(int argc, uint32_t arg);
static void sam_endwait(struct sam_dev_s *priv, sdio_eventset_t wkupevent);
static void sam_endtransfer(struct sam_dev_s *priv, sdio_eventset_t wkupevent);
static void sam_notransfer(struct sam_dev_s *priv);

/* Interrupt Handling *******************************************************/

static int  sam_hsmci_interrupt(struct sam_dev_s *priv);
#ifdef CONFIG_SAMA5_HSMCI0
static int  sam_hsmci0_interrupt(int irq, void *context);
#endif
#ifdef CONFIG_SAMA5_HSMCI1
static int  sam_hsmci1_interrupt(int irq, void *context);
#endif
#ifdef CONFIG_SAMA5_HSMCI2
static int  sam_hsmci2_interrupt(int irq, void *context);
#endif

/* SDIO interface methods ***************************************************/

/* Initialization/setup */

static void sam_reset(FAR struct sdio_dev_s *dev);
static uint8_t sam_status(FAR struct sdio_dev_s *dev);
static void sam_widebus(FAR struct sdio_dev_s *dev, bool enable);
static void sam_clock(FAR struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  sam_attach(FAR struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  sam_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);
static void sam_blocksetup(FAR struct sdio_dev_s *dev, unsigned int blocklen,
              unsigned int nblocks);
static int  sam_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
              size_t nbytes);
static int  sam_sendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer,
              size_t nbytes);
static int  sam_cancel(FAR struct sdio_dev_s *dev);
static int  sam_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd);
static int  sam_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  sam_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  sam_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rnotimpl);

/* EVENT handler */

static void sam_waitenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static sdio_eventset_t
            sam_eventwait(FAR struct sdio_dev_s *dev, uint32_t timeout);
static void sam_callbackenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  sam_registercallback(FAR struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_SDIO_DMA
static bool sam_dmasupported(FAR struct sdio_dev_s *dev);
#endif
#ifndef HSCMI_NORXDMA
static int  sam_dmarecvsetup(FAR struct sdio_dev_s *dev,
              FAR uint8_t *buffer, size_t buflen);
#endif
#ifndef HSCMI_NOTXDMA
static int  sam_dmasendsetup(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void sam_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Callbacks */

static const struct sdio_dev_s g_callbacks =
{
  .reset            = sam_reset,
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
  .recvR1           = sam_recvshort,
  .recvR2           = sam_recvlong,
  .recvR3           = sam_recvshort,
  .recvR4           = sam_recvnotimpl,
  .recvR5           = sam_recvnotimpl,
  .recvR6           = sam_recvshort,
  .recvR7           = sam_recvshort,
  .waitenable       = sam_waitenable,
  .eventwait        = sam_eventwait,
  .callbackenable   = sam_callbackenable,
  .registercallback = sam_registercallback,
#ifdef CONFIG_SDIO_DMA
  .dmasupported     = sam_dmasupported,
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
};

/* Pre-allocate memory for each HSMCI device */

#ifdef CONFIG_SAMA5_HSMCI0
static struct sam_dev_s g_hsmci0;
#endif
#ifdef CONFIG_SAMA5_HSMCI1
static struct sam_dev_s g_hsmci1;
#endif
#ifdef CONFIG_SAMA5_HSMCI2
static struct sam_dev_s g_hsmci2;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: sam_takesem
 *
 * Description:
 *   Take the wait semaphore (handling false alarm wakeups due to the receipt
 *   of signals).
 *
 * Input Parameters:
 *   dev - Instance of the SDIO device driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_takesem(struct sam_dev_s *priv)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&priv->waitsem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

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

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
static bool sam_checkreg(struct sam_dev_s *priv, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == priv->wrlast &&     /* Same kind of access? */
      value   == priv->vallast &&  /* Same value? */
      address == priv->addrlast)  /* Same address? */
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

          lldbg("...[Repeats %d times]...\n", priv->ntimes);
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

static inline uint32_t sam_getreg(struct sam_dev_s *priv, unsigned int offset)
{
  uint32_t address = priv->base + offset;
  uint32_t value = getreg32(address);

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
  if (sam_checkreg(priv, false, value, address))
    {
      lldbg("%08x->%08x\n", address, value);
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

#ifdef CONFIG_SAMA5_HSMCI_REGDEBUG
  if (sam_checkreg(priv, true, value, address))
    {
      lldbg("%08x<-%08x\n", address, value);
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

  flags = irqsave();
  priv->waitevents = waitevents;
  priv->wkupevent  = 0;
  priv->waitmask   = waitmask;
  irqrestore(flags);
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

  flags            = irqsave();
  priv->waitevents = 0;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = 0;
  sam_putreg(priv, ~priv->xfrmask, SAM_HSMCI_IDR_OFFSET);
  irqrestore(flags);
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

static inline void sam_configxfrints(struct sam_dev_s *priv, uint32_t xfrmask)
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
  irqstate_t flags = irqsave();
  priv->xfrmask = 0;
  sam_putreg(priv, ~priv->waitmask, SAM_HSMCI_IDR_OFFSET);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_enableints
 *
 * Description:
 *   Enable the previously configured HSMCI interrupts needed to suport the
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
 * Register Sampling
 ****************************************************************************/

/****************************************************************************
 * Name: sam_hsmcisample
 *
 * Description:
 *   Sample HSMCI registers
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_HSMCI_XFRDEBUG) || defined(CONFIG_SAMA5_HSMCI_CMDDEBUG)
static void sam_hsmcisample(struct sam_dev_s *priv,
                            struct sam_hsmciregs_s *regs)
{
  regs->mr    = sam_getreg(priv, SAM_HSMCI_MR_OFFSET);
  regs->dtor  = sam_getreg(priv, SAM_HSMCI_DTOR_OFFSET);
  regs->sdcr  = sam_getreg(priv, SAM_HSMCI_SDCR_OFFSET);
  regs->argr  = sam_getreg(priv, SAM_HSMCI_ARGR_OFFSET);
  regs->blkr  = sam_getreg(priv, SAM_HSMCI_BLKR_OFFSET);
  regs->cstor = sam_getreg(priv, SAM_HSMCI_CSTOR_OFFSET);
  regs->rsp0  = sam_getreg(priv, SAM_HSMCI_RSPR0_OFFSET);
  regs->rsp1  = sam_getreg(priv, SAM_HSMCI_RSPR1_OFFSET);
  regs->rsp2  = sam_getreg(priv, SAM_HSMCI_RSPR2_OFFSET);
  regs->rsp3  = sam_getreg(priv, SAM_HSMCI_RSPR3_OFFSET);
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

#if defined(CONFIG_SAMA5_HSMCI_XFRDEBUG) || defined(CONFIG_SAMA5_HSMCI_CMDDEBUG)
static void sam_hsmcidump(struct sam_dev_s *priv,
                          struct sam_hsmciregs_s *regs, const char *msg)
{
  fdbg("HSMCI Registers: %s\n", msg);
  fdbg("      MR[%08x]: %08x\n", priv->base + SAM_HSMCI_MR_OFFSET,    regs->mr);
  fdbg("    DTOR[%08x]: %08x\n", priv->base + SAM_HSMCI_DTOR_OFFSET,  regs->dtor);
  fdbg("    SDCR[%08x]: %08x\n", priv->base + SAM_HSMCI_SDCR_OFFSET,  regs->sdcr);
  fdbg("    ARGR[%08x]: %08x\n", priv->base + SAM_HSMCI_ARGR_OFFSET,  regs->argr);
  fdbg("    BLKR[%08x]: %08x\n", priv->base + SAM_HSMCI_BLKR_OFFSET,  regs->blkr);
  fdbg("   CSTOR[%08x]: %08x\n", priv->base + SAM_HSMCI_CSTOR_OFFSET, regs->cstor);
  fdbg("   RSPR0[%08x]: %08x\n", priv->base + SAM_HSMCI_RSPR0_OFFSET, regs->rsp0);
  fdbg("   RSPR1[%08x]: %08x\n", priv->base + SAM_HSMCI_RSPR1_OFFSET, regs->rsp1);
  fdbg("   RSPR2[%08x]: %08x\n", priv->base + SAM_HSMCI_RSPR2_OFFSET, regs->rsp2);
  fdbg("   RSPR3[%08x]: %08x\n", priv->base + SAM_HSMCI_RSPR3_OFFSET, regs->rsp3);
  fdbg("      SR[%08x]: %08x\n", priv->base + SAM_HSMCI_SR_OFFSET,    regs->sr);
  fdbg("     IMR[%08x]: %08x\n", priv->base + SAM_HSMCI_IMR_OFFSET,   regs->imr);
  fdbg("     DMA[%08x]: %08x\n", priv->base + SAM_HSMCI_DMA_OFFSET,   regs->dma);
  fdbg("     CFG[%08x]: %08x\n", priv->base + SAM_HSMCI_CFG_OFFSET,   regs->cfg);
  fdbg("    WPMR[%08x]: %08x\n", priv->base + SAM_HSMCI_WPMR_OFFSET,  regs->wpmr);
  fdbg("    WPSR[%08x]: %08x\n", priv->base + SAM_HSMCI_WPSR_OFFSET,  regs->wpsr);
}
#endif

/****************************************************************************
 * Name: sam_xfrsample
 *
 * Description:
 *   Sample HSMCI/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_HSMCI_XFRDEBUG
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

#ifdef CONFIG_SAMA5_HSMCI_XFRDEBUG
static void sam_xfrsampleinit(struct sam_dev_s *priv)
{
  priv->smplset = 0;
  memset(priv->xfrsamples, 0xff,
         DEBUG_NDMASAMPLES * sizeof(struct sam_xfrregs_s));

#ifdef CONFIG_SAMA5_HSMCI_CMDDEBUG
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

#ifdef CONFIG_SAMA5_HSMCI_XFRDEBUG
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
      fdbg("%s: Not collected\n", msg);
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

#ifdef CONFIG_SAMA5_HSMCI_XFRDEBUG
static void  sam_xfrdump(struct sam_dev_s *priv)
{
#ifdef CONFIG_SAMA5_HSMCI_CMDDEBUG
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
#ifdef CONFIG_SAMA5_HSMCI_CMDDEBUG
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

#ifdef CONFIG_SAMA5_HSMCI_CMDDEBUG
static void sam_cmdsampleinit(struct sam_dev_s *priv)
{
  memset(priv->cmdsamples, 0xff,
         DEBUG_NCMDSAMPLES * sizeof(struct sam_hsmciregs_s));

#ifdef CONFIG_SAMA5_HSMCI_XFRDEBUG
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

#ifdef CONFIG_SAMA5_HSMCI_CMDDEBUG
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
 *   Dump all comand/response register data
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_HSMCI_CMDDEBUG
static void sam_cmddump(struct sam_dev_s *priv)
{
#ifdef CONFIG_SAMA5_HSMCI_XFRDEBUG
  if (priv->cmdinitialized)
#endif
    {
      sam_hsmcidump(priv, &priv->cmdsamples[SAMPLENDX_AFTER_CMDR],
                    "After command setup");
      sam_hsmcidump(priv, &priv->cmdsamples[SAMPLENDX_AT_WAKEUP],
                    "After wakeup");
#ifdef CONFIG_SAMA5_HSMCI_XFRDEBUG
      priv->cmdinitialized = false;
#endif
    }
}
#endif

/****************************************************************************
 * DMA Helpers
 ****************************************************************************/

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

      /* Terminate the transfer with an I/O error in the event of a DMA failure */

      if (result < 0)
        {
          wkupevent = (result == -ETIMEDOUT ? SDIOWAIT_TIMEOUT : SDIOWAIT_ERROR);
          flldbg("ERROR: DMA failed: result=%d wkupevent=%04x\n", result, wkupevent);

          /* sam_endtransfer will terminate the transfer and wait up the waiting
           * client in this case.
           */

          sam_endtransfer(priv, wkupevent);
        }

      /* The DMA completed without error.  Wake-up the waiting client if (1) both the
       * HSMCI and DMA completion events, and (2) There is a client waiting for
       * this event.
       *
       * If the HSMCI transfer event has already completed, it must have completed
       * successfully (because the DMA was not cancelled).  sam_endtransfer() should
       * have already received the SDIOWAIT_TRANSFERDONE event, but this event would
       * not yet have been recorded.  We need to post the SDIOWAIT_TRANSFERDONE
       * again in this case here.
       *
       * The timeout will remain active until sam_endwait() is eventually called
       * so we should not have any concern about hangs if the HSMCI transfer never
       * completed.
       */

      else if (!priv->xfrbusy && (priv->waitevents & SDIOWAIT_TRANSFERDONE) != 0)
        {
          /* Okay.. wake up any waiting threads */

          sam_endwait(priv, SDIOWAIT_TRANSFERDONE);
        }
    }
}

/****************************************************************************
 * Name: hsmci_physregaddr
 *
 * Description:
 *   Return the physical address of an HSMCI register
 *
 ****************************************************************************/

static inline uintptr_t hsmci_physregaddr(struct sam_dev_s *priv,
                                          unsigned int offset)
{
  return sam_physregaddr(priv->base + offset);
}

/****************************************************************************
 * Data Transfer Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam_eventtimeout
 *
 * Description:
 *   The watchdog timeout setup when the event wait start has expired without
 *   any other waited-for event occurring.
 *
 * Input Parameters:
 *   argc   - The number of arguments (should be 1)
 *   arg    - The argument (state structure reference cast to uint32_t)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sam_eventtimeout(int argc, uint32_t arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg;

  DEBUGASSERT(argc == 1 && priv != NULL);
  sam_xfrsample((struct sam_dev_s *)arg, SAMPLENDX_TIMEOUT);

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
      flldbg("ERROR: Timeout\n");
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

  (void)wd_cancel(priv->waitwdog);

  /* Disable event-related interrupts and save wakeup event */

  sam_disablewaitints(priv, wkupevent);

  /* Wake up the waiting thread */

  sam_givesem(priv);
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
   * so we should not have any concern about hangs if the DMA never completes.
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
 * Interrupt Handling
 ****************************************************************************/

/****************************************************************************
 * Name: sam_hsmci_interrupt
 *
 * Description:
 *   HSMCI interrupt handler
 *
 * Input Parameters:
 *   irq - IRQ number of the interrupts
 *   context - Saved machine context at the time of the interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sam_hsmci_interrupt(struct sam_dev_s *priv)
{
  uint32_t sr;
  uint32_t enabled;
  uint32_t pending;

  /* Loop while there are pending interrupts. */

  for (;;)
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

              flldbg("ERROR: enabled: %08x pending: %08x\n", enabled, pending);
              if ((pending & HSMCI_DATA_TIMEOUT_ERRORS) != 0)
                {
                  /* Yes.. Terminate with a timeout. */

                  sam_endtransfer(priv, SDIOWAIT_TRANSFERDONE|SDIOWAIT_TIMEOUT);
                }
              else
                {
                  /* No..  Terminate with an I/O error. */

                  sam_endtransfer(priv, SDIOWAIT_TRANSFERDONE|SDIOWAIT_ERROR);
                }
            }

          /* No, If RXRDY is enabled, then we are doing a non-DMA receive.
           * We need to transfer word(s) from the RDR register to the user
           * buffer.
           */

          else if ((pending & HSMCI_INT_RXRDY) != 0)
            {
              /* Interrupt mode data transfer support */

              DEBUGASSERT(!priv->dmabusy && priv->xfrbusy && !priv->txbusy);
              DEBUGASSERT(priv->buffer && priv->remaining > 0);

              *priv->buffer++  = sam_getreg(priv, SAM_HSMCI_RDR_OFFSET);
              priv->remaining -= sizeof(uint32_t);

              /* Are we finished? */

              if (priv->remaining <= 0)
                {
                  /* Yes.. End the transfer */

                  priv->buffer    = NULL;
                  priv->remaining = 0;
                  sam_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
                }
            }

          /* Otherwise it must be a DMA transfer that completed successfully */

          else
            {
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

                  fllvdbg("ERROR: events: %08x SR: %08x\n",
                          priv->cmdrmask, enabled);

                  if ((pending & HSMCI_RESPONSE_TIMEOUT_ERRORS) != 0)
                    {
                      /* Yes.. signal a timeout error */

                      wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE|SDIOWAIT_TIMEOUT;
                    }
                  else
                    {
                      /* No.. signal some generic I/O error */

                      wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE|SDIOWAIT_ERROR;
                    }
                }
              else
               {
                  /* The Command-Response sequence ended with no error */

                      wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE;
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
 * Name: sam_hsmci0_interrupt, sam_hsmci1_interrupt, and sam_hsmci2_interrupt
 *
 * Description:
 *   HSMCI interrupt handler
 *
 * Input Parameters:
 *   irq - IRQ number of the interrupts
 *   context - Saved machine context at the time of the interrupt
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_HSMCI0
static int sam_hsmci0_interrupt(int irq, void *context)
{
  return sam_hsmci_interrupt(&g_hsmci0);
}
#endif

#ifdef CONFIG_SAMA5_HSMCI1
static int sam_hsmci1_interrupt(int irq, void *context)
{
  return sam_hsmci_interrupt(&g_hsmci1);
}
#endif

#ifdef CONFIG_SAMA5_HSMCI2
static int sam_hsmci2_interrupt(int irq, void *context)
{
  return sam_hsmci_interrupt(&g_hsmci2);
}
#endif

/****************************************************************************
 * SDIO Interface Methods
 ****************************************************************************/
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

static void sam_reset(FAR struct sdio_dev_s *dev)
{
  FAR struct sam_dev_s *priv = (FAR struct sam_dev_s *)dev;
  irqstate_t flags;

  /* Reset the MCI */

  flags = irqsave();
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

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */
  priv->dmabusy    = false;  /* No DMA in progress */
  wd_cancel(priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  priv->widebus    = false;  /* Required for DMA support */
  irqrestore(flags);
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

static uint8_t sam_status(FAR struct sdio_dev_s *dev)
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

static void sam_widebus(FAR struct sdio_dev_s *dev, bool wide)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regval;

  /* Set 1-bit or 4-bit bus by configuring the SDCBUS field of the SDCR register */

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

static void sam_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regval;
  bool enable = true;

  /* Fetch the current mode register and mask out the clkdiv+clockodd (and pwsdiv) */

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

static int sam_attach(FAR struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  xcpt_t handler;
  int irq;
  int ret;

  /* Select the handler and IRQ */

#ifdef CONFIG_SAMA5_HSMCI0
  if (priv->hsmci == 0)
    {
      handler = sam_hsmci0_interrupt;
      irq     = SAM_IRQ_HSMCI0;
    }
  else
#endif
#ifdef CONFIG_SAMA5_HSMCI1
  if (priv->hsmci == 1)
    {
      handler = sam_hsmci1_interrupt;
      irq     = SAM_IRQ_HSMCI1;
    }
  else
#endif
#ifdef CONFIG_SAMA5_HSMCI2
  if (priv->hsmci == 2)
    {
      handler = sam_hsmci2_interrupt;
      irq     = SAM_IRQ_HSMCI2;
    }
  else
#endif
    {
      DEBUGPANIC();
      return -EINVAL; /* Shouldn't happen */
    }

  /* Attach the HSMCI interrupt handler */

  ret = irq_attach(irq, handler);
  if (ret == OK)
    {

      /* Disable all interrupts at the HSMCI controller and clear (most) static
       * interrupt flags by reading the status register.
       */

      sam_putreg(priv, 0xffffffff, SAM_HSMCI_IDR_OFFSET);
      (void)sam_getreg(priv, SAM_HSMCI_SR_OFFSET);

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

static int sam_sendcmd(FAR struct sdio_dev_s *dev,
                       uint32_t cmd, uint32_t arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev;
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
      regval |= (HSMCI_CMDR_RSPTYP_48BIT | HSMCI_CMDR_MAXLAT);
      break;

    case MMCSD_R1B_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= (HSMCI_CMDR_RSPTYP_R1B | HSMCI_CMDR_MAXLAT);
      break;

    /* 48-bit response without CRC */

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_NOCRC_INTS;
      regval |= (HSMCI_CMDR_RSPTYP_48BIT | HSMCI_CMDR_MAXLAT);
      break;

    /* 136-bit response with CRC */

    case MMCSD_R2_RESPONSE:
      priv->cmdrmask = HSMCI_CMDRESP_INTS;
      regval |= (HSMCI_CMDR_RSPTYP_136BIT | HSMCI_CMDR_MAXLAT);
      break;
    }

  /* 'OR' in data transfer related bits */

  switch (cmd & MMCSD_DATAXFR_MASK)
    {
#if 0 /* No MMC support */
    case MMCSD_RDSTREAM: /* MMC Read stream */
      regval |= (HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRTYP_STREAM | HSMCI_CMDR_TRDIR_READ);
      break;

    case MMCSD_WRSTREAM: /* MMC Write stream */
      regval |= (HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRTYP_STREAM | HSMCI_CMDR_TRDIR_WRITE);
      break;
#endif

    case MMCSD_RDDATAXFR: /* Read block transfer */
      regval |= (HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRDIR_READ);
      regval |= (cmd & MMCSD_MULTIBLOCK) ? HSMCI_CMDR_TRTYP_MULTIPLE : HSMCI_CMDR_TRTYP_SINGLE;
      break;

    case MMCSD_WRDATAXFR: /* Write block transfer */
      regval |= (HSMCI_CMDR_TRCMD_START | HSMCI_CMDR_TRDIR_WRITE);
      regval |= (cmd & MMCSD_MULTIBLOCK) ? HSMCI_CMDR_TRTYP_MULTIPLE : HSMCI_CMDR_TRTYP_SINGLE;
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

  fvdbg("cmd: %08x arg: %08x regval: %08x\n", cmd, arg, regval);
  sam_putreg(priv, regval, SAM_HSMCI_CMDR_OFFSET);
  sam_cmdsample1(priv, SAMPLENDX_AFTER_CMDR);
  return OK;
}

/****************************************************************************
 * Name: sam_blocksetup
 *
 * Description:
 *   Some hardward needs to be informed of the selected blocksize.
 *
 * Input Parameters:
 *   dev      - An instance of the SDIO device interface
 *   blocklen - The selected block size.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_blocksetup(FAR struct sdio_dev_s *dev, unsigned int blocklen,
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
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally, SDIO_WAITEVENT
 *   will be called to receive the indication that the transfer is complete.
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

static int sam_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                         size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Initialize register sampling */

  sam_xfrsampleinit(priv);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Disable DMA handshaking */

  sam_putreg(priv, 0, SAM_HSMCI_DMA_OFFSET);

  /* Setup of the transfer configuration */

  priv->dmabusy = false;
  priv->xfrbusy = true;
  priv->txbusy  = false;

  /* Save the destination buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t*)buffer;
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
 *   Setup hardware in preparation for data transfer from the card.  This method
 *   will do whatever controller setup is necessary.  This would be called
 *   for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
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

static int sam_sendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer,
                         size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  unsigned int nwords;
  const uint32_t *ptr;
  uint32_t sr;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

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
   * REVISIT:  If TX data underruns occur, then it may be necessary to
   * disable pre-emption around this loop.
   */

  nwords = (buflen + 3) >> 2;
  ptr = (const uint32_t *)buffer;

  while (nwords > 0)
    {
      /* Check the HSMCI status */

      sr = sam_getreg(priv, SAM_HSMCI_SR_OFFSET);
      if ((sr & HSMCI_DATA_DMASEND_ERRORS) != 0)
        {
          /* Some fatal error has occurred */

          fdbg("ERROR: sr %08x\n", sr);
          return -EIO;
        }
      else if ((sr & HSMCI_INT_TXRDY) != 0)
        {
          /* TXRDY -- transfer another word */

          sam_putreg(priv, *ptr++, SAM_HSMCI_TDR_OFFSET);
          nwords--;
        }
    }

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

static int sam_cancel(FAR struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev;

  /* Disable all transfer- and event- related interrupts */

  sam_disablexfrints(priv);
  sam_disablewaitints(priv, 0);

  /* No data transfer */

  sam_notransfer(priv);

  /* Clearing (most) pending interrupt status by reading the status register */

  (void)sam_getreg(priv, SAM_HSMCI_SR_OFFSET);

  /* Cancel any watchdog timeout */

  (void)wd_cancel(priv->waitwdog);

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

static int sam_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev;
  uint32_t sr;
  uint32_t pending;
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

  for (;;)
    {
      /* Did a Command-Response sequence termination evernt occur? */

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

              fdbg("ERROR: cmd: %08x events: %08x SR: %08x\n",
                   cmd, priv->cmdrmask, sr);

              if ((pending & HSMCI_RESPONSE_TIMEOUT_ERRORS) != 0)
                {
                  /* Yes.. return a timeout error */

                  priv->wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE|SDIOWAIT_TIMEOUT;
                  return -ETIMEDOUT;
                }
              else
                {
                  /* No.. return some generic I/O error */

                  priv->wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE|SDIOWAIT_ERROR;
                  return -EIO;
                }
            }
          else
            {
              /* The Command-Response sequence ended with no error */

              priv->wkupevent = SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE;
              return OK;
            }
       }
      else if (--timeout <= 0)
        {
          fdbg("ERROR: Timeout cmd: %08x events: %08x SR: %08x\n",
               cmd, priv->cmdrmask, sr);

          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return -ETIMEDOUT;
        }
    }
}

/****************************************************************************
 * Name: sam_recvRx
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
 *   failure means only a failure to obtain the requested reponse (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intact and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int sam_recvshort(FAR struct sdio_dev_s *dev,
                         uint32_t cmd, uint32_t *rshort)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev;
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

#ifdef CONFIG_DEBUG
  if (!rshort)
    {
      fdbg("ERROR: rshort=NULL\n");
      ret = -EINVAL;
    }

  /* Check that this is the correct response to this command */

  else if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      fdbg("ERROR: Wrong response CMD=%08x\n", cmd);
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

static int sam_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t rlong[4])
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev;
  int ret = OK;

 /* R2  CID, CSD register (136-bit)
  *     135       0               Start bit
  *     134       0               Transmission bit (0=from card)
  *     133:128   bit5   - bit0   Reserved
  *     127:1     bit127 - bit1   127-bit CID or CSD register
  *                               (including internal CRC)
  *     0         1               End bit
  */

#ifdef CONFIG_DEBUG
  /* Check that R1 is the correct response to this command */

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      fdbg("ERROR: Wrong response CMD=%08x\n", cmd);
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

static int sam_recvnotimpl(FAR struct sdio_dev_s *dev,
                           uint32_t cmd, uint32_t *rnotimpl)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev;
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
 *   This sequency should eliminate race conditions between the command/trasnfer
 *   setup and the subsequent events.
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

static void sam_waitenable(FAR struct sdio_dev_s *dev,
                           sdio_eventset_t eventset)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev;
  uint32_t waitmask;

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  sam_disablewaitints(priv, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitmask = 0;
  if ((eventset & (SDIOWAIT_CMDDONE|SDIOWAIT_RESPONSEDONE)) != 0)
    {
      waitmask |= priv->cmdrmask;
    }

  /* Clear (most) pending interrupts by reading the status register.
   * No interrupts should be lost (assuming that interrupts were enabled
   * before sam_waitenable() was called).  Any interrupts that become
   * pending after this point must be valid event indications.
   */

  (void)sam_getreg(priv, SAM_HSMCI_SR_OFFSET);

  /* Wait interrupts are configured here, but not enabled until
   * sam_eventwait() is called.  Why?  Because the XFRDONE interrupt is
   * always pending until start the data transfer.
   */

  sam_configwaitints(priv, waitmask, eventset);
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

static sdio_eventset_t sam_eventwait(FAR struct sdio_dev_s *dev,
                                     uint32_t timeout)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev;
  sdio_eventset_t wkupevent = 0;
  int ret;

  /* Since interrupts not been enabled to this point, any relevant events
   * are pending and should not yet have occurred.
   */

  DEBUGASSERT(priv->waitevents != 0 && priv->wkupevent == 0);

  /* Now enable event-related interrupts. If the events are pending, they
   * may happen immediately here before entering the loop.
   */

  sam_enableints(priv);

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevent will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero).
   */

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;

      /* Yes.. Handle a cornercase */

      if (!timeout)
        {
           return SDIOWAIT_TIMEOUT;
        }

      /* Start the watchdog timer.  I am not sure why this is, but I am
       * currently seeing some additional delays when DMA is used.
       */

      if (priv->txbusy)
        {
          /* TX transfers can be VERY long in the worst case */

          timeout = MAX(5000, timeout);
        }

      delay = MSEC2TICK(timeout);
      ret   = wd_start(priv->waitwdog, delay, (wdentry_t)sam_eventtimeout,
                       1, (uint32_t)priv);
      if (ret != OK)
        {
           fdbg("ERROR: wd_start failed: %d\n", ret);
        }
    }

  /* Loop until the event (or the timeout occurs). Race conditions are avoided
   * by calling sam_waitenable prior to triggering the logic that will cause
   * the wait to terminate.  Under certain race conditions, the waited-for
   * may have already occurred before this function was called!
   */

  for (;;)
    {
      /* Wait for an event in event set to occur.  If this the event has already
       * occurred, then the semaphore will already have been incremented and
       * there will be no wait.
       */

      sam_takesem(priv);
      wkupevent = priv->wkupevent;

      /* Check if the event has occurred.  When the event has occurred, then
       * evenset will be set to 0 and wkupevent will be set to a nonzero value.
       * When wkupevent becomes non-zero, further interrupts will have already
       * been disabled.
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

static void sam_callbackenable(FAR struct sdio_dev_s *dev,
                               sdio_eventset_t eventset)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev;

  fvdbg("eventset: %02x\n", eventset);
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
 *   callback - The funtion to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

static int sam_registercallback(FAR struct sdio_dev_s *dev,
                                worker_t callback, void *arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s*)dev;

  /* Disable callbacks and register this callback and is argument */

  fvdbg("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: sam_dmasupported
 *
 * Description:
 *   Return true if the hardware can support DMA
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   true if DMA is supported.
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_DMA
static bool sam_dmasupported(FAR struct sdio_dev_s *dev)
{
  return true;
}
#endif

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
static int sam_dmarecvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                            size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t paddr;
  uint32_t maddr;
  uint32_t regval;
  unsigned int blocksize;
  unsigned int nblocks;
  unsigned int offset;
  unsigned int i;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* How many blocks?  That should have been saved by the sam_blocksetup()
   * method earlier.
   */

  regval    = sam_getreg(priv, SAM_HSMCI_BLKR_OFFSET);
  nblocks   = ((regval &  HSMCI_BLKR_BCNT_MASK) >> HSMCI_BLKR_BCNT_SHIFT);
  blocksize = ((regval &  HSMCI_BLKR_BLKLEN_MASK) >> HSMCI_BLKR_BLKLEN_SHIFT);

  DEBUGASSERT(nblocks > 0 && blocksize > 0 && (blocksize & 3) == 0);

  /* Physical address of the HSCMI source register, either the TDR (for
   * single transfers) or the first FIFO register, and the physical address
   * of the buffer in RAM.
   */

  offset = (nblocks == 1 ? SAM_HSMCI_RDR_OFFSET : SAM_HSMCI_FIFO_OFFSET);
  paddr  = hsmci_physregaddr(priv, offset);
  maddr  = sam_physramaddr((uintptr_t)buffer);

  /* Setup register sampling (only works for the case of nblocks == 1) */

  sam_xfrsampleinit(priv);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Set DMA for each block */

  for (i = 0; i < nblocks; i++)
    {
      /* Configure the RX DMA */

      sam_dmarxsetup(priv->dma, paddr, maddr, buflen);

      /* Update addresses for the next block */

      paddr += sizeof(uint32_t);
      maddr += blocksize;
   }

  /* Enable DMA handshaking */

  sam_putreg(priv, HSMCI_DMA_DMAEN | HSMCI_DMA_CHKSIZE, SAM_HSMCI_DMA_OFFSET);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_ENABLE);

  /* Start the DMA */

  priv->dmabusy = true;
  priv->xfrbusy = true;
  priv->txbusy  = false;
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
static int sam_dmasendsetup(FAR struct sdio_dev_s *dev,
                            FAR const uint8_t *buffer, size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t paddr;
  uint32_t maddr;
  uint32_t regval;
  unsigned int blocksize;
  unsigned int nblocks;
  unsigned int offset;
  unsigned int i;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* How many blocks?  That should have been saved by the sam_blocksetup()
   * method earlier.
   */

  regval    = sam_getreg(priv, SAM_HSMCI_BLKR_OFFSET);
  nblocks   = ((regval &  HSMCI_BLKR_BCNT_MASK) >> HSMCI_BLKR_BCNT_SHIFT);
  blocksize = ((regval &  HSMCI_BLKR_BLKLEN_MASK) >> HSMCI_BLKR_BLKLEN_SHIFT);

  DEBUGASSERT(nblocks > 0 && blocksize > 0 && (blocksize & 3) == 0);

  /* Physical address of the HSCMI source register, either the TDR (for
   * single transfers) or the first FIFO register, and the physical address
   * of the buffer in RAM.
   */

  offset = (nblocks == 1 ? SAM_HSMCI_TDR_OFFSET : SAM_HSMCI_FIFO_OFFSET);
  paddr  = hsmci_physregaddr(priv, offset);
  maddr  = sam_physramaddr((uintptr_t)buffer);

  /* Setup register sampling (only works for the case of nblocks == 1) */

  sam_xfrsampleinit(priv);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Set DMA for each block */

  for (i = 0; i < nblocks; i++)
    {
      /* Configure the TX DMA */

      sam_dmatxsetup(priv->dma, paddr, maddr, buflen);

      /* Update addresses for the next block */

      paddr += sizeof(uint32_t);
      maddr += blocksize;
   }

  /* Enable DMA handshaking */

  sam_putreg(priv, HSMCI_DMA_DMAEN | HSMCI_DMA_CHKSIZE, SAM_HSMCI_DMA_OFFSET);
  sam_xfrsample(priv, SAMPLENDX_BEFORE_ENABLE);

  /* Start the DMA */

  priv->dmabusy = true;
  priv->xfrbusy = true;
  priv->txbusy  = true;
  sam_dmastart(priv->dma, sam_dmacallback, priv);

  /* Configure transfer-related interrupts.  Transfer interrupts are not
   * enabled until after the transfer is stard with an SD command (i.e.,
   * at the beginning of sam_eventwait().
   */

  sam_xfrsample(priv, SAMPLENDX_AFTER_SETUP);
  sam_configxfrints(priv, HSMCI_DMASEND_INTS);
  return OK;
}
#endif

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/
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
  struct sam_dev_s *priv = (struct sam_dev_s*)arg;
  irqstate_t flags;
  int ret;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  fvdbg("Callback %p(%p) cbevents: %02x cdstatus: %02x\n",
        priv->callback, priv->cbarg, priv->cbevents, priv->cdstatus);

  flags = irqsave();
  if (priv->callback)
    {
      /* Yes.. Check for enabled callback events */

      if ((priv->cdstatus & SDIO_STATUS_PRESENT) != 0)
        {
          /* Media is present.  Is the media inserted event enabled? */

          if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
           {
             /* No... return without performing the callback */

              return;
            }
        }
      else
        {
          /* Media is not present.  Is the media eject event enabled? */

          if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
            {
              /* No... return without performing the callback */

              return;
            }
        }

      /* Perform the callback, disabling further callbacks.  Of course, the
       * the callback can (and probably should) re-enable callbacks.
       */

      priv->cbevents = 0;

      /* This function is called either from (1) the context of the calling
       * thread or from the the context of (2) card detection logic.  The
       * caller may or may not have interrupts disabled (we have them
       & disabled here!).
       *
       * So to minimize the possibility of recursive behavior and to assure
       * that callback is always performed outside of the interrupt handling
       * context and with interrupts enabled, the callback is always performed
       * on the lower priority work thread.
       */

      /* First cancel any existing work */

      ret = work_cancel(LPWORK, &priv->cbwork);
      if (ret < 0)
        {
          /* NOTE: Currently, work_cancel only returns success */

          fdbg("ERROR: Failed to cancel work: %d\n", ret);
        }

      fllvdbg("Queuing callback to %p(%p)\n", priv->callback, priv->cbarg);
      ret = work_queue(LPWORK, &priv->cbwork, (worker_t)priv->callback,
                       priv->cbarg, 0);
      if (ret < 0)
        {
          /* NOTE: Currently, work_queue only returns success */

          fdbg("ERROR: Failed to schedule work: %d\n", ret);
        }
    }

  irqrestore(flags);
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
 * Returned Values:
 *   A reference to an SDIO interface structure.  NULL is returned on failures.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *sdio_initialize(int slotno)
{
  struct sam_dev_s *priv;
  uint32_t pid;
  uint8_t dmac;

  /* Select the slot and perform slot-specific initialization.  The
   * semantics here are bad.  There are three HSMCI peripherals that we
   * will treat as "slots."  In principle they could each peripheral could
   * support 4 slots, A-D.  However, selection of slots B, C, and D is
   * listed as "reserved" in the HSMCI register definitions.  So, at least
   * for now, an* HSMCI peripheral does correspond to a slot.
   */

  fdbg("slotno: %d\n", slotno);

#ifdef CONFIG_SAMA5_HSMCI0
  if (slotno == 0)
    {
      /* Select HSMCI0 */

      priv = &g_hsmci0;

      /* HSMCI0 Initialization */

      priv->base  = SAM_HSMCI0_VBASE;
      priv->hsmci = 0;

      /* Configure PIOs for 4-bit, wide-bus operation.  NOTE: (1) the chip
       * is capable of 8-bit wide bus operation but D4-D7 are not configured,
       * (2) any card detection PIOs must be set up in board-specific logic.
       *
       * REVISIT: What about Slot B?
       */

      sam_configpio(PIO_MCI0_DA0);   /* Data 0 of Slot A */
      sam_configpio(PIO_MCI0_DA1);   /* Data 1 of Slot A */
      sam_configpio(PIO_MCI0_DA2);   /* Data 2 of Slot A */
      sam_configpio(PIO_MCI0_DA3);   /* Data 3 of Slot A */
      sam_configpio(PIO_MCI0_CK);    /* Common SD clock */
      sam_configpio(PIO_MCI0_CDA);   /* Command/Response of Slot A */

      /* Enable the HSMCI0 peripheral clock.  This really should be done in
       * sam_enable (as well as disabling peripheral clocks in sam_disable().
       */

      sam_hsmci0_enableclk();

      /* For DMA channel selection */

      dmac = HSMCI0_DMAC;
      pid  = SAM_PID_HSMCI0;
    }
  else
#endif
#ifdef CONFIG_SAMA5_HSMCI1
  if (slotno == 1)
    {
      /* Select HSMCI1 */

      priv = &g_hsmci1;

      /* HSMCI1 Initialization */

      priv->base  = SAM_HSMCI1_VBASE;
      priv->hsmci = 1;

      /* Configure PIOs for 4-bit, wide-bus operation.  NOTE: (1) the chip
       * is capable of 8-bit wide bus operation but D4-D7 are not configured,
       * (2) any card detection PIOs must be set up in board-specific logic.
       *
       * REVISIT: What about Slot B?
       */

      sam_configpio(PIO_MCI1_DA0);   /* Data 0 of Slot A */
      sam_configpio(PIO_MCI1_DA1);   /* Data 1 of Slot A */
      sam_configpio(PIO_MCI1_DA2);   /* Data 2 of Slot A */
      sam_configpio(PIO_MCI1_DA3);   /* Data 3 of Slot A */
      sam_configpio(PIO_MCI1_CK);    /* Common SD clock */
      sam_configpio(PIO_MCI1_CDA);   /* Command/Response of Slot A */

      /* Enable the HSMCI1 peripheral clock  This really should be done in
       * sam_enable (as well as disabling peripheral clocks in sam_disable().
       */

      sam_hsmci1_enableclk();

      /* For DMA channel selection */

      dmac = HSMCI1_DMAC;
      pid  = SAM_PID_HSMCI1;
    }
  else
#endif
#ifdef CONFIG_SAMA5_HSMCI2
  if (slotno == 2)
    {
      /* Select HSMCI2 */

      priv = &g_hsmci2;

      /* HSMCI2 Initialization */

      priv->base  = SAM_HSMCI2_VBASE;
      priv->hsmci = 2;

      /* Configure PIOs for 4-bit, wide-bus operation.  NOTE: (1) the chip
       * is capable of 8-bit wide bus operation but D4-D7 are not configured,
       * (2) any card detection PIOs must be set up in board-specific logic.
       *
       * REVISIT: What about Slot B?
       */

      sam_configpio(PIO_MCI2_DA0);   /* Data 0 of Slot A */
      sam_configpio(PIO_MCI2_DA1);   /* Data 1 of Slot A */
      sam_configpio(PIO_MCI2_DA2);   /* Data 2 of Slot A */
      sam_configpio(PIO_MCI1_DA3);   /* Data 3 of Slot A */
      sam_configpio(PIO_MCI2_CK);    /* Common SD clock */
      sam_configpio(PIO_MCI2_CDA);   /* Command/Response of Slot A */

      /* Enable the HSMCI2 peripheral clock  This really should be done in
       * sam_enable (as well as disabling peripheral clocks in sam_disable().
       */

      sam_hsmci1_enableclk();

      /* For DMA channel selection */

      dmac = HSMCI2_DMAC;
      pid  = SAM_PID_HSMCI2;
    }
  else
#endif
    {
      DEBUGPANIC();
      return NULL;
    }

  fvdbg("priv: %p base: %08x hsmci: %d dmac: %d pid: %d\n",
        priv, priv->base, priv->hsmci, dmac, pid);

  /* Initialize the HSMCI slot structure */

  sem_init(&priv->waitsem, 0, 0);
  priv->waitwdog = wd_create();
  DEBUGASSERT(priv->waitwdog);

  /* Initialize the callbacks */

  memcpy(&priv->dev, &g_callbacks, sizeof(struct sdio_dev_s));

  /* Allocate a DMA channel */

  priv->dma = sam_dmachannel(dmac, DMA_FLAGS(pid));
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
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   May be called from an interrupt handler.
 *
 ****************************************************************************/

void sdio_mediachange(FAR struct sdio_dev_s *dev, bool cardinslot)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint8_t cdstatus;
  irqstate_t flags;

  /* Update card status.  Interrupts are disabled here because if we are
   * not called from an interrupt handler, then the following steps must
   * still be atomic.
   */

  flags = irqsave();
  cdstatus = priv->cdstatus;
  if (cardinslot)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }

  fllvdbg("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

  /* Perform any requested callback if the status has changed */

  if (cdstatus != priv->cdstatus)
    {
      sam_callback(priv);
    }

  irqrestore(flags);
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
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void sdio_wrprotect(FAR struct sdio_dev_s *dev, bool wrprotect)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  irqstate_t flags;

  /* Update card status */

  flags = irqsave();
  if (wrprotect)
    {
      priv->cdstatus |= SDIO_STATUS_WRPROTECTED;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_WRPROTECTED;
    }

  fvdbg("cdstatus: %02x\n", priv->cdstatus);
  irqrestore(flags);
}

#endif /* CONFIG_SAMA5_HSMCI0 || CONFIG_SAMA5_HSMCI1 || CONFIG_SAMA5_HSMCI2 */
