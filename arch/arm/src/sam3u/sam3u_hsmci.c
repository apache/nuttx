/****************************************************************************
 * arch/arm/src/sam3u/sam3u_sdio.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include <wdog.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/mmcsd.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "sam3u_internal.h"
#include "sam3u_dmac.h"
#include "sam3u_pmc.h"
#include "sam3u_hsmci.h"

#if CONFIG_SAM3U_HSMCI

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if !defined(CONFIG_SDIO_DMA) || !defined(CONFIG_SAM3U_DMA)
#  warning "HSMCI requires CONFIG_SDIO_DMA and CONFIG_SAM3U_DMA"
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifndef CONFIG_HSMCI_PRI
#  define CONFIG_HSMCI_PRI        NVIC_SYSH_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_HSMCI_DMAPRIO
#  define CONFIG_HSMCI_DMAPRIO    DMA_CCR_PRIMED
#endif

#if !defined(CONFIG_DEBUG_FS) || !defined(CONFIG_DEBUG_VERBOSE)
#  undef CONFIG_HSMCI_XFRDEBUG
#endif

/* Friendly CLKCR bit re-definitions ****************************************/

#define HSMCI_CLKCR_RISINGEDGE   (0)
#define HSMCI_CLKCR_FALLINGEDGE  HSMCI_CLKCR_NEGEDGE

/* Mode dependent settings.  These depend on clock devisor settings that must
 * be defined in the board-specific board.h header file: HSMCI_INIT_CLKDIV,
 * HSMCI_MMCXFR_CLKDIV, and HSMCI_SDXFR_CLKDIV.
 */
  
#define HSMCI_CLCKCR_INIT        (((SAM3U_MCK_FREQUENCY / (  400000 * 2)) - 1) | (7 << HSMCI_MR_PWSDIV_SHIFT))
#define HSMCI_CLKCR_MMCXFR       (((SAM3U_MCK_FREQUENCY / (20000000 * 2)) - 1) | (7 << HSMCI_MR_PWSDIV_SHIFT))
#define HSMCI_CLCKR_SDXFR        (((SAM3U_MCK_FREQUENCY / (25000000 * 2)) - 1) | (7 << HSMCI_MR_PWSDIV_SHIFT))
#define HSMCI_CLCKR_SDWIDEXFR    (((SAM3U_MCK_FREQUENCY / (25000000 * 2)) - 1) | (7 << HSMCI_MR_PWSDIV_SHIFT))

/* Timing */

#define HSMCI_CMDTIMEOUT         (100000)
#define HSMCI_LONGTIMEOUT        (0x7fffffff)

/* Big DTIMER setting */

#define HSMCI_DTIMER_DATATIMEOUT (0x000fffff)

/* DMA CCR register settings */

#define HSMCI_RXDMA32_CONFIG \
  ( CONFIG_HSMCI_DMAPRIO | DMA_CCR_MSIZE_32BITS | DMA_CCR_PSIZE_32BITS | DMA_CCR_MINC)
#define HSMCI_TXDMA32_CONFIG \
  ( CONFIG_HSMCI_DMAPRIO | DMA_CCR_MSIZE_32BITS | DMA_CCR_PSIZE_32BITS | DMA_CCR_MINC | DMA_CCR_DIR)

/* FIFO sizes */

#define HSMCI_HALFFIFO_WORDS     (8)
#define HSMCI_HALFFIFO_BYTES     (8*4)

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

#define HSMCI_DATA_RECV_ERRORS \
  ( HSMCI_INT_OVRE  | HSMCI_INT_CSTOE | HSMCI_INT_DTOE    | HSMCI_INT_DCRCE )

#define HSMCI_DATA_DMARECV_ERRORS \
  ( HSMCI_INT_OVRE  | HSMCI_INT_BLKOVRE | HSMCI_INT_CSTOE | HSMCI_INT_DTOE | \
    HSMCI_INT_DCRCE )

#define HSMCI_DATA_SEND_ERRORS \
  ( HSMCI_INT_UNRE  | HSMCI_INT_CSTOE | HSMCI_INT_DTOE    | HSMCI_INT_DCRCE )

#define HSMCI_DATA_DMASEND_ERRORS \
  ( HSMCI_INT_UNRE  | HSMCI_INT_CSTOE | HSMCI_INT_DTOE    | HSMCI_INT_DCRCE )

/* Data transfer status and interrupt mask bits */

#define HSMCI_RECV_INTS \
  ( HSMCI_DATA_RECV_ERRORS    | HSMCI_INT_XFRDONE )
#define HSMCI_SEND_INTS \
  ( HSMCI_DATA_SEND_ERROR     | HSMCI_INT_XFRDONE )
#define HSMCI_DMARECV_INTS \
  ( HSMCI_DATA_DMARECV_ERRORS | HSMCI_INT_XFRDONE | HSMCI_INT_DMADONE )
#define HSMCI_DMASEND_INTS \
  ( HSMCI_DATA_DMASEND_ERRORS | HSMCI_INT_XFRDONE | HSMCI_INT_DMADONE )

/* Event waiting interrupt mask bits */

#define HSMCI_CMDDONE_INTS \
  ( HSMCI_INT_CMDRDY )
#define HSMCI_RESPONSE_INTS \
  ( HSMCI_RESPONSE_ERRORS | HSMCI_INT_CMDREND )
#define HSMCI_XFRDONE_INTS (0)

/* Register logging support */

#ifdef CONFIG_HSMCI_XFRDEBUG
#ifdef CONFIG_DEBUG_DMA
#    define SAMPLENDX_BEFORE_SETUP  0
#    define SAMPLENDX_BEFORE_ENABLE 1
#    define SAMPLENDX_AFTER_SETUP   2
#    define SAMPLENDX_END_TRANSFER  3
#    define SAMPLENDX_DMA_CALLBACK  4
#    define DEBUG_NSAMPLES          5
#  else
#    define SAMPLENDX_BEFORE_SETUP  0
#    define SAMPLENDX_AFTER_SETUP   1
#    define SAMPLENDX_END_TRANSFER  2
#    define DEBUG_NSAMPLES          3
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the SAM3U HSMCI interface */

struct sam3u_dev_s
{
  struct sdio_dev_s  dev;        /* Standard, base SDIO interface */
  
  /* SAM3U-specific extensions */
  /* Event support */

  sem_t              waitsem;    /* Implements event waiting */
  sdio_eventset_t    waitevents; /* Set of events to be waited for */
  uint32_t           waitmask;   /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  WDOG_ID            waitwdog;   /* Watchdog that handles event timeouts */

  /* Callback support */

  uint8_t            cdstatus;   /* Card status */
  sdio_eventset_t    cbevents;   /* Set of events to be cause callbacks */
  worker_t           callback;   /* Registered callback function */
  void              *cbarg;      /* Registered callback argument */
  struct work_s      cbwork;     /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t          *buffer;     /* Address of current R/W buffer */
  size_t             remaining;  /* Number of bytes remaining in the transfer */
  uint32_t           xfrmask;    /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  bool               widebus;    /* Required for DMA support */
  bool               dmamode;    /* true: DMA mode transfer */
  DMA_HANDLE         dma;        /* Handle for DMA channel */
};

/* Register logging support */

#ifdef CONFIG_HSMCI_XFRDEBUG
struct sam3u_hsmciregs_s
{
  uint8_t  power;
  uint16_t clkcr;
  uint16_t dctrl;
  uint32_t dtimer;
  uint32_t dlen;
  uint32_t dcount;
  uint32_t sr;      /* Status register */
  uint32_t imr;     /* Interrupt mask register */
  uint32_t fifocnt;
};

struct sam3u_sampleregs_s
{
  struct sam3u_hsmciregs_s hsmci;
#ifdef CONFIG_DEBUG_DMA
  struct sam3u_dmaregs_s  dma;
#endif
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void sam3u_takesem(struct sam3u_dev_s *priv);
#define     sam3u_givesem(priv) (sem_post(&priv->waitsem))
static inline void sam3u_setclkcr(uint32_t clkcr);
static void sam3u_enablewaitints(struct sam3u_dev_s *priv, uint32_t waitmask,
              sdio_eventset_t waitevents);
static void sam3u_disablewaitints(struct sam3u_dev_s *priv, sdio_eventset_t wkupevents);
static void sam3u_enablexfrints(struct sam3u_dev_s *priv, uint32_t xfrmask);
static void sam3u_disablexfrints(struct sam3u_dev_s *priv);
static inline uint32_t sam3u_getpwrctrl(void);

/* DMA Helpers **************************************************************/


#ifdef CONFIG_HSMCI_XFRDEBUG
static void sam3u_sampleinit(void);
static void sam3u_sdiosample(struct sam3u_hsmciregs_s *regs);
static void sam3u_sample(struct sam3u_dev_s *priv, int index);
static void sam3u_sdiodump(struct sam3u_hsmciregs_s *regs, const char *msg);
static void sam3u_dumpsample(struct sam3u_dev_s *priv,
              struct sam3u_sampleregs_s *regs, const char *msg);
static void sam3u_dumpsamples(struct sam3u_dev_s *priv);
#else
#  define   sam3u_sampleinit()
#  define   sam3u_sample(priv,index)
#  define   sam3u_dumpsamples(priv)
#endif

static void sam3u_dmacallback(DMA_HANDLE handle, uint8_t isr, void *arg);

/* Data Transfer Helpers ****************************************************/

static uint8_t sam3u_log2(uint16_t value);
static void sam3u_dataconfig(uint32_t timeout, uint32_t dlen, uint32_t dctrl);
static void sam3u_datadisable(void);
static void sam3u_sendfifo(struct sam3u_dev_s *priv);
static void sam3u_recvfifo(struct sam3u_dev_s *priv);
static void sam3u_eventtimeout(int argc, uint32_t arg);
static void sam3u_endwait(struct sam3u_dev_s *priv, sdio_eventset_t wkupevent);
static void sam3u_endtransfer(struct sam3u_dev_s *priv, sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  sam3u_interrupt(int irq, void *context);

/* SDIO interface methods ***************************************************/

/* Initialization/setup */

static void sam3u_reset(FAR struct sdio_dev_s *dev);
static uint8_t sam3u_status(FAR struct sdio_dev_s *dev);
static void sam3u_widebus(FAR struct sdio_dev_s *dev, bool enable);
static void sam3u_clock(FAR struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  sam3u_attach(FAR struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static void sam3u_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);
static int  sam3u_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
              size_t nbytes);
static int  sam3u_sendsetup(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, uint32_t nbytes);
static int  sam3u_cancel(FAR struct sdio_dev_s *dev);

static int  sam3u_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd);
static int  sam3u_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  sam3u_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  sam3u_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  sam3u_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rnotimpl);

/* EVENT handler */

static void sam3u_waitenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static sdio_eventset_t
            sam3u_eventwait(FAR struct sdio_dev_s *dev, uint32_t timeout);
static void sam3u_callbackenable(FAR struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  sam3u_registercallback(FAR struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

static bool sam3u_dmasupported(FAR struct sdio_dev_s *dev);
static int  sam3u_dmarecvsetup(FAR struct sdio_dev_s *dev,
              FAR uint8_t *buffer, size_t buflen);
static int  sam3u_dmasendsetup(FAR struct sdio_dev_s *dev,
              FAR const uint8_t *buffer, size_t buflen);

/* Initialization/uninitialization/reset ************************************/

static void sam3u_callback(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct sam3u_dev_s g_sdiodev =
{
  .dev =
  {
    .reset            = sam3u_reset,
    .status           = sam3u_status,
    .widebus          = sam3u_widebus,
    .clock            = sam3u_clock,
    .attach           = sam3u_attach,
    .sendcmd          = sam3u_sendcmd,
    .recvsetup        = sam3u_recvsetup,
    .sendsetup        = sam3u_sendsetup,
    .cancel           = sam3u_cancel,
    .waitresponse     = sam3u_waitresponse,
    .recvR1           = sam3u_recvshortcrc,
    .recvR2           = sam3u_recvlong,
    .recvR3           = sam3u_recvshort,
    .recvR4           = sam3u_recvnotimpl,
    .recvR5           = sam3u_recvnotimpl,
    .recvR6           = sam3u_recvshortcrc,
    .recvR7           = sam3u_recvshort,
    .waitenable       = sam3u_waitenable,
    .eventwait        = sam3u_eventwait,
    .callbackenable   = sam3u_callbackenable,
    .registercallback = sam3u_registercallback,
    .dmasupported     = sam3u_dmasupported,
    .dmarecvsetup     = sam3u_dmarecvsetup,
    .dmasendsetup     = sam3u_dmasendsetup,
  },
};

/* Register logging support */

#ifdef CONFIG_HSMCI_XFRDEBUG
static struct sam3u_sampleregs_s g_sampleregs[DEBUG_NSAMPLES];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Low-level Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: sam3u_takesem
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

static void sam3u_takesem(struct sam3u_dev_s *priv)
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
 * Name: sam3u_setclkcr
 *
 * Description:
 *   Modify oft-changed bits in the CLKCR register.  Only the following bit-
 *   fields are changed:
 *
 *   CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, and HWFC_EN
 *
 * Input Parameters:
 *   clkcr - A new CLKCR setting for the above mentions bits (other bits
 *           are ignored.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam3u_setclkcr(uint32_t clkcr)
{
  uint32_t regval = getreg32(SAM3U_HSMCI_CLKCR);
    
  /* Clear CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, HWFC_EN bits */

  regval &= ~(HSMCI_CLKCR_CLKDIV_MASK|HSMCI_CLKCR_PWRSAV|HSMCI_CLKCR_BYPASS|
              HSMCI_CLKCR_WIDBUS_MASK|HSMCI_CLKCR_NEGEDGE|HSMCI_CLKCR_HWFC_EN);

  /* Replace with user provided settings */

  clkcr  &=  (HSMCI_CLKCR_CLKDIV_MASK|HSMCI_CLKCR_PWRSAV|HSMCI_CLKCR_BYPASS|
              HSMCI_CLKCR_WIDBUS_MASK|HSMCI_CLKCR_NEGEDGE|HSMCI_CLKCR_HWFC_EN);
  regval |=  clkcr;
  putreg32(regval, SAM3U_HSMCI_CLKCR);
  fvdbg("CLKCR: %08x\n", getreg32(SAM3U_HSMCI_CLKCR));
}

/****************************************************************************
 * Name: sam3u_enablewaitints
 *
 * Description:
 *   Enable HSMCI interrupts needed to suport the wait function
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

static void sam3u_enablewaitints(struct sam3u_dev_s *priv, uint32_t waitmask,
                                 sdio_eventset_t waitevents)
{
  irqstate_t flags;

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags = irqsave();
  priv->waitevents = waitevents;
  priv->wkupevent  = 0;
  priv->waitmask   = waitmask;
  putreg32(priv->xfrmask | priv->waitmask, SAM3U_HSMCI_IER);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam3u_disablewaitints
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

static void sam3u_disablewaitints(struct sam3u_dev_s *priv,
                                  sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags = irqsave();
  priv->waitevents = 0;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = 0;
  putreg32(~priv->xfrmask, SAM3U_HSMCI_IDR);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam3u_enablexfrints
 *
 * Description:
 *   Enable HSMCI interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the HSMCI device state structure
 *   xfrmask - The set of bits in the HSMCI MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_enablexfrints(struct sam3u_dev_s *priv, uint32_t xfrmask)
{
  irqstate_t flags = irqsave();
  priv->xfrmask = xfrmask;
  putreg32(priv->xfrmask | priv->waitmask, SAM3U_HSMCI_IER);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam3u_disablexfrints
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

static void sam3u_disablexfrints(struct sam3u_dev_s *priv)
{
  irqstate_t flags = irqsave();
  priv->xfrmask = 0;
  putreg32(~priv->waitmask, SAM3U_HSMCI_IDR);
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam3u_getpwrctrl
 *
 * Description:
 *   Return the current value of the  the PWRCTRL field of the HSMCI POWER
 *   register.  This function can be used to see the the HSMCI is power ON
 *   or OFF
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current value of the  the PWRCTRL field of the HSMCI POWER register.
 *
 ****************************************************************************/

static inline uint32_t sam3u_getpwrctrl(void)
{
  return getreg32(SAM3U_HSMCI_POWER) & HSMCI_POWER_PWRCTRL_MASK;
}

/****************************************************************************
 * DMA Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_sampleinit
 *
 * Description:
 *   Setup prior to collecting DMA samples
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_XFRDEBUG
static void sam3u_sampleinit(void)
{
  memset(g_sampleregs, 0xff, DEBUG_NSAMPLES * sizeof(struct sam3u_sampleregs_s));
}
#endif

/****************************************************************************
 * Name: sam3u_sdiosample
 *
 * Description:
 *   Sample HSMCI registers
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_XFRDEBUG
static void sam3u_sdiosample(struct sam3u_hsmciregs_s *regs)
{
  regs->power   = (uint8_t)getreg32(SAM3U_HSMCI_POWER);
  regs->clkcr   = (uint16_t)getreg32(SAM3U_HSMCI_CLKCR);
  regs->dctrl   = (uint16_t)getreg32(SAM3U_HSMCI_DCTRL);
  regs->dtimer  = getreg32(SAM3U_HSMCI_DTIMER);
  regs->dlen    = getreg32(SAM3U_HSMCI_DLEN);
  regs->dcount  = getreg32(SAM3U_HSMCI_DCOUNT);
  regs->sr      = getreg32(SAM3U_HSMCI_SR);
  regs->imr     = getreg32(SAM3U_HSMCI_IMR);
  regs->fifocnt = getreg32(SAM3U_HSMCI_FIFOCNT);
}
#endif

/****************************************************************************
 * Name: sam3u_sample
 *
 * Description:
 *   Sample HSMCI/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_XFRDEBUG
static void sam3u_sample(struct sam3u_dev_s *priv, int index)
{
  struct sam3u_sampleregs_s *regs = &g_sampleregs[index];
#ifdef CONFIG_DEBUG_DMA
  if (priv->dmamode)
    {
      sam3u_dmasample(priv->dma, &regs->dma);
    }
#endif
  sam3u_sdiosample(&regs->hsmci);
}
#endif

/****************************************************************************
 * Name: sam3u_sdiodump
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_XFRDEBUG
static void sam3u_sdiodump(struct sam3u_hsmciregs_s *regs, const char *msg)
{
  fdbg("HSMCI Registers: %s\n", msg);
  fdbg("  POWER[%08x]: %08x\n", SAM3U_HSMCI_POWER,   regs->power);
  fdbg("  CLKCR[%08x]: %08x\n", SAM3U_HSMCI_CLKCR,   regs->clkcr);
  fdbg("  DCTRL[%08x]: %08x\n", SAM3U_HSMCI_DCTRL,   regs->dctrl);
  fdbg(" DTIMER[%08x]: %08x\n", SAM3U_HSMCI_DTIMER,  regs->dtimer);
  fdbg("   DLEN[%08x]: %08x\n", SAM3U_HSMCI_DLEN,    regs->dlen);
  fdbg(" DCOUNT[%08x]: %08x\n", SAM3U_HSMCI_DCOUNT,  regs->dcount);
  fdbg("     SR[%08x]: %08x\n", SAM3U_HSMCI_SR,      regs->sr);
  fdbg("    IMR[%08x]: %08x\n", SAM3U_HSMCI_IMR,     regs->imr);
  fdbg("FIFOCNT[%08x]: %08x\n", SAM3U_HSMCI_FIFOCNT, regs->fifocnt);
}
#endif

/****************************************************************************
 * Name: sam3u_dumpsample
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_XFRDEBUG
static void sam3u_dumpsample(struct sam3u_dev_s *priv,
                             struct sam3u_sampleregs_s *regs, const char *msg)
{
#ifdef CONFIG_DEBUG_DMA
  if (priv->dmamode)
    {
      sam3u_dmadump(priv->dma, &regs->dma, msg);
    }
#endif
  sam3u_sdiodump(&regs->hsmci, msg);
}
#endif

/****************************************************************************
 * Name: sam3u_dumpsamples
 *
 * Description:
 *   Dump all sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_HSMCI_XFRDEBUG
static void  sam3u_dumpsamples(struct sam3u_dev_s *priv)
{
  sam3u_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_SETUP], "Before setup");
#ifdef CONFIG_DEBUG_DMA
  if (priv->dmamode)
    {
      sam3u_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_ENABLE], "Before DMA enable");
    }
#endif
  sam3u_dumpsample(priv, &g_sampleregs[SAMPLENDX_AFTER_SETUP], "After setup");
  sam3u_dumpsample(priv, &g_sampleregs[SAMPLENDX_END_TRANSFER], "End of transfer");
#ifdef CONFIG_DEBUG_DMA
  if (priv->dmamode)
    {
      sam3u_dumpsample(priv, &g_sampleregs[SAMPLENDX_DMA_CALLBACK], "DMA Callback");
    }
#endif
}
#endif

/****************************************************************************
 * Name: sam3u_dmacallback
 *
 * Description:
 *   Called when HSMCI DMA completes
 *
 ****************************************************************************/

static void sam3u_dmacallback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  /* FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)arg; */

  /* We don't really do anything at the completion of DMA.  The termination
   * of the transfer is driven by the HSMCI interrupts.
   *
   * In fact, we won't normally get the DMA callback at all!  The HSMCI
   * appears to handle the End-Of-Transfer interrupt first and it will can
   * sam3u_dmastop() which will disable and clear the interrupt that performs
   * this callback.
   */

  sam3u_sample((struct sam3u_dev_s*)arg, SAMPLENDX_DMA_CALLBACK);
}

/****************************************************************************
 * Data Transfer Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_log2
 *
 * Description:
 *   Take (approximate) log base 2 of the provided number (Only works if the
 *   provided number is a power of 2).
 *
 ****************************************************************************/

static uint8_t sam3u_log2(uint16_t value)
{
  uint8_t log2 = 0;

  /* 0000 0000 0000 0001 -> return 0,
   * 0000 0000 0000 001x -> return 1,
   * 0000 0000 0000 01xx -> return 2,
   * 0000 0000 0000 1xxx -> return 3,
   * ...
   * 1xxx xxxx xxxx xxxx -> return 15,
   */

  DEBUGASSERT(value > 0);
  while (value != 1)
  {
    value >>= 1;
    log2++;
  }
  return log2;
}

/****************************************************************************
 * Name: sam3u_dataconfig
 *
 * Description:
 *   Configure the HSMCI data path for the next data transfer
 *
 ****************************************************************************/

static void sam3u_dataconfig(uint32_t timeout, uint32_t dlen, uint32_t dctrl)
{
  uint32_t regval = 0;

  /* Enable data path */

  putreg32(timeout, SAM3U_HSMCI_DTIMER); /* Set DTIMER */
  putreg32(dlen,    SAM3U_HSMCI_DLEN);   /* Set DLEN */

  /* Configure DCTRL DTDIR, DTMODE, and DBLOCKSIZE fields and set the DTEN
   * field
   */

  regval  =  getreg32(SAM3U_HSMCI_DCTRL);
  regval &= ~(HSMCI_DCTRL_DTDIR|HSMCI_DCTRL_DTMODE|HSMCI_DCTRL_DBLOCKSIZE_MASK);
  dctrl  &=  (HSMCI_DCTRL_DTDIR|HSMCI_DCTRL_DTMODE|HSMCI_DCTRL_DBLOCKSIZE_MASK);
  regval |=  (dctrl|HSMCI_DCTRL_DTEN);
  putreg32(regval, SAM3U_HSMCI_DCTRL);
}

/****************************************************************************
 * Name: sam3u_datadisable
 *
 * Description:
 *   Disable the the HSMCI data path setup by sam3u_dataconfig() and
 *   disable DMA.
 *
 ****************************************************************************/

static void sam3u_datadisable(void)
{
  uint32_t regval;

  /* Disable the data path */

  putreg32(HSMCI_DTIMER_DATATIMEOUT, SAM3U_HSMCI_DTIMER); /* Reset DTIMER */
  putreg32(0,                       SAM3U_HSMCI_DLEN);   /* Reset DLEN */

  /* Reset DCTRL DTEN, DTDIR, DTMODE, DMAEN, and DBLOCKSIZE fields */

  regval  = getreg32(SAM3U_HSMCI_DCTRL);
  regval &= ~(HSMCI_DCTRL_DTEN|HSMCI_DCTRL_DTDIR|HSMCI_DCTRL_DTMODE|
              HSMCI_DCTRL_DMAEN|HSMCI_DCTRL_DBLOCKSIZE_MASK);
  putreg32(regval, SAM3U_HSMCI_DCTRL);
}

/****************************************************************************
 * Name: sam3u_sendfifo
 *
 * Description:
 *   Send SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the HSMCI device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_sendfifo(struct sam3u_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[2];
  } data;

  /* Loop while there is more data to be sent and the RX FIFO is not full */

  while (priv->remaining > 0 &&
         (getreg32(SAM3U_HSMCI_SR) & HSMCI_INT_TXFIFOF) == 0)
    {
      /* Is there a full word remaining in the user buffer? */

      if (priv->remaining >= sizeof(uint32_t))
        {
          /* Yes, transfer the word to the TX FIFO */

          data.w           = *priv->buffer++;
          priv->remaining -= sizeof(uint32_t);
        }
      else
        {
           /* No.. transfer just the bytes remaining in the user buffer,
            * padding with zero as necessary to extend to a full word.
            */

           uint8_t *ptr = (uint8_t *)priv->remaining;
           int i;

           data.w = 0;
           for (i = 0; i < priv->remaining; i++)
             {
                data.b[i] = *ptr++;
             }
 
           /* Now the transfer is finished */

           priv->remaining = 0;
         }

       /* Put the word in the FIFO */

       putreg32(data.w, SAM3U_HSMCI_FIFO);
    }
}

/****************************************************************************
 * Name: sam3u_recvfifo
 *
 * Description:
 *   Receive SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the HSMCI device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam3u_recvfifo(struct sam3u_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[2];
  } data;

  /* Loop while there is space to store the data and there is more
   * data available in the RX FIFO.
   */

  while (priv->remaining > 0 &&
         (getreg32(SAM3U_HSMCI_SR) & HSMCI_INT_RXDAVL) != 0)
    {
      /* Read the next word from the RX FIFO */

      data.w = getreg32(SAM3U_HSMCI_FIFO);
      if (priv->remaining >= sizeof(uint32_t))
        {
          /* Transfer the whole word to the user buffer */

          *priv->buffer++  = data.w;
          priv->remaining -= sizeof(uint32_t);
        }
      else
        {
          /* Transfer any trailing fractional word */

          uint8_t *ptr = (uint8_t*)priv->buffer;
          int i;

          for (i = 0; i < priv->remaining; i++)
            {
               *ptr++ = data.b[i];
            }

          /* Now the transfer is finished */

          priv->remaining = 0;
        }
    }
}

/****************************************************************************
 * Name: sam3u_eventtimeout
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

static void sam3u_eventtimeout(int argc, uint32_t arg)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)arg;

  DEBUGASSERT(argc == 1 && priv != NULL);
  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

      sam3u_endwait(priv, SDIOWAIT_TIMEOUT);
      flldbg("Timeout: remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: sam3u_endwait
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

static void sam3u_endwait(struct sam3u_dev_s *priv, sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  (void)wd_cancel(priv->waitwdog);

  /* Disable event-related interrupts and save wakeup event */

  sam3u_disablewaitints(priv, wkupevent);

  /* Wake up the waiting thread */

  sam3u_givesem(priv);
}

/****************************************************************************
 * Name: sam3u_endtransfer
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

static void sam3u_endtransfer(struct sam3u_dev_s *priv, sdio_eventset_t wkupevent)
{
  /* Disable all transfer related interrupts */

  sam3u_disablexfrints(priv);

  /* If this was a DMA transfer, make sure that DMA is stopped */

  if (priv->dmamode)
    {
      /* DMA debug instrumentation */

      sam3u_sample(priv, SAMPLENDX_END_TRANSFER);

      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer terminates
       * on an error condition.
       */

      sam3u_dmastop(priv->dma);
    }

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      sam3u_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Interrrupt Handling
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_interrupt
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

static int sam3u_interrupt(int irq, void *context)
{
  struct sam3u_dev_s *priv = &g_sdiodev;
  uint32_t enabled;
  uint32_t pending;

  /* Loop while there are pending interrupts.  Check the HSMCI status
   * register.  Mask out all bits that don't correspond to enabled
   * interrupts.  (This depends on the fact that bits are ordered
   * the same in both the SR and IMR registers).  If there are non-zero
   * bits remaining, then we have work to do here.
   */

  while ((enabled = getreg32(SAM3U_HSMCI_SR) & getreg32(SAM3U_HSMCI_IMR)) != 0)
    {
      /* Handle in progress, interrupt driven data transfers ****************/

      pending  = enabled & priv->xfrmask;
      if (pending != 0)
        {
          if (!priv->dmamode)
           {
             /* Is the RX FIFO half full or more?  Is so then we must be
              * processing a receive transaction.
             */

             if ((pending & HSMCI_INT_RXFIFOHF) != 0)
               {
                 /* Receive data from the RX FIFO */

                 sam3u_recvfifo(priv);
               }

             /* Otherwise, Is the transmit FIFO half empty or less?  If so we must
              * be processing a send transaction.  NOTE:  We can't be processing
              * both!
              */

             else if ((pending & HSMCI_INT_TXFIFOHE) != 0)
               {
                 /* Send data via the TX FIFO */

                 sam3u_sendfifo(priv);
               }
           }

          /* Handle data end events */

          if ((pending & HSMCI_INT_BLKE) != 0)
            {
              /* Handle any data remaining the RX FIFO.  If the RX FIFO is
               * less than half full at the end of the transfer, then no
               * half-full interrupt will be received.
               */

              if (!priv->dmamode)
                {
                  /* Receive data from the RX FIFO */

                  sam3u_recvfifo(priv);
                }

              /* Then terminate the transfer */

              sam3u_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
            }

          /* Handle data block send/receive CRC failure */

          else if ((pending & HSMCI_INT_DCRCFAIL) != 0)
            {
              /* Terminate the transfer with an error */

              flldbg("ERROR: Data block CRC failure, remaining: %d\n", priv->remaining);
              sam3u_endtransfer(priv, SDIOWAIT_TRANSFERDONE|SDIOWAIT_ERROR);
            }

          /* Handle data timeout error */

          else if ((pending & HSMCI_INT_DTOE) != 0)
            {
              /* Terminate the transfer with an error */

              flldbg("ERROR: Data timeout, remaining: %d\n", priv->remaining);
              sam3u_endtransfer(priv, SDIOWAIT_TRANSFERDONE|SDIOWAIT_TIMEOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & HSMCI_INT_OVRE) != 0)
            {
              /* Terminate the transfer with an error */

              flldbg("ERROR: RX FIFO overrun, remaining: %d\n", priv->remaining);
              sam3u_endtransfer(priv, SDIOWAIT_TRANSFERDONE|SDIOWAIT_ERROR);
            }

          /* Handle TX FIFO underrun error */

          else if ((pending & HSMCI_INT_UNRE) != 0)
            {
              /* Terminate the transfer with an error */

              flldbg("ERROR: TX FIFO underrun, remaining: %d\n", priv->remaining);
              sam3u_endtransfer(priv, SDIOWAIT_TRANSFERDONE|SDIOWAIT_ERROR);
            }

          /* Handle start bit error */

          else if ((pending & HSMCI_INT_RENDE) != 0)
            {
              /* Terminate the transfer with an error */

              flldbg("ERROR: Start bit, remaining: %d\n", priv->remaining);
              sam3u_endtransfer(priv, SDIOWAIT_TRANSFERDONE|SDIOWAIT_ERROR);
           }
        }

      /* Handle wait events *************************************************/

      pending  = enabled & priv->waitmask;
      if (pending != 0)
        {
          /* Is this a response completion event? */

          if ((pending & HSMCI_RESPONSE_INTS) != 0)
            {
              /* Yes.. Is their a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  sam3u_endwait(priv, SDIOWAIT_RESPONSEDONE);
                }
            }

          /* Is this a command completion event? */

          if ((pending & HSMCI_CMDDONE_INTS) != 0)
            {
              /* Yes.. Is their a thread waiting for command done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  sam3u_endwait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * SDIO Interface Methods
 ****************************************************************************/
/****************************************************************************
 * Name: sam3u_reset
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

static void sam3u_reset(FAR struct sdio_dev_s *dev)
{
  FAR struct sam3u_dev_s *priv = (FAR struct sam3u_dev_s *)dev;
  irqstate_t flags;

  /* Enable the MCI clock */

  flags = irqsave();
  putreg32((1 << SAM3U_PID_HSMCI), SAM3U_PMC_PCER);
  
  /* Reset the MCI */

  putreg32(HSMCI_CR_SWRST, SAM3U_HSMCI_CR);
  
  /* Disable the MCI */

  putreg32(HSMCI_CR_MCIDIS | HSMCI_CR_PWSDIS, SAM3U_HSMCI_CR);
  
  /* Disable all the interrupts */

  putreg32(0xffffffff, SAM3U_HSMCI_IDR);
  
  /* Set the Data Timeout Register */

  putreg32(HSMCI_DTOR_DTOCYC_MAX | HSMCI_DTOR_DTOMUL_MAX, SAM3U_HSMCI_DTOR);
  
  /* Set the Mode Register: 400KHz for MCK = 48MHz (clkdiv = 58) */

  putreg32(HSMCI_CLCKCR_INIT, SAM3U_HSMCI_MR);
  
  /* Set the SDCard Register */

  putreg32(HSMCI_SDCR_SDCSEL_SLOTA | HSMCI_SDCR_SDCBUS_4BIT, SAM3U_HSMCI_SDCR);

  /* Enable the MCI and the Power Saving */

  putreg32(HSMCI_CR_MCIEN, SAM3U_HSMCI_CR);

  /* Disable the DMA interface */

  putreg32(0, SAM3U_HSMCI_DMA);
  
  /* Configure MCI */

  putreg32(HSMCI_CFG_FIFOMODE, SAM3U_HSMCI_CFG);

  /* Disable the MCI peripheral clock */

  putreg32((1 << SAM3U_PID_HSMCI), SAM3U_PMC_PCDR);

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */
  wd_cancel(priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

  /* DMA data transfer support */

  priv->widebus    = false;  /* Required for DMA support */
  priv->dmamode    = false;  /* true: DMA mode transfer */
  irqrestore(flags);
}

/****************************************************************************
 * Name: sam3u_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see sam3u_status_* defines)
 *
 ****************************************************************************/

static uint8_t sam3u_status(FAR struct sdio_dev_s *dev)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: HSMCI_WIDEBUS
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

static void sam3u_widebus(FAR struct sdio_dev_s *dev, bool wide)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
  priv->widebus = wide;
}

/****************************************************************************
 * Name: sam3u_clock
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

static void sam3u_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint32_t clckr;
  uint32_t enable = 1;

  switch (rate)
    {
    default:
    case CLOCK_HSMCI_DISABLED:     /* Clock is disabled */
      clckr  = HSMCI_CLCKCR_INIT;
      enable = 0;
      return;

    case CLOCK_IDMODE:            /* Initial ID mode clocking (<400KHz) */
      clckr  = HSMCI_CLCKCR_INIT;
      break;

    case CLOCK_MMC_TRANSFER:      /* MMC normal operation clocking */
      clckr  = HSMCI_CLKCR_MMCXFR;
      break;

    case CLOCK_SD_TRANSFER_1BIT:  /* SD normal operation clocking (narrow 1-bit mode) */
      clckr  = HSMCI_CLCKR_SDXFR;
      break;

    case CLOCK_SD_TRANSFER_4BIT:  /* SD normal operation clocking (wide 4-bit mode) */
      clckr  = HSMCI_CLCKR_SDWIDEXFR;
      break;
    };

  /* Set the new clock frequency and make sure that the clock is enabled or
   * disabled, whatever the case.
   */

  sam3u_setclkcr(clckr);
  putreg32(enable, HSMCI_CLKCR_CLKEN_BB);
}

/****************************************************************************
 * Name: sam3u_attach
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

static int sam3u_attach(FAR struct sdio_dev_s *dev)
{
  int ret;

  /* Attach the HSMCI interrupt handler */

  ret = irq_attach(SAM3U_IRQ_HSMCI, sam3u_interrupt);
  if (ret == OK)
    {

      /* Disable all interrupts at the HSMCI controller and clear (most) static
       * interrupt flags by reading the status register.
       */

      putreg32(0xffffffff, SAM3U_HSMCI_IDR);
      (void)getreg32(SAM3U_HSMCI_SR);

      /* Enable HSMCI interrupts at the NVIC.  They can now be enabled at
       * the HSMCI controller as needed.
       */

      up_enable_irq(SAM3U_IRQ_HSMCI);

      /* Set the interrrupt priority */

      up_prioritize_irq(SAM3U_IRQ_HSMCI, CONFIG_HSMCI_PRI);
    }

  return ret;
}

/****************************************************************************
 * Name: sam3u_sendcmd
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

static void sam3u_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t arg)
{
  uint32_t regval;
  uint32_t cmdidx = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;

  /* Set the HSMCI Argument value */

  putreg32(arg, SAM3U_HSMCI_ARG);

  /* Clear CMDINDEX, WAITRESP, WAITINT, WAITPEND, and CPSMEN bits */

  regval = getreg32(SAM3U_HSMCI_CMDR);
  regval &= ~(HSMCI_CMDR_CMDINDEX_MASK|HSMCI_CMDR_WAITRESP_MASK|
              HSMCI_CMDR_WAITINT|HSMCI_CMDR_WAITPEND|HSMCI_CMDR_CPSMEN);

  /* Set WAITRESP bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      regval |= HSMCI_CMDR_NORESPONSE;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= HSMCI_CMDR_SHORTRESPONSE;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= HSMCI_CMDR_LONGRESPONSE;
      break;
    }

  /* Set CPSMEN and the command index */

  cmdidx  = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= cmdidx | HSMCI_CMDR_CPSMEN;
  
  fvdbg("cmd: %08x arg: %08x regval: %08x\n", cmd, arg, regval);

  /* Write the SDIO CMD */

  putreg32(regval, SAM3U_HSMCI_CMDR);
}

/****************************************************************************
 * Name: sam3u_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data trasfer from the card in non-DMA
 *   (interrupt driven mode).  This method will do whatever controller setup
 *   is necessary.  This would be called for SD memory just BEFORE sending
 *   CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally, HSMCI_WAITEVENT
 *   will be called to receive the indication that the transfer is complete.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer in which to receive the data
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int sam3u_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                           size_t nbytes)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  sam3u_datadisable();
  sam3u_sampleinit();
  sam3u_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t*)buffer;
  priv->remaining = nbytes;
  priv->dmamode   = false;

  /* Then set up the SDIO data path */

  dblocksize = sam3u_log2(nbytes) << HSMCI_DCTRL_DBLOCKSIZE_SHIFT;
  sam3u_dataconfig(HSMCI_DTIMER_DATATIMEOUT, nbytes, dblocksize|HSMCI_DCTRL_DTDIR);

  /* And enable interrupts */

  sam3u_enablexfrints(priv, HSMCI_RECV_INTS);
  sam3u_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: sam3u_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data trasfer from the card.  This method
 *   will do whatever controller setup is necessary.  This would be called
 *   for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK), ... and before HSMCI_SENDDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - Address of the buffer containing the data to send
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int sam3u_sendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer,
                           size_t nbytes)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  sam3u_datadisable();
  sam3u_sampleinit();
  sam3u_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t*)buffer;
  priv->remaining = nbytes;
  priv->dmamode   = false;

  /* Then set up the HSMCI data path */

  dblocksize = sam3u_log2(nbytes) << HSMCI_DCTRL_DBLOCKSIZE_SHIFT;
  sam3u_dataconfig(HSMCI_DTIMER_DATATIMEOUT, nbytes, dblocksize);

  /* Enable TX interrrupts */

  sam3u_enablexfrints(priv, HSMCI_SEND_INTS);
  sam3u_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: sam3u_cancel
 *
 * Description:
 *   Cancel the data transfer setup of HSMCI_RECVSETUP, HSMCI_SENDSETUP,
 *   HSMCI_DMARECVSETUP or HSMCI_DMASENDSETUP.  This must be called to cancel
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

static int sam3u_cancel(FAR struct sdio_dev_s *dev)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;

  /* Disable all transfer- and event- related interrupts */

  sam3u_disablexfrints(priv);
  sam3u_disablewaitints(priv, 0);

  /* Clearing (most) pending interrupt status by reading the status register */
 
  (void)getreg32(SAM3U_HSMCI_SR);

  /* Cancel any watchdog timeout */

  (void)wd_cancel(priv->waitwdog);

  /* If this was a DMA transfer, make sure that DMA is stopped */

  if (priv->dmamode)
    {
      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer terminates
       * on an error condition.
       */

      sam3u_dmastop(priv->dma);
    }

  /* Mark no transfer in progress */

  priv->remaining = 0;
  return OK;
}

/****************************************************************************
 * Name: sam3u_waitresponse
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

static int sam3u_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd)
{
  int32_t timeout;
  uint32_t events;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      events  = HSMCI_CMDDONE_INTS;
      timeout = HSMCI_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      events  = HSMCI_RESPONSE_INTS;
      timeout = HSMCI_LONGTIMEOUT;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
      return -ENOSYS;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      events  = HSMCI_RESPONSE_INTS;
      timeout = HSMCI_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the response (or timeout) */

  while ((getreg32(SAM3U_HSMCI_SR) & events) == 0)
    {
      if (--timeout <= 0)
        {
          fdbg("ERROR: Timeout cmd: %08x events: %08x SR: %08x\n",
               cmd, events, getreg32(SAM3U_HSMCI_SR));

          return -ETIMEDOUT;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sam3u_recvRx
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
 *   failure means only a faiure to obtain the requested reponse (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intacta and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int sam3u_recvshortcrc(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *rshort)
{
#ifdef CONFIG_DEBUG
  uint32_t respcmd;
#endif
  uint32_t regval;
  int ret = OK;

  /* R1  Command response (48-bit)
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
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE)
    {
      fdbg("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(SAM3U_HSMCI_SR);
      if ((regval & HSMCI_INT_RTOE) != 0)
        {
          fdbg("ERROR: Command timeout: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & HSMCI_INT_RCRCE) != 0)
        {
          fdbg("ERROR: CRC failuret: %08x\n", regval);
          ret = -EIO;
        }
#ifdef CONFIG_DEBUG
      else
        {
          /* Check response received is of desired command */

          respcmd = getreg32(SAM3U_HSMCI_RESPCMD);
          if ((uint8_t)(respcmd & HSMCI_RESPCMD_MASK) != (cmd & MMCSD_CMDIDX_MASK))
            {
              fdbg("ERROR: RESCMD=%02x CMD=%08x\n", respcmd, cmd);
              ret = -EINVAL;
            }
        }
#endif
    }

  /* Clear all pending message completion events and return the R1/R6 response */

  *rshort = getreg32(SAM3U_HSMCI_RSPR0);
  return ret;
}

static int sam3u_recvlong(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t rlong[4])
{
  uint32_t regval;
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
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(SAM3U_HSMCI_SR);
      if (regval & HSMCI_INT_RTOE)
        {
          fdbg("ERROR: Timeout SR: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & HSMCI_INT_RCRCE)
        {
          fdbg("ERROR: CRC fail SR: %08x\n", regval);
          ret = -EIO;
        }
    }
    
  /* Return the long response */

  if (rlong)
    {
      rlong[0] = getreg32(SAM3U_HSMCI_RSPR0);
      rlong[1] = getreg32(SAM3U_HSMCI_RSPR1);
      rlong[2] = getreg32(SAM3U_HSMCI_RSPR2);
      rlong[3] = getreg32(SAM3U_HSMCI_RSPR3);
    }
  return ret;
}

static int sam3u_recvshort(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;

 /* R3  OCR (48-bit)
  *     47        0               Start bit
  *     46        0               Transmission bit (0=from card)
  *     45:40     bit5   - bit0   Reserved
  *     39:8      bit31  - bit0   32-bit OCR register
  *     7:1       bit6   - bit0   Reserved
  *     0         1               End bit
  */

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      fdbg("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout occurred (Apparently a CRC error can terminate
       * a good response)
       */

      regval = getreg32(SAM3U_HSMCI_SR);
      if (regval & HSMCI_INT_RTOE)
        {
          fdbg("ERROR: Timeout SR: %08x\n", regval);
          ret = -ETIMEDOUT;
        }

      /* Return the short response */
    }

  if (rshort)
    {
      *rshort = getreg32(SAM3U_HSMCI_RSPR0);
    }
  return ret;
}

/* MMC responses not supported */

static int sam3u_recvnotimpl(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *rnotimpl)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: sam3u_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the HSMCI_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling sam3u_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
 *
 *   The enabled events persist until either (1) HSMCI_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) HSMCI_EVENTWAIT
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

static void sam3u_waitenable(FAR struct sdio_dev_s *dev,
                             sdio_eventset_t eventset)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;
  uint32_t waitmask;
 
  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  sam3u_disablewaitints(priv, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitmask = 0;
  if ((eventset & SDIOWAIT_CMDDONE) != 0)
    {
      waitmask |= HSMCI_CMDDONE_INTS;
    }

  if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
    {
      waitmask |= HSMCI_RESPONSE_INTS;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
      waitmask |= HSMCI_XFRDONE_INTS;
    }

  /* Enable event-related interrupts */

  (void)getreg32(SAM3U_HSMCI_SR);
  sam3u_enablewaitints(priv, waitmask, eventset);
}

/****************************************************************************
 * Name: sam3u_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by HSMCI_WAITEVENTS are disabled when sam3u_eventwait
 *   returns.  HSMCI_WAITEVENTS must be called again before sam3u_eventwait
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

static sdio_eventset_t sam3u_eventwait(FAR struct sdio_dev_s *dev,
                                       uint32_t timeout)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;
  sdio_eventset_t wkupevent = 0;
  int ret;

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero.
   */

  DEBUGASSERT((priv->waitevents != 0 && priv->wkupevent == 0) ||
              (priv->waitevents == 0 && priv->wkupevent != 0));

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;

      /* Yes.. Handle a cornercase */

      if (!timeout)
        {
           return SDIOWAIT_TIMEOUT;
        }

      /* Start the watchdog timer */

      delay = (timeout + (MSEC_PER_TICK-1)) / MSEC_PER_TICK;
      ret   = wd_start(priv->waitwdog, delay, (wdentry_t)sam3u_eventtimeout,
                       1, (uint32_t)priv);
      if (ret != OK)
        {
           fdbg("ERROR: wd_start failed: %d\n", ret);
         }
    }

  /* Loop until the event (or the timeout occurs). Race conditions are avoided
   * by calling sam3u_waitenable prior to triggering the logic that will cause
   * the wait to terminate.  Under certain race conditions, the waited-for
   * may have already occurred before this function was called!
   */

  for (;;)
    {
      /* Wait for an event in event set to occur.  If this the event has already
       * occurred, then the semaphore will already have been incremented and
       * there will be no wait.
       */

      sam3u_takesem(priv);
      wkupevent = priv->wkupevent;
 
      /* Check if the event has occurred.  When the event has occurred, then
       * evenset will be set to 0 and wkupevent will be set to a nonzero value.
       */

      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          break;
        }
    }

  /* Disable event-related interrupts */

  sam3u_disablewaitints(priv, 0);
  sam3u_dumpsamples(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: sam3u_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in sam3u_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this methos.
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

static void sam3u_callbackenable(FAR struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;

  fvdbg("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  sam3u_callback(priv);
}

/****************************************************************************
 * Name: sam3u_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to HSMCI_CALLBACKENABLE
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

static int sam3u_registercallback(FAR struct sdio_dev_s *dev,
                                  worker_t callback, void *arg)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)dev;

  /* Disable callbacks and register this callback and is argument */

  fvdbg("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: sam3u_dmasupported
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

static bool sam3u_dmasupported(FAR struct sdio_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: sam3u_dmarecvsetup
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

static int sam3u_dmarecvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer,
                              size_t buflen)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
  uint32_t dblocksize;
  int ret = -EINVAL;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  sam3u_datadisable();

  /* Wide bus operation is required for DMA */

  if (priv->widebus)
    {
      sam3u_sampleinit();
      sam3u_sample(priv, SAMPLENDX_BEFORE_SETUP);

      /* Save the destination buffer information for use by the interrupt handler */

      priv->buffer    = (uint32_t*)buffer;
      priv->remaining = buflen;
      priv->dmamode   = true;

      /* Then set up the HSMCI data path */

      dblocksize = sam3u_log2(buflen) << HSMCI_DCTRL_DBLOCKSIZE_SHIFT;
      sam3u_dataconfig(HSMCI_DTIMER_DATATIMEOUT, buflen, dblocksize|HSMCI_DCTRL_DTDIR);

      /* Configure the RX DMA */

      sam3u_enablexfrints(priv, HSMCI_DMARECV_INTS);

      putreg32(1, HSMCI_DCTRL_DMAEN_BB);
      sam3u_dmasetup(priv->dma, SAM3U_HSMCI_FIFO, (uint32_t)buffer,
                     (buflen + 3) >> 2, HSMCI_RXDMA32_CONFIG);
 
     /* Start the DMA */

      sam3u_sample(priv, SAMPLENDX_BEFORE_ENABLE);
      sam3u_dmastart(priv->dma, sam3u_dmacallback, priv, false);
      sam3u_sample(priv, SAMPLENDX_AFTER_SETUP);
      ret = OK;
    }
  return ret;
}

/****************************************************************************
 * Name: sam3u_dmasendsetup
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

static int sam3u_dmasendsetup(FAR struct sdio_dev_s *dev,
                              FAR const uint8_t *buffer, size_t buflen)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
  uint32_t dblocksize;
  int ret = -EINVAL;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  sam3u_datadisable();

  /* Wide bus operation is required for DMA */

  if (priv->widebus)
    {
      sam3u_sampleinit();
      sam3u_sample(priv, SAMPLENDX_BEFORE_SETUP);

      /* Save the source buffer information for use by the interrupt handler */

      priv->buffer    = (uint32_t*)buffer;
      priv->remaining = buflen;
      priv->dmamode   = true;

      /* Then set up the HSMCI data path */

      dblocksize = sam3u_log2(buflen) << HSMCI_DCTRL_DBLOCKSIZE_SHIFT;
      sam3u_dataconfig(HSMCI_DTIMER_DATATIMEOUT, buflen, dblocksize);

      /* Configure the TX DMA */

      sam3u_dmasetup(priv->dma, SAM3U_HSMCI_FIFO, (uint32_t)buffer,
                     (buflen + 3) >> 2, HSMCI_TXDMA32_CONFIG);

      sam3u_sample(priv, SAMPLENDX_BEFORE_ENABLE);
      putreg32(1, HSMCI_DCTRL_DMAEN_BB);

      /* Start the DMA */

      sam3u_dmastart(priv->dma, sam3u_dmacallback, priv, false);
      sam3u_sample(priv, SAMPLENDX_AFTER_SETUP);

      /* Enable TX interrrupts */

      sam3u_enablexfrints(priv, HSMCI_DMASEND_INTS);

      ret = OK;
    }
  return ret;
}

/****************************************************************************
 * Initialization/uninitialization/reset
 ****************************************************************************/
/****************************************************************************
 * Name: sam3u_callback
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

static void sam3u_callback(void *arg)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s*)arg;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  fvdbg("Callback %p(%p) cbevents: %02x cdstatus: %02x\n",
        priv->callback, priv->cbarg, priv->cbevents, priv->cdstatus);

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

      /* Callbacks cannot be performed in the context of an interrupt handler.
       * If we are in an interrupt handler, then queue the callback to be
       * performed later on the work thread.
       */

      if (up_interrupt_context())
        {
          /* Yes.. queue it */

           fvdbg("Queuing callback to %p(%p)\n", priv->callback, priv->cbarg);
          (void)work_queue(&priv->cbwork, (worker_t)priv->callback, priv->cbarg, 0);
        }
      else
        {
          /* No.. then just call the callback here */

          fvdbg("Callback to %p(%p)\n", priv->callback, priv->cbarg);
          priv->callback(priv->cbarg);
        }
    }
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
  /* There is only one slot */

  struct sam3u_dev_s *priv = &g_sdiodev;

  /* Initialize the HSMCI slot structure */

  sem_init(&priv->waitsem, 0, 0);
  priv->waitwdog = wd_create();
  DEBUGASSERT(priv->waitwdog);

  /* Allocate a DMA channel */

  priv->dma = sam3u_dmachannel();
  DEBUGASSERT(priv->dma);

  /* Configure GPIOs for 4-bit, wide-bus operation.  NOTE: (1) the chip is capable of
   * 8-bit wide bus operation but D4-D7 are not configured, (2) any card detection
   * GPIOs must be set up in board-specific logic.
   */

  sam3u_configgpio(GPIO_MCI_DAT0);   /* Data 0 of Slot A */
  sam3u_configgpio(GPIO_MCI_DAT1);   /* Data 1 of Slot A */
  sam3u_configgpio(GPIO_MCI_DAT2);   /* Data 2 of Slot A */
  sam3u_configgpio(GPIO_MCI_DAT3);   /* Data 3 of Slot A */
  sam3u_configgpio(GPIO_MCI_CK);     /* SD clock */
  sam3u_configgpio(GPIO_MCI_DA);     /* Command/Response */

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  sam3u_reset(&priv->dev);
  return &g_sdiodev.dev;
}

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic -- posssible from an interrupt handler --
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
 ****************************************************************************/

void sdio_mediachange(FAR struct sdio_dev_s *dev, bool cardinslot)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
  uint8_t cdstatus;
  irqstate_t flags;

  /* Update card status */

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
  fvdbg("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

  /* Perform any requested callback if the status has changed */

  if (cdstatus != priv->cdstatus)
    {
      sam3u_callback(priv);
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
 *   wrprotect - true is a card is writeprotected.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void sdio_wrprotect(FAR struct sdio_dev_s *dev, bool wrprotect)
{
  struct sam3u_dev_s *priv = (struct sam3u_dev_s *)dev;
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
#endif /* CONFIG_SAM3U_HSMCI */
