/****************************************************************************
 * arch/arm/src/at32/at32_sdio.c
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
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "at32.h"
#include "at32_dma.h"
#include "at32_sdio.h"

#ifdef CONFIG_AT32_SDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Required system configuration options:
 *
 *   CONFIG_ARCH_DMA - Enable architecture-specific DMA subsystem
 *     initialization.  Required if CONFIG_AT32_SDIO_DMA is enabled.
 *   CONFIG_AT32_DMA2 - Enable AT32 DMA2 support.  Required if
 *     CONFIG_AT32_SDIO_DMA is enabled
 *   CONFIG_SCHED_WORKQUEUE -- Callback support requires work queue support.
 *
 * Driver-specific configuration options:
 *
 *   CONFIG_SDIO_MUXBUS - Setting this configuration enables some locking
 *     APIs to manage concurrent accesses on the SDIO bus.  This is not
 *     needed for the simple case of a single SD card, for example.
 *   CONFIG_AT32_SDIO_DMA - Enable SDIO.  This is a marginally optional.
 *     For most usages, SDIO will cause data overruns if used without DMA.
 *     NOTE the above system DMA configuration options.
 *   CONFIG_AT32_SDIO_WIDTH_D1_ONLY - This may be selected to force the
 *     driver operate with only a single data line (the default is to use
 *     all 4 SD data lines).
 *   CONFIG_SDM_DMAPRIO - SDIO DMA priority.  This can be selected if
 *     CONFIG_AT32_SDIO_DMA is enabled.
 *   CONFIG_SDIO_XFRDEBUG - Enables some very low-level debug output
 *     This also requires CONFIG_DEBUG_FS and CONFIG_DEBUG_INFO
 */

#if !defined(CONFIG_AT32_SDIO_DMA)
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#else
#  ifndef CONFIG_AT32_DMA2
#    error "CONFIG_AT32_SDIO_DMA support requires CONFIG_AT32_DMA2"
#  endif
#  ifndef CONFIG_SDIO_DMA
#    error CONFIG_SDIO_DMA must be defined with CONFIG_AT32_SDIO_DMA
#  endif
#endif

#ifndef CONFIG_AT32_SDIO_DMA
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#endif

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifdef CONFIG_AT32_SDIO_DMA
#  ifndef CONFIG_AT32_SDIO_DMAPRIO
#    if defined(CONFIG_AT32_AT32F43XX)
#      define CONFIG_AT32_SDIO_DMAPRIO  DMA_CCR_PRIMED  
#    else
#      error "Unknown AT32 DMA"
#    endif
#  endif
#  if defined(CONFIG_AT32_AT32F43XX)
#    if (CONFIG_AT32_SDIO_DMAPRIO & ~DMA_CCR_PL_MASK) != 0
#      error "Illegal value for CONFIG_AT32_SDIO_DMAPRIO"
#    endif
#  else
#    error "Unknown AT32 DMA"
#  endif
#else
#  undef CONFIG_AT32_SDIO_DMAPRIO
#endif

#ifndef CONFIG_DEBUG_MEMCARD_INFO
#  undef CONFIG_SDIO_XFRDEBUG
#endif

/* Enable the SDIO pull-up resistors if needed */

#ifdef CONFIG_AT32_SDIO_PULLUP
#  define SDIO_PULLUP_ENABLE GPIO_PULLUP
#else
#  define SDIO_PULLUP_ENABLE 0
#endif

/* Friendly CLKCR bit re-definitions ****************************************/

#define SDIO_CLKCR_RISINGEDGE    (0)
#define SDIO_CLKCR_FALLINGEDGE   SDIO_CLKCR_NEGEDGE

/* Use the default of the rising edge but allow a configuration,
 * that does not have the errata, to override the edge the SDIO
 * command and data is changed on.
 */

#if !defined(SDIO_CLKCR_EDGE)
#  define SDIO_CLKCR_EDGE SDIO_CLKCR_RISINGEDGE
#endif

/* Mode dependent settings.  These depend on clock divisor settings that must
 * be defined in the board-specific board.h header file: SDIO_INIT_CLKDIV,
 * SDIO_MMCXFR_CLKDIV, and SDIO_SDXFR_CLKDIV.
 */

#if (SDIO_INIT_CLKDIV > 0xff)
#define AT32_CLCKCR_INIT        ((SDIO_INIT_CLKDIV & 0xff) | SDIO_CLKCR_EDGE | \
                                  SDIO_CLKCR_WIDBUS_D1 | ((SDIO_INIT_CLKDIV >> 8) << SDIO_CLKCR_CLKDIV89_SHIFT))
#else
#define AT32_CLCKCR_INIT        (SDIO_INIT_CLKDIV | SDIO_CLKCR_EDGE | \
                                  SDIO_CLKCR_WIDBUS_D1)
#endif

#define SDIO_CLKCR_MMCXFR        (SDIO_MMCXFR_CLKDIV | SDIO_CLKCR_EDGE | \
                                  SDIO_CLKCR_WIDBUS_D1)
#define SDIO_CLCKR_SDXFR         (SDIO_SDXFR_CLKDIV | SDIO_CLKCR_EDGE | \
                                  SDIO_CLKCR_WIDBUS_D1)
#define SDIO_CLCKR_SDWIDEXFR     (SDIO_SDXFR_CLKDIV | SDIO_CLKCR_EDGE | \
                                  SDIO_CLKCR_WIDBUS_D4)

/* Timing */

#define SDIO_CMDTIMEOUT          (100000)
#define SDIO_LONGTIMEOUT         (0x7fffffff)

/* DTIMER setting */

/* Assuming Max timeout in bypass 48 Mhz */

#define IP_CLCK_FREQ               UINT32_C(48000000)
#define SDIO_DTIMER_DATATIMEOUT_MS 250

/* DMA channel/stream configuration register settings.  The following
 * must be selected.  The DMA driver will select the remaining fields.
 *
 * - 32-bit DMA
 * - Memory increment
 * - Direction (memory-to-peripheral, peripheral-to-memory)
 * - Memory burst size (F4 only)
 */

/* AT32 F1 channel configuration register (CCR) settings */

#if defined(CONFIG_AT32_AT32F43XX)
#  define SDIO_RXDMA32_CONFIG    (CONFIG_AT32_SDIO_DMAPRIO | DMA_CCR_MSIZE_32BITS | \
                                  DMA_CCR_PSIZE_32BITS | DMA_CCR_MINC)
#  define SDIO_TXDMA32_CONFIG    (CONFIG_AT32_SDIO_DMAPRIO | DMA_CCR_MSIZE_32BITS | \
                                  DMA_CCR_PSIZE_32BITS | DMA_CCR_MINC | DMA_CCR_DIR)
#else
#  error "Unknown AT32 DMA"
#endif

/* SDIO DMA Channel/Stream selection.  For the case of the AT32 F4, there
 * are multiple DMA stream options that must be dis-ambiguated in the board.h
 * file.
 */

#if defined(CONFIG_AT32_AT32F43XX)
#  define SDIO_DMACHAN           DMAMUX_SDIO
#else
#  error "Unknown AT32 DMA"
#endif

/* FIFO sizes */

#define SDIO_HALFFIFO_WORDS      (8)
#define SDIO_HALFFIFO_BYTES      (8*4)

/* Data transfer interrupt mask bits */

#define SDIO_RECV_MASK     (SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | \
                            SDIO_MASK_DATAENDIE | SDIO_MASK_RXOVERRIE | \
                            SDIO_MASK_RXFIFOHFIE | SDIO_MASK_STBITERRIE)
#define SDIO_SEND_MASK     (SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | \
                            SDIO_MASK_DATAENDIE | SDIO_MASK_TXUNDERRIE | \
                            SDIO_MASK_TXFIFOHEIE | SDIO_MASK_STBITERRIE)
#define SDIO_DMARECV_MASK  (SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | \
                            SDIO_MASK_DATAENDIE | SDIO_MASK_RXOVERRIE | \
                            SDIO_MASK_STBITERRIE)
#define SDIO_DMASEND_MASK  (SDIO_MASK_DCRCFAILIE | SDIO_MASK_DTIMEOUTIE | \
                            SDIO_MASK_DATAENDIE | SDIO_MASK_TXUNDERRIE | \
                            SDIO_MASK_STBITERRIE)

/* Event waiting interrupt mask bits */

#define SDIO_CMDDONE_STA   (SDIO_STA_CMDSENT)
#define SDIO_RESPDONE_STA  (SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL | \
                            SDIO_STA_CMDREND)
#define SDIO_XFRDONE_STA   (0)

#define SDIO_CMDDONE_MASK  (SDIO_MASK_CMDSENTIE)
#define SDIO_RESPDONE_MASK (SDIO_MASK_CCRCFAILIE | SDIO_MASK_CTIMEOUTIE | \
                            SDIO_MASK_CMDRENDIE)
#define SDIO_XFRDONE_MASK  (0)

#define SDIO_CMDDONE_ICR   (SDIO_ICR_CMDSENTC | SDIO_ICR_DBCKENDC)
#define SDIO_RESPDONE_ICR  (SDIO_ICR_CTIMEOUTC | SDIO_ICR_CCRCFAILC | \
                            SDIO_ICR_CMDRENDC | SDIO_ICR_DBCKENDC)
#define SDIO_XFRDONE_ICR   (SDIO_ICR_DATAENDC | SDIO_ICR_DCRCFAILC | \
                            SDIO_ICR_DTIMEOUTC | SDIO_ICR_RXOVERRC | \
                            SDIO_ICR_TXUNDERRC | SDIO_ICR_STBITERRC | \
                            SDIO_ICR_DBCKENDC)

#define SDIO_WAITALL_ICR   (SDIO_CMDDONE_ICR | SDIO_RESPDONE_ICR | \
                            SDIO_XFRDONE_ICR | SDIO_ICR_DBCKENDC)

/* Let's wait until we have both SDIO transfer complete and DMA complete. */

#define SDIO_XFRDONE_FLAG  (1)
#define SDIO_DMADONE_FLAG  (2)
#define SDIO_ALLDONE       (3)

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
#  ifdef CONFIG_AT32_SDIO_DMA
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

#define AT32_SDIO_USE_DEFAULT_BLOCKSIZE ((uint8_t)-1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the AT32 SDIO interface */

struct at32_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SDIO interface */

  /* AT32-specific extensions */

  /* Event support */

  sem_t              waitsem;         /* Implements event waiting */
  sdio_eventset_t    waitevents;      /* Set of events to be waited for */
  uint32_t           waitmask;        /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent; /* The event that caused the wakeup */
  struct wdog_s      waitwdog;        /* Watchdog that handles event timeouts */

  /* Callback support */

  sdio_statset_t     cdstatus;        /* Card status */
  sdio_eventset_t    cbevents;        /* Set of events to be cause callbacks */
  worker_t           callback;        /* Registered callback function */
  void              *cbarg;           /* Registered callback argument */
  struct work_s      cbwork;          /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t          *buffer;          /* Address of current R/W buffer */
  size_t             remaining;       /* Number of bytes remaining in the transfer */
  uint32_t           xfrmask;         /* Interrupt enables for data transfer */

#ifdef CONFIG_AT32_SDIO_CARD
  /* Interrupt at SDIO_D1 pin, only for SDIO cards */

  uint32_t           sdiointmask;            /* AT32 SDIO register mask */
  int               (*do_sdio_card)(void *); /* SDIO card ISR */
  void               *do_sdio_arg;           /* arg for SDIO card ISR */
#endif

  /* Fixed transfer block size support */

#ifdef CONFIG_SDIO_BLOCKSETUP
  uint8_t            block_size;
#endif

  /* DMA data transfer support */

  bool               widebus;         /* Required for DMA support */
#ifdef CONFIG_AT32_SDIO_DMA
  volatile uint8_t   xfrflags;        /* Used to synchronize SDIO and
                                       * DMA completion events */
  bool               dmamode;         /* true: DMA mode transfer */
  DMA_HANDLE         dma;             /* Handle for DMA channel */
#endif
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
struct at32_sdioregs_s
{
  uint8_t  power;
  uint16_t clkcr;
  uint16_t dctrl;
  uint32_t dtimer;
  uint32_t dlen;
  uint32_t dcount;
  uint32_t sta;
  uint32_t mask;
  uint32_t fifocnt;
};

struct at32_sampleregs_s
{
  struct at32_sdioregs_s sdio;
#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_AT32_SDIO_DMA)
  struct at32_dmaregs_s  dma;
#endif
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static inline void at32_setclkcr(uint32_t clkcr);
static void at32_configwaitints(struct at32_dev_s *priv, uint32_t waitmask,
              sdio_eventset_t waitevents, sdio_eventset_t wkupevents);
static void at32_configxfrints(struct at32_dev_s *priv, uint32_t xfrmask);
static void at32_setpwrctrl(uint32_t pwrctrl);

/* DMA Helpers **************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void at32_sampleinit(void);
static void at32_sdiosample(struct at32_sdioregs_s *regs);
static void at32_sample(struct at32_dev_s *priv, int index);
static void at32_sdiodump(struct at32_sdioregs_s *regs, const char *msg);
static void at32_dumpsample(struct at32_dev_s *priv,
              struct at32_sampleregs_s *regs, const char *msg);
static void at32_dumpsamples(struct at32_dev_s *priv);
#else
#  define   at32_sampleinit()
#  define   at32_sample(priv,index)
#  define   at32_dumpsamples(priv)
#endif

#ifdef CONFIG_AT32_SDIO_DMA
static void at32_dmacallback(DMA_HANDLE handle, uint8_t status, void *arg);
#endif

/* Data Transfer Helpers ****************************************************/

static uint8_t at32_log2(uint16_t value);
static void at32_dataconfig(uint32_t timeout, uint32_t dlen,
                             uint32_t dctrl);
static void at32_datadisable(void);
static void at32_sendfifo(struct at32_dev_s *priv);
static void at32_recvfifo(struct at32_dev_s *priv);
static void at32_eventtimeout(wdparm_t arg);
static void at32_endwait(struct at32_dev_s *priv,
                          sdio_eventset_t wkupevent);
static void at32_endtransfer(struct at32_dev_s *priv,
                              sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  at32_interrupt(int irq, void *context, void *arg);
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
static int  at32_rdyinterrupt(int irq, void *context, void *arg);
#endif

/* SDIO interface methods ***************************************************/

/* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
static int at32_lock(struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void at32_reset(struct sdio_dev_s *dev);
static sdio_capset_t at32_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t at32_status(struct sdio_dev_s *dev);
static void at32_widebus(struct sdio_dev_s *dev, bool enable);
static void at32_clock(struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  at32_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  at32_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);
#ifdef CONFIG_SDIO_BLOCKSETUP
static void at32_blocksetup(struct sdio_dev_s *dev,
              unsigned int blocklen, unsigned int nblocks);
#endif
static int  at32_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
              size_t nbytes);
static int  at32_sendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t nbytes);
static int  at32_cancel(struct sdio_dev_s *dev);

static int  at32_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  at32_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  at32_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  at32_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);

/* EVENT handler */

static void at32_waitenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t at32_eventwait(struct sdio_dev_s *dev);
static void at32_callbackenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  at32_registercallback(struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_AT32_SDIO_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
static int  at32_dmapreflight(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t buflen);
#endif
static int  at32_dmarecvsetup(struct sdio_dev_s *dev,
              uint8_t *buffer, size_t buflen);
static int  at32_dmasendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void at32_callback(void *arg);
static void at32_default(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct at32_dev_s g_sdiodev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = at32_lock,
#endif
    .reset            = at32_reset,
    .capabilities     = at32_capabilities,
    .status           = at32_status,
    .widebus          = at32_widebus,
    .clock            = at32_clock,
    .attach           = at32_attach,
    .sendcmd          = at32_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
    .blocksetup       = at32_blocksetup,
#endif
    .recvsetup        = at32_recvsetup,
    .sendsetup        = at32_sendsetup,
    .cancel           = at32_cancel,
    .waitresponse     = at32_waitresponse,
    .recv_r1          = at32_recvshortcrc,
    .recv_r2          = at32_recvlong,
    .recv_r3          = at32_recvshort,
    .recv_r4          = at32_recvshort,
    .recv_r5          = at32_recvshortcrc,
    .recv_r6          = at32_recvshortcrc,
    .recv_r7          = at32_recvshort,
    .waitenable       = at32_waitenable,
    .eventwait        = at32_eventwait,
    .callbackenable   = at32_callbackenable,
    .registercallback = at32_registercallback,
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_AT32_SDIO_DMA
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
    .dmapreflight     = at32_dmapreflight,
#endif
    .dmarecvsetup     = at32_dmarecvsetup,
    .dmasendsetup     = at32_dmasendsetup,
#else
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
    .dmapreflight     = NULL,
#endif
    .dmarecvsetup     = at32_recvsetup,
    .dmasendsetup     = at32_sendsetup,
#endif
#endif
  },
  .waitsem = SEM_INITIALIZER(0),
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
static struct at32_sampleregs_s g_sampleregs[DEBUG_NSAMPLES];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_setclkcr
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

static inline void at32_setclkcr(uint32_t clkcr)
{
  uint32_t regval = getreg32(AT32_SDIO_CLKCR);

  /* Clear CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, HWFC_EN bits */

  regval &= ~(SDIO_CLKCR_CLKDIV89_MASK | SDIO_CLKCR_CLKDIV_MASK | \
              SDIO_CLKCR_PWRSAV | SDIO_CLKCR_BYPASS | \
              SDIO_CLKCR_WIDBUS_MASK | SDIO_CLKCR_NEGEDGE | \
              SDIO_CLKCR_HWFC_EN | SDIO_CLKCR_CLKEN);

  /* Replace with user provided settings */

  clkcr  &=  (SDIO_CLKCR_CLKDIV89_MASK | SDIO_CLKCR_CLKDIV_MASK | \
              SDIO_CLKCR_PWRSAV | SDIO_CLKCR_BYPASS | \
              SDIO_CLKCR_WIDBUS_MASK | SDIO_CLKCR_NEGEDGE | \
              SDIO_CLKCR_HWFC_EN | SDIO_CLKCR_CLKEN);

  regval |=  clkcr;
  putreg32(regval, AT32_SDIO_CLKCR);

  mcinfo("CLKCR: %08" PRIx32 " PWR: %08" PRIx32 "\n",
         getreg32(AT32_SDIO_CLKCR), getreg32(AT32_SDIO_POWER));
}

/****************************************************************************
 * Name: at32_configwaitints
 *
 * Description:
 *   Enable/disable SDIO interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SDIO device state structure
 *   waitmask   - The set of bits in the SDIO MASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void at32_configwaitints(struct at32_dev_s *priv, uint32_t waitmask,
                                 sdio_eventset_t waitevents,
                                 sdio_eventset_t wkupevent)
{
  irqstate_t flags;
#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
  int pinset;
#endif

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags = enter_critical_section();

#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
  if ((waitevents & SDIOWAIT_WRCOMPLETE) != 0)
    {
      pinset = GPIO_SDIO_D0 & (GPIO_PORT_MASK | GPIO_PIN_MASK);
      pinset |= (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI);

      /* Arm the SDIO_D0 Ready and install Isr */

      at32_gpiosetevent(pinset, true, false, false,
                         at32_rdyinterrupt, priv);
    }

  /* Disarm SDIO_D0 ready and return it to SDIO D0 */

  if ((wkupevent & SDIOWAIT_WRCOMPLETE) != 0)
    {
      at32_gpiosetevent(GPIO_SDIO_D0, false, false, false,
                         NULL, NULL);
    }
#endif

  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = waitmask;
#ifdef CONFIG_AT32_SDIO_DMA
  priv->xfrflags   = 0;
#endif

#ifdef CONFIG_AT32_SDIO_CARD
  putreg32(priv->xfrmask | priv->waitmask | priv->sdiointmask,
           AT32_SDIO_MASK);
#else
  putreg32(priv->xfrmask | priv->waitmask, AT32_SDIO_MASK);
#endif

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: at32_configxfrints
 *
 * Description:
 *   Enable SDIO interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SDIO device state structure
 *   xfrmask - The set of bits in the SDIO MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void at32_configxfrints(struct at32_dev_s *priv, uint32_t xfrmask)
{
  irqstate_t flags;

  flags = enter_critical_section();
  priv->xfrmask = xfrmask;
#ifdef CONFIG_AT32_SDIO_CARD
  putreg32(priv->xfrmask | priv->waitmask | priv->sdiointmask,
           AT32_SDIO_MASK);
#else
  putreg32(priv->xfrmask | priv->waitmask, AT32_SDIO_MASK);
#endif
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: at32_setpwrctrl
 *
 * Description:
 *   Change the PWRCTRL field of the SDIO POWER register to turn the SDIO
 *   ON or OFF
 *
 * Input Parameters:
 *   clkcr - A new PWRCTRL setting
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void at32_setpwrctrl(uint32_t pwrctrl)
{
  uint32_t regval;

  regval  = getreg32(AT32_SDIO_POWER);
  regval &= ~SDIO_POWER_PWRCTRL_MASK;
  regval |= pwrctrl;
  putreg32(regval, AT32_SDIO_POWER);
}

/****************************************************************************
 * Name: at32_sampleinit
 *
 * Description:
 *   Setup prior to collecting DMA samples
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void at32_sampleinit(void)
{
  memset(g_sampleregs, 0xff,
         DEBUG_NSAMPLES * sizeof(struct at32_sampleregs_s));
}
#endif

/****************************************************************************
 * Name: at32_sdiosample
 *
 * Description:
 *   Sample SDIO registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void at32_sdiosample(struct at32_sdioregs_s *regs)
{
  regs->power   = (uint8_t)getreg32(AT32_SDIO_POWER);
  regs->clkcr   = (uint16_t)getreg32(AT32_SDIO_CLKCR);
  regs->dctrl   = (uint16_t)getreg32(AT32_SDIO_DCTRL);
  regs->dtimer  = getreg32(AT32_SDIO_DTIMER);
  regs->dlen    = getreg32(AT32_SDIO_DLEN);
  regs->dcount  = getreg32(AT32_SDIO_DCOUNT);
  regs->sta     = getreg32(AT32_SDIO_STA);
  regs->mask    = getreg32(AT32_SDIO_MASK);
  regs->fifocnt = getreg32(AT32_SDIO_FIFOCNT);
}
#endif

/****************************************************************************
 * Name: at32_sample
 *
 * Description:
 *   Sample SDIO/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void at32_sample(struct at32_dev_s *priv, int index)
{
  struct at32_sampleregs_s *regs = &g_sampleregs[index];

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_AT32_SDIO_DMA)
  if (priv->dmamode)
    {
      at32_dmasample(priv->dma, &regs->dma);
    }
#endif

  at32_sdiosample(&regs->sdio);
}
#endif

/****************************************************************************
 * Name: at32_sdiodump
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void at32_sdiodump(struct at32_sdioregs_s *regs, const char *msg)
{
  mcinfo("SDIO Registers: %s\n", msg);
  mcinfo("  POWER[%08x]: %08x\n", AT32_SDIO_POWER,   regs->power);
  mcinfo("  CLKCR[%08x]: %08x\n", AT32_SDIO_CLKCR,   regs->clkcr);
  mcinfo("  DCTRL[%08x]: %08x\n", AT32_SDIO_DCTRL,   regs->dctrl);
  mcinfo(" DTIMER[%08x]: %08x\n", AT32_SDIO_DTIMER,  regs->dtimer);
  mcinfo("   DLEN[%08x]: %08x\n", AT32_SDIO_DLEN,    regs->dlen);
  mcinfo(" DCOUNT[%08x]: %08x\n", AT32_SDIO_DCOUNT,  regs->dcount);
  mcinfo("    STA[%08x]: %08x\n", AT32_SDIO_STA,     regs->sta);
  mcinfo("   MASK[%08x]: %08x\n", AT32_SDIO_MASK,    regs->mask);
  mcinfo("FIFOCNT[%08x]: %08x\n", AT32_SDIO_FIFOCNT, regs->fifocnt);
}
#endif

/****************************************************************************
 * Name: at32_dumpsample
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void at32_dumpsample(struct at32_dev_s *priv,
                             struct at32_sampleregs_s *regs,
                             const char *msg)
{
#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_AT32_SDIO_DMA)
  if (priv->dmamode)
    {
      at32_dmadump(priv->dma, &regs->dma, msg);
    }
#endif

  at32_sdiodump(&regs->sdio, msg);
}
#endif

/****************************************************************************
 * Name: at32_dumpsamples
 *
 * Description:
 *   Dump all sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void at32_dumpsamples(struct at32_dev_s *priv)
{
  at32_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_SETUP],
                   "Before setup");

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_AT32_SDIO_DMA)
  if (priv->dmamode)
    {
      at32_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_ENABLE],
                       "Before DMA enable");
    }
#endif

  at32_dumpsample(priv, &g_sampleregs[SAMPLENDX_AFTER_SETUP],
                   "After setup");
  at32_dumpsample(priv, &g_sampleregs[SAMPLENDX_END_TRANSFER],
                   "End of transfer");

#if defined(CONFIG_DEBUG_DMA_INFO) && defined(CONFIG_AT32_SDIO_DMA)
  if (priv->dmamode)
    {
      at32_dumpsample(priv, &g_sampleregs[SAMPLENDX_DMA_CALLBACK],
                       "DMA Callback");
    }
#endif
}
#endif

/****************************************************************************
 * Name: at32_dmacallback
 *
 * Description:
 *   Called when SDIO DMA completes
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_SDIO_DMA
static void at32_dmacallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)arg;
  DEBUGASSERT(priv->dmamode);
  sdio_eventset_t result;

  /* In the normal case, SDIO appears to handle the End-Of-Transfer interrupt
   * first with the End-Of-DMA event occurring significantly later.  On
   * transfer errors, however, the DMA error will occur before the End-of-
   * Transfer.
   */

  at32_sample((struct at32_dev_s *)arg, SAMPLENDX_DMA_CALLBACK);

  /* Get the result of the DMA transfer */

  if ((status & DMA_STATUS_ERROR) != 0)
    {
      mcerr("ERROR: DMA error %02x, remaining: %d\n",
            status, priv->remaining);
      result = SDIOWAIT_ERROR;
    }
  else
    {
      result = SDIOWAIT_TRANSFERDONE;
    }

  /* Then terminate the transfer if this completes all of the steps in the
   * transfer OR if a DMA error occurred.  In the non-error case, we should
   * already have the SDIO transfer done interrupt.  If not, the transfer
   * will appropriately time out.
   */

  priv->xfrflags |= SDIO_DMADONE_FLAG;
  if (priv->xfrflags == SDIO_ALLDONE || result == SDIOWAIT_ERROR)
    {
      at32_endtransfer(priv, result);
    }
}
#endif

/****************************************************************************
 * Name: at32_log2
 *
 * Description:
 *   Take (approximate) log base 2 of the provided number (Only works if the
 *   provided number is a power of 2).
 *
 ****************************************************************************/

static uint8_t at32_log2(uint16_t value)
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
 * Name: at32_dataconfig
 *
 * Description:
 *   Configure the SDIO data path for the next data transfer
 *
 ****************************************************************************/

static void at32_dataconfig(uint32_t timeout, uint32_t dlen, uint32_t dctrl)
{
  uint32_t clkdiv;
  uint32_t regval;
  uint32_t sdio_clk = AT32_SYSCLK_FREQUENCY;

  /* Enable data path using a timeout scaled to the SD_CLOCK (the card
   * clock).
   */

  regval = getreg32(AT32_SDIO_CLKCR);

  clkdiv = (regval & SDIO_CLKCR_CLKDIV_MASK) >> SDIO_CLKCR_CLKDIV_SHIFT;
  clkdiv |= ((regval & SDIO_CLKCR_CLKDIV89_MASK) >> 8);

  if ((regval & SDIO_CLKCR_BYPASS) == 0)
    {
      sdio_clk = sdio_clk / (2 + clkdiv);
    }

  /*  Convert Timeout in Ms to SD_CLK counts */

  timeout  = timeout * (sdio_clk / 1000);

  putreg32(timeout, AT32_SDIO_DTIMER); /* Set DTIMER */
  putreg32(dlen,    AT32_SDIO_DLEN);   /* Set DLEN */

  /* Configure DCTRL DTDIR, DTMODE, and DBLOCKSIZE fields and set the DTEN
   * field
   */

  regval  =  getreg32(AT32_SDIO_DCTRL);
  regval &= ~(SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTMODE |
              SDIO_DCTRL_DBLOCKSIZE_MASK);
  dctrl  &=  (SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTMODE |
              SDIO_DCTRL_DBLOCKSIZE_MASK);

#ifdef CONFIG_AT32_SDIO_CARD              
  regval |=  (dctrl | SDIO_DCTRL_DTEN | SDIO_DCTRL_SDIOEN);
#else  
  regval |=  (dctrl | SDIO_DCTRL_DTEN);
#endif

  putreg32(regval, AT32_SDIO_DCTRL);
}

/****************************************************************************
 * Name: at32_datadisable
 *
 * Description:
 *   Disable the SDIO data path setup by at32_dataconfig() and
 *   disable DMA.
 *
 ****************************************************************************/

static void at32_datadisable(void)
{
  uint32_t regval;

  /* Disable the data path  */

  /* Reset DTIMER */

  putreg32(UINT32_MAX, AT32_SDIO_DTIMER);

  /* Reset DLEN */

  putreg32(0, AT32_SDIO_DLEN);

  /* Reset DCTRL DTEN, DTDIR, DTMODE, DMAEN, and DBLOCKSIZE fields */

  regval  = getreg32(AT32_SDIO_DCTRL);
  regval &= ~(SDIO_DCTRL_DTEN | SDIO_DCTRL_DTDIR | SDIO_DCTRL_DTMODE |
              SDIO_DCTRL_DMAEN | SDIO_DCTRL_DBLOCKSIZE_MASK);
  putreg32(regval, AT32_SDIO_DCTRL);
}

/****************************************************************************
 * Name: at32_sendfifo
 *
 * Description:
 *   Send SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void at32_sendfifo(struct at32_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Loop while there is more data to be sent and the RX FIFO is not full */

  while (priv->remaining > 0 &&
         (getreg32(AT32_SDIO_STA) & SDIO_STA_TXFIFOF) == 0)
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
          for (i = 0; i < (int)priv->remaining; i++)
            {
              data.b[i] = *ptr++;
            }

          /* Now the transfer is finished */

          priv->remaining = 0;
        }

      /* Put the word in the FIFO */

      putreg32(data.w, AT32_SDIO_FIFO);
    }
}

/****************************************************************************
 * Name: at32_recvfifo
 *
 * Description:
 *   Receive SDIO data in interrupt mode
 *
 * Input Parameters:
 *   priv - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void at32_recvfifo(struct at32_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Loop while there is space to store the data and there is more
   * data available in the RX FIFO.
   */

  while (priv->remaining > 0 &&
         (getreg32(AT32_SDIO_STA) & SDIO_STA_RXDAVL) != 0)
    {
      /* Read the next word from the RX FIFO */

      data.w = getreg32(AT32_SDIO_FIFO);
      if (priv->remaining >= sizeof(uint32_t))
        {
          /* Transfer the whole word to the user buffer */

          *priv->buffer++  = data.w;
          priv->remaining -= sizeof(uint32_t);
        }
      else
        {
          /* Transfer any trailing fractional word */

          uint8_t *ptr = (uint8_t *)priv->buffer;
          int i;

          for (i = 0; i < (int)priv->remaining; i++)
            {
               *ptr++ = data.b[i];
            }

          /* Now the transfer is finished */

          priv->remaining = 0;
        }
    }
}

/****************************************************************************
 * Name: at32_eventtimeout
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

static void at32_eventtimeout(wdparm_t arg)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)arg;

  /* There is always race conditions with timer expirations. */

  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0 ||
              priv->wkupevent != 0);

  mcinfo("sta: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
         getreg32(AT32_SDIO_STA),
         getreg32(AT32_SDIO_MASK));

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
      at32_endwait(priv, SDIOWAIT_TIMEOUT |
                    (priv->waitevents & SDIOWAIT_WRCOMPLETE));
#else
      at32_endwait(priv, SDIOWAIT_TIMEOUT);
#endif
      mcerr("Timeout: remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: at32_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the SDIO device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void at32_endwait(struct at32_dev_s *priv,
                          sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  at32_configwaitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: at32_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the SDIO interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv   - An instance of the SDIO device interface
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void at32_endtransfer(struct at32_dev_s *priv,
                              sdio_eventset_t wkupevent)
{
  /* Disable all transfer related interrupts */

  at32_configxfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */

  putreg32(SDIO_XFRDONE_ICR, AT32_SDIO_ICR);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_AT32_SDIO_DMA
  if (priv->dmamode)
    {
      /* DMA debug instrumentation */

      at32_sample(priv, SAMPLENDX_END_TRANSFER);

      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer
       * terminates on an error condition).
       */

      at32_dmastop(priv->dma);
    }
#endif

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      at32_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: at32_rdyinterrupt
 *
 * Description:
 *   SDIO ready interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
static int at32_rdyinterrupt(int irq, void *context, void *arg)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)arg;

  /* Avoid noise, check the state */

  if (at32_gpioread(GPIO_SDIO_D0))
    {
      at32_endwait(priv, SDIOWAIT_WRCOMPLETE);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: at32_interrupt
 *
 * Description:
 *   SDIO interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int at32_interrupt(int irq, void *context, void *arg)
{
  struct at32_dev_s *priv = &g_sdiodev;
  uint32_t enabled;
  uint32_t pending;

  /* Loop while there are pending interrupts.  Check the SDIO status
   * register.  Mask out all bits that don't correspond to enabled
   * interrupts.  (This depends on the fact that bits are ordered
   * the same in both the STA and MASK register).  If there are non-zero
   * bits remaining, then we have work to do here.
   */

  while ((enabled = getreg32(AT32_SDIO_STA) &
                    getreg32(AT32_SDIO_MASK)) != 0)
    {
      /* Handle in progress, interrupt driven data transfers ****************/

      pending  = enabled & priv->xfrmask;
      if (pending != 0)
        {
#ifdef CONFIG_AT32_SDIO_DMA
          if (!priv->dmamode)
#endif
            {
              /* Is the RX FIFO half full or more?  Is so then we must be
               * processing a receive transaction.
               */

              if ((pending & SDIO_STA_RXFIFOHF) != 0)
                {
                  /* Receive data from the RX FIFO */

                  at32_recvfifo(priv);
                }

              /* Otherwise, Is the transmit FIFO half empty or less?  If so
               * we must be processing a send transaction.  NOTE:  We can't
               * be processing both!
               */

              else if ((pending & SDIO_STA_TXFIFOHE) != 0)
                {
                  /* Send data via the TX FIFO */

                  at32_sendfifo(priv);
                }
            }

          /* Handle data end events */

          if ((pending & SDIO_STA_DATAEND) != 0)
            {
              /* Handle any data remaining the RX FIFO.  If the RX FIFO is
               * less than half full at the end of the transfer, then no
               * half-full interrupt will be received.
               */

              /* Was this transfer performed in DMA mode? */

#ifdef CONFIG_AT32_SDIO_DMA
              if (priv->dmamode)
                {
                  /* Yes.. Terminate the transfers only if the DMA has also
                   * finished.
                   */

                  priv->xfrflags |= SDIO_XFRDONE_FLAG;
                  if (priv->xfrflags == SDIO_ALLDONE)
                    {
                      at32_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
                    }

                  /* Otherwise, just disable further transfer interrupts and
                   * wait for the DMA complete event.
                   */

                  else
                    {
                      at32_configxfrints(priv, 0);
                    }
                }
              else
#endif
                {
                  /* Receive data from the RX FIFO */

                  at32_recvfifo(priv);

                  /* Then terminate the transfer */

                  at32_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
                }
            }

          /* Handle data block send/receive CRC failure */

          else if ((pending & SDIO_STA_DCRCFAIL) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data block CRC failure, remaining: %d\n",
                    priv->remaining);
              at32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle data timeout error */

          else if ((pending & SDIO_STA_DTIMEOUT) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data timeout, remaining: %d\n",
                     priv->remaining);
              at32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & SDIO_STA_RXOVERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: RX FIFO overrun, remaining: %d\n",
                    priv->remaining);
              at32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle TX FIFO underrun error */

          else if ((pending & SDIO_STA_TXUNDERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: TX FIFO underrun, remaining: %d\n",
                    priv->remaining);
              at32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle start bit error */

          else if ((pending & SDIO_STA_STBITERR) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Start bit, remaining: %d\n",
                    priv->remaining);
              at32_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }
        }

      /* Handle wait events *************************************************/

      pending  = enabled & priv->waitmask;
      if (pending != 0)
        {
          /* Is this a response completion event? */

          if ((pending & SDIO_RESPDONE_STA) != 0)
            {
              /* Yes.. Is their a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  putreg32(SDIO_RESPDONE_ICR | SDIO_CMDDONE_ICR,
                           AT32_SDIO_ICR);
                  at32_endwait(priv, SDIOWAIT_RESPONSEDONE);
                }
            }

          /* Is this a command completion event? */

          if ((pending & SDIO_CMDDONE_STA) != 0)
            {
              /* Yes.. Is their a thread waiting for command done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  putreg32(SDIO_CMDDONE_ICR, AT32_SDIO_ICR);
                  at32_endwait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }

#ifdef CONFIG_AT32_SDIO_CARD
      /* Handle SDIO card interrupt */

      pending = enabled & priv->sdiointmask;
      if (pending != 0)
        {
          putreg32(SDIO_STA_SDIOIT, AT32_SDIO_ICR);

          /* Perform callback */

          if (priv->do_sdio_card)
            {
              priv->do_sdio_card(priv->do_sdio_arg);
            }
        }
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: at32_lock
 *
 * Description:
 *   Locks the bus. Function calls low-level multiplexed bus routines to
 *   resolve bus requests and acknowledgment issues.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   lock   - TRUE to lock, FALSE to unlock.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_MUXBUS
static int at32_lock(struct sdio_dev_s *dev, bool lock)
{
  /* Single SDIO instance so there is only one possibility.  The multiplex
   * bus is part of board support package.
   */

  at32_muxbus_sdio_lock(lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: at32_reset
 *
 * Description:
 *   Reset the SDIO controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void at32_reset(struct sdio_dev_s *dev)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;
  irqstate_t flags;

  /* Disable clocking */

  flags = enter_critical_section();
  putreg32(0, SDIO_CLKCR_CLKEN_BB);
  at32_setpwrctrl(SDIO_POWER_PWRCTRL_OFF);

  /* Put SDIO registers in their default, reset state */

  at32_default();

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */
#ifdef CONFIG_AT32_SDIO_DMA
  priv->xfrflags   = 0;      /* Used to synchronize SDIO and DMA completion events */
#endif

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */

#ifdef CONFIG_AT32_SDIO_CARD
  priv->sdiointmask = 0;     /* SDIO card in-band interrupt mask */
#endif

  /* DMA data transfer support */

  priv->widebus    = false;  /* Required for DMA support */
#ifdef CONFIG_AT32_SDIO_DMA
  priv->dmamode    = false;  /* true: DMA mode transfer */
#endif

  /* Configure the SDIO peripheral */

  at32_setclkcr(AT32_CLCKCR_INIT | SDIO_CLKCR_CLKEN);
  at32_setpwrctrl(SDIO_POWER_PWRCTRL_ON);
  leave_critical_section(flags);

  mcinfo("CLCKR: %08" PRIx32 " POWER: %08" PRIx32 "\n",
         getreg32(AT32_SDIO_CLKCR), getreg32(AT32_SDIO_POWER));
}

/****************************************************************************
 * Name: at32_capabilities
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

static sdio_capset_t at32_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

#ifdef CONFIG_AT32_SDIO_WIDTH_D1_ONLY
  caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_AT32_SDIO_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif

  return caps;
}

/****************************************************************************
 * Name: at32_status
 *
 * Description:
 *   Get SDIO status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see at32_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t at32_status(struct sdio_dev_s *dev)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;
  return priv->cdstatus;
}

/****************************************************************************
 * Name: at32_widebus
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

static void at32_widebus(struct sdio_dev_s *dev, bool wide)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;
  priv->widebus = wide;
}

/****************************************************************************
 * Name: at32_clock
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

static void at32_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  uint32_t clckr;

  switch (rate)
    {
      /* Disable clocking (with default ID mode divisor) */

      default:
      case CLOCK_SDIO_DISABLED:
        clckr = AT32_CLCKCR_INIT;
        break;

      /* Enable in initial ID mode clocking (<400KHz) */

      case CLOCK_IDMODE:
        clckr = (AT32_CLCKCR_INIT | SDIO_CLKCR_CLKEN);
        break;

      /* Enable in MMC normal operation clocking */

      case CLOCK_MMC_TRANSFER:
        clckr = (SDIO_CLKCR_MMCXFR | SDIO_CLKCR_CLKEN);
        break;

      /* SD normal operation clocking (wide 4-bit mode) */

      case CLOCK_SD_TRANSFER_4BIT:
#ifndef CONFIG_AT32_SDIO_WIDTH_D1_ONLY
        clckr = (SDIO_CLCKR_SDWIDEXFR | SDIO_CLKCR_CLKEN);
        break;
#endif

      /* SD normal operation clocking (narrow 1-bit mode) */

      case CLOCK_SD_TRANSFER_1BIT:
        clckr = (SDIO_CLCKR_SDXFR | SDIO_CLKCR_CLKEN);
        break;
    }

  /* Set the new clock frequency along with the clock enable/disable bit */

  at32_setclkcr(clckr);
}

/****************************************************************************
 * Name: at32_attach
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

static int at32_attach(struct sdio_dev_s *dev)
{
  int ret;

  /* Attach the SDIO interrupt handler */

  ret = irq_attach(AT32_IRQ_SDIO, at32_interrupt, NULL);
  if (ret == OK)
    {
      /* Disable all interrupts at the SDIO controller and clear static
       * interrupt flags
       */

      putreg32(SDIO_MASK_RESET,      AT32_SDIO_MASK);
      putreg32(SDIO_ICR_STATICFLAGS, AT32_SDIO_ICR);

      /* Enable SDIO interrupts at the NVIC.  They can now be enabled at
       * the SDIO controller as needed.
       */

      up_enable_irq(AT32_IRQ_SDIO);
    }

  return ret;
}

/****************************************************************************
 * Name: at32_sendcmd
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

static int at32_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                         uint32_t arg)
{
  uint32_t regval;
  uint32_t cmdidx;

  /* Set the SDIO Argument value */

  putreg32(arg, AT32_SDIO_ARG);

  /* Clear CMDINDEX, WAITRESP, WAITINT, WAITPEND, and CPSMEN bits */

  regval = getreg32(AT32_SDIO_CMD);
  regval &= ~(SDIO_CMD_CMDINDEX_MASK | SDIO_CMD_WAITRESP_MASK |
              SDIO_CMD_WAITINT | SDIO_CMD_WAITPEND | SDIO_CMD_CPSMEN);

  /* Set WAITRESP bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      regval |= SDIO_CMD_NORESPONSE;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= SDIO_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= SDIO_CMD_LONGRESPONSE;
      break;
    }

  /* Set CPSMEN and the command index */

  cmdidx  = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= cmdidx | SDIO_CMD_CPSMEN;

  mcinfo("cmd: %08" PRIx32 " arg: %08" PRIx32 " regval: %08" PRIx32
         " enabled irq: %08" PRIx32 "\n",
         cmd, arg, regval, getreg32(AT32_SDIO_MASK));

  /* Write the SDIO CMD */

  putreg32(SDIO_RESPDONE_ICR | SDIO_CMDDONE_ICR, AT32_SDIO_ICR);
  putreg32(regval, AT32_SDIO_CMD);
  return OK;
}

/****************************************************************************
 * Name: at32_blocksetup
 *
 * Description:
 *   Configure block size and the number of blocks for next transfer
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO device interface
 *   blocklen  - The selected block size.
 *   nblocklen - The number of blocks to transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_BLOCKSETUP
static void at32_blocksetup(struct sdio_dev_s *dev,
                             unsigned int blocklen, unsigned int nblocks)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;

  /* Configure block size for next transfer */

  priv->block_size = at32_log2(blocklen);
}
#endif

/****************************************************************************
 * Name: at32_recvsetup
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
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int at32_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                           size_t nbytes)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  at32_datadisable();
  at32_sampleinit();
  at32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_AT32_SDIO_DMA
  priv->dmamode   = false;
#endif

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != AT32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = at32_log2(nbytes) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }

  at32_dataconfig(SDIO_DTIMER_DATATIMEOUT_MS, nbytes,
                   dblocksize | SDIO_DCTRL_DTDIR);

  /* And enable interrupts */

  at32_configxfrints(priv, SDIO_RECV_MASK);
  at32_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: at32_sendsetup
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
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int at32_sendsetup(struct sdio_dev_s *dev,
                           const uint8_t *buffer, size_t nbytes)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Reset the DPSM configuration */

  at32_datadisable();
  at32_sampleinit();
  at32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
#ifdef CONFIG_AT32_SDIO_DMA
  priv->dmamode   = false;
#endif

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != AT32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = at32_log2(nbytes) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }

  at32_dataconfig(SDIO_DTIMER_DATATIMEOUT_MS, nbytes, dblocksize);

  /* Enable TX interrupts */

  at32_configxfrints(priv, SDIO_SEND_MASK);
  at32_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}

/****************************************************************************
 * Name: at32_cancel
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

static int at32_cancel(struct sdio_dev_s *dev)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;

  /* Disable all transfer- and event- related interrupts */

  at32_configxfrints(priv, 0);
  at32_configwaitints(priv, 0, 0, 0);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  putreg32(SDIO_WAITALL_ICR, AT32_SDIO_ICR);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_AT32_SDIO_DMA
  if (priv->dmamode)
    {
      /* Make sure that the DMA is stopped (it will be stopped automatically
       * on normal transfers, but not necessarily when the transfer
       * terminates on an error condition.
       */

      at32_dmastop(priv->dma);
    }
#endif

  /* Mark no transfer in progress */

  priv->remaining = 0;
  return OK;
}

/****************************************************************************
 * Name: at32_waitresponse
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

static int at32_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  int32_t timeout;
  uint32_t events;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      events  = SDIO_CMDDONE_STA;
      timeout = SDIO_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      events  = SDIO_RESPDONE_STA;
      timeout = SDIO_LONGTIMEOUT;
      break;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      events  = SDIO_RESPDONE_STA;
      timeout = SDIO_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the response (or timeout) */

  while ((getreg32(AT32_SDIO_STA) & events) == 0)
    {
      if (--timeout <= 0)
        {
          mcerr("ERROR: Timeout cmd: %08" PRIx32 " events: %08" PRIx32
                " STA: %08" PRIx32 "\n",
                cmd, events, getreg32(AT32_SDIO_STA));

          return -ETIMEDOUT;
        }
    }

  putreg32(SDIO_CMDDONE_ICR, AT32_SDIO_ICR);
  return OK;
}

/****************************************************************************
 * Name: at32_recv*
 *
 * Description:
 *   Receive response to SDIO command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   Rx - Buffer in which to receive the response
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure.  Here a
 *   failure means only a faiure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intacta and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int at32_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t *rshort)
{
#ifdef CONFIG_DEBUG_MEMCARD_INFO
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

#ifdef CONFIG_DEBUG_MEMCARD_INFO
  if (!rshort)
    {
      mcerr("ERROR: rshort=NULL\n");
      ret = -EINVAL;
    }

  /* Check that this is the correct response to this command */

  else if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R5_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(AT32_SDIO_STA);
      if ((regval & SDIO_STA_CTIMEOUT) != 0)
        {
          mcerr("ERROR: Command timeout: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & SDIO_STA_CCRCFAIL) != 0)
        {
          mcerr("ERROR: CRC failure: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
#ifdef CONFIG_DEBUG_MEMCARD_INFO
      else
        {
          /* Check response received is of desired command */

          respcmd = getreg32(AT32_SDIO_RESPCMD);
          if ((uint8_t)(respcmd & SDIO_RESPCMD_MASK) !=
              (cmd & MMCSD_CMDIDX_MASK))
            {
              mcerr("ERROR: RESCMD=%02" PRIx32 " CMD=%08" PRIx32 "\n",
                    respcmd, cmd);
              ret = -EINVAL;
            }
        }
#endif
    }

  /* Clear all pending message completion events and return the R1/R6
   * response.
   */

  putreg32(SDIO_RESPDONE_ICR | SDIO_CMDDONE_ICR, AT32_SDIO_ICR);
  *rshort = getreg32(AT32_SDIO_RESP1);
  return ret;
}

static int at32_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                          uint32_t rlong[4])
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

#ifdef CONFIG_DEBUG_MEMCARD_INFO
  /* Check that R1 is the correct response to this command */

  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R2_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = getreg32(AT32_SDIO_STA);
      if (regval & SDIO_STA_CTIMEOUT)
        {
          mcerr("ERROR: Timeout STA: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & SDIO_STA_CCRCFAIL)
        {
          mcerr("ERROR: CRC fail STA: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response */

  putreg32(SDIO_RESPDONE_ICR | SDIO_CMDDONE_ICR, AT32_SDIO_ICR);
  if (rlong)
    {
      rlong[0] = getreg32(AT32_SDIO_RESP1);
      rlong[1] = getreg32(AT32_SDIO_RESP2);
      rlong[2] = getreg32(AT32_SDIO_RESP3);
      rlong[3] = getreg32(AT32_SDIO_RESP4);
    }

  return ret;
}

static int at32_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t *rshort)
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

#ifdef CONFIG_DEBUG_MEMCARD_INFO
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R4_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout occurred (Apparently a CRC error can terminate
       * a good response)
       */

      regval = getreg32(AT32_SDIO_STA);
      if (regval & SDIO_STA_CTIMEOUT)
        {
          mcerr("ERROR: Timeout STA: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  putreg32(SDIO_RESPDONE_ICR | SDIO_CMDDONE_ICR, AT32_SDIO_ICR);
  if (rshort)
    {
      *rshort = getreg32(AT32_SDIO_RESP1);
    }

  return ret;
}

/****************************************************************************
 * Name: at32_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling at32_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
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

static void at32_waitenable(struct sdio_dev_s *dev,
                             sdio_eventset_t eventset, uint32_t timeout)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;
  uint32_t waitmask;

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  at32_configwaitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  if ((eventset & SDIOWAIT_WRCOMPLETE) != 0)
    {
      /* eventset carries this */

      waitmask = 0;
    }
  else
#endif
    {
      waitmask = 0;
      if ((eventset & SDIOWAIT_CMDDONE) != 0)
        {
          waitmask |= SDIO_CMDDONE_MASK;
        }

      if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
        {
          waitmask |= SDIO_RESPDONE_MASK;
        }

      if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
        {
          waitmask |= SDIO_XFRDONE_MASK;
        }

      /* Enable event-related interrupts */

      putreg32(SDIO_WAITALL_ICR, AT32_SDIO_ICR);
    }

  at32_configwaitints(priv, waitmask, eventset, 0);

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;
      int ret;

      /* Yes.. Handle a cornercase: The user request a timeout event but
       * with timeout == 0?
       */

      if (!timeout)
        {
          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return;
        }

      /* Start the watchdog timer */

      delay = MSEC2TICK(timeout);
      ret   = wd_start(&priv->waitwdog, delay,
                       at32_eventtimeout, (wdparm_t)priv);
      if (ret < 0)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: at32_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDIO_WAITEVENTS are disabled when at32_eventwait
 *   returns.  SDIO_WAITEVENTS must be called again before at32_eventwait
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

static sdio_eventset_t at32_eventwait(struct sdio_dev_s *dev)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  irqstate_t flags;
  int ret;

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero.
   */

  flags = enter_critical_section();

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  /* A card ejected while in SDIOWAIT_WRCOMPLETE can lead to a
   * condition where there is no waitevents set and no wkupevent
   */

  if (priv->waitevents == 0 && priv->wkupevent == 0)
    {
      wkupevent = SDIOWAIT_ERROR;
      goto errout_with_waitints;
    }

#else
  DEBUGASSERT(priv->waitevents != 0 || priv->wkupevent != 0);
#endif

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
  if ((priv->waitevents & SDIOWAIT_WRCOMPLETE) != 0)
    {
      /* Atomically read pin to see if ready (true) and determine if ISR
       * fired.  If Pin is ready and if ISR did NOT fire end the wait here.
       */

      if (at32_gpioread(GPIO_SDIO_D0) &&
         (priv->wkupevent & SDIOWAIT_WRCOMPLETE) == 0)
        {
          at32_endwait(priv, SDIOWAIT_WRCOMPLETE);
        }
    }
#endif

  /* Loop until the event (or the timeout occurs). Race conditions are
   * avoided by calling at32_waitenable prior to triggering the logic that
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
          /* Task canceled.  Cancel the wdog (assuming it was started) and
           * return an SDIO error.
           */

          wd_cancel(&priv->waitwdog);
          wkupevent = SDIOWAIT_ERROR;
          goto errout_with_waitints;
        }

      wkupevent = priv->wkupevent;

      /* Check if the event has occurred.  When the event has occurred, then
       * evenset will be set to 0 and wkupevent will be set to a nonzero
       * value.
       */

      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          break;
        }
    }

  /* Disable event-related interrupts */

errout_with_waitints:
  at32_configwaitints(priv, 0, 0, 0);
#ifdef CONFIG_AT32_SDIO_DMA
  priv->xfrflags   = 0;
#endif

  leave_critical_section(flags);
  at32_dumpsamples(priv);
  return wkupevent;
}

/****************************************************************************
 * Name: at32_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SDIO callback events.  This is part of the
 *   the SDIO callback sequence.  The set of events is configured to enabled
 *   callbacks to the function provided in at32_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this method.
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

static void at32_callbackenable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  at32_callback(priv);
}

/****************************************************************************
 * Name: at32_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDIO_CALLBACKENABLE
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

static int at32_registercallback(struct sdio_dev_s *dev,
                                  worker_t callback, void *arg)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;

  /* Disable callbacks and register this callback and is argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: at32_dmapreflight
 *
 * Description:
 *   Preflight an SDIO DMA operation.  If the buffer is not well-formed for
 *   SDIO DMA transfer (alignment, size, etc.) returns an error.
 *
 * Input Parameters:
 *   dev    - An instance of the SDIO device interface
 *   buffer - The memory to DMA to/from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 ****************************************************************************/

#if defined(CONFIG_AT32_SDIO_DMA) && defined(CONFIG_ARCH_HAVE_SDIO_PREFLIGHT)
static int at32_dmapreflight(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t buflen)
{
#if !defined(CONFIG_AT32_AT32F4XXX)
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* Wide bus operation is required for DMA */

  if (!priv->widebus)
    {
      return -EINVAL;
    }
#endif

  /* DMA must be possible to the buffer */

  if (!at32_dmacapable((uintptr_t)buffer, (buflen + 3) >> 2,
                        SDIO_RXDMA32_CONFIG))
    {
      return -EFAULT;
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: at32_dmarecvsetup
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

#ifdef CONFIG_AT32_SDIO_DMA
static int at32_dmarecvsetup(struct sdio_dev_s *dev,
                              uint8_t *buffer, size_t buflen)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
  DEBUGASSERT(at32_dmapreflight(dev, buffer, buflen) == 0);
#endif

  /* Reset the DPSM configuration */

  at32_datadisable();

  /* Initialize register sampling */

  at32_sampleinit();
  at32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->dmamode   = true;

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != AT32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = at32_log2(buflen) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }

  at32_dataconfig(SDIO_DTIMER_DATATIMEOUT_MS, buflen,
                   dblocksize | SDIO_DCTRL_DTDIR);

  /* Configure the RX DMA */

  at32_configxfrints(priv, SDIO_DMARECV_MASK);

  putreg32(1, SDIO_DCTRL_DMAEN_BB);
  at32_dmasetup(priv->dma, AT32_SDIO_FIFO, (uint32_t)buffer,
                 (buflen + 3) >> 2, SDIO_RXDMA32_CONFIG);

  /* Start the DMA */

  at32_sample(priv, SAMPLENDX_BEFORE_ENABLE);
  at32_dmastart(priv->dma, at32_dmacallback, priv, false);
  at32_sample(priv, SAMPLENDX_AFTER_SETUP);

  return OK;
}
#endif

/****************************************************************************
 * Name: at32_dmasendsetup
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

#ifdef CONFIG_AT32_SDIO_DMA
static int at32_dmasendsetup(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t buflen)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;
  uint32_t dblocksize;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
  DEBUGASSERT(at32_dmapreflight(dev, buffer, buflen) == 0);
#endif

  /* Reset the DPSM configuration */

  at32_datadisable();

  /* Initialize register sampling */

  at32_sampleinit();
  at32_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->dmamode   = true;

  /* Then set up the SDIO data path */

#ifdef CONFIG_SDIO_BLOCKSETUP
  if (priv->block_size != AT32_SDIO_USE_DEFAULT_BLOCKSIZE)
    {
      dblocksize = priv->block_size << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }
  else
#endif
    {
      dblocksize = at32_log2(buflen) << SDIO_DCTRL_DBLOCKSIZE_SHIFT;
    }

  at32_dataconfig(SDIO_DTIMER_DATATIMEOUT_MS, buflen, dblocksize);

  /* Configure the TX DMA */

  at32_dmasetup(priv->dma, AT32_SDIO_FIFO, (uint32_t)buffer,
                 (buflen + 3) >> 2, SDIO_TXDMA32_CONFIG);

  at32_sample(priv, SAMPLENDX_BEFORE_ENABLE);
  putreg32(1, SDIO_DCTRL_DMAEN_BB);

  /* Start the DMA */

  at32_dmastart(priv->dma, at32_dmacallback, priv, false);
  at32_sample(priv, SAMPLENDX_AFTER_SETUP);

  /* Enable TX interrupts */

  at32_configxfrints(priv, SDIO_DMASEND_MASK);

  return OK;
}
#endif

/****************************************************************************
 * Name: at32_callback
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

static void at32_callback(void *arg)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)arg;

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  mcinfo("Callback %p(%p) cbevents: %02x cdstatus: %02x\n",
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

      /* Callbacks cannot be performed in the context of an interrupt
       * handler.  If we are in an interrupt handler, then queue the
       * callback to be performed later on the work thread.
       */

      if (up_interrupt_context())
        {
          /* Yes.. queue it */

           mcinfo("Queuing callback to %p(%p)\n",
                  priv->callback, priv->cbarg);
           work_queue(HPWORK, &priv->cbwork, priv->callback,
                      priv->cbarg, 0);
        }
      else
        {
          /* No.. then just call the callback here */

          mcinfo("Callback to %p(%p)\n", priv->callback, priv->cbarg);
          priv->callback(priv->cbarg);
        }
    }
}

/****************************************************************************
 * Name: at32_default
 *
 * Description:
 *   Restore SDIO registers to their default, reset values
 *
 ****************************************************************************/

static void at32_default(void)
{
  putreg32(SDIO_POWER_RESET,  AT32_SDIO_POWER);
  putreg32(SDIO_CLKCR_RESET,  AT32_SDIO_CLKCR);
  putreg32(SDIO_ARG_RESET,    AT32_SDIO_ARG);
  putreg32(SDIO_CMD_RESET,    AT32_SDIO_CMD);
  putreg32(SDIO_DTIMER_RESET, AT32_SDIO_DTIMER);
  putreg32(SDIO_DLEN_RESET,   AT32_SDIO_DLEN);
  putreg32(SDIO_DCTRL_RESET,  AT32_SDIO_DCTRL);
  putreg32(SDIO_ICR_RESET,    AT32_SDIO_ICR);
  putreg32(SDIO_MASK_RESET,   AT32_SDIO_MASK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
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
  /* There is only one slot */

  struct at32_dev_s *priv = &g_sdiodev;

  /* Allocate a DMA channel */

#ifdef CONFIG_AT32_SDIO_DMA
  priv->dma = at32_dmachannel(SDIO_DMACHAN);
  DEBUGASSERT(priv->dma);
#endif

  /* Configure GPIOs for 4-bit, wide-bus operation (the chip is capable of
   * 8-bit wide bus operation but D4-D7 are not configured).
   *
   * If bus is multiplexed then there is a custom bus configuration utility
   * in the scope of the board support package.
   */

#ifndef CONFIG_SDIO_MUXBUS
  at32_configgpio(GPIO_SDIO_D0 | SDIO_PULLUP_ENABLE);
#ifndef CONFIG_AT32_SDIO_WIDTH_D1_ONLY
  at32_configgpio(GPIO_SDIO_D1 | SDIO_PULLUP_ENABLE);
  at32_configgpio(GPIO_SDIO_D2 | SDIO_PULLUP_ENABLE);
  at32_configgpio(GPIO_SDIO_D3 | SDIO_PULLUP_ENABLE);
#endif
  at32_configgpio(GPIO_SDIO_CK | SDIO_PULLUP_ENABLE);
  at32_configgpio(GPIO_SDIO_CMD | SDIO_PULLUP_ENABLE);
#endif

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  at32_reset(&priv->dev);
  return &g_sdiodev.dev;
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
 ****************************************************************************/

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;
  sdio_statset_t cdstatus;
  irqstate_t flags;

  /* Update card status */

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

  leave_critical_section(flags);

  mcinfo("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

  /* Perform any requested callback if the status has changed */

  if (cdstatus != priv->cdstatus)
    {
      at32_callback(priv);
    }
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
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sdio_wrprotect(struct sdio_dev_s *dev, bool wrprotect)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;
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

/****************************************************************************
 * Name: sdio_set_sdio_card_isr
 *
 * Description:
 *   SDIO card generates interrupt via SDIO_DATA_1 pin.
 *   Called by board-specific logic to register an ISR for SDIO card.
 *
 * Input Parameters:
 *   func      - callback function.
 *   arg       - arg to be passed to the function.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_AT32_SDIO_CARD
void sdio_set_sdio_card_isr(struct sdio_dev_s *dev,
                            int (*func)(void *), void *arg)
{
  struct at32_dev_s *priv = (struct at32_dev_s *)dev;

  priv->do_sdio_card = func;

  if (func != NULL)
    {
      priv->sdiointmask = SDIO_STA_SDIOIT;
      priv->do_sdio_arg = arg;
    }
  else
    {
      priv->sdiointmask = 0;
    }

  putreg32(priv->xfrmask | priv->waitmask | priv->sdiointmask,
           AT32_SDIO_MASK);
}
#endif

#endif /* CONFIG_AT32_SDIO */
