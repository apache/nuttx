/****************************************************************************
 * arch/arm/src/sama5/sam_sdmmc.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <strings.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>
#include <nuttx/kmalloc.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "sam_config.h"
#include "sam_pio.h"
#include "sam_periphclks.h"
#include "sam_sdmmc.h"
#include "hardware/sam_sdmmc.h"
#include "hardware/sam_pinmap.h"
#include "hardware/sam_pio.h"

#ifdef CONFIG_SAMA5_SDMMC

/****************************************************************************
 *  Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if ((defined(CONFIG_SAMA5_SDMMC0) && !defined(CONFIG_SAMA5_SDMMC1)) || \
     (defined(CONFIG_SAMA5_SDMMC1) && !defined(CONFIG_SAMA5_SDMMC0)))
#  define  SAM_MAX_SDMMC_DEV_SLOTS  1
#elif (defined(CONFIG_SAMA5_SDMMC0) && defined(CONFIG_SAMA5_SDMMC1))
#  define  SAM_MAX_SDMMC_DEV_SLOTS  2
#else
#error Unrecognised number of SDMMC slots
#endif

#if !defined(CONFIG_SAMA5_SDMMC_DMA)
#  warning "Large Non-DMA transfer may result in RX overrun failures"
#elif !defined(CONFIG_SDIO_DMA)
#  warning CONFIG_SDIO_DMA should be defined with CONFIG_SAMA5_SDMMC_DMA
#endif

#if !defined(CONFIG_SCHED_WORKQUEUE) || !defined(CONFIG_SCHED_HPWORK)
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE and CONFIG_SCHED_HPWORK"
#endif

#if !defined(CONFIG_SDIO_BLOCKSETUP)
#  error "CONFIG_SDIO_BLOCKSETUP is mandatory for this driver"
#endif

#ifndef CONFIG_DEBUG_MEMCARD_INFO
#  undef CONFIG_SDIO_XFRDEBUG
#endif

/* Timing in ms for commands wait response */

#define SDMMC_CMDTIMEOUT            MSEC2TICK(100)
#define SDMMC_LONGTIMEOUT           MSEC2TICK(500)

/* Big DTOCV setting.  Range is 0 = SDCLK * 2^13 through 15 = SDCLK * 2^29 */

#define SDMMC_DTOCV_MAXTIMEOUT      (15)

/* Data transfer / Event waiting interrupt mask bits */

#define SDMMC_RESPERR_INTS          (SDMMC_INT_CCE | SDMMC_INT_CTOE | \
                                     SDMMC_INT_CEBE | SDMMC_INT_CIE)
#define SDMMC_RESPDONE_INTS         (SDMMC_RESPERR_INTS | SDMMC_INT_CC)

#define SDMMC_XFRERR_INTS           (SDMMC_INT_DCE | SDMMC_INT_DTOE | \
                                     SDMMC_INT_DEBE)
#define SDMMC_RCVDONE_INTS          (SDMMC_XFRERR_INTS | SDMMC_INT_BRR | \
                                     SDMMC_INT_TC)
#define SDMMC_SNDDONE_INTS          (SDMMC_XFRERR_INTS | SDMMC_INT_BWR | \
                                     SDMMC_INT_TC)
#define SDMMC_XFRDONE_INTS          (SDMMC_XFRERR_INTS | SDMMC_INT_BRR | \
                                     SDMMC_INT_BWR | SDMMC_INT_TC)

/* CD Detect Types */

#define SDMMC_DMAERR_INTS           (SDMMC_XFRERR_INTS)
#define SDMMC_DMADONE_INTS          (SDMMC_DMAERR_INTS | SDMMC_INT_TC)

#define SDMMC_WAITALL_INTS          (SDMMC_RESPDONE_INTS | \
                                     SDMMC_XFRDONE_INTS |  \
                                     SDMMC_DMADONE_INTS)

#define  SDMMC_INT_CMD_MASK     (SDMMC_INT_CC | SDMMC_INT_CTOE | \
                                 SDMMC_INT_CCE | SDMMC_INT_CEBE | \
                                 SDMMC_INT_CIE)

#define  SDMMC_INT_DATA_MASK    (SDMMC_INT_TC | SDMMC_INT_DINT | \
                                 SDMMC_INT_BRR | SDMMC_INT_BWR | \
                                 SDMMC_INT_CTOE | SDMMC_INT_CCE | \
                                 SDMMC_INT_DEBE | SDMMC_INT_ADMAE)

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
#  define DBG_BASE_ADDR             SAM_SDMMC1_VBASE
#  define SAMPLENDX_BEFORE_SETUP    0
#  define SAMPLENDX_AFTER_SETUP     1
#  define SAMPLENDX_END_TRANSFER    2
#  define DEBUG_NSAMPLES            3
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure defines the state of the SAMA5 SDIO interface */

struct sam_dev_s
{
  struct sdio_dev_s dev;         /* Standard, base SDIO interface */

  /* SAMA5-specific extensions */

  /* Event support */

  uint32_t base;                       /* SDMMC register base address */
  sem_t waitsem;                       /* Implements event waiting */
  sdio_eventset_t waitevents;          /* Set of events to be waited for */
  uint32_t waitints;                   /* Interrupt enables for event waiting */
  volatile sdio_eventset_t wkupevent;  /* The event that caused the wakeup */
  struct wdog_s waitwdog;              /* Watchdog that handles event timeouts */

  /* Callback support */

  sdio_statset_t cdstatus;       /* Card status */
  sdio_eventset_t cbevents;      /* Set of events to be cause callbacks */
  worker_t callback;             /* Registered callback function */
  void *cbarg;                   /* Registered callback argument */
  struct work_s cbwork;          /* Callback work queue structure */

  /* Interrupt mode data transfer support */

  uint32_t *buffer;              /* Address of current R/W buffer */
  size_t remaining;              /* Number of bytes remaining in the transfer */
  uint32_t xfrints;              /* Interrupt enables for data transfer */

#ifdef CONFIG_SAMA5_SDMMC_DMA
  /* DMA data transfer support */

  volatile uint8_t xfrflags;     /* Used to synchronize SDIO and DMA completion */
  uint32_t *bufferend;           /* Far end of R/W buffer for cache invalidation */
#endif

  /* Card interrupt support for SDIO */

  uint32_t cintints;             /* Interrupt enables for card ints */
  int (*do_sdio_card)(void *);   /* SDIO card ISR */
  void *do_sdio_arg;             /* arg for SDIO card ISR */

  uint32_t addr;                 /* Base address of this instances */
  uint32_t sw_cd_gpio;           /* If a non SDMMCx CD pin is used, this is its GPIO */
  uint32_t cd_invert;            /* If true invert the CD pin */

#ifdef CONFIG_SAMA5_SDMMC_REGDEBUG
  bool               wrlast;     /* Last was a write */
  uint32_t           addrlast;   /* Last address */
  uint32_t           vallast;    /* Last value */
  int                ntimes;     /* Number of times */
#endif
};

/* Register logging support */

#ifdef CONFIG_SDIO_XFRDEBUG
struct sam_sdmmcregs_s
{
  /* All read-able SDMMC registers */

  uint32_t dsaddr;               /* DMA System Address Register */
  uint32_t blkattr;              /* Block Attributes Register */
  uint32_t cmdarg;               /* Command Argument Register */
  uint32_t xferty;               /* Transfer Type Register */
  uint32_t cmdrsp0;              /* Command Response 0 */
  uint32_t cmdrsp1;              /* Command Response 1 */
  uint32_t cmdrsp2;              /* Command Response 2 */
  uint32_t cmdrsp3;              /* Command Response 3 */
  uint32_t dbap;                 /* Data buffer access port */
  uint32_t prsstat;              /* Present State Register */
  uint32_t proctl;               /* Protocol Control Register */
  uint32_t sysctl;               /* System Control Register */
  uint32_t irqstat;              /* Interrupt Status Register */
  uint32_t irqstaten;            /* Interrupt Status Enable Register */
  uint32_t irqsigen;             /* Interrupt Signal Enable Register */
  uint32_t ac12err;              /* Auto CMD12 Error Status Register */
  uint32_t htcapblt0;            /* Host Controller Capabilities 2 Register */
  uint32_t htcapblt1;            /* Host Controller Capabilities 1 Register */
  uint32_t mixctrl;              /* Mixer Control */
  uint32_t fevent;               /* Force Event */
  uint32_t admaes;               /* ADMA Error Status Register */
  uint32_t adsaddr;              /* ADMA System Address Register */
  uint32_t dllctrl;              /* Delay line control */
  uint32_t dllstat;              /* Delay line status */
  uint32_t clktune;              /* Clock tune and control */
  uint32_t tuningctrl;           /* Tuning Control */
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers ********************************************************/

static void sam_configwaitints(struct sam_dev_s *priv, uint32_t waitints,
              sdio_eventset_t waitevents, sdio_eventset_t wkupevents);
static void sam_configxfrints(struct sam_dev_s *priv, uint32_t xfrints);

/* DMA Helpers **************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void sam_sampleinit(void);
static void sam_sdmmcsample(struct sam_dev_s *priv,
                            struct sam_sdmmcregs_s *regs);
static void sam_sample(struct sam_dev_s *priv, int index);
static void sam_dumpsample(struct sam_dev_s *priv,
              struct sam_sdmmcregs_s *regs, const char *msg);
static void sam_dumpsamples(struct sam_dev_s *priv);
static void sam_showregs(struct sam_dev_s *priv, const char *msg);

#else
#  define   sam_sampleinit()
#  define   sam_sample(priv, index)
#  define   sam_dumpsamples(priv)
#  define   sam_showregs(priv, msg)
#endif

/* Data Transfer Helpers ****************************************************/

static void sam_dataconfig(struct sam_dev_s *priv, bool bwrite,
              unsigned int datalen, unsigned int timeout);

#ifndef CONFIG_SAMA5_SDMMC_DMA
static void sam_transmit(struct sam_dev_s *priv);
static void sam_receive(struct sam_dev_s *priv);
#endif

static void sam_eventtimeout(wdparm_t arg);
static void sam_endwait(struct sam_dev_s *priv,
              sdio_eventset_t wkupevent);
static void sam_endtransfer(struct sam_dev_s *priv,
              sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  sam_interrupt(int irq, void *context, void *arg);

/* SDIO interface methods ***************************************************/

/* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
static int sam_lock(struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void sam_reset(struct sdio_dev_s *dev);
static sdio_capset_t sam_capabilities(struct sdio_dev_s *dev);
static sdio_statset_t sam_status(struct sdio_dev_s *dev);
static void sam_widebus(struct sdio_dev_s *dev, bool enable);

#ifdef CONFIG_SAM_SDMMC_ABSFREQ
static void sam_frequency(struct sdio_dev_s *dev, uint32_t frequency);
#endif

static void sam_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate);
static void sam_power(struct sam_dev_s *priv);
static int  sam_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  sam_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);

#ifdef CONFIG_SDIO_BLOCKSETUP
static void sam_blocksetup(struct sdio_dev_s *dev,
              unsigned int blocklen, unsigned int nblocks);
#endif

#ifndef CONFIG_SAMA5_SDMMC_DMA
static int  sam_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
              size_t nbytes);
static int  sam_sendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t nbytes);
#endif

static int  sam_cancel(struct sdio_dev_s *dev);
static int  sam_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  sam_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  sam_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  sam_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);

/* EVENT handler */

static void sam_waitenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t sam_eventwait(struct sdio_dev_s *dev);
static void sam_callbackenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static int  sam_registercallback(struct sdio_dev_s *dev,
              worker_t callback, void *arg);

/* DMA */

#ifdef CONFIG_SAMA5_SDMMC_DMA
static int  sam_dmarecvsetup(struct sdio_dev_s *dev,
              uint8_t *buffer, size_t buflen);
static int  sam_dmasendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t buflen);
#endif

/* Initialization/uninitialization/reset ************************************/

static void sam_callback(void *arg);
void sam_set_uhs_timing(struct sam_dev_s *priv,
                        enum bus_mode selected_mode);
static int sam_set_clock(struct sam_dev_s *priv, uint32_t clock);
static void sam_power(struct sam_dev_s *priv);
static int sam_set_interrupts(struct sam_dev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct sam_dev_s g_sdmmcdev[SAM_MAX_SDMMC_DEV_SLOTS] =
{
#ifdef CONFIG_SAMA5_SDMMC0
  {
    .addr               = SAM_SDMMC0_VBASE,
#if defined(PIN_SDMMC0_CD_GPIO)
    .sw_cd_gpio         = PIN_SDMMC0_CD_GPIO,
#endif
#if defined(CONFIG_SAMA5_SDMMC0_INVERT_CD)
    .cd_invert          = true,
#endif
    .dev                =
    {
#ifdef CONFIG_SDIO_MUXBUS
      .lock             = sam_lock,
#endif
      .reset            = sam_reset,
      .capabilities     = sam_capabilities,
      .status           = sam_status,
      .widebus          = sam_widebus,
      .clock            = sam_clock,
      .attach           = sam_attach,
      .sendcmd          = sam_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
      .blocksetup       = sam_blocksetup,
#endif

#ifndef CONFIG_SAMA5_SDMMC_DMA
      .recvsetup        = sam_recvsetup,
      .sendsetup        = sam_sendsetup,
#else
      .recvsetup        = sam_dmarecvsetup,
      .sendsetup        = sam_dmasendsetup,
#endif
      .cancel           = sam_cancel,
      .waitresponse     = sam_waitresponse,
      .recv_r1          = sam_recvshortcrc,
      .recv_r2          = sam_recvlong,
      .recv_r3          = sam_recvshort,
      .recv_r4          = sam_recvshort,
      .recv_r5          = sam_recvshortcrc,
      .recv_r6          = sam_recvshortcrc,
      .recv_r7          = sam_recvshort,
      .waitenable       = sam_waitenable,
      .eventwait        = sam_eventwait,
      .callbackenable   = sam_callbackenable,
      .registercallback = sam_registercallback,
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_SAMA5_SDMMC_DMA
      .dmarecvsetup     = sam_dmarecvsetup,
      .dmasendsetup     = sam_dmasendsetup,
#else
      .dmarecvsetup     = sam_recvsetup,
      .dmasendsetup     = sam_sendsetup,
#endif
#endif
    },
    .waitsem            = SEM_INITIALIZER(0),
  },
#endif

#ifdef CONFIG_SAMA5_SDMMC1
  {
    .addr               = SAM_SDMMC1_VBASE,
#if defined(PIN_SDMMC1_CD_GPIO)
    .sw_cd_gpio         = PIN_SDMMC1_CD_GPIO,
#endif
#if defined(CONFIG_SAMA5_SDMMC1_INVERT_CD)
    .cd_invert          = true,
#endif
    .dev                =
    {
#ifdef CONFIG_SDIO_MUXBUS
      .lock             = sam_lock,
#endif
      .reset            = sam_reset,
      .capabilities     = sam_capabilities,
      .status           = sam_status,
      .widebus          = sam_widebus,
      .clock            = sam_clock,
      .attach           = sam_attach,
      .sendcmd          = sam_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
      .blocksetup       = sam_blocksetup,
#endif

#ifndef CONFIG_SAMA5_SDMMC_DMA
      .recvsetup        = sam_recvsetup,
      .sendsetup        = sam_sendsetup,
#else
      .recvsetup        = sam_dmarecvsetup,
      .sendsetup        = sam_dmasendsetup,
#endif
      .cancel           = sam_cancel,
      .waitresponse     = sam_waitresponse,
      .recv_r1          = sam_recvshortcrc,
      .recv_r2          = sam_recvlong,
      .recv_r3          = sam_recvshort,
      .recv_r4          = sam_recvshort,
      .recv_r5          = sam_recvshortcrc,
      .recv_r6          = sam_recvshortcrc,
      .recv_r7          = sam_recvshort,
      .waitenable       = sam_waitenable,
      .eventwait        = sam_eventwait,
      .callbackenable   = sam_callbackenable,
      .registercallback = sam_registercallback,
#ifdef CONFIG_SDIO_DMA
#ifdef CONFIG_SAMA5_SDMMC_DMA
      .dmarecvsetup     = sam_dmarecvsetup,
      .dmasendsetup     = sam_dmasendsetup,
#else
      .dmarecvsetup     = sam_recvsetup,
      .dmasendsetup     = sam_sendsetup,
#endif
    },
    .waitsem            = SEM_INITIALIZER(0),
  }
#endif
#endif
};

#ifdef CONFIG_SDIO_XFRDEBUG
/* Register logging support */

static struct sam_sdmmcregs_s g_sampleregs[DEBUG_NSAMPLES];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Low-level Helpers
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

#ifdef CONFIG_SAMA5_SDMMC_REGDEBUG
static bool sam_checkreg(struct sam_dev_s *priv, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == priv->wrlast &&     /* Same kind of access? */
      value   == priv->vallast &&    /* Same value? */
      address == priv->addrlast)     /* Same address? */
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
 * Name: sam_getreg8
 *
 * Description:
 *  Read an 8-bit SDMMC register
 *
 ****************************************************************************/

static inline uint32_t sam_getreg8(struct sam_dev_s *priv,
                                   unsigned int offset)
{
  uint32_t address = priv->base + offset;
  uint32_t value = getreg8(address);

#ifdef CONFIG_SAMA5_SDMMC_REGDEBUG
  if (sam_checkreg(priv, false, value, address))
    {
      mcinfo("%08x->%08x\n", address, value);
    }
#endif

  return value;
}

/****************************************************************************
 * Name: sam_getreg16
 *
 * Description:
 *  Read a 16-bit SDMMC register
 *
 ****************************************************************************/

static inline uint32_t sam_getreg16(struct sam_dev_s *priv,
                                    unsigned int offset)
{
  uint32_t address = priv->base + offset;
  uint32_t value = getreg16(address);

#ifdef CONFIG_SAMA5_SDMMC_REGDEBUG
  if (sam_checkreg(priv, false, value, address))
    {
      mcinfo("%08x->%08x\n", address, value);
    }
#endif

  return value;
}

/****************************************************************************
 * Name: sam_getreg32
 *
 * Description:
 *  Read a 32-bit SDMMC register
 *
 ****************************************************************************/

static inline uint32_t sam_getreg32(struct sam_dev_s *priv,
                                    unsigned int offset)
{
  uint32_t address = priv->base + offset;
  uint32_t value = getreg32(address);

#ifdef CONFIG_SAMA5_SDMMC_REGDEBUG
  if (sam_checkreg(priv, false, value, address))
    {
      mcinfo("%08x->%08x\n", address, value);
    }
#endif

  return value;
}

/****************************************************************************
 * Name: sam_getreg
 *
 * Description:
 *  Read a 32-bit SDMMC register
 *
 ****************************************************************************/

static inline uint32_t sam_getreg(struct sam_dev_s *priv,
                                  unsigned int offset)
{
  return sam_getreg32(priv, offset);
}

/****************************************************************************
 * Name: sam_putreg8
 *
 * Description:
 *  Write an 8-bit value to an SDMMC register
 *
 ****************************************************************************/

static inline void sam_putreg8(struct sam_dev_s *priv, uint32_t value,
                              unsigned int offset)
{
  uint32_t address = priv->base + offset;

#ifdef CONFIG_SAMA5_SDMMC_REGDEBUG
  if (sam_checkreg(priv, true, value, address))
    {
      mcinfo("%08x<-%08x\n", address, value);
    }
#endif

  putreg8(value, address);
}

/****************************************************************************
 * Name: sam_putreg16
 *
 * Description:
 *  Write a 16-bit value to an SDMMC register
 *
 ****************************************************************************/

static inline void sam_putreg16(struct sam_dev_s *priv, uint32_t value,
                                unsigned int offset)
{
  uint32_t address = priv->base + offset;

#ifdef CONFIG_SAMA5_SDMMC_REGDEBUG
  if (sam_checkreg(priv, true, value, address))
    {
      mcinfo("%08x<-%08x\n", address, value);
    }
#endif

  putreg16(value, address);
}

/****************************************************************************
 * Name: sam_putreg32
 *
 * Description:
 *  Write a value to an SDMMC register
 *
 ****************************************************************************/

static inline void sam_putreg32(struct sam_dev_s *priv, uint32_t value,
                                unsigned int offset)
{
  uint32_t address = priv->base + offset;

#ifdef CONFIG_SAMA5_SDMMC_REGDEBUG
  if (sam_checkreg(priv, true, value, address))
    {
      mcinfo("%08x<-%08x\n", address, value);
    }
#endif

  putreg32(value, address);
}

/****************************************************************************
 * Name: sam_putreg
 *
 * Description:
 *  Write a 32-bit value to an SDMMC register
 *
 ****************************************************************************/

static inline void sam_putreg(struct sam_dev_s *priv, uint32_t value,
                              unsigned int offset)
{
  return sam_putreg32(priv, value, offset);
}

/****************************************************************************
 * Name: sam_configwaitints
 *
 * Description:
 *   Enable/disable SDIO interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SDIO device state structure
 *   waitints   - The set of bits in the SDIO MASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_configwaitints(struct sam_dev_s *priv, uint32_t waitints,
                                 sdio_eventset_t waitevents,
                                 sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags            = enter_critical_section();
  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitints   = waitints;

#ifdef CONFIG_SAMA5_SDMMC_DMA
  priv->xfrflags   = 0;
#endif
  sam_putreg(priv, priv->xfrints | priv->waitints | priv->cintints | \
                   SDMMC_INT_DINT,
           SAMA5_SDMMC_IRQSIGEN_OFFSET);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_configxfrints
 *
 * Description:
 *   Enable SDIO interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SDIO device state structure
 *   xfrints - The set of bits in the SDIO MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_configxfrints(struct sam_dev_s *priv, uint32_t xfrints)
{
  irqstate_t flags;

  flags = enter_critical_section();
  priv->xfrints = xfrints;
  sam_putreg(priv, priv->xfrints | priv->waitints | priv->cintints,
           SAMA5_SDMMC_IRQSIGEN_OFFSET);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_sampleinit
 *
 * Description:
 *   Setup prior to collecting DMA samples
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void sam_sampleinit(void)
{
  memset(g_sampleregs, 0xff,
         DEBUG_NSAMPLES * sizeof(struct sam_sdmmcregs_s));
}
#endif

/****************************************************************************
 * Name: sam_sdmmcsample
 *
 * Description:
 *   Sample SDIO registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void sam_sdmmcsample(struct sam_dev_s *priv,
                            struct sam_sdmmcregs_s *regs)
{
  regs->dsaddr    = sam_getreg32(priv, SAMA5_SDMMC_DSADDR_OFFSET);
  regs->blkattr   = sam_getreg32(priv, SAMA5_SDMMC_BLKATTR_OFFSET);
  regs->cmdarg    = sam_getreg32(priv, SAMA5_SDMMC_CMDARG_OFFSET);
  regs->xferty    = sam_getreg32(priv, SAMA5_SDMMC_XFERTYP_OFFSET);
  regs->cmdrsp0   = sam_getreg32(priv, SAMA5_SDMMC_CMDRSP0_OFFSET);
  regs->cmdrsp1   = sam_getreg32(priv, SAMA5_SDMMC_CMDRSP1_OFFSET);
  regs->cmdrsp2   = sam_getreg32(priv, SAMA5_SDMMC_CMDRSP2_OFFSET);
  regs->cmdrsp3   = sam_getreg32(priv, SAMA5_SDMMC_CMDRSP3_OFFSET);
  regs->prsstat   = sam_getreg32(priv, SAMA5_SDMMC_PRSSTAT_OFFSET);
  regs->proctl    = sam_getreg32(priv, SAMA5_SDMMC_PROCTL_OFFSET);
  regs->sysctl    = sam_getreg32(priv, SAMA5_SDMMC_SYSCTL_OFFSET);
  regs->irqstat   = sam_getreg32(priv, SAMA5_SDMMC_IRQSTAT_OFFSET);
  regs->irqstaten = sam_getreg32(priv, SAMA5_SDMMC_IRQSTATEN_OFFSET);
  regs->irqsigen  = sam_getreg32(priv, SAMA5_SDMMC_IRQSIGEN_OFFSET);
  regs->ac12err   = sam_getreg32(priv, SAMA5_SDMMC_AC12ERR_OFFSET);
  regs->htcapblt0 = sam_getreg32(priv, SAMA5_SDMMC_HTCAPBLT0_OFFSET);
  regs->htcapblt1 = sam_getreg32(priv, SAMA5_SDMMC_HTCAPBLT1_OFFSET);
  regs->admaes    = sam_getreg32(priv, SAMA5_SDMMC_ADMAES_OFFSET);
  regs->adsaddr   = sam_getreg32(priv, SAMA5_SDMMC_ADSADDR_OFFSET);
}
#endif

/****************************************************************************
 * Name: sam_sample
 *
 * Description:
 *   Sample SDIO/DMA registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void sam_sample(struct sam_dev_s *priv, int index)
{
  if (priv->addr == DBG_BASE_ADDR)
    {
      sam_sdmmcsample(priv, &g_sampleregs[index]);
    }
}
#endif

/****************************************************************************
 * Name: sam_dumpsample
 *
 * Description:
 *   Dump one register sample
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void sam_dumpsample(struct sam_dev_s *priv,
                             struct sam_sdmmcregs_s *regs, const char *msg)
{
  mcinfo("SDMMC Registers: %s\n", msg);
  mcinfo("   DSADDR[%08x]: %08x\n",
         SAMA5_SDMMC_DSADDR_OFFSET, regs->dsaddr);
  mcinfo("  BLKATTR[%08x]: %08x\n",
         SAMA5_SDMMC_BLKATTR_OFFSET, regs->blkattr);
  mcinfo("   CMDARG[%08x]: %08x\n",
         SAMA5_SDMMC_CMDARG_OFFSET, regs->cmdarg);
  mcinfo("   XFERTY[%08x]: %08x\n",
         SAMA5_SDMMC_XFERTYP_OFFSET, regs->xferty);
  mcinfo("  CMDRSP0[%08x]: %08x\n",
         SAMA5_SDMMC_CMDRSP0_OFFSET, regs->cmdrsp0);
  mcinfo("  CMDRSP1[%08x]: %08x\n",
         SAMA5_SDMMC_CMDRSP1_OFFSET, regs->cmdrsp1);
  mcinfo("  CMDRSP2[%08x]: %08x\n",
         SAMA5_SDMMC_CMDRSP2_OFFSET, regs->cmdrsp2);
  mcinfo("  CMDRSP3[%08x]: %08x\n",
         SAMA5_SDMMC_CMDRSP3_OFFSET, regs->cmdrsp3);
  mcinfo("  PRSSTAT[%08x]: %08x\n",
         SAMA5_SDMMC_PRSSTAT_OFFSET, regs->prsstat);
  mcinfo("   PROCTL[%08x]: %08x\n",
         SAMA5_SDMMC_PROCTL_OFFSET, regs->proctl);
  mcinfo("   SYSCTL[%08x]: %08x\n",
         SAMA5_SDMMC_SYSCTL_OFFSET, regs->sysctl);
  mcinfo("  IRQSTAT[%08x]: %08x\n",
         SAMA5_SDMMC_IRQSTAT_OFFSET, regs->irqstat);
  mcinfo("IRQSTATEN[%08x]: %08x\n",
         SAMA5_SDMMC_IRQSTATEN_OFFSET, regs->irqstaten);
  mcinfo(" IRQSIGEN[%08x]: %08x\n",
         SAMA5_SDMMC_IRQSIGEN_OFFSET, regs->irqsigen);
  mcinfo("  AC12ERR[%08x]: %08x\n",
         SAMA5_SDMMC_AC12ERR_OFFSET, regs->ac12err);
  mcinfo("HTCAPBLT0[%08x]: %08x\n",
         SAMA5_SDMMC_HTCAPBLT0_OFFSET, regs->htcapblt0);
  mcinfo("HTCAPBLT1[%08x]: %08x\n",
         SAMA5_SDMMC_HTCAPBLT1_OFFSET, regs->htcapblt1);
  mcinfo("   ADMAES[%08x]: %08x\n",
         SAMA5_SDMMC_ADMAES_OFFSET, regs->admaes);
  mcinfo("  ADSADDR[%08x]: %08x\n",
         SAMA5_SDMMC_ADSADDR_OFFSET, regs->adsaddr);
}
#endif

/****************************************************************************
 * Name: sam_dumpsamples
 *
 * Description:
 *   Dump all sampled register data
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void sam_dumpsamples(struct sam_dev_s *priv)
{
  sam_dumpsample(priv, &g_sampleregs[SAMPLENDX_BEFORE_SETUP],
                   "Before setup");
  sam_dumpsample(priv, &g_sampleregs[SAMPLENDX_AFTER_SETUP],
                   "After setup");
  sam_dumpsample(priv, &g_sampleregs[SAMPLENDX_END_TRANSFER],
                   "End of transfer");
}
#endif

/****************************************************************************
 * Name: sam_showregs
 *
 * Description:
 *   Dump the current state of all registers
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_XFRDEBUG
static void sam_showregs(struct sam_dev_s *priv, const char *msg)
{
  struct sam_sdmmcregs_s regs;

  sam_sdmmcsample(priv, &regs);
  sam_dumpsample(priv, &regs, msg);
}
#endif

/****************************************************************************
 * Data Transfer Helpers
 ****************************************************************************/

/****************************************************************************
 * Name: sam_dataconfig
 *
 * Description:
 *   Configure the SDIO data path for the next data transfer
 *
 ****************************************************************************/

static inline void sam_dataconfig(struct sam_dev_s *priv, bool bwrite,
                             unsigned int datalen, unsigned int timeout)
{
  sam_sample(priv, SAMPLENDX_BEFORE_SETUP);
  sam_putreg8(priv, timeout & SDMMC_TCR_MASK, SAMA5_SDMMC_TCR_OFFSET);
}

/****************************************************************************
 * Name: sam_transmit
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

#ifndef CONFIG_SAMA5_SDMMC_DMA
static void sam_transmit(struct sam_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Loop while there is more data to be sent, while buffer write enable
   * (PRSSTAT.BWEN)
   */

  mcinfo("Entry: remaining: %d IRQSTAT: %08" PRIx32 "\n", priv->remaining,
         sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET));

  while (priv->remaining > 0 &&
         (sam_getreg(priv, SAMA5_SDMMC_PRSSTAT_OFFSET) &
          SDMMC_PRSSTAT_BWEN) != 0)
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

      sam_putreg(priv, data.w, SAMA5_SDMMC_DATAPORT_OFFSET);
    }

  /* Clear BWR.  If there is more data in the buffer, writing to the buffer
   * should reset BWR.
   */

  sam_putreg(priv, SDMMC_INT_BWR, SAMA5_SDMMC_IRQSTAT_OFFSET);

  mcinfo("Exit: remaining: %d IRQSTAT: %08" PRIx32 "\n", priv->remaining,
         sam_getreg16(priv, SAMA5_SDMMC_IRQSTAT_OFFSET));
}
#endif

/****************************************************************************
 * Name: sam_receive
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

#ifndef CONFIG_SAMA5_SDMMC_DMA
static void sam_receive(struct sam_dev_s *priv)
{
  union
  {
    uint32_t w;
    uint8_t  b[4];
  } data;

  /* Loop while there is space to store the data, waiting for buffer
   * read ready (BRR)
   */

  mcinfo("Entry: remaining: %d IRQSTAT: %08" PRIx32 "\n", priv->remaining,
         sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET));

  /* Clear BRR. It only fires on the first block of data. */

  sam_putreg(priv, SDMMC_INT_BRR, SAMA5_SDMMC_IRQSTAT_OFFSET);

  while (priv->remaining > 0 &&
         (sam_getreg(priv, SAMA5_SDMMC_PRSSTAT_OFFSET) &
          SDMMC_PRSSTAT_BREN) != 0)
    {
      /* Clear BREN.  If there is more data in the buffer, reading from the
       * buffer should reset BREN.
       */

      sam_putreg(priv, SDMMC_PRSSTAT_BREN, SAMA5_SDMMC_PRSSTAT_OFFSET);

      /* Read the next word from the RX buffer */

      data.w = sam_getreg32(priv, SAMA5_SDMMC_DATAPORT_OFFSET);
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

          for (i = 0; i < priv->remaining; i++)
            {
              *ptr++ = data.b[i];
            }

          /* Now the transfer is finished */

          priv->remaining = 0;
        }
    }

  mcinfo("Exit: remaining: %d IRQSTAT: %08" PRIx32 "\n", priv->remaining,
         sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET));
}
#endif

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
  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. Sample registers at the time of the timeout */

      sam_sample(priv, SAMPLENDX_END_TRANSFER);

      /* Wake up any waiting threads */

      sam_endwait(priv, SDIOWAIT_TIMEOUT);
      mcerr("ERROR: Timeout: remaining: %d\n", priv->remaining);
      sam_dumpsamples(priv);
    }
}

/****************************************************************************
 * Name: sam_endwait
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

static void sam_endwait(struct sam_dev_s *priv, sdio_eventset_t wkupevent)
{
  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  sam_configwaitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: sam_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the SDIO interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv      - An instance of the SDIO device interface
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

  sam_configxfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */

  sam_putreg(priv, SDMMC_XFRDONE_INTS | SDMMC_DMADONE_INTS,
            SAMA5_SDMMC_IRQSTAT_OFFSET);

  /* Mark the transfer finished */

  priv->remaining = 0;

#ifdef CONFIG_SAMA5_SDMMC_DMA
  /* DMA modified the buffer, so we need to flush its cache lines. */

  up_invalidate_dcache((uintptr_t) priv->buffer,
                       (uintptr_t) priv->bufferend);
#endif

  /* Debug instrumentation */

  sam_sample(priv, SAMPLENDX_END_TRANSFER);

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      sam_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: sam_interrupt
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

static int sam_interrupt(int irq, void *context, void *arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)arg;
  uint32_t enabled;
  uint32_t pending;
  uint32_t irqsigen;
  uint32_t irqstat;
#ifdef CONFIG_SAMA5_SDMMC_DMA
  uint32_t dma_start_addr;
#endif

  /* Check the SDMMC IRQSTAT register.  Mask out all bits that don't
   * correspond to enabled interrupts.  (This depends on the fact that bits
   * are ordered the same in both the IRQSTAT and IRQSIGEN registers).
   * If there are non-zero bits remaining, then we have work to do here.
   */

  irqsigen = sam_getreg(priv, SAMA5_SDMMC_IRQSIGEN_OFFSET);
  irqstat = sam_getreg32(priv, SAMA5_SDMMC_IRQSTAT_OFFSET);
  enabled = irqstat & irqsigen;
  pending = enabled & priv->xfrints;

  mcinfo("IRQSTAT: %08" PRIx32 " IRQSIGEN %08" PRIx32
         " enabled: %08" PRIx32 " pending: %08" PRIx32
         " xfrints:%08" PRIx32 "\n",
         irqstat, irqsigen, enabled, pending, priv->xfrints);

  /* Handle in progress, interrupt driven data transfers ********************/

  if (pending != 0)
    {
#ifndef CONFIG_SAMA5_SDMMC_DMA
      /* Is the RX buffer read ready? Is so then we must be processing a
       * non-DMA receive transaction.
       */

      if ((pending & SDMMC_INT_BRR) != 0)
        {
          /* Receive data from the RX buffer */

          sam_receive(priv);
        }

      /* Otherwise, Is the TX buffer write ready? If so we must be processing
       * non-DMA send transaction.  NOTE: We can't be processing both!
       */

      else if ((pending & SDMMC_INT_BWR) != 0)
        {
          /* Send data via the TX FIFO */

          sam_transmit(priv);
        }

#endif
#ifdef CONFIG_SAMA5_SDMMC_DMA
      /* SDMA Buffer Boundary pause... update the DMA System Address Register
       * to restart the transfer. See SAMA5D27 datasheet p1771.
       */

      if (((pending & SDMMC_INT_DINT) != 0) &&
          ((pending & SDMMC_INT_TC) == 0))
        {
          /* clear interrupt */

          sam_putreg(priv, SDMMC_INT_DINT, SAMA5_SDMMC_IRQSTAT_OFFSET);

          /* set SDMA start address to the next 4KB buffer */

          dma_start_addr = sam_getreg(priv, SAMA5_SDMMC_DSADDR_OFFSET);
          dma_start_addr &= ~(SDMMC_DEFAULT_BOUNDARY_SIZE - 1);
          dma_start_addr += SDMMC_DEFAULT_BOUNDARY_SIZE;
          sam_putreg(priv, dma_start_addr, SAMA5_SDMMC_DSADDR_OFFSET);
        }
#endif

      /* ... transfer complete events */

      if ((pending & SDMMC_INT_TC) != 0)
        {
          /* Terminate the transfer */

          sam_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
        }

      /* Data block send/receive CRC failure */

      else if ((pending & SDMMC_INT_DCE) != 0)
        {
          /* Terminate the transfer with an error */

          mcerr("ERROR: Data block CRC failure, remaining: %d\n",
                priv->remaining);
          sam_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
        }

      /* ... data timeout error */

      else if ((pending & SDMMC_INT_DTOE) != 0)
        {
          /* Terminate the transfer with an error */

          mcerr("ERROR: Data timeout, remaining: %d\n", priv->remaining);
          sam_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
        }
    }

  /* Handle Card interrupt events *******************************************/

  pending = enabled & priv->cintints;
  if ((pending & SDMMC_INT_CINT) != 0)
    {
      if (priv->do_sdio_card)
        {
          (priv->do_sdio_card)(priv->do_sdio_arg);
        }

      /* We don't want any more ints now, so switch it off */

      irqsigen &= ~SDMMC_INT_CINT;
      priv->cintints  = irqsigen;
      sam_putreg(priv, irqsigen,  SAMA5_SDMMC_IRQSIGEN_OFFSET);
    }

  if ((pending & SDMMC_INT_CINS) != 0 || (pending & SDMMC_INT_CRM) != 0)
    {
      if (up_interrupt_context())
        {
          /* Yes.. queue it */

          mcinfo("Queuing callback to %p(%p)\n", priv->callback,
                  priv->cbarg);
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

  /* Handle wait events *****************************************************/

  pending = enabled & priv->waitints;
  if (pending != 0)
    {
      /* Is this a response completion event? */

      if ((pending & SDMMC_RESPDONE_INTS) != 0)
        {
          /* Yes.. Is there a thread waiting for response done? */

          if ((priv->waitevents &
               (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE)) != 0)
            {
              /* Yes.. mask further interrupts and wake the thread up */

              irqsigen = sam_getreg(priv, SAMA5_SDMMC_IRQSIGEN_OFFSET);
              irqsigen &= ~SDMMC_RESPDONE_INTS;
              sam_putreg(priv, irqsigen,  SAMA5_SDMMC_IRQSIGEN_OFFSET);

              sam_endwait(priv, SDIOWAIT_RESPONSEDONE);
            }
        }
    }

  sam_dumpsamples(priv);
  return OK;
}

/****************************************************************************
 * SDIO Interface Methods
 ****************************************************************************/

/****************************************************************************
 * Name: sam_lock
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
static int sam_lock(struct sdio_dev_s *dev, bool lock)
{
  /* The multiplex bus is part of board support package. */

  sam_muxbus_sdio_lock((dev - g_sdmmcdev) /
                         sizeof(struct sam_dev_s), lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: sam_reset
 *
 * Description:
 *   Reset the SDIO controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev - An instance of the SDIO device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_reset(struct sdio_dev_s *dev)
{
  unsigned long timeout_ms;
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  /* Turn SDMMC peripheral off and then on */

  sam_sdmmc1_disableclk();
  sam_sdmmc1_enableclk();

  /* Disable all interrupts so that nothing interferes with the following. */

  sam_putreg(priv, 0,  SAMA5_SDMMC_IRQSIGEN_OFFSET);

  /* Reset the SDMMC block, putting registers in their default, reset state.
   * Initiate the reset by setting the RSTA bit in the SYSCTL register.
   */

  modifyreg32(priv->addr + SAMA5_SDMMC_SYSCTL_OFFSET, 0, SDMMC_SYSCTL_RSTA);

  /* The SDMMC will reset the RSTA bit to 0 when the capabilities registers
   * are valid and the host driver can read them.
   */

  while ((sam_getreg(priv, SAMA5_SDMMC_SYSCTL_OFFSET) &
          SDMMC_SYSCTL_RSTA) != 0)
    {
    }

  /* Make sure that all clocking is disabled */

  sam_clock(dev, CLOCK_SDIO_DISABLED);

  /* Enable all status bits (these could not all be potential sources of
   * interrupts.
   */

  sam_putreg(priv, SDMMC_INT_ALL,  SAMA5_SDMMC_IRQSTATEN_OFFSET);

  mcinfo("SYSCTL: %08" PRIx32 " PRSSTAT: %08" PRIx32
         " IRQSTATEN: %08" PRIx32 "\n",
         sam_getreg(priv, SAMA5_SDMMC_SYSCTL_OFFSET),
         sam_getreg(priv, SAMA5_SDMMC_PRSSTAT_OFFSET),
         sam_getreg(priv, SAMA5_SDMMC_IRQSTATEN_OFFSET));

  /* The next phase of the hardware reset would be to send at least 80 clock
   * ticks for card to power up and then reset the card with CMD0.  This is
   * done elsewhere.
   */

  /* Reset state data */

  priv->waitevents = 0;         /* Set of events to be waited for */
  priv->waitints   = 0;         /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;         /* The event that caused the wakeup */
#ifdef CONFIG_SAMA5_SDMMC_DMA
  priv->xfrflags   = 0;         /* Used to synchronize SDIO and DMA completion */
#endif

  wd_cancel(&priv->waitwdog);   /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;         /* Address of current R/W buffer */
  priv->remaining  = 0;         /* Number of bytes remaining in the transfer */
  priv->xfrints    = 0;         /* Interrupt enables for data transfer */

  /* SDMMC software reset - wait for a maximum of  100 ms */

  sam_putreg8(priv, SDMMC_RESET_ALL, SAMA5_SDMMC_SRR_OFFSET);
  timeout_ms = 1000;
  while (sam_getreg8(priv, SAMA5_SDMMC_SRR_OFFSET) & SDMMC_RESET_ALL)
    {
      if (timeout_ms == 0)
        {
          mcinfo("%s: Reset 0x%x never completed.\n",
                 __func__, (int)SDMMC_RESET_ALL);
          return;
        }

      timeout_ms--;
      usleep(100);
    }

    mcinfo("Reset complete\n");
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
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  switch (priv->addr)
    {
      case SAM_SDMMC0_VBASE:
#ifdef CONFIG_SAMA5_SDMMC0_WIDTH_D1_ONLY
        caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_SAMA5_SDMMC0_WIDTH_D1_D4
        caps |= SDIO_CAPS_4BIT;
#endif
        break;

      case SAM_SDMMC1_VBASE:
#ifdef CONFIG_SAMA5_SDMMC1_WIDTH_D1_ONLY
        caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_SAMA5_SDMMC1_WIDTH_D1_D4
        caps |= SDIO_CAPS_4BIT;
#endif
#ifdef CONFIG_SAMA5_SDMMC1_WIDTH_D1_D8
        caps |= SDIO_CAPS_8BIT;
#endif
        break;

      default:
        break;
    }

#ifdef CONFIG_SAMA5_SDMMC_DMA
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
 *   dev - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see sam_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t sam_status(struct sdio_dev_s *dev)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  bool present = false;

  /* Board did not use one of the GPIO_SDMMCn_CD pins
   * but instead a GPIO was used and defined in board.h
   * as PIN_SDMMCx_CD_GPIO
   */

  if (priv->sw_cd_gpio != 0)
    {
      present = priv->cd_invert ^ !sam_pioread(priv->sw_cd_gpio);
    }
  else
    {
  /* This register reflects the state of CD no matter if it's a separate pin
   * or DAT3
   */

      present = ((sam_getreg(priv, SAMA5_SDMMC_PRSSTAT_OFFSET) &
                SDMMC_PRSSTAT_CINS) != 0) ^ priv->cd_invert;
    }

  if (present)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }

  mcinfo("cdstatus=%02x\n", priv->cdstatus);

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

  /* Set the Data Transfer Width (DTW) field in the PROCTL register. */

  regval = sam_getreg(priv, SAMA5_SDMMC_PROCTL_OFFSET);
  regval &= ~SDMMC_PROCTL_DTW_MASK;
  if (wide)
    {
      regval |= SDMMC_PROCTL_DTW_4BIT;
    }
  else
    {
      regval |= SDMMC_PROCTL_DTW_1BIT;
    }

  sam_putreg(priv, regval,  SAMA5_SDMMC_PROCTL_OFFSET);
}

/****************************************************************************
 * Name: sam_frequency
 *
 * Description:
 *   Set the SD clock frequency
 *
 * Input Parameters:
 *   dev       - An instance of the SDIO device interface
 *   frequency - The frequency to use
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_SDMMC_ABSFREQ
static void sam_frequency(struct sdio_dev_s *dev, uint32_t frequency)
{
  uint32_t sdclkfs;
  uint32_t prescaled;
  uint32_t regval;
  unsigned int prescaler;
  unsigned int divisor;

  /* The SDCLK frequency is determined by
   *  (1) the frequency of the base clock that was selected as the
   *      input clock, and
   *  (2) by a prescaler and a divisor that are selected here:
   *
   *  SDCLK frequency = (base clock) / (prescaler * divisor)
   *
   * The prescaler is available only for the values: 2, 4, 8, 16, 32,
   * 64, 128, and 256.  Pick the smallest value of SDCLKFS that would
   * result in an in-range frequency. For example, if the base clock
   * frequency is 96 MHz, and the target frequency is 25 MHz, the following
   * logic will select prescaler:
   *
   *  96MHz / 2 <= 25MHz <= 96MHz / 2 /16 -- YES, prescaler == 2
   *
   * If the target frequency is 400 KHz, the following logic will
   * select prescaler:
   *
   *  96MHz / 2 <= 400KHz <= 96MHz / 2 / 16 -- NO
   *  96MHz / 4 <= 400KHz <= 96MHz / 4 / 16 -- NO
   *  96MHz / 8 <= 400KHz <= 96MHz / 8 / 16 -- NO
   *  96MHz / 16 <=400KHz <= 96MHz / 16 / 16 -- YES, prescaler == 16
   */

  if (/* frequency >= (BOARD_CORECLK_FREQ / 2) && */
       frequency <= (BOARD_CORECLK_FREQ / 2 / 16))
    {
      sdclkfs   = SDMMC_SYSCTL_SDCLKFS_DIV2;
      prescaler = 2;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 4) &&
           frequency <= (BOARD_CORECLK_FREQ / 4 / 16))
    {
      sdclkfs   = SDMMC_SYSCTL_SDCLKFS_DIV4;
      prescaler = 4;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 8) &&
           frequency <= (BOARD_CORECLK_FREQ / 8 / 16))
    {
      sdclkfs   = SDMMC_SYSCTL_SDCLKFS_DIV8;
      prescaler = 8;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 16) &&
           frequency <= (BOARD_CORECLK_FREQ / 16 / 16))
    {
      sdclkfs   = SDMMC_SYSCTL_SDCLKFS_DIV16;
      prescaler = 16;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 32) &&
           frequency <= (BOARD_CORECLK_FREQ / 32 / 16))
    {
      sdclkfs   = SDMMC_SYSCTL_SDCLKFS_DIV32;
      prescaler = 32;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 64) &&
           frequency <= (BOARD_CORECLK_FREQ / 64 / 16))
    {
      sdclkfs   = SDMMC_SYSCTL_SDCLKFS_DIV64;
      prescaler = 64;
    }
  else if (frequency >= (BOARD_CORECLK_FREQ / 128) &&
           frequency <= (BOARD_CORECLK_FREQ / 128 / 16))
    {
      sdclkfs   = SDMMC_SYSCTL_SDCLKFS_DIV128;
      prescaler = 128;
    }
  else
    {
      sdclkfs   = SDMMC_SYSCTL_SDCLKFS_DIV256;
      prescaler = 256;
    }

  /* The optimal divider can than be calculated. For example, if the base
   * clock frequency is 96 MHz, the target frequency is 25 MHz, and the
   * selected prescaler value is 2, then
   *
   *   prescaled = 96MHz / 2 = 48MHz
   *   divisor = (48MHz + 12.5MHz/ 25MHz = 2
   *
   * And the resulting frequency will be 24MHz. Or, for example, if the
   * target frequency is 400 KHz and the selected prescaler is 16, the
   * following logic will select prescaler:
   *
   *   prescaled = 96MHz / 16 = 6MHz
   *   divisor = (6MHz + 200KHz) / 400KHz = 15
   *
   * And the resulting frequency will be exactly 400KHz.
   */

  prescaled = frequency / prescaler;
  divisor   = (prescaled + (frequency >> 1)) / frequency;

  /* Set the new divisor information and enable all clocks in the SYSCTRL
   * register. TODO: Investigate using the automatically gated clocks to
   * reduce power consumption.
   */

  regval  = sam_getreg(priv, SAMA5_SDMMC_SYSCTL_OFFSET);
  regval &= ~(SDMMC_SYSCTL_SDCLKFS_MASK | SDMMC_SYSCTL_DVS_MASK);
  regval |= (sdclkfs | SDMMC_SYSCTL_DVS_DIV(divisor));
  regval |= (SDMMC_SYSCTL_SDCLKEN | SDMMC_SYSCTL_PEREN |
             SDMMC_SYSCTL_HCKEN | SDMMC_SYSCTL_IPGEN);

  sam_putreg(priv, regval,  SAMA5_SDMMC_SYSCTL_OFFSET);
  mcinfo("SYSCTRL: %08x\n", sam_getreg(priv, SAMA5_SDMMC_SYSCTL_OFFSET));
}
#endif

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
  int wait_microseconds = 0;

  /* Clear the old prescaler and divisor values so that new ones can be
   * ORed in.
   */

  regval  = sam_getreg(priv, SAMA5_SDMMC_SYSCTL_OFFSET);
  regval &= ~(SDMMC_SYSCTL_SDCLKFS_MASK | SDMMC_SYSCTL_DVS_MASK);

  /* Select the new prescaler and divisor values based on the requested
   * mode and the settings from the board.h file. Clocks are automatically
   * gated by the driver when not needed.
   */

  switch (rate)
    {
    default:
    case CLOCK_SDIO_DISABLED:      /* Clock is disabled */
      {
        /* Clear the prescaler and divisor settings */

        sam_putreg(priv, regval,  SAMA5_SDMMC_SYSCTL_OFFSET);
        mcinfo("DISABLED, SYSCTRL: %08" PRIx32 "\n",
               sam_getreg(priv, SAMA5_SDMMC_SYSCTL_OFFSET));
        sam_set_clock(priv, 0);
      }
      break;

    case CLOCK_IDMODE:
      {
        /* Initial ID mode clocking (<400KHz) */

        mcinfo("IDMODE\n");

        /* Wait for at least 74 SD Clock cycles, as per SD Card
         * specification. The e.MMC Electrical Standard specifies
         * tRSCA >= 200 usec.
         */

        regval |= (BOARD_SDMMC_IDMODE_PRESCALER |
                   BOARD_SDMMC_IDMODE_DIVISOR |
                   SDMMC_SYSCTL_INTCLKEN);
        wait_microseconds = 200;
        sam_set_clock(priv, SAMA5_SDMMC_BUS_SPEED_IDMODE);
      }
      break;

    case CLOCK_MMC_TRANSFER:
      {
        /* MMC normal operation clocking */

        mcinfo("MMCTRANSFER\n");
        regval |= (BOARD_SDMMC_MMCMODE_PRESCALER |
                   BOARD_SDMMC_MMCMODE_DIVISOR);
        sam_set_clock(priv, SAMA5_SDMMC_BUS_SPEED);
      }
      break;

    case CLOCK_SD_TRANSFER_1BIT:
      {
#ifndef CONFIG_SAM_SDMMC_WIDTH_D1_ONLY
        /* SD normal operation clocking (narrow 1-bit mode) */

        mcinfo("1BITTRANSFER\n");
        regval |= (BOARD_SDMMC_SD1MODE_PRESCALER |
                   BOARD_SDMMC_SD1MODE_DIVISOR);
        sam_set_clock(priv, SAMA5_SDMMC_BUS_SPEED);
      }
      break;
#endif

    case CLOCK_SD_TRANSFER_4BIT:
      {
        /* SD normal operation clocking (wide 4-bit mode) */

        mcinfo("4BITTRANSFER\n");
        regval |= (BOARD_SDMMC_SD4MODE_PRESCALER |
                  BOARD_SDMMC_SD4MODE_DIVISOR);
        sam_set_clock(priv, SAMA5_SDMMC_BUS_SPEED);
      }
      break;
    }

  if (wait_microseconds > 0)
    {
     usleep(wait_microseconds);
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
  int ret;
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  /* Attach the SDIO interrupt handler */

  if (priv->addr == SAM_SDMMC0_VBASE)
    {
      ret = irq_attach(SAM_IRQ_SDMMC0, sam_interrupt, &g_sdmmcdev[0]);
    }
  else
    {
      if (priv->addr == SAM_SDMMC1_VBASE)
        {
          ret = irq_attach(SAM_IRQ_SDMMC1, sam_interrupt, &g_sdmmcdev[1]);
        }
      else
        {
          ASSERT(false);
        }
    }

  if (ret == OK)
    {
      /* Disable all interrupts at the SDIO controller and clear all pending
       * interrupts.
       */

      sam_putreg(priv, 0,  SAMA5_SDMMC_IRQSIGEN_OFFSET);
      sam_putreg(priv, SDMMC_INT_ALL,  SAMA5_SDMMC_IRQSTAT_OFFSET);

      /* Enable SDIO interrupts at the NVIC.  They can now be enabled at the
       * SDIO controller as needed.
       */

      if (priv->addr == SAM_SDMMC0_VBASE)
        {
          up_enable_irq(SAM_IRQ_SDMMC0);
        }
      else if (priv->addr == SAM_SDMMC1_VBASE)
        {
          up_enable_irq(SAM_IRQ_SDMMC1);
        }
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

static int sam_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                       uint32_t arg)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  uint32_t regval;
  uint32_t cmdidx;

  /* Initialize the command index */

  cmdidx    = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval    = cmdidx << SDMMC_XFERTYP_CMDINX_SHIFT;

  if (cmdidx == SD_ACMDIDX53)
    {
      /* Dynamically set parameters for ACMD53 because it can accommodate
       * different transmission characteristics (single and multi-block,
       * rx & tx).
       */

      if (arg & (1 << 31))
        {
          /* Transmit mode */

          cmd |= MMCSD_WRDATAXFR;
        }
      else
        {
          /* Receive mode */

          cmd |= MMCSD_RDDATAXFR;
        }

      if (arg & (1 << 27))
        {
          /* In block mode */

          cmd |= SDIO_MULTIBLOCK;
        }
    }

  /* Check if a data transfer accompanies the command */

  switch (cmd & MMCSD_DATAXFR_MASK)
    {
    default:
    case MMCSD_NODATAXFR:
      {
        /* No.. no data transfer */
      }
      break;

      /* The following two cases are probably missing some setup logic */

    case MMCSD_RDSTREAM:
      {
        /* Yes.. streaming read data transfer */

        regval |= SDMMC_XFERTYP_DPSEL;
        regval |= SDMMC_XFERTYP_DTDSEL;
      }
      break;

    case MMCSD_WRSTREAM:
      {
        /* Yes.. streaming write data transfer */

       regval |= SDMMC_XFERTYP_DPSEL;
      }
      break;

    case MMCSD_RDDATAXFR:
      {
        /* Yes.. normal read data transfer */

        regval |= SDMMC_XFERTYP_DPSEL;
        regval |= SDMMC_XFERTYP_DTDSEL;
      }
      break;

    case MMCSD_WRDATAXFR:
      {
        /* Yes.. normal write data transfer */

        regval |= SDMMC_XFERTYP_DPSEL;
      }
      break;
    }

  /* Is it a multi-block transfer? */

  if ((cmd & MMCSD_MULTIBLOCK) != 0)
    {
      /* Yes.. should the transfer be stopped with ACMD12? */

      if ((cmd & MMCSD_STOPXFR) != 0)
        {
          /* Yes.. Indefinite block transfer */

          regval |= (SDMMC_XFERTYP_MSBSEL | SDMMC_XFERTYP_AC12EN);
        }
      else
        {
          /* No.. Fixed block transfer */

          regval |= (SDMMC_XFERTYP_MSBSEL | SDMMC_XFERTYP_BCEN);
        }
     }

  /* Configure response type bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      {
        /* No response */

        regval |= SDMMC_XFERTYP_RSPTYP_NONE;
      }
      break;

    case MMCSD_R1B_RESPONSE:
      {
        /* Response length 48, check busy & cmdindex */

        regval |=
          (SDMMC_XFERTYP_RSPTYP_LEN48BSY | SDMMC_XFERTYP_CICEN |
           SDMMC_XFERTYP_CCCEN);
      }
      break;

    case MMCSD_R1_RESPONSE: /* Response length 48, check cmdindex */
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      {
        regval |=
          (SDMMC_XFERTYP_RSPTYP_LEN48 | SDMMC_XFERTYP_CICEN |
           SDMMC_XFERTYP_CCCEN);
      }
      break;

    case MMCSD_R2_RESPONSE:
      {
        /* Response length 136, check CRC */

        regval |= (SDMMC_XFERTYP_RSPTYP_LEN136 | SDMMC_XFERTYP_CCCEN);
      }
      break;

    case MMCSD_R3_RESPONSE: /* Response length 48 */
    case MMCSD_R4_RESPONSE:
    case MMCSD_R7_RESPONSE:
      {
        regval |= SDMMC_XFERTYP_RSPTYP_LEN48;
      }
      break;
    }

#ifdef CONFIG_SAMA5_SDMMC_DMA
  /* Enable DMA */

  regval |= SDMMC_XFERTYP_DMAEN;

#endif

  /* Check for abort. */

  /* TODO: Check Suspend/Resume bits too in XFR_TYP::CMDTYP */

  if (cmd & MMCSD_STOPXFR)
    {
      regval |= SDMMC_XFERTYP_CMDTYP_ABORT;
    }

  mcinfo("cmd: %08" PRIx32 " arg: %08" PRIx32
         " regval: %08" PRIx32 "\n", cmd, arg, regval);

  /* If there has been a response error then perform a reset and wait for it
   * to complete.
   */

  if ((sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET) &
       SDMMC_RESPERR_INTS) != 0)
    {
      modifyreg32(priv->addr + SAMA5_SDMMC_SYSCTL_OFFSET, 0,
                  SDMMC_SYSCTL_RSTC);
      while ((sam_getreg(priv, SAMA5_SDMMC_SYSCTL_OFFSET) &
             SDMMC_SYSCTL_RSTC) != 0)
        {
        }
    }

  /* The Command Inhibit (CIHB) bit is set in the PRSSTAT bit immediately
   * after the transfer type register is written.  This bit is cleared
   * when the command response is received.  If this status bit is 0, it
   * indicates that the CMD line is not in use and the SDMMC can issue a
   * SD/MMC Command using the CMD line. CIHB should always be clear before
   * this function is called, but this check is performed here to provide
   * overlap and maximum performance.
   */

  timeout = SDMMC_CMDTIMEOUT;
  start   = clock_systime_ticks();
  while ((sam_getreg(priv, SAMA5_SDMMC_PRSSTAT_OFFSET) &
          SDMMC_PRSSTAT_CIHB) != 0)
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;
      if (elapsed >= timeout)
        {
          mcerr("ERROR: Timeout (waiting CIHB) "
                "cmd: %08" PRIx32 " PRSSTAT: %08" PRIx32 "\n",
                cmd, sam_getreg(priv, SAMA5_SDMMC_PRSSTAT_OFFSET));
          return -EBUSY;
        }
    }

  /* Set the SDMMC Argument value */

  sam_putreg(priv, arg,  SAMA5_SDMMC_CMDARG_OFFSET);

  /* Clear interrupt status and write the SDMMC CMD */

  sam_configxfrints(priv, SDMMC_RCVDONE_INTS | SDMMC_XFRDONE_INTS | \
                          SDMMC_INT_DINT);

  sam_putreg(priv, SDMMC_RESPDONE_INTS, SAMA5_SDMMC_IRQSTAT_OFFSET);
  sam_putreg(priv, regval, SAMA5_SDMMC_XFERTYP_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: sam_blocksetup
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
static void sam_blocksetup(struct sdio_dev_s *dev,
                           unsigned int blocklen,
                           unsigned int nblocks)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  mcinfo("blocklen=%d, total transfer=%d (%d blocks)\n", blocklen,
         blocklen * nblocks, nblocks);

  /* Configure block size for next transfer */

  sam_putreg16(priv, blocklen, SAMA5_SDMMC_BSR_OFFSET);
  sam_putreg16(priv, nblocks, SAMA5_SDMMC_BCR_OFFSET);
}
#endif

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
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SAMA5_SDMMC_DMA
static int sam_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                         size_t nbytes)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t) buffer & 3) == 0);

  /* Reset the DPSM configuration */

  sam_sampleinit();

  mcinfo("nbytes: %zd priv->remaining: %d IRQSTAT: %08" PRIx32 "\n", nbytes,
          priv->remaining,
          sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET));
  sam_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler and DMA memory invalidation.
   */

  priv->buffer = (uint32_t *)buffer;
  priv->remaining = nbytes;

  /* Then set up the SDIO data path */

  mcinfo("remaining: %d IRQSTAT: %08" PRIx32 "\n", priv->remaining,
         sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET));
  sam_dataconfig(priv, false, nbytes, SDMMC_DTOCV_MAXTIMEOUT);

  /* And enable interrupts */

  mcinfo("remaining: %d IRQSTAT: %08" PRIx32 "\n", priv->remaining,
         sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET));
  sam_configxfrints(priv, SDMMC_RCVDONE_INTS | SDMMC_XFRDONE_INTS);
  sam_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}
#endif

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
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

#ifndef CONFIG_SAMA5_SDMMC_DMA
static int sam_sendsetup(struct sdio_dev_s *dev,
                         const uint8_t *buffer,
                         size_t nbytes)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t) buffer & 3) == 0);

  /* Reset the DPSM configuration */

  sam_sampleinit();
  sam_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer = (uint32_t *)buffer;
  priv->remaining = nbytes;

  /* Then set up the SDIO data path */

  sam_dataconfig(priv, true, nbytes, SDMMC_DTOCV_MAXTIMEOUT);

  /* Enable TX interrupts */

  sam_configxfrints(priv, SDMMC_SNDDONE_INTS);
  sam_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}
#endif

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

#ifdef CONFIG_SAMA5_SDMMC_DMA
  uint32_t regval;
#endif

  /* Disable all transfer- and event- related interrupts */

  sam_configxfrints(priv, 0);
  sam_configwaitints(priv, 0, 0, 0);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  sam_putreg(priv, SDMMC_WAITALL_INTS,  SAMA5_SDMMC_IRQSTAT_OFFSET);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* If this was a DMA transfer, make sure that DMA is stopped */

#ifdef CONFIG_SAMA5_SDMMC_DMA

  /* Stop the DMA by resetting the data path */

  regval = sam_getreg(priv, SAMA5_SDMMC_SYSCTL_OFFSET);
  regval |= SDMMC_SYSCTL_RSTD;
  sam_putreg(priv, regval,  SAMA5_SDMMC_SYSCTL_OFFSET);
#endif

  /* Mark no transfer in progress */

  priv->remaining = 0; return OK;
}

/****************************************************************************
 * Name: sam_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.  This
 *   function should be called even after sending commands that have no
 *   response (such as CMD0) to make sure that the hardware is ready to
 *   receive the next command.
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
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  uint32_t errors;
  uint32_t enerrors;

  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  int ret = OK;

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      timeout = SDMMC_CMDTIMEOUT;
      errors  = 0;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
      {
        timeout = SDMMC_LONGTIMEOUT;
        errors  = SDMMC_RESPERR_INTS;
      }
      break;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      {
        timeout = SDMMC_CMDTIMEOUT;
        errors  = SDMMC_RESPERR_INTS;
      }
      break;

    default:
      return -EINVAL;
    }

  /* Then wait for the Command Complete (CC) indication (or timeout).  The
   * CC bit is set when the end bit of the command response is received
   * (except Auto CMD12).
   */

  start = clock_systime_ticks();
  while ((sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET) & SDMMC_INT_CC) == 0)
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;
      if (elapsed >= timeout)
        {
          mcerr("ERROR: Timeout cmd: %08" PRIx32
                " IRQSTAT: %08" PRIx32 "\n", cmd,
                sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET));
          sam_showregs(priv, "After timeout");
          ret = -ETIMEDOUT;
          break;
        }
    }

  /* Check for hardware detected errors */

  enerrors = sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET) & errors;
  if (enerrors != 0)
    {
      mcerr("ERROR: cmd: %08" PRIx32 " errors: %08" PRIx32 ", "
            "fired %08" PRIx32 " IRQSTAT: %08" PRIx32 "\n",
            cmd, errors, enerrors,
            sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET));
        ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: sam_recv_rx
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
 *   failure means only a failure to obtain the requested response (due to
 *   transport problem -- timeout, CRC, etc.).  The implementation only
 *   assures that the response is returned intact and does not check errors
 *   within the response itself.
 *
 ****************************************************************************/

static int sam_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                            uint32_t *rshort)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regval;
  int ret = OK;

  /* R1 Command response (48-bit)
   *   47    0           Start bit
   *   46    0           Transmission bit (0=from card)
   *   45:40 bit5 - bit0 Command index (0-63)
   *   39:8  bit31- bit0 32-bit card status
   *    7:1  bit6 - bit0 CRC7
   *      0  1           End bit
   *
   * R1b Identical to R1 with the additional busy signalling via the data
   *     line.
   * R6  Published RCA Response (48-bit, SD card only)
   *   47    0           Start bit
   *   46    0           Transmission bit (0=from card)
   *   45:40 bit5 - bit0 Command index (0-63)
   *   39:8 bit31 - bit0 32-bit Argument Field, consisting of:
   *                        [31:16] New published RCA of card
   *                        [15:0]  Card status bits
   *   {23,22,19,12:0}
   *   7:1 bit6 - bit0   CRC7
   *     0 1             End bit
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

      regval = sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET);
      if ((regval & SDMMC_INT_CTOE) != 0)
        {
          mcerr("ERROR: Command timeout: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & SDMMC_INT_CCE) != 0)
        {
          mcerr("ERROR: CRC failure: %08" PRIx32 "\n", regval); ret = -EIO;
        }
    }

  /* Return the R1/R1b/R6 response.  These responses are returned in CDMRSP0.
   * NOTE: This is not true for R1b (Auto CMD12 response) which is returned
   * in CMDRSP3.
   */

  *rshort = sam_getreg(priv, SAMA5_SDMMC_CMDRSP0_OFFSET);

  /* We need a short delay here to let the SDMMC peripheral respond */

  usleep(10);

  return ret;
}

static int sam_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                        uint32_t rlong[4])
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regval;
  int ret = OK;

  /* R2 CID, CSD register (136-bit)
   *  135     0             Start bit
   *  134     0             Transmission bit (0=from card)
   *  133:128 bit5 - bit0   Reserved
   *  127:1   bit127 - bit1 127-bit CID or CSD register (including int. CRC)
   *    0     1             End bit
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
    {
      /* Check if a timeout or CRC error occurred */

      regval = sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET);
      if (regval & SDMMC_INT_CTOE)
        {
          mcerr("ERROR: Timeout IRQSTAT: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & SDMMC_INT_CCE)
        {
          mcerr("ERROR: CRC fail IRQSTAT: %08" PRIx32 "\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response in CMDRSP3..0 */

  if (rlong)
    {
      uint32_t rsp3 = sam_getreg(priv, SAMA5_SDMMC_CMDRSP3_OFFSET);
      uint32_t rsp2 = sam_getreg(priv, SAMA5_SDMMC_CMDRSP2_OFFSET);
      uint32_t rsp1 = sam_getreg(priv, SAMA5_SDMMC_CMDRSP1_OFFSET);
      uint32_t rsp0 = sam_getreg(priv, SAMA5_SDMMC_CMDRSP0_OFFSET);

      rlong[0] = rsp3 << 8 | rsp2 >> 24;
      rlong[1] = rsp2 << 8 | rsp1 >> 24;
      rlong[2] = rsp1 << 8 | rsp0 >> 24; rlong[3] = rsp0 << 8;
    }

  return ret;
}

static int sam_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                         uint32_t *rshort)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t regval;
  int ret = OK;

  /* R3 OCR (48-bit)
   *    47    0           Start bit
   *    46    0           Transmission bit (0=from card)
   *    45:40 bit5 - bit0 Reserved
   *    39:8  bit31- bit0 32-bit OCR register
   *     7:1  bit6 - bit0 Reserved
   *     0    1           End bit
   */

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG_FEATURES
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R4_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%08" PRIx32 "\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout occurred (Apparently a CRC error can terminate a
       * good response)
       */

      regval = sam_getreg(priv, SAMA5_SDMMC_IRQSTAT_OFFSET);
      if (regval & SDMMC_INT_CTOE)
        {
          mcerr("ERROR: Timeout IRQSTAT: %08" PRIx32 "\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  /* Return the short response in CMDRSP0 */

  if (rshort)
    {
      *rshort = sam_getreg(priv, SAMA5_SDMMC_CMDRSP0_OFFSET);
    }

  return ret;
}

/****************************************************************************
 * Name: sam_waitenable
 *
 * Description:
 *   Enable/disable of a set of SDIO wait events.  This is part of the
 *   the SDIO_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling sam_eventwait.  This is done in this way
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

static void sam_waitenable(struct sdio_dev_s *dev,
                           sdio_eventset_t eventset, uint32_t timeout)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  uint32_t waitints;

  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  sam_configwaitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitints = 0;
  if ((eventset & (SDIOWAIT_CMDDONE | SDIOWAIT_RESPONSEDONE)) != 0)
    {
      waitints |= SDMMC_RESPDONE_INTS;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
#ifdef CONFIG_SAMA5_SDMMC_DMA
      waitints |= SDMMC_DMADONE_INTS | SDMMC_INT_DINT;
#else
      waitints |= SDMMC_XFRDONE_INTS;
#endif
    }

  /* Enable event-related interrupts */

  sam_configwaitints(priv, waitints, eventset, 0);

  /* Check if the timeout event is specified in the event set */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      int delay;
      int ret;

      /* Yes.. Handle a corner case */

      if (!timeout)
        {
          priv->wkupevent = SDIOWAIT_TIMEOUT;
          return;
        }

      /* Start the watchdog timer */

      delay = MSEC2TICK(timeout);
      ret = wd_start(&priv->waitwdog, delay,
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

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents
   * will be non-zero (and, hopefully, the semaphore count will also be
   * non-zero.
   */

  DEBUGASSERT((priv->waitevents != 0 && priv->wkupevent == 0) ||
              (priv->waitevents == 0 && priv->wkupevent != 0));

  /* Loop until the event (or the timeout occurs). Race conditions are
   * avoided by calling sam_waitenable prior to triggering the logic
   * that will cause the wait to terminate.  Under certain race
   * conditions, the waited-for may have already occurred before this
   * function was called!
   */

  for (; ; )
    {
      /* Wait for an event in event set to occur.  If this the event has
       * already occurred, then the semaphore will already have been
       * incremented and there will be no wait.
       */

      nxsem_wait_uninterruptible(&priv->waitsem);
      wkupevent = priv->wkupevent;

      /* Check if the event has occurred.  When the event has occurred, then
       * evenset will be set to 0 and wkupevent will be set to a non-zero
       * value.
       */

      if (wkupevent != 0)
        {
          /* Yes... break out of the loop with wkupevent non-zero */

          break;
        }
    }

  /* Disable event-related interrupts */

  sam_configwaitints(priv, 0, 0, 0);
#ifdef CONFIG_SAMA5_SDMMC_DMA
  priv->xfrflags = 0;
#endif
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
 *   are enabled via a call to SDIO_CALLBACKENABLE
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   callback - The function to call on the media change
 *   arg      - A caller provided value to return with the callback
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

#ifdef CONFIG_SAMA5_SDMMC_DMA
static int sam_dmarecvsetup(struct sdio_dev_s *dev,
                            uint8_t *buffer, size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t) buffer & 3) == 0);

  /* Begin sampling register values */

  sam_sampleinit();
  sam_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the destination buffer information for use by the interrupt
   * handler
   */

  priv->buffer = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->bufferend = (uint32_t *)(buffer + buflen);

  /* DMA modified the buffer, so we need to flush its cache lines. */

  up_invalidate_dcache((uintptr_t) priv->buffer,
                       (uintptr_t) priv->bufferend);

  /* Then set up the SDIO data path */

  sam_dataconfig(priv, false, buflen, SDMMC_DTOCV_MAXTIMEOUT);

  /* Turn on SDMA mode */

  uint32_t regval;
  regval = sam_getreg(priv, SAMA5_SDMMC_PROCTL_OFFSET);
  regval &= ~SDMMC_PROCTL_DMAS_MASK;
  sam_putreg(priv, regval, SAMA5_SDMMC_PROCTL_OFFSET);

  /* Configure the RX DMA */

  sam_configxfrints(priv, SDMMC_DMADONE_INTS | SDMMC_INT_DINT);
  uint32_t irqsigen;
  irqsigen = sam_getreg(priv, SAMA5_SDMMC_IRQSIGEN_OFFSET);
  sam_putreg(priv, irqsigen | SDMMC_INT_DINT, SAMA5_SDMMC_IRQSIGEN_OFFSET);

  sam_putreg(priv, (uint32_t) buffer, SAMA5_SDMMC_DSADDR_OFFSET);

  /* Sample the register state */

  sam_sample(priv, SAMPLENDX_AFTER_SETUP);
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

#ifdef CONFIG_SAMA5_SDMMC_DMA
static int sam_dmasendsetup(struct sdio_dev_s *dev,
                            const uint8_t *buffer, size_t buflen)
{
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;
  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);
  DEBUGASSERT(((uint32_t) buffer & 3) == 0);

  /* Begin sampling register values */

  sam_sampleinit();
  sam_sample(priv, SAMPLENDX_BEFORE_SETUP);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->bufferend = (uint32_t *)(buffer + buflen);

  /* DMA will read from the buffer, so we need to flush the data cache to
   * main memory.
   */

  up_flush_dcache((uintptr_t) priv->buffer, (uintptr_t) priv->bufferend);

  /* Then set up the SDIO data path */

  sam_dataconfig(priv, true, buflen, SDMMC_DTOCV_MAXTIMEOUT);

  /* Turn on SDMA mode */

  uint32_t regval;
  regval = sam_getreg(priv, SAMA5_SDMMC_PROCTL_OFFSET);
  regval &= ~SDMMC_PROCTL_DMAS_MASK;
  sam_putreg(priv, regval, SAMA5_SDMMC_PROCTL_OFFSET);

  /* Configure the TX DMA */

  sam_configxfrints(priv, SDMMC_DMADONE_INTS | SDMMC_INT_DINT);
  sam_putreg(priv, (uint32_t) buffer,  SAMA5_SDMMC_DSADDR_OFFSET);

  /* Sample the register state */

  sam_sample(priv, SAMPLENDX_AFTER_SETUP);
  return OK;
}
#endif

/****************************************************************************
 * Name: sam_callback
 *
 *  Description:
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

  /* Is a callback registered? */

  DEBUGASSERT(priv != NULL);
  mcinfo("Callback %p(%p) cbevents: %02x cdstatus: %02x\n", priv->callback,
         priv->cbarg, priv->cbevents, priv->cdstatus);

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
       * handler. If we are in an interrupt handler, then queue the callback
       * to be performed later on the work thread.
       */

      if (up_interrupt_context())
        {
          /* Yes.. queue it */

          mcinfo("Queuing callback to %p(%p)\n", priv->callback,
                  priv->cbarg);
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
 * Name: sam_set_uhs_timing
 *
 * Description:
 *   Configures the SDMMC UHS mode, needed for speeds above 25MHz.
 *   See the SAMA5D27 datasheet's SDMMC chapter for more info.
 *
 * Input Parameters:
 *   selected_mode
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_set_uhs_timing(struct sam_dev_s *priv,
                        enum bus_mode selected_mode)
{
    uint16_t reg;

    reg = sam_getreg16(priv, SAMA5_SDMMC_H2CR_OFFSET);
    reg &= ~SDMMC_H2CR_UHS_MASK;

    switch (selected_mode)
      {
        case UHS_SDR50:
        case MMC_HS_52:
            reg |= SDMMC_H2CR_UHS_SDR50;
            break;
        case UHS_DDR50:
        case MMC_DDR_52:
            reg |= SDMMC_H2CR_UHS_DDR50;
            break;
        case UHS_SDR104:
        case MMC_HS_200:
            reg |= SDMMC_H2CR_UHS_SDR104;
            break;
        default:
            reg |= SDMMC_H2CR_UHS_SDR12;
      }

      sam_putreg16(priv, reg, SAMA5_SDMMC_H2CR_OFFSET);
}

/****************************************************************************
 * Name: sam_set_clock
 *
 * Description:
 *   Called by sam_clock() to set up the SDMMC to match the capabilities
 *   of the SD Card.
 *
 * Input Parameters:
 *   clock - clock frequency in Hertz
 *
 * Returned Value:
 *   Error code
 *
 ****************************************************************************/

static int sam_set_clock(struct sam_dev_s *priv, uint32_t clock)
{
  uint16_t timeout;
  uint16_t div;
  uint16_t clk = 0;
  uint32_t clk_mul = 0;
  uint32_t max_clk = 0;
  uint32_t caps0;
  uint32_t caps1;
  uint32_t proctl;

  /* Wait max 20 ms */

  timeout = 200;
  while (sam_getreg(priv, SAMA5_SDMMC_PRSSTAT_OFFSET) &
         (SDMMC_PRSSTAT_CIHB | SDMMC_PRSSTAT_CDIHB))
    {
      if (timeout == 0)
        {
          mcinfo("%s: Timeout to wait cmd & data inhibit\n", __func__);
          return -EBUSY;
        }

        timeout--;
        usleep(100);
    }

  sam_putreg16(priv, 0, SAMA5_SDMMC_SYSCTL_OFFSET);
  if (clock == 0)
    {
      return 0;
    }

  /* Is this clock multiplier supported? */

  uint16_t version = sam_getreg16(priv, SAMA5_SDMMC_HOST_VERSION_OFFSET);

  caps0 = sam_getreg(priv, SAMA5_SDMMC_HTCAPBLT0_OFFSET);

  if (version >= SDMMC_SPEC_3)
    {
      caps1 = sam_getreg(priv, SAMA5_SDMMC_HTCAPBLT1_OFFSET);
      clk_mul = (caps1 & SDMMC_CLOCK_MUL_MASK) >> SDMMC_CLOCK_MUL_SHIFT;
    }

  if (version >= SDMMC_SPEC_3)
    {
      max_clk = (caps0 & SDMMC_CLOCK_V3_BASE_MASK) >> SDMMC_CLOCK_BASE_SHIFT;
    }
  else
    {
      max_clk = (caps0 & SDMMC_CLOCK_BASE_MASK) >> SDMMC_CLOCK_BASE_SHIFT;
    }

  max_clk *= 1000000;
  if (clk_mul)
    {
      max_clk *= clk_mul;
    }

      if (version >= SDMMC_SPEC_3)
        {
          /* Check if the SDMMC supports Programmable Clock Mode. */

          if (clk_mul)
            {
              for (div = 1; div <= 1024; div++)
                {
                  if ((max_clk / div) <= clock)
                    {
                      break;
                    }
                }

              /* Set Programmable Clock Mode */

              clk = SDMMC_SYSCTL_CLKGSEL;
              div--;
            }
          else
            {
              /* Version 3 divisors must be a multiple of 2. */

              if (max_clk <= clock)
                {
                  div = 1;
                }
              else
                {
                  for (div = 2; div < SDMMC_MAX_DIV_SPEC_3; div += 2)
                    {
                      if ((max_clk / div) <= clock)
                      break;
                    }
                }

              div >>= 1;
            }
        }

      else
        {
          /* Version 2 divisors must be a power of 2. */

          for (div = 1; div < SDMMC_MAX_DIV_SPEC_2; div *= 2)
            {
              if ((max_clk / div) <= clock)
                {
                  break;
                }
            }

            div >>= 1;
        }

      clk |= (div & SDMMC_DIV_MASK) << SDMMC_DIVIDER_SHIFT;
      clk |= ((div & SDMMC_DIV_HI_MASK) >> SDMMC_DIV_MASK_LEN)
               << SDMMC_DIVIDER_HI_SHIFT;
      clk |= SDMMC_SYSCTL_INTCLKEN;
      sam_putreg16(priv, clk, SAMA5_SDMMC_SYSCTL_OFFSET);

      /* Wait max 20 ms */

      timeout = 200;
      while (!((clk = sam_getreg16(priv, SAMA5_SDMMC_SYSCTL_OFFSET))
           & SDMMC_SYSCTL_INTCLKS))
        {
          if (timeout == 0)
            {
              mcinfo("%s: Internal clock never stabilised.\n", __func__);
              return -EBUSY;
            }

          timeout--;
          usleep(100);
        }

      /* High Speed Mode? */

      proctl = sam_getreg(priv, SAMA5_SDMMC_PROCTL_OFFSET);
      if (clock > SAMA5_SDMMC_BUS_HIGH_SPEED_THRESHOLD)
        {
           proctl |= SDMMC_PROCTL_HSEN;
        }
      else
        {
          proctl &= ~SDMMC_PROCTL_HSEN;
        }

      sam_putreg(priv, proctl, SAMA5_SDMMC_PROCTL_OFFSET);

      clk |= SDMMC_SYSCTL_SDCLKEN;
      sam_putreg16(priv, clk, SAMA5_SDMMC_SYSCTL_OFFSET);
      return 0;
}

/****************************************************************************
 * Name: sam_power
 *
 * Description:
 *   Called by sam_sdmmc_sdio_initialize() to set up the SDMMC power handling
 *   to send the correct power to the SD Card.
 *
 * Returned Value:
 *   Error code
 *
 ****************************************************************************/

static void sam_power(struct sam_dev_s *priv)
{
  uint8_t power = 0;
  uint8_t card_power = 0;

  uint32_t voltages = 0;
  uint32_t capabilities = sam_getreg(priv,
                                     SAMA5_SDMMC_HTCAPBLT0_OFFSET);

  if (capabilities & SDMMC_HTCAPBLT_VS33)
    {
      voltages |= MMCSD_VDD_32_33 | MMCSD_VDD_33_34;
    }

  if (capabilities & SDMMC_HTCAPBLT_VS30)
    {
      voltages |= MMCSD_VDD_29_30 | MMCSD_VDD_30_31;
    }

  if (capabilities & SDMMC_HTCAPBLT_VS18)
    {
      voltages |= MMCSD_VDD_19_20;
    }

  power = fls(voltages) - 1;

  uint32_t caps0 = sam_getreg(priv, SAMA5_SDMMC_HTCAPBLT0_OFFSET);
  if ((voltages & MMCSD_VDD_19_20) && (caps0 & SDMMC_HTCAPBLT_DDR50))
    {
      /* if DDR50 mode is available, set voltage to 1.8V
       * and configure the SDMMC to use DDR50 mode.
       */

       mcinfo("1.8V and DDR50 available.\n");
       card_power = SDMMC_POWER_180;
    }
  else
    {
      switch (1 << power)
        {
           case MMCSD_VDD_19_20:
             card_power = SDMMC_POWER_180;
             break;
           case MMCSD_VDD_29_30:
           case MMCSD_VDD_30_31:
             card_power = SDMMC_POWER_300;
             break;
           case MMCSD_VDD_32_33:
           case MMCSD_VDD_33_34:
             card_power = SDMMC_POWER_330;
           break;
        }
    }

  if (card_power == 0)
    {
      sam_putreg8(priv, 0, SAMA5_SDMMC_PWRCTL_OFFSET);
          return;
    }

  card_power |= SDMMC_POWER_ON;
  sam_putreg8(priv, card_power, SAMA5_SDMMC_PWRCTL_OFFSET);

  if (caps0 & SDMMC_HTCAPBLT_DDR50)
    {
      sam_set_uhs_timing(priv, UHS_DDR50); /* Double data rate, 50Mhz */
    }
  else if (caps0 & SDMMC_HTCAPBLT_SDR50)
    {
      sam_set_uhs_timing(priv, UHS_SDR50);  /* Single data rate, 50Mhz */
    }
  else if (caps0 & SDMMC_HTCAPBLT_SDR104)
    {
      sam_set_uhs_timing(priv, UHS_SDR104); /* Single data rate, 100Mhz */
    }
}

/****************************************************************************
 * Name: sam_set_interrupts
 *
 * Description:
 *   Called by sam_sdmmc_sdio_initialize() to set up the SDMMC initial
 *   interrupts.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int sam_set_interrupts(struct sam_dev_s *priv)
{
  /* Only enable interrupts used by the SDMMC */

  sam_putreg(priv, SDMMC_INT_DATA_MASK | SDMMC_INT_CMD_MASK,
             SAMA5_SDMMC_IRQSTATEN_OFFSET);

  /* Mask all SDMMC interrupt sources */

  sam_putreg(priv, 0x0, SAMA5_SDMMC_IRQSIGEN_OFFSET);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sdmmc_sdio_initialize
 *
 * Description:
 *   Initialize SDIO for operation.
 *
 * Input Parameters:
 *   slotno - Slot to be used
 *
 * Returned Value:
 *   A reference to an SDIO interface structure.
 *   NULL is returned on failures.
 *
 ****************************************************************************/

struct sdio_dev_s *sam_sdmmc_sdio_initialize(int slotno)
{
  mcinfo("slotno: %d\n", slotno);
  DEBUGASSERT(slotno < SAM_MAX_SDMMC_DEV_SLOTS);

  struct sam_dev_s *priv = &g_sdmmcdev[slotno];

  /* Initialize the SDMMC slot structure data structure */

  switch (priv->addr)
    {
    case SAM_SDMMC0_VBASE:
      priv->base  = SAM_SDMMC0_VBASE;
      /* Configure pins for 1-bit, 4-bit, or 8-bit wide-bus operation.  If
       * bus is multiplexed then there is a custom bus configuration utility
       * in the scope of the board support package.
       */

#ifndef CONFIG_SDIO_MUXBUS

#  if defined(CONFIG_SAMA5_SDMMC0)
      /* Clocking and CMD pins (all data widths) */

      sam_configpio(PIO_SDMMC0_DAT0);
      sam_configpio(PIO_SDMMC0_CK);
      sam_configpio(PIO_SDMMC0_CMD);

#    if (defined(CONFIG_SAMA5_SDMMC0_WIDTH_D1_D4) || \
        defined(CONFIG_SAMA5_SDMMC0_WIDTH_D1_D8))
      sam_configpio(PIO_SDMMC0_DAT1);
      sam_configpio(PIO_SDMMC0_DAT2);
      sam_configpio(PIO_SDMMC0_DAT3);
#    endif

#    if defined(CONFIG_SAMA5_SDMMC0_WIDTH_D1_D8)
      sam_configpio(PIO_SDMMC0_DAT4);
      sam_configpio(PIO_SDMMC0_DAT5);
      sam_configpio(PIO_SDMMC0_DAT6);
      sam_configpio(PIO_SDMMC0_DAT7);
#    endif

#    if defined(CONFIG_MMCSD_HAVE_CARDDETECT)
#      if defined(PIO_SDMMC0_CD)
      sam_configpio(PIO_SDMMC0_CD);
#      else
      if (priv->sw_cd_gpio != 0)
        {
          sam_configpio(priv->sw_cd_gpio);
        }
#      endif
#    endif

      sam_sdmmc0_enableclk();
      break;
#  endif

    case SAM_SDMMC1_VBASE:

      /* Clocking and CMD pins (all data widths) */

      priv->base  = SAM_SDMMC1_VBASE;
#  if defined(CONFIG_SAMA5_SDMMC1)
      sam_configpio(PIO_SDMMC1_DAT0);
      sam_configpio(PIO_SDMMC1_CK);
      sam_configpio(PIO_SDMMC1_CMD);

#    if defined(CONFIG_SAMA5_SDMMC1_WIDTH_D1_D4)
      sam_configpio(PIO_SDMMC1_DAT1);
      sam_configpio(PIO_SDMMC1_DAT2);
      sam_configpio(PIO_SDMMC1_DAT3);
#    endif

#    if defined(CONFIG_MMCSD_HAVE_CARDDETECT)
#      if defined(PIO_SDMMC1_CD)
      sam_configpio(PIO_SDMMC1_CD);
#      else
      if (priv->sw_cd_gpio != 0)
        {
          sam_configpio(priv->sw_cd_gpio);
        }
#      endif
#    endif
#  endif

      sam_sdmmc1_enableclk();
      break;

#endif

    default:
      return NULL;
    }

  sam_reset(&g_sdmmcdev[slotno].dev);
  sam_clock(&g_sdmmcdev[slotno].dev, CLOCK_SDIO_DISABLED);
  sam_power(priv);
  sam_set_interrupts(priv);

  return &g_sdmmcdev[slotno].dev;
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

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: sam_sdmmc_set_sdio_card_isr
 *
 * Description:
 *   SDIO card generates interrupt via SDIO_DATA_1 pin.
 *   Called by board-specific logic to register an ISR for SDIO card.
 *
 * Input Parameters:
 *   func  - callback function.
 *   arg   - arg to be passed to the function.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_sdmmc_set_sdio_card_isr(struct sdio_dev_s *dev,
                                 int (*func)(void *), void *arg)
{
  irqstate_t flags;
  uint16_t regval;
  struct sam_dev_s *priv = (struct sam_dev_s *)dev;

  priv->do_sdio_card = func;
  priv->do_sdio_arg = arg;

  if (priv->do_sdio_card != NULL)
    {
      priv->cintints = SDMMC_INT_CINT;
    }
  else
    {
      priv->cintints = 0;
    }

#if defined(CONFIG_MMCSD_HAVE_CARDDETECT)
  if (priv->sw_cd_gpio == 0)
    {
       priv->cintints |= SDMMC_INT_CINS | SDMMC_INT_CRM;
    }
#endif

  flags  = enter_critical_section();
  regval = sam_getreg16(priv, SAMA5_SDMMC_IRQSIGEN_OFFSET);
  regval = (regval & ~SDMMC_INT_CINT) | priv->cintints;
  sam_putreg16(priv, regval, SAMA5_SDMMC_IRQSIGEN_OFFSET);
  leave_critical_section(flags);
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

#endif /* CONFIG_SAMA5_SDMMC */
